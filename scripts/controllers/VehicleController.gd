extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulator
## Implements throttle, brake, steering inputs, wheel forces, and gear shifting logic
## Uses PhysicsSettings for centralized tuning and configuration
## Copyright 2026 Thalamus Racing Simulator Project

# Signals for vehicle events
signal engine_rpm_changed(rpm: float)
signal gear_changed(old_gear: int, new_gear: int)
signal vehicle_speed_changed(speed: float)
signal drift_started()
signal drift_ended()
signal collision_detected(collision_data: Dictionary)

# Enums for vehicle states
enum Gear {
	NEUTRAL = 0,
	FIRST = 1,
	SECOND = 2,
	THIRD = 3,
	FOURTH = 4,
	FIFTH = 5,
	SIXTH = 6,
	REVERSE = -1
}

enum DriftState {
	NORMAL,
	DRIFFING,
	BRAKE_DRIFT,
	OVERTHROTTLE_DRIFT
}

# Instance references
@onready var powertrain: Node = $Powertrain if has_node("Powertrain") else null
@onready var chassis: MeshInstance3D = $Chassis if has_node("Chassis") else null
@onready var wheels: Array[Node] = []
@onready var wheel_nodes: Array[Node3D] = []

# Physics properties (from PhysicsSettings)
var _mass: float = 1500.0
var _max_traction_force: float = 15000.0
var _max_steering_angle: float = 30.0 * TAU / 360.0
var _steering_speed: float = 2.5

# Current state tracking
var current_gear: Gear = Gear.NEUTRAL
var rpm: float = 0.0
var target_rpm: float = 0.0
var speed_kmh: float = 0.0
var speed_mps: float = 0.0
var drift_state: DriftState = DriftState.NORMAL
var drift_intensity: float = 0.0

# Input values (normalized -1 to 1)
var throttle_input: float = 0.0
var brake_input: float = 0.0
var steering_input: float = 0.0

# Transmission characteristics
var gear_ratios: Dictionary = {
	Gear.FIRST: 3.8,
	Gear.SECOND: 2.1,
	Gear.THIRD: 1.4,
	Gear.FOURTH: 1.0,
	Gear.FIFTH: 0.8,
	Gear.SIXTH: 0.65,
	Gear.REVERSE: 3.5
}

var final_drive_ratio: float = 3.73
var tire_radius: float = 0.33

# Engine characteristics
var idle_rpm: float = 800.0
var redline_rpm: float = 7000.0
var peak_power_rpm: float = 5500.0
var max_engine_torque: float = 450.0

# Steering state
var current_steering_angle: float = 0.0
var steering_target: float = 0.0

# Wheel configurations (front left, front right, rear left, rear right)
const WHEEL_POSITIONS: Array[Vector3] = [
	Vector3(-0.85, 0.33, 1.4),   # Front Left
	Vector3(0.85, 0.33, 1.4),    # Front Right
	Vector3(-0.85, 0.33, -1.2),  # Rear Left
	Vector3(0.85, 0.33, -1.2)    # Rear Right
]

func _ready() -> void:
	_init_vehicle()
	_connect_signals()
	_setup_wheels()
	
func _init_vehicle() -> void:
	"""Initialize vehicle physics based on PhysicsSettings"""
	_mass = PhysicsSettings.default_vehicle_mass
	
	# Load engine characteristics
	if powertrain:
		max_engine_torque = powertrain.get("max_torque", max_engine_torque)
		peak_power_rpm = powertrain.get("peak_power_rpm", peak_power_rpm)
		
	_update_velocity_from_transform()
	
func _connect_signals() -> void:
	"""Connect internal signals for coordination"""
	if powertrain:
		powertrain.rpm_changed.connect(_on_powertrain_rpm_changed)
		powertrain.torque_change.connect(_on_powertrain_torque_change)
		
	InputManager.input_changed.connect(_on_input_changed)
	
func _setup_wheels() -> void:
	"""Setup wheel nodes for visual and physics representation"""
	wheels.clear()
	wheel_nodes.clear()
	
	for i in range(4):
		var wheel_node = Node3D.new()
		wheel_node.global_position = global_position + WHEEL_POSITIONS[i]
		wheel_node.position.y -= WHEEL_POSITIONS[i].y  # Offset to ground
		
		add_child(wheel_node)
		wheel_nodes.append(wheel_node)
		
		var wheel_mesh = MeshInstance3D.new()
		var cylinder_mesh = CylinderMesh.new()
		cylinder_mesh.radius = tire_radius
		cylinder_mesh.height = tire_radius * 0.5
		cylinder_mesh.sections = 16
		
		wheel_mesh.mesh = cylinder_mesh
		wheel_mesh.scale = Vector3(1, 0.5, 1)
		
		wheel_node.add_child(wheel_mesh)
		wheels.append(wheel_node)
		
func _physics_process(delta: float) -> void:
	"""Main physics update loop"""
	_handle_inputs(delta)
	_handle_transmission(delta)
	_handle_steering(delta)
	_apply_forces(delta)
	_handle_drift(delta)
	_update_visuals(delta)
	_update_velocity()
	
func _handle_inputs(delta: float) -> void:
	"""Process input from InputManager and clamp values"""
	throttle_input = clamp(throttle_input, 0.0, 1.0)
	brake_input = clamp(brake_input, 0.0, 1.0)
	steering_input = clamp(steering_input, -1.0, 1.0)
	
	# Handle clutch logic (when shifting gears)
	if current_gear != Gear.NEUTRAL and abs(rpm - idle_rpm) < 200:
		# Auto-blip during upshift
		pass
		
func _handle_transmission(delta: float) -> void:
	"""Handle gear shifting logic and RPM calculations"""
	var gear_ratio = gear_ratios[current_gear] if current_gear != Gear.NEUTRAL else 1.0
	var total_ratio = gear_ratio * final_drive_ratio
	
	# Calculate engine torque based on RPM curve
	var torque_factor = _calculate_torque_curve(rpm)
	var engine_torque = max_engine_torque * torque_factor
	
	# Apply torque to wheels (simplified drivetrain loss)
	var drivetrain_efficiency: float = 0.85
	var wheel_torque = engine_torque * total_ratio * drivetrain_efficiency
	
	# Calculate maximum traction force based on current conditions
	var max_traction = _mass * 9.81 * 1.2  # 1.2g grip coefficient
	
	# Limit wheel force by traction
	var applied_force = min(wheel_torque / (tire_radius * total_ratio), max_traction)
	
	# Apply forward/reverse force based on gear
	var direction: float = 1.0
	if current_gear == Gear.REVERSE:
		direction = -1.0
		
	if current_gear != Gear.NEUTRAL:
		force += Vector3(direction * applied_force * throttle_input, 0, 0)
	
	# Brake force
	if brake_input > 0:
		var brake_force = _mass * 9.81 * brake_input * 2.0  # Strong braking
		force += Vector3(-brake_force, 0, 0)
		
	# Update RPM based on speed and gear
	var wheel_rps = speed_mps / (2 * PI * tire_radius)
	target_rpm = wheel_rps * total_ratio * 60  # Convert to RPM
	
	# Smooth RPM transition
	rpm = lerp(rpm, target_rpm, delta * 10.0)
	rpm = clamp(rpm, idle_rpm, redline_rpm)
	
	# Check for automatic upshift/downshift
	_auto_shift_gears(delta)
	
func _calculate_torque_curve(engine_rpm: float) -> float:
	"""Calculate torque multiplier based on RPM position"""
	if engine_rpm <= idle_rpm:
		return 0.3  # Idle torque
	elif engine_rpm >= redline_rpm:
		return 0.2  # Redline torque drop
		
	# Peak torque around 4000-5000 RPM
	var normalized = (engine_rpm - idle_rpm) / (redline_rpm - idle_rpm)
	
	if normalized < 0.4:
		return 0.6 + (normalized * 0.5)  # Rising torque
	elif normalized < 0.75:
		return 1.0  # Peak torque plateau
	else:
		return 1.0 - ((normalized - 0.75) * 0.8)  # Falling off after peak
		
func _auto_shift_gears(delta: float) -> void:
	"""Automatic gear shifting based on RPM thresholds"""
	if current_gear == Gear.NEUTRAL or current_gear == Gear.REVERSE:
		return
		
	var shift_threshold_up: float = redline_rpm * 0.95
	var shift_threshold_down: float = idle_rpm * 1.5
	
	# Upshift when approaching redline
	if rpm > shift_threshold_up and current_gear < Gear.SIXTH:
		_shift_to_gear(current_gear + 1)
		
	# Downshift when RPM drops too low
	elif rpm < shift_threshold_down and current_gear > Gear.FIRST:
		_shift_to_gear(current_gear - 1)
		
func _shift_to_gear(new_gear: Gear) -> void:
	"""Execute gear shift with proper sequencing"""
	if new_gear == current_gear:
		return
		
	var old_gear = current_gear
	current_gear = new_gear
	
	gear_changed.emit(old_gear, new_gear)
	
	# Brief neutral period for smooth shift
	await get_tree().create_timer(0.15).timeout
	# Gear already set, just emit signal
	
func _handle_steering(delta: float) -> void:
	"""Handle steering angle interpolation and limits"""
	steering_target = steering_input * _max_steering_angle
	current_steering_angle = lerp(current_steering_angle, steering_target, delta * steering_speed)
	
	# Apply steering rotation to front wheels visually
	if wheel_nodes.size() >= 2:
		wheel_nodes[0].rotate_y(current_steering_angle)
		wheel_nodes[1].rotate_y(current_steering_angle)
		
func _apply_forces(delta: float) -> void:
	"""Apply calculated forces to vehicle body"""
	# Gravity is handled by Godot physics engine
	# Add drag/resistance
	var air_resistance: float = 0.5 * 1.2 * 0.3 * speed_mps * speed_mps
	force -= velocity.normalized() * air_resistance
	
	# Rolling resistance
	var rolling_resistance: float = _mass * 9.81 * 0.015
	force -= velocity.normalized() * rolling_resistance
	
	# Apply force to character body
	velocity += force * delta / _mass
	force = Vector3.ZERO
	
func _handle_drift(delta: float) -> void:
	"""Handle drifting mechanics and intensity calculation"""
	var lateral_acceleration = _calculate_lateral_acceleration()
	var drift_threshold: float = 3.0  # m/s^2
	
	# Determine drift state
	if abs(lateral_acceleration) > drift_threshold and abs(steering_input) > 0.3:
		if brake_input > 0.5:
			drift_state = DriftState.BRAKE_DRIFT
		elif throttle_input > 0.7:
			drift_state = DriftState.OVERTHROTTLE_DRIFT
		else:
			drift_state = DriftState.DRIFFING
			
		drift_intensity = min(abs(lateral_acceleration) / drift_threshold, 1.0)
		
		if drift_state != DriftState.NORMAL:
			_on_drift_start()
			
	else:
		drift_state = DriftState.NORMAL
		drift_intensity = 0.0
		_on_drift_end()
		
func _calculate_lateral_acceleration() -> float:
	"""Calculate lateral acceleration for drift detection"""
	var horizontal_velocity = velocity.xz.length()
	var forward_direction = transform.basis.z
	
	# Project velocity onto lateral axis
	var lateral_component = velocity.xz.dot(Vector2(-forward_direction.z, forward_direction.x))
	
	return abs(lateral_component) / 1.0  # Normalize
	
func _on_drift_start() -> void:
	"""Trigger drift start events"""
	if drift_state != DriftState.NORMAL:
		if drift_state != DriftState.DRIFFING and drift_intensity > 0.1:
			drift_started.emit()
			
func _on_drift_end() -> void:
	"""Trigger drift end events"""
	if drift_state == DriftState.NORMAL:
		drift_ended.emit()
		
func _update_velocity() -> void:
	"""Update velocity properties from physics engine"""
	speed_mps = velocity.length()
	speed_kmh = speed_mps * 3.6
	
	vehicle_speed_changed.emit(speed_kmh)
	
func _update_velocity_from_transform() -> void:
	"""Extract velocity from global transform (for initialization)"""
	var prev_transform = global_transform
	await get_tree().process_frame
	
	velocity = (global_transform.origin - prev_transform.origin) / get_physics_process_delta_time()
	
func _update_visuals(delta: float) -> void:
	"""Update visual components based on vehicle state"""
	# Update wheel rotations based on speed
	for i in range(wheel_nodes.size()):
		var wheel_rotation: float = speed_mps / tire_radius * delta
		wheel_nodes[i].rotate_z(wheel_rotation)
		
	# Update chassis tilt based on acceleration/braking
	if chassis:
		var pitch_target: float = -throttle_input * 0.05 + brake_input * 0.05
		chassis.rotation.x = lerp(chassis.rotation.x, pitch_target, delta * 5.0)
		
	# Update camera follow offset
	if GameManager.current_state == GameManager.GameState.RACE_ACTIVE:
		_update_camera_offset()
		
func _update_camera_offset() -> void:
	"""Update camera follow parameters"""
	# Camera would be handled by GameManager or separate camera controller
	pass
		
func _on_powertrain_rpm_changed(new_rpm: float) -> void:
	"""Handle powertrain RPM changes"""
	rpm = new_rpm
	engine_rpm_changed.emit(rpm)
	
func _on_powertrain_torque_change(torque: float) -> void:
	"""Handle powertrain torque changes"""
	max_engine_torque = torque
	
func _on_input_changed(input_data: Dictionary) -> void:
	"""Handle input changes from InputManager"""
	if input_data.has("throttle"):
		throttle_input = input_data["throttle"]
	if input_data.has("brake"):
		brake_input = input_data["brake"]
	if input_data.has("steering"):
		steering_input = input_data["steering"]
	if input_data.has("gear"):
		var gear_value = input_data["gear"]
		if gear_value is int:
			_manual_shift_gear(gear_value)
			
func _manual_shift_gear(gear_value: int) -> void:
	"""Manual gear shifting from input"""
	match gear_value:
		0:
			_shift_to_gear(Gear.NEUTRAL)
		1:
			_shift_to_gear(Gear.FIRST)
		2:
			_shift_to_gear(Gear.SECOND)
		3:
			_shift_to_gear(Gear.THIRD)
		4:
			_shift_to_gear(Gear.FOURTH)
		5:
			_shift_to_gear(Gear.FIFTH)
		6:
			_shift_to_gear(Gear.SIXTH)
		-1:
			_shift_to_gear(Gear.REVERSE)
			
func _get_current_gear_name() -> String:
	"""Get human-readable gear name"""
	match current_gear:
		Gear.NEUTRAL:
			return "N"
		Gear.FIRST:
			return "1"
		Gear.SECOND:
			return "2"
		Gear.THIRD:
			return "3"
		Gear.FOURTH:
			return "4"
		Gear.FIFTH:
			return "5"
		Gear.SIXTH:
			return "6"
		Gear.REVERSE:
			return "R"
		_:
			return "?"
			
func reset_vehicle() -> void:
	"""Reset vehicle to initial state"""
	velocity = Vector3.ZERO
	force = Vector3.ZERO
	rpm = idle_rpm
	current_gear = Gear.NEUTRAL
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	drift_state = DriftState.NORMAL
	drift_intensity = 0.0
	global_position = Vector3.ZERO
	global_rotation = Vector3.ZERO
		
func apply_collision(collision_info: Dictionary) -> void:
	"""Handle collision event"""
	collision_detected.emit(collision_info)
	
	# Apply impact force based on collision data
	var impact_force: Vector3 = collision_info.get("force", Vector3.ZERO)
	force += impact_force
	
	# Screen shake effect (would be handled by AudioManager or VFX system)
	AudioManager.play_sound("collision_impact")
	
func get_vehicle_status() -> Dictionary:
	"""Get comprehensive vehicle status for debugging/UI"""
	return {
		"gear": current_gear,
		"gear_name": _get_current_gear_name(),
		"rpm": rpm,
		"target_rpm": target_rpm,
		"speed_kmh": speed_kmh,
		"speed_mps": speed_mps,
		"throttle": throttle_input,
		"brake": brake_input,
		"steering": steering_input,
		"drift_state": drift_state,
		"drift_intensity": drift_intensity,
		"mass": _mass
	}