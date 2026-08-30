extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulation
## Handles throttle, brake, steering, gear shifting, wheel forces, and vehicle dynamics
## Integrates with PhysicsSettings singleton for all physics constants
## Copyright 2026 Thalamus Racing Simulator Project

# ============================================================================
# SIGNALS - Events emitted by this controller
# ============================================================================
signal speed_changed(new_speed: float)
signal rpm_changed(new_rpm: float)
signal gear_changed(old_gear: int, new_gear: int)
signal vehicle_moved(position: Vector3, velocity: Vector3)
signal collision_detected(collision_data: Dictionary)
signal engine_started()
signal engine_stopped()
signal handbrake_toggled(is_active: bool)
signal drift_started(angle: float)
signal drift_ended()

# ============================================================================
# DRIVETRAIN TYPES - Enum for different drivetrain configurations
# ============================================================================
enum DrivetrainType {
	FWD,      # Front-Wheel Drive
	RWD,      # Rear-Wheel Drive
	AWD,      # All-Wheel Drive (Fixed split)
	AWD_VARYING # All-Wheel Drive (Variable torque distribution)
}

# ============================================================================
# CONSTANTS & CONFIGURATION - References to PhysicsSettings
# ============================================================================
const VEHICLE_BASE_MASS := 1500.0  # Base mass in kg
const MAX_GEAR_COUNT := 7
const NEUTRAL_GEAR := 0
const REVERSE_GEAR := -1
const GEAR_RATIO_NEUTRAL := 0.0

# ============================================================================
# PUBLIC PROPERTIES - Exposed for inspector and external access
# ============================================================================
@export_group("Vehicle Configuration")
@export var vehicle_mass: float = 1500.0: set = _set_vehicle_mass
@export var max_speed_kmh: float = 320.0: set = _set_max_speed_kmh
@export var acceleration_force: float = 8000.0: set = _set_acceleration_force
@export var braking_force: float = 15000.0: set = _set_braking_force
@export var steering_angle_max: float = 45.0: set = _set_steering_angle_max

@export_group("Drivetrain Settings")
@export var drivetrain_type: DrivetrainType = DrivetrainType.FWD
@export var final_drive_ratio: float = 3.5: set = _set_final_drive_ratio
@export var tire_radius: float = 0.33: set = _set_tire_radius
@export var torque_curve: Array[Vector2] = [
	Vector2(0.0, 0.0),   # RPM fraction -> Torque multiplier
	Vector2(0.2, 0.75),  # Low RPM
	Vector2(0.4, 0.95),  # Mid RPM
	Vector2(0.6, 1.0),   # Peak torque
	Vector2(0.8, 0.95),  # High RPM
	Vector2(1.0, 0.85)   # Redline
]: set = _set_torque_curve

# Gear ratios (ratio = engine RPM / wheel RPM)
@export var gear_ratios: Array[float] = [
	GEAR_RATIO_NEUTRAL,    # Neutral
	4.2,                   # Reverse
	3.8,                   # 1st gear
	2.6,                   # 2nd gear
	1.9,                   # 3rd gear
	1.5,                   # 4th gear
	1.2,                   # 5th gear
	1.0,                   # 6th gear
	0.85                   # 7th gear
]

@export_group("Steering & Handling")
@export var steering_sensitivity: float = 1.0
@export var steering_recovery_speed: float = 3.0
@export var grip_coefficient: float = 1.2
@export var slip_threshold: float = 0.3
@export var drift_multiplier: float = 1.5

@export_group("Suspension & Body")
@export var suspension_stiffness: float = 50000.0
@export var suspension_damping: float = 5000.0
@export var center_of_mass_offset: Vector3 = Vector3(0.0, 0.5, 0.0)
@export var aerodynamic_drag: float = 0.45
@export var frontal_area: float = 2.2

@export_group("Engine Settings")
@export var idle_rpm: float = 800.0
@export var redline_rpm: float = 7500.0
@export var engine_max_rpm: float = 8500.0
@export var clutch_disengagement_rpm: float = 200.0

# ============================================================================
# PRIVATE VARIABLES - Internal state tracking
# ============================================================================
var current_gear: int = NEUTRAL_GEAR
var current_rpm: float = 0.0
var current_speed_kmh: float = 0.0
var engine_running: bool = false
var handbrake_active: bool = false
var is_drifting: bool = false
var drift_angle: float = 0.0

# Input values (-1.0 to 1.0)
var throttle_input: float = 0.0
var brake_input: float = 0.0
var steering_input: float = 0.0

# Wheel states
var wheel_positions: Array[Vector3] = []
var wheel_velocities: Array[float] = []
var wheel_forces: Array[float] = []
var wheel_contact_points: Array[Vector3] = []
var wheel_normal_forces: Array[float] = []

# Clutch and transmission
var clutch_engaged: bool = true
var engine_torque: float = 0.0
var wheel_torque: float = 0.0
var differential_ratio: float = 1.0

# Aerodynamics
var air_density: float = 1.225  # kg/m³ at sea level
var drag_force: float = 0.0
var downforce: float = 0.0

# Physics references
var powertrain_ref: Node = null
var chassis_body: RigidBody3D = null
var wheel_colliders: Array[CollisionShape3D] = []
var wheel_meshes: Array[MeshInstance3D] = []

# Timing and smoothing
var last_update_time: float = 0.0
var steering_angle_current: float = 0.0
var steering_angle_target: float = 0.0
var velocity_smooth: Vector3 = Vector3.ZERO
var acceleration_history: Array[float] = []
var last_position: Vector3 = Vector3.ZERO

# Drift state tracking
var drift_traction_loss: float = 0.0
var lateral_velocity: float = 0.0
var longitudinal_velocity: float = 0.0

# ============================================================================
# INITIALIZATION
# ============================================================================
func _ready() -> void:
	_process_mode = ProcessModeEnum.ALWAYS
	
	# Initialize wheel arrays
	wheel_positions.resize(4)
	wheel_velocities.resize(4)
	wheel_forces.resize(4)
	wheel_contact_points.resize(4)
	wheel_normal_forces.resize(4)
	
	# Find child references
	_find_vehicle_components()
	
	# Connect to input manager
	if GameManager and GameManager.has_signal("input_changed"):
		GameManager.input_changed.connect(_on_input_changed)

func _find_vehicle_components() -> void:
	"""Find and reference all vehicle sub-components"""
	powertrain_ref = get_node_or_null("../Powertrain")
	chassis_body = get_node_or_null("Chassis")
	
	# Find wheel colliders
	for child in get_children():
		if child is CollisionShape3D and child.name.to_lower().contains("wheel"):
			wheel_colliders.append(child)
		
		if child is MeshInstance3D and child.name.to_lower().contains("wheel"):
			wheel_meshes.append(child)

func _process(delta: float) -> void:
	"""Main update loop for vehicle physics"""
	last_update_time = Time.get_ticks_msec() / 1000.0
	
	# Read inputs
	_read_inputs()
	
	# Update physics
	_update_engine(delta)
	_update_transmission(delta)
	_update_wheels(delta)
	_update_aerodynamics(delta)
	_update_drift(delta)
	
	# Apply forces to rigid body
	_apply_physics_forces(delta)
	
	# Handle collision detection
	_check_collisions()
	
	# Emit signals
	_emit_signals()

func _physics_process(delta: float) -> void:
	"""Physics-specific processing"""
	# Ensure we're using fixed timestep for physics consistency
	var dt = delta if delta < 0.1 else 0.016  # Cap at 60 FPS equivalent
	
	# Move character body (for simple movement)
	if is_on_floor():
		move_and_slide()

# ============================================================================
# INPUT HANDLING
# ============================================================================
func _read_inputs() -> void:
	"""Read and process player inputs"""
	if not InputManager:
		return
	
	# Get raw input values
	throttle_input = InputManager.get_action_value("accelerate")
	brake_input = InputManager.get_action_value("brake")
	steering_input = InputManager.get_action_value("steer_left") + InputManager.get_action_value("steer_right")
	
	# Handbrake toggle
	if Input.is_action_just_pressed("handbrake"):
		handbrake_active = !handbrake_active
		emit_signal("handbrake_toggled", handbrake_active)
	
	# Gear shifting
	if Input.is_action_just_pressed("up_shift"):
		_shift_gear(1)
	elif Input.is_action_just_pressed("down_shift"):
		_shift_gear(-1)
	
	# Engine start/stop
	if Input.is_action_just_pressed("start_engine") and not engine_running:
		_start_engine()
	elif Input.is_action_just_pressed("stop_engine") and engine_running:
		_stop_engine()

func _on_input_changed(action: String, value: float, pressed: bool) -> void:
	"""Handle input changes from InputManager"""
	match action:
		"accelerate":
			throttle_input = clamp(value, -1.0, 1.0)
		"brake":
			brake_input = clamp(value, -1.0, 1.0)
		"steer_left":
			steering_input -= value
		"steer_right":
			steering_input += value
		"handbrake":
			if pressed:
				handbrake_active = !handbrake_active
				emit_signal("handbrake_toggled", handbrake_active)

# ============================================================================
# ENGINE CONTROL
# ============================================================================
func _update_engine(delta: float) -> void:
	"""Update engine RPM and torque output"""
	if not engine_running:
		current_rpm = idle_rpm
		engine_torque = 0.0
		return
	
	# Calculate target RPM based on gear and throttle
	var target_rpm = _calculate_target_rpm()
	
	# Apply throttle response curve
	var throttle_response = pow(throttle_input, 0.7) if throttle_input > 0 else 0.0
	
	# Smooth RPM transition
	var rpm_change_rate = 3000.0 * throttle_response * delta
	target_rpm = lerp(current_rpm, target_rpm, rpm_change_rate)
	
	# Clamp to valid range
	current_rpm = clamp(target_rpm, idle_rpm, engine_max_rpm)
	
	# Calculate engine torque using torque curve interpolation
	var rpm_fraction = (current_rpm - idle_rpm) / (engine_max_rpm - idle_rpm)
	rpm_fraction = clamp(rpm_fraction, 0.0, 1.0)
	engine_torque = _interpolate_torque_curve(rpm_fraction)
	
	# Apply engine braking when not accelerating
	if throttle_input <= 0.1:
		engine_torque *= 0.3  # Reduced torque during coasting

func _calculate_target_rpm() -> float:
	"""Calculate desired RPM based on current speed and gear"""
	if current_gear == NEUTRAL_GEAR:
		return idle_rpm
	
	var wheel_rpm = current_speed_kmh / (3.6 * 2.0 * PI * tire_radius)
	var gear_ratio = gear_ratios[current_gear]
	var target_rpm = wheel_rpm * gear_ratio * final_drive_ratio * differential_ratio
	
	# Prevent RPM from going below idle or above redline
	target_rpm = clamp(target_rpm, idle_rpm, engine_max_rpm)
	
	return target_rpm

func _interpolate_torque_curve(rpm_fraction: float) -> float:
	"""Interpolate torque multiplier from torque curve data"""
	if torque_curve.size() < 2:
		return 0.0
	
	# Find surrounding points
	var lower_idx = 0
	var upper_idx = torque_curve.size() - 1
	
	for i in range(torque_curve.size() - 1):
		if rpm_fraction >= torque_curve[i].x and rpm_fraction <= torque_curve[i + 1].x:
			lower_idx = i
			upper_idx = i + 1
			break
	
	# Linear interpolation between points
	var t = (rpm_fraction - torque_curve[lower_idx].x) / (torque_curve[upper_idx].x - torque_curve[lower_idx].x)
	t = clamp(t, 0.0, 1.0)
	
	var torque_mult = torque_curve[lower_idx].y + t * (torque_curve[upper_idx].y - torque_curve[lower_idx].y)
	
	return engine_torque * torque_mult * 300.0  # Scale to realistic torque values

# ============================================================================
# TRANSMISSION CONTROL
# ============================================================================
func _update_transmission(delta: float) -> void:
	"""Update transmission and gear logic"""
	if not clutch_engaged:
		return
	
	# Automatic upshift/downshift logic
	if drivetrain_type != DrivetrainType.AWD_VARYING and gear_ratios.size() > 0:
		_auto_shift_gears()
	
	# Calculate wheel torque from engine torque
	var gear_ratio = gear_ratios[current_gear]
	if gear_ratio != GEAR_RATIO_NEUTRAL:
		wheel_torque = engine_torque * gear_ratio * final_drive_ratio * differential_ratio
		
		# Apply drivetrain efficiency losses (typically 10-15%)
		var drivetrain_efficiency = 0.85
		wheel_torque *= drivetrain_efficiency
	else:
		wheel_torque = 0.0

func _auto_shift_gears() -> void:
	"""Automatic gear shifting based on RPM and speed"""
	if current_gear == NEUTRAL_GEAR:
		return
	
	# Upshift if RPM approaches redline
	if current_rpm > redline_rpm and current_gear < gear_ratios.size() - 1:
		_shift_gear(1)
	
	# Downshift if RPM drops too low
	elif current_rpm < idle_rpm * 1.5 and current_gear > 1:
		_shift_gear(-1)

func _shift_gear(direction: int) -> void:
	"""Shift gears manually or automatically"""
	var old_gear = current_gear
	
	# Disengage clutch briefly during shift
	clutch_engaged = false
	await get_tree().create_timer(0.2).timeout  # Simulate shift time
	
	# Calculate new gear
	var new_gear = current_gear + direction
	
	# Validate new gear
	if new_gear >= 0 and new_gear < gear_ratios.size():
		current_gear = new_gear
		clutch_engaged = true
		
		# Emit signal
		if old_gear != current_gear:
			emit_signal("gear_changed", old_gear, current_gear)
			
			# Apply shift shock effect
			_apply_shift_shock()

func _apply_shift_shock() -> void:
	"""Apply visual/audio feedback for gear shifts"""
	# Screen shake effect
	if GameManager.debug_mode:
		# Implement screen shake here
		pass

func _start_engine() -> void:
	"""Start the engine"""
	if engine_running:
		return
	
	engine_running = true
	current_rpm = idle_rpm
	engine_torque = 0.0
	
	emit_signal("engine_started")

func _stop_engine() -> void:
	"""Stop the engine"""
	if not engine_running:
		return
	
	engine_running = false
	current_rpm = 0.0
	engine_torque = 0.0
	current_gear = NEUTRAL_GEAR
	
	emit_signal("engine_stopped")

func engage_clutch() -> void:
	"""Engage the clutch"""
	clutch_engaged = true

func disengage_clutch() -> void:
	"""Disengage the clutch"""
	clutch_engaged = false

# ============================================================================
# WHEEL PHYSICS
# ============================================================================
func _update_wheels(delta: float) -> void:
	"""Update individual wheel physics and forces"""
	# Calculate wheel rotation speed from vehicle speed
	var wheel_angular_velocity = current_speed_kmh / (3.6 * 2.0 * PI * tire_radius)
	
	# Update each wheel
	for i in range(wheel_colliders.size()):
		_update_single_wheel(i, delta, wheel_angular_velocity)

func _update_single_wheel(index: int, delta: float, wheel_angular_velocity: float) -> void:
	"""Update a single wheel's physics state"""
	if index >= wheel_colliders.size():
		return
	
	var wheel_shape = wheel_colliders[index].get_world_3d().direct_space_state.sphere_cast(
		SphereCastParameters3D.create(
			wheel_colliders[index].global_position,
			tire_radius,
			Vector3.UP,
			tire_radius * 2.0
		)
	)
	
	# Calculate wheel force based on torque
	var wheel_force = wheel_torque / tire_radius
	
	# Apply drivetrain-specific force distribution
	match drivetrain_type:
		DrivetrainType.FWD:
			if index < 2:  # Front wheels
				pass  # Apply full force
			else:
				wheel_force *= 0.1  # Minimal rear force
		DrivetrainType.RWD:
			if index >= 2:  # Rear wheels
				pass  # Apply full force
			else:
				wheel_force *= 0.1  # Minimal front force
		DrivetrainType.AWD:
			wheel_force *= 0.5  # Equal distribution
		DrivetrainType.AWD_VARYING:
			# Variable distribution based on traction needs
			var front_ratio = 0.4 if abs(steering_input) > 0.5 else 0.3
			if index < 2:
				wheel_force *= front_ratio
			else:
				wheel_force *= (1.0 - front_ratio)
	
	# Store wheel forces
	wheel_forces[index] = wheel_force
	
	# Update wheel position for visualization
	if index < wheel_meshes.size():
		wheel_meshes[index].global_position = wheel_colliders[index].global_position

# ============================================================================
# AERODYNAMICS
# ============================================================================
func _update_aerodynamics(delta: float) -> void:
	"""Calculate aerodynamic forces"""
	var speed_ms = current_speed_kmh / 3.6
	
	# Drag force: F_drag = 0.5 * ρ * v² * Cd * A
	drag_force = 0.5 * air_density * pow(speed_ms, 2) * aerodynamic_drag * frontal_area
	
	# Downforce (simplified - proportional to drag)
	downforce = drag_force * 0.3
	
	# Apply drag as opposite to velocity direction
	var drag_direction = velocity.normalized() if velocity.length() > 0.01 else Vector3.ZERO
	drag_force_vector = drag_direction * -drag_force

# ============================================================================
# DRIFT MECHANICS
# ============================================================================
func _update_drift(delta: float) -> void:
	"""Update drift state and traction"""
	if not handbrake_active and abs(steering_input) < 0.3:
		is_drifting = false
		return
	
	# Calculate lateral acceleration
	lateral_velocity = velocity.x * cos(velocity.angle()) - velocity.z * sin(velocity.angle())
	longitudinal_velocity = velocity.x * sin(velocity.angle()) + velocity.z * cos(velocity.angle())
	
	# Determine if drifting based on lateral vs longitudinal velocity ratio
	var slip_ratio = abs(lateral_velocity) / max(abs(longitudinal_velocity), 0.1)
	
	if slip_ratio > slip_threshold and abs(steering_input) > 0.5:
		if not is_drifting:
			is_drifting = true
			emit_signal("drift_started", slip_ratio)
		
		# Accumulate drift angle
	漂移_angle = lerp(drift_angle, steering_input * steering_angle_max, 0.1 * drift_multiplier)
		drift_traction_loss = min(slip_ratio, 1.0)
	else:
		if is_drifting:
			is_drifting = false
			emit_signal("drift_ended")
			drift_angle = 0.0
			drift_traction_loss = 0.0

# ============================================================================
# PHYSICS FORCES
# ============================================================================
func _apply_physics_forces(delta: float) -> void:
	"""Apply all calculated forces to the vehicle"""
	if not chassis_body:
		return
	
	# Apply gravity
	var gravity_force = Vector3.DOWN * vehicle_mass * PhysicsSettings.gravity
	
	# Apply engine/drive force
	var drive_force = wheel_torque / tire_radius
	var drive_direction = Vector3.FORWARD
	
	# Apply steering influence
	var steer_effect = steering_input * steering_sensitivity
	var steering_rotation = Quaternion.IDENTITY.rotated(Vector3.UP, deg_to_rad(steer_effect))
	drive_direction = steering_rotation * drive_direction
	
	# Combine forces
	var total_force = drive_force * drive_direction
	
	# Add aerodynamic forces
	total_force += drag_force_vector
	
	# Apply as force to center of mass
	chassis_body.apply_central_force(total_force * vehicle_mass)
	
	# Apply braking force
	if brake_input > 0.1 or handbrake_active:
		var brake_strength = brake_input * braking_force
		if handbrake_active:
			brake_strength *= 1.5  # Extra braking from handbrake
		
		var brake_force = -velocity.normalized() * brake_strength
		chassis_body.apply_central_force(brake_force)

# ============================================================================
# COLLISION DETECTION
# ============================================================================
func _check_collisions() -> void:
	"""Check for collisions and emit signals"""
	for collision in get_colliding_bodies():
		if collision.is_in_group("obstacle") or collision.is_in_group("vehicle"):
			var collision_data = {
				"collider": collision,
				"position": global_position,
				"velocity": velocity,
				"impact_force": velocity.length() * vehicle_mass
			}
			
			emit_signal("collision_detected", collision_data)
			
			# Apply impact force to chassis
			if chassis_body:
				chassis_body.apply_impulse(collision_data["position"] - global_position, 
					collision_data["velocity"] * 0.5)

# ============================================================================
# SIGNALS AND OUTPUT
# ============================================================================
func _emit_signals() -> void:
	"""Emit updated state signals"""
	# Speed change
	if abs(current_speed_kmh - speed_changed.last_value) > 0.1:
		emit_signal("speed_changed", current_speed_kmh)
	
	# RPM change
	if abs(current_rpm - rpm_changed.last_value) > 10.0:
		emit_signal("rpm_changed", current_rpm)
	
	# Movement
	if global_position.distance_to(last_position) > 0.1:
		emit_signal("vehicle_moved", global_position, velocity)
	
	last_position = global_position

# ============================================================================
# GETTERS AND SETTERS
# ============================================================================
func _set_vehicle_mass(value: float) -> void:
	vehicle_mass = value
	if chassis_body:
		chassis_body.mass = vehicle_mass

func _set_max_speed_kmh(value: float) -> void:
	max_speed_kmh = value

func _set_acceleration_force(value: float) -> void:
	acceleration_force = value

func _set_braking_force(value: float) -> void:
	braking_force = value

func _set_steering_angle_max(value: float) -> void:
	steering_angle_max = value

func _set_final_drive_ratio(value: float) -> void:
	final_drive_ratio = value
	_update_transmission(0.0)

func _set_tire_radius(value: float) -> void:
	tire_radius = value

func _set_torque_curve(value: Array[Vector2]) -> void:
	torque_curve = value

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================
func get_speed_ms() -> float:
	"""Convert km/h to m/s"""
	return current_speed_kmh / 3.6

func get_speed_kmh() -> float:
	"""Get current speed in km/h"""
	return current_speed_kmh

func get_rpm() -> float:
	"""Get current engine RPM"""
	return current_rpm

func get_gear() -> int:
	"""Get current gear"""
	return current_gear

func get_steering_angle() -> float:
	"""Get current steering angle in degrees"""
	return steering_angle_current

func reset_vehicle() -> void:
	"""Reset vehicle to starting position"""
	global_position = Vector3.ZERO
	velocity = Vector3.ZERO
	current_gear = NEUTRAL_GEAR
	current_rpm = idle_rpm
	engine_running = false
	handbrake_active = false
	is_drifting = false
	steering_input = 0.0
	throttle_input = 0.0
	brake_input = 0.0

func debug_set_rpm(rpm: float) -> void:
	"""Debug function to manually set RPM"""
	current_rpm = clamp(rpm, idle_rpm, engine_max_rpm)

func debug_set_gear(gear: int) -> void:
	"""Debug function to manually set gear"""
	if gear >= 0 and gear < gear_ratios.size():
		current_gear = gear

func get_wheel_contact_point(index: int) -> Vector3:
	"""Get contact point for a specific wheel"""
	if index < wheel_contact_points.size():
		return wheel_contact_points[index]
	return Vector3.ZERO

func get_wheel_normal_force(index: int) -> float:
	"""Get normal force on a specific wheel"""
	if index < wheel_normal_forces.size():
		return wheel_normal_forces[index]
	return 0.0

# ============================================================================
# DESTRUCTOR
# ============================================================================
func _exit_tree() -> void:
	"""Cleanup on exit"""
	engine_running = false
	vehicle_mass = VEHICLE_BASE_MASS

</file>