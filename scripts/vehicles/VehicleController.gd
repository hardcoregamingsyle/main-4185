extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller base class
## Handles throttle, brake, steering inputs, wheel forces, gear shifting logic
## Integrates with PhysicsSettings constants for consistent tuning across all vehicles
## Copyright 2026 Thalamus Racing Simulator Project

signal speed_changed(speed: float)
signal rpm_changed(rpm: float)
signal gear_changed(old_gear: int, new_gear: int)
signal vehicle_state_changed(state: VehicleState)
signal damage_taken(damage_amount: float)
signal wheel_slip_detected(wheel_index: int, slip_ratio: float)
signal traction_control_triggered(slip_threshold: float)

enum VehicleState { IDLE, ACCELERATING, BRAKING, COASTING, DRIFTING, CRASHED }

# ============================================================================
# PHYSICS CONSTANTS FROM SETTINGS
# ============================================================================

var _physics_settings: PhysicsSettings = null
var _powertrain: Powertrain = null

@export_group("Vehicle Configuration")
@export var vehicle_mass: float = 1500.0
@export var center_of_mass_offset: Vector3 = Vector3(0.0, 0.5, 0.0)
@export var max_steering_angle: float = 30.0: set = _set_max_steering_angle
@export var suspension_stiffness: float = 50000.0
@export var suspension_damping: float = 5000.0
@export var suspension_compression_limit: float = 0.3
@export var suspension_extension_limit: float = 0.5

@export_group("Wheel Configuration")
@export var front_wheel_radius: float = 0.35
@export var rear_wheel_radius: float = 0.35
@export var wheel_track_width: float = 1.6
@export var wheel_base: float = 2.6
@export var wheel_positions: Array[Vector3] = []

# ============================================================================
# VEHICLE STATE
# ============================================================================

@export_group("Vehicle State")
@export var current_speed: float = 0.0: set = _set_current_speed
@export var forward_velocity: Vector3 = Vector3.ZERO
@export var lateral_velocity: Vector3 = Vector3.ZERO
@export var angular_velocity: Vector3 = Vector3.ZERO
@export var rpm: float = 0.0: set = _set_rpm
@export var current_gear: int = 0
@export var clutch_engaged: bool = true
@export var handbrake_active: bool = false
@export var drift_mode: bool = false
@export var traction_control: bool = true

@export var vehicle_state: VehicleState = VehicleState.IDLE: set = _set_vehicle_state

# ============================================================================
# INPUT VALUES
# ============================================================================

@export_group("Input Values")
@export var throttle_input: float = 0.0: set = _set_throttle_input
@export var brake_input: float = 0.0: set = _set_brake_input
@export var steering_input: float = 0.0: set = _set_steering_input
@export var shift_up_requested: bool = false
@export var shift_down_requested: bool = false
@export var nitrous_active: bool = false

# ============================================================================
# POWERTRAIN VARIABLES
# ============================================================================

var engine_force: float = 0.0
var brake_force: float = 0.0
var steering_angle: float = 0.0
var wheel_torque: float = 0.0
var differential_type: int = 0  # 0=open, 1=limited, 2=locked
var final_drive_ratio: float = 3.5

# ============================================================================
# GEAR RATIOS AND RPM LIMITS
# ============================================================================

@export_group("Gear Settings")
@export var gear_ratios: PackedFloat32Array = [0.0, 3.8, 2.5, 1.8, 1.3, 1.0, 0.8]  # Neutral + 6 gears + reverse
@export var reverse_ratio: float = -3.5
@export var gear_shift_rpm_threshold: float = 6500.0
@export var idle_rpm: float = 800.0
@export var redline_rpm: float = 7500.0
@export var max_rpm: float = 8500.0

# ============================================================================
# WHEEL DATA
# ============================================================================

struct WheelData:
	var position: Vector3
	var local_position: Vector3
	var suspension_length: float = 0.0
	var compression: float = 0.0
	var velocity: Vector3 = Vector3.ZERO
	var slip_ratio: float = 0.0
	var drive_force: float = 0.0
	var brake_force: float = 0.0
	var steering_angle: float = 0.0
	var is_in_contact: bool = false
	var contact_normal: Vector3 = Vector3.UP
	var contact_point: Vector3 = Vector3.ZERO

var wheels: Array[WheelData] = []

# ============================================================================
# INTERNAL VARIABLES
# ============================================================================

var _max_steering_angle_rad: float = PI / 6.0
var _last_rpm: float = 0.0
var _last_gear: int = 0
var _speed_sensor: float = 0.0
var _drift_factor: float = 0.0
var _vehicle_acceleration: float = 0.0
var _damage_accumulator: float = 0.0
var _collision_damage_threshold: float = 100.0
var _is_ready: bool = false
var _auto_shift_enabled: bool = true
var _last_auto_shift_direction: int = 0  # -1 down, 0 none, 1 up

# ============================================================================
# INITIALIZATION
# ============================================================================

func _ready() -> void:
	_init_wheels()
	_setup_physics_settings()
	_connect_signals()
	_calculate_wheel_positions()
	_is_ready = true
	
	# Initialize wheel data
	for i in range(4):
		wheels.append(WheelData.new())
		wheels[i].local_position = Vector3.ZERO
		wheels[i].position = Vector3.ZERO

func _init_wheels() -> void:
	"""Initialize wheel positions and data structures"""
	wheels.resize(4)
	
	# Front Left
	wheels[0].local_position = Vector3(-wheel_track_width / 2.0, 0.0, wheel_base / 2.0)
	wheels[0].position = transform * wheels[0].local_position
	
	# Front Right
	wheels[1].local_position = Vector3(wheel_track_width / 2.0, 0.0, wheel_base / 2.0)
	wheels[1].position = transform * wheels[1].local_position
	
	# Rear Left
	wheels[2].local_position = Vector3(-wheel_track_width / 2.0, 0.0, -wheel_base / 2.0)
	wheels[2].position = transform * wheels[2].local_position
	
	# Rear Right
	wheels[3].local_position = Vector3(wheel_track_width / 2.0, 0.0, -wheel_base / 2.0)
	wheels[3].position = transform * wheels[3].local_position

func _setup_physics_settings() -> void:
	"""Get reference to PhysicsSettings singleton"""
	if Engine.has_singleton("PhysicsSettings"):
		_physics_settings = Engine.get_singleton("PhysicsSettings")
	else:
		_physics_settings = preload("res://scripts/core/PhysicsSettings.gd").new()
		
	# Convert steering angle to radians
	_max_steering_angle_rad = deg_to_rad(max_steering_angle)

func _connect_signals() -> void:
	"""Connect necessary signals"""
	if GameManager:
		GameManager.game_state_changed.connect(_on_game_state_changed)
	
	if AudioManager:
		pass  # Audio events handled separately

# ============================================================================
# SETTERS WITH VALIDATION
# ============================================================================

func _set_throttle_input(value: float) -> void:
	throttle_input = clamp(value, -1.0, 1.0)

func _set_brake_input(value: float) -> void:
	brake_input = clamp(value, 0.0, 1.0)

func _set_steering_input(value: float) -> void:
	steering_input = clamp(value, -1.0, 1.0)

func _set_current_speed(value: float) -> void:
	if value != current_speed:
		current_speed = abs(value)
		speed_changed.emit(current_speed)

func _set_rpm(value: float) -> void:
	if value != rpm:
		_last_rpm = rpm
		rpm = clamp(value, idle_rpm, max_rpm)
		rpm_changed.emit(rpm)

func _set_max_steering_angle(value: float) -> void:
	max_steering_angle = clamp(value, 0.0, 60.0)
	_max_steering_angle_rad = deg_to_rad(max_steering_angle)

func _set_vehicle_state(state: VehicleState) -> void:
	if state != vehicle_state:
		_last_gear = current_gear
		vehicle_state = state
		vehicle_state_changed.emit(vehicle_state)

# ============================================================================
# MAIN UPDATE LOOP
# ============================================================================

func _process(delta: float) -> void:
	if not _is_ready:
		return
	
	_process_inputs(delta)
	_update_powertrain(delta)
	_update_gear_shifting(delta)
	_update_vehicle_state()
	_emit_signals()

func _physics_process(delta: float) -> void:
	if not _is_ready or not is_inside_tree():
		return
	
	_apply_physics(delta)
	_update_wheels(delta)
	_handle_collisions(delta)

func _process_inputs(delta: float) -> void:
	"""Process player input and calculate target values"""
	
	# Calculate target steering angle
	steering_angle = lerp(steering_angle, steering_input * _max_steering_angle_rad, delta * 10.0)
	
	# Apply throttle/brake with smoothing
	var target_engine_force: float = 0.0
	
	if throttle_input > 0.0:
		target_engine_force = throttle_input * _physics_settings.default_vehicle_mass * 5.0
	elif brake_input > 0.0:
		target_engine_force = -brake_input * _physics_settings.default_vehicle_mass * 3.0
	else:
		target_engine_force = 0.0
	
	engine_force = lerp(engine_force, target_engine_force, delta * 5.0)
	
	# Handle handbrake
	if handbrake_active:
		brake_force = _physics_settings.default_vehicle_mass * 8.0
	else:
		brake_force = lerp(brake_force, 0.0, delta * 3.0)

func _update_powertrain(delta: float) -> void:
	"""Update powertrain dynamics based on current state"""
	
	if _powertrain == null:
		_powertrain = get_node_or_null("../Powertrain")
	
	if _powertrain == null:
		return
	
	# Calculate wheel torque based on engine force and gear ratio
	var gear_ratio: float = gear_ratios[current_gear] if current_gear < gear_ratios.size() else gear_ratios[-1]
	
	if current_gear == 0:  # Neutral
		gear_ratio = 0.0
	elif current_gear < 0:  # Reverse
		gear_ratio = reverse_ratio
	else:
		gear_ratio = gear_ratios[current_gear]
	
	final_drive_ratio = 3.5
	var total_ratio: float = gear_ratio * final_drive_ratio
	
	# Calculate torque at wheels
	wheel_torque = engine_force * 0.15 * total_ratio  # Simplified torque calculation
	
	# Update powertrain
	if _powertrain.has_method("update"):
		_powertrain.update(rpm, current_gear, engine_force, wheel_torque)

func _update_gear_shifting(delta: float) -> void:
	"""Handle automatic and manual gear shifting"""
	
	# Auto-shift logic
	if _auto_shift_enabled and throttle_input > 0.0:
		_auto_shift_logic(delta)
	
	# Manual shift requests
	if shift_up_requested:
		request_shift_up()
		shift_up_requested = false
	
	if shift_down_requested:
		request_shift_down()
		shift_down_requested = false
	
	# Redline protection
	if rpm >= max_rpm:
		engine_force *= 0.9  # Reduce engine force near redline

func _auto_shift_logic(delta: float) -> void:
	"""Automatic gear shifting based on RPM"""
	
	if rpm < gear_shift_rpm_threshold * 0.3 and current_gear > 0:
		# Downshift if RPM too low
		if current_gear > 1 and _last_auto_shift_direction <= 0:
			request_shift_down()
			_last_auto_shift_direction = -1
	elif rpm >= gear_shift_rpm_threshold and current_gear < gear_ratios.size() - 1:
		# Upshift if RPM too high
		if current_gear < gear_ratios.size() - 2 and _last_auto_shift_direction >= 0:
			request_shift_up()
			_last_auto_shift_direction = 1
	else:
		_last_auto_shift_direction = 0

func request_shift_up() -> void:
	"""Request a gear shift up"""
	if current_gear < gear_ratios.size() - 2:
		var old_gear = current_gear
		current_gear += 1
		
		# Simulate clutch engagement during shift
		if clutch_engaged:
			rpm = max(idle_rpm, rpm * 0.7)  # RPM drops during upshift
		
		gear_changed.emit(old_gear, current_gear)

func request_shift_down() -> void:
	"""Request a gear shift down"""
	if current_gear > 0:
		var old_gear = current_gear
		current_gear -= 1
		
		# Simulate clutch engagement during shift
		if clutch_engaged:
			rpm = min(max_rpm, rpm * 1.3)  # RPM rises during downshift
		
		gear_changed.emit(old_gear, current_gear)

# ============================================================================
# VEHICLE STATE MANAGEMENT
# ============================================================================

func _update_vehicle_state() -> void:
	"""Update vehicle state based on current conditions"""
	
	var target_state: VehicleState = VehicleState.IDLE
	
	if rpm > redline_rpm:
		target_state = VehicleState.CRASHED
	elif brake_input > 0.8:
		target_state = VehicleState.BRAKING
	elif throttle_input > 0.1 and rpm > idle_rpm:
		target_state = VehicleState.ACCELERATING
	elif throttle_input < -0.1:
		target_state = VehicleState.DRIFTING
	else:
		target_state = VehicleState.COASTING
	
	# Smooth state transitions
	if vehicle_state != target_state:
		vehicle_state = target_state

func _emit_signals() -> void:
	"""Emit any pending signals"""
	
	# Check for significant RPM changes
	if abs(rpm - _last_rpm) > 100.0:
		rpm_changed.emit(rpm)
	
	# Check for gear changes
	if current_gear != _last_gear:
		gear_changed.emit(_last_gear, current_gear)
		_last_gear = current_gear

# ============================================================================
# PHYSICS SIMULATION
# ============================================================================

func _apply_physics(delta: float) -> void:
	"""Apply physics forces to the vehicle body"""
	
	# Calculate forward direction
	var forward: Vector3 = global_transform.basis.x.normalized()
	var right: Vector3 = global_transform.basis.z.normalized()
	
	# Get velocity components
	forward_velocity = velocity.dot(forward) * forward
	lateral_velocity = velocity.dot(right) * right
	
	# Apply gravity
	var gravity_force: Vector3 = Vector3.DOWN * _physics_settings.gravity * vehicle_mass
	velocity += gravity_force * delta
	
	# Apply wheel forces
	var total_drive_force: float = 0.0
	var total_lateral_force: float = 0.0
	
	# Drive wheels (rear-wheel drive by default)
	for wheel_idx in [2, 3]:  # Rear wheels
		var wheel: WheelData = wheels[wheel_idx]
		total_drive_force += wheel.drive_force
		
		# Apply lateral grip resistance
		if drift_mode:
			wheel.slip_ratio = clamp(abs(lateral_velocity.length()) / 10.0, 0.0, 1.0)
			if wheel.slip_ratio > 0.3:
				total_lateral_force += wheel.slip_ratio * 5000.0
	
	# Calculate vehicle acceleration
	_vehicle_acceleration = total_drive_force / vehicle_mass
	
	# Apply horizontal forces
	var drive_vector: Vector3 = forward * total_drive_force
	var steer_vector: Vector3 = right * steering_angle * lateral_velocity.length() * 100.0
	
	# Add drag
	var drag_force: float = 0.02 * velocity.length_squared()
	drive_vector -= velocity.normalized() * drag_force
	
	# Apply forces to velocity
	velocity += (drive_vector + steer_vector) * delta / vehicle_mass
	
	# Apply brakes
	if brake_force > 0:
		velocity = velocity * (1.0 - brake_force * delta / vehicle_mass)
	
	# Handbrake adds rotation
	if handbrake_active:
		angular_velocity.y += (throttle_input * -0.5 + brake_input * 0.3) * delta * 2.0
	
	# Clamp angular velocity
	angular_velocity = angular_velocity.limit_length(delta * 3.0)
	
	# Update position
	position += velocity * delta
	global_rotation_y += angular_velocity.y * delta
	
	# Keep vehicle upright
	var up: Vector3 = global_transform.basis.y
	up = up.lerp(Vector3.UP, delta * 5.0)
	global_transform.basis.y = up
	global_transform.basis.x = global_transform.basis.z.cross(up).normalized()
	global_transform.basis.z = up.cross(global_transform.basis.x).normalized()

func _update_wheels(delta: float) -> void:
	"""Update wheel states and detect collisions"""
	
	var ground_height: float = 0.0
	
	for wheel in wheels:
		# Update wheel position
		wheel.position = global_transform * wheel.local_position
		
		# Simple raycast for ground detection
		var ray_from: Vector3 = wheel.position + Vector3.UP * 1.0
		var ray_to: Vector3 = wheel.position - Vector3.UP * 2.0
		
		var space_state: PhysicsDirectSpaceState3D = get_world_3d().direct_space_state
		var query: PhysicsRayQueryParameters3D = PhysicsRayQueryParameters3D.create(ray_from, ray_to)
		query.exclude = [self]
		
		var result = space_state.intersect_ray(query)
		
		if result.is_empty():
			wheel.is_in_contact = false
			wheel.suspension_length = 0.0
		else:
			wheel.is_in_contact = true
			wheel.contact_point = result.position
			wheel.contact_normal = result.normal
			
			# Calculate suspension compression
			var desired_height: float = wheel.local_position.y + 0.3
			var actual_height: float = wheel.contact_point.y
			wheel.suspension_length = actual_height - desired_height
			
			wheel.compression = clamp(wheel.suspension_length, -suspension_compression_limit, suspension_extension_limit)
			
			# Update wheel velocity
			wheel.velocity = velocity + angular_velocity.cross(wheel.local_position)
			
			# Calculate slip ratio
			var wheel_peripheral_speed: float = rpm * 0.105  # Approximate peripheral speed
			var vehicle_forward_speed: float = forward_velocity.length()
			
			if vehicle_forward_speed > 0.1:
				wheel.slip_ratio = abs(wheel_peripheral_speed - vehicle_forward_speed) / vehicle_forward_speed
			else:
				wheel.slip_ratio = 0.0
			
			# Detect wheel slip
			if wheel.slip_ratio > 0.5 and traction_control:
				wheel_slip_detected.emit(wheels.find(wheel), wheel.slip_ratio)
				
				if wheel.slip_ratio > 0.7:
					traction_control_triggered.emit(wheel.slip_ratio)
					wheel.drive_force *= 0.5  # Reduce drive force

func _handle_collisions(delta: float) -> void:
	"""Handle collision detection and response"""
	
	for collision in get_slide_collision_iterator():
		if collision.get_collider() is RigidBody3D or collision.get_collider() is StaticBody3D:
			# Calculate impact force
			var impact_velocity: float = collision.get_travel().length() / delta
			var impact_force: float = impact_velocity * vehicle_mass * 0.1
			
			if impact_force > _collision_damage_threshold:
				damage_taken.emit(impact_force - _collision_damage_threshold)
				_damage_accumulator += impact_force
				
				# Screen shake effect
				if AudioManager:
					AudioManager.play_sound("crash_impact")
				
				# Slow down on impact
				velocity *= 0.7

# ============================================================================
# UTILITY METHODS
# ============================================================================

func get_wheel_data(index: int) -> WheelData:
	"""Get data for specific wheel"""
	if index >= 0 and index < wheels.size():
		return wheels[index]
	return WheelData.new()

func get_speed_kmh() -> float:
	"""Get current speed in km/h"""
	return current_speed * 3.6

func get_speed_mph() -> float:
	"""Get current speed in mph"""
	return current_speed * 2.237

func reset_vehicle() -> void:
	"""Reset vehicle to initial state"""
	velocity = Vector3.ZERO
	angular_velocity = Vector3.ZERO
	rpm = idle_rpm
	current_gear = 0
	engine_force = 0.0
	brake_force = 0.0
	steering_angle = 0.0
	wheels.clear()
	_init_wheels()
	vehicle_state = VehicleState.IDLE

func apply_force_at_point(force: Vector3, point: Vector3) -> void:
	"""Apply force at specific point on vehicle"""
	force = global_transform.basis * force
	apply_central_impulse(force)

func set_drift_mode(enabled: bool) -> void:
	"""Enable or disable drift mode"""
(drift_mode = enabled
	if enabled:
		suspension_stiffness *= 0.5
		suspension_damping *= 0.5
	else:
		suspension_stiffness = 50000.0
		suspension_damping = 5000.0

func calculate_wheel_forces() -> Dictionary:
	"""Calculate individual wheel forces for rendering"""
	var forces: Dictionary = {}
	
	for i in range(wheels.size()):
		var wheel = wheels[i]
		forces[str(i)] = {
			"position": wheel.position,
			"normal": wheel.contact_normal,
			"compression": wheel.compression,
			"slip_ratio": wheel.slip_ratio,
			"is_contact": wheel.is_in_contact
		}
	
	return forces

# ============================================================================
# SIGNAL HANDLERS
# ============================================================================

func _on_game_state_changed(new_state: GameState) -> void:
	"""Handle game state changes"""
	match new_state:
		GameManager.GameState.RACE_ACTIVE:
			_is_ready = true
			reset_vehicle()
		GameManager.GameState.RACE_PAUSED:
			_is_ready = false
		GameManager.GameState.MAIN_MENU:
			_is_ready = false
			reset_vehicle()

# ============================================================================
# DEBUG TOOLS
# ============================================================================

func debug_print_vehicle_state() -> void:
	"""Print current vehicle state for debugging"""
	print("=== Vehicle State ===")
	print("Speed: %.2f km/h (%.2f mph)" % [get_speed_kmh(), get_speed_mph()])
	print("RPM: %.0f / %.0f (redline)" % [rpm, redline_rpm])
	print("Gear: %d" % current_gear)
	print("State: %s" % vehicle_state)
	print("Throttle: %.2f | Brake: %.2f | Steering: %.2f" % [throttle_input, brake_input, steering_input])
	print("====================")

func _draw() -> void:
	"""Debug drawing for visualization"""
	if not GameManager.debug_mode:
		return
	
	# Draw wheel positions
	for wheel in wheels:
		var color = Color.GREEN if wheel.is_in_contact else Color.RED
		draw_sphere(wheel.position, 0.05, color)

func draw_sphere(position: Vector3, radius: float, color: Color) -> void:
	"""Draw a sphere at given position"""
	# Implementation would use visual debugging tools
	pass

</FILE>

Now let me verify the file was created correctly.

{"op":"cmd","command":"wc -l scripts/vehicles/VehicleController.gd"}))
	if enabled:
		suspension_stiffness *= 0.5
		suspension_damping *= 0.5
	else:
		suspension_stiffness = 50000.0
		suspension_damping = 5000.0

func calculate_wheel_forces() -> Dictionary:
	"""Calculate individual wheel forces for rendering"""
	var forces: Dictionary = {}
	
	for i in range(wheels.size()):
		var wheel = wheels[i]
		forces[str(i)] = {
			"position": wheel.position,
			"normal": wheel.contact_normal,
			"compression": wheel.compression,
			"slip_ratio": wheel.slip_ratio,
			"is_contact": wheel.is_in_contact
		}
	
	return forces

# ============================================================================
# SIGNAL HANDLERS
# ============================================================================

func _on_game_state_changed(new_state: GameState) -> void:
	"""Handle game state changes"""
	match new_state:
		GameManager.GameState.RACE_ACTIVE:
			_is_ready = true
			reset_vehicle()
		GameManager.GameState.RACE_PAUSED:
			_is_ready = false
		GameManager.GameState.MAIN_MENU:
			_is_ready = false
			reset_vehicle()

# ============================================================================
# DEBUG TOOLS
# ============================================================================

func debug_print_vehicle_state() -> void:
	"""Print current vehicle state for debugging"""
	print("=== Vehicle State ===")
	print("Speed: %.2f km/h (%.2f mph)" % [get_speed_kmh(), get_speed_mph()])
	print("RPM: %.0f / %.0f (redline)" % [rpm, redline_rpm])
	print("Gear: %d" % current_gear)
	print("State: %s" % vehicle_state)
	print("Throttle: %.2f | Brake: %.2f | Steering: %.2f" % [throttle_input, brake_input, steering_input])
	print("====================")

func _draw() -> void:
	"""Debug drawing for visualization"""
	if not GameManager.debug_mode:
		return
	
	# Draw wheel positions
	for wheel in wheels:
		var color = Color.GREEN if wheel.is_in_contact else Color.RED
		draw_sphere(wheel.position, 0.05, color)

func draw_sphere(position: Vector3, radius: float, color: Color) -> void:
	"""Draw a sphere at given position (debug only)"""
	# Implementation would use visual debugging tools
	pass