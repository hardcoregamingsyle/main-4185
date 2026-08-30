extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulator
## Implements throttle, brake, steering inputs, wheel forces, and gear shifting logic
## Uses PhysicsSettings for centralized configuration of all physical parameters
## Copyright 2026 Thalamus Racing Simulator Project

# ============================================================================
# SIGNALS
# ============================================================================

signal speed_changed(current_speed: float)
signal gear_changed(old_gear: int, new_gear: int)
signal traction_control_changed(enabled: bool)
signal drift_mode_changed(active: bool)

# ============================================================================
# ENUMS & CONSTANTS
# ============================================================================

enum Drivetrain {
	FRONT_WHEEL_DRIVE,
	REAR_WHEEL_DRIVE,
	ALL_WHEEL_DRIVE
}

enum TransmissionType {
	MANUAL,
	AUTOMATIC
}

enum BrakingMode {
	NORMAL,
	E_BRAKE,
	CORNERING
}

const DEFAULT_ACCELERATION: float = 8.5
const DEFAULT_BRAKING_FORCE: float = 15.0
const DEFAULT_STEERING_SPEED: float = 45.0
const MAX_STEER_ANGLE: float = DEG_TO_RAD(40.0)
const MIN_GEAR: int = 0
const MAX_GEAR: int = 6

# ============================================================================
# EXPORTED SETTINGS (Tunable via Inspector)
# ============================================================================

@export_group("Vehicle Properties")
@export var mass: float = 1500.0: set = _set_mass
@export var max_power_hp: float = 400.0
@export var torque_nm: float = 600.0
@export var drivetrain: Drivetrain = Drivetrain.REAR_WHEEL_DRIVE
@export var transmission_type: TransmissionType = TransmissionType.MANUAL

@export_group("Physics Tuning")
@export var acceleration_multiplier: float = 1.0
@export var braking_multiplier: float = 1.0
@export var steering_multiplier: float = 1.0
@export var grip_level: float = 1.0

@export_group("Drift Settings")
@export var drift_enabled: bool = true
@export var drift_threshold: float = 25.0
@export var drift_gravity_penalty: float = 0.5

@export_group("Visual Feedback")
@export var headlight_enabled: bool = true
@export var tail_light_enabled: bool = true
@export var brake_light_enabled: bool = true

# ============================================================================
# INTERNAL STATE
# ============================================================================

var current_speed: float = 0.0
var forward_velocity: Vector3 = Vector3.ZERO
var lateral_velocity: Vector3 = Vector3.ZERO
var angular_velocity: Vector3 = Vector3.ZERO

var current_gear: int = 1
var target_gear: int = 1
var rpm: float = 0.0
var clutch_engaged: bool = true

var throttle_input: float = 0.0
var brake_input: float = 0.0
var steering_input: float = 0.0
var handbrake_input: float = 0.0

var last_frame_time: float = 0.0
var time_scale: float = 1.0

var _physics_settings: PhysicsSettings = null
var _current_rpm_curve: Dictionary = {}
var _gear_ratios: Array[float] = []
var _final_drive_ratio: float = 3.73

# Reference nodes
var _front_wheels: Array[Node3D] = []
var _rear_wheels: Array[Node3D] = []
var _headlights: Array[SpotLight3D] = []
var _tail_lights: Array[PointLight3D] = []

# ============================================================================
# PUBLIC API
# ============================================================================

func get_current_speed() -> float:
	return abs(current_speed)

func get_forward_velocity() -> Vector3:
	return forward_velocity

func get_gear() -> int:
	return current_gear

func get_rpm() -> float:
	return rpm

func is_driving_forward() -> bool:
	return current_speed > 0.01

func is_driving_backward() -> bool:
	return current_speed < -0.01

func is_stopped() -> bool:
	return abs(current_speed) <= 0.01

func is_in_neutral() -> bool:
	return current_gear == 0

func get_max_rpm() -> float:
	return _get_max_rpm_for_gear(current_gear)

func shift_up() -> void:
	if current_gear < MAX_GEAR:
		target_gear = min(current_gear + 1, MAX_GEAR)

func shift_down() -> void:
	if current_gear > MIN_GEAR:
		target_gear = max(current_gear - 1, MIN_GEAR)

func auto_shift() -> void:
	if RPM_MODE == TRANSMISSION_TYPE.AUTOMATIC:
		var threshold_low: float = 1500.0
		var threshold_high: float = 6500.0
		
		if current_gear < MAX_GEAR and rpm >= threshold_high:
			target_gear = current_gear + 1
		elif current_gear > MIN_GEAR and rpm <= threshold_low:
			target_gear = max(current_gear - 1, MIN_GEAR)

# ============================================================================
# INITIALIZATION
# ============================================================================

func _ready() -> void:
	_init_physics_settings()
	_init_vehicle_components()
	_calculate_gear_ratios()
	_init_rpm_curve()
	_setup_lighting()
	
	last_frame_time = Engine.get_delta()

# ============================================================================
# PHYSICS UPDATE
# ============================================================================

func _physics_process(delta: float) -> void:
	delta *= time_scale
	
	# Update time tracking
	last_frame_time = delta
	
	# Process inputs
	_process_inputs()
	
	# Handle gear shifts
	_handle_gear_shifting()
	
	# Calculate engine output
	var engine_output := _calculate_engine_output()
	
	# Apply forces based on drivetrain
	_apply_drive_forces(engine_output)
	
	# Apply braking
	_apply_braking()
	
	# Apply steering
	_apply_steering()
	
	# Update physics state
	_update_physics_state(delta)
	
	# Handle special modes (drift, etc.)
	_handle_special_modes(delta)
	
	# Emit signals
	_emit_signals()

# ============================================================================
# INPUT PROCESSING
# ============================================================================

func _process_inputs() -> void:
	# Get input values from InputManager singleton
	var input_manager: InputManager = GameManager.InputManager if GameManager.has_singleton("InputManager") else null
	
	if input_manager != null:
		throttle_input = input_manager.get_axis("vehicle_throttle", 0.0)
		brake_input = input_manager.get_axis("vehicle_brake", 0.0)
		steering_input = input_manager.get_axis("vehicle_steering_left", "vehicle_steering_right", 0.0)
		handbrake_input = input_manager.get_axis("vehicle_handbrake", 0.0)
	else:
		# Fallback to direct input
		throttle_input = Input.get_axis("ui_up", "ui_down")
		brake_input = Input.get_key(KEY_SPACE) ? 1.0 : 0.0
		steering_input = Input.get_axis("ui_left", "ui_right")
		handbrake_input = Input.get_key(KEY_LEFT_SHIFT) ? 1.0 : 0.0
	
	# Clamp inputs to valid range
	throttle_input = clamp(throttle_input, -1.0, 1.0)
	brake_input = clamp(brake_input, 0.0, 1.0)
	steering_input = clamp(steering_input, -1.0, 1.0)
	handbrake_input = clamp(handbrake_input, 0.0, 1.0)

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================

func _handle_gear_shifting() -> void:
	if current_gear == target_gear:
		return
	
	# Check if clutch is engaged (for manual transmission)
	if transmission_type == TransmissionType.MANUAL and not clutch_engaged:
		return
	
	# Smooth gear transition
	var gear_change_delay: float = 0.1
	var shift_complete_time: float = get_tree().time + gear_change_delay
	
	# Execute gear change
	current_gear = target_gear
	emit_signal("gear_changed", target_gear, current_gear)
	
	# Visual feedback could be added here

# ============================================================================
# ENGINE OUTPUT CALCULATION
# ============================================================================

func _calculate_engine_output() -> Dictionary:
	var result: Dictionary = {}
	
	# Calculate current gear ratio
	var gear_ratio: float = _get_gear_ratio(current_gear)
	
	# Calculate wheel RPM based on vehicle speed
	var wheel_rpm: float = _calculate_wheel_rpm()
	
	# Calculate engine RPM
	rpm = wheel_rpm * gear_ratio * _final_drive_ratio
	
	# Clamp RPM to valid range
	rpm = clamp(rpm, 800.0, _get_max_rpm_for_gear(current_gear))
	
	# Look up power/torque from curve
	var power_factor: float = _get_power_factor_from_rpm(rpm)
	var torque_factor: float = _get_torque_factor_from_rpm(rpm)
	
	# Calculate actual output
	var actual_torque: float = torque_nm * torque_factor * power_factor
	var actual_power: float = max_power_hp * power_factor
	
	result["torque"] = actual_torque
	result["power"] = actual_power
	result["rpm"] = rpm
	result["gear_ratio"] = gear_ratio
	
	return result

# ============================================================================
# DRIVE FORCE APPLICATION
# ============================================================================

func _apply_drive_forces(engine_output: Dictionary) -> void:
	var wheel_base: float = 2.5
	var track_width: float = 1.6
	
	match drivetrain:
		Drivetrain.FRONT_WHEEL_DRIVE:
			_apply_force_to_wheels(_front_wheels, engine_output.torque * acceleration_multiplier)
		Drivetrain.REAR_WHEEL_DRIVE:
			_apply_force_to_wheels(_rear_wheels, engine_output.torque * acceleration_multiplier)
		Drivetrain.ALL_WHEEL_DRIVE:
			var front_force: float = engine_output.torque * 0.5 * acceleration_multiplier
			var rear_force: float = engine_output.torque * 0.5 * acceleration_multiplier
			_apply_force_to_wheels(_front_wheels, front_force)
			_apply_force_to_wheels(_rear_wheels, rear_force)

func _apply_force_to_wheels(wheel_nodes: Array[Node3D], force: float) -> void:
	for wheel in wheel_nodes:
		if wheel != null:
			# Apply force in forward direction relative to car's orientation
			var forward_dir: Vector3 = transform.basis.z.normalized()
			wheel.apply_central_impulse(forward_dir * force * _physics_settings.default_vehicle_mass)

# ============================================================================
# BRAKING SYSTEM
# ============================================================================

func _apply_braking() -> void:
	var total_braking_force: float = 0.0
	
	# Normal braking
	if brake_input > 0.0:
		total_braking_force += brake_input * DEFAULT_BRAKING_FORCE * braking_multiplier
		
		# Brake light visual feedback
		if brake_light_enabled:
			_set_brake_light_intensity(brake_input)
	
	# Handbrake (affects only rear wheels)
	if handbrake_input > 0.0:
		var handbrake_force: float = handbrake_input * DEFAULT_BRAKING_FORCE * 0.7
		total_braking_force += handbrake_force
		
		# Enable drift mode when handbrake is active
		if drift_enabled and current_speed > drift_threshold:
			_enable_drift_mode(true)

# ============================================================================
# STEERING SYSTEM
# ============================================================================

func _apply_steering() -> void:
	if not is_stopped() or current_gear != MIN_GEAR:
		# Convert steering input to rotation
		var steer_angle: float = steering_input * MAX_STEER_ANGLE * steering_multiplier
		
		# Apply to front wheels only (for realistic car behavior)
		for wheel in _front_wheels:
			if wheel != null:
				wheel.rotate_y(-steer_angle)

# ============================================================================
# PHYSICS STATE UPDATE
# ============================================================================

func _update_physics_state(delta: float) -> void:
	# Move character body
	move_and_slide()
	
	# Calculate velocity components
	forward_velocity = global_transform.basis.z * linear_velocity.z
	lateral_velocity = global_transform.basis.y * linear_velocity.y
	
	# Update current speed (in km/h for display)
	current_speed = linear_velocity.length() * 3.6
	
	# Clamp speed to reasonable limits
	current_speed = clamp(current_speed, -200.0, 350.0)
	
	# Angular velocity tracking
	angular_velocity = angular_velocity * delta

# ============================================================================
# SPECIAL MODES HANDLING
# ============================================================================

func _handle_special_modes(delta: float) -> void:
	if drift_enabled and handbrake_input > 0.5 and current_speed > drift_threshold:
		_enable_drift_mode(true)
		_apply_drift_forces(delta)
	else:
		_enable_drift_mode(false)

func _enable_drift_mode(enabled: bool) -> void:
	if enabled:
		# Reduce grip during drift
		grip_level = 0.3
		emit_signal("drift_mode_changed", true)
	else:
		grip_level = 1.0
		emit_signal("drift_mode_changed", false)

func _apply_drift_forces(delta: float) -> void:
	# Apply additional lateral resistance reduction
	var lateral_drag: float = 0.1 * delta
	linear_velocity.x *= (1.0 - lateral_drag)

# ============================================================================
# SIGNAL EMITTING
# ============================================================================

func _emit_signals() -> void:
	if abs(current_speed - _last_emitted_speed) > 1.0:
		emit_signal("speed_changed", current_speed)
		_last_emitted_speed = current_speed

# ============================================================================
# HELPER METHODS
# ============================================================================

func _init_physics_settings() -> void:
	# Try to get PhysicsSettings from autoload
	if GameManager.has_singleton("PhysicsSettings"):
		_physics_settings = GameManager.PhysicsSettings
	else:
		_physics_settings = preload("res://scripts/core/PhysicsSettings.gd").new()

func _init_vehicle_components() -> void:
	# Find wheel nodes in scene tree
	_front_wheels = find_children_by_type(Node3D, "_front_wheel_")
	_rear_wheels = find_children_by_type(Node3D, "_rear_wheel_")
	_headlights = find_children_by_type(SpotLight3D, "_headlight_")
	_tail_lights = find_children_by_type(PointLight3D, "_taillight_")

func find_children_by_type(parent: Node, type_name: String) -> Array[Node]:
	var children: Array[Node] = []
	for child in parent.get_children():
		if child.is_class(type_name):
			children.append(child)
	return children

func _calculate_gear_ratios() -> void:
	# Standard gear ratios for a typical performance vehicle
	_gear_ratios = [
		4.0,   # Reverse
		3.5,   # 1st
		2.3,   # 2nd
		1.7,   # 3rd
		1.3,   # 4th
		1.0,   # 5th
		0.8    # 6th
	]

func _get_gear_ratio(gear: int) -> float:
	if gear < 0 or gear >= _gear_ratios.size():
		return 1.0
	return _gear_ratios[gear]

func _calculate_wheel_rpm() -> float:
	# Calculate wheel RPM based on vehicle speed and tire circumference
	var tire_radius: float = 0.32
	var tire_circumference: float = 2.0 * PI * tire_radius
	
	var wheel_rps: float = current_speed / (3.6 * tire_circumference)
	return wheel_rps * 60.0  # Convert to RPM

func _init_rpm_curve() -> void:
	# Define RPM curve points (RPM -> Power Factor)
	_current_rpm_curve = {
		800: 0.1,
		1500: 0.3,
		3000: 0.7,
		4500: 1.0,
		6000: 0.95,
		7500: 0.8,
		9000: 0.5
	}

func _get_power_factor_from_rpm(rpm: float) -> float:
	var factors: Array[float] = _current_rpm_curve.values()
	var rpm_values: Array[float] = _current_rpm_curve.keys()
	
	for i in range(rpm_values.size() - 1):
		if rpm >= rpm_values[i] and rpm <= rpm_values[i + 1]:
			var t: float = (rpm - rpm_values[i]) / (rpm_values[i + 1] - rpm_values[i])
			return lerp(factors[i], factors[i + 1], t)
	
	return factors.min() if rpm < rpm_values[0] else factors.max()

func _get_torque_factor_from_rpm(rpm: float) -> float:
	# Torque typically peaks at lower RPM than power
	var torque_curve: Dictionary = {
		800: 0.8,
		1500: 0.9,
		3000: 1.0,
		4500: 0.95,
		6000: 0.9,
		7500: 0.8,
		9000: 0.6
	}
	
	var factors: Array[float] = torque_curve.values()
	var rpm_values: Array[float] = torque_curve.keys()
	
	for i in range(rpm_values.size() - 1):
		if rpm >= rpm_values[i] and rpm <= rpm_values[i + 1]:
			var t: float = (rpm - rpm_values[i]) / (rpm_values[i + 1] - rpm_values[i])
			return lerp(factors[i], factors[i + 1], t)
	
	return factors.min() if rpm < rpm_values[0] else factors.max()

func _get_max_rpm_for_gear(gear: int) -> float:
	# Max RPM varies by gear due to different gear ratios
	var max_rpm_base: float = 8000.0
	return max_rpm_base / (0.5 + gear * 0.1)

func _set_mass(new_mass: float) -> void:
	mass = new_mass
	# Apply mass change to physics body
	if is_inside_tree():
		modify_body_mass(new_mass)

func modify_body_mass(new_mass: float) -> void:
	# This would modify the rigid body or character body physics properties
	pass

func _setup_lighting() -> void:
	if headlight_enabled:
		for light in _headlights:
			if light != null:
				light.enabled = true
	else:
		for light in _headlights:
			if light != null:
				light.enabled = false

func _set_brake_light_intensity(intensity: float) -> void:
	for light in _tail_lights:
		if light != null:
			light.position.y = intensity * 0.5

func _set_time_scale(value: float) -> void:
	time_scale = value
	Engine.time_scale = value

func reset_vehicle() -> void:
	current_speed = 0.0
	rpm = 0.0
	current_gear = 1
	target_gear = 1
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	handbrake_input = 0.0

# ============================================================================
# DEBUGGING & VISUALIZATION
# ============================================================================

func debug_draw() -> void:
	if not GameManager.debug_mode:
		return
	
	# Draw debug lines for vehicle state
	var start_pos: Vector3 = global_position
	var end_pos: Vector3 = global_position + global_transform.basis.z * 5.0
	
	DebugRenderer.draw_line(start_pos, end_pos, Color.GREEN)
	DebugRenderer.draw_text(str("Speed: ", current_speed), global_position + Vector3.UP * 2, Color.WHITE)
	DebugRenderer.draw_text(str("Gear: ", current_gear), global_position + Vector3.UP * 2.5, Color.YELLOW)
	DebugRenderer.draw_text(str("RPM: ", rpm), global_position + Vector3.UP * 3, Color.RED)

</FILE "scripts/controllers/VehicleController.gd">