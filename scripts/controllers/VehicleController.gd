extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for the racing simulator
## Handles throttle, brake, steering inputs, wheel forces, and gear shift logic
## All physics values are derived from PhysicsSettings singleton for consistency
## Copyright 2026 Thalamus Racing Simulator Project

# Signals for vehicle state changes
signal speed_changed(current_speed: float)
signal rpm_changed(current_rpm: float)
signal gear_changed(new_gear: int)
signal drift_started()
signal drift_ended()
signal crash_detected(impact_force: float)

# Physics settings reference (autoload)
var _physics: PhysicsSettings = GameManager.physics_settings if GameManager.has_singleton("PhysicsSettings") else preload("res://scripts/core/PhysicsSettings.gd").new()

# Vehicle mass and inertia
@export_group("Vehicle Properties")
@export var vehicle_mass: float = _physics.default_vehicle_mass
@export var center_of_mass_offset: Vector3 = Vector3.ZERO
@export var chassis_suspension_stiffness: float = 50000.0
@export var chassis_damping: float = 5000.0

# Wheel configuration
@export_group("Wheel Configuration")
@export var front_wheel_track_width: float = 1.6
@export var rear_wheel_track_width: float = 1.6
@export var wheel_base: float = 2.8
@export var wheel_radius: float = 0.35
@export var suspension_travel: float = 0.15

# Powertrain parameters
@export_group("Powertrain")
@export var max_engine_power: float = 300000.0  # Watts (approx 400 HP)
@export var max_torque: float = 500.0  # Newton-meters
@export var idle_rpm: float = 800.0
@export var redline_rpm: float = 7000.0
@export var gears_count: int = 6

# Gear ratios (gear_ratio = final_drive / gear_number)
var _gear_ratios: Array[float] = []
var _final_drive_ratio: float = 3.73

# Current state
@export_group("Current State")
var current_gear: int = 0  # 0 = neutral, 1-6 = forward gears, -1 = reverse
var current_rpm: float = 0.0
var current_speed: float = 0.0  # m/s
var drift_angle: float = 0.0
var is_drifting: bool = false
var is_braking: bool = false
var is_throttling: bool = false

# Input handling
var _throttle_input: float = 0.0  # 0.0 to 1.0
var _brake_input: float = 0.0    # 0.0 to 1.0
var _steering_input: float = 0.0 # -1.0 to 1.0

# Wheel states (for rendering and physics)
var _front_left_wheel: Dictionary = {}
var _front_right_wheel: Dictionary = {}
var _rear_left_wheel: Dictionary = {}
var _rear_right_wheel: Dictionary = {}

# Force application
var _engine_force: float = 0.0
var _brake_force: float = 0.0
var _steering_angle: float = 0.0

# Drift mechanics
var _drift_threshold: float = 15.0  # m/s lateral velocity threshold
var _drift_recovery_rate: float = 0.95
var _drift_multiplier: float = 0.75

# Collision detection
var _last_collision_time: float = 0.0
var _collision_impact: float = 0.0

func _ready() -> void:
	_init_gear_ratios()
	_reset_wheel_states()
	_process_mode = ProcessModeEnum.ALWAYS
	
	# Connect to input manager if available
	if GameManager.has_singleton("InputManager"):
		var input_mgr = GameManager.get_node("/root/InputManager") if get_tree().root.has_node("InputManager") else null
		if input_mgr != null:
			_connect_input_signals(input_mgr)

func _init_gear_ratios() -> void:
	"""Initialize gear ratios based on typical transmission characteristics"""
	_final_drive_ratio = 3.73
	
	match gears_count:
		4:
			_gear_ratios = [3.5, 2.2, 1.5, 1.0]
		5:
			_gear_ratios = [3.8, 2.4, 1.7, 1.3, 1.0]
		6:
			_gear_ratios = [4.0, 2.5, 1.8, 1.4, 1.1, 0.9]
		_:
			_gear_ratios = [3.5, 2.2, 1.5, 1.0, 0.8, 0.7]

func _reset_wheel_states() -> void:
	"""Initialize wheel state dictionaries"""
	var wheel_defaults = {
		"radius": wheel_radius,
		"suspension_compression": 0.0,
		"rotation_angle": 0.0,
		"slip_ratio": 0.0,
		"lateral_slip": 0.0,
		"is_in_contact": false,
		"ground_normal": Vector3.UP
	}
	
	_front_left_wheel = wheel_defaults.duplicate()
	_front_right_wheel = wheel_defaults.duplicate()
	_rear_left_wheel = wheel_defaults.duplicate()
	_rear_right_wheel = wheel_defaults.duplicate()

func _connect_input_signals(input_mgr: Node) -> void:
	"""Connect to input manager signals for real-time control"""
	input_mgr.input_changed.connect(_on_input_changed)

func _on_input_changed(action: String, value: float, pressed: bool) -> void:
	"""Handle input changes from InputManager"""
	match action:
		"throttle":
			if pressed or value > _throttle_input:
				_throttle_input = clamp(value, 0.0, 1.0)
		"brake":
			if pressed or value > _brake_input:
				_brake_input = clamp(value, 0.0, 1.0)
		"steer_left":
			if pressed:
				_steering_input = -1.0
			elif not pressed and _steering_input < 0:
				_steering_input = 0.0
		"steer_right":
			if pressed:
				_steering_input = 1.0
			elif not pressed and _steering_input > 0:
				_steering_input = 0.0
		"gear_up":
			if pressed:
				_shift_up()
		"gear_down":
			if pressed:
				_shift_down()
		"handbrake":
			if pressed:
				_brake_input = 1.0
			else:
				_brake_input = 0.0

func _physics_process(delta: float) -> void:
	"""Main physics update loop - runs at fixed timestep"""
	_update_state_from_input()
	_update_engine_and_transmission(delta)
	_apply_forces_to_wheels(delta)
	_update_vehicle_velocity(delta)
	_update_drift_mechanics(delta)
	_check_collisions(delta)
	_notify_signals()

func _update_state_from_input() -> void:
	"""Update internal state flags from input values"""
	is_throttling = _throttle_input > 0.05
	is_braking = _brake_input > 0.05
	
	# Smooth steering input
	_steering_angle = lerp(_steering_angle, _steering_input * _physics.max_steering_angle, delta * 10.0)

func _update_engine_and_transmission(delta: float) -> void:
	"""Update engine RPM and transmission state"""
	var target_rpm: float = _calculate_target_rpm()
	current_rpm = lerp(current_rpm, target_rpm, delta * 5.0)
	
	# Clamp RPM within valid range
	current_rpm = clamp(current_rpm, idle_rpm, redline_rpm)
	
	# Auto-shift logic when reaching redline
	if current_rpm >= redline_rpm - 200.0 and current_gear > 0 and current_gear < gears_count:
		_shift_up(true)
	elif current_rpm < idle_rpm * 1.2 and current_gear > 1:
		_shift_down(true)

func _calculate_target_rpm() -> float:
	"""Calculate target RPM based on gear and throttle input"""
	if current_gear == 0:
		return idle_rpm
	
	var gear_ratio: float = _get_current_gear_ratio()
	var wheel_rotational_speed: float = abs(current_speed) / (wheel_radius * gear_ratio * _final_drive_ratio)
	
	var target_rpm: float = wheel_rotational_speed * _final_drive_ratio * gear_ratio
	
	if is_throttling:
		target_rpm *= 1.1 + (_throttle_input * 0.4)
	else:
		target_rpm *= 0.8 + (_brake_input * 0.2)
	
	return target_rpm

func _apply_forces_to_wheels(delta: float) -> void:
	"""Apply engine force and brake force to wheels"""
	_calculate_engine_force()
	_calculate_brake_force()
	
	# Apply forces to rear wheels (RWD configuration)
	_apply_force_to_wheel(_rear_left_wheel, _engine_force * 0.5, _brake_force)
	_apply_force_to_wheel(_rear_right_wheel, _engine_force * 0.5, _brake_force)
	
	# Front wheels steer
	_front_left_wheel["rotation_angle"] = _steering_angle
	_front_right_wheel["rotation_angle"] = -_steering_angle

func _calculate_engine_force() -> void:
	"""Calculate engine force based on RPM, gear, and throttle"""
	if current_gear == 0:
		_engine_force = 0.0
		return
	
	var gear_ratio: float = _get_current_gear_ratio()
	var torque_curve_factor: float = _calculate_torque_curve_factor()
	
	# Engine torque at current RPM
	var engine_torque: float = max_torque * torque_curve_factor
	var wheel_torque: float = engine_torque * gear_ratio * _final_drive_ratio
	
	# Convert to force (F = T / r)
	var wheel_radius_effective: float = wheel_radius * 0.8  # Account for drivetrain losses
	_engine_force = (wheel_torque / wheel_radius_effective) * 10.0
	
	# Apply throttle multiplier
	_engine_force *= (0.5 + (_throttle_input * 0.5))
	
	# Cap force based on gear
	_engine_force = min(_engine_force, max_engine_power / (current_speed + 1.0))

func _calculate_torque_curve_factor() -> float:
	"""Calculate torque curve factor based on RPM relative to peak torque RPM"""
	var peak_torque_rpm: float = idle_rpm + (redline_rpm - idle_rpm) * 0.4
	var rpm_normalized: float = (current_rpm - idle_rpm) / (peak_torque_rpm - idle_rpm)
	
	# Simple bell curve approximation
	var torque_factor: float = 1.0 - pow(rpm_normalized - 0.5, 2) * 4.0
	torque_factor = clamp(torque_factor, 0.3, 1.0)
	
	return torque_factor

func _calculate_brake_force() -> void:
	"""Calculate braking force based on brake input"""
	if not is_braking:
		_brake_force = 0.0
		return
	
	# Maximum brake force based on vehicle mass and physics settings
	var max_brake_force: float = vehicle_mass * _physics.gravity * 2.0
	_brake_force = max_brake_force * _brake_input
	
	# ABS simulation - reduce force at high speeds to prevent lockup
	if current_speed > 20.0:
		_brake_force *= 0.8

func _apply_force_to_wheel(wheel_data: Dictionary, drive_force: float, brake_force: float) -> void:
	"""Apply combined drive and brake forces to a single wheel"""
	wheel_data["drive_force"] = drive_force
	wheel_data["brake_force"] = brake_force
	
	# Calculate slip ratio
	var wheel_linear_speed: float = drive_force > 0 ? wheel_data.get("velocity", 0.0) : 0.0
	var angular_velocity: float = wheel_linear_speed / wheel_radius if wheel_radius > 0 else 0.0
	
	wheel_data["slip_ratio"] = (angular_velocity - (wheel_linear_speed / wheel_radius)) / max(abs(wheel_linear_speed), 0.1)

func _update_vehicle_velocity(delta: float) -> void:
	"""Update vehicle velocity based on applied forces"""
	# Get local velocity
	var local_velocity: Vector3 = velocity
	
	# Apply acceleration in vehicle direction
	var forward_direction: Vector3 = transform.basis.z
	var lateral_direction: Vector3 = transform.basis.x
	
	# Acceleration from engine force
	var acceleration: float = _engine_force / vehicle_mass
	local_velocity += forward_direction * acceleration * delta
	
	# Deceleration from brakes
	var deceleration: float = _brake_force / vehicle_mass
	local_velocity -= forward_direction * deceleration * delta
	
	# Aerodynamic drag
	var drag_coefficient: float = 0.35
	var air_density: float = 1.225
	var frontal_area: float = 2.2
	var drag_force: float = 0.5 * air_density * drag_coefficient * frontal_area * pow(local_velocity.length(), 2)
	var drag_acceleration: float = drag_force / vehicle_mass
	
	if local_velocity.length() > 1.0:
		var drag_direction: Vector3 = -local_velocity.normalized()
		local_velocity += drag_direction * drag_acceleration * delta
	
	# Update body velocity
	velocity = local_velocity
	
	# Update speed state
	current_speed = abs(velocity.length())

func _update_drift_mechanics(delta: float) -> void:
	"""Update drift mechanics including lateral slip and recovery"""
	var lateral_velocity: float = abs(velocity.dot(transform.basis.x))
	
	# Check if drifting
	is_drifting = lateral_velocity > _drift_threshold and is_throttling
	
	if is_drifting:
		# Increase drift angle during drift
		if _drift_angle < 0.8:
			_drift_angle += delta * 0.5
	else:
		# Recover from drift
		_drift_angle = lerp(_drift_angle, 0.0, delta * _drift_recovery_rate)
	
	# Clamp drift angle
	_drift_angle = clamp(_drift_angle, -0.8, 0.8)

func _check_collisions(delta: float) -> void:
	"""Check for collisions and handle impact forces"""
	# This would be expanded with actual collision detection
	# For now, placeholder for integration with Godot's collision system
	pass

func _notify_signals() -> void:
	"""Emit signals for state changes"""
	if abs(speed_changed.emit(current_speed)) > 0:
		pass
	if abs(rpm_changed.emit(current_rpm)) > 0:
		pass

func _shift_up(auto: bool = false) -> void:
	"""Shift transmission up one gear"""
	if current_gear >= gears_count or current_gear <= 0:
		return
	
	var previous_gear: int = current_gear
	current_gear += 1
	
	gear_changed.emit(current_gear)
	
	if auto:
		# Gentle downshift RPM reduction
		current_rpm = lerp(current_rpm, _calculate_target_rpm(), 0.3)

func _shift_down(auto: bool = false) -> void:
	"""Shift transmission down one gear"""
	if current_gear <= 1 or current_gear >= gears_count:
		return
	
	var previous_gear: int = current_gear
	current_gear -= 1
	
	gear_changed.emit(current_gear)
	
	if auto:
		# Rev match on downshift
		var target_rpm: float = _calculate_target_rpm()
		current_rpm = lerp(current_rpm, target_rpm * 1.3, 0.2)  # Overshoot for rev match

func _get_current_gear_ratio() -> float:
	"""Get the gear ratio for the current gear"""
	if current_gear == 0:
		return 1.0
	
	return _gear_ratios[current_gear - 1]

func get_wheel_positions() -> Dictionary:
	"""Get world positions of all four wheels for rendering"""
	var offset_x: float = front_wheel_track_width * 0.5
	var offset_y: float = wheel_base * 0.5
	
	return {
		"front_left": position + transform.basis.x * offset_x - transform.basis.z * offset_y,
		"front_right": position + transform.basis.x * -offset_x - transform.basis.z * offset_y,
		"rear_left": position + transform.basis.x * offset_x + transform.basis.z * offset_y,
		"rear_right": position + transform.basis.x * -offset_x + transform.basis.z * offset_y
	}

func reset_vehicle() -> void:
	"""Reset vehicle to initial state"""
	current_gear = 0
	current_rpm = idle_rpm
	current_speed = 0.0
	_throttle_input = 0.0
	_brake_input = 0.0
	_steering_input = 0.0
	_drift_angle = 0.0
	is_drifting = false
	_engine_force = 0.0
	_brake_force = 0.0

func debug_get_state() -> Dictionary:
	"""Return current vehicle state for debugging"""
	return {
		"speed": current_speed,
		"rpm": current_rpm,
		"gear": current_gear,
		"throttle": _throttle_input,
		"brake": _brake_input,
		"steering": _steering_input,
		"drifting": is_drifting,
		"drift_angle": _drift_angle,
		"engine_force": _engine_force,
		"brake_force": _brake_force,
		"wheel_states": {
			"front_left": _front_left_wheel,
			"front_right": _front_right_wheel,
			"rear_left": _rear_left_wheel,
			"rear_right": _rear_right_wheel
		}
	}

</FILE>