extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for the racing simulator
## Handles throttle, brake, steering inputs, wheel forces, and gear shifting logic
## Uses PhysicsSettings for all tunable constants
## Copyright 2026 Thalamus Racing Simulator Project

# ============================================================================
# SIGNALS
# ============================================================================

signal speed_changed(new_speed: float)
signal gear_changed(old_gear: int, new_gear: int)
signal engine_rpm_changed(rpm: float)
signal vehicle_damage_taken(damage_amount: float)
signal skid_detected(skid_intensity: float)

# ============================================================================
# CONSTANTS (loaded from PhysicsSettings at runtime)
# ============================================================================

var _physics: PhysicsSettings
var _vehicle_mass: float = 1500.0
var _drag_coefficient: float = 0.35
var _rolling_resistance: float = 0.015
var _max_steering_angle: float = DEG_TO_RAD * 35.0
var _steering_sensitivity: float = 1.0

# ============================================================================
# PHYSICS PROPERTIES
# ============================================================================

# Speed properties
var current_speed: float = 0.0: set = _set_current_speed
var max_forward_speed: float = 0.0
var max_reverse_speed: float = 0.0
var acceleration_rate: float = 0.0
var braking_force: float = 0.0
var drag_factor: float = 0.0

# Engine properties
var engine_rpm: float = 0.0
var idle_rpm: float = 800.0
var redline_rpm: float = 7000.0
var torque_curve: Array[float] = []
var power_band_min: float = 2500.0
var power_band_max: float = 6000.0

# Transmission properties
var current_gear: int = 1
var total_gears: int = 6
var gear_ratios: Array[float] = [4.0, 2.5, 1.7, 1.3, 1.0, 0.8]
var reverse_ratio: float = 4.5
var final_drive_ratio: float = 3.5
var tire_radius: float = 0.32

# Steering properties
var steering_input: float = 0.0: set = _set_steering_input
var current_steering_angle: float = 0.0
var steering_speed: float = 15.0

# Drive system properties
var drive_type: DriveType = DriveType.FWD
var front_bias: float = 0.0
var rear_bias: float = 1.0
var differential_type: DifferentialType = DifferentialType.OPEN
var lockup_ratio: float = 0.2

# Wheel friction coefficients
var grip_front: float = 1.2
var grip_rear: float = 1.3
var drift_threshold: float = 0.85

# ============================================================================
# INPUT STATE
# ============================================================================

var throttle_input: float = 0.0: set = _set_throttle_input
var brake_input: float = 0.0: set = _set_brake_input
var clutch_input: float = 0.0: set = _set_clutch_input
var handbrake_input: bool = false

# ============================================================================
# INTERNAL STATE
# ============================================================================

var _is_engine_running: bool = true
var _clutch_engaged: bool = true
var _drifting: bool = false
var _skid_intensity: float = 0.0
var _last_velocity_direction: Vector3 = Vector3.ZERO
var _engine_braking_active: bool = false
var _turbo_boost: float = 0.0
var _turbo_charge: float = 0.0
var _fuel_level: float = 100.0
var _fuel_consumption_rate: float = 0.0

# ============================================================================
# ENUMERATIONS
# ============================================================================

enum DriveType { FWD, RWD, AWD }
enum DifferentialType { OPEN, LSD, LOCKED }

# ============================================================================
# INITIALIZATION
# ============================================================================

func _ready() -> void:
	_init_physics_settings()
	_init_powertrain()
	_connect_signals()
	_calculate_vehicle_properties()

func _init_physics_settings() -> void:
	_physics = GameManager.get_node_or_null("/root/PhysicsSettings")
	if _physics == null:
		_physics = preload("res://scripts/core/PhysicsSettings.gd").new()
	_physics.connect("changed", _on_physics_settings_changed)

func _connect_signals() -> void:
	GameManager.game_state_changed.connect(_on_game_state_changed)

func _init_powertrain() -> void:
	# Initialize default powertrain values
	torque_curve = [0.0, 0.3, 0.6, 0.9, 1.0, 0.95, 0.85, 0.75]
	
# ============================================================================
# VEHICLE PROPERTIES CALCULATION
# ============================================================================

func _calculate_vehicle_properties() -> void:
	# Calculate max speeds based on gear ratios and engine characteristics
	max_forward_speed = _calculate_max_forward_speed()
	max_reverse_speed = abs(_calculate_max_reverse_speed())
	
	acceleration_rate = _physics.default_vehicle_mass / 1500.0 * 3.5
	braking_force = 9.81 * _physics.default_vehicle_mass * 1.2
	drag_factor = _drag_coefficient * 0.5 * 1.225 * 2.2 * 0.35

func _calculate_max_forward_speed() -> float:
	var top_gear_ratio = gear_ratios[total_gears - 1]
	var wheel_max_rps = redline_rpm / 60.0 / (top_gear_ratio * final_drive_ratio)
	return wheel_max_rps * 2.0 * PI * tire_radius * 3.6 # Convert to km/h
	return 280.0 # Default fallback

func _calculate_max_reverse_speed() -> float:
	var wheel_max_rps = idle_rpm / 60.0 / (reverse_ratio * final_drive_ratio)
	return wheel_max_rps * 2.0 * PI * tire_radius * 3.6

# ============================================================================
# MAIN PROCESS
# ============================================================================

func _physics_process(delta: float) -> void:
	if not _is_engine_running and current_speed > 1.0:
		_apply_friction(delta)
		return
	
	_update_inputs(delta)
	_update_engine(delta)
	_update_transmission(delta)
	_update_drivetrain(delta)
	_update_steering(delta)
	_update_aerodynamics(delta)
	_update_skid_detection(delta)
	_update_fuel_system(delta)

func _input(event: InputEvent) -> void:
	if event is InputEventKey:
		_handle_keyboard_input(event)
	elif event is InputEventMouseButton:
		_handle_mouse_input(event)

func _handle_keyboard_input(event: InputEventKey) -> void:
	if event.pressed:
		match event.keycode:
			KEY_W, KEY_UP:
				throttle_input = min(throttle_input + 0.05, 1.0)
			KEY_S, KEY_DOWN:
				throttle_input = max(throttle_input - 0.05, -0.5)
			KEY_A:
				steering_input = min(steering_input + 0.05, 1.0)
			KEY_D:
				steering_input = max(steering_input - 0.05, -1.0)
			KEY_SPACE:
				handbrake_input = true
			KEY_F:
				_shift_up()
			KEY_G:
				_shift_down()
			KEY_R:
				_reset_vehicle()

func _handle_mouse_input(event: InputEventMouseButton) -> void:
	if event.button_index == MOUSE_BUTTON_LEFT and event.pressed:
		_shift_up()
	elif event.button_index == MOUSE_BUTTON_RIGHT and event.pressed:
		_shift_down()

# ============================================================================
# INPUT UPDATES
# ============================================================================

func _update_inputs(delta: float) -> void:
	# Smooth input transitions
	throttle_input = lerp(throttle_input, InputManager.get_action_strength("throttle"), delta * 5.0)
	brake_input = lerp(brake_input, InputManager.get_action_strength("brake"), delta * 5.0)
	clutch_input = lerp(clutch_input, InputManager.get_action_strength("clutch"), delta * 5.0)
	steering_input = lerp(steering_input, InputManager.get_axis("steer_left", "steer_right"), delta * 10.0)
	handbrake_input = InputManager.is_action_just_pressed("handbrake")

# ============================================================================
# ENGINE SYSTEM
# ============================================================================

func _update_engine(delta: float) -> void:
	if not _is_engine_running:
		engine_rpm = idle_rpm
		return
	
	var target_rpm = _calculate_target_rpm()
	engine_rpm = lerp(engine_rpm, target_rpm, delta * 10.0)
	engine_rpm = clamp(engine_rpm, idle_rpm, redline_rpm)
	
	# Clamp RPM at redline with rev limiter behavior
	if engine_rpm >= redline_rpm:
		engine_rpm = redline_rpm
		throttle_input = max(throttle_input - 0.02, 0.0)
	
	emit_signal("engine_rpm_changed", engine_rpm)

func _calculate_target_rpm() -> float:
	var gear_ratio = current_gear > 0 ? gear_ratios[current_gear - 1] : reverse_ratio
	var wheel_rps = current_speed * 1000.0 / 3600.0 / (2.0 * PI * tire_radius)
	var base_rpm = wheel_rps * gear_ratio * final_drive_ratio * 60.0
	
	if _clutch_engaged:
		return base_rpm
	else:
		return idle_rpm + (base_rpm - idle_rpm) * _clutch_input

# ============================================================================
# TRANSMISSION SYSTEM
# ============================================================================

func _update_transmission(delta: float) -> void:
	_auto_shift_if_needed(delta)
	_handle_manual_shifts(delta)

func _auto_shift_if_needed(delta: float) -> void:
	if InputManager.is_action_pressed("auto_shift"):
		var shift_threshold = 0.95 if current_gear < total_gears else 0.1
		if engine_rpm > redline_rpm * shift_threshold and current_gear < total_gears:
			_shift_up()
		elif engine_rpm < idle_rpm * 1.5 and current_gear > 1:
			_shift_down()

func _handle_manual_shifts(delta: float) -> void:
	if InputManager.is_action_just_pressed("shift_up"):
		_shift_up()
	elif InputManager.is_action_just_pressed("shift_down"):
		_shift_down()

func _shift_up() -> void:
	if current_gear < total_gears:
		var old_gear = current_gear
		current_gear += 1
		emit_signal("gear_changed", old_gear, current_gear)
		_calculate_vehicle_properties()

func _shift_down() -> void:
	if current_gear > 1:
		var old_gear = current_gear
		current_gear -= 1
		emit_signal("gear_changed", old_gear, current_gear)
		_calculate_vehicle_properties()

func reset_gear() -> void:
	current_gear = 1
	emit_signal("gear_changed", total_gears, 1)

# ============================================================================
# DRIVE SYSTEM
# ============================================================================

func _update_drivetrain(delta: float) -> void:
	var wheel_torque = _calculate_wheel_torque()
	var traction_loss = _calculate_traction_loss()
	
	# Apply forces to vehicle based on drive type
	match drive_type:
		DriveType.FWD:
			_apply_drive_force(wheel_torque * (1.0 - traction_loss), front_bias)
		DriveType.RWD:
			_apply_drive_force(wheel_torque * (1.0 - traction_loss), rear_bias)
		DriveType.AWD:
			_apply_drive_force(wheel_torque * (1.0 - traction_loss), 0.5)
			_apply_drive_force(wheel_torque * (1.0 - traction_loss) * 0.5, 0.5)

func _calculate_wheel_torque() -> float:
	if not _is_engine_running or not _clutch_engaged:
		return 0.0
	
	var rpm_normalized = (engine_rpm - idle_rpm) / (redline_rpm - idle_rpm)
	rpm_normalized = clamp(rpm_normalized, 0.0, 1.0)
	
	var torque_multiplier = _get_torque_at_rpm(rpm_normalized)
	var gear_ratio = current_gear > 0 ? gear_ratios[current_gear - 1] : reverse_ratio
	
	var wheel_torque = throttle_input * _physics.default_vehicle_mass * 0.5 * gear_ratio * final_drive_ratio * torque_multiplier
	
	# Turbo boost effect
	if _turbo_charge > 0:
		wheel_torque *= (1.0 + _turbo_boost)
	
	return wheel_torque

func _get_torque_at_rpm(normalized_rpm: float) -> float:
	var index = int(normalized_rpm * (torque_curve.size() - 1))
	index = clamp(index, 0, torque_curve.size() - 2)
	var t1 = torque_curve[index]
	var t2 = torque_curve[index + 1]
	var alpha = normalized_rpm * (torque_curve.size() - 1) - index
	return t1 * (1.0 - alpha) + t2 * alpha

func _apply_drive_force(force: float, distribution: float) -> void:
	if force > 0:
		velocity.x += force * 0.01 * distribution
	elif force < 0:
		velocity.x -= abs(force) * 0.01 * distribution

# ============================================================================
# STEERING SYSTEM
# ============================================================================

func _update_steering(delta: float) -> void:
	var target_steering = steering_input * _max_steering_angle
	current_steering_angle = lerp(current_steering_angle, target_steering, delta * steering_speed)
	
	# Auto-center steering when no input
	if abs(steering_input) < 0.05:
		current_steering_angle = lerp(current_steering_angle, 0.0, delta * 15.0)

# ============================================================================
# AERODYNAMICS
# ============================================================================

func _update_aerodynamics(delta: float) -> void:
	var air_density = 1.225
	var frontal_area = 2.2
	var velocity_mps = current_speed * 1000.0 / 3600.0
	
	var drag_force = 0.5 * air_density * _drag_coefficient * frontal_area * velocity_mps * velocity_mps
	
	if velocity_mps > 0:
		velocity.x -= sign(velocity.x) * drag_force * delta * 0.01

# ============================================================================
# SKID DETECTION
# ============================================================================

func _update_skid_detection(delta: float) -> void:
	var velocity_magnitude = velocity.length()
	var last_direction = _last_velocity_direction.normalized()
	var current_direction = velocity.normalized()
	
	var direction_change = last_direction.angle_to(current_direction)
	
	if abs(direction_change) > 0.1 or abs(velocity_magnitude - _last_velocity_direction.length()) > 5.0:
		_drifting = true
		_skid_intensity = min(abs(direction_change) * 2.0, 1.0)
		
		if _skid_intensity > drift_threshold:
			emit_signal("skid_detected", _skid_intensity)
	else:
		_drifting = false
		_skid_intensity = 0.0
	
	_last_velocity_direction = velocity

# ============================================================================
# FUEL SYSTEM
# ============================================================================

func _update_fuel_system(delta: float) -> void:
	if _is_engine_running and throttle_input > 0.1:
		var consumption = delta * _fuel_consumption_rate * throttle_input
		_fuel_level = max(0.0, _fuel_level - consumption)
		
		if _fuel_level <= 0:
			_is_engine_running = false
			current_speed = 0.0

# ============================================================================
# FRICTION AND DRAG
# ============================================================================

func _apply_friction(delta: float) -> void:
	var friction_coefficient = _rolling_resistance
	var deceleration = friction_coefficient * _physics.gravity * delta
	
	if abs(velocity.x) > 0:
		velocity.x -= sign(velocity.x) * deceleration
		if abs(velocity.x) < 0.1:
			velocity.x = 0.0

# ============================================================================
# POWER OUTPUT
# ============================================================================

func _calculate_power_output() -> float:
	var rpm_normalized = (engine_rpm - idle_rpm) / (redline_rpm - idle_rpm)
	rpm_normalized = clamp(rpm_normalized, 0.0, 1.0)
	
	var torque = _get_torque_at_rpm(rpm_normalized) * _physics.default_vehicle_mass * 0.5
	var angular_velocity = engine_rpm * PI / 30.0
	
	return torque * angular_velocity / 745.7 # Convert to horsepower

# ============================================================================
# DAMAGE SYSTEM
# ============================================================================

func take_damage(damage_amount: float) -> void:
	if damage_amount <= 0:
		return
	
	_fuel_level = max(0.0, _fuel_level - damage_amount * 0.1)
	_is_engine_running = damage_amount < 50.0
	
	if not _is_engine_running:
		current_speed = 0.0
	
	emit_signal("vehicle_damage_taken", damage_amount)

# ============================================================================
# RESET FUNCTIONALITY
# ============================================================================

func reset_vehicle() -> void:
	current_speed = 0.0
	velocity = Vector3.ZERO
	current_gear = 1
	engine_rpm = idle_rpm
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	_fuel_level = 100.0
	_is_engine_running = true
	reset_gear()

# ============================================================================
# HELPER METHODS
# ============================================================================

func get_current_gear_ratio() -> float:
	return current_gear > 0 ? gear_ratios[current_gear - 1] : reverse_ratio

func get_efficiency_rating() -> float:
	if current_speed <= 0:
		return 0.0
	return (_fuel_level / 100.0) * (1.0 - abs(current_speed - 60.0) / 60.0)

func simulate_turbo_response(input_delta: float) -> void:
	_turbo_charge = min(_turbo_charge + input_delta * 0.5, 1.0)
	_turbo_boost = _turbo_charge * 0.3

# ============================================================================
# SIGNAL HANDLERS
# ============================================================================

func _on_physics_settings_changed() -> void:
	_calculate_vehicle_properties()

func _on_game_state_changed(new_state: GameState) -> void:
	match new_state:
		GameState.LOADING:
			pass
		GameState.RACE_ACTIVE:
			_is_engine_running = true
		GameState.RACE_FINISHED:
			_is_engine_running = false

# ============================================================================
# EXPOSED GETTERS
# ============================================================================

func get_vehicle_status() -> Dictionary:
	return {
		"speed": current_speed,
		"rpm": engine_rpm,
		"gear": current_gear,
		"throttle": throttle_input,
		"brake": brake_input,
		"fuel": _fuel_level,
		"driving": _is_engine_running,
		"drifting": _drifting
	}

# ============================================================================
# SPEED SETTER
# ============================================================================

func _set_current_speed(value: float) -> void:
	current_speed = value
	emit_signal("speed_changed", current_speed)

</file_content>