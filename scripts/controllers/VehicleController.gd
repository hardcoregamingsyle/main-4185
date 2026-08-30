extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulator
## Implements throttle, brake, steering inputs, wheel forces, and gear shifting logic
## Uses PhysicsSettings constants for centralized tuning
## Copyright 2026 Thalamus Racing Simulator Project

# ============================================================================
# SIGNALS
# ============================================================================
signal speed_changed(new_speed: float)
signal rpm_changed(new_rpm: float)
signal gear_changed(old_gear: int, new_gear: int)
signal throttle_applied(amount: float)
signal brake_applied(amount: float)
signal steering_angle_changed(angle: float)
signal skidding(is_skidding: bool)
signal collision_detected(collision_info: Dictionary)
signal engine_stalled()
signal handbrake_toggled(is_active: bool)
signal traction_control_state_changed(active: bool)
signal anti_lock_braking_state_changed(active: bool)
signal drift_started(drift_angle: float)
signal drift_ended()

# ============================================================================
# CONFIGURATION CONSTANTS
# ============================================================================
const MAX_SPEED_KMH: float = 350.0
const ACCELERATION_POWER: float = 20000.0
const BRAKING_FORCE: float = 40000.0
const STEERING_SPEED: float = 2.5
const MAX_STEERING_ANGLE: float = 35.0 * TAU / 180.0
const MIN_GEAR: int = -1  # Reverse
const MAX_GEAR: int = 6
const NEUTRAL_GEAR: int = 0
const IDLE_RPM: float = 800.0
const REDLINE_RPM: float = 8000.0
const CLUTCH_RELEASE_TIME: float = 0.3
const DIFFERENTIAL_LOCK_RATIO: float = 0.8
const DRIFT_FACTOR: float = 0.15
const TRACTION_CONTROL_THRESHOLD: float = 0.15
const ABS_THRESHOLD: float = 0.1
const SHIFT_POINT_RPM: float = 7000.0
const SHUTDOWN_RPM: float = 9000.0
const GEAR_SHIFT_DELAY: float = 0.1
const ENGINE_REVO_DROP_ON_DOWNSHIFT: float = 2000.0

# ============================================================================
# STATE VARIABLES
# ============================================================================
var _speed_kmh: float = 0.0
var _rpm: float = IDLE_RPM
var _current_gear: int = NEUTRAL_GEAR
var _target_gear: int = NEUTRAL_GEAR
var _throttle_input: float = 0.0
var _brake_input: float = 0.0
var _steering_input: float = 0.0
var _handbrake_input: float = 0.0
var _clutch_input: float = 1.0

var _is_engine_running: bool = false
var _is_clutch_engaged: bool = true
var _is_handbrake_active: bool = false
var _is_skidding: bool = false
var _is_drifting: bool = false
var _is_in_shift: bool = false

var _gear_ratios: Array[float] = [0.0, 3.5, 2.5, 1.8, 1.4, 1.1, 0.9, 0.7]
var _final_drive_ratio: float = 3.5
var _wheel_base: float = 2.7
var _track_width: float = 1.6
var _wheel_radius: float = 0.33
var _vehicle_mass: float = 1500.0

var _last_position: Vector3 = Vector3.ZERO
var _velocity_direction: Vector3 = Vector3.ZERO
var _angular_velocity: float = 0.0
var _drift_angle: float = 0.0

var _traction_control_enabled: bool = true
var _abs_enabled: bool = true
var _diff_lock_enabled: bool = false

var _shift_timer: float = 0.0
var _engine_temperature: float = 90.0
var _fuel_level: float = 100.0
var _tire_wear: float = 0.0

var _physics_settings: PhysicsSettings = null
var _powertrain_node: Node = null

# ============================================================================
# LATERAL WHEEL FORCES (for raycast joints or custom wheels)
# ============================================================================
var _front_left_wheel_force: Vector3 = Vector3.ZERO
var _front_right_wheel_force: Vector3 = Vector3.ZERO
var _rear_left_wheel_force: Vector3 = Vector3.ZERO
var _rear_right_wheel_force: Vector3 = Vector3.ZERO

# ============================================================================
# INPUT BINDINGS (configured by InputManager)
# ============================================================================
const INPUT_THROTTLE: String = "ui_throttle"
const INPUT_BRAKE: String = "ui_brake"
const INPUT_CLUTCH: String = "ui_clutch"
const INPUT_STEERING_LEFT: String = "ui_steering_left"
const INPUT_STEERING_RIGHT: String = "ui_steering_right"
const INPUT_GEAR_UP: String = "ui_gear_up"
const INPUT_GEAR_DOWN: String = "ui_gear_down"
const INPUT_HANDBRAKE: String = "ui_handbrake"
const INPUT_ENGINE_START: String = "ui_engine_start"

# ============================================================================
# GODOT METHODS
# ============================================================================
func _ready() -> void:
	_init_physics_settings()
	_connect_signals()
	_reset_vehicle_state()
	_update_visuals()

func _physics_process(delta: float) -> void:
	if not _is_engine_running:
		return
	
	var actual_delta := delta if delta > 0 else 0.016
	
	_handle_inputs(actual_delta)
	_calculate_rpm_and_power(actual_delta)
	_handle_gear_shifting(actual_delta)
	_update_vehicle_velocity(actual_delta)
	_check_traction_and_skid(actual_delta)
	_update_drift_state(actual_delta)
	_apply_forces_to_body(actual_delta)
	_update_simulation_vars(actual_delta)
	_update_visuals()

func _input(event: InputEvent) -> void:
	if event.is_action_pressed(INPUT_ENGINE_START):
		_toggle_engine()

# ============================================================================
# PUBLIC INTERFACE
# ============================================================================
func start_engine() -> void:
	_is_engine_running = true
	_rpm = IDLE_RPM
	emit_signal("engine_started")

func stop_engine() -> void:
	_is_engine_running = false
	_rpm = IDLE_RPM
	emit_signal("engine_stopped")

func toggle_engine() -> void:
	_toggle_engine()

func set_throttle(amount: float) -> void:
	_throttle_input = clampf(amount, 0.0, 1.0)
	emit_signal("throttle_applied", _throttle_input)

func set_brake(amount: float) -> void:
	_brake_input = clampf(amount, 0.0, 1.0)
	emit_signal("brake_applied", _brake_input)

func set_clutch(amount: float) -> void:
	_clutch_input = clampf(amount, 0.0, 1.0)
	_is_clutch_engaged = _clutch_input >= 1.0

func set_steering(angle: float) -> void:
	_steering_input = clampf(angle, -1.0, 1.0)
	var actual_angle := _steering_input * MAX_STEERING_ANGLE
	emit_signal("steering_angle_changed", abs(actual_angle))

func set_handbrake(active: bool) -> void:
	_handbrake_input = 1.0 if active else 0.0
	_is_handbrake_active = active
	emit_signal("handbrake_toggled", active)

func shift_gear(gear: int) -> void:
	if gear < MIN_GEAR or gear > MAX_GEAR:
		return
	
	_current_gear = gear
	emit_signal("gear_changed", gear, _current_gear)

func auto_shift_gear() -> void:
	if _is_in_shift:
		return
	
	if _rpm > SHIFT_POINT_RPM and _current_gear < MAX_GEAR:
		_target_gear = _current_gear + 1
	elif _rpm < 2000 and _current_gear > MIN_GEAR and _current_gear != NEUTRAL_GEAR:
		_target_gear = _current_gear - 1
	else:
		_target_gear = NEUTRAL_GEAR
	
	if _target_gear != _current_gear:
		_start_shift()

func get_speed_kmh() -> float:
	return _speed_kmh

func get_rpm() -> float:
	return _rpm

func get_current_gear() -> int:
	return _current_gear

func is_engine_running() -> bool:
	return _is_engine_running

func is_skidding() -> bool:
	return _is_skidding

func is_drifting() -> bool:
	return _is_drifting

func get_fuel_level() -> float:
	return _fuel_level

func get_tire_wear() -> float:
	return _tire_wear

func reset_vehicle() -> void:
	_reset_vehicle_state()

# ============================================================================
# INTERNAL METHODS - INPUT HANDLING
# ============================================================================
func _handle_inputs(delta: float) -> void:
	# Apply input values (already clamped by setters)
	pass

func _calculate_rpm_and_power(delta: float) -> void:
	if not _is_engine_running:
		_rpm = IDLE_RPM
		return
	
	var target_rpm := _get_target_rpm_from_gear()
	
	# Engine acceleration based on throttle and gear ratio
	var gear_ratio := _gear_ratios[_current_gear] if _current_gear >= 0 and _current_gear < _gear_ratios.size() else 0.0
	var power_factor := _throttle_input * (1.0 - (_rpm / REDLINE_RPM))
	
	if _is_clutch_engaged and _current_gear != NEUTRAL_GEAR:
		# Engine RPM responds to throttle
		var rpm_change := power_factor * ACCELERATION_POWER * delta * 0.001
		_rpm = min(_rpm + rpm_change, target_rpm + 500)
		
		# Engine braking when no throttle
		if _throttle_input < 0.1:
			_rpm = max(_rpm - rpm_change * 0.5, target_rpm - 500)
	else:
		# Clutch disengaged - RPM decays slowly
		_rpm = max(_rpm - delta * 100, IDLE_RPM)
	
	# Clamp RPM
	_rpm = clampf(_rpm, IDLE_RPM, SHUTDOWN_RPM)
	
	# ECU protection at redline
	if _rpm > SHUTDOWN_RPM:
		_rpm = SHUTDOWN_RPM
		_rpm -= delta * 1000
	
	emit_signal("rpm_changed", _rpm)

func _get_target_rpm_from_gear() -> float:
	if _current_gear == NEUTRAL_GEAR:
		return IDLE_RPM
	
	var gear_ratio := _gear_ratios[_current_gear]
	var speed_ms := _speed_kmh / 3.6
	
	# Calculate expected RPM based on wheel speed
	var wheel_rps := speed_ms / (2.0 * TAU * _wheel_radius)
	var engine_rps := wheel_rps * gear_ratio * _final_drive_ratio
	return engine_rps * 60.0

# ============================================================================
# INTERNAL METHODS - GEAR SHIFTING
# ============================================================================
func _start_shift() -> void:
	_is_in_shift = true
	_shift_timer = GEAR_SHIFT_DELAY
	
	if _target_gear != NEUTRAL_GEAR and _current_gear != NEUTRAL_GEAR:
		# Downshift rev matching
		if _target_gear < _current_gear:
			_rpm += ENGINE_REVO_DROP_ON_DOWNSHIFT
	
	_rpm = max(_rpm - 2000, IDLE_RPM)

func _handle_gear_shifting(delta: float) -> void:
	if _is_in_shift:
		_shift_timer -= delta
		if _shift_timer <= 0:
			_complete_shift()
		return
	
	if Input.is_action_just_pressed(INPUT_GEAR_UP):
		if _current_gear < MAX_GEAR:
			_target_gear = _current_gear + 1
			_start_shift()
	
	if Input.is_action_just_pressed(INPUT_GEAR_DOWN):
		if _current_gear > MIN_GEAR:
			_target_gear = _current_gear - 1
			_start_shift()

func _complete_shift() -> void:
	_current_gear = _target_gear
	_is_in_shift = false
	_rpm = IDLE_RPM + 500
	emit_signal("gear_changed", _target_gear, _current_gear)

# ============================================================================
# INTERNAL METHODS - VEHICLE PHYSICS
# ============================================================================
func _update_vehicle_velocity(delta: float) -> void:
	_last_position = global_transform.origin
	
	var gear_ratio := _gear_ratios[_current_gear] if _current_gear >= 0 and _current_gear < _gear_ratios.size() else 0.0
	
	# Calculate drive force
	var drive_force := 0.0
	if _is_engine_running and _is_clutch_engaged and _current_gear != NEUTRAL_GEAR:
		var engine_torque := _throttle_input * ACCELERATION_POWER * gear_ratio * 0.0001
		drive_force = engine_torque
		
		# Reduce force at high speeds
		if _speed_kmh > 100:
			drive_force *= 1.0 - (_speed_kmh - 100) / (MAX_SPEED_KMH - 100)
	
	# Apply drag
	var air_drag := 0.02 * _speed_kmh * _speed_kmh * delta * 0.001
	
	# Apply rolling resistance
	var rolling_resistance := 0.005 * _vehicle_mass * 9.81 * delta
	
	# Apply braking force
	var braking_force := 0.0
	if _brake_input > 0 or _is_handbrake_active:
		var brake_pressure := _brake_input if not _is_handbrake_active else _brake_input * 1.5
		braking_force = BRAKING_FORCE * brake_pressure * delta * 0.0001
	
	# Net force calculation
	var net_force := drive_force - air_drag - rolling_resistance - braking_force
	
	# Acceleration: F = ma => a = F/m
	var acceleration := net_force / _vehicle_mass
	
	# Update velocity in local forward direction
	var forward_dir := transform.basis.z
	var speed_change := acceleration * delta
	
	# Handle reverse gear
	if _current_gear == MIN_GEAR:
		speed_change *= -1
	
	# Apply speed change
	_speed_kmh += speed_change * 3.6  # Convert m/s to km/h
	_speed_kmh = clampf(_speed_kmh, -MAX_SPEED_KMH / 2, MAX_SPEED_KMH)
	
	# Update body velocity
	var new_velocity := forward_dir * (_speed_kmh / 3.6)
	
	# Add steering influence on lateral movement
	if abs(_steering_input) > 0.01 and _speed_kmh > 5:
		var steer_factor := _steering_input * 0.3 * sign(_speed_kmh)
		var lateral_move := transform.basis.x * steer_factor * abs(_speed_kmh) * delta * 0.01
		new_velocity += lateral_move
	
	velocity = new_velocity
	emit_signal("speed_changed", _speed_kmh)

func _check_traction_and_skid(delta: float) -> void:
	if _speed_kmh < 1:
		_is_skidding = false
		return
	
	var slip_ratio := _calculate_slip_ratio()
	
	if abs(slip_ratio) > TRACTION_CONTROL_THRESHOLD:
		_is_skidding = true
		
		if _traction_control_enabled:
			# Reduce throttle to regain traction
			_throttle_input *= 0.8
		else:
			# Allow more aggressive driving
			_throttle_input *= 1.1
	else:
		_is_skidding = false
	
	emit_signal("skidding", _is_skidding)

func _calculate_slip_ratio() -> float:
	if _current_gear == NEUTRAL_GEAR or _speed_kmh < 1:
		return 0.0
	
	var gear_ratio := _gear_ratios[_current_gear]
	var wheel_linear_speed := _speed_kmh / 3.6
	var wheel_rotational_speed := _rpm / 60.0 * 2.0 * TAU * _wheel_radius * gear_ratio * _final_drive_ratio
	
	if wheel_rotational_speed == 0:
		return 0.0
	
	return (wheel_rotational_speed - wheel_linear_speed) / wheel_linear_speed

func _update_drift_state(delta: float) -> void:
	var drift_threshold := 0.3
	var drift_recovery := 0.95
	
	if _is_drifting:
		_drift_angle += delta * 10
		if _drift_angle > PI / 2:
			_drift_angle = PI / 2
		
		if not _is_handbrake_active and _speed_kmh > 50:
			_drift_angle *= drift_recovery
			if abs(_drift_angle) < 0.1:
				_end_drift()
	else:
		# Check if we should enter drift
		if _is_handbrake_active and abs(_steering_input) > 0.5 and _speed_kmh > 30:
			_start_drift()
		elif abs(_drift_angle) > 0.1:
			_drift_angle *= drift_recovery
			if abs(_drift_angle) < 0.05:
				_drift_angle = 0.0

func _start_drift() -> void:
	_is_drifting = true
	emit_signal("drift_started", _drift_angle)

func _end_drift() -> void:
	_is_drifting = false
	_drift_angle = 0.0
	emit_signal("drift_ended()")

func _apply_forces_to_body(delta: float) -> void:
	# Apply calculated forces to the rigid body
	var forward_force := Vector3.FORWARD * (_speed_kmh / 3.6) * _vehicle_mass * delta * 0.001
	forward_force.y = 0
	
	# Add gravitational component if on slope
	var gravity := Vector3.DOWN * _physics_settings.gravity * _vehicle_mass
	velocity.y = -gravity.y / _vehicle_mass * delta
	
	# Apply velocity
	global_transform.origin += velocity * delta

func _update_simulation_vars(delta: float) -> void:
	# Engine temperature simulation
	if _rpm > 6000:
		_engine_temperature += delta * 0.5
	else:
		_engine_temperature -= delta * 0.3
	_engine_temperature = clampf(_engine_temperature, 60.0, 120.0)
	
	# Fuel consumption
	if _throttle_input > 0:
		_fuel_level -= delta * _throttle_input * 0.01
	_fuel_level = clampf(_fuel_level, 0.0, 100.0)
	
	# Tire wear increases with skidding and drifting
	if _is_skidding or _is_drifting:
		_tire_wear += delta * 0.001
	_tire_wear = min(_tire_wear, 1.0)

# ============================================================================
# INTERNAL METHODS - VISUALS AND UTILITIES
# ============================================================================
func _update_visuals() -> void:
	# Update visual indicators (headlights, taillights, brake lights, etc.)
	pass

func _reset_vehicle_state() -> void:
	_speed_kmh = 0.0
	_rpm = IDLE_RPM
	_current_gear = NEUTRAL_GEAR
	_throttle_input = 0.0
	_brake_input = 0.0
	_steering_input = 0.0
	_handbrake_input = 0.0
	_clutch_input = 1.0
	_is_engine_running = false
	_is_clutch_engaged = true
	_is_handbrake_active = false
	_is_skidding = false
	_is_drifting = false
	_is_in_shift = false
	_drift_angle = 0.0
	_tractor_control_enabled = true
	_abs_enabled = true
	_diff_lock_enabled = false
	_fuel_level = 100.0
	_tire_wear = 0.0
	_engine_temperature = 90.0

func _init_physics_settings() -> void:
	_physics_settings = PhysicsSettings.new()
	if _physics_settings:
		_vehicle_mass = _physics_settings.default_vehicle_mass

func _connect_signals() -> void:
	# Connect to GameManager signals if needed
	if GameManager:
		GameManager.race_started.connect(_on_race_started)
		GameManager.game_state_changed.connect(_on_game_state_changed)

func _on_race_started(race_data: Dictionary) -> void:
	# Initialize vehicle for race
	start_engine()
	set_gear(1)

func _on_game_state_changed(new_state: GameManager.GameState) -> void:
	match new_state:
		GameManager.GameState.RACE_ACTIVE:
			start_engine()
		GameManager.GameState.RACE_FINISHED:
			stop_engine()

func _toggle_engine() -> void:
	if _is_engine_running:
		stop_engine()
	else:
		start_engine()

func set_gear(gear: int) -> void:
	shift_gear(gear)

func get_engine_temperature() -> float:
	return _engine_temperature

func is_overheating() -> bool:
	return _engine_temperature > 110.0

func needs_refueling() -> bool:
	return _fuel_level < 15.0

func needs_pit_stop() -> bool:
	return _tire_wear > 0.8 or _engine_temperature > 115.0 or _fuel_level < 10.0

</script>