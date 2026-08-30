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

var _is_engine_running: bool = false
var _clutch_engaged: bool = false
var _clutch_timer: float = 0.0
var _shift_timer: float = 0.0
var _last_collision_time: float = 0.0
var _collision_cooldown: float = 0.5

# Traction Control & ABS flags
var _traction_control_enabled: bool = true
var _abs_enabled: bool = true

# Drift state
var _drift_angle: float = 0.0
var _is_drifting: bool = false
var _drift_threshold: float = 0.3

# Wheel grip simulation
var _front_wheel_grip: float = 1.0
var _rear_wheel_grip: float = 1.0
var _skid_velocity: Vector3 = Vector3.ZERO

# Powertrain reference
var _powertrain: Node = null

# Physics settings reference
var _physics_settings: PhysicsSettings = null

# ============================================================================
# GEAR RATIOS AND FINAL DRIVE
# ============================================================================
var _gear_ratios: Array[float] = [3.5, 2.8, 2.1, 1.6, 1.3, 1.0, 0.8]
var _reverse_ratio: float = 3.8
var _final_drive_ratio: float = 3.73

# ============================================================================
# INITIALIZATION
# ============================================================================
func _ready() -> void:
	# Initialize references
	_physics_settings = PhysicsSettings.new()
	
	# Setup initial state
	_is_engine_running = false
	_current_gear = NEUTRAL_GEAR
	
	# Connect to global managers if available
	if GameManager.has_singleton():
		GameManager.game_state_changed.connect(_on_game_state_changed)
	
	if AudioManager.has_singleton():
		AudioManager.sound_played.connect(_on_sound_played)

# ============================================================================
# PUBLIC API
# ============================================================================
## Start the engine
func start_engine() -> void:
	if not _is_engine_running:
		_is_engine_running = true
		_rpm = IDLE_RPM
		_clutch_engaged = true
		
		if AudioManager.has_singleton():
			AudioManager._play_sfx("engine_start")

## Stop the engine
func stop_engine() -> void:
	if _is_engine_running:
		_is_engine_running = false
		_rpm = IDLE_RPM
		_current_gear = NEUTRAL_GEAR
		
		if AudioManager.has_singleton():
			AudioManager._play_sfx("engine_stop")

## Toggle engine on/off
func toggle_engine() -> void:
	if _is_engine_running:
		stop_engine()
	else:
		start_engine()

## Get current speed in km/h
func get_speed_kmh() -> float:
	return abs(_speed_kmh)

## Get current RPM
func get_rpm() -> float:
	return _rpm

## Get current gear
func get_gear() -> int:
	return _current_gear

## Get throttle input value
func get_throttle_input() -> float:
	return _throttle_input

## Get brake input value
func get_brake_input() -> float:
	return _brake_input

## Check if engine is running
func is_engine_running() -> bool:
	return _is_engine_running

## Enable/disable traction control
func set_traction_control(enabled: bool) -> void:
	_traction_control_enabled = enabled
	emit_signal("traction_control_state_changed", enabled)

## Enable/disable ABS
func set_abs(enabled: bool) -> void:
	_abs_enabled = enabled
	emit_signal("anti_lock_braking_state_changed", enabled)

## Shift gears
func shift_gear(gear: int) -> void:
	if gear < MIN_GEAR or gear > MAX_GEAR:
		return
		
	if _shift_timer > 0:
		return  # Still in shift delay
	
	_target_gear = gear
	_shift_timer = GEAR_SHIFT_DELAY
	
	# Handle reverse to forward transition
	if (_current_gear == -1 and gear > 0) or (_current_gear > 0 and gear == -1):
		# Need neutral first when changing direction
		_target_gear = NEUTRAL_GEAR
	else:
		_current_gear = gear
		emit_signal("gear_changed", gear, gear)
		
		if AudioManager.has_singleton():
			AudioManager._play_sfx("gear_shift")

## Manual upshift
func upshift() -> void:
	if _current_gear < MAX_GEAR:
		shift_gear(_current_gear + 1)

## Manual downshift
func downshift() -> void:
	if _current_gear > MIN_GEAR:
		shift_gear(_current_gear - 1)

## Auto-shift based on RPM
func auto_shift() -> void:
	if not _is_engine_running:
		return
	
	if _current_gear >= MAX_GEAR and _rpm > SHIFT_POINT_RPM:
		upshift()
	elif _current_gear <= MIN_GEAR and _rpm < IDLE_RPM:
		pass  # Already at lowest gear
	elif _current_gear > MIN_GEAR and _rpm < IDLE_RPM + 500:
		downshift()

## Apply throttle input
func apply_throttle(input_value: float) -> void:
	_throttle_input = clamp(input_value, 0.0, 1.0)
	emit_signal("throttle_applied", _throttle_input)

## Apply brake input
func apply_brake(input_value: float) -> void:
	_brake_input = clamp(input_value, 0.0, 1.0)
	emit_signal("brake_applied", _brake_input)

## Apply steering input
func apply_steering(input_value: float) -> void:
	_steering_input = clamp(input_value, -1.0, 1.0)
	emit_signal("steering_angle_changed", _steering_input)

## Apply handbrake
func apply_handbrake(input_value: float) -> void:
	_handbrake_input = clamp(input_value, 0.0, 1.0)
	emit_signal("handbrake_toggled", _handbrake_input > 0.1)

# ============================================================================
# PHYSICS UPDATE
# ============================================================================
func _physics_process(delta: float) -> void:
	if not is_engine_running():
		_rpm = IDLE_RPM
		return
	
	# Update shift timer
	if _shift_timer > 0:
		_shift_timer -= delta
		if _shift_timer <= 0:
			_current_gear = _target_gear
			emit_signal("gear_changed", _target_gear, _target_gear)

	# Calculate engine torque based on RPM and gear
	var engine_torque: float = _calculate_engine_torque()
	
	# Apply power to wheels
	var wheel_force: float = _apply_power_to_wheels(engine_torque, delta)
	
	# Handle braking
	var brake_force: float = _apply_braking(wheel_force, delta)
	
	# Calculate total acceleration
	var acceleration: float = _calculate_acceleration(wheel_force, brake_force, delta)
	
	# Update velocity
	_update_velocity(acceleration, delta)
	
	# Calculate RPM based on speed and gear
	_update_rpm()
	
	# Handle steering
	_apply_steering(delta)
	
	# Handle drift mechanics
	_update_drift(delta)
	
	# Emit signals
	emit_signal("speed_changed", _speed_kmh)
	emit_signal("rpm_changed", _rpm)

# ============================================================================
# CORE PHYSICS CALCULATIONS
# ============================================================================
## Calculate engine torque curve
func _calculate_engine_torque() -> float:
	var normalized_rpm: float = (_rpm - IDLE_RPM) / (REDLINE_RPM - IDLE_RPM)
	normalized_rpm = clamp(normalized_rpm, 0.0, 1.0)
	
	# Torque curve approximation (peaks around 4000-5000 RPM)
	var torque_curve: float = -2.0 * pow(normalized_rpm - 0.5, 2) + 0.5
	torque_curve = max(torque_curve, 0.0)
	
	var base_torque: float = 450.0  # Nm at peak
	return base_torque * torque_curve * _throttle_input

## Apply power to wheels considering gear ratio
func _apply_power_to_wheels(engine_torque: float, delta: float) -> float:
	if _current_gear == NEUTRAL_GEAR:
		return 0.0
	
	var gear_ratio: float = _get_gear_ratio(_current_gear)
	var total_ratio: float = gear_ratio * _final_drive_ratio
	
	# Force = torque * gear_ratio / wheel_radius
	var wheel_radius: float = 0.3  # meters
	var drive_force: float = (engine_torque * total_ratio) / wheel_radius
	
	# Distribute force across wheels (simplified RWD layout)
	var rear_wheel_force: float = drive_force * 0.6
	var front_wheel_force: float = drive_force * 0.4
	
	return rear_wheel_force

## Get gear ratio for current gear
func _get_gear_ratio(gear: int) -> float:
	if gear == NEUTRAL_GEAR:
		return 0.0
	elif gear < 0:
		return _reverse_ratio
	else:
		return _gear_ratios[min(gear - 1, _gear_ratios.size() - 1)]

## Apply braking force
func _apply_braking(drive_force: float, delta: float) -> float:
	var brake_force: float = 0.0
	
	if _brake_input > 0:
		var braking_effectiveness: float = 1.0
		if _abs_enabled and _speed_kmh > 5.0:
			braking_effectiveness = 1.2  # ABS improves braking efficiency
		brake_force = _brake_input * BRAKING_FORCE * braking_effectiveness
	
	if _handbrake_input > 0:
		var handbrake_force: float = _handbrake_input * BRAKING_FORCE * 0.7
		brake_force += handbrake_force
	
	return brake_force

## Calculate net acceleration
func _calculate_acceleration(wheel_force: float, brake_force: float, delta: float) -> float:
	var mass: float = 1500.0  # kg
	var net_force: float = wheel_force - brake_force
	
	var acceleration: float = net_force / mass
	
	# Apply traction control if active
	if _traction_control_enabled:
		acceleration = _apply_traction_control(acceleration)
	
	return acceleration

## Apply traction control limits
func _apply_traction_control(base_acceleration: float) -> float:
	if _speed_kmh < 5.0:
		return base_acceleration
	
	var slip_ratio: float = _calculate_slip_ratio()
	
	if slip_ratio > TRACTION_CONTROL_THRESHOLD:
		var reduction_factor: float = 1.0 - (slip_ratio - TRACTION_CONTROL_THRESHOLD) * 2.0
		reduction_factor = max(reduction_factor, 0.3)
		return base_acceleration * reduction_factor
	
	return base_acceleration

## Calculate wheel slip ratio
func _calculate_slip_ratio() -> float:
	if _speed_kmh < 1.0:
		return 0.0
	
	var wheel_circumference: float = 2.0 * PI * 0.3  # ~1.88m
	var wheel_rotations_per_second: float = _speed_kmh / 3.6 / wheel_circumference
	var engine_rotations_per_second: float = _rpm / 60.0
	var final_ratio: float = _get_gear_ratio(_current_gear) * _final_drive_ratio
	
	var theoretical_wheel_rps: float = engine_rotations_per_second * final_ratio
	var actual_wheel_rps: float = wheel_rotations_per_second
	
	if actual_wheel_rps == 0:
		return 0.0
	
	return abs(theoretical_wheel_rps - actual_wheel_rps) / actual_wheel_rps

# ============================================================================
# VELOCITY AND MOVEMENT
# ============================================================================
func _update_velocity(acceleration: float, delta: float) -> void:
	var current_speed_ms: float = _speed_kmh / 3.6
	
	# Apply acceleration
	current_speed_ms += acceleration * delta
	
	# Cap maximum speed
	var max_speed_ms: float = MAX_SPEED_KMH / 3.6
	current_speed_ms = min(current_speed_ms, max_speed_ms)
	
	# Ensure minimum negative speed (reverse)
	if _current_gear < 0:
		current_speed_ms = max(current_speed_ms, -max_speed_ms / 2)
	
	_speed_kmh = current_speed_ms * 3.6

func _apply_steering(delta: float) -> void:
	if _speed_kmh < 2.0:
		return  # No steering at very low speeds
	
	var target_angle: float = _steering_input * MAX_STEERING_ANGLE
	
	# Smooth steering transition
	var current_rotation: float = rotation.y
	var angle_diff: float = target_angle - current_rotation
	
	# Normalize angle difference
	while angle_diff > PI:
		angle_diff -= 2.0 * PI
	while angle_diff < -PI:
		angle_diff += 2.0 * PI
	
	# Apply steering with speed-based damping
	var steering_damping: float = 1.0 - (_speed_kmh / MAX_SPEED_KMH) * 0.5
	var steering_delta: float = angle_diff * STEERING_SPEED * delta * steering_damping
	
	current_rotation += steering_delta
	rotation.y = current_rotation

# ============================================================================
# RPM CALCULATION
# ============================================================================
func _update_rpm() -> void:
	if _current_gear == NEUTRAL_GEAR:
		_rpm = lerp(_rpm, IDLE_RPM, 0.1)
		return
	
	var wheel_rotational_speed: float = _speed_kmh / 3.6 / (2.0 * PI * 0.3)
	var final_drive_ratio: float = _get_gear_ratio(_current_gear) * _final_drive_ratio
	var calculated_rpm: float = wheel_rotational_speed * final_drive_ratio * 60.0
	
	# Smooth RPM transition
	_rpm = lerp(_rpm, calculated_rpm, 0.05)
	
	# Clamp to valid range
	_rpm = clamp(_rpm, IDLE_RPM, SHUTDOWN_RPM)
	
	# Redline protection
	if _rpm > SHUTDOWN_RPM:
		_rpm = SHUTDOWN_RPM

# ============================================================================
# DRIFT MECHANICS
# ============================================================================
func _update_drift(delta: float) -> void:
	if _speed_kmh < 20.0:
		_is_drifting = false
		return
	
	var lateral_acceleration: float = calculate_lateral_acceleration()
	
	# Drift threshold check
	if abs(lateral_acceleration) > _drift_threshold and _handbrake_input > 0.3:
		if not _is_drifting:
			_is_drifting = true
			_drift_angle = lateral_acceleration
			emit_signal("drift_started", _drift_angle)
		_drift_angle = lerp(_drift_angle, lateral_acceleration, 0.1)
	else:
		if _is_drifting:
			_is_drifting = false
			emit_signal("drift_ended")
			_drift_angle = 0.0

func calculate_lateral_acceleration() -> float:
	var velocity: Vector3 = velocity
	var forward: Vector3 = transform.basis.z.rotated(Vector3.UP, rotation.y)
	var lateral: Vector3 = forward.cross(Vector3.UP).normalized()
	
	return velocity.dot(lateral) * 9.81

# ============================================================================
# COLLISION HANDLING
# ============================================================================
func _process_collision(collision: CollisionShape3D) -> void:
	var now: float = Time.get_unix_time_from_system()
	if now - _last_collision_time < _collision_cooldown:
		return
	
	_last_collision_time = now
	
	var impact_speed: float = abs(_speed_kmh)
	var collision_data: Dictionary = {
		"timestamp": now,
		"impact_speed": impact_speed,
		"location": global_position
	}
	
	emit_signal("collision_detected", collision_data)
	
	if AudioManager.has_singleton():
		AudioManager._play_sfx("collision", impact_speed / MAX_SPEED_KMH)

# ============================================================================
# INPUT HANDLERS (Called from InputManager)
# ============================================================================
func handle_input_action(action: String, value: float) -> void:
	match action:
		"throttle":
			apply_throttle(value)
		"brake":
			apply_brake(value)
		"steering":
			apply_steering(value)
		"handbrake":
			apply_handbrake(value)
		"upshift":
			if value > 0.5:
				upshift()
		"downshift":
			if value > 0.5:
				downshift()
		"toggle_engine":
			if value > 0.5:
				toggle_engine()

# ============================================================================
# GAME STATE CALLBACKS
# ============================================================================
func _on_game_state_changed(new_state: GameState) -> void:
	match new_state:
		GameState.RACE_ACTIVE:
			if not _is_engine_running:
				start_engine()
		GameState.RACE_FINISHED, GameState.MAIN_MENU:
			stop_engine()

func _on_sound_played(sound_name: String) -> void:
	pass  # Can be extended for audio feedback

# ============================================================================
# DEBUG AND TESTING
# ============================================================================
func debug_get_stats() -> Dictionary:
	return {
		"speed_kmh": _speed_kmh,
		"rpm": _rpm,
		"gear": _current_gear,
		"throttle": _throttle_input,
		"brake": _brake_input,
		"steering": _steering_input,
		"handbrake": _handbrake_input,
		"engine_running": _is_engine_running,
		"drifting": _is_drifting,
		"traction_control": _traction_control_enabled,
		"abs": _abs_enabled
	}

func reset_vehicle() -> void:
	_speed_kmh = 0.0
	_rpm = IDLE_RPM
	_current_gear = NEUTRAL_GEAR
	_target_gear = NEUTRAL_GEAR
	_throttle_input = 0.0
	_brake_input = 0.0
	_steering_input = 0.0
	_handbrake_input = 0.0
	_is_drifting = false
	_drift_angle = 0.0
	_shift_timer = 0.0