extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulation
## Handles throttle, brake, steering, wheel forces, gear shifting, and vehicle dynamics
## Integrates with PhysicsSettings, Powertrain, and InputManager systems
## Copyright 2026 Thalamus Racing Simulator Project

signal speed_changed(current_speed: float)
signal rpm_changed(rpm: float)
signal gear_changed(old_gear: int, new_gear: int)
signal drift_started()
signal drift_ended()
signal collision_detected(collision_info: Dictionary)
signal lap_completed(lap_data: Dictionary)
signal race_finished(position: int, time: float)
signal engine_stalled()

# ============================================================================
# INPUT VALUES (from InputManager)
# ============================================================================

@export var _throttle_input: float = 0.0: set = _set_throttle_input
@export var _brake_input: float = 0.0: set = _set_brake_input
@export var _steering_input: float = 0.0: set = _set_steering_input
@export var _clutch_input: float = 0.0: set = _set_clutch_input
@export var _handbrake_input: float = 0.0: set = _set_handbrake_input

var _last_throttle: float = 0.0
var _last_brake: float = 0.0
var _last_steering: float = 0.0

# ============================================================================
# VEHICLE PHYSICS STATE
# ============================================================================

var current_speed: float = 0.0  # Speed in m/s
var max_speed: float = 65.0  # Max forward speed m/s
var reverse_speed: float = 20.0  # Max reverse speed m/s
var acceleration: float = 0.0
var deceleration: float = 0.0

var rotation_velocity: float = 0.0  # Yaw rate rad/s
var slip_angle: float = 0.0  # Tire slip angle
var lateral_acceleration: float = 0.0  # G-force lateral
var longitudinal_acceleration: float = 0.0  # G-force longitudinal

# ============================================================================
# GEARBOX SYSTEM
# ============================================================================

var current_gear: int = 0  # 0=Neutral, -1=Reverse, 1-6=Gears
var target_gear: int = 0
var rpm: float = 0.0  # Engine RPM
var redline_rpm: float = 7500.0
var idle_rpm: float = 800.0
var max_rpm: float = 8500.0

var _gear_ratios: Array[float] = [0.0, 4.5, 3.0, 2.0, 1.4, 1.1, 0.9]  # Neutral + gears 1-6
var _reverse_ratio: float = 4.8
var final_drive_ratio: float = 3.73
var tire_radius: float = 0.33  # ~33cm radius tires

# ============================================================================
# POWERTRAIN INTEGRATION
# ============================================================================

var powertrain: Node = null
var torque_curve: Array[float] = []
var power_curve: Array[float] = []

func _ready() -> void:
	_init_powertrain()
	_load_torque_curve()
	_connect_signals()
	_set_process_mode(ProcessModeEnum.ALWAYS)

func _init_powertrain() -> void:
	if powertrain == null:
		powertrain = get_parent().find_child("Powertrain", false, true)
		if powertrain != null:
			powertrain.set_owner(self)

func _load_torque_curve() -> void:
	torque_curve = [
		0.0, 0.3, 0.5, 0.65, 0.78, 0.85, 0.92, 0.95, 0.90, 0.80, 0.65, 0.50, 0.35, 0.20, 0.05, 0.0
	]
	
	# Build power curve from torque * RPM
	var total_steps = torque_curve.size()
	for i in range(total_steps):
		var normalized_rpm = float(i) / (total_steps - 1)
		var torque_factor = torque_curve[i]
		var power_factor = torque_factor * normalized_rpm
		power_curve.append(power_factor)

func _connect_signals() -> void:
	if GameManager:
		GameManager.game_state_changed.connect(_on_game_state_changed)

func _on_game_state_changed(new_state: GameState) -> void:
	match new_state:
		GameManager.GameState.RACE_ACTIVE:
			_start_vehicle()
		GameManager.GameState.RACE_PAUSED:
			_pause_vehicle()
		GameManager.GameState.MAIN_MENU:
			_reset_vehicle()

func _start_vehicle() -> void:
	current_gear = 1
	target_gear = 1
	rpm = idle_rpm

func _pause_vehicle() -> void:
	pass

func _reset_vehicle() -> void:
	current_speed = 0.0
	rpm = idle_rpm
	current_gear = 0
	target_gear = 0

# ============================================================================
# MAIN PHYSICS UPDATE
# ============================================================================

func _physics_process(delta: float) -> void:
	_update_inputs(delta)
	_update_physics(delta)
	_update_drift(delta)
	_update_collision_detection(delta)
	_update_audio()

func _update_inputs(delta: float) -> void:
	_last_throttle = _throttle_input
	_last_brake = _brake_input
	_last_steering = _steering_input
	
	# Clamp inputs
	_throttle_input = clampf(_throttle_input, 0.0, 1.0)
	_brake_input = clampf(_brake_input, 0.0, 1.0)
	_steering_input = clampf(_steering_input, -1.0, 1.0)
	_clutch_input = clampf(_clutch_input, 0.0, 1.0)
	_handbrake_input = clampf(_handbrake_input, 0.0, 1.0)

func _update_physics(delta: float) -> void:
	_update_rpm(delta)
	_update_gear_shifting(delta)
	_update_force_application(delta)
	_update_velocity_and_position(delta)

func _update_rpm(delta: float) -> void:
	if current_gear == 0:
		# Neutral - idle or rev matching
		if _throttle_input > 0.0:
			rpm += (_throttle_input * 2000.0) * delta
		else:
			rpm = lerp(rpm, idle_rpm, delta * 5.0)
		return
	
	var wheel_speed = abs(current_speed) / tire_radius
	var drive_ratio = _get_current_drive_ratio()
	var engine_rpm_from_wheel = wheel_speed * drive_ratio * final_drive_ratio
	
	if current_gear > 0:
		rpm = lerp(rpm, engine_rpm_from_wheel, delta * 10.0)
	else:
		rpm = lerp(rpm, engine_rpm_from_wheel * 1.1, delta * 10.0)  # Slight overspeed in reverse
	
	rpm = clampf(rpm, idle_rpm, max_rpm)
	emit_signal("rpm_changed", rpm)

func _get_current_drive_ratio() -> float:
	if current_gear == -1:
		return _reverse_ratio
	elif current_gear <= 0:
		return 0.0
	return _gear_ratios[current_gear]

func _update_gear_shifting(delta: float) -> void:
	# Simple automatic transmission logic
	if current_gear == 0:
		if _throttle_input > 0.1:
			target_gear = 1
		return
	
	if target_gear == current_gear:
		# Determine target gear based on RPM
		var ratio = _get_current_drive_ratio()
		var wheel_rpm = abs(current_speed) / tire_radius
		
		if rpm > redline_rpm * 0.9 and target_gear < 6:
			target_gear += 1
		elif rpm < idle_rpm * 1.2 and target_gear > 1:
			target_gear -= 1
		elif current_speed < 5.0 and target_gear > 1:
			target_gear = 1
	
	# Gear change with clutch
	if current_gear != target_gear:
		var shift_delay = 0.15 + _clutch_input * 0.2
		_try_shift_gear(shift_delay)

func _try_shift_gear(delay: float) -> void:
	var old_gear = current_gear
	current_gear = target_gear
	emit_signal("gear_changed", old_gear, current_gear)
	
	# Downshift rev-match
	if current_gear < old_gear:
		rpm = lerp(rpm, idle_rpm * 2.0, 0.3)

func _update_force_application(delta: float) -> void:
	var drive_ratio = _get_current_drive_ratio()
	var torque_multiplier = _calculate_torque_multiplier()
	var applied_torque = torque_multiplier * drive_ratio
	
	# Apply force based on gear and input
	if current_gear == 0:
		acceleration = 0.0
		deceleration = PhysicsSettings.gravity * 0.02
		return
	
	if current_gear > 0:
		var max_torque = 400.0 * torque_multiplier
		var engine_force = max_torque * _throttle_input * (1.0 - (ramp_speed_to_max()))
		
		# Aerodynamic drag
		var drag_coefficient = 0.35
		var air_density = 1.225
		var frontal_area = 2.2
		var drag_force = 0.5 * air_density * drag_coefficient * frontal_area * current_speed * current_speed
		
		# Rolling resistance
		var rolling_resistance = 0.015 * PhysicsSettings.default_vehicle_mass * PhysicsSettings.gravity
		
		# Net force
		var net_force = engine_force - drag_force - rolling_resistance
		
		acceleration = net_force / PhysicsSettings.default_vehicle_mass
		
		# Cap speed
		if current_speed >= max_speed:
			acceleration = -deceleration
	elif current_gear == -1:
		var reverse_force = 200.0 * _brake_input
		var drag_force = 0.5 * air_density * drag_coefficient * frontal_area * current_speed * current_speed
		var rolling_resistance = 0.015 * PhysicsSettings.default_vehicle_mass * PhysicsSettings.gravity
		var net_force = reverse_force - drag_force - rolling_resistance
		acceleration = net_force / PhysicsSettings.default_vehicle_mass
		max_speed = reverse_speed

func _calculate_torque_multiplier() -> float:
	var normalized_rpm = (rpm - idle_rpm) / (redline_rpm - idle_rpm)
	normalized_rpm = clampf(normalized_rpm, 0.0, 1.0)
	
	var index = int(normalized_rpm * (torque_curve.size() - 1))
	index = clamp(index, 0, torque_curve.size() - 1)
	
	return torque_curve[index]

func ramp_speed_to_max() -> float:
	if max_speed <= 0:
		return 0.0
	return min(abs(current_speed) / max_speed, 1.0)

func _update_velocity_and_position(delta: float) -> void:
	# Apply acceleration
	if current_gear != 0:
		velocity.x += acceleration * delta * 100.0
	else:
		velocity.x -= PhysicsSettings.gravity * 0.02 * delta * 100.0
	
	# Apply friction
	velocity.x *= 0.998
	
	# Update speed
	current_speed = velocity.x
	
	# Clamp speeds
	if current_gear > 0:
		current_speed = clampf(current_speed, -max_speed, max_speed)
	elif current_gear == -1:
		current_speed = clampf(current_speed, -reverse_speed, reverse_speed)
	else:
		current_speed = 0.0
	
	velocity.x = current_speed
	
	# Calculate accelerations
	longitudinal_acceleration = (current_speed - _last_speed) / delta if delta > 0 else 0.0
	_last_speed = current_speed
	
	emit_signal("speed_changed", current_speed)

func _update_drift(delta: float) -> void:
	# Drift mechanics using handbrake and throttle balance
	var is_drifting = _handbrake_input > 0.5 and abs(current_speed) > 10.0
	
	if is_drifting and not _is_drifting:
		_is_drifting = true
		emit_signal("drift_started")
		_drift_timer = 0.0
	elif not is_drifting and _is_drifting:
		_is_drifting = false
		emit_signal("drift_ended")
		_drift_timer = 0.0
	
	if _is_drifting:
		_drift_timer += delta
		_slip_angle = lerp(_slip_angle, _steering_input * 0.8, delta * 5.0)
		lateral_acceleration = abs(velocity.y * rotation_velocity) / 9.81
	else:
		_slip_angle = lerp(_slip_angle, 0.0, delta * 10.0)
		lateral_acceleration = 0.0

func _update_collision_detection(delta: float) -> void:
	if is_on_colliding():
		var collision = get_collider()
		var collision_normal = get_collision_normal()
		
		collision_info = {
			"collider": collision,
			"normal": collision_normal,
			"position": global_position,
			"speed_at_impact": current_speed,
			"time": Time.get_unix_time_from_system()
		}
		
		emit_signal("collision_detected", collision_info)
		
		# Bounce effect
		var bounce_force = collision_normal * current_speed * 0.5
		velocity += bounce_force

func _update_audio() -> void:
	if AudioManager:
		var audio_type = AudioType.IDLE
		if current_gear == 0:
			audio_type = AudioType.IDLE
		elif abs(current_speed) < 5.0:
			audio_type = AudioType.LOW_SPEED
		elif abs(current_speed) < 20.0:
			audio_type = AudioType.MEDIUM_SPEED
		else:
			audio_type = AudioType.HIGH_SPEED
		
		if rpm > redline_rpm * 0.9:
			audio_type = AudioType.REDLINE
		
		AudioManager.play_sound(audio_type, rpm / max_rpm)

# ============================================================================
# HELPER METHODS
# ============================================================================

func change_gear(gear: int) -> void:
	target_gear = gear
	if gear == 0:
		current_gear = 0
		rpm = idle_rpm
	elif gear >= 1 and gear <= 6:
		current_gear = gear
	elif gear == -1:
		current_gear = -1

func reset_gearbox() -> void:
	current_gear = 0
	target_gear = 0
	rpm = idle_rpm

func apply_brake(force: float) -> void:
	var braking_force = force * PhysicsSettings.default_vehicle_mass * 0.8
	velocity.x -= braking_force * _brake_input * 0.1

func reset_vehicle_state() -> void:
	current_speed = 0.0
	velocity.x = 0.0
	rpm = idle_rpm
	current_gear = 0
	target_gear = 0
	_slip_angle = 0.0
	_is_drifting = false

# ============================================================================
# SIGNAL HANDLERS
# ============================================================================

func _on_game_state_changed(new_state: GameState) -> void:
	match new_state:
		GameManager.GameState.RACE_STARTED:
			_start_vehicle()
		GameManager.GameState.RACE_ENDED:
			_reset_vehicle()

# ============================================================================
# INTERNAL VARIABLES
# ============================================================================

var _is_drifting: bool = false
var _drift_timer: float = 0.0
var _last_speed: float = 0.0
var collision_info: Dictionary = {}
var _input_delay: float = 0.0

enum AudioType {
	IDLE,
	LOW_SPEED,
	MEDIUM_SPEED,
	HIGH_SPEED,
	REDLINE,
	GEAR_CHANGE,
	COLLISION
}

</file_content_truncated_here>