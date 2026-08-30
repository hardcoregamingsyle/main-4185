extends Node2D
class_name VehicleController

## VehicleController - Core vehicle physics controller using PhysicsSettings constants
## Handles throttle, brake, steering inputs, wheel forces, and gear shifting logic
## Copyright 2026 Thalamus Racing Simulator Project

signal speed_changed(new_speed: float)
signal gear_changed(old_gear: int, new_gear: int)
signal vehicle_moved(displacement: Vector2)
signal drift_angle_changed(angle: float)
signal traction_control_active(active: bool)
signal engine_rpm_changed(rpm: float)
signal collision_detected(collision_type: String, force: float)

# ============================================================================
# PHYSICS CONSTANTS - Derived from PhysicsSettings resource
# ============================================================================

const MAX_THROTTLE_FORCE: float = 15000.0      # Newtons - maximum acceleration force
const MAX_BRAKE_FORCE: float = 20000.0         # Newtons - maximum braking force
const MAX_STEERING_ANGLE: float = PI / 3       # 60 degrees max steering
const STEERING_SPEED: float = 4.0              # Radians per second steering rate
const DRIFT_THRESHOLD: float = 0.7             # Sideslip threshold for drift mode
const TRACTION_CONTROL_SENSITIVITY: float = 0.85 # TCS activation threshold
const MIN_RPM_IDLE: float = 800.0              # Idle RPM
const MAX_RPM_REDLINE: float = 7500.0          # Redline RPM
const OPTIMAL_POWER_RPM_START: float = 3500.0  # Start of power band
const OPTIMAL_POWER_RPM_END: float = 6500.0    # End of power band

# ============================================================================
# GEAR RATIOS AND TRANSMISSION CONFIGURATION
# ============================================================================

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

const GEAR_RATIOS: Dictionary = {
    Gear.FIRST: 3.5,
    Gear.SECOND: 2.2,
    Gear.THIRD: 1.6,
    Gear.FOURTH: 1.2,
    Gear.FIFTH: 0.9,
    Gear.SIXTH: 0.75,
    Gear.REVERSE: 3.0
}

const FINAL_DRIVE_RATIO: float = 3.73          # Final drive differential ratio
const REVERSE_GEAR_RATIO: float = 3.5          # Reverse gear ratio

# ============================================================================
# VEHICLE STATE VARIABLES
# ============================================================================

@export var vehicle_mass: float = 1500.0        # kg
@export var wheelbase: float = 2.5              # meters
@export var track_width: float = 1.5            # meters
@export var center_of_gravity_height: float = 0.5  # meters above ground

var current_speed: float = 0.0                   # m/s (positive forward, negative reverse)
var current_rpm: float = MIN_RPM_IDLE
var current_gear: int = Gear.NEUTRAL
var target_gear: int = Gear.NEUTRAL
var steering_angle: float = 0.0                  # radians
var target_steering_angle: float = 0.0           # radians

var throttle_input: float = 0.0                  # -1.0 (full brake) to 1.0 (full throttle)
var brake_input: float = 0.0                     # 0.0 to 1.0
var clutch_input: float = 1.0                    # 0.0 (disengaged) to 1.0 (engaged)

var engine_braking: bool = false
var traction_control_enabled: bool = true
var abs_enabled: bool = true
var diff_locked: bool = false

var is_drifting: bool = false
var drift_score: float = 0.0
var drift_timer: float = 0.0

var _last_position: Vector2 = Vector2.ZERO
var _velocity_vector: Vector2 = Vector2.ZERO
var _angular_velocity: float = 0.0
var _acceleration: float = 0.0

var _engine_force: float = 0.0
var _brake_force: float = 0.0
var _steering_force: float = 0.0

var _gear_shift_timer: float = 0.0
var _clutch_reengage_delay: float = 0.5
var _reengage_timer: float = 0.0

var _power_curve: Array[float] = []            # Pre-computed power curve by RPM
var _torque_curve: Array[float] = []           # Pre-computed torque curve by RPM

# ============================================================================
# POWERTRAIN REFERENCE
# ============================================================================

var _powertrain_ref: Node = null

# ============================================================================
# INITIALIZATION
# ============================================================================

func _ready() -> void:
	_init_power_curves()
	_connect_signals()
	_update_state()

func _init_power_curves() -> void:
	"""Pre-compute engine power and torque curves for performance lookup"""
	_power_curve.resize(100)
	_torque_curve.resize(100)
	
	var rpm_step: float = (MAX_RPM_REDLINE - MIN_RPM_IDLE) / 99.0
	
	for i in range(100):
		var rpm: float = MIN_RPM_IDLE + (i * rpm_step)
		var normalized_rpm: float = (rpm - MIN_RPM_IDLE) / (MAX_RPM_REDLINE - MIN_RPM_IDLE)
		
		# Simulated power curve (bell-shaped with peak around optimal power RPM)
		var power_peak_ratio: float = exp(-((normalized_rpm - 0.6) ** 2) * 10.0)
		var max_power: float = 300000.0  # 300kW peak power
		_power_curve[i] = max_power * power_peak_ratio
		
		# Simulated torque curve (peaks at lower RPM than power)
		var torque_peak_ratio: float = exp(-((normalized_rpm - 0.4) ** 2) * 15.0)
		var max_torque: float = 600.0   # 600Nm peak torque
		_torque_curve[i] = max_torque * torque_peak_ratio

func _connect_signals() -> void:
	if GameManager:
		GameManager.game_state_changed.connect(_on_game_state_changed)

func _on_game_state_changed(new_state: GameManager.GameState) -> void:
	match new_state:
		GameManager.GameState.RACE_ACTIVE:
			_resumed_simulation()
		GameManager.GameState.RACE_PAUSED:
			_paused_simulation()
		GameManager.GameState.MAIN_MENU:
			_reset_vehicle()

# ============================================================================
# INPUT HANDLING
# ============================================================================

func _physics_process(delta: float) -> void:
	_handle_inputs(delta)
	_update_gear_shifting(delta)
	_calculate_engine_output(delta)
	_apply_forces(delta)
	_update_vehicle_state(delta)

func _handle_inputs(delta: float) -> void:
	"""Process raw input and clamp values to valid ranges"""
	throttle_input = clamp(throttle_input, -1.0, 1.0)
	brake_input = clamp(brake_input, 0.0, 1.0)
	clutch_input = clamp(clutch_input, 0.0, 1.0)
	
	# Smooth steering interpolation
	target_steering_angle = lerp(target_steering_angle, steering_angle, delta * STEERING_SPEED)
	steering_angle = lerp(steering_angle, target_steering_angle, delta * STEERING_SPEED)

func set_throttle(value: float) -> void:
	throttle_input = value

func set_brake(value: float) -> void:
	brake_input = value

func set_steering(value: float) -> void:
	target_steering_angle = value

func set_clutch(value: float) -> void:
	clutch_input = value

func shift_up() -> void:
	if current_gear < Gear.SIXTH and current_gear != Gear.NEUTRAL:
		target_gear = current_gear + 1
		_attempt_gear_shift()

func shift_down() -> void:
	if current_gear > Gear.FIRST and current_gear != Gear.NEUTRAL:
		target_gear = current_gear - 1
		_attempt_gear_shift()
	elif current_gear == Gear.FIRST and current_gear != Gear.NEUTRAL:
		target_gear = Gear.NEUTRAL
		_attempt_gear_shift()

func set_gear(gear: int) -> void:
	target_gear = gear
	_attempt_gear_shift()

func toggle_traction_control() -> void:
	traction_control_enabled = not traction_control_enabled
	_emit_traction_control_status()

func toggle_abs() -> void:
	abs_enabled = not abs_enabled

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================

func _attempt_gear_shift() -> void:
	if target_gear == current_gress or target_gear == current_gear:
		return
	
	_gear_shift_timer = _clutch_reengage_delay
	current_gear = target_gear
	gear_changed.emit(current_gear - 1, current_gear)

func _update_gear_shifting(delta: float) -> void:
	if _gear_shift_timer > 0.0:
		_gear_shift_timer -= delta
		if _gear_shift_timer <= 0.0:
			_reengage_clutch()

func _reengage_clutch() -> void:
	clutch_input = 1.0
	if _powertrain_ref:
		_powertrain_ref.clutch_engaged()

func get_current_gear() -> int:
	return current_gear

func get_target_gear() -> int:
	return target_gear

func get_rpm_at_speed(speed: float) -> float:
	"""Calculate engine RPM based on vehicle speed and current gear"""
	if current_gear == Gear.NEUTRAL:
		return MIN_RPM_IDLE
	
	var wheel_speed: float = speed / 0.3  # Assume 0.3m wheel radius
	var transmission_ratio: float = GEAR_RATIOS[current_gear] if current_gear in GEAR_RATIOS else 1.0
	var final_ratio: float = FINAL_DRIVE_RATIO
	var clutch_engaged: float = clutch_input
	
	var engine_rpm: float = wheel_speed * transmission_ratio * final_ratio * 60.0 * clutch_engaged
	engine_rpm = lerp(engine_rpm, MIN_RPM_IDLE, 1.0 - clutch_engaged)
	
	return clamp(engine_rpm, MIN_RPM_IDLE, MAX_RPM_REDLINE)

# ============================================================================
# ENGINE OUTPUT CALCULATION
# ============================================================================

func _calculate_engine_output(delta: float) -> void:
	"""Calculate engine force based on throttle, gear, and RPM"""
	current_rpm = get_rpm_at_speed(current_speed)
	current_rpm = clamp(current_rpm, MIN_RPM_IDLE, MAX_RPM_REDLINE)
	
	# Find closest index in pre-computed power/torque curves
	var rpm_normalized: float = (current_rpm - MIN_RPM_IDLE) / (MAX_RPM_REDLINE - MIN_RPM_IDLE)
	var curve_index: int = int(rpm_normalized * 99.0)
	curve_index = clamp(curve_index, 0, 99)
	
	var available_torque: float = _torque_curve[curve_index]
	var available_power: float = _power_curve[curve_index]
	
	# Apply throttle/clutch modifiers
	var effective_throttle: float = throttle_input * clutch_input
	effective_throttle = max(effective_throttle, 0.0)
	
	# Engine braking when no throttle and moving
	if not engine_braking and current_speed != 0.0 and throttle_input <= 0.0 and clutch_input > 0.0:
		effective_throttle = 0.0
	
	# Calculate output force
	_varies_by_gear: bool = current_gear != Gear.NEUTRAL and current_gear != Gear.REVERSE
	if varies_by_gear:
		var gear_ratio: float = GEAR_RATIOS[current_gear] if current_gear in GEAR_RATIOS else 1.0
		var drive_force: float = available_torque * gear_ratio * FINAL_DRIVE_RATIO
		drive_force *= effective_throttle
		_engine_force = sign(current_speed) * min(drive_force, MAX_THROTTLE_FORCE)
	else:
		_engine_force = 0.0
	
	# Brake force calculation
	_brake_force = brake_input * MAX_BRAKE_FORCE * clutch_input
	
	# Clamp brake force when moving in wrong direction
	if current_speed > 0.0 and brake_input > 0.0 and current_gear != Gear.REVERSE:
		_brake_force = min(_brake_force, MAX_BRAKE_FORCE)
	elif current_speed < 0.0 and brake_input > 0.0 and current_gear == Gear.REVERSE:
		_brake_force = min(_brake_force, MAX_BRAKE_FORCE)

func get_engine_force() -> float:
	return _engine_force

func get_brake_force() -> float:
	return _brake_force

# ============================================================================
# FORCE APPLICATION AND VEHICLE PHYSICS
# ============================================================================

func _apply_forces(delta: float) -> void:
	"""Apply calculated forces to vehicle body"""
	# Update position based on velocity
	var displacement: Vector2 = _velocity_vector * delta
	position += displacement
	
	# Calculate angular velocity from steering
	var steering_effectiveness: float = current_speed / 10.0 + 0.1
	_angular_velocity = steering_angle * STEERING_SPEED * steering_effectiveness
	
	# Apply drift physics
	if _check_drift_conditions():
		_is_drifting = true
		_calculate_drift_score(delta)
	else:
		_is_drifting = false
		_drift_score = max(0.0, _drift_score - delta * 10.0)

func _check_drift_conditions() -> bool:
	"""Check if vehicle is in drift state"""
	if current_speed < 5.0:  # Need minimum speed to drift
		return false
	
	var sideslip_angle: float = abs(_velocity_vector.angle())
	return sideslip_angle > DRIFT_THRESHOLD

func _calculate_drift_score(delta: float) -> void:
	"""Calculate drift score for scoring system"""
	var drift_intensity: float = 1.0 + (_drift_score * 0.5)
	_drift_score += delta * drift_intensity
	_drift_timer += delta
	
	if _drift_timer >= 3.0:  # 3 seconds minimum for drift bonus
		emit_signal("vehicle_moved", Vector2.ZERO)

# ============================================================================
# STATE UPDATE AND UTILITIES
# ============================================================================

func _update_vehicle_state(delta: float) -> void:
	"""Update vehicle state and emit signals"""
	_last_position = position
	
	# Normalize speed for signal emission
	var normalized_speed: float = current_speed * 3.6  # Convert to km/h
	speed_changed.emit(normalized_speed)
	
	# Emit RPM change
	engine_rpm_changed.emit(current_rpm)
	
	# Emit drift status
	if is_drifting:
		traction_control_active.emit(false)

func _resumed_simulation() -> void:
	"""Resume vehicle simulation after pause"""
	process_mode = ProcessModeEnum.ALWAYS
	set_physics_process(true)

func _paused_simulation() -> void:
	"""Pause vehicle simulation"""
	process_mode = ProcessModeEnum.PAUSED
	set_physics_process(false)

func _reset_vehicle() -> void:
	"""Reset vehicle to initial state"""
	current_speed = 0.0
	current_rpm = MIN_RPM_IDLE
	current_gear = Gear.NEUTRAL
	throttle_input = 0.0
	brake_input = 0.0
	clutch_input = 1.0
	is_drifting = false
	_drift_score = 0.0
	_drift_timer = 0.0
	position = Vector2.ZERO
	_velocity_vector = Vector2.ZERO

func get_speed_kmh() -> float:
	"""Get current speed in kilometers per hour"""
	return current_speed * 3.6

func get_speed_ms() -> float:
	"""Get current speed in meters per second"""
	return current_speed

func get_acceleration() -> float:
	"""Get current acceleration"""
	return _acceleration

func get_angular_velocity() -> float:
	"""Get current angular velocity"""
	return _angular_velocity

func get_drift_score() -> float:
	"""Get current drift score"""
	return _drift_score

func get_current_rpm() -> float:
	"""Get current engine RPM"""
	return current_rpm

func get_max_rpm() -> float:
	"""Get maximum engine RPM (redline)"""
	return MAX_RPM_REDLINE

func is_in_gear() -> bool:
	"""Check if vehicle is currently in a gear"""
	return current_gear != Gear.NEUTRAL and current_gear != Gear.REVERSE

func can_shift() -> bool:
	"""Check if vehicle can perform gear shift"""
	return _gear_shift_timer <= 0.0

func _emit_traction_control_status() -> void:
	emit_signal("traction_control_active", traction_control_enabled)

# ============================================================================
# DEBUG AND TESTING
# ============================================================================

func debug_get_all_values() -> Dictionary:
	"""Return all vehicle state for debugging"""
	return {
		"speed_kmh": get_speed_kmh(),
		"speed_ms": get_speed_ms(),
		"rpm": current_rpm,
		"max_rpm": MAX_RPM_REDLINE,
		"gear": current_gear,
		"throttle": throttle_input,
		"brake": brake_input,
		"clutch": clutch_input,
		"steering": steering_angle,
		"is_drifting": is_drifting,
		"drift_score": _drift_score,
		"engine_force": _engine_force,
		"brake_force": _brake_force,
		"acceleration": _acceleration,
		"angular_velocity": _angular_velocity,
		"position": position,
		"velocity": _velocity_vector,
		"traction_control": traction_control_enabled,
		"abs_enabled": abs_enabled,
		"differential_locked": diff_locked
	}

func force_set_speed(speed: float) -> void:
	"""Force set speed for testing purposes"""
	current_speed = speed
	_velocity_vector = Vector2.RIGHT * speed

func force_set_rpm(rpm: float) -> void:
	"""Force set RPM for testing purposes"""
	current_rpm = clamp(rpm, MIN_RPM_IDLE, MAX_RPM_REDLINE)

func force_set_gear(gear: int) -> void:
	"""Force set gear for testing purposes"""
	current_gear = gear
	target_gear = gear

func reset_gear_shift_cooldown() -> void:
	"""Reset gear shift cooldown for testing purposes"""
	_gear_shift_timer = 0.0