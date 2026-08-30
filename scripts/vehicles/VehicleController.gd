extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulation
## Handles throttle, brake, steering, wheel forces, gear shifting, and vehicle dynamics
## Copyright 2026 Thalamus Racing Simulator Project

# ============================================================================
# SIGNALS - Game Event Notifications
# ============================================================================
signal speed_changed(current_speed: float)
signal rpm_changed(current_rpm: float)
signal gear_changed(new_gear: int)
signal drift_started(drift_intensity: float)
signal drift_ended()
signal collision_detected(collision_info: Dictionary)
signal lap_completed(lap_data: Dictionary)
signal race_event(event_type: String, data: Dictionary)
signal engine_sound_changed(rpm_ratio: float)
signal suspension_compressed(compression_amount: float)

# ============================================================================
# CONSTANTS - Physics Tuning Values
# ============================================================================
const MAX_SPEED_KMH: float = 320.0
const ACCELERATION_RATE: float = 12.0
const BRAKING_FORCE: float = 20.0
const TURN_SPEED: float = 4.5
const DRIFT_THRESHOLD: float = 0.7
const DRIFT_INTENSITY_MAX: float = 1.0
const MIN_GEAR: int = 0
const MAX_GEAR: int = 6
const IDLE_RPM: float = 800.0
const REDLINE_RPM: float = 7500.0
const SHIFT_RPM: float = 7000.0
const CLUTCH_RELEASE_TIME: float = 0.15
const TURBO_CHARGE_CAPACITY: float = 1.0
const TURBO_RECHARGE_RATE: float = 0.5
const DRAG_COEFFICIENT: float = 0.30
const WHEEL_BASE: float = 2.5
const TRACK_WIDTH: float = 1.6
const SUSPENSION_STIFFNESS: float = 50000.0
const SUSPENSION_DAMPING: float = 3000.0
const SUSPENSION_TRAVEL: float = 0.2

# ============================================================================
# EXPORTED PROPERTIES - Runtime Configuration
# ============================================================================
@export_group("Vehicle Properties")
@export var vehicle_mass: float = 1500.0: set = _set_vehicle_mass
@export var center_of_mass_offset: Vector3 = Vector3(0.0, 0.5, 0.0): set = _set_center_of_mass_offset
@export var aerodynamic_drag: float = DRAG_COEFFICIENT: set = _set_aerodynamic_drag
@export var rolling_resistance: float = 0.015: set = _set_rolling_resistance

@export_group("Engine Properties")
@export var engine_max_torque: float = 450.0: set = _set_engine_max_torque
@export var engine_peak_power_rpm: float = 5500.0: set = _set_engine_peak_power_rpm
@export var engine_idle_rpm: float = IDLE_RPM: set = _set_engine_idle_rpm
@export var engine_redline_rpm: float = REDLINE_RPM: set = _set_engine_redline_rpm
@export var transmission_ratios: Array[float] = [3.5, 2.5, 1.8, 1.3, 0.95, 0.75, 0.6]: set = _set_transmission_ratios
@export var final_drive_ratio: float = 3.73: set = _set_final_drive_ratio
@export var differential_type: int = 0 # 0=LSD, 1=Open, 2=Locked

@export_group("Wheel Properties")
@export var wheel_radius: float = 0.33: set = _set_wheel_radius
@export var wheel_width: float = 0.25: set = _set_wheel_width
@export var grip_coefficient: float = 1.2: set = _set_grip_coefficient
@export var slip_threshold: float = 0.15: set = _set_slip_threshold

@export_group("Drift & Handling")
@export var drift_enabled: bool = true
@export var understeer_factor: float = 0.15: set = _set_understeer_factor
@export var oversteer_factor: float = 0.20: set = _set_oversteer_factor
@export var weight_transfer_factor: float = 0.4: set = _set_weight_transfer_factor

@export_group("Turbo System")
@export var turbo_enabled: bool = false
@export var boost_pressure: float = 1.2: set = _set_boost_pressure
@export var turbo_response_time: float = 0.2: set = _set_turbo_response_time

# ============================================================================
# INTERNAL STATE VARIABLES
# ============================================================================
var _current_speed: float = 0.0 # km/h
var _current_rpm: float = IDLE_RPM
var _current_gear: int = MIN_GEAR
var _target_gear: int = MIN_GEAR
var _throttle_input: float = 0.0
var _brake_input: float = 0.0
var _clutch_input: float = 0.0
var _steering_input: float = 0.0

var _rpm_during_shift: float = 0.0
var _shift_timer: float = 0.0
var _clutch_timer: float = 0.0
var _is_shifting: bool = false

var _turbo_charge: float = 0.0
var _turbo_boost_active: bool = false

var _drift_intensity: float = 0.0
var _is_drifting: bool = false
var _lateral_force: float = 0.0
var _longitudinal_force: float = 0.0

var _suspension_state: Array[Vector3] = []
var _wheel_contact_points: Array[Vector3] = []
var _wheel_rotation_angles: Array[float] = []

var _vehicle_velocity: Vector3 = Vector3.ZERO
var _vehicle_acceleration: Vector3 = Vector3.ZERO

var _engine_torque_curve: Array[float] = []
var _power_curve: Array[float] = []
var _torque_multiplier: float = 1.0

# ============================================================================
# INITIALIZATION
# ============================================================================
func _ready() -> void:
	_init_torque_curve()
	_init_suspension_state()
	_process_mode = ProcessModeEnum.PROCESS_ALWAYS
	set_physics_process(true)
	
	# Initialize wheel contact points (Front-Left, Front-Right, Rear-Left, Rear-Right)
	var half_track = TRACK_WIDTH / 2.0
	var front_z = WHEEL_BASE / 2.0
	var rear_z = -WHEEL_BASE / 2.0
	
	_wheel_contact_points = [
		Vector3(-half_track, 0.0, front_z),   # FL
		Vector3(half_track, 0.0, front_z),    # FR
		Vector3(-half_track, 0.0, rear_z),    # RL
		Vector3(half_track, 0.0, rear_z)     # RR
	]
	
	_wheel_rotation_angles = [0.0, 0.0, 0.0, 0.0]
	_suspension_state = [Vector3.ZERO, Vector3.ZERO, Vector3.ZERO, Vector3.ZERO]

func _init_torque_curve() -> void:
	"""Initialize engine torque curve based on RPM"""
	var samples: int = 100
	for i in range(samples + 1):
		var rpm_percent: float = float(i) / float(samples) * (engine_redline_rpm / 1000.0)
		# Simple bell-shaped torque curve peaking around 4000-5000 RPM
		var normalized_rpm: float = clamp(rpm_percent / 4.5, 0.0, 1.0)
		var torque: float = engine_max_torque * (-4.0 * pow(normalized_rpm - 0.55, 2.0) + 1.0)
		torque = max(torque, 0.0)
		_engine_torque_curve.append(torque)

func _init_suspension_state() -> void:
	"""Initialize suspension states for all wheels"""
	_suspension_state.resize(4)
	for i in 4:
		_suspension_state[i] = Vector3.ZERO

# ============================================================================
# MAIN PHYSICS LOOP
# ============================================================================
func _physics_process(delta: float) -> void:
	if not is_inside_tree():
		return
	
	# Handle clutch timer during shifts
	if _is_shifting:
		_clutch_timer += delta
		if _clutch_timer >= CLUTCH_RELEASE_TIME:
			_complete_shift()
	
	# Get input values
	_throttle_input = InputManager.get_axis("accelerate", "brake_reverse")
	_brake_input = InputManager.get_axis("brake", "reverse")
	_steering_input = InputManager.get_axis("turn_left", "turn_right")
	_clutch_input = InputManager.get_axis("clutch_up", "clutch_down")
	
	# Auto-clutch assist when not manually engaging
	if InputManager.is_action_pressed("auto_clutch"):
		_clutch_input = 1.0
	
	# Update vehicle physics
	_update_engine_rpm(delta)
	_update_gear_system(delta)
	_update_vehicle_dynamics(delta)
	_update_drift_system(delta)
	_update_turbo_system(delta)
	
	# Apply calculated forces to velocity
	_apply_forces_to_body(delta)
	
	# Update movement
	move_and_slide()
	
	# Emit signals
	_emit_signals()

# ============================================================================
# ENGINE & TRANSMISSION SYSTEMS
# ============================================================================
func _update_engine_rpm(delta: float) -> void:
	"""Calculate current engine RPM based on gear and vehicle speed"""
	if _current_gear == MIN_GEAR or _current_gear == 0:
		# Neutral - idle or revving
		if _throttle_input > 0.0:
			_current_rpm += _throttle_input * 2000.0 * delta
			_current_rpm = clamp(_current_rpm, engine_idle_rpm, engine_redline_rpm)
		else:
			_current_rpm -= 300.0 * delta
			_current_rpm = max(_current_rpm, engine_idle_rpm)
		return
	
	# Calculate wheel RPM from vehicle speed
	var wheel_circumference: float = 2.0 * PI * wheel_radius
	var speed_ms: float = _current_speed * 1000.0 / 3600.0 # Convert km/h to m/s
	var wheel_rps: float = speed_ms / wheel_circumference if wheel_circumference > 0 else 0.0
	var wheel_rpm: float = wheel_rps * 60.0
	
	# Calculate engine RPM based on gear ratio
	var total_ratio: float = transmission_ratios[_current_gear] * final_drive_ratio
	_current_rpm = wheel_rpm * total_ratio
	
	# Clamp to valid RPM range
	_current_rpm = clamp(_current_rpm, engine_idle_rpm, engine_redline_rpm)
	
	# Apply turbo boost effect
	if _turbo_boost_active:
		_current_rpm *= boost_pressure

func _update_gear_system(delta: float) -> void:
	"""Handle gear shifting logic"""
	# Manual shift request
	if InputManager.is_action_just_pressed("shift_up"):
		_request_gear_shift(1)
	elif InputManager.is_action_just_pressed("shift_down"):
		_request_gear_shift(-1)
	
	# Auto-shift logic (if enabled)
	if InputManager.is_action_pressed("auto_shift"):
		_auto_shift(delta)
	
	# Gear up/down based on RPM and throttle
	if !_is_shifting and _current_gear > MIN_GEAR:
		if _current_rpm >= SHIFT_RPM and _throttle_input > 0.0:
			_request_gear_shift(1)
		elif _current_rpm < engine_idle_rpm + 500 and _throttle_input < 0.3:
			_request_gear_shift(-1)

func _request_gear_shift(direction: int) -> void:
	"""Initiate a gear shift request"""
	if _is_shifting:
		return
	
	var new_gear: int = _current_gear + direction
	
	# Validate gear range
	if new_gear < MIN_GEAR or new_gear > MAX_GEAR:
		return
	
	# Check if we're trying to shift into reverse while moving forward
	if _current_gear == 0 and new_gear == -1 and _current_speed > 1.0:
		return
	
	# Start shift sequence
	_is_shifting = true
	_target_gear = new_gear
	_rpm_during_shift = _current_rpm
	_clutch_timer = 0.0
	
	# Drop RPM during clutch disengagement
	_current_rpm = engine_idle_rpm
	
	# Emitter signal
	race_event.emit("gear_shift", {"from": _current_gear, "to": new_gear})

func _auto_shift(delta: float) -> void:
	"""Automatic gear shifting based on RPM"""
	if _is_shifting or _current_gear == MIN_GEAR:
		return
	
	if _current_rpm >= SHIFT_RPM * 0.95 and _throttle_input > 0.1:
		_request_gear_shift(1)
	elif _current_rpm <= engine_idle_rpm + 300 and _throttle_input < 0.2:
		_request_gear_shift(-1)

func _complete_shift() -> void:
	"""Complete the gear shift sequence"""
	_current_gear = _target_gear
	_is_shifting = false
	_clutch_timer = 0.0
	
	gear_changed.emit(_current_gear)
	engine_sound_changed.emit(_current_rpm / engine_redline_rpm)

# ============================================================================
# VEHICLE DYNAMICS CALCULATION
# ============================================================================
func _update_vehicle_dynamics(delta: float) -> void:
	"""Calculate vehicle forces and acceleration"""
	_lateral_force = 0.0
	_longitudinal_force = 0.0
	
	# Calculate available torque
	var torque_available: float = _calculate_engine_torque()
	
	# Apply turbo boost to torque
	if _turbo_boost_active:
		torque_available *= boost_pressure
	
	# Apply clutch engagement
	var clutch_effect: float = 1.0 - _clutch_input
	torque_available *= clutch_effect
	
	# Distribute torque to driven wheels
	var drive_wheels: int = _get_drive_configuration()
	var torque_per_wheel: float = torque_available / drive_wheels if drive_wheels > 0 else 0.0
	
	# Calculate longitudinal force (traction)
	var traction_loss: float = _calculate_traction_loss()
	var effective_torque: float = torque_per_wheel * (1.0 - traction_loss)
	_longitudinal_force = effective_torque / wheel_radius
	
	# Apply weight transfer effects
	var weight_transfer: float = _calculate_weight_transfer()
	_longitudinal_force *= weight_transfer
	
	# Apply braking force
	if _brake_input > 0.0:
		var brake_force: float = BRAKING_FORCE * _brake_input * vehicle_mass
		_longitudinal_force -= brake_force
	
	# Apply air drag
	var drag_force: float = 0.5 * DRAG_COEFFICIENT * pow(_current_speed * 0.277778, 2.0) * 1.2 # Air density
	_longitudinal_force -= drag_force
	
	# Apply rolling resistance
	var rolling_force: float = rolling_resistance * vehicle_mass * 9.81
	_longitudinal_force -= rolling_force
	
	# Limit maximum speed
	var max_speed_ms: float = MAX_SPEED_KMH * 1000.0 / 3600.0
	if _current_speed > MAX_SPEED_KMH:
		_current_speed = MAX_SPEED_KMH
	
	# Calculate acceleration
	var acceleration: float = _longitudinal_force / vehicle_mass
	_vehicle_acceleration.z = acceleration
	
	# Update velocity
	var delta_speed: float = acceleration * delta * 3.6 # Convert to km/h
	_current_speed += delta_speed
	_current_speed = clamp(_current_speed, -MAX_SPEED_KMH, MAX_SPEED_KMH)
	
	# Handle reverse gear behavior
	if _current_gear == -1:
		_current_speed = -abs(_current_speed)
	
	# Calculate steering angle
	var steering_angle: float = _calculate_steering_angle()
	
	# Update lateral forces for turning
	var turn_radius: float = WHEEL_BASE / tan(steering_angle) if abs(steering_angle) > 0.01 else 999999.0
	var centripetal_force: float = pow(_current_speed * 0.277778, 2.0) / turn_radius
	_lateral_force = centripetal_force * vehicle_mass
	
	# Apply lateral grip limit
	var grip_limit: float = grip_coefficient * vehicle_mass * 9.81
	if abs(_lateral_force) > grip_limit:
		_lateral_force = sign(_lateral_force) * grip_limit
		_start_drift()
	else:
		_end_drift()

func _calculate_engine_torque() -> float:
	"""Get torque value based on current RPM"""
	var rpm_normalized: float = (_current_rpm - engine_idle_rpm) / (engine_redline_rpm - engine_idle_rpm)
	var index: int = floor(rpm_normalized * (_engine_torque_curve.size() - 1))
	index = clamp(index, 0, _engine_torque_curve.size() - 2)
	
	var t1: float = _engine_torque_curve[index]
	var t2: float = _engine_torque_curve[index + 1]
	var alpha: float = (rpm_normalized * (_engine_torque_curve.size() - 1)) - index
	var torque: float = t1 * (1.0 - alpha) + t2 * alpha
	
	return torque * _torque_multiplier

func _calculate_traction_loss() -> float:
	"""Calculate how much traction is lost due to wheel slip"""
	var slip_ratio: float = abs(_current_rpm * wheel_radius * 0.0166667 - _current_speed * 0.277778) / (_current_speed * 0.277778 + 0.1)
	slip_ratio = min(slip_ratio, 2.0)
	
	var loss: float = 0.0
	if slip_ratio > slip_threshold:
		loss = (slip_ratio - slip_threshold) * 0.5
		loss = min(loss, 0.8)
	
	return loss

func _calculate_weight_transfer() -> float:
	"""Calculate weight transfer factor affecting traction"""
	var acceleration_factor: float = _vehicle_acceleration.z * vehicle_mass / (vehicle_mass * 9.81)
	var transfer: float = acceleration_factor * weight_transfer_factor
	
	# More weight on driven wheels during acceleration
	var drive_wheels: int = _get_drive_configuration()
	if _longitudinal_force > 0:
		transfer = 1.0 + transfer / drive_wheels
	else:
		transfer = 1.0 - transfer / drive_wheels
	
	return clamp(transfer, 0.5, 1.5)

func _calculate_steering_angle() -> float:
	"""Calculate steering angle based on input"""
	var base_angle: float = _steering_input * deg_to_rad(TURN_SPEED)
	
	# Speed-dependent steering reduction
	var speed_factor: float = 1.0 - (abs(_current_speed) / MAX_SPEED_KMH) * 0.5
	base_angle *= speed_factor
	
	# Add understeer/oversteer characteristics
	if _current_speed > 50.0:
		if _current_gear % 2 == 0: # Even gears more stable
			base_angle *= (1.0 - understeer_factor)
		else:
			base_angle *= (1.0 + oversteer_factor)
	
	return base_angle

func _get_drive_configuration() -> int:
	"""Get number of driven wheels based on vehicle configuration"""
	# For now, assume rear-wheel drive
	return 2

# ============================================================================
# DRIFT SYSTEM
# ============================================================================
func _start_drift() -> void:
	"""Start drifting when lateral force exceeds grip"""
	if not drift_enabled:
		return
	
	_is_drifting = true
	_drift_intensity = min(abs(_lateral_force) / (grip_coefficient * vehicle_mass * 9.81), 1.0)
	
	drift_started.emit(_drift_intensity)

func _end_drift() -> void:
	"""End drifting when back within grip limits"""
	if _is_drifting:
		_is_drifting = false
		_drift_intensity = 0.0
		drift_ended.emit()

func _update_drift_system(delta: float) -> void:
	"""Update drift mechanics and visual feedback"""
	if not _is_drifting:
		return
	
	# Gradually reduce drift intensity when no longer applying lateral force
	if _lateral_force < grip_coefficient * vehicle_mass * 9.81 * 0.5:
		_drift_intensity -= delta * 0.5
		_drift_intensity = max(_drift_intensity, 0.0)

# ============================================================================
# TURBO SYSTEM
# ============================================================================
func _update_turbo_system(delta: float) -> void:
	"""Update turbocharger charge and boost"""
	if not turbo_enabled:
		_turbo_charge = 0.0
		_turbo_boost_active = false
		return
	
	# Charge turbo when throttle applied
	if _throttle_input > 0.0:
		_turbo_charge += _throttle_input * TURBO_RECHARGE_RATE * delta
		_turbo_charge = min(_turbo_charge, TURBO_CHARGE_CAPACITY)
	else:
		_turbo_charge -= TURBO_RECHARGE_RATE * delta
		_turbo_charge = max(_turbo_charge, 0.0)
	
	# Activate boost when charged enough
	if _turbo_charge >= 0.8:
		_turbo_boost_active = true
	else:
		_turbo_boost_active = false

func activate_turbo() -> void:
	"""Manually activate turbo boost"""
	if turbo_enabled and _turbo_charge >= 0.5:
		_turbo_boost_active = true
		_turbo_charge = 0.0
		race_event.emit("turbo_activate", {})

# ============================================================================
# FORCE APPLICATION
# ============================================================================
func _apply_forces_to_body(delta: float) -> void:
	"""Apply calculated forces to the vehicle body"""
	# Apply longitudinal force as acceleration
	var force_vector: Vector3 = Vector3(0.0, 0.0, _longitudinal_force)
	velocity += force_vector * delta / vehicle_mass
	
	# Apply lateral force as sideways movement
	var lateral_vector: Vector3 = Vector3(_lateral_force, 0.0, 0.0)
	velocity += lateral_vector * delta / vehicle_mass
	
	# Apply rotation based on steering
	var angular_velocity: float = _steering_input * TURN_SPEED * 0.5
	rotation.y += angular_velocity * delta

# ============================================================================
# SIGNAL EMITTERS
# ============================================================================
func _emit_signals() -> void:
	"""Emit update signals for UI and other systems"""
	speed_changed.emit(_current_speed)
	rpm_changed.emit(_current_rpm)
	engine_sound_changed.emit(_current_rpm / engine_redline_rpm)
	
	# Suspension compression visualization
	for i in 4:
		var compression: float = _suspension_state[i].y
		suspension_compressed.emit(compression)

# ============================================================================
# PUBLIC API - Control Methods
# ============================================================================
func set_throttle(value: float) -> void:
	"""Directly set throttle input (for AI or external control)"""
	_throttle_input = clamp(value, -1.0, 1.0)

func set_brake(value: float) -> void:
	"""Directly set brake input"""
	_brake_input = clamp(value, 0.0, 1.0)

func set_steering(value: float) -> void:
	"""Directly set steering input"""
	_steering_input = clamp(value, -1.0, 1.0)

func set_gear(gear: int) -> void:
	"""Force set gear (manual override)"""
	if gear >= MIN_GEAR and gear <= MAX_GEAR:
		_current_gear = gear
		gear_changed.emit(gear)

func get_current_speed() -> float:
	"""Get current vehicle speed in km/h"""
	return _current_speed

func get_current_rpm() -> float:
	"""Get current engine RPM"""
	return _current_rpm

func get_current_gear() -> int:
	"""Get current gear"""
	return _current_gear

func get_vehicle_velocity() -> Vector3:
	"""Get vehicle velocity vector"""
	return velocity

func reset_vehicle() -> void:
	"""Reset vehicle to initial state"""
	_current_speed = 0.0
	_current_rpm = engine_idle_rpm
	_current_gear = MIN_GEAR
	_velocity = Vector3.ZERO
	_acceleration = Vector3.ZERO
	_turbo_charge = 0.0
	_is_drifting = false
	_is_shifting = false
	position = Vector3.ZERO
	rotation = Vector3.ZERO

func apply_collision_impact(force: Vector3, point: Vector3) -> void:
	"""Handle collision impact"""
	collision_detected.emit({
		"force": force.length(),
		"point": point,
		"speed": _current_speed
	})
	
	# Apply impulse
	var impulse: Vector3 = force * 0.1 # Dampen impact
	velocity += impulse / vehicle_mass
	
	# Reduce speed significantly on impact
	_current_speed *= 0.5
	
	# Trigger crash event
	race_event.emit("collision", {
		"impact_force": force.length(),
		"damage_level": min(force.length() / 5000.0, 1.0)
	})

# ============================================================================
# PROPERTY SETTERS
# ============================================================================
func _set_vehicle_mass(value: float) -> void:
	vehicle_mass = value
	rebuild_spring_constants()

func _set_center_of_mass_offset(value: Vector3) -> void:
	center_of_mass_offset = value

func _set_aerodynamic_drag(value: float) -> void:
	aerodynamic_drag = value

func _set_rolling_resistance(value: float) -> void:
	rolling_resistance = value

func _set_engine_max_torque(value: float) -> void:
	engine_max_torque = value
	_init_torque_curve()

func _set_engine_peak_power_rpm(value: float) -> void:
	engine_peak_power_rpm = value

func _set_engine_idle_rpm(value: float) -> void:
	engine_idle_rpm = value

func _set_engine_redline_rpm(value: float) -> void:
	engine_redline_rpm = value

func _set_transmission_ratios(value: Array[float]) -> void:
	transmission_ratios = value

func _set_final_drive_ratio(value: float) -> void:
	final_drive_ratio = value

func _set_wheel_radius(value: float) -> void:
	wheel_radius = value

func _set_wheel_width(value: float) -> void:
	wheel_width = value

func _set_grip_coefficient(value: float) -> void:
	grip_coefficient = value

func _set_slip_threshold(value: float) -> void:
	slip_threshold = value

func _set_understeer_factor(value: float) -> void:
	understeer_factor = value

func _set_oversteer_factor(value: float) -> void:
	oversteer_factor = value

func _set_weight_transfer_factor(value: float) -> void:
	weight_transfer_factor = value

func _set_boost_pressure(value: float) -> void:
	boost_pressure = value

func _set_turbo_response_time(value: float) -> void:
	turbo_response_time = value

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================
func rebuild_spring_constants() -> void:
	"""Rebuild suspension spring constants based on mass"""
	pass

func get_tire_load(index: int) -> float:
	"""Get tire load for a specific wheel"""
	if index < 0 or index >= 4:
		return 0.0
	
	var normal_force: float = vehicle_mass * 9.81 / 4.0
	
	# Adjust for weight transfer
	if _vehicle_acceleration.z > 0:
		# Acceleration transfers weight back
		if index >= 2: # Rear wheels
			normal_force *= 1.2
		else:
			normal_force *= 0.8
	else:
		# Braking transfers weight forward
		if index < 2: # Front wheels
			normal_force *= 1.2
		else:
			normal_force *= 0.8
	
	return normal_force

func calculate_lap_time(start_time: float) -> float:
	"""Calculate elapsed lap time"""
	return Time.get_ticks_msec() - start_time

func record_checkpoint(checkpoint_id: int) -> void:
	"""Record passing through a checkpoint"""
	race_event.emit("checkpoint_passed", {
		"id": checkpoint_id,
		"lap_time": get_current_speed()
	})

func save_vehicle_state() -> Dictionary:
	"""Save current vehicle state for replay/saving"""
	return {
		"position": position,
		"rotation": rotation,
		"velocity": velocity,
		"speed": _current_speed,
		"rpm": _current_rpm,
		"gear": _current_gear,
		"throttle": _throttle_input,
		"brake": _brake_input,
		"steering": _steering_input,
		"turbo_charge": _turbo_charge
	}

func restore_vehicle_state(state: Dictionary) -> void:
	"""Restore vehicle state from saved data"""
	position = state["position"]
	rotation = state["rotation"]
	velocity = state["velocity"]
	_current_speed = state["speed"]
	_current_rpm = state["rpm"]
	_current_gear = state["gear"]
	_throttle_input = state["throttle"]
	_brake_input = state["brake"]
	_steering_input = state["steering"]
	_turbo_charge = state["turbo_charge"]