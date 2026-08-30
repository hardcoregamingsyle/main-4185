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
var _is_clutch_engaged: bool = true
var _is_handbrake_active: bool = false
var _is_traction_control_active: bool = true
var _is_abs_active: bool = true
var _is_drifting: bool = false
var _drift_angle: float = 0.0
var _last_collision_time: float = 0.0
var _gear_shift_timer: float = 0.0
var _clutch_timer: float = 0.0

# Powertrain reference
var _powertrain: Node = null

# Physics settings reference
var _physics_settings: PhysicsSettings = PhysicsSettings.new()

# Wheel friction coefficients
var _wheel_friction_coefficient: float = 1.2
var _wheel_slip_coefficient: float = 0.8

# Drift mechanics
var _drift_threshold: float = 0.3
var _drift_recovery_rate: float = 0.05

# ============================================================================
# PUBLIC API
# ============================================================================

func set_powertrain(powertrain_node: Node) -> void:
	"""Set reference to powertrain component"""
	_powertrain = powertrain_node

func initialize_vehicle() -> void:
	"""Initialize vehicle state and load physics settings"""
	_current_gear = NEUTRAL_GEAR
	_target_gear = NEUTRAL_GEAR
	_speed_kmh = 0.0
	_rpm = IDLE_RPM
	_is_engine_running = false
	_is_clutch_engaged = true
	_is_handbrake_active = false
	
	# Load physics settings
	var settings_path: String = "res://scripts/core/PhysicsSettings.gd"
	if ResourceLoader.exists(settings_path):
		_physics_settings = ResourceLoader.load(settings_path) as PhysicsSettings

func start_engine() -> void:
	"""Start the vehicle engine"""
	if _is_engine_running:
		return
	
	_is_engine_running = true
	_rpm = IDLE_RPM
	_rpm_changed.emit(_rpm)

func stop_engine() -> void:
	"""Stop the vehicle engine"""
	if not _is_engine_running:
		return
	
	_is_engine_running = false
	_rpm = IDLE_RPM
	_rpm_changed.emit(_rpm)
	engine_stalled.emit()

func get_speed_kmh() -> float:
	"""Get current speed in kilometers per hour"""
	return abs(_speed_kmh)

func get_rpm() -> float:
	"""Get current engine RPM"""
	return _rpm

func get_current_gear() -> int:
	"""Get current gear number"""
	return _current_gear

func get_throttle_input() -> float:
	"""Get normalized throttle input (0.0 to 1.0)"""
	return clamp(_throttle_input, 0.0, 1.0)

func get_brake_input() -> float:
	"""Get normalized brake input (0.0 to 1.0)"""
	return clamp(_brake_input, 0.0, 1.0)

func get_steering_input() -> float:
	"""Get normalized steering input (-1.0 to 1.0)"""
	return clamp(_steering_input, -1.0, 1.0)

func get_handbrake_input() -> float:
	"""Get normalized handbrake input (0.0 to 1.0)"""
	return clamp(_handbrake_input, 0.0, 1.0)

func is_engine_running() -> bool:
	"""Check if engine is running"""
	return _is_engine_running

func is_drifting() -> bool:
	"""Check if vehicle is currently drifting"""
	return _is_drifting

func set_traction_control(enabled: bool) -> void:
	"""Enable or disable traction control system"""
	if _is_traction_control_active != enabled:
		_is_traction_control_active = enabled
		traction_control_state_changed.emit(enabled)

func set_abs(enabled: bool) -> void:
	"""Enable or disable anti-lock braking system"""
	if _is_abs_active != enabled:
		_is_abs_active = enabled
		anti_lock_braking_state_changed.emit(enabled)

# ============================================================================
# INPUT HANDLING
# ============================================================================

func handle_input() -> void:
	"""Process player input for vehicle controls"""
	# Get input from InputManager singleton
	var input_manager = GameManager.get_node_or_null("/root/InputManager")
	
	if input_manager:
		# Throttle and brake (can use same axis with direction)
		var gas_axis = input_manager.get_action_strength("gas_pedal")
		var brake_axis = input_manager.get_action_strength("brake_pedal")
		
		# Steering input
		var steer_left = input_manager.get_action_strength("steer_left")
		var steer_right = input_manager.get_action_strength("steer_right")
		
		# Handbrake
		var handbrake = input_manager.get_action_strength("handbrake")
		
		# Gear shifting
		if input_manager.is_action_just_pressed("shift_up"):
			shift_up()
		elif input_manager.is_action_just_pressed("shift_down"):
			shift_down()
		
		# Store inputs for physics calculation
		_throttle_input = gas_axis
		_brake_input = brake_axis
		_steering_input = steer_right - steer_left
		_handbrake_input = handbrake
		
		throttle_applied.emit(_throttle_input)
		brake_applied.emit(_brake_input)
		steering_angle_changed.emit(_steering_input)
		handbrake_toggled.emit(_handbrake_input > 0.1)

# ============================================================================
# PHYSICS UPDATE
# ============================================================================

func _physics_process(delta: float) -> void:
	"""Update vehicle physics state"""
	if not _is_engine_running and _speed_kmh == 0.0:
		return
	
	# Update gear shift timer
	if _gear_shift_timer > 0.0:
		_gear_shift_timer -= delta
		if _gear_shift_timer <= 0.0:
			_current_gear = _target_gear
			gear_changed.emit(_current_gear, _current_gear)
	
	# Calculate engine torque based on current gear and RPM
	var torque = _calculate_engine_torque()
	
	# Apply acceleration/braking forces
	_apply_acceleration_and_braking(delta, torque)
	
	# Apply steering
	_apply_steering(delta)
	
	# Handle clutch engagement
	_handle_clutch(delta)
	
	# Check for auto-shifting
	_check_auto_shift(delta)
	
	# Detect and handle collisions
	_check_collisions(delta)

func _calculate_engine_torque() -> float:
	"""Calculate engine torque based on RPM and throttle"""
	if not _is_engine_running:
		return 0.0
	
	# Simple torque curve model
	var torque_ratio = (_rpm - IDLE_RPM) / (SHIFT_POINT_RPM - IDLE_RPM)
	torque_ratio = clamp(torque_ratio, 0.0, 1.0)
	
	# Peak torque around 4000-5000 RPM
	var peak_torque_position = 0.5
	var torque_curve = 1.0 - pow((torque_ratio - peak_torque_position) * 2.0, 2.0)
	torque_curve = max(0.0, torque_curve)
	
	var base_torque = 500.0  # Base torque in Nm
	var adjusted_torque = base_torque * torque_curve
	
	# Apply throttle multiplier
	adjusted_torque *= _throttle_input
	
	return adjusted_torque

func _apply_acceleration_and_braking(delta: float, torque: float) -> void:
	"""Apply acceleration and braking forces to vehicle"""
	if not _is_engine_running:
		return
	
	# Get mass from physics settings
	var mass = _physics_settings.default_vehicle_mass
	
	# Calculate force from torque
	var wheel_radius = 0.35  # meters
	var final_drive_ratio = 3.5
	var gear_ratios = {
		-1: 3.0,   # Reverse
		1: 4.0,    # 1st gear
		2: 2.5,    # 2nd gear
		3: 1.8,    # 3rd gear
		4: 1.4,    # 4th gear
		5: 1.1,    # 5th gear
		6: 0.9     # 6th gear
	}
	
	var current_gear_ratio = gear_ratios.get(_current_gear, 1.0)
	var drive_force = torque * current_gear_ratio * final_drive_ratio / wheel_radius
	
	# Apply drivetrain losses
	drive_force *= 0.85  # 15% loss
	
	# Handle reverse gear
	if _current_gear < 0:
		drive_force *= -1.0
	
	# Apply brakes
	var braking_force = 0.0
	if _brake_input > 0.0:
		# ABS intervention
		if _is_abs_active and _speed_kmh > ABS_THRESHOLD:
			# Reduce braking force slightly for ABS
			braking_force = BRAKING_FORCE * _brake_input * 0.9
		else:
			braking_force = BRAKING_FORCE * _brake_input
	
	# Combine forces
	var total_force = drive_force - braking_force
	
	# Apply acceleration
	var acceleration = total_force / mass
	var velocity_change = acceleration * delta
	
	# Apply to speed
	_speed_kmh += velocity_change
	
	# Clamp to maximum speed
	_speed_kmh = clamp(_speed_kmh, -MAX_SPEED_KMH / 2.0, MAX_SPEED_KMH)
	
	# Emit signal
	speed_changed.emit(_speed_kmh)

func _apply_steering(delta: float) -> void:
	"""Apply steering to vehicle rotation"""
	if abs(_speed_kmh) < 1.0:
		return  # No steering at very low speeds
	
	# Calculate steering angle
	var target_angle = _steering_input * MAX_STEERING_ANGLE
	
	# Smooth steering transition
	var current_rotation = rotation.y
	var rotation_diff = fmod(current_rotation + TAU, TAU * 2.0) - TAU
	var smoothed_angle = lerp_angle(current_rotation, target_angle, STEERING_SPEED * delta)
	
	rotation.y = smoothed_angle

func _handle_clutch(delta: float) -> void:
	"""Handle clutch engagement and disengagement"""
	if _is_clutch_engaged:
		return
	
	# Clutch release timing
	if _clutch_timer >= CLUTCH_RELEASE_TIME:
		_is_clutch_engaged = true
		_clutch_timer = 0.0

func _check_auto_shift(delta: float) -> void:
	"""Check for automatic gear shifting conditions"""
	if _current_gear == NEUTRAL_GEAR:
		return
	
	# Auto upshift at redline
	if _rpm >= SHUTDOWN_RPM and _current_gear < MAX_GEAR:
		_target_gear = _current_gear + 1
		_start_gear_shift()
	
	# Auto downshift when RPM drops too low
	elif _rpm < IDLE_RPM * 1.5 and _current_gear > MIN_GEAR:
		_target_gear = _current_gear - 1
		_start_gear_shift()

func _start_gear_shift() -> void:
	"""Initiate gear shift sequence"""
	_gear_shift_timer = GEAR_SHIFT_DELAY
	_target_gear = clamp(_target_gear, MIN_GEAR, MAX_GEAR)
	
	# Temporary clutch disengage during shift
	_is_clutch_engaged = false
	_clutch_timer = 0.0

# ============================================================================
# DRIFT MECHANICS
# ============================================================================

func update_drift_state() -> void:
	"""Update vehicle drift state based on inputs and physics"""
	# Calculate drift indicator
	var lateral_velocity = 0.0  # Would come from physics simulation
	var drift_indicator = abs(lateral_velocity) / abs(_speed_kmh) if abs(_speed_kmh) > 0 else 0.0
	
	# Check drift conditions
	var is_drifting_input = _handbrake_input > 0.5 and abs(_steering_input) > 0.5
	var is_drifting_speed = abs(_speed_kmh) > 30.0  # Minimum speed to drift
	
	if is_drifting_input and is_drifting_speed and drift_indicator > _drift_threshold:
		if not _is_drifting:
			_is_driving = true
			_drift_angle = _steering_input * 45.0 * TAU / 180.0
			drift_started.emit(_drift_angle)
	else:
		if _is_drifting:
			_is_drifting = false
			drift_ended.emit()

func apply_drift_forces() -> void:
	"""Apply drift-specific physics forces"""
	if not _is_drifting:
		return
	
	# Reduce traction during drift
	var reduced_traction = 0.6
	_wheel_friction_coefficient *= reduced_traction
	
	# Apply sideways force
	var lateral_force = _handbrake_input * DRIFT_FACTOR * _speed_kmh
	velocity.x += lateral_force * 0.1

# ============================================================================
# COLLISION HANDLING
# ============================================================================

func _check_collisions(delta: float) -> void:
	"""Check for collision events"""
	for collision in get_slide_collision_count():
		var colliding_body = get_slide_collision(collision).get_collider()
		var time_since_last = delta + _last_collision_time
		
		if time_since_last < 0.5:  # Debounce collisions
			continue
		
		# Create collision info
		var collision_info: Dictionary = {
			"collider": colliding_body.name if colliding_body else "unknown",
			"impact_speed": abs(_speed_kmh),
			"timestamp": Time.get_ticks_msec(),
			"is_high_impact": abs(_speed_kmh) > 100.0
		}
		
		collision_detected.emit(collision_info)
		_last_collision_time = Time.get_ticks_msec()

# ============================================================================
# TRACTION CONTROL
# ============================================================================

func update_traction_control() -> void:
	"""Update traction control state"""
	if not _is_traction_control_active:
		return
	
	# Check for wheel slip
	var wheel_slip = abs(_rpm) / SHIFT_POINT_RPM - abs(_speed_kmh) / MAX_SPEED_KMH
	
	if wheel_slip > TRACTION_CONTROL_THRESHOLD:
		# Reduce throttle to prevent spin
		_throttle_input = max(0.0, _throttle_input * 0.7)
		skidding.emit(true)
	else:
		skidding.emit(false)

# ============================================================================
# UTILITY METHODS
# ============================================================================

func reset_vehicle() -> void:
	"""Reset vehicle to initial state"""
	_speed_kmh = 0.0
	_rpm = IDLE_RPM
	_current_gear = NEUTRAL_GEAR
	_target_gear = NEUTRAL_GEAR
	_throttle_input = 0.0
	_brake_input = 0.0
	_steering_input = 0.0
	_handbrake_input = 0.0
	_is_engine_running = false
	_is_clutch_engaged = true
	_is_handbrake_active = false
	_is_drifting = false
	_last_collision_time = 0.0
	_gear_shift_timer = 0.0
	_clutch_timer = 0.0

func save_state() -> Dictionary:
	"""Save current vehicle state"""
	return {
		"speed_kmh": _speed_kmh,
		"rpm": _rpm,
		"current_gear": _current_gear,
		"throttle_input": _throttle_input,
		"brake_input": _brake_input,
		"steering_input": _steering_input,
		"handbrake_input": _handbrake_input,
		"is_engine_running": _is_engine_running,
		"is_drifting": _is_drifting,
		"is_traction_control_active": _is_traction_control_active,
		"is_abs_active": _is_abs_active
	}

func load_state(state: Dictionary) -> void:
	"""Load saved vehicle state"""
	_speed_kmh = state.get("speed_kmh", 0.0)
	_rpm = state.get("rpm", IDLE_RPM)
	_current_gear = state.get("current_gear", NEUTRAL_GEAR)
	_throttle_input = state.get("throttle_input", 0.0)
	_brake_input = state.get("brake_input", 0.0)
	_steering_input = state.get("steering_input", 0.0)
	_handbrake_input = state.get("handbrake_input", 0.0)
	_is_engine_running = state.get("is_engine_running", false)
	_is_drifting = state.get("is_drifting", false)
	_is_traction_control_active = state.get("is_traction_control_active", true)
	_is_abs_active = state.get("is_abs_active", true)

# ============================================================================
# HELPER METHODS
# ============================================================================

func shift_up() -> void:
	"""Shift transmission up one gear"""
	if _current_gear < MAX_GEAR:
		_target_gear = _current_gear + 1
		_start_gear_shift()

func shift_down() -> void:
	"""Shift transmission down one gear"""
	if _current_gear > MIN_GEAR:
		_target_gear = _current_gear - 1
		_start_gear_shift()

func emergency_stop() -> void:
	"""Perform emergency stop with maximum braking"""
	_brake_input = 1.0
	_throttle_input = 0.0
	_is_abs_active = true

func toggle_handbrake() -> void:
	"""Toggle handbrake state"""
	_handbrake_active = not _is_handbrake_active
	handbrake_toggled.emit(_is_handbrake_active)

func get_gear_ratio() -> float:
	"""Get current gear ratio"""
	var gear_ratios = {
		-1: 3.0,
		0: 0.0,
		1: 4.0,
		2: 2.5,
		3: 1.8,
		4: 1.4,
		5: 1.1,
		6: 0.9
	}
	return gear_ratios.get(_current_gear, 1.0)

func calculate_distance_traveled() -> float:
	"""Calculate total distance traveled in meters"""
	# This would accumulate over time in actual implementation
	return 0.0

func get_vehicle_health() -> float:
	"""Get vehicle health percentage (0.0 to 1.0)"""
	# Placeholder - would track damage in full implementation
	return 1.0

func take_damage(damage_amount: float) -> void:
	"""Apply damage to vehicle"""
	# Placeholder - would reduce health in full implementation
	pass

func is_critical_condition() -> bool:
	"""Check if vehicle is in critical condition"""
	return _rpm > SHUTDOWN_RPM or abs(_speed_kmh) > MAX_SPEED_KMH

func request_restart() -> void:
	"""Request vehicle restart after stall"""
	if _is_engine_running:
		stop_engine()
		start_engine()
	_reset_vehicle()