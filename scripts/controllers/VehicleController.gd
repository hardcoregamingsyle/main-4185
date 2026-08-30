extends Node
class_name VehicleController

## VehicleController - Core vehicle physics controller base class
## Handles throttle, brake, steering inputs, wheel forces, and gear shifting logic
## Integrates with PhysicsSettings for consistent vehicle dynamics
## Copyright 2026 Thalamus Racing Simulator Project

signal throttle_changed(value: float)
signal brake_changed(value: float)
signal steering_changed(angle: float)
signal gear_changed(old_gear: int, new_gear: int)
signal speed_changed(speed_kmh: float)
signal rpm_changed(rpm: float)
signal vehicle_moving(is_moving: bool)

enum Gear {
	NONE = 0,
	REVERSE = -1,
	FIRST = 1,
	SECOND = 2,
	THIRD = 3,
	FOURTH = 4,
	FIFTH = 5,
	SIXTH = 6,
	SEVENTH = 7,
	EIGHTH = 8,
	DIRTY = -99
}

# ============================================================================
# INPUT CONFIGURATION
# ============================================================================

@export_group("Input Configuration")
@export var max_steering_angle: float = 30.0  # degrees
@export var steering_sensitivity: float = 1.0
@export var steer_damping: float = 15.0  # smooth steering return

@export_group("Throttle Configuration")
@export var throttle_response_time: float = 0.15  # seconds to reach target
@export var auto_throttle_recovery: bool = true

@export_group("Brake Configuration")
@export var brake_pressure_curve: float = 1.5  # non-linear brake feel
@export var anti_lock_braking: bool = true
@export var brake_bias_front: float = 0.6  # front/rear brake distribution

@export_group("Gear Shifting")
@export var upshift_rpm_threshold: float = 7000.0
@export var downshift_rpm_threshold: float = 2500.0
@export var neutral_safe_zone: float = 500.0  # RPM below which shift to neutral
@export var rev_match_downshift: bool = true
@export var manual_override_enabled: bool = True

# ============================================================================
# PHYSICS STATE
# ============================================================================

var _current_speed: float = 0.0  # km/h
var _current_rpm: float = 0.0
var _current_gear: Gear = Gear.NONE
var _target_gear: Gear = Gear.NONE
var _wheel_base: float = 2.7  # meters
var _track_width: float = 1.6  # meters

# Input states (normalized 0-1)
var _input_throttle: float = 0.0
var _input_brake: float = 0.0
var _input_steer: float = 0.0  # -1 to +1

# Smoothed input values
var _smooth_throttle: float = 0.0
var _smooth_brake: float = 0.0
var _smooth_steering: float = 0.0

# Physical forces applied to wheels
var _front_wheel_torque: float = 0.0
var _rear_wheel_torque: float = 0.0
var _front_wheel_brake_force: float = 0.0
var _rear_wheel_brake_force: float = 0.0

# Clutch and transmission state
var _clutch_engaged: bool = false
var _clutch_percent: float = 1.0  # 1.0 = fully engaged, 0.0 = fully disengaged
var _gearbox_type: String = "manual"  # "manual", "automatic", "sequential"

# ============================================================================
# INTERNAL REFS
# ============================================================================

var _powertrain: Resource = null
var _physics_settings: PhysicsSettings = PhysicsSettings.new()
var _vehicle_body: CharacterBody3D = null

func _ready() -> void:
	_init_physics_settings()
	_connect_signals()
	
	# Initialize clutch to disengaged for safe start
	_clutch_engaged = false
	_clutch_percent = 0.0

func _init_physics_settings() -> void:
	# Get physics settings from autoload singleton if available
	if Engine.has_singleton("GameManager"):
		var gm = get_tree().root.get_node_or_null("GameManager")
		if gm:
			_powertrain = gm.get_node_or_null("/root/Powertrain")
			
	# Fallback to resource
	_physics_settings = preload("res://scripts/core/PhysicsSettings.gd").new()

func _connect_signals() -> void:
	# Listen for powertrain signals if available
	if _powertrain:
		_powertrain.rpm_changed.connect(_on_powertrain_rpm_changed)
		_powertrain.gear_changed.connect(_on_powertrain_gear_changed)

func _process(delta: float) -> void:
	if not is_inside_tree():
		return
	
	_update_input_smoothing(delta)
	_handle_steering(delta)
	_update_vehicle_state(delta)
	_apply_wheels(delta)

func _physics_process(delta: float) -> void:
	if not is_inside_tree():
		return
	
	# Handle gear shifts
	_handle_gear_shifting(delta)
	_update_transmission_efficiency()

# ============================================================================
# INPUT HANDLING
# ============================================================================

func set_throttle_input(value: float) -> void:
	"""Set throttle input (0.0 to 1.0)"""
	_input_throttle = clamp(value, 0.0, 1.0)
	emit_signal("throttle_changed", _input_throttle)

func set_brake_input(value: float) -> void:
	"""Set brake input (0.0 to 1.0)"""
	_input_brake = clamp(value, 0.0, 1.0)
	emit_signal("brake_changed", _input_brake)

func set_steering_input(value: float) -> void:
	"""Set steering input (-1.0 to +1.0)"""
	_input_steer = clamp(value, -1.0, 1.0)
	emit_signal("steering_changed", _input_steer)

func get_input_throttle() -> float:
	return _input_throttle

func get_input_brake() -> float:
	return _input_brake

func get_input_steering() -> float:
	return _input_steer

func reset_inputs() -> void:
	"""Reset all inputs to neutral"""
	set_throttle_input(0.0)
	set_brake_input(0.0)
	set_steering_input(0.0)

# ============================================================================
# INPUT SMOOTHING
# ============================================================================

func _update_input_smoothing(delta: float) -> void:
	"""Smooth input transitions for realistic feel"""
	# Throttle smoothing
	_smooth_throttle = lerp(_smooth_throttle, _input_throttle, delta * 10.0)
	
	# Brake smoothing
	_smooth_brake = lerp(_smooth_brake, _input_brake, delta * 15.0)
	
	# Steering smoothing
	_smooth_steering = lerp(_smooth_steering, _input_steer, delta * steer_damping)

# ============================================================================
# STEERING LOGIC
# ============================================================================

func _handle_steering(delta: float) -> void:
	"""Calculate actual steering angle with damping and limits"""
	if not _vehicle_body:
		return
	
	var target_angle: float = _smooth_steering * max_steering_angle
	var current_angle: float = _get_current_steering_angle()
	
	# Apply steering with damping
	var steer_diff: float = target_angle - current_angle
	var steering_change: float = steer_diff * delta * steer_damping
	
	# Apply to vehicle body
	_set_steering_angle(current_angle + steering_change)

func _get_current_steering_angle() -> float:
	"""Get current steering angle from vehicle body"""
	if _vehicle_body:
		return _vehicle_body.get_steering_angle()
	return 0.0

func _set_steering_angle(angle: float) -> void:
	"""Set steering angle on vehicle body"""
	if _vehicle_body:
		_vehicle_body.set_steering_angle(clamp(angle, -max_steering_angle, max_steering_angle))

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================

func _handle_gear_shifting(delta: float) -> void:
	"""Automatic gear shifting based on RPM and speed"""
	if not manual_override_enabled:
		_auto_shift_gears(delta)
	
	# Apply clutch during gear changes
	_handle_clutch_for_gear_changes(delta)

func _auto_shift_gears(delta: float) -> void:
	"""Automatically determine optimal gear"""
	if _current_rpm < neutral_safe_zone:
		_target_gear = Gear.NONE
		return
	
	if _current_speed > 0.0:
		# Determine target gear based on RPM and speed
		var gear_ratio = _get_gear_ratio(_current_gear)
		
		if _current_rpm >= upshift_rpm_threshold:
			_target_gear = _calculate_next_gear(_current_gear, 1)  # upshift
		elif _current_rpm <= downshift_rpm_threshold and _current_gear > Gear.FIRST:
			_target_gear = _calculate_next_gear(_current_gear, -1)  # downshift
	else:
		_target_gear = Gear.NONE

func _calculate_next_gear(current_gear: Gear, direction: int) -> Gear:
	"""Calculate next gear based on direction (+1 = upshift, -1 = downshift)"""
	var new_gear: int = current_gear as int + direction
	
	# Validate gear range
	if new_gear < Gear.REVERSE:
		new_gear = Gear.REVERSE
	elif new_gear > Gear.EIGHTH:
		new_gear = Gear.EIGHTH
	
	return Gear(new_gear)

func shift_gear(gear: Gear, force: bool = false) -> void:
	"""Manually shift to specified gear"""
	if not force and _current_gear == gear:
		return
	
	var old_gear: int = _current_gear as int
	_target_gear = gear
	
	if _clutch_percent >= 0.95:
		# Full engagement - perform shift
		_current_gear = gear
		emit_signal("gear_changed", old_gear, gear as int)
	else:
		# Request shift but wait for clutch
		pass

func upshift(force: bool = false) -> void:
	"""Shift to next higher gear"""
	shift_gear(_calculate_next_gear(_current_gear, 1), force)

func downshift(force: bool = false) -> void:
	"""Shift to next lower gear"""
	shift_gear(_calculate_next_gear(_current_gear, -1), force)

func put_in_neutral() -> void:
	"""Put transmission in neutral"""
	shift_gear(Gear.NONE, true)

func put_in_reverse() -> void:
	"""Put transmission in reverse"""
	shift_gear(Gear.REVERSE, true)

# ============================================================================
# CLUTCH CONTROL
# ============================================================================

func engage_clutch(percent: float = 1.0) -> void:
	"""Engage clutch (0.0 to 1.0)"""
	_clutch_percent = clamp(percent, 0.0, 1.0)
	_clutch_engaged = (_clutch_percent >= 0.5)

func disengage_clutch() -> void:
	"""Fully disengage clutch"""
	engage_clutch(0.0)

func toggle_clutch() -> void:
	"""Toggle clutch between engaged/disengaged"""
	_clutch_percent = 1.0 if _clutch_percent < 0.5 else 0.0
	_clutch_engaged = (_clutch_percent >= 0.5)

func _handle_clutch_for_gear_changes(delta: float) -> void:
	"""Auto-clutch behavior during automatic shifts"""
	if _target_gear != _current_gear and _target_gear != Gear.NONE:
		# Disengage clutch for shift
		engage_clutch(0.0)
		
		# Check if shift is ready
		if _clutch_percent <= 0.0:
			_current_gear = _target_gear
			emit_signal("gear_changed", _current_gear as int - 1, _target_gear as int)
			engage_clutch(1.0)

# ============================================================================
# TRANSMISSION EFFICIENCY
# ============================================================================

func _update_transmission_efficiency() -> void:
	"""Update transmission efficiency based on clutch and gear"""
	if _clutch_percent < 0.95:
		# Slip losses when clutch partially engaged
		_front_wheel_torque *= _clutch_percent
		_rear_wheel_torque *= _clutch_percent

func _get_gear_ratio(gear: Gear) -> float:
	"""Get gear ratio for specified gear (typical racing ratios)"""
	match gear:
		Gear.REVERSE:
			return 3.5
		Gear.FIRST:
			return 3.8
		Gear.SECOND:
			return 2.5
		Gear.THIRD:
			return 1.8
		Gear.FOURTH:
			return 1.4
		Gear.FIFTH:
			return 1.1
		Gear.SIXTH:
			return 0.9
		Gear.SEVENTH:
			return 0.75
		Gear.EIGHTH:
			return 0.65
		_:
			return 1.0

func _get_final_drive_ratio() -> float:
	"""Get final drive ratio (typically around 3.5-4.0)"""
	return 3.7

func _get_wheel_radius() -> float:
	"""Get effective wheel radius in meters"""
	return 0.32  # ~32cm radius (~64cm diameter tire)

# ============================================================================
# WHEEL FORCE APPLICATION
# ============================================================================

func _apply_wheels(delta: float) -> void:
	"""Apply calculated forces to vehicle wheels"""
	if not _vehicle_body:
		return
	
	# Front wheel braking
	_front_wheel_brake_force = _smooth_brake * _input_brake * brake_bias_front
	
	# Rear wheel braking
	_rear_wheel_brake_force = _smooth_brake * _input_brake * (1.0 - brake_bias_front)
	
	# Apply to wheels
	_vehicle_body.apply_brake_force(
		_front_wheel_brake_force,
		_rear_wheel_brake_force
	)

func _get_current_steering_angle() -> float:
	"""Get current steering angle"""
	if _vehicle_body:
		return _vehicle_body.steering_angle
	return 0.0

func _set_steering_angle(angle: float) -> void:
	"""Set steering angle"""
	if _vehicle_body:
		_vehicle_body.steering_angle = clamp(angle, -max_steering_angle, max_steering_angle)

# ============================================================================
# VEHICLE STATE UPDATES
# ============================================================================

func _update_vehicle_state(delta: float) -> void:
	"""Update vehicle state based on physics"""
	if not _vehicle_body:
		return
	
	# Get current speed
	var velocity = _vehicle_body.linear_velocity
	_current_speed = velocity.length() * 3.6  # m/s to km/h
	
	# Calculate RPM based on gear and speed
	_current_rpm = _calculate_rpm_from_speed()
	
	# Emit signals
	emit_signal("speed_changed", _current_speed)
	emit_signal("rpm_changed", _current_rpm)
	emit_signal("vehicle_moving", _current_speed > 0.5)

func _calculate_rpm_from_speed() -> float:
	"""Calculate engine RPM from vehicle speed"""
	if _current_speed <= 0.0 or _clutch_percent < 0.1:
		return 800.0  # idle RPM
	
	var wheel_radius: float = _get_wheel_radius()
	var final_drive: float = _get_final_drive_ratio()
	var gear_ratio: float = _get_gear_ratio(_current_gear)
	
	# RPM = (speed / wheel_circumference) * gear_ratio * final_drive * 60
	var wheel_rps: float = _current_speed / (3.6 * 2.0 * PI * wheel_radius)
	var engine_rpm: float = wheel_rps * gear_ratio * final_drive * 60.0
	
	# Clamp to valid range
	return clamp(engine_rpm, 800.0, _physics_settings.max_engine_rpm)

# ============================================================================
# POWERTRAIN INTEGRATION
# ============================================================================

func _on_powertrain_rpm_changed(rpm: float) -> void:
	"""Handle RPM changes from powertrain"""
	_current_rpm = rpm
	emit_signal("rpm_changed", rpm)

func _on_powertrain_gear_changed(old_gear: int, new_gear: int) -> void:
	"""Handle gear changes from powertrain"""
	_current_gear = Gear(new_gear)
	emit_signal("gear_changed", old_gear, new_gear)

func apply_power_from_powertrain(torque: float) -> void:
	"""Apply torque from powertrain to wheels"""
	if _clutch_percent < 0.01:
		torque *= 0.0  # No power transfer when clutch disengaged
	else:
		torque *= _clutch_percent
	
	# Distribute torque to rear wheels (RWD typical)
	_rear_wheel_torque = torque * _clutch_percent

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================

func get_current_speed() -> float:
	"""Get current vehicle speed in km/h"""
	return _current_speed

func get_current_rpm() -> float:
	"""Get current engine RPM"""
	return _current_rpm

func get_current_gear() -> Gear:
	"""Get current gear"""
	return _current_gear

func is_vehicle_moving() -> bool:
	"""Check if vehicle is moving"""
	return _current_speed > 0.5

func is_clutch_engaged() -> bool:
	"""Check if clutch is engaged"""
	return _clutch_engaged

func get_clutch_percent() -> float:
	"""Get clutch engagement percentage"""
	return _clutch_percent

func set_vehicle_body(body: CharacterBody3D) -> void:
	"""Reference to vehicle body for physics operations"""
	_vehicle_body = body
	_init_physics_settings()

func get_brake_force_front() -> float:
	"""Get front wheel brake force"""
	return _front_wheel_brake_force

func get_brake_force_rear() -> float:
	"""Get rear wheel brake force"""
	return _rear_wheel_brake_force

func get_wheel_torque_front() -> float:
	"""Get front wheel torque"""
	return _front_wheel_torque

func get_wheel_torque_rear() -> float:
	"""Get rear wheel torque"""
	return _rear_wheel_torque

func reset_vehicle_state() -> void:
	"""Reset vehicle to initial state"""
	_input_throttle = 0.0
	_input_brake = 0.0
	_input_steer = 0.0
	_smooth_throttle = 0.0
	_smooth_brake = 0.0
	_smooth_steering = 0.0
	_current_speed = 0.0
	_current_rpm = 800.0  # idle
	_current_gear = Gear.NONE
	_target_gear = Gear.NONE
	_clutch_engaged = false
	_clutch_percent = 0.0
	_front_wheel_torque = 0.0
	_rear_wheel_torque = 0.0
	_front_wheel_brake_force = 0.0
	_rear_wheel_brake_force = 0.0

# ============================================================================
# DEBUG & TESTING
# ============================================================================

func debug_print_vehicle_state() -> void:
	"""Print current vehicle state for debugging"""
	print("[VehicleController] Speed: %.1f km/h | RPM: %.0f | Gear: %s" % [
		_current_speed,
		_current_rpm,
		_current_gear.name
	])
	print("[VehicleController] Throttle: %.2f | Brake: %.2f | Steer: %.2f" % [
		_smooth_throttle,
		_smooth_brake,
		_smooth_steering
	])
	print("[VehicleController] Clutch: %.2f | Engaged: %s" % [
		_clutch_percent,
		"yes" if _clutch_engaged else "no"
	])
</FILE>

{"op":"continue"}