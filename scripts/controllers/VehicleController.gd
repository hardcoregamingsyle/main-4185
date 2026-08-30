extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulation
## Handles throttle, brake, steering, gear shifting, wheel forces, and vehicle dynamics
## Integrates with PhysicsSettings singleton for all physics constants
## Copyright 2026 Thalamus Racing Simulator Project

# ============================================================================
# SIGNALS - Events emitted by this controller
# ============================================================================
signal speed_changed(new_speed: float)
signal rpm_changed(new_rpm: float)
signal gear_changed(old_gear: int, new_gear: int)
signal vehicle_moved(position: Vector3, velocity: Vector3)
signal collision_detected(collision_data: Dictionary)
signal engine_started()
signal engine_stopped()
signal handbrake_toggled(is_active: bool)
signal drift_started(angle: float)
signal drift_ended()

# ============================================================================
# ENUMS & CONSTANTS
# ============================================================================
enum DrivetrainType { FWD, RWD, AWD }
enum TransmissionType { MANUAL, AUTOMATIC, SEMI_AUTOMATIC }

const VEHICLE_BASE_MASS := 1500.0
const MIN_RPM := 800.0
const MAX_RPM := 8000.0
const IDLE_RPM := 900.0
const REDLINE_RPM := 7500.0
const NEUTRAL_GEAR := 0
const MAX_GEARS := 7
const GEAR_RATIO_NEUTRAL := 0.0
const GEAR_RATIO_1 := 3.8
const GEAR_RATIO_2 := 2.5
const GEAR_RATIO_3 := 1.7
const GEAR_RATIO_4 := 1.3
const GEAR_RATIO_5 := 1.0
const GEAR_RATIO_6 := 0.85
const GEAR_RATIO_7 := 0.7
const REVERSE_GEAR := -1
const CLIP_DISPLACEMENT_THRESHOLD := 1e-3

# ============================================================================
# PUBLIC PROPERTIES - Exposed for inspector and external access
# ============================================================================
@export_group("Vehicle Configuration")
@export var vehicle_mass: float = 1500.0: set = _set_vehicle_mass
@export var max_speed_kmh: float = 320.0: set = _set_max_speed_kmh
@export var acceleration_force: float = 8000.0: set = _set_acceleration_force
@export var braking_force: float = 15000.0: set = _set_braking_force
@export var steering_angle_max: float = 45.0: set = _set_steering_angle_max

@export_group("Drivetrain Settings")
@export var drivetrain_type: DrivetrainType = DrivetrainType.RWD
@export var final_drive_ratio: float = 3.5: set = _set_final_drive_ratio
@export var tire_radius: float = 0.33: set = _set_tire_radius
@export var torque_curve: Array[Vector2] = [
	Vector2(0.0, 0.0),    # RPM fraction -> Torque multiplier
	Vector2(0.2, 0.4),
	Vector2(0.4, 0.7),
	Vector2(0.6, 0.9),
	Vector2(0.7, 1.0),
	Vector2(0.8, 0.95),
	Vector2(0.9, 0.85),
	Vector2(1.0, 0.7)
]
@export var power_curve: Array[Vector2] = [
	Vector2(0.0, 0.0),    # RPM fraction -> Power multiplier
	Vector2(0.3, 0.2),
	Vector2(0.5, 0.5),
	Vector2(0.7, 0.85),
	Vector2(0.85, 1.0),
	Vector2(1.0, 0.9)
]

@export_group("Transmission Settings")
@export var transmission_type: TransmissionType = TransmissionType.MANUAL
@export var gear_ratios: Array[float] = [
	GEAR_RATIO_NEUTRAL,
	GEAR_RATIO_1,
	GEAR_RATIO_2,
	GEAR_RATIO_3,
	GEAR_RATIO_4,
	GEAR_RATIO_5,
	GEAR_RATIO_6,
	GEAR_RATIO_7,
	REVERSE_GEAR
]
@export var shift_points: Array[float] = [2000.0, 3500.0, 4500.0, 5500.0, 6500.0, 7200.0]

@export_group("Tire & Handling")
@export var grip_level: float = 1.0: set = _set_grip_level
@export var drift_coefficient: float = 0.3
@export var aerodynamic_drag: float = 0.32
@export var frontal_area: float = 2.2

# ============================================================================
# PRIVATE STATE VARIABLES
# ============================================================================
var _input_manager: InputManager
var _powertrain: Node = null
var _current_gear: int = NEUTRAL_GEAR
var _rpm: float = IDLE_RPM
var _engine_on: bool = false
var _throttle_input: float = 0.0
var _brake_input: float = 0.0
var _steering_input: float = 0.0
var _handbrake_input: bool = false
var _clutch_input: float = 1.0
var _engine_temperature: float = 90.0
var _oil_pressure: float = 0.0
var _last_position: Vector3 = Vector3.ZERO
var _collision_cooldown: float = 0.0
var _drift_angle: float = 0.0
var _slip_ratio: float = 0.0
var _last_rpm: float = 0.0

# ============================================================================
# GETTERS & SETTERS
# ============================================================================
func get_current_gear() -> int:
	return _current_gear

func get_rpm() -> float:
	return _rpm

func get_speed_kmh() -> float:
	return velocity.length() * 3.6

func get_velocity() -> Vector3:
	return velocity

func get_throttle_input() -> float:
	return _throttle_input

func get_brake_input() -> float:
	return _brake_input

func get_steering_input() -> float:
	return _steering_input

func is_engine_on() -> bool:
	return _engine_on

func is_in_gear() -> bool:
	return _current_gear != NEUTRAL_GEAR

# ============================================================================
# SETUP METHODS
# ============================================================================
func _ready() -> void:
	_init_dependencies()
	_setup_physics()
	_connect_signals()

func _init_dependencies() -> void:
	if GameManager.has_singleton("InputManager"):
		_input_manager = InputManager.get_singleton()
	else:
		push_warning("VehicleController: InputManager not available")

	if has_node("../Powertrain"):
		_powertrain = get_node("../Powertrain")
		_connect_powertrain_signals()

func _setup_physics() -> void:
	mass = vehicle_mass
	velocity = Vector3.ZERO
	_last_position = position

func _connect_signals() -> void:
	if _input_manager:
		_input_manager.input_connected.connect(_on_input_connected)

func _connect_powertrain_signals() -> void:
	if _powertrain:
		_powertrain.engine_ready.connect(_on_engine_ready)
		_powertrain.engine_fail.connect(_on_engine_fail)

# ============================================================================
# MAIN PHYSICS LOOP
# ============================================================================
func _physics_process(delta: float) -> void:
	if not Engine.is_editor_hint():
		_handle_input(delta)
		_update_engine_state(delta)
		_handle_gear_shifting(delta)
		_apply_forces(delta)
		_update_drivetrain(delta)
		_handle_collisions(delta)
		_emit_updates(delta)

# ============================================================================
# INPUT HANDLING
# ============================================================================
func _handle_input(delta: float) -> void:
	if _input_manager:
		_throttle_input = clamp(_input_manager.throttle_axis(), 0.0, 1.0)
		_brake_input = clamp(_input_manager.brake_axis(), 0.0, 1.0)
		_steering_input = clamp(_input_manager.steer_axis(), -1.0, 1.0)
		_handbrake_input = _input_manager.handbrake_pressed()
		_clutch_input = clamp(_input_manager.clutch_axis(), 0.0, 1.0)

# ============================================================================
# ENGINE MANAGEMENT
# ============================================================================
func _update_engine_state(delta: float) -> void:
	if not _engine_on:
		_rpm = MIN_RPM
		return

	var target_rpm = _calculate_target_rpm()
	_rpm = lerp(_rpm, target_rpm, delta * 10.0)
	_rpm = clamp(_rpm, MIN_RPM, MAX_RPM)

	# Simulate engine temperature
	_engine_temperature = lerp(_engine_temperature, 
		90.0 + (_throttle_input * 50.0), 
		delta * 0.1
	)

	# Simulate oil pressure based on RPM
	_oil_pressure = _rpm / 1000.0

func _calculate_target_rpm() -> float:
	var current_gear_ratio = gear_ratios[_current_gear] if _current_gear >= 0 else 1.0
	var wheel_speed_rpm = velocity.length() * 60.0 / (2.0 * PI * tire_radius)
	var engine_speed = wheel_speed_rpm * current_gear_ratio * final_drive_ratio

	if _throttle_input > 0.0:
		return engine_speed * 1.1
	elif _brake_input > 0.0:
		return engine_speed * 0.9
	else:
		return IDLE_RPM

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================
func _handle_gear_shifting(delta: float) -> void:
	if transmission_type == TransmissionType.AUTOMATIC:
		_auto_shift(delta)
	elif transmission_type == TransmissionType.SEMI_AUTOMATIC:
		_semi_auto_shift(delta)
	else:
		_manual_shift_check(delta)

func _auto_shift(delta: float) -> void:
	if _current_gear == NEUTRAL_GEAR:
		if _throttle_input > 0.0:
			shift_to_gear(1)
		return

	for i in range(shift_points.size()):
		if _rpm >= shift_points[i]:
			var next_gear = _current_gear + 1
			if next_gear < gear_ratios.size() and _throttle_input > 0.0:
				shift_to_gear(next_gear)
			break

	if _rpm <= MIN_RPM and _throttle_input < 0.5:
		var next_gear = _current_gear - 1
		if next_gear > 0:
			shift_to_gear(next_gear)
		elif _current_gear == 1 and velocity.length() < 5.0:
			shift_to_gear(NEUTRAL_GEAR)

func _semi_auto_shift(delta: float) -> void:
	if _input_manager.shift_up_requested():
		var next_gear = _current_gear + 1
		if next_gear < gear_ratios.size() and _throttle_input > 0.0:
			shift_to_gear(next_gear)
	elif _input_manager.shift_down_requested():
		var next_gear = _current_gear - 1
		if next_gear > 0:
			shift_to_gear(next_gear)

func _manual_shift_check(delta: float) -> void:
	pass

func shift_to_gear(gear: int) -> void:
	if gear < 0 or gear >= gear_ratios.size():
		return

	var old_gear = _current_gear
	_current_gear = gear

	if old_gear != gear:
		gear_changed.emit(old_gear, gear)
		if _powertrain:
			_powertrain.set_gear(gear)

# ============================================================================
# FORCE APPLICATION
# ============================================================================
func _apply_forces(delta: float) -> void:
	if not _engine_on:
		_apply_friction_and_drag(delta)
		return

	var force = _calculate_drive_force()
	var drive_vector = _get_forward_direction() * force

	match drivetrain_type:
		DrivetrainType.FWD:
			_apply_wheels_force(drive_vector, 2)
		DrivetrainType.RWD:
			_apply_wheels_force(drive_vector, 1)
		DrivetrainType.AWD:
			_apply_wheels_force(drive_vector * 0.5, 2)
			_apply_wheels_force(drive_vector * 0.5, 1)

	# Apply braking force
	if _brake_input > 0.0 or _handbrake_input:
		var brake_strength = braking_force * _brake_input
		if _handbrake_input:
			brake_strength *= 1.5
		_apply_brake_force(brake_strength)

	# Apply steering
	_apply_steering()

	# Apply drag and friction
	_apply_friction_and_drag(delta)

func _calculate_drive_force() -> float:
	var rpm_fraction = (_rpm - MIN_RPM) / (MAX_RPM - MIN_RPM)
	var torque_multiplier = _interpolate_value(torque_curve, rpm_fraction)
	var power_multiplier = _interpolate_value(power_curve, rpm_fraction)

	var base_torque = acceleration_force * 0.1
	var actual_torque = base_torque * torque_multiplier * _clutch_input

	var gear_ratio = gear_ratios[_current_gear] if _current_gear > 0 else 1.0
	var drive_force = actual_torque * gear_ratio * final_drive_ratio / tire_radius

	return drive_force * _throttle_input

func _apply_wheels_force(force: Vector3, wheels_count: int) -> void:
	var forward = transform.basis.z.normalized()
	force = forward * force.length()

	for i in range(wheels_count):
		add_force(force)

func _apply_brake_force(brake_strength: float) -> void:
	var brake_vector = -transform.basis.z.normalized() * brake_strength
	add_force(brake_vector)

func _apply_steering() -> void:
	if velocity.length() < 1.0:
		return

	var steer_angle = deg_to_rad(steering_angle_max * _steering_input * grip_level)
	var lateral_vector = transform.basis.x.normalized() * sin(steer_angle) * velocity.length()
	velocity += lateral_vector * 0.1

func _apply_friction_and_drag(delta: float) -> void:
	var drag_force = 0.5 * aerodynamic_drag * frontal_area * (velocity.length() ** 2)
	var drag_vector = -velocity.normalized() * drag_force * delta

	velocity += drag_vector
	velocity = lerp(velocity, Vector3.ZERO, delta * 2.0)

# ============================================================================
# DRIVETRAIN UPDATES
# ============================================================================
func _update_drivetrain(delta: float) -> void:
	if _powertrain:
		_powertrain.update_rpm(_rpm)
		_powertrain.update_gear(_current_gear)
		_powertrain.update_throttle(_throttle_input)

# ============================================================================
# COLLISION HANDLING
# ============================================================================
func _handle_collisions(delta: float) -> void:
	_collision_cooldown = max(0.0, _collision_cooldown - delta)

	for i in range(get_slide_collision_count()):
		var coll = get_slide_collision(i)
		var hit_normal = coll.get_normal()
		var hit_pos = coll.get_position()

		if _collision_cooldown > 0.0:
			continue

		var relative_velocity = velocity - coll.get_collider().get_linear_velocity()
		var impact_force = relative_velocity.length()

		if impact_force > 5.0:
			collision_detected.emit({
				"position": hit_pos,
				"normal": hit_normal,
				"impact_force": impact_force,
				"time": Time.get_ticks_msec()
			})
			_collision_cooldown = 0.2

		# Simple bounce response
		var bounce = hit_normal * (relative_velocity.dot(hit_normal)) * 0.3
		velocity -= bounce

# ============================================================================
# UPDATE EMITTERS
# ============================================================================
func _emit_updates(delta: float) -> void:
	var distance_traveled = position.distance_to(_last_position)

	if distance_traveled > CLIP_DISPLACEMENT_THRESHOLD:
		vehicle_moved.emit(position, velocity)

	if abs(velocity.length() - _last_position.distance_to(position) * 3.6) > 1.0:
		speed_changed.emit(get_speed_kmh())

	if abs(_rpm - _last_rpm) > 50.0:
		rpm_changed.emit(_rpm)

	_last_position = position
	_last_rpm = _rpm

# ============================================================================
# HELPER FUNCTIONS
# ============================================================================
func _interpolate_value(curve: Array[Vector2], value: float) -> float:
	if curve.is_empty():
		return 0.0

	if value <= curve[0].x:
		return curve[0].y
	if value >= curve[curve.size() - 1].x:
		return curve[curve.size() - 1].y

	for i in range(curve.size() - 1):
		if value >= curve[i].x and value <= curve[i + 1].x:
			var t = (value - curve[i].x) / (curve[i + 1].x - curve[i].x)
			return curve[i].y + t * (curve[i + 1].y - curve[i].y)

	return curve[-1].y

# ============================================================================
# PUBLIC CONTROL METHODS
# ============================================================================
func start_engine() -> void:
	_engine_on = true
	_rpm = IDLE_RPM
	engine_started.emit()

func stop_engine() -> void:
	_engine_on = false
	_rpm = MIN_RPM
	engine_stopped.emit()

func reset_vehicle() -> void:
	position = Vector3.ZERO
	velocity = Vector3.ZERO
	_current_gear = NEUTRAL_GEAR
	_rpm = IDLE_RPM
	_engine_on = false

func set_input_enabled(enabled: bool) -> void:
	pass

func get_performance_stats() -> Dictionary:
	return {
		"speed_kmh": get_speed_kmh(),
		"rpm": _rpm,
		"gear": _current_gear,
		"throttle": _throttle_input,
		"brake": _brake_input,
		"steering": _steering_input,
		"temperature": _engine_temperature,
		"oil_pressure": _oil_pressure
	}

# ============================================================================
# PROPERTY SETTERS
# ============================================================================
func _set_vehicle_mass(value: float) -> void:
	vehicle_mass = value
	mass = vehicle_mass

func _set_max_speed_kmh(value: float) -> void:
	max_speed_kmh = value

func _set_final_drive_ratio(value: float) -> void:
	final_drive_ratio = value

func _set_tire_radius(value: float) -> void:
	tire_radius = value

func _set_grip_level(value: float) -> void:
	grip_level = value

# ============================================================================
# DEBUG / TESTING HELPERS
# ============================================================================
func debug_set_rpm(rpm: float) -> void:
	_rpm = clamp(rpm, MIN_RPM, MAX_RPM)
	rpm_changed.emit(_rpm)

func debug_set_gear(gear: int) -> void:
	shift_to_gear(gear)

func debug_reset() -> void:
	reset_vehicle()

# ============================================================================
# LIFECYCLE EVENTS
# ============================================================================
func _on_input_connected(success: bool) -> void:
	if success and _input_manager:
		print("VehicleController: Input manager connected successfully")

func _on_engine_ready() -> void:
	start_engine()

func _on_engine_fail() -> void:
	stop_engine()

# ============================================================================
# END OF FILE
# VehicleController.gd - End of implementation