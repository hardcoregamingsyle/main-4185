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
signal lap_completed(lap_time: float)
signal checkpoint_passed(checkpoint_id: int)

# ============================================================================
# DRIVETRAIN TYPES
# ============================================================================
enum DrivetrainType {
	FWD,  # Front-Wheel Drive
	RWD,  # Rear-Wheel Drive
	AWD   # All-Wheel Drive
}

enum DifferentialType {
	OPEN,          # Standard open differential
	LSD,           # Limited Slip Differential
	ELECTRONIC_LSD, # Electronic LSD
	LOCKED         # Locked differential (no slip allowed)
}

enum GearState {
	NEUTRAL = 0,
	REVERSE = -1,
	FIRST = 1,
	SECOND = 2,
	THIRD = 3,
	FOURTH = 4,
	FIFTH = 5,
	SIXTH = 6,
	SEVENTH = 7,
	EIGHTH = 8,
	NINTH = 9,
	TENTH = 10
}

# ============================================================================
# CONSTANTS & CONFIGURATION
# ============================================================================
const VEHICLE_BASE_MASS := 1500.0
const MIN_RPM := 800.0
const IDLE_RPM := 1000.0
const MAX_RPM := 8000.0
const REDLINE_RPM := 7500.0
const GEAR_RATIO_FIRST := 3.8
const GEAR_RATIO_SECOND := 2.2
const GEAR_RATIO_THIRD := 1.5
const GEAR_RATIO_FOURTH := 1.1
const GEAR_RATIO_FIFTH := 0.9
const GEAR_RATIO_SIXTH := 0.75
const FINAL_DRIVE := 4.1
const WHEEL_RADIUS := 0.32
const STEERING_SPEED := 45.0
const TRACTION_CONTROL_STRENGTH := 0.95
const ABS_STRENGTH := 0.90

const GEAR_THRESHOLDS := [0.0, 15.0, 30.0, 50.0, 75.0, 100.0, 130.0, 160.0, 190.0, 220.0]
const GEAR_DOWN_THRESHOLDS := [0.0, 12.0, 25.0, 40.0, 60.0, 85.0, 110.0, 135.0, 160.0]
const SHIFT_DELAY := 0.15
const CLAMP_TORQUE := 0.92

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
@export var differential_type: DifferentialType = DifferentialType.OPEN
@export var final_drive_ratio: float = 3.85

@export_group("Aerodynamics")
@export var drag_coefficient: float = 0.32
@export var frontal_area: float = 2.2
@export var downforce_coefficient: float = 0.4

@export_group("Suspension")
@export var suspension_stiffness: float = 80000.0
@export var damping_rate: float = 3500.0
@export var spring_rest_length: float = 0.35

@export_group("Tires")
@export var tire_friction: float = 1.2
@export var tire_compliance: float = 0.005
@export var grip_factor: float = 1.0

# ============================================================================
# PRIVATE STATE VARIABLES
# ============================================================================
var _engine_running: bool = false
var _current_gear: GearState = GearState.NEUTRAL
var _target_gear: GearState = GearState.NEUTRAL
var _rpm: float = 0.0
var _throttle_input: float = 0.0
var _brake_input: float = 0.0
var _steering_input: float = 0.0
var _handbrake_active: bool = false
var _speed_kmh: float = 0.0
var _distance_traveled: float = 0.0
var _last_update_time: float = 0.0
var _shift_timer: float = 0.0
var _drift_angle: float = 0.0
var _is_drifting: bool = false
var _wheel_steering_angle: float = 0.0
var _wheel_rotation: float = 0.0
var _current_drag_force: float = 0.0
var _current_downforce: float = 0.0
var _air_density: float = 1.225
var _clutch_disengaged: bool = false
var _rev_matching: bool = false
var _abs_active: bool = false
var _tc_active: bool = false
var _traction_control_enabled: bool = true
var _abs_enabled: bool = true

# Powertrain reference
var powertrain_node: Node = null

# ============================================================================
# GODOT LIFECYCLE METHODS
# ============================================================================
func _ready() -> void:
	_process_mode = ProcessModeEnum.ALWAYS
	_init_powertrain_reference()
	_connect_signals_to_game_manager()
	_setup_physics_material()
	
	# Initialize RPM to idle if engine starts
	if _engine_running:
		_rpm = IDLE_RPM
	
	_last_update_time = Time.get_ticks_msec() / 1000.0

func _process(delta: float) -> void:
	if not Engine.is_editor_hint():
		_handle_inputs(delta)
		_update_engine_and_gears(delta)
		_calculate_wheel_forces(delta)
		_update_drift_state(delta)
		_apply_forces(delta)
		_synchronize_with_powertrain()

func _physics_process(delta: float) -> void:
	if not Engine.is_editor_hint():
		_handle_collision_detection()
		_update_position_based_on_velocity()
		_emit_movement_signal()

# ============================================================================
# INITIALIZATION AND SETUP
# ============================================================================
func _init_powertrain_reference() -> void:
	var parent = get_parent()
	if parent != null and parent.has_method("_get_powertrain"):
		powertrain_node = parent.call("_get_powertrain")
	elif parent != null and parent is Node:
		var children = parent.get_children()
		for child in children:
			if child.name.begins_with("Powertrain"):
				powertrain_node = child
				break

func _connect_signals_to_game_manager() -> void:
	GameManager.game_state_changed.connect(_on_game_state_changed)

func _setup_physics_material() -> void:
	var material = PhysicMaterial.new()
	material.friction = tire_friction * grip_factor
	material.bounce = 0.1
	material.thermal_conduction = 0.5
	material.shear_modulus = 1e6
	
	if has_method("set_physic_material"):
		set_physic_material(material)

# ============================================================================
# INPUT HANDLING
# ============================================================================
func _handle_inputs(delta: float) -> void:
	_read_input_axis(delta)
	_handle_gear_shifting(delta)
	_handle_handbrake()
	_handle_special_controls()

func _read_input_axis(delta: float) -> void:
	_throttle_input = InputManager.get_axis("throttle", 0.0, 1.0)
	_brake_input = InputManager.get_axis("brake", 0.0, 1.0)
	_steering_input = InputManager.get_axis("steering_left_right", -1.0, 1.0)
	
	# Clamp inputs to valid range
	_throttle_input = clamp(_throttle_input, 0.0, 1.0)
	_brake_input = clamp(_brake_input, 0.0, 1.0)
	_steering_input = clamp(_steering_input, -1.0, 1.0)

func _handle_gear_shifting(delta: float) -> void:
	_shift_timer -= delta
	
	if _shift_timer <= 0.0:
		_auto_shift_logic()
		_manual_shift_request()

func _auto_shift_logic() -> void:
	if _current_gear == GearState.NEUTRAL or _current_gear == GearState.REVERSE:
		return
		
	var up_shift_threshold = GEAR_THRESHOLDS[_current_gear as int] if _current_gear > 0 else 0.0
	var down_shift_threshold = GEAR_DOWN_THRESHOLDS[_current_gear as int] if _current_gear > 0 else 0.0
	
	if _throttle_input > 0.8 and _speed_kmh > up_shift_threshold:
		_request_upshift()
	elif _throttle_input < 0.2 and _speed_kmh < down_shift_threshold and _current_gear > GearState.FIRST:
		_request_downshift()

func _manual_shift_request() -> void:
	if Input.is_action_pressed("shift_up"):
		_request_upshift()
	if Input.is_action_pressed("shift_down"):
		_request_downshift()

func _request_upshift() -> void:
	if _current_gear < GearState.TENTH and _current_gear > GearState.NEUTRAL:
		_target_gear = GearState(_current_gear as int + 1)
		_shift_timer = SHIFT_DELAY

func _request_downshift() -> void:
	if _current_gear > GearState.FIRST:
		_target_gear = GearState(_current_gear as int - 1)
		_shift_timer = SHIFT_DELAY

func _handle_handbrake() -> void:
	var was_active = _handbrake_active
	_handbrake_active = Input.is_action_pressed("handbrake")
	
	if _handbrake_active != was_active:
		emit_signal("handbrake_toggled", _handbrake_active)

func _handle_special_controls() -> void:
	if Input.is_action_just_pressed("toggle_abs"):
		_abs_enabled = !_abs_enabled
		_update_abs_status()
	
	if Input.is_action_just_pressed("toggle_tc"):
		_traction_control_enabled = !_traction_control_enabled
		_update_tc_status()

# ============================================================================
# ENGINE AND GEAR MANAGEMENT
# ============================================================================
func _update_engine_and_gears(delta: float) -> void:
	_update_rpm(delta)
	_execute_gear_change()
	_update_engine_state()

func _update_rpm(delta: float) -> void:
	var target_rpm = _calculate_target_rpm()
	
	if _clutch_disengaged:
		_rpm = lerp(_rpm, IDLE_RPM, delta * 15.0)
	else:
		var rpm_change = (target_rpm - _rpm) * delta * 30.0
		_rpm += rpm_change
		_rpm = clamp(_rpm, MIN_RPM, MAX_RPM)
	
	if abs(target_rpm - _rpm) < 10.0:
		_rpm = target_rpm
	
	if rpm_changed.emit(_rpm) != null:
		pass

func _calculate_target_rpm() -> float:
	if _current_gear == GearState.NEUTRAL:
		return IDLE_RPM
	
	var wheel_speed_rad_per_sec = (_speed_kmh / 3.6) / WHEEL_RADIUS
	var gear_ratio = _get_gear_ratio(_current_gear)
	var total_ratio = gear_ratio * final_drive_ratio
	
	var target_rpm = wheel_speed_rad_per_sec * total_ratio * (60.0 / (2.0 * PI))
	
	if _throttle_input > 0.0:
		target_rpm = max(target_rpm, IDLE_RPM)
	
	return target_rpm

func _execute_gear_change() -> void:
	if _target_gear != _current_gear and _shift_timer <= 0.0:
		var old_gear = _current_gear
		_current_gear = _target_gear
		emit_signal("gear_changed", old_gear, _current_gear)
		
		if _current_gear == GearState.NEUTRAL:
			_clutch_disengaged = true
		else:
			_clutch_disengaged = false
		
		_target_gear = GearState.NEUTRAL

func _update_engine_state() -> void:
	if _throttle_input > 0.01 and !_engine_running:
		_engine_running = true
		emit_signal("engine_started")
	elif _throttle_input <= 0.01 and _speed_kmh < 5.0:
		_engine_running = false
		emit_signal("engine_stopped")

# ============================================================================
# WHEEL FORCE CALCULATIONS
# ============================================================================
func _calculate_wheel_forces(delta: float) -> void:
	if _current_gear == GearState.NEUTRAL:
		_apply_free_roll_forces(delta)
		return
	
	var drive_torque = _calculate_drive_torque()
	var wheel_torque = _apply_differential(drive_torque)
	var wheel_force = wheel_torque / WHEEL_RADIUS
	
	_apply_wheel_forces(wheel_force, delta)

func _calculate_drive_torque() -> float:
	var gear_ratio = _get_gear_ratio(_current_gear)
	var total_ratio = gear_ratio * final_drive_ratio
	
	var engine_torque = _calculate_engine_torque()
	engine_torque *= CLAMP_TORQUE
	
	var output_torque = engine_torque * total_ratio
	
	if _handbrake_active:
		output_torque *= 0.1
	
	return output_torque

func _calculate_engine_torque() -> float:
	var normalized_rpm = (_rpm - MIN_RPM) / (REDLINE_RPM - MIN_RPM)
	
	if normalized_rpm < 0.0:
		normalized_rpm = 0.0
	if normalized_rpm > 1.0:
		normalized_rpm = 1.0
	
	var torque_curve = -4.5 * pow(normalized_rpm, 2) + 7.0 * normalized_rpm + 0.5
	torque_curve = clamp(torque_curve, 0.3, 1.0)
	
	var throttle_multiplier = _throttle_input
	if _throttle_input > 0.0:
		throttle_multiplier = pow(_throttle_input, 0.6)
	
	var max_torque = acceleration_force * 0.15
	return max_torque * torque_curve * throttle_multiplier

func _apply_differential(drive_torque: float) -> float:
	match differential_type:
		DifferentialType.LOCKED:
			return drive_torque
		DifferentialType.LSD:
			return drive_torque * 0.85
		DifferentialType.ELECTRONIC_LSD:
			return drive_torque * 0.90
		_:
			return drive_torque * 0.75

func _apply_wheel_forces(wheel_force: float, delta: float) -> void:
	match drivetrain_type:
		DrivetrainType.FWD:
			_apply_front_wheel_drive(wheel_force, delta)
		DrivetrainType.RWD:
			_apply_rear_wheel_drive(wheel_force, delta)
		DrivetrainType.AWD:
			_apply_all_wheel_drive(wheel_force, delta)

func _apply_front_wheel_drive(wheel_force: float, delta: float) -> void:
	var forward_direction = global_transform.basis.z * -1.0
	force_applied(forward_direction * wheel_force, Vector3.ZERO, ForceMode.ACCELERATION)

func _apply_rear_wheel_drive(wheel_force: float, delta: float) -> void:
	var forward_direction = global_transform.basis.z * -1.0
	force_applied(forward_direction * wheel_force, Vector3.ZERO, ForceMode.ACCELERATION)

func _apply_all_wheel_drive(wheel_force: float, delta: float) -> void:
	var forward_direction = global_transform.basis.z * -1.0
	force_applied(forward_direction * (wheel_force * 0.6), Vector3.ZERO, ForceMode.ACCELERATION)

func _apply_free_roll_forces(delta: float) -> void:
	var air_resistance = calculate_air_resistance()
	velocity = velocity * (1.0 - air_resistance * delta)

# ============================================================================
# BRAKING SYSTEM
# ============================================================================
func apply_braking(force_multiplier: float) -> void:
	var braking_effectiveness = braking_force * force_multiplier
	
	if _handbrake_active:
		braking_effectiveness *= 0.4
	
	if _abs_active and _abs_enabled:
		braking_effectiveness *= ABS_STRENGTH
	
	var braking_direction = global_transform.basis.z * -1.0
	force_applied(braking_direction * braking_effectiveness, Vector3.ZERO, ForceMode.ACCELERATION)

func update_abs_status() -> void:
	_abs_active = _abs_enabled and velocity.length() > 2.0

func _update_abs_status() -> void:
	_abs_active = _abs_enabled

func _update_tc_status() -> void:
	_tc_active = _traction_control_enabled and _throttle_input > 0.5

# ============================================================================
# DRIFT PHYSICS
# ============================================================================
func _update_drift_state(delta: float) -> void:
	if _handbrake_active and _throttle_input > 0.3:
		var slip_angle = _calculate_slip_angle()
		
		if abs(slip_angle) > 15.0:
			if not _is_drifting:
				_is_drifting = true
				_drift_angle = slip_angle
				emit_signal("drift_started", _drift_angle)
			else:
				_drift_angle = lerp(_drift_angle, slip_angle, delta * 2.0)
		else:
			if _is_drifting:
				_is_drifting = false
				emit_signal("drift_ended")
	else:
		if _is_drifting:
			_is_drifting = false
			emit_signal("drift_ended")

func _calculate_slip_angle() -> float:
	var forward_vector = global_transform.basis.z * -1.0
	var velocity_unit = velocity.normalized() if velocity.length() > 0.1 else Vector3.ZERO
	
	var dot_product = forward_vector.dot(velocity_unit)
	var cross_product = forward_vector.cross(velocity_unit).dot(Vector3.UP)
	
	var angle_rad = atan2(cross_product, dot_product)
	return degrees(angle_rad)

# ============================================================================
# AERODYNAMICS
# ============================================================================
func calculate_air_resistance() -> float:
	var speed_ms = _speed_kmh / 3.6
	
	if speed_ms <= 0.0:
		return 0.0
	
	var dynamic_pressure = 0.5 * _air_density * pow(speed_ms, 2)
	
	_current_drag_force = dynamic_pressure * drag_coefficient * frontal_area
	_current_downforce = dynamic_pressure * downforce_coefficient * frontal_area
	
	return _current_drag_force

func apply_aerodynamic_forces() -> void:
	if _speed_kmh <= 0.0:
		return
	
	var drag_direction = -velocity.normalized()
	var downforce_direction = -Vector3.UP
	
	force_applied(drag_direction * _current_drag_force, Vector3.ZERO, ForceMode.ACCELERATION)
	force_applied(downforce_direction * _current_downforce, Vector3.ZERO, ForceMode.ACCELERATION)

# ============================================================================
# POSITION AND VELOCITY UPDATE
# ============================================================================
func _update_position_based_on_velocity() -> void:
	var position_before = global_position
	
	velocity.y = -PhysicsSettings.gravity
	
	if is_colliding():
		var col = get_collision_normal()
		velocity = velocity.project(col)
	
	global_position += velocity
	
	var distance_moved = (global_position - position_before).length()
	_distance_traveled += distance_moved

func _emit_movement_signal() -> void:
	if _last_update_time > 0:
		var time_delta = (Time.get_ticks_msec() / 1000.0) - _last_update_time
		if time_delta > 0.016:
			emit_signal("vehicle_moved", global_position, velocity)
			_last_update_time = Time.get_ticks_msec() / 1000.0

func _handle_collision_detection() -> void:
	if is_colliding():
		var collision_info = {
			"collision_point": get_collision_point(),
			"collision_normal": get_collision_normal(),
			"collision_object": get_collider()
		}
		emit_signal("collision_detected", collision_info)

# ============================================================================
# GEAR RATIO HELPERS
# ============================================================================
func _get_gear_ratio(gear: GearState) -> float:
	match gear:
		GearState.FIRST: return GEAR_RATIO_FIRST
		GearState.SECOND: return GEAR_RATIO_SECOND
		GearState.THIRD: return GEAR_RATIO_THIRD
		GearState.FOURTH: return GEAR_RATIO_FOURTH
		GearState.FIFTH: return GEAR_RATIO_FIFTH
		GearState.SIXTH: return GEAR_RATIO_SIXTH
		GearState.SEVENTH: return 0.65
		GearState.EIGHTH: return 0.58
		GearState.NINTH: return 0.52
		GearState.TENTH: return 0.48
		_: return 1.0

func get_current_gear() -> GearState:
	return _current_gear

func get_current_rpm() -> float:
	return _rpm

func get_current_speed_kmh() -> float:
	return _speed_kmh

func get_total_distance() -> float:
	return _distance_traveled

# ============================================================================
# POWERTRAIN SYNCHRONIZATION
# ============================================================================
func _synchronize_with_powertrain() -> void:
	if powertrain_node != null:
		if powertrain_node.has_method("_sync_controller_state"):
			powertrain_node._sync_controller_state(
				_rpm, _current_gear, _throttle_input, _engine_running
			)

# ============================================================================
# PUBLIC API - External control methods
# ============================================================================
func start_engine() -> void:
	if not _engine_running:
		_engine_running = true
		_rpm = IDLE_RPM
		emit_signal("engine_started")

func stop_engine() -> void:
	if _engine_running:
		_engine_running = false
		_rpm = MIN_RPM
		emit_signal("engine_stopped")

func set_gear(gear: GearState) -> void:
	if gear >= GearState.FIRST and gear <= GearState.TENTH:
		_current_gear = gear
		_target_gear = gear
		_shift_timer = 0.0
		emit_signal("gear_changed", GearState.NEUTRAL, gear)
	elif gear == GearState.NEUTRAL or gear == GearState.REVERSE:
		_current_gear = gear
		_target_gear = gear

func manual_shift(direction: int) -> void:
	if direction > 0 and _current_gear < GearState.TENTH:
		_target_gear = GearState(_current_gear as int + 1)
	elif direction < 0 and _current_gear > GearState.FIRST:
		_target_gear = GearState(_current_gear as int - 1)
	_shift_timer = SHIFT_DELAY

func reset_vehicle() -> void:
	_engine_running = false
	_current_gear = GearState.NEUTRAL
	_target_gear = GearState.NEUTRAL
	_rpm = MIN_RPM
	_throttle_input = 0.0
	_brake_input = 0.0
	_steering_input = 0.0
	_handbrake_active = false
	_speed_kmh = 0.0
	_distance_traveled = 0.0
	_is_drifting = false
	_drift_angle = 0.0
	_clutch_disengaged = true
	_abs_active = false
	_tc_active = false

# ============================================================================
# PROPERTY SETTERS WITH VALIDATION
# ============================================================================
func _set_vehicle_mass(value: float) -> void:
	vehicle_mass = value
	if has_method("set_mass"):
		set_mass(vehicle_mass)

func _set_max_speed_kmh(value: float) -> void:
	max_speed_kmh = max(value, 10.0)

func _set_acceleration_force(value: float) -> void:
	acceleration_force = max(value, 1000.0)

func _set_braking_force(value: float) -> void:
	braking_force = max(value, 1000.0)

func _set_steering_angle_max(value: float) -> void:
	steering_angle_max = clamp(value, 10.0, 90.0)

# ============================================================================
# GAME MANAGER SIGNAL HANDLERS
# ============================================================================
func _on_game_state_changed(new_state: GameState) -> void:
	match new_state:
		GameState.RACE_ACTIVE:
			start_engine()
		GameState.RACE_PAUSED:
			pass
		GameState.RACE_FINISHED:
			stop_engine()

# ============================================================================
# DEBUG / TESTING HELPERS
# ============================================================================
func debug_get_statistics() -> Dictionary:
	return {
		"rpm": _rpm,
		"gear": _current_gear,
		"speed_kmh": _speed_kmh,
		"throttle": _throttle_input,
		"brake": _brake_input,
		"driving": _engine_running,
		"drifting": _is_drifting,
		"distance_meters": _distance_traveled,
		"drag_force": _current_drag_force,
		"downforce": _current_downforce
	}

func debug_set_test_conditions() -> void:
	_rpm = 5000.0
	_current_gear = GearState.FOURTH
	_throttle_input = 0.75
	_speed_kmh = 120.0

</File>