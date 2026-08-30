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
signal powertrain_connected(powertrain: Node)
signal suspension_bumped(wheel_index: int, compression: float)

# ============================================================================
# DRIVETRAIN TYPES
# ============================================================================
enum DrivetrainType {
	FWD,  # Front-Wheel Drive
	RWD,  # Rear-Wheel Drive
	AWD   # All-Wheel Drive
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
const MAX_RPM := 8000.0
const IDLE_RPM := 800.0
const REDLINE_RPM := 7500.0
const NEUTRAL_RPM := 1200.0
const MAX_STEER_ANGLE := PI / 4  # 45 degrees max steering
const STEER_SPEED := 3.0
const BRAKE_FORCE_MAX := 15000.0
const HAND_BRAKE_FORCE := 8000.0

# ============================================================================
# EXPORTED VARIABLES - Exposed in Godot Editor
# ============================================================================
@export_group("Vehicle Configuration")
@export var drivetrain_type: DrivetrainType = DrivetrainType.RWD
@export var mass: float = VEHICLE_BASE_MASS
@export var center_of_mass_offset: Vector3 = Vector3.ZERO
@export var wheel_base: float = 2.5
@export var track_width: float = 1.6
@export var wheel_radius: float = 0.32

@export_group("Engine & Transmission")
@export var max_power: float = 300000.0  # Watts (approx 400 HP)
@export var max_torque: float = 500.0    # Nm
@export var redline_rpm: float = REDLINE_RPM
@export var idle_rpm: float = IDLE_RPM
@export var final_drive_ratio: float = 3.73
@export var gear_ratios: Array[float] = [
	3.50, 2.10, 1.50, 1.10, 0.85, 0.68, 0.55, 0.45, 0.38, 0.32
]

@export_group("Steering & Suspension")
@export var max_steering_angle: float = MAX_STEER_ANGLE
@export var steering_sensitivity: float = 1.0
@export var suspension_stiffness: float = 35000.0
@export var suspension_damping: float = 2500.0
@export var suspension_travel_max: float = 0.15
@export var suspension_compression_max: float = 0.12

@export_group("Brakes & Tires")
@export var brake_force_multiplier: float = 1.0
@export var tire_friction_coefficient: float = 1.1
@export var grip_loss_threshold: float = 0.85
@export var drift_friction_reduction: float = 0.3

@export_group("AI & Difficulty")
@export var is_player_controlled: bool = true
@export var ai_enabled: bool = false
@export var difficulty_level: int = 5  # 1-10 scale

# ============================================================================
# PRIVATE STATE VARIABLES
# ============================================================================
var _current_gear: int = GearState.NEUTRAL
var _target_gear: int = GearState.NEUTRAL
var _rpm: float = IDLE_RPM
var _vehicle_speed: float = 0.0  # m/s
var _forward_speed: float = 0.0
var _lateral_speed: float = 0.0
var _engine_on: bool = false
var _handbrake_active: bool = false
var _throttle_input: float = 0.0
var _brake_input: float = 0.0
var _steering_input: float = 0.0
var _clutch_disengaged: bool = false
var _drift_mode: bool = false
var _drift_angle: float = 0.0

# Wheel state tracking
var _wheel_states: Array[Dictionary] = []
var _wheel_contact_points: Array[Vector3] = []
var _wheel_forces: Array[Vector3] = []

# Powertrain reference
var _powertrain_node: Node = null

# Input state
var _input_accumulator: Dictionary = {}
var _last_update_time: float = 0.0

# Physics references
var _rigid_body_3d: RigidBody3D = null
var _suspension_nodes: Array[Node3D] = []
var _collision_objects: Array[CollisionObject3D] = []

# Timing and lap data
var _race_start_time: float = 0.0
var _lap_times: Array[float] = []
var _current_lap_time: float = 0.0
var _last_checkpoint_time: float = 0.0
var _checkpoint_ids: Array[int] = []

# Debug state
var _debug_overlay_visible: bool = false
var _physics_debug_mode: bool = false

# ============================================================================
# PUBLIC PROPERTIES
# ============================================================================
func get_current_gear() -> int:
	return _current_gear

func get_target_gear() -> int:
	return _target_gear

func get_rpm() -> float:
	return _rpm

func get_vehicle_speed() -> float:
	return _vehicle_speed * 3.6  # Convert to km/h

func get_forward_speed() -> float:
	return _forward_speed

func get_lateral_speed() -> float:
	return _lateral_speed

func get_throttle_input() -> float:
	return _throttle_input

func get_brake_input() -> float:
	return _brake_input

func get_steering_input() -> float:
	return _steering_input

func get_handbrake_active() -> bool:
	return _handbrake_active

func get_engine_on() -> bool:
	return _engine_on

func get_drift_mode() -> bool:
	return _drift_mode

func get_drift_angle() -> float:
	return _drift_angle

func get_current_wheel_forces() -> Array[Vector3]:
	return _wheel_forces.duplicate()

func get_wheel_contact_points() -> Array[Vector3]:
	return _wheel_contact_points.duplicate()

func set_driver_name(name: String) -> void:
	pass  # Placeholder for driver name setting

func reset_to_position(position: Vector3, rotation: Vector3) -> void:
	position = position
	rotation = rotation
	set_position(position)
	set_rotation(rotation)
	velocity = Vector3.ZERO
	angular_velocity = Vector3.ZERO
	_current_gear = GearState.NEUTRAL
	_target_gear = GearState.NEUTRAL
	_rpm = IDLE_RPM
	_vehicle_speed = 0.0
	force_set_gear(GearState.NEUTRAL)

# ============================================================================
# INITIALIZATION & SETUP
# ============================================================================
func _ready() -> void:
	_init_rigid_body()
	_init_wheels()
	_init_suspension()
	_init_powertrain_connection()
	_connect_signals_to_game_manager()
	_load_saved_state()

func _process(delta: float) -> void:
	if not _engine_on and not is_player_controlled:
		return
	
	_process_input(delta)
	_update_physics(delta)
	_handle_gear_shifting(delta)
	_update_debug_overlay()

func _physics_process(delta: float) -> void:
	if _rigid_body_3d != null:
		_apply_physics_to_rigidbody(delta)
		_update_wheel_states(delta)
		_check_collisions()

# ============================================================================
# INPUT PROCESSING
# ============================================================================
func _process_input(delta: float) -> void:
	var input_map = InputManager if InputManager else null
	
	if input_map == null:
		return
	
	# Read throttle input
	_throttle_input = clampf(input_map.get_axis("throttle_up", "brake_down"), -1.0, 1.0)
	_throttle_input = _throttle_input if _throttle_input > 0 else 0.0
	
	# Read brake input
	_brake_input = clampf(input_map.get_axis("brake_up", "throttle_down"), -1.0, 1.0)
	_brake_input = _brake_input if _brake_input > 0 else 0.0
	
	# Read steering input
	_steering_input = clampf(input_map.get_axis("steer_left", "steer_right"), -1.0, 1.0)
	_steering_input *= steering_sensitivity
	
	# Handbrake input
	var handbrake_pressed = input_map.is_action_pressed("handbrake")
	if handbrake_pressed != _handbrake_active:
		_handbrake_active = handbrake_pressed
		handbrake_toggled.emit(_handbrake_active)
	
	# Clutch input (for manual transmission)
	var clutch_pressed = input_map.is_action_pressed("clutch")
	_clutch_disengaged = clutch_pressed
	
	# Gear selection input
	_handle_gear_selection_input(input_map)
	
	# Engine start/stop
	_handle_engine_input(input_map)
	
	# Drift toggle
	var drift_toggle = input_map.is_action_pressed("toggle_drift")
	if drift_toggle and not _drift_mode:
		_enter_drift_mode()
	elif drift_toggle and _drift_mode:
		_exit_drift_mode()

func _handle_gear_selection_input(input_map) -> void:
	if _clutch_disengaged or _current_gear == GearState.NEUTRAL:
		var up_shift = input_map.is_action_pressed("shift_up")
		var down_shift = input_map.is_action_pressed("shift_down")
		
		if up_shift:
			_request_shift_up()
		elif down_shift:
			_request_shift_down()

func _handle_engine_input(input_map) -> void:
	if input_map.is_action_pressed("start_engine") and not _engine_on:
		_start_engine()
	elif input_map.is_action_pressed("stop_engine") and _engine_on:
		_stop_engine()

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================
func _request_shift_up() -> void:
	if _current_gear < GearState.TENTH and _current_gear >= GearState.FIRST:
		_target_gear = min(_current_gear + 1, GearState.TENTH)
	elif _current_gear <= GearState.FIRST and _target_gear == GearState.NEUTRAL:
		_target_gear = GearState.FIRST
	_target_gear = clampi(_target_gear, GearState.FIRST, GearState.TENTH)

func _request_shift_down() -> void:
	if _current_gear > GearState.FIRST:
		_target_gear = max(_current_gear - 1, GearState.FIRST)
	elif _current_gear == GearState.FIRST:
		_target_gear = GearState.NEUTRAL
	_target_gear = clampi(_target_gear, GearState.NEUTRAL, GearState.TENTH)

func _handle_gear_shifting(delta: float) -> void:
	if _current_gear == _target_gear:
		return
	
	if _clutch_disengaged:
		_perform_gear_change()
	else:
		# Auto-shifting when not using clutch
		_auto_shift()

func _auto_shift() -> void:
	var target_rpm = _calculate_target_rpm()
	
	if target_rpm > REDLINE_RPM and _current_gear < GearState.TENTH:
		_request_shift_up()
	elif target_rpm < IDLE_RPM and _current_gear > GearState.FIRST:
		_request_shift_down()

func _calculate_target_rpm() -> float:
	var wheel_speed = _forward_speed / (2.0 * PI * wheel_radius)
	var engine_rpm = wheel_speed * gear_ratios[_current_gear - 1] * final_drive_ratio * 60.0
	return maxf(engine_rpm, IDLE_RPM)

func _perform_gear_change() -> void:
	var old_gear = _current_gear
	_current_gear = _target_gear
	
	if old_gear != _current_gear:
		gear_changed.emit(old_gear, _current_gear)
		
		# Apply RPM drop/rise based on gear change
		if _current_gear == GearState.NEUTRAL:
			_rpm = NEUTRAL_RPM
		elif _current_gear < old_gear:  # Downshift
			_rpm = minf(_rpm * (old_gear / _current_gear), REDLINE_RPM)
		else:  # Upshift
			_rpm = maxf(_rpm * (_current_gear / old_gear), IDLE_RPM)

func force_set_gear(gear: int) -> void:
	var old_gear = _current_gear
	_current_gear = gear
	_target_gear = gear
	
	if old_gear != _current_gear:
		gear_changed.emit(old_gear, _current_gear)
		
		if _current_gear == GearState.NEUTRAL:
			_rpm = NEUTRAL_RPM
		else:
			_rpm = IDLE_RPM

# ============================================================================
# ENGINE MANAGEMENT
# ============================================================================
func _start_engine() -> void:
	_engine_on = true
	engine_started.emit()
	_rpm = IDLE_RPM

func _stop_engine() -> void:
	_engine_on = false
	engine_stopped.emit()
	_rpm = IDLE_RPM
	_current_gear = GearState.NEUTRAL
	_target_gear = GearState.NEUTRAL

func _update_engine_rpm(delta: float) -> void:
	if not _engine_on:
		_rpm = IDLE_RPM
		return
	
	var target_rpm = _calculate_target_rpm_from_gear()
	
	# Smooth RPM transition
	var rpm_change_rate = delta * 2000.0
	var new_rpm = _rpm
	
	if target_rpm > _rpm:
		new_rpm = minf(_rpm + rpm_change_rate, target_rpm)
	elif target_rpm < _rpm:
		new_rpm = maxf(_rpm - rpm_change_rate, target_rpm)
	
	_rpm = new_rpm
	
	rpm_changed.emit(_rpm)

func _calculate_target_rpm_from_gear() -> float:
	if _current_gear == GearState.NEUTRAL:
		return NEUTRAL_RPM
	
	var wheel_speed = _forward_speed / (2.0 * PI * wheel_radius)
	var engine_rpm = wheel_speed * gear_ratios[_current_gear - 1] * final_drive_ratio * 60.0
	
	# Clamp to valid range
	engine_rpm = clampf(engine_rpm, IDLE_RPM, REDLINE_RPM)
	
	return engine_rpm

# ============================================================================
# WHEEL PHYSICS & FORCES
# ============================================================================
func _init_wheels() -> void:
	_wheel_states.clear()
	_wheel_contact_points.clear()
	_wheel_forces.clear()
	
	# Create wheel states for each of the 4 wheels
	for i in range(4):
		_wheel_states.append({
			"index": i,
			"position": Vector3.ZERO,
			"force": Vector3.ZERO,
			"angle": 0.0,
			"slip_ratio": 0.0,
			"slip_angle": 0.0,
			"contact_normal": Vector3.UP,
			"is_in_contact": false,
			"suspension_compression": 0.0,
			"wheel_rotation": 0.0
		})
		_wheel_contact_points.append(Vector3.ZERO)
		_wheel_forces.append(Vector3.ZERO)

func _update_wheel_states(delta: float) -> void:
	for i in range(4):
		_update_single_wheel_state(i, delta)

func _update_single_wheel_state(wheel_index: int, delta: float) -> void:
	var wheel_state = _wheel_states[wheel_index]
	var wheel_pos = _get_wheel_world_position(wheel_index)
	
	# Calculate wheel slip ratio
	var wheel_linear_speed = _calculate_wheel_linear_speed(wheel_index)
	var slip_ratio = _calculate_slip_ratio(wheel_linear_speed, wheel_state.force.length())
	wheel_state.slip_ratio = slip_ratio
	
	# Calculate lateral slip angle
	var lateral_velocity = _calculate_lateral_velocity_at_wheel(wheel_index)
	var wheel_heading = _get_wheel_heading(wheel_index)
	var slip_angle = atan2(lateral_velocity, abs(wheel_linear_speed) + 0.01)
	wheel_state.slip_angle = slip_angle
	
	# Update suspension compression
	var target_compression = _calculate_suspension_compression(wheel_index)
	var current_compression = wheel_state.suspension_compression
	var suspension_delta = clampf(target_compression - current_compression, -delta * suspension_stiffness, delta * suspension_stiffness)
	wheel_state.suspension_compression = current_compression + suspension_delta
	
	# Clamp suspension travel
	wheel_state.suspension_compression = clampf(wheel_state.suspension_compression, -suspension_travel_max, suspension_travel_max)
	
	# Emit suspension bump signal
	if abs(suspension_delta) > 0.01:
		suspension_bumped.emit(wheel_index, wheel_state.suspension_compression)

func _calculate_wheel_linear_speed(wheel_index: int) -> float:
	var wheel_is_driven = _is_wheel_driven(wheel_index)
	var wheel_is_steered = wheel_index < 2  # Front wheels are steered
	
	if wheel_is_driven and _current_gear != GearState.NEUTRAL and _current_gear != GearState.REVERSE:
		var gear_ratio = gear_ratios[_current_gear - 1]
		var drive_speed = _rpm / 60.0 * 2.0 * PI * wheel_radius / (gear_ratio * final_drive_ratio)
		return drive_speed
	elif wheel_is_driven and (_current_gear == GearState.REVERSE or _current_gear == GearState.NEUTRAL):
		return 0.0
	else:
		return _forward_speed

func _calculate_slip_ratio(wheel_linear_speed: float, wheel_force_length: float) -> float:
	if wheel_linear_speed == 0.0:
		return 0.0
	
	var target_speed = _vehicle_speed
	if _current_gear == GearState.REVERSE:
		target_speed = -abs(_vehicle_speed)
	
	var slip = (wheel_linear_speed - target_speed) / maxf(abs(target_speed), 1.0)
	return clampf(slip, -1.0, 1.0)

func _calculate_lateral_velocity_at_wheel(wheel_index: int) -> float:
	var angular_velocity_z = angular_velocity.z
	var lateral_offset = track_width / 2.0 if wheel_index % 2 == 0 else -track_width / 2.0
	return _lateral_speed + angular_velocity_z * lateral_offset

func _get_wheel_world_position(wheel_index: int) -> Vector3:
	var front_rear_offset = wheel_base / 2.0 if wheel_index < 2 else -wheel_base / 2.0
	var left_right_offset = track_width / 2.0 if wheel_index % 2 == 0 else -track_width / 2.0
	
	var local_offset = Vector3(front_rear_offset, 0.0, left_right_offset)
	local_offset.y += suspension_travel_max - _wheel_states[wheel_index].suspension_compression
	
	return global_transform * local_offset

func _get_wheel_heading(wheel_index: int) -> Vector3:
	var wheel_local_basis = basis.rotated(Vector3.RIGHT, -PI / 2)
	var steering_angle = _steering_input * max_steering_angle if wheel_index < 2 else 0.0
	var steer_basis = wheel_local_basis.rotated(Vector3.UP, steering_angle)
	
	return steer_basis.x.normalized()

func _is_wheel_driven(wheel_index: int) -> bool:
	match drivetrain_type:
		DrivetrainType.FWD:
			return wheel_index < 2
		DrivetrainType.RWD:
			return wheel_index >= 2
		DrivetrainType.AWD:
			return true
	return false

# ============================================================================
# FORCE APPLICATION TO RIGID BODY
# ============================================================================
func _apply_physics_to_rigidbody(delta: float) -> void:
	if _rigid_body_3d == null:
		return
	
	_clear_applied_forces()
	_calculate_wheel_forces(delta)
	_apply_wheel_forces_to_rigidbody()
	_apply_aerodynamic_forces(delta)
	_apply_gravity()

func _clear_applied_forces() -> void:
	_wheel_forces.fill(Vector3.ZERO)

func _calculate_wheel_forces(delta: float) -> void:
	for i in range(4):
		_calculate_single_wheel_force(i, delta)

func _calculate_single_wheel_force(wheel_index: int, delta: float) -> void:
	var wheel_state = _wheel_states[wheel_index]
	var wheel_force = Vector3.ZERO
	
	# Longitudinal force (acceleration/braking)
	var longitudinal_force = _calculate_longitudinal_force(wheel_index, delta)
	wheel_force.x = longitudinal_force
	
	# Lateral force (cornering)
	var lateral_force = _calculate_lateral_force(wheel_index, delta)
	wheel_force.z = lateral_force
	
	# Vertical force (weight transfer)
	var vertical_force = _calculate_vertical_force(wheel_index, delta)
	wheel_force.y = vertical_force
	
	_wheel_forces[wheel_index] = wheel_force

func _calculate_longitudinal_force(wheel_index: int, delta: float) -> float:
	var wheel_is_driven = _is_wheel_driven(wheel_index)
	var wheel_state = _wheel_states[wheel_index]
	
	if not wheel_is_driven:
		return 0.0
	
	var drive_force = 0.0
	var braking_force = 0.0
	
	if _current_gear == GearState.NEUTRAL or _current_gear == GearState.REVERSE:
		drive_force = 0.0
	else:
		# Calculate engine torque contribution
		var engine_torque = _calculate_engine_torque()
		var gear_ratio = gear_ratios[_current_gear - 1] if _current_gear > GearState.NEUTRAL else 1.0
		var effective_torque = engine_torque * gear_ratio * final_drive_ratio
		
		# Convert torque to force at wheel contact patch
		drive_force = effective_torque / wheel_radius
		
		# Apply throttle input multiplier
		drive_force *= _throttle_input
		
		# Limit maximum drive force
		drive_force = minf(drive_force, max_power / maxf(_vehicle_speed, 1.0))
	
	# Braking force
	if _brake_input > 0 or _handbrake_active:
		var brake_force = BRAKE_FORCE_MAX * _brake_input * brake_force_multiplier
		if _handbrake_active and (wheel_index >= 2):  # Handbrake affects rear wheels
			brake_force += HAND_BRAKE_FORCE * 0.5
		braking_force = brake_force
	
	# Combine drive and brake forces
	var total_longitudinal = drive_force - braking_force
	
	# Apply tire friction model
	var friction_limit = tire_friction_coefficient * abs(wheel_state.contact_normal.y) * mass / 4.0
	total_longitudinal = clampf(total_longitudinal, -friction_limit, friction_limit)
	
	return total_longitudinal

func _calculate_lateral_force(wheel_index: int, delta: float) -> float:
	var wheel_state = _wheel_states[wheel_index]
	var slip_angle = wheel_state.slip_angle
	
	# Pacejka-style tire model approximation
	var normal_force = abs(wheel_state.contact_normal.y) * mass / 4.0
	var cornering_stiffness = 80000.0  # N/rad
	
	var lateral_force = -cornering_stiffness * sin(slip_angle)
	
	# Grip loss at high slip angles
	if abs(slip_angle) > grip_loss_threshold:
		lateral_force *= drift_friction_reduction if _drift_mode else 1.0
	
	return lateral_force

func _calculate_vertical_force(wheel_index: int, delta: float) -> float:
	var wheel_state = _wheel_states[wheel_index]
	var static_weight = mass * PhysicsSettings.gravity / 4.0
	
	# Weight transfer calculation
	var acceleration_x = velocity.x * delta
	var weight_transfer = acceleration_x * mass * wheel_base / 2.0 / track_width
	
	# Adjust for suspension compression
	var suspension_effect = wheel_state.suspension_compression * suspension_stiffness
	
	var vertical_force = static_weight + weight_transfer - suspension_effect
	vertical_force = maxf(vertical_force, 0.0)
	
	return vertical_force

func _apply_wheel_forces_to_rigidbody() -> void:
	if _rigid_body_3d == null:
		return
	
	var world_positions = _get_wheel_world_positions()
	
	for i in range(4):
		if _wheel_forces[i].length() > 0:
			_rigid_body_3d.apply_force_at_position(_wheel_forces[i], world_positions[i])

func _get_wheel_world_positions() -> Array[Vector3]:
	var positions = []
	for i in range(4):
		positions.append(_get_wheel_world_position(i))
	return positions

func _apply_aerodynamic_forces(delta: float) -> void:
	if _vehicle_speed > 5.0:
		var drag_coefficient = 0.3
		var air_density = 1.225
		var frontal_area = 2.0
		
		var drag_force = 0.5 * air_density * drag_coefficient * frontal_area * _vehicle_speed * _vehicle_speed
		var drag_direction = -velocity.normalized() if velocity.length() > 0 else Vector3.ZERO
		
		var aero_force = drag_direction * drag_force
		_rigid_body_3d.apply_central_force(aero_force)

func _apply_gravity() -> void:
	if _rigid_body_3d != null:
		var gravity_vector = Vector3.DOWN * mass * PhysicsSettings.gravity
		_rigid_body_3d.apply_central_force(gravity_vector)

# ============================================================================
# COLLISION DETECTION & HANDLING
# ============================================================================
func _check_collisions() -> void:
	if _rigid_body_3d == null:
		return
	
	var collision_count = _rigid_body_3d.get_collision_count()
	
	for i in range(collision_count):
		var collision_info = _rigid_body_3d.get_collision_info(i)
		var collision_data = {
			"collider": collision_info.collider,
			"normal": collision_info.normal,
			"position": collision_info.position,
			"shape_index": collision_info.shape_index
		}
		
		collision_detected.emit(collision_data)

func _on_collision_entered(body: Node) -> void:
	var collision_data = {
		"collider": body,
		"impact_velocity": velocity.length(),
		"time": Time.get_unix_time_from_system()
	}
	collision_detected.emit(collision_data)

# ============================================================================
# DRIFT MODE SYSTEM
# ============================================================================
func _enter_drift_mode() -> void:
	if _vehicle_speed > 15.0:  # Minimum speed to drift
		_drift_mode = true
		_drift_angle = 0.0
		drift_started.emit(0.0)

func _exit_drift_mode() -> void:
	_drift_mode = false
	drift_ended.emit()

func _update_drift_state(delta: float) -> void:
	if not _drift_mode:
		return
	
	var drift_threshold = grip_loss_threshold
	var current_slip = abs(_wheel_states[0].slip_angle) + abs(_wheel_states[1].slip_angle)
	
	if current_slip > drift_threshold:
		_drift_angle += delta * 2.0
		drift_started.emit(_drift_angle)
	else:
		_exit_drift_mode()

# ============================================================================
# LAP TIMING & CHECKPOINTS
# ============================================================================
func start_race_timer() -> void:
	_race_start_time = Time.get_ticks_msec() / 1000.0
	_current_lap_time = 0.0
	_lap_times.clear()

func stop_race_timer() -> void:
	var race_duration = Time.get_ticks_msec() / 1000.0 - _race_start_time
	race_ended.emit({"duration": race_duration, "final_position": 0})

func record_lap_time() -> void:
	_lap_times.append(_current_lap_time)
	lap_completed.emit(_current_lap_time)
	_current_lap_time = 0.0

func register_checkpoint(checkpoint_id: int) -> void:
	_checkpoint_ids.append(checkpoint_id)
	checkpoint_passed.emit(checkpoint_id)
	_last_checkpoint_time = Time.get_ticks_msec() / 1000.0

func update_lap_timer(delta: float) -> void:
	if _race_start_time > 0:
		_current_lap_time += delta

func get_lap_history() -> Array[float]:
	return _lap_times.duplicate()

func get_best_lap_time() -> float:
	if _lap_times.is_empty():
		return 0.0
	return _lap_times.min()

# ============================================================================
# POWERTRAIN INTEGRATION
# ============================================================================
func _init_powertrain_connection() -> void:
	var parent = get_parent()
	if parent != null and parent.has_method("get_powertrain"):
		_powertrain_node = parent.get_powertrain()
		if _powertrain_node != null:
			powertrain_connected.emit(_powertrain_node)

func connect_to_powertrain(powertrain: Node) -> void:
	_powertrain_node = powertrain
	powertrain_connected.emit(powertrain)

func get_powertrain() -> Node:
	return _powertrain_node

# ============================================================================
# DEBUG & VISUALIZATION
# ============================================================================
func _update_debug_overlay() -> void:
	if not _debug_overlay_visible:
		return
	
	# Draw debug information on screen
	var debug_text = "=== Vehicle Controller Debug ===\n"
	debug_text += "Gear: %d\n" % _current_gear
	debug_text += "RPM: %.0f\n" % _rpm
	debug_text += "Speed: %.1f km/h\n" % get_vehicle_speed()
	debug_text += "Throttle: %.2f\n" % _throttle_input
	debug_text += "Brake: %.2f\n" % _brake_input
	debug_text += "Steer: %.2f\n" % _steering_input
	debug_text += "Handbrake: %s\n" % str(_handbrake_active)
	debug_text += "Drift Mode: %s\n" % str(_drift_mode)
	
	# This would typically draw to a Label or CanvasLayer
	# For now, we'll log it
	print(debug_text)

func toggle_debug_overlay() -> void:
	_debug_overlay_visible = not _debug_overlay_visible

func set_debug_mode(enabled: bool) -> void:
	_debug_overlay_visible = enabled
	_physics_debug_mode = enabled

# ============================================================================
# SAVE & LOAD STATE
# ============================================================================
func save_state() -> Dictionary:
	var state = {
		"gear": _current_gear,
		"rpm": _rpm,
		"speed": _vehicle_speed,
		"position": global_position,
		"rotation": global_rotation,
		"velocity": velocity,
		"engine_on": _engine_on,
		"handbrake": _handbrake_active,
		"drift_mode": _drift_mode,
		"lap_time": _current_lap_time,
		"lap_history": _lap_times
	}
	
	return state

func load_state(state: Dictionary) -> void:
	_current_gear = state.get("gear", GearState.NEUTRAL)
	_rpm = state.get("rpm", IDLE_RPM)
	_vehicle_speed = state.get("speed", 0.0)
	set_position(state.get("position", global_position))
	set_rotation(state.get("rotation", global_rotation))
	velocity = state.get("velocity", Vector3.ZERO)
	_engine_on = state.get("engine_on", false)
	_handbrake_active = state.get("handbrake", false)
	_drift_mode = state.get("drift_mode", false)
	_current_lap_time = state.get("lap_time", 0.0)
	_lap_times = state.get("lap_history", [])

func _load_saved_state() -> void:
	# Load any previously saved state if available
	pass

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================
func reset_vehicle() -> void:
	reset_to_position(global_position, global_rotation)
	_engine_on = false
	_rpm = IDLE_RPM
	_current_gear = GearState.NEUTRAL
	_handbrake_active = false
	_drift_mode = false
	_lap_times.clear()
	_current_lap_time = 0.0

func accelerate(amount: float) -> void:
	_throttle_input = clampf(amount, 0.0, 1.0)

func brake(amount: float) -> void:
	_brake_input = clampf(amount, 0.0, 1.0)

func steer(amount: float) -> void:
	_steering_input = clampf(amount, -1.0, 1.0)

func set_all_inputs(throttle: float, brake: float, steer: float) -> void:
	_throttle_input = clampf(throttle, 0.0, 1.0)
	_brake_input = clampf(brake, 0.0, 1.0)
	_steering_input = clampf(steer, -1.0, 1.0)

func clear_inputs() -> void:
	_throttle_input = 0.0
	_brake_input = 0.0
	_steering_input = 0.0

func calculate_acceleration() -> float:
	if _rigid_body_3d == null:
		return 0.0
	
	var forward_force = 0.0
	for i in range(4):
		forward_force += _wheel_forces[i].x
	
	return forward_force / mass if mass > 0 else 0.0

func calculate_turning_radius() -> float:
	if _vehicle_speed <= 0.0:
		return 0.0
	
	var angular_velocity = abs(angular_velocity.y)
	if angular_velocity <= 0.0:
		return 0.0
	
	return _vehicle_speed / angular_velocity

func get_distance_traveled() -> float:
	return global_position.distance_to(get_initial_position())

func get_initial_position() -> Vector3:
	# Store initial position on first call
	if not has_meta("_initial_position"):
		set_meta("_initial_position", global_position)
	return get_meta("_initial_position")

# ============================================================================
# SIGNAL CONNECTIONS
# ============================================================================
func _connect_signals_to_game_manager() -> void:
	if GameManager:
		game_state_changed.connect(_on_game_state_changed)
		race_started.connect(_on_race_started)
		race_ended.connect(_on_race_ended)

func _on_game_state_changed(new_state: GameState) -> void:
	match new_state:
		GameState.RACE_ACTIVE:
			start_race_timer()
		GameState.RACE_PAUSED:
			pass
		GameState.RACE_FINISHED:
			stop_race_timer()

func _on_race_started(race_data: Dictionary) -> void:
	start_race_timer()

func _on_race_ended(results: Dictionary) -> void:
	stop_race_timer()

# ============================================================================
# PHYSICS SETTINGS INTEGRATION
# ============================================================================
func _init_rigid_body() -> void:
	_rigid_body_3d = get_parent()
	if _rigid_body_3d is RigidBody3D:
		_rigid_body_3d.mass = mass
		_rigid_body_3d.collision_layer = 1 << 2  # Vehicle layer
		_rigid_body_3d.collision_mask = 1 << 3 | 1 << 4  # Track and obstacles

func _init_suspension() -> void:
	_suspension_nodes.clear()
	# In a full implementation, these would be actual suspension nodes
	# For now, we use calculated values

func get_physics_settings() -> PhysicsSettings:
	return PhysicsSettings if PhysicsSettings else null

func update_physics_settings(settings: PhysicsSettings) -> void:
	if settings:
		mass = settings.default_vehicle_mass
		max_power = settings.max_power
		max_torque = settings.max_torque

func _set_final_drive_ratio(value: float) -> void:
	final_drive_ratio = value

func _set_wheel_radius(value: float) -> void:
	wheel_radius = value

func _set_gravity(value: float) -> void:
	PhysicsSettings.gravity = value

func _set_physics_tick_rate(value: int) -> void:
	PhysicsSettings.physics_tick_rate = value

func _set_max_substeps(value: int) -> void:
	PhysicsSettings.max_substeps = value

func _set_time_scale(value: float) -> void:
	PhysicsSettings.time_scale = value

func _set_default_vehicle_mass(value: float) -> void:
	mass = value

func _set_default_whe