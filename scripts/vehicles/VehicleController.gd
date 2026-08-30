extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulation
## Handles throttle, brake, steering, wheel forces, gear shifting, and vehicle dynamics
## Copyright 2026 Thalamus Racing Simulator Project

# ============================================================================
# SIGNALS
# ============================================================================
signal speed_changed(new_speed: float)
signal rpm_changed(rpm: int)
signal gear_changed(gear: int)
signal drift_started(drift_angle: float)
signal drift_ended()
signal collision_detected(impact_velocity: Vector3)
signal lap_completed(lap_data: Dictionary)
signal race_event(event_type: String)
signal engine_sound_changed(rpm_ratio: float)
signal suspension_compressed(suspension_amount: float)

# ============================================================================
# CONSTANTS & CONFIGURATION
# ============================================================================
const MAX_ENGINE_RPM: int = 8500
const IDLE_RPM: int = 800
const REDLINE_RPM: int = 7200
const RPM_PER_GEAR: Array[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
const GEAR_RATIOS: Array[float] = [0.0, 3.8f, 2.4f, 1.7f, 1.3f, 1.1f, 0.85f, 0.65f]
const FINAL_DRIVE_RATIO: float = 3.73f
const MAX_REVERSE_SPEED: float = -15.0
const MAX_FORWARD_SPEED: float = 120.0
const DRIFT_THRESHOLD: float = 0.5
const GRIP_LEVEL_NORMAL: float = 0.95
const GRIP_LEVEL_DRIFT: float = 0.35
const STEERING_SPEED: float = 15.0
const BRAKING_FORCE: float = 2.5
const ACCELERATION_FORCE: float = 1.8
const TURNING_RADIUS: float = 8.0

# ============================================================================
# EXPORTED PROPERTIES
# ============================================================================
@export_group("Vehicle Configuration")
@export var vehicle_mass: float = 1500.0
@export var center_of_mass_height: float = 0.5
@export var wheelbase: float = 2.5
@export var track_width: float = 1.6
@export var max_steering_angle: float = 30.0 * DEG_TO_RAD
@export var drivetrain_type: DrivetrainType = DrivetrainType.FWD

enum DrivetrainType {
	FWD,
	RWD,
	AWD
}

@export_group("Suspension Settings")
@export var suspension_stiffness: float = 150000.0
@export var suspension_damping: float = 15000.0
@export var suspension_travel: float = 0.2
@export var static_friction: float = 1.2
@export var rolling_friction: float = 0.02
@export var side_friction: float = 0.9

@export_group("Tire Properties")
@export var tire_radius: float = 0.32
@export var tire_width: float = 0.25
@export var grip_coefficient: float = 1.0

@export_group("Powertrain Integration")
@export var powertrain_node: NodePath = null
var _powertrain: Powertrain = null

@export_group("Debug")
@export var debug_enabled: bool = false
@export_category("Advanced Tuning")
@export var aerodynamic_drag: float = 0.35
@export var frontal_area: float = 2.2
@export var weight_distribution_front: float = 0.45
@export var weight_distribution_rear: float = 0.55

# ============================================================================
# INTERNAL STATE VARIABLES
# ============================================================================
# Engine state
var _engine_rpm: int = IDLE_RPM
var _current_gear: int = 0
var _target_gear: int = 0
var _torque_output: float = 0.0
var _clutch_engaged: bool = true
var _clutch_pedal_position: float = 1.0

# Vehicle dynamics
var _vehicle_speed: float = 0.0
var _forward_direction: Vector3 = Vector3.FORWARD
var _up_direction: Vector3 = Vector3.UP
var _drift_angle: float = 0.0
var _drifting: bool = false
var _brake_pressure: float = 0.0
var _handbrake_active: bool = false

# Input states (updated by InputManager)
var _throttle_input: float = 0.0
var _brake_input: float = 0.0
var _steering_input: float = 0.0
var _gear_input_up: bool = false
var _gear_input_down: bool = false
var _reverse_input: bool = false
var _handbrake_input: bool = false

# Wheel positions relative to body
var _front_left_wheel_pos: Vector3 = Vector3(-wheelbase / 2.0, -tire_radius, -track_width / 2.0)
var _front_right_wheel_pos: Vector3 = Vector3(-wheelbase / 2.0, -tire_radius, track_width / 2.0)
var _rear_left_wheel_pos: Vector3 = Vector3(wheelbase / 2.0, -tire_radius, -track_width / 2.0)
var _rear_right_wheel_pos: Vector3 = Vector3(wheelbase / 2.0, -tire_radius, track_width / 2.0)

# Suspension state per wheel
var _front_left_suspension: float = 0.0
var _front_right_suspension: float = 0.0
var _rear_left_suspension: float = 0.0
var _rear_right_suspension: float = 0.0

# Wheel rotational speed (RPM)
var _fl_wheel_rpm: int = 0
var _fr_wheel_rpm: int = 0
var _rl_wheel_rpm: int = 0
var _rr_wheel_rpm: int = 0

# Track/timing data
var _total_distance: float = 0.0
var _lap_start_time: float = 0.0
var _current_lap_time: float = 0.0
var _last_checkpoint_position: Vector3 = Vector3.ZERO
var _checkpoints_passed: Array[int] = []

# Cache references
var _rigid_body: RigidBody3D = null
var _collision_shape: CollisionShape3D = null

# ============================================================================
# ENUMS AND TYPES
# ============================================================================
enum WheelPosition {
	FRONT_LEFT,
	FRONT_RIGHT,
	REAR_LEFT,
	REAR_RIGHT
}

# ============================================================================
# INITIALIZATION
# ============================================================================
func _ready() -> void:
	_init_references()
	_connect_signals_to_powertrain()
	_initialize_physics()
	_log("VehicleController initialized successfully")

func _init_references() -> void:
	if powertrain_node != null:
		_powertrain = get_node(powertrain_node)
	
	_rigid_body = owner if owner is RigidBody3D else get_parent() as RigidBody3D
	
	if _rigid_body == null:
		_log_error("No RigidBody3D parent found!")
		return

# ============================================================================
# SIGNAL CONNECTIONS
# ============================================================================
func _connect_signals_to_powertrain() -> void:
	if _powertrain != null:
		_powertrain.rpm_changed.connect(_on_engine_rpm_changed)
		_powertrain.torque_available.connect(_update_torque_calculation)

# ============================================================================
# INPUT HANDLING
# ============================================================================
func handle_input(delta: float) -> void:
	_throttle_input = InputManager.get_axis(InputManager.THROTTLE)
	_brake_input = InputManager.get_axis(InputManager.BRAKE)
	_steering_input = InputManager.get_axis(InputManager.STEER)
	_handbrake_input = InputManager.is_action_pressed(InputManager.HANDBRAKE)
	_reverse_input = InputManager.is_action_pressed(InputManager.REVERSE)
	
	# Gear shift input detection (edge-triggered)
	if InputManager.is_action_just_pressed(InputManager.GEAR_UP):
		_shift_gear(true)
	if InputManager.is_action_just_pressed(InputManager.GEAR_DOWN):
		_shift_gear(false)
	
	# Clutch input (if manual transmission mode)
	if InputManager.is_action_pressed(InputManager.CLUTCH):
		_clutch_engaged = false
		_clutch_pedal_position = 0.0
	else:
		_clutch_engaged = true
		_clutch_pedal_position = 1.0

# ============================================================================
# PHYSICS UPDATE
# ============================================================================
func _physics_process(delta: float) -> void:
	if not is_inside_tree():
		return
	
	# Validate rigid body reference
	if _rigid_body == null:
		return
	
	_update_vehicle_state(delta)
	_apply_forces(delta)
	_handle_suspension(delta)
	_check_drift_state()
	_update_wheel_rotations(delta)
	emit_signals()

# ============================================================================
# VEHICLE STATE UPDATES
# ============================================================================
func _update_vehicle_state(delta: float) -> void:
	# Calculate current speed magnitude
	var velocity_magnitude := _rigid_body.linear_velocity.length()
	
	# Determine forward/reverse direction
	var is_forward := _rigid_body.linear_velocity.z > 0
	vehicle_speed = _rigid_body.linear_velocity.z if is_forward else -_rigid_body.linear_velocity.z
	
	# Update forward direction based on vehicle rotation
	forward_direction = transform.basis.z.normalized()
	up_direction = transform.basis.y.normalized()
	
	# Calculate drift angle (difference between velocity direction and facing direction)
	var velocity_dir := _rigid_body.linear_velocity.project(Vector3.FORWARD, up_direction).normalized()
	if velocity_dir.dot(forward_direction) < DRIFT_THRESHOLD:
		_drifting = true
		_drift_angle = acos(velocity_dir.dot(forward_direction))
	else:
		_drifting = false
		_drift_angle = 0.0
	
	# Update total distance traveled
	_total_distance += velocity_magnitude * delta
	
	# Update lap timing
	if GameManager.current_state == GameManager.RaceActive:
		_current_lap_time += delta
		
		# Check checkpoint collisions
		_check_checkpoint_progress()

# ============================================================================
# FORCE APPLICATION
# ============================================================================
func _apply_forces(delta: float) -> void:
	if not _clutch_engaged:
		return
	
	var force_multiplier := _calculate_force_multiplier()
	var torque := _calculate_torque_output()
	
	# Apply traction forces based on drivetrain type
	match drivetrain_type:
		DrivetrainType.FWD:
			_apply_drive_force(WheelPosition.FRONT_LEFT, torque * force_multiplier)
			_apply_drive_force(WheelPosition.FRONT_RIGHT, torque * force_multiplier)
		DrivetrainType.RWD:
			_apply_drive_force(WheelPosition.REAR_LEFT, torque * force_multiplier)
			_apply_drive_force(WheelPosition.REAR_RIGHT, torque * force_multiplier)
		DrivetrainType.AWD:
			var front_weight := weight_distribution_front
			var rear_weight := weight_distribution_rear
			_apply_drive_force(WheelPosition.FRONT_LEFT, torque * force_multiplier * front_weight)
			_apply_drive_force(WheelPosition.FRONT_RIGHT, torque * force_multiplier * front_weight)
			_apply_drive_force(WheelPosition.REAR_LEFT, torque * force_multiplier * rear_weight)
			_apply_drive_force(WheelPosition.REAR_RIGHT, torque * force_multiplier * rear_weight)
	
	# Apply braking forces
	_apply_braking_forces(delta)
	
	# Apply aerodynamic drag
	_apply_aerodynamic_drag(delta)
	
	# Apply steering forces
	_apply_steering_forces(delta)

# ============================================================================
# TORQUE CALCULATION
# ============================================================================
func _calculate_torque_output() -> float:
	if _powertrain != null:
		return _powertrain.get_available_torque() * _clutch_pedal_position
	return 0.0

func _calculate_force_multiplier() -> float:
	# Get throttle input influence
	var throttle_factor := _throttle_input if _throttle_input >= 0 else 0.0
	
	# Apply gear ratio effect
	var gear_ratio := GEAR_RATIOS[_current_gear] if _current_gear > 0 else 0.0
	
	# Combine factors with smooth transition
	var multiplier := throttle_factor * gear_ratio * 10.0
	
	# Clamp to reasonable range
	multiplier = clamp(multiplier, 0.0, 1.0)
	
	return multiplier

# ============================================================================
# DRIVE FORCE APPLICATION
# ============================================================================
func _apply_drive_force(wheel_pos: WheelPosition, force: float) -> void:
	if abs(force) < 0.1:
		return
	
	# Get wheel position in world space
	var wheel_local_pos := _get_wheel_local_position(wheel_pos)
	var wheel_world_pos := transform * wheel_local_pos
	
	# Calculate drive vector (forward direction)
	var drive_vector := forward_direction * sign(_vehicle_speed)
	
	# Apply force at wheel contact point
	_rigid_body.apply_central_impulse(drive_vector * force * 100.0)

# ============================================================================
# BRAKING SYSTEM
# ============================================================================
func _apply_braking_forces(delta: float) -> void:
	var brake_force := _calculate_brake_force()
	
	if brake_force > 0.1:
		# Apply brake force opposite to velocity
		var brake_vector := -_rigid_body.linear_velocity.normalized() * brake_force
		
		# Distribute brake force across wheels
		_rigid_body.apply_central_impulse(brake_vector * brake_force * delta * 50.0)
		
		# Handbrake adds lateral force for drifting
		if _handbrake_active:
			var handbrake_vector := Vector3.RIGHT.rotated(Vector3.UP, PI / 2)
			_rigid_body.apply_central_impulse(handbrake_vector * brake_force * 0.5)

func _calculate_brake_force() -> float:
	var target_brake := _brake_input
	if _handbrake_active:
		target_brake = max(target_brake, 1.0)
	
	# Smooth brake pressure buildup
	_brake_pressure = lerp(_brake_pressure, target_brake, 10.0 * get_process_delta_time())
	
	return _brake_pressure * BRAKING_FORCE

# ============================================================================
# AERODYNAMIC DRAG
# ============================================================================
func _apply_aerodynamic_drag(delta: float) -> void:
	var velocity := _rigid_body.linear_velocity
	var speed := velocity.length()
	
	if speed < 0.1:
		return
	
	# Drag equation: F = 0.5 * rho * v^2 * Cd * A
	var air_density := 1.225 # kg/m^3 at sea level
	var drag_force := 0.5 * air_density * speed * speed * aerodynamic_drag * frontal_area
	
	# Apply drag opposite to velocity direction
	var drag_vector := -velocity.normalized() * drag_force
	_rigid_body.apply_central_force(drag_vector)

# ============================================================================
# STEERING SYSTEM
# ============================================================================
func _apply_steering_forces(delta: float) -> void:
	# Only apply steering when moving
	if _vehicle_speed < 1.0:
		return
	
	# Smooth steering interpolation
	var target_steering := _steering_input * max_steering_angle
	var current_steering := _steering_input * max_steering_angle
	
	# Apply steering to front wheels only
	var steer_vector := Vector3.RIGHT.rotated(Vector3.UP, target_steering)
	
	# Modify angular velocity for turning
	var turn_force := steer_vector * _vehicle_speed * STEERING_SPEED * delta
	_rigid_body.angular_velocity += turn_force * 0.1

# ============================================================================
# SUSPENSION SYSTEM
# ============================================================================
func _handle_suspension(delta: float) -> void:
	var suspension_positions := [_front_left_suspension, _front_right_suspension, 
	                             _rear_left_suspension, _rear_right_suspension]
	
	for i in range(4):
		var old_suspension := suspension_positions[i]
		
		# Calculate compression based on ground contact
		var ray_cast := _create_ray_cast_at_wheel(i)
		if ray_cast and ray_cast.is_colliding():
			var distance := ray_cast.get_collision_point().y - transform.origin.y
			suspension_positions[i] = clamp(distance, 0.0, suspension_travel)
		
		# Spring-damper equation
		var spring_force := -(suspension_stiffness * suspension_positions[i])
		var damping_force := -suspension_damping * (suspension_positions[i] - old_suspension)
		
		# Apply suspension force to body
		var wheel_local_pos := _get_wheel_local_position(WheelPosition(i % 4))
		var wheel_world_pos := transform * wheel_local_pos
		var force_vector := Vector3.UP * (spring_force + damping_force)
		
		_rigid_body.apply_force_at_position(force_vector, wheel_world_pos)
	
	# Emit suspension signal if significantly compressed
	if suspension_positions[0] > suspension_travel * 0.5:
		suspension_compressed.emit(suspension_positions[0])

func _create_ray_cast_at_wheel(wheel_index: int) -> RayCast3D:
	var ray := RayCast3D.new()
	ray.target_position = Vector3.DOWN * 1.0
	ray.collision_mask = 1 << 0 # Layer 0 (terrain)
	add_child(ray)
	
	var wheel_local_pos := _get_wheel_local_position(WheelPosition(wheel_index))
	var wheel_global_pos := transform * wheel_local_pos
	ray.position = wheel_global_pos
	
	ray.force_raycast_update()
	return ray

# ============================================================================
# DRIFT MECHANICS
# ============================================================================
func _check_drift_state() -> void:
	if _drifting and not _handbrake_active:
		# Exit drift naturally
		_drifting = false
		drift_ended.emit()
	elif _drifting and _handbrake_active:
		# Maintain drift
		pass

# ============================================================================
# WHEEL ROTATION UPDATE
# ============================================================================
func _update_wheel_rotations(delta: float) -> void:
	var wheel_circumference := 2.0 * PI * tire_radius
	var linear_speed := abs(_vehicle_speed)
	var wheel_rpm := int((linear_speed / wheel_circumference) * 60.0 * 100.0)
	
	_fl_wheel_rpm = wheel_rpm
	_fr_wheel_rpm = wheel_rpm
	_rl_wheel_rpm = wheel_rpm
	_rr_wheel_rpm = wheel_rpm

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================
func _shift_gear(up: bool) -> void:
	if not _clutch_engaged:
		return
	
	var new_gear := _current_gear
	if up:
		new_gear = min(new_gear + 1, GEAR_RATIOS.size() - 1)
	else:
		new_gear = max(new_gear - 1, 0)
	
	if new_gear != _current_gear:
		_target_gear = new_gear
		_current_gear = new_gear
		gear_changed.emit(_current_gear)
		_log("Gear shifted to %d" % _current_gear)

func _update_gear_from_rpm() -> void:
	if _engine_rpm > REDLINE_RPM and _current_gear < GEAR_RATIOS.size() - 1:
		_shift_gear(true)
	elif _engine_rpm < IDLE_RPM and _current_gear > 0:
		_shift_gear(false)

# ============================================================================
# CHECKPOINT/TIMING SYSTEM
# ============================================================================
func _check_checkpoint_progress() -> void:
	# Simple checkpoint system - can be expanded with actual checkpoint nodes
	var current_pos := global_position
	var checkpoint_distance := current_pos.distance_to(_last_checkpoint_position)
	
	if checkpoint_distance > 10.0:
		_last_checkpoint_position = current_pos
		_checkpoints_passed.append(Time.get_ticks_msec())

func get_lap_time() -> float:
	return _current_lap_time

func get_total_distance() -> float:
	return _total_distance

func get_checkpoint_count() -> int:
	return _checkpoints_passed.size()

# ============================================================================
# SIGNAL EMISSION
# ============================================================================
func emit_signals() -> void:
	# Speed signal
	if abs(_vehicle_speed - speed_changed.get_value()) > 1.0:
		speed_changed.emit(_vehicle_speed)
	
	# RPM signal
	if _engine_rpm != rpm_changed.get_value():
		rpm_changed.emit(_engine_rpm)
	
	# Engine sound signal
	var rpm_ratio := float(_engine_rpm) / float(MAX_ENGINE_RPM)
	engine_sound_changed.emit(rpm_ratio)

# ============================================================================
# POWERTRAIN EVENT HANDLERS
# ============================================================================
func _on_engine_rpm_changed(rpm: int) -> void:
	_engine_rpm = rpm
	_update_gear_from_rpm()

func _update_torque_calculation() -> void:
	# Called by powertrain when torque calculation is needed
	# This method allows the controller to request updated torque values
	pass

# ============================================================================
# UTILITY METHODS
# ============================================================================
func _get_wheel_local_position(wheel_pos: WheelPosition) -> Vector3:
	match wheel_pos:
		WheelPosition.FRONT_LEFT:
			return _front_left_wheel_pos
		WheelPosition.FRONT_RIGHT:
			return _front_right_wheel_pos
		WheelPosition.REAR_LEFT:
			return _rear_left_wheel_pos
		WheelPosition.REAR_RIGHT:
			return _rear_right_wheel_pos
		_:
			return Vector3.ZERO

func get_vehicle_speed() -> float:
	return _vehicle_speed

func get_engine_rpm() -> int:
	return _engine_rpm

func get_current_gear() -> int:
	return _current_gear

func is_drifting() -> bool:
	return _drifting

func get_drift_angle() -> float:
	return _drift_angle

func reset_vehicle() -> void:
	_engine_rpm = IDLE_RPM
	_current_gear = 0
	_vehicle_speed = 0.0
	_total_distance = 0.0
	_current_lap_time = 0.0
	_checkpoints_passed.clear()
	_last_checkpoint_position = Vector3.ZERO
	_rigid_body.linear_velocity = Vector3.ZERO
	_rigid_body.angular_velocity = Vector3.ZERO

func _log(message: String) -> void:
	if debug_enabled:
		print("[VehicleController] %s" % message)

func _log_error(message: String) -> void:
	push_error("[VehicleController ERROR] %s" % message)

# ============================================================================
# END OF FILE
# ============================================================================
</file>