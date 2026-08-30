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
const TURBO_CHARGE_TIME: float = 2.5
const SUSPENSION_COMPRESSION_LIMIT: float = 0.3

# ============================================================================
# EXPORTED CONFIGURATION - Vehicle Setup (Exposed in Inspector)
# ============================================================================
@export_group("Vehicle Configuration")
@export var vehicle_mass: float = 1500.0
@export var engine_torque: float = 450.0
@export var transmission_ratio: float = 3.5
@export var differential_ratio: float = 3.8
@export var wheel_radius: float = 0.35
@export var track_width: float = 1.6
@export var wheel_base: float = 2.7
@export var center_of_mass_y: float = 0.5
@export var drag_coefficient: float = 0.32
@export var air_density: float = 1.225
@export var max_steering_angle: float = 30.0 * PI / 180.0

@export_group("Suspension Settings")
@export var suspension_stiffness: float = 80000.0
@export var suspension_damping: float = 15000.0
@export var suspension_rest_length: float = 0.4
@export var suspension_travel: float = 0.25
@export var tire_friction: float = 1.1
@export var slip_angle_max: float = 15.0 * PI / 180.0

@export_group("Powertrain Settings")
@export var torque_curve_enabled: bool = true
@export var turbo_enabled: bool = false
@export var turbo_boost_pressure: float = 1.2
@export var launch_control_enabled: bool = false

# ============================================================================
# INTERNAL STATE - Vehicle Dynamics Variables
# ============================================================================
var _current_speed_kmh: float = 0.0
var _current_rpm: float = IDLE_RPM
var _current_gear: int = 0
var _throttle_input: float = 0.0
var _brake_input: float = 0.0
var _steering_input: float = 0.0
var _clutch_input: float = 1.0

var _target_gear: int = 0
var _gear_shift_progress: float = 0.0
var _clutch_release_timer: float = 0.0
var _turbo_timer: float = 0.0
var _drift_intensity: float = 0.0
var _is_drifting: bool = false
var _suspension_compression: Vector3 = Vector3.ZERO
var _wheel_rotation_angles: Vector4 = Vector4.ZERO

var _torque_output: float = 0.0
var _engine_braking_force: float = 0.0
var _aerodynamic_drag: float = 0.0
var _traction_force: float = 0.0
var _lateral_force: float = 0.0

var _last_collision_position: Vector3 = Vector3.ZERO
var _collision_impact_velocity: float = 0.0
var _collision_normal: Vector3 = Vector3.ZERO

# References to child nodes
var _powertrain_node: Node = null
var _camera_node: Node3D = null
var _wheel_nodes: Array[Node3D] = []
var _suspension_nodes: Array[Node3D] = []

# Physics references
var _physics_settings: PhysicsSettings = PhysicsSettings.new()

# Lap timing
var _lap_start_time: float = 0.0
var _lap_times: Array[float] = []
var _current_lap_distance: float = 0.0
var _total_race_distance: float = 0.0

# Cache for performance
var _velocity_cache: Vector3 = Vector3.ZERO
var _angular_velocity_cache: Vector3 = Vector3.ZERO

func _ready() -> void:
	_process_mode = ProcessModeEnum.ALWAYS
	_init_physics_settings()
	_connect_child_nodes()
	_calculate_suspension_parameters()
	_setup_initial_state()

func _init_physics_settings() -> void:
	if GameManager != null and GameManager.has_signal("game_state_changed"):
		GameManager.game_state_changed.connect(_on_game_state_changed)

# ============================================================================
# CHILD NODE CONNECTIONS
# ============================================================================
func _connect_child_nodes() -> void:
	var children = get_children()
	for child in children:
		if child.name == "Powertrain":
			_powertrain_node = child
		elif child.name == "Camera" or child.name.begins_with("Camera"):
			_camera_node = child
		elif child.name.begins_with("Wheel_") or child.name.begins_with("wheel_"):
			_wheel_nodes.append(child)
		elif child.name.begins_with("Suspension_") or child.name.begins_with("suspension_"):
			_suspension_nodes.append(child)

# ============================================================================
# PHYSICS CALCULATION METHODS
# ============================================================================
func _calculate_suspension_parameters() -> void:
	var spring_constant = suspension_stiffness
	var damper_constant = suspension_damping
	var static_deflection = (vehicle_mass * PhysicsSettings.gravity) / (4.0 * spring_constant)
	var damping_ratio = damper_constant / (2.0 * sqrt(spring_constant * vehicle_mass))

# ============================================================================
# INPUT HANDLING
# ============================================================================
func _process(delta: float) -> void:
	_handle_input(delta)
	_update_engine_and_transmission(delta)
	_apply_forces(delta)
	_update_suspension(delta)
	_update_visuals(delta)
	_check_drift_status(delta)
	_update_aerodynamics(delta)

func _handle_input(delta: float) -> void:
	_throttle_input = InputManager.get_axis("throttle_up", "throttle_down")
	_brake_input = InputManager.get_axis("brake", "reverse")
	_steering_input = InputManager.get_axis("steering_left", "steering_right")
	_clutch_input = InputManager.get_axis("clutch_up", "clutch_down")

	_limit_inputs()

func _limit_inputs() -> void:
	_throttle_input = clamp(_throttle_input, 0.0, 1.0)
	_brake_input = clamp(_brake_input, 0.0, 1.0)
	_steering_input = clamp(_steering_input, -1.0, 1.0)
	_clutch_input = clamp(_clutch_input, 0.0, 1.0)

# ============================================================================
# ENGINE AND TRANSMISSION LOGIC
# ============================================================================
func _update_engine_and_transmission(delta: float) -> void:
	_update_rpm(delta)
	_update_gearing(delta)
	_calcuate_torque_output()
	_handle_clutch_operation(delta)

func _update_rpm(delta: float) -> void:
	var target_rpm = _get_target_rpm_for_gear()
	
	if _clutch_input > 0.95:
		_current_rpm = lerp(_current_rpm, target_rpm, delta * 10.0)
	else:
		_current_rpm = lerp(_current_rpm, IDLE_RPM, delta * 5.0)
	
	_current_rpm = clamp(_current_rpm, IDLE_RPM, REDLINE_RPM)
	emit_signal("rpm_changed", _current_rpm)

func _get_target_rpm_for_gear() -> float:
	if _current_gear == 0:
		return IDLE_RPM
	
	var wheel_rpm = _current_speed_kmh / (2.0 * PI * wheel_radius) * 60.0
	var total_ratio = transmission_ratio * differential_ratio
	var gear_ratio = _get_gear_ratio(_current_gear)
	total_ratio *= gear_ratio
	
	return wheel_rpm * total_ratio

func _update_gearing(delta: float) -> void:
	if _clutch_input < 0.5:
		return
	
	_auto_shift_gears()
	_manual_shift_requests()
	_handle_gear_shift_progress(delta)

func _auto_shift_gears() -> void:
	if _current_gear == 0 and _throttle_input > 0.0:
		target_gear = 1
		return
	
	if _current_rpm >= SHIFT_RPM and _current_gear < MAX_GEAR:
		_target_gear = _current_gear + 1
	elif _current_rpm <= IDLE_RPM + 500 and _current_gear > 1:
		_target_gear = _current_gear - 1

func _manual_shift_requests() -> void:
	if Input.is_action_just_pressed("shift_up"):
		if _current_gear < MAX_GEAR:
			_target_gear = _current_gear + 1
	elif Input.is_action_just_pressed("shift_down"):
		if _current_gear > MIN_GEAR:
			_target_gear = _current_gear - 1

func _handle_gear_shift_progress(delta: float) -> void:
	if _gear_shift_progress < 1.0:
		_gear_shift_progress += delta / CLUTCH_RELEASE_TIME
		if _gear_shift_progress >= 1.0:
			_gear_shift_progress = 1.0
			_complete_gear_shift()
	else:
		_gear_shift_progress = 0.0

func _complete_gear_shift() -> void:
	_current_gear = _target_gear
	emit_signal("gear_changed", _current_gear)

func _calcuate_torque_output() -> void:
	var gear_ratio = 1.0 if _current_gear == 0 else _get_gear_ratio(_current_gear)
	var total_ratio = transmission_ratio * differential_ratio * gear_ratio
	
	var wheel_torque = engine_torque * total_ratio
	var drive_force = wheel_torque / wheel_radius
	
	if _current_gear == 0:
		drive_force *= 0.0
	else:
		drive_force *= _throttle_input * _clutch_input
	
	_torque_output = drive_force
	if _throttle_input < 0.1 and _current_gear > 0:
		_engine_braking_force = abs(_torque_output) * 0.3

func _get_gear_ratio(gear: int) -> float:
	match gear:
		1: return 3.8
		2: return 2.4
		3: return 1.7
		4: return 1.3
		5: return 1.0
		6: return 0.85
		_: return 1.0

func _handle_clutch_operation(delta: float) -> void:
	if _clutch_release_timer > 0.0:
		_clutch_release_timer -= delta
		if _clutch_release_timer <= 0.0:
			_clutch_input = 1.0

# ============================================================================
# FORCE APPLICATION
# ============================================================================
func _apply_forces(delta: float) -> void:
	_update_velocity(delta)
	_apply_drive_forces(delta)
	_apply_brake_forces(delta)
	_apply_steering_forces(delta)
	_apply_gravity(delta)

func _update_velocity(delta: float) -> void:
	var acceleration = _torque_output / vehicle_mass
	var deceleration = (_brake_input * BRAKING_FORCE + _engine_braking_force) / vehicle_mass
	
	var net_acceleration = acceleration - deceleration - _aerodynamic_drag
	
	_velocity_cache.x = velocity.x + net_acceleration * delta
	_velocity_cache.y = 0.0
	_velocity_cache.z = velocity.z + net_acceleration * delta
	
	var speed = _velocity_cache.length()
	_current_speed_kmh = speed * 3.6
	
	emit_signal("speed_changed", _current_speed_kmh)

func _apply_drive_forces(delta: float) -> void:
	if _current_gear == 0:
		return
	
	var forward_vector = -global_transform.basis.z
	var drive_force_vector = forward_vector * _torque_output
	
	add_force(drive_force_vector)

func _apply_brake_forces(delta: float) -> void:
	if _brake_input <= 0.0:
		return
	
	var braking_vector = -velocity.normalized() * _brake_input * BRAKING_FORCE
	add_force(braking_vector)

func _apply_steering_forces(delta: float) -> void:
	if _current_speed_kmh < 5.0:
		return
	
	var turn_angle = _steering_input * max_steering_angle
	var rotation_axis = Vector3.UP
	
	rotate_object_local(rotation_axis, turn_angle * delta * TURN_SPEED)

func _apply_gravity(delta: float) -> void:
	var gravity_vector = Vector3.DOWN * PhysicsSettings.gravity * vehicle_mass
	add_force(gravity_vector)

# ============================================================================
# AERODYNAMICS AND DRAG
# ============================================================================
func _update_aerodynamics(delta: float) -> void:
	var speed_ms = _current_speed_kmh / 3.6
	var dynamic_pressure = 0.5 * air_density * speed_ms * speed_ms
	
	var frontal_area = track_width * suspension_rest_length
	_aerodynamic_drag = 0.5 * drag_coefficient * air_density * speed_ms * speed_ms * frontal_area

# ============================================================================
# SUSPENSION SYSTEM
# ============================================================================
func _update_suspension(delta: float) -> void:
	for i in range(min(4, _suspension_nodes.size())):
		var suspension_node = _suspension_nodes[i]
		var compression = _calculate_suspension_compression(i)
		
		_suspension_compression[i] = compression
		suspension_node.transform.origin.y = suspension_rest_length - compression
		
		var force = -suspension_stiffness * compression - suspension_damping * compression_delta
		suspension_node.apply_central_force(force * Vector3.UP)

func _calculate_suspension_compression(wheel_index: int) -> float:
	var ground_height = _raycast_ground_height(wheel_index)
	var wheel_height = suspension_nodes[wheel_index].transform.origin.y
	var compression = suspension_rest_length - wheel_height + ground_height
	
	return clamp(compression, 0.0, suspension_travel)

func _raycast_ground_height(wheel_index: int) -> float:
	var ray_from = global_transform.origin
	var ray_to = ray_from + Vector3.DOWN * 10.0
	
	var space_state = get_world_3d().direct_space_state
	var query = PhysicsRayQueryParameters3D.create(ray_from, ray_to)
	var result = space_state.ray_query(query)
	
	if result.has_collided():
		return result.position.y - global_transform.origin.y
	
	return 0.0

# ============================================================================
# DRIFT MECHANICS
# ============================================================================
func _check_drift_status(delta: float) -> void:
	if _current_speed_kmh < 30.0:
		_is_drifting = false
		_drift_intensity = 0.0
		return
	
	var lateral_acceleration = _lateral_force / vehicle_mass
	var drift_threshold = DRIFT_THRESHOLD
	
	if lateral_acceleration > drift_threshold and _steering_input.abs() > 0.3:
		if not _is_drifting:
			_is_drifting = true
			_drift_intensity = min(abs(lateral_acceleration) / drift_threshold, DRIFT_INTENSITY_MAX)
			emit_signal("drift_started", _drift_intensity)
		else:
			_drift_intensity = min(abs(lateral_acceleration) / drift_threshold, DRIFT_INTENSITY_MAX)
	else:
		if _is_drifting:
			_is_drifting = false
			emit_signal("drift_ended")
		_drift_intensity = 0.0

# ============================================================================
# WHEEL ROTATION AND VISUAL UPDATES
# ============================================================================
func _update_visuals(delta: float) -> void:
	_update_wheel_rotations(delta)
	_update_camera_view(delta)
	_update_suspension_visuals()

func _update_wheel_rotations(delta: float) -> void:
	var wheel_rotation_increment = _current_speed_kmh * PI / 180.0 * delta / wheel_radius
	
	for i in range(min(4, _wheel_nodes.size())):
		var wheel_node = _wheel_nodes[i]
		var angle = _wheel_rotation_angles[i]
		
		angle += wheel_rotation_increment * _clutch_input
		_wheel_rotation_angles[i] = angle % (2.0 * PI)
		
		wheel_node.rotate_x(angle)

func _update_camera_view(delta: float) -> void:
	if _camera_node != null:
		var offset = Vector3(0.0, 2.0, -4.0)
		var target_position = global_position + global_transform.basis * offset
		
		_camera_node.global_position = lerp(_camera_node.global_position, target_position, delta * 5.0)
		_camera_node.look_at(global_position + Vector3(0.0, 1.0, 0.0))

func _update_suspension_visuals() -> void:
	for compression in _suspension_compression:
		if compression > SUSPENSION_COMPRESSION_LIMIT * 0.5:
			emit_signal("suspension_compressed", compression)

# ============================================================================
# COLLISION HANDLING
# ============================================================================
func _on_collision_body_entered(body: Node) -> void:
	if body is RigidBody3D:
		var impact_velocity = velocity.distance_to(Vector3.ZERO)
		var collision_info = {
			"body": body.get_path(),
			"impact_velocity": impact_velocity,
			"position": global_position,
			"normal": get_collision_normal()
		}
		
		_last_collision_position = global_position
		_collision_impact_velocity = impact_velocity
		_collision_normal = get_collision_normal()
		
		emit_signal("collision_detected", collision_info)

func _on_collision_body_exited(body: Node) -> void:
	pass

# ============================================================================
# LAP AND RACE TRACKING
# ============================================================================
func _record_lap_data() -> void:
	var lap_time = Time.get_unix_time_from_system() - _lap_start_time
	_lap_times.append(lap_time)
	
	var lap_data = {
		"lap_number": _lap_times.size(),
		"lap_time": lap_time,
		"average_speed": _total_race_distance / lap_time,
		"best_sector_time": 0.0
	}
	
	emit_signal("lap_completed", lap_data)
	_lap_start_time = Time.get_unix_time_from_system()

func update_race_distance(distance: float) -> void:
	_total_race_distance += distance
	_current_lap_distance += distance

# ============================================================================
# PUBLIC API - Control Methods
# ============================================================================
func set_throttle(amount: float) -> void:
	_throttle_input = clamp(amount, 0.0, 1.0)

func set_brake(amount: float) -> void:
	_brake_input = clamp(amount, 0.0, 1.0)

func set_steering(amount: float) -> void:
	_steering_input = clamp(amount, -1.0, 1.0)

func set_gear(gear: int) -> void:
	_target_gear = clamp(gear, MIN_GEAR, MAX_GEAR)

func get_current_speed() -> float:
	return _current_speed_kmh

func get_current_rpm() -> float:
	return _current_rpm

func get_current_gear() -> int:
	return _current_gear

func is_drifting() -> bool:
	return _is_drifting

func reset_vehicle() -> void:
	_current_speed_kmh = 0.0
	_current_rpm = IDLE_RPM
	_current_gear = 0
	_throttle_input = 0.0
	_brake_input = 0.0
	_steering_input = 0.0
	_clutch_input = 1.0
	_velocity_cache = Vector3.ZERO
	set_velocity(Vector3.ZERO)

# ============================================================================
# EVENT HANDLERS
# ============================================================================
func _on_game_state_changed(new_state: GameState.GameState) -> void:
	match new_state:
		GameManager.GameState.RACE_ACTIVE:
			_reset_for_race()
		GameManager.GameState.RACE_PAUSED:
			pause_processing()
		GameManager.GameState.MAIN_MENU:
			reset_vehicle()

func _reset_for_race() -> void:
	_lap_start_time = Time.get_unix_time_from_system()
	_lap_times.clear()
	_current_lap_distance = 0.0
	_total_race_distance = 0.0
	reset_vehicle()

# ============================================================================
# DEBUG TOOLS
# ============================================================================
func debug_print_stats() -> void:
	print("[VehicleStats]")
	print("Speed: %.1f km/h" % _current_speed_kmh)
	print("RPM: %.0f" % _current_rpm)
	print("Gear: %d" % _current_gear)
	print("Throttle: %.2f" % _throttle_input)
	print("Brake: %.2f" % _brake_input)
	print("Steering: %.2f" % _steering_input)
	print("Drifting: %s" % str(_is_drifting))
	print("Distance: %.2f m" % _total_race_distance)