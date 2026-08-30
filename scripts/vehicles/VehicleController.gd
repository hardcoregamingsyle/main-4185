extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulation
## Handles throttle, brake, steering, wheel forces, gear shifting, and vehicle dynamics
## Copyright 2026 Thalamus Racing Simulator Project

# ============================================================================
# SIGNALS
# ============================================================================
signal speed_changed(new_speed: float)
signal rpm_changed(new_rpm: float)
signal gear_changed(old_gear: int, new_gear: int)
signal drift_started(drift_factor: float)
signal drift_ended()
signal collision_detected(collision_data: Dictionary)
signal lap_completed(lap_number: int, time_elapsed: float)
signal race_event(event_type: String, data: Dictionary)
signal engine_sound_changed(rpm_ratio: float)
signal suspension_compressed(compression_amount: float)

# ============================================================================
# CONSTANTS
# ============================================================================
const GEAR_RATIOS: Array[float] = [3.8, 2.5, 1.7, 1.2, 0.9, 0.7, 0.5]
const FINAL_DRIVE_RATIO: float = 3.5
const MAX_GEAR_COUNT: int = 7
const MIN_GEAR: int = 0
const NEUTRAL_GEAR: int = 0
const FIRST_GEAR: int = 1
const REVERSE_GEAR: int = -1
const WHEEL_RADIUS: float = 0.35
const MAX_FRICTION: float = 1.2
const MIN_FRICTION: float = 0.3
const DRIFT_THRESHOLD: float = 0.65
const DRIFT_RETENTION: float = 0.92
const DRIFT_DECAY: float = 0.98

# ============================================================================
# EXPORTED PROPERTIES
# ============================================================================
@export_group("Vehicle Configuration")
@export var vehicle_mass: float = 1500.0: set = _set_vehicle_mass
@export var center_of_mass_offset: Vector3 = Vector3(0.0, 0.5, 0.0)
@export var wheelbase: float = 2.8
@export var track_width: float = 1.6
@export var ground_clearance: float = 0.15

@export_group("Powertrain Settings")
@export var engine_max_rpm: float = 8500.0
@export var engine_idle_rpm: float = 800.0
@export var max_engine_torque: float = 450.0
@export var torque_curve: Curve = null
@export var power_curve: Curve = null
@export var clutch_disengage_rpm: float = 200.0

@export_group("Transmission")
@export var transmission_type: String = "manual"
@export var shift_delay_ms: int = 150
@export var auto_shift_enabled: bool = false
@export var upshift_rpm_threshold: float = 7500.0
@export var downshift_rpm_threshold: float = 3000.0
@export var neutral_on_stop: bool = true

@export_group("Wheel Configuration")
@export var drive_wheels: String = "rear"
@export var front_wheel_angle_max: float = 45.0 * TAU / 360.0
@export var rear_wheel_angle_max: float = 0.0
@export var anti_roll_bar_stiffness_front: float = 0.8
@export var anti_roll_bar_stiffness_rear: float = 0.6

@export_group("Tire Grip & Suspension")
@export var tire_friction_coefficient: float = 1.1
@export var side_wall_compliance: float = 0.005
@export var spring_damping_rate: float = 0.35
@export var spring_stiffness: float = 120000.0
@export var suspension_travel_max: float = 0.12
@export var suspension_travel_min: float = 0.02

@export_group("Aerodynamics")
@export var drag_coefficient: float = 0.32
@export var frontal_area: float = 2.3
@export var downforce_at_0kmh: float = 0.0
@export var lift_coefficient: float = 0.05
@export var wing_angle_degrees: float = 10.0

@export_group("Braking System")
@export var brake_force_max: float = 12000.0
@export var brake_distribution_front: float = 0.6
@export var abs_enabled: bool = true
@export var brake_bias_adjustment: float = 0.0

@export_group("Drift Settings")
@export var drift_mode_enabled: bool = true
@export var drift_friction_reduction: float = 0.4
@export var drift_throttle_penalty: float = 0.3
@export var drift_recovery_factor: float = 0.85
@export var drift_input_threshold: float = 0.8

@export_group("Physics Tuning")
@export var acceleration_multiplier: float = 1.0
@export var braking_multiplier: float = 1.0
@export var steering_sensitivity: float = 1.0
@export var traction_control_level: float = 1.0

# ============================================================================
# PRIVATE MEMBER VARIABLES
# ============================================================================
var _current_gear: int = 1
var _target_gear: int = 1
var _rpm: float = engine_idle_rpm
var _speed_kmh: float = 0.0
var _throttle_input: float = 0.0
var _brake_input: float = 0.0
var _steering_input: float = 0.0
var _clutch_input: float = 1.0
var _drift_factor: float = 0.0
var _is_drifting: bool = false
var _wheel_angles: Array[float] = [0.0, 0.0, 0.0, 0.0]
var _suspension_compression: Array[float] = [0.0, 0.0, 0.0, 0.0]
var _last_collision_time: float = 0.0
var _collision_count: int = 0
var _lap_times: Array[float] = []
var _current_lap: int = 0
var _race_active: bool = false
var _engine_sound_intensity: float = 0.0
var _auto_shift_timer: float = 0.0
var _gear_change_pending: bool = false
var _previous_velocity: Vector3 = Vector3.ZERO

# Wheel positions (relative to vehicle center)
var _front_left_wheel_pos: Vector3
var _front_right_wheel_pos: Vector3
var _rear_left_wheel_pos: Vector3
var _rear_right_wheel_pos: Vector3

# ============================================================================
# GETTERS AND SETTERS
# ============================================================================
func get_current_gear() -> int:
	return _current_gear

func get_target_gear() -> int:
	return _target_gear

func get_rpm() -> float:
	return _rpm

func get_speed_kmh() -> float:
	return _speed_kmh

func get_speed_mps() -> float:
	return _speed_kmh / 3.6

func get_throttle() -> float:
	return _throttle_input

func get_brake() -> float:
	return _brake_input

func get_steering() -> float:
	return _steering_input

func get_clutch() -> float:
	return _clutch_input

func is_drifting() -> bool:
	return _is_drifting

func get_drift_factor() -> float:
	return _drift_factor

func get_current_lap() -> int:
	return _current_lap

func get_wheel_angles() -> Array[float]:
	return _wheel_angles.copy()

func get_suspension_compression() -> Array[float]:
	return _suspension_compression.copy()

func _set_vehicle_mass(new_mass: float) -> void:
	vehicle_mass = clampf(new_mass, 500.0, 5000.0)

# ============================================================================
# PUBLIC METHODS
# ============================================================================
func _ready() -> void:
	_setup_wheel_positions()
	_reset_vehicle_state()
	process_mode = ProcessModeEnum.PROCESS_ALWAYS
	
	# Initialize wheel angles
	_wheel_angles.fill(0.0)
	_suspension_compression.fill(0.0)

func _setup_wheel_positions() -> void:
	var half_track = track_width / 2.0
	var half_wheelbase = wheelbase / 2.0
	_front_left_wheel_pos = Vector3(-half_track, 0.0, -half_wheelbase + ground_clearance)
	_front_right_wheel_pos = Vector3(half_track, 0.0, -half_wheelbase + ground_clearance)
	_rear_left_wheel_pos = Vector3(-half_track, 0.0, half_wheelbase + ground_clearance)
	_rear_right_wheel_pos = Vector3(half_track, 0.0, half_wheelbase + ground_clearance)

func _reset_vehicle_state() -> void:
	_current_gear = FIRST_GEAR
	_target_gear = FIRST_GEAR
	_rpm = engine_idle_rpm
	_speed_kmh = 0.0
	_throttle_input = 0.0
	_brake_input = 0.0
	_steering_input = 0.0
	_clutch_input = 1.0
	_drift_factor = 0.0
	_is_drifting = false
	_collision_count = 0
	_current_lap = 0
	_race_active = false
	_previous_velocity = Vector3.ZERO

func start_race() -> void:
	_reset_vehicle_state()
	_current_lap = 1
	_race_active = true
	lap_times.clear()
	race_event.emit("race_started", {"vehicle_id": get_instance_id(), "lap_count": 0})

func end_race(results: Dictionary) -> void:
	_race_active = false
	race_event.emit("race_ended", results)

func reset_race() -> void:
	start_race()

func process_physics(delta: float) -> void:
	_update_inputs()
	_process_auto_shifting(delta)
	_apply_gear_changes()
	_calculate_acceleration(delta)
	_calculate_braking(delta)
	_calculate_steering(delta)
	_calculate_aerodynamics(delta)
	_calculate_drift(delta)
	_update_suspension(delta)
	_apply_velocity_to_body(delta)
	_handle_collisions(delta)

func manual_upshift() -> void:
	if _current_gear < MAX_GEAR_COUNT and _current_gear > 0:
		_target_gear = _clamp_gear(_current_gear + 1)
		_clutch_input = 0.0
		await get_tree().create_timer(float(shift_delay_ms) / 1000.0).timeout
		_apply_gear_changes()
		_clutch_input = 1.0

func manual_downshift() -> void:
	if _current_gear > FIRST_GEAR:
		_target_gear = _clamp_gear(_current_gear - 1)
		_clutch_input = 0.0
		await get_tree().create_timer(float(shift_delay_ms) / 1000.0).timeout
		_apply_gear_changes()
		_clutch_input = 1.0

func shift_to_gear(gear: int) -> void:
	_target_gear = _clamp_gear(gear)
	if _current_gear != _target_gear:
		_clutch_input = 0.0
		await get_tree().create_timer(float(shift_delay_ms) / 1000.0).timeout
		_apply_gear_changes()
		_clutch_input = 1.0

func engage_drift() -> void:
	if not drift_mode_enabled:
		return
	_drift_factor = clampf(_drift_factor + 0.1, 0.0, 1.0)
	_is_drifting = _drift_factor >= DRIFT_THRESHOLD

func disengage_drift() -> void:
	_drift_factor = clampf(_drift_factor - 0.05, 0.0, 1.0)
	if _drift_factor < DRIFT_THRESHOLD:
		_is_dripping = false
		emit_signal("drift_ended")

func get_engine_torque(rpm: float) -> float:
	if torque_curve == null:
		return max_engine_torque * 0.8
	var t = (rpm - engine_idle_rpm) / (engine_max_rpm - engine_idle_rpm)
	t = clampf(t, 0.0, 1.0)
	return max_engine_torque * torque_curve.sample(t)

func get_engine_power(rpm: float) -> float:
	if power_curve == null:
		return max_engine_torque * rpm * 0.01
	var t = (rpm - engine_idle_rpm) / (engine_max_rpm - engine_idle_rpm)
	t = clampf(t, 0.0, 1.0)
	return power_curve.sample(t) * max_engine_torque * engine_max_rpm * 0.01

func calculate_lap_time() -> float:
	if _lap_times.is_empty():
		return 0.0
	var total_time: float = 0.0
	for t in _lap_times:
		total_time += t
	return total_time

func get_best_lap_time() -> float:
	if _lap_times.is_empty():
		return 0.0
	return _lap_times.min()

func add_lap_time(time: float) -> void:
	_lap_times.append(time)
	emit_signal("lap_completed", _current_lap + 1, time)

func reset_lap_times() -> void:
	_lap_times.clear()
	_current_lap = 0

# ============================================================================
# PRIVATE METHODS
# ============================================================================
func _update_inputs() -> void:
	# Inputs are updated by InputManager through GameManager
	pass

func _process_auto_shifting(delta: float) -> void:
	if not auto_shift_enabled or _gear_change_pending:
		return
	
	_auto_shift_timer += delta
	
	if _auto_shift_timer >= 0.2: # Auto-shift check every 200ms
		if _rpm >= upshift_rpm_threshold and _current_gear < MAX_GEAR_COUNT:
			_target_gear = _clamp_gear(_current_gear + 1)
			_clutch_input = 0.0
			_gear_change_pending = true
			_auto_shift_timer = 0.0
		elif _rpm <= downshift_rpm_threshold and _current_gear > FIRST_GEAR:
			_target_gear = _clamp_gear(_current_gear - 1)
			_clutch_input = 0.0
			_gear_change_pending = true
			_auto_shift_timer = 0.0

func _apply_gear_changes() -> void:
	if _current_gear != _target_gear:
		var old_gear = _current_gear
		_current_gear = _target_gear
		emit_signal("gear_changed", old_gear, _current_gear)
		_gear_change_pending = false

func _clamp_gear(gear: int) -> int:
	return clampi(gear, MIN_GEAR, MAX_GEAR_COUNT)

func _calculate_acceleration(delta: float) -> void:
	if _brake_input > 0.1:
		return
	
	var effective_torque = get_engine_torque(_rpm)
	
	if _clutch_input < 0.5:
		effective_torque *= _clutch_input
	
	var current_gear = _current_gear if _current_gear > 0 else 1
	var gear_ratio = GEAR_RATIOS[current_gear - 1] if current_gear <= MAX_GEAR_COUNT else GEAR_RATIOS[MAX_GEAR_COUNT - 1]
	var final_drive = gear_ratio * FINAL_DRIVE_RATIO
	
	var wheel_torque = effective_torque * final_drive * 0.95
	var wheel_force = wheel_torque / WHEEL_RADIUS
	
	var drive_factor = 1.0
	match drive_wheels:
		"front":
			drive_factor = 0.4
		"rear":
			drive_factor = 1.0
		"all":
			drive_factor = 1.0
	
	wheel_force *= drive_factor * acceleration_multiplier
	
	var velocity_mps = _speed_kmh / 3.6
	var air_resistance = 0.5 * drag_coefficient * frontal_area * velocity_mps * velocity_mps * 1.225
	
	var net_force = wheel_force - air_resistance
	net_force *= _throttle_input
	
	var acceleration = net_force / vehicle_mass
	acceleration *= 0.98 # Drivetrain efficiency
	
	_speed_kmh += acceleration * delta * 3.6
	
	_speed_kmh = maxf(_speed_kmh, 0.0)
	
	var target_rpm = (_speed_kmh / 3.6) * final_drive / WHEEL_RADIUS * 60.0
	_rpm = lerp(_rpm, target_rpm, 0.1)
	
	_rpm = clampf(_rpm, engine_idle_rpm, engine_max_rpm)
	
	emit_signal("speed_changed", _speed_kmh)
	emit_signal("rpm_changed", _rpm)
	
	if _rpm > engine_max_rpm * 0.95:
		engine_sound_intensity = 1.0
	elif _rpm > engine_max_rpm * 0.7:
		engine_sound_intensity = 0.7
	else:
		engine_sound_intensity = 0.3
	
	emit_signal("engine_sound_changed", _rpm / engine_max_rpm)

func _calculate_braking(delta: float) -> void:
	if _brake_input <= 0.0:
		return
	
	var braking_force = brake_force_max * braking_multiplier
	braking_force *= _brake_input
	
	var velocity_mps = _speed_kmh / 3.6
	
	if velocity_mps < 0.5 and _brake_input > 0.5:
		_speed_kmh = 0.0
		_rpm = engine_idle_rpm * _clutch_input
		return
	
	var deceleration = braking_force / vehicle_mass
	
	_speed_kmh -= deceleration * delta * 3.6
	
	if _speed_kmh < 0:
		_speed_kmh = 0.0
	
	_rpm = lerp(_rpm, engine_idle_rpm, 0.2)
	
	emit_signal("speed_changed", _speed_kmh)
	emit_signal("rpm_changed", _rpm)

func _calculate_steering(delta: float) -> void:
	var max_steering = front_wheel_angle_max * steering_sensitivity
	_steering_input = clampf(_steering_input, -max_steering, max_steering)
	
	var steering_ratio = _steering_input / max_steering
	
	_wheel_angles[0] = steering_ratio * front_wheel_angle_max
	_wheel_angles[1] = steering_ratio * front_wheel_angle_max
	_wheel_angles[2] = 0.0
	_wheel_angles[3] = 0.0

func _calculate_aerodynamics(delta: float) -> void:
	var velocity = global_transform.basis.xform(Vector3.FORWARD.normalized())
	var speed = velocity.length()
	
	var drag_force = 0.5 * drag_coefficient * frontal_area * speed * speed * 1.225
	
	var downforce = downforce_at_0kmh + (speed * speed / 100.0) * 0.01
	var lift = lift_coefficient * 0.5 * frontal_area * speed * speed * 1.225
	
	velocity.y -= (downforce - lift) / vehicle_mass * delta

func _calculate_drift(delta: float) -> void:
	if not drift_mode_enabled:
		_drift_factor = 0.0
		_is_drifting = false
		return
	
	var lateral_velocity = abs(_velocity.dot(global_transform.basis.y))
	var forward_velocity = _velocity.length()
	
	if forward_velocity > 1.0 and lateral_velocity / forward_velocity > DRIFT_THRESHOLD:
		_drift_factor = minf(_drift_factor + 0.02, 1.0)
		_is_drifting = true
		if not _is_drifting:
			emit_signal("drift_started", _drift_factor)
	elif lateral_velocity / forward_velocity < DRIFT_THRESHOLD * 0.5:
		_drift_factor *= DRIFT_DECAY
		if _drift_factor < DRIFT_THRESHOLD * 0.5:
			_is_drifting = false
			_drift_factor = 0.0
			emit_signal("drift_ended")

func _update_suspension(delta: float) -> void:
	var gravity_vector = Vector3.DOWN * PhysicsSettings.gravity
	
	for i in range(4):
		var wheel_height = _get_wheel_world_position(i).y
		var ground_height = _get_ground_height_at_position(_get_wheel_world_position(i).xz)
		
		var compression = ground_height - wheel_height
		compression = clampf(compression, suspension_travel_min, suspension_travel_max)
		
		var spring_force = spring_stiffness * (compression - suspension_travel_min)
		var damping_force = spring_damping_rate * (_suspension_compression[i] - compression) / delta
		
		_suspension_compression[i] = compression
		
		suspension_force = spring_force + damping_force
		suspension_force /= 4.0
		
		if abs(suspension_force) > 1000.0:
			emit_signal("suspension_compressed", abs(suspension_force) / 1000.0)

func _get_wheel_world_position(wheel_index: int) -> Vector3:
	var offset = Vector3.ZERO
	match wheel_index:
		0: offset = _front_left_wheel_pos
		1: offset = _front_right_wheel_pos
		2: offset = _rear_left_wheel_pos
		3: offset = _rear_right_wheel_pos
	
	return global_transform * offset

func _get_ground_height_at_position(position_xz: Vector2) -> float:
	var ray_position = position_xz * Vector2.RIGHT + Vector3(0.0, 100.0, 0.0)
	var space_state = get_world_3d().direct_space_state
	var query = PhysicsRayQueryParameters3D.create(ray_position, position_xz * Vector2.RIGHT + Vector3(0.0, -100.0, 0.0))
	query.exclude = [self]
	var result = space_state.intersect_ray(query)
	
	if result and result.has("position"):
		return result.position.y
	return 0.0

func _apply_velocity_to_body(delta: float) -> void:
	var forward_direction = global_transform.basis.z
	var right_direction = global_transform.basis.x
	
	var drift_correction = Vector3.ZERO
	if _is_drifting:
		var friction_modifier = DRIFT_FRICTION_REDUCTION
		drift_correction = right_direction * _velocity.dot(right_direction) * (1.0 - friction_modifier)
	
	var velocity_change = forward_direction * (_speed_kmh / 3.6) + drift_correction
	_velocity = velocity_change

func _handle_collisions(delta: float) -> void:
	if _collision_count > 0:
		if Time.get_ticks_msec() - _last_collision_time > 1000:
			_collision_count = 0
	
	var collision_shape = $CollisionShape3D
	if collision_shape and collision_shape.shape:
		var shape = collision_shape.shape
		var shape_rect = shape.get_rect()
		
		if shape_rect.size.x > 1.0 or shape_rect.size.z > 1.0:
			var overlap = shape_rect.size.x * shape_rect.size.z * 0.5
			
			if overlap > 0.1:
				_collision_count += 1
				_last_collision_time = Time.get_ticks_msec()
				
				var collision_data = {
					"timestamp": Time.get_ticks_msec() / 1000.0,
					"impact_force": overlap * vehicle_mass,
					"location": global_position
				}
				
				emit_signal("collision_detected", collision_data)
				
				if _collision_count >= 3:
					race_event.emit("crash_warning", {"count": _collision_count})

func _input_event(camera: Camera3D, event: InputEvent) -> void:
	if event is InputEventMouseButton and event.pressed:
		if event.button_index == MOUSE_BUTTON_LEFT:
			if Input.is_key_pressed(KEY_SPACE):
				manual_upshift()
			elif Input.is_key_pressed(KEY_BACKSLASH):
				manual_downshift()

</file_content>