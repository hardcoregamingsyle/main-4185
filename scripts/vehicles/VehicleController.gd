extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulation
## Handles throttle, brake, steering, wheel forces, gear shifting, and vehicle dynamics
## Integrates with GameManager, AudioManager, InputManager, and PhysicsSettings singletons
## Copyright 2026 Thalamus Racing Simulator Project

signal speed_changed(current_speed: float, max_speed: float)
signal gear_changed(old_gear: int, new_gear: int)
signal clutch_engaged(engaged: bool)
signal handbrake_toggled(active: bool)
signal vehicle_acceleration(force: Vector3)
signal wheel_slip_detected(wheel_index: int, slip_ratio: float)
signal skid_detected(skid_intensity: float)
signal drift_started(drift_angle: float)
signal drift_ended()

@export_group("Vehicle Configuration")
@export var vehicle_mass: float = 1500.0: set = _set_vehicle_mass
@export var center_of_mass: Vector3 = Vector3.ZERO: set = _set_center_of_mass
@export var vehicle_length: float = 4.5: set = _set_vehicle_length
@export var vehicle_width: float = 1.8: set = _set_vehicle_width
@export var vehicle_height: float = 1.3: set = _set_vehicle_height
@export var wheel_base: float = 2.7: set = _set_wheel_base
@export var track_width: float = 1.5: set = _set_track_width
@export var wheel_radius: float = 0.35: set = _set_wheel_radius
@export var tire_friction_coefficient: float = 1.2: set = _set_tire_friction_coefficient
@export var aerodynamic_drag_coefficient: float = 0.3: set = _set_aerodynamic_drag_coefficient
@export var frontal_area: float = 2.0: set = _set_frontal_area

@export_group("Powertrain Settings")
@export var engine_max_rpm: float = 8000.0: set = _set_engine_max_rpm
@export var engine_idle_rpm: float = 800.0: set = _set_engine_idle_rpm
@export var engine_peak_torque_rpm: float = 4500.0: set = _set_engine_peak_torque_rpm
@export var engine_max_torque: float = 400.0: set = _set_engine_max_torque
@export var transmission_gears: Array[int] = [3.5, 2.4, 1.7, 1.3, 0.95, 0.75, 0.6]: set = _set_transmission_gears
@export var final_drive_ratio: float = 3.5: set = _set_final_drive_ratio
@export var clutch_slip_threshold: float = 0.15: set = _set_clutch_slip_threshold

@export_group("Throttle & Brake")
@export var throttle_response_time: float = 0.15: set = _set_throttle_response_time
@export var brake_pressure_max: float = 12.0: set = _set_brake_pressure_max
@export var brake_bias_front: float = 0.6: set = _set_brake_bias_front
@export var anti_lock_braking_enabled: bool = true
@export var regenerative_braking_enabled: bool = false
@export var regenerative_braking_factor: float = 0.3

@export_group("Steering Dynamics")
@export var steering_ratio: float = 15.0: set = _set_steering_ratio
@export var steering_max_angle: float = 30.0: set = _set_steering_max_angle
@export var steering_speed: float = 45.0: set = _set_steering_speed
@export var ackermann_geometry: bool = true
@export var steering_deadzone: float = 0.05: set = _set_steering_deadzone

@export_group("Suspension Settings")
@export var suspension_stiffness: float = 45000.0: set = _set_suspension_stiffness
@export var suspension_damping: float = 4500.0: set = _set_suspension_damping
@export var suspension_travel_max: float = 0.15: set = _set_suspension_travel_max
@export var suspension_compression_rate: float = 2.0: set = _set_suspension_compression_rate
@export var suspension_rebound_rate: float = 1.5: set = _set_suspension_rebound_rate
@export var roll_bar_stiffness: float = 8000.0: set = _set_roll_bar_stiffness
@export var pitch_balance: float = 0.5: set = _set_pitch_balance

@export_group("Drivetrain Configuration")
@export var drivetrain_type: DrivetrainType = DrivetrainType.FWD
enum DrivetrainType { FWD, RWD, AWD }

@export_group("Handbrake Settings")
@export var handbrake_force: float = 8.0: set = _set_handbrake_force
@export var handbrake_locks_rear_wheels: bool = true

@export_group("AI Control Parameters")
@export var ai_aggressiveness: float = 0.5: set = _set_ai_aggressiveness
@export var ai_line_preference: float = 0.5: set = _set_ai_line_preference

var current_speed: float = 0.0
var vehicle_rotation_degrees: float = 0.0
var throttle_input: float = 0.0
var brake_input: float = 0.0
var steering_input: float = 0.0
var clutch_pedal: float = 0.0
var handbrake_active: bool = false

var current_gear: int = 1
var rpm: float = engine_idle_rpm
var clutch_engaged_state: bool = true
var wheel_angles: Array[float] = [0.0, 0.0, 0.0, 0.0]
var wheel_rotational_speeds: Array[float] = [0.0, 0.0, 0.0, 0.0]
var wheel_forces: Array[Vector3] = []
var wheel_contact_points: Array[Vector3] = []
var wheel_normal_vectors: Array[Vector3] = []

var acceleration_vector: Vector3 = Vector3.ZERO
var velocity_vector: Vector3 = Vector3.ZERO
var angular_velocity: float = 0.0
var drift_angle: float = 0.0
var grip_level: float = 1.0

var _target_steering_angle: float = 0.0
var _current_steering_angle: float = 0.0
var _throttle_target: float = 0.0
var _throttle_current: float = 0.0
var _gear_shift_timer: float = 0.0
var _last_gear_change_time: float = 0.0
var _wheel_slip_threshold: float = 0.15
var _skid_intensity: float = 0.0
var _drift_duration: float = 0.0
var _max_drift_duration: float = 5.0
var _is_skidding: bool = false
var _is_drifting: bool = false
var _physics_tick_rate: float = 120.0
var _substep_count: int = 4
var _time_scale: float = 1.0
var _ai_target_speed: float = 0.0
var _ai_target_position: Vector3 = Vector3.ZERO

func _ready() -> void:
	_process_mode = ProcessModeEnum.ALWAYS
	_init_constants()
	_setup_wheels()
	_connect_signals()
	_reset_vehicle_state()

func _init_constants() -> void:
	if Engine.has_singleton("PhysicsSettings"):
		var ps = PhysicsSettings
		_physics_tick_rate = ps.physics_tick_rate
		_substep_count = ps.max_substeps
		_time_scale = ps.time_scale

func _setup_wheels() -> void:
	wheel_angles = [0.0, 0.0, 0.0, 0.0]
	wheel_rotational_speeds = [0.0, 0.0, 0.0, 0.0]
	wheel_forces.resize(4)
	for i in range(4):
		wheel_forces[i] = Vector3.ZERO
	
	var half_track = track_width * 0.5
	var half_wheelbase = wheel_base * 0.5
	
	wheel_contact_points = [
		Vector3(-half_wheelbase, -half_track, 0.0), # Front Left
		Vector3(half_wheelbase, -half_track, 0.0),  # Front Right
		Vector3(-half_wheelbase, half_track, 0.0),  # Rear Left
		Vector3(half_wheelbase, half_track, 0.0)    # Rear Right
	]
	
	wheel_normal_vectors = [
		Vector3.UP, Vector3.UP, Vector3.UP, Vector3.UP
	]

func _connect_signals() -> void:
	InputManager.input_updated.connect(_on_input_updated)
	GameManager.game_state_changed.connect(_on_game_state_changed)

func _reset_vehicle_state() -> void:
	current_speed = 0.0
	rpm = engine_idle_rpm
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	clutch_pedal = 0.0
	handbrake_active = false
	current_gear = 1
	clutch_engaged_state = true
	_target_steering_angle = 0.0
	_current_steering_angle = 0.0
	_is_skidding = false
	_is_drifting = false
	_drift_duration = 0.0
	_grip_level = 1.0
	acceleration_vector = Vector3.ZERO
	angular_velocity = 0.0

func _input(event: InputEvent) -> void:
	if event.is_action_pressed("ui_cancel"):
		get_tree().paused = !get_tree().paused

func _process(delta: float) -> void:
	pass

func _physics_process(delta: float) -> void:
	_apply_substeps(delta)

func _apply_substeps(delta: float) -> void:
	var substep_delta = delta / _substep_count
	for i in range(_substep_count):
		_update_vehicle_physics(substep_delta)

func _update_vehicle_physics(delta: float) -> void:
	_update_inputs(delta)
	_update_throttle(delta)
	_update_steering(delta)
	_update_gearing(delta)
	_calculate_wheel_forces(delta)
	_apply_forces(delta)
	_update_driving_state(delta)
	_update_vfx(delta)

func _update_inputs(delta: float) -> void:
	if Engine.get_main_loop().root.find_child("InputManager"):
		throttle_input = InputManager.get_throttle()
		brake_input = InputManager.get_brake()
		steering_input = InputManager.get_steering()
		clutch_pedal = InputManager.get_clutch()
		
		var handbrake_pressed = Input.is_action_just_pressed("handbrake")
		var handbrake_released = Input.is_action_just_released("handbrake")
		if handbrake_pressed:
			handbrake_active = true
			emit_signal(handbrake_toggled, true)
		if handbrake_released:
			handbrake_active = false
			emit_signal(handbrake_toggled, false)

func _update_throttle(delta: float) -> void:
	_throttle_target = throttle_input
	_throttle_current += (_throttle_target - _throttle_current) * delta * (1.0 / throttle_response_time)
	_throttle_current = clamp(_throttle_current, 0.0, 1.0)
	
	clutch_engaged_state = (clutch_pedal < (1.0 - clutch_slip_threshold))
	emit_signal(clutch_engaged, clutch_engaged_state)

func _update_steering(delta: float) -> void:
	_target_steering_angle = steering_input * steering_max_angle
	
	if ackermann_geometry:
		_wheel_angles[0] = _calculate_ackermann_left(_target_steering_angle)
		_wheel_angles[1] = _calculate_ackermann_right(_target_steering_angle)
	else:
		_wheel_angles[0] = _target_steering_angle
		_wheel_angles[1] = _target_steering_angle
	
	_wheel_angles[2] = _wheel_angles[0]
	_wheel_angles[3] = _wheel_angles[1]
	
	_current_steering_angle = lerp(_current_steering_angle, _target_steering_angle, delta * steering_speed)

func _calculate_ackermann_left(angle: float) -> float:
	return angle * 0.95

func _calculate_ackermann_right(angle: float) -> float:
	return angle * 1.05

func _update_gearing(delta: float) -> void:
	_update_rpm(delta)
	_auto_shift_gear(delta)

func _update_rpm(delta: float) -> void:
	var wheel_rpm = (current_speed / (2.0 * PI * wheel_radius)) * transmission_gears[current_gear - 1] * final_drive_ratio
	rpm = lerp(rpm, wheel_rpm, delta * 10.0)
	rpm = clamp(rpm, engine_idle_rpm, engine_max_rpm)

func _auto_shift_gear(delta: float) -> void:
	_gear_shift_timer += delta
	if _gear_shift_timer > 0.5:
		var target_gear = _find_optimal_gear()
		if target_gear != current_gear:
			_shift_gear(target_gear)
		_gear_shift_timer = 0.0

func _find_optimal_gear() -> int:
	var optimal_gear = 1
	for i in range(transmission_gears.size()):
		var wheel_rpm_at_gear = (current_speed / (2.0 * PI * wheel_radius)) * transmission_gears[i] * final_drive_ratio
		if wheel_rpm_at_gear < engine_peak_torque_rpm:
			optimal_gear = i + 1
		else:
			break
	return clamp(optimal_gear, 1, transmission_gears.size())

func _shift_gear(new_gear: int) -> void:
	if new_gear == current_gress:
		return
	
	var old_gear = current_gear
	current_gear = new_gear
	_last_gear_change_time = Time.get_ticks_msec()
	_gear_shift_timer = 0.0
	
	emit_signal(gear_changed, old_gear, new_gear)

func _calculate_wheel_forces(delta: float) -> void:
	var engine_force = _calculate_engine_force()
	var total_brake_force = _calculate_brake_force()
	var total_steering_force = _calculate_steering_force()
	var drag_force = _calculate_drag_force()
	var lift_force = _calculate_lift_force()
	
	var drive_wheels = _get_drive_wheels()
	
	for i in range(4):
		wheel_forces[i] = Vector3.ZERO
		
		if clutch_engaged_state:
			if drive_wheels.has(i):
				wheel_forces[i].x = engine_force * 0.25
				if handbrake_active && (i == 2 || i == 3):
					wheel_forces[i].z = -handbrake_force
			else:
				wheel_forces[i].x = 0.0
		
		var brake_force_per_wheel = total_brake_force * (brake_bias_front if i < 2 else (1.0 - brake_bias_front))
		wheel_forces[i].y = -brake_force_per_wheel
		
		var lateral_force = 0.0
		if i < 2:
			lateral_force = total_steering_force * abs(wheel_angles[i])
		wheel_forces[i].z = lateral_force
		
		var slip_ratio = _calculate_wheel_slip(i)
		if abs(slip_ratio) > _wheel_slip_threshold:
			emit_signal(wheel_slip_detected, i, slip_ratio)
		
		wheel_forces[i] *= grip_level

func _calculate_engine_force() -> float:
	var torque = _calculate_engine_torque()
	var gear_ratio = transmission_gears[current_gear - 1]
	var force = (torque * gear_ratio * final_drive_ratio) / wheel_radius
	return force

func _calculate_engine_torque() -> float:
	var normalized_rpm = (rpm - engine_idle_rpm) / (engine_max_rpm - engine_idle_rpm)
	var peak_normalized = (engine_peak_torque_rpm - engine_idle_rpm) / (engine_max_rpm - engine_idle_rpm)
	
	var torque_curve = exp(-pow((normalized_rpm - peak_normalized) / 0.3, 2))
	var torque = torque_curve * engine_max_torque
	
	return torque * _throttle_current

func _calculate_brake_force() -> float:
	var base_brake = brake_pressure_max * brake_input
	if anti_lock_braking_enabled:
		base_brake *= _calculate_abs_modulation()
	return base_brake

func _calculate_abs_modulation() -> float:
	if current_speed < 1.0:
		return 1.0
	var slip_indicator = _estimate_wheel_slip()
	return clamp(1.0 - slip_indicator * 0.5, 0.5, 1.0)

func _estimate_wheel_slip() -> float:
	var estimated_slip = 0.0
	for i in range(4):
		estimated_slip += abs(wheel_rotational_speeds[i] - (current_speed / wheel_radius))
	return estimated_slip / 4.0

func _calculate_steering_force() -> float:
	return steering_input * 100.0

func _calculate_drag_force() -> float:
	var air_density = 1.225
	var drag = 0.5 * air_density * aerodynamic_drag_coefficient * frontal_area * current_speed * current_speed
	return drag

func _calculate_lift_force() -> float:
	var air_density = 1.225
	var lift = 0.5 * air_density * aerodynamic_drag_coefficient * frontal_area * current_speed * current_speed
	return lift * 0.3

func _get_drive_wheels() -> Array[int>:
	match drivetrain_type:
		DrivetrainType.FWD:
			return [0, 1]
		DrivetrainType.RWD:
			return [2, 3]
		DrivetrainType.AWD:
			return [0, 1, 2, 3]
		_:
			return [0, 1, 2, 3]

func _calculate_wheel_slip(wheel_index: int) -> float:
	var wheel_linear_speed = wheel_rotational_speeds[wheel_index] * wheel_radius
	if current_speed == 0.0:
		return 0.0
	var slip = (wheel_linear_speed - current_speed) / current_speed
	return clamp(slip, -1.0, 1.0)

func _apply_forces(delta: float) -> void:
	var total_force = Vector3.ZERO
	var total_torque = 0.0
	
	for i in range(4):
		total_force += wheel_forces[i]
		
		var offset = wheel_contact_points[i]
		total_torque += offset.cross(wheel_forces[i]).y

	var net_acceleration = total_force / vehicle_mass
	velocity_vector += net_acceleration * delta
	
	rotate_y(angular_velocity * delta)
	position += velocity_vector * delta
	
	current_speed = velocity_vector.length()
	vehicle_rotation_degrees = rad_to_deg(get_rotation().y)

func _update_driving_state(delta: float) -> void:
	_check_drift_state(delta)
	_check_skid_state()
	_update_grip_level()
	
	emit_signal(speed_changed, current_speed, _calculate_max_speed())

func _check_drift_state(delta: float) -> void:
	var slip_angle = _calculate_slip_angle()
	
	if abs(slip_angle) > 10.0 && current_speed > 5.0:
		if not _is_drifting:
			_is_drifting = true
			_drift_duration = 0.0
			emit_signal(drift_started, slip_angle)
		_drift_duration += delta
		drift_angle = slip_angle
		
		if _drift_duration > _max_drift_duration:
			_end_drift()
	else:
		if _is_drifting:
			_end_drift()
		drift_angle = 0.0

func _calculate_slip_angle() -> float:
	var forward = get_forward()
	var velocity_direction = velocity_vector.normalized()
	var dot_product = forward.dot(velocity_direction)
	var cross_product_z = forward.x * velocity_direction.z - forward.z * velocity_direction.x
	var slip_angle = atan2(cross_product_z, dot_product)
	return deg_to_rad(slip_angle) * 180.0 / PI

func _end_drift() -> void:
	_is_drifting = false
	_drift_duration = 0.0
	emit_signal(drift_ended())

func _check_skid_state() -> void:
	var total_slip = 0.0
	for i in range(4):
		total_slip += abs(_calculate_wheel_slip(i))
	total_slip /= 4.0
	
	if total_slip > 0.2:
		_is_skidding = true
		_skid_intensity = total_slip * 10.0
		emit_signal(skid_detected, _skid_intensity)
	else:
		_is_skidding = false
		_skid_intensity = 0.0

func _update_grip_level() -> void:
	if _is_skidding:
		_grip_level = 1.0 - (_skid_intensity * 0.5)
	elif _is_drifting:
		_grip_level = 0.6
	else:
		_grip_level = 1.0
	
	_grip_level = clamp(_grip_level, 0.1, 1.0)

func _update_vfx(delta: float) -> void:
	if _is_skidding:
		_spawn_skid_particles()
	if _is_drifting:
		_spawn_drift_smoke()

func _spawn_skid_particles() -> void:
	if randf() < 0.1:
		var particle = GDScript.new("ParticleProcessMaterial")
		particle.emission_shape = ParticleProcessMaterial.EMISSION_SHAPE_SPHERE
		particle.amount = 5
		particle.lifetime = 0.5
		particle.color = Color(0.5, 0.5, 0.5)

func _spawn_drift_smoke() -> void:
	if randf() < 0.05:
		var smoke = GDScript.new("GPUParticles3D")
		smoke.emission_box_extents = Vector3(0.2, 0.2, 0.2)
		smoke.amount = 3
		smoke.lifetime = 1.0
		smoke.color = Color(0.3, 0.3, 0.3, 0.6)

func _calculate_max_speed() -> float:
	var gear_ratio = transmission_gears[current_gear - 1]
	var max_wheel_rpm = engine_max_rpm
	var max_speed = (max_wheel_rpm / (final_drive_ratio * gear_ratio)) * (2.0 * PI * wheel_radius)
	return max_speed * 3.6

func _set_vehicle_mass(value: float) -> void:
	vehicle_mass = value

func _set_center_of_mass(value: Vector3) -> void:
	center_of_mass = value

func _set_vehicle_length(value: float) -> void:
	vehicle_length = value

func _set_vehicle_width(value: float) -> void:
	vehicle_width = value

func _set_vehicle_height(value: float) -> void:
	vehicle_height = value

func _set_wheel_base(value: float) -> void:
	wheel_base = value

func _set_track_width(value: float) -> void:
	track_width = value

func _set_wheel_radius(value: float) -> void:
	wheel_radius = value

func _set_tire_friction_coefficient(value: float) -> void:
	tire_friction_coefficient = value

func _set_aerodynamic_drag_coefficient(value: float) -> void:
	aerodynamic_drag_coefficient = value

func _set_frontal_area(value: float) -> void:
	frontal_area = value

func _set_engine_max_rpm(value: float) -> void:
	engine_max_rpm = value

func _set_engine_idle_rpm(value: float) -> void:
	engine_idle_rpm = value

func _set_engine_peak_torque_rpm(value: float) -> void:
	engine_peak_torque_rpm = value

func _set_engine_max_torque(value: float) -> void:
	engine_max_torque = value

func _set_transmission_gears(value: Array[int]) -> void:
	transmission_gears = value

func _set_final_drive_ratio(value: float) -> void:
	final_drive_ratio = value

func _set_clutch_slip_threshold(value: float) -> void:
	clutch_slip_threshold = value

func _set_throttle_response_time(value: float) -> void:
	throttle_response_time = value

func _set_brake_pressure_max(value: float) -> void:
	brake_pressure_max = value

func _set_brake_bias_front(value: float) -> void:
	brake_bias_front = value

func _set_steering_ratio(value: float) -> void:
	steering_ratio = value

func _set_steering_max_angle(value: float) -> void:
	steering_max_angle = value

func _set_steering_speed(value: float) -> void:
	steering_speed = value

func _set_steering_deadzone(value: float) -> void:
	steering_deadzone = value

func _set_suspension_stiffness(value: float) -> void:
	suspension_stiffness = value

func _set_suspension_damping(value: float) -> void:
	suspension_damping = value

func _set_suspension_travel_max(value: float) -> void:
	suspension_travel_max = value

func _set_suspension_compression_rate(value: float) -> void:
	suspension_compression_rate = value

func _set_suspension_rebound_rate(value: float) -> void:
	suspension_rebound_rate = value

func _set_roll_bar_stiffness(value: float) -> void:
	roll_bar_stiffness = value

func _set_pitch_balance(value: float) -> void:
	pitch_balance = value

func _set_handbrake_force(value: float) -> void:
	handbrake_force = value

func _set_ai_aggressiveness(value: float) -> void:
	ai_aggressiveness = value

func _set_ai_line_preference(value: float) -> void:
	ai_line_preference = value

func _on_input_updated(data: Dictionary) -> void:
	if data.has("throttle"):
		throttle_input = data.throttle
	if data.has("brake"):
		brake_input = data.brake
	if data.has("steering"):
		steering_input = data.steering
	if data.has("clutch"):
		clutch_pedal = data.clutch

func _on_game_state_changed(state: GameState) -> void:
	match state:
		GameState.MAIN_MENU:
			_reset_vehicle_state()
		GameState.RACE_ACTIVE:
			pass

func set_ai_control(enabled: bool, target_speed: float = 0.0, target_position: Vector3 = Vector3.ZERO) -> void:
	if enabled:
		_ai_target_speed = target_speed
		_ai_target_position = target_position
		_enable_ai_logic()
	else:
		_disable_ai_logic()

func _enable_ai_logic() -> void:
	pass

func _disable_ai_logic() -> void:
	pass

func get_vehicle_state() -> Dictionary:
	return {
		"speed": current_speed,
		"rpm": rpm,
		"gear": current_gear,
		"throttle": _throttle_current,
		"brake": brake_input,
		"steering": _current_steering_angle,
		"position": position,
		"rotation": get_rotation(),
		"is_drifting": _is_drifting,
		"is_skidding": _is_skidding,
		"grip_level": _grip_level
	}

func reset_vehicle() -> void:
	_reset_vehicle_state()
	velocity_vector = Vector3.ZERO
	angular_velocity = 0.0
	set_collision_layer_value(1, true)
	set_collision_mask_value(1, true)

func apply_impact(impact_point: Vector3, impact_force: Vector3) -> void:
	var impact_offset = impact_point - position
	var impact_torque = impact_offset.cross(impact_force)
	velocity_vector += impact_force / vehicle_mass
	angular_velocity += impact_torque.y / (vehicle_mass * wheel_base * 0.5)

func debug_print_stats() -> void:
	print("[VehicleStats]")
	print("\tSpeed: %.2f km/h" % current_speed)
	print("\tRPM: %.0f" % rpm)
	print("\tGear: %d" % current_gear)
	print("\tThrottle: %.2f" % _throttle_current)
	print("\tBrake: %.2f" % brake_input)
	print("\tSteering: %.2f deg" % _current_steering_angle)
	print("\tDrifting: %s" % str(_is_drifting))
	print("\tSkidding: %s" % str(_is_skidding))