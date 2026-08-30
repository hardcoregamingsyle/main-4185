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
signal lap_completed(lap_time: float)
signal vehicle_launched(is_jumping: bool)
signal wheelie_state_changed(is_wheely: bool)
signal burnout_started(duration: float)
signal burnout_ended()

# ============================================================================
# EXPORTED SETTINGS
# ============================================================================
@export_group("Vehicle Configuration")
@export var vehicle_mass: float = 1500.0
@export var center_of_gravity_offset: Vector3 = Vector3(0.0, 0.5, 0.0)
@export var wheelbase: float = 2.5
@export var track_width: float = 1.6
@export var suspension_travel: float = 0.15
@export var tire_radius: float = 0.32

@export_group("Powertrain Settings")
@export var max_engine_rpm: float = 8000.0
@export var idle_rpm: float = 800.0
@export var redline_rpm: float = 7500.0
@export var torque_curve_points: Array[Vector2] = [
	Vector2(1000, 0.2),
	Vector2(3000, 0.5),
	Vector2(5000, 0.9),
	Vector2(6500, 1.0),
	Vector2(8000, 0.7)
]
@export var transmission_gears: Array[float] = [3.5, 2.2, 1.5, 1.0, 0.7, 0.5]
@export var final_drive_ratio: float = 3.5
@export var clutch_disengagement_threshold: float = 200.0

@export_group("Drivetrain Settings")
@export var drivetrain_type: DrivetrainType = DrivetrainType.FWD
@export var differential_type: DifferentialType = DifferentialType.OPEN
@export var limited_slip_diff_ratio: float = 1.5

@export_group("Braking System")
@export var front_brake_bias: float = 0.6
@export var max_brake_force: float = 15000.0
@export var brake_pressure_max: float = 100.0
@export var brake_sensitivity: float = 1.0

@export_group("Tire & Suspension")
@export var tire_friction_coefficient: float = 1.2
@export var grip_level: float = 1.0
@export var slip_angle_threshold: float = 5.0
@export var lateral_acceleration_limit: float = 1.5
@export var roll_bar_stiffness: float = 1.0
@export var suspension_damping: float = 0.3

@export_group("Control Systems")
@export var traction_control_enabled: bool = true
@export var abs_enabled: bool = true
@export var electronic_stability_control: bool = true
@export var automatic_transmission: bool = true
@export var auto_rev_matching: bool = true
@export var drift_mode_enabled: bool = false

@export_group("AI Behavior")
@export var ai_line_preference: float = 0.5
@export var ai_aggressiveness: float = 0.5
@export var ai_reaction_delay: float = 0.15

# ============================================================================
# CONSTANTS & ENUMS
# ============================================================================
enum DrivetrainType { FWD, RWD, AWD }
enum DifferentialType { OPEN, LSD, LOCKED }
enum WheelPosition { FRONT_LEFT, FRONT_RIGHT, REAR_LEFT, REAR_RIGHT }
enum GearState { NEUTRAL, FIRST, SECOND, THIRD, FOURTH, FIFTH, SIXTH, SEVENTH, EIGHTH, BACKWARDS }

# ============================================================================
# PHYSICS PROPERTIES (Read-only after init)
# ============================================================================
var current_speed: float = 0.0
var current_velocity: Vector3 = Vector3.ZERO
var current_rpm: float = idle_rpm
var current_gear: int = 0
var target_gear: int = 0
var is_in_gear: bool = false
var clutch_engaged: bool = true
var clutch_pedal_position: float = 1.0

var throttle_input: float = 0.0
var brake_input: float = 0.0
var steering_input: float = 0.0
var handbrake_input: float = 0.0

var wheel_steering_angles: Vector4 = Vector4(0.0, 0.0, 0.0, 0.0)
var wheel_rotation_angles: Vector4 = Vector4(0.0, 0.0, 0.0, 0.0)
var wheel_vertical_positions: Vector4 = Vector4(0.0, 0.0, 0.0, 0.0)
var wheel_contact_forces: Vector4 = Vector4(0.0, 0.0, 0.0, 0.0)

var skid_percentage: float = 0.0
var drift_angle: float = 0.0
var is_drifting: bool = false
var is_skidding: bool = false
var is_on_ground: bool = true
var is_jumping: bool = false
var is_wheely: bool = false

var acceleration_history: Array[float] = []
var max_acceleration_history: int = 60

# ============================================================================
# INTERNAL STATE
# ============================================================================
var _last_turn_direction: int = 0
var _drift_accumulator: float = 0.0
var _handbrake_timer: float = 0.0
var _burnout_timer: float = 0.0
var _engine_temperature: float = 90.0
var _oil_pressure: float = 45.0
var _fuel_level: float = 100.0
var _engine_health: float = 100.0
var _transmission_health: float = 100.0
var _tire_temperatures: Vector4 = Vector4(80.0, 80.0, 80.0, 80.0)
var _suspension_states: Vector4 = Vector4(0.0, 0.0, 0.0, 0.0)
var _brake_temperatures: Vector4 = Vector4(100.0, 100.0, 100.0, 100.0)
var _collision_cache: Array[Dictionary] = []
var _input_buffer: Array[Dictionary] = []
var _ai_target_point: Vector3 = Vector3.ZERO
var _ai_next_waypoint: Vector3 = Vector3.ZERO
var _lap_time_accumulator: float = 0.0
var _lap_count: int = 0
var _checkpoints_passed: Array[int] = []

# ============================================================================
# TIRE CONTACT POINTS
# ============================================================================
var _front_left_wheel_pos: Vector3 = Vector3.ZERO
var _front_right_wheel_pos: Vector3 = Vector3.ZERO
var _rear_left_wheel_pos: Vector3 = Vector3.ZERO
var _rear_right_wheel_pos: Vector3 = Vector3.ZERO

# ============================================================================
# VEHICLE MODES
# ============================================================================
enum VehicleMode { NORMAL, DRIFT, BURNOUT, LAUNCH, CRUISE }
var current_vehicle_mode: VehicleMode = VehicleMode.NORMAL

# ============================================================================
# AUDIO REFERENCES
# ============================================================================
var _engine_audio_node: Node = null
var _tire_audio_node: Node = null
var _gear_audio_node: Node = null

# ============================================================================
# SETUP METHODS
# ============================================================================
func _ready() -> void:
	_init_internal_state()
	_connect_signals_to_system()
	_setup_collision_detection()
	_process_inputs()

func _process(delta: float) -> void:
	_update_physics(delta)
	_update_vehicle_states(delta)
	_update_audio_feedback(delta)
	_handle_drift_logic(delta)
	_handle_collision_detection(delta)

func _physics_process(delta: float) -> void:
	_apply_physics(delta)
	_update_suspension(delta)
	_update_wheels(delta)
	_check_gear_shifts(delta)

# ============================================================================
# INITIALIZATION
# ============================================================================
func _init_internal_state() -> void:
	acceleration_history.resize(max_acceleration_history)
	for i in range(max_acceleration_history):
		acceleration_history[i] = 0.0
	
	if AudioManager != null:
		_engine_audio_node = AudioManager.create_audio_source("vehicle_engine", self)
		_tire_audio_node = AudioManager.create_audio_source("vehicle_tires", self)
		_gear_audio_node = AudioManager.create_audio_source("vehicle_gears", self)
	
	_set_initial_wheel_positions()

func _connect_signals_to_system() -> void:
	if GameManager != null:
		game_manager.game_state_changed.connect(_on_game_state_changed)
		game_manager.race_started.connect(_on_race_started)
		game_manager.vehicle_spawned.connect(_on_vehicle_spawned)

func _setup_collision_detection() -> void:
	collision_layer = 1  # Vehicle layer
	collision_mask = 7   # Collides with terrain, other vehicles, obstacles

# ============================================================================
# WHEEL POSITION SETUP
# ============================================================================
func _set_initial_wheel_positions() -> void:
	var half_track = track_width * 0.5
	var half_wheelbase = wheelbase * 0.5
	
	_front_left_wheel_pos = Vector3(-half_track, 0.0, half_wheelbase)
	_front_right_wheel_pos = Vector3(half_track, 0.0, half_wheelbase)
	_rear_left_wheel_pos = Vector3(-half_track, 0.0, -half_wheelbase)
	_rear_right_wheel_pos = Vector3(half_track, 0.0, -half_wheelbase)

# ============================================================================
# INPUT PROCESSING
# ============================================================================
func _process_inputs() -> void:
	_throttle_input = InputManager.get_axis("throttle_up", "throttle_down")
	_brake_input = InputManager.get_axis("brake", "foot_brake")
	_steering_input = InputManager.get_axis("steer_left", "steer_right")
	_handbrake_input = InputManager.get_axis("handbrake")
	
	_clutch_pedal_position = InputManager.get_axis("clutch_up", "clutch_down")
	clutch_engaged = _clutch_pedal_position > 0.5
	
	if automatic_transmission:
		target_gear = _calculate_auto_gear()
	else:
		_handle_manual_shifts()

# ============================================================================
# MANUAL GEAR SHIFTS
# ============================================================================
func _handle_manual_shifts() -> void:
	if Input.is_action_just_pressed("shift_up"):
		_request_gear_change(current_gear + 1)
	elif Input.is_action_just_pressed("shift_down"):
		_request_gear_change(current_gear - 1)

func _request_gear_change(new_gear: int) -> void:
	var old_gear = current_gear
	new_gear = clampi(new_gear, -1, transmission_gears.size())
	
	if new_gear != current_gear and new_gear >= -1:
		target_gear = new_gear
		
		if drivetrain_type == DrivetrainType.RWD or drivetrain_type == DrivetrainType.AWD:
			clutch_engaged = false
		
		await _perform_gear_change(old_gear, new_gear)
		
		if drivetrain_type == DrivetrainType.RWD or drivetrain_type == DrivetrainType.AWD:
			await get_tree().create_timer(0.15).timeout
			clutch_engaged = true

# ============================================================================
# AUTO GEAR CALCULATION
# ============================================================================
func _calculate_auto_gear() -> int:
	var speed_gear_ratio = current_speed / 10.0
	
	if speed_gear_ratio < 5.0:
		return 1
	elif speed_gear_ratio < 15.0:
		return 2
	elif speed_gear_ratio < 25.0:
		return 3
	elif speed_gear_ratio < 40.0:
		return 4
	elif speed_gear_ratio < 60.0:
		return 5
	elif speed_gear_ratio < 80.0:
		return 6
	elif speed_gear_ratio < 100.0:
		return 7
	else:
		return 8

# ============================================================================
# GEAR CHANGE EXECUTION
# ============================================================================
func _perform_gear_change(old_gear: int, new_gear: int) -> void:
	gear_changed.emit(old_gear, new_gear)
	current_gear = new_gear
	
	is_in_gear = new_gear >= 0
	
	if is_in_gear:
		_play_gear_sound(new_gear)
	else:
		_play_gear_sound("neutral")
	
	_check_engine_stall()

# ============================================================================
# ENGINE RPM & TORQUE CALCULATIONS
# ============================================================================
func _update_rpm(delta: float) -> void:
	var gear_ratio: float
	var reverse_multiplier: float = -1.0
	
	if current_gear == 0 or not is_in_gear:
		gear_ratio = 1.0
		reverse_multiplier = 1.0
	else:
		gear_ratio = transmission_gears[current_gear - 1] if current_gear > 0 else 1.0
		reverse_multiplier = -1.0 if current_gear < 0 else 1.0
	
	var wheel_rpm = current_speed / (2.0 * PI * tire_radius)
	var target_engine_rpm = wheel_rpm * gear_ratio * final_drive_ratio * reverse_multiplier
	
	if not clutch_engaged:
		target_engine_rpm = idle_rpm
	
	if target_engine_rpm < idle_rpm:
		target_engine_rpm = idle_rpm
	if target_engine_rpm > max_engine_rpm:
		target_engine_rpm = max_engine_rpm
	
	_current_rpm = lerp(_current_rpm, target_engine_rpm, delta * 15.0)
	rpm_changed.emit(_current_rpm)

# ============================================================================
# TORQUE CURVE INTERPOLATION
# ============================================================================
func get_torque_at_rpm(rpm: float) -> float:
	for i in range(torque_curve_points.size() - 1):
		var p1 = torque_curve_points[i]
		var p2 = torque_curve_points[i + 1]
		
		if rpm >= p1.x and rpm <= p2.x:
			var t = (rpm - p1.x) / (p2.x - p1.x)
			return p1.y + t * (p2.y - p1.y)
	
	return torque_curve_points.min()[1]

func get_power_at_rpm(rpm: float) -> float:
	var torque = get_torque_at_rpm(rpm)
	return torque * rpm / 9549.0

# ============================================================================
# VELOCITY UPDATE
# ============================================================================
func _update_physics(delta: float) -> void:
	_update_rpm(delta)
	_calculate_acceleration(delta)
	_apply_velocity()
	_update_current_speed()

func _calculate_acceleration(delta: float) -> void:
	var driving_force: float = 0.0
	var braking_force: float = 0.0
	var rolling_resistance: float = 0.0
	var air_drag: float = 0.0
	
	if is_in_gear and clutch_engaged:
		var torque = get_torque_at_rpm(_current_rpm) * vehicle_mass * 0.001
		var wheel_radius = tire_radius * 0.95
		driving_force = torque / wheel_radius * get_drive_distribution()
		
		if throttle_input > 0.1:
			skid_percentage = _calculate_wheel_slip(driving_force)
	
	braking_force = _calculate_braking_force()
	rolling_resistance = vehicle_mass * 9.81 * 0.015
	air_drag = 0.5 * 1.225 * pow(current_speed, 2) * 2.2 * 0.35
	
	var net_force = driving_force - braking_force - rolling_resistance - air_drag
	var acceleration = net_force / vehicle_mass
	
	acceleration_history.push_back(acceleration)
	if acceleration_history.size() > max_acceleration_history:
		acceleration_history.pop_front()
	
	velocity += Vector3(acceleration, 0.0, 0.0) * delta
	current_velocity = velocity

func _apply_velocity() -> void:
	if is_on_ground:
		move_and_slide()
	else:
		velocity.y -= PhysicsSettings.gravity * delta
		position += velocity * delta

func _update_current_speed() -> void:
	var ground_speed = velocity.length()
	
	if ground_speed > current_speed * 1.1:
		current_speed = lerp(current_speed, ground_speed, 0.1)
	else:
		current_speed = lerp(current_speed, ground_speed, 0.05)
	
	speed_changed.emit(current_speed)

# ============================================================================
# BRAKING SYSTEM
# ============================================================================
func _calculate_braking_force() -> float:
	var total_brake_pressure = _brake_input * brake_pressure_max
	
	if abs_enabled and current_speed > 10.0:
		var wheel_slip = _calculate_wheel_slip(total_brake_force)
		if wheel_slip > 0.2:
			total_brake_pressure *= 0.7
	
	var front_brake_force = total_brake_pressure * max_brake_force * front_brake_bias
	var rear_brake_force = total_brake_pressure * max_brake_force * (1.0 - front_brake_bias)
	
	var total_force = front_brake_force + rear_brake_force
	
	if handbrake_input > 0.3:
		total_force += handbrake_input * max_brake_force * 0.5
	
	brake_applied.emit(_brake_input)
	
	_update_brake_temperatures(total_brake_pressure)
	
	return total_force

func _update_brake_temperatures(pressure: float) -> void:
	var temp_increase = pressure * 2.0
	_brake_temperatures.x = min(_brake_temperatures.x + temp_increase, 600.0)
	_brake_temperatures.y = min(_brake_temperatures.y + temp_increase, 600.0)
	_brake_temperatures.z = min(_brake_temperatures.z + temp_increase, 600.0)
	_brake_temperatures.w = min(_brake_temperatures.w + temp_increase, 600.0)

# ============================================================================
# DRIVE DISTRIBUTION
# ============================================================================
func get_drive_distribution() -> Vector4:
	match drivetrain_type:
		DrivetrainType.FWD:
			return Vector4(1.0, 1.0, 0.0, 0.0)
		DrivetrainType.RWD:
			return Vector4(0.0, 0.0, 1.0, 1.0)
		DrivetrainType.AWD:
			var ratio = 0.4 + (current_rpm / max_engine_rpm) * 0.2
			return Vector4(ratio, ratio, 1.0 - ratio, 1.0 - ratio)
		_:
			return Vector4(0.5, 0.5, 0.5, 0.5)

# ============================================================================
# WHEEL SLIP CALCULATION
# ============================================================================
func _calculate_wheel_slip(applied_force: float) -> float:
	var drive_force = applied_force * get_drive_distribution()
	var normal_force = vehicle_mass * 9.81 * 0.25
	
	var slip_x = drive_force.x / (normal_force * tire_friction_coefficient)
	var slip_y = drive_force.y / (normal_force * tire_friction_coefficient)
	
	return sqrt(slip_x * slip_x + slip_y * slip_y)

# ============================================================================
# DRIFT LOGIC
# ============================================================================
func _handle_drift_logic(delta: float) -> void:
	if not drift_mode_enabled and not handbrake_input > 0.3:
		is_drifting = false
		return
	
	var lateral_acceleration = velocity.cross(Vector3.UP).length()
	var drift_threshold = 0.8 * lateral_acceleration_limit
	
	if lateral_acceleration > drift_threshold and handbrake_input > 0.3:
		_drift_accumulator += delta * (lateral_acceleration - drift_threshold)
		
		if _drift_accumulator > 0.5:
			_start_drift()
		
		is_skidding = true
		skidding.emit(true)
	else:
		_drift_accumulator = max(0.0, _drift_accumulator - delta * 2.0)
		
		if _drift_accumulator < 0.1:
			_end_drift()
		
		is_skidding = false
		skidding.emit(false)

func _start_drift() -> void:
	is_drifting = true
	current_vehicle_mode = VehicleMode.DRIFT
	var actual_drift_angle = _calculate_drift_angle()
	drift_started.emit(actual_drift_angle)
	_play_drift_sound()

func _end_drift() -> void:
	if is_drifting:
		is_drifting = false
		current_vehicle_mode = VehicleMode.NORMAL
		drift_ended.emit()
		_drift_accumulator = 0.0

func _calculate_drift_angle() -> float:
	var yaw = rotation.y
	var velocity_angle = atan2(velocity.x, velocity.z)
	return yaw - velocity_angle

# ============================================================================
# SUSPENSION SYSTEM
# ============================================================================
func _update_suspension(delta: float) -> void:
	var wheel_positions = [_front_left_wheel_pos, _front_right_wheel_pos, 
	                       _rear_left_wheel_pos, _rear_right_wheel_pos]
	
	for i in range(4):
		var world_pos = global_transform * wheel_positions[i]
		var ray_from = world_pos + Vector3.UP * 0.5
		var ray_to = world_pos - Vector3.UP * suspension_travel
		
		var space_state = get_world_3d().direct_space_state
		var query = PhysicsRayQueryParameters3D.create(ray_from, ray_to)
		var result = space_state.intersect_ray(query)
		
		if result:
			var travel = (ray_from - result.position).length()
			_suspension_states[i] = travel
			wheel_vertical_positions[i] = travel
			
			var spring_force = travel / suspension_travel * 50000.0
			wheel_contact_forces[i] = spring_force

# ============================================================================
# WHEEL ROTATION UPDATE
# ============================================================================
func _update_wheels(delta: float) -> void:
	var steering_factor = steering_input * 0.5
	
	wheel_steering_angles.x = steering_factor
	wheel_steering_angles.y = steering_factor
	wheel_steering_angles.z = 0.0
	wheel_steering_angles.w = 0.0
	
	var wheel_rotation_delta = current_speed * delta / (2.0 * PI * tire_radius)
	
	wheel_rotation_angles.x += wheel_rotation_delta
	wheel_rotation_angles.y += wheel_rotation_delta
	wheel_rotation_angles.z += wheel_rotation_delta
	wheel_rotation_angles.w += wheel_rotation_delta

# ============================================================================
# COLLISION HANDLING
# ============================================================================
func _handle_collision_detection(delta: float) -> void:
	if colliding:
		var collision_info = _get_collision_info()
		collision_detected.emit(collision_info)
		
		var impact_speed = velocity.length()
		if impact_speed > 15.0:
			_trigger_screen_shake(impact_speed * 0.1)
		
		_engine_health -= impact_speed * 0.5
		_transmission_health -= impact_speed * 0.3
		
		if _engine_health <= 0:
			engine_stalled.emit()
			_stop_vehicle()

func _get_collision_info() -> Dictionary:
	var info = {}
	info["collided"] = true
	info["position"] = global_position
	info["velocity"] = velocity.length()
	info["direction"] = velocity.normalized()
	
	if collision_get_collider():
		info["collider_name"] = collision_get_collider().name
	
	return info

func _trigger_screen_shake(intensity: float) -> void:
	if CameraShake != null:
		CameraShake.shake(intensity)

# ============================================================================
# VEHICLE MODE MANAGEMENT
# ============================================================================
func set_vehicle_mode(mode: VehicleMode) -> void:
	current_vehicle_mode = mode
	
	match mode:
		VehicleMode.NORMAL:
			enable_all_controls(true)
		VehicleMode.DRIFT:
			enable_all_controls(true)
			_drift_mode_enabled = true
		VehicleMode.BURNOUT:
			throttle_input = 1.0
			_handbrake_input = 1.0
			_burnout_timer = 5.0
		VehicleMode.LAUNCH:
			prepare_launch()
		VehicleMode.CRUISE:
			enable_cruise_control()

func enable_all_controls(enabled: bool) -> void:
	input_blocked = not enabled

# ============================================================================
# LAUNCH CONTROL
# ============================================================================
func prepare_launch() -> void:
	current_vehicle_mode = VehicleMode.LAUNCH
	var launch_rpm = 4000.0
	_current_rpm = lerp(_current_rpm, launch_rpm, 0.5)

func execute_launch() -> void:
	var torque = get_torque_at_rpm(_current_rpm) * vehicle_mass * 0.001
	var force = torque / tire_radius * 0.9
	
	velocity.x += force * 0.016
	
	if current_speed > 100.0:
		current_vehicle_mode = VehicleMode.NORMAL

# ============================================================================
# CRUISE CONTROL
# ============================================================================
var cruise_control_target_speed: float = 0.0
var cruise_control_active: bool = false

func enable_cruise_control() -> void:
	cruise_control_active = true
	cruise_control_target_speed = current_speed

func update_cruise_control(delta: float) -> void:
	if cruise_control_active:
		var speed_diff = cruise_control_target_speed - current_speed
		
		if speed_diff > 5.0:
			throttle_input = 0.8
		elif speed_diff < -5.0:
			_brake_input = 0.5
		else:
			throttle_input = 0.3

# ============================================================================
# AI BEHAVIOR
# ============================================================================
func update_ai_behavior(delta: float) -> void:
	var target_path = _get_ai_path()
	var path_distance = _get_path_distance(target_path)
	
	if path_distance < 50.0:
		_steering_input = _calculate_steering_correction(target_path)
		_brake_input = _calculate_braking_needed(path_distance)
		_throttle_input = _calculate_acceleration_needed(delta)

func _get_ai_path() -> Vector3:
	return _ai_next_waypoint

func _get_path_distance(target: Vector3) -> float:
	return global_position.distance_to(target)

func _calculate_steering_correction(target: Vector3) -> float:
	var direction = (target - global_position).normalized()
	var forward = transform.basis.z
	
	var angle = atan2(direction.x, direction.z)
	var steer_error = angle * 0.5
	
	return clampf(steer_error, -1.0, 1.0)

func _calculate_braking_needed(distance: float) -> float:
	if distance < 20.0:
		return 0.8
	elif distance < 40.0:
		return 0.4
	return 0.0

func _calculate_acceleration_needed(delta: float) -> float:
	var desired_accel = 1.0
	if current_speed < 100.0:
		return desired_accel
	return 0.3

# ============================================================================
# LAP TIMING
# ============================================================================
func start_lap_timing() -> void:
	_lap_time_accumulator = 0.0
	_lap_count += 1
	_checkpoints_passed.clear()

func update_lap_time(delta: float) -> void:
	_lap_time_accumulator += delta

func check_checkpoint(checkpoint_id: int) -> void:
	if checkpoint_id in _checkpoints_passed:
		return
	
	_checkpoints_passed.append(checkpoint_id)
	
	if _checkpoints_passed.size() == 4:
		lap_completed.emit(_lap_time_accumulator)
		_lap_time_accumulator = 0.0

func reset_lap_data() -> void:
	_lap_time_accumulator = 0.0
	_lap_count = 0
	_checkpoints_passed.clear()

# ============================================================================
# FUEL & ENGINE MANAGEMENT
# ============================================================================
func consume_fuel(delta: float) -> void:
	var fuel_consumption_rate = 0.5 + (_current_rpm / max_engine_rpm) * 0.3
	_fuel_level -= fuel_consumption_rate * delta
	
	if _fuel_level <= 0:
		_stop_vehicle()
		_fuel_level = 0.0

func update_engine_temperature(delta: float) -> void:
	var heat_generation = (_current_rpm / max_engine_rpm) * 10.0
	var cooling_rate = 2.0
	
	_engine_temperature += (heat_generation - cooling_rate) * delta
	_engine_temperature = clampf(_engine_temperature, 70.0, 130.0)

func update_oil_pressure() -> void:
	_oil_pressure = idle_rpm * 0.05 + (_current_rpm - idle_rpm) * 0.005
	_oil_pressure = clampf(_oil_pressure, 20.0, 60.0)

func check_engine_health() -> void:
	if _engine_temperature > 120.0:
		_engine_health -= 0.5 * EngineHeatDamageRate
	if _oil_pressure < 25.0:
		_engine_health -= 1.0 * OilPressureDamageRate

const EngineHeatDamageRate: float = 0.5
const OilPressureDamageRate: float = 1.0

# ============================================================================
# AUDIO FEEDBACK
# ============================================================================
func _update_audio_feedback(delta: float) -> void:
	if _engine_audio_node != null:
		var pitch = _current_rpm / max_engine_rpm
		_engine_audio_node.set_pitch_scale(pitch)
	
	if _tire_audio_node != null:
		var grip_level = 1.0 - skid_percentage
		_tire_audio_node.set_volume_db(grip_level * -10.0)

func _play_gear_sound(gear_number: String) -> void:
	if _gear_audio_node != null:
		_gear_audio_node.play()

func _play_drift_sound() -> void:
	if _tire_audio_node != null:
		_tire_audio_node.set_volume_db(-5.0)

# ============================================================================
# GAME EVENT HANDLERS
# ============================================================================
func _on_game_state_changed(new_state: GameState) -> void:
	match new_state:
		GameState.LOADING:
			_reset_vehicle()
		GameState.RACE_ACTIVE:
			_start_race()
		GameState.RACE_PAUSED:
			_pause_vehicle()
		GameState.RACE_FINISHED:
			_finish_race()

func _on_race_started(race_data: Dictionary) -> void:
	start_lap_timing()
	_fuel_level = race_data.get("starting_fuel", 100.0)
	_engine_health = 100.0

func _on_vehicle_spawned(vehicle: Node) -> void:
	if vehicle == self:
		_reset_vehicle()

func _reset_vehicle() -> void:
	velocity = Vector3.ZERO
	_current_rpm = idle_rpm
	current_gear = 0
	is_in_gear = false
	throttle_input = 0.0
	_brake_input = 0.0
	_steering_input = 0.0
	_handbrake_input = 0.0
	_fuel_level = 100.0
	_engine_health = 100.0
	_tire_temperatures = Vector4(80.0, 80.0, 80.0, 80.0)

func _pause_vehicle() -> void:
	set_process(false)
	set_physics_process(false)

func _finish_race() -> void:
	velocity = Vector3.ZERO
	_current_rpm = idle_rpm

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================
func get_current_tire_temperature(wheel_index: int) -> float:
	return _tire_temperatures[wheel_index]

func get_current_brake_temperature(wheel_index: int) -> float:
	return _brake_temperatures[wheel_index]

func get_suspension_compression(wheel_index: int) -> float:
	return _suspension_states[wheel_index]

func is_overheating() -> bool:
	return _engine_temperature > 115.0

func is_low_fuel() -> bool:
	return _fuel_level < 15.0

func is_critical_damage() -> bool:
	return _engine_health < 30.0 or _transmission_health < 30.0

func reset_tire_temperatures() -> void:
	_tire_temperatures = Vector4(80.0, 80.0, 80.0, 80.0)

func reset_brake_temperatures() -> void:
	_brake_temperatures = Vector4(100.0, 100.0, 100.0, 100.0)

func get_vehicle_stats() -> Dictionary:
	return {
		"speed": current_speed,
		"rpm": _current_rpm,
		"gear": current_gear,
		"fuel_level": _fuel_level,
		"engine_health": _engine_health,
		"transmission_health": _transmission_health,
		"engine_temperature": _engine_temperature,
		"oil_pressure": _oil_pressure,
		"is_drifting": is_drifting,
		"is_skidding": is_skidding,
		"lap_time": _lap_time_accumulator,
		"lap_count": _lap_count
	}

func respawn() -> void:
	global_position = Vector3(0.0, 5.0, 0.0)
	velocity = Vector3.ZERO
	_reset_vehicle()

func stop_vehicle() -> void:
	velocity = Vector3.ZERO
	_current_rpm = idle_rpm
	is_in_gear = false

func _stop_vehicle() -> void:
	stop_vehicle()

# ============================================================================
# DEBUG VISUALIZATION
# ============================================================================
func _draw_debug_visualization() -> void:
	if not debug_mode:
		return
	
	draw_line(global_position + Vector3(0, 1, 0), global_position + Vector3(0, 1, 5), Color.GREEN, 2.0)
	draw_line(global_position + Vector3(0, 1, 0), global_position + Vector3(0, 1, -5), Color.RED, 2.0)

# ============================================================================
# SAVE/LOAD SUPPORT
# ============================================================================
func save_state() -> Dictionary:
	return {
		"position": global_position,
		"rotation": rotation,
		"velocity": velocity,
		"current_rpm": _current_rpm,
		"current_gear": current_gear,
		"fuel_level": _fuel_level,
		"engine_health": _engine_health,
		"lap_time": _lap_time_accumulator,
		"lap_count": _lap_count
	}

func load_state(state_data: Dictionary) -> void:
	if state_data.has("position"):
		global_position = state_data["position"]
	if state_data.has("rotation"):
		rotation = state_data["rotation"]
	if state_data.has("velocity"):
		velocity = state_data["velocity"]
	if state_data.has("current_rpm"):
		_current_rpm = state_data["current_rpm"]
	if state_data.has("current_gear"):
		current_gear = state_data["current_gear"]
	if state_data.has("fuel_level"):
		_fuel_level = state_data["fuel_level"]
	if state_data.has("engine_health"):
		_engine_health = state_data["engine_health"]

# ============================================================================
# CLASS END
# ============================================================================
</File>