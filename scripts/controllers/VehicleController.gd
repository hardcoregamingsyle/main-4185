extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller using PhysicsSettings constants
## Handles throttle, brake, steering inputs, wheel forces, gear shifting logic
## Copyright 2026 Thalamus Racing Simulator Project

# ============================================================================
# SIGNALS
# ============================================================================
signal engine_started
signal engine_stopped
signal gear_changed(old_gear: int, new_gear: int)
signal nitro_used(amount: float)
signal collision_detected(damage_amount: float, impact_velocity: Vector3, collision_point: Vector3)
signal tire_skid(skid_speed: float, skid_angle: float)
signal lap_completed(lap_time: float, checkpoint_data: Dictionary)

# ============================================================================
# EXPORTED PHYSICS SETTINGS
# ============================================================================
@export_group("Vehicle Configuration")
@export var vehicle_mass: float = 1500.0: set = _set_vehicle_mass
@export var center_of_mass_offset: Vector3 = Vector3(0, 0.5, 0): set = _set_center_of_mass_offset
@export var chassis_inertia: Vector3 = Vector3(1.0, 1.0, 1.0): set = _set_chassis_inertia
@export var max_steering_angle: float = 30.0: set = _set_max_steering_angle
@export var steering_speed: float = 10.0: set = _set_steering_speed

@export_group("Engine & Powertrain")
@export var engine_torque_curve: Array[float] = [0.0, 0.3, 0.6, 0.9, 1.0]: set = _set_engine_torque_curve
@export var idle_rpm: float = 800.0: set = _set_idle_rpm
@export var redline_rpm: float = 7500.0: set = _set_redline_rpm
@export var max_rpm: float = 9000.0: set = _set_max_rpm
@export var torque_multiplier: float = 1.0: set = _set_torque_multiplier
@export var nitro_capacity: float = 100.0: set = _set_nitro_capacity
@export var nitro_consumption_rate: float = 10.0: set = _set_nitro_consumption_rate

@export_group("Transmission & Gears")
@export var transmission_type: String = "manual": set = _set_transmission_type
@export var gear_ratios: Array[float] = [3.5, 2.2, 1.5, 1.1, 0.8, 0.6]: set = _set_gear_ratios
@export var final_drive_ratio: float = 3.8: set = _set_final_drive_ratio
@export var shift_up_delay: float = 0.1: set = _set_shift_up_delay
@export var shift_down_delay: float = 0.05: set = _set_shift_down_delay
@export var clutch_disengage_threshold: float = 0.2: set = _set_clutch_disengage_threshold

@export_group("Tires & Suspension")
@export var tire_friction_coefficient: float = 1.2: set = _set_tire_friction_coefficient
@export var tire_slip_threshold: float = 0.15: set = _set_tire_slip_threshold
@export var suspension_stiffness: float = 50000.0: set = _set_suspension_stiffness
@export var suspension_damping: float = 5000.0: set = _set_suspension_damping
@export var suspension_travel: float = 0.15: set = _set_suspension_travel
@export var wheel_radius: float = 0.33: set = _set_wheel_radius
@export var track_width: float = 1.6: set = _set_track_width
@export var wheelbase: float = 2.7: set = _set_wheelbase

@export_group("Brakes")
@export var brake_force_per_wheel: float = 15000.0: set = _set_brake_force_per_wheel
@export var handbrake_force: float = 8000.0: set = _set_handbrake_force
@export var abs_enabled: bool = true: set = _set_abs_enabled
@export var brake_bias_front: float = 0.6: set = _set_brake_bias_front

@export_group("Aerodynamics")
@export var drag_coefficient: float = 0.32: set = _set_drag_coefficient
@export var frontal_area: float = 2.2: set = _set_frontal_area
@export var downforce_coefficient: float = 0.5: set = _set_downforce_coefficient
@export var wing_angle: float = 10.0: set = _set_wing_angle

# ============================================================================
# AUToload REFERENCES
# ============================================================================
var _physics_settings: PhysicsSettings
var _input_manager: InputManager
var _audio_manager: AudioManager
var _game_manager: GameManager

# ============================================================================
# INTERNAL STATE
# ============================================================================
var current_gear: int = 1
var target_gear: int = 1
var rpm: float = 0.0
var speed_kmh: float = 0.0
var wheel_rotation_angles: Array[float] = []
var wheel_angular_velocities: Array[float] = []
var throttle_input: float = 0.0
var brake_input: float = 0.0
var steering_input: float = 0.0
var clutch_pedal: float = 1.0
var handbrake_active: bool = false
var nitro_active: bool = false
var nitro_available: float = 0.0
var engine_running: bool = false
var gear_shift_cooldown: float = 0.0
var last_shift_direction: int = 0

var _current_steering_angle: float = 0.0
var _target_steering_angle: float = 0.0
var _suspension_compression: Array[float] = []
var _wheel_contact_points: Array[Vector3] = []
var _wheel_normal_forces: Array[float] = []
var _tire_skid_angles: Array[float] = []
var _lap_timer: float = 0.0
var _is_lapping: bool = false
var _lap_times: Array[float] = []
var _last_checkpoint_time: float = 0.0
var _checkpoint_count: int = 0

var _engine_force: float = 0.0
var _brake_force_total: float = 0.0
var _aerodynamic_downforce: float = 0.0
var _air_density: float = 1.225

# ============================================================================
# INITIALIZATION
# ============================================================================
func _ready() -> void:
	_init_autoloads()
	_init_physics_settings()
	_init_state()
	_connect_signals()
	_init_wheels()
	
	engine_started.emit()
	
	if Engine.is_editor_hint():
		return
	
	# Auto-start engine on spawn
	start_engine()

func _init_autoloads() -> void:
	_physics_settings = PhysicsSettings.new()
	_input_manager = InputManager if InputManager else null
	_audio_manager = AudioManager if AudioManager else null
	_game_manager = GameManager if GameManager else null

func _init_physics_settings() -> void:
	if _physics_settings:
		default_vehicle_mass = _physics_settings.default_vehicle_mass
		default_wheelfriction = _physics_settings.default_wheel_friction
		max_substeps = _physics_settings.max_substeps

func _init_state() -> void:
	current_gear = 1
	target_gear = 1
	rpm = idle_rpm
	speed_kmh = 0.0
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	clutch_pedal = 1.0
	nitro_available = nitro_capacity
	gear_shift_cooldown = 0.0
	last_shift_direction = 0
	_current_steering_angle = 0.0
	_target_steering_angle = 0.0
	engine_running = false

func _init_wheels() -> void:
	wheel_rotation_angles.resize(4)
	wheel_angular_velocities.resize(4)
	suspension_compression.resize(4)
	wheel_contact_points.resize(4)
	wheel_normal_forces.resize(4)
	tire_skid_angles.resize(4)
	
	for i in range(4):
		wheel_rotation_angles[i] = 0.0
		wheel_angular_velocities[i] = 0.0
		suspension_compression[i] = 0.0
		wheel_contact_points[i] = Vector3.ZERO
		wheel_normal_forces[i] = 0.0
		tire_skid_angles[i] = 0.0

func _connect_signals() -> void:
	if _input_manager:
		_input_manager.throttle_changed.connect(_on_throttle_changed)
		_input_manager.brake_changed.connect(_on_brake_changed)
		_input_manager.steering_changed.connect(_on_steering_changed)
		_input_manager.clutch_changed.connect(_on_clutch_changed)
		_input_manager.handbrake_toggled.connect(_on_handbrake_toggled)
		_input_manager.gear_shift_up.connect(_on_gear_shift_up)
		_input_manager.gear_shift_down.connect(_on_gear_shift_down)
		_input_manager.nitro_toggled.connect(_on_nitro_toggled)
		_input_manager.lap_start.connect(_on_lap_start)
		_input_manager.lap_stop.connect(_on_lap_stop)

# ============================================================================
# MAIN GAME LOOP
# ============================================================================
func _physics_process(delta: float) -> void:
	if Engine.is_editor_hint():
		return
	
	_update_input(delta)
	_update_engine(delta)
	_update_transmission(delta)
	_update_aerodynamics(delta)
	_update_tires(delta)
	_apply_forces(delta)
	_update_suspension(delta)
	_update_collision_detection(delta)
	_update_lap_timer(delta)
	_update_visuals(delta)

func _update_input(delta: float) -> void:
	if _input_manager:
		throttle_input = _input_manager.get_throttle()
		brake_input = _input_manager.get_brake()
		steering_input = _input_manager.get_steering()
		clutch_pedal = _input_manager.get_clutch()
		handbrake_active = _input_manager.get_handbrake()
		nitro_active = _input_manager.get_nitro()

func _update_engine(delta: float) -> void:
	if not engine_running:
		rpm = 0.0
		_engine_force = 0.0
		return
	
	var engine_load: float = min(abs(speed_kmh) / 200.0, 1.0)
	var torque_curve_value: float = _evaluate_torque_curve(rpm / max_rpm)
	_engine_force = engine_torque_curve.size() > 0 ? torque_curve_value : 1.0
	
	var effective_throttle: float = throttle_input * clutch_pedal
	if nitro_active and nitro_available > 0:
		effective_throttle *= 1.5
		nitro_available -= nitro_consumption_rate * delta
		if nitro_available <= 0:
			nitro_available = 0
			nitro_active = false
	
	var target_rpm: float = _calculate_target_rpm(effective_throttle, current_gear, speed_kmh)
	rpm = lerp(rpm, target_rpm, delta * 10.0)
	rpm = clamp(rpm, idle_rpm, max_rpm)
	
	if rpm >= redline_rpm and current_gear < gear_ratios.size():
		_auto_shift_up()
	elif rpm < idle_rpm and current_gear > 1:
		_auto_shift_down()

func _evaluate_torque_curve(normalized_rpm: float) -> float:
	if engine_torque_curve.is_empty():
		return 1.0
	
	var step_size: float = 1.0 / (engine_torque_curve.size() - 1)
	var index: int = int(normalized_rpm / step_size)
	index = clamp(index, 0, engine_torque_curve.size() - 2)
	
	var t: float = (normalized_rpm - index * step_size) / step_size
	return lerp(engine_torque_curve[index], engine_torque_curve[index + 1], t)

func _calculate_target_rpm(throttle: float, gear: int, current_speed: float) -> float:
	var gear_ratio: float = gear_ratios[gear - 1] if gear <= gear_ratios.size() else gear_ratios.back()
	var wheel_speed: float = current_speed * 1000.0 / 3600.0 / (PI * wheel_radius * 2)
	var target_rpm: float = wheel_speed * gear_ratio * final_drive_ratio * 60.0
	
	if throttle > 0.5:
		target_rpm = lerp(target_rpm, redline_rpm, throttle)
	else:
		target_rpm = lerp(target_rpm, idle_rpm, 1.0 - throttle)
	
	return target_rpm

func _auto_shift_up() -> void:
	if current_gear < gear_ratios.size():
		target_gear = current_gear + 1
		request_shift(current_gear, target_gear)

func _auto_shift_down() -> void:
	if current_gear > 1:
		target_gear = current_gear - 1
		request_shift(current_gear, target_gear)

func _update_transmission(delta: float) -> void:
	gear_shift_cooldown = max(gear_shift_cooldown - delta, 0.0)
	
	if gear_shift_cooldown > 0:
		return
	
	if target_gear != current_gear and clutch_pedal < clutch_disengage_threshold:
		var old_gear: int = current_gear
		current_gear = target_gear
		gear_shift_cooldown = shift_up_delay if target_gear > old_gear else shift_down_delay
		
		if _audio_manager:
			_audio_manager.play_sound("gear_shift")
		
		gear_changed.emit(old_gear, current_gear)

func request_shift(from_gear: int, to_gear: int) -> void:
	target_gear = to_gear
	if clutch_pedal < clutch_disengage_threshold or gear_shift_cooldown == 0:
		var old_gear: int = current_gear
		current_gear = to_gear
		gear_shift_cooldown = shift_up_delay if to_gear > old_gear else shift_down_delay
		
		if _audio_manager:
			_audio_manager.play_sound("gear_shift")
		
		gear_changed.emit(old_gear, current_gear)

func _update_aerodynamics(delta: float) -> void:
	var velocity_mps: float = global_position.distance_to(global_position + velocity.normalized() * 100.0) / 100.0
	if velocity.length() > 0:
		velocity_mps = velocity.length()
	
	var dynamic_pressure: float = 0.5 * _air_density * velocity_mps * velocity_mps
	_aerodynamic_downforce = dynamic_pressure * drag_coefficient * frontal_area
	_aerodynamic_downforce += dynamic_pressure * downforce_coefficient * frontal_area * (wing_angle / 10.0)

func _update_tires(delta: float) -> void:
	speed_kmh = velocity.length() * 3.6
	
	for i in range(4):
		var wheel_index: int = i
		var wheel_angular_velocity: float = _get_wheel_angular_velocity(wheel_index)
		wheel_angular_velocities[wheel_index] = wheel_angular_velocity
		
		var slip_ratio: float = _calculate_slip_ratio(wheel_index, wheel_angular_velocity)
		var slip_angle: float = _calculate_slip_angle(wheel_index)
		
		tire_skid_angles[wheel_index] = slip_angle
		
		if abs(slip_ratio) > tire_slip_threshold or abs(slip_angle) > 0.1:
			if _audio_manager:
				_audio_manager.play_sound("tire_skid")
			tire_skid.emit(wheel_angular_velocity, slip_angle)

func _get_wheel_angular_velocity(wheel_index: int) -> float:
	var wheel_speed: float = 0.0
	match wheel_index:
		0: # Front Left
			wheel_speed = (speed_kmh * 1000.0 / 3600.0) / wheel_radius
		1: # Front Right
			wheel_speed = (speed_kmh * 1000.0 / 3600.0) / wheel_radius
		2: # Rear Left
			wheel_speed = (speed_kmh * 1000.0 / 3600.0) / wheel_radius
		3: # Rear Right
			wheel_speed = (speed_kmh * 1000.0 / 3600.0) / wheel_radius
	return wheel_speed

func _calculate_slip_ratio(wheel_index: int, angular_velocity: float) -> float:
	var theoretical_speed: float = angular_velocity * wheel_radius
	var actual_speed: float = speed_kmh * 1000.0 / 3600.0
	
	if actual_speed == 0:
		return 0.0
	
	return (theoretical_speed - actual_speed) / max(actual_speed, 0.1)

func _calculate_slip_angle(wheel_index: int) -> float:
	var lateral_velocity: float = 0.0
	var forward_velocity: float = speed_kmh * 1000.0 / 3600.0
	
	if forward_velocity > 0:
		return atan2(abs(lateral_velocity), forward_velocity)
	
	return 0.0

func _apply_forces(delta: float) -> void:
	_brake_force_total = 0.0
	var total_downforce: float = _aerodynamic_downforce + vehicle_mass * _physics_settings.gravity
	
	if brake_input > 0:
		var brake_effective: float = brake_input * clutch_pedal
		var front_bias: float = brake_bias_front
		var rear_bias: float = 1.0 - brake_bias_front
		
		var front_brake_force: float = brake_force_per_wheel * front_bias * brake_effective
		var rear_brake_force: float = brake_force_per_wheel * rear_bias * brake_effective
		
		if handbrake_active:
			rear_brake_force = handbrake_force * brake_effective
		
		if abs_brake_force(front_brake_force) > 0:
			_brake_force_total += front_brake_force * 2.0
		if abs_brake_force(rear_brake_force) > 0:
			_brake_force_total += rear_brake_force * 2.0
	
	var drive_force: float = 0.0
	if engine_running and current_gear > 0:
		var gear_ratio: float = gear_ratios[current_gear - 1] if current_gear <= gear_ratios.size() else gear_ratios.back()
		var wheel_torque: float = _engine_force * torque_multiplier * gear_ratio * final_drive_ratio
		
		if throttle_input > 0:
			drive_force = wheel_torque / wheel_radius
			if nitro_active:
				drive_force *= 1.3
		
		if drive_force > 0 and speed_kmh > 1.0:
			drive_force *= 0.95
	
	var acceleration: float = (drive_force - _brake_force_total) / vehicle_mass
	velocity.x += acceleration * delta * 9.81
	
	if speed_kmh > 1.0:
		var air_resistance: float = 0.5 * _air_density * drag_coefficient * frontal_area * velocity.length() * velocity.length()
		var friction: float = vehicle_mass * _physics_settings.gravity * tire_friction_coefficient * 0.01
		var total_drag: float = air_resistance + friction
		
		var deceleration: float = total_drag / vehicle_mass
		velocity = velocity.normalized() * max(velocity.length() - deceleration * delta, 0.0)

func abs_brake_force(val: float) -> float:
	return abs(val)

func _update_suspension(delta: float) -> void:
	var gravity_vector: Vector3 = Vector3.UP * _physics_settings.gravity
	var local_gravity: Vector3 = global_transform.basis.xform(gravity_vector)
	
	for i in range(4):
		var contact_force: float = local_gravity.y * vehicle_mass * 0.25
		var compression: float = contact_force / suspension_stiffness
		compression = clamp(compression, 0.0, suspension_travel)
		suspension_compression[i] = compression
	
	var damping_factor: float = suspension_damping / vehicle_mass
	for i in range(4):
		var current_compression: float = suspension_compression[i]
		var target_compression: float = suspension_compression[i]
		suspension_compression[i] = lerp(current_compression, target_compression, delta * damping_factor)

func _update_collision_detection(delta: float) -> void:
	if move_and_slide():
		for i in range(get_slide_collision_count()):
			var collision: KinematicCollision3D = get_slide_collision(i)
			if collision.get_collider():
				var collider: Node = collision.get_collider()
				var impact_velocity: float = velocity.dot(collision.get_normal())
				var damage: float = abs(impact_velocity) * 0.5
				
				if damage > 5.0:
					collision_detected.emit(damage, collision.get_normal(), collision.get_position())
					
					if _audio_manager:
						_audio_manager.play_sound("collision", volume_scale * (damage / 10.0))
					
					if _game_manager:
						_game_manager.handle_vehicle_collision(self, damage)

func _update_lap_timer(delta: float) -> void:
	if _is_lapping:
		_lap_timer += delta
		if _lap_timer > 300.0:
			_is_lapping = false
			if _lap_times.size() > 0:
				_lap_times.append(_lap_timer)
				if _audio_manager:
					_audio_manager.play_sound("lap_complete")
				lap_completed.emit(_lap_timer, {})
			_lap_timer = 0.0

func _update_visuals(delta: float) -> void:
	_current_steering_angle = lerp(_current_steering_angle, _steering_input_to_angle(), delta * steering_speed)
	global_rotation.y = _current_steering_angle * PI / 180.0

func _steering_input_to_angle() -> float:
	return -steering_input * max_steering_angle

# ============================================================================
# INPUT HANDLERS
# ============================================================================
func _on_throttle_changed(value: float) -> void:
	throttle_input = value

func _on_brake_changed(value: float) -> void:
	brake_input = value

func _on_steering_changed(value: float) -> void:
	steering_input = value

func _on_clutch_changed(value: float) -> void:
	clutch_pedal = value

func _on_handbrake_toggled(active: bool) -> void:
	handbrake_active = active
	if _audio_manager:
		_audio_manager.play_sound("handbrake" if active else "handbrake_release")

func _on_gear_shift_up() -> void:
	if current_gear < gear_ratios.size():
		request_shift(current_gear, current_gear + 1)

func _on_gear_shift_down() -> void:
	if current_gear > 1:
		request_shift(current_gear, current_gear - 1)

func _on_nitro_toggled(active: bool) -> void:
	if active and nitro_available > 0:
		nitro_active = true
		nitro_used.emit(nitro_available)
		if _audio_manager:
			_audio_manager.play_sound("nitro_activate")
	else:
		nitro_active = false

func _on_lap_start() -> void:
	_is_lapping = true
	_lap_timer = 0.0
	_last_checkpoint_time = Time.get_ticks_msec()

func _on_lap_stop() -> void:
	if _is_lapping:
		_is_lapping = false
		_lap_times.append(_lap_timer)
		if _audio_manager:
			_audio_manager.play_sound("lap_complete")
		lap_completed.emit(_lap_timer, {})
		_lap_timer = 0.0

# ============================================================================
# ENGINE CONTROL
# ============================================================================
func start_engine() -> void:
	if not engine_running:
		engine_running = true
		rpm = idle_rpm
		engine_started.emit()
		if _audio_manager:
			_audio_manager.play_sound("engine_start")

func stop_engine() -> void:
	if engine_running:
		engine_running = false
		rpm = 0.0
		engine_stopped.emit()
		if _audio_manager:
			_audio_manager.play_sound("engine_stop")

func restart_engine() -> void:
	stop_engine()
	await get_tree().create_timer(0.5).timeout
	start_engine()

func toggle_engine() -> void:
	if engine_running:
		stop_engine()
	else:
		start_engine()

# ============================================================================
# GETTERS & SETTERS
# ============================================================================
func get_rpm() -> float:
	return rpm

func get_speed_kmh() -> float:
	return speed_kmh

func get_current_gear() -> int:
	return current_gear

func get_available_gears() -> int:
	return gear_ratios.size()

func get_nitro_available() -> float:
	return nitro_available

func get_max_nitro() -> float:
	return nitro_capacity

func is_engine_running() -> bool:
	return engine_running

func is_nitro_active() -> bool:
	return nitro_active

func get_wheel_contact_info() -> Dictionary:
	var info: Dictionary = {}
	for i in range(4):
		info["wheel_%d" % i] = {
			"contact_point": wheel_contact_points[i],
			"normal_force": wheel_normal_forces[i],
			"slip_angle": tire_skid_angles[i],
			"suspension_compression": suspension_compression[i]
		}
	return info

func get_vehicle_state() -> Dictionary:
	return {
		"rpm": rpm,
		"speed_kmh": speed_kmh,
		"current_gear": current_gear,
		"throttle": throttle_input,
		"brake": brake_input,
		"steering": steering_input,
		"clutch": clutch_pedal,
		"nitro_available": nitro_available,
		"engine_running": engine_running,
		"downforce": _aerodynamic_downforce,
		"lap_time": _lap_timer,
		"laps_completed": _lap_times.size()
	}

# ============================================================================
# SETTER METHODS
# ============================================================================
func _set_vehicle_mass(new_mass: float) -> void:
	vehicle_mass = new_mass
	mass = vehicle_mass

func _set_center_of_mass_offset(offset: Vector3) -> void:
	center_of_mass_offset = offset
	_global_center_of_mass = global_position + global_transform.basis * offset

func _set_chassis_inertia(inertia: Vector3) -> void:
	chassis_inertia = inertia

func _set_max_steering_angle(angle: float) -> void:
	max_steering_angle = angle

func _set_steering_speed(speed: float) -> void:
	steering_speed = speed

func _set_engine_torque_curve(curve: Array[float]) -> void:
	engine_torque_curve = curve

func _set_idle_rpm(rpm_val: float) -> void:
	idle_rpm = rpm_val

func _set_redline_rpm(rpm_val: float) -> void:
	redline_rpm = rpm_val

func _set_max_rpm(rpm_val: float) -> void:
	max_rpm = rpm_val

func _set_torque_multiplier(multiplier: float) -> void:
	torque_multiplier = multiplier

func _set_nitro_capacity(capacity: float) -> void:
	nitro_capacity = capacity
	nitro_available = capacity

func _set_nitro_consumption_rate(rate: float) -> void:
	nitro_consumption_rate = rate

func _set_transmission_type(type_str: String) -> void:
	transmission_type = type_str

func _set_gear_ratios(ratios: Array[float]) -> void:
	gear_ratios = ratios

func _set_final_drive_ratio(ratio: float) -> void:
	final_drive_ratio = ratio

func _set_shift_up_delay(delay: float) -> void:
	shift_up_delay = delay

func _set_shift_down_delay(delay: float) -> void:
	shift_down_delay = delay

func _set_clutch_disengage_threshold(threshold: float) -> void:
	clutch_disengage_threshold = threshold

func _set_tire_friction_coefficient(coeff: float) -> void:
	tire_friction_coefficient = coeff

func _set_tire_slip_threshold(threshold: float) -> void:
	tire_slip_threshold = threshold

func _set_suspension_stiffness(stiffness: float) -> void:
	suspension_stiffness = stiffness

func _set_suspension_damping(damping: float) -> void:
	suspension_damping = damping

func _set_suspension_travel(travel: float) -> void:
	suspension_travel = travel

func _set_wheel_radius(radius: float) -> void:
	wheel_radius = radius

func _set_track_width(width: float) -> void:
	track_width = width

func _set_wheelbase(length: float) -> void:
	wheelbase = length

func _set_brake_force_per_wheel(force: float) -> void:
	brake_force_per_wheel = force

func _set_handbrake_force(force: float) -> void:
	handbrake_force = force

func _set_abs_enabled(enabled: bool) -> void:
	abs_enabled = enabled

func _set_brake_bias_front(bias: float) -> void:
	brake_bias_front = bias

func _set_drag_coefficient(coeff: float) -> void:
	drag_coefficient = coeff

func _set_frontal_area(area: float) -> void:
	frontal_area = area

func _set_downforce_coefficient(coeff: float) -> void:
	downforce_coefficient = coeff

func _set_wing_angle(angle: float) -> void:
	wing_angle = angle

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================
func reset_vehicle() -> void:
	position = Vector3.ZERO
	velocity = Vector3.ZERO
	rotation = Vector3.ZERO
	rpm = idle_rpm
	speed_kmh = 0.0
	current_gear = 1
	target_gear = 1
	nitro_available = nitro_capacity
	engine_running = false
	_lap_timer = 0.0
	_lap_times.clear()
	_init_wheels()

func apply_damage(damage_amount: float) -> void:
	if damage_amount <= 0:
		return
	
	vehicle_mass = max(vehicle_mass * (1.0 - damage_amount * 0.01), 500.0)
	
	if _audio_manager:
		_audio_manager.play_sound("damage", volume_scale * (damage_amount / 10.0))
	
	if _game_manager:
		_game_manager.update_vehicle_health(self, vehicle_mass)

func heal_vehicle(heal_amount: float) -> void:
	vehicle_mass = min(vehicle_mass + heal_amount, 1500.0)
	nitro_available = min(nitro_available + heal_amount * 0.5, nitro_capacity)

func get_distance_traveled() -> float:
	var traveled: float = 0.0
	for i in range(_lap_times.size()):
		traveled += _lap_times[i] * speed_kmh / 3.6
	return traveled

func save_lap_data() -> Dictionary:
	return {
		"lap_times": _lap_times.duplicate(),
		"best_lap": _lap_times.min() if _lap_times.size() > 0 else 0.0,
		"total_laps": _lap_times.size(),
		"timestamp": Time.get_datetime_dict_from_system()
	}

func load_lap_data(data: Dictionary) -> void:
	if data.has("lap_times"):
		_lap_times = data["lap_times"].duplicate()
	if data.has("best_lap"):
		pass
	if data.has("total_laps"):
		pass

func clear_lap_data() -> void:
	_lap_times.clear()
	_lap_timer = 0.0
	_is_lapping = false

func get_debug_info() -> Dictionary:
	return {
		"position": position,
		"velocity": velocity,
		"rpm": rpm,
		"speed_kmh": speed_kmh,
		"current_gear": current_gear,
		"throttle": throttle_input,
		"brake": brake_input,
		"steering": steering_input,
		"clutch": clutch_pedal,
		"nitro": nitro_available,
		"engine": engine_running,
		"downforce": _aerodynamic_downforce,
		"brake_force": _brake_force_total,
		"drive_force": _engine_force,
		"lap_timer": _lap_timer,
		"laps": _lap_times.size()
	}

func debug_print_state() -> void:
	print("=== VEHICLE DEBUG ===")
	print("Position: %.2f, %.2f, %.2f" % [position.x, position.y, position.z])
	print("Velocity: %.2f, %.2f, %.2f" % [velocity.x, velocity.y, velocity.z])
	print("RPM: %.0f | Speed: %.1f km/h | Gear: %d/%d" % [rpm, speed_kmh, current_gear, gear_ratios.size()])
	print("Throttle: %.2f | Brake: %.2f | Steering: %.2f | Clutch: %.2f" % [throttle_input, brake_input, steering_input, clutch_pedal])
	print("Nitro: %.1f/%.1f | Engine: %s" % [nitro_available, nitro_capacity, "ON" if engine_running else "OFF"])
	print("Downforce: %.1f N | Brake Force: %.1f N | Drive Force: %.1f N" % [_aerodynamic_downforce, _brake_force_total, _engine_force])
	print("Lap Timer: %.1f s | Total Laps: %d" % [_lap_timer, _lap_times.size()])
	print("===================")

func set_debug_mode(enabled: bool) -> void:
	PhysicsSettings.debug_mode = enabled

# ============================================================================
# DESTRUCTOR
# ============================================================================
func _exit_tree() -> void:
	if engine_running:
		stop_engine()
</script>