extends Node
class_name VehicleController

## VehicleController - Core vehicle physics controller using PhysicsSettings constants
## Handles throttle, brake, steering inputs, wheel forces, and gear shifting logic
## Copyright 2026 Thalamus Racing Simulator Project

# Signals
signal engine_started
signal engine_stopped
signal gear_changed(old_gear: int, new_gear: int)
signal nitro_used(amount: float)
signal collision_detected(direction: Vector2)
signal speed_changed(current_speed: float, max_speed: float)
signal traction_loss(detected: bool)

# References
@onready var powertrain: Powertrain = $Powertrain if $Powertrain else null
@onready var chassis: Chassis = $Chassis if $Chassis else null
var _physics_body: CharacterBody2D = null
var _vehicle_mesh: Sprite2D = null

# State
enum VehicleState { IDLE, RUNNING, REVVING, BRAKING, COLLIDED, DRIFTING }
var current_vehicle_state: VehicleState = VehicleState.IDLE
var last_collision_time: float = 0.0
var collision_damping: float = 0.3

# Input state
var _throttle_input: float = 0.0
var _brake_input: float = 0.0
var _steering_input: float = 0.0
var _nitro_active: bool = false
var _reverse_active: bool = false

# Vehicle properties
var current_gear: int = 0
var engine_rpm: float = 0.0
var max_rpm: float = 8000.0
var idle_rpm: float = 800.0
var clutch_engaged: bool = true

# Speed tracking
var current_speed: float = 0.0
var max_speed: float = 0.0
var acceleration: float = 0.0
var deceleration: float = 0.0

# Physics references
var wheel_forces: Array[Vector2] = []
var steering_angle: float = 0.0

# Drift mechanics
var drift_factor: float = 0.0
var drift_threshold: float = 0.7
var grip_level: float = 1.0

# Nitro system
var nitro_amount: float = 100.0
var nitro_cooldown: float = 0.0
var nitro_multiplier: float = 1.5

# Configuration from PhysicsSettings
var _settings: PhysicsSettings = null
var _time_scale: float = 1.0

func _ready() -> void:
	_init_physics_references()
	_load_settings()
	_connect_signals()
	_reset_vehicle_state()

func _init_physics_references() -> void:
	if is_instance_valid(get_parent()):
		_physics_body = get_parent().find_child("CharacterBody2D", false, false)
		if _physics_body == null:
			_physics_body = find_child("CharacterBody2D", false, false)
		
		_vehicle_mesh = find_child("Sprite2D", false, false)
		if _vehicle_mesh == null:
			_vehicle_mesh = find_child("MeshInstance2D", false, false)
	
	if _physics_body == null:
		push_warning("VehicleController: No physics body found!")

func _load_settings() -> void:
	_settings = PhysicsSettings.new()
	if Engine.has_singleton("PhysicsSettings"):
		_settings = Engine.get_singleton("PhysicsSettings")
	
	max_rpm = _settings.max_engine_rpm if _settings != null else 8000.0
	idle_rpm = _settings.idle_rpm if _settings != null else 800.0
	
	var vehicle_mass = _settings.default_vehicle_mass if _settings != null else 1500.0
	max_speed = _calculate_max_speed(vehicle_mass)
	
	acceleration = _settings.base_acceleration if _settings != null else 5.0
	deceleration = _settings.base_deceleration if _settings != null else 8.0

func _connect_signals() -> void:
	if powertrain != null:
		powertrain.engine_started.connect(_on_engine_started)
		powertrain.engine_stopped.connect(_on_engine_stopped)
		powertrain.gear_changed.connect(_on_powertrain_gear_changed)

func _reset_vehicle_state() -> void:
	current_vehicle_state = VehicleState.IDLE
	current_gear = 0
	engine_rpm = idle_rpm
	clutch_engaged = true
	nitro_amount = 100.0
	nitro_cooldown = 0.0

func _process(delta: float) -> void:
	if delta > 0.1:
		return
	
	_time_scale = _settings.time_scale if _settings != null else 1.0
	
	_update_input_states(delta)
	_update_engine_state(delta)
	_update_vehicle_state(delta)
	_apply_forces(delta)
	_handle_drift(delta)
	_handle_nitro(delta)
	_check_traction()

func _update_input_states(delta: float) -> void:
	InputManager = get_tree().root.find_child("InputManager", false, false)
	if InputManager == null:
		InputManager = GameManager.InputManager
	
	if InputManager != null:
		var input_data = InputManager.get_vehicle_inputs()
		_throttle_input = clamp(input_data.throttle, -1.0, 1.0)
		_brake_input = clamp(input_data.brake, 0.0, 1.0)
		_steering_input = clamp(input_data.steer, -1.0, 1.0)
		_reverse_active = _throttle_input < 0
		
		var nitro_request = InputManager.get_nitro_request()
		if nitro_request.pressed and nitro_amount > 0 and nitro_cooldown <= 0:
			_activate_nitro(nitro_request.amount)

func _update_engine_state(delta: float) -> void:
	var target_rpm = _calculate_target_rpm()
	engine_rpm = lerp(engine_rpm, target_rpm, delta * 10.0)
	
	if engine_rpm >= max_rpm:
		engine_rpm = max_rpm * 0.95
		_handle_overrev()
	
	if engine_rpm <= idle_rpm and not _reverse_active:
		engine_rpm = idle_rpm
	
	if powertrain != null:
		powertrain.set_engine_rpm(engine_rpm)

func _calculate_target_rpm() -> float:
	var base_rpm = idle_rpm
	var gear_ratio = _get_gear_ratio(current_gear)
	var throttle_effect = abs(_throttle_input) * (max_rpm - idle_rpm)
	
	base_rpm += throttle_effect * gear_ratio
	
	if _brake_input > 0:
		base_rpm *= 0.7
	
	return base_rpm

func _get_gear_ratio(gear: int) -> float:
	var ratios = [0.0, 3.5, 2.8, 2.2, 1.8, 1.4, 1.1]
	if gear < ratios.size():
		return ratios[gear]
	return 1.0

func _update_vehicle_state(delta: float) -> void:
	if _brake_input > 0.8:
		current_vehicle_state = VehicleState.BRAKING
	elif _throttle_input > 0.5 and engine_rpm > 4000:
		current_vehicle_state = VehicleState.REVVING
	elif drift_factor > drift_threshold:
		current_vehicle_state = VehicleState.DRIFTING
	elif current_speed > 0.1:
		current_vehicle_state = VehicleState.RUNNING
	else:
		current_vehicle_state = VehicleState.IDLE
	
	if current_vehicle_state != VehicleState.IDLE and current_speed > 0.5:
		_calculate_current_speed()
		emit_signal("speed_changed", current_speed, max_speed)

func _calculate_current_speed() -> void:
	var velocity = Vector2.ZERO
	if _physics_body != null:
		velocity = _physics_body.velocity
	
	current_speed = velocity.length()

func _apply_forces(delta: float) -> void:
	if current_speed < 0.1 and _throttle_input == 0:
		return
	
	var total_force = _calculate_total_force()
	var direction = _get_movement_direction()
	
	var force_vector = direction * total_force
	
	if _physics_body != null:
		_physics_body.apply_central_impulse(force_vector * delta * 100.0)
	
	_steering_angle = _steering_input * _settings.max_steering_angle if _settings != null else _steering_input * 0.5
	_update_wheel_visuals()

func _calculate_total_force() -> float:
	var base_force = acceleration * _settings.default_vehicle_mass if _settings != null else acceleration * 1500.0
	
	if _throttle_input > 0:
		var gear_multiplier = _get_gear_ratio(current_gear)
		var rpm_factor = (engine_rpm - idle_rpm) / (max_rpm - idle_rpm)
		base_force *= gear_multiplier * rpm_factor
		
		if _nitro_active and nitro_amount > 0:
			base_force *= nitro_multiplier
			nitro_amount -= 0.5
	
	elif _brake_input > 0:
		base_force = -deceleration * _settings.default_vehicle_mass if _settings != null else -deceleration * 1500.0
		base_force *= _brake_input
	
	return base_force

func _get_movement_direction() -> Vector2:
	var angle = rotation
	return Vector2(cos(angle), sin(angle))

func _handle_drift(delta: float) -> void:
	if current_speed < 2.0:
		drift_factor = 0.0
		grip_level = 1.0
		return
	
	var lateral_velocity = _get_lateral_velocity()
	var turn_intensity = abs(_steering_input) * (_throttle_input + _brake_input)
	
	if turn_intensity > 0.5 and lateral_velocity > 1.0:
		drift_factor = min(lateral_velocity / 5.0, 1.0)
		grip_level = max(1.0 - drift_factor * 0.6, 0.3)
		
		if drift_factor > drift_threshold:
			current_vehicle_state = VehicleState.DRIFTING
		else:
			current_vehicle_state = VehicleState.RUNNING
	else:
		drift_factor = lerp(drift_factor, 0.0, delta * 2.0)
		grip_level = lerp(grip_level, 1.0, delta * 2.0)

func _get_lateral_velocity() -> float:
	if _physics_body == null:
		return 0.0
	
	var forward = Vector2(cos(rotation), sin(rotation))
	var velocity = _physics_body.velocity
	var lateral = velocity - forward * velocity.dot(forward)
	return lateral.length()

func _handle_nitro(delta: float) -> void:
	if nitro_cooldown > 0:
		nitro_cooldown -= delta
	
	if nitro_amount < 0:
		nitro_amount = 0.0

func _activate_nitro(amount: float) -> void:
	if nitro_amount <= 0:
		return
	
	nitro_active = true
	nitro_amount -= amount
	nitro_cooldown = 5.0
	nitro_active = false
	
	emit_signal("nitro_used", amount)
	_play_sound_effect("nitro_activate")

func _check_traction() -> void:
	var slip_ratio = _calculate_slip_ratio()
	var traction_lost = slip_ratio > _settings.max_slip_ratio if _settings != null else slip_ratio > 0.3
	
	if traction_lost != current_vehicle_state == VehicleState.DRIFTING:
		emit_signal("traction_loss", traction_lost)

func _calculate_slip_ratio() -> float:
	if current_speed < 0.1:
		return 0.0
	
	var drive_force = _calculate_total_force()
	var normal_force = _settings.default_vehicle_mass * _settings.gravity if _settings != null else 1500.0 * 9.81
	
	var friction_coefficient = grip_level * 1.2
	var max_friction = normal_force * friction_coefficient
	
	if abs(drive_force) > max_friction:
		return (abs(drive_force) - max_friction) / max_friction
	
	return 0.0

func _update_wheel_visuals() -> void:
	if _vehicle_mesh != null:
		_vehicle_mesh.rotation = _steering_angle * 0.3

func _handle_overrev() -> void:
	_on_engine_stopped()
	emit_signal("gear_changed", current_gear, current_gear - 1)
	current_gear = max(0, current_gear - 1)

func shift_up() -> void:
	if current_gear < 5:
		var old_gear = current_gear
		current_gear += 1
		emit_signal("gear_changed", old_gear, current_gear)
		_play_sound_effect("gear_shift")

func shift_down() -> void:
	if current_gear > 0:
		var old_gear = current_gear
		current_gear -= 1
		emit_signal("gear_changed", old_gear, current_gear)
		_play_sound_effect("gear_shift")

func activate_clutch() -> void:
	clutch_engaged = true
	engine_rpm = lerp(engine_rpm, idle_rpm, 0.1)

func deactivate_clutch() -> void:
	clutch_engaged = false

func reset_controls() -> void:
	_throttle_input = 0.0
	_brake_input = 0.0
	_steering_input = 0.0
	_nitro_active = false

func _on_engine_started() -> void:
	engine_rpm = idle_rpm
	current_vehicle_state = VehicleState.IDLE

func _on_engine_stopped() -> void:
	engine_rpm = 0.0
	current_vehicle_state = VehicleState.IDLE

func _on_powertrain_gear_changed(old_gear: int, new_gear: int) -> void:
	current_gear = new_gear

func _play_sound_effect(sound_name: String) -> void:
	if Engine.has_singleton("AudioManager"):
		var audio_manager = Engine.get_singleton("AudioManager")
		audio_manager.play_sound(sound_name)

func _set_gravity(new_gravity: float) -> void:
	_settings.gravity = new_gravity
	PhysicsServer2D.set_default_gravity(new_gravity)

func _set_physics_tick_rate(new_rate: int) -> void:
	_settings.physics_tick_rate = new_rate

func _set_max_substeps(new_substeps: int) -> void:
	_settings.max_substeps = new_substeps

func _set_time_scale(new_scale: float) -> void:
	_settings.time_scale = new_scale
	_time_scale = new_scale

func _set_default_vehicle_mass(new_mass: float) -> void:
	_settings.default_vehicle_mass = new_mass

func _calculate_max_speed(mass: float) -> float:
	var base_speed = 150.0
	var mass_factor = 1500.0 / mass
	return base_speed * sqrt(mass_factor)

func get_vehicle_stats() -> Dictionary:
	return {
		"current_speed": current_speed,
		"max_speed": max_speed,
		"engine_rpm": engine_rpm,
		"current_gear": current_gear,
		"nitro_amount": nitro_amount,
		"drift_factor": drift_factor,
		"grip_level": grip_level,
		"vehicle_state": current_vehicle_state
	}

func set_vehicle_config(config: Dictionary) -> void:
	if config.has("max_speed"):
		max_speed = config.max_speed
	if config.has("acceleration"):
		acceleration = config.acceleration
	if config.has("deceleration"):
		deceleration = config.deceleration
	if config.has("nitro_amount"):
		nitro_amount = config.nitro_amount
	if config.has("drift_threshold"):
		drift_threshold = config.drift_threshold

func respawn() -> void:
	_reset_vehicle_state()
	if _physics_body != null:
		_physics_body.velocity = Vector2.ZERO
	_physics_body.linear_damp = 0.9

</FILE "scripts/controllers/VehicleController.gd">