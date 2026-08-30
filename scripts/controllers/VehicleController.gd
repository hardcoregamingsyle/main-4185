extends Node2D
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
signal rpm_changed(current_rpm: float, max_rpm: float)
signal vehicle_moved(distance: float)

# References
@onready var powertrain: Powertrain = $Powertrain if get_node_or_null("Powertrain") else null
@onready var chassis: Chassis = $Chassis if get_node_or_null("Chassis") else null
var _physics_body: CharacterBody2D = null

# State
enum VehicleState { IDLE, RUNNING, REVVING, BRAKING, COLLIDED, DRIFTING }
var current_vehicle_state: VehicleState = VehicleState.IDLE
var last_collision_time: float = 0.0
var collision_damping: float = 0.3

# Speed and movement
var current_speed: float = 0.0
var target_speed: float = 0.0
var acceleration: float = 0.0
var braking_force: float = 0.0
var steering_angle: float = 0.0
var total_distance_traveled: float = 0.0

# Gear management
var current_gear: int = 0  # 0 = neutral, 1-6 = forward gears, -1 = reverse
var rpm: float = 0.0
var max_rpm: float = 8000.0
var idle_rpm: float = 800.0

# Nitrous system
var nitro_available: bool = true
var nitro_amount: float = 100.0
var nitro_cooldown: float = 0.0
const NITRO_BONUS: float = 1.5
const NITRO_CONSUMPTION_RATE: float = 50.0

# Input handling
var throttle_input: float = 0.0
var brake_input: float = 0.0
var steering_input: float = 0.0
var shift_up_request: bool = false
var shift_down_request: bool = false

# Physics settings reference
var _settings: PhysicsSettings = null

# Constants for physics calculations
const MIN_SPEED_FOR_GEAR_SHIFT: float = 50.0
const MAX_STEERING_ANGLE: float = PI / 4  # 45 degrees
const GEAR_RATIOS: Array[float] = [0.0, 3.8, 2.5, 1.7, 1.3, 1.0, 0.8]
const REVERSE_GEAR_RATIO: float = -4.0
const WHEELBASE: float = 2.5
const TRACK_WIDTH: float = 1.5
const FRICTION_COEFFICIENT: float = 0.95
const AIR_RESISTANCE_FACTOR: float = 0.01
const GRIP_COEFFICIENT: float = 0.9
const DRIFT_GRIP_REDUCTION: float = 0.6
const MAX_ACCELERATION: float = 15.0
const MAX_DECELERATION: float = 20.0
const GEAR_CHANGE_TIME: float = 0.2
const ENGINE_ROTATIONAL_INERTIA: float = 0.5
const TORQUE_MULTIPLIER: float = 350.0

# Internal state tracking
var _is_engine_running: bool = false
var _last_position: Vector2 = Vector2.ZERO
var _gear_change_timer: float = 0.0
var _nitro_timer: float = 0.0
var _accumulated_distance: float = 0.0

func _ready() -> void:
	_init_references()
	_load_physics_settings()
	_connect_signals()
	_reset_vehicle_state()

func _init_references() -> void:
	if not Engine.has_singleton("GameManager"):
		Engine.register_singleton("GameManager", GameManager)
	if not Engine.has_singleton("PhysicsSettings"):
		Engine.register_singleton("PhysicsSettings", PhysicsSettings.new())
	
	if get_node_or_null("CharacterBody2D"):
		_physics_body = $CharacterBody2D as CharacterBody2D
	elif get_node_or_null("RigidBody2D"):
		_physics_body = $RigidBody2D as RigidBody2D
	
	if powertrain == null and has_node("Powertrain"):
		powertrain = $Powertrain as Powertrain
	
	if chassis == null and has_node("Chassis"):
		chassis = $Chassis as Chassis

func _load_physics_settings() -> void:
	if Engine.has_singleton("PhysicsSettings"):
		_settings = PhysicsSettings.get_singleton()
	else:
		var default_settings := PhysicsSettings.new()
		default_settings.gravity = 9.81
		default_settings.physics_tick_rate = 120
		default_settings.max_substeps = 4
		_settings = default_settings
	
	max_rpm = _settings.default_max_rpm if _settings.default_max_rpm > 0 else 8000.0
	idle_rpm = _settings.default_idle_rpm if _settings.default_idle_rpm > 0 else 800.0

func _connect_signals() -> void:
	if Engine.has_singleton("GameManager"):
		GameManager.race_started.connect(_on_race_started)
		GameManager.race_ended.connect(_on_race_ended)

func _physics_process(delta: float) -> void:
	if not _is_active():
		return
	
	_update_inputs(delta)
	_update_physics(delta)
	_update_gear_system(delta)
	_update_nitrous(delta)
	_update_visual_feedback(delta)

func _update_inputs(delta: float) -> void:
	if Engine.has_singleton("InputManager"):
		throttle_input = InputManager.get_axis("throttle", "brake") * 0.5 + 0.5
		brake_input = InputManager.get_axis("brake", "throttle") * 0.5
		steering_input = InputManager.get_axis("steer_left", "steer_right")
		
		if InputManager.is_action_just_pressed("shift_up"):
			shift_up_request = true
		if InputManager.is_action_just_pressed("shift_down"):
			shift_down_request = true
		
		if InputManager.is_action_just_pressed("toggle_engine"):
			_toggle_engine()
		if InputManager.is_action_just_pressed("activate_nitro"):
			_activate_nitro()
	else:
		# Fallback manual input handling
		var input_throttle = Input.get_axis("ui_up", "ui_down")
		var input_brake = Input.get_axis("ui_right", "ui_left")
		var input_steering = Input.get_axis("ui_left", "ui_right")
		
		throttle_input = clamp(input_throttle, 0.0, 1.0)
		brake_input = abs(input_brake)
		steering_input = input_steering

func _update_physics(delta: float) -> void:
	if _physics_body == null:
		return
	
	var direction := Vector2(1.0, 0.0).rotated(rotation)
	var velocity := _physics_body.velocity
	
	current_speed = velocity.length()
	target_speed = _calculate_target_speed()
	
	var speed_difference := target_speed - current_speed
	
	# Apply acceleration/deceleration
	if current_vehicle_state != VehicleState.BRAKING:
		acceleration = speed_difference * _settings.default_acceleration_factor
		acceleration = clamp(acceleration, -MAX_DECELERATION, MAX_ACCELERATION)
	else:
		acceleration = -MAX_DECELERATION * brake_input
	
	# Apply friction and air resistance
	var friction_loss := current_speed * FRICTION_COEFFICIENT * delta
	var air_resistance := current_speed * current_speed * AIR_RESISTANCE_FACTOR * delta
	
	velocity.x -= friction_loss * direction.x + air_resistance * direction.x
	velocity.y -= friction_loss * direction.y + air_resistance * direction.y
	
	# Apply driving force
	if _is_engine_running and current_gear != 0:
		var gear_ratio := _get_current_gear_ratio()
		var torque := _calculate_torque()
		var drive_force := torque * gear_ratio * WHEELBASE * GRAVITY
		
		velocity.x += drive_force * direction.x * delta
		velocity.y += drive_force * direction.y * delta
	
	# Update velocity
	_physics_body.velocity = velocity
	
	# Track distance traveled
	var position_delta := global_position.distance_to(_last_position)
	_accumulated_distance += position_delta
	total_distance_traveled += position_delta
	
	_last_position = global_position
	
	# Emit signals
	rpm_changed.emit(rpm, max_rpm)
	speed_changed.emit(current_speed, target_speed)
	if position_delta > 0.1:
		vehicle_moved.emit(position_delta)

func _calculate_target_speed() -> float:
	if current_gear == 0:
		return 0.0
	
	var gear_ratio := _get_current_gear_ratio()
	var max_gear_speed := _settings.default_max_vehicle_speed / gear_ratio
	var throttle_effect := throttle_input * 0.8
	
	return current_speed + (max_gear_speed * throttle_effect - current_speed) * 0.1

func _calculate_torque() -> float:
	if current_gear == 0 or not _is_engine_running:
		return 0.0
	
	var gear_index := current_gear if current_gear > 0 else abs(current_gear)
	var base_torque := _settings.default_engine_torque if _settings.default_engine_torque > 0 else 350.0
	
	var rpm_factor := (rpm - idle_rpm) / (max_rpm - idle_rpm)
	var throttle_factor := throttle_input
	
	return base_torque * rpm_factor * throttle_factor * TORQUE_MULTIPLIER

func _get_current_gear_ratio() -> float:
	if current_gear == 0:
		return 0.0
	elif current_gear < 0:
		return REVERSE_GEAR_RATIO
	else:
		var index := current_gear if current_gear < GEAR_RATIOS.size() else GEAR_RATIOS.size() - 1
		return GEAR_RATIOS[index]

func _update_gear_system(delta: float) -> void:
	if _gear_change_timer > 0:
		_gear_change_timer -= delta
		return
	
	# Automatic upshifts based on RPM
	if current_gear > 0 and current_gear < 6:
		if rpm >= max_rpm * 0.95 and throttle_input > 0.3:
			_shift_gear(current_gear + 1)
	
	# Automatic downshifts based on RPM and speed
	if current_gear > 1:
		if rpm <= idle_rpm * 1.5 and throttle_input < 0.2:
			_shift_gear(current_gear - 1)
	
	# Manual gear shifts
	if shift_up_request and current_gear < 6:
		_shift_gear(current_gear + 1)
		shift_up_request = false
	elif shift_down_request and current_gear > 0:
		_shift_gear(current_gear - 1)
		shift_down_request = false
	
	# Neutral handling
	if current_gear == 0:
		rpm = lerp(rpm, idle_rpm, delta * 10.0)
		if not _is_engine_running:
			rpm = lerp(rpm, 0.0, delta * 10.0)

func _shift_gear(new_gear: int) -> void:
	var old_gear := current_gear
	current_gear = new_gear
	_gear_change_timer = GEAR_CHANGE_TIME
	
	gear_changed.emit(old_gear, new_gear)
	
	# Handle reverse gear
	if new_gear < 0:
		_physics_body.velocity *= -1.0
	
	# Update powertrain
	if powertrain:
		powertrain.current_gear = new_gear

func _update_nitrous(delta: float) -> void:
	if nitro_cooldown > 0:
		nitro_cooldown -= delta
		return
	
	if nitro_amount > 0 and nitro_available and throttle_input > 0.8:
		_activate_nitro()

func _activate_nitro() -> void:
	if nitro_amount <= 0 or nitro_cooldown > 0:
		return
	
	nitro_available = false
	var bonus_multiplier := NITRO_BONUS * (nitro_amount / 100.0)
	target_speed *= bonus_multiplier
	nitro_amount -= NITRO_CONSUMPTION_RATE
	
	if nitro_amount <= 0:
		nitro_amount = 0.0
		nitro_cooldown = 5.0
	
	nitro_used.emit(nitro_amount)
	_power_on_light(true)

func _update_visual_feedback(delta: float) -> void:
	# Update chassis visuals based on state
	if chassis:
		chassis.set_state(current_vehicle_state)
		chassis.set_steering_angle(steering_angle)
		chassis.set_speed_indicator(current_speed)
	
	# Update powertrain visuals
	if powertrain:
		powertrain.update_visuals(rpm, current_gear, throttle_input)

func _toggle_engine() -> void:
	if _is_engine_running:
		_stop_engine()
	else:
		_start_engine()

func _start_engine() -> void:
	if _is_engine_running:
		return
	
	_is_engine_running = true
	rpm = idle_rpm
	current_vehicle_state = VehicleState.RUNNING
	
	if powertrain:
		powertrain.engine_start()
	
	engine_started.emit()

func _stop_engine() -> void:
	if not _is_engine_running:
		return
	
	_is_engine_running = false
	rpm = 0.0
	current_vehicle_state = VehicleState.IDLE
	
	if powertrain:
		powertrain.engine_stop()
	
	engine_stopped.emit()

func _reset_vehicle_state() -> void:
	current_speed = 0.0
	target_speed = 0.0
	acceleration = 0.0
	braking_force = 0.0
	steering_angle = 0.0
	total_distance_traveled = 0.0
	current_gear = 0
	rpm = idle_rpm
	last_collision_time = 0.0
	_accumulated_distance = 0.0
	_last_position = global_position
	nitro_amount = 100.0
	nitro_cooldown = 0.0
	current_vehicle_state = VehicleState.IDLE

func _on_race_started(race_data: Dictionary) -> void:
	_reset_vehicle_state()
	_start_engine()
	nitro_amount = 100.0
	current_gear = 1

func _on_race_ended(results: Dictionary) -> void:
	_stop_engine()
	current_vehicle_state = VehicleState.IDLE

func _handle_collision(direction: Vector2) -> void:
	current_vehicle_state = VehicleState.COLLIDED
	last_collision_time = Time.get_unix_time_from_system()
	collision_damping = 0.5
	
	_physics_body.velocity *= 0.3
	
	collision_detected.emit(direction)
	
	await get_tree().create_timer(0.5).timeout
	current_vehicle_state = VehicleState.RUNNING

func _set_gravity(value: float) -> void:
	_settings.gravity = value
	if _physics_body:
		_physics_body.gravity_scale = value / 9.81

func _set_physics_tick_rate(value: int) -> void:
	_settings.physics_tick_rate = value

func _set_default_vehicle_mass(value: float) -> void:
	_settings.default_vehicle_mass = value
	if _physics_body:
		_physics_body.mass = value

func _set_time_scale(value: float) -> void:
	_settings.time_scale = value

func _is_active() -> bool:
	if Engine.has_singleton("GameManager"):
		return GameManager.current_state == GameManager.GameState.RACE_ACTIVE
	return false

func _power_on_light(active: bool) -> void:
	# Placeholder for visual feedback
	pass

func get_total_distance() -> float:
	return total_distance_traveled

func get_current_gear_info() -> Dictionary:
	return {
		"current_gear": current_gear,
		"rpm": rpm,
		"max_rpm": max_rpm,
		"speed": current_speed,
		"target_speed": target_speed
	}

func reset_all_counters() -> void:
	total_distance_traveled = 0.0
	_accumulated_distance = 0.0
	last_collision_time = 0.0
	_gear_change_timer = 0.0
	nitro_amount = 100.0
	nitro_cooldown = 0.0
	shift_up_request = false
	shift_down_request = false