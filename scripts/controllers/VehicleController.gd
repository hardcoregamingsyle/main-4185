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
const AIR_RESISTANCE: float = 0.001

func _ready() -> void:
	_load_settings()
	_init_physics_body()
	_setup_powertrain_connection()
	_reset_vehicle()

func _load_settings() -> void:
	if Engine.has_singleton("PhysicsSettings"):
		_settings = Engine.get_singleton("PhysicsSettings")
	else:
		_settings = preload("res://scripts/core/PhysicsSettings.gd").new()
	max_rpm = _settings.default_engine_max_rpm
	idle_rpm = _settings.idle_engine_rpm

func _init_physics_body() -> void:
	if not is_instance_valid(_physics_body):
		var node = get_parent()
		if node and node is CharacterBody2D:
			_physics_body = node
		elif node and node is RigidBody2D:
			_physics_body = node as CharacterBody2D
		else:
			push_warning("VehicleController: No valid physics body found on parent")

func _setup_powertrain_connection() -> void:
	if powertrain:
		powertrain.rpm_changed.connect(_on_powertrain_rpm_changed)
		powertrain.gear_changed.connect(_on_powertrain_gear_changed)

func _reset_vehicle() -> void:
	current_speed = 0.0
	target_speed = 0.0
	acceleration = 0.0
	braking_force = 0.0
	steering_angle = 0.0
	total_distance_traveled = 0.0
	current_gear = 0
	rpm = idle_rpm
	nitro_available = true
	nitro_amount = 100.0
	nitro_cooldown = 0.0
	current_vehicle_state = VehicleState.IDLE

func _process(delta: float) -> void:
	if not _is_active():
		return
	
	_update_inputs(delta)
	_update_rpm(delta)
	_update_gears(delta)
	_update_movement(delta)
	_update_nitro(delta)
	_update_state(delta)

func _update_inputs(delta: float) -> void:
	if not InputManager:
		throttle_input = 0.0
		brake_input = 0.0
		steering_input = 0.0
		shift_up_request = false
		shift_down_request = false
		return
	
	var input_dict = InputManager.get_vehicle_inputs()
	throttle_input = clamp(input_dict.throttle, -1.0, 1.0)
	brake_input = clamp(input_dict.brake, 0.0, 1.0)
	steering_input = clamp(input_dict.steer, -1.0, 1.0)
	shift_up_request = input_dict.shift_up
	shift_down_request = input_dict.shift_down

func _update_rpm(delta: float) -> void:
	var target_rpm = idle_rpm
	
	if current_gear == 0:
		target_rpm = lerp(rpm, idle_rpm, delta * 5.0)
	elif throttle_input > 0.0:
		var rev_rate = _settings.default_rev_rate * throttle_input
		target_rpm = lerp(rpm, min(rpm + rev_rate * delta, max_rpm), delta * 10.0)
	else:
		target_rpm = lerp(rpm, idle_rpm, delta * 3.0)
	
	rpm = target_rpm
	emit_signal("rpm_changed", rpm, max_rpm)

func _update_gears(delta: float) -> void:
	if current_gear != 0 and throttle_input < 0.1 and brake_input > 0.5:
		_shift_to_neutral()
		return
	
	if shift_up_request and current_gear < 6:
		if rpm >= max_rpm * 0.95 or (current_gear == 0 and throttle_input > 0.5):
			_change_gear(current_gear + 1)
	elif shift_down_request and current_gear > 0:
		_change_gear(current_gear - 1)
	elif current_gear > 0 and current_speed < MIN_SPEED_FOR_GEAR_SHIFT and current_gear > 1:
		if throttle_input < 0.2:
			_change_gear(current_gear - 1)

func _change_gear(new_gear: int) -> void:
	if new_gear == current_gear:
		return
	
	var old_gear = current_gear
	current_gear = new_gear
	
	if new_gear == 0:
		rpm = lerp(rpm, idle_rpm, 0.3)
	else:
		var gear_ratio = GEAR_RATIOS[new_gear]
		var expected_rpm = abs(current_speed / gear_ratio * 0.1)
		rpm = lerp(rpm, max(idle_rpm, min(expected_rpm, max_rpm)), 0.2)
	
	emit_signal("gear_changed", old_gear, new_gear)

func _shift_to_neutral() -> void:
	if current_gear != 0:
		_change_gear(0)

func _update_movement(delta: float) -> void:
	if current_gear == 0:
		target_speed = lerp(target_speed, 0.0, delta * 3.0)
	else:
		var gear_ratio = GEAR_RATIOS[current_gear]
		var max_gear_speed = _settings.max_vehicle_speed / gear_ratio
		target_speed = lerp(target_speed, max_gear_speed * throttle_input, delta * 2.0)
	
	if brake_input > 0.0:
		var brake_deceleration = _settings.brake_force * brake_input
		target_speed = max(0.0, target_speed - brake_deceleration * delta)
	
	var speed_diff = target_speed - current_speed
	acceleration = speed_diff * delta * 5.0
	current_speed += acceleration
	
	if abs(current_speed) < 0.1:
		current_speed = 0.0
	
	total_distance_traveled += abs(acceleration * delta)
	emit_signal("speed_changed", current_speed, _get_max_possible_speed())

func _update_nitro(delta: float) -> void:
	if nitro_cooldown > 0.0:
		nitro_cooldown = max(0.0, nitro_cooldown - delta)
	
	if nitro_available and nitro_amount <= 0.0 and nitro_cooldown <= 0.0:
		nitro_available = false

func _use_nitro() -> void:
	if nitro_available and nitro_amount > 0.0:
		nitro_amount -= NITRO_CONSUMPTION_RATE
		nitro_cooldown = 5.0
		nitro_available = nitro_amount > 0.0
		emit_signal("nitro_used", NITRO_CONSUMPTION_RATE)

func _update_state(delta: float) -> void:
	if current_gear == 0 and abs(current_speed) < 1.0:
		current_vehicle_state = VehicleState.IDLE
	elif rpm > max_rpm * 0.9:
		current_vehicle_state = VehicleState.REVVING
	elif brake_input > 0.5:
		current_vehicle_state = VehicleState.BRAKING
	elif abs(steering_input) > 0.5 and abs(current_speed) > 50.0:
		current_vehicle_state = VehicleState.DRIFTING
	else:
		current_vehicle_state = VehicleState.RUNNING

func _get_max_possible_speed() -> float:
	if current_gear == 0:
		return 0.0
	var gear_ratio = GEAR_RATIOS[current_gear]
	return _settings.max_vehicle_speed / gear_ratio

func apply_throttle(force: float) -> void:
	throttle_input = clamp(force, 0.0, 1.0)

func apply_brake(force: float) -> void:
	brake_input = clamp(force, 0.0, 1.0)

func apply_steering(angle: float) -> void:
	steering_angle = clamp(angle, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE)

func set_gear(gear: int) -> void:
	if gear >= 0 and gear <= 6:
		_change_gear(gear)
	elif gear == -1 and current_gear != 0:
		current_gear = 0
		rpm = idle_rpm

func reset_nitro() -> void:
	nitro_amount = 100.0
	nitro_available = true
	nitro_cooldown = 0.0

func _on_powertrain_rpm_changed(new_rpm: float) -> void:
	rpm = new_rpm
	emit_signal("rpm_changed", rpm, max_rpm)

func _on_powertrain_gear_changed(old_gear: int, new_gear: int) -> void:
	_change_gear(new_gear)
	emit_signal("gear_changed", old_gear, new_gear)

func handle_collision(direction: Vector2) -> void:
	last_collision_time = Time.get_unix_time_from_system()
	collision_damping = 0.3
	current_speed *= (1.0 - collision_damping)
	emit_signal("collision_detected", direction)

func is_active() -> bool:
	return Engine.is_editor_hint() == false

func _is_active() -> bool:
	return Engine.is_editor_hint() == false and _physics_body != null

func _input(event: InputEvent) -> void:
	if event is InputEventKey and event.pressed:
		match event.keycode:
			KEY_SPACE:
				if event.modifier_mask & KEY_MODIFIER_SHIFT:
					_use_nitro()
				else:
					apply_brake(1.0)
			KEY_UP:
				apply_throttle(1.0)
			KEY_DOWN:
				apply_brake(1.0)
			KEY_LEFT:
				steering_input = -1.0
			KEY_RIGHT:
				steering_input = 1.0
			KEY_Z:
				shift_down_request = true
			KEY_X:
				shift_up_request = true

func _exit_tree() -> void:
	if powertrain:
		powertrain.rpm_changed.disconnect(_on_powertrain_rpm_changed)
		powertrain.gear_changed.disconnect(_on_powertrain_gear_changed)