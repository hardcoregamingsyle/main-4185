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
const AIR_RESISTANCE: float = 0.005

# Engine torque curve (RPM ratio -> Torque multiplier)
var _torque_curve: Dictionary = {
	0.0: 0.0,
	0.1: 0.2,
	0.2: 0.4,
	0.3: 0.6,
	0.4: 0.8,
	0.5: 0.95,
	0.6: 1.0,
	0.7: 1.0,
	0.8: 0.95,
	0.9: 0.8,
	1.0: 0.6
}

# Wheel force configuration
const FRONT_WHEEL_FORCE: float = 100.0
const REAR_WHEEL_FORCE: float = 150.0
const BRAKE_FORCE_PER_WHEEL: float = 800.0
const STEERING_SENSITIVITY: float = 0.5

func _ready() -> void:
	_init_physics_body()
	_load_settings()
	_connect_signals()
	_reset_vehicle()

func _init_physics_body() -> void:
	if has_node("PhysicsBody"):
		_physics_body = get_node("PhysicsBody") as CharacterBody2D
	elif has_node("CollisionShape2D"):
		var parent = get_parent()
		if parent is CharacterBody2D:
			_physics_body = parent
	else:
		push_warning("VehicleController: No physics body found")

func _load_settings() -> void:
	if GameManager and GameManager.has_signal("game_state_changed"):
		GameManager.game_state_changed.connect(_on_game_state_changed)
	
	if Engine.has_singleton("PhysicsSettings"):
		_settings = Engine.get_singleton("PhysicsSettings")
	elif ResourceLoader.load("res://scripts/core/PhysicsSettings.gd") != null:
		_settings = preload("res://scripts/core/PhysicsSettings.gd").new()
	else:
		push_warning("VehicleController: Could not load PhysicsSettings")

func _connect_signals() -> void:
	if powertrain:
		powertrain.engine_started.connect(_on_engine_started)
		powertrain.engine_stopped.connect(_on_engine_stopped)
		powertrain.torque_applied.connect(_on_torque_applied)

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
	_process_input(delta)
	_update_engine(delta)
	_update_physics(delta)
	_check_gear_changes()
	_handle_nitro(delta)

func _input(event: InputEvent) -> void:
	if event.is_action_pressed("throttle"):
		throttle_input = 1.0
	if event.is_action_released("throttle"):
		throttle_input = 0.0
	if event.is_action_pressed("brake"):
		brake_input = 1.0
	if event.is_action_released("brake"):
		brake_input = 0.0
	if event.is_action_pressed("steer_left"):
		steering_input = -1.0
	if event.is_action_released("steer_left"):
		steering_input = 0.0
	if event.is_action_pressed("steer_right"):
		steering_input = 1.0
	if event.is_action_released("steer_right"):
		steering_input = 0.0
	if event.is_action_pressed("shift_up"):
		shift_up_request = true
	if event.is_action_pressed("shift_down"):
		shift_down_request = true

func _process_input(delta: float) -> void:
	# Smooth input transition
	throttle_input = lerp(throttle_input, throttle_input, delta * 10.0)
	brake_input = lerp(brake_input, brake_input, delta * 10.0)
	steering_input = lerp(steering_input, steering_input, delta * 10.0)

func _update_engine(delta: float) -> void:
	# Calculate RPM based on current gear and speed
	var rpm_ratio = calculate_rpm_ratio()
	rpm = lerp(rpm, _get_target_rpm(rpm_ratio), delta * 20.0)
	
	# Apply throttle effect on RPM
	if throttle_input > 0 and current_gear != 0:
		rpm = min(rpm + delta * throttle_input * 500.0, max_rpm)
	elif brake_input > 0 and current_gear != 0:
		rpm = max(rpm - delta * brake_input * 300.0, idle_rpm)
	
	# Update signal
	emit_signal("rpm_changed", rpm, max_rpm)

func calculate_rpm_ratio() -> float:
	if current_gear == 0 or current_speed <= 0:
		return 0.0
	
	var speed_ratio = current_speed / MIN_SPEED_FOR_GEAR_SHIFT
	speed_ratio = clamp(speed_ratio, 0.0, 1.0)
	
	var gear_index = abs(current_gear)
	if gear_index < GEAR_RATIOS.size():
		var gear_ratio = GEAR_RATIOS[gear_index]
		return speed_ratio * gear_ratio
	return 0.0

func _get_target_rpm(rpm_ratio: float) -> float:
	var base_rpm = idle_rpm + (max_rpm - idle_rpm) * rpm_ratio
	return base_rpm

func _update_physics(delta: float) -> void:
	if _physics_body == null:
		return
	
	# Calculate acceleration based on gear and throttle
	var gear_multiplier = _get_gear_multiplier()
	var torque = _calculate_torque() * gear_multiplier
	
	# Apply air resistance
	var air_resistance_force = current_speed * current_speed * AIR_RESISTANCE
	
	# Calculate net force
	var net_force = torque - air_resistance_force
	
	# Apply friction
	var friction_force = current_speed * FRICTION_COEFFICIENT
	
	# Update speed
	acceleration = net_force - friction_force
	current_speed += acceleration * delta
	
	# Clamp speed
	current_speed = clamp(current_speed, -MAX_SPEED.magnitude(), MAX_SPEED.magnitude())
	
	# Update distance traveled
	total_distance_traveled += abs(current_speed) * delta
	
	# Emit signals
	emit_signal("speed_changed", current_speed, MAX_SPEED.magnitude())
	emit_signal("vehicle_moved", total_distance_traveled)

func _calculate_torque() -> float:
	var rpm_ratio = rpm / max_rpm
	var torque_multiplier = _get_torque_at_rpm(rpm_ratio)
	
	var base_torque = 300.0  # Base engine torque in Nm
	return base_torque * torque_multiplier

func _get_torque_at_rpm(rpm_ratio: float) -> float:
	for key in _torque_curve:
		if key <= rpm_ratio:
			continue
		var prev_key = key - 0.1
		var next_val = _torque_curve[key]
		var prev_val = _torque_curve[prev_key]
		var t = (rpm_ratio - prev_key) / 0.1
		return prev_val + (next_val - prev_val) * t
	return _torque_curve.values()[_torque_curve.keys().max()]

func _get_gear_multiplier() -> float:
	if current_gear == 0:
		return 0.0
	
	var gear_index = abs(current_gear)
	if gear_index < GEAR_RATIOS.size():
		return GEAR_RATIOS[gear_index]
	return 1.0

func _check_gear_changes() -> void:
	if shift_up_request:
		shift_gear(true)
		shift_up_request = false
	if shift_down_request:
		shift_gear(false)
		shift_down_request = false
	
	# Automatic gear shifting based on RPM
	if current_gear > 0 and current_gear < 6:
		if rpm >= max_rpm * 0.9:
			shift_gear(true)
		elif rpm <= idle_rpm * 1.5:
			shift_gear(false)

func shift_gear(up: bool) -> void:
	var old_gear = current_gear
	var new_gear = old_gear
	
	if up:
		new_gear = min(old_gear + 1, 6)
	else:
		new_gear = max(old_gear - 1, -1)
	
	if new_gear != old_gear:
		current_gear = new_gear
		emit_signal("gear_changed", old_gear, new_gear)
		
		# Play gear shift sound
		if AudioManager:
			AudioManager.play_sfx("gear_shift")

func _handle_nitro(delta: float) -> void:
	if nitro_cooldown > 0:
		nitro_cooldown -= delta
		if nitro_cooldown <= 0:
			nitro_cooldown = 0.0
			nitro_available = true
	
	if throttle_input > 0.8 and nitro_available and nitro_amount > 0:
		nitro_amount -= delta * NITRO_CONSUMPTION_RATE
		if nitro_amount <= 0:
			nitro_amount = 0.0
			nitro_available = false
			nitro_cooldown = 10.0
		emit_signal("nitro_used", NITRO_CONSUMPTION_RATE * delta)

func apply_wheel_forces() -> void:
	if _physics_body == null:
		return
	
	# Apply forward/reverse force based on gear
	var drive_force = _get_drive_force()
	
	if drive_force != 0:
		# Apply to rear wheels (simplified 2D)
		var wheel_position = Vector2(0, -WHEELBASE / 2)
		_physics_body.apply_central_force(Vector2(drive_force, 0))
		
		# Apply steering force to front wheels
		if abs(steering_input) > 0.1:
			var steer_force = steering_input * steer_force_multiplier * FRONT_WHEEL_FORCE
			var steer_position = Vector2(0, WHEELBASE / 2)
			_physics_body.apply_central_force(Vector2(steer_force, 0))

func _get_drive_force() -> float:
	if current_gear == 0:
		return 0.0
	
	var gear_ratio = _get_gear_multiplier()
	var base_force = REAR_WHEEL_FORCE * gear_ratio
	
	if current_gear < 0:  # Reverse
		base_force *= -1
	
	if nitro_available and nitro_amount > 0:
		base_force *= NITRO_BONUS
	
	return base_force * throttle_input

func apply_brakes() -> void:
	if _physics_body == null:
		return
	
	if brake_input > 0:
		var brake_total = BRAKE_FORCE_PER_WHEEL * 4 * brake_input
		_physics_body.apply_central_force(Vector2(-brake_total, 0))

func apply_steering() -> void:
	if _physics_body == null:
		return
	
	# Convert steering input to actual rotation
	var target_rotation = steering_input * MAX_STEERING_ANGLE
	steering_angle = lerp(steering_angle, target_rotation, delta * 5.0)
	
	# Apply rotational force
	var steer_force = steering_input * STEERING_SENSITIVITY * FRONT_WHEEL_FORCE
	_physics_body.apply_torque_impulse(steer_force)

func handle_collision(direction: Vector2) -> void:
	last_collision_time = Time.get_ticks_msec() / 1000.0
	current_vehicle_state = VehicleState.COLLIDED
	
	collision_damping = 0.5
	apply_central_force(-direction * 500.0)
	
	emit_signal("collision_detected", direction)
	
	# Reduce speed after collision
	current_speed *= 0.7

func reset_collision_state() -> void:
	var time_since_collision = Time.get_ticks_msec() / 1000.0 - last_collision_time
	if time_since_collision > 0.5:
		current_vehicle_state = VehicleState.RUNNING
		collision_damping = 0.3

func start_engine() -> void:
	if current_vehicle_state != VehicleState.IDLE:
		return
	
	current_vehicle_state = VehicleState.RUNNING
	rpm = idle_rpm
	emit_signal("engine_started")

func stop_engine() -> void:
	if current_vehicle_state == VehicleState.IDLE:
		return
	
	current_vehicle_state = VehicleState.IDLE
	rpm = idle_rpm
	current_gear = 0
	emit_signal("engine_stopped")

func set_throttle(input_value: float) -> void:
	throttle_input = clamp(input_value, 0.0, 1.0)

func set_brake(input_value: float) -> void:
	brake_input = clamp(input_value, 0.0, 1.0)

func set_steering(input_value: float) -> void:
	steering_input = clamp(input_value, -1.0, 1.0)

func get_current_state() -> VehicleState:
	return current_vehicle_state

func get_speed_info() -> Dictionary:
	return {
		"current_speed": current_speed,
		"target_speed": target_speed,
		"max_speed": MAX_SPEED.magnitude(),
		"acceleration": acceleration,
		"total_distance": total_distance_traveled
	}

func get_gear_info() -> Dictionary:
	return {
		"current_gear": current_gear,
		"rpm": rpm,
		"max_rpm": max_rpm,
		"idle_rpm": idle_rpm
	}

func get_nitro_info() -> Dictionary:
	return {
		"available": nitro_available,
		"amount": nitro_amount,
		"cooldown": nitro_cooldown
	}

func _on_engine_started() -> void:
	start_engine()

func _on_engine_stopped() -> void:
	stop_engine()

func _on_torque_applied(torque: float) -> void:
	acceleration = torque / _settings.default_vehicle_mass

func _on_game_state_changed(new_state: GameState) -> void:
	if new_state == GameState.RACE_ACTIVE:
		start_engine()
	elif new_state == GameState.RACE_FINISHED:
		stop_engine()

func _get_max_speed() -> float:
	var base_max_speed = 200.0  # km/h
	var gear_ratio = _get_gear_multiplier()
	var final_max = base_max_speed / gear_ratio if gear_ratio > 0 else base_max_speed
	return final_max * 1000.0 / 3600.0  # Convert to m/s

const MAX_SPEED = Vector2(MAX_SPEED.magnitude(), 0)

func _exit_tree() -> void:
	# Cleanup any resources
	pass

</FILE>