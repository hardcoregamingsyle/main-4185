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
const GEAR_RATIOS: Array[float] = [0.0, 3.5, 2.8, 2.2, 1.8, 1.4, 1.1]
const REVERSE_GEAR_RATIO: float = -3.8
const FINAL_DRIVE_RATIO: float = 3.5
const WHEEL_RADIUS: float = 0.3
const MAX_STEERING_ANGLE: float = PI / 4  # 45 degrees
const MIN_SPEED_FOR_GEAR_SHIFT: float = 50.0
const RPM_TO_SPEED_FACTOR: float = 0.1
const FRICTION_COEFFICIENT: float = 0.98
const AIR_RESISTANCE: float = 0.999

# Movement tracking
var _distance_since_last_update: float = 0.0
var _last_position: Vector2 = Vector2.ZERO
var _velocity_vector: Vector2 = Vector2.ZERO

func _ready() -> void:
	_init_references()
	_load_settings()
	_setup_physics_body()
	_connect_signals_to_systems()
	
	current_vehicle_state = VehicleState.IDLE
	rpm = idle_rpm
	current_speed = 0.0
	
	if GameManager:
		GameManager.game_state_changed.connect(_on_game_state_changed)

func _init_references() -> void:
	if powertrain == null:
		powertrain = Powertrain.new()
	if chassis == null:
		chassis = Chassis.new()

func _load_settings() -> void:
	if PhysicsSettings:
		_settings = PhysicsSettings
	else:
		_settings = preload("res://scripts/core/PhysicsSettings.gd").new()

func _setup_physics_body() -> void:
	_physics_body = find_child("CharacterBody2D", true, false)
	if _physics_body == null:
		var body := CharacterBody2D.new()
		body.name = "CharacterBody2D"
		add_child(body)
		_physics_body = body

func _connect_signals_to_systems() -> void:
	if powertrain:
		powertrain.engine_started.connect(_on_engine_started)
		powertrain.engine_stopped.connect(_on_engine_stopped)

func _process(delta: float) -> void:
	if not is_ready():
		return
	
	_handle_inputs(delta)
	_update_gear_logic()
	_calculate_acceleration(delta)
	_apply_forces(delta)
	_update_rpm_and_speed(delta)
	_track_movement()
	_update_state(delta)
	_handle_nitro(delta)

func _handle_inputs(delta: float) -> void:
	if GameManager and GameManager.current_state != GameManager.GameState.RACE_ACTIVE:
		throttle_input = 0.0
		brake_input = 0.0
		steering_input = 0.0
		return
	
	throttle_input = InputManager.get_axis("throttle_up", "throttle_down")
	brake_input = InputManager.get_axis("brake", "brake_reverse")
	steering_input = InputManager.get_axis("steering_left", "steering_right")
	
	shift_up_request = Input.is_action_just_pressed("shift_up")
	shift_down_request = Input.is_action_just_pressed("shift_down")
	
	# Clamp inputs to valid range
	throttle_input = clamp(throttle_input, -1.0, 1.0)
	brake_input = clamp(brake_input, -1.0, 1.0)
	steering_input = clamp(steering_input, -1.0, 1.0)

func _update_gear_logic() -> void:
	# Automatic gear shifting based on RPM
	if current_gear != 0 and current_gear != -1:
		if rpm >= max_rpm * 0.95:
			request_shift_up()
		elif rpm < idle_rpm and current_gear > 1:
			request_shift_down()
		elif current_speed < MIN_SPEED_FOR_GEAR_SHIFT and current_gear > 1:
			request_shift_down()
	
	# Manual gear shifts
	if shift_up_request:
		request_shift_up()
	if shift_down_request:
		request_shift_down()
	
	# Reverse handling
	if current_gear == -1 and current_speed > 10.0:
		switch_to_gear(0)

func request_shift_up() -> void:
	if current_gear < 6:
		var old_gear = current_gear
		current_gear += 1
		emit_signal("gear_changed", old_gear, current_gear)
		
		# Engine rev down during upshift
		rpm = max(idle_rpm, rpm * 0.7)

func request_shift_down() -> void:
	if current_gear > 0:
		var old_gear = current_gear
		current_gear -= 1
		emit_signal("gear_changed", old_gear, current_gear)
		
		# Engine rev up during downshift
		rpm = min(max_rpm * 0.8, rpm * 1.3)

func switch_to_gear(gear: int) -> void:
	if gear == 0 or gear == -1 or (gear >= 1 and gear <= 6):
		var old_gear = current_gear
		current_gear = gear
		emit_signal("gear_changed", old_gear, current_gear)

func calculate_max_speed_for_gear() -> float:
	var ratio: float = GEAR_RATIOS[current_gear] if current_gear > 0 else 0.0
	if current_gear == -1:
		ratio = REVERSE_GEAR_RATIO
	return max_rpm * ratio * FINAL_DRIVE_RATIO * WHEEL_RADIUS * RPM_TO_SPEED_FACTOR

func _calculate_acceleration(delta: float) -> void:
	var current_max_speed = calculate_max_speed_for_gear()
	
	# Calculate target speed based on gear and inputs
	target_speed = current_max_speed
	
	# Apply throttle force
	if throttle_input > 0 and current_gear > 0:
		var gear_multiplier = GEAR_RATIOS[current_gear]
		var torque_factor = (rpm / max_rpm) * gear_multiplier
		acceleration = throttle_input * torque_factor * 150.0 * delta
		
		# Nitro bonus
		if nitro_available and nitro_amount > 0:
			acceleration *= NITRO_BONUS
	
	# Apply braking force
	if brake_input > 0:
		braking_force = brake_input * 300.0 * delta
		current_vehicle_state = VehicleState.BRAKING
	else:
		braking_force = 0.0
		if current_vehicle_state == VehicleState.BRAKING:
			current_vehicle_state = VehicleState.RUNNING
	
	# Natural deceleration (friction + air resistance)
	if throttle_input <= 0 and brake_input <= 0:
		var friction_loss = current_speed * (1.0 - FRICTION_COEFFICIENT)
		var air_resistance_loss = current_speed * current_speed * (1.0 - AIR_RESISTANCE)
		target_speed = max(0.0, target_speed - (friction_loss + air_resistance_loss))

func _apply_forces(delta: float) -> void:
	if _physics_body == null:
		return
	
	var movement_direction = Vector2.RIGHT.rotated(steering_angle * steering_input)
	
	# Calculate net force
	var net_force = acceleration - braking_force
	
	# Apply velocity change
	if current_gear > 0:
		_velocity_vector.x = (_velocity_vector.x + net_force * delta) * FRICTION_COEFFICIENT
	else:
		_velocity_vector = Vector2.ZERO
	
	# Apply steering influence to lateral movement
	if abs(steering_input) > 0.1:
		_velocity_vector.y = steering_input * current_speed * 0.5 * delta
	
	# Update position
	position += _velocity_vector * delta

func _update_rpm_and_speed(delta: float) -> void:
	var current_max_speed = calculate_max_speed_for_gear()
	
	# Smooth RPM transition
	var target_rpm = idle_rpm
	if current_gear > 0:
		target_rpm = current_speed / (WHEEL_RADIUS * FINAL_DRIVE_RATIO * RPM_TO_SPEED_FACTOR * GEAR_RATIOS[current_gear]) * max_rpm
		target_rpm = clamp(target_rpm, idle_rpm, max_rpm * 1.2)
	
	rpm = lerp(rpm, target_rpm, 5.0 * delta)
	current_speed = abs(_velocity_vector.length())
	
	# Emit signals
	if abs(rpm - rpm) > 10.0 or abs(current_speed - target_speed) > 1.0:
		emit_signal("rpm_changed", rpm, max_rpm)
		emit_signal("speed_changed", current_speed, current_max_speed)

func _track_movement() -> void:
	if _last_position == Vector2.ZERO:
		_last_position = position
	
	_distance_since_last_update = position.distance_to(_last_position)
	total_distance_traveled += _distance_since_last_update
	_last_position = position
	
	if _distance_since_last_update > 1.0:
		emit_signal("vehicle_moved", _distance_since_last_update)

func _update_state(delta: float) -> void:
	if current_speed < 1.0:
		current_vehicle_state = VehicleState.IDLE
	elif throttle_input > 0.1:
		current_vehicle_state = VehicleState.REVVING
	elif brake_input > 0.1:
		current_vehicle_state = VehicleState.BRAKING
	elif abs(steering_input) > 0.5:
		current_vehicle_state = VehicleState.DRIFTING
	else:
		current_vehicle_state = VehicleState.RUNNING

func _handle_nitro(delta: float) -> void:
	if nitro_cooldown > 0:
		nitro_cooldown -= delta
		return
	
	if Input.is_action_just_pressed("nitro"):
		if nitro_amount > 0:
			nitro_available = false
			nitro_amount -= NITRO_CONSUMPTION_RATE * delta
			nitro_cooldown = 10.0
			
			emit_signal("nitro_used", NITRO_CONSUMPTION_RATE * delta)

func use_nitro() -> void:
	if nitro_amount > 0 and nitro_cooldown <= 0:
		nitro_available = true
		nitro_amount = 100.0
		nitro_cooldown = 0.0

func reset_nitro() -> void:
	nitro_amount = 100.0
	nitro_cooldown = 0.0
	nitro_available = true

func start_engine() -> void:
	if current_gear == 0:
		current_gear = 1
		rpm = idle_rpm
		current_vehicle_state = VehicleState.RUNNING
		emit_signal("engine_started")

func stop_engine() -> void:
	current_gear = 0
	rpm = idle_rpm
	current_speed = 0.0
	_velocity_vector = Vector2.ZERO
	emit_signal("engine_stopped")

func apply_collision_impact(direction: Vector2) -> void:
	var impact_strength = direction.length()
	
	# Dampen velocity based on impact
	_velocity_vector = _velocity_vector * (1.0 - collision_damping * impact_strength)
	
	# Bounce effect
	if impact_strength > 10.0:
		_velocity_vector = -_velocity_vector * 0.5
	
	last_collision_time = Time.get_ticks_msec() / 1000.0
	current_vehicle_state = VehicleState.COLLIDED
	
	emit_signal("collision_detected", direction)

func get_wheel_torque() -> float:
	var gear_ratio = GEAR_RATIOS[current_gear] if current_gear > 0 else 0.0
	return rpm * gear_ratio * 0.5

func get_braking_effort() -> float:
	return braking_force

func get_current_max_speed() -> float:
	return calculate_max_speed_for_gear()

func reset_vehicle() -> void:
	current_gear = 0
	rpm = idle_rpm
	current_speed = 0.0
	_velocity_vector = Vector2.ZERO
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	total_distance_traveled = 0.0
	nitro_available = true
	nitro_amount = 100.0
	nitro_cooldown = 0.0
	reset_nitro()
	current_vehicle_state = VehicleState.IDLE
	position = Vector2.ZERO

func is_ready() -> bool:
	return _settings != null and is_instance_valid(powertrain)

func _on_game_state_changed(new_state: GameManager.GameState) -> void:
	if new_state == GameManager.GameState.MAIN_MENU or new_state == GameManager.GameState.SETTINGS:
		stop_engine()
		reset_vehicle()
	elif new_state == GameManager.GameState.RACE_ACTIVE:
		start_engine()

func _on_engine_started() -> void:
	current_vehicle_state = VehicleState.RUNNING

func _on_engine_stopped() -> void:
	current_vehicle_state = VehicleState.IDLE

func debug_get_status() -> Dictionary:
	return {
		"current_gear": current_gear,
		"current_speed": roundf(current_speed),
		"rpm": roundf(rpm),
		"max_rpm": max_rpm,
		"state": current_vehicle_state,
		"throttle": throttle_input,
		"brake": brake_input,
		"steering": steering_input,
		"nitro_amount": nitro_amount,
		"total_distance": roundf(total_distance_traveled),
		"ready": is_ready()
	}