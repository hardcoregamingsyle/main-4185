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
const AIR_RESISTANCE_COEFFICIENT: float = 0.002
const ENGINE_POWER: float = 250000.0  # Watts (~335 hp)
const MAX_THRUST_FORCE: float = 15000.0
const MIN_THRUST_FORCE: float = 500.0
const BRAKE_DECCELERATION: float = 20.0
const TURN_SMOOTHING: float = 8.0
const MAX_DRIFT_ANGLE: float = PI / 6  # 30 degrees
const DRIFT_FRICTION_REDUCTION: float = 0.7
const ACCELERATION_TIME_STEP: float = 0.1
const MAX_ACCELERATION: float = 15.0
const MIN_ACCELERATION: float = -20.0

func _ready() -> void:
	_init_settings()
	_setup_physics_body()
	_connect_signals_to_powertrain()
	_init_nitrous_system()
	_reset_vehicle()

func _init_settings() -> void:
	if GameManager.has_singleton("PhysicsSettings"):
		_settings = GameManager.get_singleton("PhysicsSettings")
	else:
		_settings = preload("res://scripts/core/PhysicsSettings.gd").new()

func _setup_physics_body() -> void:
	_physics_body = get_parent() as CharacterBody2D
	if _physics_body == null:
		push_error("VehicleController: No valid physics body found")

func _connect_signals_to_powertrain() -> void:
	if powertrain != null:
		powertrain.rpm_changed.connect(_on_powertrain_rpm_changed)
		powertrain.engine_started.connect(_on_engine_started)
		powertrain.engine_stopped.connect(_on_engine_stopped)

func _init_nitrous_system() -> void:
	nitro_amount = 100.0
	nitro_available = true
	nitro_cooldown = 0.0

func _reset_vehicle() -> void:
	current_speed = 0.0
	target_speed = 0.0
	acceleration = 0.0
	braking_force = 0.0
	steering_angle = 0.0
	current_gear = 0
	rpm = idle_rpm
	total_distance_traveled = 0.0
	current_vehicle_state = VehicleState.IDLE

func _process(delta: float) -> void:
	_update_inputs(delta)
	_handle_gear_shifting()
	_calculate_acceleration(delta)
	_apply_forces(delta)
	_update_rpm_and_gear(delta)

func _update_inputs(delta: float) -> void:
	# Get input values from InputManager singleton
	if GameManager.has_singleton("InputManager"):
		var input_data = GameManager.get_singleton("InputManager").get_vehicle_inputs()
		throttle_input = clamp(input_data.throttle, 0.0, 1.0)
		brake_input = clamp(input_data.brake, 0.0, 1.0)
		steering_input = clamp(input_data.steering, -1.0, 1.0)
		
		shift_up_request = input_data.shift_up
		shift_down_request = input_data.shift_down
		
		# Handle nitrous usage
		if input_data.nitrous_active and nitro_available and nitro_amount > 0:
			_use_nitrous()

func _handle_gear_shifting() -> void:
	# Automatic upshifts when reaching redline in higher gears
	if current_gear >= 1 and current_gear < 6 and rpm >= max_rpm * 0.95:
		_shift_gear(current_gear + 1)
	
	# Automatic downshifts when RPM drops too low
	elif current_gear > 1 and rpm <= idle_rpm * 1.2:
		_shift_gear(current_gear - 1)
	
	# Manual gear shifts
	if shift_up_request and current_gear < 6:
		_shift_gear(current_gear + 1)
	elif shift_down_request and current_gear > 0:
		_shift_gear(current_gear - 1)

func _shift_gear(new_gear: int) -> void:
	if new_gear == current_gear:
		return
	
	if new_gear == 0 and current_gear == 0:
		return
	
	var old_gear = current_gear
	current_gear = new_gear
	
	gear_changed.emit(old_gear, current_gear)
	
	# Apply clutch slip effect during gear change
	if current_gear != 0:
		rpm = max(idle_rpm, rpm * 0.7)
	else:
		rpm = idle_rpm
	
	# Update powertrain gear
	if powertrain != null:
		powertrain.set_gear(current_gear)

func _calculate_acceleration(delta: float) -> void:
	# Calculate target speed based on gear and throttle
	var gear_ratio = _get_current_gear_ratio()
	var max_gear_speed = _get_max_speed_for_gear(gear_ratio)
	target_speed = max_gear_speed * throttle_input
	
	# Apply acceleration towards target
	var speed_diff = target_speed - current_speed
	acceleration = lerp(acceleration, speed_diff * ACCELERATION_TIME_STEP, delta * 10.0)
	acceleration = clamp(acceleration, MIN_ACCELERATION, MAX_ACCELERATION)
	
	# Brake overrides acceleration
	if brake_input > 0.1:
		acceleration = -BRAKE_DECCELERATION * brake_input
	
	# Apply air resistance
	var air_resistance = current_speed * current_speed * AIR_RESISTANCE_COEFFICIENT
	acceleration -= air_resistance
	
	# Apply friction
	var friction_loss = current_speed * FRICTION_COEFFICIENT * 0.001
	acceleration -= friction_loss

func _apply_forces(delta: float) -> void:
	if _physics_body == null:
		return
	
	# Calculate thrust force based on engine power and gear
	var gear_ratio = _get_current_gear_ratio()
	var base_thrust = ENGINE_POWER / gear_ratio * throttle_input
	base_thrust = clamp(base_thrust, MIN_THRUST_FORCE, MAX_THRUST_FORCE)
	
	# Apply thrust
	var velocity_direction = global_rotation
	var thrust_vector = Vector2.RIGHT.rotated(velocity_direction) * base_thrust
	_physics_body.velocity += thrust_vector * delta
	
	# Apply steering (rotational force)
	if abs(steering_input) > 0.01:
		var steering_force = steering_input * TURN_SMOOTHING * base_thrust * 0.1
		_physics_body.angular_velocity += steering_force * delta
	
	# Apply brake force
	if brake_input > 0.1:
		var brake_vector = -_physics_body.velocity.normalized() * brake_input * BRAKE_DECCELERATION * delta
		_physics_body.velocity += brake_vector
	
	# Clamp speed
	current_speed = _physics_body.velocity.length()
	_physics_body.velocity = _physics_body.velocity.normalized() * min(current_speed, _get_max_overall_speed())

func _update_rpm_and_gear(delta: float) -> void:
	# Calculate RPM based on gear ratio and speed
	var gear_ratio = _get_current_gear_ratio()
	var theoretical_rpm = current_speed * gear_ratio * 10.0
	
	# Smooth RPM transitions
	rpm = lerp(rpm, theoretical_rpm, delta * 2.0)
	rpm = clamp(rpm, idle_rpm, max_rpm)
	
	# Signal RPM changes
	rpm_changed.emit(rpm, max_rpm)
	
	# Update vehicle state
	_update_vehicle_state()

func _update_vehicle_state() -> void:
	var time_since_collision = Time.get_unix_time_from_system() - last_collision_time
	
	if current_vehicle_state == VehicleState.COLLIDED and time_since_collision > 1.0:
		current_vehicle_state = VehicleState.RUNNING
	
	if throttle_input > 0.1 and current_speed > 10.0:
		current_vehicle_state = VehicleState.RUNNING
	elif brake_input > 0.1:
		current_vehicle_state = VehicleState.BRAKING
	elif throttle_input > 0.5 and rpm > idle_rpm * 2:
		current_vehicle_state = VehicleState.REVVING
	elif current_speed < 5.0:
		current_vehicle_state = VehicleState.IDLE

func _use_nitrous() -> void:
	if not nitro_available or nitro_amount <= 0:
		return
	
	nitro_available = false
	nitro_cooldown = 5.0
	
	# Boost speed significantly
	var boost_factor = NITRO_BONUS
	target_speed *= boost_factor
	
	# Consume nitro
	nitro_amount -= NITRO_CONSUMPTION_RATE * 0.1
	
	# Emit signal
	nitro_used.emit(NITRO_CONSUMPTION_RATE * 0.1)
	
	# Visual/audio feedback
	if AudioManager.has_singleton("AudioManager"):
		var audio = AudioManager.get_singleton("AudioManager")
		audio.play_sound("nitro_activate")

func _get_current_gear_ratio() -> float:
	if current_gear == 0:
		return 0.0
	elif current_gear < 0:
		return REVERSE_GEAR_RATIO
	else:
		return GEAR_RATIOS[current_gear] if current_gear < GEAR_RATIOS.size() else 0.8

func _get_max_speed_for_gear(gear_ratio: float) -> float:
	if gear_ratio == 0:
		return 0.0
	return (ENGINE_POWER / (gear_ratio * 100.0)) * 3.6  # Convert to km/h equivalent

func _get_max_overall_speed() -> float:
	# Maximum vehicle speed (top speed in neutral-ish gear)
	return 320.0  # km/h equivalent

func handle_collision(direction: Vector2) -> void:
	last_collision_time = Time.get_unix_time_from_system()
	collision_damping = 0.5
	collision_detected.emit(direction)
	
	# Reduce speed on collision
	current_speed *= 0.7
	if _physics_body != null:
		_physics_body.velocity = _physics_body.velocity * 0.7
	
	current_vehicle_state = VehicleState.COLLIDED

func reset_collision_state() -> void:
	collision_damping = 0.3
	last_collision_time = 0.0
	current_vehicle_state = VehicleState.RUNNING

func reset_nitrous() -> void:
	nitro_amount = 100.0
	nitro_available = true
	nitro_cooldown = 0.0

func toggle_engine(state: bool) -> void:
	if state:
		engine_started.emit()
		rpm = idle_rpm
	else:
		engine_stopped.emit()
		rpm = 0.0

func _on_engine_started() -> void:
	current_vehicle_state = VehicleState.IDLE
	rpm = idle_rpm

func _on_engine_stopped() -> void:
	current_vehicle_state = VehicleState.IDLE
	rpm = 0.0
	current_speed = 0.0

func _on_powertrain_rpm_changed(new_rpm: float) -> void:
	rpm = new_rpm
	rpm_changed.emit(rpm, max_rpm)

func get_vehicle_stats() -> Dictionary:
	return {
		"current_speed": current_speed,
		"target_speed": target_speed,
		"current_gear": current_gear,
		"rpm": rpm,
		"max_rpm": max_rpm,
		"throttle_input": throttle_input,
		"brake_input": brake_input,
		"steering_input": steering_input,
		"nitro_amount": nitro_amount,
		"nitro_available": nitro_available,
		"total_distance": total_distance_traveled,
		"vehicle_state": current_vehicle_state
	}

func set_custom_parameters(custom_params: Dictionary) -> void:
	if custom_params.has("max_rpm"):
		max_rpm = custom_params["max_rpm"]
	if custom_params.has("engine_power"):
		ENGINE_POWER = custom_params["engine_power"]
	if custom_params.has("gear_ratios") and typeof(custom_params["gear_ratios"]) == TYPE_ARRAY:
		GEAR_RATIOS = custom_params["gear_ratios"]
	if custom_params.has("thrust_force"):
		MAX_THRUST_FORCE = custom_params["thrust_force"]

func _to_string() -> String:
	return "<VehicleController speed=%.1f gear=%d rpm=%.0f>" % [current_speed, current_gear, rpm]