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
signal state_changed(new_state: VehicleState)

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
var velocity_vector: Vector2 = Vector2.ZERO

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
var _drift_factor: float = 0.0

func _ready() -> void:
	_init_references()
	_load_physics_settings()
	_connect_signals()
	_reset_vehicle_state()
	
	if Engine.has_singleton("GameManager"):
		var gm: GameManager = Engine.get_singleton("GameManager")
		gm.game_state_changed.connect(_on_game_state_changed)

func _init_references() -> void:
	# Try to find physics body in parent hierarchy
	_physics_body = self as CharacterBody2D
	if not _physics_body:
		_physics_body = get_parent() as CharacterBody2D
	
	# Initialize powertrain if exists
	if powertrain:
		powertrain.engine_started.connect(_on_engine_started)
		powertrain.engine_stopped.connect(_on_engine_stopped)

func _load_physics_settings() -> void:
	# Get PhysicsSettings singleton if available
	if Engine.has_singleton("PhysicsSettings"):
		_settings = Engine.get_singleton("PhysicsSettings")
	else:
		_settings = preload("res://scripts/core/PhysicsSettings.gd").new()
	
	# Apply settings to local values
	max_rpm = _settings.default_vehicle_rpm if _settings and _settings.default_vehicle_rpm > 0 else 8000.0
	idle_rpm = _settings.idle_rpm if _settings and _settings.idle_rpm > 0 else 800.0

func _connect_signals() -> void:
	# Connect to global game manager if available
	if Engine.has_singleton("GameManager"):
		var gm: GameManager = Engine.get_singleton("GameManager")
		gm.race_started.connect(_on_race_started)
		gm.race_ended.connect(_on_race_ended)

func _reset_vehicle_state() -> void:
	current_speed = 0.0
	target_speed = 0.0
	acceleration = 0.0
	braking_force = 0.0
	steering_angle = 0.0
	current_gear = 0
	rpm = idle_rpm
	total_distance_traveled = 0.0
	_is_engine_running = false
	current_vehicle_state = VehicleState.IDLE
	nitro_available = true
	nitro_amount = 100.0
	nitro_cooldown = 0.0
	_accumulated_distance = 0.0
	_drift_factor = 0.0
	_last_position = position

func _process(delta: float) -> void:
	_process_input(delta)
	_update_physics(delta)
	_update_gears(delta)
	_update_nitrous(delta)
	_update_state(delta)
	_emit_signals()

func _process_input(delta: float) -> void:
	# Read input from InputManager singleton
	if Engine.has_singleton("InputManager"):
		var im: InputManager = Engine.get_singleton("InputManager")
		
		throttle_input = im.get_axis("throttle", "brake")
		brake_input = im.get_axis("brake", "") * -1.0
		steering_input = im.get_axis("steer_left", "steer_right")
		shift_up_request = im.is_action_pressed("shift_up")
		shift_down_request = im.is_action_pressed("shift_down")
	else:
		# Fallback to keyboard input
		throttle_input = Input.get_key_pressed(KEY_W) ? 1.0 : (Input.get_key_pressed(KEY_S) ? -1.0 : 0.0)
		brake_input = Input.get_key_pressed(KEY_SPACE) ? 1.0 : 0.0
		steering_input = Input.get_key_pressed(KEY_A) ? -1.0 : (Input.get_key_pressed(KEY_D) ? 1.0 : 0.0)
		shift_up_request = Input.is_action_just_pressed("ui_pageup")
		shift_down_request = Input.is_action_just_pressed("ui_pagedown")

func _update_physics(delta: float) -> void:
	if not _is_engine_running:
		return
	
	# Calculate engine torque based on RPM and gear
	var torque: float = _calculate_engine_torque()
	
	# Apply throttle input multiplier
	torque *= (1.0 + throttle_input) if throttle_input > 0 else 1.0
	
	# Apply nitro boost if active
	if _is_nitro_active():
		torque *= NITRO_BONUS
	
	# Apply braking force
	if brake_input > 0:
		var brake_effective_force: float = braking_force * brake_input
		current_speed = max(0.0, current_speed - brake_effective_force * delta)
	else:
		# Calculate acceleration based on torque and gear ratio
		var gear_ratio: float = _get_current_gear_ratio()
		var wheel_torque: float = torque * gear_ratio
		
		# Apply friction and air resistance
		var resistive_force: float = (current_speed * current_speed * AIR_RESISTANCE_FACTOR) + (FRICTION_COEFFICIENT * gravity())
		var net_force: float = wheel_torque - resistive_force
		
		# Update speed
		acceleration = net_force / _settings.default_vehicle_mass if _settings and _settings.default_vehicle_mass > 0 else 0.0
		acceleration = clamp(acceleration, -MAX_DECELERATION, MAX_ACCELERATION)
		current_speed += acceleration * delta
	
	# Clamp speed to reasonable bounds
	current_speed = clamp(current_speed, -MAX_ACCELERATION * 10.0, MAX_ACCELERATION * 10.0)
	
	# Update velocity vector
	velocity_vector.x = current_speed * cos(steering_angle)
	velocity_vector.y = current_speed * sin(steering_angle)
	
	# Apply steering angle
	steering_angle += steering_input * MAX_STEERING_ANGLE * delta
	
	# Track distance traveled
	var movement_delta: float = abs(velocity_vector.length() * delta)
	_accumulated_distance += movement_delta
	total_distance_traveled += movement_delta
	
	# Update last position for collision detection
	_last_position = position

func _calculate_engine_torque() -> float:
	# Torque curve based on RPM - typical engine has peak torque in mid-RPM range
	var normalized_rpm: float = rpm / max_rpm
	
	var torque_curve: float
	if normalized_rpm < 0.3:
		torque_curve = 0.3 + (normalized_rpm * 1.5)
	elif normalized_rpm < 0.7:
		torque_curve = 1.0
	else:
		torque_curve = 1.0 - ((normalized_rpm - 0.7) * 0.8)
	
	return torque_curve * TORQUE_MULTIPLIER

func _get_current_gear_ratio() -> float:
	match current_gear:
		-1: return REVERSE_GEAR_RATIO
		0: return 0.0
		_: 
			if current_gear < GEAR_RATIOS.size():
				return GEAR_RATIOS[current_gear]
			else:
				return GEAR_RATIOS[GEAR_RATIOS.size() - 1]
	return 1.0

func _update_gears(delta: float) -> void:
	if _gear_change_timer > 0:
		_gear_change_timer -= delta
		return
	
	# Automatic gear shifting logic
	if _is_engine_running and current_gear != -1:
		var gear_ratio: float = _get_current_gear_ratio()
		var wheel_rpm: float = (current_speed * gear_ratio) / 100.0
		
		# Shift up if RPM too high
		if rpm > max_rpm * 0.95 and current_gear < 6:
			_shift_to_gear(current_gear + 1)
		
		# Shift down if RPM too low
		elif rpm < idle_rpm * 1.2 and current_gear > 1:
			_shift_to_gear(current_gear - 1)
	
	# Manual gear shift requests
	if shift_up_request:
		_shift_to_gear(min(current_gear + 1, 6))
		shift_up_request = false
	
	elif shift_down_request:
		_shift_to_gear(max(current_gear - 1, -1))
		shift_down_request = false

func _shift_to_gear(new_gear: int) -> void:
	if new_gear == current_gear:
		return
	
	var old_gear: int = current_gear
	current_gear = new_gear
	_gear_change_timer = GEAR_CHANGE_TIME
	
	# Emit signal
	gear_changed.emit(old_gear, new_gear)
	
	# Brief RPM drop during gear change
	rpm = idle_rpm * 0.5

func _update_nitrous(delta: float) -> void:
	# Handle nitro cooldown
	if nitro_cooldown > 0:
		nitro_cooldown -= delta
		return
	
	# Check if nitro can be activated
	if nitro_available and nitro_amount >= 10.0:
		var nitro_activation: bool = Input.is_action_just_pressed("nitro")
		if nitro_activation:
			_activate_nitro()

func _activate_nitro() -> void:
	if nitro_amount < 10.0:
		return
	
	nitro_available = false
	nitro_cooldown = 5.0
	nitro_amount -= 10.0
	
	# Boost current speed temporarily
	var boost_multiplier: float = NITRO_BONUS
	current_speed *= boost_multiplier
	
	# Emit signal
	nitro_used.emit(nitro_amount)

func _is_nitro_active() -> bool:
	return nitro_available and nitro_amount >= 10.0

func _update_state(delta: float) -> void:
	var previous_state: VehicleState = current_vehicle_state
	
	# Determine current state based on conditions
	if current_speed <= 0.1 and rpm < idle_rpm:
		current_vehicle_state = VehicleState.IDLE
	elif rpm > max_rpm * 0.9:
		current_vehicle_state = VehicleState.REVVING
	elif brake_input > 0.5:
		current_vehicle_state = VehicleState.BRAKING
	elif abs(steering_input) > 0.5 and abs(current_speed) > MIN_SPEED_FOR_GEAR_SHIFT:
		current_vehicle_state = VehicleState.DRIFTING
	else:
		current_vehicle_state = VehicleState.RUNNING
	
	# Detect collisions
	if _has_recent_collision():
		current_vehicle_state = VehicleState.COLLIDED
		collision_damping = 0.5
	else:
		collision_damping = lerp(collision_damping, 0.3, delta * 2.0)
	
	# Emit state change signal if changed
	if current_vehicle_state != previous_state:
		state_changed.emit(current_vehicle_state)

func _has_recent_collision() -> bool:
	return Time.get_ticks_msec() - last_collision_time < 500

func _emit_signals() -> void:
	# Emit speed change signal periodically
	if _should_emit_speed_signal():
		speed_changed.emit(current_speed, max_rpm * 0.5)
	
	# Emit RPM change signal
	rpm_changed.emit(rpm, max_rpm)
	
	# Emit distance signal periodically
	if _should_emit_distance_signal():
		vehicle_moved.emit(_accumulated_distance)

func _should_emit_speed_signal() -> bool:
	return randf() < 0.1 or abs(current_speed - target_speed) > 1.0

func _should_emit_distance_signal() -> bool:
	return _accumulated_distance > 10.0

func _on_engine_started() -> void:
	_is_engine_running = true
	current_gear = 1
	rpm = idle_rpm
	engine_started.emit()

func _on_engine_stopped() -> void:
	_is_engine_running = false
	current_gear = 0
	rpm = 0.0
	engine_stopped.emit()

func _on_game_state_changed(new_state: GameState) -> void:
	match new_state:
		GameState.MAIN_MENU:
			_stop_engine()
		GameState.RACE_ACTIVE:
			_start_engine()
		GameState.RACE_PAUSED:
			pass  # Pause handling
		GameState.RACE_FINISHED:
			_stop_engine()

func _on_race_started(race_data: Dictionary) -> void:
	_start_engine()
	_reset_vehicle_state()
	_accumulated_distance = 0.0
	nitro_amount = 100.0
	nitro_available = true

func _on_race_ended(results: Dictionary) -> void:
	_stop_engine()

func _start_engine() -> void:
	if not _is_engine_running:
		_is_engine_running = true
		current_gear = 1
		rpm = idle_rpm
		engine_started.emit()

func _stop_engine() -> void:
	if _is_engine_running:
		_is_engine_running = false
		current_gear = 0
		rpm = 0.0
		engine_stopped.emit()

func set_engine_running(is_running: bool) -> void:
	if is_running:
		_start_engine()
	else:
		_stop_engine()

func reset_nitro() -> void:
	nitro_available = true
	nitro_amount = 100.0
	nitro_cooldown = 0.0

func get_current_speed() -> float:
	return current_speed

func get_current_rpm() -> float:
	return rpm

func get_current_gear() -> int:
	return current_gear

func get_total_distance() -> float:
	return total_distance_traveled

func get_nitro_amount() -> float:
	return nitro_amount

func apply_collision_impact(direction: Vector2, force: float) -> void:
	last_collision_time = Time.get_ticks_msec()
	collision_detected.emit(direction)
	
	# Apply damping to velocity
	velocity_vector = velocity_vector * (1.0 - collision_damping)
	current_speed = velocity_vector.length()

func set_target_speed(speed: float) -> void:
	target_speed = clamp(speed, 0.0, max_rpm * 0.5)

func get_max_speed() -> float:
	return max_rpm * 0.5

func calculate_lateral_grip() -> float:
	if current_vehicle_state == VehicleState.DRIFTING:
		return GRIP_COEFFICIENT * DRIFT_GRIP_REDUCTION
	else:
		return GRIP_COEFFICIENT

func calculate_longitudinal_force() -> float:
	return torque_input * TORQUE_MULTIPLIER * (_get_current_gear_ratio() / 100.0)

func torque_input: float:
	return throttle_input

func gravity() -> float:
	if _settings:
		return _settings.gravity
	return 9.81

func dispose() -> void:
	_disconnect_signals()
	_reset_vehicle_state()
	powertrain = null
	chassis = null
	_settings = null

func _disconnect_signals() -> void:
	if Engine.has_singleton("GameManager"):
		var gm: GameManager = Engine.get_singleton("GameManager")
		gm.game_state_changed.disconnect(_on_game_state_changed)
		gm.race_started.disconnect(_on_race_started)
		gm.race_ended.disconnect(_on_race_ended)

if Engine.has_singleton("GameManager"):
	Engine.get_singleton("GameManager").game_state_changed.connect(_on_game_state_changed)
</FILE>