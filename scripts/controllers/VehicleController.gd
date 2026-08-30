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
const MAX_STEERING_ANGLE: float = PI / 4  # 45 degrees maximum steering
const STEERING_SMOOTHNESS: float = 0.1
const ACCELERATION_RATE: float = 300.0
const BRAKE_FORCE: float = 500.0
const FRICTION: float = 0.98
const TURN_SPEED: float = 3.0
const MIN_GEAR_SPEED: Array[float] = [0.0, 30.0, 50.0, 70.0, 90.0, 110.0, 130.0]
const MAX_GEAR_SPEED: Array[float] = [25.0, 50.0, 75.0, 100.0, 125.0, 150.0, 175.0]

# Time tracking
var _last_update_time: float = 0.0
var _delta_accumulator: float = 0.0
const FIXED_TIME_STEP: float = 0.01667  # 60 FPS fixed timestep

func _init() -> void:
	"""Initialize vehicle controller with default values."""
	current_vehicle_state = VehicleState.IDLE
	rpm = idle_rpm
	nitro_available = true
	nitro_amount = 100.0
	total_distance_traveled = 0.0
	last_collision_time = 0.0
	
	# Get physics settings singleton
	if Engine.has_singleton("PhysicsSettings"):
		_settings = Engine.get_singleton("PhysicsSettings")
	else:
		_settings = preload("res://scripts/core/PhysicsSettings.gd").new()

func _ready() -> void:
	"""Setup vehicle controller when node is ready."""
	_init_physics_body()
	_connect_signals()
	_setup_powertrain()
	_reset_vehicle()

func _process(delta: float) -> void:
	"""Main game loop - processes input and updates vehicle state."""
	if _settings == null:
		return
	
	_process_inputs(delta)
	_update_gear_system()
	_update_rpm(delta)
	_update_movement(delta)
	_handle_nitrous(delta)
	_check_collisions()
	_emit_signals()

func _input(event: InputEvent) -> void:
	"""Handle input events from keyboard/touch."""
	if not Engine.is_editor_hint():
		# Forward/Backward controls (WASD or Arrow keys)
		if event is InputEventKey:
			if event.pressed:
				if event.keycode == KEY_W or event.keycode == KEY_UP:
					throttle_input = 1.0
				elif event.keycode == KEY_S or event.keycode == KEY_DOWN:
					brake_input = 1.0
				elif event.keycode == KEY_A or event.keycode == KEY_LEFT:
					steering_input = -1.0
				elif event.keycode == KEY_D or event.keycode == KEY_RIGHT:
					steering_input = 1.0
				elif event.keycode == KEY_Q:
					shift_down_request = true
				elif event.keycode == KEY_E:
					shift_up_request = true
				elif event.keycode == KEY_SPACE:
					_trigger_nitrous()
		elif event is InputEventMouseButton:
			if event.button_mask & MOUSE_BUTTON_MASK_LEFT:
				throttle_input = 1.0 if event.pressed else 0.0
			elif event.button_mask & MOUSE_BUTTON_MASK_RIGHT:
				brake_input = 1.0 if event.pressed else 0.0

func _process_inputs(delta: float) -> void:
	"""Process and smooth input values."""
	# Smooth throttle input
	throttle_input = lerp(throttle_input, throttle_input * delta * 10.0, 0.1)
	
	# Smooth brake input
	brake_input = lerp(brake_input, brake_input * delta * 10.0, 0.1)
	
	# Smooth steering input
	steering_input = lerp(steering_input, steering_input * delta * 10.0, 0.1)
	
	# Reset shift requests after processing
	shift_up_request = false
	shift_down_request = false

func _update_gear_system() -> void:
	"""Update gear shifting based on RPM and speed."""
	var old_gear = current_gear
	
	# Automatic upshifting when reaching redline
	if current_gear < 6 and rpm >= max_rpm * 0.95:
		_shift_gear(current_gear + 1)
	
	# Automatic downshifting when below minimum gear speed
	elif current_gear > 1 and current_speed < MIN_GEAR_SPEED[current_gear]:
		_shift_gear(current_gear - 1)
	
	# Manual gear shifts
	if shift_up_request and current_gear < 6:
		_shift_gear(current_gear + 1)
	elif shift_down_request and current_gear > -1:
		_shift_gear(current_gear - 1)
	
	# Neutral gear handling
	if abs(current_speed) < 5.0 and current_gear != 0:
		_shift_gear(0)
	
	# Emit signal if gear changed
	if old_gear != current_gear:
		gear_changed.emit(old_gear, current_gear)

func _shift_gear(new_gear: int) -> void:
	"""Perform a gear shift with validation."""
	if new_gear < -1 or new_gear > 6:
		return
	
	# Check if we're already in this gear
	if new_gear == current_gear:
		return
	
	# Validate gear shift based on speed
	if new_gear > 0 and current_speed < MIN_GEAR_SPEED[new_gear]:
		return
	
	current_gear = new_gear
	
	# Simulate transmission delay
	await get_tree().create_timer(0.1).timeout
	
	# Apply gear change effects
	if current_gear == 0:
		target_speed *= 0.5  # Coasting effect
	elif current_gear > 0:
		# Accelerate towards new gear's top speed
		var gear_max = MAX_GEAR_SPEED[current_gear]
		target_speed = lerp(target_speed, gear_max, 0.2)
	else:
		# Reverse gear
		target_speed = -target_speed * 0.5

func _update_rpm(delta: float) -> void:
	"""Calculate and update engine RPM based on gear and speed."""
	if current_gear == 0:
		# Idle RPM in neutral
		rpm = lerp(rpm, idle_rpm, 0.1)
		return
	
	if current_gear < 0:
		# Reverse gear RPM
		rpm = lerp(rpm, idle_rpm * 1.2, 0.1)
		return
	
	# Calculate RPM based on current speed and gear ratio
	var gear_ratio: float = 3.5 / (current_gear * 0.5)  # Simple gear ratio model
	var wheel_rpm = abs(current_speed) * 100.0  # Simplified wheel RPM calculation
	rpm = min(wheel_rpm * gear_ratio, max_rpm)
	
	# Clamp RPM between idle and redline
	rpm = clamp(rpm, idle_rpm, max_rpm)
	
	# Add some variance for realism
	rpm += randf_range(-50.0, 50.0)
	rpm = clamp(rpm, idle_rpm, max_rpm)
	
	# Update target speed based on RPM
	if current_gear > 0:
		var max_speed_for_gear = MAX_GEAR_SPEED[current_gear]
		target_speed = lerp(target_speed, max_speed_for_gear, 0.05)

func _update_movement(delta: float) -> void:
	"""Update vehicle movement based on inputs and physics."""
	if _physics_body == null:
		return
	
	# Calculate target speed based on gear and throttle
	var base_target_speed = 0.0
	if current_gear > 0:
		base_target_speed = MAX_GEAR_SPEED[current_gear] * throttle_input
	elif current_gear < 0:
		base_target_speed = MAX_GEAR_SPEED[abs(current_gear)] * brake_input * -1.0
	else:
		# In neutral - minimal coasting
		base_target_speed = current_speed * 0.98
	
	# Apply nitrous bonus if available
	if nitro_available and nitro_amount > 0:
		base_target_speed *= NITRO_BONUS
	
	# Smoothly interpolate current speed toward target
	target_speed = lerp(target_speed, base_target_speed, 0.1)
	current_speed = lerp(current_speed, target_speed, 0.05)
	
	# Apply friction and resistance
	current_speed *= FRICTION
	
	# Handle braking
	if brake_input > 0:
		current_speed = lerp(current_speed, 0.0, 0.2)
		current_vehicle_state = VehicleState.BRAKING
	else:
		current_vehicle_state = VehicleState.RUNNING
	
	# Update steering angle based on input
	steering_angle = lerp(steering_angle, steering_input * MAX_STEERING_ANGLE, STEERING_SMOOTHNESS)
	
	# Apply movement to physics body
	if _physics_body != null:
		_apply_velocity_to_body()
		
	# Track distance traveled
	if abs(current_speed) > 1.0:
		total_distance_traveled += abs(current_speed) * delta * 3.6  # Convert to meters

func _apply_velocity_to_body() -> void:
	"""Apply velocity vector to the physics body."""
	if _physics_body == null:
		return
	
	# Calculate velocity direction based on current rotation
	var velocity_vector = Vector2.RIGHT.rotated(rotation) * current_speed
	
	# Apply velocity to character body
	_physics_body.velocity = velocity_vector
	_physics_body.move_and_slide()

func _handle_nitrous(delta: float) -> void:
	"""Manage nitrous system state and cooldown."""
	# Reduce cooldown over time
	if nitro_cooldown > 0:
		nitro_cooldown -= delta
		if nitro_cooldown <= 0:
			nitro_available = true
			nitro_cooldown = 0.0
	
	# Update display information
	if nitro_amount < 100.0:
		nitro_amount = clamp(nitro_amount + delta * 10.0, 0.0, 100.0)

func _trigger_nitrous() -> void:
	"""Activate nitrous boost if available."""
	if not nitro_available or nitro_amount <= 0:
		return
	
	nitro_available = false
	nitro_cooldown = 10.0  # 10 second cooldown
	nitro_amount -= NITRO_CONSUMPTION_RATE
	
	# Apply speed boost
	target_speed *= 1.5
	speed_changed.emit(current_speed, target_speed)
	
	# Emit signal
	nitro_used.emit(NITRO_CONSUMPTION_RATE)

func _check_collisions() -> void:
	"""Check for and handle vehicle collisions."""
	if _physics_body == null:
		return
	
	# Check collision with other objects
	for i in range(_physics_body.get_slide_collision_count()):
		var collision = _physics_body.get_slide_collision(i)
		var collider = collision.get_collider()
		
		# Log collision details
		var collision_normal = collision.get_normal()
		collision_detected.emit(collision_normal)
		
		# Apply collision damping
		current_speed *= collision_damping
		last_collision_time = Time.get_ticks_msec() / 1000.0
		
		# Update vehicle state
		current_vehicle_state = VehicleState.COLLIDED
		await get_tree().create_timer(0.2).timeout
		current_vehicle_state = VehicleState.RUNNING

func _emit_signals() -> void:
	"""Emit relevant signals based on vehicle state changes."""
	# Emit speed change signal if significant difference
	if abs(current_speed - speed_changed.last_value or 0) > 10.0:
		speed_changed.emit(current_speed, target_speed)
	
	# Emit RPM change signal
	if abs(rpm - rpm_changed.last_value or 0) > 100.0:
		rpm_changed.emit(rpm, max_rpm)
	
	# Emit movement signal if traveled meaningful distance
	if total_distance_traveled > 100.0:
		vehicle_moved.emit(total_distance_traveled)
		total_distance_traveled = 0.0

func _connect_signals() -> void:
	"""Connect internal signals to GameManager."""
	if GameManager == null:
		return
	
	game_manager.game_state_changed.connect(_on_game_state_changed)
	game_manager.race_started.connect(_on_race_started)

func _on_game_state_changed(new_state: GameManager.GameState) -> void:
	"""Handle game state changes affecting the vehicle."""
	match new_state:
		GameManager.GameState.RACE_ACTIVE:
			# Prepare vehicle for race
			reset_vehicle()
		GameManager.GameState.RACE_PAUSED:
			# Pause vehicle physics
			current_vehicle_state = VehicleState.IDLE
		GameManager.GameState.RACE_FINISHED:
			# Stop vehicle completely
			current_speed = 0.0
			target_speed = 0.0
			current_gear = 0
			rpm = idle_rpm

func _on_race_started(race_data: Dictionary) -> void:
	"""Handle race start event."""
	engine_started.emit()
	reset_vehicle()
	current_vehicle_state = VehicleState.REVVING

func _setup_powertrain() -> void:
	"""Configure powertrain component if exists."""
	if powertrain != null:
		powertrain.max_rpm = max_rpm
		powertrain.idle_rpm = idle_rpm
		powertrain.nitro_bonus = NITRO_BONUS

func _reset_vehicle() -> void:
	"""Reset vehicle to initial state."""
	current_speed = 0.0
	target_speed = 0.0
	current_gear = 0
	rpm = idle_rpm
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	steering_angle = 0.0
	total_distance_traveled = 0.0
	nitro_available = true
	nitro_amount = 100.0
	current_vehicle_state = VehicleState.IDLE

func _init_physics_body() -> void:
	"""Initialize the physics body reference."""
	_physics_body = $CharacterBody2D if get_node_or_null("CharacterBody2D") else null

func set_engine_state(on: bool) -> void:
	"""Turn engine on/off."""
	if on:
		engine_started.emit()
		rpm = idle_rpm
	else:
		engine_stopped.emit()
		rpm = 0.0

func get_current_gear() -> int:
	"""Get current gear number."""
	return current_gear

func get_current_speed() -> float:
	"""Get current speed in km/h."""
	return current_speed * 3.6

func get_total_distance() -> float:
	"""Get total distance traveled in meters."""
	return total_distance_traveled

func reset_vehicle_state() -> void:
	"""Completely reset vehicle state."""
	_reset_vehicle()
	_physics_body.velocity = Vector2.ZERO

func apply_damage(damage_amount: float) -> void:
	"""Apply damage to vehicle (reduces performance)."""
	if powertrain != null:
		powertrain.apply_damage(damage_amount)

func get_health() -> float:
	"""Get current vehicle health percentage."""
	if powertrain != null:
		return powertrain.health
	return 1.0

func save_state() -> Dictionary:
	"""Save current vehicle state for replay/loading."""
	return {
		"speed": current_speed,
		"gear": current_gear,
		"rpm": rpm,
		"nitro": nitro_amount,
		"distance": total_distance_traveled,
		"state": current_vehicle_state
	}

func load_state(state: Dictionary) -> void:
	"""Load vehicle state from saved data."""
	current_speed = state.get("speed", 0.0)
	current_gear = state.get("gear", 0)
	rpm = state.get("rpm", idle_rpm)
	nitro_amount = state.get("nitro", 100.0)
	total_distance_traveled = state.get("distance", 0.0)
	current_vehicle_state = state.get("state", VehicleState.IDLE)