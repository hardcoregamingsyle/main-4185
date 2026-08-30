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
const MAX_STEERING_ANGLE: float = PI / 4  # 45 degrees
const MIN_SPEED_FOR_SHIFTING: float = 1.0
const GEAR_RATIOS: Array[float] = [0.0, 3.5, 2.8, 2.2, 1.8, 1.5, 1.2]
const REVERSE_GEAR_RATIO: float = 4.0
const FRICTION_COEFFICIENT: float = 0.85
const AIR_RESISTANCE: float = 0.001

func _ready() -> void:
	_settings = GameManager.get_singleton("PhysicsSettings")
	_init_physics_body()
	_connect_signals()
	_reset_vehicle()

func _init_physics_body() -> void:
	var parent = get_parent()
	if parent is CharacterBody2D:
		_physics_body = parent
	else:
		printerr("VehicleController: No CharacterBody2D found on parent")
		_physics_body = self as CharacterBody2D

func _connect_signals() -> void:
	if powertrain:
		powertrain.engine_started.connect(_on_engine_started)
		powertrain.engine_stopped.connect(_on_engine_stopped)
		powertrain.rpm_changed.connect(_on_powertrain_rpm_changed)

func _reset_vehicle() -> void:
	current_gear = 0
	rpm = idle_rpm
	current_speed = 0.0
	target_speed = 0.0
	total_distance_traveled = 0.0
	nitro_amount = 100.0
	nitro_available = true
	current_vehicle_state = VehicleState.IDLE

func _process(delta: float) -> void:
	_handle_inputs(delta)
	_update_physics(delta)
	_update_gear_logic(delta)
	_update_nitro_system(delta)
	_emit_signals()

func _handle_inputs(delta: float) -> void:
	var input_manager = GameManager.get_singleton("InputManager")
	
	throttle_input = input_manager.get_axis("throttle_forward", "throttle_backward")
	brake_input = input_manager.get_axis("brake", "reverse_brake")
	steering_input = input_manager.get_axis("steer_left", "steer_right")
	
	shift_up_request = input_manager.is_action_pressed("shift_up")
	shift_down_request = input_manager.is_action_pressed("shift_down")

func _update_physics(delta: float) -> void:
	if not _physics_body or not powertrain:
		return
	
	var mass = _settings.default_vehicle_mass
	var dt = delta * _settings.time_scale
	
	# Calculate target speed based on gear and RPM
	target_speed = _calculate_target_speed()
	
	# Apply acceleration/deceleration
	acceleration = _calculate_acceleration(dt)
	current_speed += acceleration * dt
	
	# Apply air resistance
	var air_resistance_force = current_speed * current_speed * AIR_RESISTANCE
	current_speed -= air_resistance_force * dt
	
	# Apply friction when no input
	if abs(throttle_input) < 0.01 and abs(brake_input) < 0.01:
		var friction_force = current_speed * FRICTION_COEFFICIENT * 0.1
		current_speed -= friction_force * dt
	
	# Clamp speed
	current_speed = clamp(current_speed, -max_speed_for_gear(), max_speed_for_gear())
	
	# Update distance traveled
	var distance = abs(current_speed) * dt
	total_distance_traveled += distance
	
	# Update steering angle
	steering_angle = steering_input * MAX_STEERING_ANGLE
	
	# Apply velocity to physics body
	if _physics_body is CharacterBody2D:
		_physics_body.velocity = Vector2(current_speed, 0)
		_physics_body.move_and_slide()
	elif _physics_body == self:
		position.x += current_speed * dt

func _calculate_target_speed() -> float:
	if current_gear == 0:
		return 0.0
	
	var ratio = GEAR_RATIOS[current_gear] if current_gear > 0 else REVERSE_GEAR_RATIO
	var base_max_speed = 200.0  # Base max speed in km/h
	var top_speed = base_max_speed / ratio
	return top_speed * (rpm / max_rpm)

func _calculate_acceleration(delta: float) -> float:
	if current_gear == 0:
		return 0.0
	
	var mass = _settings.default_vehicle_mass
	var torque = powertrain.get_current_torque()
	var ratio = GEAR_RATIOS[current_gear] if current_gear > 0 else REVERSE_GEAR_RATIO
	var force = (torque * ratio) / mass
	
	# Apply throttle multiplier
	force *= (1.0 + throttle_input)
	
	# Apply nitro bonus
	if nitro_available and nitro_amount > 0 and throttle_input > 0.5:
		force *= NITRO_BONUS
	
	# Apply braking force
	if brake_input > 0:
		force -= (brake_input * mass * 9.81 * 0.5)
	
	return force

func _calculate_max_rpm_for_gear() -> float:
	return max_rpm * (1.0 / GEAR_RATIOS[current_gear]) if current_gear > 0 else max_rpm

func _update_gear_logic(delta: float) -> void:
	# Automatic upshifting
	if current_gear < 6 and rpm >= max_rpm * 0.95 and throttle_input > 0.1:
		shift_gear(1)
		shift_up_request = false
	
	# Manual upshifting
	if shift_up_request and current_gear < 6:
		shift_gear(1)
	
	# Automatic downshifting at low RPM
	if current_gear > 1 and rpm <= idle_rpm * 1.2 and throttle_input < 0.1:
		shift_gear(-1)
		shift_down_request = false
	
	# Manual downshifting
	if shift_down_request and current_gear > -1:
		shift_gear(-1)
	
	# Neutral at zero speed with no input
	if current_speed < MIN_SPEED_FOR_SHIFTING and throttle_input < 0.01 and brake_input < 0.01:
		shift_gear(0)

func _update_nitro_system(delta: float) -> void:
	if nitro_cooldown > 0:
		nitro_cooldown -= delta
		return
	
	# Activate nitro
	if throttle_input > 0.7 and nitro_available and nitro_amount > 0:
		nitro_amount -= NITRO_CONSUMPTION_RATE * delta
		if nitro_amount <= 0:
			nitro_amount = 0
			nitro_available = false
			nitro_cooldown = 5.0
		emit_signal("nitro_used", NITRO_CONSUMPTION_RATE * delta)

func shift_gear(direction: int) -> void:
	var old_gear = current_gear
	var new_gear = current_gear + direction
	
	# Validate gear change
	if new_gear < -1 or new_gear > 6:
		return
	
	# Prevent reverse while moving forward
	if new_gear == -1 and current_speed > MIN_SPEED_FOR_SHIFTING:
		new_gear = current_gear
		return
	
	# Prevent forward while reversing
	if new_gear > 0 and current_speed < -MIN_SPEED_FOR_SHIFTING:
		new_gear = current_gear
		return
	
	current_gear = new_gear
	rpm = idle_rpm
	
	emit_signal("gear_changed", old_gear, current_gear)

func start_engine() -> void:
	if current_gear != 0:
		return
	rpm = idle_rpm
	current_vehicle_state = VehicleState.RUNNING
	emit_signal("engine_started")

func stop_engine() -> void:
	rpm = 0.0
	current_vehicle_state = VehicleState.IDLE
	emit_signal("engine_stopped")

func apply_brake(force: float) -> void:
	braking_force = force
	if _physics_body is CharacterBody2D:
		_physics_body.velocity.y -= force * 0.1

func release_brake() -> void:
	braking_force = 0.0

func reset() -> void:
	_reset_vehicle()
	start_engine()

func get_speed_kmh() -> float:
	return current_speed * 3.6

func get_max_speed_kmh() -> float:
	return max_speed_for_gear() * 3.6

func max_speed_for_gear() -> float:
	if current_gear == 0:
		return 0.0
	var ratio = GEAR_RATIOS[current_gear] if current_gear > 0 else REVERSE_GEAR_RATIO
	return 200.0 / ratio

func get_gear_ratio() -> float:
	return GEAR_RATIOS[current_gear] if current_gear > 0 else REVERSE_GEAR_RATIO

func _emit_signals() -> void:
	emit_signal("speed_changed", current_speed, max_speed_for_gear())
	emit_signal("rpm_changed", rpm, max_rpm)
	emit_signal("vehicle_moved", total_distance_traveled)

func _on_engine_started() -> void:
	current_vehicle_state = VehicleState.RUNNING

func _on_engine_stopped() -> void:
	current_vehicle_state = VehicleState.IDLE

func _on_powertrain_rpm_changed(new_rpm: float) -> void:
	rpm = new_rpm

func _get_gear_display_text() -> String:
	match current_gear:
		-1: return "R"
		0: return "N"
		_: return str(current_gear)

func _is_valid_shift(old_gear: int, new_gear: int) -> bool:
	# Cannot shift directly between reverse and forward
	if (old_gear < 0 and new_gear > 0) or (old_gear > 0 and new_gear < 0):
		return false
	return true

func _clamp_steering(angle: float) -> float:
	return clamp(angle, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE)

func _apply_wheel_forces(wheel_count: int, force_multiplier: float) -> void:
	var base_force = current_speed * force_multiplier
	for i in range(wheel_count):
		pass  # Placeholder for wheel-specific force application

func _on_collision(other: Node) -> void:
	var direction = position.direction_to(other.position)
	last_collision_time = Time.get_ticks_msec() / 1000.0
	current_vehicle_state = VehicleState.COLLIDED
	collision_damping = 0.3
	emit_signal("collision_detected", direction)

func _on_player_entered(player: Node) -> void:
	pass  # Handle player interaction

func _on_player_exited(player: Node) -> void:
	pass  # Handle player exit