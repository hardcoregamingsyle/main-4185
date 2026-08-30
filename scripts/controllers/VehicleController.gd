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
var _physics_body: CharacterBody2D = null
var _vehicle_mesh: MeshInstance2D = null

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
var redline_rpm: float = 7500.0
var gear_ratios: Array[float] = [0.0, 3.8, 2.9, 2.1, 1.6, 1.3, 1.0]
var final_drive_ratio: float = 3.5

# Nitrous system
var nitro_available: bool = true
var nitro_amount: float = 100.0
var nitro_cooldown: float = 0.0
const NITRO_BONUS: float = 1.5
const NITRO_CONSUMPTION_RATE: float = 50.0
const NITRO_MAX_AMOUNT: float = 100.0
const NITRO_COOLDOWN_TIME: float = 10.0

# Input handling
var throttle_input: float = 0.0
var brake_input: float = 0.0
var steering_input: float = 0.0
var shift_up_request: bool = false
var shift_down_request: bool = false

# Physics settings reference
var _settings: PhysicsSettings = null

# Constants for vehicle dynamics
const FRICTION_COEFFICIENT: float = 0.8
const AIR_RESISTANCE: float = 0.0001
const MAX_STEERING_ANGLE: float = PI / 3  # 60 degrees
const STEERING_SENSITIVITY: float = 0.5
const ACCELERATION_RATE: float = 5.0
const BRAKING_RATE: float = 8.0
const INERTIA_FACTOR: float = 0.95

func _ready() -> void:
	_init_references()
	_load_settings()
	_setup_signals()
	_reset_vehicle_state()

func _init_references() -> void:
	_physics_body = get_parent() as CharacterBody2D
	if _physics_body == null:
		push_warning("VehicleController: No CharacterBody2D parent found")
	
	# Find mesh if present
	var parent = get_parent()
	while parent != null:
		if "MeshInstance2D" in str(parent.get_class()):
			_vehicle_mesh = parent as MeshInstance2D
			break
		parent = parent.get_parent()

func _load_settings() -> void:
	_settings = PhysicsSettings.new()
	max_rpm = _settings.default_max_rpm if _settings.has_method("get_default_max_rpm") else 8000.0
	idle_rpm = _settings.default_idle_rpm if _settings.has_method("get_default_idle_rpm") else 800.0

func _setup_signals() -> void:
	# Connect to powertrain signals if available
	if powertrain:
		powertrain.engine_started.connect(_on_powertrain_engine_started)
		powertrain.engine_stopped.connect(_on_powertrain_engine_stopped)

func _reset_vehicle_state() -> void:
	current_vehicle_state = VehicleState.IDLE
	current_gear = 0
	rpm = idle_rpm
	current_speed = 0.0
	nitro_available = true
	nitro_amount = NITRO_MAX_AMOUNT
	total_distance_traveled = 0.0
	last_collision_time = 0.0

# ============================================================================
# INPUT HANDLING
# ============================================================================

func _input(event: InputEvent) -> void:
	# Throttle control
	if event.is_action_pressed("ui_accept"):
		throttle_input = min(throttle_input + 0.1, 1.0)
	elif event.is_action_released("ui_accept"):
		throttle_input = max(throttle_input - 0.1, 0.0)
	
	# Brake control
	if event.is_action_pressed("ui_cancel"):
		brake_input = min(brake_input + 0.1, 1.0)
	elif event.is_action_released("ui_cancel"):
		brake_input = max(brake_input - 0.1, 0.0)
	
	# Steering control
	if event.is_action_pressed("ui_left"):
		steering_input = max(steering_input - STEERING_SENSITIVITY, -1.0)
	elif event.is_action_released("ui_left"):
		steering_input = min(steering_input + STEERING_SENSITIVITY, 1.0)
	
	if event.is_action_pressed("ui_right"):
		steering_input = min(steering_input + STEERING_SENSITIVITY, 1.0)
	elif event.is_action_released("ui_right"):
		steering_input = max(steering_input - STEERING_SENSITIVITY, -1.0)
	
	# Gear shifting
	if event.is_action_pressed("gear_shift_up"):
		shift_up_request = true
	if event.is_action_pressed("gear_shift_down"):
		shift_down_request = true
	
	# Nitrous use
	if event.is_action_pressed("nitro") and nitro_available:
		_use_nitrous()

func _process(delta: float) -> void:
	_update_inputs(delta)
	_handle_gear_shifting()
	_calculate_physics(delta)
	_update_vehicle_state()

func _update_inputs(delta: float) -> void:
	# Smooth throttle decay
	if throttle_input > 0.0:
		throttle_input = clampf(throttle_input - delta * 0.5, 0.0, 1.0)
	
	# Smooth brake decay
	if brake_input > 0.0:
		brake_input = clampf(brake_input - delta * 0.5, 0.0, 1.0)
	
	# Smooth steering return to center
	if abs(steering_input) > 0.01:
		steering_input = sign(steering_input) * max(0.0, abs(steering_input) - delta * 0.3)
	else:
		steering_input = 0.0

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================

func _handle_gear_shifting() -> void:
	if shift_up_request and current_gear < 6:
		shift_gear_up()
		shift_up_request = false
	
	if shift_down_request and current_gear > 0:
		shift_gear_down()
		shift_down_request = false

func shift_gear_up() -> void:
	if current_gear >= 6:
		return
	
	var old_gear = current_gear
	current_gear += 1
	
	# RPM drop when upshifting
	var rpm_drop_factor = gear_ratios[current_gear] / gear_ratios[old_gear]
	rpm = max(idle_rpm, rpm * rpm_drop_factor)
	
	gear_changed.emit(old_gear, current_gear)
	_on_gear_changed(old_gear, current_gear)

func shift_gear_down() -> void:
	if current_gear <= 0:
		return
	
	var old_gear = current_gear
	current_gear -= 1
	
	# RPM spike when downshifting
	var rpm_increase_factor = gear_ratios[old_gear] / gear_ratios[current_gear]
	rpm = min(max_rpm, rpm * rpm_increase_factor)
	
	gear_changed.emit(old_gear, current_gear)
	_on_gear_changed(old_gear, current_gear)

func _on_gear_changed(old_gear: int, new_gear: int) -> void:
	if powertrain:
		powertrain.set_gear(new_gear)
	
	# Adjust target speed based on gear
	var max_speed_for_gear = _calculate_max_speed_for_gear(new_gear)
	target_speed = max_speed_for_gear

func _calculate_max_speed_for_gear(gear: int) -> float:
	if gear == 0:
		return 0.0
	if gear == -1:  # Reverse
		return 40.0  # Max reverse speed
	return 120.0 / gear_ratios[gear] * final_drive_ratio

# ============================================================================
# PHYSICS CALCULATION
# ============================================================================

func _calculate_physics(delta: float) -> void:
	if _physics_body == null or current_vehicle_state == VehicleState.COLLIDED:
		return
	
	# Calculate RPM based on speed and gear
	_update_rpm(delta)
	
	# Apply forces based on input and state
	_apply_throttle_force(delta)
	_apply_brake_force(delta)
	_apply_steering(delta)
	
	# Apply air resistance and friction
	_apply_resistance(delta)
	
	# Update velocity
	_update_velocity(delta)
	
	# Update position
	_update_position(delta)
	
	# Emit signals
	_emit_vehicle_signals()

func _update_rpm(delta: float) -> void:
	var gear_ratio = gear_ratios[current_gear] if current_gear != 0 else 1.0
	var theoretical_rpm = (current_speed / 100.0) * gear_ratio * final_drive_ratio * 100.0
	
	# Smooth RPM changes
	var rpm_target = theoretical_rpm
	if throttle_input > 0.0 and current_gear != 0:
		rpm_target = max(rpm_target, idle_rpm + throttle_input * (max_rpm - idle_rpm))
	
	rpm = lerp(rpm, rpm_target, delta * 10.0)
	rpm = clampf(rpm, idle_rpm, max_rpm)

func _apply_throttle_force(delta: float) -> void:
	if throttle_input <= 0.0 or current_gear == 0:
		return
	
	var gear_ratio = gear_ratios[current_gear]
	var torque_factor = (rpm - idle_rpm) / (max_rpm - idle_rpm)
	torque_factor = clampf(torque_factor, 0.0, 1.0)
	
	var force = throttle_input * torque_factor * _settings.default_vehicle_mass * ACCELERATION_RATE
	force *= gear_ratio / final_drive_ratio
	
	acceleration = force
	velocity_vector.x += force * delta / _settings.default_vehicle_mass

func _apply_brake_force(delta: float) -> void:
	if brake_input <= 0.0:
		return
	
	var braking_force_val = brake_input * _settings.default_vehicle_mass * BRAKING_RATE
	velocity_vector.x -= braking_force_val * delta / _settings.default_vehicle_mass
	velocity_vector.x = clampf(velocity_vector.x, 0.0, Float.MAX)

func _apply_steering(delta: float) -> void:
	if current_speed < 1.0:
		return
	
	steering_angle = steering_input * MAX_STEERING_ANGLE
	var turn_rate = steering_angle * current_speed * 0.1
	
	if _physics_body:
		_physics_body.rotation = turn_rate * delta

func _apply_resistance(delta: float) -> void:
	# Air resistance (proportional to square of speed)
	var air_drag = velocity_vector.length() * velocity_vector.length() * AIR_RESISTANCE
	velocity_vector.x -= air_drag * delta
	
	# Rolling friction
	var friction = velocity_vector.x * FRICTION_COEFFICIENT * delta
	velocity_vector.x -= friction

func _update_velocity(delta: float) -> void:
	current_speed = abs(velocity_vector.x)
	current_speed = clampf(current_speed, 0.0, _calculate_top_speed())
	target_speed = current_speed

func _calculate_top_speed() -> float:
	if current_gear == 0:
		return 0.0
	var gear_ratio = gear_ratios[current_gear]
	var max_wheel_rpm = max_rpm / gear_ratio / final_drive_ratio
	return max_wheel_rpm * 100.0

func _update_position(delta: float) -> void:
	if _physics_body:
		_physics_body.position.x += velocity_vector.x * delta
		
		var distance_moved = abs(velocity_vector.x * delta)
		total_distance_traveled += distance_moved

# ============================================================================
# VEHICLE STATE MANAGEMENT
# ============================================================================

func _update_vehicle_state() -> void:
	match current_vehicle_state:
		VehicleState.IDLE:
			_update_idle_state()
		VehicleState.RUNNING:
			_update_running_state()
		VehicleState.BRAKING:
			_update_braking_state()
		VehicleState.DRIFTING:
			_update_drifting_state()
		VehicleState.COLLIDED:
			_update_collided_state()

func _update_idle_state() -> void:
	if throttle_input > 0.1 or brake_input > 0.1:
		current_vehicle_state = VehicleState.RUNNING
		engine_started.emit()

func _update_running_state() -> void:
	if brake_input > 0.5:
		current_vehicle_state = VehicleState.BRAKING
	elif throttle_input < 0.1 and current_speed < 1.0:
		current_vehicle_state = VehicleState.IDLE

func _update_braking_state() -> void:
	if brake_input < 0.3 and current_speed < 5.0:
		current_vehicle_state = VehicleState.RUNNING
	elif current_speed < 1.0:
		current_vehicle_state = VehicleState.IDLE

func _update_drifting_state() -> void:
	# Drift mechanics would go here
	pass

func _update_collided_state() -> void:
	var collision_age = Time.get_unix_time_from_system() - last_collision_time
	if collision_age > 2.0:
		current_vehicle_state = VehicleState.IDLE
		collision_damping = 0.3

# ============================================================================
# NITROUS SYSTEM
# ============================================================================

func _use_nitrous() -> void:
	if not nitro_available or nitro_amount <= 0.0:
		return
	
	nitro_available = false
	nitro_cooldown = NITRO_COOLDOWN_TIME
	
	# Apply nitro bonus
	var original_speed = current_speed
	var boosted_speed = current_speed * NITRO_BONUS
	
	current_speed = boosted_speed
	velocity_vector.x *= NITRO_BONUS
	
	nitro_amount -= NITRO_CONSUMPTION_RATE
	nitro_amount = max(0.0, nitro_amount)
	
	nitro_used.emit(NITRO_CONSUMPTION_RATE)
	
	# Cooldown timer
	await get_tree().create_timer(NITRO_COOLDOWN_TIME).timeout
	nitro_cooldown = 0.0
	nitro_available = true
	nitro_amount = min(nitro_amount + NITRO_CONSUMPTION_RATE, NITRO_MAX_AMOUNT)

func _process_nitro_cooldown(delta: float) -> void:
	if nitro_cooldown > 0.0:
		nitro_cooldown -= delta
		if nitro_cooldown < 0.0:
			nitro_cooldown = 0.0
			nitro_available = true

# ============================================================================
# COLLISION HANDLING
# ============================================================================

func handle_collision(direction: Vector2) -> void:
	last_collision_time = Time.get_unix_time_from_system()
	current_vehicle_state = VehicleState.COLLIDED
	
	# Dampen velocity on collision
	velocity_vector.x *= collision_damping
	current_speed = abs(velocity_vector.x)
	
	collision_damping = clampf(collision_damping - 0.05, 0.1, 0.5)
	collision_detected.emit(direction)

func reset_collision_state() -> void:
	collision_damping = 0.3
	current_vehicle_state = VehicleState.IDLE

# ============================================================================
# UTILITY METHODS
# ============================================================================

func set_speed(speed: float) -> void:
	current_speed = speed
	velocity_vector.x = speed

func set_gear_directly(gear: int) -> void:
	if gear >= 0 and gear <= 6:
		var old_gear = current_gear
		current_gear = gear
		gear_changed.emit(old_gear, current_gear)
		if powertrain:
			powertrain.set_gear(gear)
	elif gear == -1:  # Reverse
		var old_gear = current_gear
		current_gear = -1
		gear_changed.emit(old_gear, current_gear)
		if powertrain:
			powertrain.set_gear(-1)

func reset_all() -> void:
	_reset_vehicle_state()
	set_speed(0.0)
	velocity_vector = Vector2.ZERO
	reset_collision_state()

func _emit_vehicle_signals() -> void:
	speed_changed.emit(current_speed, _calculate_top_speed())
	rpm_changed.emit(rpm, max_rpm)
	
	if total_distance_traveled > 0:
		vehicle_moved.emit(total_distance_traveled)

# ============================================================================
# POWERTRAIN SIGNAL HANDLERS
# ============================================================================

func _on_powertrain_engine_started() -> void:
	current_vehicle_state = VehicleState.RUNNING

func _on_powertrain_engine_stopped() -> void:
	current_vehicle_state = VehicleState.IDLE

# ============================================================================
# DEBUG & TESTING
# ============================================================================

func debug_set_rpm(new_rpm: float) -> void:
	rpm = clampf(new_rpm, idle_rpm, max_rpm)
	rpm_changed.emit(rpm, max_rpm)

func debug_set_speed(new_speed: float) -> void:
	current_speed = new_speed
	velocity_vector.x = new_speed
	speed_changed.emit(current_speed, _calculate_top_speed())

func debug_set_gear(new_gear: int) -> void:
	var old_gear = current_gear
	current_gear = new_gear
	gear_changed.emit(old_gear, new_gear)
	if powertrain:
		powertrain.set_gear(new_gear)

</FILE>