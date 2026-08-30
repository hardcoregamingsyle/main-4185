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
var nitro_cooldown_timer: float = 0.0
var nitro_usage_timer: float = 0.0
const NITRO_COOLDOWN: float = 5.0
const NITRO_BONUS_MULTIPLIER: float = 1.5

# Driving configuration
var driving_mode: String = "normal"  # normal, sport, drift, rally
var traction_control: bool = true
var abs_enabled: bool = true

# Vehicle reference
var vehicle_reference: Node = null

# Physics settings reference
var _physics_settings: PhysicsSettings = null

func _ready() -> void:
	_init_physics_settings()
	_setup_references()
	_connect_signals()
	_reset_vehicle_state()

func _init_physics_settings() -> void:
	if GameManager.has_singleton("PhysicsSettings"):
		_physics_settings = GameManager.get_singleton("PhysicsSettings") as PhysicsSettings
	else:
		var ps = preload("res://scripts/core/PhysicsSettings.gd").new()
		ps.load_file("res://scripts/core/PhysicsSettings.gd")
		_physics_settings = ps

func _setup_references() -> void:
	_find_physics_body()
	_find_powertrain()
	_find_chassis()

func _find_physics_body() -> void:
	for child in get_children():
		if child is CharacterBody2D or child is RigidBody2D:
			_physics_body = child
			break

func _find_powertrain() -> void:
	powertrain = $Powertrain if get_node_or_null("Powertrain") else null

func _find_chassis() -> void:
	chassis = $Chassis if get_node_or_null("Chassis") else null

func _connect_signals() -> void:
	if powertrain:
		powertrain.engine_started.connect(_on_engine_started)
		powertrain.engine_stopped.connect(_on_engine_stopped)
		powertrain.rpm_changed.connect(_on_powertrain_rpm_changed)

func _reset_vehicle_state() -> void:
	current_vehicle_state = VehicleState.IDLE
	current_speed = 0.0
	target_speed = 0.0
	acceleration = 0.0
	braking_force = 0.0
	steering_angle = 0.0
	current_gear = 0
	rpm = idle_rpm
	nitro_available = true
	nitro_amount = 100.0
	total_distance_traveled = 0.0

func _process(delta: float) -> void:
	_handle_nitro_cooldown(delta)
	_handle_input(delta)
	_update_vehicle_state()
	_emit_signals()

func _physics_process(delta: float) -> void:
	apply_physics(delta)

func _handle_nitro_cooldown(delta: float) -> void:
	if nitro_available:
		return
	
	nitro_cooldown_timer -= delta
	if nitro_cooldown_timer <= 0:
		nitro_available = true
		nitro_cooldown_timer = 0.0

func _handle_input(delta: float) -> void:
	var input = InputManager.get_vehicle_inputs()
	
	# Update RPM based on input
	_update_rpm(input)
	
	# Handle throttle
	if input.throttle > 0:
		_handle_throttle(input.throttle, delta)
	elif input.brake > 0:
		_handle_brake(input.brake, delta)
	else:
		_handle_coasting(delta)
	
	# Handle steering
	_handle_steering(input.steer, delta)
	
	# Handle gear shifting
	_handle_gear_shifting(input.shift_up, input.shift_down, delta)
	
	# Handle nitro usage
	if input.nitro && nitro_available && powertrain:
		_activate_nitro(delta)

func _update_rpm(input: Dictionary) -> void:
	var base_rpm_change: float = 0.0
	
	if input.throttle > 0:
		base_rpm_change = input.throttle * 500.0
	elif input.brake > 0:
		base_rpm_change = -input.brake * 300.0
	else:
		base_rpm_change = -idle_rpm * 0.1
	
	# Apply gear multiplier
	var gear_multiplier: float = 1.0
	match current_gear:
		1: gear_multiplier = 0.5
		2: gear_multiplier = 0.7
		3: gear_multiplier = 0.9
		4: gear_multiplier = 1.1
		5: gear_multiplier = 1.3
		6: gear_multiplier = 1.5
		-1: gear_multiplier = 0.6
		_: gear_multiplier = 1.0
	
	rpm += base_rpm_change * gear_multiplier
	rpm = clamp(rpm, idle_rpm, max_rpm)

func _handle_throttle(throttle_input: float, delta: float) -> void:
	if current_vehicle_state == VehicleState.BRAKING:
		current_vehicle_state = VehicleState.RUNNING
	
	var max_acceleration: float = _get_max_acceleration_for_gear()
	var acceleration_increment: float = max_acceleration * throttle_input * delta
	
	acceleration += acceleration_increment
	acceleration = min(acceleration, max_acceleration)
	
	target_speed += acceleration * delta
	target_speed = min(target_speed, _get_max_speed_for_gear())

func _handle_brake(brake_input: float, delta: float) -> void:
	current_vehicle_state = VehicleState.BRAKING
	
	var max_deceleration: float = 15.0 * _get_deceleration_modifier()
	var deceleration_increment: float = max_deceleration * brake_input * delta
	
	if current_speed > 0:
		target_speed -= deceleration_increment * delta
		target_speed = max(target_speed, 0.0)
		braking_force = deceleration_increment
	else:
		braking_force = 0.0

func _handle_coasting(delta: float) -> void:
	if current_vehicle_state == VehicleState.BRAKING:
		current_vehicle_state = VehicleState.RUNNING
	
	var friction: float = 0.5
	target_speed *= (1.0 - friction * delta)
	target_speed = max(target_speed, 0.0)

func _handle_steering(steer_input: float, delta: float) -> void:
	var max_steering_rate: float = 60.0 * (delta * 60.0)
	var current_max_steering: float = 30.0 * (_get_current_speed_factor())
	
	steering_angle += steer_input * max_steering_rate
	steering_angle = clamp(steering_angle, -current_max_steering, current_max_steering)

func _handle_gear_shifting(shift_up: bool, shift_down: bool, delta: float) -> void:
	if shift_up:
		shift_gear_forward()
	if shift_down:
		shift_gear_backward()

func shift_gear_forward() -> void:
	if current_gear < 6:
		var old_gear: int = current_gear
		current_gear += 1
		gear_changed.emit(old_gear, current_gear)
		_apply_gear_effect()

func shift_gear_backward() -> void:
	if current_gear > 0:
		var old_gear: int = current_gear
		current_gear -= 1
		gear_changed.emit(old_gear, current_gear)
		_apply_gear_effect()

func _apply_gear_effect() -> void:
	var gear_ratio: float = 1.0
	match current_gear:
		1: gear_ratio = 3.5
		2: gear_ratio = 2.5
		3: gear_ratio = 1.8
		4: gear_ratio = 1.3
		5: gear_ratio = 1.0
		6: gear_ratio = 0.8
		-1: gear_ratio = 3.8
		_: gear_ratio = 1.0
	
	# Adjust target speed based on gear ratio
	target_speed *= gear_ratio

func _activate_nitro(delta: float) -> void:
	if nitro_amount <= 0:
		return
	
	nitro_available = false
	nitro_cooldown_timer = NITRO_COOLDOWN
	nitro_usage_timer = 2.0
	
	var bonus_speed: float = _get_max_speed_for_gear() * (NITRO_BONUS_MULTIPLIER - 1.0)
	target_speed += bonus_speed * delta
	
	nitro_amount -= 5.0 * delta
	nitro_used.emit(nitro_amount)
	
	if powertrain:
		powertrain.activate_nitro()

func _update_vehicle_state() -> void:
	if current_speed < 1.0:
		current_vehicle_state = VehicleState.IDLE
	elif current_vehicle_state == VehicleState.BRAKING and current_speed > 5.0:
		current_vehicle_state = VehicleState.RUNNING
	elif rpm > max_rpm * 0.9:
		current_vehicle_state = VehicleState.REVVING
	elif _check_collision_recently():
		current_vehicle_state = VehicleState.COLLIDED

func _emit_signals() -> void:
	speed_changed.emit(current_speed, _get_max_speed_for_gear())
	rpm_changed.emit(rpm, max_rpm)
	
	var speed_diff: float = abs(target_speed - current_speed)
	if speed_diff > 0.1:
		current_speed = lerp(current_speed, target_speed, 0.1)
		total_distance_traveled += current_speed * 0.016

func _check_collision_recently() -> bool:
	return Time.get_ticks_msec() - last_collision_time < 500

func apply_physics(delta: float) -> void:
	if _physics_body == null:
		return
	
	# Apply calculated physics values to body
	if _physics_body is CharacterBody2D:
		var velocity: Vector2 = _physics_body.velocity
		velocity.x = current_speed * cos(global_rotation)
		velocity.y = current_speed * sin(global_rotation)
		
		var drag_coefficient: float = 0.01
		velocity *= (1.0 - drag_coefficient * delta)
		
		_physics_body.velocity = velocity
		
		if abs(velocity.length()) > 0.1:
			_physics_body.move_and_slide()

func _get_max_acceleration_for_gear() -> float:
	var base_acceleration: float = 10.0
	var gear_multipliers: Array[float] = [0.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0]
	return base_acceleration * gear_multipliers[current_gear] if current_gear > 0 else 0.0

func _get_max_speed_for_gear() -> float:
	var base_top_speed: float = 200.0
	var gear_ratios: Array[float] = [0.0, 40.0, 70.0, 100.0, 130.0, 160.0, 190.0]
	return base_top_speed * gear_ratios[current_gear] / 10.0 if current_gear > 0 else 0.0

func _get_deceleration_modifier() -> float:
	return 1.0 + (current_gear * 0.1) if current_gear > 0 else 0.5

func _get_current_speed_factor() -> float:
	return min(1.0, current_speed / 50.0)

func _on_engine_started() -> void:
	current_vehicle_state = VehicleState.RUNNING
	engine_started.emit()

func _on_engine_stopped() -> void:
	current_vehicle_state = VehicleState.IDLE
	engine_stopped.emit()

func _on_powertrain_rpm_changed(new_rpm: float) -> void:
	rpm = new_rpm
	rpm_changed.emit(rpm, max_rpm)

func set_driving_mode(mode: String) -> void:
	driving_mode = mode
	match mode:
		"drift":
			traction_control = false
			collision_damping = 0.5
		"rally":
			traction_control = true
			collision_damping = 0.2
		"sport":
			traction_control = true
			collision_damping = 0.3
		_:
			traction_control = true
			collision_damping = 0.3

func reset_nitro() -> void:
	nitro_available = true
	nitro_amount = 100.0
	nitro_cooldown_timer = 0.0

func get_vehicle_status() -> Dictionary:
	return {
		"current_speed": current_speed,
		"target_speed": target_speed,
		"current_gear": current_gear,
		"rpm": rpm,
		"max_rpm": max_rpm,
		"nitro_available": nitro_available,
		"nitro_amount": nitro_amount,
		"total_distance": total_distance_traveled,
		"vehicle_state": current_vehicle_state,
		"driving_mode": driving_mode
	}

func debug_print_status() -> void:
	print("Vehicle Status:")
	print("  Speed: %.2f km/h" % current_speed)
	print("  Gear: %d" % current_gear)
	print("  RPM: %.0f / %.0f" % [rpm, max_rpm])
	print("  Nitro: %.0f%%" % nitro_amount)
	print("  Distance: %.2f m" % total_distance_traveled)

func _on_collision_detected(collision: CollisionShape2D) -> void:
	last_collision_time = Time.get_ticks_msec()
	collision_detected.emit(Vector2.RIGHT)
	
	if current_speed > 20.0:
		target_speed *= 0.7
		current_speed *= 0.7

func set_vehicle_reference(vehicle: Node) -> void:
	vehicle_reference = vehicle

func start_engine() -> void:
	if powertrain:
		powertrain.start_engine()
	else:
		current_vehicle_state = VehicleState.RUNNING

func stop_engine() -> void:
	if powertrain:
		powertrain.stop_engine()
	else:
		current_vehicle_state = VehicleState.IDLE

func restart_vehicle() -> void:
	_reset_vehicle_state()
	start_engine()
	set_driving_mode("normal")
	reset_nitro()