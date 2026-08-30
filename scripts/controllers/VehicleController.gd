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
var clutch_input: float = 0.0

# Constants from PhysicsSettings
var _max_throttle_force: float = 5000.0
var _max_brake_force: float = 8000.0
var _max_steering_angle: float = PI / 4  # 45 degrees
var _engine_power_curve: Array[float] = []

# Time tracking for distance calculation
var _last_position: Vector2 = Vector2.ZERO
var _update_timer: float = 0.0
const UPDATE_INTERVAL: float = 0.016  # ~60 FPS

func _ready() -> void:
	_process_mode = ProcessModeEnum.ALWAYS
	_init_constants()
	_init_gear_ratios()
	_setup_physics_body()
	print("VehicleController initialized successfully")

func _init_constants() -> void:
	"""Initialize physics constants from PhysicsSettings resource"""
	if Engine.has_singleton("PhysicsSettings"):
		var settings = PhysicsSettings.new()
		_max_throttle_force = settings.default_vehicle_mass * 3.33
		_max_brake_force = settings.default_vehicle_mass * 5.33
	else:
		_max_throttle_force = 5000.0
		_max_brake_force = 8000.0
	
	# Build engine power curve (simplified torque curve)
	for i in range(10):
		var ratio = float(i) / 9.0
		_engine_power_curve.append(ratio * ratio * (4.0 - 5.0 * ratio + 2.0 * ratio * ratio))

func _init_gear_ratios() -> void:
	"""Initialize gear ratios for transmission simulation"""
	if powertrain:
		powertrain.set_gear_ratios([0.0, 3.5, 2.5, 1.8, 1.4, 1.1, 0.9, -3.0])  # Neutral + gears + reverse

func _setup_physics_body() -> void:
	"""Find or create physics body for movement"""
	if get_parent():
		_physics_body = get_parent().get_node_or_null("CharacterBody2D")
	
	if not _physics_body:
		_physics_body = CharacterBody2D.new()
		_physics_body.name = "CharacterBody2D"
		add_child(_physics_body)

func _process(delta: float) -> void:
	"""Main update loop for vehicle physics"""
	_update_timer += delta
	
	if _update_timer >= UPDATE_INTERVAL:
		_update_timer = 0.0
		_process_physics(delta)
	
	_handle_nitro(delta)
	_check_vehicle_state()

func _process_physics(delta: float) -> void:
	"""Process vehicle physics calculations"""
	_calculate_rpm()
	_calculate_target_speed()
	_apply_forces(delta)
	_update_movement(delta)

func _calculate_rpm() -> void:
	"""Calculate current engine RPM based on gear and speed"""
	if current_gear == 0:
		rpm = lerp(rpm, idle_rpm, 0.1)
		return
	
	var gear_ratio: float = 1.0
	if powertrain:
		gear_ratio = powertrain.get_gear_ratio(current_gear)
	
	# RPM proportional to speed divided by gear ratio
	var base_rpm = (current_speed * 10.0) / max(gear_ratio, 0.1)
	rpm = clamp(base_rpm, idle_rpm, max_rpm)
	
	emit_signal("rpm_changed", rpm, max_rpm)

func _calculate_target_speed() -> void:
	"""Calculate target speed based on gear and input"""
	var max_speed_per_gear: Dictionary = {
		0: 0.0,
		1: 30.0,
		2: 60.0,
		3: 100.0,
		4: 140.0,
		5: 180.0,
		6: 220.0,
		-1: -40.0
	}
	
	var gear_max_speed: float = max_speed_per_gear.get(current_gear, 0.0)
	
	if current_gear == 0:
		target_speed = 0.0
	elif throttle_input > 0:
		target_speed = gear_max_speed * throttle_input
	elif brake_input > 0:
		target_speed = 0.0
	else:
		target_speed = lerp(target_speed, 0.0, 0.05)

func _apply_forces(delta: float) -> void:
	"""Apply forces to vehicle based on inputs"""
	if not _physics_body:
		return
	
	# Calculate throttle force
	var throttle_force: float = 0.0
	if current_gear != 0 and throttle_input > 0:
		var power_factor = _get_power_factor()
		throttle_force = _max_throttle_force * throttle_input * power_factor
		current_vehicle_state = VehicleState.REVVING if rpm > 4000 else VehicleState.RUNNING
	
	# Calculate braking force
	braking_force = 0.0
	if brake_input > 0:
		braking_force = _max_brake_force * brake_input
		current_vehicle_state = VehicleState.BRAKING
	
	# Apply velocity changes
	var force_direction: float = sign(throttle_force - braking_force)
	acceleration = (throttle_force - braking_force) / PhysicsSettings.default_vehicle_mass
	
	var velocity_change: float = acceleration * delta
	current_speed += velocity_change
	
	# Clamp speed to gear limits
	var gear_limit: float = [0.0, 30.0, 60.0, 100.0, 140.0, 180.0, 220.0][-1 if current_gear < 0 else current_gear]
	if abs(current_speed) > abs(gear_limit):
		current_speed = sign(current_speed) * abs(gear_limit) * 0.99
	
	# Apply friction
	current_speed *= 0.995  # Air resistance
	
	# Apply to physics body
	var move_vector: Vector2 = Vector2.RIGHT.rotated(get_rotation())
	move_vector = move_vector * current_speed
	
	if _physics_body is CharacterBody2D:
		_physics_body.velocity = move_vector
		_physics_body.move_and_slide()

func _get_power_factor() -> float:
	"""Get engine power factor based on RPM"""
	var rpm_ratio: float = (rpm - idle_rpm) / (max_rpm - idle_rpm)
	return clamp(rpm_ratio, 0.0, 1.0)

func _update_movement(delta: float) -> void:
	"""Update vehicle position and track distance"""
	if not _physics_body:
		return
	
	var displacement: Vector2 = _physics_body.position - _last_position
	var distance_moved: float = displacement.length()
	
	if distance_moved > 0:
		total_distance_traveled += distance_moved
		emit_signal("vehicle_moved", distance_moved)
	
	_last_position = _physics_body.position
	
	# Emit speed signal
	emit_signal("speed_changed", current_speed, 
		[0.0, 30.0, 60.0, 100.0, 140.0, 180.0, 220.0][-1 if current_gear < 0 else current_gear])

func _handle_nitro(delta: float) -> void:
	"""Handle nitrous oxide system"""
	nitro_cooldown -= delta
	
	if nitro_cooldown < 0:
		nitro_cooldown = 0.0
	
	# Check for nitro activation input (left Shift)
	if Input.is_action_just_pressed("nitro_activate") and nitro_available:
		_use_nitro()

func _use_nitro() -> void:
	"""Activate nitrous oxide boost"""
	if nitro_amount <= 0 or nitro_cooldown > 0:
		return
	
	nitro_amount -= NITRO_CONSUMPTION_RATE
	nitro_cooldown = 5.0  # 5 second cooldown
	
	if nitro_amount <= 0:
		nitro_available = false
	
	current_speed *= NITRO_BONUS
	emit_signal("nitro_used", NITRO_CONSUMPTION_RATE)
	
	# Visual/audio effects would go here

func shift_gear(direction: int) -> bool:
	"""Shift gears up or down"""
	var old_gear: int = current_gear
	var new_gear: int = current_gear + direction
	
	# Validate gear shift
	if new_gear < -1 or new_gear > 6:
		return false
	
	# Prevent shifting into neutral while moving
	if new_gear == 0 and abs(current_speed) > 5.0:
		return false
	
	# Auto-reverse at low speeds
	if new_gear == -1 and current_speed > 0:
		new_gear = 1
	elif new_gear == 1 and current_speed < 0:
		new_gear = -1
	
	current_gear = new_gear
	emit_signal("gear_changed", old_gear, new_gear)
	
	if powertrain:
		powertrain.shift_gear(new_gear)
	
	return true

func set_throttle(value: float) -> void:
	"""Set throttle input value (-1 to 1)"""
	throttle_input = clamp(value, -1.0, 1.0)

func set_brake(value: float) -> void:
	"""Set brake input value (0 to 1)"""
	brake_input = clamp(value, 0.0, 1.0)

func set_steering(value: float) -> void:
	"""Set steering input value (-1 to 1)"""
	steering_input = clamp(value, -1.0, 1.0)
	steering_angle = steering_input * _max_steering_angle
	if chassis:
		chassis.set_steering_angle(steering_angle)

func start_engine() -> void:
	"""Start the vehicle engine"""
	if rpm > 0:
		return
	
	rpm = idle_rpm
	current_vehicle_state = VehicleState.IDLE
	emit_signal("engine_started")
	
	if AudioManager:
		AudioManager.play_sfx("engine_start")

func stop_engine() -> void:
	"""Stop the vehicle engine"""
	rpm = 0.0
	current_gear = 0
	current_speed = 0.0
	current_vehicle_state = VehicleState.IDLE
	emit_signal("engine_stopped")
	
	if AudioManager:
		AudioManager.play_sfx("engine_stop")

func reset_vehicle() -> void:
	"""Reset vehicle to initial state"""
	current_speed = 0.0
	target_speed = 0.0
	rpm = idle_rpm
	current_gear = 0
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	nitro_available = true
	nitro_amount = 100.0
	nitro_cooldown = 0.0
	total_distance_traveled = 0.0
	current_vehicle_state = VehicleState.IDLE
	_last_position = Vector2.ZERO
	
	if _physics_body:
		_physics_body.velocity = Vector2.ZERO

func take_damage(damage_amount: float) -> void:
	"""Handle vehicle damage from collisions"""
	last_collision_time = Time.get_ticks_msec() / 1000.0
	current_vehicle_state = VehicleState.COLLIDED
	
	# Reduce speed on impact
	current_speed *= 0.7
	
	# Play damage sound
	if AudioManager:
		AudioManager.play_sfx("damage")
	
	# Screen shake effect
	if get_parent() and get_parent() is Node2D:
		get_parent().set_position(get_parent().position + Vector2(randf_range(-5, 5), randf_range(-5, 5)))

func _check_vehicle_state() -> void:
	"""Check and update vehicle state machine"""
	var time_since_collision: float = Time.get_ticks_msec() / 1000.0 - last_collision_time
	
	if time_since_collision < 0.3 and current_vehicle_state == VehicleState.COLLIDED:
		current_vehicle_state = VehicleState.COLLIDED
	elif current_vehicle_state == VehicleState.COLLIDED:
		current_vehicle_state = VehicleState.RUNNING

func _input(event: InputEvent) -> void:
	"""Handle direct input events"""
	if event is InputEventKey:
		match event.keycode:
			KEY_UP, KEY_W:
				set_throttle(1.0)
			KEY_DOWN, KEY_S:
				set_throttle(-0.5)
			KEY_LEFT, KEY_A:
				set_steering(-1.0)
			KEY_RIGHT, KEY_D:
				set_steering(1.0)
			KEY_SPACE:
				set_brake(1.0)
			KEY_SHIFT_L, KEY_SHIFT_R:
				_use_nitro()
			KEY_G:
				shift_gear(1)
			KEY_F:
				shift_gear(-1)
			KEY_E:
				if rpm > 0:
					stop_engine()
				else:
					start_engine()

func get_stats() -> Dictionary:
	"""Get current vehicle statistics"""
	return {
		"speed": current_speed,
		"max_speed": [0.0, 30.0, 60.0, 100.0, 140.0, 180.0, 220.0][-1 if current_gear < 0 else current_gear],
		"rpm": rpm,
		"max_rpm": max_rpm,
		"gear": current_gear,
		"distance": total_distance_traveled,
		"state": current_vehicle_state,
		"nitro": nitro_amount,
		"throttle": throttle_input,
		"brake": brake_input,
		"steering": steering_input
	}

func debug_print_status() -> void:
	"""Print current vehicle status to console (debug only)"""
	print("=== VEHICLE STATUS ===")
	print("Speed: %.2f km/h" % current_speed)
	print("Gear: %d" % current_gear)
	print("RPM: %.0f / %.0f" % [rpm, max_rpm])
	print("Throttle: %.2f" % throttle_input)
	print("Brake: %.2f" % brake_input)
	print("Steering: %.2f" % steering_input)
	print("Distance: %.2f m" % total_distance_traveled)
	print("Nitro: %.1f%%" % (nitro_amount / 100.0 * 100))
	print("=====================")