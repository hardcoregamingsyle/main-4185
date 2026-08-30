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

# References
@onready var powertrain: Powertrain = $Powertrain if $Powertrain else null
@onready var chassis: Chassis = $Chassis if $Chassis else null
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

# Gear management
var current_gear: int = 0  # 0 = neutral, 1-6 = forward gears, -1 = reverse
var rpm: float = 0.0
var max_rpm: float = 8000.0
var idle_rpm: float = 800.0

# Nitrous system
var nitro_available: bool = true
var nitro_amount: float = 100.0
var nitro_cooldown: float = 0.0

# Input handling
var throttle_input: float = 0.0
var brake_input: float = 0.0
var steering_input: float = 0.0
var shift_up_request: bool = false
var shift_down_request: bool = false

# Physics settings reference
var _settings: PhysicsSettings = PhysicsSettings.new()

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
	try:
		var settings_path = preload("res://scripts/core/PhysicsSettings.gd")
		if settings_path:
			_settings = settings_path.new()
	except:
		push_warning("Failed to load PhysicsSettings, using defaults")
		_settings = _create_default_settings()

func _create_default_settings() -> PhysicsSettings:
	var s = PhysicsSettings.new()
	s.gravity = 9.81
	s.physics_tick_rate = 120
	s.max_substeps = 4
	return s

func _init_physics_body() -> void:
	if get_parent() is CharacterBody2D:
		_physics_body = get_parent()
	elif get_parent():
		for child in get_parent().get_children():
			if child is CharacterBody2D:
				_physics_body = child
				break

func _setup_powertrain_connection() -> void:
	if powertrain:
		powertrain.engine_started.connect(_on_engine_started)
		powertrain.engine_stopped.connect(_on_engine_stopped)
		powertrain.rpm_changed.connect(_on_rpm_changed)

func _reset_vehicle() -> void:
	current_speed = 0.0
	target_speed = 0.0
	current_gear = 0
	rpm = idle_rpm
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	nitro_amount = 100.0
	nitro_cooldown = 0.0
	current_vehicle_state = VehicleState.IDLE

func _process(delta: float) -> void:
	if not Engine.is_editor_hint():
		_update_inputs(delta)
		_update_physics(delta)
		_update_gear_logic(delta)
		_update_nitro(delta)
		_update_state(delta)

func _update_inputs(delta: float) -> void:
	if InputManager:
		throttle_input = InputManager.get_axis("throttle", "brake")
		steering_input = InputManager.get_axis("steer_left", "steer_right")
	else:
		throttle_input = Input.get_action_strength("throttle") - Input.get_action_strength("brake")
		steering_input = Input.get_axis("steer_left", "steer_right")

	if Input.is_action_just_pressed("gear_up"):
		shift_up_request = true
	if Input.is_action_just_pressed("gear_down"):
		shift_down_request = true

func _update_physics(delta: float) -> void:
	if not _physics_body:
		return

	var dt = delta * _settings.time_scale
	
	# Calculate gear ratio
	var gear_ratio = _get_gear_ratio(current_gear)
	
	# Calculate torque based on RPM and gear
	var torque = _calculate_torque(rpm, current_gear)
	
	# Apply throttle force
	var drive_force = 0.0
	if current_gear != 0 and throttle_input > 0:
		drive_force = torque * gear_ratio * throttle_input * 0.5
		
		# Apply air resistance
		var drag_force = current_speed * current_speed * AIR_RESISTANCE
		drive_force -= drag_force
		
		# Apply friction
		var friction_force = current_speed * FRICTION_COEFFICIENT
		drive_force -= friction_force
	
	# Apply braking
	if brake_input > 0:
		braking_force = brake_input * 15.0 * _settings.default_vehicle_mass
	else:
		braking_force = 0.0
	
	# Calculate net force
	var net_force = drive_force - braking_force
	
	# Acceleration calculation
	acceleration = net_force / _settings.default_vehicle_mass
	
	# Update speed
	current_speed += acceleration * dt
	
	# Clamp speed
	current_speed = clampf(current_speed, -50.0, 200.0)
	
	# Apply to physics body
	if _physics_body.velocity:
		var direction = global_transform.basis.x
		_physics_body.velocity.x = current_speed * direction.x
		_physics_body.velocity.y = current_speed * direction.y
	
	# Update RPM based on speed and gear
	if current_gear != 0:
		rpm = (current_speed * GEAR_RATIOS[current_gear]) + idle_rpm
		rpm = clampf(rpm, idle_rpm, max_rpm)
	else:
		rpm = lerp(rpm, idle_rpm, 0.1)
	
	# Signal updates
	if _physics_body.velocity.length_squared() > 100:
		emit_signal("speed_changed", abs(current_speed), _settings.default_vehicle_mass * 0.1)
		emit_signal("rpm_changed", rpm, max_rpm)

func _calculate_torque(rpm_val: float, gear: int) -> float:
	# Simple torque curve simulation
	var normalized_rpm = (rpm_val - idle_rpm) / (max_rpm - idle_rpm)
	var base_torque = 250.0
	
	# Torque peaks around 50% RPM then drops
	if normalized_rpm < 0.5:
		return base_torque * (normalized_rpm * 2)
	else:
		return base_torque * (1.0 - (normalized_rpm - 0.5) * 1.5)

func _get_gear_ratio(gear: int) -> float:
	if gear == 0:
		return 0.0
	elif gear == -1:
		return REVERSE_GEAR_RATIO
	elif gear >= 1 and gear <= 6:
		return GEAR_RATIOS[gear]
	else:
		return 0.0

func _update_gear_logic(delta: float) -> void:
	# Automatic upshifting when approaching redline
	if rpm > max_rpm * 0.95 and current_gear < 6:
		_shift_gear(current_gear + 1)
	
	# Automatic downshifting when RPM too low
	if rpm < idle_rpm * 1.2 and current_gear > 1 and current_speed > MIN_SPEED_FOR_GEAR_SHIFT:
		_shift_gear(current_gear - 1)
	
	# Manual shift request handling
	if shift_up_request:
		if current_gear < 6:
			_shift_gear(current_gear + 1)
		shift_up_request = false
	if shift_down_request:
		if current_gear > 1 or current_gear == 0:
			_shift_gear(current_gear - 1)
		shift_down_request = false
	
	# Neutral handling
	if current_gear == 0:
		rpm = lerp(rpm, idle_rpm, 0.05)

func _shift_gear(new_gear: int) -> void:
	if new_gear == current_gear:
		return
	
	var old_gear = current_gear
	current_gear = new_gear
	
	# Simulate clutch engagement time
	await get_tree().create_timer(0.1).timeout
	
	emit_signal("gear_changed", old_gear, current_gear)

func _update_nitro(delta: float) -> void:
	if nitro_cooldown > 0:
		nitro_cooldown -= delta
		return
	
	if nitro_amount > 0 and throttle_input > 0.5 and nitro_available:
		if Input.is_action_just_pressed("nitro"):
			_use_nitro()

func _use_nitro() -> void:
	if nitro_amount <= 0 or nitro_cooldown > 0:
		return
	
	nitro_available = false
	nitro_cooldown = 5.0
	
	# Boost speed temporarily
	var boost_amount = nitro_amount * 0.5
	current_speed += boost_amount
	
	# Visual feedback
	if chassis:
		chassis.apply_boost_effect()
	
	emit_signal("nitro_used", nitro_amount)
	nitro_amount = 0.0

func _update_state(delta: float) -> void:
	# Check for collision state
	if Time.get_ticks_msec() - last_collision_time < 500:
		current_vehicle_state = VehicleState.COLLIDED
	else:
		if throttle_input > 0.1:
			current_vehicle_state = VehicleState.RUNNING
		elif rpm > idle_rpm * 1.5:
			current_vehicle_state = VehicleState.REVVING
		elif brake_input > 0.5:
			current_vehicle_state = VehicleState.BRAKING
		else:
			current_vehicle_state = VehicleState.IDLE

func _on_engine_started() -> void:
	current_vehicle_state = VehicleState.RUNNING
	rpm = idle_rpm

func _on_engine_stopped() -> void:
	current_vehicle_state = VehicleState.IDLE
	rpm = idle_rpm
	throttle_input = 0.0

func _on_rpm_changed(new_rpm: float) -> void:
	rpm = new_rpm
	emit_signal("rpm_changed", rpm, max_rpm)

func apply_collision_impact(direction: Vector2) -> void:
	last_collision_time = Time.get_ticks_msec()
	current_speed *= 0.5
	_physics_body.velocity = _physics_body.velocity * 0.5
	emit_signal("collision_detected", direction)

func reset_controls() -> void:
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	shift_up_request = false
	shift_down_request = false

func set_max_speed(speed: float) -> void:
	max_rpm = speed * 0.8
	_settings.default_vehicle_mass = 1500.0

func get_vehicle_stats() -> Dictionary:
	return {
		"speed": current_speed,
		"rpm": rpm,
		"gear": current_gear,
		"state": current_vehicle_state,
		"nitro_available": nitro_available,
		"nitro_amount": nitro_amount
	}

func save_state() -> Dictionary:
	return {
		"speed": current_speed,
		"rpm": rpm,
		"gear": current_gear,
		"throttle": throttle_input,
		"brake": brake_input,
		"steering": steering_input,
		"nitro_amount": nitro_amount,
		"nitro_cooldown": nitro_cooldown
	}

func load_state(state_data: Dictionary) -> void:
	current_speed = state_data.get("speed", 0.0)
	rpm = state_data.get("rpm", idle_rpm)
	current_gear = state_data.get("gear", 0)
	throttle_input = state_data.get("throttle", 0.0)
	brake_input = state_data.get("brake", 0.0)
	steering_input = state_data.get("steering", 0.0)
	nitro_amount = state_data.get("nitro_amount", 100.0)
	nitro_cooldown = state_data.get("nitro_cooldown", 0.0)

func _exit_tree() -> void:
	if powertrain:
		powertrain.engine_started.disconnect(_on_engine_started)
		powertrain.engine_stopped.disconnect(_on_engine_stopped)
		powertrain.rpm_changed.disconnect(_on_rpm_changed)