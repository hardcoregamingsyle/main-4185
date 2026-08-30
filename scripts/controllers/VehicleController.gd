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

# References
@onready var powertrain: Powertrain = $Powertrain if $Powertrain else null
@onready var chassis: Chassis = $Chassis if $Chassis else null
@onready var wheel_front_left: RigidBody2D = $WheelFrontLeft if $WheelFrontLeft else null
@onready var wheel_front_right: RigidBody2D = $WheelFrontRight if $WheelFrontRight else null
@onready var wheel_rear_left: RigidBody2D = $WheelRearLeft if $WheelRearLeft else null
@onready var wheel_rear_right: RigidBody2D = $WheelRearRight if $WheelRearRight else null
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
var _settings: PhysicsSettings = PhysicsSettings.get_singleton()

# Internal timers
var _last_update_time: float = 0.0
var _nitro_timer: float = 0.0

func _ready() -> void:
	_init_references()
	_connect_signals()
	_reset_state()

func _init_references() -> void:
	if powertrain == null:
		powertrain = Powertrain.new()
		add_child(powertrain)
	
	if chassis == null:
		var chassis_node = Chassis.new()
		chassis_node.name = "Chassis"
		add_child(chassis_node)
		chassis = chassis_node
	
	# Get physics body if not already set
	if _physics_body == null:
		_physics_body = get_parent() as CharacterBody2D
		if _physics_body == null:
			push_warning("VehicleController: No CharacterBody2D parent found")

func _connect_signals() -> void:
	if powertrain:
		powertrain.engine_started.connect(_on_engine_started)
		powertrain.engine_stopped.connect(_on_engine_stopped)
		powertrain.rpm_changed.connect(_on_rpm_changed)

func _reset_state() -> void:
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
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	last_collision_time = 0.0

func _process(delta: float) -> void:
	_handle_input(delta)
	_update_physics(delta)
	_update_state(delta)
	_update_nitro_system(delta)

func _handle_input(delta: float) -> void:
	# Get input values from InputManager
	if InputManager:
		throttle_input = InputManager.get_throttle()
		brake_input = InputManager.get_brake()
		steering_input = InputManager.get_steering()
		
		# Check gear shift requests
		if InputManager.is_shift_up_pressed():
			shift_up_request = true
		else:
			shift_up_request = false
			
		if InputManager.is_shift_down_pressed():
			shift_down_request = true
		else:
			shift_down_request = false

func _update_physics(delta: float) -> void:
	if _physics_body == null:
		return
	
	# Calculate effective RPM based on gear and speed
	var effective_rpm = _calculate_effective_rpm()
	rpm = lerp(rpm, effective_rpm, 0.1)
	
	# Apply gear-specific torque and force calculations
	var gear_ratios = _get_gear_ratios()
	var final_drive_ratio = 3.5
	var tire_radius = 0.3
	
	var gear_ratio = gear_ratios[current_gear] if current_gear != 0 else 0.0
	var wheel_torque = powertrain.torque * gear_ratio * final_drive_ratio
	
	# Handle different states
	match current_vehicle_state:
		VehicleState.RUNNING, VehicleState.REVVING:
			_apply_acceleration(wheel_torque, delta, gear_ratio, tire_radius)
		VehicleState.BRAKING:
			_apply_braking(delta)
		VehicleState.DRIFTING:
			_apply_drifting(delta)
		VehicleState.COLLIDED:
			_apply_collision_response(delta)
		_:
			_apply_idle_physics(delta)
	
	# Update velocity based on calculated acceleration
	var velocity_vector = Vector2.RIGHT * current_speed
	_physics_body.velocity = velocity_vector
	
	# Apply steering rotation if wheels are moving
	if abs(current_speed) > 0.1:
		rotate_wheels(steering_angle)

func _apply_acceleration(torque: float, delta: float, gear_ratio: float, tire_radius: float) -> void:
	var max_force = _settings.default_vehicle_mass * _settings.gravity * 0.8
	var force_multiplier = (rpm / max_rpm) * throttle_input
	
	# Calculate acceleration force based on gear ratio and RPM
	var acceleration_force = torque * force_multiplier * gear_ratio * 0.1
	
	# Clamp force to maximum
	acceleration_force = clampf(acceleration_force, 0.0, max_force)
	
	# Apply to physics body
	if _physics_body:
		acceleration = acceleration_force / _settings.default_vehicle_mass
		_physics_body.velocity.x += acceleration * delta
	
	current_speed = _physics_body.velocity.length() if _physics_body else 0.0

func _apply_braking(delta: float) -> void:
	var deceleration_rate = _settings.gravity * 1.5
	var actual_deceleration = min(deceleration_rate, current_speed / delta)
	
	if _physics_body:
		_physics_body.velocity.x -= actual_deceleration * delta * brake_input
		braking_force = actual_deceleration * _settings.default_vehicle_mass
	
	current_speed = _physics_body.velocity.length() if _physics_body else 0.0

func _apply_drifting(delta: float) -> void:
	# Drift mechanics - reduced traction, increased side slip
	var drift_factor = 0.7
	if _physics_body:
		_physics_body.velocity.x *= drift_factor
		current_speed = _physics_body.velocity.length()

func _apply_collision_response(delta: float) -> void:
	collision_damping = lerp(collision_damping, 0.3, delta * 2.0)
	
	if _physics_body:
		_physics_body.velocity *= collision_damping
	current_speed = _physics_body.velocity.length() if _physics_body else 0.0

func _apply_idle_physics(delta: float) -> void:
	# Apply friction when idle
	var friction_coefficient = 0.98
	if _physics_body:
		_physics_body.velocity *= friction_coefficient
	current_speed = _physics_body.velocity.length() if _physics_body else 0.0

func _update_state(delta: float) -> void:
	var prev_state = current_vehicle_state
	
	# Determine new state based on conditions
	if current_speed < 0.5 and rpm < idle_rpm + 100:
		current_vehicle_state = VehicleState.IDLE
	elif rpm > max_rpm * 0.9:
		current_vehicle_state = VehicleState.REVVING
	elif brake_input > 0.5:
		current_vehicle_state = VehicleState.BRAKING
	elif throttle_input > 0.5 and abs(steering_input) > 0.3:
		current_vehicle_state = VehicleState.DRIFTING
	else:
		current_vehicle_state = VehicleState.RUNNING
	
	# Emit signal if state changed
	if prev_state != current_vehicle_state:
		pass  # Could emit state change signal here

func _update_nitro_system(delta: float) -> void:
	# Handle nitro cooldown
	if nitro_cooldown > 0:
		nitro_cooldown -= delta
		if nitro_cooldown <= 0:
			nitro_available = true
			nitro_cooldown = 0.0
	
	# Check for nitro activation
	if InputManager.is_nitro_pressed() and nitro_available and nitro_amount > 0:
		activate_nitro()

func activate_nitro() -> void:
	if nitro_amount <= 0 or not nitro_available:
		return
	
	var nitro_boost = 1.5  # 50% speed boost
	nitro_amount -= 20.0  # Consume nitro
	nitro_cooldown = 10.0  # 10 second cooldown
	nitro_available = false
	
	# Apply temporary speed boost
	if _physics_body:
		_physics_body.velocity *= nitro_boost
	
	emit_signal("nitro_used", nitro_amount)
	print("Nitro activated! Remaining: %.1f%%" % nitro_amount)

func _calculate_effective_rpm() -> float:
	var gear_ratios = _get_gear_ratios()
	var final_drive_ratio = 3.5
	var tire_radius = 0.3
	
	if current_gear == 0:
		return idle_rpm
	
	var gear_ratio = gear_ratios[current_gear]
	var wheel_rpm = (current_speed / (tire_radius * 2.0 * PI)) * gear_ratio * final_drive_ratio
	
	return lerp(rpm, wheel_rpm, 0.05)

func _get_gear_ratios() -> Dictionary:
	# Standard 6-speed gearbox ratios
	return {
		0: 0.0,       # Neutral
		1: 3.5,       # First gear - high torque
		2: 2.2,       # Second gear
		3: 1.6,       # Third gear
		4: 1.2,       # Fourth gear
		5: 0.9,       # Fifth gear
		6: 0.7,       # Sixth gear - low torque, high speed
		-1: 3.0       # Reverse
	}

func shift_gear(new_gear: int) -> bool:
	if new_gear < -1 or new_gear > 6:
		return false
	
	var old_gear = current_gear
	current_gear = new_gear
	
	emit_signal("gear_changed", old_gear, new_gear)
	
	# Adjust RPM based on gear change
	var gear_ratios = _get_gear_ratios()
	var current_ratio = gear_ratios[old_gear]
	var new_ratio = gear_ratios[new_gear]
	
	if current_ratio != 0 and new_ratio != 0:
		var rpm_change = (current_ratio / new_ratio)
		rpm = min(max_rpm, rpm * rpm_change)
	
	# Play gear shift sound via AudioManager
	if AudioManager:
		AudioManager.play_sound("gear_shift")
	
	return true

func auto_shift_gear() -> void:
	# Automatic gear shifting based on RPM
	var gear_ratios = _get_gear_ratios()
	
	if rpm > max_rpm * 0.85 and current_gear < 6:
		shift_gear(current_gear + 1)
	elif rpm < idle_rpm * 1.5 and current_gear > 1:
		shift_gear(current_gear - 1)
	elif current_gear == 0 and current_speed > 1.0:
		shift_gear(1)
	elif current_gear > 0 and current_speed < 0.5:
		shift_gear(0)

func rotate_wheels(angle: float) -> void:
	# Rotate front wheels based on steering input
	if wheel_front_left and wheel_front_right:
		wheel_front_left.rotation = angle
		wheel_front_right.rotation = angle
		steering_angle = angle

func apply_wheel_forces(force: float) -> void:
	# Apply forces to individual wheels for realistic suspension behavior
	var wheel_configurations = [
		{"node": wheel_front_left, "position": Vector2(-0.5, -0.4)},
		{"node": wheel_front_right, "position": Vector2(-0.5, 0.4)},
		{"node": wheel_rear_left, "position": Vector2(0.5, -0.4)},
		{"node": wheel_rear_right, "position": Vector2(0.5, 0.4)}
	]
	
	for config in wheel_configurations:
		if config["node"] and config["node"].is_inside_tree():
			var force_vector = Vector2.RIGHT * force * 0.25
			config["node"].apply_central_impulse(force_vector)

func reset_vehicle() -> void:
	_reset_state()
	if _physics_body:
		_physics_body.velocity = Vector2.ZERO
	_physics_body.linear_damp = 0.0

func _on_engine_started() -> void:
	current_vehicle_state = VehicleState.RUNNING
	print("Engine started")

func _on_engine_stopped() -> void:
	current_vehicle_state = VehicleState.IDLE
	rpm = idle_rpm
	print("Engine stopped")

func _on_rpm_changed(new_rpm: float) -> void:
	rpm = new_rpm
	if rpm >= max_rpm:
		current_vehicle_state = VehicleState.REVVING

func get_current_speed() -> float:
	return current_speed

func get_max_speed() -> float:
	return _settings.default_vehicle_mass * 0.1  # Simplified calculation

func get_gear_info() -> Dictionary:
	return {
		"current_gear": current_gear,
		"rpm": rpm,
		"max_rpm": max_rpm,
		"idle_rpm": idle_rpm
	}

func get_state_info() -> Dictionary:
	return {
		"state": current_vehicle_state,
		"speed": current_speed,
		"throttle": throttle_input,
		"brake": brake_input,
		"steering": steering_input
	}