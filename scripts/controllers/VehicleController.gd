extends Node

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
signal vehicle_damage_taken(damage_amount: float)

# Constants from PhysicsSettings
var _physics_settings: PhysicsSettings = PhysicsSettings.new()
var _vehicle_mass: float = 1500.0
var _current_gear: int = 0
var _max_gears: int = 6
var _engine_rpm: float = 0.0
var _engine_idle_rpm: float = 800.0
var _engine_max_rpm: float = 7000.0
var _nitro_available: bool = true
var _nitro_cooldown: float = 3.0
var _last_nitro_use: float = -999.0

# Input states
var _input_throttle: float = 0.0
var _input_brake: float = 0.0
var _input_steering: float = 0.0
var _input_reverse: bool = false

# Vehicle state
enum VehicleState { IDLE, RUNNING, REVVING, BRAKING, COLLIDED, DRIFTING }
var current_vehicle_state: VehicleState = VehicleState.IDLE

# Physics references
@onready var powertrain: Powertrain = $Powertrain if $Powertrain else null
@onready var chassis: Chassis = $Chassis if $Chassis else null
@onready var wheel_front_left: RigidBody2D = $WheelFrontLeft if $WheelFrontLeft else null
@onready var wheel_front_right: RigidBody2D = $WheelFrontRight if $WheelFrontRight else null
@onready var wheel_rear_left: RigidBody2D = $WheelRearLeft if $WheelRearLeft else null
@onready var wheel_rear_right: RigidBody2D = $WheelRearRight if $WheelRearRight else null
@onready var physics_body: CharacterBody2D = get_parent() as CharacterBody2D

# Movement properties
var _velocity: Vector2 = Vector2.ZERO
var _speed: float = 0.0
var _acceleration: float = 0.0
var _deceleration: float = 0.0

# Collision state
var last_collision_time: float = 0.0
var collision_damping: float = 0.3
var damage_accumulated: float = 0.0

# Drift mechanics
var drift_angle: float = 0.0
var drift_intensity: float = 0.0
var drift_enabled: bool = false

func _ready() -> void:
	_init_physics_settings()
	_connect_signals()
	_reset_vehicle()

func _init_physics_settings() -> void:
	"""Initialize physics settings from global resource"""
	if Engine.has_singleton("PhysicsSettings"):
		var ps = Engine.get_singleton("PhysicsSettings")
		_vehicle_mass = ps.default_vehicle_mass
		_max_gears = ps.default_max_gears
	else:
		_vehicle_mass = 1500.0
		_max_gears = 6

func _connect_signals() -> void:
	"""Connect to global game manager signals"""
	if GameManager:
		GameManager.game_state_changed.connect(_on_game_state_changed)

func _process(delta: float) -> void:
	"""Handle input processing and game loop updates"""
	_process_input(delta)
	_update_engine_state(delta)
	_update_vehicle_movement(delta)
	_check_collision_state(delta)
	_update_drift_mechanics(delta)

func _physics_process(delta: float) -> void:
	"""Fixed timestep physics updates"""
	apply_wheel_forces(delta)
	apply_gravity_and_friction(delta)
	_update_gear_shift_logic(delta)

func _process_input(delta: float) -> void:
	"""Process player input and normalize values"""
	if GameManager.current_state != GameManager.GameState.RACE_ACTIVE:
		return
	
	# Get normalized input values (-1 to 1 range)
	_input_throttle = InputManager.get_axis("throttle")
	_input_brake = InputManager.get_axis("brake")
	_input_steering = InputManager.get_axis("steering")
	
	# Clamp input values
	_input_throttle = clampf(_input_throttle, -1.0, 1.0)
	_input_brake = clampf(_input_brake, -1.0, 1.0)
	_input_steering = clampf(_input_steering, -1.0, 1.0)
	
	# Handle reverse gear
	_input_reverse = Input.is_action_pressed("reverse")

func _update_engine_state(delta: float) -> void:
	"""Update engine RPM and power delivery"""
	var target_rpm: float = _calculate_target_rpm()
	_engine_rpm = lerp(_engine_rpm, target_rpm, delta * 10.0)
	
	# Update powertrain RPM
	if powertrain:
		powertrain.engine_rpm = _engine_rpm / 1000.0

func _calculate_target_rpm() -> float:
	"""Calculate target engine RPM based on current gear and throttle"""
	var gear_ratio: float = _get_current_gear_ratio()
	var throttle_input: float = abs(_input_throttle)
	
	# Base RPM calculation
	var base_rpm = _engine_idle_rpm + (_engine_max_rpm - _engine_idle_rpm) * throttle_input
	
	# Adjust for gear ratio
	base_rpm = base_rpm * gear_ratio
	
	# Clamp to valid range
	return clampf(base_rpm, _engine_idle_rpm, _engine_max_rpm)

func _update_vehicle_movement(delta: float) -> void:
	"""Update vehicle velocity and position"""
	if not physics_body:
		return
	
	# Calculate acceleration based on throttle and gear
	var acceleration_force: float = _calculate_acceleration()
	
	# Apply acceleration
	_velocity += Vector2.RIGHT * acceleration_force * delta
	
	# Apply braking
	if _input_brake > 0.1:
		var braking_force: float = _physics_settings.default_vehicle_mass * 0.8
		_velocity.x -= braking_force * delta
		
		current_vehicle_state = VehicleState.BRAKING
	else:
		current_vehicle_state = VehicleState.RUNNING
	
	# Apply friction/drag
	_velocity *= 1.0 - (_physics_settings.default_wheel_friction * delta)
	
	# Update speed
	_speed = abs(_velocity.x)
	
	# Emit speed change signal
	emit_signal("speed_changed", _speed, _get_max_speed_for_current_gear())
	
	# Apply velocity to physics body
	if physics_body is CharacterBody2D:
		physics_body.velocity = _velocity

func _calculate_acceleration() -> float:
	"""Calculate acceleration force based on gear and throttle"""
	if _input_throttle <= 0.0:
		return 0.0
	
	var gear_ratio: float = _get_current_gear_ratio()
	var torque: float = _powertrain_torque() * gear_ratio
	var acceleration: float = torque / _vehicle_mass
	
	return acceleration * _input_throttle

func _powertrain_torque() -> float:
	"""Get current torque from powertrain"""
	if powertrain:
		return powertrain.torque_output
	return 500.0

func _get_current_gear_ratio() -> float:
	"""Get gear ratio for current gear"""
	var gear_ratios: Array[float] = [3.5, 2.5, 1.8, 1.3, 1.0, 0.7, 0.5]
	
	if _current_gear < 0:
		return gear_ratios[0] * 3.0  # Reverse gear ratio
	elif _current_gear >= len(gear_ratios):
		return gear_ratios[-1]
	else:
		return gear_ratios[_current_gear]

func _get_max_speed_for_current_gear() -> float:
	"""Get maximum speed for current gear"""
	var max_speeds: Array[float] = [80.0, 120.0, 160.0, 200.0, 240.0, 280.0]
	
	if _current_gear < 0:
		return 30.0  # Reverse max speed
	elif _current_gear >= len(max_speeds):
		return max_speeds[-1]
	else:
		return max_speeds[_current_gear]

func apply_wheel_forces(delta: float) -> void:
	"""Apply forces to individual wheels for realistic suspension"""
	var wheel_positions: Array[Vector2] = []
	var wheel_radii: Array[float] = [0.35, 0.35, 0.35, 0.35]
	
	# Define wheel positions relative to vehicle center
	wheel_positions.append(Vector2(-0.8, 0.6))  # Front left
	wheel_positions.append(Vector2(0.8, 0.6))   # Front right
	wheel_positions.append(Vector2(-0.8, -0.6)) # Rear left
	wheel_positions.append(Vector2(0.8, -0.6))  # Rear right
	
	# Apply traction forces
	for i in range(wheel_positions.size()):
		if i < 2:  # Front wheels (steering)
			continue  # No drive force on front wheels
		else:  # Rear wheels (drive)
			var wheel_pos: Vector2 = _position + wheel_positions[i]
			var force_direction: Vector2 = _velocity.normalized() if _speed > 0.1 else Vector2.RIGHT
			
			var drive_force: float = _calculate_drive_force(i)
			
			if wheel_positions[i].y < 0:  # Rear wheel
				var wheel_rigid: RigidBody2D = wheel_rear_left if i == 2 else wheel_rear_right
				if wheel_rigid:
					wheel_rigid.apply_central_impulse(force_direction * drive_force * delta)

func _calculate_drive_force(wheel_index: int) -> float:
	"""Calculate drive force for rear wheels"""
	var base_force: float = _physics_settings.default_vehicle_mass * 0.5
	var gear_multiplier: float = _get_current_gear_ratio()
	var throttle_factor: float = _input_throttle
	
	return base_force * gear_multiplier * throttle_factor

func apply_gravity_and_friction(delta: float) -> void:
	"""Apply gravity and friction forces"""
	if not physics_body:
		return
	
	# Apply gravity
	var gravity_vector: Vector2 = Vector2.DOWN * _physics_settings.gravity
	physics_body.apply_central_force(gravity_vector * _vehicle_mass * delta)
	
	# Apply friction
	var friction_coefficient: float = _physics_settings.default_wheel_friction
	var friction_force: Vector2 = -_velocity.normalized() * _vehicle_mass * friction_coefficient * delta
	physics_body.apply_central_force(friction_force)

func _update_gear_shift_logic(delta: float) -> void:
	"""Automatically shift gears based on RPM and speed"""
	if _current_gear < 0:  # In reverse
		return
	
	# Check for upshift
	if _engine_rpm > _engine_max_rpm * 0.9 and _current_gear < _max_gears - 1:
		_shift_up()
	
	# Check for downshift
	elif _engine_rpm < _engine_idle_rpm * 1.5 and _current_gear > 0 and _input_throttle < 0.1:
		_shift_down()

func _shift_up() -> void:
	"""Shift to next higher gear"""
	if _current_gear < _max_gears - 1:
		var old_gear: int = _current_gear
		_current_gear += 1
		emit_signal("gear_changed", old_gear, _current_gear)
		
		# Slight RPM drop on upshift
		_engine_rpm = _engine_rpm * 0.7

func _shift_down() -> void:
	"""Shift to next lower gear"""
	if _current_gear > 0:
		var old_gear: int = _current_gear
		_current_gear -= 1
		emit_signal("gear_changed", old_gear, _current_gear)
		
		# RPM spike on downshift
		_engine_rpm = minf(_engine_rpm * 1.3, _engine_max_rpm)

func _check_collision_state(delta: float) -> void:
	"""Monitor and handle collision events"""
	var time_since_collision: float = Time.get_unix_time_from_system() - last_collision_time
	
	if time_since_collision < 0.5 and damage_accumulated > 0:
		current_vehicle_state = VehicleState.COLLIDED
		collision_damping = lerp(collision_damping, 0.3, delta * 5.0)
	else:
		collision_damping = lerp(collision_damping, 0.0, delta * 2.0)
		current_vehicle_state = VehicleState.RUNNING

func _update_drift_mechanics(delta: float) -> void:
	"""Update drift physics and angle calculations"""
	if not drift_enabled:
		return
	
	# Calculate drift angle based on steering and speed
	var steering_factor: float = abs(_input_steering)
	var speed_factor: float = _speed / 100.0  # Normalized by expected speed
	
	if steering_factor > 0.3 and _speed > 30.0:
		drift_angle = lerp(drift_angle, steering_factor * 45.0, delta * 3.0)
		drift_intensity = lerp(drift_intensity, 1.0, delta * 2.0)
	else:
		drift_angle = lerp(drift_angle, 0.0, delta * 5.0)
		drift_intensity = lerp(drift_intensity, 0.0, delta * 3.0)

func activate_drift(enable: bool = true) -> void:
	"""Enable or disable drift mode"""
	if enable:
		drift_enabled = true
		drift_intensity = 1.0
	else:
		drift_enabled = false
		drift_angle = 0.0

func activate_nitro() -> void:
	"""Activate nitro boost if available"""
	var current_time: float = Time.get_unix_time_from_system()
	
	if _nitro_available and current_time - _last_nitro_use > _nitro_cooldown:
		_last_nitro_use = current_time
		_nitro_available = false
		
		# Boost velocity
		var boost_strength: float = 50.0
		_velocity += Vector2.RIGHT * boost_strength
		emit_signal("nitro_used", boost_strength)
		
		# Cooldown timer (will be reset after delay)
		await get_tree().create_timer(_nitro_cooldown).timeout
		_nitro_available = true

func take_damage(damage_amount: float, direction: Vector2 = Vector2.ZERO) -> void:
	"""Apply damage to vehicle"""
	damage_accumulated += damage_amount
	last_collision_time = Time.get_unix_time_from_system()
	
	if physics_body:
		# Knockback effect
		var knockback: Vector2 = direction.normalized() * damage_amount * 2.0
		_velocity += knockback
	
	emit_signal("vehicle_damage_taken", damage_amount)
	
	if damage_accumulated >= 100.0:
		_destroy_vehicle()

func _destroy_vehicle() -> void:
	"""Destroy the vehicle when severely damaged"""
	if GameManager:
		GameManager.emit_signal("vehicle_destroyed", self)
	queue_free()

func _reset_vehicle() -> void:
	"""Reset vehicle to initial state"""
	_velocity = Vector2.ZERO
	_speed = 0.0
	_acceleration = 0.0
	_deceleration = 0.0
	_current_gear = 0
	_engine_rpm = _engine_idle_rpm
	damage_accumulated = 0.0
	last_collision_time = 0.0
	collision_damping = 0.3
	_input_throttle = 0.0
	_input_brake = 0.0
	_input_steering = 0.0
	_input_reverse = false

func set_position(new_position: Vector2) -> void:
	"""Set vehicle position directly"""
	if physics_body:
		physics_body.global_position = new_position
		_velocity = Vector2.ZERO

func get_vehicle_state() -> Dictionary:
	"""Return current vehicle state dictionary"""
	return {
		"state": current_vehicle_state,
		"speed": _speed,
		"rpm": _engine_rpm,
		"gear": _current_gear,
		"velocity": _velocity,
		"damage": damage_accumulated,
		"drift_enabled": drift_enabled,
		"nitro_available": _nitro_available
	}

func _on_game_state_changed(new_state: GameManager.GameState) -> void:
	"""Handle global game state changes"""
	match new_state:
		GameManager.GameState.MAIN_MENU:
			_reset_vehicle()
			engine_stopped.emit()
		GameManager.GameState.RACE_ACTIVE:
			current_vehicle_state = VehicleState.RUNNING
			engine_started.emit()
		GameManager.GameState.RACE_PAUSED:
			current_vehicle_state = VehicleState.IDLE

func _set_property(name: String, value: Variant) -> void:
	"""Set vehicle property by name (for remote control)"""
	match name:
		"throttle":
			_input_throttle = clampf(value as float, -1.0, 1.0)
		"brake":
			_input_brake = clampf(value as float, -1.0, 1.0)
		"steering":
			_input_steering = clampf(value as float, -1.0, 1.0)
		"gear":
			_current_gear = clampi(value as int, -1, _max_gears - 1)
		"position":
			set_position(value as Vector2)

func _get_property(name: String) -> Variant:
	"""Get vehicle property by name"""
	match name:
		"speed":
			return _speed
		"rpm":
			return _engine_rpm
		"gear":
			return _current_gear
		"state":
			return current_vehicle_state
		"velocity":
			return _velocity
		"nitro_available":
			return _nitro_available
	return null

func debug_get_all_properties() -> Dictionary:
	"""Return all vehicle properties for debugging"""
	return {
		"speed": _speed,
		"velocity": _velocity,
		"engine_rpm": _engine_rpm,
		"current_gear": _current_gear,
		"max_gears": _max_gears,
		"vehicle_state": current_vehicle_state,
		"damage": damage_accumulated,
		"input_throttle": _input_throttle,
		"input_brake": _input_brake,
		"input_steering": _input_steering,
		"nitro_available": _nitro_available,
		"drift_enabled": drift_enabled,
		"mass": _vehicle_mass
	}