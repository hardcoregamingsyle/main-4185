extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller using PhysicsSettings constants
## Handles throttle, brake, steering inputs, wheel forces, gear shifting logic
## Copyright 2026 Thalamus Racing Simulator Project

# Signals emitted by the controller
signal speed_changed(new_speed: float)
signal rpm_changed(new_rpm: float)
signal gear_changed(old_gear: int, new_gear: int)
signal vehicle_moved(position: Vector3, velocity: Vector3)
signal collision_detected(collision_data: Dictionary)

# Constants for physics simulation
const MAX_WHEEL_RADIUS: float = 0.4
const MAX_STEERING_ANGLE: float = DEG_TO_RAD(45.0)
const STEERING_SPEED: float = 180.0
const BRAKE_FORCE_MULTIPLIER: float = 2.5
const ACCELERATION_RATE: float = 0.1
const FRICTION_DAMPING: float = 0.98
const AIR_RESISTANCE: float = 0.001

# Gear ratios (ratio between engine RPM and wheel RPM)
var _gear_ratios: Array[float] = [3.5, 2.5, 1.8, 1.4, 1.1, 0.9]
var _reverse_ratio: float = 3.8

# Vehicle state variables
var _current_speed: float = 0.0  # Speed in m/s
var _max_speed: float = 0.0      # Maximum achievable speed
var _engine_rpm: float = 0.0     # Current engine RPM
var _target_rpm: float = 0.0     # Target RPM based on gear
var _current_gear: int = 0       # 0 = neutral, 1-6 = forward gears, -1 = reverse
var _is_engine_running: bool = false
var _is_braking: bool = false
var _is_throttling: bool = false
var _steering_input: float = 0.0 # -1.0 to 1.0

# Wheel properties
var _wheel_radius: float = 0.35
var _wheel_torque: float = 0.0
var _wheel_brake_force: float = 0.0
var _left_wheel_steering: float = 0.0
var _right_wheel_steering: float = 0.0

# Powertrain reference
@onready var powertrain: Node = get_parent() if has_node("../Powertrain") else null

# Input references
@onready var input_manager := InputManager if Engine.has_singleton("InputManager") else null

func _ready() -> void:
	_init_vehicle_defaults()
	_connect_signals_to_powertrain()

func _init_vehicle_defaults() -> void:
	"""Initialize vehicle with default physics values from PhysicsSettings"""
	var settings := PhysicsSettings.new()
	settings._default_vehicle_mass = 1500.0
	
	mass = settings.default_vehicle_mass
	_max_speed = settings.default_vehicle_max_speed
	_wheel_radius = settings.default_wheel_radius
	_current_gear = 0
	
	# Calculate initial max RPM based on gear ratios
	_target_rpm = settings.engine_redline_rpm * 0.3
	
func _connect_signals_to_powertrain() -> void:
	if powertrain:
		powertrain.engine_state_changed.connect(_on_engine_state_changed)
		powertrain.torque_request.connect(_on_torque_request)

func _physics_process(delta: float) -> void:
	if not is_inside_tree():
		return
	
	_update_physics(delta)
	_update_visuals()

func _update_physics(delta: float) -> void:
	"""Main physics update loop - handles all vehicle dynamics"""
	# Apply gravity
	velocity.y -= PhysicsSettings.gravity * delta
	
	# Update steering
	_update_steering(delta)
	
	# Update gear and transmission
	_update_transmission(delta)
	
	# Apply forces based on input
	_apply_drivetrain_forces(delta)
	
	# Apply resistance forces
	_apply_resistance_forces(delta)
	
	# Move and detect collisions
	move_and_slide()

func _update_steering(delta: float) -> void:
	"""Smoothly interpolate steering angle based on input"""
	var target_steering = _steering_input * MAX_STEERING_ANGLE
	
	# Smooth steering transition
	_left_wheel_steering = lerp(_left_wheel_steering, target_steering, delta * STEERING_SPEED)
	_right_wheel_steering = lerp(_right_wheel_steering, target_steering, delta * STEERING_SPEED)

func _update_transmission(delta: float) -> void:
	"""Handle gear shifting logic and RPM management"""
	if not _is_engine_running:
		_current_gear = 0
		return
	
	# Check for automatic gear shifts
	if GameManager.current_state == GameManager.GameState.RACE_ACTIVE:
		_auto_shift_gears(delta)
	else:
		_manual_shift_check()

func _auto_shift_gears(delta: float) -> void:
	"""Automatic transmission gear shifting logic"""
	var current_rpm = _engine_rpm
	var redline = PhysicsSettings.engine_redline_rpm
	var idle_rpm = PhysicsSettings.engine_idle_rpm
	
	# Shift up if approaching redline
	if _current_gear < 6 and current_rpm > redline * 0.9:
		_shift_gear(_current_gear + 1)
		return
	
	# Shift down if RPM too low
	elif _current_gear > 1 and current_rpm < idle_rpm * 1.5:
		_shift_gear(_current_gear - 1)
		return

func _manual_shift_check() -> void:
	"""Manual gear shift detection via input manager"""
	if input_manager:
		if input_manager.is_action_just_pressed("shift_up"):
			if _current_gear < 6:
				_shift_gear(_current_gear + 1)
		elif input_manager.is_action_just_pressed("shift_down"):
			if _current_gear > 0:
				_shift_gear(_current_gear - 1)

func _shift_gear(new_gear: int) -> void:
	"""Execute gear shift with proper clutch/brake transitions"""
	var old_gear = _current_gear
	
	if new_gear == 0:  # Neutral
		_current_gear = 0
		_wheel_torque = 0.0
	elif new_gear == -1:  # Reverse
		_current_gear = -1
	elif new_gear >= 1 and new_gear <= 6:
		_current_gear = new_gear
	
	gear_changed.emit(old_gear, _current_gear)

func _apply_drivetrain_forces(delta: float) -> void:
	"""Apply acceleration and braking forces to vehicle"""
	if _current_gear == 0 or not _is_engine_running:
		return
	
	var gear_ratio = _get_current_gear_ratio()
	var torque_multiplier = gear_ratio * 0.001
	
	# Calculate drive force based on gear and throttle
	var drive_force = 0.0
	if _is_throttling:
		drive_force = _calculate_drive_force(torque_multiplier)
	
	# Apply brake force if needed
	if _is_braking:
		drive_force -= _calculate_brake_force()
	
	# Apply force to vehicle body
	var forward_direction = transform.basis.z
	force = forward_direction * drive_force

func _calculate_drive_force(gear_ratio: float) -> float:
	"""Calculate drive force based on gear ratio and engine torque"""
	var max_torque = PhysicsSettings.engine_max_torque
	var current_torque = max_torque * (_engine_rpm / PhysicsSettings.engine_peak_torque_rpm)
	
	var wheel_torque = current_torque * gear_ratio * 0.5
	var force = wheel_torque / _wheel_radius
	
	return clamp(force, 0.0, PhysicsSettings.max_drive_force)

func _calculate_brake_force() -> float:
	"""Calculate braking force based on brake input intensity"""
	var brake_intensity = 1.0 if _is_braking else 0.0
	var max_brake_force = PhysicsSettings.max_brake_force
	
	return brake_intensity * max_brake_force * BRAKE_FORCE_MULTIPLIER

func _apply_resistance_forces(delta: float) -> void:
	"""Apply aerodynamic drag and rolling resistance"""
	var air_resistance = velocity.length() * velocity.length() * AIR_RESISTANCE
	var friction_loss = velocity.length() * FRICTION_DAMPING
	
	velocity = velocity.lerp(Vector3.ZERO, friction_loss * delta)

func _calculate_current_speed() -> void:
	"""Calculate current vehicle speed from velocity"""
	_current_speed = velocity.length()
	speed_changed.emit(_current_speed)

func _calculate_engine_rpm() -> void:
	"""Calculate engine RPM based on vehicle speed and gear"""
	if _current_gear == 0:
		_engine_rpm = PhysicsSettings.engine_idle_rpm
		return
	
	var wheel_rpm = _current_speed / (_wheel_radius * 2.0 * PI)
	var gear_ratio = abs(_get_current_gear_ratio())
	_engine_rpm = wheel_rpm * gear_ratio * 1000.0
	
	# Clamp RPM within safe range
	_engine_rpm = clamp(_engine_rpm, PhysicsSettings.engine_idle_rpm, PhysicsSettings.engine_redline_rpm)
	
	rpm_changed.emit(_engine_rpm)

func _get_current_gear_ratio() -> float:
	"""Get the gear ratio for current gear"""
	if _current_gear == 0:
		return 0.0
	elif _current_gear == -1:
		return _reverse_ratio
	else:
		return _gear_ratios[_current_gear - 1]

func _update_visuals() -> void:
	"""Update visual elements like wheel rotation and suspension"""
	# Rotate wheels based on movement
	if _current_speed > 0.1:
		var rotation_delta = _current_speed / _wheel_radius
		$WheelFrontLeft.rotation.x += rotation_delta
		$WheelFrontRight.rotation.x += rotation_delta
		$WheelRearLeft.rotation.x += rotation_delta
		$WheelRearRight.rotation.x += rotation_delta

func _on_engine_state_changed(running: bool) -> void:
	"""Handle engine state changes from powertrain"""
	_is_engine_running = running
	if not running:
		_current_gear = 0
		_wheel_torque = 0.0

func _on_torque_request(torque: float) -> void:
	"""Handle torque requests from powertrain"""
	_wheel_torque = torque

# Public API methods for external control
func set_throttle(input_value: float) -> void:
	"""Set throttle input (-1.0 to 1.0)"""
	_is_throttling = input_value > 0.1

func set_brake(input_value: float) -> void:
	"""Set brake input (0.0 to 1.0)"""
	_is_braking = input_value > 0.1

func set_steering(input_value: float) -> void:
	"""Set steering input (-1.0 to 1.0)"""
	_steering_input = clamp(input_value, -1.0, 1.0)

func start_engine() -> void:
	"""Start the vehicle engine"""
	_is_engine_running = true
	_current_gear = 1 if not _is_throttling else 0

func stop_engine() -> void:
	"""Stop the vehicle engine"""
	_is_engine_running = false
	_current_gear = 0

func reset_vehicle() -> void:
	"""Reset vehicle to initial state"""
	_current_speed = 0.0
	_engine_rpm = PhysicsSettings.engine_idle_rpm
	_current_gear = 0
	_wheel_torque = 0.0
	_velocity = Vector3.ZERO
	transform.origin = Vector3.ZERO

func get_vehicle_stats() -> Dictionary:
	"""Return comprehensive vehicle statistics"""
	return {
		"speed": _current_speed,
		"rpm": _engine_rpm,
		"gear": _current_gear,
		"is_engine_running": _is_engine_running,
		"throttle": _is_throttling,
		"brake": _is_braking,
		"steering": _steering_input
	}