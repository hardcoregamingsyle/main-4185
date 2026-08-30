extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for the racing simulator
## Handles throttle, brake, steering inputs, wheel forces, gear shifting logic
## Integrates with PhysicsSettings for consistent vehicle dynamics
## Copyright 2026 Thalamus Racing Simulator Project

# Signals for external communication
signal speed_changed(new_speed: float)
signal gear_changed(old_gear: int, new_gear: int)
signal drift_angle_changed(angle: float)
signal damage_taken(damage_amount: float)

# Constants - loaded from PhysicsSettings singleton
var _physics: PhysicsSettings = PhysicsSettings.get_singleton()

# Vehicle properties
@export_group("Vehicle Properties")
@export var vehicle_mass: float = 1500.0
@export var center_of_mass_offset: Vector3 = Vector3(0, 0.5, 0)
@export var max_steering_angle: float = 30.0 * DEG2RAD
@export var tire_friction_coefficient: float = 0.9

@export_group("Powertrain Settings")
@export var engine_torque_curve: Array[Vector2f] = []
@export var transmission_ratios: Array[float] = [3.5, 2.0, 1.4, 1.0, 0.8, 0.6]
@export var final_drive_ratio: float = 3.73
@export var idle_rpm: float = 800.0
@export var redline_rpm: float = 7000.0

@export_group("Drivetrain Configuration")
enum DrivetrainType { FWD, RWD, AWD }
@export var drivetrain_type: DrivetrainType = DrivetrainType.RWD

@export_group("Suspension & Handling")
@export var suspension_stiffness: float = 50000.0
@export var damping_ratio: float = 0.45
@export var roll_bar_strength: float = 10000.0
@export var anti_roll_bar_front: float = 5000.0
@export var anti_roll_bar_rear: float = 3000.0

# State variables
var _current_gear: int = 0
var _engine_rpm: float = 0.0
var _wheel_speeds: Array[float] = [0.0, 0.0, 0.0, 0.0]
var _throttle_input: float = 0.0
var _brake_input: float = 0.0
var _steering_input: float = 0.0
var _handbrake_active: bool = false

# Physical simulation state
var _velocity_magnitude: float = 0.0
var _drift_angle: float = 0.0
var _acceleration_vector: Vector3 = Vector3.ZERO
var _angular_velocity_y: float = 0.0

# Wheel positions (local space)
var _wheel_positions: Array[Vector3] = [
	Vector3(-0.8, -0.3, 1.2),  # Front Left
	Vector3(0.8, -0.3, 1.2),   # Front Right
	Vector3(-0.8, -0.3, -1.2), # Rear Left
	Vector3(0.8, -0.3, -1.2)   # Rear Right
]

# Suspension state per wheel
var _suspension_compression: Array[float] = [0.0, 0.0, 0.0, 0.0]
var _suspension_damping: Array[float] = [0.0, 0.0, 0.0, 0.0]

# Tire grip state
var _tire_grip_level: Array[float] = [1.0, 1.0, 1.0, 1.0]

# Gear shift timing
var _gear_shift_timer: float = 0.0
var _gear_shift_delay: float = 0.3

# Clutch state
var _clutch_engaged: bool = true
var _clutch_position: float = 1.0

func _ready() -> void:
	_process_mode = ProcessModeEnum.PHYSICS
	
	# Initialize default values if not set
	if engine_torque_curve.is_empty():
		_init_default_torque_curve()
	
	if transmission_ratios.is_empty():
		transmission_ratios = [3.5, 2.0, 1.4, 1.0, 0.8, 0.6]
	
	# Set up initial gear
	_current_gear = 0
	_engine_rpm = idle_rpm
	
	# Connect to input manager
	if InputManager:
		InputManager.input_changed.connect(_on_input_changed)

func _init_default_torque_curve() -> void:
	"""Initialize default engine torque curve as normalized points"""
	engine_torque_curve = [
		Vector2f(0.0, 0.3),     # Idle
		Vector2f(0.2, 0.5),     # Low RPM
		Vector2f(0.4, 0.8),     # Mid RPM
		Vector2f(0.6, 1.0),     # Peak Torque
		Vector2f(0.8, 0.95),    # High RPM
		Vector2f(1.0, 0.7)      # Redline
	]

func _physics_process(delta: float) -> void:
	"""Main physics update loop - runs at fixed timestep"""
	_handle_input_processing(delta)
	_update_engine_state(delta)
	_update_drivetrain(delta)
	_update_suspension_and_tires(delta)
	_apply_forces_to_vehicle(delta)
	_update_gear_shifting(delta)

func _handle_input_processing(delta: float) -> void:
	"""Process raw input and apply smoothing/filtering"""
	# Read input values
	var target_throttle = InputManager.get_axis("accelerate", "brake_reverse")
	var target_brake = InputManager.get_axis("brake", "accelerate_reverse")
	var target_steering = InputManager.get_axis("steer_left", "steer_right")
	var handbrake = InputManager.is_action_pressed("handbrake")
	
	# Clamp inputs to valid ranges
	_throttle_input = clamp(target_throttle, 0.0, 1.0)
	_brake_input = clamp(target_brake, 0.0, 1.0)
	_steering_input = clamp(target_steering, -1.0, 1.0)
	_handbrake_active = handbrake
	
	# Apply input smoothing for realistic response
	_throttle_input = lerp(_throttle_input, target_throttle, delta * 10.0)
	_brake_input = lerp(_brake_input, target_brake, delta * 15.0)
	_steering_input = lerp(_steering_input, target_steering, delta * 8.0)

func _update_engine_state(delta: float) -> void:
	"""Update engine RPM based on throttle, gear, and load"""
	var target_rpm = _calculate_target_rpm()
	var rpm_change_rate = _calculate_rpm_change_rate()
	
	# Apply inertia to RPM changes
	_engine_rpm = lerp(_engine_rpm, target_rpm, delta * rpm_change_rate)
	
	# Enforce RPM limits
	_engine_rpm = clamp(_engine_rpm, idle_rpm, redline_rpm)
	
	# Handle clutch engagement/disengagement
	if _clutch_engaged:
		_clutch_position = lerp(_clutch_position, 1.0, delta * 20.0)
	else:
		_clutch_position = lerp(_clutch_position, 0.0, delta * 30.0)

func _calculate_target_rpm() -> float:
	"""Calculate desired RPM based on current velocity and gear"""
	if _current_gear < 0 or _current_gear >= transmission_ratios.size():
		return idle_rpm
	
	var gear_ratio = transmission_ratios[_current_gear]
	var wheel_radius: float = 0.3  # Standard tire radius
	
	# Calculate target RPM from vehicle speed
	var speed_in_m_s = _velocity_magnitude
	var target_wheel_rps = speed_in_m_s / (2.0 * PI * wheel_radius)
	var target_engine_rpm = target_wheel_rps * gear_ratio * final_drive_ratio * 60.0
	
	# Adjust for throttle input
	if _throttle_input > 0.1:
		target_engine_rpm *= 1.1  # Slight overspeed for responsiveness
	elif _brake_input > 0.1:
		target_engine_rpm *= 0.9  # Engine braking
	
	return target_engine_rpm

func _calculate_rpm_change_rate() -> float:
	"""Calculate how quickly RPM can change based on engine characteristics"""
	var base_rate = 100.0
	
	# More torque = faster RPM changes
	var current_torque = _get_current_torque()
	base_rate += current_torque * 20.0
	
	# Heavier vehicles accelerate slower
	base_rate /= (vehicle_mass / 1000.0)
	
	return base_rate

func _get_current_torque() -> float:
	"""Get engine torque at current RPM"""
	if engine_torque_curve.is_empty():
		return 0.0
	
	# Normalize current RPM to 0-1 range
	var normalized_rpm = (_engine_rpm - idle_rpm) / (redline_rpm - idle_rpm)
	normalized_rpm = clamp(normalized_rpm, 0.0, 1.0)
	
	# Interpolate torque curve
	for i in range(engine_torque_curve.size() - 1):
		var point_a = engine_torque_curve[i]
		var point_b = engine_torque_curve[i + 1]
		
		if normalized_rpm >= point_a.x and normalized_rpm <= point_b.x:
			var t = (normalized_rpm - point_a.x) / (point_b.x - point_a.x)
			return point_a.y + (point_b.y - point_a.y) * t
	
	return engine_torque_curve.back().y

func _update_drivetrain(delta: float) -> void:
	"""Update power distribution to wheels based on drivetrain type"""
	var total_torque = _calculate_wheel_torque()
	
	# Distribute torque based on drivetrain
	match drivetrain_type:
		DrivetrainType.FWD:
			_wheel_speeds[0] += total_torque * delta * 0.5
			_wheel_speeds[1] += total_torque * delta * 0.5
		DrivetrainType.RWD:
			_wheel_speeds[2] += total_torque * delta * 0.5
			_wheel_speeds[3] += total_torque * delta * 0.5
		DrivetrainType.AWD:
			_wheel_speeds[0] += total_torque * delta * 0.25
			_wheel_speeds[1] += total_torque * delta * 0.25
			_wheel_speeds[2] += total_torque * delta * 0.25
			_wheel_speeds[3] += total_torque * delta * 0.25
	
	# Apply braking force
	if _brake_input > 0.0:
		var brake_force = _brake_input * 5000.0
		for i in range(4):
			_wheel_speeds[i] -= brake_force * delta

func _calculate_wheel_torque() -> float:
	"""Calculate torque delivered to wheels"""
	if not _clutch_engaged:
		return 0.0
	
	var engine_torque = _get_current_torque() * 400.0  # Base torque multiplier
	var gear_ratio = transmission_ratios[_current_gear]
	var total_ratio = gear_ratio * final_drive_ratio
	
	# Account for transmission efficiency
	var efficiency: float = 0.85
	var wheel_torque = engine_torque * total_ratio * efficiency
	
	# Reduce torque during gear shifts
	if _gear_shift_timer > 0:
		wheel_torque *= 0.3
	
	return wheel_torque

func _update_suspension_and_tires(delta: float) -> void:
	"""Update suspension compression and tire grip levels"""
	# Simplified suspension model - in full implementation would use raycasts
	for i in range(4):
		var gravity_component = _physics.gravity * _physics.time_scale
		var expected_compression = vehicle_mass * gravity_component / suspension_stiffness
		
		# Damped oscillation
		var current_compression = _suspension_compression[i]
		var damping = damping_ratio * 2.0 * sqrt(suspension_stiffness * vehicle_mass / 4.0)
		
		# Simple spring-damper system
		var force = -suspension_stiffness * (current_compression - expected_compression)
		var damping_force = -damping * _suspension_damping[i]
		
		_suspension_damping[i] = lerp(_suspension_damping[i], 
			(force + damping_force) / vehicle_mass, delta * 10.0)
		
		_suspension_compression[i] += _suspension_damping[i] * delta
		_suspension_compression[i] = clamp(_suspension_compression[i], 0.0, 0.3)
	
	# Update tire grip based on slip and conditions
	_update_tire_grip(delta)

func _update_tire_grip(delta: float) -> void:
	"""Calculate tire grip levels based on lateral and longitudinal slip"""
	var lateral_slip = abs(_angular_velocity_y) * 0.1
	var longitudinal_slip = abs(_throttle_input - _brake_input) * 0.5
	
	# Base grip decreases with slip
	for i in range(4):
		var slip_factor = lateral_slip + longitudinal_slip
		_tire_grip_level[i] = max(0.1, 1.0 - slip_factor * 0.5)
	
	# Handbrake reduces rear tire grip significantly
	if _handbrake_active:
		_tire_grip_level[2] *= 0.3
		_tire_grip_level[3] *= 0.3

func _apply_forces_to_vehicle(delta: float) -> void:
	"""Apply calculated forces to the vehicle body"""
	# Calculate forward vector
	var forward = transform.basis.z
	var right = transform.basis.x
	
	# Apply acceleration force
	var accel_force = _acceleration_vector.length()
	if accel_force > 0:
		velocity += forward.normalized() * accel_force * delta
		
	# Apply steering rotation
	if abs(_steering_input) > 0.01 and _velocity_magnitude > 1.0:
		var steer_angle = _steering_input * max_steering_angle
		var rotation_speed = steer_angle * _velocity_magnitude * 0.5
		_angular_velocity_y = rotation_speed * delta
		
		# Rotate vehicle
		var rotation_quat = Quaternion(Vector3.UP, _angular_velocity_y)
		transform = transform * rotation_quat
	
	# Apply drag and rolling resistance
	var drag_coefficient: float = 0.3
	var air_density: float = 1.225
	var frontal_area: float = 2.2
	
	var drag_force = 0.5 * air_density * drag_coefficient * frontal_area * \
		_velocity_magnitude * _velocity_magnitude
	
	velocity -= velocity.normalized() * drag_force * delta
	
	# Rolling resistance
	var rolling_resistance: float = 0.015
	var rr_force = vehicle_mass * _physics.gravity * rolling_resistance
	velocity -= velocity.normalized() * rr_force * delta
	
	# Apply friction when stationary
	if _velocity_magnitude < 0.1:
		velocity = Vector3.ZERO
	
	# Update velocity magnitude
	_velocity_magnitude = velocity.length()
	
	# Update angular velocity
	angular_velocity = Vector3.UP * _angular_velocity_y
	
	# Emit signals
	if speed_changed.is_connected():
		speed_changed.emit(_velocity_magnitude)

func _update_gear_shifting(delta: float) -> void:
	"""Handle automatic/manual gear shifting logic"""
	if _gear_shift_timer > 0:
		_gear_shift_timer -= delta
		return
	
	# Check if shift is needed
	var should_upshift = _should_upshift()
	var should_downshift = _should_downshift()
	
	if should_upshift and _current_gear < transmission_ratios.size() - 1:
		_shift_gear(_current_gear + 1)
	elif should_downshift and _current_gear > 0:
		_shift_gear(_current_gear - 1)

func _should_upshift() -> bool:
	"""Determine if vehicle should upshift"""
	# Upshift if RPM is near redline
	if _engine_rpm > redline_rpm * 0.95:
		return true
	
	# Don't upshift if throttle is high (manual override)
	if _throttle_input > 0.9:
		return false
	
	# Don't upshift below minimum speed
	if _velocity_magnitude < 5.0:
		return false
	
	return false

func _should_downshift() -> bool:
	"""Determine if vehicle should downshift"""
	# Downshift if RPM drops too low
	if _engine_rpm < idle_rpm * 1.2:
		return true
	
	# Downshift if braking heavily
	if _brake_input > 0.7:
		return true
	
	return false

func _shift_gear(new_gear: int) -> void:
	"""Execute gear shift"""
	var old_gear = _current_gear
	_current_gear = new_gear
	
	# Disengage clutch briefly
	_clutch_engaged = false
	_gear_shift_timer = _gear_shift_delay
	
	# Re-engage clutch after delay
	await get_tree().create_timer(_gear_shift_delay).timeout
	_clutch_engaged = true
	
	# Emit signal
	gear_changed.emit(old_gear, new_gear)

func _on_input_changed(action: String, value: float, pressed: bool) -> void:
	"""Handle input events from InputManager"""
	match action:
		"accelerate":
			_throttle_input = value if pressed else 0.0
		"brake":
			_brake_input = value if pressed else 0.0
		"steer_left":
			_steering_input = -value if pressed else 0.0
		"steer_right":
			_steering_input = value if pressed else 0.0
		"handbrake":
			_handbrake_active = pressed

func set_gear_directly(gear_index: int) -> void:
	"""Manually set gear (for AI or manual transmission)"""
	if gear_index < 0 or gear_index >= transmission_ratios.size():
		return
	
	var old_gear = _current_gear
	_current_gear = gear_index
	
	# Simulate clutch operation
	_clutch_engaged = false
	await get_tree().create_timer(0.1).timeout
	_clutch_engaged = true
	
	gear_changed.emit(old_gear, _current_gear)

func get_gear_info() -> Dictionary:
	"""Get current gear information"""
	return {
		"current_gear": _current_gear,
		"total_gears": transmission_ratios.size(),
		"rpm": _engine_rpm,
		"max_rpm": redline_rpm,
		"throttle": _throttle_input,
		"brake": _brake_input,
		"speed_kmh": _velocity_magnitude * 3.6
	}

func reset_vehicle() -> void:
	"""Reset vehicle to initial state"""
	_current_gear = 0
	_engine_rpm = idle_rpm
	_throttle_input = 0.0
	_brake_input = 0.0
	_steering_input = 0.0
	_handbrake_active = false
	_clutch_engaged = true
	_clutch_position = 1.0
	
	# Reset physical state
	velocity = Vector3.ZERO
	angular_velocity = Vector3.ZERO
	_acceleration_vector = Vector3.ZERO
	_angular_velocity_y = 0.0
	_drift_angle = 0.0
	
	# Reset suspension
	for i in range(4):
		_suspension_compression[i] = 0.0
		_suspension_damping[i] = 0.0
		_tire_grip_level[i] = 1.0

func take_damage(damage_amount: float) -> void:
	"""Apply damage to vehicle"""
	damage_taken.emit(damage_amount)
	
	# Reduce performance based on damage
	var damage_factor = 1.0 - min(damage_amount / 1000.0, 0.5)
	
	# Apply temporary effects
	_throttle_input *= damage_factor
	_brake_input *= damage_factor

func enable_debug_visuals(enabled: bool) -> void:
	"""Enable/disable debug visualization"""
	# This would connect to debug rendering systems
	pass

func calculate_optimal_braking_point(track_curvature: float) -> float:
	"""Calculate optimal braking distance based on track curvature"""
	var normal_braking_distance = (_velocity_magnitude * _velocity_magnitude) / (2.0 * 6.0)
	var curvature_factor = 1.0 + abs(track_curvature) * 0.5
	
	return normal_braking_distance * curvature_factor

func simulate_ai_control(throttle_demand: float, steering_demand: float) -> void:
	"""Simulate AI control inputs"""
	_throttle_input = throttle_demand
	_steering_input = steering_demand
	_brake_input = 0.0
	_handbrake_active = false

</file>