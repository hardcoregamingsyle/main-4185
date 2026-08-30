extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulator
## Implements throttle, brake, steering inputs, wheel forces, and gear shifting logic
## All physics values are read from PhysicsSettings resource for centralized tuning
## Copyright 2026 Thalamus Racing Simulator Project

# Signals
signal speed_changed(speed: float)
signal rpm_changed(rpm: float)
signal gear_changed(old_gear: int, new_gear: int)
signal traction_control_state_changed(on: bool)
signal anti_lock_braking_state_changed(on: bool)

# Constants (inherited from PhysicsSettings via autoload)
const GRAVITY: float = PhysicsSettings.gravity
const PHYSICS_TICK_RATE: int = PhysicsSettings.physics_tick_rate
const MAX_SUBSTEPS: int = PhysicsSettings.max_substeps

# Vehicle properties
@export_group("Vehicle Properties")
@export var vehicle_mass: float = PhysicsSettings.default_vehicle_mass
@export var center_of_mass_offset: Vector3 = Vector3.ZERO
@export var aerodynamic_drag_coefficient: float = 0.30
@export var frontal_area: float = 2.2
@export var rolling_resistance_coefficient: float = 0.015
@export var max_engine_torque: float = 450.0  # Nm
@export var max_engine_power: float = 250.0   # kW
@export var idle_rpm: float = 800.0
@export var redline_rpm: float = 7500.0
@export var rev_limit_rpm: float = 8000.0

# Gear ratios
@export_group("Transmission")
@export var number_of_gears: int = 6
@export var final_drive_ratio: float = 3.73
@export var gear_ratios: Array[float] = [3.5, 2.0, 1.4, 1.0, 0.8, 0.6]
@export var reverse_gear_ratio: float = 3.8
@export var clutch_engagement_speed: float = 100.0  # RPM threshold

# Wheel configuration
@export_group("Wheel Configuration")
@export var track_width: float = 1.6
@export var wheel_radius: float = 0.32
@export var wheel_track_front: float = 1.55
@export var wheel_track_rear: float = 1.65
@export var wheel_base: float = 2.7

# Suspension settings
@export_group("Suspension")
@export var suspension_stiffness: float = 80000.0
@export var suspension_damping: float = 5000.0
@export var suspension_compression_limit: float = 0.15
@export var suspension_extension_limit: float = 0.20
@export var unsprung_mass: float = 45.0

# Tire friction model parameters
@export_group("Tire Friction")
@export var tire_friction_stiffness: float = 120000.0
@export var tire_friction_max_force: float = 15000.0
@export var tire_friction_slip_ratio: float = 0.1
@export var tire_friction_slip_angle: float = 0.15

# Input sensitivity
@export_group("Input Sensitivity")
@export var throttle_sensitivity: float = 1.0
@export var brake_sensitivity: float = 1.0
@export var steering_sensitivity: float = 1.0
@export var min_steering_angle: float = 0.35  # radians (~20 degrees)
@export var max_steering_angle: float = 0.75  # radians (~43 degrees)

# Traction control
@export_group("Electronic Aids")
@export var traction_control_enabled: bool = true
@export var traction_control_threshold: float = 0.25  # slip ratio
@export var anti_lock_braking_enabled: bool = true
@export var abs_threshold: float = 0.15  # wheel slip threshold
@export var stability_control_enabled: bool = false

# State tracking
var _speed: float = 0.0  # Speed in m/s
var _rpm: float = 0.0    # Engine RPM
var _current_gear: int = 0  # 0=neutral, 1-6=gears, -1=reverse
var _target_gear: int = 0
var _throttle_input: float = 0.0  # 0.0 to 1.0
var _brake_input: float = 0.0     # 0.0 to 1.0
var _steering_input: float = 0.0  # -1.0 to 1.0
var _clutch_input: float = 0.0    # 0.0 to 1.0
var _handbrake_input: float = 0.0 # 0.0 to 1.0

# Wheel states (front left, front right, rear left, rear right)
var _wheel_states: Array[Dictionary] = []
var _wheel_angular_velocities: Array[float] = []

# Current wheel positions (world space)
var _wheel_positions: Array[Vector3] = []

# Aerodynamics variables
var _air_density: float = 1.225  # kg/m³ at sea level
var _drag_force: float = 0.0
var _downforce_force: float = 0.0

# Suspension compression tracking
var _suspension_compressions: Array[float] = []

# Physics state
var _is_on_ground: bool = false
var _is_drifting: bool = false
var _drift_angle: float = 0.0
var _traction_loss_factor: float = 1.0

# Time tracking
var _last_update_time: float = 0.0
var _delta_accumulator: float = 0.0

# Clutch engagement state
var _clutch_engaged: bool = false
var _clutch_slipping: bool = false

func _ready() -> void:
	_init_wheel_configuration()
	_reset_state()
	_connect_signals()

func _init_wheel_configuration() -> void:
	"""Initialize wheel arrays and positions based on vehicle geometry"""
	_wheel_states.resize(4)
	_wheel_angular_velocities.resize(4)
	_wheel_positions.resize(4)
	_suspension_compressions.resize(4)
	
	for i in range(4):
		_wheel_states[i] = {
			"radius": wheel_radius,
			"unsprung_mass": unsprung_mass,
			"suspension_compression": 0.0,
			"vertical_velocity": 0.0,
			"friction_x": 1.0,
			"friction_y": 1.0,
			"slip_ratio": 0.0,
			"slip_angle": 0.0,
			"applied_force": 0.0
		}
		_wheel_angular_velocities[i] = 0.0

func _reset_state() -> void:
	"""Reset all vehicle state variables to initial values"""
	_speed = 0.0
	_rpm = idle_rpm
	_current_gear = 0
	_target_gear = 0
	_throttle_input = 0.0
	_brake_input = 0.0
	_steering_input = 0.0
	_clutch_input = 0.0
	_handbrake_input = 0.0
	_is_on_ground = false
	_is_drifting = false
	_drift_angle = 0.0
	_traction_loss_factor = 1.0
	_clutch_engaged = false
	_clutch_slipping = false
	_delta_accumulator = 0.0
	_last_update_time = Time.get_ticks_msec() / 1000.0
	
	for i in range(4):
		_wheel_states[i]["slip_ratio"] = 0.0
		_wheel_states[i]["slip_angle"] = 0.0
		_wheel_states[i]["applied_force"] = 0.0

func _connect_signals() -> void:
	"""Connect to GameManager signals for input handling"""
	if GameManager.has_signal("game_started"):
		GameManager.game_started.connect(_on_game_started)
	if GameManager.has_signal("race_reset"):
		GameManager.race_reset.connect(_on_race_reset)

func _on_game_started() -> void:
	_reset_state()

func _on_race_reset() -> void:
	_reset_state()

func _physics_process(delta: float) -> void:
	"""Main physics update loop - runs at fixed timestep"""
	# Handle time accumulation for fixed physics step
	_delta_accumulator += delta
	var physics_step: float = 1.0 / PhysicsSettings.physics_tick_rate
	
	while _delta_accumulator >= physics_step:
		_physics_step(physics_step)
		_delta_accumulator -= physics_step
	
	# Update last update time
	_last_update_time = Time.get_ticks_msec() / 1000.0

func _physics_step(dt: float) -> void:
	"""Execute one physics step with fixed timestep"""
	_update_inputs()
	_update_clutch_state(dt)
	_update_gear_shift_logic()
	_update_engine_state(dt)
	_update_aerodynamics()
	_update_wheels(dt)
	_update_suspension(dt)
	_apply_forces_to_body()
	_update_collision_detection(dt)

func _update_inputs() -> void:
	"""Read and process player input for throttle, brake, steering"""
	# Get input from InputManager singleton
	_throttle_input = InputManager.get_axis(InputManager.AXIS_THROTTLE) * throttle_sensitivity
	_brake_input = InputManager.get_axis(InputManager.AXIS_BRAKE) * brake_sensitivity
	_steering_input = InputManager.get_axis(InputManager.AXIS_STEERING) * steering_sensitivity
	_clutch_input = InputManager.get_axis(InputManager.AXIS_CLUTCH)
	_handbrake_input = InputManager.get_axis(InputManager.AXIS_HANDBRAKE)
	
	# Clamp inputs to valid ranges
	_throttle_input = clamp(_throttle_input, 0.0, 1.0)
	_brake_input = clamp(_brake_input, 0.0, 1.0)
	_steering_input = clamp(_steering_input, -1.0, 1.0)
	_clutch_input = clamp(_clutch_input, 0.0, 1.0)
	_handbrake_input = clamp(_handbrake_input, 0.0, 1.0)

func _update_clutch_state(dt: float) -> void:
	"""Update clutch engagement state based on RPM and input"""
	var engine_speed_diff: float = abs(_rpm - _calculate_wheel_rpm())
	
	if _clutch_input < 0.1:
		# Clutch released (fully engaged)
		if _rpm > clutch_engagement_speed and _speed > 1.0:
			_clutch_engaged = true
			_clutch_slipping = false
		else:
			_clutch_engaged = false
			_clutch_slipping = true
	elif _clutch_input > 0.9:
		# Clutch depressed (disengaged)
		_clutch_engaged = false
		_clutch_slipping = true
	else:
		# Partial engagement - transitional state
		var engagement_progress: float = 1.0 - _clutch_input
		_clutch_engaged = engagement_progress > 0.8
		_clutch_slipping = engagement_progress < 0.9

func _calculate_wheel_rpm() -> float:
	"""Calculate equivalent wheel RPM based on vehicle speed"""
	if _speed <= 0.0:
		return 0.0
	var wheel_circumference: float = PI * 2.0 * wheel_radius
	var wheel_rps: float = _speed / wheel_circumference
	return wheel_rps * 60.0

func _update_gear_shift_logic() -> void:
	"""Handle automatic or manual gear shifting logic"""
	# Auto-shift logic when clutch is engaged
	if _clutch_engaged and _target_gear != _current_gear:
		_perform_gear_shift()
	
	# Determine target gear based on RPM and speed
	_determine_target_gear()

func _determine_target_gear() -> void:
	"""Calculate optimal gear based on current RPM and speed"""
	if _current_gear == 0:
		# Neutral - stay neutral until clutch engages
		return
	
	var desired_gear: int = _current_gear
	
	# Downshift if RPM too high
	if _rpm > redline_rpm:
		desired_gear = max(1, _current_gear - 1)
		_target_gear = desired_gear
		return
	
	# Upshift if RPM too low (except in first gear)
	if _rpm < idle_rpm + 500.0 and _current_gear > 1:
		desired_gear = min(number_of_gears, _current_gear + 1)
		_target_gear = desired_gear
		return
	
	# Maintain current gear
	_target_gear = _current_gear

func _perform_gear_shift() -> void:
	"""Execute gear shift with proper timing"""
	if _target_gear == _current_gear:
		return
	
	var old_gear: int = _current_gear
	_current_gear = _target_gear
	
	# Simulate shift delay
	await get_tree().create_timer(0.15).timeout
	
	gear_changed.emit(old_gear, _current_gear)

func _update_engine_state(dt: float) -> void:
	"""Update engine RPM and torque output based on throttle and gear"""
	var gear_ratio: float = _get_current_gear_ratio()
	var wheel_rpm: float = _calculate_wheel_rpm()
	var drive_wheel_rpm: float = wheel_rpm * final_drive_ratio
	
	# Calculate engine RPM based on gear and wheel speed
	var theoretical_engine_rpm: float = drive_wheel_rpm * gear_ratio
	
	if _clutch_slipping:
		# Engine free to rev independently
		_rpm = _update_free_revving_engine(dt)
	else:
		# Engine coupled to wheels
		_rpm = lerp(_rpm, theoretical_engine_rpm, dt * 30.0)
		_rpm = clamp(_rpm, idle_rpm, rev_limit_rpm)
	
	# Cap RPM at rev limit
	_rpm = min(_rpm, rev_limit_rpm)
	
	# Emit RPM signal
	rpm_changed.emit(_rpm)

func _update_free_revving_engine(dt: float) -> float:
	"""Update engine RPM when clutch is slipping or disengaged"""
	var acceleration_rate: float = _throttle_input * 4000.0  # RPM per second
	var deceleration_rate: float = 800.0  # RPM per second (idle decay)
	
	if _throttle_input > 0.0:
		_rpm += acceleration_rate * dt
	else:
		_rpm = max(_rpm - deceleration_rate * dt, idle_rpm)
	
	return _rpm

func _get_current_gear_ratio() -> float:
	"""Get the effective gear ratio for current gear"""
	if _current_gear == 0:
		return 0.0
	elif _current_gear < 0:  # Reverse
		return reverse_gear_ratio
	else:
		return gear_ratios[_current_gear - 1]

func _update_aerodynamics() -> void:
	"""Calculate aerodynamic drag and downforce forces"""
	var air_speed: float = _speed  # Simplified - could be more complex with wind
	
	# Drag force: Fd = 0.5 * rho * v² * Cd * A
	_drag_force = 0.5 * _air_density * air_speed * air_speed * aerodynamic_drag_coefficient * frontal_area
	
	# Downforce increases with speed squared
	var downforce_coefficient: float = aerodynamic_drag_coefficient * 0.5
	_downforce_force = 0.5 * _air_density * air_speed * air_speed * downforce_coefficient * frontal_area

func _update_wheels(dt: float) -> void:
	"""Update individual wheel states including slip, friction, and forces"""
	var total_force: float = 0.0
	var driving_wheels: Array[int] = [2, 3]  # Rear-wheel drive
	var steering_wheels: Array[int] = [0, 1]  # Front wheels steer
	
	for i in range(4):
		var wheel_state: Dictionary = _wheel_states[i]
		
		# Calculate wheel linear velocity
		var wheel_linear_velocity: float = _calculate_wheel_linear_velocity(i)
		
		# Calculate drive force for driven wheels
		var drive_force: float = 0.0
		if i in driving_wheels and _current_gear != 0:
			drive_force = _calculate_drive_force(wheel_state, dt)
		
		# Calculate braking force
		var brake_force: float = _calculate_brake_force(wheel_state, i, dt)
		
		# Apply forces to wheel
		wheel_state["applied_force"] = drive_force - brake_force
		
		# Calculate slip ratio
		wheel_state["slip_ratio"] = _calculate_wheel_slip(wheel_state, wheel_linear_velocity, dt)
		
		# Calculate lateral slip (for cornering)
		wheel_state["slip_angle"] = _calculate_lateral_slip(i, dt)
		
		# Apply traction control if enabled
		if traction_control_enabled and i in driving_wheels:
			_apply_traction_control(wheel_state, dt)
		
		# Apply ABS if braking
		if anti_lock_braking_enabled and _brake_input > 0.5:
			_apply_abs(wheel_state, i, dt)
		
		total_force += wheel_state["applied_force"]

func _calculate_wheel_linear_velocity(i: int) -> float:
	"""Calculate linear velocity of specific wheel"""
	var wheel_position: Vector3 = _get_wheel_world_position(i)
	var angular_velocity: float = _wheel_angular_velocities[i]
	return wheel_position.z * angular_velocity

func _calculate_drive_force(wheel_state: Dictionary, dt: float) -> float:
	"""Calculate drive force applied to driven wheel"""
	if _current_gear == 0:
		return 0.0
	
	var gear_ratio: float = _get_current_gear_ratio()
	var wheel_radius_m: float = wheel_radius
	
	# Calculate engine torque at current RPM
	var engine_torque: float = _calculate_engine_torque()
	
	# Apply torque to wheel through transmission
	var wheel_torque: float = engine_torque * gear_ratio * final_drive_ratio
	
	# Account for drivetrain efficiency (assume 85%)
	var drivetrain_efficiency: float = 0.85
	wheel_torque *= drivetrain_efficiency
	
	# Convert torque to force
	var drive_force: float = wheel_torque / wheel_radius_m
	
	# Limit force to tire friction capability
	var max_force: float = tire_friction_max_force * wheel_state["friction_x"]
	drive_force = min(drive_force, max_force)
	
	# Apply traction loss factor during drift/loss of grip
	drive_force *= _traction_loss_factor
	
	return drive_force

func _calculate_engine_torque() -> float:
	"""Calculate engine torque output based on RPM"""
	var normalized_rpm: float = (_rpm - idle_rpm) / (redline_rpm - idle_rpm)
	
	# Simple torque curve approximation
	# Peak torque around 4000 RPM
	var peak_torque_rpm: float = 4000.0
	var torque_curve: float
	
	if _rpm <= peak_torque_rpm:
		torque_curve = 1.0 - pow((_rpm - peak_torque_rpm) / peak_torque_rpm, 2.0)
	else:
		torque_curve = 1.0 - pow((_rpm - peak_torque_rpm) / (rev_limit_rpm - peak_torque_rpm), 3.0)
	
	var engine_torque: float = max_engine_torque * torque_curve
	return engine_torque

func _calculate_brake_force(wheel_state: Dictionary, i: int, dt: float) -> float:
	"""Calculate braking force applied to wheel"""
	if _brake_input <= 0.0:
		return 0.0
	
	var brake_pressure: float = _brake_input * _handbrake_input
	var max_brake_force: float = tire_friction_max_force * wheel_state["friction_x"] * 1.5
	var brake_force: float = max_brake_force * brake_pressure
	
	return brake_force

func _calculate_wheel_slip(wheel_state: Dictionary, wheel_linear_velocity: float, dt: float) -> float:
	"""Calculate longitudinal slip ratio for wheel"""
	var wheel_angular_velocity: float = _wheel_angular_velocities[i]
	var wheel_surface_velocity: float = wheel_angular_velocity * wheel_radius
	
	if wheel_surface_velocity == 0.0:
		return 0.0
	
	var slip_ratio: float = (wheel_surface_velocity - wheel_linear_velocity) / max(abs(wheel_linear_velocity), 0.1)
	slip_ratio = clamp(slip_ratio, -1.0, 1.0)
	
	return slip_ratio

func _calculate_lateral_slip(wheel_index: int, dt: float) -> float:
	"""Calculate lateral slip angle for steering wheels"""
	if wheel_index not in [0, 1]:  # Only front wheels
		return 0.0
	
	# Lateral velocity component
	var lateral_velocity: float = _velocity.x
	
	# Longitudinal velocity component
	var longitudinal_velocity: float = _velocity.z
	
	if longitudinal_velocity == 0.0:
		return 0.0
	
	# Slip angle in radians
	var slip_angle: float = atan(lateral_velocity / longitudinal_velocity)
	
	# Add steering angle influence
	var steering_angle: float = _steering_input * max_steering_angle
	slip_angle += steering_angle
	
	return clamp(slip_angle, -max_steering_angle, max_steering_angle)

func _apply_traction_control(wheel_state: Dictionary, dt: float) -> void:
	"""Apply traction control to prevent excessive wheel spin"""
	var slip_ratio: float = abs(wheel_state["slip_ratio"])
	
	if slip_ratio > traction_control_threshold:
		# Reduce drive force proportionally to slip
		var reduction_factor: float = (slip_ratio - traction_control_threshold) / 0.5
		wheel_state["applied_force"] *= (1.0 - reduction_factor * 0.5)
		wheel_state["friction_x"] *= (1.0 - reduction_factor * 0.3)

func _apply_abs(wheel_state: Dictionary, wheel_index: int, dt: float) -> void:
	"""Apply ABS to prevent wheel lockup during hard braking"""
	var slip_ratio: float = wheel_state["slip_ratio"]
	
	if slip_ratio < -abs(abs_lock_braking_threshold):
		# Wheel is locking up - reduce brake force
		var reduction_amount: float = min(abs(slip_ratio) - abs_lock_braking_threshold, 0.5) * 0.5
		wheel_state["applied_force"] *= (1.0 + reduction_amount)

func _update_suspension(dt: float) -> void:
	"""Update suspension compression and vertical dynamics"""
	var gravity_force: float = vehicle_mass * GRAVITY
	var downforce: float = _downforce_force
	var total_vertical_force: float = gravity_force - downforce
	
	# Distribute force across wheels (60% rear, 40% front for typical RWD car)
	var weight_distribution: Vector2 = Vector2(0.4, 0.6)
	
	for i in range(4):
		var wheel_state: Dictionary = _wheel_states[i]
		var wheel_index: int = i % 2  # 0 or 1
		
		# Vertical spring force
		var spring_force: float = suspension_stiffness * _suspension_compressions[i]
		
		# Damping force
		var damping_force: float = suspension_damping * wheel_state["vertical_velocity"]
		
		# Total vertical force on this wheel
		var wheel_load: float = total_vertical_force * weight_distribution[wheel_index] / 2.0
		
		# Update compression
		var net_force: float = wheel_load - spring_force - damping_force
		wheel_state["vertical_velocity"] += (net_force / unsprung_mass) * dt
		_suspension_compressions[i] += wheel_state["vertical_velocity"] * dt
		
		# Clamp compression limits
		_suspension_compressions[i] = clamp(
			_suspension_compressions[i], 
			-suspension_extension_limit, 
			suspension_compression_limit
		)
		
		# Update friction based on load
		wheel_state["friction_x"] = 1.0 + _suspension_compressions[i] * 0.2
		wheel_state["friction_y"] = 1.0 + _suspension_compressions[i] * 0.1

func _apply_forces_to_body() -> void:
	"""Apply calculated forces to the vehicle body"""
	# Apply drag force opposite to motion
	var drag_direction: Vector3 = -_velocity.normalized()
	if drag_direction.length() > 0:
		_velocity += drag_direction * (-_drag_force / vehicle_mass) * 0.016
	
	# Apply downforce vertically
	var downforce_vector: Vector3 = Vector3.DOWN * (_downforce_force / vehicle_mass)
	_velocity += downforce_vector * 0.016
	
	# Apply wheel drive forces to velocity
	var total_drive_force: float = 0.0
	for i in range(4):
		total_drive_force += _wheel_states[i]["applied_force"]
	
	# Drive force along forward direction
	var forward_direction: Vector3 = transform.basis.z * -1.0
	_velocity += forward_direction * (total_drive_force / vehicle_mass) * 0.016
	
	# Gravity effect
	_velocity.y -= GRAVITY * 0.016

func _update_collision_detection(dt: float) -> void:
	"""Detect ground contact and update collision state"""
	var ray_cast_result: RayCast3D = RayCast3D.new()
	add_child(ray_cast_result)
	ray_cast_result.target_position = Vector3(0, -2.0, 0)
	ray_cast_result.collide_with_areas = false
	
	get_tree().root.call_deferred("add_child", ray_cast_result)
	
	var collision_detected: bool = ray_cast_result.is_colliding()
	_is_on_ground = collision_detected
	
	if collision_detected:
		var collision_point: Vector3 = ray_cast_result.get_collision_point()
		var collision_normal: Vector3 = ray_cast_result.get_collision_normal()
		
		# Check if surface is relatively flat (ground detection)
		if collision_normal.y > 0.5:
			_is_on_ground = true
	
	remove_child(ray_cast_result)
	ray_cast_result.queue_free()

func _get_wheel_world_position(wheel_index: int) -> Vector3:
	"""Get world position of specific wheel"""
	var wheel_positions: Array[Vector3] = [
		Vector3(track_width_front * 0.5, 0, wheel_base * 0.25),      # Front Left
		Vector3(-track_width_front * 0.5, 0, wheel_base * 0.25),     # Front Right
		Vector3(track_width * 0.5, 0, -wheel_base * 0.25),           # Rear Left
		Vector3(-track_width * 0.5, 0, -wheel_base * 0.25)           # Rear Right
	]
	
	return wheel_positions[wheel_index]

# Public API methods

func set_throttle(input: float) -> void:
	"""Set throttle input directly (for AI or external control)"""
	_throttle_input = clamp(input, 0.0, 1.0)

func set_brake(input: float) -> void:
	"""Set brake input directly"""
	_brake_input = clamp(input, 0.0, 1.0)

func set_steering(input: float) -> void:
	"""Set steering input directly"""
	_steering_input = clamp(input, -1.0, 1.0)

func set_gear(gear: int) -> void:
	"""Set gear manually (for manual transmission)"""
	if gear >= -1 and gear <= number_of_gears:
		_target_gear = gear
		if _clutch_input < 0.1:
			_current_gear = gear
			gear_changed.emit(_target_gear, _current_gear)

func get_speed() -> float:
	"""Get current vehicle speed in m/s"""
	return _speed

func get_rpm() -> float:
	"""Get current engine RPM"""
	return _rpm

func get_gear() -> int:
	"""Get current gear"""
	return _current_gear

func get_throttle_input() -> float:
	"""Get current throttle input value"""
	return _throttle_input

func get_brake_input() -> float:
	"""Get current brake input value"""
	return _brake_input

func get_steering_input() -> float:
	"""Get current steering input value"""
	return _steering_input

func is_on_ground() -> bool:
	"""Check if vehicle is in contact with ground"""
	return _is_on_ground

func is_drifting() -> bool:
	"""Check if vehicle is in drift state"""
	return _is_drifting

func reset_vehicle() -> void:
	"""Reset vehicle to initial state"""
	_reset_state()

# Helper functions for game integration

func calculate_lap_time() -> float:
	"""Calculate current lap time for this vehicle"""
	# This would integrate with GameManager's lap timing system
	return 0.0

func get_position_data() -> Dictionary:
	"""Get comprehensive vehicle state data"""
	return {
		"position": global_transform.origin,
		"rotation": global_transform.basis.get_euler(),
		"speed": _speed,
		"rpm": _rpm,
		"gear": _current_gear,
		"throttle": _throttle_input,
		"brake": _brake_input,
		"steering": _steering_input,
		"is_on_ground": _is_on_ground,
		"is_drifting": _is_drifting
	}

func apply_damage(damage_amount: float) -> void:
	"""Apply damage to vehicle affecting performance"""
	_traction_loss_factor = max(0.1, 1.0 - (damage_amount * 0.01))
	max_engine_torque = max_engine_torque * (1.0 - (damage_amount * 0.005))
	max_engine_power = max_engine_power * (1.0 - (damage_amount * 0.005))

</FILE>