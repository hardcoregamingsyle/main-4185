extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulation
## Handles throttle, brake, steering, wheel forces, gear shifting, and vehicle dynamics
## Integrates with PhysicsSettings for centralized tuning
## Copyright 2026 Thalamus Racing Simulator Project

signal speed_changed(current_speed: float)
signal rpm_changed(current_rpm: float)
signal gear_changed(new_gear: int)
signal drift_started()
signal drift_ended()
signal collision_impact(impact_force: float, impact_point: Vector3)
signal wheel_slip(wheel_index: int, slip_ratio: float)
signal traction_control_active(active: bool)
signal anti_lock_brake_active(active: bool)

# ============================================================================
# PHYSICS SETTINGS REFERENCE
# ============================================================================

@onready var _physics = PhysicsSettings.get_singleton()

# ============================================================================
# VEHICLE CONFIGURATION
# ============================================================================

@export_group("Vehicle Configuration")
@export var vehicle_mass: float = 1500.0: set = _set_vehicle_mass
@export var center_of_mass_offset: Vector3 = Vector3(0.0, 0.5, 0.0): set = _set_center_of_mass_offset
@export var wheel_base: float = 2.8
@export var track_width: float = 1.8
@export var ground_clearance: float = 0.25
@export var drag_coefficient: float = 0.30
@export var frontal_area: float = 2.2
@export var roll_stiffness_front: float = 12000.0
@export var roll_stiffness_rear: float = 10000.0
@export var camber_angle_front: float = -0.5
@export var camber_angle_rear: float = -0.5
@export var toe_angle_front: float = 0.02
@export var toe_angle_rear: float = 0.02

@export_group("Powertrain Parameters")
@export var engine_max_rpm: float = 7500.0
@export var engine_min_rpm: float = 800.0
@export var idle_rpm: float = 900.0
@export var max_torque: float = 450.0
@export var torque_curve_points: Array[Vector2] = []
@export var gear_ratios: Array[float] = [3.8, 2.2, 1.5, 1.1, 0.9, 0.75, 0.6]
@export var final_drive_ratio: float = 3.5
@export var transmission_efficiency: float = 0.95
@export var clutch_engagement_rpm: float = 1200.0
@export var rev_matching_enabled: bool = true

@export_group("Braking System")
@export var brake_pressure_front: float = 4.0
@export var brake_pressure_rear: float = 3.5
@export var brake_disc_diameter: float = 0.32
@export var brake_pad_friction: float = 0.4
@export var brake_caliper_piston_area: float = 0.005
@export var abs_enabled: bool = true
@export var abs_threshold: float = 0.15
@export var brake_balance_front: float = 0.6
@export var brake_temperature_max: float = 900.0

@export_group("Tire Properties")
@export var tire_stiffness_factor: float = 1.0
@export var tire_friction_static: float = 1.2
@export var tire_friction_dynamic: float = 0.9
@export var tire_width: float = 0.25
@export var tire_radius: float = 0.32
@export var tire_camber_stiffness: float = 5000.0
@export var tire_toe_effectiveness: float = 0.3
@export var tire_temperature_optimal: float = 80.0
@export var tire_temperature_min: float = -10.0
@export var tire_temperature_max: float = 120.0

@export_group("Aerodynamics")
@export var downforce_coefficient: float = 0.5
@export var wing_area: float = 2.0
@export var lift_coefficient: float = -0.1
@export var aerodynamic_center_x: float = 0.5

@export_group("Drivetrain")
@export var drivetrain_type: DrivetrainType = DrivetrainType.FWD
@export var differential_type: DifferentialType = DifferentialType.OPEN
@export var differential_lock_percentage: float = 0.0
@export var clutch_friction: float = 0.35

enum DrivetrainType {
	FWD,
	RWD,
	AWD
}

enum DifferentialType {
	OPEN,
	LSD,
	LOCKED
}

@export_group("Suspension")
@export var suspension_stiffness_front: float = 25000.0
@export var suspension_stiffness_rear: float = 20000.0
@export var suspension_damping_compression_front: float = 1500.0
@export var suspension_damping_compression_rear: float = 1200.0
@export var suspension_damping_rebound_front: float = 1000.0
@export var suspension_damping_rebound_rear: float = 800.0
@export var suspension_travel_limit: float = 0.15
@export var spring_preload_front: float = 5000.0
@export var spring_preload_rear: float = 4000.0

@export_group("Steering")
@export var steering_ratio: float = 15.0
@export var steering_lock_left: float = 45.0
@export var steering_lock_right: float = 45.0
@export var steering_speed: float = 1.0
@export var steering_return_speed: float = 2.0
@export var steering_deadzone: float = 0.05

# ============================================================================
# WHEEL DATA STRUCTURES
# ============================================================================

class WheelData:
	var index: int
	var position_local: Vector3
	var position_world: Vector3
	var normal: Vector3
	var force: Vector3 = Vector3.ZERO
	var angular_velocity: float = 0.0
	var slip_ratio: float = 0.0
	var slip_angle: float = 0.0
	var lateral_force: float = 0.0
	var longitudinal_force: float = 0.0
	var vertical_force: float = 0.0
	var temperature: float = 20.0
	var pressure: float = 2.2
	var wear: float = 0.0
	
	func _init(idx: int, pos: Vector3) -> void:
		index = idx
		position_local = pos
		position_world = pos
		normal = Vector3.UP

# ============================================================================
# INTERNAL STATE
# ============================================================================

var current_gear: int = 1
var current_rpm: float = idle_rpm
var current_speed: float = 0.0
var wheel_speed: float = 0.0
var target_rpm: float = idle_rpm
var target_gear: int = 1
var clutch_state: float = 1.0
var throttle_input: float = 0.0
var brake_input: float = 0.0
var steering_input: float = 0.0
var handbrake_input: float = 0.0

var wheels: Array[WheelData] = []
var wheel_count: int = 4

var drift_angle: float = 0.0
var drift_intensity: float = 0.0
var drift_active: bool = false
var traction_control_active_flag: bool = false
var anti_lock_brake_active_flag: bool = false

var acceleration: Vector3 = Vector3.ZERO
var velocity: Vector3 = Vector3.ZERO
var angular_velocity: Vector3 = Vector3.ZERO

var tire_temperatures: Array[float] = []
var brake_temperatures: Array[float] = []
var clutch_temperature: float = 20.0

var engine_torque_curve: Dictionary = {}
var shift_timer: float = 0.0
var shift_delay: float = 0.3

var _vehicle_body: Node3D = null
var _collision_shape: CollisionShape3D = null
var _wheel_colliders: Array[Node3D] = []

# Time tracking
var _delta_time: float = 0.0
var _frame_count: int = 0

# Input state
var _input_cooldown: float = 0.0
var _max_input_rate: float = 0.1

# ============================================================================
# INITIALIZATION
# ============================================================================

func _ready() -> void:
	_process_mode = ProcessModeEnum.ALWAYS
	_init_wheels()
	_build_torque_curve()
	_setup_collision_shapes()
	_reset_physics_state()

func _physics_process(delta: float) -> void:
	_delta_time = delta
	_frame_count += 1
	
	if _frame_count % _physics.physics_tick_rate == 0:
		_update_physics(delta)
	
	_handle_inputs(delta)
	_update_drivetrain(delta)
	_update_steering(delta)
	_update_suspension(delta)
	_update_aerodynamics(delta)
	_update_tires(delta)
	_update_brakes(delta)
	_check_drift(delta)
	_emit_signals()

func _input(event: InputEvent) -> void:
	if event.is_action_pressed("gear_up"):
		_shift_gear(1)
	elif event.is_action_pressed("gear_down"):
		_shift_gear(-1)
	elif event.is_action_pressed("toggle_abs"):
		abs_enabled = !abs_enabled
		AudioManager.play_sound("ui_click")

func _set_vehicle_mass(value: float) -> void:
	vehicle_mass = clampf(value, 500.0, 5000.0)
	if _vehicle_body != null:
		_vehicle_body.mass = vehicle_mass

func _set_center_of_mass_offset(value: Vector3) -> void:
	center_of_mass_offset = value
	if _vehicle_body != null:
		_vehicle_body.center_of_mass = center_of_mass_offset

# ============================================================================
# WHEEL SETUP
# ============================================================================

func _init_wheels() -> void:
	wheels.clear()
	wheel_count = 4
	
	# Front left wheel
	wheels.append(WheelData.new(0, Vector3(track_width / 2, -ground_clearance, -wheel_base / 2)))
	# Front right wheel
	wheels.append(WheelData.new(1, Vector3(-track_width / 2, -ground_clearance, -wheel_base / 2)))
	# Rear left wheel
	wheels.append(WheelData.new(2, Vector3(track_width / 2, -ground_clearance, wheel_base / 2)))
	# Rear right wheel
	wheels.append(WheelData.new(3, Vector3(-track_width / 2, -ground_clearance, wheel_base / 2)))

	for i in range(wheels.size()):
		tire_temperatures.append(tire_temperature_optimal)
		brake_temperatures.append(20.0)

# ============================================================================
# TORQUE CURVE BUILDING
# ============================================================================

func _build_torque_curve() -> void:
	if torque_curve_points.is_empty():
		# Generate default torque curve based on typical engine characteristics
		for rpm in range(int(engine_min_rpm), int(engine_max_rpm) + 1, int((engine_max_rpm - engine_min_rpm) / 20)):
			var normalized_rpm = (rpm - engine_min_rpm) / (engine_max_rpm - engine_min_rpm)
			var torque_mult = sin(normalized_rpm * PI)
			var torque_value = max_torque * torque_mult
			engine_torque_curve[rpm] = torque_value

func get_engine_torque(rpm: float) -> float:
	rpm = clampf(rpm, engine_min_rpm, engine_max_rpm)
	var rpm_int = int(rpm)
	
	if engine_torque_curve.has(rpm_int):
		return engine_torque_curve[rpm_int]
	
	# Linear interpolation between points
	var lower_rpm = rpm_int - (rpm_int % ((engine_max_rpm - engine_min_rpm) / 20))
	var upper_rpm = lower_rpm + ((engine_max_rpm - engine_min_rpm) / 20)
	
	if not engine_torque_curve.has(lower_rpm):
		lower_rpm = engine_min_rpm
	if not engine_torque_curve.has(upper_rpm):
		upper_rpm = engine_max_rpm
	
	var lower_torque = engine_torque_curve.get(lower_rpm, max_torque * 0.5)
	var upper_torque = engine_torque_curve.get(upper_rpm, max_torque * 0.5)
	
	var t = (rpm - lower_rpm) / (upper_rpm - lower_rpm)
	return lerp(lower_torque, upper_torque, t)

# ============================================================================
# INPUT HANDLING
# ============================================================================

func _handle_inputs(delta: float) -> void:
	_input_cooldown = maxf(0.0, _input_cooldown - delta)
	
	# Throttle input
	throttle_input = Input.get_axis("throttle_negative", "throttle_positive")
	if throttle_input != null:
		throttle_input = smoothstep(-1.0, 1.0, throttle_input)
	else:
		throttle_input = 0.0
	throttle_input = clampf(throttle_input, 0.0, 1.0)
	
	# Brake input
	brake_input = Input.get_axis("brake_negative", "brake_positive")
	if brake_input != null:
		brake_input = smoothstep(-1.0, 1.0, brake_input)
	else:
		brake_input = 0.0
	brake_input = clampf(brake_input, 0.0, 1.0)
	
	# Steering input
	steering_input = Input.get_axis("steer_left", "steer_right")
	if steering_input != null:
		steering_input = smoothstep(-1.0, 1.0, steering_input)
	else:
		steering_input = 0.0
	steering_input = clampf(steering_input, -1.0, 1.0)
	
	# Handbrake
	handbrake_input = Input.get_action_strength("handbrake")
	handbrake_input = clampf(handbrake_input, 0.0, 1.0)

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================

func _shift_gear(direction: int) -> void:
	if _input_cooldown > 0.0:
		return
	
	var new_gear = current_gear + direction
	
	if new_gear < 0:
		new_gear = 0
	elif new_gear >= gear_ratios.size():
		new_gear = gear_ratios.size() - 1
	
	if new_gear != current_gear:
		current_gear = new_gear
		target_gear = current_gear
		emit_signal("gear_changed", current_gear)
		AudioManager.play_sound("gear_shift")
		
		# Rev match if descending
		if direction < 0 and rev_matching_enabled:
			target_rpm = engine_max_rpm * 0.8
		
		_input_cooldown = shift_delay

func _update_drivetrain(delta: float) -> void:
	# Calculate wheel speed based on vehicle speed and tire radius
	wheel_speed = abs(current_speed) / (tire_radius * 2.0 * PI) if tire_radius > 0 else 0.0
	
	# Calculate target RPM based on current gear and wheel speed
	var gear_ratio = gear_ratios[current_gear] if current_gear < gear_ratios.size() else 0.0
	var effective_ratio = gear_ratio * final_drive_ratio
	target_rpm = wheel_speed * effective_ratio * 60.0
	
	# Apply clutch effect
	var actual_ratio = target_rpm * clutch_state
	
	# Update current RPM with inertia and clutch engagement
	var rpm_diff = actual_rpm - current_rpm
	var rpm_change = rpm_diff * delta * 10.0
	current_rpm = clampf(current_rpm + rpm_change, engine_min_rpm, engine_max_rpm)
	
	# Engine braking when no throttle
	if throttle_input <= 0.1:
		current_rpm = maxf(current_rpm, engine_min_rpm)
	
	# Clutch temperature simulation
	clutch_temperature += (current_rpm - target_rpm) * 0.01 * delta
	clutch_temperature = clampf(clutch_temperature, 20.0, 300.0)

func _get_current_torque() -> float:
	var torque = get_engine_torque(current_rpm)
	
	# Apply throttle
	torque *= throttle_input
	
	# Apply transmission efficiency
	torque *= transmission_efficiency
	
	# Apply clutch
	torque *= clutch_state
	
	# Apply differential lock
	if differential_type == DifferentialType.LOCKED:
		torque *= (1.0 + differential_lock_percentage * 0.5)
	elif differential_type == DifferentialType.LSD:
		torque *= (1.0 + differential_lock_percentage * 0.3)
	
	return torque

func _get_wheel_torque_distribution() -> Dictionary:
	# Distribute torque based on drivetrain type
	var distribution = {
		"front_left": 0.0,
		"front_right": 0.0,
		"rear_left": 0.0,
		"rear_right": 0.0
	}
	
	var total_torque = _get_current_torque()
	
	if current_gear == 0:  # Neutral
		return distribution
	
	match drivetrain_type:
		DrivetrainType.FWD:
			distribution.front_left = total_torque * 0.5
			distribution.front_right = total_torque * 0.5
		DrivetrainType.RWD:
			distribution.rear_left = total_torque * 0.5
			distribution.rear_right = total_torque * 0.5
		DrivetrainType.AWD:
			distribution.front_left = total_torque * 0.25
			distribution.front_right = total_torque * 0.25
			distribution.rear_left = total_torque * 0.25
			distribution.rear_right = total_torque * 0.25
	
	return distribution

# ============================================================================
# STEERING SYSTEM
# ============================================================================

func _update_steering(delta: float) -> void:
	var target_steering_angle = steering_input * steering_lock_right
	
	# Apply steering deadzone
	if abs(steering_input) < steering_deadzone:
		target_steering_angle = 0.0
	
	# Smooth steering transition
	var current_steering_angle = 0.0
	if _vehicle_body != null:
		# Get current steering angle from child nodes if they exist
		pass
	
	var steering_diff = target_steering_angle - current_steering_angle
	var steering_speed_factor = steering_speed
	
	# Reduce steering speed when moving slowly
	var speed_factor = minf(abs(current_speed) / 10.0, 1.0)
	steering_speed_factor *= lerp(1.0, 0.3, speed_factor)
	
	current_steering_angle += steering_diff * steering_speed_factor * delta
	
	# Clamp to lock limits
	current_steering_angle = clampf(current_steering_angle, -steering_lock_left, steering_lock_right)
	
	# Return to center when no input
	if abs(steering_input) < steering_deadzone:
		var return_diff = 0.0 - current_steering_angle
		current_steering_angle += return_diff * steering_return_speed * delta
	
	# Store for later use
	_actual_steering_angle = current_steering_angle

func _calculate_steering_angles() -> Dictionary:
	# Calculate individual wheel steering angles using Ackermann geometry
	var ackermann_offset = wheel_base * tan(deg_to_rad(_actual_steering_angle / steering_ratio))
	
	return {
		"front_left": atan((wheel_base / 2 + ackermann_offset) / wheel_base),
		"front_right": atan((wheel_base / 2 - ackermann_offset) / wheel_base),
		"rear_left": 0.0,
		"rear_right": 0.0
	}

# ============================================================================
# SUSPENSION PHYSICS
# ============================================================================

func _update_suspension(delta: float) -> void:
	# Simple suspension model based on body height changes
	var suspension_heights = _calculate_suspension_heights()
	
	for i in range(min(wheels.size(), suspension_heights.size())):
		var wheel = wheels[i]
		var expected_height = suspension_heights[i]
		
		# Suspension compression
		var compression = expected_height - wheel.position_local.y
		compression = clampf(compression, -suspension_travel_limit, suspension_travel_limit)
		
		# Spring force
		var spring_force = compression * suspension_stiffness_front if i < 2 else compression * suspension_stiffness_rear
		spring_force -= spring_preload_front if i < 2 else spring_preload_rear
		
		# Damping force
		var damping_velocity = compression
		var damping_force = damping_velocity * suspension_damping_compression_front if i < 2 else damping_velocity * suspension_damping_compression_rear
		
		# Combine forces
		wheel.vertical_force = maxf(spring_force - damping_force, 0.0)

func _calculate_suspension_heights() -> Array[float]:
	# Calculate expected suspension heights based on vehicle orientation
	var heights = []
	
	for wheel in wheels:
		var height = wheel.position_local.y
		heights.append(height)
	
	return heights

# ============================================================================
# AERODYNAMICS
# ============================================================================

func _update_aerodynamics(delta: float) -> void:
	# Calculate air resistance (drag)
	var air_density = 1.225  # Standard sea level
	var dynamic_pressure = 0.5 * air_density * current_speed * current_speed
	
	var drag_force = dynamic_pressure * drag_coefficient * frontal_area
	
	# Apply drag opposite to velocity direction
	if current_speed > 0.1:
		var drag_direction = -velocity.normalized()
		acceleration -= drag_direction * drag_force / vehicle_mass
	
	# Calculate downforce (negative lift)
	var downforce = dynamic_pressure * downforce_coefficient * wing_area
	
	# Add downforce to vertical forces on wheels
	if downforce > 0.0:
		var front_weight = downforce * 0.6
		var rear_weight = downforce * 0.4
		
		for i in range(wheels.size()):
			if i < 2:  # Front wheels
				wheels[i].vertical_force += front_weight / 2.0
			else:  # Rear wheels
				wheels[i].vertical_force += rear_weight / 2.0

# ============================================================================
# TIRE PHYSICS
# ============================================================================

func _update_tires(delta: float) -> void:
	for i in range(wheels.size()):
		var wheel = wheels[i]
		
		# Calculate slip ratio
		var tire_linear_velocity = wheel.angular_velocity * tire_radius
		var slip_target = tire_linear_velocity / current_speed if current_speed > 0.1 else 1.0
		wheel.slip_ratio = (slip_target - 1.0)
		wheel.slip_ratio = clampf(wheel.slip_ratio, -1.0, 1.0)
		
		# Longitudinal force based on slip
		wheel.longitudinal_force = _calculate_longitudinal_force(i, wheel.slip_ratio)
		
		# Lateral force based on slip angle
		wheel.lateral_force = _calculate_lateral_force(i, wheel.slip_angle)
		
		# Temperature update based on forces
		var force_magnitude = abs(wheel.longitudinal_force) + abs(wheel.lateral_force)
		wheel.temperature += force_magnitude * 0.5 * delta
		wheel.temperature = clampf(wheel.temperature, tire_temperature_min, tire_temperature_max)
		
		# Friction reduction based on temperature
		var temp_factor = 1.0
		if wheel.temperature > tire_temperature_optimal:
			temp_factor = 1.0 - (wheel.temperature - tire_temperature_optimal) / (tire_temperature_max - tire_temperature_optimal)
		
		# Emit slip signal
		if abs(wheel.slip_ratio) > abs_threshold:
			emit_signal("wheel_slip", i, wheel.slip_ratio)

func _calculate_longitudinal_force(wheel_index: int, slip_ratio: float) -> float:
	var stiffness = tire_stiffness_factor * tire_stiffness_factor
	var friction = tire_friction_static
	
	# Simplified Pacejka-style formula
	var longitudinal_force = stiffness * slip_ratio * friction
	
	# Cap force based on vertical load
	var max_force = wheel.vertical_force * friction
	longitudinal_force = clampf(longitudinal_force, -max_force, max_force)
	
	# Apply tire temperature factor
	var temp_factor = 1.0
	if tire_temperatures[wheel_index] > tire_temperature_optimal:
		temp_factor = 1.0 - (tire_temperatures[wheel_index] - tire_temperature_optimal) / (tire_temperature_max - tire_temperature_optimal)
	
	longitudinal_force *= temp_factor
	
	return longitudinal_force

func _calculate_lateral_force(wheel_index: int, slip_angle: float) -> float:
	var stiffness = tire_camber_stiffness * tire_stiffness_factor
	var slip_angle_rad = deg_to_rad(slip_angle)
	
	# Simplified lateral force calculation
	var lateral_force = stiffness * sin(slip_angle_rad) * tire_friction_static
	
	# Cap based on vertical load
	var max_lateral_force = wheel.vertical_force * tire_friction_static
	lateral_force = clampf(lateral_force, -max_lateral_force, max_lateral_force)
	
	return lateral_force

# ============================================================================
# BRAKING SYSTEM
# ============================================================================

func _update_brakes(delta: float) -> void:
	for i in range(wheels.size()):
		var wheel = wheels[i]
		
		# Calculate brake force based on input and wheel index
		var brake_input_per_wheel = brake_input * brake_pressure_front if i < 2 else brake_input * brake_pressure_rear
		
		# Handbrake affects rear wheels more
		if handbrake_input > 0.0:
			brake_input_per_wheel += handbrake_input * 2.0 * brake_pressure_rear * (1.0 if i >= 2 else 0.5)
		
		# ABS check
		if abs_enabled and abs(wheel.slip_ratio) > abs_threshold:
			anti_lock_brake_active_flag = true
			brake_input_per_wheel *= (1.0 - abs(wheel.slip_ratio) * 0.5)
			brake_input_per_wheel = clampf(brake_input_per_wheel, 0.0, brake_input_per_wheel)
		else:
			anti_lock_brake_active_flag = false
		
		# Calculate brake force
		var brake_force = brake_input_per_wheel * brake_pad_friction * brake_caliper_piston_area * wheel.vertical_force
		brake_force *= brake_balance_front if i < 2 else (1.0 - brake_balance_front)
		
		# Apply brake force opposite to wheel rotation
		if current_speed > 0.1:
			wheel.longitudinal_force -= brake_force / tire_radius
		else:
			wheel.longitudinal_force = -brake_force / tire_radius
		
		# Update brake temperature
		brake_temperatures[i] += brake_force * 0.1 * delta
		brake_temperatures[i] = clampf(brake_temperatures[i], 20.0, brake_temperature_max)

# ============================================================================
# DRIFT SYSTEM
# ============================================================================

func _check_drift(delta: float) -> void:
	# Detect drift conditions
	var lateral_acceleration = abs(acceleration.x) if abs(velocity.z) > 1.0 else 0.0
	var cornering_force = abs(drift_angle)
	
	var drift_threshold = 30.0
	var drift_intensity_threshold = 0.5
	
	if cornering_force > drift_threshold and handbrake_input > 0.3:
		if not drift_active:
			emit_signal("drift_started")
			AudioManager.play_sound("drift_start")
		drift_active = true
	漂移_intensity = lerp(drift_intensity, 1.0, delta * 2.0)
	else:
		if drift_active:
			emit_signal("drift_ended")
			AudioManager.play_sound("drift_end")
		drift_active = false
	漂移_intensity = lerp(drift_intensity, 0.0, delta * 2.0)
	
	# Update drift angle
	drift_angle = lerp(drift_angle, steering_input * 15.0, delta * 0.5)

# ============================================================================
# TRACTION CONTROL
# ============================================================================

func _apply_traction_control() -> void:
	traction_control_active_flag = false
	
	for i in range(wheels.size()):
		if drivetrain_type == DrivetrainType.FWD and i >= 2:
			continue
		if drivetrain_type == DrivetrainType.RWD and i < 2:
			continue
		
		var slip_ratio = wheels[i].slip_ratio
		if abs(slip_ratio) > abs_threshold:
			traction_control_active_flag = true
			
			# Reduce torque to this wheel
			var reduction = abs(slip_ratio) * 0.5
			throttle_input *= (1.0 - reduction)

# ============================================================================
# PHYSICS UPDATE
# ============================================================================

func _update_physics(delta: float) -> void:
	# Apply wheel forces to vehicle
	_apply_wheel_forces(delta)
	
	# Update velocity based on acceleration
	velocity += acceleration * delta
	velocity = clamp_vector_magnitude(velocity, 200.0)
	
	# Update position
	position += velocity * delta
	
	# Apply gravity
	acceleration.y -= _physics.gravity
	
	# Reset acceleration for next frame
	acceleration = Vector3.ZERO
	
	# Update gear based on RPM
	_auto_shift_gears(delta)

func _apply_wheel_forces(delta: float) -> void:
	# Apply longitudinal forces
	for i in range(wheels.size()):
		var wheel = wheels[i]
		
		# Drive torque
		var drive_torque = _get_wheel_torque_distribution()
		var wheel_torque = drive_torque["front_left"] if i == 0 else \
						   drive_torque["front_right"] if i == 1 else \
						   drive_torque["rear_left"] if i == 2 else \
						   drive_torque["rear_right"]
		
		# Convert torque to force
		var drive_force = wheel_torque / tire_radius
		
		# Apply to appropriate axis based on vehicle orientation
		var forward_dir = Vector3.FORWARD.rotated(Vector3.UP, _actual_steering_angle)
		
		acceleration += forward_dir * drive_force / vehicle_mass
	
	# Apply lateral forces
	for i in range(wheels.size()):
		var wheel = wheels[i]
		
		var lateral_dir = Vector3.RIGHT.rotated(Vector3.UP, _actual_steering_angle)
		acceleration += lateral_dir * wheel.lateral_force / vehicle_mass

func _auto_shift_gears(delta: float) -> void:
	if target_gear != current_gear:
		shift_timer += delta
		if shift_timer >= shift_delay:
			current_gear = target_gear
			shift_timer = 0.0
			emit_signal("gear_changed", current_gear)
			return
	
	# Automatic upshifting
	if current_rpm > engine_max_rpm * 0.9 and current_gear < gear_ratios.size() - 1:
		target_gear = current_gear + 1
		shift_timer = 0.0
	
	# Automatic downshifting
	elif current_rpm < engine_min_rpm * 0.8 and current_gear > 0:
		target_gear = max(0, current_gear - 1)
		shift_timer = 0.0

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================

func _emit_signals() -> void:
	if abs(current_speed - _last_speed) > 0.5:
		emit_signal("speed_changed", current_speed)
	_last_speed = current_speed
	
	if abs(current_rpm - _last_rpm) > 100.0:
		emit_signal("rpm_changed", current_rpm)
	_last_rpm = current_rpm

func _reset_physics_state() -> void:
	velocity = Vector3.ZERO
	acceleration = Vector3.ZERO
	current_speed = 0.0
	current_rpm = idle_rpm
	current_gear = 1
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	handbrake_input = 0.0
	drift_active = false
	drift_intensity = 0.0
	drift_angle = 0.0
	
	for wheel in wheels:
		wheel.force = Vector3.ZERO
		wheel.angular_velocity = 0.0
		wheel.slip_ratio = 0.0
		wheel.slip_angle = 0.0
		wheel.lateral_force = 0.0
		wheel.longitudinal_force = 0.0
		wheel.vertical_force = 0.0

func _setup_collision_shapes() -> void:
	# Create basic collision shape for vehicle body
	_collision_shape = CollisionShape3D.new()
	_collision_shape.shape = BoxShape3D.new()
	_collision_shape.shape.size = Vector3(track_width, ground_clearance * 2, wheel_base + 0.5)
	add_child(_collision_shape)
	
	# Create wheel collision shapes
	for i in range(wheel_count):
		var wheel_collision = CollisionShape3D.new()
		wheel_collision.shape = CylinderShape3D.new()
		wheel_collision.shape.radius = tire_radius
		wheel_collision.shape.height = tire_width
		wheel_collision.global_position = wheels[i].position_local
		add_child(wheel_collision)
		_wheel_colliders.append(wheel_collision)

func clamp_vector_magnitude(vec: Vector3, max_mag: float) -> Vector3:
	var mag = vec.length()
	if mag > max_mag:
		return vec.normalized() * max_mag
	return vec

# ============================================================================
# PUBLIC API
# ============================================================================

func reset_vehicle() -> void:
	_reset_physics_state()
	position = Vector3.ZERO
	rotation = Vector3.ZERO
	set_physics_material_override(PhysicsSettings.get_default_material())

func get_vehicle_stats() -> Dictionary:
	return {
		"speed": current_speed,
		"rpm": current_rpm,
		"gear": current_gear,
		"throttle": throttle_input,
		"brake": brake_input,
		"steering": steering_input,
		"drift_active": drift_active,
		"drift_intensity": drift_intensity,
		"tires": tire_temperatures.duplicate(),
		"brakes": brake_temperatures.duplicate(),
		"clutch_temp": clutch_temperature
	}

func set_vehicle_config(config: Dictionary) -> void:
	if config.has("mass"):
		vehicle_mass = config.mass
	if config.has("gear_ratios"):
		gear_ratios = config.gear_ratios
	if config.has("final_drive_ratio"):
		final_drive_ratio = config.final_drive_ratio
	if config.has("max_torque"):
		max_torque = config.max_torque
	if config.has("engine_max_rpm"):
		engine_max_rpm = config.engine_max_rpm

func apply_damage(impact_force: float, impact_point: Vector3) -> void:
	emit_signal("collision_impact", impact_force, impact_point)
	
	# Temporary speed reduction
	current_speed *= 0.95
	
	# Visual shake effect
	_shake_camera()

func _shake_camera() -> void:
	# Trigger camera shake through GameManager if available
	if GameManager.instance != null and GameManager.instance.current_state == GameManager.GameState.RACE_ACTIVE:
		GameManager.trigger_shake(0.5, 0.3)

# ============================================================================
# DEBUG VISUALIZATION
# ============================================================================

func _draw_debug_lines() -> void:
	if not _physics.debug_mode:
		return
	
	# Draw wheel forces
	for i in range(wheels.size()):
		var wheel = wheels[i]
		var origin = global_transform * wheel.position_local
		var force_dir = origin + (wheel.force * 0.1)
		get_viewport().debug_draw_line(origin, force_dir, Color.RED, 0.05, true)

# ============================================================================
# SERIALIZATION
# ============================================================================

func save_state() -> Dictionary:
	return {
		"speed": current_speed,
		"rpm": current_rpm,
		"gear": current_gear,
		"position": position,
		"rotation": rotation,
		"velocity": velocity,
		"tires": tire_temperatures.duplicate(),
		"brakes": brake_temperatures.duplicate(),
		"clutch_temp": clutch_temperature,
		"drift_active": drift_active
	}

func load_state(state: Dictionary) -> void:
	current_speed = state.get("speed", 0.0)
	current_rpm = state.get("rpm", idle_rpm)
	current_gear = state.get("gear", 1)
	position = state.get("position", Vector3.ZERO)
	rotation = state.get("rotation", Vector3.ZERO)
	velocity = state.get("velocity", Vector3.ZERO)
	
	if state.has("tires") and state.tires.size() == wheels.size():
		tire_temperatures = state.tires
	if state.has("brakes") and state.brakes.size() == wheels.size():
		brake_temperatures = state.brakes
	if state.has("clutch_temp"):
		clutch_temperature = state.clutch_temp
	if state.has("drift_active"):
		drift_active = state.drift_active

# ============================================================================
# STATIC HELPERS
# ============================================================================

static func calculate_optimal_braking_distance(speed: float, friction: float) -> float:
	return (speed * speed) / (2.0 * friction * PhysicsSettings.get_singleton().gravity)

static func calculate_cornering_force(radius: float, speed: float, mass: float) -> float:
	return (mass * speed * speed) / radius

static func calculate_air_resistance(speed: float, cd: float, area: float, density: float = 1.225) -> float:
	return 0.5 * density * speed * speed * cd * area

# ============================================================================
# SIGNAL CONNECTIONS
# ============================================================================

func _connect_signals() -> void:
	speed_changed.connect(_on_speed_changed)
	rpm_changed.connect(_on_rpm_changed)
	gear_changed.connect(_on_gear_changed)
	collision_impact.connect(_on_collision_impact)

func _on_speed_changed(new_speed: float) -> void:
	pass

func _on_rpm_changed(new_rpm: float) -> void:
	pass

func _on_gear_changed(new_gear: int) -> void:
	pass

func _on_collision_impact(force: float, point: Vector3) -> void:
	pass

# ============================================================================
# END OF FILE
# ============================================================================
</FILE "scripts/vehicles/VehicleController.gd">