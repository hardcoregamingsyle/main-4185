extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulation
## Handles throttle, brake, steering, wheel forces, gear shifting, and vehicle dynamics
## Integrates with PhysicsSettings, Powertrain, and InputManager systems
## Copyright 2026 Thalamus Racing Simulator Project

signal speed_changed(current_speed: float)
signal rpm_changed(rpm: float)
signal gear_changed(old_gear: int, new_gear: int)
signal drift_started()
signal drift_ended()
signal collision_detected(collision_info: Dictionary)
signal lap_completed(lap_data: Dictionary)
signal race_finished(position: int, time: float)
signal engine_stalled()
signal traction_control_triggered()
signal anti_lock_braking_triggered()

# ============================================================================
# INPUT VALUES (from InputManager)
# ============================================================================

@export var _throttle_input: float = 0.0: set = _set_throttle_input
@export var _brake_input: float = 0.0: set = _set_brake_input
@export var _steering_input: float = 0.0: set = _set_steering_input
@export var _clutch_input: float = 0.0: set = _set_clutch_input
@export var _handbrake_input: float = 0.0: set = _set_handbrake_input
@export var _gear_up_input: bool = false
@export var _gear_down_input: bool = false
@export var _nitrous_input: bool = false

var _last_throttle: float = 0.0
var _last_brake: float = 0.0
var _last_steering: float = 0.0
var _input_buffer_time: float = 0.0
const INPUT_BUFFER_DURATION: float = 0.15

# ============================================================================
# VEHICLE PHYSICS STATE
# ============================================================================

var current_speed: float = 0.0  # Speed in m/s (positive forward, negative reverse)
var max_forward_speed: float = 85.0  # Max forward speed m/s (306 km/h)
var max_reverse_speed: float = 45.0  # Max reverse speed m/s (162 km/h)
var acceleration: float = 0.0
var deceleration: float = 0.0

var rotation_velocity: float = 0.0  # Yaw rate rad/s
var slip_angle: float = 0.0  # Tire slip angle degrees
var lateral_acceleration: float = 0.0  # G-force lateral
var longitudinal_acceleration: float = 0.0  # G-force longitudinal

var velocity_vector: Vector3 = Vector3.ZERO
var ground_normal: Vector3 = Vector3.UP
var is_on_ground: bool = true
var is_drifting: bool = false
var drift_intensity: float = 0.0

var _velocity_history: Array[float] = []
var _history_window_size: int = 10

# ============================================================================
# GEARBOX SYSTEM
# ============================================================================

var current_gear: int = 0  # 0 = Neutral, 1-7 = Forward gears, -1 = Reverse
var target_gear: int = 0
var gear_shift_progress: float = 0.0  # 0.0 to 1.0 during shift
var is_shifting: bool = false
var shift_duration: float = 0.25  # Seconds per gear change
var shift_timer: float = 0.0

# Gear ratios (final drive included)
var gear_ratios: Array[float] = [0.0, 3.8, 2.4, 1.8, 1.4, 1.1, 0.9, 0.75]
var reverse_ratio: float = 3.5
var final_drive_ratio: float = 3.73

# Engine characteristics
var _rpm: float = 800.0  # Current engine RPM
var idle_rpm: float = 800.0
var redline_rpm: float = 7200.0
var rev_limiter_rpm: float = 7400.0
var optimal_power_rpm: float = 5500.0
var torque_curve: Array[float] = []
var power_curve: Array[float] = []
var torque_at_idle: float = 120.0  # Nm
var peak_torque: float = 450.0  # Nm
var peak_power: float = 350.0  # kW

var is_reving: bool = false
var rev_limit_active: bool = false
var clutch_engaged: bool = true
var last_shift_direction: int = 0  # 1 up, -1 down, 0 none

# ============================================================================
# WHEEL CONFIGURATION
# ============================================================================

const NUM_WHEELS: int = 4
var wheel_positions: Array[Vector3] = []
var wheel_radii: Array[float] = [0.33, 0.33, 0.33, 0.33]
var wheel_forces: Array[float] = [0.0, 0.0, 0.0, 0.0]
var wheel_rotation_angles: Array[float] = [0.0, 0.0, 0.0, 0.0]
var wheel_suspension_compression: Array[float] = [0.0, 0.0, 0.0, 0.0]

# Wheel indices: Front Left, Front Right, Rear Left, Rear Right
const WHEEL_FL: int = 0
const WHEEL_FR: int = 1
const WHEEL_RL: int = 2
const WHEEL_RR: int = 3

# Track positions (in local vehicle space)
var _track_width_front: float = 1.6
var _track_width_rear: float = 1.6
var _wheelbase: float = 2.8

# Suspension settings
var suspension_stiffness: float = 150000.0  # N/m
var damping_rate: float = 12000.0  # Ns/m
var static_compression: float = 0.15  # Resting position
var max_compression: float = 0.3
var max_extension: float = 0.4
var ride_height: float = 0.3

# Steering geometry
var max_steering_angle: float = 35.0 * TAU / 180.0  # Radians
var steering_rate: float = 4.5  # Steer angle per second (radians)
var current_steering_angle: float = 0.0

# ============================================================================
# DRIFT & TRACTION CONTROL
# ============================================================================

var drift_threshold: float = 15.0  # Slip angle threshold for drift (degrees)
var drift_recovery_rate: float = 2.0  # Degrees per second
var drift_gain_rate: float = 0.5  # Degrees per second when accelerating
var minimum_drift_speed: float = 20.0  # Minimum speed to maintain drift (m/s)

var traction_control_active: bool = false
var traction_control_level: int = 2  # 0=off, 1=low, 2=medium, 3=high
var wheel_slip_ratios: Array[float] = [0.0, 0.0, 0.0, 0.0]
var locked_wheels_count: int = 0

# ============================================================================
# AERODYNAMICS
# ============================================================================

var drag_coefficient: float = 0.32
var frontal_area: float = 2.2  # m^2
var air_density: float = 1.225  # kg/m^3
var downforce_coefficient: float = 0.8
var downforce_balance: float = 0.5  # 0=front biased, 1=rear biased
var aerodynamic_lift: float = 0.0
var wing_angle: float = 10.0 * TAU / 180.0  # Radians

# ============================================================================
# CAR BODY PROPERTIES
# ============================================================================

var mass: float = 1500.0  # kg
var moment_of_inertia: float = 2800.0  # kg*m^2 about Z axis
var center_of_mass: Vector3 = Vector3(0.0, 0.3, 0.0)
var weight_distribution_front: float = 0.45  # Fraction on front wheels
var weight_distribution_rear: float = 0.55  # Fraction on rear wheels

# ============================================================================
# NITROUS/OXYGEN SYSTEM
# ============================================================================

var nitrous_available: bool = true
var nitrous_capacity: float = 10.0  # Seconds of boost
var nitrous_current: float = 10.0  # Remaining seconds
var nitrous_boost_factor: float = 1.35  # 35% power increase
var nitrous_decay_rate: float = 1.0  # Per second usage
var nitrous_recharge_time: float = 30.0  # Seconds to recharge
var nitrous_cooldown: float = 0.0
var nitrous_active: bool = false

# ============================================================================
# LAP & RACE DATA
# ============================================================================

var lap_number: int = 0
var lap_start_time: float = 0.0
var lap_times: Array[float] = []
var best_lap_time: float = 0.0
var current_lap_time: float = 0.0
var race_position: int = 1
var race_distance: float = 0.0
var total_race_time: float = 0.0
var checkpoints_passed: Array[int] = []
var last_checkpoint: int = -1

# ============================================================================
# POWERTRAIN REFERENCE
# ============================================================================

var powertrain: Node = null

# ============================================================================
# DEBUG & VISUALIZATION
# ============================================================================

@export_group("Debug Visualization")
@export var debug_show_force_vectors: bool = false
@export var debug_show_wheel_data: bool = false
@export var debug_show_vehicle_lines: bool = false

var _debug_lines: Array[Line3D] = []

# ============================================================================
# INITIALIZATION
# ============================================================================

func _ready() -> void:
	_init_torque_curve()
	_init_power_curve()
	_setup_wheels()
	_connect_signals()
	_init_debug_visualization()
	
	# Apply default values from PhysicsSettings if available
	var phys_settings = PhysicsSettings.get_singleton()
	if phys_settings != null:
		mass = phys_settings.default_vehicle_mass
	
	set_process(true)
	set_physics_process(true)

func _physics_process(delta: float) -> void:
	if not is_on_ground:
		_update_air_physics(delta)
		return
	
	_update_ground_physics(delta)
	_update_aerodynamics(delta)
	_check_drift_state()
	_handle_nitrous(delta)
	_update_debug_visualization(delta)

func _input(event: InputEvent) -> void:
	if event is InputEventKey and event.pressed:
		if event.keycode == KEY_UP or event.keycode == KEY_W:
			_throttle_input = clampf(_throttle_input + 0.01, 0.0, 1.0)
		elif event.keycode == KEY_DOWN or event.keycode == KEY_S:
			_brake_input = clampf(_brake_input + 0.01, 0.0, 1.0)
		elif event.keycode == KEY_SPACE:
			_handbrake_input = clampf(_handbrake_input + 0.01, 0.0, 1.0)

# ============================================================================
# TORQUE & POWER CURVE INITIALIZATION
# ============================================================================

func _init_torque_curve() -> void:
	"""Initialize engine torque curve based on RPM"""
	torque_curve.clear()
	var num_points: int = 20
	for i in range(num_points):
		var rpm_percent: float = float(i) / (num_points - 1)
		var rpm_val: float = idle_rpm + (rev_limiter_rpm - idle_rpm) * rpm_percent
		
		# Simulated torque curve - typical naturally aspirated engine profile
		var torque_val: float = 0.0
		if rpm_val <= idle_rpm:
			torque_val = torque_at_idle
		elif rpm_val <= optimal_power_rpm:
			# Linear ramp to peak torque
			torque_val = torque_at_idle + (peak_torque - torque_at_idle) * ((rpm_val - idle_rpm) / (optimal_power_rpm - idle_rpm))
		elif rpm_val <= rev_limiter_rpm:
			# Gradual decline after peak
			var decline_factor: float = minf((rpm_val - optimal_power_rpm) / (rev_limiter_rpm - optimal_power_rpm), 1.0)
			torque_val = peak_torque * (1.0 - decline_factor * 0.3)
		
		torque_curve.append(torque_val)

func _init_power_curve() -> void:
	"""Initialize power curve based on torque curve"""
	power_curve.clear()
	var num_points: int = 20
	for i in range(num_points):
		var torque_val: float = torque_curve[i]
		var rpm_val: float = idle_rpm + (rev_limiter_rpm - idle_rpm) * (float(i) / (num_points - 1))
		
		# Power = Torque * RPM / constant
		var power_val: float = torque_val * rpm_val * 0.000095493  # Convert to kW
		power_curve.append(power_val)

# ============================================================================
# WHEEL SETUP
# ============================================================================

func _setup_wheels() -> void:
	"""Setup wheel positions and initial state"""
	wheel_positions.resize(NUM_WHEELS)
	wheel_forces.resize(NUM_WHEELS)
	wheel_rotation_angles.resize(NUM_WHEELS)
	wheel_suspension_compression.resize(NUM_WHEELS)
	wheel_slip_ratios.resize(NUM_WHEELS)
	
	# Calculate wheel positions in local vehicle space
	# Front Left
	wheel_positions[WHEEL_FL] = Vector3(-_track_width_front / 2.0, -ride_height, _wheelbase / 2.0)
	# Front Right
	wheel_positions[WHEEL_FR] = Vector3(_track_width_front / 2.0, -ride_height, _wheelbase / 2.0)
	# Rear Left
	wheel_positions[WHEEL_RL] = Vector3(-_track_width_rear / 2.0, -ride_height, -_wheelbase / 2.0)
	# Rear Right
	wheel_positions[WHEEL_RR] = Vector3(_track_width_rear / 2.0, -ride_height, -_wheelbase / 2.0)
	
	# Initialize compression to static
	for i in range(NUM_WHEELS):
		wheel_suspension_compression[i] = static_compression

# ============================================================================
# SIGNAL CONNECTIONS
# ============================================================================

func _connect_signals() -> void:
	"""Connect to GameManager signals for race data"""
	if GameManager.has_signal("race_started"):
		GameManager.race_started.connect(_on_race_started)
	if GameManager.has_signal("lap_completed"):
		GameManager.lap_completed.connect(_on_lap_completed)

func _on_race_started(race_data: Dictionary) -> void:
	"""Handle race start events"""
	lap_number = 0
	lap_times.clear()
	checkpoints_passed.clear()
	last_checkpoint = -1
	current_lap_time = 0.0
	race_position = 1
	total_race_time = 0.0
	nitrous_current = nitrous_capacity

func _on_lap_completed(lap_data: Dictionary) -> void:
	"""Handle lap completion"""
	pass

# ============================================================================
# INPUT HANDLING
# ============================================================================

func _set_throttle_input(value: float) -> void:
	_throttle_input = clampf(value, 0.0, 1.0)

func _set_brake_input(value: float) -> void:
	_brake_input = clampf(value, 0.0, 1.0)

func _set_steering_input(value: float) -> void:
	_steering_input = clampf(value, -1.0, 1.0)

func _set_clutch_input(value: float) -> void:
	_clutch_input = clampf(value, 0.0, 1.0)

func _set_handbrake_input(value: float) -> void:
	_handbrake_input = clampf(value, 0.0, 1.0)

func _update_inputs_from_manager() -> void:
	"""Update input values from InputManager singleton"""
	if InputManager.is_action_pressed("throttle"):
		_throttle_input = lerp(_throttle_input, 1.0, 0.1)
	else:
		_throttle_input = lerp(_throttle_input, 0.0, 0.1)
	
	if InputManager.is_action_pressed("brake"):
		_brake_input = lerp(_brake_input, 1.0, 0.1)
	else:
		_brake_input = lerp(_brake_input, 0.0, 0.1)
	
	if InputManager.is_action_pressed("steering_left"):
		_steering_input = lerp(_steering_input, -1.0, 0.1)
	elif InputManager.is_action_pressed("steering_right"):
		_steering_input = lerp(_steering_input, 1.0, 0.1)
	else:
		_steering_input = lerp(_steering_input, 0.0, 0.1)
	
	if InputManager.is_action_pressed("clutch"):
		_clutch_input = lerp(_clutch_input, 1.0, 0.1)
	else:
		_clutch_input = lerp(_clutch_input, 0.0, 0.1)
	
	if InputManager.is_action_pressed("handbrake"):
		_handbrake_input = lerp(_handbrake_input, 1.0, 0.1)
	else:
		_handbrake_input = lerp(_handbrake_input, 0.0, 0.1)
	
	# Check for gear shift inputs
	if _gear_up_input and _last_throttle < 0.1:
		shift_gear(1)
		_gear_up_input = false
	if _gear_down_input and _last_throttle < 0.1:
		shift_gear(-1)
		_gear_down_input = false

func update_from_input_manager() -> void:
	"""Public method for InputManager to call"""
	_update_inputs_from_manager()

# ============================================================================
# GEAR SHIFTER LOGIC
# ============================================================================

func get_current_gear() -> int:
	return current_gear

func shift_gear(direction: int) -> void:
	"""Shift gears by direction (-1 down, 1 up)"""
	if is_shifting:
		return
	
	target_gear = current_gear + direction
	
	# Boundary checks
	if target_gear < 0:
		target_gear = 0
	elif target_gear > gear_ratios.size() - 1:
		target_gear = gear_ratios.size() - 1
	
	# Don't shift directly into reverse from forward without going through neutral
	if (current_gear > 0 and target_gear < 0) or (current_gear < 0 and target_gear > 0):
		target_gear = 0
	
	is_shifting = true
	gear_shift_progress = 0.0
	shift_timer = shift_duration
	last_shift_direction = direction
	
	emit_signal("gear_changed", current_gear, target_gear)
	current_gear = target_gear

func _update_gear_shifting(delta: float) -> void:
	"""Update gear shifting animation/state"""
	if not is_shifting:
		return
	
	gear_shift_progress += delta / shift_duration
	
	if gear_shift_progress >= 1.0:
		gear_shift_progress = 1.0
		is_shifting = false
		emit_signal("gear_changed", current_gear, target_gear)

func get_gear_ratio() -> float:
	"""Get current gear ratio including final drive"""
	if current_gear == 0:
		return 0.0
	elif current_gear == -1:
		return reverse_ratio * final_drive_ratio
	else:
		return gear_ratios[current_gear] * final_drive_ratio

func get_rpm_at_speed(speed: float) -> float:
	"""Calculate RPM based on vehicle speed and current gear"""
	if current_gear == 0 or current_gear == -1:
		return idle_rpm
	
	var wheel_radius: float = wheel_radii[WHEEL_RL]
	var wheel_speed: float = abs(speed) / wheel_radius
	var axle_speed: float = wheel_speed / final_drive_ratio
	var engine_speed: float = axle_speed * gear_ratios[current_gear]
	
	return maxf(engine_speed, idle_rpm)

func _update_engine_rpm(delta: float) -> void:
	"""Update engine RPM based on driving conditions"""
	var wheel_radius: float = wheel_radii[WHEEL_RL]
	var wheel_linear_speed: float = abs(current_speed)
	var wheel_angular_speed: float = wheel_linear_speed / wheel_radius
	
	if current_gear == 0:
		# In neutral, let engine idle or rev freely
		if _throttle_input > 0.1:
			_rpm = lerp(_rpm, idle_rpm + (_throttle_input * 3000.0), 0.1)
		else:
			_rpm = lerp(_rpm, idle_rpm, 0.05)
		return
	
	var final_drive: float = final_drive_ratio
	if current_gear == -1:
		final_drive *= reverse_ratio
	else:
		final_drive *= gear_ratios[current_gear]
	
	var target_rpm: float = wheel_angular_speed * final_drive
	
	# If clutch engaged, follow wheel speed
	if clutch_engaged:
		_rpm = lerp(_rpm, target_rpm, 0.1)
	else:
		# Clutch disengaged, allow engine to rev independently
		if _throttle_input > 0.1:
			_rpm = lerp(_rpm, _rpm + (_throttle_input * 500.0), 0.2)
		else:
			_rpm = lerp(_rpm, idle_rpm, 0.05)
	
	# Rev limiter
	if _rpm >= rev_limiter_rpm:
		rev_limit_active = true
		_rpm = lerp(_rpm, rev_limiter_rpm, 0.1)
	else:
		rev_limit_active = false
	
	# Clamp to valid range
	_rpm = clampf(_rpm, idle_rpm, rev_limiter_rpm + 200.0)
	
	emit_signal("rpm_changed", _rpm)

# ============================================================================
# ACCELERATION & VELOCITY CALCULATION
# ============================================================================

func calculate_acceleration() -> float:
	"""Calculate current vehicle acceleration based on engine power and resistance"""
	if current_gear == 0:
		return 0.0
	
	var wheel_radius: float = wheel_radii[WHEEL_RL]
	var gear_ratio: float = get_gear_ratio()
	var engine_torque: float = _get_engine_torque()
	
	# Force at wheels = Torque * Gear Ratio / Wheel Radius
	var drive_force: float = engine_torque * gear_ratio / wheel_radius
	
	# Apply clutch effect
	if not clutch_engaged:
		drive_force *= 0.1
	
	# Apply nitrous boost
	if nitrous_active:
		drive_force *= nitrous_boost_factor
	
	# Apply traction control reduction
	if traction_control_active:
		drive_force *= _get_tc_reduction_factor()
	
	# Air resistance (drag)
	var drag_force: float = 0.5 * drag_coefficient * frontal_area * air_density * current_speed * current_speed
	if current_speed < 0:
		drag_force = -drag_force
	
	# Rolling resistance
	var rolling_resistance: float = mass * 9.81 * 0.015
	
	# Net force
	var net_force: float = drive_force - drag_force - rolling_resistance
	
	# Acceleration = Force / Mass
	acceleration = net_force / mass
	
	# Cap acceleration to realistic limits
	acceleration = clampf(acceleration, -15.0, 12.0)
	
	longitudinal_acceleration = acceleration / 9.81  # Convert to G-forces
	
	return acceleration

func _get_engine_torque() -> float:
	"""Get torque value at current RPM"""
	# Normalize RPM to curve index
	var rpm_normalized: float = (_rpm - idle_rpm) / (rev_limiter_rpm - idle_rpm)
	rpm_normalized = clampf(rpm_normalized, 0.0, 1.0)
	
	var index: int = int(rpm_normalized * (torque_curve.size() - 1))
	index = clampf(index, 0, torque_curve.size() - 1)
	
	return torque_curve[index]

func _get_tc_reduction_factor() -> float:
	"""Get traction control reduction factor based on level"""
	match traction_control_level:
		0: return 1.0  # Off
		1: return 0.92  # Low
		2: return 0.85  # Medium
		_: return 0.75  # High
	return 1.0

# ============================================================================
# GROUND PHYSICS UPDATE
# ============================================================================

func _update_ground_physics(delta: float) -> void:
	"""Update physics while on ground"""
	_update_gear_shifting(delta)
	_update_engine_rpm(delta)
	
	# Get current acceleration
	var accel: float = calculate_acceleration()
	
	# Handle braking
	var brake_force: float = _brake_input * 15.0 * mass  # Strong braking force
	if _handbrake_input > 0.1:
		brake_force *= 1.3  # Handbrake adds extra braking
	
	# Apply acceleration
	if current_gear != 0 and is_shifting == false:
		velocity.x += accel * delta
	else:
		# Decelerate in neutral
		velocity.x *= 0.99
	
	# Apply braking
	velocity.x -= brake_force / mass * delta
	
	# Cap speeds
	if velocity.x > 0:
		velocity.x = minf(velocity.x, max_forward_speed)
	else:
		velocity.x = maxf(velocity.x, -max_reverse_speed)
	
	# Update velocity vector
	velocity_vector = velocity
	current_speed = velocity.length()
	
	# Steering
	_update_steering(delta)
	
	# Apply steering rotation to body
	var steer_rot: float = current_steering_angle
	body_rotation.y = steer_rot * 0.3
	
	# Calculate lateral acceleration
	lateral_acceleration = (body_rotation.y * current_speed) / 9.81
	
	# Update speed signal
	if abs(current_speed - _last_throttle) > 0.5:
		emit_signal("speed_changed", current_speed)
	
	_last_throttle = current_speed

func _update_steering(delta: float) -> void:
	"""Update steering angle based on input"""
	var target_steering: float = _steering_input * max_steering_angle
	
	# Smooth steering transition
	current_steering_angle = lerp(current_steering_angle, target_steering, delta * steering_rate)
	
	# Update wheel rotation angles
	wheel_rotation_angles[WHEEL_FL] = current_steering_angle
	wheel_rotation_angles[WHEEL_FR] = current_steering_angle

# ============================================================================
# AERODYNAMICS
# ============================================================================

func _update_aerodynamics(delta: float) -> void:
	"""Calculate aerodynamic forces"""
	var speed_squared: float = current_speed * current_speed
	
	# Drag force (opposes motion)
	var drag_force: float = 0.5 * drag_coefficient * frontal_area * air_density * speed_squared
	if current_speed < 0:
		drag_force = -drag_force
	
	# Downforce (pushes car down)
	var downforce: float = 0.5 * downforce_coefficient * frontal_area * air_density * speed_squared
	var downforce_front: float = downforce * downforce_balance
	var downforce_rear: float = downforce * (1.0 - downforce_balance)
	
	# Adjust grip based on downforce
	# This would affect friction calculations in real physics
	# For now, just store the values for visualization/debugging

# ============================================================================
# AIR PHYSICS (when off ground)
# ============================================================================

func _update_air_physics(delta: float) -> void:
	"""Update physics when airborne"""
	# Gravity
	velocity.y -= 9.81 * delta
	
	# Air resistance
	var drag: float = 0.5 * drag_coefficient * frontal_area * air_density * current_speed * current_speed
	velocity = velocity.normalized() * (velocity.length() - drag / mass * delta)
	
	# Airborne doesn't affect engine much
	_update_engine_rpm(delta)

# ============================================================================
# DRIFT DETECTION
# ============================================================================

func _check_drift_state() -> void:
	"""Check if vehicle is drifting based on slip angle and conditions"""
	slip_angle = lateral_acceleration * 10.0  # Simplified slip calculation
	
	if current_speed > minimum_drift_speed:
		if abs(slip_angle) > drift_threshold and _handbrake_input > 0.3:
			if not is_drifting:
				is_drifting = true
				emit_signal("drift_started")
			
			# Increase drift intensity
			if _throttle_input > 0.5:
			 drift_intensity = lerp(drift_intensity, 1.0, delta * drift_gain_rate)
		else:
			if is_drifting:
				is_drifting = false
				emit_signal("drift_ended")
			 drift_intensity = lerp(drift_intensity, 0.0, delta * drift_recovery_rate)
		 else:
		 drift_intensity = 0.0

# ============================================================================
# NITROUS SYSTEM
# ============================================================================

func _handle_nitrous(delta: float) -> void:
	"""Handle nitrous system logic"""
	if nitrous_current <= 0.0:
		nitrous_active = false
		return
	
	if InputManager.is_action_pressed("nitrous") and nitrous_current > 0.0:
		nitrous_active = true
		nitrous_current -= delta * nitrous_decay_rate
	else:
		nitrous_active = false
		
		# Recharge over time
		if nitrous_current < nitrous_capacity:
			nitrous_current += delta * (nitrous_capacity / nitrous_recharge_time)
	
	nitrous_current = clampf(nitrous_current, 0.0, nitrous_capacity)

# ============================================================================
# TRACTION CONTROL & ABS
# ============================================================================

func _update_traction_control() -> void:
	"""Update traction control and anti-lock braking"""
	var drive_wheel_slip: float = 0.0
	
	# Calculate wheel slip ratios
	for i in range(NUM_WHEELS):
		var wheel_speed: float = abs(velocity.x) / wheel_radii[i]
		var engine_wheel_speed: float = _rpm / gear_ratios[current_gear] / final_drive_ratio
		wheel_slip_ratios[i] = abs(wheel_speed - engine_wheel_speed) / maxf(wheel_speed, 1.0)
	
	# Detect wheel lockup for ABS
	locked_wheels_count = 0
	for i in range(NUM_WHEELS):
		if wheel_slip_ratios[i] > 0.8 and _brake_input > 0.3:
			locked_wheels_count += 1
	
	if locked_wheels_count > 0:
		_brake_input *= 0.8  # Reduce braking
		emit_signal("anti_lock_braking_triggered")
	
	# Traction control on acceleration
	if _throttle_input > 0.5 and current_gear > 0:
		var max_slip: float = 0.2 - (traction_control_level * 0.05)
		for i in range(NUM_WHEELS):
			if wheel_slip_ratios[i] > max_slip:
				traction_control_active = true
				emit_signal("traction_control_triggered")
				break
	else:
		traction_control_active = false

# ============================================================================
# COLLISION HANDLING
# ============================================================================

func handle_collision(info: CollisionInfo3D) -> void:
	"""Handle collision events"""
	var collision_data: Dictionary = {
		"collision_point": info.position,
		"normal": info.normal,
		"impulse": info.impulse.length(),
		"collider": info.collider.name if info.collider else "unknown"
	}
	
	emit_signal("collision_detected", collision_data)

func _on_body_entered(body: Node) -> void:
	"""Called when another body enters collision with this vehicle"""
	handle_collision(null)

# ============================================================================
# LAP & RACE TRACKING
# ============================================================================

func _update_lap_timing(delta: float) -> void:
	"""Update lap timing data"""
	if GameManager.current_state != GameManager.GameState.RACE_ACTIVE:
		return
	
	total_race_time += delta
	current_lap_time += delta

func record_checkpoint(checkpoint_id: int) -> void:
	"""Record passing through a checkpoint"""
	if checkpoint_id > last_checkpoint:
		checkpoints_passed.append(checkpoint_id)
		last_checkpoint = checkpoint_id

func complete_lap() -> void:
	"""Complete current lap"""
	lap_number += 1
	lap_times.append(current_lap_time)
	
	if best_lap_time == 0.0 or current_lap_time < best_lap_time:
		best_lap_time = current_lap_time
	
	lap_start_time = 0.0
	current_lap_time = 0.0
	
	emit_signal("lap_completed", {
		"lap_number": lap_number,
		"time": current_lap_time,
		"best_lap": best_lap_time
	})

func reset_lap_data() -> void:
	"""Reset all lap and race data"""
	lap_number = 0
	lap_times.clear()
	best_lap_time = 0.0
	current_lap_time = 0.0
	checkpoints_passed.clear()
	last_checkpoint = -1
	total_race_time = 0.0
	nitrous_current = nitrous_capacity

# ============================================================================
# DEBUG VISUALIZATION
# ============================================================================

func _init_debug_visualization() -> void:
	"""Initialize debug visualization lines"""
	if not debug_show_vehicle_lines:
		return
	
	_debug_lines.clear()
	
	var line = Line3D.new()
	line.width = 2
	add_child(line)
	_debug_lines.append(line)

func _update_debug_visualization(delta: float) -> void:
	"""Update debug visualization"""
	if not debug_show_vehicle_lines or _debug_lines.is_empty():
		return
	
	var line: Line3D = _debug_lines[0]
	line.points.clear()
	
	# Draw wheel positions
	for i in range(NUM_WHEELS):
		var pos: Vector3 = transform * wheel_positions[i]
		line.points.append(pos)
	
	# Draw center of mass
	line.points.append(transform * center_of_mass)

# ============================================================================
# PUBLIC API
# ============================================================================

func get_speed_kmh() -> float:
	"""Convert speed to kilometers per hour"""
	return current_speed * 3.6

func get_speed_mph() -> float:
	"""Convert speed to miles per hour"""
	return current_speed * 2.237

func get_current_rpm() -> float:
	return _rpm

func get_current_gear_name() -> String:
	match current_gear:
		0: return "N"
		-1: return "R"
		_: return str(current_gear)

func set_max_speed(new_max: float) -> void:
	max_forward_speed = new_max
	max_reverse_speed = new_max * 0.5

func set_vehicle_mass(new_mass: float) -> void:
	mass = new_mass

func enable_traction_control(level: int) -> void:
	traction_control_level = clampf(level, 0, 3)

func reset_to_default() -> void:
	"""Reset vehicle to default state"""
	current_speed = 0.0
	_rpm = idle_rpm
	current_gear = 0
	target_gear = 0
	is_shifting = false
	gear_shift_progress = 0.0
	is_drifting = false
	nitrous_current = nitrous_capacity
	reset_lap_data()
	velocity = Vector3.ZERO
	body_rotation = Vector3.ZERO

func save_state() -> Dictionary:
	"""Save current vehicle state for replay/loading"""
	return {
		"position": global_position,
		"rotation": global_rotation,
		"velocity": velocity,
		"current_gear": current_gear,
		"_rpm": _rpm,
		"current_speed": current_speed,
		"nitrous_current": nitrous_current,
		"lap_number": lap_number,
		"total_race_time": total_race_time
	}

func load_state(state: Dictionary) -> void:
	"""Load vehicle state from saved data"""
	global_position = state.position
	global_rotation = state.rotation
	velocity = state.velocity
	current_gear = state.current_gear
	_rpm = state._rpm
	current_speed = state.current_speed
	nitrous_current = state.nitrous_current
	lap_number = state.lap_number
	total_race_time = state.total_race_time

# ============================================================================
# UTILITIES
# ============================================================================

func get_weight_on_wheel(wheel_index: int) -> float:
	"""Calculate weight distribution on a specific wheel"""
	var base_weight: float = mass * 9.81
	
	if wheel_index == WHEEL_FL or wheel_index == WHEEL_FR:
		return base_weight * weight_distribution_front / 2.0
	else:
		return base_weight * weight_distribution_rear / 2.0

func calculate_cornering_force(wheel_index: int) -> float:
	"""Calculate cornering force for a wheel"""
	var normal_force: float = get_weight_on_wheel(wheel_index)
	var friction_coefficient: float = 1.2  # Typical tire friction
	return normal_force * friction_coefficient

func is_vehicle_stable() -> bool:
	"""Check if vehicle is in stable condition"""
	return not is_drifting and abs(slip_angle) < drift_threshold * 2

func get_vehicle_health() -> float:
	"""Get overall vehicle health (0.0 to 1.0)"""
	# This could be extended to track actual damage
	return 1.0

func apply_damage(damage_amount: float) -> void:
	"""Apply damage to vehicle"""
	# Implement damage tracking here
	pass

func reset_damage() -> void:
	"""Reset all damage"""
	pass