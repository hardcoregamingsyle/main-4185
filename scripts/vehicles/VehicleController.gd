extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulation
## Handles throttle, brake, steering, wheel forces, gear shifting, and vehicle dynamics
## Integrates with PhysicsSettings for centralized physics tuning
## Copyright 2026 Thalamus Racing Simulator Project

# ============================================================================
# SIGNALS - Vehicle state change notifications
# ============================================================================
signal speed_changed(new_speed: float)
signal rpm_changed(new_rpm: float)
signal gear_changed(new_gear: int)
signal drift_started()
signal drift_ended()
signal collision_impact(impact_force: Vector3, surface_normal: Vector3)
signal wheel_slip(wheel_index: int, slip_ratio: float)
signal engine_rev_limit_reached()
signal transmission_shifted(from_gear: int, to_gear: int)

# ============================================================================
# PHYSICS SETTINGS REFERENCE
# ============================================================================
var physics_config: PhysicsSettings = PhysicsSettings

# ============================================================================
# VEHICLE CONFIGURATION EXPORT GROUPS
# ============================================================================

@export_group("Vehicle Configuration")
@export var vehicle_mass: float = 1500.0: set = _set_vehicle_mass
@export var center_of_mass_offset: Vector3 = Vector3(0.0, 0.3, 0.0)
@export var wheel_base: float = 2.7: set = _set_wheel_base
@export var track_width: float = 1.6: set = _set_track_width
@export var ground_clearance: float = 0.15
@export var drag_coefficient: float = 0.30
@export var frontal_area: float = 2.2
@export var aero_downforce_factor: float = 0.05

@export_group("Suspension Geometry")
@export var suspension_travel_max: float = 0.15
@export var suspension_damping_rate: float = 8000.0
@export var suspension_compression_rate: float = 12000.0
@export var spring_stiffness: float = 45000.0
@export var anti_roll_bar_strength: float = 15000.0

@export_group("Wheel Configuration")
@export var wheel_radius: float = 0.31
@export var tire_friction_coefficient: float = 1.2
@export var tire_side_stiffness: float = 35000.0
@export var camber_angle_front: float = -0.03
@export var camber_angle_rear: float = -0.02
@export var toe_angle_front: float = 0.005
@export var toe_angle_rear: float = 0.003

@export_group("Powertrain Parameters")
@export var engine_max_rpm: float = 8500.0
@export var engine_idle_rpm: float = 800.0
@export var engine_peak_torque_rpm: float = 4500.0
@export var engine_peak_torque: float = 450.0  # Newton-meters
@export var engine_power_kw: float = 300.0
@export var clutch_disengagement_rpm: float = 2000.0
@export var rev_match_enabled: bool = true
@export var launch_control_enabled: bool = false

@export_group("Transmission Configuration")
@export var transmission_type: String = "manual"  # manual, automatic, sequential
@export var final_drive_ratio: float = 3.73
@export var neutral_position: int = 0
@export var first_gear_ratio: float = 3.65
@export var second_gear_ratio: float = 2.21
@export var third_gear_ratio: float = 1.58
@export var fourth_gear_ratio: float = 1.21
@export var fifth_gear_ratio: float = 0.98
@export var sixth_gear_ratio: float = 0.82
@export var reverse_ratio: float = 3.45
@export var upshift_rpm_threshold: float = 7500.0
@export var downshift_rpm_threshold: float = 2000.0
@export var auto_throttle_blip: bool = true

@export_group("Braking System")
@export var front_brake_bias: float = 0.58
@export var rear_brake_bias: float = 0.42
@export var max_brake_pressure: float = 15.0  # Bar
@export var brake_disc_radius: float = 0.32
@export var brake_pad_friction: float = 0.38
@export var abs_enabled: bool = true
@export var brake_balance_adjustment: float = 0.0

@export_group("Drivetrain Configuration")
@export var drivetrain_type: String = "rwd"  # rwd, fwd, awd
@export var torque_split_front: float = 0.0
@export var torque_split_rear: float = 1.0
@export var limited_slip_diff_ratio: float = 2.5
@export var open_diff_behavior: bool = true

@export_group("Steering System")
@export var steering_ratio: float = 14.0
@export var steering_lock_degrees: float = 45.0
@export var steering_response_rate: float = 20.0
@export var power_steering_enabled: bool = true
@export var steering_weight_curve: float = 0.5

# ============================================================================
# STATE VARIABLES
# ============================================================================

# Current vehicle state
var current_gear: int = 0
var target_gear: int = 0
var current_rpm: float = 0.0
var target_rpm: float = 0.0
var vehicle_speed: float = 0.0  # m/s forward velocity
var lateral_velocity: float = 0.0  # m/s sideways velocity
var angular_velocity_z: float = 0.0  # yaw rate rad/s
var acceleration: float = 0.0  # m/s^2 longitudinal
var braking_force: float = 0.0  # total braking force in Newtons

# Input values (0-1 range)
var throttle_input: float = 0.0
var brake_input: float = 0.0
var clutch_input: float = 0.0
var steering_input: float = 0.0
var handbrake_input: float = 0.0

# Engine state
var engine_state: String = "idle"  # idle, running, stalled, overrevved
var engine_temp: float = 90.0  # Celsius
var oil_pressure: float = 0.0  # Bar
var fuel_level: float = 100.0  # Percentage

# Drift state
var drifting: bool = false
var drift_angle: float = 0.0
var drift_intensity: float = 0.0
var drift_timer: float = 0.0
var drift_score: float = 0.0

# Wheel states (front_left, front_right, rear_left, rear_right)
var wheel_states: Array[Dictionary] = []
var wheel_contact_points: Array[Vector3] = []

# Aerodynamics
var air_density: float = 1.225  # kg/m^3 at sea level
var downforce: float = 0.0
var drag_force: float = 0.0
var lift: float = 0.0

# Physics accumulators
var _torque_to_wheels: float = 0.0
var _wheel_torque: Dictionary = {}
var _suspension_compression: Dictionary = {}
var _tire_forces: Dictionary = {}

# Timing
var _last_time_step: float = 0.0
var _drift_accumulator: float = 0.0

# References to child nodes
var _chassis_node: Node3D = null
var _collision_shape: CollisionShape3D = null
var _visual_mesh: MeshInstance3D = null

# ============================================================================
# INITIALIZATION
# ============================================================================

func _ready() -> void:
	_init_chassis_references()
	_init_wheel_states()
	_init_tire_forces()
	_init_suspension_compression()
	_initialize_engine()
	_set_initial_gear()
	_setup_collision_detection()
	
	# Ensure we're processing
	process_mode = ProcessModeEnum.ALWAYS
	
	# Log initialization
	print("[VehicleController] Initialized %s at position: %s" % [get_tree().current_scene.scene_file_path, position])

func _init_chassis_references() -> void:
	"""Initialize references to chassis components."""
	_chassis_node = get_node_or_null("../Chassis") if has_node("../Chassis") else self
	_collision_shape = get_node_or_null("CollisionShape3D")
	_visual_mesh = get_node_or_null("VisualMesh")

func _init_wheel_states() -> void:
	"""Initialize wheel state dictionaries for all four wheels."""
	wheel_states = [
		{"name": "front_left", "slip_ratio": 0.0, "slip_angle": 0.0, "load": 0.0, "torque": 0.0},
		{"name": "front_right", "slip_ratio": 0.0, "slip_angle": 0.0, "load": 0.0, "torque": 0.0},
		{"name": "rear_left", "slip_ratio": 0.0, "slip_angle": 0.0, "load": 0.0, "torque": 0.0},
		{"name": "rear_right", "slip_ratio": 0.0, "slip_angle": 0.0, "load": 0.0, "torque": 0.0}
	]
	
	_wheel_torque["front_left"] = 0.0
	_wheel_torque["front_right"] = 0.0
	_wheel_torque["rear_left"] = 0.0
	_wheel_torque["rear_right"] = 0.0

func _init_tire_forces() -> void:
	"""Initialize tire force tracking."""
	tire_forces = {
		"front_left": {"longitudinal": 0.0, "lateral": 0.0, "vertical": 0.0},
		"front_right": {"longitudinal": 0.0, "lateral": 0.0, "vertical": 0.0},
		"rear_left": {"longitudinal": 0.0, "lateral": 0.0, "vertical": 0.0},
		"rear_right": {"longitudinal": 0.0, "lateral": 0.0, "vertical": 0.0}
	}

func _init_suspension_compression() -> void:
	"""Initialize suspension compression tracking."""
	suspension_compression = {
		"front_left": 0.0,
		"front_right": 0.0,
		"rear_left": 0.0,
		"rear_right": 0.0
	}

func _initialize_engine() -> void:
	"""Initialize engine to idle state."""
	current_rpm = engine_idle_rpm
	target_rpm = engine_idle_rpm
	engine_state = "running"
	fuel_level = 100.0
	engine_temp = 90.0

func _set_initial_gear() -> void:
	"""Set initial gear based on transmission type."""
	if drivetrain_type == "fwd" or drivetrain_type == "awd":
		current_gear = 1  # Start in first gear
	else:
		current_gear = 0  # Neutral for RWD

func _setup_collision_detection() -> void:
	"""Setup collision detection signals."""
	body_entered.connect(_on_body_entered)
	body_exited.connect(_on_body_exited)

# ============================================================================
# MAIN GAME LOOP
# ============================================================================

func _physics_process(delta: float) -> void:
	"""Main physics update loop - runs at fixed timestep."""
	if delta <= 0.0:
		return
	
	# Store previous state for change detection
	var prev_rpm = current_rpm
	var prev_gear = current_gear
	
	# Calculate time step with physics scale
	var physics_delta = delta * physics_config.time_scale
	
	# Update engine state
	_update_engine_state(physics_delta)
	
	# Read inputs
	_read_inputs()
	
	# Calculate vehicle dynamics
	_calculate_driving_physics(physics_delta)
	
	# Handle gear shifting
	_handle_gear_shifting(physics_delta)
	
	# Apply wheel forces
	_apply_wheel_forces(physics_delta)
	
	# Calculate aerodynamics
	_calculate_aerodynamics()
	
	# Update vehicle movement
	_update_vehicle_movement(physics_delta)
	
	# Check collision impacts
	_check_collision_impacts()
	
	# Update drift state
	_update_drift_state(physics_delta)
	
	# Emit signals for state changes
	_emit_state_signals(prev_rpm, prev_gear)
	
	# Update last time step
	_last_time_step = Time.get_ticks_msec() / 1000.0

# ============================================================================
# INPUT HANDLING
# ============================================================================

func _read_inputs() -> void:
	"""Read and normalize all input sources."""
	# Get throttle input (accelerator pedal)
	throttle_input = clamp(InputManager.get_axis("throttle"), 0.0, 1.0)
	
	# Get brake input (brake pedal)
	brake_input = clamp(InputManager.get_axis("brake"), 0.0, 1.0)
	
	# Get clutch input (clutch pedal)
	clutch_input = clamp(InputManager.get_axis("clutch"), 0.0, 1.0)
	
	# Get steering input (left/right)
	steering_input = clamp(InputManager.get_axis("steering"), -1.0, 1.0)
	
	# Get handbrake input
	handbrake_input = clamp(InputManager.get_axis("handbrake"), 0.0, 1.0)
	
	# Gear shift inputs (handled separately via signals)
	if InputManager.is_action_just_pressed("gear_up"):
		_request_gear_shift(1)
	elif InputManager.is_action_just_pressed("gear_down"):
		_request_gear_shift(-1)

# ============================================================================
# ENGINE MANAGEMENT
# ============================================================================

func _update_engine_state(delta: float) -> void:
	"""Update engine RPM and thermal state."""
	var torque_output = _calculate_engine_torque()
	
	# Calculate engine acceleration based on torque
	var moment_of_inertia = 0.5 * vehicle_mass * pow(wheel_radius, 2)  # Simplified
	var engine_acceleration = torque_output / moment_of_inertia
	
	# Apply engine friction losses
	var friction_loss = current_rpm * 0.01
	var net_acceleration = engine_acceleration - friction_loss
	
	# Update RPM
	current_rpm += net_acceleration * delta
	
	# Clamp RPM to valid range
	current_rpm = clamp(current_rpm, engine_idle_rpm, engine_max_rpm * 1.2)
	
	# Check for over-revving
	if current_rpm > engine_max_rpm * 0.95:
		_limit_engine_rpm(delta)
	
	# Update engine temperature (simple model)
	var heat_generation = torque_output * 0.001
	var cooling_rate = 0.5 + (vehicle_speed * 0.1)
	engine_temp += (heat_generation - cooling_rate) * delta
	engine_temp = clamp(engine_temp, 60.0, 120.0)
	
	# Oil pressure depends on RPM
	oil_pressure = 0.05 * current_rpm + 0.5
	oil_pressure = clamp(oil_pressure, 0.0, 6.0)
	
	# Fuel consumption
	if throttle_input > 0.1 and current_rpm > 1000:
		var fuel_consumption = throttle_input * 0.0001 * (current_rpm / engine_peak_torque_rpm)
		fuel_level -= fuel_consumption
		fuel_level = clamp(fuel_level, 0.0, 100.0)

func _calculate_engine_torque() -> float:
	"""Calculate engine torque output based on RPM and throttle."""
	var rpm_normalized = current_rpm / engine_peak_torque_rpm
	
	# Torque curve (Gaussian-like around peak torque RPM)
	var torque_curve = exp(-pow(rpm_normalized - 1.0, 2) * 2.0)
	
	# Throttle response modifier
	var throttle_modifier = lerp(0.3, 1.0, pow(throttle_input, 0.5))
	
	# Clutch engagement factor
	var clutch_factor = 1.0 - min(clutch_input, 0.5)
	
	var torque = engine_peak_torque * torque_curve * throttle_modifier * clutch_factor
	
	return torque

func _limit_engine_rpm(delta: float) -> void:
	"""Prevent engine from exceeding safe RPM limits."""
	if current_rpm > engine_max_rpm:
		current_rpm = engine_max_rpm
		engine_state = "overrevved"
		
		# Trigger redline signal
		emit_signal("engine_rev_limit_reached")
		
		# Apply rev limiter (momentary cut)
		if randf() < 0.1:  # Simulate cut every ~10ms
			current_rpm -= 500.0 * delta

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================

func _request_gear_shift(direction: int) -> void:
	"""Request a gear shift in given direction."""
	var new_gear = current_gear + direction
	
	# Validate gear range
	match drivetrain_type:
		"rwd", "fwd":
			new_gear = clamp(new_gear, 0, 6)  # 0=neutral, 1-6=gears
		"awd":
			new_gear = clamp(new_gear, 0, 6)
	
	# Prevent illegal shifts
	if new_gear != current_gear:
		_execute_gear_shift(current_gear, new_gear)

func _execute_gear_shift(from_gear: int, to_gear: int) -> void:
	"""Execute gear shift with clutch management."""
	# Check clutch engagement for manual
	if transmission_type == "manual" and clutch_input < 0.3:
		print("[VehicleController] Cannot shift without clutch engaged!")
		return
	
	# Perform shift
	target_gear = to_gear
	
	# Auto-throttle blip on downshift
	if to_gear < from_gear and auto_throttle_blip:
		throttle_input = min(0.3, 0.1 + (from_gear - to_gear) * 0.1)
	
	# Update actual gear after shift delay
	await get_tree().create_timer(0.2).timeout
	current_gear = to_gear
	
	# Emit signals
	emit_signal("transmission_shifted", from_gear, to_gear)
	emit_signal("gear_changed", current_gear)
	
	# Audio feedback
	AudioManager.play_sound("gear_shift")

func _handle_gear_shifting(delta: float) -> void:
	"""Handle automatic gear shifting logic."""
	if transmission_type == "automatic":
		_auto_shift_gears(delta)
	elif transmission_type == "sequential":
		_sequential_shift_logic(delta)

func _auto_shift_gears(delta: float) -> void:
	"""Automatic transmission shift logic."""
	var should_upshift = current_rpm >= upshift_rpm_threshold and throttle_input > 0.1
	var should_downshift = current_rpm <= downshift_rpm_threshold and throttle_input < 0.3
	
	if should_upshift and current_gear < 6:
		_request_gear_shift(1)
	elif should_downshift and current_gear > 1:
		_request_gear_shift(-1)

func _sequential_shift_logic(delta: float) -> void:
	"""Sequential gearbox behavior (no skipping gears)."""
	pass  # Handled by explicit gear_up/gear_down inputs

# ============================================================================
# DRIVING PHYSICS CALCULATION
# ============================================================================

func _calculate_driving_physics(delta: float) -> void:
	"""Calculate longitudinal and lateral driving physics."""
	# Calculate drive torque to wheels
	_torque_to_wheels = _calculate_wheel_torque()
	
	# Apply torque to driven wheels
	_apply_torque_to_driven_wheels()
	
	# Calculate acceleration
	acceleration = _calculate_longitudinal_acceleration()
	
	# Update vehicle speed
	var current_speed_vector = velocity.length()
	var speed_change = acceleration * delta
	
	# Apply drag and rolling resistance
	var drag_resistance = _calculate_drag_force()
	var rolling_resistance = _calculate_rolling_resistance()
	
	# Net force calculation
	var total_resistance = drag_resistance + rolling_resistance
	var net_force = (_torque_to_wheels / wheel_radius) - total_resistance
	
	# Acceleration from net force (F = ma)
	var calculated_acceleration = net_force / vehicle_mass
	
	# Update speed
	vehicle_speed = clamp(vehicle_speed + calculated_acceleration * delta, 0.0, 300.0)  # Cap at 300 km/h equivalent
	
	# Lateral forces affect sliding
	lateral_velocity += angular_velocity_z * wheel_base * delta
	
	# Air density varies with altitude (simple model)
	air_density = 1.225 * exp(-position.y / 8000.0)

func _calculate_wheel_torque() -> float:
	"""Calculate torque delivered to wheels based on gear ratio."""
	if current_gear == 0:
		return 0.0
	
	var gear_ratio = _get_current_gear_ratio()
	var total_ratio = gear_ratio * final_drive_ratio
	var efficiency = 0.85  # Transmission efficiency
	
	var wheel_torque = _torque_to_wheels * total_ratio * efficiency
	
	return wheel_torque

func _get_current_gear_ratio() -> float:
	"""Get the gear ratio for current gear."""
	match current_gear:
		1: return first_gear_ratio
		2: return second_gear_ratio
		3: return third_gear_ratio
		4: return fourth_gear_ratio
		5: return fifth_gear_ratio
		6: return sixth_gear_ratio
		0: return 0.0
		_: return 0.0

func _apply_torque_to_driven_wheels() -> void:
	"""Apply calculated torque to driven wheels."""
	var torque_per_wheel = _torque_to_wheels / 2.0  # Split between two driven wheels
	
	match drivetrain_type:
		"rwd":
			_wheel_torque["rear_left"] = torque_per_wheel
			_wheel_torque["rear_right"] = torque_per_wheel
		"fwd":
			_wheel_torque["front_left"] = torque_per_wheel
			_wheel_torque["front_right"] = torque_per_wheel
		"awd":
			var front_torque = torque_per_wheel * torque_split_front
			var rear_torque = torque_per_wheel * torque_split_rear
			
			_wheel_torque["front_left"] = front_torque
			_wheel_torque["front_right"] = front_torque
			_wheel_torque["rear_left"] = rear_torque
			_wheel_torque["rear_right"] = rear_torque

func _calculate_longitudinal_acceleration() -> float:
	"""Calculate net longitudinal acceleration."""
	var drive_force = _torque_to_wheels / wheel_radius
	
	# Subtract resistive forces
	var drag = _calculate_drag_force()
	var rolling = _calculate_rolling_resistance()
	var gradient = _calculate_gradient_force()
	
	var net_force = drive_force - drag - rolling - gradient
	
	return net_force / vehicle_mass

func _calculate_drag_force() -> float:
	"""Calculate aerodynamic drag force."""
	var air_density_local = air_density
	var v_squared = pow(vehicle_speed, 2)
	
	drag_force = 0.5 * air_density_local * frontal_area * drag_coefficient * v_squared
	
	return drag_force

func _calculate_rolling_resistance() -> float:
	"""Calculate rolling resistance force."""
	var coefficient = 0.015  # Typical tire rolling resistance
	var normal_force = vehicle_mass * physics_config.gravity
	
	return coefficient * normal_force

func _calculate_gradient_force() -> float:
	"""Calculate force due to road gradient."""
	var gravity_component = vehicle_mass * physics_config.gravity * sin(get_rotation().x)
	
	return gravity_component

# ============================================================================
# AERODYNAMICS
# ============================================================================

func _calculate_aerodynamics() -> void:
	"""Calculate aerodynamic forces on vehicle."""
	var v_squared = pow(vehicle_speed, 2)
	
	# Downforce increases with speed squared
	downforce = 0.5 * air_density * frontal_area * aero_downforce_factor * v_squared
	
	# Drag already calculated in _calculate_drag_force()
	
	# Lift (usually negative/downward for race cars)
	lift = -downforce * 0.3  # Some vehicles experience lift
	
	# Update effective weight for tire grip
	var effective_weight = vehicle_mass * physics_config.gravity + downforce - lift

# ============================================================================
# BRAKING SYSTEM
# ============================================================================

func _calculate_braking_force() -> float:
	"""Calculate total braking force applied."""
	if brake_input <= 0.0 and handbrake_input <= 0.0:
		return 0.0
	
	var brake_pressure = brake_input * max_brake_pressure
	var handbrake_pressure = handbrake_input * max_brake_pressure
	
	# Total pressure
	var total_pressure = brake_pressure + handbrake_pressure
	
	# Brake force calculation (F = mu * N)
	var normal_force = vehicle_mass * physics_config.gravity
	var brake_force_coefficient = brake_pad_friction
	
	# Front/rear bias adjustment
	var front_force = total_pressure * front_brake_bias * brake_force_coefficient * normal_force
	var rear_force = total_pressure * rear_brake_bias * brake_force_coefficient * normal_force
	
	# ABS modulation (simplified)
	if abs_enabled:
		var wheel_load = normal_force / 4.0
		for wheel in ["front_left", "front_right", "rear_left", "rear_right"]:
			var slip = wheel_states.find(wheel).slip_ratio if wheel_states.size() > 0 else 0.0
			if abs(slip) > 0.2:
				total_pressure *= 0.8  # Reduce pressure if slipping
	
	return total_pressure * brake_force_coefficient * normal_force

# ============================================================================
# WHEEL FORCE APPLICATION
# ============================================================================

func _apply_wheel_forces(delta: float) -> void:
	"""Apply all calculated forces to individual wheels."""
	# Calculate vertical load on each wheel (simplified weight distribution)
	var weight_distribution = _calculate_weight_distribution()
	
	# Apply forces to each wheel
	for i, wheel_name in ["front_left", "front_right", "rear_left", "rear_right"]:
		_apply_single_wheel_forces(i, wheel_name, weight_distribution)

func _calculate_weight_distribution() -> Dictionary:
	"""Calculate weight distribution across wheels."""
	var total_weight = vehicle_mass * physics_config.gravity
	
	# Simple 40/60 front/rear split (adjustable)
	var front_weight = total_weight * 0.4
	var rear_weight = total_weight * 0.6
	
	return {
		"front_left": front_weight * 0.5,
		"front_right": front_weight * 0.5,
		"rear_left": rear_weight * 0.5,
		"rear_right": rear_weight * 0.5
	}

func _apply_single_wheel_forces(wheel_index: int, wheel_name: String, weight_dist: Dictionary) -> void:
	"""Apply forces to a single wheel."""
	var wheel_data = wheel_states[wheel_index]
	var vertical_load = weight_dist[wheel_name]
	
	# Longitudinal force from drive/brake
	var drive_force = _wheel_torque.get(wheel_name, 0.0) / wheel_radius
	
	# Braking force (if braking)
	var brake_force = 0.0
	if brake_input > 0.0:
		var bias = front_brake_bias if wheel_name.contains("front") else rear_brake_bias
		brake_force = brake_input * max_brake_pressure * brake_pad_friction * vertical_load * bias
	
	# Net longitudinal force
	var longitudinal_force = drive_force - brake_force
	
	# Lateral force (cornering)
	var slip_angle = wheel_data.slip_angle
	var lateral_force = -tire_side_stiffness * slip_angle
	
	# Clamp forces to friction circle
	var max_force = vertical_load * tire_friction_coefficient
	var resultant = sqrt(pow(longitudinal_force, 2) + pow(lateral_force, 2))
	
	if resultant > max_force:
		var scale = max_force / resultant
		longitudinal_force *= scale
		lateral_force *= scale
	
	# Update wheel forces dictionary
	tire_forces[wheel_name]["longitudinal"] = longitudinal_force
	tire_forces[wheel_name]["lateral"] = lateral_force
	tire_forces[wheel_name]["vertical"] = vertical_load
	
	# Emit wheel slip signal if significant
	var slip_ratio = abs(wheel_data.slip_ratio)
	if slip_ratio > 0.1:
		emit_signal("wheel_slip", wheel_index, slip_ratio)

# ============================================================================
# VEHICLE MOVEMENT UPDATE
# ============================================================================

func _update_vehicle_movement(delta: float) -> void:
	"""Update vehicle position and rotation based on calculated forces."""
	# Convert velocity to local coordinates
	var forward_direction = global_transform.basis.z.rotated(Vector3.UP, PI)
	var right_direction = global_transform.basis.x.rotated(Vector3.UP, PI)
	
	# Apply longitudinal movement
	var move_distance = vehicle_speed * delta
	global_position += forward_direction * move_distance
	
	# Apply steering
	var steering_angle = steering_input * deg_to_rad(steering_lock_degrees)
	var turn_radius = wheel_base / tan(steering_angle + 0.001)  # Avoid division by zero
	
	if abs(steering_input) > 0.01 and vehicle_speed > 1.0:
		angular_velocity_z = vehicle_speed / turn_radius
		rotation.y += angular_velocity_z * delta
	
	# Apply lateral forces (sliding)
	if abs(lateral_velocity) > 0.1:
		global_position += right_direction * lateral_velocity * delta
	
	# Ground friction
	var ground_friction = 0.98  # Air resistance
	velocity *= ground_friction
	
	# Sync CharacterBody3D velocity
	velocity = global_transform.basis.x * lateral_velocity + \
	           global_transform.basis.z * vehicle_speed
	
	move_and_slide()

# ============================================================================
# DRIFT MECHANICS
# ============================================================================

func _update_drift_state(delta: float) -> void:
	"""Update drift detection and scoring."""
	# Calculate drift angle (difference between heading and velocity direction)
	var heading_angle = atan2(global_transform.basis.z.y, global_transform.basis.z.x)
	var velocity_angle = atan2(velocity.y, velocity.x)
	
	# Drift angle in radians
	var angle_diff = heading_angle - velocity_angle
	angle_diff = wrap(angle_diff, -PI, PI)
	
	# Detect drift conditions
	var is_drifting = abs(angle_diff) > deg_to_rad(15) and abs(lateral_velocity) > 5.0
	
	if is_drifting and not drifting:
		# Just started drifting
		drifting = true
		emit_signal("drift_started")
		AudioManager.play_sound("drift_start")
	
	if not is_drifting and drifting:
		# Drift ended
		drifting = false
		emit_signal("drift_ended")
		AudioManager.play_sound("drift_end")
	
	# Accumulate drift score while drifting
	if drifting:
		var intensity = abs(angle_diff) / deg_to_rad(45.0)  # Normalize to max 45 degrees
		intensity = clamp(intensity, 0.0, 1.0)
		
		# Score increases with intensity and duration
		var score_gain = intensity * intensity * 10.0 * delta
	漂移积分
	漂移积分 = clamp(漂移积分，0.0，100.0)
	
	# Decay score when not drifting
	if not drifting:
		漂移积分 -= 2.0 * delta
		漂移积分 = max(漂移积分，0.0)

# ============================================================================
# COLLISION HANDLING
# ============================================================================

func _on_body_entered(body: Node3D) -> void:
	"""Handle collision entry."""
	var relative_velocity = global_velocity - body.global_velocity if body.has_method("global_velocity") else global_velocity
	var impact_force = relative_velocity.length() * vehicle_mass
	
	if impact_force > 5000:  # Significant impact threshold
		emit_signal("collision_impact", relative_velocity, global_transform.basis.z)
		
		# Screen shake effect
		GameManager.trigger_screen_shake(min(impact_force / 10000.0, 0.5))

func _on_body_exited(body: Node3D) -> void:
	"""Handle collision exit."""
	pass

func _check_collision_impacts() -> void:
	"""Check for significant collision impacts."""
	# This would be enhanced with raycast-based impact detection
	pass

# ============================================================================
# STATE SIGNALS
# ============================================================================

func _emit_state_signals(prev_rpm: float, prev_gear: int) -> void:
	"""Emit signals when state changes are detected."""
	# Speed changed
	if abs(vehicle_speed - _last_speed) > 0.1:
		emit_signal("speed_changed", vehicle_speed)
		_last_speed = vehicle_speed
	
	# RPM changed significantly
	if abs(current_rpm - prev_rpm) > 100:
		emit_signal("rpm_changed", current_rpm)
	
	# Gear changed
	if current_gear != prev_gear:
		emit_signal("gear_changed", current_gear)

# ============================================================================
# HELPER METHODS
# ============================================================================

func get_speed_kmh() -> float:
	"""Convert internal speed to kilometers per hour."""
	return vehicle_speed * 3.6

func get_speed_mph() -> float:
	"""Convert internal speed to miles per hour."""
	return vehicle_speed * 2.23694

func get_rpm_percentage() -> float:
	"""Get current RPM as percentage of maximum."""
	return current_rpm / engine_max_rpm

func get_fuel_range_km() -> float:
	"""Estimate remaining distance in kilometers."""
	var consumption_per_100km = 8.0 + (vehicle_speed * 0.5)  # Liters per 100km
	var fuel_remaining = fuel_level / 100.0
	var tank_capacity = 60.0  # Liters
	
	return (fuel_remaining * tank_capacity / consumption_per_100km) * 100.0

func reset_vehicle() -> void:
	"""Reset vehicle to initial state."""
	current_rpm = engine_idle_rpm
	current_gear = 0
	vehicle_speed = 0.0
	lateral_velocity = 0.0
	throttle_input = 0.0
	brake_input = 0.0
	clutch_input = 0.0
	steering_input = 0.0
	handbrake_input = 0.0
	fuel_level = 100.0
	engine_temp = 90.0
	drifting = false
	漂移积分 = 0.0

func set_position_safe(pos: Vector3) -> void:
	"""Set position with collision check."""
	position = pos
	velocity = Vector3.ZERO

func _set_vehicle_mass(value: float) -> void:
	vehicle_mass = value
	emit_signal("vehicle_parameters_changed", "mass", value)

func _set_wheel_base(value: float) -> void:
	wheel_base = value
	emit_signal("vehicle_parameters_changed", "wheel_base", value)

func _set_track_width(value: float) -> void:
	track_width = value
	emit_signal("vehicle_parameters_changed", "track_width", value)

# ============================================================================
# DEBUG & VISUALIZATION
# ============================================================================

func _draw_debug_lines() -> void:
	"""Draw debug visualization lines (only in debug mode)."""
	if not GameManager.debug_mode:
		return
	
	# Draw velocity vector
	RenderingServer.viewport_add_debug_line(
		get_viewport().get_viewport().render_target_id,
		position,
		position + velocity.normalized() * 5.0,
		Color.GREEN
	)
	
	# Draw steering indicator
	var steering_color = Color.YELLOW if abs(steering_input) > 0.3 else Color.GRAY
	RenderingServer.viewport_add_debug_line(
		get_viewport().get_viewport().render_target_id,
		position,
		position + Vector3.FORWARD.rotated(Vector3.UP, steering_input * PI),
		steering_color
	)

func _process(_delta: float) -> void:
	"""Process loop for non-physics updates."""
	_draw_debug_lines()

# ============================================================================
# SERIALIZE/DESERIALIZE FOR REPLAY
# ============================================================================

func serialize_state() -> Dictionary:
	"""Serialize current vehicle state for replay/demos."""
	return {
		"position": global_position,
		"rotation": rotation,
		"velocity": velocity,
		"speed": vehicle_speed,
		"rpm": current_rpm,
		"gear": current_gear,
		"throttle": throttle_input,
		"brake": brake_input,
		"clutch": clutch_input,
		"steering": steering_input,
		"handbrake": handbrake_input,
		"fuel": fuel_level,
		"timestamp": Time.get_ticks_msec()
	}

func deserialize_state(state_data: Dictionary) -> void:
	"""Restore vehicle state from serialized data."""
	global_position = state_data.position
	rotation = state_data.rotation
	velocity = state_data.velocity
	vehicle_speed = state_data.speed
	current_rpm = state_data.rpm
	current_gear = state_data.gear
	throttle_input = state_data.throttle
	brake_input = state_data.brake
	clutch_input = state_data.clutch
	steering_input = state_data.steering
	handbrake_input = state_data.handbrake
	fuel_level = state_data.fuel

# ============================================================================
# END OF FILE
# ============================================================================