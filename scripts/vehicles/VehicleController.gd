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

# ============================================================================
# PHYSICS SETTINGS REFERENCE
# ============================================================================

@onready var _physics = PhysicsSettings.get_singleton()

# ============================================================================
# VEHICLE CONFIGURATION
# ============================================================================

@export_group("Vehicle Configuration")
@export var vehicle_mass: float = 1500.0: set = _set_vehicle_mass
@export var center_of_mass_offset: Vector3 = Vector3(0.0, 0.5, 0.0): set = _set_center_of_mass
@export var wheel_base: float = 2.8: set = _set_wheel_base
@export var track_width: float = 1.8: set = _set_track_width
@export var ground_clearance: float = 0.25: set = _set_ground_clearance
@export var drag_coefficient: float = 0.30: set = _set_drag_coefficient
@export var frontal_area: float = 2.2: set = _set_frontal_area
@export var roll_stiffness_front: float = 12000.0: set = _set_roll_stiffness_front
@export var roll_stiffness_rear: float = 10000.0: set = _set_roll_stiffness_rear
@export var camber_angle_front: float = -0.5: set = _set_camber_front
@export var camber_angle_rear: float = -0.5: set = _set_camber_rear
@export var toe_angle_front: float = 0.02: set = _set_toe_front
@export var toe_angle_rear: float = 0.02: set = _set_toe_rear

@export_group("Powertrain Parameters")
@export var engine_max_rpm: float = 7500.0: set = _set_engine_max_rpm
@export var engine_min_rpm: float = 800.0: set = _set_engine_min_rpm
@export var idle_rpm: float = 900.0: set = _set_idle_rpm
@export var max_torque: float = 450.0: set = _set_max_torque
@export var torque_curve_points: Array[Vector2] = []: set = _set_torque_curve
@export var gear_ratios: Array[float] = [3.8, 2.2, 1.5, 1.1, 0.9, 0.75, 0.6]: set = _set_gear_ratios
@export var final_drive_ratio: float = 3.5: set = _set_final_drive_ratio
@export var transmission_efficiency: float = 0.95: set = _set_transmission_efficiency
@export var clutch_engagement_rpm: float = 1200.0: set = _set_clutch_engagement
@export var rev_matching_enabled: bool = true

@export_group("Braking System")
@export var brake_pressure_front: float = 4.0: set = _set_brake_pressure_front
@export var brake_pressure_rear: float = 3.5: set = _set_brake_pressure_rear
@export var brake_disc_diameter: float = 0.32: set = _set_brake_disc_diameter
@export var brake_pad_friction: float = 0.35: set = _set_brake_pad_friction
@export var abs_enabled: bool = true
@export var brake_bias: float = 0.55

@export_group("Tire & Suspension")
@export var tire_radius: float = 0.33: set = _set_tire_radius
@export var tire_friction_coef: float = 1.2
@export var tire_side_wall_compliance: float = 0.02
@export var suspension_travel: float = 0.15
@export var spring_rate_front: float = 45000.0
@export var spring_rate_rear: float = 40000.0
@export var damping_rate_compress: float = 2500.0
@export var damping_rate_rebound: float = 1500.0
@export var anti_roll_bar_stiffness_front: float = 8000.0
@export var anti_roll_bar_stiffness_rear: float = 6000.0

@export_group("Drift & Traction Control")
@export var traction_control_enabled: bool = true
@export var drift_threshold: float = 0.25
@export var drift_recovery_factor: float = 0.15
@export var oversteer_correction: float = 0.1
@export var understeer_correction: float = 0.08

@export_group("Wheel Positions (Local Space)")
@export var front_left_wheel_pos: Vector3 = Vector3(-0.9, -0.33, 1.4)
@export var front_right_wheel_pos: Vector3 = Vector3(0.9, -0.33, 1.4)
@export var rear_left_wheel_pos: Vector3 = Vector3(-0.9, -0.33, -1.4)
@export var rear_right_wheel_pos: Vector3 = Vector3(0.9, -0.33, -1.4)

# ============================================================================
# INTERNAL STATE
# ============================================================================

# Physical state
var _current_speed: float = 0.0
var _angular_velocity: Vector3 = Vector3.ZERO
var _engine_rpm: float = 0.0
var _clutch_position: float = 0.0
var _transmission_state: int = 0  # 0=neutral, 1+=gears

# Input state (normalized -1 to 1)
var _throttle_input: float = 0.0
var _brake_input: float = 0.0
var _steering_input: float = 0.0
var _handbrake_input: float = 0.0
var _gear_shift_up: bool = false
var _gear_shift_down: bool = false
var _clutch_pedal: float = 0.0

# Wheel state
var _wheel_rotations: Array[float] = [0.0, 0.0, 0.0, 0.0]
var _wheel_slip_ratios: Array[float] = [0.0, 0.0, 0.0, 0.0]
var _wheel_forces: Array[float] = [0.0, 0.0, 0.0, 0.0]
var _suspension_compressions: Array[float] = [0.0, 0.0, 0.0, 0.0]
var _wheel_contact_normal: Array[Vector3] = [Vector3.UP, Vector3.UP, Vector3.UP, Vector3.UP]

# Drift state
var _drifting: bool = false
var _drift_angle: float = 0.0
var _drift_score: float = 0.0

# Current gear management
var _current_gear: int = 1
var _max_gear: int = 6
var _neutral_gear: int = 0
var _reverse_gear: int = -1

# Physics constants derived from config
var _wheel_radius: float = 0.0
var _tire_circumference: float = 0.0
var _effective_gear_ratio: float = 1.0
var _drive_wheels_mask: int = 6  # Rear-wheel drive default (bits 1 and 2)

# ============================================================================
# WHEEL INDICES
# ============================================================================
const WHEEL_FRONT_LEFT: int = 0
const WHEEL_FRONT_RIGHT: int = 1
const WHEEL_REAR_LEFT: int = 2
const WHEEL_REAR_RIGHT: int = 3

# ============================================================================
# INITIALIZATION
# ============================================================================

func _ready() -> void:
	_init_default_torque_curve()
	_calculate_derived_values()
	_setup_collision_layers()
	
	# Initialize wheel positions
	for i in range(4):
		_wheel_rotations[i] = 0.0
		_wheel_slip_ratios[i] = 0.0
		_wheel_forces[i] = 0.0
		_suspension_compressions[i] = 0.0
	
	set_physics_material_override(_get_tire_material())

func _init_default_torque_curve() -> void:
	"""Initialize default torque curve if empty"""
	if torque_curve_points.is_empty():
		torque_curve_points = [
			Vector2(0.1, 0.0),       # Idle
			Vector2(0.2, 0.3),       # Low RPM
			Vector2(0.35, 0.6),      # Mid RPM
			Vector2(0.5, 0.85),      # Peak Torque Region
			Vector2(0.65, 0.95),     # High RPM
			Vector2(0.8, 1.0),       # Redline Start
			Vector2(1.0, 0.9)        # Power Drop-off
		]

func _calculate_derived_values() -> void:
	"""Calculate derived values from configuration"""
	_wheel_radius = tire_radius
	_tire_circumference = PI * 2.0 * tire_radius
	_current_gear = neutral_gear + 1
	_max_gear = gear_ratios.size() - 1 if gear_ratios.size() > 0 else 5
	_neutral_gear = 0
	_reverse_gear = -1

func _setup_collision_layers() -> void:
	"""Setup collision layers for proper interaction"""
	collision_layer = 1 << 2  # Vehicle layer
	collision_mask = (1 << 0) | (1 << 1) | (1 << 3)  # Ground, obstacles, other vehicles

# ============================================================================
# INPUT HANDLING
# ============================================================================

func _input(event: InputEvent) -> void:
	if event.is_action_pressed("ui_cancel"):
		pause_mode = Node.PAUSE_MODE_PAUSED
	elif event.is_action_released("ui_cancel"):
		pause_mode = Node.PAUSE_MODE_INHERIT

func _process(delta: float) -> void:
	_read_inputs(delta)
	_update_driving_state(delta)

func _read_inputs(delta: float) -> void:
	"""Read normalized inputs from InputManager"""
	_throttle_input = clamp(Input.get_axis("brake", "gas"), -1.0, 1.0)
	_brake_input = clamp(Input.get_axis("brake", "gas"), -1.0, 1.0)
	_steering_input = clamp(Input.get_axis("steer_left", "steer_right"), -1.0, 1.0)
	_handbrake_input = Input.get_action_strength("handbrake")
	_clutch_pedal = Input.get_action_strength("clutch")
	
	# Gear shift detection
	if Input.is_action_just_pressed("shift_up"):
		_gear_shift_up = true
	if Input.is_action_just_pressed("shift_down"):
		_gear_shift_down = true
	if Input.is_action_just_released("shift_up"):
		_gear_shift_up = false
	if Input.is_action_just_released("shift_down"):
		_gear_shift_down = false

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================

func _update_driving_state(delta: float) -> void:
	"""Update driving state including gear shifting"""
	_handle_gear_shifting()
	_update_engine_rpm(delta)
	_update_clutch(delta)

func _handle_gear_shifting() -> void:
	"""Handle automatic or manual gear shifting"""
	if _clutch_pedal > 0.5:
		# Clutch disengaged - can shift freely
		_attempt_manual_shift()
	else:
		# Automatic upshifting based on RPM
		if _current_gear < _max_gear and _engine_rpm >= engine_max_rpm * 0.95:
			_shift_up()
		
		# Automatic downshifting based on RPM
		if _current_gear > 1 and _engine_rpm <= engine_min_rpm * 1.1:
			_shift_down()

func _attempt_manual_shift() -> void:
	"""Handle manual gear shifts when clutch is engaged"""
	if _gear_shift_up and _current_gear < _max_gear:
		_shift_up()
	elif _gear_shift_down and _current_gear > 1:
		_shift_down()

func _shift_up() -> void:
	"""Shift to next higher gear"""
	if _current_gear < _max_gear:
		var prev_gear = _current_gear
		_current_gear += 1
		
		if rev_matching_enabled:
			_rev_match_target_rpm()
		
		_clutch_position = 0.0
		gear_changed.emit(_current_gear)

func _shift_down() -> void:
	"""Shift to next lower gear"""
	if _current_gear > 1:
		var prev_gear = _current_gear
		_current_gear -= 1
		
		if rev_matching_enabled:
			_rev_match_target_rpm()
		
		_clutch_position = 0.0
		gear_changed.emit(_current_gear)

func _rev_match_target_rpm() -> void:
	"""Calculate target RPM for rev matching"""
	var current_speed_mps = _current_speed / 3.6
	var wheel_rpm = (current_speed_mps * 60.0) / _tire_circumference
	var target_rpm = wheel_rpm * _effective_gear_ratio * final_drive_ratio
	target_rpm = clamp(target_rpm, engine_min_rpm, engine_max_rpm)

func _update_engine_rpm(delta: float) -> void:
	"""Update engine RPM based on wheel speed and gear"""
	if _current_gear == neutral_gear:
		# Neutral - engine idles
		_engine_rpm = lerp(_engine_rpm, idle_rpm, delta * 10.0)
		return
	
	if _clutch_position < 0.1:
		# Engine connected to wheels
		var wheel_speed_mps = _current_speed
		var wheel_rpm = (wheel_speed_mps * 60.0) / _tire_circumference
		var target_rpm = wheel_rpm * _effective_gear_ratio * final_drive_ratio
		
		# Apply engine friction and inertia
		var rpm_diff = target_rpm - _engine_rpm
		_engine_rpm += rpm_diff * delta * 5.0
	else:
		# Clutch disengaged - engine decays to idle
		_engine_rpm = lerp(_engine_rpm, idle_rpm, delta * 3.0)
	
	_engine_rpm = clamp(_engine_rpm, engine_min_rpm, engine_max_rpm * 1.1)
	rpm_changed.emit(_engine_rpm)

func _update_clutch(delta: float) -> void:
	"""Update clutch engagement state"""
	var target_clutch = 1.0 - _clutch_pedal
	_clutch_position = lerp(_clutch_position, target_clutch, delta * 10.0)

# ============================================================================
# PHYSICS UPDATE (MAIN SIMULATION)
# ============================================================================

func _physics_process(delta: float) -> void:
	"""Main physics simulation loop"""
	if pause_mode == Node.PAUSE_MODE_PAUSED:
		return
	
	_apply_aerodynamics(delta)
	_calculate_wheel_forces(delta)
	_apply_drive_forces(delta)
	_apply_brake_forces(delta)
	_apply_suspension_forces(delta)
	_handle_drift(delta)
	_update_body_velocity(delta)
	_update_transform()

func _apply_aerodynamics(delta: float) -> void:
	"""Apply aerodynamic drag and downforce"""
	var velocity_mps = global_velocity.length()
	var air_density = 1.225  # kg/m^3 at sea level
	
	# Drag force: F = 0.5 * rho * v^2 * Cd * A
	var drag_force = 0.5 * air_density * pow(velocity_mps, 2) * drag_coefficient * frontal_area
	drag_force *= -sign(global_velocity.length())  # Opposes motion
	
	# Apply drag as opposite to velocity direction
	if global_velocity.length() > 0.1:
		var drag_direction = -global_velocity.normalized()
		add_force(drag_direction * drag_force * delta)

func _calculate_wheel_forces(delta: float) -> void:
	"""Calculate individual wheel forces based on contact physics"""
	var wheel_positions = [front_left_wheel_pos, front_right_wheel_pos, 
		rear_left_wheel_pos, rear_right_wheel_pos]
	
	for i in range(4):
		var world_pos = global_transform * wheel_positions[i]
		var ray_from = world_pos + Vector3.UP * (ground_clearance * 2.0)
		var ray_to = world_pos - Vector3.UP * (ground_clearance * 2.0 + suspension_travel * 2.0)
		
		var space_state = get_world_3d().direct_space_state
		var query = PhysicsRayQuery3D.create(ray_from, ray_to)
		query.collide_with_areas = false
		query.exclude = [self]
		
		var result = space_state.intersect_ray(query)
		
		if result && result.has("position"):
			var contact_normal = result.normal
			var compression = suspension_travel * (1.0 - result.position.y / ray_from.y)
			compression = clamp(compression, 0.0, suspension_travel)
			
			_wheel_contact_normal[i] = contact_normal
			_suspension_compressions[i] = compression
			
			# Calculate vertical force from spring and damper
			var spring_force = spring_rate_front * compression
			var damping_force = damping_rate_compress * _vertical_velocity_at_wheel(i)
			
			var total_vertical_force = spring_force + damping_force
			_wheel_forces[i] = total_vertical_force * brake_pressure_front
		else:
			_suspension_compressions[i] = 0.0
			_wheel_contact_normal[i] = Vector3.UP

func _vertical_velocity_at_wheel(wheel_idx: int) -> float:
	"""Get vertical velocity component at specific wheel"""
	var local_offset = wheel_positions[wheel_idx]
	var angular_vel = angular_velocity
	var linear_vel = global_velocity
	
	var vel_at_point = linear_vel + angular_vel.cross(local_offset)
	return vel_at_point.y

func _apply_drive_forces(delta: float) -> void:
	"""Apply engine drive forces to driven wheels"""
	if _current_gear == neutral_gear:
		return
	
	var torque_curve_value = _get_torque_at_rpm()
	var effective_torque = max_torque * torque_curve_value * transmission_efficiency
	
	# Gear ratio effect
	_effective_gear_ratio = gear_ratios[_current_gear - 1] if _current_gear > 0 else 1.0
	var wheel_torque = effective_torque * _effective_gear_ratio * final_drive_ratio
	
	# Distribute torque between driven wheels
	var drive_wheels = _get_driven_wheels()
	var torque_per_wheel = wheel_torque / drive_wheels.size()
	
	for wheel_idx in drive_wheels:
		var wheel_force = torque_per_wheel / tire_radius
		_wheel_forces[wheel_idx] += wheel_force * _throttle_input
		
		# Update wheel rotation
		_wheel_rotations[wheel_idx] += wheel_force * delta / tire_radius

func _get_torque_at_rpm() -> float:
	"""Lookup torque value from torque curve based on current RPM"""
	if torque_curve_points.is_empty():
		return 0.0
	
	var normalized_rpm = (_engine_rpm - engine_min_rpm) / (engine_max_rpm - engine_min_rpm)
	normalized_rpm = clamp(normalized_rpm, 0.0, 1.0)
	
	# Linear interpolation through torque curve points
	var prev_point = torque_curve_points[0]
	var next_point = torque_curve_points[torque_curve_points.size() - 1]
	
	for i in range(torque_curve_points.size() - 1):
		prev_point = torque_curve_points[i]
		next_point = torque_curve_points[i + 1]
		
		if normalized_rpm >= prev_point.x and normalized_rpm <= next_point.x:
			var t = (normalized_rpm - prev_point.x) / (next_point.x - prev_point.x)
			return prev_point.y + t * (next_point.y - prev_point.y)
	
	return next_point.y

func _apply_brake_forces(delta: float) -> void:
	"""Apply braking forces to all wheels"""
	if _brake_input <= 0.0:
		return
	
	var brake_force = _brake_input * _brake_pressure_front * vehicle_mass * 9.81
	var front_brake_force = brake_force * brake_bias
	var rear_brake_force = brake_force * (1.0 - brake_bias)
	
	# Apply to front wheels
	_wheel_forces[WHEEL_FRONT_LEFT] -= front_brake_force * delta
	_wheel_forces[WHEEL_FRONT_RIGHT] -= front_brake_force * delta
	
	# Apply to rear wheels
	_wheel_forces[WHEEL_REAR_LEFT] -= rear_brake_force * delta
	_wheel_forces[WHEEL_REAR_RIGHT] -= rear_brake_force * delta

func _apply_suspension_forces(delta: float) -> void:
	"""Apply suspension forces to maintain chassis height and stability"""
	var gravity_force = vehicle_mass * _physics.gravity
	
	var total_spring_force = 0.0
	for i in range(4):
		total_spring_force += _suspension_compressions[i] * spring_rate_front
	
	var chassis_height_error = gravity_force - total_spring_force
	
	# Apply corrective force to maintain desired ride height
	add_force(Vector3.UP * -chassis_height_error * delta * 0.1)

func _handle_drift(delta: float) -> void:
	"""Handle drift mechanics when conditions are met"""
	var sideways_velocity = global_velocity.dot(global_transform.basis.x)
	var drift_threshold_met = abs(sideways_velocity) > drift_threshold * _current_speed
	
	if drift_threshold_met and not _drifting:
		_drifting = true
		_drift_angle = sideways_velocity
	(drift_started.emit()
	
	if _drifting:
		# Apply drift physics - reduce lateral grip
		var drift_damping = 1.0 - drift_recovery_factor
		sideways_velocity *= drift_damping
		
		# Accumulate drift score
		_drift_score += abs(sideways_velocity) * delta * 10.0
		
		# Apply oversteer correction
		if _steering_input != 0:
			var correction = _steering_input * oversteer_correction * drift_score
			angular_velocity.z += correction * delta
	
	elif _drifting:
		# End drifting
		_drifting = false
		_drift_score = 0.0
		_drift_ended.emit()

func _update_body_velocity(delta: float) -> void:
	"""Update body velocity based on applied forces"""
	_current_speed = global_velocity.length()
	speed_changed.emit(_current_speed)

func _update_transform() -> void:
	"""Update visual transform based on physics state"""
	var steering_angle = _steering_input * 0.5  # Max 0.5 rad (~28 degrees)
	
	# Apply steering rotation to front wheels (visual only)
	if has_node("/root/Main/Car/Wheels"):
		var front_left = get_node("/root/Main/Car/Wheels/FrontLeft")
		var front_right = get_node("/root/Main/Car/Wheels/FrontRight")
		
		if front_left and front_right:
			front_left.rotation.y = steering_angle
			front_right.rotation.y = -steering_angle

# ============================================================================
# HELPER METHODS
# ============================================================================

func _get_driven_wheels() -> Array[int]:
	"""Return indices of driven wheels based on drivetrain type"""
	return [WHEEL_REAR_LEFT, WHEEL_REAR_RIGHT]  # RWD default

func _get_tire_material() -> PhysicsMaterial:
	"""Create or return tire physics material"""
	var mat = PhysicsMaterial.new()
	mat.friction = tire_friction_coef
	mat.bounce = 0.1
	return mat

func _set_vehicle_mass(value: float) -> void:
	vehicle_mass = value
	_calculate_derived_values()

func _set_center_of_mass(value: Vector3) -> void:
	center_of_mass_offset = value
	_calculate_derived_values()

func _set_wheel_base(value: float) -> void:
	wheel_base = value
	_calculate_derived_values()

func _set_track_width(value: float) -> void:
	track_width = value
	_calculate_derived_values()

func _set_ground_clearance(value: float) -> void:
	ground_clearance = value
	_calculate_derived_values()

func _set_drag_coefficient(value: float) -> void:
	drag_coefficient = value
	_calculate_derived_values()

func _set_frontal_area(value: float) -> void:
	frontal_area = value
	_calculate_derived_values()

func _set_roll_stiffness_front(value: float) -> void:
	roll_stiffness_front = value
	_calculate_derived_values()

func _set_roll_stiffness_rear(value: float) -> void:
	roll_stiffness_rear = value
	_calculate_derived_values()

func _set_camber_front(value: float) -> void:
	camber_angle_front = value
	_calculate_derived_values()

func _set_camber_rear(value: float) -> void:
	camber_angle_rear = value
	_calculate_derived_values()

func _set_toe_front(value: float) -> void:
	toe_angle_front = value
	_calculate_derived_values()

func _set_toe_rear(value: float) -> void:
	toe_angle_rear = value
	_calculate_derived_values()

func _set_engine_max_rpm(value: float) -> void:
	engine_max_rpm = value
	_calculate_derived_values()

func _set_engine_min_rpm(value: float) -> void:
	engine_min_rpm = value
	_calculate_derived_values()

func _set_idle_rpm(value: float) -> void:
	idle_rpm = value
	_calculate_derived_values()

func _set_max_torque(value: float) -> void:
	max_torque = value
	_calculate_derived_values()

func _set_torque_curve(value: Array[Vector2]) -> void:
	torque_curve_points = value
	_calculate_derived_values()

func _set_gear_ratios(value: Array[float]) -> void:
	gear_ratios = value
	_calculate_derived_values()

func _set_final_drive_ratio(value: float) -> void:
	final_drive_ratio = value
	_calculate_derived_values()

func _set_transmission_efficiency(value: float) -> void:
	transmission_efficiency = value
	_calculate_derived_values()

func _set_clutch_engagement(value: float) -> void:
	clutch_engagement_rpm = value
	_calculate_derived_values()

func _set_brake_pressure_front(value: float) -> void:
	brake_pressure_front = value
	_calculate_derived_values()

func _set_brake_pressure_rear(value: float) -> void:
	brake_pressure_rear = value
	_calculate_derived_values()

func _set_brake_disc_diameter(value: float) -> void:
	brake_disc_diameter = value
	_calculate_derived_values()

func _set_brake_pad_friction(value: float) -> void:
	brake_pad_friction = value
	_calculate_derived_values()

func _set_tire_radius(value: float) -> void:
	tire_radius = value
	_calculate_derived_values()

func get_current_speed() -> float:
	return _current_speed

func get_current_rpm() -> float:
	return _engine_rpm

func get_current_gear() -> int:
	return _current_gear

func get_is_drifting() -> bool:
	return _drifting

func get_drift_score() -> float:
	return _drift_score

func reset_vehicle() -> void:
	"""Reset vehicle to initial state"""
	_current_speed = 0.0
	_engine_rpm = idle_rpm
	_current_gear = 1
	_drifting = false
	_drift_score = 0.0
	global_velocity = Vector3.ZERO
	angular_velocity = Vector3.ZERO
	
	for i in range(4):
		_wheel_rotations[i] = 0.0
		_wheel_slip_ratios[i] = 0.0
		_wheel_forces[i] = 0.0
		_suspension_compressions[i] = 0.0

func _set_vehicle_mass(value: float) -> void:
	vehicle_mass = value
	_calculate_derived_values()

func _set_center_of_mass(value: Vector3) -> void:
	center_of_mass_offset = value
	_calculate_derived_values()

func _set_wheel_base(value: float) -> void:
	wheel_base = value
	_calculate_derived_values()

func _set_track_width(value: float) -> void:
	track_width = value
	_calculate_derived_values()

func _set_ground_clearance(value: float) -> void:
	ground_clearance = value
	_calculate_derived_values()

func _set_drag_coefficient(value: float) -> void:
	drag_coefficient = value
	_calculate_derived_values()

func _set_frontal_area(value: float) -> void:
	frontal_area = value
	_calculate_derived_values()

func _set_roll_stiffness_front(value: float) -> void:
	roll_stiffness_front = value
	_calculate_derived_values()

func _set_roll_stiffness_rear(value: float) -> void:
	roll_stiffness_rear = value
	_calculate_derived_values()

func _set_camber_front(value: float) -> void:
	camber_angle_front = value
	_calculate_derived_values()

func _set_camber_rear(value: float) -> void:
	camber_angle_rear = value
	_calculate_derived_values()

func _set_toe_front(value: float) -> void:
	toe_angle_front = value
	_calculate_derived_values()

func _set_toe_rear(value: float) -> void:
	toe_angle_rear = value
	_calculate_derived_values()

func _set_engine_max_rpm(value: float) -> void:
	engine_max_rpm = value
	_calculate_derived_values()

func _set_engine_min_rpm(value: float) -> void:
	engine_min_rpm = value
	_calculate_derived_values()

func _set_idle_rpm(value: float) -> void:
	idle_rpm = value
	_calculate_derived_values()

func _set_max_torque(value: float) -> void:
	max_torque = value
	_calculate_derived_values()

func _set_torque_curve(value: Array[Vector2]) -> void:
	torque_curve_points = value
	_calculate_derived_values()

func _set_gear_ratios(value: Array[float]) -> void:
	gear_ratios = value
	_calculate_derived_values()

func _set_final_drive_ratio(value: float) -> void:
	final_drive_ratio = value
	_calculate_derived_values()

func _set_transmission_efficiency(value: float) -> void:
	transmission_efficiency = value
	_calculate_derived_values()

func _set_clutch_engagement(value: float) -> void:
	clutch_engagement_rpm = value
	_calculate_derived_values()

func _set_brake_pressure_front(value: float) -> void:
	brake_pressure_front = value
	_calculate_derived_values()

func _set_brake_pressure_rear(value: float) -> void:
	brake_pressure_rear = value
	_calculate_derived_values()

func _set_brake_disc_diameter(value: float) -> void:
	brake_disc_diameter = value
	_calculate_derived_values()

func _set_brake_pad_friction(value: float) -> void:
	brake_pad_friction = value
	_calculate_derived_values()

func _set_tire_radius(value: float) -> void:
	tire_radius = value
	_calculate_derived_values()

func wheel_get_rotation(wheel_index: int) -> float:
	return _wheel_rotations[wheel_index]

func wheel_get_slip_ratio(wheel_index: int) -> float:
	return _wheel_slip_ratios[wheel_index]

func wheel_get_force(wheel_index: int) -> float:
	return _wheel_forces[wheel_index]

func apply_collision_impact(force: float, point: Vector3) -> void:
	"""Called when vehicle collides with obstacle"""
	collision_impact.emit(force, point)
	
	# Apply impact damage and bounce
	var impact_direction = global_velocity.normalized()
	add_force(impact_direction * force * 0.5)
	global_velocity *= 0.8  # Energy loss on impact

func _set_vehicle_mass(value: float) -> void:
	vehicle_mass = value
	_calculate_derived_values()

func _set_center_of_mass(value: Vector3) -> void:
	center_of_mass_offset = value
	_calculate_derived_values()

func _set_wheel_base(value: float) -> void:
	wheel_base = value
	_calculate_derived_values()

func _set_track_width(value: float) -> void:
	track_width = value
	_calculate_derived_values()

func _set_ground_clearance(value: float) -> void:
	ground_clearance = value
	_calculate_derived_values()

func _set_drag_coefficient(value: float) -> void:
	drag_coefficient = value
	_calculate_derived_values()

func _set_frontal_area(value: float) -> void:
	frontal_area = value
	_calculate_derived_values()

func _set_roll_stiffness_front(value: float) -> void:
	roll_stiffness_front = value
	_calculate_derived_values()

func _set_roll_stiffness_rear(value: float) -> void:
	roll_stiffness_rear = value
	_calculate_derived_values()

func _set_camber_front(value: float) -> void:
	camber_angle_front = value
	_calculate_derived_values()

func _set_camber_rear(value: float) -> void:
	camber_angle_rear = value
	_calculate_derived_values()

func _set_toe_front(value: float) -> void:
	toe_angle_front = value
	_calculate_derived_values()

func _set_toe_rear(value: float) -> void:
	toe_angle_rear = value
	_calculate_derived_values()

func _set_engine_max_rpm(value: float) -> void:
	engine_max_rpm = value
	_calculate_derived_values()

func _set_engine_min_rpm(value: float) -> void:
	engine_min_rpm = value
	_calculate_derived_values()

func _set_idle_rpm(value: float) -> void:
	idle_rpm = value
	_calculate_derived_values()

func _set_max_torque(value: float) -> void:
	max_torque = value
	_calculate_derived_values()

func _set_torque_curve(value: Array[Vector2]) -> void:
	torque_curve_points = value
	_calculate_derived_values()

func _set_gear_ratios(value: Array[float]) -> void:
	gear_ratios = value
	_calculate_derived_values()

func _set_final_drive_ratio(value: float) -> void:
	final_drive_ratio = value
	_calculate_derived_values()

func _set_transmission_efficiency(value: float) -> void:
	transmission_efficiency = value
	_calculate_derived_values()

func _set_clutch_engagement(value: float) -> void:
	clutch_engagement_rpm = value
	_calculate_derived_values()

func _set_brake_pressure_front(value: float) -> void:
	brake_pressure_front = value
	_calculate_derived_values()

func _set_brake_pressure_rear(value: float) -> void:
	brake_pressure_rear = value
	_calculate_derived_values()

func _set_brake_disc_diameter(value: float) -> void:
	brake_disc_diameter = value
	_calculate_derived_values()

func _set_brake_pad_friction(value: float) -> void:
	brake_pad_friction = value
	_calculate_derived_values()

func _set_tire_radius(value: float) -> void:
	tire_radius = value
	_calculate_derived_values()
