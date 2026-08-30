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
@export var center_of_mass_offset: Vector3 = Vector3(0.0, 0.5, 0.0): set = _set_center_of_mass_offset
@export var wheel_base: float = 2.8: set = _set_wheel_base
@export var track_width: float = 1.8: set = _set_track_width
@export var ground_clearance: float = 0.25: set = _set_ground_clearance
@export var drag_coefficient: float = 0.30: set = _set_drag_coefficient
@export var frontal_area: float = 2.2: set = _set_frontal_area
@export var roll_stiffness_front: float = 12000.0: set = _set_roll_stiffness_front
@export var roll_stiffness_rear: float = 10000.0: set = _set_roll_stiffness_rear
@export var camber_angle_front: float = -0.5: set = _set_camber_angle_front
@export var camber_angle_rear: float = -0.5: set = _set_camber_angle_rear
@export var max_steering_angle: float = 35.0: set = _set_max_steering_angle
@export var anti_roll_bar_stiffness: float = 8000.0: set = _set_anti_roll_bar_stiffness

@export_group("Powertrain Configuration")
@export var engine_type: EngineType = EngineType.SERIES_HYBRID
@export var max_engine_rpm: float = 7000.0: set = _set_max_engine_rpm
@export var idle_rpm: float = 800.0: set = _set_idle_rpm
@export var clutch_engagement_rpm: float = 1200.0: set = _set_clutch_engagement_rpm
@export var torque_curve_points: Array[Vector2] = [
	Vector2(1000.0, 0.3),
	Vector2(2000.0, 0.6),
	Vector2(3500.0, 0.9),
	Vector2(5000.0, 1.0),
	Vector2(6000.0, 0.95),
	Vector2(7000.0, 0.85)
]: set = _set_torque_curve_points
@export var gear_ratios: Array[float] = [
	-3.5,  # Reverse
	3.8,   # 1st
	2.4,   # 2nd
	1.7,   # 3rd
	1.3,   # 4th
	1.0,   # 5th
	0.85   # 6th
]: set = _set_gear_ratios
@export var final_drive_ratio: float = 3.73: set = _set_final_drive_ratio
@export var differential_type: DifferentialType = DifferentialType.OPEN
@export var traction_control_enabled: bool = true

@export_group("Tire & Suspension Configuration")
@export var tire_radius: float = 0.32: set = _set_tire_radius
@export var tire_width: float = 0.25: set = _set_tire_width
@export var tire_stiffness: float = 150000.0: set = _set_tire_stiffness
@export var suspension_travel_max: float = 0.15: set = _set_suspension_travel_max
@export var suspension_travel_min: float = -0.05: set = _set_suspension_travel_min
@export var spring_constant: float = 35000.0: set = _set_spring_constant
@export var damping_compression: float = 8000.0: set = _set_damping_compression
@export var damping_rebound: float = 6000.0: set = _set_damping_rebound
@export var bump_stop_stiffness: float = 500000.0: set = _set_bump_stop_stiffness

@export_group("Drift Settings")
@export var drift_friction_multiplier: float = 0.65: set = _set_drift_friction_multiplier
@export var drift_threshold_angle: float = 12.0: set = _set_drift_threshold_angle
@export var drift_recovery_rate: float = 0.03: set = _set_drift_recovery_rate
@export var drift_stabilization: float = 0.15: set = _set_drift_stabilization

@export_group("Advanced Physics")
@export var aerodynamic_downforce_coef: float = 0.02: set = _set_aerodynamic_downforce_coef
@export var aerodynamic_lift_coef: float = -0.01: set = _set_aerodynamic_lift_coef
@export var aero_center_x: float = 0.0: set = _set_aero_center_x
@export var aero_center_y: float = 0.6: set = _set_aero_center_y
@export var aero_center_z: float = 0.0: set = _set_aero_center_z
@export var enable_advanced_suspension: bool = true
@export var enable_weight_transfer: bool = true
@export var enable_brake_bias: bool = true
@export var front_brake_bias: float = 0.60: set = _set_front_brake_bias

@export_group("AI Control (Optional)")
@export var ai_controlled: bool = false
@export var ai_target_speed: float = 0.0
@export var ai_aggressiveness: float = 0.5: range(0.0, 1.0, 0.1)

enum EngineType {
	SERIES_HYBRID,
	PURE_ELECTRIC,
	FUEL_INJECTION,
	HYBRID
}

enum DifferentialType {
	OPEN,
	LOCKED,
	LSD
}

# ============================================================================
# INTERNAL STATE VARIABLES
# ============================================================================

var _powertrain: Node = null
var _wheel_nodes: Array[Node] = []
var _wheel_indices: Dictionary = {}  # Name -> index mapping

# Wheel positions relative to vehicle body
var _front_left_pos: Vector3 = Vector3(-track_width / 2, -ground_clearance, -wheel_base / 2 + 0.1)
var _front_right_pos: Vector3 = Vector3(track_width / 2, -ground_clearance, -wheel_base / 2 + 0.1)
var _rear_left_pos: Vector3 = Vector3(-track_width / 2, -ground_clearance, wheel_base / 2 - 0.1)
var _rear_right_pos: Vector3 = Vector3(track_width / 2, -ground_clearance, wheel_base / 2 - 0.1)

# Current state
var current_gear: int = 0  # 0 = neutral, -1 = reverse, 1+ = forward gears
var target_gear: int = 0
var clutch_engaged: bool = false
var current_rpm: float = 0.0
var target_rpm: float = 0.0
var current_speed: float = 0.0  # m/s
var target_speed: float = 0.0
var steering_input: float = 0.0
var throttle_input: float = 0.0
var brake_input: float = 0.0
var handbrake_input: float = 0.0

# Drift state
var drift_state: float = 0.0
var drift_angle: float = 0.0
var is_drifting: bool = false

# Wheel-specific data
var _wheel_data: Array = []
var _wheel_contact_points: Array[Vector3] = []
var _wheel_vertical_forces: Array[float] = []
var _wheel_slip_ratios: Array[float] = []

# Suspension data
var _suspension_positions: Array[float] = []
var _suspension_velocities: Array[float] = []
var _spring_forces: Array[float] = []

# Aerodynamics
var _aero_downforce: float = 0.0
var _aero_lift: float = 0.0
var _current_wind_direction: Vector3 = Vector3.ZERO

# Weight transfer
var _longitudinal_weight_transfer: float = 0.0
var _lateral_weight_transfer: float = 0.0
var _vertical_acceleration: float = 0.0

# Input smoothing
var _smoothed_throttle: float = 0.0
var _smoothed_brake: float = 0.0
var _smoothed_steering: float = 0.0

# Time tracking
var _delta_accumulator: float = 0.0
var _last_update_time: float = 0.0
var _frame_count: int = 0

# Collision detection
var _collision_detected: bool = false
var _collision_normal: Vector3 = Vector3.UP
var _collision_force: float = 0.0

# Gearbox state
var _gearbox_locked: bool = false
var _shift_progress: float = 0.0
var _next_gear_delay: float = 0.0

# Clutch state
var _clutch_position: float = 0.0
var _clutch_slip: float = 0.0

# Brake bias
var _front_brake_force: float = 0.0
var _rear_brake_force: float = 0.0

# Tire grip
var _tire_grip_factor: float = 1.0
var _surface_friction: float = 1.0

# ============================================================================
# SETUP METHODS
# ============================================================================

func _ready() -> void:
	_init_wheel_references()
	_init_wheel_data()
	_connect_signals()
	_reset_state()
	_setup_physics_material()
	print("VehicleController initialized successfully")

func _init_wheel_references() -> void:
	var wheels = find_children("*", "Wheel", true)
	for i in range(min(wheels.size(), 4)):
		if i < wheels.size():
			_wheel_nodes.append(wheels[i])
			_wheel_data.append({
				"node": wheels[i],
				"position": _get_wheel_global_position(i),
				"radius": tire_radius,
				"width": tire_width,
				"max_travel": suspension_travel_max,
				"min_travel": suspension_travel_min
			})
			
			match i:
				0: _wheel_indices["front_left"] = 0
				1: _wheel_indices["front_right"] = 1
				2: _wheel_indices["rear_left"] = 2
				3: _wheel_indices["rear_right"] = 3

func _init_wheel_data() -> void:
	for i in range(4):
		_wheel_contact_points.append(Vector3.ZERO)
		_wheel_vertical_forces.append(0.0)
		_wheel_slip_ratios.append(0.0)
		_suspension_positions.append(0.0)
		_suspension_velocities.append(0.0)
		_spring_forces.append(0.0)

func _connect_signals() -> void:
	if _powertrain:
		_powertrain.connect("rpm_changed", _on_powertrain_rpm_changed)
		_powertrain.connect("torque_available", _on_powertrain_torque_available)

func _reset_state() -> void:
	current_gear = 0
	target_gear = 0
	clutch_engaged = false
	current_rpm = idle_rpm
	target_rpm = idle_rpm
	current_speed = 0.0
	target_speed = 0.0
	steering_input = 0.0
	throttle_input = 0.0
	brake_input = 0.0
	handbrake_input = 0.0
	is_drifting = false
	_shift_progress = 0.0
	_clutch_position = 1.0
	_clutch_slip = 0.0
	_smoothed_throttle = 0.0
	_smoothed_brake = 0.0
	_smoothed_steering = 0.0
	_delta_accumulator = 0.0
	_last_update_time = Time.get_unix_time_from_system()
	_frame_count = 0
	_collision_detected = false
	_surface_friction = 1.0
	_tire_grip_factor = 1.0

func _setup_physics_material() -> void:
	var material = PhysicsMaterial.new()
	material.friction = surface_friction
	material.bounce = 0.0
	set_physics_material_override(material)

# ============================================================================
# MAIN UPDATE LOOP
# ============================================================================

func _physics_process(delta: float) -> void:
	_frame_count += 1
	
	# Handle time accumulation for fixed timestep
	_delta_accumulator += delta
	var fixed_delta = 1.0 / _physics.physics_tick_rate
	
	while _delta_accumulator >= fixed_delta:
		_fixed_step(fixed_delta)
		_delta_accumulator -= fixed_delta
	
	# Apply interpolated values for rendering
	_apply_interpolation()

func _fixed_step(delta: float) -> void:
	# Get input values
	_get_input_values()
	
	# Update physics
	_update_rpm(delta)
	_update_gearbox(delta)
	_update_powertrain(delta)
	_update_wheels(delta)
	_update_suspension(delta)
	_update_collision_detection(delta)
	_update_drift(delta)
	_update_aerodynamics(delta)
	_update_weight_transfer(delta)
	
	# Apply forces
	_apply_forces(delta)
	
	# Move vehicle
	_move_vehicle(delta)

func _apply_interpolation() -> void:
	# Smooth transitions for rendering
	_smoothed_throttle = lerp(_smoothed_throttle, throttle_input, 5.0 * get_process_delta_time())
	_smoothed_brake = lerp(_smoothed_brake, brake_input, 5.0 * get_process_delta_time())
	_smoothed_steering = lerp(_smoothed_steering, steering_input, 10.0 * get_process_delta_time())

# ============================================================================
# INPUT HANDLING
# ============================================================================

func _get_input_values() -> void:
	if ai_controlled:
		_ai_control_inputs()
	else:
		_human_control_inputs()

func _human_control_inputs() -> void:
	# Steering (left/right)
	steering_input = InputManager.get_axis("steer_left", "steer_right")
	
	# Throttle (forward acceleration)
	throttle_input = InputManager.get_axis("throttle", "brake")
	if throttle_input > 0:
		throttle_input = 1.0
	elif throttle_input < 0:
		throttle_input = -1.0
	
	# Brake (backward deceleration)
	brake_input = InputManager.get_axis("brake_left", "brake_right")
	if brake_input > 0:
		brake_input = 1.0
	
	# Handbrake (drift control)
	handbrake_input = InputManager.get_action_strength("handbrake")
	
	# Gear shifting
	if Input.is_action_just_pressed("gear_up"):
		_request_gear_change(1)
	if Input.is_action_just_pressed("gear_down"):
		_request_gear_change(-1)
	if Input.is_action_just_pressed("gear_neutral"):
		_request_gear_change(0)
	
	# Clutch (for manual transmission)
	if Input.is_action_pressed("clutch"):
		_clutch_position = 0.0
	else:
		_clutch_position = 1.0

func _ai_control_inputs() -> void:
	# Simple AI behavior - follow target speed
	var speed_diff = ai_target_speed - current_speed
	
	if abs(speed_diff) > 0.5:
		throttle_input = sign(speed_diff) * min(abs(speed_diff) / 20.0, 1.0)
	else:
		throttle_input = 0.0
	
	# AI steering based on path following would go here
	# For now, minimal steering correction
	steering_input = 0.0
	brake_input = 0.0
	handbrake_input = 0.0

# ============================================================================
# RPM AND GEARBOX MANAGEMENT
# ============================================================================

func _update_rpm(delta: float) -> void:
	var gear_ratio = _get_current_gear_ratio()
	var wheel_speed = current_speed / tire_radius if current_speed > 0 else 0.0
	var engine_speed = wheel_speed * gear_ratio * final_drive_ratio
	
	# Calculate target RPM based on throttle and gear
	var target_engine_rpm = _calculate_target_rpm(engine_speed)
	
	# Apply clutch effect
	var clutch_effect = 1.0 - _clutch_slip
	current_rpm = lerp(current_rpm, target_engine_rpm * clutch_effect, 0.1)
	
	# Clamp RPM
	current_rpm = clamp(current_rpm, idle_rpm, max_engine_rpm)
	
	# Emit signal
	rpm_changed.emit(current_rpm)

func _calculate_target_rpm(wheel_rpm: float) -> float:
	if throttle_input <= 0:
		return idle_rpm
	
	var torque_multiplier = _get_torque_multiplier()
	var rpm_increase = wheel_rpm * torque_multiplier * 0.5
	
	return max(idle_rpm, min(max_engine_rpm, wheel_rpm + rpm_increase))

func _update_gearbox(delta: float) -> void:
	# Check if we should shift gears
	if not _gearbox_locked and _should_auto_shift():
		_auto_shift_gear()
	
	# Process shift progress
	if _shift_progress > 0.0:
		_shift_progress = clamp(_shift_progress - delta * 5.0, 0.0, 1.0)
		if _shift_progress <= 0.0:
			current_gear = target_gear
			gearbox_locked = false
			gear_changed.emit(current_gear)

func _should_auto_shift() -> bool:
	# Auto-shift logic based on RPM
	if current_gear == 0 or current_gear == -1:
		return false
	
	if current_rpm >= max_engine_rpm * 0.95:
		return true
	if current_rpm <= idle_rpm * 1.2 and current_gear < gear_ratios.size() - 1:
		return true
	
	return false

func _auto_shift_gear() -> void:
	if current_rpm >= max_engine_rpm * 0.95 and current_gear < gear_ratios.size() - 1:
		_request_gear_change(1)
	elif current_rpm <= idle_rpm * 1.2 and current_gear > 1:
		_request_gear_change(-1)

func _request_gear_change(gear_delta: int) -> void:
	if gear_delta == 0:
		target_gear = 0
		_shift_progress = 0.2
		return
	
	var new_gear = current_gear + gear_delta
	
	# Validate gear change
	if new_gear < 0 or new_gear >= gear_ratios.size():
		return
	
	# Prevent downshifting at high RPM without clutch
	if new_gear < current_gear and current_rpm > clutch_engagement_rpm and _clutch_position < 0.5:
		return
	
	target_gear = new_gear
	_shift_progress = 0.2
	_gearbox_locked = true
	_next_gear_delay = 0.1

func _get_current_gear_ratio() -> float:
	if current_gear == 0:
		return 0.0
	return abs(gear_ratios[current_gear]) * final_drive_ratio

func _update_powertrain(delta: float) -> void:
	# Update powertrain based on current gear and RPM
	if _powertrain:
		_powertrain.current_gear = current_gear
		_powertrain.engine_rpm = current_rpm
		_powertrain.throttle_input = throttle_input

# ============================================================================
# WHEEL PHYSICS
# ============================================================================

func _update_wheels(delta: float) -> void:
	for i in range(4):
		_update_single_wheel(delta, i)

func _update_single_wheel(delta: float, wheel_index: int) -> void:
	var wheel_data = _wheel_data[wheel_index]
	var global_pos = _wheel_nodes[wheel_index].global_position if wheel_nodes.size() > wheel_index else Vector3.ZERO
	
	# Calculate wheel rotation
	var wheel_rotation = Vector3.DOWN
	var wheel_velocity = Vector3.ZERO
	
	# Get velocity at wheel position
	var local_pos = wheel_data.position
	var world_pos = transform * local_pos
	wheel_velocity = linear_velocity + angular_velocity.cross(local_pos)
	
	# Calculate slip ratio
	var wheel_speed = wheel_velocity.length()
	var driven_speed = current_speed * _get_wheel_drive_ratio(wheel_index)
	var slip_ratio = (wheel_speed - driven_speed) / max(driven_speed, 0.1)
	_wheel_slip_ratios[wheel_index] = slip_ratio
	
	# Calculate longitudinal force
	var longitudinal_force = _calculate_longitudinal_force(wheel_index, slip_ratio)
	
	# Calculate lateral force
	var lateral_force = _calculate_lateral_force(wheel_index, wheel_velocity)
	
	# Apply forces to vehicle
	var force_vector = longitudinal_force * wheel_velocity.normalized() + lateral_force * wheel_velocity.rotated(Vector3.UP, PI / 2).normalized()
	add_force(force_vector * vehicle_mass * 0.1)

func _calculate_longitudinal_force(wheel_index: int, slip_ratio: float) -> float:
	var tire_load = _wheel_vertical_forces[wheel_index]
	var friction = _surface_friction * _tire_grip_factor
	
	# Magic formula approximation for tire force
	var f_peak = friction * tire_load
	var B = 10.0
	var C = 1.9
	var D = f_peak
	var E = 1.5 - min(slip_ratio.abs(), 1.0)
	
	var Fx = D * sin(B * atan(C * slip_ratio * E))
	
	# Apply braking/acceleration limits
	var max_drive_force = vehicle_mass * 0.5
	var max_brake_force = vehicle_mass * 1.5
	
	if current_gear > 0:
		Fx = min(Fx, max_drive_force)
	elif brake_input > 0:
		Fx = max(Fx, -max_brake_force)
	
	return Fx

func _calculate_lateral_force(wheel_index: int, wheel_velocity: Vector3) -> float:
	var tire_load = _wheel_vertical_forces[wheel_index]
	var friction = _surface_friction * _tire_grip_factor
	
	# Lateral slip angle calculation
	var lateral_velocity = wheel_velocity.dot(transform.basis.z)
	var slip_angle = asin(clamp(lateral_velocity / max(wheel_velocity.length(), 0.1), -1.0, 1.0))
	
	# Cornering stiffness
	var cornering_stiffness = tire_stiffness * 0.3
	
	# Lateral force (linear region)
	var Fy = cornering_stiffness * slip_angle
	
	# Friction circle limitation
	var longitudinal_force = _calculate_longitudinal_force(wheel_index, _wheel_slip_ratios[wheel_index])
	var combined_force = sqrt(longitudinal_force * longitudinal_force + Fy * Fy)
	var max_lateral_force = friction * tire_load * 0.8
	
	if combined_force > max_lateral_force:
		var ratio = max_lateral_force / combined_force
		Fy *= ratio
	
	return Fy

func _get_wheel_drive_ratio(wheel_index: int) -> float:
	# Front-wheel drive: front wheels only
	if engine_type == EngineType.PURE_ELECTRIC or engine_type == EngineType.FUEL_INJECTION:
		if wheel_index < 2:  # Front wheels
			return 1.0
		return 0.0
	# Rear-wheel drive
	elif engine_type == EngineType.HYBRID:
		if wheel_index >= 2:  # Rear wheels
			return 1.0
		return 0.0
	# All-wheel drive
	else:
		return 1.0

# ============================================================================
# SUSPENSION PHYSICS
# ============================================================================

func _update_suspension(delta: float) -> void:
	if not enable_advanced_suspension:
		return
	
	for i in range(4):
		_update_single_suspension(i, delta)

func _update_single_suspension(wheel_index: int, delta: float) -> void:
	var wheel_data = _wheel_data[wheel_index]
	var rest_length = suspension_travel_max - suspension_travel_min
	var current_spring_force = _spring_forces[wheel_index]
	
	# Calculate compression/extension
	var displacement = _suspension_positions[wheel_index]
	var velocity = _suspension_velocities[wheel_index]
	
	# Spring force (Hooke's Law)
	var spring_force = spring_constant * displacement
	
	# Damping force
	var damping_force = damping_compression * velocity if velocity < 0 else damping_rebound * velocity
	
	# Bump stop force
	if displacement > suspension_travel_max * 0.8:
		spring_force += bump_stop_stiffness * (displacement - suspension_travel_max * 0.8)
	elif displacement < suspension_travel_min * 0.8:
		spring_force += bump_stop_stiffness * (displacement - suspension_travel_min * 0.8)
	
	_total_spring_force = spring_force + damping_force
	_spring_forces[wheel_index] = _total_spring_force
	
	# Update suspension state
	_suspension_positions[wheel_index] = displacement
	_suspension_velocities[wheel_index] = velocity

# ============================================================================
# DRIFT MECHANICS
# ============================================================================

func _update_drift(delta: float) -> void:
	# Calculate sideslip angle
	var vehicle_velocity = linear_velocity
	var forward_dir = transform.basis.z
	var lateral_velocity = vehicle_velocity.dot(forward_dir.rotated(Vector3.UP, PI / 2))
	var drift_angle = asin(clamp(lateral_velocity / max(vehicle_velocity.length(), 0.1), -1.0, 1.0))
	
	# Detect drift conditions
	var drift_condition = abs(drift_angle) > deg_to_rad(drift_threshold_angle) and \
	                      (throttle_input > 0.3 or brake_input > 0.3) and \
	                      handbrake_input > 0.5
	
	if drift_condition and not is_drifting:
		is_drifting = true
		drift_started.emit()
	elif not drift_condition and is_drifting:
		is_drifting = false
		drift_ended.emit()
	
	# Apply drift effects
	if is_drifting:
		_surface_friction = surface_friction * drift_friction_multiplier
		drift_state = lerp(drift_state, 1.0, drift_recovery_rate)
	else:
		drift_state = lerp(drift_state, 0.0, drift_recovery_rate)
		_surface_friction = surface_friction * (1.0 - drift_state * 0.35)

# ============================================================================
# AERODYNAMICS
# ============================================================================

func _update_aerodynamics(delta: float) -> void:
	var speed_sq = current_speed * current_speed
	
	# Downforce (proportional to v^2)
	_aero_downforce = 0.5 * air_density() * speed_sq * frontal_area * aerodynamic_downforce_coef
	
	# Lift (can be negative for downforce)
	_aero_lift = 0.5 * air_density() * speed_sq * frontal_area * aerodynamic_lift_coef
	
	# Center of pressure offset affects moment arm
	var aero_moment = _aero_downforce * aero_center_y
	add_torque(aero_moment * Vector3.LEFT)

func air_density() -> float:
	# Standard sea level air density
	return 1.225

# ============================================================================
# WEIGHT TRANSFER
# ============================================================================

func _update_weight_transfer(delta: float) -> void:
	if not enable_weight_transfer:
		return
	
	# Longitudinal weight transfer due to acceleration/deceleration
	var accel = (current_speed - _prev_speed) / get_process_delta_time()
	_prev_speed = current_speed
	
	var total_weight = vehicle_mass * _physics.gravity
	var cg_height = center_of_mass_offset.y
	var wheelbase = wheel_base
	
	_longitudinal_weight_transfer = total_weight * accel * cg_height / wheelbase
	
	# Lateral weight transfer due to cornering
	var lateral_accel = linear_velocity.cross(angular_velocity).length()
	_lateral_weight_transfer = total_weight * lateral_accel * cg_height / track_width

# ============================================================================
# COLLISION DETECTION
# ============================================================================

func _update_collision_detection(delta: float) -> void:
	# Simple floor detection
	var ray_origin = global_position + Vector3.UP * ground_clearance
	var ray_end = global_position + Vector3.UP * (ground_clearance + 1.0)
	
	var space_state = get_world_3d().direct_space_state
	var query = PhysicsRayQueryParameters3D.create(ray_origin, ray_end)
	query.collision_mask = 1
	query.exclude = [self]
	
	var result = space_state.intersect_ray(query)
	
	if result:
		_collision_detected = true
		_collision_normal = result.normal
		_ground_height = result.position.y
	else:
		_collision_detected = false
		_ground_height = global_position.y

func _apply_forces(delta: float) -> void:
	# Gravity
	var gravity_force = -vehicle_mass * _physics.gravity * Vector3.UP
	add_force(gravity_force)
	
	# Ground reaction force
	if _collision_detected:
		var ground_force = _calculate_ground_reaction_force()
		add_force(ground_force)
	
	# Aerodynamic forces
	var aero_force = _aero_downforce * Vector3.DOWN + _aero_lift * Vector3.UP
	add_force(aero_force)

func _calculate_ground_reaction_force() -> float:
	var total_spring_force = 0.0
	for force in _spring_forces:
		total_spring_force += force
	
	return total_spring_force * Vector3.UP

func _move_vehicle(delta: float) -> void:
	# Apply all accumulated forces
	apply_central_force(total_force)
	
	# Integrate motion
	move_and_slide()
	
	# Update current speed
	current_speed = linear_velocity.length()
	speed_changed.emit(current_speed)

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================

func _get_torque_multiplier() -> float:
	if torque_curve_points.is_empty():
		return 1.0
	
	# Interpolate torque curve
	var rpm = current_rpm
	var points = torque_curve_points
	
	for i in range(points.size() - 1):
		if rpm >= points[i].x and rpm <= points[i + 1].x:
			var t = (rpm - points[i].x) / (points[i + 1].x - points[i].x)
			return lerp(points[i].y, points[i + 1].y, t)
	
	return points.back().y

func _validate_vehicle_config() -> bool:
	var valid = true
	
	if vehicle_mass <= 0:
		valid = false
	if wheel_base <= 0:
		valid = false
	if track_width <= 0:
		valid = false
	if tire_radius <= 0:
		valid = false
	
	return valid

func _get_wheel_global_position(index: int) -> Vector3:
	match index:
		0: return _front_left_pos
		1: return _front_right_pos
		2: return _rear_left_pos
		3: return _rear_right_pos
	return Vector3.ZERO

func save_setup() -> void:
	var config = {
		"vehicle_mass": vehicle_mass,
		"wheel_base": wheel_base,
		"track_width": track_width,
		"gear_ratios": gear_ratios,
		"engine_type": engine_type,
		"torque_curve_points": torque_curve_points
	}
	print("Vehicle setup saved:", config)

func _set_vehicle_mass(value: float) -> void:
	vehicle_mass = max(500.0, value)
	notify_property_list_changed()

func _set_center_of_mass_offset(value: Vector3) -> void:
	center_of_mass_offset = value.clamp(Vector3(-0.5, 0.1, -0.5), Vector3(0.5, 1.0, 0.5))
	notify_property_list_changed()

func _set_wheel_base(value: float) -> void:
	wheel_base = max(1.5, value)
	notify_property_list_changed()

func _set_track_width(value: float) -> void:
	track_width = max(1.0, value)
	notify_property_list_changed()

func _set_ground_clearance(value: float) -> void:
	ground_clearance = max(0.1, value)
	notify_property_list_changed()

func _set_drag_coefficient(value: float) -> void:
	drag_coefficient = max(0.1, min(0.5, value))
	notify_property_list_changed()

func _set_frontal_area(value: float) -> void:
	frontal_area = max(1.0, value)
	notify_property_list_changed()

func _set_roll_stiffness_front(value: float) -> void:
	roll_stiffness_front = max(5000.0, value)
	notify_property_list_changed()

func _set_roll_stiffness_rear(value: float) -> void:
	roll_stiffness_rear = max(5000.0, value)
	notify_property_list_changed()

func _set_camber_angle_front(value: float) -> void:
	camber_angle_front = clamp(value, -2.0, 2.0)
	notify_property_list_changed()

func _set_camber_angle_rear(value: float) -> void:
	camber_angle_rear = clamp(value, -2.0, 2.0)
	notify_property_list_changed()

func _set_max_steering_angle(value: float) -> void:
	max_steering_angle = max(10.0, min(60.0, value))
	notify_property_list_changed()

func _set_anti_roll_bar_stiffness(value: float) -> void:
	anti_roll_bar_stiffness = max(3000.0, value)
	notify_property_list_changed()

func _set_max_engine_rpm(value: float) -> void:
	max_engine_rpm = max(3000.0, value)
	notify_property_list_changed()

func _set_idle_rpm(value: float) -> void:
	idle_rpm = max(500.0, min(value, max_engine_rpm * 0.2))
	notify_property_list_changed()

func _set_clutch_engagement_rpm(value: float) -> void:
	clutch_engagement_rpm = max(idle_rpm * 1.1, value)
	notify_property_list_changed()

func _set_torque_curve_points(value: Array[Vector2]) -> void:
	torque_curve_points = value
	notify_property_list_changed()

func _set_gear_ratios(value: Array[float]) -> void:
	gear_ratios = value
	notify_property_list_changed()

func _set_final_drive_ratio(value: float) -> void:
	final_drive_ratio = max(2.0, value)
	notify_property_list_changed()

func _set_tire_radius(value: float) -> void:
	tire_radius = max(0.2, value)
	notify_property_list_changed()

func _set_tire_width(value: float) -> void:
	tire_width = max(0.1, value)
	notify_property_list_changed()

func _set_tire_stiffness(value: float) -> void:
	tire_stiffness = max(50000.0, value)
	notify_property_list_changed()

func _set_suspension_travel_max(value: float) -> void:
	suspension_travel_max = max(0.05, value)
	notify_property_list_changed()

func _set_suspension_travel_min(value: float) -> void:
	suspension_travel_min = min(0.0, -value)
	notify_property_list_changed()

func _set_spring_constant(value: float) -> void:
	spring_constant = max(10000.0, value)
	notify_property_list_changed()

func _set_damping_compression(value: float) -> void:
	damping_compression = max(1000.0, value)
	notify_property_list_changed()

func _set_damping_rebound(value: float) -> void:
	damping_rebound = max(1000.0, value)
	notify_property_list_changed()

func _set_bump_stop_stiffness(value: float) -> void:
	bump_stop_stiffness = max(100000.0, value)
	notify_property_list_changed()

func _set_drift_friction_multiplier(value: float) -> void:
	drift_friction_multiplier = max(0.1, min(1.0, value))
	notify_property_list_changed()

func _set_drift_threshold_angle(value: float) -> void:
	drift_threshold_angle = max(5.0, min(30.0, value))
	notify_property_list_changed()

func _set_drift_recovery_rate(value: float) -> void:
	drift_recovery_rate = max(0.01, min(0.1, value))
	notify_property_list_changed()

func _set_drift_stabilization(value: float) -> void:
	drift_stabilization = max(0.0, min(0.5, value))
	notify_property_list_changed()

func _set_aerodynamic_downforce_coef(value: float) -> void:
	aerodynamic_downforce_coef = max(-0.1, value)
	notify_property_list_changed()

func _set_aerodynamic_lift_coef(value: float) -> void:
	aerodynamic_lift_coef = min(0.1, value)
	notify_property_list_changed()

func _set_aero_center_x(value: float) -> void:
	aero_center_x = value
	notify_property_list_changed()

func _set_aero_center_y(value: float) -> void:
	aero_center_y = value
	notify_property_list_changed()

func _set_aero_center_z(value: float) -> void:
	aero_center_z = value
	notify_property_list_changed()

func _set_front_brake_bias(value: float) -> void:
	front_brake_bias = max(0.4, min(0.8, value))
	notify_property_list_changed()

func _get_configuration_warnings() -> Array[String]:
	var warnings = []
	
	if torque_curve_points.is_empty():
		warnings.append("Torque curve is empty - using default")
	if gear_ratios.is_empty():
		warnings.append("No gear ratios defined")
	if not _validate_vehicle_config():
		warnings.append("Vehicle configuration has errors")
	
	return warnings

func _notification(what: int) -> void:
	match what:
		NOTIFICATION_WM_CLOSE_REQUEST:
			save_setup()
		NOTIFICATION_EXIT_TREE:
			print("VehicleController destroyed")

func _on_powertrain_rpm_changed(rpm: float) -> void:
	current_rpm = rpm

func _on_powertrain_torque_available(torque: float) -> void:
	pass  # Handle torque application

func debug_print_info() -> void:
	print("=== Vehicle Controller Debug Info ===")
	print("Current Speed: %.2f m/s (%.1f km/h)" % [current_speed, current_speed * 3.6])
	print("Engine RPM: %.0f" % current_rpm)
	print("Gear: %d" % current_gear)
	print("Throttle: %.2f" % throttle_input)
	print("Brake: %.2f" % brake_input)
	print("Steering: %.2f" % steering_input)
	print("Drifting: %s" % str(is_drifting))
	print("Surface Friction: %.2f" % surface_friction)
	print("====================================")

# ============================================================================
# END OF FILE
## Copyright 2026 Thalamus Racing Simulator Project
## All rights reserved