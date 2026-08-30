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
@export var vehicle_mass: float = 1500.0
@export var center_of_mass_offset: Vector3 = Vector3(0.0, 0.5, 0.0)
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
@export var clutch_engagement_rrpm: float = 1200.0
@export var rev_matching_enabled: bool = true

@export_group("Braking System")
@export var brake_pressure_front: float = 8.0
@export var brake_pressure_rear: float = 6.0
@export var brake_balance_front: float = 0.60
@export var brake_disc_radius: float = 0.30
@export var brake_pad_friction: float = 0.40
@export var brake_caliper_piston_area: float = 0.002
@export var abs_enabled: bool = true
@export var abs_threshold: float = 0.15
@export var brake_bias_dynamic: bool = true

@export_group("Tire & Suspension")
@export var tire_cornering_stiffness: float = 80000.0
@export var tire_longitudinal_stiffness: float = 100000.0
@export var tire_max_slip_ratio: float = 0.20
@export var tire_max_slip_angle: float = 0.35
@export var suspension_stiffness: float = 35000.0
@export var suspension_damping_compression: float = 8000.0
@export var suspension_damping_rebound: float = 4000.0
@export var suspension_travel: float = 0.15
@export var anti_roll_bar_stiffness: float = 5000.0

@export_group("Aerodynamics")
@export var downforce_coefficient: float = 0.5
@export var lift_coefficient: float = 0.1
@export var aero_reference_area: float = 2.2
@export var aero_center_x: float = 0.0
@export var aero_center_y: float = 0.6
@export var wing_angle: float = 0.0

# ============================================================================
# WHEEL DEFINITIONS
# ============================================================================

@export_group("Wheel Properties")
@export var wheel_radius: float = 0.32
@export var wheel_width: float = 0.22
@export var wheel_tire_width: float = 0.25
@export var wheel_inertia: float = 0.8

# ============================================================================
# INTERNAL STATE VARIABLES
# ============================================================================

var current_rpm: float = 0.0
var current_gear: int = 0
var target_gear: int = 0
var clutch_position: float = 1.0
var throttle_input: float = 0.0
var brake_input: float = 0.0
var steering_input: float = 0.0
var handbrake_input: float = 0.0

var wheel_positions: Array[Vector3] = []
var wheel_rotations: Array[float] = []
var wheel_vertical_displacements: Array[float] = []
var wheel_contact_normals: Array[Vector3] = []
var wheel_ground_velocities: Array[Vector3] = []

var current_speed: float = 0.0
var current_velocity: Vector3 = Vector3.ZERO
var angular_velocity: Vector3 = Vector3.ZERO
var acceleration: Vector3 = Vector3.ZERO
var lateral_acceleration: float = 0.0

var traction_control_active: bool = false
var drift_mode: bool = false
var drift_angle: float = 0.0
var drift_coefficient: float = 0.90

var air_time: float = 0.0
var last_ground_contact: float = 0.0
var on_ground: bool = false
var suspended: bool = false

var suspension_states: Array[Dictionary] = []
var tire_forces: Array[Vector3] = []
var tire_slip_ratios: Array[float] = []
var tire_slip_angles: Array[float] = []

var engine_braking_force: float = 0.0
var regenerative_braking: float = 0.0
var fuel_level: float = 100.0
var fuel_consumption_rate: float = 0.05

var lap_start_time: float = 0.0
var last_checkpoint: String = ""
var checkpoint_times: Dictionary = {}

var body_rotation: float = 0.0
var pitch_angle: float = 0.0
var roll_angle: float = 0.0
var yaw_angle: float = 0.0

var suspension_heights: Array[float] = []
var suspension_forces: Array[Vector3] = []
var spring_damper_states: Array[Dictionary] = []

var chassis_velocity_local: Vector3 = Vector3.ZERO
var wheel_linear_velocities: Array[Vector3] = []
var wheel_angular_velocities: Array[float] = []

var collision_events: Array[Dictionary] = []
var impact_history: Array[Dictionary] = []
var max_collision_impact: float = 0.0

var drift_timer: float = 0.0
var drift_threshold: float = 0.30
var drift_recovery_rate: float = 0.05

var gear_shift_timer: float = 0.0
var shift_duration: float = 0.15
var is_shifting: bool = false

var turbo_charge_level: float = 0.0
var turbo_spool: float = 0.0
var boost_pressure: float = 0.0

var traction_loss_events: Array[Dictionary] = []
var grip_levels: Array[float] = []

var physics_frame_count: int = 0
var last_update_time: float = 0.0

# ============================================================================
# WHEEL POSITIONS
# ============================================================================

const FRONT_LEFT: int = 0
const FRONT_RIGHT: int = 1
const REAR_LEFT: int = 2
const REAR_RIGHT: int = 3

func _ready() -> void:
	_init_wheel_positions()
	_init_suspension_states()
	_init_tire_states()
	_reset_state()
	
	if Engine.is_editor_hint():
		return
	
	_process_mode = ProcessModeEnum.ALWAYS
	set_physics_process(true)

func _init_wheel_positions() -> void:
	var half_track = track_width * 0.5
	var half_wheelbase = wheel_base * 0.5
	var wheel_y = wheel_radius + ground_clearance
	
	wheel_positions.resize(4)
	wheel_positions[FRONT_LEFT] = Vector3(-half_track, -ground_clearance, half_wheelbase)
	wheel_positions[FRONT_RIGHT] = Vector3(half_track, -ground_clearance, half_wheelbase)
	wheel_positions[REAR_LEFT] = Vector3(-half_track, -ground_clearance, -half_wheelbase)
	wheel_positions[REAR_RIGHT] = Vector3(half_track, -ground_clearance, -half_wheelbase)
	
	for i in range(4):
		wheel_rotations.append(0.0)
		wheel_vertical_displacements.append(0.0)
		wheel_contact_normals.append(Vector3.UP)
		wheel_ground_velocities.append(Vector3.ZERO)

func _init_suspension_states() -> void:
	suspension_states.resize(4)
	spring_damper_states.resize(4)
	suspension_heights.resize(4)
	suspension_forces.resize(4)
	grip_levels.resize(4)
	
	for i in range(4):
		suspension_states[i] = {
			"position": 0.0,
			"velocity": 0.0,
			"compressed": 0.0,
			"extended": 0.0
		}
		spring_damper_states[i] = {
			"damping_force": 0.0,
			"spring_force": 0.0,
			"max_compression": -suspension_travel,
			"max_extension": suspension_travel * 0.5
		}
		suspension_heights[i] = 0.0
		suspension_forces[i] = Vector3.ZERO
		grip_levels[i] = 1.0

func _init_tire_states() -> void:
	tire_forces.resize(4)
	tire_slip_ratios.resize(4)
	tire_slip_angles.resize(4)
	wheel_linear_velocities.resize(4)
	wheel_angular_velocities.resize(4)
	
	for i in range(4):
		tire_forces[i] = Vector3.ZERO
		tire_slip_ratios[i] = 0.0
		tire_slip_angles[i] = 0.0
		wheel_linear_velocities[i] = Vector3.ZERO
		wheel_angular_velocities[i] = 0.0

func _reset_state() -> void:
	current_rpm = idle_rpm
	current_gear = 1
	target_gear = 1
	clutch_position = 1.0
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	handbrake_input = 0.0
	
	current_speed = 0.0
	current_velocity = Vector3.ZERO
	angular_velocity = Vector3.ZERO
	acceleration = Vector3.ZERO
	lateral_acceleration = 0.0
	
	on_ground = false
	suspended = false
	traction_control_active = false
	drift_mode = false
	drift_angle = 0.0
	
	fuel_level = 100.0
	body_rotation = 0.0
	pitch_angle = 0.0
	roll_angle = 0.0
	yaw_angle = 0.0
	
	air_time = 0.0
	last_ground_contact = 0.0
	is_shifting = false
	turbo_charge_level = 0.0
	turbo_spool = 0.0
	boost_pressure = 0.0
	
	collision_events.clear()
	impact_history.clear()
	max_collision_impact = 0.0
	drift_timer = 0.0
	gear_shift_timer = 0.0
	traction_loss_events.clear()
	
	physics_frame_count = 0
	last_update_time = Time.get_ticks_msec() / 1000.0

# ============================================================================
# MAIN UPDATE LOOP
# ============================================================================

func _physics_process(delta: float) -> void:
	physics_frame_count += 1
	last_update_time = Time.get_ticks_msec() / 1000.0
	
	_update_inputs(delta)
	_update_engine_rpm(delta)
	_update_gearing(delta)
	_update_vehicle_motion(delta)
	_update_suspension(delta)
	_update_tires(delta)
	_update_aerodynamics(delta)
	_update_drift(delta)
	_update_traction_control(delta)
	_update_collision_detection(delta)
	_apply_forces(delta)
	_update_fuel(delta)
	
	emit_signals(delta)

func _update_inputs(delta: float) -> void:
	var input_manager = GameManager.get_node_or_null("/root/InputManager")
	
	if input_manager != null:
		throttle_input = clamp(input_manager.get_axis("throttle", 0.0), 0.0, 1.0)
		brake_input = clamp(input_manager.get_axis("brake", 0.0), 0.0, 1.0)
		steering_input = clamp(input_manager.get_axis("steering_left", -input_manager.get_axis("steering_right", 0.0)), -1.0, 1.0)
		handbrake_input = clamp(input_manager.get_axis("handbrake", 0.0), 0.0, 1.0)
		
		if input_manager.is_action_pressed("gear_up"):
			_request_gear_change(1)
		if input_manager.is_action_pressed("gear_down"):
			_request_gear_change(-1)
		if input_manager.is_action_pressed("toggle_drift"):
			drift_mode = !drift_mode
	else:
		# Fallback keyboard inputs
		if Input.is_key_pressed(KEY_W) or Input.is_key_pressed(KEY_UP):
			throttle_input = 1.0
		elif Input.is_key_pressed(KEY_S) or Input.is_key_pressed(KEY_DOWN):
			brake_input = 1.0
		
		if Input.is_key_pressed(KEY_A) or Input.is_key_pressed(KEY_LEFT):
			steering_input = -1.0
		elif Input.is_key_pressed(KEY_D) or Input.is_key_pressed(KEY_RIGHT):
			steering_input = 1.0
		
		if Input.is_key_pressed(KEY_SPACE):
			handbrake_input = 1.0

func _request_gear_change(direction: int) -> void:
	if is_shifting:
		return
	
	target_gear = current_gear + direction
	target_gear = clamp(target_gear, 0, gear_ratios.size() - 1)
	
	if target_gear != current_gress:
		perform_gear_shift(direction > 0)

func perform_gear_shift(upshift: bool) -> void:
	is_shifting = true
	gear_shift_timer = shift_duration
	
	var old_gear = current_gear
	current_gear = target_gear
	
	if rev_matching_enabled:
		var target_rpm = _calculate_target_rpm_for_gear(current_gear)
		current_rpm = lerp(current_rpm, target_rpm, 0.5)
	
	gear_changed.emit(current_gear)
	
	await get_tree().create_timer(shift_duration).timeout
	is_shifting = false

func _calculate_target_rpm_for_gear(gear: int) -> float:
	if gear == 0:
		return idle_rpm
	
	var wheel_rpm = current_speed / (wheel_radius * 2.0 * PI) * 60.0
	var total_ratio = gear_ratios[gear] * final_drive_ratio
	var engine_rpm = wheel_rpm * total_ratio
	
	return clamp(engine_rpm, engine_min_rpm, engine_max_rpm)

func _update_engine_rpm(delta: float) -> void:
	var gear_ratio = gear_ratios[current_gear] if current_gear >= 0 else 1.0
	var wheel_speed = current_speed / (wheel_radius * 2.0 * PI)
	var wheel_rpm = wheel_speed * 60.0
	var theoretical_rpm = wheel_rpm * gear_ratio * final_drive_ratio
	
	var clutch_effect = pow(clutch_position, 3.0)
	current_rpm = lerp(current_rpm, theoretical_rpm, delta * 5.0 * clutch_effect)
	
	# Apply engine braking when no throttle
	if throttle_input <= 0.01:
		current_rpm = lerp(current_rpm, idle_rpm, delta * 10.0)
	
	# Clamp RPM
	current_rpm = clamp(current_rpm, engine_min_rpm, engine_max_rpm)
	
	if abs(current_rpm - engine_max_rpm) < 100.0:
		current_rpm = engine_max_rpm

func _update_gearing(delta: float) -> void:
	if is_shifting:
		clutch_position = lerp(clutch_position, 0.0, delta * 10.0)
	else:
		clutch_position = lerp(clutch_position, 1.0, delta * 5.0)
	
	# Calculate effective gear ratio
	var effective_ratio = gear_ratios[current_gear] * final_drive_ratio * transmission_efficiency

# ============================================================================
# VEHICLE MOTION UPDATE
# ============================================================================

func _update_vehicle_motion(delta: float) -> void:
	var mass_inv = 1.0 / vehicle_mass
	
	# Calculate aerodynamic drag
	var air_density: float = 1.225
	var drag_force = 0.5 * air_density * drag_coefficient * frontal_area * current_speed * current_speed
	var drag_vector = -current_velocity.normalized() * drag_force
	
	# Update velocity
	acceleration = acceleration.lerp(drag_vector * mass_inv, delta * 10.0)
	
	# Apply acceleration to velocity
	var new_velocity = current_velocity + acceleration * delta
	
	# Handle air time
	if not on_ground:
		air_time += delta
		new_velocity.y -= _physics.gravity * delta
	else:
		air_time = 0.0
	
	current_velocity = new_velocity
	current_speed = current_velocity.length()
	
	# Update position
	position += current_velocity * delta
	
	# Calculate lateral acceleration
	var forward_dir = transform.basis.z * -1.0
	var side_dir = transform.basis.x
	
	var projected_vel = current_velocity.project(side_dir)
	lateral_acceleration = projected_vel.length() / delta
	
	# Update rotation based on steering
	if on_ground:
		_body_rotation += steering_input * delta * 2.0
		transform.basis = transform.basis.slerp(transform.basis.rotated(Vector3.UP, steering_input * delta * 3.0), delta * 5.0)
	
	# Extract Euler angles
	yaw_angle = transform.basis.get_euler().y
	pitch_angle = transform.basis.get_euler().x
	roll_angle = transform.basis.get_euler().z

func _body_rotation += value:
	pass

func _update_suspension(delta: float) -> void:
	for i in range(4):
		var wheel_pos_global = global_transform * wheel_positions[i]
		var raycast_result = _raycast_to_ground(wheel_pos_global)
		
		var suspension_length = raycast_result.distance
		var suspension_compression = suspension_length - wheel_radius - ground_clearance
		
		var previous_compression = suspension_states[i]["position"]
		var compression_velocity = (suspension_compression - previous_compression) / delta
		
		# Spring force
		var spring_force = -suspension_stiffness * suspension_compression
		
		# Damping force
		var damping_force = -suspension_damping_compression * compression_velocity if compression_velocity < 0 else -suspension_damping_rebound * compression_velocity
		
		var total_force = spring_force + damping_force
		total_force = clamp(total_force, -vehicle_mass * _physics.gravity * 3.0, vehicle_mass * _physics.gravity * 3.0)
		
		# Update suspension state
		suspension_states[i]["position"] = suspension_compression
		suspension_states[i]["velocity"] = compression_velocity
		suspension_states[i]["compressed"] = suspension_compression
		
		spring_damper_states[i]["spring_force"] = spring_force
		spring_damper_states[i]["damping_force"] = damping_force
		
		suspension_heights[i] = suspension_compression
		suspension_forces[i] = Vector3.UP * total_force
		
		# Check ground contact
		if suspension_compression > -0.01:
			on_ground = true
			last_ground_contact = Time.get_ticks_msec() / 1000.0
			suspension_states[i]["contact"] = true
		else:
			suspension_states[i]["contact"] = false

func _raycast_to_ground(wheel_pos: Vector3) -> Dictionary:
	var space_state = get_world_3d().direct_space_state
	var query = PhysicsRayQueryParameters3D.create(global_position, wheel_pos)
	query.exclude = [self]
	query.collision_mask = 1
	
	var result = space_state.intersect_ray(query)
	
	if result.is_empty():
		return {"hit": false, "distance": 999.0, "normal": Vector3.UP}
	
	return {
		"hit": true,
		"distance": result.position.y - wheel_pos.y,
		"normal": result.normal,
		"position": result.position
	}

# ============================================================================
# TIRE MODEL & FORCES
# ============================================================================

func _update_tires(delta: float) -> void:
	for i in range(4):
		_update_single_tire(i, delta)

func _update_single_tire(wheel_index: int, delta: float) -> void:
	var wheel_pos = global_transform * wheel_positions[i]
	var wheel_normal = wheel_contact_normals[i]
	
	# Calculate wheel ground velocity
	var wheel_center_velocity = current_velocity + angular_velocity.cross(wheel_positions[i])
	
	# Slip ratio calculation
	var drive_torque = _get_drive_torque(wheel_index)
	var brake_torque = _get_brake_torque(wheel_index)
	var net_torque = drive_torque - brake_torque
	
	var wheel_angular_acc = net_torque / wheel_inertia
	wheel_angular_velocities[i] += wheel_angular_acc * delta
	wheel_angular_velocities[i] = clamp(wheel_angular_velocities[i], 0.0, engine_max_rpm * 10.0)
	
	var wheel_linear_vel = wheel_angular_velocities[i] * wheel_radius
	
	var longitudinal_slip = (wheel_linear_vel - current_speed) / max(current_speed, 1.0)
	tire_slip_ratios[i] = clamp(longitudinal_slip, -tire_max_slip_ratio, tire_max_slip_ratio)
	
	# Lateral slip angle
	var slip_angle = atan2(current_velocity.x, current_velocity.z)
	tire_slip_angles[i] = clamp(slip_angle, -tire_max_slip_angle, tire_max_slip_angle)
	
	# Calculate tire forces using simplified Pacejka model
	var vertical_load = suspension_forces[i].y
	
	var cornering_force = -tire_cornering_stiffness * tire_slip_angles[i] * clamp(vertical_load, 0.0, vehicle_mass * _physics.gravity / 4.0)
	var longitudinal_force = -tire_longitudinal_stiffness * tire_slip_ratios[i] * clamp(vertical_load, 0.0, vehicle_mass * _physics.gravity / 4.0)
	
	tire_forces[i] = Vector3(cornering_force, vertical_load, longitudinal_force)
	
	# Apply friction circle
	var total_force = sqrt(pow(cornering_force, 2.0) + pow(longitudinal_force, 2.0))
	var max_friction = vertical_load * 1.0
	
	if total_force > max_friction:
		var scale = max_friction / total_force
		tire_forces[i].x *= scale
		tire_forces[i].z *= scale
		tire_slip_ratios[i] *= scale
	
	# Emit wheel slip signal
	if abs(tire_slip_ratios[i]) > 0.05:
		wheel_slip.emit(wheel_index, tire_slip_ratios[i])

func _get_drive_torque(wheel_index: int) -> float:
	if current_gear == 0:
		return 0.0
	
	var gear_ratio = gear_ratios[current_gear]
	var total_ratio = gear_ratio * final_drive_ratio
	var engine_torque = _calculate_engine_torque()
	
	var drive_torque = engine_torque * total_ratio * transmission_efficiency
	
	# Distribute torque to driven wheels
	if wheel_index == REAR_LEFT or wheel_index == REAR_RIGHT:
		return drive_torque * 0.5
	
	return 0.0

func _get_brake_torque(wheel_index: int) -> float:
	var pressure = brake_pressure_front if wheel_index < 2 else brake_pressure_rear
	var brake_force = pressure * brake_disc_radius * brake_pad_friction * brake_caliper_piston_area
	
	if wheel_index == FRONT_LEFT or wheel_index == FRONT_RIGHT:
		return brake_force * 0.5
	elif wheel_index == REAR_LEFT or wheel_index == REAR_RIGHT:
		return brake_force * 0.5 * brake_balance_front
	
	return 0.0

func _calculate_engine_torque() -> float:
	if torque_curve_points.is_empty():
		# Default torque curve approximation
		var normalized_rpm = (current_rpm - engine_min_rpm) / (engine_max_rpm - engine_min_rpm)
		return max_torque * sin(normalized_rpm * PI * 0.5)
	
	# Interpolate from torque curve
	var min_rpm = torque_curve_points[0].x
	var max_rpm = torque_curve_points[-1].x
	
	if current_rpm <= min_rpm:
		return torque_curve_points[0].y
	
	if current_rpm >= max_rpm:
		return torque_curve_points[-1].y
	
	var t = (current_rpm - min_rpm) / (max_rpm - min_rpm)
	var idx = floor(t * (torque_curve_points.size() - 1))
	idx = clamp(idx, 0, torque_curve_points.size() - 2)
	
	var p1 = torque_curve_points[idx]
	var p2 = torque_curve_points[idx + 1]
	var interpolated = p1.y + (p2.y - p1.y) * ((current_rpm - p1.x) / (p2.x - p1.x))
	
	return interpolated * throttle_input

# ============================================================================
# AERODYNAMICS UPDATE
# ============================================================================

func _update_aerodynamics(delta: float) -> void:
	var air_density: float = 1.225
	var dynamic_pressure = 0.5 * air_density * pow(current_speed, 2.0)
	
	# Downforce
	var downforce = -downforce_coefficient * aero_reference_area * dynamic_pressure
	var lift = lift_coefficient * aero_reference_area * dynamic_pressure
	
	# Apply aerodynamic forces
	var aero_force = Vector3(0.0, downforce + lift, 0.0)
	aero_force = aero_force.rotated(transform.basis.y, yaw_angle)
	
	# Turbo effect
	if throttle_input > 0.5:
		turbo_spool = lerp(turbo_spool, 1.0, delta * 2.0)
	else:
		turbo_spool = lerp(turbo_spool, 0.0, delta * 5.0)
	
	turbo_charge_level = turbo_spool * 2.0
	boost_pressure = turbo_charge_level * 1.5
	
	if turbo_charge_level > 0.0:
		aero_force.y += turbo_charge_level * 500.0

# ============================================================================
# DRIFT SYSTEM
# ============================================================================

func _update_drift(delta: float) -> void:
	if not on_ground:
		drift_mode = false
		drift_timer = 0.0
		return
	
	# Calculate drift conditions
	var lateral_accel = abs(lateral_acceleration)
	var drift_angle_calc = abs(yaw_angle - atan2(current_velocity.x, current_velocity.z))
	
	if lateral_accel > 3.0 and drift_angle_calc > drift_threshold:
		drift_timer += delta
		drift_mode = true
		
		if drift_timer > 0.5 and not signal.drift_started.is_connected(_on_drift_started):
			signal.drift_started.connect(_on_drift_started)
			drift_started.emit()
	elif drift_timer > 0.1:
		drift_timer -= delta
		if drift_timer <= 0.0:
			drift_mode = false
			drift_ended.emit()
	
	# Apply drift coefficient to tires
	if drift_mode:
		for i in range(4):
			tire_cornering_stiffness = original_cornering_stiffness * drift_coefficient
	else:
		tire_cornering_stiffness = original_cornering_stiffness

func _on_drift_started() -> void:
	pass

# ============================================================================
# TRACTION CONTROL
# ============================================================================

func _update_traction_control(delta: float) -> void:
	if throttle_input > 0.0 and on_ground:
		var drive_wheels = [REAR_LEFT, REAR_RIGHT]
		
		for wheel_idx in drive_wheels:
			if tire_slip_ratios[wheel_idx] > 0.15:
				traction_control_active = true
				
				# Reduce throttle
				throttle_input = lerp(throttle_input, throttle_input * 0.8, delta * 10.0)
				
				# Log traction loss event
				traction_loss_events.append({
					"time": Time.get_ticks_msec(),
					"wheel": wheel_idx,
					"slip": tire_slip_ratios[wheel_idx],
					"speed": current_speed
				})
				break
	else:
		traction_control_active = false
		throttle_input = throttle_input * 1.05

# ============================================================================
# COLLISION DETECTION
# ============================================================================

func _update_collision_detection(delta: float) -> void:
	var space_state = get_world_3d().direct_space_state
	var query = PhysicsShapeQueryParameters3D.create()
	query.set_shape(CapsuleShape3D.new())
	query.shape_radius = 0.5
	query.shape_height = 1.5
	query.transform = Transform3D(Basis(), position)
	query.collision_mask = 2
	
	var results = space_state.shape_get_intersections(query)
	
	for result in results:
		var impact_force = _calculate_collision_impact(result)
		
		if impact_force > max_collision_impact:
			max_collision_impact = impact_force
		
		collision_events.append({
			"timestamp": Time.get_ticks_msec(),
			"force": impact_force,
			"position": result.position
		})
		
		collision_impact.emit(impact_force, result.position)

func _calculate_collision_impact(result: Dictionary) -> float:
	var relative_velocity = current_velocity - result.velocity
	var impact_magnitude = relative_velocity.length()
	
	return impact_magnitude * vehicle_mass

# ============================================================================
# FORCE APPLICATION
# ============================================================================

func _apply_forces(delta: float) -> void:
	# Apply suspension forces
	for i in range(4):
		force_add_force_at_position(suspension_forces[i], global_transform * wheel_positions[i])
	
	# Apply tire forces
	for i in range(4):
		force_add_force_at_position(tire_forces[i], global_transform * wheel_positions[i])
	
	# Apply aerodynamic forces
	_update_aerodynamics(delta)

# ============================================================================
# FUEL MANAGEMENT
# ============================================================================

func _update_fuel(delta: float) -> void:
	var consumption = fuel_consumption_rate * throttle_input * delta
	fuel_level -= consumption
	
	if fuel_level <= 0.0:
		fuel_level = 0.0
		throttle_input = 0.0
		current_rpm = lerp(current_rpm, engine_min_rpm, delta * 5.0)

# ============================================================================
# SIGNAL EMITTERS
# ============================================================================

func emit_signals(delta: float) -> void:
	speed_changed.emit(current_speed)
	rpm_changed.emit(current_rpm)

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================

func get_current_power() -> float:
	var engine_torque = _calculate_engine_torque()
	var power = engine_torque * current_rpm * (2.0 * PI) / 60.0
	return power / 745.7  # Convert to horsepower

func get_vehicle_weight_distribution() -> Dictionary:
	var total_weight = vehicle_mass * _physics.gravity
	var front_weight = total_weight * 0.45
	var rear_weight = total_weight * 0.55
	
	return {
		"front": front_weight,
		"rear": rear_weight,
		"total": total_weight
	}

func reset_lap() -> void:
	lap_start_time = Time.get_ticks_unix()
	checkpoint_times.clear()
	last_checkpoint = ""

func record_checkpoint(checkpoint_id: String) -> void:
	checkpoint_times[checkpoint_id] = Time.get_ticks_unix() - lap_start_time
	last_checkpoint = checkpoint_id

func get_lap_time() -> float:
	return Time.get_ticks_unix() - lap_start_time

func get_checkpoint_time(checkpoint_id: String) -> float:
	return checkpoint_times.get(checkpoint_id, -1.0)

func set_vehicle_position(pos: Vector3, rot: Quaternion) -> void:
	position = pos
	rotation = rot.to_euler()

func set_vehicle_velocity(vel: Vector3) -> void:
	current_velocity = vel

func apply_impulse(force: Vector3, point: Vector3) -> void:
	apply_central_impulse(force)

func is_valid_gear(gear: int) -> bool:
	return gear >= 0 and gear < gear_ratios.size()

func get_optimal_upshift_rpm() -> float:
	return engine_max_rpm * 0.85

func get_optimal_downshift_rpm() -> float:
	return engine_min_rpm * 1.3

func calculate_stopping_distance(speed: float) -> float:
	var deceleration = brake_pressure_front * 9.81
	return pow(speed, 2.0) / (2.0 * deceleration)

func calculate_cornering_g-force(turn_radius: float) -> float:
	if turn_radius <= 0.0:
		return 0.0
	var g_force = pow(current_speed, 2.0) / (turn_radius * 9.81)
	return g_force

func simulate_brake_test() -> Dictionary:
	var initial_speed = current_speed
	var start_time = Time.get_ticks_msec() / 1000.0
	
	brake_input = 1.0
	var stop_speed = 0.0
	
	while current_speed > stop_speed and current_speed > 0.1:
		await get_tree().process_frame
	
	var test_time = Time.get_ticks_msec() / 1000.0 - start_time
	var distance = calculate_stopping_distance(initial_speed)
	
	return {
		"initial_speed": initial_speed,
		"final_speed": current_speed,
		"test_time": test_time,
		"stopping_distance": distance
	}

func debug_dump_state() -> Dictionary:
	return {
		"rpm": current_rpm,
		"gear": current_gear,
		"speed": current_speed,
		"throttle": throttle_input,
		"brake": brake_input,
		"steering": steering_input,
		"fuel": fuel_level,
		"on_ground": on_ground,
		"drift_mode": drift_mode,
		"traction_control": traction_control_active
	}