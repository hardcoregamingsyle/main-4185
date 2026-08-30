extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulation
## Handles throttle, brake, steering, wheel forces, gear shifting, and vehicle dynamics
## Uses PhysicsSettings constants for all physics values
## Copyright 2026 Thalamus Racing Simulator Project

# ============================================================================
# SIGNALS - Vehicle state and event notifications
# ============================================================================

signal speed_changed(new_speed: float)
signal rpm_changed(new_rpm: float)
signal gear_changed(old_gear: int, new_gear: int)
signal drift_started()
signal drift_ended()
signal collision_detected(velocity: Vector3)
signal lap_checkpoint_passed(lap_time: float)
signal race_lap_completed(lap_number: int)
signal engine_revving(rpm_percentage: float)
signal tire_slip_changed(front_slip: float, rear_slip: float)
signal traction_control_active(active: bool)

# ============================================================================
# CONSTANTS - Vehicle configuration from PhysicsSettings
# ============================================================================

const DEFAULT_ACCELERATION_RATE: float = 0.02
const DEFAULT_BRAKING_RATE: float = 0.03
const DEFAULT_STEERING_SPEED: float = 0.15
const MAX_STEERING_ANGLE: float = PI / 3.0  # 60 degrees max
const DRIFT_THRESHOLD: float = 0.3
const TRACTION_CONTROL_THRESHOLD: float = 0.15
const MIN_DRIFT_SPEED: float = 10.0
const MAX_DRIFT_SPEED: float = 120.0

# ============================================================================
# EXPORTED CONFIGURATION - Tunable vehicle parameters
# ============================================================================

@export_group("Vehicle Configuration")
@export var vehicle_mass: float = 1500.0
@export var center_of_mass_offset: Vector3 = Vector3(0.0, -0.5, 0.0)
@export var wheel_base: float = 2.8
@export var track_width: float = 1.6
@export var aerodynamic_drag_coefficient: float = 0.32
@export var frontal_area: float = 2.2
@export var downforce_coefficient: float = 0.8

@export_group("Engine & Powertrain")
@export var engine_max_torque: float = 450.0  # Nm
@export var engine_max_rpm: float = 7500.0
@export var idle_rpm: float = 800.0
@export var torque_curve_multiplier: float = 1.0

@export_group("Transmission")
@export var transmission_type: String = "manual"
@export var gear_ratios: Array[float] = [3.5, 2.2, 1.6, 1.2, 0.9, 0.7]
@export var final_drive_ratio: float = 3.8
@export var shift_up_threshold: float = 0.95
@export var shift_down_threshold: float = 0.25

@export_group("Brakes")
@export var max_brake_force: float = 1500.0  # N per wheel
@export var brake_bias_front: float = 0.6
@export var brake_abs_enabled: bool = true
@export var parking_brake_force: float = 500.0

@export_group("Suspension & Tires")
@export var suspension_stiffness: float = 45000.0  # N/m
@export var suspension_damping: float = 3500.0  # Ns/m
@export var suspension_travel: float = 0.15  # meters
@export var tire_friction_coefficient: float = 1.2
@export var tire_stiffness: float = 250000.0  # N/rad slip

@export_group("Drift & Traction Control")
@export var drift_enabled: bool = true
@export var traction_control_enabled: bool = true
@export var drift_recovery_factor: float = 0.8
@export var oversteer_threshold: float = 0.4
@export var understeer_threshold: float = 0.3

# ============================================================================
# PRIVATE STATE - Internal physics simulation state
# ============================================================================

var _current_speed: float = 0.0  # m/s (forward velocity)
var _current_rpm: float = idle_rpm
var _current_gear: int = 0  # 0 = neutral, 1-6 = gears
var _target_gear: int = 0
var _steering_angle: float = 0.0  # radians
var _throttle_input: float = 0.0  # 0.0 - 1.0
var _brake_input: float = 0.0  # 0.0 - 1.0
var _clutch_input: float = 1.0  # 0.0 = clutch in, 1.0 = clutch engaged
var _handbrake_input: float = 0.0  # 0.0 - 1.0

# Wheel forces and states
var _wheel_forces: Dictionary = {
	"front_left": 0.0,
	"front_right": 0.0,
	"rear_left": 0.0,
	"rear_right": 0.0
}
var _wheel_angles: Dictionary = {
	"front_left": 0.0,
	"front_right": 0.0,
	"rear_left": 0.0,
	"rear_right": 0.0
}
var _wheel_rotations: Dictionary = {
	"front_left": 0.0,
	"front_right": 0.0,
	"rear_left": 0.0,
	"rear_right": 0.0
}

# Slip ratios and angles for tire physics
var _tire_slips: Dictionary = {
	"front_left": 0.0,
	"front_right": 0.0,
	"rear_left": 0.0,
	"rear_right": 0.0
}
var _tire_slip_angles: Dictionary = {
	"front_left": 0.0,
	"front_right": 0.0,
	"rear_left": 0.0,
	"rear_right": 0.0
}

# Aerodynamics and drag
var _aero_downforce: float = 0.0
var _aero_drag: float = 0.0
var _air_density: float = 1.225  # kg/m^3 at sea level

# Drift and traction control state
var _is_drifting: bool = false
var _drift_angle: float = 0.0
var _traction_control_active: bool = false
var _oversteer_factor: float = 0.0
var _understeer_factor: float = 0.0

# Gear shift timing
var _gear_shift_timer: float = 0.0
var _shift_complete: bool = true

# Powertrain connection
var _powertrain_node: Node = null
var _powertrain_connected: bool = false

# Input bindings
var _input_manager: Node = null
var _last_update_time: float = 0.0

# Collision detection
var _collision_velocity: Vector3 = Vector3.ZERO
var _collision_normal: Vector3 = Vector3.UP
var _colliding: bool = false

# ============================================================================
# LIFECYCLE - Initialization and cleanup
# ============================================================================

func _ready() -> void:
	_init_physics_setup()
	_connect_to_powertrain()
	_connect_signals()
	_reset_vehicle_state()

func _process(delta: float) -> void:
	_handle_input(delta)
	_update_physics(delta)
	_update_aerodynamics(delta)
	_update_wheel_states(delta)
	_check_drift_state(delta)
	_apply_traction_control(delta)
	_emit_signals()

func _physics_process(delta: float) -> void:
	apply_central_force(_calculate_total_force())
	apply_torque(_calculate_total_torque())
	apply_gravity(Vector3.DOWN * PhysicsSettings.gravity * vehicle_mass)
	move_and_slide()

# ============================================================================
# PHYSICS SETUP - Initialize vehicle physics components
# ============================================================================

func _init_physics_setup() -> void:
	# Set up physics material
	var physics_material := PhysicsMaterial.new()
	physics_material.static_friction = tire_friction_coefficient
	physics_material.dynamic_friction = tire_friction_coefficient * 0.95
	physics_material.bounce = 0.05
	physics_material.bounce_velocity_threshold = 1.0
	
	for child in get_children():
		if child is RigidBody3D or child is CollisionShape3D:
			child.physics_material_override = physics_material
	
	# Configure mass properties
	var mass_properties := MassProperties.new()
	mass_properties.mass = vehicle_mass
	mass_properties.center_of_mass = center_of_mass_offset
	# Note: Inertia calculation would be more complex for realistic physics
	
	# Reset initial state
	_reset_vehicle_state()

func _reset_vehicle_state() -> void:
	_current_speed = 0.0
	_current_rpm = idle_rpm
	_current_gear = 0
	_target_gear = 0
	_steering_angle = 0.0
	_throttle_input = 0.0
	_brake_input = 0.0
	_clutch_input = 1.0
	_handbrake_input = 0.0
	
	_wheel_forces.clear()
	_wheel_angles.clear()
	_wheel_rotations.clear()
	_tire_slips.clear()
	_tire_slip_angles.clear()
	
	_is_drifting = false
	_drift_angle = 0.0
	_traction_control_active = false
	_oversteer_factor = 0.0
	_understeer_factor = 0.0
	
	_collision_velocity = Vector3.ZERO
	_colliding = false

# ============================================================================
# POWERTRAIN CONNECTION - Link to powertrain system
# ============================================================================

func _connect_to_powertrain() -> void:
	_powertrain_node = get_parent()
	if _powertrain_node and _powertrain_node has_method "get_engine_state":
		_powertrain_connected = true
		_update_powertrain_connection()

func _update_powertrain_connection() -> void:
	if not _powertrain_connected:
		return
		
	var engine_state = _powertrain_node.get_engine_state()
	if engine_state:
		_current_rpm = engine_state.rpm
		_current_torque = engine_state.torque

# ============================================================================
# INPUT HANDLING - Process player input
# ============================================================================

func _handle_input(delta: float) -> void:
	# Get input values from InputManager singleton
	var input_data = InputManager.get_vehicle_inputs()
	
	if input_data.is_empty():
		return
	
	# Update throttle and brake inputs
	_throttle_input = clamp(input_data.throttle, 0.0, 1.0)
	_brake_input = clamp(input_data.brake, 0.0, 1.0)
	_handbrake_input = clamp(input_data.handbrake, 0.0, 1.0)
	
	# Handle steering
	var steering_input = input_data.steer
	_steering_angle = lerp(
		_steering_angle, 
		steering_input * MAX_STEERING_ANGLE, 
		delta * DEFAULT_STEERING_SPEED
	)
	
	# Clamp steering angle
	_steering_angle = clamp(_steering_angle, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE)
	
	# Handle gear shifting
	_handle_gear_shift(input_data.gear_command, delta)
	
	# Calculate wheel steering angles based on Ackermann geometry
	_calculate_ackermann_steering()

func _handle_gear_shift(command: int, delta: float) -> void:
	if command == 0:  # Neutral
		_target_gear = 0
	elif command > 0 and command <= gear_ratios.size():
		_target_gear = command
	elif command < 0 and _current_gear > 0:
		_target_gear = max(1, _current_gear + command)
	
	# Check if shift is needed
	if _target_gear != _current_gear and _shift_complete:
		_start_gear_shift()

func _start_gear_shift() -> void:
	_shift_complete = false
	_gear_shift_timer = 0.0
	
	var old_gear = _current_gear
	_current_gear = _target_gear
	
	gear_changed.emit(old_gear, _current_gear)

func _complete_gear_shift() -> void:
	_shift_complete = true
	_gear_shift_timer = 0.0
	_clutch_input = 1.0

# ============================================================================
# ACKERMANN STEERING - Calculate front wheel steering angles
# ============================================================================

func _calculate_ackermann_steering() -> void:
	var ackermann_radial = wheel_base / tan(abs(_steering_angle)) + track_width / 2.0
	
	# Inner wheel turns more than outer wheel
	var inner_turn_radius = ackermann_radial - track_width / 2.0
	var outer_turn_radius = ackermann_radial + track_width / 2.0
	
	if _steering_angle > 0:  # Steering right
		_wheel_angles["front_left"] = atan(wheel_base / inner_turn_radius)
		_wheel_angles["front_right"] = atan(wheel_base / outer_turn_radius)
	else:  # Steering left
		_wheel_angles["front_left"] = -atan(wheel_base / outer_turn_radius)
		_wheel_angles["front_right"] = -atan(wheel_base / inner_turn_radius)
	
	# Rear wheels don't steer
	_wheel_angles["rear_left"] = 0.0
	_wheel_angles["rear_right"] = 0.0

# ============================================================================
# GEAR SHIFT LOGIC - Manage gear changes and RPM limits
# ============================================================================

func _update_gear_logic(delta: float) -> void:
	if _current_gear == 0:
		return
	
	# Apply gear ratio effect on RPM
	var total_ratio = gear_ratios[_current_gear - 1] * final_drive_ratio
	var wheel_speed = _current_speed / 0.3  # Approximate wheel radius
	
	# Engine RPM calculation
	var target_rpm = wheel_speed * total_ratio
	
	# Apply clutch effect
	if _clutch_input < 1.0:
		target_rpm = lerp(target_rpm, idle_rpm, 1.0 - _clutch_input)
	
	# Smooth RPM transition
	_current_rpm = lerp(_current_rpm, target_rpm, delta * 10.0)
	
	# Auto-shift logic
	if transmission_type == "auto" and _shift_complete:
		_auto_shift(delta)

func _auto_shift(delta: float) -> void:
	if _current_rpm >= engine_max_rpm * shift_up_threshold and _current_gear < gear_ratios.size():
		_target_gear = _current_gear + 1
		_start_gear_shift()
	elif _current_rpm <= engine_max_rpm * shift_down_threshold and _current_gear > 1:
		_target_gear = _current_gear - 1
		_start_gear_shift()

# ============================================================================
# WHEEL FORCE CALCULATION - Calculate forces for each wheel
# ============================================================================

func _calculate_wheel_forces() -> void:
	var drive_wheels = ["rear_left", "rear_right"] if transmission_type == "rwd" else \
	                   ["front_left", "front_right"] if transmission_type == "fwd" else \
	                   ["front_left", "front_right", "rear_left", "rear_right"]
	
	var total_ratio = 1.0
	if _current_gear > 0:
		total_ratio = gear_ratios[_current_gear - 1] * final_drive_ratio
	
	# Calculate engine torque contribution
	var engine_torque = _calculate_engine_torque()
	var drive_torque = engine_torque * total_ratio * 0.85  # Transmission efficiency
	
	# Distribute torque to drive wheels
	for wheel in drive_wheels:
		var torque_factor = _calculate_torque_vectoring(wheel)
		_wheel_forces[wheel] = drive_torque * torque_factor / wheel_base
		
	# Apply braking forces
	for wheel in _wheel_forces.keys():
		var brake_force = _calculate_brake_force(wheel)
		_wheel_forces[wheel] -= brake_force
	
	# Handbrake affects only rear wheels
	if _handbrake_input > 0.0:
		_wheel_forces["rear_left"] -= _handbrake_input * parking_brake_force
		_wheel_forces["rear_right"] -= _handbrake_input * parking_brake_force

func _calculate_engine_torque() -> float:
	if _current_rpm <= idle_rpm:
		return 0.0
	
	var rpm_ratio = (_current_rpm - idle_rpm) / (engine_max_rpm - idle_rpm)
	
	# Simplified torque curve (parabolic shape)
	var base_torque = engine_max_torque * torque_curve_multiplier
	var torque_at_rpm = base_torque * (2.0 * rpm_ratio - pow(rpm_ratio, 2.0))
	
	return max(torque_at_rpm, 0.0)

func _calculate_torque_vectoring(wheel: String) -> float:
	# Simple differential behavior
	if transmission_type == "open_diff":
		return 0.5
	elif transmission_type == "locked_diff":
		return 1.0 if wheel in ["rear_left", "rear_right"] else 0.0
	else:
		# Limited slip differential
		var slip = abs(_tire_slips[wheel])
		if slip < TRACTION_CONTROL_THRESHOLD:
			return 0.5
		else:
			return 0.5 * (1.0 - slip)

func _calculate_brake_force(wheel: String) -> float:
	var brake_force_per_wheel = _brake_input * max_brake_force
	
	# Apply brake bias for front/rear distribution
	if wheel in ["front_left", "front_right"]:
		brake_force_per_wheel *= brake_bias_front
	else:
		brake_force_per_wheel *= (1.0 - brake_bias_front)
	
	return brake_force_per_wheel

# ============================================================================
# AERODYNAMICS - Calculate drag and downforce
# ============================================================================

func _update_aerodynamics(delta: float) -> void:
	# Calculate airspeed magnitude
	var airspeed = _current_speed
	
	# Dynamic pressure: q = 0.5 * rho * v^2
	var dynamic_pressure = 0.5 * _air_density * pow(airspeed, 2.0)
	
	# Drag force: Fd = Cd * A * q
	_aero_drag = aerodynamic_drag_coefficient * frontal_area * dynamic_pressure
	
	# Downforce: Fd = Cl * A * q
	_aero_downforce = downforce_coefficient * frontal_area * dynamic_pressure

func _calculate_aero_force() -> Vector3:
	var aero_force := Vector3.ZERO
	
	# Drag opposes motion
	aero_force.x = -sign(_current_speed) * _aero_drag
	
	# Downforce pushes car into ground
	aero_force.y = -_aero_downforce
	
	return aero_force

# ============================================================================
# TOTAL FORCE AND TORQUE - Aggregate all forces acting on vehicle
# ============================================================================

func _calculate_total_force() -> Vector3:
	var total_force := Vector3.ZERO
	
	# Add aerodynamic forces
	total_force += _calculate_aero_force()
	
	# Add wheel forces (converted to world space)
	var forward_direction = global_transform.basis.z
	var wheel_positions = _get_wheel_world_positions()
	
	for wheel in _wheel_forces.keys():
		var force_magnitude = _wheel_forces[wheel]
		var wheel_pos = wheel_positions[wheel]
		
		# Force direction based on wheel steering angle
		var wheel_direction = Quaternion.IDENTITY.rotated(Vector3.UP, _wheel_angles[wheel])
		force_magnitude *= forward_direction.dot(wheel_direction)
		
		total_force.x += force_magnitude
		
	# Add gravity component (already applied separately, but included for completeness)
	# gravity already handled in _physics_process via apply_gravity
	
	return total_force

func _calculate_total_torque() -> Vector3:
	var total_torque := Vector3.ZERO
	
	# Torque from asymmetric wheel forces
	var wheel_positions = _get_wheel_world_positions()
	var local_forward = transform.basis.z
	
	for wheel in _wheel_forces.keys():
		var force = _wheel_forces[wheel]
		var pos = wheel_positions[wheel]
		
		# Torque = r x F
		var arm = pos - transform.origin
		var torque = arm.cross(Vector3.FORWARD * force)
		
		total_torque += torque
	
	return total_torque

func _get_wheel_world_positions() -> Dictionary:
	var positions := {}
	var half_track = track_width / 2.0
	var half_wheelbase = wheel_base / 2.0
	
	positions["front_left"] = Vector3(-half_track, 0.0, half_wheelbase)
	positions["front_right"] = Vector3(half_track, 0.0, half_wheelbase)
	positions["rear_left"] = Vector3(-half_track, 0.0, -half_wheelbase)
	positions["rear_right"] = Vector3(half_track, 0.0, -half_wheelbase)
	
	# Transform to world space
	for key in positions:
		positions[key] = transform * positions[key]
	
	return positions

# ============================================================================
# TIRE SLIP CALCULATION - Calculate tire slip ratios and angles
# ============================================================================

func _update_tire_slips() -> void:
	var wheel_radii = 0.3  # Approximate wheel radius in meters
	
	for wheel in _tire_slips.keys():
		var wheel_linear_velocity = _wheel_forces[wheel] / vehicle_mass * 0.1
		
		# Tire slip ratio = (v_wheel - v_car) / v_car
		if abs(_current_speed) > 0.1:
			_tire_slips[wheel] = (wheel_linear_velocity - _current_speed) / abs(_current_speed)
		else:
			_tire_slips[wheel] = 0.0
		
		# Tire slip angle based on lateral velocity
		var lateral_velocity = velocity.y
		if abs(_current_speed) > 0.1:
			_tire_slip_angles[wheel] = atan(lateral_velocity / abs(_current_speed))
		else:
			_tire_slip_angles[wheel] = 0.0
	
	tire_slip_changed.emit(_tire_slips["front_left"], _tire_slips["rear_left"])

# ============================================================================
# DRIFT DETECTION - Identify when vehicle is drifting
# ============================================================================

func _check_drift_state(delta: float) -> void:
	if not drift_enabled:
		_is_drifting = false
		return
	
	# Calculate sideslip angle
	var velocity_vector = velocity
	var heading_vector = transform.basis.z
	
	var sideslip_angle = atan2(velocity_vector.x, velocity_vector.z)
	_drift_angle = abs(sideslip_angle)
	
	# Detect drift entry
	if _drift_angle > DRIFT_THRESHOLD and _current_speed > MIN_DRIFT_SPEED:
		if not _is_drifting:
			_is_drifting = true
			is_drifting = true
			drift_started.emit()
	
	# Detect drift exit
	if _drift_angle < DRIFT_THRESHOLD or _current_speed < MIN_DRIFT_SPEED:
		if _is_drifting:
			_is_drifting = false
			drift_ended.emit()

func _apply_drift_effects() -> void:
	if not _is_drifting:
		return
	
	# Reduce effective friction during drift
	var drift_factor = 1.0 - (_drift_angle / PI)
	
	# Apply recovery force
	if _drift_angle > oversteer_threshold:
		var recovery_torque = _drift_angle * drift_recovery_factor
		apply_torque(Vector3.UP * recovery_torque)

# ============================================================================
# TRACTION CONTROL - Prevent wheel spin and loss of control
# ============================================================================

func _apply_traction_control(delta: float) -> void:
	if not traction_control_enabled:
		return
	
	# Check for excessive wheel slip
	var max_slip = 0.0
	for wheel in _tire_slips.values():
		max_slip = max(max_slip, abs(wheel))
	
	if max_slip > TRACTION_CONTROL_THRESHOLD:
		_traction_control_active = true
		
		# Reduce engine torque
		var reduction_factor = (max_slip - TRACTION_CONTROL_THRESHOLD) / 0.5
		reduction_factor = clamp(reduction_factor, 0.0, 1.0)
		
		# Apply reduction to drive wheels
		var drive_wheels = ["rear_left", "rear_right"] if transmission_type == "rwd" else \
		                   ["front_left", "front_right"] if transmission_type == "fwd" else \
		                   ["front_left", "front_right", "rear_left", "rear_right"]
		
		for wheel in drive_wheels:
			_wheel_forces[wheel] *= (1.0 - reduction_factor * 0.5)
	else:
		_traction_control_active = false
	
	traction_control_active.emit(_traction_control_active)

# ============================================================================
# UPDATE PHASES - Main physics update cycle
# ============================================================================

func _update_physics(delta: float) -> void:
	_update_gear_logic(delta)
	_calculate_wheel_forces()
	_update_tire_slips()
	_apply_drift_effects()
	_apply_traction_control(delta)

func _update_wheel_states(delta: float) -> void:
	# Update wheel rotation based on vehicle movement
	var wheel_circumference = 2.0 * PI * 0.3  # Assuming 0.3m radius
	
	for wheel in _wheel_rotations.keys():
		var wheel_force = _wheel_forces[wheel]
		var force_to_rotation = wheel_force / vehicle_mass * delta * 10.0
		
		_wheel_rotations[wheel] += force_to_rotation
		_wheel_rotations[wheel] = fmod(_wheel_rotations[wheel], TAU)

func _emit_signals() -> void:
	# Emit state change signals
	speed_changed.emit(_current_speed)
	rpm_changed.emit(_current_rpm)
	engine_revving.emit(_current_rpm / engine_max_rpm)

# ============================================================================
# PUBLIC API - External interface for other systems
# ============================================================================

func get_vehicle_speed() -> float:
	return _current_speed

func get_vehicle_rpm() -> float:
	return _current_rpm

func get_current_gear() -> int:
	return _current_gear

func get_steering_angle() -> float:
	return _steering_angle

func is_drifting() -> bool:
	return _is_drifting

func is_colliding() -> bool:
	return _colliding

func get_collision_velocity() -> Vector3:
	return _collision_velocity

func set_throttle(value: float) -> void:
	_throttle_input = clamp(value, 0.0, 1.0)

func set_brake(value: float) -> void:
	_brake_input = clamp(value, 0.0, 1.0)

func set_handbrake(value: float) -> void:
	_handbrake_input = clamp(value, 0.0, 1.0)

func request_gear(gear: int) -> void:
	if gear >= 0 and gear <= gear_ratios.size():
		_target_gear = gear

func reset_vehicle() -> void:
	_reset_vehicle_state()
	global_position = Vector3.ZERO
	global_rotation = Vector3.ZERO
	velocity = Vector3.ZERO

func apply_collision(velocity: Vector3, normal: Vector3) -> void:
	_collision_velocity = velocity
	_collision_normal = normal
	_colliding = true
	
	collision_detected.emit(velocity)
	
	# Apply impact damping
	velocity = velocity.linear_interpolate(Vector3.ZERO, 0.5)
	set_velocity(velocity)

func _on_collision_entered(body: Node) -> void:
	pass  # Can be overridden for custom collision handling

func _on_body_entered(body: Node) -> void:
	pass  # Can be overridden for custom body interaction

# ============================================================================
# UTILITY FUNCTIONS - Helper methods
# ============================================================================

func calculate_cornering_force(slip_angle: float) -> float:
	# Linear cornering stiffness model
	return -tire_stiffness * slip_angle

func calculate_longitudinal_force(slip_ratio: float) -> float:
	# Simplified Pacejka magic formula approximation
	if abs(slip_ratio) < 0.1:
		return tire_friction_coefficient * vehicle_mass * slip_ratio
	else:
		return tire_friction_coefficient * vehicle_mass * sign(slip_ratio) * 0.9

func get_wheel_load(wheel: String) -> float:
	# Static weight distribution approximation
	var static_weight = vehicle_mass * 9.81 / 4.0
	var load_transfer = _aero_downforce / 4.0
	return static_weight + load_transfer

func _set_gravity(new_value: float) -> void:
	gravity = new_value
	PhysicsSettings.gravity = new_value

func _set_physics_tick_rate(new_value: int) -> void:
	physics_tick_rate = new_value
	PhysicsSettings.physics_tick_rate = new_value

func _set_max_substeps(new_value: int) -> void:
	max_substeps = new_value
	PhysicsSettings.max_substeps = new_value

func _set_time_scale(new_value: float) -> void:
	time_scale = new_value
	PhysicsSettings.time_scale = new_value

func _set_default_vehicle_mass(new_value: float) -> void:
	default_vehicle_mass = new_value
	PhysicsSettings.default_vehicle_mass = new_value

func _set_default_wheel_base(new_value: float) -> void:
	default_wheel_base = new_value
	PhysicsSettings.default_wheel_base = new_value

func _set_default_track_width(new_value: float) -> void:
	default_track_width = new_value
	PhysicsSettings.default_track_width = new_value

func _set_default_suspension_stiffness(new_value: float) -> void:
	default_suspension_stiffness = new_value
	PhysicsSettings.default_suspension_stiffness = new_value

func _set_default_tire_friction(new_value: float) -> void:
	default_tire_friction = new_value
	PhysicsSettings.default_tire_friction = new_value

func _set_default_aero_drag(new_value: float) -> void:
	default_aero_drag = new_value
	PhysicsSettings.default_aero_drag = new_value

func _set_default_aero_downforce(new_value: float) -> void:
	default_aero_downforce = new_value
	PhysicsSettings.default_aero_downforce = new_value

func _set_default_brake_force(new_value: float) -> void:
	default_brake_force = new_value
	PhysicsSettings.default_brake_force = new_value

func _set_default_engine_torque(new_value: float) -> void:
	default_engine_torque = new_value
	PhysicsSettings.default_engine_torque = new_value

func _set_default_engine_rpm(new_value: float) -> void:
	default_engine_rpm = new_value
	PhysicsSettings.default_engine_rpm = new_value

func _set_default_transmission_ratio(new_value: float) -> void:
	default_transmission_ratio = new_value
	PhysicsSettings.default_transmission_ratio = new_value

func _set_default_final_drive(new_value: float) -> void:
	default_final_drive = new_value
	PhysicsSettings.default_final_drive = new_value

func _set_default_tire_stiffness(new_value: float) -> void:
	default_tire_stiffness = new_value
	PhysicsSettings.default_tire_stiffness = new_value

func _set_default_suspension_travel(new_value: float) -> void:
	default_suspension_travel = new_value
	PhysicsSettings.default_suspension_travel = new_value

func _set_default_suspension_damping(new_value: float) -> void:
	default_suspension_damping = new_value
	PhysicsSettings.default_suspension_damping = new_value

func _set_default_center_of_mass(new_value: Vector3) -> void:
	default_center_of_mass = new_value
	PhysicsSettings.default_center_of_mass = new_value

func _set_default_vehicle_mass(new_value: float) -> void:
	default_vehicle_mass = new_value
	PhysicsSettings.default_vehicle_mass = new_value

func _set_default_wheel_base(new_value: float) -> void:
	default_wheel_base = new_value
	PhysicsSettings.default_wheel_base = new_value

func _set_default_track_width(new_value: float) -> void:
	default_track_width = new_value
	PhysicsSettings.default_track_width = new_value

func _set_default_suspension_stiffness(new_value: float) -> void:
	default_suspension_stiffness = new_value
	PhysicsSettings.default_suspension_stiffness = new_value

func _set_default_tire_friction(new_value: float) -> void:
	default_tire_friction = new_value
	PhysicsSettings.default_tire_friction = new_value

func _set_default_aero_drag(new_value: float) -> void:
	default_aero_drag = new_value
	PhysicsSettings.default_aero_drag = new_value

func _set_default_aero_downforce(new_value: float) -> void:
	default_aero_downforce = new_value
	PhysicsSettings.default_aero_downforce = new_value

func _set_default_brake_force(new_value: float) -> void:
	default_brake_force = new_value
	PhysicsSettings.default_brake_force = new_value

func _set_default_engine_torque(new_value: float) -> void:
	default_engine_torque = new_value
	PhysicsSettings.default_engine_torque = new_value

func _set_default_engine_rpm(new_value: float) -> void:
	default_engine_rpm = new_value
	PhysicsSettings.default_engine_rpm = new_value

func _set_default_transmission_ratio(new_value: float) -> void:
	default_transmission_ratio = new_value
	PhysicsSettings.default_transmission_ratio = new_value

func _set_default_final_drive(new_value: float) -> void:
	default_final_drive = new_value
	PhysicsSettings.default_final_drive = new_value

func _set_default_tire_stiffness(new_value: float) -> void:
	default_tire_stiffness = new_value
	PhysicsSettings.default_tire_stiffness = new_value

func _set_default_suspension_travel(new_value: float) -> void:
	default_suspension_travel = new_value
	PhysicsSettings.default_suspension_travel = new_value

func _set_default_suspension_damping(new_value: float) -> void:
	default_suspension_damping = new_value
	PhysicsSettings.default_suspension_damping = new_value

func _set_default_center_of_mass(new_value: Vector3) -> void:
	default_center_of_mass = new_value
	PhysicsSettings.default_center_of_mass = new_value

func _connect_signals() -> void:
	# Connect to GameManager for race events
	GameManager.race_started.connect(_on_race_started)
	GameManager.race_paused.connect(_on_race_paused)
	GameManager.race_resumed.connect(_on_race_resumed)
	GameManager.race_finished.connect(_on_race_finished)

func _on_race_started(data: Dictionary) -> void:
	# Prepare vehicle for race
	_reset_vehicle_state()
	_current_gear = 1  # Start in first gear
	_current_rpm = idle_rpm

func _on_race_paused() -> void:
	# Pause physics updates
	_process_mode = ProcessModeEnum.INHERITED

func _on_race_resumed() -> void:
	# Resume physics updates
	_process_mode = ProcessModeEnum.ALWAYS

func _on_race_finished(results: Dictionary) -> void:
	# Clean up after race
	_reset_vehicle_state()
	_current_gear = 0