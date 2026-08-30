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
@export var idle_rpm: float = 800.0
@export var torque_curve: Array[float] = [0.0, 0.3, 0.6, 0.9, 1.0, 0.95, 0.9, 0.85, 0.8, 0.75]
@export var transmission_gears: Array[int> = [3.5, 2.5, 1.8, 1.3, 0.9, 0.7]
@export var final_drive_ratio: float = 4.1
@export var clutch_friction: float = 0.35
@export var flywheel_inertia: float = 0.08

@export_group("Brake System")
@export var brake_pressure_threshold: float = 0.2
@export var max_brake_force: float = 12000.0
@export var brake_bias_front: float = 0.6
@export var abs_enabled: bool = true
@export var brake_temperature_max: float = 1000.0
@export var brake_decay_rate: float = 5.0

@export_group("Drift & Traction Control")
@export var drift_threshold: float = 0.4
@export var traction_control_enabled: bool = true
@export var stability_control_enabled: bool = true
@export var oversteer_factor: float = 1.1
@export var understeer_factor: float = 0.9
@export var tire_friction_multiplier: float = 1.0

@export_group("Wheel Configuration")
@export var wheel_radius: float = 0.33
@export var wheel_tire_width: float = 0.25
@export var wheel_tire_compliance: float = 0.001
@export var wheel_suspension_travel: float = 0.15
@export var wheel_spring_constant: float = 45000.0
@export var wheel_damping_constant: float = 3500.0
@export var wheel_steering_limit: float = 30.0 * PI / 180.0

@export_group("Tire Properties")
@export var tire_friction_coefficient: float = 1.1
@export var tire_side_slip_coefficient: float = 0.85
@export var tire_longitudinal_slip_coefficient: float = 1.0
@export var tire_normal_load_exponent: float = 0.5
@export var tire_friction_peak: float = 1.2
@export var tire_friction_asymptote: float = 0.8

# ============================================================================
# PRIVATE STATE VARIABLES
# ============================================================================

# Powertrain state
var _current_rpm: float = 800.0
var _target_rpm: float = 800.0
var _engine_torque: float = 0.0
var _clutch_engaged: bool = false
var _clutch_position: float = 1.0
var _flywheel_velocity: float = 0.0

# Transmission state
var _current_gear: int = 0
var _gearbox_input: int = 0
var _shift_progress: float = 0.0
var _shift_target_gear: int = 0
var _shift_timer: float = 0.0

# Wheel states (front-left, front-right, rear-left, rear-right)
var _wheel_states: Array[Dictionary] = []
var _wheel_contact_points: Array[Vector3] = []
var _wheel_vertical_forces: Array[float] = []
var _wheel_longitudinal_velocities: Array[float] = []
var _wheel_lateral_velocities: Array[float] = []

# Suspension states per wheel
var _suspension_travels: Array[float] = []
var _suspension_velocities: Array[float] = []

# Brake system state
var _brake_temperatures: Array[float] = []
var _brake_pressures: Array[float] = []
var _brake_lock_flags: Array[bool] = []

# Vehicle dynamics state
var _velocity_local: Vector3 = Vector3.ZERO
var _angular_velocity_local: Vector3 = Vector3.ZERO
var _acceleration_vector: Vector3 = Vector3.ZERO
var _slip_angle: float = 0.0
var _drift_state: int = 0  # 0=none, 1=starting, 2=active, 3=ending
var _drift_intensity: float = 0.0

# Input values
var _throttle_input: float = 0.0
var _brake_input: float = 0.0
var _clutch_input: float = 0.0
var _steering_input: float = 0.0
var _gear_input: int = 0

# Simulation state
var _speed_kmh: float = 0.0
var _speed_mps: float = 0.0
var _distance_traveled: float = 0.0
var _last_update_time: float = 0.0
var _physics_delta: float = 0.0

# Tire temperature tracking
var _tire_temperatures: Array[float] = []

# Collision detection
var _collision_active: bool = false
var _last_collision_time: float = 0.0
var _collision_force_accumulator: float = 0.0

# Reference points
var _ground_plane_normal: Vector3 = Vector3.UP
var _ground_plane_distance: float = 0.0

# ============================================================================
# WHEEL SETUP CONSTANTS
# ============================================================================

enum WheelIndex {
	FrontLeft,
	FrontRight,
	RearLeft,
	RearRight,
	WheelCount
}

func _init_wheel_states() -> void:
	"""Initialize wheel state arrays"""
	_wheel_states.resize(WheelIndex.WheelCount)
	_wheel_contact_points.resize(WheelIndex.WheelCount)
	_wheel_vertical_forces.resize(WheelIndex.WheelCount)
	_wheel_longitudinal_velocities.resize(WheelIndex.WheelCount)
	_wheel_lateral_velocities.resize(WheelIndex.WheelCount)
	_suspension_travels.resize(WheelIndex.WheelCount)
	_suspension_velocities.resize(WheelIndex.WheelCount)
	_brake_temperatures.resize(WheelIndex.WheelCount)
	_brake_pressures.resize(WheelIndex.WheelCount)
	_brake_lock_flags.resize(WheelIndex.WheelCount)
	_tire_temperatures.resize(WheelIndex.WheelCount)
	
	for i in WheelIndex.WheelCount:
		_wheel_states[i] = {
			"radius": wheel_radius,
			"width": wheel_tire_width,
			"compliance": wheel_tire_compliance,
			"spring": wheel_spring_constant,
			"damping": wheel_damping_constant,
			"travel": 0.0,
			"velocity": 0.0,
			"vertical_force": 0.0,
			"longitudinal_force": 0.0,
			"lateral_force": 0.0,
			"slip_ratio": 0.0,
			"slip_angle": 0.0,
			"friction_circle_usage": 0.0,
			"temperature": 20.0,
			"wear": 0.0
		}
		
		_suspension_travels[i] = 0.0
		_suspension_velocities[i] = 0.0
		_brake_temperatures[i] = 20.0
		_brake_pressures[i] = 0.0
		_brake_lock_flags[i] = false
		_tire_temperatures[i] = 20.0

func _setup_wheel_positions() -> void:
	"""Setup initial wheel contact point positions relative to car body"""
	var half_track: float = track_width / 2.0
	var half_wheelbase: float = wheel_base / 2.0
	var wheel_offset_x: float = -half_wheelbase
	var rear_wheel_offset_x: float = half_wheelbase
	
	# Front wheels
	_wheel_contact_points[WheelIndex.FrontLeft] = Vector3(wheel_offset_x, -ground_clearance, half_track)
	_wheel_contact_points[WheelIndex.FrontRight] = Vector3(wheel_offset_x, -ground_clearance, -half_track)
	
	# Rear wheels
	_wheel_contact_points[WheelIndex.RearLeft] = Vector3(rear_wheel_offset_x, -ground_clearance, half_track)
	_wheel_contact_points[WheelIndex.RearRight] = Vector3(rear_wheel_offset_x, -ground_clearance, -half_track)

# ============================================================================
# INITIALIZATION
# ============================================================================

func _ready() -> void:
	_setup_wheel_states()
	_setup_wheel_positions()
	_current_gear = 1
	_clutch_engaged = true
	_last_update_time = Time.get_ticks_usec()
	
	# Register with GameManager
	if GameManager.has_signal("vehicle_spawned"):
		GameManager.vehicle_spawned.emit(self)

func _process(delta: float) -> void:
	"""Process input and high-level logic"""
	_physics_delta = delta
	
	_process_input()
	_update_powertrain()
	_update_transmission()
	_update_driving_dynamics()
	_update_braking_system()
	_update_suspension()
	_update_vehicle_motion()
	_update_tires()
	_update_drift_system()
	_update_collision_detection()
	
	emit_signals()

func _physics_process(delta: float) -> void:
	"""Fixed timestep physics updates"""
	pass

# ============================================================================
# INPUT PROCESSING
# ============================================================================

func _process_input() -> void:
	"""Process all vehicle control inputs"""
	_throttle_input = clamp(InputManager.get_axis("throttle"), 0.0, 1.0)
	_brake_input = clamp(InputManager.get_axis("brake"), 0.0, 1.0)
	_clutch_input = clamp(InputManager.get_axis("clutch"), 0.0, 1.0)
	_steering_input = clamp(InputManager.get_axis("steering"), -1.0, 1.0)
	_gear_input = InputManager.get_action_strength("gear_up") - InputManager.get_action_strength("gear_down")
	
	# Handle manual gear shift
	if _gear_input != 0:
		_shift_progress = 0.0
		_shift_target_gear = _clamp_gear(_current_gear + _gear_input)
		_shift_timer = 0.25  # Shift takes 250ms
	elif _shift_progress > 0.0:
		_shift_progress += _physics_delta / 0.25
		if _shift_progress >= 1.0:
			_current_gear = _shift_target_gear
			_shift_progress = 0.0
			gear_changed.emit(_current_gear)

# ============================================================================
# POWERTRAIN SYSTEM
# ============================================================================

func _update_powertrain() -> void:
	"""Update engine, clutch, and power delivery"""
	# Update clutch position
	_clutch_position = lerp(_clutch_position, _clutch_input, 5.0 * _physics_delta)
	_clutch_engaged = _clutch_position > 0.1
	
	# Calculate target RPM based on gear and throttle
	_target_rpm = _calculate_target_rpm()
	
	# Engine torque curve lookup
	_engine_torque = _lookup_torque_curve(_current_rpm / engine_max_rpm) * 800.0  # Max torque ~800 Nm
	
	# Flywheel dynamics
	var inertia_effective: float = flywheel_inertia * _clutch_position
	var rpm_difference: float = (_target_rpm - _current_rpm)
	var rpm_change: float = rpm_difference * 10.0 * _physics_delta * _clutch_position
	
	_current_rpm = clamp(_current_rpm + rpm_change, engine_min_rpm, engine_max_rpm)
	
	# Idle behavior
	if _throttle_input < 0.1 and _current_gear == 0:
		_current_rpm = lerp(_current_rpm, idle_rpm, 2.0 * _physics_delta)
	
	rpm_changed.emit(_current_rpm)

func _calculate_target_rpm() -> float:
	"""Calculate target RPM based on current speed and gear"""
	if _current_gear == 0:
		return idle_rpm
	
	var speed_ratio: float = _speed_mps / 10.0  # Simplified ratio
	var gear_ratio: float = transmission_gears[_current_gear - 1]
	var wheel_circumference: float = PI * wheel_radius * 2.0
	var wheel_rpm: float = _speed_mps / wheel_circumference * 60.0
	var drivetrain_ratio: float = gear_ratio * final_drive_ratio
	
	var target: float = wheel_rpm * drivetrain_ratio
	target = clamp(target, engine_min_rpm, engine_max_rpm)
	
	return target

func _lookup_torque_curve(rpm_normalized: float) -> float:
	"""Lookup torque from pre-defined curve"""
	var index: int = int(rpm_normalized * (torque_curve.size() - 1))
	index = clamp(index, 0, torque_curve.size() - 2)
	var t: float = (rpm_normalized * (torque_curve.size() - 1)) - index
	return torque_curve[index] * (1.0 - t) + torque_curve[index + 1] * t

# ============================================================================
# TRANSMISSION SYSTEM
# ============================================================================

func _update_transmission() -> void:
	"""Update gearbox and gear selection"""
	if _shift_progress > 0.0:
		# During shift, reduce power temporarily
		_clutch_position = lerp(_clutch_position, 0.0, 10.0 * _physics_delta)
	else:
		# Return clutch to desired position
		_clutch_position = lerp(_clutch_position, _clutch_input, 5.0 * _physics_delta)

func _clamp_gear(gear: int) -> int:
	"""Ensure gear is within valid range"""
	return clampi(gear, 0, transmission_gears.size())

# ============================================================================
# DRIVING DYNAMICS
# ============================================================================

func _update_driving_dynamics() -> void:
	"""Update vehicle driving dynamics including lateral/longitudinal forces"""
	# Calculate wheel velocities in local space
	_velocity_local = global_transform.basis.xform(speed)
	_angular_velocity_local = global_transform.basis.xform(angular_velocity)
	
	# Calculate slip angle at center of gravity
	var direction: Vector3 = _velocity_local.normalized()
	direction.y = 0.0
	direction = direction.normalized()
	_slip_angle = atan2(direction.z, direction.x)
	
	# Apply aerodynamic drag
	var air_density: float = 1.225  # kg/m^3 at sea level
	var drag_force: float = 0.5 * air_density * drag_coefficient * frontal_area * _speed_mps * _speed_mps
	var drag_acceleration: float = -drag_force / vehicle_mass
	_acceleration_vector.x += drag_acceleration
	
	# Apply steering to front wheels
	var front_wheel_steering: float = _steering_input * wheel_steering_limit
	_wheel_states[WheelIndex.FrontLeft]["steering"] = front_wheel_steering
	_wheel_states[WheelIndex.FrontRight]["steering"] = -front_wheel_steering

# ============================================================================
# BRAKING SYSTEM
# ============================================================================

func _update_braking_system() -> void:
	"""Update brake pressure, temperatures, and lock prevention"""
	if _brake_input > brake_pressure_threshold:
		var brake_pressure: float = lerp(_brake_input, brake_pressure_threshold, 5.0 * _physics_delta)
		
		# Apply brake bias
		var front_pressure: float = brake_pressure * brake_bias_front
		var rear_pressure: float = brake_pressure * (1.0 - brake_bias_front)
		
		_brake_pressures[WheelIndex.FrontLeft] = front_pressure
		_brake_pressures[WheelIndex.FrontRight] = front_pressure
		_brake_pressures[WheelIndex.RearLeft] = rear_pressure
		_brake_pressures[WheelIndex.RearRight] = rear_pressure
		
		# Heat generation during braking
		var heat_generation: float = brake_pressure * _speed_mps * 50.0
		for i in WheelIndex.WheelCount:
			_brake_temperatures[i] += heat_generation * _physics_delta
			_brake_temperatures[i] = min(_brake_temperatures[i], brake_temperature_max)
	else:
		# Cool down brakes when not braking
		for i in WheelIndex.WheelCount:
			_brake_temperatures[i] -= brake_decay_rate * _physics_delta
			_brake_temperatures[i] = max(_brake_temperatures[i], 20.0)
		
		# Zero out pressures
		for i in WheelIndex.WheelCount:
			_brake_pressures[i] = lerp(_brake_pressures[i], 0.0, 10.0 * _physics_delta)
	
	# ABS check (simplified)
	if abs_enabled:
		for i in WheelIndex.WheelCount:
			var slip_ratio: float = abs(_wheel_states[i]["slip_ratio"])
			if slip_ratio > 0.3 and _brake_pressures[i] > 0.1:
				_brake_lock_flags[i] = true
				_brake_pressures[i] *= 0.9  # Reduce pressure
			else:
				_brake_lock_flags[i] = false

# ============================================================================
# SUSPENSION SYSTEM
# ============================================================================

func _update_suspension() -> void:
	"""Update suspension travel and forces for each wheel"""
	for i in WheelIndex.WheelCount:
		var wheel_state: Dictionary = _wheel_states[i]
		var suspension_travel: float = _suspension_travels[i]
		var suspension_velocity: float = _suspension_velocities[i]
		
		# Spring force
		var spring_force: float = -wheel_state["spring"] * suspension_travel
		
		# Damping force
		var damping_force: float = -wheel_state["damping"] * suspension_velocity
		
		# Total vertical force
		var total_force: float = spring_force + damping_force
		
		# Apply to wheel vertical velocity
		var mass_per_wheel: float = vehicle_mass / WheelIndex.WheelCount
		var acceleration: float = total_force / mass_per_wheel
		suspension_velocity += acceleration * _physics_delta
		suspension_travel += suspension_velocity * _physics_delta
		
		# Clamp suspension travel
		suspension_travel = clamp(suspension_travel, -wheel_suspension_travel, wheel_suspension_travel)
		
		# Update states
		_suspension_travels[i] = suspension_travel
		_suspension_velocities[i] = suspension_velocity
		wheel_state["travel"] = suspension_travel

# ============================================================================
# VEHICLE MOTION
# ============================================================================

func _update_vehicle_motion() -> void:
	"""Update overall vehicle motion based on wheel forces"""
	# Sum longitudinal forces from all wheels
	var total_longitudinal_force: float = 0.0
	for i in WheelIndex.WheelCount:
		total_longitudinal_force += _wheel_states[i]["longitudinal_force"]
	
	# Acceleration from wheel forces
	var acceleration_from_wheels: float = total_longitudinal_force / vehicle_mass
	_acceleration_vector.x += acceleration_from_wheels
	
	# Update velocity
	_velocity_local.x += _acceleration_vector.x * _physics_delta
	
	# Convert back to world space
	speed = global_transform.basis.xform(_velocity_local)
	move_and_slide()
	
	# Track distance
	var distance_this_frame: float = _velocity_local.length() * _physics_delta
	_distance_traveled += distance_this_frame
	
	# Update speed metrics
	_speed_mps = _velocity_local.length()
	_speed_kmh = _speed_mps * 3.6
	
	speed_changed.emit(_speed_kmh)

# ============================================================================
# TIRE MODEL
# ============================================================================

func _update_tires() -> void:
	"""Update tire forces using simplified Pacejka model"""
	for i in WheelIndex.WheelCount:
		var wheel_state: Dictionary = _wheel_states[i]
		var wheel_pos: Vector3 = _wheel_contact_points[i]
		
		# Get world position of wheel
		var wheel_world_pos: Vector3 = global_transform * wheel_pos
		var wheel_velocity: Vector3 = _get_wheel_world_velocity(i)
		
		# Calculate longitudinal slip
		var wheel_rotational_velocity: float = _current_rpm / 60.0 * 2.0 * PI * wheel_state["radius"]
		var forward_velocity: float = wheel_velocity.dot(global_transform.basis.x)
		var slip_ratio: float = (forward_velocity - wheel_rotational_velocity) / max(forward_velocity, 0.1)
		
		wheel_state["slip_ratio"] = slip_ratio
		
		# Calculate lateral slip (slip angle)
		var lateral_velocity: float = wheel_velocity.dot(global_transform.basis.z)
		var slip_angle: float = atan2(lateral_velocity, max(abs(forward_velocity), 0.1))
		wheel_state["slip_angle"] = slip_angle
		
		# Normal load calculation (weight transfer)
		var base_load: float = vehicle_mass * _physics.gravity / WheelIndex.WheelCount
		var load_transfer: float = _calculate_load_transfer(i)
		var normal_load: float = base_load + load_transfer
		normal_load = max(normal_load, 100.0)  # Minimum load to prevent singularity
		
		wheel_state["vertical_force"] = normal_load
		
		# Calculate friction coefficient based on normal load
		var friction_coef: float = tire_friction_coefficient * pow(normal_load / base_load, tire_normal_load_exponent)
		
		# Longitudinal force (simplified)
		var max_longitudinal_force: float = friction_coef * normal_load
		var drive_brake_force: float = 0.0
		
		if _current_gear > 0 and _clutch_engaged:
			# Driving force
			drive_brake_force = _engine_torque * transmission_gears[_current_gear - 1] * final_drive_ratio / wheel_state["radius"]
			if _throttle_input > 0.1:
				drive_brake_force *= _throttle_input
		else:
			# Braking force
			var brake_force: float = _brake_pressures[i] * max_brake_force
			drive_brake_force = -brake_force
		
		# Limit by friction circle
		var lateral_force_cap: float = sqrt(max_longitudinal_force * max_longitudinal_force - drive_brake_force * drive_brake_force)
		if lateral_force_cap < 0.0:
			lateral_force_cap = 0.0
		
		# Lateral force (simplified)
		var lateral_force: float = friction_coef * normal_load * sin(slip_angle)
		lateral_force = clamp(lateral_force, -lateral_force_cap, lateral_force_cap)
		
		wheel_state["longitudinal_force"] = drive_brake_force
		wheel_state["lateral_force"] = lateral_force
		
		# Store velocities
		_wheel_longitudinal_velocities[i] = forward_velocity
		_wheel_lateral_velocities[i] = lateral_velocity
		
		# Emit slip signal
		if abs(slip_ratio) > 0.1:
			wheel_slip.emit(i, slip_ratio)
		
		# Update tire temperature
		var temp_increase: float = (abs(drive_brake_force) + abs(lateral_force)) * 0.01
		_tire_temperatures[i] += temp_increase * _physics_delta
		_tire_temperatures[i] = min(_tire_temperatures[i], 120.0)  # Max tire temp

func _get_wheel_world_velocity(wheel_index: int) -> Vector3:
	"""Get world-space velocity at wheel contact point"""
	var wheel_pos_rel: Vector3 = _wheel_contact_points[wheel_index]
	var wheel_pos_world: Vector3 = global_transform * wheel_pos_rel
	
	var vel_at_point: Vector3 = speed + angular_velocity.cross(wheel_pos_world - global_position)
	return vel_at_point

func _calculate_load_transfer(wheel_index: int) -> float:
	"""Calculate load transfer due to acceleration/turning"""
	var weight_transfer: float = 0.0
	
	# Longitudinal weight transfer (acceleration/braking)
	var accel_weight_transfer: float = (vehicle_mass * _acceleration_vector.x * center_of_mass_offset.y) / wheel_base
	if wheel_index == WheelIndex.FrontLeft or wheel_index == WheelIndex.FrontRight:
		weight_transfer -= accel_weight_transfer / 2.0
	else:
		weight_transfer += accel_weight_transfer / 2.0
	
	# Lateral weight transfer (cornering)
	var lateral_accel: float = _velocity_local.z / max(_speed_mps, 0.1) * _velocity_local.z
	var cornering_weight_transfer: float = (vehicle_mass * lateral_accel * center_of_mass_offset.y) / track_width
	if wheel_index == WheelIndex.FrontLeft or wheel_index == WheelIndex.RearLeft:
		weight_transfer += cornering_weight_transfer / 2.0
	else:
		weight_transfer -= cornering_weight_transfer / 2.0
	
	return weight_transfer

# ============================================================================
# DRIFT SYSTEM
# ============================================================================

func _update_drift_system() -> void:
	"""Manage drift state and intensity"""
	var rear_slip_angle: float = (_wheel_states[WheelIndex.RearLeft]["slip_angle"] + _wheel_states[WheelIndex.RearRight]["slip_angle"]) / 2.0
	var rear_slip_mag: float = abs(rear_slip_angle)
	
	# Detect drift entry
	if _drift_state == 0 and rear_slip_mag > drift_threshold and _speed_kmh > 20.0:
		_drift_state = 1  # Starting
		_drift_intensity = 0.0
		if drift_started.is_connected(drift_started):
		(drift_started.emit())
	
	# Drift active
	elif _drift_state == 1 or _drift_state == 2:
		if rear_slip_mag > drift_threshold:
			_drift_state = 2  # Active
			_drift_intensity = min(_drift_intensity + _physics_delta * 2.0, 1.0)
			
			# Apply drift effects (reduced lateral grip)
			var drift_modifier: float = 1.0 - _drift_intensity * 0.5
			for i in WheelIndex.WheelCount:
				_wheel_states[i]["lateral_force"] *= drift_modifier
				
		else:
			_drift_state = 3  # Ending
			_drift_intensity = max(_drift_intensity - _physics_delta * 3.0, 0.0)
			
			if _drift_intensity <= 0.0:
				_drift_state = 0
				if drift_ended.is_connected(drift_ended):
					(drift_ended.emit())
	
	# Stabilization
	if stability_control_enabled and _drift_state == 2:
		# Reduce engine torque slightly to regain control
		_engine_torque *= 0.95

# ============================================================================
# COLLISION DETECTION
# ============================================================================

func _update_collision_detection() -> void:
	"""Check for collisions and handle impacts"""
	# Simple sphere-based collision detection
	var sensor_radius: float = wheel_radius + ground_clearance + 0.5
	var sensor_center: Vector3 = global_position + Vector3(0.0, sensor_radius, 0.0)
	
	# Check for ground contact
	var ray_result: Dictionary = get_world_3d().direct_space_state.intersect_ray(
		sensor_center,
		sensor_center + Vector3.DOWN * 10.0,
		[self]
	)
	
	if ray_result and ray_result.position:
		_ground_plane_normal = ray_result.normal
		_ground_plane_distance = ray_result.position.y - sensor_center.y
		
		# Collision with objects
		var collision_list: Array = get_overlapping_bodies()
		for other in collision_list:
			if other != self and other != get_parent():
				var impact_velocity: float = _velocity_local.length()
				if impact_velocity > 5.0:
					_collision_active = true
					_collision_force_accumulator += impact_velocity * vehicle_mass * 0.1
					
					if _collision_force_accumulator > 5000.0:
						collision_impact.emit(_collision_force_accumulator, global_position)
						_collision_force_accumulator = 0.0
						_collision_active = false
						_last_collision_time = Time.get_ticks_msec()
						
						# Apply crash effect
						_apply_crash_effects()
				
				break

func _apply_crash_effects() -> void:
	"""Apply visual/audio effects for collision"""
	# Screen shake
	get_viewport().camera_3d.shake_camera(0.5, 0.2)
	
	# Audio feedback
	if AudioManager.has_signal("sound_played"):
		AudioManager.sound_played.emit("crash")
	
	# Particle burst
	_create_collision_particles(global_position)

func _create_collision_particles(position: Vector3) -> void:
	"""Create particle burst at collision point"""
	pass  # Implementation depends on particle system availability

# ============================================================================
# SIGNALS
# ============================================================================

func emit_signals() -> void:
	"""Emit relevant signals for external systems"""
	# Speed signal already emitted in _update_vehicle_motion
	# RPM signal already emitted in _update_powertrain
	# Gear signal already emitted on shift completion
	# Wheel slip signals emitted in _update_tires
	# Collision signals emitted in _update_collision_detection

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================

func get_vehicle_speed_kmh() -> float:
	"""Get current vehicle speed in km/h"""
	return _speed_kmh

func get_vehicle_speed_mps() -> float:
	"""Get current vehicle speed in m/s"""
	return _speed_mps

func get_current_rpm() -> float:
	"""Get current engine RPM"""
	return _current_rpm

func get_current_gear() -> int:
	"""Get current gear number"""
	return _current_gear

func get_total_distance() -> float:
	"""Get total distance traveled in meters"""
	return _distance_traveled

func reset_vehicle() -> void:
	"""Reset vehicle to starting conditions"""
	_current_rpm = idle_rpm
	_current_gear = 1
	_speed = Vector3.ZERO
	_angular_velocity = Vector3.ZERO
	_acceleration_vector = Vector3.ZERO
	_distance_traveled = 0.0
	_drift_state = 0
	_drift_intensity = 0.0
	_collision_active = false
	_collision_force_accumulator = 0.0
	
	for i in WheelIndex.WheelCount:
		_suspension_travels[i] = 0.0
		_suspension_velocities[i] = 0.0
		_brake_temperatures[i] = 20.0
		_brake_pressures[i] = 0.0
		_tire_temperatures[i] = 20.0

func set_input_override(throttle: float, brake: float, steering: float, gear: int) -> void:
	"""Override inputs for AI/driving assistance"""
	_throttle_input = clamp(throttle, 0.0, 1.0)
	_brake_input = clamp(brake, 0.0, 1.0)
	_steering_input = clamp(steering, -1.0, 1.0)
	if gear >= 0 and gear < transmission_gears.size() + 1:
		_current_gear = gear

func debug_get_wheel_data(wheel_index: int) -> Dictionary:
	"""Debug helper to retrieve wheel state data"""
	if wheel_index >= 0 and wheel_index < WheelIndex.WheelCount:
		return _wheel_states[wheel_index].duplicate()
	return {}

func _set_gravity(value: float) -> void:
	gravity = value
	PhysicsSettings.gravity = value

func _set_physics_tick_rate(value: int) -> void:
	physics_tick_rate = value
	PhysicsSettings.physics_tick_rate = value

func _set_max_substeps(value: int) -> void:
	max_substeps = value
	PhysicsSettings.max_substeps = value

func _set_time_scale(value: float) -> void:
	time_scale = value
	PhysicsSettings.time_scale = value

func _set_default_vehicle_mass(value: float) -> void:
	default_vehicle_mass = value
	vehicle_mass = value

func _set_default_wheel_config(value: Dictionary) -> void:
	default_wheel_config = value
