extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulation
## Handles throttle, brake, steering, wheel forces, gear shifting, and vehicle dynamics
## Integrates with PhysicsSettings singleton for all physics constants
## Copyright 2026 Thalamus Racing Simulator Project

# ============================================================================
# SIGNALS
# ============================================================================

signal speed_changed(new_speed: float)
signal rpm_changed(new_rpm: float)
signal gear_changed(old_gear: int, new_gear: int)
signal drift_started(drift_angle: float)
signal drift_ended(drift_angle: float)
signal collision_impact(impact_force: Vector3, impact_point: Vector3)
signal wheel_slip(wheel_index: int, slip_ratio: float)
signal engine_sound_changed(rpm_percentage: float)

# ============================================================================
# PHYSICS SETTINGS REFERENCE
# ============================================================================

var physics: PhysicsSettings = null

# ============================================================================
# VEHICLE CONFIGURATION (EXPORTED FOR TUNING)
# ============================================================================

@export_group("Vehicle Configuration")
@export var vehicle_mass: float = 1500.0: set = _set_vehicle_mass
@export var center_of_mass_offset: Vector3 = Vector3(0, 0.3, 0): set = _set_center_of_mass
@export var track_width: float = 1.6: set = _set_track_width
@export var wheelbase: float = 2.5: set = _set_wheelbase
@export var ride_height: float = 0.3: set = _set_ride_height
@export var drag_coefficient: float = 0.32: set = _set_drag_coeff
@export var frontal_area: float = 2.2: set = _set_frontal_area
@export var downforce_coefficient: float = 0.5: set = _set_downforce_coeff
@export var max_lift: float = 0.1: set = _set_max_lift

@export_group("Suspension Geometry")
@export var suspension_travel: float = 0.15: set = _set_suspension_travel
@export var spring_stiffness: float = 45000.0: set = _set_spring_stiffness
@export var damping_rate: float = 5000.0: set = _set_damping_rate
@export var anti_roll_bar_stiffness: float = 15000.0: set = _set_anti_roll
@export var camber_gain: float = -0.02: set = _set_camber_gain
@export var toe_in: float = 0.005: set = _set_toe_in

@export_group("Powertrain Parameters")
@export var engine_max_rpm: float = 8500.0: set = _set_engine_max_rpm
@export var engine_idle_rpm: float = 850.0: set = _set_engine_idle_rpm
@export var engine_peak_torque: float = 450.0: set = _set_peak_torque
@export var engine_peak_power: float = 350.0: set = _set_peak_power
@export var torque_curve_flatness: float = 0.7: set = _set_torque_flatness
@export var final_drive_ratio: float = 3.73: set = _set_final_drive
@export var clutch_friction: float = 0.4: set = _set_clutch_friction
@export var transmission_efficiency: float = 0.92: set = _set_trans_eff

@export_group("Gear Ratios [Neutral, Reverse, 1st-6th]")
@export var gear_ratios: Array[float] = [0.0, -3.5, 3.8, 2.5, 1.8, 1.4, 1.1, 0.9]: set = _set_gear_ratios
@export var neutral_position: int = 0
@export var reverse_position: int = 1
@export var first_gear_position: int = 2
@export var sixth_gear_position: int = 7

@export_group("Braking System")
@export var front_brake_bias: float = 0.55: set = _set_brake_bias
@export var max_brake_pressure: float = 120.0: set = _set_max_brake_press
@export var brake_disc_radius: float = 0.15: set = _set_brake_disc_radius
@export var brake_caliper_piston_area: float = 0.008: set = _set_caliper_area
@export var brake_pad_friction: float = 0.4: set = _set_brake_pad_frict

@export_group("Tire Properties")
@export var tire_stiffness: float = 180000.0: set = _set_tire_stiffness
@export var tire_friction_coefficient: float = 1.2: set = _set_tire_friction
@export var tire_width: float = 0.28: set = _set_tire_width
@export var tire_radius: float = 0.32: set = _set_tire_radius
@export var tire_vertical_compliance: float = 0.001: set = _set_tire_compliance

@export_group("Drift & Handling")
@export var drift_threshold: float = 0.35: set = _set_drift_thresh
@export var drift_recovery: float = 0.95: set = _set_drift_recovery
@export var understeer_coefficient: float = 0.15: set = _set_understeer
@export var oversteer_coefficient: float = 0.12: set = _set_oversteer

# ============================================================================
# STATE VARIABLES
# ============================================================================

# Current driving state
var current_gear: int = 0
var current_rpm: float = 0.0
var target_rpm: float = 0.0
var current_speed: float = 0.0  # km/h
var current_velocity_mps: float = 0.0  # m/s
var throttle_input: float = 0.0  # 0.0 to 1.0
var brake_input: float = 0.0    # 0.0 to 1.0
var steering_input: float = 0.0 # -1.0 to 1.0
var clutch_input: float = 0.0   # 0.0 to 1.0
var handbrake_input: float = 0.0 # 0.0 to 1.0

# Derived values
var wheel_torque: float = 0.0
var braking_force: float = 0.0
var steering_angle: float = 0.0
var slip_ratio: float = 0.0
var lateral_acceleration: float = 0.0
var longitudinal_acceleration: float = 0.0
var yaw_rate: float = 0.0

# Drift state
var drift_angle: float = 0.0
var drift_intensity: float = 0.0
var drift_active: bool = false

# Suspension state
var suspension_compression: Vector4 = Vector4(0.0, 0.0, 0.0, 0.0)
var suspension_velocity: Vector4 = Vector4(0.0, 0.0, 0.0, 0.0)

# Wheel positions (relative to vehicle center)
var wheel_positions: Array[Vector3] = []
var wheel_contact_points: Array[Vector3] = []
var wheel_forces: Array[Vector3] = []

# Engine power curve cache
var _torque_curve_cache: Dictionary = {}

# ============================================================================
# POWERTRAIN REFERENCE
# ============================================================================

var powertrain: Node = null

# ============================================================================
# WHEEL REFERENCES
# ============================================================================

var front_left_wheel: Node = null
var front_right_wheel: Node = null
var rear_left_wheel: Node = null
var rear_right_wheel: Node = null

# ============================================================================
# HELPER METHODS
# ============================================================================

func _init() -> void:
	pass

func _ready() -> void:
	_init_physics_settings()
	_init_wheel_positions()
	_connect_signals()
	_setup_audio()
	_generate_torque_curve()

func _init_physics_settings() -> void:
	if Engine.has_singleton("PhysicsSettings"):
		physics = Engine.get_singleton("PhysicsSettings")
	else:
		physics = preload("res://scripts/core/PhysicsSettings.gd").new()
	
	vehicle_mass = physics.default_vehicle_mass

func _init_wheel_positions() -> void:
	var half_track = track_width * 0.5
	var half_wb = wheelbase * 0.5
	
	# Front wheels (positive Z is forward)
	wheel_positions.push_back(Vector3(-half_track, 0, half_wb))  # Front Left
	wheel_positions.push_back(Vector3(half_track, 0, half_wb))   # Front Right
	wheel_positions.push_back(Vector3(-half_track, 0, -half_wb)) # Rear Left
	wheel_positions.push_back(Vector3(half_track, 0, -half_wb))  # Rear Right
	
	# Initialize wheel contact points
	for i in range(4):
		wheel_contact_points.append(Vector3.ZERO)
		wheel_forces.append(Vector3.ZERO)

func _connect_signals() -> void:
	if powertrain:
		powertrain.rpm_changed.connect(_on_powertrain_rpm_changed)
		powertrain.engine_ready.connect(_on_powertrain_engine_ready)

func _setup_audio() -> void:
	if Engine.has_singleton("AudioManager"):
		# Audio will be triggered through signals
		pass

func _generate_torque_curve() -> void:
	"""Generate cached torque curve for performance"""
	_torque_curve_cache.clear()
	var step: float = 500.0
	for rpm in range(int(engine_idle_rpm), int(engine_max_rpm) + int(step), int(step)):
		var rpm_normalized: float = (rpm - engine_idle_rpm) / (engine_max_rpm - engine_idle_rpm)
		var torque: float = _calculate_torque_at_rpm(rpm)
		_torque_curve_cache[rpm] = torque

func _calculate_torque_at_rpm(rpm: float) -> float:
	"""Calculate engine torque at given RPM based on torque curve"""
	var rpm_normalized: float = (rpm - engine_idle_rpm) / (engine_max_rpm - engine_idle_rpm)
	rpm_normalized = clamp(rpm_normalized, 0.0, 1.0)
	
	# Torque curve follows a bell-shaped distribution peaking at ~40% of redline
	var peak_rpm: float = engine_idle_rpm + (engine_max_rpm - engine_idle_rpm) * 0.4
	var distance_from_peak: float = abs(rpm - peak_rpm)
	var spread: float = (engine_max_rpm - engine_idle_rpm) * 0.3
	
	var torque: float = engine_peak_torque * exp(-(distance_from_peak ** 2) / (2 * spread ** 2))
	torque *= torque_curve_flatness
	
	return torque

# ============================================================================
# INPUT HANDLING
# ============================================================================

func handle_throttle_input(input_value: float) -> void:
	throttle_input = clamp(input_value, 0.0, 1.0)

func handle_brake_input(input_value: float) -> void:
	brake_input = clamp(input_value, 0.0, 1.0)

func handle_steering_input(input_value: float) -> void:
	steering_input = clamp(input_value, -1.0, 1.0)

func handle_clutch_input(input_value: float) -> void:
	clutch_input = clamp(input_value, 0.0, 1.0)

func handle_handbrake_input(input_value: float) -> void:
	handbrake_input = clamp(input_value, 0.0, 1.0)

func handle_shift_up() -> void:
	_shift_gear(current_gear + 1)

func handle_shift_down() -> void:
	_shift_gear(current_gear - 1)

func handle_neutral() -> void:
	_shift_gear(neutral_position)

func handle_reverse() -> void:
	_shift_gear(reverse_position)

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================

func _shift_gear(target_gear: int) -> void:
	var old_gear: int = current_gear
	current_gear = clamp(target_gear, 0, gear_raties.size() - 1)
	
	if current_gear != old_gear:
		gear_changed.emit(old_gear, current_gear)
		
		# Apply gear ratio
		var gear_ratio: float = gear_ratios[current_gear]
		if gear_ratio != 0.0:
			_target_rpm = current_rpm * (gear_ratio / gear_ratios[old_gear]) if old_gear != 0 else current_rpm
		else:
			_target_rpm = engine_idle_rpm
		
		# Clutch engagement effect
		if clutch_input < 0.5:
			# Quick shift with clutch slip
			current_rpm = lerp(current_rpm, _target_rpm, 0.1)
		else:
			# Smooth shift with clutch engaged
			current_rpm = _target_rpm

func _can_shift() -> bool:
	"""Check if safe to shift gears"""
	if current_gear == neutral_position:
		return true
	
	if handbrake_input > 0.0:
		return false
	
	if abs(current_rpm - target_rpm) > 1500.0:
		return false
	
	return true

func _should_upshift() -> bool:
	"""Auto upshift condition"""
	if current_gear >= first_gear_position and current_gear < sixth_gear_position:
		return current_rpm > engine_max_rpm * 0.95
	return false

func _should_downshift() -> bool:
	"""Auto downshift condition"""
	if current_gear > neutral_position:
		var next_lower_gear: int = current_gear - 1
		if next_lower_gear >= neutral_position:
			var speed_at_next_gear: float = _get_speed_at_rpm(engine_idle_rpm, next_lower_gear)
			if current_speed < speed_at_next_gear * 1.2:
				return true
	return false

func _get_speed_at_rpm(rpm: float, gear: int) -> float:
	"""Calculate vehicle speed at given RPM and gear"""
	if gear <= neutral_position:
		return 0.0
	
	var wheel_angular_velocity: float = rpm * 2.0 * PI / 60.0
	var wheel_linear_velocity: float = wheel_angular_velocity * tire_radius
	var gear_ratio: float = abs(gear_ratios[gear])
	var final_drive: float = final_drive_ratio
	
	var speed_mps: float = wheel_linear_velocity * gear_ratio * final_drive
	var speed_kmh: float = speed_mps * 3.6
	
	return speed_kmh

# ============================================================================
# VEHICLE DYNAMICS CALCULATION
# ============================================================================

func _physics_process(delta: float) -> void:
	_update_state(delta)
	_calculate_physics(delta)
	_apply_forces(delta)
	_handle_collision_detection(delta)

func _update_state(delta: float) -> void:
	"""Update current vehicle state based on inputs"""
	# Update RPM based on throttle and gear
	if current_gear > neutral_position:
		var gear_ratio: float = gear_ratios[current_gear]
		var wheel_speed_from_rpm: float = current_rpm / gear_ratio / final_drive_ratio
		var wheel_linear_speed: float = wheel_speed_from_rpm * 2.0 * PI * tire_radius
		var estimated_speed: float = wheel_linear_speed * 3.6
		
		# RPM rises with throttle, falls with braking/downshifting
		if throttle_input > 0.0:
			target_rpm = min(current_rpm + delta * 1500.0 * throttle_input, engine_max_rpm)
		elif brake_input > 0.0:
			target_rpm = max(current_rpm - delta * 800.0 * brake_input, engine_idle_rpm)
		else:
			target_rpm = lerp(current_rpm, target_rpm, delta * 5.0)
		
		current_rpm = lerp(current_rpm, target_rpm, delta * 10.0)
	else:
		# Neutral or reverse - idle RPM
		target_rpm = engine_idle_rpm
		current_rpm = lerp(current_rpm, target_rpm, delta * 10.0)
	
	# Calculate velocity from physics body
	current_velocity_mps = global_transform.basis.z.dot(velocity) * -1.0
	current_speed = abs(current_velocity_mps) * 3.6
	
	# Update steering angle
	steering_angle = steering_input * 30.0 * DEG2RAD
	
	# Auto-shift detection
	if Engine.has_singleton("GameManager"):
		var gm = Engine.get_singleton("GameManager")
		if not gm.debug_mode:
			if _should_upshift():
				handle_shift_up()
			elif _should_downshift():
				handle_shift_down()

func _calculate_physics(delta: float) -> void:
	"""Calculate all vehicle dynamics"""
	# Calculate wheel torque based on throttle and gear
	if current_gear > neutral_position:
		var gear_ratio: float = gear_ratios[current_gear]
		var torque_at_wheel: float = _calculate_wheel_torque()
		wheel_torque = torque_at_wheel * gear_ratio * final_drive_ratio * transmission_efficiency
	else:
		wheel_torque = 0.0
	
	# Calculate braking force
	braking_force = _calculate_braking_force()
	
	# Calculate aerodynamic forces
	var aero_downforce: float = _calculate_aero_downforce()
	var aero_drag: float = _calculate_aero_drag()
	
	# Calculate acceleration
	longitudinal_acceleration = _calculate_longitudinal_acceleration(aero_drag)
	
	# Calculate lateral forces for drift
	_calculate_lateral_forces()
	
	# Check drift conditions
	_check_drift_state()
	
	# Update suspension compression
	_update_suspension(delta)

func _calculate_wheel_torque() -> float:
	"""Calculate engine torque delivered to wheels"""
	var rpm_normalized: float = (current_rpm - engine_idle_rpm) / (engine_max_rpm - engine_idle_rpm)
	rpm_normalized = clamp(rpm_normalized, 0.0, 1.0)
	
	# Look up or calculate torque
	if _torque_curve_cache.has(current_rpm):
		var base_torque: float = _torque_curve_cache[current_rpm]
	else:
		var base_torque: float = _calculate_torque_at_rpm(current_rpm)
	
	# Apply throttle multiplier
	var applied_torque: float = base_torque * throttle_input
	
	# Apply clutch slip factor
	applied_torque *= (1.0 - (clutch_input * (1.0 - clutch_friction)))
	
	return applied_torque

func _calculate_braking_force() -> float:
	"""Calculate total braking force"""
	var brake_pressure: float = brake_input * max_brake_pressure
	
	var brake_force_per_piston: float = brake_pressure * brake_caliper_piston_area
	var pad_friction_force: float = brake_force_per_piston * brake_pad_friction
	
	# Disc radius converts to torque
	var brake_torque: float = pad_friction_force * brake_disc_radius * 2.0  # Two pads
	
	# Distribute front/rear based on brake bias
	var front_brake: float = brake_torque * front_brake_bias
	var rear_brake: float = brake_torque * (1.0 - front_brake_bias)
	
	# Convert to linear braking force (simplified)
	var total_braking_force: float = (front_brake + rear_brake) / (tire_radius * 2.0)
	
	return total_braking_force

func _calculate_aero_downforce() -> float:
	"""Calculate aerodynamic downforce"""
	var speed_squared: float = current_velocity_mps ** 2
	var air_density: float = 1.225  # kg/m^3 at sea level
	
	var downforce: float = 0.5 * air_density * speed_squared * frontal_area * downforce_coefficient
	
	# Add speed-dependent lift reduction
	var lift: float = 0.5 * air_density * speed_squared * frontal_area * max_lift
	downforce -= lift
	
	return max(downforce, 0.0)

func _calculate_aero_drag() -> float:
	"""Calculate aerodynamic drag force"""
	var speed_squared: float = current_velocity_mps ** 2
	var air_density: float = 1.225
	
	var drag: float = 0.5 * air_density * speed_squared * frontal_area * drag_coefficient
	
	return drag

func _calculate_longitudinal_acceleration(aero_drag: float) -> float:
	"""Calculate net longitudinal acceleration"""
	# Driving force from wheels
	var driving_force: float = wheel_torque / tire_radius
	
	# Braking force
	var braking_force_total: float = braking_force + (handbrake_input * braking_force * 0.5)
	
	# Aerodynamic drag
	var total_resistance: float = aero_drag + braking_force_total
	
	# Net force
	var net_force: float = driving_force - total_resistance
	
	# F = ma, so a = F/m
	var acceleration: float = net_force / vehicle_mass
	
	return acceleration

func _calculate_lateral_forces() -> void:
	"""Calculate lateral forces for handling and drift"""
	# Simplified lateral acceleration estimate
	lateral_acceleration = yaw_rate * current_velocity_mps
	
	# Slip ratio calculation
	slip_ratio = _calculate_slip_ratio()

func _calculate_slip_ratio() -> float:
	"""Calculate wheel slip ratio"""
	var wheel_angular_velocity: float = current_rpm * 2.0 * PI / 60.0
	var wheel_linear_velocity: float = wheel_angular_velocity * tire_radius
	
	if abs(current_velocity_mps) > 0.1:
		var slip: float = (wheel_linear_velocity - current_velocity_mps) / abs(current_velocity_mps)
		return clamp(slip, -1.0, 1.0)
	
	return 0.0

func _check_drift_state() -> void:
	"""Determine if vehicle is drifting"""
	var lateral_acc_threshold: float = gravity * drift_threshold
	
	if abs(lateral_acceleration) > lateral_acc_threshold and throttle_input > 0.3:
		if not drift_active:
		(drift_active = true
		 drift_angle = lateral_acceleration / gravity * 180.0 / PI
		 drift_started.emit(drift_angle)
		else:
		 drift_intensity = lerp(drift_intensity, 1.0, delta * 2.0)
	else:
		if drift_active:
		 drift_active = false
		 drift_intensity = lerp(drift_intensity, 0.0, delta * drift_recovery)
		 drift_ended.emit(drift_angle)
		 drift_angle = 0.0

func _update_suspension(delta: float) -> void:
	"""Update suspension compression and velocity"""
	var weight_distribution: float = 0.4  # 40% front, 60% rear
	
	var axle_weight_front: float = vehicle_mass * 9.81 * weight_distribution
	var axle_weight_rear: float = vehicle_mass * 9.81 * (1.0 - weight_distribution)
	
	# Spring compression = force / stiffness
	var front_compression: float = axle_weight_front / spring_stiffness
	var rear_compression: float = axle_weight_rear / spring_stiffness
	
	# Clamp to travel limits
	front_compression = clamp(front_compression, 0.0, suspension_travel)
	rear_compression = clamp(rear_compression, 0.0, suspension_travel)
	
	suspension_compression.x = front_compression
	suspension_compression.y = front_compression
	suspension_compression.z = rear_compression
	suspension_compression.w = rear_compression

func _apply_forces(delta: float) -> void:
	"""Apply calculated forces to vehicle physics body"""
	# Apply longitudinal acceleration
	var acceleration_vector: Vector3 = Vector3.FORWARD * longitudinal_acceleration * physics.gravity
	
	# Apply lateral forces (steering effect)
	var steering_factor: float = steering_input * understeer_coefficient * (1.0 - abs(yaw_rate))
	var lateral_force: Vector3 = global_transform.basis.x * steering_factor * lateral_acceleration
	
	# Combine accelerations
	var total_acceleration: Vector3 = acceleration_vector + lateral_force
	
	# Apply to velocity
	velocity += total_acceleration * delta
	
	# Apply damping for realistic friction
	var friction_damping: float = 0.98
	velocity = velocity * friction_damping
	
	# Limit maximum speed
	var max_speed: float = 300.0  # km/h limit
	if current_speed > max_speed:
		velocity = velocity.normalized() * (max_speed / 3.6)
	
	# Apply to character body
	move_and_slide()

func _handle_collision_detection(delta: float) -> void:
	"""Detect and handle collisions"""
	for collision in get_slide_collision_count():
		var col = get_slide_collision(collision)
		var collider: Node = col.get_collider()
		var impact_normal: Vector3 = col.get_normal()
		
		# Calculate impact force
		var impact_velocity: float = velocity.length()
		var impact_force: Vector3 = impact_velocity * impact_normal * vehicle_mass
		
		collision_impact.emit(impact_force, col.get_position())
		
		# Screen shake and particle effects could go here
		# via GameManager or AudioManager signals

# ============================================================================
# WHEEL FORCE APPLICATION
# ============================================================================

func apply_wheel_forces(wheel_index: int, force: Vector3) -> void:
	"""Apply force to specific wheel"""
	if wheel_index >= 0 and wheel_index < wheel_forces.size():
		wheel_forces[wheel_index] = force

func get_wheel_contact_point(wheel_index: int) -> Vector3:
	"""Get world position of wheel contact point"""
	if wheel_index >= 0 and wheel_index < wheel_contact_points.size():
		return wheel_contact_points[wheel_index]
	return Vector3.ZERO

func update_wheel_positions() -> void:
	"""Update wheel world positions based on vehicle transform"""
	for i in range(4):
		wheel_contact_points[i] = global_transform * wheel_positions[i]

# ============================================================================
# SIGNAL EMITTERS
# ============================================================================

func emit_engine_sound_signal() -> void:
	"""Emit signal for audio manager to adjust engine sound pitch"""
	if Engine.has_singleton("AudioManager"):
		var amplitude: float = current_rpm / engine_max_rpm
		engine_sound_changed.emit(amplitude)

func emit_speed_signal() -> void:
	"""Emit signal for UI updates"""
	speed_changed.emit(current_speed)

func emit_rpm_signal() -> void:
	"""Emit signal for tachometer updates"""
	rpm_changed.emit(current_rpm)

# ============================================================================
# UTILITY METHODS
# ============================================================================

func reset_vehicle() -> void:
	"""Reset vehicle to initial state"""
	current_gear = neutral_position
	current_rpm = engine_idle_rpm
	target_rpm = engine_idle_rpm
	current_speed = 0.0
	current_velocity_mps = 0.0
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	clutch_input = 0.0
	handbrake_input = 0.0
	drift_active = false
	drift_intensity = 0.0
	velocity = Vector3.ZERO

func set_vehicle_configuration(config: Dictionary) -> void:
	"""Set vehicle configuration from external source"""
	if config.has("mass"):
		vehicle_mass = config["mass"]
	if config.has("max_rpm"):
		engine_max_rpm = config["max_rpm"]
	if config.has("peak_torque"):
		engine_peak_torque = config["peak_torque"]
	if config.has("gear_ratios"):
		gear_ratios = config["gear_ratios"]

# ============================================================================
# PROPERTY SETTERS
# ============================================================================

func _set_vehicle_mass(value: float) -> void:
	vehicle_mass = value

func _set_center_of_mass(value: Vector3) -> void:
	center_of_mass_offset = value

func _set_track_width(value: float) -> void:
	track_width = value
	_init_wheel_positions()

func _set_wheelbase(value: float) -> void:
	wheelbase = value
	_init_wheel_positions()

func _set_ride_height(value: float) -> void:
	ride_height = value

func _set_drag_coeff(value: float) -> void:
	drag_coefficient = value

func _set_frontal_area(value: float) -> void:
	frontal_area = value

func _set_downforce_coeff(value: float) -> void:
	downforce_coefficient = value

func _set_max_lift(value: float) -> void:
	max_lift = value

func _set_suspension_travel(value: float) -> void:
	suspension_travel = value

func _set_spring_stiffness(value: float) -> void:
	spring_stiffness = value

func _set_damping_rate(value: float) -> void:
	damping_rate = value

func _set_anti_roll(value: float) -> void:
	anti_roll_bar_stiffness = value

func _set_camber_gain(value: float) -> void:
	camber_gain = value

func _set_toe_in(value: float) -> void:
	toe_in = value

func _set_engine_max_rpm(value: float) -> void:
	engine_max_rpm = value
	_generate_torque_curve()

func _set_engine_idle_rpm(value: float) -> void:
	engine_idle_rpm = value
	_generate_torque_curve()

func _set_peak_torque(value: float) -> void:
	engine_peak_torque = value
	_generate_torque_curve()

func _set_peak_power(value: float) -> void:
	engine_peak_power = value

func _set_torque_flatness(value: float) -> void:
	torque_curve_flatness = value

func _set_final_drive(value: float) -> void:
	final_drive_ratio = value

func _set_clutch_friction(value: float) -> void:
	clutch_friction = value

func _set_trans_eff(value: float) -> void:
	transmission_efficiency = value

func _set_gear_ratios(value: Array[float]) -> void:
	gear_ratios = value

func _set_brake_bias(value: float) -> void:
	front_brake_bias = value

func _set_max_brake_press(value: float) -> void:
	max_brake_pressure = value

func _set_brake_disc_radius(value: float) -> void:
	brake_disc_radius = value

func _set_caliper_area(value: float) -> void:
	brake_caliper_piston_area = value

func _set_brake_pad_frict(value: float) -> void:
	brake_pad_friction = value

func _set_tire_stiffness(value: float) -> void:
	tire_stiffness = value

func _set_tire_friction(value: float) -> void:
	tire_friction_coefficient = value

func _set_tire_width(value: float) -> void:
	tire_width = value

func _set_tire_radius(value: float) -> void:
	tire_radius = value

func _set_tire_compliance(value: float) -> void:
	tire_vertical_compliance = value

func _set_drift_thresh(value: float) -> void:
漂移阈值 = value

func _set_drift_recovery(value: float) -> void:
漂移恢复 = value

func _set_understeer(value: float) -> void:
understeer系数 = value

func _set_oversteer(value: float) -> void:
oversteer系数 = value

# ============================================================================
# POWERTRAIN SIGNAL HANDLERS
# ============================================================================

func _on_powertrain_rpm_changed(rpm: float) -> void:
	current_rpm = rpm
	emit_rpm_signal()

func _on_powertrain_engine_ready() -> void:
	pass

# ============================================================================
# EXPORTED DEBUG METHODS
# ============================================================================

func debug_get_stats() -> Dictionary:
	"""Get current vehicle stats for debugging"""
	return {
		"speed_kmh": current_speed,
		"velocity_mps": current_velocity_mps,
		"rpm": current_rpm,
		"target_rpm": target_rpm,
		"gear": current_gear,
		"throttle": throttle_input,
		"brake": brake_input,
		"steering": steering_input,
		"drift_active": drift_active,
		"drift_intensity": drift_intensity,
		"acceleration": longitudinal_acceleration,
		"lateral_acceleration": lateral_acceleration,
		"wheel_torque": wheel_torque,
		"braking_force": braking_force
	}