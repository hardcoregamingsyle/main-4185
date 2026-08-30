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
@export var clutch_engagement_rpm: float = 1200.0
@export var rev_matching_enabled: bool = true

@export_group("Braking System")
@export var brake_pressure_front: float = 1.0
@export var brake_pressure_rear: float = 0.8
@export var brake_distribution_front: float = 0.6
@export var brake_balance: float = 0.5
@export var abs_enabled: bool = true
@export var brake_bias_active: bool = false

@export_group("Suspension Parameters")
@export var spring_rate_front: float = 45000.0
@export var spring_rate_rear: float = 40000.0
@export var damping_compression_front: float = 5000.0
@export var damping_rebound_front: float = 2000.0
@export var damping_compression_rear: float = 4500.0
@export var damping_rebound_rear: float = 1800.0
@export var travel_front: float = 0.15
@export var travel_rear: float = 0.18
@export var ride_height_front: float = 0.12
@export var ride_height_rear: float = 0.14

@export_group("Tire Friction Coefficients")
@export var tire_friction_lateral: float = 1.2
@export var tire_friction_longitudinal: float = 1.0
@export var tire_stiffness_lateral: float = 25000.0
@export var tire_stiffness_longitudinal: float = 15000.0
@export var tire_load_sensitivity: float = 0.3

@export_group("Drift & Handling")
@export var drift_threshold: float = 0.15
@export var drift_recovery_rate: float = 0.1
@export var oversteer_factor: float = 0.8
@export var understeer_factor: float = 1.0
@export var anti_roll_bar_stiffness: float = 5000.0

# ============================================================================
# INTERNAL STATE VARIABLES
# ============================================================================

var current_speed: float = 0.0
var current_rpm: float = idle_rpm
var current_gear: int = 0
var target_gear: int = 0
var accelerator_input: float = 0.0
var brake_input: float = 0.0
var steering_input: float = 0.0
var clutch_input: float = 0.0
var handbrake_input: float = 0.0
var drift_state: float = 0.0
var wheel_slip_values: Array[float] = [0.0, 0.0, 0.0, 0.0]

# Wheel positions (relative to chassis center)
var wheel_positions: Array[Vector3] = []
var wheel_rotations: Array[float] = [0.0, 0.0, 0.0, 0.0]
var wheel_vertical_velocities: Array[float] = [0.0, 0.0, 0.0, 0.0]

# Engine state
var engine_braking: bool = false
var throttle_value: float = 0.0
var applied_torque: float = 0.0
var drivetrain_loss: float = 0.0

# Aerodynamics
var air_density: float = 1.225
var downforce: float = 0.0
var lift: float = 0.0
var drag_force: float = 0.0

# Suspension state
var suspension_compression_front: float = 0.0
var suspension_compression_rear: float = 0.0
var roll_angle: float = 0.0

# Collision tracking
var last_collision_time: float = 0.0
var collision_impact_velocity: Vector3 = Vector3.ZERO
var is_colliding: bool = false

# ============================================================================
# CONSTRUCTOR / INITIALIZATION
# ============================================================================

func _init() -> void:
	_reset_vehicle_state()
	_initialize_wheel_positions()

func _reset_vehicle_state() -> void:
	current_speed = 0.0
	current_rpm = idle_rpm
	current_gear = 0
	target_gear = 0
	throttle_value = 0.0
	applied_torque = 0.0
	engine_braking = false
	wheel_slip_values.fill(0.0)
	suspension_compression_front = 0.0
	suspension_compression_rear = 0.0
	roll_angle = 0.0
	is_colliding = false

func _initialize_wheel_positions() -> void:
	var half_track = track_width * 0.5
	var half_wheelbase = wheel_base * 0.5
	var wheel_y = ground_clearance
	
	wheel_positions = [
		Vector3(-half_track, wheel_y, half_wheelbase),  # Front Left
		Vector3(half_track, wheel_y, half_wheelbase),   # Front Right
		Vector3(-half_track, wheel_y, -half_wheelbase), # Rear Left
		Vector3(half_track, wheel_y, -half_wheelbase)   # Rear Right
	]

# ============================================================================
# MAIN PHYSICS PROCESS
# ============================================================================

func _physics_process(delta: float) -> void:
	if not is_ready():
		return
	
	# Update time scale from physics settings
	var scaled_delta = delta * _physics.time_scale
	
	# Accumulate inputs
	_accumulate_inputs(scaled_delta)
	
	# Calculate engine RPM based on gear and speed
	_calculate_engine_rpm(scaled_delta)
	
	# Apply throttle and brake inputs
	_apply_powertrain(scaled_delta)
	
	# Calculate aerodynamic forces
	_calculate_aerodynamics()
	
	# Calculate wheel forces and apply traction
	_calculate_wheel_forces(scaled_delta)
	
	# Apply suspension and gravity
	_update_suspension(scaled_delta)
	
	# Apply calculated forces to velocity
	_apply_physics_to_body(scaled_delta)
	
	# Handle collisions
	_handle_collisions(scaled_delta)
	
	# Emit signals for game systems
	_emit_signals()

func is_ready() -> bool:
	return engine != null and body != null

func _accumulate_inputs(delta: float) -> void:
	# Smooth input transitions
	var smoothing = delta * 5.0
	
	accelerator_input = lerp(accelerator_input, InputManager.get_axis("accelerate"), smoothing)
	brake_input = lerp(brake_input, InputManager.get_axis("brake"), smoothing)
	steering_input = lerp(steering_input, InputManager.get_axis("steer_left", "steer_right"), smoothing)
	clutch_input = lerp(clutch_input, InputManager.get_axis("clutch"), smoothing)
	handbrake_input = lerp(handbrake_input, InputManager.get_axis("handbrake"), smoothing)
	
	# Clamp inputs to valid ranges
	accelerator_input = clamp(accelerator_input, 0.0, 1.0)
	brake_input = clamp(brake_input, 0.0, 1.0)
	steering_input = clamp(steering_input, -1.0, 1.0)
	clutch_input = clamp(clutch_input, 0.0, 1.0)
	handbrake_input = clamp(handbrake_input, 0.0, 1.0)

func _calculate_engine_rpm(delta: float) -> void:
	# Calculate wheel RPM based on vehicle speed and gear
	if current_gear >= 0 and current_gear < gear_ratios.size():
		var wheel_rpm = current_speed * 60.0 / (2.0 * PI * 0.3)  # Assuming 0.3m wheel radius
		var total_reduction = gear_ratios[current_gear] * final_drive_ratio
		
		var target_rpm = wheel_rpm * total_reduction * transmission_efficiency
		
		# Apply engine inertia (simplified)
		var rpm_change = (target_rpm - current_rpm) * delta * 0.1
		current_rpm += rpm_change
		
		# Clamp to engine limits
		current_rpm = clamp(current_rpm, engine_min_rpm, engine_max_rpm)
		
		# Idle behavior when no throttle
		if accelerator_input < 0.05 and current_rpm > idle_rpm:
			current_rpm = lerp(current_rpm, idle_rpm, delta * 2.0)

func _apply_powertrain(delta: float) -> void:
	# Determine if we're in neutral or a valid gear
	var has_valid_gear = current_gear >= 0 and current_gear < gear_ratios.size()
	
	# Calculate desired torque based on accelerator input
	var desired_torque = accelerator_input * max_torque
	
	# Get torque multiplier from curve (if defined)
	if torque_curve_points.size() > 0:
		desired_torque *= _get_torque_multiplier(current_rpm)
	else:
		# Default linear curve
		var rpm_ratio = (current_rpm - engine_min_rpm) / (engine_max_rpm - engine_min_rpm)
		desired_torque *= 0.5 + 0.5 * sin(rpm_ratio * PI)
	
	# Apply transmission losses
	applied_torque = desired_torque * transmission_efficiency
	
	# Engine braking when releasing throttle
	if accelerator_input < 0.05 and current_rpm > idle_rpm:
		engine_braking = true
		applied_torque *= -0.3  # Moderate engine braking
	else:
		engine_braking = false
	
	# Apply brake forces
	_apply_brakes(delta)
	
	# Handbrake adds additional rear brake force
	if handbrake_input > 0.0:
		_apply_handbrake(delta, handbrake_input)

func _get_torque_multiplier(rpm: float) -> float:
	if torque_curve_points.is_empty():
		return 1.0
	
	var ratio = (rpm - engine_min_rpm) / (engine_max_rpm - engine_min_rpm)
	
	# Linear interpolation between torque curve points
	for i in range(torque_curve_points.size() - 1):
		var p1 = torque_curve_points[i]
		var p2 = torque_curve_points[i + 1]
		
		if ratio >= p1.x and ratio <= p2.x:
			var local_ratio = (ratio - p1.x) / (p2.x - p1.x)
			return p1.y + (p2.y - p1.y) * local_ratio
	
	return 1.0

func _apply_brakes(delta: float) -> void:
	if brake_input > 0.0:
		var brake_force = brake_input * brake_pressure_front * vehicle_mass * 9.81
		var front_brake = brake_force * brake_distribution_front
		var rear_brake = brake_force * (1.0 - brake_distribution_front)
		
		# Apply braking to slow down
		var deceleration = (front_brake + rear_brake) / vehicle_mass
		var velocity_magnitude = velocity.length()
		
		if velocity_magnitude > 0.1:
			var direction = velocity.normalized()
			var brake_velocity = deceleration * delta
			
			if brake_velocity >= velocity_magnitude:
				velocity = Vector3.ZERO
			else:
				velocity -= direction * brake_velocity

func _apply_handbrake(delta: float, handbrake_force: float) -> void:
	# Handbrake primarily affects rear wheels
	var handbrake_force_actual = handbrake_force * 0.5 * vehicle_mass * 9.81
	var rear_brake_addition = handbrake_force_actual * 0.8
	
	# Increase drift tendency
	if current_speed > 5.0:
		var drift_increase = handbrake_force * delta * 0.5
		wheel_slip_values[2] += drift_increase
		wheel_slip_values[3] += drift_increase

func _calculate_aerodynamics() -> void:
	# Calculate drag force
	drag_force = 0.5 * air_density * drag_coefficient * frontal_area * current_speed * current_speed
	
	# Calculate downforce (simplified - proportional to speed squared)
	downforce = 0.5 * air_density * 0.3 * frontal_area * current_speed * current_speed
	
	# Calculate lift (usually negative for race cars)
	lift = 0.5 * air_density * (-0.1) * frontal_area * current_speed * current_speed

func _calculate_wheel_forces(delta: float) -> void:
	# Calculate individual wheel forces based on slip ratios
	for wheel_index in 4:
		var longitudinal_slip = _calculate_longitudinal_slip(wheel_index)
		var lateral_slip = _calculate_lateral_slip(wheel_index)
		
		wheel_slip_values[wheel_index] = longitudinal_slip
		
		# Calculate friction force using simplified Pacejka model
		var normal_force = _get_wheel_normal_force(wheel_index)
		var friction_force = _calculate_tire_friction(longitudinal_slip, lateral_slip, normal_force)
		
		# Apply wheel force to vehicle velocity
		if wheel_index < 2:  # Front wheels
			velocity.x += friction_force * steering_input * delta / vehicle_mass
		else:  # Rear wheels
			# Rear wheel drive bias
			if not engine_braking and applied_torque > 0:
				var drive_force = applied_torque * gear_ratios[current_gear] * final_drive_ratio / 0.3
				velocity.z += drive_force * delta / vehicle_mass

func _calculate_longitudinal_slip(wheel_index: int) -> float:
	if current_speed == 0.0:
		return 0.0
	
	var wheel_radius = 0.3
	var wheel_rpm = current_rpm * gear_ratios[current_gear] * final_drive_ratio
	
	var theoretical_speed = wheel_rpm * wheel_radius * 2.0 * PI / 60.0
	var slip = (theoretical_speed - current_speed) / max(current_speed, 1.0)
	
	return clamp(slip, -1.0, 1.0)

func _calculate_lateral_slip(wheel_index: int) -> float:
	# Simplified lateral slip calculation
	var lateral_velocity = velocity.x
	var forward_velocity = velocity.z
	
	if forward_velocity == 0.0:
		return 0.0
	
	var slip_angle = atan(lateral_velocity / forward_velocity)
	
	# Adjust for steering angle on front wheels
	if wheel_index < 2:
		slip_angle += steering_input * 0.3
	
	return sin(slip_angle)

func _calculate_tire_friction(longitudinal: float, lateral: float, normal_force: float) -> float:
	# Simplified friction circle model
	var max_friction = normal_force * tire_friction_lateral
	var combined_slip = sqrt(longitudinal * longitudinal + lateral * lateral)
	
	var friction_force = max_friction * combined_slip
	
	return friction_force

func _get_wheel_normal_force(wheel_index: int) -> float:
	# Distribute weight between front and rear
	var weight_distribution = 0.4 if wheel_index < 2 else 0.6
	var base_weight = vehicle_mass * 9.81 * weight_distribution
	
	# Add dynamic load transfer
	var acceleration = velocity.length()
	var load_transfer = acceleration * vehicle_mass * 0.1
	
	return base_weight + load_transfer

func _update_suspension(delta: float) -> void:
	# Simplified suspension compression
	var gravity_force = vehicle_mass * 9.81
	var aerodynamic_downforce = downforce
	
	var total_vertical_force = gravity_force - aerodynamic_downforce
	var front_spring = spring_rate_front
	var rear_spring = spring_rate_rear
	
	suspension_compression_front = total_vertical_force / front_spring
	suspension_compression_rear = total_vertical_force / rear_spring
	
	# Damping (simplified)
	var damping_front = damping_compression_front * vertical_velocity.y
	var damping_rear = damping_compression_rear * vertical_velocity.y
	
	vertical_velocity.y -= (damping_front + damping_rear) * delta / vehicle_mass

func _apply_physics_to_body(delta: float) -> void:
	# Apply drag force opposite to velocity direction
	if velocity.length() > 0:
		var drag_direction = velocity.normalized()
		velocity -= drag_direction * drag_force * delta / vehicle_mass
	
	# Apply gravity
	velocity.y -= _physics.gravity * delta
	
	# Apply aerodynamic forces
	velocity.y += (downforce - lift) * delta / vehicle_mass

func _handle_collisions(delta: float) -> void:
	if is_colliding:
		var impact_velocity = collision_impact_velocity.length()
		
		# Register collision event
		collision_impact.emit(impact_velocity, Vector3.ZERO)
		
		# Reduce speed significantly
		velocity *= 0.5
		
		# Reset collision flag after short delay
		last_collision_time = get_tree().time
		
		if get_tree().time - last_collision_time > 0.5:
			is_colliding = false

func _emit_signals() -> void:
	speed_changed.emit(current_speed)
	rpm_changed.emit(current_rpm)
	gear_changed.emit(current_gear)

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================

func shift_gear(target_gear_idx: int) -> void:
	if target_gear_idx < 0 or target_gear_idx >= gear_ratios.size():
		return
	
	if target_gear_idx == current_gear:
		return
	
	# Validate gear shift based on RPM
	var next_rpm = _calculate_next_rpm(target_gear_idx)
	
	if rev_matching_enabled:
		_adjust_rpm_for_shift(target_gear_idx)
	
	target_gear = target_gear_idx
	
	# Execute shift
	await _execute_gear_shift()
	gear_changed.emit(target_gear)

func _calculate_next_rpm(gear_idx: int) -> float:
	if gear_idx >= gear_ratios.size():
		return engine_max_rpm
	
	var wheel_rpm = current_speed * 60.0 / (2.0 * PI * 0.3)
	var total_reduction = gear_ratios[gear_idx] * final_drive_ratio
	
	return wheel_rpm * total_reduction * transmission_efficiency

func _adjust_rpm_for_shift(gear_idx: int) -> void:
	var current_total_reduction = gear_ratios[current_gear] * final_drive_ratio
	var target_total_reduction = gear_ratios[gear_idx] * final_drive_ratio
	
	var rpm_ratio = target_total_reduction / current_total_reduction
	current_rpm *= rpm_ratio
	current_rpm = clamp(current_rpm, engine_min_rpm, engine_max_rpm)

func _execute_gear_shift() -> void:
	# Simulate shift duration
	await get_tree().create_timer(0.1).timeout
	current_gear = target_gear

# ============================================================================
# UTILITY METHODS
# ============================================================================

func get_current_speed_kmh() -> float:
	return current_speed * 3.6

func get_current_speed_mph() -> float:
	return current_speed * 2.237

func is_in_neutral() -> bool:
	return current_gear < 0

func get_max_speed() -> float:
	# Calculate theoretical max speed in top gear
	if gear_ratios.is_empty():
		return 0.0
	
	var top_gear = gear_ratios[gear_ratios.size() - 1]
	var wheel_rpm_at_max = engine_max_rpm * top_gear * final_drive_ratio * transmission_efficiency
	return wheel_rpm_at_max * 2.0 * PI * 0.3 / 60.0

func reset() -> void:
	_reset_vehicle_state()
	velocity = Vector3.ZERO
.angular_velocity = Vector3.ZERO
position = Vector3.ZERO
</file>
