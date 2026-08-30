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

# ============================================================================
# INPUT VALUES (from InputManager)
# ============================================================================

@export var _throttle_input: float = 0.0: set = _set_throttle_input
@export var _brake_input: float = 0.0: set = _set_brake_input
@export var _steering_input: float = 0.0: set = _set_steering_input
@export var _clutch_input: float = 0.0: set = _set_clutch_input
@export var _handbrake_input: float = 0.0: set = _set_handbrake_input

var _last_throttle: float = 0.0
var _last_brake: float = 0.0
var _last_steering: float = 0.0

# ============================================================================
# VEHICLE PHYSICS STATE
# ============================================================================

var current_speed: float = 0.0  # Speed in m/s
var max_speed: float = 65.0  # Max forward speed m/s
var reverse_speed: float = 20.0  # Max reverse speed m/s
var acceleration: float = 0.0
var deceleration: float = 0.0

var rotation_velocity: float = 0.0  # Yaw rate rad/s
var slip_angle: float = 0.0  # Tire slip angle
var lateral_acceleration: float = 0.0  # G-force lateral
var longitudinal_acceleration: float = 0.0  # G-force longitudinal

# ============================================================================
# GEARBOX SYSTEM
# ============================================================================

var current_gear: int = 0  # 0 = Neutral, 1-6 = Forward gears, -1 = Reverse
var target_gear: int = 0
var gear_shift_progress: float = 0.0  # 0.0 to 1.0 during shift
var is_shifting: bool = false
var shift_duration: float = 0.2  # Seconds per gear change

var _rpm: float = 800.0  # Current engine RPM
var idle_rpm: float = 800.0
var redline_rpm: float = 7500.0
var rev_limiter_active: bool = false

# ============================================================================
# WHEEL CONFIGURATION
# ============================================================================

const NUM_WHEELS: int = 4
var wheel_positions: Array[Vector3] = []
var wheel_radii: Array[float] = [0.33, 0.33, 0.33, 0.33]
var wheel_forces: Array[float] = [0.0, 0.0, 0.0, 0.0]
var wheel_rotation_angles: Array[float] = [0.0, 0.0, 0.0, 0.0]

# Wheel indices: Front Left, Front Right, Rear Left, Rear Right
const WHEEL_FL: int = 0
const WHEEL_FR: int = 1
const WHEEL_RL: int = 2
const WHEEL_RR: int = 3

# Track geometry from powertrain
var track_width: float = 1.6
var wheel_base: float = 2.8

# ============================================================================
# DRIFT AND HANDLING
# ============================================================================

var is_drifting: bool = false
var drift_intensity: float = 0.0  # 0.0 to 1.0
var drift_threshold: float = 0.35  # Slip angle threshold for drift
var grip_level: float = 1.0  # 1.0 = full grip, lower = slippery
var tire_temperature: Array[float] = [90.0, 90.0, 90.0, 90.0]  # Celsius

# ============================================================================
# COLLISION AND DAMAGE
# ============================================================================

var damage_level: float = 0.0  # 0.0 to 1.0
var body_health: float = 100.0
var suspension_compression: Array[float] = [0.0, 0.0, 0.0, 0.0]
var chassis_tilt_x: float = 0.0
var chassis_tilt_z: float = 0.0

# ============================================================================
# RACE DATA
# ============================================================================

var lap_count: int = 0
var current_lap_time: float = 0.0
var best_lap_time: float = 9999.0
var last_checkpoint_time: float = 0.0
var checkpoint_times: Array[float] = []
var race_position: int = 1
var total_race_time: float = 0.0
var is_race_active: bool = false

# ============================================================================
# INTERNAL REFERENCES
# ============================================================================

var _powertrain: Powertrain
var _physics_settings: PhysicsSettings

# ============================================================================
# INITIALIZATION
# ============================================================================

func _ready() -> void:
	_init_references()
	_setup_wheels()
	_connect_signals()
	_apply_default_settings()

func _init_references() -> void:
	if GameManager.has_singleton("PhysicsSettings"):
		_physics_settings = GameManager.get_singleton("PhysicsSettings")
	
	if GameManager.has_singleton("Powertrain"):
		_powertrain = GameManager.get_singleton("Powertrain")
	else:
		_powertrain = get_node_or_null("^Powertrain")
	
	if not _powertrain:
		_powertrain = preload("res://scripts/vehicles/Powertrain.gd").new()

func _setup_wheels() -> void:
	wheel_positions.resize(NUM_WHEELS)
	wheel_forces.resize(NUM_WHEELS)
	wheel_rotation_angles.resize(NUM_WHEELS)
	suspension_compression.resize(NUM_WHEELS)
	tire_temperature.resize(NUM_WHEELS)
	
	# Set wheel positions relative to center
	var half_track: float = track_width / 2.0
	var half_wheelbase: float = wheel_base / 2.0
	
	# Front wheels
	wheel_positions[WHEEL_FL] = Vector3(-half_track, -0.33, -half_wheelbase)
	wheel_positions[WHEEL_FR] = Vector3(half_track, -0.33, -half_wheelbase)
	# Rear wheels
	wheel_positions[WHEEL_RL] = Vector3(-half_track, -0.33, half_wheelbase)
	wheel_positions[WHEEL_RR] = Vector3(half_track, -0.33, half_wheelbase)

func _connect_signals() -> void:
	if _powertrain:
		_powertrain.rpm_changed.connect(_on_powertrain_rpm_changed)
		_powertrain.gear_changed.connect(_on_powertrain_gear_changed)

func _apply_default_settings() -> void:
	if _physics_settings:
		max_speed = _physics_settings.default_vehicle_max_speed
		reverse_speed = _physics_settings.default_reverse_max_speed
		grip_level = _physics_settings.default_tire_grip

# ============================================================================
# MAIN PROCESS LOOP
# ============================================================================

func _physics_process(delta: float) -> void:
	_update_physics(delta)
	_update_suspension(delta)
	_update_chassis(delta)
	_handle_collision_detection(delta)

func _process(delta: float) -> void:
	_update_gameplay(delta)
	_update_display(delta)

# ============================================================================
# PHYSICS UPDATE
# ============================================================================

func _update_physics(delta: float) -> void:
	# Apply forces based on input and powertrain state
	_apply_drive_force(delta)
	_apply_brake_force(delta)
	_apply_steering(delta)
	_apply_aerodynamic_drag(delta)
	_apply_gravity(delta)
	
	# Update velocity based on forces
	_update_velocity(delta)
	
	# Calculate derived values
	_calculate_rpm_from_speed()
	_check_gear_shifts(delta)
	_update_slip_angles()
	_update_drift_state(delta)

func _apply_drive_force(delta: float) -> void:
	# Get drive force from powertrain
	var engine_force: float = 0.0
	
	if not is_shifting:
		engine_force = _calculate_engine_force()
	
	# Distribute force to driven wheels
	var drivetrain_type: String = "AWD" if _powertrain and _powertrain.drivetrain_type == "AWD" else "RWD"
	
	if drivetrain_type == "RWD":
		wheel_forces[WHEEL_RL] = engine_force * _get_wheel_distribution_ratio(0.6)
		wheel_forces[WHEEL_RR] = engine_force * _get_wheel_distribution_ratio(0.4)
	elif drivetrain_type == "FWD":
		wheel_forces[WHEEL_FL] = engine_force * _get_wheel_distribution_ratio(0.6)
		wheel_forces[WHEEL_FR] = engine_force * _get_wheel_distribution_ratio(0.4)
	else:  # AWD
		wheel_forces[WHEEL_FL] = engine_force * 0.25
		wheel_forces[WHEEL_FR] = engine_force * 0.25
		wheel_forces[WHEEL_RL] = engine_force * 0.25
		wheel_forces[WHEEL_RR] = engine_force * 0.25

func _calculate_engine_force() -> float:
	if not _powertrain:
		return 0.0
	
	var torque: float = _powertrain.get_current_torque(_rpm)
	var gear_ratio: float = _powertrain.get_current_gear_ratio(current_gear)
	var final_drive: float = _powertrain.final_drive_ratio
	var wheel_radius: float = wheel_radii[0]
	var efficiency: float = _powertrain.drivetrain_efficiency
	
	var wheel_torque: float = torque * gear_ratio * final_drive * efficiency
	var force: float = wheel_torque / wheel_radius
	
	# Apply throttle curve
	force *= _throttle_input
	
	# Apply clutch engagement
	if _clutch_input > 0.1:
		force *= lerp(0.0, 1.0, _clutch_input)
	
	return force

func _apply_brake_force(delta: float) -> void:
	var braking_force: float = 0.0
	
	if _brake_input > 0.0:
		# Main brake force
		braking_force = _brake_input * _get_max_brake_force()
		
		# Handbrake adds extra rear brake force for drifting
		if _handbrake_input > 0.0:
			braking_force *= (1.0 + _handbrake_input * 0.5)
			
			# Apply additional handbrake force to rear wheels only
			wheel_forces[WHEEL_RL] -= _handbrake_input * _get_max_brake_force() * 0.3
			wheel_forces[WHEEL_RR] -= _handbrake_input * _get_max_brake_force() * 0.3
	
	# Apply brakes to all wheels
	for i in range(NUM_WHEELS):
		wheel_forces[i] -= braking_force * 0.25

func _get_max_brake_force() -> float:
	return 8000.0  # Newtons per wheel max

func _apply_steering(delta: float) -> void:
	# Limit steering based on speed (faster = less steering)
	var steering_limit: float = 0.5  # radians max
	var speed_factor: float = 1.0 - min(current_speed / max_speed, 1.0)
	steering_limit *= lerp(0.3, 1.0, speed_factor)
	
	# Apply steering to front wheels
	var steer_angle: float = _steering_input * steering_limit
	steer_angle *= _get_tire_grip_modifier()
	
	wheel_rotation_angles[WHEEL_FL] = steer_angle
	wheel_rotation_angles[WHEEL_FR] = -steer_angle  # Opposite direction

func _get_tire_grip_modifier() -> float:
	var avg_temp: float = (tire_temperature[0] + tire_temperature[1]) / 2.0
	# Optimal temperature around 90C
	if avg_temp < 60.0:
		return 0.5
	elif avg_temp > 110.0:
		return 0.7
	return 1.0

func _apply_aerodynamic_drag(delta: float) -> void:
	# Simple drag model: F = 0.5 * rho * v^2 * Cd * A
	var air_density: float = 1.225  # kg/m^3 at sea level
	var drag_coefficient: float = 0.30  # Typical sports car
	var frontal_area: float = 2.2  # m^2
	
	var drag_force: float = 0.5 * air_density * current_speed * current_speed * drag_coefficient * frontal_area
	
	# Apply opposite to velocity direction
	var drag_vector: Vector3 = velocity.normalized() * -drag_force
	add_force(drag_vector)

func _apply_gravity(delta: float) -> void:
	# Gravity is handled by Godot physics system
	# But we apply additional load transfer effects
	pass

func _update_velocity(delta: float) -> void:
	# Sum all wheel forces
	var total_force: Vector3 = Vector3.ZERO
	
	for i in range(NUM_WHEELS):
		var world_pos: Vector3 = global_transform.origin + global_transform.basis * wheel_positions[i]
		var force_direction: Vector3 = global_transform.basis.z  # Car's forward direction
		
		# Adjust for steering angle
		if i < 2:  # Front wheels
			force_direction = force_direction.rotated(Vector3.UP, wheel_rotation_angles[i])
		
		total_force += force_direction * wheel_forces[i]
	
	# Apply forces to rigid body
	if is_instance_valid(get_parent()) and get_parent() is RigidBody3D:
		get_parent().apply_central_impulse(total_force * delta)
	
	# Calculate current speed magnitude
	current_speed = velocity.length()
	
	# Update longitudinal acceleration
	longitudinal_acceleration = (current_speed - _last_speed_magnitude) / delta
	_last_speed_magnitude = current_speed
	
	# Clamp speeds
	if current_speed > max_speed:
		current_speed = max_speed
		velocity.x = sign(velocity.x) * max_speed
	
	if current_speed < 0.0 and abs(current_speed) > reverse_speed:
		current_speed = reverse_speed
		velocity.x = sign(velocity.x) * reverse_speed
	
	# Emit signal
	emit_signal("speed_changed", current_speed)

var _last_speed_magnitude: float = 0.0

func _update_suspension(delta: float) -> void:
	# Simulate spring-damper suspension system
	for i in range(NUM_WHEELS):
		var wheel_ground_dist: float = wheel_radii[i]
		var compression_target: float = wheel_ground_dist
		var current_compression: float = suspension_compression[i]
		
		# Spring force calculation
		var spring_stiffness: float = _physics_settings.suspension_stiffness if _physics_settings else 50000.0
		var damping_coefficient: float = _physics_settings.suspension_damping if _physics_settings else 5000.0
		
		var force: float = -spring_stiffness * current_compression - damping_coefficient * (current_compression - _last_compression[i])
		
		# Apply to chassis tilt
		if i == WHEEL_FL or i == WHEEL_FR:
			chassis_tilt_z = lerp(chassis_tilt_z, force * 0.0001, 0.1)
		else:
			chassis_tilt_x = lerp(chassis_tilt_x, force * 0.0001, 0.1)
		
		_last_compression[i] = current_compression

var _last_compression: Array[float] = [0.0, 0.0, 0.0, 0.0]

func _update_chassis(delta: float) -> void:
	# Apply chassis tilts for visual effect
	var chassis_transform: Transform3D = global_transform
	chassis_transform.basis = chassis_transform.basis.scaled(Vector3(1.0, 1.0, 1.0))
	
	# Rotate based on suspension compression
	var x_rotation: float = -chassis_tilt_x
	var z_rotation: float = -chassis_tilt_z
	
	global_transform.basis = global_transform.basis.rotated(Vector3.X_AXIS, x_rotation)
	global_transform.basis = global_transform.basis.rotated(Vector3.Z_AXIS, z_rotation)

func _handle_collision_detection(delta: float) -> void:
	# Check for significant impacts
	if velocity.length_squared() > 100.0:
		var impact_force: float = velocity.length()
		
		if impact_force > 20.0:
			damage_level = min(damage_level + impact_force * 0.001, 1.0)
			body_health = max(body_health - impact_force * 0.5, 0.0)
			
			emit_signal("collision_detected", {
				"force": impact_force,
				"damage": damage_level,
				"health": body_health
			})

# ============================================================================
# RPM AND GEAR LOGIC
# ============================================================================

func _calculate_rpm_from_speed() -> void:
	if current_gear == 0:  # Neutral
		_rpm = idle_rpm
		return
	
	var wheel_radius: float = wheel_radii[0]
	var gear_ratio: float = _powertrain.get_current_gear_ratio(current_gear) if _powertrain else 1.0
	var final_drive: float = _powertrain.final_drive_ratio if _powertrain else 3.45
	
	# Calculate wheel RPM from speed
	var wheel_rpm: float = (current_speed * 60.0) / (2.0 * PI * wheel_radius)
	
	# Engine RPM = wheel RPM * gear ratio * final drive
	_rpm = wheel_rpm * gear_ratio * final_drive
	
	# Ensure RPM stays within bounds
	_rpm = clamp(_rpm, idle_rpm, rev_limiter_active ? _powertrain.rev_limit_rpm if _powertrain else 8000.0 : redline_rpm)
	
	emit_signal("rpm_changed", _rpm)

func _check_gear_shifts(delta: float) -> void:
	if is_shifting:
		gear_shift_progress += delta / shift_duration
		if gear_shift_progress >= 1.0:
			is_shifting = false
			gear_shift_progress = 0.0
			current_gear = target_gear
			target_gear = 0
			emit_signal("gear_changed", target_gear, current_gear)
		return
	
	# Auto-shift logic
	if _throttle_input > 0.1:
		_auto_shift_up()
	else:
		_auto_shift_down()

func _auto_shift_up() -> void:
	if current_gear >= 6:
		return
	
	if _rpm >= redline_rpm - 500.0:
		target_gear = current_gear + 1
		is_shifting = true
		gear_shift_progress = 0.0

func _auto_shift_down() -> void:
	if current_gear <= 1:
		return
	
	if _rpm <= idle_rpm + 200.0:
		target_gear = current_gear - 1
		is_shifting = true
		gear_shift_progress = 0.0

func _on_powertrain_rpm_changed(rpm: float) -> void:
	_rpm = rpm
	emit_signal("rpm_changed", rpm)

func _on_powertrain_gear_changed(old_gear: int, new_gear: int) -> void:
	current_gear = new_gear
	emit_signal("gear_changed", old_gear, new_gear)

# ============================================================================
# SLIP AND DRIFT CALCULATIONS
# ============================================================================

func _update_slip_angles() -> void:
	# Calculate lateral slip based on turning and speed
	if current_speed > 1.0:
		var turn_rate: float = rotation_velocity
		var slip_calc: float = (turn_rate * wheel_base) / (2.0 * current_speed)
		
		slip_angle = clamp(slip_calc, -0.5, 0.5)
	else:
		slip_angle = 0.0

func _update_drift_state(delta: float) -> void:
	var prev_drift: bool = is_drifting
	
	if abs(slip_angle) > drift_threshold and _handbrake_input > 0.1:
		is_drifting = true
		if not prev_drift:
			emit_signal("drift_started")
		
		# Increase drift intensity
		drift_intensity = min(drift_intensity + delta * 2.0, 1.0)
		
		# Reduce grip during drift
		grip_level = 0.3 + drift_intensity * 0.2
	else:
		is_drifting = false
		drift_intensity = max(drift_intensity - delta * 3.0, 0.0)
		
		if prev_drift:
			emit_signal("drift_ended")
		
		# Restore grip
		grip_level = lerp(grip_level, 1.0, delta * 2.0)

func _get_wheel_distribution_ratio(front_weight: float) -> float:
	# Weight distribution affects traction
	return front_weight

# ============================================================================
# GAMEPLAY UPDATES
# ============================================================================

func _update_gameplay(delta: float) -> void:
	if is_race_active:
		_update_race_timer(delta)
		_update_lap_timing(delta)

func _update_race_timer(delta: float) -> void:
	total_race_time += delta

func _update_lap_timing(delta: float) -> void:
	current_lap_time += delta
	
	# Check for lap completion (simplified - would normally use checkpoints)
	if current_lap_time > 60.0:  # Assume 60 second minimum lap
		lap_count += 1
		checkpoint_times.append(current_lap_time)
		
		if current_lap_time < best_lap_time:
			best_lap_time = current_lap_time
		
		current_lap_time = 0.0
		
		emit_signal("lap_completed", {
			"lap_number": lap_count,
			"time": current_lap_time,
			"best_lap": best_lap_time
		})

func _update_display(delta: float) -> void:
	# Update HUD data through GameManager
	if GameManager.current_state == GameManager.GameState.RACE_ACTIVE:
		GameManager._race_data["vehicle_speed"] = current_speed
		GameManager._race_data["vehicle_rpm"] = _rpm
		GameManager._race_data["vehicle_gear"] = current_gear
		GameManager._race_data["vehicle_drift"] = is_drifting

# ============================================================================
# INPUT SETTERS (validate and store)
# ============================================================================

func _set_throttle_input(value: float) -> void:
	_last_throttle = _throttle_input
	_throttle_input = clamp(value, 0.0, 1.0)
	if _last_throttle != _throttle_input:
		emit_signal("throttle_change", _throttle_input, _last_throttle)

func _set_brake_input(value: float) -> void:
	_last_brake = _brake_input
	_brake_input = clamp(value, 0.0, 1.0)

func _set_steering_input(value: float) -> void:
	_last_steering = _steering_input
	_steering_input = clamp(value, -1.0, 1.0)

func _set_clutch_input(value: float) -> void:
	_clutch_input = clamp(value, 0.0, 1.0)

func _set_handbrake_input(value: float) -> void:
	_handbrake_input = clamp(value, 0.0, 1.0)

# ============================================================================
# PUBLIC API
# ============================================================================

func reset_vehicle() -> void:
	current_speed = 0.0
	_rpm = idle_rpm
	current_gear = 1
	target_gear = 0
	is_shifting = false
	gear_shift_progress = 0.0
	damage_level = 0.0
	body_health = 100.0
	is_drifting = false
	drift_intensity = 0.0
	grip_level = 1.0
	lap_count = 0
	current_lap_time = 0.0
	best_lap_time = 9999.0
	total_race_time = 0.0
	velocity = Vector3.ZERO

func start_race() -> void:
	is_race_active = true
	reset_vehicle()

func end_race(position: int = 1) -> void:
	is_race_active = false
	emit_signal("race_finished", position, total_race_time)

func get_vehicle_stats() -> Dictionary:
	return {
		"speed": current_speed,
		"rpm": _rpm,
		"gear": current_gear,
		"throttle": _throttle_input,
		"brake": _brake_input,
		"steering": _steering_input,
		"is_drifting": is_drifting,
		"damage": damage_level,
		"health": body_health,
		"lap_count": lap_count,
		"best_lap": best_lap_time
	}

func adjust_gear_manually(direction: int) -> void:
	"""Manually change gear (+1 or -1)"""
	if is_shifting:
		return
	
	var new_gear: int = current_gear + direction
	
	if new_gear < -1:
		new_gear = -1
	elif new_gear > 6:
		new_gear = 6
	elif new_gear == 0:
		new_gear = 1  # Can't stay in neutral when moving
	
	if new_gear != current_gear:
		target_gear = new_gear
		is_shifting = true
		gear_shift_progress = 0.0

func set_all_wheels_together(wheel_index: int, value: float) -> void:
	"""Set all wheel parameters for debugging/testing"""
	for i in range(NUM_WHEELS):
		wheel_forces[i] = value

func get_wheel_data(index: int) -> Dictionary:
	if index < 0 or index >= NUM_WHEELS:
		return {}
	
	return {
		"position": wheel_positions[index],
		"radius": wheel_radii[index],
		"force": wheel_forces[index],
		"angle": wheel_rotation_angles[index],
		"temperature": tire_temperature[index],
		"compression": suspension_compression[index]
	}

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================

func _get_tire_grip_modifier() -> float:
	return grip_level

func is_moving() -> bool:
	return abs(current_speed) > 0.5

func is_in_gear() -> bool:
	return current_gear != 0

func can_shift() -> bool:
	return not is_shifting and current_gear != 0

func get_distance_traveled() -> float:
	# Accumulated distance since reset
	return current_lap_time * current_speed if current_speed > 0 else 0.0

</script>>