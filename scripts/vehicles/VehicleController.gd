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

# ============================================================================
# DRIFT & TRACTION CONTROL
# ============================================================================

var drift_coefficient: float = 0.0
var traction_control_active: bool = false
var anti_lock_braking_active: bool = false
var tire_friction_coefficient: float = 1.2
var surface_friction: float = 1.0

# ============================================================================
# RACE DATA TRACKING
# ============================================================================

var distance_traveled: float = 0.0
var last_checkpoint_distance: float = 0.0
var total_laps: int = 0
var current_lap_time: float = 0.0
var best_lap_time: float = 9999.99

# ============================================================================
# POWERTRAIN REFERENCE
# ============================================================================

var powertrain: Node = null

func _ready() -> void:
	_init_wheel_positions()
	_connect_signals()
	_setup_collision_detection()
	
	if GameManager != null:
		GameManager.game_state_changed.connect(_on_game_state_changed)
	
	powertrain = get_parent() if has_node("../Powertrain") else null

func _init_wheel_positions() -> void:
	# Wheel positions relative to vehicle center (meters)
	wheel_positions.resize(NUM_WHEELS)
	
	# Track width and wheelbase dimensions
	var track_width: float = 1.6
	var wheelbase: float = 2.8
	
	# Front wheels (positive X is forward)
	wheel_positions[WHEEL_FL] = Vector3(-track_width / 2, -0.33, wheelbase / 2)
	wheel_positions[WHEEL_FR] = Vector3(track_width / 2, -0.33, wheelbase / 2)
	
	# Rear wheels
	wheel_positions[WHEEL_RL] = Vector3(-track_width / 2, -0.33, -wheelbase / 2)
	wheel_positions[WHEEL_RR] = Vector3(track_width / 2, -0.33, -wheelbase / 2)

func _connect_signals() -> void:
	pass  # Signals connected in _ready()

func _setup_collision_detection() -> void:
	pass  # Collision setup handled by Godot's built-in system

func _set_throttle_input(value: float) -> void:
	_throttle_input = clamp(value, -1.0, 1.0)

func _set_brake_input(value: float) -> void:
	_brake_input = clamp(value, -1.0, 1.0)

func _set_steering_input(value: float) -> void:
	_steering_input = clamp(value, -1.0, 1.0)

func _set_clutch_input(value: float) -> void:
	_clutch_input = clamp(value, -1.0, 1.0)

func _set_handbrake_input(value: float) -> void:
	_handbrake_input = clamp(value, -1.0, 1.0)

func _physics_process(delta: float) -> void:
	_update_inputs(delta)
	_calculate_physics(delta)
	_handle_gear_shifting(delta)
	_apply_motor_and_brake(delta)
	_calculate_drift_and_traction(delta)
	_update_vehicle_state(delta)
	_track_race_data(delta)
	_handle_collisions(delta)
	_move_vehicle(delta)

func _update_inputs(delta: float) -> void:
	# Store previous inputs for differential calculations
	_last_throttle = _throttle_input
	_last_brake = _brake_input
	_last_steering = _steering_input
	
	# Smooth input transitions (input lag compensation)
	_throttle_input = lerp(_throttle_input, _last_throttle, 0.9)
	_brake_input = lerp(_brake_input, _last_brake, 0.9)
	_steering_input = lerp(_steering_input, _last_steering, 0.9)

func _calculate_physics(delta: float) -> void:
	# Calculate current speed from velocity
	current_speed = velocity.length() * cos(rotation_degrees.y * PI / 180.0)
	
	# Calculate accelerations (G-forces)
	var dt: float = delta * PhysicsSettings.time_scale
	if dt > 0.0:
		longitudinal_acceleration = (current_speed - _get_previous_speed()) / dt
	else:
		longitudinal_acceleration = 0.0
		
	lateral_acceleration = velocity.x * sin(rotation_degrees.y * PI / 180.0) - 
						   velocity.z * cos(rotation_degrees.y * PI / 180.0)
		
	rotation_velocity = _calculate_yaw_rate()

func _get_previous_speed() -> float:
	return current_speed * 0.95 + velocity.length() * 0.05

func _calculate_yaw_rate() -> float:
	# Calculate yaw rate based on steering input and speed
	var steer_factor: float = abs(_steering_input) * _steering_gain()
	return steer_factor * current_speed / wheelbase_length()

func _steering_gain() -> float:
	# Steering gain decreases at higher speeds for stability
	var speed_factor: float = min(current_speed / 30.0, 1.0)
	return lerp(1.0, 0.5, speed_factor)

func _handle_gear_shifting(delta: float) -> void:
	if not is_shifting:
		_determine_target_gear()
		if current_gear != target_gear:
			_start_gear_shift()
	
	if is_shifting:
		gear_shift_progress += delta / shift_duration
		if gear_shift_progress >= 1.0:
			_complete_gear_shift()

func _determine_target_gear() -> void:
	# Determine optimal gear based on RPM and speed
	var target: int = 1
	
	if current_speed < 5.0:
		target = 1
	elif current_speed < 12.0:
		target = 2
	elif current_speed < 20.0:
		target = 3
	elif current_speed < 30.0:
		target = 4
	elif current_speed < 45.0:
		target = 5
	elif current_speed < 60.0:
		target = 6
	else:
		target = 6
	
	# Consider clutch state
	if _clutch_input > 0.5:
		target = 0  # Neutral when clutch depressed
		current_gear = 0
	else:
		target = max(target, 1)
	
	target_gear = target

func _start_gear_shift() -> void:
	is_shifting = true
	gear_shift_progress = 0.0
	var old_gear: int = current_gear
	current_gear = 0  # Cut power during shift
	gear_changed.emit(old_gear, current_gear)

func _complete_gear_shift() -> void:
	is_shifting = false
	gear_shift_progress = 0.0
	var old_gear: int = current_gear
	current_gear = target_gear
	gear_changed.emit(old_gear, current_gear)
	
	# Ensure we don't go below neutral
	if current_gear < -1 or current_gear > 6:
		current_gear = 0

func _apply_motor_and_brake(delta: float) -> void:
	# Apply motor torque based on throttle and current gear
	var motor_force: float = _calculate_motor_force()
	
	# Distribute force to drive wheels (RWD example)
	wheel_forces[WHEEL_RL] = motor_force * 0.5
	wheel_forces[WHEEL_RR] = motor_force * 0.5
	
	# Brake forces
	var brake_force: float = _brake_input * PhysicsSettings.default_vehicle_mass * 3.0
	
	# Apply brakes to all wheels
	for i in wheel_forces.size():
		wheel_forces[i] -= brake_force * 0.25
	
	# Handbrake only affects rear wheels
	if _handbrake_input > 0.1:
		wheel_forces[WHEEL_RL] -= _handbrake_input * brake_force * 0.5
		wheel_forces[WHEEL_RR] -= _handbrake_input * brake_force * 0.5

func _calculate_motor_force() -> float:
	# Calculate motor force based on gear ratio and throttle
	if current_gear <= 0:
		return 0.0
	
	var gear_ratios: Array[float] = [0.0, 3.8, 2.2, 1.5, 1.1, 0.9, 0.7]
	var final_drive_ratio: float = 3.5
	
	var gear_ratio: float = gear_ratios[current_gear]
	var total_ratio: float = gear_ratio * final_drive_ratio
	
	# Torque curve approximation
	var torque_curve: float = _get_torque_curve_value()
	var motor_torque: float = torque_curve * total_ratio
	
	# Force at wheel = torque / radius
	var wheel_radius: float = wheel_radii[WHEEL_FL]
	var force: float = motor_torque / wheel_radius
	
	# Apply throttle input
	force *= _throttle_input
	
	# Clamp to maximum
	force = clamp(force, -2000.0, 5000.0)
	
	return force

func _get_torque_curve_value() -> float:
	# Simple torque curve approximation (peak at mid RPM)
	var rpm_normalized: float = (_rpm - idle_rpm) / (redline_rpm - idle_rpm)
	rpm_normalized = clamp(rpm_normalized, 0.0, 1.0)
	
	# Torque peaks around 4000 RPM
	var peak_rpm: float = 4000.0
	var rpm_diff: float = abs(_rpm - peak_rpm)
	var torque_dropoff: float = exp(-(rpm_diff / 1500.0) ** 2)
	
	return torque_dropoff * 500.0  # Peak torque ~500 Nm

func _calculate_drift_and_traction(delta: float) -> void:
	# Calculate slip angle based on lateral acceleration
	slip_angle = asin(clamp(lateral_acceleration / 9.81, -1.0, 1.0)) * (180.0 / PI)
	
	# Determine drift state
	var drift_threshold: float = 15.0  # degrees
	var is_drifting: bool = abs(slip_angle) > drift_threshold
	
	if is_drifting and not _is_previously_drifting():
		_drift_started()
	elif not is_drifting and _is_previously_drifting():
		_drift_ended()
	
	# Traction control
	if traction_control_active and _throttle_input > 0.8:
		_apply_traction_control()
	
	# Anti-lock braking
	if anti_lock_braking_active and _brake_input > 0.5:
		_apply_abs()

func _is_previously_drifting() -> bool:
	return abs(slip_angle) > 15.0

func _drift_started() -> void:
	drift_started.emit()
	tire_friction_coefficient = 0.8  # Reduced friction during drift

func _drift_ended() -> void:
	drift_ended.emit()
	tire_friction_coefficient = 1.2  # Normal friction

func _apply_traction_control() -> void:
	# Reduce motor force if wheel slip detected
	if current_speed > 5.0 and _throttle_input > 0.8:
		wheel_forces[WHEEL_RL] *= 0.7
		wheel_forces[WHEEL_RR] *= 0.7

func _apply_abs() -> void:
	# Prevent wheel lockup during hard braking
	var wheel_lock_threshold: float = 0.3
	var front_wheel_speeds: Array[float] = [_get_wheel_speed(WHEEL_FL), _get_wheel_speed(WHEEL_FR)]
	var rear_wheel_speeds: Array[float] = [_get_wheel_speed(WHEEL_RL), _get_wheel_speed(WHEEL_RR)]
	
	for i in range(front_wheel_speeds.size()):
		if front_wheel_speeds[i] < vehicle_speed() * wheel_lock_threshold:
			wheel_forces[i] *= 0.5  # Reduce brake force
		if rear_wheel_speeds[i] < vehicle_speed() * wheel_lock_threshold:
			wheel_forces[i + 2] *= 0.5

func _update_vehicle_state(delta: float) -> void:
	# Update RPM based on gear and speed
	_update_rpm()
	
	# Update acceleration values
	acceleration = _calculate_acceleration()
	deceleration = _calculate_deceleration()
	
	# Clamp speeds
	if current_speed > max_speed and current_gear > 0:
		current_speed = max_speed
	elif current_speed < -reverse_speed and current_gear < 0:
		current_speed = -reverse_speed
	
	# Emit signals
	speed_changed.emit(current_speed)
	rpm_changed.emit(_rpm)

func _update_rpm() -> void:
	if current_gear == 0:
		# Engine idles when in neutral
		_rpm = lerp(_rpm, idle_rpm, 0.1)
	else:
		# Calculate RPM based on gear ratio and wheel speed
		var gear_ratios: Array[float] = [0.0, 3.8, 2.2, 1.5, 1.1, 0.9, 0.7]
		var final_drive_ratio: float = 3.5
		var transmission_efficiency: float = 0.95
		
		var gear_ratio: float = gear_ratios[current_gear]
		var total_ratio: float = gear_ratio * final_drive_ratio * transmission_efficiency
		
		# Wheel circumference
		var wheel_circumference: float = 2.0 * PI * wheel_radii[WHEEL_FL]
		
		# RPM = (speed / circumference) * total_ratio * 60
		var wheel_rps: float = current_speed / wheel_circumference
		_rpm = wheel_rps * total_ratio * 60.0
		
		# Clamp to idle and redline
		_rpm = clamp(_rpm, idle_rpm, redline_rpm)
		
		# Check rev limiter
		if _rpm >= redline_rpm * 0.95:
			rev_limiter_active = true
			_rpm = lerp(_rpm, redline_rpm * 0.95, 0.05)
		else:
			rev_limiter_active = false

func _calculate_acceleration() -> float:
	# Acceleration based on motor force and vehicle mass
	var total_force: float = sum(wheel_forces)
	var mass: float = PhysicsSettings.default_vehicle_mass
	
	if total_force != 0.0:
		return total_force / mass
	return 0.0

func _calculate_deceleration() -> float:
	# Natural deceleration (drag, rolling resistance)
	var drag_coefficient: float = 0.3
	var frontal_area: float = 2.2
	var air_density: float = 1.225
	
	var drag_force: float = 0.5 * drag_coefficient * frontal_area * air_density * current_speed * current_speed
	var rolling_resistance: float = 0.015 * PhysicsSettings.default_vehicle_mass * 9.81
	
	var total_resistance: float = drag_force + rolling_resistance
	return total_resistance / PhysicsSettings.default_vehicle_mass

func _track_race_data(delta: float) -> void:
	# Accumulate distance traveled
	distance_traveled += current_speed * delta
	
	# Lap timing
	current_lap_time += delta
	
	# Check for lap completion (simple distance-based for now)
	var checkpoint_distance: float = 5000.0  # Example checkpoint
	if distance_traveled >= last_checkpoint_distance + checkpoint_distance:
		last_checkpoint_distance = distance_traveled
		total_laps += 1
		
		# Record lap time
		if current_lap_time < best_lap_time:
			best_lap_time = current_lap_time
			
		lap_completed.emit({
			"lap_number": total_laps,
			"time": current_lap_time,
			"best_lap": best_lap_time
		})
		
		current_lap_time = 0.0

func _handle_collisions(delta: float) -> void:
	# Process collision information
	if colliding():
		var collision_info: Dictionary = _gather_collision_info()
		collision_detected.emit(collision_info)
		
		# Apply collision response
		_apply_collision_response(collision_info)

func _gather_collision_info() -> Dictionary:
	var info: Dictionary = {
		"collider": get_collision_collider(),
		"position": global_position,
		"velocity": velocity.duplicate(),
		"normal": get_collision_normal(),
		"time": get_collision_depth()
	}
	
	return info

func _apply_collision_response(info: Dictionary) -> void:
	# Simple bounce effect
	var bounce_factor: float = 0.3
	var impact_velocity: float = info["velocity"].length()
	
	if impact_velocity > 5.0:
		# Reduce velocity based on impact
		velocity *= (1.0 - bounce_factor)
		
		# Screen shake effect
		if AudioManager != null:
			AudioManager.play_sound("vehicle_impact", impact_velocity / 10.0)

func _move_vehicle(delta: float) -> void:
	# Apply calculated forces to vehicle movement
	var total_force_x: float = 0.0
	var total_force_z: float = 0.0
	
	# Sum wheel forces and apply to body
	for i in range(wheel_forces.size()):
		var force_direction: int = 1 if i < 2 else -1  # Front positive, rear negative
		total_force_x += wheel_forces[i] * force_direction
		total_force_z += wheel_forces[i] * 0.0  # Lateral forces minimal
	
	# Convert local forces to world direction
	var forward_vector: Vector3 = transform.basis.z.rotated(Vector3.UP, rotation_degrees.y)
	var right_vector: Vector3 = transform.basis.x.rotated(Vector3.UP, rotation_degrees.y)
	
	# Apply forces
	velocity.x += total_force_x * forward_vector.x * delta
	velocity.z += total_force_x * forward_vector.z * delta
	
	# Steering rotation
	var steer_amount: float = _steering_input * steer_angle_max()
	rotation_degrees.y += steer_amount * delta * 50.0
	
	# Apply gravity
	velocity.y -= PhysicsSettings.gravity * delta
	
	# Apply movement
	move_and_slide()

func steer_angle_max() -> float:
	# Maximum steering angle in degrees
	return 30.0

func vehicle_speed() -> float:
	return current_speed

func wheelbase_length() -> float:
	return 2.8

func _get_wheel_speed(wheel_index: int) -> float:
	# Estimate wheel speed based on vehicle speed and gear
	if current_gear <= 0:
		return 0.0
	
	var gear_ratios: Array[float] = [0.0, 3.8, 2.2, 1.5, 1.1, 0.9, 0.7]
	var final_drive_ratio: float = 3.5
	var wheel_circumference: float = 2.0 * PI * wheel_radii[wheel_index]
	
	var gear_ratio: float = gear_ratios[current_gear]
	var total_ratio: float = gear_ratio * final_drive_ratio
	
	return (current_speed / wheel_circumference) * total_ratio

func _on_game_state_changed(new_state: GameState) -> void:
	match new_state:
		GameState.MAIN_MENU:
			_reset_vehicle()
		GameState.RACE_ACTIVE:
			_start_racing()
		GameState.RACE_PAUSED:
			_pause_racing()

func _reset_vehicle() -> void:
	current_speed = 0.0
	current_gear = 0
	_rpm = idle_rpm
	velocity = Vector3.ZERO
	position = Vector3.ZERO
	rotation_degrees = Vector3.ZERO

func _start_racing() -> void:
	current_speed = 0.0
	current_gear = 1
	_rpm = idle_rpm
	distance_traveled = 0.0
	current_lap_time = 0.0
	best_lap_time = 9999.99
	total_laps = 0

func _pause_racing() -> void:
	pass  # Pause logic handled by game manager

func get_current_speed_kmh() -> float:
	return current_speed * 3.6

func get_current_rpm() -> float:
	return _rpm

func get_current_gear() -> int:
	return current_gear

func reset_all() -> void:
	current_speed = 0.0
	max_speed = 65.0
	reverse_speed = 20.0
	acceleration = 0.0
	deceleration = 0.0
	rotation_velocity = 0.0
	slip_angle = 0.0
	lateral_acceleration = 0.0
	longitudinal_acceleration = 0.0
	current_gear = 0
	target_gear = 0
	gear_shift_progress = 0.0
	is_shifting = false
	_rpm = idle_rpm
	rev_limiter_active = false
	wheel_forces.fill(0.0)
	wheel_rotation_angles.fill(0.0)
	distance_traveled = 0.0
	last_checkpoint_distance = 0.0
	total_laps = 0
	current_lap_time = 0.0
	best_lap_time = 9999.99
	traction_control_active = false
	anti_lock_braking_active = false
	tire_friction_coefficient = 1.2
	surface_friction = 1.0

func _to_string() -> String:
	return "VehicleController:\n" \
		"\tSpeed: %.1f km/h\n" % get_current_speed_kmh() + \
		"\tRPM: %.0f\n" % get_current_rpm() + \
		"\tGear: %d\n" % get_current_gear() + \
		"\tDrift: %s\n" % ("Active" if slip_angle > 15.0 else "Inactive")

</File>>