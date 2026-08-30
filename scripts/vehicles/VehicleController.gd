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
signal engine_stalled()

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
# POWERTRAIN REFERENCE
# ============================================================================

var powertrain: Node = null

# ============================================================================
# DRIFT AND TRACTION CONTROL
# ============================================================================

var is_drifting: bool = false
var drift_threshold: float = 0.3  # Slip angle threshold for drift
var drift_intensity: float = 0.0
var traction_control_active: bool = true
var stability_control_active: bool = true

# ============================================================================
# COLLISION AND DAMAGE
# ============================================================================

var total_damage: float = 0.0
var structural_integrity: float = 1.0
var last_collision_time: float = 0.0
var collision_impact_force: float = 0.0

# ============================================================================
# RACE DATA
# ============================================================================

var lap_times: Array[float] = []
var best_lap_time: float = 999999.0
var current_lap_start_time: float = 0.0
var lap_count: int = 0
var checkpoints_passed: Array[int] = []
var current_checkpoint: int = 0

# ============================================================================
# TIRE PARAMETERS
# ============================================================================

var tire_friction_coefficient: float = 1.2
var tire_slip_max: float = 0.3
var tire_normal_force: Array[float] = [2000.0, 2000.0, 2000.0, 2000.0]

# ============================================================================
# AERODYNAMICS
# ============================================================================

var aerodynamic_drag_coefficient: float = 0.32
var aerodynamic_area: float = 2.2  # m^2
var downforce_coefficient: float = 0.8
var air_density: float = 1.225  # kg/m^3
var current_downforce: float = 0.0
var drag_force: float = 0.0

# ============================================================================
# INTERNAL TIMING
# ============================================================================

var _shift_timer: float = 0.0
var _drift_timer: float = 0.0
var _engine_sound_level: float = 0.0
var _movement_accumulator: float = 0.0
const _physics_step: float = 1.0 / 120.0

# ============================================================================
# GETTERS AND SETTERS
# ============================================================================

func get_rpm() -> float:
	return _rpm

func get_current_gear() -> int:
	return current_gear

func get_speed_kmh() -> float:
	return current_speed * 3.6

func get_engine_torque() -> float:
	if powertrain:
		return powertrain.get_torque(_rpm)
	return 0.0

func get_power_output() -> float:
	if powertrain:
		return powertrain.get_power(_rpm)
	return 0.0

func _set_throttle_input(value: float) -> void:
	_throttle_input = clamp(value, -1.0, 1.0)

func _set_brake_input(value: float) -> void:
	_brake_input = clamp(value, 0.0, 1.0)

func _set_steering_input(value: float) -> void:
	_steering_input = clamp(value, -1.0, 1.0)

func _set_clutch_input(value: float) -> void:
	_clutch_input = clamp(value, 0.0, 1.0)

func _set_handbrake_input(value: float) -> void:
	_handbrake_input = clamp(value, 0.0, 1.0)

# ============================================================================
# INITIALIZATION
# ============================================================================

func _ready() -> void:
	_init_wheel_positions()
	_connect_signals()
	_update_gear_ratios()

func _init_wheel_positions() -> void:
	# Define wheel positions relative to vehicle center (in meters)
	wheel_positions = [
		Vector3(1.4, -0.4, 0.8),   # Front Left
		Vector3(1.4, 0.4, 0.8),    # Front Right
		Vector3(-1.4, -0.4, 0.8),  # Rear Left
		Vector3(-1.4, 0.4, 0.8)    # Rear Right
	]

func _connect_signals() -> void:
	if GameManager:
		GameManager.game_state_changed.connect(_on_game_state_changed)

func _on_game_state_changed(new_state: GameManager.GameState) -> void:
	if new_state == GameManager.GameState.RACE_ACTIVE:
		current_lap_start_time = Time.get_ticks_msec() / 1000.0
		lap_times.clear()
		lap_count = 0
		checkpoints_passed.clear()
		current_checkpoint = 0

# ============================================================================
# MAIN PROCESS
# ============================================================================

func _physics_process(delta: float) -> void:
	_movement_accumulator += delta
	
	while _movement_accumulator >= _physics_step:
		_physics_step_internal(_physics_step)
		_movement_accumulator -= _physics_step
	
	_movement_accumulator = fposmod(_movement_accumulator, _physics_step)
	
	_update_audio_levels(delta)

func _physics_step_internal(dt: float) -> void:
	_update_inputs()
	_calculate_gear_ratios()
	_update_engine_rpm()
	_apply_drive_forces()
	_apply_brake_forces()
	_apply_steering()
	_calculate_aerodynamics()
	_handle_drift()
	_apply_traction_control()
	_update_vehicle_position(dt)
	_check_collisions()
	_update_race_data()

# ============================================================================
# INPUT HANDLING
# ============================================================================

func _update_inputs() -> void:
	_last_throttle = _throttle_input
	_last_brake = _brake_input
	_last_steering = _steering_input

func _update_inputs_from_manager() -> void:
	if InputManager:
		_throttle_input = InputManager.get_action_value("vehicle_throttle")
		_brake_input = InputManager.get_action_value("vehicle_brake")
		_steering_input = InputManager.get_action_value("vehicle_steering")
		_clutch_input = InputManager.get_action_value("vehicle_clutch")
		_handbrake_input = InputManager.get_action_value("vehicle_handbrake")

# ============================================================================
# GEAR SHIFTER LOGIC
# ============================================================================

func _calculate_gear_ratios() -> void:
	var gear_ratios: Array[float] = [0.0, 3.8, 2.2, 1.5, 1.1, 0.85, 0.65, -3.5]
	var final_drive: float = 3.73
	
	if current_gear < gear_ratios.size():
		var gear_ratio: float = gear_ratios[current_gear]
		var drive_ratio: float = gear_ratio * final_drive
		
		# Update powertrain with gear ratio if available
		if powertrain:
			powertrain.set_final_drive_ratio(drive_ratio)

func _update_gear_ratios() -> void:
	# Called when vehicle properties change
	_calculate_gear_ratios()

func auto_shift_gear() -> void:
	if is_shifting or current_gear == 0:
		return
	
	if _rpm >= redline_rpm:
		target_gear = current_gear - 1 if current_gear > 1 else 0
	elif _rpm < idle_rpm * 1.3 and current_gear > 0:
		target_gear = current_gear - 1
	else:
		target_gear = current_gear + 1
	
	if target_gear != current_gear:
		request_gear_shift(target_gear)

func request_gear_shift(new_gear: int) -> void:
	if is_shifting:
		target_gear = new_gear
		return
	
	if new_gear == current_ggear:
		return
	
	if new_gear < -1 or new_gear > 6:
		return
	
	# Validate gear transition
	if new_gear == 0 and current_gear == 0:
		return
	
	if abs(new_gear - current_gear) > 2:
		# Overshift prevention - shift through intermediate gears
		var direction: int = sign(new_gear - current_gear)
		for g in range(current_gear + direction, new_gear + direction, direction):
			if g != 0:
				target_gear = g
				break
		return
	
	current_gear = new_gear
	is_shifting = true
	gear_shift_progress = 0.0
	_shift_timer = shift_duration
	
	emit_signal("gear_changed", current_gear, new_gear)

func _handle_gear_shift(dt: float) -> void:
	if not is_shifting:
		return
	
	_shift_timer -= dt
	gear_shift_progress = 1.0 - (_shift_timer / shift_duration)
	
	if _shift_timer <= 0.0:
		is_shifting = false
		gear_shift_progress = 0.0
		_shift_timer = 0.0

# ============================================================================
# ENGINE RPM CALCULATION
# ============================================================================

func _update_engine_rpm() -> void:
	var wheel_linear_speed: float = abs(current_speed)
	var wheel_radius: float = wheel_radii[WHEEL_RL]
	var wheel_angular_speed: float = wheel_linear_speed / wheel_radius
	
	var gear_ratio: float = 0.0
	var gear_list: Array[float] = [0.0, 3.8, 2.2, 1.5, 1.1, 0.85, 0.65, -3.5]
	var final_drive: float = 3.73
	
	if current_gear < gear_list.size():
		gear_ratio = gear_list[current_gear] * final_drive
	
	# Calculate RPM based on wheel speed and gear ratio
	var base_rpm: float = wheel_angular_speed * gear_ratio * 60.0 / (2.0 * PI)
	
	if current_gear > 0:
		_rpm = max(base_rpm, idle_rpm)
	elif current_gear < 0:
		_rpm = min(base_rpm * -1, idle_rpm)
	else:
		_rpm = idle_rpm
	
	# Apply rev limiter
	if _rpm >= redline_rpm:
		rev_limiter_active = true
		_rpm = lerp(_rpm, redline_rpm * 0.98, 0.1)
	else:
		rev_limiter_active = false
	
	# Smooth RPM transitions
	_rpm = lerp(_rpm, _rpm, 0.9)
	
	emit_signal("rpm_changed", _rpm)

# ============================================================================
# DRIVE FORCE APPLICATION
# ============================================================================

func _apply_drive_forces() -> void:
	var drive_force: float = 0.0
	var torque: float = 0.0
	
	if powertrain:
		torque = powertrain.get_torque(_rpm)
		drive_force = torque * 0.05  # Scale factor for force application
	
	# Apply clutch modulation
	drive_force *= (1.0 - _clutch_input)
	
	# Distribute drive force to rear wheels (RWD layout)
	wheel_forces[WHEEL_RL] = drive_force * 0.5
	wheel_forces[WHEEL_RR] = drive_force * 0.5
	
	# Adjust for handbrake
	if _handbrake_input > 0.5:
		wheel_forces[WHEEL_RL] *= 0.5
		wheel_forces[WHEEL_RR] *= 0.5
	
	# Apply wheel spin control
	if traction_control_active and _throttle_input > 0.8:
		var spin_factor: float = _get_wheel_spin_factor()
		wheel_forces[WHEEL_RL] *= spin_factor
		wheel_forces[WHEEL_RR] *= spin_factor

func _get_wheel_spin_factor() -> float:
	var rear_speed: float = (wheel_forces[WHEEL_RL] + wheel_forces[WHEEL_RR]) / 2
	var current_velocity: float = velocity.length()
	
	if current_velocity > 0.1:
		var slip: float = abs(rear_speed / current_velocity - 1.0)
		if slip > tire_slip_max:
			return 1.0 - ((slip - tire_slip_max) / 0.2)
	
	return 1.0

# ============================================================================
# BRAKE FORCE APPLICATION
# ============================================================================

func _apply_brake_forces() -> void:
	var brake_force: float = 0.0
	
	if _brake_input > 0.0:
		var max_brake_force: float = 25000.0  # Newtons
		brake_force = max_brake_force * _brake_input
		
		# ABS prevents lockup
		if traction_control_active:
			var speed_per_wheel: float = current_speed / 4.0
			var wheel_lock_threshold: float = speed_per_wheel * 0.2
			
			for i in range(NUM_WHEELS):
				if wheel_forces[i] < wheel_lock_threshold:
					wheel_forces[i] = wheel_lock_threshold
	
	if _handbrake_input > 0.0:
		# Handbrake applies only to rear wheels
		var handbrake_force: float = 15000.0 * _handbrake_input
		wheel_forces[WHEEL_RL] -= handbrake_force
		wheel_forces[WHEEL_RR] -= handbrake_force

# ============================================================================
# STEERING MECHANICS
# ============================================================================

func _apply_steering() -> void:
	var steer_angle: float = _steering_input * 0.5  # Max ~28 degrees
	
	# Apply steering to front wheels only
	wheel_rotation_angles[WHEEL_FL] = steer_angle
	wheel_rotation_angles[WFR] = -steer_angle
	
	# Anti-lock steering (prevent oversteering at high speeds)
	if abs(current_speed) > 30.0 and stability_control_active:
		steer_angle *= 0.7
	
	rotation_velocity = steer_angle * 3.0 * current_speed / 20.0

func _get_turning_rate() -> float:
	return rotation_velocity

# ============================================================================
# AERODYNAMICS CALCULATION
# ============================================================================

func _calculate_aerodynamics() -> void:
	var speed_squared: float = current_speed * current_speed
	
	# Drag force: F = 0.5 * rho * v^2 * Cd * A
	drag_force = 0.5 * air_density * speed_squared * aerodynamic_drag_coefficient * aerodynamic_area
	
	# Downforce: proportional to v^2
	current_downforce = 0.5 * air_density * speed_squared * downforce_coefficient * aerodynamic_area

# ============================================================================
# DRIFT MECHANICS
# ============================================================================

func _handle_drift() -> void:
	slip_angle = _calculate_slip_angle()
	
	if abs(slip_angle) > drift_threshold and _handbrake_input > 0.3:
		if not is_drifting:
			is_drifting = true
			emit_signal("drift_started")
		
		_drift_timer += 1.0
		# Increase drift intensity based on duration
		Drift_intensity = min(_drift_timer * 0.05, 1.0)
		
		# Apply drift friction reduction
		tire_friction_coefficient = 1.2 * (1.0 - drift_intensity * 0.7)
	else:
		if is_drifting:
			is_drifting = false
			emit_signal("drift_ended")
		
		_drift_timer = 0.0
		Drift_intensity = 0.0
		tire_friction_coefficient = 1.2

func _calculate_slip_angle() -> float:
	var velocity_vector: Vector3 = velocity.normalized()
	var forward_vector: Vector3 = transform.basis.z * -1
	var heading: float = velocity_vector.angle_to(forward_vector)
	
	return heading

# ============================================================================
# TRACTION CONTROL
# ============================================================================

func _apply_traction_control() -> void:
	if not traction_control_active:
		return
	
	# Check for excessive wheel slip
	var drive_slip: float = _calculate_drive_slip()
	
	if drive_slip > 0.3:
		# Reduce engine torque
		if powertrain:
			powertrain.reduce_torque(drive_slip * 0.5)
		
		# Apply braking to slipping wheel
		var slip_index: int = _get_most_slipping_wheel()
		wheel_forces[slip_index] *= 0.5

func _calculate_drive_slip() -> float:
	var rear_wheel_speed: float = (wheel_forces[WHEEL_RL] + wheel_forces[WHEEL_RR]) / 2
	var actual_speed: float = current_speed
	
	if actual_speed > 0.1:
		return abs(rear_wheel_speed / actual_speed - 1.0)
	
	return 0.0

func _get_most_slipping_wheel() -> int:
	return WHEEL_RL if randf() < 0.5 else WHEEL_RR

# ============================================================================
# MOVEMENT UPDATE
# ============================================================================

func _update_vehicle_position(dt: float) -> void:
	var total_force_x: float = wheel_forces[WHEEL_FL] + wheel_forces[WHEEL_FR]
	var total_force_y: float = wheel_forces[WHEEL_RL] + wheel_forces[WHEEL_RR]
	
	# Apply horizontal movement
	var move_delta: Vector3 = Vector3.ZERO
	
	move_delta.x = total_force_x * dt * 0.001
	move_delta.y = total_force_y * dt * 0.001
	
	# Apply yaw rotation
	var yaw_change: float = rotation_velocity * dt
	transform.basis = transform.basis.rotated(Vector3.UP, yaw_change)
	
	# Move vehicle
	move_and_slide(move_delta)
	
	# Update speed from velocity
	var ground_velocity: Vector3 = velocity.project_on_plane(Vector3.UP)
	current_speed = ground_velocity.length()
	longitudinal_acceleration = (current_speed - _last_throttle) / dt
	
	# Clamp speed limits
	if current_speed > max_speed:
		current_speed = max_speed
		velocity.x *= max_speed / current_speed
	elif current_speed > reverse_speed and _throttle_input < 0:
		current_speed = reverse_speed
		velocity.x *= reverse_speed / current_speed

func _update_audio_levels(delta: float) -> void:
	_engine_sound_level = lerp(_engine_sound_level, _rpm / redline_rpm, delta * 2.0)
	
	if AudioManager and AudioManager.sound_played.is_connected(_on_engine_sound_updated):
		AudioManager.update_engine_sound(_engine_sound_level)

# ============================================================================
# COLLISION DETECTION
# ============================================================================

func _check_collisions() -> void:
	for i in range(get_collision_count()):
		var collision: KinematicCollision3D = get_collision(i)
		
		collision_impact_force = collision.get_normal().length() * current_speed * 10.0
		
		if collision_impact_force > 5000.0:
			total_damage += collision_impact_force / 10000.0
			structural_integrity = max(0.0, 1.0 - total_damage)
			
			var collision_info: Dictionary = {
				"impact_force": collision_impact_force,
				"collision_point": collision.get_position(),
				"normal": collision.get_normal(),
				"time": Time.get_ticks_msec() / 1000.0
			}
			
			emit_signal("collision_detected", collision_info)
			last_collision_time = Time.get_ticks_msec() / 1000.0

# ============================================================================
# RACE DATA MANAGEMENT
# ============================================================================

func _update_race_data() -> void:
	if GameManager.current_state != GameManager.GameState.RACE_ACTIVE:
		return
	
	var current_time: float = Time.get_ticks_msec() / 1000.0
	
	# Lap timing
	if lap_count > 0:
		var lap_elapsed: float = current_time - current_lap_start_time
		lap_times.append(lap_elapsed)
		
		if lap_elapsed < best_lap_time:
			best_lap_time = lap_elapsed
		
		current_lap_start_time = current_time
		lap_count += 1

func get_best_lap() -> float:
	return best_lap_time

func get_average_lap() -> float:
	if lap_times.is_empty():
		return 0.0
	var sum: float = 0.0
	for t in lap_times:
		sum += t
	return sum / lap_times.size()

func get_current_lap_number() -> int:
	return lap_count

# ============================================================================
# VEHICLE RESET
# ============================================================================

func reset_vehicle() -> void:
	position = Vector3.ZERO
	velocity = Vector3.ZERO
	rotation = Vector3.ZERO
	
	current_speed = 0.0
	current_gear = 0
	target_gear = 0
	_rpm = idle_rpm
	is_shifting = false
	
	total_damage = 0.0
	structural_integrity = 1.0
	is_drifting = false
	Drift_intensity = 0.0
	
	wheel_forces.fill(0.0)
	wheel_rotation_angles.fill(0.0)

func respawn_at_position(pos: Vector3) -> void:
	position = pos
	velocity = Vector3.ZERO
	current_speed = 0.0
	current_gear = 0
	_rpm = idle_rpm

# ============================================================================
# DEBUG INFO
# ============================================================================

func get_debug_dictionary() -> Dictionary:
	return {
		"speed": current_speed,
		"speed_kmh": get_speed_kmh(),
		"rpm": _rpm,
		"gear": current_gear,
		"throttle": _throttle_input,
		"brake": _brake_input,
		"steering": _steering_input,
		"is_drifting": is_drifting,
		"damage": total_damage,
		"integrity": structural_integrity,
		"lap": lap_count,
		"best_lap": best_lap_time
	}

func print_debug_info() -> void:
	print_debug_dictionary(get_debug_dictionary())