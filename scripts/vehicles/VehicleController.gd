extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulation
## Handles throttle, brake, steering, wheel forces, gear shifting, and vehicle dynamics
## Copyright 2026 Thalamus Racing Simulator Project

# ============================================================================
# SIGNALS
# ============================================================================
signal speed_changed(new_speed: float)
signal rpm_changed(rpm: int)
signal gear_changed(gear: int)
signal drift_started(drift_angle: float)
signal drift_ended()
signal collision_detected(impact_velocity: Vector3)
signal lap_completed(lap_data: Dictionary)
signal race_event(event_type: String)
signal engine_sound_changed(rpm_ratio: float)
signal suspension_compressed(suspension_amount: float)

# ============================================================================
# CONSTANTS & CONFIGURATION
# ============================================================================
const MAX_ENGINE_RPM: int = 8500
const IDLE_RPM: int = 800
const REDLINE_RPM: int = 7200
const RPM_PER_GEAR: Array[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
const GEAR_RATIOS: Array[float] = [0.0, 3.8f, 2.4f, 1.7f, 1.3f, 1.1f, 0.85f, 0.65f]
const FINAL_DRIVE_RATIO: float = 3.73f
const MAX_REVERSE_SPEED: float = -15.0
const MAX_FORWARD_SPEED: float = 120.0
const DRIFT_THRESHOLD: float = 0.5
const GRIP_LEVEL_NORMAL: float = 0.95
const GRIP_LEVEL_DRIFT: float = 0.35
const STEERING_SPEED: float = 15.0
const BRAKING_FORCE: float = 2.5
const ACCELERATION_FORCE: float = 1.8
const TURNING_RADIUS: float = 8.0

# ============================================================================
# EXPORTED PROPERTIES
# ============================================================================
@export_group("Vehicle Configuration")
@export var vehicle_mass: float = 1500.0
@export var center_of_mass: Vector3 = Vector3(0.0, 0.5, 0.0)
@export var wheelbase: float = 2.5
@export var track_width: float = 1.6
@export var max_steering_angle: float = 0.6 # ~35 degrees

@export_group("Drift Settings")
@export var drift_enabled: bool = true
@export var drift_gr_reduction: float = 0.4
@export var drift_recovery_rate: float = 0.15
@export var drift_force_multiplier: float = 1.2

@export_group("Suspension")
@export var suspension_stiffness: float = 50.0
@export var suspension_damping: float = 2.0
@export var suspension_travel: float = 0.15
@export var wheel_radius: float = 0.32

# ============================================================================
# POWERTRAIN INTEGRATION
# ============================================================================
var _powertrain: Node = null
var _audio_manager_ref: Node = null

# ============================================================================
# INPUT VARIABLES (connected from InputManager)
# ============================================================================
var throttle_input: float = 0.0
var brake_input: float = 0.0
var steering_input: float = 0.0
var clutch_input: float = 0.0
var handbrake_input: float = 0.0
var nitrous_input: bool = false

# ============================================================================
# VEHICLE PHYSICS STATE
# ============================================================================
var current_speed: float = 0.0
var acceleration: float = 0.0
var slip_angle: float = 0.0
var lateral_g: float = 0.0
var longitudinal_g: float = 0.0
var yaw_rate: float = 0.0
var rotation_velocity: float = 0.0

# ============================================================================
# GEARBOX SYSTEM
# ============================================================================
var current_gear: int = 0  # 0 = neutral, -1 = reverse, 1-7 = forward gears
var target_gear: int = 0
var is_shifting: bool = false
var shift_progress: float = 0.0
var shift_timer: float = 0.0
var upshift_rpm_threshold: int = 6500
var downshift_rpm_threshold: int = 2500
var auto_shift_enabled: bool = true
var manual_override: bool = false

# ============================================================================
# WHEEL DATA
# ============================================================================
var front_wheel_angle: float = 0.0
var rear_wheel_angle: float = 0.0
var wheel_rotations: Array[Vector2] = []
var wheel_contact_points: Array[Vector3] = []
var wheel_normal_forces: Array[float] = []

# ============================================================================
# DRIFT & SLIDING STATE
# ============================================================================
var is_drifting: bool = false
var drift_angle_target: float = 0.0
var drift_angle_current: float = 0.0
var drift_score: float = 0.0
var drift_combo: int = 0
var drift_combo_timer: float = 0.0
var drift_max_angle: float = 1.5
var drift_recovery_needed: bool = false

# ============================================================================
# SUSPENSION DATA
# ============================================================================
var front_suspension_compression: float = 0.0
var rear_suspension_compression: float = 0.0
var wheel_load_distribution: Vector4 = Vector4(1.0, 1.0, 1.0, 1.0)

# ============================================================================
# COLLISION DETECTION
# ============================================================================
var last_collision_time: float = 0.0
var collision_impact: Vector3 = Vector3.ZERO
var collision_normal: Vector3 = Vector3.ZERO
var collision_object: Object = null
var collision_count: int = 0

# ============================================================================
# LAP & RACE TRACKING
# ============================================================================
var current_lap: int = 1
var best_lap_time: float = 999.0
var current_lap_time: float = 0.0
var start_time: float = 0.0
var checkpoints_passed: int = 0
var total_distance: float = 0.0

# ============================================================================
# ENGINE STATE
# ============================================================================
var engine_rpm: int = IDLE_RPM
var engine_torque: float = 0.0
var engine_power: float = 0.0
var fuel_level: float = 100.0
var fuel_consumption_rate: float = 0.01
var engine_temperature: float = 85.0
var engine_health: float = 100.0

# ============================================================================
# NITROUS BOOST SYSTEM
# ============================================================================
var nitrous_available: bool = true
var nitrous_fuel: float = 100.0
var nitrous_charge_time: float = 2.0
var nitrous_active_time: float = 0.0
var nitrous_timer: float = 0.0
var nitrous_boost_factor: float = 1.5

# ============================================================================
# CAMERA FOLLOW TARGET
# ============================================================================
var camera_offset: Vector3 = Vector3(0.0, 2.0, -4.0)
var camera_rotation: float = 0.0
var camera_follow_smooth: float = 0.15

# ============================================================================
# DEBUG & VISUALIZATION
# ============================================================================
@export var debug_visualization: bool = false
var debug_lines: Array[Dictionary] = []

# ============================================================================
# INITIALIZATION
# ============================================================================
func _ready() -> void:
	_process_mode = ProcessModeEnum.ALWAYS
	
	# Initialize powertrain reference
	if Engine.has_singleton("Powertrain"):
		_powertrain = Engine.get_singleton("Powertrain")
	else:
		var parent = get_parent()
		if parent:
			_powertrain = parent.find_child("Powertrain", true, false)
	
	# Get audio manager reference
	_audio_manager_ref = GameManager.AudioManager if GameManager else null
	
	# Initialize wheel data arrays
	for i in range(4):
		wheel_rotations.push_back(Vector2(0.0, 0.0))
		wheel_contact_points.push_back(Vector3.ZERO)
		wheel_normal_forces.push_back(0.0)
	
	# Set initial state
	current_gear = 1
	target_gear = 1
	
	# Connect to GameManager signals if available
	if GameManager:
		GameManager.game_state_changed.connect(_on_game_state_changed)
		GameManager.race_started.connect(_on_race_started)
		GameManager.race_ended.connect(_on_race_ended)

# ============================================================================
# MAIN UPDATE LOOP
# ============================================================================
func _physics_process(delta: float) -> void:
	_update_inputs(delta)
	_update_physics(delta)
	_update_gearbox(delta)
	_update_drift(delta)
	_update_engine(delta)
	_update_suspension(delta)
	_apply_forces(delta)
	_update_camera(delta)
	_handle_collisions(delta)
	_update_debug_visualization(delta)

func _process(delta: float) -> void:
	_update_nitrous(delta)
	_update_lap_timing(delta)
	_update_fuel(delta)

# ============================================================================
# INPUT HANDLING
# ============================================================================
func _update_inputs(delta: float) -> void:
	# Clamp inputs to valid ranges
	throttle_input = clamp(throttle_input, 0.0, 1.0)
	brake_input = clamp(brake_input, 0.0, 1.0)
	steering_input = clamp(steering_input, -1.0, 1.0)
	clutch_input = clamp(clutch_input, 0.0, 1.0)
	handbrake_input = clamp(handbrake_input, 0.0, 1.0)
	
	# Apply steering smoothing
	front_wheel_angle = lerp(front_wheel_angle, steering_input * max_steering_angle, delta * STEERING_SPEED)
	rear_wheel_angle = lerp(rear_wheel_angle, steering_input * 0.2, delta * STEERING_SPEED) # Less rear steer

# ============================================================================
# PHYSICS CALCULATIONS
# ============================================================================
func _update_physics(delta: float) -> void:
	# Calculate wheel rotational speed based on gear ratio
	var wheel_speed = _calculate_wheel_rpm()
	engine_rpm = round(wheel_speed)
	
	# Update engine torque based on RPM curve
	engine_torque = _get_engine_torque_curve(engine_rpm)
	
	# Apply acceleration/braking forces
	acceleration = _calculate_acceleration()
	
	# Update velocity
	var current_velocity = velocity.length()
	
	# Limit speed based on gear
	var max_speed = _get_max_speed_for_gear()
	max_speed = min(max_speed, MAX_FORWARD_SPEED)
	
	if current_speed < 0:
		max_speed = abs(MAX_REVERSE_SPEED)
	
	# Apply acceleration
	current_speed += acceleration * delta
	current_speed = clamp(current_speed, -max_speed, max_speed)
	
	# Apply friction/drag
	var drag_factor = 0.02
	current_speed -= drag_factor * abs(current_speed) * abs(current_speed) * delta
	
	# Update velocity vector
	var direction = transform.basis.z.normalized()
	direction.y = 0
	direction = direction.normalized()
	
	velocity.x = direction.x * current_speed
	velocity.z = direction.z * current_speed
	
	# Update position
	position += velocity * delta
	
	# Update distance traveled
	total_distance += abs(velocity.length()) * delta
	
	# Calculate G-forces
	longitudinal_g = acceleration / 9.81
	lateral_g = yaw_rate * velocity.length() / 9.81
	
	# Update RPM
	rpm_changed.emit(engine_rpm)
	speed_changed.emit(current_speed)
	
	# Emit engine sound change
	var rpm_ratio = (engine_rpm - IDLE_RPM) as float / (MAX_ENGINE_RPM - IDLE_RPM)
	engine_sound_changed.emit(rpm_ratio)

# ============================================================================
# GEARBOX LOGIC
# ============================================================================
func _update_gearbox(delta: float) -> void:
	if is_shifting:
		shift_timer -= delta
		shift_progress = 1.0 - (shift_timer / 0.5)
		
		if shift_timer <= 0:
			is_shifting = false
			current_gear = target_gear
			gear_changed.emit(current_gear)
		return
	
	# Auto shift logic
	if auto_shift_enabled and not manual_override:
		_auto_shift_logic()
	
	# Manual shift override
	if clutch_input > 0.8:
		_manual_shift_request()

func _auto_shift_logic() -> void:
	# Upshift when approaching redline
	if current_gear < 7 and engine_rpm >= upshift_rpm_threshold:
		target_gear = current_gear + 1
		initiate_shift()
	
	# Downshift when below threshold and slowing
	elif current_gear > 1 and engine_rpm <= downshift_rpm_threshold:
		if current_speed > 20.0:
			target_gear = max(1, current_gear - 1)
			initiate_shift()

func _manual_shift_request() -> void:
	# Check for shift inputs
	if Input.is_action_pressed("gear_up"):
		if current_gear < 7:
			target_gear = current_gear + 1
			initiate_shift()
	elif Input.is_action_pressed("gear_down"):
		if current_gear > 1:
			target_gear = current_gear - 1
			initiate_shift()

func initiate_shift() -> void:
	if current_gear != target_gear:
		is_shifting = true
		shift_timer = 0.5
		shift_progress = 0.0
		if _audio_manager_ref:
			_audio_manager_ref.play_sfx("gear_shift")

func _calculate_wheel_rpm() -> int:
	if current_gear == 0:
		return IDLE_RPM
	
	var gear_ratio = GEAR_RATIOS[current_gear] if current_gear >= 1 else 0.5
	var final_drive = FINAL_DRIVE_RATIO
	var wheel_circumference = 2.0 * PI * wheel_radius
	var speed_mps = current_speed * 0.277778  # km/h to m/s
	
	if current_gear == -1:  # Reverse
		speed_mps *= -1
	
	var wheel_rps = speed_mps / wheel_circumference
	var wheel_rpm = wheel_rps * 60.0
	
	return round((wheel_rpm * gear_ratio * final_drive) as int)

func _get_max_speed_for_gear() -> float:
	var gear = current_gear
	if gear == 0:
		return 0.0
	elif gear == -1:
		return abs(MAX_REVERSE_SPEED)
	
	var gear_ratio = GEAR_RATIOS[gear]
	var max_engine_rpm = REDLINE_RPM
	var final_drive = FINAL_DRIVE_RATIO
	var wheel_circumference = 2.0 * PI * wheel_radius
	
	var max_wheel_rpm = (max_engine_rpm / gear_ratio / final_drive)
	var max_wheel_rps = max_wheel_rpm / 60.0
	var max_speed_mps = max_wheel_rps * wheel_circumference
	var max_speed_kmh = max_speed_mps / 0.277778
	
	return max_speed_kmh

func _get_engine_torque_curve(rpm: int) -> float:
	# Simplified torque curve - peak around 4500 RPM
	var normalized_rpm = (rpm - IDLE_RPM) as float / (REDLINE_RPM - IDLE_RPM)
	
	if normalized_rpm < 0:
		return 0.0
	elif normalized_rpm > 1:
		return 0.0
	
	# Torque curve shape
	var torque_peak = 0.6  # Peak at 60% of max RPM
	var torque_value = sin(normalized_rpm * PI * 0.5) * 0.9 + cos(normalized_rpm * PI * 2.5) * 0.1
	
	return torque_value * 450.0  # Max torque 450 Nm

# ============================================================================
# ACCELERATION & BRAKING
# ============================================================================
func _calculate_acceleration() -> float:
	var accel = 0.0
	
	# Engine acceleration
	if current_gear > 0:
		var torque = engine_torque
		var gear_ratio = GEAR_RATIOS[current_gear]
		var drive_ratio = FINAL_DRIVE_RATIO
		var effective_torque = torque * gear_ratio * drive_ratio
		
		# Convert to force at wheels
		var wheel_force = effective_torque / wheel_radius
		var max_traction_force = vehicle_mass * 9.81 * 0.8  # 0.8 traction coefficient
		
		wheel_force = min(wheel_force, max_traction_force)
		accel = wheel_force / vehicle_mass
		
		# Apply throttle input
		accel *= throttle_input
	else:
		accel = 0.0
	
	# Brake force
	var brake_force = brake_input * BRAKING_FORCE * (vehicle_mass * 9.81) / vehicle_mass
	accel -= brake_force
	
	# Handbrake adds additional braking
	if handbrake_input > 0:
		accel -= handbrake_input * 1.5
	
	# Nitrous boost
	if nitrous_active_time > 0:
		accel *= nitrous_boost_factor
	
	return accel

# ============================================================================
# DRIFT MECHANICS
# ============================================================================
func _update_drift(delta: float) -> void:
	if not drift_enabled:
		is_drifting = false
		return
	
	# Determine if drifting based on lateral G and slip angle
	var lateral_g_threshold = DRIFT_THRESHOLD
	var slip_angle_threshold = 0.3
	
	is_drifting = (abs(lateral_g) > lateral_g_threshold or abs(slip_angle) > slip_angle_threshold)
	
	if is_drifting:
		# Accumulate drift score
		var drift_intensity = abs(lateral_g) / lateral_g_threshold
		var drift_point_gain = drift_intensity * 10.0 * delta
		
		if drift_combo_timer > 0:
		漂移_combo += 1
		漂移_combo_timer -= delta
		else:
			漂移_combo = 1
			漂移_combo_timer = 1.0
		
		漂移_score += drift_point_gain
		漂移_angle_target = sign(lateral_g) * drift_max_angle
		漂移_angle_current = lerp(漂移_angle_current, 漂移_angle_target, delta * 2.0)
		
		# Reduce grip during drift
		slip_angle += lateral_g * 0.05
	else:
		# Recover from drift
		if abs(漂移_angle_current) > 0.1:
			漂移_angle_current = lerp(漂移_angle_current, 0.0, delta * drift_recovery_rate)
		
		# Reset combo
		漂移_combo_timer -= delta
		if 漂移_combo_timer <= 0:
			漂移_combo = 0
			漂移_score = 0
		
		# Gradually restore grip
		slip_angle = lerp(slip_angle, 0.0, delta * 0.1)
	
	# Handbrake can initiate drift
	if handbrake_input > 0.7 and current_speed > 30.0:
		is_drifting = true
		漂移_angle_target = steering_input * drift_max_angle
		漂移_angle_current = lerp(漂移_angle_current, 漂移_angle_target, delta * 3.0)
	
	# Emit drift events
	if is_drifting and not prev_was_drifting:
		drift_started.emit(abs(漂移_angle_current))
	elif not is_drifting and prev_was_drifting:
		drift_ended.emit()
		if _audio_manager_ref:
			_audio_manager_ref.play_sfx("drift_end")
	
	prev_was_drifting = is_drifting

# ============================================================================
# ENGINE MANAGEMENT
# ============================================================================
func _update_engine(delta: float) -> void:
	# Fuel consumption
	if current_gear > 0:
		var consumption = fuel_consumption_rate * (throttle_input * 2.0)
		fuel_level -= consumption * delta
		fuel_level = max(0.0, fuel_level)
	
	# Engine temperature (increases with load)
	if throttle_input > 0.5:
		engine_temperature += 0.5 * delta
	else:
		engine_temperature -= 0.3 * delta
	
	engine_temperature = clamp(engine_temperature, 70.0, 130.0)
	
	# Engine health degradation
	if engine_temperature > 110.0:
		engine_health -= 0.01 * delta
		engine_health = max(0.0, engine_health)

# ============================================================================
# FUEL SYSTEM
# ============================================================================
func _update_fuel(delta: float) -> void:
	if fuel_level <= 0 and current_gear > 0:
		# Engine stalls
		current_gear = 0
		current_speed = 0
		velocity = Vector3.ZERO
		if _audio_manager_ref:
			_audio_manager_ref.play_sfx("engine_stall")

# ============================================================================
# NITROUS BOOST
# ============================================================================
func _update_nitrous(delta: float) -> void:
	if nitrous_available:
		nitrous_timer -= delta
		if nitrous_timer <= 0:
			nitrous_available = false
			nitrous_fuel = 100.0
	
	# Activate nitrous
	if Input.is_action_pressed("nitrous") and nitrous_fuel > 0:
		nitrous_active_time = 3.0
		nitrous_fuel -= 5.0 * delta
		nitrous_fuel = max(0.0, nitrous_fuel)
		
		if nitrous_fuel <= 0:
			nitrous_available = true
			nitrous_timer = nitrous_charge_time
	
	if nitrous_active_time > 0:
		nitrous_active_time -= delta
		if nitrous_active_time <= 0:
			if _audio_manager_ref:
				_audio_manager_ref.play_sfx("nitrous_end")

# ============================================================================
# SUSPENSION SIMULATION
# ============================================================================
func _update_suspension(delta: float) -> void:
	# Simulate suspension compression based on terrain and forces
	var vertical_accel = velocity.y / delta if delta > 0 else 0.0
	
	front_suspension_compression = lerp(front_suspension_compression, 
		vertical_accel * 0.001, delta * suspension_stiffness)
	rear_suspension_compression = lerp(rear_suspension_compression,
		vertical_accel * 0.001, delta * suspension_stiffness)
	
	front_suspension_compression = clamp(front_suspension_compression, -suspension_travel, suspension_travel)
	rear_suspension_compression = clamp(rear_suspension_compression, -suspension_travel, suspension_travel)
	
	suspension_compressed.emit(front_suspension_compression)

# ============================================================================
# FORCES APPLICATION
# ============================================================================
func _apply_forces(delta: float) -> void:
	# Apply steering influence to rotation
	var steer_factor = front_wheel_angle / max_steering_angle
	yaw_rate = steer_factor * 2.5
	
	# Apply yaw to rotation
	transform.basis = Quaternion(transform.basis.z, yaw_rate * delta)

# ============================================================================
# CAMERA FOLLOW
# ============================================================================
func _update_camera(delta: float) -> void:
	# Camera follows behind vehicle smoothly
	var target_pos = position - transform.basis.z * 4.0 + transform.basis.up * 2.0
	
	camera_offset = lerp(camera_offset, target_pos - position, delta * camera_follow_smooth)

# ============================================================================
# COLLISION HANDLING
# ============================================================================
func _handle_collisions(delta: float) -> void:
	var collider = get_world_3d().direct_space_state.intersect_ray(
		position,
		position + Vector3.DOWN * 2.0,
		["terrain", "obstacle", "wall"]
	)
	
	if collider and collider.collider:
		collision_count += 1
		collision_impact = velocity
		collision_normal = collider.normal
		collision_object = collider.collider
		
		last_collision_time = Time.get_ticks_msec() as float / 1000.0
		
		if _audio_manager_ref:
			_audio_manager_ref.play_sfx("collision")
		
		collision_detected.emit(collision_impact)

# ============================================================================
# LAP TIMING
# ============================================================================
func _update_lap_timing(delta: float) -> void:
	if GameManager.current_state == GameManager.GameState.RACE_ACTIVE:
		if start_time == 0:
			start_time = Time.get_unix_time_from_system()
		
		current_lap_time += delta
		
		# Check checkpoint passing (simplified)
		checkpoints_passed += 1
		if checkpoints_passed >= 4:  # Assume 4 checkpoints per lap
			checkpoints_passed = 0
			current_lap += 1
			
			if current_lap_time < best_lap_time:
				best_lap_time = current_lap_time
			
			lap_completed.emit({
				"lap_number": current_lap,
				"time": current_lap_time,
				"best": current_lap_time < best_lap_time
			})
			
			current_lap_time = 0.0

# ============================================================================
# EVENT HANDLERS
# ============================================================================
func _on_game_state_changed(new_state: GameState) -> void:
	match new_state:
		GameManager.GameState.RACE_ACTIVE:
			start_time = Time.get_unix_time_from_system()
			current_lap = 1
			best_lap_time = 999.0
		GameManager.GameState.RACE_PAUSED:
			pass
		GameManager.GameState.RACE_FINISHED:
			is_drifting = false
			current_gear = 0
			throttle_input = 0.0
			brake_input = 0.0

func _on_race_started(race_data: Dictionary) -> void:
	start_time = Time.get_unix_time_from_system()
	current_lap = 1
	best_lap_time = 999.0
	fuel_level = 100.0

func _on_race_ended(results: Dictionary) -> void:
	is_drifting = false
	current_gear = 0
	throttle_input = 0.0
	brake_input = 0.0

# ============================================================================
# DEBUG VISUALIZATION
# ============================================================================
func _update_debug_visualization(delta: float) -> void:
	if not debug_visualization:
		return
	
	debug_lines.clear()
	
	# Draw velocity vector
	debug_lines.push_back({
		"from": position,
		"to": position + velocity * 2.0,
		"color": Color.GREEN
	})
	
	# Draw steering angle visualization
	var steer_dir = transform.basis.z.rotated(transform.basis.y, front_wheel_angle) * 2.0
	debug_lines.push_back({
		"from": position,
		"to": position + steer_dir,
		"color": Color.YELLOW
	})

# ============================================================================
# PUBLIC API
# ============================================================================
func reset_vehicle() -> void:
	current_speed = 0.0
	velocity = Vector3.ZERO
	current_gear = 1
	target_gear = 1
	engine_rpm = IDLE_RPM
	fuel_level = 100.0
	is_drifting = false
	漂移_score = 0.0
	漂移_combo = 0
	start_time = 0.0
	current_lap_time = 0.0
	current_lap = 1

func set_gear(gear: int) -> void:
	target_gear = clamp(gear, -1, 7)
	if gear == 0:
		target_gear = 0

func set_throttle(input: float) -> void:
	throttle_input = clamp(input, 0.0, 1.0)

func set_brake(input: float) -> void:
	brake_input = clamp(input, 0.0, 1.0)

func set_steering(input: float) -> void:
	steering_input = clamp(input, -1.0, 1.0)

func set_handbrake(input: float) -> void:
	handbrake_input = clamp(input, 0.0, 1.0)

func get_speed_kmh() -> float:
	return current_speed

func get_rpm() -> int:
	return engine_rpm

func get_gear() -> int:
	return current_gear

func is_drifting_active() -> bool:
	return is_drifting

func get_drift_score() -> float:
	return 漂移_score

func get_fuel_percentage() -> float:
	return fuel_level

func get_engine_temperature() -> float:
	return engine_temperature

func get_total_distance() -> float:
	return total_distance

func get_best_lap_time() -> float:
	return best_lap_time

func is_engine_running() -> bool:
	return current_gear != 0 and engine_health > 0

func request_reset() -> void:
	reset_vehicle()

# ============================================================================
# PRIVATE HELPER METHODS
# ============================================================================
func _set_gravity(value: float) -> void:
	gravity = value
	if Engine.has_singleton("PhysicsServer3D"):
		Engine.get_singleton("PhysicsServer3D").set_default_gravity(gravity)

func _set_physics_tick_rate(value: int) -> void:
	physics_tick_rate = value

func _set_max_substeps(value: int) -> void:
	max_substeps = value

func _set_time_scale(value: float) -> void:
	time_scale = value

func _set_default_vehicle_mass(value: float) -> void:
	default_vehicle_mass = value

func _set_default_wheels(value: float) -> void:
	default_wheels = value

# ============================================================================
# PREVIOUS STATE TRACKING
# ============================================================================
var prev_was_drifting: bool = false

# ============================================================================
# END OF FILE
</script>