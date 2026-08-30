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
@export var vehicle_mass: float = 1500.0: set = _set_vehicle_mass
@export var center_of_mass: Vector3 = Vector3(0.0, 0.5, 0.0): set = _set_center_of_mass
@export var wheelbase: float = 2.5: set = _set_wheelbase
@export var track_width: float = 1.6: set = _set_track_width
@export var max_steering_angle: float = 0.6: set = _set_max_steering_angle

@export_group("Drift Settings")
@export var drift_enabled: bool = true
@export var drift_gr_reduction: float = 0.4
@export var drift_recovery_rate: float = 0.15
@export var drift_force_multiplier: float = 1.2

@export_group("Suspension")
@export var suspension_stiffness: float = 50.0: set = _set_suspension_stiffness
@export var suspension_damping: float = 2.0: set = _set_suspension_damping
@export var suspension_travel: float = 0.15: set = _set_suspension_travel
@export var wheel_radius: float = 0.32: set = _set_wheel_radius

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
var gear_shift_direction: int = 0 # -1 down, +1 up, 0 none

# ============================================================================
# VEHICLE STATE
# ============================================================================
var current_speed: float = 0.0
var current_rpm: int = IDLE_RPM
var current_gear: int = 0 # 0 = neutral, 1-6 = forward gears, -1 = reverse
var max_gear: int = 6
var is_in_drift: bool = false
var drift_angle: float = 0.0
var slip_angle: float = 0.0
var lateral_slip: float = 0.0
var longitudinal_slip: float = 0.0

# ============================================================================
# SUSPENSION SYSTEM
# ============================================================================
var front_suspension_compression: float = 0.0
var rear_suspension_compression: float = 0.0
var wheel_contact_points: Array[Vector3] = []
var wheel_forces: Array[Vector3] = []

# ============================================================================
# PHYSICS VARIABLES
# ============================================================================
var acceleration: float = 0.0
var braking_force: float = 0.0
var steering_angle: float = 0.0
var wheel_torque: float = 0.0
var drag_coefficient: float = 0.30
var air_density: float = 1.225
var frontal_area: float = 2.2
var friction_coefficient: float = 0.8
var ground_friction: float = 0.7

# ============================================================================
# RACE DATA
# ============================================================================
var total_distance_traveled: float = 0.0
var lap_start_time: float = 0.0
var last_checkpoint: Vector3 = Vector3.ZERO
var checkpoint_distances: Array[float] = []
var current_lap: int = 1

# ============================================================================
# REFERENCE TO PHYSICS SETTINGS
# ============================================================================
var physics_settings: PhysicsSettings = null

# ============================================================================
# INITIALIZATION
# ============================================================================
func _ready() -> void:
	_process_mode = ProcessModeEnum.ALWAYS
	_init_references()
	_connect_signals_to_systems()
	_calculate_initial_values()

func _init_references() -> void:
	if GameManager != null:
		_audio_manager_ref = GameManager.get_node_or_null("/root/AudioManager")
	
	if PhysicsSettings != null:
		physics_settings = PhysicsSettings.new()
		_load_physics_constants()

func _connect_signals_to_systems() -> void:
	gear_changed.connect(_on_gear_changed)
	speed_changed.connect(_on_speed_changed)
	rpm_changed.connect(_on_rpm_changed)
	engine_sound_changed.connect(_on_engine_sound_changed)
	collision_detected.connect(_on_collision_detected)
	drift_started.connect(_on_drift_started)
	drift_ended.connect(_on_drift_ended)

func _load_physics_constants() -> void:
	if physics_settings:
		vehicle_mass = physics_settings.default_vehicle_mass
		gravity = physics_settings.gravity
		time_scale = physics_settings.time_scale

func _calculate_initial_values() -> void:
	_update_wheels()
	_apply_initial_state()

# ============================================================================
# MAIN UPDATE LOOP
# ============================================================================
func _physics_process(delta: float) -> void:
	_validate_inputs(delta)
	_process_throttle_and_brake(delta)
	_process_steering(delta)
	_process_gear_shifting(delta)
	_update_engine_and_transmission(delta)
	_apply_physics(delta)
	_update_suspension(delta)
	_update_race_data()
	_check_drift_conditions(delta)

func _validate_inputs(delta: float) -> void:
	throttle_input = clamp(throttle_input, 0.0, 1.0)
	brake_input = clamp(brake_input, 0.0, 1.0)
	steering_input = clamp(steering_input, -1.0, 1.0)
	clutch_input = clamp(clutch_input, 0.0, 1.0)
	handbrake_input = clamp(handbrake_input, 0.0, 1.0)

# ============================================================================
# THROTTLE AND BRAKE PROCESSING
# ============================================================================
func _process_throttle_and_brake(delta: float) -> void:
	var effective_throttle: float = throttle_input * (1.0 - clutch_input)
	var effective_brake: float = brake_input * (1.0 - clutch_input)
	
	# Calculate engine torque based on throttle and RPM
	wheel_torque = _calculate_engine_torque(effective_throttle)
	
	# Apply throttle force
	if effective_throttle > 0.01 and current_gear > 0:
		var drive_force: float = wheel_torque * GEAR_RATIOS[current_gear] * FINAL_DRIVE_RATIO / wheel_radius
		drive_force *= 0.1 # Scale factor for game feel
		acceleration = sign(current_speed) * min(abs(acceleration), drive_force) if abs(current_speed) > 0 else drive_force
	
	# Apply braking force
	if effective_brake > 0.01:
		braking_force = BRAKING_FORCE * effective_brake * vehicle_mass * delta
	else:
		braking_force = 0.0
	
	# Apply drag resistance
	var air_drag: float = 0.5 * drag_coefficient * air_density * frontal_area * current_speed * current_speed
	acceleration -= air_drag / vehicle_mass

# ============================================================================
# STEERING PROCESSING
# ============================================================================
func _process_steering(delta: float) -> void:
	var target_steering_angle: float = steering_input * max_steering_angle
	steering_angle = lerp(steering_angle, target_steering_angle, STEERING_SPEED * delta)
	
	# Apply steering to velocity vector
	var lateral_velocity: Vector3 = Vector3.RIGHT.normalized() * current_speed
	lateral_velocity.x = steering_angle * lateral_velocity.length()
	body_velocity += lateral_velocity * delta * 0.5

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================
func _process_gear_shifting(delta: float) -> void:
	# Automatic gear shifting based on RPM
	if clutch_input < 0.1 and current_gear != 0:
		_auto_shift_gears(delta)
	
	# Manual gear shifting via signal
	if gear_shift_direction != 0:
		_manual_shift_gear(gear_shift_direction)
		gear_shift_direction = 0

func _auto_shift_gears(delta: float) -> void:
	var should_upshift: bool = current_rpm >= REDLINE_RPM and current_gear < max_gear
	var should_downshift: bool = current_rpm <= IDLE_RPM and current_gear > 1
	
	if should_upshift:
		_change_gear(1)
	elif should_downshift:
		_change_gear(-1)

func _manual_shift_gear(direction: int) -> void:
	var new_gear: int = current_gear + direction
	if new_gear >= -1 and new_gear <= max_gear:
		_change_gear(direction)
	else:
		print_debug("Gear shift blocked: out of range")

func _change_gear(direction: int) -> void:
	var old_gear: int = current_gear
	current_gear += direction
	
	if direction == 1 and old_gear > 0:
		current_gear = min(current_gear, max_gear)
	elif direction == -1 and old_gear < 0:
		current_gear = max(current_gear, -1)
	
	if current_gear != old_gear:
		gear_changed.emit(current_gear)
		_on_gear_changed(current_gear)

# ============================================================================
# ENGINE AND TRANSMISSION UPDATE
# ============================================================================
func _update_engine_and_transmission(delta: float) -> void:
	# Calculate wheel angular velocity from vehicle speed
	var wheel_angular_velocity: float = abs(current_speed) / wheel_radius
	
	# Calculate engine RPM based on gear ratio
	if current_gear != 0:
		var transmission_ratio: float = GEAR_RATIOS[current_gear] * FINAL_DRIVE_RATIO
		current_rpm = int(wheel_angular_velocity * transmission_ratio * 9.55)
		
		# Cap RPM at limits
		current_rpm = clamp(current_rpm, IDLE_RPM, MAX_ENGINE_RPM)
	else:
		# Engine idles when in neutral
		current_rpm = lerp(current_rpm, IDLE_RPM, 5.0 * delta)
	
	# Update engine sound signal
	var rpm_ratio: float = float(current_rpm) / float(MAX_ENGINE_RPM)
	engine_sound_changed.emit(rpm_ratio)
	rpm_changed.emit(current_rpm)

# ============================================================================
# PHYSICS APPLICATION
# ============================================================================
func _apply_physics(delta: float) -> void:
	# Apply gravity
	var gravity_vector: Vector3 = Vector3.DOWN * gravity
	add_gravity(gravity_vector)
	
	# Apply vehicle movement
	if not is_on_floor():
		velocity.y -= gravity * delta * 0.5
	
	# Apply acceleration and braking
	var move_direction: Vector3 = Vector3.FORWARD * (throttle_input - brake_input)
	var forward_force: Vector3 = move_direction * acceleration * vehicle_mass * delta
	
	velocity += forward_force
	
	# Apply lateral grip/friction
	var lateral_force: Vector3 = velocity.cross(Vector3.UP).normalized() * -lateral_slip * friction_coefficient * vehicle_mass * delta
	velocity += lateral_force
	
	# Update speed from velocity
	current_speed = velocity.length()
	
	# Clamp speed to reasonable bounds
	current_speed = clamp(current_speed, MAX_REVERSE_SPEED, MAX_FORWARD_SPEED)
	velocity = velocity.normalized() * current_speed
	
	# Apply position
	move_and_slide()

# ============================================================================
# SUSPENSION SYSTEM
# ============================================================================
func _update_suspension(delta: float) -> void:
	# Raycast to get wheel contact points
	var wheel_positions: Array[Vector3] = _get_wheel_world_positions()
	
	for i in range(wheel_positions.size()):
		var ray_from: Vector3 = wheel_positions[i] + Vector3.UP * 0.5
		var ray_to: Vector3 = wheel_positions[i] - Vector3.UP * 1.0
		
		var space_state: SpaceState3D = get_world_3d().direct_space_state
		var query: PhysicsRayQueryParameters3D = PhysicsRayQueryParameters3D.create(
			ray_from, ray_to, 1 << body_layer
		)
		
		var result: Dictionary = space_state.intersect_ray(query)
		
		if result.is_empty():
			continue
		
		var collision_point: Vector3 = result.position
		var compression: float = (wheel_positions[i].y - collision_point.y) - suspension_travel
		
		compression = clamp(compression, -suspension_travel, suspension_travel)
		
		if i % 2 == 0: # Front wheels
			front_suspension_compression = compression
		else: # Rear wheels
			rear_suspension_compression = compression
		
		# Calculate spring force
		var spring_force: float = suspension_stiffness * compression
		var damping_force: float = suspension_damping * (compression - _last_suspension_compression(i))
		var total_force: float = spring_force + damping_force
		
		# Apply upward force to body
		var wheel_force: Vector3 = Vector3.UP * total_force / 4.0
		wheel_forces.append(wheel_force)
		
		_last_suspension_compressions[i] = compression
	
	# Emit suspension signal if significant change
	var avg_compression: float = (front_suspension_compression + rear_suspension_compression) / 2.0
	if abs(avg_compression - _prev_avg_compression) > 0.01:
		suspension_compressed.emit(avg_compression)
		_prev_avg_compression = avg_compression

func _get_wheel_world_positions() -> Array[Vector3]:
	var positions: Array[Vector3] = []
	var offset_x: float = track_width / 2.0
	var offset_z_front: float = wheelbase / 2.0
	var offset_z_rear: float = -wheelbase / 2.0
	
	positions.append(global_position + Vector3(offset_x, 0, offset_z_front)) # Front Left
	positions.append(global_position + Vector3(-offset_x, 0, offset_z_front)) # Front Right
	positions.append(global_position + Vector3(offset_x, 0, offset_z_rear)) # Rear Left
	positions.append(global_position + Vector3(-offset_x, 0, offset_z_rear)) # Rear Right
	
	return positions

func _last_suspension_compression(index: int) -> float:
	return _last_suspension_compressions[index]

func _last_suspension_compressions: Array[float] = [0.0, 0.0, 0.0, 0.0]
var _prev_avg_compression: float = 0.0

# ============================================================================
# DRIFT MECHANICS
# ============================================================================
func _check_drift_conditions(delta: float) -> void:
	if not drift_enabled:
		if is_in_drift:
			is_in_drift = false
			drift_ended.emit()
		return
	
	# Calculate slip angle
	var heading: Vector3 = global_transform.basis.z.normalized()
	var velocity_direction: Vector3 = velocity.normalized()
	
	slip_angle = heading.angle_to(velocity_direction)
	
	# Check if drifting conditions are met
	var lateral_accel: float = abs(velocity.cross(Vector3.UP)).length()
	var throttle_applied: bool = throttle_input > 0.3
	var handbrake_applied: bool = handbrake_input > 0.5
	
	if (lateral_accel > DRIFT_THRESHOLD * 2.0 or handbrake_applied) and throttle_applied:
		if not is_in_drift:
			is_in_drift = true
			drift_started.emit(slip_angle)
			_apply_drift_modifications(delta)
	else:
		if is_in_drift:
			is_in_drift = false
			drift_ended.emit()
			_recover_grip(delta)

func _apply_drift_modifications(delta: float) -> void:
	# Reduce grip during drift
	friction_coefficient = GRIP_LEVEL_DRIFT * (1.0 - drift_gr_reduction)
	
	# Apply drift force multiplier
	var drift_force: Vector3 = velocity.cross(Vector3.UP).normalized() * current_speed * drift_force_multiplier * delta
	velocity += drift_force
	
	# Accumulate drift angle
	drift_angle = lerp(drift_angle, slip_angle, drift_recovery_rate * delta)

func _recover_grip(delta: float) -> void:
	# Gradually restore grip
	friction_coefficient = lerp(friction_coefficient, GRIP_LEVEL_NORMAL, drift_recovery_rate * delta)
	drift_angle = lerp(drift_angle, 0.0, drift_recovery_rate * delta)

# ============================================================================
# ENGINE TORQUE CALCULATION
# ============================================================================
func _calculate_engine_torque(throttle: float) -> float:
	# Torque curve based on RPM (simplified bell curve)
	var normalized_rpm: float = float(current_rpm) / float(MAX_ENGINE_RPM)
	var peak_torque_rpm: float = 0.5
	var torque_peak: float = 350.0 # Nm
	
	# Simple torque curve formula
	var torque_curve: float = 1.0 - pow((normalized_rpm - peak_torque_rpm) * 4.0, 2.0)
	torque_curve = max(torque_curve, 0.0)
	
	var base_torque: float = torque_peak * torque_curve * throttle
	
	# Add boost from lower gears
	var gear_bonus: float = 1.0 + (max_gear - current_gear) * 0.1 if current_gear > 0 else 0.5
	
	return base_torque * gear_bonus

# ============================================================================
# WHEEL MANAGEMENT
# ============================================================================
func _update_wheels() -> void:
	wheel_contact_points.clear()
	wheel_forces.clear()
	
	var wheel_positions: Array[Vector3] = _get_wheel_world_positions()
	
	for pos in wheel_positions:
		wheel_contact_points.append(pos)
		wheel_forces.append(Vector3.ZERO)

# ============================================================================
# RACE DATA UPDATES
# ============================================================================
func _update_race_data() -> void:
	total_distance_traveled += velocity.length() * 0.01
	
	# Lap timing
	if GameManager.current_state == GameManager.GameState.RACE_ACTIVE:
		if lap_start_time == 0.0:
			lap_start_time = Time.get_ticks_msec()
		
		var elapsed: float = (Time.get_ticks_msec() - lap_start_time) / 1000.0
		var distance_from_start: float = global_position.distance_to(Vector3.ZERO)
		
		# Checkpoint system (simplified)
		if distance_from_start < 50.0 and current_lap == 1:
			_complete_lap()

func _complete_lap() -> void:
	var lap_time: float = (Time.get_ticks_msec() - lap_start_time) / 1000.0
	var lap_data: Dictionary = {
		"lap_number": current_lap,
		"time": lap_time,
		"distance": total_distance_traveled,
		"timestamp": Time.get_unix_time_from_system()
	}
	
	lap_completed.emit(lap_data)
	current_lap += 1
	lap_start_time = 0.0

# ============================================================================
# COLLISION HANDLING
# ============================================================================
func _on_collision_detected(impact_velocity: Vector3) -> void:
	var impact_force: float = impact_velocity.length() * vehicle_mass
	
	# Trigger damage event if impact is severe
	if impact_force > 5000.0:
		race_event.emit("collision_damage")
		_apply_collision_effects(impact_velocity)

func _apply_collision_effects(impact: Vector3) -> void:
	# Screen shake effect
	var shake_intensity: float = min(impact.length(), 5.0)
	get_parent().call_deferred("add_impact", impact, shake_intensity)
	
	# Eject particles
	_spawn_collision_particles(impact)

func _spawn_collision_particles(position: Vector3) -> void:
	var particle_count: int = 20
	for i in range(particle_count):
		var particle: Marker3D = Marker3D.new()
		particle.global_position = position + Vector3(randf_range(-1, 1), randf_range(0, 1), randf_range(-1, 1))
		
		var velocity: Vector3 = Vector3(randf_range(-1, 1), randf_range(0, 2), randf_range(-1, 1)).normalized() * randf_range(2, 5)
		
		get_parent().call_deferred("add_child", particle)
		particle.call_deferred("emit_signal", "body_entered", velocity)

# ============================================================================
# SIGNAL HANDLERS
# ============================================================================
func _on_gear_changed(new_gear: int) -> void:
	if _audio_manager_ref:
		_audio_manager_ref.play_sound("gear_shift")
	print_debug("Gear changed to: ", new_gear)

func _on_speed_changed(new_speed: float) -> void:
	pass

func _on_rpm_changed(rpm: int) -> void:
	pass

func _on_engine_sound_changed(rpm_ratio: float) -> void:
	if _audio_manager_ref:
		_audio_manager_ref.set_pitch("engine", 0.5 + 0.5 * rpm_ratio)

func _on_drift_started(drift_angle: float) -> void:
	if _audio_manager_ref:
		_audio_manager_ref.play_sound("drift_start")
	print_debug("Drift started at angle: ", drift_angle)

func _on_drift_ended() -> void:
	if _audio_manager_ref:
		_audio_manager_ref.play_sound("drift_end")
	print_debug("Drift ended")

func _on_collision_detected(impact_velocity: Vector3) -> void:
	pass

# ============================================================================
# HELPER METHODS
# ============================================================================
func reset_vehicle() -> void:
	current_speed = 0.0
	current_rpm = IDLE_RPM
	current_gear = 0
	velocity = Vector3.ZERO
	acceleration = 0.0
	braking_force = 0.0
	steering_angle = 0.0
	total_distance_traveled = 0.0
	lap_start_time = 0.0
	current_lap = 1
	is_in_drift = false
	drift_angle = 0.0
	slip_angle = 0.0
	lateral_slip = 0.0
	longitudinal_slip = 0.0
	front_suspension_compression = 0.0
	rear_suspension_compression = 0.0
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0

func set_powertrain(powertrain_node: Node) -> void:
	_powertrain = powertrain_node

func get_current_stats() -> Dictionary:
	return {
		"speed": current_speed,
		"rpm": current_rpm,
		"gear": current_gear,
		"is_drifting": is_in_drift,
		"total_distance": total_distance_traveled,
		"current_lap": current_lap,
		"front_suspension": front_suspension_compression,
		"rear_suspension": rear_suspension_compression
	}

func _set_vehicle_mass(value: float) -> void:
	vehicle_mass = max(value, 500.0)
	max_vehicle_mass = vehicle_mass

func _set_center_of_mass(value: Vector3) -> void:
	center_of_mass = value.clamp(Vector3(-1.0, 0.0, -1.0), Vector3(1.0, 2.0, 1.0))

func _set_wheelbase(value: float) -> void:
	wheelbase = max(value, 1.5)

func _set_track_width(value: float) -> void:
	track_width = max(value, 1.0)

func _set_max_steering_angle(value: float) -> void:
	max_steering_angle = clamp(value, 0.1, 1.5)

func _set_suspension_stiffness(value: float) -> void:
	suspension_stiffness = max(value, 10.0)

func _set_suspension_damping(value: float) -> void:
	suspension_damping = max(value, 0.5)

func _set_suspension_travel(value: float) -> void:
	suspension_travel = clamp(value, 0.05, 0.5)

func _set_wheel_radius(value: float) -> void:
	wheel_radius = max(value, 0.2)

# ============================================================================
# DEBUG UTILITIES
# ============================================================================
func debug_print_stats() -> void:
	print_debug("=== Vehicle Stats ===")
	print_debug("Speed: ", current_speed, " m/s")
	print_debug("RPM: ", current_rpm)
	print_debug("Gear: ", current_gear)
	print_debug("Drift: ", is_in_drift)
	print_debug("Throttle: ", throttle_input)
	print_debug("Brake: ", brake_input)
	print_debug("Steering: ", steering_angle)

func apply_debug_force(force_vector: Vector3) -> void:
	velocity += force_vector * 0.1

func toggle_debug_mode(enabled: bool) -> void:
	if enabled:
		debug_print_stats()
	else:
		reset_vehicle()

# ============================================================================
# SERIALIZATION FOR SAVE/LOAD
# ============================================================================
func serialize_state() -> Dictionary:
	return {
		"position": global_position,
		"rotation": global_rotation,
		"velocity": velocity,
		"current_speed": current_speed,
		"current_rpm": current_rpm,
		"current_gear": current_gear,
		"total_distance_traveled": total_distance_traveled,
		"current_lap": current_lap,
		"is_in_drift": is_in_drift,
		"drift_angle": drift_angle,
		"throttle_input": throttle_input,
		"brake_input": brake_input,
		"steering_input": steering_input
	}

func deserialize_state(state: Dictionary) -> void:
	if state.has("position"):
		global_position = state.position
	if state.has("rotation"):
		global_rotation = state.rotation
	if state.has("velocity"):
		velocity = state.velocity
	if state.has("current_speed"):
		current_speed = state.current_speed
	if state.has("current_rpm"):
		current_rpm = state.current_rpm
	if state.has("current_gear"):
		current_gear = state.current_gear
	if state.has("total_distance_traveled"):
		total_distance_traveled = state.total_distance_traveled
	if state.has("current_lap"):
		current_lap = state.current_lap
	if state.has("is_in_drift"):
		is_in_drift = state.is_in_drift
	if state.has("drift_angle"):
		drift_angle = state.drift_angle
	if state.has("throttle_input"):
		throttle_input = state.throttle_input
	if state.has("brake_input"):
		brake_input = state.brake_input
	if state.has("steering_input"):
		steering_input = state.steering_input

# ============================================================================
# END OF FILE
# ============================================================================
</FILE "scripts/vehicles/VehicleController.gd">>