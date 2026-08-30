extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulation
## Handles throttle, brake, steering, wheel forces, gear shifting, and vehicle dynamics
## Copyright 2026 Thalamus Racing Simulator Project

# ============================================================================
# SIGNALS - Game Event Notifications
# ============================================================================
signal speed_changed(current_speed: float)
signal rpm_changed(current_rpm: float)
signal gear_changed(new_gear: int)
signal drift_started(drift_intensity: float)
signal drift_ended()
signal collision_detected(collision_info: Dictionary)
signal lap_completed(lap_data: Dictionary)
signal race_event(event_type: String, data: Dictionary)
signal engine_sound_changed(rpm_ratio: float)
signal suspension_compressed(compression_amount: float)

# ============================================================================
# CONSTANTS - Physics Tuning Values
# ============================================================================
const MAX_SPEED_KMH: float = 320.0
const ACCELERATION_RATE: float = 12.0
const BRAKING_FORCE: float = 20.0
const TURN_SPEED: float = 4.5
const DRIFT_THRESHOLD: float = 0.7
const DRIFT_INTENSITY_MAX: float = 1.0
const MIN_GEAR: int = 0
const MAX_GEAR: int = 6
const IDLE_RPM: float = 800.0
const REDLINE_RPM: float = 7500.0
const SHIFT_RPM: float = 7000.0
const CLUTCH_RELEASE_TIME: float = 0.15
const TURBO_CHARGE_TIME: float = 2.5
const SUSPENSION_COMPRESSION_LIMIT: float = 0.3
const GRAVITY_ACCEL: float = 9.81
const METERS_TO_KMH: float = 3.6

# ============================================================================
# EXPORTED CONFIGURATION - Vehicle Setup (Exposed in Inspector)
# ============================================================================
@export_group("Vehicle Configuration")
@export var vehicle_mass: float = 1500.0
@export var engine_torque: float = 450.0
@export var transmission_ratio: float = 3.5
@export var differential_ratio: float = 3.8
@export var wheel_radius: float = 0.35
@export var track_width: float = 1.6
@export var wheel_base: float = 2.7
@export var center_of_mass_y: float = 0.5
@export var drag_coefficient: float = 0.32
@export var air_density: float = 1.225
@export var max_steering_angle: float = 30.0 * PI / 180.0

@export_group("Suspension Settings")
@export var suspension_stiffness: float = 80000.0
@export var suspension_damping: float = 15000.0
@export var suspension_rest_length: float = 0.4
@export var suspension_travel: float = 0.25
@export var tire_friction: float = 1.1
@export var slip_angle_max: float = 15.0 * PI / 180.0

@export_group("Powertrain Settings")
@export var torque_curve_enabled: bool = true
@export var turbo_enabled: bool = false
@export var turbo_boost_pressure: float = 1.2
@export var launch_control_enabled: bool = false

# ============================================================================
# INTERNAL STATE - Vehicle Dynamics Variables
# ============================================================================
var _current_speed: float = 0.0
var _current_rpm: float = IDLE_RPM
var _current_gear: int = 0
var _throttle_input: float = 0.0
var _brake_input: float = 0.0
var _steering_input: float = 0.0
var _clutch_pedal: float = 1.0
var _turbo_active: bool = false
var _turbo_charge_progress: float = 0.0
var _drift_intensity: float = 0.0
var _is_drifting: bool = false
var _wheel_positions: Array[Vector3] = []
var _wheel_forces: Array[float] = []
var _suspension_compression: Array[float] = []
var _collision_points: Array[Vector3] = []
var _slip_angles: Array[float] = []
var _engine_braking: bool = false
var _in_neutral: bool = false
var _vehicle_facing: Vector3 = Vector3.FORWARD
var _ground_normal: Vector3 = Vector3.UP
var _surface_friction: float = 1.0
var _last_collision_time: float = 0.0
var _total_distance_traveled: float = 0.0
var _lap_start_position: Vector3 = Vector3.ZERO
var _lap_count: int = 0
var _checkpoint_passed: bool = false

# Powertrain internal state
var _torque_output: float = 0.0
var _power_output: float = 0.0
var _gear_ratios: Dictionary = {
	1: 3.8,
	2: 2.1,
	3: 1.5,
	4: 1.1,
	5: 0.9,
	6: 0.75,
	"R": 3.5
}
var _rpm_to_wheel_speed: float = 0.0
var _wheel_rotation_velocity: float = 0.0

# Audio reference for engine sounds
var _audio_bus_idx: int = 0
var _engine_pitch_modifier: float = 1.0

# References to child nodes (will be set in _ready)
var _physics_body: Node3D = null
var _camera_holder: Node3D = null
var _wheels: Array[Node] = []

# Time tracking
var _delta: float = 0.0
var _accumulator: float = 0.0
var _fixed_step: float = 1.0 / 120.0
var _time_since_last_shift: float = 0.0

# Turbine/spool up tracking
var _turbo_spool_timer: float = 0.0
var _turbo_available: bool = true

# Launch control state
var _launch_control_triggered: bool = false
var _launch_control_hold_time: float = 0.0
var _launch_control_active: bool = false

# ============================================================================
# GEAR SHIFT LIMITS - Speed ranges for each gear
# ============================================================================
var _gear_shift_limits: Dictionary = {
	1: {"min": 0, "max": 40},
	2: {"min": 35, "max": 70},
	3: {"min": 60, "max": 110},
	4: {"min": 95, "max": 150},
	5: {"min": 130, "max": 200},
	6: {"min": 170, "max": 320},
	"R": {"min": -5, "max": 0}
}

# ============================================================================
# TORQUE CURVE DATA - Engine torque vs RPM profile
# ============================================================================
var _torque_curve: Array[float] = [
	0.6,  # 0% RPM
	0.75, # 20% RPM
	0.9,  # 40% RPM
	1.0,  # 60% RPM
	1.05, # 80% RPM
	0.95  # 100% RPM
]

# ============================================================================
# CORE INITIALIZATION
# ============================================================================
func _ready() -> void:
	_init_references()
	_setup_wheels()
	_init_audio()
	_set_initial_state()
	_connect_signals_to_systems()

func _init_references() -> void:
	# Find child nodes if they exist
	if has_node("PhysicsBody"):
		_physics_body = get_node("PhysicsBody") as Node3D
	else:
		_physics_body = self
	
	if has_node("CameraHolder"):
		_camera_holder = get_node("CameraHolder") as Node3D

func _setup_wheels() -> void:
	# Initialize wheel positions relative to vehicle center
	_wheel_positions.resize(4)
	_wheel_forces.resize(4)
	_suspension_compression.resize(4)
	_slip_angles.resize(4)
	
	var half_track: float = track_width / 2.0
	var half_wheelbase: float = wheel_base / 2.0
	
	# Front Left Wheel
	_wheel_positions[0] = Vector3(-half_track, 0.0, -half_wheelbase)
	_wheel_forces[0] = 0.0
	_suspension_compression[0] = 0.0
	_slip_angles[0] = 0.0
	
	# Front Right Wheel
	_wheel_positions[1] = Vector3(half_track, 0.0, -half_wheelbase)
	_wheel_forces[1] = 0.0
	_suspension_compression[1] = 0.0
	_slip_angles[1] = 0.0
	
	# Rear Left Wheel
	_wheel_positions[2] = Vector3(-half_track, 0.0, half_wheelbase)
	_wheel_forces[2] = 0.0
	_suspension_compression[2] = 0.0
	_slip_angles[2] = 0.0
	
	# Rear Right Wheel
	_wheel_positions[3] = Vector3(half_track, 0.0, half_wheelbase)
	_wheel_forces[3] = 0.0
	_suspension_compression[3] = 0.0
	_slip_angles[3] = 0.0

func _init_audio() -> void:
	_audio_bus_idx = AudioManager.get_bus_index("Engine") if AudioManager.has_method("get_bus_index") else 0

func _set_initial_state() -> void:
	_current_speed = 0.0
	_current_rpm = IDLE_RPM
	_current_gear = 0
	_throttle_input = 0.0
	_brake_input = 0.0
	_steering_input = 0.0
	_clutch_pedal = 1.0
	_engine_braking = false
	_in_neutral = false
	_turbo_active = false
	_turbo_available = true
	_is_drifting = false
	_total_distance_traveled = 0.0
	_lap_count = 0
	_checkpoint_passed = false
	_launch_control_active = false
	_lap_start_position = global_position

# ============================================================================
# PHYSICS UPDATE LOOP
# ============================================================================
func _physics_process(delta: float) -> void:
	_delta = delta
	_accumulator += delta
	
	# Fixed timestep for physics
	while _accumulator >= _fixed_step:
		_update_vehicle_physics(_fixed_step)
		_accumulator -= _fixed_step
	
	# Variable timestep for rendering
	_update_rendering(delta)

func _update_vehicle_physics(dt: float) -> void:
	# Process input
	_process_inputs()
	
	# Calculate gear ratio based on current gear
	var gear_ratio: float = _calculate_gear_ratio(_current_gear)
	
	# Calculate wheel angular velocity from vehicle speed
	_wheel_rotation_velocity = _current_speed / wheel_radius
	
	# Calculate RPM based on gear and wheel speed
	_calculate_rpm(gear_ratio, dt)
	
	# Calculate torque output from powertrain
	_torque_output = _calculate_torque_output(dt)
	
	# Apply acceleration or braking
	_apply_acceleration(dt, gear_ratio)
	
	# Apply steering
	_apply_steering(dt)
	
	# Update suspension
	_update_suspension(dt)
	
	# Detect and handle collisions
	_handle_collisions(dt)
	
	# Calculate drift mechanics
	_update_drift(dt)
	
	# Update turbo system
	_update_turbo(dt)
	
	# Update launch control
	_update_launch_control(dt)
	
	# Update total distance traveled
	_update_distance_traveled(dt)
	
	# Check for automatic gear shifts
	_check_gear_shifts(dt)
	
	# Emit signals for changes
	_emit_signals()

func _update_rendering(delta: float) -> void:
	# Camera follow and positioning would go here
	pass

# ============================================================================
# INPUT PROCESSING
# ============================================================================
func _process_inputs() -> void:
	# Get input values from InputManager singleton
	if GameManager.has_method("get_input_value"):
		_throttle_input = GameManager.get_input_value("throttle")
		_brake_input = GameManager.get_input_value("brake")
		_steering_input = GameManager.get_input_value("steering")
		
		# Clutch pedal
		_clutch_pedal = GameManager.get_input_value("clutch")
		
		# Handbrake for drifting
		var handbrake = GameManager.get_input_value("handbrake")
		if handbrake > 0.5:
			_trigger_drift_attempt()
		
		# Gear shift input
		if GameManager.is_action_pressed("shift_up"):
			_request_gear_shift(1)
		elif GameManager.is_action_pressed("shift_down"):
			_request_gear_shift(-1)
		
		# Turbo activation
		if GameManager.is_action_pressed("turbo"):
			_activate_turbo()
		
		# Launch control trigger
		if GameManager.is_action_pressed("launch_control"):
			_trigger_launch_control()

func _request_gear_shift(direction: int) -> void:
	# Validate direction
	if direction != 1 and direction != -1:
		return
	
	# Prevent rapid shifting
	if _time_since_last_shift < CLUTCH_RELEASE_TIME:
		return
	
	# Calculate new gear
	var target_gear: int = _current_gear + direction
	
	# Clamp to valid range
	target_gear = clamp(target_gear, MIN_GEAR, MAX_GEAR)
	
	# Neutral position check
	if target_gear == 0:
		_in_neutral = true
		_current_gear = 0
		_current_rpm = IDLE_RPM
		gear_changed.emit(0)
		return
	
	# Perform gear shift
	_current_gear = target_gear
	_in_neutral = false
	_time_since_last_shift = 0.0
	gear_changed.emit(_current_gear)
	
	# Simulate clutch engagement delay
	await get_tree().create_timer(CLUTCH_RELEASE_TIME).timeout
	_time_since_last_shift = CLUTCH_RELEASE_TIME

# ============================================================================
# GEAR & RPM CALCULATIONS
# ============================================================================
func _calculate_gear_ratio(gear: int) -> float:
	match gear:
		1: return _gear_ratios[1]
		2: return _gear_ratios[2]
		3: return _gear_ratios[3]
		4: return _gear_ratios[4]
		5: return _gear_ratios[5]
		6: return _gear_ratios[6]
		_: return transmission_ratio * differential_ratio

func _calculate_rpm(gear_ratio: float, dt: float) -> void:
	# Calculate theoretical RPM based on wheel speed and gear ratio
	var wheel_rad_per_sec: float = _current_speed / wheel_radius
	var engine_rad_per_sec: float = wheel_rad_per_sec * gear_ratio * transmission_ratio * differential_ratio
	var engine_rpm: float = engine_rad_per_sec * 60.0 / (2.0 * PI)
	
	# Smooth RPM transition
	var target_rpm: float = engine_rpm
	if _current_gear == 0:
		# In neutral, RPM goes to idle
		target_rpm = lerp(_current_rpm, IDLE_RPM, dt * 5.0)
	else:
		# In gear, calculate based on throttle
		if _throttle_input > 0:
			# Accelerating
			target_rpm = lerp(_current_rpm, target_rpm, dt * 10.0)
		elif _brake_input > 0 or _engine_braking:
			# Decelerating/engine braking
			target_rpm = lerp(_current_rpm, target_rpm, dt * 8.0)
		else:
			# Coasting
			target_rpm = lerp(_current_rpm, IDLE_RPM + (_current_speed * 0.5), dt * 3.0)
	
	# Clamp to safe limits
	target_rpm = clamp(target_rpm, IDLE_RPM, REDLINE_RPM * 1.2)
	
	# Handle over-rev protection
	if target_rpm > REDLINE_RPM:
		target_rpm = REDLINE_RPM * 0.99
		# Trigger warning signal
		race_event.emit("engine_overrev", {"rpm": target_rpm})
	
	_current_rpm = target_rpm
	_rpm_to_wheel_speed = _current_rpm / (gear_ratio * transmission_ratio * differential_ratio) * (2.0 * PI) / 60.0 * wheel_radius

func _calculate_torque_output(dt: float) -> float:
	var base_torque: float = engine_torque
	
	# Apply turbo boost if active
	if _turbo_active:
		base_torque *= turbo_boost_pressure
	
	# Apply torque curve if enabled
	if torque_curve_enabled:
		var rpm_percent: float = (_current_rpm - IDLE_RPM) / (REDLINE_RPM - IDLE_RPM)
		rpm_percent = clamp(rpm_percent, 0.0, 1.0)
		var curve_index: int = floor(rpm_percent * (_torque_curve.size() - 1))
		curve_index = min(curve_index, _torque_curve.size() - 2)
		var torque_multiplier: float = _torque_curve[curve_index]
		base_torque *= torque_multiplier
	
	# Apply engine braking when not accelerating
	if _brake_input > 0 or (_throttle_input == 0 and _current_rpm > IDLE_RPM):
		base_torque *= 0.3
	
	# Cap torque output
	return min(base_torque, engine_torque * 1.5)

# ============================================================================
# ACCELERATION & BRAKING
# ============================================================================
func _apply_acceleration(dt: float, gear_ratio: float) -> void:
	# Calculate force based on torque and gear ratio
	var drive_force: float = _torque_output * gear_ratio / wheel_radius
	
	# Apply aerodynamic drag
	var drag_force: float = 0.5 * air_density * drag_coefficient * (_current_speed * _current_speed)
	drive_force -= drag_force
	
	# Apply rolling resistance
	var rolling_resistance: float = vehicle_mass * GRAVITY_ACCEL * 0.015
	drive_force -= rolling_resistance
	
	# Apply force to vehicle
	var mass_factor: float = 1.0 / vehicle_mass
	var velocity_change: float = drive_force * mass_factor * dt
	
	# Update speed
	_current_speed += velocity_change
	
	# Cap maximum speed
	if _current_speed > MAX_SPEED_KMH / METERS_TO_KMH:
		_current_speed = MAX_SPEED_KMH / METERS_TO_KMH
		drive_force = 0.0
	
	# Handle reverse gear
	if _current_gear == 0 and _brake_input < 0:
		_current_speed += velocity_change * 0.5
		_current_speed = max(_current_speed, -MAX_SPEED_KMH / METERS_TO_KMH)

func _apply_steering(dt: float) -> void:
	# Only steer if moving and not reversed
	if abs(_current_speed) < 0.5:
		return
	
	# Calculate steering effect based on speed
	var steering_effectiveness: float = 1.0
	if abs(_current_speed) > 20.0:
		steering_effectiveness = 20.0 / abs(_current_speed)
	steering_effectiveness = clamp(steering_effectiveness, 0.3, 1.0)
	
	# Apply steering rotation
	var turn_angle: float = _steering_input * max_steering_angle * TURN_SPEED * dt * steering_effectiveness
	
	# Rotate vehicle
	var rotation_axis: Vector3 = Vector3.UP
	global_rotation.y -= turn_angle
	_vehicle_facing = global_transform.basis.z.rotated(Vector3.UP, -turn_angle)

# ============================================================================
# SUSPENSION SYSTEM
# ============================================================================
func _update_suspension(dt: float) -> void:
	for i in range(4):
		var wheel_pos: Vector3 = _wheel_positions[i]
		var ground_contact: Vector3 = _get_ground_height_at(wheel_pos)
		var compression: float = ground_contact.y - wheel_pos.y
		
		# Limit compression
		compression = clamp(compression, -suspension_travel, suspension_travel)
		_suspension_compression[i] = compression
		
		# Calculate suspension force
		var spring_force: float = -compression * suspension_stiffness
		var damping_force: float = -_suspension_compression[i] * suspension_damping
		
		# Store force for application
		_wheel_forces[i] = spring_force + damping_force
		
		# Emit suspension signal if significant compression
		if abs(compression) > SUSPENSION_COMPRESSION_LIMIT:
			suspension_compressed.emit(compression)

func _get_ground_height_at(position: Vector3) -> Vector3:
	# Simple ground plane detection
	var ray_end: Vector3 = position + Vector3.UP * 2.0
	var ray_start: Vector3 = position + Vector3.UP * 10.0
	var space_state: PhysicsDirectSpaceState3D = get_world_3d().direct_space_state
	var query: PhysicsRayQueryParameters3D = PhysicsRayQueryParameters3D.create(position, ray_end)
	var result: Dictionary = space_state.intersect_ray(query)
	
	if result.empty():
		return position + Vector3.DOWN * suspension_rest_length
	
	return result.position

# ============================================================================
# COLLISION HANDLING
# ============================================================================
func _handle_collisions(dt: float) -> void:
	var collisions: Array[Dictionary] = []
	
	# Simple collision detection using shape cast
	var space_state: PhysicsDirectSpaceState3D = get_world_3d().direct_space_state
	var query: PhysicsShapeQueryParameters3D = PhysicsShapeQueryParameters3D.new()
	query.shape = ShapeCreate.box(Vector3.ONE * 2.0)
	query.transform = Transform3D(global_position, global_rotation)
	
	var results: Array[Dictionary] = space_state.intersect_shape(query)
	
	for result in results:
		var collision_time: float = Time.get_unix_time_from_system()
		if collision_time - _last_collision_time > 0.1:
			collisions.append({
				"object": result.collider,
				"normal": result.normal,
				"position": result.position
			})
			_last_collision_time = collision_time
	
	if not collisions.empty():
		for collision in collisions:
			collision_detected.emit({
				"object": collision.object,
				"normal": collision.normal,
				"velocity": _current_speed,
				"impact_force": abs(_current_speed * vehicle_mass)
			})
			
			# Reduce speed on impact
			var impact_reduction: float = collision.impact_force * 0.1
			_current_speed = max(0.0, _current_speed - impact_reduction)

# ============================================================================
# DRIFT MECHANICS
# ============================================================================
func _trigger_drift_attempt() -> void:
	# Check if conditions are met for drift
	if abs(_current_speed) < 15.0:
		return
	
	# Calculate lateral velocity component
	var lateral_velocity: float = abs(_current_speed * sin(_steering_input * PI / 2.0))
	
	if lateral_velocity > 10.0:
		_is_drifting = true
		_drift_intensity = min(lateral_velocity / 20.0, DRIFT_INTENSITY_MAX)
		drift_started.emit(_drift_intensity)

func _update_drift(dt: float) -> void:
	if not _is_drifting:
		return
	
	# Gradually reduce drift intensity
	_drift_intensity = max(0.0, _drift_intensity - dt * 0.5)
	
	if _drift_intensity <= 0.1:
		_is_drifting = false
		_drift_intensity = 0.0
		drift_ended.emit()
	
	# Apply drift effects (reduce grip)
	if _drift_intensity > 0:
		tire_friction = 1.1 * (1.0 - _drift_intensity * 0.5)

# ============================================================================
# TURBO SYSTEM
# ============================================================================
func _activate_turbo() -> void:
	if not turbo_enabled:
		return
	
	if _turbo_available and _turbo_charge_progress < 1.0:
		_turbo_active = true
		_turbo_spool_timer = TURBO_CHARGE_TIME
		_turbo_available = false

func _update_turbo(dt: float) -> void:
	if not turbo_enabled:
		return
	
	if _turbo_active:
		_turbo_spool_timer -= dt
		if _turbo_spool_timer <= 0:
			_turbo_active = false
			_turbo_charge_progress = 1.0
			_turbo_available = true
	else:
		# Recharge turbo
		if _turbo_charge_progress < 1.0:
			_turbo_charge_progress += dt * 0.3
			if _turbo_charge_progress >= 1.0:
				_turbo_charge_progress = 1.0
				_turbo_available = true

# ============================================================================
# LAUNCH CONTROL
# ============================================================================
func _trigger_launch_control() -> void:
	if not launch_control_enabled:
		return
	
	if _current_gear == 1 and _current_speed == 0:
		_launch_control_triggered = true
		_launch_control_hold_time = 0.0
		_launch_control_active = true

func _update_launch_control(dt: float) -> void:
	if not _launch_control_active:
		return
	
	_launch_control_hold_time += dt
	
	if _launch_control_hold_time >= 2.0:
		_launch_control_active = false
		_launch_control_triggered = false
		# Release clutch for optimal launch
		_clutch_pedal = 1.0

# ============================================================================
# DISTANCE & LAP TRACKING
# ============================================================================
func _update_distance_traveled(dt: float) -> void:
	var previous_position: Vector3 = global_position
	var movement: float = abs(global_position.distance_to(previous_position))
	_total_distance_traveled += movement * dt
	
	# Update lap timing
	if _check_lap_complete():
		_lap_count += 1
		lap_completed.emit({"lap_number": _lap_count, "distance": _total_distance_traveled})

func _check_lap_complete() -> bool:
	# Simplified lap completion check
	var distance_to_start: float = global_position.distance_to(_lap_start_position)
	
	if distance_to_start < 50.0 and _checkpoint_passed:
		return true
	
	# Reset checkpoint when passing start line
	if distance_to_start < 10.0:
		_checkpoint_passed = true
	
	return false

# ============================================================================
# GEAR SHIFT LOGIC
# ============================================================================
func _check_gear_shifts(dt: float) -> void:
	if _in_neutral:
		return
	
	# Automatic upshift logic
	if _current_rpm > SHIFT_RPM and _current_gear < MAX_GEAR:
		_request_gear_shift(1)
	
	# Downshift logic for engine braking
	if _current_rpm < IDLE_RPM + 500 and _current_gear > 1:
		_request_gear_shift(-1)

# ============================================================================
# SIGNAL EMITTING
# ============================================================================
func _emit_signals() -> void:
	# Speed change
	speed_changed.emit(_current_speed * METERS_TO_KMH)
	
	# RPM change
	rpm_changed.emit(_current_rpm)
	
	# Engine sound pitch based on RPM
	_engine_pitch_modifier = _current_rpm / REDLINE_RPM
	engine_sound_changed.emit(_engine_pitch_modifier)

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================
func get_current_speed() -> float:
	return _current_speed * METERS_TO_KMH

func get_current_rpm() -> float:
	return _current_rpm

func get_current_gear() -> int:
	return _current_gear

func is_drifting() -> bool:
	return _is_drifting

func is_turbo_available() -> bool:
	return _turbo_available

func reset_vehicle() -> void:
	_current_speed = 0.0
	_current_rpm = IDLE_RPM
	_current_gear = 0
	_in_neutral = true
	_total_distance_traveled = 0.0
	_lap_count = 0
	_checkpoint_passed = false
	_is_drifting = false
	_turbo_active = false
	_turbo_available = true
	_launch_control_active = false
	_lap_start_position = global_position
	position = Vector3.ZERO
	rotation = Vector3.ZERO

func set_vehicle_position(pos: Vector3) -> void:
	global_position = pos

func set_vehicle_rotation(rot: Vector3) -> void:
	global_rotation = rot

func apply_force(force: Vector3) -> void:
	add_force(force)

func apply_impulse(impulse: Vector3) -> void:
	add_impulse(impulse)

func set_motor_power(power: float) -> void:
	_torque_output = power

func set_brake_force(force: float) -> void:
	_brake_input = clamp(force, 0.0, 1.0)

func set_steering(angle: float) -> void:
	_steering_input = clamp(angle, -1.0, 1.0)

func resume_audio_context() -> void:
	if AudioManager.has_method("resume_audio"):
		AudioManager.resume_audio()

# ============================================================================
# DESTRUCTOR CLEANUP
# ============================================================================
func _exit_tree() -> void:
	# Clean up any pending timers or processes
	pass