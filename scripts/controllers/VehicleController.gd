extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulation
## Handles throttle, brake, steering, gear shifting, wheel forces, and vehicle dynamics
## Integrates with PhysicsSettings singleton for all physics constants
## Copyright 2026 Thalamus Racing Simulator Project

# ============================================================================
# SIGNALS - Events emitted by this controller
# ============================================================================
signal speed_changed(new_speed: float)
signal rpm_changed(new_rpm: float)
signal gear_changed(old_gear: int, new_gear: int)
signal vehicle_moved(position: Vector3, velocity: Vector3)
signal collision_detected(collision_data: Dictionary)
signal engine_started()
signal engine_stopped()
signal handbrake_toggled(is_active: bool)
signal drift_started(angle: float)
signal drift_ended()
signal lap_completed(lap_time: float)
signal checkpoint_passed(checkpoint_id: int)

# ============================================================================
# DRIVETRAIN TYPES
# ============================================================================
enum DrivetrainType {
	FWD,  # Front-Wheel Drive
	RWD,  # Rear-Wheel Drive
	AWD   # All-Wheel Drive
}

enum GearState {
	NEUTRAL = 0,
	REVERSE = -1,
	FIRST = 1,
	SECOND = 2,
	THIRD = 3,
	FOURTH = 4,
	FIFTH = 5,
	SIXTH = 6,
	SEVENTH = 7,
	EIGHTH = 8,
	NINTH = 9,
	TENTH = 10
}

# ============================================================================
# CONSTANTS & CONFIGURATION
# ============================================================================
const VEHICLE_BASE_MASS := 1500.0
const MIN_RPM := 800.0
const IDLE_RPM := 1000.0
const MAX_RPM := 8000.0
const REDLINE_RPM := 7500.0
const GEAR_RATIO_FIRST := 3.8
const GEAR_RATIO_SECOND := 2.2
const GEAR_RATIO_THIRD := 1.5
const GEAR_RATIO_FOURTH := 1.1
const GEAR_RATIO_FIFTH := 0.9
const GEAR_RATIO_SIXTH := 0.75
const FINAL_DRIVE := 4.1
const WHEEL_RADIUS := 0.32
const STEERING_SPEED := 45.0
const TRACTION_CONTROL_STRENGTH := 0.95
const ABS_STRENGTH := 0.90

# ============================================================================
# PUBLIC PROPERTIES - Exposed for inspector and external access
# ============================================================================
@export_group("Vehicle Configuration")
@export var vehicle_mass: float = 1500.0: set = _set_vehicle_mass
@export var max_speed_kmh: float = 320.0: set = _set_max_speed_kmh
@export var acceleration_force: float = 8000.0: set = _set_acceleration_force
@export var braking_force: float = 15000.0: set = _set_braking_force
@export var steering_angle_max: float = 45.0: set = _set_steering_angle_max

@export_group("Drivetrain Settings")
@export var drivetrain_type: DrivetrainType = DrivetrainType.RWD
@export var differential_type: DifferentialType = DifferentialType.OPEN
@export var final_drive_ratio: float = 3.85

@export_group("Aerodynamics")
@export var drag_coefficient: float = 0.32
@export var frontal_area: float = 2.2
@export var downforce_coefficient: float = 0.5

@export_group("Tire Parameters")
@export var tire_friction_ground: float = 1.2
@export var tire_friction_air: float = 0.05
@export var tire_spring_rate: float = 50000.0
@export var tire_damping_rate: float = 5000.0

# ============================================================================
# PRIVATE STATE VARIABLES
# ============================================================================
var _speed_kmh: float = 0.0
var _rpm: float = IDLE_RPM
var _current_gear: GearState = GearState.NEUTRAL
var _throttle_input: float = 0.0
var _brake_input: float = 0.0
var _steering_input: float = 0.0
var _handbrake_active: bool = false
var _engine_running: bool = false
var _clutch_engaged: bool = true

# Steering angle (degrees)
var _current_steering_angle: float = 0.0
var _target_steering_angle: float = 0.0

# RPM control
var _target_rpm: float = IDLE_RPM

# Drift state
var _drift_angle: float = 0.0
var _is_drifting: bool = false
var _drift_threshold: float = 15.0

# Wheel states (simplified - 4 corners)
var _wheel_states: Array[Dictionary] = []

# Powertrain reference
var _powertrain: Node = null

# Physics references
var _physics_settings: PhysicsSettings
var _input_manager: InputManager

# Race data
var _race_distance: float = 0.0
var _lap_times: Array[float] = []
var _last_checkpoint: int = -1

# Timing variables
var _accumulated_time: float = 0.0
var _frame_count: int = 0
var _last_update_time: float = 0.0

# ============================================================================
# LATCHED SETTINGS
# ============================================================================
enum DifferentialType {
	OPEN,
	LSD,      # Limited Slip Differential
	LOCKING   # Locking Differential
}

# ============================================================================
# INITIALIZATION
# ============================================================================
func _ready() -> void:
	_init_singleton_references()
	_setup_wheel_states()
	_connect_signals()
	_reset_vehicle()
	print("[VehicleController] Ready - %s loaded" % get_path())

func _init_singleton_references() -> void:
	if Engine.has_singleton("PhysicsSettings"):
		_physics_settings = Engine.get_singleton("PhysicsSettings")
	else:
		_physics_settings = preload("res://scripts/core/PhysicsSettings.gd").new()
	
	if GameManager.has_singleton("InputManager"):
		_input_manager = GameManager.InputManager

func _setup_wheel_states() -> void:
	_wheel_states = [
		{"name": "FL", "position": Vector3(-1.0, 0.0, 0.5), "radius": WHEEL_RADIUS},
		{"name": "FR", "position": Vector3(1.0, 0.0, 0.5), "radius": WHEEL_RADIUS},
		{"name": "RL", "position": Vector3(-1.0, 0.0, -0.5), "radius": WHEEL_RADIUS},
		{"name": "RR", "position": Vector3(1.0, 0.0, -0.5), "radius": WHEEL_RADIUS}
	]

func _connect_signals() -> void:
	if _input_manager:
		_input_manager.throttle_requested.connect(_on_throttle_requested)
		_input_manager.brake_requested.connect(_on_brake_requested)
		_input_manager.steer_left.connect(_on_steer_left)
		_input_manager.steer_right.connect(_on_steer_right)
		_input_manager.gear_up_requested.connect(_shift_up)
		_input_manager.gear_down_requested.connect(_shift_down)
		_input_manager.handbrake_toggled.connect(_toggle_handbrake)
		_input_manager.engine_start_requested.connect(_start_engine)
		_input_manager.engine_stop_requested.connect(_stop_engine)

func _process(delta: float) -> void:
	_frame_count += 1
	_accumulated_time += delta
	
	_update_inputs(delta)
	_update_engine_rpm(delta)
	_update_steering(delta)
	_apply_forces(delta)
	_handle_collision_detection(delta)
	_emit_signals()

func _physics_process(delta: float) -> void:
	_update_vehicle_velocity(delta)
	_handle_gravity(delta)
	_handle_ground_contact(delta)

# ============================================================================
# INPUT HANDLING
# ============================================================================
func _update_inputs(delta: float) -> void:
	if not _input_manager:
		return
	
	# Get raw inputs
	_throttle_input = _input_manager.get_throttle()
	_brake_input = _input_manager.get_brake()
	var steer_raw = _input_manager.get_steering()
	
	# Apply input smoothing
	_throttle_input = _smooth_input(_throttle_input, delta, 0.1)
	_brake_input = _smooth_input(_brake_input, delta, 0.1)
	_target_steering_angle = steer_raw * steering_angle_max

func _smooth_input(value: float, delta: float, factor: float) -> float:
	return lerp(value, value, 1.0 - exp(-factor * delta * 60.0))

func _on_throttle_requested(amount: float) -> void:
	_throttle_input = clamp(amount, 0.0, 1.0)

func _on_brake_requested(amount: float) -> void:
	_brake_input = clamp(amount, 0.0, 1.0)

func _on_steer_left() -> void:
	_target_steering_angle -= STEERING_SPEED

func _on_steer_right() -> void:
	_target_steering_angle += STEERING_SPEED

func _toggle_handbrake() -> void:
	_handbrake_active = !_handbrake_active
	handbrake_toggled.emit(_handbrake_active)

func _start_engine() -> void:
	if not _engine_running:
		_engine_running = true
		_current_gear = GearState.NEUTRAL
		_rpm = IDLE_RPM
		engine_started.emit()
		if AudioManager:
			AudioManager.play_sound("engine_start")

func _stop_engine() -> void:
	if _engine_running:
		_engine_running = false
		_rpm = MIN_RPM
		engine_stopped.emit()
		if AudioManager:
			AudioManager.play_sound("engine_stop")

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================
func _shift_up() -> void:
	if not _can_shift():
		return
	
	var old_gear = _current_gear
	var new_gear = min(int(_current_gear) + 1, int(GearState.TENTH))
	
	if new_gear > int(GearState.FIRST):
		_current_gear = GearState(new_gear)
		gear_changed.emit(old_gear, _current_gear)
		
		if AudioManager:
			AudioManager.play_sound("gear_shift_up")

func _shift_down() -> void:
	if not _can_shift():
		return
	
	var old_gear = _current_gear
	var new_gear = max(int(_current_gear) - 1, int(GearState.REVERSE))
	
	if new_gear >= int(GearState.FIRST):
		_current_gear = GearState(new_gear)
		gear_changed.emit(old_gear, _current_gear)
		
		if AudioManager:
			AudioManager.play_sound("gear_shift_down")

func _can_shift() -> bool:
	if _current_gear == GearState.NEUTRAL:
		return true
	if _current_gear == GearState.REVERSE and _speed_kmh > 5.0:
		return false
	return true

func shift_to_gear(gear: GearState) -> void:
	var old_gear = _current_gear
	_current_gear = gear
	gear_changed.emit(old_gear, _current_gear)

func get_current_gear() -> GearState:
	return _current_gear

func get_gear_ratio() -> float:
	match _current_gear:
		GearState.NEUTRAL: return 0.0
		GearState.REVERSE: return GEAR_RATIO_FIRST * -1.0
		GearState.FIRST: return GEAR_RATIO_FIRST
		GearState.SECOND: return GEAR_RATIO_SECOND
		GearState.THIRD: return GEAR_RATIO_THIRD
		GearState.FOURTH: return GEAR_RATIO_FOURTH
		GearState.FIFTH: return GEAR_RATIO_FIFTH
		GearState.SIXTH: return GEAR_RATIO_SIXTH
		_: return GEAR_RATIO_FIRST

func get_optimal_gear(rpm: float, speed_kmh: float) -> GearState:
	var ratios = [GEAR_RATIO_FIRST, GEAR_RATIO_SECOND, GEAR_RATIO_THIRD, 
	              GEAR_RATIO_FOURTH, GEAR_RATIO_FIFTH, GEAR_RATIO_SIXTH]
	var optimal_gear = GearState.FIRST
	
	for i in range(ratios.size()):
		var theoretical_speed = _calculate_theoretical_speed(rpm, ratios[i])
		if theoretical_speed >= speed_kmh and theoretical_speed < speed_kmh * 1.5:
			optimal_gear = GearState(i + 1)
			break
	
	return optimal_gear

func _calculate_theoretical_speed(rpm: float, gear_ratio: float) -> float:
	var wheel_rpm = rpm / (gear_ratio * final_drive_ratio)
	var wheel_circumference = 2.0 * PI * WHEEL_RADIUS
	var speed_ms = (wheel_rpm / 60.0) * wheel_circumference
	return speed_ms * 3.6  # Convert to km/h

# ============================================================================
# ENGINE RPM CONTROL
# ============================================================================
func _update_engine_rpm(delta: float) -> void:
	if not _engine_running:
		_rpm = lerp(_rpm, MIN_RPM, delta * 10.0)
		return
	
	# Calculate target RPM based on throttle and gear
	var gear_ratio = get_gear_ratio()
	
	if gear_ratio != 0:
		# Engine driven by wheels
		var wheel_speed_ms = _speed_kmh / 3.6
		var wheel_rotation_per_sec = wheel_speed_ms / (2.0 * PI * WHEEL_RADIUS)
		var wheel_rpm = wheel_rotation_per_sec * 60.0
		_target_rpm = wheel_rpm * abs(gear_ratio) * final_drive_ratio
	else:
		# Neutral - use throttle to determine RPM
		_target_rpm = IDLE_RPM + (_throttle_input * (MAX_RPM - IDLE_RPM))
	
	# Smooth RPM transition
	_rpm = lerp(_rpm, _clamp_rpm(_target_rpm), delta * 15.0)
	
	# Redline protection
	if _rpm > REDLINE_RPM:
		_rpm = lerp(_rpm, REDLINE_RPM, delta * 20.0)

func _clamp_rpm(rpm: float) -> float:
	return clamp(rpm, MIN_RPM, MAX_RPM)

# ============================================================================
# STEERING CONTROL
# ============================================================================
func _update_steering(delta: float) -> void:
	_current_steering_angle = lerp(_current_steering_angle, _target_steering_angle, delta * STEERING_SPEED)

func get_steering_angle() -> float:
	return _current_steering_angle

func set_steering_angle_direct(angle: float) -> void:
	_current_steering_angle = clamp(angle, -steering_angle_max, steering_angle_max)
	_target_steering_angle = _current_steering_angle

# ============================================================================
# PHYSICS FORCES APPLICATION
# ============================================================================
func _apply_forces(delta: float) -> void:
	if not _engine_running:
		return
	
	var forward_vector = global_transform.basis.z * -1.0
	var drive_force = _calculate_drive_force()
	
	# Apply drive force to appropriate wheels based on drivetrain
	match drivetrain_type:
		DrivetrainType.FWD:
			_apply_wheel_force(_wheel_states[0], drive_force, forward_vector)
			_apply_wheel_force(_wheel_states[1], drive_force, forward_vector)
		DrivetrainType.RWD:
			_apply_wheel_force(_wheel_states[2], drive_force, forward_vector)
			_apply_wheel_force(_wheel_states[3], drive_force, forward_vector)
		DrivetrainType.AWD:
			_apply_wheel_force(_wheel_states[0], drive_force * 0.6, forward_vector)
			_apply_wheel_force(_wheel_states[1], drive_force * 0.6, forward_vector)
			_apply_wheel_force(_wheel_states[2], drive_force * 0.4, forward_vector)
			_apply_wheel_force(_wheel_states[3], drive_force * 0.4, forward_vector)
	
	# Apply braking force
	if _brake_input > 0.0 or _handbrake_active:
		var total_brake = _brake_input + (_handbrake_active ? 1.0 : 0.0)
		var brake_force = braking_force * total_brake
		
		for wheel in _wheel_states:
			_apply_brake_force(wheel, brake_force)

func _calculate_drive_force() -> float:
	var gear_ratio = get_gear_ratio()
	if gear_ratio == 0:
		return 0.0
	
	var torque_at_wheels = (acceleration_force * _throttle_input) / (abs(gear_ratio) * final_drive_ratio)
	return torque_at_wheels

func _apply_wheel_force(wheel: Dictionary, force: float, direction: Vector3) -> void:
	if force <= 0:
		return
	
	var world_position = global_position + global_transform.basis * wheel["position"]
	var force_direction = direction.normalized() * force
	
	apply_impulse(force_direction, Vector3.ZERO)
	wheel["applied_force"] = force

func _apply_brake_force(wheel: Dictionary, brake_force: float) -> void:
	wheel["brake_force"] = brake_force

# ============================================================================
# VEHICLE VELOCITY & MOVEMENT
# ============================================================================
func _update_vehicle_velocity(delta: float) -> void:
	# Update speed from velocity
	var velocity_magnitude = velocity.length()
	_speed_kmh = velocity_magnitude * 3.6
	
	# Handle reverse
	if velocity.dot(global_transform.basis.z) < 0:
		_speed_kmh *= -1.0
	
	speed_changed.emit(abs(_speed_kmh))

func _handle_gravity(delta: float) -> void:
	if has_node("CollisionShape3D"):
		var gravity = _physics_settings.gravity if _physics_settings else 9.81
		velocity.y -= gravity * delta

func _handle_ground_contact(delta: float) -> void:
	var ground_normal = Vector3.UP
	var collision = move_and_collide(velocity * delta)
	
	if collision:
		ground_normal = collision.normal
		velocity = velocity.slide(collision.normal)

func _reset_vehicle() -> void:
	_speed_kmh = 0.0
	_rpm = MIN_RPM
	_current_gear = GearState.NEUTRAL
	_throttle_input = 0.0
	_brake_input = 0.0
	_steering_input = 0.0
	_handbrake_active = false
	_is_drifting = false
	_drift_angle = 0.0

# ============================================================================
# AERODYNAMICS & DRAG CALCULATION
# ============================================================================
func calculate_aerodynamic_drag() -> float:
	var air_density = 1.225  # kg/m^3 at sea level
	var drag_force = 0.5 * air_density * drag_coefficient * frontal_area * (_speed_kmh / 3.6) ** 2
	return drag_force

func calculate_downforce() -> float:
	var air_density = 1.225
	var downforce = 0.5 * air_density * downforce_coefficient * frontal_area * (_speed_kmh / 3.6) ** 2
	return downforce

# ============================================================================
# DRIFT MECHANICS
# ============================================================================
func _detect_drift() -> void:
	var drift_indicator = abs(_get_lateral_slip_angle())
	
	if drift_indicator > drift_threshold:
		if not _is_drifting:
			_is_drifting = true
			drift_started.emit(drift_indicator)
			if AudioManager:
				AudioManager.play_sound("drift_start")
	elif _is_drifting:
		_is_drifting = false
		drift_ended.emit()
		if AudioManager:
			AudioManager.play_sound("drift_end")

func _get_lateral_slip_angle() -> float:
	var speed = velocity.length()
	if speed < 0.1:
		return 0.0
	
	var forward = global_transform.basis.z * -1.0
	var lateral = forward.cross(velocity).length()
	return asin(lateral / speed) * 180.0 / PI

# ============================================================================
# COLLISION DETECTION
# ============================================================================
func _handle_collision_detection(delta: float) -> void:
	var collisions = get_overlapping_bodies()
	
	for body in collisions:
		var collision_data = {
			"body": body,
			"collision_point": transform.origin,
			"relative_velocity": velocity
		}
		
		collision_detected.emit(collision_data)
		
		if body.has_method("take_damage"):
			body.take_damage(_calculate_collision_damage(body))

func _calculate_collision_damage(other_body: Node) -> float:
	var impact_speed = velocity.length()
	var damage = impact_speed * impact_speed * 0.01
	return min(damage, 100.0)

# ============================================================================
# RACE SYSTEM INTEGRATION
# ============================================================================
func update_race_distance(distance: float) -> void:
	_race_distance += distance

func get_total_race_distance() -> float:
	return _race_distance

func record_lap_time(time_seconds: float) -> void:
	_lap_times.append(time_seconds)

func get_lap_times() -> Array[float]:
	return _lap_times

func get_best_lap_time() -> float:
	if _lap_times.is_empty():
		return 0.0
	return _lap_times.min()

func get_average_lap_time() -> float:
	if _lap_times.is_empty():
		return 0.0
	var sum = 0.0
	for t in _lap_times:
		sum += t
	return sum / _lap_times.size()

func record_checkpoint(checkpoint_id: int) -> void:
	if checkpoint_id > _last_checkpoint:
		_last_checkpoint = checkpoint_id
		checkpoint_passed.emit(checkpoint_id)

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================
func get_speed_kmh() -> float:
	return abs(_speed_kmh)

func get_rpm() -> float:
	return _rpm

func is_engine_running() -> bool:
	return _engine_running

func is_drifting() -> bool:
	return _is_drifting

func get_drift_angle() -> float:
	return _drift_angle

func set_vehicle_mass(mass: float) -> void:
	vehicle_mass = mass
	rigidBody.mass = mass if rigidBody else mass

func set_max_speed(speed: float) -> void:
	max_speed_kmh = speed

func reset_all_stats() -> void:
	_race_distance = 0.0
	_lap_times.clear()
	_last_checkpoint = -1

# ============================================================================
# DEBUG TOOLS
# ============================================================================
func debug_print_state() -> void:
	print("[VehicleDebug]")
	print("  Speed: %.2f km/h" % _speed_kmh)
	print("  RPM: %.0f" % _rpm)
	print("  Gear: %s" % _current_gear)
	print("  Throttle: %.2f" % _throttle_input)
	print("  Brake: %.2f" % _brake_input)
	print("  Steering: %.2f°" % _current_steering_angle)
	print("  Handbrake: %s" % _handbrake_active)

func enable_debug_mode(enabled: bool) -> void:
	if enabled:
		Engine.print_verbose = true
	else:
		Engine.print_verbose = false

# ============================================================================
# DESTRUCTOR
# ============================================================================
func _exit_tree() -> void:
	_log_cleanup()

func _log_cleanup() -> void:
	print("[VehicleController] Cleanup completed for %s" % get_path())

# ============================================================================
# EXPORTED METHODS FOR EXTERNAL ACCESS
# ============================================================================
func get_vehicle_data() -> Dictionary:
	return {
		"speed_kmh": _speed_kmh,
		"rpm": _rpm,
		"gear": _current_gear,
		"throttle": _throttle_input,
		"brake": _brake_input,
		"steering": _current_steering_angle,
		"handbrake": _handbrake_active,
		"engine_running": _engine_running,
		"is_drifting": _is_drifting,
		"race_distance": _race_distance,
		"lap_times": _lap_times
	}

func apply_replay_data(data: Dictionary) -> void:
	_speed_kmh = data.get("speed_kmh", 0.0)
	_rpm = data.get("rpm", IDLE_RPM)
	_current_gear = data.get("gear", GearState.NEUTRAL)
	_throttle_input = data.get("throttle", 0.0)
	_brake_input = data.get("brake", 0.0)
	_current_steering_angle = data.get("steering", 0.0)
	_handbrake_active = data.get("handbrake", false)
	_race_distance = data.get("race_distance", 0.0)

# ============================================================================
# AUDIO INTEGRATION
# ============================================================================
func update_engine_audio() -> void:
	if not AudioManager:
		return
	
	var pitch_variation = (_rpm / MAX_RPM) * 0.5 + 0.5
	AudioManager.set_pitch_modulation("engine", pitch_variation)
	AudioManager.set_volume_modulation("engine", _throttle_input * 0.8 + 0.2)

func play_collision_sound() -> void:
	if AudioManager:
		AudioManager.play_sound("collision")

func play_crash_sound() -> void:
	if AudioManager:
		AudioManager.play_sound("crash")

func play_skid_sound() -> void:
	if _is_drifting and AudioManager:
		AudioManager.play_sound("skid")

# ============================================================================
# AUTO-DRIVE ASSISTANT
# ============================================================================
func activate_auto_drive(target_speed: float = 0.0) -> void:
	_auto_drive_enabled = true
	_auto_drive_target_speed = target_speed

func deactivate_auto_drive() -> void:
	_auto_drive_enabled = false

func update_auto_drive(delta: float) -> void:
	if not _auto_drive_enabled:
		return
	
	var speed_diff = _auto_drive_target_speed - _speed_kmh
	if speed_diff > 1.0:
		_throttle_input = 1.0
	elif speed_diff < -1.0:
		_brake_input = 1.0
	else:
		_throttle_input = 0.5

var _auto_drive_enabled: bool = false
var _auto_drive_target_speed: float = 0.0

</FILE>