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

# ============================================================================
# EXPORTED CONFIGURATION - Vehicle Setup (Exposed in Inspector)
# ============================================================================
@export_group("Vehicle Configuration")
@export var vehicle_mass: float = 1500.0
@export var center_of_gravity_height: float = 0.55
@export var track_width: float = 1.6
@export var wheelbase: float = 2.6
@export var drag_coefficient: float = 0.32
@export var frontal_area: float = 2.2

@export_group("Powertrain Settings")
@export var engine_max_power: float = 350.0  # kW
@export var engine_max_torque: float = 500.0  # Nm
@export var transmission_type: String = "manual"  # manual, automatic, dual_clutch
@export var final_drive_ratio: float = 3.5
@export var differential_type: String = "limited_slip"

@export_group("Wheel Configuration")
@export var front_wheel_radius: float = 0.32
@export var rear_wheel_radius: float = 0.32
@export var wheel_inertia: float = 1.5
@export var tire_friction_coefficient: float = 1.2

@export_group("Drift & Handling")
@export var grip_level: float = 0.95
@export var oversteer_bias: float = 0.1
@export var understeer_bias: float = 0.1
@export var anti_roll_bar_stiffness: float = 0.8
@export var steering_angle_max: float = 0.5  # radians (~28 degrees)

@export_group("Aerodynamics")
@export var downforce_coefficient: float = 0.15
@export var wing_angle: float = 5.0  # degrees
@export var ground_effect_factor: float = 0.25

@export_group("Suspension Settings")
@export var spring_stiffness: float = 50000.0
@export var damping_rebound: float = 3000.0
@export var damping_compression: float = 1500.0
@export var ride_height: float = 0.12

# ============================================================================
# PRIVATE STATE VARIABLES
# ============================================================================
var _current_speed_kmh: float = 0.0
var _current_rpm: float = IDLE_RPM
var _current_gear: int = 0
var _target_gear: int = 0
var _clutch_engaged: bool = true
var _throttle_input: float = 0.0
var _brake_input: float = 0.0
var _steering_input: float = 0.0
var _drift_mode: bool = false
var _drift_intensity: float = 0.0
var _turbo_active: bool = false
var _turbo_charge_level: float = 0.0
var _suspension_states: Array[float] = [0.0, 0.0, 0.0, 0.0]  # FL, FR, RL, RR
var _acceleration_vector: Vector3 = Vector3.ZERO
var _angular_velocity: Vector3 = Vector3.ZERO
var _collision_history: Array[Dictionary] = []
var _lap_start_time: float = 0.0
var _current_lap: int = 0
var _checkpoint_positions: Array[Vector3] = []
var _last_checkpoint_index: int = -1

# ============================================================================
# INITIALIZATION
# ============================================================================
func _ready() -> void:
	_init_suspension_states()
	_setup_collision_detection()
	_reset_vehicle_state()
	print_debug("VehicleController initialized successfully")

func _init_suspension_states() -> void:
	for i in range(4):
		_suspension_states[i] = ride_height

func _setup_collision_detection() -> void:
	pass  # Collision detection set up via scene tree

func _reset_vehicle_state() -> void:
	_current_speed_kmh = 0.0
	_current_rpm = IDLE_RPM
	_current_gear = 0
	_target_gear = 0
	_throttle_input = 0.0
	_brake_input = 0.0
	_steering_input = 0.0
	_drift_mode = false
	_drift_intensity = 0.0
	_turbo_active = false
	_turbo_charge_level = 0.0
	_acceleration_vector = Vector3.ZERO

# ============================================================================
# INPUT HANDLING
# ============================================================================
func handle_input(delta: float) -> void:
	# Get input from InputManager singleton
	if GameManager.instance and GameManager.instance.input_manager:
		_throttle_input = GameManager.instance.input_manager.get_axis("throttle", "brake")
		_brake_input = GameManager.instance.input_manager.get_axis("brake", "handbrake")
		_steering_input = GameManager.instance.input_manager.get_axis("steer_left", "steer_right")

# ============================================================================
# PHYSICS UPDATE LOOP
# ============================================================================
func _physics_process(delta: float) -> void:
	# Validate delta time
	if delta > 0.05:
		return
	
	# Handle input
	handle_input(delta)
	
	# Update vehicle state
	_update_engine_and_gear(delta)
	_calculate_acceleration(delta)
	_apply_forces_and_movement(delta)
	_update_suspension(delta)
	_check_drift_conditions(delta)
	_update_audio_signals()
	_emit_signals()

# ============================================================================
# ENGINE AND GEAR MANAGEMENT
# ============================================================================
func _update_engine_and_gear(delta: float) -> void:
	if not _clutch_engaged:
		return
	
	var gear_ratios: Dictionary = {
		0: 0.0,      # Neutral
		1: 3.5,
		2: 2.5,
		3: 1.8,
		4: 1.3,
		5: 1.0,
		6: 0.85
	}
	
	var ratio = gear_ratios[_current_gear] if _current_gear > 0 else 0.0
	
	# Calculate RPM based on speed and gear
	if _current_speed_kmh > 0 and _current_gear > 0:
		var wheel_rotation_rps = (_current_speed_kmh / 3.6) / (2.0 * PI * front_wheel_radius)
		var wheel_shaft_rps = wheel_rotation_rps * final_drive_ratio
		_current_rpm = wheel_shaft_rps * 60.0 * ratio
	else:
		_current_rpm = lerp(_current_rpm, IDLE_RPM, delta * 10.0)
	
	# Auto-shift logic
	if _current_gear != _target_gear:
		_shift_gear(_target_gear, delta)
	
	# Check for redline
	if _current_rpm >= REDLINE_RPM:
		_target_gear = min(_current_gear + 1, MAX_GEAR)
	
	# Turbo charging
	if _turbo_active:
		_turbo_charge_level = min(_turbo_charge_level + delta * (1.0 / TURBO_CHARGE_TIME), 1.0)
	else:
		_turbo_charge_level = max(_turbo_charge_level - delta * 2.0, 0.0)

func _shift_gear(target_gear: int, delta: float) -> void:
	if target_gear < MIN_GEAR or target_gear > MAX_GEAR:
		return
	
	if not _clutch_engaged:
		_current_gear = target_gear
		emit_signal("gear_changed", _current_gear)
		return
	
	# Simulate clutch engagement time
	var shift_progress: float = 0.0
	shift_progress += delta / CLUTCH_RELEASE_TIME
	
	if shift_progress >= 1.0:
		_clutch_engaged = true
		_current_gear = target_gear
		emit_signal("gear_changed", _current_gear)
	else:
		_clutch_engaged = false

func change_gear(gear_direction: int) -> void:
	"""Change gear by direction (-1 down, 1 up)"""
	var new_gear = _current_gear + gear_direction
	new_gear = clamp(new_gear, MIN_GEAR, MAX_GEAR)
	_target_gear = new_gear

# ============================================================================
# PHYSICS CALCULATIONS
# ============================================================================
func _calculate_acceleration(delta: float) -> void:
	var air_density: float = 1.225  # kg/m^3 at sea level
	var speed_ms: float = _current_speed_kmh / 3.06
	var air_resistance: float = 0.5 * air_density * drag_coefficient * frontal_area * speed_ms * speed_ms
	
	var torque_at_wheels: float = engine_max_torque
	if _current_gear > 0:
		var gear_ratios: Dictionary = {
			1: 3.5, 2: 2.5, 3: 1.8, 4: 1.3, 5: 1.0, 6: 0.85
		}
		torque_at_wheels *= gear_ratios[_current_gear] * final_drive_ratio
	
	# Apply throttle effect
	var effective_torque = torque_at_wheels * _throttle_input
	if _turbo_active:
		effective_torque *= (1.0 + _turbo_charge_level * 0.3)
	
	# Convert torque to force
	var drive_force: float = effective_torque / (front_wheel_radius * wheelbase)
	drive_force -= air_resistance
	
	# Apply braking force
	if _brake_input > 0:
		drive_force -= BRAKING_FORCE * _brake_input
	
	# Clamp acceleration
	drive_force = clamp(drive_force, -BRAKING_FORCE * vehicle_mass, ACCELERATION_RATE * vehicle_mass)
	
	# Calculate actual acceleration
	_acceleration_vector = drive_force / vehicle_mass

func _apply_forces_and_movement(delta: float) -> void:
	# Apply acceleration to velocity
	var forward_velocity: float = _velocity.length()
	var lateral_velocity: float = Vector2(_velocity.x, _velocity.z).length()
	
	# Steering effects
	if abs(forward_velocity) > 1.0:
		var steer_effect: float = _steering_input * TURN_SPEED
		_angular_velocity.y = steer_effect * sign(forward_velocity)
		rotate_y(-_angular_velocity.y * delta)
	
	# Update speed
	var current_accel = _acceleration_vector.x
	_current_speed_kmh += current_accel * delta * 3.6
	_current_speed_kmh = clamp(_current_speed_kmh, -MAX_SPEED_KMH / 2.0, MAX_SPEED_KMH)
	
	# Apply friction/drag
	_current_speed_kmh *= (1.0 - 0.01 * delta)
	
	# Update position based on rotation
	var move_vector: Vector3 = transform.basis.z * (_current_speed_kmh / 3.6) * delta
	move_vector.x = 0.0  # Keep on ground plane
	position += move_vector
	
	# Update velocity for physics body
	_velocity.x = move_vector.x / delta
	_velocity.z = move_vector.z / delta

func _update_suspension(delta: float) -> void:
	# Simplified suspension calculation
	var vertical_accel: float = gravity * 0.1
	for i in range(4):
		var compression: float = _suspension_states[i] + vertical_accel * delta
		compression = clamp(compression, 0.0, SUSPENSION_COMPRESSION_LIMIT)
		_suspension_states[i] = compression
	
	# Emit suspension signal
	var avg_compression: float = sum(_suspension_states) / 4.0
	emit_signal("suspension_compressed", avg_compression)

func _check_drift_conditions(delta: float) -> void:
	var speed_ms: float = _current_speed_kmh / 3.6
	var turn_rate: float = abs(_angular_velocity.y)
	
	# Drift activation conditions
	var should_drift: bool = (speed_ms > 15.0) and (_brake_input > 0.5 or _throttle_input > 0.7)
	
	if should_drift and not _drift_mode:
		_drift_mode = true
		_drift_intensity = 0.0
		emit_signal("drift_started", 0.0)
	elif not should_drift and _drift_mode:
		_drift_mode = false
		_drift_intensity = 0.0
		emit_signal("drift_ended")
	
	# Drift intensity calculation
	if _drift_mode:
		var target_intensity: float = min(_brake_input + _throttle_input, 1.0) * DRIFT_THRESHOLD
		_drift_intensity = lerp(_drift_intensity, target_intensity, delta * 5.0)
		emit_signal("drift_started", _drift_intensity)

# ============================================================================
# AUDIO SIGNALS
# ============================================================================
func _update_audio_signals() -> void:
	var rpm_ratio: float = (_current_rpm - IDLE_RPM) / (REDLINE_RPM - IDLE_RPM)
	rpm_ratio = clamp(rpm_ratio, 0.0, 1.0)
	emit_signal("engine_sound_changed", rpm_ratio)

# ============================================================================
# SIGNAL EMITTERS
# ============================================================================
func _emit_signals() -> void:
	emit_signal("speed_changed", _current_speed_kmh)
	emit_signal("rpm_changed", _current_rpm)
	
	# Lap timing
	if GameManager.instance and GameManager.instance.current_state == GameManager.GameState.RACE_ACTIVE:
		if _lap_start_time == 0.0:
			_lap_start_time = Time.get_ticks_msec()
		
		# Simple lap completion check
		if position.distance_to(Vector3.ZERO) < 5.0 and _current_lap > 0:
			_complete_lap()

func _complete_lap() -> void:
	var lap_time: float = (Time.get_ticks_msec() - _lap_start_time) / 1000.0
	var lap_data: Dictionary = {
		"lap_number": _current_lap,
		"time": lap_time,
		"timestamp": Time.get_datetime_string_from_system()
	}
	emit_signal("lap_completed", lap_data)
	_current_lap += 1
	_lap_start_time = Time.get_ticks_msec()

# ============================================================================
# HELPER METHODS
# ============================================================================
func get_current_speed_kmh() -> float:
	return _current_speed_kmh

func get_current_rpm() -> float:
	return _current_rpm

func get_current_gear() -> int:
	return _current_gear

func is_drifting() -> bool:
	return _drift_mode

func get_drift_intensity() -> float:
	return _drift_intensity

func is_turbo_available() -> bool:
	return _turbo_charge_level >= 1.0

func activate_turbo() -> void:
	if _turbo_charge_level >= 1.0:
		_turbo_active = true
		_current_speed_kmh *= 1.15  # Temporary boost

func deactivate_turbo() -> void:
	_turbo_active = false

func reset_vehicle() -> void:
	_reset_vehicle_state()
	position = Vector3(0.0, 0.0, 0.0)
	rotation = Vector3.ZERO
	_velocity = Vector3.ZERO

func get_wheel_positions() -> Dictionary:
	"""Return world positions of all four wheels"""
	var half_track: float = track_width / 2.0
	var half_wheelbase: float = wheelbase / 2.0
	
	return {
		"front_left": Vector3(-half_track, 0.0, -half_wheelbase),
		"front_right": Vector3(half_track, 0.0, -half_wheelbase),
		"rear_left": Vector3(-half_track, 0.0, half_wheelbase),
		"rear_right": Vector3(half_track, 0.0, half_wheelbase)
	}

func calculate_cornering_force(angle: float) -> float:
	"""Calculate lateral grip force based on cornering angle"""
	return grip_level * tan(angle) * vehicle_mass * gravity

func apply_downforce(speed_kmh: float) -> float:
	"""Calculate downforce at given speed"""
	var speed_ms: float = speed_kmh / 3.6
	return downforce_coefficient * speed_ms * speed_ms * frontal_area * 0.5

# ============================================================================
# DEBUGGING TOOLS
# ============================================================================
func debug_log_message(message: String) -> void:
	if GameManager.instance and GameManager.instance.debug_mode:
		print("[VehicleController] ", message)

func debug_print_stats() -> void:
	debug_log_message("Speed: %.1f km/h | RPM: %.0f | Gear: %d | Drift: %.2f" % [
		_current_speed_kmh, _current_rpm, _current_gear, _drift_intensity
	])

# ============================================================================
# DESTRUCTOR
# ============================================================================
func _exit_tree() -> void:
	debug_log_message("VehicleController cleanup complete")
</file>