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
@export var wing_angle: float = 0.1

# ============================================================================
# PRIVATE VARIABLES - Internal State Management
# ============================================================================
# Vehicle state
var _current_speed_kmh: float = 0.0
var _current_rpm: float = IDLE_RPM
var _current_gear: int = 0
var _target_gear: int = 0
var _clutch_engaged: bool = true
var _clutch_target: float = 1.0
var _turbo_active: bool = false
var _turbo_timer: float = 0.0
var _drift_mode: bool = false
var _drift_intensity: float = 0.0
var _is_moving: bool = false

# Input states (normalized -1 to 1)
var _throttle_input: float = 0.0
var _brake_input: float = 0.0
var _clutch_input: float = 0.0
var _steering_input: float = 0.0

# Velocity and forces
var _velocity_vector: Vector3 = Vector3.ZERO
var _acceleration_vector: Vector3 = Vector3.ZERO
var _drag_force: float = 0.0
var _downforce: float = 0.0
var _wheel_forces: Dictionary = {
	"front_left": 0.0,
	"front_right": 0.0,
	"rear_left": 0.0,
	"rear_right": 0.0
}

# Suspension system
var _suspension_state: Dictionary = {
	"front_left": {"compression": 0.0, "velocity": 0.0},
	"front_right": {"compression": 0.0, "velocity": 0.0},
	"rear_left": {"compression": 0.0, "velocity": 0.0},
	"rear_right": {"compression": 0.0, "velocity": 0.0}
}

# Powertrain ratios
var _gear_ratios: Array[float] = [3.8, 2.2, 1.5, 1.1, 0.9, 0.75]
var _reverse_ratio: float = 3.5
var _idle_ratio: float = 1.0

# Reference frames
var _reference_frame: Transform3D = Transform3D.IDENTITY
var _forward_direction: Vector3 = Vector3.FORWARD
var _right_direction: Vector3 = Vector3.RIGHT
var _up_direction: Vector3 = Vector3.UP

# Timing and delta
var _delta_time: float = 0.0
var _last_collision_time: float = 0.0
var _lap_start_time: float = 0.0
var _distance_traveled: float = 0.0
var _lap_distance: float = 0.0

# Physics references
var _powertrain: Node = null
var _audio_manager: Node = null
var _physics_settings: PhysicsSettings = PhysicsSettings.new()

# ============================================================================
# PUBLIC PROPERTIES - Read-only Accessors
# ============================================================================
func get_current_speed() -> float: return _current_speed_kmh
func get_current_rpm() -> float: return _current_rpm
func get_current_gear() -> int: return _current_gear
func get_is_drifting() -> bool: return _drift_mode
func get_drift_intensity() -> float: return _drift_intensity
func get_vehicle_mass() -> float: return vehicle_mass
func get_acceleration_vector() -> Vector3: return _acceleration_vector
func get_velocity_vector() -> Vector3: return _velocity_vector
func get_distance_traveled() -> float: return _distance_traveled
func get_lap_distance() -> float: return _lap_distance

# ============================================================================
# INITIALIZATION - Setup and Connection
# ============================================================================
func _ready() -> void:
	_init_singletons()
	_load_physics_settings()
	_reset_vehicle_state()
	_setup_suspension()
	set_process(true)
	set_physics_process(true)
	
	if Engine.is_editor_hint():
		return
	
	print("[VehicleController] Initialized successfully")

func _init_singletons() -> void:
	_audio_manager = GameManager.get_node_or_null("/root/AudioManager")
	if not _audio_manager:
		_warning("AudioManager singleton not found")

func _load_physics_settings() -> void:
	# Use autoloaded PhysicsSettings if available
	var ps = GameManager.get_node_or_null("/root/PhysicsSettings")
	if ps and ps is PhysicsSettings:
		_physics_settings = ps
	else:
		# Create local instance with defaults
		_physics_settings.gravity = 9.81
		_physics_settings.physics_tick_rate = 120

func _reset_vehicle_state() -> void:
	_current_speed_kmh = 0.0
	_current_rpm = IDLE_RPM
	_current_gear = MIN_GEAR
	_target_gear = MIN_GEAR
	_clutch_engaged = true
	_clutch_target = 1.0
	_turbo_active = false
	_turbo_timer = 0.0
	_drift_mode = false
	_drift_intensity = 0.0
	_is_moving = false
	_velocity_vector = Vector3.ZERO
	_acceleration_vector = Vector3.ZERO
	_drag_force = 0.0
	_downforce = 0.0
	_distance_traveled = 0.0
	_lap_distance = 0.0
	_lap_start_time = Time.get_unix_time_from_system()
	
	# Reset wheel forces
	for key in _wheel_forces:
		_wheel_forces[key] = 0.0
	
	# Reset suspension
	for key in _suspension_state:
		_suspension_state[key].compression = 0.0
		_suspension_state[key].velocity = 0.0

func _setup_suspension() -> void:
	# Initialize suspension dampers and springs
	pass  # Can be expanded with spring-damper calculations

# ============================================================================
# INPUT HANDLING - Process player controls
# ============================================================================
func _input(event: InputEvent) -> void:
	if event is InputEventKey and event.pressed:
		_handle_input_keys(event)
	elif event is InputEventMouseButton and event.pressed:
		_handle_input_mouse(event)

func _handle_input_keys(event: InputEventKey) -> void:
	match event.keycode:
		KEY_W, KEY_UP:
			_throttle_input = 1.0
		KEY_S, KEY_DOWN:
			_brake_input = 1.0
		KEY_A, KEY_LEFT:
			_steering_input = -1.0
		KEY_D, KEY_RIGHT:
			_steering_input = 1.0
		KEY_SHIFT:
			_turbo_active = true
		KEY_C:
			_toggle_manual_gear()
		KEY_F:
			_shift_clutch()
		KEY_SPACE:
			_brake_input = 1.0

func _handle_input_mouse(event: InputEventMouseButton) -> void:
	match event.button_index:
		MOUSE_BUTTON_LEFT:
			_throttle_input = 1.0
		MOUSE_BUTTON_RIGHT:
			_brake_input = 1.0

func _process(delta: float) -> void:
	_delta_time = delta
	_update_input_states(delta)
	_update_gear_shifting()
	_update_clutch(delta)
	_update_turbo(delta)
	_check_drift_conditions(delta)
	_generate_engine_sound()

func _physics_process(delta: float) -> void:
	_delta_time = delta
	_apply_vehicle_dynamics()
	_update_suspension_system()
	_update_collision_detection()
	_update_position_and_rotation()
	_emit_signals()

# ============================================================================
# INPUT STATE MANAGEMENT
# ============================================================================
func _update_input_states(delta: float) -> void:
	# Smooth input transitions for better feel
	const SMOOTHING_FACTOR: float = 5.0 * delta
	
	_throttle_input = _smooth_value(_throttle_input, _get_raw_throttle(), SMOOTHING_FACTOR)
	_brake_input = _smooth_value(_brake_input, _get_raw_brake(), SMOOTHING_FACTOR)
	_steering_input = _smooth_value(_steering_input, _get_raw_steering(), SMOOTHING_FACTOR)
	_clutch_input = _smooth_value(_clutch_input, _get_raw_clutch(), SMOOTHING_FACTOR)

func _get_raw_throttle() -> float:
	var throttle = Input.get_axis(KEY_S, KEY_W) or Input.get_axis(KEY_DOWN, KEY_UP)
	if throttle == null:
		throttle = 0.0
	return clampf(throttle, 0.0, 1.0)

func _get_raw_brake() -> float:
	var brake = Input.get_axis(KEY_A, KEY_D) or Input.get_axis(KEY_LEFT, KEY_RIGHT)
	if brake == null:
		brake = 0.0
	return clampf(brake, 0.0, 1.0)

func _get_raw_steering() -> float:
	var steer = Input.get_axis(KEY_A, KEY_D) or Input.get_axis(KEY_LEFT, KEY_RIGHT)
	if steer == null:
		steer = 0.0
	return clampf(steer, -1.0, 1.0)

func _get_raw_clutch() -> float:
	var clutch = Input.get_key_pressed(KEY_F) ? 1.0 : 0.0
	return clutch

func _smooth_value(current: float, target: float, smoothing_factor: float) -> float:
	if abs(target - current) < 0.01:
		return target
	if target > current:
		return current + smoothing_factor
	else:
		return current - smoothing_factor

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================
func _update_gear_shifting() -> void:
	# Auto-shift logic based on RPM and throttle
	if transmission_type != "manual":
		_auto_shift_gear()
	
	# Manual shift override
	if _clutch_input > 0.5:
		_manual_shift_request()

func _auto_shift_gear() -> void:
	var target_gear = _calculate_auto_gear()
	if target_gear != _current_gear:
		_change_gear(target_gear)

func _calculate_auto_gear() -> int:
	var rpm_ratio = _current_rpm / REDLINE_RPM
	var throttle = _throttle_input
	
	if throttle < 0.1:
		if _current_speed_kmh < 5.0:
			return MIN_GEAR
		elif _current_rpm < IDLE_RPM + 200.0:
			return _max(0, _current_gear - 1)
	
	if _current_rpm < SHIFT_RPM * 0.7:
		if _current_gear < MAX_GEAR:
			return _current_gear + 1
	elif _current_rpm > SHIFT_RPM * 1.1:
		if _current_gear > MIN_GEAR:
			return _current_gear - 1
	
	return _current_gear

func _manual_shift_request() -> void:
	if Input.is_key_pressed(KEY_COMMA):
		if _current_gear > MIN_GEAR:
			_change_gear(_current_gear - 1)
	elif Input.is_key_pressed(KEY_PERIOD):
		if _current_gear < MAX_GEAR:
			_change_gear(_current_gear + 1)

func _toggle_manual_gear() -> void:
	transmission_type = "manual" if transmission_type == "automatic" else "automatic"
	print("[VehicleController] Transmission mode toggled: ", transmission_type)

func _shift_clutch() -> void:
	_clutch_input = 1.0 if _clutch_input < 0.5 else 0.0

func _change_gear(new_gear: int) -> void:
	if new_gear < MIN_GEAR or new_gear > MAX_GEAR:
		return
	
	if new_gear == _current_gear:
		return
	
	_target_gear = new_gear
	_clutch_engaged = false
	_clutch_target = 0.0
	
	gear_changed.emit(new_gear)
	
	# Schedule clutch re-engagement after shift
	await get_tree().create_timer(CLUTCH_RELEASE_TIME).timeout
	_clutch_target = 1.0

func _update_clutch(delta: float) -> void:
	if not _clutch_engaged:
		_clutch_target = lerpf(_clutch_target, 1.0, delta * 10.0)
		
		if _clutch_target >= 1.0:
			_clutch_engaged = true
			if _target_gear != _current_gear:
				_current_gear = _target_gear
				_target_gear = _current_gear

# ============================================================================
# TURBO SYSTEM
# ============================================================================
func _update_turbo(delta: float) -> void:
	if _turbo_active:
		_turbo_timer += delta
		
		if _turbo_timer >= TURBO_CHARGE_TIME:
			_apply_turbo_boost()
			_turbo_timer = 0.0
			_turbo_active = false

func _apply_turbo_boost() -> void:
	var boost_multiplier: float = 1.3
	_current_speed_kmh *= boost_multiplier
	_current_rpm *= boost_multiplier
	
	race_event.emit("turbo_boost", {
		"speed_kmh": _current_speed_kmh,
		"rpm": _current_rpm
	})

func activate_turbo() -> void:
	_turbo_active = true
	_turbo_timer = 0.0

# ============================================================================
# DRIFT MECHANICS
# ============================================================================
func _check_drift_conditions(delta: float) -> void:
	var speed_ratio = _current_speed_kmh / MAX_SPEED_KMH
	var turn_intensity = abs(_steering_input)
	var lateral_velocity = _velocity_vector.x.length()
	
	var drift_threshold = DRIFT_THRESHOLD * (1.0 - grip_level)
	
	if speed_ratio > 0.5 and turn_intensity > 0.5 and lateral_velocity > 2.0:
		_enter_drift_mode()
	else:
		_exit_drift_mode()

func _enter_drift_mode() -> void:
	if not _drift_mode:
		_drift_mode = true
		_drift_intensity = 0.0
		_drift_intensity = _calculate_drift_intensity()
		drift_started.emit(_drift_intensity)

func _exit_drift_mode() -> void:
	if _drift_mode:
		_drift_mode = false
		_drift_intensity = 0.0
		drift_ended.emit()

func _calculate_drift_intensity() -> float:
	var speed_factor = min(_current_speed_kmh / MAX_SPEED_KMH, 1.0)
	var turn_factor = abs(_steering_input)
	var lateral_factor = min(abs(_velocity_vector.x), 10.0) / 10.0
	
	var intensity = (speed_factor + turn_factor + lateral_factor) / 3.0
	return clampf(intensity, 0.0, DRIFT_INTENSITY_MAX)

func set_grip_level(level: float) -> void:
	grip_level = clampf(level, 0.1, 1.0)

# ============================================================================
# VEHICLE PHYSICS CALCULATION
# ============================================================================
func _apply_vehicle_dynamics() -> void:
	_calculate_rpm()
	_calculate_wheel_forces()
	_calculate_aerodynamic_forces()
	_calculate_acceleration()
	_update_velocity()
	_update_speed_display()

func _calculate_rpm() -> void:
	var wheel_radius: float = rear_wheel_radius
	var wheel_circumference: float = 2.0 * PI * wheel_radius
	var gear_ratio: float = _get_current_gear_ratio()
	var total_ratio: float = gear_ratio * final_drive_ratio
	
	var wheel_speed_rps: float = _current_speed_kmh / 3.6 / wheel_circumference
	var engine_rpm: float = wheel_speed_rps * total_ratio * 60.0
	
	if _throttle_input > 0.0:
		engine_rpm = lerp(engine_rpm, _throttle_input * REDLINE_RPM, 0.1)
	else:
		engine_rpm = lerp(engine_rpm, IDLE_RPM, 0.1)
	
	_current_rpm = maxf(engine_rpm, IDLE_RPM)

func _get_current_gear_ratio() -> float:
	if _current_gear <= MIN_GEAR:
		return _reverse_ratio
	return _gear_ratios[_current_gear - 1]

func _calculate_wheel_forces() -> void:
	var torque_at_wheels: float = _engine_torque_output()
	var drive_distribution: float = 0.6  # 60% rear-wheel drive by default
	
	var drive_torque: float = torque_at_wheels * drive_distribution
	var non_drive_torque: float = torque_at_wheels * (1.0 - drive_distribution)
	
	# Apply forces to wheels
	_wheel_forces.rear_left = drive_torque * 0.5
	_wheel_forces.rear_right = drive_torque * 0.5
	_wheel_forces.front_left = non_drive_torque * 0.5
	_wheel_forces.front_right = non_drive_torque * 0.5
	
	# Apply braking force
	var brake_torque: float = _brake_input * BRAKING_FORCE * vehicle_mass
	for key in _wheel_forces:
		_wheel_forces[key] -= brake_torque

func _engine_torque_output() -> float:
	var rpm_ratio: float = (_current_rpm - IDLE_RPM) / (REDLINE_RPM - IDLE_RPM)
	var torque_curve: float = _calculate_torque_curve(rpm_ratio)
	var throttle_factor: float = _throttle_input
	
	var current_torque: float = engine_max_torque * torque_curve * throttle_factor
	
	if _turbo_active:
		current_torque *= 1.3
	
	return current_torque

func _calculate_torque_curve(rpm_ratio: float) -> float:
	# Simple bell curve for torque
	var peak_rpm: float = 0.5
	var curve: float = 4.0 * rpm_ratio * (1.0 - rpm_ratio)
	return clampf(curve, 0.0, 1.0)

func _calculate_aerodynamic_forces() -> void:
	var velocity_squared: float = _current_speed_kmh * _current_speed_kmh / 3600.0
	
	# Drag force (opposes motion)
	_drag_force = 0.5 * drag_coefficient * frontal_area * velocity_squared
	
	# Downforce (presses car to ground)
	_downforce = downforce_coefficient * frontal_area * velocity_squared * 9.81

func _calculate_acceleration() -> void:
	var net_force: float = _calculate_net_force()
	var acceleration: float = net_force / vehicle_mass
	
	_acceleration_vector = acceleration * _forward_direction

func _calculate_net_force() -> float:
	var propulsion_force: float = _calculate_propulsion_force()
	var resistance_force: float = _drag_force
	
	return propulsion_force - resistance_force

func _calculate_propulsion_force() -> float:
	var gear_ratio: float = _get_current_gear_ratio()
	var wheel_torque: float = _engine_torque_output() * gear_ratio * final_drive_ratio
	var wheel_radius: float = rear_wheel_radius
	
	var force: float = wheel_torque / wheel_radius
	return force * _throttle_input

func _update_velocity() -> void:
	_velocity_vector += _acceleration_vector * _delta_time
	_velocity_vector = _velocity_vector.limit_length(MAX_SPEED_KMH / 3.6)

func _update_speed_display() -> void:
	var new_speed: float = _velocity_vector.length() * 3.6
	_current_speed_kmh = lerp(_current_speed_kmh, new_speed, 0.1)

# ============================================================================
# SUSPENSION SYSTEM
# ============================================================================
func _update_suspension_system() -> void:
	for wheel in ["front_left", "front_right", "rear_left", "rear_right"]:
		_update_single_suspension(wheel)

func _update_single_suspension(wheel: String) -> void:
	var state: Dictionary = _suspension_state[wheel]
	var compression_limit: float = SUSPENSION_COMPRESSION_LIMIT
	
	# Simplified spring-damper model
	var spring_force: float = state.compression * 100.0
	var damping_force: float = state.velocity * 50.0
	
	state.compression = lerp(state.compression, 0.0, _delta_time * 5.0)
	state.velocity = lerp(state.velocity, 0.0, _delta_time * 10.0)
	
	state.compression = clampf(state.compression, -compression_limit, compression_limit)
	
	suspension_compressed.emit(state.compression)

# ============================================================================
# COLLISION DETECTION
# ============================================================================
func _update_collision_detection() -> void:
	var collisions: Array[Dictionary] = []
	
	for i in range(get_slide_count()):
		var collision: KinematicCollision3D = get_slide(i)
		collisions.append({
			"collider": collision.get_collider(),
			"normal": collision.get_normal(),
			"position": collision.get_position(),
			"impact_speed": collision.get_travel().length()
		})
	
	if len(collisions) > 0:
		_last_collision = collisions[0]
		_last_collision_time = Time.get_unix_time_from_system()
		collision_detected.emit(_last_collision)

var _last_collision: Dictionary = {}
var _last_collision_time: float = 0.0

# ============================================================================
# POSITION AND ROTATION UPDATE
# ============================================================================
func _update_position_and_rotation() -> void:
	var rotation_speed: float = TURN_SPEED * _steering_input
	var angular_velocity: float = rotation_speed * _delta_time
	
	rotate_y(-angular_velocity)
	move_and_slide()

# ============================================================================
# SIGNAL EMITTING
# ============================================================================
func _emit_signals() -> void:
	speed_changed.emit(_current_speed_kmh)
	rpm_changed.emit(_current_rpm)
	
	if abs(_drift_intensity - _last_drift_intensity) > 0.1:
		engine_sound_changed.emit(_current_rpm / REDLINE_RPM)
	
	_last_drift_intensity = _drift_intensity

var _last_drift_intensity: float = 0.0

# ============================================================================
# LAP AND RACE TRACKING
# ============================================================================
func start_lap() -> void:
	_lap_start_time = Time.get_unix_time_from_system()
	_lap_distance = 0.0

func record_lap_checkpoint() -> void:
	_lap_distance += _delta_time * _current_speed_kmh / 3.6

func end_lap() -> Dictionary:
	var lap_time: float = Time.get_unix_time_from_system() - _lap_start_time
	var lap_data: Dictionary = {
		"time_seconds": lap_time,
		"distance_meters": _lap_distance,
		"average_speed_kmh": _lap_distance / lap_time * 3.6 if lap_time > 0 else 0.0
	}
	
	lap_completed.emit(lap_data)
	return lap_data

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================
func reset_vehicle() -> void:
	_reset_vehicle_state()
	position = Vector3.ZERO
	rotation = Vector3.ZERO
	_velocity_vector = Vector3.ZERO

func apply_force(force: Vector3) -> void:
	add_force(force)

func apply_torque(torque: Vector3) -> void:
	add_torque(torque)

func get_forward_vector() -> Vector3:
	return transform.basis.z.inverse()

func get_right_vector() -> Vector3:
	return transform.basis.x.inverse()

func get_up_vector() -> Vector3:
	return transform.basis.y

func set_vehicle_color(color: Color) -> void:
	# Apply color to vehicle meshes
	pass

func get_vehicle_health() -> float:
	return 1.0 - (_last_collision_time > 0.0 ? 0.2 : 0.0)

func take_damage(damage_amount: float) -> void:
	_current_speed_kmh *= (1.0 - damage_amount)
	_current_rpm *= (1.0 - damage_amount)
	
	race_event.emit("damage_taken", {
		"amount": damage_amount,
		"remaining_health": get_vehicle_health()
	})

func _warning(message: String) -> void:
	printerr("[VehicleController Warning] ", message)

func _max(a: float, b: float) -> float:
	return a if a > b else b

</FILE>