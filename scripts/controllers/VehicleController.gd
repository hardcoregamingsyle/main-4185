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

# ============================================================================
# ENUMS & TYPES
# ============================================================================
enum DrivetrainType { FWD, RWD, AWD }
enum GearState { NEUTRAL = 0, FIRST = 1, SECOND = 2, THIRD = 3, FOURTH = 4, FIFTH = 5, SIXTH = 6, REVERSE = -1 }

# ============================================================================
# CONSTANTS & CONFIGURATION - References to PhysicsSettings
# ============================================================================
const VEHICLE_BASE_MASS := 1500.0  # Base mass in kg
const GROUND_FRICTION := 0.98
const AIR_RESISTANCE_FACTOR := 0.01
const MAX_STEERING_SPEED := 2.5  # radians per second

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
@export var final_drive_ratio: float = 3.5: set = _set_final_drive_ratio
@export var tire_radius: float = 0.33: set = _set_tire_radius
@export var torque_curve: Array[Vector2f] = [
	Vector2f(0.0, 0.0),   # RPM fraction -> Torque multiplier
	Vector2f(0.2, 0.6),
	Vector2f(0.4, 0.9),
	Vector2f(0.6, 1.0),   # Peak torque at 60% RPM
	Vector2f(0.8, 0.95),
	Vector2f(1.0, 0.85),  # Redline torque drop
]: set = _set_torque_curve

@export_group("Gear Ratios")
@export var gear_ratios: Array[float] = [
	-4.0,   # Reverse
	4.0,    # First gear
	2.5,    # Second gear
	1.8,    # Third gear
	1.4,    # Fourth gear
	1.1,    # Fifth gear
	0.9     # Sixth gear
]

@export_group("Engine Parameters")
@export var idle_rpm: float = 800.0
@export var redline_rpm: float = 7000.0
@export var max_rpm: float = 8500.0
@export var engine_torque_peak: float = 400.0  # Nm at peak

@export_group("Handling Characteristics")
@export var steering_sensitivity: float = 1.0
@export var suspension_stiffness: float = 50000.0
@export var damping_coefficient: float = 15000.0

# ============================================================================
# PRIVATE STATE VARIABLES
# ============================================================================
var _current_speed_kmh: float = 0.0
var _current_rpm: float = 0.0
var _current_gear: GearState = GearState.NEUTRAL
var _target_gear: GearState = GearState.NEUTRAL
var _is_engine_running: bool = false
var _handbrake_active: bool = false
var _is_drifting: bool = false
var _drift_angle: float = 0.0

# Input state
var _throttle_input: float = 0.0
var _brake_input: float = 0.0
var _steering_input: float = 0.0
var _handbrake_input: bool = false
var _upshift_input: bool = false
var _downshift_input: bool = false

# Wheel configuration
var _front_wheel_count: int = 2
var _rear_wheel_count: int = 2

# Physics state
var _wheel_forces: Array[Vector3] = []
var _wheel_contact_points: Array[Vector3] = []
var _applied_force: Vector3 = Vector3.ZERO
var _applied_torque: Vector3 = Vector3.ZERO

# Reference to powertrain component (if exists)
var _powertrain_node: Node = null

# Timing variables
var _time_since_last_shift: float = 0.0
var _last_position: Vector3 = Vector3.ZERO
var _velocity_history: Array[Vector3] = []
var _angular_velocity: Vector3 = Vector3.ZERO

# Drift mechanics
var _drift_threshold_angle: float = PI / 6  # 30 degrees
var _drift_recovery_rate: float = 0.1
var _drift_momentum: float = 0.0

# ============================================================================
# INITIALIZATION
# ============================================================================
func _ready() -> void:
	_process_mode = ProcessModeEnum.ALWAYS
	
	# Initialize physics settings reference
	if not Engine.has_singleton("PhysicsSettings"):
		push_warning("VehicleController: PhysicsSettings singleton not found!")
	
	# Setup initial state
	_current_gear = GearState.NEUTRAL
	_target_gear = GearState.NEUTRAL
	_is_engine_running = false
	
	# Initialize wheel data structures
	_wheel_forces.resize(4)
	_wheel_contact_points.resize(4)
	
	# Set initial position
	_last_position = global_transform.origin
	
	# Connect to audio manager if available
	if GameManager and GameManager.audio_manager:
		GameManager.audio_manager.sound_played.connect(_on_sound_played)

func _process(delta: float) -> void:
	_handle_input(delta)
	_update_gear_shifting(delta)
	_update_engine_state(delta)
	_update_drift_state(delta)

func _physics_process(delta: float) -> void:
	apply_physics(delta)
	update_collision_detection(delta)
	emit_signals()

# ============================================================================
# INPUT HANDLING
# ============================================================================
func _handle_input(delta: float) -> void:
	if not GameManager or not GameManager.input_manager:
		return
	
	# Get normalized inputs (-1 to 1 range)
	_throttle_input = GameManager.input_manager.get_axis("throttle", "brake")
	_brake_input = GameManager.input_manager.get_axis("brake", "reverse")
	_steering_input = GameManager.input_manager.get_axis("steer_left", "steer_right")
	_handbrake_input = GameManager.input_manager.is_action_pressed("handbrake")
	_upshift_input = GameManager.input_manager.is_action_just_pressed("upshift")
	_downshift_input = GameManager.input_manager.is_action_just_pressed("downshift")
	
	# Clamp inputs
	_throttle_input = clamp(_throttle_input, -1.0, 1.0)
	_brake_input = clamp(_brake_input, -1.0, 1.0)
	_steering_input = clamp(_steering_input, -1.0, 1.0)
	
	# Apply handbrake override
	if _handbrake_input:
		_handbrake_active = true
	else:
		_handbrake_active = _handbrake_active and (_throttle_input > 0.8 or _current_speed_kmh < 5.0)

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================
func _update_gear_shifting(delta: float) -> void:
	_time_since_last_shift += delta
	
	# Prevent rapid gear changes (minimum 0.1s between shifts)
	if _time_since_last_shift < 0.1:
		return
	
	# Manual gear shift requests
	if _upshift_input and _current_gear != GearState.NEUTRAL and _current_gear > 0:
		_target_gear = min(GearState.SIXTH, _current_gear + 1)
		_attempt_gear_change()
	elif _downshift_input and _current_gear != GearState.NEUTRAL:
		_target_gear = max(GearState.FIRST, _current_gear - 1)
		_attempt_gear_change()
	
	# Automatic gear shifting based on RPM
	if _is_engine_running and _current_rpm > 0:
		_auto_shift_gear()

func _auto_shift_gear() -> void:
	var current_rpm_fraction: float = _get_rpm_fraction()
	
	# Upshift when approaching redline
	if _current_gear < GearState.SIXTH and current_rpm_fraction > 0.85:
		_target_gear = _current_gear + 1
		_attempt_gear_change()
	
	# Downshift when RPM drops too low
	elif _current_gear > GearState.FIRST and current_rpm_fraction < 0.3:
		_target_gear = max(GearState.FIRST, _current_gear - 1)
		_attempt_gear_change()

func _attempt_gear_change() -> void:
	if _current_gear == _target_gear:
		return
	
	var old_gear: int = _current_gear as int
	var new_gear: int = _target_gear as int
	
	# Validate gear change
	if not _can_change_gear(old_gear, new_gear):
		return
	
	# Execute gear change
	_current_gear = _target_gear
	_time_since_last_shift = 0.0
	
	# Emit signal
	gear_changed.emit(old_gear, new_gear)
	
	# Play shift sound if available
	if AudioManager:
		AudioManager.play_sound("gear_shift")

func _can_change_gear(from_gear: int, to_gear: int) -> bool:
	# Prevent reverse while moving forward
	if (from_gear >= 0 and to_gear < 0) or (from_gear < 0 and to_gear >= 0):
		if _current_speed_kmh > 5.0:
			return false
	
	# Neutral transitions are always allowed
	if from_gear == 0 or to_gear == 0:
		return true
	
	return true

func _get_rpm_fraction() -> float:
	if _current_rpm <= 0:
		return 0.0
	return _current_rpm / max_rpm

# ============================================================================
# ENGINE MANAGEMENT
# ============================================================================
func _update_engine_state(delta: float) -> void:
	# Update RPM based on gear ratio and vehicle speed
	_update_rpm_from_speed()
	
	# Rev engine based on throttle input
	_apply_throttle_effect(delta)
	
	# Engine braking effect when releasing throttle
	if _throttle_input < 0.1 and _current_speed_kmh > 10.0:
		_current_rpm = lerp(_current_rpm, idle_rpm, delta * 2.0)

func _update_rpm_from_speed() -> void:
	if _current_gear == GearState.NEUTRAL or _current_gear < GearState.FIRST:
		_current_rpm = idle_rpm
		return
	
	var gear_idx: int = _current_gear as int
	var gear_ratio: float = gear_ratios[gear_idx]
	
	# Calculate theoretical RPM based on wheel rotation
	var wheel_rotation_speed: float = _current_speed_kmh / 3.6  # m/s
	var wheel_rps: float = wheel_rotation_speed / (2.0 * PI * tire_radius)
	var engine_rpm: float = wheel_rps * gear_ratio * final_drive_ratio * 60.0
	
	# Smooth transition to calculated RPM
	_current_rpm = lerp(_current_rpm, engine_rpm, delta * 5.0)
	
	# Clamp to valid range
	_current_rpm = clamp(_current_rpm, idle_rpm, max_rpm)

func _apply_throttle_effect(delta: float) -> void:
	var target_rpm: float = idle_rpm
	
	if _throttle_input > 0.1:
		# Calculate target RPM based on throttle
		var rpm_range: float = max_rpm - idle_rpm
		target_rpm = idle_rpm + (_throttle_input * rpm_range * 0.95)
		
		# Accelerate towards target RPM
		_current_rpm = lerp(_current_rpm, target_rpm, delta * 10.0)
	
	elif _throttle_input < -0.1:
		# Reverse/throttle down
		target_rpm = idle_rpm * 0.5
		_current_rpm = lerp(_current_rpm, target_rpm, delta * 8.0)
	
	# Ensure we don't go below idle when running
	if _is_engine_running and _current_rpm < idle_rpm:
		_current_rpm = idle_rpm

func start_engine() -> void:
	_is_engine_running = true
	_current_rpm = idle_rpm
	engine_started.emit()
	if AudioManager:
		AudioManager.play_sound("engine_start")

func stop_engine() -> void:
	_is_engine_running = false
	_current_rpm = 0.0
	engine_stopped.emit()
	if AudioManager:
		AudioManager.play_sound("engine_stop")

# ============================================================================
# PHYSICS CALCULATIONS
# ============================================================================
func apply_physics(delta: float) -> void:
	if not _is_engine_running and _current_speed_kmh == 0.0:
		return
	
	# Calculate applied force based on gear and throttle
	var applied_force: float = _calculate_applied_force()
	
	# Apply drag and friction
	_apply_drag_and_friction(delta)
	
	# Apply steering influence
	_apply_steering_influence(delta)
	
	# Handle handbrake sliding
	if _handbrake_active:
		_apply_handbrake_effect(delta)
	
	# Update velocity
	apply_impulse(applied_force * delta)
	
	# Store last position for movement detection
	_last_position = global_transform.origin

func _calculate_applied_force() -> float:
	if _current_gear == GearState.NEUTRAL:
		return 0.0
	
	var gear_idx: int = _current_gear as int
	if gear_idx < 0 or gear_idx >= gear_ratios.size():
		return 0.0
	
	var gear_ratio: float = gear_ratios[gear_idx]
	var rpm_fraction: float = _get_rpm_fraction()
	var torque_multiplier: float = _interpolate_torque(rpm_fraction)
	
	# Calculate engine torque
	var engine_torque: float = engine_torque_peak * torque_multiplier
	
	# Convert torque to force at wheels
	var wheel_torque: float = engine_torque * gear_ratio * final_drive_ratio
	var drive_force: float = wheel_torque / tire_radius
	
	# Apply drivetrain loss factor
	var drivetrain_efficiency: float = 0.85
	drive_force *= drivetrain_efficiency
	
	# Scale by throttle input
	var throttle_factor: float = _throttle_input
	if _throttle_input < 0:
		throttle_factor = abs(_throttle_input) * 0.5  # Less power in reverse
	
	return drive_force * throttle_factor

func _interpolate_torque(rpm_fraction: float) -> float:
	if rpm_fraction <= 0:
		return 0.0
	elif rpm_fraction >= 1.0:
		return torque_curve.back().y
	
	# Linear interpolation between curve points
	for i in range(torque_curve.size() - 1):
		var p1: Vector2f = torque_curve[i]
		var p2: Vector2f = torque_curve[i + 1]
		
		if p1.x <= rpm_fraction and rpm_fraction <= p2.x:
			var t: float = (rpm_fraction - p1.x) / (p2.x - p1.x)
			return p1.y + t * (p2.y - p1.y)
	
	return torque_curve[-1].y

func _apply_drag_and_friction(delta: float) -> void:
	# Air resistance (proportional to v^2)
	var air_resistance: float = _current_speed_kmh * _current_speed_kmh * AIR_RESISTANCE_FACTOR
	var drag_vector: Vector3 = -velocity.normalized() * air_resistance
	
	# Ground friction
	var ground_friction: float = VEHICLE_BASE_MASS * GROUND_FRICTION * delta
	
	# Apply total resistive force
	apply_impulse(drag_vector * delta - ground_friction * velocity.normalized())

func _apply_steering_influence(delta: float) -> void:
	if _current_speed_kmh < 2.0:
		return
	
	var steering_angle: float = _steering_input * deg_to_rad(steering_angle_max) * steering_sensitivity
	steering_angle = lerp(steering_angle, 0.0, delta * 10.0)
	
	# Apply steering torque
	var steer_torque: float = steering_angle * acceleration_force * 0.1
	_angular_velocity = Vector3.UP * steer_torque

func _apply_handbrake_effect(delta: float) -> void:
	# Reduce traction and increase slide tendency
	var handbrake_force: float = braking_force * 1.5
	
	# Apply lateral friction reduction
	var lateral_friction: float = 0.3
	var longitudinal_friction: float = 0.5
	
	# Create sliding vector
	var slide_direction: Vector3 = velocity.normalized()
	var lateral_slide: Vector3 = Vector3(-slide_direction.z, 0, slide_direction.x).normalized()
	
	# Apply reduced friction forces
	apply_impulse(-lateral_slide * lateral_friction * VEHICLE_BASE_MASS * delta)
	apply_impulse(-slide_direction * longitudinal_friction * VEHICLE_BASE_MASS * delta)

# ============================================================================
# DRIFT MECHANICS
# ============================================================================
func _update_drift_state(delta: float) -> void:
	if _current_speed_kmh < 30.0:
		_exit_drift()
		return
	
	# Calculate slip angle
	var direction: Vector3 = velocity.normalized()
	var heading: Vector3 = global_transform.basis.z.normalized()
	var angle: float = direction.angle_to(heading)
	
	if abs(angle) > _drift_threshold_angle:
		_enter_drift(angle)
	else:
		_exit_drift()

func _enter_drift(angle: float) -> void:
	if not _is_drifting:
		_is_drifting = true
		_drift_angle = angle
		_drift_momentum = 1.0
	-drift_started.emit(angle)
	
	if AudioManager:
		AudioManager.play_sound("drift_start")

func _exit_drift() -> void:
	if _is_drifting:
		_is_drifting = false
		_drift_momentum = 0.0
		_drift_angle = 0.0
		drift_ended.emit()
		
		if AudioManager:
			AudioManager.play_sound("drift_end")

func _get_drift_momentum() -> float:
	return _drift_momentum

# ============================================================================
# COLLISION DETECTION
# ============================================================================
func update_collision_detection(delta: float) -> void:
	# Check for collisions using shape cast or overlap tests
	var colliders: Array = get_colliding_bodies()
	
	for collider in colliders:
		var collision_info: Dictionary = {
			"collider": collider,
			"position": global_transform.origin,
			"velocity": velocity,
			"speed_kmh": _current_speed_kmh,
			"time": Time.get_ticks_msec() / 1000.0
		}
		
		collision_detected.emit(collision_info)

func on_body_entered(body: Node) -> void:
	var collision_data: Dictionary = {
		"collider": body.name,
		"collision_normal": body.global_transform.origin - global_transform.origin,
		"relative_velocity": velocity - body.velocity,
		"impact_force": velocity.length() * vehicle_mass
	}
	
	collision_detected.emit(collision_data)

func on_body_exited(body: Node) -> void:
	pass  # Cleanup if needed

# ============================================================================
# SIGNAL EMISSION
# ============================================================================
func emit_signals() -> void:
	# Only emit signals if values changed significantly
	var speed_changed_significant: bool = abs(_current_speed_kmh - get_speed_kmh()) > 1.0
	
	if speed_changed_significant:
		speed_changed.emit(get_speed_kmh())
	
	if abs(_current_rpm - get_current_rpm()) > 50.0:
		rpm_changed.emit(get_current_rpm())

func get_speed_kmh() -> float:
	return velocity.length() * 3.6

func get_current_rpm() -> float:
	return _current_rpm

# ============================================================================
# CONTROL METHODS - External API
# ============================================================================
func set_throttle(value: float) -> void:
	_throttle_input = clamp(value, -1.0, 1.0)

func set_brake(value: float) -> void:
	_brake_input = clamp(value, -0.0, 1.0)

func set_steering(value: float) -> void:
	_steering_input = clamp(value, -1.0, 1.0)

func toggle_handbrake(active: bool) -> void:
	_handbrake_input = active

func force_gear(gear: GearState) -> void:
	_target_gear = gear
	_attempt_gear_change()

func reset_vehicle() -> void:
	_current_gear = GearState.NEUTRAL
	_target_gear = GearState.NEUTRAL
	_current_speed_kmh = 0.0
	_current_rpm = idle_rpm
	_handbrake_active = false
	_is_drifting = false
	
	velocity = Vector3.ZERO
	angular_velocity = Vector3.ZERO

# ============================================================================
# PROPERTY SETTERS
# ============================================================================
func _set_vehicle_mass(new_value: float) -> void:
	vehicle_mass = new_value
	mass = vehicle_mass

func _set_max_speed_kmh(new_value: float) -> void:
	max_speed_kmh = new_value

func _set_acceleration_force(new_value: float) -> void:
	acceleration_force = new_value

func _set_braking_force(new_value: float) -> void:
	braking_force = new_value

func _set_steering_angle_max(new_value: float) -> void:
	steering_angle_max = new_value

func _set_final_drive_ratio(new_value: float) -> void:
	final_drive_ratio = new_value

func _set_tire_radius(new_value: float) -> void:
	tire_radius = new_value

func _set_torque_curve(new_value: Array[Vector2f]) -> void:
	torque_curve = new_value

# ============================================================================
# DEBUG & TESTING
# ============================================================================
func debug_get_status() -> Dictionary:
	return {
		"is_engine_running": _is_engine_running,
		"current_gear": _current_gear as int,
		"current_rpm": _current_rpm,
		"speed_kmh": _current_speed_kmh,
		"throttle_input": _throttle_input,
		"brake_input": _brake_input,
		"steering_input": _steering_input,
		"handbrake_active": _handbrake_active,
		"is_drifting": _is_drifting,
		"drift_angle": _drift_angle,
		"applied_force": _applied_force,
		"velocity": velocity,
		"position": global_transform.origin
	}

# ============================================================================
# AUDIO INTEGRATION
# ============================================================================
func _on_sound_played(sound_name: String) -> void:
	match sound_name:
		"gear_shift":
			# Already handled in _attempt_gear_change
			pass
		"engine_idle":
			if _is_engine_running and _current_rpm < idle_rpm * 1.5:
				AudioManager.play_sound("engine_idle_loop")
		"engine_rev":
			if _current_rpm > idle_rpm * 3:
				AudioManager.play_sound("engine_rev_loop")

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================
func calculate_distance_traveled() -> float:
	var displacement: Vector3 = global_transform.origin - _last_position
	return displacement.length()

func calculate_acceleration() -> float:
	return velocity.length() / Time.get_unix_time_from_system()

func is_moving() -> bool:
	return velocity.length() > 0.5

func get_forward_vector() -> Vector3:
	return global_transform.basis.z.normalized()

func get_right_vector() -> Vector3:
	return global_transform.basis.x.normalized()

</script>