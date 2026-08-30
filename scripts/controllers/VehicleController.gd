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
signal powertrain_connected(powertrain: Node)
signal suspension_bumped(wheel_index: int, compression: float)

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
@export var drivetrain_type: DrivetrainType = DrivetrainType.RWD
@export var max_engine_power: float = 300.0  # Horsepower
@export var max_engine_torque: float = 450.0  # Nm
@export var gear_ratios: Array[float] = [GEAR_RATIO_FIRST, GEAR_RATIO_SECOND, GEAR_RATIO_THIRD, GEAR_RATIO_FOURTH, GEAR_RATIO_FIFTH, GEAR_RATIO_SIXTH]
@export var clutch_engagement_threshold: float = 0.1
@export var auto_shift_enabled: bool = true
@export var shift_rpm_target: float = 6500.0

@export_group("Physics Properties")
@export var friction_coefficient: float = 0.85
@export var air_drag_coefficient: float = 0.32
@export var downforce_coefficient: float = 0.05
@export var roll_resistance: float = 0.015

@export_group("Steering")
@export var max_steering_angle: float = 30.0  # degrees
@export var steering_ratio: float = 15.0
@export var steering_damping: float = 0.8

# ============================================================================
# PRIVATE STATE VARIABLES
# ============================================================================
var _current_gear: GearState = GearState.NEUTRAL
var _target_gear: GearState = GearState.NEUTRAL
var _current_rpm: float = MIN_RPM
var _engine_on: bool = false
var _clutch_engaged: bool = false
var _throttle_input: float = 0.0
var _brake_input: float = 0.0
var _steering_input: float = 0.0
var _handbrake_active: bool = false
var _abs_active: bool = false
var _traction_control_active: bool = false

var _speed_kmh: float = 0.0
var _velocity_vector: Vector3 = Vector3.ZERO
var _angular_velocity: Vector3 = Vector3.ZERO
var _wheel_angles: Array[float] = [0.0, 0.0, 0.0, 0.0]
var _wheel_forces: Array[float] = [0.0, 0.0, 0.0, 0.0]
var _wheel_suspension_compression: Array[float] = [0.0, 0.0, 0.0, 0.0]

var _powertrain_node: Node = null
var _acceleration: float = 0.0
var _deceleration: float = 0.0
var _gearbox_efficiency: float = 0.95
var _differential_type: String = "open"

var _drift_angle: float = 0.0
var _is_drifting: bool = false
var _drift_threshold: float = 15.0
var _grip_level: float = 1.0

var _last_update_time: float = 0.0
var _physics_delta: float = 0.0

# ============================================================================
# GETTERS AND SETTERS
# ============================================================================
func get_current_gear() -> GearState:
	return _current_gear

func get_current_rpm() -> float:
	return _current_rpm

func get_speed_kmh() -> float:
	return _speed_kmh

func get_speed_ms() -> float:
	return _speed_kmh / 3.6

func get_velocity() -> Vector3:
	return _velocity_vector

func is_engine_running() -> bool:
	return _engine_on

func is_clutch_engaged() -> bool:
	return _clutch_engaged

func get_throttle_input() -> float:
	return _throttle_input

func get_brake_input() -> float:
	return _brake_input

func get_steering_angle() -> float:
	return _wheel_angles[0]  # Front left wheel angle (primary steering)

func get_wheel_angles() -> Array[float]:
	return _wheel_angles

func get_acceleration() -> float:
	return _acceleration

func get_slip_angle() -> float:
	return _drift_angle

func is_drifting() -> bool:
	return _is_drifting

# ============================================================================
# LIFECYCLE METHODS
# ============================================================================
func _ready() -> void:
	_process_mode = ProcessModeEnum.ALWAYS
	_init_physics_properties()
	_connect_signals()
	_reset_state()

func _process(delta: float) -> void:
	_physics_delta = delta
	_last_update_time = Time.get_ticks_usec() / 1000000.0
	
	if not _engine_on:
		return
	
	_update_physics(delta)
	_update_inputs()
	_update_gearing(delta)
	_update_steering(delta)
	_update_driving_dynamics()
	_apply_forces()

func _input(event: InputEvent) -> void:
	if event.is_action_pressed("gear_up"):
		_shift_gear(1)
	elif event.is_action_pressed("gear_down"):
		_shift_gear(-1)
	elif event.is_action_pressed("toggle_handbrake"):
		_toggle_handbrake()

# ============================================================================
# INITIALIZATION
# ============================================================================
func _init_physics_properties() -> void:
	# Initialize wheel arrays
	for i in range(4):
		_wheel_angles.append(0.0)
		_wheel_forces.append(0.0)
		_wheel_suspension_compression.append(0.0)

func _connect_signals() -> void:
	# Connect to GameManager signals if available
	var gm = GameManager if GameManager else null
	if gm:
		gm.game_state_changed.connect(_on_game_state_changed)

func _reset_state() -> void:
	_current_gear = GearState.NEUTRAL
	_target_gear = GearState.NEUTRAL
	_current_rpm = MIN_RPM
	_engine_on = false
	_clutch_engaged = false
	_throttle_input = 0.0
	_brake_input = 0.0
	_steering_input = 0.0
	_handbrake_active = false
	_abs_active = false
	_traction_control_active = false
	_speed_kmh = 0.0
	_velocity_vector = Vector3.ZERO
	_wheel_angles.fill(0.0)
	_wheel_forces.fill(0.0)
	_wheel_suspension_compression.fill(0.0)
	_is_drifting = false
	_drift_angle = 0.0

# ============================================================================
# INPUT HANDLING
# ============================================================================
func _update_inputs() -> void:
	# Get input values from InputManager
	var input_manager = InputManager if InputManager else null
	if input_manager:
		_throttle_input = input_manager.get_axis("throttle", 0.0)
		_brake_input = input_manager.get_axis("brake", 0.0)
		_steering_input = input_manager.get_axis("steer_left", "steer_right")

# ============================================================================
# PHYSICS UPDATE
# ============================================================================
func _update_physics(delta: float) -> void:
	# Calculate current speed from velocity
	_velocity_vector = linear_interpolate(_velocity_vector, _velocity_vector, 0.9)
	_speed_kmh = _velocity_vector.length() * 3.6

	# Update RPM based on gear and speed
	_update_rpm_from_speed()

	# Calculate acceleration/deceleration
	_calculate_acceleration()

	# Apply movement
	_apply_movement(delta)

	# Check for collisions
	_check_collisions()

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================
func _update_gearing(delta: float) -> void:
	# Auto-shift logic
	if auto_shift_enabled and _current_gear != GearState.NEUTRAL:
		_auto_shift()

	# Manual shift overrides
	if _target_gear != _current_gear and _clutch_engaged:
		_perform_shift()

# func _auto_shift() -> void:
# 	# Shift up if above target RPM
# 	if _current_rpm > shift_rpm_target and _current_gear < GearState.TENTH:
# 		_shift_gear(1)
# 	# Downshift if below minimum RPM and moving
# 	elif _current_rpm < MIN_RPM * 1.2 and _speed_kmh > 5.0 and _current_gear > GearState.FIRST:
# 		_shift_gear(-1)

func _perform_shift() -> void:
	var old_gear = _current_gear
	_current_gear = _target_gear
	_clutch_engaged = false
	
	gear_changed.emit(old_gear, _current_gear)
	
	# Re-engage clutch after delay
	await get_tree().create_timer(0.3).timeout
	_clutch_engaged = true

func _shift_gear(direction: int) -> void:
	if direction == 0:
		return
	
	var previous_gear = _current_gear
	_target_gear = clampi(_current_gear + direction, GearState.REVERSE, GearState.TENTH)
	
	# Engage clutch during shift
	_clutch_engaged = false
	await get_tree().create_timer(0.15).timeout
	_clutch_engaged = true

# ============================================================================
# ENGINE & RPM MANAGEMENT
# ============================================================================
func _update_rpm_from_speed() -> void:
	if _current_gear == GearState.NEUTRAL or _current_gear == GearState.NEUTRAL:
		# Engine idles when in neutral
		_current_rpm = lerp(_current_rpm, IDLE_RPM, 0.1)
		return
	
	if _current_gear == GearState.REVERSE:
		# Reverse ratio (same as first but reversed direction)
		var effective_ratio = gear_ratios[0] * 1.2
		_current_rpm = calculate_rpm_from_speed(_speed_kmh / 3.6, effective_ratio)
	else:
		var gear_index = _current_gear - 1
		if gear_index >= 0 and gear_index < gear_ratios.size():
			var ratio = gear_ratios[gear_index]
			_current_rpm = calculate_rpm_from_speed(_speed_kmh / 3.6, ratio)
		else:
			_current_rpm = MIN_RPM
	
	# Clamp RPM
	_current_rpm = clampf(_current_rpm, MIN_RPM, MAX_RPM)
	rpm_changed.emit(_current_rpm)

func calculate_rpm_from_speed(speed_ms: float, gear_ratio: float) -> float:
	# Formula: RPM = (Speed / Wheel Circumference) * Gear Ratio * Final Drive * 60 / 2π
	var wheel_circumference = 2.0 * PI * WHEEL_RADIUS
	var wheel_rps = speed_ms / wheel_circumference
	var drive_shaft_rps = wheel_rps * gear_ratio * FINAL_DRIVE
	var rpm = drive_shaft_rps * 60.0
	return rpm

func _calculate_acceleration() -> void:
	if not _clutch_engaged or _current_gear == GearState.NEUTRAL:
		_acceleration = 0.0
		return
	
	# Calculate torque at wheels based on engine output
	var torque_multiplier = _get_torque_curve_factor()
	var wheel_torque = max_engine_torque * torque_multiplier
	
	# Apply gear ratio and final drive
	var gear_ratio = gear_ratios[_current_gear - 1] if _current_gear > GearState.FIRST else GEAR_RATIO_FIRST
	wheel_torque *= gear_ratio * FINAL_DRIVE * _gearbox_efficiency
	
	# Convert torque to force (Force = Torque / Radius)
	var drive_force = wheel_torque / WHEEL_RADIUS
	
	# Apply drivetrain distribution
	drive_force *= _get_drive_distribution_factor()
	
	# Apply losses
	drive_force *= 1.0 - _drivetrain_loss()
	
	# Calculate acceleration (F = ma)
	_acceleration = drive_force / vehicle_mass

func _get_torque_curve_factor() -> float:
	# Simple torque curve approximation
	var rpm_ratio = (_current_rpm - MIN_RPM) / (MAX_RPM - MIN_RPM)
	# Peak torque around 4000 RPM
	var peak_ratio = 0.5
	var factor = 1.0 - abs(rpm_ratio - peak_ratio) * 1.5
	return clampf(factor, 0.3, 1.0)

func _get_drive_distribution_factor() -> float:
	match drivetrain_type:
		DrivetrainType.FWD:
			return 0.6  # 60% front, 40% rear
		DrivetrainType.RWD:
			return 0.5  # 50/50 split
		DrivetrainType.AWD:
			return 0.45  # 45/55 front/rear
		_:
			return 0.5

func _drivetrain_loss() -> float:
	return 0.05  # 5% loss through drivetrain

# ============================================================================
# STEERING CONTROL
# ============================================================================
func _update_steering(delta: float) -> void:
	# Smooth steering transition
	var target_steering = _steering_input * max_steering_angle
	var current_steering = _wheel_angles[0]
	_wheel_angles[0] = lerp(current_steering, target_steering, STEERING_SPEED * delta)
	
	# Adjust other wheels based on Ackermann geometry
	_wheel_angles[1] = _wheel_angles[0] * 0.9  # Front right
	_wheel_angles[2] = 0.0  # Rear wheels don't steer
	_wheel_angles[3] = 0.0

# ============================================================================
# DRIVING DYNAMICS
# ============================================================================
func _update_driving_dynamics() -> void:
	# Calculate grip level based on conditions
	_grip_level = _calculate_grip_level()
	
	# Check for drifting conditions
	_check_drift_conditions()
	
	# Apply traction control if active
	if _traction_control_active:
		_apply_traction_control()
	
	# Apply ABS if braking hard
	if _brake_input > 0.8 and _abs_active:
		_apply_abs()

func _calculate_grip_level() -> float:
	# Base grip reduced by speed, surface type, etc.
	var base_grip = friction_coefficient * _grip_level
	
	# Reduce grip at high speeds
	var speed_factor = 1.0 - minf(_speed_kmh / 300.0, 0.3)
	
	# Weather/surface effects could be added here
	return base_grip * speed_factor

func _check_drift_conditions() -> void:
	var slip_angle = _calculate_slip_angle()
	
	if abs(slip_angle) > _drift_threshold and _speed_kmh > 30.0:
		if not _is_drifting:
			_is_drifting = true
			_drift_angle = slip_angle
			drift_started.emit(slip_angle)
	else:
		if _is_drifting:
			_is_drifting = false
			drift_ended.emit()

func _calculate_slip_angle() -> float:
	# Simplified slip angle calculation
	var forward_dir = global_transform.basis.z
	var velocity_dir = _velocity_vector.normalized()
	if _velocity_vector.length() < 0.1:
		return 0.0
	
	var angle = forward_dir.angle_to(velocity_dir)
	return deg_to_rad(angle)

func _apply_traction_control() -> void:
	# Reduce wheel spin on acceleration
	if _throttle_input > 0.5 and _speed_kmh < 10.0:
		_wheel_forces[0] *= TRACTION_CONTROL_STRENGTH
		_wheel_forces[1] *= TRACTION_CONTROL_STRENGTH

func _apply_abs() -> void:
	# Modulate brake force to prevent lockup
	if _speed_kmh < 5.0:
		_brake_input *= ABS_STRENGTH

# ============================================================================
# FORCE APPLICATION
# ============================================================================
func _apply_forces() -> void:
	# Apply acceleration force
	if _acceleration > 0 and _clutch_engaged:
		var accel_force = _acceleration * vehicle_mass
		force_applied(Vector3.FORWARD * accel_force)
	
	# Apply braking force
	if _brake_input > 0:
		var brake_force = _brake_input * vehicle_mass * 5.0
		force_applied(Vector3.BACKWARD * brake_force)
	
	# Apply drag force
	var drag_force = _speed_kmh * _speed_kmh * air_drag_coefficient * 0.1
	force_applied(-_velocity_vector.normalized() * drag_force)

func _apply_movement(delta: float) -> void:
	if not _engine_on:
		return
	
	# Add acceleration to velocity
	_velocity_vector.x += _acceleration * delta * 3.6  # m/s to km/h
	
	# Apply rotation based on steering
	if abs(_velocity_vector.length()) > 0.5:
		var turn_rate = _wheel_angles[0] * delta * 0.5
		global_rotation.y -= turn_rate

# ============================================================================
# COLLISION DETECTION
# ============================================================================
func _check_collisions() -> void:
	var collider = get_slide_collision(0)
	if collider:
		var collision_data = {
			"collider": collider.get_path(),
			"position": global_position,
			"velocity": _velocity_vector,
			"impulse": get_slide_collision(0).get_collider_impulse()
		}
		collision_detected.emit(collision_data)

# ============================================================================
# ENGINE CONTROL
# ============================================================================
func start_engine() -> void:
	if _engine_on:
		return
	
	_engine_on = true
	_current_rpm = IDLE_RPM
	engine_started.emit()

func stop_engine() -> void:
	if not _engine_on:
		return
	
	_engine_on = false
	_current_rpm = MIN_RPM
	engine_stopped.emit()

# ============================================================================
# HAND BRAKE CONTROL
# ============================================================================
func _toggle_handbrake() -> void:
	_handbrake_active = !_handbrake_active
	handbrake_toggled.emit(_handbrake_active)

func set_handbrake(active: bool) -> void:
	_handbrake_active = active
	handbrake_toggled.emit(active)

# ============================================================================
# POWERTRAIN INTEGRATION
# ============================================================================
func connect_powertrain(powertrain: Node) -> void:
	_powertrain_node = powertrain
	powertrain.connected.emit(self)
	powertrain_connected.emit(powertrain)

func disconnect_powertrain() -> void:
	_powertrain_node = null

# ============================================================================
# DEBUG & TESTING HELPERS
# ============================================================================
func debug_set_input(throttle: float, brake: float, steering: float) -> void:
	_throttle_input = throttle
	_brake_input = brake
	_steering_input = steering

func debug_set_gear(gear: GearState) -> void:
	_current_gear = gear
	_target_gear = gear

func debug_set_rpm(rpm: float) -> void:
	_current_rpm = clampf(rpm, MIN_RPM, MAX_RPM)

func debug_print_status() -> void:
	print("=== Vehicle Controller Status ===")
	print("Gear: %s" % _current_gear)
	print("RPM: %.0f" % _current_rpm)
	print("Speed: %.1f km/h" % _speed_kmh)
	print("Throttle: %.2f" % _throttle_input)
	print("Brake: %.2f" % _brake_input)
	print("Steering: %.2f deg" % _wheel_angles[0])
	print("Engine: %s" % ("ON" if _engine_on else "OFF"))
	print("Clutch: %s" % ("ENGAGED" if _clutch_engaged else "DISengaged"))
	print("================================")

# ============================================================================
# EXTERNAL EVENT HANDLERS
# ============================================================================
func _on_game_state_changed(new_state: GameManager.GameState) -> void:
	match new_state:
		GameManager.GameState.MAIN_MENU:
			stop_engine()
			_reset_state()
		GameManager.GameState.RACE_ACTIVE:
			if not _engine_on:
				start_engine()
		_ :
			pass

# ============================================================================
# SERIALIZE/DATA PERSISTENCE
# ============================================================================
func serialize_state() -> Dictionary:
	return {
		"gear": _current_gear,
		"rpm": _current_rpm,
		"speed_kmh": _speed_kmh,
		"throttle": _throttle_input,
		"brake": _brake_input,
		"steering": _steering_input,
		"engine_on": _engine_on,
		"clutch_engaged": _clutch_engaged,
		"handbrake": _handbrake_active
	}

func deserialize_state(data: Dictionary) -> void:
	_current_gear = data.get("gear", GearState.NEUTRAL)
	_current_rpm = data.get("rpm", MIN_RPM)
	_speed_kmh = data.get("speed_kmh", 0.0)
	_throttle_input = data.get("throttle", 0.0)
	_brake_input = data.get("brake", 0.0)
	_steering_input = data.get("steering", 0.0)
	_engine_on = data.get("engine_on", false)
	_clutch_engaged = data.get("clutch_engaged", false)
	_handbrake_active = data.get("handbrake", false)

# ============================================================================
# PROPERTY SETTERS
# ============================================================================
func _set_vehicle_mass(value: float) -> void:
	vehicle_mass = value
	_reset_state()

# ============================================================================
# HELPER FUNCTIONS
# ============================================================================
func get_distance_traveled() -> float:
	# Track total distance traveled (would need additional state tracking)
	return 0.0

func get_lap_time() -> float:
	# Would need lap timing system integration
	return 0.0

func reset_odometer() -> void:
	pass  # Implementation depends on game requirements

func set_grip_modifier(modifier: float) -> void:
	_grip_level = modifier
</file>