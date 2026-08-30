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
@export var max_speed: float = MAX_SPEED_KMH * 1000.0 / 3600.0: set = _set_max_speed
@export var acceleration_power: float = ACCELERATION_RATE: set = _set_acceleration_power
@export var braking_force: float = BRAKING_FORCE: set = _set_braking_force
@export var turn_sensitivity: float = TURN_SPEED: set = _set_turn_sensitivity
@export var vehicle_mass: float = PhysicsSettings.default_vehicle_mass: set = _set_vehicle_mass

@export_group("Engine Settings")
@export var engine_torque: float = 450.0
@export var idle_rpm: float = IDLE_RPM: set = _set_idle_rpm
@export var redline_rpm: float = REDLINE_RPM: set = _set_redline_rpm
@export var shift_rpm: float = SHIFT_RPM: set = _set_shift_rpm
@export var transmission_type: TransmissionType = TransmissionType.MANUAL

@export_group("Drivetrain")
@export var drivetrain_type: DrivetrainType = DrivetrainType.AWD
@export var final_drive_ratio: float = 3.73
@export var differential_type: DifferentialType = DifferentialType.LSD

@export_group("Aerodynamics")
@export var drag_coefficient: float = 0.32
@export var frontal_area: float = 2.2
@export var downforce_at_max_speed: float = 500.0

@export_group("Tires & Suspension")
@export var tire_friction_coefficient: float = 1.2
@export var suspension_stiffness: float = 25000.0
@export var suspension_damping: float = 1500.0
@export var suspension_travel: float = 0.15

# ============================================================================
# GEAR RATIO TABLES
# ============================================================================
var _gear_ratios: Array[float] = [3.5, 2.2, 1.6, 1.2, 0.9, 0.7, 0.5]
var _reverse_ratio: float = 3.8

# ============================================================================
# INTERNAL STATE VARIABLES
# ============================================================================
var current_speed: float = 0.0
var current_rpm: float = IDLE_RPM
var current_gear: int = 0
var target_gear: int = 0
var clutch_engaged: bool = true
var turbo_charged: bool = false
var turbo_timer: float = 0.0
var drift_mode: bool = false
var drift_intensity: float = 0.0
var last_input_direction: Vector3 = Vector3.ZERO
var input_history: Array[Vector3] = []
var suspension_states: Array[SuspensionState] = []

# Wheel force application points
var _wheel_positions: Array[Vector3] = []
var _wheel_rotations: Array[float] = []
var _wheel_forces: Array[float] = []

# Collision tracking
var _collision_impact: float = 0.0
var _collision_normal: Vector3 = Vector3.UP
var _last_collision_time: float = 0.0

# Track reference
var track_reference: Node3D = null

# Input handling
var _input_throttle: float = 0.0
var _input_brake: float = 0.0
var _input_steering: float = 0.0
var _input_clutch: float = 0.0
var _input_handbrake: bool = false
var _input_shift_up: bool = false
var _input_shift_down: bool = false

# Audio references
var _engine_player: AudioStreamPlayer3D = null
var _tire_screech_player: AudioStreamPlayer3D = null

# ============================================================================
# ENUMERATIONS
# ============================================================================
enum TransmissionType {
	MANUAL,
	AUTOMATIC,
	SEMI_AUTOMATIC,
	CVT
}

enum DrivetrainType {
	FWD,
	RWD,
	AWD,
	4WD
}

enum DifferentialType {
	OPEN,
	LSD,
	LOCKED
}

class SuspensionState:
	var compression: float = 0.0
	var velocity: float = 0.0
	var spring_force: float = 0.0
	var damping_force: float = 0.0
	var ground_contact: bool = false

# ============================================================================
# INITIALIZATION
# ============================================================================
func _ready() -> void:
	_init_wheel_positions()
	_init_suspension_states()
	_connect_signals()
	_setup_audio()
	_init_physics()

# ============================================================================
# PHYSICS SETUP
# ============================================================================
func _init_wheel_positions() -> void:
	# Define wheel positions relative to vehicle center
	var wheel_base: float = 2.8
	var track_width: float = 1.6
	var wheel_offset_y: float = -0.3
	
	_wheel_positions = [
		Vector3(-wheel_base/2, wheel_offset_y, track_width/2),   # Front Left
		Vector3(-wheel_base/2, wheel_offset_y, -track_width/2),  # Front Right
		Vector3(wheel_base/2, wheel_offset_y, track_width/2),    # Rear Left
		Vector3(wheel_base/2, wheel_offset_y, -track_width/2)    # Rear Right
	]
	
	for pos in _wheel_positions:
		suspension_states.append(SuspensionState.new())

# ============================================================================
# SIGNAL CONNECTIONS
# ============================================================================
func _connect_signals() -> void:
	GameManager.game_state_changed.connect(_on_game_state_changed)
	InputManager.input_updated.connect(_on_input_updated)

# ============================================================================
# AUDIO SETUP
# ============================================================================
func _setup_audio() -> void:
	_engine_player = AudioStreamPlayer3D.new()
	_engine_player.bus = "SFX"
	_add_child(_engine_player)
	
	_tire_screech_player = AudioStreamPlayer3D.new()
	_tire_screech_player.bus = "SFX"
	_tire_screech_player.stream = _generate_tire_sound()
	_add_child(_tire_screech_player)

func _generate_tire_sound() -> AudioBuffer:
	return AudioBuffer.new()

# ============================================================================
# INPUT HANDLING
# ============================================================================
func _on_input_updated(input_data: Dictionary) -> void:
	_input_throttle = clamp(input_data.get("throttle", 0.0), -1.0, 1.0)
	_input_brake = clamp(input_data.get("brake", 0.0), 0.0, 1.0)
	_input_steering = clamp(input_data.get("steering", 0.0), -1.0, 1.0)
	_input_clutch = clamp(input_data.get("clutch", 0.0), 0.0, 1.0)
	_input_handbrake = input_data.get("handbrake", false)
	_input_shift_up = input_data.get("shift_up", false)
	_input_shift_down = input_data.get("shift_down", false)
	
	if _input_shift_up or _input_shift_down:
		_handle_gear_shifting()

# ============================================================================
# MAIN UPDATE LOOP
# ============================================================================
func _physics_process(delta: float) -> void:
	_update_physics(delta)
	_apply_forces()
	_update_suspension(delta)
	_check_drift_conditions()
	_update_audio(delta)
	_emit_signals()

# ============================================================================
# PHYSICS CALCULATION
# ============================================================================
func _update_physics(delta: float) -> void:
	# Calculate effective throttle based on clutch engagement
	var effective_throttle: float = _input_throttle * (_input_clutch if _input_clutch > 0 else 1.0)
	
	# Calculate engine RPM based on gear and speed
	_update_engine_rpm(delta, effective_throttle)
	
	# Apply acceleration or deceleration
	_apply_longitudinal_forces(effective_throttle, delta)
	
	# Apply steering
	_apply_lateral_control(delta)
	
	# Update velocity
	_update_velocity(delta)
	
	# Handle collisions
	_handle_collisions()

# ============================================================================
# ENGINE RPM MANAGEMENT
# ============================================================================
func _update_engine_rpm(delta: float, throttle_input: float) -> void:
	var gear_ratio: float = _get_current_gear_ratio()
	var wheel_speed: float = current_speed / (PhysicsSettings.default_vehicle_mass / 1000.0)
	
	# Calculate theoretical RPM based on gear ratio and speed
	var theoretical_rpm: float = wheel_speed * gear_ratio * final_drive_ratio * 100.0
	
	# Blend between theoretical and actual RPM based on clutch engagement
	if not clutch_engaged:
		current_rpm = lerp(current_rpm, IDLE_RPM, delta / CLUTCH_RELEASE_TIME)
	else:
		# Engine response curve
		var target_rpm: float = IDLE_RPM + (REDLINE_RPM - IDLE_RPM) * throttle_input
		target_rpm = lerp(target_rpm, theoretical_rpm, 0.3)
		
		# Apply RPM rise/fall rates
		var rpm_change_rate: float = (target_rpm - current_rpm) * delta * 5.0
		current_rpm += rpm_change_rate
		
		# Clamp to valid range
		current_rpm = clamp(current_rpm, IDLE_RPM, REDLINE_RPM)

# ============================================================================
# GEAR SHIFTER LOGIC
# ============================================================================
func _handle_gear_shifting() -> void:
	match transmission_type:
		TransmissionType.MANUAL:
			_manual_shift()
		TransmissionType.AUTOMATIC:
			_automatic_shift_logic()
		TransmissionType.SEMI_AUTOMATIC:
			_semi_auto_shift_logic()
		TransmissionType.CVT:
			pass # CVT doesn't have discrete gears

func _manual_shift() -> void:
	if _input_shift_up and current_gear < MAX_GEAR:
		_target_gear()
	elif _input_shift_down and current_gear > MIN_GEAR:
		_target_gear()
	else:
		return
	
	clutch_engaged = false
	await get_tree().create_timer(CLUTCH_RELEASE_TIME).timeout
	clutch_engaged = true

func _automatic_shift_logic() -> void:
	if current_rpm >= shift_rpm and current_gear < MAX_GEAR:
		_target_gear()
	elif current_rpm <= IDLE_RPM and current_gear > MIN_GEAR:
		_target_gear()

func _semi_auto_shift_logic() -> void:
	# Same as automatic but requires confirmation
	if _input_shift_up and current_rpm >= shift_rpm and current_gear < MAX_GEAR:
		_target_gear()
	elif _input_shift_down and current_rpm <= IDLE_RPM and current_gear > MIN_GEAR:
		_target_gear()

func _target_gear() -> void:
	target_gear = current_gear + 1 if _input_shift_up else current_gear - 1
	target_gear = clamp(target_gear, MIN_GEAR, MAX_GEAR)
	gear_changed.emit(target_gear)
	current_gear = target_gear

# ============================================================================
# GEAR RATIO ACCESSOR
# ============================================================================
func _get_current_gear_ratio() -> float:
	if current_gear == 0:
		return _reverse_ratio if _input_throttle < 0 else 0.0
	elif current_gear < _gear_ratios.size():
		return _gear_ratios[current_gear - 1]
	else:
		return _gear_ratios.max()

# ============================================================================
# LONGITUDINAL FORCES (ACCELERATION/BRAKING)
# ============================================================================
func _apply_longitudinal_forces(throttle: float, delta: float) -> void:
	var mass: float = vehicle_mass
	var max_force: float = engine_torque * final_drive_ratio / 0.3
	
	# Calculate drive force from engine
	var drive_force: float = 0.0
	if current_gear > 0:
		drive_force = engine_torque * _get_current_gear_ratio() * final_drive_ratio * 0.1
		drive_force *= throttle * 10.0
	
	# Apply drive force in forward direction
	var forward_vector: Vector3 = transform.basis.z.normalized()
	add_force(forward_vector * drive_force)
	
	# Apply braking force
	if _input_brake > 0:
		var brake_force: float = braking_force * mass * _input_brake
		add_force(-forward_vector * brake_force)
	
	# Apply air resistance
	var air_resistance: float = 0.5 * drag_coefficient * frontal_area * \
		(current_speed * current_speed) / 1000.0
	add_force(-forward_vector * air_resistance)

# ============================================================================
# LATERAL CONTROL (STEERING)
# ============================================================================
func _apply_lateral_control(delta: float) -> void:
	if current_speed < 1.0:
		return
	
	var steering_input: float = _input_steering * turn_sensitivity
	var turn_factor: float = 1.0 - (current_speed / MAX_SPEED_KMH)
	turn_factor = pow(turn_factor, 2.0)
	
	var lateral_force: float = steering_input * turn_factor * suspension_stiffness * 0.1
	
	# Apply steering torque
	var up_vector: Vector3 = transform.basis.y
	var steer_axis: Vector3 = cross(up_vector, transform.basis.z)
	torque = steer_axis * lateral_force * delta

# ============================================================================
# VELOCITY UPDATE
# ============================================================================
func _update_velocity(delta: float) -> void:
	velocity = move_and_slide_with_custom_velocity(
		transform.basis.z * current_speed, 
		delta
	)
	current_speed = velocity.length()

# ============================================================================
# WHEEL FORCE APPLICATION
# ============================================================================
func _apply_forces() -> void:
	for i in range(4):
		var wheel_pos: Vector3 = transform.basis * _wheel_positions[i] + global_position
		var wheel_force: float = _calculate_wheel_force(i)
		_wheel_forces[i] = wheel_force
		
		# Apply downward force for weight transfer
		var gravity_force: float = (vehicle_mass * 9.81) / 4.0 * 0.7
		add_force(Vector3.DOWN * gravity_force)

# ============================================================================
# WHEEL FORCE CALCULATION
# ============================================================================
func _calculate_wheel_force(wheel_index: int) -> float:
	var is_front: bool = wheel_index < 2
	var is_rear: bool = wheel_index >= 2
	
	var base_force: float = engine_torque * _get_current_gear_ratio() * 0.05
	
	# Weight transfer calculation
	var weight_transfer: float = 0.0
	if current_speed > 0:
		var longitudinal_acc: float = _input_throttle * ACCELERATION_RATE
		var rear_weight_share: float = 0.5 + (longitudinal_acc / 20.0)
		if is_rear:
			base_force *= rear_weight_share
		else:
			base_force *= (1.0 - rear_weight_share)
	
	# Drivetrain distribution
	match drivetrain_type:
		DrivetrainType.FWD:
			return base_force if is_front else 0.0
		DrivetrainType.RWD:
			return base_force if is_rear else 0.0
		DrivetrainType.AWD:
			return base_force * 0.5
		DrivetrainType._4WD:
			return base_force * 0.6
	
	return base_force

# ============================================================================
# SUSPENSION SYSTEM
# ============================================================================
func _update_suspension(delta: float) -> void:
	for i in range(suspension_states.size()):
		var state: SuspensionState = suspension_states[i]
		
		# Calculate compression based on height difference
		var target_height: float = suspension_travel * 0.5
		var current_height: float = target_height
		
		state.compression = lerp(state.compression, current_height, delta * 10.0)
		
		# Spring force calculation
		state.spring_force = -suspension_stiffness * state.compression
		
		# Damping force calculation
		state.damping_force = -suspension_damping * state.velocity
		
		# Total suspension force
		var total_force: float = state.spring_force + state.damping_force
		
		# Apply suspension force to vehicle
		var wheel_global_pos: Vector3 = global_position + transform.basis * _wheel_positions[i]
		add_force(Vector3.UP * total_force)
		
		# Emit suspension signal
		if abs(state.compression) > SUSPENSION_COMPRESSION_LIMIT * 0.5:
			suspension_compressed.emit(state.compression)

# ============================================================================
# DRIFT MECHANICS
# ============================================================================
func _check_drift_conditions() -> void:
	if _input_handbrake and abs(_input_steering) > DRIFT_THRESHOLD:
		if current_speed > 10.0:
			if not drift_mode:
				_start_drift()
			else:
				_drift_intensity = min(_drift_intensity + 0.05, DRIFT_INTENSITY_MAX)
		else:
			if drift_mode:
				_end_drift()
	elif not _input_handbrake:
		if drift_mode:
			_end_drift()
		_drift_intensity = max(_drift_intensity - 0.02, 0.0)

func _start_drift() -> void:
	drift_mode = true
	drift_started.emit(_drift_intensity)

func _end_drift() -> void:
	drift_mode = false
	drift_ended.emit()

# ============================================================================
# COLLISION HANDLING
# ============================================================================
func _handle_collisions() -> void:
	for i in range(get_slide_collision_count()):
		var collision: KinematicCollision3D = get_slide_collision(i)
		var impact_velocity: float = velocity.dot(collision.get_normal())
		
		if abs(impact_velocity) > 5.0:
			_collision_impact = abs(impact_velocity)
			_collision_normal = collision.get_normal()
			_last_collision_time = Time.get_ticks_msec()
			
			collision_detected.emit({
				"impact_velocity": impact_velocity,
				"normal": _collision_normal,
				"position": collision.position,
				"body": collision.get_collider()
			})

# ============================================================================
# AUDIO UPDATES
# ============================================================================
func _update_audio(delta: float) -> void:
	if _engine_player:
		var rpm_ratio: float = (current_rpm - IDLE_RPM) / (REDLINE_RPM - IDLE_RPM)
		rpm_ratio = clamp(rpm_ratio, 0.0, 1.0)
		_engine_player.pitch_scale = 0.5 + rpm_ratio * 1.5
		engine_sound_changed.emit(rpm_ratio)

# ============================================================================
# SIGNAL EMITTERS
# ============================================================================
func _emit_signals() -> void:
	speed_changed.emit(current_speed)
	rpm_changed.emit(current_rpm)

# ============================================================================
# PROPERTY SETTERS
# ============================================================================
func _set_max_speed(value: float) -> void:
	max_speed = value

func _set_acceleration_power(value: float) -> void:
	acceleration_power = value

func _set_braking_force(value: float) -> void:
	braking_force = value

func _set_turn_sensitivity(value: float) -> void:
	turn_sensitivity = value

func _set_vehicle_mass(value: float) -> void:
	vehicle_mass = value

func _set_idle_rpm(value: float) -> void:
	idle_rpm = value

func _set_redline_rpm(value: float) -> void:
	redline_rpm = value

func _set_shift_rpm(value: float) -> void:
	shift_rpm = value

# ============================================================================
# GAME STATE HANDLERS
# ============================================================================
func _on_game_state_changed(new_state: GameState) -> void:
	match new_state:
		GameManager.GameState.RACE_ACTIVE:
			_on_race_started()
		GameManager.GameState.RACE_PAUSED:
			_on_race_paused()
		GameManager.GameState.RACE_FINISHED:
			_on_race_finished()

func _on_race_started() -> void:
	current_speed = 0.0
	current_rpm = IDLE_RPM
	current_gear = 1
	clutch_engaged = true

func _on_race_paused() -> void:
	pass

func _on_race_finished() -> void:
	current_speed = 0.0
	current_rpm = IDLE_RPM
	current_gear = 0
	_reset_vehicle()

# ============================================================================
# VEHICLE RESET
# ============================================================================
func _reset_vehicle() -> void:
	velocity = Vector3.ZERO
	current_speed = 0.0
	current_rpm = IDLE_RPM
	current_gear = 0
	target_gear = 0
	clutch_engaged = true
	drift_mode = false
	_drift_intensity = 0.0

# ============================================================================
# DEBUG TOOLS
# ============================================================================
func _draw_debug() -> void:
	if GameManager.debug_mode:
		# Draw wheel positions
		for i in range(_wheel_positions.size()):
			var world_pos: Vector3 = global_position + transform.basis * _wheel_positions[i]
			Debug.draw_sphere(world_pos, 0.2, Color.YELLOW)

# ============================================================================
# PUBLIC API
# ============================================================================
func get_current_speed_kmh() -> float:
	return current_speed * 3.6

func get_current_rpm() -> float:
	return current_rpm

func get_current_gear() -> int:
	return current_gear

func is_drifting() -> bool:
	return drift_mode

func get_drift_intensity() -> float:
	return _drift_intensity

func reset() -> void:
	_reset_vehicle()

func set_gear(gear: int) -> void:
	current_gear = clamp(gear, MIN_GEAR, MAX_GEAR)
	target_gear = current_gear

func enable_turbo(enable: bool) -> void:
	turbo_charged = enable
	if enable:
		turbo_timer = TURBO_CHARGE_TIME

func update_turbo(delta: float) -> void:
	if turbo_charged:
		turbo_timer -= delta
		if turbo_timer <= 0:
			turbo_charged = false

func add_turbo_boost() -> void:
	if turbo_charged:
		acceleration_power *= 1.5

</file_content>