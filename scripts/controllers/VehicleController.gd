extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulator
## Implements throttle, brake, steering inputs, wheel forces, and gear shifting logic
## Uses PhysicsSettings constants for centralized tuning
## Copyright 2026 Thalamus Racing Simulator Project

# ============================================================================
# SIGNALS
# ============================================================================
signal speed_changed(new_speed: float)
signal rpm_changed(new_rpm: float)
signal gear_changed(old_gear: int, new_gear: int)
signal throttle_applied(amount: float)
signal brake_applied(amount: float)
signal steering_angle_changed(angle: float)
signal skidding(is_skidding: bool)
signal collision_detected(collision_info: Dictionary)
signal engine_stalled()
signal handbrake_toggled(is_active: bool)
signal traction_control_state_changed(active: bool)
signal anti_lock_braking_state_changed(active: bool)
signal drift_started(drift_angle: float)
signal drift_ended()
signal lap_completed(lap_time: float)
signal checkpoint_passed(checkpoint_id: int)
signal boost_used(duration: float)

# ============================================================================
# CONFIGURATION CONSTANTS
# ============================================================================
const MAX_SPEED_KMH: float = 350.0
const ACCELERATION_POWER: float = 20000.0
const BRAKE_FORCE: float = 35000.0
const STEERING_SPEED: float = 2.5
const MAX_STEER_ANGLE: float = PI / 3.5
const GEAR_RATIOS: Array[float] = [4.5, 2.8, 1.9, 1.4, 1.1, 0.9]
const FINAL_DRIVE: float = 3.5
const CLUTCH_DISengage_THRESHOLD: float = 0.05
const TRACTION_CONTROL_MAX_SLIP: float = 0.15
const ABS_BRAKE_MODULATION: float = 0.8
const DRIFT_THRESHOLD: float = 0.4
const DRIFT_RECOVERY_TORQUE: float = 0.3

# ============================================================================
# ENUMERATIONS
# ============================================================================
enum Gear {
	NEUTRAL = 0,
	FIRST = 1,
	SECOND = 2,
	THIRD = 3,
	FOURTH = 4,
	FIFTH = 5,
	SIXTH = 6,
	REVERSE = -1
}

enum BrakeMode {
	NORMAL,
	ABS,
	DIRECT
}

enum TractionControlMode {
	NONE,
	MILD,
	STRICT
}

# ============================================================================
# EXPORTED VARIABLES (Inspector Editable)
# ============================================================================
@export_group("Vehicle Configuration")
@export var vehicle_mass: float = 1500.0
@export var center_of_mass_offset: Vector3 = Vector3(0.0, 0.5, 0.0)
@export var wheel_base: float = 2.6
@export var track_width: float = 1.7
@export var max_rpm: float = 8500.0
@export var idle_rpm: float = 800.0
@export var redline_rpm: float = 7500.0
@export var rev_limit_rpm: float = 8500.0

@export_group("Wheel Configuration")
@export var front_wheel_radius: float = 0.32
@export var rear_wheel_radius: float = 0.35
@export var suspension_travel: float = 0.15
@export var suspension_stiffness: float = 50000.0
@export var damping_rate: float = 8000.0

@export_group("Drivetrain Settings")
@export var drivetrain_type: DrivetrainType = DrivetrainType.RWD
@export enum DrivetrainType { FWD, RWD, AWD }
@export var torque_bias_front: float = 0.0
@export var torque_bias_rear: float = 1.0

@export_group("Physics Tuning")
@export var tire_friction_coefficient: float = 1.2
@export var wheel_slip_threshold: float = 0.3
@export var aerodynamic_drag_coefficient: float = 0.32
@export var frontal_area: float = 2.1
@export var downforce_coefficient: float = 0.5

@export_group("Driver Assist Systems")
@export var traction_control_enabled: bool = true
@export var abs_enabled: bool = true
@export var stability_control_enabled: bool = true
@export var auto_clutch_enabled: bool = false
@export var manual_mode: bool = false

# ============================================================================
# PRIVATE STATE VARIABLES
# ============================================================================
var _current_gear: int = Gear.NEUTRAL
var _target_gear: int = Gear.NEUTRAL
var _rpm: float = 0.0
var _speed_kmh: float = 0.0
var _throttle_input: float = 0.0
var _brake_input: float = 0.0
var _steering_input: float = 0.0
var _handbrake_active: bool = false
var _engine_running: bool = false
var _clutch_engaged: bool = true
var _wheel_slip_front: Vector2 = Vector2.ZERO
var _wheel_slip_rear: Vector2 = Vector2.ZERO
var _drift_angle: float = 0.0
var _is_drifting: bool = false
var _collision_normal: Vector3 = Vector3.UP
var _last_collision_velocity: float = 0.0
var _lap_start_time: float = 0.0
var _current_lap_time: float = 0.0
var _checkpoints_passed: Array[int] = []
var _last_checkpoint_id: int = -1
var _boost_available: bool = false
var _boost_timer: float = 0.0

# Wheel positions relative to vehicle body
var _front_left_pos: Vector3 = Vector3.ZERO
var _front_right_pos: Vector3 = Vector3.ZERO
var _rear_left_pos: Vector3 = Vector3.ZERO
var _rear_right_pos: Vector3 = Vector3.ZERO

# Suspension state
var _suspension_compression_front: Vector2 = Vector2.ONE
var _suspension_compression_rear: Vector2 = Vector2.ONE

# Input buffers for smooth transitions
var _input_buffer_throttle: float = 0.0
var _input_buffer_brake: float = 0.0
var _input_buffer_steering: float = 0.0

# ============================================================================
# INITIALIZATION
# ============================================================================
func _ready() -> void:
	_setup_wheel_positions()
	_connect_signals_to_game_manager()
	_init_audio_system()
	_reset_vehicle_state()
	print("[VehicleController] Ready - Vehicle initialized successfully")

func _setup_wheel_positions() -> void:
	var half_track = track_width * 0.5
	var half_wb = wheel_base * 0.5
	
	_front_left_pos = Vector3(-half_track, 0.0, half_wb)
	_front_right_pos = Vector3(half_track, 0.0, half_wb)
	_rear_left_pos = Vector3(-half_track, 0.0, -half_wb)
	_rear_right_pos = Vector3(half_track, 0.0, -half_wb)

func _connect_signals_to_game_manager() -> void:
	if GameManager:
		GameManager.race_started.connect(_on_race_started)
		GameManager.game_state_changed.connect(_on_game_state_changed)

func _init_audio_system() -> void:
	# Audio system will be handled by AudioManager singleton
	pass

func _reset_vehicle_state() -> void:
	_current_gear = Gear.NEUTRAL
	_target_gear = Gear.NEUTRAL
	_rpm = idle_rpm
	_speed_kmh = 0.0
	_throttle_input = 0.0
	_brake_input = 0.0
	_steering_input = 0.0
	_handbrake_active = false
	_engine_running = false
	_clutch_engaged = true
	_wheel_slip_front = Vector2.ZERO
	_wheel_slip_rear = Vector2.ZERO
	_drift_angle = 0.0
	_is_drifting = false
	_checkpoints_passed.clear()
	_last_checkpoint_id = -1
	_boost_available = false
	_boost_timer = 0.0

# ============================================================================
# MAIN PROCESS LOOP
# ============================================================================
func _physics_process(delta: float) -> void:
	if not _engine_running:
		return
	
	# Update input buffers
	_update_input_buffers(delta)
	
	# Read current inputs from InputManager
	_read_inputs()
	
	# Update RPM based on gear and throttle
	_update_rpm(delta)
	
	# Handle gear shifting
	_handle_gear_shifting(delta)
	
	# Calculate wheel forces
	_calculate_wheel_forces(delta)
	
	# Apply physics movement
	_apply_physics_movement(delta)
	
	# Check for drifting conditions
	_update_drift_state(delta)
	
	# Update collision detection
	_update_collision_detection()
	
	# Update lap timing if race active
	if GameManager.current_state == GameManager.GameState.RACE_ACTIVE:
		_update_lap_timing(delta)
	
	# Emit signals for changed states
	_emit_state_signals()
	
	# Process boost timer
	if _boost_timer > 0.0:
		_boost_timer -= delta
		if _boost_timer <= 0.0:
			_boost_available = true

# ============================================================================
# INPUT HANDLING
# ============================================================================
func _read_inputs() -> void:
	if InputManager:
		# Read throttle input (0.0 to 1.0)
		_throttle_input = InputManager.get_axis("accelerate", "brake_reverse")
		
		# Read brake input (0.0 to 1.0)
		_brake_input = InputManager.get_axis("brake", "reverse")
		
		# Read steering input (-1.0 to 1.0)
		_steering_input = InputManager.get_axis("steer_left", "steer_right")
		
		# Handbrake toggle
		if InputManager.is_action_just_pressed("handbrake"):
			_handbrake_active = not _handbrake_active
			emit_signal(handbrake_toggled, _handbrake_active)
		
		# Manual gear shift inputs
		if not manual_mode:
			if InputManager.is_action_just_pressed("gear_up"):
				_request_gear_shift(Gear.FIRST + _current_gear)
			elif InputManager.is_action_just_pressed("gear_down"):
				_request_gear_shift(Gear.FIRST + _current_gear - 1)
		else:
			# Manual mode gear handling
			if InputManager.is_action_just_pressed("gear_up"):
				_request_gear_shift(_current_gear + 1)
			elif InputManager.is_action_just_pressed("gear_down"):
				_request_gear_shift(_current_gear - 1)
			elif InputManager.is_action_just_pressed("neutral"):
				_request_gear_shift(Gear.NEUTRAL)
			elif InputManager.is_action_just_pressed("reverse"):
				_request_gear_shift(Gear.REVERSE)

func _update_input_buffers(delta: float) -> void:
	var smoothing_factor = 10.0 * delta
	
	_input_buffer_throttle = lerp(_input_buffer_throttle, _throttle_input, smoothing_factor)
	_input_buffer_brake = lerp(_input_buffer_brake, _brake_input, smoothing_factor)
	_input_buffer_steering = lerp(_input_buffer_steering, _steering_input, smoothing_factor)

# ============================================================================
# RPM & ENGINE MANAGEMENT
# ============================================================================
func _update_rpm(delta: float) -> void:
	var target_rpm: float = 0.0
	
	match _current_gear:
		Gear.NEUTRAL:
			target_rpm = _get_idle_rpm()
		
		Gear.REVERSE:
			target_rpm = _calculate_rpm_from_velocity(-_speed_kmh * 1000.0 / 3600.0)
		
		_:
			target_rpm = _calculate_rpm_from_velocity(_speed_kmh * 1000.0 / 3600.0)
	
	# Apply throttle influence on RPM
	var throttle_influence = _input_buffer_throttle * (max_rpm - target_rpm)
	target_rpm += throttle_influence
	
	# Clamp RPM within valid range
	target_rpm = clamp(target_rpm, idle_rpm, rev_limit_rpm)
	
	# Smooth RPM transition
	_rpm = lerp(_rpm, target_rpm, 5.0 * delta)
	
	# Check for stall condition
	if _rpm < idle_rpm * 0.5 and _input_buffer_throttle < CLUTCH_DISengage_THRESHOLD:
		_engine_stalled.emit()
		_engine_running = false
		_rpm = idle_rpm
	
	# Redline protection
	if _rpm >= redline_rpm:
		_rpm = redline_rpm

func _get_idle_rpm() -> float:
	return idle_rpm * (1.0 + randf() * 0.02)  # Slight variation for realism

func _calculate_rpm_from_velocity(velocity_ms: float) -> float:
	if velocity_ms == 0.0:
		return idle_rpm
	
	var wheel_circumference: float = 2.0 * PI * ((front_wheel_radius + rear_wheel_radius) * 0.5)
	var wheel_rps: float = velocity_ms / wheel_circumference
	
	var total_ratio = GEAR_RATIOS[abs(_current_gear) - 1] * FINAL_DRIVE if _current_gear != Gear.NEUTRAL else 1.0
	
	return wheel_rps * total_ratio * 60.0  # Convert to RPM

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================
func _request_gear_shift(target_gear: int) -> void:
	if target_gear < Gear.NEUTRAL or target_gear > Gear.SIXTH:
		return
	
	if _current_gear == target_gear:
		return
	
	# Auto-clutch handling
	if auto_clutch_enabled and _clutch_engaged:
		_clutch_engaged = false
		await _wait_for_clutch_disengage()
	
	_target_gear = target_gear

func _handle_gear_shifting(delta: float) -> void:
	if _target_gear == _current_gear:
		return
	
	# Check if shift is possible (RPM match)
	var gear_change_ok = _can_perform_gear_shift()
	
	if gear_change_ok:
		var old_gear = _current_gear
		_current_gear = _target_gear
		
		# Re-engage clutch after brief delay
		if auto_clutch_enabled:
			await _wait_for_clutch_disengage()
			_clutch_engaged = true
		
		_target_gear = Gear.NEUTRAL
		emit_signal(gear_changed, old_gear, _current_gear)

func _can_perform_gear_shift() -> bool:
	var rpm_margin = 500.0
	
	if _current_gear == Gear.NEUTRAL:
		return true
	
	if _current_gear > 0:  # Upshift
		return _rpm < (idle_rpm + rpm_margin)
	else:  # Downshift
		return _rpm > (redline_rpm - rpm_margin)

func _wait_for_clutch_disengage() -> void:
	var timeout = 0.3
	var elapsed = 0.0
	while elapsed < timeout and not _clutch_engaged:
		await get_tree().create_timer(0.016).timeout
		elapsed += 0.016

# ============================================================================
# WHEEL FORCE CALCULATION
# ============================================================================
func _calculate_wheel_forces(delta: float) -> void:
	var driving_force: float = 0.0
	var braking_force: float = 0.0
	
	# Calculate driving force based on gear and RPM
	if _current_gear != Gear.NEUTRAL and _current_gear != Gear.REVERSE:
		driving_force = _calculate_engine_torque() * _get_gear_ratio() * FINAL_DRIVE * (1.0 / wheel_base)
	
	# Apply drivetrain torque distribution
	var drive_wheels = _get_drive_wheels()
	for wheel in drive_wheels:
		wheel.force = driving_force * _get_wheel_torque_distribution(wheel)
	
	# Apply braking force
	if _input_buffer_brake > 0.0 or _handbrake_active:
		braking_force = BRAKE_FORCE * _input_buffer_brake
		if _handbrake_active:
			braking_force *= 1.5  # Handbrake bonus
		_apply_braking_force(braking_force)

func _calculate_engine_torque() -> float:
	# Simplified torque curve based on RPM
	var normalized_rpm = (_rpm - idle_rpm) / (max_rpm - idle_rpm)
	var peak_torque_rpm = 0.65
	
	var torque_curve = sin(normalized_rpm * PI * 0.5) * 0.5 + sin(normalized_rpm * PI) * 0.5
	
	return torque_curve * ACCELERATION_POWER

func _get_gear_ratio() -> float:
	if _current_gear == Gear.NEUTRAL:
		return 1.0
	return GEAR_RATIOS[abs(_current_gear) - 1]

func _get_drive_wheels() -> Array[String]:
	match drivetrain_type:
		DrivetrainType.FWD:
			return ["front_left", "front_right"]
		DrivetrainType.RWD:
			return ["rear_left", "rear_right"]
		DrivetrainType.AWD:
			return ["front_left", "front_right", "rear_left", "rear_right"]
	return []

func _get_wheel_torque_distribution(wheel: String) -> float:
	match drivetrain_type:
		DrivetrainType.FWD:
			return 0.5
		DrivetrainType.RWD:
			return 0.5
		DrivetrainType.AWD:
			return 0.25
	return 0.5

func _apply_braking_force(force: float) -> void:
	# Apply braking force to all wheels
	var wheels = ["front_left", "front_right", "rear_left", "rear_right"]
	for wheel in wheels:
		wheel.brake_force = force

# ============================================================================
# PHYSICS MOVEMENT
# ============================================================================
func _apply_physics_movement(delta: float) -> void:
	# Calculate velocity vector
	var forward_vector = transform.basis.z
	var right_vector = transform.basis.x
	
	# Apply steering rotation
	var steer_angle = _input_buffer_steering * MAX_STEER_ANGLE
	transform.basis = transform.basis.rotated(Vector3.UP, steer_angle * delta * STEERING_SPEED)
	
	# Calculate acceleration based on driving forces
	var acceleration: float = 0.0
	
	if _current_gear != Gear.NEUTRAL:
		var engine_acceleration = _calculate_engine_acceleration()
		var drag_deceleration = _calculate_drag_force()
		acceleration = engine_acceleration - drag_deceleration
	
	# Apply braking deceleration
	if _input_buffer_brake > 0.0 or _handbrake_active:
		acceleration -= _input_buffer_brake * BRAKE_FORCE / vehicle_mass
	
	# Apply acceleration to velocity
	velocity += forward_vector * acceleration * delta
	
	# Apply friction/drag
	velocity *= 1.0 - (0.01 * delta)
	
	# Update speed
	_speed_kmh = velocity.length() * 3.6
	
	# Limit maximum speed
	if _speed_kmh > MAX_SPEED_KMH:
		velocity = velocity.normalized() * (MAX_SPEED_KMH / 3.6)
		_speed_kmh = MAX_SPEED_KMH
	
	# Apply gravity
	velocity.y -= PhysicsSettings.gravity * delta
	
	# Move character body
	move_and_slide()

func _calculate_engine_acceleration() -> float:
	var gear_ratio = _get_gear_ratio()
	var engine_power = ACCELERATION_POWER * gear_ratio
	return engine_power / vehicle_mass

func _calculate_drag_force() -> float:
	var velocity_ms = _speed_kmh * 1000.0 / 3600.0
	var drag_force = 0.5 * aerodynamic_drag_coefficient * frontal_area * velocity_ms * velocity_ms
	return drag_force / vehicle_mass

# ============================================================================
# DRIFT MECHANICS
# ============================================================================
func _update_drift_state(delta: float) -> void:
	if _speed_kmh < 30.0:  # Minimum speed for drifting
		_is_drifting = false
		_drift_angle = 0.0
		return
	
	# Calculate lateral slip angle
	var lateral_velocity = velocity.dot(transform.basis.x)
	var longitudinal_velocity = velocity.dot(transform.basis.z)
	
	if longitudinal_velocity.abs() > 0.1:
		var slip_angle = atan(lateral_velocity / longitudinal_velocity.abs())
		slip_angle = sign(slip_angle) * abs(slip_angle)
		
		# Check if drifting threshold exceeded
		if abs(slip_angle) > DRIFT_THRESHOLD:
			_is_drifting = true
			_drift_angle = slip_angle
			
			if not drift_started.is_connected(_on_drift_started):
				drift_started.connect(_on_drift_started)
			
			# Apply drift recovery torque
			if stability_control_enabled:
				var recovery_torque = _drift_angle * DRIFT_RECOVERY_TORQUE
				transform.basis = transform.basis.rotated(Vector3.UP, recovery_torque * delta)
		else:
			if _is_drifting:
				_is_drifting = false
				_drift_angle = 0.0
				emit_signal(drift_ended())

func _on_drift_started(drift_angle: float) -> void:
	print("[Drift] Drifting started at angle: ", drift_angle)
	AudioManager.play_sfx("drift_start")

# ============================================================================
# COLLISION DETECTION
# ============================================================================
func _update_collision_detection() -> void:
	for i in get_slide_count():
		var collision = get_slide_collision(i)
		if collision:
			var collision_velocity = collision.get_contact_normal().dot(velocity)
			var impact_force = collision_velocity * vehicle_mass
			
			if abs(impact_force) > _last_collision_velocity:
				_last_collision_velocity = abs(impact_force)
				
				collision_detected.emit({
					"position": collision.get_position(),
					"normal": collision.get_contact_normal(),
					"velocity": collision_velocity,
					"impact_force": impact_force,
					"collider": collision.get_collider()
				})
				
				# Screen shake effect
				if GameManager.debug_mode:
					_trigger_screen_shake(impact_force / 1000.0)

func _trigger_screen_shake(intensity: float) -> void:
	if GameManager and GameManager._camera_shake:
		GameManager._camera_shake.shake(intensity)

# ============================================================================
# LAP TIMING SYSTEM
# ============================================================================
func _update_lap_timing(delta: float) -> void:
	if GameManager.current_state == GameManager.GameState.RACE_ACTIVE:
		if _lap_start_time == 0.0:
			_lap_start_time = Time.get_unix_time_from_system()
		
		_current_lap_time = Time.get_unix_time_from_system() - _lap_start_time
		
		# Check checkpoints
		_check_checkpoint_passes()

func _check_checkpoint_passes() -> void:
	# Simple checkpoint detection using position-based triggers
	# In a full implementation, this would use Area3D nodes
	var current_position = global_position
	
	# Example checkpoint zones (would be defined per track)
	var checkpoints = [
		Vector3(0.0, 0.0, 100.0),
		Vector3(100.0, 0.0, 50.0),
		Vector3(0.0, 0.0, 0.0),
		Vector3(-100.0, 0.0, 50.0)
	]
	
	for i in range(checkpoints.size()):
		var distance = current_position.distance_to(checkpoints[i])
		if distance < 10.0 and i > _last_checkpoint_id:
			_checkpoints_passed.append(i)
			_last_checkpoint_id = i
			emit_signal(checkpoint_passed, i)
			AudioManager.play_sfx("checkpoint")

# ============================================================================
# GAME STATE HANDLERS
# ============================================================================
func _on_race_started(race_data: Dictionary) -> void:
	_reset_vehicle_state()
	_lap_start_time = Time.get_unix_time_from_system()
	_engine_running = true
	print("[VehicleController] Race started - Vehicle engaged")

func _on_game_state_changed(new_state: GameState) -> void:
	match new_state:
		GameManager.GameState.MAIN_MENU:
			_engine_running = false
			_current_gear = Gear.NEUTRAL
		GameManager.GameState.RACE_PAUSED:
			# Pause physics processing
			_process_mode = ProcessModeEnum.ALWAYS
		GameManager.GameState.RACE_FINISHED:
			# Calculate final results
			var result = {
				"final_position": 0,
				"total_laps": _checkpoints_passed.size(),
				"best_lap_time": _current_lap_time,
				"peak_speed": _speed_kmh
			}
			GameManager.emit_signal("race_ended", result)

# ============================================================================
# PUBLIC API METHODS
# ============================================================================
func start_engine() -> void:
	_engine_running = true
	_rpm = idle_rpm
	AudioManager.play_sfx("engine_start")

func stop_engine() -> void:
	_engine_running = false
	_rpm = 0.0
	AudioManager.play_sfx("engine_stop")

func reset_vehicle() -> void:
	_reset_vehicle_state()
	velocity = Vector3.ZERO
	global_position = Vector3.ZERO
	transform.basis = Basis.IDENTITY

func get_speed_kmh() -> float:
	return _speed_kmh

func get_rpm() -> float:
	return _rpm

func get_current_gear() -> int:
	return _current_gear

func get_throttle_input() -> float:
	return _input_buffer_throttle

func get_brake_input() -> float:
	return _input_buffer_brake

func get_steering_input() -> float:
	return _input_buffer_steering

func set_manual_mode(enabled: bool) -> void:
	manual_mode = enabled

func enable_traction_control(enabled: bool) -> void:
	traction_control_enabled = enabled
	emit_signal(traction_control_state_changed, enabled)

func enable_abs(enabled: bool) -> void:
	abs_enabled = enabled
	emit_signal(anti_lock_braking_state_changed, enabled)

func activate_boost(duration: float) -> void:
	_boost_available = false
	_boost_timer = duration
	AudioManager.play_sfx("boost_activate")

# ============================================================================
# HELPER FUNCTIONS
# ============================================================================
func _emit_state_signals() -> void:
	if _speed_kmh != 0.0:
		emit_signal(speed_changed, _speed_kmh)
	
	if _rpm != 0.0:
		emit_signal(rpm_changed, _rpm)
	
	emit_signal(throttle_applied, _input_buffer_throttle)
	emit_signal(brake_applied, _input_buffer_brake)
	emit_signal(steering_angle_changed, _input_buffer_steering * MAX_STEER_ANGLE)
	emit_signal(skidding, _is_drifting)

func _on_drift_ended() -> void:
	print("[Drift] Drifting ended")
	AudioManager.play_sfx("drift_end")

# ============================================================================
# DEBUG MODE
# ============================================================================
func _process(debug_delta: float) -> void:
	if GameManager and GameManager.debug_mode:
		_show_debug_hud()

func _show_debug_hud() -> void:
	var hud_text = """
Vehiclespeed: %.1f km/h | RPM: %.0f | Gear: %d
Throttle: %.2f | Brake: %.2f | Steering: %.2f
Drift: %s (%.2f°) | Engine: %s
""" % [
	_speed_kmh,
	_rpm,
	_current_gear,
	_input_buffer_throttle,
	_input_buffer_brake,
	_input_buffer_steering,
	"is_drifting" if _is_drifting else "not_drifting",
	_drift_angle,
	"running" if _engine_running else "stopped"
]
	# In production, this would draw to a debug canvas
	# For now, just print to console
	print(hud_text)

# ============================================================================
# DESTRUCTION CLEANUP
# ============================================================================
func _notification(what: int) -> void:
	if what == NOTIFICATION_PREDELETE:
		print("[VehicleController] Cleaning up vehicle resources")
</script>>

Now let me verify the file was created correctly and check if we need any additional files for the vehicle system.

{"op":"cmd","command":"cat scripts/controllers/VehicleController.gd | wc -l"}<<FILE "scripts/controllers/VehicleController.gd">>
	if GameManager and GameManager.debug_mode:
		_show_debug_hud()

func _show_debug_hud() -> void:
	var hud_text = """
Vehiclespeed: %.1f km/h | RPM: %.0f | Gear: %d
Throttle: %.2f | Brake: %.2f | Steering: %.2f
Drift: %s (%.2f°) | Engine: %s
""" % [
	_speed_kmh,
	_rpm,
	_current_gear,
	_input_buffer_throttle,
	_input_buffer_brake,
	_input_buffer_steering,
	"is_drifting" if _is_drifting else "not_drifting",
	_drift_angle,
	"running" if _engine_running else "stopped"
]
	# In production, this would draw to a debug canvas
	# For now, just print to console
	print(hud_text)

# ============================================================================
# DESTRUCTION CLEANUP
# ============================================================================
func _notification(what: int) -> void:
	if what == NOTIFICATION_PREDELETE:
		print("[VehicleController] Cleaning up vehicle resources")