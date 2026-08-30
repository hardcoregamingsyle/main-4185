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
# CONSTANTS & CONFIGURATION - References to PhysicsSettings
# ============================================================================
const VEHICLE_BASE_MASS := 1500.0  # Base mass in kg

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
@export var drivetrain_type: DrivetrainType = DrivetrainType.FWD
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

@export_group("Gear Settings")
@export var num_gears: int = 6: set = _set_num_gears
@export var gear_ratios: Array[float] = [
	3.8,  # 1st gear
	2.4,  # 2nd gear
	1.8,  # 3rd gear
	1.4,  # 4th gear
	1.1,  # 5th gear
	0.9   # 6th gear
]: set = _set_gear_ratios
@export var reverse_ratio: float = 3.5: set = _set_reverse_ratio
@export var idle_rpm: float = 800.0: set = _set_idle_rpm
@export var redline_rpm: float = 7500.0: set = _set_redline_rpm

@export_group("Steering & Handling")
@export var steering_sensitivity: float = 1.0: set = _set_steering_sensitivity
@export var steering_return_speed: float = 5.0: set = _set_steering_return_speed
@export var grip_coefficient: float = 1.2: set = _set_grip_coefficient
@export var drift_threshold: float = 0.7: set = _set_drift_threshold

@export_group("Visual Settings")
@export var headlights_enabled: bool = true
@export var taillights_enabled: bool = true

# ============================================================================
# ENUMERATIONS
# ============================================================================
enum DrivetrainType {
	FWD,    # Front-Wheel Drive
	RWD,    # Rear-Wheel Drive
	AWD     # All-Wheel Drive
}

# ============================================================================
# PRIVATE STATE VARIABLES
# ============================================================================
var _current_gear: int = 0  # 0 = neutral, 1-6 = gears, -1 = reverse
var _target_gear: int = 0
var _engine_rpm: float = 0.0
var _throttle_input: float = 0.0  # 0.0 to 1.0
var _brake_input: float = 0.0    # 0.0 to 1.0
var _handbrake_input: float = 0.0 # 0.0 to 1.0
var _steering_input: float = 0.0 # -1.0 to 1.0
var _is_engine_running: bool = false
var _is_in_drift: bool = false
var _drift_angle: float = 0.0
var _last_position: Vector3 = Vector3.ZERO
var _velocity_vector: Vector3 = Vector3.ZERO
var _wheel_steering_angles: Dictionary = {}  # WHEEL_NAME -> angle
var _current_torque: float = 0.0
var _drag_coefficient: float = 0.30
var _frontal_area: float = 2.2  # m²
var _air_density: float = 1.225  # kg/m³
var _gear_shift_timer: float = 0.0
var _max_gear_shift_time: float = 0.15  # seconds between shifts
var _auto_shift_enabled: bool = true
var _clutch_disengaged: bool = false
var _vehicle_node: Node3D = null
var _wheel_nodes: Dictionary = {}  # WHEEL_NAME -> RigidBody3D

# ============================================================================
# SINGLETON REFERENCES
# ============================================================================
var _physics_settings: PhysicsSettings = null
var _input_manager: InputManager = null
var _game_manager: GameManager = null
var _audio_manager: AudioManager = null

# ============================================================================
# NODE REFERENCES - Populated when children are added
# ============================================================================
var _main_body: Node3D = null
var _engine_sound_player: AudioStreamPlayer3D = null
var _tire_screech_players: Dictionary = {}
var _headlight_lights: Dictionary = {}
var _taillight_lights: Dictionary = {}

# ============================================================================
# INITIALIZATION
# ============================================================================
func _ready() -> void:
	_init_singletons()
	_setup_wheels()
	_setup_audio()
	_setup_visuals()
	_reset_vehicle_state()
	_connect_signals_to_inputs()
	print("[VehicleController] Initialized successfully")

func _init_singletons() -> void:
	_physics_settings = PhysicsSettings
	_input_manager = InputManager
	_game_manager = GameManager
	_audio_manager = AudioManager
	
	if not _physics_settings:
		push_error("[VehicleController] PhysicsSettings singleton not found!")
	if not _input_manager:
		push_error("[VehicleController] InputManager singleton not found!")
	if not _game_manager:
		push_error("[VehicleController] GameManager singleton not found!")
	if not _audio_manager:
		push_error("[VehicleController] AudioManager singleton not found!")

func _setup_wheels() -> void:
	_wheel_nodes.clear()
	_wheel_steering_angles.clear()
	
	# Find wheel child nodes (should be named "FrontLeft", "FrontRight", etc.)
	var wheel_names = ["FrontLeft", "FrontRight", "RearLeft", "RearRight"]
	for wheel_name in wheel_names:
		var wheel_node = find_child(wheel_name, true, false)
		if wheel_node:
			_wheel_nodes[wheel_name] = wheel_node
			_wheel_steering_angles[wheel_name] = 0.0
		else:
			push_warning(f"[VehicleController] Wheel '{wheel_name}' not found as child")

func _setup_audio() -> void:
	# Find or create audio players
	_engine_sound_player = find_child("EngineSound", true, false)
	if not _engine_sound_player:
		_engine_sound_player = AudioStreamPlayer3D.new()
		_engine_sound_player.name = "EngineSound"
		_add_audio_player(_engine_sound_player)
	
	# Create tire screech players if not exist
	var screech_names = ["ScreechFL", "ScreechFR", "ScreechRL", "ScreechRR"]
	for name in screech_names:
		var screech = find_child(name, true, false)
		if not screech:
			screech = AudioStreamPlayer3D.new()
			screech.name = name
			_add_audio_player(screech)
		_tire_screech_players[name] = screech

func _add_audio_player(player: AudioStreamPlayer3D) -> void:
	player.bus = "SFX"
	player.autoplay = false
	player.finished.connect(_on_audio_finished.bind(player))
	add_child(player)

func _on_audio_finished(player: AudioStreamPlayer3D) -> void:
	if _audio_manager:
		_audio_manager.sound_played.emit(player.name)

func _setup_visuals() -> void:
	# Find or create light nodes
	var headlight_names = ["HeadlightLeft", "HeadlightRight"]
	for name in headlight_names:
		var light = find_child(name, true, false)
		if light:
			light.visible = headlights_enabled
			_headlight_lights[name] = light
	
	var taillight_names = ["TaillightLeft", "TaillightRight"]
	for name in taillight_names:
		var light = find_child(name, true, false)
		if light:
			light.visible = taillights_enabled
			_taillight_lights[name] = light

func _reset_vehicle_state() -> void:
	_current_gear = 0
	_target_gear = 0
	_engine_rpm = _idle_rpm
	_throttle_input = 0.0
	_brake_input = 0.0
	_handbrake_input = 0.0
	_steering_input = 0.0
	_is_engine_running = false
	_is_in_drift = false
	_drift_angle = 0.0
	_clutch_disengaged = false
	_velocity_vector = Vector3.ZERO
	_last_position = global_position

# ============================================================================
# INPUT HANDLING
# ============================================================================
func _process(delta: float) -> void:
	_process_inputs(delta)
	_update_engine(delta)
	_update_gearing(delta)
	_update_physics(delta)
	_handle_drifting(delta)
	_emit_signals()

func _process_inputs(delta: float) -> void:
	if not _input_manager:
		return
	
	# Get normalized input values from InputManager
	_throttle_input = _input_manager.get_axis("accelerate").normalized()
	_brake_input = _input_manager.get_axis("brake").normalized()
	_handbrake_input = _input_manager.get_axis("handbrake").normalized()
	_steering_input = _input_manager.get_axis("steer_left").normalized() - _input_manager.get_axis("steer_right").normalized()
	
	# Clamp inputs to valid range
	_throttle_input = clampf(_throttle_input, 0.0, 1.0)
	_brake_input = clampf(_brake_input, 0.0, 1.0)
	_handbrake_input = clampf(_handbrake_input, 0.0, 1.0)
	_steering_input = clampf(_steering_input, -1.0, 1.0)
	
	# Handle gear shift inputs
	_handle_gear_shift_input()
	
	# Engine start/stop
	if _input_manager.is_action_pressed("start_engine") and not _is_engine_running:
		start_engine()
	elif _input_manager.is_action_pressed("stop_engine") and _is_engine_running:
		stop_engine()

func _handle_gear_shift_input() -> void:
	if not _input_manager:
		return
	
	# Manual gear shifting
	if _input_manager.is_action_just_pressed("shift_up"):
		_request_gear_shift(_current_gear + 1)
	elif _input_manager.is_action_just_pressed("shift_down"):
		_request_gear_shift(_current_gear - 1)
	
	# Auto-shift trigger
	if _auto_shift_enabled and _input_manager.is_action_just_pressed("auto_shift"):
		_auto_shift_gear()

func _request_gear_shift(target: int) -> void:
	if _clutch_disengaged:
		_target_gear = target
		_apply_gear_change()
	else:
		# Disengage clutch first
		_clutch_disengaged = true
		_target_gear = target

# ============================================================================
# ENGINE MANAGEMENT
# ============================================================================
func _update_engine(delta: float) -> void:
	if not _is_engine_running:
		_engine_rpm = lerp(_engine_rpm, _idle_rpm, delta * 5.0)
		return
	
	var current_ratio = _get_current_gear_ratio()
	var wheel_speed = _calculate_wheel_rpm()
	var theoretical_rpm = wheel_speed * current_ratio * _final_drive_ratio
	
	# Calculate engine torque based on RPM and throttle
	_current_torque = _calculate_engine_torque()
	
	# Apply torque to increase RPM
	var torque_effect = _current_torque * (1.0 / vehicle_mass) * delta * 0.01
	_engine_rpm += torque_effect
	
	# Natural RPM decay when not accelerating
	if _throttle_input < 0.1:
		_engine_rpm -= delta * 20.0
	
	# Clamp RPM
	_engine_rpm = clampf(_engine_rpm, _idle_rpm, _redline_rpm * 1.1)
	
	# Engine braking effect
	if _brake_input > 0.5 and _throttle_input < 0.1:
		_engine_rpm -= delta * 15.0
	
	# Prevent over-revving
	if _engine_rpm > _redline_rpm * 1.1:
		_engine_rpm = _redline_rpm * 1.1
		_engine_rpm -= delta * 500.0

func _calculate_engine_torque() -> float:
	if _engine_rpm <= 0:
		return 0.0
	
	# Normalize RPM to 0-1 range
	var normalized_rpm = (_engine_rpm - _idle_rpm) / (_redline_rpm - _idle_rpm)
	normalized_rpm = clampf(normalized_rpm, 0.0, 1.0)
	
	# Interpolate torque from curve
	var torque_multiplier = _interpolate_torque_curve(normalized_rpm)
	
	# Apply throttle influence
	var base_torque = 450.0  # Nm peak torque
	var actual_torque = base_torque * torque_multiplier * _throttle_input
	
	# Reduce torque if clutch is disengaged
	if _clutch_disengaged:
		actual_torque *= 0.1
	
	return actual_torque

func _interpolate_torque_curve(normalized_rpm: float) -> float:
	if not _torque_curve or _torque_curve.size() < 2:
		return _throttle_input
	
	var last_point = _torque_curve[0]
	for i in range(1, _torque_curve.size()):
		var point = _torque_curve[i]
		if normalized_rpm <= point.x:
			# Linear interpolation between last_point and point
			if point.x == last_point.x:
				return last_point.y
			var t = (normalized_rpm - last_point.x) / (point.x - last_point.x)
			return lerp(last_point.y, point.y, t)
		last_point = point
	
	return last_point.y

func _calculate_wheel_rpm() -> float:
	if _velocity_vector.length() <= 0.001:
		return 0.0
	
	var speed_ms = _velocity_vector.length()
	var wheel_circumference = 2.0 * PI * _tire_radius
	return (speed_ms / wheel_circumference) * 60.0  # Convert to RPM

func _get_current_gear_ratio() -> float:
	if _current_gear == 0:
		return 0.0
	elif _current_gear < 0:
		return _reverse_ratio
	else:
		if _current_gear >= gear_ratios.size():
			return gear_ratios.back()
		return gear_ratios[_current_gear - 1]

# ============================================================================
# GEARING SYSTEM
# ============================================================================
func _update_gearing(delta: float) -> void:
	_gear_shift_timer += delta
	
	# Handle clutch re-engagement
	if _clutch_disengaged and _gear_shift_timer > _max_gear_shift_time:
		_clutch_disengaged = false
		_apply_gear_change()
	
	# Auto-shift logic
	if _auto_shift_enabled and not _clutch_disengaged:
		_auto_shift_logic(delta)

func _auto_shift_logic(delta: float) -> void:
	if _current_gear == 0:
		return
	
	# Upshift if hitting redline
	if _engine_rpm >= _redline_rpm * 0.95 and _current_gear < num_gears:
		_request_gear_shift(_current_gear + 1)
	
	# Downshift if stalling risk
	elif _engine_rpm <= _idle_rpm * 1.2 and _current_gear > 1:
		_request_gear_shift(_current_gear - 1)

func _request_gear_shift(target: int) -> void:
	if target < -1 or target > num_gears:
		return
	
	if _clutch_disengaged:
		_target_gear = target
	else:
		_clutch_disengaged = true
		_target_gear = target
		_gear_shift_timer = 0.0

func _apply_gear_change() -> void:
	var old_gear = _current_gear
	_current_gear = _target_gear
	
	if old_gear != _current_gear:
		gear_changed.emit(old_gear, _current_gear)
		
		if _audio_manager:
			_audio_manager.play_sound("gear_shift")
		
		# Play gear sound based on direction
		if _current_gear > old_gear:
			_audio_manager.play_sound("upshift")
		elif _current_gear < old_gear:
			_audio_manager.play_sound("downshift")
	
	# Ensure minimum RPM after shift
	if _current_gear != 0 and _engine_rpm < _idle_rpm * 1.5:
		_engine_rpm = _idle_rpm * 1.5

func _auto_shift_gear() -> void:
	if _current_gear == num_gears:
		_request_gear_shift(-num_gears)  # Shift to neutral then down
	else:
		_request_gear_shift(_current_gear + 1)

# ============================================================================
# PHYSICS & MOVEMENT
# ============================================================================
func _update_physics(delta: float) -> void:
	# Calculate drag force
	var drag_force = _calculate_drag_force()
	
	# Calculate drive force based on drivetrain
	var drive_force = _calculate_drive_force()
	
	# Calculate braking force
	var brake_force = _calculate_brake_force()
	
	# Combine forces
	var total_force = drive_force - drag_force - brake_force
	
	# Apply to velocity
	var acceleration = total_force / vehicle_mass
	_velocity_vector += acceleration * delta
	
	# Apply friction/drag
	_velocity_vector *= 0.995  # Simple air resistance
	
	# Move vehicle
	move_and_slide(_velocity_vector, Vector3.UP)
	
	# Update position tracking
	_last_position = global_position

func _calculate_drag_force() -> float:
	var speed_ms = _velocity_vector.length()
	var drag = 0.5 * _drag_coefficient * _frontal_area * _air_density * speed_ms * speed_ms
	return drag

func _calculate_drive_force() -> float:
	if _current_gear == 0 or _clutch_disengaged:
		return 0.0
	
	var wheel_rpm = _calculate_wheel_rpm()
	var gear_ratio = _get_current_gear_ratio()
	var final_ratio = gear_ratio * _final_drive_ratio
	
	# Calculate torque at wheels
	var wheel_torque = _current_torque * final_ratio
	
	# Convert to force (torque / radius)
	var drive_force = wheel_torque / _tire_radius
	
	# Limit based on traction
	var max_traction = vehicle_mass * 9.81 * _grip_coefficient
	drive_force = min(drive_force, max_traction)
	
	# Drivetrain-specific adjustments
	match drivetrain_type:
		DrivetrainType.FWD:
			drive_force *= 0.6  # FWD typically less efficient
		DrivetrainType.RWD:
			drive_force *= 0.65
		DrivetrainType.AWD:
			drive_force *= 1.0  # AWD most efficient
	
	return drive_force

func _calculate_brake_force() -> float:
	var base_brake = _brake_input * _braking_force
	
	# Handbrake adds additional rear brake bias
	if _handbrake_input > 0:
		base_brake += _handbrake_input * _braking_force * 0.5
		handbrake_toggled.emit(_handbrake_input > 0)
	
	return base_brake

# ============================================================================
# DRIFT MECHANICS
# ============================================================================
func _handle_drifting(delta: float) -> void:
	var lateral_velocity = _velocity_vector.xz.length()
	var longitudinal_velocity = _velocity_vector.z
	
	if lateral_velocity > 2.0 and abs(_steering_input) > 0.5:
		_is_in_drift = true
		_drift_angle = atan2(_velocity_vector.x, _velocity_vector.z)
		
		if not _is_in_drift:
			# First frame of drift
			_is_in_drift = true
			_drift_angle = 0.0
			drift_started.emit(0.0)
			
			if _audio_manager:
				_audio_manager.play_sound("drift_start")
	
	if _is_in_drift:
		# Gradually reduce drift angle
		_drift_angle = lerp(_drift_angle, 0.0, delta * 2.0)
		
		# Increase tire screech volume based on drift intensity
		if _audio_manager and _tire_screech_players:
			var screech_vol = min(1.0, abs(_drift_angle) * 2.0)
			for player in _tire_screech_players.values():
				player.volume_db = lerp(player.volume_db, db_from_gain(screech_vol), delta * 5.0)
		
		# Exit drift condition
		if lateral_velocity < 1.0 or abs(_steering_input) < 0.2:
			_exit_drift()

func _exit_drift() -> void:
	if _is_in_drift:
		_is_in_drift = false
		_drift_angle = 0.0
		
		if _audio_manager:
			_audio_manager.play_sound("drift_end")
		
		drift_ended.emit()

# ============================================================================
# ENGINE CONTROL
# ============================================================================
func start_engine() -> void:
	if _is_engine_running:
		return
	
	_is_engine_running = true
	_engine_rpm = _idle_rpm
	
	engine_started.emit()
	
	if _audio_manager:
		_audio_manager.play_sound("engine_start")
	
	# Start engine sound loop
	if _engine_sound_player:
		_engine_sound_player.stream = _get_engine_stream()
		_engine_sound_player.play()

func stop_engine() -> void:
	if not _is_engine_running:
		return
	
	_is_engine_running = false
	_engine_rpm = _idle_rpm
	
	engine_stopped.emit()
	
	if _audio_manager:
		_audio_manager.play_sound("engine_stop")
	
	# Stop engine sound
	if _engine_sound_player:
		_engine_sound_player.stop()

func _get_engine_stream() -> AudioStream:
	# This would return an appropriate engine sound stream
	# For now, we'll use a placeholder approach via AudioManager
	if _audio_manager:
		return _audio_manager.get_engine_stream()
	return null

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================
func get_speed_kmh() -> float:
	return _velocity_vector.length() * 3.6  # Convert m/s to km/h

func get_rpm() -> float:
	return _engine_rpm

func get_current_gear() -> int:
	return _current_gear

func get_total_distance() -> float:
	return _last_position.distance_to(Vector3.ZERO)

func reset_vehicle() -> void:
	_reset_vehicle_state()
	global_position = Vector3.ZERO
	_velocity_vector = Vector3.ZERO
	move_and_slide(_velocity_vector, Vector3.UP)

func set_drivetrain(type: DrivetrainType) -> void:
	drivetrain_type = type

func set_gear_ratios(ratios: Array[float]) -> void:
	if ratios.size() >= num_gears:
		gear_ratios = ratios

func enable_auto_shift(enable: bool) -> void:
	_auto_shift_enabled = enable

func _emit_signals() -> void:
	# Emit movement signal
	var distance_traveled = global_position.distance_to(_last_position)
	if distance_traveled > 0.1:
		vehicle_moved.emit(global_position, _velocity_vector)
		_last_position = global_position
	
	# Emit speed signal
	var current_speed = get_speed_kmh()
	if abs(current_speed - speed_changed.last_value) > 0.5:
		speed_changed.emit(current_speed)
	
	# Emit RPM signal
	if abs(_engine_rpm - rpm_changed.last_value) > 50:
		rpm_changed.emit(_engine_rpm)

# ============================================================================
# PROPERTY SETTERS WITH VALIDATION
# ============================================================================
func _set_vehicle_mass(value: float) -> void:
	vehicle_mass = value
	# Recalculate physics parameters
	vehicle_mass = value

func _set_max_speed_kmh(value: float) -> void:
	max_speed_kmh = value

func _set_acceleration_force(value: float) -> void:
	acceleration_force = value

func _set_braking_force(value: float) -> void:
	braking_force = value

func _set_steering_angle_max(value: float) -> void:
	steering_angle_max = value

func _set_final_drive_ratio(value: float) -> void:
	final_drive_ratio = value

func _set_tire_radius(value: float) -> void:
	tire_radius = value

func _set_torque_curve(value: Array[Vector2f]) -> void:
	torque_curve = value

func _set_num_gears(value: int) -> void:
	num_gears = max(4, min(9, value))

func _set_gear_ratios(value: Array[float]) -> void:
	gear_ratios = value

func _set_reverse_ratio(value: float) -> void:
	reverse_ratio = value

func _set_idle_rpm(value: float) -> void:
	idle_rpm = max(500, value)

func _set_redline_rpm(value: float) -> void:
	redline_rpm = max(idle_rpm * 2, value)

func _set_steering_sensitivity(value: float) -> void:
	steering_sensitivity = clampf(value, 0.1, 3.0)

func _set_steering_return_speed(value: float) -> void:
	steering_return_speed = value

func _set_grip_coefficient(value: float) -> void:
	grip_coefficient = value

func _set_drift_threshold(value: float) -> void:
.drift_threshold = value

# ============================================================================
# HELPER FUNCTIONS
# ============================================================================
func db_from_gain(gain: float) -> float:
	return 20.0 * log10(gain) if gain > 0 else -80.0

func world_to_local(world_pos: Vector3) -> Vector3:
	return global_transform.basis.inverse() * (world_pos - global_position)

func local_to_world(local_pos: Vector3) -> Vector3:
	return global_position + global_transform.basis * local_pos

# ============================================================================
# DEBUG VISUALIZATION
# ============================================================================
func _draw_debug() -> void:
	if not _game_manager or not _game_manager.debug_mode:
		return
	
	# Draw velocity vector
	var color = Color.GREEN
	draw_line(global_position, global_position + _velocity_vector * 2.0, color, 3.0)
	
	# Draw steering angles
	for wheel_name in _wheel_steering_angles:
		var angle = _wheel_steering_angles[wheel_name]
		var color = Color.YELLOW
		draw_circle(Vector3.ZERO, abs(angle) * 10.0, color, false)

# ============================================================================
# DESTRUCTOR
# ============================================================================
func _exit_tree() -> void:
	# Cleanup audio players
	if _engine_sound_player:
		_engine_sound_player.queue_free()
	
	for player in _tire_screech_players.values():
		player.queue_free()
	
	print("[VehicleController] Cleaned up successfully")
</FILE "scripts/controllers/VehicleController.gd">>