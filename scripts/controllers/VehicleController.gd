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

# ============================================================================
# CONFIGURATION CONSTANTS
# ============================================================================
const MAX_SPEED_KMH: float = 350.0
const ACCELERATION_POWER: float = 20000.0
const BRAKING_FORCE: float = 40000.0
const STEERING_SPEED: float = 2.5
const MAX_STEERING_ANGLE: float = 35.0 * TAU / 180.0
const MIN_GEAR: int = -1  # Reverse
const MAX_GEAR: int = 6
const NEUTRAL_GEAR: int = 0
const IDLE_RPM: float = 800.0
const REDLINE_RPM: float = 8000.0
const CLUTCH_RELEASE_TIME: float = 0.3
const DIFFERENTIAL_LOCK_RATIO: float = 0.8
const DRIFT_FACTOR: float = 0.15

# ============================================================================
# STATE VARIABLES
# ============================================================================
var _speed_kmh: float = 0.0
var _rpm: float = IDLE_RPM
var _current_gear: int = NEUTRAL_GEAR
var _throttle_input: float = 0.0
var _brake_input: float = 0.0
var _steering_input: float = 0.0
var _handbrake_input: float = 0.0

var _is_engine_running: bool = false
var _is_clutch_engaged: bool = true
var _is_handbrake_active: bool = false
var _is_skidding: bool = false
var _is_drifting: bool = false

var _gear_ratios: Array[float] = [0.0, 3.5, 2.5, 1.8, 1.4, 1.1, 0.9, 0.7]
var _final_drive_ratio: float = 3.5
var _wheel_base: float = 2.7
var _track_width: float = 1.6
var _wheel_radius: float = 0.33

var _last_position: Vector3 = Vector3.ZERO
var _velocity_direction: Vector3 = Vector3.ZERO

# References
var _rigid_body: RigidBody3D
var _engine_node: Node
var _suspension_nodes: Array[Node3D] = []

# ============================================================================
# INITIALIZATION
# ============================================================================
func _ready() -> void:
	_init_rigid_body()
	_init_vehicle_components()
	_connect_signals_to_game_manager()
	_load_configuration()
	_set_initial_state()

func _physics_process(delta: float) -> void:
	if not _is_ready():
		return
	
	var game_settings: PhysicsSettings = get_physics_settings()
	if not game_settings:
		return
	
	_update_physics(game_settings, delta)
	_update_visual_feedback()

func _input(event: InputEvent) -> void:
	if not _is_ready():
		return
	
	_handle_input_event(event)

func _on_collision(body: Node3D, collision_shape: CollisionShape3D, normal: Vector3, position: Vector3, motion: Vector3) -> void:
	var collision_data: Dictionary = {
		"body": body.get_path(),
		"position": position,
		"normal": normal,
		"impact_velocity": motion.length(),
		"time_stamp": Time.get_ticks_msec()
	}
	
	collision_detected.emit(collision_data)
	
	_handle_collision_impact(collision_data)

# ============================================================================
# PHYSICS UPDATE CORE
# ============================================================================
func _update_physics(settings: PhysicsSettings, delta: float) -> void:
	_apply_inputs(settings, delta)
	_calculate_wheel_forces(settings, delta)
	_update_vehicle_dynamics(settings, delta)
	_update_transmission_logic(settings, delta)
	_check_conditions(settings)

func _apply_inputs(settings: PhysicsSettings, delta: float) -> void:
	# Read input values from InputManager singleton
	var input_values: Dictionary = InputManager.get_vehicle_inputs()
	
	_throttle_input = clampf(input_values.get("throttle", 0.0), 0.0, 1.0)
	_brake_input = clampf(input_values.get("brake", 0.0), 0.0, 1.0)
	_steering_input = clampf(input_values.get("steering", 0.0), -1.0, 1.0)
	_handbrake_input = clampf(input_values.get("handbrake", 0.0), 0.0, 1.0)
	
	# Apply input smoothing for realistic feel
	_throttle_input = _lerp_inputs(_throttle_input, _get_smoothed_throttle(), settings.time_scale)
	_brake_input = _lerp_inputs(_brake_input, _get_smoothed_brake(), settings.time_scale)
	_steering_input = _lerp_inputs(_steering_input, _get_smoothed_steering(), settings.time_scale)
	_handbrake_input = _lerp_inputs(_handbrake_input, _get_smoothed_handbrake(), settings.time_scale)
	
	# Emit signals for input changes
	if abs(_throttle_input - _get_smoothed_throttle()) > 0.01:
		throttle_applied.emit(_throttle_input)
	if abs(_brake_input - _get_smoothed_brake()) > 0.01:
		brake_applied.emit(_brake_input)
	if abs(_steering_input - _get_smoothed_steering()) > 0.01:
		steering_angle_changed.emit(_steering_input)

func _calculate_wheel_forces(settings: PhysicsSettings, delta: float) -> void:
	if not _is_engine_running or _current_gear == NEUTRAL_GEAR:
		return
	
	# Calculate effective power based on gear and RPM
	var effective_power: float = _get_effective_power()
	
	# Apply power to rear wheels (RWD configuration - configurable)
	var drive_wheels: Array[int] = [1, 2]  # Rear wheels indices
	
	for wheel_idx in drive_wheels:
		var wheel_force: float = effective_power * settings.time_scale
		
		# Reduce force at low RPM (clutch slip simulation)
		if not _is_clutch_engaged:
			wheel_force *= (_rpm - IDLE_RPM) / (REDLINE_RPM - IDLE_RPM)
		
		# Apply braking force if brake is engaged
		if _brake_input > 0.1:
			wheel_force -= BRAKING_FORCE * _brake_input
		
		# Apply torque vector to wheel node
		_apply_wheel_torque(wheel_idx, wheel_force, delta)

func _update_vehicle_dynamics(settings: PhysicsSettings, delta: float) -> void:
	# Update velocity direction based on steering
	var forward: Vector3 = transform.basis.z
	var right: Vector3 = transform.basis.x
	
	# Calculate target rotation based on steering input
	var steer_amount: float = _steering_input * MAX_STEERING_ANGLE * settings.time_scale
	
	# Smooth steering transition
	var target_rotation: float = _calculate_target_rotation(steer_amount)
	_rigidbody.rotation.y = _smooth_rotation(_rigid_body.rotation.y, target_rotation, STEERING_SPEED * settings.time_scale)
	
	# Calculate speed from velocity magnitude
	var velocity: Vector3 = _rigid_body.linear_velocity
	_speed_kmh = velocity.length() * 3.6  # Convert m/s to km/h
	
	# Update RPM based on speed and gear
	_update_rpm_from_speed()
	
	# Handle skidding detection
	_check_skidding_condition()

func _update_transmission_logic(settings: PhysicsSettings, delta: float) -> void:
	# Automatic gear shifting logic
	if _current_gear != NEUTRAL_GEAR and _is_engine_running:
		_auto_shift_gears(settings, delta)
	
	# Manual gear shift override via input
	if Input.is_action_just_pressed("gear_up"):
		_shift_gear_manual(1)
	elif Input.is_action_just_pressed("gear_down"):
		_shift_gear_manual(-1)
	
	# Clutch control
	if Input.is_action_just_pressed("clutch"):
		_toggle_clutch()

func _check_conditions(settings: PhysicsSettings) -> void:
	# Check for engine stall condition
	if _rpm < IDLE_RPM * 0.5 and _throttle_input > 0.1:
		_engine_stall(settings)
	
	# Check for over-revving
	if _rpm > REDLINE_RPM:
		_limit_rpm(settings)
	
	# Check for extreme speeds
	if _speed_kmh > MAX_SPEED_KMH * 1.1:
		_aerodynamic_drag(settings)

# ============================================================================
# INPUT HANDLING
# ============================================================================
func _handle_input_event(event: InputEvent) -> void:
	if event is InputEventKey:
		# Keyboard input handling
		if event.pressed:
			match event.key_code:
				KEY_W, KEY_UP:
					_throttle_input = 1.0
				KEY_S, KEY_DOWN:
					_brake_input = 1.0
				KEY_A, KEY_LEFT:
					_steering_input = -1.0
				KEY_D, KEY_RIGHT:
					_steering_input = 1.0
				KEY_SPACE:
					_handbrake_input = 1.0
				KEY_1, KEY_2, KEY_3, KEY_4, KEY_5, KEY_6:
					var gear_num: int = event.key_code - KEY_1
					_shift_gear_manual(gear_num - NEUTRAL_GEAR)
	
	elif event is InputEventMouseButton:
		# Mouse input for steering (optional)
		pass

func _get_smoothed_throttle() -> float:
	return _smooth_input_value(_throttle_input)

func _get_smoothed_brake() -> float:
	return _smooth_input_value(_brake_input)

func _get_smoothed_steering() -> float:
	return _smooth_input_value(_steering_input)

func _get_smoothed_handbrake() -> float:
	return _smooth_input_value(_handbrake_input)

func _smooth_input_value(current: float) -> float:
	var smoothing_factor: float = 0.1
	return lerp(_last_input_value, current, smoothing_factor)

var _last_input_value: float = 0.0

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================
func _shift_gear_manual(direction: int) -> void:
	var old_gear: int = _current_gear
	var new_gear: int = _clamp_gear(_current_gear + direction)
	
	if new_gear != old_gress:
		_current_gear = new_gear
		gear_changed.emit(old_gear, new_gear)
		
		# Sound effect for gear change
		if AudioManager:
			AudioManager.play_sound("gear_change")

func _auto_shift_gears(settings: PhysicsSettings, delta: float) -> void:
	# Shift up when approaching redline
	if _rpm >= REDLINE_RPM * 0.95 and _current_gear < MAX_GEAR:
		_shift_gear_manual(1)
	
	# Shift down when RPM drops too low
	if _rpm <= IDLE_RPM * 1.2 and _current_gear > MIN_GEAR and _current_gear != NEUTRAL_GEAR:
		_shift_gear_manual(-1)
	
	# Downshift when braking
	if _brake_input > 0.5 and _current_gear > MIN_GEAR:
		_shift_gear_manual(-1)

func _clamp_gear(gear: int) -> int:
	return clampi(gear, MIN_GEAR, MAX_GEAR)

func _get_effective_power() -> float:
	# Calculate power curve based on RPM
	var normalized_rpm: float = (_rpm - IDLE_RPM) / (REDLINE_RPM - IDLE_RPM)
	
	# Power curve approximation (peak around 60% of redline)
	var power_curve: float = -normalized_rpm * normalized_rpm + 1.2 * normalized_rpm
	
	# Apply gear ratio effect
	var gear_ratio: float = _gear_ratios[_current_gear] if _current_gear >= 0 else _gear_ratios[MAX_GEAR]
	
	return power_curve * ACCELERATION_POWER * gear_ratio

# ============================================================================
# WHEEL AND SUSPENSION SYSTEM
# ============================================================================
func _init_suspension() -> void:
	_suspension_nodes.clear()
	
	# Create suspension nodes for each wheel
	var wheel_positions: Array[Vector3] = [
		Vector3(-_wheel_base/2, 0, -_track_width/2),   # Front left
		Vector3(-_wheel_base/2, 0, _track_width/2),    # Front right
		Vector3(_wheel_base/2, 0, -_track_width/2),    # Rear left
		Vector3(_wheel_base/2, 0, _track_width/2)      # Rear right
	]
	
	for pos in wheel_positions:
		var wheel_node: Node3D = Node3D.new()
		wheel_node.position = pos
		add_child(wheel_node)
		_suspension_nodes.append(wheel_node)

func _apply_wheel_torque(wheel_index: int, torque: float, delta: float) -> void:
	if wheel_index >= _suspension_nodes.size():
		return
	
	var wheel_node: Node3D = _suspension_nodes[wheel_index]
	var wheel_force: Vector3 = -transform.basis.z * torque
	
	# Add differential effect
	if wheel_index % 2 == 1:  # Right wheels
		wheel_force *= DIFFERENTIAL_LOCK_RATIO
	
	# Apply force with damping
	var damping_factor: float = 0.9
	_wheel_torques[wheel_index] = _wheel_torques[wheel_index] * damping_factor + wheel_force * delta

var _wheel_torques: Array[Vector3] = [Vector3.ZERO, Vector3.ZERO, Vector3.ZERO, Vector3.ZERO]

# ============================================================================
# VEHICLE DYNAMICS HELPERS
# ============================================================================
func _calculate_target_rotation(steer_amount: float) -> float:
	var current_rotation: float = _rigid_body.rotation.y
	var target: float = current_rotation + steer_amount
	
	# Normalize rotation to [-PI, PI]
	while target > TAU / 2:
		target -= TAU
	while target < -TAU / 2:
		target += TAU
	
	return target

func _smooth_rotation(current: float, target: float, speed: float) -> float:
	var diff: float = target - current
	
	# Normalize difference
	while diff > TAU / 2:
		diff -= TAU
	while diff < -TAU / 2:
		diff += TAU
	
	# Clamp movement to max speed
	diff = clampf(diff, -speed, speed)
	
	return current + diff

func _update_rpm_from_speed() -> void:
	if _current_gear == NEUTRAL_GEAR:
		_rpm = IDLE_RPM
		return
	
	var gear_ratio: float = _gear_ratios[_current_gear]
	var effective_ratio: float = gear_ratio * _final_drive_ratio
	
	# RPM = (Speed / Wheel Radius) * Gear Ratio * Conversion Factor
	var wheel_rps: float = _speed_kmh / 3.6 / (_wheel_radius * 2 * TAU)
	_rpm = wheel_rps * effective_ratio * 60.0
	
	# Clamp to idle when stationary
	if _speed_kmh < 1.0 and _current_gear != MIN_GEAR:
		_rpm = IDLE_RPM

func _check_skidding_condition() -> void:
	var velocity: Vector3 = _rigid_body.linear_velocity
	var forward: Vector3 = transform.basis.z
	
	# Calculate lateral velocity component
	var lateral_velocity: float = abs(velocity.dot(transform.basis.x))
	
	# Threshold for skidding (depends on speed and surface)
	var skid_threshold: float = MAX_SPEED_KMH * 0.15 / 3.6
	
	_is_skidding = lateral_velocity > skid_threshold
	
	if _is_skidding != skidding.is_skidding:
		skidding.emit(_is_skidding)
	
	# Drift mode when handbrake is active and skidding
	_is_drifting = _is_skidding and _handbrake_input > 0.5

func _handle_collision_impact(collision_info: Dictionary) -> void:
	var impact_velocity: float = collision_info.get("impact_velocity", 0.0)
	
	# Screen shake based on impact
	if impact_velocity > 5.0:
		_trigger_screen_shake(impact_velocity * 0.01)
	
	# Sound effects for different impact levels
	if impact_velocity > 10.0:
		if AudioManager:
			AudioManager.play_sound("crash_hard")
	elif impact_velocity > 5.0:
		if AudioManager:
			AudioManager.play_sound("crash_medium")
	else:
		if AudioManager:
			AudioManager.play_sound("crash_light")

func _trigger_screen_shake(intensity: float) -> void:
	# Implement screen shake effect
	# This would typically affect camera or render offset
	pass

# ============================================================================
# ENGINE MANAGEMENT
# ============================================================================
func _engine_stall(settings: PhysicsSettings) -> void:
	_rpm = 0.0
	_is_engine_running = false
	
	if AudioManager:
		AudioManager.play_sound("engine_stall")
	
	engine_stalled.emit()

func _limit_rpm(settings: PhysicsSettings) -> void:
	_rpm = REDLINE_RPM
	
	# Cut fuel temporarily
	if AudioManager:
		AudioManager.play_sound("fuel_cut")

func start_engine() -> void:
	_is_engine_running = true
	_rpm = IDLE_RPM
	
	if AudioManager:
		AudioManager.play_sound("engine_start")

func stop_engine() -> void:
	_is_engine_running = false
	_rpm = 0.0
	
	if AudioManager:
		AudioManager.play_sound("engine_stop")

func _toggle_clutch() -> void:
	_is_clutch_engaged = !_is_clutch_engaged
	
	if _is_clutch_engaged and _rpm > IDLE_RPM:
		# Smooth clutch engagement
		await get_tree().create_timer(CLUTCH_RELEASE_TIME).timeout
		_is_clutch_engaged = true

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================
func _set_final_drive_ratio(value: float) -> void:
	_final_drive_ratio = value

func _set_wheel_radius(value: float) -> float:
	_wheel_radius = value

func _set_gravity(value: float) -> void:
	PhysicsSettings.gravity = value

func _set_physics_tick_rate(value: int) -> void:
	PhysicsSettings.physics_tick_rate = value

func _set_max_substeps(value: int) -> void:
	PhysicsSettings.max_substeps = value

func _set_time_scale(value: float) -> void:
	PhysicsSettings.time_scale = value

func _set_default_vehicle_mass(value: float) -> void:
	if _rigid_body:
		_rigid_body.mass = value

func _set_default_wheelforce(value: float) -> void:
	ACCELERATION_POWER = value

# ============================================================================
# GETTERS AND SETTERS
# ============================================================================
func get_speed_kmh() -> float:
	return _speed_kmh

func get_rpm() -> float:
	return _rpm

func get_current_gear() -> int:
	return _current_gear

func get_throttle_input() -> float:
	return _throttle_input

func get_brake_input() -> float:
	return _brake_input

func get_steering_input() -> float:
	return _steering_input

func is_engine_running() -> bool:
	return _is_engine_running

func is_skidding() -> bool:
	return _is_skidding

func is_drifting() -> bool:
	return _is_drifting

func get_gear_ratio() -> float:
	return _gear_ratios[_current_gear] if _current_gear >= 0 else _gear_ratios[MAX_GEAR]

func get_total_drive_ratio() -> float:
	return get_gear_ratio() * _final_drive_ratio

# ============================================================================
# RESET AND CLEANUP
# ============================================================================
func reset_vehicle() -> void:
	_speed_kmh = 0.0
	_rpm = IDLE_RPM
	_current_gear = NEUTRAL_GEAR
	_throttle_input = 0.0
	_brake_input = 0.0
	_steering_input = 0.0
	_handbrake_input = 0.0
	_is_engine_running = false
	_is_clutch_engaged = true
	_is_handbrake_active = false
	_is_skidding = false
	_is_drifting = false
	
	_rigidbody.linear_velocity = Vector3.ZERO
	_rigidbody.angular_velocity = Vector3.ZERO

func _init_rigid_body() -> void:
	_rigid_body = get_parent()
	if _rigid_body is RigidBody3D:
		_rigid_body.mass = PhysicsSettings.default_vehicle_mass
		_rigid_body.collision_layer = 1 << 2  # Vehicle layer
		_rigid_body.collision_mask = 1 << 3 | 1 << 4  # Track and obstacles

func _init_vehicle_components() -> void:
	_init_suspension()
	_init_wheel_nodes()

func _init_wheel_nodes() -> void:
	# Initialize visual wheel nodes
	pass

func _connect_signals_to_game_manager() -> void:
	GameManager.game_state_changed.connect(_on_game_state_changed)
	GameManager.race_started.connect(_on_race_started)
	GameManager.race_ended.connect(_on_race_ended)

func _load_configuration() -> void:
	# Load vehicle-specific configuration
	# This could come from a config file or scene properties
	pass

func _set_initial_state() -> void:
	start_engine()
	reset_vehicle()

# ============================================================================
# GAME MANAGER EVENT HANDLERS
# ============================================================================
func _on_game_state_changed(new_state: GameManager.GameState) -> void:
	match new_state:
		GameManager.GameState.RACE_ACTIVE:
			start_engine()
		GameManager.GameState.RACE_PAUSED:
			stop_engine()
		GameManager.GameState.MAIN_MENU:
			reset_vehicle()
		_:
			pass

func _on_race_started(race_data: Dictionary) -> void:
	print("VehicleController: Race started")
	start_engine()

func _on_race_ended(results: Dictionary) -> void:
	print("VehicleController: Race ended")
	stop_engine()
	reset_vehicle()

# ============================================================================
# VISUAL FEEDBACK
# ============================================================================
func _update_visual_feedback() -> void:
	# Update wheel rotation visuals
	for i in _suspension_nodes.size():
		var wheel_node: Node3D = _suspension_nodes[i]
		wheel_node.rotation.x = -_speed_kmh * 0.1 * 0.01745  # Convert to radians

func update_physics_settings(settings: PhysicsSettings) -> void:
	if settings:
		mass = settings.default_vehicle_mass
		max_power = settings.max_power
		max_torque = settings.max_torque

func get_physics_settings() -> PhysicsSettings:
	return PhysicsSettings

func _is_ready() -> bool:
	return _rigid_body != null and is_instance_valid(_rigid_body)

</file_content>