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
const SHIFT_SHOCK_TIME: float = 0.1
const DRIFT_THRESHOLD: float = 0.3
const TRACTION_CONTROL_ACTIVE: bool = true
const ABS_ACTIVE: bool = true
const DIFFERENTIAL_LOCK: float = 0.7

# ============================================================================
# MEMBER VARIABLES - State Tracking
# ============================================================================
# Current state
var current_speed: float = 0.0  # km/h
var current_rpm: float = IDLE_RPM
var current_gear: int = NEUTRAL_GEAR
var target_gear: int = NEUTRAL_GEAR
var clutch_position: float = 1.0  # 0 = pressed, 1 = released
var clutch_target: float = 1.0
var is_clutch_pressed: bool = false
var is_throttle_pressed: bool = false
var is_brake_pressed: bool = false
var is_handbrake_pressed: bool = false
var steering_input: float = 0.0  # -1 to 1
var actual_steering_angle: float = 0.0
var drift_angle: float = 0.0
var is_drifting: bool = false
var is_traction_control_active: bool = TRACTION_CONTROL_ACTIVE
var is_abs_active: bool = ABS_ACTIVE
var is_engine_running: bool = true
var is_stalled: bool = false
var is_reversing: bool = false

# ============================================================================
# MEMBER VARIABLES - Physics References
# ============================================================================
var _vehicle_body: RigidBody3D = null
var _powertrain_node: Node = null
var _wheel_nodes: Array[Node3D] = []
var _suspension_nodes: Array[Node3D] = []
var _collider_areas: Array[Area3D] = []

# ============================================================================
# MEMBER VARIABLES - Simulation State
# ============================================================================
var _last_time: float = 0.0
var _acceleration: Vector3 = Vector3.ZERO
var _torque_output: float = 0.0
var _brake_force: float = 0.0
var _engine_braking: float = 0.0
var _gear_ratios: Dictionary = {
	-1: -0.4,  # Reverse
	1: 3.5,    # 1st gear
	2: 2.5,    # 2nd gear
	3: 1.8,    # 3rd gear
	4: 1.4,    # 4th gear
	5: 1.1,    # 5th gear
	6: 0.9     # 6th gear
}
var _final_drive_ratio: float = 3.73
var _tire_radius: float = 0.32  # meters
var _max_wheel_slip: float = 0.3
var _traction_loss_factor: float = 0.0
var _drift_multiplier: float = 1.0
var _collision_damage: float = 0.0
var _damage_threshold: float = 50.0

# ============================================================================
# MEMBER VARIABLES - Timer & Cooldowns
# ============================================================================
var _shift_cooldown: float = 0.0
var _engine_start_delay: float = 1.5
var _cooldown_timer: float = 0.0
var _skid_timer: float = 0.0
var _damage_timer: float = 0.0
var _clutch_timer: float = 0.0

# ============================================================================
# MEMBER VARIABLES - Audio References
# ============================================================================
var _engine_audio_player: AudioStreamPlayer3D = null
var _tire_audio_player: AudioStreamPlayer3D = null
var _transmission_audio_player: AudioStreamPlayer3D = null

# ============================================================================
# PUBLIC ACCESSORS
# ============================================================================
func get_current_speed_kmh() -> float: return current_speed
func get_current_speed_ms() -> float: return current_speed / 3.6
func get_current_rpm() -> float: return current_rpm
func get_current_gear() -> int: return current_gear
func is_engine_on() -> bool: return is_engine_running && !is_stalled
func is_in_neutral() -> bool: return current_gear == NEUTRAL_GEAR
func is_reversing() -> bool: return is_reversing
func get_steering_angle() -> float: return actual_steering_angle
func get_clutch_position() -> float: return clutch_position
func get_acceleration_vector() -> Vector3: return _acceleration

# ============================================================================
# INITIALIZATION
# ============================================================================
func _ready() -> void:
	_process_mode = ProcessModeEnum.ALWAYS
	await ready()
	_setup_physics_references()
	_init_audio()
	_reset_vehicle_state()
	_connect_signals_to_powertrain()

func _setup_physics_references() -> void:
	# Find child nodes by type or name
	_vehicle_body = find_child("*", true, false) as RigidBody3D
	if _vehicle_body == null:
		warning("VehicleController: No RigidBody3D found as child")
		return
	
	_powertrain_node = find_child("Powertrain", true, false)
	_wheel_nodes.clear()
	_suspension_nodes.clear()
	_collider_areas.clear()
	
	# Find all wheels and suspension points
	for child in get_children():
		if child.name.to_lower().contains("wheel"):
			_wheel_nodes.append(child)
		elif child.name.to_lower().contains("suspension"):
			_suspension_nodes.append(child)
		elif child is Area3D:
			_collider_areas.append(child)
	
	# Connect collision areas
	for area in _collider_areas:
		area.body_entered.connect(_on_collision_enter.bind(area))
		area.body_exited.connect(_on_collision_exit.bind(area))

func _init_audio() -> void:
	# Initialize audio players if they exist
	_engine_audio_player = find_child("EngineAudio", true, false) as AudioStreamPlayer3D
	_tire_audio_player = find_child("TireAudio", true, false) as AudioStreamPlayer3D
	_transmission_audio_player = find_child("TransmissionAudio", true, false) as AudioStreamPlayer3D

func _reset_vehicle_state() -> void:
	current_speed = 0.0
	current_rpm = IDLE_RPM
	current_gear = NEUTRAL_GEAR
	target_gear = NEUTRAL_GEAR
	clutch_position = 1.0
	is_clutch_pressed = false
	is_throttle_pressed = false
	is_brake_pressed = false
	is_handbrake_pressed = false
	steering_input = 0.0
	actual_steering_angle = 0.0
	is_drifting = false
	drift_angle = 0.0
	_acceleration = Vector3.ZERO
	_torque_output = 0.0
	_brake_force = 0.0
	_engine_braking = 0.0
	_collision_damage = 0.0
	_shift_cooldown = 0.0
	_is_stalled = false
	_is_reversing = false
	is_engine_running = true

# ============================================================================
# INPUT HANDLING
# ============================================================================
func handle_inputs(delta: float) -> void:
	var input_manager = GameManager.get_input_manager()
	if input_manager == null:
		return
	
	# Read all input axes
	var throttle_raw: float = input_manager.get_axis("throttle")
	var brake_raw: float = input_manager.get_axis("brake")
	var handbrake_raw: float = input_manager.get_axis("handbrake")
	var steering_raw: float = input_manager.get_axis("steering")
	var shift_up_raw: float = input_manager.get_axis("shift_up")
	var shift_down_raw: float = input_manager.get_axis("shift_down")
	var clutch_raw: float = input_manager.get_axis("clutch")
	var restart_raw: float = input_manager.get_axis("restart")
	
	# Update state based on input
	_update_input_states(throttle_raw, brake_raw, handbrake_raw, steering_raw, 
	                     clutch_raw, shift_up_raw, shift_down_raw, restart_raw)
	_process_gear_shifting(shift_up_raw, shift_down_raw, delta)
	_process_clutch_operation(clutch_raw, delta)
	_process_steering(steering_raw, delta)
	_handle_restart(restart_raw)

func _update_input_states(throttle: float, brake: float, handbrake: float, 
                          steering: float, clutch: float, shift_up: float, 
                          shift_down: float, restart: float) -> void:
	is_throttle_pressed = throttle > 0.1
	is_brake_pressed = brake > 0.1
	is_handbrake_pressed = handbrake > 0.1
	is_clutch_pressed = clutch > 0.1
	steering_input = clamp(steering, -1.0, 1.0)
	
	# Emit signals for input changes
	if is_throttle_pressed != signal_throttle_was_pressed:
		throttle_applied.emit(throttle)
		signal_throttle_was_pressed = is_throttle_pressed
	
	if is_brake_pressed != signal_brake_was_pressed:
		brake_applied.emit(brake)
		signal_brake_was_pressed = is_brake_pressed

func _process_gear_shifting(shift_up: float, shift_down: float, delta: float) -> void:
	if _shift_cooldown > 0.0:
		_shift_cooldown -= delta
		return
	
	if shift_up > 0.1 and current_gear < MAX_GEAR:
		_request_gear_change(current_gear + 1)
		_shift_cooldown = SHIFT_SHOCK_TIME
	
	elif shift_down > 0.1 and current_gear > MIN_GEAR:
		_request_gear_change(current_gear - 1)
		_shift_cooldown = SHIFT_SHOCK_TIME

func _request_gear_change(new_gear: int) -> void:
	if new_gear == current_gear:
		return
	
	var old_gear = current_gear
	target_gear = new_gear
	
	# Check if we need to press clutch
	if abs(target_gear - current_gear) > 1:
		# Multi-gear shift requires clutch
		if !is_clutch_pressed:
			return
	
	# Execute shift
	current_gear = target_gear
	gear_changed.emit(old_gear, current_gear)
	
	# Play shift sound via AudioManager
	if AudioManager:
		AudioManager.play_sound("gear_shift")

func _process_clutch_operation(clutch_input: float, delta: float) -> void:
	if clutch_input > 0.1:
		if clutch_position < 1.0:
			clutch_target = 0.0
	else:
		if clutch_position > 0.0:
			clutch_target = 1.0
	
	# Smooth clutch transition
	var clutch_speed = delta / CLUTCH_RELEASE_TIME
	if clutch_target < clutch_position:
		clutch_position = max(clutch_target, clutch_position - clutch_speed)
	else:
		clutch_position = min(clutch_target, clutch_position + clutch_speed)

func _process_steering(input: float, delta: float) -> void:
	# Smooth steering angle transition
	var target_angle = input * MAX_STEERING_ANGLE
	var steer_speed = STEERING_SPEED * delta
	
	if target_angle > actual_steering_angle:
		actual_steering_angle = min(target_angle, actual_steering_angle + steer_speed)
	else:
		actual_steering_angle = max(target_angle, actual_steering_angle - steer_speed)
	
	# Track drift angle
	drift_angle = abs(actual_steering_angle) - abs(input * MAX_STEERING_ANGLE)
	if drift_angle < 0:
		drift_angle = 0

func _handle_restart(restart_input: float) -> void:
	if restart_input > 0.1 and not is_engine_running:
		_attempt_engine_start()

# ============================================================================
# PHYSICS UPDATE LOOP
# ============================================================================
func _physics_process(delta: float) -> void:
	_last_time = _get_time_ticks_msec() / 1000.0
	
	# Handle engine running state
	if is_engine_running:
		_update_engine_rpm(delta)
	
	# Apply forces based on current state
	_apply_vehicle_forces(delta)
	
	# Update movement
	_update_velocity(delta)
	
	# Handle braking and collision damage
	_handle_braking_and_damage(delta)
	
	# Handle drifting mechanics
	_update_drift_state(delta)
	
	# Move the character body
	move_and_slide()

func _get_time_ticks_msec() -> float:
	return Time.get_ticks_msec()

func _update_engine_rpm(delta: float) -> void:
	var gear_ratio = _gear_ratios.get(current_gear, 1.0)
	var final_ratio = gear_ratio * _final_drive_ratio
	
	# Calculate theoretical RPM based on speed
	var wheel_rpm = (current_speed / 3.6) / (2.0 * PI * _tire_radius) * 60.0
	var theoretical_rpm = wheel_rpm * final_ratio
	
	# Engine RPM follows speed when clutch engaged, torque when disconnected
	if clutch_position < 0.1:
		# Clutch disengaged - engine can rev freely
		if is_throttle_pressed:
			current_rpm = lerp(current_rpm, REDLINE_RPM, 50.0 * delta)
		else:
			current_rpm = lerp(current_rpm, IDLE_RPM, 20.0 * delta)
	else:
		# Clutch engaged - RPM tied to wheels
		current_rpm = lerp(current_rpm, theoretical_rpm, 10.0 * delta)
	
	# Redline protection
	if current_rpm > REDLINE_RPM:
		current_rpm = REDLINE_RPM
		# Cut throttle briefly
		if is_throttle_pressed:
			is_throttle_pressed = false
	
	# Stall check
	if current_rpm < IDLE_RPM * 0.5 and !is_throttle_pressed and current_gear != NEUTRAL_GEAR:
		_stall_engine()
	
	# Emit RPM signal periodically
	if _rpm_signal_timer <= 0.0:
		rpm_changed.emit(current_rpm)
		_rpm_signal_timer = 0.1
	else:
		_rpm_signal_timer -= delta

func _apply_vehicle_forces(delta: float) -> void:
	if _vehicle_body == null:
		return
	
	# Calculate torque output based on gear and throttle
	_torque_output = _calculate_torque_output()
	
	# Apply engine torque to wheels
	_apply_torque_to_wheels()
	
	# Apply engine braking when in gear without throttle
	_engine_braking = _calculate_engine_braking()
	
	# Apply drag forces
	_apply_aerodynamic_drag(delta)
	
	# Apply rolling resistance
	_apply_rolling_resistance(delta)

func _calculate_torque_output() -> float:
	if !is_engine_running or is_stalled:
		return 0.0
	
	var gear_ratio = _gear_ratios.get(current_gear, 1.0)
	var final_ratio = gear_ratio * _final_drive_ratio
	
	# Torque curve - peak around 4000-6000 RPM
	var normalized_rpm = (current_rpm - IDLE_RPM) / (REDLINE_RPM - IDLE_RPM)
	var torque_curve = sin(normalized_rpm * PI) * 1.2  # Peak at ~50% RPM
	
	# Apply throttle multiplier
	var throttle_effect = 1.0 if is_throttle_pressed else 0.0
	
	# Apply clutch effect
	var clutch_effect = clutch_position
	
	var base_torque = ACCELERATION_POWER * torque_curve * throttle_effect * clutch_effect
	
	# Apply drift multiplier
	if is_drifting:
		base_torque *= _drift_multiplier
	
	return base_torque * final_ratio

func _apply_torque_to_wheels() -> void:
	if _wheel_nodes.is_empty() or _vehicle_body == null:
		return
	
	# Distribute torque to driven wheels (RWD simulation)
	var front_left = _wheel_nodes[0] if len(_wheel_nodes) > 0 else null
	var front_right = _wheel_nodes[1] if len(_wheel_nodes) > 1 else null
	var rear_left = _wheel_nodes[2] if len(_wheel_nodes) > 2 else null
	var rear_right = _wheel_nodes[3] if len(_wheel_nodes) > 3 else null
	
	# RWD only - apply torque to rear wheels
	if rear_left and rear_right:
		var drive_torque = _torque_output * 0.5 * DIFFERENTIAL_LOCK
		rear_left.apply_central_impulse(Vector3.FORWARD * drive_torque)
		rear_right.apply_central_impulse(Vector3.FORWARD * drive_torque)

func _calculate_engine_braking() -> float:
	if current_gear == NEUTRAL_GEAR or !is_engine_running:
		return 0.0
	
	var gear_ratio = _gear_ratios.get(current_gear, 1.0)
	var final_ratio = gear_ratio * _final_drive_ratio
	
	# Engine braking is stronger in lower gears
	var braking_factor = 0.3 if current_gear == 1 else 0.15
	
	return ACCELERATION_POWER * braking_factor * final_ratio

func _apply_aerodynamic_drag(delta: float) -> void:
	if _vehicle_body == null:
		return
	
	# Drag force proportional to velocity squared
	var speed = get_velocity().length()
	var drag_coefficient = 0.35  # Typical sports car
	var air_density = 1.225  # kg/m³ at sea level
	var frontal_area = 2.2  # m²
	
	var drag_force = 0.5 * air_density * drag_coefficient * frontal_area * speed * speed
	var drag_direction = -get_velocity().normalized() if speed > 0 else Vector3.ZERO
	
	_vehicle_body.apply_central_force(drag_direction * drag_force)

func _apply_rolling_resistance(delta: float) -> void:
	if _vehicle_body == null:
		return
	
	# Rolling resistance proportional to weight and velocity
	var mass = _vehicle_body.mass
	var crr = 0.015  # Coefficient of rolling resistance for tires
	var gravity = PhysicsSettings.gravity
	
	var rolling_resistance = crr * mass * gravity
	var velocity = get_velocity()
	var speed = velocity.length()
	
	if speed > 0:
		var resistance_direction = -velocity.normalized()
		_vehicle_body.apply_central_force(resistance_direction * rolling_resistance)

func _update_velocity(delta: float) -> void:
	if _vehicle_body == null:
		return
	
	var velocity = get_velocity()
	var speed_ms = velocity.length()
	
	# Update speed in km/h
	current_speed = speed_ms * 3.6
	
	# Limit top speed
	if current_speed > MAX_SPEED_KMH:
		var direction = velocity.normalized()
		var limited_speed = MAX_SPEED_KMH / 3.6
		velocity = direction * limited_speed
		set_velocity(velocity)
	
	# Apply acceleration vector
	_acceleration = velocity - _last_velocity
	_last_velocity = velocity
	
	# Emit speed signal
	if _speed_signal_timer <= 0.0:
		speed_changed.emit(current_speed)
		_speed_signal_timer = 0.1
	else:
		_speed_signal_timer -= delta

func _handle_braking_and_damage(delta: float) -> void:
	# Apply brake force if brake or handbrake pressed
	if is_brake_pressed or is_handbrake_pressed:
		_brake_force = BRAKING_FORCE if is_brake_pressed else BRAKING_FORCE * 1.5
		
		# ABS system check
		if is_abs_active and _check_wheel_lockup():
			_brake_force *= 0.5
		
		# Apply braking force to all wheels
		for wheel in _wheel_nodes:
			if wheel:
				wheel.apply_central_impulse(-get_velocity().normalized() * _brake_force * delta)
		
		# Handbrake induces drift
		if is_handbrake_pressed and current_speed > 30:
			is_drifting = true
			handbrake_toggled.emit(true)
	else:
		_brake_force = 0.0
		handbrake_toggled.emit(false)

func _check_wheel_lockup() -> bool:
	# Simplified wheel lockup detection
	var velocity = get_velocity()
	var speed = velocity.length()
	
	# If sliding significantly, wheels are locked
	return speed > 5 and _sliding_velocity > 10

func _update_drift_state(delta: float) -> void:
	var velocity = get_velocity()
	var speed = velocity.length()
	var lateral_velocity = velocity.x  # Simplified lateral check
	
	# Drift threshold
	if abs(lateral_velocity) > DRIFT_THRESHOLD * speed and is_handbrake_pressed:
		is_drifting = true
		if not _was_drifting:
			_drift_started_time = _get_time_ticks_msec() / 1000.0
			drift_started.emit(abs(drift_angle))
			if AudioManager:
				AudioManager.play_sound("drift_start")
	else:
		if is_drifting:
			is_drifting = false
			drift_ended.emit()
			if AudioManager:
				AudioManager.play_sound("drift_end")
	
	_was_drifting = is_drifting

# ============================================================================
# VEHICLE STATE MANAGEMENT
# ============================================================================
func start_engine() -> void:
	if is_engine_running:
		return
	
	is_engine_running = true
	is_stalled = false
	
	# Ramp up RPM to idle
	await create_timer(_engine_start_delay).timeout
	current_rpm = IDLE_RPM
	
	if AudioManager:
		AudioManager.play_sound("engine_start")

func stop_engine() -> void:
	if !is_engine_running:
		return
	
	is_engine_running = false
	current_rpm = IDLE_RPM
	
	if AudioManager:
		AudioManager.play_sound("engine_stop")

func _attempt_engine_start() -> void:
	if is_engine_running:
		return
	
	start_engine()

func _stall_engine() -> void:
	if is_stalled:
		return
	
	is_stalled = true
	is_engine_running = false
	current_rpm = IDLE_RPM
	
	stalled.emit()
	
	if AudioManager:
		AudioManager.play_sound("engine_stall")

func reset_vehicle() -> void:
	_reset_vehicle_state()
	_reset_physics()

func _reset_physics() -> void:
	if _vehicle_body:
		_vehicle_body.linear_damp = 0.5
		_vehicle_body.angular_damp = 0.5
		_vehicle_body.apply_central_impulse(Vector3.UP * 10.0)

# ============================================================================
# COLLISION HANDLING
# ============================================================================
func _on_collision_enter(body: Node3D, area: Area3D) -> void:
	if not is_engine_running:
		return
	
	var impact_speed = get_velocity().length()
	var impact_force = impact_speed * _vehicle_body.mass
	
	if impact_force > _damage_threshold:
		_collision_damage += impact_force / 10.0
		
		# Screen shake effect
		if GameManager:
			GameManager.trigger_screen_shake(min(impact_speed / 10, 0.5))
		
		collision_detected.emit({
			"body": body.name if body else "unknown",
			"impact_speed": impact_speed,
			"impact_force": impact_force,
			"damage": _collision_damage
		})
		
		if _collision_damage >= 100.0:
			_destroy_vehicle()

func _on_collision_exit(body: Node3D, area: Area3D) -> void:
	pass  # Can track exit events if needed

func _destroy_vehicle() -> void:
	is_engine_running = false
	is_stalled = true
	
	if AudioManager:
		AudioManager.play_sound("crash_explosion")
	
	# Apply massive deceleration
	if _vehicle_body:
		_vehicle_body.linear_damp = 5.0
		_vehicle_body.apply_central_impulse(Vector3.DOWN * 100.0)

# ============================================================================
# HELPER FUNCTIONS
# ============================================================================
func calculate_optimal_gear(speed: float, rpm: float) -> int:
	# Find best gear for current speed and RPM
	var optimal_gear = NEUTRAL_GEAR
	
	for gear in range(MIN_GEAR, MAX_GEAR + 1):
		if gear == NEUTRAL_GEAR:
			continue
		
		var gear_ratio = _gear_ratios.get(gear, 1.0)
		var final_ratio = gear_ratio * _final_drive_ratio
		var wheel_rpm = (speed / 3.6) / (2.0 * PI * _tire_radius) * 60.0
		var engine_rpm = wheel_rpm * final_ratio
		
		if engine_rpm >= IDLE_RPM and engine_rpm <= REDLINE_RPM:
			optimal_gear = gear
			break
	
	return optimal_gear

func calculate_distance_traveled() -> float:
	return position.distance_to(_initial_position) if _initial_position else 0.0

func get_lap_time() -> float:
	return _lap_start_time if _lap_start_time else 0.0

func start_lap() -> void:
	_lap_start_time = _get_time_ticks_msec() / 1000.0

func end_lap() -> float:
	var lap_time = (_get_time_ticks_msec() / 1000.0) - _lap_start_time
	_lap_start_time = 0.0
	return lap_time

# ============================================================================
# DEBUG AND TESTING
# ============================================================================
func debug_print_status() -> void:
	print("=== Vehicle Status ===")
	print("Speed: %.2f km/h" % current_speed)
	print("RPM: %.0f" % current_rpm)
	print("Gear: %d" % current_gear)
	print("Throttle: %s" % ("ON" if is_throttle_pressed else "OFF"))
	print("Brake: %s" % ("ON" if is_brake_pressed else "OFF"))
	print("Handbrake: %s" % ("ON" if is_handbrake_pressed else "OFF"))
	print("Clutch: %.2f" % clutch_position)
	print("Drifting: %s" % ("YES" if is_drifting else "NO"))
	print("Engine Running: %s" % ("YES" if is_engine_running else "NO"))
	print("=====================")

func set_debug_mode(enabled: bool) -> void:
	debug_mode = enabled

# ============================================================================
# CLEANUP
# ============================================================================
func _exit_tree() -> void:
	# Disconnect any remaining signals
	_disconnect_all_signals()

func _disconnect_all_signals() -> void:
	pass  # Remove signal connections if needed

# ============================================================================
# PRIVATE VARIABLES FOR TIMERS
# ============================================================================
var _rpm_signal_timer: float = 0.0
var _speed_signal_timer: float = 0.0
var _last_velocity: Vector3 = Vector3.ZERO
var _sliding_velocity: float = 0.0
var _was_drifting: bool = false
var _drift_started_time: float = 0.0
var _initial_position: Vector3 = Vector3.ZERO
var _lap_start_time: float = 0.0
var _damage_timer: float = 0.0
var _signal_throttle_was_pressed: bool = false
var _signal_brake_was_pressed: bool = false
var debug_mode: bool = false
var _timer_instance: Timer = null