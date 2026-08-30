extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulation
## Handles throttle, brake, steering, wheel forces, gear shifting, and vehicle dynamics
## Copyright 2026 Thalamus Racing Simulator Project

# ============================================================================
# SIGNALS
# ============================================================================
signal speed_changed(new_speed: float)
signal rpm_changed(rpm: int)
signal gear_changed(gear: int)
signal drift_started(drift_angle: float)
signal drift_ended()
signal collision_detected(impact_velocity: Vector3)
signal lap_completed(lap_data: Dictionary)
signal race_event(event_type: String)
signal engine_sound_changed(rpm_ratio: float)
signal suspension_compressed(suspension_amount: float)

# ============================================================================
# CONSTANTS & CONFIGURATION
# ============================================================================
const MAX_ENGINE_RPM: int = 8500
const IDLE_RPM: int = 800
const REDLINE_RPM: int = 7200
const GEAR_RATIOS: Array[float] = [0.0, 3.8f, 2.4f, 1.7f, 1.3f, 1.1f, 0.85f, 0.65f]
const FINAL_DRIVE_RATIO: float = 3.73f
const WHEEL_RADIUS: float = 0.32
const MAX_REVERSE_SPEED: float = -15.0
const MAX_FORWARD_SPEED: float = 120.0
const DRIFT_THRESHOLD: float = 0.5
const GRIP_LEVEL_NORMAL: float = 0.95
const GRIP_LEVEL_DRIFT: float = 0.35
const STEERING_SPEED: float = 15.0
const BRAKING_FORCE: float = 2.5
const ACCELERATION_FORCE: float = 1.8
const TURNING_RADIUS: float = 8.0
const MIN_GEAR: int = 0
const MAX_GEAR: int = 6
const SHIFT_DELAY_TIME: float = 0.15
const CLUTCH_DISENGAGE_RPM: float = 0.0
const ENGINE_BRAKE_FACTOR: float = 0.3

# ============================================================================
# EXPORTED PROPERTIES
# ============================================================================
@export_group("Vehicle Configuration")
@export var vehicle_mass: float = 1500.0: set = _set_vehicle_mass
@export var center_of_mass: Vector3 = Vector3(0.0, 0.5, 0.0)
@export var aerodynamic_drag: float = 0.35
@export var rolling_resistance: float = 0.015
@export var tire_friction_coefficient: float = 1.2
@export var front_weight_distribution: float = 0.45

@export_group("Engine Performance")
@export var max_torque: float = 450.0
@export var torque_curve: Curve = null
@export var power_band_start: float = 3000.0
@export var power_band_end: float = 6500.0

@export_group("Steering & Handling")
@export var max_steering_angle: float = 30.0 * DEG2RAD
@export var steering_sensitivity: float = 1.0
@export var steer_recovery_speed: float = 10.0
@export var understeer_factor: float = 0.1
@export var oversteer_factor: float = 0.15

@export_group("Suspension Settings")
@export var suspension_stiffness: float = 50000.0
@export var suspension_damping: float = 8000.0
@export var suspension_travel: float = 0.15
@export var suspension_rest_length: float = 0.3

# ============================================================================
# INTERNAL STATE VARIABLES
# ============================================================================
var current_gear: int = 1
var target_gear: int = 1
var clutch_engaged: bool = true
var brake_pressed: bool = false
var handbrake_active: bool = false
var throttle_input: float = 0.0
var steering_input: float = 0.0
var current_rpm: int = IDLE_RPM
var current_speed_kmh: float = 0.0
var acceleration_force: float = 0.0
var braking_force: float = 0.0
var steering_angle: float = 0.0
var drift_angle: float = 0.0
var is_drifting: bool = false
var wheel_slip_front: float = 0.0
var wheel_slip_rear: float = 0.0
var traction_control_active: bool = true
var abs_active: bool = true

# Timing variables
var _last_shift_time: float = 0.0
var _acceleration_accumulator: float = 0.0
var _deceleration_accumulator: float = 0.0
var _turn_accumulator: float = 0.0
var _last_frame_time: float = 0.0

# Physics components (will be assigned by parent scene)
var _wheel_colliders: Array[Node3D] = []
var _suspension_nodes: Array[Node3D] = []
var _engine_node: Node = null
var _transmission_node: Node = null

# Reference to input manager
var _input_manager: Node = null

# ============================================================================
# GETTERS & SETTERS
# ============================================================================
func get_current_speed() -> float:
	return current_speed_kmh

func get_current_rpm() -> int:
	return current_rpm

func get_current_gear() -> int:
	return current_gear

func get_is_engine_running() -> bool:
	return current_rpm > IDLE_RPM

func get_wheel_slip_front() -> float:
	return wheel_slip_front

func get_wheel_slip_rear() -> float:
	return wheel_slip_rear

func get_drift_status() -> bool:
	return is_drifting

func get_steering_angle() -> float:
	return steering_angle

# ============================================================================
# LIFECYCLE METHODS
# ============================================================================
func _ready() -> void:
	_process_mode = ProcessModeEnum.PROCESS_MODE_PHYSICS
	_last_frame_time = Time.get_ticks_msec() / 1000.0
	
	_init_physics_components()
	_connect_signals_to_singleton()
	_apply_initial_configuration()

func _exit_tree() -> void:
	_disconnect_all_signals()

func _physics_process(delta: float) -> void:
	if delta <= 0.0:
		return
	
	_update_physics_state(delta)
	_update_vehicle_dynamics(delta)
	_update_audio_feedback(delta)
	
	apply_velocity()

# ============================================================================
# PHYSICS UPDATE METHODS
# ============================================================================
func _update_physics_state(delta: float) -> void:
	_handle_inputs(delta)
	_update_transmission(delta)
	_calculate_acceleration(delta)
	_calculate_braking(delta)
	_update_steering(delta)
	_update_suspension(delta)
	_check_drift_conditions(delta)
	_update_wheels(delta)

func _handle_inputs(delta: float) -> void:
	if _input_manager:
		throttle_input = _input_manager.get_action_value("throttle")
		steering_input = _input_manager.get_action_value("steering")
		brake_pressed = _input_manager.is_action_pressed("brake")
		handbrake_active = _input_manager.is_action_pressed("handbrake")
		
		# Gear shifting inputs
		if _input_manager.is_action_just_pressed("upshift"):
			_request_gear_shift(1)
		elif _input_manager.is_action_just_pressed("downshift"):
			_request_gear_shift(-1)
	else:
		# Fallback if no input manager
		pass

func _update_transmission(delta: float) -> void:
	# Handle clutch engagement delay during shifts
	if not clutch_engaged:
		_last_shift_time += delta
		if _last_shift_time >= SHIFT_DELAY_TIME:
			clutch_engaged = true
			_last_shift_time = 0.0
	
	# Update target gear based on RPM and speed
	target_gear = _calculate_target_gear()
	
	# Perform gear shift if needed
	if current_gear != target_gear and clutch_engaged:
		_execute_gear_shift(target_gear)

func _calculate_target_gear() -> int:
	var target: int = current_gear
	
	# Prevent upshifting beyond top gear
	if current_gear < MAX_GEAR and current_rpm >= REDLINE_RPM:
		target = min(current_gear + 1, MAX_GEAR)
	# Prevent downshifting below neutral
	elif current_gear > MIN_GEAR and current_rpm <= IDLE_RPM and current_speed_kmh < 5.0:
		target = max(current_gear - 1, MIN_GEAR)
	
	# Auto-downshift if speed drops too low for current gear
	if current_gear > MIN_GEAR:
		var min_speed_for_gear = _get_min_speed_for_gear(current_gear)
		if current_speed_kmh < min_speed_for_gear * 0.7:
			target = max(current_gear - 1, MIN_GEAR)
	
	return clamp(target, MIN_GEAR, MAX_GEAR)

func _execute_gear_shift(new_gear: int) -> void:
	current_gear = new_gear
	target_gear = new_gear
	clutch_engaged = false
	_last_shift_time = 0.0
	
	gear_changed.emit(current_gear)
	
	# Trigger transmission event
	race_event.emit("gear_shift_%d" % new_gear)

# ============================================================================
# DYNAMICS CALCULATIONS
# ============================================================================
func _calculate_acceleration(delta: float) -> void:
	# Calculate engine RPM based on current gear and speed
	var wheel_rps = abs(current_speed_kmh) / (3.6 * 2.0 * PI * WHEEL_RADIUS)
	var engine_rps = wheel_rps * GEAR_RATIOS[current_gear] * FINAL_DRIVE_RATIO
	current_rpm = int(engine_rps * 60.0)
	
	# Clamp RPM to valid range
	current_rpm = clamp(current_rpm, IDLE_RPM, MAX_ENGINE_RPM)
	
	# Get torque output based on RPM curve
	var torque_output = _get_torque_at_rpm(current_rpm)
	
	# Apply clutch state
	if clutch_engaged:
		acceleration_force = torque_output * GEAR_RATIOS[current_gear] * FINAL_DRIVE_RATIO / WHEEL_RADIUS
		
		# Apply traction control if active
		if traction_control_active:
			acceleration_force *= _calculate_traction_factor()
	else:
		acceleration_force = 0.0
	
	# Apply aerodynamic drag
	var drag_force = 0.5 * aerodynamic_drag * current_speed_kmh * current_speed_kmh / 100.0
	acceleration_force -= drag_force
	
	# Apply rolling resistance
	var rolling_force = rolling_resistance * vehicle_mass * gravity
	acceleration_force -= rolling_force
	
	# Accumulate acceleration for smooth force application
	_acceleration_accumulator += acceleration_force * delta
	_acceleration_accumulator = clamp(_acceleration_accumulator, -ACCELERATION_FORCE, ACCELERATION_FORCE)

func _calculate_braking(delta: float) -> void:
	braking_force = 0.0
	
	# Brake pedal input
	if brake_pressed:
		braking_force = BRAKING_FORCE * (1.0 - current_rpm / float(MAX_ENGINE_RPM))
	
	# Handbrake adds additional braking
	if handbrake_active:
		braking_force += BRAKING_FORCE * 0.5
	
	# Engine braking in gear
	if current_gear > MIN_GEAR and not brake_pressed:
		var engine_brake = ENGINE_BRAKE_FACTOR * max_torque * GEAR_RATIOS[current_gear] * FINAL_DRIVE_RATIO / WHEEL_RADIUS
		if current_speed_kmh > 0:
			engine_brake *= -1.0
		braking_force += engine_brake
	
	# ABS check
	if abs_active and braking_force > 0:
		braking_force *= _calculate_abs_factor()
	
	# Apply negative acceleration
	_deceleration_accumulator += braking_force * delta
	_deceleration_accumulator = max(_deceleration_accumulator, -BRAKING_FORCE)

func _update_steering(delta: float) -> void:
	var target_steering = steering_input * max_steering_angle * steering_sensitivity
	
	# Smooth steering transition
	_turn_accumulator += (target_steering - steering_angle) * delta * STEERING_SPEED
	_turn_accumulator = clamp(_turn_accumulator, -max_steering_angle, max_steering_angle)
	
	steering_angle = lerp(steering_angle, target_steering, delta * steer_recovery_speed)
	
	# Steering recovery when released
	if abs(steering_input) < 0.05:
		steering_angle = lerp(steering_angle, 0.0, delta * steer_recovery_speed)

func _update_suspension(delta: float) -> void:
	# Simplified suspension calculation
	for i in range(_suspension_nodes.size()):
		var node: Node3D = _suspension_nodes[i]
		if node:
			var compression = _calculate_suspension_compression(i, delta)
			node.position.y = suspension_rest_length - compression
			suspension_compressed.emit(compression)

func _check_drift_conditions(delta: float) -> void:
	var velocity_vector = velocity.xz.length()
	var lateral_velocity = velocity.xz.dot(Vector2(-velocity.y, velocity.x)).length()
	
	# Calculate slip angle approximation
	var slip_angle = asin(lateral_velocity / max(velocity_vector, 1.0))
	
	# Check drift threshold
	if abs(slip_angle) > DRIFT_THRESHOLD and handbrake_active:
		if not is_drifting:
			is_drifting = true
			drift_started.emit(abs(slip_angle))
		drift_angle = slip_angle
	else:
		if is_drifting:
			is_drifting = false
			drift_ended.emit()
		drift_angle = lerp(drift_angle, 0.0, delta * 2.0)

func _update_wheels(delta: float) -> void:
	# Calculate wheel slip ratios
	var drive_wheels_slip = acceleration_force / (vehicle_mass * tire_friction_coefficient)
	wheel_slip_rear = clamp(drive_wheels_slip, -1.0, 1.0)
	wheel_slip_front = 0.0  # Front wheels don't drive in rear-wheel drive
	
	# Update wheel collider rotations
	for i in range(_wheel_colliders.size()):
		var wheel: Node3D = _wheel_colliders[i]
		if wheel:
			var rotation_speed = current_speed_kmh / 3.6 / WHEEL_RADIUS
			wheel.rotation.y -= rotation_speed * delta

# ============================================================================
# AUDIO FEEDBACK
# ============================================================================
func _update_audio_feedback(delta: float) -> void:
	var rpm_ratio = (current_rpm - IDLE_RPM) / float(MAX_ENGINE_RPM - IDLE_RPM)
	engine_sound_changed.emit(rpm_ratio)
	speed_changed.emit(current_speed_kmh)
	rpm_changed.emit(current_rpm)

# ============================================================================
# HELPER FUNCTIONS
# ============================================================================
func _get_torque_at_rpm(rpm: int) -> float:
	if torque_curve:
		return torque_curve.sample(float(rpm) / float(MAX_ENGINE_RPM)) * max_torque
	else:
		# Default bell curve torque profile
		var normalized_rpm = float(rpm) / float(MAX_ENGINE_RPM)
		if normalized_rpm < 0.3:
			return max_torque * normalized_rpm / 0.3
		elif normalized_rpm > 0.75:
			return max_torque * (1.0 - (normalized_rpm - 0.75) / 0.25)
		else:
			return max_torque

func _get_min_speed_for_gear(gear: int) -> float:
	if gear == MIN_GEAR:
		return 0.0
	var gear_ratio = GEAR_RATIOS[gear]
	return 10.0 / (gear_ratio * FINAL_DRIVE_RATIO * WHEEL_RADIUS * 3.6)

func _calculate_traction_factor() -> float:
	var base_grif = GRIP_LEVEL_NORMAL
	if is_drifting:
		base_grif = GRIP_LEVEL_DRIFT
	return base_grif

func _calculate_abs_factor() -> float:
	if wheel_slip_rear > 0.3 or wheel_slip_rear < -0.2:
		return 0.7
	return 1.0

func _calculate_suspension_compression(wheel_index: int, delta: float) -> float:
	# Simplified compression calculation
	var vertical_velocity = velocity.y
	var compression = abs(vertical_velocity) * delta * suspension_stiffness
	return min(compression, suspension_travel)

# ============================================================================
# INITIALIZATION
# ============================================================================
func _init_physics_components() -> void:
	# Find child nodes that represent wheels and suspension
	for child in get_children():
		if child.name.contains("Wheel"):
			_wheel_colliders.append(child)
		elif child.name.contains("Suspension"):
			_suspension_nodes.append(child)
		elif child.name.contains("Engine"):
			_engine_node = child
		elif child.name.contains("Transmission"):
			_transmission_node = child

func _connect_signals_to_singleton() -> void:
	if GameManager:
		GameManager.game_state_changed.connect(_on_game_state_changed)

func _disconnect_all_signals() -> void:
	if GameManager:
		GameManager.game_state_changed.disconnect(_on_game_state_changed)

func _on_game_state_changed(new_state: GameState) -> void:
	match new_state:
		GameManager.GameState.RACE_ACTIVE:
			_resume_simulation()
		GameManager.GameState.RACE_PAUSED:
			_pause_simulation()
		GameManager.GameState.RACE_FINISHED:
			_complete_race()

func _resume_simulation() -> void:
	_process_mode = ProcessModeEnum.PROCESS_MODE_PHYSICS

func _pause_simulation() -> void:
	_process_mode = ProcessModeEnum.PROCESS_MODE_DISABLED

func _complete_race() -> void:
	# Handle race completion logic
	pass

# ============================================================================
# PUBLIC API
# ============================================================================
func reset_vehicle() -> void:
	current_gear = 1
	target_gear = 1
	current_rpm = IDLE_RPM
	current_speed_kmh = 0.0
	throttle_input = 0.0
	steering_input = 0.0
	brake_pressed = false
	handbrake_active = false
	is_drifting = false
	acceleration_force = 0.0
	braking_force = 0.0
	steering_angle = 0.0
	wheel_slip_front = 0.0
	wheel_slip_rear = 0.0
	_acceleration_accumulator = 0.0
	_deceleration_accumulator = 0.0
	_turn_accumulator = 0.0

func force_gear(gear: int) -> void:
	current_gear = clamp(gear, MIN_GEAR, MAX_GEAR)
	target_gear = current_gear
	gear_changed.emit(current_gear)

func toggle_traction_control(active: bool) -> void:
	traction_control_active = active

func toggle_abs(active: bool) -> void:
	abs_active = active

func set_aerodynamic_settings(drag_coefficient: float, downforce: float) -> void:
	aerodynamic_drag = drag_coefficient
	# Downforce would be applied here in a full implementation

func get_vehicle_stats() -> Dictionary:
	return {
		"speed": current_speed_kmh,
		"rpm": current_rpm,
		"gear": current_gear,
		"is_drifting": is_drifting,
		"throttle": throttle_input,
		"brake": brake_pressed,
		"handbrake": handbrake_active,
		"steering": steering_angle,
		"wheel_slip_front": wheel_slip_front,
		"wheel_slip_rear": wheel_slip_rear
	}

# ============================================================================
# PROPERTY SETTERS
# ============================================================================
func _set_vehicle_mass(value: float) -> void:
	vehicle_mass = value
	# Recalculate inertia and other mass-dependent values
	pass

# ============================================================================
# RACE DATA HANDLING
# ============================================================================
func record_lap_data(lap_number: int, time_seconds: float, sector_times: Array[float]) -> void:
	var lap_data: Dictionary = {
		"lap_number": lap_number,
		"time": time_seconds,
		"sector_times": sector_times,
		"timestamp": Time.get_unix_time_from_system(),
		"average_speed": current_speed_kmh,
		"peak_rpm": current_rpm,
		"drift_events": 0,
		"gear_shifts": 0
	}
	lap_completed.emit(lap_data)

func get_best_lap_time() -> float:
	# Placeholder for retrieving best lap time from stored data
	return 0.0

func clear_lap_records() -> void:
	# Clear stored lap data
	pass

# ============================================================================
# DEBUG TOOLS
# ============================================================================
func debug_print_status() -> void:
	print("[VehicleController] Speed: %.1f km/h | RPM: %d | Gear: %d | Drift: %s" % [
		current_speed_kmh, current_rpm, current_gear, "YES" if is_drifting else "NO"
	])
	print("[VehicleController] Throttle: %.2f | Brake: %s | Handbrake: %s" % [
		throttle_input, str(brake_pressed), str(handbrake_active)
	])
	print("[VehicleController] Wheel Slip Front: %.2f | Rear: %.2f" % [wheel_slip_front, wheel_slip_rear])

func toggle_debug_mode(enabled: bool) -> void:
	debug_mode = enabled
	pass

</file>