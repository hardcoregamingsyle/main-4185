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
const DIFFERENTIAL_LOCK_RATIO: float = 0.8
const DRIFT_FACTOR: float = 0.15
const TRACTION_CONTROL_THRESHOLD: float = 0.15
const ABS_THRESHOLD: float = 0.1
const SHIFT_POINT_RPM: float = 7000.0
const SHUTDOWN_RPM: float = 9000.0
const GEAR_SHIFT_DELAY: float = 0.1
const ENGINE_REVO_DROP_ON_DOWNSHIFT: float = 2000.0

# ============================================================================
# STATE VARIABLES
# ============================================================================
var _speed_kmh: float = 0.0
var _rpm: float = IDLE_RPM
var _current_gear: int = NEUTRAL_GEAR
var _target_gear: int = NEUTRAL_GEAR
var _throttle_input: float = 0.0
var _brake_input: float = 0.0
var _steering_input: float = 0.0
var _handbrake_input: float = 0.0

var _is_engine_running: bool = false
var _is_clutch_engaged: bool = true
var _is_handbrake_active: bool = false
var _is_traction_control_active: bool = true
var _is_abs_active: bool = true
var _is_drifting: bool = false
var _drift_angle: float = 0.0
var _last_collision_time: float = 0.0
var _gear_shift_timer: float = 0.0
var _clutch_timer: float = 0.0

# Powertrain reference
var _powertrain: Node = null

# Physics settings reference
var _physics_settings: PhysicsSettings = PhysicsSettings.new()

# Wheel friction coefficients
var _wheel_friction_coefficient: float = 1.2
var _wheel_slip_coefficient: float = 0.8

# Drift state
var _drift_velocity_threshold: float = 5.0
var _drift_recovery_rate: float = 0.1

# Gear ratios (final drive ratio included)
var _gear_ratios: Array[float] = [
	-3.5,  # Reverse
	3.8,   # 1st
	2.5,   # 2nd
	1.8,   # 3rd
	1.4,   # 4th
	1.1,   # 5th
	0.9    # 6th
]

# Final drive ratio
var _final_drive_ratio: float = 3.73

# Tire parameters
var _tire_radius: float = 0.33
var _tire_width: float = 0.25

# Collision tracking
var _collisions: Array[Dictionary] = []
var _max_collisions_history: int = 10

# Debug flags
var _debug_enabled: bool = false
var _show_debug_overlay: bool = false

# ============================================================================
# PUBLIC ACCESSORS
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

func get_handbrake_input() -> float:
	return _handbrake_input

func is_engine_running() -> bool:
	return _is_engine_running

func is_clutch_engaged() -> bool:
	return _is_clutch_engaged

func is_drifting() -> bool:
	return _is_drifting

func get_drift_angle() -> float:
	return _drift_angle

func get_max_speed_kmh() -> float:
	return MAX_SPEED_KMH

func get_acceleration_power() -> float:
	return ACCELERATION_POWER

func get_braking_force() -> float:
	return BRAKING_FORCE

# ============================================================================
# INITIALIZATION
# ============================================================================
func _ready() -> void:
	_init_physics_settings()
	_connect_signals_to_manager()
	_setup_audio()
	_reset_vehicle()
	
	# Set process mode to always for physics updates
	process_mode = ProcessModeEnum.ALWAYS
	
	if _physics_settings:
		_wheel_friction_coefficient = _physics_settings.default_vehicle_mass / 1500.0 * 1.2

func _init_physics_settings() -> void:
	"""Initialize physics settings from resource"""
	_physics_settings.gravity = 9.81
	_physics_settings.physics_tick_rate = 120
	_physics_settings.max_substeps = 4
	_physics_settings.time_scale = 1.0

func _connect_signals_to_manager() -> void:
	"""Connect to GameManager if available"""
	if GameManager:
		GameManager.game_state_changed.connect(_on_game_state_changed)
		GameManager.race_started.connect(_on_race_started)

func _setup_audio() -> void:
	"""Setup audio references if AudioManager exists"""
	if AudioManager:
		pass  # Audio will be triggered when needed

# ============================================================================
# INPUT HANDLING
# ============================================================================
func _input(event: InputEvent) -> void:
	"""Handle player input events"""
	if event.is_action_pressed("ui_up"):
		_throttle_input = min(_throttle_input + 0.1, 1.0)
	elif event.is_action_released("ui_up"):
		_throttle_input = max(_throttle_input - 0.1, 0.0)
		
	if event.is_action_pressed("ui_down"):
		_brake_input = min(_brake_input + 0.1, 1.0)
	elif event.is_action_released("ui_down"):
		_brake_input = max(_brake_input - 0.1, 0.0)
		
	if event.is_action_pressed("ui_left"):
		_steering_input = min(_steering_input + 0.1, 1.0)
	elif event.is_action_released("ui_left"):
		_steering_input = max(_steering_input - 0.1, 0.0)
		
	if event.is_action_pressed("ui_right"):
		_steering_input = max(_steering_input - 0.1, -1.0)
	elif event.is_action_released("ui_right"):
		_steering_input = min(_steering_input + 0.1, 1.0)

func _process(delta: float) -> void:
	"""Main game loop processing"""
	if not is_inside_tree():
		return
	
	# Update physics based on delta time
	_update_physics(delta)
	
	# Handle input smoothing
	_smooth_inputs(delta)
	
	# Check for automatic gear shifts
	_check_auto_shifts(delta)
	
	# Update RPM based on current gear and speed
	_update_rpm()
	
	# Handle drift mechanics
	_update_drift(delta)
	
	# Apply collisions and effects
	_handle_collisions(delta)
	
	# Emit signals for changes
	_emit_signal_updates()
	
	# Update debug overlay if enabled
	if _debug_enabled and _show_debug_overlay:
		_update_debug_overlay()

func _physics_process(delta: float) -> void:
	"""Physics-based processing with fixed timestep"""
	if not is_inside_tree():
		return
	
	# Use fixed timestep accumulator for consistent physics
	var fixed_delta: float = 1.0 / _physics_settings.physics_tick_rate
	var accumulator: float = delta
	var steps_taken: int = 0
	
	while accumulator >= fixed_delta and steps_taken < _physics_settings.max_substeps:
		_step_physics(fixed_delta)
		accumulator -= fixed_delta
		steps_taken += 1
	
	# Interpolate for rendering if partial step remains
	if accumulator > 0 and steps_taken > 0:
		var interpolation_factor: float = accumulator / fixed_delta
		_interpolate_physics(interpolation_factor)

func _step_physics(delta: float) -> void:
	"""Execute one physics substep"""
	# Calculate forces
	var drive_force: float = _calculate_drive_force()
	var drag_force: float = _calculate_drag_force()
	var braking_force: float = _calculate_braking_force()
	var lateral_force: float = _calculate_lateral_force()
	
	# Apply forces to velocity
	_apply_forces_to_body(drive_force, drag_force, braking_force, lateral_force, delta)
	
	# Update position
	_move_and_slide()
	
	# Clamp velocity within bounds
	_clamp_velocity()

func _interpolate_physics(factor: float) -> void:
	"""Interpolate physics state between frames"""
	pass  # Placeholder for interpolation logic

# ============================================================================
# PHYSICS CALCULATIONS
# ============================================================================
func _calculate_drive_force() -> float:
	"""Calculate forward/backward drive force based on throttle and gear"""
	if not _is_engine_running:
		return 0.0
	
	if not _is_clutch_engaged:
		return 0.0
	
	if _current_gear == NEUTRAL_GEAR:
		return 0.0
	
	# Get engine torque curve (simplified)
	var engine_torque: float = _get_engine_torque(_rpm)
	
	# Apply throttle multiplier
	engine_torque *= _throttle_input
	
	# Calculate transmission output torque
	var total_ratio: float = _gear_ratios[_current_gear] * _final_drive_ratio
	var output_torque: float = engine_torque * total_ratio
	
	# Convert to linear force (torque / radius)
	var drive_force: float = output_torque / _tire_radius
	
	# Apply differential lock effect
	if _is_drifting:
		drive_force *= DIFFERENTIAL_LOCK_RATIO
	
	return drive_force

func _calculate_drag_force() -> float:
	"""Calculate aerodynamic drag force"""
	if _speed_kmh <= 0:
		return 0.0
	
	# Simplified drag equation: F = 0.5 * rho * v^2 * Cd * A
	# Using simplified constants for gameplay feel
	var air_density: float = 1.225
	var velocity_mps: float = _speed_kmh / 3.6
	var drag_coefficient: float = 0.35  # Sports car coefficient
	var frontal_area: float = 2.2  # m^2
	
	var drag_force: float = 0.5 * air_density * pow(velocity_mps, 2) * drag_coefficient * frontal_area
	
	# Apply to opposite direction of motion
	return -drag_force

func _calculate_braking_force() -> float:
	"""Calculate braking force based on brake input"""
	if _brake_input <= 0:
		return 0.0
	
	var base_braking_force: float = BRAKING_FORCE * _brake_input
	
	# Apply ABS if active and wheels are locking
	if _is_abs_active and _check_wheels_locking():
		base_braking_force *= (1.0 - ABS_THRESHOLD)
	
	# Handbrake adds additional rear braking
	if _is_handbrake_active:
		base_braking_force *= 0.5
	
	return base_braking_force

func _calculate_lateral_force() -> float:
	"""Calculate lateral force for cornering and drifting"""
	var lateral_force: float = 0.0
	
	# Steering input creates lateral force
	lateral_force = _steering_input * MAX_STEERING_ANGLE
	
	# Apply to body rotation
	_apply_lateral_rotation(lateral_force)
	
	return lateral_force

func _apply_forces_to_body(drive_force: float, drag_force: float, braking_force: float, lateral_force: float, delta: float) -> void:
	"""Apply calculated forces to the vehicle body"""
	# Sum longitudinal forces
	var longitudinal_force: float = drive_force + drag_force + braking_force
	
	# Apply acceleration from longitudinal forces
	var acceleration: float = longitudinal_force / _physics_settings.default_vehicle_mass
	
	# Update velocity
	var current_velocity: Vector3 = global_transform.basis.z.normalized() * _speed_kmh / 3.6
	current_velocity += acceleration * delta * Vector3.FORWARD
	
	# Apply lateral force (side slip)
	current_velocity += lateral_force * delta * Vector3.RIGHT
	
	# Update body velocity
	velocity = current_velocity

func _clamp_velocity() -> void:
	"""Clamp vehicle velocity within physical limits"""
	# Calculate actual speed in km/h
	var speed_vector: Vector3 = velocity.length()
	var speed_actual_kmh: float = speed_vector * 3.6
	
	# Clamp to maximum speed
	if speed_actual_kmh > MAX_SPEED_KMH:
		speed_actual_kmh = MAX_SPEED_KMH
	
	# Reconstruct velocity vector with clamped speed
	var direction: Vector3 = velocity.normalized() if velocity.length() > 0 else Vector3.ZERO
	velocity = direction * speed_actual_kmh / 3.6

func _move_and_slide() -> void:
	"""Move vehicle and handle collisions"""
	move_and_slide()

# ============================================================================
# ENGINE AND TRANSMISSION
# ============================================================================
func _update_rpm() -> void:
	"""Update engine RPM based on current gear and vehicle speed"""
	if _current_gear == NEUTRAL_GEAR:
		_rpm = lerp(_rpm, IDLE_RPM, 0.1)
		return
	
	if not _is_clutch_engaged:
		# Engine revs freely when clutch disengaged
		_rpm = lerp(_rpm, _throttle_input * REDLINE_RPM, 0.1)
		return
	
	# Calculate theoretical RPM based on gear ratio and speed
	var wheel_rpm: float = (_speed_kmh / 3.6) / (2.0 * PI * _tire_radius) * 60.0
	var theoretical_rpm: float = wheel_rpm * _gear_ratios[_current_gear] * _final_drive_ratio
	
	# Smooth transition to target RPM
	_rpm = lerp(_rpm, theoretical_rpm, 0.05)
	
	# Enforce redline
	if _rpm >= SHUTDOWN_RPM:
		_rpm = SHUTDOWN_RPM
	
	# Clamp minimum RPM
	if _rpm < IDLE_RPM and _throttle_input > 0.1:
		_rpm = IDLE_RPM + (_throttle_input * (REDLINE_RPM - IDLE_RPM))

func _get_engine_torque(rpm: float) -> float:
	"""Get engine torque value at given RPM (simplified power curve)"""
	# Simplified torque curve peaking around 4500 RPM
	var normalized_rpm: float = (rpm - IDLE_RPM) / (REDLINE_RPM - IDLE_RPM)
	
	if normalized_rpm < 0.0 or normalized_rpm > 1.0:
		return 0.0
	
	# Quadratic torque curve (peak at ~0.55 normalized RPM)
	var peak_rpm_ratio: float = 0.55
	var torque_peak: float = 450.0  # Nm peak torque
	
	var torque: float = torque_peak * (1.0 - pow((normalized_rpm - peak_rpm_ratio) / 0.45, 2))
	
	return torque

func _check_auto_shifts(delta: float) -> void:
	"""Check if automatic gear shift should occur"""
	if _gear_shift_timer > 0:
		_gear_shift_timer -= delta
		return
	
	# Only auto-shift if manual override is not active
	if GameManager and GameManager.current_state != GameManager.GameState.GARAGE:
		return
	
	# Upshift if RPM exceeds shift point and throttle is high
	if _current_gear < MAX_GEAR and _rpm >= SHIFT_POINT_RPM and _throttle_input > 0.2:
		_shift_gear(_current_gear + 1)
	
	# Downshift if RPM drops below idle and throttle is low
	elif _current_gear > MIN_GEAR and _rpm < IDLE_RPM * 1.2 and _throttle_input < 0.1:
		_shift_gear(_current_gear - 1)

func _shift_gear(new_gear: int) -> void:
	"""Execute gear shift"""
	if new_gear < MIN_GEAR or new_gear > MAX_GEAR:
		return
	
	if new_gear == _current_gear:
		return
	
	# Disengage clutch
	_is_clutch_engaged = false
	_clutch_timer = CLUTCH_RELEASE_TIME
	
	var old_gear: int = _current_gear
	_target_gear = new_gear
	
	# Apply RPM drop on downshift
	if new_gear < old_gear:
		var rpm_drop: float = ENGINE_REVO_DROP_ON_DOWNSHIFT
		_rpm = max(_rpm - rpm_drop, IDLE_RPM)
	
	# Schedule gear change after clutch engagement
	await _wait_for_clutch_engagement()
	
	# Complete gear shift
	_current_gear = _target_gear
	_target_gear = NEUTRAL_GEAR
	_is_clutch_engaged = true
	
	# Emit signal
	gear_changed.emit(old_gear, _current_gear)

func _wait_for_clutch_engagement() -> void:
	"""Wait for clutch to engage during gear shift"""
	await get_tree().create_timer(CLUTCH_RELEASE_TIME).timeout

func _start_engine() -> void:
	"""Start the vehicle engine"""
	if _is_engine_running:
		return
	
	_is_engine_running = true
	_rpm = IDLE_RPM
	engine_stalled.emit()

func _stop_engine() -> void:
	"""Stop the vehicle engine"""
	_is_engine_running = false
	_rpm = IDLE_RPM
	engine_stalled.emit()

func _reset_vehicle() -> void:
	"""Reset vehicle to initial state"""
	_speed_kmh = 0.0
	_rpm = IDLE_RPM
	_current_gear = NEUTRAL_GEAR
	_throttle_input = 0.0
	_brake_input = 0.0
	_steering_input = 0.0
	_handbrake_input = 0.0
	_is_engine_running = false
	_is_clutch_engaged = true
	_is_drifting = false
	_drift_angle = 0.0
	_collisions.clear()

# ============================================================================
# DRIFT MECHANICS
# ============================================================================
func _update_drift(delta: float) -> void:
	"""Update drift state and calculate drift angle"""
	if _speed_kmh < _drift_velocity_threshold:
		_exit_drift()
		return
	
	# Detect drift conditions
	var drift_conditions: bool = _detect_drift_conditions()
	
	if drift_conditions and not _is_drifting:
		_enter_drift()
	elif not drift_conditions and _is_drifting:
		_exit_drift()
	
	if _is_drifting:
		_calculate_drift_angle(delta)
	else:
		# Recovery from drift
		_drift_angle = lerp(_drift_angle, 0.0, _drift_recovery_rate)

func _detect_drift_conditions() -> bool:
	"""Detect if vehicle is in drift condition"""
	# Check for excessive lateral velocity
	var lateral_velocity: float = abs(velocity.x)
	
	# Check steering angle vs direction
	var steering_angle: float = _steering_input * MAX_STEERING_ANGLE
	
	# Simple drift detection heuristic
	return (lateral_velocity > _drift_velocity_threshold * 0.5 and 
			abs(steering_angle) > 0.3 and 
			_throttle_input > 0.3)

func _enter_drift() -> void:
	"""Enter drift state"""
	_is_drifting = true
	_drift_angle = 0.0
	skidding.emit(true)
	drift_started.emit(_drift_angle)

func _exit_drift() -> void:
	"""Exit drift state"""
	_is_drifting = false
	skidding.emit(false)
	drift_ended.emit()

func _calculate_drift_angle(delta: float) -> void:
	"""Calculate drift angle based on vehicle dynamics"""
	# Drift angle increases with lateral velocity and steering
	var target_drift: float = (_steering_input * _speed_kmh / MAX_SPEED_KMH) * DRIFT_FACTOR
	
	# Smoothly interpolate to target
	_drift_angle = lerp(_drift_angle, target_drift, 0.05)

# ============================================================================
# COLLISION HANDLING
# ============================================================================
func _handle_collisions(delta: float) -> void:
	"""Process collision events and apply effects"""
	if _collisions.is_empty():
		return
	
	for collision in _collisions.duplicate():
		if collision.time + 0.5 < Time.get_ticks_msec() / 1000.0:
			_collisions.erase(collision)
			continue
		
		# Apply collision effects
		_apply_collision_effects(collision)

func _apply_collision_effects(collision: Dictionary) -> void:
	"""Apply visual and physical effects to collision"""
	# Screen shake
	if GameManager and GameManager.debug_mode:
		# Implement screen shake here
		pass
	
	# Damage calculation
	var damage: float = _calculate_collision_damage(collision)
	
	# Speed reduction
	_speed_kmh *= 0.8
	
	# Emit collision signal
	collision_detected.emit(collision)

func _calculate_collision_damage(collision: Dictionary) -> float:
	"""Calculate damage from collision"""
	var impact_velocity: float = collision.velocity.length()
	var impact_force: float = impact_velocity * _physics_settings.default_vehicle_mass
	
	# Scale damage based on impact force
	var damage: float = impact_force / 1000.0
	
	return min(damage, 100.0)

func _on_collision_entered(body: Node) -> void:
	"""Handle collision entry"""
	var collision_info: Dictionary = {
		"body": body,
		"time": Time.get_ticks_msec() / 1000.0,
		"velocity": velocity,
		"normal": move_and_slide_result.normal
	}
	
	_collisions.append(collision_info)
	_last_collision_time = collision_info["time"]

func _on_collision_exited(body: Node) -> void:
	"""Handle collision exit"""
	pass

# ============================================================================
# INPUT MANAGEMENT
# ============================================================================
func _smooth_inputs(delta: float) -> void:
	"""Smooth input transitions for better control feel"""
	# Throttle smoothing
	_throttle_input = lerp(_throttle_input, _read_throttle(), 0.1)
	
	# Brake smoothing
	_brake_input = lerp(_brake_input, _read_brake(), 0.1)
	
	# Steering smoothing
	_steering_input = lerp(_steering_input, _read_steering(), 0.15)
	
	# Handbrake smoothing
	_handbrake_input = lerp(_handbrake_input, _read_handbrake(), 0.2)

func _read_throttle() -> float:
	"""Read raw throttle input from InputManager"""
	if InputManager:
		return InputManager.get_axis("throttle_forward", "throttle_backward")
	return 0.0

func _read_brake() -> float:
	"""Read raw brake input from InputManager"""
	if InputManager:
		return InputManager.get_axis("brake", "brake_cancel")
	return 0.0

func _read_steering() -> float:
	"""Read raw steering input from InputManager"""
	if InputManager:
		return InputManager.get_axis("steering_left", "steering_right")
	return 0.0

func _read_handbrake() -> float:
	"""Read raw handbrake input from InputManager"""
	if InputManager:
		return InputManager.get_axis("handbrake", "handbrake_cancel")
	return 0.0

# ============================================================================
# SIGNAL EMISSION
# ============================================================================
func _emit_signal_updates() -> void:
	"""Emit signals when values change significantly"""
	if abs(_speed_kmh - _speed_kmh) > 1.0:
		speed_changed.emit(_speed_kmh)
	
	if abs(_rpm - _previous_rpm) > 100:
		rpm_changed.emit(_rpm)
	
	_previous_rpm = _rpm

# ============================================================================
# GAME MANAGER HOOKS
# ============================================================================
func _on_game_state_changed(new_state: GameState) -> void:
	"""Handle game state changes"""
	match new_state:
		GameState.RACE_ACTIVE:
			_start_engine()
		GameState.RACE_PAUSED:
			pass
		GameState.RACE_FINISHED:
			_stop_engine()
		_:
			pass

func _on_race_started(race_data: Dictionary) -> void:
	"""Handle race start event"""
	_reset_vehicle()
	_start_engine()

# ============================================================================
# DEBUG VISUALIZATION
# ============================================================================
func _update_debug_overlay() -> void:
	"""Update debug visualization overlays"""
	pass  # Implementation would add debug UI elements

func toggle_debug_mode(enabled: bool = true) -> void:
	"""Toggle debug mode on/off"""
	_debug_enabled = enabled
	_show_debug_overlay = enabled

# ============================================================================
# UTILITY METHODS
# ============================================================================
func reset_all_variables() -> void:
	"""Reset all vehicle variables to defaults"""
	_reset_vehicle()
	_is_traction_control_active = true
	_is_abs_active = true
	_is_drifting = false
	_drift_angle = 0.0

func set_gear(gear: int) -> void:
	"""Manually set gear"""
	if gear < MIN_GEAR or gear > MAX_GEAR:
		return
	_current_gear = gear

func set_traction_control(active: bool) -> void:
	"""Enable/disable traction control"""
	_is_traction_control_active = active
	traction_control_state_changed.emit(active)

func set_abs(active: bool) -> void:
	"""Enable/disable ABS"""
	_is_abs_active = active
	anti_lock_braking_state_changed.emit(active)

func set_handbrake(active: bool) -> void:
	"""Toggle handbrake"""
	_is_handbrake_active = active
	handbrake_toggled.emit(active)

func activate_drift() -> void:
	"""Force enter drift state"""
	_is_drifting = true
	_enter_drift()

func deactivate_drift() -> void:
	"""Force exit drift state"""
	_exit_drift()

</FILE>