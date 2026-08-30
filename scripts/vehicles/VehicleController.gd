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
@export var engine_torque: float = 450.0
@export var transmission_ratio: float = 3.5
@export var differential_ratio: float = 3.8
@export var wheel_radius: float = 0.35
@export var track_width: float = 1.6
@export var wheelbase: float = 2.5
@export var center_of_mass_height: float = 0.5
@export var drag_coefficient: float = 0.32
@export var frontal_area: float = 2.2
@export var tire_friction: float = 1.2
@export var air_resistance_factor: float = 0.5

@export_group("Steering & Suspension")
@export var max_steering_angle: float = 45.0
@export var steering_speed: float = 2.0
@export var spring_stiffness: float = 50000.0
@export var damper_damping: float = 5000.0
@export var suspension_travel: float = 0.2

@export_group("Drivetrain")
@export var drivetrain_type: String = "RWD"
@export var torque_curve: Array[float] = []
@export var shift_points: Array[int] = [1500, 3000, 4500, 5500, 6500, 7200, 7500]
@export var has_turbo: bool = false
@export var turbo_multiplier: float = 1.3

# ============================================================================
# PRIVATE VARIABLES - Internal State Tracking
# ============================================================================
var _current_speed_kmh: float = 0.0
var _current_rpm: float = IDLE_RPM
var _current_gear: int = 0
var _throttle_input: float = 0.0
var _brake_input: float = 0.0
var _steering_input: float = 0.0
var _clutch_pressed: bool = false
var _turbo_charging: bool = false
var _drift_mode: bool = false
var _drift_intensity: float = 0.0
var _turbo_timer: float = 0.0
var _last_collision_time: float = 0.0
var _suspension_states: Array[Dictionary] = []

# Physical properties
var _velocity_vector: Vector3 = Vector3.ZERO
var _angular_velocity: Vector3 = Vector3.ZERO
var _acceleration_vector: Vector3 = Vector3.ZERO
var _drag_force: float = 0.0
var _downforce: float = 0.0

# Gear ratios (1st through 6th + reverse)
var _gear_ratios: Array[float] = [3.8, 2.5, 1.7, 1.3, 1.0, 0.85, 3.5]

# Wheel positions relative to chassis
var _wheel_positions: Array[Vector3] = []
var _wheel_forces: Array[float] = []

# Input state tracking
var _input_buffer: Dictionary = {}
var _gear_shift_pending: bool = false
var _auto_shift_active: bool = true

# Drift mechanics
var _drift_counter: float = 0.0
var _traction_loss: float = 0.0
var _lateral_velocity: float = 0.0

# ============================================================================
# LIFECYCLE - Initialization
# ============================================================================
func _ready() -> void:
	_init_wheel_positions()
	_init_suspension_states()
	_connect_signals()
	_apply_default_torque_curve()

func _init_wheel_positions() -> void:
	"""Initialize wheel position offsets from chassis center"""
	var half_track: float = track_width / 2.0
	var half_wheelbase: float = wheelbase / 2.0
	
	_wheel_positions = [
		Vector3(-half_track, -center_of_mass_height, -half_wheelbase),  # Front Left
		Vector3(half_track, -center_of_mass_height, -half_wheelbase),   # Front Right
		Vector3(-half_track, -center_of_mass_height, half_wheelbase),   # Rear Left
		Vector3(half_track, -center_of_mass_height, half_wheelbase)     # Rear Right
	]
	
	# Initialize wheel force values
	for _i in range(4):
		_wheel_forces.append(0.0)

func _init_suspension_states() -> void:
	"""Initialize suspension compression states for each wheel"""
	for _i in range(4):
		_suspension_states.append({
			"compression": 0.0,
			"velocity": 0.0,
			"spring_force": 0.0,
			"damping_force": 0.0
		})

func _connect_signals() -> void:
	"""Connect game manager signals"""
	if GameManager:
		GameManager.game_state_changed.connect(_on_game_state_changed)

func _apply_default_torque_curve() -> void:
	"""Apply default torque curve if not provided"""
	if torque_curve.is_empty():
		torque_curve = [0.3, 0.6, 0.85, 1.0, 0.95, 0.8, 0.6]

# ============================================================================
# INPUT HANDLING - Player Controls
# ============================================================================
func _physics_process(delta: float) -> void:
	_process_input(delta)
	_update_physics(delta)
	_handle_drift(delta)
	_update_audio(delta)
	_check_collisions(delta)
	_update_suspension(delta)
	_move_character(delta)

func _process_input(delta: float) -> void:
	"""Process raw input from InputManager"""
	var input_manager := InputManager if InputManager else get_tree().get_first_node_in_group("InputManager")
	
	if input_manager:
		_throttle_input = clamp(input_manager.get_axis("throttle"), -0.1, 1.0)
		_brake_input = clamp(input_manager.get_axis("brake"), -0.1, 1.0)
		_steering_input = clamp(input_manager.get_axis("steer_left_right"), -1.0, 1.0)
		_clutch_pressed = input_manager.is_action_pressed("clutch")
		
		# Auto-shift logic
		if _auto_shift_active and not _clutch_pressed:
			_auto_shift_gear()

func _update_physics(delta: float) -> void:
	"""Update vehicle physics calculations"""
	_calculate_engine_output()
	_apply_forces(delta)
	_calculate_drag_and_downforce(delta)
	_apply_suspension_forces()
	_update_velocity(delta)

func _calculate_engine_output() -> void:
	"""Calculate current RPM and power output based on gear and throttle"""
	var gear_ratio: float = _gear_ratios[_current_gear] if _current_gear >= 0 and _current_gear < _gear_ratios.size() else 1.0
	
	# Calculate target RPM based on current speed and gear
	var wheel_rotation_speed: float = (_current_speed_kmh * 1000.0 / 3600.0) / (2.0 * PI * wheel_radius)
	var engine_target_rpm: float = wheel_rotation_speed * gear_ratio * transmission_ratio * differential_ratio * 60.0
	
	# Apply clutch effect
	if _clutch_pressed:
		engine_target_rpm = IDLE_RPM
	
	# Smooth RPM transition
	_current_rpm = lerp(_current_rpm, engine_target_rpm, delta * 10.0)
	
	# Clamp RPM to valid range
	_current_rpm = clamp(_current_rpm, IDLE_RPM, REDLINE_RPM)
	
	# Get torque multiplier from curve
	var torque_index: float = linear_map(_current_rpm, IDLE_RPM, REDLINE_RPM, 0.0, float(torque_curve.size() - 1))
	var torque_mult: float = _interpolate_torque_curve(torque_index)
	
	# Calculate actual torque output
	var base_torque: float = engine_torque * torque_mult
	var final_torque: float = base_torque
	
	# Apply turbo boost if charging
	if has_turbo and _turbo_charging:
		final_torque *= turbo_multiplier
	
	# Apply gear-specific power delivery
	if _current_gear == 0:
		final_torque *= 0.7  # Reverse gear reduced power
	elif _current_gear > 0:
		var acceleration_gain: float = _gear_ratios[_current_gear] / _gear_ratios[0]
		final_torque *= acceleration_gain
	
	# Calculate wheel force
	var wheel_force: float = (final_torque * transmission_ratio * differential_ratio) / wheel_radius
	
	# Distribute force based on drivetrain type
	_set_wheel_distribution(wheel_force)
	
	# Emit signals
	rpm_changed.emit(_current_rpm)
	gear_changed.emit(_current_gear)
	engine_sound_changed.emit((_current_rpm - IDLE_RPM) / (REDLINE_RPM - IDLE_RPM))

func _set_wheel_distribution(wheel_force: float) -> void:
	"""Distribute wheel force based on drivetrain configuration"""
	match drivetrain_type:
		"FWD":
			_wheel_forces[0] = wheel_force * 0.5
			_wheel_forces[1] = wheel_force * 0.5
			_wheel_forces[2] = 0.0
			_wheel_forces[3] = 0.0
		"RWD":
			_wheel_forces[0] = 0.0
			_wheel_forces[1] = 0.0
			_wheel_forces[2] = wheel_force * 0.5
			_wheel_forces[3] = wheel_force * 0.5
		"AWD":
			_wheel_forces[0] = wheel_force * 0.35
			_wheel_forces[1] = wheel_force * 0.35
			_wheel_forces[2] = wheel_force * 0.15
			_wheel_forces[3] = wheel_force * 0.15
		_:  # Default to RWD
			_wheel_forces[0] = 0.0
			_wheel_forces[1] = 0.0
			_wheel_forces[2] = wheel_force * 0.5
			_wheel_forces[3] = wheel_force * 0.5

func _apply_forces(delta: float) -> void:
	"""Apply calculated forces to vehicle body"""
	var forward_direction: Vector3 = global_transform.basis.z
	var total_drive_force: float = sum(_wheel_forces)
	
	# Apply drive force
	_acceleration_vector += forward_direction * total_drive_force / vehicle_mass
	
	# Apply braking force
	if _brake_input > 0:
		var brake_force: float = BRAKING_FORCE * _brake_input * vehicle_mass
		_acceleration_vector -= forward_direction * brake_force / vehicle_mass
	
	# Apply steering rotation
	if abs(_steering_input) > 0.01:
		var steer_angle: float = deg_to_rad(max_steering_angle * _steering_input)
		_angular_velocity.y = steer_angle * TURN_SPEED * delta

func _calculate_drag_and_downforce(delta: float) -> void:
	"""Calculate aerodynamic effects"""
	var speed_ms: float = _current_speed_kmh / 3.6
	
	# Air resistance: F = 0.5 * Cd * A * ρ * v²
	var air_density: float = 1.225  # kg/m³ at sea level
	_drag_force = 0.5 * drag_coefficient * frontal_area * air_density * pow(speed_ms, 2)
	
	# Apply drag opposite to velocity
	_drag_force *= -1.0
	
	# Downforce increases with speed squared
	_downforce = 0.5 * drag_coefficient * frontal_area * air_density * pow(speed_ms, 2) * 0.3
	
	# Add downforce to normal force calculation (affects traction)

func _apply_suspension_forces() -> void:
	"""Apply suspension forces based on wheel compression"""
	for i in range(4):
		var state: Dictionary = _suspension_states[i]
		var wheel_position: Vector3 = _wheel_positions[i]
		
		# Calculate spring force: F = -kx
		var spring_force: float = -spring_stiffness * state["compression"]
		state["spring_force"] = spring_force
		
		# Calculate damping force: F = -cv
		var damping_force: float = -damper_damping * state["velocity"]
		state["damping_force"] = damping_force
		
		# Total suspension force
		var total_suspension_force: float = spring_force + damping_force
		
		# Apply to vehicle body (simplified)
		_acceleration_vector.y += total_suspension_force / vehicle_mass

func _update_velocity(delta: float) -> void:
	"""Update velocity based on acceleration"""
	# Apply acceleration
	_velocity_vector += _acceleration_vector * delta
	
	# Apply drag deceleration
	if _drag_force != 0:
		_velocity_vector *= 1.0 - (_drag_force / vehicle_mass) * delta
	
	# Update speed in km/h
	_current_speed_kmh = _velocity_vector.length() * 3.6
	
	# Update angular velocity
	_transform.origin += _velocity_vector * delta
	_rotate_y(_angular_velocity.y * delta)
	
	# Reset acceleration for next frame
	_acceleration_vector = Vector3.ZERO
	_angular_velocity.y = 0.0
	
	# Emit speed signal
	speed_changed.emit(_current_speed_kmh)

func _handle_drift(delta: float) -> void:
	"""Handle drift mechanics when lateral forces exceed threshold"""
	_lateral_velocity = _velocity_vector.x  # Simplified lateral calculation
	
	# Check if drifting
	if abs(_lateral_velocity) > DRIFT_THRESHOLD and _current_speed_kmh > 30.0:
		_drift_counter += delta
		_drift_intensity = min(_drift_counter * 2.0, DRIFT_INTENSITY_MAX)
		
		if _drift_counter > 0.5:
			if not _drift_mode:
				_drift_mode = true
				drift_started.emit(_drift_intensity)
		
		# Reduce traction during drift
		_traction_loss = _drift_intensity * 0.5
	else:
		_drift_counter = max(0.0, _drift_counter - delta * 3.0)
		
		if _drift_counter < 0.1 and _drift_mode:
			_drift_mode = false
			drift_ended.emit()
		
		_traction_loss = 0.0

func _update_audio(delta: float) -> void:
	"""Update audio parameters based on vehicle state"""
	if AudioManager:
		var rpm_ratio: float = (_current_rpm - IDLE_RPM) / (REDLINE_RPM - IDLE_RPM)
		AudioManager.set_bus_volume("Engine", map_range(rpm_ratio, 0.0, 1.0, 0.0, 0.8))

func _check_collisions(delta: float) -> void:
	"""Check for collisions and handle them"""
	if colliding:
		var now: float = Time.get_ticks_msec() / 1000.0
		if now - _last_collision_time > 0.1:
			_last_collision_time = now
			
			var collision_info: Dictionary = {
				"normal": collision_normal,
				"position": collision_point,
				"speed": _current_speed_kmh,
				"timestamp": now
			}
			
			collision_detected.emit(collision_info)
			_handle_collision_impact(collision_info)

func _handle_collision_impact(collision_info: Dictionary) -> void:
	"""Handle collision impact effects"""
	var impact_force: float = collision_info["speed"] * vehicle_mass * 0.1
	
	# Apply bounce
	_velocity_vector *= 0.5
	
	# Screen shake effect (if camera exists)
	var camera: Camera3D = get_viewport().get_camera_3d()
	if camera:
		camera.shake_screen(impact_force * 0.01, 0.2)

func _update_suspension(delta: float) -> void:
	"""Update suspension compression based on vertical forces"""
	for i in range(4):
		var state: Dictionary = _suspension_states[i]
		
		# Simple vertical force approximation
		var vertical_force: float = gravity * vehicle_mass / 4.0
		
		# Update compression
		state["compression"] = clamp(state["compression"] + (vertical_force - state["spring_force"]) * delta * 0.01, 
			-suspension_travel, suspension_travel)
		
		# Update velocity
		state["velocity"] = state["compression"] * 10.0
		
		# Track maximum compression for signal
		if abs(state["compression"]) > abs(suspension_states[i]["max_compression"]):
			suspension_compressed.emit(abs(state["compression"]))

func _move_character(delta: float) -> void:
	"""Move character body with calculated velocity"""
	move_and_slide()

# ============================================================================
# GEAR SHIFTER LOGIC
# ============================================================================
func auto_shift_gear(up: bool = true) -> void:
	"""Shift gears automatically or manually"""
	if _clutch_pressed:
		return
	
	var target_gear: int = _current_gear + (1 if up else -1)
	
	# Boundary check
	target_gear = clamp(target_gear, MIN_GEAR, MAX_GEAR)
	
	# Shift with delay for realism
	if _current_gear != target_gear:
		_current_gear = target_gear
		_gear_shift_pending = true
		
		# Simulate clutch engagement
		await get_tree().create_timer(CLUTCH_RELEASE_TIME).timeout
		_gear_shift_pending = false

func manual_shift_up() -> void:
	"""Manual gear up command"""
	auto_shift_gear(true)

func manual_shift_down() -> void:
	"""Manual gear down command"""
	auto_shift_gear(false)

func _auto_shift_gear() -> void:
	"""Automatic gear shifting based on RPM"""
	if _current_rpm >= SHIFT_RPM and _current_gear < MAX_GEAR:
		_current_gear += 1
		gear_changed.emit(_current_gear)
	elif _current_rpm <= IDLE_RPM and _current_gear > MIN_GEAR:
		_current_gear -= 1
		gear_changed.emit(_current_gear)

func set_gear(gear: int) -> void:
	"""Set gear directly (for AI or debug)"""
	_current_gear = clamp(gear, MIN_GEAR, MAX_GEAR)
	gear_changed.emit(_current_gear)

func get_current_gear() -> int:
	"""Get current gear number"""
	return _current_gear

func get_current_speed() -> float:
	"""Get current speed in km/h"""
	return _current_speed_kmh

func get_current_rpm() -> float:
	"""Get current engine RPM"""
	return _current_rpm

# ============================================================================
# TURBO SYSTEM
# ============================================================================
func activate_turbo() -> void:
	"""Activate turbo boost"""
	if has_turbo and not _turbo_charging:
		_turbo_charging = true
		_turbo_timer = TURBO_CHARGE_TIME
		race_event.emit("turbo_activate", {"vehicle_id": name})

func deactivate_turbo() -> void:
	"""Deactivate turbo boost"""
	_turbo_charging = false

func update_turbo(delta: float) -> void:
	"""Update turbo timer and charge state"""
	if has_turbo:
		if _turbo_charging:
			_turbo_timer -= delta
			if _turbo_timer <= 0:
				deactivate_turbo()

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================
func _interpolate_torque_curve(index: float) -> float:
	"""Interpolate torque value from curve array"""
	if torque_curve.is_empty():
		return 1.0
	
	var low_idx: int = floor(index)
	var high_idx: int = ceil(index)
	
	if high_idx >= torque_curve.size():
		return torque_curve[torque_curve.size() - 1]
	
	var t: float = index - low_idx
	return lerp(torque_curve[low_idx], torque_curve[high_idx], t)

func _lerp_value(value: float, min_val: float, max_val: float, out_min: float, out_max: float) -> float:
	"""Linear interpolation helper"""
	return ((value - min_val) / (max_val - min_val)) * (out_max - out_min) + out_min

func _reset_vehicle() -> void:
	"""Reset vehicle to starting state"""
	_current_speed_kmh = 0.0
	_current_rpm = IDLE_RPM
	_current_gear = 0
	_velocity_vector = Vector3.ZERO
	_acceleration_vector = Vector3.ZERO
	_angular_velocity = Vector3.ZERO
	_throttle_input = 0.0
	_brake_input = 0.0
	_steering_input = 0.0
	_drift_mode = false
	_drift_intensity = 0.0

# ============================================================================
# DEBUG & TESTING
# ============================================================================
func _on_game_state_changed(new_state: GameState) -> void:
	"""Handle game state changes affecting vehicle"""
	match new_state:
		GameManager.GameState.LOADING:
			_reset_vehicle()
		GameManager.GameState.RACE_ACTIVE:
			pass  # Ready for action
		GameManager.GameState.RACE_PAUSED:
			set_process(false)
		GameManager.GameState.RACE_FINISHED:
			_reset_vehicle()

func debug_get_stats() -> Dictionary:
	"""Return current vehicle statistics for debugging"""
	return {
		"speed_kmh": _current_speed_kmh,
		"rpm": _current_rpm,
		"gear": _current_gear,
		"throttle": _throttle_input,
		"brake": _brake_input,
		"steering": _steering_input,
		"drift_intensity": _drift_intensity,
		"turbo_charging": _turbo_charging,
		"velocity": _velocity_vector,
		"acceleration": _acceleration_vector
	}

</file_content>