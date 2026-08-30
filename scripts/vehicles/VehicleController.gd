extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulation
## Handles throttle, brake, steering, wheel forces, gear shifting, and vehicle dynamics
## Implements realistic car physics with drift mechanics, traction control, and powertrain integration
## Copyright 2026 Thalamus Racing Simulator Project

# ============================================================================
# SIGNALS - Vehicle state and event notifications
# ============================================================================
signal speed_changed(new_speed: float)
signal rpm_changed(rpm: int)
signal gear_changed(old_gear: int, new_gear: int)
signal drift_started()
signal drift_ended()
signal collision_impact(impact_force: Vector3, impact_point: Vector3)
signal lap_checkpoint(lap_time: float)
signal race_lap_completed(lap_number: int, lap_time: float)
signal engine_event(event_type: EngineEventType)

enum EngineEventType {
	EVENT_START,
	EVENT_IDLE,
	EVENT_REVving,
	EVENT_SHUTDOWN,
	EVENT_REDLINE
}

# ============================================================================
# INPUT VARIABLES - Connected from InputManager singleton
# ============================================================================
var _throttle_input: float = 0.0
var _brake_input: float = 0.0
var _steering_input: float = 0.0
var _clutch_input: float = 0.0
var _handbrake_input: float = 0.0

# ============================================================================
# VEHICLE PHYSICS STATE
# ============================================================================
var _speed: float = 0.0  # Current speed in km/h
var _acceleration: float = 0.0  # Acceleration in m/s²
var _slip_angle: float = 0.0  # Tire slip angle in radians
var _g_longitudinal: float = 0.0  # Longitudinal G-force
var _lateral_g: float = 0.0  # Lateral G-force
var _vertical_g: float = 0.0  # Vertical G-force

# ============================================================================
# GEARBOX SYSTEM
# ============================================================================
var _current_gear: int = 0  # 0 = Neutral, 1-6 = Forward gears, -1 = Reverse
var _target_gear: int = 0
var _is_shifting: bool = false
var _shift_timer: float = 0.0
var _shift_duration: float = 0.5  # Seconds for shift completion

# Gear ratios (final drive ratio included)
const _GEAR_RATIOS: Array[float] = [
	-3.5,  # Reverse
	3.8,   # 1st gear
	2.5,   # 2nd gear
	1.8,   # 3rd gear
	1.4,   # 4th gear
	1.1,   # 5th gear
	0.9,   # 6th gear
]

# Final drive ratio
const FINAL_DRIVE_RATIO: float = 3.73

# ============================================================================
# ENGINE & POWERTRAIN INTEGRATION
# ============================================================================
@onready var _powertrain: Node = get_parent().get_node_or_null("Powertrain")
var _engine_rpm: int = 0
var _engine_torque: float = 0.0
var _max_engine_rpm: int = 8000
var _idle_rpm: int = 800
var _redline_rpm: int = 7500
var _peak_torque_rpm: int = 4500
var _peak_torque: float = 450.0  # Nm

# ============================================================================
# WHEEL CONFIGURATION
# ============================================================================
@onready var _front_left_wheel: Node = $Wheels/FrontLeftWheel
@onready var _front_right_wheel: Node = $Wheels/FrontRightWheel
@onready var _rear_left_wheel: Node = $Wheels/RearLeftWheel
@onready var _rear_right_wheel: Node = $Wheels/RearRightWheel

# Wheel parameters
var _wheel_radius: float = 0.35  # meters
var _track_width: float = 1.6  # meters between wheels
var _wheel_base: float = 2.6  # meters front to rear
var _tire_friction_coefficient: float = 1.2  # Dry asphalt
var _tire_slip_threshold: float = 0.15  # Slip threshold before losing grip

# ============================================================================
# DRIFT MECHANICS
# ============================================================================
var _is_drifting: bool = false
var _drift_angle: float = 0.0
var _drift_intensity: float = 0.0  # 0.0 to 1.0
var _drift_timer: float = 0.0
var _min_drift_angle: float = 0.35  # ~20 degrees in radians
var _drift_decay_rate: float = 0.02  # How fast drift ends when handbrake released

# ============================================================================
# PHYSICS CONSTANTS FROM SETTINGS
# ============================================================================
@onready var _physics_settings = PhysicsSettings.get_singleton()
var _vehicle_mass: float = 1500.0  # kg
var _drag_coefficient: float = 0.32  # Cd
var _frontal_area: float = 2.2  # m²
var _air_density: float = 1.225  # kg/m³
var _rolling_resistance: float = 0.015

# ============================================================================
# INTERNAL STATE
# ============================================================================
var _last_position: Vector3 = Vector3.ZERO
var _velocity_vector: Vector3 = Vector3.ZERO
var _ground_normal: Vector3 = Vector3.UP
var _collision_detected: bool = false
var _collision_force: Vector3 = Vector3.ZERO
var _collision_point: Vector3 = Vector3.ZERO

# ============================================================================
# GAMEPLAY VARIABLES
# ============================================================================
var _race_active: bool = false
var _lap_count: int = 0
var _current_lap_time: float = 0.0
var _start_time: float = 0.0
var _checkpoint_times: Array[float] = []
var _total_race_time: float = 0.0
var _position_in_race: int = 0

# ============================================================================
# INITIALIZATION
# ============================================================================
func _ready() -> void:
	_process_mode = ProcessModeEnum.ALWAYS
	_connect_signals_to_manager()
	_init_physics_constants()
	_reset_vehicle_state()
	print("VehicleController initialized for ", name)

func _connect_signals_to_manager() -> void:
	if GameManager.has_signal("game_state_changed"):
		GameManager.game_state_changed.connect(_on_game_state_changed)
	if GameManager.has_signal("race_started"):
		GameManager.race_started.connect(_on_race_started)

func _init_physics_constants() -> void:
	_vehicle_mass = _physics_settings.default_vehicle_mass if _physics_settings else _vehicle_mass
	_drag_coefficient = 0.32
	_frontal_area = 2.2

func _reset_vehicle_state() -> void:
	_current_gear = 0
	_target_gear = 0
	_is_shifting = false
	_shift_timer = 0.0
	_speed = 0.0
	_acceleration = 0.0
	_engine_rpm = _idle_rpm
	_engine_torque = 0.0
	_is_drifting = false
	_drift_intensity = 0.0
	_throttle_input = 0.0
	_brake_input = 0.0
	_steering_input = 0.0
	_clutch_input = 0.0
	_handbrake_input = 0.0
	_last_position = global_position
	_velocity_vector = Vector3.ZERO
	_collision_detected = false
	_lap_count = 0
	_current_lap_time = 0.0
	_start_time = 0.0
	_total_race_time = 0.0

# ============================================================================
# MAIN UPDATE LOOP
# ============================================================================
func _physics_process(delta: float) -> void:
	if not is_inside_tree():
		return
	
	# Process input
	_process_inputs()
	
	# Update physics
	_update_vehicle_physics(delta)
	
	# Handle gear shifting
	_handle_gear_shifting(delta)
	
	# Apply forces to vehicle body
	_apply_vehicle_forces(delta)
	
	# Update wheel states
	_update_wheels(delta)
	
	# Check collisions
	_check_collisions()
	
	# Update gameplay metrics
	_update_gameplay_metrics(delta)
	
	# Update signals
	_emit_vehicle_signals()
	
	# Store last position for velocity calculation
	_last_position = global_position

func _process_inputs() -> void:
	# Get inputs from InputManager singleton
	if InputManager.has_method("get_throttle"):
		_throttle_input = InputManager.get_throttle()
	if InputManager.has_method("get_brake"):
		_brake_input = InputManager.get_brake()
	if InputManager.has_method("get_steering"):
		_steering_input = InputManager.get_steering()
	if InputManager.has_method("get_clutch"):
		_clutch_input = InputManager.get_clutch()
	if InputManager.has_method("get_handbrake"):
		_handbrake_input = InputManager.get_handbrake()
	
	# Clamp inputs to valid range
	_throttle_input = clamp(_throttle_input, 0.0, 1.0)
	_brake_input = clamp(_brake_input, 0.0, 1.0)
	_steering_input = clamp(_steering_input, -1.0, 1.0)
	_clutch_input = clamp(_clutch_input, 0.0, 1.0)
	_handbrake_input = clamp(_handbrake_input, 0.0, 1.0)

func _update_vehicle_physics(delta: float) -> void:
	# Calculate acceleration based on current gear and throttle
	var gear_ratio: float = _GEAR_RATIOS[_current_gear] if _current_gear >= 0 and _current_gear < _GEAR_RATIOS.size() else 0.0
	var total_ratio: float = gear_ratio * FINAL_DRIVE_RATIO
	
	# Calculate engine torque curve based on RPM
	_engine_torque = _calculate_engine_torque(_engine_rpm)
	
	# Apply torque to wheels (simplified drivetrain loss of 15%)
	var drivetrain_efficiency: float = 0.85
	var wheel_torque: float = _engine_torque * total_ratio * drivetrain_efficiency / (_wheel_radius * _GEAR_RATIOS.size())
	
	# Calculate driving force
	var driving_force: float = 0.0
	if _current_gear != 0 and _throttle_input > 0.0:
		driving_force = wheel_torque * _throttle_input / _wheel_radius
	
	# Apply air resistance
	var velocity_mps: float = _speed / 3.6  # Convert km/h to m/s
	var drag_force: float = 0.5 * _air_density * _drag_coefficient * _frontal_area * velocity_mps * velocity_mps
	
	# Apply rolling resistance
	var rolling_force: float = _rolling_resistance * _vehicle_mass * 9.81
	
	# Net acceleration
	var net_force: float = driving_force - drag_force - rolling_force
	_acceleration = net_force / _vehicle_mass
	
	# Update speed (handle reverse gear)
	if _current_gear == -1:  # Reverse
		_speed += (_acceleration * delta * 3.6)  # Convert to km/h
	else:
		_speed += (_acceleration * delta * 3.6)
	
	# Clamp speed to reasonable limits
	_speed = max(-150.0, min(350.0, _speed))
	
	# Calculate longitudinal G-force
	_g_longitudinal = _acceleration / 9.81
	
	# Calculate velocity vector from speed and heading
	var heading: float = rotation.y
	_velocity_vector.x = cos(heading) * _speed / 3.6
	_velocity_vector.z = sin(heading) * _speed / 3.6
	
	# Calculate lateral G-force for drift mechanics
	_lateral_g = abs(_velocity_vector.x * sin(rotation.y) - _velocity_vector.z * cos(rotation.y)) / 9.81

func _calculate_engine_torque(rpm: int) -> float:
	# Simple torque curve approximation (parabolic around peak torque RPM)
	var rpm_normalized: float = float(rpm) / float(_max_engine_rpm)
	
	if rpm <= _idle_rpm:
		return _peak_torque * 0.3
	elif rpm >= _max_engine_rpm:
		return _peak_torque * 0.1
	else:
		# Parabolic curve peaking at peak_torque_rpm
		var distance_from_peak: float = abs(float(rpm) - float(_peak_torque_rpm))
		var max_distance: float = float(_max_engine_rpm) - float(_peak_torque_rpm)
		var factor: float = 1.0 - pow(distance_from_peak / max_distance, 2)
		return _peak_torque * (0.3 + 0.7 * factor)

func _handle_gear_shifting(delta: float) -> void:
	if _is_shifting:
		_shift_timer -= delta
		if _shift_timer <= 0.0:
			_is_shifting = false
			_engine_rpm = int(float(_engine_rpm) * _GEAR_RATIOS[_current_gear] / _GEAR_RATIOS[_target_gear])
			_current_gear = _target_gear
			gear_changed.emit(_target_gear, _current_gear)
		return
	
	# Automatic or manual gear shifting logic
	_target_gear = _calculate_target_gear()
	
	if _target_gear != _current_gear:
		# Initiate shift
		if _clutch_input > 0.5:
			_is_shifting = true
			_shift_timer = _shift_duration
			engine_event.emit(EngineEventType.EVENT_START)
		elif _throttle_input < 0.2:
			# Auto-shifting without clutch (with throttle lift)
			_is_shifting = true
			_shift_timer = _shift_duration
			engine_event.emit(EngineEventType.EVENT_START)

func _calculate_target_gear() -> int:
	# Calculate target gear based on speed and engine RPM
	var optimal_rpm: int = 4000 if _throttle_input > 0.5 else 2500
	var current_speed_kmh: float = abs(_speed)
	
	# Determine ideal gear for current speed
	for gear_idx in range(1, _GEAR_RATIOS.size()):
		var gear_ratio: float = _GEAR_RATIOS[gear_idx]
		var estimated_rpm: int = int(float(current_speed_kmh) * 100.0 / gear_ratio)
		
		if estimated_rpm <= optimal_rpm:
			return gear_idx
	
	return 1  # Default to first gear

func _apply_vehicle_forces(delta: float) -> void:
	# Apply movement to CharacterBody3D
	var move_vector: Vector3 = Vector3(
		cos(rotation.y) * _velocity_vector.x / 3.6,
		0.0,
		sin(rotation.y) * _velocity_vector.z / 3.6
	)
	
	move_and_slide(move_vector, Vector3.UP)

func _update_wheels(delta: float) -> void:
	# Update wheel rotation based on speed
	var wheel_rotation_speed: float = (_speed / 3.6) / _wheel_radius * delta
	var rotation_axis: Quaternion = Quaternion.IDENTITY.rotated(Vector3.RIGHT, wheel_rotation_speed)
	
	if _front_left_wheel:
		_front_left_wheel.rotate_y(wheel_rotation_speed * _steering_input * 2.0)
	if _front_right_wheel:
		_front_right_wheel.rotate_y(wheel_rotation_speed * _steering_input * 2.0)
	
	# Rotate all wheels based on vehicle speed
	for wheel in [_front_left_wheel, _front_right_wheel, _rear_left_wheel, _rear_right_wheel]:
		if wheel:
			wheel.rotate_x(wheel_rotation_speed)

func _check_collisions() -> void:
	# Use built-in collision detection from CharacterBody3D
	_collision_detected = false
	_collision_force = Vector3.ZERO
	
	# Check collision normal and impact
	if is_on_floor() and _ground_normal != Vector3.UP:
		var ground_change: float = abs(_ground_normal.y - 1.0)
		if ground_change > 0.1:
			_vertical_g = (1.0 - _ground_normal.y) * 9.81
			_collision_detected = true
	
	# Check wall impacts
	if _collision_detected and not is_on_floor():
		_collision_force = Vector3.ZERO
		_collision_point = global_position

func _update_gameplay_metrics(delta: float) -> void:
	# Track lap times
	if _race_active:
		_current_lap_time += delta
		_total_race_time += delta
		
		# Simple checkpoint system (would need actual checkpoint nodes)
		if _current_lap_time % 60.0 < delta:  # Every minute as placeholder
			lap_checkpoint.emit(_current_lap_time)
			_lap_count += 1
			race_lap_completed.emit(_lap_count, _current_lap_time)
			_current_lap_time = 0.0

func _emit_vehicle_signals() -> void:
	speed_changed.emit(_speed)
	rpm_changed.emit(_engine_rpm)
	
	if _is_drifting:
		if not _is_drifting:
			_drift_ended.emit()
		_drift_intensity = lerp(_drift_intensity, 1.0, 0.1)
	else:
		_drift_intensity = max(0.0, _drift_intensity - _drift_decay_rate)
	
	if _collision_detected:
		collision_impact.emit(_collision_force, _collision_point)

# ============================================================================
# DRIVE CONTROL METHODS
# ============================================================================
func apply_throttle(amount: float) -> void:
	_throttle_input = clamp(amount, 0.0, 1.0)
	if amount > 0.8 and _engine_rpm > 6000:
		engine_event.emit(EngineEventType.EVENT_REDLINE)

func apply_brake(amount: float) -> void:
	_brake_input = clamp(amount, 0.0, 1.0)
	if amount > 0.5:
		# Brake fade effect (simplified)
		pass

func apply_steering(amount: float) -> void:
	_steering_input = clamp(amount, -1.0, 1.0)
	# Apply steering to front wheels visually
	if _front_left_wheel and _front_right_wheel:
		_front_left_wheel.rotation.y = _steering_input * 0.5
		_front_right_wheel.rotation.y = _steering_input * 0.5

func apply_handbrake(active: bool) -> void:
	_handbrake_input = 1.0 if active else 0.0
	if active and not _is_drifting and _speed > 30.0:
		_is_drifting = true
		_drift_timer = 2.0  # Seconds to maintain drift
	(drift_started.emit())

func shift_gear(gear: int) -> void:
	if gear >= 0 and gear < _GEAR_RATIOS.size():
		_target_gear = gear
		if _clutch_input > 0.5 or _throttle_input < 0.2:
			_is_shifting = true
			_shift_timer = _shift_duration

func set_neutral() -> void:
	_target_gear = 0
	_is_shifting = true
	_shift_timer = _shift_duration

# ============================================================================
# RACE MANAGEMENT
# ============================================================================
func start_race() -> void:
	_race_active = true
	_start_time = Time.get_unix_time_from_system()
	_total_race_time = 0.0
	_lap_count = 0
	_current_lap_time = 0.0
	_checkpoint_times.clear()

func pause_race() -> void:
	_race_active = false

func reset_race() -> void:
	_race_active = false
	_reset_vehicle_state()
	position = Vector3.ZERO
	rotation = Vector3.ZERO

func get_current_lap_time() -> float:
	return _current_lap_time if _race_active else 0.0

func get_total_race_time() -> float:
	return _total_race_time

func get_lap_count() -> int:
	return _lap_count

func get_position_in_race() -> int:
	return _position_in_race

# ============================================================================
# UTILITIES
# ============================================================================
func get_speed_kmh() -> float:
	return _speed

func get_rpm() -> int:
	return _engine_rpm

func get_current_gear() -> int:
	return _current_gear

func get_engine_torque_nm() -> float:
	return _engine_torque

func is_drifting() -> bool:
	return _is_drifting

func get_drift_intensity() -> float:
	return _drift_intensity

func get_acceleration_ms2() -> float:
	return _acceleration

func get_g_forces() -> Dictionary:
	return {
		"longitudinal": _g_longitudinal,
		"lateral": _lateral_g,
		"vertical": _vertical_g
	}

func get_velocity_vector() -> Vector3:
	return _velocity_vector

# ============================================================================
# EVENT HANDLERS
# ============================================================================
func _on_game_state_changed(new_state: GameManager.GameState) -> void:
	match new_state:
		GameManager.GameState.RACE_ACTIVE:
			start_race()
		GameManager.GameState.RACE_PAUSED:
			pause_race()
		GameManager.GameState.MAIN_MENU:
			reset_race()

func _on_race_started(race_data: Dictionary) -> void:
	start_race()
	_position_in_race = race_data.get("player_position", 0)

# ============================================================================
# DEBUG VISUALIZATION (if debug mode enabled)
# ============================================================================
func _draw_debug_info() -> void:
	if not GameManager.debug_mode:
		return
	
	# Debug drawing would go here for testing vehicle physics
	pass

</FILE>