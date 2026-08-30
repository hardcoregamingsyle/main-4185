@export_group("Drivetrain Settings")
@export var drivetrain_type: DrivetrainType = DrivetrainType.RWD
@export var final_drive_ratio: float = 3.5: set = _set_final_drive_ratio
@export var tire_radius: float = 0.33: set = _set_tire_radius
@export var torque_curve: Array[Vector2] = [
	Vector2(0.0, 0.0), Vector2(1000.0, 0.6), Vector2(3000.0, 0.9), 
	Vector2(5000.0, 1.0), Vector2(7000.0, 0.85), Vector2(9000.0, 0.6)
]: set = _set_torque_curve

@export_group("Advanced Physics")
@export var aerodynamic_drag_coefficient: float = 0.32
@export var frontal_area: float = 2.2
@export var rolling_resistance_coefficient: float = 0.015
@export var grip_level: float = 1.0: set = _set_grip_level

# ============================================================================
# PRIVATE VARIABLES
# ============================================================================
var _current_speed_kmh: float = 0.0
var _current_rpm: float = 0.0
var _current_gear: int = 1
var _max_gears: int = 6
var _is_engine_running: bool = false
var _is_handbrake_active: bool = false
var _is_traction_control_active: bool = true

var _input_throttle: float = 0.0
var _input_brake: float = 0.0
var _input_steering: float = 0.0
var _input_gear_up: bool = false
var _input_gear_down: bool = false

var _wheel_forces: Array[Vector3] = []
var _wheel_angles: Array[float] = []
var _wheel_slip_ratios: Array[float] = []

var _torque_output: float = 0.0
var _engine_force: float = 0.0
var _air_resistance: float = 0.0
var _rolling_resistance: float = 0.0

var _drift_angle: float = 0.0
var _drifting: bool = false

var _powertrain_node: Node = null

# ============================================================================
# GETTERS & SETTERS
# ============================================================================
func get_current_speed() -> float: return _current_speed_kmh
func get_current_rpm() -> float: return _current_rpm
func get_current_gear() -> int: return _current_gear
func get_is_engine_running() -> bool: return _is_engine_running
func get_is_drifting() -> bool: return _drifting
func get_input_throttle() -> float: return _input_throttle
func get_input_brake() -> float: return _input_brake
func get_input_steering() -> float: return _input_steering

# ============================================================================
# PROPERTY SETTERS WITH VALIDATION
# ============================================================================
func _set_vehicle_mass(value: float) -> void:
	vehicle_mass = value if value > 0 else VEHICLE_BASE_MASS
	max_speed_kmh = max_speed_kmh if vehicle_mass > 0 else 0.0
	max_acceleration_force = acceleration_force * (VEHICLE_BASE_MASS / vehicle_mass)

func _set_max_speed_kmh(value: float) -> void:
	max_speed_kmh = value if value > 0 else 0.0

func _set_final_drive_ratio(value: float) -> void:
	final_drive_ratio = value if value > 0 else 3.5

func _set_tire_radius(value: float) -> void:
	tire_radius = value if value > 0.1 else 0.33

func _set_torque_curve(value: Array[Vector2]) -> void:
	if value.size() > 0:
		torque_curve = value
	else:
		torque_curve = [Vector2(0.0, 0.0), Vector2(5000.0, 1.0)]

func _set_grip_level(value: float) -> void:
	grip_level = clamp(value, 0.3, 1.5)

# ============================================================================
# PUBLIC METHODS
# ============================================================================
func start_engine() -> void:
	if not _is_engine_running:
		_is_engine_running = true
		_current_rpm = 800.0  # Idle RPM
		engine_started.emit()

func stop_engine() -> void:
	if _is_engine_running:
		_is_engine_running = false
		_current_rpm = 0.0
		_current_gear = 0
		engine_stopped.emit()

func toggle_handbrake(is_active: bool) -> void:
	_is_handbrake_active = is_active
	handbrake_toggled.emit(is_active)

func shift_gear(new_gear: int) -> void:
	var old_gear: int = _current_gear
	_current_gear = clamp(new_gear, 0, _max_gears)
	
	if old_gear != _current_gear:
		gear_changed.emit(old_gear, _current_gear)
		
		# Downshift protection
		if _current_gear == 0 and _current_speed_kmh < 5.0:
			stop_engine()

func upshift() -> void:
	if _current_gear < _max_gears:
		shift_gear(_current_gear + 1)

func downshift() -> void:
	if _current_gear > 0:
		shift_gear(_current_gear - 1)

func reset() -> void:
	_current_speed_kmh = 0.0
	_current_rpm = 0.0
	_current_gear = 0
	_is_engine_running = false
	_drift_angle = 0.0
	_drifting = false
	_wheel_forces.clear()
	_wheel_angles.clear()
	_wheel_slip_ratios.clear()
	_velocity = Vector3.ZERO
	angular_velocity = Vector3.ZERO

# ============================================================================
# PHYSICS UPDATE METHOD
# ============================================================================
func _physics_process(delta: float) -> void:
	if not _is_engine_running and _current_speed_kmh <= 0.5:
		return
	
	_update_inputs()
	_calculate_engine_output(delta)
	_apply_physics(delta)
	_handle_drift()
	_update_wheel_data(delta)
	_emit_signals()

# ============================================================================
# INPUT HANDLING
# ============================================================================
func _update_inputs() -> void:
	# Throttle input (0-1)
	if Input.is_action_pressed("ui_accept"):
		_input_throttle = min(_input_throttle + 0.05, 1.0)
	elif Input.is_action_released("ui_accept"):
		_input_throttle = max(_input_throttle - 0.05, 0.0)
	elif Input.is_action_pressed("ui_cancel"):
		_input_throttle = 0.0
	
	# Brake input (0-1)
	if Input.is_action_pressed("ui_left") or Input.is_action_pressed("ui_right"):
		_input_brake = 0.5
	elif Input.is_action_pressed("ui_up"):
		_input_brake = 1.0
	else:
		_input_brake = 0.0
	
	# Steering input (-1 to 1)
	if Input.is_action_pressed("ui_down"):
		_input_steering = 1.0
	elif Input.is_action_pressed("ui_page_down"):
		_input_steering = -1.0
	else:
		_input_steering = lerp(_input_steering, 0, 0.1)
	
	# Gear controls
	if Input.is_action_just_pressed("ui_page_up"):
		upshift()
	if Input.is_action_just_pressed("ui_page_down"):
		downshift()
	
	# Handbrake
	_is_handbrake_active = Input.is_action_pressed("ui_space")

# ============================================================================
# ENGINE OUTPUT CALCULATION
# ============================================================================
func _calculate_engine_output(delta: float) -> void:
	if not _is_engine_running:
		_torque_output = 0.0
		_engine_force = 0.0
		return
	
	# Calculate RPM based on gear and speed
	var wheel_speed_rps: float = (_current_speed_kmh / 3.6) / (2.0 * PI * tire_radius)
	var engine_rps: float = wheel_speed_rps * final_drive_ratio * _get_gear_ratio(_current_gear)
	_current_rpm = engine_rps * 60.0
	
	# Clamp RPM
	_current_rpm = clamp(_current_rpm, 0.0, 9000.0)
	
	# Get torque multiplier from curve
	var torque_multiplier: float = _interpolate_torque_curve(_current_rpm)
	
	# Calculate output torque
	var base_torque: float = 350.0  # Base engine torque in Nm
	_torque_output = base_torque * torque_multiplier * _input_throttle
	
	# Apply to wheel force
	_engine_force = (_tqque_output * final_drive_ratio * _get_gear_ratio(_current_gear)) / tire_radius

# ============================================================================
# GEAR RATIO CALCULATION
# ============================================================================
func _get_gear_ratio(gear: int) -> float:
	match gear:
		1: return 3.5
		2: return 2.1
		3: return 1.5
		4: return 1.1
		5: return 0.85
		6: return 0.65
		GearState.REVERSE: return 3.8
		_: return 1.0

# ============================================================================
# TORQUE CURVE INTERPOLATION
# ============================================================================
func _interpolate_torque_curve(rpm: float) -> float:
	if rpm <= 0.0:
		return 0.0
	
	for i in range(torque_curve.size() - 1):
		var point_a: Vector2 = torque_curve[i]
		var point_b: Vector2 = torque_curve[i + 1]
		
		if rpm >= point_a.x and rpm <= point_b.x:
			var ratio: float = (rpm - point_a.x) / (point_b.x - point_a.x)
			return point_a.y + ratio * (point_b.y - point_a.y)
	
	if rpm > torque_curve.back().x:
		return torque_curve.back().y
	
	return 0.0

# ============================================================================
# PHYSICS APPLICATION
# ============================================================================
func _apply_physics(delta: float) -> void:
	# Calculate air resistance
	var velocity_mps: float = _current_speed_kmh / 3.06
	_air_resistance = 0.5 * 1.225 * aerodynamic_drag_coefficient * frontal_area * pow(velocity_mps, 2)
	
	# Calculate rolling resistance
	_rolling_resistance = vehicle_mass * 9.81 * rolling_resistance_coefficient
	
	# Net force calculation
	var total_resistance: float = _air_resistance + _rolling_resistance
	var net_force: float = _engine_force - total_resistance
	
	# Apply acceleration
	var acceleration: float = net_force / vehicle_mass
	var velocity_change: float = acceleration * delta
	
	# Update speed
	_current_speed_kmh += velocity_change * 3.6
	
	# Handle reverse
	if _current_speed_kmh < 0 and _current_gear > 0:
		_current_speed_kmh = 0.0
	
	# Apply to CharacterBody3D velocity
	var direction: Vector3 = global_transform.basis.z
	_velocity = direction * _current_speed_kmh / 3.6
	
	# Steering
	var steering_effective: float = _input_steering * MAX_STEERING_RATE * delta
	var rotation: float = steering_effective * deg_to_rad(1)
	rotation_y += rotation

# ============================================================================
# DRIFT HANDLING
# ============================================================================
func _handle_drift() -> void:
	if _is_handbrake_active and abs(_input_steering) > 0.3:
		_drifting = true
		_drift_angle += 0.5 * _input_steering
		
		if not _drifting:
		(drift_started.emit(abs(_drift_angle)))
		_drifting = false
		(drift_ended.emit())
	else:
		_drifting = false
		_drift_angle = lerp(_drift_angle, 0, 0.05)

# ============================================================================
# WHEEL DATA UPDATE
# ============================================================================
func _update_wheel_data(delta: float) -> void:
	# Initialize wheels if needed
	if _wheel_forces.size() < 4:
		for i in range(4):
			_wheel_forces.append(Vector3.ZERO)
			_wheel_angles.append(0.0)
			_wheel_slip_ratios.append(1.0)
	
	# Update wheel slip ratios
	for i in range(_wheel_forces.size()):
		var drive_wheel: bool = _is_drive_wheel(i)
		var target_slip: float = 1.0 if drive_wheel else 1.0
		_wheel_slip_ratios[i] = lerp(_wheel_slip_ratios[i], target_slip, 0.1)
	
	# Apply traction control if active
	if _is_traction_control_active:
		for i in range(_wheel_slip_ratios.size()):
			if _wheel_slip_ratios[i] > TRACTION_CONTROL_THRESHOLD:
				_wheel_slip_ratios[i] = clamp(_wheel_slip_ratios[i], 0.0, TRACTION_CONTROL_THRESHOLD)

# ============================================================================
# DRIVE WHEEL CHECK
# ============================================================================
func _is_drive_wheel(wheel_index: int) -> bool:
	match drivetrain_type:
		DrivetrainType.FWD:
			return wheel_index < 2
		DrivetrainType.RWD:
			return wheel_index >= 2
		DrivetrainType.AWD:
			return true
		_:
			return wheel_index >= 2

# ============================================================================
# SIGNAL EMITTERS
# ============================================================================
func _emit_signals() -> void:
	speed_changed.emit(_current_speed_kmh)
	rpm_changed.emit(_current_rpm)
	vehicle_moved.emit(global_position, _velocity)

# ============================================================================
# DEBUG VISUALIZATION
# ============================================================================
func _draw_debug_visualization() -> void:
	if GameManager.debug_mode:
		# Draw wheel forces
		for i in range(_wheel_forces.size()):
			var pos: Vector3 = _get_wheel_position(i)
			draw_line(pos, pos + _wheel_forces[i] * 0.01, Color.RED)

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================
func _get_wheel_position(wheel_index: int) -> Vector3:
	var offset_x: float = 0.75
	var offset_z: float = 0.5
	var positions: Array[Vector3] = [
		Vector3(-offset_x, 0, -offset_z),  # Front Left
		Vector3(offset_x, 0, -offset_z),   # Front Right
		Vector3(-offset_x, 0, offset_z),   # Rear Left
		Vector3(offset_x, 0, offset_z)     # Rear Right
	]
	return global_position + global_transform.basis * positions[wheel_index]

func calculate_power_hp() -> float:
	return (_torque_output * _current_rpm) / 7127.0

func calculate_power_kw() -> float:
	return power_hp * 0.7457

## Called when the node enters the scene tree for the first time
func _ready() -> void:
	_process_mode = ProcessModeEnum.ALWAYS
	_init_wheels()
	start_engine()
	_connect_signals()

func _init_wheels() -> void:
	_wheel_forces.resize(4)
	_wheel_angles.resize(4)
	_wheel_slip_ratios.resize(4)

func _connect_signals() -> void:
	pass

func _exit_tree() -> void:
	stop_engine()