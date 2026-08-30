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
const MAX_GEAR := 7
const MIN_GEAR := -1  # -1 = reverse, 0 = neutral, 1-7 = forward gears
const NEUTRAL_GEAR := 0
const REVERSE_GEAR := -1

enum DrivetrainType {
	FWD,    # Front Wheel Drive
	RWD,    # Rear Wheel Drive
	AWD     # All Wheel Drive
}

enum SuspensionType {
	MACFERSON,
	DOUBLE_WISHBONE,
	STRUT,
	TORSION_BAR
}

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
@export var drivetrain_type: DrivetrainType = DrivetrainType.RWD
@export var final_drive_ratio: float = 3.5: set = _set_final_drive_ratio
@export var tire_radius: float = 0.33: set = _set_tire_radius
@export var torque_curve: Array[Vector2f] = [
	Vector2f(0.0, 0.0),
	Vector2f(0.1, 0.3),
	Vector2f(0.2, 0.6),
	Vector2f(0.3, 0.9),
	Vector2f(0.4, 1.1),
	Vector2f(0.5, 1.2),
	Vector2f(0.6, 1.25),
	Vector2f(0.7, 1.2),
	Vector2f(0.8, 1.1),
	Vector2f(0.9, 0.9),
	Vector2f(1.0, 0.7)
]: set = _set_torque_curve
@export var idle_rpm: float = 800.0: set = _set_idle_rpm
@export var redline_rpm: float = 7000.0: set = _set_redline_rpm
@export var clutch_engagement_point: float = 0.2

@export_group("Suspension & Tires")
@export var suspension_travel: float = 0.15: set = _set_suspension_travel
@export var spring_rate: float = 45000.0: set = _set_spring_rate
@export var damping_rate: float = 3500.0: set = _set_damping_rate
@export var camber_angle: float = -1.5: set = _set_camber_angle
@export var toe_angle: float = 0.1: set = _set_toe_angle
@export var caster_angle: float = 3.0: set = _set_caster_angle

@export_group("Aerodynamics")
@export var drag_coefficient: float = 0.32: set = _set_drag_coefficient
@export var frontal_area: float = 2.2: set = _set_frontal_area
@export var downforce_coefficient: float = 0.5: set = _set_downforce_coefficient
@export var wing_angle: float = 10.0: set = _set_wing_angle

@export_group("Brakes")
@export var brake_pressure: float = 120.0: set = _set_brake_pressure
@export var brake_bias_front: float = 0.6: set = _set_brake_bias_front
@export var anti_lock_braking: bool = true

@export_group("Engine")
@export var engine_displacement: float = 3.5: set = _set_engine_displacement
@export var compression_ratio: float = 11.5: set = _set_compression_ratio
@export var fuel_consumption_rate: float = 0.08: set = _set_fuel_consumption_rate

# ============================================================================
# PRIVATE STATE VARIABLES
# ============================================================================
var _current_speed_kmh: float = 0.0
var _current_rpm: float = 0.0
var _current_gear: int = NEUTRAL_GEAR
var _throttle_input: float = 0.0
var _brake_input: float = 0.0
var _steering_input: float = 0.0
var _handbrake_active: bool = false
var _engine_running: bool = false
var _clutch_pedal: float = 0.0
var _fuel_level: float = 100.0
var _tire_temperatures: Array[float] = []
var _wheel_slip_angles: Array[float] = []
var _drift_angle: float = 0.0
var _is_drifting: bool = false
var _suspension_compression: Array[float] = []
var _brake_temperatures: Array[float] = []
var _torque_multiplier: float = 1.0
var _gear_ratios: Array[float] = [0.0, 3.8, 2.4, 1.7, 1.3, 1.0, 0.8, 0.65]
var _reverse_ratio: float = 3.9
var _air_density: float = 1.225
var _track_width: float = 1.6
var _wheelbase: float = 2.7
var _center_of_gravity_height: float = 0.55
var _moment_of_inertia_x: float = 2500.0
var _moment_of_inertia_y: float = 2500.0
var _moment_of_inertia_z: float = 4000.0

# ============================================================================
# WHEEL DATA STRUCTURES
# ============================================================================
class WheelData:
	var position: Vector3
	var radius: float
	var rotation: float
	var slip_ratio: float = 0.0
	var slip_angle: float = 0.0
	var lateral_force: float = 0.0
	var longitudinal_force: float = 0.0
	var vertical_load: float = 0.0
	var temperature: float = 20.0
	var lock_angle: float = 0.0
	var suspension_compression: float = 0.0
	
	func _init(pos: Vector3, rad: float):
		position = pos
		radius = rad

var _front_left_wheel: WheelData
var _front_right_wheel: WheelData
var _rear_left_wheel: WheelData
var _rear_right_wheel: WheelData

# ============================================================================
# INITIALIZATION
# ============================================================================
func _ready() -> void:
	_init_wheels()
	_init_suspension()
	_connect_signals()
	_update_tire_temps()
	_update_brake_temps()

func _init_wheels() -> void:
	_front_left_wheel = WheelData.new(Vector3.ZERO, tire_radius)
	_front_right_wheel = WheelData.new(Vector3.ZERO, tire_radius)
	_rear_left_wheel = WheelData.new(Vector3.ZERO, tire_radius)
	_rear_right_wheel = WheelData.new(Vector3.ZERO, tire_radius)
	
	for i in range(4):
		_tire_temperatures.append(20.0)
		_wheel_slip_angles.append(0.0)
		_suspension_compression.append(0.0)
		_brake_temperatures.append(20.0)

func _init_suspension() -> void:
	pass  # Will be connected to actual suspension nodes in scene

func _connect_signals() -> void:
	InputManager.throttle_connected.connect(_on_throttle_input)
	InputManager.brake_connected.connect(_on_brake_input)
	InputManager.steer_connected.connect(_on_steer_input)
	InputManager.clutch_connected.connect(_on_clutch_input)
	InputManager.handbrake_connected.connect(_on_handbrake_input)
	InputManager.gear_up.connect(_shift_up)
	InputManager.gear_down.connect(_shift_down)
	InputManager.neutral.connect(_shift_neutral)
	InputManager.start_engine.connect(_start_engine)

# ============================================================================
# MAIN PROCESS LOOP
# ============================================================================
func _physics_process(delta: float) -> void:
	if not is_inside_tree():
		return
	
	# Update input values
	_apply_inputs(delta)
	
	# Calculate engine output
	_calculate_engine_output(delta)
	
	# Apply forces based on drivetrain type
	_apply_drivetrain_forces(delta)
	
	# Calculate aerodynamic forces
	_apply_aerodynamics(delta)
	
	# Handle suspension
	_update_suspension(delta)
	
	# Update wheel states
	_update_wheels(delta)
	
	# Apply braking forces
	_apply_brakes(delta)
	
	# Handle handbrake and drifting
	_handle_drifting(delta)
	
	# Update physics body
	_update_physics_body(delta)
	
	# Update internal state
	_update_state(delta)
	
	# Emit signals
	_emit_signals()

func _apply_inputs(delta: float) -> void:
	# Smoothly interpolate inputs
	_throttle_input = lerp(_throttle_input, InputManager.get_throttle(), delta * 10.0)
	_brake_input = lerp(_brake_input, InputManager.get_brake(), delta * 10.0)
	_steering_input = lerp(_steering_input, InputManager.get_steer(), delta * 15.0)
	_clutch_pedal = lerp(_clutch_pedal, InputManager.get_clutch(), delta * 10.0)
	_handbrake_active = InputManager.is_handbrake_pressed()

# ============================================================================
# ENGINE MANAGEMENT
# ============================================================================
func _calculate_engine_output(delta: float) -> void:
	if not _engine_running:
		_current_rpm = 0.0
		_torque_multiplier = 0.0
		return
	
	# Calculate target RPM based on gear and speed
	var target_rpm: float = _get_target_rpm()
	
	# Apply clutch engagement
	var clutch_factor: float = 1.0
	if _clutch_pedal > clutch_engagement_point:
		clutch_factor = (_clutch_pedal - clutch_engagement_point) / (1.0 - clutch_engagement_point)
	else:
		clutch_factor = 0.0
	
	# Smooth RPM transition
	_current_rpm = lerp(_current_rpm, target_rpm * clutch_factor, delta * 5.0)
	
	# Clamp RPM within valid range
	_current_rpm = clamp(_current_rpm, 0.0, redline_rpm * 1.2)
	
	# Calculate torque multiplier from curve
	_torque_multiplier = _interpolate_torque_curve()
	
	# Engine braking when no throttle and in gear
	if _throttle_input < 0.05 and _current_gear != NEUTRAL_GEAR and _current_gear != REVERSE_GEAR:
		_torque_multiplier *= 0.3  # Reduced torque during engine braking

func _interpolate_torque_curve() -> float:
	var rpm_fraction: float = _current_rpm / redline_rpm
	rpm_fraction = clamp(rpm_fraction, 0.0, 1.0)
	
	for i in range(torque_curve.size() - 1):
		var p1: Vector2f = torque_curve[i]
		var p2: Vector2f = torque_curve[i + 1]
		
		if rpm_fraction >= p1.x and rpm_fraction <= p2.x:
			var t: float = (rpm_fraction - p1.x) / (p2.x - p1.x)
			return p1.y + t * (p2.y - p1.y)
	
	return torque_curve[-1].y if torque_curve.size() > 0 else 1.0

func _get_target_rpm() -> float:
	if _current_gear == NEUTRAL_GEAR or _current_gear == REVERSE_GEAR:
		return idle_rpm
	
	var speed_ms: float = _current_speed_kmh / 3.6
	var gear_ratio: float = _get_gear_ratio()
	var wheel_rpm: float = speed_ms / (2.0 * PI * tire_radius)
	var engine_rpm: float = wheel_rpm * gear_ratio * final_drive_ratio
	
	return engine_rpm

func _get_gear_ratio() -> float:
	match _current_gear:
		NEUTRAL_GEAR:
			return 0.0
		REVERSE_GEAR:
			return _reverse_ratio
		_:
			if _current_gear < gear_ratios.size():
				return gear_ratios[_current_gear]
			return gear_ratios[-1]

# ============================================================================
# DRIVETRAIN SYSTEM
# ============================================================================
func _apply_drivetrain_forces(delta: float) -> void:
	if _current_gear == NEUTRAL_GEAR or _current_gear == REVERSE_GEAR:
		return
	
	var engine_torque: float = _calculate_engine_torque()
	var effective_torque: float = engine_torque * _torgue_multiplier * final_drive_ratio
	
	# Distribute torque based on drivetrain type
	match drivetrain_type:
		DrivetrainType.FWD:
			_apply_traction_force(_front_left_wheel, effective_torque * 0.5)
			_apply_traction_force(_front_right_wheel, effective_torque * 0.5)
		DrivetrainType.RWD:
			_apply_traction_force(_rear_left_wheel, effective_torque * 0.5)
			_apply_traction_force(_rear_right_wheel, effective_torque * 0.5)
		DrivetrainType.AWD:
			_apply_traction_force(_front_left_wheel, effective_torque * 0.4)
			_apply_traction_force(_front_right_wheel, effective_torque * 0.4)
			_apply_traction_force(_rear_left_wheel, effective_torque * 0.3)
			_apply_traction_force(_rear_right_wheel, effective_torque * 0.3)

func _apply_traction_force(wheel: WheelData, force: float) -> void:
	if _clutch_pedal > clutch_engagement_point:
		return
	
	wheel.longitudinal_force += force
	wheel.slip_ratio += force / (acceleration_force * 10.0)

func _calculate_engine_torque() -> float:
	var base_torque: float = engine_displacement * compression_ratio * 100.0
	return base_torque * _torque_multiplier

# ============================================================================
# AERODYNAMICS
# ============================================================================
func _apply_aerodynamics(delta: float) -> void:
	var speed_ms: float = _current_speed_kmh / 3.6
	var speed_squared: float = speed_ms * speed_ms
	
	# Drag force (opposes motion)
	var drag_force: float = 0.5 * _air_density * drag_coefficient * frontal_area * speed_squared
	drag_force *= -1.0  # Opposite direction
	
	# Downforce (increases tire grip at high speeds)
	var downforce: float = 0.5 * _air_density * downforce_coefficient * frontal_area * speed_squared
	downforce *= sin(deg_to_rad(wing_angle))
	
	# Apply forces to center of mass
	add_force(Vector3(drag_force, 0.0, 0.0))
	add_force(Vector3(0.0, -downforce, 0.0))

# ============================================================================
# SUSPENSION SYSTEM
# ============================================================================
func _update_suspension(delta: float) -> void:
	# Simplified suspension calculation
	for i in range(4):
		var gravity_force: float = vehicle_mass * PhysicsSettings.gravity / 4.0
		var spring_force: float = _spring_rate * _suspension_compression[i]
		var damping_force: float = _damping_rate * _get_suspension_velocity(i)
		
		var total_force: float = gravity_force - spring_force - damping_force
		
		_suspension_compression[i] += total_force * delta / spring_rate
		_suspension_compression[i] = clamp(_suspension_compression[i], 0.0, suspension_travel)

func _get_suspension_velocity(index: int) -> float:
	# Placeholder - would calculate based on wheel movement
	return 0.0

# ============================================================================
# BRAKING SYSTEM
# ============================================================================
func _apply_brakes(delta: float) -> void:
	if _brake_input < 0.01 and not _handbrake_active:
		return
	
	var brake_force_per_wheel: float = _brake_force_per_wheel()
	var front_bias: float = _brake_bias_front
	var rear_bias: float = 1.0 - front_bias
	
	# Apply brakes to all wheels
	_set_wheel_brake_force(_front_left_wheel, brake_force_per_wheel * front_bias)
	_set_wheel_brake_force(_front_right_wheel, brake_force_per_wheel * front_bias)
	_set_wheel_brake_force(_rear_left_wheel, brake_force_per_wheel * rear_bias)
	_set_wheel_brake_force(_rear_right_wheel, brake_force_per_wheel * rear_bias)
	
	# Heat up brake discs
	_heat_brakes(brake_force_per_wheel * 4.0)

func _brake_force_per_wheel() -> float:
	var max_brake_force: float = braking_force
	var brake_pressure_factor: float = min(_brake_input, 1.0)
	
	if anti_lock_braking:
		# Simple ABS logic
		var wheel_slip_threshold: float = 0.2
		for wheel in [_front_left_wheel, _front_right_wheel, _rear_left_wheel, _rear_right_wheel]:
			if abs(wheel.slip_ratio) > wheel_slip_threshold:
				brake_pressure_factor *= 0.7
	
	return max_brake_force * brake_pressure_factor

func _set_wheel_brake_force(wheel: WheelData, force: float) -> void:
	wheel.longitudinal_force -= force
	wheel.temperature += force * 0.01

func _heat_brakes(power: float) -> void:
	for i in range(4):
		_brake_temperatures[i] += power * 0.1
		_brake_temperatures[i] = clamp(_brake_temperatures[i], 20.0, 900.0)

# ============================================================================
# DRIFTING SYSTEM
# ============================================================================
func _handle_drifting(delta: float) -> void:
	if _current_gear == NEUTRAL_GEAR or _current_gear == REVERSE_GEAR:
		_is_drifting = false
		_drift_angle = 0.0
		return
	
	var speed_ms: float = _current_speed_kmh / 3.6
	var steer_input: float = _steering_input * deg_to_rad(steering_angle_max)
	
	# Calculate lateral acceleration
	var lateral_acc: float = speed_ms * steer_input
	
	# Drift threshold based on speed and grip
	var drift_threshold: float = 3.0 + (speed_ms * 0.1)
	
	if abs(lateral_acc) > drift_threshold and _handbrake_active:
		_is_drifting = true
		_drift_angle = lerp(_drift_angle, steer_input, delta * 5.0)
		_drift_angle = clamp(_drift_angle, -1.0, 1.0)
		
		if not signal_drift_started.is_connected(_on_drift_started):
			signal_drift_started.connect(_on_drift_started)
		emit_signal("drift_started", abs(_drift_angle))
	elif abs(_drift_angle) > 0.1:
		_drift_angle = lerp(_drift_angle, 0.0, delta * 3.0)
		if abs(_drift_angle) < 0.05:
			_is_drifting = false
			signal_drift_ended.emit()
	else:
		_is_drifting = false
		_drift_angle = 0.0

func _on_drift_started(angle: float) -> void:
	AudioManager.play_sound("drift_start", 0.7)

# ============================================================================
# WHEEL PHYSICS
# ============================================================================
func _update_wheels(delta: float) -> void:
	var wheels: Array[WheelData] = [
		_front_left_wheel,
		_front_right_wheel,
		_rear_left_wheel,
		_rear_right_wheel
	]
	
	for wheel in wheels:
		# Update rotation based on speed
		var linear_velocity: float = _current_speed_kmh / 3.6
		wheel.rotation += linear_velocity / tire_radius * delta
		
		# Update slip ratios
		wheel.slip_ratio = lerp(wheel.slip_ratio, 0.0, delta * 2.0)
		
		# Update temperatures
		wheel.temperature = lerp(wheel.temperature, 20.0, delta * 0.5)
		
		# Limit temperatures
		wheel.temperature = clamp(wheel.temperature, 20.0, 200.0)

# ============================================================================
# PHYSICS BODY UPDATE
# ============================================================================
func _update_physics_body(delta: float) -> void:
	# Apply net forces to character body
	var net_force: Vector3 = get_total_force()
	velocity = velocity.linear_interpolate(net_force / vehicle_mass, delta * 10.0)
	move_and_slide()

func get_total_force() -> Vector3:
	var forces: Vector3 = Vector3.ZERO
	
	# Add wheel forces
	forces += _front_left_wheel.longitudinal_force * transform.basis.z.normalized()
	forces += _front_right_wheel.longitudinal_force * transform.basis.z.normalized()
	forces += _rear_left_wheel.longitudinal_force * transform.basis.z.normalized()
	forces += _rear_right_wheel.longitudinal_force * transform.basis.z.normalized()
	
	return forces

# ============================================================================
# STATE MANAGEMENT
# ============================================================================
func _update_state(delta: float) -> void:
	# Update speed from velocity
	var speed_vector: Vector3 = velocity
	_current_speed_kmh = speed_vector.length() * 3.6
	
	# Fuel consumption
	if _engine_running and _throttle_input > 0.1:
		_fuel_level -= fuel_consumption_rate * _throttle_input * delta
		_fuel_level = max(_fuel_level, 0.0)

func _emit_signals() -> void:
	emit_signal("speed_changed", _current_speed_kmh)
	emit_signal("rpm_changed", _current_rpm)
	emit_signal("vehicle_moved", global_position, velocity)

# ============================================================================
# GEAR SHIFTING
# ============================================================================
func shift_to_gear(gear: int) -> void:
	var old_gear: int = _current_gear
	_current_gear = clamp(gear, MIN_GEAR, MAX_GEAR)
	
	if old_gear != _current_gear:
		emit_signal("gear_changed", old_gear, _current_gear)
		AudioManager.play_sound("gear_shift", 0.5)

func _shift_up() -> void:
	if _current_gear < MAX_GEAR and _current_gear >= 0:
		shift_to_gear(_current_gear + 1)

func _shift_down() -> void:
	if _current_gear > MIN_GEAR:
		shift_to_gear(_current_gear - 1)

func _shift_neutral() -> void:
	shift_to_gear(NEUTRAL_GEAR)

# ============================================================================
# ENGINE CONTROL
# ============================================================================
func start_engine() -> void:
	if not _engine_running:
		_engine_running = true
		_current_rpm = idle_rpm
		emit_signal("engine_started")
		AudioManager.play_sound("engine_start", 0.6)

func stop_engine() -> void:
	if _engine_running:
		_engine_running = false
		_current_rpm = 0.0
		emit_signal("engine_stopped")
		AudioManager.play_sound("engine_stop", 0.5)

func toggle_engine() -> void:
	if _engine_running:
		stop_engine()
	else:
		start_engine()

# ============================================================================
# INPUT HANDLERS
# ============================================================================
func _on_throttle_input(value: float) -> void:
	_throttle_input = value

func _on_brake_input(value: float) -> void:
	_brake_input = value

func _on_steer_input(value: float) -> void:
	_steering_input = value

func _on_clutch_input(value: float) -> void:
	_clutch_pedal = value

func _on_handbrake_input(active: bool) -> void:
	_handbrake_active = active
	emit_signal("handbrake_toggled", active)

# ============================================================================
# PROPERTY SETTERS
# ============================================================================
func _set_vehicle_mass(value: float) -> void:
	vehicle_mass = value
	_update_mass_properties()

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

func _set_idle_rpm(value: float) -> void:
	idle_rpm = value

func _set_redline_rpm(value: float) -> void:
	redline_rpm = value

func _set_suspension_travel(value: float) -> void:
	suspension_travel = value

func _set_spring_rate(value: float) -> void:
	spring_rate = value

func _set_damping_rate(value: float) -> void:
	damping_rate = value

func _set_camber_angle(value: float) -> void:
	camber_angle = value

func _set_toe_angle(value: float) -> void:
	toe_angle = value

func _set_caster_angle(value: float) -> void:
	caster_angle = value

func _set_drag_coefficient(value: float) -> void:
	drag_coefficient = value

func _set_frontal_area(value: float) -> void:
	frontal_area = value

func _set_downforce_coefficient(value: float) -> void:
	downforce_coefficient = value

func _set_wing_angle(value: float) -> void:
	wing_angle = value

func _set_brake_pressure(value: float) -> void:
	brake_pressure = value

func _set_brake_bias_front(value: float) -> void:
	brake_bias_front = value

func _set_engine_displacement(value: float) -> void:
	engine_displacement = value

func _set_compression_ratio(value: float) -> void:
	compression_ratio = value

func _set_fuel_consumption_rate(value: float) -> void:
	fuel_consumption_rate = value

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================
func _update_mass_properties() -> void:
	# Recalculate moment of inertia based on mass
	_moment_of_inertia_x = vehicle_mass * _wheelbase * _wheelbase / 12.0
	_moment_of_inertia_y = vehicle_mass * _wheelbase * _wheelbase / 12.0
	_moment_of_inertia_z = vehicle_mass * (_track_width * _track_width + _wheelbase * _wheelbase) / 12.0

func _update_tire_temps() -> void:
	for i in range(4):
		_tire_temperatures[i] = _front_left_wheel.temperature

func _update_brake_temps() -> void:
	for i in range(4):
		_brake_temperatures[i] = _front_left_wheel.temperature

func reset_vehicle() -> void:
	_current_speed_kmh = 0.0
	_current_rpm = idle_rpm
	_current_gear = NEUTRAL_GEAR
	_throttle_input = 0.0
	_brake_input = 0.0
	_steering_input = 0.0
	_handbrake_active = false
	_fuel_level = 100.0
	_is_drifting = false
	_drift_angle = 0.0
	
	for wheel in [_front_left_wheel, _front_right_wheel, _rear_left_wheel, _rear_right_wheel]:
		wheel.longitudinal_force = 0.0
		wheel.lateral_force = 0.0
		wheel.slip_ratio = 0.0
		wheel.temperature = 20.0
	
	velocity = Vector3.ZERO

func get_vehicle_stats() -> Dictionary:
	return {
		"speed_kmh": _current_speed_kmh,
		"rpm": _current_rpm,
		"gear": _current_gear,
		"fuel_level": _fuel_level,
		"is_drifting": _is_drifting,
		"drift_angle": _drift_angle,
		"tire_temps": _tire_temperatures,
		"brake_temps": _brake_temperatures
	}

func save_settings() -> void:
	var settings: Dictionary = {
		"vehicle_mass": vehicle_mass,
		"max_speed_kmh": max_speed_kmh,
		"acceleration_force": acceleration_force,
		"braking_force": braking_force,
		"steering_angle_max": steering_angle_max,
		"drivetrain_type": drivetrain_type,
		"final_drive_ratio": final_drive_ratio,
		"tire_radius": tire_radius,
		"idle_rpm": idle_rpm,
		"redline_rpm": redline_rpm,
		"suspension_travel": suspension_travel,
		"spring_rate": spring_rate,
		"damping_rate": damping_rate,
		"drag_coefficient": drag_coefficient,
		"frontal_area": frontal_area,
		"downforce_coefficient": downforce_coefficient,
		"brake_bias_front": brake_bias_front,
		"anti_lock_braking": anti_lock_braking
	}
	
	GameManager.save_game("vehicle_settings", settings)

func load_settings(settings: Dictionary) -> void:
	if settings.has("vehicle_mass"):
		vehicle_mass = settings["vehicle_mass"]
	if settings.has("max_speed_kmh"):
		max_speed_kmh = settings["max_speed_kmh"]
	if settings.has("acceleration_force"):
		acceleration_force = settings["acceleration_force"]
	if settings.has("braking_force"):
		braking_force = settings["braking_force"]
	if settings.has("steering_angle_max"):
		steering_angle_max = settings["steering_angle_max"]
	if settings.has("drivetrain_type"):
		drivetrain_type = settings["drivetrain_type"]
	if settings.has("final_drive_ratio"):
		final_drive_ratio = settings["final_drive_ratio"]
	if settings.has("tire_radius"):
		tire_radius = settings["tire_radius"]
	if settings.has("idle_rpm"):
		idle_rpm = settings["idle_rpm"]
	if settings.has("redline_rpm"):
		redline_rpm = settings["redline_rpm"]
	if settings.has("suspension_travel"):
		suspension_travel = settings["suspension_travel"]
	if settings.has("spring_rate"):
		spring_rate = settings["spring_rate"]
	if settings.has("damping_rate"):
		damping_rate = settings["damping_rate"]
	if settings.has("drag_coefficient"):
		drag_coefficient = settings["drag_coefficient"]
	if settings.has("frontal_area"):
		frontal_area = settings["frontal_area"]
	if settings.has("downforce_coefficient"):
		downforce_coefficient = settings["downforce_coefficient"]
	if settings.has("brake_bias_front"):
		brake_bias_front = settings["brake_bias_front"]
	if settings.has("anti_lock_braking"):
		anti_lock_braking = settings["anti_lock_braking"]

func _set_final_drive_ratio(value: float) -> void:
	if value > 0:
		final_drive_ratio = value

func _set_tire_radius(value: float) -> void:
	if value > 0:
		tire_radius = value

func _set_torque_curve(value: Array[Vector2f]) -> void:
	if value.size() > 0:
		torque_curve = value

func _set_idle_rpm(value: float) -> void:
	if value > 0:
		idle_rpm = value

func _set_redline_rpm(value: float) -> void:
	if value > idle_rpm:
		redline_rpm = value

func _set_suspension_travel(value: float) -> void:
	if value > 0:
		suspension_travel = value

func _set_spring_rate(value: float) -> void:
	if value > 0:
		spring_rate = value

func _set_damping_rate(value: float) -> void:
	if value > 0:
		damping_rate = value

func _set_camber_angle(value: float) -> void:
	camber_angle = value

func _set_toe_angle(value: float) -> void:
	toe_angle = value

func _set_caster_angle(value: float) -> void:
	caster_angle = value

func _set_drag_coefficient(value: float) -> void:
	if value > 0:
		drag_coefficient = value

func _set_frontal_area(value: float) -> void:
	if value > 0:
		frontal_area = value

func _set_downforce_coefficient(value: float) -> void:
	if value >= 0:
		downforce_coefficient = value

func _set_wing_angle(value: float) -> void:
	if value >= 0:
		wing_angle = value

func _set_brake_pressure(value: float) -> void:
	if value > 0:
		brake_pressure = value

func _set_brake_bias_front(value: float) -> void:
	if value >= 0 and value <= 1:
		brake_bias_front = value

func _set_engine_displacement(value: float) -> void:
	if value > 0:
		engine_displacement = value

func _set_compression_ratio(value: float) -> void:
	if value > 1:
		compression_ratio = value

func _set_fuel_consumption_rate(value: float) -> void:
	if value >= 0:
		fuel_consumption_rate = value

func _set_vehicle_mass(value: float) -> void:
	if value > 0:
		vehicle_mass = value
		_update_mass_properties()

func _set_max_speed_kmh(value: float) -> void:
	if value > 0:
		max_speed_kmh = value

func _set_acceleration_force(value: float) -> void:
	if value > 0:
		acceleration_force = value

func _set_braking_force(value: float) -> void:
	if value > 0:
		braking_force = value

func _set_steering_angle_max(value: float) -> void:
	if value > 0:
		steering_angle_max = value

</FILE>