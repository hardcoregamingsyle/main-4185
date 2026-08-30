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
var _clutch_input: float = 0.0
var _engine_on: bool = false
var _is_drifting: bool = false
var _drift_angle: float = 0.0
var _traction_control_enabled: bool = true
var _abs_enabled: bool = true
var _differential_locked: bool = false
var _skid_state: bool = false
var _last_collision_time: float = 0.0
var _gear_shift_timer: float = 0.0
var _clutch_timer: float = 0.0
var _wheel_steering_angles: Vector3 = Vector3.ZERO
var _vehicle_mass: float = 1500.0
var _powertrain_node: Node = null
var _physics_settings: PhysicsSettings = null
var _collision_normal: Normal = Vector3.UP
var _surface_friction: float = 1.0
var _acceleration_vector: Vector3 = Vector3.ZERO
var _velocity_world: Vector3 = Vector3.ZERO

# ============================================================================
# GEAR RATIO TABLES (Final Drive Ratios per Gear)
# ============================================================================
var _gear_ratios: Array[float] = [
	0.0,           # Neutral
	3.8,           # 1st
	2.5,           # 2nd
	1.8,           # 3rd
	1.4,           # 4th
	1.1,           # 5th
	0.9,           # 6th
	3.5            # Reverse
]

# ============================================================================
# INITIALIZATION
# ============================================================================
func _ready() -> void:
	_init_dependencies()
	_connect_signals()
	_reset_vehicle_state()

func _init_dependencies() -> void:
	_physics_settings = PhysicsSettings.get_singleton() if is_instance_valid(PhysicsSettings.get_singleton()) else PhysicsSettings.new()
	var powertrain_script = load("res://scripts/vehicles/Powertrain.gd")
	if powertrain_script:
		_powertrain_node = get_node_or_null("../Powertrain")
		if not _powertrain_node:
			_powertrain_node = Powertrain.new()
			add_child(_powertrain_node)
			_powertrain_node.owner = self

func _connect_signals() -> void:
	if _powertrain_node:
		_powertrain_node.engine_started.connect(_on_engine_started)
		_powertrain_node.engine_stopped.connect(_on_engine_stopped)
		_powertrain_node.torque_available.connect(_on_torque_available)

func _reset_vehicle_state() -> void:
	_current_gear = NEUTRAL_GEAR
	_target_gear = NEUTRAL_GEAR
	_speed_kmh = 0.0
	_rpm = IDLE_RPM
	_throttle_input = 0.0
	_brake_input = 0.0
	_steering_input = 0.0
	_handbrake_input = 0.0
	_clutch_input = 0.0
	_engine_on = false
	_is_drifting = false
	_traction_control_enabled = true
	_abs_enabled = true
	_differential_locked = false
	_skid_state = false
	_last_collision_time = 0.0
	_gear_shift_timer = 0.0
	_clutch_timer = 0.0
	_wheel_steering_angles = Vector3.ZERO
	_velocity_world = Vector3.ZERO

# ============================================================================
# INPUT HANDLING
# ============================================================================
func _input_event(camera: Camera3D, event: InputEvent) -> void:
	if event is InputEventKey:
		_handle_keyboard_input(event)
	elif event is InputEventMouseMotion:
		_handle_mouse_input(event)

func _handle_keyboard_input(event: InputEventKey) -> void:
	if event.pressed:
		match event.keycode:
			KEY_W, KEY_UP, KEY_PAGEUP:
				_set_throttle(1.0)
			KEY_S, KEY_DOWN, KEY_PAGEDOWN:
				_set_throttle(-1.0)
			KEY_A, KEY_LEFT:
				_set_steering(-1.0)
			KEY_D, KEY_RIGHT:
				_set_steering(1.0)
			KEY_SPACE:
				_set_handbrake(1.0)
			KEY_Q:
				_set_brake(1.0)
			KEY_E:
				_toggle_engine()
			KEY_TAB:
				_cycle_differential_lock()
			KEY_C:
				_toggle_traction_control()
			KEY_B:
				_toggle_abs()
			KEY_Z:
				_upshift()
			KEY_X:
				_downshift()
			KEY_V:
				_manual_clutch()
			KEY_R:
				_reset_vehicle()

func _set_throttle(value: float) -> void:
	_throttle_input = clampf(value, -1.0, 1.0)
	throttle_applied.emit(abs(_throttle_input))

func _set_brake(value: float) -> void:
	_brake_input = clampf(value, 0.0, 1.0)
	brake_applied.emit(_brake_input)

func _set_steering(value: float) -> void:
	_steering_input = clampf(value, -1.0, 1.0)
	steering_angle_changed.emit(abs(_steering_input))

func _set_handbrake(value: float) -> void:
	_handbrake_input = clampf(value, 0.0, 1.0)
	handbrake_toggled.emit(_handbrake_input > 0.5)

func _toggle_engine() -> void:
	_engine_on = !_engine_on
	if _powertrain_node:
		if _engine_on:
			_powertrain_node.start_engine()
		else:
			_powertrain_node.stop_engine()

func _cycle_differential_lock() -> void:
	_differential_locked = !_differential_locked
	print("Differential Lock: ", _differential_locked)

func _toggle_traction_control() -> void:
	_traction_control_enabled = !_traction_control_enabled
	traction_control_state_changed.emit(_traction_control_enabled)

func _toggle_abs() -> void:
	_abs_enabled = !_abs_enabled
	anti_lock_braking_state_changed.emit(_abs_enabled)

func _upshift() -> void:
	if _current_gear < MAX_GEAR:
		_target_gear = min(_current_gear + 1, MAX_GEAR)
		_trigger_gear_shift()

func _downshift() -> void:
	if _current_gear > MIN_GEAR:
		_target_gear = max(_current_gear - 1, MIN_GEAR)
		_trigger_gear_shift()

func _manual_clutch() -> void:
	_clutch_input = 1.0 - _clutch_input
	_clutch_timer = CLUTCH_RELEASE_TIME if _clutch_input > 0.5 else 0.0

func _reset_vehicle() -> void:
	position = Vector3.ZERO
	rotation = Vector3.ZERO
	velocity = Vector3.ZERO
	_reset_vehicle_state()
	_engine_on = true
	if _powertrain_node:
		_powertrain_node.start_engine()

# ============================================================================
# PHYSICS UPDATE
# ============================================================================
func _physics_process(delta: float) -> void:
	if not _engine_on and _speed_kmh == 0.0:
		return
	
	_apply_inputs(delta)
	_update_rpm_and_gears(delta)
	_calculate_forces(delta)
	_apply_wheel_forces(delta)
	_handle_collision_detection(delta)
	_update_physics(delta)

func _apply_inputs(delta: float) -> void:
	var effective_throttle = _throttle_input
	var effective_brake = _brake_input
	var effective_steering = _steering_input
	
	# Clutch effect on power delivery
	if _clutch_input > 0.5:
		effective_throttle *= (_clutch_input - 0.5) * 2.0
	
	# Handbrake affects braking and enables drifting
	var handbrake_effect = _handbrake_input * 0.7
	effective_brake += handbrake_effect
	
	# Steering smoothing
	_wheel_steering_angles.x = lerp(_wheel_steering_angles.x, effective_steering * MAX_STEERING_ANGLE, delta * STEERING_SPEED)

func _update_rpm_and_gears(delta: float) -> void:
	# Calculate RPM based on gear and speed
	var gear_ratio = _gear_ratios[abs(_current_gear)] if _current_gear != 0 else 0.0
	var final_drive_ratio = 3.5
	var wheel_radius = 0.35
	var wheels_per_meter = 1.0 / (2.0 * PI * wheel_radius)
	
	if _current_gear != NEUTRAL_GEAR:
		var wheel_rps = _speed_kmh / 3.6 * wheels_per_meter
		_rpm = wheel_rps * gear_ratio * final_drive_ratio
	else:
		# Engine idle in neutral
		_rpm = lerp(_rpm, IDLE_RPM, delta * 5.0)
	
	# Clamp RPM
	_rpm = clampf(_rpm, 0.0, SHUTDOWN_RPM)
	
	# Auto-shift logic
	if _gear_shift_timer <= 0.0 and abs(_throttle_input) > 0.1:
		_auto_shift_logic(delta)
	
	# Check for stalling
	if _rpm < 300.0 and _current_gear != NEUTRAL_GEAR and abs(_throttle_input) < 0.3:
		_stall_engine()

func _auto_shift_logic(delta: float) -> void:
	if _current_gear >= MAX_GEAR and _rpm >= SHIFT_POINT_RPM:
		_target_gear = MAX_GEAR
		_trigger_gear_shift()
	elif _current_gear > MIN_GEAR and _rpm < IDLE_RPM + 500.0 and _speed_kmh > 5.0:
		_target_gear = _current_gear - 1
		_trigger_gear_shift()

func _trigger_gear_shift() -> void:
	if _gear_shift_timer > 0.0 or _clutch_input > 0.5:
		return
	
	if _target_gear != _current_gear:
		var old_gear = _current_gear
		_current_gear = _target_gear
		gear_changed.emit(old_gear, _current_gear)
		
		# Rev drop on downshift
		if _target_gear < old_gear:
			_rpm = max(_rpm - ENGINE_REVO_DROP_ON_DOWNSHIFT, IDLE_RPM)
		
		_gear_shift_timer = GEAR_SHIFT_DELAY

func _calculate_forces(delta: float) -> void:
	var force_vector = Vector3.ZERO
	
	# Engine torque calculation
	var torque_multiplier = _get_torque_curve_factor()
	var engine_torque = 400.0 * torque_multiplier
	
	if _current_gear != NEUTRAL_GEAR:
		var gear_ratio = _gear_ratios[abs(_current_gear)]
		var drive_force = engine_torque * gear_ratio * 0.8 / 0.35
		
		if _current_gear > 0:
			force_vector.x = drive_force * _throttle_input
		else:
			force_vector.x = -drive_force * _throttle_input
	
	# Braking force
	var braking_force = BRAKING_FORCE * _brake_input
	force_vector.x -= braking_force
	
	# Handbrake lateral force for drifting
	if _handbrake_input > 0.5:
		var lateral_drift_force = _speed_kmh * _handbrake_input * 0.5
		force_vector.y = -lateral_drift_force * sign(_steering_input)
	
	# Drag and rolling resistance
	var air_resistance = 0.05 * _speed_kmh * _speed_kmh
	var rolling_resistance = 0.01 * _vehicle_mass * 9.81
	force_vector.x -= air_resistance * sign(_speed_kmh)
	force_vector.x -= rolling_resistance * sign(_speed_kmh)
	
	_acceleration_vector = force_vector

func _get_torque_curve_factor() -> float:
	var normalized_rpm = (_rpm - IDLE_RPM) / (REDLINE_RPM - IDLE_RPM)
	normalized_rpm = clampf(normalized_rpm, 0.0, 1.0)
	
	# Simple torque curve approximation
	var peak_rpm = 0.6
	var torque_factor = sin(PI * normalized_rpm) if normalized_rpm <= peak_rpm else sin(PI * (1.0 - normalized_rpm) * 0.8)
	
	return clampf(torque_factor, 0.0, 1.0)

func _apply_wheel_forces(delta: float) -> void:
	# Apply calculated forces to velocity
	var acceleration = _acceleration_vector.x / _vehicle_mass
	_velocity_world.x += acceleration * delta
	
	# Friction simulation
	var ground_friction = _surface_friction * 0.98
	_velocity_world.x *= ground_friction
	
	# Update actual speed
	_speed_kmh = abs(_velocity_world.x) * 3.6
	
	# Apply velocity to body
	velocity.x = _velocity_world.x
	
	# Skid detection
	var skid_threshold = 15.0 * _surface_friction
	if abs(_velocity_world.x) > skid_threshold and _handbrake_input > 0.5:
		_skid_state = true
		skidding.emit(true)
	else:
		_skid_state = false
		skidding.emit(false)

func _handle_collision_detection(delta: float) -> void:
	var col_shape = CollisionShape3D.new()
	col_shape.shape = BoxShape3D.new()
	col_shape.position = Vector3.ZERO
	col_shape.scale = Vector3.ONE * 1.5
	add_child(col_shape)
	
	var collision_results = get_slide_collisions()
	for collision in collision_results:
		var colliding_body = collision.get_collider()
		if colliding_body.has_method("_on_vehicle_collision"):
			colliding_body._on_vehicle_collision(self, collision)
		
		var collision_time = Time.get_ticks_msec() / 1000.0
		if collision_time - _last_collision_time < 1.0:
			continue
		
		_last_collision_time = collision_time
		collision_detected.emit({
			"collider": colliding_body.name,
			"normal": collision.get_normal(),
			"penetration": collision.get_travel(),
			"impact_velocity": _velocity_world.length()
		})

func _update_physics(delta: float) -> void:
	# Clamp speed limits
	_speed_kmh = clampf(_speed_kmh, 0.0, MAX_SPEED_KMH)
	
	# Update signals
	speed_changed.emit(_speed_kmh)
	rpm_changed.emit(_rpm)
	
	# Apply damping
	velocity = velocity.linear_interpolate(Vector3.ZERO, delta * 0.1)

# ============================================================================
# HELPER METHODS
# ============================================================================
func _on_engine_started() -> void:
	_engine_on = true
	_rpm = IDLE_RPM

func _on_engine_stopped() -> void:
	_engine_on = false
	_rpm = 0.0
	engine_stalled.emit()

func _on_torque_available(torque: float) -> void:
	pass  # Torque available signal handler

func _stall_engine() -> void:
	if _current_gear != NEUTRAL_GEAR:
		_rpm = 0.0
		engine_stalled.emit()
		_current_gear = NEUTRAL_GEAR
		gear_changed.emit(_current_gear, NEUTRAL_GEAR)

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

func is_engine_on() -> bool:
	return _engine_on

func is_drifting() -> bool:
	return _is_drifting

func set_surface_friction(friction: float) -> void:
	_surface_friction = clampf(friction, 0.1, 2.0)

func set_vehicle_mass(mass: float) -> void:
	_vehicle_mass = mass

func calculate_drift_angle() -> float:
	if _handbrake_input > 0.5 and abs(_speed_kmh) > 20.0:
		_is_drifting = true
		_drift_angle = abs(_steering_input) * DRIFT_FACTOR * _speed_kmh / 100.0
		if _drift_angle > 0.5:
			drift_started.emit(_drift_angle)
		return _drift_angle
	else:
		if _is_drifting:
			_is_drifting = false
			drift_ended.emit()
		return 0.0

func reset_gear_shift_timer() -> void:
	_gear_shift_timer = 0.0

func apply_brake_force(force: float) -> void:
	_brake_input = clampf(force, 0.0, 1.0)

func apply_throttle_force(force: float) -> void:
	_throttle_input = clampf(force, -1.0, 1.0)

func apply_steering_angle(angle: float) -> void:
	_steering_input = clampf(angle, -1.0, 1.0)

func get_wheel_steering_angles() -> Vector3:
	return _wheel_steering_angles

func get_collision_normal() -> Vector3:
	return _collision_normal

func get_surface_type() -> String:
	if _surface_friction < 0.5:
		return "ice"
	elif _surface_friction < 0.8:
		return "gravel"
	else:
		return "asphalt"

func simulate_wet_conditions(wetness: float) -> void:
	_surface_friction = 1.0 - wetness * 0.4
	set_surface_friction(_surface_friction)

func simulate_wind_effect(wind_vector: Vector3) -> void:
	var wind_force = wind_vector.length() * 0.01
	_velocity_world.x -= wind_force * sign(wind_vector.x)

func get_vehicle_status() -> Dictionary:
	return {
		"speed_kmh": _speed_kmh,
		"rpm": _rpm,
		"gear": _current_gear,
		"throttle": _throttle_input,
		"brake": _brake_input,
		"steering": _steering_input,
		"handbrake": _handbrake_input,
		"engine_on": _engine_on,
		"is_drifting": _is_drifting,
		"traction_control": _traction_control_enabled,
		"abs": _abs_enabled,
		"differential_locked": _differential_locked,
		"skid_state": _skid_state
	}

func apply_damage(damage_amount: float) -> void:
	if damage_amount > 0.0:
		_vehicle_mass -= damage_amount * 10.0
		_vehicle_mass = maxf(_vehicle_mass, 500.0)
		print("Vehicle damaged! Mass reduced to: ", _vehicle_mass)

func repair_vehicle(repair_percentage: float) -> void:
	_vehicle_mass = _vehicle_mass / (1.0 - repair_percentage * 0.5)
	_vehicle_mass = minf(_vehicle_mass, 1500.0)
	print("Vehicle repaired! New mass: ", _vehicle_mass)

func debug_print_status() -> void:
	print("=== Vehicle Status ===")
	print("Speed: ", _speed_kmh, " km/h")
	print("RPM: ", _rpm)
	print("Gear: ", _current_gear)
	print("Throttle: ", _throttle_input)
	print("Brake: ", _brake_input)
	print("Steering: ", _steering_input)
	print("Handbrake: ", _handbrake_input)
	print("Engine On: ", _engine_on)
	print("Drifting: ", _is_drifting)
	print("=====================")

# ============================================================================
# DESTRUCTION
# ============================================================================
func _exit_tree() -> void:
	if _powertrain_node:
		_powertrain_node.queue_free()

# ============================================================================
# END OF FILE
# ============================================================================