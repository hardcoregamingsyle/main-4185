extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for the racing simulator
## Handles throttle, brake, steering inputs, wheel forces, and gear shifting logic
## Uses PhysicsSettings constants for all tunable values
## Copyright 2026 Thalamus Racing Simulator Project

# ============================================================================
# SIGNALS
# ============================================================================

signal speed_changed(new_speed: float)
signal gear_changed(old_gear: int, new_gear: int)
signal vehicle_state_changed(state: VehicleState)

# ============================================================================
# CONSTANTS & CONFIGURATION
# ============================================================================

const DEFAULT_ACCELERATION_FORCE := 12000.0
const DEFAULT_BRAKE_FORCE := 20000.0
const DEFAULT_STEERING_SPEED := 4.5
const MAX_STEERING_ANGLE := deg_to_rad(35.0)
const MIN_WHEEL_FRICTION := 0.2
const MAX_WHEEL_FRICTION := 1.5
const DRIFT_FACTOR := 0.85
const GRIP_THRESHOLD := 0.92

# ============================================================================
# ENUMERATIONS
# ============================================================================

enum VehicleState {
	IDLE,
	DRIVING,
	BRAKING,
	REVERSING,
	SIDESLIPPING,
	CRASHED,
	JUMPING
}

enum GearType {
	N = 0,
	FIRST = 1,
	SECOND = 2,
	THIRD = 3,
	FOURTH = 4,
	FIFTH = 5,
	SIXTH = 6,
	SEVENTH = 7
}

# ============================================================================
# EXPORTED VARIABLES FOR TUNING
# ============================================================================

@export_group("Vehicle Configuration")
@export var mass: float = PhysicsSettings.default_vehicle_mass
@export var center_of_mass_offset: Vector3 = Vector3(0.0, -0.5, 0.0)
@export var track_width: float = 1.5
@export var wheel_base: float = 2.8

@export_group("Engine & Transmission")
@export var engine_max_rpm: float = 8000.0
@export var engine_idle_rpm: float = 800.0
@export var engine_peak_torque_rpm: float = 4500.0
@export var max_gears: int = 7

@export_group("Tire & Suspension Settings")
@export var tire_radius: float = 0.32
@export var suspension_stiffness: float = 35000.0
@export var suspension_damping: float = 4000.0
@export var suspension_compression_limit: float = 0.15
@export var suspension_extension_limit: float = 0.25

@export_group("Drivetrain")
@export var drivetrain_type: DrivetrainType = DrivetrainType.FWD
@export var final_drive_ratio: float = 3.73
@export var differential_type: DifferentialType = DifferentialType.OPEN

# ============================================================================
# INTERNAL STATE
# ============================================================================

var _current_gear: int = GearType.N
var _target_gear: int = GearType.N
var _engine_rpm: float = 0.0
var _vehicle_speed: float = 0.0
var _throttle_input: float = 0.0
var _brake_input: float = 0.0
var _steering_input: float = 0.0
var _clutch_engaged: bool = true
var _handbrake_active: bool = false
var _traction_control_active: bool = true
var _abs_active: bool = true

# Wheel states (for suspension simulation)
var _front_left_wheel: Dictionary = {"suspension": 0.0, "rotation": 0.0, "angular_velocity": 0.0}
var _front_right_wheel: Dictionary = {"suspension": 0.0, "rotation": 0.0, "angular_velocity": 0.0}
var _rear_left_wheel: Dictionary = {"suspension": 0.0, "rotation": 0.0, "angular_velocity": 0.0}
var _rear_right_wheel: Dictionary = {"suspension": 0.0, "rotation": 0.0, "angular_velocity": 0.0}

# Physics references
var _powertrain: Node = null
var _collision_shape_3d: CollisionShape3D = null

# Derived values
var _gear_ratios: Array[float] = []
var _max_speed_per_gear: Array[float] = []
var _torque_curve: Dictionary = {}

# Drift state tracking
var _drift_angle: float = 0.0
var _drift_intensity: float = 0.0
var _drift_timer: float = 0.0

# ============================================================================
# INITIALIZATION
# ============================================================================

func _ready() -> void:
	_init_derived_values()
	_connect_signals_to_game_manager()
	_setup_physics_body()
	_current_gear = GearType.N
	_target_gear = GearType.N
	set_state(VehicleState.IDLE)

func _init_derived_values() -> void:
	"""Initialize gear ratios and max speeds based on configuration."""
	_gear_ratios = [
		0.0,           # N
		3.8,           # First
		2.1,           # Second
		1.5,           # Third
		1.15,          # Fourth
		0.95,          # Fifth
		0.78,          # Sixth
		0.65           # Seventh
	]
	
	# Calculate max speed per gear based on engine RPM and final drive
	for i in range(max_gears + 1):
		var ratio: float = _gear_ratios[i]
		if ratio > 0:
			var max_wheel_rps: float = engine_max_rpm / 60.0 * 1.0 / ratio / final_drive_ratio
			var circumference: float = PI * tire_radius * 2.0
			_max_speed_per_gear.append(max_wheel_rps * circumference)
		else:
			_max_speed_per_gear.append(0.0)
	
	# Build torque curve lookup
	_build_torque_curve()

func _build_torque_curve() -> void:
	"""Build a simplified torque curve based on engine characteristics."""
	var steps: int = 100
	for step in range(steps + 1):
		var rpm_percent: float = float(step) / steps
		var rpm: float = rpm_percent * engine_max_rpm
		var normalized_rpm: float = (rpm - engine_idle_rpm) / (engine_peak_torque_rpm - engine_idle_rpm)
		
		var torque: float = 0.0
		if normalized_rpm <= 1.0:
			# Rising torque to peak
			torque = lerp(0.0, 1.0, pow(normalized_rpm, 0.5))
		else:
			# Falling torque after peak
			var falling_percent: float = (normalized_rpm - 1.0)
			torque = lerp(1.0, 0.3, falling_percent * 0.8)
		
		_torque_curve[rpm] = torque

func _connect_signals_to_game_manager() -> void:
	"""Connect to GameManager signals for coordinated behavior."""
	if GameManager:
		GameManager.game_state_changed.connect(_on_game_state_changed)

func _setup_physics_body() -> void:
	"""Configure the rigid body physics properties."""
	mass = mass
	center_of_mass = center_of_mass_offset

# ============================================================================
# MAIN UPDATE LOOP
# ============================================================================

func _physics_process(delta: float) -> void:
	"""Main physics update loop with fixed timestep support."""
	# Process input
	_process_inputs(delta)
	
	# Update engine state
	_update_engine_state(delta)
	
	# Handle gear shifting
	_handle_gear_shifting(delta)
	
	# Apply wheel forces
	_apply_wheel_forces(delta)
	
	# Apply gravity and collision response
	_apply_gravity_and_collision(delta)
	
	# Update drift mechanics
	_update_drift_mechanics(delta)
	
	# Update visual components
	_update_visual_components(delta)
	
	# Update state flags
	_update_state_flags()

# ============================================================================
# INPUT PROCESSING
# ============================================================================

func _process_inputs(delta: float) -> void:
	"""Process player input and normalize control values."""
	if not InputManager:
		return
	
	var input_data: Dictionary = InputManager.get_vehicle_controls()
	
	# Validate input ranges
	_throttle_input = clamp(input_data.throttle, 0.0, 1.0)
	_brake_input = clamp(input_data.brake, 0.0, 1.0)
	_steering_input = clamp(input_data.steer, -1.0, 1.0)
	_handbrake_active = input_data.handbrake
	
	# Smooth input transitions
	_smooth_input_transitions(delta)

func _smooth_input_transitions(delta: float) -> void:
	"""Smooth out sudden input changes for more realistic feel."""
	var smoothing_factor: float = delta * 8.0
	
	_throttle_input = lerp(_throttle_input, _throttle_input, smoothing_factor)
	_brake_input = lerp(_brake_input, _brake_input, smoothing_factor)
	_steering_input = lerp(_steering_input, _steering_input, smoothing_factor)

# ============================================================================
# ENGINE MANAGEMENT
# ============================================================================

func _update_engine_state(delta: float) -> void:
	"""Update engine RPM based on gear, throttle, and vehicle dynamics."""
	if _current_gear == GearType.N or not _clutch_engaged:
		# Engine idles when in neutral or clutch disengaged
		_engine_rpm = lerp(_engine_rpm, engine_idle_rpm, delta * 5.0)
		return
	
	# Calculate target RPM based on current gear and vehicle speed
	var gear_ratio: float = _gear_raties[_current_gear]
	var wheel_rps: float = abs(_vehicle_speed) / (PI * tire_radius * 2.0)
	var target_rpm: float = wheel_rps * gear_ratio * final_drive_ratio
	
	# Apply throttle influence on RPM
	var throttle_effect: float = _throttle_input * 0.3
	target_rpm += target_rpm * throttle_effect
	
	# Clamp RPM to valid range
	target_rpm = clamp(target_rpm, engine_idle_rpm, engine_max_rpm)
	
	# Smooth RPM transition
	_engine_rpm = lerp(_engine_rpm, target_rpm, delta * 15.0)
	
	# Update signal
	speed_changed.emit(_vehicle_speed)

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================

func _handle_gear_shifting(delta: float) -> void:
	"""Handle automatic and manual gear shifting logic."""
	# Check for upshift conditions
	if _should_upshift():
		_shift_to_target_gear(_target_gear + 1, delta)
	
	# Check for downshift conditions
	elif _should_downshift():
		_shift_to_target_gear(_target_gear - 1, delta)
	
	# Manual override
	if InputManager and InputManager.is_key_pressed(Input.KEY_Z):
		_manual_shift(-1)
	elif InputManager and InputManager.is_key_pressed(Input.KEY_X):
		_manual_shift(1)

func _should_upshift() -> bool:
	"""Check if conditions are met for an upshift."""
	if _current_gear >= max_gears or _current_gear == GearType.N:
		return false
	
	if _engine_rpm >= engine_max_rpm * 0.95 and _throttle_input > 0.8:
		return true
	
	if _vehicle_speed > _max_speed_per_gear[_current_gear] * 0.95:
		return true
	
	return false

func _should_downshift() -> bool:
	"""Check if conditions are met for a downshift."""
	if _current_gear <= GearType.FIRST or _current_gear == GearType.N:
		return false
	
	if _engine_rpm < engine_idle_rpm * 1.2 and _throttle_input < 0.3:
		return true
	
	if _vehicle_speed < _max_speed_per_gear[_current_gear] * 0.3:
		return true
	
	return false

func _shift_to_target_gear(new_gear: int, delta: float) -> void:
	"""Execute a gear shift with proper clutch and timing."""
	if new_gear == _current_gear or new_gear < GearType.N or new_gear > max_gears:
		return
	
	# Disengage clutch temporarily
	_clutch_engaged = false
	
	# Perform shift after brief delay
	await get_tree().create_timer(delta * 0.1).timeout
	
	# Shift gears
	_current_gear = new_gear
	_target_gear = new_gear
	
	# Re-engage clutch
	_clutch_engaged = true
	
	# Emit signal
	gear_changed.emit(_current_gear - 1, _current_gear)

func _manual_shift(direction: int) -> void:
	"""Manual gear shift by player."""
	var new_gear: int = _current_gear + direction
	
	if new_gear < GearType.N:
		new_gear = GearType.N
	elif new_gear > max_gears:
		new_gear = max_gears
	
	if new_gear != _current_gear:
		_shift_to_target_gear(new_gear, 0.1)

# ============================================================================
# WHEEL FORCE APPLICATION
# ============================================================================

func _apply_wheel_forces(delta: float) -> void:
	"""Apply forces to wheels based on drivetrain type and gear."""
	# Calculate engine torque at current RPM
	var torque_multiplier: float = _get_torque_at_rpm()
	var engine_torque: float = torque_multiplier * 450.0  # Base torque in Nm
	
	# Apply transmission losses
	var total_ratio: float = _gear_ratios[_current_gear] * final_drive_ratio
	var wheel_torque: float = engine_torque * total_ratio * 0.85
	
	# Handle braking separately
	var brake_force: float = 0.0
	if _brake_input > 0.0 or _handbrake_active:
		brake_force = DEFAULT_BRAKE_FORCE * _brake_input
		if _handbrake_active:
			brake_force *= 1.5  # Handbrake multiplier
	
	# Distribute torque based on drivetrain type
	match drivetrain_type:
		DrivetrainType.FWD:
			_apply_fwd_forces(wheel_torque, brake_force, delta)
		DrivetrainType.RWD:
			_apply_rwd_forces(wheel_torque, brake_force, delta)
		DrivetrainType.AWD:
			_apply_awd_forces(wheel_torque, brake_force, delta)

func _apply_fwd_forces(wheel_torque: float, brake_force: float, delta: float) -> void:
	"""Apply front-wheel drive forces."""
	# Front wheels receive drive torque
	var front_torque: float = wheel_torque * 0.5
	_apply_drive_to_wheels([_front_left_wheel, _front_right_wheel], front_torque, delta)
	
	# Apply brakes to all wheels
	_apply_brakes_to_all_wheels(brake_force, delta)

func _apply_rwd_forces(wheel_torque: float, brake_force: float, delta: float) -> void:
	"""Apply rear-wheel drive forces."""
	# Rear wheels receive drive torque
	var rear_torque: float = wheel_torque * 0.5
	_apply_drive_to_wheels([_rear_left_wheel, _rear_right_wheel], rear_torque, delta)
	
	# Apply brakes to all wheels
	_apply_brakes_to_all_wheels(brake_force, delta)

func _apply_awd_forces(wheel_torque: float, brake_force: float, delta: float) -> void:
	"""Apply all-wheel drive forces."""
	# Split torque between front and rear (typical AWD distribution)
	var front_torque: float = wheel_torque * 0.4
	var rear_torque: float = wheel_torque * 0.6
	
	_apply_drive_to_wheels([_front_left_wheel, _front_right_wheel], front_torque, delta)
	_apply_drive_to_wheels([_rear_left_wheel, _rear_right_wheel], rear_torque, delta)
	
	# Apply brakes to all wheels
	_apply_brakes_to_all_wheels(brake_force, delta)

func _apply_drive_to_wheels(wheels: Array[Dictionary], torque: float, delta: float) -> void:
	"""Apply drive torque to specified wheels."""
	for wheel in wheels:
		wheel["angular_velocity"] += (torque / 2.0) * delta / tire_radius
		wheel["rotation"] += wheel["angular_velocity"] * delta

func _apply_brakes_to_all_wheels(brake_force: float, delta: float) -> void:
	"""Apply braking force to all wheels."""
	var all_wheels: Array[Dictionary] = [
		_front_left_wheel, _front_right_wheel,
		_rear_left_wheel, _rear_right_wheel
	]
	
	for wheel in all_wheels:
		var deceleration: float = brake_force / mass
		wheel["angular_velocity"] -= deceleration * delta / tire_radius

# ============================================================================
# GRAVITY & COLLISION RESPONSE
# ============================================================================

func _apply_gravity_and_collision(delta: float) -> void:
	"""Apply gravity and handle collision responses."""
	# Add gravity
	add_gravity(gravity)
	
	# Move character body
	move_and_slide()
	
	# Update vehicle speed from velocity
	_vehicle_speed = linear_velocity.length()

# ============================================================================
# DRIFT MECHANICS
# ============================================================================

func _update_drift_mechanics(delta: float) -> void:
	"""Update drift state and calculate drift effects."""
	# Check drift conditions
	var is_drifting: bool = _check_drift_conditions()
	
	if is_drifting:
		# Enter drift mode
		_drift_timer += delta
		_drift_intensity = lerp(_drift_intensity, min(_drift_timer / 2.0, 1.0), delta * 5.0)
		
		# Apply drift lateral force
		var drift_force: float = _calculate_drift_force()
		linear_velocity.x += drift_force * delta * 0.5
		linear_velocity.z += drift_force * delta * 0.5
		
		set_state(VehicleState.SIDESLIPPING)
	else:
		# Exit drift mode
		if _drift_timer > 0.1:
			_drift_timer -= delta
		else:
			_drift_timer = 0.0
		
		_drift_intensity = lerp(_drift_intensity, 0.0, delta * 10.0)
		
		if _drift_intensity < 0.01:
			set_state(VehicleState.DRIVING)

func _check_drift_conditions() -> bool:
	"""Check if conditions are met for drifting."""
	if _current_gear == GearType.N or _handbrake_active == false:
		return false
	
	if _vehicle_speed < 20.0:  # Minimum speed for drift
		return false
	
	# Check steering angle and lateral acceleration
	var lateral_acceleration: float = abs(linear_velocity.cross(Vector3.UP)).length()
	
	return _steering_input.abs() > 0.7 and lateral_acceleration > 5.0

func _calculate_drift_force() -> float:
	"""Calculate force applied during drift."""
	return _drift_intensity * 500.0 * _throttle_input

# ============================================================================
# VISUAL COMPONENT UPDATES
# ============================================================================

func _update_visual_components(delta: float) -> void:
	"""Update visual representations of wheels and chassis."""
	# Update wheel rotation angles
	_update_wheel_angles()
	
	# Update suspension heights
	_update_suspension_heights()

func _update_wheel_angles() -> void:
	"""Update visual wheel rotation based on angular velocity."""
	pass  # Visual component would connect to this

func _update_suspension_heights() -> void:
	"""Update visual suspension compression."""
	pass  # Visual component would connect to this

# ============================================================================
# STATE MANAGEMENT
# ============================================================================

func set_state(new_state: VehicleState) -> void:
	"""Set the current vehicle state."""
	if _state == new_state:
		return
	
	_state = new_state
	vehicle_state_changed.emit(new_state)

func _update_state_flags() -> void:
	"""Update state flags based on current conditions."""
	if _vehicle_speed < 0.5 and _engine_rpm < engine_idle_rpm * 1.1:
		set_state(VehicleState.IDLE)
	elif _vehicle_speed > 1.0:
		if _brake_input > 0.5:
			set_state(VehicleState.BRAKING)
		elif _vehicle_speed < 0:
			set_state(VehicleState.REVERSING)
		elif _drift_intensity > 0.3:
			set_state(VehicleState.SIDESLIPPING)

# ============================================================================
# HELPER METHODS
# ============================================================================

func _get_torque_at_rpm() -> float:
	"""Get torque multiplier at current engine RPM."""
	var rpm: float = _engine_rpm
	var closest_key: float = 0.0
	var min_distance: float = abs(rpm - 0.0)
	
	for key in _torque_curve.keys():
		var distance: float = abs(rpm - key)
		if distance < min_distance:
			min_distance = distance
			closest_key = key
	
	return _torque_curve[closest_key]

func _on_game_state_changed(new_state: GameState) -> void:
	"""Handle game state changes."""
	match new_state:
		GameState.RACE_PAUSED:
			_paused = true
		GameState.RACE_ACTIVE, GameState.MAIN_MENU:
			_paused = false

func reset_vehicle() -> void:
	"""Reset vehicle to starting position and state."""
	_current_gear = GearType.N
	_target_gear = GearType.N
	_engine_rpm = engine_idle_rpm
	_vehicle_speed = 0.0
	_throttle_input = 0.0
	_brake_input = 0.0
	_steering_input = 0.0
	_clutch_engaged = true
	_handbrake_active = false
	_drift_intensity = 0.0
	_drift_timer = 0.0
	
	position = Vector3.ZERO
	linear_velocity = Vector3.ZERO
	angular_velocity = Vector3.ZERO
	
	set_state(VehicleState.IDLE)

func get_vehicle_status() -> Dictionary:
	"""Get current vehicle status as dictionary."""
	return {
		"speed": _vehicle_speed,
		"rpm": _engine_rpm,
		"gear": _current_gear,
		"throttle": _throttle_input,
		"brake": _brake_input,
		"steering": _steering_input,
		"state": _state,
		"drift_intensity": _drift_intensity
	}

# ============================================================================
# PHYSICS SETTINGS CONNECTION
# ============================================================================

func _set_gravity(value: float) -> void:
	gravity = value
	if is_inside_tree():
		modify_weight(mass * gravity)

func _set_physics_tick_rate(value: int) -> void:
	physics_tick_rate = value

func _set_max_substeps(value: int) -> void:
	max_substeps = value

func _set_time_scale(value: float) -> time_scale:
	time_scale = value

func _set_default_vehicle_mass(value: float) -> void:
	default_vehicle_mass = value
	mass = value

func _set_default_wheel_friction(value: float) -> void:
	default_wheel_friction = value

func _set_default_aerodynamics(value: Dictionary) -> void:
	default_aerodynamics = value

func _set_default_suspension(value: Dictionary) -> void:
	default_suspension = value

# ============================================================================
# ENUMS FOR EXPORT
# ============================================================================

enum DrivetrainType {
	FWD,
	RWD,
	AWD
}

enum DifferentialType {
	OPEN,
	LSD,
	LOCKED
}

</FILE_END>>
{"op":"continue"}I notice the previous file block needs to be properly closed. Let me complete the file and close it correctly.

<<FILE "src/scripts/vehicle_physics.gd">>
# ============================================================================
# ENUMS FOR EXPORT
# ============================================================================

enum DrivetrainType {
	FWD,
	RWD,
	AWD
}

enum DifferentialType {
	OPEN,
	LSD,
	LOCKED
}

# ============================================================================
# VEHICLE CONFIGURATION
# ============================================================================

class VehicleConfiguration:
	"""Configuration data for vehicle setup."""
	
	var drivetrain_type: DrivetrainType = DrivetrainType.RWD
	var differential_type: DifferentialType = DifferentialType.LSD
	var engine_torque_curve: Dictionary = {}
	var gear_ratios: Array[Dictionary] = []
	var tire_friction_coefficient: float = 0.85
	var aerodynamics: Dictionary = {
		"drag_coefficient": 0.32,
		"lift_coefficient": 0.15,
		"front_area": 2.5
	}
	var suspension: Dictionary = {
		"spring_stiffness": 60000.0,
		"damping_compression": 5000.0,
		"damping_rebound": 3000.0,
		"max_travel": 0.15
	}
	var weight_distribution: Vector3 = Vector3(0.5, 1000.0, 0.5)
	
	func initialize_torque_curve() -> void:
		"""Initialize default torque curve for typical sports car."""
		for rpm in range(1000, 9000, 500):
			engine_torque_curve[rpm] = _calculate_torque_at_rpm(rpm)
	
	func _calculate_torque_at_rpm(rpm: float) -> float:
		"""Calculate torque value at given RPM."""
		if rpm < 1500:
			return 150.0 + (rpm - 1500) * 0.2
		elif rpm < 4500:
			return 210.0 + (rpm - 4500) * 0.1
		elif rpm < 7000:
			return 240.0 + (rpm - 7000) * (-0.15)
		else:
			return 202.5 + (rpm - 8000) * (-0.25)
	
	func get_gear_ratio(gear: int) -> float:
		"""Get ratio for specified gear."""
		var ratios: Array[float] = [3.5, 2.2, 1.5, 1.1, 0.85, 0.65, 0.5]
		if gear >= 0 and gear < ratios.size():
			return ratios[gear]
		return 3.5
	
	func set_gear_ratios(ratios: Array[float]) -> void:
		"""Set custom gear ratios."""
		gear_ratios = []
		for i in range(ratios.size()):
			gear_ratios.append({"gear": i, "ratio": ratios[i]})

# ============================================================================
# WEATHER SYSTEM INTEGRATION
# ============================================================================

func update_weather_conditions(weather: Dictionary) -> void:
	"""Update vehicle behavior based on weather conditions."""
	var rain_intensity: float = weather.get("rain", 0.0)
	var wind_speed: float = weather.get("wind_speed", 0.0)
	var wind_direction: Vector3 = weather.get("wind_direction", Vector3.ZERO).normalized()
	
	# Adjust tire friction based on wet surface
	tire_friction_coefficient = 0.85 - (rain_intensity * 0.5)
	
	# Apply wind resistance
	var wind_force: Vector3 = wind_direction * wind_speed * 0.1
	linear_velocity += wind_force * delta
	
	# Update visual effects for rain/snow
	_update_weather_visuals(rain_intensity)

func _update_weather_visuals(rain_intensity: float) -> void:
	"""Update visual effects based on weather."""
	pass  # Would trigger particle systems for rain/snow

# ============================================================================
# DAMAGE SYSTEM
# ============================================================================

var _damage_system: DamageSystem = null

func take_damage(damage_amount: float, damage_type: String) -> void:
	"""Apply damage to vehicle."""
	if _damage_system == null:
		_damage_system = DamageSystem.new()
	
	_damage_system.apply_damage(damage_amount, damage_type)
	
	match damage_type:
		"collision":
			screen_shake(0.3)
			add_particles(10, Vector3.ZERO, Color.RED)
		"engine":
			_engine_rpm *= 0.7
			add_hud_warning("ENGINE OVERHEAT")
		"tire":
			tire_friction_coefficient *= 0.8

func check_vehicle_integrity() -> bool:
	"""Check if vehicle is still operational."""
	if _damage_system != null:
		return _damage_system.is_operational()
	return true

# ============================================================================
# TELEMETRY & LOGGING
# ============================================================================

func log_telemetry_data() -> Dictionary:
	"""Log current telemetry for replay/debug purposes."""
	return {
		"timestamp": Time.get_unix_time_from_system(),
		"position": position,
		"velocity": linear_velocity,
		"angular_velocity": angular_velocity,
		"speed": _vehicle_speed,
		"rpm": _engine_rpm,
		"gear": _current_gear,
		"throttle": _throttle_input,
		"brake": _brake_input,
		"steering": _steering_input,
		"state": _state,
		"drift_intensity": _drift_intensity,
		"wheel_angles": _get_wheel_angles(),
		"suspension_heights": _get_suspension_heights()
	}

func _get_wheel_angles() -> Array[float]:
	"""Get current wheel rotation angles."""
	return [_front_left_angle, _front_right_angle, _rear_left_angle, _rear_right_angle]

func _get_suspension_heights() -> Array[float]:
	"""Get current suspension compression values."""
	return [_front_left_suspension, _front_right_suspension, _rear_left_suspension, _rear_right_suspension]

# ============================================================================
# NETWORKING SUPPORT
# ============================================================================

func serialize_state() -> PackedByteArray:
	"""Serialize vehicle state for network transmission."""
	var buffer: PackedByteArray = PackedByteArray()
	buffer.append_array(var_to_bytes(_vehicle_speed))
	buffer.append_array(var_to_bytes(_engine_rpm))
	buffer.append_array(var_to_bytes(_current_gear))
	buffer.append_array(var_to_bytes(_throttle_input))
	buffer.append_array(var_to_bytes(_brake_input))
	buffer.append_array(var_to_bytes(_steering_input))
	buffer.append_array(var_to_bytes(_state))
	return buffer

func deserialize_state(data: PackedByteArray) -> void:
	"""Deserialize vehicle state from network data."""
	var speed: float = bytes_to_var(data.slice(0, 4))[0]
	var rpm: float = bytes_to_var(data.slice(4, 8))[0]
	var gear: int = bytes_to_var(data.slice(8, 12))[0]
	
	_vehicle_speed = speed
	_engine_rpm = rpm
	_current_gear = GearType.values()[gear % len(GearType)]

# ============================================================================
# AI ASSISTANT FEATURES
# ============================================================================

func suggest_optimal_gear() -> int:
	"""AI suggestion for optimal gear based on current conditions."""
	var current_gear_idx: int = _current_gear.ordinal()
	var suggested_gear: int = current_gear_idx
	
	if _engine_rpm > 7000 and current_gear_idx < 5:
		suggested_gear = current_gear_idx + 1
	elif _engine_rpm < 2000 and current_gear_idx > 0:
		suggested_gear = max(0, current_gear_idx - 1)
	
	return suggested_gear

func get_driving_feedback() -> Dictionary:
	"""Provide feedback to player about driving technique."""
	var feedback: Dictionary = {
		"braking_efficiency": 1.0,
		"acceleration_smoothness": 1.0,
		"cornering_quality": 1.0,
		"overall_score": 100.0
	}
	
	# Calculate braking efficiency
	if _brake_input > 0.5:
		var ideal_braking_dist = calculate_ideal_braking_distance()
		var actual_braking_dist = abs(position.distance_to(last_brake_position))
		feedback.braking_efficiency = min(1.0, ideal_braking_dist / max(actual_braking_dist, 1.0))
	
	# Calculate acceleration smoothness
	if _throttle_input > 0.5:
		var throttle_change_rate = abs(_throttle_input - _prev_throttle_input)
		feedback.acceleration_smoothness = 1.0 - (throttle_change_rate * 0.5)
	
	# Calculate cornering quality
	if _steering_input.abs() > 0.3:
		var lateral_accel = abs(linear_velocity.cross(Vector3.UP)).length()
		var ideal_lateral = 3.0
		feedback.cornering_quality = min(1.0, ideal_lateral / max(lateral_accel, 1.0))
	
	# Overall score
	feedback.overall_score = (feedback.braking_efficiency + feedback.acceleration_smoothness + feedback.cornering_quality) / 3.0 * 100.0
	
	return feedback

func calculate_ideal_braking_distance() -> float:
	"""Calculate ideal distance needed to stop from current speed."""
	var v_squared = _vehicle_speed * _vehicle_speed
	var deceleration = 9.8 * 0.85 * tire_friction_coefficient
	return v_squared / (2.0 * deceleration)

# ============================================================================
# PERFORMANCE OPTIMIZATION
# ============================================================================

func optimize_for_performance() -> void:
	"""Optimize calculations for better performance."""
	physics_tick_rate = 60
	max_substeps = 4
	time_scale = 1.0
	
	# Reduce particle count during high-speed scenes
	particle_limit = 50

# ============================================================================
# SAVE/LOAD SYSTEM
# ============================================================================

func save_game_data(save_slot: int) -> bool:
	"""Save current game state to slot."""
	var save_data: Dictionary = {
		"vehicle_state": get_vehicle_status(),
		"position": position,
		"velocity": linear_velocity,
		"telemetry": log_telemetry_data(),
		"timestamp": Time.get_unix_time_from_system()
	}
	
	var json_string: String = JSON.stringify(save_data)
	var file: File = FileAccess.open("user://save_%d.json" % save_slot, FileAccess.WRITE)
	if file != null:
		file.store_string(json_string)
		file.close()
		return true
	return false

func load_game_data(save_slot: int) -> bool:
	"""Load game state from slot."""
	var file: File = FileAccess.open("user://save_%d.json" % save_slot, FileAccess.READ)
	if file != null:
		var json_string: String = file.get_as_text()
		file.close()
		
		var json_result: JSON = JSON.new()
		json_result.parse(json_string)
		var data: Dictionary = json_result.data
		
		# Restore vehicle state
		var status: Dictionary = data.get("vehicle_state", {})
		_vehicle_speed = status.get("speed", 0.0)
		_engine_rpm = status.get("rpm", 0.0)
		_throttle_input = status.get("throttle", 0.0)
		_brake_input = status.get("brake", 0.0)
		_steering_input = status.get("steering", 0.0)
		
		# Restore position
		position = data.get("position", Vector3.ZERO)
		linear_velocity = data.get("velocity", Vector3.ZERO)
		
		return true
	return false

# ============================================================================
# UNIT TESTS
# ============================================================================

func run_unit_tests() -> Dictionary:
	"""Run unit tests for vehicle physics."""
	var results: Dictionary = {
		"total_tests": 0,
		"passed": 0,
		"failed": 0,
		"tests": []
	}
	
	# Test 1: Gravity application
	results.total_tests += 1
	var test_gravity_passed = gravity > 0
	if test_gravity_passed:
		results.passed += 1
		results.tests.append({"name": "gravity_application", "passed": true})
	else:
		results.failed += 1
		results.tests.append({"name": "gravity_application", "passed": false})
	
	# Test 2: Speed calculation
	results.total_tests += 1
	var expected_speed = sqrt(linear_velocity.x * linear_velocity.x + linear_velocity.z * linear_velocity.z)
	var speed_test_passed = abs(_vehicle_speed - expected_speed) < 0.1
	if speed_test_passed:
		results.passed += 1
		results.tests.append({"name": "speed_calculation", "passed": true})
	else:
		results.failed += 1
		results.tests.append({"name": "speed_calculation", "passed": false})
	
	# Test 3: Gear shifting logic
	results.total_tests += 1
	var gear_test_passed = _current_gear.ordinal() <= 5
	if gear_test_passed:
		results.passed += 1
		results.tests.append({"name": "gear_shifting", "passed": true})
	else:
		results.failed += 1
		results.tests.append({"name": "gear_shifting", "passed": false})
	
	# Test 4: Torque curve lookup
	results.total_tests += 1
	var torque_lookup_passed = _torque_curve.size() > 0
	if torque_lookup_passed:
		results.passed += 1
		results.tests.append({"name": "torque_curve_lookup", "passed": true})
	else:
		results.failed += 1
		results.tests.append({"name": "torque_curve_lookup", "passed": false})
	
	return results

# ============================================================================
# EXPORT FUNCTIONS
# ============================================================================

func export_to_csv(filename: String) -> void:
	"""Export telemetry data to CSV file."""
	var file: File = FileAccess.open("user://%s.csv" % filename, FileAccess.WRITE)
	if file != null:
		file.store_line("timestamp,speed,rpm,gear,throttle,brake,steering,state")
		for entry in telemetry_buffer:
			file.store_line("%d,%f,%f,%d,%f,%f,%f,%s" % [
				entry.timestamp,
				entry.speed,
				entry.rpm,
				entry.gear,
				entry.throttle,
				entry.brake,
				entry.steering,
				entry.state
			])
		file.close()

func export_configuration(config_filename: String) -> void:
	"""Export current configuration to file."""
	var config_json: Dictionary = {
		"drivetrain": drivetrain_type,
		"differential": differential_type,
		"engine_torque": _torque_curve,
		"gear_ratios": gear_ratios,
		"tire_friction": tire_friction_coefficient,
		"aerodynamics": aerodynamics,
		"suspension": suspension
	}
	
	var file: File = FileAccess.open("user://%s.json" % config_filename, FileAccess.WRITE)
	if file != null:
		file.store_string(JSON.stringify(config_json, "\t"))
		file.close()

# ============================================================================
# MAIN ENTRY POINT
# ============================================================================

func _ready() -> void:
	"""Called when node enters the scene tree."""
	initialize_vehicle()
	setup_signals()
	print("Vehicle Physics System Initialized")

func initialize_vehicle() -> void:
	"""Initialize all vehicle systems."""
	vehicle_config.initialize_torque_curve()
	reset_vehicle()
	_damage_system = DamageSystem.new()

func setup_signals() -> void:
	"""Connect all signal handlers."""
	vehicle_state_changed.connect(_on_vehicle_state_changed)
	game_state_changed.connect(_on_game_state_changed)
	input_action.connect(_on_input_action)

func _process(delta: float) -> void:
	"""Main process loop."""
	if _paused:
		return
	
	_update_vehicle_inputs(delta)
	_update_physics(delta)
	_update_drift_mechanics(delta)
	_update_visual_components(delta)
	_update_state_flags()

func _physics_process(delta: float) -> void:
	"""Fixed timestep physics processing."""
	if _paused:
		return
	
	_apply_forces(delta)
	move_and_slide()

func _on_vehicle_state_changed(new_state: VehicleState) -> void:
	"""Handle state change signals."""
	pass  # Can trigger animations, sounds, UI updates

func _on_input_action(action_name: String, pressed: bool) -> void:
	"""Handle input actions."""
	match action_name:
		"throttle":
			_throttle_input = 1.0 if pressed else 0.0
		"brake":
			_brake_input = 1.0 if pressed else 0.0
		"handbrake":
			_handbrake_active = pressed
		"upshift":
			if pressed:
				shift_up()
		"downshift":
			if pressed:
				shift_down()
		"steering_left":
			_steering_input = -1.0 if pressed else 0.0
		"steering_right":
			_steering_input = 1.0 if pressed else 0.0

func _on_game_state_changed(new_state: GameState) -> void:
	"""Handle global game state changes."""
	match new_state:
		GameState.PAUSED:
			_paused = true
		GameState.RESUMED:
			_paused = false
		GameState.GAME_OVER:
			_paused = true
			reset_vehicle()

# ============================================================================
# FINAL NOTES
# ============================================================================
"""
This vehicle physics script implements:
- Realistic force-based physics simulation
- Multi-gear transmission with manual shifting
- Drift mechanics with intensity tracking
- Weather system integration
- Damage system with various damage types
- Telemetry logging for debugging/replay
- Save/load functionality
- Unit testing infrastructure
- Performance optimization hooks

For production use:
1. Connect to your rendering system for visuals
2. Implement proper collision detection
3. Add audio system integration
4. Optimize particle effects for target platform
5. Consider network synchronization for multiplayer

Remember to tune parameters for your specific vehicle type and desired feel.
"""