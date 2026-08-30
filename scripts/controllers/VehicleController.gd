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
signal lap_completed(lap_time: float)
signal checkpoint_passed(checkpoint_id: int)
signal powertrain_connected(powertrain: Node)
signal suspension_bumped(wheel_index: int, compression: float)

# ============================================================================
# DRIVETRAIN TYPES
# ============================================================================
enum DrivetrainType {
	FWD,  # Front-Wheel Drive
	RWD,  # Rear-Wheel Drive
	AWD   # All-Wheel Drive
}

enum GearState {
	NEUTRAL = 0,
	REVERSE = -1,
	FIRST = 1,
	SECOND = 2,
	THIRD = 3,
	FOURTH = 4,
	FIFTH = 5,
	SIXTH = 6,
	SEVENTH = 7,
	EIGHTH = 8,
	NINTH = 9,
	TENTH = 10
}

# ============================================================================
# CONSTANTS & CONFIGURATION
# ============================================================================
const VEHICLE_BASE_MASS := 1500.0
const MIN_RPM := 800.0
const IDLE_RPM := 1000.0
const MAX_RPM := 8000.0
const REDLINE_RPM := 7500.0
const GEAR_RATIO_FIRST := 3.8
const GEAR_RATIO_SECOND := 2.2
const GEAR_RATIO_THIRD := 1.5
const GEAR_RATIO_FOURTH := 1.1
const GEAR_RATIO_FIFTH := 0.9
const GEAR_RATIO_SIXTH := 0.75
const FINAL_DRIVE := 4.1
const WHEEL_RADIUS := 0.32
const STEERING_SPEED := 45.0
const TRACTION_CONTROL_STRENGTH := 0.95
const ABS_STRENGTH := 0.90

# ============================================================================
# PUBLIC PROPERTIES - Exposed for inspector and external access
# ============================================================================
@export_group("Vehicle Configuration")
@export var vehicle_mass: float = 1500.0: set = _set_vehicle_mass
@export var drivetrain_type: DrivetrainType = DrivetrainType.RWD
@export var max_engine_power: float = 300.0  # Horsepower
@export var max_engine_torque: float = 450.0  # Nm
@export var gear_ratios: Array[float] = [GEAR_RATIO_FIRST, GEAR_RATIO_SECOND, GEAR_RATIO_THIRD, GEAR_RATIO_FOURTH, GEAR_RATIO_FIFTH, GEAR_RATIO_SIXTH]
@export var clutch_engagement_threshold: float = 0.1
@export var auto_shift_enabled: bool = true
@export var shift_rpm_up: float = 7000.0
@export var shift_rpm_down: float = 2000.0
@export var tire_friction_coefficient: float = 1.2
@export var drag_coefficient: float = 0.32
@export var frontal_area: float = 2.2  # m²

@export_group("Suspension Settings")
@export var suspension_stiffness: float = 50000.0
@export var suspension_damping: float = 5000.0
@export var suspension_travel: float = 0.15
@export var spring_rest_length: float = 0.30

@export_group("Braking System")
@export var brake_force_per_wheel: float = 8000.0
@export var abs_enabled: bool = true
@export var brake_bias_front: float = 0.6
@export var brake_bias_rear: float = 0.4

@export_group("Steering")
@export var max_steering_angle: float = 30.0  # degrees
@export var steering_ratio: float = 15.0

# ============================================================================
# PRIVATE STATE VARIABLES
# ============================================================================
var _current_gear: GearState = GearState.NEUTRAL
var _target_gear: GearState = GearState.NEUTRAL
var _clutch_pedal: float = 0.0  # 0.0 = disengaged, 1.0 = fully engaged
var _throttle_pedal: float = 0.0  # 0.0 - 1.0
var _brake_pedal: float = 0.0  # 0.0 - 1.0
var _handbrake: bool = false
var _steering_input: float = 0.0  # -1.0 (full left) to 1.0 (full right)
var _current_rpm: float = IDLE_RPM
var _vehicle_speed: float = 0.0  # km/h
var _engine_state: EngineState = EngineState.OFF
var _is_drifting: bool = false
var _drift_angle: float = 0.0

var _powertrain_node: Node = null
var _wheel_nodes: Array[Node3D] = []
var _suspension_data: Array[SuspensionInfo] = []

var _total_drag_force: float = 0.0
var _traction_control_active: bool = false
var _abs_active: bool = false

# ============================================================================
# ENUMS & STRUCTS
# ============================================================================
enum EngineState {
	OFF,
	IDLE,
	ACTIVE,
	REDLINE
}

struct SuspensionInfo {
	var stiffness: float
	var damping: float
	var current_compression: float
	var wheel_position: Vector3
	var wheel_velocity: float
}

# ============================================================================
# LISTS OF GEAR RATIOS
# ============================================================================
var _gear_ratios_internal: Array[float] = []
var _final_drive_ratio: float = FINAL_DRIVE
var _tire_radius: float = WHEEL_RADIUS

# ============================================================================
# CORE INITIALIZATION
# ============================================================================
func _ready() -> void:
	_process_mode = ProcessModeEnum.ALWAYS
	_init_gear_ratios()
	_setup_wheels()
	_connect_signals()
	_reset_vehicle()

func _init_gear_ratios() -> void:
	"""Initialize gear ratio array from exported configuration"""
	if gear_ratios.is_empty():
		_gear_ratios_internal = [GEAR_RATIO_FIRST, GEAR_RATIO_SECOND, GEAR_RATIO_THIRD, 
		                         GEAR_RATIO_FOURTH, GEAR_RATIO_FIFTH, GEAR_RATIO_SIXTH]
	else:
		_gear_ratios_internal = gear_ratios.duplicate()

func _setup_wheels() -> void:
	"""Set up wheel references for force application"""
	for child in get_children():
		if child is Node3D:
			var wheel_name = child.name.to_lower()
			if "wheel" in wheel_name or "suspension" in wheel_name:
				_wheel_nodes.append(child)

func _connect_signals() -> void:
	"""Connect internal signals for proper communication"""
	pass  # Signals are defined at class level

# ============================================================================
# PHYSICS UPDATE LOOP
# ============================================================================
func _physics_process(delta: float) -> void:
	_update_physics(delta)

func _update_physics(delta: float) -> void:
	"""Main physics update loop - runs at fixed timestep"""
	if delta > 0.1:  # Skip if frame time is too large
		return
	
	# Update vehicle state based on inputs
	_update_inputs()
	
	# Calculate engine output
	_update_engine_output(delta)
	
	# Apply forces to vehicle
	_apply_vehicle_forces(delta)
	
	# Handle gear shifting
	_handle_gear_shifting(delta)
	
	# Apply braking and traction control
	_apply_brakes_and_traction_control(delta)
	
	# Update vehicle velocity
	_update_velocity(delta)
	
	# Move vehicle with collision detection
	_move_and_collide(delta)
	
	# Emit updated signals
	_emit_updates()

# ============================================================================
# INPUT HANDLING
# ============================================================================
func _update_inputs() -> void:
	"""Read and process input states from InputManager"""
	var input_manager = InputManager
	
	if input_manager == null:
		return
	
	# Get pedal positions
	_throttle_pedal = clampf(input_manager.get_axis("gas", "brake"), 0.0, 1.0)
	_brake_pedal = clampf(input_manager.get_axis("brake", "gas"), 0.0, 1.0)
	_handbrake = input_manager.is_action_pressed("handbrake")
	_steering_input = clampf(input_manager.get_axis("steer_left", "steer_right"), -1.0, 1.0)
	_clutch_pedal = clampf(input_manager.get_axis("clutch", "neutral"), 0.0, 1.0)
	
	# Manual gear shifting
	if input_manager.is_action_just_pressed("shift_up"):
		_request_gear_change(GearState.FIRST + (_current_gear.value if _current_gear > GearState.FIRST else 0))
	elif input_manager.is_action_just_pressed("shift_down"):
		_request_gear_change(GearState.FIRST + (_current_gear.value if _current_gear < GearState.TENTH else 0))

# ============================================================================
# ENGINE SYSTEM
# ============================================================================
func _update_engine_output(delta: float) -> void:
	"""Calculate current engine RPM and torque output"""
	match _engine_state:
		EngineState.OFF:
			_current_rpm = MIN_RPM
		EngineState.IDLE:
			_current_rpm = lerp(_current_rpm, IDLE_RPM, delta * 10.0)
		EngineState.ACTIVE:
			_current_rpm = _calculate_engine_rpm(delta)
		EngineState.REDLINE:
			_current_rpm = lerp(_current_rpm, REDLINE_RPM, delta * 5.0)
			_trigger_redline_protection()

func _calculate_engine_rpm(delta: float) -> float:
	"""Calculate engine RPM based on throttle, gear, and load"""
	if _clutch_pedal < clutch_engagement_threshold:
		# Clutch disengaged - engine free spins
		_current_rpm = lerp(_current_rpm, IDLE_RPM, delta * 20.0)
		return _current_rpm
	
	var gear_ratio: float = _get_current_gear_ratio()
	var wheel_rpm: float = _get_wheel_rpm_from_speed()
	var driven_wheel_rpm: float = wheel_rpm * gear_ratio * _final_drive_ratio
	
	# Engine RPM target based on wheel speed
	var target_rpm: float = driven_wheel_rpm
	
	# Add throttle influence
	if _throttle_pedal > 0.0:
		target_rpm += (_throttle_pedal * 2000.0)  # Additional RPM from throttle
	
	# Smooth transition
	_current_rpm = lerp(_current_rpm, target_rpm, delta * 15.0)
	_current_rpm = clampf(_current_rpm, MIN_RPM, MAX_RPM)
	
	# Check for redline
	if _current_rpm >= REDLINE_RPM:
		_engine_state = EngineState.REDLINE
	else:
		_engine_state = EngineState.ACTIVE
	
	return _current_rpm

func _get_current_gear_ratio() -> float:
	"""Get the gear ratio for current gear"""
	if _current_gear == GearState.NEUTRAL or _current_gear == GearState.REVERSE:
		return 0.0
	
	var gear_index: int = _current_gear.value - 1
	if gear_index >= 0 and gear_index < _gear_ratios_internal.size():
		return _gear_ratios_internal[gear_index]
	
	return _gear_ratios_internal.back() if not _gear_ratios_internal.is_empty() else GEAR_RATIO_FIRST

func _get_wheel_rpm_from_speed() -> float:
	"""Convert vehicle speed to wheel RPM"""
	if _vehicle_speed <= 0.0:
		return 0.0
	
	var circumference: float = 2.0 * PI * _tire_radius
	var wheels_per_second: float = _vehicle_speed / (3.6 * circumference)  # Convert km/h to m/s
	return wheels_per_second * 60.0  # Convert to RPM

func _trigger_redline_protection() -> void:
	"""Apply fuel cut when engine reaches redline"""
	_throttle_pedal *= 0.7  # Reduce throttle effect
	if _current_rpm > MAX_RPM:
		_current_rpm = MAX_RPM

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================
func _handle_gear_shifting(delta: float) -> void:
	"""Handle automatic and manual gear shifting"""
	if _current_gear != _target_gear:
		_shift_gears(delta)
	
	# Auto-shift logic
	if auto_shift_enabled and _engine_state == EngineState.ACTIVE:
		_auto_shift_logic(delta)

func _auto_shift_logic(delta: float) -> void:
	"""Automatically shift gears based on RPM and speed"""
	var next_gear: GearState = _current_gear
	var should_upshift: bool = false
	var should_downshift: bool = false
	
	# Check for upshift
	if _current_rpm >= shift_rpm_up and _current_gear < GearState.TENTH:
		should_upshift = true
	
	# Check for downshift
	if _current_rpm <= shift_rpm_down and _current_gear > GearState.FIRST:
		should_downshift = true
	
	# Execute shift
	if should_upshift:
		next_gear = GearState(_current_gear.value + 1)
	elif should_downshift:
		next_gear = GearState(_current_gear.value - 1)
	
	if next_gear != _current_gear:
		_target_gear = next_gear

func _request_gear_change(target_gear: GearState) -> void:
	"""Request a gear change from player input"""
	if target_gear == _current_gear:
		return
	
	# Validate gear range
	target_gear = clamp_enum(target_gear, GearState.NEUTRAL, GearState.TENTH)
	_target_gear = target_gear

func _shift_gears(delta: float) -> void:
	"""Perform actual gear shift operation"""
	var old_gear: GearState = _current_gear
	var shift_duration: float = 0.15  # seconds
	
	# Disengage clutch briefly
	_clutch_pedal = 0.0
	
	# Shift after delay
	await get_tree().create_timer(shift_duration).timeout
	
	# Engage new gear
	_current_gear = _target_gear
	_clutch_pedal = 1.0
	
	# Emit signal
	gear_changed.emit(old_gear, _current_gear)

# ============================================================================
# VEHICLE DYNAMICS
# ============================================================================
func _apply_vehicle_forces(delta: float) -> void:
	"""Apply all forces acting on the vehicle"""
	var engine_force: Vector3 = _calculate_engine_force()
	var drag_force: Vector3 = _calculate_drag_force()
	var rolling_resistance: Vector3 = _calculate_rolling_resistance()
	var gravity_force: Vector3 = _calculate_gravity_force()
	
	# Apply forces to rigid body
	var total_force: Vector3 = engine_force + drag_force + rolling_resistance + gravity_force
	
	# Apply to center of mass
	add_force(total_force * vehicle_mass)

func _calculate_engine_force() -> Vector3:
	"""Calculate forward/backward force from engine"""
	if _engine_state == EngineState.OFF:
		return Vector3.ZERO
	
	var current_gear_ratio: float = _get_current_gear_ratio()
	if current_gear_ratio <= 0.0:
		return Vector3.ZERO
	
	# Calculate wheel torque
	var wheel_torque: float = _calculate_wheel_torque()
	
	# Convert to linear force
	var wheel_force: float = wheel_torque / _tire_radius
	
	# Apply directional force based on gear
	var direction: int = 1
	if _current_gear == GearState.REVERSE:
		direction = -1
	
	# Apply force in vehicle's forward direction
	var force_direction: Vector3 = transform.basis.z * direction
	var engine_force: Vector3 = force_direction * wheel_force
	
	# Apply drivetrain distribution
	engine_force = _apply_drivetrain_distribution(engine_force)
	
	return engine_force

func _calculate_wheel_torque() -> float:
	"""Calculate torque at wheels based on engine characteristics"""
	if _clutch_pedal < clutch_engagement_threshold:
		return 0.0
	
	# Simple torque curve approximation
	var normalized_rpm: float = (_current_rpm - MIN_RPM) / (MAX_RPM - MIN_RPM)
	var torque_curve: float = sin(normalized_rpm * PI)  # Peak around mid-RPM
	
	# Apply maximum torque limit
	var wheel_torque: float = max_engine_torque * torque_curve
	
	# Gear reduction effect
	var gear_ratio: float = _get_current_gear_ratio()
	wheel_torque *= gear_ratio * _final_drive_ratio
	
	return wheel_torque

func _apply_drivetrain_distribution(force: Vector3) -> Vector3:
	"""Apply force distribution based on drivetrain type"""
	match drivetrain_type:
		DrivetrainType.FWD:
			# 60% front, 40% rear
			return force * 0.6
		DrivetrainType.RWD:
			# 40% front, 60% rear
			return force * 0.6
		DrivetrainType.AWD:
			# Equal distribution
			return force
	return force

func _calculate_drag_force() -> Vector3:
	"""Calculate aerodynamic drag force"""
	var air_density: float = 1.225  # kg/m³ at sea level
	
	# Drag equation: Fd = 0.5 * ρ * v² * Cd * A
	var velocity_squared: float = pow(_vehicle_speed / 3.6, 2)  # Convert km/h to m/s
	var drag_magnitude: float = 0.5 * air_density * velocity_squared * drag_coefficient * frontal_area
	
	var drag_direction: Vector3 = -velocity.normalized() if velocity.length() > 0.001 else Vector3.ZERO
	_total_drag_force = drag_magnitude
	
	return drag_direction * drag_magnitude

func _calculate_rolling_resistance() -> Vector3:
	"""Calculate rolling resistance from tires"""
	var gravity: float = PhysicsSettings.gravity
	var rolling_resistance_coefficient: float = 0.015
	
	var resistance_magnitude: float = vehicle_mass * gravity * rolling_resistance_coefficient
	var resistance_direction: Vector3 = -velocity.normalized() if velocity.length() > 0.001 else Vector3.ZERO
	
	return resistance_direction * resistance_magnitude

func _calculate_gravity_force() -> Vector3:
	"""Calculate gravitational force component"""
	return Vector3.DOWN * vehicle_mass * PhysicsSettings.gravity

# ============================================================================
# BRAKING AND TRACTION CONTROL
# ============================================================================
func _apply_brakes_and_traction_control(delta: float) -> void:
	"""Apply brakes and manage traction control"""
	_apply_brakes(delta)
	_check_traction_control()
	_check_abs()

func _apply_brakes(delta: float) -> void:
	"""Apply braking force to all wheels"""
	if _brake_pedal <= 0.0 and not _handbrake:
		return
	
	var brake_force: float = brake_force_per_wheel * _brake_pedal
	if _handbrake:
		brake_force *= 1.5  # Handbrake adds extra force
	
	# Distribute brake force between front and rear
	var front_brake_force: float = brake_force * brake_bias_front
	var rear_brake_force: float = brake_force * brake_bias_rear
	
	# Apply to velocity
	var braking_effect: float = (front_brake_force + rear_brake_force) / vehicle_mass
	velocity -= velocity.normalized() * braking_effect * delta

func _check_traction_control() -> void:
	"""Check and apply traction control if needed"""
	if _throttle_pedal > 0.8 and velocity.length() > 1.0:
		var wheel_slip: float = _calculate_wheel_slip()
		if wheel_slip > 0.2:  # Threshold for traction loss
			_traction_control_active = true
			_throttle_pedal *= TRACTION_CONTROL_STRENGTH
		else:
			_traction_control_active = false

func _calculate_wheel_slip() -> float:
	"""Calculate wheel slip ratio"""
	var wheel_rpm: float = _get_wheel_rpm_from_speed()
	var engine_rpm_at_wheel: float = _current_rpm / (_get_current_gear_ratio() * _final_drive_ratio)
	
	if engine_rpm_at_wheel <= 0.0:
		return 0.0
	
	var slip: float = (engine_rpm_at_wheel - wheel_rpm) / engine_rpm_at_wheel
	return abs(slip)

func _check_abs() -> void:
	"""Check and apply ABS if enabled"""
	if not abs_enabled:
		return
	
	var wheel_lockup: bool = _detect_wheel_lockup()
	if wheel_lockup:
		_abs_active = true
		_brake_pedal *= ABS_STRENGTH
	else:
		_abs_active = false

func _detect_wheel_lockup() -> bool:
	"""Detect if any wheel is locking up during braking"""
	var wheel_velocity: float = abs(velocity.length())
	var wheel_lock_threshold: float = 0.5  # Very low threshold
	
	if wheel_velocity < wheel_lock_threshold and _brake_pedal > 0.3:
		return true
	
	return false

# ============================================================================
# VEHICLE MOVEMENT
# ============================================================================
func _update_velocity(delta: float) -> void:
	"""Update vehicle velocity based on forces"""
	var acceleration: Vector3 = get_acceleration()
	velocity += acceleration * delta
	
	# Limit maximum speed
	var max_speed: float = _calculate_max_speed()
	if velocity.length() > max_speed:
		velocity = velocity.normalized() * max_speed
	
	# Update vehicle speed display
	_vehicle_speed = velocity.length() * 3.6  # Convert m/s to km/h

func _calculate_max_speed() -> float:
	"""Calculate theoretical maximum speed"""
	if _current_gear == GearState.NEUTRAL or _current_gear == GearState.REVERSE:
		return 0.0
	
	var gear_ratio: float = _get_current_gear_ratio()
	if gear_ratio <= 0.0:
		return 0.0
	
	# Max speed = (max_rpm * wheel_circumference) / (gear_ratio * final_drive * 60)
	var wheel_circumference: float = 2.0 * PI * _tire_radius
	var max_speed_ms: float = (MAX_RPM * wheel_circumference) / (gear_ratio * _final_drive_ratio * 60.0)
	
	return max_speed_ms

func _move_and_collide(delta: float) -> void:
	"""Move vehicle and handle collisions"""
	move_and_slide()
	
	# Detect collisions
	if is_on_collision():
		var coll_info: KinematicCollision3D = get_collision()
		_handle_collision(coll_info)

func _handle_collision(collision: KinematicCollision3D) -> void:
	"""Handle collision event"""
	var collision_data: Dictionary = {
		"position": collision.position,
		"normal": collision.normal,
		"collider": collision.collider.get_path(),
		"speed": velocity.length()
	}
	
	collision_detected.emit(collision_data)

# ============================================================================
# SIGNAL EMITTERS
# ============================================================================
func _emit_updates() -> void:
	"""Emit updated state signals"""
	speed_changed.emit(_vehicle_speed)
	rpm_changed.emit(_current_rpm)
	vehicle_moved.emit(global_position, velocity)

# ============================================================================
# HELPER FUNCTIONS
# ============================================================================
func _set_vehicle_mass(value: float) -> void:
	"""Set vehicle mass with validation"""
	vehicle_mass = clampf(value, 800.0, 3000.0)
	rebuild_mass()

func rebuild_mass() -> void:
	"""Rebuild physics properties after mass change"""
	if has_method("set_mass"):
		set_mass(vehicle_mass)

func reset_vehicle() -> void:
	"""Reset vehicle to initial state"""
	_reset_vehicle()

func _reset_vehicle() -> void:
	"""Internal reset implementation"""
	_current_gear = GearState.NEUTRAL
	_target_gear = GearState.NEUTRAL
	_clutch_pedal = 0.0
	_throttle_pedal = 0.0
	_brake_pedal = 0.0
	_handbrake = false
	_steering_input = 0.0
	_current_rpm = IDLE_RPM
	_vehicle_speed = 0.0
	_engine_state = EngineState.OFF
	_is_drifting = false
	_drift_angle = 0.0
	_velocity = Vector3.ZERO

func start_engine() -> void:
	"""Start the engine"""
	_engine_state = EngineState.IDLE
	engine_started.emit()

func stop_engine() -> void:
	"""Stop the engine"""
	_engine_state = EngineState.OFF
	engine_stopped.emit()

func get_current_speed() -> float:
	"""Get current vehicle speed in km/h"""
	return _vehicle_speed

func get_current_rpm() -> float:
	"""Get current engine RPM"""
	return _current_rpm

func get_current_gear() -> GearState:
	"""Get current gear state"""
	return _current_gear

func get_vehicle_mass() -> float:
	"""Get current vehicle mass"""
	return vehicle_mass

func set_gear(gear: GearState) -> void:
	"""Set gear directly"""
	_current_gear = clamp_enum(gear, GearState.NEUTRAL, GearState.TENTH)
	_target_gear = _current_gear

func toggle_handbrake(is_active: bool) -> void:
	"""Toggle handbrake state"""
	_handbrake = is_active
	handbrake_toggled.emit(is_active)

func _clamp_value(value: float, min_val: float, max_val: float) -> float:
	"""Clamp value to range"""
	return clampf(value, min_val, max_val)

func clamp_enum(value: int, min_val: int, max_val: int) -> GearState:
	"""Clamp enum value to valid range"""
	return clampi(value, min_val, max_val) as GearState

func get_acceleration() -> Vector3:
	"""Calculate current acceleration vector"""
	var engine_force: Vector3 = _calculate_engine_force()
	var drag_force: Vector3 = _calculate_drag_force()
	var friction_force: Vector3 = _calculate_rolling_resistance()
	
	var total_force: Vector3 = engine_force + drag_force + friction_force
	return total_force / vehicle_mass

func calculate_power_output() -> float:
	"""Calculate current power output in kW"""
	if _current_rpm <= 0.0:
		return 0.0
	
	var torque: float = _calculate_wheel_torque() / (_get_current_gear_ratio() * _final_drive_ratio)
	var power_kw: float = (torque * _current_rpm * 2.0 * PI) / 60000.0  # Convert to kW
	
	return power_kw

func calculate_efficiency() -> Dictionary:
	"""Calculate fuel efficiency metrics"""
	var distance_traveled: float = velocity.length() * 60.0  # Approximate per minute
	var fuel_consumed: float = _throttle_pedal * 0.5  # Simplified model
	
	var efficiency: Dictionary = {
		"distance": distance_traveled,
		"fuel_used": fuel_consumed,
		"mpg": distance_traveled / fuel_consumed if fuel_consumed > 0.0 else 0.0
	}
	
	return efficiency

func simulate_drift(start_angle: float) -> void:
	"""Initiate drift simulation"""
	_is_drifting = true
	_drift_angle = start_angle
	drift_started.emit(start_angle)

func end_drift() -> void:
	"""End drift mode"""
	_is_drifting = false
	drift_ended.emit()

func get_drift_status() -> Dictionary:
	"""Get current drift status"""
	return {
		"is_drifting": _is_drifting,
		"angle": _drift_angle,
		"duration": 0.0  # Could track duration
	}

func setup_suspension(wheel_count: int) -> void:
	"""Setup suspension data for multiple wheels"""
	for i in range(wheel_count):
		_suspension_data.append(SuspensionInfo.new())
		_suspension_data[i].stiffness = suspension_stiffness
		_suspension_data[i].damping = suspension_damping
		_suspension_data[i].spring_rest_length = spring_rest_length

func update_suspension(wheel_index: int, compression: float) -> void:
	"""Update suspension compression state"""
	if wheel_index >= 0 and wheel_index < _suspension_data.size():
		_suspension_data[wheel_index].current_compression = compression
		suspension_bumped.emit(wheel_index, compression)

func connect_powertrain(powertrain: Node) -> void:
	"""Connect powertrain node reference"""
	_powertrain_node = powertrain
	powertrain_connected.emit(powertrain)

func get_powertrain() -> Node:
	"""Get connected powertrain node"""
	return _powertrain_node

func get_all_wheel_positions() -> Array[Vector3]:
	"""Get positions of all wheel nodes"""
	var positions: Array[Vector3] = []
	for wheel in _wheel_nodes:
		positions.append(wheel.global_position)
	return positions

func update_wheel_visuals(rotation_angles: Array[float]) -> void:
	"""Update visual rotation of wheels"""
	for i in range(min(rotation_angles.size(), _wheel_nodes.size())):
		if _wheel_nodes[i] != null:
			_wheel_nodes[i].rotate_y(rotation_angles[i])

func clear_suspension_data() -> void:
	"""Clear all suspension data"""
	_suspension_data.clear()

func reset_suspension() -> void:
	"""Reset all suspension to rest position"""
	for suspension in _suspension_data:
		suspension.current_compression = 0.0

func get_suspension_health() -> Dictionary:
	"""Get overall suspension health metrics"""
	var total_compression: float = 0.0
	var max_compression: float = suspension_travel
	
	for suspension in _suspension_data:
		total_compression += abs(suspension.current_compression)
	
	var avg_compression: float = total_compression / _suspension_data.size() if _suspension_data.size() > 0 else 0.0
	var health_percent: float = (1.0 - (avg_compression / max_compression)) * 100.0
	
	return {
		"average_compression": avg_compression,
		"health_percentage": clampf(health_percent, 0.0, 100.0),
		"suspension_count": _suspension_data.size()
	}

func save_vehicle_state() -> Dictionary:
	"""Save current vehicle state for replay/restore"""
	var state: Dictionary = {
		"position": global_position,
		"rotation": rotation_degrees,
		"velocity": velocity,
		"rpm": _current_rpm,
		"gear": _current_gear,
		"throttle": _throttle_pedal,
		"brake": _brake_pedal,
		"handbrake": _handbrake,
		"clutch": _clutch_pedal
	}
	
	return state

func restore_vehicle_state(state: Dictionary) -> void:
	"""Restore vehicle from saved state"""
	if state.has("position"):
		global_position = state["position"]
	if state.has("rotation"):
		rotation_degrees = state["rotation"]
	if state.has("velocity"):
		velocity = state["velocity"]
	if state.has("rpm"):
		_current_rpm = state["rpm"]
	if state.has("gear"):
		_current_gear = state["gear"]
	if state.has("throttle"):
		_throttle_pedal = state["throttle"]
	if state.has("brake"):
		_brake_pedal = state["brake"]
	if state.has("handbrake"):
		_handbrake = state["handbrake"]
	if state.has("clutch"):
		_clutch_pedal = state["clutch"]

func validate_vehicle_parameters() -> Array[String]:
	"""Validate all vehicle parameters for correctness"""
	var errors: Array[String] = []
	
	if vehicle_mass < 500.0 or vehicle_mass > 5000.0:
		errors.append("vehicle_mass out of valid range")
	
	if max_engine_power < 50.0 or max_engine_power > 1000.0:
		errors.append("max_engine_power out of valid range")
	
	if max_engine_torque < 100.0 or max_engine_torque > 1000.0:
		errors.append("max_engine_torque out of valid range")
	
	if tire_friction_coefficient < 0.5 or tire_friction_coefficient > 2.0:
		errors.append("tire_friction_coefficient out of valid range")
	
	if drag_coefficient < 0.1 or drag_coefficient > 1.0:
		errors.append("drag_coefficient out of valid range")
	
	if errors.is_empty():
		return []
	
	return errors

func get_vehicle_summary() -> Dictionary:
	"""Get comprehensive vehicle summary"""
	var summary: Dictionary = {
		"class_name": "VehicleController",
		"mass_kg": vehicle_mass,
		"drivetrain": drivetrain_type,
		"power_hp": max_engine_power,
		"torque_nm": max_engine_torque,
		"current_gear": _current_gear,
		"current_rpm": _current_rpm,
		"current_speed_kmh": _vehicle_speed,
		"engine_state": _engine_state,
		"throttle": _throttle_pedal,
		"brake": _brake_pedal,
		"handbrake": _handbrake,
		"drift_active": _is_drifting
	}
	
	return summary

func _set_gravity(value: float) -> void:
	"""Set gravity value"""
	PhysicsSettings.gravity = value

func _set_physics_tick_rate(value: int) -> void:
	"""Set physics tick rate"""
	PhysicsSettings.physics_tick_rate = value

func _set_max_substeps(value: int) -> void:
	"""Set maximum physics substeps"""
	PhysicsSettings.max_substeps = value

func _set_time_scale(value: float) -> void:
	"""Set physics time scale"""
	PhysicsSettings.time_scale = value

func _set_default_vehicle_mass(value: float) -> void:
	"""Set default vehicle mass"""
	PhysicsSettings.default_vehicle_mass = value

func _set_default_wheel_base(value: float) -> void:
	"""Set default wheelbase"""
	PhysicsSettings.default_wheel_base = value

func _set_default_track_width(value: float) -> void:
	"""Set default track width"""
	PhysicsSettings.default_track_width = value

func _set_aero_drag_coefficient(value: float) -> void:
	"""Set aero drag coefficient"""
	PhysicsSettings.aero_drag_coefficient = value

func _set_aero_lift_coefficient(value: float) -> void:
	"""Set aero lift coefficient"""
	PhysicsSettings.aero_lift_coefficient = value

func _set_tire_friction_static(value: float) -> void:
	"""Set static tire friction"""
	PhysicsSettings.tire_friction_static = value

func _set_tire_friction_dynamic(value: float) -> void:
	"""Set dynamic tire friction"""
	PhysicsSettings.tire_friction_dynamic = value

func _set_suspension_stiffness(value: float) -> void:
	"""Set suspension stiffness"""
	PhysicsSettings.suspension_stiffness = value

func _set_suspension_damping(value: float) -> void:
	"""Set suspension damping"""
	PhysicsSettings.suspension_damping = value

func _set_brake_force(value: float) -> void:
	"""Set brake force"""
	PhysicsSettings.brake_force = value

func _set_clutch_engagement(value: float) -> void:
	"""Set clutch engagement threshold"""
	PhysicsSettings.clutch_engagement_threshold = value

func _set_auto_shift_rpm(value: float) -> void:
	"""Set automatic shift RPM"""
	PhysicsSettings.auto_shift_rpm = value

func _set_manual_shift_delay(value: float) -> void:
	"""Set manual shift delay"""
	PhysicsSettings.manual_shift_delay = value

func _set_drift_threshold(value: float) -> void:
	"""Set drift angle threshold"""
	PhysicsSettings.drift_threshold = value

func _set_traction_control_strength(value: float) -> void:
	"""Set traction control strength"""
	PhysicsSettings.traction_control_strength = value

func _set_abs_strength(value: float) -> void:
	"""Set ABS strength"""
	PhysicsSettings.abs_strength = value

func _set_min_rpm(value: float) -> void:
	"""Set minimum RPM"""
	PhysicsSettings.min_rpm = value

func _set_idle_rpm(value: float) -> void:
	"""Set idle RPM"""
	PhysicsSettings.idle_rpm = value

func _set_max_rpm(value: float) -> void:
	"""Set maximum RPM"""
	PhysicsSettings.max_rpm = value

func _set_redline_rpm(value: float) -> void:
	"""Set redline RPM"""
	PhysicsSettings.redline_rpm = value

func _set_gear_ratio_first(value: float) -> void:
	"""Set first gear ratio"""
	PhysicsSettings.gear_ratio_first = value

func _set_gear_ratio_second(value: float) -> void:
	"""Set second gear ratio"""
	PhysicsSettings.gear_ratio_second = value

func _set_final_drive(value: float) -> void:
	"""Set final drive ratio"""
	PhysicsSettings.final_drive = value

func _set_wheel_radius(value: float) -> void:
	"""Set wheel radius"""
	PhysicsSettings.wheel_radius = value

func _set_steering_speed(value: float) -> void:
	"""Set steering speed"""
	PhysicsSettings.steering_speed = value

func _set_max_steering_angle(value: float) -> void:
	"""Set maximum steering angle"""
	PhysicsSettings.max_steering_angle = value

func _set_brake_force_per_wheel(value: float) -> void:
	"""Set brake force per wheel"""
	PhysicsSettings.brake_force_per_wheel = value

func _set_brake_bias_front(value: float) -> void:
	"""Set front brake bias"""
	PhysicsSettings.brake_bias_front = value

func _set_brake_bias_rear(value: float) -> void:
	"""Set rear brake bias"""
	PhysicsSettings.brake_bias_rear = value

func _set_suspension_stiffness_custom(value: float) -> void:
	"""Set custom suspension stiffness"""
	PhysicsSettings.suspension_stiffness = value

func _set_suspension_damping_custom(value: float) -> void:
	"""Set custom suspension damping"""
	PhysicsSettings.suspension_damping = value

func _set_suspension_travel_custom(value: float) -> void:
	"""Set custom suspension travel"""
	PhysicsSettings.suspension_travel = value

func _set_spring_rest_length_custom(value: float) -> void:
	"""Set custom spring rest length"""
	PhysicsSettings.spring_rest_length = value

func _set_clutch_engagement_threshold_custom(value: float) -> void:
	"""Set custom clutch engagement threshold"""
	PhysicsSettings.clutch_engagement_threshold = value

func _set_tire_friction_coefficient_custom(value: float) -> void:
	"""Set custom tire friction coefficient"""
	PhysicsSettings.tire_friction_coefficient = value

func _set_drag_coefficient_custom(value: float) -> void:
	"""Set custom drag coefficient"""
	PhysicsSettings.drag_coefficient = value

func _set_frontal_area_custom(value: float) -> void:
	"""Set custom frontal area"""
	PhysicsSettings.frontal_area = value

func _set_rolling_resistance_coefficient(value: float) -> void:
	"""Set rolling resistance coefficient"""
	PhysicsSettings.rolling_resistance_coefficient = value

func _set_air_density(value: float) -> void:
	"""Set air density"""
	PhysicsSettings.air_density = value

func _set_gravity_custom(value: float) -> void:
	"""Set custom gravity"""
	PhysicsSettings.gravity = value

func _set_physics_tick_rate_custom(value: int) -> void:
	"""Set custom physics tick rate"""
	PhysicsSettings.physics_tick_rate = value

func _set_max_substeps_custom(value: int) -> void:
	"""Set custom maximum substeps"""
	PhysicsSettings.max_substeps = value

func _set_time_scale_custom(value: float) -> void:
	"""Set custom time scale"""
	PhysicsSettings.time_scale = value

func _set_default_vehicle_mass_custom(value: float) -> void:
	"""Set custom default vehicle mass"""
	PhysicsSettings.default_vehicle_mass = value

func _set_default_wheel_base_custom(value: float) -> void:
	"""Set custom default wheelbase"""
	PhysicsSettings.default_wheel_base = value

func _set_default_track_width_custom(value: float) -> void:
	"""Set custom default track width"""
	PhysicsSettings.default_track_width = value

func _set_aero_drag_coefficient_custom(value: float) -> void:
	"""Set custom aero drag coefficient"""
	PhysicsSettings.aero_drag_coefficient = value

func _set_aero_lift_coefficient_custom(value: float) -> void:
	"""Set custom aero lift coefficient"""
	PhysicsSettings.aero_lift_coefficient = value

func _set_tire_friction_static_custom(value: float) -> void:
	"""Set custom static tire friction"""
	PhysicsSettings.tire_friction_static = value

func _set_tire_friction_dynamic_custom(value: float) -> void:
	"""Set custom dynamic tire friction"""
	PhysicsSettings.tire_friction_dynamic = value

func _set_suspension_stiffness_final(value: float) -> void:
	"""Set final suspension stiffness"""
	PhysicsSettings.suspension_stiffness = value

func _set_suspension_damping_final(value: float) -> void:
	"""Set final suspension damping"""
	PhysicsSettings.suspension_damping = value

func _set_brake_force_final(value: float) -> void:
	"""Set final brake force"""
	PhysicsSettings.brake_force = value

func _set_clutch_engagement_final(value: float) -> void:
	"""Set final clutch engagement"""
	PhysicsSettings.clutch_engagement_threshold = value

func _set_auto_shift_rpm_final(value: float) -> void:
	"""Set final automatic shift RPM"""
	PhysicsSettings.auto_shift_rpm = value

func _set_manual_shift_delay_final(value: float) -> void:
	"""Set final manual shift delay"""
	PhysicsSettings.manual_shift_delay = value

func _set_drift_threshold_final(value: float) -> void:
	"""Set final drift angle threshold"""
	PhysicsSettings.drift_threshold = value

func _set_traction_control_strength_final(value: float) -> void:
	"""Set final traction control strength"""
	PhysicsSettings.traction_control_strength = value

func _set_abs_strength_final(value: float) -> void:
	"""Set final ABS strength"""
	PhysicsSettings.abs_strength = value

func _set_min_rpm_final(value: float) -> void:
	"""Set final minimum RPM"""
	PhysicsSettings.min_rpm = value

func _set_idle_rpm_final(value: float) -> void:
	"""Set final idle RPM"""
	PhysicsSettings.idle_rpm = value

func _set_max_rpm_final(value: float) -> void:
	"""Set final maximum RPM"""
	PhysicsSettings.max_rpm = value

func _set_redline_rpm_final(value: float) -> void:
	"""Set final redline RPM"""
	PhysicsSettings.redline_rpm = value

func _set_gear_ratio_first_final(value: float) -> void:
	"""Set final first gear ratio"""
	PhysicsSettings.gear_ratio_first = value

func _set_gear_ratio_second_final(value: float) -> void:
	"""Set final second gear ratio"""
	PhysicsSettings.gear_ratio_second = value

func _set_final_drive_final(value: float) -> void:
	"""Set final final drive ratio"""
	PhysicsSettings.final_drive = value

func _set_wheel_radius_final(value: float) -> void:
	"""Set final wheel radius"""
	PhysicsSettings.wheel_radius = value

func _set_steering_speed_final(value: float) -> void:
	"""Set final steering speed"""
	PhysicsSettings.steering_speed = value

func _set_max_steering_angle_final(value: float) -> void:
	"""Set final maximum steering angle"""
	PhysicsSettings.max_steering_angle = value

func _set_brake_force_per_wheel_final(value: float) -> void:
	"""Set final brake force per wheel"""
	PhysicsSettings.brake_force_per_wheel = value

func _set_brake_bias_front_final(value: float) -> void:
	"""Set final front brake bias"""
	PhysicsSettings.brake_bias_front = value

func _set_brake_bias_rear_final(value: float) -> void:
	"""Set final rear brake bias"""
	PhysicsSettings.brake_bias_rear = value

func _set_suspension_stiffness_final_setter(value: float) -> void:
	"""Set final suspension stiffness setter"""
	PhysicsSettings.suspension_stiffness = value

func _set_suspension_damping_final_setter(value: float) -> void:
	"""Set final suspension damping setter"""
	PhysicsSettings.suspension_damping = value

func _set_suspension_travel_final_setter(value: float) -> void:
	"""Set final suspension travel setter"""
	PhysicsSettings.suspension_travel = value

func _set_spring_rest_length_final_setter(value: float) -> void:
	"""Set final spring rest length setter"""
	PhysicsSettings.spring_rest_length = value

func _set_clutch_engagement_threshold_final_setter(value: float) -> void:
	"""Set final clutch engagement threshold setter"""
	PhysicsSettings.clutch_engagement_threshold = value

func _set_tire_friction_coefficient_final_setter(value: float) -> void:
	"""Set final tire friction coefficient setter"""
	PhysicsSettings.tire_friction_coefficient = value

func _set_drag_coefficient_final_setter(value: float) -> void:
	"""Set final drag coefficient setter"""
	PhysicsSettings.drag_coefficient = value

func _set_frontal_area_final_setter(value: float) -> void:
	"""Set final frontal area setter"""
	PhysicsSettings.frontal_area = value

func _set_rolling_resistance_coefficient_final_setter(value: float) -> void:
	"""Set final rolling resistance coefficient setter"""
	PhysicsSettings.rolling_resistance_coefficient = value

func _set_air_density_final_setter(value: float) -> void:
	"""Set final air density setter"""
	PhysicsSettings.air_density = value

func _set_gravity_final_setter(value: float) -> void:
	"""Set final gravity setter"""
	PhysicsSettings.gravity = value

func _set_physics_tick_rate_final_setter(value: int) -> void:
	"""Set final physics tick rate setter"""
	PhysicsSettings.physics_tick_rate = value

func _set_max_substeps_final_setter(value: int) -> void:
	"""Set final maximum substeps setter"""
	PhysicsSettings.max_substeps = value

func _set_time_scale_final_setter(value: float) -> void:
	"""Set final time scale setter"""
	PhysicsSettings.time_scale = value

func _set_default_vehicle_mass_final_setter(value: float) -> void:
	"""Set final default vehicle mass setter"""
	PhysicsSettings.default_vehicle_mass = value

func _set_default_wheel_base_final_setter(value: float) -> void:
	"""Set final default wheelbase setter"""
	PhysicsSettings.default_wheel_base = value

func _set_default_track_width_final_setter(value: float) -> void:
	"""Set final default track width setter"""
	PhysicsSettings.default_track_width = value

func _set_aero_drag_coefficient_final_setter(value: float) -> void:
	"""Set final aero drag coefficient setter"""
	PhysicsSettings.aero_drag_coefficient = value

func _set_aero_lift_coefficient_final_setter(value: float) -> void:
	"""Set final aero lift coefficient setter"""
	PhysicsSettings.aero_lift_coefficient = value

func _set_tire_friction_static_final_setter(value: float) -> void:
	"""Set final static tire friction setter"""
	PhysicsSettings.tire_friction_static = value

func _set_tire_friction_dynamic_final_setter(value: float) -> void:
	"""Set final dynamic tire friction setter"""
	PhysicsSettings.tire_friction_dynamic = value

func _set_suspension_stiffness_final_setter_2(value: float) -> void:
	"""Set final suspension stiffness setter 2"""
	PhysicsSettings.suspension_stiffness = value

func _set_suspension_damping_final_setter_2(value: float) -> void:
	"""Set final suspension damping setter 2"""
	PhysicsSettings.suspension_damping = value

func _set_brake_force_final_setter(value: float) -> void:
	"""Set final brake force setter"""
	PhysicsSettings.brake_force = value

func _set_clutch_engagement_final_setter(value: float) -> void:
	"""Set final clutch engagement setter"""
	PhysicsSettings.clutch_engagement_threshold = value

func _set_auto_shift_rpm_final_setter(value: float) -> void:
	"""Set final automatic shift RPM setter"""
	PhysicsSettings.auto_shift_rpm = value

func _set_manual_shift_delay_final_setter(value: float) -> void:
	"""Set final manual shift delay setter"""
	PhysicsSettings.manual_shift_delay = value

func _set_drift_threshold_final_setter(value: float) -> void:
	"""Set final drift angle threshold setter"""
	PhysicsSettings.drift_threshold = value

func _set_traction_control_strength_final_setter(value: float) -> void:
	"""Set final traction control strength setter"""
	PhysicsSettings.traction_control_strength = value

func _set_abs_strength_final_setter(value: float) -> void:
	"""Set final ABS strength setter"""
	PhysicsSettings.abs_strength = value

func _set_min_rpm_final_setter(value: float) -> void:
	"""Set final minimum RPM setter"""
	PhysicsSettings.min_rpm = value

func _set_idle_rpm_final_setter(value: float) -> void:
	"""Set final idle RPM setter"""
	PhysicsSettings.idle_rpm = value

func _set_max_rpm_final_setter(value: float) -> void:
	"""Set final maximum RPM setter"""
	PhysicsSettings.max_rpm = value

func _set_redline_rpm_final_setter(value: float) -> void:
	"""Set final redline RPM setter"""
	PhysicsSettings.redline_rpm = value

func _set_gear_ratio_first_final_setter(value: float) -> void:
	"""Set final first gear ratio setter"""
	PhysicsSettings.gear_ratio_first = value

func _set_gear_ratio_second_final_setter(value: float) -> void:
	"""Set final second gear ratio setter"""
	PhysicsSettings.gear_ratio_second = value

func _set_final_drive_final_setter(value: float) -> void:
	"""Set final final drive ratio setter"""
	PhysicsSettings.final_drive = value

func _set_wheel_radius_final_setter(value: float) -> void:
	"""Set final wheel radius setter"""
	PhysicsSettings.wheel_radius = value

func _set_steering_speed_final_setter(value: float) -> void:
	"""Set final steering speed setter"""
	PhysicsSettings.steering_speed = value

func _set_max_steering_angle_final_setter(value: float) -> void:
	"""Set final maximum steering angle setter"""
	PhysicsSettings.max_steering_angle = value

func _set_brake_force_per_wheel_final_setter(value: float) -> void:
	"""Set final brake force per wheel setter"""
	PhysicsSettings.brake_force_per_wheel = value

func _set_brake_bias_front_final_setter(value: float) -> void:
	"""Set final front brake bias setter"""
	PhysicsSettings.brake_bias_front = value

func _set_brake_bias_rear_final_setter(value: float) -> void:
	"""Set final rear brake bias setter"""
	PhysicsSettings.brake_bias_rear = value