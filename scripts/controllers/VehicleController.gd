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
const GRAVITY := 9.81
const EARTH_VELOCITY_THRESHOLD := 0.1
const MIN_GEAR := 0  # Neutral
const MAX_GEAR := 7  # Maximum forward gear
const REVERSE_GEAR := -1

# ============================================================================
# ENUMERATIONS
# ============================================================================
enum DrivetrainType {
	FWD,    # Front-Wheel Drive
	RWD,    # Rear-Wheel Drive
	AWD     # All-Wheel Drive
}

enum EngineState {
	OFF,
	IDLE,
	RUNNING,
	REVING
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
@export var transmission_gears: Array[float] = [0.0, 3.8, 2.4, 1.7, 1.3, 1.0, 0.8, 0.6]: set = _set_transmission_gears

@export_group("Engine Settings")
@export var idle_rpm: float = 800.0
@export var redline_rpm: float = 7500.0
@export var engine_power_hp: float = 450.0: set = _set_engine_power_hp
@export var torque_curve: Dictionary = {}

@export_group("Physics Parameters")
@export var suspension_stiffness: float = 80000.0
@export var suspension_damping: float = 15000.0
@export var grip_coefficient: float = 1.2
@export var drag_coefficient: float = 0.32
@export var frontal_area: float = 2.2

@export_group("Drift Settings")
@export var drift_threshold_angle: float = 15.0
@export var drift_gravity_factor: float = 0.85
@export var drift_recovery_rate: float = 2.5

# ============================================================================
# PRIVATE PROPERTIES - Internal state management
# ============================================================================
var _current_speed_kmh: float = 0.0
var _current_rpm: float = 0.0
var _current_gear: int = 0
var _target_gear: int = 0
var _throttle_input: float = 0.0
var _brake_input: float = 0.0
var _steering_input: float = 0.0
var _handbrake_active: bool = false
var _engine_state: EngineState = EngineState.OFF
var _is_drifting: bool = false
var _drift_angle: float = 0.0
var _last_update_time: float = 0.0
var _wheel_forces: Vector3 = Vector3.ZERO
var _suspension_compression: Vector4 = Vector4.ONE * 0.5
var _airborne: bool = false

# References
var _powertrain_node: Node = null
var _collision_ray_casts: Array[RayCast3D] = []

# ============================================================================
# INITIALIZATION
# ============================================================================
func _ready() -> void:
	_process_mode = ProcessModeEnum.ALWAYS
	_init_references()
	_setup_collision_detection()
	_apply_defaults()
	
func _init_references() -> void:
	## Find child nodes that may exist in scene
	var children = get_children()
	for child in children:
		if child is Node and child.name.contains("Powertrain"):
			_powertrain_node = child
		elif child is RayCast3D:
			_collision_ray_casts.append(child)
			
func _setup_collision_detection() -> void:
	## Setup collision detection if not already done
	pass
	
func _apply_defaults() -> void:
	## Apply default values from PhysicsSettings singleton
	var settings = PhysicsSettings.get_singleton()
	if settings != null:
		vehicle_mass = settings.default_vehicle_mass
		
# ============================================================================
# MAIN PROCESS LOOP
# ============================================================================
func _process(delta: float) -> void:
	## Main game loop processing
	_current_speed_kmh = _calculate_current_speed()
	_current_rpm = _calculate_current_rpm()
	_handle_input(delta)
	_update_physics(delta)
	_handle_drift(delta)
	_emit_signals()
	
func _physics_process(delta: float) -> void:
	## Physics processing for rigid body interactions
	if is_on_floor():
		_airborne = false
	else:
		_airborne = true
	_apply_gravity(delta)
		
# ============================================================================
# INPUT HANDLING
# ============================================================================
func _handle_input(delta: float) -> void:
	## Process input from InputManager singleton
	var input_manager = InputManager.get_singleton()
	if input_manager == null:
		return
		
	## Get normalized inputs (0.0 to 1.0 for throttle/brake, -1.0 to 1.0 for steering)
	_throttle_input = clamp(input_manager.get_axis("throttle"), 0.0, 1.0)
	_brake_input = clamp(input_manager.get_axis("brake"), 0.0, 1.0)
	_steering_input = clamp(input_manager.get_axis("steer"), -1.0, 1.0)
	_handbrake_active = input_manager.is_action_pressed("handbrake")
	
	## Handle gear shifting
	_handle_gear_shifting(delta)
	
func _handle_gear_shifting(delta: float) -> void:
	## Automatic or manual gear shifting logic
	var next_gear = _determine_target_gear()
	
	if next_gear != _current_gear:
		_shift_gear(next_gear)
		
func _determine_target_gear() -> int:
	## Calculate target gear based on RPM and speed
	var target_gear = _current_gear
	
	if _current_rpm >= redline_rpm and _current_gear < MAX_GEAR:
		target_gear = min(_current_gear + 1, MAX_GEAR)
	elif _current_rpm <= idle_rpm and _current_gear > MIN_GEAR:
		target_gear = max(_current_gear - 1, MIN_GEAR)
		
	return target_gear
	
func _shift_gear(target: int) -> void:
	## Perform gear shift with transition
	var old_gear = _current_gear
	_current_gear = target
	
	gear_changed.emit(old_gear, target)
	
	if old_gear != target:
		## Temporary power reduction during shift
		_throttle_input *= 0.5
		
# ============================================================================
# PHYSICS CALCULATIONS
# ============================================================================
func _update_physics(delta: float) -> void:
	## Update vehicle physics state
	_calculate_wheel_forces()
	_apply_forces_to_body(delta)
	
func _calculate_current_speed() -> float:
	## Calculate current speed in km/h
	var velocity_magnitude = global_position.distance_to(Vector3(0, 0, 0)) / 1.0
	return velocity_magnitude * 3.6  ## Convert m/s to km/h
	
func _calculate_current_rpm() -> float:
	## Calculate current engine RPM based on gear and speed
	if _current_gear <= MIN_GEAR:
		return idle_rpm
		
	var gear_ratio = transmission_gears[_current_gear]
	var wheel_rpm = (_current_speed_kmh / 3.6) / (2.0 * PI * tire_radius) * 60.0
	return clamp(wheel_rpm * gear_ratio * final_drive_ratio, idle_rpm, redline_rpm)
	
func _calculate_wheel_forces() -> void:
	## Calculate forces applied to wheels
	_wheel_forces = Vector3.ZERO
	
	## Acceleration force (only applied to driven wheels)
	if _throttle_input > 0.0 and _engine_state == EngineState.RUNNING:
		var gear_ratio = transmission_gears[_current_gear] if _current_gear > MIN_GEAR else 1.0
		var effective_force = acceleration_force * _throttle_input * gear_ratio
		
		match drivetrain_type:
			DrivetrainType.FWD:
				_wheel_forces.x += effective_force
			DrivetrainType.RWD:
				_wheel_forces.x -= effective_force
			DrivetrainType.AWD:
				_wheel_forces.x += effective_force * 0.6
				
	## Braking force
	if _brake_input > 0.0 or _handbrake_active:
		var brake_effectiveness = 1.0 if _handbrake_active else 0.7
		_wheel_forces.x -= braking_force * _brake_input * brake_effectiveness
		
	## Drag force (air resistance)
	var drag_force = 0.5 * drag_coefficient * frontal_area * pow(_current_speed_kmh / 3.6, 2)
	_wheel_forces.x -= drag_force
		
	## Apply wheel forces to velocity
	global_velocity.x += (_wheel_forces.x / vehicle_mass) * 0.016  ## Scale for stability
		
func _apply_forces_to_body(delta: float) -> void:
	## Apply calculated forces to the character body
	pass
	
func _apply_gravity(delta: float) -> void:
	## Apply gravity when airborne
	if _airborne:
		global_velocity.y -= GRAVITY * delta
		
# ============================================================================
# DRIFT MECHANICS
# ============================================================================
func _handle_drift(delta: float) -> void:
	## Handle drift state and transitions
	var slip_angle = _calculate_slip_angle()
	
	if slip_angle > drift_threshold_angle and _throttle_input > 0.3:
		_start_drift(slip_angle)
	elif _is_drifting and slip_angle < drift_threshold_angle:
		_end_drift()
		
	if _is_drifting:
		_update_drift_state(delta)
		
func _calculate_slip_angle() -> float:
	## Calculate tire slip angle for drift detection
	var velocity_direction = global_velocity.normalized()
	var forward_direction = global_transform.basis.z.normalized()
	
	var dot_product = velocity_direction.dot(forward_direction)
	var slip_angle = rad_to_deg(acos(dot_product.clamp(-1.0, 1.0)))
	
	return slip_angle
	
func _start_drift(slip_angle: float) -> void:
	## Initiate drift mode
	_is_drifting = true
	_drift_angle = slip_angle
	drift_started.emit(slip_angle)
	
func _end_drift() -> void:
	## Exit drift mode
	_is_drifting = false
	drift_ended.emit()
	
func _update_drift_state(delta: float) -> void:
	## Update drift physics
	var drift_multiplier = _drift_gravity_factor + ((_drift_angle - drift_threshold_angle) / 90.0)
	
	## Reduce grip during drift
	var reduced_grip = grip_coefficient * drift_multiplier
	## Apply lateral force reduction for drift feel
	pass
	
# ============================================================================
# ENGINE MANAGEMENT
# ============================================================================
func start_engine() -> void:
	## Start the engine
	if _engine_state == EngineState.OFF:
		_engine_state = EngineState.IDLE
		engine_started.emit()
		
func stop_engine() -> void:
	## Stop the engine
	if _engine_state != EngineState.OFF:
		_engine_state = EngineState.OFF
		stop_vehicle()
		engine_stopped.emit()
		
func rev_engine() -> void:
	## Rev the engine (increase RPM without moving)
	if _engine_state in [EngineState.OFF, EngineState.IDLE]:
		_engine_state = EngineState.REVING
		_current_rpm = min(_current_rpm + 500.0, redline_rpm)
		
# ============================================================================
# VEHICLE CONTROL METHODS
# ============================================================================
func stop_vehicle() -> void:
	## Bring vehicle to a stop
	_throttle_input = 0.0
	_brake_input = 1.0
	_handbrake_active = true
	
func apply_brakes(force: float) -> void:
	## Apply brakes with specified force
	_brake_input = clamp(force, 0.0, 1.0)
	
func release_brakes() -> void:
	## Release all brakes
	_brake_input = 0.0
	_handbrake_active = false
	
func steer(amount: float) -> void:
	## Set steering amount (-1.0 to 1.0)
	_steering_input = clamp(amount, -1.0, 1.0)
	
func reset_controls() -> void:
	## Reset all controls to neutral
	_throttle_input = 0.0
	_brake_input = 0.0
	_steering_input = 0.0
	_handbrake_active = false
	
# ============================================================================
# STATE CHECKERS
# ============================================================================
func is_moving() -> bool:
	## Check if vehicle is in motion
	return _current_speed_kmh > 0.1
	
func is_in_gear() -> bool:
	## Check if vehicle has an active gear
	return _current_gear > MIN_GEAR
	
func can_shift_up() -> bool:
	## Check if upshifting is possible
	return _current_gear < MAX_GEAR and _current_rpm > idle_rpm
	
func can_shift_down() -> bool:
	## Check if downshifting is possible
	return _current_gear > MIN_GEAR
	
func is_at_redline() -> bool:
	## Check if engine is at or near redline
	return _current_rpm >= redline_rpm * 0.95
	
# ============================================================================
# EXTERNAL ACCESSORS
# ============================================================================
func get_current_speed() -> float:
	## Get current speed in km/h
	return _current_speed_kmh
	
func get_current_rpm() -> float:
	## Get current engine RPM
	return _current_rpm
	
func get_current_gear() -> int:
	## Get current gear number
	return _current_gear
	
func get_steering_angle() -> float:
	## Get current steering angle in degrees
	return _steering_input * steering_angle_max
	
func get_wheel_forces() -> Vector3:
	## Get calculated wheel forces
	return _wheel_forces
	
# ============================================================================
# SIGNAL EMITTERS
# ============================================================================
func _emit_signals() -> void:
	## Emit any changed signals
	speed_changed.emit(_current_speed_kmh)
	rpm_changed.emit(_current_rpm)
	vehicle_moved.emit(global_position, global_velocity)
	
# ============================================================================
# PROPERTY SETTERS - With validation and side effects
# ============================================================================
func _set_vehicle_mass(value: float) -> void:
	vehicle_mass = value
	# Mass changes affect inertia calculations
	
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
	
func _set_transmission_gears(value: Array[float]) -> void:
	transmission_gears = value
	
func _set_engine_power_hp(value: float) -> void:
	engine_power_hp = value
	
func _set_gravity(value: float) -> void:
	GRAVITY = value

# ============================================================================
# DEBUG & TESTING
# ============================================================================
func debug_get_statistics() -> Dictionary:
	## Return debug statistics for diagnostics
	return {
		"speed_kmh": _current_speed_kmh,
		"rpm": _current_rpm,
		"gear": _current_gear,
		"throttle": _throttle_input,
		"brake": _brake_input,
		"steering": _steering_input,
		"handbrake": _handbrake_active,
		"is_drifting": _is_drifting,
		"airborne": _airborne,
		"engine_state": _engine_state,
		"wheel_forces_x": _wheel_forces.x
	}

func debug_set_gear(gear: int) -> void:
	## Debug function to manually set gear
	_current_gear = clamp(gear, MIN_GEAR, MAX_GEAR)
	
func debug_reset_all() -> void:
	## Debug function to reset all states
	reset_controls()
	_current_gear = MIN_GEAR
	_current_rpm = idle_rpm
	_current_speed_kmh = 0.0
	_is_drifting = false
	_airborne = false