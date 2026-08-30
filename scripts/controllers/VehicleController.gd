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
signal differential_locked(is_locked: bool)
signal wheel_slip_detected(wheel_index: int, slip_ratio: float)

# ============================================================================
# CONSTANTS & CONFIGURATION
# ============================================================================
const DEFAULT_MAX_SPEED: float = 300.0  # km/h
const DEFAULT_ACCELERATION: float = 8.0  # m/s²
const DEFAULT_BRAKING_FORCE: float = 15.0  # m/s²
const DEFAULT_STEERING_SPEED: float = 2.5  # rad/s
const DEFAULT_GEAR_RATIOS: Array[float] = [3.8, 2.4, 1.7, 1.3, 1.1, 0.9]
const DEFAULT_FINAL_DRIVE: float = 4.1
const DEFAULT_REVERSE_RATIO: float = 3.5
const DEFAULT_IDLE_RPM: float = 800.0
const DEFAULT_REDLINE_RPM: float = 7500.0
const DEFAULT_ENGINE_TORQUE: float = 450.0  # Nm
const WHEEL_BASE: float = 2.7  # meters

# ============================================================================
# PUBLIC PROPERTIES
# ============================================================================
@export_group("Vehicle Configuration")
@export var max_speed_kmh: float = DEFAULT_MAX_SPEED
@export var acceleration_rate: float = DEFAULT_ACCELERATION
@export var braking_force: float = DEFAULT_BRAKING_FORCE
@export var steering_sensitivity: float = 1.0
@export var mass: float = PhysicsSettings.default_vehicle_mass
@export var center_of_mass_offset: Vector3 = Vector3(0.0, 0.5, 0.0)

@export_group("Engine Settings")
@export var idle_rpm: float = DEFAULT_IDLE_RPM
@export var redline_rpm: float = DEFAULT_REDLINE_RPM
@export var engine_torque: float = DEFAULT_ENGINE_TORQUE
@export var torque_curve: Curve = null

@export_group("Gear Ratios")
@export var gear_ratios: Array[float] = DEFAULT_GEAR_RATIOS.duplicate()
@export var final_drive_ratio: float = DEFAULT_FINAL_DRIVE
@export var reverse_ratio: float = DEFAULT_REVERSE_RATIO

@export_group("Tire & Suspension")
@export var tire_friction_coefficient: float = 1.2
@export var suspension_stiffness: float = 50000.0
@export var suspension_damping: float = 5000.0
@export var suspension_compression_limit: float = 0.15
@export var suspension_extension_limit: float = 0.25

@export_group("Drivetrain")
@export var drivetrain_type: DrivetrainType = DrivetrainType.FWD
@export var limited_slip_diff_strength: float = 0.7
@export var open_diff_enabled: bool = true

@export_group("Aerodynamics")
@export var drag_coefficient: float = 0.30
@export var frontal_area: float = 2.2  # m²
@export var downforce_coefficient: float = 0.05
@export var wing_angle_degrees: float = 0.0

# ============================================================================
# ENUMERATIONS
# ============================================================================
enum DrivetrainType { FWD, RWD, AWD }
enum TransmissionType { MANUAL, AUTOMATIC, CVT }

# ============================================================================
# STATE VARIABLES
# ============================================================================
var current_gear: int = 0  # 0 = neutral, 1-N = forward gears, -1 = reverse
var current_rpm: float = DEFAULT_IDLE_RPM
var current_speed_kmh: float = 0.0
var current_throttle: float = 0.0
var current_brake: float = 0.0
var current_steering: float = 0.0
var handbrake_active: bool = false
var traction_control_enabled: bool = true
var differential_locked: bool = false

# Gear shift state tracking
var _gear_shift_target: int = 0
var _gear_shift_timer: float = 0.0
var _gear_shift_duration: float = 0.2  # seconds per shift

# RPM management
var _engine_max_rpm: float = DEFAULT_REDLINE_RPM
var _engine_min_rpm: float = DEFAULT_IDLE_RPM
var _torque_multiplier: float = 1.0

# Skid detection
var _wheel_slip_ratios: Array[float] = [0.0, 0.0, 0.0, 0.0]
var _is_skidding: bool = false
var _skid_threshold: float = 0.15  # 15% slip ratio

# Aerodynamic forces
var _aero_downforce: float = 0.0
var _aero_drag: float = 0.0

# Collision tracking
var _last_collision_time: float = 0.0
var _collision_impact: Vector3 = Vector3.ZERO
var _collision_velocity: Vector3 = Vector3.ZERO

# Input buffer for smooth control
var _input_buffer: Dictionary = {}
var _input_smoothing: float = 0.15

# ============================================================================
# INITIALIZATION
# ============================================================================
func _ready() -> void:
	_process_mode = ProcessModeEnum.ALWAYS
	process_priority = 100
	
	# Initialize input buffer
	_input_buffer["throttle"] = 0.0
	_input_buffer["brake"] = 0.0
	_input_buffer["steering"] = 0.0
	
	# Set initial gear to neutral
	current_gear = 0
	_current_gear = 0
	
	# Apply center of mass offset
	if has_node("RigidBody3D"):
		var rb = get_node("RigidBody3D")
		rb.center_of_mass = center_of_mass_offset
	
	# Connect to GameManager signals if available
	if GameManager:
		GameManager.game_state_changed.connect(_on_game_state_changed)

func _physics_process(delta: float) -> void:
	_update_input_buffer(delta)
	_apply_physics(delta)
	_update_visual_state()

# ============================================================================
# INPUT HANDLING
# ============================================================================
func _update_input_buffer(delta: float) -> void:
	# Get raw inputs from InputManager
	var input_manager = InputManager if InputManager else null
	if not input_manager:
		return
	
	# Throttle input (0.0 to 1.0)
	var raw_throttle = input_manager.get_axis("accelerate", "brake")
	if raw_throttle > 0:
		raw_throttle = 1.0
	elif raw_throttle < 0:
		raw_throttle = -1.0
	else:
		raw_throttle = 0.0
	
	# Steering input (-1.0 to 1.0)
	var raw_steering = input_manager.get_axis("turn_left", "turn_right")
	
	# Brake/Holdbrake override
	var raw_brake = input_manager.get_axis("brake", "handbrake")
	if input_manager.is_action_pressed("handbrake"):
		raw_brake = 1.0
		handbrake_active = true
	else:
		handbrake_active = false
	
	# Smooth input transitions
	_input_buffer["throttle"] = lerp(_input_buffer["throttle"], raw_throttle, _input_smoothing)
	_input_buffer["brake"] = lerp(_input_buffer["brake"], raw_brake, _input_smoothing)
	_input_buffer["steering"] = lerp(_input_buffer["steering"], raw_steering, _input_smoothing * steering_sensitivity)
	
	# Update public properties
	current_throttle = clamp(_input_buffer["throttle"], 0.0, 1.0)
	current_brake = clamp(_input_buffer["brake"], 0.0, 1.0)
	current_steering = clamp(_input_buffer["steering"], -1.0, 1.0)

# ============================================================================
# PHYSICS UPDATE
# ============================================================================
func _apply_physics(delta: float) -> void:
	# Calculate target RPM based on gear and speed
	_target_rpm = _calculate_target_rpm()
	
	# Apply engine torque
	_apply_engine_torque(delta)
	
	# Apply braking forces
	_apply_braking(delta)
	
	# Handle gear shifting
	_handle_gear_shifting(delta)
	
	# Calculate aerodynamic forces
	_calculate_aerodynamics()
	
	# Apply drag force
	_apply_drag(delta)
	
	# Check for skidding
	_check_skid_condition()
	
	# Apply traction control if enabled
	_apply_traction_control()
	
	# Update velocity
	_update_velocity(delta)
	
	# Move with calculated velocity
	move_and_slide()

func _calculate_target_rpm() -> float:
	if current_gear == 0:
		return idle_rpm
	
	var gear_ratio: float = gear_ratios[current_gear - 1] if current_gear > 0 else reverse_ratio
	var wheel_speed_rps = current_speed_kmh / 3.6 / (2.0 * PI * 0.3)  # Assuming 0.3m wheel radius
	
	if current_gear > 0:
		return wheel_speed_rps * gear_ratio * final_drive_ratio * 60.0
	else:
		return wheel_speed_rps * reverse_ratio * final_drive_ratio * 60.0

func _apply_engine_torque(delta: float) -> void:
	if current_gear == 0:
		return
	
	var gear_ratio: float = gear_ratios[current_gear - 1] if current_gear > 0 else reverse_ratio
	var total_ratio = gear_ratio * final_drive_ratio
	
	# Calculate torque at wheels
	var wheel_torque = engine_torque * total_ratio * _torque_multiplier
	
	# Apply force based on drivetrain type
	match drivetrain_type:
		DrivetrainType.FWD:
			_apply_drive_force(wheel_torque, [0, 1])
		DrivetrainType.RWD:
			_apply_drive_force(wheel_torque, [2, 3])
		DrivetrainType.AWD:
			_apply_drive_force(wheel_torque * 0.5, [0, 1, 2, 3])

func _apply_drive_force(torque: float, wheel_indices: Array[int]) -> void:
	var wheel_radius: float = 0.3
	var drive_force = torque / wheel_radius
	
	for idx in wheel_indices:
		var wheel_direction = Vector3(1, 0, 0) if current_gear >= 0 else Vector3(-1, 0, 0)
		var wheel_position = _get_wheel_position(idx)
		apply_central_force(drive_force * wheel_direction)

func _apply_braking(delta: float) -> void:
	if current_brake <= 0.0 and not handbrake_active:
		return
	
	var braking_force = braking_force * mass * current_brake
	if handbrake_active:
		braking_force *= 1.5  # Handbrake provides extra force
	
	var brake_vector = Vector3(-1, 0, 0) * braking_force
	apply_central_force(brake_vector)

func _handle_gear_shifting(delta: float) -> void:
	# Automatic upshifts
	if current_gear < gear_ratios.size() and current_rpm >= _engine_max_rpm * 0.9:
		_attempt_gear_shift(current_gear + 1)
	
	# Automatic downshifts
	elif current_gear > 1 and current_rpm < _engine_min_rpm * 1.2:
		_attempt_gear_shift(current_gear - 1)
	
	# Manual gear shift override
	if _gear_shift_target != current_gear and _gear_shift_timer <= 0:
		_current_gear = _gear_shift_target
		_gear_shift_timer = _gear_shift_duration

func _attempt_gear_shift(target_gear: int) -> void:
	if target_gear <= 0 or target_gear > gear_ratios.size():
		return
	
	var old_gear = current_gear
	current_gear = target_gear
	emit_signal("gear_changed", old_gear, current_gear)
	emit_signal("rpm_changed", current_rpm)

# ============================================================================
# AERODYNAMICS
# ============================================================================
func _calculate_aerodynamics() -> void:
	var speed_ms = current_speed_kmh / 3.6
	
	# Drag force: Fd = 0.5 * ρ * Cd * A * v²
	var air_density: float = 1.225  # kg/m³ at sea level
	_aero_drag = 0.5 * air_density * drag_coefficient * frontal_area * pow(speed_ms, 2)
	
	# Downforce: Fd = 0.5 * ρ * Cl * A * v²
	_aero_downforce = 0.5 * air_density * downforce_coefficient * frontal_area * pow(speed_ms, 2)

func _apply_drag(delta: float) -> void:
	var speed_vector = velocity
	var speed_mag = speed_vector.length()
	
	if speed_mag > 0.1:
		var drag_force = _aero_drag / speed_mag
		var drag_direction = -speed_vector.normalized()
		
		# Apply drag as opposite to velocity
		var drag_vector = drag_direction * drag_force
		
		# Apply downforce to increase tire grip
		var downforce_vector = Vector3(0, -_aero_downforce, 0)
		
		apply_central_force(drag_vector + downforce_vector)

# ============================================================================
# SKID DETECTION & TRACTION CONTROL
# ============================================================================
func _check_skid_condition() -> void:
	var total_slip = 0.0
	var slip_count = 0
	
	for slip_ratio in _wheel_slip_ratios:
		total_slip += abs(slip_ratio)
		slip_count += 1
	
	var avg_slip = total_slip / slip_count if slip_count > 0 else 0.0
	
	if avg_slip > _skid_threshold:
		_is_skidding = true
		emit_signal("skidding", true)
		
		for i in range(_wheel_slip_ratios.size()):
			emit_signal("wheel_slip_detected", i, _wheel_slip_ratios[i])
	else:
		_is_skidding = false
		emit_signal("skidding", false)

func _apply_traction_control() -> void:
	if not traction_control_enabled or not _is_skidding:
		return
	
	# Reduce engine torque when skidding detected
	_torque_multiplier = 0.5

# ============================================================================
# GEAR SHIFT MANAGEMENT
# ============================================================================
func shift_up() -> void:
	if current_gear < gear_ratios.size():
		_gear_shift_target = current_gear + 1
		_gear_shift_timer = 0.0

func shift_down() -> void:
	if current_gear > 1:
		_gear_shift_target = current_gear - 1
		_gear_shift_timer = 0.0

func set_gear(gear: int) -> void:
	if gear <= 0 or gear > gear_ratios.size():
		current_gear = 0
	else:
		_gear_shift_target = gear
		_gear_shift_timer = 0.0

func reset_gear() -> void:
	current_gear = 0

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================
func get_wheel_position(index: int) -> Vector3:
	# Return approximate wheel positions relative to vehicle center
	match index:
		0: return Vector3(0.7, -0.3, 0.8)   # Front left
		1: return Vector3(0.7, -0.3, -0.8)  # Front right
		2: return Vector3(-0.7, -0.3, 0.8)  # Rear left
		3: return Vector3(-0.7, -0.3, -0.8) # Rear right
		_: return Vector3.ZERO

func get_speed_mps() -> float:
	return current_speed_kmh / 3.6

func get_speed_ms() -> float:
	return get_speed_mps()

func is_in_gear() -> bool:
	return current_gear != 0

func get_current_torque() -> float:
	var gear_ratio: float = gear_ratios[current_gear - 1] if current_gear > 0 else reverse_ratio
	return engine_torque * gear_ratio * final_drive_ratio * _torque_multiplier

func stall_engine() -> void:
	current_rpm = 0.0
	current_gear = 0
	emit_signal("engine_stalled")

func restart_engine() -> void:
	current_rpm = idle_rpm
	emit_signal("rpm_changed", current_rpm)

# ============================================================================
# COLLISION HANDLING
# ============================================================================
func _on_collision_entered(body: Node) -> void:
	var collision_info = {
		"body": body,
		"time": get_tree().time,
		"impact": _collision_impact,
		"velocity": _collision_velocity
	}
	
	emit_signal("collision_detected", collision_info)
	_last_collision_time = get_tree().time

func get_last_collision_info() -> Dictionary:
	return {
		"time": _last_collision_time,
		"impact": _collision_impact,
		"velocity": _collision_velocity
	}

# ============================================================================
# DEBUG & VISUALIZATION
# ============================================================================
func _update_visual_state() -> void:
	# Update visual indicators if available
	if has_node("VisualEffects"):
		var ve = get_node("VisualEffects")
		ve.update_state(self)

func debug_get_status() -> Dictionary:
	return {
		"gear": current_gear,
		"rpm": current_rpm,
		"speed_kmh": current_speed_kmh,
		"throttle": current_throttle,
		"brake": current_brake,
		"steering": current_steering,
		"handbrake": handbrake_active,
		"skidding": _is_skidding,
		"traction_control": traction_control_enabled,
		"differential_locked": differential_locked
	}

# ============================================================================
# SIGNAL CONNECTIONS
# ============================================================================
func _on_game_state_changed(new_state: GameState) -> void:
	match new_state:
		GameState.GAME_PAUSED:
			# Pause physics processing if needed
			pass
		GameState.GAME_RESUMED:
			# Resume physics processing
			pass
		_:
			pass

# ============================================================================
# SERIALIZE/DESERIALIZE FOR REPLAY SYSTEM
# ============================================================================
func serialize_vehicle_state() -> Dictionary:
	return {
		"gear": current_gear,
		"rpm": current_rpm,
		"speed_kmh": current_speed_kmh,
		"throttle": current_throttle,
		"brake": current_brake,
		"steering": current_steering,
		"position": global_position,
		"rotation": global_rotation,
		"velocity": velocity
	}

func deserialize_vehicle_state(state: Dictionary) -> void:
	current_gear = state.get("gear", 0)
	current_rpm = state.get("rpm", 0.0)
	current_speed_kmh = state.get("speed_kmh", 0.0)
	current_throttle = state.get("throttle", 0.0)
	current_brake = state.get("brake", 0.0)
	current_steering = state.get("steering", 0.0)
	
	global_position = state.get("position", global_position)
	global_rotation = state.get("rotation", global_rotation)
	velocity = state.get("velocity", Vector3.ZERO)

# ============================================================================
# EXPORT/IMPORT
# ============================================================================
func export_configuration() -> Dictionary:
	return {
		"max_speed_kmh": max_speed_kmh,
		"acceleration_rate": acceleration_rate,
		"braking_force": braking_force,
		"steering_sensitivity": steering_sensitivity,
		"mass": mass,
		"idle_rpm": idle_rpm,
		"redline_rpm": redline_rpm,
		"engine_torque": engine_torque,
		"gear_ratios": gear_ratios,
		"final_drive_ratio": final_drive_ratio,
		"drivetrain_type": drivetrain_type,
		"tire_friction_coefficient": tire_friction_coefficient
	}

func import_configuration(config: Dictionary) -> void:
	max_speed_kmh = config.get("max_speed_kmh", max_speed_kmh)
	acceleration_rate = config.get("acceleration_rate", acceleration_rate)
	braking_force = config.get("braking_force", braking_force)
	steering_sensitivity = config.get("steering_sensitivity", steering_sensitivity)
	mass = config.get("mass", mass)
	idle_rpm = config.get("idle_rpm", idle_rpm)
	redline_rpm = config.get("redline_rpm", redline_rpm)
	engine_torque = config.get("engine_torque", engine_torque)
	gear_ratios = config.get("gear_ratios", gear_ratios)
	final_drive_ratio = config.get("final_drive_ratio", final_drive_ratio)
	drivetrain_type = config.get("drivetrain_type", drivetrain_type)
	tire_friction_coefficient = config.get("tire_friction_coefficient", tire_friction_coefficient)
</File>