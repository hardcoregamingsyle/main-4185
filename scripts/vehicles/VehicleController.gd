extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulation
## Handles throttle, brake, steering, wheel forces, gear shifting, and vehicle dynamics
## Integrates with PhysicsSettings for centralized tuning
## Copyright 2026 Thalamus Racing Simulator Project

# ============================================================================
# SIGNALS
# ============================================================================

signal rpm_changed(current_rpm: float, max_rpm: float)
signal gear_changed(old_gear: int, new_gear: int)
signal vehicle_hit(impact_force: Vector3, impact_point: Vector3)
signal collision_detected(collision_type: String)
signal traction_lost()
signal traction_regained()

# ============================================================================
# CONSTANTS FROM PHYSICS SETTINGS
# ============================================================================

const GRAVITY := PhysicsSettings.gravity
const DEFAULT_MASS := PhysicsSettings.default_vehicle_mass
const MAX_SUBSTEPS := PhysicsSettings.max_substeps
const TIME_SCALE := PhysicsSettings.time_scale

# ============================================================================
# EXPORTED PHYSICS CONFIGURATION
# ============================================================================

@export_group("Vehicle Mass & Inertia")
@export var mass: float = DEFAULT_MASS: set = _set_mass
@export var center_of_mass_offset: Vector3 = Vector3.ZERO
@export var inertia_tensor: Vector3 = Vector3.ONE

@export_group("Engine & Powertrain")
@export var engine_max_rpm: float = 8000.0
@export var engine_min_rpm: float = 800.0
@export var idle_rpm: float = 900.0
@export var engine_torque_curve: Curve = null
@export var engine_power_curve: Curve = null

@export_group("Transmission & Gearing")
@export var number_of_gears: int = 6
@export var reverse_gear_ratio: float = -3.5
@export var first_gear_ratio: float = 3.8
@export var gear_ratios: Array[float] = [3.8, 2.4, 1.7, 1.3, 1.0, 0.85]
@export var final_drive_ratio: float = 3.5
@export var clutch_engagement_point: float = 0.7

@export_group("Throttle & Braking")
@export var throttle_response_curve: Curve = null
@export var brake_force_per_wheel: float = 8000.0
@export var brake_bias_front: float = 0.6
@export var brake_bias_rear: float = 0.4
@export var anti_lock_brake_system: bool = true
@export var electronic_stability_control: bool = true

@export_group("Steering & Suspension")
@export var max_steering_angle: float = 30.0 * deg_to_rad(1)
@export var steering_speed: float = 2.5
@export var steering_centering_force: float = 0.8
@export var suspension_stiffness: float = 35000.0
@export var suspension_damping: float = 3500.0
@export var suspension_travel: float = 0.15
@export var tire_friction_coefficient: float = 1.15
@export var tire_side_slip_coefficient: float = 0.85

@export_group("Aerodynamics")
@export var drag_coefficient: float = 0.32
@export var frontal_area: float = 2.2
@export var downforce_coefficient: float = 0.5
@export var wing_angle: float = 10.0 * deg_to_rad(1)

# ============================================================================
# WHEEL POSITIONS (local space relative to vehicle body)
# ============================================================================

@export_group("Wheel Configuration")
@export var front_track_width: float = 1.6
@export var rear_track_width: float = 1.65
@export var wheelbase: float = 2.65
@export var wheel_radius: float = 0.32
@export var wheel_positions: Dictionary = {}

# ============================================================================
# INTERNAL STATE VARIABLES
# ============================================================================

var _powertrain: Node = null
var _audio_manager: AudioManager = null
var _game_manager: GameManager = null

# Current state
var current_gear: int = 0  # 0 = neutral, -1 = reverse, 1-6 = forward gears
var target_gear: int = 0
var clutch_pedal: float = 1.0  # 0 = pressed, 1 = released
var clutch_engaged: bool = true
var rpm: float = idle_rpm
var vehicle_speed: float = 0.0
var acceleration: Vector3 = Vector3.ZERO
var angular_velocity: Vector3 = Vector3.ZERO

# Input states (0-1 range)
var throttle_input: float = 0.0
var brake_input: float = 0.0
var steering_input: float = 0.0  # -1 left, +1 right, 0 centered

# Wheel states
var wheel_rotation_angles: Array[float] = [0.0, 0.0, 0.0, 0.0]  # FL, FR, RL, RR
var wheel_angular_velocities: Array[float] = [0.0, 0.0, 0.0, 0.0]
var wheel_contact_forces: Array[Vector3] = []

# Force accumulators
var total_engine_force: float = 0.0
var total_brake_force: float = 0.0
var total_aero_drag: float = 0.0
var total_downforce: float = 0.0

# Traction control state
var wheels_spinning: bool = false
var previous_speed: float = 0.0
var slip_ratio: float = 0.0

# ============================================================================
# INITIALIZATION
# ============================================================================

func _ready() -> void:
	_init_references()
	_setup_wheels()
	_connect_signals()
	_reset_vehicle_state()

func _init_references() -> void:
	_powertrain = get_parent() if is_instance_valid(get_parent()) and get_parent().has_method("get_powertrain") else null
	
	if Engine.has_singleton("GameManager"):
		_game_manager = Engine.get_singleton("GameManager")
	else:
		_game_manager = get_tree().root.get_node_or_null("GameManager")
	
	if Engine.has_singleton("AudioManager"):
		_audio_manager = Engine.get_singleton("AudioManager")
	else:
		_audio_manager = get_tree().root.get_node_or_null("AudioManager")

func _setup_wheels() -> void:
	wheel_positions = {
		"front_left": Vector3(-front_track_width / 2, -wheel_radius, wheelbase / 2),
		"front_right": Vector3(front_track_width / 2, -wheel_radius, wheelbase / 2),
		"rear_left": Vector3(-rear_track_width / 2, -wheel_radius, -wheelbase / 2),
		"rear_right": Vector3(rear_track_width / 2, -wheel_radius, -wheelbase / 2)
	}
	
	wheel_contact_forces.resize(4)
	for i in range(4):
		wheel_contact_forces[i] = Vector3.ZERO

func _connect_signals() -> void:
	if is_instance_valid(_game_manager):
		_game_manager.game_state_changed.connect(_on_game_state_changed)
	
	if is_instance_valid(_powertrain):
		_powertrain.rpm_changed.connect(_on_powertrain_rpm_changed)
		_powertrain.engine_torque_available.connect(_on_engine_torque_available)

func _reset_vehicle_state() -> void:
	current_gear = 0
	target_gear = 0
	clutch_pedal = 1.0
	clutch_engaged = true
	rpm = idle_rpm
	vehicle_speed = 0.0
	acceleration = Vector3.ZERO
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	total_engine_force = 0.0
	total_brake_force = 0.0
	total_aero_drag = 0.0
	total_downforce = 0.0
	wheels_spinning = false
	slip_ratio = 0.0

# ============================================================================
# MAIN PHYSICS LOOP
# ============================================================================

func _physics_process(delta: float) -> void:
	_apply_physics(delta)
	_update_visual_state()
	_handle_inputs(delta)
	_calculate_forces(delta)
	_apply_forces_to_body()

func _apply_physics(delta: float) -> void:
	# Update vehicle kinematics
	move_and_slide()
	
	# Apply gravity
	velocity.y -= GRAVITY * delta * TIME_SCALE
	
	# Update RPM based on transmission
	_update_rpm(delta)
	
	# Calculate vehicle speed from velocity vector
	var ground_speed := velocity.xz.length()
	vehicle_speed = ground_speed
	
	# Detect traction loss
	if _check_traction_loss():
		_emit_traction_lost_signal()
	else:
		emit_signal("traction_regained")

func _handle_inputs(delta: float) -> void:
	# Read input values
	var input_dir := InputManager.get_vector("move_left", "move_right", "move_forward", "move_backward")
	var brake_action := InputManager.get_action_strength("brake")
	var handbrake_action := InputManager.get_action_strength("handbrake")
	var gear_up := InputManager.get_action_strength("gear_up")
	var gear_down := InputManager.get_action_strength("gear_down")
	var clutch_action := InputManager.get_action_strength("clutch")
	
	# Update throttle (positive = accelerate, negative = decelerate/reverse)
	throttle_input = clamp(input_dir.y, -1.0, 1.0)
	
	# Brake combines foot brake and handbrake
	brake_input = clamp(brake_action + handbrake_action * 0.5, 0.0, 1.0)
	
	# Steering input (-1 full left, +1 full right)
	steering_input = clamp(input_dir.x, -1.0, 1.0)
	
	# Clutch pedal
	clutch_pedal = 1.0 - clamp(clutch_action, 0.0, 1.0)
	clutch_engaged = clutch_pedal > clutch_engagement_point
	
	# Gear shifting
	if gear_up > 0.1 and clutch_engaged:
		_shift_gear_up()
	elif gear_down > 0.1 and clutch_engaged:
		_shift_gear_down()

func _update_rpm(delta: float) -> void:
	var desired_rpm: float = 0.0
	
	if current_gear != 0 and clutch_engaged:
		# Calculate desired RPM based on vehicle speed and gear ratio
		var wheel_speed := vehicle_speed / (2.0 * PI * wheel_radius)
		var transmission_ratio := _get_transmission_ratio(current_gear)
		desired_rpm = wheel_speed * transmission_ratio * final_drive_ratio
		
		# Clamp to valid RPM range
		desired_rpm = clamp(desired_rpm, engine_min_rpm, engine_max_rpm)
		
		# Smooth RPM transition
		var rpm_change := (desired_rpm - rpm) * 10.0 * delta
		rpm = lerp(rpm, desired_rpm, 1.0 - exp(-10.0 * delta))
	else:
		# Engine idling or neutral
		rpm = lerp(rpm, idle_rpm, 5.0 * delta)
	
	# Check for over-rev protection
	if rpm >= engine_max_rpm:
		rpm = engine_max_rpm
		_limit_throttle()
	
	# Emit signal
	emit_signal("rpm_changed", rpm, engine_max_rpm)

# ============================================================================
# FORCE CALCULATION
# ============================================================================

func _calculate_forces(delta: float) -> void:
	# Reset force accumulators
	total_engine_force = 0.0
	total_brake_force = 0.0
	total_aero_drag = 0.0
	total_downforce = 0.0
	
	# Calculate engine torque based on RPM and throttle
	var engine_torque := _calculate_engine_torque()
	
	# Apply engine force through transmission
	if current_gear != 0 and clutch_engaged:
		var transmission_ratio := _get_transmission_ratio(current_gear)
		var drive_force := engine_torque * transmission_ratio * final_drive_ratio
		var wheel_radius_effective := wheel_radius * 0.95
		
		# Distribute force based on drivetrain type (FWD/RWD/AWD)
		var drivetrain_type := _get_drivetrain_type()
		var force_distribution := _get_force_distribution(drivetrain_type)
		
		if drivetrain_type == "RWD":
			total_engine_force = drive_force * force_distribution.rear
		elif drivetrain_type == "FWD":
			total_engine_force = drive_force * force_distribution.front
		elif drivetrain_type == "AWD":
			total_engine_force = drive_force * force_distribution.total
	
	# Calculate braking forces
	_total_brake_force = brake_input * brake_force_per_wheel * 4.0
	
	# Calculate aerodynamic drag
	var air_density := 1.225
	var dynamic_pressure := 0.5 * air_density * vehicle_speed * vehicle_speed
	total_aero_drag = drag_coefficient * frontal_area * dynamic_pressure
	
	# Calculate downforce
	total_downforce = downforce_coefficient * frontal_area * dynamic_pressure
	
	# Apply friction reduction at high speeds
	var effective_friction := tire_friction_coefficient
	if vehicle_speed > 50.0:
		effective_friction *= 0.95
	if vehicle_speed > 100.0:
		effective_friction *= 0.90

func _calculate_engine_torque() -> float:
	var normalized_rpm := (rpm - engine_min_rpm) / (engine_max_rpm - engine_min_rpm)
	var base_torque := 450.0  # Base torque in Nm
	
	if engine_torque_curve != null:
		base_torque = engine_torque_curve.interpolate(normalized_rpm)
	
	var throttle_factor := throttle_response_curve.interpolate(throttle_input) if throttle_response_curve else throttle_input
	return base_torque * throttle_factor

func _get_transmission_ratio(gear: int) -> float:
	if gear < 0:
		return reverse_gear_ratio
	elif gear == 0:
		return 0.0
	else:
		return gear_ratios[gear - 1] if gear <= gear_ratios.size() else gear_ratios[-1]

func _get_drivetrain_type() -> String:
	return "RWD"  # Default to RWD, can be configured per vehicle

func _get_force_distribution(drivetrain_type: String) -> Dictionary:
	match drivetrain_type:
		"RWD":
			return {"front": 0.0, "rear": 1.0, "total": 1.0}
		"FWD":
			return {"front": 1.0, "rear": 0.0, "total": 1.0}
		"AWD":
			return {"front": 0.45, "rear": 0.55, "total": 1.0}
		_:
			return {"front": 0.0, "rear": 1.0, "total": 1.0}

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================

func _shift_gear_up() -> void:
	if current_gear < number_of_gears:
		var old_gear := current_gear
		current_gear += 1
		target_gear = current_gear
		emit_signal("gear_changed", old_gear, current_gear)
		_play_shift_sound()

func _shift_gear_down() -> void:
	if current_gear > -1:
		var old_gear := current_gear
		current_gear -= 1
		target_gear = current_gear
		emit_signal("gear_changed", old_gear, current_gear)
		_play_shift_sound()

func _limit_throttle() -> void:
	if rpm >= engine_max_rpm:
		throttle_input = 0.0

func _play_shift_sound() -> void:
	if is_instance_valid(_audio_manager):
		_audio_manager.play_sfx("gear_shift")

# ============================================================================
# TRACTION CONTROL
# ============================================================================

func _check_traction_loss() -> bool:
	var wheel_base_speed := vehicle_speed
	var engine_wheel_speed := rpm / (final_drive_ratio * _get_transmission_ratio(current_gear)) if current_gear != 0 else 0.0
	
	if current_gear != 0 and engine_wheel_speed > wheel_base_speed * 1.3:
		wheels_spinning = true
		return true
	
	return false

func _emit_traction_lost_signal() -> void:
	emit_signal("collision_detected", "traction_loss")
	if is_instance_valid(_audio_manager):
		_audio_manager.play_sfx("tire_spin")

# ============================================================================
# COLLISION HANDLING
# ============================================================================

func _on_collision_started(body: Node) -> void:
	var collision_type := _identify_collision_type(body)
	emit_signal("collision_detected", collision_type)
	
	if is_instance_valid(_audio_manager):
		_audio_manager.play_sfx("collision_" + collision_type)

func _on_collision_object_entered(body: Node) -> void:
	var impact_force := body.linear_velocity.distance_to(Vector3.ZERO) * mass
	var impact_point := global_position
	emit_signal("vehicle_hit", impact_force, impact_point)

func _identify_collision_type(body: Node) -> String:
	if body.is_in_group("obstacle"):
		return "obstacle"
	elif body.is_in_group("barrier"):
		return "barrier"
	elif body.is_in_group("wall"):
		return "wall"
	elif body.is_in_group("car"):
		return "vehicle"
	else:
		return "generic"

# ============================================================================
# INPUT EVENT HANDLER
# ============================================================================

func _input(event: InputEvent) -> void:
	if event.is_action_pressed("pause_game"):
		_toggle_pause()
	
	if event.is_action_pressed("toggle_debug"):
		_game_manager.debug_mode = not _game_manager.debug_mode

func _toggle_pause() -> void:
	if is_instance_valid(_game_manager):
		_game_manager.current_state = GameManager.GameState.RACE_PAUSED

# ============================================================================
# VISUAL UPDATE
# ============================================================================

func _update_visual_state() -> void:
	# Update wheel rotation angles based on distance traveled
	var delta_distance := vehicle_speed * get_physics_process_delta_time()
	
	for i in range(4):
		wheel_rotation_angles[i] += delta_distance / wheel_radius
		wheel_angular_velocities[i] = delta_distance / get_physics_process_delta_time() / wheel_radius

# ============================================================================
# SETTERS
# ============================================================================

func _set_mass(value: float) -> void:
	mass = value
	mass_set.emit(mass)

# ============================================================================
# GAME STATE HANDLERS
# ============================================================================

func _on_game_state_changed(new_state: GameManager.GameState) -> void:
	match new_state:
		GameManager.GameState.MAIN_MENU:
			_reset_vehicle_state()
		GameManager.GameState.RACE_ACTIVE:
			_resume_vehicle_state()
		GameManager.GameState.RACE_PAUSED:
			_pause_vehicle_state()

func _resume_vehicle_state() -> void:
	# Resume normal operation
	pass

func _pause_vehicle_state() -> void:
	# Save state before pause
	pass

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================

func get_current_speed_kmh() -> float:
	return vehicle_speed * 3.6

func get_current_speed_mph() -> float:
	return vehicle_speed * 2.237

func get_current_rpm_percentage() -> float:
	return (rpm / engine_max_rpm) * 100.0

func get_current_gear_display() -> String:
	match current_gear:
		-1: return "R"
		0: return "N"
		_: return str(current_gear)

func calculate_lateral_g_force() -> float:
	var lateral_acc := abs(velocity.xz.cross(Vector3.UP).length()) / 9.81
	return lateral_acc

func calculate_longitudinal_g_force() -> float:
	return abs(acceleration.z) / 9.81

# ============================================================================
# DEBUG FUNCTIONS
# ============================================================================

func debug_print_vehicle_stats() -> void:
	print("=== VEHICLE STATS ===")
	print("RPM: %f / %f" % [rpm, engine_max_rpm])
	print("Speed: %.2f km/h" % get_current_speed_kmh())
	print("Gear: %s" % get_current_gear_display())
	print("Throttle: %.2f" % throttle_input)
	print("Brake: %.2f" % brake_input)
	print("Steering: %.2f" % steering_input)
	print("Lateral G: %.2f g" % calculate_lateral_g_force())
	print("Longitudinal G: %.2f g" % calculate_longitudinal_g_force())
	print("====================")

func _get_transmission_ratio(gear: int) -> float:
	if gear < 0:
		return reverse_gear_ratio
	elif gear == 0:
		return 0.0
	else:
		return gear_ratios[gear - 1] if gear <= gear_ratios.size() else gear_ratios[-1]
</FILE "scripts/vehicles/VehicleController.gd">