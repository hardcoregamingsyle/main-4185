extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulator
## Implements throttle, brake, steering inputs, wheel forces, and gear shifting logic
## All physics values sourced from PhysicsSettings resource for centralized tuning
## Copyright 2026 Thalamus Racing Simulator Project

# ============================================================================
# SIGNALS
# ============================================================================

signal speed_changed(new_speed: float)
signal rpm_changed(new_rpm: float)
signal gear_changed(old_gear: int, new_gear: int)
signal collision_detected(collision_info: Dictionary)
signal lap_completed(lap_time: float)

# ============================================================================
# CONSTANTS & CONFIGURATION
# ============================================================================

const DEFAULT_ACCELERATION = 8.0
const DEFAULT_BRAKING_FORCE = 15.0
const MAX_STEERING_ANGLE = PI / 4  # 45 degrees
const STEERING_SPEED = 3.0

# Gear ratios (engine RPM : wheel RPM)
const GEAR_RATIOS: Array[float] = [3.5, 2.5, 1.7, 1.3, 1.0, 0.8]
const NEUTRAL_GEAR = -1
const REVERSE_GEAR = 0

# RPM thresholds
const MIN_IDLE_RPM = 800.0
const MAX_ENGINE_RPM = 7500.0
const SHIFT_UP_RPM = 6500.0
const SHIFT_DOWN_RPM = 2000.0

# ============================================================================
# EXPORTED PROPERTIES
# ============================================================================

@export_group("Vehicle Configuration")
@export var vehicle_mass: float = 1500.0
@export var center_of_mass_offset: Vector3 = Vector3(0.0, -0.5, 0.0)
@export var track_width: float = 1.6
@export var wheelbase: float = 2.6

@export_group("Drivetrain Settings")
@export var drivetrain_type: DrivetrainType = DrivetrainType.FWD
@export var final_drive_ratio: float = 4.0

@export_group("Tire Grip Settings")
@export var front_grip_coefficient: float = 1.2
@export var rear_grip_coefficient: float = 1.1
@export var max_cornering_force: float = 30000.0

enum DrivetrainType {
	FWD,      # Front-Wheel Drive
	RWD,      # Rear-Wheel Drive
	AWD       # All-Wheel Drive
}

# ============================================================================
# INTERNAL STATE
# ============================================================================

var _current_speed: float = 0.0
var _current_rpm: float = MIN_IDLE_RPM
var _current_gear: int = NEUTRAL_GEAR
var _target_steering_angle: float = 0.0
var _current_steering_angle: float = 0.0
var _throttle_input: float = 0.0
var _brake_input: float = 0.0
var _steering_input: float = 0.0

var _acceleration_vector: Vector3 = Vector3.ZERO
var _wheel_forces: Dictionary = {
	"front_left": Vector3.ZERO,
	"front_right": Vector3.ZERO,
	"rear_left": Vector3.ZERO,
	"rear_right": Vector3.ZERO
}

var _is_braking: bool = false
var _is_driving: bool = false
var _is_reversing: bool = false

# Reference to Powertrain (if attached)
var _powertrain: Node = null

# Timer for auto-shifting
var _shift_timer: float = 0.0
const AUTO_SHIFT_COOLDOWN: float = 0.5

# ============================================================================
# PHYSICS SETTINGS REFERENCE
# ============================================================================

var _physics_settings: PhysicsSettings = null

# ============================================================================
# INITIALIZATION
# ============================================================================

func _ready() -> void:
	_init_physics_reference()
	_connect_signals_to_game_manager()
	_configure_center_of_mass()
	_apply_initial_state()

func _init_physics_reference() -> void:
	if Engine.has_singleton("PhysicsSettings"):
		_physics_settings = Engine.get_singleton("PhysicsSettings")
	elif ResourceLoader.exists("res://scripts/core/PhysicsSettings.gd"):
		_physics_settings = load("res://scripts/core/PhysicsSettings.gd").new()
	else:
		push_warning("VehicleController: PhysicsSettings not found, using defaults")

func _connect_signals_to_game_manager() -> void:
	if GameManager:
		GameManager.game_state_changed.connect(_on_game_state_changed)

func _configure_center_of_mass() -> void:
	var com_position = transform * center_of_mass_offset
	set_collision_layer_value(1, true)  # Vehicle layer
	set_collision_mask_value(1, true)   # Vehicle vs world
	# Center of mass affects suspension and weight distribution

func _apply_initial_state() -> void:
	_current_gear = NEUTRAL_GEAR
	_current_rpm = MIN_IDLE_RPM
	_throttle_input = 0.0
	_brake_input = 0.0
	_steering_input = 0.0
	emit_signal("gear_changed", NEUTRAL_GEAR, NEUTRAL_GEAR)

# ============================================================================
# MAIN UPDATE LOOP
# ============================================================================

func _physics_process(delta: float) -> void:
	_update_inputs(delta)
	_update_engine(delta)
	_update_gear_shifting(delta)
	_calculate_wheel_forces(delta)
	_apply_vehicle_dynamics(delta)
	_update_visual_feedback(delta)

# ============================================================================
# INPUT PROCESSING
# ============================================================================

func _update_inputs(delta: float) -> void:
	if not InputManager:
		return
	
	# Get unified input from InputManager singleton
	_throttle_input = InputManager.get_axis("throttle", 0.0, 1.0)
	_brake_input = InputManager.get_axis("brake", 0.0, 1.0)
	_steering_input = InputManager.get_axis("steering", -1.0, 1.0)
	
	# Clamp inputs to valid ranges
	_throttle_input = clamp(_throttle_input, 0.0, 1.0)
	_brake_input = clamp(_brake_input, 0.0, 1.0)
	_steering_input = clamp(_steering_input, -1.0, 1.0)
	
	# Update braking state
	_is_braking = _brake_input > 0.1
	
	# Update driving state
	_is_driving = _throttle_input > 0.1

# ============================================================================
# ENGINE & RPM MANAGEMENT
# ============================================================================

func _update_engine(delta: float) -> void:
	var target_rpm = _calculate_target_rpm()
	
	# Smooth RPM transition
	_current_rpm = lerp(_current_rpm, target_rpm, delta * 10.0)
	
	# Clamp RPM to valid range
	_current_rpm = clamp(_current_rpm, MIN_IDLE_RPM, MAX_ENGINE_RPM)
	
	# Emit RPM change signal
	emit_signal("rpm_changed", _current_rpm)
	
	# Update powertrain if available
	if _powertrain:
		_powertrain.engine_rpm = _current_rpm

func _calculate_target_rpm() -> float:
	if _current_gear == NEUTRAL_GEAR:
		return MIN_IDLE_RPM
	
	# Calculate expected RPM based on vehicle speed and gear ratio
	var effective_ratio = _get_effective_ratio()
	var wheel_rpm = (_current_speed * 60.0) / (PI * _get_wheel_radius())
	var expected_rpm = wheel_rpm * effective_ratio
	
	# If no movement, idle RPM
	if abs(_current_speed) < 0.5:
		return MIN_IDLE_RPM
	
	return expected_rpm

func _get_effective_ratio() -> float:
	var gear_ratio = GEAR_RATIOS[_current_gear] if _current_gear >= 0 else 1.0
	return gear_ratio * final_drive_ratio

func _get_wheel_radius() -> float:
	# Approximate tire radius (can be made configurable)
	return 0.35

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================

func _update_gear_shifting(delta: float) -> void:
	_shift_timer += delta
	
	# Manual gear shift detection via InputManager
	if InputManager.is_action_just_pressed("shift_up"):
		_attempt_shift_up()
	elif InputManager.is_action_just_pressed("shift_down"):
		_attempt_shift_down()
	
	# Auto-shift logic (optional feature)
	if _should_auto_shift():
		_auto_shift()

func _attempt_shift_up() -> void:
	if _current_gear == NEUTRAL_GEAR or _current_gear >= GEAR_RATIOS.size() - 1:
		return
	
	var old_gear = _current_gear
	_current_gear += 1
	
	emit_signal("gear_changed", old_gear, _current_gear)
	_play_sound_effect("shift_up")

func _attempt_shift_down() -> void:
	if _current_gear == NEUTRAL_GEAR or _current_gear <= 0:
		return
	
	var old_gear = _current_gear
	_current_gear -= 1
	
	emit_signal("gear_changed", old_gear, _current_gear)
	_play_sound_effect("shift_down")

func _auto_shift() -> void:
	if _shift_timer < AUTO_SHIFT_COOLDOWN:
		return
	
	# Shift up at high RPM
	if _current_rpm > SHIFT_UP_RPM and _current_gear < GEAR_RATIOS.size() - 1:
		var old_gear = _current_gear
		_current_gear += 1
		emit_signal("gear_changed", old_gear, _current_gear)
		_shift_timer = 0.0
		return
	
	# Shift down at low RPM
	if _current_rpm < SHIFT_DOWN_RPM and _current_gear > 0:
		var old_gear = _current_gear
		_current_gear -= 1
		emit_signal("gear_changed", old_gear, _current_gear)
		_shift_timer = 0.0
		return

func _should_auto_shift() -> bool:
	# Auto-shift enabled when no manual shift is happening
	return not InputManager.is_action_pressed("shift_up") and \
	       not InputManager.is_action_pressed("shift_down")

# ============================================================================
# WHEEL FORCE CALCULATION
# ============================================================================

func _calculate_wheel_forces(delta: float) -> void:
	# Reset all wheel forces
	for key in _wheel_forces:
		_wheel_forces[key] = Vector3.ZERO
	
	var total_drive_force = _calculate_total_drive_force()
	var total_brake_force = _calculate_total_brake_force()
	
	# Distribute forces based on drivetrain type
	_distribute_drive_force(total_drive_force)
	_distribute_brake_force(total_brake_force)
	
	# Apply lateral grip forces during cornering
	_apply_lateral_grip(delta)

func _calculate_total_drive_force() -> float:
	if _current_gear == NEUTRAL_GEAR:
		return 0.0
	
	# Force based on throttle input and engine torque curve approximation
	var torque_curve_factor = _get_torque_curve_factor()
	var drive_force = _throttle_input * torque_curve_factor * DEFAULT_ACCELERATION * vehicle_mass
	
	return drive_force

func _calculate_total_brake_force() -> float:
	if not _is_braking:
		return 0.0
	
	return _brake_input * DEFAULT_BRAKING_FORCE * vehicle_mass

func _get_torque_curve_factor() -> float:
	# Simple torque curve approximation based on RPM
	# Peak torque typically around 4000-5000 RPM
	var normalized_rpm = (_current_rpm - MIN_IDLE_RPM) / (MAX_ENGINE_RPM - MIN_IDLE_RPM)
	
	if normalized_rpm < 0.3:
		return 0.5 + normalized_rpm * 0.5
	elif normalized_rpm < 0.7:
		return 1.0
	else:
		return 1.0 - (normalized_rpm - 0.7) * 0.5

func _distribute_drive_force(total_force: float) -> void:
	if total_force <= 0:
		return
	
	var forward_direction = -global_transform.basis.z
	var magnitude = total_force * _get_drivetrain_distribution()
	
	match drivetrain_type:
		DrivetrainType.FWD:
			_wheel_forces["front_left"] = forward_direction * magnitude * 0.5
			_wheel_forces["front_right"] = forward_direction * magnitude * 0.5
		DrivetrainType.RWD:
			_wheel_forces["rear_left"] = forward_direction * magnitude * 0.5
			_wheel_forces["rear_right"] = forward_direction * magnitude * 0.5
		DrivetrainType.AWD:
			_wheel_forces["front_left"] = forward_direction * magnitude * 0.25
			_wheel_forces["front_right"] = forward_direction * magnitude * 0.25
			_wheel_forces["rear_left"] = forward_direction * magnitude * 0.25
			_wheel_forces["rear_right"] = forward_direction * magnitude * 0.25

func _get_drivetrain_distribution() -> float:
	match drivetrain_type:
		DrivetrainType.FWD:
			return 0.6  # 60% front bias
		DrivetrainType.RWD:
			return 0.7  # 70% rear bias
		DrivetrainType.AWD:
			return 0.5  # Even distribution

func _distribute_brake_force(total_force: float) -> void:
	if total_force <= 0:
		return
	
	var magnitude = total_force
	var brake_force_per_wheel = magnitude / 4.0
	
	# Apply braking force opposite to travel direction
	var backward_direction = global_transform.basis.z
	
	_wheel_forces["front_left"] += backward_direction * brake_force_per_wheel
	_wheel_forces["front_right"] += backward_direction * brake_force_per_wheel
	_wheel_forces["rear_left"] += backward_direction * brake_force_per_wheel
	_wheel_forces["rear_right"] += backward_direction * brake_force_per_wheel

func _apply_lateral_grip(delta: float) -> void:
	# Calculate lateral velocity (sideways motion)
	var lateral_velocity = linear_velocity.dot(global_transform.basis.y)
	
	if abs(lateral_velocity) < 0.1:
		return
	
	# Apply counteracting force based on grip coefficient
	var grip_factor = _get_current_grip_factor()
	var lateral_force = -lateral_velocity * grip_factor * vehicle_mass
	
	# Limit by max cornering force
	lateral_force = clampf(abs(lateral_force), 0.0, max_cornering_force)
	
	var lateral_direction = global_transform.basis.y
	if lateral_velocity < 0:
		lateral_direction = -lateral_direction
	
	_wheel_forces["front_left"] += lateral_direction * lateral_force * 0.25
	_wheel_forces["front_right"] += lateral_direction * lateral_force * 0.25
	_wheel_forces["rear_left"] += lateral_direction * lateral_force * 0.25
	_wheel_forces["rear_right"] += lateral_direction * lateral_force * 0.25

func _get_current_grip_factor() -> float:
	if _current_gear == NEUTRAL_GGER:
		return 0.0
	
	var avg_grip = (front_grip_coefficient + rear_grip_coefficient) / 2.0
	return avg_grip

# ============================================================================
# VEHICLE DYNAMICS APPLICATION
# ============================================================================

func _apply_vehicle_dynamics(delta: float) -> void:
	# Apply all wheel forces to vehicle body
	for wheel_name in _wheel_forces:
		var wheel_force = _wheel_forces[wheel_name]
		apply_central_force(wheel_force)
	
	# Apply gravity
	var gravity_vector = Vector3.DOWN * _physics_settings.gravity if _physics_settings else Vector3.DOWN * 9.81
	apply_central_force(gravity_vector * vehicle_mass)
	
	# Update velocity
	move_and_slide()
	
	# Update speed tracking
	_update_speed()
	
	# Check for collisions
	_check_collisions()

func _update_speed() -> void:
	_current_speed = linear_velocity.length()
	
	if _current_speed != speed_changed:
		speed_changed.emit(_current_speed)

func _check_collisions() -> void:
	if get_collision_count() > 0:
		for i in get_collision_count():
			var collision_info = {
				"collider": get_collider(i),
				"normal": get_collision_normal(i),
				"position": get_collision_point(i)
			}
			collision_detected.emit(collision_info)

# ============================================================================
# VISUAL FEEDBACK
# ============================================================================

func _update_visual_feedback(delta: float) -> void:
	# Smooth steering animation
	var target_angle = _steering_input * MAX_STEERING_ANGLE
	_current_steering_angle = lerp(_current_steering_angle, target_angle, delta * STEERING_SPEED)
	
	# Apply steering rotation to front wheels (placeholder - actual wheel nodes needed)
	# This would normally rotate the front wheel meshes
	_set_front_wheels_rotation(_current_steering_angle)

func _set_front_wheels_rotation(angle: float) -> void:
	# Find and rotate front wheel nodes
	var front_wheels = _find_child_nodes_by_tag("front_wheel")
	for wheel_node in front_wheels:
		wheel_node.rotation.y = angle

func _find_child_nodes_by_tag(tag: String) -> Array[Node]:
	var results: Array[Node] = []
	for child in get_children():
		if child.has_meta("wheel_tag") and child.get_meta("wheel_tag") == tag:
			results.append(child)
	return results

# ============================================================================
# SOUND EFFECTS
# ============================================================================

func _play_sound_effect(sound_name: String) -> void:
	if AudioManager:
		AudioManager.play_sfx(sound_name)

func _update_engine_sound() -> void:
	if AudioManager:
		AudioManager.update_engine_volume(_current_rpm, _current_speed)

# ============================================================================
# PUBLIC API
# ============================================================================

func get_current_speed() -> float:
	return _current_speed

func get_current_rpm() -> float:
	return _current_rpm

func get_current_gear() -> int:
	return _current_gear

func set_gear(gear_index: int) -> void:
	if gear_index < 0 or gear_index >= GEAR_RATIOS.size():
		return
	
	var old_gear = _current_gear
	_current_gear = gear_index
	emit_signal("gear_changed", old_gear, _current_gear)

func reset_vehicle() -> void:
	_current_gear = NEUTRAL_GEAR
	_current_rpm = MIN_IDLE_RPM
	_throttle_input = 0.0
	_brake_input = 0.0
	_steering_input = 0.0
	linear_velocity = Vector3.ZERO
	angular_velocity = Vector3.ZERO

func enable_debug_mode(enabled: bool) -> void:
	# Toggle debug visualization
	pass

# ============================================================================
# SIGNAL HANDLERS
# ============================================================================

func _on_game_state_changed(new_state: GameState) -> void:
	match new_state:
		GameManager.GameState.RACE_ACTIVE:
			_is_driving = true
		GameManager.GameState.RACE_PAUSED:
			_is_driving = false
		GameManager.GameState.RACE_FINISHED:
			reset_vehicle()

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================

func serialize_vehicle_state() -> Dictionary:
	return {
		"speed": _current_speed,
		"rpm": _current_rpm,
		"gear": _current_gear,
		"throttle": _throttle_input,
		"brake": _brake_input,
		"steering": _steering_input,
		"position": global_position,
		"rotation": global_rotation
	}

func deserialize_vehicle_state(state_data: Dictionary) -> void:
	if state_data.has("speed"):
		_current_speed = state_data["speed"]
	if state_data.has("rpm"):
		_current_rpm = state_data["rpm"]
	if state_data.has("gear"):
		_current_gear = state_data["gear"]
	if state_data.has("throttle"):
		_throttle_input = state_data["throttle"]
	if state_data.has("brake"):
		_brake_input = state_data["brake"]
	if state_data.has("steering"):
		_steering_input = state_data["steering"]
	if state_data.has("position"):
		global_position = state_data["position"]
	if state_data.has("rotation"):
		global_rotation = state_data["rotation"]

</FILE>