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
signal wheel_slip_detected(wheel_position: String, slip_ratio: float)

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
const REDLINE_RPM = 7800.0

# Tire friction coefficients
const FRICTION_ASPHALT = 1.0
const FRICTION_GRASS = 0.6
const FRICTION_GRAVEL = 0.4
const FRICTION_ICE = 0.1

# ============================================================================
# EXPORTED PROPERTIES
# ============================================================================

@export_group("Vehicle Configuration")
@export var vehicle_mass: float = 1500.0: set = _set_vehicle_mass
@export var center_of_mass_offset: Vector3 = Vector3(0.0, -0.5, 0.0)
@export var track_width: float = 1.6
@export var wheelbase: float = 2.6
@export var aerodynamic_drag_coefficient: float = 0.35
@export var frontal_area: float = 2.2

@export_group("Drivetrain Settings")
@export var drivetrain_type: DrivetrainType = DrivetrainType.RWD
@export var final_drive_ratio: float = 4.0
@export var clutch_engagement_point: float = 0.15

@export_group("Tire Grip Settings")
@export var front_grip_coefficient: float = 1.2
@export var rear_grip_coefficient: float = 1.1
@export var max_cornering_force: float = 30000.0
@export var grip_loss_threshold: float = 0.85

enum DrivetrainType {
	FWD,      # Front-Wheel Drive
	RWD,      # Rear-Wheel Drive
	AWD       # All-Wheel Drive
}

@export_group("Wheel Configuration")
@export var wheel_radius: float = 0.33
@export var suspension_travel: float = 0.15
@export var suspension_stiffness: float = 50000.0
@export var damping_ratio: float = 0.4

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
var _clutch_engaged: bool = true
var _clutch_pedal: float = 1.0

# Reference to Powertrain (if attached)
var _powertrain: Node = null

# Wheel positions (relative to vehicle origin)
var _wheel_positions: Dictionary = {}

# Suspension states
var _wheel_suspension: Dictionary = {
	"front_left": 0.0,
	"front_right": 0.0,
	"rear_left": 0.0,
	"rear_right": 0.0
}

var _wheel_contact_points: Dictionary = {}

# Aerodynamics
var _drag_force: float = 0.0
var _downforce: float = 0.0

# Lap timing
var _lap_start_time: float = 0.0
var _current_lap_time: float = 0.0
var _last_checkpoint_position: Vector3 = Vector3.ZERO
var _checkpoints_passed: Array[int] = []

# Physics reference
var _physics_settings: PhysicsSettings = null

# Collision detection
var _collision_normal: Vector3 = Vector3.UP
var _collision_velocity: Vector3 = Vector3.ZERO
var _collision_impact_force: float = 0.0

# ============================================================================
# WHEEL POSITIONS
# ============================================================================

func _init_wheel_positions() -> void:
	var half_track = track_width * 0.5
	var half_wheelbase = wheelbase * 0.5
	
	_wheel_positions = {
		"front_left": Vector3(-half_track, 0.0, -half_wheelbase),
		"front_right": Vector3(half_track, 0.0, -half_wheelbase),
		"rear_left": Vector3(-half_track, 0.0, half_wheelbase),
		"rear_right": Vector3(half_track, 0.0, half_wheelbase)
	}

# ============================================================================
# LIFECYCLE
# ============================================================================

func _ready() -> void:
	_process_mode = ProcessModeEnum.ALWAYS
	_init_wheel_positions()
	_connect_signals()
	_load_physics_settings()
	_reset_vehicle_state()

func _exit_tree() -> void:
	if _powertrain != null:
		_powertrain.disconnect("engine_ready", Callable(self, "_on_engine_ready"))

func _physics_process(delta: float) -> void:
	if not is_instance_valid(_physics_settings):
		return
	
	_apply_inputs(delta)
	_update_gear_shifting(delta)
	_calculate_wheel_forces(delta)
	_apply_physics(delta)
	_update_aerodynamics(delta)
	_check_collisions(delta)

func _input(event: InputEvent) -> void:
	if event.is_action_pressed("vehicle_throttle"):
		_on_throttle_pressed()
	elif event.is_action_released("vehicle_throttle"):
		_on_throttle_released()
	elif event.is_action_pressed("vehicle_brake"):
		_on_brake_pressed()
	elif event.is_action_released("vehicle_brake"):
		_on_brake_released()
	elif event.is_action_pressed("vehicle_steering_left"):
		_on_steering_left()
	elif event.is_action_pressed("vehicle_steering_right"):
		_on_steering_right()
	elif event.is_action_pressed("vehicle_steering_center"):
		_on_steering_center()
	elif event.is_action_pressed("vehicle_shift_up"):
		_shift_up()
	elif event.is_action_pressed("vehicle_shift_down"):
		_shift_down()
	elif event.is_action_pressed("vehicle_clutch"):
		_toggle_clutch()

# ============================================================================
# INITIALIZATION
# ============================================================================

func _load_physics_settings() -> void:
	_physics_settings = ResourceManager.get_singleton("PhysicsSettings") as PhysicsSettings
	if _physics_settings == null:
		print("[VehicleController] Warning: PhysicsSettings singleton not found!")

func _connect_signals() -> void:
	if _powertrain != null:
		_powertrain.connect("engine_ready", Callable(self, "_on_engine_ready"))

func _reset_vehicle_state() -> void:
	_current_speed = 0.0
	_current_rpm = MIN_IDLE_RPM
	_current_gear = NEUTRAL_GEAR
	_target_steering_angle = 0.0
	_current_steering_angle = 0.0
	_throttle_input = 0.0
	_brake_input = 0.0
	_steering_input = 0.0
	_is_braking = false
	_is_driving = false
	_is_reversing = false
	_clutch_engaged = true
	_clutch_pedal = 1.0
	_checkpoints_passed.clear()
	_wheel_suspension = {
		"front_left": 0.0,
		"front_right": 0.0,
		"rear_left": 0.0,
		"rear_right": 0.0
	}
	_wheel_forces = {
		"front_left": Vector3.ZERO,
		"front_right": Vector3.ZERO,
		"rear_left": Vector3.ZERO,
		"rear_right": Vector3.ZERO
	}
	_drag_force = 0.0
	_downforce = 0.0

# ============================================================================
# INPUT HANDLING
# ============================================================================

func _on_throttle_pressed() -> void:
	_throttle_input += 0.1

func _on_throttle_released() -> void:
	_throttle_input -= 0.1
	_clamp_input_values()

func _on_brake_pressed() -> void:
	_brake_input += 0.1
	_is_braking = true

func _on_brake_released() -> void:
	_brake_input -= 0.1
	_is_braking = false
	_clamp_input_values()

func _on_steering_left() -> void:
	_steering_input -= 0.1
	_clamp_input_values()

func _on_steering_right() -> void:
	_steering_input += 0.1
	_clamp_input_values()

func _on_steering_center() -> void:
	_steering_input = 0.0

func _toggle_clutch() -> void:
	_clutch_engaged = not _clutch_engaged

func _clamp_input_values() -> void:
	_throttle_input = clampf(_throttle_input, 0.0, 1.0)
	_brake_input = clampf(_brake_input, 0.0, 1.0)
	_steering_input = clampf(_steering_input, -1.0, 1.0)

func _apply_inputs(delta: float) -> void:
	# Smooth steering transition
	var steering_delta = (_target_steering_angle - _current_steering_angle) * delta * STEERING_SPEED
	_current_steering_angle += steering_delta
	
	# Apply steering based on input
	_target_steering_angle = _steering_input * MAX_STEERING_ANGLE
	
	# Update driving state based on throttle and gear
	if _throttle_input > 0.01 or _brake_input < 0.99:
		_is_driving = true
	else:
		_is_driving = false
	
	# Calculate effective power delivery based on clutch
	var clutch_factor = _clutch_engaged ? 1.0 : 0.0
	
	# Apply throttle/brake to velocity
	var direction = Vector3.FORWARD if _current_gear >= 0 else Vector3.BACKWARD
	direction = direction.rotated(Vector3.UP, rotation.y)
	
	if _is_driving and _clutch_engaged:
		var engine_force = _calculate_engine_force()
		_acceleration_vector = direction * engine_force * clutch_factor
		
		# Reverse direction for reverse gear
		if _current_gear == REVERSE_GEAR:
			_acceleration_vector *= -1.0
			_is_reversing = true
		else:
			_is_reversing = false
	
	# Apply braking force
	if _brake_input > 0.01:
		var braking_force = _get_braking_force() * _brake_input
		_acceleration_vector -= direction.normalized() * braking_force * clutch_factor

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================

func _update_gear_shifting(delta: float) -> void:
	# Automatic upshifts when RPM reaches threshold
	if _current_gear != NEUTRAL_GEAR and _current_rpm >= SHIFT_UP_RPM:
		if _current_gear < GEAR_RATIOS.size():
			_auto_shift_up()
	
	# Automatic downshifts when RPM drops below threshold
	elif _current_gear > REVERSE_GEAR and _current_rpm <= SHIFT_DOWN_RPM:
		_auto_shift_down()

func _shift_up() -> void:
	if _current_gear < GEAR_RATIOS.size():
		var old_gear = _current_gear
		_current_gear += 1
		emit_signal("gear_changed", old_gear, _current_gear)
		_disengage_clutch()

func _shift_down() -> void:
	if _current_gear > REVERSE_GEAR:
		var old_gear = _current_gear
		_current_gear -= 1
		emit_signal("gear_changed", old_gear, _current_gear)
		_disengage_clutch()

func _auto_shift_up() -> void:
	if _current_gear < GEAR_RATIOS.size():
		var old_gear = _current_gear
		_current_gear += 1
		emit_signal("gear_changed", old_gear, _current_gear)

func _auto_shift_down() -> void:
	if _current_gear > REVERSE_GEAR:
		var old_gear = _current_gear
		_current_gear -= 1
		emit_signal("gear_changed", old_gear, _current_gear)

func _disengage_clutch() -> void:
	_clutch_engaged = false
	await get_tree().create_timer(0.1).timeout
	_clutch_engaged = true

# ============================================================================
# ENGINE & POWER CALCULATIONS
# ============================================================================

func _calculate_engine_force() -> float:
	# Base acceleration scaled by vehicle mass
	var base_acceleration = DEFAULT_ACCELERATION * (1500.0 / vehicle_mass)
	
	# Apply throttle input
	var throttle_factor = _throttle_input
	
	# Calculate gear ratio effect
	var gear_index = max(0, _current_gear)
	var gear_ratio = GEAR_RATIOS[gear_index] if gear_index < GEAR_RATIOS.size() else 1.0
	
	# Final drive ratio
	var total_ratio = gear_ratio * final_drive_ratio
	
	# Engine torque curve approximation
	var torque_curve = _get_torque_curve_value()
	
	# Power delivery varies by drivetrain type
	var drivetrain_efficiency = 0.85 if drivetrain_type == DrivetrainType.AWD else 0.90
	drivetrain_efficiency = 0.88 if drivetrain_type == DrivetrainType.FWD else drivetrain_efficiency
	
	var engine_force = base_acceleration * throttle_factor * total_ratio * torque_curve * drivetrain_efficiency
	
	return engine_force

func _get_torque_curve_value() -> float:
	# Simulated torque curve (peak around 4000-5000 RPM)
	var normalized_rpm = (_current_rpm - MIN_IDLE_RPM) / (MAX_ENGINE_RPM - MIN_IDLE_RPM)
	
	if normalized_rpm < 0.2:
		return 0.5 + normalized_rpm * 0.5
	elif normalized_rpm < 0.6:
		return 1.0
	elif normalized_rpm < 0.85:
		return 1.0 - (normalized_rpm - 0.6) * 0.5
	else:
		return 0.5 - (normalized_rpm - 0.85) * 0.3

func _get_braking_force() -> float:
	return DEFAULT_BRAKING_FORCE * (1500.0 / vehicle_mass)

func _calculate_rpm_from_speed(speed: float) -> float:
	# Convert wheel speed to engine RPM
	var wheel_circumference = 2.0 * PI * wheel_radius
	var wheel_rps = abs(speed) / wheel_circumference
	
	var gear_index = max(0, _current_gear)
	var gear_ratio = GEAR_RATIOS[gear_index] if gear_index < GEAR_RATIOS.size() else 1.0
	
	var engine_rps = wheel_rps * gear_ratio * final_drive_ratio
	var rpm = engine_rps * 60.0
	
	return rpm

func _calculate_speed_from_rpm(rpm: float) -> float:
	# Convert engine RPM to vehicle speed
	var gear_index = max(0, _current_gear)
	var gear_ratio = GEAR_RATIOS[gear_index] if gear_index < GEAR_RATIOS.size() else 1.0
	
	var wheel_rps = rpm / 60.0 / gear_ratio / final_drive_ratio
	var speed = wheel_rps * 2.0 * PI * wheel_radius
	
	return speed if _current_gear >= 0 else -speed

# ============================================================================
# WHEEL FORCE CALCULATION
# ============================================================================

func _calculate_wheel_forces(delta: float) -> void:
	# Reset wheel forces
	for wheel in _wheel_forces:
		_wheel_forces[wheel] = Vector3.ZERO
	
	if not _is_driving and not _is_braking:
		return
	
	# Calculate traction distribution based on drivetrain
	var force_distribution = _get_traction_distribution()
	
	# Distribute forces to appropriate wheels
	var total_force = _calculate_total_wheel_force()
	
	for wheel in force_distribution:
		var wheel_pos = _wheel_positions[wheel]
		var local_force = total_force * force_distribution[wheel]
		
		# Apply longitudinal force
		var forward_dir = Vector3.FORWARD.rotated(Vector3.UP, rotation.y)
		var wheel_force = forward_dir * local_force
		
		# Add lateral force based on steering angle
		if wheel.begins_with("front"):
			wheel_force.x += _current_steering_angle * local_force * 0.5
		
		_wheel_forces[wheel] = wheel_force

func _get_traction_distribution() -> Dictionary:
	match drivetrain_type:
		DrivetrainType.FWD:
			return {"front_left": 0.5, "front_right": 0.5, "rear_left": 0.0, "rear_right": 0.0}
		DrivetrainType.RWD:
			return {"front_left": 0.0, "front_right": 0.0, "rear_left": 0.5, "rear_right": 0.5}
		DrivetrainType.AWD:
			return {"front_left": 0.25, "front_right": 0.25, "rear_left": 0.25, "rear_right": 0.25}
		_:
			return {"front_left": 0.0, "front_right": 0.0, "rear_left": 0.0, "rear_right": 0.0}

func _calculate_total_wheel_force() -> float:
	var base_force = _calculate_engine_force()
	
	# Adjust for surface grip
	var ground_surface = _get_ground_friction()
	base_force *= ground_surface
	
	# Slip reduction
	var slip_factor = _calculate_slip_factor()
	base_force *= slip_factor
	
	return base_force

func _get_ground_friction() -> float:
	# Simplified - would normally raycast to determine surface
	return FRICTION_ASPHALT

func _calculate_slip_factor() -> float:
	# Calculate wheel slip ratio
	var wheel_slip = (_current_rpm / 60.0) / (abs(_current_speed) / (2.0 * PI * wheel_radius)) - 1.0
	
	if abs(wheel_slip) > grip_loss_threshold:
		return 1.0 - (abs(wheel_slip) - grip_loss_threshold) * 0.5
	
	return 1.0

# ============================================================================
# AERODYNAMICS
# ============================================================================

func _update_aerodynamics(delta: float) -> void:
	# Drag force calculation: F_d = 0.5 * rho * v^2 * Cd * A
	var air_density = 1.225  # kg/m^3 at sea level
	var speed_squared = _current_speed * _current_speed
	
	_drag_force = 0.5 * air_density * speed_squared * aerodynamic_drag_coefficient * frontal_area
	
	# Downforce increases with speed squared
	_downforce = _drag_force * 0.5  # Typical downforce coefficient ratio

func _apply_aerodynamic_effects() -> void:
	# Apply drag opposite to velocity
	var drag_force_vector = -velocity.normalized() * _drag_force
	velocity += drag_force_vector * get_physics_process_delta_time()

# ============================================================================
# PHYSICS APPLICATION
# ============================================================================

func _apply_physics(delta: float) -> void:
	# Apply acceleration
	velocity += _acceleration_vector * delta
	
	# Apply drag
	_apply_aerodynamic_effects()
	
	# Apply gravity
	velocity.y -= _physics_settings.gravity * delta
	
	# Handle collisions
	if move_and_slide():
		# Ground contact established
		pass

func move_and_slide() -> bool:
	# Simple ground collision detection
	var ground_collision = move_and_collide(Vector3.DOWN * 1.0)
	
	if ground_collision:
		# On ground
		position.y = ground_collision.position.y + wheel_radius
		velocity.y = 0.0
		return true
	
	# In air
	return false

# ============================================================================
# COLLISION DETECTION
# ============================================================================

func _check_collisions(delta: float) -> void:
	# Raycast for ground contact and obstacles
	var collision_shape = CollisionShape3D.new()
	collision_shape.shape = SphereShape3D.new()
	collision_shape.shape.radius = wheel_radius
	
	add_child(collision_shape)
	
	var collision_result = collide_with_shape(collision_shape, transform)
	
	if collision_result:
		_handle_collision(collision_result)
	
	collision_shape.queue_free()

func collide_with_shape(shape: Shape3D, transform: Transform3D) -> bool:
	# Simplified collision check
	return false

func _handle_collision(result: CollisionResult) -> void:
	_collision_normal = result.normal
	_collision_velocity = result.get_collision_normal()
	_collision_impact_force = result.get_impact_force()
	
	var collision_info = {
		"normal": _collision_normal,
		"velocity": _collision_velocity,
		"impact_force": _collision_impact_force,
		"position": result.position
	}
	
	emit_signal("collision_detected", collision_info)

# ============================================================================
# VEHICLE STATE MANAGEMENT
# ============================================================================

func _set_vehicle_mass(new_mass: float) -> void:
	vehicle_mass = new_mass
	# Recalculate inertia tensor if needed
	_recalculate_inertia()

func _recalculate_inertia() -> void:
	# Placeholder for inertia recalculation
	pass

func _on_engine_ready() -> void:
	_current_rpm = MIN_IDLE_RPM

func get_current_speed() -> float:
	return _current_speed

func get_current_rpm() -> float:
	return _current_rpm

func get_current_gear() -> int:
	return _current_gear

func get_throttle_input() -> float:
	return _throttle_input

func get_brake_input() -> float:
	return _brake_input

func get_steering_angle() -> float:
	return _current_steering_angle

func get_wheels() -> Dictionary:
	return _wheel_positions

func get_wheel_forces() -> Dictionary:
	return _wheel_forces

func reset_lap() -> void:
	_lap_start_time = 0.0
	_current_lap_time = 0.0
	_checkpoints_passed.clear()
	_last_checkpoint_position = Vector3.ZERO

func start_lap() -> void:
	_lap_start_time = Time.get_unix_time_from_system()
	_current_lap_time = 0.0

func update_lap_time() -> void:
	if _lap_start_time > 0:
		var elapsed = Time.get_unix_time_from_system() - _lap_start_time
		_current_lap_time = elapsed

func get_lap_time() -> float:
	return _current_lap_time

func add_checkpoint(checkpoint_id: int) -> void:
	if checkpoint_id > _checkpoints_passed.size() or checkpoint_id == 0:
		_checkpoints_passed.append(checkpoint_id)

func is_lap_complete() -> bool:
	return _checkpoints_passed.size() > 0

func complete_lap() -> void:
	update_lap_time()
	emit_signal("lap_completed", _current_lap_time)
	reset_lap()

# ============================================================================
# DEBUG & VISUALIZATION
# ============================================================================

func _draw_debug_overlays() -> void:
	if not GameManager.debug_mode:
		return
	
	# Draw wheel positions
	for wheel in _wheel_positions:
		var pos = _wheel_positions[wheel] + position
		RenderingServer.debug_draw_line(pos, pos + Vector3.UP * 0.5, Color.YELLOW)
	
	# Draw velocity vector
	RenderingServer.debug_draw_line(position, position + velocity * 0.1, Color.RED)
	
	# Draw steering angle indicator
	var steer_color = Color.GREEN if abs(_current_steering_angle) < PI / 8 else Color.RED
	RenderingServer.debug_draw_line(position, position + Vector3.RIGHT * _current_steering_angle * 2.0, steer_color)

# ============================================================================
# POWERTRAIN INTEGRATION
# ============================================================================

func connect_to_powertrain(powertrain_node: Node) -> void:
	_powertrain = powertrain_node
	_connect_signals()

func disconnect_from_powertrain() -> void:
	if _powertrain != null:
		_disconnect_signals()
	_powertrain = null

func _disconnect_signals() -> void:
	if _powertrain != null:
		_powertrain.disconnect("engine_ready", Callable(self, "_on_engine_ready"))

# ============================================================================
# UTILITIES
# ============================================================================

func apply_damage(impact_force: float) -> void:
	if impact_force > 5000.0:
		# Significant damage
		print("[VehicleController] Critical impact detected! Damage applied.")
		# Would trigger particle effects, audio, etc.
		pass
	elif impact_force > 2000.0:
		# Minor damage
		print("[VehicleController] Moderate impact detected.")
		pass

func recover_from_spinout() -> void:
	# Gradually reduce speed during spinout
	if abs(_current_speed) > 10.0:
		velocity *= 0.95
		_current_rpm *= 0.95

func emergency_brake() -> void:
	_brake_input = 1.0
	_is_braking = true

func unlock_controls() -> void:
	# Unlock player controls after respawn/pause
	pass

func lock_controls() -> void:
	# Lock player controls during cutscenes/failures
	pass

</file_content>