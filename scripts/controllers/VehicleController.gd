extends CharacterBody3D
class_name VehicleController

## VehicleController - Base class for vehicle physics control
## Handles throttle, brake, steering, wheel forces, and gear shifting
## Integrates with PhysicsSettings for centralized tuning
## Copyright 2026 Thalamus Racing Simulator Project

# Signal definitions
signal speed_changed(speed: float)
signal gear_changed(old_gear: int, new_gear: int)
signal traction_loss(percentage: float)
signal vehicle_collision(impact_velocity: float, impact_point: Vector3)
signal engine_rpm_changed(rpm: float)

# Vehicle state enum
enum VehicleState {
	IDLE,
	MOVING,
	DRIFTING,
	SKIDDING,
	JUMPING,
	CRASHED
}

# Gear configuration (matches Powertrain gear ratios)
const MAX_GEAR: int = 6
const REVERSE_GEAR: int = -1
const NEUTRAL_GEAR: int = 0

@export_group("Physics Configuration")
@export var max_speed_kmh: float = 320.0
@export var acceleration_force: float = 5000.0
@export var braking_force: float = 8000.0
@export var turning_speed: float = 3.5
@export var friction_coefficient: float = 0.92
@export var air_resistance: float = 0.005

@export_group("Wheel Configuration")
@export var wheel_radius: float = 0.35
@export var track_width: float = 1.6
@export var wheel_offset_y: float = 0.5
@export var wheel_offset_z: float = 1.0

@export_group("Drift & Traction Settings")
@export var drift_threshold: float = 15.0
@export var drift_recovery_rate: float = 0.95
@export var grip_recovery_rate: float = 0.98
@export var minimum_traction: float = 0.3

# Internal state variables
var _current_speed: float = 0.0
var _current_gear: int = 0
var _target_gear: int = 0
var _throttle_input: float = 0.0
var _brake_input: float = 0.0
var _steering_input: float = 0.0
var _vehicle_state: VehicleState = VehicleState.IDLE
var _is_drifting: bool = false
var _drift_angle: float = 0.0
var _traction_level: float = 1.0
var _engine_rpm: float = 0.0
var _max_engine_rpm: float = 8000.0
var _idle_rpm: float = 800.0
var _torque_curve: Dictionary = {}

# References to child nodes
var _powertrain_node: Node = null
var _suspension_nodes: Array[Node] = []
var _wheel_colliders: Array[CollisionShape3D] = []

# Collision tracking
var _last_collision_time: float = 0.0
var _collision_impact_velocity: float = 0.0

func _ready() -> void:
	_init_vehicle_state()
	_connect_signals_to_powertrain()
	_setup_wheel_colliders()
	_build_torque_curve()
	
	# Ensure physics process runs at high rate for accurate vehicle physics
	process_physics_priority = 10
	
	# Log vehicle initialization
	print("[VehicleController] %s initialized at position: %s" % [name, global_position])

func _physics_process(delta: float) -> void:
	if not is_inside_tree():
		return
	
	_handle_input(delta)
	_update_gearing(delta)
	_calculate_vehicle_dynamics(delta)
	_apply_forces(delta)
	_update_visual_state(delta)
	_check_vehicle_state_changes()

func _input(event: InputEvent) -> void:
	"""Handle direct input events for keyboard/mouse support"""
	if event.is_action_pressed("shift_up"):
		_shift_gear(1)
	elif event.is_action_pressed("shift_down"):
		_shift_gear(-1)

func _init_vehicle_state() -> void:
	"""Initialize vehicle state variables to default values"""
	_current_speed = 0.0
	_current_gear = REVERSE_GEAR if _get_initial_direction() == -1 else 0
	_target_gear = _current_gear
	_throttle_input = 0.0
	_brake_input = 0.0
	_steering_input = 0.0
	_vehicle_state = VehicleState.IDLE
	_is_drifting = false
	_drift_angle = 0.0
	_traction_level = 1.0
	_engine_rpm = _idle_rpm

func _get_initial_direction() -> int:
	"""Get initial movement direction based on vehicle orientation"""
	var forward_dir = transform.basis.z.normalized()
	if forward_dir.dot(Vector3.FORWARD) < 0:
		return -1
	return 1

func _connect_signals_to_powertrain() -> void:
	"""Connect to Powertrain node signals if present"""
	_powertrain_node = get_node_or_null("../Powertrain")
	if _powertrain_node != null:
		_powertrain_node.connect("rpm_changed", _on_powertrain_rpm_changed)
		_powertrain_node.connect("gear_changed", _on_powertrain_gear_changed)

func _setup_wheel_colliders() -> void:
	"""Setup wheel collision shapes if they exist in the scene"""
	for child in get_children():
		if child is CollisionShape3D and child.name.contains("Wheel"):
			_wheel_colliders.append(child)
		elif child.name.contains("Suspension"):
			_suspension_nodes.append(child)

func _build_torque_curve() -> void:
	"""Build torque curve dictionary mapping RPM to torque percentage"""
	_torque_curve = {
		_idle_rpm * 0.3: 0.3,      # Low RPM idle
		_idle_rpm * 0.6: 0.5,      # Low RPM torque start
		_max_engine_rpm * 0.3: 0.6,   # Mid-range torque
		_max_engine_rpm * 0.5: 0.85,  # Peak torque region
		_max_engine_rpm * 0.7: 0.95,  # High torque
		_max_engine_rpm * 0.9: 0.9,   # Declining torque
		_max_engine_rpm * 1.0: 0.7    # Redline
	}

func _handle_input(delta: float) -> void:
	"""Process input and update control values"""
	# Get input axes (normalized -1 to 1)
	_throttle_input = Input.get_axis("brake", "accelerate")
	_brake_input = Input.get_axis("brake", "accelerate")
	_steering_input = Input.get_axis("turn_left", "turn_right")
	
	# Clamp inputs to valid range
	_throttle_input = clamp(_throttle_input, -1.0, 1.0)
	_brake_input = clamp(_brake_input, -1.0, 1.0)
	_steering_input = clamp(_steering_input, -1.0, 1.0)
	
	# Handle manual gear shifting
	if Input.is_action_just_pressed("shift_up"):
		_shift_gear(1)
	elif Input.is_action_just_pressed("shift_down"):
		_shift_gear(-1)

func _update_gearing(delta: float) -> void:
	"""Update current gear based on speed and driver input"""
	var target_gear = _calculate_target_gear()
	
	if target_gear != _current_gear:
		_shift_gear_internal(target_gear)
	
	# Update engine RPM based on gear and speed
	_update_engine_rpm(delta)

func _calculate_target_gear() -> int:
	"""Calculate optimal gear based on current speed and throttle input"""
	if abs(_throttle_input) < 0.05 and _current_speed > 0.1:
		return NEUTRAL_GEAR
	
	var speed_factor = _current_speed / max_speed_kmh * 1000.0
	
	# Simple gear calculation based on speed thresholds
	if _current_gear == REVERSE_GEAR and _throttle_input > 0:
		return 1
	
	if _current_speed < 10.0:
		return 1
	elif _current_speed < 25.0:
		return 2
	elif _current_speed < 45.0:
		return 3
	elif _current_speed < 70.0:
		return 4
	elif _current_speed < 100.0:
		return 5
	else:
		return MAX_GEAR

func _shift_gear(direction: int) -> void:
	"""Shift gear by specified direction (-1 down, 1 up)"""
	var old_gear = _current_gear
	var new_gear = _current_gear + direction
	
	# Prevent invalid gear shifts
	if new_gear > MAX_GEAR or new_gear < REVERSE_GEAR:
		return
	
	# Prevent neutral jumps without clutch simulation
	if old_gear == NEUTRAL_GEAR and new_gear != NEUTRAL_GEAR:
		_current_gear = new_gear
	elif old_gear != NEUTRAL_GEAR and new_gear != NEUTRAL_GEAR:
		_current_gear = new_gear
	
	if _current_gear != old_gear:
		gear_changed.emit(old_gear, _current_gear)
		print("[VehicleController] Gear shifted: %d -> %d" % [old_gear, _current_gear])

func _shift_gear_internal(new_gear: int) -> void:
	"""Internal gear shift without boundary checking"""
	var old_gear = _current_gear
	_current_gear = new_gear
	if old_gear != new_gear:
		gear_changed.emit(old_gear, new_gear)

func _update_engine_rpm(delta: float) -> void:
	"""Update engine RPM based on vehicle speed and gear ratio"""
	var gear_ratio = _get_gear_ratio(_current_gear)
	var final_drive_ratio = 3.5  # Typical final drive ratio
	var wheel_speed = _current_speed * (1000.0 / 3600.0) / (2.0 * PI * wheel_radius)
	
	# Calculate engine RPM
	var rpm = wheel_speed * gear_ratio * final_drive_ratio * 60.0
	
	# Clamp to valid RPM range
	_engine_rpm = clamp(rpm, _idle_rpm, _max_engine_rpm * 1.2)
	
	engine_rpm_changed.emit(_engine_rpm)

func _get_gear_ratio(gear: int) -> float:
	"""Get gear ratio for specified gear"""
	var gear_ratios = {
		REVERSE_GEAR: 3.8,
		1: 4.5,
		2: 2.9,
		3: 2.1,
		4: 1.6,
		5: 1.3,
		6: 1.0,
		NEUTRAL_GEAR: 0.0
	}
	return gear_ratios.get(gear, 0.0)

func _calculate_vehicle_dynamics(delta: float) -> void:
	"""Calculate vehicle dynamics including acceleration, deceleration, and turning"""
	var current_magnitude = velocity.length()
	
	# Apply acceleration or braking
	if _throttle_input > 0:
		# Accelerate
		var accel_force = _apply_acceleration()
		velocity.x += accel_force * delta * sign(velocity.x) if velocity.x != 0 else 0.0
		velocity.z += accel_force * delta
	elif _brake_input > 0:
		# Brake
		velocity.linear_interpolate(Vector3.ZERO, _brake_force_multiplier() * delta)
	else:
		# Coasting with friction
		velocity *= pow(friction_coefficient, delta * 60)
	
	# Apply steering
	if _current_speed > 0.5:
		_apply_steering(delta)
	
	# Store current speed for signals
	_current_speed = velocity.length()
	speed_changed.emit(_current_speed)

func _apply_acceleration() -> float:
	"""Apply acceleration force based on throttle input and gear"""
	var torque_multiplier = _get_torque_at_rpm()
	var gear_ratio = _get_gear_ratio(_current_gear)
	
	if gear_ratio <= 0:
		return 0.0
	
	var effective_force = acceleration_force * _throttle_input * torque_multiplier * gear_ratio
	return effective_force

func _brake_force_multiplier() -> float:
	"""Calculate brake force multiplier"""
	return braking_force / (vehicle_mass() + 500.0)  # Adjust for weight

func _apply_steering(delta: float) -> void:
	"""Apply steering to change vehicle direction"""
	if abs(velocity.length()) < 0.1:
		return
	
	var turn_amount = _steering_input * turning_speed * (velocity.length() / 10.0)
	
	# Rotate velocity vector
	var rotation = Quaternion(transform.basis.z, deg_to_rad(turn_amount))
	velocity = transform.basis * rotation * velocity

func _apply_forces(delta: float) -> void:
	"""Apply calculated forces to the rigid body"""
	# Air resistance
	var drag = velocity.length_squared() * air_resistance * delta
	velocity -= velocity.normalized() * drag
	
	# Apply gravity if jumping
	if _vehicle_state == VehicleState.JUMPING:
		velocity.y -= PhysicsSettings.gravity * delta
	
	# Move character body
	move_and_slide()

func _update_visual_state(delta: float) -> void:
	"""Update visual indicators based on vehicle state"""
	# Update suspension visualizers if they exist
	for suspension in _suspension_nodes:
		if suspension.has_method("_update_visual"):
			suspension._update_visual(delta, velocity.y, _vehicle_state)

func _check_vehicle_state_changes() -> void:
	"""Check and update vehicle state based on conditions"""
	var was_state = _vehicle_state
	
	# Determine new state
	if abs(velocity.length()) < 0.1:
		_vehicle_state = VehicleState.IDLE
	elif _is_skidding():
		_vehicle_state = VehicleState.SKIDDING
	elif _is_drifting():
		_vehicle_state = VehicleState.DRIFTING
	elif velocity.y < -2.0:
		_vehicle_state = VehicleState.JUMPING
	else:
		_vehicle_state = VehicleState.MOVING
	
	# Emit state change signal if different
	if was_state != _vehicle_state:
		print("[VehicleController] State changed: %d -> %d" % [was_state, _vehicle_state])

func _is_drifting() -> bool:
	"""Check if vehicle is in drift mode"""
	if _current_speed < drift_threshold:
		return false
	
	var slip_angle = _calculate_slip_angle()
	return abs(slip_angle) > 15.0 and abs(_steering_input) > 0.3

func _is_skidding() -> bool:
	"""Check if vehicle is skidding due to low traction"""
	return _traction_level < 0.5 and _current_speed > 10.0

func _calculate_slip_angle() -> float:
	"""Calculate vehicle slip angle for drift detection"""
	var forward_dir = transform.basis.z.normalized()
	var velocity_dir = velocity.normalized() if velocity.length() > 0.1 else Vector3.ZERO
	
	var dot_product = forward_dir.dot(velocity_dir)
	var cross_product = forward_dir.cross(velocity_dir).y
	
	return rad_to_deg(atan2(cross_product, dot_product))

func _get_torque_at_rpm() -> float:
	"""Get torque percentage at current RPM from torque curve"""
	var rpm = _engine_rpm
	
	# Linear interpolation through torque curve points
	var points = _torque_curve.keys()
	points.sort()
	
	for i in range(len(points) - 1):
		if rpm >= points[i] and rpm <= points[i + 1]:
			var t = (rpm - points[i]) / (points[i + 1] - points[i])
			return lerp(_torque_curve[points[i]], _torque_curve[points[i + 1]], t)
	
	return _torque_curve[points[-1]] if points.size() > 0 else 0.5

func _get_current_traction() -> float:
	"""Get current traction level (0-1)"""
	return _traction_level

func _set_traction_level(level: float) -> void:
	"""Set traction level (affects grip)"""
	_traction_level = clamp(level, 0.0, 1.0)
	traction_loss.emit(1.0 - _traction_level)

func vehicle_mass() -> float:
	"""Get vehicle mass in kg"""
	return PhysicsSettings.default_vehicle_mass

func reset_vehicle() -> void:
	"""Reset vehicle to initial state"""
	velocity = Vector3.ZERO
	angular_velocity = Vector3.ZERO
	_current_speed = 0.0
	_current_gear = 0
	_throttle_input = 0.0
	_brake_input = 0.0
	_steering_input = 0.0
	_engine_rpm = _idle_rpm
	_set_traction_level(1.0)
	_vehicle_state = VehicleState.IDLE

func apply_collision_impact(impact_velocity: float, impact_point: Vector3) -> void:
	"""Handle collision impact effects"""
	_collision_impact_velocity = impact_velocity
	_last_collision_time = Time.get_unix_time_from_system()
	
	vehicle_collision.emit(impact_velocity, impact_point)
	
	# Apply damage based on impact severity
	if impact_velocity > 30.0:
		_set_traction_level(max(0.1, _traction_level - 0.3))
		_vehicle_state = VehicleState.CRASHED
		await get_tree().create_timer(2.0).timeout
		_reset_crash_state()

func _reset_crash_state() -> void:
	"""Reset vehicle after crash"""
	_vehicle_state = VehicleState.MOVING
	_set_traction_level(min(1.0, _traction_level + 0.2))

func _on_powertrain_rpm_changed(rpm: float) -> void:
	"""Handle Powertrain RPM change signals"""
	_engine_rpm = rpm
	engine_rpm_changed.emit(rpm)

func _on_powertrain_gear_changed(old_gear: int, new_gear: int) -> void:
	"""Handle Powertrain gear change signals"""
	_current_gear = new_gear
	gear_changed.emit(old_gear, new_gear)

## Helper methods for external access
func get_current_speed_kmh() -> float:
	"""Get current speed in km/h"""
	return _current_speed * 3.6

func get_current_gear() -> int:
	"""Get current gear number"""
	return _current_gear

func get_engine_rpm() -> float:
	"""Get current engine RPM"""
	return _engine_rpm

func get_vehicle_state() -> VehicleState:
	"""Get current vehicle state"""
	return _vehicle_state

func is_moving() -> bool:
	"""Check if vehicle is moving"""
	return _current_speed > 0.5

func is_in_gear() -> bool:
	"""Check if vehicle is in a driving gear"""
	return _current_gear != NEUTRAL_GEAR and _current_gear != REVERSE_GEAR

func get_forward_vector() -> Vector3:
	"""Get normalized forward vector"""
	return transform.basis.z.normalized()

func get_right_vector() -> Vector3:
	"""Get normalized right vector"""
	return transform.basis.x.normalized()

func set_max_speed(kmh: float) -> void:
	"""Set maximum speed in km/h"""
	max_speed_kmh = kmh

func set_acceleration(force: float) -> void:
	"""Set acceleration force"""
	acceleration_force = force

func set_braking_force(force: float) -> void:
	"""Set braking force"""
	braking_force = force
</FILE>