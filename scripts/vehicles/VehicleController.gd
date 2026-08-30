extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulation
## Handles throttle, brake, steering, wheel forces, gear shifting, and vehicle dynamics
## Integrates with PhysicsSettings constants and Powertrain system
## Copyright 2026 Thalamus Racing Simulator Project

# Signals for external communication
signal speed_changed(new_speed: float)
signal rpm_changed(new_rpm: int)
signal gear_changed(old_gear: int, new_gear: int)
signal vehicle_collided(impact_force: Vector3)
signal engine_stalled()
signal clutch_disengaged()
signal clutch_engaged()

# Required references (set by parent scene)
var powertrain: Node = null
var chassis: Node3D = null

# Input state tracking
var _throttle_input: float = 0.0
var _brake_input: float = 0.0
var _steering_input: float = 0.0
var _clutch_input: float = 0.0
var _gear_shift_request: int = 0

# Vehicle state variables
var current_gear: int = 1
var target_gear: int = 1
var current_rpm: int = 800
var max_rpm: int = 7000
var min_rpm: int = 800
var vehicle_mass: float = 1500.0
var current_speed: float = 0.0  # meters per second
var acceleration: Vector3 = Vector3.ZERO
var angular_velocity: Vector3 = Vector3.ZERO

# Wheel positions (local space relative to chassis)
var front_left_wheel_pos: Vector3
var front_right_wheel_pos: Vector3
var rear_left_wheel_pos: Vector3
var rear_right_wheel_pos: Vector3

# Wheel force calculation results
var front_left_drive_force: float = 0.0
var front_right_drive_force: float = 0.0
var rear_left_drive_force: float = 0.0
var rear_right_drive_force: float = 0.0
var front_left_brake_force: float = 0.0
var front_right_brake_force: float = 0.0
var rear_left_brake_force: float = 0.0
var rear_right_brake_force: float = 0.0

# Physics configuration (from PhysicsSettings singleton)
var _physics_settings: PhysicsSettings = null
var _wheel_radius: float = 0.33
var _track_width: float = 1.5
var _wheel_base: float = 2.5
var _max_steering_angle: float = 35.0 * deg_to_rad(1)  # Convert degrees to radians

# Gear ratios and final drive ratio
var _gear_ratios: Dictionary = {
	1: 3.8,
	2: 2.4,
	3: 1.7,
	4: 1.3,
	5: 1.0,
	6: 0.85,
	7: 0.7,
	reverse: -4.2
}
var _final_drive_ratio: float = 3.5
var _tire_friction_coefficient: float = 1.2

# Engine torque curve (simplified lookup table)
var _engine_torque_curve: Array[float] = [
	0.0,   # 0% RPM
	0.35,  # 10% RPM
	0.65,  # 20% RPM
	0.85,  # 30% RPM
	0.95,  # 40% RPM
	1.0,   # 50% RPM
	1.0,   # 60% RPM
	0.98,  # 70% RPM
	0.95,  # 80% RPM
	0.90,  # 90% RPM
	0.80   # 100% RPM
]

# Clutch engagement state
var _clutch_engaged: bool = true
var _clutch_engagement_progress: float = 1.0

# Auto-shift settings
var _auto_shift_enabled: bool = false
var _shift_up_at_rpm: int = 6800
var _shift_down_at_rpm: int = 1200
var _gear_hold_time: float = 0.5  # seconds before allowing another shift

# Timing variables
var _last_shift_time: float = -999.0
var _last_update_time: float = 0.0

func _ready() -> void:
	_init_references()
	_connect_signals()
	_calculate_wheel_positions()
	_load_physics_settings()
	_reset_vehicle_state()

func _init_references() -> void:
	# Find child nodes
	chassis = find_child("Chassis") or self
	powertrain = find_child("Powertrain")
	
	if powertrain == null:
		push_warning("VehicleController: No Powertrain node found!")
	else:
		_connect_powertrain_signals()

func _connect_powertrain_signals() -> void:
	if powertrain.has_signal("rpm_changed"):
		powertrain.rpm_changed.connect(_on_powertrain_rpm_changed)
	if powertrain.has_signal("torque_delivered"):
		powertrain.torque_delivered.connect(_on_powertrain_torque_delivered)

func _load_physics_settings() -> void:
	# Load settings from singleton if available
	if Engine.has_singleton("PhysicsSettings"):
		_physics_settings = Engine.get_singleton("PhysicsSettings")
	elif has_node("/root/PhysicsSettings"):
		_physics_settings = get_node("/root/PhysicsSettings")
	else:
		_physics_settings = preload("res://scripts/core/PhysicsSettings.gd").new()

func _reset_vehicle_state() -> void:
	current_gear = 1
	target_gear = 1
	current_rpm = 800
	current_speed = 0.0
	_throttle_input = 0.0
	_brake_input = 0.0
	_steering_input = 0.0
	_clutch_input = 0.0
	_clutch_engaged = true
	_clutch_engagement_progress = 1.0
	_auto_shift_enabled = false
	gear_changed.emit(current_gear, current_gear)

func _calculate_wheel_positions() -> void:
	# Calculate wheel positions relative to chassis center
	var half_track: float = _track_width / 2.0
	var half_wheelbase: float = _wheel_base / 2.0
	
	front_left_wheel_pos = Vector3(-half_track, 0.0, -half_wheelbase)
	front_right_wheel_pos = Vector3(half_track, 0.0, -half_wheelbase)
	rear_left_wheel_pos = Vector3(-half_track, 0.0, half_wheelbase)
	rear_right_wheel_pos = Vector3(half_track, 0.0, half_wheelbase)

func _get_torque_at_rpm(rpm: int) -> float:
	"""Get engine torque multiplier based on current RPM"""
	var rpm_percentage: float = clampf(float(rpm) / float(max_rpm), 0.0, 1.0)
	var index: int = floor(rpm_percentage * (_engine_torque_curve.size() - 1))
	index = clampi(index, 0, _engine_torque_curve.size() - 2)
	var t1: float = _engine_torque_curve[index]
	var t2: float = _engine_torque_curve[index + 1]
	var alpha: float = (float(rpm) / float(max_rpm) - float(index) / float(_engine_torque_curve.size() - 1)) * (_engine_torque_curve.size() - 1)
	return t1 + (t2 - t1) * alpha

func _update_rpm(delta: float) -> void:
	"""Update engine RPM based on vehicle speed and gear"""
	if not _clutch_engaged:
		# Engine idles when clutch disengaged
		current_rpm = lerp(current_rpm, 800, delta * 5.0)
		return
	
	var gear_ratio: float = _gear_ratios[current_gear] if current_gear > 0 else _gear_ratios["reverse"]
	var wheel_angular_velocity: float = current_speed / (_wheel_radius * 0.5)  # Simplified
	var engine_angular_velocity: float = wheel_angular_velocity * gear_ratio * _final_drive_ratio
	
	# Convert to RPM (radians/sec to rev/min)
	var target_rpm: int = int(wheel_angular_velocity * gear_ratio * _final_drive_ratio * 9.549)
	
	# Smooth RPM transition
	current_rpm = lerp(current_rpm, target_rpm, delta * 10.0)
	
	# Clamp RPM
	current_rpm = clampi(current_rpm, min_rpm, max_rpm)
	rpm_changed.emit(current_rpm)

func _calculate_drive_forces() -> void:
	"""Calculate drive forces for each wheel based on throttle and gear"""
	if not _clutch_engaged:
		force_all_wheels(0.0, 0.0)
		return
	
	var gear_ratio: float = _gear_ratios[current_gear] if current_gear > 0 else _gear_ratios["reverse"]
	var total_ratio: float = gear_ratio * _final_drive_ratio
	
	# Get engine torque at current RPM
	var torque_multiplier: float = _get_torque_at_rpm(current_rpm)
	var max_engine_torque: float = 450.0  # Nm (typical performance car)
	var engine_torque: float = max_engine_torque * torque_multiplier
	
	# Apply throttle input
	var effective_torque: float = engine_torque * _throttle_input * _clutch_engagement_progress
	
	# Account for gear ratio and final drive
	var wheel_torque: float = effective_torque * total_ratio
	
	# Distribute torque to wheels (RWD default, can be overridden)
	var drive_type: String = "RWD"  # Could be exposed as export
	var rear_wheel_torque: float = 0.0
	var front_wheel_torque: float = 0.0
	
	if drive_type == "RWD":
		rear_wheel_torque = wheel_torque
	elif drive_type == "FWD":
		front_wheel_torque = wheel_torque
	elif drive_type == "AWD":
		rear_wheel_torque = wheel_torque * 0.6
		front_wheel_torque = wheel_torque * 0.4
	
	# Convert torque to linear force (F = T / r)
	var rear_force: float = rear_wheel_torque / _wheel_radius
	var front_force: float = front_wheel_torque / _wheel_radius
	
	# Apply to appropriate wheels
	if drive_type != "FWD":
		rear_left_drive_force = rear_force * 0.5
		rear_right_drive_force = rear_force * 0.5
	if drive_type != "RWD":
		front_left_drive_force = front_force * 0.5
		front_right_drive_force = front_force * 0.5

func _calculate_brake_forces() -> void:
	"""Calculate brake forces for each wheel"""
	var max_brake_pressure: float = 15000.0  # Newtons
	var brake_distribution: float = 0.6  # 60% front, 40% rear typical
	
	var total_brake_force: float = max_brake_pressure * _brake_input * _clutch_engagement_progress
	
	var front_total: float = total_brake_force * brake_distribution
	var rear_total: float = total_brake_force * (1.0 - brake_distribution)
	
	front_left_brake_force = front_total * 0.5
	front_right_brake_force = front_total * 0.5
	rear_left_brake_force = rear_total * 0.5
	rear_right_brake_force = rear_total * 0.5

func _apply_driving_forces() -> void:
	"""Apply calculated forces to vehicle body"""
	var forward_vector: Vector3 = -transform.basis.z
	
	# Apply drive forces to rear wheels (RWD example)
	if rear_left_drive_force > 0:
		apply_impulse(forward_vector * rear_left_drive_force * 0.016)  # Scale factor
	if rear_right_drive_force > 0:
		apply_impulse(forward_vector * rear_right_drive_force * 0.016)
	
	# Apply drive forces to front wheels (FWD/AWD)
	if front_left_drive_force > 0:
		apply_impulse(forward_vector * front_left_drive_force * 0.016)
	if front_right_drive_force > 0:
		apply_impulse(forward_vector * front_right_drive_force * 0.016)

func _apply_braking_forces() -> void:
	"""Apply brake forces (simplified - reduces velocity)"""
	if _brake_input > 0.01:
		# Simple braking by reducing forward velocity
		var forward_vel: float = velocity.dot(-transform.basis.z)
		var brake_decel: float = 15.0 * _brake_input  # m/s²
		
		if forward_vel > 0:
			var new_forward_vel: float = forward_vel - brake_decel * 0.016
			new_forward_vel = maxf(new_forward_vel, 0.0)
			
			var forward_axis: Vector3 = -transform.basis.z.normalized()
			var current_forward_vel_vec: Vector3 = velocity * forward_axis
			
			velocity = velocity - current_forward_vel_vec + (forward_axis * new_forward_vel)

func _handle_steering() -> void:
	"""Handle steering input and apply rotation"""
	if current_speed < 0.5:  # No steering at very low speeds
		angular_velocity.y = 0.0
		return
	
	var steering_speed: float = 3.0  # rad/s max turn rate
	var desired_turn_rate: float = _steering_input * steering_speed
	
	# Limit turn rate based on speed
	var speed_factor: float = clampf(current_speed / 20.0, 0.0, 1.0)
	desired_turn_rate *= speed_factor
	
	# Apply smooth turn rate
	angular_velocity.y = lerp(angular_velocity.y, desired_turn_rate, 0.1)
	
	# Apply rotation
	rotate_y(angular_velocity.y * 0.016)

func _handle_gear_shifting(delta: float) -> void:
	"""Handle automatic or manual gear shifting"""
	var time_since_last_shift: float = Time.get_unix_time_from_system() - _last_shift_time
	
	# Check for auto-shift
	if _auto_shift_enabled:
		if current_rpm > _shift_up_at_rpm and current_gear < 7:
			_request_gear_shift(current_gear + 1)
		elif current_rpm < _shift_down_at_rpm and current_gear > 1:
			_request_gear_shift(current_gear - 1)
	
	# Check for manual shift request
	if _gear_shift_request != 0:
		if time_since_last_shift > _gear_hold_time:
			if _gear_shift_request > 0 and current_gear < 7:
				_request_gear_shift(current_gear + 1)
			elif _gear_shift_request < 0 and current_gear > 1:
				_request_gear_shift(current_gear - 1)
			_gear_shift_request = 0
	
	# Handle clutch input
	if abs(_clutch_input) > 0.5:
		_clutch_engaged = false
		_clutch_engagement_progress = lerp(_clutch_engagement_progress, 0.0, delta * 5.0)
		clutch_disengaged.emit()
	else:
		if not _clutch_engaged and _clutch_engagement_progress < 0.1:
			_clutch_engaged = true
			_clutch_engagement_progress = lerp(_clutch_engagement_progress, 1.0, delta * 10.0)
			clutch_engaged.emit()

func _request_gear_shift(new_gear: int) -> void:
	"""Request a gear change"""
	if new_gear == current_gear:
		return
	
	target_gear = new_gear
	_last_shift_time = Time.get_unix_time_from_system()
	
	# Simulate clutch disengagement during shift
	_clutch_engaged = false
	await get_tree().create_timer(0.2).timeout
	_clutch_engaged = true
	
	current_gear = new_gear
	gear_changed.emit(target_gear, current_gear)

func _process_inputs() -> void:
	"""Read and process input states"""
	# These should connect to InputManager in full implementation
	# For now, use direct Godot input for demo purposes
	
	# Throttle (W, Up Arrow, or gas pedal)
	if Input.is_action_pressed("move_forward"):
		_throttle_input = 1.0
	elif Input.is_action_pressed("move_backward"):
		_throttle_input = -0.3  # Reverse throttle
	else:
		_throttle_input = 0.0
	
	# Brake (S, Down Arrow, or brake pedal)
	if Input.is_action_pressed("brake"):
		_brake_input = 1.0
	else:
		_brake_input = 0.0
	
	# Steering (A/D or Left/Right Arrow)
	if Input.is_action_pressed("turn_left"):
		_steering_input = 1.0
	elif Input.is_action_pressed("turn_right"):
		_steering_input = -1.0
	else:
		_steering_input = 0.0
	
	# Clutch (Q or space)
	if Input.is_action_pressed("clutch"):
		_clutch_input = 1.0
	else:
		_clutch_input = 0.0
	
	# Gear shift (PageUp/PageDown or R/L)
	if Input.is_action_just_pressed("shift_up"):
		_gear_shift_request = 1
	elif Input.is_action_just_pressed("shift_down"):
		_gear_shift_request = -1

func _update_vehicle_state(delta: float) -> void:
	"""Update vehicle state variables"""
	# Update speed from velocity
	var forward_speed: float = velocity.dot(-transform.basis.z)
	current_speed = absf(forward_speed)
	speed_changed.emit(current_speed)
	
	# Update RPM based on gear and speed
	_update_rpm(delta)

func _on_powertrain_rpm_changed(new_rpm: int) -> void:
	current_rpm = new_rpm
	rpm_changed.emit(new_rpm)

func _on_powertrain_torque_delivered(torque: float) -> void:
	pass  # Torque already calculated locally

func _on_collision_started(body: Node) -> void:
	if body.has_method("take_damage"):
		body.take_damage(10.0)  # Simplified damage

func _physics_process(delta: float) -> void:
	# Validate delta
	if delta > 0.1:
		return
	
	# Process inputs
	_process_inputs()
	
	# Update vehicle state
	_update_vehicle_state(delta)
	
	# Handle gear shifting
	_handle_gear_shifting(delta)
	
	# Calculate wheel forces
	_calculate_drive_forces()
	_calculate_brake_forces()
	
	# Apply forces
	_apply_driving_forces()
	_apply_braking_forces()
	
	# Handle steering
	_handle_steering()
	
	# Move character body
	move_and_slide()

func set_auto_shift(enabled: bool) -> void:
	_auto_shift_enabled = enabled

func set_gear(gear: int) -> void:
	if gear >= 1 and gear <= 7:
		_request_gear_shift(gear)
	elif gear == -1:  # Reverse
		_request_gear_shift(-1)

func get_current_speed_kmh() -> float:
	return current_speed * 3.6

func get_current_speed_mph() -> float:
	return current_speed * 2.237

func get_current_gear() -> int:
	return current_gear

func is_clutch_engaged() -> bool:
	return _clutch_engaged

func reset() -> void:
	_reset_vehicle_state()
	velocity = Vector3.ZERO
	angular_velocity = Vector3.ZERO

func _exit_tree() -> void:
	_disconnect_signals()

func _disconnect_signals() -> void:
	if powertrain and powertrain.has_signal("rpm_changed"):
		powertrain.rpm_changed.disconnect(_on_powertrain_rpm_changed)
	if powertrain and powertrain.has_signal("torque_delivered"):
		powertrain.torque_delivered.disconnect(_on_powertrain_torque_delivered)

## Public API for game logic interaction
func accelerate(amount: float) -> void:
	_throttle_input = clampf(_throttle_input + amount, 0.0, 1.0)

func brake(amount: float) -> void:
	_brake_input = clampf(_brake_input + amount, 0.0, 1.0)

func steer(amount: float) -> void:
	_steering_input = clampf(_steering_input + amount, -1.0, 1.0)

func engage_clutch() -> void:
	_clutch_input = 1.0

func release_clutch() -> void:
	_clutch_input = 0.0

func shift_up() -> void:
	_gear_shift_request = 1

func shift_down() -> void:
	_gear_shift_request = -1

func set_max_rpm(rpm: int) -> void:
	max_rpm = rpm

func set_min_rpm(rpm: int) -> void:
	min_rpm = rpm

func get_vehicle_mass() -> float:
	return vehicle_mass

func set_vehicle_mass(mass: float) -> void:
	vehicle_mass = mass
</FILE>