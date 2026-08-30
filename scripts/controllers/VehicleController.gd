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
signal lap_completed(lap_time: float)
signal checkpoint_passed(checkpoint_id: int)
signal boost_used(duration: float)

# ============================================================================
# CONSTANTS & CONFIGURATION
# ============================================================================
const MAX_STEERING_ANGLE: float = PI / 3.5  # Max 51 degrees steering
const MIN_STEERING_SPEED: float = 0.5       # Minimum steering response time
const DRIFT_THRESHOLD: float = 0.7          # Skid threshold for drift detection
const TRACTION_CONTROL_SENSITIVITY: float = 0.85
const ABS_LOCK_THRESHOLD: float = 0.1       # Wheel lock threshold for ABS
const GEAR_RATIOS: Array[float] = [0.0, 3.8, 2.5, 1.8, 1.3, 1.0, 0.8]  # Neutral + gears 1-6
const FINAL_DRIVE_RATIO: float = 4.1        # Final drive ratio
const CLUTCH_DISPERCTION_TIME: float = 0.3  # Clutch engagement time
const BOOST_MULTIPLIER: float = 1.5         # Boost power multiplier
const MAX_BOOST_DURATION: float = 5.0       # Maximum continuous boost time

# ============================================================================
# MEMBER VARIABLES
# ============================================================================
# Powertrain reference
var powertrain: Node = null

# Physical properties (overridable per vehicle)
@export var vehicle_mass: float = 1500.0
@export var center_of_mass_offset: Vector3 = Vector3(0.0, -0.3, 0.0)
@export var wheel_base: float = 2.5
@export var track_width: float = 1.5
@export var suspension_stiffness: float = 45000.0
@export var suspension_damping: float = 3500.0
@export var suspension_compression_limit: float = 0.2
@export var suspension_extension_limit: float = 0.2

# Tire properties
@export var tire_friction_coefficient: float = 1.2
@export var tire_vertical_stiffness: float = 50000.0
@export var lateral_slip_stiffness: float = 15000.0
@export var longitudinal_slip_stiffness: float = 25000.0

# Performance limits
@export var max_engine_rpm: float = 8000.0
@export var idle_rpm: float = 800.0
@export var redline_rpm: float = 7500.0
@export var torque_curve: Dictionary = {}
@export var power_curve: Dictionary = {}

# State variables
var current_gear: int = 0  # 0=neutral, 1-6=gears
var target_gear: int = 0
var clutch_engaged: bool = true
var clutch_progress: float = 0.0  # 0.0 to 1.0

# Speed measurements
var current_speed: float = 0.0  # km/h
var engine_rpm: float = 0.0
var wheel_rotation_angles: Dictionary = {}  # Per-wheel rotation angles

# Driving state
var is_throttling: bool = false
var is_braking: bool = false
var is_handbraking: bool = false
var steering_input: float = 0.0  # -1.0 to 1.0
var current_steering_angle: float = 0.0

# Advanced systems
var traction_control_enabled: bool = true
var abs_enabled: bool = true
var drift_mode_enabled: bool = false
var boost_available: bool = true
var boost_charge_level: float = 100.0

# Drift tracking
var drift_angle: float = 0.0
var drift_timer: float = 0.0
var drift_intensity: float = 0.0

# Lap tracking
var last_checkpoint_id: int = -1
var lap_start_time: float = 0.0
var current_lap_time: float = 0.0
var lap_count: int = 0

# Collision tracking
var last_collision_time: float = 0.0
var collision_impact_force: float = 0.0
var is_on_track: bool = true

# Timers and accumulators
var _time_since_last_update: float = 0.0
var _wheel_forces: Array[Vector3] = []
var _tire_slip_data: Dictionary = {}
var _boost_accumulator: float = 0.0
var _drift_accumulator: float = 0.0

# ============================================================================
# PUBLIC METHODS
# ============================================================================
func get_current_speed_kmh() -> float:
	"""Get current vehicle speed in km/h"""
	return current_speed

func get_current_rpm() -> float:
	"""Get current engine RPM"""
	return engine_rpm

func get_current_gear() -> int:
	"""Get current gear number"""
	return current_gear

func is_clutch_engaged() -> bool:
	"""Check if clutch is fully engaged"""
	return clutch_engaged and clutch_progress >= 0.95

func can_shift_to(gear: int) -> bool:
	"""Check if shift to specific gear is possible"""
	if gear < 0 or gear > 6:
		return false
	if not clutch_engaged and current_gear != gear:
		return false
	if gear == 0:
		return true
	if gear == current_gear:
		return true
	
	# Prevent dangerous downshifts at high RPM
	if gear < current_gear and engine_rpm > max_engine_rpm * 0.9:
		return false
	
	return true

func attempt_upshift() -> bool:
	"""Attempt automatic upshift"""
	if current_gear < 6 and engine_rpm > max_engine_rpm * 0.85:
		target_gear = min(current_gear + 1, 6)
		engage_clutch()
		await _clutch_engagement_finished()
		switch_gear(target_gear)
		return true
	return false

func attempt_downshift() -> bool:
	"""Attempt automatic downshift"""
	if current_gear > 1:
		target_gear = max(current_gear - 1, 1)
		engage_clutch()
		await _clutch_engagement_finished()
		switch_gear(target_gear)
		return true
	return false

func force_full_boost() -> bool:
	"""Activate full boost mode"""
	if not boost_available or boost_charge_level <= 0:
		return false
	boost_accumulator = BOOST_MULTIPLIER
	_emit_signal("boost_used", boost_accumulator)
	return true

func reset_boost() -> void:
	"""Reset boost charge"""
	boost_charge_level = 100.0
	boost_available = true

# ============================================================================
# INPUT HANDLING
# ============================================================================
func handle_input(delta: float) -> void:
	"""Process all driving input commands"""
	var input_manager = InputManager if Engine.has_singleton("InputManager") else null
	if input_manager:
		_process_throttle(input_manager.get_axis("throttle"))
		_process_brake(input_manager.get_axis("brake"), input_manager.get_button("handbrake"))
		_process_steering(input_manager.get_axis("steer_left_right"))
		
		# Handle gear shifts
		if input_manager.get_button_pressed("upshift"):
			attempt_upshift()
		elif input_manager.get_button_pressed("downshift"):
			attempt_downshift()
		
		# Handle advanced systems
		if input_manager.get_button_pressed("toggle_traction_control"):
			toggle_traction_control()
		if input_manager.get_button_pressed("toggle_abs"):
			toggle_abs()
		if input_manager.get_button_pressed("activate_boost"):
			force_full_boost()

func _process_throttle(value: float) -> void:
	"""Process throttle input (-1.0 to 1.0)"""
	is_throttling = value > 0.1
	
	throttle_applied.emit(value)
	
	# Apply throttle to engine torque
	if is_throttling and clutch_engaged:
		engine_rpm += value * delta * 500.0

func _process_brake(brake_value: float, handbrake: bool) -> void:
	"""Process brake and handbrake input"""
	is_braking = brake_value > 0.1
	is_handbraking = handbrake
	
	brake_applied.emit(brake_value)
	handbrake_toggled.emit(handbrake)
	
	if is_braking:
		_apply_brake_force(brake_value)
	
	if is_handbraking:
		_apply_handbrake_force()

func _process_steering(value: float) -> void:
	"""Process steering input (-1.0 to 1.0)"""
	steering_input = clamp(value, -1.0, 1.0)
	
	# Smooth steering transition
	var steering_target = steering_input * MAX_STEERING_ANGLE
	current_steering_angle = _smooth_damp(
		current_steering_angle,
		steering_target,
		0.0,
		MIN_STEERING_SPEED
	)
	
	steering_angle_changed.emit(current_steering_angle)

# ============================================================================
# PHYSICS UPDATE
# ============================================================================
func _physics_process(delta: float) -> void:
	"""Main physics update loop"""
	_time_since_last_update += delta
	
	# Update wheel positions and calculate forces
	_update_wheel_physics(delta)
	
	# Calculate drivetrain output
	_update_drivetrain(delta)
	
	# Apply forces to vehicle body
	_apply_vehicle_forces(delta)
	
	# Update speed and RPM readings
	_update_vehicle_readings(delta)
	
	# Check for special conditions
	_check_special_conditions(delta)
	
	# Move character body
	move_and_slide()

func _update_wheel_physics(delta: float) -> void:
	"""Calculate individual wheel forces and slip"""
	var wheel_positions = _get_wheel_world_positions()
	
	for wheel_key in wheel_positions.keys():
		var wheel_pos = wheel_positions[wheel_key]
		var contact_result = world_direct_ray_cast(wheel_pos, Vector3.UP, 0.1)
		
		if contact_result:
			_calculate_wheel_contact(wheel_key, contact_result)
		else:
			_reset_wheel_contact(wheel_key)

func _calculate_wheel_contact(wheel_key: String, contact: RayCastResult) -> void:
	"""Calculate contact forces for a single wheel"""
	pass

func _reset_wheel_contact(wheel_key: String) -> void:
	"""Reset wheel contact data"""
	wheel_forces[wheel_key] = Vector3.ZERO
	_tire_slip_data[wheel_key] = {
		"slip_ratio": 0.0,
		"slip_angle": 0.0,
		"vertical_force": 0.0
	}

func _update_drivetrain(delta: float) -> void:
	"""Update drivetrain and calculate wheel forces"""
	if not clutch_engaged and current_gear != 0:
		return
	
	# Calculate gear ratio
	var gear_ratio = GEAR_RATIOS[current_gear]
	var total_ratio = gear_ratio * FINAL_DRIVE_RATIO
	
	# Calculate wheel angular velocity from engine RPM
	var wheel_angular_velocity = (engine_rpm * 2 * PI / 60.0) / total_ratio
	
	# Apply torque to wheels based on gear
	var torque = _calculate_engine_torque()
	var wheel_torque = torque * gear_ratio * 0.95  # 5% drivetrain loss
	
	# Distribute torque to driven wheels
	var driven_wheels = _get_driven_wheels()
	for wheel in driven_wheels:
		_wheel_forces[wheel] = Vector3.FORWARD * (wheel_torque / 2.0)

func _calculate_engine_torque() -> float:
	"""Calculate current engine torque based on RPM"""
	if engine_rpm <= idle_rpm:
		return 0.0
	
	# Use torque curve if available, otherwise use simplified model
	if torque_curve.is_empty():
		# Simplified bell-curve torque model
		var normalized_rpm = engine_rpm / max_engine_rpm
		return 400.0 * sin(normalized_rpm * PI) * (1.0 + is_throttling * 0.5)
	
	# Lookup torque from curve
	var torque = torque_curve.get(engine_rpm, 0.0)
	return torque * (1.0 + is_throttling * 0.5)

func _apply_vehicle_forces(delta: float) -> void:
	"""Apply calculated forces to vehicle body"""
	var total_force = Vector3.ZERO
	var total_torque = Vector3.ZERO
	
	# Sum wheel forces
	for wheel_key in _wheel_forces.keys():
		total_force += _wheel_forces[wheel_key]
	
	# Apply gravity
	var gravity = PhysicsSettings.gravity * vehicle_mass
	total_force.y -= gravity
	
	# Apply air resistance
	var air_drag = _calculate_air_resistance()
	total_force -= Vector3.FORWARD * air_drag
	
	# Apply force to body center
	apply_central_force(total_force)
	
	# Apply rotational forces for steering
	if current_steering_angle != 0:
		total_torque.y = total_torque.y + current_steering_angle * 100.0
	
	angular_velocity = angular_velocity.lerp(Vector3.ZERO, 0.1)

func _calculate_air_resistance() -> float:
	"""Calculate aerodynamic drag force"""
	var air_density = 1.225  # kg/m^3 at sea level
	var drag_coefficient = 0.3  # Typical sports car Cd
	var frontal_area = 2.2  # m^2
	
	var speed_ms = current_speed / 3.6
	var drag_force = 0.5 * air_density * drag_coefficient * frontal_area * speed_ms * speed_ms
	
	return drag_force

func _apply_brake_force(brake_value: float) -> void:
	"""Apply braking force to all wheels"""
	if not abs_enabled:
		# Simple brake without ABS
		var brake_force = brake_value * 15000.0
		for wheel in _get_all_wheels():
			_wheel_forces[wheel] = _wheel_forces[wheel].moved_by(-Vector3.FORWARD * brake_force / 4.0)
		return
	
	# ABS-controlled braking
	var wheel_lock_threshold = ABS_LOCK_THRESHOLD
	for wheel in _get_all_wheels():
		var wheel_velocity = _get_wheel_velocity(wheel)
		var slip_ratio = _calculate_slip_ratio(wheel_velocity)
		
		if slip_ratio < wheel_lock_threshold:
			var brake_force = brake_value * 15000.0 * (1.0 - slip_ratio)
			_wheel_forces[wheel] = _wheel_forces[wheel].moved_by(-Vector3.FORWARD * brake_force / 4.0)

func _apply_handbrake_force() -> void:
	"""Apply handbrake force to rear wheels only"""
	var rear_wheels = _get_rear_wheels()
	var handbrake_force = 8000.0
	
	for wheel in rear_wheels:
		_wheel_forces[wheel] = _wheel_forces[wheel].moved_by(-Vector3.FORWARD * handbrake_force / 2.0)
		_wheel_forces[wheel] = _wheel_forces[wheel].moved_by(Vector3.RIGHT * handbrake_force / 2.0)

# ============================================================================
# VEHICLE READINGS
# ============================================================================
func _update_vehicle_readings(delta: float) -> void:
	"""Update vehicle speed, RPM, and other readings"""
	# Calculate speed from linear velocity
	var speed_mps = linear_velocity.length()
	current_speed = speed_mps * 3.6  # Convert to km/h
	
	# Update RPM based on throttle and gear
	if clutch_engaged and current_gear > 0:
		var gear_ratio = GEAR_RATIOS[current_gear]
		var theoretical_rpm = (current_speed / 3.6) * gear_ratio * FINAL_DRIVE_RATIO * 1.5
		engine_rpm = lerp(engine_rpm, theoretical_rpm, 0.1)
	else:
		engine_rpm = lerp(engine_rpm, idle_rpm, 0.05)
	
	# Clamp RPM
	engine_rpm = clamp(engine_rpm, idle_rpm, max_engine_rpm)
	
	# Emit signals
	speed_changed.emit(current_speed)
	rpm_changed.emit(engine_rpm)

func _check_special_conditions(delta: float) -> void:
	"""Check for drift, collisions, and other special events"""
	# Track drift
	if _is_skidding():
		_drift_accumulator += delta
		if _drift_accumulator > 0.5:
			_trigger_drift()
	else:
		_drift_accumulator = 0.0
	
	# Check collision damage
	if _has_recent_collision():
		_handle_collision_damage()
	
	# Update lap timing
	if GameManager.current_state == GameManager.RaceActive:
		current_lap_time += delta

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================
func switch_gear(new_gear: int) -> void:
	"""Switch to a specific gear"""
	if new_gear == current_gear:
		return
	
	var old_gear = current_gear
	current_gear = new_gear
	
	gear_changed.emit(old_gear, new_gear)
	
	# Disengage clutch during shift
	clutch_progress = 0.0
	clutch_engaged = false

func engage_clutch() -> void:
	"""Engage clutch smoothly"""
	clutch_engaged = true
	clutch_progress = 0.0

func _clutch_engagement_finished() -> void:
	"""Wait for clutch to fully engage"""
	while clutch_progress < 1.0:
		clutch_progress = min(clutch_progress + 1.0 / CLUTCH_DISPERCTION_TIME, 1.0)
		await get_tree().process_frame

func _smooth_damp(current: float, target: float, damping: float, max_speed: float) -> float:
	"""Smooth interpolation between values"""
	var diff = target - current
	return current + diff * damping * 0.5

# ============================================================================
# ADVANCED SYSTEMS
# ============================================================================
func toggle_traction_control() -> void:
	"""Toggle traction control system"""
	traction_control_enabled = not traction_control_enabled
	traction_control_state_changed.emit(traction_control_enabled)

func toggle_abs() -> void:
	"""Toggle anti-lock braking system"""
	abs_enabled = not abs_enabled
	anti_lock_braking_state_changed.emit(abs_enabled)

func _is_skidding() -> bool:
	"""Check if vehicle is skidding"""
	var wheel_velocities = _get_all_wheel_velocities()
	var average_velocity = _average_vector(wheel_velocities)
	var sideways_velocity = linear_velocity.projected_to_plane(linear_velocity, Vector3.UP).length()
	
	return sideways_velocity > average_velocity * DRIFT_THRESHOLD

func _trigger_drift() -> void:
	"""Trigger drift state"""
	if not drift_mode_enabled:
		return
	
	drift_angle = _calculate_drift_angle()
	drift_started.emit(drift_angle)
	drift_intensity = 1.0

func _calculate_drift_angle() -> float:
	"""Calculate current drift angle"""
	var velocity_direction = linear_velocity.normalized()
	var forward_direction = transform.basis.z
	var angle = velocity_direction.angle_to(forward_direction)
	return sign(angle) * min(abs(angle), PI / 2)

func _get_all_wheel_velocities() -> Array[Vector3]:
	"""Get velocities of all four wheels"""
	return [Vector3.ZERO, Vector3.ZERO, Vector3.ZERO, Vector3.ZERO]  # Placeholder

func _average_vector(vectors: Array[Vector3]) -> Vector3:
	"""Calculate average of vector array"""
	var sum = Vector3.ZERO
	for v in vectors:
		sum += v
	return sum / len(vectors) if len(vectors) > 0 else Vector3.ZERO

# ============================================================================
# HELPER METHODS
# ============================================================================
func _get_wheel_world_positions() -> Dictionary:
	"""Get world positions of all four wheels"""
	var offset_front = Vector3(0.0, 0.0, wheel_base / 2.0)
	var offset_rear = Vector3(0.0, 0.0, -wheel_base / 2.0)
	var offset_left = Vector3(track_width / 2.0, 0.0, 0.0)
	var offset_right = Vector3(-track_width / 2.0, 0.0, 0.0)
	
	return {
		"front_left": transform.origin + offset_front + offset_left,
		"front_right": transform.origin + offset_front + offset_right,
		"rear_left": transform.origin + offset_rear + offset_left,
		"rear_right": transform.origin + offset_rear + offset_right
	}

func _get_driven_wheels() -> Array[String]:
	"""Get list of driven wheel names"""
	return ["front_left", "front_right"]  # Default FWD, override per vehicle

func _get_rear_wheels() -> Array[String]:
	"""Get list of rear wheel names"""
	return ["rear_left", "rear_right"]

func _get_all_wheels() -> Array[String]:
	"""Get all wheel names"""
	return ["front_left", "front_right", "rear_left", "rear_right"]

func _get_wheel_velocity(wheel_key: String) -> float:
	"""Get velocity of specific wheel"""
	return 0.0  # Placeholder implementation

func _calculate_slip_ratio(wheel_velocity: float) -> float:
	"""Calculate wheel slip ratio"""
	if wheel_velocity <= 0:
		return 0.0
	return (current_speed / 3.6 - wheel_velocity) / wheel_velocity

func _has_recent_collision() -> bool:
	"""Check if recent collision occurred"""
	return (Time.get_ticks_msec() - last_collision_time) < 1000

func _handle_collision_damage() -> void:
	"""Handle collision damage effects"""
	collision_detected.emit({
		"timestamp": Time.get_ticks_msec(),
		"impact_force": collision_impact_force,
		"location": global_position
	})
	
	# Visual and audio feedback
	collision_impact_force = 0.0

func _emit_signal(signal_name: String, args: Variant) -> void:
	"""Emit signal safely"""
	if has_signal(signal_name):
		emit_signal(signal_name, args)

# ============================================================================
# RESET AND CLEANUP
# ============================================================================
func reset_vehicle() -> void:
	"""Reset vehicle to initial state"""
	current_gear = 0
	target_gear = 0
	clutch_engaged = true
	clutch_progress = 1.0
	
	is_throttling = false
	is_braking = false
	is_handbraking = false
	steering_input = 0.0
	current_steering_angle = 0.0
	
	linear_velocity = Vector3.ZERO
	angular_velocity = Vector3.ZERO
	
	boost_charge_level = 100.0
	boost_available = true
	_boost_accumulator = 0.0
	
	lap_count = 0
	current_lap_time = 0.0
	last_checkpoint_id = -1
	
	_clear_wheel_data()

func _clear_wheel_data() -> void:
	"""Clear all wheel-related data"""
	_wheel_forces.clear()
	_tire_slip_data.clear()
	wheel_rotation_angles.clear()

func _free() -> void:
	"""Cleanup when node is freed"""
	_clear_wheel_data()
	super._free()
</FILE>