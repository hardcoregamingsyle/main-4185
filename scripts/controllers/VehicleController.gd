extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulator
## Implements throttle, brake, steering inputs, wheel forces, and gear shifting logic
## Uses PhysicsSettings constants for all physical parameters
## Copyright 2026 Thalamus Racing Simulator Project

# ============================================================================
# SIGNALS - Vehicle state change notifications
# ============================================================================
signal gear_changed(old_gear: int, new_gear: int)
signal speed_changed(speed_kmh: float)
signal traction_lost()
signal traction_gained()
signal engine_rpm_changed(rpm: float)
signal damage_taken(damage_amount: float)
signal vehicle_stalled()
signal vehicle_revved(rev_limit_hit: bool)

# ============================================================================
# ENUMS - Vehicle states and modes
# ============================================================================
enum DrivingMode {
	AUTO,      # Automatic transmission
	MANUAL,    # Manual transmission with clutch
	RACE,      # Race mode with performance tuning
	DRIFT,     # Drift mode with modified grip
	CUSTOM     # Custom configuration
}

enum GearState {
	REVERSE = -1,
	IDLE = 0,
	FIRST = 1,
	SECOND = 2,
	THIRD = 3,
	FOURTH = 4,
	FIFTH = 5,
	SIXTH = 6,
	OVERDRIVE = 7
}

# ============================================================================
# EXPORTED CONFIGURATION - Runtime tweakable parameters
# ============================================================================
@export_group("Vehicle Configuration")
@export var vehicle_mass: float = 1500.0
@export var center_of_mass_offset: Vector3 = Vector3(0.0, 0.5, 0.0)
@export var drag_coefficient: float = 0.30
@export var frontal_area: float = 2.2  # m^2

@export_group("Wheel Configuration")
@export var track_width_front: float = 1.6  # meters
@export var track_width_rear: float = 1.6   # meters
@export var wheel_base: float = 2.8         # meters
@export var tire_radius: float = 0.33       # meters
@export var suspension_travel_max: float = 0.15  # meters

@export_group("Engine Configuration")
@export var engine_displacement: float = 3.5  # liters
@export var max_engine_rpm: float = 7500.0
@export var idle_rpm: float = 800.0
@export var redline_rpm: float = 7000.0
@export var torque_curve: Dictionary = {}

@export_group("Transmission Configuration")
@export var final_drive_ratio: float = 3.5
@export var gear_ratios: Array[float] = [3.8, 2.2, 1.5, 1.1, 0.9, 0.75]

@export_group("Braking System")
@export var brake_force_front: float = 4000.0   # Newtons per wheel
@export var brake_force_rear: float = 3500.0    # Newtons per wheel
@export var abs_enabled: bool = true
@export var brake_bias_front: float = 0.6       # 0-1, front bias

@export_group("Tire & Grip Model")
@export var grip_level: float = 1.0             # Multiplier (0-2)
@export var drift_mode_factor: float = 0.7      # Reduced lateral grip for drift
@export var slip_angle_threshold: float = 5.0   # degrees before significant grip loss

# ============================================================================
# PRIVATE STATE VARIABLES
# ============================================================================
var _current_gear: int = GearState.IDLE
var _target_gear: int = GearState.IDLE
var _clutch_engaged: bool = false
var _clutch_position: float = 1.0              # 0 = disengaged, 1 = fully engaged
var _engine_rpm: float = 0.0
var _vehicle_speed_kmh: float = 0.0
var _throttle_input: float = 0.0               # 0-1
var _brake_input: float = 0.0                  # 0-1
var _steering_input: float = 0.0               # -1 to 1
var _driving_mode: DrivingMode = DrivingMode.RACE

# Powertrain references
var _powertrain_node: Node = null
var _audio_manager: AudioManager = null

# Physics simulation state
var _wheel_forces: Dictionary = {}
var _lateral_slip_angles: Dictionary = {}
var _longitudinal_slip_ratios: Dictionary = {}
var _traction_loss_timer: float = 0.0
var _is_traction_lost: bool = false

# Game feel variables
var _suspension_compression: Dictionary = {}
var _body_roll_angle: float = 0.0
var _pitch_angle: float = 0.0
var _damage_level: float = 0.0
var _last_collision_time: float = 0.0

# Constants loaded from PhysicsSettings
var _physics_settings: PhysicsSettings = null
var _gravity: float = 9.81
var _max_substeps: int = 4
var _time_scale: float = 1.0

# Engine torque curve cache
var _torque_at_current_rpm: float = 0.0

# ============================================================================
# INITIALIZATION
# ============================================================================
func _ready() -> void:
	# Load physics settings
	_physics_settings = ResourceManager.get_singleton_or_load("PhysicsSettings", PhysicsSettings)
	if _physics_settings != null:
		_gravity = _physics_settings.gravity
		_max_substeps = _physics_settings.max_substeps
		_time_scale = _physics_settings.time_scale
	
	# Get audio manager singleton
	_audio_manager = GameManager.get_singleton() if is_instance_valid(GameManager) else null
	
	# Initialize wheel force dictionaries
	_wheel_forces = {
		"front_left": 0.0,
		"front_right": 0.0,
		"rear_left": 0.0,
		"rear_right": 0.0
	}
	
	_lateral_slip_angles = {
		"front_left": 0.0,
		"front_right": 0.0,
		"rear_left": 0.0,
		"rear_right": 0.0
	}
	
	_longitudinal_slip_ratios = {
		"front_left": 0.0,
		"front_right": 0.0,
		"rear_left": 0.0,
		"rear_right": 0.0
	}
	
	_suspension_compression = {
		"front_left": 0.0,
		"front_right": 0.0,
		"rear_left": 0.0,
		"rear_right": 0.0
	}
	
	# Set center of mass
	_center_of_mass = center_of_mass_offset
	
	# Calculate engine torque curve if not provided
	if torque_curve.is_empty():
		_generate_torque_curve()
	
	# Initial gear setup
	_current_gear = GearState.REVERSE
	_set_gear(GearState.FIRST)
	
	print("[VehicleController] Ready - %dkg vehicle initialized" % vehicle_mass)

func _generate_torque_curve() -> void:
	"""Generate a realistic V8-style torque curve based on engine displacement"""
	# Standard torque curve points (rpm, torque_percentage)
	var torque_points: Array[Vector2i] = [
		Vector2i(1000, 0.3),
		Vector2i(2000, 0.5),
		Vector2i(3000, 0.7),
		Vector2i(4000, 0.85),
		Vector2i(5000, 0.95),
		Vector2i(6000, 1.0),
		Vector2i(7000, 0.8),
		Vector2i(7500, 0.5)
	]
	
	for point in torque_points:
		torque_curve[str(point.x)] = point.y

# ============================================================================
# MAIN PROCESS LOOP
# ============================================================================
func _physics_process(delta: float) -> void:
	# Update time scale from physics settings
	_physics_settings = ResourceManager.get_singleton_or_load("PhysicsSettings", PhysicsSettings)
	if _physics_settings:
		_time_scale = _physics_settings.time_scale
	
	delta *= _time_scale
	
	# Process input
	_process_input(delta)
	
	# Update engine RPM
	_update_engine_rpm(delta)
	
	# Calculate power delivery
	_calculate_power_delivery(delta)
	
	# Apply forces to vehicle
	_apply_vehicle_forces(delta)
	
	# Handle suspension dynamics
	_update_suspension(delta)
	
	# Calculate aerodynamic drag
	_apply_aerodynamics(delta)
	
	# Update telemetry
	_update_telemetry(delta)
	
	# Apply movement
	move_and_slide()
	
	# Check for traction loss
	_check_traction_status(delta)

# ============================================================================
# INPUT PROCESSING
# ============================================================================
func _process_input(delta: float) -> void:
	"""Process player input and update control variables"""
	# Get input values from InputManager or direct input
	if is_instance_valid(GameManager.InputManager):
		_throttle_input = GameManager.InputManager.get_axis("throttle", 0.0)
		_brake_input = GameManager.InputManager.get_axis("brake", 0.0)
		_steering_input = GameManager.InputManager.get_axis("steering", 0.0)
	else:
		# Fallback to direct input
		_throttle_input = Input.get_action_strength("throttle")
		_brake_input = Input.get_action_strength("brake")
		_steering_input = Input.get_axis("steering_left", "steering_right")
	
	# Clamp inputs
	_throttle_input = clamp(_throttle_input, 0.0, 1.0)
	_brake_input = clamp(_brake_input, 0.0, 1.0)
	_steering_input = clamp(_steering_input, -1.0, 1.0)
	
	# Gear shifting inputs
	_handle_gear_shifting()
	
	# Clutch control (manual mode)
	if _driving_mode == DrivingMode.MANUAL:
		_handle_clutch(delta)

func _handle_gear_shifting() -> void:
	"""Handle automatic/manual gear shifting logic"""
	match _driving_mode:
		DrivingMode.AUTO:
			_auto_shift_gear()
		DrivingMode.MANUAL:
			_manual_shift_gear()

func _auto_shift_gear() -> void:
	"""Automatic transmission up/down shifts"""
	# Prevent rapid shifting
	if _current_gear == _target_gear:
		return
	
	# Upshift at appropriate RPM threshold
	if _throttle_input > 0.8 and _engine_rpm > redline_rpm * 0.95:
		_shift_up()
	elif _engine_rpm < idle_rpm * 1.2 and _current_gear > GearState.FIRST:
		_shift_down()
	
	# Downshift when braking hard
	if _brake_input > 0.7 and _current_gear > GearState.FIRST:
		if _engine_rpm < idle_rpm * 1.5:
			_shift_down()

func _manual_shift_gear() -> void:
	"""Manual transmission with clutch control"""
	if Input.is_action_just_pressed("gear_up"):
		if _clutch_position < 0.1:
			_shift_up()
	
	if Input.is_action_just_pressed("gear_down"):
		if _clutch_position < 0.1:
			_shift_down()

func _shift_up() -> void:
	"""Upshift one gear"""
	if _current_gear < GearState.SIXTH:
		var old_gear = _current_gear
		_current_gear += 1
		_target_gear = _current_gear
		
		emit_signal("gear_changed", old_gear, _current_gear)
		
		if _audio_manager:
			_audio_manager.play_sound("gear_up")

func _shift_down() -> void:
	"""Downshift one gear"""
	if _current_gear > GearState.FIRST:
		var old_gear = _current_gear
		_current_gear -= 1
		_target_gear = _current_gear
		
		emit_signal("gear_changed", old_gear, _current_gear)
		
		if _audio_manager:
			_audio_manager.play_sound("gear_down")

func _handle_clutch(delta: float) -> void:
	"""Clutch engagement/disengagement"""
	if Input.is_action_pressed("clutch"):
		_clutch_position = lerp(_clutch_position, 0.0, delta * 10.0)
	else:
		_clutch_position = lerp(_clutch_position, 1.0, delta * 10.0)
	
	_clutch_engaged = _clutch_position > 0.9

# ============================================================================
# ENGINE RPM MANAGEMENT
# ============================================================================
func _update_engine_rpm(delta: float) -> void:
	"""Update engine RPM based on gear ratio and vehicle speed"""
	if _current_gear <= GearState.IDLE:
		# Engine idling or in neutral
		_engine_rpm = lerp(_engine_rpm, idle_rpm, delta * 5.0)
		_torque_at_current_rpm = 0.0
		return
	
	# Calculate theoretical wheel RPM from vehicle speed
	var wheel_circumference = 2.0 * PI * tire_radius
	var wheel_rps = _vehicle_speed_kmh / 3.6 / wheel_circumference
	var engine_rpm_from_wheel = wheel_rps * gear_ratios[_current_gear - 1] * final_drive_ratio
	
	# Blend between current RPM and target based on clutch position
	var target_rpm = engine_rpm_from_wheel
	if _clutch_engaged:
		_engine_rpm = lerp(_engine_rpm, target_rpm, delta * 20.0)
	else:
		# Decouple engine from wheels
		_engine_rpm = lerp(_engine_rpm, target_rpm * 0.3, delta * 10.0)
	
	# Clamp RPM
	_engine_rpm = clamp(_engine_rpm, 0.0, max_engine_rpm)
	
	# Get torque at current RPM
	_torque_at_current_rpm = _get_torque_at_rpm(_engine_rpm)
	
	# Emit signal
	emit_signal("engine_rpm_changed", _engine_rpm)
	
	# Check for redline hit
	if _engine_rpm >= redline_rpm:
		emit_signal("vehicle_revved", true)

func _get_torque_at_rpm(rpm: float) -> float:
	"""Get torque percentage at given RPM from torque curve"""
	if rpm < 1000.0:
		return 0.0
	
	# Find surrounding points in torque curve
	var keys = torque_curve.keys()
	keys.sort()
	
	var lower_key = 1000
	var upper_key = max_engine_rpm
	
	for key in keys:
		var key_val = float(key)
		if key_val <= rpm:
			lower_key = key_val
		if key_val > rpm:
			upper_key = key_val
			break
	
	# Linear interpolation between points
	if lower_key == upper_key:
		return torque_curve[str(lower_key)]
	
	var lower_torque = torque_curve[str(lower_key)]
	var upper_torque = torque_curve[str(upper_key)]
	
	var ratio = (rpm - lower_key) / (upper_key - lower_key)
	return lower_torque + (upper_torque - lower_torque) * ratio

# ============================================================================
# POWER DELIVERY CALCULATION
# ============================================================================
func _calculate_power_delivery(delta: float) -> void:
	"""Calculate power delivered to wheels based on throttle and engine state"""
	if _current_gear <= GearState.IDLE:
		_wheel_forces["front_left"] = 0.0
		_wheel_forces["front_right"] = 0.0
		_wheel_forces["rear_left"] = 0.0
		_wheel_forces["rear_right"] = 0.0
		return
	
	# Calculate drivetrain losses
	var drivetrain_efficiency = 0.85 if _driving_mode == DrivingMode.AUTO else 0.88
	
	# Calculate torque at wheels
	var gear_ratio = gear_ratios[_current_gear - 1]
	var wheel_torque = _torque_at_current_rpm * (engine_displacement * 100.0) * gear_ratio * final_drive_ratio * drivetrain_efficiency
	
	# Distribute torque to wheels
	var total_drive_force = wheel_torque / tire_radius
	
	# Apply differential type (simplified LSD behavior)
	var rear_torque = total_drive_force * 0.6
	var front_torque = total_drive_force * 0.4
	
	# Front-wheel split
	var front_per_wheel = front_torque * _throttle_input / 2.0
	_wheel_forces["front_left"] = front_per_wheel
	_wheel_forces["front_right"] = front_per_wheel
	
	# Rear-wheel split with LSD effect
	var rear_per_wheel = rear_torque * _throttle_input / 2.0
	
	# LSD adjustment - transfer torque to wheel with more traction
	var traction_diff = _calculate_traction_difference()
	rear_per_wheel += traction_diff * 0.1
	
	_wheel_forces["rear_left"] = rear_per_wheel
	_wheel_forces["rear_right"] = rear_per_wheel

func _calculate_traction_difference() -> float:
	"""Calculate traction difference between rear wheels"""
	var left_slip = abs(_longitudinal_slip_ratios["rear_left"])
	var right_slip = abs(_longitudinal_slip_ratios["rear_right"])
	return left_slip - right_slip

# ============================================================================
# VEHICLE FORCES APPLICATION
# ============================================================================
func _apply_vehicle_forces(delta: float) -> void:
	"""Apply all calculated forces to the vehicle body"""
	# Forward/backward forces from wheels
	var forward_force = (_wheel_forces["front_left"] + _wheel_forces["front_right"] + 
	                    _wheel_forces["rear_left"] + _wheel_forces["rear_right"])
	
	# Apply force in local forward direction
	var forward_vector = global_transform.basis.z * -1.0
	add_force(forward_vector * forward_force * vehicle_mass * 0.01)
	
	# Braking forces
	_apply_brakes(delta)
	
	# Steering effects on lateral movement
	_apply_steering_effects(delta)

func _apply_brakes(delta: float) -> void:
	"""Apply braking force to all wheels"""
	if _brake_input <= 0.0:
		return
	
	# ABS check - reduce brake force if wheels are slipping
	var brake_multiplier = 1.0
	if abs_enabled and _is_traction_lost:
		brake_multiplier = 0.7
	
	# Calculate brake forces per wheel
	var total_brake_force = (brake_force_front * 2.0 + brake_force_rear * 2.0) * _brake_input * brake_multiplier
	
	# Apply brake bias
	var front_brake_total = total_brake_force * brake_bias_front
	var rear_brake_total = total_brake_force * (1.0 - brake_bias_front)
	
	var front_brake_per_wheel = front_brake_total / 2.0
	var rear_brake_per_wheel = rear_brake_total / 2.0
	
	# Apply braking as negative force
	var brake_vector = global_transform.basis.z * 1.0  # Opposite to forward
	
	add_force(brake_vector * front_brake_per_wheel * 0.01)
	add_force(brake_vector * rear_brake_per_wheel * 0.01)

func _apply_steering_effects(delta: float) -> void:
	"""Apply lateral steering forces based on steering input"""
	if abs(_steering_input) < 0.05:
		return
	
	# Steering sensitivity based on speed
	var speed_factor = 1.0
	if _vehicle_speed_kmh > 50.0:
		speed_factor = 1.0 - (_vehicle_speed_kmh - 50.0) / 200.0
	
	var effective_steering = _steering_input * speed_factor
	
	# Convert steering to lateral velocity component
	var lateral_velocity = effective_steering * _vehicle_speed_kmh * 0.1
	
	# Apply lateral force
	var lateral_vector = global_transform.basis.x * lateral_velocity * 0.5
	add_force(lateral_vector)

# ============================================================================
# SUSPENSION DYNAMICS
# ============================================================================
func _update_suspension(delta: float) -> void:
	"""Update suspension compression and body angles"""
	# Simplified suspension model - use collision detection results
	# In full implementation, this would use RayCast3D for each wheel
	
	# Body roll calculation based on lateral acceleration
	var lateral_acceleration = _calculate_lateral_acceleration()
	_body_roll_angle = lateral_acceleration * 0.05 * grip_level
	
	# Pitch calculation based on longitudinal acceleration
	var longitudinal_acceleration = _calculate_longitudinal_acceleration()
	_pitch_angle = longitudinal_acceleration * 0.03
	
	# Suspension compression per wheel (simulated)
	var base_compression = 0.05
	var load_transfer = abs(longitudinal_acceleration) * 0.02
	
	_suspension_compression["front_left"] = base_compression + load_transfer * 0.3
	_suspension_compression["front_right"] = base_compression + load_transfer * 0.3
	_suspension_compression["rear_left"] = base_compression + load_transfer * 0.5
	_suspension_compression["rear_right"] = base_compression + load_transfer * 0.5

func _calculate_lateral_acceleration() -> float:
	"""Calculate lateral acceleration in g-forces"""
	# Simplified calculation based on turning
	return _steering_input * (_vehicle_speed_kmh / 100.0)

func _calculate_longitudinal_acceleration() -> float:
	"""Calculate longitudinal acceleration in m/s²"""
	var total_force = (_wheel_forces["front_left"] + _wheel_forces["front_right"] + 
	                  _wheel_forces["rear_left"] + _wheel_forces["rear_right"])
	return total_force / vehicle_mass

# ============================================================================
# AERODYNAMICS
# ============================================================================
func _apply_aerodynamics(delta: float) -> void:
	"""Apply aerodynamic drag forces"""
	var air_density = 1.225  # kg/m³ at sea level
	
	# Air resistance formula: F = 0.5 * Cd * A * rho * v²
	var velocity_mps = _vehicle_speed_kmh / 3.6
	var drag_force = 0.5 * drag_coefficient * frontal_area * air_density * velocity_mps * velocity_mps
	
	# Apply drag opposite to velocity
	var drag_vector = -velocity_direction.normalized() * drag_force
	add_force(drag_vector * 0.01)

func get_velocity_direction() -> Vector3:
	"""Get normalized velocity direction"""
	if linear_velocity.length_squared() > 0.01:
		return linear_velocity.normalized()
	return Vector3.ZERO

# ============================================================================
# TRACTION & SLIP MANAGEMENT
# ============================================================================
func _check_traction_status(delta: float) -> void:
	"""Monitor and report traction loss conditions"""
	var max_slip = 0.0
	for wheel in _longitudinal_slip_ratios.keys():
		max_slip = max(max_slip, abs(_longitudinal_slip_ratios[wheel]))
	
	if max_slip > 0.3 and not _is_traction_lost:
		_is_traction_lost = true
		_traction_loss_timer = 0.0
		emit_signal("traction_lost")
		if _audio_manager:
			_audio_manager.play_sound("tire_squeal")
	elif max_slip < 0.1 and _is_traction_lost:
		_is_traction_lost = false
		emit_signal("traction_gained")

func _update_telemetry(delta: float) -> void:
	"""Update vehicle telemetry data"""
	# Calculate actual speed from velocity vector
	var speed_mps = linear_velocity.length()
	_vehicle_speed_kmh = speed_mps * 3.6
	
	emit_signal("speed_changed", _vehicle_speed_kmh)

# ============================================================================
# DAMAGE SYSTEM
# ============================================================================
func take_damage(damage_amount: float) -> void:
	"""Apply damage to vehicle"""
	_damage_level = min(_damage_level + damage_amount, 100.0)
	
	if _damage_level >= 50.0:
		# Performance degradation
		grip_level *= 0.8
	
	if _damage_level >= 100.0:
		emit_signal("vehicle_destroyed")
	
	emit_signal("damage_taken", damage_amount)
	
	if _audio_manager:
		_audio_manager.play_sound("impact")

func reset_damage() -> void:
	"""Reset vehicle damage"""
	_damage_level = 0.0
	grip_level = 1.0

# ============================================================================
# GEAR STATUS ACCESSORS
# ============================================================================
func get_current_gear() -> int:
	return _current_gear

func get_target_gear() -> int:
	return _target_gear

func is_in_gear() -> bool:
	return _current_gear > GearState.IDLE

func get_engine_rpm() -> float:
	return _engine_rpm

func get_vehicle_speed_kmh() -> float:
	return _vehicle_speed_kmh

func get_driving_mode() -> DrivingMode:
	return _driving_mode

func set_driving_mode(mode: DrivingMode) -> void:
	_driving_mode = mode

func toggle_driving_mode() -> void:
	match _driving_mode:
		DrivingMode.AUTO:
			_driving_mode = DrivingMode.MANUAL
		DrivingMode.MANUAL:
			_driving_mode = DrivingMode.AUTO
		_:
			pass

# ============================================================================
# HELPER METHODS
# ============================================================================
func get_wheel_positions() -> Dictionary:
	"""Get world positions of all four wheels"""
	var half_track_front = track_width_front / 2.0
	var half_track_rear = track_width_rear / 2.0
	var half_wheelbase = wheel_base / 2.0
	
	return {
		"front_left": transform.origin + Vector3(-half_track_front, 0, -half_wheelbase),
		"front_right": transform.origin + Vector3(half_track_front, 0, -half_wheelbase),
		"rear_left": transform.origin + Vector3(-half_track_rear, 0, half_wheelbase),
		"rear_right": transform.origin + Vector3(half_track_rear, 0, half_wheelbase)
	}

func set_wheel_visual_state(wheel_name: String, rotation: float, compression: float) -> void:
	"""Update wheel visual mesh transform"""
	var wheel_node = get_node_or_null("WheelVisuals/" + wheel_name)
	if wheel_node:
		wheel_node.rotation.y = rotation
		wheel_node.position.y = -compression

# ============================================================================
# SAVE/LOAD STATE
# ============================================================================
func save_state() -> Dictionary:
	"""Save vehicle state for replay/saving"""
	return {
		"position": global_position,
		"rotation": global_rotation,
		"velocity": linear_velocity,
		"angular_velocity": angular_velocity,
		"current_gear": _current_gear,
		"engine_rpm": _engine_rpm,
		"vehicle_speed_kmh": _vehicle_speed_kmh,
		"throttle_input": _throttle_input,
		"brake_input": _brake_input,
		"steering_input": _steering_input,
		"driving_mode": _driving_mode,
		"damage_level": _damage_level
	}

func load_state(state: Dictionary) -> void:
	"""Load vehicle state from saved data"""
	global_position = state.get("position", global_position)
	global_rotation = state.get("rotation", global_rotation)
	linear_velocity = state.get("velocity", linear_velocity)
	angular_velocity = state.get("angular_velocity", angular_velocity)
	_current_gear = state.get("current_gear", _current_gear)
	_engine_rpm = state.get("engine_rpm", _engine_rpm)
	_vehicle_speed_kmh = state.get("vehicle_speed_kmh", _vehicle_speed_kmh)
	_throttle_input = state.get("throttle_input", 0.0)
	_brake_input = state.get("brake_input", 0.0)
	_steering_input = state.get("steering_input", 0.0)
	_driving_mode = state.get("driving_mode", _driving_mode)
	_damage_level = state.get("damage_level", _damage_level)

# ============================================================================
# DEBUG VISUALIZATION
# ============================================================================
func _draw_debug_lines() -> void:
	"""Draw debug visualization lines for physics debugging"""
	if not GameManager.debug_mode:
		return
	
	# Draw wheel positions
	var wheels = get_wheel_positions()
	for name in wheels:
		debug_draw_line(wheels[name], wheels[name] + Vector3.UP * 0.5, Color.YELLOW)
	
	# Draw velocity vector
	debug_draw_line(global_position, global_position + linear_velocity * 0.5, Color.GREEN)
	
	# Draw engine RPM indicator
	debug_draw_text("RPM: %.0f | Speed: %.1f km/h | Gear: %d" % [_engine_rpm, _vehicle_speed_kmh, _current_gear], 
	               global_position + Vector3.UP * 2.0, Color.CYAN)

func debug_draw_line(from: Vector3, to: Vector3, color: Color) -> void:
	"""Debug helper for drawing lines"""
	push_line(from, to, color, 0.1)

func debug_draw_text(text: String, position: Vector3, color: Color) -> void:
	"""Debug helper for drawing text"""
	pass  # Would use CanvasLayer or 3D text in production

# ============================================================================
# CLEANUP
# ============================================================================
func _exit_tree() -> void:
	"""Cleanup on node removal"""
	pass

</FILE>