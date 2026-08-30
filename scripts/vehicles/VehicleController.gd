extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulation
## Handles throttle, brake, steering, wheel forces, gear shifting, and vehicle dynamics
## Uses centralized PhysicsSettings for all physics constants
## Copyright 2026 Thalamus Racing Simulator Project

# ============================================================================
# SIGNALS
# ============================================================================

signal speed_changed(new_speed: float)
signal rpm_changed(rpm: float)
signal gear_changed(old_gear: int, new_gear: int)
signal drift_started(drift_angle: float)
signal drift_ended()
signal collision_detected(collision_info: Dictionary)
signal lap_completed(lap_data: Dictionary)
signal race_lap_updated(current_lap: int, split_time: float)
signal engine_revving(rev_level: float)
signal traction_loss()
signal traction_gained()

# ============================================================================
# ENUMS
# ============================================================================

enum Gear {
	PARK = 0,
	REVERSE = -1,
	NEUTRAL = 0,
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

enum DriftMode {
	NONE,
	BEGINNING,
	MID_DRIFT,
	EXITING
}

enum DriveType {
	FWD,
	RWD,
	AWD
}

# ============================================================================
# EXPORTED VARIABLES (Tunable in Inspector)
# ============================================================================

@export_group("Vehicle Configuration")
@export var drive_type: DriveType = DriveType.RWD
@export var vehicle_mass: float = 1500.0
@export var center_of_mass_offset: Vector3 = Vector3(0.0, 0.5, 0.0)
@export var wheelbase: float = 2.5
@export var track_width: float = 1.6

@export_group("Engine & Transmission")
@export var max_engine_rpm: float = 8000.0
@export var idle_rpm: float = 800.0
@export var redline_rpm: float = 7500.0
@export var gear_ratios: Array[float] = [3.8, 2.2, 1.6, 1.3, 1.1, 0.9, 0.75, 0.65, 0.55, 0.5]
@export var final_drive_ratio: float = 3.5
@export var tire_radius: float = 0.33

@export_group("Power & Performance")
@export var max_power_torque: float = 500.0
@export var power_curve: Curve = null
@export var torque_curve: Curve = null
@export var drag_coefficient: float = 0.30
@export var frontal_area: float = 2.2

@export_group("Suspension & Tires")
@export var suspension_stiffness: float = 45000.0
@export var suspension_damping: float = 3500.0
@export var suspension_compression: float = 0.15
@export var suspension_extension: float = 0.20
@export var tire_friction: float = 1.2
@export var lateral_stiffness: float = 45000.0
@export var longitudinal_stiffness: float = 50000.0

@export_group("Drift Mechanics")
@export var drift_threshold: float = 0.35
@export var drift_recovery_rate: float = 0.08
@export var drift_momentum_factor: float = 0.6
@export var drift_bonus_traction: float = 0.25
@export var min_drift_angle: float = 15.0
@export var max_drift_angle: float = 45.0

@export_group("Braking System")
@export var brake_force_per_wheel: float = 15000.0
@export var abs_enabled: bool = true
@export var brake_bias_front: float = 0.60
@export var brake_bias_rear: float = 0.40

@export_group("Steering")
@export var max_steering_angle: float = 35.0
@export var steering_speed: float = 2.5
@export var steering_sensitivity: float = 1.0
@export var ackermann_multiplier: float = 0.8

# ============================================================================
# PRIVATE STATE VARIABLES
# ============================================================================

var _current_gear: int = Gear.NEUTRAL
var _engine_rpm: float = idle_rpm
var _vehicle_speed: float = 0.0
var _acceleration: Vector3 = Vector3.ZERO
var _angular_velocity: Vector3 = Vector3.ZERO

# Input values (normalized -1 to 1)
var _throttle_input: float = 0.0
var _brake_input: float = 0.0
var _steering_input: float = 0.0
var _handbrake_input: float = 0.0

# Wheel states
var _wheel_states: Array[Dictionary] = []
var _wheel_contacts: Array[ContactInfo] = []

# Drift state
var _drift_mode: DriftMode = DriftMode.NONE
var _drift_angle: float = 0.0
var _drift_momentum: float = 0.0

# Race/Lap data
var _lap_times: Array[float] = []
var _current_lap: int = 0
var _current_lap_start: float = 0.0
var _total_race_time: float = 0.0

# Audio references
var _powertrain_node: Node = null
var _audio_manager: Node = null

# Timing
var _last_update_time: float = 0.0
var _delta_accumulator: float = 0.0
const FIXED_TIME_STEP: float = 1.0 / 120.0

# ============================================================================
# CONTACT INFO STRUCTURE
# ============================================================================

class ContactInfo:
	var wheel_index: int
	var contact_point: Vector3
	var normal: Vector3
	var penetration: float
	var friction_force: Vector3
	var slip_ratio: float
	var slip_angle: float

# ============================================================================
# WHEEL DATA STRUCTURE
# ============================================================================

class WheelData:
	var position: Vector3
	var local_position: Vector3
	var suspension_length: float = 0.0
	var compression: float = 0.0
	var extension: float = 0.0
	var force: Vector3 = Vector3.ZERO
	var angular_velocity: float = 0.0
	var slip_ratio: float = 0.0
	var slip_angle: float = 0.0
	var grounded: bool = false
	var locked: bool = false

# ============================================================================
# PUBLIC PROPERTIES
# ============================================================================

func get_current_gear() -> int:
	return _current_gear

func get_engine_rpm() -> float:
	return _engine_rpm

func get_vehicle_speed() -> float:
	return _vehicle_speed

func get_absolute_speed() -> float:
	return _vehicle_speed

func get_acceleration() -> Vector3:
	return _acceleration

func get_drift_mode() -> DriftMode:
	return _drift_mode

func get_drift_angle() -> float:
	return _drift_angle

func get_throttle_input() -> float:
	return _throttle_input

func get_brake_input() -> float:
	return _brake_input

func get_steering_input() -> float:
	return _steering_input

func is_in_gear() -> bool:
	return _current_gear > Gear.NEUTRAL

func get_current_top_speed() -> float:
	return _calculate_top_speed(_current_gear)

func reset_laps() -> void:
	_lap_times.clear()
	_current_lap = 0
	_current_lap_start = 0.0

func add_lap_time(time: float) -> void:
	_lap_times.append(time)
	if _current_lap == 0:
		_current_lap = 1
	emit_signal("lap_completed", {"lap_number": _current_lap, "time": time})

func get_lap_count() -> int:
	return _lap_times.size()

func get_last_lap_time() -> float:
	if _lap_times.is_empty():
		return 0.0
	return _lap_times.back()

func get_best_lap_time() -> float:
	if _lap_times.is_empty():
		return 0.0
	var best_time: float = _lap_times[0]
	for t in _lap_times:
		if t < best_time:
			best_time = t
	return best_time

# ============================================================================
# INITIALIZATION
# ============================================================================

func _ready() -> void:
	_process_mode = ProcessModeEnum.ALWAYS
	_init_wheels()
	_connect_signals_to_singletons()
	_reset_physics_state()

func _init_wheels() -> void:
	"""Initialize wheel data structures for all four wheels"""
	var positions: Array[Vector3] = [
		Vector3(track_width * 0.5, 0.0, wheelbase * 0.5),    # Front Left
		Vector3(-track_width * 0.5, 0.0, wheelbase * 0.5),   # Front Right
		Vector3(track_width * 0.5, 0.0, -wheelbase * 0.5),   # Rear Left
		Vector3(-track_width * 0.5, 0.0, -wheelbase * 0.5)   # Rear Right
	]
	
	for i in positions.size():
		var wheel: WheelData = WheelData.new()
		wheel.position = positions[i]
		wheel.local_position = positions[i]
		_wheel_states.append(wheel)

func _connect_signals_to_singletons() -> void:
	_audio_manager = AudioManager
	# Powertrain connection would be established here if needed
	# _powertrain_node = find_child("Powertrain")

func _reset_physics_state() -> void:
	"""Reset all physics-related state variables"""
	_current_gear = Gear.PARK
	_engine_rpm = idle_rpm
	_vehicle_speed = 0.0
	_acceleration = Vector3.ZERO
	_drift_mode = DriftMode.NONE
	_drift_angle = 0.0
	_drift_momentum = 0.0
	_handbrake_input = 0.0
	
	for wheel in _wheel_states:
		wheel.compression = 0.0
		wheel.extension = 0.0
		wheel.force = Vector3.ZERO
		wheel.angular_velocity = 0.0
		wheel.slip_ratio = 0.0
		wheel.slip_angle = 0.0
		wheel.grounded = false
		wheel.locked = false

# ============================================================================
# MAIN UPDATE LOOP
# ============================================================================

func _physics_process(delta: float) -> void:
	"""Fixed timestep physics update for consistent vehicle behavior"""
	_delta_accumulator += delta
	
	while _delta_accumulator >= FIXED_TIME_STEP:
		_fixed_step_update(FIXED_TIME_STEP)
		_delta_accumulator -= FIXED_TIME_STEP
	
	_handle_variable_frame_updates(delta)

func _fixed_step_update(dt: float) -> void:
	"""Core physics calculations executed at fixed timestep"""
	_update_input_values(dt)
	_update_engine_and_transmission(dt)
	_update_wheel_dynamics(dt)
	_update_vehicle_motion(dt)
	_update_drift_state(dt)
	_apply_forces_to_body(dt)
	_check_collision_conditions()

func _handle_variable_frame_updates(delta: float) -> void:
	"""Variable frame rate updates (visuals, UI, smoothing)"""
	_smooth_steering(delta)
	_update_visual_components(delta)
	_emit_state_signals()

# ============================================================================
# INPUT HANDLING
# ============================================================================

func _update_input_values(dt: float) -> void:
	"""Read and process input values from InputManager singleton"""
	if GameManager.current_state != GameManager.GameState.RACE_ACTIVE:
		_throttle_input = 0.0
		_brake_input = 0.0
		_steering_input = 0.0
		_handbrake_input = 0.0
		return
	
	# Get normalized inputs (-1 to 1 range)
	_throttle_input = clamp(InputManager.get_axis("gas"), -1.0, 1.0)
	_brake_input = clamp(InputManager.get_axis("brake"), -1.0, 1.0)
	_steering_input = clamp(InputManager.get_axis("steer_left_right"), -1.0, 1.0)
	_handbrake_input = clamp(InputManager.get_axis("handbrake"), 0.0, 1.0)
	
	# Apply sensitivity modifiers
	_throttle_input *= Steering.sensitivity
	_brake_input *= Steering.sensitivity
	_steering_input *= Steering.sensitivity
	
	# Auto-shift logic (optional, can be toggled)
	if GameManager.auto_shift and is_in_gear():
		_auto_shift_gear()

# ============================================================================
# ENGINE & TRANSMISSION
# ============================================================================

func _update_engine_and_transmission(dt: float) -> void:
	"""Update engine RPM and transmission state"""
	var target_rpm: float = _calculate_target_rpm()
	_engine_rpm = lerp(_engine_rpm, target_rpm, dt * 10.0)
	
	# Clamp RPM within valid range
	_engine_rpm = clamp(_engine_rpm, idle_rpm, max_engine_rpm)
	
	# Handle gear changes
	_handle_gear_changes()
	
	# Update gear ratio based on current gear
	var current_ratio: float = _get_current_gear_ratio()
	
	# Calculate wheel angular velocity from engine RPM
	var wheel_rpm: float = _engine_rpm / (current_ratio * final_drive_ratio)
	var wheel_angular_velocity: float = wheel_rpm * PI / 30.0
	
	# Distribute angular velocity to wheels based on drive type
	_distribute_wheel_angular_velocity(wheel_angular_velocity)
	
	# Calculate vehicle speed from wheel rotation
	_vehicle_speed = wheel_angular_velocity * tire_radius * sign(_engine_rpm)
	
	# Emit signals for external systems
	emit_signal("rpm_changed", _engine_rpm)
	emit_signal("speed_changed", _vehicle_speed)

func _calculate_target_rpm() -> float:
	"""Calculate target engine RPM based on driving conditions"""
	if _current_gear == Gear.PARK or _current_gear == Gear.NEUTRAL:
		return idle_rpm
	
	var current_ratio: float = _get_current_gear_ratio()
	var drive_ratio: float = current_ratio * final_drive_ratio
	
	# Calculate desired wheel RPM based on throttle and speed
	var wheel_rpm: float = (_vehicle_speed / tire_radius) * 30.0 / PI
	
	# Target engine RPM
	var target_rpm: float = wheel_rpm * drive_ratio
	
	# Apply throttle influence
	if _throttle_input > 0.0:
		target_rpm *= (1.0 + _throttle_input * 0.5)
	elif _throttle_input < 0.0:
		target_rpm *= (1.0 + _throttle_input * 0.3)
	
	return target_rpm

func _get_current_gear_ratio() -> float:
	"""Get the gear ratio for the current gear"""
	if _current_gear <= Gear.NEUTRAL:
		return 0.0
	if _current_gear > gear_ratios.size():
		return gear_ratios.back()
	return gear_ratios[_current_gear - 1]

func _distribute_wheel_angular_velocity(wheel_angular_vel: float) -> void:
	"""Distribute wheel angular velocity based on drive type"""
	match drive_type:
		DriveType.FWD:
			_wheel_states[0].angular_velocity = wheel_angular_vel
			_wheel_states[1].angular_velocity = wheel_angular_vel
		DriveType.RWD:
			_wheel_states[2].angular_velocity = wheel_angular_vel
			_wheel_states[3].angular_velocity = wheel_angular_vel
		DriveType.AWD:
			for wheel in _wheel_states:
				wheel.angular_velocity = wheel_angular_vel

func _handle_gear_changes() -> void:
	"""Handle automatic or manual gear shifting"""
	if _current_gear == Gear.PARK:
		return
	
	var current_ratio: float = _get_current_gear_ratio()
	var current_top_speed: float = _calculate_top_speed(_current_gear)
	
	# Check if we should upshift
	if _throttle_input > 0.1 and _engine_rpm > redline_rpm * 0.95:
		_shift_up()
	
	# Check if we should downshift
	elif _throttle_input < 0.1 and _engine_rpm < idle_rpm * 1.5:
		_shift_down()
	
	# Emergency downshift on heavy braking
	elif _brake_input > 0.5 and _engine_rpm < idle_rpm * 1.2:
		_shift_down()

func _shift_up() -> void:
	"""Shift to next higher gear"""
	if _current_gear >= Gear.TENTH:
		return
	
	var old_gear: int = _current_gear
	_current_gear += 1
	
	# Drop RPM during shift
	_engine_rpm = lerp(_engine_rpm, _engine_rpm * 0.7, 0.3)
	
	emit_signal("gear_changed", old_gear, _current_gear)
	if _audio_manager:
		_audio_manager.play_sound("gear_change")

func _shift_down() -> void:
	"""Shift to next lower gear"""
	if _current_gear <= Gear.NEUTRAL:
		return
	
	var old_gear: int = _current_gear
	_current_gear -= 1
	
	# Rev match during downshift
	_engine_rpm = lerp(_engine_rpm, _engine_rpm * 1.3, 0.3)
	
	emit_signal("gear_changed", old_gear, _current_gear)
	if _audio_manager:
		_audio_manager.play_sound("gear_change")

func _auto_shift_gear() -> void:
	"""Automatic transmission logic"""
	var rpm_range: float = max_engine_rpm - idle_rpm
	var rpm_percent: float = (_engine_rpm - idle_rpm) / rpm_range
	
	if rpm_percent > 0.85 and _current_gear < Gear.TENTH:
		_shift_up()
	elif rpm_percent < 0.2 and _current_gear > Gear.FIRST:
		_shift_down()

# ============================================================================
# WHEEL DYNAMICS
# ============================================================================

func _update_wheel_dynamics(dt: float) -> void:
	"""Calculate forces and states for each wheel"""
	var gravity: Vector3 = PhysicsSettings.gravity * Vector3.DOWN
	
	for i in _wheel_states.size():
		var wheel: WheelData = _wheel_states[i]
		_update_wheel_contact(i, wheel, gravity)
		_update_wheel_forces(i, wheel)
		_update_wheel_slip(i, wheel)

func _update_wheel_contact(index: int, wheel: WheelData, gravity: Vector3) -> void:
	"""Calculate wheel contact point and suspension state"""
	var wheel_world_pos: Vector3 = global_transform * wheel.position
	
	# Raycast down for ground detection
	var ray_from: Vector3 = wheel_world_pos
	var ray_to: Vector3 = wheel_world_pos + Vector3.UP * 2.0
	var space_state: CollisionShape3D = get_world_3d().direct_space_state
	var query: Dictionary = {
		"from": ray_from,
		"to": ray_to,
		"collision_mask": 1,
		"include_areas": false
	}
	
	var result: Dictionary = space_state.ray_query(query)
	
	if result.has("collider"):
		wheel.grounded = true
		wheel.contact_point = result["intersection"]
		wheel.normal = result["normal"]
		
		# Calculate suspension compression
		var ideal_height: float = tire_radius + suspension_compression
		var current_height: float = wheel_world_pos.y - wheel.contact_point.y
		
		if current_height < ideal_height:
			wheel.compression = (ideal_height - current_height) / suspension_compression
		else:
			wheel.compression = 0.0
		
		# Calculate suspension extension
		if wheel_world_pos.y > wheel.contact_point.y + suspension_extension:
			wheel.extension = 1.0
		else:
			wheel.extension = 0.0
	else:
		wheel.grounded = false
		wheel.compression = 0.0
		wheel.extension = 1.0

func _update_wheel_forces(index: int, wheel: WheelData) -> void:
	"""Calculate suspension and tire forces for wheel"""
	if not wheel.grounded:
		wheel.force = Vector3.ZERO
		return
	
	var gravity: Vector3 = PhysicsSettings.gravity * Vector3.DOWN
	var suspension_force: Vector3 = Vector3.ZERO
	
	# Suspension force (spring + damper)
	var spring_force: float = -suspension_stiffness * wheel.compression
	var damper_force: float = -suspension_damping * wheel.compression
	
	suspension_force.y = spring_force + damper_force
	
	# Weight distribution per wheel
	var weight_per_wheel: float = vehicle_mass * 9.81 / 4.0
	suspension_force.y += weight_per_wheel
	
	wheel.force = suspension_force.normalized() * suspension_force.length()

func _update_wheel_slip(index: int, wheel: WheelData) -> void:
	"""Calculate slip ratio and slip angle for wheel"""
	if not wheel.grounded:
		wheel.slip_ratio = 0.0
		wheel.slip_angle = 0.0
		return
	
	var velocity_at_wheel: Vector3 = linear_velocity + angular_velocity.cross(wheel.position)
	var forward_vector: Vector3 = transform.basis.z
	
	# Slip ratio (longitudinal slip)
	var wheel_linear_velocity: float = wheel.angular_velocity * tire_radius
	var ground_velocity: float = velocity_at_wheel.dot(forward_vector)
	
	if ground_velocity != 0.0:
		wheel.slip_ratio = (wheel_linear_velocity - ground_velocity) / abs(ground_velocity)
	else:
		wheel.slip_ratio = 0.0
	
	# Slip angle (lateral slip)
	var lateral_velocity: Vector3 = velocity_at_wheel - forward_vector * ground_velocity
	var lateral_speed: float = lateral_velocity.length()
	if lateral_speed > 0.001:
		wheel.slip_angle = atan2(lateral_speed, ground_velocity)
	else:
		wheel.slip_angle = 0.0

# ============================================================================
# VEHICLE MOTION
# ============================================================================

func _update_vehicle_motion(dt: float) -> void:
	"""Calculate vehicle acceleration and apply forces"""
	var total_force: Vector3 = _calculate_total_force()
	var torque: Vector3 = _calculate_steering_torque()
	
	# Apply forces
	apply_central_impulse(total_force * dt)
	apply_torque(torque * dt)
	
	# Store acceleration for display/debug
	_acceleration = total_force / vehicle_mass

func _calculate_total_force() -> Vector3:
	"""Calculate total forces acting on the vehicle"""
	var forward_vector: Vector3 = transform.basis.z
	var right_vector: Vector3 = transform.basis.x
	var total_force: Vector3 = Vector3.ZERO
	
	# Engine/traction force
	var traction_force: Vector3 = _calculate_traction_force()
	total_force += traction_force
	
	# Brake force
	var brake_force: Vector3 = _calculate_brake_force()
	total_force += brake_force
	
	# Aerodynamic drag
	var drag_force: Vector3 = _calculate_drag_force()
	total_force += drag_force
	
	# Rolling resistance
	var rolling_resistance: Vector3 = _calculate_rolling_resistance()
	total_force += rolling_resistance
	
	# Gravity component on slopes
	var gravity_component: Vector3 = PhysicsSettings.gravity * vehicle_mass * Vector3.DOWN
	var slope_normal: Vector3 = _estimate_slope_normal()
	gravity_component -= slope_normal * (gravity_component.dot(slope_normal))
	total_force += gravity_component
	
	return total_force

func _calculate_traction_force() -> Vector3:
	"""Calculate traction/propulsion force from engine"""
	if _current_gear == Gear.PARK or _current_gear == Gear.NEUTRAL:
		return Vector3.ZERO
	
	var current_ratio: float = _get_current_gear_ratio()
	var drive_ratio: float = current_ratio * final_drive_ratio
	var wheel_rpm: float = _engine_rpm / drive_ratio
	
	# Calculate torque at wheels
	var wheel_torque: float = _calculate_wheel_torque()
	
	# Convert to linear force
	var traction_force: float = wheel_torque / tire_radius
	
	# Apply drive type multiplier
	var drive_multiplier: float = 1.0
	match drive_type:
		DriveType.FWD:
			drive_multiplier = 0.9
		DriveType.RWD:
			drive_multiplier = 1.0
		DriveType.AWD:
			drive_multiplier = 1.1
	
	traction_force *= drive_multiplier
	
	# Direction based on gear and throttle
	var direction: float = 1.0
	if _current_gear == Gear.REVERSE:
		direction = -1.0
	
	return forward_vector * traction_force * direction * _throttle_input

func _calculate_wheel_torque() -> float:
	"""Calculate torque output from engine at current RPM"""
	var engine_torque: float = max_power_torque
	
	if torque_curve != null:
		engine_torque = torque_curve.sample_clamped(_engine_rpm / max_engine_rpm) * max_power_torque
	
	return engine_torque

func _calculate_brake_force() -> Vector3:
	"""Calculate braking force applied to wheels"""
	if _brake_input <= 0.0 and _handbrake_input <= 0.0:
		return Vector3.ZERO
	
	var total_brake_force: float = brake_force_per_wheel * _brake_input
	
	# Handbrake adds additional rear brake bias
	if _handbrake_input > 0.0:
		total_brake_force *= (1.0 + _handbrake_input * 0.5)
		total_brake_force *= 0.4  # Handbrake only affects rear
	
	# Apply ABS logic
	if abs and _brake_input > 0.5 and _vehicle_speed > 1.0:
		total_brake_force *= 0.7  # ABS reduces max braking slightly
	
	# Split between front/rear based on brake bias
	var front_brake: float = total_brake_force * brake_bias_front
	var rear_brake: float = total_brake_force * brake_bias_rear
	
	# Apply opposite to movement direction
	var direction: float = -sign(_vehicle_speed)
	
	return forward_vector * (front_brake + rear_brake) * direction

func _calculate_drag_force() -> Vector3:
	"""Calculate aerodynamic drag force"""
	var air_density: float = 1.225
	var drag: float = 0.5 * air_density * drag_coefficient * frontal_area * _vehicle_speed * _vehicle_speed
	
	return -forward_vector * drag * sign(_vehicle_speed)

func _calculate_rolling_resistance() -> Vector3:
	"""Calculate rolling resistance from tires"""
	var rolling_resistance_coeff: float = 0.015
	var rolling_resistance: float = rolling_resistance_coeff * vehicle_mass * PhysicsSettings.gravity
	
	return -forward_vector * rolling_resistance * sign(_vehicle_speed)

func _estimate_slope_normal() -> Vector3:
	"""Estimate ground normal for gravity calculation"""
	var gravity_ray: RayCast3D = RayCast3D.new()
	gravity_ray.position = global_position
	gravity_ray.target_position = global_position + Vector3.DOWN * 2.0
	add_child(gravity_ray)
	
	gravity_ray.force_render()
	var normal: Vector3 = Vector3.UP
	
	if gravity_ray.is_colliding():
		normal = gravity_ray.get_collision_normal()
	
	remove_child(gravity_ray)
	gravity_ray.queue_free()
	
	return normal

func _calculate_steering_torque() -> Vector3:
	"""Calculate steering torque for vehicle rotation"""
	if _steering_input == 0.0:
		return Vector3.ZERO
	
	var steering_angle: float = deg_to_rad(max_steering_angle * abs(_steering_input))
	var torque_strength: float = _steering_input * 5000.0 * steering_speed
	
	return Vector3.UP * torque_strength

# ============================================================================
# DRIFT MECHANICS
# ============================================================================

func _update_drift_state(dt: float) -> void:
	"""Update drift mode and momentum"""
	var lateral_slip: float = _calculate_lateral_slip()
	
	if handbrake_input > 0.5 and abs(lateral_slip) > drift_threshold:
		if _drift_mode == DriftMode.NONE:
			_drift_mode = DriftMode.BEGINNING
			emit_signal("drift_started", 0.0)
			if _audio_manager:
				_audio_manager.play_sound("drift_start")
		elif _drift_mode == DriftMode.BEGINNING:
			_drift_mode = DriftMode.MID_DRIFT
		_drift_momentum = clamp(_drift_momentum + dt * 0.1, 0.0, 1.0)
	else:
		if _drift_mode != DriftMode.NONE:
			_drift_mode = DriftMode.EXITING
			_drift_momentum = max(0.0, _drift_momentum - dt * drift_recovery_rate)
			
			if _drift_momentum <= 0.1:
				_drift_mode = DriftMode.NONE
				emit_signal("drift_ended")
				if _audio_manager:
					_audio_manager.play_sound("drift_end")

func _calculate_lateral_slip() -> float:
	"""Calculate lateral slip angle for drift detection"""
	var velocity: Vector3 = linear_velocity
	var forward_vector: Vector3 = transform.basis.z
	var right_vector: Vector3 = transform.basis.x
	
	var forward_speed: float = velocity.dot(forward_vector)
	var lateral_speed: float = velocity.dot(right_vector)
	
	if abs(forward_speed) < 0.1:
		return 0.0
	
	return atan2(lateral_speed, forward_speed) * rad_to_deg(1.0)

func _apply_drift_effects() -> void:
	"""Apply special effects when drifting"""
	if _drift_mode == DriftMode.NONE:
		return
	
	# Increase lateral drift tendency
	var drift_factor: float = _drift_momentum * drift_bonus_traction
	
	# Reduce traction during drift
	var traction_reduction: float = 1.0 - (drift_factor * 0.5)
	
	# Visual effect trigger (particles, sound)
	if _drift_mode == DriftMode.MID_DRIFT and randf() < 0.05:
		if _audio_manager:
			_audio_manager.play_sound("drift_loop")

# ============================================================================
# FORCE APPLICATION
# ============================================================================

func _apply_forces_to_body(dt: float) -> void:
	"""Apply calculated forces to the rigid body"""
	var total_force: Vector3 = Vector3.ZERO
	
	for wheel in _wheel_states:
		if wheel.grounded:
			total_force += wheel.force
	
	# Apply central impulse
	apply_central_impulse(total_force * dt)
	
	# Apply damping for stability
	var damping: float = 0.99
	linear_velocity *= damping
	angular_velocity *= damping

# ============================================================================
# COLLISION HANDLING
# ============================================================================

func _check_collision_conditions() -> void:
	"""Check for collision events and emit signals"""
	pass  # Collision detected via signal from collision shapes

func _on_collision_entered(body: Node) -> void:
	"""Handle collision with other objects"""
	var collision_info: Dictionary = {
		"body": body.name if body else "unknown",
		"velocity": linear_velocity.length(),
		"timestamp": Time.get_ticks_msec()
	}
	
	emit_signal("collision_detected", collision_info)
	
	if _audio_manager:
		_audio_manager.play_sound("collision")
	
	# Screen shake effect
	if GameManager.debug_mode:
		GameManager.shake_camera(abs(linear_velocity).length() * 0.1)

# ============================================================================
# STEERING SMOOTHING
# ============================================================================

func _smooth_steering(delta: float) -> void:
	"""Smooth steering input transitions"""
	var target_steering: float = _steering_input * max_steering_angle
	
	# Interpolate current steering toward target
	var current_rotation: float = _get_current_rotation_angle()
	current_rotation = lerp(current_rotation, target_steering, delta * steering_speed)
	
	# Apply rotation to front wheels
	_set_wheel_rotation(0, current_rotation)
	_set_wheel_rotation(1, current_rotation * ackermann_multiplier)

func _set_wheel_rotation(wheel_index: int, angle: float) -> void:
	"""Set rotation angle for a specific wheel"""
	var wheel_node: Node = find_child("Wheel_" + str(wheel_index))
	if wheel_node:
		wheel_node.rotation.y = deg_to_rad(angle)

func _get_current_rotation_angle() -> float:
	"""Get current steering rotation angle"""
	return 0.0  # Placeholder - would read from actual wheel nodes

# ============================================================================
# VISUAL COMPONENTS
# ============================================================================

func _update_visual_components(delta: float) -> void:
	"""Update visual aspects like particles, lights, etc."""
	_update_headlights(delta)
	_update_exhaust_particles(delta)
	_update_skid_marks(delta)

func _update_headlights(delta: float) -> void:
	"""Update headlight intensity based on conditions"""
	pass  # Would connect to light nodes

func _update_exhaust_particles(delta: float) -> void:
	"""Update exhaust particle system"""
	if _engine_rpm > idle_rpm * 1.5 and randf() < delta * 5.0:
		pass  # Trigger exhaust particle emission

func _update_skid_marks(delta: float) -> void:
	"""Create skid mark decals when wheels slip heavily"""
	for wheel in _wheel_states:
		if wheel.grounded and abs(wheel.slip_ratio) > 0.5:
			pass  # Create skid mark at contact point

# ============================================================================
# TOP SPEED CALCULATION
# ============================================================================

func _calculate_top_speed(gear: int) -> float:
	"""Calculate theoretical top speed for a given gear"""
	if gear <= Gear.NEUTRAL:
		return 0.0
	
	var gear_ratio: float = gear_ratios[gear - 1] if gear <= gear_ratios.size() else gear_ratios.back()
	var drive_ratio: float = gear_ratio * final_drive_ratio
	
	# Top speed formula: RPM * tire_circumference / gear_ratio
	var wheel_circumference: float = 2.0 * PI * tire_radius
	var top_rpm: float = redline_rpm
	
	var top_speed: float = (top_rpm / drive_ratio) * wheel_circumference / 60.0  # m/s
	
	# Convert to km/h
	top_speed *= 3.6
	
	# Apply aerodynamic limit
	var aero_limit: float = sqrt((max_power_torque * 2) / (drag_coefficient * frontal_area * 1.225)) * 3.6
	
	return min(top_speed, aero_limit)

# ============================================================================
# RACE/LAP MANAGEMENT
# ============================================================================

func start_lap_timing() -> void:
	"""Start timing a new lap"""
	_current_lap_start = Time.get_unix_time_from_system()
	_current_lap = 1

func record_lap() -> void:
	"""Record completion of a lap"""
	var lap_time: float = Time.get_unix_time_from_system() - _current_lap_start
	add_lap_time(lap_time)

func get_race_time() -> float:
	"""Get total elapsed race time"""
	return _total_race_time

func update_race_time(delta: float) -> void:
	"""Update total race timer"""
	_total_race_time += delta

# ============================================================================
# GEAR SHIFT HELPERS
# ============================================================================

func set_gear_directly(new_gear: int) -> void:
	"""Manually set gear (for testing or AI control)"""
	if new_gear < Gear.REVERSE or new_gear > Gear.TENTH:
		return
	
	var old_gear: int = _current_gear
	_current_gear = new_gear
	
	if old_gear != new_gear:
		emit_signal("gear_changed", old_gear, new_gear)

func park() -> void:
	"""Put vehicle in park"""
	set_gear_directly(Gear.PARK)

func neutral() -> void:
	"""Put vehicle in neutral"""
	set_gear_directly(Gear.NEUTRAL)

func reverse() -> void:
	"""Put vehicle in reverse"""
	set_gear_directly(Gear.REVERSE)

# ============================================================================
# DEBUG/INFO FUNCTIONS
# ============================================================================

func print_debug_info() -> void:
	"""Print current vehicle state for debugging"""
	printi("=== Vehicle Controller Debug ===")
	printi("Gear: %s (%d)" % [_get_gear_name(_current_gear), _current_gear])
	printi("RPM: %.0f / %.0f" % [_engine_rpm, max_engine_rpm])
	printi("Speed: %.2f km/h" % (_vehicle_speed * 3.6))
	printi("Throttle: %.2f | Brake: %.2f | Steering: %.2f" % [_throttle_input, _brake_input, _steering_input])
	printi("Drift Mode: %s | Angle: %.1f°" % [_get_drift_mode_name(_drift_mode), _drift_angle])
	printi("Laps: %d | Best: %.2fs" % [_lap_times.size(), get_best_lap_time()])
	printi("===============================")

func _get_gear_name(gear: int) -> String:
	"""Convert gear enum value to readable name"""
	match gear:
		Gear.PARK: return "PARK"
		Gear.REVERSE: return "REVERSE"
		Gear.NEUTRAL: return "NEUTRAL"
		Gear.FIRST: return "1st"
		Gear.SECOND: return "2nd"
		Gear.THIRD: return "3rd"
		Gear.FOURTH: return "4th"
		Gear.FIFTH: return "5th"
		Gear.SIXTH: return "6th"
		Gear.SEVENTH: return "7th"
		Gear.EIGHTH: return "8th"
		Gear.NINTH: return "9th"
		Gear.TENTH: return "10th"
	_: return "UNKNOWN"

func _get_drift_mode_name(mode: DriftMode) -> String:
	"""Convert drift mode enum to readable name"""
	match mode:
		DriftMode.NONE: return "NONE"
		DriftMode.BEGINNING: return "BEGINNING"
		DriftMode.MID_DRIFT: return "MID_DRIFT"
		DriftMode.EXITING: return "EXITING"
	_: return "UNKNOWN"

# ============================================================================
# SAVE/LOAD SUPPORT
# ============================================================================

func save_vehicle_state() -> Dictionary:
	"""Save current vehicle state for replay/saving"""
	return {
		"position": global_position,
		"rotation": global_rotation,
		"velocity": linear_velocity,
		"angular_velocity": angular_velocity,
		"gear": _current_gear,
		"rpm": _engine_rpm,
		"speed": _vehicle_speed,
		"throttle": _throttle_input,
		"brake": _brake_input,
		"steering": _steering_input,
		"lap_count": _lap_times.size(),
		"best_lap": get_best_lap_time()
	}

func load_vehicle_state(state: Dictionary) -> void:
	"""Load vehicle state from saved data"""
	global_position = state["position"]
	global_rotation = state["rotation"]
	linear_velocity = state["velocity"]
	angular_velocity = state["angular_velocity"]
	_current_gear = state["gear"]
	_engine_rpm = state["rpm"]
	_vehicle_speed = state["speed"]
	_throttle_input = state["throttle"]
	_brake_input = state["brake"]
	_steering_input = state["steering"]
	_lap_times.resize(state["lap_count"])
	if state.has("best_lap"):
		pass  # Could restore best lap

# ============================================================================
# TEST/HARDCODE CONTROL
# ============================================================================

func test_input(throttle: float, brake: float, steering: float, handbrake: float) -> void:
	"""Test input values directly (for debugging/AI)"""
	_throttle_input = clamp(throttle, -1.0, 1.0)
	_brake_input = clamp(brake, -1.0, 1.0)
	_steering_input = clamp(steering, -1.0, 1.0)
	_handbrake_input = clamp(handbrake, 0.0, 1.0)

func set_engine_force(force: float) -> void:
	"""Override engine output (testing purposes)"""
	max_power_torque = force

func set_max_speed(speed: float) -> void:
	"""Set maximum allowed speed (testing purposes)"""
	pass  # Would modify drag or other limiting factors

</FILE>