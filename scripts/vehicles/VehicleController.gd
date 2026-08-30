extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulation
## Handles throttle, brake, steering, wheel forces, gear shifting, and vehicle dynamics
## Integrates with PhysicsSettings for consistent physics behavior
## Copyright 2026 Thalamus Racing Simulator Project

# Signals for external communication
signal speed_changed(new_speed: float)
signal rpm_changed(new_rpm: float)
signal gear_changed(old_gear: int, new_gear: int)
signal clutch_engaged(engaged: bool)
signal handbrake_toggled(engaged: bool)
signal collision_detected(collision_info: Dictionary)
signal drift_state_changed(is_drifting: bool)

# Powertrain reference
@onready var powertrain: Node = $Powertrain if has_node("Powertrain") else null

# Input states (normalized -1 to 1)
var throttle_input: float = 0.0
var brake_input: float = 0.0
var steering_input: float = 0.0
var clutch_input: float = 0.0
var handbrake_input: float = 0.0

# Vehicle state variables
var current_speed: float = 0.0       # Meters per second
var current_rpm: float = 0.0         # Revolutions per minute
var current_gear: int = 0            # Gear number (-1 = reverse, 0 = neutral, 1+ = forward)
var max_gears: int = 6               # Maximum forward gears
var is_in_reverse: bool = false
var is_neutral: bool = true
var is_clutch_engaged: bool = false
var is_handbrake_on: bool = false
var is_drifting: bool = false
var drift_angle: float = 0.0         # Drift angle in radians
var traction_factor: float = 1.0     # Current traction multiplier

# Wheel configuration
const WHEEL_BASE_WIDTH: float = 1.4      # Distance between left/right wheels
const WHEEL_BASE_LENGTH: float = 2.5     # Distance between front/rear wheels
const MAX_STEERING_ANGLE: float = 0.5    # Maximum steering angle in radians (~29 degrees)
const DRIFT_THRESHOLD_SPEED: float = 8.0 # Minimum speed to initiate drift
const DRIFT_THRESHOLD_STEER: float = 0.3 # Steering threshold for drift

# Physics constants from settings
var _physics_settings: PhysicsSettings = GameManager.physics_settings
var vehicle_mass: float = _physics_settings.default_vehicle_mass
var drag_coefficient: float = 0.3        # Aerodynamic drag coefficient
var rolling_resistance: float = 0.015    # Rolling resistance factor
var braking_force: float = 15000.0       # Maximum braking force Newtons
var cornering_stiffness: float = 80000.0 # Tire cornering stiffness

# Wheel force application points (relative to center)
var _front_left_wheel_pos: Vector3
var _front_right_wheel_pos: Vector3
var _rear_left_wheel_pos: Vector3
var _rear_right_wheel_pos: Vector3

# Engine torque curve (indexed by RPM percentage 0-1)
var _engine_torque_curve: PackedFloat32Array
var _engine_power_curve: PackedFloat32Array
var _idle_rpm: float = 800.0
var _max_rpm: float = 7500.0
var _peak_torque_rpm: float = 4000.0
var _peak_power_rpm: float = 6500.0
var _max_torque: float = 450.0           # Maximum engine torque Nm
var _max_power: float = 250.0            # Maximum power kW

# Gear ratios (ratio of transmission output to engine input)
var _gear_ratios: Array[float] = [0.0, 3.8, 2.2, 1.5, 1.1, 0.9, 0.75]
var _reverse_ratio: float = 3.9
var _final_drive_ratio: float = 3.5

# Drift parameters
var _drift_threshold: float = 0.25       # Slip threshold for drift
var _drift_recovery_rate: float = 0.15   # How fast drift ends
var _drift_traction_penalty: float = 0.6 # Traction loss during drift

func _ready() -> void:
	_setup_wheels()
	_generate_engine_curves()
	_connect_signals()
	_init_physics()

func _setup_wheels() -> void:
	"""Setup wheel position offsets relative to vehicle center."""
	var half_width = WHEEL_BASE_WIDTH * 0.5
	var half_length = WHEEL_BASE_LENGTH * 0.5
	
	_front_left_wheel_pos = Vector3(-half_width, -0.3, -half_length)
	_front_right_wheel_pos = Vector3(half_width, -0.3, -half_length)
	_rear_left_wheel_pos = Vector3(-half_width, -0.3, half_length)
	_rear_right_wheel_pos = Vector3(half_width, -0.3, half_length)

func _generate_engine_curves() -> void:
	"""Generate smooth torque and power curves based on engine characteristics."""
	const SAMPLE_POINTS: int = 100
	
	_engine_torque_curve.resize(SAMPLE_POINTS)
	_engine_power_curve.resize(SAMPLE_POINTS)
	
	for i in range(SAMPLE_POINTS):
		var rpm_percent = float(i) / float(SAMPLE_POINTS - 1)
		var normalized_rpm = rpm_percent * (_max_rpm - _idle_rpm) + _idle_rpm
		
		# Torque curve: bell-shaped around peak torque RPM
		var rpm_diff = abs(normalized_rpm - _peak_torque_rpm)
		var torque_factor = exp(-(rpm_diff * rpm_diff) / (2.0 * 1500.0 * 1500.0))
		var idle_factor = clamp((normalized_rpm - _idle_rpm) / (_peak_torque_rpm - _idle_rpm), 0.0, 1.0)
		var rpm_above_peak = max(0.0, (normalized_rpm - _peak_torque_rpm) / (_max_rpm - _peak_torque_rpm))
		var drop_off = 1.0 - (rpm_above_peak * rpm_above_peak * 0.3)
		
		_engine_torque_curve[i] = _max_torque * (torque_factor * idle_factor * drop_off)
		
		# Power = Torque * RPM / constant
		var rpm_rad = normalized_rpm * 2.0 * PI / 60.0
		_engine_power_curve[i] = (_engine_torque_curve[i] * rpm_rad) / 1000.0

func _init_physics() -> void:
	"""Initialize vehicle physics state."""
	current_gear = 0
	is_neutral = true
	is_clutch_engaged = false
	traction_factor = 1.0
	drift_angle = 0.0

func _connect_signals() -> void:
	"""Connect internal signals to handlers."""
	if powertrain:
		powertrain.rpm_changed.connect(_on_powertrain_rpm_changed)

func _process(delta: float) -> void:
	"""Main game loop for visual updates and non-physics calculations."""
	_update_visual_state(delta)
	_check_drift_state(delta)
	_handle_drift_traction()

func _physics_process(delta: float) -> void:
	"""Physics simulation step - runs at fixed timestep."""
	if not is_inside_tree():
		return
	
	# Update velocity from character body
	current_speed = linear_velocity.length()
	
	# Apply inputs
	_handle_inputs(delta)
	
	# Calculate and apply forces
	_apply_physics(delta)
	
	# Move character body
	move_and_slide()

func _handle_inputs(delta: float) -> void:
	"""Process player input and update control variables."""
	# Get normalized input values (already handled by InputManager)
	throttle_input = InputManager.get_axis("throttle", "brake").x
	brake_input = InputManager.get_axis("brake", "throttle").y
	steering_input = InputManager.get_axis("steer_left", "steer_right")
	clutch_input = InputManager.get_axis("clutch_down", "clutch_up")
	handbrake_input = InputManager.get_axis("handbrake", "")
	
	# Clamp inputs to valid range
	throttle_input = clamp(throttle_input, 0.0, 1.0)
	brake_input = clamp(brake_input, 0.0, 1.0)
	steering_input = clamp(steering_input, -1.0, 1.0)
	clutch_input = clamp(clutch_input, 0.0, 1.0)
	handbrake_input = clamp(handbrake_input, 0.0, 1.0)
	
	# Handle clutch engagement
	is_clutch_engaged = clutch_input > 0.1
	
	# Handle handbrake
	is_handbrake_on = handbrake_input > 0.1
	
	# Automatic gear shifting when neutral and speed conditions met
	if is_neutral and not is_clutch_engaged:
		_auto_shift_gears()

func _auto_shift_gears() -> void:
	"""Automatically shift gears based on RPM and speed."""
	if current_rpm <= _idle_rpm and current_speed < 1.0:
		current_gear = 0
		is_neutral = true
		gear_changed.emit(current_gear, current_gear)
		return
	
	if current_gear == 0 or current_gear == -1:
		if current_speed >= 1.0:
			current_gear = 1
			is_neutral = false
			gear_changed.emit(0, 1)
		return
	
	# Upshift logic
	if current_rpm > _max_rpm * 0.95 and current_gear < max_gears:
		_shift_gear(true)
	
	# Downshift logic
	elif current_rpm < _idle_rpm * 1.2 and current_gear > 1:
		_shift_gear(false)

func _shift_gear(up: bool) -> void:
	"""Manual or automatic gear shift with rev-matching."""
	var old_gear = current_gear
	var target_gear = current_gear + (1 if up else -1)
	
	# Validate gear change
	if up and target_gear > max_gears:
		return
	if not up and target_gear < -1:
		return
	
	# Neutral transition
	if target_gear == 0:
		current_gear = 0
		is_neutral = true
	else:
		current_gear = target_gear
		is_neutral = false
		is_in_reverse = (current_gear == -1)
	
	gear_changed.emit(old_gear, current_gear)
	clutch_engaged.emit(true)
	
	# Play shift sound
	if AudioManager:
		AudioManager.play_sound("gear_shift")

func _apply_physics(delta: float) -> void:
	"""Apply all vehicle physics forces including engine, brakes, drag, etc."""
	if is_neutral and not is_clutch_engaged:
		return
	
	# Calculate drive forces
	var drive_force: float = _calculate_drive_force()
	var brake_force: float = _calculate_brake_force()
	var steering_force: float = _calculate_steering_force()
	var drag_force: float = _calculate_drag_force()
	var rolling_resistance_force: float = _calculate_rolling_resistance()
	
	# Combine forces along velocity direction
	var total_drive_force = drive_force + brake_force + drag_force + rolling_resistance_force
	
	# Apply acceleration/deceleration
	var acceleration = total_drive_force / vehicle_mass
	
	# Update velocity based on acceleration
	if linear_velocity.length() > 0.1:
		var velocity_direction = linear_velocity.normalized()
		var new_speed = linear_velocity.length() + acceleration * delta
		new_speed = clamp(new_speed, 0.0, 120.0) # Max speed cap m/s (~432 km/h)
		linear_velocity = velocity_direction * new_speed
	else:
		linear_velocity = Vector3.ZERO
	
	# Apply steering rotation
	if current_speed > 0.5:
		var turn_rate = steering_input * 2.0 * (current_speed / 10.0)
		turn_rate = clamp(turn_rate, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE)
		rotation.y += turn_rate * delta
	
	# Apply lateral forces for drifting
	if is_drifting:
		_apply_drift_lateral_force(delta)
	
	# Sync velocity back to character body
	velocity = linear_velocity
	angular_velocity = Vector3(0.0, rotation.y, 0.0)

func _calculate_drive_force() -> float:
	"""Calculate drive force from engine torque and transmission."""
	if is_clutch_engaged or is_neutral:
		return 0.0
	
	var gear_ratio: float = _gear_ratios[current_gear] if current_gear > 0 else _reverse_ratio
	var total_ratio = gear_ratio * _final_drive_ratio
	
	# Get current engine torque based on RPM
	var torque = _get_current_engine_torque()
	
	# Drive force at wheels = torque * ratio / wheel_radius
	var wheel_radius: float = 0.33 # meters
	var drive_force = (torque * total_ratio) / wheel_radius
	
	# Apply traction limits
	var max_traction = vehicle_mass * 9.81 * traction_factor
	drive_force = min(drive_force, max_traction)
	
	return drive_force

func _calculate_brake_force() -> float:
	"""Calculate braking force applied to all wheels."""
	var base_brake_force = brake_input * braking_force
	
	# Handbrake adds extra rear brake bias
	if is_handbrake_on:
		base_brake_force *= 1.5
	
	# ABS prevents locking up
	if current_speed > 2.0:
		var slip_threshold = 0.15
		var wheel_slip = (base_brake_force / vehicle_mass) / (9.81 * 0.7)
		if wheel_slip > slip_threshold:
			base_brake_force *= 0.7
	
	return -base_brake_force

func _calculate_steering_force() -> float:
	"""Calculate lateral steering force for turning."""
	if current_speed < 1.0:
		return 0.0
	
	var steer_angle = steering_input * MAX_STEERING_ANGLE
	var cornering_force = cornering_stiffness * steer_angle * (current_speed / 20.0)
	cornering_force = clamp(cornering_force, -15000.0, 15000.0)
	
	return cornering_force

func _calculate_drag_force() -> float:
	"""Calculate aerodynamic drag force."""
	var air_density: float = 1.225  # kg/m^3 at sea level
	var frontal_area: float = 2.2    # m^2 typical car
	var velocity_squared = current_speed * current_speed
	
	var drag_force = 0.5 * air_density * drag_coefficient * frontal_area * velocity_squared
	return -drag_force

func _calculate_rolling_resistance() -> float:
	"""Calculate rolling resistance from tire friction."""
	var normal_force = vehicle_mass * 9.81
	var rolling_resist = normal_force * rolling_resistance * traction_factor
	return -rolling_resist

func _calculate_drift_lateral_force() -> float:
	"""Calculate lateral force during drift state."""
	if not is_drifting:
		return 0.0
	
	var lateral_accel = drift_angle * current_speed * 9.81
	return lateral_accel * vehicle_mass

func _get_current_engine_torque() -> float:
	"""Get engine torque based on current RPM."""
	if is_neutral or is_clutch_engaged:
		return 0.0
	
	# Normalize RPM to curve index
	var rpm_percent = (current_rpm - _idle_rpm) / (_max_rpm - _idle_rpm)
	rpm_percent = clamp(rpm_percent, 0.0, 1.0)
	
	var index = int(rpm_percent * (_engine_torque_curve.size() - 1))
	var torque = _engine_torque_curve[index]
	
	# Apply throttle interpolation
	var throttle_factor = throttle_input
	torque *= throttle_factor
	
	return torque

func _update_visual_state(delta: float) -> void:
	"""Update visual representations of vehicle state."""
	# Update speed display
	speed_changed.emit(current_speed)
	
	# Update RPM display
	rpm_changed.emit(current_rpm)
	
	# Animate suspension based on forces
	_update_suspension_animation()

func _check_drift_state(delta: float) -> void:
	"""Check and manage drift state based on inputs and physics."""
	if current_speed < DRIFT_THRESHOLD_SPEED:
		is_drifting = false
		return
	
	# Check if drift should start
	var steer_threshold = abs(steering_input) > DRIFT_THRESHOLD_STEER
	var throttle_threshold = throttle_input > 0.5
	var is_oversteering = angular_velocity.y > 0.3
	
	if steer_threshold and throttle_threshold and is_oversteering:
		is_drifting = true
		drift_state_changed.emit(true)
	elif abs(steering_input) < DRIFT_THRESHOLD_STEER * 0.5:
		is_drifting = false
		drift_state_changed.emit(false)

func _handle_drift_traction(delta: float) -> void:
	"""Handle traction changes during drift."""
	if is_drifting:
		traction_factor = lerp(traction_factor, _drift_traction_penalty, delta * _drift_recovery_rate)
	else:
		traction_factor = lerp(traction_factor, 1.0, delta * _drift_recovery_rate)
	
	traction_factor = clamp(traction_factor, 0.3, 1.0)

func _apply_drift_lateral_force(delta: float) -> void:
	"""Apply lateral forces that cause sliding during drift."""
	var lateral_force = steering_input * 8000.0 * traction_factor
	var drift_offset = drift_angle * current_speed * 0.5
	
	# Add lateral velocity component
	var lateral_velocity = Vector3(0.0, 0.0, 0.0)
	lateral_velocity.x = drift_offset
	
	# Blend with main velocity
	var combined_velocity = linear_velocity + lateral_velocity
	linear_velocity = linear_velocity.lerp(combined_velocity, delta * 0.1)

func _update_suspension_animation() -> void:
	"""Animate suspension compression based on forces."""
	# Simplified suspension animation
	var suspension_compression = current_speed * 0.01
	suspension_compression = clamp(suspension_compression, 0.0, 0.1)
	
	# Apply to child nodes if they exist
	if has_node("Suspension"):
		var susp_node = $Suspension
		susp_node.scale.y = 1.0 - suspension_compression

func _on_powertrain_rpm_changed(new_rpm: float) -> void:
	"""Handle RPM changes from powertrain component."""
	current_rpm = new_rpm

func get_vehicle_status() -> Dictionary:
	"""Return comprehensive vehicle status dictionary."""
	return {
		"speed": current_speed,
		"speed_kmh": current_speed * 3.6,
		"rpm": current_rpm,
		"gear": current_gear,
		"is_neutral": is_neutral,
		"is_reverse": is_in_reverse,
		"throttle": throttle_input,
		"brake": brake_input,
		"steering": steering_input,
		"is_handbrake_on": is_handbrake_on,
		"is_drifting": is_drifting,
		"traction_factor": traction_factor
	}

func reset_vehicle() -> void:
	"""Reset vehicle to initial state."""
	linear_velocity = Vector3.ZERO
	angular_velocity = Vector3.ZERO
	current_speed = 0.0
	current_rpm = _idle_rpm
	current_gear = 0
	is_neutral = true
	is_in_reverse = false
	is_handbrake_on = false
	is_drifting = false
	traction_factor = 1.0
	drift_angle = 0.0

func set_position_and_rotation(pos: Vector3, rot: Quaternion) -> void:
	"""Set vehicle position and orientation directly."""
	global_position = pos
	rotation = rot.to_euler()

func take_damage(damage_amount: float, collision_point: Vector3) -> void:
	"""Apply damage to vehicle from collision."""
	if AudioManager:
		AudioManager.play_sound("collision_impact")
	
	# Reduce traction temporarily
	traction_factor = max(0.3, traction_factor - 0.2)
	
	# Apply knockback
	var knockback_dir = -linear_velocity.normalized() if linear_velocity.length() > 0.1 else Vector3.UP
	linear_velocity += knockback_dir * damage_amount * 2.0
	
	# Log collision
	collision_detected.emit({
		"damage": damage_amount,
		"point": collision_point,
		"timestamp": Time.get_unix_time_from_system()
	})

func save_vehicle_state() -> Dictionary:
	"""Save current vehicle state for replay/saving."""
	return {
		"position": global_position,
		"rotation": global_transform.basis.get_euler(),
		"velocity": linear_velocity,
		"angular_velocity": angular_velocity,
		"speed": current_speed,
		"rpm": current_rpm,
		"gear": current_gear,
		"inputs": {
			"throttle": throttle_input,
			"brake": brake_input,
			"steering": steering_input,
			"handbrake": handbrake_input
		}
	}

func load_vehicle_state(state_data: Dictionary) -> void:
	"""Load vehicle state from saved data."""
	if state_data.has("position"):
		global_position = state_data.position
	if state_data.has("rotation"):
		rotation = state_data.rotation
	if state_data.has("velocity"):
		linear_velocity = state_data.velocity
	if state_data.has("speed"):
		current_speed = state_data.speed
	if state_data.has("rpm"):
		current_rpm = state_data.rpm
	if state_data.has("gear"):
		current_gear = state_data.gear
		is_neutral = (current_gear == 0)
		is_in_reverse = (current_gear == -1)

func _input(event: InputEvent) -> void:
	"""Handle direct input events as fallback."""
	if event.is_action_pressed("shift_up"):
		_shift_gear(true)
	elif event.is_action_pressed("shift_down"):
		_shift_gear(false)
	elif event.is_action_pressed("toggle_drift"):
		is_drifting = !is_drifting
		drift_state_changed.emit(is_drifting)
</FILE>