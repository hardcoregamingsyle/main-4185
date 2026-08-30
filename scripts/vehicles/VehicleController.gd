extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller base class
## Handles throttle, brake, steering inputs, wheel forces, gear shifting logic
## Integrates with PhysicsSettings constants for consistent tuning across all vehicles
## Copyright 2026 Thalamus Racing Simulator Project

signal speed_changed(speed: float)
signal rpm_changed(rpm: float)
signal gear_changed(old_gear: int, new_gear: int)
signal vehicle_state_changed(state: VehicleState)
signal damage_taken(damage_amount: float)
signal wheel_slip_detected(wheel_index: int, slip_ratio: float)

enum VehicleState { IDLE, ACCELERATING, BRAKING, COASTING, DRIFTING, CRASHED }

# ============================================================================
# PHYSICS CONSTANTS FROM SETTINGS
# ============================================================================

var _physics_settings: PhysicsSettings = null
var _powertrain: Powertrain = null

# ============================================================================
# VEHICLE STATE
# ============================================================================

@export_group("Vehicle State")
@export var current_speed: float = 0.0: set = _set_current_speed
@export var forward_velocity: Vector3 = Vector3.ZERO
@export var lateral_velocity: Vector3 = Vector3.ZERO
@export var angular_velocity: Vector3 = Vector3.ZERO
@export var rpm: float = 0.0: set = _set_rpm
@export var current_gear: int = 0
@export var clutch_engaged: bool = true
@export var handbrake_active: bool = false

@export var vehicle_state: VehicleState = VehicleState.IDLE: set = _set_vehicle_state

# ============================================================================
# INPUT VALUES
# ============================================================================

@export_group("Input Values")
@export var throttle_input: float = 0.0: set = _set_throttle_input
@export var brake_input: float = 0.0: set = _set_brake_input
@export var steering_input: float = 0.0: set = _set_steering_input
@export var shift_up_requested: bool = false
@export var shift_down_requested: bool = false
@export var nitro_active: bool = false

# ============================================================================
# WHEEL PHYSICS
# ============================================================================

@export_group("Wheel Configuration")
@export var front_track_width: float = 1.6
@export var rear_track_width: float = 1.6
@export var wheelbase: float = 2.8
@export var suspension_travel: float = 0.15
@export var wheel_radius: float = 0.33

# Wheel indices
const FRONT_LEFT_WHEEL: int = 0
const FRONT_RIGHT_WHEEL: int = 1
const REAR_LEFT_WHEEL: int = 2
const REAR_RIGHT_WHEEL: int = 3

# Wheel state tracking
var _wheel_states: Array[Dictionary] = []
var _wheel_contact_points: Array[Vector3] = []
var _wheel_ground_normal: Array[Vector3] = []
var _wheel_suspension_compression: Array[float] = [0.0, 0.0, 0.0, 0.0]

# ============================================================================
# PHYSICS CALCULATIONS
# ============================================================================

var _vehicle_mass: float = 1500.0
var _drag_coefficient: float = 0.32
var _frontal_area: float = 2.2
var _downforce_coefficient: float = 0.5
var _traction_control_active: bool = true
var _anti_lock_braking_active: bool = true

# ============================================================================
# ENGINE OUTPUTS
# ============================================================================

var _engine_torque: float = 0.0
var _transmission_torque: float = 0.0
var _drive_wheels_force: Vector3 = Vector3.ZERO
var _brake_force: float = 0.0
var _drift_angle: float = 0.0
var _grip_level: float = 1.0

# ============================================================================
# INTERNAL VARIABLES
# ============================================================================

var _acceleration: float = 0.0
var _deceleration: float = 0.0
var _steering_angle: float = 0.0
var _target_gear: int = 0
var _gear_shift_timer: float = 0.0
var _min_gear_shift_delay: float = 0.5
var _speedometer_reading: float = 0.0
var _odometer_total: float = 0.0

# Nitro system
var _nitro_available: bool = true
var _nitro_capacity: float = 100.0
var _nitro_charge: float = 100.0
var _nitro_depletion_rate: float = 50.0
var _nitro_boost_multiplier: float = 1.5

# Damage system
var _health: float = 100.0
var _max_health: float = 100.0
var _collision_damage_accumulator: float = 0.0
var _last_collision_impact: float = 0.0

# ============================================================================
# INITIALIZATION
# ============================================================================

func _ready() -> void:
	_init_physics_settings()
	_init_powertrain()
	_init_wheel_states()
	_connect_signals()
	_reset_vehicle()

func _init_physics_settings() -> void:
	# Initialize from autoload singleton
	if Engine.has_singleton("PhysicsSettings"):
		_physics_settings = Engine.get_singleton("PhysicsSettings")
	else:
		# Fallback if not loaded yet
		var settings_script: Script = load("res://scripts/core/PhysicsSettings.gd")
		_physics_settings = settings_script.new()
	
	# Apply physics settings to vehicle
	_apply_physics_settings()

func _init_powertrain() -> void:
	# Initialize powertrain if not already present
	if _powertrain == null:
		_powertrain = Powertrain.new()
	
	# Connect powertrain signals
	_powertrain.rpm_changed.connect(_on_powertrain_rpm_changed)
	_powertrain.gear_changed.connect(_on_powertrain_gear_changed)
	_powertrain.clutch_engaged.connect(_on_clutch_engaged)
	_powertrain.clutch_disengaged.connect(_on_clutch_disengaged)
	_powertrain.throttle_change.connect(_on_throttle_change)

func _init_wheel_states() -> void:
	# Initialize wheel state arrays
	for i in range(4):
		_wheel_states.append({
			"index": i,
			"angular_velocity": 0.0,
			"slip_ratio": 0.0,
			"ground_friction": 1.0,
			"suspension_position": 0.0,
			"is_in_air": true,
			"brake_pressure": 0.0,
			"drive_force": 0.0,
			"lateral_force": 0.0,
			"vertical_force": 0.0
		})
		
		_wheel_contact_points.append(Vector3.ZERO)
		_wheel_ground_normal.append(Vector3.UP)
		_wheel_suspension_compression.append(0.0)

func _connect_signals() -> void:
	# Connect to GameManager for game state changes
	if GameManager:
		GameManager.game_state_changed.connect(_on_game_state_changed)

func _reset_vehicle() -> void:
	current_speed = 0.0
	rpm = 0.0
	current_gear = 0
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	vehicle_state = VehicleState.IDLE
	_engine_torque = 0.0
	_transmission_torque = 0.0
	_drift_angle = 0.0
	_grip_level = 1.0
	_health = _max_health
	_nitro_charge = _nitro_capacity

# ============================================================================
# MAIN UPDATE LOOP
# ============================================================================

func _physics_process(delta: float) -> void:
	# Update physics every frame
	_update_input(delta)
	_update_engine(delta)
	_update_transmission(delta)
	_update_wheels(delta)
	_update_vehicle_dynamics(delta)
	_update_aerodynamics(delta)
	_update_damage_system(delta)
	_handle_gear_shifting(delta)

func _update_input(delta: float) -> void:
	# Clamp input values
	throttle_input = clamp(throttle_input, 0.0, 1.0)
	brake_input = clamp(brake_input, 0.0, 1.0)
	steering_input = clamp(steering_input, -1.0, 1.0)
	
	# Handle handbrake override
	if handbrake_active:
		brake_input = max(brake_input, 0.8)
	
	# Apply steering smoothing
	_steer_smooth(delta)

func _steer_smooth(delta: float) -> void:
	# Smooth steering transitions
	var target_steering = steering_input * _get_max_steering_angle()
	_steering_angle = lerp(_steering_angle, target_steering, delta * 10.0)

func _get_max_steering_angle() -> float:
	# Maximum steering angle in radians
	return deg_to_rad(35.0)

func _update_engine(delta: float) -> void:
	# Calculate engine torque based on RPM and throttle
	_calculate_engine_torque()
	
	# Apply clutch effect
	if not clutch_engaged:
		_engine_torque *= 0.1
	
	# Apply nitro boost if active
	if nitro_active and _nitro_charge > 0:
		_engine_torque *= _nitro_boost_multiplier
		_nitro_charge -= delta * _nitro_depletion_rate
		if _nitro_charge < 0:
			_nitro_charge = 0
			nitro_active = false
		else:
			nitro_available = false

func _calculate_engine_torque() -> void:
	# Get torque curve value based on current RPM
	var torque_curve_factor = _get_torque_curve_factor()
	
	# Calculate torque from throttle input
	if throttle_input > 0.0:
		_engine_torque = _powertrain.peak_torque_nm * torque_curve_factor * throttle_input
	else:
		# Engine braking when no throttle
		_engine_torque = -_powertrain.peak_torque_nm * 0.3 * (throttle_input + 1.0) * (1.0 - torque_curve_factor)
	
	# Clamp torque
	_engine_torque = clamp(_engine_torque, -_powertrain.peak_torque_nm, _powertrain.peak_torque_nm)

func _get_torque_curve_factor() -> float:
	# Get normalized torque output based on RPM curve
	var rpm_ratio = (rpm - _powertrain.idle_rpm) / (_powertrain.max_rpm - _powertrain.idle_rpm)
	rpm_ratio = clamp(rpm_ratio, 0.0, 1.0)
	
	# Simple parabolic torque curve approximation
	# Peak torque typically at ~40-60% of redline
	var peak_torque_ratio = _powertrain.peak_torque_rpm / _powertrain.max_rpm
	var torque_factor = 1.0 - pow((rpm_ratio - peak_torque_ratio) / peak_torque_ratio, 2)
	
	return clamp(torque_factor, 0.0, 1.2)

func _update_transmission(delta: float) -> void:
	# Calculate transmission torque output
	_transmission_torque = _engine_torque * _get_gear_ratio() * _powertrain.drivetrain_efficiency
	
	# Apply final drive ratio
	_transmission_torque *= _powertrain.final_drive_ratio

func _get_gear_ratio() -> float:
	# Get gear ratio for current gear
	if current_gear < 0:
		return _powertrain._reverse_ratio
	elif current_gear >= _powertrain.number_of_gears:
		return _powertrain._gear_ratios[_powertrain.number_of_gears - 1]
	else:
		if current_gear == 0:
			return 1.0  # Neutral
		elif current_gear <= _powertrain.number_of_gears:
			return _powertrain._gear_ratios[current_gear - 1]
		else:
			return _powertrain._gear_ratios[_powertrain.number_of_gears - 1]

func _update_wheels(delta: float) -> void:
	# Update each wheel's physics state
	for i in range(4):
		_update_wheel(i, delta)

func _update_wheel(wheel_index: int, delta: float) -> void:
	var wheel_state: Dictionary = _wheel_states[wheel_index]
	
	# Calculate wheel angular velocity based on vehicle speed and slip
	var wheel_speed = _calculate_wheel_speed(wheel_index)
	wheel_state.angular_velocity = wheel_speed
	
	# Calculate slip ratio
	var ground_speed = _calculate_ground_speed(wheel_index)
	var slip_ratio = (wheel_speed - ground_speed) / max(abs(ground_speed), 1.0)
	wheel_state.slip_ratio = slip_ratio
	
	# Detect excessive slip
	if abs(slip_ratio) > 0.3:
		emit_signal("wheel_slip_detected", wheel_index, slip_ratio)
	
	# Update suspension compression
	_update_wheel_suspension(wheel_index, delta)
	
	# Apply traction control if active
	if _traction_control_active and abs(slip_ratio) > 0.15:
		_apply_traction_control(wheel_index, delta, slip_ratio)
	
	# Apply anti-lock braking if active
	if _anti_lock_braking_active and wheel_state.brake_pressure > 0 and abs(slip_ratio) > 0.2:
		_apply_abs(wheel_index, delta, slip_ratio)

func _calculate_wheel_speed(wheel_index: int) -> float:
	# Calculate wheel rotational speed in rad/s
	var wheel_type = _get_wheel_type(wheel_index)
	var gear_ratio = _get_gear_ratio()
	var final_drive = _powertrain.final_drive_ratio
	var wheel_circumference = PI * 2.0 * _powertrain.wheel_radius_meters
	
	var drive_torque = _transmission_torque
	
	# Rear wheels get drive torque (RWD configuration)
	if wheel_type == "rear":
		drive_torque /= 2.0
	else:
		drive_torque = 0.0
	
	# Convert torque to linear force at wheel contact patch
	var wheel_torque = drive_torque * _powertrain.drivetrain_efficiency
	var wheel_force = wheel_torque / _powertrain.wheel_radius_meters
	
	# Convert force to angular velocity
	var wheel_acceleration = wheel_force / (_powertrain.vehicle_mass_kg / 4.0)
	var wheel_omega = wheel_acceleration * delta
	
	return wheel_omega

func _calculate_ground_speed(wheel_index: int) -> float:
	# Calculate actual ground speed at wheel position
	var wheel_type = _get_wheel_type(wheel_index)
	
	if wheel_type == "front":
		return current_speed * cos(_steering_angle)
	else:
		return current_speed

func _get_wheel_type(wheel_index: int) -> String:
	if wheel_index == FRONT_LEFT_WHEEL or wheel_index == FRONT_RIGHT_WHEEL:
		return "front"
	else:
		return "rear"

func _update_wheel_suspension(wheel_index: int, delta: float) -> void:
	var wheel_state: Dictionary = _wheel_states[wheel_index]
	
	# Simulate suspension compression based on vertical forces
	var target_compression = _calculate_suspension_target(wheel_index)
	wheel_state.suspension_position = lerp(wheel_state.suspension_position, target_compression, delta * 5.0)
	
	# Clamp suspension travel
	wheel_state.suspension_position = clamp(wheel_state.suspension_position, -suspension_travel, suspension_travel)

func _calculate_suspension_target(wheel_index: int) -> float:
	# Simplified suspension calculation
	var weight_distribution = 0.5  # Front/rear weight distribution
	var wheel_weight = _vehicle_mass * 9.81 * (weight_distribution if _is_front_wheel(wheel_index) else 1.0 - weight_distribution)
	var spring_stiffness = 50000.0
	
	return wheel_weight / spring_stiffness

func _is_front_wheel(wheel_index: int) -> bool:
	return wheel_index == FRONT_LEFT_WHEEL or wheel_index == FRONT_RIGHT_WHEEL

func _apply_traction_control(wheel_index: int, delta: float, slip_ratio: float) -> void:
	# Reduce drive torque to wheel causing excessive slip
	var wheel_state: Dictionary = _wheel_states[wheel_index]
	var reduction_factor = min(abs(slip_ratio) * 2.0, 0.5)
	wheel_state.drive_force *= (1.0 - reduction_factor)

func _apply_abs(wheel_index: int, delta: float, slip_ratio: float) -> void:
	# Modulate brake pressure to prevent lockup
	var wheel_state: Dictionary = _wheel_states[wheel_index]
	var modulation = 1.0 - abs(slip_ratio) * 0.5
	wheel_state.brake_pressure *= modulation

func _update_vehicle_dynamics(delta: float) -> void:
	# Calculate net forces on vehicle
	_calculate_net_forces()
	
	# Apply forces to velocity
	_update_velocity(delta)
	
	# Update vehicle state
	_update_vehicle_state()
	
	# Update odometer
	_odometer_total += current_speed * delta

func _calculate_net_forces() -> void:
	# Calculate total drive force from wheels
	var drive_force = Vector3.ZERO
	
	for i in range(4):
		var wheel_state: Dictionary = _wheel_states[i]
		var wheel_forward = forward_velocity.normalized() if forward_velocity.length() > 0.1 else Vector3.FORWARD
		
		# Drive force along wheel direction
		var force_magnitude = wheel_state.drive_force
		if _is_rear_wheel(i):
			force_magnitude *= _transmission_torque / (_powertrain.peak_torque_nm * 0.5)
		
		drive_force += wheel_forward * force_magnitude
	
	# Add drag force (air resistance)
	var air_density = 1.225  # kg/m³ at sea level
	var drag_force = 0.5 * air_density * self._drag_coefficient * self._frontal_area * current_speed * current_speed
	var drag_vector = -forward_velocity.normalized() * drag_force if forward_velocity.length() > 0.1 else Vector3.ZERO
	
	# Add downforce (aerodynamic)
	var downforce = 0.5 * air_density * self._downforce_coefficient * self._frontal_area * current_speed * current_speed
	var downforce_vector = Vector3.DOWN * downforce
	
	# Combine forces
	var net_force = drive_force + drag_vector + downforce_vector
	
	# Calculate acceleration from net force
	_acceleration = net_force.x / _vehicle_mass

func _is_rear_wheel(wheel_index: int) -> bool:
	return wheel_index == REAR_LEFT_WHEEL or wheel_index == REAR_RIGHT_WHEEL

func _update_velocity(delta: float) -> void:
	# Apply acceleration to velocity
	forward_velocity.x += _acceleration * delta
	
	# Apply gravity component
	forward_velocity.y -= _physics_settings.gravity * delta
	
	# Apply friction/drag
	var friction = 0.995  # Air and rolling resistance
	forward_velocity *= friction
	
	# Clamp maximum speed
	var max_speed = _get_max_speed()
	forward_velocity = forward_velocity.limit_length(max_speed)
	
	# Update speed property
	current_speed = forward_velocity.length()

func _get_max_speed() -> float:
	# Calculate theoretical maximum speed based on top gear
	var top_gear_ratio = _powertrain._gear_ratios[_powertrain.number_of_gears - 1]
	var max_wheel_omega = _powertrain.max_rpm * 2.0 * PI / 60.0
	var wheel_circumference = 2.0 * PI * _powertrain.wheel_radius_meters
	
	return max_wheel_omega * wheel_circumference * top_gear_ratio * _powertrain.final_drive_ratio

func _update_vehicle_state() -> void:
	# Determine current vehicle state based on inputs
	var old_state = vehicle_state
	
	if current_speed < 0.1 and throttle_input < 0.01:
		vehicle_state = VehicleState.IDLE
	elif throttle_input > 0.7:
		vehicle_state = VehicleState.ACCELERATING
	elif brake_input > 0.5:
		vehicle_state = VehicleState.BRAKING
	elif throttle_input < 0.1 and brake_input < 0.1:
		vehicle_state = VehicleState.COASTING
	elif _drift_angle > 0.3:
		vehicle_state = VehicleState.DRIFTING
	elif _health < 30.0:
		vehicle_state = VehicleState.CRASHED
	
	if old_state != vehicle_state:
		emit_signal("vehicle_state_changed", vehicle_state)

func _update_aerodynamics(delta: float) -> void:
	# Update aerodynamic effects
	var air_density = 1.225
	
	# Downforce calculation
	var dynamic_pressure = 0.5 * air_density * current_speed * current_speed
	var total_downforce = dynamic_pressure * self._downforce_coefficient * self._frontal_area
	
	# Distribute downforce between front and rear
	var front_downforce = total_downforce * self.front_downforce_ratio
	var rear_downforce = total_downforce * self.rear_downforce_ratio
	
	# Apply to grip levels
	_grip_level = 1.0 + (total_downforce / (_vehicle_mass * _physics_settings.gravity)) * 0.5

func _update_damage_system(delta: float) -> void:
	# Accumulate collision damage
	if _collision_damage_accumulator > 0:
		_health -= _collision_damage_accumulator * delta
		_collision_damage_accumulator = 0
	
	# Check for critical damage
	if _health <= 0:
		_health = 0
		vehicle_state = VehicleState.CRASHED
		emit_signal("damage_taken", _max_health)

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================

func _handle_gear_shifting(delta: float) -> void:
	# Reset shift timers
	if shift_up_requested or shift_down_requested:
		_gear_shift_timer = _min_gear_shift_delay
		shift_up_requested = false
		shift_down_requested = false
	
	# Apply gear shifts after delay
	if _gear_shift_timer > 0:
		_gear_shift_timer -= delta
		return
	
	# Auto-shift logic
	if _auto_shift_enabled():
		_auto_shift_gear()
	
	# Manual shift logic
	if shift_up_requested:
		_manual_shift_up()
	elif shift_down_requested:
		_manual_shift_down()

func _auto_shift_enabled() -> bool:
	# Check if auto-shifting is enabled (could be controlled by input manager)
	return true

func _auto_shift_gear() -> void:
	# Automatic gear shifting based on RPM
	var optimal_gear = _calculate_optimal_gear()
	if optimal_gear != current_gear:
		current_gear = optimal_gear
		emit_signal("gear_changed", optimal_gear - current_gear, current_gear)

func _calculate_optimal_gear() -> int:
	# Calculate best gear based on current RPM and speed
	var ideal_rpm = _powertrain.peak_power_rpm * 0.85  # Shift at 85% of peak power RPM
	
	# Find gear that keeps RPM near ideal
	for gear in range(1, _powertrain.number_of_gears + 1):
		var expected_rpm = _calculate_expected_rpm(gear)
		if expected_rpm <= ideal_rpm:
			return gear
	
	return 1

func _calculate_expected_rpm(gear: int) -> float:
	# Calculate what RPM would be at current speed in given gear
	var wheel_omega = current_speed / _powertrain.wheel_radius_meters
	var gear_ratio = _get_gear_ratio_for(gear)
	var final_drive = _powertrain.final_drive_ratio
	
	var engine_omega = wheel_omega * gear_ratio * final_drive
	var rpm = engine_omega * 60.0 / (2.0 * PI)
	
	return rpm

func _get_gear_ratio_for(gear: int) -> float:
	if gear <= 0:
		return _powertrain._reverse_ratio
	elif gear > _powertrain.number_of_gears:
		return _powertrain._gear_ratios[_powertrain.number_of_gears - 1]
	else:
		return _powertrain._gear_ratios[gear - 1]

func _manual_shift_up() -> void:
	if current_gear < _powertrain.number_of_gears:
		var old_gear = current_gengear
		current_gear += 1
		emit_signal("gear_changed", old_gear, current_gear)
		_powertrain.emit_signal("gear_changed", old_gear, current_gear)

func _manual_shift_down() -> void:
	if current_gear > 1:
		var old_gear = current_gear
		current_gear -= 1
		emit_signal("gear_changed", old_gear, current_gear)
		_powertrain.emit_signal("gear_changed", old_gear, current_gear)

# ============================================================================
# PHYSICS APPLIERS
# ============================================================================

func apply_throttle(force_value: float) -> void:
	"""Apply direct throttle force to vehicle"""
	throttle_input = clamp(force_value, 0.0, 1.0)

func apply_brake(force_value: float) -> void:
	"""Apply direct brake force to vehicle"""
	brake_input = clamp(force_value, 0.0, 1.0)

func apply_steering(angle_degrees: float) -> void:
	"""Apply steering angle directly"""
	steering_input = normalize_steering_angle(angle_degrees)

func apply_force(direction: Vector3, magnitude: float) -> void:
	"""Apply external force to vehicle body"""
	forward_velocity += direction * magnitude / _vehicle_mass

func apply_torque(torque_vector: Vector3) -> void:
	"""Apply rotational torque to vehicle"""
	angular_velocity += torque_vector / (_vehicle_mass * 10.0)

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================

func reset() -> void:
	"""Reset vehicle to initial state"""
	_reset_vehicle()

func get_speed_kmh() -> float:
	"""Get speed in kilometers per hour"""
	return current_speed * 3.6

func get_speed_mph() -> float:
	"""Get speed in miles per hour"""
	return current_speed * 2.23694

func get_distance_traveled() -> float:
	"""Get total distance traveled in meters"""
	return _odometer_total

func take_damage(damage_amount: float) -> void:
	"""Take damage from collision"""
	_health -= damage_amount
	_collision_damage_accumulator = damage_amount
	emit_signal("damage_taken", damage_amount)

func is_operational() -> bool:
	"""Check if vehicle is still operational"""
	return _health > 0

func _set_current_speed(value: float) -> void:
	current_speed = value
	emit_signal("speed_changed", value)

func _set_rpm(value: float) -> void:
	rpm = value
	emit_signal("rpm_changed", value)

func _set_vehicle_state(value: VehicleState) -> void:
	vehicle_state = value

func _set_throttle_input(value: float) -> void:
	throttle_input = value

func _set_brake_input(value: float) -> void:
	brake_input = value

func _set_steering_input(value: float) -> void:
	steering_input = value

func normalize_steering_angle(degrees: float) -> float:
	"""Normalize steering angle to -1.0 to 1.0 range"""
	var max_angle = deg_to_rad(35.0)
	return clamp(degrees / max_angle, -1.0, 1.0)

# ============================================================================
# SIGNAL HANDLERS
# ============================================================================

func _on_powertrain_rpm_changed(new_rpm: float) -> void:
	rpm = new_rpm

func _on_powertrain_gear_changed(old_gear: int, new_gear: int) -> void:
	current_gear = new_gear

func _on_clutch_engaged() -> void:
	clutch_engaged = true

func _on_clutch_disengaged() -> void:
	clutch_engaged = false

func _on_throttle_change(value: float) -> void:
	throttle_input = value

func _on_game_state_changed(new_state: GameManager.GameState) -> void:
	if new_state == GameManager.GameState.RACE_ACTIVE:
		pass  # Resume vehicle controls
	elif new_state == GameManager.GameState.RACE_PAUSED:
		pass  # Pause vehicle physics

# ============================================================================
# DEBUG & VISUALIZATION
# ============================================================================

func _draw_debug_info() -> void:
	"""Draw debug visualization of vehicle physics state"""
	if not GameManager.debug_mode:
		return
	
	# Draw wheel positions
	for i in range(4):
		var offset = _get_wheel_offset(i)
		var world_pos = global_transform * offset
		draw_sphere(world_pos, 0.1, Color.GREEN if _wheel_states[i].is_in_air else Color.YELLOW)

func _get_wheel_offset(wheel_index: int) -> Vector3:
	"""Get wheel position offset from vehicle center"""
	var track_half = (REAR_LEFT_WHEEL if wheel_index >= REAR_LEFT_WHEEL else FRONT_LEFT_WHEEL)
	var wheelbase_offset = (FRONT_LEFT_WHEEL if wheel_index < FRONT_LEFT_WHEEL else REAR_LEFT_WHEEL)
	
	return Vector3(
		(track_half * (front_track_width if wheel_index < FRONT_LEFT_WHEEL else rear_track_width)),
		-0.33,  # Wheel radius height
		(wheelbase_offset * wheelbase)
	)

func draw_sphere(center: Vector3, radius: float, color: Color) -> void:
	"""Draw debug sphere (placeholder for visual debugging)"""
	pass