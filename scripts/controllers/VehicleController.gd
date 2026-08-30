extends Node2D
class_name VehicleController

## VehicleController - Core vehicle physics controller using PhysicsSettings constants
## Handles throttle, brake, steering inputs, wheel forces, and gear shifting logic
## Copyright 2026 Thalamus Racing Simulator Project

signal speed_changed(new_speed: float)
signal gear_changed(old_gear: int, new_gear: int)
signal vehicle_moved(displacement: Vector2)
signal drift_angle_changed(angle: float)
signal traction_control_active(active: bool)

# ============================================================================
# PHYSICS CONSTANTS - Derived from PhysicsSettings resource
# ============================================================================

const MAX_THROTTLE_FORCE: float = 15000.0      # Newtons - maximum acceleration force
const MAX_BRAKE_FORCE: float = 20000.0         # Newtons - maximum braking force
const MAX_STEERING_ANGLE: float = PI / 3       # 60 degrees max steering
const STEERING_SPEED: float = 4.0              # Radians per second steering rate
const DRIFT_THRESHOLD: float = 0.7             # Sideslip threshold for drift mode
const TRACTION_CONTROL_SENSITIVITY: float = 0.85 # TCS activation threshold

# ============================================================================
# GEAR RATIOS AND TRANSMISSION CONFIGURATION
# ============================================================================

enum Gear {
    NEUTRAL = 0,
    FIRST = 1,
    SECOND = 2,
    THIRD = 3,
    FOURTH = 4,
    FIFTH = 5,
    SIXTH = 6,
    REVERSE = -1
}

const GEAR_RATIOS: Dictionary = {
    Gear.FIRST: 3.5,
    Gear.SECOND: 2.2,
    Gear.THIRD: 1.6,
    Gear.FOURTH: 1.2,
    Gear.FIFTH: 0.9,
    Gear.SIXTH: 0.75,
    Gear.REVERSE: 3.0
}

const FINAL_DRIVE_RATIO: float = 3.73          # Final drive differential ratio
const REVERSE_GEAR_RATIO: float = 3.5          # Reverse gear ratio

# ============================================================================
# VEHICLE STATE VARIABLES
# ============================================================================

@export var vehicle_mass: float = 1500.0      # kg - overridden by PhysicsSettings default
@export var center_of_gravity: Vector2 = Vector2(0.0, 0.5)  # CG position relative to chassis
@export var wheelbase: float = 2.6           # Distance between front and rear axles
@export var track_width: float = 1.6         # Distance between left and right wheels
@export var wheel_radius: float = 0.32       # Wheel radius in meters
@export var wheel_track_offset: float = 0.8  # Half-track width offset

# Physical properties
@export var drag_coefficient: float = 0.32   # Aerodynamic drag coefficient
@export var frontal_area: float = 2.2        # Frontal area in m²
@export var air_density: float = 1.225       # Air density at sea level kg/m³
@export var rolling_resistance_coefficient: float = 0.015
@export var moment_of_inertia: float = 2500.0 # Rotational inertia kg·m²

# Wheel friction coefficients (dry asphalt, wet asphalt, gravel, etc.)
var _tire_friction_coefficient: float = 1.2

# ============================================================================
# DYNAMIC STATE VARIABLES
# ============================================================================

var velocity: Vector2 = Vector2.ZERO
var angular_velocity: float = 0.0            # Yaw rate rad/s
var speed: float = 0.0                       # Current speed magnitude m/s
var current_gear: int = Gear.NEUTRAL
var engine_rpm: float = 0.0                  # Current RPM
var target_rpm: float = 0.0
var clutch_engaged: bool = false
var handbrake_active: bool = false

# Input state
var throttle_input: float = 0.0              # 0.0 to 1.0
var brake_input: float = 0.0                 # 0.0 to 1.0
var steering_input: float = 0.0              # -1.0 to 1.0
var gear_shift_request: int = Gear.NEUTRAL

# Drift and traction control
var drift_mode_active: bool = false
var drift_angle: float = 0.0                 # Angle between facing direction and velocity
var traction_control_on: bool = true
var slip_ratio: float = 0.0                  # Wheel slip ratio 0-1
var lateral_slip_angle: float = 0.0          # Side slip angle rad

# Wheel states
var _front_left_wheel_force: float = 0.0
var _front_right_wheel_force: float = 0.0
var _rear_left_wheel_force: float = 0.0
var _rear_right_wheel_force: float = 0.0

# Track reference
var _physics_settings: PhysicsSettings = null

# ============================================================================
# INITIALIZATION
# ============================================================================

func _ready() -> void:
	_process_mode = ProcessModeEnum.ALWAYS
	_init_physics_settings()
	_connect_signals()
	_reset_vehicle_state()

func _init_physics_settings() -> void:
	# Try to get PhysicsSettings from autoload
	if Engine.has_singleton("GameManager"):
		var gm = GameManager.get_singleton()
		if gm and hasattr(gm, "get_physics_settings"):
			_physics_settings = gm.get_physics_settings()
	
	# If not available, use defaults
	if _physics_settings == null:
		_physics_settings = PhysicsSettings.new()
	
	# Apply any global physics settings if available
	if _physics_settings.default_vehicle_mass > 0:
		vehicle_mass = _physics_settings.default_vehicle_mass

func _connect_signals() -> void:
	pass  # Signals are handled internally

func _reset_vehicle_state() -> void:
	velocity = Vector2.ZERO
	angular_velocity = 0.0
	speed = 0.0
	current_gear = Gear.NEUTRAL
	engine_rpm = 0.0
	target_rpm = 0.0
	clutch_engaged = false
	handbrake_active = false
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	gear_shift_request = Gear.NEUTRAL
	drift_mode_active = false
	drift_angle = 0.0
	traction_control_on = true
	slip_ratio = 0.0
	lateral_slip_angle = 0.0
	
	_front_left_wheel_force = 0.0
	_front_right_wheel_force = 0.0
	_rear_left_wheel_force = 0.0
	_rear_right_wheel_force = 0.0
	
	emit_signal("speed_changed", speed)
	emit_signal("gear_changed", -1, current_gear)
	emit_signal("traction_control_active", traction_control_on)

# ============================================================================
# MAIN GAME LOOP
# ============================================================================

func _process(delta: float) -> void:
	if delta <= 0.0:
		return
	
	# Handle gear shifts first
	_handle_gear_shifts(delta)
	
	# Update engine RPM based on gear and speed
	_update_engine_rpm(delta)
	
	# Get updated input values
	_update_input_values()
	
	# Calculate wheel forces based on input and current state
	_calculate_wheel_forces(delta)
	
	# Apply physics forces to vehicle
	_apply_physics_forces(delta)
	
	# Update vehicle kinematics
	_update_kinematics(delta)
	
	# Check and update drift state
	_update_drift_state()
	
	# Emit signals for changes
	_emit_state_changes()

func _physics_process(delta: float) -> void:
	# High-frequency physics updates (should match physics_tick_rate)
	if _physics_settings != null:
		var fixed_delta = 1.0 / _physics_settings.physics_tick_rate
		if delta > fixed_delta:
			delta = fixed_delta
	
	# Sub-stepping for more accurate physics
	for substep in _physics_settings.max_substeps:
		_sub_step_physics(fixed_delta)

func _sub_step_physics(dt: float) -> void:
	# Sub-step collision detection and response
	_check_collisions(dt)
	_resolve_collision_response(dt)
	
	# Additional high-frequency updates here
	pass

# ============================================================================
# INPUT HANDLING
# ============================================================================

func _update_input_values() -> void:
	# This should be replaced with actual InputManager calls
	# For now, we'll check local input directly
	
	# Throttle input (W, Up Arrow, or positive Z-axis)
	if Input.is_action_pressed("throttle_up"):
		throttle_input = clamp(throttle_input + 5.0 * Engine.get_process_delta_time(), 0.0, 1.0)
	elif Input.is_action_pressed("throttle_down"):
		throttle_input = max(throttle_input - 5.0 * Engine.get_process_delta_time(), 0.0)
	else:
		# Auto-return to neutral when no key pressed
		throttle_input = lerp(throttle_input, 0.0, 5.0 * Engine.get_process_delta_time())
	
	# Brake input (S, Down Arrow, Space, or negative Z-axis)
	if Input.is_action_pressed("brake"):
		brake_input = clamp(brake_input + 5.0 * Engine.get_process_delta_time(), 0.0, 1.0)
	elif Input.is_action_pressed("handbrake"):
		handbrake_active = true
	else:
		brake_input = lerp(brake_input, 0.0, 5.0 * Engine.get_process_delta_time())
		handbrake_active = false
	
	# Steering input (A/Left Arrow/D/Right Arrow)
	if Input.is_action_pressed("steer_left"):
		steering_input = min(steering_input + 5.0 * Engine.get_process_delta_time(), 1.0)
	elif Input.is_action_pressed("steer_right"):
		steering_input = max(steering_input - 5.0 * Engine.get_process_delta_time(), -1.0)
	else:
		steering_input = lerp(steering_input, 0.0, 10.0 * Engine.get_process_delta_time())
	
	# Gear shift requests
	if Input.is_action_just_pressed("shift_up"):
		_request_gear_shift(current_gear + 1)
	elif Input.is_action_just_pressed("shift_down"):
		_request_gear_shift(current_gear - 1)
	elif Input.is_action_just_pressed("neutral"):
		_request_gear_shift(Gear.NEUTRAL)

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================

func _request_gear_shift(target_gear: int) -> void:
	# Validate gear request
	if not _is_valid_gear_request(target_gear):
		return
	
	gear_shift_request = target_gear

func _handle_gear_shifts(delta: float) -> void:
	if gear_shift_request == current_gear:
		return
	
	var old_gear = current_gear
	current_gear = gear_shift_request
	
	# Implement smooth gear transition
	if current_gear != Gear.NEUTRAL:
		clutch_engaged = true
	else:
		clutch_engaged = false
	
	emit_signal("gear_changed", old_gear, current_gear)
	
	# Reset shift request after successful change
	if current_gear == gear_shift_request:
		gear_shift_request = Gear.NEUTRAL

func _is_valid_gear_request(requested_gear: int) -> bool:
	# Prevent invalid transitions
	match requested_gear:
		Gear.NEUTRAL:
			return true
		
		Gear.REVERSE:
			# Can only shift to reverse when stopped
			if speed < 0.5:
				return true
			return false
		
		Gear.FIRST:
			# Can always shift to first
			return true
		
		_:
			# Forward gears: can't skip gears down
			if requested_gear < current_gear - 1:
				return false
			# Can't go up two gears without going through intermediate
			if requested_gear > current_gear + 1:
				return false
			return true
	
	return false

func _update_engine_rpm(delta: float) -> void:
	if current_gear == Gear.NEUTRAL:
		target_rpm = 1000.0  # Idle RPM
		engine_rpm = lerp(engine_rpm, target_rpm, 5.0 * delta)
		return
	
	var gear_ratio = GEAR_RATIOS[current_gear]
	var effective_ratio = gear_ratio * FINAL_DRIVE_RATIO
	
	# Calculate theoretical RPM based on wheel rotation
	var wheel_speed = abs(speed) / (PI * wheel_radius * 2.0)  # Revolutions per second
	var theoretical_rpm = wheel_speed * effective_ratio * 60.0
	
	target_rpm = theoretical_rpm
	
	# Simulate engine inertia and friction
	var rpm_change_rate = 3000.0  # Max RPM change per second
	var target_diff = target_rpm - engine_rpm
	
	if abs(target_diff) < rpm_change_rate * delta:
		engine_rpm = target_rpm
	else:
		engine_rpm += signum(target_diff) * rpm_change_rate * delta
	
	# Clamp RPM limits
	engine_rpm = clamp(engine_rpm, 800.0, 8000.0)

# ============================================================================
# WHEEL FORCE CALCULATION
# ============================================================================

func _calculate_wheel_forces(delta: float) -> void:
	if current_gear == Gear.NEUTRAL:
		_zero_out_wheel_forces()
		return
	
	var gear_ratio = GEAR_RATIOS[current_gear]
	var effective_ratio = gear_ratio * FINAL_DRIVE_RATIO
	
	# Calculate torque at wheels
	var engine_torque = _calculate_engine_torque()
	var wheel_torque = engine_torque * effective_ratio * 0.9  # Transmission efficiency
	
	# Distribute torque based on drivetrain type (RWD for this base implementation)
	var total_drive_force = wheel_torque / wheel_radius
	
	# Apply throttle influence
	var drive_force = total_drive_force * throttle_input
	
	# Apply traction control (reduce drive force if slipping)
	if traction_control_on and slip_ratio > 0.2:
		drive_force *= (1.0 - (slip_ratio - 0.2) * 0.5)
	
	# Apply braking force
	var total_brake_force = MAX_BRAKE_FORCE * brake_input * 0.5  # Brakes are more powerful than acceleration
	if handbrake_active:
		total_brake_force *= 1.5  # Handbrake adds extra braking power
	
	# Distribute forces to wheels
	# Rear-wheel drive configuration
	var rear_force = drive_force - total_brake_force
	var front_force = -total_brake_force * 0.3  # Front brakes also help
	
	# Adjust for steering (reduced grip when turning)
	var steer_factor = 1.0 - abs(steering_input) * 0.2
	
	_rear_left_wheel_force = rear_force * steer_factor * 0.5
	_rear_right_wheel_force = rear_force * steer_factor * 0.5
	_front_left_wheel_force = front_force * steer_factor * 0.5
	_front_right_wheel_force = front_force * steer_factor * 0.5
	
	# Ensure minimum braking even when no brake input
	if speed > 0 and brake_input < 0.1:
		var rolling_resistance = vehicle_mass * 9.81 * rolling_resistance_coefficient
		_rear_left_wheel_force -= rolling_resistance * 0.5
		_rear_right_wheel_force -= rolling_resistance * 0.5

func _calculate_engine_torque() -> float:
	# Simplified engine torque curve
	# Peak torque around 4000-5000 RPM
	var rpm_ratio = (engine_rpm - 1500.0) / (6500.0 - 1500.0)
	rpm_ratio = clamp(rpm_ratio, 0.0, 1.0)
	
	# Torque curve (parabolic peak around middle RPM)
	var torque_curve = 1.0 - pow(rpm_ratio - 0.5, 2) * 4.0
	torque_curve = max(torque_curve, 0.2)  # Minimum torque at idle
	
	return 450.0 * torque_curve  # Peak torque 450 Nm

func _zero_out_wheel_forces() -> void:
	_front_left_wheel_force = 0.0
	_front_right_wheel_force = 0.0
	_rear_left_wheel_force = 0.0
	_rear_right_wheel_force = 0.0

# ============================================================================
# PHYSICS APPLICATION
# ============================================================================

func _apply_physics_forces(delta: float) -> void:
	# Calculate aerodynamic drag
	var drag_force = 0.5 * air_density * drag_coefficient * frontal_area * (speed * speed)
	
	# Calculate rolling resistance
	var rolling_resistance = vehicle_mass * 9.81 * rolling_resistance_coefficient
	
	# Total resistive forces
	var total_resistive = drag_force + rolling_resistance
	
	# Net drive force
	var net_drive_force = (_rear_left_wheel_force + _rear_right_wheel_force + 
		_front_left_wheel_force + _front_right_wheel_force) - total_resistive
	
	# Apply force to velocity (F = ma)
	var acceleration = net_drive_force / vehicle_mass
	
	# Update velocity
	velocity.x += acceleration * cos(get_rotation()) * delta
	velocity.y += acceleration * sin(get_rotation()) * delta
	
	# Apply lateral forces for steering effects
	var lateral_acceleration = 0.0
	if abs(steering_input) > 0.1 and speed > 1.0:
		# Centripetal-like effect from steering
		var turn_radius = wheelbase / tan(steering_input * MAX_STEERING_ANGLE)
		if turn_radius != 0:
			lateral_acceleration = (speed * speed) / turn_radius
			# Apply perpendicular to current heading
			var perp_dir = Vector2(-sin(get_rotation()), cos(get_rotation()))
			velocity += perp_dir * lateral_acceleration * delta * 0.5
	
	# Apply friction/drag to reduce velocity over time
	velocity *= 0.999  # Small natural deceleration

func _update_kinematics(delta: float) -> void:
	# Update position based on velocity
	var displacement = velocity * delta
	
	move_and_slide(displacement)
	
	# Update speed from velocity magnitude
	var old_speed = speed
	speed = velocity.length()
	
	# Update angular velocity based on steering and speed
	if speed > 1.0:
		var turn_angle = steering_input * MAX_STEERING_ANGLE
		var yaw_rate = (speed / wheelbase) * tan(turn_angle)
		angular_velocity = lerp(angular_velocity, yaw_rate, 2.0 * delta)
	else:
		angular_velocity = lerp(angular_velocity, 0.0, 5.0 * delta)
	
	# Apply angular velocity to rotation
	set_rotation(get_rotation() + angular_velocity * delta)
	
	# Store displacement for signal emission
	if displacement.length() > 0.01:
		emit_signal("vehicle_moved", displacement)
	
	# Emit speed change if significant
	if abs(speed - old_speed) > 0.5:
		emit_signal("speed_changed", speed)

# ============================================================================
# DRIFT MECHANICS
# ============================================================================

func _update_drift_state() -> void:
	# Calculate drift angle (difference between facing direction and velocity direction)
	var facing_angle = get_rotation()
	var velocity_angle = velocity.angle()
	
	drift_angle = abs(facing_angle - velocity_angle)
	
	# Normalize angle to [-PI, PI]
	while drift_angle > PI:
		drift_angle -= 2.0 * PI
	while drift_angle < -PI:
		drift_angle += 2.0 * PI
	
	drift_angle = abs(drift_angle)
	
	# Determine if in drift mode
	var was_drifting = drift_mode_active
	drift_mode_active = drift_angle > DRIFT_THRESHOLD
	
	if drift_mode_active != was_drifting:
		emit_signal("drift_angle_changed", drift_angle)
	
	# Update lateral slip angle
	lateral_slip_angle = drift_angle
	
	# Calculate slip ratio (longitudinal wheel slip)
	if current_gear != Gear.NEUTRAL and speed > 0.1:
		var wheel_angular_velocity = speed / wheel_radius
		var wheel_linear_velocity = engine_rpm * 2.0 * PI / 60.0 / GEAR_RATIOS[current_gear] / FINAL_DRIVE_RATIO
		slip_ratio = abs(wheel_angular_velocity - wheel_linear_velocity) / max(abs(wheel_angular_velocity), 0.1)
		slip_ratio = min(slip_ratio, 1.0)
	
	# Traction control adjustment
	var tcs_should_be_on = slip_ratio > TRACTION_CONTROL_SENSITIVITY
	if traction_control_on != tcs_should_be_on:
		traction_control_on = tcs_should_be_on
		emit_signal("traction_control_active", traction_control_on)

# ============================================================================
# COLLISION DETECTION
# ============================================================================

func _check_collisions(dt: float) -> void:
	# Simple AABB collision detection
	# This would be extended with actual collision shapes in a full implementation
	pass

func _resolve_collision_response(dt: float) -> void:
	# Resolve any detected collisions
	# Would apply bounce, friction, etc.
	pass

# ============================================================================
# HELPER METHODS
# ============================================================================

func get_current_gear_name() -> String:
	match current_gear:
		Gear.NEUTRAL: return "Neutral"
		Gear.FIRST: return "1st"
		Gear.SECOND: return "2nd"
		Gear.THIRD: return "3rd"
		Gear.FOURTH: return "4th"
		Gear.FIFTH: return "5th"
		Gear.SIXTH: return "6th"
		Gear.REVERSE: return "Reverse"
		_: return "Unknown"

func get_max_speed_for_gear() -> float:
	"""Calculate theoretical maximum speed for current gear based on redline RPM"""
	if current_gear == Gear.NEUTRAL:
		return 0.0
	
	var gear_ratio = GEAR_RATIOS[current_gear]
	var effective_ratio = gear_ratio * FINAL_DRIVE_RATIO
	var redline_rpm = 7500.0
	
	var max_wheel_rps = redline_rpm / (effective_ratio * 60.0)
	return max_wheel_rps * 2.0 * PI * wheel_radius

func reset_all() -> void:
	_reset_vehicle_state()

func set_drift_mode(enabled: bool) -> void:
	drift_mode_active = enabled
	if enabled:
		# Reduce traction when drifting manually
		traction_control_on = false

func force_stop() -> void:
	velocity = Vector2.ZERO
	angular_velocity = 0.0
	speed = 0.0
	current_gear = Gear.NEUTRAL
	engine_rpm = 1000.0
	_zero_out_wheel_forces()
	emit_signal("speed_changed", 0.0)

# ============================================================================
# DEBUG INFO
# ============================================================================

func get_debug_info() -> Dictionary:
	return {
		"speed": speed,
		"rpm": engine_rpm,
		"gear": current_gear,
		"gear_name": get_current_gear_name(),
		"throttle": throttle_input,
		"brake": brake_input,
		"steering": steering_input,
		"drift_mode": drift_mode_active,
		"drift_angle": drift_angle,
		"slip_ratio": slip_ratio,
		"traction_control": traction_control_on,
		"wheel_forces": {
			"front_left": _front_left_wheel_force,
			"front_right": _front_right_wheel_force,
			"rear_left": _rear_left_wheel_force,
			"rear_right": _rear_right_wheel_force
		},
		"position": global_position,
		"rotation": get_rotation(),
		"velocity": velocity,
		"angular_velocity": angular_velocity
	}

</file_content>