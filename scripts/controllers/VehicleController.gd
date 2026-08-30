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
signal engine_rpm_changed(rpm: float)
signal clutch_engaged(engaged: bool)

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
@export var wheelbase: float = 2.6           # Distance between front and rear axles (meters)
@export var track_width: float = 1.6         # Width between left and right wheels (meters)

# Physical dimensions
var _width: float = 1.8                      # Vehicle width in pixels
var _height: float = 3.6                     # Vehicle height in pixels
var _chassis_color: Color = Color(0.3, 0.3, 0.3, 1.0)

# Current state tracking
var current_speed: float = 0.0               # km/h
var current_velocity: Vector2 = Vector2.ZERO # m/s
var current_rpm: float = 0.0                 # Engine RPM (0-8000)
var current_gear: Gear = Gear.NEUTRAL        # Current transmission gear
var target_gear: Gear = Gear.NEUTRAL         # Desired gear (for automated shifting)
var steering_input: float = 0.0              # -1.0 (full left) to 1.0 (full right)
var throttle_input: float = 0.0              # 0.0 (no throttle) to 1.0 (full throttle)
var brake_input: float = 0.0                 # 0.0 (no brake) to 1.0 (full brake)
var clutch_input: float = 1.0                # 1.0 (engaged) to 0.0 (disengaged)
var handbrake: bool = false

# Drift and traction state
var is_drifting: bool = false
var drift_angle: float = 0.0                 # Angle between velocity vector and facing direction
var sideslip_angle: float = 0.0              # Lateral slip angle at tires
var traction_control_enabled: bool = true
var abs_enabled: bool = true

# Wheel configuration
var front_wheel_steering_angle: float = 0.0  # Actual steering angle
var rear_wheel_brake_force: float = 0.0      # Rear brake force applied
var front_left_brake_force: float = 0.0
var front_right_brake_force: float = 0.0

# Engine torque curve (RPM -> Torque Nm)
var _engine_torque_curve: Dictionary = {}
var _max_engine_torque: float = 400.0        # Maximum engine torque (Nm)
var _idle_rpm: float = 800.0                 # Idle engine RPM
var _redline_rpm: float = 7500.0             # Redline RPM
var _peak_torque_rpm: float = 4500.0         # RPM where max torque occurs

# Physics simulation state
var _last_position: Vector2 = Vector2.ZERO
var _acceleration: Vector2 = Vector2.ZERO
var _angular_velocity: float = 0.0           # Rotation speed (rad/s)
var _inertia_factor: float = 0.85            # How much momentum carries over

# Time and delta tracking
var _delta_time: float = 0.0
var _last_physics_update: float = 0.0
var _shifting_delay_timer: float = 0.0       # Delay between gear changes

# ============================================================================
# INITIALIZATION AND SETUP
# ============================================================================

func _ready() -> void:
	_process_mode = ProcessModeEnum.ALWAYS
	_init_engine_torque_curve()
	_apply_default_settings()
	_initialize_wheels()
	
	if GameManager.current_state == GameManager.GameState.RACE_ACTIVE:
		_activate_vehicle()

func _init_engine_torque_curve() -> void:
	"""Build a realistic engine torque curve based on RPM"""
	var points: Array[Vector2] = [
		Vector2(0.0, 0.0),
		Vector2(_idle_rpm, 50.0),
		Vector2(_peak_torque_rpm, _max_engine_torque),
		Vector2(_peak_torque_rpm * 1.3, _max_engine_torque * 0.9),
		Vector2(_redline_rpm, _max_engine_torque * 0.6),
		Vector2(_redline_rpm * 1.2, 0.0)
	]
	
	for i in range(points.size()):
		var rpm: float = points[i].x
		var torque: float = points[i].y
		_engine_torque_curve[rpm] = torque

func _apply_default_settings() -> void:
	"""Apply default values from PhysicsSettings if available"""
	if PhysicsSettings.has_singleton():
		var settings: PhysicsSettings = PhysicsSettings.get_singleton()
		if settings.default_vehicle_mass > 0:
			vehicle_mass = settings.default_vehicle_mass

func _initialize_wheels() -> void:
	"""Initialize wheel positions and properties"""
	_front_left_wheel = Vector2(-track_width / 2, -wheelbase / 2)
	_front_right_wheel = Vector2(track_width / 2, -wheelbase / 2)
	_rear_left_wheel = Vector2(-track_width / 2, wheelbase / 2)
	_rear_right_wheel = Vector2(track_width / 2, wheelbase / 2)

# ============================================================================
# MAIN GAME LOOP
# ============================================================================

func _process(delta: float) -> void:
	_delta_time = delta
	
	_handle_input()
	_update_physics(delta)
	_update_visual_state()

func _physics_process(delta: float) -> void:
	"""Physics update called at fixed timestep for consistent simulation"""
	_last_physics_update += delta
	
	if _last_physics_update >= 1.0 / PhysicsSettings.physics_tick_rate:
		_perform_detailed_physics(delta)
		_last_physics_update -= 1.0 / PhysicsSettings.physics_tick_rate

func _perform_detailed_physics(delta: float) -> void:
	"""Detailed physics calculations including friction, aerodynamics, etc."""
	# Calculate actual forces applied
	var drive_force: float = _calculate_drive_force()
	var brake_force: float = _calculate_brake_force()
	var drag_force: float = _calculate_aerodynamic_drag()
	var lateral_force: float = _calculate_lateral_forces()
	
	# Apply forces to vehicle body
	var total_force: float = drive_force - brake_force - drag_force
	_acceleration = (total_force / vehicle_mass) * current_velocity.normalized()
	
	# Update velocity
	current_velocity += _acceleration * delta
	current_velocity = current_velocity.clamp_length(0, 120) # Max 120 m/s (~432 km/h)
	
	# Update position
	var displacement: Vector2 = current_velocity * delta
	position += displacement
	
	# Track movement
	if displacement.length_squared() > 0.01:
		vehicle_moved.emit(displacement)
	
	# Convert velocity to km/h for display
	current_speed = current_velocity.length() * 3.6
	
	# Update RPM based on gear and speed
	_update_engine_rpm()
	
	# Check for drift conditions
	_check_drift_conditions()

# ============================================================================
# INPUT HANDLING
# ============================================================================

func _handle_input() -> void:
	"""Read input from InputManager singleton"""
	var input: Dictionary = InputManager.get_vehicle_inputs()
	
	throttle_input = clamp(input.throttle, 0.0, 1.0)
	brake_input = clamp(input.brake, 0.0, 1.0)
	clutch_input = clamp(input.clutch, 0.0, 1.0)
	handbrake = input.handbrake
	
	# Steering with deadzone
	if abs(input.steer) > 0.1:
		steering_input = lerp(steering_input, input.steer * sign(input.steer), STEERING_SPEED * _delta_time)
	else:
		steering_input = lerp(steering_input, 0.0, STEERING_SPEED * _delta_time)
	
	# Automatic gear shifting when not manual control
	if not input.manual_gearing:
		_auto_shift_gears()

# ============================================================================
# PHYSICS CALCULATIONS
# ============================================================================

func _calculate_drive_force() -> float:
	"""Calculate drive force based on engine torque, gear ratio, and throttle"""
	if clutch_input < 0.1 or current_gear == Gear.NEUTRAL:
		return 0.0
	
	var gear_ratio: float = GEAR_RATIOS[current_gear]
	var effective_ratio: float = gear_ratio * FINAL_DRIVE_RATIO
	
	# Get engine torque at current RPM
	var engine_torque: float = _get_engine_torque(current_rpm)
	
	# Drive force calculation
	var drive_force: float = engine_torque * effective_ratio * 0.1 # Simplified transmission efficiency
	
	# Apply throttle
	drive_force *= throttle_input
	
	# Limit drive force
	drive_force = min(drive_force, MAX_THROTTLE_FORCE)
	
	return drive_force

func _calculate_brake_force() -> float:
	"""Calculate total brake force applied"""
	var total_brake: float = brake_input + (handbrake ? 0.5 : 0.0)
	total_brake = min(total_brake, 1.0)
	
	# ABS modulation
	if abs_enabled and current_speed > 5.0 and total_brake > 0.5:
		total_brake *= _modulate_abs()
	
	# Distribute brake force
	rear_wheel_brake_force = total_brake * MAX_BRAKE_FORCE * 0.6
	front_left_brake_force = total_brake * MAX_BRAKE_FORCE * 0.2
	front_right_brake_force = total_brake * MAX_BRAKE_FORCE * 0.2
	
	return total_brake * MAX_BRAKE_FORCE

func _modulate_abs() -> float:
	"""ABS modulation to prevent wheel lockup"""
	# Simplified ABS: reduce brake force if wheel would lock
	var slip_ratio: float = _calculate_wheel_slip()
	if slip_ratio > 0.2:
		return 0.7 # Reduce brake by 30%
	return 1.0

func _calculate_wheel_slip() -> float:
	"""Calculate wheel slip ratio for ABS/TCS"""
	if current_velocity.length() == 0:
		return 0.0
	
	var wheel_angular_velocity: float = current_rpm / 60.0 * 2.0 * PI
	var wheel_linear_velocity: float = wheelbase * _angular_velocity + current_velocity.x
	
	return abs(wheel_linear_velocity - current_velocity.x) / max(abs(current_velocity.x), 0.1)

func _calculate_aerodynamic_drag() -> float:
	"""Calculate air resistance/drag force"""
	var velocity_squared: float = current_velocity.length_squared()
	var drag_coefficient: float = 0.32  # Typical sports car Cd
	var frontal_area: float = 2.2       # m^2
	var air_density: float = 1.225      # kg/m^3
	
	var drag_force: float = 0.5 * drag_coefficient * frontal_area * air_density * velocity_squared
	return drag_force

func _calculate_lateral_forces() -> float:
	"""Calculate lateral forces during cornering/drifting"""
	sideslip_angle = atan2(current_velocity.y, current_velocity.x) - rotation
	
	if sideslip_angle > PI:
		sideslip_angle -= 2.0 * PI
	elif sideslip_angle < -PI:
		sideslip_angle += 2.0 * PI
	
	var lateral_stiffness: float = 15000.0  # Tire lateral stiffness (N/rad)
	var lateral_force: float = lateral_stiffness * sideslip_angle
	
	# Apply traction control if enabled
	if traction_control_enabled and abs(lateral_force) > TRACTION_CONTROL_SENSITIVITY * MAX_THROTTLE_FORCE:
		lateral_force *= 0.7  # Reduce lateral force
	
	return lateral_force

# ============================================================================
# ENGINE AND GEAR LOGIC
# ============================================================================

func _get_engine_torque(rpm: float) -> float:
	"""Get engine torque at specific RPM using linear interpolation"""
	if rpm <= 0:
		return 0.0
	
	# Find surrounding points in torque curve
	var sorted_rpms: Array[float] = _engine_torque_curve.keys()
	sorted_rpms.sort()
	
	var low_idx: int = 0
	var high_idx: int = sorted_rpms.size() - 1
	
	for i in range(sorted_rpms.size()):
		if sorted_rpms[i] > rpm:
			high_idx = i
			low_idx = i - 1
			break
	
	# Linear interpolation
	if low_idx < 0:
		return _engine_torque_curve[sorted_rpms[0]]
	
	var t: float = (rpm - sorted_rpms[low_idx]) / (sorted_rpms[high_idx] - sorted_rpms[low_idx])
	var low_torque: float = _engine_torque_curve[sorted_rpms[low_idx]]
	var high_torque: float = _engine_torque_curve[sorted_rpms[high_idx]]
	
	return lerp(low_torque, high_torque, t)

func _update_engine_rpm() -> void:
	"""Update engine RPM based on gear and vehicle speed"""
	if current_gear == Gear.NEUTRAL:
		current_rpm = lerp(current_rpm, _idle_rpm, 0.1)
		engine_rpm_changed.emit(current_rpm)
		return
	
	if clutch_input < 0.1:
		current_rpm = _idle_rpm
		engine_rpm_changed.emit(current_rpm)
		return
	
	var gear_ratio: float = GEAR_RATIOS[current_gear]
	var effective_ratio: float = gear_ratio * FINAL_DRIVE_RATIO
	
	# Calculate expected RPM from vehicle speed
	var wheel_radius: float = 0.32  # meters
	var wheel_rps: float = current_velocity.length() / (2.0 * PI * wheel_radius)
	var expected_rpm: float = wheel_rps * effective_ratio * 60.0
	
	# Smooth RPM transition
	current_rpm = lerp(current_rpm, expected_rpm, 0.2)
	
	# Clamp to idle/redline
	current_rpm = clamp(current_rpm, _idle_rpm, _redline_rpm)
	
	engine_rpm_changed.emit(current_rpm)

func _auto_shift_gears() -> void:
	"""Automatically select optimal gear based on RPM and speed"""
	if _shifting_delay_timer > 0:
		_shifting_delay_timer -= _delta_time
		return
	
	var shift_up_threshold: float = _redline_rpm * 0.95
	var shift_down_threshold: float = _idle_rpm * 1.5
	
	# Shift up if at high RPM
	if current_rpm > shift_up_threshold and current_gear != Gear.SIXTH:
		_change_gear(_get_next_higher_gear(), true)
		return
	
	# Shift down if at low RPM and moving
	if current_rpm < shift_down_threshold and current_speed > 2.0:
		_change_gear(_get_next_lower_gear(), false)
		return
	
	# Neutral at standstill
	if current_speed < 1.0 and current_gear != Gear.NEUTRAL:
		_change_gear(Gear.NEUTRAL, true)

func _get_next_higher_gear() -> Gear:
	"""Get next higher gear"""
	match current_gear:
		Gear.FIRST: return Gear.SECOND
		Gear.SECOND: return Gear.THIRD
		Gear.THIRD: return Gear.FOURTH
		Gear.FOURTH: return Gear.FIFTH
		Gear.FIFTH: return Gear.SIXTH
		_: return Gear.NEUTRAL

func _get_next_lower_gear() -> Gear:
	"""Get next lower gear"""
	match current_gear:
		Gear.SIXTH: return Gear.FIFTH
		Gear.FIFTH: return Gear.FOURTH
		Gear.FOURTH: return Gear.THIRD
		Gear.THIRD: return Gear.SECOND
		Gear.SECOND: return Gear.FIRST
		Gear.FIRST: return Gear.NEUTRAL
		_: return Gear.FIRST

func manual_shift(direction: int) -> void:
	"""Manually shift gears (+1 or -1)"""
	if _shifting_delay_timer > 0:
		return
	
	var new_gear: Gear = current_gear + direction
	
	# Validate gear change
	if new_gear < Gear.FIRST or new_gair > Gear.SIXTH:
		return
	
	_change_gear(new_gear, true)

func _change_gear(new_gear: Gear, engage_clutch: bool) -> void:
	"""Change to specified gear with clutch engagement"""
	if new_gear == current_gear:
		return
	
	var old_gear: Gear = current_gear
	
	# Clutch disengagement delay
	if engage_clutch and clutch_input < 0.5:
		return
	
	current_gear = new_gear
	gear_changed.emit(old_gear, new_gear)
	
	# Shifting delay to simulate transmission time
	_shifting_delay_timer = 0.3
	
	# Sound effect
	if AudioManager.has_singleton():
		AudioManager.play_sound("gear_shift")

# ============================================================================
# DRIFT AND TRACTION CONTROL
# ============================================================================

func _check_drift_conditions() -> void:
	"""Check if vehicle is in drift state"""
	var drift_threshold: float = DRIFT_THRESHOLD
	
	is_drifting = abs(sideslip_angle) > drift_threshold and throttle_input > 0.3
	
	# Update drift angle signal
	if abs(drift_angle - sideslip_angle) > 0.01:
		drift_angle = sideslip_angle
		drift_angle_changed.emit(drift_angle)
	
	# Enable/disable traction control based on drift
	if is_drifting and traction_control_enabled:
		traction_control_enabled = false
		traction_control_active.emit(false)
	elif not is_drifting and not traction_control_enabled:
		traction_control_enabled = true
		traction_control_active.emit(true)

# ============================================================================
# VISUAL UPDATES
# ============================================================================

func _update_visual_state() -> void:
	"""Update visual representation of vehicle"""
	# Update steering angle visually
	var target_steering: float = steering_input * MAX_STEERING_ANGLE
	front_wheel_steering_angle = lerp(front_wheel_steering_angle, target_steering, 0.2)
	
	# Animate suspension compression based on vertical acceleration
	# (would be implemented with actual suspension system)
	pass

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================

func get_current_power() -> float:
	"""Calculate current power output (kW)"""
	var torque: float = _get_engine_torque(current_rpm)
	var power_kw: float = (torque * current_rpm) / 9549.0  # Conversion factor
	return power_kw

func get_efficiency() -> float:
	"""Calculate fuel efficiency estimate (km/L)"""
	if current_speed <= 0:
		return 0.0
	
	var base_efficiency: float = 12.0  # Base efficiency at optimal RPM
	var rpm_penalty: float = 1.0
	if current_rpm > _peak_torque_rpm:
		rpm_penalty = 1.0 - (current_rpm - _peak_torque_rpm) / (_redline_rpm - _peak_torque_rpm)
	
	return base_efficiency * rpm_penalty

func reset() -> void:
	"""Reset vehicle to initial state"""
	current_speed = 0.0
	current_velocity = Vector2.ZERO
	current_rpm = _idle_rpm
	current_gear = Gear.NEUTRAL
	steering_input = 0.0
	throttle_input = 0.0
	brake_input = 0.0
	is_drifting = false
	traction_control_enabled = true
	acceleration = Vector2.ZERO
	_angular_velocity = 0.0

func _activate_vehicle() -> void:
	"""Activate vehicle for racing"""
	reset()
	AudioManager.play_sound("engine_start")

func _deactivate_vehicle() -> void:
	"""Deactivate vehicle"""
	_reset_all()

func _reset_all() -> void:
	"""Complete reset of all vehicle systems"""
	reset()
	current_gear = Gear.NEUTRAL
	clutch_input = 1.0

# ============================================================================
# WHEEL ACCESSORS
# ============================================================================

var _front_left_wheel: Vector2
var _front_right_wheel: Vector2
var _rear_left_wheel: Vector2
var _rear_right_wheel: Vector2

func get_wheel_positions() -> Dictionary:
	"""Get positions of all four wheels relative to vehicle center"""
	return {
		"front_left": position + _front_left_wheel.rotated(rotation),
		"front_right": position + _front_right_wheel.rotated(rotation),
		"rear_left": position + _rear_left_wheel.rotated(rotation),
		"rear_right": position + _rear_right_wheel.rotated(rotation)
	}

func get_wheel_forces() -> Dictionary:
	"""Get brake forces applied to each wheel"""
	return {
		"front_left": front_left_brake_force,
		"front_right": front_right_brake_force,
		"rear_left": rear_wheel_brake_force * 0.5,
		"rear_right": rear_wheel_brake_force * 0.5
	}

# ============================================================================
# DEBUG INFO
# ============================================================================

func get_debug_info() -> Dictionary:
	"""Return debug information about vehicle state"""
	return {
		"speed_kmh": round(current_speed * 100) / 100,
		"velocity": current_velocity,
		"rpm": round(current_rpm * 10) / 10,
		"gear": current_gear,
		"throttle": throttle_input,
		"brake": brake_input,
		"steering": steering_input,
		"is_drifting": is_drifting,
		"drift_angle": round(drift_angle * 100) / 100,
		"power_kw": round(get_current_power() * 10) / 10,
		"efficiency": round(get_efficiency() * 100) / 100
	}