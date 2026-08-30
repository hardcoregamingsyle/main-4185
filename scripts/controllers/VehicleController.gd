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
signal collision_detected(collision_type: String, force: float)

# ============================================================================
# PHYSICS CONSTANTS - Derived from PhysicsSettings resource
# ============================================================================

const MAX_THROTTLE_FORCE: float = 15000.0      # Newtons - maximum acceleration force
const MAX_BRAKE_FORCE: float = 20000.0         # Newtons - maximum braking force
const MAX_STEERING_ANGLE: float = PI / 3       # 60 degrees max steering
const STEERING_SPEED: float = 4.0              # Radians per second steering rate
const DRIFT_THRESHOLD: float = 0.7             # Sideslip threshold for drift mode
const TRACTION_CONTROL_SENSITIVITY: float = 0.85 # TCS activation threshold
const MIN_RPM_IDLE: float = 800.0              # Idle RPM
const MAX_RPM_REDLINE: float = 7500.0          # Redline RPM
const OPTIMAL_POWER_RPM_START: float = 3500.0  # Start of power band
const OPTIMAL_POWER_RPM_END: float = 6500.0    # End of power band

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
const WHEEL_RADIUS: float = 0.32               # Meters - standard tire radius
const WHEEL_COUNT: int = 4                     # Number of wheels

# ============================================================================
# STATE VARIABLES
# ============================================================================

# Vehicle dynamics state
var current_speed: float = 0.0                  # Current speed in m/s (longitudinal)
var current_rpm: float = MIN_RPM_IDLE           # Current engine RPM
var current_gear: Gear = Gear.NEUTRAL           # Currently selected gear
var target_gear: Gear = Gear.NEUTRAL            # Target gear for shifting
var steering_angle: float = 0.0                 # Current steering angle (radians)
var target_steering_angle: float = 0.0          # Target steering angle
var drift_angle: float = 0.0                    # Current sideslip angle
var drift_state: bool = false                   # Whether drifting

# Input states (normalized -1 to 1)
var throttle_input: float = 0.0                 # 0 to 1
var brake_input: float = 0.0                    # 0 to 1
var clutch_input: float = 0.0                   # 0 to 1
var steering_input: float = 0.0                 # -1 to 1
var handbrake_input: float = 0.0                # 0 to 1

# Force application values
var drive_force: float = 0.0                    # Applied drive force
var brake_force: float = 0.0                    # Applied brake force
var lateral_force: float = 0.0                  # Lateral/steering force

# Physical properties (set by parent vehicle)
var vehicle_mass: float = 1500.0                # Vehicle mass in kg
var inertia_tensor: Vector3 = Vector3(1.0, 1.0, 1.0)  # Inertia factors
var wheel_base: float = 2.5                     # Distance between front/rear axles
var track_width: float = 1.6                    # Distance between left/right wheels

# Traction control system
var traction_control_enabled: bool = true
var wheel_slip_ratio: float = 0.0               # Slip ratio for each wheel
var wheel_lock_threshold: float = 0.25          # Max slip before lockup

# Drift mechanics
var drift_torque_multiplier: float = 1.0        # Multiplier during drift
var grip_recovery_rate: float = 5.0             # Grip recovery when exiting drift

# Collision detection
var _last_collision_time: float = 0.0
var _collision_cooldown: float = 0.5            # Seconds between collision events
var _is_colliding: bool = false

# Reference to vehicle body (set by parent)
var _vehicle_body: Node = null
var _physics_settings: PhysicsSettings = null

# ============================================================================
# INITIALIZATION
# ============================================================================

func _ready() -> void:
	_process_mode = ProcessModeEnum.ALWAYS
	_physics_settings = PhysicsSettings.new() if not PhysicsSettings else ProjectSettings.get_setting("autoload/PhysicsSettings")
	_init_vehicle_references()
	_connect_signals()

func _init_vehicle_references() -> void:
	"""Initialize references to vehicle body and components"""
	var parent = get_parent()
	if parent:
		_vehicle_body = parent
		
		# Get vehicle physical properties
		if parent.has_method("get_vehicle_mass"):
			vehicle_mass = parent.get_vehicle_mass()
		
		if parent.has_method("get_wheel_base"):
			wheel_base = parent.get_wheel_base()
		
		if parent.has_method("get_track_width"):
			truck_width = parent.get_track_width()

func _connect_signals() -> void:
	"""Connect to relevant game signals"""
	if GameManager:
		GameManager.game_state_changed.connect(_on_game_state_changed)

# ============================================================================
# INPUT HANDLING
# ============================================================================

func handle_inputs(delta: float) -> void:
	"""Process all input and update control targets"""
	_throttle_control(delta)
	_brake_control(delta)
	_steering_control(delta)
	_clutch_control(delta)
	_handbrake_control(delta)
	_gear_shifting_control(delta)

func _throttle_control(delta: float) -> void:
	"""Handle throttle input with smooth ramping"""
	throttle_input = clamp(throttle_input + delta * 3.0, 0.0, 1.0)

func _brake_control(delta: float) -> void:
	"""Handle brake input with smooth ramping"""
	brake_input = clamp(brake_input + delta * 4.0, 0.0, 1.0)

func _steering_control(delta: float) -> void:
	"""Handle steering input with smooth angle transitions"""
	target_steering_angle = -steering_input * MAX_STEERING_ANGLE
	
	# Smoothly interpolate steering angle
	steering_angle = lerp(steering_angle, target_steering_angle, delta * STEERING_SPEED)

func _clutch_control(delta: float) -> void:
	"""Handle clutch input for gear changes"""
	clutch_input = clamp(clutch_input + delta * 5.0, 0.0, 1.0)

func _handbrake_control(delta: float) -> void:
	"""Handle handbrake for drifting and parking"""
	handbrake_input = clamp(handbrake_input + delta * 3.0, 0.0, 1.0)

func _gear_shifting_control(delta: float) -> void:
	"""Handle automatic or manual gear shifting"""
	if clutch_input > 0.9:
		_attempt_gear_shift(delta)
	else:
		_reset_gear_shift_buffer()

func _attempt_gear_shift(delta: float) -> void:
	"""Attempt to change gears based on RPM and speed"""
	var shift_threshold: float = 0.05 * delta  # Small delay for stability
	
	if abs(target_gear - current_gear) > shift_threshold:
		if target_gear != current_gear:
			var old_gear = current_gear
			current_gear = target_gear
			emit_signal("gear_changed", old_gear, current_gear)
			
			# Visual/audio feedback for gear change
			_play_gear_change_sfx(old_gear, current_gear)
	
	# Reset clutch after successful shift
	if abs(target_gear - current_gear) < 0.01:
		clutch_input = 0.0

func _reset_gear_shift_buffer() -> void:
	"""Reset gear shift buffer when clutch is released"""
	target_gear = _calculate_optimal_gear()

func _calculate_optimal_gear() -> Gear:
	"""Calculate optimal gear based on current RPM and speed"""
	if current_speed < 5.0:
		return Gear.FIRST
	elif current_speed < 15.0:
		return Gear.SECOND
	elif current_speed < 30.0:
		return Gear.THIRD
	elif current_speed < 50.0:
		return Gear.FOURTH
	elif current_speed < 70.0:
		return Gear.FIFTH
	else:
		return Gear.SIXTH

func _play_gear_change_sfx(old_gear: Gear, new_gear: Gear) -> void:
	"""Play gear change sound effect"""
	if AudioManager:
		var gear_names = ["Neutral", "First", "Second", "Third", "Fourth", "Fifth", "Sixth"]
		var old_name = gear_names[old_gear] if old_gear >= 0 else "Reverse"
		var new_name = gear_names[new_gear] if new_gear >= 0 else "Reverse"
		AudioManager.play_sound("gear_change_{}_to_{}".format(old_name, new_name))

# ============================================================================
# VEHICLE PHYSICS SIMULATION
# ============================================================================

func update_physics(delta: float) -> void:
	"""Main physics update loop for vehicle dynamics"""
	if not _vehicle_body:
		return
	
	_update_engine_rpm(delta)
	_apply_drive_forces(delta)
	_apply_braking_forces(delta)
	_update_drift_state(delta)
	_apply_traction_control(delta)
	_update_vehicle_velocity(delta)

func _update_engine_rpm(delta: float) -> void:
	"""Update engine RPM based on vehicle speed and gear"""
	var wheel_rotation_speed: float = 0.0
	
	if current_speed > 0.0 and current_gear != Gear.NEUTRAL and current_gear != Gear.REVERSE:
		# Calculate wheel rotation speed from vehicle speed
		wheel_rotation_speed = current_speed / WHEEL_RADIUS
		
		# Apply gear ratio to get engine RPM
		var gear_ratio = GEAR_RATIOS[current_gear]
		current_rpm = wheel_rotation_speed * gear_ratio * FINAL_DRIVE_RATIO * 60.0 / (2.0 * PI)
	
	elif current_gear == Gear.REVERSE:
		wheel_rotation_speed = -current_speed / WHEEL_RADIUS
		var gear_ratio = GEAR_RATIOS[Gear.REVERSE]
		current_rpm = abs(wheel_rotation_speed) * gear_ratio * FINAL_DRIVE_RATIO * 60.0 / (2.0 * PI)
	else:
		# Neutral or stopped - idle RPM
		current_rpm = lerp(current_rpm, MIN_RPM_IDLE, delta * 5.0)
	
	# Clamp RPM to valid range
	current_rpm = clamp(current_rpm, MIN_RPM_IDLE, MAX_RPM_REDLINE)
	
	emit_signal("engine_rpm_changed", current_rpm)

func _apply_drive_forces(delta: float) -> void:
	"""Apply drive force based on throttle input and gear"""
	if current_gear == Gear.NEUTRAL:
		drive_force = 0.0
		return
	
	# Calculate effective gear ratio
	var gear_ratio = GEAR_RATIOS[current_gear]
	var total_ratio = gear_ratio * FINAL_DRIVE_RATIO
	
	# Base torque calculation (simplified engine model)
	var base_torque: float = 0.0
	
	if current_rpm < OPTIMAL_POWER_RPM_START:
		# Low RPM - less torque
		base_torque = 300.0 * (current_rpm / OPTIMAL_POWER_RPM_START)
	elif current_rpm > OPTIMAL_POWER_RPM_END:
		# High RPM - reduced torque
		base_torque = 300.0 * ((MAX_RPM_REDLINE - current_rpm) / (MAX_RPM_REDLINE - OPTIMAL_POWER_RPM_END))
	else:
		# Optimal power band - peak torque
		base_torque = 300.0
	
	# Apply throttle multiplier
	var applied_torque = base_torque * throttle_input
	
	# Convert torque to force at wheels
	drive_force = (applied_torque * total_ratio) / WHEEL_RADIUS
	
	# Cap drive force
	drive_force = min(drive_force, MAX_THROTTLE_FORCE)

func _apply_braking_forces(delta: float) -> void:
	"""Apply braking forces based on brake and handbrake input"""
	var primary_brake = brake_input * MAX_BRAKE_FORCE
	var handbrake_brake = handbrake_input * MAX_BRAKE_FORCE * 0.7
	
	brake_force = primary_brake + handbrake_brake
	brake_force = min(brake_force, MAX_BRAKE_FORCE)

func _update_drift_state(delta: float) -> void:
	"""Update drift mechanics and state"""
	var sideslip_factor = abs(steering_input) * handbrake_input
	
	if sideslip_factor > DRIFT_THRESHOLD and current_speed > 10.0:
		if not drift_state:
			drift_state = true
			emit_signal("drift_angle_changed", drift_angle)
		
		drift_angle = lerp(drift_angle, steering_input * PI / 4, delta * grip_recovery_rate)
		drift_torque_multiplier = lerp(drift_torque_multiplier, 0.7, delta * 2.0)
	else:
		if drift_state:
			drift_state = false
			emit_signal("drift_angle_changed", drift_angle)
		
		drift_torque_multiplier = lerp(drift_torque_multiplier, 1.0, delta * grip_recovery_rate)
		drift_angle = lerp(drift_angle, 0.0, delta * grip_recovery_rate)

func _apply_traction_control(delta: float) -> void:
	"""Apply traction control logic to prevent wheel spin"""
	if not traction_control_enabled:
		return
	
	# Simple slip detection based on RPM vs speed
	var expected_rpm = (abs(current_speed) / WHEEL_RADIUS) * GEAR_RATIOS[current_gear] * FINAL_DRIVE_RATIO * 60.0 / (2.0 * PI)
	var slip_ratio = abs(current_rpm - expected_rpm) / current_rpm if current_rpm > 0 else 0
	
	if slip_ratio > TRACTION_CONTROL_SENSITIVITY and throttle_input > 0.5:
		emit_signal("traction_control_active", true)
		
		# Reduce drive force to reduce slip
		drive_force *= (1.0 - (slip_ratio - TRACTION_CONTROL_SENSITIVITY) * 0.5)
	else:
		emit_signal("traction_control_active", false)

func _update_vehicle_velocity(delta: float) -> void:
	"""Update vehicle velocity based on applied forces"""
	if not _vehicle_body or not _physics_settings:
		return
	
	# Calculate net force
	var net_force = drive_force - brake_force
	
	# Apply acceleration (F = ma)
	var acceleration = net_force / vehicle_mass
	
	# Update speed
	current_speed += acceleration * delta
	
	# Apply air resistance (simplified drag)
	var drag_coefficient: float = 0.3
	var air_density: float = 1.225
	var frontal_area: float = 2.2
	var drag_force = 0.5 * air_density * drag_coefficient * frontal_area * current_speed * current_speed
	
	current_speed -= (drag_force / vehicle_mass) * delta
	
	# Clamp speed to reasonable limits
	current_speed = clamp(current_speed, -50.0, 200.0)  # m/s
	
	emit_signal("speed_changed", current_speed)

# ============================================================================
# COLLISION HANDLING
# ============================================================================

func handle_collision(collision_type: String, force: float) -> void:
	"""Handle collision events with proper cooldown and filtering"""
	var time_since_last_collision = Time.get_ticks_msec() / 1000.0 - _last_collision_time
	
	if time_since_last_collision < _collision_cooldown:
		return
	
	_is_colliding = true
	_last_collision_time = Time.get_ticks_msec() / 1000.0
	
	emit_signal("collision_detected", collision_type, force)
	
	# Screen shake and particle effects handled by parent
	
	await get_tree().create_timer(_collision_cooldown).timeout
	_is_colliding = false

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================

func get_current_power_output() -> float:
	"""Calculate current power output in horsepower"""
	var torque: float = drive_force * WHEEL_RADIUS / (GEAR_RATIOS[current_gear] * FINAL_DRIVE_RATIO) if current_gear != Gear.NEUTRAL else 0.0
	var rpm_decimal = current_rpm / 60.0
	var power_watts = torque * rpm_decimal * 2.0 * PI
	return power_watts / 745.7  # Convert watts to horsepower

func get_efficiency_rating() -> float:
	"""Calculate efficiency rating based on RPM position in power band"""
	if current_rpm < OPTIMAL_POWER_RPM_START:
		return 0.3 + 0.7 * (current_rpm / OPTIMAL_POWER_RPM_START)
	elif current_rpm > OPTIMAL_POWER_RPM_END:
		return 1.0 - 0.7 * ((current_rpm - OPTIMAL_POWER_RPM_END) / (MAX_RPM_REDLINE - OPTIMAL_POWER_RPM_END))
	else:
		return 1.0

func reset_vehicle_state() -> void:
	"""Reset all vehicle controller state to initial values"""
	current_speed = 0.0
	current_rpm = MIN_RPM_IDLE
	current_gear = Gear.NEUTRAL
	target_gear = Gear.NEUTRAL
	steering_angle = 0.0
	target_steering_angle = 0.0
	drift_angle = 0.0
	drift_state = false
	throttle_input = 0.0
	brake_input = 0.0
	clutch_input = 0.0
	steering_input = 0.0
	handbrake_input = 0.0
	drive_force = 0.0
	brake_force = 0.0
	lateral_force = 0.0
	wheel_slip_ratio = 0.0

func set_physics_properties(mass: float, wheel_base: float, track_width: float) -> void:
	"""Set physical properties of the vehicle"""
	vehicle_mass = mass
	self.wheel_base = wheel_base
	self.track_width = track_width

func enable_traction_control(enabled: bool) -> void:
	"""Enable or disable traction control system"""
	traction_control_enabled = enabled

# ============================================================================
# GAME EVENT HANDLERS
# ============================================================================

func _on_game_state_changed(new_state: GameManager.GameState) -> void:
	"""Handle game state changes affecting vehicle"""
	match new_state:
		GameManager.GameState.RACE_PAUSED:
			# Pause physics updates
			process_mode = ProcessModeEnum.DISABLED
		GameManager.GameState.RACE_ACTIVE:
			# Resume physics updates
			process_mode = ProcessModeEnum.ALWAYS
		GameManager.GameState.RACE_FINISHED:
			# Reset vehicle state
			reset_vehicle_state()

# ============================================================================
# DEBUG INFORMATION
# ============================================================================

func get_debug_info() -> Dictionary:
	"""Return debug information about vehicle state"""
	return {
		"speed": current_speed,
		"rpm": current_rpm,
		"gear": current_gear,
		"throttle": throttle_input,
		"brake": brake_input,
		"steering": steering_input,
		"drift_state": drift_state,
		"drift_angle": drift_angle,
		"power_output": get_current_power_output(),
		"efficiency": get_efficiency_rating(),
		"is_colliding": _is_colliding
	}