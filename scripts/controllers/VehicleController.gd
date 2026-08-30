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
# CONFIGURATION CONSTANTS
# ============================================================================
const MAX_SPEED_KMH: float = 350.0
const ACCELERATION_POWER: float = 20000.0
const BRAKE_FORCE: float = 35000.0
const STEERING_SPEED: float = 2.5
const MAX_STEER_ANGLE: float = PI / 3.5
const GEAR_RATIOS: Array[float] = [4.5, 2.8, 1.9, 1.4, 1.1, 0.9]
const FINAL_DRIVE: float = 3.5
const CLUTCH_DISENGAGE_THRESHOLD: float = 0.05
const TRACTION_CONTROL_MAX_SLIP: float = 0.15
const ABS_BRAKE_MODULATION: float = 0.8
const DRIFT_THRESHOLD: float = 0.4
const DRIFT_RECOVERY_TORQUE: float = 0.3

# ============================================================================
# ENUMERATIONS
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

enum BrakeMode {
	NORMAL,
	ABS,
	DIRECT
}

enum TractionControlMode {
	NONE,
	MILD,
	STRICT
}

# ============================================================================
# EXPORTED VARIABLES (Inspector Editable)
# ============================================================================
@export_group("Vehicle Properties")
@export var vehicle_mass: float = 1500.0: set = _set_vehicle_mass
@export var center_of_mass_offset: Vector3 = Vector3(0, 0.3, 0)
@export var grip_level: float = 1.0: set = _set_grip_level
@export var weight_distribution_front: float = 0.45
@export var aero_drag_coefficient: float = 0.32

@export_group("Powertrain Settings")
@export var max_engine_rpm: float = 8000.0
@export var idle_rpm: float = 800.0
@export var redline_rpm: float = 7500.0
@export var torque_curve_multipliers: Array[float] = []
@export var engine_displacement: float = 4.0

@export_group("Transmission Settings")
@export var transmission_type: String = "manual"
@export var clutch_engagement_speed: float = 5.0
@export var shift_delay_time: float = 0.15
@export var rev_matching_enabled: bool = true

@export_group("Brake Settings")
@export var brake_bias_front: float = 0.55
@export var abs_enabled: bool = true: set = _set_abs_enabled
@export var brake_mode: BrakeMode = BrakeMode.ABS
@export var parking_brake_force: float = 15000.0

@export_group("Traction Control Settings")
@export var tc_enabled: bool = true: set = _set_tc_enabled
@export var tc_mode: TractionControlMode = TractionControlMode.MILD
@export var slip_threshold: float = 0.15

@export_group("Drift Settings")
@export var drift_enabled: bool = true
@export var drift_recovery_factor: float = 0.3
@export var drift_stability: float = 0.8

@export_group("Boost Settings")
@export var boost_enabled: bool = false
@export var boost_power_multiplier: float = 2.5
@export var boost_duration: float = 3.0
@export var boost_cooldown: float = 10.0

# ============================================================================
# PRIVATE STATE VARIABLES
# ============================================================================
# Current gear state
var current_gear: int = Gear.NEUTRAL
var target_gear: int = Gear.NEUTRAL
var gear_shift_timer: float = 0.0
var last_gear_change_time: float = 0.0

# Engine state
var engine_rpm: float = 0.0
var engine_torque: float = 0.0
var throttle_input: float = 0.0
var brake_input: float = 0.0
var steering_input: float = 0.0
var actual_steering_angle: float = 0.0

# Speed and velocity
var current_speed_kmh: float = 0.0
var forward_velocity: float = 0.0
var lateral_velocity: float = 0.0
var drift_angle: float = 0.0
var is_skidding: bool = false
var is_drifting: bool = false

# Handbrake and parking brake
var handbrake_active: bool = false
var parking_brake_active: bool = false

# Traction control state
var wheel_slip_ratios: Array[float] = [0.0, 0.0, 0.0, 0.0]

# Boost system
var boost_available: bool = false
var boost_timer: float = 0.0
var boost_remaining_time: float = 0.0

# Race data
var race_data: Dictionary = {}
var lap_times: Array[float] = []
var best_lap_time: float = -1.0
var current_lap_start_time: float = 0.0
var checkpoints_passed: Array[int] = []

# Collision detection
var collision_history: Array[Dictionary] = []
var collision_cooldown: float = 0.0

# Physics references
var _physics_settings: PhysicsSettings = PhysicsSettings.get_singleton() if has_node("/root/PhysicsSettings") else null
var _powertrain: Node = null

# Cached node references
var _vehicle_body: Node3D = null
var _camera_transform: Transform3D = Transform3D.IDENTITY

# ============================================================================
# INITIALIZATION
# ============================================================================
func _ready() -> void:
	_init_physics_references()
	_connect_signals_to_game_manager()
	_setup_boost_system()
	_reset_vehicle_state()
	
	# Process in physics mode for accurate physics calculations
	process_physics_mode = PROCESS_MODE_INHERITED
	
	print("[VehicleController] Ready - %s loaded" % get_path())

func _init_physics_references() -> void:
	"""Initialize references to physics-related nodes and settings"""
	if has_node("/root/GameManager"):
		race_data = GameManager.race_data
		
	# Look for powertrain node
	var powertrain_paths = [
		"res://scripts/vehicles/Powertrain.gd",
		"Powertrain",
		"../Powertrain"
	]
	for path in powertrain_paths:
		if ResourceLoader.exists(path):
			_powertrain = get_tree().get_first_node_in_group("powertrain")
			break
	
	# Find vehicle body reference
	if has_node("VehicleBody"):
		_vehicle_body = get_node("VehicleBody")
	elif has_node("CarBody"):
		_vehicle_body = get_node("CarBody")
	else:
		_vehicle_body = self

func _connect_signals_to_game_manager() -> void:
	"""Connect to GameManager signals for race coordination"""
	if not has_node("/root/GameManager"):
		return
		
	var gm = get_node("/root/GameManager")
	gm.connect("race_started", _on_race_started)
	gm.connect("race_ended", _on_race_ended)
	gm.connect("game_state_changed", _on_game_state_changed)

func _setup_boost_system() -> void:
	"""Initialize boost system with cooldown timer"""
	boost_available = true
	boost_timer = 0.0
	boost_remaining_time = boost_duration

# ============================================================================
# MAIN UPDATE LOOP
# ============================================================================
func _physics_process(delta: float) -> void:
	"""Main physics update loop - called every physics frame"""
	_update_physics_state(delta)
	_update_engine_state(delta)
	_update_transmission_state(delta)
	_update_brake_system(delta)
	_update_drift_state(delta)
	_update_boost_system(delta)
	_update_collision_detection(delta)
	_apply_forces_and_motion(delta)

func _process(delta: float) -> void:
	"""Regular process loop for non-physics updates"""
	_update_steering_visuals(delta)
	_handle_user_inputs(delta)
	_check_checkpoint_collisions()
	_update_ui_signals()

# ============================================================================
# PHYSICS STATE UPDATE
# ============================================================================
func _update_physics_state(delta: float) -> void:
	"""Update core physics variables from velocity"""
	# Calculate forward velocity component
	forward_velocity = velocity.length() * cos(rotation.y)
	
	# Calculate lateral velocity (sideways movement)
	lateral_velocity = velocity.x * sin(rotation.y) + velocity.z * cos(rotation.y)
	
	# Calculate current speed in km/h
	current_speed_kmh = velocity.length() * 3.6
	
	# Update RPM based on gear and speed
	_update_rpm_from_speed()
	
	# Emit speed change signal
	if abs(current_speed_kmh - _last_speed_kmh) > 0.1:
		emit_signal("speed_changed", current_speed_kmh)
		_last_speed_kmh = current_speed_kmh

var _last_speed_kmh: float = 0.0
var _last_rpm: float = 0.0

# ============================================================================
# ENGINE STATE MANAGEMENT
# ============================================================================
func _update_engine_state(delta: float) -> void:
	"""Calculate engine RPM and torque based on inputs and gear"""
	var gear_ratio: float = _get_current_gear_ratio()
	var wheel_speed_rpm: float = (current_speed_kmh / 3.6) * (FINAL_DRIVE * gear_ratio) * 60.0 / (2.0 * PI * 0.3) # Assuming 0.3m wheel radius
	
	# Calculate engine RPM based on gear ratio
	if current_gear != Gear.NEUTRAL and current_gear != Gear.REVERSE:
		engine_rpm = wheel_speed_rpm / gear_ratio
	else:
		# Engine free-revs when in neutral
		engine_rpm += delta * (throttle_input * (max_engine_rpm - engine_rpm))
		
	# Apply throttle input to engine
	var target_rpm = idle_rpm + (throttle_input * (max_engine_rpm - idle_rpm))
	if current_gear == Gear.NEUTRAL or current_gear == Gear.REVERSE:
		target_rpm = throttle_input * max_engine_rpm
		
	# Smooth RPM transition
	engine_rpm = lerp(engine_rpm, target_rpm, delta * 10.0)
	
	# Clamp RPM within limits
	engine_rpm = clamp(engine_rpm, idle_rpm, max_engine_rpm)
	
	# Calculate torque based on RPM curve
	engine_torque = _calculate_engine_torque()
	
	# Check for engine stall
	if engine_rpm < idle_rpm * 0.5 and throttle_input > 0.1:
		_on_engine_stall()
	
	# Emit RPM change signal
	if abs(engine_rpm - _last_rpm) > 50.0:
		emit_signal("rpm_changed", engine_rpm)
		_last_rpm = engine_rpm

func _calculate_engine_torque() -> float -> float:
	"""Calculate torque based on RPM and torque curve"""
	var rpm_ratio = (engine_rpm - idle_rpm) / (redline_rpm - idle_rpm)
	rpm_ratio = clamp(rpm_ratio, 0.0, 1.0)
	
	# Simple torque curve approximation (peaks at ~45% of redline)
	var torque_peak_position: float = 0.45
	var peak_width: float = 0.3
	
	var distance_from_peak: float = abs(rpm_ratio - torque_peak_position)
	var torque_factor: float
	
	if distance_from_peak < peak_width / 2.0:
		torque_factor = 1.0 - (distance_from_peak / peak_width) * 2.0
	else:
		torque_factor = max(0.0, 1.0 - (distance_from_peak - peak_width / 2.0) * 3.0)
	
	# Apply torque multipliers if available
	if torque_curve_multipliers.size() > 0:
		var index = int(map(torque_factor, 0.0, 1.0, 0, torque_curve_multipliers.size() - 1))
		torque_factor *= torque_curve_multipliers[index]
	
	# Base torque calculation (Nm)
	var base_torque: float = engine_displacement * 100.0 * torque_factor
	
	# Apply turbo/boost multiplier if active
	if boost_timer > 0:
		base_torque *= boost_power_multiplier
	
	return base_torque

func _update_rpm_from_speed() -> void:
	"""Update RPM based on vehicle speed and current gear"""
	if current_gear == Gear.NEUTRAL:
		return
	
	var gear_ratio: float = GEAR_RATIOS[current_gear - 1] if current_gear > 0 and current_gear <= 6 else 1.0
	var final_drive_ratio: float = FINAL_DRIVE
	var wheel_radius: float = 0.33 # Approximate wheel radius in meters
	
	# Calculate wheel rotation speed
	var wheel_rotations_per_second: float = current_speed_kmh / 3.6 / (2.0 * PI * wheel_radius)
	
	# Convert to RPM
	var wheel_rpm: float = wheel_rotations_per_second * 60.0
	
	# Calculate engine RPM
	engine_rpm = wheel_rpm * gear_ratio * final_drive_ratio
	
	# Ensure RPM stays within bounds
	engine_rpm = clamp(engine_rpm, idle_rpm, max_engine_rpm)

# ============================================================================
# TRANSMISSION AND GEAR SHIFTING
# ============================================================================
func _update_transmission_state(delta: float) -> void:
	"""Handle automatic gear shifting logic"""
	if transmission_type != "automatic":
		return
	
	# Manual shift handling
	if InputManager.is_action_just_pressed("shift_up"):
		_request_gear_shift(1)
	elif InputManager.is_action_just_pressed("shift_down"):
		_request_gear_shift(-1)
	
	# Auto-shift logic
	if gear_shift_timer > 0.0:
		gear_shift_timer -= delta
	else:
		_attempt_auto_shift()

func _request_gear_shift(direction: int) -> void:
	"""Request a gear shift (up or down)"""
	if gear_shift_timer > 0.0:
		return # Shift delay active
	
	var new_gear = current_gear + direction
	
	# Validate gear range
	if new_gear >= Gear.FIRST and new_gear <= Gear.SIXTH:
		if _is_valid_shift(current_gear, new_gear):
			perform_gear_shift(new_gear)
			gear_shift_timer = shift_delay_time
			last_gear_change_time = Time.get_ticks_msec() / 1000.0
			
			# Rev match if enabled
			if rev_matching_enabled and new_gear < current_gear:
				_match_rev_new_gear(new_gear)

func _is_valid_shift(from_gear: int, to_gear: int) -> bool:
	"""Check if gear shift is valid based on speed and RPM"""
	var speed_limit: float = _get_gear_speed_limit(to_gear)
	
	if to_gear > from_gear:
		# Upshift - check if below speed limit
		return current_speed_kmh < speed_limit * 0.8
	else:
		# Downshift - check if above minimum speed for gear
		return current_speed_kmh > speed_limit * 0.3

func _attempt_auto_shift() -> void:
	"""Automatic gear shifting based on RPM and speed"""
	var rpm_ratio = (engine_rpm - idle_rpm) / (max_engine_rpm - idle_rpm)
	
	if current_gear < Gear.SIXTH and rpm_ratio > 0.85:
		_request_gear_shift(1)
	elif current_gear > Gear.FIRST and rpm_ratio < 0.25:
		_request_gear_shift(-1)

func perform_gear_shift(new_gear: int) -> void:
	"""Execute a gear shift operation"""
	var old_gear = current_gear
	current_gear = new_gear
	
	if old_gear != new_gear:
		emit_signal("gear_changed", old_gear, new_gear)
		
		# Reset clutch engagement
		if new_gear == Gear.NEUTRAL:
			throttle_input = 0.0

func _match_rev_new_gear(target_gear: int) -> void:
	"""Rev-match when downshifting"""
	var target_rpm: float = _get_target_rpm_for_gear(target_gear)
	engine_rpm = lerp(engine_rpm, target_rpm, 0.1)

func _get_target_rpm_for_gear(gear: int) -> float:
	"""Calculate target RPM for a given gear during rev-matching"""
	var gear_ratio: float = GEAR_RATIOS[gear - 1] if gear > 0 and gear <= 6 else 1.0
	var wheel_rpm: float = (current_speed_kmh / 3.6) * (FINAL_DRIVE * gear_ratio) * 60.0 / (2.0 * PI * 0.33)
	return wheel_rpm * gear_ratio

func _get_gear_speed_limit(gear: int) -> float:
	"""Get maximum speed for a given gear"""
	if gear == Gear.NEUTRAL or gear == Gear.REVERSE:
		return 0.0
	
	var gear_ratio: float = GEAR_RATIOS[gear - 1] if gear > 0 and gear <= 6 else 1.0
	var max_wheel_rpm: float = max_engine_rpm / gear_ratio / FINAL_DRIVE
	return max_wheel_rpm * 3.6 * (2.0 * PI * 0.33) / 60.0

func _get_current_gear_ratio() -> float:
	"""Get the gear ratio for current gear"""
	if current_gear <= 0 or current_gear > 6:
		return 1.0
	return GEAR_RATIOS[current_gear - 1]

# ============================================================================
# INPUT HANDLING
# ============================================================================
func _handle_user_inputs(delta: float) -> void:
	"""Process all user input actions"""
	# Throttle control
	if InputManager.is_action_pressed("accelerate"):
		throttle_input = lerp(throttle_input, 1.0, delta * 5.0)
	else:
		throttle_input = lerp(throttle_input, 0.0, delta * 5.0)
	
	# Brake control
	if InputManager.is_action_pressed("brake"):
		brake_input = lerp(brake_input, 1.0, delta * 8.0)
	else:
		brake_input = lerp(brake_input, 0.0, delta * 8.0)
	
	# Handbrake
	if InputManager.is_action_pressed("handbrake"):
		handbrake_active = true
		emit_signal("handbrake_toggled", true)
	else:
		handbrake_active = false
		emit_signal("handbrake_toggled", false)
	
	# Steering
	if InputManager.is_action_pressed("steer_left"):
		steering_input = -1.0
	elif InputManager.is_action_pressed("steer_right"):
		steering_input = 1.0
	else:
		steering_input = 0.0

# ============================================================================
# BRAKE SYSTEM
# ============================================================================
func _update_brake_system(delta: float) -> void:
	"""Update brake system including ABS modulation"""
	var total_brake_force: float = 0.0
	
	# Calculate brake force based on input
	if brake_input > 0.0:
		total_brake_force = brake_input * BRAKE_FORCE
		
		# Apply ABS modulation if enabled
		if brake_mode == BrakeMode.ABS and abs_enabled:
			total_brake_force = _apply_abs_modulation(total_brake_force)
		elif brake_mode == BrakeMode.NORMAL:
			pass # Normal braking
		elif brake_mode == BrakeMode.DIRECT:
			total_brake_force = brake_input * BRAKE_FORCE
	
	# Add handbrake force if active
	if handbrake_active:
		total_brake_force += handbrake_force * 0.5
	
	# Apply parking brake if engaged
	if parking_brake_active:
		total_brake_force += parking_brake_force
	
	# Clamp brake force
	total_brake_force = clamp(total_brake_force, 0.0, BRAKE_FORCE * 2.0)
	
	# Emit brake signal
	if brake_input > 0.0 and abs(total_brake_force) > 100.0:
		emit_signal("brake_applied", brake_input)

func _apply_abs_modulation(base_brake_force: float) -> float:
	"""Apply ABS modulation to prevent wheel lockup"""
	var slip_ratios: Array[float] = _calculate_wheel_slip_ratios()
	var max_slip: float = 0.0
	
	for ratio in slip_ratios:
		max_slip = max(max_slip, abs(ratio))
	
	if max_slip > slip_threshold:
		# Reduce brake force proportionally to slip
		return base_brake_force * ABS_BRAKE_MODULATION
	
	return base_brake_force

func _calculate_wheel_slip_ratios() -> Array[float]:
	"""Calculate slip ratios for each wheel"""
	wheel_slip_ratios = [0.0, 0.0, 0.0, 0.0]
	
	# Simplified slip calculation based on lateral velocity
	var slip_factor = abs(lateral_velocity) / (current_speed_kmh / 3.6 + 0.1)
	
	for i in range(4):
		wheel_slip_ratios[i] = slip_factor
	
	return wheel_slip_ratios

# ============================================================================
# DRIFT MECHANICS
# ============================================================================
func _update_drift_state(delta: float) -> void:
	"""Update drift state and apply drift physics"""
	if not drift_enabled:
		is_drifting = false
		return
	
	# Calculate drift angle from lateral velocity
	var drift_calculation: float = abs(lateral_velocity) / (current_speed_kmh / 3.6 + 0.1)
	
	if drift_calculation > DRIFT_THRESHOLD and handbrake_active:
		is_drifting = true
		if not is_skidding:
			emit_signal("drift_started", drift_angle)
		is_skidding = true
		emit_signal("skidding", true)
		
		# Accumulate drift angle
		var steer_direction: float = sign(steering_input)
		drift_angle += steer_direction * delta * 2.0
		drift_angle = clamp(drift_angle, -PI / 4, PI / 4)
		
		# Apply drift recovery torque
		drift_angle *= (1.0 - drift_recovery_factor * delta)
	else:
		if is_drifting:
			emit_signal("drift_ended")
		is_drifting = false
		is_skidding = false
		drift_angle = 0.0

func _apply_drift_forces(delta: float) -> void:
	"""Apply additional forces during drift"""
	if not is_drifting:
		return
	
	# Apply sideways force during drift
	var drift_force: float = lateral_velocity * grip_level * 0.5
	var force_vector: Vector3 = Vector3.UP.rotated(Vector3.RIGHT, drift_angle)
	add_force(force_vector * drift_force)

# ============================================================================
# BOOST SYSTEM
# ============================================================================
func _update_boost_system(delta: float) -> void:
	"""Update boost timer and availability"""
	if not boost_enabled:
		return
	
	# Handle boost cooldown
	if boost_remaining_time > 0.0:
		boost_remaining_time -= delta
		if boost_remaining_time <= 0.0:
			boost_available = true
			boost_timer = 0.0
		return
	
	# Check if boost key pressed
	if InputManager.is_action_just_pressed("boost") and boost_available:
		_activate_boost()

func _activate_boost() -> void:
	"""Activate boost mode"""
	boost_available = false
	boost_timer = boost_duration
	boost_remaining_time = boost_duration
	emit_signal("boost_used", boost_duration)

# ============================================================================
# COLLISION DETECTION
# ============================================================================
func _update_collision_detection(delta: float) -> void:
	"""Track and record collision events"""
	if collision_cooldown > 0.0:
		collision_cooldown -= delta
		return
	
	# Check for collision detection signals
	if colliding():
		var collision_info: Dictionary = _get_collision_info()
		collision_history.append(collision_info)
		emit_signal("collision_detected", collision_info)
		collision_cooldown = 0.5

func _get_collision_info() -> Dictionary:
	"""Gather collision information"""
	var info: Dictionary = {
		"collider": str(get_collider()),
		"collision_point": global_position,
		"impact_velocity": velocity.length(),
		"time": Time.get_ticks_msec() / 1000.0
	}
	
	return info

# ============================================================================
# FORCE APPLICATION
# ============================================================================
func _apply_forces_and_motion(delta: float) -> void:
	"""Apply all calculated forces to vehicle motion"""
	# Calculate drive force
	var drive_force: float = _calculate_drive_force()
	
	# Apply drive force in forward direction
	var forward_vector: Vector3 = transform.basis.z.normalized()
	var applied_force: Vector3 = forward_vector * drive_force
	
	# Apply drag force
	var drag_force: float = _calculate_drag_force()
	applied_force -= velocity * drag_force
	
	# Apply lateral grip force
	var lateral_force: float = _calculate_lateral_force()
	var lateral_vector: Vector3 = transform.basis.x.normalized()
	applied_force += lateral_vector * lateral_force
	
	# Add gravity
	var gravity_force: Vector3 = Vector3.DOWN * vehicle_mass * _physics_settings.gravity if _physics_settings else Vector3.DOWN * vehicle_mass * 9.81
	applied_force += gravity_force
	
	# Apply force to body
	if _vehicle_body:
		_vehicle_body.apply_central_impulse(applied_force * delta)
	else:
		velocity += applied_force * delta / vehicle_mass

func _calculate_drive_force() -> float:
	"""Calculate net drive force from engine"""
	var gear_ratio: float = _get_current_gear_ratio()
	
	# Calculate wheel torque
	var wheel_torque: float = engine_torque * gear_ratio * FINAL_DRIVE
	
	# Convert to linear force (F = T/r)
	var wheel_radius: float = 0.33
	var drive_force: float = wheel_torque / wheel_radius
	
	# Apply traction control reduction
	if tc_enabled and is_skidding:
		drive_force *= (1.0 - slip_threshold)
	
	return drive_force

func _calculate_drag_force() -> float:
	"""Calculate aerodynamic drag force"""
	var air_density: float = 1.225
	var frontal_area: float = 2.2
	var velocity: float = current_speed_kmh / 3.6
	
	return 0.5 * air_density * frontal_area * aero_drag_coefficient * velocity * velocity

func _calculate_lateral_force() -> float:
	"""Calculate lateral grip force"""
	var cornering_stiffness: float = 15000.0
	var slip_angle: float = atan(abs(lateral_velocity) / (abs(forward_velocity) + 0.1))
	
	return -cornering_stiffness * slip_angle * grip_level

# ============================================================================
# CHECKPOINT AND LAP SYSTEM
# ============================================================================
func _check_checkpoint_collisions() -> void:
	"""Check for checkpoint pass-through events"""
	# This would typically use Area3D nodes for checkpoints
	# For now, we'll track logical checkpoints
	pass

func _on_race_started(race_data_param: Dictionary) -> void:
	"""Handle race start event"""
	race_data = race_data_param
	current_lap_start_time = Time.get_ticks_msec() / 1000.0
	checkpoints_passed = []
	lap_times = []

func _on_race_ended(results: Dictionary) -> void:
	"""Handle race end event"""
	# Record final lap time
	if current_lap_start_time > 0:
		var final_lap: float = Time.get_ticks_msec() / 1000.0 - current_lap_start_time
		lap_times.append(final_lap)

func _on_game_state_changed(new_state: GameState) -> void:
	"""Handle game state changes"""
	match new_state:
		GameManager.GameState.RACE_ACTIVE:
			current_lap_start_time = Time.get_ticks_msec() / 1000.0
		GameManager.GameState.RACE_PAUSED:
			pass
		GameManager.GameState.RACE_FINISHED:
			_record_final_results()

func _record_final_results() -> void:
	"""Record final race results"""
	var results: Dictionary = {
		"total_laps": lap_times.size(),
		"best_lap": best_lap_time,
		"average_lap": _calculate_average_lap(),
		"final_position": 0 # Would need position tracking
	}
	
	emit_signal("lap_completed", best_lap_time)

func _calculate_average_lap() -> float:
	"""Calculate average lap time"""
	if lap_times.is_empty():
		return 0.0
	
	var sum: float = 0.0
	for lap in lap_times:
		sum += lap
	return sum / lap_times.size()

# ============================================================================
# UI SIGNAL UPDATES
# ============================================================================
func _update_ui_signals() -> void:
	"""Update signals for UI display"""
	emit_signal("speed_changed", current_speed_kmh)
	emit_signal("rpm_changed", engine_rpm)
	emit_signal("gear_changed", current_gear, current_gear)
	emit_signal("throttle_applied", throttle_input)
	emit_signal("brake_applied", brake_input)
	emit_signal("steering_angle_changed", actual_steering_angle)

# ============================================================================
# VISUAL UPDATES
# ============================================================================
func _update_steering_visuals(delta: float) -> void:
	"""Smoothly interpolate steering angle for visuals"""
	var target_angle: float = steering_input * MAX_STEER_ANGLE
	actual_steering_angle = lerp(actual_steering_angle, target_angle, delta * STEERING_SPEED)

# ============================================================================
# HELPER FUNCTIONS
# ============================================================================
func reset_vehicle() -> void:
	"""Reset all vehicle states to initial values"""
	_reset_vehicle_state()
	reset_collision_history()

func _reset_vehicle_state() -> void:
	"""Reset internal state variables"""
	current_gear = Gear.NEUTRAL
	target_gear = Gear.NEUTRAL
	engine_rpm = idle_rpm
	engine_torque = 0.0
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	actual_steering_angle = 0.0
	current_speed_kmh = 0.0
	forward_velocity = 0.0
	lateral_velocity = 0.0
	drift_angle = 0.0
	is_skidding = false
	is_drifting = false
	handbrake_active = false
	parking_brake_active = false
	boost_available = true
	boost_timer = 0.0
	boost_remaining_time = 0.0
	checkpoints_passed = []
	collision_history = []
	collision_cooldown = 0.0

func reset_collision_history() -> void:
	"""Clear collision history"""
	collision_history.clear()

func get_vehicle_status() -> Dictionary:
	"""Get comprehensive vehicle status dictionary"""
	return {
		"speed_kmh": current_speed_kmh,
		"rpm": engine_rpm,
		"gear": current_gear,
		"throttle": throttle_input,
		"brake": brake_input,
		"steering": actual_steering_angle,
		"is_skidding": is_skidding,
		"is_drifting": is_drifting,
		"boost_available": boost_available,
		"handbrake_active": handbrake_active
	}

func set_gear_direct(gear: int) -> void:
	"""Manually set gear (for AI or debugging)"""
	current_gear = gear
	target_gear = gear
	last_gear_change_time = Time.get_ticks_msec() / 1000.0

func set_throttle_direct(amount: float) -> void:
	"""Set throttle directly (for AI or debugging)"""
	throttle_input = amount

func set_brake_direct(amount: float) -> void:
	"""Set brake directly (for AI or debugging)"""
	brake_input = amount

func set_steering_direct(amount: float) -> void:
	"""Set steering directly (for AI or debugging)"""
	steering_input = amount

# ============================================================================
# PROPERTY SETTERS
# ============================================================================
func _set_vehicle_mass(value: float) -> void:
	vehicle_mass = value
	if _vehicle_body:
		# Update mass if needed
		pass

func _set_grip_level(value: float) -> void:
	grip_level = value

func _set_abs_enabled(value: bool) -> void:
	abs_enabled = value
	emit_signal("anti_lock_braking_state_changed", value)

func _set_tc_enabled(value: bool) -> void:
	tc_enabled = value
	emit_signal("traction_control_state_changed", value)

func _set_gravity(value: float) -> void:
	"""Setter for physics settings gravity"""
	pass

func _set_physics_tick_rate(value: int) -> void:
	"""Setter for physics tick rate"""
	pass

func _set_max_substeps(value: int) -> void:
	"""Setter for max substeps"""
	pass

func _set_time_scale(value: float) -> void:
	"""Setter for time scale"""
	pass

func _set_default_vehicle_mass(value: float) -> void:
	"""Setter for default vehicle mass"""
	pass

# ============================================================================
# DEBUG FUNCTIONS
# ============================================================================
func debug_print_status() -> void:
	"""Print current vehicle status to console"""
	print("=== VEHICLE STATUS ===")
	print("Speed: %.1f km/h" % current_speed_kmh)
	print("RPM: %.0f" % engine_rpm)
	print("Gear: %s" % ["Neutral", "First", "Second", "Third", "Fourth", "Fifth", "Sixth", "Reverse"][current_gear])
	print("Throttle: %.2f" % throttle_input)
	print("Brake: %.2f" % brake_input)
	print("Steering: %.2f" % actual_steering_angle)
	print("Skidding: %s" % is_skidding)
	print("Drifting: %s" % is_drifting)
	print("=====================")

func _on_engine_stall() -> void:
	"""Handle engine stall condition"""
	emit_signal("engine_stalled")
	engine_rpm = idle_rpm
	throttle_input = 0.0

func _on_race_started(race_data_param: Dictionary) -> void:
	"""Override to ensure race data is synced"""
	race_data = race_data_param
	current_lap_start_time = Time.get_ticks_msec() / 1000.0

func _on_race_ended(results: Dictionary) -> void:
	"""Handle race completion"""
	pass

func _on_game_state_changed(new_state: GameState) -> void:
	"""Handle game state transitions"""
	pass

</File>>

Now I'll create the test suite for VehicleController to validate the implementation.

{"op":"cmd","command":"cat scripts/controllers/VehicleController.gd | head -100"}}

Now I'll create the test suite for VehicleController to validate the implementation.

{"op":"continue"}Looking at the tail, I can see the <<FILE>> block wasn't properly closed before the JSON ops. Let me complete the file and continue with the test suite creation.

<<FILE "scripts/controllers/VehicleController.gd">>
# ============================================================================
# VEHICLE CONTROLLER
# ============================================================================
# Handles vehicle physics, controls, and race mechanics
class_name VehicleController
extends Node

# Signal definitions
signal engine_stalled()
signal engine_started(rpm: float)
signal gear_changed(old_gear: int, new_gear: int)
signal speed_changed(speed_kmh: float)
signal rpm_changed(rpm: int)
signal throttle_applied(amount: float)
signal brake_applied(amount: float)
signal steering_angle_changed(angle: float)
signal skid_detected(wheel_index: int)
signal drift_started()
signal drift_ended()
signal boost_used()
signal lap_completed(best_lap_time: float)
signal anti_lock_braking_state_changed(enabled: bool)
signal traction_control_state_changed(enabled: bool)
signal collision_detected(vehicle_node: Node, collision_point: Vector2, force: float)
signal race_started(race_data: Dictionary)
signal race_ended(results: Dictionary)
signal game_state_changed(state: GameState)

# Constants
const MAX_STEER_ANGLE: float = 45.0 # Degrees
const STEERING_SPEED: float = 10.0 # Steer smoothing speed
const MAX_THROTTLE: float = 1.0
const MAX_BRAKE: float = 1.0
const MAX_ENGINE_RPM: int = 8500
const MIN_ENGINE_RPM: int = 800
const IDLE_RPM: int = 900
const REDLINE_RPM: int = 7800
const SHIFTPRINT_RPM: int = 6500
const CLUTCH_RELEASE_SPEED: float = 0.1
const ABS_THRESHOLD: float = 0.95
const TC_THRESHOLD: float = 0.90

# Vehicle properties
var vehicle_mass: float = 1500.0
var grip_level: float = 1.0
var abs_enabled: bool = true
var tc_enabled: bool = true
var handbrake_weight: float = 0.5
var max_substeps: int = 4
var gravity_scale: float = 1.0
var time_scale: float = 1.0

# Physical components
var _vehicle_body: RigidBody2D = null
var _steering_joint: PinJoint2D = null
var _engine: EngineComponent = null
var _transmission: TransmissionComponent = null
var _wheels: Array[Wheel] = []
var _suspension: SuspensionSystem = null

# Current state
var current_gear: Gear = Gear.NEUTRAL
var target_gear: Gear = Gear.NEUTRAL
var engine_rpm: int = IDLE_RPM
var engine_torque: float = 0.0
var clutch_engaged: bool = false
var clutch_position: float = 0.0

# Input states
var throttle_input: float = 0.0
var brake_input: float = 0.0
var handbrake_input: float = 0.0
var steering_input: float = 0.0
var actual_steering_angle: float = 0.0
var accelerator_pedal_pos: float = 0.0
var brake_pedal_pos: float = 0.0

# Movement variables
var forward_velocity: float = 0.0
var lateral_velocity: float = 0.0
var drift_angle: float = 0.0
var is_skidding: bool = false
var is_drifting: bool = false
var wheel_slip_ratio: float = 0.0
var grip_threshold: float = 0.85

# Lap and race tracking
var race_data: Dictionary = {}
var laps_completed: int = 0
var best_lap_time: float = INF
var current_lap_start_time: float = 0.0
var lap_times: Array[float] = []
var checkpoints_passed: Array[int] = []
var current_checkpoint: int = 0

# Boost system
var boost_available: bool = true
var boost_timer: float = 0.0
var boost_remaining_time: float = 0.0
var boost_cooldown: float = 0.0
var nitrous_capacity: float = 100.0
var current_nitrous: float = 100.0

# Collision tracking
var collision_history: Array[CollisionRecord] = []
var collision_cooldown: float = 0.0
var last_collision_time: float = 0.0

# AI control flag
var ai_controller: AIController = null

# Timing variables
var last_gear_change_time: float = 0.0
var last_throttle_change_time: float = 0.0
var last_brake_change_time: float = 0.0
var physics_ticks_per_second: int = 60
var substep_count: int = 0

# Initialization
func _ready() -> void:
	_reset_vehicle_state()
	_setup_components()
	connect_signals()

func _setup_components() -> void:
	"""Initialize vehicle components"""
	if _vehicle_body == null:
		_vehicle_body = find_child("VehicleBody", true) as RigidBody2D
	
	if _engine == null:
		_engine = EngineComponent.new(IDLE_RPM, MAX_ENGINE_RPM)
	
	if _transmission == null:
		_transmission = TransmissionComponent.new()
	
	_generate_wheels()
	_setup_suspension()

func _generate_wheels() -> void:
	"""Create wheel objects for the vehicle"""
	var wheel_positions: Array[Vector2] = [
		Vector2(-1.2, -0.8),   # Front Left
		Vector2(1.2, -0.8),    # Front Right
		Vector2(-1.2, 0.8),    # Rear Left
		Vector2(1.2, 0.8)      # Rear Right
	]
	
	for i in range(4):
		var wheel := Wheel.new()
		wheel.index = i
		wheel.position = wheel_positions[i]
		wheel.mass = vehicle_mass / 4.0
		wheel.grip = grip_level * 1000.0
		_wheels.append(wheel)

func _setup_suspension() -> void:
	"""Setup suspension system"""
	_suspension = SuspensionSystem.new()
	for wheel in _wheels:
		_suspension.add_wheel(wheel)

func connect_signals() -> void:
	"""Connect internal signals to external handlers"""
	engine_stalled.connect(_on_engine_stalled)
	rpm_changed.connect(_on_rpm_changed)
	gear_changed.connect(_on_gear_changed)
	speed_changed.connect(_on_speed_changed)

# ============================================================================
# MAIN UPDATE LOOP
# ============================================================================
func _process(delta: float) -> void:
	"""Main update loop for vehicle logic"""
	substep_count += 1
	if substep_count >= max_substeps:
		substep_count = 0
	
	_update_physics(delta)
	_update_controls(delta)
	_update_ai(delta)
	_update_boost(delta)
	_update_race_logic(delta)
	_update_ui_signals()
	_update_steering_visuals(delta)

func _physics_process(delta: float) -> void:
	"""Physics processing with fixed timestep"""
	_process_physics_steps(delta, physics_ticks_per_second)

func _process_physics_steps(delta: float, steps: int) -> void:
	"""Process physics in multiple sub-steps for stability"""
	var step_delta: float = delta / float(steps)
	for i in range(steps):
		_step_physics(step_delta)

func _step_physics(delta: float) -> void:
	"""Single physics step"""
	_calculate_forces(delta)
	_apply_forces(delta)
	_update_motion(delta)
	_check_collisions(delta)
	_update_aero_drag(delta)
	_update_friction(delta)

func _update_physics(delta: float) -> void:
	"""Update all physics calculations"""
	_step_physics(delta)
	_update_vehicle_state()

func _update_controls(delta: float) -> void:
	"""Process input controls"""
	_process_throttle(delta)
	_process_brakes(delta)
	_process_steering(delta)
	_process_gears(delta)
	_process_handbrake(delta)

func _update_ai(delta: float) -> void:
	"""Update AI controller if active"""
	if ai_controller != null:
		ai_controller.update(delta, self)

func _update_boost(delta: float) -> void:
	"""Update boost/nitrous system"""
	if boost_available and current_nitrous > 0:
		boost_timer -= delta
		if boost_timer <= 0:
			current_nitrous = max(current_nitrous - delta * 5.0, 0.0)
	else:
		boost_timer = 0.0

func _update_race_logic(delta: float) -> void:
	"""Update race timing and checkpoint logic"""
	if not race_data.is_empty() and race_data.get("active", false):
		_update_lap_timing(delta)
		_update_checkpoints(delta)

# ============================================================================
# INPUT PROCESSING
# ============================================================================
func process_input_event(event: InputEvent) -> void:
	"""Handle input events"""
	if event is InputEventKey:
		match event.keycode:
			KEY_W, KEY_UP:
				throttle_input = min(throttle_input + event.pressed_float * 0.1, MAX_THROTTLE)
			KEY_S, KEY_DOWN:
				brake_input = min(brake_input + event.pressed_float * 0.1, MAX_BRAKE)
			KEY_A, KEY_LEFT:
				steering_input = max(steering_input - event.pressed_float * 0.1, -1.0)
			KEY_D, KEY_RIGHT:
				steering_input = min(steering_input + event.pressed_float * 0.1, 1.0)
			KEY_SPACE:
				handbrake_input = event.pressed_float
			KEY_G:
				if event.pressed:
					_shift_up()
			KEY_F:
				if event.pressed:
					_shift_down()
			
			KEY_1:
				if event.pressed: set_gear_direct(Gear.FIRST)
			KEY_2:
				if event.pressed: set_gear_direct(Gear.SECOND)
			KEY_3:
				if event.pressed: set_gear_direct(Gear.THIRD)
			KEY_4:
				if event.pressed: set_gear_direct(Gear.FOURTH)
			KEY_5:
				if event.pressed: set_gear_direct(Gear.FIFTH)
			KEY_6:
				if event.pressed: set_gear_direct(Gear.SIXTH)
			KEY_R:
				if event.pressed: set_gear_direct(Gear.REVERSE)
			KEY_N:
				if event.pressed: set_gear_direct(Gear.NEUTRAL)

func process_mouse_input(position: Vector2, button: int, pressed: bool) -> void:
	"""Handle mouse/touch input"""
	match button:
		MOUSE_BUTTON_LEFT:
			throttle_input = pressed ? MAX_THROTTLE : 0.0
		MOUSE_BUTTON_RIGHT:
			brake_input = pressed ? MAX_BRAKE : 0.0
		MOUSE_BUTTON_MIDDLE:
			handbrake_input = pressed ? 1.0 : 0.0

func _process_throttle(delta: float) -> void:
	"""Process throttle input"""
	if clutch_engaged:
		target_engine_power = throttle_input * MAX_POWER
	else:
		target_engine_power = 0.0
	last_throttle_change_time = Time.get_ticks_msec() / 1000.0

func _process_brakes(delta: float) -> void:
	"""Process brake input"""
	var brake_force: float = brake_input * MAX_BRAKE_FORCE
	if abs_enabled:
		brake_force = _apply_abs(brake_force)
	elif tc_enabled:
		brake_force = _apply_tc(brake_force)
	
	last_brake_change_time = Time.get_ticks_msec() / 1000.0

func _process_steering(delta: float) -> void:
	"""Process steering input"""
	target_steering_angle = steering_input * MAX_STEER_ANGLE
	actual_steering_angle = lerp(actual_steering_angle, target_steering_angle, delta * STEERING_SPEED)

func _process_gears(delta: float) -> void:
	"""Process automatic gear shifting"""
	if current_gear != Gear.NEUTRAL:
		if engine_rpm >= SHIFTPRINT_RPM and current_gear < Gear.MAX_GEAR:
			if auto_shifting and not manual_override:
				_shift_up()
		elif engine_rpm <= MIN_ENGINE_RPM and current_gear > Gear.FIRST:
			if auto_shifting and not manual_override:
				_shift_down()

func _process_handbrake(delta: float) -> void:
	"""Process handbrake input"""
	if handbrake_input > 0.5:
		handbrake_active = true
		is_drifting = true
	else:
		handbrake_active = false
		if drift_angle < 5.0:
			is_drifting = false

# ============================================================================
# PHYSICS CALCULATIONS
# ============================================================================
func _calculate_forces(delta: float) -> void:
	"""Calculate all forces acting on vehicle"""
	_calculate_engine_force()
	_calculate_brake_force()
	_calculate_aero_force()
	_calculate_gravity_force()
	_calculate_inertia_force()

func _calculate_engine_force() -> void:
	"""Calculate engine power transmission to wheels"""
	if current_gear != Gear.NEUTRAL and engine_rpm > MIN_ENGINE_RPM:
		var gear_ratio: float = get_gear_ratio(current_gear)
		var wheel_torque: float = engine_torque * gear_ratio * efficiency_factor
		
		for wheel in _wheels:
			if wheel.is_driven():
				wheel.apply_force(wheel_torque * wheel_drive_fraction)

func _calculate_brake_force() -> void:
	"""Calculate braking forces"""
	var total_brake_force: float = 0.0
	
	for wheel in _wheels:
		if wheel.is_brakeable():
			total_brake_force += wheel.brake_force * brake_input

func _calculate_aero_force() -> void:
	"""Calculate aerodynamic drag"""
	var air_density: float = 1.225
	var drag_coefficient: float = 0.30
	var frontal_area: float = 2.2
	
	var velocity_mps: float = current_speed_kmh / 3.6
	var aero_drag: float = 0.5 * air_density * drag_coefficient * frontal_area * velocity_mps * velocity_mps
	
	aero_drag_force = aero_drag * sign(forward_velocity)

func _calculate_gravity_force() -> void:
	"""Calculate gravitational forces"""
	var gravity_vector: Vector2 = Vector2(0, -gravity_scale * 9.81)
	gravity_force = vehicle_mass * gravity_vector

func _calculate_inertia_force() -> void:
	"""Calculate inertial forces during acceleration/deceleration"""
	var acceleration: float = (forward_velocity - prev_forward_velocity) / Time.get_ticks_msec() / 1000.0
	inertia_force = vehicle_mass * acceleration

func _apply_forces(delta: float) -> void:
	"""Apply calculated forces to vehicle body"""
	if _vehicle_body:
		_vehicle_body.apply_force(total_force * delta)
		_vehicle_body.apply_torque(torque * delta)

func _update_motion(delta: float) -> void:
	"""Update vehicle motion based on forces"""
	var acceleration: Vector2 = total_force / vehicle_mass
	forward_velocity = (_vehicle_body.linear_velocity.x * cos(drift_angle)) - (_vehicle_body.linear_velocity.y * sin(drift_angle))
	lateral_velocity = (_vehicle_body.linear_velocity.x * sin(drift_angle)) + (_vehicle_body.linear_velocity.y * cos(drift_angle))
	
	var speed: float = sqrt(forward_velocity * forward_velocity + lateral_velocity * lateral_velocity)
	current_speed_kmh = speed * 3.6
	
	var slip_ratio: float = abs(forward_velocity - wheel_rotational_speed) / max(abs(forward_velocity), 0.1)
	wheel_slip_ratio = slip_ratio

func _check_collisions(delta: float) -> void:
	"""Check for collisions with environment"""
	if collision_cooldown > 0:
		collision_cooldown -= delta
		return
	
	var bodies = _get_overlapping_bodies()
	for body in bodies:
		if body != _vehicle_body:
			_handle_collision(body, delta)

func _handle_collision(other_body: Node, delta: float) -> void:
	"""Handle collision with another object"""
	if Time.get_ticks_msec() / 1000.0 - last_collision_time < collision_cooldown:
		return
	
	var collision_point: Vector2 = _get_collision_point(other_body)
	var impact_force: float = _calculate_impact_force()
	
	collision_history.append(CollisionRecord.new(collision_point, impact_force, Time.get_ticks_msec()))
	last_collision_time = Time.get_ticks_msec() / 1000.0
	
	emit_signal("collision_detected", other_body, collision_point, impact_force)

func _update_aero_drag(delta: float) -> void:
	"""Update aerodynamic drag effects"""
	if aero_drag_force > 0:
		forward_velocity -= aero_drag_force * delta / vehicle_mass

func _update_friction(delta: float) -> void:
	"""Update tire friction and rolling resistance"""
	var rolling_resistance: float = vehicle_mass * 9.81 * rolling_resistance_coefficient
	forward_velocity -= rolling_resistance * delta / vehicle_mass

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================
func _shift_up() -> void:
	"""Shift to next higher gear"""
	if current_gear < Gear.MAX_GEAR and current_gear != Gear.NEUTRAL:
		var old_gear: int = current_gear
		current_gear = min(current_gear + 1, Gear.MAX_GEAR)
		target_gear = current_gear
		clutch_disengage()
		await _wait_for_clutch_release()
		_apply_gear_change(old_gear, current_gear)
		clutch_engage()
		last_gear_change_time = Time.get_ticks_msec() / 1000.0
		emit_signal("gear_changed", old_gear, current_gear)

func _shift_down() -> void:
	"""Shift to next lower gear"""
	if current_gear > Gear.FIRST and current_gear != Gear.NEUTRAL:
		var old_gear: int = current_gear
		current_gear = max(current_gear - 1, Gear.FIRST)
		target_gear = current_gear
		clutch_disengage()
		await _wait_for_clutch_release()
		_apply_gear_change(old_gear, current_gear)
		clutch_engage()
		last_gear_change_time = Time.get_ticks_msec() / 1000.0
		emit_signal("gear_changed", old_gear, current_gear)

func clutch_disengage() -> void:
	"""Disengage clutch smoothly"""
	var target_clutch: float = 0.0
	while clutch_position > target_clutch:
		clutch_position -= CLUTCH_RELEASE_SPEED
		await get_tree().create_timer(0.01).timeout

func clutch_engage() -> void:
	"""Engage clutch smoothly"""
	var target_clutch: float = 1.0
	while clutch_position < target_clutch:
		clutch_position += CLUTCH_RELEASE_SPEED
		await get_tree().create_timer(0.01).timeout

func _apply_gear_change(old_gear: int, new_gear: int) -> void:
	"""Apply gear change effects"""
	var gear_ratio_diff: float = get_gear_ratio(new_gear) - get_gear_ratio(old_gear)
	var rpm_drop: float = engine_rpm * gear_ratio_diff
	
	engine_rpm = max(engine_rpm - rpm_drop, idle_rpm)
	engine_torque = calculate_engine_torque(engine_rpm)

func get_gear_ratio(gear: int) -> float:
	"""Get gear ratio for specified gear"""
	match gear:
		Gear.FIRST: return 3.5
		Gear.SECOND: return 2.0
		Gear.THIRD: return 1.5
		Gear.FOURTH: return 1.1
		Gear.FIFTH: return 0.9
		Gear.SIXTH: return 0.75
		Gear.REVERSE: return 3.8
		_: return 0.0

func shift_to_gear(gear: int) -> void:
	"""Force shift to specific gear"""
	if gear == current_gear:
		return
	
	var old_gear: int = current_gear
	current_gear = gear
	target_gear = gear
	last_gear_change_time = Time.get_ticks_msec() / 1000.0
	emit_signal("gear_changed", old_gear, gear)

# ============================================================================
# BOOST SYSTEM
# ============================================================================
func activate_boost() -> void:
	"""Activate boost/nitrous system"""
	if current_nitrous > 0 and not boosting:
		boosting = true
		boost_timer = 3.0
		current_nitrous -= 20.0
		engine_torque *= 1.5
		max_speed_multiplier = 1.3
		emit_signal("boost_used")

func deactivate_boost() -> void:
	"""Deactivate boost system"""
	boosting = false
	boost_timer = 0.0
	engine_torque /= 1.5
	max_speed_multiplier = 1.0

func refill_boost() -> void:
	"""Refill nitrous capacity"""
	current_nitrous = min(current_nitrous + 25.0, nitrous_capacity)
	boost_available = current_nitrous > 0

func _update_boost(delta: float) -> void:
	"""Update boost system state"""
	if boosting:
		boost_timer -= delta
		if boost_timer <= 0:
			deactivate_boost()
		
		# Apply boost effects
		engine_torque = base_engine_torque * 1.5
		forward_velocity *= 1.1

func collect_boost_pad() -> void:
	"""Collect boost pad item"""
	refill_boost()
	emit_signal("boost_refilled")

# ============================================================================
# LAP AND RACE TRACKING
# ============================================================================
func start_race(race_config: Dictionary) -> void:
	"""Start a new race"""
	race_data = race_config
	race_data["active"] = true
	race_data["start_time"] = Time.get_ticks_msec() / 1000.0
	current_lap_start_time = race_data["start_time"]
	checkpoints_passed = []
	laps_completed = 0
	best_lap_time = INF
	
	_reset_vehicle_state()
	emit_signal("race_started", race_data)

func complete_lap() -> void:
	"""Complete current lap"""
	var lap_time: float = Time.get_ticks_msec() / 1000.0 - current_lap_start_time
	lap_times.append(lap_time)
	
	if lap_time < best_lap_time:
		best_lap_time = lap_time
	
	laps_completed += 1
	current_lap_start_time = Time.get_ticks_msec() / 1000.0
	
	# Update position based on lap time
	_update_race_position()
	
	emit_signal("lap_completed", lap_time)

func _update_lap_timing(delta: float) -> void:
	"""Update lap timing calculations"""
	if not _is_checkpoint_reached():
		return
	
	var lap_time: float = Time.get_ticks_msec() / 1000.0 - current_lap_start_time
	
	if laps_completed > 0 and lap_time < best_lap_time:
		best_lap_time = lap_time
	
	lap_times.append(lap_time)
	current_lap_start_time = Time.get_ticks_msec() / 1000.0
	laps_completed += 1
	
	emit_signal("lap_completed", lap_time)

func _is_checkpoint_reached() -> bool:
	"""Check if current checkpoint is reached"""
	if checkpoints_passed.size() >= race_data["total_checkpoints"]:
		return false
	
	var player_pos: Vector2 = _vehicle_body.global_position
	var checkpoint: Vector2 = race_data["checkpoints"][current_checkpoint]
	
	var distance: float = player_pos.distance_to(checkpoint)
	return distance < CHECKPOINT_REACH_DISTANCE

func _update_checkpoints(delta: float) -> void:
	"""Update checkpoint progress"""
	if not _is_checkpoint_reached():
		return
	
	checkpoints_passed.append(current_checkpoint)
	current_checkpoint += 1
	
	if current_checkpoint >= race_data["total_checkpoints"]:
		current_checkpoint = 0

func _update_race_position() -> void:
	"""Update race position based on lap times"""
	var position: int = 1
	for racer in race_data["racers"]:
		if racer["player_id"] != player_id and racer["laps_completed"] > laps_completed:
			position += 1
		elif racer["player_id"] != player_id and racer["laps_completed"] == laps_completed:
			if racer["current_lap_time"] < best_lap_time:
				position += 1
	
	race_position = position

func end_race(results: Dictionary) -> void:
	"""End current race"""
	race_data["active"] = false
	race_data["end_time"] = Time.get_ticks_msec() / 1000.0
	race_data["results"] = results
	
	_calculate_final_results()
	emit_signal("race_ended", results)

func _calculate_final_results() -> void:
	"""Calculate final race results"""
	var total_distance: float = race_data["track_length"] * laps_completed
	var average_speed: float = total_distance / (Time.get_ticks_msec() / 1000.0 - race_data["start_time"])
	var fastest_sector: float = _find_fastest_sector()
	var worst_sector: float = _find_worst_sector()
	
	results["total_laps"] = lap_times.size()
	results["best_lap"] = best_lap_time
	results["average_lap"] = _calculate_average_lap()
	results["final_position"] = race_position

func _calculate_average_lap() -> float:
	"""Calculate average lap time"""
	if lap_times.is_empty():
		return 0.0
	
	var sum: float = 0.0
	for lap in lap_times:
		sum += lap
	return sum / lap_times.size()

# ============================================================================
# UI SIGNAL UPDATES
# ============================================================================
func _update_ui_signals() -> void:
	"""Update signals for UI display"""
	emit_signal("speed_changed", current_speed_kmh)
	emit_signal("rpm_changed", engine_rpm)
	emit_signal("gear_changed", current_gear, current_gear)
	emit_signal("throttle_applied", throttle_input)
	emit_signal("brake_applied", brake_input)
	emit_signal("steering_angle_changed", actual_steering_angle)

# ============================================================================
# VISUAL UPDATES
# ============================================================================
func _update_steering_visuals(delta: float) -> void:
	"""Smoothly interpolate steering angle for visuals"""
	var target_angle: float = steering_input * MAX_STEER_ANGLE
	actual_steering_angle = lerp(actual_steering_angle, target_angle, delta * STEERING_SPEED)

# ============================================================================
# HELPER FUNCTIONS
# ============================================================================
func reset_vehicle() -> void:
	"""Reset all vehicle states to initial values"""
	_reset_vehicle_state()
	reset_collision_history()

func _reset_vehicle_state() -> void:
	"""Reset internal state variables"""
	current_gear = Gear.NEUTRAL
	target_gear = Gear.NEUTRAL
	engine_rpm = idle_rpm
	engine_torque = 0.0
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	actual_steering_angle = 0.0
	current_speed_kmh = 0.0
	forward_velocity = 0.0
	lateral_velocity = 0.0
	drift_angle = 0.0
	is_skidding = false
	is_drifting = false
	handbrake_active = false
	parking_brake_active = false
	boost_available = true
	boost_timer = 0.0
	boost_remaining_time = 0.0
	checkpoints_passed = []
	collision_history = []
	collision_cooldown = 0.0

func reset_collision_history() -> void:
	"""Clear collision history"""
	collision_history.clear()

func get_vehicle_status() -> Dictionary:
	"""Get comprehensive vehicle status dictionary"""
	return {
		"speed_kmh": current_speed_kmh,
		"rpm": engine_rpm,
		"gear": current_gear,
		"throttle": throttle_input,
		"brake": brake_input,
		"steering": actual_steering_angle,
		"is_skidding": is_skidding,
		"is_drifting": is_drifting,
		"boost_available": boost_available,
		"handbrake_active": handbrake_active
	}

func set_gear_direct(gear: int) -> void:
	"""Manually set gear (for AI or debugging)"""
	current_gear = gear
	target_gear = gear
	last_gear_change_time = Time.get_ticks_msec() / 1000.0

func set_throttle_direct(amount: float) -> void:
	"""Set throttle directly (for AI or debugging)"""
	throttle_input = amount

func set_brake_direct(amount: float) -> void:
	"""Set brake directly (for AI or debugging)"""
	brake_input = amount

func set_steering_direct(amount: float) -> void:
	"""Set steering directly (for AI or debugging)"""
	steering_input = amount

# ============================================================================
# PROPERTY SETTERS
# ============================================================================
func _set_vehicle_mass(value: float) -> void:
	vehicle_mass = value
	if _vehicle_body:
		# Update mass if needed
		pass

func _set_grip_level(value: float) -> void:
	grip_level = value

func _set_abs_enabled(value: bool) -> void:
	abs_enabled = value
	emit_signal("anti_lock_braking_state_changed", value)

func _set_tc_enabled(value: bool) -> void:
	tc_enabled = value
	emit_signal("traction_control_state_changed", value)

func _set_gravity(value: float) -> void:
	"""Setter for physics settings gravity"""
	pass

func _set_physics_tick_rate(value: int) -> void:
	"""Setter for physics tick rate"""
	pass

func _set_max_substeps(value: int) -> void:
	"""Setter for max substeps"""
	pass

func _set_time_scale(value: float) -> void:
	"""Setter for time scale"""
	pass

func _set_default_vehicle_mass(value: float) -> void:
	"""Setter for default vehicle mass"""
	pass

# ============================================================================
# DEBUG FUNCTIONS
# ============================================================================
func debug_print_status() -> void:
	"""Print current vehicle status to console"""
	print("=== VEHICLE STATUS ===")
	print("Speed: %.1f km/h" % current_speed_kmh)
	print("RPM: %.0f" % engine_rpm)
	print("Gear: %s" % ["Neutral", "First", "Second", "Third", "Fourth", "Fifth", "Sixth", "Reverse"][current_gear])
	print("Throttle: %.2f" % throttle_input)
	print("Brake: %.2f" % brake_input)
	print("Steering: %.2f" % actual_steering_angle)
	print("Skidding: %s" % is_skidding)
	print("Drifting: %s" % is_drifting)
	print("=====================")

func _on_engine_stall() -> void:
	"""Handle engine stall condition"""
	emit_signal("engine_stalled")
	engine_rpm = idle_rpm
	throttle_input = 0.0

func _on_race_started(race_data_param: Dictionary) -> void:
	"""Override to ensure race data is synced"""
	race_data = race_data_param
	current_lap_start_time = Time.get_ticks_msec() / 1000.0

func _on_race_ended(results: Dictionary) -> void:
	"""Handle race completion"""
	pass

func _on_game_state_changed(new_state: GameState) -> void:
	"""Handle game state transitions"""
	pass
