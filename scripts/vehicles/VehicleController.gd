extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulation
## Handles throttle, brake, steering, wheel forces, gear shifting, and vehicle dynamics
## Integrates with PhysicsSettings, Powertrain, and InputManager systems
## Copyright 2026 Thalamus Racing Simulator Project

signal speed_changed(current_speed: float)
signal rpm_changed(rpm: float)
signal gear_changed(old_gear: int, new_gear: int)
signal drift_started()
signal drift_ended()
signal collision_detected(collision_info: Dictionary)
signal lap_completed(lap_data: Dictionary)
signal race_finished(position: int, time: float)
signal engine_stalled()

# ============================================================================
# INPUT VALUES (from InputManager)
# ============================================================================

@export var _throttle_input: float = 0.0: set = _set_throttle_input
@export var _brake_input: float = 0.0: set = _set_brake_input
@export var _steering_input: float = 0.0: set = _set_steering_input
@export var _clutch_input: float = 0.0: set = _set_clutch_input
@export var _handbrake_input: float = 0.0: set = _set_handbrake_input

var _last_throttle: float = 0.0
var _last_brake: float = 0.0
var _last_steering: float = 0.0

# ============================================================================
# VEHICLE PHYSICS STATE
# ============================================================================

var current_speed: float = 0.0  # Speed in m/s
var max_speed: float = 65.0  # Max forward speed m/s
var reverse_speed: float = 20.0  # Max reverse speed m/s
var acceleration: float = 0.0
var deceleration: float = 0.0

var rotation_velocity: float = 0.0  # Yaw rate rad/s
var slip_angle: float = 0.0  # Tire slip angle
var lateral_acceleration: float = 0.0  # G-force lateral
var longitudinal_acceleration: float = 0.0  # G-force longitudinal

# ============================================================================
# GEARBOX & TRANSMISSION
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

var current_gear: int = Gear.NEUTRAL
var target_gear: int = Gear.FIRST
var gear_ratios: Array[float] = [3.8, 2.1, 1.5, 1.1, 0.9, 0.7]
var reverse_ratio: float = 3.5
var final_drive_ratio: float = 4.1
var clutch_engaged: bool = true
var shift_delay_timer: float = 0.0

# ============================================================================
# RPM & ENGINE
# ============================================================================

var current_rpm: float = 0.0
var idle_rpm: float = 800.0
var redline_rpm: float = 7000.0
var optimal_power_rpm: float = 5500.0
var maximum_rpm: float = 8000.0
var torque_curve: Array[float] = []
var power_curve: Array[float] = []

# ============================================================================
# WHEELS & DRIVETRAIN
# ============================================================================

enum DrivetrainType {
	FWD,
	RWD,
	AWD
}

var drivetrain_type: DrivetrainType = DrivetrainType.RWD
var wheel_base: float = 2.7  # meters
var track_width: float = 1.6  # meters
var wheel_radius: float = 0.32  # meters
var tire_friction_coefficient: float = 1.2
var suspension_travel: float = 0.15  # meters
var unsprung_mass: float = 50.0  # kg per wheel

# ============================================================================
# DRIFT & SLIP CONTROL
# ============================================================================

var is_drifting: bool = false
var drift_threshold: float = 0.3  # Slip angle threshold for drift
var drift_recovery_rate: float = 0.95
var grip_recovery_rate: float = 0.98
var drift_force_multiplier: float = 0.6

# ============================================================================
# BRAKING SYSTEM
# ============================================================================

var brake_pressure: float = 0.0
var brake_bias_front: float = 0.6  # Front bias percentage
var brake_bias_rear: float = 0.4   # Rear bias percentage
var abs_active: bool = false
var braking_distance: float = 0.0
var locked_wheels: Array[int] = []  # Wheel indices that are locked

# ============================================================================
# PHYSICS CONSTANTS FROM SETTINGS
# ============================================================================

var _vehicle_mass: float = 1500.0
var _engine_torque: float = 400.0  # Nm
var _aerodynamic_drag: float = 0.35
var _drag_coefficient: float = 0.30
var _frontal_area: float = 2.2  # m²
var _air_density: float = 1.225  # kg/m³

# ============================================================================
# RACE DATA
# ============================================================================

var lap_times: Array[float] = []
var best_lap_time: float = INF
var current_lap_start_time: float = 0.0
var total_race_time: float = 0.0
var race_position: int = 0
var checkpoint_passed: bool = false

# ============================================================================
# INTERNAL VARIABLES
# ============================================================================

var _powertrain_node: Node = null
var _audio_manager: Node = null
var _input_manager: Node = null
var _physics_settings: PhysicsSettings = null
var _delta_time_accumulator: float = 0.0
var _fixed_physics_step: float = 0.00833  # ~120 Hz fixed timestep
var _is_initialized: bool = false

func _ready() -> void:
	_init_systems()
	_setup_vehicle()
	_is_initialized = true

func _init_systems() -> void:
	"""Initialize external system references."""
	if GameManager.has_singleton("GameManager"):
		GameManager.game_state_changed.connect(_on_game_state_changed)
	
	if AudioManager.has_singleton("AudioManager"):
		_audio_manager = get_node_or_null("/root/AudioManager")
	
	if InputManager.has_singleton("InputManager"):
		_input_manager = get_node_or_null("/root/InputManager")
	
	if PhysicsSettings.has_singleton("PhysicsSettings"):
		_physics_settings = get_node_or_null("/root/PhysicsSettings")
		_load_settings_from_singleton()

func _load_settings_from_singleton() -> void:
	"""Load physics settings from singleton if available."""
	if _physics_settings != null:
		_vehicle_mass = _physics_settings.default_vehicle_mass
		_fixed_physics_step = 1.0 / _physics_settings.physics_tick_rate

func _setup_vehicle() -> void:
	"""Setup vehicle-specific configuration."""
	_setup_torque_curve()
	_setup_power_curve()
	
	# Apply initial state
	current_gear = Gear.FIRST
	target_gear = Gear.FIRST
	_current_rpm = idle_rpm
	
	# Connect to powertrain if exists
	var powertrain_path = "res://scripts/vehicles/Powertrain.gd"
	if FileAccess.file_exists(powertrain_path):
		_powertrain_node = get_node_or_null("../Powertrain")

func _setup_torque_curve() -> void:
	"""Setup engine torque curve based on RPM."""
	torque_curve.clear()
	for i in range(100):
		var rpm_percent = float(i) / 100.0
		var normalized_rpm = min(max(rpm_percent * 0.8 + 0.1, 0.1), 1.0)
		
		# Quadratic torque curve peaking at optimal power RPM
		var peak_point = optimal_power_rpm / maximum_rpm
		var torque = _engine_torque * (1.0 - pow(normalized_rpm - peak_point, 2)) * 2.0
		torque = clamp(torque, 0.0, _engine_torque * 1.2)
		
		torque_curve.append(torque)

func _setup_power_curve() -> void:
	"""Setup engine power curve based on RPM."""
	power_curve.clear()
	for i in range(100):
		var rpm_percent = float(i) / 100.0
		var normalized_rpm = min(max(rpm_percent * 0.8 + 0.1, 0.1), 1.0)
		var torque = torque_curve[i] if i < torque_curve.size() else _engine_torque
		
		# Power = Torque * RPM
		var actual_rpm = normalized_rpm * maximum_rpm
		var power = (torque * actual_rpm) / 9549.0  # Convert to kW
		power_curve.append(power)

# ============================================================================
# MAIN UPDATE LOOP
# ============================================================================

func _physics_process(delta: float) -> void:
	"""Fixed timestep physics update for consistent vehicle dynamics."""
	_delta_time_accumulator += delta
	
	while _delta_time_accumulator >= _fixed_physics_step:
		_update_physics(_fixed_physics_step)
		_delta_time_accumulator -= _fixed_physics_step

func _update_physics(delta: float) -> void:
	"""Perform one physics step of vehicle simulation."""
	if not _is_initialized:
		return
	
	# Update input values
	_update_inputs(delta)
	
	# Calculate engine output
	var engine_output = _calculate_engine_output(delta)
	
	# Apply forces to vehicle body
	_apply_forces(engine_output, delta)
	
	# Handle collisions and ground detection
	_handle_collisions(delta)
	
	# Update velocity and position
	_move_and_slide()
	
	# Calculate derived physics metrics
	_calculate_derived_metrics(delta)
	
	# Check for game events
	_check_game_events(delta)

func _update_inputs(delta: float) -> void:
	"""Update and smooth input values."""
	# Smooth throttle input
	var throttle_diff = _throttle_input - _last_throttle
	_last_throttle = lerp(_last_throttle, _throttle_input, 0.1)
	
	# Smooth brake input
	var brake_diff = _brake_input - _last_brake
	_last_brake = lerp(_last_brake, _brake_input, 0.1)
	
	# Smooth steering input
	var steering_diff = _steering_input - _last_steering
	_last_steering = lerp(_last_steering, _steering_input, 0.1)
	
	# Detect rapid changes for events
	if abs(throttle_diff) > 0.5:
		pass  # Could trigger launch control or similar
	
	if abs(brake_diff) > 0.5:
		pass  # Could trigger ABS activation

func _calculate_engine_output(delta: float) -> Dictionary:
	"""Calculate engine torque and power output based on current state."""
	var output: Dictionary = {}
	
	# Determine gear ratio
	var gear_ratio: float = 1.0
	match current_gear:
		Gear.NEUTRAL:
			gear_ratio = 0.0
		Gear.REVERSE:
			gear_ratio = reverse_ratio
		Gear.FIRST:
			gear_ratio = gear_ratios[0]
		Gear.SECOND:
			gear_ratio = gear_ratios[1]
		Gear.THIRD:
			gear_ratio = gear_ratios[2]
		Gear.FOURTH:
			gear_ratio = gear_ratios[3]
		Gear.FIFTH:
			gear_ratio = gear_ratios[4]
		Gear.SIXTH:
			gear_ratio = gear_ratios[5]
	
	output.gear_ratio = gear_ratio
	output.final_drive = final_drive_ratio
	
	# Calculate transmission efficiency
	var transmission_efficiency: float = 0.95 if clutch_engaged else 0.0
	
	# Calculate wheel RPM based on vehicle speed
	var wheel_rpm: float = 0.0
	if current_speed != 0.0:
		wheel_rpm = abs(current_speed) / (2.0 * PI * wheel_radius) * 60.0
	
	# Calculate engine RPM
	if gear_ratio > 0:
		current_rpm = wheel_rpm * gear_ratio * final_drive_ratio
	else:
		# Engine decays to idle when in neutral
		current_rpm = lerp(current_rpm, idle_rpm, 0.05)
	
	output.rpm = current_rpm
	
	# Get torque at current RPM
	var torque_index = int((current_rpm / maximum_rpm) * 99.0)
	torque_index = clamp(torque_index, 0, torque_curve.size() - 1)
	var engine_torque = torque_curve[torque_index]
	
	# Apply driver input effects on torque
	var input_factor: float = 0.0
	if current_gear != Gear.NEUTRAL and current_gear != Gear.REVERSE:
		input_factor = _throttle_input
	elif current_gear == Gear.REVERSE:
		input_factor = -_throttle_input * 0.5
	
	engine_torque *= input_factor
	
	# Reduce torque during upshifts
	if shift_delay_timer > 0:
		engine_torque *= 0.3
	
	output.torque = engine_torque
	output.transmission_efficiency = transmission_efficiency
	
	# Send to powertrain if connected
	if _powertrain_node != null:
		_powertrain_node.emit_signal("engine_output_updated", output)
	
	return output

func _apply_forces(engine_output: Dictionary, delta: float) -> void:
	"""Apply calculated forces to vehicle body."""
	var force_vector: Vector3 = Vector3.ZERO
	
	# Calculate drive force
	var drive_force: float = 0.0
	if clutch_engaged and engine_output.gear_ratio > 0:
		drive_force = (engine_output.torque * engine_output.gear_ratio * 
		               engine_output.final_drive * engine_output.transmission_efficiency) / wheel_radius
	
	# Apply drive force in vehicle forward direction
	if drive_force > 0:
		force_vector.x = drive_force * transform.basis.z.x
		force_vector.y = drive_force * transform.basis.z.y
		force_vector.z = drive_force * transform.basis.z.z
	
	# Apply braking force
	var braking_force: float = 0.0
	if _brake_input > 0:
		var brake_force_per_wheel: float = (_brake_input * 5000.0)  # N per wheel
		braking_force = brake_force_per_wheel * 4.0 * _brake_bias_front
		
		# Apply opposite to motion
		if current_speed > 0:
			force_vector.x -= braking_force * transform.basis.z.x
			force_vector.y -= braking_force * transform.basis.z.y
			force_vector.z -= braking_force * transform.basis.z.z
	
	# Apply aerodynamic drag
	var air_resistance: float = 0.5 * _air_density * _drag_coefficient * _frontal_area * current_speed * current_speed
	if air_resistance > 0 and current_speed > 0:
		force_vector.x -= air_resistance * transform.basis.z.x
		force_vector.y -= air_resistance * transform.basis.z.y
		force_vector.z -= air_resistance * transform.basis.z.z
	
	# Apply friction
	var rolling_resistance: float = _vehicle_mass * 9.81 * 0.015  # ~1.5% rolling resistance
	if current_speed > 0:
		force_vector.x -= rolling_resistance * transform.basis.z.x
		force_vector.y -= rolling_resistance * transform.basis.z.y
		force_vector.z -= rolling_resistance * transform.basis.z.z
	
	# Add force to velocity
	acceleration = force_vector.length() / _vehicle_mass
	var acceleration_vector = force_vector.normalized() * acceleration
	velocity += acceleration_vector * delta
	
	# Clamp speed
	var speed_magnitude = velocity.length()
	if speed_magnitude > max_speed:
		velocity = velocity.normalized() * max_speed
	elif speed_magnitude < reverse_speed and speed_magnitude > 0 and current_gear != Gear.REVERSE:
		velocity = velocity.normalized() * reverse_speed
	
	# Handle reverse
	if current_gear == Gear.REVERSE and _throttle_input > 0:
		velocity = velocity.normalized() * (-reverse_speed)
	
	# Apply steering rotation
	var steer_angle: float = _steering_input * 0.5  # Max ~30 degrees
	var rotation_speed = steer_angle * (abs(current_speed) + 1.0) * 0.1
	global_rotation.y += rotation_speed * delta
	
	# Apply gravity
	velocity.y -= _physics_settings.gravity * delta if _physics_settings else 9.81 * delta
	
	current_speed = abs(velocity.length())

func _handle_collisions(delta: float) -> void:
	"""Handle ground collision and vehicle contact."""
	# Simple ground detection - assume flat surface for now
	# In production, use RayCast3D for terrain height
	var ground_height: float = 0.0
	var ray_result = cast_motion_to_ground()
	
	if ray_result:
		ground_height = ray_result.position.y
	
	# Keep vehicle on ground
	transform.origin.y = max(transform.origin.y, ground_height + suspension_travel)
	
	# Handle slide/drift physics
	_handle_drift(delta)

func cast_motion_to_ground() -> PhysicsTestMotionResult3D:
	"""Cast a ray downwards to find ground position."""
	var space_state = get_world_3d().direct_space_state
	var query = PhysicsRayQueryParameters3D.create(
		transform.origin,
		Vector3(0, -5, 0)
	)
	query.exclude = [self]
	
	var result = space_state.intersect_ray(query)
	return result if result and result.has("position") else null

func _handle_drift(delta: float) -> void:
	"""Simulate drift behavior based on slip angle and inputs."""
	# Calculate slip angle based on steering and lateral velocity
	var lateral_velocity = velocity.cross(transform.basis.z).length()
	slip_angle = atan2(lateral_velocity, current_speed) if current_speed > 0 else 0.0
	
	# Check drift conditions
	is_drifting = false
	if abs(slip_angle) > drift_threshold:
		is_drifting = true
		
		if not drift_started.is_connected(_on_drift_started):
			drift_started.connect(_on_drift_started)
		if not drift_ended.is_connected(_on_drift_ended):
			drift_ended.connect(_on_drift_ended)
		
		# Reduce grip during drift
		var drift_force = drift_force_multiplier * lateral_acceleration
		velocity.x += drift_force * transform.basis.x.x * delta
		velocity.z += drift_force * transform.basis.x.z * delta
	else:
		# Recover grip
		slip_angle *= drift_recovery_rate

func _on_drift_started() -> void:
	"""Handle drift start event."""
	if _audio_manager != null:
		_audio_manager.play_sound("drift_start")

func _on_drift_ended() -> void:
	"""Handle drift end event."""
	if _audio_manager != null:
		_audio_manager.play_sound("drift_end")

func _move_and_slide() -> void:
	"""Move vehicle body with sliding physics."""
	# Use Godot's move_and_slide for basic movement
	# For more complex physics, implement custom physics here
	move_and_slide()

func _calculate_derived_metrics(delta: float) -> void:
	"""Calculate secondary physics metrics from primary values."""
	# Longitudinal acceleration in Gs
	longitudinal_acceleration = acceleration / 9.81
	
	# Lateral acceleration in Gs
	lateral_acceleration = abs(velocity.cross(transform.basis.z).length()) / 9.81
	
	# Rotate velocity to match vehicle heading
	var heading = global_rotation.y
	var forward_x = cos(heading)
	var forward_z = sin(heading)
	
	var vel_forward = velocity.x * forward_x + velocity.z * forward_z
	var vel_sideways = velocity.x * -forward_z + velocity.z * forward_x
	
	velocity.x = vel_forward * forward_x - vel_sideways * forward_z
	velocity.z = vel_forward * forward_z + vel_sideways * forward_x
	
	# Emit signals
	emit_signal("speed_changed", current_speed)
	emit_signal("rpm_changed", current_rpm)

func _check_game_events(delta: float) -> void:
	"""Check for race events and emit appropriate signals."""
	# Check for gear change
	if current_gear != target_gear:
		_attempt_shift(target_gear)
	
	# Check for rev limiter
	if current_rpm >= maximum_rpm * 0.95:
		_trigger_rev_limiter()
	
	# Track lap times
	if GameManager.current_state == GameManager.GameState.RACE_ACTIVE:
		_update_race_timing(delta)

func _attempt_shift(new_gear: int) -> void:
	"""Attempt to shift to specified gear."""
	if shift_delay_timer > 0:
		return
	
	if new_gear == current_gear:
		return
	
	var old_gear = current_gear
	current_gear = new_gear
	
	shift_delay_timer = 0.2  # 200ms shift delay
	clutch_engaged = false
	
	emit_signal("gear_changed", old_gear, new_gear)
	
	# Simulate clutch engagement after delay
	create_timer(0.1, _engage_clutch)

func _engage_clutch() -> void:
	"""Engage clutch after shift delay."""
	clutch_engaged = true
	shift_delay_timer = 0.0

func _trigger_rev_limiter() -> void:
	"""Trigger rev limiter to prevent over-revving."""
	current_rpm = maximum_rpm * 0.98
	# Could reduce throttle or cut ignition here

func _update_race_timing(delta: float) -> void:
	"""Update race timing data."""
	total_race_time += delta
	
	if current_speed > 1.0 and not checkpoint_passed:
		checkpoint_passed = true
		current_lap_start_time = Time.get_ticks_msec() / 1000.0

func _reset_lap() -> void:
	"""Reset lap timing for next lap."""
	if current_lap_start_time > 0:
		var lap_time = Time.get_ticks_msec() / 1000.0 - current_lap_start_time
		lap_times.append(lap_time)
		
		if lap_time < best_lap_time:
			best_lap_time = lap_time
		
		emit_signal("lap_completed", {
			"lap_number": lap_times.size(),
			"time": lap_time,
			"best_lap": best_lap_time
		})
	
	current_lap_start_time = 0.0
	checkpoint_passed = false

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================

func shift_up() -> bool:
	"""Shift to next higher gear."""
	if current_gear >= Gear.SIXTH or current_gear <= Gear.NEUTRAL:
		return false
	
	target_gear = current_gear + 1
	return true

func shift_down() -> bool:
	"""Shift to next lower gear."""
	if current_gear <= Gear.FIRST or current_gear == Gear.NEUTRAL:
		return false
	
	target_gear = current_gear - 1
	return true

func shift_to_gear(gear: int) -> bool:
	"""Shift directly to specified gear."""
	if gear < Gear.FIRST or gear > Gear.SIXTH or gear == Gear.NEUTRAL:
		return false
	
	target_gear = gear
	return true

func auto_shift() -> void:
	"""Automatically select optimal gear based on RPM and speed."""
	var optimal_gear = Gear.FIRST
	
	# Find optimal gear based on RPM
	if current_rpm < idle_rpm * 1.5:
		optimal_gear = max(Gear.FIRST, current_gear - 1)
	elif current_rpm > redline_rpm * 0.9:
		optimal_gear = min(Gear.SIXTH, current_gear + 1)
	else:
		# Stay in current gear
		optimal_gear = current_gear
	
	target_gear = optimal_gear

# ============================================================================
# DRIVER CONTROLS
# ============================================================================

func manual_shift_up() -> void:
	"""Driver-initiated upshift."""
	if shift_up():
		emit_signal("gear_changed", current_gear, target_gear)
		shift_delay_timer = 0.2

func manual_shift_down() -> void:
	"""Driver-initiated downshift."""
	if shift_down():
		emit_signal("gear_changed", current_gear, target_gear)
		shift_delay_timer = 0.2

func manual_shift_to(gear: int) -> void:
	"""Driver-initiated direct gear selection."""
	if shift_to_gear(gear):
		emit_signal("gear_changed", current_gear, target_gear)
		shift_delay_timer = 0.2

func engage_clutch_manually() -> void:
	"""Manually engage/disengage clutch."""
	clutch_engaged = not clutch_engaged

func activate_abs() -> void:
	"""Activate ABS braking system."""
	abs_active = true
	# Implement ABS logic here - modulate brake pressure on locked wheels

# ============================================================================
# VEHICLE STATUS & DIAGNOSTICS
# ============================================================================

func get_vehicle_status() -> Dictionary:
	"""Get comprehensive vehicle status dictionary."""
	return {
		"speed": current_speed,
		"rpm": current_rpm,
		"gear": current_gear,
		"target_gear": target_gear,
		"throttle_input": _throttle_input,
		"brake_input": _brake_input,
		"steering_input": _steering_input,
		"clutch_engaged": clutch_engaged,
		"is_drifting": is_drifting,
		"slip_angle": slip_angle,
		"lateral_acceleration": lateral_acceleration,
		"longitudinal_acceleration": longitudinal_acceleration,
		"brake_pressure": brake_pressure,
		"race_position": race_position,
		"total_race_time": total_race_time,
		"best_lap_time": best_lap_time,
		"current_lap_times": lap_times.duplicate()
	}

func reset_vehicle() -> void:
	"""Reset vehicle to initial state."""
	current_speed = 0.0
	current_rpm = idle_rpm
	current_gear = Gear.FIRST
	target_gear = Gear.FIRST
	clutch_engaged = true
	shift_delay_timer = 0.0
	is_drifting = false
	slip_angle = 0.0
	lateral_acceleration = 0.0
	longitudinal_acceleration = 0.0
	
	velocity = Vector3.ZERO
	global_rotation.y = 0.0
	
	# Reset race data
	lap_times.clear()
	best_lap_time = INF
	current_lap_start_time = 0.0
	total_race_time = 0.0
	race_position = 0
	checkpoint_passed = false

# ============================================================================
# SIGNAL HANDLERS
# ============================================================================

func _on_game_state_changed(new_state: GameState) -> void:
	"""Handle game state changes."""
	match new_state:
		GameState.MAIN_MENU:
			reset_vehicle()
		GameState.RACE_ACTIVE:
			pass  # Continue normal operation
		GameState.RACE_FINISHED:
			_on_race_finished()

func _on_race_finished() -> void:
	"""Handle race completion."""
	emit_signal("race_finished", race_position, total_race_time)

# ============================================================================
# EXPORT SETTER FUNCTIONS
# ============================================================================

func _set_throttle_input(value: float) -> void:
	_throttle_input = clamp(value, 0.0, 1.0)

func _set_brake_input(value: float) -> void:
	_brake_input = clamp(value, 0.0, 1.0)

func _set_steering_input(value: float) -> void:
	_steering_input = clamp(value, -1.0, 1.0)

func _set_clutch_input(value: float) -> void:
	_clutch_input = clamp(value, 0.0, 1.0)
	clutch_engaged = value > 0.5

func _set_handbrake_input(value: float) -> void:
	_handbrake_input = clamp(value, 0.0, 1.0)
	# Handbrake could induce drift by locking rear wheels

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================

func set_vehicle_mass(mass: float) -> void:
	"""Set vehicle mass (for customization or damage)."""
	_vehicle_mass = mass

func set_aerodynamics(drag_coeff: float, frontal_area: float) -> void:
	"""Set aerodynamic properties."""
	_drag_coefficient = drag_coeff
	_frontal_area = frontal_area

func configure_drivetrain(type: DrivetrainType) -> void:
	"""Configure drivetrain type."""
	drivetrain_type = type

func set_max_speed(speed: float) -> void:
	"""Set maximum forward speed."""
	max_speed = speed

func set_reverse_speed(speed: float) -> void:
	"""Set maximum reverse speed."""
	reverse_speed = speed

func get_speed_kmh() -> float:
	"""Convert speed to km/h."""
	return current_speed * 3.6

func get_speed_mph() -> float:
	"""Convert speed to mph."""
	return current_speed * 2.23694

func get_distance_traveled() -> float:
	"""Get total distance traveled."""
	# This would track cumulative distance
	return 0.0  # Placeholder - implement tracking

func calculate_optimal_braking_point(distance_to_turn: float) -> float:
	"""Calculate optimal point to begin braking for a turn."""
	# Simplified braking calculation
	var braking_distance = (current_speed * current_speed) / (2.0 * 9.81 * 0.8)
	return braking_distance

func enable_debug_display(enabled: bool) -> void:
	"""Enable/disable debug information display."""
	debug_mode = enabled

func log_vehicle_data() -> Dictionary:
	"""Log current vehicle data for debugging."""
	return {
		"timestamp": Time.get_unix_time_from_system(),
		"position": global_position,
		"velocity": velocity,
		"rotation": global_rotation,
		"speed_kmh": get_speed_kmh(),
		"rpm": current_rpm,
		"gear": current_gear,
		"inputs": {
			"throttle": _throttle_input,
			"brake": _brake_input,
			"steering": _steering_input
		}
	}

# ============================================================================
# SAVE/LOAD SUPPORT
# ============================================================================

func save_state() -> Dictionary:
	"""Save current vehicle state for persistence."""
	return {
		"speed": current_speed,
		"rpm": current_rpm,
		"gear": current_gear,
		"position": global_position,
		"rotation": global_rotation,
		"velocity": velocity,
		"throttle_input": _throttle_input,
		"brake_input": _brake_input,
		"steering_input": _steering_input,
		"race_position": race_position,
		"total_race_time": total_race_time,
		"best_lap_time": best_lap_time
	}

func load_state(state: Dictionary) -> void:
	"""Restore vehicle state from saved data."""
	if state.has("speed"):
		current_speed = state["speed"]
	if state.has("rpm"):
		current_rpm = state["rpm"]
	if state.has("gear"):
		current_gear = state["gear"]
	if state.has("position"):
		global_position = state["position"]
	if state.has("rotation"):
		global_rotation = state["rotation"]
	if state.has("velocity"):
		velocity = state["velocity"]
	if state.has("race_position"):
		race_position = state["race_position"]
	if state.has("total_race_time"):
		total_race_time = state["total_race_time"]
	if state.has("best_lap_time"):
		best_lap_time = state["best_lap_time"]

# ============================================================================
# DESTRUCTOR
# ============================================================================

func _exit_tree() -> void:
	"""Cleanup when node is removed."""
	if _is_initialized:
		pass  # Cleanup any resources

</file_content>