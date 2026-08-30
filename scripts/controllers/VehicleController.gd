extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulation
## Handles throttle, brake, steering, gear shifting, wheel forces, and vehicle dynamics
## Integrates with PhysicsSettings singleton for all physics constants
## Copyright 2026 Thalamus Racing Simulator Project

# ============================================================================
# SIGNALS - Events emitted by this controller
# ============================================================================
signal speed_changed(new_speed: float)
signal rpm_changed(new_rpm: float)
signal gear_changed(old_gear: int, new_gear: int)
signal vehicle_moved(position: Vector3, velocity: Vector3)
signal collision_detected(collision_data: Dictionary)
signal engine_started()
signal engine_stopped()
signal handbrake_toggled(is_active: bool)
signal drift_started(angle: float)
signal drift_ended()
signal traction_control_active(active: bool)

# ============================================================================
# ENUMERATIONS & CONSTANTS
# ============================================================================
enum DrivetrainType { FWD, RWD, AWD }
enum GearState { NEUTRAL = 0, REVERSE = -1 }

const VEHICLE_BASE_MASS := 1500.0  # Base mass in kg
const MAX_STEERING_RATE := 90.0    # Degrees per second
const DRIFT_THRESHOLD := 0.7       # Sliding coefficient threshold
const TRACTION_CONTROL_THRESHOLD := 0.85  # Wheel slip threshold

# ============================================================================
# PUBLIC PROPERTIES - Exposed for inspector and external access
# ============================================================================
@export_group("Vehicle Configuration")
@export var vehicle_mass: float = 1500.0: set = _set_vehicle_mass
@export var max_speed_kmh: float = 320.0: set = _set_max_speed_kmh
@export var acceleration_force: float = 8000.0: set = _set_acceleration_force
@export var braking_force: float = 15000.0: set = _set_braking_force
@export var steering_angle_max: float = 45.0: set = _set_steering_angle_max

@export_group("Drivetrain Settings")
@export var drivetrain_type: DrivetrainType = DrivetrainType.RWD
@export var final_drive_ratio: float = 3.5: set = _set_final_drive_ratio
@export var tire_radius: float = 0.33: set = _set_tire_radius
@export var torque_curve: Array[Vector2f] = [
	Vector2f(0.0, 0.0),   # RPM fraction -> Torque multiplier
	Vector2f(0.2, 0.6),
	Vector2f(0.4, 0.9),
	Vector2f(0.6, 1.0),   # Peak torque at 60% RPM
	Vector2f(0.8, 0.95),
	Vector2f(1.0, 0.85),  # Redline torque drop
]: set = _set_torque_curve

@export_group("Gear Settings")
@export var num_gears: int = 6: set = _set_num_gears
@export var gear_ratios: Array[float] = [
	3.8,  # 1st gear
	2.4,  # 2nd gear
	1.8,  # 3rd gear
	1.4,  # 4th gear
	1.1,  # 5th gear
	0.9   # 6th gear
]: set = _set_gear_ratios
@export var reverse_ratio: float = 3.5: set = _set_reverse_ratio
@export var idle_rpm: float = 800.0: set = _set_idle_rpm
@export var redline_rpm: float = 7500.0: set = _set_redline_rpm

@export_group("Handling Characteristics")
@export var grip_coefficient: float = 1.2: set = _set_grip_coefficient
@export var drift_factor: float = 0.3: set = _set_drift_factor
@export var traction_control_enabled: bool = true
@export var anti_roll_bar_stiffness: float = 0.5: set = _set_anti_roll_bar_stiffness

@export_group("Wheel Configuration")
@export var track_width_front: float = 1.5: set = _set_track_width_front
@export var track_width_rear: float = 1.5: set = _set_track_width_rear
@export var wheelbase: float = 2.7: set = _set_wheelbase
@export var center_of_mass_height: float = 0.5: set = _set_center_of_mass_height

# ============================================================================
# PRIVATE STATE VARIABLES
# ============================================================================
# Engine state
var current_rpm: float = 0.0
var engine_power: float = 0.0
var engine_brake: float = 0.0
var clutch_engaged: bool = false
var engine_running: bool = false
var ignition_on: bool = false

# Transmission state
var current_gear: int = 0
var target_gear: int = 0
var shift_timer: float = 0.0
var shift_cooldown: float = 0.5
var last_shift_time: float = 0.0

# Driving inputs
var throttle_input: float = 0.0
var brake_input: float = 0.0
var steering_input: float = 0.0
var handbrake_input: float = 0.0
var nitrous_input: float = 0.0

# Velocity and movement
var road_speed_kmh: float = 0.0
var angular_velocity_z: float = 0.0
var lateral_velocity: float = 0.0
var longitudinal_velocity: float = 0.0
var slip_angle: float = 0.0
var slide_coefficient: float = 0.0

# Drift state
var drifting: bool = false
var drift_angle: float = 0.0
var drift_score: float = 0.0
var drift_multiplier: float = 1.0

# Traction control
var traction_loss: float = 0.0
var wheel_slip_rate: float = 0.0
var wheel_spin: float = 0.0

# Physics components
var powertrain_node: Node = null
var chassis_node: Node3D = null
var wheel_nodes: Array[Node3D] = []
var suspension_nodes: Array[Node] = []

# Tire grip tracking per wheel
var front_left_grip: float = 1.0
var front_right_grip: float = 1.0
var rear_left_grip: float = 1.0
var rear_right_grip: float = 1.0

# Wind resistance coefficients
var drag_coefficient: float = 0.32
var frontal_area: float = 2.2
var air_density: float = 1.225

# Track reference (for AI and positioning)
var track_reference: Node = null

# ============================================================================
# SETUP AND INITIALIZATION
# ============================================================================
func _ready() -> void:
	_process_mode = ProcessModeEnum.ALWAYS
	chassis_node = $Chassis if has_node("Chassis") else self
	_init_wheels()
	_connect_signals_to_audio()
	_load_configuration()

func _init_wheels() -> void:
	"""Initialize wheel nodes for physics calculations."""
	var wheel_names = ["FrontLeft", "FrontRight", "RearLeft", "RearRight"]
	for name in wheel_names:
		if has_node(name):
			wheel_nodes.append(get_node(name))
		else:
			wheel_nodes.append(null)

func _connect_signals_to_audio() -> void:
	"""Connect vehicle signals to audio manager."""
	rpm_changed.connect(_on_rpm_changed)
	speed_changed.connect(_on_speed_changed)
	gear_changed.connect(_on_gear_changed)
	collision_detected.connect(_on_collision_detected)

# ============================================================================
# CONFIGURATION METHODS
# ============================================================================
func _load_configuration() -> void:
	"""Load vehicle configuration from resources."""
	var settings = PhysicsSettings.get_singleton()
	if settings:
		vehicle_mass = settings.default_vehicle_mass
		max_speed_kmh = settings.max_vehicle_speed
		acceleration_force = settings.base_acceleration_force
		braking_force = settings.base_braking_force

func _set_vehicle_mass(value: float) -> void:
	vehicle_mass = clamp(value, 500.0, 5000.0)
	_update_mass_properties()

func _set_max_speed_kmh(value: float) -> void:
	max_speed_kmh = clamp(value, 50.0, 500.0)

func _set_acceleration_force(value: float) -> void:
	acceleration_force = max(value, 1000.0)

func _set_braking_force(value: float) -> void:
	braking_force = max(value, 5000.0)

func _set_steering_angle_max(value: float) -> void:
	steering_angle_max = clamp(value, 10.0, 90.0)

func _set_final_drive_ratio(value: float) -> void:
	final_drive_ratio = clamp(value, 2.0, 5.0)

func _set_tire_radius(value: float) -> void:
	tire_radius = clamp(value, 0.2, 0.5)

func _set_torque_curve(value: Array[Vector2f]) -> void:
	torque_curve = value
	_sort_torque_curve()

func _sort_torque_curve() -> void:
	"""Ensure torque curve is sorted by RPM fraction."""
	torque_curve.sort_custom(func(a, b): return a.x < b.x)

func _set_num_gears(value: int) -> void:
	num_gears = clamp(value, 4, 10)
	_recalculate_gear_ratios()

func _set_gear_ratios(value: Array[float]) -> void:
	gear_ratios = value
	if len(gear_ratios) != num_gears:
		gear_ratios.resize(num_gears)

func _set_reverse_ratio(value: float) -> void:
	reverse_ratio = clamp(value, 2.0, 5.0)

func _set_idle_rpm(value: float) -> void:
	idle_rpm = clamp(value, 400.0, 1500.0)

func _set_redline_rpm(value: float) -> void:
	redline_rpm = clamp(value, 5000.0, 10000.0)

func _set_grip_coefficient(value: float) -> void:
	grip_coefficient = clamp(value, 0.3, 2.0)

func _set_drift_factor(value: float) -> void:
	drift_factor = clamp(value, 0.0, 1.0)

func _set_anti_roll_bar_stiffness(value: float) -> void:
	anti_roll_bar_stiffness = clamp(value, 0.0, 1.0)

func _set_track_width_front(value: float) -> void:
	track_width_front = clamp(value, 1.0, 2.5)

func _set_track_width_rear(value: float) -> void:
	track_width_rear = clamp(value, 1.0, 2.5)

func _set_wheelbase(value: float) -> void:
	wheelbase = clamp(value, 2.0, 4.0)

func _set_center_of_mass_height(value: float) -> void:
	center_of_mass_height = clamp(value, 0.3, 1.0)

func _update_mass_properties() -> void:
	"""Update mass-related physics values."""
	apply_central_force(Vector3(0, -(mass * 9.81), 0))

# ============================================================================
# INPUT HANDLING
# ============================================================================
func _input(event: InputEvent) -> void:
	"""Handle input events for vehicle control."""
	if event is InputEventKey:
		match event.pressed:
			true:
				_handle_key_press(event)
			false:
				_handle_key_release(event)

func _handle_key_press(event: InputEventKey) -> void:
	"""Process key press for driving controls."""
	match event.keycode:
		KEY_W, KEY_UP:
			throttle_input = 1.0
		KEY_S, KEY_DOWN:
			throttle_input = -0.3 if current_gear == 0 else 0.0
			brake_input = 1.0
		KEY_A, KEY_LEFT:
			steering_input = -1.0
		KEY_D, KEY_RIGHT:
			steering_input = 1.0
		KEY_SPACE:
			handbrake_input = 1.0
		KEY_F:
			engage_clutch()
		KEY_G:
			toggle_manual_shift()
		KEY_SHIFT:
			nitrous_input = 1.0

func _handle_key_release(event: InputEventKey) -> void:
	"""Process key release for driving controls."""
	match event.keycode:
		KEY_W, KEY_UP:
			throttle_input = 0.0
		KEY_S, KEY_DOWN:
			brake_input = 0.0
		KEY_A, KEY_LEFT:
			steering_input = 0.0
		KEY_D, KEY_RIGHT:
			steering_input = 0.0
		KEY_SPACE:
			handbrake_input = 0.0
		KEY_SHIFT:
			nitrous_input = 0.0

# ============================================================================
# PHYSICS UPDATE LOOP
# ============================================================================
func _physics_process(delta: float) -> void:
	"""Main physics update loop called every frame."""
	_update_inputs(delta)
	_update_engine_state(delta)
	_update_transmission(delta)
	_update_vehicle_dynamics(delta)
	_update_suspension(delta)
	_update_drift_and_traction(delta)
	_apply_forces(delta)
	_sync_visuals(delta)

func _update_inputs(delta: float) -> void:
	"""Smoothly interpolate input values."""
	throttle_input = lerp(throttle_input, get_throttle(), delta * 10.0)
	brake_input = lerp(brake_input, get_brake(), delta * 15.0)
	steering_input = lerp(steering_input, get_steering(), delta * 10.0)
	handbrake_input = lerp(handbrake_input, get_handbrake(), delta * 20.0)

func get_throttle() -> float:
	return Input.get_action_strength("vehicle_throttle") - Input.get_action_strength("vehicle_brake")

func get_brake() -> float:
	return Input.get_action_strength("vehicle_brake") + Input.get_action_strength("vehicle_handbrake")

func get_steering() -> float:
	return Input.get_axis("vehicle_steering_left", "vehicle_steering_right")

func get_handbrake() -> float:
	return Input.get_action_strength("vehicle_handbrake")

func _update_engine_state(delta: float) -> void:
	"""Calculate engine RPM and power output."""
	if not ignition_on or not clutch_engaged:
		current_rpm = lerp(current_rpm, idle_rpm if engine_running else 0.0, delta * 5.0)
		return
	
	var gear_ratio = _get_current_gear_ratio()
	var wheel_speed = road_speed_kmh / 3.6 / (PI * 2 * tire_radius) * 60.0
	var theoretical_rpm = wheel_speed * gear_ratio * final_drive_ratio
	
	# Apply throttle effect on RPM
	var rpm_target = _calculate_target_rpm()
	current_rpm = lerp(current_rpm, rpm_target, delta * 15.0)
	
	# Clamp to limits
	current_rpm = clamp(current_rpm, 0.0, redline_rpm * 1.2)
	
	# Calculate engine power based on torque curve
	var rpm_fraction = current_rpm / redline_rpm
	engine_power = _interpolate_torque(rpm_fraction) * (current_rpm / redline_rpm) * 100.0
	
	# Apply engine braking when not accelerating
	if throttle_input <= 0:
		engine_brake = abs(engine_power) * 0.3
	
	# Emissions check
	if current_rpm > redline_rpm:
		limiter_triggered()

func _calculate_target_rpm() -> float:
	"""Calculate target RPM based on current gear and throttle."""
	var gear_ratio = _get_current_gear_ratio()
	var ideal_rpm = road_speed_kmh / 3.6 / (PI * 2 * tire_radius) * gear_ratio * final_drive_ratio * 60.0
	
	if throttle_input > 0:
		return ideal_rpm + idle_rpm * 0.5
	elif throttle_input < 0:
		return idle_rpm * 0.7
	else:
		return idle_rpm

func _get_current_gear_ratio() -> float:
	"""Get the gear ratio for current gear."""
	if current_gear == GearState.NEUTRAL:
		return 0.0
	elif current_gear == GearState.REVERSE:
		return -reverse_ratio
	elif current_gear >= 1 and current_gear <= num_gears:
		return gear_ratios[current_gear - 1]
	else:
		return gear_ratios[0]

func _interpolate_torque(rpm_fraction: float) -> float:
	"""Interpolate torque from torque curve."""
	if torque_curve.is_empty():
		return 0.0
	
	var first = torque_curve.front()
	var last = torque_curve.back()
	
	if rpm_fraction <= first.x:
		return first.y
	elif rpm_fraction >= last.x:
		return last.y
	
	for i in range(len(torque_curve) - 1):
		var point_a = torque_curve[i]
		var point_b = torque_curve[i + 1]
		if rpm_fraction >= point_a.x and rpm_fraction <= point_b.x:
			var t = (rpm_fraction - point_a.x) / (point_b.x - point_a.x)
			return lerp(point_a.y, point_b.y, t)
	
	return last.y

func limiter_triggered() -> void:
	"""Engine limiter activated - reduce throttle."""
	throttle_input *= 0.5
 AudioManager.play_sound("engine_limiter")

func _update_transmission(delta: float) -> void:
	"""Handle automatic transmission logic."""
	shift_timer += delta
	
	if shift_timer >= shift_cooldown:
		_auto_shift()
		shift_timer = 0.0

func _auto_shift() -> void:
	"""Automatic gear shifting logic."""
	if current_gear == GearState.NEUTRAL:
		if road_speed_kmh > 0.1:
			target_gear = 1
		return
	
	var gear_thresholds = [
		[25.0, 40.0],   # Upshift from 1st
		[50.0, 70.0],   # Upshift from 2nd
		[75.0, 100.0],  # Upshift from 3rd
		[100.0, 130.0], # Upshift from 4th
		[130.0, 160.0], # Upshift from 5th
	]
	
	if target_gear == 0:
		target_gear = 1
	
	# Check upshift conditions
	if current_gear > 0 and current_gear < num_gears:
		if current_rpm > gear_thresholds[current_gear - 1][0] and throttle_input > 0.7:
			target_gear = min(current_gear + 1, num_gears)
	
	# Check downshift conditions
	elif current_rpm < idle_rpm * 1.5 and throttle_input < 0.3:
		target_gear = max(current_gear - 1, 1)
	
	# Execute shift
	if target_gear != current_gear and abs(target_gear - current_gear) <= 1:
		perform_shift(target_gear)

func perform_shift(new_gear: int) -> void:
	"""Execute a gear change."""
	if new_gear == current_gengear:
		return
	
	var old_gear = current_gear
	current_gear = new_gear
	last_shift_time = Time.get_ticks_msec()
	
	# Engage clutch temporarily during shift
	clutch_engaged = false
	await get_tree().create_timer(shift_cooldown).timeout
	clutch_engaged = true
	
	emit_signal("gear_changed", old_gear, new_gear)
	AudioManager.play_sound("gear_change")

func toggle_manual_shift() -> void:
	"""Toggle manual/automatic transmission mode."""
	pass

func engage_clutch() -> void:
	"""Engage/disengage clutch."""
	clutch_engaged = not clutch_engaged
	if clutch_engaged:
		AudioManager.play_sound("clutch_engage")

# ============================================================================
# VEHICLE DYNAMICS CALCULATION
# ============================================================================
func _update_vehicle_dynamics(delta: float) -> void:
	"""Calculate vehicle movement dynamics."""
	var forward_vector = transform.basis.z.normalized()
	var right_vector = transform.basis.x.normalized()
	
	# Calculate velocity components
	var velocity = global_transform.basis.z * road_speed_kmh / 3.6
	longitudinal_velocity = velocity.length()
	lateral_velocity = velocity.dot(right_vector)
	
	# Calculate slip angle
	slip_angle = atan2(lateral_velocity, max(longitudinal_velocity, 0.1)) * RAD_TO_DEG
	
	# Calculate slide coefficient
	var total_velocity = sqrt(pow(longitudinal_velocity, 2) + pow(lateral_velocity, 2))
	var wheel_velocity = current_rpm / 60.0 * PI * 2 * tire_radius
	wheel_slip_rate = abs(wheel_velocity - total_velocity) / max(total_velocity, 1.0)
	slide_coefficient = wheel_slip_rate / TRACTION_CONTROL_THRESHOLD
	
	# Update road speed
	road_speed_kmh = total_velocity * 3.6
	
	# Emit signals
	emit_signal("speed_changed", road_speed_kmh)
	emit_signal("rpm_changed", current_rpm)

func _update_suspension(delta: float) -> void:
	"""Update suspension compression and rebound."""
	if suspension_nodes.is_empty():
		return
	
	for i in range(suspension_nodes.size()):
		var suspension = suspension_nodes[i]
		if suspension:
			var body_height = chassis_node.global_position.y
			var wheel_height = suspension.global_position.y
			var compression = (body_height - wheel_height) / tire_radius
			compression = clamp(compression, -0.3, 0.5)
			
			# Update suspension visual
			suspension.scale.y = 1.0 + compression

func _update_drift_and_traction(delta: float) -> void:
	"""Calculate drift state and traction loss."""
	var absolute_slip = abs(slip_angle)
	
	# Determine if drifting
	if absolute_slip > DRIFT_THRESHOLD and handbrake_input > 0.3:
		if not drifting:
			drifting = true
			drift_angle = 0.0
			emit_signal("drift_started", absolute_slip)
			AudioManager.play_sound("drift_start")
		
		drift_angle = lerp(drift_angle, absolute_slip, delta * 2.0)
		drift_score += abs(absolute_slip - DRIFT_THRESHOLD) * delta * 100.0
		
	elif absolute_slip > DRIFT_THRESHOLD * 0.5 and handbrake_input < 0.1:
		if drifting:
			drifting = false
			drift_multiplier = 1.0
			emit_signal("drift_ended")
			AudioManager.play_sound("drift_end")
		drift_score = 0.0
		drift_angle = 0.0
	
	# Traction control logic
	if traction_control_enabled and wheel_slip_rate > TRACTION_CONTROL_THRESHOLD:
		throttle_input *= 0.5
		traction_loss = wheel_slip_rate - TRACTION_CONTROL_THRESHOLD
		emit_signal("traction_control_active", true)
	else:
		traction_loss = 0.0
		emit_signal("traction_control_active", false)

# ============================================================================
# FORCE APPLICATION
# ============================================================================
func _apply_forces(delta: float) -> void:
	"""Apply all forces to the vehicle."""
	var forward_vector = transform.basis.z.normalized()
	var right_vector = transform.basis.x.normalized()
	
	# Calculate drive force
	var drive_force = _calculate_drive_force() * forward_vector
	
	# Calculate braking force
	var brake_force = _calculate_brake_force() * forward_vector * -1.0
	
	# Apply aerodynamic drag
	var drag_force = _calculate_drag_force()
	
	# Apply lateral grip forces
	var lateral_force = _calculate_lateral_force()
	
	# Combine all forces
	var total_force = drive_force + brake_force + drag_force + lateral_force
	
	# Apply to vehicle
	apply_central_force(total_force)
	
	# Handle rotation (steering)
	var turn_force = _calculate_turn_force()
	if turn_force.length() > 0:
		apply_torque(turn_force)

func _calculate_drive_force() -> float:
	"""Calculate drive force based on engine power and gear."""
	if not clutch_engaged:
		return 0.0
	
	var gear_ratio = _get_current_gear_ratio()
	var effective_power = engine_power * gear_ratio * final_drive_ratio
	
	# Apply drivetrain losses
	var drivetrain_efficiency = 0.85
	if drivetrain_type == DrivetrainType.FWD:
		drivetrain_efficiency = 0.80
	elif drivetrain_type == DrivetrainType.AWD:
		drivetrain_efficiency = 0.90
	
	effective_power *= drivetrain_efficiency
	
	# Apply throttle
	var applied_force = effective_power * throttle_input
	
	# Cap at maximum acceleration force
	applied_force = min(applied_force, acceleration_force)
	
	return applied_force

func _calculate_brake_force() -> float:
	"""Calculate braking force."""
	if brake_input <= 0:
		return 0.0
	
	var brake_effectiveness = 1.0
	if handbrake_input > 0:
		brake_effectiveness = 1.5  # Handbrake adds extra braking
	
	return braking_force * brake_input * brake_effectiveness

func _calculate_drag_force() -> Vector3:
	"""Calculate aerodynamic drag."""
	var velocity_squared = pow(road_speed_kmh / 3.6, 2)
	var drag = 0.5 * air_density * drag_coefficient * frontal_area * velocity_squared
	var drag_vector = -transform.basis.z.normalized() * drag
	
	return drag_vector

func _calculate_lateral_force() -> Vector3:
	"""Calculate lateral grip force based on slip angle."""
	var available_grip = grip_coefficient * vehicle_mass * 9.81
	var slip_force = available_grip * slip_angle * DEG_TO_RAD
	
	# Apply drift modifier
	if drifting:
		slip_force *= drift_factor
	
	# Clamp to available grip
	slip_force = min(slip_force, available_grip)
	
	return -transform.basis.x.normalized() * slip_force

func _calculate_turn_force() -> Vector3:
	"""Calculate turning torque based on steering input."""
	if steering_input == 0.0:
		return Vector3.ZERO
	
	var steering_rotation = steering_input * steering_angle_max * DEG_TO_RAD
	var turn_torque = steering_rotation * vehicle_mass * center_of_mass_height
	
	return Vector3.UP * turn_torque

# ============================================================================
# COLLISION HANDLING
# ============================================================================
func _on_collision_detected(collision_data: Dictionary) -> void:
	"""Handle collision events."""
	var impact_speed = collision_data.get("impact_speed", 0.0)
	var impact_direction = collision_data.get("direction", Vector3.ZERO)
	
	# Calculate damage
	var damage = (impact_speed / 100.0) * 10.0
	damage = min(damage, 100.0)
	
	# Screen shake
	_screen_shake(impact_speed / 50.0)
	
	# Audio feedback
	if impact_speed > 50:
		AudioManager.play_sound("collision_heavy")
	elif impact_speed > 20:
		AudioManager.play_sound("collision_medium")
	else:
		AudioManager.play_sound("collision_light")
	
	# Particle effects
	_spawn_collision_particles(collision_data)

func _screen_shake(intensity: float) -> void:
	"""Apply screen shake effect."""
	if chassis_node:
		chassis_node.position += Vector3(randf() * intensity, randf() * intensity, randf() * intensity)

func _spawn_collision_particles(data: Dictionary) -> void:
	"""Spawn particle effects on collision."""
	var position = data.get("position", global_position)
	var direction = data.get("direction", Vector3.DOWN)
	
	# Spawn explosion particles
	AudioManager.spawn_particles(position, "collision_sparks", direction)

# ============================================================================
# ENGINE CONTROL
# ============================================================================
func start_engine() -> void:
	"""Start the vehicle engine."""
	if not ignition_on:
		ignition_on = true
	
	engine_running = true
	AudioManager.play_sound("engine_start")
	emit_signal("engine_started")

func stop_engine() -> void:
	"""Stop the vehicle engine."""
	engine_running = false
	ignition_on = false
	current_rpm = 0.0
	emit_signal("engine_stopped")
	AudioManager.play_sound("engine_stop")

func toggle_engine() -> void:
	"""Toggle engine state."""
	if engine_running:
		stop_engine()
	else:
		start_engine()

func emergency_stop() -> void:
	"""Emergency stop - kill engine and apply maximum brakes."""
	stop_engine()
	brake_input = 1.0
	throttle_input = 0.0
	clutch_engaged = false

# ============================================================================
# VISUAL SYNC
# ============================================================================
func _sync_visuals(delta: float) -> void:
	"""Sync visual elements with physics state."""
	# Sync wheels rotation
	for i in range(wheel_nodes.size()):
		var wheel = wheel_nodes[i]
		if wheel:
			var wheel_rotation = road_speed_kmh / 3.6 / (PI * 2 * tire_radius) * 60.0 * delta
			wheel.rotate_y(wheel_rotation)
	
	# Sync steering visual
	if has_node("SteeringVisual"):
		var steering = get_node("SteeringVisual")
		var target_rotation = steering_input * steering_angle_max * DEG_TO_RAD
		steering.rotation.y = lerp(steering.rotation.y, target_rotation, delta * 10.0)

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================
func reset_vehicle() -> void:
	"""Reset vehicle to initial state."""
	position = Vector3(0, 1, 0)
	rotation = Vector3(0, 0, 0)
	velocity = Vector3.ZERO
	current_rpm = idle_rpm
	current_gear = 0
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	handbrake_input = 0.0
	drifting = false
	drift_score = 0.0

func set_position(pos: Vector3) -> void:
	"""Set vehicle position directly."""
	global_position = pos

func set_velocity(vel: Vector3) -> void:
	"""Set vehicle velocity directly."""
	velocity = vel

func get_velocity_world() -> Vector3:
	"""Get current world-space velocity."""
	return velocity

func get_speed_kmh() -> float:
	"""Get current speed in km/h."""
	return road_speed_kmh

func get_rpm() -> float:
	"""Get current engine RPM."""
	return current_rpm

func get_gear() -> int:
	"""Get current gear."""
	return current_gear

func is_drifting() -> bool:
	"""Check if currently drifting."""
	return drifting

func get_drift_score() -> float:
	"""Get current drift score."""
	return drift_score

func get_drift_multiplier() -> float:
	"""Get drift score multiplier."""
	return drift_multiplier

# ============================================================================
# SAVE/LOAD SUPPORT
# ============================================================================
func save_state() -> Dictionary:
	"""Save current vehicle state."""
	return {
		"position": global_position,
		"rotation": rotation,
		"velocity": velocity,
		"current_rpm": current_rpm,
		"current_gear": current_gear,
		"throttle_input": throttle_input,
		"brake_input": brake_input,
		"steering_input": steering_input,
		"handbrake_input": handbrake_input,
		"drifting": drifting,
		"drift_score": drift_score,
		"engine_running": engine_running,
		"ignition_on": ignition_on
	}

func load_state(state: Dictionary) -> void:
	"""Load vehicle state from dictionary."""
	if state.has("position"):
		global_position = state["position"]
	if state.has("rotation"):
		rotation = state["rotation"]
	if state.has("velocity"):
		velocity = state["velocity"]
	if state.has("current_rpm"):
		current_rpm = state["current_rpm"]
	if state.has("current_gear"):
		current_gear = state["current_gear"]
	if state.has("throttle_input"):
		throttle_input = state["throttle_input"]
	if state.has("brake_input"):
		brake_input = state["brake_input"]
	if state.has("steering_input"):
		steering_input = state["steering_input"]
	if state.has("handbrake_input"):
		handbrake_input = state["handbrake_input"]
	if state.has("drifting"):
		drifting = state["drifting"]
	if state.has("drift_score"):
		drift_score = state["drift_score"]
	if state.has("engine_running"):
		engine_running = state["engine_running"]
	if state.has("ignition_on"):
		ignition_on = state["ignition_on"]

# ============================================================================
# SIGNAL CALLBACKS
# ============================================================================
func _on_rpm_changed(new_rpm: float) -> void:
	"""Called when RPM changes."""
	pass

func _on_speed_changed(new_speed: float) -> void:
	"""Called when speed changes."""
	pass

func _on_gear_changed(old_gear: int, new_gear: int) -> void:
	"""Called when gear changes."""
	pass

func _on_collision_detected(collision_data: Dictionary) -> void:
	"""Called on collision."""
	pass

func _on_drift_started(angle: float) -> void:
	"""Called when drift starts."""
	pass

func _on_drift_ended() -> void:
	"""Called when drift ends."""
	pass

func _on_traction_control_active(active: bool) -> void:
	"""Called when traction control state changes."""
	pass

# ============================================================================
# DEBUG/INFO METHODS
# ============================================================================
func print_debug_info() -> void:
	"""Print debug information to console."""
	print("\n=== VEHICLE CONTROLLER DEBUG ===")
	print(f"Speed: {road_speed_kmh:.2f} km/h")
	print(f"RPM: {current_rpm:.0f}")
	print(f"Gear: {current_gear}")
	print(f"Throttle: {throttle_input:.2f}")
print(f"Brake: {brake_input:.2f}")
print(f"Steering: {steering_input:.2f}")
print(f"Handbrake: {handbrake_input:.2f}")
print(f"Drifting: {drifting}")
print(f"Slip Angle: {slip_angle:.2f}°")
print(f"Slide Coefficient: {slide_coefficient:.2f}")
print(f"Drive Force: {_calculate_drive_force():.2f}")
print("==============================\n")

func print_torque_curve() -> void:
	"""Print torque curve data."""
	print("\n=== TORQUE CURVE ===")
	for point in torque_curve:
		var rpm_percent = point.x * 100
		var torque = point.y * 100
		print(f"{rpm_percent:.0f}% RPM -> {torque:.0f}% Torque")
	print("=====================\n")

</script>