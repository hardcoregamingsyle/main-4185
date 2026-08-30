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
signal nitrous_active(is_active: bool)
signal vehicle_health_changed(current_health: float, max_health: float)
signal tire_wear_changed(wear_level: float)
signal oversteer_detected(angle: float)
signal understeer_detected(angle: float)

# ============================================================================
# EXPORTED CONFIGURATION
# ============================================================================
@export_group("Vehicle Configuration")
@export var vehicle_mass: float = 1500.0
@export var center_of_mass_offset: Vector3 = Vector3(0.0, 0.5, 0.0)
@export var wheel_base: float = 2.6
@export var track_width: float = 1.6
@export var suspension_stiffness: float = 35000.0
@export var damping_ratio: float = 0.3
@export var max_suspension_travel: float = 0.15

@export_group("Engine & Transmission")
@export var max_engine_power: float = 300.0  # in kW
@export var max_engine_torque: float = 500.0  # in Nm
@export var engine_idle_rpm: float = 800.0
@export var engine_max_rpm: float = 7000.0
@export var redline_rpm: float = 7500.0
@export var clutch_disengage_rpm: float = 500.0
@export var gear_ratios: Array[float] = [3.8, 2.2, 1.5, 1.1, 0.85, 0.65, 0.55]
@export var final_drive_ratio: float = 3.5
@export var transmission_type: String = "manual"  # manual or automatic

@export_group("Braking System")
@export var front_brake_bias: float = 0.6
@export var max_brake_force: float = 12000.0
@export var brake_pressure_factor: float = 1.0

@export_group("Tires & Suspension")
@export var tire_friction_coefficient: float = 1.2
@export var tire_side_friction: float = 0.8
@export var tire_vertical_stiffness: float = 40000.0
@export var tire_damping: float = 500.0
@export var tire_inflation_pressure: float = 2.2  # bar

@export_group("Aerodynamics")
@export var drag_coefficient: float = 0.30
@export var frontal_area: float = 2.2  # m^2
@export var downforce_coefficient: float = 0.5
@export var aero_balance_front: float = 0.55

@export_group("Handling & Control")
@export var max_steering_angle: float = 30.0  # degrees
@export var steering_rate: float = 15.0  # degrees per second
@export var traction_control_max_slip: float = 0.15
@export var abs_threshold: float = 0.25
@export var drift_threshold: float = 0.20

@export_group("Game Settings")
@export var health_points: int = 100
@export var repair_rate: float = 0.5  # HP per second when not moving
@export var boost_capacity: float = 100.0
@export var boost_recharge_rate: float = 5.0  # per second
@export var nitrous_multiplier: float = 1.5

# ============================================================================
# PUBLIC PROPERTIES
# ============================================================================
var current_speed: float = 0.0  # m/s
var current_rpm: float = 0.0
var current_gear: int = 0
var target_gear: int = 0
var steering_angle: float = 0.0  # degrees
var throttle_input: float = 0.0
var brake_input: float = 0.0
var handbrake_input: float = 0.0
var clutch_input: float = 0.0
var drift_angle: float = 0.0
var is_drifting: bool = false
var is_skidding: bool = false
var is_boosting: bool = false
var is_nitrous_active: bool = false
var traction_control_enabled: bool = true
var abs_enabled: bool = true
var current_health: float = 100.0
var current_boost: float = 100.0
var current_tire_wear: float = 0.0
var is_engine_running: bool = true
var is_in_park: bool = false

# ============================================================================
# PRIVATE STATE
# ============================================================================
var _current_velocity: Vector3 = Vector3.ZERO
var _acceleration_vector: Vector3 = Vector3.ZERO
var _angular_velocity: Vector3 = Vector3.ZERO
var _slip_ratio: float = 0.0
var _last_checkpoints: Array[int] = []
var _lap_start_time: float = 0.0
var _lap_times: Array[float] = []
var _total_laps: int = 0
var _vehicle_body: Node3D = null
var _suspension_states: Dictionary = {}
var _wheel_forces: Dictionary = {}
var _audio_source: AudioStreamPlayer3D = null
var _powertrain_node: Node = null
var _collision_raycasters: Array[RayCast3D] = []
var _drift_history: Array[float] = []
var _tire_conditions: Dictionary = {
	"front_left": {"wear": 0.0, "temperature": 80.0, "pressure": 2.2},
	"front_right": {"wear": 0.0, "temperature": 80.0, "pressure": 2.2},
	"rear_left": {"wear": 0.0, "temperature": 80.0, "pressure": 2.2},
	"rear_right": {"wear": 0.0, "temperature": 80.0, "pressure": 2.2}
}

# ============================================================================
# CONSTANTS
# ============================================================================
const DEGREES_TO_RADIANS := 0.01745329252
const RADIANS_TO_DEGREES := 57.29577951
const KM_H_TO_MS := 0.277777778
const MS_TO_KM_H := 3.6
const EARTH_GRAVITY := PhysicsSettings.gravity
const MIN_GEAR: int = 1
const MAX_GEAR: int = 7
const NEUTRAL_GEAR: int = 0
const PARK_GEAR: int = -1

# ============================================================================
# LIFECYCLE
# ============================================================================
func _ready() -> void:
	_init_components()
	_connect_signals()
	_reset_vehicle_state()
	_setup_wheel_collisions()
	print_debug("VehicleController initialized successfully")

func _process(_delta: float) -> void:
	_update_gameplay_logic()

func _physics_process(delta: float) -> void:
	_apply_physics(delta)

func _input(event: InputEvent) -> void:
	_handle_input(event)

func _on_collision_entered(body: Node3D) -> void:
	_handle_collision(body)

func _on_collision_exited(body: Node3D) -> void:
	pass  # Track collisions if needed

# ============================================================================
# INITIALIZATION
# ============================================================================
func _init_components() -> void:
	"""Initialize all vehicle components and subsystems"""
	# Get body node for transforms
	_vehicle_body = get_parent() if get_parent() is Node3D else self
	
	# Setup suspension states for each wheel position
	_suspension_states = {
		"front_left": {"compression": 0.0, "velocity": 0.0},
		"front_right": {"compression": 0.0, "velocity": 0.0},
		"rear_left": {"compression": 0.0, "velocity": 0.0},
		"rear_right": {"compression": 0.0, "velocity": 0.0}
	}
	
	# Initialize wheel force dictionaries
	_wheel_forces = {
		"front_left": Vector3.ZERO,
		"front_right": Vector3.ZERO,
		"rear_left": Vector3.ZERO,
		"rear_right": Vector3.ZERO
	}
	
	# Find powertrain node if exists
	_powertrain_node = find_child("Powertrain", false, false)
	if _powertrain_node == null:
		var search_result = search_children_by_class("Powertrain")
		if search_result.size() > 0:
			_powertrain_node = search_result[0]

func _connect_signals() -> void:
	"""Connect internal signals to handlers"""
	speed_changed.connect(_on_speed_changed)
	rpm_changed.connect(_on_rpm_changed)
	gear_changed.connect(_on_gear_changed)
	skidding.connect(_on_skidding)
	collision_detected.connect(_on_collision_detected)
	engine_stalled.connect(_on_engine_stalled)
	handbrake_toggled.connect(_on_handbrake_toggled)
	traction_control_state_changed.connect(_on_traction_control_changed)
	anti_lock_braking_state_changed.connect(_on_abs_changed)
	drift_started.connect(_on_drift_started)
	drift_ended.connect(_on_drift_ended)
	boost_used.connect(_on_boost_used)
	nitrous_active.connect(_on_nitrous_active)
	vehicle_health_changed.connect(_on_vehicle_health_changed)

func _reset_vehicle_state() -> void:
	"""Reset vehicle to initial state"""
	current_speed = 0.0
	current_rpm = PhysicsSettings.engine_idle_rpm
	current_gear = NEUTRAL_GEAR
	target_gear = NEUTRAL_GEAR
	throttle_input = 0.0
	brake_input = 0.0
	handbrake_input = 0.0
	clutch_input = 0.0
	steering_angle = 0.0
	is_drifting = false
	is_skidding = false
	is_boosting = false
	is_nitrous_active = false
	current_health = health_points
	current_boost = boost_capacity
	current_tire_wear = 0.0
	is_engine_running = true
	is_in_park = false
	_last_checkpoints.clear()
	_lap_times.clear()
	_total_laps = 0
	_drift_history.clear()
	for wheel in _tire_conditions.keys():
		_tire_conditions[wheel]["wear"] = 0.0
		_tire_conditions[wheel]["temperature"] = 80.0
		_tire_conditions[wheel]["pressure"] = tire_inflation_pressure

func _setup_wheel_collisions() -> void:
	"""Setup raycasters for wheel contact points"""
	# Create collision raycasters for each wheel
	var wheel_positions = ["front_left", "front_right", "rear_left", "rear_right"]
	for wheel in wheel_positions:
		var raycaster = RayCast3D.new()
		raycaster.name = "%s_WheelRaycaster" % wheel
		raycaster.target_position = Vector3(0.0, -max_suspension_travel, 0.0)
		raycaster.collision_mask = 1  # Adjust based on your scene colliders
		add_child(raycaster)
		_collision_raycasters.append(raycaster)

func search_children_by_class(class_name: String) -> Array[Node]:
	"""Search for children nodes of specified class"""
	var result = []
	for child in get_children():
		if class_name in str(child):
			result.append(child)
	return result

# ============================================================================
# INPUT HANDLING
# ============================================================================
func _handle_input(event: InputEvent) -> void:
	"""Process input events for vehicle control"""
	if event.is_pressed():
		# Throttle input (W, Up Arrow, Right Shift)
		if event.as_text() == "w" or event.as_text() == "up" or event.as_text() == "shift+right":
			if not is_in_park and is_engine_running:
				throttle_input = min(throttle_input + 0.05, 1.0)
		
		# Brake input (S, Down Arrow, Left Shift)
		elif event.as_text() == "s" or event.as_text() == "down" or event.as_text() == "shift+left":
			if not is_in_park:
				brake_input = min(brake_input + 0.05, 1.0)
		
		# Steering input (A/D, Left/Right Arrow)
		elif event.as_text() == "a" or event.as_text() == "left":
			steering_angle = max(steering_angle - 2.0, -max_steering_angle)
		elif event.as_text() == "d" or event.as_text() == "right":
			steering_angle = min(steering_angle + 2.0, max_steering_angle)
		
		# Handbrake (Space)
		elif event.as_text() == "space":
			handbrake_input = 1.0
		
		# Gear up (Q)
		elif event.as_text() == "q":
			_shift_gear_up()
		
		# Gear down (E)
		elif event.as_text() == "e":
			_shift_gear_down()
		
		# Toggle traction control (T)
		elif event.as_text() == "t":
			traction_control_enabled = !traction_control_enabled
			traction_control_state_changed.emit(traction_control_enabled)
		
		# Toggle ABS (G)
		elif event.as_text() == "g":
			abs_enabled = !abs_enabled
			anti_lock_braking_state_changed.emit(abs_enabled)
		
		# Activate boost (F)
		elif event.as_text() == "f" and current_boost > 0:
			_activate_boost()
		
		# Activate nitrous (H)
		elif event.as_text() == "h" and current_boost >= boost_capacity * 0.5:
			_activate_nitrous()
	
	elif event.is_released():
		# Release handbrake
		if event.as_text() == "space":
			handbrake_input = 0.0

func _update_gameplay_logic() -> void:
	"""Update gameplay-related logic independent of physics"""
	# Boost recharge
	if not is_boosting:
		current_boost = min(current_boost + boost_recharge_rate * Engine.get_physics_delta(), boost_capacity)
	
	# Health regeneration when stationary
	if current_speed < 1.0 and current_health < health_points:
		current_health = min(current_health + repair_rate * Engine.get_physics_delta(), float(health_points))
	
	# Tire wear calculation
	_update_tire_wear()

func _update_tire_wear() -> void:
	"""Calculate tire wear based on driving conditions"""
	var total_slip = 0.0
	var total_temperature = 0.0
	var wear_increase = 0.0
	
	for wheel_data in _tire_conditions.values():
		total_slip += wheel_data["wear"]
		total_temperature += wheel_data["temperature"]
	
	var avg_slip = total_slip / 4.0
	var temp_factor = max(0.0, (_tire_conditions["front_left"]["temperature"] - 80.0) / 100.0)
	wear_increase = (avg_slip * 0.01 * temp_factor) * Engine.get_physics_delta()
	
	current_tire_wear = min(current_tire_wear + wear_increase, 1.0)

# ============================================================================
# PHYSICS SIMULATION
# ============================================================================
func _apply_physics(delta: float) -> void:
	"""Apply all vehicle physics calculations"""
	if not is_engine_running:
		_apply_drag_and_friction(delta)
		apply_global_velocity(_current_velocity)
		return
	
	# Calculate engine RPM based on gear and speed
	_update_engine_rpm(delta)
	
	# Apply throttle and torque
	var engine_torque = _calculate_engine_torque()
	_apply_drive_torque(engine_torque, delta)
	
	# Handle braking
	_apply_brakes(delta)
	
	# Apply steering and calculate lateral forces
	_apply_steering(delta)
	
	# Calculate aerodynamic forces
	_apply_aerodynamics(delta)
	
	# Update velocity based on forces
	_update_velocity(delta)
	
	# Check for skidding and drifting
	_check_skid_and_drift()
	
	# Apply movement
	apply_global_velocity(_current_velocity)

func _update_engine_rpm(delta: float) -> void:
	"""Calculate current engine RPM based on gear ratio and vehicle speed"""
	if current_gear <= NEUTRAL_GEAR:
		# In neutral or park, RPM follows idle or revs freely
		if is_engine_running:
			current_rpm = lerp(current_rpm, PhysicsSettings.engine_idle_rpm, 0.1)
		else:
			current_rpm = 0.0
		return
	
	# Calculate theoretical RPM based on gear ratio and vehicle speed
	var wheel_radius = 0.33  # Approximate tire radius in meters
	var wheel_rotation_speed = current_speed / wheel_radius  # rad/s
	var wheel_rotations_per_second = wheel_rotation_speed / (2.0 * PI)
	var drive_shaft_rotations = wheel_rotations_per_second * gear_ratios[current_gear - 1] * final_drive_ratio
	current_rpm = drive_shaft_rotations * 60.0  # Convert to RPM
	
	# Clamp RPM to valid range
	current_rpm = clamp(current_rpm, 0.0, engine_max_rpm)
	
	# Rev limiter behavior
	if current_rpm >= redline_rpm:
		current_rpm = redline_rpm

func _calculate_engine_torque() -> float:
	"""Calculate engine torque output based on RPM and throttle"""
	var normalized_rpm = (current_rpm - engine_idle_rpm) / (engine_max_rpm - engine_idle_rpm)
	
	# Torque curve (parabolic approximation)
	var torque_curve = max(0.0, 1.0 - pow(normalized_rpm - 0.6, 2.0) / 0.36)
	var base_torque = max_engine_torque * torque_curve
	
	# Apply throttle
	var actual_torque = base_torque * throttle_input
	
	# Apply boost multiplier
	if is_boosting:
		actual_torque *= 1.3
	
	# Apply nitrous multiplier
	if is_nitrous_active:
		actual_torque *= nitrous_multiplier
	
	return actual_torque

func _apply_drive_torque(torque: float, delta: float) -> void:
	"""Apply drive torque to rear wheels (RWD layout)"""
	if current_gear <= NEUTRAL_GEAR:
		return
	
	# Distribute torque to rear wheels
	var rear_torque = torque * 0.5  # Split between left and right
	
	# Apply to rear wheels
	_wheel_forces["rear_left"].x = rear_torque / wheel_base
	_wheel_forces["rear_right"].x = rear_torque / wheel_base

func _apply_brakes(delta: float) -> void:
	"""Apply braking forces to all wheels"""
	if brake_input <= 0.0 and handbrake_input <= 0.0:
		return
	
	var total_brake_force = max_brake_force * brake_input * brake_pressure_factor
	
	# Add handbrake effect
	if handbrake_input > 0.0:
		total_brake_force += max_brake_force * handbrake_input * 0.5
	
	# Front/Rear brake bias
	var front_force = total_brake_force * front_brake_bias
	var rear_force = total_brake_force * (1.0 - front_brake_bias)
	
	# Apply to wheels
	_wheel_forces["front_left"].x -= front_force / 2.0
	_wheel_forces["front_right"].x -= front_force / 2.0
	_wheel_forces["rear_left"].x -= rear_force / 2.0
	_wheel_forces["rear_right"].x -= rear_force / 2.0

func _apply_steering(delta: float) -> void:
	"""Apply steering angle and calculate lateral forces"""
	var steering_target = 0.0
	
	if throttle_input > 0.0:
		steering_target = steering_angle
	elif brake_input > 0.0:
		steering_target = steering_angle * 0.5  # Less effective while braking
	else:
		steering_target = lerp(steering_angle, 0.0, 0.1)
	
	steering_angle = lerp(steering_angle, steering_target, steering_rate * delta * RADIANS_TO_DEGREES)
	
	# Apply lateral force based on steering
	var lateral_force = current_speed * sin(steering_angle * DEGREES_TO_RADIANS) * tire_side_friction
	_current_velocity.y += lateral_force * delta

func _apply_aerodynamics(delta: float) -> void:
	"""Apply aerodynamic drag and downforce"""
	var speed_squared = _current_velocity.length_squared()
	var air_density = 1.225  # kg/m^3 at sea level
	
	# Drag force: F_drag = 0.5 * rho * v^2 * Cd * A
	var drag_force = 0.5 * air_density * speed_squared * drag_coefficient * frontal_area
	
	# Downforce: F_downforce = 0.5 * rho * v^2 * Cl * A
	var downforce = 0.5 * air_density * speed_squared * downforce_coefficient * frontal_area
	
	# Apply drag opposite to velocity direction
	if _current_velocity.length() > 0:
		var drag_direction = _current_velocity.normalized()
		_current_velocity -= drag_direction * drag_force * delta / vehicle_mass
	
	# Apply downforce as additional normal force (affects tire grip)
	# This would modify tire friction coefficient in a more detailed model

func _update_velocity(delta: float) -> void:
	"""Update vehicle velocity based on applied forces"""
	# Apply wheel forces to velocity
	for wheel_name in _wheel_forces.keys():
		_current_velocity += _wheel_forces[wheel_name] * delta / vehicle_mass
	
	# Cap maximum speed
	var max_speed = _calculate_max_speed()
	if _current_velocity.length() > max_speed:
		_current_velocity = _current_velocity.normalized() * max_speed
	
	# Update current speed for signals
	var old_speed = current_speed
	current_speed = _current_velocity.length()
	if abs(current_speed - old_speed) > 0.1:
		speed_changed.emit(current_speed)

func _calculate_max_speed() -> float:
	"""Calculate theoretical maximum speed in top gear"""
	if current_gear != MAX_GEAR:
		return current_speed * 1.5  # Allow some buffer below max
	
	var wheel_radius = 0.33
	var max_wheel_rps = engine_max_rpm / 60.0 / gear_ratios[MAX_GEAR - 1] / final_drive_ratio
	return max_wheel_rps * 2.0 * PI * wheel_radius

func _check_skid_and_drift() -> void:
	"""Check for skidding and drifting conditions"""
	var slip_ratio = abs(current_rpm / (current_speed + 0.1) - 1.0)
	_slip_ratio = slip_ratio
	
	# Skidding check
	if slip_ratio > traction_control_max_slip and traction_control_enabled:
		is_skidding = true
		skidding.emit(true)
	else:
		is_skidding = false
		skidding.emit(false)
	
	# Drifting check
	if handbrake_input > 0.0 and current_speed > 5.0 and abs(steering_angle) > 10.0:
		is_drifting = true
		drift_angle = steering_angle * 0.7
		if not _drift_history.has(drift_angle):
			_drift_history.append(drift_angle)
			drift_started.emit(drift_angle)
	elif abs(steering_angle) < 5.0:
		if is_drifting:
			is_drifting = false
			drift_ended.emit()
			_drift_history.clear()

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================
func _shift_gear_up() -> void:
	"""Shift transmission to next higher gear"""
	if current_gear >= MAX_GEAR or current_gear <= NEUTRAL_GEAR:
		return
	
	var old_gear = current_gear
	target_gear = current_gear + 1
	
	if clutch_input > 0.5:  # Only shift if clutch is engaged
		current_gear = target_gear
		gear_changed.emit(old_gear, current_gear)
		_on_gear_changed(old_gear, current_gear)

func _shift_gear_down() -> void:
	"""Shift transmission to next lower gear"""
	if current_gear <= MIN_GEAR:
		return
	
	var old_gear = current_gear
	target_gear = max(MIN_GEAR, current_gear - 1)
	
	if clutch_input > 0.5:
		current_gear = target_gear
		gear_changed.emit(old_gear, current_gear)
		_on_gear_changed(old_gear, current_gear)

func set_gear(gear_index: int) -> void:
	"""Set gear directly (for AI or debugging)"""
	if gear_index < NEUTRAL_GEAR or gear_index > MAX_GEAR:
		return
	
	var old_gear = current_gear
	current_gear = gear_index
	gear_changed.emit(old_gear, current_gear)

func set_transmission_type(type: String) -> void:
	"""Set transmission type (manual/auto)"""
	transmission_type = type.lower()

# ============================================================================
# BOOST & NITROUS SYSTEMS
# ============================================================================
func _activate_boost() -> void:
	"""Activate boost system"""
	if current_boost <= 0:
		return
	
	is_boosting = true
	boost_used.emit(3.0)  # 3 second boost duration

func _deactivate_boost() -> void:
	"""Deactivate boost system"""
	is_boosting = false

func _activate_nitrous() -> void:
	"""Activate nitrous oxide system"""
	if current_boost < boost_capacity * 0.5:
		return
	
	is_nitrous_active = true
	nitrous_active.emit(true)

func _deactivate_nitrous() -> void:
	"""Deactivate nitrous system"""
	is_nitrous_active = false
	nitrous_active.emit(false)

# ============================================================================
# COLLISION HANDLING
# ============================================================================
func _handle_collision(body: Node3D) -> void:
	"""Handle vehicle collision events"""
	var collision_info = {
		"body": body.name if body.has_method("get_name") else str(body),
		"position": global_position,
		"impact_velocity": _current_velocity.length(),
		"time": Time.get_ticks_msec()
	}
	
	collision_detected.emit(collision_info)
	
	# Apply damage based on impact velocity
	var impact_damage = _calculate_collision_damage(collision_info["impact_velocity"])
	current_health = max(0.0, current_health - impact_damage)
	vehicle_health_changed.emit(current_health, float(health_points))

func _calculate_collision_damage(impact_velocity: float) -> float:
	"""Calculate damage based on collision impact"""
	var damage = 0.0
	if impact_velocity > 20.0:
		damage = (impact_velocity - 20.0) * 0.5
	elif impact_velocity > 10.0:
		damage = (impact_velocity - 10.0) * 0.3
	return damage

# ============================================================================
# LAP & CHECKPOINT SYSTEM
# ============================================================================
func start_lap() -> void:
	"""Start timing a lap"""
	_lap_start_time = Time.get_ticks_msec()
	_last_checkpoints.clear()

func record_checkpoint(checkpoint_id: int) -> void:
	"""Record passing through a checkpoint"""
	if not _last_checkpoints.has(checkpoint_id):
		_last_checkpoints.append(checkpoint_id)
		checkpoint_passed.emit(checkpoint_id)

func complete_lap() -> void:
	"""Record lap completion"""
	var lap_time_ms = Time.get_ticks_msec() - _lap_start_time
	var lap_time_seconds = lap_time_ms / 1000.0
	
	_lap_times.append(lap_time_seconds)
	lap_completed.emit(lap_time_seconds)
	_total_laps += 1
	_lap_start_time = Time.get_ticks_msec()

func get_lap_stats() -> Dictionary:
	"""Get statistics about laps"""
	return {
		"total_laps": _total_laps,
		"best_lap": _lap_times.min() if _lap_times.size() > 0 else 0.0,
		"average_lap": _lap_times.sum() / _lap_times.size() if _lap_times.size() > 0 else 0.0,
		"current_lap_time": (Time.get_ticks_msec() - _lap_start_time) / 1000.0 if _lap_start_time > 0 else 0.0
	}

# ============================================================================
# VEHICLE HEALTH & DAMAGE
# ============================================================================
func take_damage(amount: float) -> void:
	"""Take direct damage to vehicle"""
	current_health = max(0.0, current_health - amount)
	vehicle_health_changed.emit(current_health, float(health_points))
	
	if current_health <= 0:
		_on_engine_stalled()

func heal(amount: float) -> void:
	"""Repair vehicle"""
	current_health = min(float(health_points), current_health + amount)
	vehicle_health_changed.emit(current_health, float(health_points))

func reset_health() -> void:
	"""Reset vehicle health to full"""
	current_health = float(health_points)
	vehicle_health_changed.emit(current_health, float(health_points))

# ============================================================================
# ENGINE CONTROL
# ============================================================================
func start_engine() -> void:
	"""Start the vehicle engine"""
	is_engine_running = true
	current_rpm = PhysicsSettings.engine_idle_rpm

func stop_engine() -> void:
	"""Stop the vehicle engine"""
	is_engine_running = false
	current_rpm = 0.0
	engine_stalled.emit()

func toggle_engine() -> void:
	"""Toggle engine state"""
	if is_engine_running:
		stop_engine()
	else:
		start_engine()

# ============================================================================
# HANDLING & PARKING
# ============================================================================
func engage_park() -> void:
	"""Engage parking mode"""
	is_in_park = true
	current_gear = PARK_GEAR
	brake_input = 1.0

func disengage_park() -> void:
	"""Disengage parking mode"""
	is_in_park = false
	if current_gear == PARK_GEAR:
		current_gear = NEUTRAL_GEAR

func toggle_park() -> void:
	"""Toggle parking mode"""
	if is_in_park:
		disengage_park()
	else:
		engage_park()

# ============================================================================
# TIRE MANAGEMENT
# ============================================================================
func get_tire_condition(wheel: String) -> Dictionary:
	"""Get condition data for specific tire"""
	if wheel in _tire_conditions:
		return _tire_conditions[wheel].duplicate()
	return {}

func set_tire_pressure(wheel: String, pressure: float) -> void:
	"""Set tire inflation pressure"""
	if wheel in _tire_conditions:
		_tire_conditions[wheel]["pressure"] = pressure

func set_tire_temperature(wheel: String, temperature: float) -> void:
	"""Set tire temperature"""
	if wheel in _tire_conditions:
		_tire_conditions[wheel]["temperature"] = temperature

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================
func get_speed_kmh() -> float:
	"""Convert speed to kilometers per hour"""
	return current_speed * MS_TO_KM_H

func get_speed_mph() -> float:
	"""Convert speed to miles per hour"""
	return current_speed * 2.23694

func get_acceleration() -> float:
	"""Get current acceleration in m/s²"""
	return _acceleration_vector.length()

func get_angular_velocity() -> Vector3:
	"""Get current angular velocity"""
	return _angular_velocity

func is_vehicle_moving() -> bool:
	"""Check if vehicle is currently moving"""
	return current_speed > 0.1

func is_vehicle_in_gear() -> bool:
	"""Check if vehicle has a gear selected"""
	return current_gear > NEUTRAL_GEAR

func debug_get_state() -> Dictionary:
	"""Get debug information about vehicle state"""
	return {
		"speed": current_speed,
		"rpm": current_rpm,
		"gear": current_gear,
		"throttle": throttle_input,
		"brake": brake_input,
		"handbrake": handbrake_input,
		"steering": steering_angle,
		"is_drifting": is_drifting,
		"is_skidding": is_skidding,
		"health": current_health,
		"boost": current_boost,
		"tire_wear": current_tire_wear,
		"in_park": is_in_park,
		"engine_running": is_engine_running
	}

# ============================================================================
# DEBUG & TESTING HELPERS
# ============================================================================
func force_set_speed(speed: float) -> void:
	"""Force set vehicle speed (for testing)"""
	_current_velocity = Vector3(speed, 0.0, 0.0)
	current_speed = speed

func force_set_rpm(rpm: float) -> void:
	"""Force set engine RPM (for testing)"""
	current_rpm = clamp(rpm, 0.0, engine_max_rpm)

func force_set_gear(gear: int) -> void:
	"""Force set gear (for testing)"""
	current_gear = clamp(gear, NEUTRAL_GEAR, MAX_GEAR)

func force_set_throttle(input: float) -> void:
	"""Force set throttle input (for testing)"""
	throttle_input = clamp(input, 0.0, 1.0)

func force_set_brake(input: float) -> void:
	"""Force set brake input (for testing)"""
	brake_input = clamp(input, 0.0, 1.0)

func force_set_steering(angle: float) -> void:
	"""Force set steering angle (for testing)"""
	steering_angle = clamp(angle, -max_steering_angle, max_steering_angle)

# ============================================================================
# SIGNAL HANDLERS
# ============================================================================
func _on_speed_changed(new_speed: float) -> void:
	"""Handle speed change signal"""
	pass

func _on_rpm_changed(new_rpm: float) -> void:
	"""Handle RPM change signal"""
	pass

func _on_gear_changed(old_gear: int, new_gear: int) -> void:
	"""Handle gear change signal"""
	pass

func _on_skidding(is_skidding: bool) -> void:
	"""Handle skidding signal"""
	pass

func _on_collision_detected(collision_info: Dictionary) -> void:
	"""Handle collision signal"""
	pass

func _on_engine_stalled() -> void:
	"""Handle engine stall signal"""
	pass

func _on_handbrake_toggled(is_active: bool) -> void:
	"""Handle handbrake toggle signal"""
	pass

func _on_traction_control_changed(active: bool) -> void:
	"""Handle traction control state change"""
	pass

func _on_abs_changed(active: bool) -> void:
	"""Handle ABS state change"""
	pass

func _on_drift_started(drift_angle: float) -> void:
	"""Handle drift start signal"""
	pass

func _on_drift_ended() -> void:
	"""Handle drift end signal"""
	pass

func _on_boost_used(duration: float) -> void:
	"""Handle boost usage signal"""
	pass

func _on_nitrous_active(is_active: bool) -> void:
	"""Handle nitrous active signal"""
	pass

func _on_vehicle_health_changed(current_health: float, max_health: float) -> void:
	"""Handle vehicle health change signal"""
	pass

# ============================================================================
# PRIVATE HELPERS
# ============================================================================
func _apply_drag_and_friction(delta: float) -> void:
	"""Apply drag and friction when engine is off"""
	_current_velocity *= 0.98  # Air resistance
	_current_velocity *= 0.95  # Rolling resistance
	if _current_velocity.length() < 0.1:
		_current_velocity = Vector3.ZERO

func _set_gravity(value: float) -> void:
	gravity = value
	EARTH_GRAVITY = gravity

func _set_physics_tick_rate(value: int) -> void:
	physics_tick_rate = value

func _set_max_substeps(value: int) -> void:
	max_substeps = value

func _set_time_scale(value: float) -> void:
	time_scale = value

func _set_default_vehicle_mass(value: float) -> void:
	default_vehicle_mass = value

func print_debug(message: String) -> void:
	"""Print debug message if debug mode is enabled"""
	if GameManager.debug_mode:
		print("[VehicleController] %s" % message)