extends Node2D
class_name VehicleController

## VehicleController - Core vehicle physics controller using PhysicsSettings constants
## Handles throttle, brake, steering inputs, wheel forces, and gear shifting logic
## Copyright 2026 Thalamus Racing Simulator Project

# Signals
signal engine_started
signal engine_stopped
signal gear_changed(old_gear: int, new_gear: int)
signal nitro_used(amount: float)
signal collision_detected(direction: Vector2)
signal speed_changed(current_speed: float, max_speed: float)
signal rpm_changed(current_rpm: float, max_rpm: float)
signal vehicle_moved(distance: float)
signal traction_control_toggled(active: bool)
signal anti_lock_brakes_toggled(active: bool)

# References
@onready var powertrain: Powertrain = $Powertrain if get_node_or_null("Powertrain") else null
@onready var chassis: Chassis = $Chassis if get_node_or_null("Chassis") else null
var _physics_body: CharacterBody2D = null

# State
enum VehicleState { IDLE, RUNNING, REVVING, BRAKING, COLLIDED, DRIFTING }
var current_vehicle_state: VehicleState = VehicleState.IDLE

# Physics body reference
var _vehicle_body: RigidBody2D = null

# Vehicle properties from PhysicsSettings
var _mass: float = 0.0
var _max_force: float = 0.0
var _engine_power: float = 0.0
var _braking_force: float = 0.0
var _turning_force: float = 0.0
var _max_steering_angle: float = 0.0
var _wheelbase_length: float = 0.0
var _track_width: float = 0.0

# Current performance metrics
var current_speed: float = 0.0
var current_rpm: float = 0.0
var current_gear: int = 0
var current_road_friction: float = 1.0
var drift_factor: float = 0.0

# Input values (normalized 0.0 to 1.0)
var throttle_input: float = 0.0
var brake_input: float = 0.0
var steering_input: float = 0.0
var nitro_input: float = 0.0
var clutch_input: float = 0.0

# Gear ratios (gear ratio = engine RPM / wheel RPM)
var _gear_ratios: Array[float] = []
var _final_drive_ratio: float = 0.0
var _transmission_type: String = "manual"

# Engine characteristics
var idle_rpm: float = 800.0
var redline_rpm: float = 7000.0
var optimal_power_rpm: float = 5500.0
var torque_curve: Dictionary = {}

# Traction control settings
var traction_control_enabled: bool = true
var antilock_braking_enabled: bool = true
var traction_slip_threshold: float = 0.2
var braking_max_pressure: float = 1000.0

# Nitrous system
var nitrous_available: float = 100.0
var nitrous_capacity: float = 100.0
var nitrous_depletion_rate: float = 10.0
var nitrous_cooldown_time: float = 10.0
var nitrous_active: bool = false
var _nitrous_timer: float = 0.0

# Drift mechanics
var drift_angle: float = 0.0
var drift_score: float = 0.0
var drift_multiplier: float = 1.0
var minimum_drift_speed: float = 30.0

# Wheel states
var front_wheel_angle: float = 0.0
var rear_wheel_angle: float = 0.0
var wheel_rotation_speed: float = 0.0

# Distance tracking
var total_distance_traveled: float = 0.0
var _last_position: Vector2 = Vector2.ZERO
var _frame_distance: float = 0.0

# Collision history
var recent_collisions: Array[Dictionary] = []
var collision_impact_threshold: float = 5.0

# Setup and initialization
func _ready() -> void:
	_init_physics_body()
	_load_settings()
	_calculate_gear_ratios()
	_connect_signals_to_systems()
	_setup_torque_curve()
	
	# Initialize position tracking
	_last_position = global_position
	
	print("[VehicleController] Initialized: %s" % name)

func _init_physics_body() -> void:
	# Try to find the physics body in different possible locations
	_physics_body = _find_child_with_type("CharacterBody2D")
	if not _physics_body:
		_physics_body = _find_child_with_type("RigidBody2D")
	
	if _physics_body:
		_mass = _physics_body.mass
	else:
		# Fallback to PhysicsSettings default
		_mass = PhysicsSettings.default_vehicle_mass
		print("[VehicleController] Warning: No physics body found, using default mass")

func _find_child_with_type(type_name: String) -> Node:
	for child in get_children():
		if type_name in str(child.get_class()):
			return child
	return null

func _load_settings() -> void:
	# Load vehicle-specific settings from PhysicsSettings
	_mass = PhysicsSettings.default_vehicle_mass
	_engine_power = PhysicsSettings.default_vehicle_mass * 0.5
	_braking_force = _mass * 9.81 * 2.0
	_turning_force = _engine_power * 0.3
	_max_steering_angle = deg_to_rad(30.0)
	_wheelbase_length = 2.5
	_track_width = 1.5
	
	# Transmission setup
	_final_drive_ratio = 3.73
	_gear_ratios = [0.0, 3.5, 2.5, 1.8, 1.3, 1.0, 0.8]  # Neutral, 1st through 6th, overdrive
	
	# Engine characteristics based on vehicle class
	idle_rpm = 800.0
	redline_rpm = 7000.0
	optimal_power_rpm = 5500.0
	
	# Nitrous system
	nitrous_available = nitrous_capacity
	nitrous_active = false
	_nitrous_timer = 0.0

func _calculate_gear_ratios() -> void:
	"""Calculate effective gear ratios including final drive"""
	var actual_ratios: Array[float] = []
	for gear_ratio in _gear_ratios:
		actual_ratios.append(gear_ratio * _final_drive_ratio)
	_gear_ratios = actual_ratios

func _connect_signals_to_systems() -> void:
	"""Connect internal signals to external systems"""
	if GameManager:
		game_state_changed.connect(_on_game_state_changed)
	
	if AudioManager:
		engine_started.connect(_on_engine_started)
		gear_changed.connect(_on_gear_changed)
		collision_detected.connect(_on_collision_event)

func _setup_torque_curve() -> void:
	"""Setup simplified torque curve based on engine characteristics"""
	torque_curve = {
		idle_rpm: 0.1,
		2000.0: 0.6,
		3000.0: 0.85,
		optimal_power_rpm: 1.0,
		6000.0: 0.95,
		redline_rpm: 0.85,
		redline_rpm + 500.0: 0.7
	}

func _process(delta: float) -> void:
	_update_inputs(delta)
	_handle_nitrous(delta)
	_update_drift(delta)
	_update_distance(delta)
	_update_display_data()

func _physics_process(delta: float) -> void:
	if not _physics_body:
		return
	
	_update_physics(delta)
	_apply_forces(delta)
	_check_vehicle_state()
	_update_wheels(delta)

func _update_inputs(delta: float) -> void:
	"""Read and process input values from InputManager"""
	if InputManager:
		throttle_input = clamp(InputManager.get_axis("accelerate"), 0.0, 1.0)
		brake_input = clamp(InputManager.get_axis("brake"), 0.0, 1.0)
		steering_input = clamp(InputManager.get_axis("steer_left") - InputManager.get_axis("steer_right"), -1.0, 1.0)
		clutch_input = clamp(InputManager.get_axis("clutch"), 0.0, 1.0)
		nitro_input = clamp(InputManager.get_axis("nitro"), 0.0, 1.0)
	else:
		# Default values if InputManager not available
		throttle_input = 0.0
		brake_input = 0.0
		steering_input = 0.0
		clutch_input = 0.0
		nitro_input = 0.0

func _handle_nitrous(delta: float) -> void:
	"""Handle nitrous activation and cooldown"""
	if nitrous_active:
		nitrous_available -= nitrous_depletion_rate * delta
		
		if nitrous_available <= 0.0:
			nitrous_active = false
			emit_signal("nitro_used", nitrous_capacity)
	
	elif nitrous_input > 0.5 and nitrous_available > 0.0:
		if _nitrous_timer <= 0.0:
			nitrous_active = true
			_nitrous_timer = nitrous_cooldown_time
			emit_signal("nitro_used", nitrous_capacity * 0.1)
	
	if _nitrous_timer > 0.0:
		_nitrous_timer -= delta

func _update_drift(delta: float) -> void:
	"""Update drift mechanics and scoring"""
	var speed_vector = _get_velocity_vector()
	var forward_vector = transform.basis.xform(Vector2.RIGHT).normalized()
	
	if current_speed > minimum_drift_speed:
		var angle_diff = abs(angle_difference(speed_vector.angle(), forward_vector.angle()))
		
		if angle_diff > deg_to_rad(15.0):
			current_vehicle_state = VehicleState.DRIFTING
			drift_angle = angle_diff
			drift_score += 0.5 * delta
			drift_multiplier = 1.0 + min(drift_score * 0.1, 0.5)
		else:
			current_vehicle_state = VehicleState.RUNNING
			drift_score = max(0.0, drift_score - delta * 0.3)
			drift_multiplier = 1.0
	else:
		current_vehicle_state = VehicleState.RUNNING
		drift_score = 0.0

func _update_distance(delta: float) -> void:
	"""Track distance traveled by the vehicle"""
	var current_position = global_position
	var frame_delta = (current_position - _last_position).length()
	_frame_distance = frame_delta
	total_distance_traveled += frame_delta
	_last_position = current_position
	
	if frame_delta > 0.01:
		emit_signal("vehicle_moved", frame_delta)

func _update_display_data() -> void:
	"""Update any display/UI data"""
	# This can be extended to update HUD elements
	pass

func _update_physics(delta: float) -> void:
	"""Main physics update for vehicle movement"""
	if not _physics_body:
		return
	
	# Calculate target velocity based on gear and engine output
	var target_velocity = _calculate_target_velocity()
	
	# Apply acceleration/deceleration
	var current_velocity = _physics_body.linear_velocity
	var velocity_magnitude = current_velocity.length()
	
	# Update current speed for signals
	current_speed = velocity_magnitude
	
	# Update RPM based on current gear and speed
	_current_rpm_update(delta)
	
	# Apply friction and drag
	_apply_drag_and_friction(delta)
	
	# Handle gear changes automatically if needed
	_handle_automatic_gearing(delta)

func _calculate_target_velocity() -> float:
	"""Calculate target velocity based on current gear and input"""
	if current_gear <= 0:
		return 0.0
	
	var gear_ratio = _gear_ratios[current_gear]
	var wheel_speed = current_rpm / gear_ratio
	
	# Convert wheel speed to vehicle speed
	var wheel_circumference = 0.6  # Approximate tire circumference
	var max_gear_speed = wheel_speed * wheel_circumference * gear_ratio
	
	# Apply throttle influence
	var throttle_factor = throttle_input
	if nitrous_active:
		throttle_factor *= 1.5
	
	# Brake reduces target speed
	if brake_input > 0.0:
		target_velocity = current_speed * (1.0 - brake_input * 0.3)
	else:
		target_velocity = max_gear_speed * throttle_factor
	
	return target_velocity

func _current_rpm_update(delta: float) -> void:
	"""Update current RPM based on vehicle state"""
	if current_gear <= 0:
		current_rpm = idle_rpm
		return
	
	var gear_ratio = _gear_ratios[current_gear]
	var wheel_speed = current_speed / (_gear_ratios[1]) if current_gear > 1 else current_speed
	
	current_rpm = wheel_speed * gear_ratio
	
	# Clamp to valid range
	current_rpm = clamp(current_rpm, idle_rpm, redline_rpm + 500.0)
	
	# Smooth transition
	current_rpm = lerp(current_rpm, current_rpm, 0.9)

func _apply_drag_and_friction(delta: float) -> void:
	"""Apply aerodynamic drag and rolling resistance"""
	if not _physics_body:
		return
	
	var velocity = _physics_body.linear_velocity
	var speed = velocity.length()
	
	if speed > 0.01:
		# Aerodynamic drag (F = 0.5 * Cd * A * rho * v^2)
		var drag_coefficient = 0.3
		var air_density = 1.225
		var frontal_area = 2.0
		var drag_force = 0.5 * drag_coefficient * frontal_area * air_density * speed * speed
		
		# Rolling resistance (F = Crr * m * g)
		var rolling_resistance = 0.015 * _mass * 9.81
		
		# Total resistive force
		var total_resistance = drag_force + rolling_resistance
		var resistance_acceleration = total_resistance / _mass
		
		# Apply deceleration opposite to velocity direction
		if speed > 0.01:
			var decel_direction = -velocity.normalized()
			var decel = min(resistance_acceleration * delta, speed)
			_physics_body.apply_central_impulse(decel_direction * decel * _mass)

func _handle_automatic_gearing(delta: float) -> void:
	"""Handle automatic upshifts and downshifts based on RPM"""
	if _transmission_type != "automatic":
		return
	
	# Upshift condition
	if current_rpm >= optimal_power_rpm and current_gear < _gear_raties.size() - 1:
		shift_up()
	
	# Downshift condition (with hysteresis)
	if current_rpm < idle_rpm + 500.0 and current_gear > 0:
		shift_down()

func _apply_forces(delta: float) -> void:
	"""Apply wheel forces to the vehicle body"""
	if not _physics_body or not powertrain:
		return
	
	# Calculate engine torque based on RPM
	var engine_torque = _calculate_engine_torque()
	
	# Apply torque based on current gear
	if current_gear > 0 and throttle_input > 0.01:
		var gear_ratio = _gear_ratios[current_gear]
		var wheel_torque = engine_torque * gear_ratio * 0.85  # 15% drivetrain loss
		
		# Distribute torque to wheels (simplified rear-wheel drive)
		var drive_force = wheel_torque / 0.3  # Simplified wheel radius
		var traction_force = drive_force * current_road_friction * drift_multiplier
		
		# Apply forward force
		var forward_dir = transform.basis.xform(Vector2.RIGHT).normalized()
		if current_vehicle_state == VehicleState.DRIFTING:
			forward_dir = _get_velocity_vector().normalized()
		
		_physics_body.apply_central_force(forward_dir * traction_force)
	
	# Apply braking force
	if brake_input > 0.0:
		var braking_effectiveness = 0.8 if antilock_braking_enabled else 1.0
		var brake_force = _braking_force * brake_input * braking_effectiveness
		var brake_dir = -transform.basis.xform(Vector2.RIGHT).normalized()
		_physics_body.apply_central_force(brake_dir * brake_force)
	
	# Steering force (applied as rotation)
	if abs(steering_input) > 0.01:
		_front_wheel_angle_update(steering_input)
		_turning_force_application(delta)

func _calculate_engine_torque() -> float:
	"""Calculate engine torque based on RPM using torque curve"""
	var torque_factor = 0.5  # Base torque factor
	
	# Find best match in torque curve
	var closest_rpm = idle_rpm
	var min_diff = abs(idle_rpm - current_rpm)
	
	for rpm_key in torque_curve:
		var diff = abs(rpm_key - current_rpm)
		if diff < min_diff:
			min_diff = diff
			closest_rpm = rpm_key
	
	var base_torque = torque_curve.get(closest_rpm, 0.5)
	
	# Add nitrous boost
	if nitrous_active:
		base_torque *= 1.3
	
	# Calculate final torque
	return base_torque * _engine_power * torque_factor

func _front_wheel_angle_update(input: float) -> void:
	"""Update front wheel steering angle"""
	front_wheel_angle = input * _max_steering_angle
	back_wheel_angle = input * _max_steering_angle * 0.3  # Ackermann steering approximation

func _turning_force_application(delta: float) -> void:
	"""Apply turning force to change vehicle direction"""
	if not _physics_body:
		return
	
	var turn_force = _turning_force * abs(steering_input) * delta
	var turn_direction = steering_input
	
	# Apply rotational force
	var angular_velocity = _physics_body.angular_velocity
	var target_angular_velocity = turn_direction * turn_force
	
	# Smooth angular velocity change
	var new_angular_velocity = lerp(angular_velocity, target_angular_velocity, 0.1)
	_physics_body.angular_velocity = new_angular_velocity

func _check_vehicle_state() -> void:
	"""Update vehicle state based on conditions"""
	if current_rpm <= idle_rpm + 100.0 and throttle_input < 0.1:
		current_vehicle_state = VehicleState.IDLE
	elif brake_input > 0.5:
		current_vehicle_state = VehicleState.BRAKING
	elif current_vehicle_state != VehicleState.DRIFTING and current_speed < minimum_drift_speed:
		current_vehicle_state = VehicleState.RUNNING
	elif current_rpm > redline_rpm * 0.9:
		current_vehicle_state = VehicleState.REVVING

func _update_wheels(delta: float) -> void:
	"""Update wheel visual state"""
	if not chassis:
		return
	
	# Update wheel rotation visualization
	wheel_rotation_speed = current_rpm / 60.0 * 2.0  # Convert RPM to rotations per second

func shift_up() -> void:
	"""Shift transmission up one gear"""
	if current_gear < _gear_ratios.size() - 1 and current_gear > 0:
		var old_gear = current_gear
		current_gear += 1
		emit_signal("gear_changed", old_gear, current_gear)
		
		if AudioManager:
			AudioManager.play_sound("gear_shift_up")

func shift_down() -> void:
	"""Shift transmission down one gear"""
	if current_gear > 0:
		var old_gear = current_gear
		current_gear -= 1
		emit_signal("gear_changed", old_gear, current_gear)
		
		if AudioManager:
			AudioManager.play_sound("gear_shift_down")

func start_engine() -> void:
	"""Start the vehicle engine"""
	if current_rpm < idle_rpm:
		current_rpm = idle_rpm
		current_vehicle_state = VehicleState.IDLE
		emit_signal("engine_started")
		
		if AudioManager:
			AudioManager.play_sound("engine_start")

func stop_engine() -> void:
	"""Stop the vehicle engine"""
	current_rpm = 0.0
	current_vehicle_state = VehicleState.IDLE
	current_gear = 0
	emit_signal("engine_stopped")
	
	if AudioManager:
		AudioManager.play_sound("engine_stop")

func apply_collision(force: Vector2, impact_velocity: float) -> void:
	"""Handle collision event"""
	current_vehicle_state = VehicleState.COLLIDED
	
	recent_collisions.append({
		"force": force,
		"impact_velocity": impact_velocity,
		"time": Time.get_unix_time_from_system()
	})
	
	# Keep only recent collisions
	if recent_collisions.size() > 10:
		recent_collisions.pop_front()
	
	emit_signal("collision_detected", force)
	
	# Brief slow down after collision
	if _physics_body:
		var current_vel = _physics_body.linear_velocity
		_physics_body.linear_velocity = current_vel * 0.7

func set_road_friction(friction: float) -> void:
	"""Set road surface friction coefficient"""
	current_road_friction = clamp(friction, 0.1, 2.0)

func reset_vehicle() -> void:
	"""Reset vehicle to starting state"""
	current_rpm = idle_rpm
	current_speed = 0.0
	current_gear = 0
	current_vehicle_state = VehicleState.IDLE
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	drift_score = 0.0
	_nitrous_timer = 0.0
	nitrous_active = false
	nitrous_available = nitrous_capacity

func _get_velocity_vector() -> Vector2:
	"""Get current velocity vector"""
	if _physics_body:
		return _physics_body.linear_velocity
	return Vector2.ZERO

func _angle_difference(a1: float, a2: float) -> float:
	"""Calculate shortest angle difference between two angles"""
	var diff = fmod(a1 - a2 + PI, 2.0 * PI)
	if diff > PI:
		diff -= 2.0 * PI
	return abs(diff)

# External API calls for other systems
func get_status() -> Dictionary:
	"""Get comprehensive vehicle status"""
	return {
		"speed": current_speed,
		"rpm": current_rpm,
		"gear": current_gear,
		"state": current_vehicle_state,
		"throttle": throttle_input,
		"brake": brake_input,
		"steering": steering_input,
		"nitrous_available": nitrous_available,
		"total_distance": total_distance_traveled,
		"drift_score": drift_score
	}

func set_transmission_type(type: String) -> void:
	"""Set transmission type (manual/automatic)"""
	_transmission_type = type

func set_vehicle_parameters(mass: float, engine_power: float) -> void:
	"""Set custom vehicle parameters"""
	_mass = mass
	_engine_power = engine_power
	_braking_force = _mass * 9.81 * 2.0

func _on_game_state_changed(new_state: GameState) -> void:
	"""Handle game state changes"""
	match new_state:
		GameManager.GameState.RACE_ACTIVE:
			start_engine()
		GameManager.GameState.RACE_PAUSED:
			stop_engine()
		GameManager.GameState.RACE_FINISHED:
			reset_vehicle()

func _on_engine_started() -> void:
	"""Handle engine started signal"""
	print("[VehicleController] Engine Started")

func _on_gear_changed(old_gear: int, new_gear: int) -> void:
	"""Handle gear change signal"""
	print("[VehicleController] Gear changed: %d -> %d" % [old_gear, new_gear])

func _on_collision_event(direction: Vector2) -> void:
	"""Handle collision event"""
	print("[VehicleController] Collision detected!")

func _to_string() -> String:
	"""Return string representation of vehicle state"""
	return "VehicleController{speed=%.1f, rpm=%.0f, gear=%d, state=%s}" % [
		current_speed, current_rpm, current_gear, 
		VehicleController.VehicleState.keys()[current_vehicle_state]
	]