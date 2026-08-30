extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulation
## Handles throttle, brake, steering, wheel forces, gear shifting, and vehicle dynamics
## Copyright 2026 Thalamus Racing Simulator Project

signal speed_changed(speed: float)
signal gear_changed(old_gear: int, new_gear: int)
signal drift_started(drift_intensity: float)
signal drift_ended()
signal engine_rpm_changed(rpm: float)
signal impact_detected(impact_force: Vector3)
signal wheel_slip(wheel_index: int, slip_ratio: float)
signal collision_event(collision_data: Dictionary)

@export_group("Vehicle Configuration")
@export var vehicle_mass: float = 1500.0: set = _set_vehicle_mass
@export var center_of_mass_offset: Vector3 = Vector3.ZERO
@export var wheelbase: float = 2.8
@export var track_width: float = 1.6
@export var ground_clearance: float = 0.15
@export var max_steering_angle: float = 35.0 * TAU / 180.0
@export var air_resistance_coefficient: float = 0.3
@export var rolling_resistance_coefficient: float = 0.015

@export_group("Engine & Powertrain")
@export var engine_max_rpm: float = 7500.0
@export var engine_min_idle_rpm: float = 800.0
@export var peak_torque_rpm: float = 4200.0
@export var max_engine_torque: float = 450.0
@export var torque_curve_enabled: bool = true

@export_group("Transmission")
@export var transmission_type: TransmissionType = TransmissionType.MANUAL
@export var final_drive_ratio: float = 3.5
@export var gear_ratios: Array[float] = [3.9, 2.4, 1.6, 1.2, 0.9, 0.7, 0.6]
@export var reverse_gear_ratio: float = -3.8
@export var clutch_disengage_rpm: float = 150.0

@export_group("Braking System")
@export var front_brake_bias: float = 0.6
@export var max_brake_pressure: float = 100.0
@export var brake_disc_radius: float = 0.3
@export var brake_pad_friction: float = 0.4
@export var abs_enabled: bool = true
@export var brake_balance_adjustment: float = 0.0

@export_group("Suspension")
@export var suspension_stiffness: float = 80000.0
@export var suspension_damping: float = 8000.0
@export var suspension_compression_limit: float = 0.35
@export var suspension_extension_limit: float = 0.45
@export var ride_height: float = 0.25
@export var anti_roll_bar_stiffness: float = 25000.0

@export_group("Tires")
@export var tire_friction_coefficient: float = 1.2
@export var tire_lateral_stiffness: float = 45000.0
@export var tire_longitudinal_stiffness: float = 50000.0
@export var tire_width: float = 0.25
@export var tire_radius: float = 0.33
@export var tire_pressure: float = 2.2

@export_group("Drift Mechanics")
@export var drift_enabled: bool = true
@export var handbrake_lock_threshold: float = 0.7
@export var drift_recovery_rate: float = 0.15
@export var drift_score_multiplier: float = 1.5

@export_group("AI Control")
@export var ai_enabled: bool = false
@export var ai_aggressiveness: float = 0.8
@export var ai_tracking_distance: float = 15.0
@export var ai_target_prediction_time: float = 1.0

enum TransmissionType {
	MANUAL,
	AUTOMATIC,
	SEMI_AUTOMATIC,
	CVT
}

# Physics state variables
var current_speed: float = 0.0
var current_velocity: Vector3 = Vector3.ZERO
var angular_velocity: Vector3 = Vector3.ZERO
var current_rpm: float = 0.0
var current_gear: int = 0
var target_gear: int = 0
var clutch_engaged: bool = true
var throttle_input: float = 0.0
var brake_input: float = 0.0
var steering_input: float = 0.0
var handbrake_input: float = 0.0
var drift_intensity: float = 0.0
var drift_angle: float = 0.0

# Wheel state
var wheel_positions: Array[Vector3] = []
var wheel_rotation_angles: Array[float] = []
var wheel_slip_ratios: Array[float] = []
var wheel_vertical_forces: Array[float] = []

# Engine torque curve data
var torque_curve_points: Array[Vector2] = []

# Game time tracking
var lap_time: float = 0.0
var last_update_time: float = 0.0
var delta_time: float = 0.0

func _ready() -> void:
	_init_wheel_positions()
	_init_torque_curve()
	_reset_physics_state()
	
	# Connect to GameManager signals if available
	if GameManager.has_signal("game_state_changed"):
		GameManager.game_state_changed.connect(_on_game_state_changed)

func _process(delta: float) -> void:
	delta_time = delta
	last_update_time = Time.get_unix_time_from_system()
	
	if not PhysicsSettings.time_scale.is_equal_approx(1.0):
		delta *= PhysicsSettings.time_scale
	
	_update_physics(delta)
	_update_audio()
	_update_debug_visualization()

func _physics_process(delta: float) -> void:
	if not PhysicsSettings.time_scale.is_equal_approx(1.0):
		delta *= PhysicsSettings.time_scale
	
	_apply_gravity(delta)
	_apply_driving_forces(delta)
	_apply_aerodynamics(delta)
	_update_suspension(delta)
	_handle_drifting(delta)
	_update_gear_shifting(delta)
	_sync_body_to_wheels()

func _init_wheel_positions() -> void:
	wheel_positions.resize(4)
	wheel_rotation_angles.resize(4)
	wheel_slip_ratios.resize(4)
	wheel_vertical_forces.resize(4)
	
	var half_track = track_width / 2.0
	var front_axle_z = wheelbase / 2.0
	var rear_axle_z = -wheelbase / 2.0
	
	# Front left
	wheel_positions[0] = Vector3(-half_track, -ground_clearance, front_axle_z)
	# Front right
	wheel_positions[1] = Vector3(half_track, -ground_clearance, front_axle_z)
	# Rear left
	wheel_positions[2] = Vector3(-half_track, -ground_clearance, rear_axle_z)
	# Rear right
	wheel_positions[3] = Vector3(half_track, -ground_clearance, rear_axle_z)

func _init_torque_curve() -> void:
	if not torque_curve_enabled:
		torque_curve_points.append(Vector2(0.0, 0.0))
		torque_curve_points.append(Vector2(engine_max_rpm, max_engine_torque))
		return
	
	# Generate realistic torque curve
	var num_points = 20
	for i in range(num_points + 1):
		var rpm_normalized = float(i) / float(num_points)
		var rpm = engine_min_idle_rpm + (engine_max_rpm - engine_min_idle_rpm) * rpm_normalized
		
		# Bell-shaped torque curve around peak torque RPM
		var torque = max_engine_torque * exp(-pow((rpm - peak_torque_rpm), 2) / (peak_torque_rpm * 0.3))
		torque_curve_points.append(Vector2(rpm, clamp(torque, 0.0, max_engine_torque)))

func _reset_physics_state() -> void:
	current_speed = 0.0
	current_velocity = Vector3.ZERO
	angular_velocity = Vector3.ZERO
	current_rpm = engine_min_idle_rpm
	current_gear = 0
	target_gear = 0
	clutch_engaged = true
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	handbrake_input = 0.0
	drift_intensity = 0.0
	drift_angle = 0.0

func _update_physics(delta: float) -> void:
	# Update velocity from movement
	current_velocity = linear_interpolate(current_velocity, global_velocity, 0.1)
	current_speed = current_velocity.length()
	
	# Apply gravity
	_apply_gravity(delta)
	
	# Calculate driving forces
	_apply_driving_forces(delta)
	
	# Apply aerodynamic drag
	_apply_aerodynamics(delta)
	
	# Update rotation based on steering and velocity
	_update_rotation(delta)
	
	# Move character body
	move_and_slide()

func _apply_gravity(delta: float) -> void:
	var gravity_vector = Vector3.UP * PhysicsSettings.gravity * -vehicle_mass
	
	add_force(gravity_vector)

func _apply_driving_forces(delta: float) -> void:
	if not clutch_engaged:
		return
	
	var wheel_forward = Vector3.FORWARD.rotated(Vector3.UP, get_global_rotation().y)
	var wheel_right = Vector3.RIGHT.rotated(Vector3.UP, get_global_rotation().y)
	
	# Calculate drive wheels based on drivetrain (simplified FWD/RWD/AWD)
	var drive_wheels: Array[int] = [2, 3] # RWD default
	
	# Apply throttle force
	var engine_force = _calculate_engine_force()
	
	if throttle_input > 0.0 and clutch_engaged:
		var wheel_force = engine_force * throttle_input * delta
		
		for wheel_idx in drive_wheels:
			var wheel_pos = global_position + wheel_positions[wheel_idx].rotated(Vector3.UP, get_global_rotation().y)
			var force_direction = wheel_forward
			add_force(force_direction * wheel_force, wheel_pos)
			
			# Apply wheel rotation visual effect
			wheel_rotation_angles[wheel_idx] += (current_speed / tire_radius) * delta
			
			# Track wheel slip
			var wheel_speed = current_speed * (1.0 if wheel_idx < 2 else 1.0)
			var slip_ratio = abs(wheel_speed - (current_rpm / 60.0 * 2.0 * PI * tire_radius)) / max(1.0, current_speed)
			wheel_slip_ratios[wheel_idx] = slip_ratio
			
			if slip_ratio > 0.3:
				emit_signal("wheel_slip", wheel_idx, slip_ratio)
	
	# Apply braking force
	var brake_force = _calculate_brake_force()
	
	if brake_input > 0.0 or handbrake_input > 0.0:
		var total_brake = brake_input + (handbrake_input * 0.5)
		var effective_brake = min(total_brake, 1.0) * max_brake_pressure
		
		for wheel_idx in range(4):
			var wheel_pos = global_position + wheel_positions[wheel_idx].rotated(Vector3.UP, get_global_rotation().y)
			var force_direction = -wheel_forward
			add_force(force_direction * brake_force * effective_brake * 0.25, wheel_pos)
			
			# Handbrake locks rear wheels
			if handbrake_input > 0.0 and wheel_idx >= 2:
				wheel_rotation_angles[wheel_idx] = 0.0

func _calculate_engine_force() -> float:
	var gear_ratio = _get_current_gear_ratio()
	var total_ratio = gear_ratio * final_drive_ratio
	
	# Calculate engine RPM based on vehicle speed and gear
	var wheel_rpm = (current_speed / (2.0 * PI * tire_radius)) * 60.0
	var calculated_rpm = wheel_rpm * total_ratio
	
	# Clamp RPM to valid range
	calculated_rpm = clamp(calculated_rpm, engine_min_idle_rpm, engine_max_rpm)
	
	# Get torque from curve
	var torque = _get_torque_at_rpm(calculated_rpm)
	
	# Convert torque to force at wheel
	var wheel_torque = torque * total_ratio * 0.9 # 10% drivetrain loss
	var wheel_force = wheel_torque / tire_radius
	
	return wheel_force

func _calculate_brake_force() -> float:
	var base_brake_force = max_brake_pressure * brake_disc_radius * brake_pad_friction
	
	# Adjust for ABS
	if abs_brake_enabled and current_speed > 5.0:
		base_brake_force *= 0.8
	
	return base_brake_force

func _apply_aerodynamics(delta: float) -> void:
	var air_density = 1.225 # kg/m^3 at sea level
	var frontal_area = track_width * 1.5 # Approximate car height
	
	var drag_force = 0.5 * air_density * air_resistance_coefficient * frontal_area * current_speed * current_speed
	
	var drag_direction = -current_velocity.normalized() if current_speed > 0.0 else Vector3.ZERO
	add_force(drag_direction * drag_force)

func _update_suspension(delta: float) -> void:
	# Simplified suspension calculation
	# In production, this would use raycasts to detect ground contact
	
	for wheel_idx in range(4):
		var wheel_world_pos = global_position + wheel_positions[wheel_idx].rotated(Vector3.UP, get_global_rotation().y)
		
		# Raycast down to find ground distance
		var space_state = get_world_3d().direct_space_state
		var query = PhysicsRayQueryParameters3D.create(global_position, wheel_world_pos + Vector3.DOWN * 10.0)
		var result = space_state.intersect_ray(query)
		
		if result.has("collider"):
			var distance_to_ground = (wheel_world_pos - result.position).length()
			var compression = ride_height - distance_to_ground
			
			# Clamp compression
			compression = clamp(compression, -suspension_extension_limit, suspension_compression_limit)
			
			# Spring force
			var spring_force = compression * suspension_stiffness
			
			# Damping force
			var damping_factor = 0.1 # Simplified
			wheel_vertical_forces[wheel_idx] = spring_force * damping_factor

func _handle_drifting(delta: float) -> void:
	if not drift_enabled:
		drift_intensity = lerp(drift_intensity, 0.0, delta * drift_recovery_rate)
		return
	
	var turn_input = abs(steering_input)
	var lateral_velocity = current_velocity.cross(get_up()).dot(get_transform().basis.x)
	
	# Detect drift initiation
	if handbrake_input > handbrake_lock_threshold and abs(lateral_velocity) > 3.0:
		drift_intensity = min(drift_intensity + delta * 0.1, 1.0)
		
		if drift_intensity > 0.5 and drift_angle == 0.0:
			emit_signal("drift_started", drift_intensity)
	
	# Maintain drift angle
	if drift_intensity > 0.0:
		drift_angle = lerp(drift_angle, steering_input * 0.5, delta * 0.1)
	else:
		drift_angle = lerp(drift_angle, 0.0, delta * drift_recovery_rate)
		
		drift_intensity = lerp(drift_intensity, 0.0, delta * drift_recovery_rate)

func _update_gear_shifting(delta: float) -> void:
	match transmission_type:
		TransmissionType.MANUAL:
			_handle_manual_shift()
		TransmissionType.AUTOMATIC:
			_handle_automatic_shift()
		TransmissionType.SEMI_AUTOMATIC:
			_handle_semi_auto_shift()
		TransmissionType.CVT:
			_handle_cvt_behavior()
	
	# Update RPM display
	emit_signal("engine_rpm_changed", current_rpm)

func _handle_manual_shift() -> void:
	# Shift up/down based on input (handled by InputManager)
	pass

func _handle_automatic_shift() -> void:
	var optimal_gear = _calculate_optimal_gear()
	
	if optimal_gear != current_gear:
		var old_gear = current_gear
		current_gear = optimal_gear
		target_gear = optimal_gear
		emit_signal("gear_changed", old_gear, current_gear)

func _calculate_optimal_gear() -> int:
	var speed = current_speed * 3.6 # Convert m/s to km/h
	var gear_count = gear_ratios.size()
	
	# Simple gear mapping based on speed
	if speed < 10.0:
		return 0
	elif speed < 25.0:
		return 1
	elif speed < 45.0:
		return 2
	elif speed < 70.0:
		return 3
	elif speed < 100.0:
		return 4
	elif speed < 140.0:
		return 5
	elif speed < 180.0:
		return 6
	else:
		return min(gear_count - 1, 6)

func _handle_semi_auto_shift() -> void:
	# Similar to automatic but player can override
	_handle_automatic_shift()

func _handle_cvt_behavior() -> void:
	# CVT maintains optimal RPM
	var target_rpm = peak_torque_rpm
	var current_ratio = current_rpm / target_rpm
	
	# Simulate CVT ratio adjustment
	pass

func _sync_body_to_wheels() -> void:
	# Sync visual wheel rotation with physics
	pass

func _get_current_gear_ratio() -> float:
	if current_gear >= 0 and current_gear < gear_ratios.size():
		return gear_ratios[current_gear]
	elif current_gear == -1:
		return reverse_gear_ratio
	else:
		return gear_ratios.back()

func _get_torque_at_rpm(rpm: float) -> float:
	if not torque_curve_enabled:
		return max_engine_torque
	
	if rpm <= engine_min_idle_rpm:
		return max_engine_torque * 0.3
	
	var torque_value = 0.0
	for i in range(torque_curve_points.size() - 1):
		var point1 = torque_curve_points[i]
		var point2 = torque_curve_points[i + 1]
		
		if rpm >= point1.x and rpm <= point2.x:
			var t = (rpm - point1.x) / (point2.x - point1.x)
			torque_value = point1.y + t * (point2.y - point1.y)
			break
	
	return torque_value

func _set_vehicle_mass(value: float) -> void:
	vehicle_mass = value
	rebuild_collision_shape()

func rebuild_collision_shape() -> void:
	pass

func _on_game_state_changed(new_state: GameState) -> void:
	match new_state:
		GameState.RACE_ACTIVE:
			_reset_physics_state()
		GameState.RACE_PAUSED:
			process_mode = ProcessModeEnum.DISABLED
		GameState.MAIN_MENU, GameState.RACE_FINISHED:
			process_mode = ProcessModeEnum.ALWAYS

func _update_audio() -> void:
	# Trigger audio events based on vehicle state
	if AudioManager:
		var engine_volume = lerp(0.3, 1.0, current_rpm / engine_max_rpm)
		AudioManager.play_sound("engine_loop", engine_volume)
		
		if throttle_input > 0.8:
			AudioManager.play_sound("throttle_response", 0.5)
		
		if handbrake_input > 0.5:
			AudioManager.play_sound("tire_squeal", 0.7)

func _update_debug_visualization() -> void:
	if not GameManager.debug_mode:
		return
	
	# Draw debug lines for wheel positions
	for wheel_idx in range(4):
		var wheel_world_pos = global_position + wheel_positions[wheel_idx].rotated(Vector3.UP, get_global_rotation().y)
		DebugDraw.draw_sphere(wheel_world_pos, 0.1, Color.RED)
		
		var slip_color = Color.GREEN
		if wheel_slip_ratios[wheel_idx] > 0.3:
			slip_color = Color.YELLOW
		if wheel_slip_ratios[wheel_idx] > 0.5:
			slip_color = Color.RED
		
		DebugDraw.draw_line(wheel_world_pos, wheel_world_pos + Vector3.UP * 0.5, slip_color, 0.02)

func _input(event: InputEvent) -> void:
	if event.is_action_pressed("vehicle_throttle_up"):
		throttle_input = min(throttle_input + 0.05, 1.0)
	elif event.is_action_released("vehicle_throttle_up"):
		throttle_input = max(throttle_input - 0.05, 0.0)
	
	if event.is_action_pressed("vehicle_brake"):
		brake_input = 1.0
	elif event.is_action_released("vehicle_brake"):
		brake_input = 0.0
	
	if event.is_action_pressed("vehicle_handbrake"):
		handbrake_input = 1.0
	elif event.is_action_released("vehicle_handbrake"):
		handbrake_input = 0.0

func set_transmission_type(type: TransmissionType) -> void:
	transmission_type = type

func set_gear(gear: int) -> void:
	if gear < -1 or gear >= gear_ratios.size():
		return
	
	target_gear = gear
	if clutch_engaged:
		current_gear = gear
		emit_signal("gear_changed", current_gear, gear)

func engage_clutch() -> void:
	clutch_engaged = true

func disengage_clutch() -> void:
	clutch_engaged = false

func reset_controls() -> void:
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	handbrake_input = 0.0

func get_speed_kmh() -> float:
	return current_speed * 3.6

func get_rpm_percentage() -> float:
	return current_rpm / engine_max_rpm

func get_drift_status() -> Dictionary:
	return {
		"active": drift_intensity > 0.1,
		"intensity": drift_intensity,
		"angle": drift_angle
	}

func apply_impact(impact_force: Vector3) -> void:
	emit_signal("impact_detected", impact_force)
	
	# Add screen shake effect
	if GameManager:
		GameManager.trigger_screen_shake(impact_force.length() * 0.01)

func save_state() -> Dictionary:
	return {
		"speed": current_speed,
		"rpm": current_rpm,
		"gear": current_gear,
		"throttle": throttle_input,
		"brake": brake_input,
		"steering": steering_input,
		"handbrake": handbrake_input,
		"position": global_position,
		"rotation": global_rotation
	}

func load_state(state: Dictionary) -> void:
	current_speed = state.get("speed", 0.0)
	current_rpm = state.get("rpm", engine_min_idle_rpm)
	current_gear = state.get("gear", 0)
	throttle_input = state.get("throttle", 0.0)
	brake_input = state.get("brake", 0.0)
	steering_input = state.get("steering", 0.0)
	handbrake_input = state.get("handbrake", 0.0)
	global_position = state.get("position", global_position)
	global_rotation = state.get("rotation", global_rotation)

func _to_dict() -> Dictionary:
	return {
		"class": "VehicleController",
		"mass": vehicle_mass,
		"transmission": transmission_type,
		"max_rpm": engine_max_rpm,
		"gears": gear_ratios
	}