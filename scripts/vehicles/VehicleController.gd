extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulation
## Handles throttle, brake, steering, wheel forces, gear shifting, and vehicle dynamics
## Implements realistic car physics with powertrain integration and suspension modeling
## Copyright 2026 Thalamus Racing Simulator Project

# ============================================================================
# SIGNALS
# ============================================================================
signal speed_changed(new_speed: float)
signal rpm_changed(new_rpm: float)
signal gear_changed(old_gear: int, new_gear: int)
signal drift_started(drift_angle: float)
signal drift_ended()
signal collision_impact(impact_force: Vector3, impact_point: Vector3)
signal wheel_slip(wheel_index: int, slip_ratio: float)

# ============================================================================
# SINGLETON REFERENCES
# ============================================================================
var physics_settings: PhysicsSettings = null
var game_manager: GameManager = null
var audio_manager: AudioManager = null

# ============================================================================
# VEHICLE CONFIGURATION - EXPORT GROUPS
# ============================================================================
@export_group("Vehicle Configuration")
@export var mass: float = 1500.0: set = _set_mass
@export var center_of_mass: Vector3 = Vector3.ZERO
@export var width: float = 1.9
@export var length: float = 4.5
@export var height: float = 1.4
@export var drag_coefficient: float = 0.32
@export var frontal_area: float = 2.2
@export var roll_center_height: float = 0.3
@export var pitch_damping: float = 5000.0
@export var yaw_damping: float = 8000.0

@export_group("Suspension Geometry")
@export var front_track: float = 1.6
@export var rear_track: float = 1.6
@export var front_suspension_travel: float = 0.15
@export var rear_suspension_travel: float = 0.18
@export var spring_rate_front: float = 45000.0
@export var spring_rate_rear: float = 48000.0
@export var damper_rate_front: float = 3500.0
@export var damper_rate_rear: float = 3800.0
@export var ride_height: float = 0.12
@export var camber_gain_front: float = 0.003
@export var camber_gain_rear: float = 0.0025
@export var toe_in_front: float = 0.002
@export var toe_in_rear: float = 0.0015

@export_group("Aerodynamics")
@export var downforce_coefficient: float = 0.45
@export var lift_coefficient: float = -0.12
@export var aero_center_x: float = 0.0
@export var aero_center_y: float = 0.35
@export var aero_center_z: float = 0.0
@export var wing_angle_front: float = 0.0
@export var wing_angle_rear: float = 12.0 * TAU / 360.0

# ============================================================================
# POWERTRAIN PARAMETERS - EXPORT GROUPS
# ============================================================================
@export_group("Engine Specifications")
@export var engine_type: String = "V8"
@export var max_rpm: float = 7500.0
@export var idle_rpm: float = 800.0
@export var peak_torque_rpm: float = 4500.0
@export var peak_torque: float = 550.0
@export var redline_rpm: float = 7800.0
@export var torque_curve_points: Array[Vector2] = [
	Vector2(0, 0),
	Vector2(1500, 0.3),
	Vector2(3000, 0.7),
	Vector2(4500, 1.0),
	Vector2(6000, 0.85),
	Vector2(7500, 0.6)
]
@export var friction_losses: float = 0.12

@export_group("Transmission Settings")
@export var transmission_type: String = "manual"
@export var final_drive_ratio: float = 3.73
@export var gear_ratios: Array[float] = [3.65, 2.05, 1.40, 1.05, 0.85, 0.70]
@export var reverse_ratio: float = 3.25
@export var num_gears: int = 6
@export var clutch_engagement_rpm: float = 1200.0
@export var clutch_release_threshold: float = 0.1

@export_group("Clutch System")
@export var clutch_friction_coefficient: float = 0.35
@export var clutch_pressure_plate_force: float = 4500.0
@export var clutch_disengagement_time: float = 0.15
@export var clutch_engagement_time: float = 0.2
@export var flywheel_inertia: float = 0.15
@export var drivetrain_efficiency: float = 0.88

# ============================================================================
# BRAKING SYSTEM - EXPORT GROUPS
# ============================================================================
@export_group("Brake Configuration")
@export var brake_disc_radius: float = 0.14
@export var brake_pad_area: float = 0.008
@export var brake_pad_friction: float = 0.45
@export var brake_master_cylinder_ratio: float = 1.25
@export var brake_line_loss: float = 0.08
@export var brake_balance: float = 0.52
@export var abs_enabled: bool = true
@export var abs_threshold: float = 0.15
@export var brake_bleed_rate: float = 0.98

@export_group("Brake Bias Front/Rear")
@export var brake_bias_front: float = 0.52
@export var brake_bias_rear: float = 0.48

# ============================================================================
# SUSPENSION & TIRES - EXPORT GROUPS
# ============================================================================
@export_group("Tire Properties")
@export var tire_width: float = 0.28
@export var tire_radius: float = 0.32
@export var tire_stiffness: float = 180000.0
@export var tire_friction_peak: float = 1.1
@export var tire_friction_slide: float = 0.8
@export var tire_side_wall_compliance: float = 0.02
@export var tire_max_steering_angle: float = 35.0 * TAU / 360.0
@export var tire_load_sensitivity: float = 0.1

@export_group("Wheel Alignment")
@export var caster_angle: float = 10.0 * TAU / 360.0
@export var kingpin_inclination: float = 12.0 * TAU / 360.0
@export var scrub_radius: float = 0.03

# ============================================================================
# STATE TRACKING VARIABLES
# ============================================================================
var current_speed: float = 0.0
var current_rpm: float = 0.0
var current_gear: int = 0
var target_gear: int = 0
var throttle_input: float = 0.0
var brake_input: float = 0.0
var steering_input: float = 0.0
var clutch_input: float = 1.0
var handbrake_input: float = 0.0
var drift_angle: float = 0.0
var drift_state: bool = false
var wheel_states: Array[Dictionary] = []
var suspension_states: Array[Dictionary] = []
var acceleration_history: Array[float] = []
var last_collision_time: float = 0.0

# ============================================================================
# PRIVATE HELPER VARIABLES
# ============================================================================
var _wheel_positions: Array[Vector3] = []
var _suspension_rest_lengths: Array[float] = []
var _current_wheel_angles: Array[float] = []
var _engine_braking_factor: float = 0.0
var _torque_converter_slip: float = 0.0
var _clutch_slip_ratio: float = 0.0
var _vehicle_rotation: float = 0.0
var _last_delta: float = 0.0
var _substep_accumulator: float = 0.0
var _fixed_step: float = 0.0
var _wheel_velocity: Array[float] = []
var _drift_threshold: float = 10.0
var _drift_recovery_rate: float = 0.5
var _airborne: bool = false
var _ground_contact_count: int = 0
var _collision_buffer: Array[Dictionary] = []
var _audio_context: AudioStreamPlayer = null

# ============================================================================
# INITIALIZATION
# ============================================================================
func _ready() -> void:
	_init_physics_settings()
	_init_vehicle_geometry()
	_init_wheel_states()
	_connect_signals()
	_create_audio_players()
	
	# Apply default mass
	mass = _calculate_default_mass()
	
	print("[VehicleController] Initialized successfully")

func _init_physics_settings() -> void:
	if Engine.has_singleton("PhysicsSettings"):
		physics_settings = Engine.get_singleton("PhysicsSettings")
	else:
		var ps = preload("res://scripts/core/PhysicsSettings.gd")
		physics_settings = Resource.new()
		physics_settings.name = "PhysicsSettings"

func _init_vehicle_geometry() -> void:
	# Calculate wheel positions based on vehicle dimensions
	var half_track = width / 2.0
	var front_offset = length * 0.45
	var rear_offset = length * 0.55
	
	_wheel_positions = [
		Vector3(-half_track, -ride_height, front_offset),    # Front Left
		Vector3(half_track, -ride_height, front_offset),     # Front Right
		Vector3(-half_track, -ride_height, -rear_offset),    # Rear Left
		Vector3(half_track, -ride_height, -rear_offset)      # Rear Right
	]
	
	# Initialize suspension rest lengths
	for pos in _wheel_positions:
		_suspension_rest_lengths.append(spring_rate_front if pos.z > 0 else spring_rate_rear)
	
	# Initialize empty wheel states
	for i in range(4):
		wheel_states.append({
			"index": i,
			"position": Vector3.ZERO,
			"velocity": 0.0,
			"rotation": 0.0,
			"slip_ratio": 0.0,
			"vertical_force": 0.0,
			"lateral_force": 0.0,
			"longitudinal_force": 0.0,
			"suspension_travel": 0.0,
			"camber_angle": 0.0,
			"brake_force": 0.0,
			"drive_force": 0.0,
			"traction_control": true
		})
		
		wheel_velocity.append(0.0)
		current_wheel_angles.append(0.0)

func _connect_signals() -> void:
	if game_manager != null:
		game_manager.race_started.connect(_on_race_started)
		game_manager.game_state_changed.connect(_on_game_state_changed)

func _create_audio_players() -> void:
	_audio_context = AudioStreamPlayer.new()
	_audio_context.name = "VehicleAudio"
	_add_child(_audio_context)

# ============================================================================
# MAIN PHYSICS LOOP
# ============================================================================
func _physics_process(delta: float) -> void:
	_last_delta = delta
	
	# Sub-stepping for accurate physics
	_fixed_step = 1.0 / physics_settings.physics_tick_rate
	_substep_accumulator += delta
	
	while _substep_accumulator >= _fixed_step:
		_substep_accumulator -= _fixed_step
		_perform_physics_substep(_fixed_step)
	
	# Update state signals
	_update_state_signals()

func _perform_physics_substep(dt: float) -> void:
	_apply_inputs()
	_calculate_aerodynamics()
	_calculate_suspension_forces()
	_calculate_wheel_forces()
	_calculate_powertrain()
	_calculate_vehicle_dynamics(dt)
	_handle_drift_physics(dt)
	_handle_collision_detection()
	_move_vehicle(dt)

# ============================================================================
# INPUT HANDLING
# ============================================================================
func _apply_inputs() -> void:
	# Read input values (normalized -1.0 to 1.0 for throttle/brake, 0.0 to 1.0 for steering)
	throttle_input = InputManager.get_axis("throttle", 0.0, 1.0)
	brake_input = InputManager.get_axis("brake", 0.0, 1.0)
	steering_input = clamp(InputManager.get_axis("steering_left", -1.0, 1.0), -1.0, 1.0)
	clutch_input = InputManager.get_axis("clutch", 0.0, 1.0)
	handbrake_input = InputManager.get_axis("handbrake", 0.0, 1.0)
	
	# Clamp inputs to valid ranges
	throttle_input = clamp(throttle_input, 0.0, 1.0)
	brake_input = clamp(brake_input, 0.0, 1.0)
	steering_input = clamp(steering_input, -1.0, 1.0)
	clutch_input = clamp(clutch_input, 0.0, 1.0)
	handbrake_input = clamp(handbrake_input, 0.0, 1.0)
	
	# Apply smooth input transitions
	_smooth_input_transition(throttle_input, "throttle")
	_smooth_input_transition(brake_input, "brake")
	_smooth_input_transition(steering_input, "steering")
	
	# Gear shifting logic
	_handle_gear_shifting()

func _smooth_input_transition(value: float, axis: String) -> void:
	# Smooth input ramping for more natural feel
	var transition_speed = 15.0 if axis == "throttle" else 20.0 if axis == "brake" else 30.0
	var smoothing = lerp(current_speed, value, transition_speed * _last_delta)
	
	match axis:
		"throttle":
			throttle_input = smoothing
		"brake":
			brake_input = smoothing
		"steering":
			steering_input = smoothing

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================
func _handle_gear_shifting() -> void:
	# Auto-shift detection
	if transmission_type == "auto":
		_auto_shift_logic()
	else:
		_manual_shift_detection()
	
	# Calculate clutch engagement
	_clutch_slip_ratio = calculate_clutch_slip()

func _auto_shift_logic() -> void:
	# Automatic transmission shift points
	var shift_up_threshold: float = max_rpm * 0.92
	var shift_down_threshold: float = idle_rpm * 1.5
	
	if current_rpm >= shift_up_threshold and current_gear < num_gears:
		_change_gear(current_gear + 1)
	elif current_rpm <= shift_down_threshold and current_gear > 0:
		_change_gear(current_gear - 1)

func _manual_shift_detection() -> void:
	# Manual transmission shift indicators
	var shift_up = Input.is_action_just_pressed("shift_up")
	var shift_down = Input.is_action_just_pressed("shift_down")
	
	if shift_up and current_gear < num_gears:
		_change_gear(current_gear + 1)
	elif shift_down and current_gear > 0:
		_change_gear(max(0, current_gear - 1))

func _change_gear(new_gear: int) -> void:
	if new_gear == current_gear:
		return
	
	var old_gear = current_gear
	current_gear = new_gear
	
	gear_changed.emit(old_gear, new_gear)
	audio_manager.play_sound("gear_shift")
	
	# Simulate clutch disengagement during shift
	if transmission_type == "manual":
		var shift_duration: float = 0.15
		await get_tree().create_timer(shift_duration).timeout
		clutch_input = 1.0

func calculate_clutch_slip() -> float:
	# Calculate clutch slip ratio between engine and wheels
	if clutch_input >= clutch_release_threshold:
		return 0.0
	
	var wheel_speed_rad = current_speed / tire_radius
	var engine_speed_rad = current_rpm * TAU / 60.0
	
	if wheel_speed_rad == 0:
		return 1.0
	
	return abs(engine_speed_rad - wheel_speed_rad) / max(engine_speed_rad, wheel_speed_rad)

# ============================================================================
# AERODYNAMICS CALCULATION
# ============================================================================
func _calculate_aerodynamics() -> void:
	# Calculate air density at altitude (standard atmosphere model)
	var air_density: float = 1.225  # Sea level kg/m³
	
	# Get vehicle speed magnitude
	var velocity_magnitude = velocity.length()
	
	# Drag force calculation: F_drag = 0.5 * rho * v^2 * Cd * A
	var drag_force: float = 0.5 * air_density * pow(velocity_magnitude, 2) * drag_coefficient * frontal_area
	
	# Downforce/lift calculation
	var aero_force: float = 0.5 * air_density * pow(velocity_magnitude, 2) * downforce_coefficient * frontal_area
	
	# Apply aerodynamic forces to vehicle
	var aero_direction = -velocity.normalized() if velocity_magnitude > 0.1 else Vector3.ZERO
	
	var aero_force_vector = aero_direction * aero_force
	aero_force_vector.y += aero_force  # Downforce adds to vertical load
	
	# Apply to center of mass offset
	position.y += aero_force_vector.y * 0.001
	
	return aero_force

# ============================================================================
# SUSPENSION FORCE CALCULATION
# ============================================================================
func _calculate_suspension_forces() -> void:
	# Determine ground contact for each wheel
	_ground_contact_count = 0
	
	for i in range(4):
		var wheel_pos = _get_world_position(_wheel_positions[i])
		var ray_result = _raycast_to_ground(wheel_pos)
		
		if ray_result.collided:
			var suspension_compression = _calculate_suspension_compression(i, ray_result.position)
			
			# Spring force: F_spring = k * x
			var spring_force = spring_rate_front if i < 2 else spring_rate_rear
			spring_force *= suspension_compression
			
			# Damper force: F_damper = c * v
			var damper_force = damper_rate_front if i < 2 else damper_rate_rear
			damper_force *= suspension_compression / _last_delta if _last_delta > 0 else 0
			
			# Total vertical force
			var total_force = spring_force + damper_force
			
			# Distribute weight based on static distribution
			var weight_distribution = 0.52 if i < 2 else 0.48
			var normal_force = mass * physics_settings.gravity * weight_distribution
			
			# Update wheel state
			wheel_states[i].vertical_force = normal_force - total_force
			wheel_states[i].suspension_travel = suspension_compression
			
			if suspension_compression > 0:
				_ground_contact_count += 1
		else:
			wheel_states[i].vertical_force = 0.0
			wheel_states[i].suspension_travel = front_suspension_travel if i < 2 else rear_suspension_travel

func _calculate_suspension_compression(wheel_index: int, ground_position: Vector3) -> float:
	var wheel_local_pos = _wheel_positions[wheel_index]
	var expected_wheel_height = wheel_local_pos.y + ride_height
	var actual_height = ground_position.y - position.y
	
	return max(0.0, expected_wheel_height - actual_height)

func _raycast_to_ground(start_pos: Vector3) -> RayCast3DResult:
	var ray_cast = RayCast3D.new()
	ray_cast.target_position = Vector3.DOWN * 2.0
	ray_cast.position = start_pos
	ray_cast.collision_mask = 1  # Ground layer
	
	add_child(ray_cast)
	ray_cast.force_raycast_update()
	
	var result = RayCast3DResult()
	result.collided = ray_cast.is_colliding()
	result.position = ray_cast.get_collision_point() if result.collided else start_pos
	
	remove_child(ray_cast)
	queue_free(ray_cast)
	
	return result

# ============================================================================
# WHEEL FORCE CALCULATION
# ============================================================================
func _calculate_wheel_forces() -> void:
	for i in range(4):
		var wheel = wheel_states[i]
		
		# Calculate longitudinal slip
		var drive_force = _calculate_drive_force(i)
		var brake_force = _calculate_brake_force(i)
		
		wheel.drive_force = drive_force
		wheel.brake_force = brake_force
		
		# Calculate lateral slip (for steering)
		var lateral_force = _calculate_lateral_force(i)
		wheel.lateral_force = lateral_force
		
		# Apply traction control if enabled
		if wheel.traction_control:
			wheel.longitudinal_force = _apply_traction_control(i, drive_force - brake_force)
		else:
			wheel.longitudinal_force = drive_force - brake_force
		
		# Calculate total wheel force vector
		var total_force = wheel.longitudinal_force + wheel.lateral_force
		wheel.velocity = total_force / (tire_friction_peak * wheel.vertical_force + 0.01)

func _calculate_drive_force(wheel_index: int) -> float:
	var wheel_is_driven = (transmission_type == "awd") or \
		((wheel_index % 2 == 0) and transmission_type == "rwd") or \
		((wheel_index % 2 != 0) and transmission_type == "fwd")
	
	if not wheel_is_driven:
		return 0.0
	
	# Calculate engine torque based on RPM
	var engine_torque = _get_engine_torque(current_rpm)
	var torque_multiplier = throttle_input * (1.0 - clutch_slip_ratio)
	
	# Apply gear ratio and final drive
	var gear_ratio = gear_ratios[current_gear] if current_gear > 0 else reverse_ratio
	var total_ratio = gear_ratio * final_drive_ratio
	
	var wheel_torque = engine_torque * torque_multiplier * total_ratio * drivetrain_efficiency
	
	# Convert to force at tire contact patch
	var wheel_radius = tire_radius
	var drive_force = wheel_torque / wheel_radius
	
	# Limit by available traction
	var max_traction = wheel.vertical_force * tire_friction_peak
	drive_force = min(drive_force, max_traction)
	
	return drive_force

func _calculate_brake_force(wheel_index: int) -> float:
	var brake_pressure = brake_input * brake_master_cylinder_ratio
	
	# Apply brake bias for front/rear wheels
	var bias = brake_bias_front if wheel_index < 2 else brake_bias_rear
	
	# Calculate maximum brake force
	var max_brake_force = wheel.vertical_force * brake_pad_friction * bias
	
	# Apply brake pressure
	var applied_brake = brake_pressure * max_brake_force
	
	# ABS modulation if enabled
	if abs_enabled and wheel.slip_ratio > abs_threshold:
		applied_brake *= (1.0 - (wheel.slip_ratio - abs_threshold) * 0.5)
	
	return applied_brake

func _calculate_lateral_force(wheel_index: int) -> float:
	var steer_angle = steering_input * tire_max_steering_angle
	var camber_angle = _calculate_camber(wheel_index)
	
	# Simplified lateral force model
	var slip_angle = atan2(velocity.x, velocity.z) - steer_angle
	var lateral_stiffness = tire_stiffness * wheel.vertical_force
	
	var lateral_force = slip_angle * lateral_stiffness
	
	# Limit by friction circle
	var max_lateral_force = wheel.vertical_force * tire_friction_slide
	lateral_force = sign(lateral_force) * min(abs(lateral_force), max_lateral_force)
	
	return lateral_force

func _apply_traction_control(wheel_index: int, drive_force: float) -> float:
	var slip_ratio = _calculate_wheel_slip(wheel_index)
	
	# Reduce drive force if slip exceeds threshold
	if slip_ratio > 0.2:
		return drive_force * (1.0 - (slip_ratio - 0.2) * 0.5)
	
	return drive_force

func _calculate_wheel_slip(wheel_index: int) -> float:
	var wheel_linear_velocity = wheel_velocity[wheel_index]
	var vehicle_speed = current_speed
	
	if vehicle_speed == 0:
		return 0.0
	
	return abs(wheel_linear_velocity - vehicle_speed) / max(vehicle_speed, 1.0)

# ============================================================================
# POWERTRAIN CALCULATION
# ============================================================================
func _calculate_powertrain() -> void:
	# Calculate engine torque curve
	var engine_torque = _get_engine_torque(current_rpm)
	
	# Apply throttle response
	var effective_torque = engine_torque * throttle_input * (1.0 - friction_losses)
	
	# Calculate transmission output
	var gear_ratio = gear_ratios[current_gear] if current_gear > 0 else reverse_ratio
	var total_ratio = gear_ratio * final_drive_ratio
	
	var wheel_torque = effective_torque * total_ratio * drivetrain_efficiency
	
	# Calculate wheel acceleration
	var wheel_radius = tire_radius
	var wheel_acceleration = wheel_torque / (mass * wheel_radius)
	
	# Update RPM based on wheel speed
	var wheel_speed_rad = current_speed / wheel_radius
	var engine_speed_rad = wheel_speed_rad * total_ratio
	current_rpm = engine_speed_rad * 60.0 / TAU
	
	# Engine braking when not throttling
	if throttle_input < 0.1 and current_gear > 0:
		var engine_braking = _calculate_engine_braking()
		wheel_acceleration -= engine_braking
	
	# Apply to velocity
	velocity.x += wheel_acceleration * _last_delta * cos(_vehicle_rotation)
	velocity.z += wheel_acceleration * _last_delta * sin(_vehicle_rotation)

func _get_engine_torque(rpm: float) -> float:
	# Interpolate torque from torque curve points
	if rpm <= 0:
		return 0.0
	
	var torque = 0.0
	for i in range(torque_curve_points.size() - 1):
		var p1 = torque_curve_points[i]
		var p2 = torque_curve_points[i + 1]
		
		if p1.x <= rpm <= p2.x:
			var t = (rpm - p1.x) / (p2.x - p1.x)
			torque = p1.y * (1.0 - t) + p2.y * t
			break
	
	return torque * peak_torque

func _calculate_engine_braking() -> float:
	# Calculate deceleration from engine compression resistance
	var braking_torque = peak_torque * 0.15 * (1.0 - current_rpm / max_rpm)
	var gear_ratio = gear_ratios[current_gear] if current_gear > 0 else reverse_ratio
	var total_ratio = gear_ratio * final_drive_ratio
	
	var braking_force = braking_torque * total_ratio / tire_radius / mass
	return braking_force

# ============================================================================
# VEHICLE DYNAMICS
# ============================================================================
func _calculate_vehicle_dynamics(dt: float) -> void:
	# Apply gravity
	velocity.y -= physics_settings.gravity * dt
	
	# Apply aerodynamic drag
	var drag_deceleration = _calculate_aerodynamics() / mass
	velocity.x -= drag_deceleration * dt
	velocity.z -= drag_deceleration * dt
	
	# Apply centrifugal forces in turns
	var turn_rate = abs(velocity.x * velocity.z)
	if turn_rate > 0:
		var centrifugal_force = turn_rate / (tire_radius * 10.0)
		velocity.y -= centrifugal_force * dt * 0.5
	
	# Update rotation based on steering
	var steer_effectiveness = steering_input * 2.5
	var angular_velocity = steer_effectiveness * current_speed / 10.0
	
	_vehicle_rotation += angular_velocity * dt
	transform.basis = Basis.from_euler(Vector3(0, _vehicle_rotation, 0))

# ============================================================================
# DRIFT PHYSICS
# ============================================================================
func _handle_drift_physics(dt: float) -> void:
	# Detect drift conditions
	var lateral_slip = abs(velocity.x) / max(abs(velocity.z), 1.0)
	
	if lateral_slip > 0.5 and abs(steering_input) > 0.3:
		if not drift_state:
		漂移_state = true
			drift_angle = abs(velocity.x) / current_speed * 180.0 / TAU
			drift_started.emit(drift_angle)
	
	elif drift_state:
		# Recover from drift
		var recovery_rate = _drift_recovery_rate * dt
		drift_angle = max(0.0, drift_angle - recovery_rate)
		
		if drift_angle < 5.0:
			drift_state = false
			drift_ended.emit()

# ============================================================================
# COLLISION DETECTION
# ============================================================================
func _handle_collision_detection() -> void:
	# Check for collisions with objects
	var colliders = _get_overlapping_bodies()
	
	for collider in colliders:
		var collision_data = {
			"object": collider,
			"time": Time.get_ticks_msec(),
			"relative_velocity": velocity.distance_to(collider.linear_velocity)
		}
		
		_collision_buffer.append(collision_data)
		
		# Emit collision signal
		if collision_data.relative_velocity > 5.0:
			collision_impact.emit(velocity * collision_data.relative_velocity, position)
			audio_manager.play_sound("collision")

func _on_collision() -> void:
	# Handle collision response
	if _collision_buffer.size() > 0:
		var latest_collision = _collision_buffer.pop_back()
		var impact_time = Time.get_ticks_msec() - latest_collision.time
		
		if impact_time < 100:  # Within same frame
			# Apply bounce/damage
			velocity *= 0.7  # Energy loss
			audio_manager.play_sound("damage")

# ============================================================================
# VEHICLE MOVEMENT
# ============================================================================
func _move_vehicle(dt: float) -> void:
	# Update position
	move_and_slide()
	
	# Sync visual rotation
	rotate_y(velocity.x * dt * 0.1)
	
	# Store last collision time
	last_collision_time = Time.get_ticks_msec()

# ============================================================================
# UTILITY METHODS
# ============================================================================
func _get_world_position(local_pos: Vector3) -> Vector3:
	return transform.xform(local_pos)

func _calculate_camber(wheel_index: int) -> float:
	var camber_base = -1.5 * TAU / 360.0  # Negative camber for grip
	var camber_dynamic = camber_gain_front if wheel_index < 2 else camber_gain_rear
	var body_roll = velocity.x * 0.01
	
	return camber_base + camber_dynamic * body_roll

func _update_state_signals() -> void:
	# Update current speed
	var speed_kmh = current_speed * 3.6
	speed_changed.emit(speed_kmh)
	rpm_changed.emit(current_rpm)

func _on_race_started(race_data: Dictionary) -> void:
	# Reset vehicle state for race start
	current_speed = 0.0
	current_rpm = idle_rpm
	current_gear = 0
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	drift_state = false
	_airborne = false

func _on_game_state_changed(new_state: GameState) -> void:
	match new_state:
		GameState.RACE_ACTIVE:
			_process_mode = ProcessModeEnum.ALWAYS
		GameState.RACE_PAUSED:
			_process_mode = ProcessModeEnum.WHEN_PAUSED
		GameState.MAIN_MENU:
			_process_mode = ProcessModeEnum.WHEN_PAUSED

# ============================================================================
# PROPERTY SETTERS
# ============================================================================
func _set_mass(value: float) -> void:
	mass = value
	# Recalculate inertia based on mass
	inertia = Vector3(mass * 0.5, mass * 0.5, mass * 0.5)

func _set_gravity(value: float) -> void:
	physics_settings.gravity = value

func _set_physics_tick_rate(value: int) -> void:
	physics_settings.physics_tick_rate = value
	_fixed_step = 1.0 / value

func _set_max_substeps(value: int) -> void:
	physics_settings.max_substeps = value

func _set_time_scale(value: float) -> void:
	physics_settings.time_scale = value

func _set_default_vehicle_mass(value: float) -> void:
	physics_settings.default_vehicle_mass = value

func _set_default_wheel_tire_pressure(value: float) -> void:
	physics_settings.default_wheel_tire_pressure = value

func _set_default_brake_force(value: float) -> void:
	physics_settings.default_brake_force = value

func _set_default_spring_rate(value: float) -> void:
	physics_settings.default_spring_rate = value

func _set_default_damper_rate(value: float) -> void:
	physics_settings.default_damper_rate = value

func _set_default_friction_coefficient(value: float) -> void:
	physics_settings.default_friction_coefficient = value

func _set_default_aero_coefficient(value: float) -> void:
	physics_settings.default_aero_coefficient = value

func _set_default_weight_distribution(value: float) -> void:
	physics_settings.default_weight_distribution = value

func _set_default_center_of_gravity(value: Vector3) -> void:
	physics_settings.default_center_of_gravity = value

func _set_default_suspension_travel(value: float) -> void:
	physics_settings.default_suspension_travel = value

func _set_default_wheelbase(value: float) -> void:
	physics_settings.default_wheelbase = value

func _set_default_track_width(value: float) -> void:
	physics_settings.default_track_width = value

func _set_default_roll_center_height(value: float) -> void:
	physics_settings.default_roll_center_height = value

func _set_default_pitch_damping(value: float) -> void:
	physics_settings.default_pitch_damping = value

func _set_default_yaw_damping(value: float) -> void:
	physics_settings.default_yaw_damping = value

func _calculate_default_mass() -> float:
	return physics_settings.default_vehicle_mass if physics_settings else 1500.0

# ============================================================================
# PUBLIC API
# ============================================================================
func reset_vehicle() -> void:
	current_speed = 0.0
	current_rpm = idle_rpm
	current_gear = 0
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	drift_state = false
	velocity = Vector3.ZERO
	transform.origin = Vector3.ZERO

func set_target_gear(gear: int) -> void:
	target_gear = clamp(gear, 0, num_gears)

func get_current_speed_kmh() -> float:
	return current_speed * 3.6

func get_current_rpm() -> float:
	return current_rpm

func get_current_gear() -> int:
	return current_gear

func get_drift_angle() -> float:
	return drift_angle

func get_wheel_slip(index: int) -> float:
	if index >= 0 and index < wheel_states.size():
		return wheel_states[index].slip_ratio
	return 0.0

func is_drifting() -> bool:
	return drift_state

func is_airborne() -> bool:
	return _ground_contact_count == 0

func apply_force(force: Vector3, point: Vector3) -> void:
	apply_central_impulse(force)

func apply_torque(torque: float, axis: Vector3) -> void:
	angular_velocity += torque * axis

func enable_abs(enabled: bool) -> void:
	abs_enabled = enabled

func set_brake_bias(front: float) -> void:
	brake_bias_front = clamp(front, 0.0, 1.0)
	brake_bias_rear = 1.0 - front

func set_suspension_stiffness(front: float, rear: float) -> void:
	spring_rate_front = front
	spring_rate_rear = rear

func set_suspension_damping(front: float, rear: float) -> void:
	damper_rate_front = front
	damper_rate_rear = rear

func get_vehicle_health() -> float:
	# Simple health calculation based on accumulated damage
	var damage_score = 0.0
	for collision in _collision_buffer:
		damage_score += collision.relative_velocity * 0.1
	
	return max(0.0, 100.0 - damage_score)

func get_fuel_level() -> float:
	# Placeholder for fuel system integration
	return 100.0

func refuel(amount: float) -> void:
	# Placeholder for fuel refill
	pass

func update_lap_time(time: float) -> void:
	# Placeholder for lap timing
	pass

func get_performance_metrics() -> Dictionary:
	return {
		"speed_kmh": current_speed * 3.6,
		"rpm": current_rpm,
		"gear": current_gear,
		"drift_angle": drift_angle,
		"wheel_slips": [w.slip_ratio for w in wheel_states],
		"fuel_level": get_fuel_level(),
		"health": get_vehicle_health()
	}

</File>