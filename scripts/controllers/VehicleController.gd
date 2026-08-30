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

@export_group("Steering & Handling")
@export var steering_sensitivity: float = 1.0: set = _set_steering_sensitivity
@export var steering_return_speed: float = 5.0: set = _set_steering_return_speed
@export var understeer_coefficient: float = 0.3: set = _set_understeer_coefficient
@export var oversteer_coefficient: float = 0.2: set = _set_oversteer_coefficient

@export_group("Suspension & Chassis")
@export var suspension_stiffness: float = 50000.0: set = _set_suspension_stiffness
@export var suspension_damping: float = 3000.0: set = _set_suspension_damping
@export var center_of_mass_offset: Vector3 = Vector3(0.0, -0.3, 0.2): set = _set_center_of_mass_offset
@export var track_width: float = 1.6: set = _set_track_width
@export var wheelbase: float = 2.7: set = _set_wheelbase

@export_group("Advanced Dynamics")
@export var aerodynamic_drag_coeffent: float = 0.32: set = _set_aero_drag_coefficient
@export var frontal_area: float = 2.2: set = _set_frontal_area
@export var downforce_coefficient: float = 0.5: set = _set_downforce_coefficient
@export var friction_coefficient: float = 0.9: set = _set_friction_coefficient
@export var roll_bar_effectiveness: float = 0.15: set = _set_roll_bar_effectiveness

# ============================================================================
# PRIVATE STATE VARIABLES
# ============================================================================
var _current_speed: float = 0.0              # Current speed km/h
var _current_rpm: float = 0.0                # Current engine RPM
var _current_gear: int = 0                   # Current gear (positive = forward, -1 = reverse, 0 = neutral)
var _target_gear: int = 0                    # Target gear for automatic shifting
var _is_engine_running: bool = false         # Engine running state
var _is_handbrake_active: bool = false       # Handbrake state
var _is_traction_control_on: bool = true     # Traction control enabled
var _current_steering_angle: float = 0.0     # Current steering angle in degrees
var _target_steering_angle: float = 0.0      # Target steering angle
var _slip_ratio: float = 0.0                 # Wheel slip ratio
var _drift_angle: float = 0.0                # Current drift angle
var _is_drifting: bool = false               # Drift state flag
var _last_velocity: Vector3 = Vector3.ZERO   # Previous frame velocity
var _wheel_forces: Array[Vector3] = []       # Force vectors for each wheel
var _suspension_compression: Array[float] = [] # Suspension compression per wheel

# ============================================================================
# WHEEL POSITIONS (Local to vehicle)
# ============================================================================
var _front_left_wheel: Vector3 = Vector3(-track_width / 2, -0.3, wheelbase / 2)
var _front_right_wheel: Vector3 = Vector3(track_width / 2, -0.3, wheelbase / 2)
var _rear_left_wheel: Vector3 = Vector3(-track_width / 2, -0.3, -wheelbase / 2)
var _rear_right_wheel: Vector3 = Vector3(track_width / 2, -0.3, -wheelbase / 2)

# ============================================================================
# INPUT HANDLING
# ============================================================================
var _input_throttle: float = 0.0             # 0.0 to 1.0 throttle input
var _input_brake: float = 0.0                # 0.0 to 1.0 brake input
var _input_steering: float = 0.0             # -1.0 to 1.0 steering input
var _input_clutch: float = 1.0               # 0.0 to 1.0 clutch input
var _input_shift_up: bool = false            # Shift up request
var _input_shift_down: bool = false          # Shift down request
var _input_handbrake: bool = false           # Handbrake toggle

# ============================================================================
# PHYSICS SUB-STATES
# ============================================================================
var _physics_time_accumulator: float = 0.0
var _physics_substeps: int = 4               # Physics substeps for stability
var _delta_per_substep: float = 0.0

# ============================================================================
# AUDIO REFERENCES
# ============================================================================
var _engine_audio_node: AudioStreamPlayer3D = null
var _tire_audio_node: AudioStreamPlayer3D = null
var _gear_audio_node: AudioStreamPlayer3D = null

# ============================================================================
# PROPERTY SETTERS WITH VALIDATION
# ============================================================================
func _set_vehicle_mass(value: float) -> void:
	vehicle_mass = max(value, 500.0)
	body.mass = vehicle_mass

func _set_max_speed_kmh(value: float) -> void:
	max_speed_kmh = max(value, 10.0)

func _set_acceleration_force(value: float) -> void:
	acceleration_force = max(value, 1000.0)

func _set_braking_force(value: float) -> void:
	braking_force = max(value, 5000.0)

func _set_steering_angle_max(value: float) -> void:
	steering_angle_max = clamp(value, 30.0, 60.0)

func _set_final_drive_ratio(value: float) -> void:
	final_drive_ratio = max(value, 1.5)

func _set_tire_radius(value: float) -> void:
	tire_radius = max(value, 0.25)

func _set_torque_curve(value: Array[Vector2f]) -> void:
	if value.size() >= 2:
		torque_curve = value
	else:
		printerr("Torque curve must have at least 2 points")

func _set_num_gears(value: int) -> void:
	num_gears = clamp(value, 4, 10)

func _set_gear_ratios(value: Array[float]) -> void:
	if value.size() == num_gears:
		gear_ratios = value
	else:
		printerr("Gear ratios array size mismatch")

func _set_idle_rpm(value: float) -> void:
	idle_rpm = clamp(value, 500.0, 1500.0)

func _set_redline_rpm(value: float) -> void:
	redline_rpm = max(value, 6000.0)

func _set_steering_sensitivity(value: float) -> void:
	steering_sensitivity = clamp(value, 0.5, 2.0)

func _set_steering_return_speed(value: float) -> void:
	steering_return_speed = max(value, 1.0)

func _set_understeer_coefficient(value: float) -> void:
	understeer_coefficient = clamp(value, 0.0, 0.5)

func _set_oversteer_coefficient(value: float) -> void:
	oversteer_coefficient = clamp(value, 0.0, 0.5)

func _set_suspension_stiffness(value: float) -> void:
	suspension_stiffness = max(value, 10000.0)

func _set_suspension_damping(value: float) -> void:
	suspension_damping = max(value, 1000.0)

func _set_center_of_mass_offset(value: Vector3) -> void:
	center_of_mass_offset = value

func _set_track_width(value: float) -> void:
	track_width = max(value, 1.0)

func _set_wheelbase(value: float) -> void:
	wheelbase = max(value, 2.0)

func _set_aero_drag_coefficient(value: float) -> void:
	aerodynamic_drag_coefficient = clamp(value, 0.1, 0.6)

func _set_frontal_area(value: float) -> void:
	frontal_area = max(value, 1.0)

func _set_downforce_coefficient(value: float) -> void:
	downforce_coefficient = clamp(value, 0.0, 2.0)

func _set_friction_coefficient(value: float) -> void:
	friction_coefficient = clamp(value, 0.3, 1.2)

func _set_roll_bar_effectiveness(value: float) -> void:
	roll_bar_effectiveness = clamp(value, 0.0, 0.3)

# ============================================================================
# INITIALIZATION
# ============================================================================
func _ready() -> void:
	_init_physics_properties()
	_init_wheels()
	_init_audio_references()
	_reset_state()
	set_process(true)
	set_physics_process(true)
	print("VehicleController initialized successfully")

func _init_physics_properties() -> void:
	"""Initialize physics-based vehicle properties from settings"""
	body.mass = vehicle_mass
	body.collision_layer = 1 << 1  # Vehicle layer
	body.collision_mask = 1 << 2 | 1 << 3  # Collide with ground and other vehicles
	
	# Initialize suspension compression array
	for i in range(4):
		_suspension_compression.append(0.0)

func _init_wheels() -> void:
	"""Initialize wheel force arrays"""
	_wheel_forces.resize(4)
	for i in range(4):
		_wheel_forces[i] = Vector3.ZERO

func _init_audio_references() -> void:
	"""Find audio stream players attached to this node"""
	_engine_audio_node = find_child("EngineAudio", false) as AudioStreamPlayer3D
	_tire_audio_node = find_child("TireAudio", false) as AudioStreamPlayer3D
	_gear_audio_node = find_child("GearAudio", false) as AudioStreamPlayer3D

func _reset_state() -> void:
	"""Reset all vehicle state variables to defaults"""
	_current_speed = 0.0
	_current_rpm = idle_rpm
	_current_gear = 0
	_target_gear = 0
	_is_engine_running = false
	_is_handbrake_active = false
	_is_traction_control_on = true
	_current_steering_angle = 0.0
	_target_steering_angle = 0.0
	_slip_ratio = 0.0
	_drift_angle = 0.0
	_is_drifting = false
	_last_velocity = Vector3.ZERO
	_input_throttle = 0.0
	_input_brake = 0.0
	_input_steering = 0.0
	_input_clutch = 1.0
	_input_shift_up = false
	_input_shift_down = false
	_input_handbrake = false

# ============================================================================
# MAIN PHYSICS LOOP
# ============================================================================
func _physics_process(delta: float) -> void:
	"""Fixed timestep physics update with sub-stepping for stability"""
	_physics_time_accumulator += delta
	_delta_per_substep = delta / _physics_substeps
	
	while _physics_time_accumulator >= _delta_per_substep:
		_step_physics(_delta_per_substep)
		_physics_time_accumulator -= _delta_per_substep
	
	_handle_inputs(delta)
	_update_steering(delta)
	_update_audio()

func _step_physics(dt: float) -> void:
	"""Execute one physics sub-step"""
	_calculate_wheel_forces(dt)
	_apply_forces_to_body()
	_update_vehicle_state(dt)
	_check_collisions()
	_update_suspension()

# ============================================================================
# INPUT HANDLING
# ============================================================================
func _handle_inputs(delta: float) -> void:
	"""Process player input and update vehicle controls"""
	# Read input values from InputManager
	_input_throttle = GameManager.input.get_axis_strength("throttle")
	_input_brake = GameManager.input.get_axis_strength("brake")
	_input_steering = GameManager.input.get_axis_strength("steering") * steering_sensitivity
	_input_handbrake = GameManager.input.is_action_pressed("handbrake")
	
	# Handle gear shifting inputs
	if GameManager.input.is_action_just_pressed("shift_up"):
		_request_shift_up()
	elif GameManager.input.is_action_just_pressed("shift_down"):
		_request_shift_down()
	
	# Update handbrake state
	if _input_handbrake != _is_handbrake_active:
		_is_handbrake_active = _input_handbrake
		handbrake_toggled.emit(_is_handbrake_active)
	
	# Update clutch based on gear changes
	if _current_gear != 0:
		_input_clutch = 1.0
	else:
		_input_clutch = 0.0

func _update_steering(delta: float) -> void:
	"""Smoothly interpolate steering angle towards target"""
	var target_angle = _input_steering * steering_angle_max
	
	# Apply steering return to center when no input
	if abs(_input_steering) < 0.05:
		target_angle = 0.0
	
	# Smooth steering transition
	var steering_diff = target_angle - _current_steering_angle
	var max_turn = MAX_STEERING_RATE * steering_return_speed * delta
	
	if abs(steering_diff) > max_turn:
		steering_diff = sign(steering_diff) * max_turn
	
	_current_steering_angle += steering_diff
	_target_steering_angle = target_angle

# ============================================================================
# ENGINE & GEAR LOGIC
# ============================================================================
func calculate_engine_rpm(speed_kmh: float, gear: int) -> float:
	"""Calculate engine RPM based on vehicle speed and gear"""
	if gear == 0:  # Neutral
		return idle_rpm
	
	var gear_ratio = gear_ratios[gear - 1] if gear > 0 else reverse_ratio
	var wheel_rotational_speed = (speed_kmh * 1000.0 / 3600.0) / (2.0 * PI * tire_radius)
	var engine_rpm = wheel_rotational_speed * gear_ratio * final_drive_ratio * 60.0
	
	return engine_rpm

func get_torque_at_rpm(rpm: float) -> float:
	"""Get torque multiplier based on current RPM using torque curve"""
	if rpm <= 0.0:
		return 0.0
	
	var rpm_fraction = (rpm - idle_rpm) / (redline_rpm - idle_rpm)
	rpm_fraction = clamp(rpm_fraction, 0.0, 1.0)
	
	# Linear interpolation through torque curve points
	for i in range(torque_curve.size() - 1):
		if rpm_fraction >= torque_curve[i].x and rpm_fraction <= torque_curve[i + 1].x:
			var t = (rpm_fraction - torque_curve[i].x) / (torque_curve[i + 1].x - torque_curve[i].x)
			return torque_curve[i].y + t * (torque_curve[i + 1].y - torque_curve[i].y)
	
	return torque_curve.back().y

func shift_gear(target_gear: int) -> void:
	"""Change to specified gear"""
	if target_gear < -1 or target_gear > num_gears:
		return
	
	if _current_gear != target_gear:
		var old_gear = _current_gear
		_current_gear = target_gear
		
		# Play gear change sound
		if _gear_audio_node:
			_gear_audio_node.play()
		
		gear_changed.emit(old_gear, _current_gear)
		
		# Rev matching for downshifts
		if _current_gear < old_gear and old_gear > 0:
			_current_rpm = lerp(_current_rpm, calculate_engine_rpm(_current_speed, _current_gear) * 1.2, 0.1)

func _request_shift_up() -> void:
	"""Request shift to next higher gear"""
	if _current_gear < num_gears and _current_gear >= 0:
		shift_gear(_current_gear + 1)

func _request_shift_down() -> void:
	"""Request shift to next lower gear"""
	if _current_gear > 0:
		shift_gear(_current_gear - 1)
	elif _current_gear == 0:
		shift_gear(-1)  # Reverse

func auto_shift_logic() -> void:
	"""Automatic transmission logic based on RPM and speed"""
	if not _is_engine_running:
		return
	
	var target_rpm = calculate_engine_rpm(_current_speed, _current_gear)
	
	# Upshift if above 85% of redline
	if _current_gear < num_gears and _current_rpm > redline_rpm * 0.85:
		shift_gear(_current_gear + 1)
		return
	
	# Downshift if below 15% of redline and moving
	if _current_gear > 1 and _current_rpm < redline_rpm * 0.15 and _current_speed > 5.0:
		shift_gear(_current_gear - 1)

# ============================================================================
# WHEEL FORCE CALCULATION
# ============================================================================
func _calculate_wheel_forces(dt: float) -> void:
	"""Calculate individual wheel forces based on physics model"""
	var total_torque = _calculate_engine_torque()
	var drive_torque = _apply_gearing(total_torque)
	
	# Distribute torque based on drivetrain type
	var wheel_torques = _distribute_torque(drive_torque)
	
	# Calculate longitudinal forces
	for i in range(4):
		var wheel_torque = wheel_torques[i]
		var wheel_force = wheel_torque / tire_radius
		
		# Apply slip calculation
		_slip_ratio = _calculate_wheel_slip(i, wheel_force)
		
		# Apply friction limit
		var max_force = friction_coefficient * (_suspension_compression[i] * suspension_stiffness)
		wheel_force = min(wheel_force, max_force)
		
		_wheel_forces[i] = Vector3(0.0, 0.0, -wheel_force)

func _calculate_engine_torque() -> float:
	"""Calculate total engine torque output"""
	if not _is_engine_running:
		return 0.0
	
	var base_torque = 400.0  # Nm at idle
	var torque_multiplier = get_torque_at_rpm(_current_rpm)
	var actual_torque = base_torque * torque_multiplier
	
	# Apply clutch effect
	actual_torque *= _input_clutch
	
	return actual_torque

func _apply_gearing(torque: float) -> float:
	"""Apply gear reduction to get wheel torque"""
	if _current_gear == 0:
		return 0.0
	
	var gear_ratio = gear_ratios[_current_gear - 1] if _current_gear > 0 else reverse_ratio
	return torque * gear_ratio * final_drive_ratio

func _distribute_torque(wheel_torque: float) -> Array[float]:
	"""Distribute torque across wheels based on drivetrain type"""
	match drivetrain_type:
		DrivetrainType.FWD:
			return [wheel_torque * 0.5, wheel_torque * 0.5, 0.0, 0.0]
		DrivetrainType.RWD:
			return [0.0, 0.0, wheel_torque * 0.5, wheel_torque * 0.5]
		DrivetrainType.AWD:
			return [wheel_torque * 0.3, wheel_torque * 0.3, wheel_torque * 0.2, wheel_torque * 0.2]
		_:
			return [wheel_torque * 0.25, wheel_torque * 0.25, wheel_torque * 0.25, wheel_torque * 0.25]

func _calculate_wheel_slip(wheel_index: int, force: float) -> float:
	"""Calculate wheel slip ratio for traction control"""
	var wheel_velocity = _velocity.length()
	var theoretical_velocity = force * dt / vehicle_mass
	
	if wheel_velocity > 0.1:
		return (theoretical_velocity - wheel_velocity) / wheel_velocity
	return 0.0

# ============================================================================
# FORCE APPLICATION TO BODY
# ============================================================================
func _apply_forces_to_body() -> void:
	"""Apply calculated wheel forces to vehicle body"""
	var total_force = Vector3.ZERO
	var total_torque = Vector3.ZERO
	
	for i in range(4):
		total_force += _wheel_forces[i]
		
		# Calculate torque contribution from each wheel
		var wheel_pos = transform.basis * _get_wheel_local_position(i)
		var force_torque = wheel_pos.cross(_wheel_forces[i])
		total_torque += force_torque
	
	# Apply aerodynamic drag
	var aero_drag = _calculate_aerodynamic_drag()
	total_force += aero_drag
	
	# Apply gravity
	var gravity_force = Vector3(0.0, -vehicle_mass * PhysicsSettings.gravity, 0.0)
	total_force += gravity_force
	
	# Move the body
	move_and_slide(total_force, Vector3.UP)

func _get_wheel_local_position(index: int) -> Vector3:
	"""Get local position of wheel by index"""
	match index:
		0: return _front_left_wheel
		1: return _front_right_wheel
		2: return _rear_left_wheel
		3: return _rear_right_wheel
		_: return Vector3.ZERO

func _calculate_aerodynamic_drag() -> Vector3:
	"""Calculate aerodynamic drag force"""
	var speed_ms = _current_speed * 1000.0 / 3600.0
	var drag_force = 0.5 * aerodynamic_drag_coefficient * frontal_area * pow(speed_ms, 2)
	
	if speed_ms > 0.1:
		var drag_direction = -velocity.normalized()
		return drag_direction * drag_force
	return Vector3.ZERO

# ============================================================================
# VEHICLE STATE UPDATE
# ============================================================================
func _update_vehicle_state(dt: float) -> void:
	"""Update vehicle internal state after physics step"""
	# Calculate current speed
	_current_speed = velocity.length() * 3.6  # Convert m/s to km/h
	
	# Calculate current RPM based on speed and gear
	if _current_gear != 0:
		_current_rpm = calculate_engine_rpm(_current_speed, _current_gear)
	else:
		# Engine decays to idle when in neutral
		_current_rpm = lerp(_current_rpm, idle_rpm, 5.0 * dt)
	
	# Clamp RPM
	_current_rpm = clamp(_current_rpm, idle_rpm, redline_rpm * 1.1)
	
	# Emit signals
	speed_changed.emit(_current_speed)
	rpm_changed.emit(_current_rpm)
	
	# Check for rev limiter
	if _current_rpm >= redline_rpm * 0.98:
		_current_rpm = redline_rpm * 0.98
	
	# Vehicle movement signal
	if _last_velocity != velocity:
		vehicle_moved.emit(global_position, velocity)
		_last_velocity = velocity
	
	# Auto-shift logic
	auto_shift_logic()

# ============================================================================
# SUSPENSION SYSTEM
# ============================================================================
func _update_suspension() -> void:
	"""Update suspension compression based on forces"""
	var wheel_positions = [_front_left_wheel, _front_right_wheel, _rear_left_wheel, _rear_right_wheel]
	
	for i in range(4):
		var wheel_world_pos = global_transform * wheel_positions[i]
		var ray_from = wheel_world_pos + Vector3.UP * 1.0
		var ray_to = wheel_world_pos + Vector3.DOWN * 2.0
		
		var space_state = get_world_3d().direct_space_state
		var query = PhysicsRayQueryParameters3D.create(ray_from, ray_to)
		var result = space_state.intersect_ray(query)
		
		if result:
			var distance = ray_from.distance_to(result.position)
			var compression = max(0.0, 1.0 - distance / 2.0)
			
			# Spring-damper system
			var spring_force = compression * suspension_stiffness
			var damping_force = velocity.y * suspension_damping
			
			_suspension_compression[i] = lerp(_suspension_compression[i], compression, 10.0 * Time.delta)

# ============================================================================
# COLLISION DETECTION
# ============================================================================
func _check_collisions() -> void:
	"""Check and handle collisions with environment"""
	for i in range(get_slide_collision_count()):
		var collision = get_slide_collision(i)
		var collider = collision.get_collider()
		
		# Create collision data dictionary
		var collision_data = {
			"collider": collider.name if collider else "unknown",
			"position": global_position,
			"velocity": velocity,
			"normal": collision.get_normal(),
			"contact_point": collision.get_contact_local_pos()
		}
		
		collision_detected.emit(collision_data)

# ============================================================================
# DRIFT & TRACTION CONTROL
# ============================================================================
func _update_drift_detection() -> void:
	"""Detect and manage drifting state"""
	if _current_speed < 20.0:  # Minimum speed for drift
		_is_drifting = false
		_drift_angle = 0.0
		return
	
	var heading = velocity.normalized()
	var forward = transform.basis.z * -1
	
	var angle_between = heading.angle_to(forward)
	
	if angle_between > DRIFT_THRESHOLD and _input_handbrake:
		if not _is_drifting:
			_is_drifting = true
			_drift_angle = angle_between
			drift_started.emit(_drift_angle)
	else:
		if _is_drifting:
			_is_drifting = false
			drift_ended.emit()

func _update_traction_control() -> void:
	"""Apply traction control if enabled"""
	if not _is_traction_control_on:
		return
	
	# Check slip ratio and reduce torque if too high
	if _slip_ratio > TRACTION_CONTROL_THRESHOLD and _input_throttle > 0.1:
		_input_throttle *= 0.8  # Reduce throttle
		traction_control_active.emit(false)
	else:
		traction_control_active.emit(true)

# ============================================================================
# ENGINE START/STOP
# ============================================================================
func start_engine() -> void:
	"""Start the engine"""
	if _is_engine_running:
		return
	
	_is_engine_running = true
	_current_rpm = idle_rpm
	engine_started.emit()
	
	if _engine_audio_node:
		_engine_audio_node.play()

func stop_engine() -> void:
	"""Stop the engine"""
	if not _is_engine_running:
		return
	
	_is_engine_running = false
	_current_rpm = 0.0
	engine_stopped.emit()
	
	if _engine_audio_node:
		_engine_audio_node.stop()

func toggle_engine() -> void:
	"""Toggle engine state"""
	if _is_engine_running:
		stop_engine()
	else:
		start_engine()

# ============================================================================
# AUDIO UPDATES
# ============================================================================
func _update_audio() -> void:
	"""Update audio parameters based on vehicle state"""
	if _engine_audio_node:
		var pitch_variation = (_current_rpm - idle_rpm) / (redline_rpm - idle_rpm)
		_engine_audio_node.pitch_scale = 0.5 + pitch_variation * 1.5
		_engine_audio_node.volume_db = linear_map(_input_throttle, -80.0, 0.0)

func linear_map(value: float, in_min: float, in_max: float) -> float:
	"""Linear mapping utility function"""
	return (value - in_min) * (in_max - in_min) / (in_max - in_min) + in_min

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================
func reset_vehicle() -> void:
	"""Reset vehicle to initial state"""
	global_position = Vector3(0.0, 0.0, 0.0)
	rotation = Vector3.ZERO
	velocity = Vector3.ZERO
	_reset_state()

func get_vehicle_stats() -> Dictionary:
	"""Get current vehicle statistics"""
	return {
		"speed_kmh": _current_speed,
		"rpm": _current_rpm,
		"gear": _current_gear,
		"is_engine_running": _is_engine_running,
		"is_handbrake_active": _is_handbrake_active,
		"steering_angle": _current_steering_angle,
		"slip_ratio": _slip_ratio,
		"is_drifting": _is_drifting
	}

func set_custom_torque_curve(curve_points: Array[Vector2f]) -> void:
	"""Set custom torque curve programmatically"""
	if curve_points.size() >= 2:
		torque_curve = curve_points

func set_custom_gear_ratios(ratios: Array[float]) -> void:
	"""Set custom gear ratios programmatically"""
	if ratios.size() == num_gears:
		gear_ratios = ratios

# ============================================================================
# DEBUG VISUALIZATION (if debug mode is enabled)
# ============================================================================
func _draw_debug() -> void:
	"""Draw debug visualization of vehicle physics"""
	if not GameManager.debug_mode:
		return
	
	# Draw suspension compression
	for i in range(4):
		var color = Color.GREEN if _suspension_compression[i] < 0.5 else Color.RED
		var wheel_pos = global_transform * _get_wheel_local_position(i)
		draw_line(wheel_pos, wheel_pos + Vector3.UP * _suspension_compression[i] * 2.0, color)

</FILE "scripts/controllers/VehicleController.gd">