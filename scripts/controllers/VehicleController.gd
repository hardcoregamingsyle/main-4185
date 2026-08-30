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
signal lap_completed(lap_time: float)
signal checkpoint_passed(checkpoint_id: int)
signal powertrain_connected(powertrain: Node)
signal suspension_bumped(wheel_index: int, compression: float)

# ============================================================================
# DRIVETRAIN TYPES
# ============================================================================
enum DrivetrainType {
	FWD,  # Front-Wheel Drive
	RWD,  # Rear-Wheel Drive
	AWD   # All-Wheel Drive
}

enum GearState {
	NEUTRAL = 0,
	REVERSE = -1,
	FIRST = 1,
	SECOND = 2,
	THIRD = 3,
	FOURTH = 4,
	FIFTH = 5,
	SIXTH = 6,
	SEVENTH = 7,
	EIGHTH = 8,
	NINTH = 9,
	TENTH = 10
}

# ============================================================================
# CONSTANTS & CONFIGURATION
# ============================================================================
const VEHICLE_BASE_MASS := 1500.0
const MIN_RPM := 800.0
const IDLE_RPM := 1000.0
const MAX_RPM := 8000.0
const REDLINE_RPM := 7500.0
const GEAR_RATIO_FIRST := 3.8
const GEAR_RATIO_SECOND := 2.4
const GEAR_RATIO_THIRD := 1.7
const GEAR_RATIO_FOURTH := 1.3
const GEAR_RATIO_FIFTH := 1.0
const GEAR_RATIO_SIXTH := 0.85
const GEAR_RATIO_REVERSE := 3.5
const FINAL_DRIVE := 3.73
const WHEEL_RADIUS := 0.33

# ============================================================================
# PHYSICS PROPERTIES (exported for inspector)
# ============================================================================
@export_group("Vehicle Properties")
@export var vehicle_mass: float = 1500.0
@export var center_of_mass_offset: Vector3 = Vector3(0, 0.3, 0)
@export var drivetrain_type: DrivetrainType = DrivetrainType.RWD

@export_group("Engine Properties")
@export var engine_max_power: float = 300.0  # horsepower
@export var engine_max_torque: float = 450.0  # Nm
@export var engine_redline: float = REDLINE_RPM

@export_group("Transmission Properties")
@export var transmission_ratio: float = 1.0
@export var final_drive_ratio: float = FINAL_DRIVE
@export var clutch_engagement_threshold: float = 0.1

@export_group("Wheel Properties")
@export var wheel_radius: float = WHEEL_RADIUS
@export var wheel_width: float = 0.25
@export var wheel_friction: float = 1.0

@export_group("Steering Properties")
@export var max_steering_angle: float = 30.0  # degrees
@export var steering_sensitivity: float = 1.0
@export var steering_return_rate: float = 15.0

@export_group("Brake Properties")
@export var brake_force: float = 8000.0  # Newtons per wheel
@export var abs_enabled: bool = true
@export var brake_bias_front: float = 0.6  # 0-1, front brake bias

@export_group("Handbrake Properties")
@export var handbrake_force: float = 5000.0  # Newtons per rear wheel

@export_group("Suspension Properties")
@export var suspension_stiffness: float = 50000.0  # N/m
@export var suspension_damping: float = 5000.0  # Ns/m
@export var suspension_travel: float = 0.15  # meters
@export var suspension_rest_length: float = 0.3

@export_group("Aerodynamics")
@export var drag_coefficient: float = 0.32
@export var frontal_area: float = 2.2  # square meters
@export var downforce_coefficient: float = 0.5

# ============================================================================
# INTERNAL STATE
# ============================================================================
var current_rpm: float = IDLE_RPM
var current_gear: int = GearState.NEUTRAL
var current_speed: float = 0.0  # m/s
var target_steering_angle: float = 0.0
var actual_steering_angle: float = 0.0
var is_engine_running: bool = false
var is_braking: bool = false
var is_handbrake_active: bool = false
var is_drifting: bool = false
var drift_angle: float = 0.0
var drift_factor: float = 0.0

var _powertrain_node: Node = null
var _wheel_nodes: Array[Node] = []
var _suspension_nodes: Array[Node] = []

# Input tracking
var _throttle_input: float = 0.0
var _brake_input: float = 0.0
var _steering_input: float = 0.0
var _clutch_input: float = 0.0
var _gear_input_direction: int = 0  # -1 downshift, +1 upshift

# Previous state for change detection
var _previous_velocity: Vector3 = Vector3.ZERO
var _previous_gear: int = GearState.NEUTRAL
var _last_collision_time: float = 0.0

# Wheel force data
var _wheel_forces: Array[float] = [0.0, 0.0, 0.0, 0.0]  # FL, FR, RL, RR
var _wheel_caster_angles: Array[float] = [0.0, 0.0, 0.0, 0.0]

# Suspension data
var _wheel_positions: Array[Vector3] = [Vector3.ZERO] * 4
var _wheel_contact_normals: Array[Vector3] = [Vector3.UP] * 4
var _wheel_compression: Array[float] = [0.0] * 4

# Timing
var _last_update_time: float = 0.0
var _accumulated_time: float = 0.0

# ============================================================================
# SETUP & INITIALIZATION
# ============================================================================
func _ready() -> void:
	_process_mode = ProcessModeEnum.PHYSICS
	_init_vehicle_setup()
	_connect_to_powertrain()
	
	# Initialize wheel positions relative to vehicle body
	_setup_wheel_positions()
	
	print("VehicleController ready - Mass:", vehicle_mass, "kg")

func _init_vehicle_setup() -> void:
	"""Initialize vehicle setup and apply defaults from PhysicsSettings"""
	# Apply default values from PhysicsSettings if not explicitly set
	if vehicle_mass <= VEHICLE_BASE_MASS:
		vehicle_mass = PhysicsSettings.default_vehicle_mass
	
	# Set mass and gravity scale
	mass = vehicle_mass
	gravity_scale = PhysicsSettings.gravity / 9.81
	
	# Reset all states
	reset_vehicle_state()

func _connect_to_powertrain() -> void:
	"""Connect to powertrain node if it exists as a child"""
	var powertrain_children = get_children()
	for child in powertrain_children:
		if child.has_method("get_powertrain_type"):
			_powertrain_node = child
			powertrain_connected.emit(_powertrain_node)
			break

func _setup_wheel_positions() -> void:
	"""Setup initial wheel positions relative to vehicle center"""
	# Standard wheelbase and track width
	var track_width: float = 1.5
	var wheelbase: float = 2.5
	
	# Calculate positions (FL, FR, RL, RR)
	_wheel_positions[0] = Vector3(-track_width/2, -suspension_rest_length, -wheelbase/2)
	_wheel_positions[1] = Vector3(track_width/2, -suspension_rest_length, -wheelbase/2)
	_wheel_positions[2] = Vector3(-track_width/2, -suspension_rest_length, wheelbase/2)
	_wheel_positions[3] = Vector3(track_width/2, -suspension_rest_length, wheelbase/2)

func reset_vehicle_state() -> void:
	"""Reset vehicle to initial state"""
	current_rpm = IDLE_RPM
	current_gear = GearState.NEUTRAL
	current_speed = 0.0
	target_steering_angle = 0.0
	actual_steering_angle = 0.0
	is_engine_running = false
	is_braking = false
	is_handbrake_active = false
	is_drifting = false
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	clutch_input = 0.0
	
	_wheel_forces.fill(0.0)
	_wheel_caster_angles.fill(0.0)
	_wheel_compression.fill(0.0)
	
	_previous_velocity = Vector3.ZERO
	_previous_gear = GearState.NEUTRAL

# ============================================================================
# INPUT HANDLING
# ============================================================================
func _physics_process(delta: float) -> void:
	_accumulated_time += delta
	
	# Update at fixed timestep for consistent physics
	const FIXED_DELTA: float = 1.0 / PhysicsSettings.physics_tick_rate
	while _accumulated_time >= FIXED_DELTA:
		_update_physics(FIXED_DELTA)
		_accumulated_time -= FIXED_DELTA
	
	# Smoothly interpolate values for rendering
	_interpolate_render_values()
	
	# Emit movement signal
	if velocity.length() > 0.1:
		vehicle_moved.emit(global_position, velocity)

func _update_physics(delta: float) -> void:
	"""Core physics update - runs at fixed timestep"""
	# Update input from InputManager singleton
	_throttle_input = InputManager.get_axis("accelerate")
	_brake_input = InputManager.get_axis("brake")
	_steering_input = InputManager.get_axis("steer_left") - InputManager.get_axis("steer_right")
	_clutch_input = InputManager.get_axis("clutch")
	_gear_input_direction = InputManager.get_axis("upshift") - InputManager.get_axis("downshift")
	
	# Check handbrake input
	var handbrake_pressed = InputManager.is_action_pressed("handbrake")
	if handbrake_pressed != is_handbrake_active:
		is_handbrake_active = handbrake_pressed
		handbrake_toggled.emit(is_handbrake_active)
	
	# Handle gear shifting
	_handle_gear_shifting(delta)
	
	# Update RPM based on gear and speed
	_update_engine_rpm(delta)
	
	# Calculate wheel forces
	_calculate_wheel_forces(delta)
	
	# Apply forces to vehicle
	_apply_vehicle_forces(delta)
	
	# Handle collisions
	_check_collisions(delta)
	
	# Update suspension
	_update_suspension(delta)
	
	# Track velocity changes
	_previous_velocity = velocity

func _handle_gear_shifting(delta: float) -> void:
	"""Handle automatic or manual gear shifting"""
	if _gear_input_direction != 0:
		_manual_shift(_gear_input_direction)
	elif current_gear == GearState.NEUTRAL:
		_auto_shift(delta)

func _manual_shift(direction: int) -> void:
	"""Manual gear shift request"""
	var old_gear = current_gear
	var new_gear = current_gear + direction
	
	# Clamp to valid range
	new_gear = clamp(new_gear, GearState.REVERSE, GearState.TENTH)
	
	# Prevent invalid shifts
	if new_gear == GearState.FIRST and old_gear == GearState.REVERSE:
		return  # Can't go directly from reverse to first
	
	if new_gear == GearState.REVERSE and old_gear == GearState.FIRST:
		return  # Can't go directly from first to reverse
	
	# Shift if different
	if new_gear != old_gear:
		current_gear = new_gear
		gear_changed.emit(old_gear, current_gear)
		
		# Add slight delay for realistic shift feel
		if is_engine_running:
			await get_tree().create_timer(0.15).timeout

func _auto_shift(delta: float) -> void:
	"""Automatic gear shifting based on RPM and speed"""
	if current_gear == GearState.NEUTRAL:
		# Start in first gear if moving forward
		if current_speed > 1.0:
			current_gear = GearState.FIRST
			gear_changed.emit(GearState.NEUTRAL, GearState.FIRST)
		return
	
	# Determine optimal gear based on RPM
	var target_gear = _calculate_optimal_gear()
	
	if target_gear != current_gear:
		var old_gear = current_gear
		current_gear = target_gear
		gear_changed.emit(old_gear, current_gear)

func _calculate_optimal_gear() -> int:
	"""Calculate the optimal gear based on current RPM and speed"""
	if current_speed < 2.0:
		return GearState.FIRST
	
	# Calculate approximate gear ratios for each gear
	var gear_ratios = [
		GEAR_RATIO_FIRST,
		GEAR_RATIO_SECOND,
		GEAR_RATIO_THIRD,
		GEAR_RATIO_FOURTH,
		GEAR_RATIO_FIFTH,
		GEAR_RATIO_SIXTH
	]
	
	var target_rpm = 0.0
	for i in range(gear_ratios.size()):
		var gear_rpm = _calculate_rpm_from_speed(current_speed, i + 1)
		if gear_rpm < REDLINE_RPM * 0.85:
			target_rpm = gear_rpm
			return i + 1
	
	return GearState.SIXTH  # Default to top gear

func _calculate_rpm_from_speed(speed: float, gear: int) -> float:
	"""Calculate engine RPM from vehicle speed and gear"""
	if gear <= 0:
		return IDLE_RPM
	
	var wheel_rotation_speed: float = speed / (WHEEL_RADIUS * TWO_PI)  # rotations per second
	var total_ratio: float = gear_ratios[gear - 1] * final_drive_ratio
	
	return wheel_rotation_speed * total_ratio * 60.0  # convert to RPM

# ============================================================================
# ENGINE & RPM MANAGEMENT
# ============================================================================
func _update_engine_rpm(delta: float) -> void:
	"""Update engine RPM based on inputs and physics"""
	if not is_engine_running:
		# Engine cooling down
		current_rpm = lerp(current_rpm, MIN_RPM, delta * 2.0)
		return
	
	# Calculate target RPM based on throttle and current conditions
	var target_rpm: float = 0.0
	
	if _throttle_input > 0.0:
		# Accelerating - increase RPM
		target_rpm = _calculate_acceleration_rpm()
	else:
		# Decelerating or coasting - decrease RPM
		target_rpm = _calculate_deceleration_rpm()
	
	# Apply smooth transitions
	var rpm_change_rate: float = 1500.0  # RPM per second
	var max_rpm_change: float = rpm_change_rate * delta
	
	if target_rpm > current_rpm:
		current_rpm = min(current_rpm + max_rpm_change, target_rpm)
	else:
		current_rpm = max(current_rpm - max_rpm_change, target_rpm)
	
	# Ensure we stay within bounds
	current_rpm = clamp(current_rpm, MIN_RPM, engine_redline)
	
	# Emit RPM change signal
	rpm_changed.emit(current_rpm)

func _calculate_acceleration_rpm() -> float:
	"""Calculate target RPM during acceleration"""
	var gear_ratio = _get_current_gear_ratio()
	var wheel_rpm: float = current_speed / (WHEEL_RADIUS * TWO_PI) * 60.0
	
	var theoretical_rpm = wheel_rpm * gear_ratio * final_drive_ratio
	
	# Allow RPM to rise above theoretical during hard acceleration
	var throttle_factor: float = 1.0 + (_throttle_input * 0.3)
	return theoretical_rpm * throttle_factor

func _calculate_deceleration_rpm() -> float:
	"""Calculate target RPM during deceleration"""
	var gear_ratio = _get_current_gear_ratio()
	var wheel_rpm: float = current_speed / (WHEEL_RADIUS * TWO_PI) * 60.0
	
	if gear_ratio > 0:
		return wheel_rpm * gear_ratio * final_drive_ratio
	else:
		return IDLE_RPM

func _get_current_gear_ratio() -> float:
	"""Get the current gear ratio"""
	match current_gear:
		GearState.REVERSE:
			return GEAR_RATIO_REVERSE
		GearState.FIRST:
			return GEAR_RATIO_FIRST
		GearState.SECOND:
			return GEAR_RATIO_SECOND
		GearState.THIRD:
			return GEAR_RATIO_THIRD
		GearState.FOURTH:
			return GEAR_RATIO_FOURTH
		GearState.FIFTH:
			return GEAR_RATIO_FIFTH
		GearState.SIXTH:
			return GEAR_RATIO_SIXTH
		_:
			return 1.0

# ============================================================================
# WHEEL FORCE CALCULATION
# ============================================================================
func _calculate_wheel_forces(delta: float) -> void:
	"""Calculate forces applied to each wheel"""
	var drive_wheels: Array[int] = []
	
	# Determine which wheels are driven
	match drivetrain_type:
		DrivetrainType.FWD:
			drive_wheels = [0, 1]  # Front wheels
		DrivetrainType.RWD:
			drive_wheels = [2, 3]  # Rear wheels
		DrivetrainType.AWD:
			drive_wheels = [0, 1, 2, 3]  # All wheels
	
	# Calculate drive torque
	var drive_torque: float = _calculate_drive_torque()
	
	# Distribute torque to drive wheels
	for wheel_index in range(4):
		var is_driven = drive_wheels.has(wheel_index)
		
		if is_driven and current_gear > 0:
			# Apply drive force
			var wheel_torque = drive_torque / drive_wheels.size()
			_wheel_forces[wheel_index] = wheel_torque / wheel_radius
		else:
			# Non-driven wheels - minimal resistance
			_wheel_forces[wheel_index] = 0.0
		
		# Apply braking force if needed
		if is_braking:
			var brake_bias: float = brake_bias_front if wheel_index < 2 else (1.0 - brake_bias_front)
			var wheel_brake_force: float = brake_force * brake_bias * _brake_input
			_wheel_forces[wheel_index] -= wheel_brake_force
		
		# Apply handbrake force to rear wheels
		if is_handbrake_active and wheel_index >= 2:
			_wheel_forces[wheel_index] -= handbrake_force * 0.5

func _calculate_drive_torque() -> float:
	"""Calculate available drive torque from engine"""
	# Torque curve approximation (peak around 4000 RPM)
	var normalized_rpm: float = (current_rpm - MIN_RPM) / (engine_redline - MIN_RPM)
	
	# Simple torque curve: peaks at 0.5 normalized RPM
	var torque_curve: float = 1.0 - abs(normalized_rpm - 0.5) * 2.0
	torque_curve = max(torque_curve, 0.3)  # Minimum 30% torque
	
	var engine_torque: float = engine_max_torque * torque_curve
	
	# Apply gear multiplication
	var gear_ratio: float = _get_current_gear_ratio()
	var total_ratio: float = gear_ratio * final_drive_ratio
	
	return engine_torque * total_ratio * transmission_ratio

# ============================================================================
# VEHICLE FORCES APPLICATION
# ============================================================================
func _apply_vehicle_forces(delta: float) -> void:
	"""Apply calculated forces to the vehicle body"""
	# Calculate total longitudinal force
	var total_longitudinal_force: float = 0.0
	for force in _wheel_forces:
		total_longitudinal_force += force
	
	# Apply aerodynamic drag
	var air_density: float = 1.225  # kg/m³ at sea level
	var drag_force: float = 0.5 * air_density * drag_coefficient * frontal_area * current_speed * current_speed
	
	# Apply downforce (increases tire grip at speed)
	var downforce: float = 0.5 * air_density * downforce_coefficient * frontal_area * current_speed * current_speed
	
	# Apply longitudinal force
	var force_vector: Vector3 = velocity.normalized() * total_longitudinal_force - Vector3.RIGHT * drag_force
	add_force(force_vector)
	
	# Apply lateral forces for steering
	_apply_steering_forces(delta)

func _apply_steering_forces(delta: float) -> void:
	"""Apply steering forces to change vehicle direction"""
	if current_speed < 1.0:
		# No steering effect at very low speeds
		return
	
	# Calculate desired steering angle
	target_steering_angle = clamp(
		steering_input * max_steering_angle * steering_sensitivity,
		-max_steering_angle,
		max_steering_angle
	)
	
	# Smooth steering transition
	actual_steering_angle = lerp(actual_steering_angle, target_steering_angle, delta * steering_return_rate)
	
	# Convert steering angle to radians
	var steering_rad: float = deg_to_rad(actual_steering_angle)
	
	# Apply lateral force based on steering
	var lateral_force: float = current_speed * 10.0 * sin(steering_rad)
	
	# Apply to vehicle
	add_force(Vector3.BACK * lateral_force)

# ============================================================================
# SUSPENSION UPDATE
# ============================================================================
func _update_suspension(delta: float) -> void:
	"""Update suspension for each wheel"""
	for wheel_index in range(4):
		var wheel_pos: Vector3 = global_position + transform.basis * _wheel_positions[wheel_index]
		var ray_cast: RayCast3D = _raycast_to_ground(wheel_pos)
		
		if ray_cast.is_colliding():
			var hit_point: Vector3 = ray_cast.get_collision_point()
			var distance: float = wheel_pos.y - hit_point.y
			
			# Calculate compression
			var compression: float = max(0.0, distance - suspension_rest_length)
			compression = min(compression, suspension_travel)
			
			_wheel_compression[wheel_index] = compression
			
			# Calculate spring force
			var spring_force: float = suspension_stiffness * compression
			var damping_force: float = suspension_damping * velocity.y
			
			# Total suspension force
			var total_force: float = spring_force - damping_force
			
			# Apply force upward
			add_force(Vector3.UP * total_force / 4.0)  # Divide among 4 wheels
			
			# Emit suspension bump signal
			suspension_bumped.emit(wheel_index, compression)
		else:
			# Wheel in air
			_wheel_compression[wheel_index] = 0.0

func _raycast_to_ground(start_position: Vector3) -> RayCast3D:
	"""Create ray cast to detect ground contact"""
	var ray_cast: RayCast3D = RayCast3D.new()
	ray_cast.target_position = Vector3.DOWN * (suspension_travel + suspension_rest_length + 1.0)
	ray_cast.collision_mask = 1  # Adjust based on your scene's collision layers
	ray_cast.force_non_colliding_iterations = true
	
	add_child(ray_cast)
	ray_cast.position = start_position
	ray_cast.force_raycast_update()
	
	return ray_cast

# ============================================================================
# COLLISION DETECTION
# ============================================================================
func _check_collisions(delta: float) -> void:
	"""Check for collisions and handle impacts"""
	if get_slide_collision_count() > 0:
		var collision = get_slide_collision(0)
		var collision_data: Dictionary = {
			"position": collision.get_position(),
			"normal": collision.get_normal(),
			"collider": collision.get_collider(),
			"time": delta
		}
		
		collision_detected.emit(collision_data)
		
		# Screen shake or visual feedback could be added here
		_last_collision_time = Time.get_ticks_msec() / 1000.0

func _handle_collision_impact(collision_data: Dictionary) -> void:
	"""Handle the physical impact of a collision"""
	var impact_velocity: float = velocity.dot(collision_data.normal)
	
	# Apply bounce force
	var bounce_factor: float = 0.3  # Restitution coefficient
	var bounce_force: Vector3 = collision_data.normal * impact_velocity * bounce_factor
	
	add_force(bounce_force * vehicle_mass)

# ============================================================================
# DRIFT MECHANICS
# ============================================================================
func _update_drift(delta: float) -> void:
	"""Update drift state and mechanics"""
	if is_handbrake_active and abs(current_speed) > 15.0:
		# Calculate drift angle based on slip
		var slip_angle: float = _calculate_slip_angle()
		
		if abs(slip_angle) > 10.0:
			if not is_drifting:
				is_drifting = true
				drift_started.emit(slip_angle)
			
			drift_angle = lerp(drift_angle, slip_angle, delta * 5.0)
			drift_factor = clamp(abs(drift_angle) / 30.0, 0.0, 1.0)
		else:
			_end_drift()
	else:
		_end_drift()

func _calculate_slip_angle() -> float:
	"""Calculate vehicle slip angle"""
	var velocity_direction: Vector3 = velocity.normalized()
	var forward_direction: Vector3 = transform.basis.z
	
	# Cross product to find angle between velocity and forward
	var cross: float = velocity_direction.x * forward_direction.z - velocity_direction.z * forward_direction.x
	var dot: float = velocity_direction.dot(forward_direction)
	
	return rad_to_deg(atan2(cross, dot))

func _end_drift() -> void:
	"""End drift state"""
	if is_drifting:
		is_drifting = false
		drift_ended.emit()
		drift_angle = 0.0
		drift_factor = 0.0

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================
func get_speed_kmh() -> float:
	"""Get current speed in km/h"""
	return current_speed * 3.6

func get_speed_mph() -> float:
	"""Get current speed in mph"""
	return current_speed * 2.237

func get_distance_traveled() -> float:
	"""Get total distance traveled since last reset"""
	return _distance_traveled

func reset_distance_traveled() -> void:
	"""Reset distance traveled counter"""
	_distance_traveled = 0.0

func start_engine() -> void:
	"""Start the vehicle engine"""
	if not is_engine_running:
		is_engine_running = true
		engine_started.emit()
		current_rpm = IDLE_RPM

func stop_engine() -> void:
	"""Stop the vehicle engine"""
	if is_engine_running:
		is_engine_running = false
		engine_stopped.emit()
		current_rpm = MIN_RPM

func toggle_engine() -> void:
	"""Toggle engine state"""
	if is_engine_running:
		stop_engine()
	else:
		start_engine()

func _interpolate_render_values() -> void:
	"""Smooth interpolation for rendering purposes"""
	# Steering angle smoothing
	actual_steering_angle = lerp(actual_steering_angle, target_steering_angle, 0.1)
	
	# RPM smoothing
	current_rpm = lerp(current_rpm, current_rpm, 0.05)  # Already smoothed in physics

func _set_gravity(value: float) -> void:
	"""Setter for gravity property"""
	PhysicsSettings.gravity = value

func _set_default_vehicle_mass(value: float) -> void:
	"""Setter for default vehicle mass"""
	vehicle_mass = value

func _set_final_drive_ratio(value: float) -> void:
	"""Setter for final drive ratio"""
	final_drive_ratio = value

func _set_wheel_radius(value: float) -> void:
	"""Setter for wheel radius"""
	wheel_radius = value

# ============================================================================
# DEBUG & VISUALIZATION
# ============================================================================
func _draw_debug_lines() -> void:
	"""Draw debug visualization lines (only in debug mode)"""
	if not GameManager.debug_mode:
		return
	
	# Draw wheel positions
	for i in range(4):
		var pos: Vector3 = global_position + transform.basis * _wheel_positions[i]
		Debug.draw_line(pos, pos + Vector3.UP * suspension_travel, Color.GREEN)
	
	# Draw velocity vector
	Debug.draw_arrow(global_position, velocity * 0.1, Color.YELLOW)
	
	# Draw steering angle indicator
	var steering_visual: float = actual_steering_angle * 0.01
	Debug.draw_arc(global_position, 1.0, steering_visual, Color.CYAN)

func get_status_info() -> Dictionary:
	"""Get comprehensive vehicle status information"""
	return {
		"speed_kmh": get_speed_kmh(),
		"speed_mph": get_speed_mph(),
		"rpm": current_rpm,
		"gear": current_gear,
		"throttle": _throttle_input,
		"brake": _brake_input,
		"steering": actual_steering_angle,
		"is_engine_running": is_engine_running,
		"is_braking": is_braking,
		"is_handbrake_active": is_handbrake_active,
		"is_drifting": is_drifting,
		"drift_angle": drift_angle,
		"wheel_forces": _wheel_forces,
		"suspension_compression": _wheel_compression
	}
</FILE "scripts/controllers/VehicleController.gd">