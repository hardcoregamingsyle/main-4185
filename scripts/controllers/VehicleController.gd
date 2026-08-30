extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulation
## Handles throttle, brake, steering inputs, wheel forces, and gear shifting logic
## Copyright 2026 Thalamus Racing Simulator Project

# ============================================================================
# SIGNALS
# ============================================================================
signal engine_started
signal engine_stopped
signal gear_changed(old_gear: int, new_gear: int)
signal nitro_used(amount: float)
signal collision_detected(impact_force: float, collision_point: Vector3)
signal wheel_slip(wheel_index: int, slip_ratio: float)
signal rpm_changed(rpm: float)

# ============================================================================
# CONFIGURATION CONSTANTS
# ============================================================================
const GEAR_RATIOS: Array[float] = [3.8, 2.4, 1.7, 1.3, 1.0, 0.8]
const FINAL_DRIVE_RATIO: float = 3.73
const MAX_RPM: float = 8500.0
const IDLE_RPM: float = 800.0
const REDLINE_RPM: float = 7500.0
const MIN_GEAR: int = -1  # Reverse
const MAX_GEAR: int = 5   # 6th gear (index 0-5)

# ============================================================================
# EXPORTED SETTINGS (PhysicsSettings Integration)
# ============================================================================
@export_group("Vehicle Configuration")
@export var vehicle_mass: float = 1500.0: set = _set_vehicle_mass
@export var center_of_mass: Vector3 = Vector3(0, 0.5, 0): set = _set_center_of_mass
@export var wheel_base: float = 2.7: set = _set_wheel_base
@export var track_width: float = 1.6: set = _set_track_width

@export_group("Engine Parameters")
@export var max_engine_torque: float = 500.0: set = _set_max_engine_torque
@export var max_engine_power: float = 350000.0  # Watts
@export var idle_rpm: float = IDLE_RPM
@export var redline_rpm: float = REDLINE_RPM
@export var rev_damping: float = 0.1

@export_group("Wheel Settings")
@export var tire_friction: float = 1.2
@export var suspension_stiffness: float = 35000.0
@export var suspension_damping: float = 2500.0
@export var suspension_travel: float = 0.15
@export var wheel_radius: float = 0.32

@export_group("Braking System")
@export var brake_force: float = 15000.0
@export var abs_enabled: bool = true
@export var brake_bias_front: float = 0.6

@export_group("Steering System")
@export var max_steering_angle: float = 35.0 * DEG2RAD
@export var steering_speed: float = 6.0
@export var ackermann_factor: float = 0.9

@export_group("Transmission")
@export var clutch_engagement_threshold: float = 0.1
@export var shift_up_shift_time: float = 0.2
@export var shift_down_shift_time: float = 0.15
@export var auto_shifting_enabled: bool = true

@export_group("Nitrous System")
@export var nitrous_available: bool = true
@export var nitrous_capacity: float = 100.0
@export var nitrous_consumption_rate: float = 50.0  # per second when active
@export var nitrous_boost_multiplier: float = 1.5

# ============================================================================
# INTERNAL STATE VARIABLES
# ============================================================================
var current_gear: int = 0
var target_gear: int = 0
var gear_shift_timer: float = 0.0
var is_gear_shifting: bool = false
var clutch_pedal: float = 1.0  # 1.0 = engaged, 0.0 = disengaged

var engine_rpm: float = IDLE_RPM
var engine_torque_output: float = 0.0
var engine_braking: bool = false

var throttle_input: float = 0.0
var brake_input: float = 0.0
var steering_input: float = 0.0

var current_nitrous: float = 0.0
var is_nitrous_active: bool = false

var wheel_positions: Array[Vector3] = []
var wheel_collisions: Array[Dictionary] = []
var wheel_contact_points: Array[Vector3] = []
var wheel_normal_forces: Array[float] = []

var velocity_world: Vector3 = Vector3.ZERO
var angular_velocity_world: Vector3 = Vector3.ZERO
var acceleration_vector: Vector3 = Vector3.ZERO

var total_lap_time: float = 0.0
var checkpoint_times: Dictionary = {}
var lap_count: int = 0

var is_moving: bool = false
var is_reversing: bool = false
var is_braking: bool = false

var audio_manager: AudioManager
var physics_settings: PhysicsSettings

# ============================================================================
# WHEEL RAYCAST SETUP
# ============================================================================
const WHEEL_FRONT_LEFT: int = 0
const WHEEL_FRONT_RIGHT: int = 1
const WHEEL_REAR_LEFT: int = 2
const WHEEL_REAR_RIGHT: int = 3

var wheel_raycast_nodes: Array[Node3D] = []
var wheel_suspension_length: float = 0.0

# ============================================================================
# DEBUG VISUALIZATION
# ============================================================================
@export var debug_visualization: bool = false
var debug_lines: Array[Line3D] = []

# ============================================================================
# INITIALIZATION
# ============================================================================
func _ready() -> void:
	_init_audio_manager()
	_init_physics_settings()
	_setup_wheels()
	_initialize_variables()
	_connect_signals()
	
	if Engine.is_editor_hint():
		return
	
	_set_process(true)
	_set_physics_process(true)

func _init_audio_manager() -> void:
	if GameManager.has_singleton("AudioManager"):
		audio_manager = GameManager.get_singleton("AudioManager")
	else:
		print("[VehicleController] Warning: AudioManager singleton not found")

func _init_physics_settings() -> void:
	if GameManager.has_singleton("PhysicsSettings"):
		physics_settings = GameManager.get_singleton("PhysicsSettings")
	else:
		print("[VehicleController] Warning: PhysicsSettings singleton not found")

func _setup_wheels() -> void:
	# Create wheel raycast nodes if they don't exist
	var wheel_names = ["front_left", "front_right", "rear_left", "rear_right"]
	wheel_raycast_nodes.resize(4)
	
	for i in range(4):
		var wheel_node = find_child("Wheel_" + wheel_names[i])
		if wheel_node == null:
			wheel_node = RayCast3D.new()
			wheel_node.name = "Wheel_" + wheel_names[i]
			add_child(wheel_node)
		
		wheel_raycast_nodes[i] = wheel_node
		
		# Configure raycast settings
		wheel_node.cast_to = Vector3.DOWN * (wheel_radius + suspension_travel)
		wheel_node.collision_mask = 1  # Adjust based on your collision layers

func _initialize_variables() -> void:
	# Reset all state variables
	current_gear = 0
	target_gear = 0
	gear_shift_timer = 0.0
	is_gear_shifting = false
	clutch_pedal = 1.0
	
	engine_rpm = IDLE_RPM
	engine_torque_output = 0.0
	engine_braking = false
	
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	
	current_nitrous = nitrous_capacity
	is_nitrous_active = false
	
	wheel_positions.clear()
	wheel_collisions.clear()
	wheel_contact_points.clear()
	wheel_normal_forces.clear()
	
	velocity_world = Vector3.ZERO
	angular_velocity_world = Vector3.ZERO
	acceleration_vector = Vector3.ZERO
	
	total_lap_time = 0.0
	lap_count = 0
	
	is_moving = false
	is_reversing = false
	is_braking = false
	
	debug_lines.clear()

func _connect_signals() -> void:
	# Connect to InputManager for input data
	if GameManager.has_singleton("InputManager"):
		var input_manager = GameManager.get_singleton("InputManager")
		input_manager.input_changed.connect(_on_input_changed)

# ============================================================================
# PHYSICS UPDATE LOOP
# ============================================================================
func _physics_process(delta: float) -> void:
	_update_input(delta)
	_update_engine(delta)
	_update_transmission(delta)
	_update_steering(delta)
	_update_wheels(delta)
	_update_aerodynamics(delta)
	_apply_physics(delta)
	_update_debug_visualization(delta)

func _update_input(delta: float) -> void:
	# Get input values from InputManager or direct input
	if GameManager.has_singleton("InputManager"):
		var input_manager = GameManager.get_singleton("InputManager")
		throttle_input = input_manager.get_throttle()
		brake_input = input_manager.get_brake()
		steering_input = input_manager.get_steering()
		clutch_pedal = input_manager.get_clutch()
		
		# Check for gear shift inputs
		if input_manager.shift_up_requested():
			_request_gear_shift(1)
		elif input_manager.shift_down_requested():
			_request_gear_shift(-1)
		
		# Check for nitrous activation
		if nitrous_available and input_manager.nitrous_pressed():
			_activate_nitrous()
	else:
		# Fallback to direct input if InputManager unavailable
		throttle_input = Input.get_action_strength("vehicle_throttle")
		brake_input = Input.get_action_strength("vehicle_brake")
		steering_input = Input.get_axis("vehicle_steering_left", "vehicle_steering_right")
		clutch_pedal = Input.get_action_strength("vehicle_clutch")
		
		if Input.is_action_just_pressed("vehicle_shift_up"):
			_request_gear_shift(1)
		elif Input.is_action_just_pressed("vehicle_shift_down"):
			_request_gear_shift(-1)
		
		if Input.is_action_just_pressed("vehicle_nitrous"):
			_activate_nitrous()

func _update_engine(delta: float) -> void:
	# Calculate engine RPM based on vehicle speed and gear
	if not is_zero_approx(velocity_world.length()):
		var wheel_rotation_speed = velocity_world.length() / wheel_radius
		var gear_ratio = GEAR_RATIOS[current_gear] if current_gear >= 0 else 1.0
		
		var theoretical_rpm = wheel_rotation_speed * gear_ratio * FINAL_DRIVE_RATIO * 60.0 / (2.0 * PI)
		
		# Smooth RPM transition
		var target_rpm = theoretical_rpm
		if is_gear_shifting:
			target_rpm = IDLE_RPM
		else:
			target_rpm = min(max_rpm, theoretical_rpm)
		
		engine_rpm = lerp(engine_rpm, target_rpm, delta * rev_damping * 10.0)
	else:
		# Engine idles when stationary
		if engine_rpm > IDLE_RPM:
			engine_rpm = lerp(engine_rpm, IDLE_RPM, delta * rev_damping * 5.0)
		else:
			engine_rpm = IDLE_RPM

func _calculate_engine_torque() -> float:
	# Torque curve based on RPM (simplified peak torque model)
	var normalized_rpm = (engine_rpm - IDLE_RPM) / (MAX_RPM - IDLE_RPM)
	
	if normalized_rpm < 0.0:
		normalized_rpm = 0.0
	elif normalized_rpm > 1.0:
		normalized_rpm = 1.0
	
	# Simulate torque curve with peak around 4000-5000 RPM
	var torque_curve_factor = -2.0 * pow(normalized_rpm - 0.6, 2.0) + 1.0
	torque_curve_factor = clamp(torque_curve_factor, 0.3, 1.0)
	
	# Apply throttle input
	var throttle_factor = clamp(throttle_input, 0.0, 1.0)
	
	# Apply nitrous boost
	var nitrous_factor = 1.0
	if is_nitrous_active and current_nitrous > 0.0:
		nitrous_factor = nitrous_boost_multiplier
		current_nitrous -= nitrous_consumption_rate * delta
		if current_nitrous < 0.0:
			current_nitrous = 0.0
			is_nitrous_active = false
	
	# Calculate final torque output
	var torque_output = max_engine_torque * torque_curve_factor * throttle_factor * nitrous_factor
	
	# Apply engine braking when off throttle
	if throttle_input < 0.1 and current_gear >= 0:
		engine_braking = true
		torque_output *= -0.3  # Reduced negative torque
	else:
		engine_braking = false
	
	return torque_output

func _update_transmission(delta: float) -> void:
	# Handle gear shifting
	if is_gear_shifting:
		gear_shift_timer += delta
		
		if gear_shift_timer >= (shift_up_shift_time if target_gear > current_gear else shift_down_shift_time):
			_complete_gear_shift()
	else:
		# Auto-shifting logic
		if auto_shifting_enabled and current_gear >= 0:
			_auto_shift_gear()

func _request_gear_shift(direction: int) -> void:
	if direction == 0:
		return
	
	var new_gear = current_gear + direction
	
	# Validate gear range
	if new_gear < MIN_GEAR or new_gear > MAX_GEAR:
		return
	
	# Disengage clutch during shift
	clutch_pedal = 0.0
	target_gear = new_gear
	is_gear_shifting = true
	gear_shift_timer = 0.0
	
	# Signal gear change start
	gear_changed.emit(current_gear, new_gear)

func _complete_gear_shift() -> void:
	current_gear = target_gard
	is_gear_shifting = false
	clutch_pedal = 1.0
	
	# Re-engage clutch smoothly
	var engagement_timer: float = 0.0
	
func _auto_shift_gear() -> void:
	# Simple auto-shift logic based on RPM
	if engine_rpm >= MAX_RPM and current_gear < MAX_GEAR:
		_request_gear_shift(1)
	elif engine_rpm <= IDLE_RPM and current_gear > MIN_GEAR:
		_request_gear_shift(-1)

func _activate_nitrous() -> void:
	if nitrous_available and current_nitrous > 0.0 and not is_nitrous_active:
		is_nitrous_active = true
		nitro_used.emit(nitrous_capacity - current_nitrous)

# ============================================================================
# STEERING SYSTEM
# ============================================================================
func _update_steering(delta: float) -> void:
	# Apply steering angle to wheels
	var target_steering = steering_input * max_steering_angle
	
	# Smooth steering transition
	var current_steering_angle = 0.0  # Track actual wheel angle
	
	if is_zero_approx(target_steering):
		current_steering_angle = lerp(current_steering_angle, 0.0, delta * steering_speed)
	else:
		var dir = sign(target_steering - current_steering_angle)
		current_steering_angle += dir * min(abs(target_steering - current_steering_angle), delta * steering_speed)
	
	# Apply to front wheels
	_set_wheel_steering(WHEEL_FRONT_LEFT, current_steering_angle)
	_set_wheel_steering(WHEEL_FRONT_RIGHT, -current_steering_angle * ackermann_factor)

func _set_wheel_steering(wheel_index: int, angle: float) -> void:
	pass  # Implementation depends on wheel node structure

# ============================================================================
# WHEEL PHYSICS
# ============================================================================
func _update_wheels(delta: float) -> void:
	# Update wheel positions and collect collisions
	wheel_positions.clear()
	wheel_collisions.clear()
	wheel_contact_points.clear()
	wheel_normal_forces.clear()
	
	var half_track = track_width * 0.5
	var half_wheelbase = wheel_base * 0.5
	
	# Define wheel positions relative to vehicle center
	var wheel_offsets = [
		Vector3(half_track, 0, half_wheelbase),      # Front Left
		Vector3(-half_track, 0, half_wheelbase),     # Front Right
		Vector3(half_track, 0, -half_wheelbase),     # Rear Left
		Vector3(-half_track, 0, -half_wheelbase)     # Rear Right
	]
	
	for i in range(4):
		var world_position = global_position + global_transform.basis * wheel_offsets[i]
		wheel_positions.append(world_position)
		
		# Perform raycast for suspension
		var result = _get_wheel_collision(i, world_position)
		wheel_collisions.append(result)
		
		if result.collider != null:
			wheel_contact_points.append(result.position)
			wheel_normal_forces.append(result.normal.y)  # Y component is vertical force

func _get_wheel_collision(wheel_index: int, start_position: Vector3) -> Dictionary:
	var result: Dictionary = {
		"hit": false,
		"position": Vector3.ZERO,
		"normal": Vector3.UP,
		"collider": null
	}
	
	if wheel_raycast_nodes.size() <= wheel_index:
		return result
	
	var raycast = wheel_raycast_nodes[wheel_index]
	raycast.transform = Transform3D(Basis(), start_position)
	raycast.force_raycast_update()
	
	if raycast.is_colliding():
		result["hit"] = true
		result["position"] = raycast.get_collision_point()
		result["normal"] = raycast.get_collision_normal()
		result["collider"] = raycast.get_collider()
	
	return result

func _apply_wheel_forces(delta: float) -> void:
	# Apply forces to each wheel based on traction and suspension
	var total_vertical_force: float = 0.0
	var total_traction_force: Vector3 = Vector3.ZERO
	
	for i in range(4):
		if wheel_collisions[i].hit:
			var contact_point = wheel_collisions[i].position
			var normal = wheel_collisions[i].normal
			
			# Calculate suspension compression
			var suspension_compression = max(0.0, (wheel_radius + suspension_travel) - (contact_point.y - global_position.y))
			
			# Suspension force (spring + damper)
			var spring_force = -suspension_stiffness * suspension_compression
			var damper_force = -suspension_damping * normal.dot(get_linear_velocity())
			var suspension_total = spring_force + damper_force
			
			total_vertical_force += suspension_total
			
			# Calculate traction force
			var drive_force = _calculate_wheel_drive_force(i, delta)
			var braking_force = _calculate_wheel_braking_force(i, delta)
			
			var wheel_total_force = drive_force + braking_force
			
			# Apply force at contact point
			apply_central_impulse(wheel_total_force * delta)
	
	# Apply overall gravity correction
	var gravity_correction = (vehicle_mass * physics_settings.gravity) - total_vertical_force
	apply_central_impulse(Vector3.UP * gravity_correction * delta)

func _calculate_wheel_drive_force(wheel_index: int, delta: float) -> Vector3:
	var is_front_wheel = wheel_index < 2
	var is_rear_wheel = wheel_index >= 2
	
	# Determine if this wheel is driven
	var is_driven = false
	if is_rear_wheel:
		is_driven = true  # Assume rear-wheel drive for now
	elif is_front_wheel:
		is_driven = false  # Not driven
	
	if not is_driven:
		return Vector3.ZERO
	
	# Calculate wheel speed
	var wheel_speed = velocity_world.length()
	
	# Slip ratio calculation
	var drive_speed = engine_rpm * wheel_radius * 2.0 * PI / 60.0
	var slip_ratio = (drive_speed - wheel_speed) / max(0.1, wheel_speed)
	
	# Traction coefficient based on slip
	var traction_coef = tire_friction * clamp(1.0 - abs(slip_ratio), 0.0, 1.0)
	
	# Calculate force magnitude
	var force_magnitude = engine_torque_output * GEAR_RATIOS[current_gear] * FINAL_DRIVE_RATIO / wheel_radius
	
	# Limit force by available traction
	var max_traction = brake_force * traction_coef
	force_magnitude = min(force_magnitude, max_traction)
	
	# Direction of force (forward/backward)
	var forward_direction = global_transform.basis.z
	
	if is_reversing:
		forward_direction = -forward_direction
	
	return forward_direction * force_magnitude

func _calculate_wheel_braking_force(wheel_index: int, delta: float) -> Vector3:
	if brake_input <= 0.0:
		return Vector3.ZERO
	
	# Distribute brake force between axles
	var brake_distribution = brake_bias_front if wheel_index < 2 else (1.0 - brake_bias_front)
	
	# Apply ABS logic if enabled
	var actual_brake_force = brake_force * brake_input * brake_distribution
	
	if abs_enabled and wheel_collisions[wheel_index].hit:
		var wheel_speed = velocity_world.length()
		var slip_speed = engine_rpm * wheel_radius * 2.0 * PI / 60.0 - wheel_speed
		
		# Reduce brake force if wheel is locking up
		if abs(slip_speed) < 0.5:
			actual_brake_force *= 0.5
	
	return -global_transform.basis.z * actual_brake_force

# ============================================================================
# AERODYNAMICS & PHYSICS
# ============================================================================
func _update_aerodynamics(delta: float) -> void:
	# Simple aerodynamic drag model
	var velocity_magnitude = velocity_world.length()
	
	if velocity_magnitude > 0.1:
		var drag_coefficient = 0.3  # Cd value
		var frontal_area = 2.2  # m² typical car
		var air_density = 1.225  # kg/m³
		
		var drag_force = 0.5 * air_density * drag_coefficient * frontal_area * velocity_magnitude * velocity_magnitude
		
		var drag_direction = -velocity_world.normalized()
		apply_central_force(drag_direction * drag_force)

func _apply_physics(delta: float) -> void:
	# Clear previous frame's accumulated impulses
	clear_accumulated_impulses()
	
	# Apply calculated forces
	_apply_wheel_forces(delta)
	_update_aerodynamics(delta)
	
	# Update velocity
	move_and_slide()
	
	# Record state
	velocity_world = linear_velocity
	is_moving = velocity_world.length() > 0.1
	is_reversing = linear_velocity.z < -0.1
	is_braking = brake_input > 0.1

# ============================================================================
# DEBUG & VISUALIZATION
# ============================================================================
func _update_debug_visualization(delta: float) -> void:
	if not debug_visualization or Engine.is_editor_hint():
		return
	
	# Draw debug lines for wheel positions and forces
	for i in range(4):
		if wheel_positions.size() > i:
			var position = wheel_positions[i]
			
			# Draw suspension line
			var line = Line3D.new()
			line.add_point(position)
			line.add_point(position + Vector3.UP * (wheel_radius + suspension_travel))
			line.material = StandardMaterial3D.new()
			line.material.albedo_color = Color.GREEN
			line.width = 2
			
			if debug_lines.size() <= i * 2:
				debug_lines.append_array([line, line.duplicate()])
			
			debug_lines[i * 2].add_point(position)
			debug_lines[i * 2].add_point(position + Vector3.UP * (wheel_radius + suspension_travel))

func _set_vehicle_mass(new_mass: float) -> void:
	vehicle_mass = new_mass
	mass = vehicle_mass

func _set_center_of_mass(new_com: Vector3) -> void:
	center_of_mass = new_com

func _set_wheel_base(new_base: float) -> void:
	wheel_base = new_base

func _set_track_width(new_width: float) -> void:
	track_width = new_width

func _set_max_engine_torque(new_torque: float) -> void:
	max_engine_torque = new_torque

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================
func get_current_speed() -> float:
	return velocity_world.length() * 3.6  # Convert to km/h

func get_current_rpm() -> float:
	return engine_rpm

func get_current_gear() -> int:
	return current_gear

func get_nitrous_level() -> float:
	return current_nitrous

func is_engine_running() -> bool:
	return engine_rpm > IDLE_RPM

func reset_vehicle() -> void:
	_initialize_variables()
	global_position = Vector3.ZERO
	linear_velocity = Vector3.ZERO
	angular_velocity = Vector3.ZERO

func _on_input_changed(input_data: Dictionary) -> void:
	throttle_input = input_data.throttle
	brake_input = input_data.brake
	steering_input = input_data.steering
	clutch_pedal = input_data.clutch

# ============================================================================
# CLEANUP
# ============================================================================
func _exit_tree() -> void:
	debug_lines.clear()
	wheel_raycast_nodes.clear()
	wheel_positions.clear()
	wheel_collisions.clear()
	wheel_contact_points.clear()
	wheel_normal_forces.clear()