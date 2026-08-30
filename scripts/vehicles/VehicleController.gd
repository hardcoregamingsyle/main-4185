extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller using PhysicsSettings constants
## Handles throttle, brake, steering inputs, wheel forces, gear shifting logic
## Copyright 2026 Thalamus Racing Simulator Project

# ============================================================================
# SIGNALS
# ============================================================================
signal engine_started
signal engine_stopped
signal gear_changed(old_gear: int, new_gear: int)
signal nitro_used(amount: float)
signal collision_detected(impact_force: Vector3, location: Vector3)
signal lap_completed(lap_number: int)
signal vehicle_destroyed(vehicle_id: String)

# ============================================================================
# CONSTANTS & CONFIGURATION
# ============================================================================
const MAX_GEAR_COUNT := 6
const GEAR_RATIOS: Array[float] = [3.5, 2.5, 1.8, 1.3, 1.0, 0.8]
const REVERSE_RATIO := -3.8
const IDLE_RPM := 800.0
const REDLINE_RPM := 7500.0
const OPTIMAL_POWER_RPM := 5500.0
const CLIP_THRESHOLD := 0.01

# ============================================================================
# PHYSICS SETTINGS INTEGRATION
# ============================================================================
var _physics_settings: PhysicsSettings = null

@export_group("Vehicle Configuration")
@export var vehicle_id: String = "vehicle_001"
@export var vehicle_type: VehicleType = VehicleType.SPORT
@export var vehicle_mass: float = 1500.0
@export var center_of_mass_offset: Vector3 = Vector3(0.0, 0.5, 0.0)

@export_group("Throttle Settings")
@export var throttle_response_curve: Curve = null
@export var max_throttle: float = 1.0
@export var min_throttle: float = 0.0

@export_group("Brake Settings")
@export var brake_strength: float = 2.0
@export var brake_lock_threshold: float = 0.9
@export var anti_lock_braking: bool = true

@export_group("Steering Settings")
@export var max_steering_angle: float = 30.0 * TAU / 180.0
@export var steering_speed: float = 15.0
@export var steering_damping: float = 0.85

@export_group("Gear Shifting")
@export var upshift_rpm_threshold: float = 6800.0
@export var downshift_rpm_threshold: float = 3000.0
@export var automatic_gearbox: bool = true

@export_group("Nitrous System")
@export var nitro_capacity: float = 100.0
@export var nitro_consumption_rate: float = 50.0
@export var nitro_boost_multiplier: float = 1.5
@export var nitro_cooldown_time: float = 5.0

@export_group("Wheel Configuration")
@export var front_wheel_width: float = 0.3
@export var rear_wheel_width: float = 0.4
@export var wheel_radius: float = 0.35
@export var suspension_travel: float = 0.15
@export var suspension_stiffness: float = 15000.0
@export var suspension_damping: float = 1500.0

@export_group("Aerodynamics")
@export var drag_coefficient: float = 0.30
@export var frontal_area: float = 2.2
@export var downforce_coefficient: float = 0.5
@export var wing_angle: float = 5.0

@export_group("Drivetrain")
@export var drivetrain_type: DrivetrainType = DrivetrainType.FWD
@export var front_torque_bias: float = 0.4
@export var rear_torque_bias: float = 0.6
@export var differential_type: DifferentialType = DifferentialType.LSD

# ============================================================================
# STATE VARIABLES
# ============================================================================
enum VehicleType {
	COMPACT,
	SPORT,
	SUPERCAR,
	TRUCK,
	MOTORCYCLE,
	GOKART
}

enum DrivetrainType {
	FWD,
	RWD,
	AWD,
	AWD_TORQUE_VECTORING
}

enum DifferentialType {
	OPEN,
	LSD,
	LOCKED
}

# Current state
var current_gear: int = 0
var rpm: float = IDLE_RPM
var speed_kmh: float = 0.0
var forward_speed: float = 0.0
var lateral_speed: float = 0.0
var vertical_speed: float = 0.0
var steering_input: float = 0.0
var throttle_input: float = 0.0
var brake_input: float = 0.0
var clutch_engaged: bool = true
var handbrake_active: bool = false

# Nitrous system
var nitro_available: float = 100.0
var nitro_in_use: bool = false
var last_nitro_use_time: float = 0.0

# Collision tracking
var collision_impacts: Array[Dictionary] = []
var collision_count: int = 0
var total_collision_damage: float = 0.0

# Wheel states (simplified for procedural wheel simulation)
var front_left_wheel_rotation: float = 0.0
var front_right_wheel_rotation: float = 0.0
var rear_left_wheel_rotation: float = 0.0
var rear_right_wheel_rotation: float = 0.0
var wheel_vertical_positions: Array[float] = [0.0, 0.0, 0.0, 0.0]

# Suspension states
var suspension_compression: Array[float] = [0.0, 0.0, 0.0, 0.0]
var suspension_velocity: Array[float] = [0.0, 0.0, 0.0, 0.0]

# Engine power
var engine_power_output: float = 0.0
var torque_output: float = 0.0
var engine_efficiency: float = 1.0

# Lap/race data
var current_lap: int = 1
var lap_times: Array[float] = []
var best_lap_time: float = 0.0
var race_distance: float = 0.0

# Timing
var last_update_time: float = 0.0
var delta_time: float = 0.0

# References
var powertrain_node: Node = null
var chassis_body: RigidBody3D = null
var wheel_colliders: Array[Area3D] = []

# ============================================================================
# INITIALIZATION
# ============================================================================
func _ready() -> void:
	# Load physics settings from autoload
	if GameManager.has_singleton("PhysicsSettings"):
		_physics_settings = PhysicsSettings.get_singleton()
	else:
		_physics_settings = PhysicsSettings.new()
	
	# Initialize default curves if not set
	if throttle_response_curve == null:
		throttle_response_curve = _create_throttle_curve()
	
	# Set initial values
	current_gear = 1
	rpm = IDLE_RPM
	speed_kmh = 0.0
	nitro_available = nitro_capacity
	
	# Get references
	_chassis_body = $ChassisBody
	if _chassis_body == null:
		_chassis_body = get_parent() as RigidBody3D
	
	# Find wheel colliders
	for child in get_children():
		if child is Area3D and child.name.contains("Wheel"):
			wheel_colliders.append(child)
	
	# Find powertrain node
	if $Powertrain != null:
		powertrain_node = $Powertrain
	
	# Connect to powertrain signals if available
	if powertrain_node != null:
		_connect_powertrain_signals()
	
	# Initial setup
	_setup_vehicle_geometry()
	_initialize_suspension()
	
	print("[VehicleController] %s initialized successfully" % vehicle_id)

func _connect_powertrain_signals() -> void:
	if powertrain_node and powertrain_node.has_signal("engine_revving"):
		powertrain_node.engine_revving.connect(_on_engine_revving)
	if powertrain_node and powertrain_node.has_signal("engine_overheat"):
		powertrain_node.engine_overheat.connect(_on_engine_overheat)

func _setup_vehicle_geometry() -> void:
	# Adjust center of mass
	if _chassis_body:
		_chassis_body.center_of_mass = center_of_mass_offset
	
	# Scale wheels based on configuration
	var wheel_scale := Vector3(wheel_radius, wheel_radius * 0.5, wheel_radius)
	
	# Apply to wheel meshes if they exist
	for child in get_children():
		if child is MeshInstance3D:
			if child.name.contains("Front"):
				child.scale.x = front_wheel_width / wheel_radius
			elif child.name.contains("Rear"):
				child.scale.x = rear_wheel_width / wheel_radius

func _initialize_suspension() -> void:
	suspension_compression.fill(0.0)
	suspension_velocity.fill(0.0)

func _create_throttle_curve() -> Curve:
	var curve := Curve.new()
	curve.add_point(Vector2(0.0, 0.0))
	curve.add_point(Vector2(0.3, 0.2))
	curve.add_point(Vector2(0.6, 0.5))
	curve.add_point(Vector2(0.8, 0.8))
	curve.add_point(Vector2(1.0, 1.0))
	return curve

# ============================================================================
# MAIN GAME LOOP
# ============================================================================
func _physics_process(delta: float) -> void:
	delta_time = delta
	
	# Update timing
	last_update_time = Time.get_ticks_msec() / 1000.0
	
	# Update inputs from InputManager
	_update_inputs()
	
	# Update RPM based on gear and throttle
	_update_engine_rpm()
	
	# Handle gear shifting
	_handle_gear_shifting()
	
	# Calculate speed
	_update_vehicle_speed()
	
	# Apply forces
	_apply_vehicle_forces()
	
	# Update suspension
	_update_suspension()
	
	# Check collisions
	_check_collisions()
	
	# Update nitrous cooldown
	_update_nitrous_system()
	
	# Update lap timing
	_update_race_data()
	
	# Emit updated state
	_emit_state_signals()

# ============================================================================
# INPUT HANDLING
# ============================================================================
func _update_inputs() -> void:
	# Read inputs from InputManager singleton
	if GameManager.has_singleton("InputManager"):
		var input_manager := InputManager.get_singleton()
		
		steering_input = input_manager.get_axis("steer_left", "steer_right")
		throttle_input = input_manager.get_axis("throttle", "brake")
		brake_input = input_manager.get_axis("brake", "reverse")
		clutch_engaged = !input_manager.is_action_pressed("clutch")
		handbrake_active = input_manager.is_action_pressed("handbrake")
		
		# Nitrous trigger
		if input_manager.is_action_just_pressed("nitro") and nitro_available > 0:
			_activate_nitrous()
		
		# Manual gear shift
		if not automatic_gearbox:
			if input_manager.is_action_just_pressed("upshift"):
				_change_gear(1)
			elif input_manager.is_action_just_pressed("downshift"):
				_change_gear(-1)

# ============================================================================
# ENGINE CONTROL
# ============================================================================
func _update_engine_rpm() -> void:
	var target_rpm: float = IDLE_RPM
	var gear_ratio: float = _get_current_gear_ratio()
	
	# Calculate target RPM based on gear and throttle
	if clutch_engaged or current_gear == 0:
		target_rpm = _calculate_target_rpm(gear_ratio)
	else:
		# Clutch disengaged, let engine idle
		target_rpm = lerp(rpm, IDLE_RPM, 0.1)
	
	# Smooth RPM transition
	var rpm_transition_speed: float = 150.0 * delta_time
	rpm = lerp(rpm, target_rpm, rpm_transition_speed)
	
	# Clamp RPM to valid range
	rpm = clamp(rpm, IDLE_RPM, REDLINE_RPM)

func _calculate_target_rpm(gear_ratio: float) -> float:
	var base_rpm: float = rpm
	var throttle_factor: float = throttle_response_curve.sample(throttle_input)
	
	# Calculate desired RPM based on throttle
	var desired_rpm: float = IDLE_RPM + (REDLINE_RPM - IDLE_RPM) * throttle_factor
	
	# Adjust based on current speed
	var speed_factor: float = speed_kmh / 200.0
	desired_rpm = lerp(desired_rpm, base_rpm, speed_factor * 0.3)
	
	return desired_rpm

func _get_current_gear_ratio() -> float:
	if current_gear <= 0:
		return REVERSE_RATIO
	elif current_gear >= MAX_GEAR_COUNT:
		return GEAR_RATIOS[MAX_GEAR_COUNT - 1]
	else:
		return GEAR_RATIOS[current_gear - 1]

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================
func _handle_gear_shifting() -> void:
	if not automatic_gearbox:
		return
	
	var should_upshift: bool = rpm > upshift_rpm_threshold
	var should_downshift: bool = rpm < downshift_rpm_threshold and speed_kmh > 5.0
	
	if should_upshift and current_gear < MAX_GEAR_COUNT:
		_change_gear(1)
	elif should_downshift and current_gear > 1:
		_change_gear(-1)
	elif should_downshift and current_gear == 1 and speed_kmh < 10.0:
		_change_gear(-1) # Downshift to neutral

func _change_gear(direction: int) -> void:
	var old_gear: int = current_gear
	
	# Handle neutral
	if current_gear == 0 and direction > 0:
		current_gear = 1
	elif current_gear == 1 and direction < 0:
		current_gear = 0
	else:
		current_gear = clamp(current_gear + direction, 0, MAX_GEAR_COUNT)
	
	# Only emit signal if gear actually changed
	if old_gear != current_gear:
		gear_changed.emit(old_gear, current_gear)
		
		# Sound effect via AudioManager
		if GameManager.has_singleton("AudioManager"):
			var audio_manager := AudioManager.get_singleton()
			audio_manager.play_sound("gear_shift")

func _rev_match(target_rpm: float) -> void:
	# Match engine RPM to match target gear speed
	var gear_ratio: float = _get_current_gear_ratio()
	var wheel_rpm: float = rpm / gear_ratio
	var target_gear_ratio: float = _get_gear_ratio_after_shift(gear_ratio)
	var matched_rpm: float = wheel_rpm * target_gear_ratio
	
	rpm = lerp(rpm, matched_rpm, 0.2)

func _get_gear_ratio_after_shift(current_ratio: float) -> float:
	if current_ratio == REVERSE_RATIO:
		return GEAR_RATIOS[0] # Upshift from reverse to first gear
	else:
		var index: int = GEAR_RATIOS.find(current_ratio)
		if index >= 0 and index < GEAR_RATIOS.size() - 1:
			return GEAR_RATIOS[index + 1]
		return current_ratio

# ============================================================================
# SPEED CALCULATION
# ============================================================================
func _update_vehicle_speed() -> void:
	# Speed from velocity vector
	var ground_velocity: Vector3 = global_transform.basis * velocity
	
	forward_speed = ground_velocity.z
	lateral_speed = ground_velocity.x
	vertical_speed = ground_velocity.y
	
	# Convert to km/h
	speed_kmh = abs(forward_speed) * 3.6
	
	# Handle reverse
	if forward_speed < 0:
		speed_kmh = -speed_kmh

# ============================================================================
# VEHICLE FORCES APPLICATION
# ============================================================================
func _apply_vehicle_forces() -> void:
	if not _chassis_body:
		return
	
	# Calculate engine force based on RPM and gear
	engine_power_output = _calculate_engine_power()
	torque_output = _calculate_engine_torque()
	
	# Apply drivetrain distribution
	var drive_force: float = _apply_drivetrain(torque_output)
	
	# Apply aerodynamic forces
	_apply_aerodynamics()
	
	# Apply braking forces
	_apply_brakes()
	
	# Apply steering forces
	_apply_steering()
	
	# Apply gravity
	_gravity = _physics_settings.gravity * Vector3.UP
	
	# Handle nitrous boost
	if nitro_in_use:
		drive_force *= nitro_boost_multiplier

func _calculate_engine_power() -> float:
	# Power increases with RPM up to optimal point, then decreases
	var power_curve: float = 1.0
	
	if rpm < OPTIMAL_POWER_RPM:
		power_curve = rpm / OPTIMAL_POWER_RPM
	else:
		power_curve = 1.0 - (rpm - OPTIMAL_POWER_RPM) / (REDLINE_RPM - OPTIMAL_POWER_RPM)
	
	var max_power: float = 250.0 # Base horsepower in kW
	var throttle_factor: float = throttle_response_curve.sample(throttle_input)
	
	return max_power * power_curve * throttle_factor * engine_efficiency

func _calculate_engine_torque() -> float:
	# Torque curve peaks earlier than power
	var torque_curve: float = 1.0
	
	if rpm < OPTIMAL_POWER_RPM * 0.7:
		torque_curve = rpm / (OPTIMAL_POWER_RPM * 0.7)
	elif rpm < OPTIMAL_POWER_RPM:
		torque_curve = 1.0
	else:
		torque_curve = 1.0 - (rpm - OPTIMAL_POWER_RPM) / (REDLINE_RPM - OPTIMAL_POWER_RPM) * 0.5
	
	var max_torque: float = 400.0 # Base torque in Nm
	var throttle_factor: float = throttle_response_curve.sample(throttle_input)
	
	return max_torque * torque_curve * throttle_factor * engine_efficiency

func _apply_drivetrain(torque: float) -> float:
	var drive_force: float = 0.0
	
	match drivetrain_type:
		DrivetrainType.FWD:
			drive_force = torque * front_torque_bias
		DrivetrainType.RWD:
			drive_force = torque * rear_torque_bias
		DrivetrainType.AWD:
			drive_force = torque * (front_torque_bias + rear_torque_bias)
		DrivetrainType.AWD_TORQUE_VECTORING:
			# More complex torque distribution based on cornering
			var cornering_factor: float = abs(lateral_speed) / 100.0
			var front_bias: float = front_torque_bias * (1.0 - cornering_factor)
			var rear_bias: float = rear_torque_bias * (1.0 + cornering_factor)
			drive_force = torque * (front_bias + rear_bias)
	
	# Apply to chassis body
	if _chassis_body:
		var forward_vector: Vector3 = global_transform.basis.z
		var force: Vector3 = forward_vector * drive_force * vehicle_mass
		
		_chassis_body.apply_central_force(force)
	
	return drive_force

func _apply_aerodynamics() -> void:
	if not _chassis_body:
		return
	
	var air_density: float = 1.225 # kg/m^3 at sea level
	var speed_ms: float = speed_kmh / 3.6
	
	# Drag force (opposes motion)
	var drag_force: float = 0.5 * air_density * drag_coefficient * frontal_area * speed_ms * speed_ms
	var drag_vector: Vector3 = -global_transform.basis.z.normalized() * drag_force
	
	_chassis_body.apply_central_force(drag_vector)
	
	# Downforce (pushes car into ground)
	var downforce: float = 0.5 * air_density * downforce_coefficient * frontal_area * speed_ms * speed_ms
	var downforce_vector: Vector3 = -Vector3.UP * downforce
	
	_chassis_body.apply_central_force(downforce_vector)

func _apply_brakes() -> void:
	if not _chassis_body:
		return
	
	var brake_force: float = 0.0
	
	if handbrake_active:
		# Handbrake affects all wheels equally
		brake_force = brake_strength * vehicle_mass * 0.5 * brake_input
	else:
		# Regular brakes
		brake_force = brake_strength * vehicle_mass * brake_input
		
		# Anti-lock braking prevents wheel lockup
		if anti_lock_braking and brake_input > brake_lock_threshold:
			brake_force *= (1.0 - (brake_input - brake_lock_threshold) * 0.5)
	
	# Apply brake force opposite to direction of travel
	if abs(speed_kmh) > 0.1:
		var brake_vector: Vector3 = -global_transform.basis.z.normalized() * brake_force
		_chassis_body.apply_central_force(brake_vector)
	else:
		# Static friction when stopped
		pass

func _apply_steering() -> void:
	# Steering only affects front wheels at low speeds
	if abs(speed_kmh) < 5.0:
		return
	
	var steering_effectiveness: float = 1.0 - abs(speed_kmh) / 200.0
	steering_effectiveness = clamp(steering_effectiveness, 0.0, 1.0)
	
	var actual_steering: float = steering_input * max_steering_angle * steering_effectiveness
	
	# Apply steering torque to front wheels (simulated)
	var steering_torque: float = actual_steering * 500.0 * steering_damping
	
	# Rotate front wheel nodes
	if $FrontLeftWheel:
		$FrontLeftWheel.rotation.y = lerp($FrontLeftWheel.rotation.y, -actual_steering, 0.1)
	if $FrontRightWheel:
		$FrontRightWheel.rotation.y = lerp($FrontRightWheel.rotation.y, actual_steering, 0.1)

# ============================================================================
# SUSPENSION SYSTEM
# ============================================================================
func _update_suspension() -> void:
	# Simple spring-damper model for each wheel
	for i in 4:
		var compression: float = suspension_compression[i]
		var velocity: float = suspension_velocity[i]
		
		# Spring force
		var spring_force: float = -suspension_stiffness * compression
		
		# Damping force
		var damping_force: float = -suspension_damping * velocity
		
		# Total force
		var total_force: float = spring_force + damping_force
		
		# Update position
		suspension_compression[i] += total_force * delta_time * 0.001
		suspension_compression[i] = clamp(suspension_compression[i], -suspension_travel, suspension_travel)
		
		# Update velocity
		suspension_velocity[i] = (suspension_compression[i] - suspension_compression[i]) / delta_time

func _check_wheel_ground_contact() -> void:
	for i in wheel_colliders.size():
		var collider: Area3D = wheel_colliders[i]
		var contact_count: int = collider.get_collision_count()
		
		if contact_count > 0:
			var contact_info: Dictionary = collider.get_collision_with_info(contact_count - 1)
			var contact_normal: Vector3 = contact_info.normal
			
			# Calculate compression based on contact normal
			var expected_height: float = wheel_radius
			var actual_height: float = contact_info.position.y - transform.origin.y
			
			suspension_compression[i] = expected_height - actual_height
		else:
			# Airborne
			suspension_compression[i] = suspension_travel

# ============================================================================
# COLLISION DETECTION
# ============================================================================
func _check_collisions() -> void:
	if not _chassis_body:
		return
	
	var collision_list: Array = _chassis_body.get_colliding_bodies()
	
	for collision_body in collision_list:
		var collision_info: Dictionary = _get_collision_details(collision_body)
		
		if collision_info["impact_force"] > 500.0: # Significant impact threshold
			collision_detected.emit(collision_info["impact_force"], collision_info["location"])
			
			collision_count += 1
			total_collision_damage += collision_info["impact_force"].length()
			
			collision_impacts.append({
				"time": last_update_time,
				"force": collision_info["impact_force"],
				"location": collision_info["location"],
				"body": collision_info["body"]
			})

func _get_collision_details(body: RigidBody3D) -> Dictionary:
	var details: Dictionary = {}
	
	# Get collision information
	var collision_points: Array = []
	for i in range(_chassis_body.get_collision_count()):
		var info: Dictionary = _chassis_body.get_collision_with_info(i)
		if info.collider == body:
			collision_points.append(info)
	
	if collision_points.is_empty():
		details["impact_force"] = Vector3.ZERO
		details["location"] = Vector3.ZERO
		details["body"] = body
		return details
	
	# Calculate average impact force
	var total_force: Vector3 = Vector3.ZERO
	var total_location: Vector3 = Vector3.ZERO
	
	for point in collision_points:
		total_force += point.local_impact_normal * point.local_impact_force
		total_location += point.position
	
	details["impact_force"] = total_force / collision_points.size()
	details["location"] = total_location / collision_points.size()
	details["body"] = body
	
	return details

func _emit_state_signals() -> void:
	# Emit periodic state updates
	if randf() < 0.1: # ~10% chance per frame
		# Could emit detailed vehicle state for debugging/UI
		pass

# ============================================================================
# NITROUS SYSTEM
# ============================================================================
func _activate_nitrous() -> void:
	if nitro_available <= 0:
		return
	
	nitro_in_use = true
	nitro_available -= nitro_consumption_rate
	last_nitro_use_time = last_update_time
	
	nitro_used.emit(nitro_consumption_rate)
	
	# Visual/audio feedback
	if GameManager.has_singleton("AudioManager"):
		var audio_manager := AudioManager.get_singleton()
		audio_manager.play_sound("nitro_activation")

func _update_nitrous_system() -> void:
	if nitro_in_use and last_update_time - last_nitro_use_time > 2.0:
		# Nitrous duration expired
		nitro_in_use = false
	
	if not nitro_in_use and nitro_available < nitro_capacity:
		# Recharge nitrous slowly
		nitro_available += nitro_capacity * 0.05 * delta_time
		nitro_available = min(nitro_available, nitro_capacity)

# ============================================================================
# RACE DATA TRACKING
# ============================================================================
func _update_race_data() -> void:
	# Track distance traveled
	race_distance += abs(forward_speed) * delta_time
	
	# Could add checkpoint detection here
	pass

func record_lap_complete() -> void:
	lap_completed.emit(current_lap)
	
	var lap_time: float = 0.0 # Would be calculated from checkpoint times
	lap_times.append(lap_time)
	
	if best_lap_time == 0.0 or lap_time < best_lap_time:
		best_lap_time = lap_time

func reset_race_data() -> void:
	current_lap = 1
	lap_times.clear()
	best_lap_time = 0.0
	race_distance = 0.0
	collision_count = 0
	total_collision_damage = 0.0
	collision_impacts.clear()

# ============================================================================
# ENGINE EVENTS
# ============================================================================
func _on_engine_revving(rpm_value: float) -> void:
	rpm = rpm_value
	engine_started.emit()

func _on_engine_overheat() -> void:
	# Reduce engine efficiency when overheating
	engine_efficiency = max(engine_efficiency - 0.1, 0.3)
	
	# Warning sound
	if GameManager.has_singleton("AudioManager"):
		var audio_manager := AudioManager.get_singleton()
		audio_manager.play_sound("engine_warning")

# ============================================================================
# VEHICLE CONTROLS
# ============================================================================
func start_engine() -> void:
	rpm = IDLE_RPM
	engine_efficiency = 1.0
	engine_started.emit()

func stop_engine() -> void:
	rpm = 0.0
	engine_efficiency = 0.0
	engine_stopped.emit()

func restart_vehicle() -> void:
	stop_engine()
	
	await get_tree().process_frame
	
	start_engine()
	reset_vehicle_position()

func reset_vehicle_position() -> void:
	if _chassis_body:
		_chassis_body.velocity = Vector3.ZERO
		_chassis_body.angular_velocity = Vector3.ZERO
		_chassis_body.global_transform.origin = Vector3(0.0, 2.0, 0.0)
	
	reset_race_data()

func set_vehicle_health(health: float) -> void:
	# Health affects performance
	engine_efficiency = health / 100.0
	vehicle_mass = 1500.0 * (1.0 - (1.0 - health / 100.0) * 0.1)

func get_vehicle_status() -> Dictionary:
	return {
		"vehicle_id": vehicle_id,
		"current_gear": current_gear,
		"rpm": rpm,
		"speed_kmh": speed_kmh,
		"forward_speed": forward_speed,
		"lateral_speed": lateral_speed,
		"throttle_input": throttle_input,
		"brake_input": brake_input,
		"steering_input": steering_input,
		"nitro_available": nitro_available,
		"collision_count": collision_count,
		"total_collision_damage": total_collision_damage,
		"current_lap": current_lap,
		"race_distance": race_distance
	}

func destroy_vehicle() -> void:
	vehicle_destroyed.emit(vehicle_id)
	
	# Remove from scene after delay
	await get_tree().create_timer(1.0).timeout
	queue_free()

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================
func get_wheel_position(wheel_index: int) -> Vector3:
	# Return world position of specified wheel
	var positions: Array[Vector3] = [
		Vector3(-0.8, 0.0, 1.2),   # Front Left
		Vector3(0.8, 0.0, 1.2),    # Front Right
		Vector3(-0.8, 0.0, -1.2),  # Rear Left
		Vector3(0.8, 0.0, -1.2)    # Rear Right
	]
	
	var local_pos: Vector3 = positions[wheel_index]
	local_pos.y += suspension_compression[wheel_index]
	
	return global_transform * local_pos

func set_drive_force(force: float) -> void:
	# Override automatic force calculation with manual force
	if _chassis_body:
		var forward_vector: Vector3 = global_transform.basis.z
		_chassis_body.apply_central_force(forward_vector * force)

func apply_impulse_at_wheel(wheel_index: int, impulse: Vector3) -> void:
	var wheel_pos: Vector3 = get_wheel_position(wheel_index)
	if _chassis_body:
		_chassis_body.apply_impulse(impulse, wheel_pos - _chassis_body.global_transform.origin)

</FILE>