extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for the racing simulator
## Handles throttle, brake, steering inputs, wheel forces, gear shifting, and vehicle dynamics
## All physics values are sourced from PhysicsSettings resource for centralized tuning
## Copyright 2026 Thalamus Racing Simulator Project

# ============================================================================
# SIGNALS
# ============================================================================
signal acceleration_changed(acceleration: float)
signal braking_changed(braking: float)
signal steering_changed(steering_angle: float)
signal gear_changed(old_gear: int, new_gear: int)
signal speed_changed(current_speed: float, max_speed: float)
signal lap_completed(lap_number: int, lap_time: float)
signal collision_detected(collision_info: Dictionary)

# ============================================================================
# ENUMERATIONS
# ============================================================================
enum Gear {
	PARK = 0,
	REVERSE = -1,
	NEUTRAL = 0,
	FIRST = 1,
	SECOND = 2,
	THIRD = 3,
	FOURTH = 4,
	FIFTH = 5,
	SIXTH = 6,
	SEVENTH = 7,
	EIGHTH = 8,
	OVERRIDE = 9  # Manual override for special modes
}

enum DriveType {
	FWD,   # Front-Wheel Drive
	RWD,   # Rear-Wheel Drive
	AWD,   # All-Wheel Drive
}

enum BrakeMode {
	STANDARD,     # Standard friction brakes
	Traction,     # Traction-focused braking
	Dynamic,      # Dynamic weight distribution
	Ballistic,    # Emergency ballistic stop
}

# ============================================================================
# EXPORTED CONFIGURATION
# ============================================================================
@export_group("Vehicle Configuration")
@export var vehicle_mass: float = 1500.0: set = _set_vehicle_mass
@export var center_of_mass_offset: Vector3 = Vector3.ZERO: set = _set_center_of_mass_offset
@export var drive_type: DriveType = DriveType.RWD
@export var brake_mode: BrakeMode = BrakeMode.STANDARD

@export_group("Performance Limits")
@export var max_top_speed_kmh: float = 350.0
@export var min_reverse_speed_kmh: float = -10.0
@export var acceleration_force: float = 8.0
@export var braking_force: float = 15.0
@export var steering_sensitivity: float = 1.0
@export var tire_friction_coefficient: float = 1.2

@export_group("Gear Ratios")
@export var gear_ratios: Array[float] = [5.0, 3.0, 2.0, 1.5, 1.2, 1.0, 0.8, 0.7]
@export var final_drive_ratio: float = 3.5
@export var clutch_disengagement_threshold: float = 0.1

@export_group("Wheel Configuration")
@export var wheel_base_length: float = 2.8
@export var track_width: float = 1.8
@export var wheel_radius: float = 0.35
@export var num_wheels: int = 4

# ============================================================================
# SINGLETON REFERENCES
# ============================================================================
var physics_settings: PhysicsSettings = null
var input_manager: InputManager = null
var game_manager: GameManager = null
var audio_manager: AudioManager = null

# ============================================================================
# INTERNAL STATE
# ============================================================================
var current_gear: int = Gear.NEUTRAL
var target_gear: int = Gear.NEUTRAL
var engine_rpm: float = 0.0
var max_engine_rpm: float = 8000.0
var idle_rpm: float = 800.0
var torque_curve: Array[float] = []
var wheel_torque_values: Array[float] = []

var speed_kmh: float = 0.0
var speed_ms: float = 0.0
var acceleration_value: float = 0.0
var braking_value: float = 0.0
var steering_value: float = 0.0

var throttle_input: float = 0.0
var brake_input: float = 0.0
var steering_input: float = 0.0

var is_moving: bool = false
var is_braking: bool = false
var is_reversing: bool = false
var has_control: bool = true

# Collision tracking
var last_collision_time: float = 0.0
var collision_count: int = 0
var total_collision_impact: float = 0.0

# Wheel state
var wheel_states: Array[Dictionary] = []

# ============================================================================
# INITIALIZATION
# ============================================================================
func _ready() -> void:
	_init_references()
	_configure_physics()
	_init_torque_curve()
	_init_wheel_states()
	_connect_signals()
	_apply_default_configuration()

func _init_references() -> void:
	# Get references to singletons
	if Engine.has_singleton("GameManager"):
		game_manager = Engine.get_singleton("GameManager")
	elif get_tree().root.has_node("GameManager"):
		game_manager = get_tree().root.get_node("GameManager")
	
	if Engine.has_singleton("InputManager"):
		input_manager = Engine.get_singleton("InputManager")
	elif get_tree().root.has_node("InputManager"):
		input_manager = get_tree().root.get_node("InputManager")
	
	if Engine.has_singleton("PhysicsSettings"):
		physics_settings = Engine.get_singleton("PhysicsSettings")
	elif get_tree().root.has_node("PhysicsSettings"):
		physics_settings = get_tree().root.get_node("PhysicsSettings")
	
	if Engine.has_singleton("AudioManager"):
		audio_manager = Engine.get_singleton("AudioManager")
	elif get_tree().root.has_node("AudioManager"):
		audio_manager = get_tree().root.get_node("AudioManager")

func _configure_physics() -> void:
	"""Configure physics properties based on settings"""
	if physics_settings:
		mass = physics_settings.default_vehicle_mass * (vehicle_mass / 1500.0)
	else:
		mass = vehicle_mass
	
	# Apply gravity scale
	gravity_scale = 1.0 if physics_settings == null else physics_settings.gravity / 9.81

func _init_torque_curve() -> void:
	"""Generate torque curve for engine simulation"""
	torque_curve.resize(100)
	var step: float = max_engine_rpm / 100.0
	
	for i in range(100):
		var rpm = i * step
		# Simulate typical V8 torque curve
		var normalized_rpm = rpm / max_engine_rpm
		var torque: float
		
		if normalized_rpm < 0.3:
			torque = 0.3 + (normalized_rpm * 2.0)  # Low RPM rise
		elif normalized_rpm < 0.7:
			torque = 1.5 + ((normalized_rpm - 0.3) * 1.5)  # Peak torque region
		else:
			torque = 2.5 - ((normalized_rpm - 0.7) * 0.5)  # High RPM decline
		
		torque_curve[i] = torque * 400.0  # Scale to Nm

func _init_wheel_states() -> void:
	"""Initialize wheel state tracking"""
	wheel_states.resize(num_wheels)
	
	var half_track: float = track_width / 2.0
	var half_base: float = wheel_base_length / 2.0
	
	for i in range(num_wheels):
		var position: Vector3
		var is_front: bool = (i % 2 == 0)
		
		match i:
			0: position = Vector3(-half_track, 0, half_base)  # Front Left
			1: position = Vector3(half_track, 0, half_base)   # Front Right
			2: position = Vector3(-half_track, 0, -half_base) # Rear Left
			3: position = Vector3(half_track, 0, -half_base)  # Rear Right
		
		wheel_states[i] = {
			"position": position,
			"is_front": is_front,
			"is_steering": is_front,
			"angular_velocity": 0.0,
			"slip_ratio": 0.0,
			"brake_pressure": 0.0,
			"drive_torque": 0.0
		}

func _connect_signals() -> void:
	"""Connect internal signals"""
	if input_manager:
		input_manager.input_changed.connect(_on_input_changed)

# ============================================================================
# CORE GAME LOOP
# ============================================================================
func _physics_process(delta: float) -> void:
	"""Main physics update loop"""
	if not has_control:
		return
	
	# Update delta time based on physics settings
	delta *= physics_settings.time_scale if physics_settings else 1.0
	
	# Read inputs
	_read_inputs()
	
	# Update vehicle dynamics
	_update_vehicle_dynamics(delta)
	
	# Handle gear shifting
	_handle_gear_shifting(delta)
	
	# Apply wheel forces
	_apply_wheel_forces(delta)
	
	# Update motion
	_update_motion(delta)
	
	# Check collisions
	_check_collisions()
	
	# Emit signals
	_emit_status_signals()

# ============================================================================
# INPUT HANDLING
# ============================================================================
func _read_inputs() -> void:
	"""Read and process player inputs"""
	if not input_manager:
		return
	
	# Read throttle input (accelerator pedal)
	throttle_input = clamp(input_manager.get_axis("throttle"), 0.0, 1.0)
	
	# Read brake input (brake pedal)
	brake_input = clamp(input_manager.get_axis("brake"), 0.0, 1.0)
	
	# Read steering input (left/right)
	steering_input = clamp(input_manager.get_axis("steer"), -1.0, 1.0)
	
	# Calculate derived values
	acceleration_value = throttle_input * acceleration_force
	braking_value = brake_input * braking_force
	steering_value = steering_input * steering_sensitivity

# ============================================================================
# VEHICLE DYNAMICS
# ============================================================================
func _update_vehicle_dynamics(delta: float) -> void:
	"""Update vehicle physical dynamics"""
	# Calculate current speed
	speed_ms = velocity.length()
	speed_kmh = speed_ms * 3.6
	
	# Determine if reversing
	is_reversing = speed_kmh < 0.5
	
	# Determine if moving
	is_moving = abs(speed_kmh) > 0.1
	
	# Update RPM based on gear and speed
	_update_engine_rpm()
	
	# Apply damping
	_apply_drag_and_friction(delta)

func _update_engine_rpm() -> void:
	"""Calculate engine RPM based on gear and speed"""
	if current_gear == Gear.PARK or current_gear == Gear.NEUTRAL:
		engine_rpm = idle_rpm
		return
	
	var gear_num: int = current_gear
	var ratio: float = gear_ratios[gear_num - 1] if gear_num <= len(gear_ratios) else 1.0
	
	# Calculate wheel rotation speed
	var wheel_circumference: float = PI * 2.0 * wheel_radius
	var wheel_rps: float = speed_ms / wheel_circumference
	
	# Calculate engine RPM
	var total_ratio: float = ratio * final_drive_ratio
	engine_rpm = wheel_rps * total_ratio * 60.0
	
	# Clamp RPM
	engine_rpm = clamp(engine_rpm, 0.0, max_engine_rpm)

func _apply_drag_and_friction(delta: float) -> void:
	"""Apply aerodynamic drag and rolling resistance"""
	if not physics_settings:
		return
	
	# Air density factor
	var air_density: float = 1.225
	
	# Drag coefficient (typical sports car)
	var drag_coefficient: float = 0.3
	
	# Frontal area
	var frontal_area: float = 2.2
	
	# Aerodynamic drag force
	var velocity_squared: float = speed_ms * speed_ms
	var drag_force: float = 0.5 * air_density * drag_coefficient * frontal_area * velocity_squared
	
	# Rolling resistance
	var normal_force: float = mass * physics_settings.gravity
	var rolling_resistance_coefficient: float = 0.015
	var rolling_resistance: float = normal_force * rolling_resistance_coefficient
	
	# Total resistive force
	var total_resistance: float = drag_force + rolling_resistance
	
	# Apply as opposite to velocity direction
	if velocity.length() > 0:
		var resistive_acceleration: float = total_resistance / mass
		var deceleration_vector: Vector3 = velocity.normalized() * resistive_acceleration * delta
		velocity -= deceleration_vector

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================
func _handle_gear_shifting(delta: float) -> void:
	"""Handle automatic and manual gear shifting"""
	# Auto-shift logic
	if current_gear != target_gear:
		_auto_shift_gears(delta)
	
	# Manual shift trigger check
	if input_manager and input_manager.is_action_just_pressed("shift_up"):
		_manual_shift_up()
	elif input_manager and input_manager.is_action_just_pressed("shift_down"):
		_manual_shift_down()

func _auto_shift_gears(delta: float) -> void:
	"""Automatic transmission gear shifting"""
	var target_gear_to_use: int = target_gear
	
	# Shift up logic
	if speed_kmh > 200.0 and current_gear < 8:
		target_gear_to_use = 8
	elif speed_kmh > 160.0 and current_gear < 7:
		target_gear_to_use = 7
	elif speed_kmh > 120.0 and current_gear < 6:
		target_gear_to_use = 6
	elif speed_kmh > 90.0 and current_gear < 5:
		target_gear_to_use = 5
	elif speed_kmh > 60.0 and current_gear < 4:
		target_gear_to_use = 4
	elif speed_kmh > 40.0 and current_gear < 3:
		target_gear_to_use = 3
	elif speed_kmh > 25.0 and current_gear < 2:
		target_gear_to_use = 2
	elif speed_kmh < 10.0 and current_gear > 1:
		target_gear_to_use = 1
	
	# Prevent downshift when going backwards
	if is_reversing and current_gear >= Gear.FIRST:
		target_gear_to_use = Gear.REVERSE
	
	# Execute gear change
	if current_gear != target_gear_to_use:
		var old_gear: int = current_gear
		current_gear = target_gear_to_use
		
		if audio_manager:
			audio_manager.play_sound("gear_change")
		
		gear_changed.emit(old_gear, current_gear)
		_on_gear_changed(old_gear, current_gear)

func _manual_shift_up() -> void:
	"""Manual gear up command"""
	if current_gear < 8:
		var old_gear: int = current_gear
		current_gear += 1
		if audio_manager:
			audio_manager.play_sound("gear_change")
		gear_changed.emit(old_gear, current_gear)
		_on_gear_changed(old_gear, current_gear)

func _manual_shift_down() -> void:
	"""Manual gear down command"""
	if current_gear > Gear.REVERSE:
		var old_gear: int = current_gear
		current_gear -= 1
		if audio_manager:
			audio_manager.play_sound("gear_change")
		gear_changed.emit(old_gear, current_gear)
		_on_gear_changed(old_gear, current_gear)

func _on_gear_changed(old_gear: int, new_gear: int) -> void:
	"""Called when gear changes"""
	pass  # Can be overridden by subclasses

func set_gear(new_gear: int) -> void:
	"""Set gear directly (for AI or networked control)"""
	if new_gear >= Gear.REVERSE and new_gear <= Gear.EIGHTH:
		var old_gear: int = current_gear
		current_gear = new_gear
		gear_changed.emit(old_gear, current_gear)
		_on_gear_changed(old_gear, current_gear)

func set_target_gear(new_target: int) -> void:
	"""Set target gear for auto-shift"""
	target_gear = clamp(new_target, Gear.REVERSE, Gear.EIGHTH)

# ============================================================================
# WHEEL FORCE APPLICATION
# ============================================================================
func _apply_wheel_forces(delta: float) -> void:
	"""Apply forces to each wheel based on driving configuration"""
	var total_torque: float = 0.0
	
	# Calculate available torque based on RPM
	var available_torque: float = _get_available_torque()
	
	# Distribute torque based on drive type
	var torque_per_wheel: float = available_torque / 4.0
	
	match drive_type:
		DriveType.FWD:
			# Front wheels only
			wheel_torque_values = [torque_per_wheel, torque_per_wheel, 0.0, 0.0]
		DriveType.RWD:
			# Rear wheels only
			wheel_torque_values = [0.0, 0.0, torque_per_wheel, torque_per_wheel]
		DriveType.AWD:
			# All wheels
			wheel_torque_values = [torque_per_wheel * 0.4, torque_per_wheel * 0.4, 
			                       torque_per_wheel * 0.6, torque_per_wheel * 0.6]
	
	# Apply traction control during braking
	if is_braking:
		_apply_traction_control()
	
	# Apply individual wheel torques
	_apply_individual_wheel_torques()

func _get_available_torque() -> float:
	"""Get available torque based on current RPM and gear"""
	if current_gear == Gear.PARK or current_gear == Gear.NEUTRAL:
		return 0.0
	
	var gear_num: int = current_gear
	var ratio: float = gear_ratios[gear_num - 1] if gear_num <= len(gear_ratios) else 1.0
	
	# Get torque from curve
	var rpm_index: int = int((engine_rpm / max_engine_rpm) * 100)
	rpm_index = clamp(rpm_index, 0, 99)
	var torque_from_curve: float = torque_curve[rpm_index]
	
	# Apply gear reduction
	var total_ratio: float = ratio * final_drive_ratio
	var wheel_torque: float = torque_from_curve * total_ratio
	
	# Apply throttle input
	wheel_torque *= throttle_input
	
	# Apply differential loss
	wheel_torque *= 0.85
	
	return wheel_torque

func _apply_traction_control() -> void:
	"""Apply traction control to prevent wheel spin during braking"""
	if not physics_settings:
		return
	
	var slip_threshold: float = 0.15
	
	for i in range(wheel_states.size()):
		var wheel_state: Dictionary = wheel_states[i]
		
		# Calculate slip ratio
		var wheel_speed: float = wheel_state.angular_velocity * wheel_radius
		var slip_ratio: float = abs(wheel_speed - speed_ms) / max(abs(speed_ms), 0.1)
		
		# Reduce torque if slipping too much
		if slip_ratio > slip_threshold:
			wheel_torque_values[i] *= (1.0 - (slip_ratio - slip_threshold) * 2.0)
			wheel_torque_values[i] = max(0.0, wheel_torque_values[i])

func _apply_individual_wheel_torques() -> void:
	"""Apply calculated torques to individual wheels"""
	for i in range(wheel_states.size()):
		wheel_states[i].drive_torque = wheel_torque_values[i]

# ============================================================================
# MOTION UPDATE
# ============================================================================
func _update_motion(delta: float) -> void:
	"""Update vehicle movement and velocity"""
	# Apply braking
	if brake_input > 0.0:
		_apply_brakes(brake_input, delta)
		is_braking = true
	else:
		is_braking = false
	
	# Apply acceleration
	if throttle_input > 0.0:
		_apply_acceleration(throttle_input, delta)
	
	# Apply steering
	_apply_steering(steering_value, delta)
	
	# Apply kinematic movement
	move_and_slide()

func _apply_brakes(brake_amount: float, delta: float) -> void:
	"""Apply braking force"""
	var braking_deceleration: float = braking_force * brake_amount
	
	# Different brake modes
	match brake_mode:
		BrakeMode.STANDARD:
			velocity -= velocity.normalized() * braking_deceleration * delta
		BrakeMode.Traction:
			# More aggressive rear braking
			velocity.x -= velocity.x * 1.2 * braking_deceleration * delta
			velocity.z -= velocity.z * braking_deceleration * delta
		BrakeMode.Dynamic:
			# Weight transfer simulation
			velocity -= velocity.normalized() * braking_deceleration * delta * 1.1
		BrakeMode.Ballistic:
			# Emergency stop
			velocity -= velocity.normalized() * braking_deceleration * 1.5 * delta

func _apply_acceleration(accel_amount: float, delta: float) -> void:
	"""Apply acceleration force in forward direction"""
	if current_gear == Gear.PARK or current_gear == Gear.NEUTRAL:
		return
	
	# Calculate forward vector based on vehicle orientation
	var forward_vector: Vector3 = transform.basis.z
	
	# Reverse if in reverse gear
	if current_gear == Gear.REVERSE:
		forward_vector = -forward_vector
	
	# Apply acceleration
	var accel_force: Vector3 = forward_vector * acceleration_value * accel_amount * delta
	velocity += accel_force

func _apply_steering(steer_amount: float, delta: float) -> void:
	"""Apply steering rotation"""
	if speed_kmh < 1.0:
		# Minimum speed threshold for steering
		return
	
	# Steering sensitivity based on speed
	var speed_factor: float = 1.0 - (min(abs(speed_kmh), 100.0) / 100.0) * 0.5
	
	var steer_rotation: float = steer_amount * 45.0 * speed_factor * deg_to_rad(1.0)
	
	# Rotate around Y axis
	var rotation_quat: Quaternion = Quaternion.IDENTITY.slerp(
		Quaternion(Vector3.UP, steer_rotation),
		delta * 10.0
	)
	transform.basis = transform.basis * rotation_quat.get_rotation_matrix()

# ============================================================================
# COLLISION HANDLING
# ============================================================================
func _check_collisions() -> void:
	"""Check for and handle collisions"""
	for i in range(get_contact_count()):
		var collision: KinematicCollision3D = get_slide_collision(i)
		
		if collision:
			_handle_collision(collision, i)

func _handle_collision(collision: KinematicCollision3D, index: int) -> void:
	"""Handle collision event"""
	var impact_speed: float = collision.get_travel()
	var impact_force: float = impact_speed * mass
	
	collision_count += 1
	total_collision_impact += impact_force
	
	var collision_data: Dictionary = {
		"time": Time.get_ticks_msec(),
		"impact_speed": impact_speed,
		"impact_force": impact_force,
		"collision_index": index,
		"normal": collision.get_normal(),
		"collider_id": collision.get_collider().get_instance_id()
	}
	
	collision_detected.emit(collision_data)
	
	# Play sound effect
	if audio_manager:
		audio_manager.play_sound("collision")
	
	# Screen shake effect
	_trigger_screen_shake(impact_force * 0.001)

func _trigger_screen_shake(amount: float) -> void:
	"""Trigger screen shake effect"""
	if audio_manager and audio_manager.shake_camera:
		audio_manager.shake_camera(amount)

# ============================================================================
# STATUS SIGNALS
# ============================================================================
func _emit_status_signals() -> void:
	"""Emit status update signals"""
	acceleration_changed.emit(acceleration_value)
	braking_changed.emit(braking_value)
	steering_changed.emit(steering_value)
	speed_changed.emit(speed_kmh, max_top_speed_kmh)

# ============================================================================
# PUBLIC API
# ============================================================================
func reset_vehicle() -> void:
	"""Reset vehicle to starting state"""
	current_gear = Gear.NEUTRAL
	target_gear = Gear.NEUTRAL
	engine_rpm = idle_rpm
	speed_kmh = 0.0
	speed_ms = 0.0
	velocity = Vector3.ZERO
	
	collision_count = 0
	total_collision_impact = 0.0
	last_collision_time = 0.0
	
	has_control = true

func take_control(active: bool = true) -> void:
	"""Enable or disable player control"""
	has_control = active

func set_max_speed(kmh: float) -> void:
	"""Set maximum vehicle speed"""
	max_top_speed_kmh = max(kmh, 10.0)

func get_current_speed() -> float:
	"""Get current speed in km/h"""
	return speed_kmh

func get_current_rpm() -> float:
	"""Get current engine RPM"""
	return engine_rpm

func get_current_gear() -> int:
	"""Get current gear number"""
	return current_gear

func is_braking_active() -> bool:
	"""Check if braking is currently applied"""
	return is_braking

func is_moving_forward() -> bool:
	"""Check if vehicle is moving forward"""
	return is_moving and not is_reversing

func is_in_park() -> bool:
	"""Check if vehicle is in park"""
	return current_gear == Gear.PARK

func is_neutral() -> bool:
	"""Check if vehicle is in neutral"""
	return current_gear == Gear.NEUTRAL

func calculate_lap_time() -> float:
	"""Calculate current lap time"""
	if game_manager and game_manager.current_state == GameManager.GameState.RACE_ACTIVE:
		return Time.get_ticks_msec() / 1000.0
	return 0.0

# ============================================================================
# SETTERS
# ============================================================================
func _set_vehicle_mass(value: float) -> void:
	vehicle_mass = max(value, 500.0)
	mass = vehicle_mass

func _set_center_of_mass_offset(value: Vector3) -> void:
	center_of_mass_offset = value
	# Note: Actual COM offset would require rigid body manipulation

# ============================================================================
# UTILITY METHODS
# ============================================================================
func debug_print_status() -> void:
	"""Print current vehicle status for debugging"""
	print("[VehicleController]")
	print("  Speed: %.1f km/h (%.1f m/s)" % [speed_kmh, speed_ms])
	print("  Gear: %d" % current_gear)
	print("  RPM: %.0f" % engine_rpm)
	print("  Throttle: %.2f" % throttle_input)
	print("  Brake: %.2f" % brake_input)
	print("  Steering: %.2f" % steering_input)
	print("  Moving: %s" % str(is_moving))
	print("  Reversing: %s" % str(is_reversing))

func save_state() -> Dictionary:
	"""Save current vehicle state"""
	return {
		"velocity": velocity,
		"position": global_position,
		"rotation": global_rotation,
		"current_gear": current_gear,
		"engine_rpm": engine_rpm,
		"speed_kmh": speed_kmh,
		"throttle_input": throttle_input,
		"brake_input": brake_input,
		"steering_input": steering_input,
		"collision_count": collision_count
	}

func load_state(state: Dictionary) -> void:
	"""Load saved vehicle state"""
	if state.has("velocity"):
		velocity = state["velocity"]
	if state.has("position"):
		global_position = state["position"]
	if state.has("rotation"):
		global_rotation = state["rotation"]
	if state.has("current_gear"):
		current_gear = state["current_gear"]
	if state.has("engine_rpm"):
		engine_rpm = state["engine_rpm"]
	if state.has("speed_kmh"):
		speed_kmh = state["speed_kmh"]
		speed_ms = speed_kmh / 3.6
	if state.has("throttle_input"):
		throttle_input = state["throttle_input"]
	if state.has("brake_input"):
		brake_input = state["brake_input"]
	if state.has("steering_input"):
		steering_input = state["steering_input"]

# ============================================================================
# DEBUG VISUALIZATION
# ============================================================================
func _draw_debug_lines() -> void:
	"""Draw debug visualization lines (editor only)"""
	if not Engine.editor_hint:
		return
	
	# Draw wheel positions
	for i in range(wheel_states.size()):
		var pos: Vector3 = global_position + transform.basis * wheel_states[i]["position"]
		draw_line(global_position, pos, Color.GREEN, 3.0)

# ============================================================================
# DESTRUCTOR
# ============================================================================
func _exit_tree() -> void:
	"""Cleanup on exit"""
	if audio_manager:
		pass  # Audio manager should handle cleanup

</FILE>