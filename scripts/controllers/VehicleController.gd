extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulator
## Implements throttle, brake, steering inputs, wheel forces, and gear shifting logic
## Copyright 2026 Thalamus Racing Simulator Project

signal speed_changed(new_speed: float)
signal gear_changed(old_gear: int, new_gear: int)
signal vehicle_collision(impulse: Vector3, collision_point: Vector3)
signal engine_rpm_changed(rpm: float)
signal lap_checkpoint_passed(checkpoint_id: int)

# ============================================================================
# PHYSICS CONSTANTS (from PhysicsSettings resource)
# ============================================================================
@export var vehicle_mass: float = 1500.0
@export var center_of_mass_offset: Vector3 = Vector3.ZERO
@export var wheel_base: float = 2.5
@export var track_width: float = 1.8
@export var wheel_radius: float = 0.35
@export var tire_friction: float = 1.2
@export var suspension_stiffness: float = 50000.0
@export var suspension_damping: float = 5000.0
@export var suspension_travel: float = 0.15

# ============================================================================
# GEAR BOX CONFIGURATION
# ============================================================================
enum Gear {
	REVERSE = -1,
	NEUTRAL = 0,
	FIRST = 1,
	SECOND = 2,
	THIRD = 3,
	FOURTH = 4,
	FIFTH = 5,
	SIXTH = 6,
	OVERDRIVE = 7
}

@export_group("Transmission Settings")
@export var max_rpm: float = 7000.0
@export var idle_rpm: float = 800.0
@export var redline_rpm: float = 6800.0
@export var gear_ratios: Array[float] = [4.0, 2.5, 1.8, 1.4, 1.1, 0.9, 0.8]
@export var final_drive_ratio: float = 3.5
@export var clutch_disengagement_rpm: float = 1000.0

# ============================================================================
# INPUT BINDINGS
# ============================================================================
@export_group("Input Configuration")
var _throttle_input: float = 0.0
var _brake_input: float = 0.0
var _steering_input: float = 0.0
var _handbrake_input: bool = false
var _clutch_input: bool = false
var _shift_up_input: bool = false
var _shift_down_input: bool = false

# ============================================================================
# CURRENT STATE
# ============================================================================
var current_gear: int = Gear.NEUTRAL
var target_gear: int = Gear.NEUTRAL
var engine_rpm: float = 0.0
var wheel_speed_rps: float = 0.0
var vehicle_speed_kmh: float = 0.0
var acceleration_force: float = 0.0
var braking_force: float = 0.0
var is_engine_running: bool = false
var is_clutch_engaged: bool = true
var is_in_revving: bool = false

# ============================================================================
# WHEEL PHYSICS
# ============================================================================
var front_wheels: Dictionary = {}
var rear_wheels: Dictionary = {}
var wheel_states: Array[Dictionary] = []

# ============================================================================
# AERODYNAMICS & DRAG
# ============================================================================
var drag_coefficient: float = 0.30
var frontal_area: float = 2.2
var air_density: float = 1.225
var downforce_coefficient: float = 0.8

# ============================================================================
# INTERNAL VARIABLES
# ============================================================================
var _physics_settings: PhysicsSettings = null
var _input_manager: InputManager = null
var _audio_manager: AudioManager = null
var _last_tick_time: float = 0.0
var _gear_shift_timer: float = 0.0
var _max_shift_time: float = 0.15
var _engine_sound_pitch: float = 1.0
var _current_lap_distance: float = 0.0
var _total_distance_traveled: float = 0.0

# ============================================================================
# LIFECYCLE METHODS
# ============================================================================
func _ready() -> void:
	_init_physics_settings()
	_init_audio_manager()
	_init_input_manager()
	_init_wheel_systems()
	_reset_vehicle_state()
	print("VehicleController initialized for ", get_parent().name if get_parent() else "unknown vehicle")

func _process(_delta: float) -> void:
	_process_inputs()
	_update_gear_shifting()
	_update_audio_effects()

func _physics_process(delta: float) -> void:
	_apply_physics(delta)
	_update_wheels(delta)
	_check_collisions()

# ============================================================================
# INITIALIZATION
# ============================================================================
func _init_physics_settings() -> void:
	if GameManager.has_singleton("PhysicsSettings"):
		_physics_settings = GameManager.get_singleton("PhysicsSettings")
	else:
		_physics_settings = preload("res://scripts/core/PhysicsSettings.gd").new()
	
	if _physics_settings:
		vehicle_mass = _physics_settings.default_vehicle_mass
		suspension_stiffness *= _physics_settings.physics_scale_factor

func _init_audio_manager() -> void:
	if GameManager.has_singleton("AudioManager"):
		_audio_manager = GameManager.get_singleton("AudioManager")

func _init_input_manager() -> void:
	if GameManager.has_singleton("InputManager"):
		_input_manager = GameManager.get_singleton("InputManager")

func _init_wheel_systems() -> void:
	# Initialize front wheel states
	front_wheels = {
		"left": {"position": Vector3(0, wheel_radius, -track_width / 2), "angle": 0.0},
		"right": {"position": Vector3(0, wheel_radius, track_width / 2), "angle": 0.0}
	}
	
	# Initialize rear wheel states (non-steering)
	rear_wheels = {
		"left": {"position": Vector3(0, wheel_radius, -track_width / 2)},
		"right": {"position": Vector3(0, wheel_radius, track_width / 2)}
	}
	
	# Create wheel state array for all 4 wheels
	wheel_states = [
		{"wheel": "front_left", "velocity": 0.0, "slip_ratio": 0.0, "load": 0.0},
		{"wheel": "front_right", "velocity": 0.0, "slip_ratio": 0.0, "load": 0.0},
		{"wheel": "rear_left", "velocity": 0.0, "slip_ratio": 0.0, "load": 0.0},
		{"wheel": "rear_right", "velocity": 0.0, "slip_ratio": 0.0, "load": 0.0}
	]

func _reset_vehicle_state() -> void:
	current_gear = Gear.NEUTRAL
	target_gear = Gear.NEUTRAL
	engine_rpm = idle_rpm
	wheel_speed_rps = 0.0
	vehicle_speed_kmh = 0.0
	acceleration_force = 0.0
	braking_force = 0.0
	is_engine_running = false
	is_clutch_engaged = true
	_throttle_input = 0.0
	_brake_input = 0.0
	_steering_input = 0.0
	_handbrake_input = false
	_clutch_input = false
	_shift_up_input = false
	_shift_down_input = false

# ============================================================================
# INPUT PROCESSING
# ============================================================================
func _process_inputs() -> void:
	if _input_manager:
		# Get normalized input values from InputManager
		var input_data = _input_manager.get_vehicle_inputs(get_instance_id())
		
		# Clamp inputs to valid range [-1.0, 1.0]
		_throttle_input = clamp(input_data.throttle, -1.0, 1.0)
		_brake_input = clamp(input_data.brake, 0.0, 1.0)
		_steering_input = clamp(input_data.steering, -1.0, 1.0)
		_handbrake_input = input_data.handbrake
		_clutch_input = input_data.clutch
		
		# Handle gear shift requests
		if input_data.shift_up:
			_request_shift_up()
		elif input_data.shift_down:
			_request_shift_down()
		
		# Engine start/stop
		if input_data.engine_start and not is_engine_running:
			start_engine()
		elif input_data.engine_stop and is_engine_running:
			stop_engine()

func _request_shift_up() -> void:
	if current_gear < Gear.OVERDRIVE and current_gear != Gear.NEUTRAL:
		target_gear = min(current_gear + 1, Gear.OVERDRIVE)

func _request_shift_down() -> void:
	if current_gear > Gear.FIRST and current_gear != Gear.NEUTRAL:
		target_gear = max(current_gear - 1, Gear.FIRST)

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================
func _update_gear_shifting() -> void:
	# Allow automatic upshifts at high RPM
	if target_gear == current_gear and engine_rpm >= redline_rpm:
		if current_gear < Gear.OVERDRIVE:
			target_gear = min(current_gear + 1, Gear.OVERDRIVE)
	
	# Prevent downshifting below idle RPM
	if target_gear < current_gear and engine_rpm <= clutch_disengagement_rpm:
		target_gear = current_gear
	
	# Execute gear shift if requested
	if target_gear != current_gear and _gear_shift_timer <= 0.0:
		_execute_gear_shift(target_gear)

func _execute_gear_shift(new_gear: int) -> void:
	var old_gear = current_gear
	current_gear = new_gear
	
	# Disengage clutch during shift
	is_clutch_engaged = false
	_gear_shift_timer = _max_shift_time
	
	# Emit signal
	emit_signal("gear_changed", old_gear, new_gear)
	
	# Audio feedback
	if _audio_manager:
		_audio_manager.play_sfx("gear_shift")
	
	# Re-engage clutch after shift delay
	get_tree().create_timer(_max_shift_time).connect(
		"timeout", Callable(self, "_engage_clutch")
	)

func _engage_clutch() -> void:
	if current_gear != Gear.NEUTRAL:
		is_clutch_engaged = true

# ============================================================================
# ENGINE CONTROL
# ============================================================================
func start_engine() -> void:
	is_engine_running = true
	engine_rpm = idle_rpm
	if _audio_manager:
		_audio_manager.play_sfx("engine_start")

func stop_engine() -> void:
	is_engine_running = false
	engine_rpm = 0.0
	if _audio_manager:
		_audio_manager.play_sfx("engine_stop")

func _update_engine_rpm(delta: float) -> void:
	if not is_engine_running:
		engine_rpm = 0.0
		return
	
	var gear_ratio = gear_ratios[current_gear - 1] if current_gear > 0 else 0.0
	var drive_ratio = final_drive_ratio * gear_ratio
	
	# Calculate engine load based on vehicle speed
	var wheel_rpm = wheel_speed_rps * 60.0
	var expected_engine_rpm = wheel_rpm * drive_ratio
	
	# Rev matching when downshifting
	if current_gear < target_gear or (current_gear > target_gear and not is_clutch_engaged):
		var rpm_diff = expected_engine_rpm - engine_rpm
		if abs(rpm_diff) > 500.0:
			engine_rpm += sign(rpm_diff) * 2000.0 * delta
	
	# Apply throttle effect
	if is_clutch_engaged:
		var throttle_effect = _throttle_input * 0.8
		var rpm_target = idle_rpm + (max_rpm - idle_rpm) * throttle_effect
		var rpm_change = (rpm_target - engine_rpm) * 0.1 * delta
		engine_rpm += rpm_change
	
	# Brake effect
	if _brake_input > 0.0:
		engine_rpm -= _brake_input * 100.0 * delta
	
	# Clamp RPM
	engine_rpm = clamp(engine_rpm, idle_rpm, max_rpm)
	
	# Redline protection
	if engine_rpm >= redline_rpm:
		engine_rpm = redline_rpm
		_is_in_revving = true
	else:
		_is_in_revving = false
	
	# Emit signal
	emit_signal("engine_rpm_changed", engine_rpm)

# ============================================================================
# VEHICLE PHYSICS APPLICATION
# ============================================================================
func _apply_physics(delta: float) -> void:
	_update_engine_rpm(delta)
	
	if not is_engine_running:
		_apply_drag_and_friction(delta)
		return
	
	# Calculate drivetrain force
	var drivetrain_efficiency = 0.85
	var torque_curve = _get_torque_curve(engine_rpm)
	var total_ratio = gear_ratios[current_gear - 1] if current_gear > 0 else 1.0
	total_ratio *= final_drive_ratio
	
	# Torque at wheels
	var wheel_torque = torque_curve * total_ratio * drivetrain_efficiency
	
	# Force at contact patch
	var drive_force = wheel_torque / wheel_radius
	
	# Apply acceleration
	if current_gear != Gear.NEUTRAL:
		acceleration_force = drive_force * _throttle_input
		
		# Forward/reverse direction
		var direction = 1.0 if current_gear > 0 else -1.0
		velocity.x += acceleration_force * direction * delta * 0.1

# ============================================================================
# WHEEL SYSTEM UPDATES
# ============================================================================
func _update_wheels(delta: float) -> void:
	# Update wheel rotational speed based on vehicle velocity
	var wheel_circumference = 2.0 * PI * wheel_radius
	wheel_speed_rps = abs(velocity.x) / wheel_circumference if velocity.x.length() > 0.0 else 0.0
	
	# Convert to km/h
	vehicle_speed_kmh = velocity.x * 3.6
	
	# Update wheel states
	for i in wheel_states.size():
		wheel_states[i].velocity = wheel_speed_rps
		wheel_states[i].slip_ratio = _calculate_slip_ratio(i)
		wheel_states[i].load = _calculate_wheel_load(i)

func _calculate_slip_ratio(wheel_index: int) -> float:
	if wheel_speed_rps == 0.0:
		return 0.0
	
	var wheel_velocity = wheel_speed_rps * wheel_circumference
	var slip = (wheel_velocity - velocity.x) / max(abs(velocity.x), 1.0)
	return clamp(slip, -1.0, 1.0)

func _calculate_wheel_load(wheel_index: int) -> float:
	# Simplified weight distribution
	var base_load = vehicle_mass * 9.81 / 4.0
	var lateral_load_transfer = abs(velocity.y) * 0.5
	var longitudinal_load_transfer = abs(acceleration_force) * 0.3
	
	match wheel_index:
		0, 1: # Front wheels
			return base_load + lateral_load_transfer + longitudinal_load_transfer
		_: # Rear wheels
			return base_load + lateral_load_transfer - longitudinal_load_transfer

# ============================================================================
# TORQUE CURVE CALCULATION
# ============================================================================
func _get_torque_curve(rpm: float) -> float:
	# Polynomial torque curve approximation
	# Peak torque around 4000 RPM, falling off at extremes
	var normalized_rpm = rpm / max_rpm
	
	# Torque curve polynomial (normalized 0-1)
	var torque = -4.0 * pow(normalized_rpm, 4) + 8.0 * pow(normalized_rpm, 3) - 5.0 * pow(normalized_rpm, 2) + 1.5 * normalized_rpm + 0.3
	
	# Apply min/max bounds
	torque = clamp(torque, 0.3, 1.0)
	
	# Multiply by base torque (Nm)
	var base_torque = 400.0  # Typical sports car peak torque
	return base_torque * torque

# ============================================================================
# DRAG AND FRICTION
# ============================================================================
func _apply_drag_and_friction(delta: float) -> void:
	# Aerodynamic drag
	var drag_force = 0.5 * drag_coefficient * frontal_area * air_density * pow(velocity.x, 2)
	
	# Rolling resistance
	var rolling_resistance = vehicle_mass * 9.81 * 0.015
	
	# Total resistive force
	var total_resistance = drag_force + rolling_resistance
	
	# Apply deceleration
	var deceleration = total_resistance / vehicle_mass
	velocity.x -= deceleration * delta
	
	# Stop completely if very slow
	if abs(velocity.x) < 0.1:
		velocity.x = 0.0

# ============================================================================
# COLLISION HANDLING
# ============================================================================
func _check_collisions() -> void:
	if colliding:
		var collision_info = _get_collision_details()
		emit_signal("vehicle_collision", collision_info.impulse, collision_info.point)
		
		# Audio feedback
		if _audio_manager:
			_audio_manager.play_sfx("collision_impact")
		
		# Screen shake effect
		if GameManager.debug_mode:
			_trigger_screen_shake(collision_info.impulse.length())

func _get_collision_details() -> Dictionary:
	var collider = get_collider()
	var impulse = get_collision_impulse()
	var point = get_collision_point()
	
	return {
		"collider": collider,
		"impulse": impulse,
		"point": point,
		"normal": get_collision_normal()
	}

func _trigger_screen_shake(magnitude: float) -> void:
	# Simple screen shake via camera offset
	var shake_intensity = magnitude * 0.01
	var shake_amount = Vector3(randf() - 0.5, randf() - 0.5, randf() - 0.5) * shake_intensity
	
	# This would typically affect a camera node
	# For now, just log it
	if GameManager.debug_mode:
		print("Screen shake: ", shake_amount)

# ============================================================================
# AUDIO EFFECTS
# ============================================================================
func _update_audio_effects() -> void:
	if not _audio_manager or not is_engine_running:
		return
	
	# Map engine RPM to sound pitch
	_engine_sound_pitch = lerp(0.5, 1.5, engine_rpm / max_rpm)
	
	# Update audio parameters
	_audio_manager.update_vehicle_audio(get_instance_id(), {
		"pitch": _engine_sound_pitch,
		"volume": 0.7,
		"rpm": engine_rpm
	})

# ============================================================================
# PUBLIC API
# ============================================================================
func apply_brake(force: float) -> void:
	_brake_input = clamp(force, 0.0, 1.0)

func apply_throttle(amount: float) -> void:
	_throttle_input = clamp(amount, -1.0, 1.0)

func apply_steering(angle: float) -> void:
	_steering_input = clamp(angle, -1.0, 1.0)

func set_gear(gear: int) -> void:
	if gear >= Gear.FIRST and gear <= Gear.OVERDRIVE:
		target_gear = gear
	elif gear == Gear.REVERSE:
		target_gear = Gear.REVERSE
	elif gear == Gear.NEUTRAL:
		target_gear = Gear.NEUTRAL

func get_vehicle_speed_kmh() -> float:
	return vehicle_speed_kmh

func get_current_gear() -> int:
	return current_gear

func get_engine_rpm() -> float:
	return engine_rpm

func reset_controls() -> void:
	_throttle_input = 0.0
	_brake_input = 0.0
	_steering_input = 0.0
	_handbrake_input = false
	_clutch_input = false

# ============================================================================
# DEBUG INFO
# ============================================================================
func _get_debug_info() -> Dictionary:
	return {
		"speed_kmh": roundi(vehicle_speed_kmh),
		"gear": current_gear,
		"rpm": roundi(engine_rpm),
		"throttle": roundf(_throttle_input),
		"brake": roundf(_brake_input),
		"steering": roundf(_steering_input),
		"is_engine_running": is_engine_running,
		"clutch_engaged": is_clutch_engaged
	}

</FILE "scripts/controllers/VehicleController.gd">>