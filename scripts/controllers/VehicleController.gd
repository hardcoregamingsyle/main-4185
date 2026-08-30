extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for the racing simulator
## Handles throttle, brake, steering inputs, wheel forces, and gear shift logic
## Integrates with PhysicsSettings, InputManager, GameManager, AudioManager, and Powertrain
## Copyright 2026 Thalamus Racing Simulator Project

# Signals for external communication
signal speed_changed(current_speed: float)
signal gear_changed(old_gear: int, new_gear: int)
signal vehicle_state_changed(state: VehicleState)
signal wheel_slip_detected(wheel_index: int, slip_ratio: float)
signal crash_detected(impact_force: float)

enum VehicleState {
	IDLE,
	DRIVING,
	BRAKING,
	SLIDING,
	CRASHED,
	REV_UP
}

# Physics constants reference
@onready var _physics_settings: PhysicsSettings = $VehiclePhysics/PhysicsSettings

# Vehicle state tracking
var current_gear: int = 1
var target_gear: int = 1
var max_gears: int = 6
var is_reverse_capable: bool = true
var clutch_engaged: bool = true

# Movement properties
var current_speed: float = 0.0
var target_max_speed: float = 0.0
var acceleration_rate: float = 0.0
var braking_rate: float = 0.0
var turning_angle: float = 0.0
var turning_speed: float = 0.0

# Input handling (values from -1.0 to 1.0)
var throttle_input: float = 0.0
var brake_input: float = 0.0
var steering_input: float = 0.0

# Wheel states (for 4-wheel vehicles)
const WHEEL_COUNT: int = 4
var wheel_states: Array[Dictionary] = []
var wheel_rpm: Array[float] = []
var wheel_slip_ratio: Array[float] = []

# Powertrain integration
@onready var powertrain: Powertrain = $VehiclePhysics/Powertrain
@onready var differential: Node3D = $VehiclePhysics/Differential
@onready var chassis: CharacterBody3D = $Chassis
@onready var suspension_system: Node3D = $VehiclePhysics/SuspensionSystem

# Audio references
var engine_sound_player: AudioStreamPlayer3D = null
var tire_squeal_player: AudioStreamPlayer3D = null
var crash_sound_player: AudioStreamPlayer3D = null

# Collision tracking
var last_collision_time: float = 0.0
var collision_impact: Vector3 = Vector3.ZERO
var is_crashed: bool = false
var respawn_timer: float = 0.0

# Time tracking
var _delta_accumulator: float = 0.0
var _last_frame_time: float = 0.0

# Tuning parameters (can be overridden by exported values)
@export_group("Acceleration Tuning")
@export var min_acceleration: float = 5.0
@export var max_acceleration: float = 20.0
@export var low_rpm_acceleration: float = 1.5
@export var high_rpm_acceleration: float = 0.8

@export_group("Braking Tuning")
@export var max_braking_force: float = 15.0
@export var abs_enabled: bool = true
@export var brake_bias_front: float = 0.6

@export_group("Steering Tuning")
@export var max_steering_angle: float = 30.0  # degrees
@export var steering_rate: float = 0.5
@export var steering_return_speed: float = 1.0

@export_group("Drivetrain Tuning")
@export var final_drive_ratio: float = 3.5
@export var differential_type: DifferentialType = DifferentialType.OPEN
@export var limited_slip_diff: float = 0.75

enum DifferentialType {
	OPEN,
	LIMITED_SLIP,
	LOCKER
}

# Gear ratios (gear ratio * final drive)
var gear_ratios: Array[float] = [3.8, 2.5, 1.8, 1.4, 1.1, 0.9]
var reverse_ratio: float = 4.0

func _ready() -> void:
	_init_wheels()
	_connect_signals()
	_initialize_audio()
	_setup_physics()
	_reset_vehicle_state()

func _process(_delta: float) -> void:
	_update_input_reading()
	_update_gear_logic()
	_update_wheel_states()
	_update_audio()

func _physics_process(delta: float) -> void:
	# Fixed timestep accumulator for consistent physics
	_delta_accumulator += delta
	var fixed_step: float = 1.0 / _physics_settings.physics_tick_rate
	
	while _delta_accumulator >= fixed_step:
		_apply_physics(fixed_step)
		_delta_accumulator -= fixed_step
	
	# Handle respawn if crashed
	if is_crashed:
		_handle_respawn(delta)

func _apply_physics(delta: float) -> void:
	# Calculate effective engine torque based on RPM and gear
	var engine_torque: float = _calculate_engine_torque()
	
	# Apply driving force through wheels
	_apply_driving_force(engine_torque, delta)
	
	# Apply braking force if needed
	_apply_braking_force(delta)
	
	# Apply steering input
	_apply_steering(delta)
	
	# Update velocity based on forces
	_update_velocity(delta)
	
	# Apply gravity
	_apply_gravity(delta)
	
	# Move character body
	move_and_slide()

func _calculate_engine_torque() -> float:
	"""Calculate torque output based on engine RPM, gear, and throttle input."""
	if !clutch_engaged or current_gear == 0:
		return 0.0
	
	# Get engine RPM
	var engine_rpm: float = powertrain.get_engine_rpm()
	
	# Calculate torque curve (simplified peak torque model)
	var peak_torque: float = 400.0  # Nm
	var peak_rpm: float = 4000.0
	var redline_rpm: float = 7000.0
	
	# Torque drops off after peak
	var rpm_factor: float = 1.0
	if engine_rpm > peak_rpm:
		rpm_factor = max(0.3, 1.0 - (engine_rpm - peak_rpm) / (redline_rpm - peak_rpm))
	elif engine_rpm < peak_rpm * 0.5:
		rpm_factor *= engine_rpm / (peak_rpm * 0.5)
	
	# Apply throttle input
	var throttle_factor: float = clamp(throttle_input, 0.0, 1.0)
	
	# Apply gear effect (lower gears have more torque multiplication)
	var gear_effect: float = gear_ratios[current_gear - 1] if current_gear > 0 else 1.0
	gear_effect = lerp(gear_effect, 1.0, throttle_factor)
	
	return engine_torque * rpm_factor * throttle_factor * gear_effect

func _apply_driving_force(torque: float, delta: float) -> void:
	"""Apply driving force to all driven wheels."""
	if !clutch_engaged:
		return
	
	# Determine which wheels are driven (FWD, RWD, AWD simplified)
	var driven_wheels: Array[int] = [2, 3]  # Rear wheel drive by default
	
	# Distribute torque to driven wheels
	for wheel_idx in driven_wheels:
		if wheel_idx < wheel_states.size():
			var wheel_force: float = torque * wheel_states[wheel_idx]["force_multiplier"]
			wheel_force *= final_drive_ratio
			
			# Apply force in direction of travel
			var forward_dir: Vector3 = chassis.global_transform.basis.z.normalized()
			force_vector = forward_dir * wheel_force
			
			# Apply slight slip resistance
			var wheel_velocity: float = wheel_states[wheel_idx]["current_velocity"]
			var slip_resistance: float = _calculate_wheel_slip(wheel_idx)
			
			# Adjust force based on slip
			wheel_force *= (1.0 - slip_resistance * 0.1)
			
			# Add to wheel state
			wheel_states[wheel_idx]["applied_force"] += wheel_force

func _apply_braking_force(delta: float) -> void:
	"""Apply braking force to all wheels."""
	if brake_input <= 0.0:
		return
	
	var braking_force_per_wheel: float = max_braking_force * brake_input
	var front_bias: float = brake_bias_front
	
	# Apply different forces to front vs rear wheels
	for i in range(WHEEL_COUNT):
		var is_front_wheel: bool = i < 2
		var bias: float = front_bias if is_front_wheel else (1.0 - front_bias)
		
		var actual_force: float = braking_force_per_wheel * bias
		
		# ABS modulation if enabled
		if abs_enabled:
			actual_force *= _abs_modulation(i)
		
		# Apply opposite to travel direction
		var backward_dir: Vector3 = -chassis.global_transform.basis.z.normalized()
		var brake_vector = backward_dir * actual_force
		
		wheel_states[i]["brake_force"] += brake_vector

func _abs_modulation(wheel_idx: int) -> float:
	"""ABS modulation factor based on wheel slip."""
	var slip_ratio: float = wheel_slip_ratio[wheel_idx]
	
	# Simple ABS logic - reduce brake force if slip is too high
	if abs(slip_ratio) > 0.2:
		return max(0.3, 1.0 - abs(slip_ratio) * 2.0)
	
	return 1.0

func _apply_steering(delta: float) -> void:
	"""Apply steering to front wheels."""
	if steering_input == 0.0:
		turning_angle = 0.0
		return
	
	# Smooth steering transition
	var target_angle: float = steering_input * max_steering_angle
	turning_angle = lerp(turning_angle, target_angle, steering_rate * delta)
	
	# Rotate front wheels
	for wheel_idx in [0, 1]:  # Front wheels
		var wheel_node: Node3D = chassis.get_node_or_null("WheelFront" + str(wheel_idx))
		if wheel_node:
			wheel_node.rotation.y = deg_to_rad(turning_angle)

func _update_velocity(delta: float) -> void:
	"""Update vehicle velocity based on applied forces."""
	# Calculate current speed magnitude
	current_speed = linear_interpolate(
		current_speed,
		velocity.length(),
		delta * 5.0
	)
	
	# Update signal
	speed_changed.emit(current_speed)

func _apply_gravity(delta: float) -> void:
	"""Apply gravitational force."""
	var gravity_force: Vector3 = Vector3.UP * _physics_settings.gravity * _physics_settings.default_vehicle_mass
	velocity += gravity_force * delta

func _init_wheels() -> void:
	"""Initialize wheel state arrays."""
	wheel_states.resize(WHEEL_COUNT)
	wheel_rpm.resize(WHEEL_COUNT)
	wheel_slip_ratio.resize(WHEEL_COUNT)
	
	for i in range(WHEEL_COUNT):
		wheel_states[i] = {
			"position": Vector3.ZERO,
			"radius": 0.33,
			"width": 0.2,
			"force_multiplier": 1.0 if i >= 2 else 0.0,  # RWD default
			"current_velocity": 0.0,
			"applied_force": 0.0,
			"brake_force": 0.0,
			"normal": Vector3.UP
		}
		wheel_rpm[i] = 0.0
		wheel_slip_ratio[i] = 0.0

func _connect_signals() -> void:
	"""Connect internal signals."""
	if powertrain:
		powertrain.rpm_changed.connect(_on_powertrain_rpm_changed)
		powertrain.torque_changed.connect(_on_powertrain_torque_changed)

func _initialize_audio() -> void:
	"""Initialize audio players for vehicle sounds."""
	engine_sound_player = AudioStreamPlayer3D.new()
	engine_sound_player.bus = "SFX"
	engine_sound_player.autoplay = false
	add_child(engine_sound_player)
	
	tire_squeal_player = AudioStreamPlayer3D.new()
	tire_squeal_player.bus = "SFX"
	tire_squeal_player.autoplay = false
	add_child(tire_squeal_player)
	
	crash_sound_player = AudioStreamPlayer3D.new()
	crash_sound_player.bus = "SFX"
	crash_sound_player.autoplay = false
	add_child(crash_sound_player)

func _setup_physics() -> void:
	"""Setup physics-based vehicle configuration."""
	# Set up mass and inertia
	var mass: float = _physics_settings.default_vehicle_mass
	chassis.mass = mass
	
	# Configure suspension damping
	var suspension_damping: float = 0.8
	var suspension_stiffness: float = 0.6
	
	# Initial gear setup
	current_gear = 1
	target_gear = 1
	target_max_speed = 120.0  # km/h base

func _reset_vehicle_state() -> void:
	"""Reset vehicle to initial state."""
	velocity = Vector3.ZERO
	angular_velocity = Vector3.ZERO
	current_speed = 0.0
	current_gear = 1
	is_crashed = false
	respawn_timer = 0.0
	clutch_engaged = true
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	vehicle_state_changed.emit(VehicleState.IDLE)

func _update_input_reading() -> void:
	"""Read and process input from InputManager singleton."""
	if not Engine.is_editor_hint():
		throttle_input = InputManager.get_action_strength("vehicle_throttle")
		brake_input = InputManager.get_action_strength("vehicle_brake")
		steering_input = InputManager.get_action_strength("vehicle_steering")
	else:
		# Editor preview fallback
		throttle_input = Input.is_action_pressed("ui_up") ? 1.0 : 0.0
		brake_input = Input.is_action_pressed("ui_down") ? 1.0 : 0.0
		steering_input = Input.get_axis("ui_left", "ui_right")

func _update_gear_logic() -> void:
	"""Handle automatic/manual gear shifting logic."""
	if current_gear != target_gear:
		_shift_gear(target_gear)
	
	# Automatic upshift logic
	if target_gear < max_gears and current_speed > 0:
		var rpm_threshold: float = powertrain.get_redline_rpm() * 0.85
		if powertrain.get_engine_rpm() > rpm_threshold:
			target_gear = min(max_gears, target_gear + 1)
	
	# Automatic downshift logic
	if target_gear > 0 and current_speed > 0:
		var rpm_min: float = powertrain.get_idle_rpm() * 1.2
		if powertrain.get_engine_rpm() < rpm_min:
			target_gear = max(1, target_gear - 1)

func _shift_gear(new_gear: int) -> void:
	"""Execute gear shift with clutch engagement."""
	var old_gear: int = current_gear
	current_gear = new_gear
	
	# Clutch disengage/reengage simulation
	clutch_engaged = false
	await get_tree().create_timer(0.15).timeout
	clutch_engaged = true
	
	gear_changed.emit(old_gear, new_gear)
	vehicle_state_changed.emit(VehicleState.DRIVING if new_gear > 0 else VehicleState.IDLE)

func _update_wheel_states() -> void:
	"""Update individual wheel states based on vehicle dynamics."""
	var vehicle_velocity: Vector3 = chassis.velocity
	
	for i in range(WHEEL_COUNT):
		# Calculate wheel rotation speed
		var wheel_radius: float = wheel_states[i].radius
		var wheel_linear_velocity: float = abs(vehicle_velocity.length())
		wheel_rpm[i] = (wheel_linear_velocity / wheel_radius) * 60.0 / (2.0 * PI)
		
		# Calculate slip ratio
		wheel_slip_ratio[i] = _calculate_wheel_slip(i)
		
		# Clamp values
		wheel_rpm[i] = max(0.0, min(wheel_rpm[i], powertrain.get_redline_rpm()))
		wheel_slip_ratio[i] = clamp(wheel_slip_ratio[i], -1.0, 1.0)
		
		# Signal slip detection
		if abs(wheel_slip_ratio[i]) > 0.15:
			wheel_slip_detected.emit(i, wheel_slip_ratio[i])

func _calculate_wheel_slip(wheel_idx: int) -> float:
	"""Calculate slip ratio for a specific wheel."""
	var wheel_velocity: float = wheel_states[wheel_idx]["current_velocity"]
	var ground_velocity: float = abs(chassis.velocity.length())
	
	if ground_velocity < 0.1:
		return 0.0
	
	var slip_ratio: float = (wheel_velocity - ground_velocity) / max(ground_velocity, 0.1)
	return slip_ratio

func _update_audio() -> void:
	"""Update audio parameters based on vehicle state."""
	if not engine_sound_player or not powertrain:
		return
	
	var engine_rpm: float = powertrain.get_engine_rpm()
	var redline: float = powertrain.get_redline_rpm()
	var idle: float = powertrain.get_idle_rpm()
	
	# Pitch based on RPM
	var pitch_scale: float = lerp(
		lerp(idle, 1.0, engine_rpm / redline),
		0.8,
		engine_rpm / redline
	)
	engine_sound_player.pitch_scale = pitch_scale
	
	# Volume based on throttle
	var volume: float = lerp(0.3, 1.0, throttle_input)
	engine_sound_player.volume_db = lerp(-10.0, 0.0, volume)
	
	# Tire squeal when slipping
	for slip in wheel_slip_ratio:
		if abs(slip) > 0.3:
			tire_squeal_player.volume_db = 0.0
			break
	else:
		tire_squeal_player.volume_db = -20.0

func _handle_respawn(delta: float) -> void:
	"""Handle vehicle respawn after crash."""
	respawn_timer += delta
	
	if respawn_timer >= 2.0:
		_reset_vehicle_state()
		velocity = Vector3.ZERO
		position = _get_spawn_position()
		respawn_timer = 0.0

func _get_spawn_position() -> Vector3:
	"""Get spawn position for crashed vehicle."""
	# Default to current position with slight offset
	return position + Vector3(0.0, 0.5, 0.0)

func set_gear(gear: int) -> void:
	"""Manually set gear (manual transmission mode)."""
	if gear < 0 or gear > max_gears:
		return
	
	target_gear = gear
	
	# For manual mode, also update current gear immediately with clutch
	clutch_engaged = false
	await get_tree().create_timer(0.1).timeout
	clutch_engaged = true
	current_gear = gear
	
	gear_changed.emit((gear if gear > 0 else 1), gear)

func set_throttle(value: float) -> void:
	"""Set throttle input value (-1.0 to 1.0)."""
	throttle_input = clamp(value, -1.0, 1.0)

func set_brake(value: float) -> void:
	"""Set brake input value (-1.0 to 1.0)."""
	brake_input = clamp(value, -1.0, 1.0)

func set_steering(value: float) -> void:
	"""Set steering input value (-1.0 to 1.0)."""
	steering_input = clamp(value, -1.0, 1.0)

func engage_clutch() -> void:
	"""Engage clutch for gear shifting."""
	clutch_engaged = true

func disengage_clutch() -> void:
	"""Disengage clutch for gear shifting."""
	clutch_engaged = false

func reset_gear() -> void:
	"""Reset to neutral gear."""
	current_gear = 0
	target_gear = 0
	clutch_engaged = false

func apply_handbrake(enabled: bool) -> void:
	"""Apply handbrake to rear wheels only."""
	if enabled:
		for i in range(2, 4):  # Rear wheels
			wheel_states[i]["handbrake_force"] = max_braking_force * 1.5
	else:
		for i in range(2, 4):
			wheel_states[i]["handbrake_force"] = 0.0

func detect_collision(impact: Vector3) -> void:
	"""Register collision event."""
	last_collision_time = Time.get_unix_time_from_system()
	collision_impact = impact
	
	var impact_magnitude: float = impact.length()
	
	if impact_magnitude > 15.0:
		is_crashed = true
		crash_detected.emit(impact_magnitude)
		vehicle_state_changed.emit(VehicleState.CRASHED)
		
		# Play crash sound
		if crash_sound_player:
			crash_sound_player.play()

func _on_powertrain_rpm_changed(rpm: float) -> void:
	"""Handle powertrain RPM changes."""
	pass  # Could trigger audio or telemetry updates

func _on_powertrain_torque_changed(torque: float) -> void:
	"""Handle powertrain torque changes."""
	pass  # Could trigger vibration or haptic feedback

func _notification(what: int) -> void:
	"""Handle node notifications."""
	match what:
		NOTIFICATION_ENTER_TREE:
			# Ensure physics settings are loaded
			if not _physics_settings:
				_physics_settings = ResourceLoader.load("res://scripts/core/PhysicsSettings.gd") as PhysicsSettings
		NOTIFICATION_EXIT_TREE:
			# Cleanup audio players
			if engine_sound_player:
				engine_sound_player.queue_free()
			if tire_squeal_player:
				tire_squeal_player.queue_free()
			if crash_sound_player:
				crash_sound_player.queue_free()

func _to_string() -> String:
	"""Return string representation of vehicle state."""
	return "VehicleController(speed=%.2f, gear=%d, state=%s)" % [
		current_speed, current_gear, vehicle_state
	]

func _debug_print_status() -> void:
	"""Print debug information about vehicle state."""
	print("[VehicleController]")
	print("  Speed: %.2f km/h" % (current_speed * 3.6))
	print("  Gear: %d/%d" % (current_gear, max_gears))
	print("  RPM: %.0f / %.0f" % [powertrain.get_engine_rpm(), powertrain.get_redline_rpm()])
	print("  Throttle: %.2f" % throttle_input)
	print("  Brake: %.2f" % brake_input)
	print("  Steering: %.2f" % steering_input)
	print("  State: %s" % vehicle_state)
</FILE "scripts/controllers/VehicleController.gd">>