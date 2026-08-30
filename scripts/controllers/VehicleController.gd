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

# References
@onready var powertrain: Powertrain = $Powertrain if get_node_or_null("Powertrain") else null
@onready var chassis: Chassis = $Chassis if get_node_or_null("Chassis") else null
var _physics_body: CharacterBody2D = null

# State
enum VehicleState { IDLE, RUNNING, REVVING, BRAKING, COLLIDED, DRIFTING }
var current_vehicle_state: VehicleState = VehicleState.IDLE
var last_collision_time: float = 0.0
var collision_damping: float = 0.3

# Speed and movement
var current_speed: float = 0.0
var target_speed: float = 0.0
var acceleration: float = 0.0
var braking_force: float = 0.0
var steering_angle: float = 0.0
var total_distance_traveled: float = 0.0

# Gear management
var current_gear: int = 0  # 0 = neutral, 1-6 = forward gears, -1 = reverse
var rpm: float = 0.0
var max_rpm: float = 8000.0
var idle_rpm: float = 800.0

# Nitrous system
var nitro_available: bool = true
var nitro_amount: float = 100.0
var nitro_cooldown: float = 0.0
const NITRO_BONUS: float = 1.5
const NITRO_CONSUMPTION_RATE: float = 50.0

# Input handling
var throttle_input: float = 0.0
var brake_input: float = 0.0
var steering_input: float = 0.0
var shift_up_request: bool = false
var shift_down_request: bool = false

# Physics settings reference
var _settings: PhysicsSettings = null

# Constants
const MIN_SPEED_FOR_GEAR_CHANGE: float = 5.0
const GEAR_RATIOS: Array[float] = [0.0, 3.5, 2.8, 2.1, 1.6, 1.2, 0.9, -3.8]
const MAX_STEERING_ANGLE: float = 45.0 * deg_to_rad(1)
const STEERING_SENSITIVITY: float = 0.6
const ACCELERATION_RATE: float = 1500.0
const BRAKE_ACCELERATION: float = 3000.0
const FRICTION_COEFFICIENT: float = 0.85
const AIR_RESISTANCE: float = 0.02
const TORQUE_MULTIPLIER: float = 0.9
const CLUTCH engages: float = 0.3

func _init() -> void:
	"""Initialize physics controller with default values."""
	_settings = PhysicsSettings.new()
	current_speed = 0.0
	rpm = idle_rpm
	current_gear = 0
	nitro_available = true
	nitro_amount = 100.0
	nitro_cooldown = 0.0

func _ready() -> void:
	"""Setup vehicle controller when scene loads."""
	_settings = PhysicsSettings.get_singleton()
	_physics_body = _get_physics_body()
	if _physics_body == null:
		push_warning("VehicleController: No physics body found!")
	
	# Setup input connections
	_connect_inputs()
	
	# Initialize audio references
	if AudioManager != null:
		AudioManager.sound_played.connect(_on_sound_played)
	
	# Start processes
	set_process(true)
	set_physics_process(true)

func _get_physics_body() -> CharacterBody2D:
	"""Get the main physics body node for movement."""
	var candidates = ["RigidBody2D", "CharacterBody2D"]
	for candidate in candidates:
		var body = get_node_or_null(candidate)
		if body != null:
			return body
	return null

func _connect_inputs() -> void:
	"""Connect to global input manager."""
	if InputManager != null:
		InputManager.throttle_changed.connect(_on_throttle_changed)
		InputManager.brake_changed.connect(_on_brake_changed)
		InputManager.steering_changed.connect(_on_steering_changed)
		InputManager.shift_up.connect(_request_shift_up)
		InputManager.shift_down.connect(_request_shift_down)
		InputManager.nitro_toggled.connect(_toggle_nitro)

func _on_throttle_changed(value: float) -> void:
	throttle_input = clamp(value, -1.0, 1.0)

func _on_brake_changed(value: float) -> void:
	brake_input = clamp(value, 0.0, 1.0)

func _on_steering_changed(value: float) -> void:
	steering_input = clamp(value, -1.0, 1.0)

func _request_shift_up() -> void:
	shift_up_request = true

func _request_shift_down() -> void:
	shift_down_request = true

func _toggle_nitro() -> void:
	if nitro_available and nitro_amount > 0:
		activate_nitro()

func _process(delta: float) -> void:
	"""Update non-physics calculations."""
	_update_nitro_cooldown(delta)
	_handle_gear_shifting_requests()
	_update_vehicle_state()

func _physics_process(delta: float) -> void:
	"""Main physics update loop."""
	if _physics_body == null:
		return
	
	# Apply physics forces
	_apply_thrust(delta)
	_apply_steering(delta)
	_apply_friction_and_drag(delta)
	
	# Update velocity
	_update_velocity(delta)
	
	# Move vehicle
	_move_vehicle(delta)
	
	# Update telemetry
	_update_telemetry(delta)

func _apply_thrust(delta: float) -> void:
	"""Apply engine thrust based on throttle input and current gear."""
	if current_gear == 0:
		acceleration = 0.0
		return
	
	var gear_ratio: float = GEAR_RATIOS[current_gear]
	var base_torque: float = ACCELERATION_RATE * gear_ratio
	var torque_modifier: float = _calculate_torque_modifier()
	var actual_torque: float = base_torque * torque_modifier * TORQUE_MULTIPLIER
	
	# Apply throttle curve
	var effective_throttle: float = throttle_input * actual_torque
	
	# Apply nitro bonus if active
	if nitro_amount > 0 and throttle_input > 0.7:
		effective_throttle *= NITRO_BONUS
		nitro_amount -= delta * NITRO_CONSUMPTION_RATE
	
	acceleration = effective_throttle
	target_speed = calculate_max_speed_for_gear()
	
	# Clamp acceleration based on current state
	if current_vehicle_state == VehicleState.BRAKING:
		acceleration = min(acceleration, 0.0)

func _calculate_torque_modifier() -> float:
	"""Calculate torque modifier based on RPM and gear."""
	var rpm_ratio: float = rpm / max_rpm
	if rpm_ratio < 0.3:
		return 0.5
	elif rpm_ratio > 0.8:
		return 0.7
	else:
		return 1.0

func calculate_max_speed_for_gear() -> float:
	"""Calculate maximum speed achievable in current gear."""
	if current_gear == 0:
		return 0.0
	var gear_ratio: float = GEAR_RATIOS[current_gear]
	var max_engine_speed: float = max_rpm * 0.9
	var wheel_base_speed: float = 120.0 * gear_ratio
	return wheel_base_speed

func _apply_steering(delta: float) -> void:
	"""Apply steering input to vehicle orientation."""
	if abs(current_speed) < 1.0:
		steering_angle = lerp(steering_angle, 0.0, delta * 5.0)
		return
	
	var max_effective_angle: float = MAX_STEERING_ANGLE * (abs(current_speed) / 50.0)
	max_effective_angle = min(max_effective_angle, MAX_STEERING_ANGLE)
	
	var target_steering: float = steering_input * max_effective_angle
	steering_angle = lerp(steering_angle, target_steering, delta * STEERING_SENSITIVITY)
	
	# Rotate vehicle based on steering angle
	if _physics_body:
		_physics_body.rotation += steering_angle * delta * sign(current_speed)

func _apply_friction_and_drag(delta: float) -> void:
	"""Apply friction and air resistance to slow vehicle down."""
	if abs(current_speed) < 0.1:
		current_speed = 0.0
		return
	
	var friction_force: float = FRICTION_COEFFICIENT * current_speed * delta
	var drag_force: float = AIR_RESISTANCE * current_speed * current_speed * delta
	
	# Apply drag and friction
	current_speed -= friction_force + drag_force
	
	if current_speed < 0.0:
		current_speed = 0.0

func _update_velocity(delta: float) -> void:
	"""Update velocity based on acceleration and current speed."""
	var speed_change: float = acceleration * delta
	
	# Smooth acceleration
	if current_speed < target_speed:
		current_speed = min(current_speed + speed_change, target_speed)
	else:
		current_speed = max(current_speed - speed_change, target_speed)
	
	# Handle braking
	if brake_input > 0.1:
		var braking_change: float = BRAKE_ACCELERATION * brake_input * delta
		current_speed = max(current_speed - braking_change, 0.0)

func _move_vehicle(delta: float) -> void:
	"""Move the vehicle based on current velocity."""
	if _physics_body == null:
		return
	
	# Calculate movement direction
	var move_direction: Vector2 = Vector2.RIGHT.rotated(rotation)
	var movement_vector: Vector2 = move_direction * current_speed * delta
	
	# Apply movement
	_physics_body.position += movement_vector
	
	# Track distance traveled
	total_distance_traveled += abs(movement_vector.length())

func _update_telemetry(delta: float) -> void:
	"""Update vehicle telemetry and emit signals."""
	# Calculate RPM based on speed and gear
	if current_gear != 0:
		var gear_ratio: float = GEAR_RATIOS[current_gear]
		var calculated_rpm: float = (current_speed * 1000.0 * gear_ratio) / 50.0
		rpm = lerp(rpm, calculated_rpm, delta * 5.0)
		rpm = clamp(rpm, idle_rpm, max_rpm)
	else:
		rpm = lerp(rpm, idle_rpm, delta * 3.0)
	
	# Emit signals
	emit_signal("speed_changed", current_speed, target_speed)
	emit_signal("rpm_changed", rpm, max_rpm)
	
	if abs(total_distance_traveled - last_distance_traveled) > 1.0:
		emit_signal("vehicle_moved", abs(total_distance_traveled - last_distance_traveled))
	last_distance_traveled = total_distance_traveled

func _update_vehicle_state() -> void:
	"""Update current vehicle state based on conditions."""
	if current_speed < 0.1:
		current_vehicle_state = VehicleState.IDLE
	elif throttle_input > 0.8 and rpm > max_rpm * 0.9:
		current_vehicle_state = VehicleState.REVVING
	elif brake_input > 0.5:
		current_vehicle_state = VehicleState.BRAKING
	elif abs(steering_angle) > MAX_STEERING_ANGLE * 0.8:
		current_vehicle_state = VehicleState.DRIFTING
	else:
		current_vehicle_state = VehicleState.RUNNING

func _handle_gear_shifting_requests() -> void:
	"""Process pending gear shift requests."""
	if shift_up_request:
		shift_gear(1)
		shift_up_request = false
	elif shift_down_request:
		shift_gear(-1)
		shift_down_request = false

func shift_gear(direction: int) -> void:
	"""Shift transmission gear by direction (-1 or 1)."""
	var old_gear: int = current_gear
	var new_gear: int = current_gear + direction
	
	# Validate gear change
	if not _is_valid_gear_change(new_gear):
		return
	
	current_gear = new_gear
	
	# Handle neutral case
	if new_gear == 0:
		current_speed = 0.0
		rpm = idle_rpm
		emit_signal("engine_stopped")
	else:
		if old_gear == 0:
			emit_signal("engine_started")
		emit_signal("gear_changed", old_gear, new_gear)
		
		# Apply gear shift effect
		apply_gear_shift_effect()

func _is_valid_gear_change(gear: int) -> bool:
	"""Check if gear change is valid."""
	if gear == 0:
		return true
	
	if gear < -1 or gear > 6:
		return false
	
	# Require minimum speed for upshifting
	if gear > current_gear and abs(current_speed) < MIN_SPEED_FOR_GEAR_CHANGE:
		return false
	
	return true

func activate_nitro() -> void:
	"""Activate nitrous boost system."""
	if nitro_amount <= 0:
		return
	
	nitro_available = false
	nitro_cooldown = 5.0
	nitro_amount = 0.0
	
	emit_signal("nitro_used", 100.0)
	
	# Play sound effect
	if AudioManager != null:
		AudioManager.play_sfx("nitro_activate")

func _update_nitro_cooldown(delta: float) -> void:
	"""Update nitro cooldown timer."""
	if nitro_cooldown > 0:
		nitro_cooldown -= delta
		if nitro_cooldown <= 0:
			nitro_cooldown = 0.0
			nitro_available = true
			emit_signal("nitro_refilled")

func apply_gear_shift_effect() -> void:
	"""Apply visual/audio effects for gear shift."""
	# Screen shake effect
	if AudioManager != null:
		AudioManager.play_sfx("gear_shift")
	
	# Trigger particle burst at wheels
	_spawn_wheel_particles()

func _spawn_wheel_particles() -> void:
	"""Spawn particle effects at wheel positions."""
	if get_node_or_null("WheelFL") != null:
		spawn_particle_burst(get_node("WheelFL").position)
	if get_node_or_null("WheelFR") != null:
		spawn_particle_burst(get_node("WheelFR").position)
	if get_node_or_null("WheelBL") != null:
		spawn_particle_burst(get_node("WheelBL").position)
	if get_node_or_null("WheelBR") != null:
		spawn_particle_burst(get_node("WheelBR").position)

func spawn_particle_burst(position: Vector2) -> void:
	"""Create particle burst at given position."""
	var particle_system: CPUParticles2D = CPUParticles2D.new()
	particle_system.emitting = true
	particle_system.amount = 10
	particle_system.lifetime = 0.5
	particle_system.one_shot = true
	particle_system.color = Color.YELLOW
	particle_system.position = position
	
	add_child(particle_system)
	await get_tree().create_timer(0.5).timeout
	if is_instance_valid(particle_system):
		particle_system.queue_free()

func _on_sound_played(sound_name: String) -> void:
	"""Handle sound playback events."""
	match sound_name:
		"throttle":
			# Adjust pitch based on RPM
			pass
		"gear_shift":
			# Visual feedback only
			pass
		_:
			pass

func reset_vehicle() -> void:
	"""Reset vehicle to initial state."""
	current_speed = 0.0
	target_speed = 0.0
	acceleration = 0.0
	braking_force = 0.0
	steering_angle = 0.0
	total_distance_traveled = 0.0
	rpm = idle_rpm
	current_gear = 0
	nitro_available = true
	nitro_amount = 100.0
	nitro_cooldown = 0.0
	current_vehicle_state = VehicleState.IDLE
	last_collision_time = 0.0

func set_position(new_position: Vector2) -> void:
	"""Set vehicle position directly."""
	if _physics_body:
		_physics_body.position = new_position

func set_rotation(new_rotation: float) -> void:
	"""Set vehicle rotation directly."""
	if _physics_body:
		_physics_body.rotation = new_rotation

func get_speed_kmh() -> float:
	"""Convert internal speed to kilometers per hour."""
	return current_speed * 3.6

func get_speed_mph() -> float:
	"""Convert internal speed to miles per hour."""
	return current_speed * 2.237

func take_damage(damage_amount: float) -> void:
	"""Apply damage to vehicle."""
	last_collision_time = Time.get_unix_time_from_system()
	current_vehicle_state = VehicleState.COLLIDED
	
	# Reduce speed on impact
	current_speed *= 0.7
	
	# Apply damping
	collision_damping = 0.3
	
	# Emit signal
	emit_signal("collision_detected", _physics_body.velocity.normalized() if _physics_body else Vector2.ZERO)
	
	# Play crash sound
	if AudioManager != null:
		AudioManager.play_sfx("crash")
	
	# Return to running state after delay
	get_tree().create_timer(1.0).timeout.connect(func():
		if current_vehicle_state == VehicleState.COLLIDED:
			current_vehicle_state = VehicleState.RUNNING
	)

func _on_collided_with(other: Node) -> void:
	"""Handle collision with other objects."""
	take_damage(10.0)

func resume_simulation() -> void:
	"""Resume vehicle simulation after pause."""
	set_process(true)
	set_physics_process(true)

func pause_simulation() -> void:
	"""Pause vehicle simulation."""
	set_process(false)
	set_physics_process(false)

func _exit_tree() -> void:
	"""Cleanup when vehicle is destroyed."""
	if _physics_body:
		_physics_body = null
	powertrain = null
	chassis = null

func debug_print_stats() -> void:
	"""Print current vehicle statistics to console."""
	print("=== Vehicle Stats ===")
	print("Speed: %.2f km/h (%.2f mph)" % [get_speed_kmh(), get_speed_mph()])
	print("Gear: %d" % current_gear)
	print("RPM: %.0f / %.0f" % [rpm, max_rpm])
	print("Throttle: %.2f" % throttle_input)
	print("Brake: %.2f" % brake_input)
	print("Steering: %.2f" % steering_input)
	print("Nitro: %.0f%%" % [nitro_amount])
	print("Distance: %.0f m" % total_distance_traveled)
	print("=====================")
</FILE>