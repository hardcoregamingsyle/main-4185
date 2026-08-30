extends CharacterBody2D
class_name VehicleController

## VehicleController - Core vehicle physics controller using PhysicsSettings constants
## Handles throttle, brake, steering inputs, wheel forces, and gear shifting logic
## Copyright 2026 Thalamus Racing Simulator Project

# Signal definitions
signal speed_changed(new_speed: float)
signal gear_changed(old_gear: int, new_gear: int)
signal crash_detected(impact_force: float)
signal lap_completed(lap_data: Dictionary)
signal checkpoint_passed(checkpoint_id: int)

# State enums
enum DrivingState { IDLE, ACCELERATING, BRAKING, CORNERING, SKIDDING, CRASHED }
enum GearState { NEUTRAL = 0, FIRST = 1, SECOND = 2, THIRD = 3, FOURTH = 4, FIFTH = 5, REVERSE = -1 }

# Physics constants from central settings
var _physics: PhysicsSettings

# Movement state variables
var current_speed: float = 0.0  # m/s
var target_speed: float = 0.0   # Desired speed from powertrain
var acceleration: float = 0.0   # Current acceleration value
var rotation_angle: float = 0.0 # Steering angle in radians
var gear: int = GearState.NEUTRAL
var driving_state: DrivingState = DrivingState.IDLE

# Input values (normalized -1 to 1)
var throttle_input: float = 0.0
var brake_input: float = 0.0
var steering_input: float = 0.0

# Wheel force application (for visual representation)
var front_left_wheel: Node2D
var front_right_wheel: Node2D
var rear_left_wheel: Node2D
var rear_right_wheel: Node2D

# Collision detection
var _collision_box: CollisionShape2D
var _wheel_colliders: Array[CollisionShape2D] = []

# Game feel effects
var _particle_system: GPUParticles2D
var _screen_shake_timer: float = 0.0
var _screen_shake_intensity: float = 0.0
var _hit_flash_timer: float = 0.0
var _damage_flash_color: Color = Color.RED

# Powertrain reference
var powertrain_script: Script = null

# Lap/timing data
var _lap_start_time: float = 0.0
var _current_lap: int = 0
var _checkpoints_passed: Array[int] = []

# Touch/mobile input tracking
var _touch_positions: Dictionary = {}
var _last_touch_time: float = 0.0

func _ready() -> void:
	# Initialize physics reference
	_physics = PhysicsSettings.new() if not Engine.has_singleton("PhysicsSettings") else PhysicsSettings
	
	# Connect to GameManager signals
	if GameManager:
		GameManager.game_state_changed.connect(_on_game_state_changed)
	
	# Setup collision box
	_collision_box = $CollisionShape2D if $has_node("CollisionShape2D") else null
	
	# Setup wheel references
	_setup_wheels()
	
	# Setup particle system
	_particle_system = $GPUParticles2D if $has_node("GPUParticles2D") else null
	
	# Initialize powertrain script
	_powertrain_init()
	
	# Set initial state
	_update_vehicle_state()
	
	# Process mode: always run even when paused
	process_mode = ProcessModeEnum.ALWAYS

func _physics_process(delta: float) -> void:
	# Prevent runaway physics
	if delta > 0.1:
		return
	
	# Update vehicle physics
	_handle_throttle_brake(delta)
	_handle_steering(delta)
	_apply_forces_and_movement(delta)
	_check_collision_and_crash(delta)
	_update_visual_effects(delta)
	_handle_lap_timing(delta)
	
	# Emit state changes
	_update_vehicle_state()

func _input(event: InputEvent) -> void:
	# Keyboard input handling
	if event.is_action_pressed("ui_up") or event.is_action_pressed("w"):
		throttle_input = clamp(throttle_input + 0.1, 0.0, 1.0)
	elif event.is_action_released("ui_up") or event.is_action_released("w"):
		throttle_input = max(throttle_input - 0.1, 0.0)
	
	if event.is_action_pressed("ui_down") or event.is_action_pressed("s"):
		brake_input = clamp(brake_input + 0.1, 0.0, 1.0)
	elif event.is_action_released("ui_down") or event.is_action_released("s"):
		brake_input = max(brake_input - 0.1, 0.0)
	
	if event.is_action_pressed("ui_left") or event.is_action_pressed("a"):
		steering_input = clamp(steering_input - 0.1, -1.0, 1.0)
	elif event.is_action_released("ui_left") or event.is_action_released("a"):
		steering_input = min(steering_input + 0.1, 1.0)
	
	if event.is_action_pressed("ui_right") or event.is_action_pressed("d"):
		steering_input = clamp(steering_input + 0.1, -1.0, 1.0)
	elif event.is_action_released("ui_right") or event.is_action_released("d"):
		steering_input = max(steering_input - 0.1, -1.0)
	
	# Gear shift inputs
	if event.is_action_pressed("gear_down"):
		_shift_gear(-1)
	elif event.is_action_pressed("gear_up"):
		_shift_gear(1)
	elif event.is_action_pressed("neutral"):
		gear = GearState.NEUTRAL
	
	# Touch/mouse input for mobile
	if event is InputEventMouseMotion or event is InputEventScreenTouch:
		_handle_touch_input(event, delta)

func _handle_throttle_brake(delta: float) -> void:
	var max_acceleration: float = _physics.default_vehicle_mass * 0.8  # m/s²
	var max_deceleration: float = _physics.default_vehicle_mass * 2.0  # Braking is stronger
	
	var desired_acceleration: float = 0.0
	
	# Calculate desired acceleration from inputs
	if throttle_input > 0.0:
		desired_acceleration = throttle_input * max_acceleration
		driving_state = DrivingState.ACCELERATING
	
	elif brake_input > 0.0:
		desired_acceleration = -brake_input * max_deceleration
		driving_state = DrivingState.BRAKING
	
	else:
		# Coasting - apply small friction
		desired_acceleration = -current_speed * 0.05
		if abs(current_speed) < 0.5:
			driving_state = DrivingState.IDLE
		else:
			driving_state = DrivingState.CORNERING
	
	# Apply acceleration to current speed
	current_speed += desired_acceleration * delta
	
	# Clamp speed within reasonable bounds
	var max_speed: float = _calculate_max_speed()
	current_speed = clamp(current_speed, -max_speed / 3.6, max_speed / 3.6)  # Convert km/h to m/s
	
	# Store for next frame
	target_speed = current_speed

func _handle_steering(delta: float) -> void:
	var max_steering_angle: float = deg_to_rad(45.0)  # Max 45 degrees
	var steering_speed: float = 3.0  # Radians per second
	
	# Smooth steering transition
	rotation_angle = lerp(rotation_angle, steering_input * max_steering_angle, delta * steering_speed)
	
	# Apply steering to body rotation
	var angular_velocity: float = rotation_angle * 10.0
	velocity.rotate(deg_to_rad(angular_velocity))
	
	# Cap maximum turn rate
	var max_turn_rate: float = deg_to_rad(90.0)
	velocity = velocity.normalized() * current_speed
	rotation = atan2(velocity.y, velocity.x)

func _apply_forces_and_movement(delta: float) -> void:
	# Apply movement using CharacterBody2D physics
	move_and_slide()
	
	# Update wheel positions visually
	_update_wheel_visuals()
	
	# Apply screen shake if needed
	if _screen_shake_timer > 0.0:
		position += Vector2(randf_range(-_screen_shake_intensity, _screen_shake_intensity), 
		                    randf_range(-_screen_shake_intensity, _screen_shake_intensity))
		_screen_shake_timer -= delta

func _check_collision_and_crash(delta: float) -> void:
	for collision in get_collision_with_nodes():
		if collision.has_method("is_obstacle"):
			var impact_force: float = abs(velocity.length()) * 0.5
			
			# Trigger crash effect
			_trigger_crash_effect(impact_force)
			
			# Reduce speed significantly on crash
			current_speed *= 0.3
			
			# Emit signal
			crash_detected.emit(impact_force)
			
			# Apply knockback
			var knockback_direction: Vector2 = (global_position - collision.global_position).normalized()
			velocity = knockback_direction * 2.0
			
			break

func _update_visual_effects(delta: float) -> void:
	# Hit flash effect
	if _hit_flash_timer > 0.0:
		_hit_flash_timer -= delta
		
		# Flash color briefly
		modulate = Color(1.0, 1.0, 1.0, 1.0) if _hit_flash_timer < 0.2 else Color.WHITE
	else:
		modulate = Color.WHITE
	
	# Particle effects for wheels
	if _particle_system and driving_state in [DrivingState.ACCELERATING, DrivingState.SKIDDING]:
		_particle_system.emitting = true
		_particle_system.amount = 20
		_particle_system.rate = 30
	else:
		_particle_system.emitting = false

func _handle_lap_timing(delta: float) -> void:
	# Simple lap timing system
	if GameManager.current_state == GameManager.GameState.RACE_ACTIVE:
		if _lap_start_time == 0.0:
			_lap_start_time = Time.get_ticks_msec()
		
		# Check checkpoints
		for i in range(get_child_count()):
			var node = get_child(i)
			if node.has_method("is_checkpoint"):
				if _is_within_range(node, 5.0):  # 5 unit radius
					if not _checkpoints_passed.has(i):
						_checkpoints_passed.append(i)
						checkpoint_passed.emit(i)
						
						# Check if lap completed (simple version)
						if len(_checkpoints_passed) >= 3:  # Assume 3 checkpoints per lap
							_complete_lap()

func _powertrain_init() -> void:
	# Load powertrain script reference
	var powertrain_path: String = "res://scripts/vehicles/Powertrain.gd"
	if FileAccess.file_exists(powertrain_path):
		powertrain_script = load(powertrain_path)
		if powertrain_script:
			# Add powertrain node if not present
			if not has_node("Powertrain"):
				var powertrain_node = Node2D.new()
				powertrain_node.name = "Powertrain"
				add_child(powertrain_node)
				powertrain_node.set_script(powertrain_script)

func _setup_wheels() -> void:
	# Create wheel nodes if they don't exist
	var wheel_names: Array[String] = ["FrontLeft", "FrontRight", "RearLeft", "RearRight"]
	
	for name in wheel_names:
		if not has_node(name):
			var wheel = Node2D.new()
			wheel.name = name
			
			# Wheel visual
			var wheel_shape = CircleShape2D.new()
			wheel_shape.radius = 12.0
			var wheel_collision = CollisionShape2D.new()
			wheel_collision.shape = wheel_shape
			
			var wheel_sprite = Sprite2D.new()
			wheel_sprite.texture = _create_wheel_texture()
			
			wheel.add_child(wheel_collision)
			wheel.add_child(wheel_sprite)
			add_child(wheel)
			
			match name:
				"FrontLeft":
					front_left_wheel = wheel
				"FrontRight":
					front_right_wheel = wheel
				"RearLeft":
					rear_left_wheel = wheel
				"RearRight":
					rear_right_wheel = wheel

func _create_wheel_texture() -> Texture2D:
	# Procedural wheel texture generation
	var canvas = CanvasTexture.new()
	canvas.size = Vector2i(64, 64)
	
	var canvas_item = CanvasItem.new()
	canvas_item.draw_circle(Vector2i(32, 32), 32, Color.DARK_GRAY)
	canvas_item.draw_circle(Vector2i(32, 32), 10, Color.GRAY)
	canvas_item.draw_circle(Vector2i(32, 32), 5, Color.WHITE)
	
	return canvas

func _update_wheel_visuals() -> void:
	# Position wheels around vehicle
	var offset: Vector2 = Vector2(40.0, 0.0)
	var height: float = 30.0
	
	if front_left_wheel:
		front_left_wheel.position = Vector2(-offset.x, -height)
	if front_right_wheel:
		front_right_wheel.position = Vector2(offset.x, -height)
	if rear_left_wheel:
		rear_left_wheel.position = Vector2(-offset.x, height)
	if rear_right_wheel:
		rear_right_wheel.position = Vector2(offset.x, height)
	
	# Rotate wheels based on speed
	var wheel_rotation: float = current_speed * 0.5
	
	if front_left_wheel:
		front_left_wheel.rotation = wheel_rotation
	if front_right_wheel:
		front_right_wheel.rotation = wheel_rotation
	if rear_left_wheel:
		rear_left_wheel.rotation = wheel_rotation
	if rear_right_wheel:
		rear_right_wheel.rotation = wheel_rotation

func _trigger_crash_effect(impact_force: float) -> void:
	# Screen shake
	_screen_shake_intensity = min(impact_force * 2.0, 20.0)
	_screen_shake_timer = 0.5
	
	# Hit flash
	_hit_flash_timer = 0.3
	
	# Spawn particles
	if _particle_system:
		_particle_system.restart()
	
	# Play sound (via AudioManager)
	if AudioManager:
		AudioManager.play_sound("crash", {"volume": 0.8})

func _shift_gear(direction: int) -> void:
	var old_gear: int = gear
	var new_gear: int = gear + direction
	
	# Validate gear shift
	if new_gear < GearState.NEUTRAL or new_gear > GearState.FIFTH:
		return
	
	if gear == GearState.REVERSE and new_gear != GearState.NEUTRAL:
		new_gear = GearState.NEUTRAL  # Cannot go directly from reverse to drive
	
	gear = new_gear
	gear_changed.emit(old_gear, new_gear)

func _calculate_max_speed() -> float:
	# Base speed varies by gear
	var base_speed: float = 60.0  # km/h
	
	match gear:
		GearState.FIRST:
			base_speed = 40.0
		GearState.SECOND:
			base_speed = 60.0
		GearState.THIRD:
			base_speed = 80.0
		GearState.FOURTH:
			base_speed = 100.0
		GearState.FIFTH:
			base_speed = 120.0
		GearState.REVERSE:
			base_speed = 30.0
	
	return base_speed

func _complete_lap() -> void:
	var lap_time: float = (Time.get_ticks_msec() - _lap_start_time) / 1000.0
	var lap_data: Dictionary = {
		"lap_number": _current_lap + 1,
		"time": lap_time,
		"timestamp": Time.get_datetime_string_from_unix_time(Time.get_unix_time_from_string())
	}
	
	lap_completed.emit(lap_data)
	
	# Reset for next lap
	_current_lap += 1
	_checkpoints_passed.clear()
	_lap_start_time = Time.get_ticks_msec()

func _is_within_range(reference: Node, distance: float) -> bool:
	var dist: float = global_position.distance_to(reference.global_position)
	return dist < distance

func _handle_touch_input(event: InputEvent, delta: float) -> void:
	# Simplified touch controls for mobile
	if event is InputEventScreenTouch:
		var touch: InputEventScreenTouch = event as InputEventScreenTouch
		
		if touch.pressed:
			_last_touch_time = Time.get_ticks_msec()
			
			# Left side of screen = steer left
			if touch.position.x < get_viewport_rect().size.x / 3:
				steering_input = -1.0
			# Right side = steer right
			elif touch.position.x > get_viewport_rect().size.x * 2 / 3:
				steering_input = 1.0
			# Center = neutral steering
			else:
				steering_input = 0.0
			
			# Bottom area = throttle
			if touch.position.y > get_viewport_rect().size.y * 2 / 3:
				throttle_input = 1.0
			else:
				throttle_input = 0.0

func _on_game_state_changed(new_state: GameState) -> void:
	match new_state:
		GameManager.GameState.RACE_ACTIVE:
			_lap_start_time = 0.0
			_current_lap = 0
			_checkpoints_passed.clear()
			driving_state = DrivingState.IDLE
			current_speed = 0.0
		GameManager.GameState.MAIN_MENU:
			# Stop all movement
			velocity = Vector2.ZERO
			current_speed = 0.0
			throttle_input = 0.0
			brake_input = 0.0
			steering_input = 0.0

func _update_vehicle_state() -> void:
	# Emit speed change signal if significant
	if abs(current_speed - last_emitted_speed) > 1.0:
		speed_changed.emit(current_speed)
		last_emitted_speed = current_speed

func reset_vehicle() -> void:
	"""Reset vehicle to initial state"""
	velocity = Vector2.ZERO
	current_speed = 0.0
	target_speed = 0.0
	gear = GearState.NEUTRAL
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	driving_state = DrivingState.IDLE
	_checkpoints_passed.clear()
	_lap_start_time = 0.0
	_screen_shake_timer = 0.0
	_hit_flash_timer = 0.0

func set_debug_mode(enabled: bool) -> void:
	"""Enable/disable debug mode for development"""
	debug_mode_enabled = enabled

# Public getter/setter methods
func get_current_speed_kmh() -> float:
	return current_speed * 3.6  # Convert m/s to km/h

func get_gear_name() -> String:
	match gear:
		GearState.NEUTRAL: return "N"
		GearState.FIRST: return "1"
		GearState.SECOND: return "2"
		GearState.THIRD: return "3"
		GearState.FOURTH: return "4"
		GearState.FIFTH: return "5"
		GearState.REVERSE: return "R"
		return "?"

func get_driving_state_name() -> String:
	match driving_state:
		DrivingState.IDLE: return "IDLE"
		DrivingState.ACCELERATING: return "ACCEL"
		DrivingState.BRAKING: return "BRAKE"
		DrivingState.CORNERING: return "TURN"
		DrivingState.SKIDDING: return "SLIDE"
		DrivingState.CRASHED: return "CRASH"
		return "?"

# Private variables for state tracking
var last_emitted_speed: float = 0.0
var debug_mode_enabled: bool = false

# Expose public properties for inspector editing
@export_group("Vehicle Configuration")
@export var max_steering_angle: float = 45.0
@export var acceleration_factor: float = 1.0
@export var braking_factor: float = 1.5
@export var friction_coefficient: float = 0.1

@export_group("Visual Effects")
@export var enable_screen_shake: bool = true
@export var enable_hit_flash: bool = true
@export var enable_particles: bool = true

@export_group("Debug Settings")
@export var show_debug_info: bool = false
@export var debug_text: Label = null

func _draw() -> void:
	"""Draw debug information if enabled"""
	if not show_debug_info:
		return
	
	# Draw current speed
	if debug_text:
		debug_text.text = "Speed: %.1f km/h\nGear: %s\nState: %s" % [
			get_current_speed_kmh(),
			get_gear_name(),
			get_driving_state_name()
		]
	
	# Draw steering indicator
	var steering_color: Color = Color.GREEN if steering_input != 0 else Color.GRAY
	draw_line(Vector2(0, -30), Vector2(0, -50), steering_color, 3.0)
	
	# Draw throttle indicator
	var throttle_color: Color = Color.GREEN if throttle_input > 0 else Color.GRAY
	draw_line(Vector2(0, -60), Vector2(0, -80), throttle_color, 3.0)
	
	# Draw brake indicator
	var brake_color: Color = Color.RED if brake_input > 0 else Color.GRAY
	draw_line(Vector2(0, -90), Vector2(0, -110), brake_color, 3.0)