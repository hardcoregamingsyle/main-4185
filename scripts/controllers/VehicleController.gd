extends CharacterBody2D
class_name VehicleController

## VehicleController - Core vehicle physics controller using PhysicsSettings constants
## Handles throttle, brake, steering inputs, wheel forces, and gear shifting logic
## Copyright 2026 Thalamus Racing Simulator Project

signal speed_changed(current_speed: float)
signal rpm_changed(rpm: float)
signal gear_changed(old_gear: int, new_gear: int)
signal vehicle_destroyed(vehicle: Node2D)
signal lap_completed(lap_time: float)
signal checkpoint_passed(checkpoint_id: int)

# Constants loaded from PhysicsSettings singleton
var _physics: PhysicsSettings = PhysicsSettings.new()
const DEFAULT_ACCELERATION: float = 30000.0
const DEFAULT_BRAKING_FORCE: float = 50000.0
const MAX_STEERING_ANGLE: float = 0.5
const STEERING_SPEED: float = 15.0
const WHEEL_BASE: float = 2.5
const TRACK_WIDTH: float = 1.8
const FRICTION_COEFFICIENT: float = 0.98
const DRAG_COEFFICIENT: float = 0.45

# Vehicle state
var current_speed: float = 0.0
var max_speed: float = 0.0
var current_rpm: float = 0.0
var target_rpm: float = 0.0
var current_gear: int = 0
var engine_on: bool = false

# Powertrain properties
var powertrain_node: Node = null
var chassis_node: Node2D = null
var wheels: Array[Node2D] = []

# Input processing
var throttle_input: float = 0.0
var brake_input: float = 0.0
var steering_input: float = 0.0

# Gear ratios and RPM ranges
var gear_ratios: Array[float] = [3.8, 2.1, 1.4, 1.0, 0.8, 0.6]
var final_drive_ratio: float = 4.0
var idle_rpm: float = 800.0
var redline_rpm: float = 7000.0
var optimal_rpm_range: Range = Range(3500.0, 6000.0)

# Race statistics
var distance_traveled: float = 0.0
var lap_start_time: float = 0.0
var current_lap_time: float = 0.0
var best_lap_time: float = INF
var lap_count: int = 0

# Physics simulation variables
var velocity_vector: Vector2 = Vector2.ZERO
var acceleration_vector: Vector2 = Vector2.ZERO
var drag_force: Vector2 = Vector2.ZERO
var friction_force: Vector2 = Vector2.ZERO

func _ready() -> void:
	_init_vehicle_components()
	_connect_signals_to_systems()
	_load_configuration()
	
func _init_vehicle_components() -> void:
	"""Initialize vehicle component references from scene tree"""
	powertrain_node = get_node_or_null("../Powertrain") if has_node("../Powertrain") else null
	chassis_node = get_node_or_null("../Chassis") if has_node("../Chassis") else self
	
	# Find all wheel nodes (expecting child nodes named Wheel_Front_Left, Wheel_Front_Right, etc.)
	for child in get_children():
		if child is Node2D and child.name.contains("Wheel"):
			wheels.append(child)
			
func _connect_signals_to_systems() -> void:
	"""Connect internal signals to GameManager and AudioManager"""
	speed_changed.connect(_on_speed_changed)
	rpm_changed.connect(_on_rpm_changed)
	gear_changed.connect(_on_gear_changed)
	
func _load_configuration() -> void:
	"""Load configuration from resources or defaults"""
	max_speed = _physics.default_vehicle_max_speed
	current_gear = 1
	engine_on = true
	target_rpm = idle_rpm
	
func _process(delta: float) -> void:
	"""Process input and update vehicle state"""
	if not engine_on:
		return
		
	_process_inputs(delta)
	_update_physics(delta)
	_update_engine(delta)
	_update_gear_shifting(delta)
	_update_race_statistics(delta)
	
func _process_inputs(delta: float) -> void:
	"""Read and process player input for throttle, brake, steering"""
	throttle_input = InputManager.get_axis("throttle", "brake")
	brake_input = InputManager.get_axis("brake", "")
	steering_input = InputManager.get_axis("left", "right")
	
	# Clamp inputs to valid range
	throttle_input = clamp(throttle_input, -1.0, 1.0)
	brake_input = clamp(brake_input, -1.0, 1.0)
	steering_input = clamp(steering_input, -1.0, 1.0)
	
func _update_physics(delta: float) -> void:
	"""Update vehicle physics based on inputs and current state"""
	_calculate_acceleration()
	_apply_forces(delta)
	_move_and_collide()
	
func _calculate_acceleration() -> void:
	"""Calculate acceleration vector based on throttle/brake input and gear"""
	var traction_force: float = _get_traction_force()
	var braking_force: float = _get_braking_force()
	
	# Determine forward/backward force
	var drive_force: float = 0.0
	if throttle_input > 0.0:
		drive_force = throttle_input * traction_force
	elif brake_input > 0.0:
		drive_force = -brake_input * braking_force
	
	acceleration_vector.x = drive_force / _physics.default_vehicle_mass
	
func _get_traction_force() -> float:
	"""Calculate available traction force based on gear and RPM"""
	if current_gear <= 0:
		return 0.0
		
	var gear_ratio: float = gear_ratios[current_gear - 1]
	var engine_efficiency: float = _get_engine_efficiency()
	var max_torque: float = 400.0  # Nm typical for performance car
	
	var traction: float = max_torque * gear_ratio * final_drive_ratio * engine_efficiency
	return min(traction, DEFAULT_ACCELERATION)
	
func _get_braking_force() -> float:
	"""Calculate maximum braking force available"""
	return DEFAULT_BRAKING_FORCE
	
func _apply_forces(delta: float) -> void:
	"""Apply calculated forces to vehicle velocity"""
	# Apply acceleration
	velocity_vector += acceleration_vector * delta * 100.0
	
	# Calculate air drag
	var speed_magnitude: float = velocity_vector.length()
	drag_force = -velocity_vector.normalized() * speed_magnitude * speed_magnitude * drag_coefficient * delta
	
	# Apply drag
	velocity_vector += drag_force
	
	# Apply ground friction
	friction_force = -velocity_vector.normalized() * FRICTION_COEFFICIENT * delta
	velocity_vector += friction_force
	
	# Update speed from velocity magnitude
	current_speed = velocity_vector.length()
	
func _move_and_collide() -> void:
	"""Move vehicle and handle collisions"""
	move_and_slide(velocity_vector)
	
	# Update position-based properties
	distance_traveled += abs(velocity_vector.x) * delta
	
func _update_engine(delta: float) -> void:
	"""Update engine RPM based on vehicle speed and gear"""
	if current_gear == 0:
		target_rpm = lerp(target_rpm, idle_rpm, delta * 10.0)
		return
		
	var wheel_speed: float = current_speed / (WHEEL_BASE * PI * 0.3)  # Assume 0.3m tire radius
	var wheel_rpm: float = wheel_speed * 60.0
	var engine_rpm: float = wheel_rpm * gear_ratios[current_gear - 1] * final_drive_ratio
	
	target_rpm = engine_rpm
	
	# Smooth RPM transition
	current_rpm = lerp(current_rpm, target_rpm, delta * 5.0)
	
func _update_gear_shifting(delta: float) -> void:
	"""Handle automatic or manual gear shifting logic"""
	if current_gear == 0:
		return
		
	# Auto-shift logic
	if target_rpm >= redline_rpm and current_gear < gear_raties.size():
		_shift_up()
	elif target_rpm <= idle_rpm and current_gear > 1:
		_shift_down()
		
	# Manual override via input
	if InputManager.is_action_just_pressed("upshift"):
		_shift_up()
	if InputManager.is_action_just_pressed("downshift"):
		_shift_down()
		
func _shift_up() -> void:
	"""Shift transmission up one gear"""
	if current_gear >= gear_ratios.size():
		return
		
	var old_gear: int = current_gear
	current_gear += 1
	emit_signal("gear_changed", old_gear, current_gear)
	
func _shift_down() -> void:
	"""Shift transmission down one gear"""
	if current_gear <= 1:
		return
		
	var old_gear: int = current_gear
	current_gear -= 1
	emit_signal("gear_changed", old_gear, current_gear)
	
func _get_engine_efficiency() -> float:
	"""Calculate engine efficiency factor based on RPM relative to optimal range"""
	if optimal_rpm_range.contains(current_rpm):
		return 1.0
	elif current_rpm < optimal_rpm_range.min:
		return current_rpm / optimal_rpm_range.min
	else:
		return (redline_rpm - current_rpm) / (redline_rpm - optimal_rpm_range.max)
		
func _update_race_statistics(delta: float) -> void:
	"""Update race timing and lap data"""
	if GameManager.current_state != GameManager.GameState.RACE_ACTIVE:
		lap_start_time = 0.0
		return
		
	if lap_start_time == 0.0:
		lap_start_time = Time.get_ticks_msec()
		
	current_lap_time = (Time.get_ticks_msec() - lap_start_time) / 1000.0
	
	# Lap completion check (would be implemented by checkpoint system)
	if GameManager.checkpoint_system and GameManager.checkpoint_system.lap_complete:
		lap_count += 1
		best_lap_time = min(best_lap_time, current_lap_time)
		emit_signal("lap_completed", current_lap_time)
		lap_start_time = Time.get_ticks_msec()
		
func start_engine() -> void:
	"""Start the vehicle engine"""
	engine_on = true
	current_rpm = idle_rpm
	target_rpm = idle_rpm
	emit_signal("rpm_changed", current_rpm)
	
func stop_engine() -> void:
	"""Stop the vehicle engine"""
	engine_on = false
	current_rpm = 0.0
	target_rpm = 0.0
	emit_signal("rpm_changed", current_rpm)
	
func reset_vehicle() -> void:
	"""Reset vehicle to initial state"""
	current_speed = 0.0
	velocity_vector = Vector2.ZERO
	acceleration_vector = Vector2.ZERO
	current_gear = 1
	current_rpm = idle_rpm
	target_rpm = idle_rpm
	distance_traveled = 0.0
	lap_count = 0
	best_lap_time = INF
	lap_start_time = 0.0
	position = Vector2.ZERO
	
func set_gear(gear: int) -> void:
	"""Manually set transmission gear"""
	if gear < 0 or gear > gear_ratios.size():
		push_warning("Invalid gear requested: ", gear)
		return
		
	var old_gear: int = current_gear
	current_gear = gear
	emit_signal("gear_changed", old_gear, current_gear)
	
func set_throttle(amount: float) -> void:
	"""Set throttle input directly"""
	throttle_input = clamp(amount, 0.0, 1.0)
	
func set_brake(amount: float) -> void:
	"""Set brake input directly"""
	brake_input = clamp(amount, 0.0, 1.0)
	
func set_steering(angle: float) -> void:
	"""Set steering angle directly"""
	steering_input = clamp(angle, -1.0, 1.0)
	
func get_velocity() -> Vector2:
	"""Get current velocity vector"""
	return velocity_vector
	
func get_speed() -> float:
	"""Get current speed in km/h"""
	return current_speed * 3.6
	
func get_rpm() -> float:
	"""Get current engine RPM"""
	return current_rpm
	
func get_gear() -> int:
	"""Get current gear"""
	return current_gear
	
func is_moving() -> bool:
	"""Check if vehicle is in motion"""
	return abs(current_speed) > 0.1
	
func is_stalled() -> bool:
	"""Check if engine is stalled"""
	return not engine_on and current_rpm < 500.0
	
func _on_speed_changed(new_speed: float) -> void:
	"""Internal handler for speed changes"""
	pass
	
func _on_rpm_changed(new_rpm: float) -> void:
	"""Internal handler for RPM changes"""
	if AudioManager:
		AudioManager.play_sound("engine_rpm_change")
		
func _on_gear_changed(old_gear: int, new_gear: int) -> void:
	"""Internal handler for gear changes"""
	if AudioManager:
		AudioManager.play_sound("gear_change_" + str(new_gear))
		
func die() -> void:
	"""Handle vehicle destruction/crash"""
	emit_signal("vehicle_destroyed", self)
	stop_engine()
	velocity_vector = Vector2.ZERO
	current_rpm = 0.0
	
</VehicleController>