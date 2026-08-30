extends Node

class_name VehicleController

# Signals
signal engine_started
signal engine_stopped
signal gear_changed(old_gear: int, new_gear: int)
signal nitro_used(amount: float)
signal collision_detected(direction: Vector2)

# References
var powertrain: Powertrain
var chassis: Chassis
var physics_body: CharacterBody2D

# State
var current_vehicle_state: VehicleState = VehicleState.IDLE
var last_collision_time: float = 0.0
var collision_damping: float = 0.3

# Configuration
var acceleration_max: float = 10.0
var braking_max: float = 15.0
var steering_sensitivity: float = 0.08
var drift_threshold: float = 0.3

# Nitro cooldown tracking
var nitro_cooldown_remaining: float = 0.0

func _ready() -> void:
	current_vehicle_state = VehicleState.IDLE
	nitro_cooldown_remaining = 0.0

func _physics_process(delta: float) -> void:
	if current_vehicle_state != VehicleState.DRIVING:
		return
	
	# Update cooldowns
	if nitro_cooldown_remaining > 0.0:
		nitro_cooldown_remaining = max(0.0, nitro_cooldown_remaining - delta)
	
	# Get input
	var input_direction := Input.get_vector("ui_left", "ui_right", "ui_up", "ui_down")
	var throttle_input := input_direction.y
	var brake_input := Input.is_action_pressed("brake")
	var clutch_input := Input.is_action_pressed("clutch")
	var shift_up_input := Input.is_action_just_pressed("shift_up")
	var shift_down_input := Input.is_action_just_pressed("shift_down")
	var nitro_input := Input.is_action_just_pressed("nitro")
	var steer_input := input_direction.x
	
	# Handle nitro activation
	if nitro_input and powertrain.nitro_available > 0.0 and nitro_cooldown_remaining <= 0.0:
		powertrain.activate_nitro()
		nitro_cooldown_remaining = 15.0  # 15 second cooldown
		nitro_used.emit(powertrain.nitro_available * 0.5)
	
	# Apply steering
	if abs(steer_input) > 0.1:
		_apply_steering(steer_input, delta)
	
	# Apply throttle/braking
	_apply_drive(brake_input, throttle_input, delta)
	
	# Handle clutch
	if clutch_input:
		powertrain.disengage_clutch()
	else:
		powertrain.engage_clutch()
	
	# Handle shifting
	if shift_up_input:
		powertrain.shift_up()
	elif shift_down_input:
		powertrain.shift_down()
	
	# Check for launch conditions
	_check_launch_conditions()
	
	# Update physics body velocity
	_update_physics_velocity(delta)

func _apply_steering(input_value: float, delta: float) -> void:
	var current_rotation = physics_body.rotation
	
	# Smooth steering rotation
	var target_rotation = current_rotation + input_value * steering_sensitivity * delta
	
	# Apply limit to prevent over-rotation
	target_rotation = clamp(target_rotation, current_rotation - PI * 0.5, current_rotation + PI * 0.5)
	
	physics_body.rotation = lerp(current_rotation, target_rotation, 1.0 - exp(-delta * 10.0))

func _apply_drive(brake: bool, throttle: float, delta: float) -> void:
	if current_vehicle_state != VehicleState.DRIVING:
		return
	
	# Calculate desired acceleration
	var desired_acceleration = throttle * acceleration_max
	
	# Apply braking if requested
	if brake:
		desired_acceleration -= braking_max
	
	# Apply to physics body
	var current_velocity = physics_body.velocity
	var speed = current_velocity.length()
	
	if speed < 0.5:
		# Stationary - need to overcome inertia
		current_vehicle_state = VehicleState.IDLE
	
	# Calculate traction limits based on surface
	var max_traction = calculate_traction_limit()
	
	# Apply acceleration within traction limits
	var actual_accel = min(abs(desired_acceleration), max_traction)
	if desired_acceleration > 0:
		physics_body.velocity.x += actual_accel * delta
	else:
		physics_body.velocity.x -= actual_accel * delta
	
	# Clamp speed to realistic limits
	var max_speed = calculate_max_speed()
	if physics_body.velocity.length() > max_speed:
		physics_body.velocity = physics_body.velocity.normalized() * max_speed

func calculate_traction_limit() -> float:
	var base_traction = 8.0
	var surface_type = get_surface_type()
	
	match surface_type:
		"SURFACE_ASPHALT":
			return base_traction * 1.0
		"SURFACE_GRASS":
			return base_traction * 0.6
		"SURFACE_DIRT":
			return base_traction * 0.7
		"SURFACE_SNOW":
			return base_traction * 0.4
		_:
			return base_traction

func calculate_max_speed() -> float:
	var gear_ratios = powertrain.gear_ratios
	var final_drive = powertrain.final_drive_ratio
	
	var top_gear_index = min(gear_ratios.size() - 1, 5)
	var wheel_rpm_at_max_engine = powertrain.max_rpm / (gear_ratios[top_gear_index] * final_drive)
	
	var wheel_circumference = 2.0 * PI * chassis.wheel_radius
	var max_speed_mps = wheel_rpm_at_max_engine * wheel_circumference
	
	return max_speed_mps * 3.6  # Convert to km/h

func _update_physics_velocity(delta: float) -> void:
	if current_vehicle_state == VehicleState.CRASHED:
		# Dampen velocity after crash
		physics_body.velocity = physics_body.velocity.move_toward(Vector2.ZERO, delta * 2.0)
		return
	
	# Apply drag
	var drag_coefficient = 0.01
	physics_body.velocity *= (1.0 - drag_coefficient * delta)

func _check_launch_conditions() -> void:
	if powertrain.engine_rpm > idle_rpm * 1.5 and current_vehicle_state == VehicleState.IDLE:
		current_vehicle_state = VehicleState.DRIVING

func handle_collision(collision_data: CollisionShape2D) -> void:
	var now = Time.get_ticks_msec()
	
	if now - last_collision_time < 500:
		return  # Debounce collisions
	
	last_collision_time = now
	current_vehicle_state = VehicleState.CRASHED
	
	collision_damping = 0.3
	
	# Damage vehicle
	powertrain.output_torque_nm = 0.0
	engine_stopped.emit()
	collision_detected.emit(Vector2.RIGHT)

func reset_after_collision() -> void:
	current_vehicle_state = VehicleState.IDLE
	physics_body.velocity = Vector2.ZERO
	physics_body.rotation = 0.0
	powertrain.force_idle()
	engine_started.emit()

func start_engine() -> void:
	powertrain.start_engine()
	current_vehicle_state = VehicleState.RUNNING
	engine_started.emit()

func stop_engine() -> void:
	powertrain.stop_engine()
	current_vehicle_state = VehicleState.IDLE
	engine_stopped.emit()

func reset_vehicle() -> void:
	current_vehicle_state = VehicleState.IDLE
	nitro_cooldown_remaining = 0.0
	reset_after_collision()
