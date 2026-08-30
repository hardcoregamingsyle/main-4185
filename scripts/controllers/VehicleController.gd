extends Node3D
class_name VehicleController

## VehicleController - Core vehicle physics controller using PhysicsSettings constants
## Handles throttle, brake, steering inputs, wheel forces, gear shifting logic
## Copyright 2026 Thalamus Racing Simulator Project

signal engine_started
signal engine_stopped
signal gear_changed(old_gear: int, new_gear: int)
signal nitro_used(amount: float)
signal collision_detected(collision_info: Dictionary)
signal vehicle_accelerating(speed: float)
signal vehicle_decelerating(speed: float)

@export_group("Vehicle Configuration")
@export var vehicle_mass: float = 1500.0
@export var center_of_mass_offset: Vector3 = Vector3(0.0, 0.5, 0.0)
@export var max_steering_angle: float = 45.0 * TAU / 360.0

@export_group("Powertrain Settings")
@export var engine_max_rpm: float = 8000.0
@export var engine_idle_rpm: float = 800.0
@export var engine_peak_power_rpm: float = 5500.0
@export var max_engine_torque: float = 450.0
@export var transmission_gears: Array[int] = [1, 2, 3, 4, 5, 6, 'R']
@export var gear_ratios: Array[float] = [3.5, 2.2, 1.5, 1.1, 0.9, 0.7, 3.8]
@export var differential_ratio: float = 4.1
@export var final_drive_ratio: float = 1.0

@export_group("Brake System")
@export var max_brake_force: float = 12000.0
@export var brake_bias_front: float = 0.6
@export var brake_pressure_build_rate: float = 50.0

@export_group("Tire & Suspension")
@export var tire_friction_coefficient: float = 1.2
@export var suspension_stiffness: float = 50000.0
@export var suspension_damping: float = 3000.0
@export var suspension_travel: float = 0.2

@export_group("Aerodynamics")
@export var drag_coefficient: float = 0.35
@export var frontal_area: float = 2.2
@export var downforce_coefficient: float = 0.5
@export var downforce_at_speed: float = 100.0

@export_group("Nitrous System")
@export var nitro_available: bool = true
@export var nitro_capacity: float = 100.0
@export var nitro_consumption_rate: float = 2.0
@export var nitro_boost_multiplier: float = 1.5
var _current_nitro: float = 0.0

# State tracking
var current_gear: int = 0
var current_rpm: float = 0.0
var is_engine_running: bool = false
var is_in_reverse: bool = false
var steering_angle: float = 0.0
var acceleration_input: float = 0.0
var braking_input: float = 0.0
var steering_input: float = 0.0

# References (set via script or inspector)
var rigid_body_3d: RigidBody3D = null
var powertrain_node: Node = null
var wheel_nodes: Array[Node3D] = []

# Internal state
var _target_gear: int = 0
var _engine_torque: float = 0.0
var _wheel_torque: float = 0.0
var _brake_pressures: Array[float] = [0.0, 0.0, 0.0, 0.0]
var _speed_kmh: float = 0.0
var _acceleration: float = 0.0
var _last_update_time: float = 0.0

# Physics reference
var _physics_settings: PhysicsSettings = PhysicsSettings.new()

func _ready() -> void:
	_init_references()
	_connect_signals()
	_reset_state()

func _init_references() -> void:
	rigid_body_3d = get_parent() as RigidBody3D
	
	var powertrain_script = load("res://scripts/vehicles/Powertrain.gd")
	if powertrain_script != null:
		for child in get_children():
			if child.has_method("get_engine_state"):
				powertrain_node = child
	
	_find_wheels()
	
	_current_nitro = nitro_capacity

func _find_wheels() -> void:
	wheel_nodes.clear()
	for child in get_children():
		if child.name.to_lower().contains("wheel") or \
		   child.name.to_lower().contains("tire") or \
		   child is RayCast3D:
			wheel_nodes.append(child)

func _connect_signals() -> void:
	if powertrain_node != null:
		powertrain_node.engine_started.connect(_on_engine_started)
		powertrain_node.engine_stopped.connect(_on_engine_stopped)

func _reset_state() -> void:
	current_gear = 1
	_target_gear = 1
	current_rpm = engine_idle_rpm
	is_engine_running = false
	is_in_reverse = false
	steering_angle = 0.0
	_engine_torque = 0.0
	_wheel_torque = 0.0
	_speed_kmh = 0.0
	_acceleration = 0.0
	_last_update_time = Time.get_ticks_msec() / 1000.0
	
	for i in range(4):
		_brake_pressures[i] = 0.0

func _process(delta: float) -> void:
	_handle_inputs(delta)
	_update_physics(delta)
	_update_display_values()

func _physics_process(delta: float) -> void:
	if not Engine.is_editor_hint() and rigid_body_3d != null:
		_apply_forces(delta)

func _handle_inputs(delta: float) -> void:
	var input_manager = InputManager if Engine.get_singleton("InputManager") else null
	if input_manager == null:
		return
	
	# Get normalized inputs from InputManager
	acceleration_input = input_manager.get_axis("throttle", "brake")
	braking_input = input_manager.get_axis("brake", "reverse")
	steering_input = input_manager.get_axis("steer_left", "steer_right")
	
	# Clamp inputs to valid ranges
	acceleration_input = clamp(acceleration_input, -1.0, 1.0)
	braking_input = clamp(braking_input, -1.0, 1.0)
	steering_input = clamp(steering_input, -1.0, 1.0)
	
	# Apply steering with deadzone
	if abs(steering_input) > 0.1:
		steering_angle = lerp(steering_angle, 
			steering_input * max_steering_angle, delta * 10.0)
	else:
		steering_angle = lerp(steering_angle, 0.0, delta * 10.0)
	
	# Handle gear shifting
	_handle_gear_shifting()
	
	# Handle nitrous
	_handle_nitrous_input()
	
	# Start/stop engine
	_handle_engine_start_stop()

func _update_physics(delta: float) -> void:
	if not is_engine_running:
		return
	
	var dt = delta
	if dt > 0.1:
		dt = 0.016  # Cap delta time for stability
	
	# Calculate target RPM based on gear and speed
	var target_rpm = _calculate_target_rpm()
	current_rpm = lerp(current_rpm, target_rpm, dt * 5.0)
	
	# Update torque delivery
	_update_torque_delivery(dt)
	
	# Update wheel speeds
	_update_wheel_states(dt)

func _calculate_target_rpm() -> float:
	var wheel_radius = 0.3  # Default tire radius
	var effective_ratio = gear_ratios[current_gear] * differential_ratio
	
	if wheel_nodes.size() > 0:
		var front_wheel = wheel_nodes[0]
		if front_wheel is RigidBody3D:
			var linear_velocity = front_wheel.linear_velocity
			var wheel_speed = linear_velocity.length()
			
			if wheel_speed > 0.1:
				return (wheel_speed * effective_ratio) / wheel_radius
			elif abs(wheel_speed) > 0.1:
				return (abs(wheel_speed) * effective_ratio) / wheel_radius
	
	return engine_idle_rpm

func _update_torque_delivery(delta: float) -> void:
	var gear_ratio = gear_ratios[current_gear]
	var total_ratio = gear_ratio * differential_ratio
	
	# Calculate desired engine torque based on input
	var desired_torque = 0.0
	
	if acceleration_input > 0.0:
		var throttle_factor = acceleration_input
		var rpm_factor = _get_rpm_power_curve_factor()
		desired_torque = max_engine_torque * throttle_factor * rpm_factor
		
		if nitro_available and _current_nitro > 0:
			desired_torque *= nitro_boost_multiplier
	
	elif braking_input > 0.0:
		desired_torque = -max_engine_torque * braking_input * 0.3
	
	# Apply torque limits based on current RPM
	if current_rpm < engine_idle_rpm:
		desired_torque = max(desired_torque, 0.0)
	
	if current_rpm >= engine_max_rpm:
		desired_torque = min(desired_torque, 0.0)
	
	_engine_torque = desired_torque
	
	# Calculate wheel torque
	_wheel_torque = _engine_torque * total_ratio * 0.95  # Transmission efficiency
	
	# Apply to powertrain if available
	if powertrain_node != null and powertrain_node.has_method("apply_torque"):
		powertrain_node.apply_torque(_engine_torque)

func _get_rpm_power_curve_factor() -> float:
	if current_rpm <= engine_idle_rpm:
		return 0.0
	elif current_rpm >= engine_peak_power_rpm:
		return 1.0
	else:
		# Linear interpolation between idle and peak power
		return (current_rpm - engine_idle_rpm) / (engine_peak_power_rpm - engine_idle_rpm)

func _update_wheel_states(delta: float) -> void:
	for wheel in wheel_nodes:
		if wheel is Area3D:
			var contact_impulse = wheel.contact_impulse if wheel.has_method("get_contact_impulse") else Vector3.ZERO
			if contact_impulse.length() > 0:
				# Update wheel rotation based on vehicle speed
				pass

func _apply_forces(delta: float) -> void:
	if not is_engine_running or rigid_body_3d == null:
		return
	
	var dt = delta
	
	# Calculate aerodynamic forces
	_apply_aerodynamics(delta)
	
	# Apply drivetrain forces to wheels
	_apply_drivetrain_forces(delta)
	
	# Apply braking forces
	_apply_braking_forces(delta)
	
	# Calculate and report acceleration
	_calculate_acceleration()

func _apply_aerodynamics(delta: float) -> void:
	if rigid_body_3d == null:
		return
	
	var velocity = rigid_body_3d.linear_velocity
	var speed = velocity.length()
	
	if speed < 1.0:
		return
	
	# Air density
	var air_density = 1.225
	
	# Drag force: Fd = 0.5 * Cd * A * rho * v^2
	var drag_force = 0.5 * drag_coefficient * frontal_area * air_density * speed * speed
	
	# Apply drag opposite to velocity direction
	if speed > 0:
		var drag_direction = -velocity.normalized()
		var drag_vector = drag_direction * drag_force
		rigid_body_3d.apply_central_force(drag_vector)
	
	# Downforce (if positive coefficient)
	if downforce_coefficient > 0:
		var downforce = downforce_coefficient * air_density * speed * speed
		var downforce_vector = Vector3(0, -downforce, 0)
		rigid_body_3d.apply_central_force(downforce_vector)

func _apply_drivetrain_forces(delta: float) -> void:
	if rigid_body_3d == null or wheel_nodes.is_empty():
		return
	
	var vehicle_forward = rigid_body_3d.transform.basis.z
	var wheel_positions: Array[Vector3] = []
	var wheel_types: Array[String] = ["front_left", "front_right", "rear_left", "rear_right"]
	
	# Get wheel positions relative to vehicle
	for i in range(wheel_nodes.size()):
		if i < wheel_nodes.size():
			wheel_positions.append(wheel_nodes[i].global_position - rigid_body_3d.global_position)
	
	# Distribute drive torque to wheels
	var drive_wheels_count = 2  # Assume rear-wheel drive
	var torque_per_wheel = _wheel_torque / drive_wheels_count
	
	for i in range(drive_wheels_count):
		if i < wheel_nodes.size():
			var wheel = wheel_nodes[i]
			var wheel_local_pos = wheel.global_position - rigid_body_3d.global_position
			
			# Apply forward/reverse force at wheel position
			var force_direction = vehicle_forward if current_gear >= 0 else -vehicle_forward
			var force_magnitude = torque_per_wheel / 0.3  # Divide by approximate wheel radius
			
			if current_rpm > engine_idle_rpm:
				var force_vector = force_direction * force_magnitude
				rigid_body_3d.apply_force(force_vector, wheel.global_position)

func _apply_braking_forces(delta: float) -> void:
	if rigid_body_3d == null or wheel_nodes.is_empty():
		return
	
	var speed = rigid_body_3d.linear_velocity.length()
	if speed < 0.5:
		return
	
	# Build brake pressure based on input
	var brake_pressure = 0.0
	if braking_input > 0.0:
		brake_pressure = braking_input * max_brake_force
	
	# Distribute brake force between axles
	var front_brake_force = brake_pressure * brake_bias_front
	var rear_brake_force = brake_pressure * (1.0 - brake_bias_front)
	
	# Apply braking to each wheel
	for i in range(wheel_nodes.size()):
		var wheel = wheel_nodes[i]
		var wheel_local_pos = wheel.global_position - rigid_body_3d.global_position
		
		var wheel_brake_force = front_brake_force if i < 2 else rear_brake_force
		
		if wheel_brake_force > 0:
			var brake_vector = -rigid_body_3d.linear_velocity.normalized() * wheel_brake_force
			rigid_body_3d.apply_force(brake_vector, wheel.global_position)

func _calculate_acceleration() -> void:
	if rigid_body_3d == null:
		return
	
	var velocity = rigid_body_3d.linear_velocity
	var current_speed_kmh = velocity.length() * 3.6
	
	var delta_time = Time.get_ticks_msec() / 1000.0 - _last_update_time
	if delta_time > 0.016:
		delta_time = 0.016
	
	if _last_update_time > 0:
		var prev_speed_kmh = _speed_kmh
		_acceleration = (current_speed_kmh - prev_speed_kmh) / delta_time
		
		if _acceleration > 0.1:
			vehicle_accelerating.emit(current_speed_kmh)
		elif _acceleration < -0.1:
			vehicle_decelerating.emit(current_speed_kmh)
	
	_speed_kmh = current_speed_kmh
	_last_update_time = Time.get_ticks_msec() / 1000.0

func _handle_gear_shifting() -> void:
	var input_manager = InputManager if Engine.get_singleton("InputManager") else null
	if input_manager == null:
		return
	
	# Check for upshift/downshift inputs
	var upshift_requested = input_manager.is_action_pressed("gear_up")
	var downshift_requested = input_manager.is_action_pressed("gear_down")
	
	if upshift_requested and _can_shift_up():
		_shift_gear_up()
	elif downshift_requested and _can_shift_down():
		_shift_gear_down()

func _can_shift_up() -> bool:
	if current_gear >= transmission_gears.size() - 1:
		return false
	if current_rpm < engine_idle_rpm + 500.0:
		return false
	return true

func _can_shift_down() -> bool:
	if current_gear <= 0:
		return false
	if current_rpm > engine_max_rpm - 1000.0:
		return false
	return true

func _shift_gear_up() -> void:
	var old_gear = current_gear
	current_gear += 1
	_target_gear = current_gear
	gear_changed.emit(old_gear, current_gear)

func _shift_gear_down() -> void:
	var old_gear = current_gear
	current_gear -= 1
	_target_gear = current_gear
	gear_changed.emit(old_gear, current_gear)

func _handle_nitrous_input() -> void:
	if not nitro_available or _current_nitro <= 0:
		return
	
	var input_manager = InputManager if Engine.get_singleton("InputManager") else null
	if input_manager == null:
		return
	
	if input_manager.is_action_pressed("nitrous"):
		_use_nitrous()

func _use_nitrous() -> void:
	if _current_nitro <= 0:
		return
	
	_current_nitro -= nitro_consumption_rate * 0.1
	nitro_used.emit(nitro_consumption_rate * 0.1)
	
	if _current_nitro <= 0:
		_current_nitro = 0.0

func _handle_engine_start_stop() -> void:
	var input_manager = InputManager if Engine.get_singleton("InputManager") else null
	if input_manager == null:
		return
	
	if input_manager.is_action_pressed("ignite_engine") and not is_engine_running:
		start_engine()
	elif input_manager.is_action_pressed("kill_engine") and is_engine_running:
		stop_engine()

func start_engine() -> void:
	if is_engine_running:
		return
	
	is_engine_running = true
	current_rpm = engine_idle_rpm
	_reset_state()
	engine_started.emit()
	
	if powertrain_node != null and powertrain_node.has_method("start_engine"):
		powertrain_node.start_engine()

func stop_engine() -> void:
	if not is_engine_running:
		return
	
	is_engine_running = false
	current_rpm = 0.0
	_engine_torque = 0.0
	_wheel_torque = 0.0
	engine_stopped.emit()
	
	if powertrain_node != null and powertrain_node.has_method("stop_engine"):
		powertrain_node.stop_engine()

func set_gear(gear_index: int) -> void:
	if gear_index < 0 or gear_index >= transmission_gears.size():
		return
	
	current_gear = gear_index
	_target_gear = gear_index
	gear_changed.emit(-1, current_gear)

func shift_to_reverse() -> void:
	if current_gear == 0:  # Neutral
		set_gear(transmission_gears.size() - 1)  # Reverse gear
		is_in_reverse = true
	else:
		# Shift through gears to reverse
		while current_gear > 0:
			_shift_gear_down()

func shift_to_neutral() -> void:
	if current_gear != 0:
		var old_gear = current_gear
		current_gear = 0
		_target_gear = 0
		gear_changed.emit(old_gear, current_gear)
		is_in_reverse = false

func reset_vehicle() -> void:
	stop_engine()
	_reset_state()
	
	if rigid_body_3d != null:
		rigid_body_3d.linear_velocity = Vector3.ZERO
		rigid_body_3d.angular_velocity = Vector3.ZERO

func _update_display_values() -> void:
	# Update any display nodes or HUD elements here
	pass

func get_vehicle_status() -> Dictionary:
	return {
		"is_engine_running": is_engine_running,
		"current_gear": current_gear,
		"current_rpm": current_rpm,
		"engine_torque": _engine_torque,
		"wheel_torque": _wheel_torque,
		"speed_kmh": _speed_kmh,
		"acceleration": _acceleration,
		"nitro_available": nitro_available,
		"nitro_level": _current_nitro,
		"brake_pressures": _brake_pressures,
		"steering_angle": steering_angle
	}

func _on_engine_started() -> void:
	is_engine_running = true
	engine_started.emit()

func _on_engine_stopped() -> void:
	is_engine_running = false
	engine_stopped.emit()

func apply_collision_force(collision_point: Vector3, impact_velocity: float) -> void:
	if rigid_body_3d == null:
		return
	
	var collision_info: Dictionary = {
		"collision_point": collision_point,
		"impact_velocity": impact_velocity,
		"time_stamp": Time.get_ticks_msec()
	}
	
	collision_detected.emit(collision_info)
	
	# Apply impulse at collision point
	var impact_force = impact_velocity * vehicle_mass * 0.5
	var force_vector = -rigid_body_3d.linear_velocity.normalized() * impact_force
	rigid_body_3d.apply_impulse(impact_force, collision_point)

func set_suspension_stiffness(stiffness: float) -> void:
	suspension_stiffness = stiffness

func set_suspension_damping(damping: float) -> void:
	suspension_damping = damping

func set_tire_friction(friction: float) -> void:
	tire_friction_coefficient = friction

func calculate_top_speed() -> float:
	var wheel_radius = 0.3
	var top_gear_ratio = gear_raties[-2] if len(gear_ratios) > 1 else 0.7
	var final_ratio = top_gear_ratio * differential_ratio
	
	# Theoretical top speed based on max RPM and gear ratio
	var max_wheel_rps = engine_max_rpm / 60.0
	var wheel_circumference = 2.0 * PI * wheel_radius
	var top_speed_ms = max_wheel_rps * wheel_circumference / final_ratio
	
	return top_speed_ms * 3.6  # Convert to km/h

func _get_rpm_power_curve_factor() -> float:
	if current_rpm <= engine_idle_rpm:
		return 0.0
	elif current_rpm >= engine_peak_power_rpm:
		return 1.0
	else:
		return (current_rpm - engine_idle_rpm) / (engine_peak_power_rpm - engine_idle_rpm)
</FILE_BLOCK>