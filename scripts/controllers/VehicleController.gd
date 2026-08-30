extends Node3D
class_name VehicleController

## VehicleController - Core vehicle physics controller using PhysicsSettings constants
## Handles throttle, brake, steering inputs, wheel forces, gear shifting logic
## Copyright 2026 Thalamus Racing Simulator Project

# Signals for vehicle behavior events
signal acceleration_changed(throttle_value: float)
signal braking_changed(brake_value: float)
signal steering_changed(steering_angle: float)
signal gear_shifted(new_gear: int)
signal vehicle_speed_changed(speed: float)
signal traction_control_active(active: bool)

# Required child nodes (will be validated in _ready)
const REQUIRED_CHILDREN = ["Chassis", "Powertrain", "SuspensionSystem", "WheelFL", "WheelFR", "WheelBL", "WheelBR"]

# Physical properties (from PhysicsSettings)
@export_group("Vehicle Configuration")
@export var vehicle_mass: float = 1500.0
@export var center_of_mass_offset: Vector3 = Vector3(0, 0.3, 0.2)
@export var aerodynamic_drag_coefficient: float = 0.35
@export var frontal_area: float = 2.2
@export var max_engine_rpm: float = 8000.0
@export var idle_rpm: float = 800.0

# Input deadzones and sensitivity
@export_group("Input Settings")
@export var throttle_deadzone: float = 0.05
@export var brake_deadzone: float = 0.05
@export var steering_deadzone: float = 0.1
@export var steering_sensitivity: float = 1.0
@export var auto_shift: bool = true
@export var manual_override_enabled: bool = false

# Current state
var current_speed: float = 0.0
var current_rpm: float = 0.0
var current_gear: int = 0
var target_gear: int = 0
var throttle_input: float = 0.0
var brake_input: float = 0.0
var steering_input: float = 0.0

# Suspension state
var suspension_compression: Array[float] = []
var suspension_velocity: Array[float] = []

# Traction control state
var traction_control_enabled: bool = true
var wheel_slip_threshold: float = 0.25
var wheel_slip_state: Array[float] = []

# Aerodynamic forces
var downforce_coefficient: float = 0.5
var lift_coefficient: float = -0.2
var current_downforce: float = 0.0
var current_lift: float = 0.0

# Internal references
var _powertrain: Node = null
var _chassis: Node3D = null
var _wheels: Dictionary = {}
var _physics_settings: PhysicsSettings = null
var _last_update_time: float = 0.0

func _ready() -> void:
	_validate_children()
	_connect_signals()
	_init_wheel_references()
	_reset_vehicle_state()
	
	if Engine.is_editor_hint():
		return
	
	# Get physics settings reference
	_physics_settings = PhysicsSettings.new() if not PhysicsSettings.has_singleton() else get_node_or_null("/root/PhysicsSettings") as PhysicsSettings
	
	_process_mode = ProcessModeEnum.PROCESS_ALWAYS

func _validate_children() -> void:
	for child_name in REQUIRED_CHILDREN:
		var child = find_child(child_name, false, false)
		if child == null:
			push_warning("VehicleController: Missing required child node '%s'" % child_name)

func _connect_signals() -> void:
	# Connect to powertrain signals if available
	if has_node("Powertrain"):
		_powertrain = get_node("Powertrain")
		if _powertrain.has_signal("engine_rpm_changed"):
			_powertrain.engine_rpm_changed.connect(_on_engine_rpm_changed)
		if _powertrain.has_signal("gear_changed"):
			_powertrain.gear_changed.connect(_on_gear_changed)

func _init_wheel_references() -> void:
	# Reference all wheel nodes
	var wheel_names = ["WheelFL", "WheelFR", "WheelBL", "WheelBR"]
	for wheel_name in wheel_names:
		var wheel_node = find_child(wheel_name, false, false)
		if wheel_node != null:
			_wheels[wheel_name] = wheel_node
			suspension_compression.append(0.0)
			suspension_velocity.append(0.0)
			wheel_slip_state.append(0.0)

func _reset_vehicle_state() -> void:
	current_speed = 0.0
	current_rpm = idle_rpm
	current_gear = 1
	target_gear = 1
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	suspension_compression.fill(0.0)
	suspension_velocity.fill(0.0)
	wheel_slip_state.fill(0.0)

func _process(delta: float) -> void:
	if Engine.is_editor_hint():
		return
	
	# Update time tracking
	_last_update_time = Time.get_unix_time_from_seconds()
	
	# Read inputs
	_read_inputs(delta)
	
	# Calculate physics
	_update_rpm_and_gears(delta)
	_apply_aerodynamics(delta)
	_update_suspension(delta)
	_check_traction_control(delta)
	
	# Apply forces to wheels
	_apply_wheel_forces(delta)
	
	# Update visual rotation
	_update_wheel_visuals(delta)

func _read_inputs(delta: float) -> void:
	# Get input values from InputManager singleton
	var input_manager = get_node_or_null("/root/InputManager")
	if input_manager == null:
		return
	
	# Read throttle (positive = gas, negative = reverse)
	var raw_throttle = input_manager.get_axis("throttle_forward", "brake_reverse")
	throttle_input = _apply_deadzone(raw_throttle, throttle_deadzone)
	
	# Read brake (standalone brake input)
	var raw_brake = input_manager.get_axis("brake", "handbrake")
	brake_input = _apply_deadzone(raw_brake, brake_deadzone) * -1.0
	
	# Read steering (-1 left, 1 right)
	var raw_steering = input_manager.get_axis("steer_left", "steer_right")
	steering_input = _apply_deadzone(raw_steering, steering_deadzone) * steering_sensitivity
	
	# Clamp values
	throttle_input = clampf(throttle_input, -1.0, 1.0)
	brake_input = clampf(brake_input, -1.0, 1.0)
	steering_input = clampf(steering_input, -1.0, 1.0)
	
	# Emit signals
	emit_signal("acceleration_changed", throttle_input)
	emit_signal("braking_changed", brake_input)
	emit_signal("steering_changed", steering_input)

func _apply_deadzone(value: float, deadzone: float) -> float:
	if abs(value) < deadzone:
		return 0.0
	if value > 0:
		return (value - deadzone) / (1.0 - deadzone)
	else:
		return (value + deadzone) / (1.0 - deadzone)

func _update_rpm_and_gears(delta: float) -> void:
	if _powertrain == null:
		return
	
	# Calculate target RPM based on throttle and current speed
	var gear_ratio = _get_current_gear_ratio()
	var final_drive_ratio = _get_final_drive_ratio()
	var wheel_radius = _get_wheel_radius()
	
	# Calculate wheel RPM from current speed
	var wheel_rpm = (current_speed * 60.0) / (2.0 * PI * wheel_radius) if wheel_radius > 0 else 0.0
	var drivetrain_rpm = wheel_rpm * gear_ratio * final_drive_ratio
	
	# Calculate engine torque curve based on RPM
	var torque_curve = _calculate_torque_curve(current_rpm)
	var max_torque = _get_max_torque()
	var actual_torque = max_torque * torque_curve
	
	# Apply throttle to RPM change
	var rpm_change = 0.0
	if throttle_input > 0.1:
		rpm_change = (max_engine_rpm - idle_rpm) * throttle_input * delta * 0.5
	elif throttle_input < -0.1:
		rpm_change = -(max_engine_rpm - idle_rpm) * abs(throttle_input) * delta * 0.3
	else:
		# Idle or minimal throttle - slow return to idle RPM
		if current_rpm > idle_rpm:
			rpm_change = (idle_rpm - current_rpm) * delta * 2.0
		elif current_rpm < idle_rpm:
			rpm_change = (idle_rpm - current_rpm) * delta * 1.0
	
	# Brake affects RPM reduction
	if brake_input > 0.5:
		rpm_change -= (current_rpm - idle_rpm) * brake_input * delta * 3.0
	
	# Apply RPM changes
	current_rpm += rpm_change
	current_rpm = clampf(current_rpm, idle_rpm, max_engine_rpm)
	
	# Automatic gear shifting
	if auto_shift and not manual_override_enabled:
		_auto_shift_gears(delta)
	else:
		# Manual gear shift via input
		_handle_manual_gear_shift()
	
	# Update powertrain
	_powertrain.set_rpm(current_rpm)
	_powertrain.set_gear(current_gear)
	_powertrain.set_throttle(throttle_input)
	
	# Emit speed signal
	emit_signal("vehicle_speed_changed", current_speed)

func _auto_shift_gears(delta: float) -> void:
	var target_new_gear = 1
	
	# Simple automatic shifting logic based on RPM and speed
	if current_rpm >= max_engine_rpm * 0.9:
		# Upshift
		if current_gear < 6:
			target_new_gear = current_gear + 1
	elif current_rpm <= idle_rpm * 1.2 and current_speed > 1.0:
		# Downshift
		if current_gear > 1:
			target_new_gear = current_gear - 1
	elif current_rpm < idle_rpm and current_speed < 2.0:
		# Neutral at very low speeds
		target_new_gear = 0
	
	if target_new_gear != target_gear:
		target_gear = target_new_gear
		
		# Smooth transition
		if abs(target_gear - current_gear) == 1:
			_shift_gear(target_gear)

func _handle_manual_gear_shift() -> void:
	var input_manager = get_node_or_null("/root/InputManager")
	if input_manager == null:
		return
	
	# Check for upshift/downshift inputs
	var upshift_requested = input_manager.get_action_strength("upshift") > 0.5
	var downshift_requested = input_manager.get_action_strength("downshift") > 0.5
	
	if upshift_requested and current_gear < 6:
		_shift_gear(current_gear + 1)
	elif downshift_requested and current_gear > 1:
		_shift_gear(current_gear - 1)

func _shift_gear(new_gear: int) -> void:
	if new_gear == current_gear:
		return
	
	var old_gear = current_gear
	current_gear = new_gear
	
	# Brief clutch disengagement effect (simulated RPM drop)
	var rpm_drop = current_rpm * 0.3
	current_rpm = idle_rpm + (current_rpm - idle_rpm) * 0.7
	
	# Signal gear shift
	emit_signal("gear_shifted", current_gear)
	
	# Notify powertrain
	if _powertrain != null:
		_powertrain.shift_gear(new_gear)

func _get_current_gear_ratio() -> float:
	var gear_ratios = [0.0, 4.0, 2.5, 1.7, 1.3, 1.0, 0.8]  # Neutral, 1-6
	return gear_ratios[current_gear] if current_gear < gear_ratios.size() else 1.0

func _get_final_drive_ratio() -> float:
	return 3.73

func _get_wheel_radius() -> float:
	return 0.32

func _calculate_torque_curve(rpm: float) -> float:
	# Simplified torque curve (bell-shaped around peak torque RPM)
	var peak_torque_rpm = 4500.0
	var torque_peak = 0.85
	var curve_width = 2000.0
	
	# Gaussian-like curve
	var normalized_rpm = (rpm - peak_torque_rpm) / curve_width
	return torque_peak * exp(-normalized_rpm * normalized_rpm * 2.0)

func _get_max_torque() -> float:
	return 450.0  # Nm

func _apply_aerodynamics(delta: float) -> void:
	if chassis_transform == null:
		return
	
	# Calculate airspeed (vehicle speed relative to air)
	var airspeed = current_speed
	
	# Drag force: F_drag = 0.5 * rho * v^2 * Cd * A
	var air_density = 1.225  # kg/m^3 at sea level
	var drag_force = 0.5 * air_density * pow(airspeed, 2) * aerodynamic_drag_coefficient * frontal_area
	
	# Lift/downforce calculation
	var dynamic_pressure = 0.5 * air_density * pow(airspeed, 2)
	current_downforce = dynamic_pressure * downforce_coefficient * frontal_area
	current_lift = dynamic_pressure * lift_coefficient * frontal_area
	
	# Apply aerodynamic forces (downforce increases tire grip, lift reduces it)
	var total_vertical_force = (vehicle_mass * PhysicsSettings.gravity) - current_downforce + current_lift
	
	# Store for use in wheel calculations
	_set_vertical_load(total_vertical_force)

func _set_vertical_load(load: float) -> void:
	pass  # Can be overridden by specific implementations

func _update_suspension(delta: float) -> void:
	if _wheels.is_empty():
		return
	
	var spring_stiffness = 45000.0  # N/m
	var damping_coefficient = 3500.0  # Ns/m
	var rest_length = 0.35  # m
	
	for i in range(suspension_compression.size()):
		var wheel_name = ["WheelFL", "WheelFR", "WheelBL", "WheelBR"][i]
		var wheel_node = _wheels.get(wheel_name)
		
		if wheel_node == null:
			continue
		
		# Get wheel position relative to chassis
		var wheel_position = wheel_node.global_position
		var chassis_position = global_position
		
		# Calculate compression (distance from expected rest length)
		var current_distance = wheel_position.y - chassis_position.y
		var compression = rest_length - current_distance
		
		# Clamp compression to physical limits
		compression = clampf(compression, -0.1, 0.2)
		
		# Calculate velocity of compression
		var wheel_velocity = wheel_node.get_velocity().y
		var compression_velocity = wheel_velocity
		
		# Spring-damper force
		var spring_force = spring_stiffness * compression
		var damper_force = damping_coefficient * compression_velocity
		
		# Total suspension force
		var suspension_force = spring_force + damper_force
		
		# Apply force to wheel (simplified)
		if wheel_node.has_method("apply_suspension_force"):
			wheel_node.apply_suspension_force(suspension_force)
		
		# Update state
		suspension_compression[i] = compression
		suspension_velocity[i] = compression_velocity

func _check_traction_control(delta: float) -> void:
	if not traction_control_enabled:
		return
	
	for i in range(wheel_slip_state.size()):
		var wheel_name = ["WheelFL", "WheelFR", "WheelBL", "WheelBR"][i]
		var wheel_node = _wheels.get(wheel_name)
		
		if wheel_node == null:
			continue
		
		# Calculate wheel slip ratio
		var wheel_angular_velocity = wheel_node.get_angular_velocity()
		var wheel_linear_velocity = wheel_angular_velocity * _get_wheel_radius()
		
		# Slip ratio: (wheel_speed - vehicle_speed) / vehicle_speed
		var slip_ratio = 0.0
		if abs(current_speed) > 0.5:
			slip_ratio = (wheel_linear_velocity - current_speed) / abs(current_speed)
		else:
			slip_ratio = wheel_linear_velocity
		
		wheel_slip_state[i] = slip_ratio
		
		# Activate traction control if slip exceeds threshold
		if abs(slip_ratio) > wheel_slip_threshold:
			emit_signal("traction_control_active", true)
			_reduce_power_to_slipping_wheel(i, delta)
		else:
			emit_signal("traction_control_active", false)

func _reduce_power_to_slipping_wheel(wheel_index: int, delta: float) -> void:
	# Reduce throttle slightly when traction control is active
	var reduction_factor = 0.8
	throttle_input *= reduction_factor

func _apply_wheel_forces(delta: float) -> void:
	var gear_ratio = _get_current_gear_ratio()
	var final_drive_ratio = _get_final_drive_ratio()
	var wheel_radius = _get_wheel_radius()
	
	# Calculate engine output
	var torque = _get_max_torque() * _calculate_torque_curve(current_rpm)
	var drive_torque = torque * gear_ratio * final_drive_ratio
	
	# Distribute torque to driven wheels (RWD example)
	var wheel_names = ["WheelFL", "WheelFR", "WheelBL", "WheelBR"]
	var driven_wheels = ["WheelBL", "WheelBR"]
	
	for wheel_name in wheel_names:
		var wheel_node = _wheels.get(wheel_name)
		if wheel_node == null:
			continue
		
		var is_driven = driven_wheels.contains(wheel_name)
		
		# Calculate drive force
		var drive_force = 0.0
		if is_driven and throttle_input > 0:
			drive_force = (drive_torque * throttle_input) / wheel_radius
			drive_force = min(drive_force, _get_max_traction_force(wheel_name))
		
		# Calculate brake force
		var brake_force = 0.0
		if brake_input > 0:
			var max_brake_force = 15000.0  # Max brake capability
			brake_force = max_brake_force * brake_input
			brake_force = min(brake_force, _get_max_traction_force(wheel_name))
		
		# Apply forces to wheel
		if wheel_node.has_method("apply_drive_force"):
			wheel_node.apply_drive_force(drive_force, brake_force)

func _get_max_traction_force(wheel_name: String) -> float:
	var vertical_load = vehicle_mass * PhysicsSettings.gravity / 4.0
	var friction_coefficient = 1.05  # Racing tires
	return vertical_load * friction_coefficient

func _update_wheel_visuals(delta: float) -> void:
	for wheel_name in _wheels.keys():
		var wheel_node = _wheels.get(wheel_name)
		if wheel_node == null:
			continue
		
		# Rotate wheel based on angular velocity
		var wheel_angular_velocity = wheel_node.get_angular_velocity()
		var rotation_delta = wheel_angular_velocity * delta
		
		# Apply rotation around Y axis
		var current_rotation = wheel_node.rotation.y
		wheel_node.rotation.y = fmod(current_rotation + rotation_delta, TAU)
		
		# Steering animation for front wheels
		if wheel_name.begins_with("WheelF"):
			var steer_angle = steering_input * 0.4  # Max ~23 degrees
			wheel_node.rotation.z = steer_angle

func _on_engine_rpm_changed(new_rpm: float) -> void:
	current_rpm = new_rpm

func _on_gear_changed(new_gear: int) -> void:
	current_gear = new_gear

func get_speed() -> float:
	return current_speed

func get_rpm() -> float:
	return current_rpm

func get_gear() -> int:
	return current_gear

func get_throttle() -> float:
	return throttle_input

func get_brake() -> float:
	return brake_input

func get_steering() -> float:
	return steering_input

func set_traction_control(enabled: bool) -> void:
	traction_control_enabled = enabled

func reset_vehicle() -> void:
	_reset_vehicle_state()
	chassis_transform.origin = Vector3.ZERO
	chassis_transform.basis = Basis.IDENTITY
	set_velocity(Vector3.ZERO)
	setAngularVelocity(Vector3.ZERO)

func _get_chassis() -> Node3D:
	return _chassis

func _set_chassis(node: Node3D) -> void:
	_chassis = node

func _set_vertical_load(load: float) -> void:
	pass

</FILE "scripts/controllers/VehicleController.gd">