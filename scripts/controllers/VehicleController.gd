extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller
## Handles throttle, brake, steering inputs, wheel forces, and gear shifting logic
## Uses PhysicsSettings resource for all physics constants
## Copyright 2026 Thalamus Racing Simulator Project

signal speed_changed(new_speed: float)
signal rpm_changed(new_rpm: float)
signal gear_changed(old_gear: int, new_gear: int)
signal drift_state_changed(is_drifting: bool)
signal collision_detected(vehicle_node: Node)

@export_group("Vehicle Properties")
@export var mass: float = 1500.0: set = _set_mass
@export var max_steering_angle: float = 45.0: set = _set_max_steering_angle
@export var steer_sensitivity: float = 1.0: set = _set_steer_sensitivity

@export_group("Powertrain Settings")
@export var engine_torque_curve: Array[Vector2] = []
@export var gear_ratios: Array[float] = []
@export var final_drive_ratio: float = 3.5: set = _set_final_drive_ratio
@export var wheel_radius: float = 0.3: set = _set_wheel_radius

@export_group("Drivetrain")
var drivetrain_type: DrivetrainType = DrivetrainType.FWD
enum DrivetrainType { FWD, RWD, AWD }

@export_group("AI Settings")
@export var ai_enabled: bool = false
@export var ai_target_speed: float = 0.0

var current_gear: int = 1
var target_gear: int = 1
var clutch_engaged: bool = true
var handbrake_active: bool = false

# Powertrain state
var engine_rpm: float = 0.0
var engine_max_rpm: float = 7000.0
var engine_min_rpm: float = 800.0
var torque_at_wheels: float = 0.0

# Speed states
var forward_speed: float = 0.0
var lateral_speed: float = 0.0
var vertical_speed: float = 0.0
var ground_speed: float = 0.0

# Input states
var throttle_input: float = 0.0
var brake_input: float = 0.0
var steering_input: float = 0.0

# Drift state
var drift_factor: float = 0.0
var is_drifting: bool = false
var drift_threshold: float = 0.5

# Wheel properties
var wheel_indices: Array[int] = [0, 1, 2, 3]
var wheel_contacts: Array[Dictionary] = []

# Physics references
var powertrain_node: Node = null
var chassis_node: Node = null
var raycaster_nodes: Array[RayCast3D] = []

func _ready() -> void:
	_init_physics_references()
	_setup_wheels()
	_connect_signals()
	_reset_vehicle_state()

func _init_physics_references() -> void:
	chassis_node = self
	
	if get_parent() != null:
		var parent = get_parent()
		if parent.has_method("get_powertrain"):
			powertrain_node = parent.get_powertrain()

func _setup_wheels() -> void:
	wheel_contacts.resize(wheel_indices.size())
	for i in wheel_indices.size():
		wheel_contacts[i] = {
			"contact_point": Vector3.ZERO,
			"normal": Vector3.UP,
			"force": Vector3.ZERO,
			"slip_ratio": 0.0,
			"slip_angle": 0.0,
			"driven": false,
			"braking": false
		}

func _connect_signals() -> void:
	if AudioManager:
		AudioManager.sound_played.connect(_on_sound_played)

func _reset_vehicle_state() -> void:
	current_gear = 1
	target_gear = 1
	clutch_engaged = true
	engine_rpm = engine_min_rpm
	forward_speed = 0.0
	lateral_speed = 0.0
	vertical_speed = 0.0
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	torque_at_wheels = 0.0
	is_drifting = false
	drift_factor = 0.0

func _physics_process(delta: float) -> void:
	if not is_inside_tree():
		return
	
	update_physics(delta)
	process_inputs(delta)
	calculate_forces(delta)
	apply_forces(delta)
	sync_visuals()

func update_physics(delta: float) -> void:
	"""Update vehicle physics state including velocity and position"""
	
	var old_velocity = velocity
	move_and_slide()
	
	forward_speed = velocity.x * sign(velocity.x) if velocity.x.abs() > 0.1 else 0.0
	lateral_speed = velocity.z
	vertical_speed = velocity.y
	
	ground_speed = sqrt(forward_speed * forward_speed + lateral_speed * lateral_speed)
	
	if abs(old_velocity.length() - velocity.length()) > 0.1:
		collision_detected.emit(self)

func process_inputs(delta: float) -> void:
	"""Process player/AI inputs and convert to control values"""
	
	if ai_enabled:
		_process_ai_inputs(delta)
	else:
		_process_player_inputs(delta)
	
	_apply_clutch_logic(delta)

func _process_ai_inputs(delta: float) -> void:
	"""AI-controlled input processing"""
	var target_speed_abs = abs(ai_target_speed)
	
	if target_speed_abs > 0.1:
		if ground_speed < target_speed_abs * 0.9:
			throttle_input = min(1.0, (target_speed_abs - ground_speed) / 50.0)
		elif ground_speed > target_speed_abs * 1.1:
			throttle_input = 0.0
			brake_input = min(1.0, (ground_speed - target_speed_abs) / 30.0)
		else:
			throttle_input = 0.0
			brake_input = 0.0
	else:
		throttle_input = 0.0
		brake_input = 0.0
	
	steering_input = 0.0
	# AI steering would be calculated based on track position

func _process_player_inputs(delta: float) -> void:
	"""Player-controlled input processing"""
	var input_manager = InputManager
	
	if input_manager:
		throttle_input = input_manager.get_axis("throttle", "brake")
		brake_input = input_manager.get_axis("reverse", "brake")
		steering_input = input_manager.get_axis("turn_left", "turn_right")
		
		if input_manager.is_action_pressed("handbrake"):
			handbrake_active = true
		else:
			handbrake_active = false
	else:
		throttle_input = Input.get_axis("w", "s")
		brake_input = Input.get_axis("a", "d")
		steering_input = Input.get_axis("left", "right")
		
		if Input.is_action_pressed("space"):
			handbrake_active = true
		else:
			handbrake_active = false

func _apply_clutch_logic(delta: float) -> void:
	"""Handle clutch engagement logic"""
	var target_rpm = _calculate_target_rpm()
	
	if clutch_engaged:
		engine_rpm = lerp(engine_rpm, target_rpm, delta * 10.0)
	else:
		engine_rpm = lerp(engine_rpm, engine_min_rpm, delta * 15.0)
	
	engine_rpm = clamp(engine_rpm, engine_min_rpm, engine_max_rpm)
	rpm_changed.emit(engine_rpm)

func _calculate_target_rpm() -> float:
	"""Calculate target RPM based on current speed and gear"""
	if ground_speed <= 0:
		return engine_min_rpm
	
	var gear_ratio = gear_ratios[current_gear - 1] if current_gear > 0 else 1.0
	var wheel_angular_speed = ground_speed / wheel_radius
	var target_rpm = wheel_angular_speed * gear_ratio * final_drive_ratio * 60.0 / (2.0 * PI)
	
	return target_rpm

func calculate_forces(delta: float) -> void:
	"""Calculate all forces acting on the vehicle"""
	
	torque_at_wheels = _calculate_engine_torque()
	var drag_force = _calculate_aerodynamic_drag()
	var rolling_resistance = _calculate_rolling_resistance()
	var gravity_component = _calculate_gravity_component()
	
	var total_drive_force = _apply_drivetrain_distribution(torque_at_wheels)
	
	var net_force_x = total_drive_force - drag_force - rolling_resistance + gravity_component
	var net_force_y = vertical_speed * PhysicsSettings.suspension_damping
	var net_force_z = lateral_speed * PhysicsSettings.tire_friction_coefficient
	
	acceleration = Vector3(net_force_x / mass, net_force_y / mass, net_force_z / mass)

func _calculate_engine_torque() -> float:
	"""Calculate torque output from engine based on RPM"""
	if engine_rpm < engine_min_rpm or engine_rpm > engine_max_rpm:
		return 0.0
	
	if engine_torque_curve.is_empty():
		var peak_torque = 400.0
		var peak_rpm = engine_max_rpm * 0.6
		return peak_torque * (1.0 - abs(engine_rpm - peak_rpm) / peak_rpm)
	
	for i in range(engine_torque_curve.size() - 1):
		var rpm_low = engine_torque_curve[i].x
		var rpm_high = engine_torque_curve[i + 1].x
		
		if engine_rpm >= rpm_low and engine_rpm <= rpm_high:
			var t_low = engine_torque_curve[i].y
			var t_high = engine_torque_curve[i + 1].y
			var t = t_low + (t_high - t_low) * ((engine_rpm - rpm_low) / (rpm_high - rpm_low))
			return t
	
	return 0.0

func _apply_drivetrain_distribution(torque: float) -> float:
	"""Apply torque distribution based on drivetrain type"""
	match drivetrain_type:
		DrivetrainType.FWD:
			return torque * 0.6
		DrivetrainType.RWD:
			return torque * 0.6
		DrivetrainType.AWD:
			return torque * 0.8
		_:
			return torque * 0.6

func _calculate_aerodynamic_drag() -> float:
	"""Calculate aerodynamic drag force"""
	var air_density = PhysicsSettings.air_density
	var drag_coefficient = PhysicsSettings.drag_coefficient
	var frontal_area = PhysicsSettings.frontal_area
	
	var v_squared = ground_speed * ground_speed
	return 0.5 * air_density * drag_coefficient * frontal_area * v_squared

func _calculate_rolling_resistance() -> float:
	"""Calculate rolling resistance force"""
	var rolling_resistance_coefficient = PhysicsSettings.rolling_resistance_coefficient
	return rolling_resistance_coefficient * mass * PhysicsSettings.gravity

func _calculate_gravity_component() -> float:
	"""Calculate gravity component along slope"""
	var slope_angle = atan2(vertical_speed, forward_speed)
	return mass * PhysicsSettings.gravity * sin(slope_angle)

func apply_forces(delta: float) -> void:
	"""Apply calculated forces to vehicle body"""
	
	if acceleration:
		var new_velocity = velocity + acceleration * delta
		new_velocity.y = clamp(new_velocity.y, -PhysicsSettings.suspension_travel * 2.0, 0.0)
		velocity = new_velocity
	
	force_applied.emit(acceleration * mass)

func sync_visuals() -> void:
	"""Sync visual elements with physics state"""
	speed_changed.emit(ground_speed)
	
	if is_drifting != _check_drift_condition():
		is_drifting = _check_drift_condition()
		drift_state_changed.emit(is_drifting)
	
	_update_wheel_visuals()

func _check_drift_condition() -> bool:
	"""Check if vehicle should enter drift mode"""
	if handbrake_active and abs(lateral_speed) > drift_threshold:
		drift_factor = lerp(drift_factor, 1.0, 0.1)
		return true
	elif abs(lateral_speed) < drift_threshold * 0.5:
		drift_factor = lerp(drift_factor, 0.0, 0.1)
		return false
	
	return false

func _update_wheel_visuals() -> void:
	"""Update individual wheel rotation and suspension visuals"""
	for i in wheel_contacts.size():
		var wheel_contact = wheel_contacts[i]
		if wheel_contact["driven"]:
			wheel_contact["rotation"] += ground_speed / wheel_radius

func shift_gear(target: int) -> void:
	"""Shift to specified gear with validation"""
	if target < 1 or target > gear_ratios.size():
		return
	
	if target == current_gress:
		return
	
	var old_gear = current_gear
	current_gear = target
	target_gear = target
	
	gear_changed.emit(old_gear, target)
	
	if AudioManager:
		AudioManager.play_sound("gear_shift")

func engage_clutch() -> void:
	"""Engage clutch temporarily"""
	clutch_engaged = false
	await get_tree().create_timer(0.1).timeout
	clutch_engaged = true

func reset_vehicle() -> void:
	"""Reset vehicle to starting state"""
	_reset_vehicle_state()
	velocity = Vector3.ZERO
	position = Vector3.ZERO
	rotate(Vector3.DOWN, 0.0)

func _set_mass(value: float) -> void:
	mass = value

func _set_max_steering_angle(value: float) -> void:
	max_steering_angle = value

func _set_steer_sensitivity(value: float) -> void:
	steer_sensitivity = value

func _set_final_drive_ratio(value: float) -> void:
	final_drive_ratio = value

func _set_wheel_radius(value: float) -> void:
	wheel_radius = value

func _on_sound_played(sound_name: String) -> void:
	pass

func _on_collision_entered(body: Node) -> void:
	collision_detected.emit(body)

func _notification(what: int) -> void:
	match what:
		NOTIFICATION_PHYSICS_PROCESS:
			# Additional physics hooks if needed
			pass
		NOTIFICATION_EXIT_TREE:
			# Cleanup if needed
			pass

</FILE BLOCK>