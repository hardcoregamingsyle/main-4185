extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulation
## Handles throttle, brake, steering, wheel forces, gear shifting, and vehicle dynamics
## Integrates with PhysicsSettings for centralized tuning
## Copyright 2026 Thalamus Racing Simulator Project

signal speed_changed(current_speed: float)
signal rpm_changed(current_rpm: float)
signal gear_changed(new_gear: int)
signal drift_started()
signal drift_ended()
signal collision_impact(impact_force: float, impact_point: Vector3)
signal wheel_slip(wheel_index: int, slip_ratio: float)

# ============================================================================
# PHYSICS SETTINGS REFERENCE
# ============================================================================

@onready var _physics = PhysicsSettings.get_singleton()

# ============================================================================
# VEHICLE CONFIGURATION
# ============================================================================

@export_group("Vehicle Configuration")
@export var vehicle_mass: float = 1500.0
@export var center_of_mass_offset: Vector3 = Vector3(0.0, 0.5, 0.0)
@export var wheel_base: float = 2.8
@export var track_width: float = 1.8
@export var ground_clearance: float = 0.25
@export var drag_coefficient: float = 0.30
@export var frontal_area: float = 2.2
@export var roll_stiffness_front: float = 12000.0
@export var roll_stiffness_rear: float = 10000.0
@export var camber_angle_front: float = -0.5
@export var camber_angle_rear: float = -0.5
@export var toe_angle_front: float = 0.02
@export var toe_angle_rear: float = 0.02

@export_group("Powertrain Parameters")
@export var engine_max_rpm: float = 7500.0
@export var engine_min_rpm: float = 800.0
@export var idle_rpm: float = 900.0
@export var max_torque: float = 450.0
@export var torque_curve_points: Array[Vector2] = []
@export var gear_ratios: Array[float] = [3.8, 2.2, 1.5, 1.1, 0.9, 0.75, 0.6]
@export var final_drive_ratio: float = 3.5
@export var transmission_efficiency: float = 0.95
@export var clutch_engagement_rpm: float = 1200.0
@export var rev_matching_enabled: bool = true

@export_group("Braking System")
@export var brake_pressure_front: float = 4.0
@export var brake_pressure_rear: float = 3.5
@export var brake_disc_diameter: float = 0.32
@export var brake_pad_friction: float = 0.4
@export var brake_caliper_piston_area: float = 0.005
@export var abs_enabled: bool = true
@export var abs_threshold: float = 0.15
@export var brake_balance_front: float = 0.6
@export var brake_temperature_max: float = 900.0

@export_group("Tire Properties")
@export var tire_stiffness_factor: float = 1.0
@export var tire_friction_static: float = 1.2
@export var tire_friction_dynamic: float = 0.9
@export var tire_width: float = 0.25
@export var tire_radius: float = 0.32
@export var tire_camber_stiffness: float = 5000.0
@export var tire_toe_effectiveness: float = 0.3
@export var tire_temperature_optimal: float = 80.0
@export var tire_temperature_max: float = 150.0

@export_group("Aerodynamics")
@export var downforce_coefficient: float = 0.5
@export var lift_coefficient: float = -0.1
@export var aero_reference_length: float = 2.5
@export var aero_reference_width: float = 1.8
@export var wing_angle_front: float = 0.0
@export var wing_angle_rear: float = 1.0
@export var aero_debug: bool = false

@export_group("Drift & Handling")
@export var drift_enabled: bool = true
@export var drift_threshold: float = 0.8
@export var drift_recovery_rate: float = 0.1
@export var drift_stability: float = 0.7
@export var traction_control_enabled: bool = true
@export var traction_control_threshold: float = 0.2
@export var stability_control_enabled: bool = true
@export var lateral_acceleration_limit: float = 1.5

# ============================================================================
# INTERNAL STATE
# ============================================================================

var _current_gear: int = 0
var _target_gear: int = 0
var _engine_rpm: float = 0.0
var _vehicle_speed: float = 0.0
var _throttle_input: float = 0.0
var _brake_input: float = 0.0
var _steering_input: float = 0.0
var _clutch_input: float = 0.0
var _handbrake: bool = false
var _is_drifting: bool = false
var _slip_angle: Vector3 = Vector3.ZERO
var _wheel_rotation_angles: Array[float] = [0.0, 0.0, 0.0, 0.0]
var _wheel_rotation_speeds: Array[float] = [0.0, 0.0, 0.0, 0.0]
var _brake_temperatures: Array[float] = [0.0, 0.0, 0.0, 0.0]
var _tire_temperatures: Array[float] = [0.0, 0.0, 0.0, 0.0]
var _vertical_forces: Array[float] = [0.0, 0.0, 0.0, 0.0]
var _longitudinal_forces: Array[float] = [0.0, 0.0, 0.0, 0.0]
var _lateral_forces: Array[float] = [0.0, 0.0, 0.0, 0.0]
var _suspension_compression: Array[float] = [0.0, 0.0, 0.0, 0.0]
var _suspension_velocity: Array[float] = [0.0, 0.0, 0.0, 0.0]
var _fuel_level: float = 100.0
var _fuel_consumption_rate: float = 0.0
var _distance_traveled: float = 0.0
var _last_update_time: float = 0.0
var _acceleration_history: Array[float] = []
var _angular_momentum: Vector3 = Vector3.ZERO
var _roll_angle: float = 0.0
var _pitch_angle: float = 0.0
var _yaw_rate: float = 0.0

# Wheel positions (relative to vehicle center)
const WHEEL_FFL := 0
const WHEEL_FR := 1
const WHEEL_RL := 2
const WHEEL_RR := 3

func _ready() -> void:
	_process_mode = ProcessModeEnum.ALWAYS
	_init_wheel_positions()
	_init_torque_curve()
	_reset_state()
	print("VehicleController initialized successfully")

func _init_wheel_positions() -> void:
	# Front-left, front-right, rear-left, rear-right
	var half_track = track_width * 0.5
	var half_wheelbase = wheel_base * 0.5
	
	# Store wheel world position offsets
	_wheel_positions = [
		Vector3(-half_wheelbase, -ground_clearance, -half_track),  # FL
		Vector3(-half_wheelbase, -ground_clearance, half_track),   # FR
		Vector3(half_wheelbase, -ground_clearance, -half_track),   # RL
		Vector3(half_wheelbase, -ground_clearance, half_track)     # RR
	]

func _init_torque_curve() -> void:
	if torque_curve_points.is_empty():
		torque_curve_points = [
			Vector2(0.0, 0.0),
			Vector2(0.3, 0.6),
			Vector2(0.5, 0.9),
			Vector2(0.7, 1.0),
			Vector2(0.9, 0.95),
			Vector2(1.0, 0.85)
		]

func _reset_state() -> void:
	_current_gear = 0
	_target_gear = 0
	_engine_rpm = idle_rpm
	_vehicle_speed = 0.0
	_throttle_input = 0.0
	_brake_input = 0.0
	_steering_input = 0.0
	_clutch_input = 0.0
	_handbrake = false
	_is_drifting = false
	_fuel_level = 100.0
	_distance_traveled = 0.0
	_roll_angle = 0.0
	_pitch_angle = 0.0
	_yaw_rate = 0.0
	
	for i in range(4):
		_wheel_rotation_angles[i] = 0.0
		_wheel_rotation_speeds[i] = 0.0
		_brake_temperatures[i] = 20.0
		_tire_temperatures[i] = 20.0
		_vertical_forces[i] = 0.0
		_longitudinal_forces[i] = 0.0
		_lateral_forces[i] = 0.0
		_suspension_compression[i] = 0.0
		_suspension_velocity[i] = 0.0
	
	_acceleration_history.clear()

func _process(delta: float) -> void:
	if delta <= 0.0:
		return
	
	_last_update_time += delta
	_update_physics(delta)
	_handle_inputs()
	_update_powertrain(delta)
	_update_suspension(delta)
	_update_aerodynamics(delta)
	_update_drift(delta)
	_check_gear_shifts(delta)
	_update_fuel(delta)
	_update_signals(delta)

func _physics_process(delta: float) -> void:
	if delta <= 0.0:
		return
	
	_apply_physics(delta)
	_update_collision_detection(delta)
	_cleanup_acceleration_history()

func _handle_inputs() -> void:
	# Get input values from InputManager
	_throttle_input = InputManager.get_action_strength("vehicle_throttle")
	_brake_input = InputManager.get_action_strength("vehicle_brake")
	_steering_input = InputManager.get_action_strength("vehicle_steering")
	_clutch_input = InputManager.get_action_strength("vehicle_clutch")
	_handbrake = InputManager.is_action_pressed("vehicle_handbrake")
	
	# Clamp inputs
	_throttle_input = clamp(_throttle_input, 0.0, 1.0)
	_brake_input = clamp(_brake_input, 0.0, 1.0)
	_steering_input = clamp(_steering_input, -1.0, 1.0)
	_clutch_input = clamp(_clutch_input, 0.0, 1.0)

func _update_powertrain(delta: float) -> void:
	if _clutch_input > 0.5:
		# Clutch disengaged - engine freewheels
		_engine_rpm = lerp(_engine_rpm, idle_rpm, delta * 2.0)
		return
	
	var wheel_speed_rad = _get_wheel_speed_radians()
	var drive_wheel_idx = _get_drive_wheels()[_current_gear]
	var total_reduction = gear_ratios[_current_gear] * final_drive_ratio
	
	var target_rpm = wheel_speed_rad / total_reduction
	
	# Calculate engine torque based on RPM
	var engine_torque = _calculate_engine_torque()
	
	# Apply torque to wheels
	var wheel_torque = engine_torque * total_reduction * transmission_efficiency
	
	# Update RPM based on applied torque
	var inertia = vehicle_mass * 0.01
	var rpm_change = (wheel_torque * 0.1) / inertia
	_engine_rpm = clamp(_engine_rpm + rpm_change * delta * 60.0, engine_min_rpm, engine_max_rpm)
	
	# Rev matching on downshift
	if _target_gear < _current_gear and rev_matching_enabled:
		var downshift_rpm = target_rpm * 60.0
		_engine_rpm = lerp(_engine_rpm, downshift_rpm, delta * 5.0)
	
	# Check for over-rev
	if _engine_rpm > engine_max_rpm:
		_engine_rpm = engine_max_rpm

func _calculate_engine_torque() -> float:
	var normalized_rpm = (_engine_rpm - engine_min_rpm) / (engine_max_rpm - engine_min_rpm)
	
	if normalized_rpm <= 0.0:
		return max_torque * 0.3
	
	# Interpolate torque curve
	for i in range(torque_curve_points.size() - 1):
		var p1 = torque_curve_points[i]
		var p2 = torque_curve_points[i + 1]
		
		if normalized_rpm >= p1.x and normalized_rpm <= p2.x:
			var t = (normalized_rpm - p1.x) / (p2.x - p1.x)
			return max_torque * (p1.y + t * (p2.y - p1.y))
	
	return max_torque * torque_curve_points.back().y

func _get_wheel_speed_radians() -> float:
	if _vehicle_speed == 0.0:
		return 0.0
	
	var wheel_circumference = 2.0 * PI * tire_radius
	return _vehicle_speed / wheel_circumference

func _get_drive_wheels() -> Array[int>:
	# Rear-wheel drive configuration
	return [WHEEL_RL, WHEEL_RR]

func _update_suspension(delta: float) -> void:
	var gravity_vector = Vector3(0.0, -_physics.gravity, 0.0)
	var suspension_travel = 0.2
	var spring_constant = vehicle_mass * _physics.gravity / suspension_travel
	
	for i in range(4):
		var wheel_pos = _get_world_wheel_position(i)
		var ground_height = _raycast_ground(wheel_pos)
		
		var desired_compression = -wheel_pos.y - ground_height
		desired_compression = clamp(desired_compression, 0.0, suspension_travel)
		
		var compression_diff = desired_compression - _suspension_compression[i]
		_suspension_velocity[i] = lerp(_suspension_velocity[i], compression_diff * 10.0, delta * 5.0)
		_suspension_velocity[i] = clamp(_suspension_velocity[i], -15.0, 15.0)
		
		_suspension_compression[i] += _suspension_velocity[i] * delta
		
		var spring_force = spring_constant * _suspension_compression[i]
		var damping_force = _suspension_velocity[i] * 2000.0
		
		_vertical_forces[i] = spring_force - damping_force
		_vertical_forces[i] = max(_vertical_forces[i], 0.0)

func _raycast_ground(position: Vector3) -> float:
	var ray_from = position + Vector3(0.0, 1.0, 0.0)
	var ray_to = position + Vector3(0.0, -10.0, 0.0)
	
	var space_state = get_world_3d().direct_space_state
	var query = PhysicsRayQueryParameters3D.create(ray_from, ray_to)
	query.collision_mask = 1
	
	var result = space_state.intersect_ray(query)
	
	if result.has("position"):
		return result.position.y
	else:
		return -100.0

func _get_world_wheel_position(index: int) -> Vector3:
	var offset = _wheel_positions[index]
	return global_transform * offset

func _update_aerodynamics(delta: float) -> void:
	if _vehicle_speed <= 0.0:
		return
	
	var speed_squared = _vehicle_speed * _vehicle_speed
	
	var downforce = 0.5 * downforce_coefficient * frontal_area * speed_squared
	var lift = 0.5 * lift_coefficient * frontal_area * speed_squared
	
	# Apply downforce/lift to vertical forces
	_vertical_forces[WHEEL_FFL] += downforce * 0.25
	_vertical_forces[WHEEL_FR] += downforce * 0.25
	_vertical_forces[WHEEL_RL] += downforce * 0.25
	_vertical_forces[WHEEL_RR] += downforce * 0.25
	
	if aero_debug:
		print("Downforce: ", downforce, "N, Lift: ", lift, "N")

func _update_drift(delta: float) -> void:
	if not drift_enabled or _vehicle_speed < 5.0:
		_is_drifting = false
		return
	
	var velocity = linear_velocity
	var forward = transform.basis.z.normalized()
	var right = transform.basis.x.normalized()
	
	var lateral_velocity = velocity.dot(right)
	var longitudinal_velocity = velocity.dot(forward)
	
	var slip_angle = atan2(abs(lateral_velocity), abs(longitudinal_velocity))
	
	if slip_angle > _slip_angle:
		_slip_angle = slip_angle
	
	var drift_critical = _slip_angle > drift_threshold
	
	if drift_critical and _handbrake:
		if not _is_drifting:
			_is_drifting = true
			emit_signal("drift_started")
	elif not drift_critical or not _handbrake:
		if _is_drifting:
			_is_drifting = false
			emit_signal("drift_ended")
			_slip_angle = lerp(_slip_angle, 0.0, delta * drift_recovery_rate)

func _check_gear_shifts(delta: float) -> void:
	if _target_gear != _current_gear:
		var old_gear = _current_gear
		_current_gear = _target_gear
		emit_signal("gear_changed", _current_gear)
		
		if _current_gear == 0:
			_engine_rpm = idle_rpm

func _update_fuel(delta: float) -> void:
	if _throttle_input > 0.0:
		_fuel_consumption_rate = 0.5 * _throttle_input + 0.1
		_fuel_level -= _fuel_consumption_rate * delta
		_fuel_level = max(_fuel_level, 0.0)

func _update_signals(delta: float) -> void:
	emit_signal("speed_changed", _vehicle_speed)
	emit_signal("rpm_changed", _engine_rpm)
	emit_signal("wheel_slip", 0, _slip_angle)

func _apply_physics(delta: float) -> void:
	var total_vertical_force = 0.0
	for force in _vertical_forces:
		total_vertical_force += force
	
	var effective_gravity = (_physics.gravity * vehicle_mass - total_vertical_force) / vehicle_mass
	
	velocity.y -= effective_gravity * delta
	
	var horizontal_force = Vector3.ZERO
	
	for i in range(4):
		horizontal_force += _get_wheel_force(i)
	
	velocity += horizontal_force * delta / vehicle_mass
	
	velocity = velocity.limit_length(150.0)

func _get_wheel_force(index: int) -> Vector3:
	var force = Vector3.ZERO
	
	force.y = _vertical_forces[index]
	
	var wheel_pos = _get_world_wheel_position(index)
	var ground_normal = Vector3(0.0, 1.0, 0.0)
	
	var tire_friction = _get_tire_friction(index)
	
	var longitudinal_force = _longitudinal_forces[index]
	var lateral_force = _lateral_forces[index]
	
	var wheel_forward = transform.basis.z
	var wheel_right = transform.basis.x
	
	force += longitudinal_force * wheel_forward
	force += lateral_force * wheel_right
	
	return force

func _get_tire_friction(index: int) -> float:
	var temp = _tire_temperatures[index]
	var optimal_temp = tire_temperature_optimal
	
	var temp_factor = 1.0
	if temp > optimal_temp:
		temp_factor = 1.0 - (temp - optimal_temp) / (tire_temperature_max - optimal_temp)
	
	return tire_friction_dynamic * temp_factor

func _update_collision_detection(delta: float) -> void:
	var collisions = get_colliding_bodies()
	
	for body in collisions:
		var collision_info = body.get_meta("_collision_info", {})
		var impact_speed = linear_velocity.length()
		
		if impact_speed > 5.0:
			emit_signal("collision_impact", impact_speed, global_position)

func _cleanup_acceleration_history() -> void:
	while _acceleration_history.size() > 10:
		_acceleration_history.pop_front()

func set_gear(gear_index: int) -> void:
	_target_gear = clamp(gear_index, 0, gear_ratios.size() - 1)

func reset_vehicle() -> void:
	_reset_state()
	linear_velocity = Vector3.ZERO
	angular_velocity = Vector3.ZERO
	global_position = Vector3.ZERO
	rotation = Vector3.ZERO

func get_current_speed() -> float:
	return _vehicle_speed

func get_current_rpm() -> float:
	return _engine_rpm

func get_current_gear() -> int:
	return _current_gear

func get_fuel_level() -> float:
	return _fuel_level

func is_drifting() -> bool:
	return _is_drifting

func get_wheel_rotation_angle(index: int) -> float:
	if index >= 0 and index < 4:
		return _wheel_rotation_angles[index]
	return 0.0

func set_wheel_rotation_angle(index: int, angle: float) -> void:
	if index >= 0 and index < 4:
		_wheel_rotation_angles[index] = angle

func get_wheel_slip_ratio(index: int) -> float:
	if index >= 0 and index < 4:
		return _wheel_rotation_speeds[index] / max(_vehicle_speed, 0.1)
	return 0.0

func apply_brake_force(index: int, pressure: float) -> void:
	if index >= 0 and index < 4:
		var brake_force = pressure * brake_pressure_front * 1000.0
		_longitudinal_forces[index] -= brake_force

func reset_brakes() -> void:
	for i in range(4):
		_longitudinal_forces[i] = 0.0

func update_tire_temperature(index: int, temp: float) -> void:
	if index >= 0 and index < 4:
		_tire_temperatures[index] = clamp(temp, 0.0, tire_temperature_max)

func update_brake_temperature(index: int, temp: float) -> void:
	if index >= 0 and index < 4:
		_brake_temperatures[index] = clamp(temp, 0.0, brake_temperature_max)

func calculate_cornering_force(slip_angle: float) -> float:
	var cornering_stiffness = tire_stiffness_factor * tire_friction_static * 100.0
	return -cornering_stiffness * slip_angle

func calculate_longitudinal_force(slips: float) -> float:
	var peak_friction = tire_friction_static * _vertical_forces[0] if _vertical_forces[0] > 0 else 1.0
	return peak_friction * slips

func debug_print_status() -> void:
	print("\n=== Vehicle Status ===")
	print("Speed: ", _vehicle_speed, " km/h")
	print("RPM: ", _engine_rpm)
	print("Gear: ", _current_gear)
	print("Fuel: ", _fuel_level, "%")
	print("Drifting: ", _is_drifting)
	print("Throttle: ", _throttle_input)
	print("Brake: ", _brake_input)
	print("Steering: ", _steering_input)
	print("====================\n")

func _on_vehicle_damage(damage_amount: float, impact_point: Vector3) -> void:
	_fuel_level -= damage_amount * 0.5
	print("Vehicle damaged! Fuel lost: ", damage_amount * 0.5)

func _on_vehicle_fuel_empty() -> void:
	print("OUT OF FUEL!")
	_engine_rpm = 0.0
	_linear_velocity *= 0.99

func get_aerodynamic_downforce() -> float:
	if _vehicle_speed <= 0.0:
		return 0.0
	return 0.5 * downforce_coefficient * frontal_area * _vehicle_speed * _vehicle_speed

func get_drag_force() -> float:
	if _vehicle_speed <= 0.0:
		return 0.0
	return 0.5 * drag_coefficient * frontal_area * _vehicle_speed * _vehicle_speed

func get_total_vertical_load() -> float:
	var total = 0.0
	for force in _vertical_forces:
		total += force
	return total

func reset_all_settings() -> void:
	vehicle_mass = _physics.default_vehicle_mass
	engine_max_rpm = 7500.0
	max_torque = 450.0
	gear_ratios = [3.8, 2.2, 1.5, 1.1, 0.9, 0.75, 0.6]
	final_drive_ratio = 3.5
	drag_coefficient = 0.30
	frontal_area = 2.2
	vehicle_mass = 1500.0
	_reset_state()
	print("All settings reset to defaults")

func load_custom_setup(config: Dictionary) -> void:
	if config.has("mass"):
		vehicle_mass = config["mass"]
	if config.has("max_rpm"):
		engine_max_rpm = config["max_rpm"]
	if config.has("torque"):
		max_torque = config["torque"]
	if config.has("gears"):
		gear_ratios = config["gears"]
	if config.has("final_drive"):
		final_drive_ratio = config["final_drive"]
	if config.has("drag"):
		drag_coefficient = config["drag"]
	if config.has("area"):
		frontal_area = config["area"]
	_reset_state()
	print("Custom setup loaded")

func save_setup() -> Dictionary:
	return {
		"mass": vehicle_mass,
		"max_rpm": engine_max_rpm,
		"torque": max_torque,
		"gears": gear_raties.duplicate(),
		"final_drive": final_drive_ratio,
		"drag": drag_coefficient,
		"area": frontal_area
	}

func simulate_race_lap(duration_seconds: float) -> void:
	var start_time = Time.get_ticks_msec()
	var lap_data = {}
	
	while Time.get_ticks_msec() - start_time < duration_seconds * 1000:
		await get_tree().create_timer(0.016).timeout
		_process(0.016)
	
	lap_data["duration"] = duration_seconds
	lap_data["avg_speed"] = _distance_traveled / duration_seconds
	lap_data["max_rpm"] = _engine_rpm
	lap_data["fuel_used"] = 100.0 - _fuel_level
	
	print("Race simulation complete")
	print("Duration: ", duration_seconds, "s")
	print("Avg Speed: ", lap_data["avg_speed"], " m/s")
	print("Max RPM: ", lap_data["max_rpm"])
	print("Fuel Used: ", lap_data["fuel_used"], "%")
	
	return lap_data

func _on_input_event(viewport: Viewport, event: InputEvent) -> void:
	if event is InputEventKey:
		if event.pressed:
			match event.key_code:
				KEY_1:
					set_gear(0)
				KEY_2:
					set_gear(1)
				KEY_3:
					set_gear(2)
				KEY_4:
					set_gear(3)
				KEY_5:
					set_gear(4)
				KEY_6:
					set_gear(5)
				KEY_7:
					set_gear(6)
				KEY_R:
					reset_vehicle()
				KEY_F:
					debug_print_status()

func _validate_vehicle_config() -> bool:
	var errors = []
	
	if vehicle_mass <= 0.0:
		errors.append("Invalid vehicle mass")
	if wheel_base <= 0.0:
		errors.append("Invalid wheel base")
	if track_width <= 0.0:
		errors.append("Invalid track width")
	if tire_radius <= 0.0:
		errors.append("Invalid tire radius")
	if gear_ratios.size() < 1:
		errors.append("At least one gear required")
	if engine_max_rpm <= engine_min_rpm:
		errors.append("Max RPM must exceed min RPM")
	
	if errors.size() > 0:
		push_error("Vehicle configuration errors:")
		for error in errors:
			push_error("  - " + error)
		return false
	
	return true

func _on_scene_changed() -> void:
	_reset_state()
	print("Scene changed, resetting vehicle state")

func _notification(what: int) -> void:
	match what:
		NOTIFICATION_WM_CLOSE_REQUEST:
			save_setup()
		NOTIFICATION_EXIT_TREE:
			print("VehicleController destroyed")

func _get_configuration_warnings() -> Array[String]:
	var warnings = []
	
	if torque_curve_points.is_empty():
		warnings.append("Torque curve is empty - using default")
	if gear_ratios.is_empty():
		warnings.append("No gear ratios defined")
	if not _validate_vehicle_config():
		warnings.append("Vehicle configuration has errors")
	
	return warnings