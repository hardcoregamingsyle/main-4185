extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulation
## Handles throttle, brake, steering, wheel forces, gear shifting, and vehicle dynamics
## Copyright 2026 Thalamus Racing Simulator Project

# Signals for game state communication
signal speed_changed(speed: float)
signal rpm_changed(rpm: float)
signal gear_changed(gear: int)
signal drift_started()
signal drift_ended()
signal collision_impact(impact_force: float, impact_position: Vector3)
signal wheel_slip(wheel_index: int, slip_ratio: float)

# Physics Settings reference
var physics_settings: PhysicsSettings = PhysicsSettings

# Vehicle Configuration
@export_group("Vehicle Mass & Dimensions")
@export var mass: float = 1500.0: set = _set_mass
@export var center_of_mass_offset: Vector3 = Vector3(0.0, -0.5, 0.0)
@export var track_width_front: float = 1.6
@export var track_width_rear: float = 1.7
@export var wheelbase: float = 2.6
@export var ride_height: float = 0.3

@export_group("Aerodynamics")
@export var drag_coefficient: float = 0.30
@export var frontal_area: float = 2.2
@export var downforce_coefficient: float = 0.8
@export var aero_balance: float = 0.4: range(0.0, 1.0) # 0=rear, 1=front

@export_group("Suspension Geometry")
@export var suspension_travel_max: float = 0.15
@export var suspension_stiffness: float = 50000.0
@export var suspension_damping: float = 3000.0
@export var camber_gain: float = -0.02
@export var toe_gain: float = 0.005

@export_group("Engine & Powertrain")
@export var engine_power_hp: float = 400.0
@export var engine_torque_nm: float = 500.0
@export var redline_rpm: float = 7500.0
@export var idle_rpm: float = 800.0
@export var torque_curve_points: Array[Vector2] = [
	Vector2(0, 0.3),
	Vector2(2000, 0.7),
	Vector2(4000, 1.0),
	Vector2(6000, 0.95),
	Vector2(7500, 0.8)
]

@export_group("Transmission")
@export var transmission_type: String = "manual"
@export var final_drive_ratio: float = 3.73
@export var gear_ratios: Dictionary = {
	1: 3.5,
	2: 2.1,
	3: 1.5,
	4: 1.1,
	5: 0.9,
	6: 0.75,
	R: 3.8,
	N: 0.0
}
@export var shift_up_threshold: float = 0.9
@export var shift_down_threshold: float = 0.3

@export_group("Braking System")
@export var brake_force_factor: float = 12.0
@export var brake_bias_front: float = 0.6: range(0.3, 0.7)
@export var abs_enabled: bool = true
@export var brake_temperature: float = 100.0
@export var brake_efficiency: float = 1.0

@export_group("Tires & Friction")
@export var tire_friction_coefficient: float = 1.2
@export var tire_spring_rate: float = 80000.0
@export var tire_damping: float = 4000.0
@export var lateral_grip: float = 1.2
@export var longitudinal_grip: float = 1.1
@export var grip_loss_threshold: float = 0.85

@export_group("Steering")
@export var steering_ratio: float = 14.0
@export var max_steering_angle: float = 0.5: radians_to_degrees(0.5)
@export var steering_speed: float = 2.5
@export var power_assist: bool = true
@export var lock_to_lock_turns: float = 2.8

@export_group("Drift Mechanics")
@export var drift_mode_enabled: bool = false
@export var drift_grip_reduction: float = 0.6
@export var drift_recovery_delay: float = 2.0
@export var drift_threshold_angle: float = 15.0: radians_to_degrees(15.0)
@export var drift_recovery_rate: float = 0.1

@export_group("Wheel Properties")
@export var wheel_radius: float = 0.32
@export var wheel_inertia: float = 0.8
@export var unsprung_mass: float = 45.0
@export var wheel_positions: Array[Vector3] = []

# Runtime State Variables
var current_gear: int = 0
var target_gear: int = 0
var clutch_engaged: bool = true
var rpm: float = 0.0
var speed_kmh: float = 0.0
var wheel_rpms: Array[float] = []
var wheel_forces: Array[Vector3] = []
var wheel_vertical_loads: Array[float] = []
var wheel_slip_ratios: Array[float] = []
var wheel_contact_positions: Array[Vector3] = []
var wheel_contact_normals: Array[Vector3] = []

var throttle_input: float = 0.0
var brake_input: float = 0.0
var steering_input: float = 0.0
var handbrake_input: float = 0.0

var drift_active: bool = false
var drift_timer: float = 0.0
var drift_angle: float = 0.0
var drift_counter: int = 0

var brake_temp_history: Array[float] = []
var brake_efficiency_current: float = 1.0

var aerodynamic_downforce: float = 0.0
var aerodynamic_drag: float = 0.0

var _engine_torque_output: float = 0.0
var _vehicle_velocity_world: Vector3 = Vector3.ZERO
var _ground_normal: Vector3 = Vector3.UP
var _is_on_ground: bool = false
var _collisions_pending: Array[Dictionary] = []

# Wheel indices
const WHEEL_FRONT_LEFT: int = 0
const WHEEL_FRONT_RIGHT: int = 1
const WHEEL_REAR_LEFT: int = 2
const WHEEL_REAR_RIGHT: int = 3

func _ready() -> void:
	_init_wheel_positions()
	_init_wheel_states()
	_connect_signals_to_systems()
	_update_physics_settings()

func _init_wheel_positions() -> void:
	var half_track_front: float = track_width_front / 2.0
	var half_track_rear: float = track_width_rear / 2.0
	var front_offset: float = wheelbase * 0.5
	var rear_offset: float = -wheelbase * 0.5
	
	wheel_positions = [
		Vector3(-half_track_front, -ride_height, front_offset),
		Vector3(half_track_front, -ride_height, front_offset),
		Vector3(-half_track_rear, -ride_height, rear_offset),
		Vector3(half_track_rear, -ride_height, rear_offset)
	]

func _init_wheel_states() -> void:
	for i in range(4):
		wheel_rpms.append(0.0)
		wheel_forces.append(Vector3.ZERO)
		wheel_vertical_loads.append(0.0)
		wheel_slip_ratios.append(0.0)
		wheel_contact_positions.append(Vector3.ZERO)
		wheel_contact_normals.append(Vector3.UP)

func _connect_signals_to_systems() -> void:
	speed_changed.connect(_on_speed_changed)
	rpm_changed.connect(_on_rpm_changed)
	gear_changed.connect(_on_gear_changed)
	collision_impact.connect(_on_collision_impact)

func _update_physics_settings() -> void:
	if Engine.has_singleton("PhysicsSettings"):
		physics_settings = PhysicsSettings
	else:
		printerr("[VehicleController] PhysicsSettings singleton not found!")

func _input(event: InputEvent) -> void:
	if event.is_action_pressed("shift_up"):
		_shift_gear(1)
	elif event.is_action_pressed("shift_down"):
		_shift_gear(-1)

func _physics_process(delta: float) -> void:
	_handle_inputs(delta)
	_calculate_vehicle_dynamics(delta)
	_apply_wheel_forces(delta)
	_update_aerodynamics(delta)
	_update_brake_temperatures(delta)
	_handle_drift_mechanics(delta)
	_synchronize_body_state()

func _handle_inputs(delta: float) -> void:
	throttle_input = Input.get_axis("throttle_minus", "throttle_plus")
	brake_input = Input.get_axis("brake_minus", "brake_plus")
	steering_input = Input.get_axis("steering_left", "steering_right")
	handbrake_input = float(Input.is_action_pressed("handbrake"))
	
	if power_assist:
		steering_input *= 0.6

func _calculate_vehicle_dynamics(delta: float) -> void:
	_get_ground_raycast()
	_calculate_engine_output()
	_calculate_transmission()
	_calculate_wheel_slip()
	_calculate_tire_forces(delta)
	_calculate_total_forces(delta)
	apply_central_force(get_total_force())
	apply_torque(get_total_torque())

func _get_ground_raycast() -> void:
	var origin: Vector3 = global_position + Vector3(0.0, -0.1, 0.0)
	var direction: Vector3 = Vector3.DOWN
	var distance: float = 2.0
	var space_state: SpaceState = get_world_3d().direct_space_state
	
	var query: PhysicsRayQueryParameters3D = PhysicsRayQueryParameters3D.create(
		origin,
		origin + direction * distance
	)
	query.exclude = [self]
	
	var result: Dictionary = space_state.ray_query(query)
	
	if result["hit"]:
		_ground_normal = result["normal"]
		_is_on_ground = true
	else:
		_ground_normal = Vector3.UP
		_is_on_ground = false

func _calculate_engine_output() -> void:
	var gear_ratio: float = gear_ratios[current_gear] if current_gear in gear_ratios else 1.0
	var effective_ratio: float = gear_ratio * final_drive_ratio
	
	var target_rpm: float = (speed_kmh * 1000.0 / (2.0 * PI * wheel_radius)) / effective_ratio
	target_rpm = clamp(target_rpm, 0.0, redline_rpm)
	
	if clutch_engaged:
		var rpm_target: float = target_rpm
		if throttle_input > 0.0:
			rpm_target = min(rpm_target + delta * 3000.0, redline_rpm)
		else:
			rpm_target = max(rpm_target - delta * 2000.0, idle_rpm)
		
		rpm = lerp(rpm, rpm_target, delta * 10.0)
	else:
		rpm = lerp(rpm, idle_rpm, delta * 5.0)
	
	_engine_torque_output = _interpolate_torque_curve(rpm) * engine_torque_nm

func _interpolate_torque_curve(rpm_value: float) -> float:
	if torque_curve_points.is_empty():
		return 1.0
	
	var points = torque_curve_points
	var max_rpm: float = points[-1].x
	
	if rpm_value >= max_rpm:
		return points[-1].y
	
	for i in range(points.size() - 1):
		var p1 = points[i]
		var p2 = points[i + 1]
		
		if rpm_value >= p1.x and rpm_value <= p2.x:
			var t: float = (rpm_value - p1.x) / (p2.x - p1.x)
			return p1.y + t * (p2.y - p1.y)
	
	return 1.0

func _calculate_transmission() -> void:
	var wheel_speed_rad_s: float = (speed_kmh * 1000.0 / 3600.0) / wheel_radius
	var engine_speed_rad_s: float = rpm * PI / 30.0
	
	var current_ratio: float = gear_ratios[current_gear] if current_gear in gear_ratios else 1.0
	var theoretical_rpm: float = (wheel_speed_rad_s * current_ratio * final_drive_ratio) * 30.0 / PI
	
	if current_gear == 0:
		current_gear = 1
	
	if rpm > redline_rpm * shift_up_threshold and current_gear < 6:
		_shift_gear(1)
	elif rpm < idle_rpm * shift_down_threshold and current_gear > 1:
		_shift_gear(-1)

func _shift_gear(direction: int) -> void:
	if not clutch_engaged:
		return
	
	var new_gear: int = current_gear + direction
	
	if new_gear < 0:
		new_gear = 0
	elif new_gear > 6:
		new_gear = 6
	
	if new_gear != current_gear:
		current_gear = new_gear
		gear_changed.emit(current_gear)

func _calculate_wheel_slip() -> void:
	var total_force: Vector3 = get_total_force()
	var forward_vector: Vector3 = transform.basis.z.normalized()
	var velocity_mps: float = speed_kmh / 3.6
	
	for i in range(4):
		var wheel_velocity: float = wheel_rpms[i] * wheel_radius
		var slip_direction: float = 1.0 if i < 2 else -1.0
		
		var drive_slip: float = 0.0
		var braking_slip: float = 0.0
		
		if current_gear != 0 and throttle_input > 0.1:
			var engine_wheel_speed: float = rpm * wheel_radius / (gear_ratios[current_gear] * final_drive_ratio)
			drive_slip = (engine_wheel_speed - wheel_velocity) / max(abs(wheel_velocity), 0.1)
		
		if brake_input > 0.1:
			braking_slip = (velocity_mps - wheel_velocity) / max(abs(wheel_velocity), 0.1)
		
		wheel_slip_ratios[i] = drive_slip - braking_slip
		wheel_slip_ratios[i] = clamp(wheel_slip_ratios[i], -1.0, 1.0)
		
		if abs(wheel_slip_ratios[i]) > 0.1:
			wheel_slip.emit(i, wheel_slip_ratios[i])

func _calculate_tire_forces(delta: float) -> void:
	var grip_multiplier: float = drift_grip_reduction if drift_active else 1.0
	
	for i in range(4):
		var lateral_grip_local: float = lateral_grip * grip_multiplier
		var longitudinal_grip_local: float = longitudinal_grip * grip_multiplier
		
		var slip: float = wheel_slip_ratios[i]
		var force_mag: float = 0.0
		
		if abs(slip) > 0.01:
			force_mag = -slip * longitudinal_grip_local * wheel_vertical_loads[i]
		
		var wheel_transform: Transform3D = Transform3D(Basis(), wheel_positions[i] + global_position)
		var wheel_forward: Vector3 = wheel_transform.basis.z.normalized()
		
		var longitudinal_force: Vector3 = wheel_forward * force_mag
		wheel_forces[i] = longitudinal_force
		wheel_rpms[i] = (speed_kmh / 3.6) / wheel_radius if current_gear != 0 else rpm * wheel_radius / (gear_ratios[current_gear] * final_drive_ratio)

func _calculate_total_forces(delta: float) -> void:
	var total_force: Vector3 = Vector3.ZERO
	var total_torque: Vector3 = Vector3.ZERO
	
	for i in range(4):
		total_force += wheel_forces[i]
		
		var wheel_pos: Vector3 = wheel_positions[i]
		var cross_product: Vector3 = wheel_pos.cross(wheel_forces[i])
		total_torque += cross_product
	
	set_mass(mass)
	set_total_force(total_force)
	set_total_torque(total_torque)

func _apply_wheel_forces(delta: float) -> void:
	var contact_point_offset: float = wheel_radius
	
	for i in range(4):
		var contact_point: Vector3 = global_position + wheel_positions[i]
		var force: Vector3 = wheel_forces[i]
		
		apply_force(force, contact_point)

func _update_aerodynamics(delta: float) -> void:
	var velocity_mps: float = speed_kmh / 3.6
	
	aerodynamic_drag = 0.5 * drag_coefficient * frontal_area * velocity_mps * velocity_mps
	aerodynamic_downforce = 0.5 * downforce_coefficient * frontal_area * velocity_mps * velocity_mps
	
	var drag_force: Vector3 = -transform.basis.z.normalized() * aerodynamic_drag
	var downforce_force: Vector3 = Vector3.DOWN * aerodynamic_downforce
	
	apply_force(drag_force, global_position)
	apply_force(downforce_force, global_position)

func _update_brake_temperatures(delta: float) -> void:
	var braking_energy: float = brake_input * brake_force_factor * speed_kmh
	
	if brake_input > 0.1:
		brake_temp_history.append(braking_energy)
		if brake_temp_history.size() > 60:
			brake_temp_history.resize(60)
		
		var avg_temp: float = sum_array(brake_temp_history) / brake_temp_history.size()
		brake_temperature = max(brake_temperature, avg_temp)
		
		if brake_temperature > 300.0:
			brake_efficiency_current = 1.0 - ((brake_temperature - 300.0) / 500.0)
			brake_efficiency_current = clamp(brake_efficiency_current, 0.3, 1.0)
		else:
			brake_efficiency_current = 1.0
	
	if brake_input < 0.1 and brake_temperature > 100.0:
		cooling_rate: float = 2.0
		brake_temperature = max(brake_temperature - cooling_rate * delta, 100.0)

func _handle_drift_mechanics(delta: float) -> void:
	if not drift_mode_enabled:
		return
	
	var angular_velocity_z: float = rotation.y
	var drift_angle_local: float = degrees_to_radians(angular_velocity_z)
	
	if abs(drift_angle_local) > degrees_to_radians(drift_threshold_angle):
		if not drift_active:
			drift_active = true
			drift_timer = drift_recovery_delay
			drift_started.emit()
		
		drift_counter += 1
		drift_angle = lerp(drift_angle, drift_angle_local, delta * drift_recovery_rate)
	else:
		drift_counter -= 1
		if drift_counter <= 0:
			drift_active = false
			drift_ended.emit()
			drift_angle = 0.0
			drift_timer = 0.0

func _synchronize_body_state() -> void:
	_vehicle_velocity_world = linear_velocity
	speed_kmh = _vehicle_velocity_world.length() * 3.6
	
	if not is_on_floor():
		ground_normal = Vector3.UP
		is_on_ground = false

func _on_speed_changed(new_speed: float) -> void:
	pass

func _on_rpm_changed(new_rpm: float) -> void:
	pass

func _on_gear_changed(new_gear: int) -> void:
	pass

func _on_collision_impact(impact_force: float, impact_position: Vector3) -> void:
	_collisions_pending.append({
		"force": impact_force,
		"position": impact_position,
		"time": Time.get_ticks_msec()
	})

func apply_central_force(force: Vector3) -> void:
	linear_velocity += force * get_physics_process_delta_time() / mass

func apply_torque(torque: Vector3) -> void:
	angular_velocity += torque * get_physics_process_delta_time() / moment_of_inertia()

func get_total_force() -> Vector3:
	var total: Vector3 = Vector3.ZERO
	for force in wheel_forces:
		total += force
	return total

func get_total_torque() -> Vector3:
	var total: Vector3 = Vector3.ZERO
	for i in range(4):
		var pos: Vector3 = wheel_positions[i]
		total += pos.cross(wheel_forces[i])
	return total

func moment_of_inertia() -> float:
	return mass * (wheelbase * wheelbase + track_width_front * track_width_front) / 12.0

func set_mass(new_mass: float) -> void:
	mass = new_mass
	body_mass = new_mass

func _set_mass(value: float) -> void:
	mass = value
	body_mass = value

func _set_gravity(value: float) -> void:
	gravity = value

func _set_physics_tick_rate(value: int) -> void:
	physics_tick_rate = value

func _set_max_substeps(value: int) -> void:
	max_substeps = value

func _set_time_scale(value: float) -> void:
	time_scale = value

func _set_default_vehicle_mass(value: float) -> void:
	default_vehicle_mass = value

func _set_default_wheel_base(value: float) -> void:
	default_wheelbase = value

func _set_default_track_width(value: float) -> void:
	default_track_width = value

func _set_default_suspension_stiffness(value: float) -> void:
	default_suspension_stiffness = value

func _set_default_suspension_damping(value: float) -> void:
	default_suspension_damping = value

func _set_default_tire_friction(value: float) -> void:
	default_tire_friction = value

func _set_default_brake_force(value: float) -> void:
	default_brake_force = value

func _set_default_steering_ratio(value: float) -> void:
	default_steering_ratio = value

func _set_default_max_steering(value: float) -> void:
	default_max_steering = value

func _set_default_engine_power(value: float) -> void:
	default_engine_power = value

func _set_default_engine_torque(value: float) -> void:
	default_engine_torque = value

func _set_default_redline(value: float) -> void:
	default_redline = value

func _set_default_idle_rpm(value: float) -> void:
	default_idle_rpm = value

func _set_default_final_drive(value: float) -> void:
	default_final_drive = value

func _set_default_gear_ratios(value: Dictionary) -> void:
	default_gear_ratios = value

func _set_default_shift_up_threshold(value: float) -> void:
	default_shift_up_threshold = value

func _set_default_shift_down_threshold(value: float) -> void:
	default_shift_down_threshold = value

func _set_default_abs_enabled(value: bool) -> void:
	default_abs_enabled = value

func _set_default_drift_mode_enabled(value: bool) -> void:
	default_drift_mode_enabled = value

func _set_default_drift_grip_reduction(value: float) -> void:
	default_drift_grip_reduction = value

func _set_default_drift_threshold_angle(value: float) -> void:
	default_drift_threshold_angle = value

func _set_default_drift_recovery_rate(value: float) -> void:
	default_drift_recovery_rate = value

func reset_vehicle() -> void:
	current_gear = 0
	clutch_engaged = true
	rpm = 0.0
	speed_kmh = 0.0
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	handbrake_input = 0.0
	drift_active = false
	drift_timer = 0.0
	drift_angle = 0.0
	drift_counter = 0
	brake_temperature = 100.0
	brake_efficiency_current = 1.0
	aerodynamic_downforce = 0.0
	aerodynamic_drag = 0.0
	_engine_torque_output = 0.0
	_linear_velocity = Vector3.ZERO
	_angular_velocity = Vector3.ZERO
	_is_on_ground = false
	_ground_normal = Vector3.UP
	_collisions_pending.clear()
	wheel_rpms.fill(0.0)
	wheel_forces.fill(Vector3.ZERO)
	wheel_vertical_loads.fill(0.0)
	wheel_slip_ratios.fill(0.0)
	wheel_contact_positions.fill(Vector3.ZERO)
	wheel_contact_normals.fill(Vector3.UP)

func sum_array(arr: Array[float]) -> float:
	var total: float = 0.0
	for val in arr:
		total += val
	return total

func degrees_to_radians(degrees: float) -> float:
	return degrees * PI / 180.0

func radians_to_degrees(radians: float) -> float:
	return radians * 180.0 / PI

func set_global_position(pos: Vector3) -> void:
	global_position = pos

func set_global_rotation(rot: float) -> void:
	global_rotation.y = rot

func get_linear_velocity() -> Vector3:
	return linear_velocity

func get_angular_velocity() -> Vector3:
	return angular_velocity

func is_vehicle_moving() -> bool:
	return speed_kmh > 1.0

func is_vehicle_stopped() -> bool:
	return speed_kmh < 0.5 and abs(angular_velocity.y) < 0.01

func get_vehicle_speed() -> float:
	return speed_kmh

func get_vehicle_rpm() -> float:
	return rpm

func get_vehicle_gear() -> int:
	return current_gear

func is_clutch_engaged() -> bool:
	return clutch_engaged

func toggle_clutch() -> void:
	clutch_engaged = not clutch_engaged

func engage_clutch() -> void:
	clutch_engaged = true

func disengage_clutch() -> void:
	clutch_engaged = false

func set_throttle_input(value: float) -> void:
	throttle_input = clamp(value, -1.0, 1.0)

func set_brake_input(value: float) -> void:
	brake_input = clamp(value, -1.0, 1.0)

func set_steering_input(value: float) -> void:
	steering_input = clamp(value, -1.0, 1.0)

func set_handbrake_input(value: float) -> void:
	handbrake_input = clamp(value, 0.0, 1.0)

func get_throttle_input() -> float:
	return throttle_input

func get_brake_input() -> float:
	return brake_input

func get_steering_input() -> float:
	return steering_input

func get_handbrake_input() -> float:
	return handbrake_input

func activate_drift_mode() -> void:
	drift_mode_enabled = true

func deactivate_drift_mode() -> void:
	drift_mode_enabled = false

func is_drifting() -> bool:
	return drift_active

func get_drift_angle() -> float:
	return drift_angle

func get_brake_efficiency() -> float:
	return brake_efficiency_current

func get_brake_temperature() -> float:
	return brake_temperature

func get_downforce() -> float:
	return aerodynamic_downforce

func get_drag() -> float:
	return aerodynamic_drag

func get_wheel_slip(wheel_index: int) -> float:
	if wheel_index >= 0 and wheel_index < 4:
		return wheel_slip_ratios[wheel_index]
	return 0.0

func get_wheel_vertical_load(wheel_index: int) -> float:
	if wheel_index >= 0 and wheel_index < 4:
		return wheel_vertical_loads[wheel_index]
	return 0.0

func get_wheel_force(wheel_index: int) -> Vector3:
	if wheel_index >= 0 and wheel_index < 4:
		return wheel_forces[wheel_index]
	return Vector3.ZERO

func get_wheel_rpm(wheel_index: int) -> float:
	if wheel_index >= 0 and wheel_index < 4:
		return wheel_rpms[wheel_index]
	return 0.0

func get_center_of_mass() -> Vector3:
	return center_of_mass_offset

func get_track_width_front() -> float:
	return track_width_front

func get_track_width_rear() -> float:
	return track_width_rear

func get_wheelbase() -> float:
	return wheelbase

func get_mass() -> float:
	return mass

func get_engine_power_hp() -> float:
	return engine_power_hp

func get_engine_torque_nm() -> float:
	return engine_torque_nm

func get_redline_rpm() -> float:
	return redline_rpm

func get_idle_rpm() -> float:
	return idle_rpm

func get_current_rpm() -> float:
	return rpm

func get_current_gear() -> int:
	return current_gear

func get_all_gear_ratios() -> Dictionary:
	return gear_ratios

func get_final_drive_ratio() -> float:
	return final_drive_ratio

func get_drag_coefficient() -> float:
	return drag_coefficient

func get_frontal_area() -> float:
	return frontal_area

func get_downforce_coefficient() -> float:
	return downforce_coefficient

func get_tire_friction_coefficient() -> float:
	return tire_friction_coefficient

func get_lateral_grip() -> float:
	return lateral_grip

func get_longitudinal_grip() -> float:
	return longitudinal_grip

func get_steering_ratio() -> float:
	return steering_ratio

func get_max_steering_angle() -> float:
	return max_steering_angle

func get_brake_force_factor() -> float:
	return brake_force_factor

func get_brake_bias_front() -> float:
	return brake_bias_front

func get_suspension_travel_max() -> float:
	return suspension_travel_max

func get_suspension_stiffness() -> float:
	return suspension_stiffness

func get_suspension_damping() -> float:
	return suspension_damping

func get_wheels_count() -> int:
	return 4

func get_wheel_positions() -> Array[Vector3]:
	return wheel_positions

func get_wheel_radius() -> float:
	return wheel_radius

func get_unsprung_mass() -> float:
	return unsprung_mass

func get_wheel_inertia() -> float:
	return wheel_inertia

func get_shift_up_threshold() -> float:
	return shift_up_threshold

func get_shift_down_threshold() -> float:
	return shift_down_threshold

func get_clutch_engaged() -> bool:
	return clutch_engaged

func get_abs_enabled() -> bool:
	return abs_enabled

func get_drift_mode_enabled() -> bool:
	return drift_mode_enabled

func get_drift_grip_reduction() -> float:
	return drift_grip_reduction

func get_drift_recovery_delay() -> float:
	return drift_recovery_delay

func get_drift_threshold_angle() -> float:
	return drift_threshold_angle

func get_drift_recovery_rate() -> float:
	return drift_recovery_rate

func set_transmission_type(type_str: String) -> void:
	transmission_type = type_str

func set_gear_ratios(new_ratios: Dictionary) -> void:
	gear_ratios = new_ratios

func set_final_drive_ratio(new_ratio: float) -> void:
	final_drive_ratio = new_ratio

func set_torque_curve(new_curve: Array[Vector2]) -> void:
	torque_curve_points = new_curve

func set_brake_force_factor(new_factor: float) -> void:
	brake_force_factor = new_factor

func set_brake_bias(new_bias: float) -> void:
	brake_bias_front = clamp(new_bias, 0.3, 0.7)

func set_steering_ratio(new_ratio: float) -> void:
	steering_ratio = new_ratio

func set_max_steering_angle(new_angle: float) -> void:
	max_steering_angle = new_angle

func set_suspension_stiffness(new_stiffness: float) -> void:
	suspension_stiffness = new_stiffness

func set_suspension_damping(new_damping: float) -> void:
	suspension_damping = new_damping

func set_tire_friction(new_friction: float) -> void:
	tire_friction_coefficient = new_friction

func set_lateral_grip(new_grip: float) -> void:
	lateral_grip = new_grip

func set_longitudinal_grip(new_grip: float) -> void:
	longitudinal_grip = new_grip

func set_mass(new_mass: float) -> void:
	mass = new_mass
	body_mass = new_mass

func set_engine_power(new_power: float) -> void:
	engine_power_hp = new_power

func set_engine_torque(new_torque: float) -> void:
	engine_torque_nm = new_torque

func set_redline(new_redline: float) -> void:
	redline_rpm = new_redline

func set_idle_rpm(new_idle: float) -> void:
	idle_rpm = new_idle

func set_drag_coefficient(new_cd: float) -> void:
	drag_coefficient = new_cd

func set_frontal_area(new_area: float) -> void:
	frontal_area = new_area

func set_downforce_coefficient(new_cd: float) -> void:
	downforce_coefficient = new_cd

func set_drift_mode(enabled: bool) -> void:
	drift_mode_enabled = enabled

func set_drift_grip_reduction(new_reduction: float) -> void:
	drift_grip_reduction = new_reduction

func set_drift_threshold_angle(new_angle: float) -> void:
	drift_threshold_angle = new_angle

func set_drift_recovery_rate(new_rate: float) -> void:
	drift_recovery_rate = new_rate

func print_vehicle_stats() -> void:
	print("[VehicleStats]")
	print("Speed: %.2f km/h" % speed_kmh)
	print("RPM: %.0f" % rpm)
	print("Gear: %d" % current_gear)
	print("Throttle: %.2f" % throttle_input)
	print("Brake: %.2f" % brake_input)
	print("Steering: %.2f" % steering_input)
	print("Handbrake: %.2f" % handbrake_input)
	print("Downforce: %.2f N" % aerodynamic_downforce)
	print("Drag: %.2f N" % aerodynamic_drag)
	print("Drift Active: %s" % ("Yes" if drift_active else "No"))
	print("Brake Temp: %.1f C" % brake_temperature)
	print("Brake Efficiency: %.2f" % brake_efficiency_current)

func _exit_tree() -> void:
	pass
