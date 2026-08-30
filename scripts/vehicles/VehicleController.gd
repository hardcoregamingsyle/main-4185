extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulation
## Handles throttle, brake, steering, wheel forces, gear shifting, and vehicle dynamics
## Integrates with PhysicsSettings for centralized tuning
## Copyright 2026 Thalamus Racing Simulator Project

# Signals for external communication
signal speed_changed(current_speed: float)
signal rpm_changed(current_rpm: float)
signal gear_changed(new_gear: int)
signal drift_started(intensity: float)
signal drift_ended()
signal collision_detected(impact_force: float, collision_point: Vector3)
signal engine_tuned(tune_parameters: Dictionary)

@export_group("Vehicle Configuration")
@export var vehicle_mass: float = 1500.0: set = _set_vehicle_mass
@export var center_of_mass_offset: Vector3 = Vector3(0.0, 0.5, 0.0): set = _set_center_of_mass_offset
@export var wheel_base: float = 2.7: set = _set_wheel_base
@export var track_width: float = 1.6: set = _set_track_width
@export var suspension_travel: float = 0.2: set = _set_suspension_travel
@export var drag_coefficient: float = 0.32: set = _set_drag_coefficient
@export var frontal_area: float = 2.2: set = _set_frontal_area
@export var tire_friction_coefficient: float = 1.2: set = _set_tire_friction_coefficient
@export var max_steering_angle: float = 35.0: set = _set_max_steering_angle

@export_group("Transmission Settings")
@export var transmission_type: TransmissionType = TransmissionType.MANUAL
@export var gear_ratios: Array[float] = [3.8, 2.2, 1.5, 1.1, 0.9, 0.7, 0.5]: set = _set_gear_ratios
@export var final_drive_ratio: float = 3.5: set = _set_final_drive_ratio
@export var clutch_capacity: float = 1500.0: set = _set_clutch_capacity
@export var torque_converter_efficiency: float = 0.85: set = _set_torque_converter_efficiency

@export_group("Engine Characteristics")
@export var idle_rpm: float = 800.0: set = _set_idle_rpm
@export var redline_rpm: float = 7500.0: set = _set_redline_rpm
@export var peak_torque_rpm: float = 4500.0: set = _set_peak_torque_rpm
@export var max_torque: float = 500.0: set = _set_max_torque
@export var max_power: float = 300.0: set = _set_max_power
@export var throttle_response_curve: float = 0.75: set = _set_throttle_response_curve
@export var turbo_enabled: bool = false: set = _set_turbo_enabled
@export var turbo_boost_pressure: float = 1.2: set = _set_turbo_boost_pressure
@export var turbo_charge_capacity: float = 100.0: set = _set_turbo_charge_capacity

@export_group("Braking System")
@export var front_brake_bias: float = 0.6: set = _set_front_brake_bias
@export var rear_brake_bias: float = 0.4: set = _set_rear_brake_bias
@export var brake_force_multiplier: float = 8.0: set = _set_brake_force_multiplier
@export var abs_enabled: bool = true: set = _set_abs_enabled
@export var brake_distribution_mode: BrakeDistributionMode = BrakeDistributionMode.ELECTRONIC

@export_group("Drift Mechanics")
@export var handbrake_lock_threshold: float = 0.3: set = _set_handbrake_lock_threshold
@export var drift_tires_factor: float = 0.85: set = _set_drift_tires_factor
@export var drift_recovery_rate: float = 0.1: set = _set_drift_recovery_rate
@export var min_drift_speed: float = 20.0: set = _set_min_drift_speed

@export_group("Debug Options")
@export var debug_visualization: bool = false: set = _set_debug_visualization
@export var show_physics_debug: bool = false

enum TransmissionType {
	MANUAL,
	AUTOMATIC,
	SEMI_AUTOMATIC,
	CVT
}

enum BrakeDistributionMode {
	FIXED,
	ELECTRONIC,
	DYNAMIC
}

# Internal state tracking
var current_gear: int = 1
var target_gear: int = 1
var current_rpm: float = 0.0
var current_speed: float = 0.0
var wheel_speeds: Array[float] = [0.0, 0.0, 0.0, 0.0]
var throttle_input: float = 0.0
var brake_input: float = 0.0
var steering_input: float = 0.0
var handbrake_active: bool = false
var engine_on: bool = true
var clutch_engaged: bool = true

# Turbo system state
var turbo_charge: float = 0.0
var turbo_spool: float = 0.0

# Drift state
var drift_intensity: float = 0.0
var drift_angle: float = 0.0
var is_drifting: bool = false

# Suspension state
var wheel_compression: Array[float] = [0.0, 0.0, 0.0, 0.0]
var wheel_velocity: Array[float] = [0.0, 0.0, 0.0, 0.0]

# Physical constants
const WHEEL_RADIUS: float = 0.32
const GEAR_NEUTRAL: int = 0
const MAX_GEAR_INDEX: int = 6

# Physics references
var _physics_settings: PhysicsSettings = PhysicsSettings.new()

func _ready() -> void:
	_init_physics_settings()
	_reset_vehicle_state()
	set_process(true)
	set_physics_process(true)

func _init_physics_settings() -> void:
	_physics_settings.gravity = PhysicsSettings.gravity
	_physics_settings.physics_tick_rate = PhysicsSettings.physics_tick_rate
	_physics_settings.max_substeps = PhysicsSettings.max_substeps
	_physics_settings.time_scale = PhysicsSettings.time_scale

func _reset_vehicle_state() -> void:
	current_gear = 1
	target_gear = 1
	current_rpm = _physics_settings.default_vehicle_mass
	current_speed = 0.0
	wheel_speeds.fill(0.0)
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	handbrake_active = false
	engine_on = true
	clutch_engaged = true
	turbo_charge = 0.0
	turbo_spool = 0.0
	drift_intensity = 0.0
	drift_angle = 0.0
	is_drifting = false
	wheel_compression.fill(0.0)
	wheel_velocity.fill(0.0)

func _physics_process(delta: float) -> void:
	if not engine_on:
		return
	
	_update_physics(delta)
	_update_audio_signals()
	
	if debug_visualization or show_physics_debug:
		_draw_debug_visualization()

func _update_physics(delta: float) -> void:
	var dt = delta * _physics_settings.time_scale
	
	_calculate_engine_output()
	_apply_transmission()
	_handle_steering(dt)
	_handle_brakes(dt)
	_apply_forces_to_body(dt)
	_update_wheel_states(dt)
	_update_suspension(dt)
	_check_collision_state()

func _calculate_engine_output() -> void:
	var torque_output: float = _get_current_torque()
	current_rpm = _lerp_rpm_to_target(torque_output)
	
	if transmission_type == TransmissionType.CVT:
		_update_cvt_gearing()
	else:
		_auto_shift_gears()
	
	emit_signal(rpm_changed, current_rpm)

func _get_current_torque() -> float:
	var rpm_ratio: float = (current_rpm - idle_rpm) / (peak_torque_rpm - idle_rpm)
	rpm_ratio = clamp(rpm_ratio, 0.0, 1.0)
	
	var base_torque: float = max_torque * (1.0 - abs(rpm_ratio - 0.5) * 0.5)
	
	var turbo_multiplier: float = 1.0
	if turbo_enabled:
		turbo_multiplier = 1.0 + (turbo_boost_pressure - 1.0) * (turbo_charge / turbo_charge_capacity)
	
	var final_torque: float = base_torque * turbo_multiplier
	
	return final_torque * throttle_input

func _lerp_rpm_to_target(target_torque: float) -> float:
	var target_rpm: float = idle_rpm + (redline_rpm - idle_rpm) * throttle_input
	
	if clutch_engaged:
		return lerp(current_rpm, target_rpm, 0.1)
	else:
		return lerp(current_rpm, idle_rpm, 0.2)

func _apply_transmission() -> void:
	var gear_ratio: float = 1.0
	var wheel_torque: float = 0.0
	
	if current_gear > 0:
		gear_ratio = gear_ratios[current_gear - 1] * final_drive_ratio
		
		if clutch_engaged:
			wheel_torque = _get_current_torque() * gear_ratio * torque_converter_efficiency
		else:
			wheel_torque = 0.0
	
	_update_wheel_speeds_from_gear(wheel_torque)

func _handle_steering(delta: float) -> void:
	var desired_steering: float = steering_input * max_steering_angle
	var current_rotation: Node3D = get_node_or_null("../Mesh") as Node3D
	
	if current_rotation:
		current_rotation.rotation.y = lerp(
			current_rotation.rotation.y,
			deg_to_rad(desired_steering),
			0.1
		)

func _handle_brakes(delta: float) -> void:
	var braking_force: float = 0.0
	
	if brake_input > 0.0:
		var total_brake_force: float = brake_input * brake_force_multiplier
		var front_force: float = total_brake_force * front_brake_bias
		var rear_force: float = total_brake_force * rear_brake_bias
		
		if abs(current_speed) < 1.0:
			front_force = 0.0
			rear_force = 0.0
		
		braking_force = front_force + rear_force
	
	if handbrake_active:
		braking_force += handbrake_lock_threshold * brake_force_multiplier * 2.0
	
	_apply_braking_force(braking_force)

func _apply_forces_to_body(delta: float) -> void:
	var forward_vector: Vector3 = transform.basis.z
	var lateral_vector: Vector3 = transform.basis.x
	
	var drive_force: float = 0.0
	var brake_force: float = 0.0
	
	if clutch_engaged:
		var wheel_torque: float = _get_current_torque() * gear_ratios[current_gear - 1] * final_drive_ratio
		drive_force = wheel_torque / WHEEL_RADIUS
	
	var velocity_magnitude: float = linear_velocity.length()
	var air_resistance: float = 0.5 * drag_coefficient * frontal_area * velocity_magnitude * velocity_magnitude
	
	force = forward_vector * (drive_force - brake_force - air_resistance)

func _update_wheel_states(delta: float) -> void:
	for i in range(4):
		var wheel_radius_effective: float = WHEEL_RADIUS * (1.0 - wheel_compression[i])
		wheel_speeds[i] = current_speed / wheel_radius_effective

func _update_suspension(delta: float) -> void:
	for i in range(4):
		var compression_rate: float = 0.05
		var damping_rate: float = 0.1
		
		wheel_compression[i] = lerp(wheel_compression[i], 0.0, compression_rate)
		wheel_velocity[i] = lerp(wheel_velocity[i], 0.0, damping_rate)

func _check_collision_state() -> void:
	pass

func _auto_shift_gears() -> void:
	if transmission_type != TransmissionType.MANUAL:
		var shift_up_threshold: float = redline_rpm * 0.9
		var shift_down_threshold: float = idle_rpm * 1.5
		
		if current_rpm >= shift_up_threshold and current_gear < MAX_GEAR_INDEX:
			target_gear = current_gear + 1
		elif current_rpm <= shift_down_threshold and current_gear > 1:
			target_gear = current_gear - 1
		
		if target_gear != current_gear:
			perform_gear_shift(target_gear)

func perform_gear_shift(gear: int) -> void:
	if gear == current_gear:
		return
	
	if gear > 0:
		clutch_engaged = false
		await get_tree().create_timer(0.15).timeout
		current_gear = gear
		clutch_engaged = true
	else:
		current_gear = GEAR_NEUTRAL
	
	emit_signal(gear_changed, current_gear)

func update_cvt_gearing() -> void:
	if transmission_type == TransmissionType.CVT:
		var ratio: float = 1.0 + ((current_rpm - idle_rpm) / (redline_rpm - idle_rpm)) * 2.0
		gear_ratios[0] = ratio

func _update_cvt_gearing() -> void:
	update_cvt_gearing()

func _update_audio_signals() -> void:
	if AudioManager:
		AudioManager.play_sound("engine", current_rpm / redline_rpm)

func _draw_debug_visualization() -> void:
	if debug_visualization:
		draw_line(Vector3.ZERO, Vector3.FORWARD * 10.0, Color.GREEN)
		draw_line(Vector3.ZERO, Vector3.RIGHT * 10.0, Color.RED)
		draw_circle(Vector3.ZERO, 1.0, Color.BLUE)

func set_throttle(input_value: float) -> void:
	throttle_input = clamp(input_value, 0.0, 1.0)

func set_brake(input_value: float) -> void:
	brake_input = clamp(input_value, 0.0, 1.0)

func set_steering(input_value: float) -> void:
	steering_input = clamp(input_value, -1.0, 1.0)

func activate_handbrake(active: bool) -> void:
	handbrake_active = active

func toggle_engine(on: bool) -> void:
	engine_on = on
	if not on:
		current_rpm = idle_rpm

func start_drift() -> void:
	if current_speed > min_drift_speed:
		is_drifting = true
		drift_intensity = 1.0
		emit_signal(drift_started, 1.0)

func end_drift() -> void:
	is_drifting = false
	drift_intensity = 0.0
	emit_signal(drift_ended())

func apply_collision_impact(force: float, point: Vector3) -> void:
	collision_detected.emit(force, point)
	
	if force > 5000.0:
		engage_damage_system()

func engage_damage_system() -> void:
	if not engine_on:
		return
	
	var damage_torque: float = _get_current_torque() * 0.5
	_get_current_torque() = damage_torque

func _set_vehicle_mass(value: float) -> void:
	vehicle_mass = value
	mass = value

func _set_center_of_mass_offset(value: Vector3) -> void:
	center_of_mass_offset = value

func _set_wheel_base(value: float) -> void:
	wheel_base = value

func _set_track_width(value: float) -> void:
	track_width = value

func _set_suspension_travel(value: float) -> void:
	suspension_travel = value

func _set_drag_coefficient(value: float) -> void:
	drag_coefficient = value

func _set_frontal_area(value: float) -> void:
	frontal_area = value

func _set_tire_friction_coefficient(value: float) -> void:
	tire_friction_coefficient = value

func _set_max_steering_angle(value: float) -> void:
	max_steering_angle = value

func _set_gear_ratios(value: Array[float]) -> void:
	gear_ratios = value

func _set_final_drive_ratio(value: float) -> void:
	final_drive_ratio = value

func _set_clutch_capacity(value: float) -> void:
	clutch_capacity = value

func _set_torque_converter_efficiency(value: float) -> void:
	torque_converter_efficiency = value

func _set_idle_rpm(value: float) -> void:
	idle_rpm = value

func _set_redline_rpm(value: float) -> void:
	redline_rpm = value

func _set_peak_torque_rpm(value: float) -> void:
	peak_torque_rpm = value

func _set_max_torque(value: float) -> void:
	max_torque = value

func _set_max_power(value: float) -> void:
	max_power = value

func _set_throttle_response_curve(value: float) -> void:
	throttle_response_curve = value

func _set_turbo_enabled(value: bool) -> void:
	turbo_enabled = value

func _set_turbo_boost_pressure(value: float) -> void:
	turbo_boost_pressure = value

func _set_turbo_charge_capacity(value: float) -> void:
	turbo_charge_capacity = value

func _set_front_brake_bias(value: float) -> void:
	front_brake_bias = value

func _set_rear_brake_bias(value: float) -> void:
	rear_brake_bias = value

func _set_brake_force_multiplier(value: float) -> void:
	brake_force_multiplier = value

func _set_abs_enabled(value: bool) -> void:
	abs_enabled = value

func _set_brake_distribution_mode(value: BrakeDistributionMode) -> void:
	brake_distribution_mode = value

func _set_handbrake_lock_threshold(value: float) -> void:
	handbrake_lock_threshold = value

func _set_drift_tires_factor(value: float) -> void:
	drift_tires_factor = value

func _set_drift_recovery_rate(value: float) -> void:
	drift_recovery_rate = value

func _set_min_drift_speed(value: float) -> void:
	min_drift_speed = value

func _set_debug_visualization(value: bool) -> void:
	debug_visualization = value

func _set_show_physics_debug(value: bool) -> void:
	show_physics_debug = value

func reset_all_inputs() -> void:
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	handbrake_active = false

func get_wheel_contact_points() -> Array[Vector3]:
	var points: Array[Vector3] = []
	var offset: Vector3 = Vector3(track_width / 2.0, 0.0, 0.0)
	
	points.append(transform * (-offset + Vector3(0.0, 0.0, wheel_base / 2.0)))
	points.append(transform * (offset + Vector3(0.0, 0.0, wheel_base / 2.0)))
	points.append(transform * (-offset + Vector3(0.0, 0.0, -wheel_base / 2.0)))
	points.append(transform * (offset + Vector3(0.0, 0.0, -wheel_base / 2.0)))
	
	return points

func get_vehicle_state() -> Dictionary:
	return {
		"speed": current_speed,
		"rpm": current_rpm,
		"gear": current_gear,
		"throttle": throttle_input,
		"brake": brake_input,
		"steering": steering_input,
		"drift_intensity": drift_intensity,
		"turbo_charge": turbo_charge,
		"is_drifting": is_drifting
	}