extends Resource
class_name PhysicsSettings

## PhysicsSettings - Centralized physics constants and configuration for the racing simulator
## All physics values are defined here so they can be tweaked without touching simulation code
## This resource provides a single source of truth for vehicle dynamics tuning
## Copyright 2026 Thalamus Racing Simulator Project

@export_group("Global Physics Constants")
@export var gravity: float = 9.81: set = _set_gravity
@export var physics_tick_rate: int = 120: set = _set_physics_tick_rate
@export var max_substeps: int = 4: set = _set_max_substeps
@export var time_scale: float = 1.0: set = _set_time_scale

@export_group("Vehicle Physics Defaults")
@export var default_vehicle_mass: float = 1500.0: set = _set_default_vehicle_mass
@export var default_wheel_radius: float = 0.32: set = _set_default_wheel_radius
@export var default_wheel_width: float = 0.22: set = _set_default_wheel_width
@export var wheel_track_front: float = 1.55: set = _set_wheel_track_front
@export var wheel_track_rear: float = 1.55: set = _set_wheel_track_rear
@export var wheelbase: float = 2.75: set = _set_wheelbase
@export var center_of_mass_offset_x: float = 0.0: set = _set_center_of_mass_offset_x
@export var center_of_mass_offset_y: float = 0.45: set = _set_center_of_mass_offset_y
@export var center_of_mass_offset_z: float = 0.0: set = _set_center_of_mass_offset_z
@export var roll_inertia: float = 1200.0: set = _set_roll_inertia
@export var pitch_inertia: float = 800.0: set = _set_pitch_inertia
@export var yaw_inertia: float = 2000.0: set = _set_yaw_inertia

@export_group("Engine & Powertrain")
@export var engine_max_rpm: float = 8000.0: set = _set_engine_max_rpm
@export var engine_min_rpm: float = 800.0: set = _set_engine_min_rpm
@export var idle_rpm: float = 900.0: set = _set_idle_rpm
@export var peak_torque_rpm: float = 4500.0: set = _set_peak_torque_rpm
@export var peak_torque_nm: float = 450.0: set = _set_peak_torque_nm
@export var max_power_kw: float = 250.0: set = _set_max_power_kw
@export var torque_curve_points: Array[Vector2] = [
	Vector2(0.15, 0.3), Vector2(0.35, 0.7), Vector2(0.55, 1.0), 
	Vector2(0.75, 0.95), Vector2(0.90, 0.85), Vector2(1.0, 0.75)
]
@export var num_gears: int = 6: set = _set_num_gears
@export var gear_ratios: Array[float] = [3.8, 2.1, 1.4, 1.0, 0.8, 0.65]
@export var final_drive_ratio: float = 3.73: set = _set_final_drive_ratio
@export var clutch_flywheel_inertia: float = 0.8: set = _set_clutch_flywheel_inertia
@export var rev_limit_rpm: float = 8200.0: set = _set_rev_limit_rpm
@export var fuel_consumption_rate: float = 0.00008: set = _set_fuel_consumption_rate

@export_group("Differential Settings")
@export var diff_type: int = 1: set = _set_diff_type
enum DifferentialType {
	OPEN, LOCKED, LIMITED_SLIP, TORQUE_BIAS
}
@export var limited_slip_locking_coefficient: float = 0.65: set = _set_limited_slip_locking_coefficient
@export var torque_bias_ratio: float = 2.5: set = _set_torque_bias_ratio
@export var pre_load_torque: float = 150.0: set = _set_pre_load_torque

@export_group("Suspension System")
@export var suspension_stiffness_front: float = 45000.0: set = _set_suspension_stiffness_front
@export var suspension_stiffness_rear: float = 52000.0: set = _set_suspension_stiffness_rear
@export var suspension_damping_compression: float = 8500.0: set = _set_suspension_damping_compression
@export var suspension_damping_rebound: float = 4500.0: set = _set_suspension_damping_rebound
@export var suspension_travel_max: float = 0.12: set = _set_suspension_travel_max
@export var suspension_travel_min: float = 0.02: set = _set_suspension_travel_min
@export var anti_roll_bar_stiffness_front: float = 1800.0: set = _set_anti_roll_bar_stiffness_front
@export var anti_roll_bar_stiffness_rear: float = 2200.0: set = _set_anti_roll_bar_stiffness_rear
@export var camber_gain_per_travel: float = 0.015: set = _set_camber_gain_per_travel
@export var toe_change_per_travel: float = 0.008: set = _set_toe_change_per_travel
@export var ride_height_static: float = 0.25: set = _set_ride_height_static
@export var ride_height_loaded: float = 0.22: set = _set_ride_height_loaded

@export_group("Tire & Friction Model")
@export var tire_stiffness_factor: float = 1.0: set = _set_tire_stiffness_factor
@export var friction_peak_coef: float = 1.35: set = _set_friction_peak_coef
@export var friction_slide_coef: float = 0.85: set = _set_friction_slide_coef
@export var slip_angle_peak: float = 4.5: set = _set_slip_angle_peak
@export var slip_ratio_peak: float = 0.12: set = _set_slip_ratio_peak
@export var lateral_friction_exponent: float = 2.0: set = _set_lateral_friction_exponent
@export var longitudinal_friction_exponent: float = 2.0: set = _set_longitudinal_friction_exponent
@export var camber_stiffness_factor: float = 0.15: set = _set_camber_stiffness_factor
@export var load_sensitivity: float = 0.15: set = _set_load_sensitivity
@export var tire_temp_optimal: float = 80.0: set = _set_tire_temp_optimal
@export var tire_temp_min: float = -20.0: set = _set_tire_temp_min
@export var tire_temp_max: float = 120.0: set = _set_tire_temp_max
@export var tire_wear_rate: float = 0.0001: set = _set_tire_wear_rate
@export var grip_loss_on_slip: float = 0.3: set = _set_grip_loss_on_slip
@export var road_friction_base: float = 1.0: set = _set_road_friction_base
@export var road_friction_asphalt: float = 1.0
@export var road_friction_concrete: float = 0.95
@export var road_friction_gravel: float = 0.75
@export var road_friction_mud: float = 0.55
@export var road_friction_wet: float = 0.70
@export var road_friction_ice: float = 0.15

@export_group("Aerodynamics")
@export var drag_coefficient: float = 0.32: set = _set_drag_coefficient
@export var lift_coefficient: float = -0.15: set = _set_lift_coefficient
@export var front_downforce_coefficient: float = -0.08: set = _set_front_downforce_coefficient
@export var rear_downforce_coefficient: float = -0.12: set = _set_rear_downforce_coefficient
@export var aero_reference_area: float = 2.1: set = _set_aero_reference_area
@export var aero_center_of_pressure_x: float = 0.0: set = _set_aero_center_of_pressure_x
@export var aero_center_of_pressure_y: float = 0.35: set = _set_aero_center_of_pressure_y
@export var aero_center_of_pressure_z: float = 0.0: set = _set_aero_center_of_pressure_z
@export var wing_angle_front_deg: float = 5.0: set = _set_wing_angle_front_deg
@export var wing_angle_rear_deg: float = 15.0: set = _set_wing_angle_rear_deg
@export var air_density_sea_level: float = 1.225: set = _set_air_density_sea_level
@export var drag_divergence_speed: float = 60.0: set = _set_drag_divergence_speed

@export_group("Steering System")
@export var steering_ratio: float = 14.5: set = _set_steering_ratio
@export var steering_lock_left: float = 2.35: set = _set_steering_lock_left
@export var steering_lock_right: float = 2.35: set = _set_steering_lock_right
@export var steering_max_input: float = 1.0: set = _set_steering_max_input
@export var steering_deadzone: float = 0.02: set = _set_steering_deadzone
@export var steering_return_speed: float = 3.0: set = _set_steering_return_speed
@export var steering_force_feedback_gain: float = 1.0: set = _set_steering_force_feedback_gain
@export var power_assist_ratio: float = 15.0: set = _set_power_assist_ratio
@export var power_assist_speed_curve: Array[Vector2] = [
	Vector2(0.0, 1.0), Vector2(30.0, 0.7), Vector2(60.0, 0.4), Vector2(100.0, 0.2)
]

@export_group("Braking System")
@export var brake_force_per_pedal: float = 5500.0: set = _set_brake_force_per_pedal
@export var brake_bias_front: float = 0.58: set = _set_brake_bias_front
@export var brake_bias_rear: float = 0.42: set = _set_brake_bias_rear
@export var abs_threshold: float = 0.25: set = _set_abs_threshold
@export var abs_recovery_rate: float = 15.0: set = _set_abs_recovery_rate
@export var abs_activation_speed: float = 5.0: set = _set_abs_activation_speed
@export var parking_brake_force: float = 3000.0: set = _set_parking_brake_force
@export var brake_pad_wear_rate: float = 0.00005: set = _set_brake_pad_wear_rate
@export var brake_disc_capacity: float = 0.5: set = _set_brake_disc_capacity
@export var brake_temperature_max: float = 650.0: set = _set_brake_temperature_max
@export var brake_fade_start_temp: float = 400.0: set = _set_brake_fade_start_temp
@export var brake_fade_multiplier: float = 0.6: set = _set_brake_fade_multiplier

@export_group("Transmission & Clutch")
@export var clutch_slip_threshold: float = 0.15: set = _set_clutch_slip_threshold
@export var clutch_engagement_ramp: float = 0.1: set = _set_clutch_engagement_ramp
@export var clutch_disengagement_ramp: float = 0.05: set = _set_clutch_disengagement_ramp
@export var upshift_delay_ms: int = 150: set = _set_upshift_delay_ms
@export var downshift_delay_ms: int = 100: set = _set_downshift_delay_ms
@export var auto_shift_enabled: bool = false: set = _set_auto_shift_enabled
@export var auto_shift_rpm_buffer: float = 300.0: set = _set_auto_shift_rpm_buffer
@export var launch_control_enabled: bool = false: set = _set_launch_control_enabled
@export var launch_control_target_rpm: float = 3500.0: set = _set_launch_control_target_rpm

@export_group("Damage & Wear")
@export var damage_threshold_impact: float = 15.0: set = _set_damage_threshold_impact
@export var damage_threshold_deformation: float = 0.15: set = _set_damage_threshold_deformation
@export var deformation_recovery_rate: float = 0.01: set = _set_deformation_recovery_rate
@export var max_deformation_percent: float = 0.45: set = _set_max_deformation_percent
@export var crash_sound_volume: float = 1.0: set = _set_crash_sound_volume
@export var debris_spawn_chance: float = 0.3: set = _set_debris_spawn_chance

@export_group("Simulation Accuracy")
@export var use_exact_substeps: bool = true: set = _set_use_exact_substeps
@export var collision_precision: float = 0.001: set = _set_collision_precision
@export var contact_tolerance: float = 0.002: set = _set_contact_tolerance
@export var sleep_threshold_velocity: float = 0.1: set = _set_sleep_threshold_velocity
@export var sleep_threshold_rotation: float = 0.05: set = _set_sleep_threshold_rotation
@export var min_simulation_step: float = 0.0001: set = _set_min_simulation_step
@export var max_simulation_step: float = 0.0166667: set = _set_max_simulation_step

var _last_valid_gravity: float = 9.81
var _last_valid_tick_rate: int = 120
var _last_valid_mass: float = 1500.0

func _validate_property(info: Dictionary) -> void:
	if info.name == "torque_curve_points":
		info.usage = PROPERTY_USAGE_SCRIPT_VARIABLE | PROPERTY_USAGE_ARRAY
	elif info.name == "gear_ratios":
		info.usage = PROPERTY_USAGE_SCRIPT_VARIABLE | PROPERTY_USAGE_ARRAY

func get_torque_at_rpm(rpm: float) -> float:
	var normalized = clamp((rpm - engine_min_rpm) / (engine_max_rpm - engine_min_rpm), 0.0, 1.0)
	for i in range(torque_curve_points.size() - 1):
		var t0 = torque_curve_points[i].x
		var t1 = torque_curve_points[i + 1].x
		if normalized >= t0 and normalized <= t1:
			var alpha = (normalized - t0) / (t1 - t0)
			var y0 = torque_curve_points[i].y
			var y1 = torque_curve_points[i + 1].y
			return lerp(y0, y1, alpha) * peak_torque_nm
	return peak_torque_nm * 0.5

func get_power_at_rpm(rpm: float) -> float:
	var torque = get_torque_at_rpm(rpm)
	return (torque * rpm * 2.0 * PI) / 60000.0

func get_drag_force(speed: float) -> float:
	return 0.5 * air_density_sea_level * drag_coefficient * aero_reference_area * speed * speed

func get_downforce(speed: float) -> float:
	var total_cd = lift_coefficient + front_downforce_coefficient + rear_downforce_coefficient
	return 0.5 * air_density_sea_level * total_cd * aero_reference_area * speed * speed

func get_suspension_force(displacement: float, velocity: float) -> float:
	var compression = displacement < 0.0
	var stiffness = suspension_stiffness_front if compression else suspension_stiffness_rear
	var damping = suspension_damping_compression if compression else suspension_damping_rebound
	var force = -stiffness * displacement - damping * velocity
	return clamp(force, -suspension_travel_max * stiffness, suspension_travel_max * stiffness)

func calculate_friction_normal(load: float) -> float:
	var effective_coef = friction_peak_coef * (1.0 - load_sensitivity * load / default_vehicle_mass)
	return effective_coef * load

func calculate_friction_slip(slip_ratio: float, normal_load: float) -> float:
	var slip_normalized = abs(slip_ratio) / slip_ratio_peak
	var friction_coef = friction_slide_coef + (friction_peak_coef - friction_slide_coef) * exp(-slip_normalized)
	return friction_coef * normal_load

func apply_temporary_modifications(modifiers: Dictionary) -> void:
	for key in modifiers:
		match key:
			"gravity": gravity = modifiers[key]
			"vehicle_mass": default_vehicle_mass = modifiers[key]
			"drag_coefficient": drag_coefficient = modifiers[key]
			"brake_bias": brake_bias_front = modifiers[key]; brake_bias_rear = 1.0 - modifiers[key]
			"suspension_stiffness": suspension_stiffness_front = modifiers[key]
			"tire_friction": friction_peak_coef = modifiers[key]

func reset_to_defaults() -> void:
	_last_valid_gravity = 9.81
	_last_valid_tick_rate = 120
	_last_valid_mass = 1500.0
	# Reset all exports to their default values would require manual assignment
	# or using @export hint ranges as defaults

func _set_gravity(value: float) -> void:
	gravity = value
	_last_valid_gravity = value

func _set_physics_tick_rate(value: int) -> void:
	physics_tick_rate = value
	_last_valid_tick_rate = value

func _set_max_substeps(value: int) -> void:
	max_substeps = value

func _set_time_scale(value: float) -> void:
	time_scale = clamp(value, 0.0, 2.0)

func _set_default_vehicle_mass(value: float) -> void:
	default_vehicle_mass = value
	_last_valid_mass = value

func _set_default_wheel_radius(value: float) -> void:
	wheel_radius = value

func _set_default_wheel_width(value: float) -> void:
	wheel_width = value

func _set_wheel_track_front(value: float) -> void:
	wheel_track_front = value

func _set_wheel_track_rear(value: float) -> void:
	wheel_track_rear = value

func _set_wheelbase(value: float) -> void:
	wheelbase = value

func _set_center_of_mass_offset_x(value: float) -> void:
	center_of_mass_offset_x = value

func _set_center_of_mass_offset_y(value: float) -> void:
	center_of_mass_offset_y = value

func _set_center_of_mass_offset_z(value: float) -> void:
	center_of_mass_offset_z = value

func _set_roll_inertia(value: float) -> void:
	roll_inertia = value

func _set_pitch_inertia(value: float) -> void:
	pitch_inertia = value

func _set_yaw_inertia(value: float) -> void:
	yaw_inertia = value

func _set_engine_max_rpm(value: float) -> void:
	engine_max_rpm = value

func _set_engine_min_rpm(value: float) -> void:
	engine_min_rpm = value

func _set_idle_rpm(value: float) -> void:
	idle_rpm = value

func _set_peak_torque_rpm(value: float) -> void:
	peak_torque_rpm = value

func _set_peak_torque_nm(value: float) -> void:
	peak_torque_nm = value

func _set_max_power_kw(value: float) -> void:
	max_power_kw = value

func _set_num_gears(value: int) -> void:
	num_gears = clampi(value, 4, 10)

func _set_final_drive_ratio(value: float) -> void:
	final_drive_ratio = value

func _set_clutch_flywheel_inertia(value: float) -> void:
	clutch_flywheel_inertia = value

func _set_rev_limit_rpm(value: float) -> void:
	rev_limit_rpm = value

func _set_fuel_consumption_rate(value: float) -> void:
	fuel_consumption_rate = value

func _set_diff_type(value: int) -> void:
	diff_type = clampi(value, 0, 3)

func _set_limited_slip_locking_coefficient(value: float) -> void:
	limited_slip_locking_coefficient = clampf(value, 0.0, 1.0)

func _set_torque_bias_ratio(value: float) -> void:
	torque_bias_ratio = value

func _set_pre_load_torque(value: float) -> void:
	pre_load_torque = value

func _set_suspension_stiffness_front(value: float) -> void:
	suspension_stiffness_front = value

func _set_suspension_stiffness_rear(value: float) -> void:
	suspension_stiffness_rear = value

func _set_suspension_damping_compression(value: float) -> void:
	suspension_damping_compression = value

func _set_suspension_damping_rebound(value: float) -> void:
	suspension_damping_rebound = value

func _set_suspension_travel_max(value: float) -> void:
	suspension_travel_max = value

func _set_suspension_travel_min(value: float) -> void:
	suspension_travel_min = value

func _set_anti_roll_bar_stiffness_front(value: float) -> void:
	anti_roll_bar_stiffness_front = value

func _set_anti_roll_bar_stiffness_rear(value: float) -> void:
	anti_roll_bar_stiffness_rear = value

func _set_camber_gain_per_travel(value: float) -> void:
	camber_gain_per_travel = value

func _set_toe_change_per_travel(value: float) -> void:
	toe_change_per_travel = value

func _set_ride_height_static(value: float) -> void:
	ride_height_static = value

func _set_ride_height_loaded(value: float) -> void:
	ride_height_loaded = value

func _set_tire_stiffness_factor(value: float) -> void:
	tire_stiffness_factor = value

func _set_friction_peak_coef(value: float) -> void:
	friction_peak_coef = value

func _set_friction_slide_coef(value: float) -> void:
	friction_slide_coef = value

func _set_slip_angle_peak(value: float) -> void:
	slip_angle_peak = value

func _set_slip_ratio_peak(value: float) -> void:
	slip_ratio_peak = value

func _set_lateral_friction_exponent(value: float) -> void:
	lateral_friction_exponent = value

func _set_longitudinal_friction_exponent(value: float) -> void:
	longitudinal_friction_exponent = value

func _set_camber_stiffness_factor(value: float) -> void:
	camber_stiffness_factor = value

func _set_load_sensitivity(value: float) -> void:
	load_sensitivity = value

func _set_tire_temp_optimal(value: float) -> void:
	tire_temp_optimal = value

func _set_tire_temp_min(value: float) -> void:
	tire_temp_min = value

func _set_tire_temp_max(value: float) -> void:
	tire_temp_max = value

func _set_tire_wear_rate(value: float) -> void:
	tire_wear_rate = value

func _set_grip_loss_on_slip(value: float) -> void:
	grip_loss_on_slip = value

func _set_road_friction_base(value: float) -> void:
	road_friction_base = value

func _set_drag_coefficient(value: float) -> void:
	drag_coefficient = value

func _set_lift_coefficient(value: float) -> void:
	lift_coefficient = value

func _set_front_downforce_coefficient(value: float) -> void:
	front_downforce_coefficient = value

func _set_rear_downforce_coefficient(value: float) -> void:
	rear_downforce_coefficient = value

func _set_aero_reference_area(value: float) -> void:
	aero_reference_area = value

func _set_aero_center_of_pressure_x(value: float) -> void:
	aero_center_of_pressure_x = value

func _set_aero_center_of_pressure_y(value: float) -> void:
	aero_center_of_pressure_y = value

func _set_aero_center_of_pressure_z(value: float) -> void:
	aero_center_of_pressure_z = value

func _set_wing_angle_front_deg(value: float) -> void:
	wing_angle_front_deg = value

func _set_wing_angle_rear_deg(value: float) -> void:
	wing_angle_rear_deg = value

func _set_air_density_sea_level(value: float) -> void:
	air_density_sea_level = value

func _set_drag_divergence_speed(value: float) -> void:
	drag_divergence_speed = value

func _set_steering_ratio(value: float) -> void:
	steering_ratio = value

func _set_steering_lock_left(value: float) -> void:
	steering_lock_left = value

func _set_steering_lock_right(value: float) -> void:
	steering_lock_right = value

func _set_steering_max_input(value: float) -> void:
	steering_max_input = value

func _set_steering_deadzone(value: float) -> void:
	steering_deadzone = value

func _set_steering_return_speed(value: float) -> void:
	steering_return_speed = value

func _set_steering_force_feedback_gain(value: float) -> void:
	steering_force_feedback_gain = value

func _set_power_assist_ratio(value: float) -> void:
	power_assist_ratio = value

func _set_auto_shift_enabled(value: bool) -> void:
	auto_shift_enabled = value

func _set_auto_shift_rpm_buffer(value: float) -> void:
	auto_shift_rpm_buffer = value

func _set_launch_control_enabled(value: bool) -> void:
	launch_control_enabled = value

func _set_launch_control_target_rpm(value: float) -> void:
	launch_control_target_rpm = value

func _set_brake_force_per_pedal(value: float) -> void:
	brake_force_per_pedal = value

func _set_brake_bias_front(value: float) -> void:
	brake_bias_front = clampf(value, 0.3, 0.7)
	brake_bias_rear = 1.0 - brake_bias_front

func _set_brake_bias_rear(value: float) -> void:
	brake_bias_rear = clampf(value, 0.3, 0.7)
	brake_bias_front = 1.0 - brake_bias_rear

func _set_abs_threshold(value: float) -> void:
	abs_threshold = value

func _set_abs_recovery_rate(value: float) -> void:
	abs_recovery_rate = value

func _set_abs_activation_speed(value: float) -> void:
	abs_activation_speed = value

func _set_parking_brake_force(value: float) -> void:
	parking_brake_force = value

func _set_brake_pad_wear_rate(value: float) -> void:
	brake_pad_wear_rate = value

func _set_brake_disc_capacity(value: float) -> void:
	brake_disc_capacity = value

func _set_brake_temperature_max(value: float) -> void:
	brake_temperature_max = value

func _set_brake_fade_start_temp(value: float) -> void:
	brake_fade_start_temp = value

func _set_brake_fade_multiplier(value: float) -> void:
	brake_fade_multiplier = value

func _set_clutch_slip_threshold(value: float) -> void:
	clutch_slip_threshold = value

func _set_clutch_engagement_ramp(value: float) -> void:
	clutch_engagement_ramp = value

func _set_clutch_disengagement_ramp(value: float) -> void:
	clutch_disengagement_ramp = value

func _set_upshift_delay_ms(value: int) -> void:
	upshift_delay_ms = value

func _set_downshift_delay_ms(value: int) -> void:
	downshift_delay_ms = value

func _set_damage_threshold_impact(value: float) -> void:
	damage_threshold_impact = value

func _set_damage_threshold_deformation(value: float) -> void:
	damage_threshold_deformation = value

func _set_deformation_recovery_rate(value: float) -> void:
	deformation_recovery_rate = value

func _set_max_deformation_percent(value: float) -> void:
	max_deformation_percent = value

func _set_crash_sound_volume(value: float) -> void:
	crash_sound_volume = value

func _set_debris_spawn_chance(value: float) -> void:
	debris_spawn_chance = value

func _set_use_exact_substeps(value: bool) -> void:
	use_exact_substeps = value

func _set_collision_precision(value: float) -> void:
	collision_precision = value

func _set_contact_tolerance(value: float) -> void:
	contact_tolerance = value

func _set_sleep_threshold_velocity(value: float) -> void:
	sleep_threshold_velocity = value

func _set_sleep_threshold_rotation(value: float) -> void:
	sleep_threshold_rotation = value

func _set_min_simulation_step(value: float) -> void:
	min_simulation_step = value

func _set_max_simulation_step(value: float) -> void:
	max_simulation_step = value

func _set_auto_shift_enabled(value: bool) -> void:
	auto_shift_enabled = value

func _set_launch_control_enabled(value: bool) -> void:
	launch_control_enabled = value

func kmh_to_ms(kmh: float) -> float:
	return kmh / 3.6

func ms_to_kmh(ms: float) -> float:
	return ms * 3.6

func rad_to_deg(rad: float) -> float:
	return rad * 57.2958

func deg_to_rad(deg: float) -> float:
	return deg * 0.0174533

func Nm_to_lb_ft(nm: float) -> float:
	return nm * 0.737562

def lb_ft_to_Nm(lb_ft: float) -> float:
	return lb_ft * 1.35582

func hp_to_kw(hp: float) -> float:
	return hp * 0.7457

func kw_to_hp(kw: float) -> float:
	return kw * 1.34102

func psi_to_bar(psi: float) -> float:
	return psi * 0.0689476

func bar_to_psi(bar: float) -> float:
	return bar * 14.5038

func inches_to_mm(inches: float) -> float:
	return inches * 25.4

func mm_to_inches(mm: float) -> float:
	return mm * 0.0393701

func kg_to_lbs(kg: float) -> float:
	return kg * 2.20462

func lbs_to_kg(lbs: float) -> float:
	return lbs * 0.453592

func get_current_gear_ratio(gear_index: int) -> float:
	if gear_index < 0 or gear_index >= num_gears:
		return 1.0
	return gear_ratios[gear_index] * final_drive_ratio

func get_optimal_shift_point(current_rpm: float) -> float:
	return rev_limit_rpm - auto_shift_rpm_buffer

func should_auto_shift(current_rpm: float, current_gear: int, target_gear: int) -> bool:
	if not auto_shift_enabled:
		return false
	var shift_point = get_optimal_shift_point(current_rpm)
	return current_rpm >= shift_point and current_gear > target_gear

func calculate_brake_force(total_brake_input: float) -> Dictionary:
	var front_force = total_brake_input * brake_force_per_pedal * brake_bias_front
	var rear_force = total_brake_input * brake_force_per_pedal * brake_bias_rear
	return {"front": front_force, "rear": rear_force}

func calculate_aero_downforce(speed_ms: float) -> Dictionary:
	var dynamic_pressure = 0.5 * air_density_sea_level * speed_ms * speed_ms
	var front_downforce = front_downforce_coefficient * aero_reference_area * dynamic_pressure
	var rear_downforce = rear_downforce_coefficient * aero_reference_area * dynamic_pressure
	return {"total": front_downforce + rear_downforce, "front": front_downforce, "rear": rear_downforce}

func get_road_friction(surface_type: String) -> float:
	match surface_type:
		"asphalt": return road_friction_asphalt
		"concrete": return road_friction_concrete
		"gravel": return road_friction_gravel
		"mud": return road_friction_mud
		"wet": return road_friction_wet
		"ice": return road_friction_ice
		_: return road_friction_base

func clone_with_modifications(modifiers: Dictionary) -> PhysicsSettings:
	var cloned = duplicate()
	for key in modifiers:
		cloned.apply_temporary_modifications({key: modifiers[key]})
	return cloned

func save_settings_to_file(file_path: String) -> Error:
	var file = FileAccess.open(file_path, FileAccess.WRITE)
	if file == null:
		return ERR_CANT_CREATE
	var settings_dict: Dictionary = {
		"gravity": gravity,
		"vehicle_mass": default_vehicle_mass,
		"drag_coefficient": drag_coefficient,
		"lift_coefficient": lift_coefficient,
		"brake_bias_front": brake_bias_front,
		"suspension_stiffness_front": suspension_stiffness_front,
		"suspension_stiffness_rear": suspension_stiffness_rear,
		"tire_friction_peak": friction_peak_coef,
		"tire_friction_slide": friction_slide_coef,
		"aero_drag": drag_coefficient,
		"aero_downforce": lift_coefficient,
		"gear_ratios": gear_ratios,
		"final_drive": final_drive_ratio
	}
	file.store_string(JSON.stringify(settings_dict))
	file.close()
	return OK

func load_settings_from_file(file_path: String) -> Error:
	var file = FileAccess.open(file_path, FileAccess.READ)
	if file == null:
		return ERR_FILE_NOT_FOUND
	var json_string = file.get_as_text()
	file.close()
	var parser = JSON.new()
	var error = parser.parse(json_string)
	if error != OK:
		return error
	var settings = parser.data as Dictionary
	if settings != null:
		if settings.has("gravity"): gravity = settings["gravity"]
		if settings.has("vehicle_mass"): default_vehicle_mass = settings["vehicle_mass"]
		if settings.has("drag_coefficient"): drag_coefficient = settings["drag_coefficient"]
		if settings.has("lift_coefficient"): lift_coefficient = settings["lift_coefficient"]
		if settings.has("brake_bias_front"): brake_bias_front = settings["brake_bias_front"]
		if settings.has("suspension_stiffness_front"): suspension_stiffness_front = settings["suspension_stiffness_front"]
		if settings.has("suspension_stiffness_rear"): suspension_stiffness_rear = settings["suspension_stiffness_rear"]
		if settings.has("tire_friction_peak"): friction_peak_coef = settings["tire_friction_peak"]
		if settings.has("tire_friction_slide"): friction_slide_coef = settings["tire_friction_slide"]
		if settings.has("gear_ratios") and typeof(settings["gear_ratios"]) == TYPE_ARRAY:
			gear_ratios = []
			for ratio in settings["gear_ratios"]:
				gear_ratios.append(ratio)
		if settings.has("final_drive"): final_drive_ratio = settings["final_drive"]
	return OK