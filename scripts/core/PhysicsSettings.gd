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
	Vector2(0.75, 0.95), Vector2(0.95, 0.8)]
@export var power_curve_points: Array[Vector2] = [
	Vector2(0.15, 0.1), Vector2(0.35, 0.5), Vector2(0.55, 1.0), 
	Vector2(0.75, 1.2), Vector2(0.95, 1.1)]
@export var transmission_type: TransmissionType = TransmissionType.MANUAL
@export var final_drive_ratio: float = 3.73: set = _set_final_drive_ratio
@export var clutch_slip_angle: float = 0.08: set = _set_clutch_slip_angle
@export var flywheel_inertia: float = 0.8: set = _set_flywheel_inertia
@export var drivetrain_efficiency: float = 0.85: set = _set_drivetrain_efficiency

@export_group("Gear Ratios")
@export var first_gear_ratio: float = 3.65: set = _set_first_gear_ratio
@export var second_gear_ratio: float = 2.15: set = _set_second_gear_ratio
@export var third_gear_ratio: float = 1.55: set = _set_third_gear_ratio
@export var fourth_gear_ratio: float = 1.15: set = _set_fourth_gear_ratio
@export var fifth_gear_ratio: float = 0.95: set = _set_fifth_gear_ratio
@export var sixth_gear_ratio: float = 0.78: set = _set_sixth_gear_ratio
@export var reverse_gear_ratio: float = 3.85: set = _set_reverse_gear_ratio
@export var neutral_gear_ratio: float = 0.0: set = _set_neutral_gear_ratio
var _gear_ratios: Dictionary = {
	"1st": first_gear_ratio, "2nd": second_gear_ratio, "3rd": third_gear_ratio,
	"4th": fourth_gear_ratio, "5th": fifth_gear_ratio, "6th": sixth_gear_ratio,
	"R": reverse_gear_ratio, "N": neutral_gear_ratio
}

@export_group("Differential Settings")
@export var differential_type: DifferentialType = DifferentialType.LSD
@export var differential_lock_preload: float = 15.0: set = _set_differential_lock_preload
@export var differential_acceleration_bias: float = 0.35: set = _set_differential_acceleration_bias
@export var differential_deceleration_bias: float = 0.25: set = _set_differential_deceleration_bias
@export var max_differential_lock_percentage: float = 0.95: set = _set_max_differential_lock_percentage
@export var diff_friction_coefficient: float = 0.02: set = _set_diff_friction_coefficient

@export_group("Suspension Parameters")
@export_group("Front Suspension")
@export var front_spring_stiffness: float = 45000.0: set = _set_front_spring_stiffness
@export var front_spring_preload: float = 150.0: set = _set_front_spring_preload
@export var front_damping_compression: float = 8500.0: set = _set_front_damping_compression
@export var front_damping_rebound: float = 12000.0: set = _set_front_damping_rebound
@export var front_travel_limit_pos: float = 0.15: set = _set_front_travel_limit_pos
@export var front_travel_limit_neg: float = 0.12: set = _set_front_travel_limit_neg
@export var front_bump_stop_stiffness: float = 250000.0: set = _set_front_bump_stop_stiffness
@export var front_bump_stop_trigger: float = 0.13: set = _set_front_bump_stop_trigger

@export_group("Rear Suspension")
@export var rear_spring_stiffness: float = 52000.0: set = _set_rear_spring_stiffness
@export var rear_spring_preload: float = 180.0: set = _set_rear_spring_preload
@export var rear_damping_compression: float = 9500.0: set = _set_rear_damping_compression
@export var rear_damping_rebound: float = 13500.0: set = _set_rear_damping_rebound
@export var rear_travel_limit_pos: float = 0.16: set = _set_rear_travel_limit_pos
@export var rear_travel_limit_neg: float = 0.13: set = _set_rear_travel_limit_neg
@export var rear_bump_stop_stiffness: float = 280000.0: set = _set_rear_bump_stop_stiffness
@export var rear_bump_stop_trigger: float = 0.14: set = _set_rear_bump_stop_trigger

@export_group("Anti-Roll Bars")
@export var front_anti_roll_bar_stiffness: float = 18000.0: set = _set_front_anti_roll_bar_stiffness
@export var rear_anti_roll_bar_stiffness: float = 15000.0: set = _set_rear_anti_roll_bar_stiffness

@export_group("Tire Physics - Pacejka Magic Formula")
@export_group("Longitudinal Friction")
@export var tire_long_B: float = 11.0: set = _set_tire_long_B
@export var tire_long_C: float = 1.8: set = _set_tire_long_C
@export var tire_long_D: float = 1.2: set = _set_tire_long_D
@export var tire_long_E: float = -0.3: set = _set_tire_long_E
@export var tire_long_peak_mu: float = 1.2: set = _set_tire_long_peak_mu

@export_group("Lateral Friction")
@export var tire_lat_B: float = 10.0: set = _set_tire_lat_B
@export var tire_lat_C: float = 1.9: set = _set_tire_lat_C
@export var tire_lat_D: float = 1.15: set = _set_tire_lat_D
@export var tire_lat_E: float = -0.2: set = _set_tire_lat_E
@export var tire_lat_peak_mu: float = 1.15: set = _set_tire_lat_peak_mu

@export_group("Combined Friction")
@export var tire_friction_ellipse_exponent: float = 1.5: set = _set_tire_friction_ellipse_exponent
@export var tire_friction_transition_factor: float = 0.7: set = _set_tire_friction_transition_factor

@export_group("Tire Construction Properties")
@export var tire_vertical_stiffness: float = 320000.0: set = _set_tire_vertical_stiffness
@export var tire_hysteresis_loss: float = 0.05: set = _set_tire_hysteresis_loss
@export var tire_contact_patch_length: float = 0.12: set = _set_tire_contact_patch_length
@export var tire_contact_patch_width: float = 0.20: set = _set_tire_contact_patch_width
@export var tire_pressure_pa: float = 210000.0: set = _set_tire_pressure_pa
@export var tire_temperature_optimal: float = 80.0: set = _set_tire_temperature_optimal
@export var tire_temperature_min: float = -10.0: set = _set_tire_temperature_min
@export var tire_temperature_max: float = 140.0: set = _set_tire_temperature_max

@export_group("Aerodynamics")
@export var aero_drag_coefficient: float = 0.32: set = _set_aero_drag_coefficient
@export var aero_frontal_area: float = 2.1: set = _set_aero_frontal_area
@export var aero_lift_coefficient: float = -0.15: set = _set_aero_lift_coefficient
@export var aero_downforce_coefficient: float = 0.45: set = _set_aero_downforce_coefficient
@export var aero_center_of_pressure_x: float = 0.35: set = _set_aero_center_of_pressure_x
@export var aero_center_of_pressure_y: float = 0.0: set = _set_aero_center_of_pressure_y
@export var aero_center_of_pressure_z: float = 0.4: set = _set_aero_center_of_pressure_z
@export var aero_side_force_coefficient: float = 0.08: set = _set_aero_side_force_coefficient
@export var aero_yaw_moment_coefficient: float = 0.02: set = _set_aero_yaw_moment_coefficient
@export var aero_ground_effect_enabled: bool = true: set = _set_aero_ground_effect_enabled
@export var aero_ground_clearance: float = 0.08: set = _set_aero_ground_clearance

@export_group("Steering System")
@export var steering_ratio: float = 14.0: set = _set_steering_ratio
@export var steering_max_angle_deg: float = 45.0: set = _set_steering_max_angle_deg
@export var steering_max_angle_rad: float = 0.785: set = _set_steering_max_angle_rad
@export var steering_response_delay: float = 0.05: set = _set_steering_response_delay
@export var steering_self_aligning_torque_gain: float = 1.0: set = _set_steering_self_aligning_torque_gain
@export var steering_feedback_strength: float = 0.75: set = _set_steering_feedback_strength

@export_group("Braking System")
@export var brake_force_distribution_front: float = 0.65: set = _set_brake_force_distribution_front
@export var brake_force_distribution_rear: float = 0.35: set = _set_brake_force_distribution_rear
@export var brake_max_pressure_bar: float = 150.0: set = _set_brake_max_pressure_bar
@export var brake_caliper_piston_area_cm2: float = 12.0: set = _set_brake_caliper_piston_area_cm2
@export var brake_disc_rotor_diameter_mm: float = 330.0: set = _set_brake_disc_rotor_diameter_mm
@export var brake_pad_friction_coefficient: float = 0.45: set = _set_brake_pad_friction_coefficient
@export var brake_bleed_time_s: float = 0.1: set = _set_brake_bleed_time_s
@export var brake_hot_fade_threshold_c: float = 400.0: set = _set_brake_hot_fade_threshold_c
@export var abs_threshold_speed_ms: float = 2.0: set = _set_abs_threshold_speed_ms
@export var abs_modulation_rate_hz: float = 20.0: set = _set_abs_modulation_rate_hz
@export var abs_enable: bool = true: set = _set_abs_enable

@export_group("Transmission Behavior")
@export var upshift_rpm_threshold: float = 7500.0: set = _set_upshift_rpm_threshold
@export var downshift_rpm_threshold: float = 2000.0: set = _set_downshift_rpm_threshold
@export var rev_matching_enabled: bool = true: set = _set_rev_matching_enabled
@export var rev_matching_tolerance_rpm: float = 150.0: set = _set_rev_matching_tolerance_rpm
@export var auto_blip_enabled: bool = false: set = _set_auto_blip_enabled
@export var throttle_drop_on_shift_percent: float = 0.15: set = _set_throttle_drop_on_shift_percent

@export_group("Race Rules")
@export var track_surface_friction_asphalt: float = 0.95: set = _set_track_surface_friction_asphalt
@export var track_surface_friction_gravel: float = 0.65: set = _set_track_surface_friction_gravel
@export var track_surface_friction_grass: float = 0.45: set = _set_track_surface_friction_grass
@export var track_surface_friction_wet: float = 0.6: set = _set_track_surface_friction_wet
@export var track_surface_friction_ice: float = 0.15: set = _set_track_surface_friction_ice
@export var wind_resistance_enabled: bool = true: set = _set_wind_resistance_enabled
@export var wind_variation_enabled: bool = true: set = _set_wind_variation_enabled
@export var weather_effects_enabled: bool = true: set = _set_weather_effects_enabled

enum TransmissionType { MANUAL, AUTOMATIC, SEMI_AUTOMATIC, CVT }
enum DifferentialType { OPEN, LOCKING, LSD }

## Helper Methods for Unit Conversions

func rpm_to_rads_per_sec(rpm: float) -> float:
	return (rpm * PI) / 30.0

func rads_per_sec_to_rpm(rads_per_sec: float) -> float:
	return (rads_per_sec * 30.0) / PI

func kmh_to_ms(kmh: float) -> float:
	return kmh / 3.6

func ms_to_kmh(ms: float) -> float:
	return ms * 3.6

func deg_to_rad(degrees: float) -> float:
	return degrees * PI / 180.0

func rad_to_deg(rad: float) -> float:
	return rad * 180.0 / PI

func nm_to_lb_ft(nm: float) -> float:
	return nm * 0.737562

func lb_ft_to_nm(lb_ft: float) -> float:
	return lb_ft / 0.737562

def kw_to_hp(kw: float) -> float:
	return kw * 1.34102

def hp_to_kw(hp: float) -> float:
	return hp / 1.34102

func bar_to_pa(bar: float) -> float:
	return bar * 100000.0

func pa_to_bar(pa: float) -> float:
	return pa / 100000.0

func mm_to_m(mm: float) -> float:
	return mm / 1000.0

func cm2_to_m2(cm2: float) -> float:
	return cm2 * 0.0001

func get_gear_ratio(gear: int) -> float:
	match gear:
		1: return first_gear_ratio
		2: return second_gear_ratio
		3: return third_gear_ratio
		4: return fourth_gear_ratio
		5: return fifth_gear_ratio
		6: return sixth_gear_ratio
		-1: return reverse_gear_ratio
		0: return neutral_gear_ratio
		_: return first_gear_ratio

func get_total_drive_ratio(gear: int) -> float:
	return get_gear_ratio(gear) * final_drive_ratio

func calculate_engine_torque_at_rpm(rpm: float) -> float:
	var normalized_rpm = clamp((rpm - engine_min_rpm) / (engine_max_rpm - engine_min_rpm), 0.0, 1.0)
	var interpolated_torque = interpolate_torque_curve(normalized_rpm)
	return peak_torque_nm * interpolated_torque

func calculate_engine_power_at_rpm(rpm: float) -> float:
	var torque = calculate_engine_torque_at_rpm(rpm)
	var angular_velocity = rpm_to_rads_per_sec(rpm)
	var power_watts = torque * angular_velocity
	return power_watts / 1000.0

func calculate_aero_drag(speed_ms: float) -> float:
	var air_density = 1.225
	return 0.5 * air_density * aero_drag_coefficient * aero_frontal_area * speed_ms * speed_ms

func calculate_aero_downforce(speed_ms: float) -> float:
	var air_density = 1.225
	return 0.5 * air_density * aero_downforce_coefficient * aero_frontal_area * speed_ms * speed_ms

func calculate_tire_friction_longitudinal(slips: float, normal_load: float, slip_angle: float = 0.0) -> float:
	var alpha = abs(slip_angle)
	var combined_slip = sqrt(slips * slips + tan(alpha) * tan(alpha))
	var mu = normalize_tire_friction(combined_slip, normal_load, true)
	return mu * sign(slips)

func calculate_tire_friction_lateral(slip_angle: float, normal_load: float, longitudinal_slip: float = 0.0) -> float:
	var combined_slip = sqrt(longitudinal_slip * longitudinal_slip + tan(slip_angle) * tan(slip_angle))
	var mu = normalize_tire_friction(combined_slip, normal_load, false)
	return mu * sign(slip_angle)

func normalize_tire_friction(combined_slip: float, normal_load: float, is_longitudinal: bool) -> float:
	var params = tire_long if is_longitudinal else tire_lat
	var B = params.B
	var C = params.C
	var D = params.D
	var E = params.E
	
	var shape_factor = B * C
	var sin_arg = shape_factor * combined_slip
	var curve = D * sin(sin_arg)
	
	if E < 0:
		curve -= E * (sin_arg - atan(sin_arg))
	
	return curve

func interpolate_torque_curve(normalized_rpm: float) -> float:
	if normalized_rpm <= 0.0: return torque_curve_points[0].y
	if normalized_rpm >= 1.0: return torque_curve_points[-1].y
	
	for i in range(torque_curve_points.size() - 1):
		var p1 = torque_curve_points[i]
		var p2 = torque_curve_points[i + 1]
		
		if normalized_rpm >= p1.x and normalized_rpm <= p2.x:
			var t = (normalized_rpm - p1.x) / (p2.x - p1.x)
			return lerp(p1.y, p2.y, t)
	
	return torque_curve_points[-1].y

func _set_gravity(value: float) -> void:
	if value > 0.0:
		gravity = value
		emit_changed()

func _set_physics_tick_rate(value: int) -> void:
	if value > 0 and value <= 240:
		physics_tick_rate = value
		emit_changed()

func _set_max_substeps(value: int) -> void:
	if value > 0 and value <= 8:
		max_substeps = value
		emit_changed()

func _set_time_scale(value: float) -> void:
	if value > 0.0 and value <= 2.0:
		time_scale = value
		emit_changed()

func _set_default_vehicle_mass(value: float) -> void:
	if value > 500.0 and value < 10000.0:
		default_vehicle_mass = value
		emit_changed()

func _set_default_wheel_radius(value: float) -> void:
	if value > 0.1 and value < 1.0:
		default_wheel_radius = value
		emit_changed()

func _set_default_wheel_width(value: float) -> void:
	if value > 0.1 and value < 0.5:
		default_wheel_width = value
		emit_changed()

func _set_wheel_track_front(value: float) -> void:
	if value > 0.5 and value < 3.0:
		wheel_track_front = value
		emit_changed()

func _set_wheel_track_rear(value: float) -> void:
	if value > 0.5 and value < 3.0:
		wheel_track_rear = value
		emit_changed()

func _set_wheelbase(value: float) -> void:
	if value > 1.5 and value < 5.0:
		wheelbase = value
		emit_changed()

func _set_center_of_mass_offset_x(value: float) -> void:
	if value > -1.0 and value < 1.0:
		center_of_mass_offset_x = value
		emit_changed()

func _set_center_of_mass_offset_y(value: float) -> void:
	if value > 0.1 and value < 1.5:
		center_of_mass_offset_y = value
		emit_changed()

func _set_center_of_mass_offset_z(value: float) -> void:
	if value > -1.0 and value < 1.0:
		center_of_mass_offset_z = value
		emit_changed()

func _set_roll_inertia(value: float) -> void:
	if value > 100.0 and value < 10000.0:
		roll_inertia = value
		emit_changed()

func _set_pitch_inertia(value: float) -> void:
	if value > 100.0 and value < 10000.0:
		pitch_inertia = value
		emit_changed()

func _set_yaw_inertia(value: float) -> void:
	if value > 100.0 and value < 10000.0:
		yaw_inertia = value
		emit_changed()

func _set_engine_max_rpm(value: float) -> void:
	if value > 2000.0 and value < 15000.0:
		engine_max_rpm = value
		emit_changed()

func _set_engine_min_rpm(value: float) -> void:
	if value > 100.0 and value < 1000.0:
		engine_min_rpm = value
		emit_changed()

func _set_idle_rpm(value: float) -> void:
	if value > 500.0 and value < 2000.0:
		idle_rpm = value
		emit_changed()

func _set_peak_torque_rpm(value: float) -> void:
	if value > 1000.0 and value < 8000.0:
		peak_torque_rpm = value
		emit_changed()

func _set_peak_torque_nm(value: float) -> void:
	if value > 50.0 and value < 2000.0:
		peak_torque_nm = value
		emit_changed()

func _set_max_power_kw(value: float) -> void:
	if value > 10.0 and value < 2000.0:
		max_power_kw = value
		emit_changed()

func _set_final_drive_ratio(value: float) -> void:
	if value > 1.0 and value < 10.0:
		final_drive_ratio = value
		emit_changed()

func _set_clutch_slip_angle(value: float) -> void:
	if value > 0.01 and value < 0.5:
		clutch_slip_angle = value
		emit_changed()

func _set_flywheel_inertia(value: float) -> void:
	if value > 0.1 and value < 5.0:
		flywheel_inertia = value
		emit_changed()

func _set_drivetrain_efficiency(value: float) -> void:
	if value > 0.5 and value < 1.0:
		drivetrain_efficiency = value
		emit_changed()

func _set_first_gear_ratio(value: float) -> void:
	if value > 1.0 and value < 10.0:
		first_gear_ratio = value
		emit_changed()

func _set_second_gear_ratio(value: float) -> void:
	if value > 1.0 and value < 10.0:
		second_gear_ratio = value
		emit_changed()

func _set_third_gear_ratio(value: float) -> void:
	if value > 1.0 and value < 10.0:
		third_gear_ratio = value
		emit_changed()

func _set_fourth_gear_ratio(value: float) -> void:
	if value > 0.5 and value < 5.0:
		fourth_gear_ratio = value
		emit_changed()

func _set_fifth_gear_ratio(value: float) -> void:
	if value > 0.5 and value < 5.0:
		fifth_gear_ratio = value
		emit_changed()

func _set_sixth_gear_ratio(value: float) -> void:
	if value > 0.5 and value < 5.0:
		sixth_gear_ratio = value
		emit_changed()

func _set_reverse_gear_ratio(value: float) -> void:
	if value > 1.0 and value < 10.0:
		reverse_gear_ratio = value
		emit_changed()

func _set_neutral_gear_ratio(value: float) -> void:
	if value == 0.0:
		neutral_gear_ratio = value
		emit_changed()

func _set_differential_lock_preload(value: float) -> void:
	if value >= 0.0 and value <= 100.0:
		differential_lock_preload = value
		emit_changed()

func _set_differential_acceleration_bias(value: float) -> void:
	if value >= 0.0 and value <= 1.0:
		differential_acceleration_bias = value
		emit_changed()

func _set_differential_deceleration_bias(value: float) -> void:
	if value >= 0.0 and value <= 1.0:
		differential_deceleration_bias = value
		emit_changed()

func _set_max_differential_lock_percentage(value: float) -> void:
	if value >= 0.0 and value <= 1.0:
		max_differential_lock_percentage = value
		emit_changed()

func _set_diff_friction_coefficient(value: float) -> void:
	if value >= 0.0 and value <= 1.0:
		diff_friction_coefficient = value
		emit_changed()

func _set_front_spring_stiffness(value: float) -> void:
	if value > 10000.0 and value < 200000.0:
		front_spring_stiffness = value
		emit_changed()

func _set_front_spring_preload(value: float) -> void:
	if value >= 0.0 and value < 300.0:
		front_spring_preload = value
		emit_changed()

func _set_front_damping_compression(value: float) -> void:
	if value > 1000.0 and value < 50000.0:
		front_damping_compression = value
		emit_changed()

func _set_front_damping_rebound(value: float) -> void:
	if value > 1000.0 and value < 50000.0:
		front_damping_rebound = value
		emit_changed()

func _set_front_travel_limit_pos(value: float) -> void:
	if value > 0.05 and value < 0.3:
		front_travel_limit_pos = value
		emit_changed()

func _set_front_travel_limit_neg(value: float) -> void:
	if value > 0.05 and value < 0.3:
		front_travel_limit_neg = value
		emit_changed()

func _set_front_bump_stop_stiffness(value: float) -> void:
	if value > 50000.0 and value < 1000000.0:
		front_bump_stop_stiffness = value
		emit_changed()

func _set_front_bump_stop_trigger(value: float) -> void:
	if value > 0.0 and value < 0.2:
		front_bump_stop_trigger = value
		emit_changed()

func _set_rear_spring_stiffness(value: float) -> void:
	if value > 10000.0 and value < 200000.0:
		rear_spring_stiffness = value
		emit_changed()

func _set_rear_spring_preload(value: float) -> void:
	if value >= 0.0 and value < 300.0:
		rear_spring_preload = value
		emit_changed()

func _set_rear_damping_compression(value: float) -> void:
	if value > 1000.0 and value < 50000.0:
		rear_damping_compression = value
		emit_changed()

func _set_rear_damping_rebound(value: float) -> void:
	if value > 1000.0 and value < 50000.0:
		rear_damping_rebound = value
		emit_changed()

func _set_rear_travel_limit_pos(value: float) -> void:
	if value > 0.05 and value < 0.3:
		rear_travel_limit_pos = value
		emit_changed()

func _set_rear_travel_limit_neg(value: float) -> void:
	if value > 0.05 and value < 0.3:
		rear_travel_limit_neg = value
		emit_changed()

func _set_rear_bump_stop_stiffness(value: float) -> void:
	if value > 50000.0 and value < 1000000.0:
		rear_bump_stop_stiffness = value
		emit_changed()

func _set_rear_bump_stop_trigger(value: float) -> void:
	if value > 0.0 and value < 0.2:
		rear_bump_stop_trigger = value
		emit_changed()

func _set_front_anti_roll_bar_stiffness(value: float) -> void:
	if value > 1000.0 and value < 100000.0:
		front_anti_roll_bar_stiffness = value
		emit_changed()

func _set_rear_anti_roll_bar_stiffness(value: float) -> void:
	if value > 1000.0 and value < 100000.0:
		rear_anti_roll_bar_stiffness = value
		emit_changed()

func _set_tire_long_B(value: float) -> void:
	if value > 5.0 and value < 20.0:
		tire_long_B = value
		emit_changed()

func _set_tire_long_C(value: float) -> void:
	if value > 1.0 and value < 3.0:
		tire_long_C = value
		emit_changed()

func _set_tire_long_D(value: float) -> void:
	if value > 0.5 and value < 2.0:
		tire_long_D = value
		emit_changed()

func _set_tire_long_E(value: float) -> void:
	if value > -1.0 and value < 0.0:
		tire_long_E = value
		emit_changed()

func _set_tire_long_peak_mu(value: float) -> void:
	if value > 0.5 and value < 2.0:
		tire_long_peak_mu = value
		emit_changed()

func _set_tire_lat_B(value: float) -> void:
	if value > 5.0 and value < 20.0:
		tire_lat_B = value
		emit_changed()

func _set_tire_lat_C(value: float) -> void:
	if value > 1.0 and value < 3.0:
		tire_lat_C = value
		emit_changed()

func _set_tire_lat_D(value: float) -> void:
	if value > 0.5 and value < 2.0:
		tire_lat_D = value
		emit_changed()

func _set_tire_lat_E(value: float) -> void:
	if value > -1.0 and value < 0.0:
		tire_lat_E = value
		emit_changed()

func _set_tire_lat_peak_mu(value: float) -> void:
	if value > 0.5 and value < 2.0:
		tire_lat_peak_mu = value
		emit_changed()

func _set_tire_friction_ellipse_exponent(value: float) -> void:
	if value > 1.0 and value < 3.0:
		tire_friction_ellipse_exponent = value
		emit_changed()

func _set_tire_friction_transition_factor(value: float) -> void:
	if value > 0.0 and value < 1.0:
		tire_friction_transition_factor = value
		emit_changed()

func _set_tire_vertical_stiffness(value: float) -> void:
	if value > 50000.0 and value < 1000000.0:
		tire_vertical_stiffness = value
		emit_changed()

func _set_tire_hysteresis_loss(value: float) -> void:
	if value >= 0.0 and value < 0.2:
		tire_hysteresis_loss = value
		emit_changed()

func _set_tire_contact_patch_length(value: float) -> void:
	if value > 0.05 and value < 0.3:
		tire_contact_patch_length = value
		emit_changed()

func _set_tire_contact_patch_width(value: float) -> void:
	if value > 0.05 and value < 0.3:
		tire_contact_patch_width = value
		emit_changed()

func _set_tire_pressure_pa(value: float) -> void:
	if value > 100000.0 and value < 400000.0:
		tire_pressure_pa = value
		emit_changed()

func _set_tire_temperature_optimal(value: float) -> void:
	if value > 0.0 and value < 200.0:
		tire_temperature_optimal = value
		emit_changed()

func _set_tire_temperature_min(value: float) -> void:
	if value > -50.0 and value < 0.0:
		tire_temperature_min = value
		emit_changed()

func _set_tire_temperature_max(value: float) -> void:
	if value > 100.0 and value < 300.0:
		tire_temperature_max = value
		emit_changed()

func _set_aero_drag_coefficient(value: float) -> void:
	if value > 0.1 and value < 1.5:
		aero_drag_coefficient = value
		emit_changed()

func _set_aero_frontal_area(value: float) -> void:
	if value > 1.0 and value < 5.0:
		aero_frontal_area = value
		emit_changed()

func _set_aero_lift_coefficient(value: float) -> void:
	if value > -1.0 and value < 1.0:
		aero_lift_coefficient = value
		emit_changed()

func _set_aero_downforce_coefficient(value: float) -> void:
	if value > 0.0 and value < 2.0:
		aero_downforce_coefficient = value
		emit_changed()

func _set_aero_center_of_pressure_x(value: float) -> void:
	if value > 0.0 and value < 1.0:
		aero_center_of_pressure_x = value
		emit_changed()

func _set_aero_center_of_pressure_y(value: float) -> void:
	if value > -1.0 and value < 1.0:
		aero_center_of_pressure_y = value
		emit_changed()

func _set_aero_center_of_pressure_z(value: float) -> void:
	if value > 0.0 and value < 2.0:
		aero_center_of_pressure_z = value
		emit_changed()

func _set_aero_side_force_coefficient(value: float) -> void:
	if value > -0.5 and value < 0.5:
		aero_side_force_coefficient = value
		emit_changed()

func _set_aero_yaw_moment_coefficient(value: float) -> void:
	if value > -0.5 and value < 0.5:
		aero_yaw_moment_coefficient = value
		emit_changed()

func _set_aero_ground_effect_enabled(value: bool) -> void:
	aero_ground_effect_enabled = value
	emit_changed()

func _set_aero_ground_clearance(value: float) -> void:
	if value > 0.0 and value < 0.5:
		aero_ground_clearance = value
		emit_changed()

func _set_steering_ratio(value: float) -> void:
	if value > 8.0 and value < 25.0:
		steering_ratio = value
		emit_changed()

func _set_steering_max_angle_deg(value: float) -> void:
	if value > 10.0 and value < 90.0:
		steering_max_angle_deg = value
		emit_changed()

func _set_steering_max_angle_rad(value: float) -> void:
	if value > 0.1 and value < 2.0:
		steering_max_angle_rad = value
		emit_changed()

func _set_steering_response_delay(value: float) -> void:
	if value >= 0.0 and value < 0.5:
		steering_response_delay = value
		emit_changed()

func _set_steering_self_aligning_torque_gain(value: float) -> void:
	if value > 0.0 and value < 3.0:
		steering_self_aligning_torque_gain = value
		emit_changed()

func _set_steering_feedback_strength(value: float) -> void:
	if value >= 0.0 and value <= 1.0:
		steering_feedback_strength = value
		emit_changed()

func _set_brake_force_distribution_front(value: float) -> void:
	if value > 0.3 and value < 0.9:
		brake_force_distribution_front = value
		emit_changed()

func _set_brake_force_distribution_rear(value: float) -> void:
	if value > 0.1 and value < 0.7:
		brake_force_distribution_rear = value
		emit_changed()

func _set_brake_max_pressure_bar(value: float) -> void:
	if value > 50.0 and value < 300.0:
		brake_max_pressure_bar = value
		emit_changed()

func _set_brake_caliper_piston_area_cm2(value: float) -> void:
	if value > 2.0 and value < 50.0:
		brake_caliper_piston_area_cm2 = value
		emit_changed()

func _set_brake_disc_rotor_diameter_mm(value: float) -> void:
	if value > 150.0 and value < 500.0:
		brake_disc_rotor_diameter_mm = value
		emit_changed()

func _set_brake_pad_friction_coefficient(value: float) -> void:
	if value > 0.1 and value < 1.0:
		brake_pad_friction_coefficient = value
		emit_changed()

func _set_brake_bleed_time_s(value: float) -> void:
	if value > 0.01 and value < 1.0:
		brake_bleed_time_s = value
		emit_changed()

func _set_brake_hot_fade_threshold_c(value: float) -> void:
	if value > 100.0 and value < 800.0:
		brake_hot_fade_threshold_c = value
		emit_changed()

func _set_abs_threshold_speed_ms(value: float) -> void:
	if value > 0.0 and value < 10.0:
		abs_threshold_speed_ms = value
		emit_changed()

func _set_abs_modulation_rate_hz(value: float) -> void:
	if value > 5.0 and value < 50.0:
		abs_modulation_rate_hz = value
		emit_changed()

func _set_abs_enable(value: bool) -> void:
	abs_enable = value
	emit_changed()

func _set_upshift_rpm_threshold(value: float) -> void:
	if value > 3000.0 and value < 12000.0:
		upshift_rpm_threshold = value
		emit_changed()

func _set_downshift_rpm_threshold(value: float) -> void:
	if value > 500.0 and value < 5000.0:
		downshift_rpm_threshold = value
		emit_changed()

func _set_rev_matching_enabled(value: bool) -> void:
	rev_matching_enabled = value
	emit_changed()

func _set_rev_matching_tolerance_rpm(value: float) -> void:
	if value > 0.0 and value < 1000.0:
		rev_matching_tolerance_rpm = value
		emit_changed()

func _set_auto_blip_enabled(value: bool) -> void:
	auto_blip_enabled = value
	emit_changed()

func _set_throttle_drop_on_shift_percent(value: float) -> void:
	if value >= 0.0 and value <= 1.0:
		throttle_drop_on_shift_percent = value
		emit_changed()

func _set_track_surface_friction_asphalt(value: float) -> void:
	if value > 0.1 and value < 1.5:
		track_surface_friction_asphalt = value
		emit_changed()

func _set_track_surface_friction_gravel(value: float) -> void:
	if value > 0.1 and value < 1.5:
		track_surface_friction_gravel = value
		emit_changed()

func _set_track_surface_friction_grass(value: float) -> void:
	if value > 0.1 and value < 1.5:
		track_surface_friction_grass = value
		emit_changed()

func _set_track_surface_friction_wet(value: float) -> void:
	if value > 0.1 and value < 1.5:
		track_surface_friction_wet = value
		emit_changed()

func _set_track_surface_friction_ice(value: float) -> void:
	if value > 0.0 and value < 0.5:
		track_surface_friction_ice = value
		emit_changed()

func _set_wind_resistance_enabled(value: bool) -> void:
	wind_resistance_enabled = value
	emit_changed()

func _set_wind_variation_enabled(value: bool) -> void:
	wind_variation_enabled = value
	emit_changed()

func _set_weather_effects_enabled(value: bool) -> void:
	weather_effects_enabled = value
	emit_changed()

func clone_with_custom_values(customizations: Dictionary) -> PhysicsSettings:
	var new_settings = PhysicsSettings.new()
	new_settings.gravity = customizations.get("gravity", gravity)
	new_settings.default_vehicle_mass = customizations.get("vehicle_mass", default_vehicle_mass)
	new_settings.engine_max_rpm = customizations.get("engine_max_rpm", engine_max_rpm)
	new_settings.aero_drag_coefficient = customizations.get("aero_drag", aero_drag_coefficient)
	return new_settings

func to_dictionary() -> Dictionary:
	return {
		"gravity": gravity,
		"physics_tick_rate": physics_tick_rate,
		"max_substeps": max_substeps,
		"time_scale": time_scale,
		"default_vehicle_mass": default_vehicle_mass,
		"default_wheel_radius": default_wheel_radius,
		"default_wheel_width": default_wheel_width,
		"wheel_track_front": wheel_track_front,
		"wheel_track_rear": wheel_track_rear,
		"wheelbase": wheelbase,
		"center_of_mass_offset_x": center_of_mass_offset_x,
		"center_of_mass_offset_y": center_of_mass_offset_y,
		"center_of_mass_offset_z": center_of_mass_offset_z,
		"roll_inertia": roll_inertia,
		"pitch_inertia": pitch_inertia,
		"yaw_inertia": yaw_inertia,
		"engine_max_rpm": engine_max_rpm,
		"engine_min_rpm": engine_min_rpm,
		"idle_rpm": idle_rpm,
		"peak_torque_rpm": peak_torque_rpm,
		"peak_torque_nm": peak_torque_nm,
		"max_power_kw": max_power_kw,
		"torque_curve_points": torque_curve_points,
		"power_curve_points": power_curve_points,
		"transmission_type": transmission_type,
		"final_drive_ratio": final_drive_ratio,
		"first_gear_ratio": first_gear_ratio,
		"second_gear_ratio": second_gear_ratio,
		"third_gear_ratio": third_gear_ratio,
		"fourth_gear_ratio": fourth_gear_ratio,
		"fifth_gear_ratio": fifth_gear_ratio,
		"sixth_gear_ratio": sixth_gear_ratio,
		"reverse_gear_ratio": reverse_gear_ratio,
		"differential_type": differential_type,
		"differential_lock_preload": differential_lock_preload,
		"front_spring_stiffness": front_spring_stiffness,
		"rear_spring_stiffness": rear_spring_stiffness,
		"aero_drag_coefficient": aero_drag_coefficient,
		"aero_downforce_coefficient": aero_downforce_coefficient,
		"steering_max_angle_rad": steering_max_angle_rad,
		"brake_force_distribution_front": brake_force_distribution_front,
		"tire_long_peak_mu": tire_long_peak_mu,
		"tire_lat_peak_mu": tire_lat_peak_mu
	}

func _to_string() -> String:
	return "PhysicsSettings(gravity=%.2f, mass=%.0f, max_rpm=%.0f, aero_drag=%.2f)" % [
		gravity, default_vehicle_mass, engine_max_rpm, aero_drag_coefficient
	]