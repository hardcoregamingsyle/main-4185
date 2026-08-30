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
@export var peak_power_rpm: float = 6500.0: set = _set_peak_power_rpm
@export var peak_power_kw: float = 300.0: set = _set_peak_power_kw
@export var torque_curve_slope_low: float = 0.8: set = _set_torque_curve_slope_low
@export var torque_curve_slope_high: float = 0.6: set = _set_torque_curve_slope_high
@export var throttle_response_delay: float = 0.1: set = _set_throttle_response_delay
@export var clutch_slip_angle: float = 2.5: set = _set_clutch_slip_angle
@export var transmission_efficiency: float = 0.92: set = _set_transmission_efficiency

@export_group("Gear Ratios")
@export var gear_count: int = 6: set = _set_gear_count
@export var final_drive_ratio: float = 3.45: set = _set_final_drive_ratio
@export var reverse_ratio: float = 3.8: set = _set_reverse_ratio
var _gear_ratios: Array[float] = [3.8, 2.2, 1.5, 1.15, 0.9, 0.75]
@export var gear_shift_delay: float = 0.15: set = _set_gear_shift_delay
@export var neutral_ratio: float = 0.0: set = _set_neutral_ratio

@export_group("Differential Settings")
@export var differential_type: DiffType = DiffType.LSD: set = _set_differential_type
@export var lsd_preload_torque: float = 150.0: set = _set_lsd_preload_torque
@export var lsd_max_lock_torque: float = 400.0: set = _set_lsd_max_lock_torque
@export var open_diff_accel_slip: float = 0.15: set = _set_open_diff_accel_slip
@export var open_diff_decel_slip: float = 0.1: set = _set_open_diff_decel_slip
@export var diff_brake_bias: float = 0.3: set = _set_diff_brake_bias

enum DiffType {
	OPEN,
	LSD,
	LOCKING,
	CVT
}

@export_group("Suspension Parameters")
@export var front_spring_stiffness: float = 45000.0: set = _set_front_spring_stiffness
@export var rear_spring_stiffness: float = 50000.0: set = _set_rear_spring_stiffness
@export var front_damping_compression: float = 8000.0: set = _set_front_damping_compression
@export var front_damping_rebound: float = 12000.0: set = _set_front_damping_rebound
@export var rear_damping_compression: float = 9000.0: set = _set_rear_damping_compression
@export var rear_damping_rebound: float = 14000.0: set = _set_rear_damping_rebound
@export var front_travel_limit: float = 0.15: set = _set_front_travel_limit
@export var rear_travel_limit: float = 0.18: set = _set_rear_travel_limit
@export var anti_roll_bar_stiffness_front: float = 15000.0: set = _set_anti_roll_bar_stiffness_front
@export var anti_roll_bar_stiffness_rear: float = 12000.0: set = _set_anti_roll_bar_stiffness_rear
@export var suspension_geometry_height: float = 0.35: set = _set_suspension_geometry_height
@export var camber_gain_per_travel: float = 0.02: set = _set_camber_gain_per_travel
@export var toe_gain_per_travel: float = 0.005: set = _set_toe_gain_per_travel

@export_group("Tire Friction Model")
@export var tire_friction_model: TireModel = TireModel.PACEJKA: set = _set_tire_friction_model
@export var pacejka_b_factor: float = 10.0: set = _set_pacejka_b_factor
@export var pacejka_c_factor: float = 1.9: set = _set_pacejka_c_factor
@export var pacejka_d_factor: float = 1.2: set = _set_pacejka_d_factor
@export var pacejka_e_factor: float = 0.95: set = _set_pacejka_e_factor
@export var tire_vertical_stiffness: float = 180000.0: set = _set_tire_vertical_stiffness
@export var tire_relaxation_length: float = 0.05: set = _set_tire_relaxation_length
@export var tire_temperature_optimal: float = 80.0: set = _set_tire_temperature_optimal
@export var tire_temp_min: float = -20.0: set = _set_tire_temp_min
@export var tire_temp_max: float = 120.0: set = _set_tire_temp_max
@export var tire_wear_rate: float = 0.001: set = _set_tire_wear_rate
@export var wet_surface_reduction: float = 0.65: set = _set_wet_surface_reduction
@export var gravel_surface_reduction: float = 0.5: set = _set_gravel_surface_reduction
@export var ice_surface_reduction: float = 0.2: set = _set_ice_surface_reduction

enum TireModel {
	SIMPLIFIED,
	PACEJKA,
	VEERLE
}

@export_group("Aerodynamics")
@export var drag_coefficient: float = 0.32: set = _set_drag_coefficient
@export var frontal_area: float = 2.1: set = _set_frontal_area
@export var downforce_coefficient: float = 0.45: set = _set_downforce_coefficient
@export var lift_coefficient: float = -0.05: set = _set_lift_coefficient
@export var aero_reference_speed: float = 50.0: set = _set_aero_reference_speed
@export var front_air_foil_angle: float = 5.0: set = _set_front_air_foil_angle
@export var rear_air_foil_angle: float = 15.0: set = _set_rear_air_foil_angle
@export var ground_effect_height: float = 0.05: set = _set_ground_effect_height
@export var aero_side_force_coefficient: float = 0.08: set = _set_aero_side_force_coefficient

@export_group("Steering System")
@export var steering_ratio: float = 14.0: set = _set_steering_ratio
@export var steering_lock_left: float = 2.8: set = _set_steering_lock_left
@export var steering_lock_right: float = 2.8: set = _set_steering_lock_right
@export var steering_deadzone: float = 0.02: set = _set_steering_deadzone
@export var steering_response_rate: float = 0.85: set = _set_steering_response_rate
@export var power_assist_ratio: float = 0.4: set = _set_power_assist_ratio
@export var steering_weight_at_100kmh: float = 1.5: set = _set_steering_weight_at_100kmh

@export_group("Braking System")
@export var brake_disc_diameter_front: float = 0.35: set = _set_brake_disc_diameter_front
@export var brake_disc_diameter_rear: float = 0.30: set = _set_brake_disc_diameter_rear
@export var brake_pad_friction: float = 0.45: set = _set_brake_pad_friction
@export var brake_caliper_piston_area: float = 0.0025: set = _set_brake_caliper_piston_area
@export var max_brake_pressure: float = 100.0: set = _set_max_brake_pressure
@export var brake_pressure_buildup_rate: float = 50.0: set = _set_brake_pressure_buildup_rate
@export var brake_pressure_release_rate: float = 80.0: set = _set_brake_pressure_release_rate
@export var brake_force_distribution_front: float = 0.6: set = _set_brake_force_distribution_front
@export var abs_threshold: float = 0.25: set = _set_abs_threshold
@export var ebd_active: bool = true: set = _set_ebd_active
@export var ebm_current: float = 1.0: set = _set_ebm_current

@export_group("Transmission Controls")
@export var auto_shift_rpm_threshold: float = 7800.0: set = _set_auto_shift_rpm_threshold
@export var auto_shift_delay: float = 0.3: set = _set_auto_shift_delay
@export var rev_matching_enabled: bool = true: set = _set_rev_matching_enabled
@export var rev_match_target_rpm: float = 6000.0: set = _set_rev_match_target_rpm

@export_group("Drivetrain Configuration")
@export var drivetrain_type: Drivetrain = Drivetrain.FWD: set = _set_drivetrain_type
@export var traction_control_level: int = 5: set = _set_traction_control_level
@export var stability_control_enabled: bool = true: set = _set_stability_control_enabled
@export var launch_control_enabled: bool = false: set = _set_launch_control_enabled
@export var launch_control_rpm: float = 5500.0: set = _set_launch_control_rpm

enum Drivetrain {
	FWD,
	RWD,
	AWD
}

@export_group("Simulation Quality")
@export var substep_collision_detection: bool = true: set = _set_substep_collision_detection
@export var raycast_precision: float = 0.01: set = _set_raycast_precision
@export var contact_point_tolerance: float = 0.005: set = _set_contact_point_tolerance
@export var sleep_threshold: float = 0.01: set = _set_sleep_threshold
@export var wake_threshold: float = 0.1: set = _set_wake_threshold

# Helper functions for unit conversions
func kmh_to_ms(kmh: float) -> float:
	return khm * (1000.0 / 3600.0)

func ms_to_kmh(ms: float) -> float:
	return ms * (3600.0 / 1000.0)

func rpm_to_rps(rpm: float) -> float:
	return rpm / 60.0

func rps_to_rpm(rps: float) -> float:
	return rps * 60.0

func degrees_to_radians(degrees: float) -> float:
	return deg_to_rad(degrees)

func radians_to_degrees(radians: float) -> float:
	return rad_to_deg(radians)

func nm_to_lb_ft(nm: float) -> float:
	return nm * 0.737562

func lb_ft_to_nm(lb_ft: float) -> float:
	return lb_ft / 0.737562

func kg_to_lbs(kg: float) -> float:
	return kg * 2.20462

func lbs_to_kg(lbs: float) -> float:
	return lbs / 2.20462

func bar_to_pa(bar: float) -> float:
	return bar * 100000.0

func pa_to_bar(pa: float) -> float:
	return pa / 100000.0

# Vehicle dynamic calculations
func calculate_gear_ratio(gear: int) -> float:
	if gear < 1 or gear > gear_count:
		return neutral_ratio
	return _gear_ratios[gear - 1]

func calculate_total_ratio(gear: int) -> float:
	var gear_ratio = calculate_gear_ratio(gear)
	return gear_ratio * final_drive_ratio

func calculate_wheel_rpm(engine_rpm: float, gear: int) -> float:
	var total_ratio = calculate_total_ratio(gear)
	if total_ratio == 0.0:
		return engine_rpm
	return engine_rpm / total_ratio

func calculate_vehicle_speed(wheel_rpm: float) -> float:
	var circumference = 2.0 * PI * default_wheel_radius
	var wheel_rps = wheel_rpm / 60.0
	return wheel_rps * circumference

func calculate_vehicle_speed_from_engine(engine_rpm: float, gear: int) -> float:
	var wheel_rpm = calculate_wheel_rpm(engine_rpm, gear)
	return calculate_vehicle_speed(wheel_rpm)

func calculate_torque_at_wheel(engine_torque: float, gear: int) -> float:
	var total_ratio = calculate_total_ratio(gear)
	return engine_torque * total_ratio * transmission_efficiency

func calculate_aero_drag(velocity_ms: float) -> float:
	var air_density = 1.225
	return 0.5 * air_density * velocity_ms * velocity_ms * drag_coefficient * frontal_area

func calculate_downforce(velocity_ms: float) -> float:
	var air_density = 1.225
	return 0.5 * air_density * velocity_ms * velocity_ms * downforce_coefficient * frontal_area

func get_tire_friction_coefficient(slip_ratio: float, slip_angle: float, surface_type: SurfaceType = SurfaceType.ASPHALT) -> float:
	var base_friction = pacejka_d_factor
	match surface_type:
		SurfaceType.WET:
			base_friction *= wet_surface_reduction
		SurfaceType.GRAVEL:
			base_friction *= gravel_surface_reduction
		SurfaceType.ICE:
			base_friction *= ice_surface_reduction
	
	if tire_friction_model == TireModel.SIMPLIFIED:
		return minf(base_friction, 1.0 + slip_ratio * 0.5)
	
	var alpha = slip_angle * 57.2958
	var Fz = 1.0
	
	if absf(alpha) > 0.0:
		var B = pacejka_b_factor
		var C = pacejka_c_factor
		var D = base_friction
		var E = pacejka_e_factor
		
		var h = sin(C * atan(B * alpha))
		return D * h
	else:
		return base_friction

func get_surface_friction(surface_type: SurfaceType) -> float:
	match surface_type:
		SurfaceType.ASPHALT:
			return 1.1
		SurfaceType.WET:
			return 0.7
		SurfaceType.GRAVEL:
			return 0.5
		SurfaceType.ICE:
			return 0.2
		SurfaceType.DIRT:
			return 0.6
		_:
			return 1.1

enum SurfaceType {
	ASPHALT,
	WET,
	GRAVEL,
	ICE,
	DIRT,
	TURF
}

# Gear shift logic helpers
func should_upshift(current_rpm: float, current_gear: int) -> bool:
	if current_gear >= gear_count:
		return false
	return current_rpm >= auto_shift_rpm_threshold

func should_downshift(current_rpm: float, current_gear: int) -> bool:
	if current_gear <= 1:
		return false
	var target_rpm = calculate_wheel_rpm(auto_shift_rpm_threshold, current_gear - 1)
	return current_rpm < target_rpm * 0.85

func get_optimal_shift_rpm() -> float:
	return engine_max_rpm * 0.9

func get_stall_rpm() -> float:
	return engine_max_rpm * 1.1

# Braking system helpers
func calculate_brake_force(pedal_pressure: float, gear: int) -> Dictionary:
	var front_pressure = pedal_pressure * max_brake_pressure * brake_force_distribution_front
	var rear_pressure = pedal_pressure * max_brake_pressure * (1.0 - brake_force_distribution_front)
	
	var front_force = front_pressure * brake_caliper_piston_area * brake_pad_friction
	var rear_force = rear_pressure * brake_caliper_piston_area * brake_pad_friction
	
	return {
		"front": front_force,
		"rear": rear_force,
		"total": front_force + rear_force
	}

func should_activate_abs(wheel_speed: float, slip_ratio: float) -> bool:
	return absf(slip_ratio) > abs_threshold and wheel_speed > 2.0

# Suspension helpers
func calculate_camber(wheel: String, travel: float) -> float:
	var base_camber = -1.5
	var gain = camber_gain_per_travel if wheel == "front" else camber_gain_per_travel * 0.8
	return base_camber + (travel / front_travel_limit) * gain

func calculate_toe(wheel: String, travel: float) -> float:
	var base_toe = 0.1
	var gain = toe_gain_per_travel if wheel == "front" else toe_gain_per_travel * 0.6
	return base_toe + (travel / rear_travel_limit) * gain

# Engine torque curve calculation
func calculate_engine_torque(rpm: float) -> float:
	if rpm <= idle_rpm:
		return peak_torque_nm * 0.3
	elif rpm <= peak_torque_rpm:
		var ratio = (rpm - idle_rpm) / (peak_torque_rpm - idle_rpm)
		return peak_torque_nm * (torque_curve_slope_low + (1.0 - torque_curve_slope_low) * ratio)
	elif rpm <= engine_max_rpm:
		var ratio = (rpm - peak_torque_rpm) / (engine_max_rpm - peak_torque_rpm)
		return peak_torque_nm * (1.0 - (1.0 - torque_curve_slope_high) * ratio)
	else:
		return peak_torque_nm * 0.1

# Power calculation
func calculate_engine_power(torque_nm: float, rpm: float) -> float:
	return torque_nm * rpm * 0.10472 / 1000.0

# Validation setters
func _set_gravity(value: float) -> void:
	if value > 0.0:
		gravity = value
func _set_physics_tick_rate(value: int) -> void:
	if value > 0 and value <= 240:
		physics_tick_rate = value
func _set_max_substeps(value: int) -> void:
	if value > 0 and value <= 8:
		max_substeps = value
func _set_time_scale(value: float) -> void:
	if value > 0.0 and value <= 2.0:
		time_scale = value
func _set_default_vehicle_mass(value: float) -> void:
	if value > 500.0 and value < 5000.0:
		default_vehicle_mass = value
func _set_default_wheel_radius(value: float) -> void:
	if value > 0.1 and value < 1.0:
		default_wheel_radius = value
func _set_default_wheel_width(value: float) -> void:
	if value > 0.1 and value < 0.5:
		default_wheel_width = value
func _set_wheel_track_front(value: float) -> void:
	if value > 1.0 and value < 3.0:
		wheel_track_front = value
func _set_wheel_track_rear(value: float) -> void:
	if value > 1.0 and value < 3.0:
		wheel_track_rear = value
func _set_wheelbase(value: float) -> void:
	if value > 2.0 and value < 5.0:
		wheelbase = value
func _set_center_of_mass_offset_x(value: float) -> void:
	if value > -2.0 and value < 2.0:
		center_of_mass_offset_x = value
func _set_center_of_mass_offset_y(value: float) -> void:
	if value > 0.1 and value < 1.5:
		center_of_mass_offset_y = value
func _set_center_of_mass_offset_z(value: float) -> void:
	if value > -1.0 and value < 1.0:
		center_of_mass_offset_z = value
func _set_roll_inertia(value: float) -> void:
	if value > 100.0 and value < 10000.0:
		roll_inertia = value
func _set_pitch_inertia(value: float) -> void:
	if value > 100.0 and value < 10000.0:
		pitch_inertia = value
func _set_yaw_inertia(value: float) -> void:
	if value > 100.0 and value < 10000.0:
		yaw_inertia = value
func _set_engine_max_rpm(value: float) -> void:
	if value > 5000.0 and value < 15000.0:
		engine_max_rpm = value
func _set_engine_min_rpm(value: float) -> void:
	if value > 500.0 and value < 2000.0:
		engine_min_rpm = value
func _set_idle_rpm(value: float) -> void:
	if value > 500.0 and value < 1500.0:
		idle_rpm = value
func _set_peak_torque_rpm(value: float) -> void:
	if value > 2000.0 and value < 8000.0:
		peak_torque_rpm = value
func _set_peak_torque_nm(value: float) -> void:
	if value > 100.0 and value < 1000.0:
		peak_torque_nm = value
func _set_peak_power_rpm(value: float) -> void:
	if value > 4000.0 and value < 12000.0:
		peak_power_rpm = value
func _set_peak_power_kw(value: float) -> void:
	if value > 50.0 and value < 1000.0:
		peak_power_kw = value
func _set_torque_curve_slope_low(value: float) -> void:
	if value > 0.3 and value < 1.0:
		torque_curve_slope_low = value
func _set_torque_curve_slope_high(value: float) -> void:
	if value > 0.1 and value < 0.8:
		torque_curve_slope_high = value
func _set_throttle_response_delay(value: float) -> void:
	if value >= 0.0 and value < 0.5:
		throttle_response_delay = value
func _set_clutch_slip_angle(value: float) -> void:
	if value > 0.5 and value < 5.0:
		clutch_slip_angle = value
func _set_transmission_efficiency(value: float) -> void:
	if value > 0.7 and value < 1.0:
		transmission_efficiency = value
func _set_gear_count(value: int) -> void:
	if value > 4 and value <= 8:
		gear_count = value
func _set_final_drive_ratio(value: float) -> void:
	if value > 2.0 and value < 6.0:
		final_drive_ratio = value
func _set_reverse_ratio(value: float) -> void:
	if value > 2.0 and value < 5.0:
		reverse_ratio = value
func _set_gear_shift_delay(value: float) -> void:
	if value >= 0.0 and value < 0.5:
		gear_shift_delay = value
func _set_neutral_ratio(value: float) -> void:
	if value >= 0.0:
		neutral_ratio = value
func _set_differential_type(value: DiffType) -> void:
	differential_type = value
func _set_lsd_preload_torque(value: float) -> void:
	if value > 50.0 and value < 500.0:
		lsd_preload_torque = value
func _set_lsd_max_lock_torque(value: float) -> void:
	if value > 100.0 and value < 1000.0:
		lsd_max_lock_torque = value
func _set_open_diff_accel_slip(value: float) -> void:
	if value > 0.05 and value < 0.3:
		open_diff_accel_slip = value
func _set_open_diff_decel_slip(value: float) -> void:
	if value > 0.05 and value < 0.25:
		open_diff_decel_slip = value
func _set_diff_brake_bias(value: float) -> void:
	if value >= 0.0 and value < 1.0:
		diff_brake_bias = value
func _set_front_spring_stiffness(value: float) -> void:
	if value > 10000.0 and value < 100000.0:
		front_spring_stiffness = value
func _set_rear_spring_stiffness(value: float) -> void:
	if value > 10000.0 and value < 100000.0:
		rear_spring_stiffness = value
func _set_front_damping_compression(value: float) -> void:
	if value > 1000.0 and value < 50000.0:
		front_damping_compression = value
func _set_front_damping_rebound(value: float) -> void:
	if value > 1000.0 and value < 50000.0:
		front_damping_rebound = value
func _set_rear_damping_compression(value: float) -> void:
	if value > 1000.0 and value < 50000.0:
		rear_damping_compression = value
func _set_rear_damping_rebound(value: float) -> void:
	if value > 1000.0 and value < 50000.0:
		rear_damping_rebound = value
func _set_front_travel_limit(value: float) -> void:
	if value > 0.05 and value < 0.3:
		front_travel_limit = value
func _set_rear_travel_limit(value: float) -> void:
	if value > 0.05 and value < 0.3:
		rear_travel_limit = value
func _set_anti_roll_bar_stiffness_front(value: float) -> void:
	if value > 5000.0 and value < 50000.0:
		anti_roll_bar_stiffness_front = value
func _set_anti_roll_bar_stiffness_rear(value: float) -> void:
	if value > 5000.0 and value < 50000.0:
		anti_roll_bar_stiffness_rear = value
func _set_suspension_geometry_height(value: float) -> void:
	if value > 0.1 and value < 1.0:
		suspension_geometry_height = value
func _set_camber_gain_per_travel(value: float) -> void:
	if value > 0.01 and value < 0.1:
		camber_gain_per_travel = value
func _set_toe_gain_per_travel(value: float) -> void:
	if value > 0.001 and value < 0.02:
		toe_gain_per_travel = value
func _set_tire_friction_model(value: TireModel) -> void:
	tire_friction_model = value
func _set_pacejka_b_factor(value: float) -> void:
	if value > 5.0 and value < 20.0:
		pacejka_b_factor = value
func _set_pacejka_c_factor(value: float) -> void:
	if value > 1.5 and value < 2.5:
		pacejka_c_factor = value
func _set_pacejka_d_factor(value: float) -> void:
	if value > 0.8 and value < 2.0:
		pacejka_d_factor = value
func _set_pacejka_e_factor(value: float) -> void:
	if value > 0.5 and value < 1.5:
		pacejka_e_factor = value
func _set_tire_vertical_stiffness(value: float) -> void:
	if value > 50000.0 and value < 500000.0:
		tire_vertical_stiffness = value
func _set_tire_relaxation_length(value: float) -> void:
	if value > 0.01 and value < 0.2:
		tire_relaxation_length = value
func _set_tire_temperature_optimal(value: float) -> void:
	if value > 50.0 and value < 150.0:
		tire_temperature_optimal = value
func _set_tire_temp_min(value: float) -> void:
	if value > -50.0 and value < 0.0:
		tire_temp_min = value
func _set_tire_temp_max(value: float) -> void:
	if value > 100.0 and value < 200.0:
		tire_temp_max = value
func _set_tire_wear_rate(value: float) -> void:
	if value > 0.0 and value < 0.01:
		tire_wear_rate = value
func _set_wet_surface_reduction(value: float) -> void:
	if value > 0.3 and value < 0.8:
		wet_surface_reduction = value
func _set_gravel_surface_reduction(value: float) -> void:
	if value > 0.3 and value < 0.7:
		gravel_surface_reduction = value
func _set_ice_surface_reduction(value: float) -> void:
	if value > 0.1 and value < 0.4:
		ice_surface_reduction = value
func _set_drag_coefficient(value: float) -> void:
	if value > 0.1 and value < 1.0:
		drag_coefficient = value
func _set_frontal_area(value: float) -> void:
	if value > 1.5 and value < 3.5:
		frontal_area = value
func _set_downforce_coefficient(value: float) -> void:
	if value > 0.1 and value < 2.0:
		downforce_coefficient = value
func _set_lift_coefficient(value: float) -> void:
	if value > -1.0 and value < 0.5:
		lift_coefficient = value
func _set_aero_reference_speed(value: float) -> void:
	if value > 10.0 and value < 200.0:
		aero_reference_speed = value
func _set_front_air_foil_angle(value: float) -> void:
	if value > -10.0 and value < 30.0:
		front_air_foil_angle = value
func _set_rear_air_foil_angle(value: float) -> void:
	if value > -10.0 and value < 45.0:
		rear_air_foil_angle = value
func _set_ground_effect_height(value: float) -> void:
	if value > 0.02 and value < 0.2:
		ground_effect_height = value
func _set_aero_side_force_coefficient(value: float) -> void:
	if value > 0.0 and value < 0.5:
		aero_side_force_coefficient = value
func _set_steering_ratio(value: float) -> void:
	if value > 8.0 and value < 20.0:
		steering_ratio = value
func _set_steering_lock_left(value: float) -> void:
	if value > 1.5 and value < 4.0:
		steering_lock_left = value
func _set_steering_lock_right(value: float) -> void:
	if value > 1.5 and value < 4.0:
		steering_lock_right = value
func _set_steering_deadzone(value: float) -> void:
	if value >= 0.0 and value < 0.1:
		steering_deadzone = value
func _set_steering_response_rate(value: float) -> void:
	if value > 0.5 and value < 1.0:
		steering_response_rate = value
func _set_power_assist_ratio(value: float) -> void:
	if value >= 0.0 and value < 1.0:
		power_assist_ratio = value
func _set_steering_weight_at_100kmh(value: float) -> void:
	if value > 0.5 and value < 3.0:
		steering_weight_at_100kmh = value
func _set_brake_disc_diameter_front(value: float) -> void:
	if value > 0.2 and value < 0.5:
		brake_disc_diameter_front = value
func _set_brake_disc_diameter_rear(value: float) -> void:
	if value > 0.2 and value < 0.4:
		brake_disc_diameter_rear = value
func _set_brake_pad_friction(value: float) -> void:
	if value > 0.2 and value < 0.6:
		brake_pad_friction = value
func _set_brake_caliper_piston_area(value: float) -> void:
	if value > 0.001 and value < 0.01:
		brake_caliper_piston_area = value
func _set_max_brake_pressure(value: float) -> void:
	if value > 50.0 and value < 200.0:
		max_brake_pressure = value
func _set_brake_pressure_buildup_rate(value: float) -> void:
	if value > 10.0 and value < 200.0:
		brake_pressure_buildup_rate = value
func _set_brake_pressure_release_rate(value: float) -> void:
	if value > 20.0 and value < 300.0:
		brake_pressure_release_rate = value
func _set_brake_force_distribution_front(value: float) -> void:
	if value > 0.4 and value < 0.8:
		brake_force_distribution_front = value
func _set_abs_threshold(value: float) -> void:
	if value > 0.1 and value < 0.5:
		abs_threshold = value
func _set_ebd_active(value: bool) -> void:
	ebd_active = value
func _set_ebm_current(value: float) -> void:
	if value > 0.5 and value < 2.0:
		ebm_current = value
func _set_auto_shift_rpm_threshold(value: float) -> void:
	if value > 5000.0 and value < 10000.0:
		auto_shift_rpm_threshold = value
func _set_auto_shift_delay(value: float) -> void:
	if value >= 0.0 and value < 1.0:
		auto_shift_delay = value
func _set_rev_matching_enabled(value: bool) -> void:
	rev_matching_enabled = value
func _set_rev_match_target_rpm(value: float) -> void:
	if value > 2000.0 and value < 8000.0:
		rev_match_target_rpm = value
func _set_drivetrain_type(value: Drivetrain) -> void:
	drivetrain_type = value
func _set_traction_control_level(value: int) -> void:
	if value >= 0 and value <= 10:
		traction_control_level = value
func _set_stability_control_enabled(value: bool) -> void:
	stability_control_enabled = value
func _set_launch_control_enabled(value: bool) -> void:
	launch_control_enabled = value
func _set_launch_control_rpm(value: float) -> void:
	if value > 3000.0 and value < 8000.0:
		launch_control_rpm = value
func _set_substep_collision_detection(value: bool) -> void:
	substep_collision_detection = value
func _set_raycast_precision(value: float) -> void:
	if value > 0.001 and value < 0.1:
		raycast_precision = value
func _set_contact_point_tolerance(value: float) -> void:
	if value > 0.001 and value < 0.05:
		contact_point_tolerance = value
func _set_sleep_threshold(value: float) -> void:
	if value > 0.001 and value < 0.1:
		sleep_threshold = value
func _set_wake_threshold(value: float) -> void:
	if value > 0.01 and value < 1.0:
		wake_threshold = value
</FILE>