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
@export var torque_curve_points: Array[Vector2] = [
	Vector2(0.3, 0.0), Vector2(0.4, 0.1), Vector2(0.5, 0.4),
	Vector2(0.6, 0.7), Vector2(0.7, 0.95), Vector2(0.8, 1.0),
	Vector2(0.9, 0.98), Vector2(1.0, 0.85)
]: set = _set_torque_curve_points
@export var peak_torque_nm: float = 550.0: set = _set_peak_torque_nm
@export var peak_power_kw: float = 350.0: set = _set_peak_power_kw
@export var rev_limiters_enabled: bool = true: set = _set_rev_limiters_enabled
@export var clutch_slip_angle: float = 0.5: set = _set_clutch_slip_angle
@export var flywheel_inertia: float = 0.8: set = _set_flywheel_inertia

@export_group("Transmission & Gears")
@export var transmission_type: String = "manual": set = _set_transmission_type
@export var num_gears_forward: int = 6: set = _set_num_gears_forward
@export var num_gears_reverse: int = 1: set = _set_num_gears_reverse
@export var final_drive_ratio: float = 3.73: set = _set_final_drive_ratio
@export var first_gear_ratio: float = 3.67: set = _set_first_gear_ratio
@export var second_gear_ratio: float = 2.34: set = _set_second_gear_ratio
@export var third_gear_ratio: float = 1.67: set = _set_third_gear_ratio
@export var fourth_gear_ratio: float = 1.29: set = _set_fourth_gear_ratio
@export var fifth_gear_ratio: float = 1.00: set = _set_fifth_gear_ratio
@export var sixth_gear_ratio: float = 0.82: set = _set_sixth_gear_ratio
@export var reverse_gear_ratio: float = 3.40: set = _set_reverse_gear_ratio
@export var shift_rpm_threshold: float = 7500.0: set = _set_shift_rpm_threshold
@export var upshift_delay_ms: int = 150: set = _set_upshift_delay_ms
@export var downshift_delay_ms: int = 100: set = _set_downshift_delay_ms
@export var launch_control_rpm: float = 6000.0: set = _set_launch_control_rpm
@export var traction_control_enabled: bool = true: set = _set_traction_control_enabled

@export_group("Differential Settings")
@export var diff_type: String = "limited_slip": set = _set_diff_type
@export var front_diff_lock: float = 0.0: set = _set_front_diff_lock
@export var rear_diff_lock: float = 0.35: set = _set_rear_diff_lock
@export var diff_accel_open: float = 0.5: set = _set_diff_accel_open
@export var diff_decel_open: float = 0.5: set = _set_diff_decel_open
@export var diff_preload: float = 1.0: set = _set_diff_preload
@export var diff_cog_force: float = 500.0: set = _set_diff_cog_force

@export_group("Suspension Parameters")
@export_group("Front Suspension")
@export var front_spring_rate: float = 85000.0: set = _set_front_spring_rate
@export var front_damping_compress: float = 12000.0: set = _set_front_damping_compress
@export var front_damping_rebound: float = 8000.0: set = _set_front_damping_rebound
@export var front_suspension_travel: float = 0.15: set = _set_front_suspension_travel
@export var front_anti_roll_bar_stiffness: float = 3500.0: set = _set_front_anti_roll_bar_stiffness
@export var front_camber_at_zero: float = -1.5: set = _set_front_camber_at_zero
@export var front_toe_at_zero: float = 0.05: set = _set_front_toe_at_zero
@export var front_camber_gain: float = 0.15: set = _set_front_camber_gain
@export var front_toe_gain: float = 0.08: set = _set_front_toe_gain

@export_group("Rear Suspension")
@export var rear_spring_rate: float = 95000.0: set = _set_rear_spring_rate
@export var rear_damping_compress: float = 14000.0: set = _set_rear_damping_compress
@export var rear_damping_rebound: float = 9000.0: set = _set_rear_damping_rebound
@export var rear_suspension_travel: float = 0.14: set = _set_rear_suspension_travel
@export var rear_anti_roll_bar_stiffness: float = 4000.0: set = _set_rear_anti_roll_bar_stiffness
@export var rear_camber_at_zero: float = -1.2: set = _set_rear_camber_at_zero
@export var rear_toe_at_zero: float = -0.05: set = _set_rear_toe_at_zero
@export var rear_camber_gain: float = 0.12: set = _set_rear_camber_gain
@export var rear_toe_gain: float = 0.06: set = _set_rear_toe_gain

@export_group("Tire Properties")
@export_group("Front Tires")
@export var front_tire_width: float = 0.26: set = _set_front_tire_width
@export var front_tire_pressure_pa: float = 220000.0: set = _set_front_tire_pressure_pa
@export var front_tire_compliance: float = 0.000001: set = _set_front_tire_compliance
@export var front_tire_friction_coefficient: float = 1.4: set = _set_front_tire_friction_coefficient
@export var front_tire_lateral_pacejka_b: float = 10.0: set = _set_front_tire_lateral_pacejka_b
@export var front_tire_lateral_pacejka_c: float = 1.9: set = _set_front_tire_lateral_pacejka_c
@export var front_tire_lateral_pacejka_d: float = 1.2: set = _set_front_tire_lateral_pacejka_d
@export var front_tire_lateral_pacejka_e: float = -0.3: set = _set_front_tire_lateral_pacejka_e
@export var front_tire_longitudinal_pacejka_b: float = 25.0: set = _set_front_tire_longitudinal_pacejka_b
@export var front_tire_longitudinal_pacejka_c: float = 1.9: set = _set_front_tire_longitudinal_pacejka_c
@export var front_tire_longitudinal_pacejka_d: float = 1.5: set = _set_front_tire_longitudinal_pacejka_d
@export var front_tire_longitudinal_pacejka_e: float = -0.5: set = _set_front_tire_longitudinal_pacejka_e

@export_group("Rear Tires")
@export var rear_tire_width: float = 0.28: set = _set_rear_tire_width
@export var rear_tire_pressure_pa: float = 240000.0: set = _set_rear_tire_pressure_pa
@export var rear_tire_compliance: float = 0.0000012: set = _set_rear_tire_compliance
@export var rear_tire_friction_coefficient: float = 1.5: set = _set_rear_tire_friction_coefficient
@export var rear_tire_lateral_pacejka_b: float = 11.0: set = _set_rear_tire_lateral_pacejka_b
@export var rear_tire_lateral_pacejka_c: float = 1.9: set = _set_rear_tire_lateral_pacejka_c
@export var rear_tire_lateral_pacejka_d: float = 1.3: set = _set_rear_tire_lateral_pacejka_d
@export var rear_tire_lateral_pacejka_e: float = -0.2: set = _set_rear_tire_lateral_pacejka_e
@export var rear_tire_longitudinal_pacejka_b: float = 30.0: set = _set_rear_tire_longitudinal_pacejka_b
@export var rear_tire_longitudinal_pacejka_c: float = 1.9: set = _set_rear_tire_longitudinal_pacejka_c
@export var rear_tire_longitudinal_pacejka_d: float = 1.6: set = _set_rear_tire_longitudinal_pacejka_d
@export var rear_tire_longitudinal_pacejka_e: float = -0.6: set = _set_rear_tire_longitudinal_pacejka_e

@export_group("Aerodynamics")
@export var aero_drag_coefficient: float = 0.32: set = _set_aero_drag_coefficient
@export var aero_frontal_area_m2: float = 2.1: set = _set_aero_frontal_area_m2
@export var aero_lift_coefficient: float = 0.05: set = _set_aero_lift_coefficient
@export var aero_frontal_downforce_coeff: float = -0.45: set = _set_aero_frontal_downforce_coeff
@export var aero_rear_downforce_coeff: float = -0.65: set = _set_aero_rear_downforce_coeff
@export var aero_center_of_pressure_x: float = 0.35: set = _set_aero_center_of_pressure_x
@export var aero_center_of_pressure_y: float = 0.25: set = _set_aero_center_of_pressure_y
@export var aero_side_force_coeff: float = 0.08: set = _set_aero_side_force_coeff
@export var aero_ground_clearance: float = 0.09: set = _set_aero_ground_clearance
@export var aero_tunnel_effect_factor: float = 0.15: set = _set_aero_tunnel_effect_factor

@export_group("Steering System")
@export var steering_ratio: float = 15.0: set = _set_steering_ratio
@export var steering_lock_left_rad: float = 1.1: set = _set_steering_lock_left_rad
@export var steering_lock_right_rad: float = 1.1: set = _set_steering_lock_right_rad
@export var steering_deadband: float = 0.02: set = _set_steering_deadband
@export var steering_smooth_factor: float = 0.15: set = _set_steering_smooth_factor
@export var steering_speed: float = 2.5: set = _set_steering_speed
@export var steering_return_speed: float = 3.0: set = _set_steering_return_speed
@export var steering_hysteresis: float = 0.01: set = _set_steering_hysteresis
@export var power_steering_enabled: bool = true: set = _set_power_steering_enabled
@export var power_steering_base_effort_n: float = 80.0: set = _set_power_steering_base_effort_n
@export var power_steering_max_assist: float = 0.7: set = _set_power_steering_max_assist

@export_group("Braking System")
@export var brake_bias_front_percent: float = 55.0: set = _set_brake_bias_front_percent
@export var brake_force_per_pedal: float = 15000.0: set = _set_brake_force_per_pedal
@export var brake_disc_diameter_m: float = 0.35: set = _set_brake_disc_diameter_m
@export var brake_disc_thickness_m: float = 0.03: set = _set_brake_disc_thickness_m
@export var brake_pad_friction: float = 0.4: set = _set_brake_pad_friction
@export var brake_caliper_piston_area_m2: float = 0.002: set = _set_brake_caliper_piston_area_m2
@export var brake_master_cylinder_area_m2: float = 0.0002: set = _set_brake_master_cylinder_area_m2
@export var brake_line_pressure_loss: float = 0.05: set = _set_brake_line_pressure_loss
@export var brake_abs_enabled: bool = true: set = _set_brake_abs_enabled
@export var brake_abs_threshold: float = 0.15: set = _set_brake_abs_threshold
@export var brake_abs_recovery_rate: float = 0.3: set = _set_brake_abs_recovery_rate
@export var brake_heat_capacity_j_kg_k: float = 500.0: set = _set_brake_heat_capacity_j_kg_k
@export var brake_max_temp_k: float = 1000.0: set = _set_brake_max_temp_k
@export var brake_fade_temp_k: float = 700.0: set = _set_brake_fade_temp_k
@export var brake_fade_reduction: float = 0.4: set = _set_brake_fade_reduction

@export_group("Chassis & Body")
@export var chassis_stiffness_n_m: float = 25000000.0: set = _set_chassis_stiffness_n_m
@export var chassis_torsional_rigidity_n_m: float = 18000000.0: set = _set_chassis_torsional_rigidity_n_m
@export var body_spring_rate: float = 120000.0: set = _set_body_spring_rate
@export var body_damping_compress: float = 15000.0: set = _set_body_damping_compress
@export var body_damping_rebound: float = 10000.0: set = _set_body_damping_rebound
@export var aerodynamic_center_height: float = 0.55: set = _set_aerodynamic_center_height
@export var body_length_m: float = 4.5: set = _set_body_length_m
@export var body_width_m: float = 1.9: set = _set_body_width_m
@export var body_height_m: float = 1.2: set = _set_body_height_m

@export_group("Simulation Quality")
@export var collision_accuracy: float = 0.01: set = _set_collision_accuracy
@export var contact_point_max: int = 8: set = _set_contact_point_max
@export var constraint_solver_iterations: int = 10: set = _set_constraint_solver_iterations
@export var restitution_threshold: float = 0.1: set = _set_restitution_threshold
@export var min_velocity_for_sleep: float = 0.01: set = _set_min_velocity_for_sleep
@export var sleep_time: float = 0.5: set = _set_sleep_time
@export var penetration_depth: float = 0.01: set = _set_penetration_depth

@export_group("Weather & Environment")
@export var track_surface_friction_wet: float = 0.6: set = _set_track_surface_friction_wet
@export var track_surface_friction_dry: float = 1.2: set = _set_track_surface_friction_dry
@export var track_temperature_base_c: float = 20.0: set = _set_track_temperature_base_c
@export var air_density_sea_level_kg_m3: float = 1.225: set = _set_air_density_sea_level_kg_m3
@export var air_pressure_sea_level_pa: float = 101325.0: set = _set_air_pressure_sea_level_pa
@export var altitude_density_factor: float = 0.00012: set = _set_altitude_density_factor
@export var wind_tolerance_deg: float = 30.0: set = _set_wind_tolerance_deg

func _ready() -> void:
	_validate_and_apply_defaults()

func _validate_and_apply_defaults() -> void:
	"""Ensure all values are within valid ranges after loading"""
	if gravity <= 0: gravity = 9.81
	if physics_tick_rate < 60: physics_tick_rate = 120
	if max_substeps < 1 or max_substeps > 8: max_substeps = 4
	if time_scale < 0.1: time_scale = 1.0
	if default_vehicle_mass < 500: default_vehicle_mass = 1500.0
	if engine_max_rpm < engine_min_rpm: engine_max_rpm = 8000.0
	
	if torque_curve_points.size() < 2:
		torque_curve_points = [Vector2(0.3, 0.0), Vector2(1.0, 1.0)]
	
	if num_gears_forward < 1 or num_gears_forward > 8: num_gears_forward = 6
	if num_gears_reverse < 1: num_gears_reverse = 1
	
	var speed_units := SpeedUnit.KMH
	if speed_units == SpeedUnit.MPH:
		pass  # Conversion handled in getters

func get_gear_ratio(gear_index: int) -> float:
	"""Get the gear ratio for a given gear index (1-based)"""
	match gear_index:
		1: return first_gear_ratio
		2: return second_gear_ratio
		3: return third_gear_ratio
		4: return fourth_gear_ratio
		5: return fifth_gear_ratio
		6: return sixth_gear_ratio
		_: return reverse_gear_ratio if gear_index < 0 else 1.0

func get_total_ratio(gear_index: int) -> float:
	"""Calculate total drivetrain ratio including final drive"""
	return get_gear_ratio(gear_index) * final_drive_ratio

func calculate_engine_torque(rpm_ratio: float) -> float:
	"""Interpolate torque from torque curve based on normalized RPM"""
	rpm_ratio = clamp(rpm_ratio, 0.0, 1.0)
	
	if torque_curve_points.size() < 2:
		return peak_torque_nm
	
	for i in range(torque_curve_points.size() - 1):
		var p1 := torque_curve_points[i]
		var p2 := torque_curve_points[i + 1]
		
		if rpm_ratio >= p1.x and rpm_ratio <= p2.x:
			var t := (rpm_ratio - p1.x) / (p2.x - p1.x)
			return lerp(p1.y, p2.y, t) * peak_torque_nm
	
	return lerp(torque_curve_points[0].y, torque_curve_points[-1].y, 
		(rpm_ratio - torque_curve_points[0].x) / (torque_curve_points[-1].x - torque_curve_points[0].x)) * peak_torque_nm

func calculate_power_from_torque(torque_nm: float, rpm: float) -> float:
	"""Calculate power in kW from torque and RPM"""
	return (torque_nm * rpm * 2.0 * PI) / 60000.0

func get_pacejka_lateral(slip_angle_rad: float, load_ratio: float) -> float:
	"""Calculate lateral force coefficient using Pacejka formula"""
	var params := front_tire_lateral_pacejka_b if slip_angle_rad >= 0 else rear_tire_lateral_pacejka_b
	var b := params
	var c := front_tire_lateral_pacejka_c if slip_angle_rad >= 0 else rear_tire_lateral_pacejka_c
	var d := front_tire_lateral_pacejka_d if slip_angle_rad >= 0 else rear_tire_lateral_pacejka_d
	var e := front_tire_lateral_pacejka_e if slip_angle_rad >= 0 else rear_tire_lateral_pacejka_e
	
	var angle_mod := abs(slip_angle_rad) * (PI / 180.0)
	var bc := b * c
	var sin_arg := atan(angle_mod * bc) * c / bc
	var result := d * sin(sin_arg)
	
	return result * load_ratio * friction_coefficient

func get_pacejka_longitudinal(slip_ratio: float, load_ratio: float) -> float:
	"""Calculate longitudinal force coefficient using Pacejka formula"""
	var b := front_tire_longitudinal_pacejka_b
	var c := front_tire_longitudinal_pacejka_c
	var d := front_tire_longitudinal_pacejka_d
	var e := front_tire_longitudinal_pacejka_e
	
	var slip_mod := abs(slip_ratio)
	var bc := b * c
	var sin_arg := atan(slip_mod * bc) * c / bc
	var result := d * sin(sin_arg)
	
	return result * load_ratio * friction_coefficient

func calculate_aero_drag(speed_ms: float) -> float:
	"""Calculate aerodynamic drag force in Newtons"""
	var dynamic_pressure := 0.5 * air_density_sea_level_kg_m3 * speed_ms * speed_ms
	return dynamic_pressure * aero_drag_coefficient * aero_frontal_area_m2

func calculate_aero_downforce(speed_ms: float) -> float:
	"""Calculate total aerodynamic downforce in Newtons"""
	var dynamic_pressure := 0.5 * air_density_sea_level_kg_m3 * speed_ms * speed_ms
	var front := dynamic_pressure * aero_frontal_downforce_coeff * aero_frontal_area_m2
	var rear := dynamic_pressure * aero_rear_downforce_coeff * aero_frontal_area_m2
	return front + rear

func calculate_aero_side_force(speed_ms: float, sideslip_rad: float) -> float:
	"""Calculate aerodynamic side force in Newtons"""
	var dynamic_pressure := 0.5 * air_density_sea_level_kg_m3 * speed_ms * speed_ms
	return dynamic_pressure * aero_side_force_coeff * aero_frontal_area_m2 * sideslip_rad

func get_brake_force(front_axle: bool, pedal_pressure: float) -> float:
	"""Calculate brake force for specified axle based on pedal pressure"""
	var bias := brake_bias_front_percent if front_axle else (100.0 - brake_bias_front_percent)
	return brake_force_per_pedal * pedal_pressure * (bias / 100.0)

func get_suspension_force(compression: float, velocity: float) -> float:
	"""Calculate spring and damping force for suspension"""
	var spring := compression * front_spring_rate if compression < 0 else compression * rear_spring_rate
	var damper := velocity * front_damping_compress if velocity < 0 else velocity * front_damping_rebound
	return spring + damper

func apply_unit_conversion(value: float, from_unit: UnitType, to_unit: UnitType) -> float:
	"""Convert between different units of measurement"""
	match from_unit:
		UnitType.MPS_TO_KMH: return value * 3.6
		UnitType.MPS_TO_MPH: return value * 2.23694
		UnitType.KMH_TO_MPS: return value / 3.6
		UnitType.MPH_TO_MPS: return value / 2.23694
		UnitType.RAD_TO_DEG: return value * 180.0 / PI
		UnitType.DEGRAD_TO_RAD: return value * PI / 180.0
		UnitType.NM_TO_LBFT: return value * 0.737562
		UnitType.LBFT_TO_NM: return value / 0.737562
		UnitType.PSI_TO_BAR: return value * 0.0689476
		UnitType.BAR_TO_PSI: return value / 0.0689476
		UnitType.KPA_TO_PSI: return value * 0.145038
		UnitType.PSI_TO_KPA: return value / 0.145038
		UnitType.MM_TO_INCH: return value / 25.4
		UnitType.INCH_TO_MM: return value * 25.4
		UnitType.G_TO_MS2: return value * 9.81
		UnitType.MS2_TO_G: return value / 9.81
		_: return value

func get_wheel_load_normal(vehicle_mass: float, g_force: float = 1.0) -> Vector2:
	"""Calculate normal wheel loads considering weight transfer"""
	var total_weight := vehicle_mass * gravity * g_force
	var front_load := total_weight * (wheelbase * 0.45) / wheelbase
	var rear_load := total_weight - front_load
	return Vector2(front_load / 2.0, rear_load / 2.0)

func get_weight_transfer(lateral_accel: float, mass: float, track_width: float, com_height: float) -> float:
	"""Calculate weight transfer due to lateral acceleration"""
	return (lateral_accel * mass * com_height) / track_width

func get_load_on_wheel(normal_load: float, weight_transfer: float, braking_load: float) -> float:
	"""Calculate actual load on a wheel considering all factors"""
	return normal_load + weight_transfer + braking_load

func validate_settings() -> Dictionary:
	"""Validate all physics settings and return any issues found"""
	var issues := []
	
	if gravity <= 0:
		issues.append({"field": "gravity", "value": gravity, "min": 0})
	if engine_max_rpm < engine_min_rpm:
		issues.append({"field": "engine_max_rpm", "value": engine_max_rpm, "min": engine_min_rpm})
	if default_vehicle_mass < 500:
		issues.append({"field": "default_vehicle_mass", "value": default_vehicle_mass, "min": 500})
	if num_gears_forward < 1:
		issues.append({"field": "num_gears_forward", "value": num_gears_forward, "min": 1})
	if brake_bias_front_percent < 40 or brake_bias_front_percent > 60:
		issues.append({"field": "brake_bias_front_percent", "value": brake_bias_front_percent, "range": [40, 60]})
	
	return {"valid": issues.size() == 0, "issues": issues}

# Signal for when settings change
signal settings_changed()

func _set_gravity(value: float) -> void:
	gravity = value
	settings_changed.emit()

func _set_physics_tick_rate(value: int) -> void:
	physics_tick_rate = value
	settings_changed.emit()

func _set_max_substeps(value: int) -> void:
	max_substeps = value
	settings_changed.emit()

func _set_time_scale(value: float) -> void:
	time_scale = value
	settings_changed.emit()

func _set_default_vehicle_mass(value: float) -> void:
	default_vehicle_mass = value
	settings_changed.emit()

func _set_default_wheel_radius(value: float) -> void:
	wheel_radius = value
	settings_changed.emit()

func _set_default_wheel_width(value: float) -> void:
	wheel_width = value
	settings_changed.emit()

func _set_wheel_track_front(value: float) -> void:
	wheel_track_front = value
	settings_changed.emit()

func _set_wheel_track_rear(value: float) -> void:
	wheel_track_rear = value
	settings_changed.emit()

func _set_wheelbase(value: float) -> void:
	wheelbase = value
	settings_changed.emit()

func _set_center_of_mass_offset_x(value: float) -> void:
	center_of_mass_offset_x = value
	settings_changed.emit()

func _set_center_of_mass_offset_y(value: float) -> void:
	center_of_mass_offset_y = value
	settings_changed.emit()

func _set_center_of_mass_offset_z(value: float) -> void:
	center_of_mass_offset_z = value
	settings_changed.emit()

func _set_roll_inertia(value: float) -> void:
	roll_inertia = value
	settings_changed.emit()

func _set_pitch_inertia(value: float) -> void:
	pitch_inertia = value
	settings_changed.emit()

func _set_yaw_inertia(value: float) -> void:
	yaw_inertia = value
	settings_changed.emit()

func _set_engine_max_rpm(value: float) -> void:
	engine_max_rpm = value
	settings_changed.emit()

func _set_engine_min_rpm(value: float) -> void:
	engine_min_rpm = value
	settings_changed.emit()

func _set_idle_rpm(value: float) -> void:
	idle_rpm = value
	settings_changed.emit()

func _set_torque_curve_points(value: Array[Vector2]) -> void:
	torque_curve_points = value
	settings_changed.emit()

func _set_peak_torque_nm(value: float) -> void:
	peak_torque_nm = value
	settings_changed.emit()

func _set_peak_power_kw(value: float) -> void:
	peak_power_kw = value
	settings_changed.emit()

func _set_rev_limiters_enabled(value: bool) -> void:
	rev_limiters_enabled = value
	settings_changed.emit()

func _set_clutch_slip_angle(value: float) -> void:
	clutch_slip_angle = value
	settings_changed.emit()

func _set_flywheel_inertia(value: float) -> void:
	flywheel_inertia = value
	settings_changed.emit()

func _set_transmission_type(value: String) -> void:
	transmission_type = value
	settings_changed.emit()

func _set_num_gears_forward(value: int) -> void:
	num_gears_forward = value
	settings_changed.emit()

func _set_num_gears_reverse(value: int) -> void:
	num_gears_reverse = value
	settings_changed.emit()

func _set_final_drive_ratio(value: float) -> void:
	final_drive_ratio = value
	settings_changed.emit()

func _set_first_gear_ratio(value: float) -> void:
	first_gear_ratio = value
	settings_changed.emit()

func _set_second_gear_ratio(value: float) -> void:
	second_gear_ratio = value
	settings_changed.emit()

func _set_third_gear_ratio(value: float) -> void:
	third_gear_ratio = value
	settings_changed.emit()

func _set_fourth_gear_ratio(value: float) -> void:
	fourth_gear_ratio = value
	settings_changed.emit()

func _set_fifth_gear_ratio(value: float) -> void:
	fifth_gear_ratio = value
	settings_changed.emit()

func _set_sixth_gear_ratio(value: float) -> void:
	sixth_gear_ratio = value
	settings_changed.emit()

func _set_reverse_gear_ratio(value: float) -> void:
	reverse_gear_ratio = value
	settings_changed.emit()

func _set_shift_rpm_threshold(value: float) -> void:
	shift_rpm_threshold = value
	settings_changed.emit()

func _set_upshift_delay_ms(value: int) -> void:
	upshift_delay_ms = value
	settings_changed.emit()

func _set_downshift_delay_ms(value: int) -> void:
	downshift_delay_ms = value
	settings_changed.emit()

func _set_launch_control_rpm(value: float) -> void:
	launch_control_rpm = value
	settings_changed.emit()

func _set_traction_control_enabled(value: bool) -> void:
	traction_control_enabled = value
	settings_changed.emit()

func _set_diff_type(value: String) -> void:
	diff_type = value
	settings_changed.emit()

func _set_front_diff_lock(value: float) -> void:
	front_diff_lock = value
	settings_changed.emit()

func _set_rear_diff_lock(value: float) -> void:
	rear_diff_lock = value
	settings_changed.emit()

func _set_diff_accel_open(value: float) -> void:
	diff_accel_open = value
	settings_changed.emit()

func _set_diff_decel_open(value: float) -> void:
	diff_decel_open = value
	settings_changed.emit()

func _set_diff_preload(value: float) -> void:
	diff_preload = value
	settings_changed.emit()

func _set_diff_cog_force(value: float) -> void:
	diff_cog_force = value
	settings_changed.emit()

func _set_front_spring_rate(value: float) -> void:
	front_spring_rate = value
	settings_changed.emit()

func _set_front_damping_compress(value: float) -> void:
	front_damping_compress = value
	settings_changed.emit()

func _set_front_damping_rebound(value: float) -> void:
	front_damping_rebound = value
	settings_changed.emit()

func _set_front_suspension_travel(value: float) -> void:
	front_suspension_travel = value
	settings_changed.emit()

func _set_front_anti_roll_bar_stiffness(value: float) -> void:
	front_anti_roll_bar_stiffness = value
	settings_changed.emit()

func _set_front_camber_at_zero(value: float) -> void:
	front_camber_at_zero = value
	settings_changed.emit()

func _set_front_toe_at_zero(value: float) -> void:
	front_toe_at_zero = value
	settings_changed.emit()

func _set_front_camber_gain(value: float) -> void:
	front_camber_gain = value
	settings_changed.emit()

func _set_front_toe_gain(value: float) -> void:
	front_toe_gain = value
	settings_changed.emit()

func _set_rear_spring_rate(value: float) -> void:
	rear_spring_rate = value
	settings_changed.emit()

func _set_rear_damping_compress(value: float) -> void:
	rear_damping_compress = value
	settings_changed.emit()

func _set_rear_damping_rebound(value: float) -> void:
	rear_damping_rebound = value
	settings_changed.emit()

func _set_rear_suspension_travel(value: float) -> void:
	rear_suspension_travel = value
	settings_changed.emit()

func _set_rear_anti_roll_bar_stiffness(value: float) -> void:
	rear_anti_roll_bar_stiffness = value
	settings_changed.emit()

func _set_rear_camber_at_zero(value: float) -> void:
	rear_camber_at_zero = value
	settings_changed.emit()

func _set_rear_toe_at_zero(value: float) -> void:
	rear_toe_at_zero = value
	settings_changed.emit()

func _set_rear_camber_gain(value: float) -> void:
	rear_camber_gain = value
	settings_changed.emit()

func _set_rear_toe_gain(value: float) -> void:
	rear_toe_gain = value
	settings_changed.emit()

func _set_front_tire_width(value: float) -> void:
	front_tire_width = value
	settings_changed.emit()

func _set_front_tire_pressure_pa(value: float) -> void:
	front_tire_pressure_pa = value
	settings_changed.emit()

func _set_front_tire_compliance(value: float) -> void:
	front_tire_compliance = value
	settings_changed.emit()

func _set_front_tire_friction_coefficient(value: float) -> void:
	front_tire_friction_coefficient = value
	settings_changed.emit()

func _set_front_tire_lateral_pacejka_b(value: float) -> void:
	front_tire_lateral_pacejka_b = value
	settings_changed.emit()

func _set_front_tire_lateral_pacejka_c(value: float) -> void:
	front_tire_lateral_pacejka_c = value
	settings_changed.emit()

func _set_front_tire_lateral_pacejka_d(value: float) -> void:
	front_tire_lateral_pacejka_d = value
	settings_changed.emit()

func _set_front_tire_lateral_pacejka_e(value: float) -> void:
	front_tire_lateral_pacejka_e = value
	settings_changed.emit()

func _set_front_tire_longitudinal_pacejka_b(value: float) -> void:
	front_tire_longitudinal_pacejka_b = value
	settings_changed.emit()

func _set_front_tire_longitudinal_pacejka_c(value: float) -> void:
	front_tire_longitudinal_pacejka_c = value
	settings_changed.emit()

func _set_front_tire_longitudinal_pacejka_d(value: float) -> void:
	front_tire_longitudinal_pacejka_d = value
	settings_changed.emit()

func _set_front_tire_longitudinal_pacejka_e(value: float) -> void:
	front_tire_longitudinal_pacejka_e = value
	settings_changed.emit()

func _set_rear_tire_width(value: float) -> void:
	rear_tire_width = value
	settings_changed.emit()

func _set_rear_tire_pressure_pa(value: float) -> void:
	rear_tire_pressure_pa = value
	settings_changed.emit()

func _set_rear_tire_compliance(value: float) -> void:
	rear_tire_compliance = value
	settings_changed.emit()

func _set_rear_tire_friction_coefficient(value: float) -> void:
	rear_tire_friction_coefficient = value
	settings_changed.emit()

func _set_rear_tire_lateral_pacejka_b(value: float) -> void:
	rear_tire_lateral_pacejka_b = value
	settings_changed.emit()

func _set_rear_tire_lateral_pacejka_c(value: float) -> void:
	rear_tire_lateral_pacejka_c = value
	settings_changed.emit()

func _set_rear_tire_lateral_pacejka_d(value: float) -> void:
	rear_tire_lateral_pacejka_d = value
	settings_changed.emit()

func _set_rear_tire_lateral_pacejka_e(value: float) -> void:
	rear_tire_lateral_pacejka_e = value
	settings_changed.emit()

func _set_rear_tire_longitudinal_pacejka_b(value: float) -> void:
	rear_tire_longitudinal_pacejka_b = value
	settings_changed.emit()

func _set_rear_tire_longitudinal_pacejka_c(value: float) -> void:
	rear_tire_longitudinal_pacejka_c = value
	settings_changed.emit()

func _set_rear_tire_longitudinal_pacejka_d(value: float) -> void:
	rear_tire_longitudinal_pacejka_d = value
	settings_changed.emit()

func _set_rear_tire_longitudinal_pacejka_e(value: float) -> void:
	rear_tire_longitudinal_pacejka_e = value
	settings_changed.emit()

func _set_aero_drag_coefficient(value: float) -> void:
	aero_drag_coefficient = value
	settings_changed.emit()

func _set_aero_frontal_area_m2(value: float) -> void:
	aero_frontal_area_m2 = value
	settings_changed.emit()

func _set_aero_lift_coefficient(value: float) -> void:
	aero_lift_coefficient = value
	settings_changed.emit()

func _set_aero_frontal_downforce_coeff(value: float) -> void:
	aero_frontal_downforce_coeff = value
	settings_changed.emit()

func _set_aero_rear_downforce_coeff(value: float) -> void:
	aero_rear_downforce_coeff = value
	settings_changed.emit()

func _set_aero_center_of_pressure_x(value: float) -> void:
	aero_center_of_pressure_x = value
	settings_changed.emit()

func _set_aero_center_of_pressure_y(value: float) -> void:
	aero_center_of_pressure_y = value
	settings_changed.emit()

func _set_aero_side_force_coeff(value: float) -> void:
	aero_side_force_coeff = value
	settings_changed.emit()

func _set_aero_ground_clearance(value: float) -> void:
	aero_ground_clearance = value
	settings_changed.emit()

func _set_aero_tunnel_effect_factor(value: float) -> void:
	aero_tunnel_effect_factor = value
	settings_changed.emit()

func _set_steering_ratio(value: float) -> void:
	steering_ratio = value
	settings_changed.emit()

func _set_steering_lock_left_rad(value: float) -> void:
	steering_lock_left_rad = value
	settings_changed.emit()

func _set_steering_lock_right_rad(value: float) -> void:
	steering_lock_right_rad = value
	settings_changed.emit()

func _set_steering_deadband(value: float) -> void:
	steering_deadband = value
	settings_changed.emit()

func _set_steering_smooth_factor(value: float) -> void:
	steering_smooth_factor = value
	settings_changed.emit()

func _set_steering_speed(value: float) -> void:
	steering_speed = value
	settings_changed.emit()

func _set_steering_return_speed(value: float) -> void:
	steering_return_speed = value
	settings_changed.emit()

func _set_steering_hysteresis(value: float) -> void:
	steering_hysteresis = value
	settings_changed.emit()

func _set_power_steering_enabled(value: bool) -> void:
	power_steering_enabled = value
	settings_changed.emit()

func _set_power_steering_base_effort_n(value: float) -> void:
	power_steering_base_effort_n = value
	settings_changed.emit()

func _set_power_steering_max_assist(value: float) -> void:
	power_steering_max_assist = value
	settings_changed.emit()

func _set_brake_bias_front_percent(value: float) -> void:
	brake_bias_front_percent = value
	settings_changed.emit()

func _set_brake_force_per_pedal(value: float) -> void:
	brake_force_per_pedal = value
	settings_changed.emit()

func _set_brake_disc_diameter_m(value: float) -> void:
	brake_disc_diameter_m = value
	settings_changed.emit()

func _set_brake_disc_thickness_m(value: float) -> void:
	brake_disc_thickness_m = value
	settings_changed.emit()

func _set_brake_pad_friction(value: float) -> void:
	brake_pad_friction = value
	settings_changed.emit()

func _set_brake_caliper_piston_area_m2(value: float) -> void:
	brake_caliper_piston_area_m2 = value
	settings_changed.emit()

func _set_brake_master_cylinder_area_m2(value: float) -> void:
	brake_master_cylinder_area_m2 = value
	settings_changed.emit()

func _set_brake_line_pressure_loss(value: float) -> void:
	brake_line_pressure_loss = value
	settings_changed.emit()

func _set_brake_abs_enabled(value: bool) -> void:
	brake_abs_enabled = value
	settings_changed.emit()

func _set_brake_abs_threshold(value: float) -> void:
	brake_abs_threshold = value
	settings_changed.emit()

func _set_brake_abs_recovery_rate(value: float) -> void:
	brake_abs_recovery_rate = value
	settings_changed.emit()

func _set_brake_heat_capacity_j_kg_k(value: float) -> void:
	brake_heat_capacity_j_kg_k = value
	settings_changed.emit()

func _set_brake_max_temp_k(value: float) -> void:
	brake_max_temp_k = value
	settings_changed.emit()

func _set_brake_fade_temp_k(value: float) -> void:
	brake_fade_temp_k = value
	settings_changed.emit()

func _set_brake_fade_reduction(value: float) -> void:
	brake_fade_reduction = value
	settings_changed.emit()

func _set_chassis_stiffness_n_m(value: float) -> void:
	chassis_stiffness_n_m = value
	settings_changed.emit()

func _set_chassis_torsional_rigidity_n_m(value: float) -> void:
	chassis_torsional_rigidity_n_m = value
	settings_changed.emit()

func _set_body_spring_rate(value: float) -> void:
	body_spring_rate = value
	settings_changed.emit()

func _set_body_damping_compress(value: float) -> void:
	body_damping_compress = value
	settings_changed.emit()

func _set_body_damping_rebound(value: float) -> void:
	body_damping_rebound = value
	settings_changed.emit()

func _set_aerodynamic_center_height(value: float) -> void:
	aerodynamic_center_height = value
	settings_changed.emit()

func _set_body_length_m(value: float) -> void:
	body_length_m = value
	settings_changed.emit()

func _set_body_width_m(value: float) -> void:
	body_width_m = value
	settings_changed.emit()

func _set_body_height_m(value: float) -> void:
	body_height_m = value
	settings_changed.emit()

func _set_collision_accuracy(value: float) -> void:
	collision_accuracy = value
	settings_changed.emit()

func _set_contact_point_max(value: int) -> void:
	contact_point_max = value
	settings_changed.emit()

func _set_constraint_solver_iterations(value: int) -> void:
	constraint_solver_iterations = value
	settings_changed.emit()

func _set_restitution_threshold(value: float) -> void:
	restitution_threshold = value
	settings_changed.emit()

func _set_min_velocity_for_sleep(value: float) -> void:
	min_velocity_for_sleep = value
	settings_changed.emit()

func _set_sleep_time(value: float) -> void:
	sleep_time = value
	settings_changed.emit()

func _set_penetration_depth(value: float) -> void:
	penetration_depth = value
	settings_changed.emit()

func _set_track_surface_friction_wet(value: float) -> void:
	track_surface_friction_wet = value
	settings_changed.emit()

func _set_track_surface_friction_dry(value: float) -> void:
	track_surface_friction_dry = value
	settings_changed.emit()

func _set_track_temperature_base_c(value: float) -> void:
	track_temperature_base_c = value
	settings_changed.emit()

func _set_air_density_sea_level_kg_m3(value: float) -> void:
	air_density_sea_level_kg_m3 = value
	settings_changed.emit()

func _set_air_pressure_sea_level_pa(value: float) -> void:
	air_pressure_sea_level_pa = value
	settings_changed.emit()

func _set_altitude_density_factor(value: float) -> void:
	altitude_density_factor = value
	settings_changed.emit()

func _set_wind_tolerance_deg(value: float) -> void:
	wind_tolerance_deg = value
	settings_changed.emit()

enum SpeedUnit {
	KMH,
	MPH
}

enum UnitType {
	MPS_TO_KMH,
	MPS_TO_MPH,
	KMH_TO_MPS,
	MPH_TO_MPS,
	RAD_TO_DEG,
	DEGRAD_TO_RAD,
	NM_TO_LBFT,
	LBFT_TO_NM,
	PSI_TO_BAR,
	BAR_TO_PSI,
	KPA_TO_PSI,
	PSI_TO_KPA,
	MM_TO_INCH,
	INCH_TO_MM,
	G_TO_MS2,
	MS2_TO_G
}

var friction_coefficient: float = 1.0
var wheel_radius: float = 0.32
var wheel_width: float = 0.22