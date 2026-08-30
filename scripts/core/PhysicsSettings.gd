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
@export var diff_type: int = 1: set = _set_diff_type
@export var diff_preload: float = 150.0: set = _set_diff_preload
@export var diff_lock_low: float = 0.15: set = _set_diff_lock_low
@export var diff_lock_high: float = 0.6: set = _set_diff_lock_high
@export var diff_lock_accel: float = 0.4: set = _set_diff_lock_accel
@export var diff_lock_decel: float = 0.3: set = _set_diff_lock_decel
@export var diff_open_left: bool = false: set = _set_diff_open_left
@export var diff_locked: bool = false: set = _set_diff_locked
@export var diff_cooling_factor: float = 1.0: set = _set_diff_cooling_factor

enum DiffType {
	OPEN,
	LOCKED,
	LSD_OPEN,
	LSD_LOCKED,
	CVT
}

@export_group("Suspension Parameters")
@export_group("Front Suspension")
@export var front_spring_stiffness: float = 35000.0: set = _set_front_spring_stiffness
@export var front_spring_damping_compression: float = 8000.0: set = _set_front_spring_damping_compression
@export var front_spring_damping_rebound: float = 6000.0: set = _set_front_spring_damping_rebound
@export var front_spring_rest_length: float = 0.35: set = _set_front_spring_rest_length
@export var front_spring_max_travel: float = 0.15: set = _set_front_spring_max_travel
@export var front_spring_min_travel: float = -0.05: set = _set_front_spring_min_travel
@export var front_bump_stop_force: float = 50000.0: set = _set_front_bump_stop_force
@export var front_bump_stop_position: float = 0.12: set = _set_front_bump_stop_position
@export var front_anti_roll_bar_stiffness: float = 2500.0: set = _set_front_anti_roll_bar_stiffness
@export var front_suspension_geometry: Dictionary = {
	"caster_angle": 4.0,
	"camber_gain": -1.2,
	"toe_gain": -0.05,
	"kingpin_inclination": 12.0
}

@export_group("Rear Suspension")
@export var rear_spring_stiffness: float = 38000.0: set = _set_rear_spring_stiffness
@export var rear_spring_damping_compression: float = 9000.0: set = _set_rear_spring_damping_compression
@export var rear_spring_damping_rebound: float = 7000.0: set = _set_rear_spring_damping_rebound
@export var rear_spring_rest_length: float = 0.35: set = _set_rear_spring_rest_length
@export var rear_spring_max_travel: float = 0.15: set = _set_rear_spring_max_travel
@export var rear_spring_min_travel: float = -0.05: set = _set_rear_spring_min_travel
@export var rear_bump_stop_force: float = 55000.0: set = _set_rear_bump_stop_force
@export var rear_bump_stop_position: float = 0.12: set = _set_rear_bump_stop_position
@export var rear_anti_roll_bar_stiffness: float = 3000.0: set = _set_rear_anti_roll_bar_stiffness
@export var rear_suspension_geometry: Dictionary = {
	"caster_angle": 3.5,
	"camber_gain": -1.0,
	"toe_gain": 0.02,
	"kingpin_inclination": 11.0
}

@export_group("Tire Friction Curves")
@export_group("Front Tires")
@export var front_tire_friction_static: float = 1.35: set = _set_front_tire_friction_static
@export var front_tire_friction_dynamic: float = 1.15: set = _set_front_tire_friction_dynamic
@export var front_tire_pacejka_a: float = 2.8: set = _set_front_tire_pacejka_a
@export var front_tire_pacejka_b: float = 1.85: set = _set_front_tire_pacejka_b
@export var front_tire_pacejka_c: float = 0.95: set = _set_front_tire_pacejka_c
@export var front_tire_pacejka_e: float = 0.5: set = _set_front_tire_pacejka_e
@export var front_tire_side_friction_exponent: float = 1.2: set = _set_front_tire_side_friction_exponent
@export var front_tire_vertical_stiffness: float = 180000.0: set = _set_front_tire_vertical_stiffness
@export var front_tire_width: float = 0.265: set = _set_front_tire_width
@export var front_tire_pressure_bar: float = 2.2: set = _set_front_tire_pressure_bar
@export var front_tire_temperature_optimal: float = 90.0: set = _set_front_tire_temperature_optimal
@export var front_tire_temperature_min: float = -10.0: set = _set_front_tire_temperature_min
@export var front_tire_temperature_max: float = 120.0: set = _set_front_tire_temperature_max

@export_group("Rear Tires")
@export var rear_tire_friction_static: float = 1.45: set = _set_rear_tire_friction_static
@export var rear_tire_friction_dynamic: float = 1.25: set = _set_rear_tire_friction_dynamic
@export var rear_tire_pacejka_a: float = 2.9: set = _set_rear_tire_pacejka_a
@export var rear_tire_pacejka_b: float = 1.9: set = _set_rear_tire_pacejka_b
@export var rear_tire_pacejka_c: float = 0.92: set = _set_rear_tire_pacejka_c
@export var rear_tire_pacejka_e: float = 0.45: set = _set_rear_tire_pacejka_e
@export var rear_tire_side_friction_exponent: float = 1.25: set = _set_rear_tire_side_friction_exponent
@export var rear_tire_vertical_stiffness: float = 200000.0: set = _set_rear_tire_vertical_stiffness
@export var rear_tire_width: float = 0.295: set = _set_rear_tire_width
@export var rear_tire_pressure_bar: float = 2.0: set = _set_rear_tire_pressure_bar
@export var rear_tire_temperature_optimal: float = 95.0: set = _set_rear_tire_temperature_optimal
@export var rear_tire_temperature_min: float = -10.0: set = _set_rear_tire_temperature_min
@export var rear_tire_temperature_max: float = 125.0: set = _set_rear_tire_temperature_max

@export_group("Aerodynamics")
@export var aero_drag_coefficient: float = 0.32: set = _set_aero_drag_coefficient
@export var aero_frontal_area: float = 2.1: set = _set_aero_frontal_area
@export var aero_downforce_coefficient: float = 0.85: set = _set_aero_downforce_coefficient
@export var aero_lift_coefficient: float = -0.15: set = _set_aero_lift_coefficient
@export var aero_front_spreader: float = 0.55: set = _set_aero_front_spreader
@export var aero_rear_spreader: float = 0.45: set = _set_aero_rear_spreader
@export var aero_ground_clearance: float = 0.09: set = _set_aero_ground_clearance
@export var aero_floor_effect_distance: float = 0.12: set = _set_aero_floor_effect_distance
@export var aero_wing_incidence_front: float = 2.5: set = _set_aero_wing_incidence_front
@export var aero_wing_incidence_rear: float = 6.0: set = _set_aero_wing_incidence_rear
@export var aero_drag_decay_speed: float = 150.0: set = _set_aero_drag_decay_speed
@export var aero_downforce_decay_speed: float = 120.0: set = _set_aero_downforce_decay_speed

@export_group("Steering System")
@export var steering_ratio: float = 14.5: set = _set_steering_ratio
@export var steering_lock_angle: float = 2.8: set = _set_steering_lock_angle
@export var steering_lock_degrees: float = 160.0: set = _set_steering_lock_degrees
@export var steering_deadzone: float = 0.02: set = _set_steering_deadzone
@export var steering_return_speed: float = 2.5: set = _set_steering_return_speed
@export var steering_weight_curve: Array[float] = [0.3, 0.45, 0.65, 0.85, 1.0]
@export var steering_weight_speed_threshold: float = 50.0: set = _set_steering_weight_speed_threshold
@export var steering_progressive_ratio: bool = true: set = _set_steering_progressive_ratio
@export var steering_assist_enabled: bool = false: set = _set_steering_assist_enabled
@export var steering_assist_strength: float = 0.3: set = _set_steering_assist_strength

@export_group("Brake System")
@export var brake_force_distribution: Vector2 = Vector2(0.6, 0.4): set = _set_brake_force_distribution
@export var brake_master_cylinder_diameter: float = 0.022: set = _set_brake_master_cylinder_diameter
@export var brake_caliper_piston_area: float = 0.0025: set = _set_brake_caliper_piston_area
@export var brake_pad_friction: float = 0.4: set = _set_brake_pad_friction
@export var brake_disc_radius: float = 0.31: set = _set_brake_disc_radius
@export var brake_disc_thickness: float = 0.032: set = _set_brake_disc_thickness
@export var brake_max_pressure_bar: float = 100.0: set = _set_brake_max_pressure_bar
@export var brake_line_pressure_loss: float = 0.05: set = _set_brake_line_pressure_loss
@export var brake_compound_temperature_optimal: float = 250.0: set = _set_brake_compound_temperature_optimal
@export var brake_compound_fade_temp: float = 400.0: set = _set_brake_compound_fade_temp
@export var brake_recovery_time: float = 0.8: set = _set_brake_recovery_time
@export var abs_enabled: bool = true: set = _set_abs_enabled
@export var abs_threshold: float = 0.15: set = _set_abs_threshold
@export var abs_modulation_rate: float = 25.0: set = _set_abs_modulation_rate
@export var brake_balance_adjustable: bool = true: set = _set_brake_balance_adjustable
@export var brake_balance_range: Vector2 = Vector2(0.45, 0.75): set = _set_brake_balance_range

@export_group("Transmission & Clutch")
@export var clutch_capacity: float = 550.0: set = _set_clutch_capacity
@export var clutch_engagement_point: float = 0.7: set = _set_clutch_engagement_point
@export var clutch_spring_stiffness: float = 15000.0: set = _set_clutch_spring_stiffness
@export var flywheel_inertia: float = 2.5: set = _set_flywheel_inertia
@export var rev_limit_shutoff_rpm: float = 8200.0: set = _set_rev_limit_shutoff_rpm
@export var rev_limit_redline: float = 8500.0: set = _set_rev_limit_redline
@export var downshift_rpm_threshold: float = 6800.0: set = _set_downshift_rpm_threshold
@export var upshift_rpm_threshold: float = 7800.0: set = _set_upshift_rpm_threshold
@export var launch_control_enabled: bool = false: set = _set_launch_control_enabled
@export var launch_control_rpm: float = 6500.0: set = _set_launch_control_rpm
@export var traction_control_enabled: bool = true: set = _set_traction_control_enabled
@export var traction_control_level: int = 2: set = _set_traction_control_level
@export var traction_control_max_slip: float = 0.22: set = _set_traction_control_max_slip

@export_group("Race Configuration")
@export var track_surface_friction: float = 0.9: set = _set_track_surface_friction
@export var track_temperature_base: float = 30.0: set = _set_track_temperature_base
@export var track_wetness_factor: float = 0.0: set = _set_track_wetness_factor
@export var wind_variation: float = 0.0: set = _set_wind_variation
@export var simulation_accuracy: int = 2: set = _set_simulation_accuracy

enum SimulationAccuracy {
	FAST,
	BALANCED,
	HIGH
}

var _gear_ratios_internal: Array[float] = []
var _torque_curve_cache: Array[Vector2] = []

func _init() -> void:
	_gear_ratios_internal = _gear_ratios.duplicate()
	_generate_torque_curve()

func _generate_torque_curve() -> void:
	_torque_curve_cache.clear()
	var rpm_steps: int = 100
	for i in range(rpm_steps + 1):
		var rpm = lerp(engine_min_rpm, engine_max_rpm, float(i) / float(rpm_steps))
		var normalized_rpm = (rpm - engine_min_rpm) / (engine_max_rpm - engine_min_rpm)
		var torque_factor: float
		if normalized_rpm <= 0.5:
			torque_factor = lerp(torque_curve_slope_low, 1.0, normalized_rpm * 2.0)
		else:
			torque_factor = 1.0 - lerp(0.0, 1.0 - torque_curve_slope_high, (normalized_rpm - 0.5) * 2.0)
		var torque = peak_torque_nm * torque_factor
		_torque_curve_cache.append(Vector2(rpm, torque))

func get_torque_at_rpm(rpm: float) -> float:
	rpm = clampf(rpm, engine_min_rpm, engine_max_rpm)
	var step_size: float = (engine_max_rpm - engine_min_rpm) / 100.0
	var index: int = int((rpm - engine_min_rpm) / step_size)
	index = clamp(index, 0, 99)
	var next_index = min(index + 1, 99)
	var t = (rpm - engine_min_rpm) / step_size - index
	var torque = lerp(_torque_curve_cache[index].y, _torque_curve_cache[next_index].y, t)
	return torque

func get_power_at_rpm(rpm: float) -> float:
	var torque_nm = get_torque_at_rpm(rpm)
	var power_kw = (torque_nm * rpm) / 9549.2966
	return power_kw

func get_gear_ratio(gear: int) -> float:
	gear = clamp(gear, 0, gear_count - 1)
	if gear < _gear_ratios_internal.size():
		return _gear_ratios_internal[gear]
	return neutral_ratio

func get_total_ratio(gear: int = 1) -> float:
	var gear_ratio: float = get_gear_ratio(gear - 1) if gear > 0 else reverse_ratio
	return gear_ratio * final_drive_ratio

func get_road_rpm(rpm_wheel: float, gear: int = 1) -> float:
	return rpm_wheel * get_total_ratio(gear)

func get_wheel_rpm(engine_rpm: float, gear: int = 1) -> float:
	return engine_rpm / get_total_ratio(gear)

func get_vehicle_speed_from_rpm(wheel_rpm: float, wheel_radius: float = -1.0) -> float:
	if wheel_radius < 0.0:
		wheel_radius = default_wheel_radius
	var circumference = 2.0 * PI * wheel_radius
	var meters_per_second = (wheel_rpm / 60.0) * circumference
	return meters_per_second

func get_rpm_from_vehicle_speed(speed_mps: float, wheel_radius: float = -1.0) -> float:
	if wheel_radius < 0.0:
		wheel_radius = default_wheel_radius
	var circumference = 2.0 * PI * wheel_radius
	var wheel_rps = speed_mps / circumference
	return wheel_rps * 60.0

func get_top_speed(gear: int, gear_ratio_override: float = -1.0) -> float:
	var ratio: float = gear_ratio_override if gear_ratio_override > 0 else get_total_ratio(gear)
	var wheel_rpm_at_max = engine_max_rpm / ratio
	var speed_mps = get_vehicle_speed_from_rpm(wheel_rpm_at_max)
	return speed_mps * 3.6

func calculate_downforce(velocity: float, angle_of_attack: float = 0.0) -> float:
	var dynamic_pressure = 0.5 * 1.225 * velocity * velocity
	var total_downforce = dynamic_pressure * aero_frontal_area * aero_downforce_coefficient
	var aoa_factor = cos(angle_of_attack * PI / 180.0)
	return total_downforce * aoa_factor

func calculate_drag(velocity: float) -> float:
	var dynamic_pressure = 0.5 * 1.225 * velocity * velocity
	return dynamic_pressure * aero_frontal_area * aero_drag_coefficient

func apply_aero_correction(downforce: float, speed: float) -> float:
	var speed_ratio = min(speed / aero_drag_decay_speed, 1.0)
	var correction = 1.0 - (1.0 - aero_front_spreader) * speed_ratio
	return downforce * correction

func calculate_tire_friction(sliding_ratio: float, slip_angle: float, load: float, temperature: float = -1.0) -> float:
	if temperature < 0.0:
		temperature = front_tire_temperature_optimal
	var temp_factor = 1.0
	if temperature >= front_tire_temperature_min and temperature <= front_tire_temperature_max:
		temp_factor = 1.0 - abs(temperature - front_tire_temperature_optimal) / (front_tire_temperature_max - front_tire_temperature_optimal) * 0.3
	var pacejka_input = abs(slip_angle) * front_tire_pacejka_b
	var friction = front_tire_pacejka_a * sin(front_tire_pacejka_c * pacejka_input)
	friction *= temp_factor
	friction *= (1.0 - sliding_ratio * 0.5)
	return clampf(friction, 0.1, front_tire_friction_static)

func convert_km_h_to_ms(kmh: float) -> float:
	return kmh / 3.6

func convert_ms_to_km_h(ms: float) -> float:
	return ms * 3.6

func convert_rpm_to_rps(rpm: float) -> float:
	return rpm / 60.0

func convert_rps_to_rpm(rps: float) -> float:
	return rps * 60.0

func convert_bar_to_pa(bar: float) -> float:
	return bar * 100000.0

func convert_pa_to_bar(pa: float) -> float:
	return pa / 100000.0

func convert_newton_meter_to_lb_ft(nm: float) -> float:
	return nm * 0.737562

func convert_lb_ft_to_newton_meter(lb_ft: float) -> float:
	return lb_ft / 0.737562

func convert_kg_to_lb(kg: float) -> float:
	return kg * 2.20462

func convert_lb_to_kg(lb: float) -> float:
	return lb / 2.20462

func convert_mm_to_in(mm: float) -> float:
	return mm / 25.4

func convert_in_to_mm(inches: float) -> float:
	return inches * 25.4

func deg_to_rad(degrees: float) -> float:
	return degrees * PI / 180.0

func rad_to_deg(radians: float) -> float:
	return radians * 180.0 / PI

func calculate_brake_force(pedal_pressure: float, brake_balance: float = 0.6) -> Vector2:
	brake_balance = clamp(brake_balance, brake_balance_range.x, brake_balance_range.y)
	var total_force = pedal_pressure * brake_max_pressure_bar * brake_master_cylinder_diameter
	var front_force = total_force * brake_balance
	var rear_force = total_force * (1.0 - brake_balance)
	return Vector2(front_force, rear_force)

func calculate_suspension_load(wheel_position: float, spring_stiffness: float, rest_length: float) -> float:
	var compression = rest_length - wheel_position
	return compression * spring_stiffness

func get_suspension_damping(compression_velocity: float, damping_type: int) -> float:
	var damping_value: float
	if damping_type == 1:
		damping_value = front_spring_damping_compression
	else:
		damping_value = rear_spring_damping_compression
	return damping_value * compression_velocity

func _set_gravity(value: float) -> void:
	gravity = value
	notify_property_list_changed()

func _set_physics_tick_rate(value: int) -> void:
	physics_tick_rate = value
	notify_property_list_changed()

func _set_max_substeps(value: int) -> void:
	max_substeps = value
	notify_property_list_changed()

func _set_time_scale(value: float) -> void:
	time_scale = value
	notify_property_list_changed()

func _set_default_vehicle_mass(value: float) -> void:
	default_vehicle_mass = value
	notify_property_list_changed()

func _set_default_wheel_radius(value: float) -> void:
	default_wheel_radius = value
	notify_property_list_changed()

func _set_default_wheel_width(value: float) -> void:
	default_wheel_width = value
	notify_property_list_changed()

func _set_wheel_track_front(value: float) -> void:
	wheel_track_front = value
	notify_property_list_changed()

func _set_wheel_track_rear(value: float) -> void:
	wheel_track_rear = value
	notify_property_list_changed()

func _set_wheelbase(value: float) -> void:
	wheelbase = value
	notify_property_list_changed()

func _set_center_of_mass_offset_x(value: float) -> void:
	center_of_mass_offset_x = value
	notify_property_list_changed()

func _set_center_of_mass_offset_y(value: float) -> void:
	center_of_mass_offset_y = value
	notify_property_list_changed()

func _set_center_of_mass_offset_z(value: float) -> void:
	center_of_mass_offset_z = value
	notify_property_list_changed()

func _set_roll_inertia(value: float) -> void:
	roll_inertia = value
	notify_property_list_changed()

func _set_pitch_inertia(value: float) -> void:
	pitch_inertia = value
	notify_property_list_changed()

func _set_yaw_inertia(value: float) -> void:
	yaw_inertia = value
	notify_property_list_changed()

func _set_engine_max_rpm(value: float) -> void:
	engine_max_rpm = value
	_generate_torque_curve()
	notify_property_list_changed()

func _set_engine_min_rpm(value: float) -> void:
	engine_min_rpm = value
	_generate_torque_curve()
	notify_property_list_changed()

func _set_idle_rpm(value: float) -> void:
	idle_rpm = value
	notify_property_list_changed()

func _set_peak_torque_rpm(value: float) -> void:
	peak_torque_rpm = value
	notify_property_list_changed()

func _set_peak_torque_nm(value: float) -> void:
	peak_torque_nm = value
	notify_property_list_changed()

func _set_peak_power_rpm(value: float) -> void:
	peak_power_rpm = value
	notify_property_list_changed()

func _set_peak_power_kw(value: float) -> void:
	peak_power_kw = value
	notify_property_list_changed()

func _set_torque_curve_slope_low(value: float) -> void:
	torque_curve_slope_low = value
	_generate_torque_curve()
	notify_property_list_changed()

func _set_torque_curve_slope_high(value: float) -> void:
	torque_curve_slope_high = value
	_generate_torque_curve()
	notify_property_list_changed()

func _set_throttle_response_delay(value: float) -> void:
	throttle_response_delay = value
	notify_property_list_changed()

func _set_clutch_slip_angle(value: float) -> void:
	clutch_slip_angle = value
	notify_property_list_changed()

func _set_transmission_efficiency(value: float) -> void:
	transmission_efficiency = value
	notify_property_list_changed()

func _set_gear_count(value: int) -> void:
	gear_count = value
	notify_property_list_changed()

func _set_final_drive_ratio(value: float) -> void:
	final_drive_ratio = value
	notify_property_list_changed()

func _set_reverse_ratio(value: float) -> void:
	reverse_ratio = value
	notify_property_list_changed()

func _set_gear_shift_delay(value: float) -> void:
	gear_shift_delay = value
	notify_property_list_changed()

func _set_neutral_ratio(value: float) -> void:
	neutral_ratio = value
	notify_property_list_changed()

func _set_diff_type(value: int) -> void:
	diff_type = value
	notify_property_list_changed()

func _set_diff_preload(value: float) -> void:
	diff_preload = value
	notify_property_list_changed()

func _set_diff_lock_low(value: float) -> void:
	diff_lock_low = value
	notify_property_list_changed()

func _set_diff_lock_high(value: float) -> void:
	diff_lock_high = value
	notify_property_list_changed()

func _set_diff_lock_accel(value: float) -> void:
	diff_lock_accel = value
	notify_property_list_changed()

func _set_diff_lock_decel(value: float) -> void:
	diff_lock_decel = value
	notify_property_list_changed()

func _set_diff_open_left(value: bool) -> void:
	diff_open_left = value
	notify_property_list_changed()

func _set_diff_locked(value: bool) -> void:
	diff_locked = value
	notify_property_list_changed()

func _set_diff_cooling_factor(value: float) -> void:
	diff_cooling_factor = value
	notify_property_list_changed()

func _set_front_spring_stiffness(value: float) -> void:
	front_spring_stiffness = value
	notify_property_list_changed()

func _set_front_spring_damping_compression(value: float) -> void:
	front_spring_damping_compression = value
	notify_property_list_changed()

func _set_front_spring_damping_rebound(value: float) -> void:
	front_spring_damping_rebound = value
	notify_property_list_changed()

func _set_front_spring_rest_length(value: float) -> void:
	front_spring_rest_length = value
	notify_property_list_changed()

func _set_front_spring_max_travel(value: float) -> void:
	front_spring_max_travel = value
	notify_property_list_changed()

func _set_front_spring_min_travel(value: float) -> void:
	front_spring_min_travel = value
	notify_property_list_changed()

func _set_front_bump_stop_force(value: float) -> void:
	front_bump_stop_force = value
	notify_property_list_changed()

func _set_front_bump_stop_position(value: float) -> void:
	front_bump_stop_position = value
	notify_property_list_changed()

func _set_front_anti_roll_bar_stiffness(value: float) -> void:
	front_anti_roll_bar_stiffness = value
	notify_property_list_changed()

func _set_rear_spring_stiffness(value: float) -> void:
	rear_spring_stiffness = value
	notify_property_list_changed()

func _set_rear_spring_damping_compression(value: float) -> void:
	rear_spring_damping_compression = value
	notify_property_list_changed()

func _set_rear_spring_damping_rebound(value: float) -> void:
	rear_spring_damping_rebound = value
	notify_property_list_changed()

func _set_rear_spring_rest_length(value: float) -> void:
	rear_spring_rest_length = value
	notify_property_list_changed()

func _set_rear_spring_max_travel(value: float) -> void:
	rear_spring_max_travel = value
	notify_property_list_changed()

func _set_rear_spring_min_travel(value: float) -> void:
	rear_spring_min_travel = value
	notify_property_list_changed()

func _set_rear_bump_stop_force(value: float) -> void:
	rear_bump_stop_force = value
	notify_property_list_changed()

func _set_rear_bump_stop_position(value: float) -> void:
	rear_bump_stop_position = value
	notify_property_list_changed()

func _set_rear_anti_roll_bar_stiffness(value: float) -> void:
	rear_anti_roll_bar_stiffness = value
	notify_property_list_changed()

func _set_front_tire_friction_static(value: float) -> void:
	front_tire_friction_static = value
	notify_property_list_changed()

func _set_front_tire_friction_dynamic(value: float) -> void:
	front_tire_friction_dynamic = value
	notify_property_list_changed()

func _set_front_tire_pacejka_a(value: float) -> void:
	front_tire_pacejka_a = value
	notify_property_list_changed()

func _set_front_tire_pacejka_b(value: float) -> void:
	front_tire_pacejka_b = value
	notify_property_list_changed()

func _set_front_tire_pacejka_c(value: float) -> void:
	front_tire_pacejka_c = value
	notify_property_list_changed()

func _set_front_tire_pacejka_e(value: float) -> void:
	front_tire_pacejka_e = value
	notify_property_list_changed()

func _set_front_tire_side_friction_exponent(value: float) -> void:
	front_tire_side_friction_exponent = value
	notify_property_list_changed()

func _set_front_tire_vertical_stiffness(value: float) -> void:
	front_tire_vertical_stiffness = value
	notify_property_list_changed()

func _set_front_tire_width(value: float) -> void:
	front_tire_width = value
	notify_property_list_changed()

func _set_front_tire_pressure_bar(value: float) -> void:
	front_tire_pressure_bar = value
	notify_property_list_changed()

func _set_front_tire_temperature_optimal(value: float) -> void:
	front_tire_temperature_optimal = value
	notify_property_list_changed()

func _set_front_tire_temperature_min(value: float) -> void:
	front_tire_temperature_min = value
	notify_property_list_changed()

func _set_front_tire_temperature_max(value: float) -> void:
	front_tire_temperature_max = value
	notify_property_list_changed()

func _set_rear_tire_friction_static(value: float) -> void:
	rear_tire_friction_static = value
	notify_property_list_changed()

func _set_rear_tire_friction_dynamic(value: float) -> void:
	rear_tire_friction_dynamic = value
	notify_property_list_changed()

func _set_rear_tire_pacejka_a(value: float) -> void:
	rear_tire_pacejka_a = value
	notify_property_list_changed()

func _set_rear_tire_pacejka_b(value: float) -> void:
	rear_tire_pacejka_b = value
	notify_property_list_changed()

func _set_rear_tire_pacejka_c(value: float) -> void:
	rear_tire_pacejka_c = value
	notify_property_list_changed()

func _set_rear_tire_pacejka_e(value: float) -> void:
	rear_tire_pacejka_e = value
	notify_property_list_changed()

func _set_rear_tire_side_friction_exponent(value: float) -> void:
	rear_tire_side_friction_exponent = value
	notify_property_list_changed()

func _set_rear_tire_vertical_stiffness(value: float) -> void:
	rear_tire_vertical_stiffness = value
	notify_property_list_changed()

func _set_rear_tire_width(value: float) -> void:
	rear_tire_width = value
	notify_property_list_changed()

func _set_rear_tire_pressure_bar(value: float) -> void:
	rear_tire_pressure_bar = value
	notify_property_list_changed()

func _set_rear_tire_temperature_optimal(value: float) -> void:
	rear_tire_temperature_optimal = value
	notify_property_list_changed()

func _set_rear_tire_temperature_min(value: float) -> void:
	rear_tire_temperature_min = value
	notify_property_list_changed()

func _set_rear_tire_temperature_max(value: float) -> void:
	rear_tire_temperature_max = value
	notify_property_list_changed()

func _set_aero_drag_coefficient(value: float) -> void:
	aero_drag_coefficient = value
	notify_property_list_changed()

func _set_aero_frontal_area(value: float) -> void:
	aero_frontal_area = value
	notify_property_list_changed()

func _set_aero_downforce_coefficient(value: float) -> void:
	aero_downforce_coefficient = value
	notify_property_list_changed()

func _set_aero_lift_coefficient(value: float) -> void:
	aero_lift_coefficient = value
	notify_property_list_changed()

func _set_aero_front_spreader(value: float) -> void:
	aero_front_spreader = value
	notify_property_list_changed()

func _set_aero_rear_spreader(value: float) -> void:
	aero_rear_spreader = value
	notify_property_list_changed()

func _set_aero_ground_clearance(value: float) -> void:
	aero_ground_clearance = value
	notify_property_list_changed()

func _set_aero_floor_effect_distance(value: float) -> void:
	aero_floor_effect_distance = value
	notify_property_list_changed()

func _set_aero_wing_incidence_front(value: float) -> void:
	aero_wing_incidence_front = value
	notify_property_list_changed()

func _set_aero_wing_incidence_rear(value: float) -> void:
	aero_wing_incidence_rear = value
	notify_property_list_changed()

func _set_aero_drag_decay_speed(value: float) -> void:
	aero_drag_decay_speed = value
	notify_property_list_changed()

func _set_aero_downforce_decay_speed(value: float) -> void:
	aero_downforce_decay_speed = value
	notify_property_list_changed()

func _set_steering_ratio(value: float) -> void:
	steering_ratio = value
	notify_property_list_changed()

func _set_steering_lock_angle(value: float) -> void:
	steering_lock_angle = value
	notify_property_list_changed()

func _set_steering_lock_degrees(value: float) -> void:
	steering_lock_degrees = value
	notify_property_list_changed()

func _set_steering_deadzone(value: float) -> void:
	steering_deadzone = value
	notify_property_list_changed()

func _set_steering_return_speed(value: float) -> void:
	steering_return_speed = value
	notify_property_list_changed()

func _set_steering_weight_speed_threshold(value: float) -> void:
	steering_weight_speed_threshold = value
	notify_property_list_changed()

func _set_steering_progressive_ratio(value: bool) -> void:
	steering_progressive_ratio = value
	notify_property_list_changed()

func _set_steering_assist_enabled(value: bool) -> void:
	steering_assist_enabled = value
	notify_property_list_changed()

func _set_steering_assist_strength(value: float) -> void:
	steering_assist_strength = value
	notify_property_list_changed()

func _set_brake_force_distribution(value: Vector2) -> void:
	brake_force_distribution = value
	notify_property_list_changed()

func _set_brake_master_cylinder_diameter(value: float) -> void:
	brake_master_cylinder_diameter = value
	notify_property_list_changed()

func _set_brake_caliper_piston_area(value: float) -> void:
	brake_caliper_piston_area = value
	notify_property_list_changed()

func _set_brake_pad_friction(value: float) -> void:
	brake_pad_friction = value
	notify_property_list_changed()

func _set_brake_disc_radius(value: float) -> void:
	brake_disc_radius = value
	notify_property_list_changed()

func _set_brake_disc_thickness(value: float) -> void:
	brake_disc_thickness = value
	notify_property_list_changed()

func _set_brake_max_pressure_bar(value: float) -> void:
	brake_max_pressure_bar = value
	notify_property_list_changed()

func _set_brake_line_pressure_loss(value: float) -> void:
	brake_line_pressure_loss = value
	notify_property_list_changed()

func _set_brake_compound_temperature_optimal(value: float) -> void:
	brake_compound_temperature_optimal = value
	notify_property_list_changed()

func _set_brake_compound_fade_temp(value: float) -> void:
	brake_compound_fade_temp = value
	notify_property_list_changed()

func _set_brake_recovery_time(value: float) -> void:
	brake_recovery_time = value
	notify_property_list_changed()

func _set_abs_enabled(value: bool) -> void:
	abs_enabled = value
	notify_property_list_changed()

func _set_abs_threshold(value: float) -> void:
	abs_threshold = value
	notify_property_list_changed()

func _set_abs_modulation_rate(value: float) -> void:
	abs_modulation_rate = value
	notify_property_list_changed()

func _set_brake_balance_adjustable(value: bool) -> void:
	brake_balance_adjustable = value
	notify_property_list_changed()

func _set_brake_balance_range(value: Vector2) -> void:
	brake_balance_range = value
	notify_property_list_changed()

func _set_clutch_capacity(value: float) -> void:
	clutch_capacity = value
	notify_property_list_changed()

func _set_clutch_engagement_point(value: float) -> void:
	clutch_engagement_point = value
	notify_property_list_changed()

func _set_clutch_spring_stiffness(value: float) -> void:
	clutch_spring_stiffness = value
	notify_property_list_changed()

func _set_flywheel_inertia(value: float) -> void:
	flywheel_inertia = value
	notify_property_list_changed()

func _set_rev_limit_shutoff_rpm(value: float) -> void:
	rev_limit_shutoff_rpm = value
	notify_property_list_changed()

func _set_rev_limit_redline(value: float) -> void:
	rev_limit_redline = value
	notify_property_list_changed()

func _set_downshift_rpm_threshold(value: float) -> void:
	downshift_rpm_threshold = value
	notify_property_list_changed()

func _set_upshift_rpm_threshold(value: float) -> void:
	upshift_rpm_threshold = value
	notify_property_list_changed()

func _set_launch_control_enabled(value: bool) -> void:
	launch_control_enabled = value
	notify_property_list_changed()

func _set_launch_control_rpm(value: float) -> void:
	launch_control_rpm = value
	notify_property_list_changed()

func _set_traction_control_enabled(value: bool) -> void:
	traction_control_enabled = value
	notify_property_list_changed()

func _set_traction_control_level(value: int) -> void:
	traction_control_level = value
	notify_property_list_changed()

func _set_traction_control_max_slip(value: float) -> void:
	traction_control_max_slip = value
	notify_property_list_changed()

func _set_track_surface_friction(value: float) -> void:
	track_surface_friction = value
	notify_property_list_changed()

func _set_track_temperature_base(value: float) -> void:
	track_temperature_base = value
	notify_property_list_changed()

func _set_track_wetness_factor(value: float) -> void:
	track_wetness_factor = value
	notify_property_list_changed()

func _set_wind_variation(value: float) -> void:
	wind_variation = value
	notify_property_list_changed()

func _set_simulation_accuracy(value: int) -> void:
	simulation_accuracy = value
	notify_property_list_changed()

func _get_property_list() -> Array[Dictionary]:
	var props: Array[Dictionary] = []
	
	props.append({
		"name": "gear_ratios",
		"type": TYPE_ARRAY,
		"hint": PROPERTY_HINT_ARRAY,
		"user_type": "float",
		"usage": PROPERTY_USAGE_STORAGE | PROPERTY_USAGE_EDITOR
	})
	
	props.append({
		"name": "torque_curve_cache",
		"type": TYPE_ARRAY,
		"hint": PROPERTY_HINT_NONE,
		"user_type": "Vector2",
		"usage": PROPERTY_USAGE_NO_EDITOR
	})
	
	return props

func save_to_file(path: String) -> void:
	var file = FileAccess.open(path, FileAccess.WRITE)
	if file != null:
		file.store_string(to_json())
		file.close()

static func load_from_file(path: String) -> PhysicsSettings:
	var file = FileAccess.open(path, FileAccess.READ)
	if file != null:
		var json = JSON.new()
		var error = json.parse(file.get_as_text())
		if error == OK:
			var data = json.data
			var settings = PhysicsSettings.new()
			settings.from_json(data)
			file.close()
			return settings
		file.close()
	return null

func from_json(data: Dictionary) -> void:
	if data.has("gravity"):
		gravity = data["gravity"]
	if data.has("default_vehicle_mass"):
		default_vehicle_mass = data["default_vehicle_mass"]
	if data.has("gear_ratios"):
		_gear_ratios = data["gear_ratios"]
	if data.has("aero_drag_coefficient"):
		aero_drag_coefficient = data["aero_drag_coefficient"]

func to_json() -> String:
	var data: Dictionary = {}
	data["gravity"] = gravity
	data["default_vehicle_mass"] = default_vehicle_mass
	data["gear_ratios"] = _gear_ratios
	data["aero_drag_coefficient"] = aero_drag_coefficient
	data["timestamp"] = Time.get_unix_time_from_system()
	return JSON.stringify(data, "\t")