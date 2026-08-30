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
@export var pre_load_torque: float = 50.0: set = _set_pre_load_torque
@export var acceleration_lock: float = 0.75: set = _set_acceleration_lock
@export var deceleration_lock: float = 0.50: set = _set_deceleration_lock
@export var open_diff_friction: float = 0.1: set = _set_open_diff_friction
@export var locked_diff_friction: float = 0.0: set = _set_locked_diff_friction

@export_group("Suspension Parameters")
@export_group("Front Suspension")
@export var front_spring_stiffness: float = 45000.0: set = _set_front_spring_stiffness
@export var front_damping_compression: float = 3500.0: set = _set_front_damping_compression
@export var front_damping_rebound: float = 4500.0: set = _set_front_damping_rebound
@export var front_suspension_travel: float = 0.15: set = _set_front_suspension_travel
@export var front_camber_static: float = -2.0: set = _set_front_camber_static
@export var front_camber_dynamic: float = 1.5: set = _set_front_camber_dynamic
@export var front_toe_static: float = 0.1: set = _set_front_toe_static
@export var front_toe_dynamic: float = 0.05: set = _set_front_toe_dynamic
@export var front_anti_roll_bar_stiffness: float = 8000.0: set = _set_front_anti_roll_bar_stiffness

@export_group("Rear Suspension")
@export var rear_spring_stiffness: float = 55000.0: set = _set_rear_spring_stiffness
@export var rear_damping_compression: float = 4000.0: set = _set_rear_damping_compression
@export var rear_damping_rebound: float = 5000.0: set = _set_rear_damping_rebound
@export var rear_suspension_travel: float = 0.15: set = _set_rear_suspension_travel
@export var rear_camber_static: float = -1.5: set = _set_rear_camber_static
@export var rear_camber_dynamic: float = 1.2: set = _set_rear_camber_dynamic
@export var rear_toe_static: float = 0.05: set = _set_rear_toe_static
@export var rear_toe_dynamic: float = 0.03: set = _set_rear_toe_dynamic
@export var rear_anti_roll_bar_stiffness: float = 10000.0: set = _set_rear_anti_roll_bar_stiffness

@export_group("Tire Friction Curves (Pacejka Simplified)")
@export_group("Longitudinal Friction")
@export var longitudinal_b_coefficient: float = 9.5: set = _set_longitudinal_b_coefficient
@export var longitudinal_c_coefficient: float = 1.9: set = _set_longitudinal_c_coefficient
@export var longitudinal_d_coefficient: float = 1.1: set = _set_longitudinal_d_coefficient
@export var longitudinal_e_coefficient: float = -0.2: set = _set_longitudinal_e_coefficient

@export_group("Lateral Friction")
@export var lateral_b_coefficient: float = 11.0: set = _set_lateral_b_coefficient
@export var lateral_c_coefficient: float = 1.9: set = _set_lateral_c_coefficient
@export var lateral_d_coefficient: float = 1.15: set = _set_lateral_d_coefficient
@export var lateral_e_coefficient: float = -0.15: set = _set_lateral_e_coefficient

@export_group("Friction Properties")
@export var peak_friction_coefficient: float = 1.2: set = _set_peak_friction_coefficient
@export var sliding_friction_coefficient: float = 0.8: set = _set_sliding_friction_coefficient
@export var combined_friction_factor: float = 0.9: set = _set_combined_friction_factor
@export var tire_pressure: float = 2.3: set = _set_tire_pressure
@export var tire_temperature_optimal: float = 80.0: set = _set_tire_temperature_optimal
@export var tire_temperature_min: float = -20.0: set = _set_tire_temperature_min
@export var tire_temperature_max: float = 120.0: set = _set_tire_temperature_max
@export var tire_temp_ramp_rate: float = 0.5: set = _set_tire_temp_ramp_rate

@export_group("Aerodynamics")
@export var aero_drag_coefficient: float = 0.32: set = _set_aero_drag_coefficient
@export var aero_frontal_area: float = 2.2: set = _set_aero_frontal_area
@export var aero_downforce_coefficient: float = 0.8: set = _set_aero_downforce_coefficient
@export var aero_lift_coefficient: float = -0.15: set = _set_aero_lift_coefficient
@export var aero_cg_height: float = 0.35: set = _set_aero_cg_height
@export var aero_front_splitter_angle: float = 2.0: set = _set_aero_front_splitter_angle
@export var aero_rear_wing_angle: float = 15.0: set = _set_aero_rear_wing_angle
@export var aero_ground_clearance: float = 0.05: set = _set_aero_ground_clearance
@export var aero_side_skirt_gap: float = 0.02: set = _set_aero_side_skirt_gap
@export var aero_diffuser_angle: float = 10.0: set = _set_aero_diffuser_angle
@export var aero_air_density_sea_level: float = 1.225: set = _set_aero_air_density_sea_level
@export var aero_altitude_reduction_factor: float = 0.00012: set = _set_aero_altitude_reduction_factor

@export_group("Steering System")
@export var steering_ratio: float = 14.0: set = _set_steering_ratio
@export var steering_lock_left_degrees: float = 450.0: set = _set_steering_lock_left_degrees
@export var steering_lock_right_degrees: float = 450.0: set = _set_steering_lock_right_degrees
@export var steering_speed_multiplier: float = 1.0: set = _set_steering_speed_multiplier
@export var steering_deadzone: float = 0.05: set = _set_steering_deadzone
@export var steering_return_speed: float = 12.0: set = _set_steering_return_speed
@export var steering_center_position: float = 0.5: set = _set_steering_center_position
@export var steering_max_input: float = 1.0: set = _set_steering_max_input
@export var steering_weight_curve_points: Array[Vector2] = [
	Vector2(0.0, 0.5), Vector2(0.25, 0.7), Vector2(0.5, 0.85), 
	Vector2(0.75, 0.95), Vector2(1.0, 1.0)
]

@export_group("Brake System")
@export var brake_force_distribution_front: float = 0.6: set = _set_brake_force_distribution_front
@export var brake_force_distribution_rear: float = 0.4: set = _set_brake_force_distribution_rear
@export var max_brake_pressure: float = 100.0: set = _set_max_brake_pressure
@export var brake_master_cylinder_ratio: float = 1.0: set = _set_brake_master_cylinder_ratio
@export var brake_caliper_piston_area: float = 0.003: set = _set_brake_caliper_piston_area
@export var brake_disc_radius: float = 0.15: set = _set_brake_disc_radius
@export var brake_pad_friction_coefficient: float = 0.4: set = _set_brake_pad_friction_coefficient
@export var brake_pad_temperature_optimal: float = 150.0: set = _set_brake_pad_temperature_optimal
@export var brake_pad_temperature_max: float = 650.0: set = _set_brake_pad_temperature_max
@export var brake_pad_ramp_rate: float = 2.0: set = _set_brake_pad_ramp_rate
@export var brake_bleed_time: float = 1.0: set = _set_brake_bleed_time
@export var abs_threshold: float = 0.15: set = _set_abs_threshold
@export var abs_recovery_threshold: float = 0.05: set = _set_abs_recovery_threshold

@export_group("Chassis Rigidity")
@export var chassis_vertical_stiffness: float = 50000000.0: set = _set_chassis_vertical_stiffness
@export var chassis_horizontal_stiffness: float = 30000000.0: set = _set_chassis_horizontal_stiffness
@export var chassis_torsional_stiffness: float = 25000000.0: set = _set_chassis_torsional_stiffness
@export var suspension_mount_stiffness: float = 100000000.0: set = _set_suspension_mount_stiffness

@export_group("Transmission")
@export var transmission_efficiency: float = 0.92: set = _set_transmission_efficiency
@export var clutch_engagement_point: float = 0.7: set = _set_clutch_engagement_point
@export var clutch_slip_tolerance: float = 0.15: set = _set_clutch_slip_tolerance
@export var synchromesh_shift_rpm_window: float = 500.0: set = _set_synchromesh_shift_rpm_window
@export var auto_shift_delay_ms: int = 150: set = _set_auto_shift_delay_ms
@export var manual_shift_delay_ms: int = 50: set = _set_manual_shift_delay_ms

@export_group("Fuel System")
@export var fuel_tank_capacity: float = 60.0: set = _set_fuel_tank_capacity
@export var initial_fuel_amount: float = 45.0: set = _set_initial_fuel_amount
@export var fuel_density: float = 0.74: set = _set_fuel_density
@export var fuel_viscosity: float = 0.0005: set = _set_fuel_viscosity
@export var fuel_pump_flow_rate: float = 0.0002: set = _set_fuel_pump_flow_rate
@export var fuel_injection_timing_advance: float = 15.0: set = _set_fuel_injection_timing_advance

@export_group("Cooling System")
@export var coolant_capacity: float = 8.0: set = _set_coolant_capacity
@export var oil_capacity: float = 5.5: set = _set_oil_capacity
@export var radiator_surface_area: float = 0.8: set = _set_radiator_surface_area
@export var radiator_efficiency: float = 0.85: set = _set_radiator_efficiency
@export var fan_efficiency: float = 0.75: set = _set_fan_efficiency
@export var optimal_water_temp: float = 90.0: set = _set_optimal_water_temp
@export var max_water_temp: float = 115.0: set = _set_max_water_temp
@export var optimal_oil_temp: float = 100.0: set = _set_optimal_oil_temp
@export var max_oil_temp: float = 130.0: set = _set_max_oil_temp

@export_group("Damage & Wear")
@export var tire_wear_rate: float = 0.00001: set = _set_tire_wear_rate
@export var brake_wear_rate: float = 0.00002: set = _set_brake_wear_rate
@export var engine_rpm_stress_factor: float = 0.000005: set = _set_engine_rpm_stress_factor
@export var collision_damage_threshold: float = 50.0: set = _set_collision_damage_threshold
@export var crash_force_multiplier: float = 1.5: set = _set_crash_force_multiplier

var _last_saved_timestamp: int = 0
var _version: String = "1.0.0"

func _ready() -> void:
	_last_saved_timestamp = Time.get_unix_time_from_system()

## Unit Conversion Helpers

func rpm_to_rad_per_sec(rpm: float) -> float:
	return rpm * PI / 30.0

func rad_per_sec_to_rpm(rad_per_sec: float) -> float:
	return rad_per_sec * 30.0 / PI

func deg_to_rad(degrees: float) -> float:
	return degrees * PI / 180.0

func rad_to_deg(radians: float) -> float:
	return radians * 180.0 / PI

func mm_to_m(mm: float) -> float:
	return mm / 1000.0

def m_to_mm(meters: float) -> float:
	return meters * 1000.0

func kmh_to_ms(kmh: float) -> float:
	return kmh / 3.6

func ms_to_kmh(ms: float) -> float:
	return ms * 3.6

func bar_to_pa(bar: float) -> float:
	return bar * 100000.0

func pa_to_bar(pa: float) -> float:
	return pa / 100000.0

func psi_to_bar(psi: float) -> float:
	return psi * 0.0689476

func bar_to_psi(bar: float) -> float:
	return bar / 0.0689476

func kg_to_lb(kg: float) -> float:
	return kg * 2.20462

func lb_to_kg(lb: float) -> float:
	return lb / 2.20462

func newton_to_lb_ft(newton_meters: float) -> float:
	return newton_meters * 0.737562

func lb_ft_to_newton(lb_ft: float) -> float:
	return lb_ft / 0.737562

func joule_to_watt_hour(joules: float) -> float:
	return joules / 3600.0

func watt_hour_to_joule(wh: float) -> float:
	return wh * 3600.0

## Aerodynamic Force Calculations

func calculate_drag_force(velocity: float) -> float:
	var air_density = get_effective_air_density()
	return 0.5 * air_density * velocity * velocity * aero_drag_coefficient * aero_frontal_area

func calculate_downforce_force(velocity: float) -> float:
	var air_density = get_effective_air_density()
	return 0.5 * air_density * velocity * velocity * aero_downforce_coefficient * aero_frontal_area

func calculate_lift_force(velocity: float) -> float:
	var air_density = get_effective_air_density()
	return 0.5 * air_density * velocity * velocity * aero_lift_coefficient * aero_frontal_area

func get_effective_air_density() -> float:
	var altitude: float = GameManager.get_active_vehicle().position.y if GameManager.active_vehicles.size() > 0 else 0.0
	return aero_air_density_sea_level * exp(-altitude * aero_altitude_reduction_factor)

func calculate_total_aero_force(velocity: float, direction: Vector3) -> Vector3:
	var drag = calculate_drag_force(abs(velocity.length()))
	var downforce = calculate_downforce_force(abs(velocity.length()))
	var lift = calculate_lift_force(abs(velocity.length()))
	
	var total_force = Vector3.ZERO
	total_force.x = -drag * direction.x
	total_force.y = downforce + lift
	total_force.z = -drag * direction.z
	
	return total_force.normalized() * abs(velocity.length())

## Tire Friction Calculations (Pacejka Simplified)

func calculate_longitudinal_friction(slip_ratio: float, vertical_force: float) -> float:
	var normal = slip_ratio.abs()
	var angle = b_coefficient * normal + c_coefficient * pow(normal, d_coefficient)
	if angle > PI / 2.0:
		angle = PI / 2.0
	var sin_value = sin(angle)
	var curve_shape = e_coefficient
	var adjusted_slip = normal * curve_shape
	if adjusted_slip > 1.0:
		adjusted_slip = 1.0
	elif adjusted_slip < 0.0:
		adjusted_slip = 0.0
	var result = d_coefficient * sin_value * (1.0 - (1.0 - e_coefficient) * pow((1.0 - e_coefficient), 2.0))
	
	return result * peak_friction_coefficient * vertical_force

func calculate_lateral_friction(slip_angle: float, vertical_force: float) -> float:
	var normal = slip_angle.abs()
	var angle = b_coefficient * normal + c_coefficient * pow(normal, d_coefficient)
	if angle > PI / 2.0:
		angle = PI / 2.0
	var sin_value = sin(angle)
	var curve_shape = e_coefficient
	var adjusted_slip = normal * curve_shape
	if adjusted_slip > 1.0:
		adjusted_slip = 1.0
	elif adjusted_slip < 0.0:
		adjusted_slip = 0.0
	var result = d_coefficient * sin_value * (1.0 - (1.0 - e_coefficient) * pow((1.0 - e_coefficient), 2.0))
	
	return result * peak_friction_coefficient * vertical_force

func calculate_combined_friction(longitudinal_slip: float, lateral_slip: float, vertical_force: float) -> float:
	var longitudinal_force = calculate_longitudinal_friction(longitudinal_slip, vertical_force)
	var lateral_force = calculate_lateral_friction(lateral_slip, vertical_force)
	
	var combined_slip = sqrt(pow(longitudinal_slip, 2) + pow(lateral_slip, 2))
	var combined_factor = 1.0 - combined_friction_factor * combined_slip
	
	return min(longitudinal_force, lateral_force) * combined_factor

## Braking Force Calculations

func calculate_brake_force(pedal_input: float, speed: float) -> float:
	var pressure = pedal_input * max_brake_pressure
	var temp_factor = get_brake_temp_factor()
	var pad_condition = get_brake_pad_condition()
	
	var brake_force = pressure * brake_pad_friction_coefficient * brake_caliper_piston_area * brake_disc_radius
	brake_force *= temp_factor * pad_condition
	
	return clamp(brake_force, 0.0, max_brake_pressure * brake_master_cylinder_ratio)

func get_brake_temp_factor() -> float:
	var current_temp = GameManager.get_active_vehicle().brake_temps.front() if GameManager.active_vehicles.size() > 0 else brake_pad_temperature_optimal
	if current_temp < brake_pad_temperature_optimal:
		return 0.8 + 0.2 * (current_temp / brake_pad_temperature_optimal)
	elif current_temp > brake_pad_temperature_max:
		return 0.3
	else:
		return 1.0

func get_brake_pad_condition() -> float:
	var wear_percentage = GameManager.get_active_vehicle().brake_wear.front() if GameManager.active_vehicles.size() > 0 else 0.0
	return 1.0 - (wear_percentage / 100.0) * 0.3

## Suspension Force Calculation

func calculate_suspension_force(displacement: float, velocity: float, stiffness: float, damping_compression: float, damping_rebound: float) -> float:
	var spring_force = stiffness * displacement
	var damping_direction = sign(velocity)
	var damping_force = damping_compression if displacement >= 0 else damping_rebound
	damping_force *= abs(velocity) * damping_direction
	
	return spring_force + damping_force

func calculate_anti_roll_bar_force(roll_angle: float, stiffness: float) -> float:
	return stiffness * roll_angle

## Gear Ratio Helpers

func get_gear_ratio(gear: int) -> float:
	if gear < 0 or gear >= num_gears:
		return gear_ratios[num_gears - 1]
	return gear_ratios[gear]

func get_final_drive() -> float:
	return final_drive_ratio

func get_total_ratio(gear: int) -> float:
	return get_gear_ratio(gear) * final_drive_ratio

func get_wheel_rpm(engine_rpm: float, gear: int) -> float:
	return engine_rpm / get_total_ratio(gear)

func get_engine_rpm(wheel_rpm: float, gear: int) -> float:
	return wheel_rpm * get_total_ratio(gear)

func get_vehicle_speed(engine_rpm: float, gear: int) -> float:
	var wheel_rpm = get_wheel_rpm(engine_rpm, gear)
	var circumference = 2.0 * PI * default_wheel_radius
	var m_per_sec = (wheel_rpm / 60.0) * circumference
	return ms_to_kmh(m_per_sec)

## Engine Torque Curve

func get_torque_at_rpm(rpm: float) -> float:
	var normalized_rpm = (rpm - engine_min_rpm) / (engine_max_rpm - engine_min_rpm)
	normalized_rpm = clamp(normalized_rpm, 0.0, 1.0)
	
	var torque = interpolate_torque_curve(normalized_rpm) * peak_torque_nm
	
	if rpm >= rev_limit_rpm:
		torque *= 0.5
	
	return torque

func get_power_at_rpm(rpm: float) -> float:
	var torque = get_torque_at_rpm(rpm)
	var power_watts = torque * rpm_to_rad_per_sec(rpm)
	return power_watts / 1000.0

func interpolate_torque_curve(normalized_rpm: float) -> float:
	if torque_curve_points.is_empty():
		return 1.0
	
	for i in range(torque_curve_points.size() - 1):
		var p1 = torque_curve_points[i]
		var p2 = torque_curve_points[i + 1]
		
		if normalized_rpm >= p1.x and normalized_rpm <= p2.x:
			var t = (normalized_rpm - p1.x) / (p2.x - p1.x)
			return p1.y + t * (p2.y - p1.y)
	
	return torque_curve_points.back().y

## Steering Helpers

func calculate_steering_angle(input: float) -> float:
	input = clamp(input, -steering_max_input, steering_max_input)
	
	if abs(input) < steering_deadzone:
		return 0.0
	
	var input_normalized = abs(input)
	var weight = interpolate_steering_weight(input_normalized)
	
	var max_angle = steering_lock_left_degrees if input < 0 else steering_lock_right_degrees
	var angle = weight * max_angle
	
	return -angle if input < 0 else angle

func interpolate_steering_weight(input_normalized: float) -> float:
	if steering_weight_curve_points.is_empty():
		return input_normalized
	
	for i in range(steering_weight_curve_points.size() - 1):
		var p1 = steering_weight_curve_points[i]
		var p2 = steering_weight_curve_points[i + 1]
		
		if input_normalized >= p1.x and input_normalized <= p2.x:
			var t = (input_normalized - p1.x) / (p2.x - p1.x)
			return p1.y + t * (p2.y - p1.y)
	
	return steering_weight_curve_points.back().y

## Vehicle Dynamics Helpers

func calculate_cornering_force(velocity: float, slip_angle: float, vertical_load: float) -> float:
	var cornering_stiffness = peak_friction_coefficient * vertical_load
	return cornering_stiffness * tan(deg_to_rad(slip_angle))

func calculate_understeer_gradient() -> float:
	var front_camber = front_camber_static + front_camber_dynamic
	var rear_camber = rear_camber_static + rear_camber_dynamic
	
	var front_friction = peak_friction_coefficient * cos(deg_to_rad(front_camber))
	var rear_friction = peak_friction_coefficient * cos(deg_to_rad(rear_camber))
	
	return (rear_friction - front_friction) * 100.0

func calculate_oversteer_margin() -> float:
	var total_mass = default_vehicle_mass
	var front_weight_ratio = 0.45
	var rear_weight_ratio = 0.55
	
	var front_vertical_load = total_mass * gravity * front_weight_ratio
	var rear_vertical_load = total_mass * gravity * rear_weight_ratio
	
	var front_grip = calculate_cornering_force(10.0, 5.0, front_vertical_load)
	var rear_grip = calculate_cornering_force(10.0, 5.0, rear_vertical_load)
	
	return rear_grip - front_grip

## Fuel Consumption

func calculate_fuel_consumption(engine_rpm: float, throttle_input: float) -> float:
	var base_consumption = fuel_consumption_rate * throttle_input
	var rpm_factor = (engine_rpm / engine_max_rpm) * 0.5
	var load_factor = throttle_input * 0.5
	
	return base_consumption * (1.0 + rpm_factor + load_factor)

func get_remaining_fuel(fuel_amount: float) -> float:
	return max(0.0, fuel_amount)

func is_fuel_empty(fuel_amount: float) -> bool:
	return fuel_amount < 1.0

## Cooling System

func calculate_engine_heat(engine_rpm: float, throttle_input: float) -> float:
	var heat_generation = engine_rpm * throttle_input * engine_rpm_stress_factor
	return heat_generation

func calculate_cooling_rate(ambient_temp: float, fan_on: bool) -> float:
	var radiator_temp = ambient_temp + 20.0
	var cooling_efficiency = radiator_efficiency
	if fan_on:
		cooling_efficiency *= fan_efficiency
	
	return cooling_efficiency * (radiator_temp - ambient_temp)

func get_optimal_engine_temp() -> float:
	return optimal_water_temp

func is_engine_overheating(temp: float) -> bool:
	return temp > max_water_temp

## Validation Methods

func validate_settings() -> Dictionary:
	var errors: Array[String] = []
	var warnings: Array[String] = []
	
	if gravity <= 0:
		errors.append("Gravity must be positive")
	
	if engine_max_rpm < engine_min_rpm:
		errors.append("Max RPM must be greater than min RPM")
	
	if gear_ratios.size() != num_gears:
		errors.append("Gear ratios array size must match num_gears")
	
	if default_vehicle_mass <= 0:
		errors.append("Vehicle mass must be positive")
	
	if wheel_track_front <= 0 or wheel_track_rear <= 0:
		errors.append("Wheel tracks must be positive")
	
	if wheelbase <= 0:
		errors.append("Wheelbase must be positive")
	
	if peak_torque_nm <= 0:
		errors.append("Peak torque must be positive")
	
	if max_power_kw <= 0:
		errors.append("Max power must be positive")
	
	if aero_frontal_area <= 0:
		errors.append("Frontal area must be positive")
	
	if peak_friction_coefficient <= 0:
		errors.append("Peak friction coefficient must be positive")
	
	if braking_force_distribution_front < 0 or braking_force_distribution_front > 1:
		errors.append("Brake distribution front must be between 0 and 1")
	
	if braking_force_distribution_rear < 0 or braking_force_distribution_rear > 1:
		errors.append("Brake distribution rear must be between 0 and 1")
	
	if abs(braking_force_distribution_front + braking_force_distribution_rear - 1.0) > 0.01:
		warnings.append("Brake distribution should sum to approximately 1.0")
	
	return {
		"errors": errors,
		"warnings": warnings,
		"is_valid": errors.is_empty()
	}

## Serialization Helpers

func save_to_file(path: String) -> void:
	var config = ConfigFile.new()
	config.set_value("global", "version", _version)
	config.set_value("global", "timestamp", _last_saved_timestamp)
	
	# Save all exportable properties recursively
	_save_property_recursive(config, "", self)
	
	var err = config.save(path)
	if err != OK:
		push_error("Failed to save PhysicsSettings: %s" % Error.get_message(err))
	else:
		_last_saved_timestamp = Time.get_unix_time_from_system()

func _save_property_recursive(config: ConfigFile, prefix: String, node: Object) -> void:
	for key in node.get_property_list():
		var prop_info = key["name"]
		var property_value = node.get(key["name"])
		var property_type = key["type"]
		
		if property_type == TYPE_FLOAT:
			config.set_value(prefix, prop_info, property_value)
		elif property_type == TYPE_INT:
			config.set_value(prefix, prop_info, property_value)
		elif property_type == TYPE_BOOL:
			config.set_value(prefix, prop_info, property_value)
		elif property_type == TYPE_ARRAY:
			var array_str = ""
			for item in property_value:
				array_str += str(item) + ";"
			config.set_value(prefix, prop_info, array_str)

func load_from_file(path: String) -> void:
	var config = ConfigFile.new()
	var err = config.load(path)
	
	if err != OK:
		push_warning("Failed to load PhysicsSettings from %s" % path)
		return
	
	if config.has_section_key("global", "version"):
		_version = config.get_value("global", "version")
	
	_load_property_recursive(config, "", self)

func _load_property_recursive(config: ConfigFile, prefix: String, node: Object) -> void:
	for key in node.get_property_list():
		var prop_info = key["name"]
		var property_type = key["type"]
		
		if config.has_section_key(prefix, prop_info):
			var value = config.get_value(prefix, prop_info)
			
			if property_type == TYPE_FLOAT and typeof(value) == TYPE_FLOAT:
				node.set(prop_info, value)
			elif property_type == TYPE_INT and typeof(value) == TYPE_INT:
				node.set(prop_info, value)
			elif property_type == TYPE_BOOL and typeof(value) == TYPE_BOOL:
				node.set(prop_info, value)
			elif property_type == TYPE_ARRAY and typeof(value) == TYPE_STRING:
				var parts = value.split(";")
				var array = []
				for part in parts:
					if not part.is_empty():
						array.append(float(part))
				node.set(prop_info, array)

## Preset Management

func create_preset(name: String) -> PhysicsSettings:
	var preset = PhysicsSettings.new()
	preset.name = name
	
	preset.gravity = gravity
	preset.default_vehicle_mass = default_vehicle_mass
	preset.engine_max_rpm = engine_max_rpm
	preset.peak_torque_nm = peak_torque_nm
	preset.aero_drag_coefficient = aero_drag_coefficient
	preset.peak_friction_coefficient = peak_friction_coefficient
	
	return preset

func apply_preset(preset: PhysicsSettings) -> void:
	if preset == null:
		return
	
	gravity = preset.gravity
	default_vehicle_mass = preset.default_vehicle_mass
	engine_max_rpm = preset.engine_max_rpm
	peak_torque_nm = preset.peak_torque_nm
	aero_drag_coefficient = preset.aero_drag_coefficient
	peak_friction_coefficient = preset.peak_friction_coefficient

## Debug Helpers

func print_debug_info() -> void:
	print("\n=== PhysicsSettings Debug Info ===")
	print("Version: %s" % _version)
	print("Tick Rate: %d Hz" % physics_tick_rate)
	print("Substeps: %d" % max_substeps)
	print("Time Scale: %.2f" % time_scale)
	print("\nVehicle Mass: %.2f kg" % default_vehicle_mass)
	print("Wheel Radius: %.2f m" % default_wheel_radius)
	print("Wheelbase: %.2f m" % wheelbase)
	print("\nEngine Max RPM: %.0f" % engine_max_rpm)
	print("Peak Torque: %.1f Nm @ %.0f RPM" % [peak_torque_nm, peak_torque_rpm])
	print("Max Power: %.1f kW" % max_power_kw)
	print("Gears: %d" % num_gears)
	print("Gear Ratios: %s" % str(gear_ratios))
	print("\nFinal Drive: %.2f" % final_drive_ratio)
	print("\nAero Drag Coefficient: %.2f" % aero_drag_coefficient)
	print("Aero Downforce Coefficient: %.2f" % aero_downforce_coefficient)
	print("\nPeak Friction: %.2f" % peak_friction_coefficient)
	print("Sliding Friction: %.2f" % sliding_friction_coefficient)
	print("\nFront Suspension Travel: %.2f m" % front_suspension_travel)
	print("Rear Suspension Travel: %.2f m" % rear_suspension_travel)
	print("\nSteering Lock: %.0f°" % steering_lock_left_degrees)
	print("Brake Distribution Front: %.0f%%" % (brake_force_distribution_front * 100))
	print("Brake Distribution Rear: %.0f%%" % (brake_force_distribution_rear * 100))
	print("================================\n")

## Inspector Export Helper

func _get_property_list() -> Array[Dictionary]:
	var list = super()._get_property_list()
	list.append({
		"name": "_debug_print",
		"type": TYPE_FUNC,
		"hint_string": "print_debug_info()"
	})
	return list

func _set_property(property: String, value: Variant) -> bool:
	return true

func _get_property(property: String) -> Variant:
	return null