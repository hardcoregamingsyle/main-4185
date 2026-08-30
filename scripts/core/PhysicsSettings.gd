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
@export var default_wheel_radius: float = 0.33: set = _set_default_wheel_radius
@export var default_wheel_width: float = 0.22: set = _set_default_wheel_width
@export var default_center_of_mass_height: float = 0.45: set = _set_default_com_height
@export var default_center_of_mass_x: float = 0.0: set = _set_default_com_x
@export var default_track_width: float = 1.5: set = _set_default_track_width
@export var default_wheelbase: float = 2.5: set = _set_default_wheelbase

@export_group("Engine & Powertrain")
@export var engine_max_rpm: float = 8500.0: set = _set_engine_max_rpm
@export var engine_idle_rpm: float = 800.0: set = _set_engine_idle_rpm
@export var engine_peak_torque_rpm: float = 4500.0: set = _set_engine_peak_torque_rpm
@export var engine_peak_torque_nm: float = 450.0: set = _set_engine_peak_torque
@export var engine_max_torque_factor: float = 1.3: set = _set_engine_max_torque_factor
@export var transmission_type: TransmissionType = TransmissionType.MANUAL
@export var final_drive_ratio: float = 3.45: set = _set_final_drive_ratio
@export var gear_ratios: Array[float] = [3.85, 2.35, 1.68, 1.31, 1.05, 0.88]: set = _set_gear_ratios
@export var clutch_friction_coefficient: float = 1.2: set = _set_clutch_friction
@export var flywheel_inertia_kg_m2: float = 0.18: set = _set_flywheel_inertia

@export_group("Differential Settings")
@export var diff_type: DifferentialType = DifferentialType.LSD
@export var diff_lock_front: float = 0.0: set = _set_diff_lock_front
@export var diff_lock_rear: float = 0.45: set = _set_diff_lock_rear
@export var diff_preload_torque_nm: float = 15.0: set = _set_diff_preload
@export var diff_slip_threshold_rad_s: float = 1.5: set = _set_diff_slip_threshold

@export_group("Suspension Parameters")
@export_group("Front Suspension")
@export var front_spring_stiffness_n_mm: float = 45.0: set = _set_front_spring_stiffness
@export var front_damping_compression_n_mm: float = 3.5: set = _set_front_damping_comp
@export var front_damping_rebound_n_mm: float = 2.8: set = _set_front_damping_rebound
@export var front_suspension_travel_mm: float = 120.0: set = _set_front_travel
@export var front_anti_roll_bar_stiffness_n_mm: float = 8.5: set = _set_front_arb
@export_group("Rear Suspension")
@export var rear_spring_stiffness_n_mm: float = 55.0: set = _set_rear_spring_stiffness
@export var rear_damping_compression_n_mm: float = 4.2: set = _set_rear_damping_comp
@export var rear_damping_rebound_n_mm: float = 3.2: set = _set_rear_damping_rebound
@export var rear_suspension_travel_mm: float = 115.0: set = _set_rear_travel
@export var rear_anti_roll_bar_stiffness_n_mm: float = 12.0: set = _set_rear_arb

@export_group("Tire Friction Model")
@export var tire_model: TireFrictionModel = TireFrictionModel.SIMPLIFIED
@export_group("Simplified Friction Coefficients")
@export var tire_static_friction: float = 1.65: set = _set_static_friction
@export var tire_dynamic_friction: float = 1.25: set = _set_dynamic_friction
@export var tire_roll_resistance_coefficient: float = 0.012: set = _set_roll_resistance
@export_group("Pacejka Magic Formula Coefficients")
@export var pacejka_b: float = 1.9: set = _set_pacejka_b
@export var pacejka_c: float = 1.9: set = _set_pacejka_c
@export var pacejka_d: float = 1.35: set = _set_pacejka_d
@export var pacejka_e: float = -0.35: set = _set_pacejka_e
@export var pacejka_beta: float = 1.0: set = _set_pacejka_beta

@export_group("Aerodynamics")
@export var drag_coefficient: float = 0.32: set = _set_drag_coefficient
@export var lift_coefficient: float = -0.15: set = _set_lift_coefficient
@export var downforce_coefficient: float = 0.85: set = _set_downforce_coefficient
@export var aero_reference_area_m2: float = 2.1: set = _set_aero_ref_area
@export var aero_center_of_pressure_x: float = 0.0: set = _set_aero_cp_x
@export var aero_center_of_pressure_y: float = 0.0: set = _set_aero_cp_y
@export var aero_center_of_pressure_z: float = 0.55: set = _set_aero_cp_z
@export var wing_front_angle_deg: float = 5.0: set = _set_wing_front_angle
@export var wing_rear_angle_deg: float = 15.0: set = _set_wing_rear_angle

@export_group("Steering System")
@export var steering_max_lock_deg: float = 360.0: set = _set_steering_lock
@export var steering_ratio: float = 14.0: set = _set_steering_ratio
@export var steering_feedback_gain: float = 0.85: set = _set_steering_feedback
@export var steering_deadzone: float = 0.05: set = _set_steering_deadzone
@export var steering_speed_multiplier: float = 1.0: set = _set_steering_speed

@export_group("Brake System")
@export var brake_pressure_max_bar: float = 140.0: set = _set_brake_pressure_max
@export var brake_pressure_min_bar: float = 0.0: set = _set_brake_pressure_min
@export var brake_bias_front_percent: float = 55.0: set = _set_brake_bias
@export var brake_caliper_effective_radius_m: float = 0.14: set = _set_brake_caliper_radius
@export var brake_pad_friction_coefficient: float = 0.42: set = _set_brake_pad_friction
@export var brake_temperature_decay_rate: float = 0.005: set = _set_brake_temp_decay
@export var brake_optimal_temp_k: float = 350.0: set = _set_brake_optimal_temp
@export var brake_overheat_temp_k: float = 650.0: set = _set_brake_overheat_temp

@export_group("Wheel Dynamics")
@export var wheel_rotational_inertia_kg_m2: float = 0.85: set = _set_wheel_inertia
@export var wheel_tire_mass_kg: float = 10.5: set = _set_wheel_tire_mass
@export var wheel_unsprung_mass_kg: float = 12.0: set = _set_wheel_unsprung_mass
@export var wheel_max_lateral_acceleration_g: float = 1.4: set = _set_max_lateral_g
@export var wheel_max_longitudinal_acceleration_g: float = 1.5: set = _set_max_longitudinal_g

@export_group("Collision & Physics")
@export var collision_tolerance: float = 0.001: set = _set_collision_tolerance
@export var contact_detection_iterations: int = 3: set = _set_contact_iterations
@export var velocity_threshold_m_s: float = 0.01: set = _set_velocity_threshold
@export var sleep_threshold_m_s: float = 0.05: set = _set_sleep_threshold

## Enums for configuration types
enum TransmissionType { MANUAL, AUTOMATIC, CVT }
enum DifferentialType { OPEN, LSD, LOCKED }
enum TireFrictionModel { SIMPLIFIED, PACEJKA, COMPOUND }

## Unit conversion constants
const MM_TO_M: float = 0.001
const DEG_TO_RAD: float = PI / 180.0
const RAD_TO_DEG: float = 180.0 / PI
const BAR_TO_PA: float = 100000.0
const MPH_TO_MS: float = 0.44704
const KMH_TO_MS: float = 0.27778
const MS_TO_KMH: float = 3.6
const G_TO_MS2: float = 9.81

## Cache for computed values (avoid recalculating every frame)
var _computed_values: Dictionary = {}

func _init() -> void:
	_compute_cached_values()

func _set_gravity(value: float) -> void:
	if value > 0 and value < 100:
		gravity = value
		_update_cached_values()

func _set_physics_tick_rate(value: int) -> void:
	if value >= 60 and value <= 240:
		physics_tick_rate = value
		_update_cached_values()

func _set_max_substeps(value: int) -> void:
	if value >= 1 and value <= 8:
		max_substeps = value
		_update_cached_values()

func _set_time_scale(value: float) -> void:
	if value > 0 and value <= 5:
		time_scale = value
		_update_cached_values()

func _set_default_vehicle_mass(value: float) -> void:
	if value >= 500 and value <= 5000:
		default_vehicle_mass = value
		_update_cached_values()

func _set_default_wheel_radius(value: float) -> void:
	if value >= 0.2 and value <= 0.6:
		default_wheel_radius = value
		_update_cached_values()

func _set_default_wheel_width(value: float) -> void:
	if value >= 0.15 and value <= 0.4:
		default_wheel_width = value
		_update_cached_values()

func _set_default_com_height(value: float) -> void:
	if value >= 0.2 and value <= 1.0:
		default_center_of_mass_height = value
		_update_cached_values()

func _set_default_com_x(value: float) -> void:
	if value >= -1.0 and value <= 1.0:
		default_center_of_mass_x = value
		_update_cached_values()

func _set_default_track_width(value: float) -> void:
	if value >= 1.0 and value <= 2.5:
		default_track_width = value
		_update_cached_values()

func _set_default_wheelbase(value: float) -> void:
	if value >= 2.0 and value <= 4.0:
		default_wheelbase = value
		_update_cached_values()

func _set_engine_max_rpm(value: float) -> void:
	if value >= 3000 and value <= 15000:
		engine_max_rpm = value
		_update_cached_values()

func _set_engine_idle_rpm(value: float) -> void:
	if value >= 500 and value < engine_max_rpm * 0.2:
		engine_idle_rpm = value
		_update_cached_values()

func _set_engine_peak_torque_rpm(value: float) -> void:
	if value >= engine_idle_rpm + 500 and value < engine_max_rpm:
		engine_peak_torque_rpm = value
		_update_cached_values()

func _set_engine_peak_torque(value: float) -> void:
	if value >= 50 and value <= 2000:
		engine_peak_torque_nm = value
		_update_cached_values()

func _set_engine_max_torque_factor(value: float) -> void:
	if value >= 1.0 and value <= 2.5:
		engine_max_torque_factor = value
		_update_cached_values()

func _set_final_drive_ratio(value: float) -> void:
	if value >= 2.0 and value <= 8.0:
		final_drive_ratio = value
		_update_cached_values()

func _set_gear_ratios(ratios: Array[float]) -> void:
	if ratios.size() >= 5 and ratios.size() <= 8:
		for r in ratios:
			if r >= 0.5 and r <= 5.0:
				gear_ratios = ratios
				_update_cached_values()
				break

func _set_clutch_friction(value: float) -> void:
	if value >= 0.5 and value <= 3.0:
		clutch_friction_coefficient = value
		_update_cached_values()

func _set_flywheel_inertia(value: float) -> void:
	if value >= 0.05 and value <= 1.0:
		flywheel_inertia_kg_m2 = value
		_update_cached_values()

func _set_diff_lock_front(value: float) -> void:
	if value >= 0.0 and value <= 1.0:
		diff_lock_front = value
		_update_cached_values()

func _set_diff_lock_rear(value: float) -> void:
	if value >= 0.0 and value <= 1.0:
		diff_lock_rear = value
		_update_cached_values()

func _set_diff_preload(value: float) -> void:
	if value >= 0.0 and value <= 50.0:
		diff_preload_torque_nm = value
		_update_cached_values()

func _set_diff_slip_threshold(value: float) -> void:
	if value >= 0.1 and value <= 5.0:
		diff_slip_threshold_rad_s = value
		_update_cached_values()

func _set_front_spring_stiffness(value: float) -> void:
	if value >= 10.0 and value <= 150.0:
		front_spring_stiffness_n_mm = value
		_update_cached_values()

func _set_front_damping_comp(value: float) -> void:
	if value >= 1.0 and value <= 20.0:
		front_damping_compression_n_mm = value
		_update_cached_values()

func _set_front_damping_rebound(value: float) -> void:
	if value >= 1.0 and value <= 20.0:
		front_damping_rebound_n_mm = value
		_update_cached_values()

func _set_front_travel(value: float) -> void:
	if value >= 50.0 and value <= 250.0:
		front_suspension_travel_mm = value
		_update_cached_values()

func _set_front_arb(value: float) -> void:
	if value >= 1.0 and value <= 30.0:
		front_anti_roll_bar_stiffness_n_mm = value
		_update_cached_values()

func _set_rear_spring_stiffness(value: float) -> void:
	if value >= 10.0 and value <= 150.0:
		rear_spring_stiffness_n_mm = value
		_update_cached_values()

func _set_rear_damping_comp(value: float) -> void:
	if value >= 1.0 and value <= 20.0:
		rear_damping_compression_n_mm = value
		_update_cached_values()

func _set_rear_damping_rebound(value: float) -> void:
	if value >= 1.0 and value <= 20.0:
		rear_damping_rebound_n_mm = value
		_update_cached_values()

func _set_rear_travel(value: float) -> void:
	if value >= 50.0 and value <= 250.0:
		rear_suspension_travel_mm = value
		_update_cached_values()

func _set_rear_arb(value: float) -> void:
	if value >= 1.0 and value <= 30.0:
		rear_anti_roll_bar_stiffness_n_mm = value
		_update_cached_values()

func _set_static_friction(value: float) -> void:
	if value >= 0.5 and value <= 3.0:
		tire_static_friction = value
		_update_cached_values()

func _set_dynamic_friction(value: float) -> void:
	if value >= 0.3 and value <= 2.0:
		tire_dynamic_friction = value
		_update_cached_values()

func _set_roll_resistance(value: float) -> void:
	if value >= 0.005 and value <= 0.05:
		tire_roll_resistance_coefficient = value
		_update_cached_values()

func _set_pacejka_b(value: float) -> void:
	if value >= 1.0 and value <= 3.0:
		pacejka_b = value
		_update_cached_values()

func _set_pacejka_c(value: float) -> void:
	if value >= 1.0 and value <= 3.0:
		pacejka_c = value
		_update_cached_values()

func _set_pacejka_d(value: float) -> void:
	if value >= 0.5 and value <= 3.0:
		pacejka_d = value
		_update_cached_values()

func _set_pacejka_e(value: float) -> void:
	if value >= -2.0 and value <= 0.0:
		pacejka_e = value
		_update_cached_values()

func _set_pacejka_beta(value: float) -> void:
	if value >= 0.5 and value <= 2.0:
		pacejka_beta = value
		_update_cached_values()

func _set_drag_coefficient(value: float) -> void:
	if value >= 0.1 and value <= 1.0:
		drag_coefficient = value
		_update_cached_values()

func _set_lift_coefficient(value: float) -> void:
	if value >= -1.0 and value <= 0.5:
		lift_coefficient = value
		_update_cached_values()

func _set_downforce_coefficient(value: float) -> void:
	if value >= 0.0 and value <= 3.0:
		downforce_coefficient = value
		_update_cached_values()

func _set_aero_ref_area(value: float) -> void:
	if value >= 1.0 and value <= 5.0:
		aero_reference_area_m2 = value
		_update_cached_values()

func _set_aero_cp_x(value: float) -> void:
	if value >= -2.0 and value <= 2.0:
		aero_center_of_pressure_x = value
		_update_cached_values()

func _set_aero_cp_y(value: float) -> void:
	if value >= -2.0 and value <= 2.0:
		aero_center_of_pressure_y = value
		_update_cached_values()

func _set_aero_cp_z(value: float) -> void:
	if value >= 0.1 and value <= 2.0:
		aero_center_of_pressure_z = value
		_update_cached_values()

func _set_wing_front_angle(value: float) -> void:
	if value >= -10.0 and value <= 30.0:
		wing_front_angle_deg = value
		_update_cached_values()

func _set_wing_rear_angle(value: float) -> void:
	if value >= -10.0 and value <= 45.0:
		wing_rear_angle_deg = value
		_update_cached_values()

func _set_steering_lock(value: float) -> void:
	if value >= 180.0 and value <= 720.0:
		steering_max_lock_deg = value
		_update_cached_values()

func _set_steering_ratio(value: float) -> void:
	if value >= 10.0 and value <= 20.0:
		steering_ratio = value
		_update_cached_values()

func _set_steering_feedback(value: float) -> void:
	if value >= 0.0 and value <= 2.0:
		steering_feedback_gain = value
		_update_cached_values()

func _set_steering_deadzone(value: float) -> void:
	if value >= 0.0 and value <= 0.2:
		steering_deadzone = value
		_update_cached_values()

func _set_steering_speed(value: float) -> void:
	if value >= 0.5 and value <= 3.0:
		steering_speed_multiplier = value
		_update_cached_values()

func _set_brake_pressure_max(value: float) -> void:
	if value >= 50.0 and value <= 250.0:
		brake_pressure_max_bar = value
		_update_cached_values()

func _set_brake_pressure_min(value: float) -> void:
	if value >= 0.0 and value <= brake_pressure_max_bar:
		brake_pressure_min_bar = value
		_update_cached_values()

func _set_brake_bias(value: float) -> void:
	if value >= 40.0 and value <= 70.0:
		brake_bias_front_percent = value
		_update_cached_values()

func _set_brake_caliper_radius(value: float) -> void:
	if value >= 0.08 and value <= 0.25:
		brake_caliper_effective_radius_m = value
		_update_cached_values()

func _set_brake_pad_friction(value: float) -> void:
	if value >= 0.2 and value <= 0.7:
		brake_pad_friction_coefficient = value
		_update_cached_values()

func _set_brake_temp_decay(value: float) -> void:
	if value >= 0.001 and value <= 0.05:
		brake_temperature_decay_rate = value
		_update_cached_values()

func _set_brake_optimal_temp(value: float) -> void:
	if value >= 250.0 and value <= 500.0:
		brake_optimal_temp_k = value
		_update_cached_values()

func _set_brake_overheat_temp(value: float) -> void:
	if value >= brake_optimal_temp_k and value <= 1000.0:
		brake_overheat_temp_k = value
		_update_cached_values()

func _set_wheel_inertia(value: float) -> void:
	if value >= 0.2 and value <= 3.0:
		wheel_rotational_inertia_kg_m2 = value
		_update_cached_values()

func _set_wheel_tire_mass(value: float) -> void:
	if value >= 5.0 and value <= 30.0:
		wheel_tire_mass_kg = value
		_update_cached_values()

func _set_wheel_unsprung_mass(value: float) -> void:
	if value >= 8.0 and value <= 40.0:
		wheel_unsprung_mass_kg = value
		_update_cached_values()

func _set_max_lateral_g(value: float) -> void:
	if value >= 0.5 and value <= 3.0:
		wheel_max_lateral_acceleration_g = value
		_update_cached_values()

func _set_max_longitudinal_g(value: float) -> void:
	if value >= 0.5 and value <= 4.0:
		wheel_max_longitudinal_acceleration_g = value
		_update_cached_values()

func _set_collision_tolerance(value: float) -> void:
	if value >= 0.0001 and value <= 0.01:
		collision_tolerance = value
		_update_cached_values()

func _set_contact_iterations(value: int) -> void:
	if value >= 1 and value <= 10:
	(contact_detection_iterations = value
		_update_cached_values()

func _set_velocity_threshold(value: float) -> void:
	if value >= 0.001 and value <= 1.0:
		velocity_threshold_m_s = value
		_update_cached_values()

func _set_sleep_threshold(value: float) -> void:
	if value >= 0.01 and value <= 2.0:
		sleep_threshold_m_s = value
		_update_cached_values()

## Get current timestep in seconds
func get_timestep() -> float:
	return 1.0 / float(physics_tick_rate)

## Get physics step duration for fixed timestep updates
func get_step_duration() -> float:
	return get_timestep() * time_scale

## Calculate torque at current RPM given gear ratio
func calculate_engine_torque(rpm: float, gear_index: int) -> float:
	var normalized_rpm: float = clamp((rpm - engine_idle_rpm) / (engine_max_rpm - engine_idle_rpm), 0.0, 1.0)
	
	# Quadratic torque curve approximation
	var torque_curve: float = engine_peak_torque_nm * pow(normalized_rpm, 2.0)
	
	# Apply peak torque location adjustment
	var rpm_to_peak: float = (engine_peak_torque_rpm - engine_idle_rpm) / (engine_max_rpm - engine_idle_rpm)
	if rpm_to_peak > 0.0:
		torque_curve *= smoothstep(0.0, rpm_to_peak, normalized_rpm) * smoothstep(rpm_to_peak, 1.0, normalized_rpm)
	
	return min(torque_curve * engine_max_torque_factor, engine_peak_torque_nm * engine_max_torque_factor)

## Get effective gear ratio including final drive
func get_effective_gear_ratio(gear_index: int) -> float:
	if gear_index < 0 or gear_index >= gear_ratios.size():
		return 0.0
	return gear_ratios[gear_index] * final_drive_ratio

## Calculate wheel angular velocity from vehicle speed
func get_wheel_angular_velocity(vehicle_speed_ms: float) -> float:
	if default_wheel_radius <= 0:
		return 0.0
	return vehicle_speed_ms / default_wheel_radius

## Convert mm/s to m/s
func mm_to_ms(speed_mm_s: float) -> float:
	return speed_mm_s * MM_TO_M

## Convert m/s to mm/s
func ms_to_mm(speed_ms: float) -> float:
	return speed_ms / MM_TO_M

## Convert degrees to radians
func deg_to_rad(degrees: float) -> float:
	return degrees * DEG_TO_RAD

## Convert radians to degrees
func rad_to_deg(radians: float) -> float:
	return radians * RAD_TO_DEG

## Convert bar to pascal
func bar_to_pa(bar: float) -> float:
	return bar * BAR_TO_PA

## Convert km/h to m/s
func kmh_to_ms(kmh: float) -> float:
	return kmh * KMH_TO_MS

## Convert m/s to km/h
func ms_to_kmh(ms: float) -> float:
	return ms * MS_TO_KMH

## Convert mph to m/s
func mph_to_ms(mph: float) -> float:
	return mph * MPH_TO_MS

## Convert g-force to m/s²
func g_to_ms2(g_force: float) -> float:
	return g_force * G_TO_MS2

## Calculate downforce at given speed
func calculate_downforce(vehicle_speed_ms: float) -> float:
	var dynamic_pressure: float = 0.5 * 1.225 * pow(vehicle_speed_ms, 2.0)
	return dynamic_pressure * downforce_coefficient * aero_reference_area_m2

## Calculate drag force at given speed
func calculate_drag_force(vehicle_speed_ms: float) -> float:
	var dynamic_pressure: float = 0.5 * 1.225 * pow(vehicle_speed_ms, 2.0)
	return dynamic_pressure * drag_coefficient * aero_reference_area_m2

## Calculate total aerodynamic force (downforce + lift)
func calculate_total_aero_force(vehicle_speed_ms: float) -> float:
	var downforce: float = calculate_downforce(vehicle_speed_ms)
	var lift_force: float = 0.5 * 1.225 * pow(vehicle_speed_ms, 2.0) * lift_coefficient * aero_reference_area_m2
	return downforce - lift_force

## Calculate tire friction force using simplified model
func calculate_tire_friction(lateral_load_newton: float, slip_angle_rad: float) -> float:
	var normal_force: float = abs(lateral_load_newton)
	var friction_coefficient: float = tire_static_friction
	if lateral_load_newton != 0:
		friction_coefficient = tire_dynamic_friction
	
	return normal_force * friction_coefficient

## Calculate tire friction using Pacejka magic formula
func calculate_pacejka_friction(slip_ratio: float, vertical_load: float, camber_angle: float) -> float:
	var alpha: float = abs(slip_ratio)
	var B: float = pacejka_b
	var C: float = pacejka_c
	var D: float = pacejka_d * vertical_load / 1000.0
	var E: float = pacejka_e
	
	var angle: float = (B * C * atan(B * alpha * (1.0 - E) + E * atan(B * alpha))))
	
	return D * sin(C * angle)

## Get maximum cornering force for given conditions
func get_max_cornering_force(vertical_load: float) -> float:
	return vertical_load * wheel_max_lateral_acceleration_g * G_TO_MS2

## Get maximum braking/acceleration force
func get_max_traction_force(vertical_load: float) -> float:
	return vertical_load * wheel_max_longitudinal_acceleration_g * G_TO_MS2

## Calculate suspension compression based on wheel position
func calculate_suspension_compression(wheel_position: float, rest_length: float) -> float:
	return rest_length - wheel_position

## Validate vehicle configuration matches physics settings
func validate_vehicle_config(mass: float, wheelbase: float, track_width: float) -> bool:
	if mass < 500 or mass > 5000:
		return false
	if wheelbase < 2.0 or wheelbase > 4.0:
		return false
	if track_width < 1.0 or track_width > 2.5:
		return false
	return true

## Compute cached values once on initialization
func _compute_cached_values() -> void:
	_computed_values["timestep"] = get_timestep()
	_computed_values["step_duration"] = get_step_duration()
	_computed_values["front_travel_m"] = front_suspension_travel_mm * MM_TO_M
	_computed_values["rear_travel_m"] = rear_suspension_travel_mm * MM_TO_M
	_computed_values["front_spring_n_m"] = front_spring_stiffness_n_mm * 1000.0
	_computed_values["rear_spring_n_m"] = rear_spring_stiffness_n_mm * 1000.0
	_computed_values["front_damping_comp_n_s_m"] = front_damping_compression_n_mm * 1000.0
	_computed_values["front_damping_rebound_n_s_m"] = front_damping_rebound_n_mm * 1000.0
	_computed_values["rear_damping_comp_n_s_m"] = rear_damping_compression_n_mm * 1000.0
	_computed_values["rear_damping_rebound_n_s_m"] = rear_damping_rebound_n_mm * 1000.0

## Update cache when values change
func _update_cached_values() -> void:
	_compute_cached_values()

## Export settings to dictionary for serialization
func export_to_dict() -> Dictionary:
	return {
		"gravity": gravity,
		"physics_tick_rate": physics_tick_rate,
		"max_substeps": max_substeps,
		"time_scale": time_scale,
		"default_vehicle_mass": default_vehicle_mass,
		"default_wheel_radius": default_wheel_radius,
		"default_wheel_width": default_wheel_width,
		"default_center_of_mass_height": default_center_of_mass_height,
		"default_center_of_mass_x": default_center_of_mass_x,
		"default_track_width": default_track_width,
		"default_wheelbase": default_wheelbase,
		"engine_max_rpm": engine_max_rpm,
		"engine_idle_rpm": engine_idle_rpm,
		"engine_peak_torque_rpm": engine_peak_torque_rpm,
		"engine_peak_torque_nm": engine_peak_torque_nm,
		"engine_max_torque_factor": engine_max_torque_factor,
		"transmission_type": transmission_type,
		"final_drive_ratio": final_drive_ratio,
		"gear_ratios": gear_ratios,
		"clutch_friction_coefficient": clutch_friction_coefficient,
		"flywheel_inertia_kg_m2": flywheel_inertia_kg_m2,
		"diff_type": diff_type,
		"diff_lock_front": diff_lock_front,
		"diff_lock_rear": diff_lock_rear,
		"diff_preload_torque_nm": diff_preload_torque_nm,
		"diff_slip_threshold_rad_s": diff_slip_threshold_rad_s,
		"front_spring_stiffness_n_mm": front_spring_stiffness_n_mm,
		"front_damping_compression_n_mm": front_damping_compression_n_mm,
		"front_damping_rebound_n_mm": front_damping_rebound_n_mm,
		"front_suspension_travel_mm": front_suspension_travel_mm,
		"front_anti_roll_bar_stiffness_n_mm": front_anti_roll_bar_stiffness_n_mm,
		"rear_spring_stiffness_n_mm": rear_spring_stiffness_n_mm,
		"rear_damping_compression_n_mm": rear_damping_compression_n_mm,
		"rear_damping_rebound_n_mm": rear_damping_rebound_n_mm,
		"rear_suspension_travel_mm": rear_suspension_travel_mm,
		"rear_anti_roll_bar_stiffness_n_mm": rear_anti_roll_bar_stiffness_n_mm,
		"tire_model": tire_model,
		"tire_static_friction": tire_static_friction,
		"tire_dynamic_friction": tire_dynamic_friction,
		"tire_roll_resistance_coefficient": tire_roll_resistance_coefficient,
		"pacejka_b": pacejka_b,
		"pacejka_c": pacejka_c,
		"pacejka_d": pacejka_d,
		"pacejka_e": pacejka_e,
		"pacejka_beta": pacejka_beta,
		"drag_coefficient": drag_coefficient,
		"lift_coefficient": lift_coefficient,
		"downforce_coefficient": downforce_coefficient,
		"aero_reference_area_m2": aero_reference_area_m2,
		"aero_center_of_pressure_x": aero_center_of_pressure_x,
		"aero_center_of_pressure_y": aero_center_of_pressure_y,
		"aero_center_of_pressure_z": aero_center_of_pressure_z,
		"wing_front_angle_deg": wing_front_angle_deg,
		"wing_rear_angle_deg": wing_rear_angle_deg,
		"steering_max_lock_deg": steering_max_lock_deg,
		"steering_ratio": steering_ratio,
		"steering_feedback_gain": steering_feedback_gain,
		"steering_deadzone": steering_deadzone,
		"steering_speed_multiplier": steering_speed_multiplier,
		"brake_pressure_max_bar": brake_pressure_max_bar,
		"brake_pressure_min_bar": brake_pressure_min_bar,
		"brake_bias_front_percent": brake_bias_front_percent,
		"brake_caliper_effective_radius_m": brake_caliper_effective_radius_m,
		"brake_pad_friction_coefficient": brake_pad_friction_coefficient,
		"brake_temperature_decay_rate": brake_temperature_decay_rate,
		"brake_optimal_temp_k": brake_optimal_temp_k,
		"brake_overheat_temp_k": brake_overheat_temp_k,
		"wheel_rotational_inertia_kg_m2": wheel_rotational_inertia_kg_m2,
		"wheel_tire_mass_kg": wheel_tire_mass_kg,
		"wheel_unsprung_mass_kg": wheel_unsprung_mass_kg,
		"wheel_max_lateral_acceleration_g": wheel_max_lateral_acceleration_g,
		"wheel_max_longitudinal_acceleration_g": wheel_max_longitudinal_acceleration_g,
		"collision_tolerance": collision_tolerance,
		"contact_detection_iterations": contact_detection_iterations,
		"velocity_threshold_m_s": velocity_threshold_m_s,
		"sleep_threshold_m_s": sleep_threshold_m_s
	}

## Import settings from dictionary
func import_from_dict(data: Dictionary) -> void:
	if not data.has("gravity"): return
	gravity = data["gravity"]
	physics_tick_rate = data["physics_tick_rate"]
	max_substeps = data["max_substeps"]
	time_scale = data["time_scale"]
	default_vehicle_mass = data["default_vehicle_mass"]
	default_wheel_radius = data["default_wheel_radius"]
	default_wheel_width = data["default_wheel_width"]
	default_center_of_mass_height = data["default_center_of_mass_height"]
	default_center_of_mass_x = data["default_center_of_mass_x"]
	default_track_width = data["default_track_width"]
	default_wheelbase = data["default_wheelbase"]
	engine_max_rpm = data["engine_max_rpm"]
	engine_idle_rpm = data["engine_idle_rpm"]
	engine_peak_torque_rpm = data["engine_peak_torque_rpm"]
	engine_peak_torque_nm = data["engine_peak_torque_nm"]
	engine_max_torque_factor = data["engine_max_torque_factor"]
	transmission_type = data["transmission_type"]
	final_drive_ratio = data["final_drive_ratio"]
	gear_ratios = data["gear_ratios"]
	clutch_friction_coefficient = data["clutch_friction_coefficient"]
	flywheel_inertia_kg_m2 = data["flywheel_inertia_kg_m2"]
	diff_type = data["diff_type"]
	diff_lock_front = data["diff_lock_front"]
	diff_lock_rear = data["diff_lock_rear"]
	diff_preload_torque_nm = data["diff_preload_torque_nm"]
	diff_slip_threshold_rad_s = data["diff_slip_threshold_rad_s"]
	front_spring_stiffness_n_mm = data["front_spring_stiffness_n_mm"]
	front_damping_compression_n_mm = data["front_damping_compression_n_mm"]
	front_damping_rebound_n_mm = data["front_damping_rebound_n_mm"]
	front_suspension_travel_mm = data["front_suspension_travel_mm"]
	front_anti_roll_bar_stiffness_n_mm = data["front_anti_roll_bar_stiffness_n_mm"]
	rear_spring_stiffness_n_mm = data["rear_spring_stiffness_n_mm"]
	rear_damping_compression_n_mm = data["rear_damping_compression_n_mm"]
	rear_damping_rebound_n_mm = data["rear_damping_rebound_n_mm"]
	rear_suspension_travel_mm = data["rear_suspension_travel_mm"]
	rear_anti_roll_bar_stiffness_n_mm = data["rear_anti_roll_bar_stiffness_n_mm"]
	tire_model = data["tire_model"]
	tire_static_friction = data["tire_static_friction"]
	tire_dynamic_friction = data["tire_dynamic_friction"]
	tire_roll_resistance_coefficient = data["tire_roll_resistance_coefficient"]
	pacejka_b = data["pacejka_b"]
	pacejka_c = data["pacejka_c"]
	pacejka_d = data["pacejka_d"]
	pacejka_e = data["pacejka_e"]
	pacejka_beta = data["pacejka_beta"]
	drag_coefficient = data["drag_coefficient"]
	lift_coefficient = data["lift_coefficient"]
	downforce_coefficient = data["downforce_coefficient"]
	aero_reference_area_m2 = data["aero_reference_area_m2"]
	aero_center_of_pressure_x = data["aero_center_of_pressure_x"]
	aero_center_of_pressure_y = data["aero_center_of_pressure_y"]
	aero_center_of_pressure_z = data["aero_center_of_pressure_z"]
	wing_front_angle_deg = data["wing_front_angle_deg"]
	wing_rear_angle_deg = data["wing_rear_angle_deg"]
	steering_max_lock_deg = data["steering_max_lock_deg"]
	steering_ratio = data["steering_ratio"]
	steering_feedback_gain = data["steering_feedback_gain"]
	steering_deadzone = data["steering_deadzone"]
	steering_speed_multiplier = data["steering_speed_multiplier"]
	brake_pressure_max_bar = data["brake_pressure_max_bar"]
	brake_pressure_min_bar = data["brake_pressure_min_bar"]
	brake_bias_front_percent = data["brake_bias_front_percent"]
	brake_caliper_effective_radius_m = data["brake_caliper_effective_radius_m"]
	brake_pad_friction_coefficient = data["brake_pad_friction_coefficient"]
	brake_temperature_decay_rate = data["brake_temperature_decay_rate"]
	brake_optimal_temp_k = data["brake_optimal_temp_k"]
	brake_overheat_temp_k = data["brake_overheat_temp_k"]
	wheel_rotational_inertia_kg_m2 = data["wheel_rotational_inertia_kg_m2"]
	wheel_tire_mass_kg = data["wheel_tire_mass_kg"]
	wheel_unsprung_mass_kg = data["wheel_unsprung_mass_kg"]
	wheel_max_lateral_acceleration_g = data["wheel_max_lateral_acceleration_g"]
	wheel_max_longitudinal_acceleration_g = data["wheel_max_longitudinal_acceleration_g"]
	collision_tolerance = data["collision_tolerance"]
.contact_detection_iterations = data["contact_detection_iterations"]
	velocity_threshold_m_s = data["velocity_threshold_m_s"]
	sleep_threshold_m_s = data["sleep_threshold_m_s"]
	_update_cached_values()

## Preset configurations for different car types
static func get_presets() -> Dictionary:
	return {
		"sports_car": {
			"default_vehicle_mass": 1350.0,
			"engine_max_rpm": 9000.0,
			"engine_peak_torque_nm": 380.0,
			"drag_coefficient": 0.28,
			"downforce_coefficient": 1.2,
			"front_spring_stiffness_n_mm": 50.0,
			"rear_spring_stiffness_n_mm": 55.0,
			"tire_static_friction": 1.75,
			"steering_max_lock_deg": 320.0
		},
		"rally_car": {
			"default_vehicle_mass": 1450.0,
			"engine_max_rpm": 7500.0,
			"engine_peak_torque_nm": 420.0,
			"drag_coefficient": 0.35,
			"downforce_coefficient": 0.4,
			"front_spring_stiffness_n_mm": 35.0,
			"rear_spring_stiffness_n_mm": 40.0,
			"front_suspension_travel_mm": 180.0,
			"rear_suspension_travel_mm": 180.0,
			"tire_static_friction": 1.55
		},
		"f1_car": {
			"default_vehicle_mass": 798.0,
			"engine_max_rpm": 15000.0,
			"engine_peak_torque_nm": 350.0,
			"drag_coefficient": 0.7,
			"downforce_coefficient": 4.5,
			"front_spring_stiffness_n_mm": 80.0,
			"rear_spring_stiffness_n_mm": 90.0,
			"front_suspension_travel_mm": 70.0,
			"rear_suspension_travel_mm": 70.0,
			"tire_static_friction": 2.2,
			"steering_max_lock_deg": 240.0
		},
		"muscle_car": {
			"default_vehicle_mass": 1800.0,
			"engine_max_rpm": 6500.0,
			"engine_peak_torque_nm": 600.0,
			"drag_coefficient": 0.38,
			"downforce_coefficient": 0.3,
			"front_spring_stiffness_n_mm": 40.0,
			"rear_spring_stiffness_n_mm": 45.0,
			"tire_static_friction": 1.45,
			"different_lock_rear": 0.7
		}
	}

## Apply preset configuration
func apply_preset(preset_name: String) -> void:
	var presets: Dictionary = get_presets()
	if not presets.has(preset_name):
		push_warning("PhysicsSettings: Preset '%s' not found" % preset_name)
		return
	
	var preset: Dictionary = presets[preset_name]
	import_from_dict(preset)

## Load preset as a new instance
static func load_preset(preset_name: String) -> PhysicsSettings:
	var settings: PhysicsSettings = PhysicsSettings.new()
	settings.apply_preset(preset_name)
	return settings