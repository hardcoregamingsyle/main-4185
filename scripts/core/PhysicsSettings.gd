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

@export_group("Suspension Parameters - Front")
@export var front_spring_stiffness_n_mm: float = 45.0: set = _set_front_spring_stiffness
@export var front_damping_compression_n_mm: float = 3.5: set = _set_front_damping_comp
@export var front_damping_rebound_n_mm: float = 2.8: set = _set_front_damping_rebound
@export var front_suspension_travel_mm: float = 120.0: set = _set_front_travel
@export var front_anti_roll_bar_stiffness_n_mm: float = 8.5: set = _set_front_arb
@export var front_camber_gain_per_mm: float = 0.02: set = _set_front_camber_gain
@export var front_toe_gain_per_mm: float = 0.01: set = _set_front_toe_gain

@export_group("Suspension Parameters - Rear")
@export var rear_spring_stiffness_n_mm: float = 50.0: set = _set_rear_spring_stiffness
@export var rear_damping_compression_n_mm: float = 3.8: set = _set_rear_damping_comp
@export var rear_damping_rebound_n_mm: float = 3.0: set = _set_rear_damping_rebound
@export var rear_suspension_travel_mm: float = 115.0: set = _set_rear_travel
@export var rear_anti_roll_bar_stiffness_n_mm: float = 10.0: set = _set_rear_arb
@export var rear_camber_gain_per_mm: float = 0.018: set = _set_rear_camber_gain
@export var rear_toe_gain_per_mm: float = 0.008: set = _set_rear_toe_gain

@export_group("Tire Friction Properties")
@export var tire_peak_friction_coefficient: float = 1.25: set = _set_tire_peak_friction
@export var tire_sliding_friction_coefficient: float = 0.85: set = _set_tire_sliding_friction
@export var tire_pacejka_b_coefficient: float = 8.5: set = _set_tire_b_coefficient
@export var tire_pacejka_c_coefficient: float = 1.8: set = _set_tire_c_coefficient
@export var tire_pacejka_d_coefficient: float = 0.35: set = _set_tire_d_coefficient
@export var tire_pacejka_e_coefficient: float = -0.3: set = _set_tire_e_coefficient
@export var tire_vertical_stiffness_n_mm: float = 250.0: set = _set_tire_vertical_stiffness
@export var tire_longitudinal_stiffness_n_mm: float = 180.0: set = _set_tire_longitudinal_stiffness
@export var tire_lateral_stiffness_n_mm: float = 220.0: set = _set_tire_lateral_stiffness
@export var tire_pressure_bar: float = 2.2: set = _set_tire_pressure
@export var tire_temperature_optimal_min_c: float = 70.0: set = _set_tire_temp_opt_min
@export var tire_temperature_optimal_max_c: float = 100.0: set = _set_tire_temp_opt_max
@export var tire_temperature_effective_range_c: float = 30.0: set = _set_tire_temp_effective_range

@export_group("Aerodynamics")
@export var aero_drag_coefficient: float = 0.32: set = _set_aero_drag_coeff
@export var aero_frontal_area_m2: float = 2.1: set = _set_aero_frontal_area
@export var aero_downforce_coefficient: float = 0.85: set = _set_aero_downforce_coeff
@export var aero_front_balance: float = 0.42: set = _set_aero_front_balance
@export var aero_lift_coefficient: float = -0.05: set = _set_aero_lift_coeff
@export var aero_side_force_coefficient: float = 0.15: set = _set_aero_side_force
@export var aero_ground_clearance_mm: float = 100.0: set = _set_aero_ground_clearance
@export var aero_stall_speed_ms: float = 85.0: set = _set_aero_stall_speed

@export_group("Steering System")
@export var steering_ratio: float = 14.5: set = _set_steering_ratio
@export var steering_lock_angle_deg: float = 450.0: set = _set_steering_lock
@export var steering_deadband_deg: float = 2.5: set = _set_steering_deadband
@export var steering_return_rate: float = 0.85: set = _set_steering_return
@export var steering_nonlinearity_curve: float = 0.3: set = _set_steering_nonlinear
@export var steering_power_assist_ratio: float = 0.6: set = _set_steering_assist

@export_group("Brake System")
@export var brake_force_distribution_front: float = 0.60: set = _set_brake_dist_front
@export var brake_force_distribution_rear: float = 0.40: set = _set_brake_dist_rear
@export var brake_pad_friction_coefficient: float = 0.45: set = _set_brake_pad_friction
@export var brake_rotor_radius_mm: float = 145.0: set = _set_brake_rotor_radius
@export var brake_caliper_piston_count: int = 4: set = _set_brake_piston_count
@export var brake_caliper_piston_area_mm2: float = 450.0: set = _set_brake_piston_area
@export var brake_max_clamping_force_n: float = 15000.0: set = _set_brake_max_clamp
@export var brake_hysteresis_factor: float = 0.15: set = _set_brake_hysteresis
@export var brake_bleed_threshold_percent: float = 5.0: set = _set_brake_bleed_thresh

@export_group("Chassis & Body")
@export var chassis_stiffness_front_n_mm: float = 25000.0: set = _set_chassis_stiffness_front
@export var chassis_stiffness_rear_n_mm: float = 30000.0: set = _set_chassis_stiffness_rear
@export var roll_center_height_front_mm: float = 120.0: set = _set_roll_center_front
@export var roll_center_height_rear_mm: float = 140.0: set = _set_roll_center_rear
@export var cg_offset_x_mm: float = 0.0: set = _set_cg_offset_x
@export var cg_offset_y_mm: float = 0.0: set = _set_cg_offset_y
@export var cg_offset_z_mm: float = 0.0: set = _set_cg_offset_z
@export var body_swing_period_s: float = 0.8: set = _set_body_swing_period
@export var damping_ratio_body_mode: float = 0.25: set = _set_damping_ratio_body

@export_group("Traction Control")
@export var tc_enabled: bool = true: set = _set_tc_enabled
@export var tc_max_slip_ratio: float = 0.18: set = _set_tc_max_slip
@export var tc_integrator_gain: float = 0.5: set = _set_tc_integrator
@export var tc_proportional_gain: float = 2.5: set = _set_tc_proportional
@export var tc_reaction_delay_s: float = 0.05: set = _set_tc_delay
@export var tc_recovery_rate: float = 0.95: set = _set_tc_recovery

@export_group("ABS System")
@export var abs_enabled: bool = true: set = _set_abs_enabled
@export var abs_slip_target: float = 0.15: set = _set_abs_slip_target
@export var abs_modulation_frequency_hz: float = 15.0: set = _set_abs_freq
@export var abs_pressure_drop_rate_bar_s: float = 120.0: set = _set_abs_drop_rate
@export var abs_pressure_build_rate_bar_s: float = 150.0: set = _set_abs_build_rate
@export var abs_recovery_threshold: float = 0.08: set = _set_abs_recovery

@export_group("Gameplay Tuning")
@export var ai_skill_modifier: float = 1.0: set = _set_ai_skill
@export var player_skill_modifier: float = 1.0: set = _set_player_skill
@export var damage_enabled: bool = true: set = _set_damage_enabled
@export var collision_sound_volume: float = 0.7: set = _set_collision_vol
@export var skid_mark_enabled: bool = true: set = _set_skidmarks
@export var particle_quality: int = 2: set = _set_particle_quality

enum TransmissionType {
	MANUAL,
	SEMI_AUTOMATIC,
	AUTOMATIC,
	CVT
}

enum DifferentialType {
	OPEN,
	LOCKING,
	LSD,
	WHEEL_DIFF
}

enum BrakingMode {
	NORMAL,
	ANTI_LOCK,
	ELECTRONIC_BRAKE_FORCE_DISTRIBUTION
}

enum TireCompound {
	SOFT,
	MEDIUM,
	HARD,
	INTER,
	WET
}

# Helper method: Calculate engine torque at given RPM using Gaussian curve
func get_engine_torque(rpm: float) -> float:
	var peak_rpm: float = engine_peak_torque_rpm
	var max_torque: float = engine_peak_torque_nm * engine_max_torque_factor
	var spread: float = 2500.0
	
	var torque: float = max_torque * exp(-pow((rpm - peak_rpm) / spread, 2))
	
	if rpm < engine_idle_rpm:
		torque *= 0.3
	
	return clamp(torque, 0.0, max_torque)

# Helper method: Calculate power at given RPM
func get_engine_power(rpm: float) -> float:
	return rpm * get_engine_torque(rpm) * 0.00010471975511966

# Helper method: Get gear ratio for specific gear (0-indexed)
func get_gear_ratio(gear: int) -> float:
	if gear >= gear_ratios.size() or gear < 0:
		return 0.0
	return gear_ratios[gear]

# Helper method: Convert degrees to radians
func deg_to_rad(degrees: float) -> float:
	return degrees * PI / 180.0

# Helper method: Convert radians to degrees
func rad_to_deg(radians: float) -> float:
	return radians * 180.0 / PI

# Helper method: Convert mm to meters
func mm_to_m(mm: float) -> float:
	return mm / 1000.0

# Helper method: Convert meters to mm
func m_to_mm(meters: float) -> float:
	return meters * 1000.0

# Helper method: Convert bar to Pascal
func bar_to_pa(bar: float) -> float:
	return bar * 100000.0

# Helper method: Convert km/h to m/s
func kmh_to_ms(kmh: float) -> float:
	return kmh / 3.6

# Helper method: Convert m/s to km/h
func ms_to_kmh(ms: float) -> float:
	return ms * 3.6

# Helper method: Calculate Pacejka 'Magic Formula' for lateral force
func calculate_pacejka_lateral(slip_angle_rad: float, vertical_load_n: float) -> float:
	var alpha: float = abs(slip_angle_rad)
	var c: float = tire_pacejka_c_coefficient
	var b: float = tire_pacejka_b_coefficient
	var d: float = tire_peak_friction_coefficient * vertical_load_n / 1000.0
	var e: float = tire_pacejka_e_coefficient
	
	var cbde: float = c * b * d
	var angle_shift: float = atan(e * ((PI / 2) - abs(atan(b * alpha))))
	
	var force: float = d * sin(cbde * atan(b * alpha * (1.0 - e) + e * angle_shift))
	
	return sign(slip_angle_rad) * force

# Helper method: Calculate Pacejka 'Magic Formula' for longitudinal force
func calculate_pacejka_longitudinal(slips: float, vertical_load_n: float) -> float:
	var s: float = abs(slips)
	var c: float = tire_pacejka_c_coefficient
	var b: float = tire_pacejka_b_coefficient
	var d: float = tire_peak_friction_coefficient * vertical_load_n / 1000.0
	var e: float = tire_pacejka_e_coefficient
	
	var cbde: float = c * b * d
	var angle_shift: float = atan(e * ((PI / 2) - abs(atan(b * s))))
	
	var force: float = d * sin(cbde * atan(b * s * (1.0 - e) + e * angle_shift))
	
	return sign(slips) * force

# Helper method: Calculate aerodynamic downforce
func calculate_downforce(speed_ms: float) -> float:
	var dynamic_pressure: float = 0.5 * 1.225 * speed_ms * speed_ms
	var downforce: float = dynamic_pressure * aero_frontal_area_m2 * aero_downforce_coefficient
	return downforce

# Helper method: Calculate aerodynamic drag force
func calculate_drag_force(speed_ms: float) -> float:
	var dynamic_pressure: float = 0.5 * 1.225 * speed_ms * speed_ms
	var drag: float = dynamic_pressure * aero_frontal_area_m2 * aero_drag_coefficient
	return drag

# Helper method: Validate and clamp all physics values after editor changes
func _validate_all() -> void:
	gravity = max(gravity, 0.1)
	time_scale = clamp(time_scale, 0.0, 2.0)
	default_vehicle_mass = max(default_vehicle_mass, 500.0)
	engine_max_rpm = max(engine_max_rpm, 2000.0)
	engine_idle_rpm = min(engine_idle_rpm, engine_max_rpm * 0.2)
	
	for i in gear_ratios.size():
		gear_ratios[i] = max(gear_raties[i], 0.1)
	
	diff_lock_front = clamp(diff_lock_front, 0.0, 1.0)
	diff_lock_rear = clamp(diff_lock_rear, 0.0, 1.0)
	
	front_spring_stiffness_n_mm = max(front_spring_stiffness_n_mm, 1.0)
	rear_spring_stiffness_n_mm = max(rear_spring_stiffness_n_mm, 1.0)
	
	tire_peak_friction_coefficient = clamp(tire_peak_friction_coefficient, 0.1, 2.0)
	aero_drag_coefficient = max(aero_drag_coefficient, 0.01)
	
	steering_ratio = max(steering_ratio, 8.0)
	brake_force_distribution_front = clamp(brake_force_distribution_front, 0.3, 0.8)
	brake_force_distribution_rear = 1.0 - brake_force_distribution_front

# Property setters with validation
func _set_gravity(value: float) -> void:
	gravity = max(value, 0.1)
	changed.emit()

func _set_physics_tick_rate(value: int) -> void:
	physics_tick_rate = clamp(value, 60, 240)
	changed.emit()

func _set_max_substeps(value: int) -> void:
	max_substeps = clamp(value, 1, 8)
	changed.emit()

func _set_time_scale(value: float) -> void:
	time_scale = clamp(value, 0.0, 2.0)
	changed.emit()

func _set_default_vehicle_mass(value: float) -> void:
	default_vehicle_mass = max(value, 500.0)
	changed.emit()

func _set_default_wheel_radius(value: float) -> void:
	default_wheel_radius = max(value, 0.1)
	changed.emit()

func _set_default_wheel_width(value: float) -> void:
	default_wheel_width = max(value, 0.1)
	changed.emit()

func _set_default_com_height(value: float) -> void:
	default_center_of_mass_height = max(value, 0.1)
	changed.emit()

func _set_default_com_x(value: float) -> void:
	default_center_of_mass_x = value
	changed.emit()

func _set_default_track_width(value: float) -> void:
	default_track_width = max(value, 0.5)
	changed.emit()

func _set_default_wheelbase(value: float) -> void:
	default_wheelbase = max(value, 1.5)
	changed.emit()

func _set_engine_max_rpm(value: float) -> void:
	engine_max_rpm = max(value, 2000.0)
	changed.emit()

func _set_engine_idle_rpm(value: float) -> void:
	engine_idle_rpm = min(max(value, 400.0), engine_max_rpm * 0.3)
	changed.emit()

func _set_engine_peak_torque_rpm(value: float) -> void:
	engine_peak_torque_rpm = clamp(value, engine_idle_rpm, engine_max_rpm * 0.95)
	changed.emit()

func _set_engine_peak_torque(value: float) -> void:
	engine_peak_torque_nm = max(value, 50.0)
	changed.emit()

func _set_engine_max_torque_factor(value: float) -> void:
	engine_max_torque_factor = clamp(value, 1.0, 2.0)
	changed.emit()

func _set_final_drive_ratio(value: float) -> void:
	final_drive_ratio = max(value, 1.5)
	changed.emit()

func _set_gear_raties(value: Array[float]) -> void:
	gear_ratios = []
	for r in value:
		gear_ratios.append(max(r, 0.1))
	changed.emit()

func _set_clutch_friction(value: float) -> void:
	clutch_friction_coefficient = clamp(value, 0.5, 2.0)
	changed.emit()

func _set_flywheel_inertia(value: float) -> void:
	flywheel_inertia_kg_m2 = max(value, 0.01)
	changed.emit()

func _set_diff_lock_front(value: float) -> void:
	diff_lock_front = clamp(value, 0.0, 1.0)
	changed.emit()

func _set_diff_lock_rear(value: float) -> void:
	diff_lock_rear = clamp(value, 0.0, 1.0)
	changed.emit()

func _set_diff_preload(value: float) -> void:
	diff_preload_torque_nm = max(value, 0.0)
	changed.emit()

func _set_diff_slip_threshold(value: float) -> void:
	diff_slip_threshold_rad_s = max(value, 0.1)
	changed.emit()

func _set_front_spring_stiffness(value: float) -> void:
	front_spring_stiffness_n_mm = max(value, 1.0)
	changed.emit()

func _set_front_damping_comp(value: float) -> void:
	front_damping_compression_n_mm = max(value, 0.5)
	changed.emit()

func _set_front_damping_rebound(value: float) -> void:
	front_damping_rebound_n_mm = max(value, 0.5)
	changed.emit()

func _set_front_travel(value: float) -> void:
	front_suspension_travel_mm = max(value, 50.0)
	changed.emit()

func _set_front_arb(value: float) -> void:
	front_anti_roll_bar_stiffness_n_mm = max(value, 0.5)
	changed.emit()

func _set_front_camber_gain(value: float) -> void:
	front_camber_gain_per_mm = clamp(value, 0.001, 0.05)
	changed.emit()

func _set_front_toe_gain(value: float) -> void:
	front_toe_gain_per_mm = clamp(value, 0.001, 0.03)
	changed.emit()

func _set_rear_spring_stiffness(value: float) -> void:
	rear_spring_stiffness_n_mm = max(value, 1.0)
	changed.emit()

func _set_rear_damping_comp(value: float) -> void:
	rear_damping_compression_n_mm = max(value, 0.5)
	changed.emit()

func _set_rear_damping_rebound(value: float) -> void:
	rear_damping_rebound_n_mm = max(value, 0.5)
	changed.emit()

func _set_rear_travel(value: float) -> void:
	rear_suspension_travel_mm = max(value, 50.0)
	changed.emit()

func _set_rear_arb(value: float) -> void:
	rear_anti_roll_bar_stiffness_n_mm = max(value, 0.5)
	changed.emit()

func _set_rear_camber_gain(value: float) -> void:
	rear_camber_gain_per_mm = clamp(value, 0.001, 0.05)
	changed.emit()

func _set_rear_toe_gain(value: float) -> void:
	rear_toe_gain_per_mm = clamp(value, 0.001, 0.03)
	changed.emit()

func _set_tire_peak_friction(value: float) -> void:
	tire_peak_friction_coefficient = clamp(value, 0.1, 2.0)
	changed.emit()

func _set_tire_sliding_friction(value: float) -> void:
	tire_sliding_friction_coefficient = clamp(value, 0.05, 1.5)
	changed.emit()

func _set_tire_b_coefficient(value: float) -> void:
	tire_pacejka_b_coefficient = clamp(value, 2.0, 15.0)
	changed.emit()

func _set_tire_c_coefficient(value: float) -> void:
	tire_pacejka_c_coefficient = clamp(value, 1.0, 3.0)
	changed.emit()

func _set_tire_d_coefficient(value: float) -> void:
	tire_pacejka_d_coefficient = clamp(value, 0.1, 1.0)
	changed.emit()

func _set_tire_e_coefficient(value: float) -> void:
	tire_pacejka_e_coefficient = clamp(value, -1.0, 1.0)
	changed.emit()

func _set_tire_vertical_stiffness(value: float) -> void:
	tire_vertical_stiffness_n_mm = max(value, 50.0)
	changed.emit()

func _set_tire_longitudinal_stiffness(value: float) -> void:
	tire_longitudinal_stiffness_n_mm = max(value, 50.0)
	changed.emit()

func _set_tire_lateral_stiffness(value: float) -> void:
	tire_lateral_stiffness_n_mm = max(value, 50.0)
	changed.emit()

func _set_tire_pressure(value: float) -> void:
	tire_pressure_bar = clamp(value, 1.5, 3.5)
	changed.emit()

func _set_tire_temp_opt_min(value: float) -> void:
	tire_temperature_optimal_min_c = max(value, 20.0)
	changed.emit()

func _set_tire_temp_opt_max(value: float) -> void:
	tire_temperature_optimal_max_c = max(value, 50.0)
	changed.emit()

func _set_tire_temp_effective_range(value: float) -> void:
	tire_temperature_effective_range_c = max(value, 10.0)
	changed.emit()

func _set_aero_drag_coeff(value: float) -> void:
	aero_drag_coefficient = max(value, 0.01)
	changed.emit()

func _set_aero_frontal_area(value: float) -> void:
	aero_frontal_area_m2 = max(value, 1.0)
	changed.emit()

func _set_aero_downforce_coeff(value: float) -> void:
	aero_downforce_coefficient = max(value, 0.0)
	changed.emit()

func _set_aero_front_balance(value: float) -> void:
	aero_front_balance = clamp(value, 0.2, 0.8)
	changed.emit()

func _set_aero_lift_coeff(value: float) -> void:
	aero_lift_coefficient = value
	changed.emit()

func _set_aero_side_force(value: float) -> void:
	aero_side_force_coefficient = clamp(value, 0.0, 0.5)
	changed.emit()

func _set_aero_ground_clearance(value: float) -> void:
	aero_ground_clearance_mm = max(value, 50.0)
	changed.emit()

func _set_aero_stall_speed(value: float) -> void:
	aero_stall_speed_ms = max(value, 50.0)
	changed.emit()

func _set_steering_ratio(value: float) -> void:
	steering_ratio = max(value, 8.0)
	changed.emit()

func _set_steering_lock(value: float) -> void:
	steering_lock_angle_deg = max(value, 360.0)
	changed.emit()

func _set_steering_deadband(value: float) -> void:
	steering_deadband_deg = clamp(value, 0.0, 10.0)
	changed.emit()

func _set_steering_return(value: float) -> void:
	steering_return_rate = clamp(value, 0.1, 1.0)
	changed.emit()

func _set_steering_nonlinear(value: float) -> void:
	steering_nonlinearity_curve = clamp(value, 0.0, 0.5)
	changed.emit()

func _set_steering_assist(value: float) -> void:
	steering_power_assist_ratio = clamp(value, 0.0, 1.0)
	changed.emit()

func _set_brake_dist_front(value: float) -> void:
	brake_force_distribution_front = clamp(value, 0.3, 0.8)
	changed.emit()

func _set_brake_dist_rear(value: float) -> void:
	brake_force_distribution_rear = 1.0 - brake_force_distribution_front
	changed.emit()

func _set_brake_pad_friction(value: float) -> void:
	brake_pad_friction_coefficient = clamp(value, 0.2, 0.6)
	changed.emit()

func _set_brake_rotor_radius(value: float) -> void:
	brake_rotor_radius_mm = max(value, 50.0)
	changed.emit()

func _set_brake_piston_count(value: int) -> void:
	brake_caliper_piston_count = clamp(value, 1, 12)
	changed.emit()

func _set_brake_piston_area(value: float) -> void:
	brake_caliper_piston_area_mm2 = max(value, 50.0)
	changed.emit()

func _set_brake_max_clamp(value: float) -> void:
	brake_max_clamping_force_n = max(value, 1000.0)
	changed.emit()

func _set_brake_hysteresis(value: float) -> void:
	brake_hysteresis_factor = clamp(value, 0.0, 0.5)
	changed.emit()

func _set_brake_bleed_thresh(value: float) -> void:
	brake_bleed_threshold_percent = clamp(value, 0.0, 20.0)
	changed.emit()

func _set_chassis_stiffness_front(value: float) -> void:
	chassis_stiffness_front_n_mm = max(value, 1000.0)
	changed.emit()

func _set_chassis_stiffness_rear(value: float) -> void:
	chassis_stiffness_rear_n_mm = max(value, 1000.0)
	changed.emit()

func _set_roll_center_front(value: float) -> void:
	roll_center_height_front_mm = max(value, 50.0)
	changed.emit()

func _set_roll_center_rear(value: float) -> void:
	roll_center_height_rear_mm = max(value, 50.0)
	changed.emit()

func _set_cg_offset_x(value: float) -> void:
	cg_offset_x_mm = value
	changed.emit()

func _set_cg_offset_y(value: float) -> void:
	cg_offset_y_mm = value
	changed.emit()

func _set_cg_offset_z(value: float) -> void:
	cg_offset_z_mm = value
	changed.emit()

func _set_body_swing_period(value: float) -> void:
	body_swing_period_s = max(value, 0.1)
	changed.emit()

func _set_damping_ratio_body(value: float) -> void:
	damping_ratio_body_mode = clamp(value, 0.05, 0.5)
	changed.emit()

func _set_tc_enabled(value: bool) -> void:
	tc_enabled = value
	changed.emit()

func _set_tc_max_slip(value: float) -> void:
	tc_max_slip_ratio = clamp(value, 0.05, 0.3)
	changed.emit()

func _set_tc_integrator(value: float) -> void:
	tc_integrator_gain = clamp(value, 0.1, 1.0)
	changed.emit()

func _set_tc_proportional(value: float) -> void:
	tc_proportional_gain = clamp(value, 0.5, 5.0)
	changed.emit()

func _set_tc_delay(value: float) -> void:
	tc_reaction_delay_s = max(value, 0.01)
	changed.emit()

func _set_tc_recovery(value: float) -> void:
	tc_recovery_rate = clamp(value, 0.5, 1.0)
	changed.emit()

func _set_abs_enabled(value: bool) -> void:
	abs_enabled = value
	changed.emit()

func _set_abs_slip_target(value: float) -> void:
	abs_slip_target = clamp(value, 0.05, 0.3)
	changed.emit()

func _set_abs_freq(value: float) -> void:
	abs_modulation_frequency_hz = clamp(value, 5.0, 30.0)
	changed.emit()

func _set_abs_drop_rate(value: float) -> void:
	abs_pressure_drop_rate_bar_s = max(value, 50.0)
	changed.emit()

func _set_abs_build_rate(value: float) -> void:
	abs_pressure_build_rate_bar_s = max(value, 50.0)
	changed.emit()

func _set_abs_recovery(value: float) -> void:
	abs_recovery_threshold = clamp(value, 0.01, 0.15)
	changed.emit()

func _set_ai_skill(value: float) -> void:
	ai_skill_modifier = clamp(value, 0.5, 2.0)
	changed.emit()

func _set_player_skill(value: float) -> void:
	player_skill_modifier = clamp(value, 0.5, 2.0)
	changed.emit()

func _set_damage_enabled(value: bool) -> void:
	damage_enabled = value
	changed.emit()

func _set_collision_vol(value: float) -> void:
	collision_sound_volume = clamp(value, 0.0, 1.0)
	changed.emit()

func _set_skidmarks(value: bool) -> void:
	skid_mark_enabled = value
	changed.emit()

func _set_particle_quality(value: int) -> void:
	particle_quality = clamp(value, 1, 3)
	changed.emit()

func _on_ready() -> void:
	_validate_all()

</script>