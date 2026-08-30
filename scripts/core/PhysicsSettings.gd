extends Resource
class_name PhysicsSettings

## PhysicsSettings - Centralized physics constants and configuration for the racing simulator
## All physics values are defined here so they can be tweaked without touching simulation code
## This resource provides a single source of truth for vehicle dynamics tuning

@export_group("Global Physics Constants")
@export var gravity: float = 9.81: set = _set_gravity
@export var physics_tick_rate: int = 120: set = _set_physics_tick_rate
@export var max_substeps: int = 4: set = _set_max_substeps
@export var time_scale: float = 1.0: set = _set_time_scale

@export_group("Vehicle Physics Defaults")
@export var default_vehicle_mass: float = 1500.0: set = _set_default_vehicle_mass
@export var default_wheel_radius: float = 0.33: set = _set_default_wheel_radius
@export var default_wheel_width: float = 0.215: set = _set_default_wheel_width
@export var default_wheel_mass: float = 25.0: set = _set_default_wheel_mass
@export var default_wheel_inertia: float = 1.2: set = _set_default_wheel_inertia
@export var chassis_height: float = 0.45: set = _set_chassis_height
@export var chassis_width: float = 1.85: set = _set_chassis_width
@export var chassis_length: float = 4.5: set = _set_chassis_length
@export var center_of_mass_offset: Vector3 = Vector3(0.0, 0.25, 0.0): set = _set_center_of_mass_offset

@export_group("Engine & Powertrain")
@export var engine_max_power: float = 250.0: set = _set_engine_max_power
@export var engine_max_torque: float = 500.0: set = _set_engine_max_torque
@export var engine_peak_power_rpm: float = 6500.0: set = _set_engine_peak_power_rpm
@export var engine_peak_torque_rpm: float = 4500.0: set = _set_engine_peak_torque_rpm
@export var idle_rpm: float = 800.0: set = _set_idle_rpm
@export var redline_rpm: float = 8000.0: set = _set_redline_rpm
@export var revlimiter_rpm: float = 8200.0: set = _set_revlimiter_rpm
@export var throttle_response_curve: float = 0.85: set = _set_throttle_response_curve
@export var engine_braking_factor: float = 0.3: set = _set_engine_braking_factor
@export var clutch_slip_factor: float = 0.95: set = _set_clutch_slip_factor

@export_group("Torque Curve Configuration")
@export var torque_curve_points: Array[Vector2f] = []: set = _set_torque_curve_points
@export var power_curve_points: Array[Vector2f] = []: set = _set_power_curve_points
@export var use_custom_torque_curve: bool = false: set = _set_use_custom_torque_curve

@export_group("Transmission")
@export var transmission_type: TransmissionType = TransmissionType.MANUAL
@export var gear_count: int = 6: set = _set_gear_count
@export var gear_ratios: Array[float] = [3.8, 2.2, 1.6, 1.2, 0.9, 0.75]: set = _set_gear_ratios
@export var reverse_gear_ratio: float = 3.5: set = _set_reverse_gear_ratio
@export var final_drive_ratio: float = 3.5: set = _set_final_drive_ratio
@export var shift_up_delay_ms: int = 150: set = _set_shift_up_delay_ms
@export var shift_down_delay_ms: int = 100: set = _set_shift_down_delay_ms
@export var rev_matching_enabled: bool = true: set = _set_rev_matching_enabled
@export var rev_matching_tolerance: float = 50.0: set = _set_rev_matching_tolerance

@export_group("Differential Settings")
@export var differential_type: DifferentialType = DifferentialType.LSD
@export var open_diff_lock: float = 0.0: set = _set_open_diff_lock
@export var lsd_preload: float = 0.1: set = _set_lsd_preload
@export var lsd_max_lock: float = 0.8: set = _set_lsd_max_lock
@export var mechanical_lsd_bias: float = 0.6: set = _set_mechanical_lsd_bias
@export var viscous_coupling_constant: float = 0.5: set = _set_viscous_coupling_constant
@export var center_diff_type: CenterDiffType = CenterDiffType.OPEN
@export var center_diff_lock: float = 0.0: set = _set_center_diff_lock

@export_group("Suspension Parameters")
@export_group("Front Suspension")
@export var front_suspension_stiffness: float = 35000.0: set = _set_front_suspension_stiffness
@export var front_suspension_compression_damping: float = 3500.0: set = _set_front_suspension_compression_damping
@export var front_suspension_rebound_damping: float = 2500.0: set = _set_front_suspension_rebound_damping
@export var front_suspension_travel: float = 0.15: set = _set_front_suspension_travel
@export var front_suspension_bottomout_buffer: float = 0.02: set = _set_front_suspension_bottomout_buffer
@export var front_suspension_topout_buffer: float = 0.02: set = _set_front_suspension_topout_buffer

@export_group("Rear Suspension")
@export var rear_suspension_stiffness: float = 32000.0: set = _set_rear_suspension_stiffness
@export var rear_suspension_compression_damping: float = 3200.0: set = _set_rear_suspension_compression_damping
@export var rear_suspension_rebound_damping: float = 2200.0: set = _set_rear_suspension_rebound_damping
@export var rear_suspension_travel: float = 0.15: set = _set_rear_suspension_travel
@export var rear_suspension_bottomout_buffer: float = 0.02: set = _set_rear_suspension_bottomout_buffer
@export var rear_suspension_topout_buffer: float = 0.02: set = _set_rear_suspension_topout_buffer

@export_group("Anti-Roll Bars")
@export var front_anti_roll_bar_stiffness: float = 1200.0: set = _set_front_anti_roll_bar_stiffness
@export var rear_anti_roll_bar_stiffness: float = 1000.0: set = _set_rear_anti_roll_bar_stiffness
@export var adjustable_arb: bool = true: set = _set_adjustable_arb

@export_group("Tire Friction Models")
@export var tire_model: TireModel = TireModel.PACEJKA_SIMPLIFIED
@export var tire_friction_coefficient: float = 1.2: set = _set_tire_friction_coefficient
@export var lateral_friction_peak: float = 1.3: set = _set_lateral_friction_peak
@export var lateral_friction_transition: float = 0.1: set = _set_lateral_friction_transition
@export var longitudinal_friction_peak: float = 1.4: set = _set_longitudinal_friction_peak
@export var longitudinal_friction_transition: float = 0.08: set = _set_longitudinal_friction_transition
@export var slip_angle_degrees_at_peak: float = 6.5: set = _set_slip_angle_degrees_at_peak
@export var slip_ratio_at_peak: float = 0.12: set = _set_slip_ratio_at_peak

@export_group("Pacejka Coefficients (Simplified)")
@export var pacejka_b: float = 1.5: set = _set_pacejka_b
@export var pacejka_c: float = 1.5: set = _set_pacejka_c
@export var pacejka_d: float = 1.2: set = _set_pacejka_d
@export var pacejka_e: float = 0.9: set = _set_pacejka_e

@export_group("Tire Temperature Model")
@export var tire_min_temperature: float = -10.0: set = _set_tire_min_temperature
@export var tire_optimal_temperature: float = 80.0: set = _set_tire_optimal_temperature
@export var tire_max_temperature: float = 120.0: set = _set_tire_max_temperature
@export var tire_heat_capacity: float = 500.0: set = _set_tire_heat_capacity
@export var tire_cooling_rate: float = 0.5: set = _set_tire_cooling_rate
@export var track_temperature: float = 25.0: set = _set_track_temperature

@export_group("Aerodynamics")
@export var aero_drag_coefficient: float = 0.30: set = _set_aero_drag_coefficient
@export var aero_downforce_coefficient: float = 0.8: set = _set_aero_downforce_coefficient
@export var aero_lift_coefficient: float = -0.1: set = _set_aero_lift_coefficient
@export var frontal_area: float = 2.1: set = _set_frontal_area
@export var wing_angle: float = 0.0: set = _set_wing_angle
@export var ground_effect_enabled: bool = true: set = _set_ground_effect_enabled
@export var ground_effect_distance: float = 0.05: set = _set_ground_effect_distance
@export var ground_effect_strength: float = 1.5: set = _set_ground_effect_strength
@export var crosswind_sensitivity: float = 0.5: set = _set_crosswind_sensitivity

@export_group("Steering System")
@export var steering_lock_angle: float = 45.0: set = _set_steering_lock_angle
@export var steering_ratio: float = 14.0: set = _set_steering_ratio
@export var steering_spring_force: float = 1.0: set = _set_steering_spring_force
@export var steering_return_speed: float = 3.0: set = _set_steering_return_speed
@export var steering_deadzone: float = 0.02: set = _set_steering_deadzone
@export var steering_max_input: float = 1.0: set = _set_steering_max_input
@export var steering_linearity: float = 0.9: set = _set_steering_linearity
@export var rack_travel_mm: float = 150.0: set = _set_rack_travel_mm

@export_group("Brake System")
@export var brake_force_distribution: Vector2 = Vector2(0.6, 0.4): set = _set_brake_force_distribution
@export var brake_pressure_multiplier: float = 8.0: set = _set_brake_pressure_multiplier
@export var brake_bias_favor_front: bool = true: set = _set_brake_bias_favor_front
@export var brake_pad_compound: PadCompound = PadCompound.RACING
@export var brake_disc_size_front: float = 0.35: set = _set_brake_disc_size_front
@export var brake_disc_size_rear: float = 0.30: set = _set_brake_disc_size_rear
@export var brake_caliper_pistons: int = 4: set = _set_brake_caliper_pistons
@export var brake_rotor_thickness: float = 0.032: set = _set_brake_rotor_thickness

@export_group("ABS & Traction Control")
@export var abs_enabled: bool = true: set = _set_abs_enabled
@export var abs_threshold: float = 0.15: set = _set_abs_threshold
@export var abs_pulse_frequency: float = 15.0: set = _set_abs_pulse_frequency
@export var abs_recovery_rate: float = 0.3: set = _set_abs_recovery_rate
@export var traction_control_enabled: bool = true: set = _set_traction_control_enabled
@export var tc_interference_level: float = 0.5: set = _set_tc_interference_level
@export var tc_slip_target: float = 0.10: set = _set_tc_slip_target

@export_group("Collision & Damage")
@export var collision_accuracy: CollisionAccuracy = CollisionAccuracy.HIGH
@export var damage_enabled: bool = true: set = _set_damage_enabled
@export var deformation_enabled: bool = false: set = _set_deformation_enabled
@export var crash_force_threshold: float = 50000.0: set = _set_crash_force_threshold
@export var crumple_zone_front: float = 0.5: set = _set_crumple_zone_front
@export var crumple_zone_rear: float = 0.4: set = _set_crumple_zone_rear
@export var chassis_torsional_stiffness: float = 25000.0: set = _set_chassis_torsional_stiffness

@export_group("Track Physics")
@export var track_surface_friction: float = 1.0: set = _set_track_surface_friction
@export var track_curbing_friction: float = 1.3: set = _set_track_curbing_friction
@export var track_gravel_friction: float = 0.6: set = _set_track_gravel_friction
@export var track_water_friction: float = 0.3: set = _set_track_water_friction
@export var track_dust_friction: float = 0.5: set = _set_track_dust_friction

@export_group("Simulation Quality")
@export var simulation_quality: SimulationQuality = SimulationQuality.BALANCED
@export var enable_jitter_reduction: bool = true: set = _set_enable_jitter_reduction
@export var jitter_reduction_factor: float = 0.95: set = _set_jitter_reduction_factor
@export var contact_patch_samples: int = 4: set = _set_contact_patch_samples
@export var raycast_debug_mode: bool = false: set = _set_raycast_debug_mode

# Enums
enum TransmissionType { MANUAL, AUTOMATIC, SEMI_AUTOMATIC, CVT }
enum DifferentialType { OPEN, LSD_VISCOSOUS, LSD_MECHANICAL, LOCKING }
enum CenterDiffType { OPEN, LIMITED_SLIP, TORSEN, HYDRAULIC }
enum TireModel { SIMPLIFIED, PACEJKA, PRESCOTT, FULL_PACEJKA }
enum PadCompound { STREET, SPORT, RACING, CERAMIC }
enum CollisionAccuracy { LOW, MEDIUM, HIGH, ULTRA }
enum SimulationQuality { FAST, BALANCED, PRECISE, ULTRA }

# Cached computed values
var _gravity_normalized: float = 1.0
var _tick_delta: float = 0.008333
var _gear_ratios_valid: bool = false

func _init() -> void:
	_init_defaults()

func _init_defaults() -> void:
	_compute_torque_and_power_curves()
	_validate_gear_ratios()
	_normalize_gravity()
	_compute_tick_delta()

func _set_gravity(value: float) -> void:
	if value > 0.0 and value < 50.0:
		gravity = value
		_normalize_gravity()

func _normalize_gravity() -> void:
	_gravity_normalized = 1.0 / gravity if gravity != 0 else 1.0

func _set_physics_tick_rate(value: int) -> void:
	if value >= 30 and value <= 240:
		physics_tick_rate = value
		_compute_tick_delta()

func _compute_tick_delta() -> void:
	_tick_delta = 1.0 / float(physics_tick_rate)

func _set_max_substeps(value: int) -> void:
	if value >= 1 and value <= 10:
		max_substeps = value

func _set_time_scale(value: float) -> void:
	if value > 0.0:
		time_scale = clamp(value, 0.0, 5.0)

func _set_default_vehicle_mass(value: float) -> void:
	if value > 500.0 and value < 5000.0:
		default_vehicle_mass = value

func _set_default_wheel_radius(value: float) -> void:
	if value > 0.1 and value < 1.0:
		default_wheel_radius = value

func _set_default_wheel_width(value: float) -> void:
	if value > 0.1 and value < 0.5:
		default_wheel_width = value

func _set_default_wheel_mass(value: float) -> void:
	if value > 5.0 and value < 100.0:
		default_wheel_mass = value

func _set_default_wheel_inertia(value: float) -> void:
	if value > 0.1 and value < 10.0:
		default_wheel_inertia = value

func _set_chassis_height(value: float) -> void:
	if value > 0.2 and value < 1.5:
		chassis_height = value

func _set_chassis_width(value: float) -> void:
	if value > 1.0 and value < 3.0:
		chassis_width = value

func _set_chassis_length(value: float) -> void:
	if value > 3.0 and value < 10.0:
		chassis_length = value

func _set_center_of_mass_offset(value: Vector3) -> void:
	center_of_mass_offset = value

func _set_engine_max_power(value: float) -> void:
	if value > 10.0 and value < 2000.0:
		engine_max_power = value
		_compute_torque_and_power_curves()

func _set_engine_max_torque(value: float) -> void:
	if value > 50.0 and value < 2000.0:
		engine_max_torque = value
		_compute_torque_and_power_curves()

func _set_engine_peak_power_rpm(value: float) -> void:
	if value > 1000.0 and value < 15000.0:
		engine_peak_power_rpm = value
		_compute_torque_and_power_curves()

func _set_engine_peak_torque_rpm(value: float) -> void:
	if value > 500.0 and value < 10000.0:
		engine_peak_torque_rpm = value
		_compute_torque_and_power_curves()

func _set_idle_rpm(value: float) -> void:
	if value > 200.0 and value < 2000.0:
		idle_rpm = value

func _set_redline_rpm(value: float) -> void:
	if value > engine_peak_power_rpm + 500.0:
		redline_rpm = value
		revlimiter_rpm = value + 200.0

func _set_revlimiter_rpm(value: float) -> void:
	if value > redline_rpm:
		revlimiter_rpm = value

func _set_throttle_response_curve(value: float) -> void:
	throttle_response_curve = clamp(value, 0.1, 1.5)

func _set_engine_braking_factor(value: float) -> void:
	engine_braking_factor = clamp(value, 0.0, 1.0)

func _set_clutch_slip_factor(value: float) -> void:
	clutch_slip_factor = clamp(value, 0.5, 1.0)

func _set_torque_curve_points(value: Array[Vector2f]) -> void:
	torque_curve_points = value
	use_custom_torque_curve = len(value) > 0

func _set_power_curve_points(value: Array[Vector2f]) -> void:
	power_curve_points = value

func _set_use_custom_torque_curve(value: bool) -> void:
	use_custom_torque_curve = value

func _set_gear_count(value: int) -> void:
	if value >= 4 and value <= 10:
		gear_count = value
		_gear_ratios_valid = false

func _set_gear_ratios(value: Array[float]) -> void:
	if len(value) >= gear_count:
		gear_ratios = value
		_gear_ratios_valid = true

func _set_reverse_gear_ratio(value: float) -> void:
	if value > 0.0:
		reverse_gear_ratio = value

func _set_final_drive_ratio(value: float) -> void:
	if value > 1.0 and value < 10.0:
		final_drive_ratio = value

func _set_shift_up_delay_ms(value: int) -> void:
	shift_up_delay_ms = clamp(value, 50, 500)

func _set_shift_down_delay_ms(value: int) -> void:
	shift_down_delay_ms = clamp(value, 50, 300)

func _set_rev_matching_enabled(value: bool) -> void:
	rev_matching_enabled = value

func _set_rev_matching_tolerance(value: float) -> void:
	rev_matching_tolerance = clamp(value, 10.0, 200.0)

func _set_open_diff_lock(value: float) -> void:
	open_diff_lock = clamp(value, 0.0, 1.0)

func _set_lsd_preload(value: float) -> void:
	lsd_preload = clamp(value, 0.0, 0.5)

func _set_lsd_max_lock(value: float) -> void:
	lsd_max_lock = clamp(value, 0.1, 1.0)

func _set_mechanical_lsd_bias(value: float) -> void:
	mechanical_lsd_bias = clamp(value, 0.0, 1.0)

func _set_viscous_coupling_constant(value: float) -> void:
	viscous_coupling_constant = clamp(value, 0.0, 1.0)

func _set_center_diff_lock(value: float) -> void:
	center_diff_lock = clamp(value, 0.0, 1.0)

func _set_front_suspension_stiffness(value: float) -> void:
	front_suspension_stiffness = clamp(value, 5000.0, 100000.0)

func _set_front_suspension_compression_damping(value: float) -> void:
	front_suspension_compression_damping = clamp(value, 500.0, 10000.0)

func _set_front_suspension_rebound_damping(value: float) -> void:
	front_suspension_rebound_damping = clamp(value, 500.0, 10000.0)

func _set_front_suspension_travel(value: float) -> void:
	front_suspension_travel = clamp(value, 0.05, 0.30)

func _set_front_suspension_bottomout_buffer(value: float) -> void:
	front_suspension_bottomout_buffer = clamp(value, 0.0, 0.10)

func _set_front_suspension_topout_buffer(value: float) -> void:
	front_suspension_topout_buffer = clamp(value, 0.0, 0.10)

func _set_rear_suspension_stiffness(value: float) -> void:
	rear_suspension_stiffness = clamp(value, 5000.0, 100000.0)

func _set_rear_suspension_compression_damping(value: float) -> void:
	rear_suspension_compression_damping = clamp(value, 500.0, 10000.0)

func _set_rear_suspension_rebound_damping(value: float) -> void:
	rear_suspension_rebound_damping = clamp(value, 500.0, 10000.0)

func _set_rear_suspension_travel(value: float) -> void:
	rear_suspension_travel = clamp(value, 0.05, 0.30)

func _set_rear_suspension_bottomout_buffer(value: float) -> void:
	rear_suspension_bottomout_buffer = clamp(value, 0.0, 0.10)

func _set_rear_suspension_topout_buffer(value: float) -> void:
	rear_suspension_topout_buffer = clamp(value, 0.0, 0.10)

func _set_front_anti_roll_bar_stiffness(value: float) -> void:
	front_anti_roll_bar_stiffness = clamp(value, 100.0, 5000.0)

func _set_rear_anti_roll_bar_stiffness(value: float) -> void:
	rear_anti_roll_bar_stiffness = clamp(value, 100.0, 5000.0)

func _set_adjustable_arb(value: bool) -> void:
	adjustable_arb = value

func _set_tire_friction_coefficient(value: float) -> void:
	tire_friction_coefficient = clamp(value, 0.3, 2.0)

func _set_lateral_friction_peak(value: float) -> void:
	lateral_friction_peak = clamp(value, 0.8, 2.5)

func _set_lateral_friction_transition(value: float) -> void:
	lateral_friction_transition = clamp(value, 0.01, 0.5)

func _set_longitudinal_friction_peak(value: float) -> void:
	longitudinal_friction_peak = clamp(value, 0.8, 2.5)

func _set_longitudinal_friction_transition(value: float) -> void:
	longitudinal_friction_transition = clamp(value, 0.01, 0.5)

func _set_slip_angle_degrees_at_peak(value: float) -> void:
	slip_angle_degrees_at_peak = clamp(value, 2.0, 15.0)

func _set_slip_ratio_at_peak(value: float) -> void:
	slip_ratio_at_peak = clamp(value, 0.05, 0.30)

func _set_pacejka_b(value: float) -> void:
	pacejka_b = clamp(value, 1.0, 3.0)

func _set_pacejka_c(value: float) -> void:
	pacejka_c = clamp(value, 1.0, 2.0)

func _set_pacejka_d(value: float) -> void:
	pacejka_d = clamp(value, 0.5, 3.0)

func _set_pacejka_e(value: float) -> void:
	pacejka_e = clamp(value, 0.5, 1.5)

func _set_tire_min_temperature(value: float) -> void:
	tire_min_temperature = value

func _set_tire_optimal_temperature(value: float) -> void:
	tire_optimal_temperature = clamp(value, 50.0, 120.0)

func _set_tire_max_temperature(value: float) -> void:
	tire_max_temperature = clamp(value, 100.0, 200.0)

func _set_tire_heat_capacity(value: float) -> void:
	tire_heat_capacity = clamp(value, 100.0, 2000.0)

func _set_tire_cooling_rate(value: float) -> void:
	tire_cooling_rate = clamp(value, 0.1, 2.0)

func _set_track_temperature(value: float) -> void:
	track_temperature = clamp(value, -20.0, 60.0)

func _set_aero_drag_coefficient(value: float) -> void:
	aero_drag_coefficient = clamp(value, 0.1, 1.0)

func _set_aero_downforce_coefficient(value: float) -> void:
	aero_downforce_coefficient = clamp(value, -1.0, 5.0)

func _set_aero_lift_coefficient(value: float) -> void:
	aero_lift_coefficient = clamp(value, -2.0, 1.0)

func _set_frontal_area(value: float) -> void:
	frontal_area = clamp(value, 1.0, 5.0)

func _set_wing_angle(value: float) -> void:
	wing_angle = clamp(value, -10.0, 30.0)

func _set_ground_effect_enabled(value: bool) -> void:
	ground_effect_enabled = value

func _set_ground_effect_distance(value: float) -> void:
	ground_effect_distance = clamp(value, 0.01, 0.20)

func _set_ground_effect_strength(value: float) -> void:
	ground_effect_strength = clamp(value, 0.5, 5.0)

func _set_crosswind_sensitivity(value: float) -> void:
	crosswind_sensitivity = clamp(value, 0.0, 1.0)

func _set_steering_lock_angle(value: float) -> void:
	steering_lock_angle = clamp(value, 30.0, 90.0)

func _set_steering_ratio(value: float) -> void:
	steering_ratio = clamp(value, 8.0, 25.0)

func _set_steering_spring_force(value: float) -> void:
	steering_spring_force = clamp(value, 0.0, 2.0)

func _set_steering_return_speed(value: float) -> void:
	steering_return_speed = clamp(value, 1.0, 10.0)

func _set_steering_deadzone(value: float) -> void:
	steering_deadzone = clamp(value, 0.0, 0.1)

func _set_steering_max_input(value: float) -> void:
	steering_max_input = clamp(value, 0.5, 2.0)

func _set_steering_linearity(value: float) -> void:
	steering_linearity = clamp(value, 0.5, 1.0)

func _set_rack_travel_mm(value: float) -> void:
	rack_travel_mm = clamp(value, 100.0, 250.0)

func _set_brake_force_distribution(value: Vector2) -> void:
	var total = value.x + value.y
	if total > 0.0:
		brake_force_distribution = Vector2(value.x / total, value.y / total)

func _set_brake_pressure_multiplier(value: float) -> void:
	brake_pressure_multiplier = clamp(value, 2.0, 20.0)

func _set_brake_bias_favor_front(value: bool) -> void:
	brake_bias_favor_front = value

func _set_brake_disc_size_front(value: float) -> void:
	brake_disc_size_front = clamp(value, 0.20, 0.45)

func _set_brake_disc_size_rear(value: float) -> void:
	brake_disc_size_rear = clamp(value, 0.20, 0.40)

func _set_brake_caliper_pistons(value: int) -> void:
	brake_caliper_pistons = clamp(value, 2, 12)

func _set_brake_rotor_thickness(value: float) -> void:
	brake_rotor_thickness = clamp(value, 0.015, 0.060)

func _set_abs_enabled(value: bool) -> void:
	abs_enabled = value

func _set_abs_threshold(value: float) -> void:
	abs_threshold = clamp(value, 0.05, 0.30)

func _set_abs_pulse_frequency(value: float) -> void:
	abs_pulse_frequency = clamp(value, 5.0, 50.0)

func _set_abs_recovery_rate(value: float) -> void:
	abs_recovery_rate = clamp(value, 0.1, 0.9)

func _set_traction_control_enabled(value: bool) -> void:
	traction_control_enabled = value

func _set_tc_interference_level(value: float) -> void:
	tc_interference_level = clamp(value, 0.0, 1.0)

func _set_tc_slip_target(value: float) -> void:
	tc_slip_target = clamp(value, 0.05, 0.25)

func _set_damage_enabled(value: bool) -> void:
	damage_enabled = value

func _set_deformation_enabled(value: bool) -> void:
	deformation_enabled = value

func _set_crash_force_threshold(value: float) -> void:
	crash_force_threshold = clamp(value, 10000.0, 200000.0)

func _set_crumple_zone_front(value: float) -> void:
	crumple_zone_front = clamp(value, 0.1, 1.0)

func _set_crumple_zone_rear(value: float) -> void:
	crumple_zone_rear = clamp(value, 0.1, 1.0)

func _set_chassis_torsional_stiffness(value: float) -> void:
	chassis_torsional_stiffness = clamp(value, 5000.0, 100000.0)

func _set_track_surface_friction(value: float) -> void:
	track_surface_friction = clamp(value, 0.3, 1.5)

func _set_track_curbing_friction(value: float) -> void:
	track_curbing_friction = clamp(value, 0.5, 2.0)

func _set_track_gravel_friction(value: float) -> void:
	track_gravel_friction = clamp(value, 0.1, 1.0)

func _set_track_water_friction(value: float) -> void:
	track_water_friction = clamp(value, 0.05, 0.5)

func _set_track_dust_friction(value: float) -> void:
	track_dust_friction = clamp(value, 0.2, 0.8)

func _set_simulation_quality(value: SimulationQuality) -> void:
	simulation_quality = value

func _set_enable_jitter_reduction(value: bool) -> void:
	enable_jitter_reduction = value

func _set_jitter_reduction_factor(value: float) -> void:
	jitter_reduction_factor = clamp(value, 0.8, 1.0)

func _set_contact_patch_samples(value: int) -> void:
	contact_patch_samples = clamp(value, 1, 8)

func _set_raycast_debug_mode(value: bool) -> void:
	raycast_debug_mode = value

## Get the normalized gravity factor (1/g)
func get_gravity_normalized() -> float:
	return _gravity_normalized

## Get physics tick delta time
func get_tick_delta() -> float:
	return _tick_delta

## Compute torque at given RPM using curve interpolation
func get_torque_at_rpm(rpm: float) -> float:
	if not use_custom_torque_curve or len(torque_curve_points) == 0:
		return _get_default_torque_curve(rpm)
	
	return _interpolate_torque_curve(rpm)

## Compute power at given RPM using curve interpolation
func get_power_at_rpm(rpm: float) -> float:
	if not use_custom_torque_curve or len(power_curve_points) == 0:
		return _get_default_power_curve(rpm)
	
	return _interpolate_power_curve(rpm)

## Get wheel radius
func get_wheel_radius() -> float:
	return default_wheel_radius

## Get wheel width
func get_wheel_width() -> float:
	return default_wheel_width

## Get wheel mass
func get_wheel_mass() -> float:
	return default_wheel_mass

## Get wheel inertia
func get_wheel_inertia() -> float:
	return default_wheel_inertia

## Get current gear ratio for a gear index
func get_gear_ratio(gear_index: int) -> float:
	if gear_index < 0:
		return reverse_gear_ratio
	
	if not _gear_ratios_valid or gear_index >= len(gear_ratios):
		return gear_ratios[min(gear_index, len(gear_ratios) - 1)]
	
	return gear_ratios[gear_index]

## Get effective gear ratio including final drive
func get_effective_gear_ratio(gear_index: int) -> float:
	return get_gear_ratio(gear_index) * final_drive_ratio

## Calculate engine braking torque
func calculate_engine_braking_torque(engine_rpm: float) -> float:
	var base_torque = get_torque_at_rpm(engine_rpm)
	return base_torque * engine_braking_factor * -1.0

## Calculate downforce at given velocity (m/s)
func calculate_downforce(velocity: float) -> float:
	var dynamic_pressure = 0.5 * 1.225 * velocity * velocity
	return dynamic_pressure * aero_downforce_coefficient * frontal_area

## Calculate drag at given velocity (m/s)
func calculate_drag(velocity: float) -> float:
	var dynamic_pressure = 0.5 * 1.225 * velocity * velocity
	return dynamic_pressure * aero_drag_coefficient * frontal_area

## Calculate lift at given velocity (m/s)
func calculate_lift(velocity: float) -> float:
	var dynamic_pressure = 0.5 * 1.225 * velocity * velocity
	return dynamic_pressure * aero_lift_coefficient * frontal_area

## Get steering angle from input (-1 to 1)
func get_steering_angle_from_input(input: float) -> float:
	input = clamp(input, -1.0, 1.0)
	input = _apply_steering_linearity(input)
	return deg_to_rad(steering_lock_angle) * sign(input)

func _apply_steering_linearity(input: float) -> float:
	return input * pow(abs(input), 1.0 - steering_linearity) * sign(input)

## Apply Pacejka sine function for lateral force
func apply_pacejka_lateral(slip_angle_rad: float, normal_load: float) -> float:
	var alpha = abs(slip_angle_rad)
	var b = pacejka_b
	var c = pacejka_c
	var d = lateral_friction_peak * normal_load
	var e = pacejka_e
	
	var shape_factor = (1.0 - e) * sin(c * atan(b * alpha)) + e * sin(c * atan(b * alpha))
	return d * shape_factor

## Apply Pacejka function for longitudinal force
func apply_pacejka_longitudinal(slip_ratio: float, normal_load: float) -> float:
	var B = pacejka_b
	var C = pacejka_c
	var D = longitudinal_friction_peak * normal_load
	var E = pacejka_e
	
	var term = B * slip_ratio
	var result = D * sin(C * atan(term - E * (term - atan(term))))
	return result

## Convert vehicle speed from m/s to km/h
func convert_mps_to_kmh(speed: float) -> float:
	return speed * 3.6

## Convert vehicle speed from km/h to m/s
func convert_kmh_to_mps(speed: float) -> float:
	return speed / 3.6

## Convert RPM to radians per second
func convert_rpm_to_rad_per_sec(rpm: float) -> float:
	return rpm * 2.0 * PI / 60.0

## Convert radians per second to RPM
func convert_rad_per_sec_to_rpm(rads: float) -> float:
	return rads * 60.0 / (2.0 * PI)

## Calculate optimal tire temperature multiplier
func get_tire_temperature_multiplier(current_temp: float) -> float:
	if current_temp < tire_min_temperature:
		return 0.3
	elif current_temp <= tire_optimal_temperature:
		return 1.0
	elif current_temp <= tire_max_temperature:
		return 1.0 - ((current_temp - tire_optimal_temperature) / (tire_max_temperature - tire_optimal_temperature))
	else:
		return 0.1

## Get maximum cornering force based on tire temp
func get_max_cornering_force(normal_load: float) -> float:
	var temp_mult = get_tire_temperature_multiplier(track_temperature)
	return normal_load * lateral_friction_peak * temp_mult

## Get maximum acceleration/deceleration force based on tire temp
func get_max_traction_force(normal_load: float) -> float:
	var temp_mult = get_tire_temperature_multiplier(track_temperature)
	return normal_load * longitudinal_friction_peak * temp_mult

## Calculate theoretical vehicle weight on each wheel (static)
func get_static_weight_distribution() -> Dictionary:
	var total_weight = default_vehicle_mass * gravity
	var rear_weight_pct = 0.45  # Default rear-weight bias
	var front_weight = total_weight * (1.0 - rear_weight_pct)
	var rear_weight = total_weight * rear_weight_pct
	
	return {
		"front_axle": front_weight,
		"rear_axle": rear_weight,
		"left_wheel": front_weight * 0.5,
		"right_wheel": front_weight * 0.5,
		"total": total_weight
	}

## Validate gear ratios are descending (except reverse)
func _validate_gear_ratios() -> void:
	for i in range(len(gear_ratios) - 1):
		if gear_ratios[i] < gear_ratios[i + 1]:
			gear_ratios[i + 1] = gear_ratios[i] * 0.8
	_gear_ratios_valid = true

## Generate default torque curve if custom not provided
func _get_default_torque_curve(rpm: float) -> float:
	var clamped_rpm = clamp(rpm, idle_rpm, redline_rpm)
	var rpm_range = redline_rpm - idle_rpm
	var normalized = (clamped_rpm - idle_rpm) / rpm_range
	
	if normalized < 0.0 or normalized > 1.0:
		return 0.0
	
	var peak_pos = (engine_peak_torque_rpm - idle_rpm) / rpm_range
	
	if clamped_rpm < engine_peak_torque_rpm:
		return engine_max_torque * (1.0 - pow(1.0 - normalized / peak_pos, 2.0))
	else:
		return engine_max_torque * pow((redline_rpm - clamped_rpm) / (redline_rpm - engine_peak_torque_rpm), 2.0)

## Generate default power curve if custom not provided
func _get_default_power_curve(rpm: float) -> float:
	var torque = _get_default_torque_curve(rpm)
	var rpm_rad = convert_rpm_to_rad_per_sec(rpm)
	return torque * rpm_rad

## Interpolate custom torque curve
func _interpolate_torque_curve(rpm: float) -> float:
	if len(torque_curve_points) < 2:
		return _get_default_torque_curve(rpm)
	
	var sorted_points = torque_curve_points.duplicate()
	sort_points_by_x(sorted_points)
	
	for i in range(len(sorted_points) - 1):
		if rpm >= sorted_points[i].x and rpm <= sorted_points[i + 1].x:
			var t = (rpm - sorted_points[i].x) / (sorted_points[i + 1].x - sorted_points[i].x)
			return lerp(sorted_points[i].y, sorted_points[i + 1].y, t)
	
	if rpm < sorted_points[0].x:
		return sorted_points[0].y
	return sorted_points[len(sorted_points) - 1].y

## Interpolate custom power curve
func _interpolate_power_curve(rpm: float) -> float:
	if len(power_curve_points) < 2:
		return _get_default_power_curve(rpm)
	
	var sorted_points = power_curve_points.duplicate()
	sort_points_by_x(sorted_points)
	
	for i in range(len(sorted_points) - 1):
		if rpm >= sorted_points[i].x and rpm <= sorted_points[i + 1].x:
			var t = (rpm - sorted_points[i].x) / (sorted_points[i + 1].x - sorted_points[i].x)
			return lerp(sorted_points[i].y, sorted_points[i + 1].y, t)
	
	if rpm < sorted_points[0].x:
		return sorted_points[0].y
	return sorted_points[len(sorted_points) - 1].y

func sort_points_by_x(points: Array[Vector2f]) -> void:
	points.sort_custom(func(a: Vector2f, b: Vector2f) -> bool:
		return a.x < b.x)

func _notification(what: int) -> void:
	match what:
		NOTIFICATION_ENTER_TREE:
			_init_defaults()
		NOTIFICATION_THEME_CHANGED:
			pass
</FILE>
