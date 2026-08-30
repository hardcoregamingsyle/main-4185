extends Resource
class_name PhysicsSettings

## PhysicsSettings - Centralized physics constants and configuration for the racing simulator
## All physics values are defined here so they can be tweaked without touching simulation code

@export_group("Global Physics Constants")
@export var gravity: float = 9.81
@export var physics_tick_rate: int = 120  # Fixed timestep Hz for physics simulation
@export var max_substeps: int = 4

@export_group("Vehicle Physics Defaults")
@export var default_vehicle_mass: float = 1500.0  # kg
@export var default_wheel_radius: float = 0.33  # meters (approx 17 inch wheel)
@export var default_wheel_width: float = 0.215  # meters
@export var default_wheel_mass: float = 25.0  # kg per wheel
@export var default_wheel_inertia: float = 1.2  # kg*m^2

@export_group("Suspension Defaults")
@export var suspension_stiffness: float = 35000.0  # N/m
@export var suspension_compression_damping: float = 1200.0  # Ns/m
@export var suspension_rebound_damping: float = 600.0  # Ns/m
@export var max_suspension_travel: float = 0.25  # meters
@export var spring_rest_length: float = 0.35  # meters

@export_group("Tire Friction Parameters")
@export var tire_friction_horizontal: float = 1.2
@export var tire_friction_vertical: float = 0.3
@export var slip_threshold: float = 0.15
@export var lateral_slip_stiffness: float = 3.5e5
@export var longitudinal_slip_stiffness: float = 8.0e5

@export_group("Drift Physics")
@export var drift_multiplier: float = 1.5
@export var drift_recovery_speed: float = 2.0
@export var minimum_drift_angle: float = 15.0  # degrees
@export var maximum_drift_angle: float = 45.0  # degrees

@export_group("Aerodynamics")
@export var air_density: float = 1.225  # kg/m^3 at sea level
@export var drag_coefficient: float = 0.32  # typical sports car
@export var lift_coefficient: float = 0.15
@export var front_area: float = 2.2  # m^2 frontal area
@export var center_of_pressure_x: float = 0.5  # normalized vehicle length
@export var wing_downforce_coefficient: float = 0.8
@export var ground_effect_coefficient: float = 1.2
@export var downforce_falloff_distance: float = 0.1  # meters from ground

@export_group("Engine & Powertrain")
@export var engine_max_rpm: float = 8500.0
@export var engine_idle_rpm: float = 800.0
@export var engine_peak_torque_rpm: float = 5500.0
@export var peak_torque_nm: float = 450.0
@export var torque_curve_points: Array[Vector2] = [
	Vector2(0.0, 0.0),
	Vector2(0.1, 0.3),
	Vector2(0.3, 0.7),
	Vector2(0.5, 1.0),
	Vector2(0.7, 0.95),
	Vector2(0.85, 0.85),
	Vector2(1.0, 0.75)
]
@export var idle_throttle: float = 0.05

@export_group("Gear Ratios")
@export var gear_ratios: Array[float] = [3.8, 2.4, 1.7, 1.3, 1.0, 0.85]
@export var final_drive_ratio: float = 3.55
@export var reverse_gear_ratio: float = 3.2
@export var clutch_slip_factor: float = 0.95
@export var transmission_efficiency: float = 0.92

@export_group("Differential Settings")
@export var diff_type: String = "limited_slip"  # open, locked, limited_slip
@export var diff_lock_percentage: float = 0.35  # 0-1, higher = more lock
@export var diff_preload_torque: float = 15.0  # Nm
@export var diff_side_gear_ratio: float = 1.5

@export_group("Brake System")
@export var brake_force_distribution_front: float = 0.6
@export var brake_force_distribution_rear: float = 0.4
@export var max_brake_pressure_bar: float = 150.0
@export var brake_disc_radius: float = 0.29  # meters
@export var brake_piston_area: float = 0.0008  # m^2 per piston
@export var brake_pad_friction: float = 0.4
@export var brake_caliper_piston_count: int = 4
@export var brake_bias_adjustment: float = 0.0  # -1.0 to +1.0 manual bias

@export_group("Steering System")
@export var steering_ratio: float = 14.0  # ratio of steering wheel angle to wheel angle
@export var max_steering_angle: float = 35.0  # degrees
@export var steering_response_rate: float = 25.0  # degrees per second
@export var steering_centering_torque: float = 2.5  # Nm
@export var power_assist_ratio: float = 3.0
@export var bump_stop_angle: float = 40.0  # degrees before mechanical limit

@export_group("Chassis Dynamics")
@export var track_width_front: float = 1.62  # meters
@export var track_width_rear: float = 1.58  # meters
@export var wheelbase: float = 2.70  # meters
@export var cg_height: float = 0.55  # meters above ground
@export var cg_offset_front: float = 0.45  # meters forward from center
@export var body_roll_coefficient: float = 0.002
@export var pitch_under_acceleration: float = 0.001
@export var pitch_under_braking: float = 0.0015

@export_group("Gameplay Tuning")
@export var lap_time_target: float = 90.0  # seconds for reference lap
@export var ai_skill_level: float = 0.85  # 0-1 multiplier for AI performance
@export var player_skill_level: float = 0.75  # 0-1 multiplier for player performance
@export var assist_steering: bool = true
@export var assist_traction_control: bool = true
@export var assist_braking: bool = false
@export var assist_collision: bool = false

func get_torque_at_rpm(rpm: float) -> float:
	"""Calculate engine torque based on RPM using interpolated curve points."""
	var normalized_rpm = clampf(rpm / engine_max_rpm, 0.0, 1.0)
	if normalized_rpm <= 0.0:
		return peak_torque_nm * 0.1
	
	for i in range(1, torque_curve_points.size()):
		if normalized_rpm >= torque_curve_points[i-1].x and normalized_rpm <= torque_curve_points[i].x:
			var t = linear_interpolate(torque_curve_points[i-1].y, torque_curve_points[i].y, 
				(normalized_rpm - torque_curve_points[i-1].x) / (torque_curve_points[i].x - torque_curve_points[i-1].x))
			return peak_torque_nm * t
	
	return peak_torque_nm * 0.75

func calculate_drag_force(speed_ms: float) -> float:
	"""Calculate aerodynamic drag force at given speed."""
	var speed_squared = speed_ms * speed_ms
	return 0.5 * air_density * drag_coefficient * front_area * speed_squared

func calculate_downforce(speed_ms: float, ground_clearance: float = 0.1) -> float:
	"""Calculate total downforce considering wing and ground effect."""
	var speed_squared = speed_ms * speed_ms
	var base_downforce = 0.5 * air_density * wing_downforce_coefficient * front_area * speed_squared
	var ground_effect_bonus = ground_effect_coefficient if ground_clearance < downforce_falloff_distance else 0.0
	return base_downforce + ground_effect_bonus

func calculate_brake_force(pedal_pressure: float, speed_ms: float) -> Dictionary:
	"""Calculate brake forces for front and rear axles."""
	var pressure_bar = pedal_pressure * max_brake_pressure_bar
	var normal_force_total = default_vehicle_mass * gravity
	var normal_force_front = normal_force_total * brake_force_distribution_front
	var normal_force_rear = normal_force_total * brake_force_distribution_rear
	
	var friction_front = normal_force_front * brake_pad_friction
	var friction_rear = normal_force_rear * brake_pad_friction
	
	var total_brake_force = (friction_front * brake_force_distribution_front + friction_rear * brake_force_distribution_rear) * pedal_pressure
	return {
		"total_force": total_brake_force,
		"front_force": friction_front * brake_force_distribution_front * pedal_pressure,
		"rear_force": friction_rear * brake_force_distribution_rear * pedal_pressure
	}

func convert_to_game_units(value: float, unit_type: String) -> float:
	"""Convert physical units to game scale units."""
	match unit_type:
		"length_meters": return value * 100.0  # Game uses cm
		"mass_kg": return value / 100.0  # Game uses hg
		"velocity_ms": return value * 10.0  # Game uses cm/s
		"acceleration_ms2": return value * 100.0  # Game uses cm/s^2
		"pressure_bar": return value * 10.0  # Game uses kPa
		"angle_degrees": return value * 10.0  # Game uses dec-degrees
		"torque_nm": return value * 10.0  # Game uses dNm
		"force_newtons": return value / 10.0  # Game uses daN
		"power_kw": return value * 100.0  # Game uses W
		_: return value

func convert_from_game_units(value: float, unit_type: String) -> float:
	"""Convert game scale units to physical units."""
	match unit_type:
		"length_meters": return value / 100.0
		"mass_kg": return value * 100.0
		"velocity_ms": return value / 10.0
		"acceleration_ms2": return value / 100.0
		"pressure_bar": return value / 10.0
		"angle_degrees": return value / 10.0
		"torque_nm": return value / 10.0
		"force_newtons": return value * 10.0
		"power_kw": return value / 100.0
		_: return value

func get_gear_ratio(gear_index: int) -> float:
	"""Get gear ratio for a specific gear index."""
	if gear_index == -1:  # Reverse
		return -reverse_gear_ratio * final_drive_ratio
	elif gear_index >= gear_ratios.size():  # Overdrive/cruise
		return gear_ratios[-1] * final_drive_ratio
	else:
		return gear_ratios[gear_index] * final_drive_ratio

func get_optimal_shift_point(current_rpm: float) -> float:
	"""Calculate optimal RPM for upshifting based on torque curve."""
	var torque_at_current = get_torque_at_rpm(current_rpm)
	var next_gear_ratio = get_gear_ratio(current_rpm > 0 ? current_rpm : 0)
	
	for rpm in range(engine_idle_rpm, engine_max_rpm, 100):
		var next_gear_torque = get_torque_at_rpm(rpm / next_gear_ratio)
		if next_gear_torque >= torque_at_current:
			return min(rpm, engine_max_rpm)
	
	return engine_max_rpm * 0.9

func calculate_camber_gain(suspension_travel: float) -> float:
	"""Calculate camber change based on suspension travel."""
	var compression = suspension_travel > 0
	var travel_ratio = abs(suspension_travel) / max_suspension_travel
	return (compression ? -1.0 : 1.0) * 0.01 * travel_ratio  # radians

func calculate_ackermann_correction(left_wheel_angle: float, right_wheel_angle: float, track_width: float) -> Vector2:
	"""Calculate Ackermann steering correction for inner/outer wheels."""
	var inner_angle = min(abs(left_wheel_angle), abs(right_wheel_angle))
	var outer_angle = max(abs(left_wheel_angle), abs(right_wheel_angle))
	var ackermann_factor = tan(inner_angle) / tan(outer_angle)
	var ideal_inner = atan(track_width / (wheelbase * ackermann_factor))
	return Vector2(deg_to_rad(left_wheel_angle), deg_to_rad(right_wheel_angle))

func reset_to_defaults() -> void:
	"""Reset all settings to factory defaults."""
	gravity = 9.81
	default_vehicle_mass = 1500.0
	default_wheel_radius = 0.33
	suspension_stiffness = 35000.0
	tire_friction_horizontal = 1.2
	drift_multiplier = 1.5
	drag_coefficient = 0.32
	engine_max_rpm = 8500.0
	gear_ratios = [3.8, 2.4, 1.7, 1.3, 1.0, 0.85]
	brake_force_distribution_front = 0.6
	steering_ratio = 14.0
	wheelbase = 2.70
	cg_height = 0.55
	lap_time_target = 90.0
	ai_skill_level = 0.85
	player_skill_level = 0.75

func validate_settings() -> Error:
	"""Validate all physics settings for consistency."""
	var errors: PackedStringArray = []
	
	if default_vehicle_mass <= 0:
		errors.append("default_vehicle_mass must be positive")
	if default_wheel_radius <= 0:
		errors.append("default_wheel_radius must be positive")
	if gravity <= 0:
		errors.append("gravity must be positive")
	if suspension_stiffness <= 0:
		errors.append("suspension_stiffness must be positive")
	if engine_max_rpm <= 0:
		errors.append("engine_max_rpm must be positive")
	if engine_idle_rpm >= engine_max_rpm:
		errors.append("engine_idle_rpm must be less than engine_max_rpm")
	if abs(brake_force_distribution_front - 1.0) < 0.001 and abs(brake_force_distribution_rear - 1.0) < 0.001:
		errors.append("brake_force_distribution sum should equal 1.0")
	if len(gear_ratios) < 1:
		errors.append("gear_ratios array cannot be empty")
	
	if errors.is_empty():
		return OK
	else:
		push_warning("PhysicsSettings validation failed:")
		for error in errors:
			push_warning("  - %s" % error)
		return ERR_INVALID_DATA