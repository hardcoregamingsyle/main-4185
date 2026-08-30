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
@export var center_of_pressure_y: float = 0.0
@export var center_of_pressure_z: float = 0.3

@export_group("Steering System")
@export var steering_max_lock: float = 30.0  # degrees (half lock)
@export var steering_max_lock_radians: float = deg_to_rad(30.0)
@export var steering_speed: float = 45.0  # degrees per second
@export var steering_ratio: float = 14.0  # rack and pinion ratio
@export var steering_effort_curve: Array[Vector2] = []  # [speed_kmh, effort_factor]

@export_group("Brake System")
@export var brake_pressure_max: float = 100.0  # bar
@export var brake_force_distribution_front: float = 0.6  # percentage to front wheels
@export var brake_force_distribution_rear: float = 0.4  # percentage to rear wheels
@export var brake_bias_curve: Array[Vector2] = []  # [speed_kmh, bias_front]
@export var abs_activation_threshold: float = 0.95  # slip ratio threshold
@export var abs_modulation_frequency: float = 15.0  # Hz
@export var parking_brake_force: float = 0.5  # relative to max brake force

@export_group("Engine & Powertrain")
@export var engine_displacement: float = 3.0  # liters
@export var engine_cylinders: int = 6
@export var max_engine_rpm: float = 8500.0  # RPM
@export var idle_rpm: float = 800.0  # RPM
@export var torque_peak_rpm: float = 4500.0  # RPM
@export var power_peak_rpm: float = 7000.0  # RPM
@export var max_torque_nm: float = 400.0  # Nm
@export var max_power_kw: float = 300.0  # kW

@export_group("Torque Curve")
@export var torque_curve_points: Array[Vector2] = []  # [rpm, torque_normalized_0_1]

@export_group("Transmission & Gears")
@export var transmission_type: String = "manual"  # manual, automatic, cvt
@export var final_drive_ratio: float = 3.5
@export var clutch_slip_angle: float = 15.0  # degrees
@export var clutch_engagement_time: float = 0.3  # seconds

@export_group("Gear Ratios")
@export var first_gear_ratio: float = 3.8
@export var second_gear_ratio: float = 2.4
@export var third_gear_ratio: float = 1.7
@export var fourth_gear_ratio: float = 1.3
@export var fifth_gear_ratio: float = 1.0
@export var sixth_gear_ratio: float = 0.85
@export var reverse_gear_ratio: float = -4.0
@export var neutral_ratio: float = 0.0

@export_group("Differential Settings")
@export var diff_type: String = "limited_slip"  # open, locked, limited_slip
@export var diff_preload_torque: float = 50.0  # Nm
@export var diff_limit_torque: float = 200.0  # Nm
@export var diff_locking_speed: float = 2.0  # rad/s threshold

@export_group("Clutch & Rev Matching")
@export var clutch_friction_coefficient: float = 0.4
@export var clutch_contact_area: float = 0.025  # m^2
@export var rev_match_aggression: float = 0.7  # 0.0-1.0 how much to match RPM on downshift

@export_group("Traction Control")
@export var tc_enabled: bool = true
@export var tc_intervention_level: float = 0.6  # 0.0-1.0 throttle intervention
@export var tc_slip_target: float = 0.15  # target wheel slip ratio

@export_group("Stability Control")
@export var esp_enabled: bool = true
@export var esp_yaw_threshold: float = 0.3  # rad/s deviation trigger
@export var esp_brake_pressure: float = 0.5  # relative brake pressure when ESP active

@export_group("Damage & Wear")
@export var tire_wear_rate_normal: float = 0.0001  # wear per meter at normal conditions
@export var tire_wear_rate_drifting: float = 0.0005  # wear per meter while drifting
@export var brake_pad_life_km: float = 50.0  # km before replacement needed
@export var engine_redline_risk: bool = false  # enable damage on redline

@export_group("Weather Effects")
@export var rain_friction_reduction: float = 0.3  # multiplier to friction in rain
@export var snow_friction_reduction: float = 0.5  # multiplier to friction in snow
@export var ice_friction_reduction: float = 0.1  # multiplier to friction on ice
@export var wet_track_recovery_time: float = 30.0  # seconds to dry after rain stops

@export_group("Simulation Accuracy")
@export var use_advanced_suspension: bool = true
@export var use_pacejka_curves: bool = true
@export var simulate_tire_temp: bool = true
@export var simulate_fuel_weight: bool = true
@export var fuel_consumption_per_km: float = 0.15  # liters per km

# ============================================================================
# HELPER METHODS
# ============================================================================

func get_torque_at_rpm(rpm: float) -> float:
	"""Calculate torque output at given RPM using interpolated curve"""
	if torque_curve_points.is_empty():
		# Default bell curve if no custom curve provided
		var rpm_norm = (rpm - idle_rpm) / (max_engine_rpm - idle_rpm)
		rpm_norm = clamp(rpm_norm, 0.0, 1.0)
		return max_torque_nm * (1.0 - pow(rpm_norm - 0.6, 2) / 0.36)
	
	var last_point: Vector2 = torque_curve_points[0]
	for point in torque_curve_points:
		if rpm <= point.x:
			break
		last_point = point
	
	# Linear interpolation between points
	if len(torque_curve_points) < 2:
		return max_torque_nm * last_point.y
	
	var next_point: Vector2 = torque_curve_points[min(len(torque_curve_points) - 1, 
		torque_curve_points.find(last_point) + 1)]
	
	var t = (rpm - last_point.x) / (next_point.x - last_point.x)
	t = clamp(t, 0.0, 1.0)
	
	var normalized_torque = lerp(last_point.y, next_point.y, t)
	return max_torque_nm * normalized_torque

func get_power_at_rpm(rpm: float) -> float:
	"""Calculate power output at given RPM"""
	var torque = get_torque_at_rpm(rpm)
	return (torque * rpm) / 9549.3  # kW formula: (Nm * RPM) / 9549.3

func get_gear_ratio(gear: int) -> float:
	"""Get gear ratio for specified gear number"""
	match gear:
		1: return first_gear_ratio
		2: return second_gear_ratio
		3: return third_gear_ratio
		4: return fourth_gear_ratio
		5: return fifth_gear_ratio
		6: return sixth_gear_ratio
		-1: return reverse_gear_ratio
		0: return neutral_ratio
		_: return first_gear_ratio

func calculate_aero_forces(speed_ms: float) -> Vector3:
	"""Calculate aerodynamic drag and downforce forces"""
	var speed_sq = speed_ms * speed_ms
	var drag_force = 0.5 * air_density * drag_coefficient * front_area * speed_sq
	var lift_force = 0.5 * air_density * lift_coefficient * front_area * speed_sq
	
	# Downforce is negative lift (pushes car down)
	return Vector3(-drag_force, 0.0, -lift_force)

func calculate_brake_force(total_brake_input: float, vehicle_mass: float) -> Dictionary:
	"""Calculate individual wheel brake forces"""
	var total_brake_force = brake_pressure_max * brake_force_distribution_front * total_brake_input
	var front_force = total_brake_force * brake_force_distribution_front
	var rear_force = total_brake_force * brake_force_distribution_rear
	
	return {
		"front_left": front_force,
		"front_right": front_force,
		"rear_left": rear_force,
		"rear_right": rear_force
	}

func convert_kmh_to_ms(kmh: float) -> float:
	"""Convert kilometers per hour to meters per second"""
	return kmh / 3.6

func convert_ms_to_kmh(ms: float) -> float:
	"""Convert meters per second to kilometers per hour"""
	return ms * 3.6

func convert_rpm_to_rps(rpm: float) -> float:
	"""Convert revolutions per minute to revolutions per second"""
	return rpm / 60.0

func convert_rad_to_deg(rad: float) -> float:
	"""Convert radians to degrees"""
	return rad * (180.0 / PI)

func convert_deg_to_rad(deg: float) -> float:
	"""Convert degrees to radians"""
	return deg * (PI / 180.0)

func calculate_spring_force(compression: float, velocity: float) -> float:
	"""Calculate suspension spring force with damping"""
	var spring_force = -suspension_stiffness * compression
	var damping_force = -suspension_compression_damping * velocity if compression < 0 else \
						-suspension_rebound_damping * velocity
	
	return spring_force + damping_force

func calculate_pacejka_lateral_force(normal_force: float, slip_angle: float) -> float:
	"""Calculate lateral force using simplified Pacejka formula"""
	var B = 10.0  # Stiffness factor
	var C = 1.9   # Shape factor
	var D = normal_force * tire_friction_horizontal  # Peak factor
	var E = 0.95  # Curvature factor
	
	var angle_rad = deg_to_rad(slip_angle)
	var horizontal_shift = 0.0
	var vertical_shift = 0.0
	
	return D * sin(C * atan(B * angle_rad - E * (B * angle_rad - atan(B * angle_rad)))) + vertical_shift

func calculate_pacejka_longitudinal_force(normal_force: float, slip_ratio: float) -> float:
	"""Calculate longitudinal force using simplified Pacejka formula"""
	var B = 20.0  # Stiffness factor
	var C = 1.9   # Shape factor
	var D = normal_force * tire_friction_vertical  # Peak factor
	var E = 0.95  # Curvature factor
	
	var horizontal_shift = 0.0
	var vertical_shift = 0.0
	
	return D * sin(C * atan(B * slip_ratio - E * (B * slip_ratio - slip_ratio))) + vertical_shift

func get_optimal_tire_pressure(temp_celsius: float) -> float:
	"""Calculate optimal tire pressure based on temperature (bar)"""
	var base_pressure = 2.2  # bar at 20°C
	var temp_deviation = temp_celsius - 20.0
	return base_pressure + (temp_deviation * 0.01)

func calculate_fuel_weight_remaining(fuel_liters: float) -> float:
	"""Calculate remaining fuel mass (kg)"""
	var fuel_density = 0.75  # kg/L gasoline
	return fuel_liters * fuel_density

func calculate_total_vehicle_mass(fuel_liters: float) -> float:
	"""Calculate total vehicle mass including fuel"""
	var base_mass = default_vehicle_mass
	var fuel_mass = calculate_fuel_weight_remaining(fuel_liters)
	return base_mass + fuel_mass

func should_abs_activate(longitudinal_slip: float) -> bool:
	"""Check if ABS should intervene"""
	return not tc_enabled or longitudinal_slip > abs_activation_threshold

func should_tc_intervene(longitudinal_slip: float) -> bool:
	"""Check if traction control should intervene"""
	return tc_enabled and longitudinal_slip > tc_slip_target

func get_weather_friction_multiplier(weather_type: String) -> float:
	"""Get friction multiplier based on weather conditions"""
	match weather_type:
		"rain": return rain_friction_reduction
		"snow": return snow_friction_reduction
		"ice": return ice_friction_reduction
		"dry": return 1.0
		"wet": return 0.7
		_: return 1.0

func validate_settings() -> bool:
	"""Validate all physics settings for reasonable values"""
	var errors: Array[String] = []
	
	if gravity <= 0.0:
		errors.append("gravity must be positive")
	if physics_tick_rate < 30 or physics_tick_rate > 240:
		errors.append("physics_tick_rate should be between 30 and 240")
	if default_vehicle_mass <= 0.0:
		errors.append("default_vehicle_mass must be positive")
	if default_wheel_radius <= 0.0:
		errors.append("default_wheel_radius must be positive")
	if suspension_stiffness <= 0.0:
		errors.append("suspension_stiffness must be positive")
	if max_suspension_travel <= 0.0:
		errors.append("max_suspension_travel must be positive")
	if tire_friction_horizontal < 0.1 or tire_friction_horizontal > 2.0:
		errors.append("tire_friction_horizontal should be between 0.1 and 2.0")
	if air_density <= 0.0:
		errors.append("air_density must be positive")
	if drag_coefficient < 0.0:
		errors.append("drag_coefficient must be non-negative")
	if max_engine_rpm <= idle_rpm:
		errors.append("max_engine_rpm must be greater than idle_rpm")
	if first_gear_ratio <= 0.0:
		errors.append("first_gear_ratio must be positive")
	
	return errors.size() == 0

func get_validation_errors() -> Array[String]:
	"""Get array of validation error messages"""
	var errors: Array[String] = []
	
	if gravity <= 0.0:
		errors.append("Invalid: gravity must be positive (current: %.2f)" % gravity)
	if physics_tick_rate < 30 or physics_tick_rate > 240:
		errors.append("Invalid: physics_tick_rate should be 30-240 (current: %d)" % physics_tick_rate)
	if default_vehicle_mass <= 0.0:
		errors.append("Invalid: default_vehicle_mass must be positive (current: %.2f)" % default_vehicle_mass)
	if default_wheel_radius <= 0.0:
		errors.append("Invalid: default_wheel_radius must be positive (current: %.3f)" % default_wheel_radius)
	if suspension_stiffness <= 0.0:
		errors.append("Invalid: suspension_stiffness must be positive (current: %.2f)" % suspension_stiffness)
	if max_suspension_travel <= 0.0:
		errors.append("Invalid: max_suspension_travel must be positive (current: %.3f)" % max_suspension_travel)
	if tire_friction_horizontal < 0.1 or tire_friction_horizontal > 2.0:
		errors.append("Invalid: tire_friction_horizontal should be 0.1-2.0 (current: %.2f)" % tire_friction_horizontal)
	if air_density <= 0.0:
		errors.append("Invalid: air_density must be positive (current: %.3f)" % air_density)
	if drag_coefficient < 0.0:
		errors.append("Invalid: drag_coefficient must be non-negative (current: %.2f)" % drag_coefficient)
	if max_engine_rpm <= idle_rpm:
		errors.append("Invalid: max_engine_rpm must exceed idle_rpm (max: %.0f, idle: %.0f)" % [max_engine_rpm, idle_rpm])
	if first_gear_ratio <= 0.0:
		errors.append("Invalid: first_gear_ratio must be positive (current: %.2f)" % first_gear_ratio)
	
	return errors

func clone_with_scale(scale_factor: float = 1.0) -> PhysicsSettings:
	"""Create a scaled copy of these settings (for different track types)"""
	var new_settings = PhysicsSettings.new()
	new_settings.gravity = gravity
	new_settings.physics_tick_rate = physics_tick_rate
	new_settings.max_substeps = max_substeps
	new_settings.default_vehicle_mass = default_vehicle_mass * scale_factor
	new_settings.default_wheel_radius = default_wheel_radius
	new_settings.default_wheel_width = default_wheel_width
	new_settings.default_wheel_mass = default_wheel_mass
	new_settings.default_wheel_inertia = default_wheel_inertia
	new_settings.suspension_stiffness = suspension_stiffness * scale_factor
	new_settings.suspension_compression_damping = suspension_compression_damping * scale_factor
	new_settings.suspension_rebound_damping = suspension_rebound_damping * scale_factor
	new_settings.max_suspension_travel = max_suspension_travel
	new_settings.spring_rest_length = spring_rest_length
	new_settings.tire_friction_horizontal = tire_friction_horizontal
	new_settings.tire_friction_vertical = tire_friction_vertical
	new_settings.slip_threshold = slip_threshold
	new_settings.lateral_slip_stiffness = lateral_slip_stiffness
	new_settings.longitudinal_slip_stiffness = longitudinal_slip_stiffness
	new_settings.drift_multiplier = drift_multiplier
	new_settings.drift_recovery_speed = drift_recovery_speed
	new_settings.minimum_drift_angle = minimum_drift_angle
	new_settings.maximum_drift_angle = maximum_drift_angle
	new_settings.air_density = air_density
	new_settings.drag_coefficient = drag_coefficient
	new_settings.lift_coefficient = lift_coefficient
	new_settings.front_area = front_area
	new_settings.center_of_pressure_x = center_of_pressure_x
	new_settings.center_of_pressure_y = center_of_pressure_y
	new_settings.center_of_pressure_z = center_of_pressure_z
	new_settings.steering_max_lock = steering_max_lock
	new_settings.steering_max_lock_radians = steering_max_lock_radians
	new_settings.steering_speed = steering_speed
	new_settings.steering_ratio = steering_ratio
	new_settings.brake_pressure_max = brake_pressure_max
	new_settings.brake_force_distribution_front = brake_force_distribution_front
	new_settings.brake_force_distribution_rear = brake_force_distribution_rear
	new_settings.abs_activation_threshold = abs_activation_threshold
	new_settings.abs_modulation_frequency = abs_modulation_frequency
	new_settings.parking_brake_force = parking_brake_force
	new_settings.engine_displacement = engine_displacement
	new_settings.engine_cylinders = engine_cylinders
	new_settings.max_engine_rpm = max_engine_rpm
	new_settings.idle_rpm = idle_rpm
	new_settings.torque_peak_rpm = torque_peak_rpm
	new_settings.power_peak_rpm = power_peak_rpm
	new_settings.max_torque_nm = max_torque_nm * scale_factor
	new_settings.max_power_kw = max_power_kw * scale_factor
	new_settings.transmission_type = transmission_type
	new_settings.final_drive_ratio = final_drive_ratio
	new_settings.clutch_slip_angle = clutch_slip_angle
	new_settings.clutch_engagement_time = clutch_engagement_time
	new_settings.first_gear_ratio = first_gear_ratio
	new_settings.second_gear_ratio = second_gear_ratio
	new_settings.third_gear_ratio = third_gear_ratio
	new_settings.fourth_gear_ratio = fourth_gear_ratio
	new_settings.fifth_gear_ratio = fifth_gear_ratio
	new_settings.sixth_gear_ratio = sixth_gear_ratio
	new_settings.reverse_gear_ratio = reverse_gear_ratio
	new_settings.neutral_ratio = neutral_ratio
	new_settings.diff_type = diff_type
	new_settings.diff_preload_torque = diff_preload_torque
	new_settings.diff_limit_torque = diff_limit_torque
	new_settings.diff_locking_speed = diff_locking_speed
	new_settings.clutch_friction_coefficient = clutch_friction_coefficient
	new_settings.clutch_contact_area = clutch_contact_area
	new_settings.rev_match_aggression = rev_match_aggression
	new_settings.tc_enabled = tc_enabled
	new_settings.tc_intervention_level = tc_intervention_level
	new_settings.tc_slip_target = tc_slip_target
	new_settings.esp_enabled = esp_enabled
	new_settings.esp_yaw_threshold = esp_yaw_threshold
	new_settings.esp_brake_pressure = esp_brake_pressure
	new_settings.tire_wear_rate_normal = tire_wear_rate_normal
	new_settings.tire_wear_rate_drifting = tire_wear_rate_drifting
	new_settings.brake_pad_life_km = brake_pad_life_km
	new_settings.engine_redline_risk = engine_redline_risk
	new_settings.rain_friction_reduction = rain_friction_reduction
	new_settings.snow_friction_reduction = snow_friction_reduction
	new_settings.ice_friction_reduction = ice_friction_reduction
	new_settings.wet_track_recovery_time = wet_track_recovery_time
	new_settings.use_advanced_suspension = use_advanced_suspension
	new_settings.use_pacejka_curves = use_pacejka_curves
	new_settings.simulate_tire_temp = simulate_tire_temp
	new_settings.simulate_fuel_weight = simulate_fuel_weight
	new_settings.fuel_consumption_per_km = fuel_consumption_per_km
	
	return new_settings

func _validate_torque_curve() -> void:
	"""Ensure torque curve is valid and has enough points"""
	if torque_curve_points.is_empty():
		# Generate default bell curve
		for rpm in range(int(idle_rpm), int(max_engine_rpm), int((max_engine_rpm - idle_rpm) / 10)):
			var rpm_norm = (float(rpm) - idle_rpm) / (max_engine_rpm - idle_rpm)
			rpm_norm = clamp(rpm_norm, 0.0, 1.0)
			var torque = 1.0 - pow(rpm_norm - 0.6, 2) / 0.36
			torque = clamp(torque, 0.1, 1.0)
			torque_curve_points.append(Vector2(float(rpm), torque))
	
	# Ensure we have endpoints
	var has_idle = false
	var has_max = false
	for point in torque_curve_points:
		if abs(point.x - idle_rpm) < 100.0:
			has_idle = true
		if abs(point.x - max_engine_rpm) < 100.0:
			has_max = true
	
	if not has_idle:
		torque_curve_points.insert(0, Vector2(float(idle_rpm), 0.4))
	if not has_max:
		torque_curve_points.append(Vector2(float(max_engine_rpm), 0.3))

func _ready() -> void:
	_init_default_curves()

func _init_default_curves() -> void:
	"""Initialize default torque and steering curves if empty"""
	_validate_torque_curve()
	
	# Default steering effort curve (increases with speed)
	if steering_effort_curve.is_empty():
		steering_effort_curve.append(Vector2(0.0, 1.0))       # Low speed = light steering
		steering_effort_curve.append(Vector2(50.0, 1.2))      # Medium speed
		steering_effort_curve.append(Vector2(100.0, 1.5))     # High speed
		steering_effort_curve.append(Vector2(200.0, 2.0))     # Very high speed
	
	# Default brake bias curve (more front bias at higher speeds)
	if brake_bias_curve.is_empty():
		brake_bias_curve.append(Vector2(0.0, 0.55))          # Low speed
		brake_bias_curve.append(Vector2(100.0, 0.60))        # Medium speed
		brake_bias_curve.append(Vector2(200.0, 0.65))        # High speed
		brake_bias_curve.append(Vector2(300.0, 0.70))        # Very high speed

func set_steering_effort_curve(points: Array[Vector2]) -> void:
	"""Set custom steering effort curve"""
	steering_effort_curve = points.duplicate()

func set_brake_bias_curve(points: Array[Vector2]) -> void:
	"""Set custom brake bias curve"""
	brake_bias_curve = points.duplicate()

func get_steering_effort(speed_kmh: float) -> float:
	"""Get steering effort multiplier at given speed"""
	if steering_effort_curve.is_empty():
		return 1.0
	
	var last_point: Vector2 = steering_effort_curve[0]
	for point in steering_effort_curve:
		if speed_kmh <= point.x:
			break
		last_point = point
	
	if len(steering_effort_curve) < 2:
		return last_point.y
	
	var next_point: Vector2 = steering_effort_curve[min(len(steering_effort_curve) - 1, 
		steering_effort_curve.find(last_point) + 1)]
	
	var t = (speed_kmh - last_point.x) / (next_point.x - last_point.x)
	t = clamp(t, 0.0, 1.0)
	
	return lerp(last_point.y, next_point.y, t)

func get_brake_bias(speed_kmh: float) -> float:
	"""Get front brake bias at given speed"""
	if brake_bias_curve.is_empty():
		return brake_force_distribution_front
	
	var last_point: Vector2 = brake_bias_curve[0]
	for point in brake_bias_curve:
		if speed_kmh <= point.x:
			break
		last_point = point
	
	if len(brake_bias_curve) < 2:
		return last_point.y
	
	var next_point: Vector2 = brake_bias_curve[min(len(brake_bias_curve) - 1, 
		brake_bias_curve.find(last_point) + 1)]
	
	var t = (speed_kmh - last_point.x) / (next_point.x - last_point.x)
	t = clamp(t, 0.0, 1.0)
	
	return lerp(last_point.y, next_point.y, t)

func save_to_file(path: String) -> void:
	"""Save physics settings to a resource file"""
	var error = ResourceSaver.save(self, path)
	if error != OK:
		push_error("Failed to save PhysicsSettings: %s" % Error.get_string(error))

func load_from_file(path: String) -> void:
	"""Load physics settings from a resource file"""
	var loaded = load(path) as PhysicsSettings
	if loaded:
		copy_from(loaded)
	else:
		push_error("Failed to load PhysicsSettings from: %s" % path)

func copy_from(other: PhysicsSettings) -> void:
	"""Copy all values from another PhysicsSettings instance"""
	gravity = other.gravity
	physics_tick_rate = other.physics_tick_rate
	max_substeps = other.max_substeps
	default_vehicle_mass = other.default_vehicle_mass
	default_wheel_radius = other.default_wheel_radius
	default_wheel_width = other.default_wheel_width
	default_wheel_mass = other.default_wheel_mass
	default_wheel_inertia = other.default_wheel_inertia
	suspension_stiffness = other.suspension_stiffness
	suspension_compression_damping = other.suspension_compression_damping
	suspension_rebound_damping = other.suspension_rebound_damping
	max_suspension_travel = other.max_suspension_travel
	spring_rest_length = other.spring_rest_length
	tire_friction_horizontal = other.tire_friction_horizontal
	tire_friction_vertical = other.tire_friction_vertical
	slipping_threshold = other.slip_threshold
	lateral_slip_stiffness = other.lateral_slip_stiffness
	longitudinal_slip_stiffness = other.longitudinal_slip_stiffness
	drift_multiplier = other.drift_multiplier
	drift_recovery_speed = other.drift_recovery_speed
	minimum_drift_angle = other.minimum_drift_angle
	maximum_drift_angle = other.maximum_drift_angle
	air_density = other.air_density
	drag_coefficient = other.drag_coefficient
	lift_coefficient = other.lift_coefficient
	front_area = other.front_area
	center_of_pressure_x = other.center_of_pressure_x
	center_of_pressure_y = other.center_of_pressure_y
	center_of_pressure_z = other.center_of_pressure_z
	steering_max_lock = other.steering_max_lock
	steering_max_lock_radians = other.steering_max_lock_radians
	steering_speed = other.steering_speed
	steering_ratio = other.steering_ratio
	brake_pressure_max = other.brake_pressure_max
	brake_force_distribution_front = other.brake_force_distribution_front
	brake_force_distribution_rear = other.brake_force_distribution_rear
	abs_activation_threshold = other.abs_activation_threshold
	abs_modulation_frequency = other.abs_modulation_frequency
	parking_brake_force = other.parking_brake_force
	engine_displacement = other.engine_displacement
	engine_cylinders = other.engine_cylinders
	max_engine_rpm = other.max_engine_rpm
	idle_rpm = other.idle_rpm
	torque_peak_rpm = other.torque_peak_rpm
	power_peak_rpm = other.power_peak_rpm
	max_torque_nm = other.max_torque_nm
	max_power_kw = other.max_power_kw
	transmission_type = other.transmission_type
	final_drive_ratio = other.final_drive_ratio
	clutch_slip_angle = other.clutch_slip_angle
	clutch_engagement_time = other.clutch_engagement_time
	first_gear_ratio = other.first_gear_ratio
	second_gear_ratio = other.second_gear_ratio
	third_gear_ratio = other.third_gear_ratio
	fourth_gear_ratio = other.fourth_gear_ratio
	fifth_gear_ratio = other.fifth_gear_ratio
	sixth_gear_ratio = other.sixth_gear_ratio
	reverse_gear_ratio = other.reverse_gear_ratio
	neutral_ratio = other.neutral_ratio
	diff_type = other.diff_type
	diff_preload_torque = other.diff_preload_torque
	diff_limit_torque = other.diff_limit_torque
	diff_locking_speed = other.diff_locking_speed
	clutch_friction_coefficient = other.clutch_friction_coefficient
	clutch_contact_area = other.clutch_contact_area
	rev_match_aggression = other.rev_match_aggression
	tc_enabled = other.tc_enabled
	tc_intervention_level = other.tc_intervention_level
	tc_slip_target = other.tc_slip_target
	esp_enabled = other.esp_enabled
	esp_yaw_threshold = other.esp_yaw_threshold
	esp_brake_pressure = other.esp_brake_pressure
	tire_wear_rate_normal = other.tire_wear_rate_normal
	tire_wear_rate_drifting = other.tire_wear_rate_drifting
	brake_pad_life_km = other.brake_pad_life_km
	engine_redline_risk = other.engine_redline_risk
	rain_friction_reduction = other.rain_friction_reduction
	snow_friction_reduction = other.snow_friction_reduction
	ice_friction_reduction = other.ice_friction_reduction
	wet_track_recovery_time = other.wet_track_recovery_time
	use_advanced_suspension = other.use_advanced_suspension
	use_pacejka_curves = other.use_pacejka_curves
	simulate_tire_temp = other.simulate_tire_temp
	simulate_fuel_weight = other.simulate_fuel_weight
	fuel_consumption_per_km = other.fuel_consumption_per_km
	torque_curve_points = other.torque_curve_points.duplicate()
	steering_effort_curve = other.steering_effort_curve.duplicate()
	brake_bias_curve = other.brake_bias_curve.duplicate()

func get_summary() -> Dictionary:
	"""Return a summary dictionary of key physics parameters"""
	return {
		"vehicle_mass": default_vehicle_mass,
		"wheel_radius": default_wheel_radius,
		"engine_max_rpm": max_engine_rpm,
		"engine_max_torque": max_torque_nm,
		"engine_max_power": max_power_kw,
		"gears": [first_gear_ratio, second_gear_ratio, third_gear_ratio, 
		         fourth_gear_ratio, fifth_gear_ratio, sixth_gear_ratio],
		"differential": diff_type,
		"transmission": transmission_type,
		"aero_drag": drag_coefficient,
		"aero_lift": lift_coefficient,
		"tire_friction": tire_friction_horizontal,
		"brake_bias_front": brake_force_distribution_front,
		"abs_enabled": abs_activation_threshold < 1.0,
		"tc_enabled": tc_enabled,
		"esp_enabled": esp_enabled
	}