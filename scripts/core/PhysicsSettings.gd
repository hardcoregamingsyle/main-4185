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

@export_group("Engine Physics")
@export var idle_rpm: float = 800.0
@export var redline_rpm: float = 7000.0
@export var peak_torque_rpm: float = 4000.0
@export var peak_horsepower_rpm: float = 6000.0
@export var throttle_response_time: float = 0.15  # seconds
@export var engine_braking_factor: float = 0.3

@export_group("Transmission")
@export var gear_ratios: PackedFloat32Array = [4.0, 2.5, 1.8, 1.3, 1.0, 0.8]
@export var final_drive_ratio: float = 3.5
@export var clutch_engagement_time: float = 0.1
@export var automatic_shift_rpm: float = 6500.0
@export var automatic_downshift_rpm: float = 3000.0

@export_group("Braking System")
@export var brake_force_per_wheel: float = 8000.0
@export var abs_threshold: float = 0.85
@export var brake_bias_front: float = 0.6
@export var brake_bleed_rate: float = 0.98
@export var parking_brake_force: float = 4000.0

@export_group("Chassis Properties")
@export var center_of_mass_x: float = 0.5  # normalized from front bumper
@export var center_of_mass_y: float = 0.35  # height above ground
@export var center_of_mass_z: float = 0.0
@export var chassis_width: float = 1.85  # meters
@export var chassis_length: float = 4.5  # meters
@export var chassis_height: float = 1.4  # meters

@export_group("Steering System")
@export var steering_lock_angle: float = 30.0  # degrees
@export var steering_ratio: float = 14.0  # steering ratio
@export var steering_deadzone: float = 0.02
@export var steering_speed: float = 2.5  # radians per second

@export_group("Differential Settings")
@export var differential_type: int = 0  # 0=open, 1:limited_slip, 2:differential_locked
@export var limited_slip_diff_limit: float = 2.5  # torque multiplier for LSD
@export var open_diff_lock_ratio: float = 0.15  # lock percentage for open diff
@export var differential_efficiency: float = 0.95  # efficiency factor

@export_group("Pacejka Tire Model")
@export var pacejka_a: float = 1.85
@export var pacejka_b: float = 6.5
@export var pacejka_c: float = 1.15
@export var pacejka_d: float = 1.1
@export var pacejka_e: float = -0.25
@export var camber_stiffness_factor: float = 0.5
@export var vertical_load_variation: float = 0.15

@export_group("Roll Dynamics")
@export var roll_bar_stiffness: float = 5000.0  # N/rad
@export var anti_roll_bar_rate: float = 25000.0  # N/rad per wheel
@export var body_roll_max: float = 4.0  # degrees
@export var roll_rate_front: float = 0.55
@export var roll_rate_rear: float = 0.45

@export_group("Power Output")
@export var peak_torque: float = 450.0  # Nm
@export var peak_power: float = 350.0  # kW
@export var power_curve_shape: float = 0.7  # curve shape factor
@export var torque_curve_shape: float = 0.8  # torque curve shape factor
@export var rev_limit_fuel_cut: bool = true
@export var fuel_consumption_rate: float = 0.0001  # liters per joule

@export_group("Track & Environment")
@export var track_gravel_friction: float = 0.5
@export var track_grass_friction: float = 0.3
@export var track_ice_friction: float = 0.15
@export var track_water_friction: float = 0.1
@export var wind_gust_probability: float = 0.05
@export var max_wind_speed: float = 15.0  # m/s
@export var weather_friction_modifier: float = 0.7

# ============================================================================
# Helper Methods
# ============================================================================

func get_current_gear_ratio(current_gear: int) -> float:
	"""Get the gear ratio for a specific gear (0-indexed)"""
	if current_gear < 0 or current_gear >= gear_ratios.size():
		return gear_ratios[current_gear] if current_gear >= 0 else gear_ratios[0]
	return gear_ratios[current_gear] * final_drive_ratio

func calculate_engine_torque(rpm: float) -> float:
	"""Calculate engine torque based on RPM using a simplified curve"""
	var normalized_rpm = rpm / redline_rpm
	
	if normalized_rpm <= 0.0:
		return peak_torque * 0.1
	
	if normalized_rpm >= 1.0:
		if rev_limit_fuel_cut:
			return 0.0
		return peak_torque * 0.3
	
	var torque_factor = _calculate_torque_curve(normalized_rpm)
	return peak_torque * torque_factor

func calculate_engine_power(rpm: float) -> float:
	"""Calculate engine power (kW) based on RPM"""
	var torque = calculate_engine_torque(rpm)
	var power_kw = (torque * rpm * 2.0 * PI) / 60000.0
	return min(power_kw, peak_power)

func calculate_drag_force(velocity: Vector3, speed_kmh: float) -> Vector3:
	"""Calculate aerodynamic drag force"""
	var speed = velocity.length()
	
	if speed == 0.0:
		return Vector3.ZERO
	
	var drag_force_magnitude = 0.5 * air_density * drag_coefficient * front_area * speed * speed
	
	var direction = -velocity.normalized()
	return direction * drag_force_magnitude

func calculate_lift_force(velocity: Vector3, speed_kmh: float) -> Vector3:
	"""Calculate aerodynamic lift/downforce"""
	var speed = velocity.length()
	
	if speed == 0.0:
		return Vector3.ZERO
	
	var lift_force_magnitude = 0.5 * air_density * lift_coefficient * front_area * speed * speed
	
	return Vector3.UP * (-lift_force_magnitude)

func calculate_cornering_stiffness(vertical_load: float, slip_angle: float) -> float:
	"""Calculate cornering stiffness based on Pacejka parameters"""
	var alpha_rad = deg_to_rad(slip_angle)
	
	var B = pacejka_b
	var C = pacejka_c
	var D = pacejka_d * vertical_load / 6000.0
	var E = pacejka_e
	
	var stiff = (B * C * D) * exp(-E * (C * alpha_rad - atan(C * alpha_rad)))
	return stiff

func calculate_longitudinal_force(vertical_load: float, slip_ratio: float) -> float:
	"""Calculate longitudinal force based on slip ratio"""
	var B = pacejka_b
	var C = pacejka_c
	var D = pacejka_d * vertical_load / 6000.0
	var E = pacejka_e
	
	var friction_circle = 1.0 - (abs(slip_ratio) / slip_threshold)
	friction_circle = max(friction_circle, 0.0)
	
	var Fx = (B * C * D) * sin(C * atan(B * slip_ratio)) * friction_circle
	return Fx

func calculate_steering_angle(input_value: float) -> float:
	"""Convert steering input to actual steering angle"""
	input_value = clamp(input_value, -1.0, 1.0)
	
	if abs(input_value) < steering_deadzone:
		return 0.0
	
	var sign = 1.0 if input_value > 0 else -1.0
	var magnitude = abs(input_value)
	
	var target_angle = steering_lock_angle * magnitude * sign
	return target_angle

func convert_to_imperial(unit: String, value: float) -> Dictionary:
	"""Convert metric units to imperial/imperial equivalents"""
	match unit:
		"mass":
			return {"value": value * 2.20462, "unit": "lbs"}
		"length":
			return {"value": value * 39.3701, "unit": "inches"}
		"area":
			return {"value": value * 10.7639, "unit": "sq_ft"}
		"volume":
			return {"value": value * 264.172, "unit": "gallons"}
		"speed":
			return {"value": value * 2.23694, "unit": "mph"}
		"torque":
			return {"value": value * 0.737562, "unit": "lb-ft"}
		"power":
			return {"value": value * 1.34102, "unit": "hp"}
		"pressure":
			return {"value": value * 0.145038, "unit": "psi"}
		"temperature":
			return {"value": (value * 9.0 / 5.0) + 32.0, "unit": "fahrenheit"}
		_:
			return {"value": value, "unit": unit}
	return {"value": value, "unit": unit}

func convert_to_metric(value: float, unit: String) -> Dictionary:
	"""Convert imperial units to metric"""
	match unit:
		"lbs":
			return {"value": value / 2.20462, "unit": "kg"}
		"inches":
			return {"value": value / 39.3701, "unit": "meters"}
		"sq_ft":
			return {"value": value / 10.7639, "unit": "sq_m"}
		"gallons":
			return {"value": value / 264.172, "unit": "liters"}
		"mph":
			return {"value": value / 2.23694, "unit": "km/h"}
		"lb-ft":
			return {"value": value / 0.737562, "unit": "Nm"}
		"hp":
			return {"value": value / 1.34102, "unit": "kW"}
		"psi":
			return {"value": value / 0.145038, "unit": "bar"}
		"fahrenheit":
			return {"value": (value - 32.0) * 5.0 / 9.0, "unit": "celsius"}
		_:
			return {"value": value, "unit": unit}
	return {"value": value, "unit": unit}

func _calculate_torque_curve(normalized_rpm: float) -> float:
	"""Internal helper for torque curve calculation"""
	var base_curve = pow(normalized_rpm, torque_curve_shape)
	var decline_factor = 1.0 - (normalized_rpm - 0.5) * 0.5
	return base_curve * decline_factor

func _calculate_power_curve(normalized_rpm: float) -> float:
	"""Internal helper for power curve calculation"""
	var torque = _calculate_torque_curve(normalized_rpm)
	return torque * normalized_rpm

func get_suspation_force(compression_velocity: float, displacement: float) -> float:
	"""Calculate total suspension force (spring + damper)"""
	var spring_force = -suspension_stiffness * displacement
	var damping_force = -suspension_compression_damping if compression_velocity < 0 else -suspension_rebound_damping * compression_velocity
	return spring_force + damping_force

func is_within_traction_circle(longitudinal_slip: float, lateral_slip: float, mu: float) -> bool:
	"""Check if combined slip is within traction limits"""
	var slip_magnitude = sqrt(pow(longitudinal_slip, 2) + pow(lateral_slip, 2))
	return slip_magnitude <= mu * slip_threshold

func get_brake_distribution(front_weight_ratio: float) -> Dictionary:
	"""Calculate front/rear brake force distribution"""
	var front_ratio = brake_bias_front
	var rear_ratio = 1.0 - front_ratio
	
	return {
		"front": front_ratio * brake_force_per_wheel,
		"rear": rear_ratio * brake_force_per_wheel,
		"total": brake_force_per_wheel
	}

func calculate_g_force(acceleration: Vector3) -> Vector3:
	"""Convert acceleration to G-forces"""
	return acceleration / gravity

func get_optimal_shift_point(current_gear: int, rpm: float) -> int:
	"""Calculate optimal shift point for manual transmission"""
	var next_gear_ratio = gear_ratios[min(current_gear + 1, gear_ratios.size() - 1)]
	var current_gear_ratio = gear_ratios[current_gear]
	
	var ideal_rpm_after_shift = 6000.0
	var new_rpm = rpm * next_gear_ratio / current_gear_ratio
	
	if new_rpm < idle_rpm:
		return current_gear
	
	return current_gear + 1 if new_rpm <= automatic_shift_rpm else current_gear

func reset_to_defaults() -> void:
	"""Reset all settings to factory defaults"""
	gravity = 9.81
	physics_tick_rate = 120
	max_substeps = 4
	default_vehicle_mass = 1500.0
	default_wheel_radius = 0.33
	default_wheel_width = 0.215
	default_wheel_mass = 25.0
	default_wheel_inertia = 1.2
	suspension_stiffness = 35000.0
	suspension_compression_damping = 1200.0
	suspension_rebound_damping = 600.0
	max_suspension_travel = 0.25
	spring_rest_length = 0.35
	tire_friction_horizontal = 1.2
	tire_friction_vertical = 0.3
	slipp_threshold = 0.15
	lateral_slip_stiffness = 3.5e5
	longitudinal_slip_stiffness = 8.0e5
	drift_multiplier = 1.5
	drift_recovery_speed = 2.0
	minimum_drift_angle = 15.0
	maximum_drift_angle = 45.0
	air_density = 1.225
	drag_coefficient = 0.32
	lift_coefficient = 0.15
	front_area = 2.2
	center_of_pressure_x = 0.5
	center_of_pressure_y = 0.0
	center_of_pressure_z = 0.3
	idle_rpm = 800.0
	redline_rpm = 7000.0
	peak_torque_rpm = 4000.0
	peak_horsepower_rpm = 6000.0
	throttle_response_time = 0.15
	engine_braking_factor = 0.3
	gear_ratios = PackedFloat32Array([4.0, 2.5, 1.8, 1.3, 1.0, 0.8])
	final_drive_ratio = 3.5
	clutch_engagement_time = 0.1
	automatic_shift_rpm = 6500.0
	automatic_downshift_rpm = 3000.0
	brake_force_per_wheel = 8000.0
	abs_threshold = 0.85
	brake_bias_front = 0.6
	brake_bleed_rate = 0.98
	parking_brake_force = 4000.0
	center_of_mass_x = 0.5
	center_of_mass_y = 0.35
	center_of_mass_z = 0.0
	chassis_width = 1.85
	chassis_length = 4.5
	chassis_height = 1.4
	steering_lock_angle = 30.0
	steering_ratio = 14.0
	steering_deadzone = 0.02
	steering_speed = 2.5
	differential_type = 0
	limited_slip_diff_limit = 2.5
	open_diff_lock_ratio = 0.15
	differential_efficiency = 0.95
	pacejka_a = 1.85
	pacejka_b = 6.5
	pacejka_c = 1.15
	pacejka_d = 1.1
	pacejka_e = -0.25
	camber_stiffness_factor = 0.5
	vertical_load_variation = 0.15
	roll_bar_stiffness = 5000.0
	anti_roll_bar_rate = 25000.0
	body_roll_max = 4.0
	roll_rate_front = 0.55
	roll_rate_rear = 0.45
	peak_torque = 450.0
	peak_power = 350.0
	power_curve_shape = 0.7
	torque_curve_shape = 0.8
	rev_limit_fuel_cut = true
	fuel_consumption_rate = 0.0001
	track_gravel_friction = 0.5
	track_grass_friction = 0.3
	track_ice_friction = 0.15
	track_water_friction = 0.1
	wind_gust_probability = 0.05
	max_wind_speed = 15.0
	weather_friction_modifier = 0.7

## Static getter methods for easy access to common values
static func get_default_settings() -> PhysicsSettings:
	return PhysicsSettings.new()

static func get_gravity() -> float:
	return PhysicsSettings.get_default_settings().gravity

static func get_physics_tick_rate() -> int:
	return PhysicsSettings.get_default_settings().physics_tick_rate

static func get_vehicle_mass() -> float:
	return PhysicsSettings.get_default_settings().default_vehicle_mass

static func get_wheel_radius() -> float:
	return PhysicsSettings.get_default_settings().default_wheel_radius

static func get_tire_friction() -> float:
	return PhysicsSettings.get_default_settings().tire_friction_horizontal

static func get_air_density() -> float:
	return PhysicsSettings.get_default_settings().air_density

static func get_drag_coefficient() -> float:
	return PhysicsSettings.get_default_settings().drag_coefficient