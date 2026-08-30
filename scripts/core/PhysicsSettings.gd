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

@export_group("Gear Ratios")
@export var gear_ratios: Array[float] = [3.8, 2.4, 1.6, 1.2, 0.9, 0.7]  # 6-speed transmission
@export var final_drive_ratio: float = 3.73
@export var neutral_ratio: float = 0.0

@export_group("Differential Settings")
@export var diff_type: DiffType = DiffType.LSD
@export var diff_lock_front: float = 0.0  # FWD bias
@export var diff_lock_rear: float = 0.45  # LSD lock percentage
@export var diff_acceleration_bias: float = 0.65  # torque bias during acceleration
@export var diff_deceleration_bias: float = 0.35  # torque bias during deceleration

enum DiffType {
	OPEN,      # Open differential - equal torque split
	LSD,       # Limited Slip Differential
	LOCKED,    # Locked differential - locked together
	ELECTRONIC # Electronic active differential
}

@export_group("Brake System")
@export var brake_force_per_pedal: float = 10000.0  # N per brake pedal
@export var brake_disc_diameter: float = 0.32  # meters
@export var brake_disc_thickness: float = 0.025  # meters
@export var brake_caliper_piston_count: int = 4
@export var brake_pad_friction_coefficient: float = 0.4
@export var brake_temp_max: float = 900.0  # Celsius
@export var brake_fade_factor: float = 0.001  # friction loss per degree C above optimal
@export var optimal_brake_temp: float = 350.0  # Celsius
@export var brake_bias_front: float = 0.60  # percentage to front brakes

@export_group("Steering System")
@export var steering_ratio: float = 14.0  # ratio of steering wheel angle to wheel angle
@export var steering_lock_angle: float = 2.5  # radians (max wheel turn)
@export var steering_speed: float = 4.0  # radians per second max steering speed
@export var steering_return_speed: float = 2.0  # radians per second return speed
@export var power_assistance_ratio: float = 0.5  # reduction in steering effort
@export var steering_weight_curve: Array[float] = [0.3, 0.4, 0.6, 0.8, 1.0]  # weight vs speed normalized

@export_group("Chassis Dynamics")
@export var chassis_stiffness: float = 15000.0  # Nm/rad torsional stiffness
@export var cg_height: float = 0.55  # Center of gravity height in meters
@export var cg_distance_from_front_axle: float = 1.4  # meters
@export var track_width_front: float = 1.65  # meters
@export var track_width_rear: float = 1.62  # meters
@export var wheelbase: float = 2.7  # meters
@export var roll_center_height_front: float = 0.15  # meters
@export var roll_center_height_rear: float = 0.18  # meters
@export var anti_roll_bar_stiffness_front: float = 8000.0  # Nm/rad
@export var anti_roll_bar_stiffness_rear: float = 6000.0  # Nm/rad

@export_group("Collision & Materials")
@export var chassis_bounce_factor: float = 0.3
@export var chassis_friction_ground: float = 0.7
@export var chassis_friction_air: float = 0.05
@export var obstacle_collision_strength: float = 0.8
@export var terrain_bumpiness: float = 0.15

@export_group("Simulation Accuracy")
@export var contact_patch_radius: float = 0.08  # meters
@export var num_contact_points_per_wheel: int = 4
@export var raycast_offset: float = 0.01  # meters below wheel center
@export var ground_plane_normal: Vector3 = Vector3.UP
@export var ground_plane_check_distance: float = 0.5  # meters

# Unit conversion constants
const MILES_TO_KM = 1.60934
const FEET_TO_METERS = 0.3048
const INCHES_TO_METERS = 0.0254
const PSI_TO_BAR = 0.0689476
const BAR_TO_PSI = 14.5038
const RPM_TO_RADS = PI / 30.0
const DEGREES_TO_RADS = PI / 180.0
const RADS_TO_DEGREES = 180.0 / PI

func _init() -> void:
	_validate_and_clamp_values()

func _validate_and_clamp_values() -> void:
	"""Ensure all physics values are within reasonable bounds."""
	
	# Clamp gravity to Earth-like range
	gravity = clamp(gravity, 1.0, 20.0)
	
	# Clamp mass to realistic vehicle range
	default_vehicle_mass = clamp(default_vehicle_mass, 500.0, 5000.0)
	
	# Ensure wheel radius is positive and reasonable
	default_wheel_radius = clamp(default_wheel_radius, 0.2, 0.6)
	
	# Clamp suspension travel
	max_suspension_travel = clamp(max_suspension_travel, 0.1, 0.5)
	
	# Clamp friction coefficients
	tire_friction_horizontal = clamp(tire_friction_horizontal, 0.5, 2.5)
	tire_friction_vertical = clamp(tire_friction_vertical, 0.1, 1.0)
	
	# Clamp aerodynamic coefficients
	drag_coefficient = clamp(drag_coefficient, 0.1, 1.0)
	lift_coefficient = clamp(lift_coefficient, -0.5, 0.5)
	
	# Clamp RPM values
	idle_rpm = clamp(idle_rpm, 500.0, 1500.0)
	redline_rpm = clamp(redline_rpm, 5000.0, 10000.0)
	peak_torque_rpm = clamp(peak_torque_rpm, idle_rpm, redline_rpm)
	peak_horsepower_rpm = clamp(peak_horsepower_rpm, peak_torque_rpm, redline_rpm)
	
	# Validate gear ratios (all should be positive)
	for i in range(gear_ratios.size()):
		gear_ratios[i] = max(0.1, gear_ratios[i])
	final_drive_ratio = max(2.0, min(5.0, final_drive_ratio))
	
	# Clamp brake bias
	brake_bias_front = clamp(brake_bias_front, 0.4, 0.7)
	
	# Clamp steering lock
	steering_lock_angle = clamp(steering_lock_angle, 1.0, 4.0)

## Get engine torque at given RPM using a simplified bell curve model
func get_engine_torque(rpm: float) -> float:
	"""Calculate engine torque based on RPM using a bell curve approximation."""
	rpm = clamp(rpm, idle_rpm * 0.5, redline_rpm * 1.1)
	
	var rpm_normalized = (rpm - idle_rpm) / (redline_rpm - idle_rpm)
	
	# Bell curve centered around peak_torque_rpm
	var peak_normalized = (peak_torque_rpm - idle_rpm) / (redline_rpm - idle_rpm)
	var torque_peak = 400.0  # Nm peak torque reference
	
	var bell_curve = exp(-pow((rpm_normalized - peak_normalized), 2) * 8.0)
	
	return torque_peak * bell_curve

## Calculate horsepower from torque and RPM
func calculate_horsepower(torque_nm: float, rpm: float) -> float:
	"""Convert torque and RPM to horsepower (HP = Torque * RPM / 5252)."""
	return torque_nm * rpm / 5252.0

## Get gear ratio for current gear
func get_gear_ratio(gear_index: int) -> float:
	if gear_index < 0 or gear_index >= gear_ratios.size():
		return neutral_ratio
	return gear_ratios[gear_index]

## Calculate wheel angular velocity from vehicle linear velocity
func calculate_wheel_angular_velocity(vehicle_speed_ms: float) -> float:
	"""Convert linear speed to wheel RPM assuming no slip."""
	var wheel_circumference = 2.0 * PI * default_wheel_radius
	var wheel_rps = vehicle_speed_ms / wheel_circumference
	return wheel_rps * 60.0  # Convert to RPM

## Calculate theoretical vehicle speed from wheel RPM
func calculate_vehicle_speed_from_wheel_rpm(wheel_rpm: float) -> float:
	"""Convert wheel RPM back to vehicle speed in m/s."""
	var wheel_rps = wheel_rpm / 60.0
	var wheel_circumference = 2.0 * PI * default_wheel_radius
	return wheel_rps * wheel_circumference

## Calculate aerodynamic drag force
func calculate_drag_force(vehicle_speed_ms: float) -> float:
	"""Calculate aerodynamic drag force (F = 0.5 * rho * Cd * A * v^2)."""
	var speed_sq = vehicle_speed_ms * vehicle_speed_ms
	return 0.5 * air_density * drag_coefficient * front_area * speed_sq

## Calculate aerodynamic downforce
func calculate_downforce(vehicle_speed_ms: float) -> float:
	"""Calculate aerodynamic downforce (negative lift = downforce)."""
	var speed_sq = vehicle_speed_ms * vehicle_speed_ms
	return 0.5 * air_density * lift_coefficient * front_area * speed_sq

## Calculate braking force based on pedal input
func calculate_braking_force(pedal_input: float) -> float:
	"""Calculate total braking force based on pedal pressure (0-1)."""
	pedal_input = clamp(ped al_input, 0.0, 1.0)
	return brake_force_per_pedal * pedal_input * brake_bias_front

## Get maximum braking force considering temperature effects
func get_max_braking_force_at_temp(temp_celsius: float) -> float:
	"""Adjust braking force based on brake temperature."""
	temp_celsius = clamp(temp_celsius, 0.0, brake_temp_max)
	
	var temp_factor = 1.0
	if temp_celsius > optimal_brake_temp:
		var temp_excess = temp_celsius - optimal_brake_temp
		temp_factor = 1.0 - (brake_fade_factor * temp_excess)
	
	temp_factor = max(0.1, min(1.0, temp_factor))
	return brake_force_per_pedal * temp_factor

## Convert speed units
func kmh_to_ms(speed_kmh: float) -> float:
	return speed_kmh / 3.6

func ms_to_kmh(speed_ms: float) -> float:
	return speed_ms * 3.6

func mph_to_ms(speed_mph: float) -> float:
	return speed_mph * 0.44704

func ms_to_mph(speed_ms: float) -> float:
	return speed_ms / 0.44704

func feet_to_meters(feet: float) -> float:
	return feet * FEET_TO_METERS

func meters_to_feet(meters: float) -> float:
	return meters / FEET_TO_METERS

func inches_to_meters(inches: float) -> float:
	return inches * INCHES_TO_METERS

func meters_to_inches(meters: float) -> float:
	return meters / INCHES_TO_METERS

## Angle conversions
func degrees_to_radians(degrees: float) -> float:
	return degrees * DEGREES_TO_RADS

func radians_to_degrees(radians: float) -> float:
	return radians * RADS_TO_DEGREES

## Pressure conversions
func psi_to_bar(psi: float) -> float:
	return psi * PSI_TO_BAR

func bar_to_psi(bar: float) -> float:
	return bar * BAR_TO_PSI

## RPM conversions
func rpm_to_rads_per_sec(rpm: float) -> float:
	return rpm * RPM_TO_RADS

func rads_per_sec_to_rpm(rads: float) -> float:
	return rads / RPM_TO_RADS

## Calculate cornering force using simplified Pacejka equation
func calculate_lateral_force(slip_angle_rad: float, normal_load: float) -> float:
	"""Simplified Pacejka magic formula for lateral force."""
	slip_angle_rad = clamp(slip_angle_rad, -PI/2, PI/2)
	
	# B * sin(A * atan(C * tan^-1(C * slip)))
	# Simplified version for stability
	var c = 3.0
	var d = 1.2
	var e = 0.9
	
	var term1 = atan(c * slip_angle_rad)
	var term2 = atan(d * term1)
	var lateral_force = normal_load * sin(term2)
	
	return lateral_force

## Calculate longitudinal force (traction/braking)
func calculate_longitudinal_force(slip_ratio: float, normal_load: float) -> float:
	"""Calculate longitudinal force based on slip ratio."""
	slip_ratio = clamp(slip_ratio, -1.0, 1.0)
	
	# Peak coefficient at small slip
	var peak_coefficient = tire_friction_horizontal
	var slip_peak = 0.1
	
	# Force increases with slip up to peak, then decreases
	var force_coefficient = peak_coefficient * exp(-abs(slip_ratio - slip_peak) * 10.0)
	
	return normal_load * force_coefficient * sign(slip_ratio)

## Get tire friction circle limit
func get_friction_circle_limit(normal_load: float) -> float:
	"""Get maximum combined lateral and longitudinal force."""
	return normal_load * tire_friction_horizontal

## Check if vehicle is drifting
func is_drifting(lateral_slip: float, longitudinal_slip: float) -> bool:
	"""Determine if vehicle is currently drifting based on slip values."""
	var slip_angle = asin(lateral_slip)
	return abs(sl ip_angle) > minimum_drift_angle * DEGREES_TO_RADS

## Apply drift recovery modifier to steering
func apply_drift_recovery(steer_input: float, drift_angle: float) -> float:
	"""Modify steering input based on drift state."""
	if not is_drifting(drift_angle, 0):
		return steer_input
	
	# Reduce effective steering when drifting
	var recovery_factor = 1.0 - (drift_recovery_speed * drift_angle)
	recovery_factor = max(0.1, min(1.0, recovery_factor))
	
	return steer_input * recovery_factor

## Calculate centrifugal force during cornering
func calculate_centrifugal_force(vehicle_mass: float, lateral_acceleration_ms2: float) -> float:
	"""Calculate centrifugal force acting on vehicle during cornering."""
	return vehicle_mass * lateral_acceleration_ms2

## Get recommended tire pressure for conditions
func get_recommended_tire_pressure(track_temp_celsius: float, weather_condition: String) -> float:
	"""Calculate recommended tire pressure based on conditions."""
	var base_pressure = 2.2  # bar
	var track_temp_adjustment = (track_temp_celsius - 20.0) * 0.01
	
	var weather_factor = 1.0
	match weather_condition:
		"rain":
			weather_factor = 0.95
		"snow":
			weather_factor = 0.85
		"dry":
			weather_factor = 1.05
	
	return base_pressure + track_temp_adjustment * weather_factor

## Calculate traction loss probability based on surface conditions
func calculate_traction_loss_probability(surface_friction: float, wheel_slip: float) -> float:
	"""Estimate probability of losing traction based on conditions."""
	var combined_slip = abs(wheel_slip) + abs(surface_friction - 1.0) * 5.0
	return clamp(combined_slip, 0.0, 1.0)

## Reset all values to defaults
func reset_to_defaults() -> void:
	"""Restore all physics settings to factory defaults."""
	_init()

## Export settings to dictionary for save/load
func export_to_dict() -> Dictionary:
	return {
		"gravity": gravity,
		"physics_tick_rate": physics_tick_rate,
		"default_vehicle_mass": default_vehicle_mass,
		"default_wheel_radius": default_wheel_radius,
		"suspension_stiffness": suspension_stiffness,
		"tire_friction_horizontal": tire_friction_horizontal,
		"drag_coefficient": drag_coefficient,
		"gear_ratios": gear_ratios,
		"diff_lock_rear": diff_lock_rear,
		"brake_bias_front": brake_bias_front,
		"steering_lock_angle": steering_lock_angle
	}

## Import settings from dictionary
func import_from_dict(data: Dictionary) -> void:
	if data.has("gravity"):
		gravity = data["gravity"]
	if data.has("default_vehicle_mass"):
		default_vehicle_mass = data["default_vehicle_mass"]
	if data.has("suspension_stiffness"):
		suspension_stiffness = data["suspension_stiffness"]
	if data.has("tire_friction_horizontal"):
		tire_friction_horizontal = data["tire_friction_horizontal"]
	if data.has("drag_coefficient"):
		drag_coefficient = data["drag_coefficient"]
	if data.has("gear_ratios") and data["gear_ratios"].size() == 6:
		gear_ratios = data["gear_ratios"]
	if data.has("diff_lock_rear"):
		diff_lock_rear = data["diff_lock_rear"]
	if data.has("brake_bias_front"):
		brake_bias_front = data["brake_bias_front"]
	if data.has("steering_lock_angle"):
		steering_lock_angle = data["steering_lock_angle"]
	
	_validate_and_clamp_values()