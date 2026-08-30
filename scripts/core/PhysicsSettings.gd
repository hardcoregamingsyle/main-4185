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
@export var suspension_stiffness: float = 35000.0
@export var suspension_compression_damping: float = 1200.0
@export var suspension_rebound_damping: float = 600.0
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
@export var center_of_mass_x: float = 0.5  # normalized from front bumper (0.0 to 1.0)
@export var center_of_mass_y: float = 0.45  # normalized from ground (0.0 = ground, 1.0 = roof)
@export var center_of_mass_z: float = 0.5  # normalized from left to right (-1.0 to 1.0)
@export var chassis_width: float = 1.85  # meters
@export var chassis_height: float = 1.4  # meters
@export var chassis_length: float = 4.5  # meters
@export var wheelbase: float = 2.7  # meters
@export var track_width_front: float = 1.6  # meters
@export var track_width_rear: float = 1.65  # meters

@export_group("Differential Settings")
@export var differential_type: int = 1  # 0=Open, 1=LSD, 2=Locked
@export var lsd_preload_torque: float = 100.0  # Nm
@export var lsd_max_locking_torque: float = 1500.0  # Nm
@export var open_diff_slip_angle: float = 5.0  # degrees
@export var locked_diff_torque_split: float = 0.5  # 50/50 split

@export_group("Steering System")
@export var steering_lock_angle: float = 30.0  # degrees (maximum turn angle)
@export var steering_ratio: float = 14.0  # steering ratio (degrees input to wheel angle)
@export var steering_sensitivity: float = 1.0  # sensitivity multiplier
@export var steering_return_speed: float = 2.0  # radians/sec return speed
@export var steering_deadzone: float = 0.02  # deadzone for stick drift prevention

@export_group("Fuel System")
@export var fuel_capacity_liters: float = 60.0
@export var fuel_consumption_base: float = 0.12  # liters per second at idle
@export var fuel_consumption_multiplier: float = 0.0001  # additional consumption per RPM above idle
@export var fuel_tank_position_x: float = 0.4  # normalized position along vehicle
@export var fuel_tank_position_y: float = 0.3  # normalized height from ground

@export_group("Damage Physics")
@export var collision_damage_multiplier: float = 1.0
@export var impact_velocity_threshold: float = 10.0  # m/s before damage applies
@export var structural_integrity_initial: float = 1.0
@export var deformation_rate: float = 0.001  # deformation per impact severity

@export_group("AI Racing Parameters")
@export var ai_optimal_line_buffer: float = 0.5  # meters from ideal line
@export var ai_braking_distance_margin: float = 5.0  # meters extra braking distance
@export var ai_cornering_speed_factor: float = 0.85  # percentage of max cornering speed
@export var ai_acceleration_curve_smoothness: float = 0.9

# Helper Methods for Unit Conversions

## Convert RPM to rad/s
func rpm_to_rad(rpm: float) -> float:
	return rpm * PI / 30.0

## Convert rad/s to RPM
func rad_to_rpm(rad: float) -> float:
	return rad * 30.0 / PI

## Convert km/h to m/s
func kmh_to_ms(kmh: float) -> float:
	return kmh / 3.6

## Convert m/s to km/h
func ms_to_kmh(ms: float) -> float:
	return ms * 3.6

## Convert degrees to radians
func deg_to_rad(deg: float) -> float:
	return deg * PI / 180.0

## Convert radians to degrees
func rad_to_deg(rad: float) -> float:
	return rad * 180.0 / PI

## Convert Newtons to pounds-force
func newton_to_lbf(newton: float) -> float:
	return newton * 0.224809

## Convert pounds-force to Newtons
func lbf_to_newton(lbf: float) -> float:
	return lbf / 0.224809

## Convert kg to pounds
func kg_to_lb(kg: float) -> float:
	return kg * 2.20462

## Convert pounds to kg
func lb_to_kg(lb: float) -> float:
	return lb / 2.20462

## Convert inches to meters
func inch_to_m(inch: float) -> float:
	return inch * 0.0254

## Convert meters to inches
func m_to_inch(m: float) -> float:
	return m / 0.0254

## Calculate aerodynamic drag force at given velocity
func calculate_drag_force(velocity: Vector3, mass: float) -> Vector3:
	var speed = velocity.length()
	var drag = 0.5 * air_density * drag_coefficient * front_area
	var drag_magnitude = drag * speed * speed
	return -velocity.normalized() * drag_magnitude

## Calculate aerodynamic downforce at given velocity
func calculate_downforce(velocity: Vector3, mass: float) -> float:
	var speed = velocity.length()
	var downforce = 0.5 * air_density * lift_coefficient * front_area * speed * speed
	return downforce

## Calculate torque at current RPM based on engine curve
func get_engine_torque(rpm: float) -> float:
	if rpm <= idle_rpm:
		return 0.0
	elif rpm >= redline_rpm:
		return 0.0
	
	# Simple parabolic torque curve peaking at peak_torque_rpm
	var peak_rpm = peak_torque_rpm
	var torque_peak = 450.0  # Nm peak torque value
	var torque_at_rpm = torque_peak * (1.0 - pow((rpm - peak_rpm) / (redline_rpm - peak_rpm), 2))
	
	return clamp(torque_at_rpm, 0.0, torque_peak)

## Get wheel angular velocity from vehicle speed and gear ratio
func calculate_wheel_angular_velocity(vehicle_speed: float, gear_index: int) -> float:
	var total_ratio = gear_ratios[gear_index] * final_drive_ratio
	var wheel_circumference = 2.0 * PI * default_wheel_radius
	var wheel_angular_vel = vehicle_speed / default_wheel_radius
	return wheel_angular_vel

## Calculate maximum cornering force based on tire friction
func calculate_max_cornering_force(normal_force: float) -> float:
	return normal_force * tire_friction_horizontal

## Calculate maximum acceleration force based on tire friction
func calculate_max_acceleration_force(normal_force: float) -> float:
	return normal_force * tire_friction_horizontal

## Check if vehicle is drifting based on slip angle
func is_drifting(slip_angle: float) -> bool:
	return abs(slip_angle) > minimum_drift_angle

## Get effective friction coefficient considering drift state
func get_effective_friction(slip_angle: float, is_drifting_state: bool) -> float:
	if is_drifting_state:
		return tire_friction_horizontal * drift_multiplier
	else:
		return tire_friction_horizontal

## Clamp value between min and max
func clamp_value(value: float, min_val: float, max_val: float) -> float:
	return clamp(value, min_val, max_val)

## Linear interpolation between two values
func lerp_values(a: float, b: float, t: float) -> float:
	return a + (b - a) * t

## Smoothstep interpolation for smoother transitions
func smoothstep_interp(t: float) -> float:
	return t * t * (3.0 - 2.0 * t)

## Generate preset configurations for different vehicle types
static func create_preset(vehicle_type: String) -> PhysicsSettings:
	var settings = PhysicsSettings.new()
	
	match vehicle_type:
		"racing":
			settings.drag_coefficient = 0.28
			settings.lift_coefficient = -0.15  # Downforce
			settings.suspension_stiffness = 45000.0
			settings.brake_force_per_wheel = 12000.0
			settings.tire_friction_horizontal = 1.4
		"street":
			settings.drag_coefficient = 0.32
			settings.lift_coefficient = 0.10
			settings.suspension_stiffness = 35000.0
			settings.brake_force_per_wheel = 8000.0
			settings.tire_friction_horizontal = 1.2
		"offroad":
			settings.drag_coefficient = 0.45
			settings.lift_coefficient = 0.20
			settings.suspension_stiffness = 25000.0
			settings.brake_force_per_wheel = 6000.0
			settings.tire_friction_horizontal = 1.0
			settings.max_suspension_travel = 0.35
		"drift":
			settings.drag_coefficient = 0.30
			settings.lift_coefficient = 0.12
			settings.drift_multiplier = 2.0
			settings.minimum_drift_angle = 10.0
			settings.maximum_drift_angle = 60.0
			settings.tire_friction_horizontal = 1.0
		_:
			pass  # Use defaults
	
	return settings

## Validate all physics values are within reasonable bounds
func validate_settings() -> Dictionary:
	var warnings = []
	var errors = []
	
	if gravity <= 0.0:
		errors.append("Gravity must be positive")
	
	if default_vehicle_mass <= 0.0:
		errors.append("Vehicle mass must be positive")
	
	if default_wheel_radius <= 0.0:
		errors.append("Wheel radius must be positive")
	
	for i in range(gear_ratios.size()):
		if gear_ratios[i] <= 0.0:
			errors.append("Gear ratio %d must be positive" % i)
	
	if idle_rpm < 0.0 or idle_rpm > 2000.0:
		warnings.append("Idle RPM seems unusual")
	
	if redline_rpm <= idle_rpm:
		errors.append("Redline must be higher than idle RPM")
	
	if peak_torque_rpm < idle_rpm or peak_torque_rpm > redline_rpm:
		warnings.append("Peak torque RPM outside expected range")
	
	if brake_bias_front < 0.0 or brake_bias_front > 1.0:
		errors.append("Brake bias must be between 0.0 and 1.0")
	
	return {"errors": errors, "warnings": warnings}

</FileBlock>>