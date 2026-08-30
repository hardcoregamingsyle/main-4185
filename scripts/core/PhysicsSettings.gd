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
@export var engine_bore: float = 0.086  # meters
@export var engine_stroke: float = 0.086  # meters
@export var compression_ratio: float = 11.0
@export var fuel_air_ratio: float = 0.068  # stoichiometric ratio

@export_group("Gear Ratios")
@export var gear_ratios: Array[float] = [3.8, 2.4, 1.7, 1.3, 1.0, 0.85]
@export var reverse_gear_ratio: float = -4.0
@export var final_drive_ratio: float = 3.5
@export var gear_shift_delay: float = 0.15  # seconds

@export_group("Differential Settings")
@export var differential_type: int = 1  # 0=Open, 1=LSD, 2=Locked
@export var lsd_preload: float = 0.3  # LSD preload percentage (0-1)
@export var lsd_lockup_ratio: float = 0.6  # Maximum lockup percentage
@export var open_diff_slip_factor: float = 0.15  # Slip factor for open diff

@export_group("Steering System")
@export var steering_ratio: float = 15.0  # Turns wheel : turns road wheels
@export var steering_lock_left: float = 270.0  # degrees
@export var steering_lock_right: float = 270.0  # degrees
@export var steering_speed: float = 45.0  # degrees per second
@export var steering_offset: float = 0.0  # Deadzone offset
@export var power_steering_ratio: float = 0.0  # Power assist multiplier (0=no, 1=max)

@export_group("Braking System")
@export var brake_force_distribution_front: float = 0.6  # Front brake bias (0-1)
@export var brake_force_distribution_rear: float = 0.4  # Rear brake bias (0-1)
@export var max_brake_pressure: float = 100.0  # bar
@export var brake_linearization: float = 1.0
@export var brake_fade_factor: float = 0.999  # Fade per lap (1=no fade)
@export var abs_threshold: float = 0.25  # ABS slip threshold
@export var ebd_curve: Array[float] = [0.0, 0.2, 0.4, 0.6, 0.8, 1.0]
@export var ebd_values: Array[float] = [0.0, 0.5, 0.7, 0.85, 0.92, 1.0]

@export_group("Transmission")
@export var transmission_type: int = 0  # 0=Manual, 1=Automatic, 2=Semi-auto
@export var auto_shift_rpm_up: float = 6500.0
@export var auto_shift_rpm_down: float = 3000.0
@export var clutch_disengage_rpm: float = 1500.0
@export var clutch_engage_rpm: float = 1000.0
@export var clutch_friction: float = 0.85

@export_group("Track & Surface Properties")
@export var asphalt_friction: float = 1.1
@export var gravel_friction: float = 0.6
@export var dirt_friction: float = 0.55
@export var grass_friction: float = 0.4
@export var ice_friction: float = 0.1
@export var water_friction: float = 0.05
@export var track_roughness: float = 0.02  # Bumpiness factor
@export var track_camber_factor: float = 0.05  # Lateral camber effect

@export_group("Collision Physics")
@export var collision_margin: float = 0.01  # meters
@export var restitution: float = 0.1  # Bounciness
@export var contact_offset: float = 0.005  # Contact detection offset
@export var raycast_max_distance: float = 2.0  # Suspension raycast distance
@export var raycast_collision_filter: int = 0x1

## Helper Methods for Unit Conversions

func rpm_to_radians_per_second(rpm: float) -> float:
	"""Convert RPM to radians per second."""
	return rpm * PI / 30.0

func radians_per_second_to_rpm(rad_per_sec: float) -> float:
	"""Convert radians per second to RPM."""
	return rad_per_sec * 30.0 / PI

func kmh_to_ms(kmh: float) -> float:
	"""Convert kilometers per hour to meters per second."""
	return kmh / 3.6

func ms_to_kmh(ms: float) -> float:
	"""Convert meters per second to kilometers per hour."""
	return ms * 3.6

func mph_to_ms(mph: float) -> float:
	"""Convert miles per hour to meters per second."""
	return mph * 0.44704

func ms_to_mph(ms: float) -> float:
	"""Convert meters per second to miles per hour."""
	return ms / 0.44704

def degrees_to_radians(degrees: float) -> float:
	"""Convert degrees to radians."""
	return degrees * PI / 180.0

func radians_to_degrees(radians: float) -> float:
	"""Convert radians to degrees."""
	return radians * 180.0 / PI

func inches_to_meters(inches: float) -> float:
	"""Convert inches to meters."""
	return inches * 0.0254

func meters_to_inches(meters: float) -> float:
	"""Convert meters to inches."""
	return meters / 0.0254

func pounds_to_kg(pounds: float) -> float:
	"""Convert pounds to kilograms."""
	return pounds * 0.453592

func kg_to_pounds(kg: float) -> float:
	"""Convert kilograms to pounds."""
	return kg / 0.453592

func ft_lb_to_nm(ft_lb: float) -> float:
	"""Convert foot-pounds to Newton-meters (torque)."""
	return ft_lb * 1.3558179

func nm_to_ft_lb(nm: float) -> float:
	"""Convert Newton-meters to foot-pounds (torque)."""
	return nm / 1.3558179

func hp_to_kw(hp: float) -> float:
	"""Convert horsepower to kilowatts."""
	return hp * 0.745699872

func kw_to_hp(kw: float) -> float:
	"""Convert kilowatts to horsepower."""
	return kw / 0.745699872

func psi_to_bar(psi: float) -> float:
	"""Convert PSI to bar (pressure)."""
	return psi * 0.0689476

func bar_to_psi(bar: float) -> float:
	"""Convert bar to PSI (pressure)."""
	return bar / 0.0689476

## Torque Curve Calculation

func calculate_engine_torque(rpm: float) -> float:
	"""Calculate engine torque based on RPM using a bell curve approximation."""
	if rpm <= idle_rpm or rpm >= redline_rpm:
		return 0.0
	
	var normalized = (rpm - idle_rpm) / (redline_rpm - idle_rpm)
	
	# Bell curve centered around peak torque RPM
	var peak_normalized = (peak_torque_rpm - idle_rpm) / (redline_rpm - idle_rpm)
	var torque_factor = exp(-pow((normalized - peak_normalized) / 0.25, 2))
	
	# Base torque (will be scaled by actual engine specs)
	var base_torque = 350.0  # Nm placeholder
	
	return base_torque * torque_factor

## Horsepower Calculation

func calculate_engine_horsepower(torque_nm: float, rpm: float) -> float:
	"""Calculate horsepower from torque and RPM."""
	return torque_nm * rpm / 7127.0  # HP = (Torque * RPM) / 7127

## Aerodynamic Force Calculation

func calculate_drag_force(speed_ms: float) -> float:
	"""Calculate aerodynamic drag force."""
	var velocity_squared = speed_ms * speed_ms
	var drag_force = 0.5 * air_density * drag_coefficient * front_area * velocity_squared
	return drag_force

func calculate_lift_force(speed_ms: float) -> float:
	"""Calculate aerodynamic lift force (negative = downforce)."""
	var velocity_squared = speed_ms * speed_ms
	var lift_force = 0.5 * air_density * lift_coefficient * front_area * velocity_squared
	return lift_force

## Tire Friction Model (Simplified Pacejka)

func calculate_longitudinal_friction(slip_ratio: float, vertical_load: float) -> float:
	"""Calculate longitudinal friction coefficient using simplified Pacejka model."""
	var b = 5.0  # Shape factor
	var c = 1.1  # Curve factor
	var d = min(tire_friction_horizontal * vertical_load / 1000.0 + 0.5, 2.0)
	var e = 0.95
	
	var sin_input = b * atan(c * slip_ratio * (1.0 - e) + e * asin(b * slip_ratio))
	return d * sin(sin_input)

func calculate_lateral_friction(slip_angle: float, vertical_load: float) -> float:
	"""Calculate lateral friction coefficient using simplified Pacejka model."""
	var slip_angle_rad = slip_angle * PI / 180.0
	var b = 5.0
	var c = 1.1
	var d = min(tire_friction_horizontal * vertical_load / 1000.0 + 0.5, 2.0)
	var e = 0.95
	
	var sin_input = b * atan(c * slip_angle_rad * (1.0 - e) + e * asin(b * slip_angle_rad))
	return d * sin(sin_input)

## Suspension Force Calculation

func calculate_suspension_force(compression: float, velocity: float) -> Vector3:
	"""Calculate suspension force based on compression and velocity."""
	var force = Vector3.ZERO
	
	# Spring force (Hooke's law)
	var spring_force = -suspension_stiffness * compression
	spring_force = clamp(spring_force, -suspension_stiffness * max_suspension_travel, suspension_stiffness * max_suspension_travel)
	
	# Damper force
	var damping_force = -suspension_compression_damping * velocity if velocity < 0 else -suspension_rebound_damping * velocity
	
	force.y = spring_force + damping_force
	return force

## Brake Force Distribution

func get_brake_force_total(total_brake_input: float, speed_ms: float) -> float:
	"""Calculate total brake force based on pedal input and speed."""
	var effective_brake = total_brake_input * max_brake_pressure * brake_linearization
	effective_brake *= max(0.1, speed_ms / 10.0) if speed_ms > 0 else 1.0
	return effective_brake

func get_front_brake_force(total_force: float) -> float:
	"""Get front axle brake force based on distribution."""
	return total_force * brake_force_distribution_front

func get_rear_brake_force(total_force: float) -> float:
	"""Get rear axle brake force based on distribution."""
	return total_force * brake_force_distribution_rear

## Vehicle Performance Metrics

func calculate_top_speed(gear_index: int) -> float:
	"""Calculate theoretical top speed for a given gear."""
	var gear_ratio = gear_ratios[gear_index] if gear_index < gear_ratios.size() else gear_ratios[-1]
	var total_ratio = gear_ratio * final_drive_ratio
	var wheel_rpm = redline_rpm / total_ratio
	var wheel_circumference = 2.0 * PI * default_wheel_radius
	var top_speed_ms = wheel_rpm * wheel_circumference / 60.0
	return ms_to_kmh(top_speed_ms)

func calculate_acceleration(force_n: float) -> float:
	"""Calculate acceleration from force (F=ma)."""
	return force_n / default_vehicle_mass

## Track Surface Lookup

func get_surface_friction(surface_type: String) -> float:
	"""Get friction coefficient for track surface type."""
	match surface_type.to_lower():
		"asphalt", "tarmac": return asphalt_friction
		"gravel": return gravel_friction
		"dirt", "earth": return dirt_friction
		"grass", "turf": return grass_friction
		"ice", "snow": return ice_friction
		"water", "wet": return water_friction
		_: return asphalt_friction  # Default to asphalt

## Utility Functions

func normalize_value(value: float, min_val: float, max_val: float) -> float:
	"""Normalize value to 0-1 range."""
	return clamp((value - min_val) / (max_val - min_val), 0.0, 1.0)

func lerp_value(min_val: float, max_val: float, t: float) -> float:
	"""Linear interpolation between two values."""
	return min_val + (max_val - min_val) * t

func map_range(value: float, in_min: float, in_max: float, out_min: float, out_max: float) -> float:
	"""Map a value from one range to another."""
	var mapped = normalize_value(value, in_min, in_max)
	return lerp_value(out_min, out_max, mapped)

func get_current_settings_as_dict() -> Dictionary:
	"""Export current settings as dictionary for serialization."""
	return {
		"gravity": gravity,
		"physics_tick_rate": physics_tick_rate,
		"vehicle_mass": default_vehicle_mass,
		"suspension_stiffness": suspension_stiffness,
		"tire_friction": tire_friction_horizontal,
		"aero_drag": drag_coefficient,
		"gears": gear_ratios.duplicate(),
		"differential_type": differential_type,
		"steering_lock": steering_lock_left,
		"brake_bias": brake_force_distribution_front
	}

func load_from_dict(settings_dict: Dictionary) -> void:
	"""Load settings from dictionary."""
	if settings_dict.has("gravity"): gravity = settings_dict["gravity"]
	if settings_dict.has("physics_tick_rate"): physics_tick_rate = settings_dict["physics_tick_rate"]
	if settings_dict.has("vehicle_mass"): default_vehicle_mass = settings_dict["vehicle_mass"]
	if settings_dict.has("suspension_stiffness"): suspension_stiffness = settings_dict["suspension_stiffness"]
	if settings_dict.has("tire_friction"): tire_friction_horizontal = settings_dict["tire_friction"]
	if settings_dict.has("aero_drag"): drag_coefficient = settings_dict["aero_drag"]
	if settings_dict.has("gears"): gear_ratios = settings_dict["gears"].duplicate()
	if settings_dict.has("differential_type"): differential_type = settings_dict["differential_type"]
	if settings_dict.has("steering_lock"): steering_lock_left = settings_dict["steering_lock"]
	if settings_dict.has("brake_bias"): brake_force_distribution_front = settings_dict["brake_bias"]

</File>