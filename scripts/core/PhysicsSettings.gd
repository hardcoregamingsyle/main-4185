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
@export var fuel_consumption_base: float = 8.5  # liters per 100km
@export var fuel_tank_capacity: float = 60.0  # liters

@export_group("Gear Ratios")
@export var gear_ratios: Array[float] = [3.8, 2.4, 1.6, 1.2, 0.9, 0.7]
@export var final_drive_ratio: float = 3.5
@export var reverse_gear_ratio: float = 3.6
@export var clutch_slip_torque: float = 500.0  # Nm

@export_group("Differential Settings")
@export var diff_type: DiffType = DiffType.LSD
@export var diff_lock: float = 0.5  # LSD lock percentage (0-1)
@export var diff_preload: float = 150.0  # Nm preload torque
@export var diff_compression_ratio: float = 1.0
@export var diff_extension_ratio: float = 1.0

enum DiffType {
	OPEN,
	LSD,
	SUPERCHARGED_LSD,
	CLOSED_DIFF
}

@export_group("Brake System")
@export var brake_force_per_pedal: float = 15000.0  # N per wheel at full brake
@export var brake_distribution_front: float = 0.55  # front/rear bias
@export var brake_distribution_rear: float = 0.45
@export var abs_threshold: float = 0.85  # slip ratio threshold for ABS
@export var brake_bias_adjustment_range: float = 0.2  # +/- 0.2 via steering wheel controls
@export var parking_brake_force: float = 8000.0  # N per rear wheel

@export_group("Steering")
@export var steering_ratio: float = 14.0  # steering wheel turns to wheel angle
@export var steering_max_angle: float = 30.0  # degrees wheel turn
@export var steering_deadzone: float = 0.05  # input deadzone
@export var steering_speed: float = 8.0  # degrees per second at full input
@export var steering_weight_curve: float = 0.6  # curve factor for weight increase at higher speeds

@export_group("Powertrain Limits")
@export var max_engine_torque: float = 450.0  # Nm
@export var max_engine_power: float = 280.0  # kW
@export var min_engine_torque: float = 150.0  # Nm at idle
@export var engine_spool_up_time: float = 0.1  # seconds to reach target torque
@export var engine_spool_down_time: float = 0.05  # seconds to reduce torque
@export var overrev_penalty_factor: float = 0.5  # torque reduction when exceeding redline

@export_group("Transmission")
@export var shift_time: float = 0.15  # seconds for gear change
@export var upshift_rpm_buffer: float = 100.0  # RPM buffer above redline for auto-upshift
@export var downshift_rpm_buffer: float = 300.0  # RPM buffer below optimal for auto-downshift
@export var rev_matching_enabled: bool = true
@export var rev_match_target_rpm_offset: float = 500.0  # RPM offset for rev matching

@export_group("Track Surface Properties")
@export var asphalt_friction: float = 0.9
@export var concrete_friction: float = 0.85
@export var gravel_friction: float = 0.5
@export var dirt_friction: float = 0.4
@export var wet_asphalt_friction: float = 0.6
@export var ice_friction: float = 0.15
@export var rain_intensity_effect: float = 0.1  # friction reduction per rain intensity level

@export_group("Collision Physics")
@export var collision_margin: float = 0.01  # collision detection margin
@export var chassis_collision_segments: int = 12
@export var raycast_distance: float = 1.0  # suspension raycast distance
@export var raycast_frustum_angle: float = 0.1  # radians cone angle for multi-point contact

@export_group("Simulation Accuracy")
@export var use_precise_physics: bool = true
@export var substep_interpolation: bool = true
@export var interpolation_quality: float = 0.5  # 0=none, 0.5=between frames, 1=full
@export var physics_debug_render: bool = false

@export_group("Miscellaneous")
@export var time_scale_normal: float = 1.0
@export var time_scale_pause: float = 0.0
@export var time_scale_slowmo: float = 0.5
@export var slow_mo_trigger_threshold: float = 0.9  # speed ratio trigger for slow-mo
@export var crash_damage_mult: float = 1.0  # damage multiplier based on impact severity


# Helper Methods for Unit Conversions

func rpm_to_rads_per_sec(rpm: float) -> float:
	"""Convert RPM to radians per second"""
	return rpm * PI / 30.0

func rads_per_sec_to_rpm(rad_per_sec: float) -> float:
	"""Convert radians per second to RPM"""
	return rad_per_sec * 30.0 / PI

func kmh_to_ms(kmh: float) -> float:
	"""Convert km/h to m/s"""
	return kmh / 3.6

func ms_to_kmh(ms: float) -> float:
	"""Convert m/s to km/h"""
	return ms * 3.6

func mph_to_ms(mph: float) -> float:
	"""Convert miles per hour to m/s"""
	return mph * 0.44704

func ms_to_mph(ms: float) -> float:
	"""Convert m/s to miles per hour"""
	return ms / 0.44704

func deg_to_rad(degrees: float) -> float:
	"""Convert degrees to radians"""
	return degrees * PI / 180.0

func rad_to_deg(rad: float) -> float:
	"""Convert radians to degrees"""
	return rad * 180.0 / PI

func newton_to_lb_ft(newton: float) -> float:
	"""Convert Newton-meters to pound-feet"""
	return newton * 0.737562

func lb_ft_to_newton(lb_ft: float) -> float:
	"""Convert pound-feet to Newton-meters"""
	return lb_ft / 0.737562

func kg_to_lb(kg: float) -> float:
	"""Convert kilograms to pounds"""
	return kg * 2.20462

func lb_to_kg(lb: float) -> float:
	"""Convert pounds to kilograms"""
	return lb / 2.20462

func hp_to_kw(hp: float) -> float:
	"""Convert horsepower to kilowatts"""
	return hp * 0.7457

func kw_to_hp(kw: float) -> float:
	"""Convert kilowatts to horsepower"""
	return kw / 0.7457


# Engine Torque Curve Calculation

func calculate_engine_torque(rpm: float) -> float:
	"""Calculate engine torque based on RPM using a simplified bell curve"""
	var normalized_rpm: float = clamp(rpm / redline_rpm, 0.0, 1.0)
	
	# Simple polynomial approximation of torque curve
	# Peak torque around 0.57 of redline (4000/7000)
	var torque_peak_pos: float = peak_torque_rpm / redline_rpm
	
	# Quadratic approximation centered at peak torque point
	var dist_from_peak: float = abs(normalized_rpm - torque_peak_pos)
	var torque_drop: float = dist_from_peak * dist_from_peak * 0.8
	
	var base_torque: float = max_engine_torque * (1.0 - torque_drop)
	
	# Apply some noise/smoothing near idle
	if rpm < idle_rpm:
		base_torque = min_engine_torque
		
	# Apply overrev penalty if past redline
	if rpm > redline_rpm:
		var overrev_factor: float = (rpm - redline_rpm) / (redline_rpm * 0.2)
		base_torque *= (1.0 - overrev_factor * overrev_penalty_factor)
	
	return clamp(base_torque, min_engine_torque, max_engine_torque)


# Aerodynamic Force Calculation

func calculate_drag_force(velocity_ms: float) -> float:
	"""Calculate aerodynamic drag force in Newtons"""
	var velocity_squared: float = velocity_ms * velocity_ms
	var drag_force: float = 0.5 * air_density * drag_coefficient * front_area * velocity_squared
	return drag_force

func calculate_lift_force(velocity_ms: float) -> float:
	"""Calculate aerodynamic lift force in Newtons (negative = downforce)"""
	var velocity_squared: float = velocity_ms * velocity_ms
	var lift_force: float = 0.5 * air_density * lift_coefficient * front_area * velocity_squared
	return -lift_force  # Negative because we want downforce for racing


# Tire Friction Model (Simplified Pacejka)

func calculate_longitudinal_friction(slips: float) -> float:
	"""Calculate longitudinal friction coefficient based on slip ratio"""
	slips = abs(clamp(slips, -2.0, 2.0))
	
	# Simplified Pacejka magic formula approximation
	# Fx = D * sin(C * atan(B * slips - E * (B * slips - atan(B * slips))))
	var B: float = longitudinal_slip_stiffness / 1e5  # stiffness factor
	var C: float = 1.9  # shape factor
	var D: float = tire_friction_horizontal  # peak friction
	var E: float = 0.9  # curvature factor
	
	var term: float = B * slips
	var sin_term: float = sin(C * atan(term - E * (term - atan(term))))
	
	return D * sin_term

func calculate_lateral_friction(sideslip: float) -> float:
	"""Calculate lateral friction coefficient based on sideslip angle"""
	sideslip = abs(clamp(sideslip, -PI / 2, PI / 2))
	
	# Simplified linear-to-curve transition
	var linear_region: float = 0.1  # radians where response is linear
	var max_friction: float = tire_friction_horizontal
	
	if sideslip < linear_region:
		return (sideslip / linear_region) * max_friction
	else:
		# Smooth transition to peak friction
		var normalized_slip: float = clamp((sideslip - linear_region) / (PI / 2 - linear_region), 0.0, 1.0)
		return lerp(max_friction * 0.8, max_friction, smoothstep(0.0, 0.5, normalized_slip))


# Suspension Force Calculation

func calculate_suspension_force(compression: float, velocity: float) -> float:
	"""Calculate total suspension force given compression and compression velocity"""
	var spring_force: float = -suspension_stiffness * compression
	var damping_force: float = -suspension_compression_damping * velocity if compression < 0 else -suspension_rebound_damping * velocity
	
	return spring_force + damping_force


# Gear Ratio Helper

func get_current_gear_ratio(current_gear: int) -> float:
	"""Get the effective gear ratio for current gear including final drive"""
	if current_gear == 0:
		return reverse_gear_ratio * final_drive_ratio
	elif current_gear >= 1 and current_gear <= gear_ratios.size():
		return gear_ratios[current_gear - 1] * final_drive_ratio
	else:
		return gear_ratios.min() * final_drive_ratio  # Use lowest gear as fallback


# Brake Force Distribution Helper

func get_front_brake_force(total_brake_force: float) -> float:
	"""Calculate front wheel brake force based on distribution"""
	return total_brake_force * brake_distribution_front

func get_rear_brake_force(total_brake_force: float) -> float:
	"""Calculate rear wheel brake force based on distribution"""
	return total_brake_force * brake_distribution_rear


# Utility Functions

func get_optimal_shift_rpm(current_gear: int, velocity_ms: float) -> float:
	"""Calculate optimal RPM for shifting up/down"""
	var gear_ratio: float = get_current_gear_ratio(current_gear)
	var wheel_angular_velocity: float = velocity_ms / default_wheel_radius
	var engine_rpm: float = wheel_angular_velocity * gear_ratio
	
	return engine_rpm + upshift_rpm_buffer if current_gear < gear_ratios.size() else engine_rpm - downshift_rpm_buffer

func is_vehicle_within_traction_limits(longitudinal_slip: float, lateral_slip: float) -> bool:
	"""Check if vehicle is operating within traction limits (circle criterion)"""
	var longitudinal_friction: float = abs(calculate_longitudinal_friction(longitudinal_slip))
	var lateral_friction: float = abs(calculate_lateral_friction(lateral_slip))
	
	# Circle criterion check
	var traction_circle: float = longitudinal_friction * longitudinal_friction + lateral_friction * lateral_friction
	var max_traction: float = tire_friction_horizontal * tire_friction_horizontal
	
	return traction_circle <= max_traction

func apply_weather_effect(frain_intensity: float) -> Dictionary:
	"""Apply weather effects to surface properties"""
	var weather_effects: Dictionary = {}
	
	if frain_intensity > 0:
		weather_effects.surface_friction_reduction = frain_intensity * rain_intensity_effect
		weather_effects.visibility_reduction = frain_intensity * 0.3
		weather_effects.engine_warmup_time = frain_intensity * 0.5
		weather_effects.brake_efficiency = 1.0 - (frain_intensity * 0.15)
	else:
		weather_effects.surface_friction_reduction = 0.0
		weather_effects.visibility_reduction = 0.0
		weather_effects.engine_warmup_time = 0.0
		weather_effects.brake_efficiency = 1.0
	
	return weather_effects


# Static getter accessors for common queries

static func get_gravity() -> float:
	return PhysicsSettings.gravity

static func get_default_vehicle_settings() -> Dictionary:
	return {
		"mass": PhysicsSettings.default_vehicle_mass,
		"wheel_radius": PhysicsSettings.default_wheel_radius,
		"wheel_width": PhysicsSettings.default_wheel_width,
		"wheel_mass": PhysicsSettings.default_wheel_mass,
		"wheel_inertia": PhysicsSettings.default_wheel_inertia
	}

static func get_track_surface_friction(surface_type: String) -> float:
	match surface_type:
		"asphalt":
			return PhysicsSettings.asphalt_friction
		"concrete":
			return PhysicsSettings.concrete_friction
		"gravel":
			return PhysicsSettings.gravel_friction
		"dirt":
			return PhysicsSettings.dirt_friction
		"wet_asphalt":
			return PhysicsSettings.wet_asphalt_friction
		"ice":
			return PhysicsSettings.ice_friction
		_:
			return PhysicsSettings.asphalt_friction  # Default to asphalt