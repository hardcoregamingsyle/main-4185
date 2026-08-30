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
@export var center_of_mass_x: float = 0.0
@export var center_of_mass_y: float = 0.3  # height above ground
@export var center_of_mass_z: float = 0.0
@export var wheelbase: float = 2.7  # meters distance between front and rear axles
@export var track_width_front: float = 1.6  # meters
@export var track_width_rear: float = 1.65  # meters
@export var chassis_height: float = 0.45  # meters from ground to roof
@export var ground_clearance: float = 0.12  # meters lowest point clearance
@export var roll_center_height: float = 0.15  # meters
@export var pitch_motion_ratio: float = 1.0

@export_group("Differential Settings")
@export var differential_type: int = 0  # 0=Open, 1=LSD, 2=Coupe LSD, 3=Locked
@export var lsd_diff_limit: float = 2000.0  # Nm locking torque
@export var lsd_preload: float = 200.0  # Nm preload torque
@export var lsd_acceleration_lock: float = 1500.0  # Nm acceleration lock
@export var lsd_deceleration_lock: float = 800.0  # Nm deceleration lock
@export var limited_slip_effectiveness: float = 0.5  # 0-1 effectiveness factor
@export var open_diff_behavior: float = 1.0  # Open diff behavior multiplier

@export_group("Steering System")
@export var steering_ratio: float = 14.0  # ratio of steering wheel to wheel angle
@export var max_steering_angle: float = 30.0  # degrees maximum wheel angle
@export var steering_speed: float = 45.0  # degrees per second
@export var steering_weight_curve: PackedFloat32Array = [0.3, 0.5, 0.7, 0.9, 1.2]  # speed vs weight curve points
@export var steering_center_return: float = 1.5  # N*m return force at center
@export var steering_hard_stop: float = 0.1  # radians hard stop buffer
@export var steering_noise_amplitude: float = 0.02  # radians noise amplitude
@export var steering_noise_frequency: float = 5.0  # Hz noise frequency

@export_group("Powertrain Performance")
@export var max_engine_torque: float = 450.0  # Nm
@export var max_engine_power: float = 250.0  # kW
@export var power_curve_smoothness: float = 0.8  # curve interpolation smoothness
@export var torque_curve_points: PackedFloat32Array = [
	0.0, 0.3, 0.6, 0.9, 1.1, 1.2, 1.15, 1.05, 0.9, 0.75, 0.55, 0.35, 0.15
]
@export var rpm_curve_points: PackedFloat32Array = [
	0.0, 1000.0, 2000.0, 3000.0, 4000.0, 5000.0, 6000.0, 7000.0, 8000.0, 9000.0, 10000.0, 11000.0, 12000.0
]
@export var fuel_consumption_rate: float = 0.08  # liters per km at full throttle
@export var fuel_tank_capacity: float = 60.0  # liters
@export var ignition_delay: float = 0.02  # seconds delay after throttle input
@export var fuel_cutoff_rpm: float = 7200.0  # RPM where fuel cuts off

@export_group("Collision & Damage")
@export var collision_margin: float = 0.05  # meters collision buffer
@export var damage_threshold: float = 50.0  # impact force threshold for damage
@export var structural_integrity: float = 100.0  # percent starting integrity
@export var damage_decay_rate: float = 0.01  # health regeneration rate
@export var crumple_zone_front: float = 0.5  # meters front crumple zone
@export var crumple_zone_rear: float = 0.4  # meters rear crumple zone
@export var rollover_threshold: float = 60.0  # degrees tilt for rollover risk
@export var debris_spawn_force: float = 500.0  # force applied to spawned debris

@export_group("Environmental Factors")
@export var road_surface_friction: float = 0.9  # dry asphalt coefficient
@export var wet_surface_friction: float = 0.6  # wet asphalt coefficient
@export var ice_surface_friction: float = 0.15  # ice coefficient
@export var gravel_surface_friction: float = 0.5  # loose gravel
@export var grass_surface_friction: float = 0.35  # short grass
@export var mud_surface_friction: float = 0.25  # thick mud
@export var temperature_factor: float = 1.0  # affects tire grip (1.0 = optimal temp)
@export var track_wear_factor: float = 0.95  # grip degradation per lap (1.0 = no wear)

@export_group("Simulation Accuracy")
@export var substep_integration: int = 2  # integration method: 0=Euler, 1=RK2, 2=RK4
@export var contact_tolerance: float = 0.001  # meters for contact detection
@export var sleep_threshold: float = 0.01  # velocity below this triggers sleep
@export var wake_velocity: float = 0.05  # velocity that wakes sleeping objects
@export var collision_priority: int = 100  # collision layer priority
@export var raycast_max_distance: float = 10.0  # max raycast distance for terrain
@export var contact_point_count: int = 8  # max contact points per wheel

@export_group("AI Behavior Tuning")
@export var ai_thinking_interval: float = 0.1  # seconds between AI decisions
@export var ai_line_smoothing: float = 0.7  # how much AI follows ideal line
@export var ai_aggression: float = 0.8  # 0-1 aggression level
@export var ai_pace_compensation: float = 1.1  # AI pace multiplier over player
@export var ai_reaction_delay: float = 0.15  # seconds delay before reacting
@export var ai_path_follow_error: float = 0.5  # meters path following tolerance

func _init() -> void:
	_setup_default_curves()

func _setup_default_curves() -> void:
	"""Initialize default parameter curves if empty"""
	if torque_curve_points.is_empty():
		torque_curve_points = PackedFloat32Array([0.0, 0.3, 0.6, 0.9, 1.1, 1.2, 1.15, 1.05, 0.9, 0.75, 0.55, 0.35, 0.15])
	if rpm_curve_points.is_empty():
		rpm_curve_points = PackedFloat32Array([0.0, 1000.0, 2000.0, 3000.0, 4000.0, 5000.0, 6000.0, 7000.0, 8000.0, 9000.0, 10000.0, 11000.0, 12000.0])

## Unit Conversion Helpers

func rpm_to_rad_per_sec(rpm: float) -> float:
	"""Convert RPM to radians per second"""
	return rpm * TAU / 60.0

func rad_per_sec_to_rpm(rad_per_sec: float) -> float:
	"""Convert radians per second to RPM"""
	return rad_per_sec * 60.0 / TAU

func mph_to_mps(mph: float) -> float:
	"""Convert miles per hour to meters per second"""
	return mph * 0.44704

func mps_to_mph(mps: float) -> float:
	"""Convert meters per second to miles per hour"""
	return mps / 0.44704

func kmh_to_mps(kmh: float) -> float:
	"""Convert kilometers per hour to meters per second"""
	return kmh / 3.6

func mps_to_kmh(mps: float) -> float:
	"""Convert meters per second to kilometers per hour"""
	return mps * 3.6

func ft_lbs_to_nm(ft_lbs: float) -> float:
	"""Convert foot-pounds to Newton-meters"""
	return ft_lbs * 1.3558179483314004

def nm_to_ft_lbs(nm: float) -> float:
	"""Convert Newton-meters to foot-pounds"""
	return nm / 1.3558179483314004

func lb_to_kg(lb: float) -> float:
	"""Convert pounds to kilograms"""
	return lb * 0.45359237

func kg_to_lb(kg: float) -> float:
	"""Convert kilograms to pounds"""
	return kg / 0.45359237

func psi_to_bar(psi: float) -> float:
	"""Convert PSI to bar"""
	return psi * 0.06894757

func bar_to_psi(bar: float) -> float:
	"""Convert bar to PSI"""
	return bar / 0.06894757

func inches_to_mm(inches: float) -> float:
	"""Convert inches to millimeters"""
	return inches * 25.4

func mm_to_inches(mm: float) -> float:
	"""Convert millimeters to inches"""
	return mm / 25.4

## Tire Friction Calculations

func get_longitudinal_friction(slip_ratio: float, surface_friction: float = 1.0) -> float:
	"""Calculate longitudinal friction force based on slip ratio using simplified Pacejka"""
	var alpha = abs(slip_ratio)
	var C = 1.9
	var B = 10.0
	var E = 1.0
	
	# Simplified Magic Formula
	var friction = sin(B * atan(E * alpha)) / max(alpha, 0.0001)
	return clamp(friction * surface_friction, 0.0, 1.5)

func get_lateral_friction(slip_angle: float, surface_friction: float = 1.0) -> float:
	"""Calculate lateral friction force based on slip angle"""
	var alpha = deg_to_rad(abs(slip_angle))
	var C = 1.3
	var B = 10.0
	var E = 1.0
	
	# Simplified Magic Formula for lateral force
	var friction = sin(B * atan(E * alpha))
	return clamp(friction * surface_friction, 0.0, 1.5)

func calculate_pacejka_coefficient(A: float, B: float, C: float, D: float, E: float, x: float) -> float:
	"""Calculate Pacejka 'Magic Formula' output"""
	var BxC = B * C
	var BCx = BxC * x
	var result = D * sin(C * atan(Bx - E * (Bx - atan(Bx))))
	return result

## Aerodynamic Force Calculations

func calculate_drag_force(velocity: float, drag_coeff: float = 0.32, area: float = 2.2) -> Vector3:
	"""Calculate aerodynamic drag force vector"""
	var speed = velocity.length()
	var density = air_density
	
	# F_drag = 0.5 * rho * v^2 * Cd * A
	var drag_magnitude = 0.5 * density * speed * speed * drag_coeff * area
	
	return -velocity.normalized() * drag_magnitude

func calculate_lift_force(velocity: float, lift_coeff: float = 0.15, area: float = 2.2) -> Vector3:
	"""Calculate aerodynamic lift/downforce vector"""
	var speed = velocity.length()
	var density = air_density
	
	# F_lift = 0.5 * rho * v^2 * Cl * A
	var lift_magnitude = 0.5 * density * speed * speed * lift_coeff * area
	
	# Lift is typically downward for race cars (negative Y)
	return Vector3.UP * (-lift_magnitude)

func calculate_downforce_at_speed(speed_kmh: float) -> float:
	"""Calculate total downforce at given speed in km/h"""
	var speed_mps = kmh_to_mps(speed_kmh)
	var velocity = Vector3.DOWN * speed_mps
	
	var aero_downforce = calculate_lift_force(velocity, -drag_coefficient * 0.5, front_area).y
	return aero_downforce

## Vehicle Dynamics Helpers

func calculate_weight_transfer(acceleration: float, wheelbase: float = 2.7, cg_height: float = 0.3) -> float:
	"""Calculate longitudinal weight transfer during acceleration/deceleration"""
	var mass = default_vehicle_mass
	var g = gravity
	
	# Delta_F = (mass * a * h_cog) / wheelbase
	return (mass * acceleration * cg_height) / wheelbase

func calculate_cornering_weight_transfer(lateral_acc: float, track_width: float = 1.6, cg_height: float = 0.3) -> float:
	"""Calculate lateral weight transfer during cornering"""
	var mass = default_vehicle_mass
	var g = gravity
	
	# Delta_F = (mass * lateral_acc * h_cog) / track_width
	return (mass * lateral_acc * cg_height) / track_width

func get_available_grip(surface: String, temp_factor: float = 1.0) -> float:
	"""Get base grip coefficient for different surfaces"""
	match surface:
		"asphalt": return road_surface_friction * temp_factor
		"wet_asphalt": return wet_surface_friction * temp_factor
		"ice": return ice_surface_friction * temp_factor
		"gravel": return gravel_surface_friction * temp_factor
		"grass": return grass_surface_friction * temp_factor
		"mud": return mud_surface_friction * temp_factor
		_: return road_surface_friction * temp_factor

func get_engine_torque_at_rpm(rpm: float) -> float:
	"""Get engine torque value at given RPM using curve lookup"""
	if rpm <= 0.0 or rpm >= rpm_curve_points[rpm_curve_points.size() - 1]:
		return 0.0
	
	# Linear interpolation through curve points
	var index = 0
	for i in range(rpm_curve_points.size() - 1):
		if rpm >= rpm_curve_points[i] and rpm < rpm_curve_points[i + 1]:
			index = i
			break
	
	var t = (rpm - rpm_curve_points[index]) / (rpm_curve_points[index + 1] - rpm_curve_points[index])
	var torque_low = torque_curve_points[index]
	var torque_high = torque_curve_points[index + 1]
	
	var interpolated = torque_low + t * (torque_high - torque_low)
	return interpolated * max_engine_torque

func get_optimal_shift_point(gear: int) -> float:
	"""Get optimal RPM for shifting up from given gear"""
	if gear >= gear_ratios.size():
		return redline_rpm
	return automatic_shift_rpm

func get_brake_distribution(front_percent: float = 0.6) -> Dictionary:
	"""Get brake force distribution between front and rear wheels"""
	var front_force = brake_force_per_wheel * front_percent
	var rear_force = brake_force_per_wheel * (1.0 - front_percent)
	
	return {
		"front": front_force,
		"rear": rear_force,
		"ratio": front_percent / (1.0 - front_percent)
	}

func validate_vehicle_settings(vehicle_mass: float, wheelbase: float, track_width: float) -> bool:
	"""Validate that vehicle settings are physically reasonable"""
	var valid = true
	
	# Mass validation
	if vehicle_mass < 800.0 or vehicle_mass > 2500.0:
		valid = false
	
	# Wheelbase validation
	if wheelbase < 2.0 or wheelbase > 4.0:
		valid = false
	
	# Track width validation
	if track_width < 1.3 or track_width > 2.0:
		valid = false
	
	# Center of mass height validation
	if center_of_mass_y < 0.2 or center_of_mass_y > 0.6:
		valid = false
	
	return valid

func clone_with_modifications(modifications: Dictionary) -> PhysicsSettings:
	"""Create a new instance with specific modifications"""
	var new_settings = duplicate()
	
	for key in modifications:
		if new_settings.has(key):
			set(key, modifications[key])
	
	return new_settings

## Debug & Profiling

func print_statistics() -> void:
	"""Print comprehensive physics statistics to console"""
	print("\n=== Physics Settings Statistics ===")
	print("Vehicle Mass: %.1f kg" % default_vehicle_mass)
	print("Wheel Radius: %.2f m (%.1f inches)" % [default_wheel_radius, default_wheel_radius * 39.3701])
	print("Gear Ratios: %s" % gear_ratios)
	print("Final Drive: %.2f:1" % final_drive_ratio)
	print("Max Torque: %.1f Nm (%.1f ft-lbs)" % [max_engine_torque, ft_lbs_to_nm(max_engine_torque)])
	print("Max Power: %.1f kW (%.1f hp)" % [max_engine_power, max_engine_power * 1.34102])
	print("Redline: %.0f RPM" % redline_rpm)
	print("Brake Bias Front: %.0f%%" % (brake_bias_front * 100))
	print("Aero Drag Coefficient: %.2f" % drag_coefficient)
	print("Air Density: %.3f kg/m³" % air_density)
	print("===================================\n")

func get_simulation_time_step() -> float:
	"""Get actual time step used for physics simulation"""
	return 1.0 / physics_tick_rate

func get_maximum_framerate() -> int:
	"""Get theoretical maximum framerate for physics updates"""
	return physics_tick_rate * max_substeps

</function_calls>