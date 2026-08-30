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
@export var center_of_mass_x: float = 0.0  # relative to chassis origin
@export var center_of_mass_y: float = 0.5  # height above ground
@export var center_of_mass_z: float = 0.0  # lateral offset
@export var roll_center_height: float = 0.15  # meters
@export var track_width: float = 1.6  # meters between wheels
@export var wheelbase: float = 2.7  # meters between axles

@export_group("Steering System")
@export var steering_lock_degrees: float = 30.0  # maximum lock angle per side
@export var steering_ratio: float = 15.0  # ratio of steering wheel to wheel turn
@export var steering_response_time: float = 0.1  # seconds to reach full lock
@export var steering_sensitivity: float = 1.0  # multiplier for input sensitivity

@export_group("Differential Settings")
@export var diff_type: DiffType = DiffType.OPEN  # type of differential
@export var diff_preload: float = 200.0  # Nm preload torque
@export var diff_max_locked: float = 0.9  # maximum locking percentage (0-1)
@export var diff_open_ratio: float = 0.2  # torque split for open diff
@export var diff_ramp_up: float = 0.5  # acceleration ramp rate
@export var diff_ramp_down: float = 0.5  # deceleration ramp rate

enum DiffType {
	OPEN,
	LIMITED_SLIP,
	LOCKER
}

@export_group("Gear Shift Logic")
@export var shift_time: float = 0.15  # seconds for gear change
@export var rev_match_delay: float = 0.05  # delay before rev-matching
@export var downshift_rpm_threshold: float = 5500.0  # rpm for forced downshift
@export var upshift_rpm_threshold: float = 6800.0  # rpm for forced upshift

@export_group("Collision Physics")
@export var collision_margin: float = 0.05  # meters for AABB padding
@export var collision_sleep_tolerance: float = 0.01  # velocity below which sleep
@export var collision_velocity_threshold: float = 0.5  # m/s for wake-up
@export var restitution_default: float = 0.1  # bounciness coefficient

@export_group("Track Surface Properties")
@export var asphalt_friction_base: float = 1.1
@export var asphalt_friction_wet: float = 0.8
@export var grass_friction_base: float = 0.4
@export var gravel_friction_base: float = 0.3
@export var ice_friction_base: float = 0.15
@export var curb_friction: float = 0.6

func _init() -> void:
	"""Initialize physics settings with validated defaults."""
	_validate_settings()

func _validate_settings() -> void:
	"""Ensure all physics values are within reasonable ranges."""
	if gravity <= 0:
		gravity = 9.81
	if physics_tick_rate < 30:
		physics_tick_rate = 120
	if max_substeps < 1:
		max_substeps = 4
	if default_vehicle_mass <= 0:
		default_vehicle_mass = 1500.0
	if default_wheel_radius <= 0:
		default_wheel_radius = 0.33
	if suspension_stiffness <= 0:
		suspension_stiffness = 35000.0
	if suspension_compression_damping <= 0:
		suspension_compression_damping = 1200.0
	if suspension_rebound_damping <= 0:
		suspension_rebound_damping = 600.0
	if max_suspension_travel <= 0:
		max_suspension_travel = 0.25
	if slip_threshold < 0 or slip_threshold > 1:
		sl ip_threshold = 0.15
	if air_density <= 0:
		air_density = 1.225
	if idle_rpm >= redline_rpm:
		idle_rpm = 800.0
		redline_rpm = 7000.0
	if gear_ratios.is_empty():
		gear_ratios = PackedFloat32Array([4.0, 2.5, 1.8, 1.3, 1.0, 0.8])
	if brake_bias_front < 0 or brake_bias_front > 1:
		brake_bias_front = 0.6
	if steering_lock_degrees < 0 or steering_lock_degrees > 60:
		steering_lock_degrees = 30.0
	if steering_ratio <= 0:
		steering_ratio = 15.0
	if diff_max_locked < 0 or diff_max_locked > 1:
		diff_max_locked = 0.9
	if shift_time <= 0:
		shift_time = 0.15
	if diff_preload < 0:
		diff_preload = 200.0
	if collision_margin < 0:
		collision_margin = 0.05
	if track_width <= 0:
		track_width = 1.6
	if wheelbase <= 0:
		wheelbase = 2.7
	if center_of_mass_y <= 0:
		center_of_mass_y = 0.5
	if rear_area <= 0:
		rear_area = 2.2
	if diff_ramp_up < 0:
		diff_ramp_up = 0.5
	if diff_ramp_down < 0:
		diff_ramp_down = 0.5
	if shift_time < 0:
		shift_time = 0.15
	if rev_match_delay < 0:
		rev_match_delay = 0.05
	if downshift_rpm_threshold > redline_rpm:
		downshift_rpm_threshold = 5500.0
	if upshift_rpm_threshold > redline_rpm:
		upshift_rpm_threshold = 6800.0
	if collision_sleep_tolerance < 0:
		collision_sleep_tolerance = 0.01
	if collision_velocity_threshold < 0:
		collision_velocity_threshold = 0.5
	if restitution_default < 0 or restitution_default > 1:
		restitution_default = 0.1
	if asphalt_friction_base <= 0:
		asphalt_friction_base = 1.1
	if asphalt_friction_wet <= 0:
		asphalt_friction_wet = 0.8
	if grass_friction_base <= 0:
		grass_friction_base = 0.4
	if gravel_friction_base <= 0:
		gravel_friction_base = 0.3
	if ice_friction_base <= 0:
		ice_friction_base = 0.15
	if curb_friction <= 0:
		curb_friction = 0.6

# ============================================================================
# HELPER METHODS - UNIT CONVERSIONS
# ============================================================================

func rpm_to_rad_per_sec(rpm: float) -> float:
	"""Convert RPM to radians per second."""
	return rpm * PI / 30.0

func rad_per_sec_to_rpm(rad_per_sec: float) -> float:
	"""Convert radians per second to RPM."""
	return rad_per_sec * 30.0 / PI

func kmh_to_ms(kmh: float) -> float:
	"""Convert kilometers per hour to meters per second."""
	return kmh / 3.6

func ms_to_kmh(ms: float) -> float:
	"""Convert meters per second to kilometers per hour."""
	return ms * 3.6

func deg_to_rad(degrees: float) -> float:
	"""Convert degrees to radians."""
	return degrees * PI / 180.0

func rad_to_deg(radians: float) -> float:
	"""Convert radians to degrees."""
	return radians * 180.0 / PI

func mm_to_m(mm: float) -> float:
	"""Convert millimeters to meters."""
	return mm / 1000.0

func m_to_mm(m: float) -> float:
	"""Convert meters to millimeters."""
	return m * 1000.0

func newton_to_pounds(force_n: float) -> float:
	"""Convert Newtons to pounds-force."""
	return force_n * 0.224809

func pound_to_newton(force_lbs: float) -> float:
	"""Convert pounds-force to Newtons."""
	return force_lbs * 4.44822

func inch_to_meter(inch: float) -> float:
	"""Convert inches to meters."""
	return inch * 0.0254

def meter_to_inch(m: float) -> float:
	"""Convert meters to inches."""
	return m / 0.0254

func kg_to_lb(mass_kg: float) -> float:
	"""Convert kilograms to pounds."""
	return mass_kg * 2.20462

func lb_to_kg(mass_lb: float) -> float:
	"""Convert pounds to kilograms."""
	return mass_lb / 2.20462

# ============================================================================
# HELPERS - PHYSICS CALCULATIONS
# ============================================================================

func calculate_aero_drag(velocity: Vector3) -> Vector3:
	"""Calculate aerodynamic drag force vector.
	
	Args:
		velocity: Vehicle velocity vector in world space (m/s)
		
	Returns:
		Drag force vector opposing motion
	"""
	var speed = velocity.length()
	if speed <= 0:
		return Vector3.ZERO
	
	# Drag equation: F = 0.5 * rho * Cd * A * v^2
	var drag_magnitude = 0.5 * air_density * drag_coefficient * front_area * speed * speed
	
	# Apply drag direction opposite to velocity
	var drag_direction = velocity.normalized()
	return -drag_direction * drag_magnitude

func calculate_aero_lift(velocity: Vector3) -> float:
	"""Calculate aerodynamic lift force magnitude.
	
	Args:
		velocity: Vehicle velocity vector in world space (m/s)
		
	Returns:
		Lift force magnitude (positive = upward)
	"""
	var speed = velocity.length()
	if speed <= 0:
		return 0.0
	
	# Lift equation: L = 0.5 * rho * Cl * A * v^2
	return 0.5 * air_density * lift_coefficient * front_area * speed * speed

func get_gear_ratio(gear_index: int) -> float:
	"""Get gear ratio for a specific gear index.
	
	Args:
		gear_index: Gear number (0 = reverse, 1-6 = forward gears)
		
	Returns:
		Gear ratio value
	"""
	if gear_index == 0:
		return -gear_ratios.size()  # Reverse uses negative ratio
	elif gear_index >= 1 and gear_index <= gear_ratios.size():
		return gear_ratios[gear_index - 1]
	else:
		return gear_ratios.back()  # Default to highest gear

func get_current_gear_ratio(current_gear: int) -> float:
	"""Calculate overall gear ratio including final drive.
	
	Args:
		current_gear: Current gear index (1-based, 0 = neutral)
		
	Returns:
		Overall drivetrain ratio
	"""
	if current_gear <= 0:
		return 0.0
	var gear_idx = clamp(current_gear - 1, 0, gear_ratios.size() - 1)
	return gear_ratios[gear_idx] * final_drive_ratio

func calculate_wheel_torque(engine_torque: float, gear_ratio: float) -> float:
	"""Calculate torque delivered to wheels after transmission.
	
	Args:
		engine_torque: Engine torque in Nm
		gear_ratio: Overall gear ratio
		
	Returns:
		Wheel torque in Nm
	"""
	return engine_torque * gear_ratio * 0.98  # 2% drivetrain loss

func calculate_engine_braking_torque(engine_rpm: float, gear_ratio: float) -> float:
	"""Calculate engine braking torque based on RPM.
	
	Args:
		engine_rpm: Current engine RPM
		gear_ratio: Overall gear ratio
		
	Returns:
		Braking torque in Nm
	"""
	if engine_rpm < idle_rpm:
		return 0.0
	var rpm_excess = engine_rpm - idle_rpm
	var max_braking = redline_rpm - idle_rpm
	var braking_factor = min(rpm_excess / max_braking, 1.0) * engine_braking_factor
	return calculate_wheel_torque(100.0 * braking_factor, gear_ratio)  # Base 100Nm reference

func calculate_suspension_force(compression: float, velocity: float) -> float:
	"""Calculate suspension force using spring-damper model.
	
	Args:
		compression: Spring compression in meters (negative = extension)
		velocity: Suspension velocity in m/s
		
	Returns:
		Force in Newtons
	"""
	# Spring force: F = k * x
	var spring_force = suspension_stiffness * compression
	
	# Damping force: F = c * v
	var damping_force = compression < 0:
		suspension_compression_damping * velocity
		:
		suspension_rebound_damping * velocity
	
	return spring_force + damping_force

func calculate_tire_friction(normal_force: float, slip_ratio: float, slip_angle: float, surface_friction: float) -> Vector2:
	"""Calculate tire friction forces using simplified Pacejka-like model.
	
	Args:
		normal_force: Normal load on tire in Newtons
		slip_ratio: Longitudinal slip ratio (-1 to 1)
		slip_angle: Slip angle in radians
		surface_friction: Surface friction coefficient
		
	Returns:
		Vector2 containing [longitudinal_force, lateral_force]
	"""
	# Simplified friction circle model
	var max_longitudinal = normal_force * surface_friction * tire_friction_horizontal
	var max_lateral = normal_force * surface_friction * tire_friction_vertical
	
	var combined_slip = sqrt(slip_ratio * slip_ratio + slip_angle * slip_angle)
	var slip_limit = min(max_longitudinal, max_lateral)
	
	# Longitudinal force
	var longitudinal_force = 0.0
	if abs(slip_ratio) > 0:
		longitudinal_force = sign(slip_ratio) * min(abs(slip_ratio) * longitudinal_slip_stiffness * normal_force / 1000.0, max_longitudinal)
	
	# Lateral force
	var lateral_force = 0.0
	if abs(slip_angle) > 0:
		lateral_force = sign(slip_angle) * min(abs(slip_angle) * lateral_slip_stiffness * normal_force / 1000.0, max_lateral)
	
	# Enforce friction circle
	var combined = sqrt(longitudinal_force * longitudinal_force + lateral_force * lateral_force)
	if combined > slip_limit:
		var scale = slip_limit / combined
		longitudinal_force *= scale
		lateral_force *= scale
	
	return Vector2(longitudinal_force, lateral_force)

func calculate_max_cornering_force(normal_load: float, surface_friction: float) -> float:
	"""Calculate maximum lateral cornering force.
	
	Args:
		normal_load: Normal load on tire in Newtons
		surface_friction: Surface friction coefficient
		
	Returns:
		Max lateral force in Newtons
	"""
	return normal_load * surface_friction * tire_friction_horizontal

func calculate_max_traction_force(normal_load: float, surface_friction: float) -> float:
	"""Calculate maximum traction (acceleration) force.
	
	Args:
		normal_load: Normal load on tire in Newtons
		surface_friction: Surface friction coefficient
		
	Returns:
		Max traction force in Newtons
	"""
	return normal_load * surface_friction * tire_friction_horizontal

func get_surface_friction(surface_type: String) -> float:
	"""Get friction coefficient for different surfaces.
	
	Args:
		surface_type: Type of surface ("asphalt", "wet_asphalt", "grass", "gravel", "ice", "curb")
		
	Returns:
		Friction coefficient
	"""
	match surface_type:
		"asphalt": return asphalt_friction_base
		"wet_asphalt": return asphalt_friction_wet
		"grass": return grass_friction_base
		"gravel": return gravel_friction_base
		"ice": return ice_friction_base
		"curb": return curb_friction
		_: return asphalt_friction_base  # Default to asphalt

func calculate_brake_force(total_brake_input: float, current_gear: int, is_abs_active: bool) -> float:
	"""Calculate total brake force applied.
	
	Args:
		total_brake_input: Brake pedal position (0-1)
		current_gear: Current gear (0 = neutral)
		is_abs_active: Whether ABS is active
		
	Returns:
		Total brake force in Newtons
	"""
	var base_force = brake_force_per_wheel * total_brake_input
	
	# Reduce braking in neutral to prevent over-braking
	if current_gear <= 0:
		base_force *= 0.5
	
	# ABS modulation
	if is_abs_active and total_brake_input > abs_threshold:
		base_force *= abs_threshold
	
	return base_force

func get_steering_angle(input_value: float) -> float:
	"""Calculate wheel steering angle from input.
	
	Args:
		input_value: Steering input (-1 to 1)
		
	Returns:
		Steering angle in degrees
	"""
	var target_angle = input_value * steering_lock_degrees
	return target_angle

func calculate_diff_torque_distribution(left_torque: float, right_torque: float) -> Vector2f:
	"""Calculate torque distribution across differential.
	
	Args:
		left_torque: Left wheel torque request
		right_torque: Right wheel torque request
		
	Returns:
		Vector2 with [left_output, right_output]
	"""
	var total = left_torque + right_torque
	if total <= 0:
		return Vector2f.ZERO
	
	var left_share = 0.5 if diff_type == DiffType.OPEN else diff_open_ratio
	var right_share = 1.0 - left_share
	
	# Limited-slip adjustment
	if diff_type == DiffType.LIMITED_SLIP:
		var diff = abs(left_torque - right_torque)
		var locked_amount = min(diff * diff_max_locked, diff)
		left_share += locked_amount / total * 0.5
		right_share += locked_amount / total * 0.5
	
	# Clamp shares
	left_share = clamp(left_share, 0.0, 1.0)
	right_share = clamp(right_share, 0.0, 1.0)
	
	return Vector2f(left_share * total, right_share * total)

func calculate_center_of_mass_offset(chassis_rotation: Quaternion) -> Vector3:
	"""Calculate center of mass position accounting for chassis rotation.
	
	Args:
		chassis_rotation: Chassis orientation quaternion
		
	Returns:
		COM position in world space
	"""
	var com_local = Vector3(center_of_mass_x, center_of_mass_y, center_of_mass_z)
	return chassis_rotation.xform(com_local)

func get_wheel_positions(chassis_origin: Vector3, chassis_rotation: Quaternion) -> Array[Vector3]:
	"""Calculate positions of all four wheels.
	
	Args:
		chassis_origin: Chassis position in world space
		chassis_rotation: Chassis orientation quaternion
		
	Returns:
		Array of 4 wheel positions [FL, FR, RL, RR]
	"""
	var half_track = track_width / 2.0
	var half_wheelbase = wheelbase / 2.0
	
	var fl_pos = Vector3(-half_wheelbase, 0, -half_track)
	var fr_pos = Vector3(-half_wheelbase, 0, half_track)
	var rl_pos = Vector3(half_wheelbase, 0, -half_track)
	var rr_pos = Vector3(half_wheelbase, 0, half_track)
	
	# Transform to world space
	fl_pos = chassis_rotation.xfl(frm(fl_pos)) + chassis_origin
	fr_pos = chassis_rotation.xfr(fr_pos) + chassis_origin
	rl_pos = chassis_rotation.xrl(rl_pos) + chassis_origin
	rr_pos = chassis_rotation.xrr(rr_pos) + chassis_origin
	
	return [fl_pos, fr_pos, rl_pos, rr_pos]

func clone() -> PhysicsSettings:
	"""Create a deep copy of this settings resource."""
	var copy = PhysicsSettings.new()
	copy.gravity = gravity
	copy.physics_tick_rate = physics_tick_rate
	copy.max_substeps = max_substeps
	copy.default_vehicle_mass = default_vehicle_mass
	copy.default_wheel_radius = default_wheel_radius
	copy.default_wheel_width = default_wheel_width
	copy.default_wheel_mass = default_wheel_mass
	copy.default_wheel_inertia = default_wheel_inertia
	copy.suspension_stiffness = suspension_stiffness
	copy.suspension_compression_damping = suspension_compression_damping
	copy.suspension_rebound_damping = suspension_rebound_damping
	copy.max_suspension_travel = max_suspension_travel
	copy.spring_rest_length = spring_rest_length
	copy.tire_friction_horizontal = tire_friction_horizontal
	copy.tire_friction_vertical = tire_friction_vertical
	copy.slip_threshold = slip_threshold
	copy.lateral_slip_stiffness = lateral_slip_stiffness
	copy.longitudinal_slip_stiffness = longitudinal_slip_stiffness
	copy.drift_multiplier = drift_multiplier
	copy.drift_recovery_speed = drift_recovery_speed
	copy.minimum_drift_angle = minimum_drift_angle
	copy.maximum_drift_angle = maximum_drift_angle
	copy.air_density = air_density
	copy.drag_coefficient = drag_coefficient
	copy.lift_coefficient = lift_coefficient
	copy.front_area = front_area
	copy.center_of_pressure_x = center_of_pressure_x
	copy.center_of_pressure_y = center_of_pressure_y
	copy.center_of_pressure_z = center_of_pressure_z
	copy.idle_rpm = idle_rpm
	copy.redline_rpm = redline_rpm
	copy.peak_torque_rpm = peak_torque_rpm
	copy.peak_horsepower_rpm = peak_horsepower_rpm
	copy.throttle_response_time = throttle_response_time
	copy.engine_braking_factor = engine_braking_factor
	copy.gear_ratios = gear_ratios.duplicate()
	copy.final_drive_ratio = final_drive_ratio
	copy.clutch_engagement_time = clutch_engagement_time
	copy.automatic_shift_rpm = automatic_shift_rpm
	copy.automatic_downshift_rpm = automatic_downshift_rpm
	copy.brake_force_per_wheel = brake_force_per_wheel
	copy.abs_threshold = abs_threshold
	copy.brake_bias_front = brake_bias_front
	copy.brake_bleed_rate = brake_bleed_rate
	copy.parking_brake_force = parking_brake_force
	copy.center_of_mass_x = center_of_mass_x
	copy.center_of_mass_y = center_of_mass_y
	copy.center_of_mass_z = center_of_mass_z
	copy.roll_center_height = roll_center_height
	copy.track_width = track_width
	copy.wheelbase = wheelbase
	copy.steering_lock_degrees = steering_lock_degrees
	copy.steering_ratio = steering_ratio
	copy.steering_response_time = steering_response_time
	copy.steering_sensitivity = steering_sensitivity
	copy.diff_type = diff_type
	copy.diff_preload = diff_preload
	copy.diff_max_locked = diff_max_locked
	copy.diff_open_ratio = diff_open_ratio
	copy.diff_ramp_up = diff_ramp_up
	copy.diff_ramp_down = diff_ramp_down
	copy.shift_time = shift_time
	copy.rev_match_delay = rev_match_delay
	copy.downshift_rpm_threshold = downshift_rpm_threshold
	copy.upshift_rpm_threshold = upshift_rpm_threshold
	copy.collision_margin = collision_margin
	copy.collision_sleep_tolerance = collision_sleep_tolerance
	copy.collision_velocity_threshold = collision_velocity_threshold
	copy.restitution_default = restitution_default
	copy.asphalt_friction_base = asphalt_friction_base
	copy.asphalt_friction_wet = asphalt_friction_wet
	copy.grass_friction_base = grass_friction_base
	copy.gravel_friction_base = gravel_friction_base
	copy.ice_friction_base = ice_friction_base
	copy.curb_friction = curb_friction
	
	return copy

static func create_default() -> PhysicsSettings:
	"""Create a new PhysicsSettings instance with default values."""
	return PhysicsSettings.new()

</script>