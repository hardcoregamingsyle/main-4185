extends Resource
class_name PhysicsSettings

## PhysicsSettings - Centralized physics constants and configuration for the racing simulator
## All physics values are defined here so they can be tweaked without touching simulation code

@export_group("Global Physics Constants")
@export var gravity: float = 9.81
@export var physics_tick_rate: int = 120  # Fixed timestep Hz for physics simulation

@export_group("Vehicle Physics Defaults")
@export var default_vehicle_mass: float = 1500.0  # kg
@export var default_wheel_radius: float = 0.33  # meters (approx 17 inch wheel)
@export var default_wheel_width: float = 0.215  # meters
@export var default_wheel_mass: float = 25.0  # kg per wheel
@export var default_wheel_inertia: float = 1.2  # kg*m^2

@export_group("Suspension Defaults")
@export var default_spring_rate: float = 35000.0  # N/m
@export var default_damper_rate_compression: float = 3000.0  # Ns/m
@export var default_damper_rate_rebound: float = 4500.0  # Ns/m
@export var default_suspension_travel: float = 0.15  # meters
@export var default_anti_roll_bar_stiffness: float = 15000.0  # Nm/rad

@export_group("Tire Physics (Pacejka Magic Formula)")
@export var pacejka_bx: float = 10.0  # Stiffness factor
@export var pacejka_cx: float = 1.9  # Shape factor
@export var pacejka_dx: float = 1.0  # Peak factor
@export var pacejka_ex: float = 0.97  # Curvature factor
@export var pacejka_by: float = 10.0
@export var pacejka_cy: float = 1.9
@export var pacejka_dy: float = 1.0
@export var pacejka_ey: float = 0.97
@export var pacejka_bz: float = 10.0
@export var pacejka_cz: float = 1.9
@export var pacejka_dz: float = 1.0
@export var pacejka_ez: float = 0.97

@export var tire_load_sensitivity: float = 0.8  # How much grip changes with load
@export var tire_pressure_base: float = 2.2  # bar
@export var tire_temp_optimal: float = 90.0  # Celsius
@export var tire_temp_range: float = 40.0  # Celsius range for good grip
@export var tire_wear_rate: float = 0.0001  # Per meter driven

@export_group("Drivetrain Defaults")
@export var default_engine_redline_rpm: float = 7000.0
@export var default_engine_idle_rpm: float = 800.0
@export var default_engine_max_torque: float = 400.0  # Nm
@export var default_engine_max_power_rpm: float = 5500.0
@export var default_final_drive_ratio: float = 3.5
@export var default_gear_ratios: Array[float] = [3.5, 2.2, 1.5, 1.1, 0.85, 0.7]
@export var default_clutch_strength: float = 500.0  # Nm
@export var default_lsd_lockup: float = 0.3  # 0=open, 1=locked

@export_group("Aerodynamics Defaults")
@export var default_drag_coefficient: float = 0.32
@export var default_frontal_area: float = 2.2  # m^2
@export var default_lift_coefficient: float = 0.1
@export var default_downforce_coefficient: float = 0.8
@export var air_density: float = 1.225  # kg/m^3 at sea level

@export_group("Brake Defaults")
@export var default_brake_force_front: float = 15000.0  # Nm
@export var default_brake_force_rear: float = 10000.0  # Nm
@export var default_brake_bias: float = 0.65  # Front bias

@export_group("Steering Defaults")
@export var default_steering_lock: float = 0.6  # radians (~34 degrees)
@export var default_steering_ratio: float = 15.0  # Steering wheel : wheel angle
@export var default_ackermann_factor: float = 0.3

@export_group("Surface Friction Modifiers")
@export var surface_friction: Dictionary = {
	"asphalt": 1.0,
	"asphalt_wet": 0.7,
	"concrete": 0.95,
	"gravel": 0.6,
	"dirt": 0.5,
	"sand": 0.4,
	"grass": 0.35,
	"mud": 0.3,
	"ice": 0.1,
	"snow": 0.2,
	"curb": 0.8,
	"rumble_strip": 0.75
}

@export_group("Damage System")
@export var damage_threshold_impulse: float = 5000.0  # Ns
@export var damage_multiplier: float = 0.001
@export var max_engine_damage: float = 1.0
@export var max_transmission_damage: float = 1.0
@export var max_suspension_damage: float = 1.0
@export var max_tire_damage: float = 1.0
@export var max_aero_damage: float = 1.0

@export_group("FFB (Force Feedback) Settings")
@export var ffb_gain: float = 1.0
@export var ffb_damping: float = 0.1
@export var ffb_friction: float = 0.05
@export var ffb_min_force: float = 0.05
@export var ffb_max_force: float = 1.0
@export var ffb_center_spring: float = 0.2
@export var ffb_dynamic_damping: float = 0.15

@export_group("Advanced Physics")
@export var enable_load_transfer: bool = true
@export var enable_tire_flex: bool = true
@export var enable_chassis_flex: bool = false
@export var enable_thermal_model: bool = true
@export var enable_wear_model: bool = true
@export var enable_damage_model: bool = true

func _init() -> void:
	pass

func get_surface_friction(surface_name: String) -> float:
	return surface_friction.get(surface_name, 1.0)

func calculate_drag_force(velocity: Vector3, drag_coeff: float, frontal_area: float) -> Vector3:
	var speed = velocity.length()
	if speed < 0.1:
		return Vector3.ZERO
	var drag_magnitude = 0.5 * air_density * drag_coeff * frontal_area * speed * speed
	return -velocity.normalized() * drag_magnitude

func calculate_downforce(velocity: Vector3, downforce_coeff: float, frontal_area: float) -> float:
	var speed = velocity.length()
	return 0.5 * air_density * downforce_coeff * frontal_area * speed * speed

func pacejka_formula(slip: float, B: float, C: float, D: float, E: float) -> float:
	## Pacejka Magic Formula: y = D * sin(C * arctan(B * x - E * (B * x - arctan(B * x))))
	var bx = B * slip
	var arctan_bx = atan(bx)
	return D * sin(C * arctan(bx - E * (bx - arctan_bx)))

func calculate_tire_forces(slip_angle: float, slip_ratio: float, normal_load: float, 
		camber_angle: float = 0.0, tire_pressure: float = 2.2, tire_temp: float = 90.0,
		surface_grip: float = 1.0) -> Dictionary:
	## Calculate lateral (Fy) and longitudinal (Fx) forces using Pacejka
	## Also returns aligning torque (Mz)
	
	# Load sensitivity
	var load_factor = pow(normal_load / (default_vehicle_mass * gravity / 4.0), tire_load_sensitivity)
	
	# Temperature factor
	var temp_factor = 1.0 - abs(tire_temp - tire_temp_optimal) / tire_temp_range
	temp_factor = clamp(temp_factor, 0.5, 1.0)
	
	# Pressure factor
	var pressure_factor = 1.0 - abs(tire_pressure - tire_pressure_base) * 0.1
	pressure_factor = clamp(pressure_factor, 0.8, 1.1)
	
	# Combined grip factor
	var grip_factor = surface_grip * load_factor * temp_factor * pressure_factor
	
	# Longitudinal force (slip ratio)
	var Fx = pacejka_formula(slip_ratio, pacejka_bx, pacejka_cx, pacejka_dx * grip_factor, pacejka_ex) * normal_load
	
	# Lateral force (slip angle)
	var Fy = pacejka_formula(slip_angle, pacejka_by, pacejka_cy, pacejka_dy * grip_factor, pacejka_ey) * normal_load
	
	# Aligning torque (simplified)
	var Mz = pacejka_formula(slip_angle, pacejka_bz, pacejka_cz, pacejka_dz * grip_factor * 0.1, pacejka_ez) * normal_load * default_wheel_radius
	
	# Combined slip reduction (friction circle)
	var combined_slip = sqrt(slip_ratio * slip_ratio + (slip_angle * 0.1) * (slip_angle * 0.1))
	if combined_slip > 0.1:
		var reduction = 1.0 / (1.0 + combined_slip * 5.0)
		Fx *= reduction
		Fy *= reduction
		Mz *= reduction
	
	return {
		"Fx": Fx,
		"Fy": Fy,
		"Mz": Mz,
		"grip_factor": grip_factor
	}

func calculate_engine_torque(rpm: float, throttle: float, max_torque: float, max_torque_rpm: float, redline: float) -> float:
	## Simplified engine torque curve
	if rpm < default_engine_idle_rpm:
		return 0.0
	if rpm > redline:
		return 0.0
	
	# Normalized RPM (0-1)
	var rpm_norm = (rpm - default_engine_idle_rpm) / (redline - default_engine_idle_rpm)
	var torque_rpm_norm = (max_torque_rpm - default_engine_idle_rpm) / (redline - default_engine_idle_rpm)
	
	# Torque curve: rises to peak, then falls
	var torque_curve = 0.0
	if rpm_norm <= torque_rpm_norm:
		# Rising portion
		torque_curve = sin(rpm_norm / torque_rpm_norm * PI * 0.5)
	else:
		# Falling portion
		var fall_ratio = (rpm_norm - torque_rpm_norm) / (1.0 - torque_rpm_norm)
		torque_curve = cos(fall_ratio * PI * 0.5)
	
	return max_torque * torque_curve * throttle

func calculate_power_from_torque(torque: float, rpm: float) -> float:
	## Power (Watts) = Torque (Nm) * Angular Velocity (rad/s)
	return torque * (rpm * 2.0 * PI / 60.0)

func rpm_from_wheel_speed(wheel_angular_vel: float, gear_ratio: float, final_drive: float) -> float:
	## Convert wheel angular velocity to engine RPM
	return abs(wheel_angular_vel) * gear_ratio * final_drive * 60.0 / (2.0 * PI)

func wheel_speed_from_rpm(rpm: float, gear_ratio: float, final_drive: float) -> float:
	## Convert engine RPM to wheel angular velocity
	return rpm * 2.0 * PI / 60.0 / (gear_ratio * final_drive)

func calculate_suspension_force(spring_rate: float, damper_rate: float, 
		compression: float, compression_vel: float, 
		travel_limit: float) -> float:
	## Spring-damper force with travel limiting
	var spring_force = spring_rate * compression
	var damper_force = 0.0
	
	if compression_vel > 0:
		damper_force = damper_rate * compression_vel  # Compression
	else:
		damper_force = damper_rate * 1.5 * compression_vel  # Rebound (higher damping)
	
	# Progressive bump stop
	var total_force = spring_force + damper_force
	if compression > travel_limit * 0.8:
		var bump_factor = (compression - travel_limit * 0.8) / (travel_limit * 0.2)
		total_force += spring_rate * bump_factor * bump_factor * 10.0
	
	return total_force