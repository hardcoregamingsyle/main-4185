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
@export var center_of_mass_x: float = 0.5  # normalized from front
@export var center_of_mass_y: float = 0.35  # normalized height
@export var center_of_mass_z: float = 0.0
@export var roll_center_height: float = 0.15
@export var anti_roll_bar_front: float = 5000.0
@export var anti_roll_bar_rear: float = 4000.0

# Pre-calculated constants
var _gear_ratios_with_final: Array[float] = []
var _wheel_positions: Array[Vector3] = []
var _vehicle_dimensions: Vector3 = Vector3(4.5, 1.5, 2.0)

func _init() -> void:
	_calculate_gear_ratios()
	_setup_wheel_positions()

func _calculate_gear_ratios() -> void:
	_gear_ratios_with_final = []
	for ratio in gear_ratios:
		_gear_ratios_with_final.append(ratio * final_drive_ratio)

func _setup_wheel_positions() -> void:
	_wheel_positions = [
		Vector3(1.5, 0.0, 1.0),   # Front Left
		Vector3(1.5, 0.0, -1.0),  # Front Right
		Vector3(-1.5, 0.0, 1.0),  # Rear Left
		Vector3(-1.5, 0.0, -1.0)  # Rear Right
	]

func get_wheel_position(index: int) -> Vector3:
	return _wheel_positions[index] if index < 4 else Vector3.ZERO

func get_total_gear_ratio(gear_index: int) -> float:
	if gear_index < _gear_ratios_with_final.size():
		return _gear_ratios_with_final[gear_index]
	return _gear_ratios_with_final.back()

func calculate_aerodynamic_drag(speed: float) -> float:
	var speed_sq = speed * speed
	return 0.5 * air_density * drag_coefficient * front_area * speed_sq

func calculate_aerodynamic_lift(speed: float) -> float:
	var speed_sq = speed * speed
	return 0.5 * air_density * lift_coefficient * front_area * speed_sq

func calculate_engine_torque(rpm: float, throttle: float) -> float:
	# Simple torque curve approximation
	var rpm_normalized = (rpm - idle_rpm) / (redline_rpm - idle_rpm)
	rpm_normalized = clamp(rpm_normalized, 0.0, 1.0)
	
	var torque_curve = sin(rpm_normalized * PI)
	torque_curve += 0.2 * sin(rpm_normalized * PI * 2)
	torque_curve *= 1.2
	
	return torque_curve * throttle * 400.0  # Peak ~400 Nm

func calculate_horsepower(rpm: float, torque: float) -> float:
	return (torque * rpm) / 7127.0  # HP conversion factor

func get_optimal_shift_point(current_rpm: float) -> float:
	return minf(redline_rpm * 0.9, peak_horsepower_rpm * 1.1)

func get_brake_force_percentage(brake_input: float) -> float:
	return clampf(brake_input, 0.0, 1.0) * brake_force_per_wheel

func reset() -> void:
	pass
</FILE>