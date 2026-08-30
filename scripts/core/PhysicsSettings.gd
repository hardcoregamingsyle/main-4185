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
@export var peak_torque_nm: float = 450.0  # Nm at peak torque rpm
@export var max_engine_braking: float = 0.8  # Engine braking multiplier

@export_group("Gear Ratios")
@export var final_drive_ratio: float = 3.5
@export var first_gear_ratio: float = 3.8
@export var second_gear_ratio: float = 2.5
@export var third_gear_ratio: float = 1.8
@export var fourth_gear_ratio: float = 1.35
@export var fifth_gear_ratio: float = 1.0
@export var sixth_gear_ratio: float = 0.85
@export var reverse_gear_ratio: float = 3.5

@export_group("Differential Settings")
@export var differential_type: int = DifferentialType.LSD  # LSD=1, Open=0, Locked=2
@export var lsd_preload_torque: float = 50.0  # Nm preload torque
@export var lsd_acceleration_lock: float = 0.3  # Acceleration lock ratio (0-1)
@export var lsd_deceleration_lock: float = 0.2  # Deceleration lock ratio (0-1)
@export var open_diff_max_slip: float = 0.5  # Max slip before open diff locks
@export var locked_diff_lock_force: float = 1000.0  # Nm locking force

@export_group("Brake System")
@export var brake_force_per_pedal: float = 15000.0  # N max brake force per wheel
@export var brake_bias_front: float = 0.6  # Front brake bias (0-1)
@export var brake_pressure_ratio: float = 10.0  # Master cylinder pressure ratio
@export var abs_threshold: float = 0.15  # ABS activation threshold
@export var abs_recovery_speed: float = 10.0  # ABS recovery speed

@export_group("Steering System")
@export var steering_ratio: float = 15.0  # Steering ratio (degrees wheel : degrees wheel turn)
@export var max_steering_angle: float = 30.0  # Max wheel angle in degrees
@export var steering_speed: float = 45.0  # Degrees per second max steering speed
@export var steering_return_speed: float = 20.0  # Degrees per second return speed
@export var steering_friction: float = 0.1  # Steering friction coefficient
@export var power_steering_assist: float = 0.5  # Power steering assist (0-1)

@export_group("Transmission")
@export var clutch_engagement_time: float = 0.15  # Seconds to fully engage clutch
@export var clutch_disengagement_time: float = 0.1  # Seconds to disengage
@export var transmission_shift_delay: float = 0.1  # Delay after shift command
@export var automatic_shifting_enabled: bool = true
@export var manual_shifting_enabled: bool = true

@export_group("Track Physics")
@export var track_roughness: float = 0.05  # Track surface roughness
@export var track_temperature_base: float = 20.0  # Base track temperature Celsius
@export var track_temp_variation: float = 15.0  # Temperature variation during race
@export var weather_effects_enabled: bool = true
@export var rain_friction_reduction: float = 0.4  # Friction reduction in rain
@export var snow_friction_reduction: float = 0.2  # Friction reduction in snow

@export_group("Collision Detection")
@export var collision_margin: float = 0.01  # Collision detection margin in meters
@export var penetration_tolerance: float = 0.005  # Maximum allowed penetration
@export var contact_point_offset: float = 0.02  # Offset for contact point generation

@export_group("Simulation Quality")
@export var high_quality_physics: bool = false
@export var accurate_aerodynamics: bool = false
@export var detailed_suspension: bool = false
@export var tire_heat_model: bool = true
@export var tire_deformation_model: bool = false

enum DifferentialType {
	OPEN,
	LSD,
	LOCKED
}

enum TireCompound {
	HARD,
	MEDIUM,
	SOFT,
	RAIN,
	SNOW
}

# Predefined tire compound configurations
var tire_compounds: Dictionary = {
	TireCompound.HARD: {
		"friction": 1.0,
		"temperature_min": 50.0,
		"temperature_max": 120.0,
		"optimal_temp": 80.0,
		"degradation_rate": 0.5
	},
	TireCompound.MEDIUM: {
		"friction": 1.1,
		"temperature_min": 40.0,
		"temperature_max": 100.0,
		"optimal_temp": 70.0,
		"degradation_rate": 1.0
	},
	TireCompound.SOFT: {
		"friction": 1.25,
		"temperature_min": 30.0,
		"temperature_max": 90.0,
		"optical_temp": 60.0,
		"degradation_rate": 2.0
	},
	TireCompound.RAIN: {
		"friction": 0.6,
		"temperature_min": 10.0,
		"temperature_max": 60.0,
		"optimal_temp": 35.0,
		"degradation_rate": 0.3
	},
	TireCompound.SNOW: {
		"friction": 0.3,
		"temperature_min": -20.0,
		"temperature_max": 10.0,
		"optimal_temp": -5.0,
		"degradation_rate": 0.1
	}
}

## Helper methods for unit conversions

func kmh_to_ms(speed_kmh: float) -> float:
	return speed_kmh / 3.6

func ms_to_kmh(speed_ms: float) -> float:
	return speed_ms * 3.6

func deg_to_rad(angle_deg: float) -> float:
	return angle_deg * PI / 180.0

func rad_to_deg(angle_rad: float) -> float:
	return angle_rad * 180.0 / PI

func rpm_to_frequency(rpm: float) -> float:
	return rpm / 60.0

func frequency_to_rpm(frequency_hz: float) -> float:
	return frequency_hz * 60.0

func nm_to_lb_ft(torque_nm: float) -> float:
	return torque_nm * 0.737562

func lb_ft_to_nm(torque_lb_ft: float) -> float:
	return torque_lb_ft * 1.35582

func inches_to_meters(length_inches: float) -> float:
	return length_inches * 0.0254

def meters_to_inches(length_meters: float) -> float:
	return length_meters / 0.0254

func psi_to_bar(pressure_psi: float) -> float:
	return pressure_psi * 0.0689476

func bar_to_psi(pressure_bar: float) -> float:
	return pressure_bar / 0.0689476

func hp_to_kw(power_hp: float) -> float:
	return power_hp * 0.7457

func kw_to_hp(power_kw: float) -> float:
	return power_kw / 0.7457

## Calculate engine torque based on RPM using a simplified curve
func get_engine_torque(rpm: float) -> float:
	var normalized_rpm: float = clamp((rpm - idle_rpm) / (redline_rpm - idle_rpm), 0.0, 1.0)
	
	# Simplified torque curve - peak at peak_torque_rpm
	if rpm <= peak_torque_rpm:
		var ramp_up: float = (rpm - idle_rpm) / (peak_torque_rpm - idle_rpm)
		return peak_torque_nm * (0.7 + 0.3 * ramp_up)
	else:
		var ramp_down: float = (rpm - peak_torque_rpm) / (redline_rpm - peak_torque_rpm)
		return peak_torque_nm * (1.0 - 0.3 * ramp_down)

## Calculate drag force on vehicle
func calculate_drag_force(velocity_ms: float) -> float:
	var velocity_sq: float = abs(velocity_ms) * abs(velocity_ms)
	return 0.5 * air_density * drag_coefficient * front_area * velocity_sq

## Calculate downforce on vehicle
func calculate_downforce(velocity_ms: float) -> float:
	var velocity_sq: float = abs(velocity_ms) * abs(velocity_ms)
	return 0.5 * air_density * lift_coefficient * front_area * velocity_sq

## Get current gear based on RPM
func get_current_gear(rpm: float) -> int:
	if rpm < idle_rpm:
		return -1  # Reverse
	elif rpm < 1500:
		return 1
	elif rpm < 3000:
		return 2
	elif rpm < 4500:
		return 3
	elif rpm < 5500:
		return 4
	elif rpm < 6200:
		return 5
	else:
		return 6

## Get gear ratio for current gear
func get_gear_ratio(gear: int) -> float:
	match gear:
		1: return first_gear_ratio
		2: return second_gear_ratio
		3: return third_gear_ratio
		4: return fourth_gear_ratio
		5: return fifth_gear_ratio
		6: return sixth_gear_ratio
		-1: return reverse_gear_ratio
		_: return 1.0

## Calculate optimal shift point for acceleration
func get_optimal_shift_rpm() -> float:
	return redline_rpm * 0.95

## Calculate fuel consumption rate (L/hour)
func calculate_fuel_consumption(engine_load: float, rpm: float) -> float:
	var base_consumption: float = 5.0  # L/h at idle
	var load_factor: float = max(1.0, engine_load * 3.0)
	var rpm_factor: float = max(1.0, rpm / 3000.0)
	return base_consumption * load_factor * rpm_factor

## Reset all physics settings to defaults
func reset_to_defaults() -> void:
	gravity = 9.81
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

## Load preset configuration
func load_preset(preset_name: String) -> void:
	match preset_name:
		"simulator":
			high_quality_physics = true
			accurate_aerodynamics = true
			detailed_suspension = true
			tire_heat_model = true
		"arcade":
			high_quality_physics = false
			accurate_aerodynamics = false
			detailed_suspension = false
			tire_heat_model = false
			drift_multiplier = 2.0
		"realistic":
			high_quality_physics = true
			accurate_aerodynamics = true
			detailed_suspension = true
			tire_heat_model = true
			tire_deformation_model = true
		_:
			push_warning("Unknown preset: " + preset_name)

## Validate all physics settings
func validate_settings() -> Array[String]:
	var warnings: Array[String] = []
	
	if gravity < 0:
		warnings.append("Gravity cannot be negative")
	if default_vehicle_mass <= 0:
		warnings.append("Vehicle mass must be positive")
	if default_wheel_radius <= 0:
		warnings.append("Wheel radius must be positive")
	if suspension_stiffness <= 0:
		warnings.append("Suspension stiffness must be positive")
	if tire_friction_horizontal <= 0:
		warnings.append("Tire friction must be positive")
	if redline_rpm <= idle_rpm:
		warnings.append("Redline must be higher than idle RPM")
	if max_steering_angle < 0 or max_steering_angle > 60:
		warnings.append("Steering angle should be between 0 and 60 degrees")
	if brake_bias_front < 0 or brake_bias_front > 1:
		warnings.append("Brake bias must be between 0 and 1")
	
	return warnings
</file_content>