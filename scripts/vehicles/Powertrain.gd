extends Resource
class_name Powertrain

## Powertrain - Realistic engine powertrain simulation
## All vehicle dynamics derived from physical parameters, not direct speed control
## Implements: RPM, torque curves, horsepower, transmission, drivetrain losses

@export_group("Engine Core Parameters")
@export var displacement: float = 4000.0  # cc (cubic centimeters)
@export var max_rpm: float = 8500.0  # Maximum engine RPM
@export var idle_rpm: float = 800.0  # Idle engine speed
@export var redline: float = 7500.0  # RPM where rev limiter engages
@export var compression_ratio: float = 11.5  # Compression ratio
@export var cylinder_count: int = 6  # Number of cylinders
@export var firing_order: PackedStringArray = ["1", "4", "3", "6", "2", "5"]  # Firing order

@export_group("Torque Characteristics")
@export var peak_torque: float = 450.0  # Nm at peak
@export var torque_peak_rpm: float = 4200.0  # RPM where peak torque occurs
@export var torque_curve_type: String = "linear"  # linear, exponential, custom

@export_group("Horsepower & Power")
@export var peak_horsepower: float = 400.0  # HP at peak
@export var hp_peak_rpm: float = 6500.0  # RPM where peak HP occurs
@export var flywheel_efficiency: float = 0.95  # Flywheel energy storage efficiency

@export_group("Drivetrain Configuration")
@export var drivetrain_type: String = "rwd"  # rwd, fwd, awd
@export var final_drive_ratio: float = 3.73  # Final drive gear ratio
@export var differential_type: String = "open"  # open, limited_slip, locking
@export var drivetrain_loss: float = 0.12  # 12% drivetrain loss

@export_group("Transmission")
@export var transmission_type: String = "manual"  # manual, automatic, cvt
@export var gear_ratios: Dictionary = {
	"1": 3.67,
	"2": 2.18,
	"3": 1.52,
	"4": 1.17,
	"5": 0.96,
	"6": 0.81,
	"r": 3.42
}
@export var shift_points: Array[float] = [5500.0, 5800.0, 6200.0, 6500.0, 6800.0, 7000.0]

@export_group("Wheel & Tire Configuration")
@export var wheel_diameter: float = 0.66  # meters (approx 26 inch tire + rim)
@export var tire_width: float = 0.26  # meters
@export var tire_pressure_front: float = 220000.0  # Pascals (2.2 bar)
@export var tire_pressure_rear: float = 240000.0  # Pascals (2.4 bar)
@export var tire_grip_coefficient: float = 1.1  # Friction coefficient

@export_group("Aerodynamics")
@export var drag_coefficient: float = 0.29  # Cd
@export var frontal_area: float = 2.2  # m^2
@export var downforce_coefficient: float = 0.8  # Downforce per velocity squared
@export var wing_angle: float = 5.0  # degrees

# Runtime state
var current_rpm: float = 0.0
var current_gear: int = 0
var clutch_engaged: bool = true
var throttle_input: float = 0.0  # 0.0 to 1.0
var brake_input: float = 0.0  # 0.0 to 1.0
var vehicle_speed: float = 0.0  # m/s

var _torque_curve: Array[Vector2] = []
var _hp_curve: Array[Vector2] = []
var _inertia: float = 0.0  # Total rotational inertia

func _init() -> void:
	_generate_torque_curve()
	_generate_hp_curve()
	_calculate_inertia()

func _generate_torque_curve() -> void:
	"""Generate torque curve across RPM range"""
	var rpm_samples: int = 100
	_torque_curve.clear()
	
	for i in range(rpm_samples):
		var rpm = lerp(idle_rpm, redline, float(i) / (rpm_samples - 1))
		var torque: float = 0.0
		
		if torque_curve_type == "linear":
			torque = _calculate_linear_torque(rpm)
		elif torque_curve_type == "exponential":
			torque = _calculate_exponential_torque(rpm)
		else:
			torque = _calculate_custom_torque(rpm)
		
		_torque_curve.push_back(Vector2(rpm, torque))

func _calculate_linear_torque(rpm: float) -> float:
	"""Linear rise to peak torque, then gradual decline"""
	var torque_dropoff: float = 0.0
	
	if rpm <= torque_peak_rpm:
		# Linear ramp up to peak
		var slope = peak_torque / torque_peak_rpm
		torque = slope * rpm
	else:
		# Gradual decline after peak
		var drop_range = max_rpm - torque_peak_rpm
		var drop_progress = (rpm - torque_peak_rpm) / drop_range
		torque = peak_torque * (1.0 - drop_progress * 0.4)  # 40% drop to redline
	
	return clamp(torque, 0.0, peak_torque * 1.1)

func _calculate_exponential_torque(rpm: float) -> float:
	"""Exponential curve similar to high-revving engines"""
	var torque_factor: float = 1.0
	
	if rpm < idle_rpm:
		torque_factor = 0.1
	elif rpm <= torque_peak_rpm:
		torque_factor = pow(float(rpm) / idle_rpm, 1.5)
	else:
		torque_factor = pow(1.0, 1.5) * exp(-0.0001 * (rpm - torque_peak_rpm))
	
	return peak_torque * torque_factor

func _calculate_custom_torque(rpm: float) -> float:
	"""Custom curve with more flat torque band"""
	var rpm_normalized: float = (rpm - idle_rpm) / (max_rpm - idle_rpm)
	
	var low_end: float = 0.85 if rpm_normalized < 0.3 else 1.0
	var mid_band: float = 1.0 if rpm_normalized > 0.3 && rpm_normalized < 0.7 else 0.9
	var high_end: float = 1.0 - (rpm_normalized - 0.7) * 0.3 if rpm_normalized > 0.7 else 1.0
	
	return peak_torque * low_end * mid_band * high_end

func _generate_hp_curve() -> void:
	"""Generate horsepower curve from torque (HP = Torque * RPM / 5252)"""
	_hp_curve.clear()
	
	for sample in _torque_curve:
		var rpm = sample.x
		var torque = sample.y
		var hp = torque * rpm / 5252.0  # Standard conversion formula
		_hp_curve.push_back(Vector2(rpm, hp))

func _calculate_inertia() -> void:
	"""Calculate total rotational inertia of powertrain"""
	# Engine rotating mass inertia
	var engine_inertia: float = 0.02 * displacement / 1000.0  # kg*m^2
	# Transmission inertia
	var trans_inertia: float = 0.01 * cylinder_count  # kg*m^2
	# Driveshaft and differential
	var driveshaft_inertia: float = 0.005 * cylinder_count  # kg*m^2
	
	_inertia = engine_inertia + trans_inertia + driveshaft_inertia

func get_torque_at_rpm(rpm: float) -> float:
	"""Get torque output at specific RPM"""
	rpm = clamp(rpm, 0.0, max_rpm)
	
	for i in range(_torque_curve.size() - 1):
		if rpm >= _torque_curve[i].x and rpm <= _torque_curve[i+1].x:
			var t0 = _torque_curve[i]
			var t1 = _torque_curve[i+1]
			
			var alpha = (rpm - t0.x) / (t1.x - t0.x)
			return lerp(t0.y, t1.y, alpha)
	
	return _torque_curve.back().y

func get_horsepower_at_rpm(rpm: float) -> float:
	"""Get horsepower output at specific RPM"""
	rpm = clamp(rpm, 0.0, max_rpm)
	
	for i in range(_hp_curve.size() - 1):
		if rpm >= _hp_curve[i].x and rpm <= _hp_curve[i+1].x:
			var t0 = _hp_curve[i]
			var t1 = _hp_curve[i+1]
			
			var alpha = (rpm - t0.x) / (t1.x - t0.x)
			return lerp(t0.y, t1.y, alpha)
	
	return _hp_curve.back().y

func calculate_wheel_speed(rpm: float, gear: int) -> float:
	"""Calculate vehicle wheel RPM based on engine RPM and gear"""
	var gear_ratio: float = gear_ratios[str(gear)]
	var wheel_rpm = rpm / (gear_ratio * final_drive_ratio)
	
	return wheel_rpm

func calculate_vehicle_speed(wheel_rpm: float) -> float:
	"""Calculate vehicle speed in m/s from wheel RPM"""
	var wheel_circumference: float = PI * wheel_diameter
	var wheel_rps: float = wheel_rpm / 60.0  # Convert RPM to RPS
	var speed: float = wheel_rps * wheel_circumference
	
	return speed

func update(dt: float) -> void:
	"""Update powertrain physics simulation"""
	if not clutch_engaged:
		current_rpm = lerp(current_rpm, idle_rpm, dt * 5.0)
		return
	
	# Calculate target RPM based on throttle and current gear
	var target_rpm: float = _calculate_target_rpm()
	
	# Apply engine acceleration/deceleration based on load
	var engine_accel: float = _calculate_engine_accel(target_rpm, dt)
	current_rpm += engine_accel * dt
	
	# Rev limiter protection
	if current_rpm >= redline:
		current_rpm = redline - (current_rpm - redline) * 0.1
	
	# Clutch slip handling
	if not clutch_engaged:
		current_rpm = lerp(current_rpm, idle_rpm, dt * 3.0)

func _calculate_target_rpm() -> float:
	"""Calculate target RPM based on input and gear"""
	var gear_ratio: float = gear_ratios[str(current_gear)]
	var wheel_rps: float = vehicle_speed / (PI * wheel_diameter)
	var driven_rpm: float = wheel_rps * 60.0 * gear_ratio * final_drive_ratio
	
	if throttle_input > 0.0:
		return lerp(driven_rpm, current_rpm + throttle_input * (max_rpm - idle_rpm), 0.3)
	else:
		return lerp(driven_rpm, idle_rpm, 0.5)

func _calculate_engine_accel(target_rpm: float, dt: float) -> float:
	"""Calculate engine angular acceleration"""
	var rpm_diff: float = target_rpm - current_rpm
	
	# Engine inertia affects acceleration rate
	var inertia_factor: float = 1.0 / (_inertia * 100.0)
	
	# Throttle affects acceleration magnitude
	var throttle_factor: float = throttle_input * 0.5 + 0.5
	
	# Load factor (braking/engine braking reduces acceleration)
	var load_factor: float = 1.0 - brake_input * 0.3
	
	var accel: float = rpm_diff * inertia_factor * throttle_factor * load_factor
	
	return accel

func apply_throttle(throttle_val: float) -> void:
	throttle_input = clamp(throttle_val, 0.0, 1.0)

func apply_brake(brake_val: float) -> void:
	brake_input = clamp(brake_val, 0.0, 1.0)

func set_clutch_state(engaged: bool) -> void:
	clutch_engaged = engaged

func change_gear(gear: int) -> void:
	"""Change transmission gear"""
	if gear in gear_ratries.keys():
		current_gear = gear
		# Brief RPM drop during upshift, rise during downshift
		if gear > current_gear:
			current_rpm *= 0.7  # Upshift RPM drop
		else:
			current_rpm *= 1.3  # Downshift RPM rise

func get_drivetrain_output_torque() -> float:
	"""Calculate actual torque delivered to wheels"""
	var engine_torque: float = get_torque_at_rpm(current_rpm)
	var gear_ratio: float = gear_ratios[str(current_gear)]
	var wheel_torque: float = engine_torque * gear_ratio * final_drive_ratio
	var drivetrain_torque: float = wheel_torque * (1.0 - drivetrain_loss)
	
	return drivetrain_torque

func calculate_aerodynamic_drag(speed_mps: float) -> float:
	"""Calculate aerodynamic drag force at given speed"""
	var air_density: float = 1.225  # kg/m^3 at sea level
	var drag_force: float = 0.5 * air_density * drag_coefficient * frontal_area * speed_mps * speed_mps
	
	return drag_force

func calculate_downforce(speed_mps: float) -> float:
	"""Calculate aerodynamic downforce at given speed"""
	var air_density: float = 1.225
	var downforce: float = 0.5 * air_density * downforce_coefficient * frontal_area * speed_mps * speed_mps
	
	return downforce

func reset() -> void:
	current_rpm = idle_rpm
	current_gear = 0
 clutch_engaged = true
 throttle_input = 0.0
 brake_input = 0.0
 vehicle_speed = 0.0

func serialize() -> Dictionary:
	return {
		"displacement": displacement,
		"max_rpm": max_rpm,
		"peak_torque": peak_torque,
		"peak_horsepower": peak_horsepower,
		"drivetrain_type": drivetrain_type,
		"final_drive_ratio": final_drive_ratio,
		"wheel_diameter": wheel_diameter,
		"drag_coefficient": drag_coefficient,
		"current_rpm": current_rpm,
		"current_gear": current_gear
	}

func deserialize(data: Dictionary) -> void:
	displacement = data.get("displacement", displacement)
	max_rpm = data.get("max_rpm", max_rpm)
	peak_torque = data.get("peak_torque", peak_torque)
	peak_horsepower = data.get("peak_horsepower", peak_horsepower)
	drivetrain_type = data.get("drivetrain_type", drivetrain_type)
	final_drive_ratio = data.get("final_drive_ratio", final_drive_ratio)
	wheel_diameter = data.get("wheel_diameter", wheel_diameter)
	drag_coefficient = data.get("drag_coefficient", drag_coefficient)
	current_rpm = data.get("current_rpm", current_rpm)
	current_gear = data.get("current_gear", current_gear)
	_generate_torque_curve()
	_generate_hp_curve()