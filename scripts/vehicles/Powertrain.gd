extends Resource
class_name Powertrain

## Powertrain - Advanced engine physics simulation
## Calculates vehicle speed from RPM, torque, horsepower, gear ratios, and wheel diameter
## Implements realistic engine behavior including redline, idle, torque curves, and transmission

signal rpm_changed(current_rpm: float, max_rpm: float)
signal gear_changed(old_gear: int, new_gear: int)
signal clutch_engaged()
signal clutch_disengaged()
signal throttle_changed(throttle_percent: float)

@export_group("Engine Specifications")
@export var idle_rpm: float = 800.0          # Idle RPM
@export var max_rpm: float = 7500.0         # Maximum safe RPM (redline)
@export var peak_torque_rpm: float = 4500.0 # RPM where peak torque occurs
@export var peak_power_rpm: float = 6500.0  # RPM where peak power occurs
@export var peak_torque_nm: float = 450.0   # Peak torque in Newton-meters
@export var peak_power_kw: float = 250.0    # Peak power in kilowatts
@export var torque_curve_slope: float = 0.7 # How quickly torque falls off after peak

@export_group("Transmission")
@export var num_gears: int = 6              # Number of forward gears
@export var reverse_ratio: float = 3.5      # Reverse gear ratio
@export var final_drive_ratio: float = 3.73 # Final drive differential ratio
var _gear_ratios: Array[float] = []         # Per-gear ratios

@export_group("Wheel & Drivetrain")
@export var wheel_diameter_m: float = 0.66  # Wheel diameter in meters (including tire)
@export var drivetrain_type: DrivetrainType = DrivetrainType.AWD
@export var drivetrain_efficiency: float = 0.85  # Power loss through drivetrain
@export var differential_type: DifferentialType = DifferentialType.OPEN_DIFF

@export_group("Clutch & Throttle")
@export var clutch_slip_threshold: float = 0.15  # Clutch engagement threshold
@export var throttle_response_time: float = 0.1  # How fast throttle responds

@export_group("Aerodynamics")
@export var drag_coeffent_cd: float = 0.29     # Aerodynamic drag coefficient
@export var frontal_area_m2: float = 2.1       # Frontal area in square meters
@export var downforce_coefficient: float = 0.5 # Downforce vs lift coefficient

enum DrivetrainType { RWD, FWD, AWD }
enum DifferentialType { OPEN_DIFF, LSD, LIMITED_SLIP }

# Runtime state
var current_rpm: float = 0.0
var current_gear: int = 0                      # 0 = neutral, 1-6 = forward gears, -1 = reverse
var throttle_input: float = 0.0                # 0.0 to 1.0
var clutch_engaged: bool = true
var vehicle_speed_ms: float = 0.0              # Current vehicle speed in m/s
var wheel_angular_velocity: float = 0.0        # Wheel rotation in rad/s
var output_torque_nm: float = 0.0              # Torque at wheels
var engine_braking: bool = false

func _init() -> void:
	_init_gear_ratios()

func _init_gear_ratios() -> void:
	"""Initialize typical gear ratios for performance vehicles"""
	var base_ratio: float = 3.5
	for i in range(num_gears):
		var gear_index = i + 1
		# Progressive gearing: lower gears have higher ratios
		_gear_ratios.append(base_ratio / pow(1.3, gear_index - 1))

func get_wheel_radius_m() -> float:
	return wheel_diameter_m * 0.5

func get_circumference_m() -> float:
	return PI * wheel_diameter_m

func calculate_engine_braking_factor() -> float:
	"""Calculate engine braking force based on RPM vs throttle"""
	if engine_braking:
		var rpm_ratio = current_rpm / max_rpm
		return clamp(rpm_ratio * 0.3, 0.0, 0.3)
	return 0.0

func update(dt: float, throttle: float, brake_input: float, steering_input: float) -> void:
	"""Update engine physics state"""
	throttle_input = lerp(throttle_input, throttle, dt / throttle_response_time)
	
	var target_rpm: float = idle_rpm
	
	# Calculate desired RPM based on gear and throttle
	if clutch_engaged and current_gear != 0:
		target_rpm = _calculate_target_rpm(throttle)
	else:
		target_rpm = idle_rpm
		
	# Apply RPM changes
	current_rpm = _update_rpm(target_rpm, dt)
	
	# Update vehicle speed from wheel rotation
	wheel_angular_velocity = current_rpm * PI / 30.0
	vehicle_speed_ms = wheel_angular_velocity * get_wheel_radius_m()
	
	# Calculate output torque
	output_torque_nm = _calculate_output_torque()
	
	# Handle gear shifts automatically if needed
	_handle_auto_shifts()
	
	# Check if clutch should disengage
	_check_clutch_state(brake_input)
	
	rpm_changed.emit(current_rpm, max_rpm)
	throttle_changed.emit(throttle_input)

func _calculate_target_rpm(throttle_input: float) -> float:
	"""Calculate what RPM should be based on throttle input"""
	var min_target: float = idle_rpm + 100.0
	var max_target: float = max_rpm * 0.95
	return lerp(min_target, max_target, throttle_input)

func _update_rpm(target_rpm: float, dt: float) -> float:
	"""Smoothly transition current RPM toward target"""
	var change_rate: float = (max_rpm - idle_rpm) * 8.0  # Acceleration rate
	var diff: float = target_rpm - current_rpm
	
	if abs(diff) < 10.0:
		return target_rpm
	
	return current_rpm + sign(diff) * min(abs(diff), change_rate * dt)

func _calculate_output_torque() -> float:
	"""Calculate torque delivered to wheels"""
	if not clutch_engaged or current_gear == 0:
		return 0.0
	
	var gear_ratio: float = _get_current_gear_ratio()
	var total_ratio: float = gear_ratio * final_drive_ratio
	
	# Get torque curve value based on current RPM
	var torque_multiplier: float = _get_torque_curve_multiplier()
	var engine_torque: float = peak_torque_nm * torque_multiplier
	
	# Apply drivetrain losses
	var wheel_torque: float = engine_torque * total_ratio * drivetrain_efficiency
	
	# Subtract engine braking effect
	var engine_brake: float = calculate_engine_braking_factor() * wheel_torque
	wheel_torque -= engine_brake
	
	return max(wheel_torque, 0.0)

func _get_torque_curve_multiplier() -> float:
	"""Get torque multiplier based on torque curve"""
	var normalized_rpm: float = current_rpm / max_rpm
	
	if normalized_rpm <= (peak_torque_rpm / max_rpm):
		return 1.0
	else:
		var drop_range: float = 1.0 - (peak_torque_rpm / max_rpm)
		var drop_distance: float = normalized_rpm - (peak_torque_rpm / max_rpm)
		return 1.0 - (drop_distance / drop_range) * (1.0 - torque_curve_slope)

func _get_current_gear_ratio() -> float:
	if current_gear == 0:
		return 0.0
	elif current_gear < 0:  # Reverse
		return reverse_ratio
	elif current_gear <= num_gears:
		return _gear_ratios[current_gear - 1]
	return 1.0

func shift_to_gear(new_gear: int) -> void:
	"""Manually shift to specific gear"""
	var old_gear: int = current_gear
	current_gear = new_gear
	gear_changed.emit(old_gear, new_gear)

func shift_up() -> void:
	if current_gear < num_gears:
		shift_to_gear(current_gear + 1)

func shift_down() -> void:
	if current_gear > 1:
		shift_to_gear(current_gear - 1)

func _handle_auto_shifts() -> void:
	"""Auto-shift logic based on RPM thresholds"""
	if current_gear == 0:
		return
		
	if current_gear < num_gears and current_rpm >= max_rpm * 0.95:
		shift_up()
	elif current_gear > 1 and current_rpm < idle_rpm * 1.2:
		shift_down()

func _check_clutch_state(brake_input: float) -> void:
	if brake_input > 0.8 and current_rpm < idle_rpm * 1.5:
		clutch_engaged = false
		clutch_disengaged.emit()
	elif brake_input < 0.5 and current_rpm > idle_rpm * 1.1:
		clutch_engaged = true
		clutch_engaged.emit()

func get_aerodynamic_drag_force(speed_ms: float) -> float:
	"""Calculate aerodynamic drag force at given speed"""
	var air_density: float = 1.225  # kg/m^3 at sea level
	var velocity_squared: float = speed_ms * speed_ms
	var drag_force: float = 0.5 * air_density * drag_coeffent_cd * frontal_area_m2 * velocity_squared
	return drag_force

func get_downforce_force(speed_ms: float) -> float:
	"""Calculate downforce generated at given speed"""
	var air_density: float = 1.225
	var velocity_squared: float = speed_ms * speed_ms
	return 0.5 * air_density * downforce_coefficient * frontal_area_m2 * velocity_squared

func get_horsepower_at_rpm(rpm: float) -> float:
	"""Calculate horsepower at specific RPM"""
	if rpm <= 0:
		return 0.0
	
	var torque_mult: float = _get_torque_curve_multiplier()
	var torque: float = peak_torque_nm * torque_mult
	var hp: float = (torque * rpm) / 7127.0  # Conversion factor Nm*RPM to HP
	return hp

func get_kilowatts_at_rpm(rpm: float) -> float:
	"""Calculate kilowatts at specific RPM"""
	return get_horsepower_at_rpm(rpm) * 0.7457

</FILE>