extends Resource
class_name Powertrain

## Powertrain - Complete engine physics simulation for realistic vehicle acceleration
## All power delivery comes from physical variables, not hardcoded speeds
## Handles: RPM curve, torque multiplication, gear ratios, throttle response, aerodynamics

signal rpm_changed(new_rpm: float)
signal gear_changed(old_gear: int, new_gear: int)
signal clutch_state_changed(is_engaged: bool)

@export_group("Engine Specifications")
@export var displacement_liters: float = 5.0  # Engine displacement
@export var max_rpm: float = 8000.0
@export var idle_rpm: float = 800.0
@export var redline_rpm: float = 7500.0

@export_group("Torque Curve")
var _torque_curve: Array[float] = []  # Indexed by normalized RPM (0-1)
@export var peak_torque_nm: float = 600.0
@export var peak_torque_rpm: float = 4000.0
@export var low_end_torque_factor: float = 0.75  # Torque at 2000 RPM / peak torque

@export_group("Horsepower & Power")
@export var peak_hp: float = 450.0  # Peak horsepower
@export var hp_at_max_rpm: float = 400.0  # HP at maximum RPM

@export_group("Transmission")
@export var transmission_type: String = "manual"  # "manual", "automatic", "semi-auto"
@export var gear_ratios: PackedFloat32Array = [3.5, 2.2, 1.6, 1.2, 0.9, 0.7]  # 6-speed
@export var final_drive_ratio: float = 3.73
@export var differential_type: String = "limited_slip"  # "open", "limited_slip", "locking"

@export_group("Clutch & Throttle")
@export var clutch_disengage_rpm: float = 1000.0
@export var throttle_response_time: float = 0.15  # seconds to reach full throttle
@export var rev_matcher_enabled: bool = true

@export_group("Aerodynamics")
@export var drag_coefficient: float = 0.32
@export var frontal_area: float = 2.2  # square meters
@export var downforce_coefficient: float = 0.5
@export var wing_angle_degrees: float = 10.0

@export_group("Wheel Configuration")
@export var wheel_diameter: float = 0.66  # meters (26 inch wheel + tire)
@export var wheel_circumference: float = 2.073  # pre-calculated pi * diameter

# Runtime state
var current_rpm: float = 0.0
var target_rpm: float = 0.0
var current_gear: int = 0  # 0 = neutral, 1-6 = gears
var clutch_engaged: bool = true
var throttle_position: float = 0.0  # 0.0 to 1.0
var brake_applied: float = 0.0  # 0.0 to 1.0
var current_speed_ms: float = 0.0  # Current vehicle speed in m/s
var current_torque_nm: float = 0.0
var current_power_watts: float = 0.0
var aerodynamic_downforce_newtons: float = 0.0
var air_resistance_newtons: float = 0.0

# Internal physics state
var _engine_braking_factor: float = 0.3
var _gear_shift_timer: float = 0.0
var _shift_delay_seconds: float = 0.2
var _throttle_accumulator: float = 0.0
var _rpm_inertia: float = 150.0  # How quickly RPM responds to throttle

func _init() -> void:
	_generate_torque_curve()

func _generate_torque_curve() -> void:
	"""Generate smooth torque curve based on engine specs"""
	var points: int = 100
	_torque_curve.resize(points)
	
	for i in range(points):
		var normalized_rpm: float = float(i) / float(points - 1)
		var actual_rpm: float = normalized_rpm * max_rpm
		
		if actual_rpm <= peak_torque_rpm:
			# Rising portion of torque curve
			var progress: float = actual_rpm / peak_torque_rpm
			_torque_curve[i] = lerp(low_end_torque_factor, 1.0, pow(progress, 0.5)) * peak_torque_nm
		else:
			# Falling portion after peak
			var progress: float = (actual_rpm - peak_torque_rpm) / (max_rpm - peak_torque_rpm)
			_torque_curve[i] = peak_torque_nm * (1.0 - progress * 0.4)  # Drop 40% by redline

func get_torque_at_rpm(rpm: float) -> float:
	"""Get torque output at given RPM"""
	if rpm <= 0.0 or rpm >= max_rpm:
		return 0.0
	
	var normalized_idx: float = (rpm / max_rpm) * (_torque_curve.size() - 1)
	var idx: int = int(normalized_idx)
	var blend: float = normalized_idx - idx
	
	if idx >= _torque_curve.size() - 1:
		return _torque_curve.back()
	
	return lerp(_torque_curve[idx], _torque_curve[min(idx + 1, _torque_curve.size() - 1)], blend)

func update(dt: float, input_throttle: float, input_brake: float, vehicle_speed_ms: float) -> void:
	"""Main update function - calculate all powertrain physics"""
	current_speed_ms = vehicle_speed_ms
	throttle_position = _update_throttle(input_throttle, dt)
	brake_applied = input_brake
	
	# Calculate wheel rotation speed
	var wheel_angular_velocity: float = _get_wheel_angular_velocity(vehicle_speed_ms)
	
	# Calculate clutch slip if disengaged
	var effective_rpm: float = current_rpm
	
	if not clutch_engaged:
		# Engine spins freely with minimal load
		effective_rpm = _apply_engine_friction(current_rpm, dt)
	else:
		# Engine connected to wheels
		var target_from_gear: float = _get_target_rpm_from_gear(wheel_angular_velocity)
		target_rpm = target_from_gear
	
	# Apply throttle effect on RPM
	current_rpm = _apply_throttle_effect(dt, throttle_position, effective_rpm)
	
	# Calculate current torque output
	current_torque_nm = get_torque_at_rpm(current_rpm)
	
	# Apply transmission losses (5-8% typical)
	var transmission_efficiency: float = 0.95
	current_torque_nm *= transmission_efficiency
	
	# Calculate power output (HP = torque * RPM / 5252)
	current_power_watts = (current_torque_nm * current_rpm * 2.0 * PI) / 60.0
	
	# Calculate aerodynamic forces
	_update_aerodynamics(vehicle_speed_ms)
	
	# Check for automatic shifting
	if transmission_type == "automatic":
		_auto_shift(dt)
	elif transmission_type == "semi-auto":
		_semi_auto_shift(dt)

func _update_throttle(target: float, dt: float) -> float:
	"""Smooth throttle response with inertia"""
	_throttle_accumulator += (target - throttle_position) * 1.0 / throttle_response_time
	_throttle_accumulator = clamp(_throttle_accumulator, -1.0, 1.0)
	return throttle_position + _throttle_accumulator * dt

func _get_wheel_angular_velocity(speed_ms: float) -> float:
	"""Convert linear speed to wheel angular velocity (rad/s)"""
	return speed_ms / (wheel_diameter / 2.0)

func _get_target_rpm_from_gear(wheel_angular_velocity: float) -> float:
	"""Calculate what RPM the engine should be at given wheel speed and gear"""
	if current_gear == 0:  # Neutral
		return idle_rpm
	
	var total_ratio: float = gear_ratios[current_gear - 1] * final_drive_ratio
	return wheel_angular_velocity * total_ratio

func _apply_throttle_effect(dt: float, throttle: float, base_rpm: float) -> float:
	"""Apply throttle to change RPM with realistic inertia"""
	var rpm_change: float = 0.0
	
	if throttle > 0.0:
		# Accelerating - increase RPM
		var torque_available: float = get_torque_at_rpm(base_rpm)
		var rpm_acceleration: float = torque_available / _rpm_inertia * throttle
		rpm_change = min(rpm_acceleration * dt, (max_rpm - base_rpm) * 0.1)
	elif throttle < 0.01 and base_rpm > idle_rpm:
		# Decelerating - decrease RPM
		rpm_change = -base_rpm * 2.0 * dt
	else:
		# Maintain RPM with slight friction
		rpm_change = -base_rpm * 0.05 * dt
	
	return clamp(base_rpm + rpm_change, idle_rpm, max_rpm)

func _apply_engine_friction(rpm: float, dt: float) -> float:
	"""Simulate engine internal friction when clutch disengaged"""
	var friction_loss: float = rpm * 0.02 * dt
	return max(rpm - friction_loss, idle_rpm)

func _update_aerodynamics(speed_ms: float) -> void:
	"""Calculate aerodynamic forces based on speed"""
	var air_density: float = 1.225  # kg/m^3 at sea level
	
	# Drag force: F = 0.5 * density * v^2 * Cd * A
	var dynamic_pressure: float = 0.5 * air_density * speed_ms * speed_ms
	air_resistance_newtons = dynamic_pressure * drag_coefficient * frontal_area
	
	# Downforce: F = 0.5 * density * v^2 * Cl * A
	aerodynamic_downforce_newtons = dynamic_pressure * downforce_coefficient * frontal_area

func _auto_shift(dt: float) -> void:
	"""Automatic transmission logic"""
	if current_gear == 0:
		return
	
	# Shift up at redline
	if current_rpm >= redline_rpm and current_gear < gear_ratios.size():
		_shift_to_gear(current_gear + 1, dt)
	# Keep optimal RPM for efficiency
	elif current_rpm < idle_rpm * 1.5 and current_gear > 1:
		_shift_to_gear(current_gear - 1, dt)

func _semi_auto_shift(dt: float) -> void:
	"""Manual transmission with auto-blip helper"""
	pass  # Handled by InputManager

func shift_up() -> void:
	"""Shift to next gear"""
	if current_gear > 0 and current_gear < gear_ratios.size():
		_shift_to_gear(current_gear + 1)

func shift_down() -> void:
	"""Shift to previous gear"""
	if current_gear > 1:
		_shift_to_gear(current_gear - 1)

func set_gear(gear_num: int) -> void:
	"""Set specific gear (0 = neutral)"""
	if gear_num != current_gear:
		_shift_to_gear(clamp(gear_num, 0, gear_ratios.size()))

func _shift_to_gear(new_gear: int, dt: float = 0.0) -> void:
	"""Perform gear shift"""
	if new_gear == current_gear:
		return
	
	var old_gear: int = current_gear
	current_gear = new_gear
	
	gear_signal.emit(old_gear, new_gear)
	
	# Auto-blip for downshifts
	if new_gear < old_gear and rev_matcher_enabled:
		var target_blip_rpm: float = _get_target_rpm_from_gear(
			_get_wheel_angular_velocity(current_speed_ms)
		)
		current_rpm = clamp(target_blip_rpm, idle_rpm, max_rpm)
	
	# Brief clutch disengage during shift
	clutch_engaged = false
	_engine_shifting = true

func reset_clutch() -> void:
	"""Re-engage clutch after shift"""
	clutch_engaged = true
	_engine_shifting = false

func get_driving_force() -> float:
	"""Calculate driving force at wheels"""
	if current_gear == 0:
		return 0.0
	
	var total_ratio: float = gear_ratios[current_gear - 1] * final_drive_ratio
	var wheel_torque: float = current_torque_nm * total_ratio
	
	# Account for wheel radius to get force
	var wheel_radius: float = wheel_diameter / 2.0
	var driving_force: float = wheel_torque / wheel_radius
	
	# Subtract rolling resistance (approx 1.5% of weight)
	var rolling_resistance: float = 0.015 * GameManager.physics_settings.default_vehicle_mass * GameManager.physics_settings.gravity
	
	return max(driving_force - rolling_resistance, 0.0)

func get_braking_force() -> float:
	"""Calculate braking force"""
	var brake_force: float = brake_applied * 8000.0  # Max 8000N braking
	return brake_force

func get_total_forces() -> Dictionary:
	"""Get all forces acting on vehicle"""
	return {
		"driving_force": get_driving_force(),
		"braking_force": get_braking_force(),
		"drag_force": air_resistance_newtons,
		"downforce": aerodynamic_downforce_newtons,
		"total_traction": get_driving_force() - get_braking_force() - air_resistance_newtons
	}

func simulate_crash_impact(impact_speed_ms: float) -> float:
	"""Calculate engine stress from crash impact"""
	var stress_factor: float = impact_speed_ms / 20.0  # 20 m/s = ~72 km/h
	return stress_factor

func dump_stats() -> Dictionary:
	"""Return current powertrain state for debugging"""
	return {
		"rpm": current_rpm,
		"gear": current_gear,
		"throttle": throttle_position,
		"brake": brake_applied,
		"torque_nm": current_torque_nm,
		"power_hp": current_power_watts / 745.7,  # Convert watts to HP
		"speed_kmh": current_speed_ms * 3.6,
		"drag_n": air_resistance_newtons,
		"downforce_n": aerodynamic_downforce_newtons
	}