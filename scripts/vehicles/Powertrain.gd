extends Resource
class_name Powertrain

## Powertrain - Complete engine and transmission physics simulation
## Calculates actual vehicle speed from engine parameters: RPM, torque, HP, gearing, wheel size
## Never sets speed directly - speed emerges from physics

signal rpm_changed(rpm: float)
signal gear_changed(old_gear: int, new_gear: int)
signal throttle_changed(throttle_value: float)
signal clutch_engaged(bool engaged)

@export_group("Engine Configuration")
@export var max_rpm: float = 7000.0          # Redline RPM
@export var idle_rpm: float = 800.0           # Idle RPM
@export var stall_rpm: float = 400.0          # Stall RPM
@export var engine_displacement: float = 4.0  # Liters
@export var cylinders: int = 8                # Cylinder count
@export var compression_ratio: float = 10.5   # Compression ratio
@export var redline_rpm: float = 7000.0       # Max safe RPM

@export_group("Power Characteristics")
@export var peak_horsepower: float = 450.0    # Peak HP at certain RPM
@export var peak_hp_rpm: float = 6000.0       # RPM where peak HP occurs
@export var peak_torque_nm: float = 600.0     # Peak torque in Newton-meters
@export var peak_torque_rpm: float = 4500.0   # RPM where peak torque occurs
@export var horsepower_curve: Array[Vector2]  # Custom HP curve points (RPM -> HP)
@export var torque_curve: Array[Vector2]      # Custom torque curve points (RPM -> Nm)

@export_group("Transmission")
@export var transmission_type: String = "manual"  # "manual", "automatic", "dual_clutch"
@export var final_drive_ratio: float = 3.73      # Final drive differential ratio
@export var gear_ratios: Dictionary = {          # Gear ratios (negative = reverse)
	"neutral": 0.0,
	"r": -3.5,
	"1": 4.0,
	"2": 2.8,
	"3": 2.1,
	"4": 1.6,
	"5": 1.3,
	"6": 1.0,
	"7": 0.8,
	"8": 0.65,
	"d": 1.0,              # Drive mode for auto/transmission
	"l": 3.0               # Low range/off-road
}
@export var current_gear: int = 1  # Current selected gear (1-8, R, N)

@export_group("Wheel Configuration")
@export var wheel_diameter: float = 0.68  # Meters (approx 26 inch tire including sidewall)
@export var wheel_radius: float = 0.34    # Meters
@export var tire_friction_coefficient: float = 1.2  # Dry asphalt
@export var tire_width: float = 0.275     # Meters

@export_group("Drivetrain")
@export var drivetrain_type: String = "rwd"  # "fwd", "rwd", "awd", "4wd"
@export var front_torque_bias: float = 0.0   # Percentage to front wheels (for AWD/FWD)
@export var rear_torque_bias: float = 1.0    # Percentage to rear wheels (for AWD/RWD)
@export var limited_slip_diff: bool = true
@export var diff_lock_ratio: float = 0.7     # How locked the LSD is (0 = open, 1 = locked)

@export_group("Vehicle Mass & Inertia")
@export var vehicle_mass_kg: float = 1500.0   # Total vehicle mass
@export var drivetrain_loss_factor: float = 0.15  # 15% power loss through drivetrain
@export var rotating_mass_inertia: float = 2.5  # kg*m^2 equivalent rotating mass

@export_group("Aerodynamics")
@export var drag_coefficient: float = 0.32    # Cd value
@export var frontal_area_m2: float = 2.2      # m^2
@export var air_density: float = 1.225        # kg/m^3 sea level

var _current_rpm: float = 0.0                  # Actual current RPM
var _throttle_input: float = 0.0               # 0.0 to 1.0 pedal position
var _clutch_pedal: float = 1.0                 # 1.0 = disengaged, 0.0 = fully engaged
var _brake_pressure: float = 0.0               # 0.0 to 1.0 brake pressure
var _vehicle_speed_ms: float = 0.0             # Calculated vehicle speed (m/s)
var _wheel_angular_velocity: float = 0.0       # Rad/s
var _engine_braking_enabled: bool = true
var _auto_shift_mode: bool = false
var _shift_points: Dictionary = {}              # Auto shift thresholds per gear
var _engine_temp: float = 90.0                  # Celsius
var _turbo_boost: float = 0.0                   # Bar of boost pressure
var _turbo_spool_level: float = 0.0             # 0.0 to 1.0 turbo spool state

func _init() -> void:
	_default_shift_points()
	_init_turbo_model()

func _default_shift_points() -> void:
	_shift_points = {
		"up": Vector2(peak_hp_rpm * 0.95, peak_hp_rpm * 1.0),  # Upshift near peak HP
		"down": Vector2(idle_rpm * 1.3, idle_rpm * 1.5),       # Downshift above idle + margin
	}

func _init_turbo_model() -> void:
	# Initialize turbo characteristics
	_turbo_spool_level = 0.0
	_turbo_boost = 0.0

func get_current_rpm() -> float:
	return _current_rpm

func get_throttle_input() -> float:
	return _throttle_input

func get_vehicle_speed_ms() -> float:
	return _vehicle_speed_ms

func get_vehicle_speed_kmh() -> float:
	return _vehicle_speed_ms * 3.6

func set_throttle(value: float) -> void:
	_throttle_input = clamp(value, 0.0, 1.0)
	throttle_changed.emit(_throttle_input)

func set_clutch(value: float) -> void:
	_clutch_pedal = clamp(value, 0.0, 1.0)
	clutch_engaged.emit(_clutch_pedal < 0.1)

func set_brake_pressure(value: float) -> void:
	_brake_pressure = clamp(value, 0.0, 1.0)

func set_gear(gear: String) -> void:
	if gear in gear_ratios:
		var old_gear = current_gear
		current_gear = gear
		gear_changed.emit(old_gear, gear)

func set_gear_numeric(gear_num: int) -> void:
	if gear_num >= 1 and gear_num <= 8:
		set_gear(str(gear_num))
	elif gear_num == -1:
		set_gear("r")
	elif gear_num == 0:
		set_gear("n")

func get_gear_ratio() -> float:
	if current_gear in gear_ratios:
		return abs(gear_ratios[current_gear])
	return 0.0

func calculate_torque_at_wheels() -> float:
	"""Calculate torque delivered to wheels based on current RPM and throttle."""
	if current_gear == "n":
		return 0.0
	
	var engine_torque = get_engine_torque(_current_rpm)
	var total_ratio = get_total_ratio()
	var effective_torque = engine_torque * total_ratio * (1.0 - drivetrain_loss_factor)
	
	# Apply clutch engagement factor
	effective_torque *= (1.0 - _clutch_pedal)
	
	# Apply turbo boost multiplier if applicable
	var turbo_multiplier = 1.0 + (_turbo_boost * 0.3)
	effective_torque *= turbo_multiplier
	
	return effective_torque

func get_engine_torque(rpm: float) -> float:
	"""Get torque output at given RPM using interpolated curve."""
	if torque_curve.is_empty():
		# Use default bell curve approximation
		var rpm_normalized = (rpm - idle_rpm) / (max_rpm - idle_rpm)
		rpm_normalized = clamp(rpm_normalized, 0.0, 1.0)
		
		# Bell curve peaking at peak_torque_rpm
		var peak_pos = (peak_torque_rpm - idle_rpm) / (max_rpm - idle_rpm)
		var distance_from_peak = abs(rpm_normalized - peak_pos)
		var torque_reduction = pow(distance_from_peak, 2) * 0.5
		
		return peak_torque_nm * (1.0 - torque_reduction)
	else:
		# Interpolate custom curve
		for i in range(torque_curve.size() - 1):
			var point1 = torque_curve[i]
			var point2 = torque_curve[i + 1]
			
			if rpm >= point1.x and rpm <= point2.x:
				var t = (rpm - point1.x) / (point2.x - point1.x)
				return lerp(point1.y, point2.y, t)
		
		# Fallback to last point
		return torque_curve.back().y

func get_engine_horsepower(rpm: float) -> float:
	"""Calculate horsepower at given RPM."""
	var torque = get_engine_torque(rpm)
	# HP = (Torque * RPM) / 5252 (imperial) or (Torque * RPM) / 9549 (metric Nm)
	return (torque * rpm) / 9549.0

func get_total_ratio() -> float:
	"""Get total gear reduction from engine to wheels."""
	if current_gear == "n":
		return 0.0
	
	var gear_ratio = abs(gear_ratios[current_gear])
	return gear_ratio * final_drive_ratio

func calculate_acceleration(force_n: float) -> float:
	"""Calculate acceleration from force using F=ma."""
	return force_n / vehicle_mass_kg

func calculate_force_from_power(power_watts: float, velocity_ms: float) -> float:
	"""Calculate force available at given power and velocity."""
	if velocity_ms <= 0.0:
		return power_watts / 1.0  # Handle initial acceleration
	return power_watts / velocity_ms

func update_physics(dt: float) -> void:
	"""Update engine physics state based on inputs."""
	_update_rpm(dt)
	_update_wheel_velocity()
	_update_engine_temperature(dt)
	_update_turbo(dt)

func _update_rpm(dt: float) -> void:
	"""Simulate engine RPM changes based on throttle, load, and gear."""
	if current_gear == "n":
		# Engine idles or revs freely
		if _throttle_input > 0.0:
			var target_rpm = idle_rpm + (_throttle_input * (max_rpm - idle_rpm))
			_current_rpm = lerp(_current_rpm, target_rpm, dt * 5.0)
		else:
			_current_rpm = lerp(_current_rpm, idle_rpm, dt * 3.0)
		return
	
	# Calculate target wheel RPM based on vehicle speed
	var wheel_circumference = PI * wheel_diameter
	var gear_ratio = get_gear_ratio()
	var target_engine_rpm = (_vehicle_speed_ms * gear_ratio * final_drive_ratio) / (PI * wheel_radius)
	
	# Apply throttle effect
	var engine_acceleration_rate = 1000.0 * _throttle_input  # RPM per second
	var engine_deceleration_rate = 500.0 * (1.0 - _throttle_input)
	
	if _current_rpm < target_engine_rpm:
		_current_rpm += engine_acceleration_rate * dt
	else:
		_current_rpm -= engine_deceleration_rate * dt
	
	# Limit to operational range
	_current_rpm = clamp(_current_rpm, stall_rpm, max_rpm + 500.0)
	
	# Check for over-rev protection
	if _current_rpm > redline_rpm:
		_current_rpm = redline_rpm * 0.95  # Soft limit
	
	# Apply engine braking when throttle released
	if _throttle_input <= 0.0 and _clutch_pedal < 0.1:
		_current_rpm = lerp(_current_rpm, target_engine_rpm, dt * 8.0)
	
	rpm_changed.emit(_current_rpm)

func _update_wheel_velocity() -> void:
	"""Calculate wheel angular velocity from vehicle speed."""
	if current_gear != "n" and _clutch_pedal < 0.1:
		var gear_ratio = get_gear_ratio()
		var total_ratio = gear_ratio * final_drive_ratio
		_wheel_angular_velocity = (_vehicle_speed_ms / wheel_radius) / total_ratio
	else:
		_wheel_angular_velocity = 0.0

func _update_engine_temperature(dt: float) -> void:
	"""Simulate engine temperature changes."""
	var heat_generation = _throttle_input * 0.3  # More throttle = more heat
	var cooling_factor = 0.1  # Natural cooling rate
	
	# Turbo adds heat
	heat_generation += _turbo_boost * 0.5
	
	_engine_temp += (heat_generation - cooling_factor) * dt
	_engine_temp = clamp(_engine_temp, 60.0, 120.0)  # Normal operating range

func _update_turbo(dt: float) -> void:
	"""Simulate turbocharger spool and boost."""
	var target_boost = _throttle_input * 1.5  # Max 1.5 bar boost
	var spool_rate = 2.0  # Turbo spool speed
	var bleed_rate = 1.0  # Boost bleed-off rate
	
	# Spool up
	_turbo_spool_level = lerp(_turbo_spool_level, _throttle_input, dt * spool_rate)
	_turbo_boost = lerp(_turbo_boost, _turbo_spool_level * target_boost, dt * spool_rate)
	
	# Bleed off when throttle released
	if _throttle_input < 0.1:
		_turbo_boost = max(0.0, _turbo_boost - dt * bleed_rate)
		_turbo_spool_level = max(0.0, _turbo_spool_level - dt * bleed_rate)

func calculate_aerodynamic_drag(velocity_ms: float) -> float:
	"""Calculate aerodynamic drag force at given velocity."""
	var dynamic_pressure = 0.5 * air_density * pow(velocity_ms, 2)
	var drag_force = dynamic_pressure * drag_coefficient * frontal_area_m2
	return drag_force

func calculate_max_theoretical_speed() -> float:
	"""Calculate theoretical maximum speed at peak horsepower."""
	var peak_hp_watts = peak_horsepower * 745.7  # Convert HP to watts
	var drag_at_max = calculate_aerodynamic_drag(0)  # Initial
	# Simplified: max speed when power = drag
	# P = F*v = (0.5*rho*Cd*A*v^2)*v = 0.5*rho*Cd*A*v^3
	var v_cubed = peak_hp_watts / (0.5 * air_density * drag_coefficient * frontal_area_m2)
	return pow(v_cubed, 1.0 / 3.0)

func simulate_drift(lateral_force: float, grip_limit: float) -> float:
	"""Calculate drift angle based on lateral forces vs available grip."""
	if grip_limit <= 0.0:
		return 0.0
	
	var slip_angle = asin(clamp(lateral_force / grip_limit, -1.0, 1.0))
	return rad_to_deg(slip_angle)

func get_powerband_status() -> String:
	"""Return current engine powerband status string."""
	var normalized_rpm = (_current_rpm - idle_rpm) / (max_rpm - idle_rpm)
	
	if normalized_rpm < 0.3:
		return "low"
	elif normalized_rpm < 0.6:
		return "mid"
	elif normalized_rpm < 0.85:
		return "high"
	else:
		return "redline"

func reset() -> void:
	"""Reset powertrain to initial state."""
	_current_rpm = idle_rpm
	_throttle_input = 0.0
	_clutch_pedal = 1.0
	_brake_pressure = 0.0
	_vehicle_speed_ms = 0.0
	_wheel_angular_velocity = 0.0
	_engine_temp = 90.0
	_turbo_boost = 0.0
	_turbo_spool_level = 0.0