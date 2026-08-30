extends Resource
class_name Powertrain

## Powertrain - Advanced vehicle powertrain physics simulation
## Simulates real-world engine behavior including RPM, torque curves, horsepower, 
## gearboxes, transmission efficiency, and aerodynamic drag

signal rpm_changed(rpm: float)
signal gear_changed(old_gear: int, new_gear: int)
signal clutch_engaged()
signal clutch_disengaged()
signal throttle_change(value: float)

enum PowertrainType { ICE, ELECTRIC, HYBRID }
enum ClutchState { ENGAGED, DISENGAGED, PARTIAL }

# ============================================================================
# ENGINE PHYSICS PARAMETERS
# ============================================================================

@export_group("Engine Specifications")
@export var powertrain_type: PowertrainType = PowertrainType.ICE

@export var engine_displacement_liters: float = 5.0  # Cylinder volume in liters
@export var cylinder_count: int = 8
@export var compression_ratio: float = 11.5
@export var max_rpm: float = 7500.0  # Redline RPM
@export var idle_rpm: float = 800.0  # Idle RPM
@export var stall_rpm: float = 400.0  # Stall RPM threshold

@export var peak_power_hp: float = 450.0  # Maximum horsepower
@export var peak_power_rpm: float = 6200.0  # RPM at peak power
@export var peak_torque_nm: float = 600.0  # Maximum torque in Newton-meters
@export var peak_torque_rpm: float = 4500.0  # RPM at peak torque

@export var rev_limit_rpm: float = 7800.0  # Absolute maximum before limiter engages
@export var redline_rpm: float = 7500.0  # Warning threshold

# ============================================================================
# TRANSMISSION SPECIFICATIONS
# ============================================================================

@export_group("Transmission")
@export var transmission_type: String = "Manual"
@export var number_of_gears: int = 6
@export var final_drive_ratio: float = 3.45
@export var clutch_friction_coefficient: float = 0.35
@export var drivetrain_efficiency: float = 0.85  # 85% efficiency loss

# Gear ratios (reverse included if needed)
var _gear_ratios: Array[float] = [3.9, 2.4, 1.7, 1.3, 1.0, 0.8]
var _reverse_ratio: float = -3.5

# ============================================================================
# VEHICLE PHYSICS INPUTS
# ============================================================================

@export_group("Vehicle Parameters")
@export var vehicle_mass_kg: float = 1500.0
@export var wheel_radius_meters: float = 0.33
@export var wheel_diameter_meters: float = 0.66
@export var tire_friction_coefficient: float = 1.2
@export var aerodynamic_drag_coefficient: float = 0.32
@export var frontal_area_m2: float = 2.2

@export_group("Aerodynamics")
@export var downforce_coefficient: float = 0.5
@export var front_downforce_ratio: float = 0.45
@export var rear_downforce_ratio: float = 0.55

# ============================================================================
# INTERNAL STATE
# ============================================================================

var current_rpm: float = 0.0
var current_gear: int = 0  # 0 = Neutral
var clutch_state: ClutchState = ClutchState.DISENGAGED
var throttle_input: float = 0.0  # 0.0 to 1.0
var brake_input: float = 0.0  # 0.0 to 1.0
var vehicle_speed_ms: float = 0.0  # Speed in meters per second
var vehicle_speed_kmh: float = 0.0  # Speed in km/h

var _torque_curve_points: Array[Vector2] = []
var _horsepower_curve_points: Array[Vector2] = []
var _engine_braking_factor: float = 0.4
var _last_updated_time: float = 0.0

func _init() -> void:
	_generate_torque_curve()
	_generate_horsepower_curve()
	_last_updated_time = Time.get_ticks_msec()

# ============================================================================
# TORQUE & HORSEPOWER CURVE GENERATION
# ============================================================================

func _generate_torque_curve() -> void:
	"""Generate smooth torque curve based on engine characteristics"""
	var points := 100
	var step: float = max_rpm / points
	
	for i in range(points + 1):
		var rpm: float = i * step
		var normalized_rpm: float = fposmod(rpm, max_rpm) / max_rpm
		
		# Gaussian-based torque curve peaking at peak_torque_rpm
		var gaussian_peak: float = exp(-pow(normalized_rpm - (peak_torque_rpm / max_rpm), 2) / 0.15)
		
		# Add some asymmetry for realistic feel
		var low_ramp: float = 0.0 if normalized_rpm < 0.1 else pow(normalized_rpm - 0.1, 2) * 0.5
		var high_cut: float = 1.0 if normalized_rpm > 0.95 else (1.0 - normalized_rpm) * 4.0
		
		var torque_value: float = peak_torque_nm * gaussian_peak * (1.0 - high_cut) + (low_ramp * peak_torque_nm * 0.3)
		
		# Ensure minimum torque at idle
		if rpm <= idle_rpm:
			torque_value = max(torque_value, peak_torque_nm * 0.4)
			
		_torque_curve_points.append(Vector2(rpm, torque_value))

func _generate_horsepower_curve() -> void:
	"""Generate horsepower curve from torque curve"""
	for point in _torque_curve_points:
		var rpm: float = point.x
		var torque: float = point.y
		# HP = (Torque * RPM) / 5252 (imperial units conversion)
		var hp: float = (torque * rpm) / 5252.0
		_horsepower_curve_points.append(Vector2(rpm, hp))

# ============================================================================
# CURVE LOOKUP FUNCTIONS
# ============================================================================

func get_torque_at_rpm(rpm: float) -> float:
	"""Get torque value at specified RPM using linear interpolation"""
	if rpm <= 0.0:
		return 0.0
	if rpm >= max_rpm:
		rpm = max_rpm
		
	# Linear interpolation through torque curve points
	for i in range(_torque_curve_points.size() - 1):
		var p1: Vector2 = _torque_curve_points[i]
		var p2: Vector2 = _torque_curve_points[i + 1]
		
		if rpm >= p1.x and rpm <= p2.x:
			var ratio: float = (rpm - p1.x) / (p2.x - p1.x)
			return lerp(p1.y, p2.y, ratio)
	
	return _torque_curve_points.back().y

func get_horsepower_at_rpm(rpm: float) -> float:
	"""Get horsepower value at specified RPM using linear interpolation"""
	if rpm <= 0.0:
		return 0.0
	if rpm >= max_rpm:
		rpm = max_rpm
	
	for i in range(_horsepower_curve_points.size() - 1):
		var p1: Vector2 = _horsepower_curve_points[i]
		var p2: Vector2 = _horsepower_curve_points[i + 1]
		
		if rpm >= p1.x and rpm <= p2.x:
			var ratio: float = (rpm - p1.x) / (p2.x - p1.x)
			return lerp(p1.y, p2.y, ratio)
	
	return _horsepower_curve_points.back().y

func get_optimal_shift_rpm() -> float:
	"""Calculate optimal RPM for upshifting based on power band"""
	return peak_power_rpm + (peak_power_rpm - idle_rpm) * 0.3

func get_optimal_downshift_rpm() -> float:
	"""Calculate optimal RPM for downshifting to maintain power"""
	return peak_torque_rpm * 0.85

# ============================================================================
# GEAR RATIO CALCULATIONS
# ============================================================================

func get_current_gear_ratio() -> float:
	"""Get the current gear ratio (including final drive)"""
	if current_gear == 0:
		return 0.0  # Neutral
	
	var gear_idx: int = current_gear - 1  # Convert 1-indexed to 0-indexed
	var gear_ratio: float = gear_ratios[gear_idx] if gear_idx < gear_ratios.size() else gear_ratios.back()
	
	return gear_ratio * final_drive_ratio

func get_wheel_rpm() -> float:
	"""Calculate wheel RPM based on engine RPM and current gear"""
	if current_gear == 0:
		return 0.0
	
	var total_ratio: float = get_current_gear_ratio()
	return current_rpm / abs(total_ratio)

func calculate_vehicle_speed_from_rpm() -> float:
	"""Calculate vehicle speed based on engine RPM and gear"""
	var wheel_rpm: float = get_wheel_rpm()
	var wheel_circumference: float = PI * wheel_diameter_meters
	var speed_m_per_min: float = wheel_rpm * wheel_circumference
	var speed_m_per_sec: float = speed_m_per_min / 60.0
	var speed_kmh: float = speed_m_per_sec * 3.6
	
	return speed_kmh

func rpm_for_speed(speed_kmh: float, gear: int) -> float:
	"""Calculate required RPM for given speed in specific gear"""
	if gear <= 0 or gear > number_of_gears:
		return 0.0
	
	var gear_idx: int = gear - 1
	var gear_ratio: float = gear_ratios[gear_idx] if gear_idx < gear_ratios.size() else gear_ratios.back()
	
	var wheel_rps: float = (speed_kmh / 3.6) / (PI * wheel_diameter_meters)
	var wheel_rpm: float = wheel_rps * 60.0
	
	return wheel_rpm * abs(gear_ratio * final_drive_ratio)

# ============================================================================
# THROTTLE & BRAKE INPUT HANDLING
# ============================================================================

func set_throttle(input: float) -> void:
	throttle_input = clamp(input, 0.0, 1.0)
	throttle_change.emit(throttle_input)

func set_brake(input: float) -> void:
	brake_input = clamp(input, 0.0, 1.0)

func update_physics(delta: float) -> void:
	"""Update powertrain physics calculations each frame"""
	_update_rpm(delta)
	_apply_aerodynamic_effects()
	_check_clutch_behavior()

func _update_rpm(delta: float) -> void:
	"""Simulate engine RPM changes based on throttle, load, and gear"""
	if current_gear == 0:
		# Neutral - engine freewheels
		if throttle_input > 0.0:
			current_rpm = lerp(current_rpm, throttle_input * max_rpm * 0.8, delta * 15.0)
		else:
			current_rpm = lerp(current_rpm, idle_rpm, delta * 5.0)
		return
	
	# Calculate wheel speed from vehicle speed
	var expected_rpm: float = rpm_for_speed(vehicle_speed_kmh, current_gear)
	
	if clutch_state == ClutchState.DISENGAGED:
		# Clutch disengaged - engine goes to idle or throttle response
		if throttle_input > 0.1:
			current_rpm = lerp(current_rpm, throttle_input * max_rpm * 0.6, delta * 20.0)
		else:
			current_rpm = lerp(current_rpm, idle_rpm, delta * 8.0)
	elif clutch_state == ClutchState.PARTIAL:
		# Partial engagement - blend between engine and wheel speeds
		var slip_factor: float = 0.5  # Tunable partial engagement
		var target_rpm: float = lerp(expected_rpm, throttle_input * max_rpm, 0.7)
		current_rpm = lerp(current_rpm, target_rpm, delta * (slip_factor + 0.3))
	else:
		# Fully engaged - connect engine to wheels
		var engine_acceleration: float = _calculate_engine_acceleration(delta)
		
		# Apply acceleration or deceleration
		if throttle_input > 0.1:
			current_rpm += engine_acceleration * throttle_input * delta * 100.0
		else:
			# Engine braking when no throttle
			current_rpm -= engine_acceleration * _engine_braking_factor * delta * 50.0
		
		# Prevent going below idle when moving
		if current_rpm < idle_rpm and vehicle_speed_kmh > 1.0:
			current_rpm = idle_rpm
		
		# Clamp within operational range
		current_rpm = clamp(current_rpm, idle_rpm, rev_limit_rpm)
	
	# Emit signal
	rpm_changed.emit(current_rpm)

func _calculate_engine_acceleration(delta: float) -> float:
	"""Calculate how fast the engine can accelerate based on torque"""
	var current_torque: float = get_torque_at_rpm(current_rpm)
	var effective_torque: float = current_torque * drivetrain_efficiency
	
	# Simplified rotational inertia model
	var effective_inertia: float = 0.15  # kg*m^2 equivalent
	var angular_acceleration: float = effective_torque / effective_inertia
	
	# Convert to RPM change per second
	var rpm_per_second: float = angular_acceleration * 9.55
	
	return rpm_per_second

# ============================================================================
# CLUTCH BEHAVIOR
# ============================================================================

func engage_clutch() -> void:
	clutch_state = ClutchState.ENGINE_BRKING
	clutch_engaged.emit()

func disengage_clutch() -> void:
	clutch_state = ClutchState.DISENGAGED
	clutch_disengaged.emit()

func partial_engage_clutch(amount: float) -> void:
	clutch_state = ClutchState.PARTIAL
	# amount parameter controls engagement level

func _check_clutch_behavior() -> void:
	"""Handle clutch slip and heat buildup during engagement"""
	if clutch_state == ClutchState.PARTIAL:
		var rpm_difference: float = abs(current_rpm - get_wheel_rpm())
		var slip_heat: float = rpm_difference * (1.0 - get_partial_engagement_amount())
		# Could track clutch temperature here for wear simulation

func get_partial_engagement_amount() -> float:
	return 0.5  # Placeholder for actual clutch pedal position

# ============================================================================
# VEHICLE SPEED SIMULATION
# ============================================================================

func apply_vehicle_force(force_n: float, delta: float) -> void:
	"""Apply external force to vehicle (from traction, drag, etc.)"""
	var acceleration: float = force_n / vehicle_mass_kg
	vehicle_speed_ms += acceleration * delta
	vehicle_speed_kmh = vehicle_speed_ms * 3.6

func apply_aerodynamic_effects() -> void:
	"""Calculate and apply aerodynamic drag forces"""
	var air_density: float = 1.225  # kg/m^3 at sea level
	var drag_force: float = 0.5 * air_density * aerodynamic_drag_coefficient * frontal_area_m2 * vehicle_speed_ms * vehicle_speed_ms
	
	apply_vehicle_force(-drag_force, 1.0)

func _apply_aerodynamic_effects() -> void:
	apply_aerodynamic_effects()

func calculate_downforce() -> float:
	"""Calculate total downforce generated by aerodynamics"""
	var air_density: float = 1.225
	var dynamic_pressure: float = 0.5 * air_density * vehicle_speed_ms * vehicle_speed_ms
	var total_downforce: float = downforce_coefficient * dynamic_pressure * frontal_area_m2
	
	return total_downforce

func get_front_axle_load() -> float:
	"""Calculate weight distribution on front axle"""
	var base_weight: float = vehicle_mass_kg * 9.81 * front_downforce_ratio
	var aero_downforce: float = calculate_downforce() * front_downforce_ratio
	return base_weight + aero_downforce

func get_rear_axle_load() -> float:
	"""Calculate weight distribution on rear axle"""
	var base_weight: float = vehicle_mass_kg * 9.81 * rear_downforce_ratio
	var aero_downforce: float = calculate_downforce() * rear_downforce_ratio
	return base_weight + aero_downforce

# ============================================================================
# SHIFT LOGIC
# ============================================================================

func shift_up() -> bool:
	"""Attempt to shift up one gear"""
	if current_gear < number_of_gears:
		var old_gear: int = current_gear
		current_gear += 1
		gear_changed.emit(old_gear, current_gear)
		return true
	return false

func shift_down() -> bool:
	"""Attempt to shift down one gear"""
	if current_gear > 1:
		var old_gear: int = current_gear
		current_gear -= 1
		gear_changed.emit(old_gear, current_gear)
		return true
	return false

func set_gear(gear: int) -> bool:
	"""Set specific gear (with validation)"""
	if gear < 0 or gear > number_of_gears:
		return false
	
	if gear != current_gear:
		var old_gear: int = current_gear
		current_gear = gear
		gear_changed.emit(old_gear, current_gear)
	return true

func neutral() -> void:
	current_gear = 0

func reverse() -> bool:
	"""Engage reverse gear"""
	if current_gear == 0 or current_gear == number_of_gears:
		current_gear = -1  # Representing reverse
		gear_changed.emit(0, -1)
		return true
	return false

# ============================================================================
# POWER OUTPUT CALCULATIONS
# ============================================================================

func get_wheel_torque() -> float:
	"""Calculate torque delivered to the wheels"""
	var engine_torque: float = get_torque_at_rpm(current_rpm)
	var gear_ratio: float = get_current_gear_ratio()
	
	if gear_ratio == 0.0:
		return 0.0
	
	return engine_torque * abs(gear_ratio) * drivetrain_efficiency

func get_wheel_power() -> float:
	"""Calculate power delivered to the wheels"""
	var wheel_torque: float = get_wheel_torque()
	var wheel_rps: float = get_wheel_rpm() / 60.0
	
	# Power = Torque * Angular Velocity
	return wheel_torque * wheel_rps * 1.34102  # Convert to horsepower

func get_available_traction_force() -> float:
	"""Calculate maximum traction force available at wheels"""
	var normal_force: float = get_rear_axle_load()  # Assume RWD for now
	var max_traction: float = normal_force * tire_friction_coefficient
	
	return max_traction

func calculate_acceleration() -> float:
	"""Calculate vehicle acceleration in m/s^2"""
	var wheel_torque: float = get_wheel_torque()
	var driving_force: float = wheel_torque / wheel_radius_meters
	var max_traction: float = get_available_traction_force()
	
	# Limit by available traction
	driving_force = min(driving_force, max_traction)
	
	# Subtract aerodynamic drag
	var drag_force: float = 0.5 * 1.225 * aerodynamic_drag_coefficient * frontal_area_m2 * vehicle_speed_ms * vehicle_speed_ms
	driving_force -= drag_force
	
	# Calculate acceleration
	var acceleration: float = driving_force / vehicle_mass_kg
	
	return acceleration

# ============================================================================
# CRASH & DAMAGE SIMULATION
# ============================================================================

var _engine_damage_level: float = 0.0
var _transmission_damage_level: float = 0.0

func take_impact(impact_force: float, impact_location: String) -> void:
	"""Simulate damage from collision impact"""
	match impact_location:
		"front":
			_engine_damage_level += impact_force * 0.01
		"rear":
			_transmission_damage_level += impact_force * 0.008
		"side":
			_engine_damage_level += impact_force * 0.005
			_transmission_damage_level += impact_force * 0.005

func get_performance_penalty() -> float:
	"""Get performance reduction due to damage"""
	return (_engine_damage_level + _transmission_damage_level) * 0.5

func get_max_effective_rpm() -> float:
	"""Get RPM limit after accounting for damage"""
	return max_rpm * (1.0 - get_performance_penalty())

func reset_damage() -> void:
	_engine_damage_level = 0.0
	_transmission_damage_level = 0.0

# ============================================================================
# FUEL CONSUMPTION (for ICE vehicles)
# ============================================================================

var fuel_remaining: float = 60.0  # Liters
var fuel_consumption_rate: float = 0.0  # L/min

func calculate_fuel_consumption() -> float:
	"""Calculate fuel consumption based on throttle and RPM"""
	if powertrain_type != PowertrainType.ICE:
		return 0.0
	
	# Base consumption + throttle-dependent consumption
	var base_consumption: float = 0.4  # L/h at idle
	var throttle_consumption: float = throttle_input * 25.0  # Max 25 L/h at full throttle
	
	fuel_consumption_rate = base_consumption + throttle_consumption
	
	return fuel_consumption_rate

func consume_fuel(delta: float) -> void:
	"""Consume fuel over time"""
	var consumption: float = calculate_fuel_consumption()
	fuel_remaining -= consumption * delta / 60.0  # Convert to minutes
	
	fuel_remaining = max(fuel_remaining, 0.0)

func is_out_of_fuel() -> bool:
	return fuel_remaining <= 0.0

# ============================================================================
# TELEMETRY & DEBUG INFO
# ============================================================================

func get_telemetry_data() -> Dictionary:
	"""Return comprehensive telemetry for debugging and UI display"""
	return {
		"rpm": current_rpm,
		"max_rpm": max_rpm,
		"current_gear": current_gear,
		"throttle_input": throttle_input,
		"brake_input": brake_input,
		"vehicle_speed_kmh": vehicle_speed_kmh,
		"vehicle_speed_ms": vehicle_speed_ms,
		"wheel_torque_nm": get_wheel_torque(),
		"wheel_power_hp": get_wheel_power(),
		"acceleration_ms2": calculate_acceleration(),
		"clutch_state": clutch_state,
		"fuel_remaining_l": fuel_remaining if powertrain_type == PowertrainType.ICE else null,
		"torque_at_rpm": get_torque_at_rpm(current_rpm),
		"hp_at_rpm": get_horsepower_at_rpm(current_rpm),
		"performance_penalty": get_performance_penalty(),
		"aero_drag_n": 0.5 * 1.225 * aerodynamic_drag_coefficient * frontal_area_m2 * vehicle_speed_ms * vehicle_speed_ms,
		"downforce_n": calculate_downforce(),
		"front_axle_load_n": get_front_axle_load(),
		"rear_axle_load_n": get_rear_axle_load()
	}

func dump_debug_info() -> void:
	print("\n===== POWERTRAIN DEBUG =====")
	print("RPM: %.1f / %d (redline: %d)" % [current_rpm, max_rpm, redline_rpm])
	print("Gear: %d | Throttle: %.2f | Brake: %.2f" % [current_gear, throttle_input, brake_input])
	print("Speed: %.1f km/h (%.2f m/s)" % [vehicle_speed_kmh, vehicle_speed_ms])
	print("Wheel Torque: %.1f Nm | Wheel Power: %.1f hp" % [get_wheel_torque(), get_wheel_power()])
	print("Acceleration: %.2f m/s²")
	print("============================\n")

</FILE "scripts/vehicles/Powertrain.gd">