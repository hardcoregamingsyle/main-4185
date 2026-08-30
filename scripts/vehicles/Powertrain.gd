extends Node3D
class_name Powertrain

## Powertrain - Realistic engine physics simulation
## Models: RPM curve, torque output, horsepower, transmission ratios, clutch behavior
## Speed emerges from these variables, not set directly

signal rpm_changed(rpm: float)
signal gear_changed(old_gear: int, new_gear: int)
signal throttle_input(value: float)
signal shift_event(gear: int)

@export_group("Engine Configuration")
@export var max_rpm: float = 8000.0
@export var idle_rpm: float = 800.0
@export var redline_rpm: float = 7500.0
@export var engine_displacement: float = 5.0  # liters

@export_group("Torque & Horsepower")
@export var peak_torque_nm: float = 650.0  # Newton-meters
@export var peak_torque_rpm: float = 4500.0  # RPM where peak torque occurs
@export var peak_horsepower_hp: float = 450.0  # Peak horsepower
@export var peak_hp_rpm: float = 6500.0  # RPM where peak HP occurs

@export_group("Transmission")
@export var num_gears: int = 6
@export_array var gear_ratios: Array[float] = [3.8, 2.4, 1.7, 1.3, 1.0, 0.8]
@export var final_drive_ratio: float = 3.5
@export var reverse_ratio: float = -3.9

@export_group("Clutch & Throttle")
@export var clutch_engagement_point: float = 0.2  # Where clutch begins engaging
@export var max_clutch_slip: float = 0.15  # Max slip percentage
@export var throttle_response: float = 0.1  # How quickly throttle responds

@export_group("Drivetrain")
enum DrivetrainType { RWD, FWD, AWD }
@export var drivetrain_type: DrivetrainType = DrivetrainType.RWD
@export var differential_lock_factor: float = 0.5  # LSD effect (0=open, 1=locked)

# Runtime state
var current_rpm: float = 0.0
var current_gear: int = 0  # 0 = neutral, -1 = reverse
var clutch_pedal: float = 1.0  # 1.0 = fully engaged, 0.0 = disengaged
var throttle_pedal: float = 0.0  # 0.0 to 1.0
var brake_pedal: float = 0.0  # 0.0 to 1.0
var wheel_radius: float = 0.33  # meters
var wheel_count: int = 4

# Derived values
var current_torque_nm: float = 0.0
var current_horsepower_hp: float = 0.0
var output_torque_nm: float = 0.0  # After transmission
var output_speed_rps: float = 0.0  # Output shaft speed in revs per second

func _ready() -> void:
	current_rpm = idle_rpm
	current_gear = 0
	_update_torque_curve()

func _process(delta: float) -> void:
	_update_engine_state(delta)

## Update engine physics each frame
func _update_engine_state(delta: float) -> void:
	# Calculate desired RPM based on throttle and gear
	var target_rpm = _calculate_target_rpm()
	
	# Apply throttle response smoothing
	current_rpm = lerp(current_rpm, target_rpm, throttle_response * delta * 10.0)
	
	# Clamp to valid range
	current_rpm = clamp(current_rpm, idle_rpm, max_rpm)
	
	# Prevent over-revving when in neutral or reverse
	if current_gear == 0 or current_gear < 0:
		current_rpm = lerp(current_rpm, idle_rpm, delta * 2.0)
	
	# Emit signals
	rpm_changed.emit(current_rpm)
	
	# Update derived values
	_update_derived_values()

## Calculate target RPM based on vehicle speed and gear
func _calculate_target_rpm() -> float:
	if current_gear == 0:
		return idle_rpm
	
	# Get wheel speed from parent vehicle body
	var vehicle_body = get_parent()
	if vehicle_body and vehicle_body.has_method("get_linear_velocity"):
		var velocity = vehicle_body.get_linear_velocity()
		var speed_mps = velocity.length()
		
		# Calculate what RPM should be for this speed in current gear
		var gear_ratio = _get_current_gear_ratio()
		var total_ratio = gear_ratio * final_drive_ratio
		
		# RPM = (speed / (wheel_circumference)) * total_ratio * 60
		var wheel_rps = speed_mps / (2.0 * PI * wheel_radius)
		var wheel_rpm = wheel_rps * 60.0
		var target_rpm = wheel_rpm * total_ratio
		
		# Apply clutch slip if disengaged
		if clutch_pedal < 1.0:
			var slip = (1.0 - clutch_pedal) * max_clutch_slip
			target_rpm = lerp(target_rpm, current_rpm * (1.0 + slip), slip)
		
		return target_rpm
	
	return idle_rpm

## Calculate current torque based on RPM curve
func _calculate_torque_at_rpm(rpm: float) -> float:
	if rpm <= 0:
		return 0.0
	
	# Parabolic torque curve around peak torque point
	var distance_from_peak = abs(rpm - peak_torque_rpm)
	var taper_width: float = 2000.0  # RPM range for torque taper
	
	var torque_reduction = 1.0 - (distance_from_peak / taper_width)
	torque_reduction = max(0.0, min(1.0, torque_reduction))
	
	# Add low-RPM boost for turbo-like feel
	var low_rpm_boost = 0.0
	if rpm < peak_torque_rpm * 0.5:
		low_rpm_boost = (rpm / (peak_torque_rpm * 0.5)) * 0.1
	
	return peak_torque_nm * torque_reduction * (1.0 + low_rpm_boost)

## Calculate current horsepower from torque and RPM
func _calculate_horsepower(torque_nm: float, rpm: float) -> float:
	# HP = (Torque * RPM) / 5252 (standard conversion)
	return (torque_nm * rpm) / 5252.0

## Update derived torque and horsepower values
func _update_derived_values() -> void:
	current_torque_nm = _calculate_torque_at_rpm(current_rpm)
	current_horsepower_hp = _calculate_horsepower(current_torque_nm, current_rpm)
	
	# Apply transmission effects
	output_torque_nm = current_torque_nm * _get_current_gear_ratio() * final_drive_ratio * 0.95  # 5% loss
	output_speed_rps = (current_rpm / 60.0) / (_get_current_gear_ratio() * final_drive_ratio)

## Get current gear ratio including final drive
func _get_current_gear_ratio() -> float:
	match current_gear:
		0: return 0.0  # Neutral
		-1: return reverse_ratio
		_:
			if current_gear >= 1 and current_gear <= gear_ratios.size():
				return gear_ratios[current_gear - 1]
			else:
				return gear_ratios[0]  # Default to first gear

## Shift to specific gear
func shift_to_gear(gear: int) -> bool:
	if gear < -1 or gear > num_gears:
		return false
	
	var old_gear = current_germ
	current_gear = gear
	
	gear_changed.emit(old_gear, current_gear)
	shift_event.emit(current_gear)
	
	return true

## Auto-shift based on RPM
func auto_shift() -> int:
	if current_gear == 0 or current_rpm < idle_rpm:
		return current_gear
	
	# Upshift if approaching redline
	if current_rpm > redline_rpm and current_gear < num_gears:
		return current_gear + 1
	
	# Downshift if below torque band
	if current_rpm < peak_torque_rpm * 0.4 and current_gear > 1:
		return current_gear - 1
	
	return current_gear

## Apply braking force to engine (engine braking)
func apply_engine_braking(brake_force: float) -> float:
	var engine_resistance = current_torque_nm * 0.3 * brake_force  # 30% resistance
	return engine_resistance

## Get wheel speed from current gear and RPM
func get_wheel_rps() -> float:
	var gear_ratio = _get_current_gear_ratio()
	if gear_ratio == 0:
		return 0.0
	return output_speed_rps

## Get vehicle speed in m/s from current wheel rotation
func get_vehicle_speed() -> float:
	return get_wheel_rps() * (2.0 * PI * wheel_radius)

## Get vehicle speed in km/h
func get_vehicle_speed_kmh() -> float:
	return get_vehicle_speed() * 3.6

## Get vehicle speed in mph
func get_vehicle_speed_mph() -> float:
	return get_vehicle_speed() * 2.237

## Reset powertrain to idle state
func reset() -> void:
	current_rpm = idle_rpm
	current_gear = 0
	clutch_pedal = 1.0
	throttle_pedal = 0.0
	brake_pedal = 0.0
	_update_derived_values()