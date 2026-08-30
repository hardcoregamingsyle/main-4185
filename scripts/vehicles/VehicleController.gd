extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulation
## Handles throttle, brake, steering, wheel forces, gear shifting, and vehicle dynamics
## Integrates with PhysicsSettings for centralized tuning
## Copyright 2026 Thalamus Racing Simulator Project

signal speed_changed(current_speed: float)
signal rpm_changed(current_rpm: float)
signal gear_changed(new_gear: int)
signal drift_started()
signal drift_ended()
signal collision_impact(impact_force: float, impact_point: Vector3)
signal wheel_slip(wheel_index: int, slip_ratio: float)

# ============================================================================
# PHYSICS SETTINGS REFERENCE
# ============================================================================

@onready var _physics = PhysicsSettings.get_singleton()

# ============================================================================
# VEHICLE CONFIGURATION PROPERTIES
# ============================================================================

@export_group("Vehicle Config")
@export var max_engine_power: float = 300.0 # kW
@export var engine_displacement: float = 3.5 # liters
@export var torque_curve_points: Array[Vector2] = [] # (RPM -> Torque)
@export var gear_ratios: Array[float] = [3.5, 2.2, 1.5, 1.0, 0.8, 0.6]
@export var final_drive_ratio: float = 3.73
@export var differential_type: String = "open" # open, limited_slip, locked
@export var curb_weight: float = 1500.0 # kg
@export var center_of_mass_height: float = 0.55 # meters
@export var track_width: float = 1.65 # meters
@export var wheelbase: float = 2.7 # meters

@export_group("Tire Configuration")
@export var tire_width: float = 0.28 # meters
@export var tire_pressure_front: float = 2.2 # bar
@export var tire_pressure_rear: float = 2.2 # bar
@export var tire_friction_coefficient: float = 1.2 # dry asphalt
@export var tire_slickness: float = 0.85 # wet conditions multiplier

@export_group("Brake System")
@export var brake_pressure_front: float = 6.0 # bar
@export var brake_pressure_rear: float = 5.0 # bar
@export var brake_bias_front: float = 0.6 # front bias ratio
@export var brake_caliper_efficiency: float = 0.92
@export var brake_disc_diameter: float = 0.35 # meters
@export var brake_pad_friction: float = 0.45

@export_group("Aerodynamics")
@export var drag_coefficient: float = 0.32
@export var frontal_area: float = 2.2 # m²
@export var downforce_coefficient: float = 0.45
@export var wing_angle_degrees: float = 12.0

@export_group("Steering System")
@export var steering_ratio: float = 14.0 # rack ratio
@export var steering_lock_degrees: float = 450.0
@export var steering_response_time: float = 0.15 # seconds

# ============================================================================
# INTERNAL STATE VARIABLES
# ============================================================================

var current_speed: float = 0.0 # km/h
var current_rpm: float = 0.0 # engine RPM
var current_gear: int = 0 # 0 = neutral, 1-6 = gears
var target_gear: int = 0
var throttle_input: float = 0.0 # 0.0 - 1.0
var brake_input: float = 0.0 # 0.0 - 1.0
var steering_input: float = 0.0 # -1.0 (left) to +1.0 (right)
var clutch_engaged: bool = true

var _wheel_angles: Dictionary = { # actual wheel angles in degrees
	"front_left": 0.0,
	"front_right": 0.0,
	"rear_left": 0.0,
	"rear_right": 0.0
}

var _wheel_slip_ratios: Dictionary = {
	"front_left": 0.0,
	"front_right": 0.0,
	"rear_left": 0.0,
	"rear_right": 0.0
}

var _wheel_forces: Dictionary = {
	"front_left": Vector3.ZERO,
	"front_right": Vector3.ZERO,
	"rear_left": Vector3.ZERO,
	"rear_right": Vector3.ZERO
}

var _drift_mode: bool = false
var _drift_angle: float = 0.0
var _traction_control_active: bool = true
var _abs_active: bool = true
var _engine_braking: bool = true
var _lap_count: int = 0
var _best_lap_time: float = 0.0
var _current_lap_start: float = 0.0
var _vehicle_health: float = 1.0
var _fuel_level: float = 1.0
var _tire_wear: Dictionary = {
	"front_left": 0.0,
	"front_right": 0.0,
	"rear_left": 0.0,
	"rear_right": 0.0
}

var _torque_output: float = 0.0
var _brake_force: float = 0.0
var _downforce: float = 0.0
var _drag_force: float = 0.0
var _inertia_rotor: Vector3 = Vector3.ZERO
var _last_collision_time: float = 0.0
var _collision_buffer: Array[Dictionary] = []

# ============================================================================
# WHEEL POSITIONS (relative to vehicle center)
# ============================================================================

const WHEEL_OFFSETS: Dictionary = {
	"front_left": Vector3(-wheelbase * 0.45, track_width * 0.5, 0),
	"front_right": Vector3(-wheelbase * 0.45, -track_width * 0.5, 0),
	"rear_left": Vector3(wheelbase * 0.55, track_width * 0.5, 0),
	"rear_right": Vector3(wheelbase * 0.55, -track_width * 0.5, 0)
}

# ============================================================================
# ENGINE TORQUE CALCULATION
# ============================================================================

func _calculate_torque(rpm: float) -> float:
	if torque_curve_points.is_empty():
		# Default curve approximation based on displacement
		return _default_torque_curve(rpm)
	
	# Interpolate through torque curve points
	var torque = 0.0
	for i in range(torque_curve_points.size()):
		if i == torque_curve_points.size() - 1:
			break
		var point_a = torque_curve_points[i]
		var point_b = torque_curve_points[i + 1]
		
		if rpm >= point_a.x and rpm <= point_b.x:
			var t = (rpm - point_a.x) / (point_b.x - point_a.x)
			torque = point_a.y + t * (point_b.y - point_a.y)
			break
	
	return torque

func _default_torque_curve(rpm: float) -> float:
	var normalized_rpm = (rpm - 1000.0) / (8000.0 - 1000.0)
	normalized_rpm = clamp(normalized_rpm, 0.0, 1.0)
	
	# Quadratic peak torque curve
	var peak_rpm = 0.6
	var torque_peak = engine_displacement * 180.0 # Nm per liter
	var torque = torque_peak * (4.0 * normalized_rpm * (1.0 - normalized_rpm))
	
	return torque

# ============================================================================
# GEAR RATIO CALCULATION
# ============================================================================

func get_current_gear_ratio() -> float:
	if current_gear == 0 or current_gear > gear_ratios.size():
		return 0.0
	return gear_ratios[current_gear - 1]

func get_effective_ratio() -> float:
	return get_current_gear_ratio() * final_drive_ratio

# ============================================================================
# SPEED CONVERSIONS
# ============================================================================

func rpm_to_speed(rpm: float) -> float:
	if current_gear == 0 or gear_ratios.is_empty():
		return 0.0
	
	var effective_ratio = get_effective_ratio()
	var wheel_circumference = PI * 0.65 # ~0.65m tire diameter
	var speed_mps = (rpm * wheel_circumference) / (effective_ratio * 60.0)
	return speed_mps * 3.6 # Convert to km/h

func speed_to_rpm(speed_kmh: float) -> float:
	if current_gear == 0 or gear_ratios.is_empty():
		return 0.0
	
	var effective_ratio = get_effective_ratio()
	var wheel_circumference = PI * 0.65
	var speed_mps = speed_kmh / 3.6
	var rpm = (speed_mps * effective_ratio * 60.0) / wheel_circumference
	
	return max(0.0, rpm)

# ============================================================================
# PHYSICS UPDATE LOOP
# ============================================================================

func _physics_process(delta: float) -> void:
	if not is_inside_tree():
		return
	
	# Update internal state
	_update_physics_state(delta)
	
# ============================================================================
# PHYSICS STATE UPDATES
# ============================================================================

func _update_physics_state(delta: float) -> void:
	# Calculate engine torque based on current RPM
	_torque_output = _calculate_torque(current_rpm) * throttle_input
	
	# Apply gear reduction if clutch engaged
	if clutch_engaged and current_gear > 0:
		_torque_output /= get_effective_ratio()
	else:
		_torque_output = 0.0
	
	# Calculate aerodynamic forces
	_downforce = _calculate_downforce()
	_drag_force = _calculate_drag()
	
	# Calculate brake force
	_brake_force = _calculate_brake_force()
	
	# Apply forces to velocity
	_apply_vehicle_forces(delta)
	
	# Update wheel states
	_update_wheels(delta)
	
	# Handle gear shifting
	_handle_gear_shifting(delta)
	
	# Check drift conditions
	_check_drift_conditions()
	
	# Update telemetry signals
	_emit_telemetry_signals()
	
# ============================================================================
# FORCE CALCULATIONS
# ============================================================================

func _calculate_downforce() -> float:
	var speed_ms = current_speed / 3.6
	var dynamic_pressure = 0.5 * 1.225 * pow(speed_ms, 2)
	return dynamic_pressure * drag_coefficient * frontal_area * downforce_coefficient

func _calculate_drag() -> float:
	var speed_ms = current_speed / 3.6
	var dynamic_pressure = 0.5 * 1.225 * pow(speed_ms, 2)
	return dynamic_pressure * drag_coefficient * frontal_area

func _calculate_brake_force() -> float:
	var total_brake_pressure = (brake_pressure_front * brake_bias_front) + 
	                           (brake_pressure_rear * (1.0 - brake_bias_front))
	
	var max_brake_force = total_brake_pressure * brake_caliper_efficiency * brake_disc_diameter * PI / 4.0
	var actual_brake_force = max_brake_force * brake_input * brake_pad_friction
	
	return min(actual_brake_force, max_brake_force)

# ============================================================================
# VEHICLE DYNAMICS APPLICATION
# ============================================================================

func _apply_vehicle_forces(delta: float) -> void:
	# Calculate acceleration forces
	var drive_force = _torque_output / 0.325 # Divide by wheel radius
	drive_force *= 0.9 # Drivetrain efficiency
	
	# Apply acceleration or deceleration
	var net_force = drive_force - _brake_force - _drag_force
	
	# Apply to velocity
	var acceleration = net_force / curb_weight
	var velocity_change = acceleration * delta
	
	current_speed += velocity_change * 3.6 # Convert m/s² to km/h
	current_speed = clamp(current_speed, 0.0, 350.0) # Cap max speed
	
	# Update angular velocity from body
	angular_velocity = Vector3.ZERO
	
# ============================================================================
# WHEEL STATE MANAGEMENT
# ============================================================================

func _update_wheels(delta: float) -> void:
	# Update each wheel's slip ratio and angle
	for wheel_name in WHEEL_OFFSETS.keys():
		var wheel_offset = WHEEL_OFFSETS[wheel_name]
		
		# Calculate wheel position in world space
		var wheel_pos = global_position + transform.basis * wheel_offset
		
		# Get wheel velocity
		var wheel_velocity = velocity
		
		# Calculate slip ratio (difference between wheel speed and vehicle speed)
		var wheel_radius = 0.325
		var wheel_rotational_speed = current_speed / 3.6 / wheel_radius
		
		_wheel_slip_ratios[wheel_name] = abs(wheel_rotational_speed - velocity.length()) / max(1.0, wheel_rotational_speed)
		
		# Apply wear based on slip
		_tire_wear[wheel_name] += _wheel_slip_ratios[wheel_name] * delta * 0.01
		_tire_wear[wheel_name] = min(1.0, _tire_wear[wheel_name])
		
		# Emit wheel slip signal
		if _wheel_slip_ratios[wheel_name] > 0.1:
			emit_signal("wheel_slip", ["front_left" if wheel_name.contains("left") else "front_right" if wheel_name.contains("front") else "rear_left", _wheel_slip_ratios[wheel_name]])

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================

func _handle_gear_shifting(delta: float) -> void:
	# Automatic upshifts
	if current_gear < gear_ratios.size() and current_rpm > 7000.0 and not throttle_input < 0.1:
		target_gear = min(current_gear + 1, gear_ratios.size())
	
	# Automatic downshifts
	elif current_gear > 1 and current_rpm < 2500.0:
		target_gear = max(current_gear - 1, 1)
	
	# Manual gear override
	if abs(target_gear - current_gear) > 0:
		_shift_gear(target_gear)

func shift_gear(gear: int) -> void:
	target_gear = clamp(gear, 0, gear_ratios.size())
	
	if gear == 0:
		# Neutral
		clutch_engaged = false
		_torque_output = 0.0
	elif gear > 0:
		# Engage gear
		clutch_engaged = true
		_shift_gear(gear)

func _shift_gear(new_gear: int) -> void:
	if new_gear == current_gear:
		return
	
	var old_gear = current_gear
	current_gear = new_gear
	target_gear = new_gear
	
	# Short shift delay for realism
	await get_tree().create_timer(0.15).timeout
	
	# Calculate target RPM
	var target_rpm = speed_to_rpm(current_speed)
	target_rpm = clamp(target_rpm, 1000.0, 8500.0)
	
	# Smooth RPM transition
	var rpm_change = target_rpm - current_rpm
	var rpm_delta_per_frame = rpm_change / 10.0
	
	for i in range(10):
		current_rpm += rpm_delta_per_frame
		current_rpm = clamp(current_rpm, 1000.0, 8500.0)
		emit_signal("rpm_changed", current_rpm)
		await get_tree().process_frame
	
	# If manual shift, keep clutch disengaged briefly
	if new_gear != old_gear:
		clutch_engaged = true
	
	emit_signal("gear_changed", new_gear)

# ============================================================================
# DRIFT MECHANICS
# ============================================================================

func _check_drift_conditions() -> void:
	var lateral_acceleration = velocity.length() * sin(abs(_drift_angle))
	var lateral_threshold = 0.5 * _physics.gravity
	
	if abs(lateral_acceleration) > lateral_threshold and throttle_input > 0.3:
		if not _drift_mode:
			_drift_mode = true
			emit_signal("drift_started")
	else:
		_drift_mode = false
		emit_signal("drift_ended")

func calculate_drift_score() -> float:
	if not _drift_mode:
		return 0.0
	
	return (_drift_angle * throttle_input * abs(velocity.length())) / 100.0

# ============================================================================
# INPUT HANDLING METHODS
# ============================================================================

func set_throttle(input_value: float) -> void:
	throttle_input = clamp(input_value, 0.0, 1.0)

func set_brake(input_value: float) -> void:
	brake_input = clamp(input_value, 0.0, 1.0)

func set_steering(input_value: float) -> void:
	steering_input = clamp(input_value, -1.0, 1.0)
	
	# Calculate wheel angles based on steering input
	var max_steering_angle = steering_lock_degrees / steering_ratio
	for wheel_name in _wheel_angles.keys():
		if wheel_name.contains("front"):
			_wheel_angles[wheel_name] = steering_input * max_steering_angle
		else:
			_wheel_angles[wheel_name] = 0.0 # Rear wheels don't steer

func set_clutch(engaged: bool) -> void:
	clutch_engaged = engaged
	if not engaged:
		_torque_output = 0.0

# ============================================================================
# COLLISION DETECTION
# ============================================================================

func _on_collision_body_entered(body: Node) -> void:
	if body.has_method("get_collision_damage"):
		var damage = body.get_collision_damage()
		_vehicle_health -= damage
		
		if _vehicle_health <= 0:
			_trigger_crash_sequence()

func _trigger_crash_sequence() -> void:
	print("Vehicle destroyed!")
	queue_free()

func register_collision(impact_force: float, impact_point: Vector3) -> void:
	_last_collision_time = Time.get_ticks_msec()
	_collision_buffer.append({
		"time": _last_collision_time,
		"force": impact_force,
		"point": impact_point
	})
	
	# Keep buffer size manageable
	while _collision_buffer.size() > 100:
		_collision_buffer.pop_front()
	
	emit_signal("collision_impact", impact_force, impact_point)

# ============================================================================
# TELEMTRY AND SIGNALS
# ============================================================================

func _emit_telemetry_signals() -> void:
	if abs(current_speed - speed_changed.last_value) > 1.0:
		emit_signal("speed_changed", current_speed)
		speed_changed.last_value = current_speed
	
	if abs(current_rpm - rpm_changed.last_value) > 100.0:
		emit_signal("rpm_changed", current_rpm)
		rpm_changed.last_value = current_rpm

# ============================================================================
# LAP TIMING SYSTEM
# ============================================================================

func start_lap() -> void:
	_current_lap_start = Time.get_ticks_usec() / 1_000_000.0

func end_lap() -> void:
	var lap_time = Time.get_ticks_usec() / 1_000_000.0 - _current_lap_start
	_lap_count += 1
	
	if _best_lap_time == 0.0 or lap_time < _best_lap_time:
		_best_lap_time = lap_time
	
	return lap_time

func get_lap_times() -> Dictionary:
	return {
		"current_lap": Time.get_ticks_usec() / 1_000_000.0 - _current_lap_start,
		"best_lap": _best_lap_time,
		"total_laps": _lap_count
	}

# ============================================================================
# VEHICLE HEALTH & FUEL
# ============================================================================

func take_damage(damage_amount: float) -> void:
	_vehicle_health = max(0.0, _vehicle_health - damage_amount)
	
	# Engine damage reduces max power
	max_engine_power *= _vehicle_health
	
	return _vehicle_health

func check_fuel_consumption(delta: float) -> void:
	var consumption_rate = 0.01 * throttle_input * (current_rpm / 8000.0)
	_fuel_level = max(0.0, _fuel_level - consumption_rate * delta)
	
	if _fuel_level <= 0.0:
		_engine_stopped()

func _engine_stopped() -> void:
	throttle_input = 0.0
	current_rpm = 0.0
	current_speed = 0.0
	vehicle_health = 0.0

# ============================================================================
# DEBUG & VISUALIZATION HELPERS
# ============================================================================

func debug_print_status() -> void:
	print("\n=== VEHICLE STATUS ===")
	print("Speed: %.1f km/h" % current_speed)
	print("RPM: %.0f" % current_rpm)
	print("Gear: %d/%d" % [current_gear, gear_ratios.size()])
	print("Throttle: %.2f" % throttle_input)
	print("Brake: %.2f" % brake_input)
	print("Steering: %.2f" % steering_input)
	print("Drift Mode: %s" % ("YES" if _drift_mode else "NO"))
	print("Health: %.1f%%" % (_vehicle_health * 100))
	print("Fuel: %.1f%%" % (_fuel_level * 100))
	print("====================\n")

func get_debug_data() -> Dictionary:
	return {
		"speed": current_speed,
		"rpm": current_rpm,
		"gear": current_gear,
		"throttle": throttle_input,
		"brake": brake_input,
		"steering": steering_input,
		"drift_mode": _drift_mode,
		"health": _vehicle_health,
		"fuel": _fuel_level,
		"tire_wear": _tire_wear.duplicate(),
		"slip_ratios": _wheel_slip_ratios.duplicate(),
		"forces": {
			"drive": _torque_output,
			"brake": _brake_force,
			"drag": _drag_force,
			"downforce": _downforce
		}
	}

# ============================================================================
# SETUP & CLEANUP
# ============================================================================

func _ready() -> void:
	_setup_default_torque_curve()
	_init_vehicle_state()

func _setup_default_torque_curve() -> void:
	if torque_curve_points.is_empty():
		torque_curve_points = [
			Vector2(1000, 100),
			Vector2(2000, 250),
			Vector2(3500, 350),
			Vector2(5000, 400),
			Vector2(6500, 380),
			Vector2(8000, 250),
			Vector2(9000, 100)
		]

func _init_vehicle_state() -> void:
	current_speed = 0.0
	current_rpm = 1000.0
	current_gear = 1
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	_vehicle_health = 1.0
	_fuel_level = 1.0
	_lap_count = 0
	_best_lap_time = 0.0

func _notification(what: int) -> void:
	match what:
		NOTIFICATION_WM_CLOSE_REQUEST:
			save_setup()
		NOTIFICATION_EXIT_TREE:
			print("VehicleController destroyed")

func _get_configuration_warnings() -> Array[String]:
	var warnings = []
	
	if torque_curve_points.is_empty():
		warnings.append("Torque curve is empty - using default")
	if gear_ratios.is_empty():
		warnings.append("No gear ratios defined")
	if not _validate_vehicle_config():
		warnings.append("Vehicle configuration has errors")
	
	return warnings

func _validate_vehicle_config() -> bool:
	if max_engine_power <= 0:
		return false
	if curb_weight <= 0:
		return false
	if tire_width <= 0:
		return false
	
	return true

func save_setup() -> void:
	var data = {
		"max_engine_power": max_engine_power,
		"curb_weight": curb_weight,
		"center_of_mass_height": center_of_mass_height,
		"track_width": track_width,
		"wheelbase": wheelbase,
		"tire_width": tire_width,
		"tire_friction_coefficient": tire_friction_coefficient,
		"gear_ratios": gear_ratias,
		"differential_type": differential_type
	}
	
	# Save to project settings or file
	ProjectSettings.set_setting("vehicle/custom_config", data)
	ProjectSettings.save()

func load_setup() -> void:
	if ProjectSettings.has_setting("vehicle/custom_config"):
		var config = ProjectSettings.get_setting("vehicle/custom_config")
		max_engine_power = config.max_engine_power
		curb_weight = config.curb_weight
		center_of_mass_height = config.center_of_mass_height
		track_width = config.track_width
		wheelbase = config.wheelbase
		tire_width = config.tire_width
		tire_friction_coefficient = config.tire_friction_coefficient
		gear_ratios = config.gear_ratios
		differential_type = config.differential_type

func reset_vehicle() -> void:
	current_speed = 0.0
	current_rpm = 1000.0
	current_gear = 1
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	_vehicle_health = 1.0
	_fuel_level = 1.0
	_lap_count = 0
	_best_lap_time = 0.0
	_tire_wear.clear()
	_collision_buffer.clear()

func set_track_condition(condition: String) -> void:
	match condition:
		"dry":
			tire_friction_coefficient = 1.2
		"wet":
			tire_friction_coefficient = 0.85
		"rain":
			tire_friction_coefficient = 0.55
		"snow":
			tire_friction_coefficient = 0.35
		"ice":
			tire_friction_coefficient = 0.15

func apply_custom_torque_curve(points: Array[Vector2]) -> void:
	torque_curve_points = points

func get_performance_metrics() -> Dictionary:
	return {
		"acceleration_0_100": _calculate_acceleration_time(0, 100),
		"top_speed": _calculate_top_speed(),
		"braking_distance": _calculate_braking_distance(),
		"power_to_weight": max_engine_power / curb_weight,
		"current_lap": get_lap_times()
	}

func _calculate_acceleration_time(start_speed: float, end_speed: float) -> float:
	var time_accumulator = 0.0
	var current = start_speed
	
	while current < end_speed:
		var delta_time = 1.0 / _physics.physics_tick_rate
		var torque = _calculate_torque(speed_to_rpm(current))
		var force = torque * get_effective_ratio() / 0.325
		var acceleration = force / curb_weight
		current += acceleration * delta_time * 3.6
		time_accumulator += delta_time
	
	return time_accumulator

func _calculate_top_speed() -> float:
	var speed = 0.0
	var last_speed = 0.0
	
	while speed < last_speed:
		last_speed = speed
		var ms = speed / 3.6
		var drag = 0.5 * 1.225 * drag_coefficient * frontal_area * pow(ms, 2)
		var torque = _calculate_torque(speed_to_rpm(speed))
		var force = torque * get_effective_ratio() / 0.325
		var acceleration = (force - drag) / curb_weight
		speed += acceleration * 0.01 * 3.6
	
	return last_speed

func _calculate_braking_distance() -> float:
	var distance = 0.0
	var speed = current_speed
	
	while speed > 0:
		var ms = speed / 3.6
		var brake_force = _calculate_brake_force()
		var deceleration = brake_force / curb_weight
		distance += ms * 0.01
		speed -= deceleration * 0.01 * 3.6
	
	return distance

</FILE>