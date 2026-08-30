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
# VEHICLE CONFIGURATION
# ============================================================================

@export_group("Vehicle Configuration")
@export var vehicle_mass: float = 1500.0
@export var center_of_mass_offset: Vector3 = Vector3(0.0, 0.5, 0.0)
@export var wheel_base: float = 2.8
@export var track_width: float = 1.8
@export var ground_clearance: float = 0.25
@export var drag_coefficient: float = 0.30
@export var frontal_area: float = 2.2
@export var roll_stiffness_front: float = 12000.0
@export var roll_stiffness_rear: float = 10000.0
@export var camber_angle_front: float = -0.5
@export var camber_angle_rear: float = -0.5
@export var toe_angle_front: float = 0.02
@export var toe_angle_rear: float = 0.02

@export_group("Powertrain Parameters")
@export var engine_max_rpm: float = 7500.0
@export var engine_min_rpm: float = 800.0
@export var idle_rpm: float = 900.0
@export var max_torque: float = 450.0
@export var torque_curve_points: Array[Vector2] = []
@export var gear_ratios: Array[float] = [3.8, 2.2, 1.5, 1.1, 0.9, 0.75, 0.6]
@export var final_drive_ratio: float = 3.5
@export var transmission_efficiency: float = 0.95
@export var clutch_engagement_rpm: float = 1200.0
@export var rev_matching_enabled: bool = true

@export_group("Tire & Suspension")
@export var tire_friction_static: float = 1.1
@export var tire_friction_dynamic: float = 0.9
@export var suspension_travel: float = 0.15
@export var spring_constant: float = 35000.0
@export var damper_constant: float = 2500.0
@export var anti_roll_bar_stiffness: float = 5000.0

@export_group("Drift Settings")
@export var drift_threshold: float = 0.3
@export var drift_recovery_rate: float = 0.85
@export var drift_multiplier: float = 0.7
@export var min_drift_speed: float = 15.0

@export_group("Braking System")
@export var brake_force_per_wheel: float = 12000.0
@export var abs_enabled: bool = true
@export var brake_bias_front: float = 0.6
@export var emergency_brake_force: float = 15000.0

# ============================================================================
# STATE VARIABLES
# ============================================================================

# Current state
var current_speed: float = 0.0
var current_rpm: float = idle_rpm
var current_gear: int = 0
var target_gear: int = 0
var is_in_neutral: bool = false
var is_engine_running: bool = true

# Input values (normalized -1 to 1)
var throttle_input: float = 0.0
var brake_input: float = 0.0
var steering_input: float = 0.0
var reverse_input: bool = false

# Wheel states (0-3: FL, FR, RL, RR)
var wheel_rotations: Array[float] = [0.0, 0.0, 0.0, 0.0]
var wheel_slip_ratios: Array[float] = [0.0, 0.0, 0.0, 0.0]
var wheel_contact_forces: Array[float] = [0.0, 0.0, 0.0, 0.0]

# Drift state
var drift_state: bool = false
var drift_angle: float = 0.0
var lateral_velocity: float = 0.0
var drift_progression: float = 0.0

# Aerodynamics
var air_density: float = 1.225
var downforce: float = 0.0
var lift: float = 0.0

# Collision tracking
var last_collision_time: float = 0.0
var collision_impact_force: float = 0.0

# ============================================================================
# WHEEL POSITIONS (local space)
# ============================================================================

const WHEEL_FL: Vector3 = Vector3(track_width * 0.5, -ground_clearance, wheel_base * 0.5)
const WHEEL_FR: Vector3 = Vector3(-track_width * 0.5, -ground_clearance, wheel_base * 0.5)
const WHEEL_RL: Vector3 = Vector3(track_width * 0.5, -ground_clearance, -wheel_base * 0.5)
const WHEEL_RR: Vector3 = Vector3(-track_width * 0.5, -ground_clearance, -wheel_base * 0.5)

var _wheel_positions: Array[Vector3] = [WHEEL_FL, WHEEL_FR, WHEEL_RL, WHEEL_RR]

# ============================================================================
# ENGINE TORQUE CURVE CACHE
# ============================================================================

var _torque_cache: Dictionary = {}

func _get_torque_at_rpm(rpm: float) -> float:
	if torque_curve_points.is_empty():
		return _calculate_default_torque(rpm)
	
	if _torque_cache.has(rpm):
		return _torque_cache[rpm]
	
	var torque = _interpolate_torque_curve(rpm)
	_torque_cache[rpm] = torque
	return torque

func _calculate_default_torque(rpm: float) -> float:
	var rpm_normalized = (rpm - engine_min_rpm) / (engine_max_rpm - engine_min_rpm)
	rpm_normalized = clamp(rpm_normalized, 0.0, 1.0)
	
	var peak_rpm = engine_max_rpm * 0.7
	var torque_peak = max_torque
	
	if rpm < engine_min_rpm:
		return 0.0
	
	if rpm <= peak_rpm:
		return max_torque * (rpm / peak_rpm)
	else:
		return max_torque * exp(-(rpm - peak_rpm) / (peak_rpm * 0.5))

func _interpolate_torque_curve(rpm: float) -> float:
	var points = torque_curve_points
	if points.size() < 2:
		return _calculate_default_torque(rpm)
	
	for i in range(points.size() - 1):
		var point1 = points[i]
		var point2 = points[i + 1]
		
		if rpm >= point1.x and rpm <= point2.x:
			var t = (rpm - point1.x) / (point2.x - point1.x)
			return point1.y + t * (point2.y - point1.y)
	
	if rpm > points[-1].x:
		return points[-1].y
	elif rpm < points[0].x:
		return points[0].y
	
	return 0.0

# ============================================================================
# GEAR CALCULATION
# ============================================================================

func get_current_gear_ratio() -> float:
	if is_in_neutral or current_gear < 0:
		return 0.0
	if current_gear >= gear_ratios.size():
		current_gear = gear_ratios.size() - 1
	return gear_ratios[current_gear] * final_drive_ratio

func calculate_wheel_rpm_from_speed(speed_kmh: float) -> float:
	var wheel_radius = 0.33
	var circumference = 2 * PI * wheel_radius
	var speed_ms = speed_kmh / 3.6
	var wheel_rps = speed_ms / circumference
	return wheel_rps * 60.0

func calculate_speed_from_rpm(rpm: float) -> float:
	var wheel_radius = 0.33
	var circumference = 2 * PI * wheel_radius
	var wheel_rps = rpm / 60.0
	var speed_ms = wheel_rps * circumference * get_current_gear_ratio()
	return speed_ms * 3.6

func shift_up() -> void:
	if current_gear < gear_ratios.size() - 1:
		target_gear = current_gear + 1
		_trigger_shift()

func shift_down() -> void:
	if current_gear > 0:
		target_gear = current_gear - 1
		_trigger_shift()

func set_gear(gear: int) -> void:
	if gear < -1:
		gear = -1
	elif gear >= gear_ratios.size():
		gear = gear_ratios.size() - 1
	target_gear = gear
	if gear == -1:
		is_in_neutral = true
	else:
		is_in_neutral = false
		_trigger_shift()

func _trigger_shift() -> void:
	if target_gear != current_gear:
		current_gear = target_gear
		gear.emit(current_gear)
		_play_shift_sound()

# ============================================================================
# INPUT HANDLING
# ============================================================================

func _process_inputs(delta: float) -> void:
	throttle_input = InputManager.get_axis("throttle_forward", "throttle_backward")
	brake_input = InputManager.get_axis("brake_pedal", "emergency_brake")
	steering_input = InputManager.get_axis("steer_left", "steer_right")
	reverse_input = InputManager.action_pressed("reverse_mode")

# ============================================================================
# PHYSICS UPDATE
# ============================================================================

func _physics_process(delta: float) -> void:
	_process_inputs(delta)
	_update_engine_and_transmission(delta)
	_update_vehicle_dynamics(delta)
	_update_aerodynamics(delta)
	_update_wheels(delta)
	_update_drift(delta)
	_check_clutch_and_gear(delta)
	_publish_signals()

func _update_engine_and_transmission(delta: float) -> void:
	var effective_throttle = throttle_input if not is_in_neutral else 0.0
	var effective_brake = brake_input
	
	var target_rpm = idle_rpm
	if effective_throttle > 0.0:
		target_rpm = _calculate_target_rpm(effective_throttle)
	elif effective_brake > 0.0 and current_speed > 0.1:
		target_rpm = _calculate_brake_drag_rpm(effective_brake)
	
	current_rpm = _lerp_rpm(current_rpm, target_rpm, delta)

func _calculate_target_rpm(throttle: float) -> float:
	var target_ratio = get_current_gear_ratio()
	if target_ratio <= 0:
		return idle_rpm
	
	var speed_factor = current_speed / 120.0
	var rpm_increase = (engine_max_rpm - idle_rpm) * throttle
	return min(engine_max_rpm, idle_rpm + rpm_increase * (1.0 + speed_factor * 0.5))

func _calculate_brake_drag_rpm(brake: float) -> float:
	var drag_factor = brake * 0.3
	return max(idle_rpm, current_rpm * (1.0 - drag_factor))

func _lerp_rpm(current: float, target: float, delta: float) -> float:
	var lerp_amount = 5.0 * delta
	return lerp(current, target, lerp_amount)

func _update_vehicle_dynamics(delta: float) -> void:
	var forward_vector = transform.basis.z * -1.0
	var acceleration = _calculate_acceleration(delta)
	var deceleration = _calculate_deceleration(delta)
	
	var net_acceleration = acceleration - deceleration
	
	current_speed += net_acceleration * delta
	current_speed = max(0.0, current_speed)
	
	var velocity_magnitude = global_velocity.length()
	var direction = global_velocity.normalized() if velocity_magnitude > 0.1 else forward_vector
	
	global_velocity = direction * current_speed

func _calculate_acceleration(delta: float) -> float:
	if is_in_neutral or not is_engine_running:
		return 0.0
	
	var gear_ratio = get_current_gear_ratio()
	if gear_ratio <= 0:
		return 0.0
	
	var wheel_torque = _calculate_wheel_torque()
	var wheel_radius = 0.33
	var drive_force = (wheel_torque * gear_ratio * transmission_efficiency) / wheel_radius
	
	var effective_mass = vehicle_mass
	var friction_loss = 0.01 * vehicle_mass * 9.81
	
	return (drive_force - friction_loss) / effective_mass

func _calculate_wheel_torque() -> float:
	var rpm = current_rpm
	var base_torque = _get_torque_at_rpm(rpm)
	var throttle = throttle_input
	var efficiency = 1.0 if rpm > clutch_engagement_rpm else 0.3
	
	return base_torque * throttle * efficiency

func _calculate_deceleration(delta: float) -> float:
	var braking_force = 0.0
	
	if brake_input > 0.0:
		var abs_modulation = 1.0 if not abs_enabled else _calculate_abs_modulation()
		braking_force = brake_force_per_wheel * brake_input * abs_modulation
	
	var total_braking_force = braking_force * 4.0
	var mass = vehicle_mass
	
	return total_braking_force / mass if mass > 0 else 0.0

func _calculate_abs_modulation() -> float:
	var modulation = 1.0
	for slip in wheel_slip_ratios:
		if abs(slip) > 0.2:
			modulation -= 0.25
	return max(0.0, modulation)

func _update_aerodynamics(delta: float) -> void:
	var speed_sq = current_speed * current_speed / 3.6 / 3.6
	var dynamic_pressure = 0.5 * air_density * speed_sq
	
	downforce = dynamic_pressure * drag_coefficient * frontal_area * 0.8
	lift = dynamic_pressure * drag_coefficient * frontal_area * 0.1
	
	var aerodynamic_drag = dynamic_pressure * drag_coefficient * frontal_area
	var drag_acceleration = aerodynamic_drag / vehicle_mass
	
	current_speed -= drag_acceleration * delta
	current_speed = max(0.0, current_speed)

func _update_wheels(delta: float) -> void:
	var wheel_radius = 0.33
	var rotation_increment = current_speed / 3.6 / wheel_radius * delta
	
	for i in range(4):
		wheel_rotations[i] += rotation_increment
		
		var slip = _calculate_wheel_slip(i)
		wheel_slip_ratios[i] = slip
		
		var force = _calculate_wheel_contact_force(i)
		wheel_contact_forces[i] = force
		
		if abs(slip) > 0.1:
			wheel_slip.emit(i, slip)

func _calculate_wheel_slip(wheel_index: int) -> float:
	var wheel_radius = 0.33
	var rotational_speed = wheel_rotations[wheel_index] / wheel_radius * 3.6
	var slip = (rotational_speed - current_speed) / current_speed if current_speed > 0.1 else 0.0
	return clamp(slip, -1.0, 1.0)

func _calculate_wheel_contact_force(wheel_index: int) -> float:
	var normal_force = vehicle_mass * 9.81 / 4.0
	var load_transfer = _calculate_load_transfer(wheel_index)
	return normal_force + load_transfer

func _calculate_load_transfer(wheel_index: int) -> float:
	var acceleration_z = current_speed * 0.1
	var cg_height = center_of_mass_offset.y
	var wheelbase = wheel_base
	
	var front_weight_transfer = acceleration_z * cg_height * vehicle_mass / wheelbase
	var rear_weight_transfer = -front_weight_transfer
	
	match wheel_index:
		0: return rear_weight_transfer * 0.25
		1: return rear_weight_transfer * 0.25
		2: return front_weight_transfer * 0.25
		3: return front_weight_transfer * 0.25
		_: return 0.0

func _update_drift(delta: float) -> void:
	if current_speed < min_drift_speed:
		_end_drift()
		return
	
	var lateral_component = lateral_velocity
	var longitudinal_component = current_speed
	
	var slip_angle = atan(abs(lateral_component) / max(longitudinal_component, 0.1))
	var lateral_acceleration = input_lateral_acceleration()
	
	if slip_angle > drift_threshold or lateral_acceleration > drift_threshold:
		if not drift_state:
			_start_drift()
		else:
			_continue_drift(delta, slip_angle)
	else:
		_continue_drift_recovery(delta)

func input_lateral_acceleration() -> float:
	return steering_input * 9.81 * 0.3

func _start_drift() -> void:
(drift_state = true
drift_progression = 0.0
lateral_velocity = current_speed * drift_threshold
drift_started.emit()

func _continue_drift(delta: float, slip_angle: float) -> void:
drift_progression += delta * 2.0
if drift_progression > 1.0:
drift_progression = 1.0
	
	var drift_scale = 1.0 + drift_progression * 0.5
	lateral_velocity = current_speed * drift_threshold * drift_scale
	
	var recovery = drift_recovery_rate * delta
	current_speed *= recovery

func _continue_drift_recovery(delta: float) -> void:
if drift_state:
drift_progression -= delta * 3.0
if drift_progression <= 0.0:
_end_drift()
else:
current_speed *= drift_recovery_rate

func _end_drift() -> void:
if drift_state:
drift_state = false
drift_progression = 0.0
lateral_velocity = 0.0
drift_ended.emit()

func _check_clutch_and_gear(delta: float) -> void:
if is_in_neutral:
return

if current_rpm < clutch_engagement_rpm and throttle_input > 0.0:
	var clutch_effectiveness = current_rpm / clutch_engagement_rpm
	throttle_input *= clutch_effectiveness

if rev_matching_enabled and target_gear != current_gear:
	var target_rpm_for_gear = _calculate_target_rpm_for_gear(target_gear)
	current_rpm = lerp(current_rpm, target_rpm_for_gear, delta * 3.0)

func _calculate_target_rpm_for_gear(gear: int) -> float:
	var gear_ratio = gear_ratios[gear] if gear >= 0 else 1.0
	var wheel_rpm = current_speed / 3.6 / 0.33 * 60.0
	return wheel_rpm * gear_ratio * final_drive_ratio

func _publish_signals() -> void:
speed_changed.emit(current_speed)
rpm_changed.emit(current_rpm)

func _play_shift_sound() -> void:
AudioManager.play_sfx("gear_shift")

# ============================================================================
# COLLISION HANDLING
# ============================================================================

func _on_body_entered(body: Node) -> void:
if body.has_method("on_vehicle_collision"):
body.on_vehicle_collision(global_position, linear_velocity)

func _on_body_exited(body: Node) -> void:
pass

func handle_collision(impact_vector: Vector3, impact_point: Vector3) -> void:
collision_impact_force = impact_vector.length()
last_collision_time = Time.get_unix_time_from_system()

if collision_impact_force > 5000.0:
collision_impact.emit(collision_impact_force, impact_point)

# ============================================================================
# HELPER METHODS
# ============================================================================

func reset_vehicle() -> void:
current_speed = 0.0
current_rpm = idle_rpm
current_gear = 0
target_gear = 0
is_in_neutral = false
throttle_input = 0.0
brake_input = 0.0
steering_input = 0.0
drift_state = false
drift_progression = 0.0
lateral_velocity = 0.0

for i in range(4):
wheel_rotations[i] = 0.0
wheel_slip_ratios[i] = 0.0
wheel_contact_forces[i] = 0.0

func get_wheel_position(index: int) -> Vector3:
if index < 0 or index >= 4:
return Vector3.ZERO
return _wheel_positions[index] + global_position

func set_vehicle_mass(new_mass: float) -> void:
vehicle_mass = new_mass

func get_total_downforce() -> float:
return downforce

func get_total_lift() -> float:
return lift

func is_drifting() -> bool:
return drift_state

func get_current_gear_name() -> String:
match current_gear:
	0: return "N"
	1: return "1"
	2: return "2"
	3: return "3"
	4: return "4"
	5: return "5"
	6: return "6"
	7: return "R"
	_: return "?"
return "N"

func get_vehicle_stats() -> Dictionary:
return {
	"speed": current_speed,
	"rpm": current_rpm,
	"gear": current_gear,
	"is_drifting": drift_state,
	"downforce": downforce,
	"throttle": throttle_input,
	"brake": brake_input,
	"steering": steering_input
}
