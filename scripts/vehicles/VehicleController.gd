extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulation
## Handles throttle, brake, steering, wheel forces, gear shifting, and vehicle dynamics
## Integrates with PhysicsSettings, Powertrain, and InputManager systems
## Copyright 2026 Thalamus Racing Simulator Project

signal speed_changed(current_speed: float)
signal rpm_changed(rpm: float)
signal gear_changed(old_gear: int, new_gear: int)
signal drift_started()
signal drift_ended()
signal collision_detected(collision_info: Dictionary)
signal lap_completed(lap_data: Dictionary)
signal race_finished(position: int, time: float)
signal engine_stalled()

# ============================================================================
# INPUT VALUES (from InputManager)
# ============================================================================

@export var _throttle_input: float = 0.0: set = _set_throttle_input
@export var _brake_input: float = 0.0: set = _set_brake_input
@export var _steering_input: float = 0.0: set = _set_steering_input
@export var _clutch_input: float = 0.0: set = _set_clutch_input
@export var _handbrake_input: float = 0.0: set = _set_handbrake_input

var _last_throttle: float = 0.0
var _last_brake: float = 0.0
var _last_steering: float = 0.0

# ============================================================================
# VEHICLE PHYSICS STATE
# ============================================================================

var current_speed: float = 0.0  # Speed in m/s
var max_speed: float = 65.0  # Max forward speed m/s
var reverse_speed: float = 20.0  # Max reverse speed m/s
var acceleration: float = 0.0
var deceleration: float = 0.0

var rotation_velocity: float = 0.0  # Yaw rate rad/s
var slip_angle: float = 0.0  # Tire slip angle
var lateral_acceleration: float = 0.0  # G-force lateral
var longitudinal_acceleration: float = 0.0  # G-force longitudinal

# ============================================================================
# GEARBOX SYSTEM
# ============================================================================

enum Gear {
	NEUTRAL = -1,
	REVERSE = 0,
	FIRST = 1,
	SECOND = 2,
	THIRD = 3,
	FOURTH = 4,
	FIFTH = 5,
	SIXTH = 6,
	SEVENTH = 7,
	EIGHTH = 8
}

var current_gear: Gear = Gear.NEUTRAL
var target_gear: Gear = Gear.NEUTRAL
var engine_rpm: float = 0.0
var redline_rpm: float = 7500.0
var idle_rpm: float = 800.0
var torque_curve: Array[float] = []

@export_group("Gear Ratios")
@export var gear_ratios: Array[float] = [3.8, 2.4, 1.6, 1.2, 0.9, 0.75, 0.6, 0.5]
@export var final_drive_ratio: float = 3.5
@export var transmission_type: String = "manual"  # manual, automatic, sequential

@export_group("Shift Logic")
@export var shift_up_rpm_threshold: float = 6500.0
@export var shift_down_rpm_threshold: float = 3000.0
@export var auto_shift_enabled: bool = true
@export var rev_matching_enabled: bool = true

func _init() -> void:
	torque_curve = _generate_torque_curve()

func _generate_torque_curve() -> Array[float]:
	"""Generate engine torque curve based on RPM percentage"""
	var curve: Array[float] = []
	for i in range(101):
		var rpm_percent = float(i) / 100.0
		var normalized_rpm = lerp(0.2, 1.0, pow(rpm_percent, 0.8))
		var torque = normalized_rpm * (1.0 + sin(rpm_percent * PI * 2) * 0.1)
		curve.append(torque)
	return curve

func _set_throttle_input(value: float) -> void:
	_throttle_input = clamp(value, -1.0, 1.0)

func _set_brake_input(value: float) -> void:
	_brake_input = clamp(value, -1.0, 1.0)

func _set_steering_input(value: float) -> void:
	_steering_input = clamp(value, -1.0, 1.0)

func _set_clutch_input(value: float) -> void:
	_clutch_input = clamp(value, -1.0, 1.0)

func _set_handbrake_input(value: float) -> void:
	_handbrake_input = clamp(value, -1.0, 1.0)

func _update_gearbox() -> void:
	if _clutch_input > 0.5:
		current_gear = Gear.NEUTRAL
		return
	
	match transmission_type:
		"manual":
			_manual_shift_logic()
		"automatic":
			_automatic_shift_logic()
		"sequential":
			_sequential_shift_logic()

func _manual_shift_logic() -> void:
	if not auto_shift_enabled:
		if _throttle_input < 0.1 and current_gear > Gear.FIRST:
			target_gear = current_gear - 1
		elif _throttle_input > 0.8 and current_gear < Gear.EIGHTH:
			target_gear = current_gear + 1
		else:
			target_gear = current_gear
	
	_perform_gear_shift()

func _automatic_shift_logic() -> void:
	var speed_ratio = abs(current_speed) / max_speed
	if speed_ratio < 0.1 and current_gear > Gear.REVERSE:
		target_gear = Gear.FIRST if current_speed >= 0 else Gear.REVERSE
	elif speed_ratio < 0.25:
		target_gear = min(Gear.FIRST + 1, Gear.EIGHTH)
	elif speed_ratio < 0.4:
		target_gear = min(Gear.FIRST + 2, Gear.EIGHTH)
	elif speed_ratio < 0.55:
		target_gear = min(Gear.FIRST + 3, Gear.EIGHTH)
	elif speed_ratio < 0.7:
		target_gear = min(Gear.FIRST + 4, Gear.EIGHTH)
	elif speed_ratio < 0.85:
		target_gear = min(Gear.FIRST + 5, Gear.EIGHTH)
	elif speed_ratio < 0.95:
		target_gear = min(Gear.FIRST + 6, Gear.EIGHTH)
	else:
		target_gear = min(Gear.FIRST + 7, Gear.EIGHTH)
	
	if current_gear != target_gear:
		target_gear = target_gear.clamp(Gear.REVERSE, Gear.EIGHTH)
	
	_perform_gear_shift()

func _sequential_shift_logic() -> void:
	if Engine.input_manager.has_action("gear_up"):
		if current_gear < Gear.EIGHTH:
			target_gear = current_gear + 1
	if Engine.input_manager.has_action("gear_down"):
		if current_gear > Gear.REVERSE:
			target_gear = current_gear - 1
	_perform_gear_shift()

func _perform_gear_shift() -> void:
	if current_gear == target_gear or _clutch_input > 0.5:
		return
	
	var old_gear = current_gear
	current_gear = target_gear
	emit_signal("gear_changed", old_gear, current_gear)
	
	if Engine.audio_manager:
		Engine.audio_manager.play_sfx("gear_shift")

func _calculate_engine_rpm() -> void:
	var gear_ratio = 1.0
	if current_gear == Gear.NEUTRAL:
		gear_ratio = 0.0
	elif current_gear == Gear.REVERSE:
		gear_ratio = gear_ratios[0] * final_drive_ratio * 1.1
	else:
		var index = current_gear - 1
		if index < gear_ratios.size():
			gear_ratio = gear_ratios[index] * final_drive_ratio
	
	if gear_ratio > 0:
		engine_rpm = (abs(current_speed) * gear_ratio * 100.0) + idle_rpm
	else:
		engine_rpm = lerp(engine_rpm, idle_rpm, 0.1)
	
	engine_rpm = engine_rpm.clamp(idle_rpm, redline_rpm)
	emit_signal("rpm_changed", engine_rpm)

func _get_current_torque() -> float:
	var rpm_percent = (engine_rpm - idle_rpm) / (redline_rpm - idle_rpm)
	rpm_percent = clamp(rpm_percent, 0.0, 1.0)
	var torque_index = int(rpm_percent * 100)
	return torque_curve[torque_index] * 450.0  # Peak torque 450 Nm

# ============================================================================
# POWERTRAIN INTEGRATION
# ============================================================================

@onready var powertrain: Node = get_node_or_null("../Powertrain")

func _apply_powertrain_forces() -> void:
	if powertrain == null:
		return
	
	var torque = _get_current_torque()
	var drivetrain_efficiency = 0.85
	var effective_torque = torque * drivetrain_efficiency
	
	var wheel_radius = 0.32  # meters
	var force_at_wheels = (effective_torque * current_gear.gear_ratio()) / (wheel_radius * final_drive_ratio)
	
	powertrain.apply_force(force_at_wheels)

# ============================================================================
# WHEEL FORCE CALCULATIONS
# ============================================================================

@export_group("Wheel Configuration")
@export var track_width: float = 1.5
@export var wheelbase: float = 2.7
@export var wheel_radius: float = 0.32
@export var tire_friction_coefficient: float = 1.2

@export_group("Suspension")
@export var suspension_stiffness: float = 50000.0
@export var suspension_damping: float = 5000.0
@export var suspension_compression: float = 0.15
@export var suspension_rebound: float = 0.15

@export_group("Braking System")
@export var front_brake_bias: float = 0.6
@export var brake_force_constant: float = 15000.0
@export var abs_enabled: bool = true

func _calculate_wheel_forces() -> Vector3:
	var drive_force: float = 0.0
	var brake_force: float = 0.0
	var steer_angle: float = 0.0
	
	# Calculate drive force
	if current_gear != Gear.NEUTRAL:
		var torque = _get_current_torque()
		var gear_ratio = current_gear.get_ratio() * final_drive_ratio
		var wheel_torque = torque * gear_ratio * 0.85  # drivetrain efficiency
		drive_force = wheel_torque / wheel_radius
	
	# Apply throttle
	if _throttle_input > 0:
		drive_force *= _throttle_input * 2.0
	
	# Calculate brake force
	if _brake_input > 0:
		var total_brake_force = brake_force_constant * _brake_input * _brake_input
		var front_brake = total_brake_force * front_brake_bias
		var rear_brake = total_brake_force * (1.0 - front_brake_bias)
		
		if abs(drive_force) < front_brake:
			brake_force = -front_brake
		
		# ABS check
		if abs(current_speed) < 2.0 and abs(brake_force) > 5000:
			brake_force *= 0.5
	
	# Calculate steering angle
	steer_angle = _steering_input * 0.5  # Max 30 degrees
	
	return Vector3(drive_force, brake_force, steer_angle)

# ============================================================================
# DRIFT MECHANICS
# ============================================================================

var drift_state: bool = false
var drift_intensity: float = 0.0
var drift_timer: float = 0.0

func _check_drift_conditions() -> void:
	var is_turning = abs(_steering_input) > 0.3
	var is_sliding = abs(lateral_acceleration) > 0.5
	var has_power = _throttle_input > 0.2
	
	if _handbrake_input > 0.5 and is_turning and abs(current_speed) > 15.0:
		if not drift_state:
			drift_state = true
			emit_signal("drift_started")
		drift_intensity = lerp(drift_intensity, 1.0, 0.1)
	elif is_sliding and has_power and not _handbrake_input > 0.5:
		if not drift_state:
			drift_state = true
			emit_signal("drift_started")
		drift_intensity = lerp(drift_intensity, 0.8, 0.05)
	else:
		if drift_state:
			drift_state = false
			emit_signal("drift_ended")
		drift_intensity = lerp(drift_intensity, 0.0, 0.1)

func _apply_drift_effects() -> void:
	if not drift_state:
		return
	
	# Reduce traction during drift
	var reduced_traction = 0.3 + (0.7 * drift_intensity)
	tire_friction_coefficient = 1.2 * reduced_traction
	
	# Add lateral slide velocity
	var slide_factor = drift_intensity * 0.3
	velocity.x += slide_factor * sin(transform.basis.z.y)
	velocity.z += slide_factor * cos(transform.basis.z.y)

# ============================================================================
# COLLISION HANDLING
# ============================================================================

func _on_collision(body: Node, collision_point: Vector3, normal: Vector3) -> void:
	var impact_speed = velocity.length()
	var impact_force = impact_speed * 1000.0
	
	collision_info = {
		"body_id": body.get_instance_id(),
		"position": collision_point,
		"normal": normal,
		"impact_speed": impact_speed,
		"impact_force": impact_force
	}
	
	emit_signal("collision_detected", collision_info)
	
	# Screen shake effect
	if Engine.game_manager and Engine.game_manager.debug_mode:
		_apply_screen_shake(min(impact_force / 1000.0, 1.0))

func _apply_screen_shake(amount: float) -> void:
	transform.origin += Vector3(randf() - 0.5, randf() - 0.5, randf() - 0.5) * amount

# ============================================================================
# MAIN UPDATE LOOP
# ============================================================================

func _physics_process(delta: float) -> void:
	_update_inputs(delta)
	_update_gearbox()
	_calculate_engine_rpm()
	var wheel_forces = _calculate_wheel_forces()
	_check_drift_conditions()
	_apply_physics(delta, wheel_forces)
	_handle_collisions(delta)

func _update_inputs(delta: float) -> void:
	_last_throttle = _throttle_input
	_last_brake = _brake_input
	_last_steering = _steering_input

func _apply_physics(delta: float, wheel_forces: Vector3) -> void:
	var gravity = PhysicsSettings.gravity
	var mass = PhysicsSettings.default_vehicle_mass
	
	# Calculate acceleration
	var drive_force = wheel_forces.x
	var brake_force = wheel_forces.y
	var steer_angle = wheel_forces.z
	
	# Apply longitudinal forces
	var net_force = drive_force + brake_force
	longitudinal_acceleration = net_force / mass
	
	# Apply movement
	var forward_dir = transform.basis.z
	var right_dir = transform.basis.x
	
	# Steering effect
	var turn_speed = steer_angle * 2.5
	rotation_velocity = turn_speed * sin(current_speed * 0.1)
	
	# Apply velocity changes
	var current_velocity_length = velocity.length()
	if current_velocity_length > 0:
		var direction = velocity.normalized()
		velocity = velocity + forward_dir * longitudinal_acceleration * delta * mass
	
	# Friction and drag
	var air_resistance = current_speed * current_speed * 0.5
	var rolling_resistance = mass * 0.015
	velocity -= velocity.normalized() * (air_resistance + rolling_resistance) * delta
	
	# Clamp speeds
	if current_speed > max_speed:
		velocity = velocity.normalized() * max_speed
	elif current_speed < -reverse_speed:
		velocity = -velocity.normalized() * reverse_speed
	
	# Update position
	move_and_slide()
	
	# Calculate lateral acceleration
	lateral_acceleration = rotation_velocity * current_speed / gravity
	
	# Update signals
	emit_signal("speed_changed", current_speed)

func _handle_collisions(delta: float) -> void:
	for i in range(get_slide_collision_count()):
		var collision = get_slide_collision(i)
		var body = collision.get_collider()
		var point = collision.get_position()
		var normal = collision.get_normal()
		
		_on_collision(body, point, normal)

# ============================================================================
# UTILITY METHODS
# ============================================================================

func reset_vehicle() -> void:
	current_speed = 0.0
	current_gear = Gear.NEUTRAL
	target_gear = Gear.NEUTRAL
	engine_rpm = idle_rpm
	drift_state = false
	drift_intensity = 0.0
	velocity = Vector3.ZERO
	transform.origin = Vector3.ZERO
	transform.basis = Basis.IDENTITY

func get_gear_display_name() -> String:
	match current_gear:
		Gear.NEUTRAL: return "N"
		Gear.REVERSE: return "R"
		_: return str(current_gear)

func get_rpm_percentage() -> float:
	return ((engine_rpm - idle_rpm) / (redline_rpm - idle_rpm)) * 100.0

func apply_damage(damage_amount: float) -> void:
	if damage_amount > 100.0:
		engine_stalled.emit()
		reset_vehicle()

func save_vehicle_state() -> Dictionary:
	return {
		"speed": current_speed,
		"gear": current_gear,
		"rpm": engine_rpm,
		"position": transform.origin,
		"rotation": transform.basis.get_euler()
	}

func load_vehicle_state(state: Dictionary) -> void:
	current_speed = state.get("speed", 0.0)
	current_gear = state.get("gear", Gear.NEUTRAL)
	engine_rpm = state.get("rpm", idle_rpm)
	transform.origin = state.get("position", Vector3.ZERO)
	transform.basis = Basis.from_euler(state.get("rotation", Vector3.ZERO))

# ============================================================================
# ENUM EXTENSIONS
# ============================================================================

extension Gear:
	func get_ratio() -> float:
		if self == Gear.NEUTRAL:
			return 0.0
		elif self == Gear.REVERSE:
			return gear_ratios[0] * 1.1
		else:
			var index = self - 1
			if index < gear_ratios.size():
				return gear_ratios[index]
			return gear_ratios.back()
</File>