extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulation
## Handles throttle, brake, steering, wheel forces, gear shifting, and vehicle dynamics
## All physics calculations use constants from PhysicsSettings resource
## Copyright 2026 Thalamus Racing Simulator Project

signal speed_changed(speed: float)
signal rpm_changed(rpm: float)
signal gear_changed(gear: int)
signal drift_started(drift_angle: float)
signal drift_ended()
signal collision_detected(impact_force: float)
signal lap_completed(lap_time: float)
signal race_event(event_type: String, data: Dictionary)
signal engine_sound_changed(engine_rpm: float, engine_load: float)
signal suspension_compressed(compression_ratio: float)

# ============================================================================
# CONSTANTS & CONFIGURATION
# ============================================================================

const MAX_GEAR_COUNT := 8
const NEUTRAL_GEAR := 0
const REVERSE_GEAR := -1
const MAX_RPM := 9000.0
const IDLE_RPM := 800.0
const REDLINE_RPM := 8500.0
const SHIFTPRINT_THRESHOLD := 0.1
const DRIFT_MIN_SPEED := 15.0
const DRIFT_MAX_ANGLE := 45.0
const TRACTION_CONTROL_ENABLED := true
const ABS_ENABLED := true

# ============================================================================
# EXPORTED CONFIGURATION PROPERTIES
# ============================================================================

@export_group("Vehicle Configuration")
@export var vehicle_mass: float = 1500.0: set = _set_vehicle_mass
@export var center_of_mass_offset: Vector3 = Vector3(0.0, 0.5, 0.0)
@export var drag_coefficient: float = 0.30: set = _set_drag_coefficient
@export var frontal_area: float = 2.2: set = _set_frontal_area
@export var rolling_resistance: float = 0.015: set = _set_rolling_resistance

@export_group("Wheel Configuration")
@export var track_width: float = 1.6
@export var wheelbase: float = 2.6
@export var wheel_radius: float = 0.33
@export var suspension_travel_max: float = 0.15
@export var suspension_damping: float = 15000.0
@export var suspension_stiffness: float = 35000.0

@export_group("Tire Grip Settings")
@export var max_lateral_grip: float = 1.2
@export var max_longitudinal_grip: float = 1.0
@export var grip_drop_off_rate: float = 0.3
@export var grip_recovery_rate: float = 0.5

@export_group("Drift Settings")
@export var drift_friction_multiplier: float = 0.6
@export var drift_recover_speed: float = 2.0
@export var drift_stability_factor: float = 0.8
@export var oversteer_threshold: float = 1.1
@export var understeer_threshold: float = 1.05

@export_group("Powertrain Settings")
@export var engine_torque_curve: Array[Vector2]
@export var final_drive_ratio: float = 3.5
@export var gear_ratios: Array[float] = [3.8, 2.4, 1.7, 1.3, 1.0, 0.85, 0.72, 0.62]
@export var clutch_engage_time: float = 0.15
@export var clutch_disengage_time: float = 0.1

@export_group("Steering Settings")
@export var max_steering_angle: float = 0.55
@export var steering_ratio: float = 14.0
@export var steering_sensitivity: float = 1.0
@export var ackermann_correction: bool = true

@export_group("Brake Settings")
@export var brake_pressure_multiplier: float = 1.0
@export var brake_bias_front: float = 0.6
@export var brake_bleed_rate: float = 0.98

@export_group("Transmission Settings")
@export var shift_up_shiftpoint: float = 8200.0
@export var shift_down_shiftpoint: float = 3500.0
@export var auto_shift_enabled: bool = false
@export var rev_matching_enabled: bool = true

# ============================================================================
# PRIVATE MEMBER VARIABLES
# ============================================================================

# Engine state
var current_rpm: float = IDLE_RPM
var target_rpm: float = IDLE_RPM
var current_gear: int = NEUTRAL_GEAR
var previous_gear: int = NEUTRAL_GEAR
var transmission_state: TransmissionState = TransmissionState.DISENGAGED
var clutch_position: float = 1.0
var torque_output: float = 0.0
var engine_braking_torque: float = 0.0

# Input state
var throttle_input: float = 0.0
var brake_input: float = 0.0
var steering_input: float = 0.0
var handbrake_input: float = 0.0
var nitro_input: float = 0.0

# Vehicle dynamics state
var current_speed: float = 0.0
var acceleration_vector: Vector3 = Vector3.ZERO
var angular_velocity: float = 0.0
var lateral_slip: float = 0.0
var longitudinal_slip: float = 0.0
var slip_angle: float = 0.0
var drift_angle: float = 0.0
var in_drift_mode: bool = false
var drift_timer: float = 0.0
var grip_level: float = 1.0

# Wheel-specific states
var front_left_wheel: VehicleWheel = null
var front_right_wheel: VehicleWheel = null
var rear_left_wheel: VehicleWheel = null
var rear_right_wheel: VehicleWheel = null

# Suspension states
var suspension_compression_front_left: float = 0.0
var suspension_compression_front_right: float = 0.0
var suspension_compression_rear_left: float = 0.0
var suspension_compression_rear_right: float = 0.0

# Aerodynamics
var air_density: float = 1.225
var downforce: float = 0.0
var lift: float = 0.0

# Race/track state
var distance_traveled: float = 0.0
var lap_start_time: float = 0.0
var last_checkpoint: float = 0.0
var current_lap: int = 0
var best_lap_time: float = 0.0

# Physics references
var physics_settings: PhysicsSettings = PhysicsSettings.new()
var powertrain_node: Node = null

enum TransmissionState {
	DISENGAGED,
	ENGAGING,
	ENGAGED,
	DISENGAGING
}

enum WheelType {
	FRONT_LEFT,
	FRONT_RIGHT,
	REAR_LEFT,
	REAR_RIGHT
}

struct VehicleWheel {
	var position: Vector3
	var local_rotation: Quaternion
	var contact_point: Vector3
	var normal: Vector3
	var force: float = 0.0
	var steering_angle: float = 0.0
	var suspension_length: float = 0.0
	var locked: bool = false
	var camber: float = 0.0
	var toe: float = 0.0
}

# ============================================================================
# INITIALIZATION
# ============================================================================

func _ready() -> void:
	_init_physics_settings()
	_init_wheels()
	_connect_signals()
	_set_initial_state()

func _init_physics_settings() -> void:
	if ResourceLoader.exists("res://scripts/core/PhysicsSettings.gd"):
		physics_settings = load("res://scripts/core/PhysicsSettings.gd").new()
	else:
		physics_settings = PhysicsSettings.new()

func _init_wheels() -> void:
	front_left_wheel = VehicleWheel.new()
	front_right_wheel = VehicleWheel.new()
	rear_left_wheel = VehicleWheel.new()
	rear_right_wheel = VehicleWheel.new()
	
	_calculate_wheel_positions()

func _connect_signals() -> void:
	if GameManager:
		GameManager.game_state_changed.connect(_on_game_state_changed)

func _set_initial_state() -> void:
	current_rpm = IDLE_RPM
	target_rpm = IDLE_RPM
	current_gear = NEUTRAL_GEAR
	transmission_state = TransmissionState.DISENGAGED
	clutch_position = 1.0
	torque_output = 0.0
	in_drift_mode = false
	distance_traveled = 0.0
	current_lap = 0
	grip_level = max_lateral_grip

# ============================================================================
# MAIN PHYSICS PROCESS
# ============================================================================

func _physics_process(delta: float) -> void:
	if not is_inside_tree():
		return
	
	_handle_inputs(delta)
	_update_transmission(delta)
	_update_engine(delta)
	_apply_forces(delta)
	_update_wheel_states(delta)
	_update_drift(delta)
	_update_aerodynamics(delta)
	_update_race_state(delta)
	_update_outputs(delta)

func _input(event: InputEvent) -> void:
	if event.is_action_pressed("vehicle_shift_up"):
		shift_gear(true)
	elif event.is_action_pressed("vehicle_shift_down"):
		shift_gear(false)
	elif event.is_action_pressed("vehicle_toggle_abs"):
		ABS_ENABLED = !ABS_ENABLED
	elif event.is_action_pressed("vehicle_toggle_tc"):
		TRACTION_CONTROL_ENABLED = !TRACTION_CONTROL_ENABLED

# ============================================================================
# INPUT HANDLING
# ============================================================================

func _handle_inputs(delta: float) -> void:
	throttle_input = Input.get_axis("vehicle_accelerate", "vehicle_decelerate")
	brake_input = Input.get_axis("vehicle_brake", "vehicle_reverse")
	steering_input = Input.get_axis("vehicle_turn_left", "vehicle_turn_right") * steering_sensitivity
	handbrake_input = Input.get_axis("vehicle_handbrake", "")
	nitro_input = Input.get_axis("vehicle_nitro", "")
	
	_clamp_inputs()
	_process_steering_input(delta)

func _clamp_inputs() -> void:
	throttle_input = clampf(throttle_input, -1.0, 1.0)
	brake_input = clampf(brake_input, 0.0, 1.0)
	steering_input = clampf(steering_input, -1.0, 1.0)
	handbrake_input = clampf(handbrake_input, 0.0, 1.0)
	nitro_input = clampf(nitro_input, 0.0, 1.0)

func _process_steering_input(delta: float) -> void:
	if transmission_state == TransmissionState.DISENGAGED or current_gear == NEUTRAL_GEAR:
		max_steering_angle = 0.3
	
	var target_angle = steering_input * max_steering_angle
	
	if ackermann_correction and current_gear != NEUTRAL_GEAR:
		var ackermann_factor = _calculate_ackermann_factor()
		target_angle *= ackermann_factor
	
	steering_input = target_angle / max_steering_angle

func _calculate_ackermann_factor() -> float:
	var turn_radius = wheelbase / tan(abs(steering_input)) if abs(steering_input) > 0.01 else 999.0
	var inner_wheel_turn = atan(wheelbase / (turn_radius + track_width / 2.0))
	var outer_wheel_turn = atan(wheelbase / (turn_radius - track_width / 2.0))
	var ideal_turn = (inner_wheel_turn + outer_wheel_turn) / 2.0
	var actual_turn = asin(sin(max_steering_angle))
	return min(1.0, ideal_turn / actual_turn) if actual_turn > 0.01 else 1.0

# ============================================================================
# TRANSMISSION & GEAR LOGIC
# ============================================================================

func _update_transmission(delta: float) -> void:
	if auto_shift_enabled and current_gear != NEUTRAL_GEAR:
		_auto_shift_gears()
	
	if transmission_state == TransmissionState.ENGAGING or transmission_state == TransmissionState.DISENGAGING:
		_update_clutch(delta)
		return
	
	if clutch_position < 0.95 and current_gear != NEUTRAL_GEAR:
		transmission_state = TransmissionState.ENGAGED
	else:
		transmission_state = TransmissionState.DISENGAGED

func _auto_shift_gears() -> void:
	if current_rpm >= shift_up_shiftpoint and current_gear < MAX_GEAR_COUNT:
		shift_gear(true)
	elif current_rpm <= shift_down_shiftpoint and current_gear > 1:
		shift_gear(false)

func shift_gear(up: bool) -> void:
	if transmission_state != TransmissionState.ENGAGED:
		return
	
	var target_gear = current_gear + (1 if up else -1)
	
	if target_gear < 1 or target_gear > MAX_GEAR_COUNT:
		return
	
	_disengage_clutch()
	await _wait_for_clutch_disengage()
	
	previous_gear = current_gear
	current_gear = target_gear
	gear_changed.emit(current_gear)
	
	engage_clutch()
	await _wait_for_clutch_engage()
	
	if rev_matching_enabled and current_gear > previous_gear:
		_match_rpm_for_downshift()

func _disengage_clutch() -> void:
	transmission_state = TransmissionState.DISENGAGING
	var disengage_duration = Time.get_ticks_msec() / 1000.0
	clutch_position -= delta / max(clutch_disengage_time, 0.01)
	clutch_position = maxf(clutch_position, 0.0)

func _engage_clutch() -> void:
	transmission_state = TransmissionState.ENGAGING
	var engage_duration = Time.get_ticks_msec() / 1000.0
	clutch_position += delta / max(clutch_engage_time, 0.01)
	clutch_position = minf(clutch_position, 1.0)

func _update_clutch(delta: float) -> void:
	match transmission_state:
		TransmissionState.ENGAGING:
			clutch_position += delta / max(clutch_engage_time, 0.01)
			clutch_position = minf(clutch_position, 1.0)
			if clutch_position >= 1.0:
				transmission_state = TransmissionState.ENGAGED
		TransmissionState.DISENGAGING:
			clutch_position -= delta / max(clutch_disengage_time, 0.01)
			clutch_position = maxf(clutch_position, 0.0)
			if clutch_position <= 0.0:
				transmission_state = TransmissionState.DISENGAGED

func _wait_for_clutch_disengage() -> void:
	while clutch_position > 0.01:
		await get_tree().create_timer(0.01).timeout

func _wait_for_clutch_engage() -> void:
	while clutch_position < 0.99:
		await get_tree().create_timer(0.01).timeout

func _match_rpm_for_downshift() -> void:
	var target_rpm = current_rpm * (gear_ratios[current_gear - 1] / gear_ratios[current_gear])
	target_rpm = min(target_rpm, REDLINE_RPM)
	target_rpm = max(target_rpm, IDLE_RPM)
	
	var rpm_target = current_rpm + (target_rpm - current_rpm) * 0.5
	current_rpm = lerp(current_rpm, rpm_target, delta * 10.0)

# ============================================================================
# ENGINE PHYSICS
# ============================================================================

func _update_engine(delta: float) -> void:
	var gear_ratio = _get_current_gear_ratio()
	var overall_ratio = gear_ratio * final_drive_ratio
	
	var effective_rpm = current_rpm * overall_ratio
	var engine_load = _calculate_engine_load()
	
	torque_output = _calculate_engine_torque(engine_load)
	engine_braking_torque = _calculate_engine_braking_torque()
	
	if clutch_position < 0.95 or current_gear == NEUTRAL_GEAR:
		target_rpm = IDLE_RPM + (throttle_input * 2000.0)
	else:
		var load_factor = 1.0 if throttle_input > 0.0 else 0.3
		target_rpm = IDLE_RPM + (throttle_input * (REDLINE_RPM - IDLE_RPM)) * load_factor
	
	current_rpm = lerp(current_rpm, target_rpm, delta * 15.0)
	current_rpm = clampf(current_rpm, IDLE_RPM, MAX_RPM)
	
	rpm_changed.emit(current_rpm)
	engine_sound_changed.emit(current_rpm, engine_load)

func _get_current_gear_ratio() -> float:
	if current_gear == NEUTRAL_GEAR:
		return 0.0
	elif current_gear == REVERSE_GEAR:
		return -gear_ratios[min(MAX_GEAR_COUNT - 1, abs(current_gear))]
	else:
		return gear_ratios[current_gear - 1]

func _calculate_engine_load() -> float:
	var speed_factor = current_speed / 100.0
	var rpm_factor = current_rpm / MAX_RPM
	var throttle_factor = throttle_input
	
	var load = (rpm_factor * 0.4 + speed_factor * 0.3 + (1.0 - throttle_factor) * 0.3)
	return clampf(load, 0.0, 1.0)

func _calculate_engine_torque(engine_load: float) -> float:
	if engine_torque_curve.is_empty():
		return 400.0 * engine_load
	
	var rpm_normalized = (current_rpm - IDLE_RPM) / (MAX_RPM - IDLE_RPM)
	rpm_normalized = clampf(rpm_normalized, 0.0, 1.0)
	
	var torque = _interpolate_torque_curve(rpm_normalized)
	torque *= engine_load * clutch_position
	
	if ABS_ENABLED and brake_input > 0.1:
		torque *= 0.5
	
	return torque

func _interpolate_torque_curve(rpm_normalized: float) -> float:
	if engine_torque_curve.size() < 2:
		return 400.0
	
	var segment_count = engine_torque_curve.size() - 1
	var segment_index = floor(rpm_normalized * segment_count)
	segment_index = clampi(segment_index, 0, segment_count - 1)
	
	var p1 = engine_torque_curve[segment_index]
	var p2 = engine_torque_curve[segment_index + 1]
	
	var t = (rpm_normalized * segment_count) - segment_index
	var torque = p1.y + (p2.y - p1.y) * t
	
	return torque

func _calculate_engine_braking_torque() -> float:
	if current_gear == NEUTRAL_GEAR or clutch_position < 0.95:
		return 0.0
	
	var base_braking = 50.0
	var rpm_factor = (current_rpm - IDLE_RPM) / (REDLINE_RPM - IDLE_RPM)
	var gear_factor = gear_ratios[current_gear - 1] if current_gear > 0 else 1.0
	
	return base_braking * rpm_factor * gear_factor * 2.0

# ============================================================================
# WHEEL FORCES & VEHICLE DYNAMICS
# ============================================================================

func _apply_forces(delta: float) -> void:
	var total_force = Vector3.ZERO
	var total_torque = 0.0
	
	_calculate_wheel_forces(delta)
	
	for wheel in [front_left_wheel, front_right_wheel, rear_left_wheel, rear_right_wheel]:
		total_force += wheel.force * wheel.normal
		total_torque += wheel.steering_angle * wheel.force
		
	velocity = _apply_vehicle_velocity(total_force, delta)
	move_and_slide()
	
	_handle_collision_detection()
	_handle_suspension_events()

func _calculate_wheel_forces(delta: float) -> void:
	var drive_torque = torque_output * clutch_position * final_drive_ratio
	
	_apply_drive_torque_to_wheels(drive_torque)
	_apply_brake_forces(delta)
	_apply_steering_angles()
	_calculate_wheel_contact_forces(delta)

func _apply_drive_torque_to_wheels(drive_torque: float) -> void:
	if current_gear == NEUTRAL_GEAR or current_gear == REVERSE_GEAR:
		drive_torque *= -1.0
	
	var drive_ratio = _get_current_gear_ratio()
	var wheel_torque = drive_torque * drive_ratio * 0.5
	
	if current_gear > 0:
		rear_left_wheel.force = wheel_torque / wheel_radius
		rear_right_wheel.force = wheel_torque / wheel_radius
	else:
		front_left_wheel.force = wheel_torque / wheel_radius
		front_right_wheel.force = wheel_torque / wheel_radius

func _apply_brake_forces(delta: float) -> void:
	var brake_force_per_wheel = brake_input * brake_pressure_multiplier * 5000.0
	
	if ABS_ENABLED and current_speed > 5.0:
		brake_force_per_wheel *= _calculate_abs_modulation()
	
	var front_brake_force = brake_force_per_wheel * brake_bias_front
	var rear_brake_force = brake_force_per_wheel * (1.0 - brake_bias_front)
	
	front_left_wheel.force -= front_brake_force
	front_right_wheel.force -= front_brake_force
	rear_left_wheel.force -= rear_brake_force
	rear_right_wheel.force -= rear_brake_force

func _calculate_abs_modulation() -> float:
	var slip_threshold = 0.2
	var max_modulation = 0.8
	
	if longitudinal_slip > slip_threshold:
		return max_modulation * (1.0 - (longitudinal_slip - slip_threshold))
	
	return 1.0

func _apply_steering_angles() -> void:
	var wheel_caster_angle = steering_input * max_steering_angle
	
	if ackermann_correction:
		var ackermann_factor = _calculate_ackermann_factor()
		
		front_left_wheel.steering_angle = wheel_caster_angle * ackermann_factor
		front_right_wheel.steering_angle = wheel_caster_angle * ackermann_factor * 0.85
	else:
		front_left_wheel.steering_angle = wheel_caster_angle
		front_right_wheel.steering_angle = -wheel_caster_angle
	
	rear_left_wheel.steering_angle = 0.0
	rear_right_wheel.steering_angle = 0.0

func _calculate_wheel_contact_forces(delta: float) -> void:
	var grip_modifier = _calculate_grip_modifier()
	
	for wheel in [front_left_wheel, front_right_wheel, rear_left_wheel, rear_right_wheel]:
		wheel.force *= grip_modifier
		wheel.contact_point = _calculate_contact_point(wheel)
		wheel.normal = Vector3.UP
		wheel.suspension_length = _calculate_suspension_length(wheel)

func _calculate_contact_point(wheel: VehicleWheel) -> Vector3:
	var offset = Vector3(0.0, -wheel_radius, 0.0)
	var pos = global_transform * offset
	pos.x += wheel.steering_angle * wheel_radius * sin(wheel.camber)
	pos.z += wheel.steering_angle * wheel_radius * cos(wheel.camber)
	return pos

func _calculate_suspension_length(wheel: VehicleWheel) -> float:
	var rest_length = suspension_travel_max * 0.5
	var compressed_length = rest_length - suspension_compression_front_left
	
	return clampf(compressed_length, 0.0, suspension_travel_max)

func _apply_vehicle_velocity(total_force: Vector3, delta: float) -> Vector3:
	var velocity_change = total_force / vehicle_mass * delta
	
	var new_velocity = velocity + velocity_change
	new_velocity.y = 0.0
	
	var drag_force = _calculate_drag_force(new_velocity.length())
	var rolling_resistance = _calculate_rolling_resistance()
	
	new_velocity -= drag_force.normalized() * delta
	new_velocity -= rolling_resistance.normalized() * delta
	
	return new_velocity

func _calculate_drag_force(speed: float) -> float:
	var dynamic_pressure = 0.5 * air_density * speed * speed
	return dynamic_pressure * drag_coefficient * frontal_area

func _calculate_rolling_resistance() -> float:
	return vehicle_mass * gravity * rolling_resistance

func _handle_collision_detection() -> void:
	if motion.length() > 5.0:
		var impact_force = motion.length() * vehicle_mass
		collision_detected.emit(impact_force)

func _handle_suspension_events() -> void:
	var avg_suspension = (suspension_compression_front_left + suspension_compression_front_right + 
		suspension_compression_rear_left + suspension_compression_rear_right) / 4.0
	
	if avg_suspension > suspension_travel_max * 0.8:
		suspension_compressed.emit(avg_suspension / suspension_travel_max)

# ============================================================================
# DRIFT MECHANICS
# ============================================================================

func _update_drift(delta: float) -> void:
	if current_speed < DRIFT_MIN_SPEED:
		end_drift()
		return
	
	var lateral_acceleration = _calculate_lateral_acceleration()
	var drift_threshold = oversteer_threshold * grip_level
	
	if abs(lateral_acceleration) > drift_threshold and abs(steering_input) > 0.3:
		start_drift()
	
	if in_drift_mode:
		_update_drift_state(delta)

func _calculate_lateral_acceleration() -> float:
	var forward_dir = global_transform.basis.z
	var lateral_vel = velocity.dot(global_transform.basis.x)
	return lateral_vel / delta if delta > 0.0 else 0.0

func start_drift() -> void:
	in_drift_mode = true
漂移_timer = 0.0
drift_angle = 0.0
grip_level *= drift_friction_multiplier
drift_started.emit(drift_angle)

func _update_drift_state(delta: float) -> void:
漂移_timer += delta
drift_angle += steering_input * delta * 10.0
drift_angle = clampf(drift_angle, -DRIFT_MAX_ANGLE, DRIFT_MAX_ANGLE)

if abs(drift_angle) < 5.0 and abs(steering_input) < 0.2:
	end_drift()
else:
	lateral_slip = drift_angle * PI / 180.0

func end_drift() -> void:
	if in_drift_mode:
		in_drift_mode = false
		grip_level = max_lateral_grip
		漂移_timer = 0.0
		drift_ended.emit()

# ============================================================================
# AERODYNAMICS
# ============================================================================

func _update_aerodynamics(delta: float) -> void:
	var speed = velocity.length()
	var dynamic_pressure = 0.5 * air_density * speed * speed
	
	var lift_coefficient = _calculate_lift_coefficient()
	var downforce_coefficient = _calculate_downforce_coefficient()
	
	lift = dynamic_pressure * frontal_area * lift_coefficient
	downforce = dynamic_pressure * frontal_area * downforce_coefficient

func _calculate_lift_coefficient() -> float:
	var speed_factor = current_speed / 200.0
	var angle_of_attack = slip_angle * PI / 180.0
	return -0.1 * speed_factor + 0.05 * sin(angle_of_attack)

func _calculate_downforce_coefficient() -> float:
	var speed_factor = current_speed / 200.0
	return 0.3 * speed_factor * speed_factor

# ============================================================================
# RACE STATE MANAGEMENT
# ============================================================================

func _update_race_state(delta: float) -> void:
	distance_traveled += velocity.length() * delta
	
	if distance_traveled > 1000.0:
		current_lap += 1
		if current_lap == 1:
			lap_start_time = Time.get_ticks_msec() / 1000.0

func _calculate_lap_time() -> float:
	if lap_start_time > 0.0:
		return Time.get_ticks_msec() / 1000.0 - lap_start_time
	return 0.0

func record_lap() -> void:
	var lap_time = _calculate_lap_time()
	if lap_time > 0.0:
		if best_lap_time == 0.0 or lap_time < best_lap_time:
			best_lap_time = lap_time
		lap_completed.emit(lap_time)

# ============================================================================
# OUTPUT UPDATES
# ============================================================================

func _update_outputs(delta: float) -> void:
	current_speed = velocity.length()
	speed_changed.emit(current_speed)
	
	var wheel_rpm = (current_speed / (2.0 * PI * wheel_radius)) * 60.0
	var wheel_slip = abs(wheel_rpm - current_rpm / _get_current_gear_ratio()) / current_rpm
	longitudinal_slip = wheel_slip if wheel_slip > 0.0 else 0.0
	
	var lateral_velocity = velocity.x
	var forward_velocity = velocity.z
	slip_angle = atan2(lateral_velocity, forward_velocity) if forward_velocity > 0.0 else 0.0

func _calculate_grip_modifier() -> float:
	var combined_slip = sqrt(longitudinal_slip * longitudinal_slip + lateral_slip * lateral_slip)
	var grip_drop = combined_slip * grip_drop_off_rate
	var recovered_grip = grip_level * (1.0 - grip_drop)
	
	return recovered_grip if recovered_grip > 0.0 else 0.0

# ============================================================================
# WHEEL POSITION CALCULATION
# ============================================================================

func _calculate_wheel_positions() -> void:
	var half_track = track_width / 2.0
	var half_wheelbase = wheelbase / 2.0
	
	var car_pos = global_position
	var offset_y = -center_of_mass_offset.y
	
	front_left_wheel.position = car_pos + Vector3(-half_track, offset_y, -half_wheelbase)
	front_right_wheel.position = car_pos + Vector3(half_track, offset_y, -half_wheelbase)
	rear_left_wheel.position = car_pos + Vector3(-half_track, offset_y, half_wheelbase)
	rear_right_wheel.position = car_pos + Vector3(half_track, offset_y, half_wheelbase)

# ============================================================================
# PUBLIC API METHODS
# ============================================================================

func reset_vehicle() -> void:
	current_rpm = IDLE_RPM
	current_gear = NEUTRAL_GEAR
	current_speed = 0.0
	in_drift_mode = false
	distance_traveled = 0.0
	current_lap = 0
	best_lap_time = 0.0
	
	velocity = Vector3.ZERO
	angular_velocity = 0.0
	sliding_input = 0.0
	
	reset_wheels()

func reset_wheels() -> void:
	for wheel in [front_left_wheel, front_right_wheel, rear_left_wheel, rear_right_wheel]:
		wheel.force = 0.0
		wheel.steering_angle = 0.0
		wheel.suspension_length = suspension_travel_max * 0.5

func set_powertrain(powertrain: Powertrain) -> void:
	powertrain_node = powertrain
	if powertrain:
		powertrain.engine_torque_curve = engine_torque_curve

func get_vehicle_status() -> Dictionary:
	return {
		"speed": current_speed,
		"rpm": current_rpm,
		"gear": current_gear,
		"in_drift": in_drift_mode,
		"distance": distance_traveled,
		"lap": current_lap,
		"best_lap": best_lap_time,
		"throttle": throttle_input,
		"brake": brake_input,
		"steering": steering_input,
		"handbrake": handbrake_input
	}

func _on_game_state_changed(new_state: GameState) -> void:
	match new_state:
		GameManager.GameState.RACE_ACTIVE:
			_activate_racing_mode()
		GameManager.GameState.RACE_PAUSED:
			_pause_racing()
		GameManager.GameState.MAIN_MENU:
			_reset_for_menu()

func _activate_racing_mode() -> void:
	current_rpm = IDLE_RPM
	velocity = Vector3.ZERO

func _pause_racing() -> void:
	pass

func _reset_for_menu() -> void:
	reset_vehicle()

# ============================================================================
# PROPERTY SETTERS
# ============================================================================

func _set_vehicle_mass(value: float) -> void:
	vehicle_mass = value
	if is_ready():
		mass = value

func _set_drag_coefficient(value: float) -> void:
	drag_coefficient = value

func _set_frontal_area(value: float) -> void:
	frontal_area = value

func _set_rolling_resistance(value: float) -> void:
	rolling_resistance = value
</FILE>