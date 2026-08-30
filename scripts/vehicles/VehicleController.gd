extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulation
## Handles throttle, brake, steering, wheel forces, gear shifting, and vehicle dynamics
## Uses PhysicsSettings constants for all physics values
## Copyright 2026 Thalamus Racing Simulator Project

# References
@onready var powertrain: Node = $Powertrain
@onready var chassis: RigidBody3D = $Chassis
@onready var wheel_front_left: CollisionShape3D = $Wheels/WheelFrontLeft
@onready var wheel_front_right: CollisionShape3D = $Wheels/WheelFrontRight
@onready var wheel_rear_left: CollisionShape3D = $Wheels/WheelRearLeft
@onready var wheel_rear_right: CollisionShape3D = $Wheels/WheelRearRight
@onready var suspension_system: Node = $SuspensionSystem
@onready var audio_player: AudioStreamPlayer3D = $AudioStreamPlayer3D

# Signals
signal speed_changed(speed: float)
signal rpm_changed(rpm: float)
signal gear_changed(old_gear: int, new_gear: int)
signal drift_started()
signal drift_ended()
signal engine_stalled()

# Physics references
var _physics_settings: PhysicsSettings = PhysicsSettings.get_singleton()

# Input state
var _throttle_input: float = 0.0
var _brake_input: float = 0.0
var _steering_input: float = 0.0
var _handbrake_input: bool = false

# Vehicle state
var current_speed: float = 0.0
var current_rpm: float = 0.0
var current_gear: int = 0
var max_rpm: float = 8000.0
var idle_rpm: float = 800.0
var redline_rpm: float = 7500.0

# Gear ratios (final drive included)
var _gear_ratios: Array[float] = [0.0, 3.8, 2.4, 1.7, 1.3, 1.0, 0.85]  # Reverse + gears 1-6
var _reverse_ratio: float = 3.9
var final_drive_ratio: float = 3.45
var differential_efficiency: float = 0.95

# Tire and friction properties
var tire_friction_coefficient: float = 1.2
var lateral_friction_coefficient: float = 1.0
var longitudinal_friction_coefficient: float = 1.3
var grip_loss_threshold: float = 0.75

# Drift mechanics
var drift_intensity: float = 0.0
var drift_timer: float = 0.0
var drift_max_intensity: float = 1.0
var drift_recovery_rate: float = 0.98

# Aerodynamics
var drag_coefficient: float = 0.35
var frontal_area: float = 2.2
var downforce_coefficient: float = 0.1
var downforce_multiplier: float = 1.0

# Vehicle configuration
@export_group("Vehicle Configuration")
@export var vehicle_mass: float = 1500.0
@export var center_of_mass_offset: Vector3 = Vector3(0.0, -0.3, 0.2)
@export var wheelbase: float = 2.6
@export var track_width: float = 1.6
@export var engine_power_hp: float = 450.0
@export var engine_torque_nm: float = 600.0
@export var clutch_engagement_point: float = 0.3

@export_group("Wheel Settings")
@export var wheel_radius: float = 0.33
@export var wheel_rotation_speed_limit: float = 150.0
@export var braking_force_per_wheel: float = 5000.0
@export var acceleration_force_per_wheel: float = 3000.0

@export_group("Steering Settings")
@export var max_steering_angle: float = 30.0
@export var steering_speed: float = 45.0
@export var steering_return_speed: float = 60.0
@export var steering_lock_position: float = 0.35

@export_group("Transmission Settings")
@export var transmission_type: TransmissionType = TransmissionType.MANUAL
@export var shift_up_delay: float = 0.3
@export var shift_down_delay: float = 0.15
@export var auto_shift_enabled: bool = true

enum TransmissionType {
	MANUAL,
	SEMI_AUTO,
	AUTOMATIC
}

# Internal state
var _shift_cooldown: float = 0.0
var _target_gear: int = 0
var _wheel_rotation_angles: Dictionary = {}
var _current_engine_force: float = 0.0
var _current_brake_force: float = 0.0
var _driving_direction: int = 1  # 1 forward, -1 reverse
var _is_clutch_engaged: bool = true
var _engine_on: bool = true
var _vehicle_in_air: bool = false

func _ready() -> void:
	_init_vehicle_state()
	_setup_physics_properties()
	_connect_signals_to_audio()
	_apply_initial_transform()

func _init_vehicle_state() -> void:
	current_speed = 0.0
	current_rpm = idle_rpm
	current_gear = 0
	_throttle_input = 0.0
	_brake_input = 0.0
	_steering_input = 0.0
	_handbrake_input = false
	_drift_intensity = 0.0
	_is_clutch_engaged = true
	_engine_on = true

func _setup_physics_properties() -> void:
	if chassis:
		chassis.mass = vehicle_mass
		chassis.center_of_mass = center_of_mass_offset
		
	var inertia = calculate_inertia_matrix()
	if chassis:
		chassis.set_axis_moments(inertia.x, inertia.y, inertia.z)

func calculate_inertia_matrix() -> Vector3:
	# Simplified inertia calculation based on vehicle dimensions and mass
	var length = wheelbase
	var width = track_width
	var height = 0.6
	
	var lx = 0.5 * vehicle_mass * (width * width + height * height)
	var ly = 0.5 * vehicle_mass * (length * length + height * height)
	var lz = 0.5 * vehicle_mass * (length * length + width * width)
	
	return Vector3(lx, ly, lz)

func _apply_initial_transform() -> void:
	var initial_pos = get_global_transform().origin
	set_global_position(initial_pos)
	global_velocity = Vector3.ZERO
	global_rotational_velocity = Vector3.ZERO

func _connect_signals_to_audio() -> void:
	rpm_changed.connect(_on_rpm_changed)
	speed_changed.connect(_on_speed_changed)

func _on_rpm_changed(new_rpm: float) -> void:
	pass

func _on_speed_changed(new_speed: float) -> void:
	pass

func _physics_process(delta: float) -> void:
	if not _engine_on:
		return
	
	_handle_inputs(delta)
	_update_transmission(delta)
	_calculate_wheel_forces(delta)
	_apply_vehicle_dynamics(delta)
	_update_aerodynamics(delta)
	_update_drift_state(delta)
	_check_ground_contact()
	_synchronize_wheels()

func _input(event: InputEvent) -> void:
	if event.is_action_pressed("ui_cancel"):
		get_tree().paused = !get_tree().paused

func _handle_inputs(delta: float) -> void:
	# Read input actions
	_throttle_input = Input.get_action_strength("vehicle_throttle")
	_brake_input = Input.get_action_strength("vehicle_brake")
	_handbrake_input = Input.is_action_pressed("vehicle_handbrake")
	
	# Steering input with deadzone
	var raw_steering = Input.get_axis("vehicle_steering_left", "vehicle_steering_right")
	_steering_input = deadzone(raw_steering, 0.1)
	
	# Clamp inputs
	_throttle_input = clamp(_throttle_input, 0.0, 1.0)
	_brake_input = clamp(_brake_input, 0.0, 1.0)
	_steering_input = clamp(_steering_input, -1.0, 1.0)
	
	# Auto-shift logic if enabled
	if auto_shift_enabled and _is_clutch_engaged:
		_auto_shift_logic()

func _update_transmission(delta: float) -> void:
	if _shift_cooldown > 0.0:
		_shift_cooldown -= delta
	
	# Manual gear shifting
	if Input.is_action_just_pressed("vehicle_shift_up"):
		_attempt_gear_shift(1)
	elif Input.is_action_just_pressed("vehicle_shift_down"):
		_attempt_gear_shift(-1)
	
	# Calculate target gear based on RPM
	if auto_shift_enabled:
		_target_gear = _calculate_optimal_gear()
		
	# Update current gear
	if _target_gear != current_gear:
		_execute_gear_change(_target_gear)
	
	# Engine RPM calculation
	_current_engine_force = _calculate_engine_output(delta)
	_current_rpm = _calculate_rpm_from_speed()

func _attempt_gear_shift(direction: int) -> void:
	if _shift_cooldown > 0.0 or not _is_clutch_engaged:
		return
	
	var new_gear = current_gear + direction
	new_gear = clamp(new_gear, 0, _gear_raties.size() - 1)
	
	if new_gear == current_gear:
		return
	
	_disengage_clutch()
	await get_tree().create_timer(shift_up_delay).timeout
	_is_clutch_engaged = true
	_execute_gear_change(new_gear)

func _execute_gear_change(new_gear: int) -> void:
	var old_gear = current_gear
	current_gear = new_gear
	gear_changed.emit(old_gear, new_gear)
	
	# Apply gear change effects
	_adjust_engine_behavior()

func _calculate_optimal_gear() -> int:
	var target_rpm = idle_rpm + (redline_rpm - idle_rpm) * 0.6
	
	if current_rpm < target_rpm and current_gear < _gear_ratios.size() - 1:
		return current_gear + 1
	elif current_rpm > target_rpm and current_gear > 0:
		return current_gear - 1
	
	return current_gear

func _calculate_engine_output(delta: float) -> float:
	var ratio = _get_current_gear_ratio()
	var wheel_rads_per_sec = current_speed / wheel_radius
	var engine_rads_per_sec = wheel_rads_per_sec * ratio * final_drive_ratio
	
	current_rpm = lerp(current_rpm, engine_rads_per_sec * 9.549, 0.1)
	
	if current_rpm >= max_rpm:
		_cut_engine_fuel()
		return 0.0
	
	return _calculate_torque_curve() * ratio * differential_efficiency

func _calculate_torque_curve() -> float:
	var normalized_rpm = current_rpm / max_rpm
	
	if normalized_rpm < 0.3:
		return engine_torque_nm * normalized_rpm / 0.3
	elif normalized_rpm < 0.8:
		return engine_torque_nm
	else:
		return engine_torque_nm * (1.0 - (normalized_rpm - 0.8) * 2.0)

func _cut_engine_fuel() -> void:
	# Simulate fuel cut at redline
	_current_engine_force *= 0.1

func _get_current_gear_ratio() -> float:
	if current_gear == 0:
		return _reverse_ratio
	return _gear_ratios[current_gear]

func _calculate_rpm_from_speed() -> float:
	var ratio = _get_current_gear_ratio()
	var wheel_rads_per_sec = current_speed / wheel_radius
	return wheel_rads_per_sec * ratio * final_drive_ratio * 9.549

func _calculate_wheel_forces(delta: float) -> void:
	if not chassis:
		return
	
	var total_force = _current_engine_force * _throttle_input
	var brake_force = _brake_input * braking_force_per_wheel
	
	# Distribute force to wheels
	var drive_force = _distribute_drive_force(total_force)
	var brake_forces = _distribute_brake_force(brake_force)
	
	# Apply forces to wheels
	_apply_wheel_forces(drive_force, brake_forces, delta)

func _distribute_drive_force(total_force: float) -> Dictionary:
	var distribution = {"front_left": 0.0, "front_right": 0.0, 
						"rear_left": 0.0, "rear_right": 0.0}
	
	# Simple rear-wheel drive distribution
	if current_gear > 0:
		distribution["rear_left"] = total_force * 0.5
		distribution["rear_right"] = total_force * 0.5
	else:
		distribution["front_left"] = total_force * 0.5
		distribution["front_right"] = total_force * 0.5
	
	return distribution

func _distribute_brake_force(total_brake: float) -> Dictionary:
	var distribution = {"front_left": 0.0, "front_right": 0.0,
						"rear_left": 0.0, "rear_right": 0.0}
	
	# Brake bias towards front (typical ~60%)
	distribution["front_left"] = total_brake * 0.3
	distribution["front_right"] = total_brake * 0.3
	distribution["rear_left"] = total_brake * 0.2
	distribution["rear_right"] = total_brake * 0.2
	
	return distribution

func _apply_wheel_forces(drive: Dictionary, brakes: Dictionary, delta: float) -> void:
	# Get wheel positions relative to chassis
	var wheel_positions = _get_wheel_local_positions()
	
	for wheel in ["front_left", "front_right", "rear_left", "rear_right"]:
		var position = wheel_positions[wheel]
		var force_vector = transform.basis.z * drive[wheel]
		
		if chassis:
			chassis.apply_central_impulse(force_vector * delta)
		
		# Apply individual wheel rotation
		_update_wheel_rotation(wheel, drive[wheel], delta)

func _get_wheel_local_positions() -> Dictionary:
	var half_track = track_width / 2.0
	var half_wheelbase = wheelbase / 2.0
	
	return {
		"front_left": Vector3(-half_track, -0.3, half_wheelbase),
		"front_right": Vector3(half_track, -0.3, half_wheelbase),
		"rear_left": Vector3(-half_track, -0.3, -half_wheelbase),
		"rear_right": Vector3(half_track, -0.3, -half_wheelbase)
	}

func _update_wheel_rotation(wheel_name: String, force: float, delta: float) -> void:
	var angle_change = force / wheel_radius * delta
	if abs(angle_change) > wheel_rotation_speed_limit * delta:
		angle_change = sign(angle_change) * wheel_rotation_speed_limit * delta
	
	_wheel_rotation_angles[wheel_name] = (_wheel_rotation_angles.get(wheel_name, 0.0) + angle_change) % (TWO_PI)

func _synchronize_wheels() -> void:
	for wheel_name in _wheel_rotation_angles:
		var wheel_node = _get_wheel_node(wheel_name)
		if wheel_node:
			wheel_node.rotation.z = _wheel_rotation_angles[wheel_name]

func _get_wheel_node(wheel_name: String) -> Node3D:
	match wheel_name:
		"front_left": return $Wheels/WheelFrontLeft
		"front_right": return $Wheels/WheelFrontRight
		"rear_left": return $Wheels/WheelRearLeft
		"rear_right": return $Wheels/WheelRearRight
	return null

func _apply_vehicle_dynamics(delta: float) -> void:
	if not chassis:
		return
	
	# Apply gravity
	var gravity_vector = _physics_settings.gravity * Vector3.DOWN
	chassis.apply_force(gravity_vector * vehicle_mass, Vector3.ZERO)
	
	# Apply aerodynamic drag
	var velocity = global_velocity
	var speed = velocity.length()
	
	if speed > 0.0:
		var drag_force = -velocity.normalized() * _calculate_drag_force(speed)
		chassis.apply_central_force(drag_force)
	
	# Handle air resistance when moving
	if _vehicle_in_air:
		var air_resistance = speed * speed * 0.01
		var resistance_force = -velocity.normalized() * air_resistance
		chassis.apply_central_force(resistance_force)
	
	# Ground friction when not in air
	if not _vehicle_in_air:
		var ground_friction = _calculate_ground_friction()
		var friction_force = -velocity.normalized() * ground_friction
		chassis.apply_central_force(friction_force)

func _calculate_drag_force(speed: float) -> float:
	var dynamic_pressure = 0.5 * 1.225 * speed * speed
	return dynamic_pressure * drag_coefficient * frontal_area

func _calculate_ground_friction() -> float:
	var normal_force = vehicle_mass * _physics_settings.gravity
	
	if _handbrake_input:
		return normal_force * lateral_friction_coefficient * 2.0
	
	return normal_force * tire_friction_coefficient * 0.05

func _update_aerodynamics(delta: float) -> void:
	if not chassis:
		return
	
	var speed = global_velocity.length()
	var downforce = speed * speed * downforce_coefficient * downforce_multiplier
	
	var downforce_vector = Vector3.DOWN * downforce
	chassis.apply_central_force(downforce_vector)

func _update_drift_state(delta: float) -> void:
	if not _vehicle_in_air:
		var lateral_velocity = global_velocity.dot(transform.basis.x.normalized())
		var steering_angle = _steering_input * max_steering_angle
		
		var drift_threshold = grip_loss_threshold * lateral_friction_coefficient
		
		if abs(lateral_velocity) > drift_threshold and abs(steering_angle) > 10.0:
			_drift_intensity = min(_drift_intensity + delta * 2.0, drift_max_intensity)
			
			if _drift_intensity > 0.5:
				if drift_intensity <= 0.5:
				漂移_started.emit()
		else:
			_drift_intensity = max(_drift_intensity - delta * drift_recovery_rate, 0.0)
			
			if _drift_intensity <= 0.0:
				if drift_intensity > 0.0:
				漂移_ended.emit()

func _check_ground_contact() -> void:
	var ray_length = 1.0
	var ray_start = global_position + Vector3.UP * ray_length
	var ray_end = ray_start - Vector3.UP * ray_length
	
	var space_state = world_environment.physics_direct_space_state
	var query = PhysicsRayQueryParameters3D.create(ray_start, ray_end)
	query.exclude = [self]
	
	var result = space_state.ray_query(query)
	
	_vehicle_in_air = not result.has_collisions()

func _auto_shift_logic() -> void:
	if not auto_shift_enabled:
		return
	
	var optimal_gear = _calculate_optimal_gear()
	
	if optimal_gear != current_gear and _shift_cooldown <= 0.0:
		_target_gear = optimal_gear
		_attempt_manual_shift()

func _attempt_manual_shift() -> void:
	if _shift_cooldown > 0.0:
		return
	
	_disengage_clutch()
	await get_tree().create_timer(shift_down_delay).timeout
	_is_clutch_engaged = true
	_execute_gear_change(_target_gear)

func _disengage_clutch() -> void:
	_is_clutch_engaged = false
	_shift_cooldown = shift_up_delay

func _adjust_engine_behavior() -> void:
	# Adjust engine sound pitch based on gear
	if audio_player:
		var pitch_variation = 1.0 + (current_gear - 1) * 0.1
		audio_player.pitch_scale = max(pitch_variation, 0.5)

func set_engine_state(on: bool) -> void:
	_engine_on = on
	if on:
		current_rpm = idle_rpm
	else:
		current_rpm = 0.0
		engine_stalled.emit()

func reset_vehicle() -> void:
	_init_vehicle_state()
	
	if chassis:
		chassis.linear_velocity = Vector3.ZERO
		chassis.angular_velocity = Vector3.ZERO
		chassis.apply_force(Vector3.ZERO, Vector3.ZERO)

func get_vehicle_stats() -> Dictionary:
	return {
		"speed": current_speed,
		"rpm": current_rpm,
		"gear": current_gear,
		"throttle": _throttle_input,
		"brake": _brake_input,
		"steering": _steering_input,
		"drift_intensity": _drift_intensity,
		"in_air": _vehicle_in_air
	}

func set_vehicle_position(position: Vector3, rotation: Quaternion) -> void:
	global_position = position
	global_rotation = rotation.to_euler()

func set_vehicle_velocity(velocity: Vector3) -> void:
	global_velocity = velocity

func _set_gravity(value: float) -> void:
	_physics_settings.gravity = value

func _set_physics_tick_rate(value: int) -> void:
	_physics_settings.physics_tick_rate = value

func _set_max_substeps(value: int) -> void:
	_physics_settings.max_substeps = value

func _set_time_scale(value: float) -> void:
	_physics_settings.time_scale = value

func _set_default_vehicle_mass(value: float) -> void:
	_physics_settings.default_vehicle_mass = value

func _set_default_wheel_config(value: Dictionary) -> void:
	_physics_settings.default_wheel_config = value

func _set_default_suspension_config(value: Dictionary) -> void:
	_physics_settings.default_suspension_config = value

func _set_default_engine_config(value: Dictionary) -> void:
	_physics_settings.default_engine_config = value

func _set_default_transmission_config(value: Dictionary) -> void:
	_physics_settings.default_transmission_config = value

func _set_default_aero_config(value: Dictionary) -> void:
	_physics_settings.default_aero_config = value

func _set_default_handling_config(value: Dictionary) -> void:
	_physics_settings.default_handling_config = value

func _set_default_tire_config(value: Dictionary) -> void:
	_physics_settings.default_tire_config = value

func _set_default_brake_config(value: Dictionary) -> void:
	_physics_settings.default_brake_config = value

func _set_default_diff_config(value: Dictionary) -> void:
	_physics_settings.default_diff_config = value

func _set_default_body_config(value: Dictionary) -> void:
	_physics_settings.default_body_config = value

func _set_default_chassis_config(value: Dictionary) -> void:
	_physics_settings.default_chassis_config = value

func _set_default_material_config(value: Dictionary) -> void:
	_physics_settings.default_material_config = value

func _set_default_collision_config(value: Dictionary) -> void:
	_physics_settings.default_collision_config = value

func _set_default_sound_config(value: Dictionary) -> void:
	_physics_settings.default_sound_config = value

func _set_default_ai_config(value: Dictionary) -> void:
	_physics_settings.default_ai_config = value

func _set_default_physics_config(value: Dictionary) -> void:
	_physics_settings.default_physics_config = value

func _set_default_game_config(value: Dictionary) -> void:
	_physics_settings.default_game_config = value

func _set_default_ui_config(value: Dictionary) -> void:
	_physics_settings.default_ui_config = value

func _set_default_network_config(value: Dictionary) -> void:
	_physics_settings.default_network_config = value

func _set_default_save_config(value: Dictionary) -> void:
	_physics_settings.default_save_config = value

func _set_default_debug_config(value: Dictionary) -> void:
	_physics_settings.default_debug_config = value

func _set_default_performance_config(value: Dictionary) -> void:
	_physics_settings.default_performance_config = value

func _set_default_quality_config(value: Dictionary) -> void:
	_physics_settings.default_quality_config = value

func _set_default_accessibility_config(value: Dictionary) -> void:
	_physics_settings.default_accessibility_config = value

func _set_default_input_config(value: Dictionary) -> void:
	_physics_settings.default_input_config = value

func _set_default_camera_config(value: Dictionary) -> void:
	_physics_settings.default_camera_config = value

func _set_default_lighting_config(value: Dictionary) -> void:
	_physics_settings.default_lighting_config = value

func _set_default_postprocess_config(value: Dictionary) -> void:
	_physics_settings.default_postprocess_config = value

func _set_default_particle_config(value: Dictionary) -> void:
	_physics_settings.default_particle_config = value

func _set_default_shader_config(value: Dictionary) -> void:
	_physics_settings.default_shader_config = value

func _set_default_texture_config(value: Dictionary) -> void:
	_physics_settings.default_texture_config = value

func _set_default_model_config(value: Dictionary) -> void:
	_physics_settings.default_model_config = value

func _set_default_animation_config(value: Dictionary) -> void:
	_physics_settings.default_animation_config = value

func _set_default_scene_config(value: Dictionary) -> void:
	_physics_settings.default_scene_config = value

func _set_default_level_config(value: Dictionary) -> void:
	_physics_settings.default_level_config = value

func _set_default_track_config(value: Dictionary) -> void:
	_physics_settings.default_track_config = value

func _set_default_race_config(value: Dictionary) -> void:
	_physics_settings.default_race_config = value

func _set_default_car_config(value: Dictionary) -> void:
	_physics_settings.default_car_config = value

func _set_default_drift_config(value: Dictionary) -> void:
	_physics_settings.default_drift_config = value

func _set_default_collision_config(value: Dictionary) -> void:
	_physics_settings.default_collision_config = value

func _set_default_vehicle_config(value: Dictionary) -> void:
	_physics_settings.default_vehicle_config = value

func _set_gravity(value: float) -> void:
	_physics_settings.gravity = value

func _set_physics_tick_rate(value: int) -> void:
	_physics_settings.physics_tick_rate = value

func _set_max_substeps(value: int) -> void:
	_physics_settings.max_substeps = value

func _set_time_scale(value: float) -> void:
	_physics_settings.time_scale = value

func _set_default_vehicle_mass(value: float) -> void:
	_physics_settings.default_vehicle_mass = value

func _set_default_wheel_config(value: Dictionary) -> void:
	_physics_settings.default_wheel_config = value

func _set_default_suspension_config(value: Dictionary) -> void:
	_physics_settings.default_suspension_config = value

func _set_default_engine_config(value: Dictionary) -> void:
	_physics_settings.default_engine_config = value

func _set_default_transmission_config(value: Dictionary) -> void:
	_physics_settings.default_transmission_config = value

func _set_default_aero_config(value: Dictionary) -> void:
	_physics_settings.default_aero_config = value

func _set_default_handling_config(value: Dictionary) -> void:
	_physics_settings.default_handling_config = value

func _set_default_tire_config(value: Dictionary) -> void:
	_physics_settings.default_tire_config = value

func _set_default_brake_config(value: Dictionary) -> void:
	_physics_settings.default_brake_config = value

func _set_default_diff_config(value: Dictionary) -> void:
	_physics_settings.default_diff_config = value

func _set_default_body_config(value: Dictionary) -> void:
	_physics_settings.default_body_config = value

func _set_default_chassis_config(value: Dictionary) -> void:
	_physics_settings.default_chassis_config = value

func _set_default_material_config(value: Dictionary) -> void:
	_physics_settings.default_material_config = value

func _set_default_collision_config(value: Dictionary) -> void:
	_physics_settings.default_collision_config = value

func _set_default_sound_config(value: Dictionary) -> void:
	_physics_settings.default_sound_config = value

func _set_default_ai_config(value: Dictionary) -> void:
	_physics_settings.default_ai_config = value

func _set_default_physics_config(value: Dictionary) -> void:
	_physics_settings.default_physics_config = value

func _set_default_game_config(value: Dictionary) -> void:
	_physics_settings.default_game_config = value

func _set_default_ui_config(value: Dictionary) -> void:
	_physics_settings.default_ui_config = value

func _set_default_network_config(value: Dictionary) -> void:
	_physics_settings.default_network_config = value

func _set_default_save_config(value: Dictionary) -> void:
	_physics_settings.default_save_config = value

func _set_default_debug_config(value: Dictionary) -> void:
	_physics_settings.default_debug_config = value

func _set_default_performance_config(value: Dictionary) -> void:
	_physics_settings.default_performance_config = value

func _set_default_quality_config(value: Dictionary) -> void:
	_physics_settings.default_quality_config = value

func _set_default_accessibility_config(value: Dictionary) -> void:
	_physics_settings.default_accessibility_config = value

func _set_default_input_config(value: Dictionary) -> void:
	_physics_settings.default_input_config = value

func _set_default_camera_config(value: Dictionary) -> void:
	_physics_settings.default_camera_config = value

func _set_default_lighting_config(value: Dictionary) -> void:
	_physics_settings.default_lighting_config = value

func _set_default_postprocess_config(value: Dictionary) -> void:
	_physics_settings.default_postprocess_config = value

func _set_default_particle_config(value: Dictionary) -> void:
	_physics_settings.default_particle_config = value

func _set_default_shader_config(value: Dictionary) -> void:
	_physics_settings.default_shader_config = value

func _set_default_texture_config(value: Dictionary) -> void:
	_physics_settings.default_texture_config = value

func _set_default_model_config(value: Dictionary) -> void:
	_physics_settings.default_model_config = value

func _set_default_animation_config(value: Dictionary) -> void:
	_physics_settings.default_animation_config = value

func _set_default_scene_config(value: Dictionary) -> void:
	_physics_settings.default_scene_config = value

func _set_default_level_config(value: Dictionary) -> void:
	_physics_settings.default_level_config = value

func _set_default_track_config(value: Dictionary) -> void:
	_physics_settings.default_track_config = value

func _set_default_race_config(value: Dictionary) -> void:
	_physics_settings.default_race_config = value

func _set_default_car_config(value: Dictionary) -> void:
	_physics_settings.default_car_config = value

func _set_default_drift_config(value: Dictionary) -> void:
	_physics_settings.default_drift_config = value

func _set_default_collision_config(value: Dictionary) -> void:
	_physics_settings.default_collision_config = value

func _set_default_vehicle_config(value: Dictionary) -> void:
	_physics_settings.default_vehicle_config = value

func deadzone(value: float, threshold: float) -> float:
	if abs(value) < threshold:
		return 0.0
	return (value - sign(value) * threshold) / (1.0 - threshold)