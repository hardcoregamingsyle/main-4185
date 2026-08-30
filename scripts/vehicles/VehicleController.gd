extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulation
## Handles throttle, brake, steering, wheel forces, gear shifting, and vehicle dynamics
## Copyright 2026 Thalamus Racing Simulator Project

# Signals for external communication
signal speed_changed(new_speed: float)
signal rpm_changed(new_rpm: float)
signal gear_changed(new_gear: int)
signal drift_started(drift_angle: float)
signal drift_ended()
signal collision_impact(impact_force: Vector3)
signal wheel_slip(wheel_index: int, slip_ratio: float)

# References to core systems
var physics_settings: PhysicsSettings = null
var powertrain: Node = null

# ============================================================================
# VEHICLE CONFIGURATION
# ============================================================================
@export_group("Vehicle Configuration")
@export var mass: float = 1500.0: set = _set_mass
@export var center_of_mass_offset: Vector3 = Vector3(0.0, 0.5, 0.0)
@export var wheelbase: float = 2.7
@export var track_width_front: float = 1.6
@export var track_width_rear: float = 1.65
@export var height: float = 1.4
@export var width: float = 1.9
@export var length: float = 4.5

@export_group("Aerodynamics")
@export var drag_coefficient: float = 0.29
@export var frontal_area: float = 2.2
@export var downforce_coefficient: float = 0.5
@export var wing_angle: float = 10.0 # degrees

@export_group("Suspension Geometry")
@export var suspension_travel: float = 0.15
@export var spring_rate_front: float = 45000.0
@export var spring_rate_rear: float = 48000.0
@export var damper_compression_front: float = 12000.0
@export var damper_rebound_front: float = 4000.0
@export var damper_compression_rear: float = 13000.0
@export var damper_rebound_rear: float = 4500.0
@export var roll_stiffness_front: float = 2500.0
@export var roll_stiffness_rear: float = 2800.0

# ============================================================================
# POWERTRAIN PARAMETERS
# ============================================================================
@export_group("Engine Specs")
@export var engine_displacement: float = 3.0 # liters
@export var max_rpm: float = 8000.0
@export var idle_rpm: float = 800.0
@export var torque_curve: Dictionary = {
	"min_torque": 200.0,
	"max_torque": 450.0,
	"torque_peak_rpm": 4500.0,
	"power_peak_rpm": 6500.0
}
@export var clutch_friction_coefficient: float = 0.35
@export var flywheel_inertia: float = 0.5

@export_group("Transmission")
@export var gear_count: int = 6
@export var final_drive_ratio: float = 3.73
@export var transmission_ratios: Array[float] = [3.8, 2.2, 1.5, 1.1, 0.9, 0.75]
@export var reverse_ratio: float = 3.5
@export var shift_up_rpm_threshold: float = 7500.0
@export var shift_down_rpm_threshold: float = 3000.0

@export_group("Clutch Settings")
@export var clutch_engagement_point: float = 0.2
@export var clutch_slip_factor: float = 0.15

# ============================================================================
# BRAKING SYSTEM
# ============================================================================
@export_group("Braking System")
@export var brake_caliper_piston_area: float = 0.002 # m² per piston
@export var brake_pad_friction: float = 0.4
@export var brake_disc_radius: float = 0.15
@export var brake_pressure_max: float = 100.0 # bar
@export var brake_bias_front: float = 0.6
@export var brake_distribution: float = 0.55 # front/rear bias during combined braking
@export var abs_enabled: bool = true
@export var brake_temperature_decay: float = 0.01 # per second
@export var max_brake_temp: float = 650.0 # Celsius

# ============================================================================
# TIRE PARAMETERS
# ============================================================================
@export_group("Tire Characteristics")
@export var tire_width: float = 0.255
@export var tire_radius: float = 0.315
@export var tire_aspect_ratio: float = 0.35
@export var cornering_stiffness: float = 120000.0
@export var lateral_friction: float = 1.2
@export var longitudinal_friction: float = 1.15
@export var tire_temperature_optimal: float = 80.0 # Celsius
@export var tire_compound_type: String = "Slick"

# ============================================================================
# ELECTRONICS & AI
# ============================================================================
@export_group("Electronics & AI")
@export var traction_control_enabled: bool = true
@export var traction_control_aggressiveness: float = 0.5
@export var stability_control_enabled: bool = true
@export var ai_skill_level: float = 0.5 # 0.0-1.0
@export var cheat_mode_enabled: bool = false

# ============================================================================
# INTERNAL STATE VARIABLES
# ============================================================================
var current_speed: float = 0.0 # m/s
var current_rpm: float = 0.0
var current_gear: int = 0 # 0 = neutral, 1-6 = forward, -1 = reverse
var target_gear: int = 0
var throttle_input: float = 0.0 # 0.0-1.0
var brake_input: float = 0.0 # 0.0-1.0
var steering_input: float = 0.0 # -1.0 to 1.0
var clutch_input: float = 0.0 # 0.0-1.0 (auto-managed unless manual mode)

var wheel_slip_ratios: Array[float] = [0.0, 0.0, 0.0, 0.0]
var wheel_vertical_forces: Array[float] = [0.0, 0.0, 0.0, 0.0]
var drift_angle: float = 0.0
var drift_intensity: float = 0.0
var brake_temperature: float = 20.0
var tire_temperatures: Array[float] = [20.0, 20.0, 20.0, 20.0]

var aerodynamic_downforce: float = 0.0
var aerodynamic_drag: float = 0.0
var total_downforce: float = 0.0
var grip_available: float = 0.0

var _initial_position: Vector3 = Vector3.ZERO
var _initial_rotation: Quaternion = Quaternion.IDENTITY
var _is_ready: bool = false
var _last_physics_time: float = 0.0
var _gear_shift_timer: float = 0.0
var _shift_complete: bool = true
var _drift_history: Array[float] = []

# Wheel positions (local space)
var _wheel_positions_local: Array[Vector3] = []
var _wheel_collision_shapes: Array[Shape3D] = []

# ============================================================================
# INITIALIZATION
# ============================================================================
func _ready() -> void:
	_init_reference_systems()
	_setup_wheel_positions()
	_apply_initial_transform()
	_connect_powertrain_signals()
	_is_ready = true
	_initialize_state()
	
	print("[VehicleController] Initialization complete")

func _init_reference_systems() -> void:
	# Get reference to PhysicsSettings singleton
	if Engine.has_singleton("PhysicsSettings"):
		physics_settings = Engine.get_singleton("PhysicsSettings")
	elif get_node_or_null("/root/PhysicsSettings"):
		physics_settings = get_node("/root/PhysicsSettings")
	else:
		physics_settings = preload("res://scripts/core/PhysicsSettings.gd").new()
	
	# Get powertrain node
	var powertrain_path = "Powertrain"
	if get_node(powertrain_path):
		powertrain = get_node(powertrain_path)
		_connect_powertrain_signals()

func _setup_wheel_positions() -> void:
	_wheel_positions_local.clear()
	_wheel_positions_local.resize(4)
	
	# Front Left, Front Right, Rear Left, Rear Right
	var half_track_front: float = track_width_front * 0.5
	var half_track_rear: float = track_width_rear * 0.5
	
	_wheel_positions_local[0] = Vector3(-half_track_front, -0.315, wheelbase * 0.5 + 0.3)
	_wheel_positions_local[1] = Vector3(half_track_front, -0.315, wheelbase * 0.5 + 0.3)
	_wheel_positions_local[2] = Vector3(-half_track_rear, -0.315, -wheelbase * 0.5 - 0.3)
	_wheel_positions_local[3] = Vector3(half_track_rear, -0.315, -wheelbase * 0.5 - 0.3)

func _apply_initial_transform() -> void:
	_initial_position = global_position
	_initial_rotation = global_rotation_quaternion

func _connect_powertrain_signals() -> void:
	if powertrain:
		powertrain.connect("rpm_updated", _on_powertrain_rpm_updated)
		powertrain.connect("throttle_requested", _on_powertrain_throttle_requested)

# ============================================================================
# MAIN PHYSICS LOOP
# ============================================================================
func _physics_process(delta: float) -> void:
	if not _is_ready:
		return
	
	_last_physics_time = Time.get_ticks_msec() / 1000.0
	
	# Update input states
	_update_inputs(delta)
	
	# Calculate vehicle dynamics
	_calculate_aerodynamics()
	_calculate_suspension()
	_calculate_wheels()
	_calculate_drift()
	
	# Apply physics forces
	_apply_vehicle_forces()
	
	# Handle movement
	_move_and_slide()
	
	# Update state and emit signals
	_update_vehicle_state(delta)
	
	# Process gear shifts
	_process_gear_shifting(delta)
	
	# Check for special conditions
	_check_drift_conditions()
	_check_collision_events()

# ============================================================================
# INPUT HANDLING
# ============================================================================
func _update_inputs(delta: float) -> void:
	# Get inputs from InputManager singleton
	var input_manager: Node = GameManager.get_node_or_null("/root/InputManager")
	if input_manager:
		throttle_input = input_manager.get_action_value("vehicle_throttle")
		brake_input = input_manager.get_action_value("vehicle_brake")
		steering_input = input_manager.get_action_value("vehicle_steering")
		clutch_input = input_manager.get_action_value("vehicle_clutch")
	else:
		# Fallback to keyboard inputs
		throttle_input = Input.get_action_strength("ui_accept") # mapped to throttle
		brake_input = Input.get_action_strength("ui_cancel") # mapped to brake
		steering_input = Input.get_axis("ui_left", "ui_right")
		clutch_input = Input.get_action_strength("z") # Z key for clutch
	
	# Clamp inputs
	throttle_input = clampf(throttle_input, 0.0, 1.0)
	brake_input = clampf(brake_input, 0.0, 1.0)
	steering_input = clampf(steering_input, -1.0, 1.0)
	clutch_input = clampf(clutch_input, 0.0, 1.0)
	
	# Auto-clutch logic (optional)
	if not cheat_mode_enabled:
		_auto_manage_clutch()

func _auto_manage_clutch() -> void:
	# Automatic clutch engagement based on RPM and gear
	if current_gear != 0 and current_rpm < idle_rpm:
		# Rev matching downshift
		target_gear = _calculate_target_gear(current_rpm)
		if target_gear != current_gear:
			clutch_input = 0.0
			await get_tree().create_timer(0.1).timeout
			clutch_input = 1.0
			current_gear = target_gear
			_emmit_signal("gear_changed", target_gear)
	else:
		clutch_input = 1.0

# ============================================================================
# AERODYNAMICS CALCULATIONS
# ============================================================================
func _calculate_aerodynamics() -> void:
	var speed_squared: float = current_speed * current_speed
	
	# Drag force: F_drag = 0.5 * rho * Cd * A * v²
	aerodynamic_drag = 0.5 * 1.225 * drag_coefficient * frontal_area * speed_squared
	
	# Downforce: F_downforce = 0.5 * rho * Cl * A * v²
	aerodynamic_downforce = 0.5 * 1.225 * downforce_coefficient * frontal_area * speed_squared
	
	# Total downforce includes weight contribution
	total_downforce = (mass * physics_settings.gravity) + aerodynamic_downforce

# ============================================================================
# SUSPENSION CALCULATIONS
# ============================================================================
func _calculate_suspension() -> void:
	var total_weight: float = mass * physics_settings.gravity
	var weight_distribution: Vector2 = _get_weight_distribution()
	
	# Distribute weight between axles
	var front_axle_load: float = weight_distribution.x * total_weight
	var rear_axle_load: float = weight_distribution.y * total_weight
	
	# Per-wheel loads (assuming symmetrical)
	wheel_vertical_forces[0] = front_axle_load * 0.5
	wheel_vertical_forces[1] = front_axle_load * 0.5
	wheel_vertical_forces[2] = rear_axle_load * 0.5
	wheel_vertical_forces[3] = rear_axle_load * 0.5
	
	# Adjust for acceleration/deceleration load transfer
	var acceleration_force: float = _calculate_longitudinal_acceleration()
	var load_transfer: float = (acceleration_force * height) / wheelbase
	var normal_load_change: float = load_transfer * 0.5
	
	# Apply load transfer
	wheel_vertical_forces[0] -= normal_load_change
	wheel_vertical_forces[1] -= normal_load_change
	wheel_vertical_forces[2] += normal_load_change
	wheel_vertical_forces[3] += normal_load_change
	
	# Ensure minimum load
	for i in range(4):
		wheel_vertical_forces[i] = maxf(wheel_vertical_forces[i], 500.0)

func _get_weight_distribution() -> Vector2:
	# Default 50/50 split, can be overridden by CG position
	var cg_x_position: float = center_of_mass_offset.z
	var distance_to_front: float = wheelbase * 0.5 - cg_x_position
	var distance_to_rear: float = wheelbase * 0.5 + cg_x_position
	
	var total_distance: float = distance_to_front + distance_to_rear
	
	var front_ratio: float = distance_to_rear / total_distance if total_distance > 0 else 0.5
	var rear_ratio: float = distance_to_front / total_distance if total_distance > 0 else 0.5
	
	return Vector2(front_ratio, rear_ratio)

# ============================================================================
# WHEEL DYNAMICS
# ============================================================================
func _calculate_wheels() -> void:
	var wheel_circumference: float = 2.0 * PI * tire_radius
	var drive_ratio: float = _get_current_drive_ratio()
	var effective_radius: float = tire_radius * 0.95 # slight deformation
	
	# Calculate wheel rotational speeds based on vehicle speed and gear
	var wheel_rpm_array: Array[float] = [0.0, 0.0, 0.0, 0.0]
	var drive_wheels: Array[int] = [2, 3] # Rear-wheel drive
	
	# Calculate individual wheel speeds
	for i in range(4):
		var is_drive_wheel: bool = i in drive_wheels
		var steering_wheel: bool = i == 0 or i == 1
		
		var lateral_velocity: float = _calculate_lateral_velocity(i)
		var longitudinal_velocity: float = _calculate_longitudinal_velocity(i)
		
		# Slip ratio calculation
		var wheel_linear_speed: float = sqrtf(longitudinal_velocity * longitudinal_velocity + lateral_velocity * lateral_velocity)
		var theoretical_wheel_speed: float = current_speed if not is_drive_wheel else _get_engine_wheel_speed()
		
		wheel_slip_ratios[i] = (wheel_linear_speed - theoretical_wheel_speed) / maxf(theoretical_wheel_speed, 0.1)
		
		# Clamp slip ratio
		wheel_slip_ratios[i] = clampf(wheel_slip_ratios[i], -1.0, 1.0)
		
		# Emit slip signal
		wheel_slip.emit(i, wheel_slip_ratios[i])

func _calculate_lateral_velocity(wheel_index: int) -> float:
	# Lateral velocity based on vehicle yaw rate and wheel position
	var yaw_rate: float = linear_velocity.length() / maxf(length(), 0.1)
	var lateral_offset: float = wheel_positions_local[wheel_index].x
	
	return lateral_offset * yaw_rate

func _calculate_longitudinal_velocity(wheel_index: int) -> float:
	var drive_ratio: float = _get_current_drive_ratio()
	var wheel_base_radius: float = tire_radius
	
	# Longitudinal velocity is primarily vehicle speed with some slip adjustment
	var base_velocity: float = current_speed
	
	# Adjust for drive wheels
	if wheel_index in [2, 3]: # Rear wheels
		var drive_slip: float = wheel_slip_ratios[wheel_index]
		base_velocity *= (1.0 + drive_slip * 0.1)
	
	return base_velocity

func _get_engine_wheel_speed() -> float:
	var wheel_circumference: float = 2.0 * PI * tire_radius
	var drive_ratio: float = _get_current_drive_ratio()
	var rpm_conversion: float = max_rpm / (drive_ratio * wheel_circumference * 60.0)
	
	return current_rpm / rpm_conversion

# ============================================================================
# DRIFT CALCULATIONS
# ============================================================================
func _calculate_drift() -> void:
	# Calculate drift angle based on lateral velocity vs longitudinal velocity
	var lateral_velocity: float = _calculate_lateral_velocity(0)
	var longitudinal_velocity: float = _calculate_longitudinal_velocity(0)
	
	var drift_calculation: float = atan2(lateral_velocity, longitudinal_velocity)
	
	# Smooth drift angle
	var new_drift: float = lerp(drift_angle, drift_calculation, 0.1)
	
	# Calculate drift intensity based on magnitude
	var drift_magnitude: float = absf(new_drift)
	
	# Update drift state
	if drift_magnitude > 0.3:
		if drift_intensity == 0.0:
			# Drift just started
			_drift_history.append(Time.get_ticks_msec())
			_emmit_signal("drift_started", new_drift)
		
	(drift_intensity = minf(drift_magnitude * 2.0, 1.0)
	漂移角 = new_drift
	else:
		if drift_intensity > 0.0:
			# Drift ended
			_emmit_signal("drift_ended")
			_drift_history.clear()
		
	(drift_intensity = 0.0
	漂移角 = 0.0

# ============================================================================
# FORCE APPLICATION
# ============================================================================
func _apply_vehicle_forces() -> void:
	# Calculate engine torque based on RPM and throttle
	var engine_torque: float = _calculate_engine_torque()
	
	# Apply torque to drive wheels
	var drive_wheels: Array[int] = [2, 3] # RWD
	for wheel_idx in drive_wheels:
		var wheel_torque: float = engine_torque * 0.5 # Split between two wheels
		var wheel_force: float = wheel_torque / tire_radius
		
		# Apply friction limit
		var max_force: float = wheel_vertical_forces[wheel_idx] * longitudinal_friction
		wheel_force = minf(wheel_force, max_force)
		
		# Apply force in local forward direction
		var force_direction: Vector3 = transform.basis.z.normalized()
		force_direction.y = 0.0
		force_direction = force_direction.normalized()
		
		var applied_force: Vector3 = force_direction * wheel_force
		
		# Add to body force
		apply_central_force(applied_force)
	
	# Apply braking forces
	_apply_braking_forces()
	
	# Apply aerodynamic forces
	_apply_aerodynamic_forces()
	
	# Apply gravity
	var gravity_force: Vector3 = Vector3(0.0, -mass * physics_settings.gravity, 0.0)
	apply_central_force(gravity_force)

func _calculate_engine_torque() -> float:
	# Torque curve interpolation
	var rpm_normalized: float = current_rpm / max_rpm
	
	# Simple torque curve model
	var torque: float = torque_curve.max_torque
	if current_rpm < torque_curve.torque_peak_rpm:
		torque = torque_curve.min_torque + (current_rpm / torque_curve.torque_peak_rpm) * (torque_curve.max_torque - torque_curve.min_torque)
	else:
		torque = torque_curve.max_torque - ((current_rpm - torque_curve.torque_peak_rpm) / (max_rpm - torque_curve.torque_peak_rpm)) * (torque_curve.max_torque * 0.3)
	
	# Apply throttle
	var applied_torque: float = torque * throttle_input
	
	# Apply clutch factor
	applied_torque *= clutch_input
	
	# Apply traction control reduction
	if traction_control_enabled and current_gear != 0:
		var slip_penalty: float = 0.0
		for i in range(4):
			if i in [2, 3]: # Drive wheels
				slip_penalty += absf(wheel_slip_ratios[i])
		
		slip_penalty /= 2.0
		slip_penalty *= traction_control_aggressiveness
		applied_torque *= (1.0 - slip_penalty)
	
	return applied_torque

func _apply_braking_forces() -> void:
	if brake_input <= 0.0:
		return
	
	var brake_force_per_wheel: float = brake_input * brake_pressure_max * brake_caliper_piston_area * brake_pad_friction * brake_disc_radius
	
	# Apply brake bias
	var front_brake_force: float = brake_force_per_wheel * brake_bias_front
	var rear_brake_force: float = brake_force_per_wheel * (1.0 - brake_bias_front)
	
	var braking_direction: Vector3 = -transform.basis.z.normalized()
	braking_direction.y = 0.0
	braking_direction = braking_direction.normalized()
	
	# Front wheels
	for i in [0, 1]:
		var force: Vector3 = braking_direction * front_brake_force
		apply_central_force(force)
	
	# Rear wheels
	for i in [2, 3]:
		var force: Vector3 = braking_direction * rear_brake_force
		apply_central_force(force)
	
	# Update brake temperature
	brake_temperature = minf(brake_temperature + brake_input * 5.0, max_brake_temp)

func _apply_aerodynamic_forces() -> void:
	# Drag opposes motion
	if current_speed > 0:
		var drag_force: Vector3 = -transform.basis.z.normalized() * aerodynamic_drag
		drag_force.y = 0.0
		drag_force = drag_force.normalized()
		apply_central_force(drag_force)
	
	# Downforce pushes down
	var downforce_vector: Vector3 = Vector3(0.0, -aerodynamic_downforce, 0.0)
	apply_central_force(downforce_vector)

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================
func _process_gear_shifting(delta: float) -> void:
	_gear_shift_timer += delta
	
	if not _shift_complete:
		return
	
	# Automatic upshift
	if current_gear < gear_count and current_rpm >= shift_up_rpm_threshold and throttle_input > 0.1:
		_execute_gear_shift(current_gear + 1)
		return
	
	# Automatic downshift
	if current_gear > 1 and current_rpm <= shift_down_rpm_threshold and throttle_input < 0.1:
		_execute_gear_shift(current_gear - 1)
		return
	
	# Manual shift override
	if Input.is_action_just_pressed("vehicle_upshift"):
		if current_gear < gear_count:
			_execute_gear_shift(current_gear + 1)
	elif Input.is_action_just_pressed("vehicle_downshift"):
		if current_gear > 1:
			_execute_gear_shift(current_gear - 1)

func _execute_gear_shift(new_gear: int) -> void:
	if new_gear == current_gress:
		return
	
	# Disengage clutch
	_shift_complete = false
	clutch_input = 0.0
	
	# Schedule gear change
	await get_tree().create_timer(0.15).timeout
	
	# Engage new gear
	current_gear = new_gear
	clutch_input = 1.0
	_shift_complete = true
	
	# Emit signal
	gear_changed.emit(new_gear)
	
	# Audio feedback
	AudioManager.play_sound("gear_shift")

func _calculate_target_gear(target_rpm: float) -> int:
	var best_gear: int = 1
	var best_diff: float = absf(target_rpm - idle_rpm)
	
	for i in range(1, gear_count + 1):
		var gear_rpm: float = _calculate_rpm_at_speed(current_speed, i)
		var diff: float = absf(gear_rpm - target_rpm)
		if diff < best_diff:
			best_diff = diff
			best_gear = i
	
	return best_gear

func _calculate_rpm_at_speed(speed: float, gear: int) -> float:
	var drive_ratio: float = _get_drive_ratio_for_gear(gear)
	var wheel_circumference: float = 2.0 * PI * tire_radius
	var rpm: float = speed * drive_ratio * wheel_circumference * 60.0 / (2.0 * PI * tire_radius)
	
	return rpm

func _get_current_drive_ratio() -> float:
	if current_gear == 0:
		return 1.0
	
	var gear_ratio: float = transmission_ratios[current_gear - 1] if current_gear <= gear_count else reverse_ratio
	
	return gear_ratio * final_drive_ratio

func _get_drive_ratio_for_gear(gear: int) -> float:
	if gear == 0:
		return 1.0
	
	var gear_ratio: float = transmission_ratios[gear - 1] if gear <= gear_count else reverse_ratio
	
	return gear_ratio * final_drive_ratio

# ============================================================================
# MOVEMENT & COLLISION
# ============================================================================
func _move_and_slide() -> void:
	move_and_slide()

func _check_collision_events() -> void:
	for i in range(get_slide_collision_count()):
		var collision: KinematicCollision3D = get_slide_collision(i)
		var impact: Vector3 = collision.get_normal() * collision.get_depth()
		
		collision_impact.emit(impact)
		
		# Sound effect
		AudioManager.play_sound("collision")

func _check_drift_conditions() -> void:
	# Additional drift condition checks could go here
	pass

# ============================================================================
# VEHICLE STATE MANAGEMENT
# ============================================================================
func _update_vehicle_state(delta: float) -> void:
	# Update speed
	current_speed = linear_velocity.length()
	speed_changed.emit(current_speed)
	
	# Update RPM
	current_rpm = _calculate_current_rpm()
	rpm_changed.emit(current_rpm)
	
	# Decay brake temperature
	brake_temperature -= brake_temperature_decay * delta
	brake_temperature = maxf(brake_temperature, 20.0)
	
	# Decay tire temperatures
	for i in range(tire_temperatures.size()):
		tire_temperatures[i] -= 0.1 * delta
		tire_temperatures[i] = maxf(tire_temperatures[i], 20.0)
		tire_temperatures[i] = minf(tire_temperatures[i], 120.0)

func _calculate_current_rpm() -> float:
	var drive_ratio: float = _get_current_drive_ratio()
	var wheel_circumference: float = 2.0 * PI * tire_radius
	
	var rpm: float = current_speed * drive_ratio * wheel_circumference * 60.0 / (2.0 * PI * tire_radius)
	
	# Apply clutch slip
	rpm *= clutch_input
	
	# Clamp to valid range
	rpm = clampf(rpm, 0.0, max_rpm * 1.2)
	
	return rpm

func _calculate_longitudinal_acceleration() -> float:
	var net_force: float = aerodynamic_drag - _calculate_engine_torque() / tire_radius
	var acceleration: float = net_force / mass
	
	return acceleration

# ============================================================================
# HELPER METHODS
# ============================================================================
func reset_vehicle() -> void:
	linear_velocity = Vector3.ZERO
	angular_velocity = Vector3.ZERO
	current_speed = 0.0
	current_rpm = idle_rpm
	current_gear = 0
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	clutch_input = 0.0
	brake_temperature = 20.0
	tire_temperatures.fill(20.0)
	global_position = _initial_position
	global_rotation = _initial_rotation
	set_collision_layer_value(1, true)
	set_collision_mask_value(1, true)

func _set_mass(value: float) -> void:
	mass = value
	# Recalculate initial gravity force
	_gravity_force = Vector3(0.0, -mass * physics_settings.gravity, 0.0)

func _emmit_signal(signal_name: String, *args) -> void:
	if has_signal(signal_name):
		emit_signal(signal_name, *args)

func get_wheel_contact_point(wheel_index: int) -> Vector3:
	var world_pos: Vector3 = global_transform * _wheel_positions_local[wheel_index]
	return world_pos

func get_vehicle_state() -> Dictionary:
	return {
		"speed": current_speed,
		"rpm": current_rpm,
		"gear": current_gear,
		"throttle": throttle_input,
		"brake": brake_input,
		"steering": steering_input,
		"drift_angle": drift_angle,
		"brake_temp": brake_temperature,
		"tire_temps": tire_temperatures.duplicate(),
		"downforce": aerodynamic_downforce,
		"drag": aerodynamic_drag
	}

func set_vehicle_state(state: Dictionary) -> void:
	current_speed = state.get("speed", 0.0)
	current_rpm = state.get("rpm", 0.0)
	current_gear = state.get("gear", 0)
	throttle_input = state.get("throttle", 0.0)
	brake_input = state.get("brake", 0.0)
	steering_input = state.get("steering", 0.0)
	brake_temperature = state.get("brake_temp", 20.0)
	tire_temperatures = state.get("tire_temps", [20.0, 20.0, 20.0, 20.0]).duplicate()

func _initialize_state() -> void:
	current_rpm = idle_rpm
	current_gear = 0
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	clutch_input = 0.0
	brake_temperature = 20.0
	tire_temperatures.fill(20.0)
	wheel_slip_ratios.fill(0.0)
	wheel_vertical_forces.fill(0.0)
	aerodynamic_downforce = 0.0
	aerodynamic_drag = 0.0
	total_downforce = mass * physics_settings.gravity
	grip_available = total_downforce * lateral_friction
	_drift_history.clear()

# ============================================================================
# DEBUG & VISUALIZATION
# ============================================================================
func draw_debug_vectors() -> void:
	if not GameManager.debug_mode:
		return
	
	# Draw wheel contact points
	for i in range(4):
		var contact_point: Vector3 = get_wheel_contact_point(i)
		Debug.draw_sphere(contact_point, 0.1, Color.GREEN)
	
	# Draw force vectors
	var force_dir: Vector3 = transform.basis.z.normalized()
	Debug.draw_arrow(global_position, global_position + force_dir * 5.0, Color.YELLOW, 2.0)

# ============================================================================
# SIGNAL WRAPPERS
# ============================================================================
func _on_powertrain_rpm_updated(rpm: float) -> void:
	current_rpm = rpm
	rpm_changed.emit(rpm)

func _on_powertrain_throttle_requested(amount: float) -> void:
	throttle_input = amount
	_update_vehicle_state(0.0)

# ============================================================================
# END OF FILE
</VehicleController>