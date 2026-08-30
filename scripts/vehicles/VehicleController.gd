extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulation
## Handles throttle, brake, steering, wheel forces, gear shifting, and vehicle dynamics
## Copyright 2026 Thalamus Racing Simulator Project

# ============================================================================
# SIGNALS
# ============================================================================
signal speed_changed(new_speed: float)
signal gear_changed(old_gear: int, new_gear: int)
signal traction_control_triggered(lost_traction: bool)
signal engine_stalled()
signal clutch_engaged(is_engaged: bool)
signal lap_completed(lap_time: float)

# ============================================================================
# EXPORTED CONFIGURATION GROUPS
# ============================================================================

@export_group("Vehicle Configuration")
@export var mass: float = 1500.0: set = _set_mass
@export var center_of_mass_offset: Vector3 = Vector3(0.0, 0.3, 0.0): set = _set_center_of_mass_offset
@export var wheel_track_width: float = 1.6: set = _set_wheel_track_width
@export var wheel_base_length: float = 2.8: set = _set_wheel_base_length
@export var wheel_radius: float = 0.35: set = _set_wheel_radius
@export var wheel_suspension_travel: float = 0.15: set = _set_wheel_suspension_travel
@export var suspension_stiffness: float = 45000.0: set = _set_suspension_stiffness
@export var suspension_damping: float = 5000.0: set = _set_suspension_damping

@export_group("Engine & Powertrain")
@export var engine_max_rpm: float = 7500.0: set = _set_engine_max_rpm
@export var engine_idle_rpm: float = 800.0: set = _set_engine_idle_rpm
@export var engine_peak_torque_rpm: float = 4200.0: set = _set_engine_peak_torque_rpm
@export var engine_peak_torque: float = 450.0: set = _set_engine_peak_torque
@export var engine_power_kw: float = 220.0: set = _set_engine_power_kw
@export var transmission_type: String = "manual": set = _set_transmission_type
@export var final_drive_ratio: float = 3.45: set = _set_final_drive_ratio

@export_group("Transmission Gears")
@export var gear_ratios: Array[float] = [3.8, 2.4, 1.7, 1.3, 1.0, 0.85]: set = _set_gear_ratios
@export var neutral_gear: int = -1: set = _set_neutral_gear
@export var reverse_gear: int = 0: set = _set_reverse_gear

@export_group("Braking System")
@export var front_brake_bias: float = 0.6: set = _set_front_brake_bias
@export var max_brake_force: float = 45000.0: set = _set_max_brake_force
@export var abs_enabled: bool = true: set = _set_abs_enabled
@export var brake_pressure_per_pedal: float = 150.0: set = _set_brake_pressure_per_pedal

@export_group("Aerodynamics")
@export var drag_coefficient: float = 0.32: set = _set_drag_coefficient
@export var frontal_area: float = 2.2: set = _set_frontal_area
@export var downforce_coefficient: float = 0.8: set = _set_downforce_coefficient
@export var wing_angle: float = 5.0: set = _set_wing_angle

@export_group("Tire & Friction")
@export var tire_friction_stiffness: float = 25000.0: set = _set_tire_friction_stiffness
@export var tire_friction_asymmetry: float = 1.2: set = _set_tire_friction_asymmetry
@export var grip_level: float = 1.0: set = _set_grip_level
@export var slip_threshold: float = 0.15: set = _set_slip_threshold

@export_group("AI Settings")
@export var ai_target_speed_percentage: float = 0.95: set = _set_ai_target_speed_percentage
@export var ai_steering_smoothness: float = 5.0: set = _set_ai_steering_smoothness
@export var ai_optimal_line_offset: float = 0.0: set = _set_ai_optimal_line_offset

# ============================================================================
# INTERNAL STATE VARIABLES
# ============================================================================

var _engine_rpm: float = 800.0
var _current_gear: int = 1
var _throttle_input: float = 0.0
var _brake_input: float = 0.0
var _steering_input: float = 0.0
var _clutch_input: float = 0.0
var _is_manual_shift: bool = false
var _shifted_this_frame: bool = false
var _inertia_factor: float = 1.0

# Tire state tracking
var _tire_state: Dictionary = {
	"front_left": {"slip_ratio": 0.0, "slip_angle": 0.0, "vertical_load": 0.0},
	"front_right": {"slip_ratio": 0.0, "slip_angle": 0.0, "vertical_load": 0.0},
	"rear_left": {"slip_ratio": 0.0, "slip_angle": 0.0, "vertical_load": 0.0},
	"rear_right": {"slip_ratio": 0.0, "slip_angle": 0.0, "vertical_load": 0.0}
}

# Suspension state
var _wheel_positions: Dictionary = {}
var _wheel_velocities: Dictionary = {}

# Aerodynamic state
var _downforce_z: float = 0.0
var _drag_force_x: float = 0.0

# Vehicle kinematics
var _vehicle_speed: float = 0.0
var _max_vehicle_speed: float = 0.0
var _acceleration: float = 0.0
var _deceleration: float = 0.0

# Traction control state
var _traction_control_active: bool = false
var _drive_wheel_slip: float = 0.0

# AI navigation
var _ai_current_waypoint_index: int = 0
var _ai_target_position: Vector3 = Vector3.ZERO
var _ai_steering_target: float = 0.0

# Lap timing
var _lap_start_time: float = 0.0
var _lap_completed_count: int = 0
var _last_checkpoint_time: float = 0.0

# Reference to powertrain component
var _powertrain_node: Node = null

# Time accumulator for substepping
var _time_accumulator: float = 0.0
var _fixed_time_step: float = 1.0 / 120.0

# ============================================================================
# SETTERS FOR EXPORTED PROPERTIES
# ============================================================================

func _set_mass(value: float) -> void:
	mass = value
	if mass > 0:
		_inertia_factor = 1.0 / mass

func _set_center_of_mass_offset(value: Vector3) -> void:
	center_of_mass_offset = value

func _set_wheel_track_width(value: float) -> void:
	wheel_track_width = clampf(value, 0.5, 3.0)

func _set_wheel_base_length(value: float) -> void:
	wheel_base_length = clampf(value, 1.5, 5.0)

func _set_wheel_radius(value: float) -> void:
	wheel_radius = clampf(value, 0.2, 0.5)

func _set_suspension_stiffness(value: float) -> void:
	suspension_stiffness = clampf(value, 10000.0, 100000.0)

func _set_suspension_damping(value: float) -> void:
	suspension_damping = clampf(value, 1000.0, 20000.0)

func _set_engine_max_rpm(value: float) -> void:
	engine_max_rpm = clampf(value, 5000.0, 12000.0)

func _set_engine_idle_rpm(value: float) -> void:
	engine_idle_rpm = clampf(value, 500.0, 1500.0)

func _set_engine_peak_torque_rpm(value: float) -> void:
	engine_peak_torque_rpm = clampf(value, 3000.0, 6000.0)

func _set_engine_peak_torque(value: float) -> void:
	engine_peak_torque = clampf(value, 100.0, 1000.0)

func _set_engine_power_kw(value: float) -> void:
	engine_power_kw = clampf(value, 50.0, 500.0)

func _set_transmission_type(value: String) -> void:
	transmission_type = value

func _set_final_drive_ratio(value: float) -> void:
	final_drive_ratio = clampf(value, 2.0, 5.0)

func _set_gear_ratios(value: Array[float]) -> void:
	gear_ratios = value if value.size() >= 5 else [3.8, 2.4, 1.7, 1.3, 1.0, 0.85]

func _set_neutral_gear(value: int) -> void:
	neutral_gear = value

func _set_reverse_gear(value: int) -> void:
	reverse_gear = value

func _set_front_brake_bias(value: float) -> void:
	front_brake_bias = clampf(value, 0.4, 0.7)

func _set_max_brake_force(value: float) -> void:
	max_brake_force = clampf(value, 10000.0, 100000.0)

func _set_abs_enabled(value: bool) -> void:
	abs_enabled = value

func _set_brake_pressure_per_pedal(value: float) -> void:
	brake_pressure_per_pedal = value

func _set_drag_coefficient(value: float) -> void:
	drag_coefficient = clampf(value, 0.2, 0.6)

func _set_frontal_area(value: float) -> -> void:
	frontal_area = clampf(value, 1.5, 3.0)

func _set_downforce_coefficient(value: float) -> void:
	downforce_coefficient = clampf(value, 0.1, 2.0)

func _set_wing_angle(value: float) -> void:
	wing_angle = clampf(value, 0.0, 15.0)

func _set_tire_friction_stiffness(value: float) -> void:
	tire_friction_stiffness = clampf(value, 5000.0, 50000.0)

func _set_tire_friction_asymmetry(value: float) -> void:
	tire_friction_asymmetry = clampf(value, 1.0, 2.0)

func _set_grip_level(value: float) -> void:
	grip_level = clampf(value, 0.5, 1.5)

func _set_slip_threshold(value: float) -> void:
	slip_threshold = clampf(value, 0.05, 0.30)

func _set_ai_target_speed_percentage(value: float) -> void:
	ai_target_speed_percentage = clampf(value, 0.7, 1.0)

func _set_ai_steering_smoothness(value: float) -> void:
	ai_steering_smoothness = clampf(value, 1.0, 10.0)

func _set_ai_optimal_line_offset(value: float) -> void:
	ai_optimal_line_offset = value

# ============================================================================
# INITIALIZATION
# ============================================================================

func _ready() -> void:
	_init_components()
	_calculate_vehicle_parameters()
	_setup_wheel_positions()
	_reset_lap_data()
	
	# Connect to input manager
	InputManager.input_throttle.connect(_on_throttle_input_changed)
	InputManager.input_brake.connect(_on_brake_input_changed)
	InputManager.input_steering.connect(_on_steering_input_changed)
	InputManager.input_clutch.connect(_on_clutch_input_changed)
	InputManager.input_up_shift.connect(_handle_up_shift)
	InputManager.input_down_shift.connect(_handle_down_shift)
	InputManager.input_tcs_toggle.connect(_toggle_traction_control)
	InputManager.input_auto_shift.connect(_toggle_auto_shift)
	
	print_debug("VehicleController initialized with %d gears" % gear_ratios.size())

func _init_components() -> void:
	# Find powertrain node if attached
	var powertrain_path = get_parent().get_path_to(get_parent()) + "/Powertrain"
	_powertrain_node = get_tree().get_first_node_in_group("powertrain")
	if _powertrain_node == null:
		var parent = get_parent()
		for child in parent.get_children():
			if child.has_method("get_rpm"):
				_powertrain_node = child
				break
	
	if _powertrain_node != null:
		print_debug("Powertrain component found")
	else:
		push_warning("Powertrain component not found - using default torque curve")

func _calculate_vehicle_parameters() -> void:
	# Calculate aerodynamic parameters
	_drag_force_coefficient = 0.5 * PhysicsSettings.drag_air_density * drag_coefficient * frontal_area
	_downforce_coefficient_val = 0.5 * PhysicsSettings.drag_air_density * downforce_coefficient * frontal_area
	
	# Calculate maximum theoretical speed based on top gear
	if gear_ratios.size() > 0:
		var top_gear_ratio = gear_ratios.min()
		_max_vehicle_speed = (engine_max_rpm / (top_gear_ratio * final_drive_ratio)) * 2.0 * PI * wheel_radius
		
	# Set up wheel position offsets
	_wheel_half_track = wheel_track_width / 2.0
	_wheel_half_base = wheel_base_length / 2.0

# ============================================================================
# INPUT HANDLING
# ============================================================================

func _physics_process(delta: float) -> void:
	# Fixed timestep accumulator for consistent physics
	_time_accumulator += delta
	while _time_accumulator >= _fixed_time_step:
		_update_physics(_fixed_time_step)
		_time_accumulator -= _fixed_time_step
	
	# Handle continuous input updates
	_process_continuous_inputs()
	_update_aerodynamics()
	_apply_forces_and_constraints()
	_update_vehicle_kinematics()
	_check_traction_control()
	_handle_ai_navigation()
	
	# Emit signals
	emit_signal("speed_changed", _vehicle_speed)
	emit_signal("gear_changed", 0, _current_gear)

func _process_continuous_inputs() -> void:
	# Smooth input transitions
	_throttle_input = lerp(_throttle_input, InputManager.throttle_value, 0.1)
	_brake_input = lerp(_brake_input, InputManager.brake_value, 0.1)
	_steering_input = lerp(_steering_input, InputManager.steering_value, 0.1)
	_clutch_input = lerp(_clutch_input, InputManager.clutch_value, 0.1)

# ============================================================================
# PHYSICS UPDATE (FIXED TIMESTEP)
# ============================================================================

func _update_physics(dt: float) -> void:
	# Update engine RPM based on gear ratio and vehicle speed
	_update_engine_rpm()
	
	# Apply driving forces
	_apply_driving_forces(dt)
	
	# Calculate tire forces
	_calculate_tire_forces(dt)
	
	# Apply braking forces
	_apply_braking_forces(dt)
	
	# Apply aerodynamic forces
	_apply_aerodynamic_forces()
	
	# Update suspension
	_update_suspension(dt)
	
	# Check collision constraints
	_handle_collision_constraints()

func _update_engine_rpm() -> void:
	var target_rpm: float = 0.0
	var wheel_rpm: float = 0.0
	
	if _vehicle_speed > 0.1:
		wheel_rpm = _vehicle_speed / (2.0 * PI * wheel_radius)
		
		if _current_gear != neutral_gear and _current_gear != -1:
			var gear_ratio = gear_ratios[_current_gear]
			target_rpm = wheel_rpm * gear_ratio * final_drive_ratio
		else:
			target_rpm = engine_idle_rpm
	
	# Engine friction and idle behavior
	var engine_friction: float = 500.0
	if _clutch_input > 0.8 and _current_gear != neutral_gear:
		# Clutch disengaged - engine spins freely
		_engine_rpm = lerp(_engine_rpm, engine_idle_rpm, dt * engine_friction)
	elif _clutch_input < 0.2:
		# Clutch engaged - engine follows wheels
		_engine_rpm = lerp(_engine_rpm, target_rpm, dt * 15.0)
	else:
		# Partial engagement - blend between idle and wheel RPM
		var engagement_factor = 1.0 - _clutch_input
		var blended_rpm = (target_rpm * engagement_factor) + (engine_idle_rpm * (1.0 - engagement_factor))
		_engine_rpm = lerp(_engine_rpm, blended_rpm, dt * 10.0)
	
	# Clamp RPM
	_engine_rpm = clampf(_engine_rpm, engine_idle_rpm, engine_max_rpm)
	
	# Call powertrain update if available
	if _powertrain_node != null and _powertrain_node.has_method("update_rpm"):
		_powertrain_node.update_rpm(_engine_rpm, _current_gear)

func _apply_driving_forces(dt: float) -> void:
	var drive_wheel_force: float = 0.0
	
	# Get engine torque at current RPM
	var engine_torque = _calculate_engine_torque()
	
	# Apply torque through transmission
	if _current_gear != neutral_gear and _clutch_input < 0.8:
		var gear_ratio = gear_ratios[_current_gear]
		var total_ratio = gear_ratio * final_drive_ratio
		
		# Drive wheel force calculation
		drive_wheel_force = (engine_torque * total_ratio * 0.85) / wheel_radius
		
		# Apply throttle modulation
		if _throttle_input > 0.1:
			drive_wheel_force *= _throttle_input
		else:
			drive_wheel_force *= 0.1 # Idle torque
	
	# Distribute force to rear wheels (RWD drivetrain)
	var force_per_wheel = drive_wheel_force / 2.0
	
	# Apply forward velocity change
	if _current_gear != reverse_gear:
		velocity.x += drive_wheel_force * dt * 0.001 * _grip_level
	else:
		velocity.x -= drive_wheel_force * dt * 0.001 * _grip_level
	
	# Store drive wheel slip for traction control
	var wheel_slip_speed = (_engine_rpm / 60.0) * 2.0 * PI * wheel_radius / total_ratio
	_drive_wheel_slip = abs(wheel_slip_speed - _vehicle_speed) / max(wheel_slip_speed, 0.1)

func _calculate_engine_torque() -> float:
	var rpm_normalized = (_engine_rpm - engine_idle_rpm) / (engine_max_rpm - engine_idle_rpm)
	rpm_normalized = clampf(rpm_normalized, 0.0, 1.0)
	
	# Torque curve approximation (bell-shaped around peak torque RPM)
	var peak_ratio = (engine_peak_torque_rpm - engine_idle_rpm) / (engine_max_rpm - engine_idle_rpm)
	var torque_curve = exp(-pow((rpm_normalized - peak_ratio) / 0.3, 2)) * engine_peak_torque
	
	# Add some low-RPM torque boost
	if rpm_normalized < 0.3:
		torque_curve *= 1.1
	
	return torque_curve

func _calculate_tire_forces(dt: float) -> void:
	# Simplified tire model - calculate slip and generate lateral/longitudinal forces
	var grip_strength = tire_friction_stiffness * grip_level
	
	for wheel_key in _tire_state.keys():
		var state = _tire_state[wheel_key]
		
		# Calculate slip ratio from wheel rotation vs vehicle speed
		var wheel_vertical_speed = _get_wheel_vertical_velocity(wheel_key)
		state.vertical_load = maxf(1.0, wheel_vertical_speed) # Minimum load normalization
		
		# Longitudinal slip
		var wheel_rotation_speed = _engine_rpm / 60.0 * 2.0 * PI * wheel_radius / wheel_base_length
		state.slip_ratio = (wheel_rotation_speed - _vehicle_speed) / max(wheel_rotation_speed, 0.1)
		
		# Lateral slip angle (based on steering and body slide)
		state.slip_angle = _steering_input * 0.5
		
		# Generate tire force vector
		var longitudinal_force = -state.slip_ratio * grip_strength * state.vertical_load
		var lateral_force = -state.slip_angle * grip_strength * state.vertical_load * tire_friction_asymmetry
		
		# Limit forces to friction circle
		var total_force = sqrt(pow(longitudinal_force, 2) + pow(lateral_force, 2))
		var max_friction_force = grip_strength * state.vertical_load * 1.2
		
		if total_force > max_friction_force:
			var scale = max_friction_force / total_force
			longitudinal_force *= scale
			lateral_force *= scale
		
		# Apply tire forces to vehicle velocity
		velocity.x += longitudinal_force * dt * 0.0001
		velocity.y += lateral_force * dt * 0.0001

func _apply_braking_forces(dt: float) -> void:
	if _brake_input < 0.1:
		return
	
	var brake_force_per_wheel = _brake_input * max_brake_force / 4.0
	
	# Front/rear bias distribution
	var front_force = brake_force_per_wheel * front_brake_bias * 2.0
	var rear_force = brake_force_per_wheel * (1.0 - front_brake_bias) * 2.0
	
	# ABS check (simplified)
	if abs_enabled and _drive_wheel_slip > slip_threshold:
		brake_force_per_wheel *= 0.5 # Reduce braking when slipping
	
	# Apply braking force opposite to motion
	var direction_sign = sign(_vehicle_speed)
	if direction_sign == 0:
		direction_sign = 1
	
	velocity.x -= (front_force + rear_force) * direction_sign * dt * 0.001

func _apply_aerodynamic_forces() -> void:
	if _vehicle_speed <= 0.1:
		_drag_force_x = 0.0
		_downforce_z = 0.0
		return
	
	var speed_squared = _vehicle_speed * _vehicle_speed
	
	# Drag force opposes motion
	_drag_force_x = -_drag_force_coefficient * speed_squared * sign(_vehicle_speed)
	
	# Downforce increases with speed squared
	_downforce_z = -_downforce_coefficient_val * speed_squared
	
	# Apply aerodynamic forces
	velocity.x += _drag_force_x * 0.0001
	
	# Downforce affects normal force on tires (increases grip)
	for wheel_key in _tire_state.keys():
		_tire_state[wheel_key].vertical_load += abs(_downforce_z) * 0.01

func _update_suspension(dt: float) -> void:
	# Simplified suspension simulation
	# In full implementation, this would use raycasts to ground and spring-damper equations
	
	pass

func _handle_collision_constraints() -> void:
	# Prevent vehicle from going below ground plane
	if position.y < PhysicsSettings.granular_ground_height:
		position.y = PhysicsSettings.granular_ground_height
		velocity.y = 0.0
	
	# Ground friction
	if position.y <= PhysicsSettings.granular_ground_height + 0.01:
		var ground_friction = PhysicsSettings.default_ground_friction * grip_level
		velocity.x *= (1.0 - ground_friction * dt)

# ============================================================================
# AERODYNAMICS
# ============================================================================

func _update_aerodynamics() -> void:
	# Recalculate aerodynamic coefficients based on speed and wing angle
	pass

func _get_aero_downforce() -> float:
	return _downforce_z

func _get_aero_drag() -> float:
	return _drag_force_x

# ============================================================================
# VEHICLE KINEMATICS
# ============================================================================

func _update_vehicle_kinematics() -> void:
	# Update speed magnitude
	_vehicle_speed = velocity.length()
	
	# Update acceleration
	_acceleration = (_vehicle_speed - _last_vehicle_speed) / _physics_process_delta
	_last_vehicle_speed = _vehicle_speed
	
	# Update deceleration
	if _vehicle_speed < _last_vehicle_speed:
		_deceleration = (_last_vehicle_speed - _vehicle_speed) / _physics_process_delta
	else:
		_deceleration = 0.0
	
	# Cap vehicle speed
	_vehicle_speed = minf(_vehicle_speed, _max_vehicle_speed)
	
	# Apply speed changes to actual velocity vector
	var direction = velocity.normalized() if _vehicle_speed > 0.1 else Vector3.ZERO
	velocity = direction * _vehicle_speed
	
	# Store previous speed for acceleration calculation
	_last_vehicle_speed = _vehicle_speed

# ============================================================================
# TRACTION CONTROL
# ============================================================================

func _check_traction_control() -> void:
	if not InputManager.tcs_enabled:
		_traction_control_active = false
		return
	
	# Trigger traction control if excessive wheel slip detected
	if _drive_wheel_slip > slip_threshold:
		_traction_control_active = true
		
		# Reduce engine torque to reduce slip
		var slip_reduction = (_drive_wheel_slip - slip_threshold) / slip_threshold
		_throttle_input *= (1.0 - slip_reduction * 0.5)
		
		emit_signal("traction_control_triggered", true)
	else:
		_traction_control_active = false
		emit_signal("traction_control_triggered", false)

func _toggle_traction_control() -> void:
	InputManager.tcs_enabled = !InputManager.tcs_enabled

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================

func _handle_up_shift() -> void:
	if _shifted_this_frame or _clutch_input > 0.5:
		return
	
	if _current_gear < gear_ratios.size() - 1:
		_current_gear += 1
		emit_signal("gear_changed", _current_gear - 1, _current_gear)
		_shifted_this_frame = true
		_play_shift_sound(true)

func _handle_down_shift() -> void:
	if _shifted_this_frame or _clutch_input > 0.5:
		return
	
	if _current_gear > 0:
		_current_gear -= 1
		emit_signal("gear_changed", _current_gear + 1, _current_gear)
		_shifted_this_frame = true
		_play_shift_sound(false)

func _auto_shift_logic() -> void:
	if not InputManager.auto_shift_enabled:
		return
	
	# Auto shift based on RPM thresholds
	var shift_up_rpm = engine_max_rpm * 0.9
	var shift_down_rpm = engine_idle_rpm * 1.5
	
	if _engine_rpm > shift_up_rpm and _current_gear < gear_ratios.size() - 1:
		_handle_up_shift()
	elif _engine_rpm < shift_down_rpm and _current_gear > 0 and _throttle_input > 0.1:
		_handle_down_shift()
	
	_shifted_this_frame = false

func _toggle_auto_shift() -> void:
	InputManager.auto_shift_enabled = !InputManager.auto_shift_enabled

func _set_gear(gear_index: int) -> void:
	if gear_index >= 0 and gear_index < gear_ratios.size():
		var old_gear = _current_gear
		_current_gear = gear_index
		emit_signal("gear_changed", old_gear, _current_gear)
		_shifted_this_frame = true

func get_current_gear() -> int:
	return _current_gear

func get_engine_rpm() -> float:
	return _engine_rpm

func get_vehicle_speed() -> float:
	return _vehicle_speed

func get_throttle_input() -> float:
	return _throttle_input

func get_brake_input() -> float:
	return _brake_input

func get_steering_input() -> float:
	return _steering_input

# ============================================================================
# INPUT EVENT HANDLERS
# ============================================================================

func _on_throttle_input_changed(value: float) -> void:
	_throttle_input = clampf(value, 0.0, 1.0)

func _on_brake_input_changed(value: float) -> void:
	_brake_input = clampf(value, 0.0, 1.0)

func _on_steering_input_changed(value: float) -> void:
	_steering_input = clampf(value, -1.0, 1.0)

func _on_clutch_input_changed(value: float) -> void:
	_clutch_input = clampf(value, 0.0, 1.0)

# ============================================================================
# AI NAVIGATION
# ============================================================================

func _handle_ai_navigation() -> void:
	if not InputManager.ai_enabled:
		return
	
	# Simple waypoint following
	var distance_to_target = global_position.distance_to(_ai_target_position)
	
	if distance_to_target < 5.0:
		# Move to next waypoint
		_ai_current_waypoint_index = (_ai_current_waypoint_index + 1) % 10
		_ai_target_position = _get_next_waypoint()
	
	# Calculate desired steering
	var direction_to_target = (_ai_target_position - global_position).normalized()
	var desired_direction = direction_to_target.project(Vector3.RIGHT)
	_ai_steering_target = desired_direction.x
	
	# Smooth steering
	_ai_steering_target = lerp(_steering_input, _ai_steering_target, dt * ai_steering_smoothness)
	steering_input = _ai_steering_target

func _get_next_waypoint() -> Vector3:
	# Placeholder - in real implementation, read from track waypoints array
	return global_position + Vector3.FORWARD * 20.0

# ============================================================================
# LAP SYSTEM
# ============================================================================

func start_lap() -> void:
	_lap_start_time = Time.get_ticks_msec() / 1000.0
	_lap_completed_count = 0
	_last_checkpoint_time = _lap_start_time

func finish_lap() -> float:
	var lap_time = Time.get_ticks_msec() / 1000.0 - _lap_start_time
	_lap_completed_count += 1
	
	emit_signal("lap_completed", lap_time)
	
	return lap_time

func reset_lap_data() -> void:
	_lap_start_time = 0.0
	_lap_completed_count = 0
	_last_checkpoint_time = 0.0

func get_lap_count() -> int:
	return _lap_completed_count

func get_best_lap_time() -> float:
	# Placeholder - store best times in an array
	return 0.0

# ============================================================================
# WHEEL POSITION MANAGEMENT
# ============================================================================

func _setup_wheel_positions() -> void:
	_wheel_positions = {
		"front_left": Vector3(-_wheel_half_track, 0.0, -_wheel_half_base),
		"front_right": Vector3(_wheel_half_track, 0.0, -_wheel_half_base),
		"rear_left": Vector3(-_wheel_half_track, 0.0, _wheel_half_base),
		"rear_right": Vector3(_wheel_half_track, 0.0, _wheel_half_base)
	}

func get_wheel_position(wheel_name: String) -> Vector3:
	return _wheel_positions.get(wheel_name, Vector3.ZERO)

func get_wheel_velocity(wheel_name: String) -> Vector3:
	return _wheel_velocities.get(wheel_name, Vector3.ZERO)

func _get_wheel_vertical_velocity(wheel_name: String) -> float:
	# Simplified - returns vertical component of velocity at wheel location
	var wheel_pos = get_wheel_position(wheel_name)
	var world_wheel_pos = global_position + transform.basis * wheel_pos
	var wheel_vel = velocity + angular_velocity.cross(wheel_pos)
	return wheel_vel.y

# ============================================================================
# SOUND & FEEDBACK
# ============================================================================

func _play_shift_sound(upward: bool) -> void:
	if AudioManager:
		var sound_name = "gear_shift_" + ("up" if upward else "down")
		AudioManager.play_sound(sound_name)

func play_engine_sound() -> void:
	if AudioManager and _engine_rpm > engine_idle_rpm:
		var pitch = _engine_rpm / engine_max_rpm
		AudioManager.set_pitch_modulation("engine", pitch)

func stop_engine_sound() -> void:
	if AudioManager:
		AudioManager.stop_sound("engine")

# ============================================================================
# DEBUG & VISUALIZATION
# ============================================================================

func debug_print_vehicle_state() -> void:
	if GameManager.debug_mode:
		print_debug("=== Vehicle State ===")
		print_debug("Speed: %.2f km/h" % (_vehicle_speed * 3.6))
		print_debug("Gear: %d" % _current_gear)
		print_debug("RPM: %.0f" % _engine_rpm)
		print_debug("Throttle: %.2f" % _throttle_input)
		print_debug("Brake: %.2f" % _brake_input)
		print_debug("Steering: %.2f" % _steering_input)
		print_debug("Acceleration: %.2f m/s²" % _acceleration)
		print_debug("====================")

func debug_enable_visualization() -> void:
	# Create debug visualizations for wheels, forces, etc.
	pass

# ============================================================================
# UTILITY METHODS
# ============================================================================

func reset_vehicle() -> void:
	velocity = Vector3.ZERO
	angular_velocity = Vector3.ZERO
	position = Vector3.ZERO
	_engine_rpm = engine_idle_rpm
	_current_gear = 1
	_throttle_input = 0.0
	_brake_input = 0.0
	_steering_input = 0.0
	_clutch_input = 0.0
	_reset_lap_data()
	_shifted_this_frame = false

func _reset_lap_data() -> void:
	_lap_start_time = 0.0
	_lap_completed_count = 0
	_last_checkpoint_time = 0.0

func _set_vehicle_position(pos: Vector3) -> void:
	position = pos

func _set_vehicle_rotation(rot: float) -> void:
	rotation.y = rot

func get_vehicle_bounding_box() -> AABB:
	return AABB(-Vector3(1.0, 0.5, 2.0), Vector3(2.0, 1.0, 4.0))

func apply_damage(damage_amount: float) -> void:
	# Apply damage to vehicle
	# Could affect handling, engine performance, etc.
	pass

func is_on_ground() -> bool:
	return position.y <= PhysicsSettings.granular_ground_height + 0.1

func has_traction() -> bool:
	return _drive_wheel_slip < slip_threshold

func get_performance_stats() -> Dictionary:
	return {
		"speed": _vehicle_speed,
		"gear": _current_gear,
		"rpm": _engine_rpm,
		"acceleration": _acceleration,
		"lap_count": _lap_completed_count,
		"traction_active": _traction_control_active
	}

# ============================================================================
# SIGNAL EMIT HELPERS
# ============================================================================

func emit_debug_message(message: String) -> void:
	if GameManager.debug_mode:
		print_debug("[VehicleController] %s" % message)

func log_event(event_type: String, details: Dictionary = {}) -> void:
	var log_entry = {
		"timestamp": Time.get_ticks_msec(),
		"type": event_type,
		"details": details,
		"vehicle_speed": _vehicle_speed,
		"gear": _current_gear,
		"rpm": _engine_rpm
	}
	
	# Could save to replay buffer here
	pass

# ============================================================================
# DESTRUCTOR CLEANUP
# ============================================================================

func _exit_tree() -> void:
	# Cleanup any resources
	_powertrain_node = null
	print_debug("VehicleController destroyed")
</FILE>