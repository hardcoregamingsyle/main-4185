extends Node2D
class_name VehicleController

## VehicleController - Core vehicle physics controller using PhysicsSettings constants
## Handles throttle, brake, steering inputs, wheel forces, and gear shifting logic
## Copyright 2026 Thalamus Racing Simulator Project

signal speed_changed(new_speed: float)
signal gear_changed(old_gear: int, new_gear: int)
signal vehicle_moved(displacement: Vector2)
signal drift_angle_changed(angle: float)
signal traction_control_active(active: bool)
signal engine_rpm_changed(rpm: float)
signal collision_detected(collision_type: String, force: float)

# ============================================================================
# PHYSICS CONSTANTS - Derived from PhysicsSettings resource
# ============================================================================

const MAX_THROTTLE_FORCE: float = 15000.0      # Newtons - maximum acceleration force
const MAX_BRAKE_FORCE: float = 20000.0         # Newtons - maximum braking force
const MAX_STEERING_ANGLE: float = PI / 3       # 60 degrees max steering
const STEERING_SPEED: float = 4.0              # Radians per second steering rate
const DRIFT_THRESHOLD: float = 0.7             # Sideslip threshold for drift mode
const TRACTION_CONTROL_SENSITIVITY: float = 0.85 # TCS activation threshold
const MIN_RPM_IDLE: float = 800.0              # Idle RPM
const MAX_RPM_REDLINE: float = 7500.0          # Redline RPM
const OPTIMAL_POWER_RPM_START: float = 3500.0  # Start of power band
const OPTIMAL_POWER_RPM_END: float = 6500.0    # End of power band
const GEAR_UPSHIFT_RPM: float = 6800.0         # RPM threshold for upshifting
const GEAR_DOWNSHIFT_RPM: float = 2000.0       # RPM threshold for downshifting
const CLUTCH_RELEASE_TIME: float = 0.2         # Seconds for smooth clutch engagement

# ============================================================================
# GEAR RATIOS AND TRANSMISSION CONFIGURATION
# ============================================================================

enum Gear {
    NEUTRAL = 0,
    FIRST = 1,
    SECOND = 2,
    THIRD = 3,
    FOURTH = 4,
    FIFTH = 5,
    SIXTH = 6,
    REVERSE = -1
}

const GEAR_RATIOS: Dictionary = {
    Gear.FIRST: 3.5,
    Gear.SECOND: 2.2,
    Gear.THIRD: 1.6,
    Gear.FOURTH: 1.2,
    Gear.FIFTH: 0.9,
    Gear.SIXTH: 0.75,
    Gear.REVERSE: 3.0
}

const FINAL_DRIVE_RATIO: float = 3.73          # Final drive differential ratio
const REVERSE_GEAR_RATIO: float = 3.5          # Reverse gear ratio multiplier

# ============================================================================
# VEHICLE STATE VARIABLES
# ============================================================================

@export var vehicle_mass: float = 1500.0      # kg - overridden by PhysicsSettings default
@export var center_of_gravity: Vector2 = Vector2(0.0, 0.5)  # CG position relative to chassis
@export var wheelbase: float = 2.6           # Distance between front and rear axles (meters)
@export var track_width: float = 1.6         # Distance between left and right wheels (meters)
@export var wheel_radius: float = 0.3        # Wheel radius in meters
@export var tire_friction_coefficient: float = 1.15  # Dry asphalt coefficient
@export var aerodynamic_drag_coefficient: float = 0.32  # Cd value
@export var frontal_area: float = 2.2        # Frontal area in square meters

# Runtime state variables
var current_speed: float = 0.0               # km/h
var current_rpm: float = MIN_RPM_IDLE
var current_gear: Gear = Gear.NEUTRAL
var steering_angle: float = 0.0              # Current steering angle in radians
var target_steering_angle: float = 0.0       # Target steering angle
var acceleration: float = 0.0                # Longitudinal acceleration m/s^2
var lateral_acceleration: float = 0.0        # Lateral acceleration m/s^2
var is_drifting: bool = false
var is_traction_control_active: bool = false
var clutch_engaged: bool = true
var brake_pressure: float = 0.0              # 0.0 to 1.0 scale

# Input state
var throttle_input: float = 0.0              # 0.0 to 1.0 scale
var brake_input: float = 0.0                 # 0.0 to 1.0 scale
var steering_input: float = 0.0              # -1.0 to 1.0 scale
var gear_input_direction: int = 0            # -1 down, 0 none, 1 up

# Previous frame data for delta calculations
var previous_position: Vector2 = Vector2.ZERO
var previous_velocity: Vector2 = Vector2.ZERO
var drift_angle: float = 0.0                 # Angle between velocity vector and heading
var slip_ratio: float = 0.0                  # Wheel slip ratio
var wheel_slip_angles: Array[float] = []     # Slip angles for each wheel

# Physical simulation components
var _powertrain_node: Node = null
var _wheel_nodes: Array[Node2D] = []
var _collider_shape: CollisionShape2D = null
var _rigid_body_2d: RigidBody2D = null

# Time tracking
var _last_update_time: float = 0.0
var _clutch_release_timer: float = 0.0

# ============================================================================
# PUBLIC INTERFACE
# ============================================================================

func get_current_speed() -> float:
	return current_speed

func get_current_rpm() -> float:
	return current_rpm

func get_current_gear() -> Gear:
	return current_gear

func get_steering_angle() -> float:
	return steering_angle

func get_drift_angle() -> float:
	return drift_angle

func get_is_traction_control_active() -> bool:
	return is_traction_control_active

func get_wheel_slip_ratio(wheel_index: int) -> float:
	if wheel_index >= 0 and wheel_index < wheel_slip_sizes.size():
		return wheel_slip_ratios[wheel_index]
	return 0.0

func set_vehicle_mass(mass: float) -> void:
	vehicle_mass = max(mass, 500.0)  # Minimum reasonable mass

func reset_vehicle_state() -> void:
	current_speed = 0.0
	current_rpm = MIN_RPM_IDLE
	current_gear = Gear.NEUTRAL
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	gear_input_direction = 0
	clutch_engaged = true
	brake_pressure = 0.0
	is_drifting = false
	is_traction_control_active = false
	wheel_slip_ratios = [0.0, 0.0, 0.0, 0.0]

# ============================================================================
# INPUT HANDLING
# ============================================================================

func handle_input(event: InputEvent) -> void:
	if not is_instance_valid(_rigid_body_2d):
		return
	
	if event.is_action_pressed("throttle"):
		throttle_input = 1.0
	elif event.is_action_released("throttle"):
		throttle_input = 0.0
	
	if event.is_action_pressed("brake"):
		brake_input = 1.0
	elif event.is_action_released("brake"):
		brake_input = 0.0
	
	if event.is_action_pressed("steer_left"):
		steering_input = -1.0
	elif event.is_action_pressed("steer_right"):
		steering_input = 1.0
	elif event.is_action_released("steer_left") and event.is_action_released("steer_right"):
		steering_input = 0.0
	
	# Gear shifting with input buffering
	if event.is_action_pressed("gear_up"):
		request_shift(Gear.UP)
	elif event.is_action_pressed("gear_down"):
		request_shift(Gear.DOWN)

func process_continuous_inputs(delta: float) -> void:
	# Smooth steering interpolation
	target_steering_angle = steering_input * MAX_STEERING_ANGLE
	var steering_diff = target_steering_angle - steering_angle
	if abs(steering_diff) < STEERING_SPEED * delta:
		steering_angle = target_steering_angle
	else:
		steering_angle += sign(steering_diff) * STEERING_SPEED * delta
	
	# Smooth brake pressure application
	if brake_input > 0:
		_brake_pressure += min(brake_input * 5.0 * delta, 1.0 - brake_pressure)
	else:
		_brake_pressure -= min(3.0 * delta, brake_pressure)
	brake_pressure = clamp(_brake_pressure, 0.0, 1.0)
	
	# Clutch management during gear shifts
	if _clutch_release_timer > 0.0:
		_clutch_release_timer -= delta
		if _clutch_release_timer <= 0.0:
			clutch_engaged = true

# ============================================================================
# PHYSICS UPDATE
# ============================================================================

func _physics_process(delta: float) -> void:
	if not is_instance_valid(_rigid_body_2d):
		return
	
	_last_update_time = Time.get_ticks_usec()
	
	# Process continuous inputs
	process_continuous_inputs(delta)
	
	# Update physics based on current state
	_update_powertrain(delta)
	_update_vehicle_motion(delta)
	_update_wheels(delta)
	_check_drift_conditions(delta)
	_apply_forces_to_body(delta)
	
	# Emit signals for changes
	_emit_state_signals(delta)

func _update_powertrain(delta: float) -> void:
	# Calculate engine torque curve based on RPM
	var torque_curve_factor = _calculate_torque_curve(current_rpm)
	var gear_ratio = GEAR_RATIOS[current_gear] if current_gear != Gear.NEUTRAL else 0.0
	
	# Engine output torque
	var engine_torque = torque_curve_factor * 400.0  # Max 400 Nm
	
	# Transmission losses (2% per gear)
	var transmission_efficiency = pow(0.98, gear_ratio) if gear_ratio > 0 else 0.0
	engine_torque *= transmission_efficiency
	
	# Apply clutch engagement factor
	var clutch_factor = 1.0 if clutch_engaged else 0.0
	engine_torque *= clutch_factor
	
	# Calculate wheel torque
	var final_drive = engine_torque * gear_ratio * FINAL_DRIVE_RATIO
	var wheel_torque = final_drive / 4.0  # Distribute to 4 wheels
	
	# Throttle affects torque delivery
	var torque_delivery = engine_torque * throttle_input
	
	# Brake override
	if brake_input > 0:
		var braking_torque = MAX_BRAKE_FORCE * brake_pressure * wheel_radius
		wheel_torque = max(0.0, wheel_torque - braking_torque)
	
	# Update wheel torques
	for i in range(4):
		wheel_torques[i] = wheel_torque

func _calculate_torque_curve(rpm: float) -> float:
	"""Calculate torque curve factor based on RPM (bell curve centered at optimal power)"""
	if rpm < MIN_RPM_IDLE or rpm > MAX_RPM_REDLINE:
		return 0.0
	
	# Simple bell curve approximation
	var normalized_rpm = (rpm - MIN_RPM_IDLE) / (MAX_RPM_REDLINE - MIN_RPM_IDLE)
	
	# Peak torque around 4000-5000 RPM
	var peak_rpm = 4500.0
	var rpm_deviation = abs(rpm - peak_rpm)
	var torque_factor = 1.0 - (rpm_deviation / 2000.0)
	torque_factor = max(0.0, torque_factor)
	
	return torque_factor

func _update_vehicle_motion(delta: float) -> void:
	# Calculate net force on vehicle
	var total_force = _calculate_total_force()
	
	# Apply drag
	var drag_force = 0.5 * aerodynamic_drag_coefficient * frontal_area * current_speed * current_speed
	total_force -= drag_force
	
	# Calculate acceleration (F = ma)
	acceleration = total_force / vehicle_mass
	
	# Update velocity
	var velocity_change = acceleration * delta
	current_speed += velocity_change
	
	# Clamp speed to realistic limits
	current_speed = clamp(current_speed, -MAX_SPEED_KMH, MAX_SPEED_KMH)
	
	# Update position based on velocity
	var displacement = Vector2.RIGHT.rotated(rotation).normalized() * current_speed * delta
	global_position += displacement
	
	# Track movement for signals
	var movement_delta = global_position - previous_position
	if movement_delta.length() > 0.01:
		emit_signal("vehicle_moved", movement_delta)
	
	previous_position = global_position

func _calculate_total_force() -> float:
	"""Calculate total longitudinal force from all wheels"""
	var total_force = 0.0
	
	for i in range(4):
		var wheel_torque = wheel_torques[i]
		var wheel_force = wheel_torque / wheel_radius
		
		# Apply traction control if active
		if is_traction_control_active:
			wheel_force *= _apply_traction_control(i)
		
		total_force += wheel_force
	
	return total_force

func _apply_traction_control(wheel_index: int) -> float:
	"""Apply traction control reduction if wheel slip exceeds threshold"""
	var slip_ratio = wheel_slip_ratios[wheel_index]
	
	if abs(slip_ratio) > TRACTION_CONTROL_SENSITIVITY:
		var reduction = (abs(slip_ratio) - TRACTION_CONTROL_SENSITIVITY) * 0.5
		return max(0.0, 1.0 - reduction)
	
	return 1.0

func _update_wheels(delta: float) -> void:
	"""Update individual wheel states and calculate slip ratios"""
	for i in range(4):
		var wheel_speed_rad = current_speed / wheel_radius * (PI / 180.0)
		var wheel_torque = wheel_torques[i]
		var wheel_force = wheel_torque / wheel_radius
		
		# Calculate wheel angular acceleration
		var wheel_inertia = 1.5  # Approximate wheel moment of inertia
		var angular_acceleration = wheel_force * wheel_radius / wheel_inertia
		
		# Update wheel angular velocity
		wheel_angular_velocities[i] += angular_acceleration * delta
		
		# Calculate slip ratio
		var wheel_linear_speed = wheel_angular_velocities[i] * wheel_radius
		var slip_ratio_value = (wheel_linear_speed - current_speed) / max(current_speed, 0.01)
		wheel_slip_ratios[i] = clamp(slip_ratio_value, -1.0, 1.0)

func _check_drift_conditions(delta: float) -> void:
	"""Check if vehicle is entering or exiting drift mode"""
	# Calculate sideslip angle
	var velocity_vector = global_position - previous_position
	var heading_vector = Vector2.RIGHT.rotated(rotation)
	
	if velocity_vector.length() > 0.1:
		var velocity_angle = velocity_vector.angle()
		var heading_angle = rotation
		drift_angle = abs(velocity_angle - heading_angle)
		
		# Normalize drift angle to [-PI, PI]
		while drift_angle > PI:
			drift_angle -= 2.0 * PI
		while drift_angle < -PI:
			drift_angle += 2.0 * PI
		
		drift_angle = abs(drift_angle)
		
		# Check drift threshold
		var was_drifting = is_drifting
		is_drifting = drift_angle > DRIFT_THRESHOLD
		
		if is_drifting != was_drifting:
			emit_signal("drift_angle_changed", drift_angle)
			emit_signal("traction_control_active", is_drifting)

func _apply_forces_to_body(delta: float) -> void:
	"""Apply calculated forces to rigid body"""
	if not is_instance_valid(_rigid_body_2d):
		return
	
	# Apply acceleration as linear velocity change
	var forward_direction = Vector2.RIGHT.rotated(rotation)
	_rigid_body_2d.apply_central_impulse(forward_direction * acceleration * vehicle_mass * delta)
	
	# Apply rotational forces based on steering
	var steering_torque = steering_input * 500.0 * delta
	_rigid_body_2d.apply_torque(steering_torque)

func _emit_state_signals(delta: float) -> void:
	"""Emit signals when state changes"""
	if abs(current_speed - last_reported_speed) > 1.0:
		emit_signal("speed_changed", current_speed)
		last_reported_speed = current_speed
	
	if abs(current_rpm - last_reported_rpm) > 100:
		emit_signal("engine_rpm_changed", current_rpm)
		last_reported_rpm = current_rpm

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================

enum ShiftDirection { UP, DOWN }

func request_shift(direction: ShiftDirection) -> void:
	if current_gear == Gear.NEUTRAL:
		return
	
	if direction == ShiftDirection.UP:
		if current_gear < Gear.SIXTH:
			_execute_shift(current_gear + 1)
	elif direction == ShiftDirection.DOWN:
		if current_gear > Gear.FIRST:
			_execute_shift(current_gear - 1)
		elif current_gear == Gear.FIRST and current_speed < 10.0:
			_execute_shift(Gear.NEUTRAL)

func _execute_shift(target_gear: Gear) -> void:
	if target_gear == current_gear:
		return
	
	var old_gear = current_gear
	current_gear = target_gear
	
	# Disengage clutch
	clutch_engaged = false
	_clutch_release_timer = CLUTCH_RELEASE_TIME
	
	# Calculate RPM drop/gain for smoother transition
	var old_gear_ratio = GEAR_RATIOS[old_gear]
	var new_gear_ratio = GEAR_RATIOS[target_gear] if target_gear != Gear.NEUTRAL else 0.0
	
	if new_gear_ratio > 0 and old_gear_ratio > 0:
		var rpm_multiplier = old_gear_ratio / new_gear_ratio
		var expected_rpm = current_rpm * rpm_multiplier
		current_rpm = clamp(expected_rpm, MIN_RPM_IDLE, MAX_RPM_REDLINE)
	
	emit_signal("gear_changed", old_gear, target_gear)

# ============================================================================
# COLLISION HANDLING
# ============================================================================

func handle_collision(collision_info: Dictionary) -> void:
	var collision_type = collision_info.get("type", "unknown")
	var impact_force = collision_info.get("force", 0.0)
	
	# Apply damage based on impact force
	var damage_threshold = 5000.0
	if impact_force > damage_threshold:
		_trigger_damage_response(impact_force)
	
	emit_signal("collision_detected", collision_type, impact_force)

func _trigger_damage_response(impact_force: float) -> void:
	"""Handle high-impact collision effects"""
	# Reduce speed significantly
	current_speed *= 0.3
	
	# Reset drift state
	is_drifting = false
	
	# Trigger audio feedback via GameManager
	if is_instance_valid(GameManager):
		GameManager.trigger_audio_event("collision_impact", impact_force)

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================

func get_gear_ratio(gear: Gear) -> float:
	return GEAR_RATIOS[gear] if GEAR_RATIOS.has(gear) else 0.0

func get_optimal_shift_point() -> float:
	return GEAR_UPSHIFT_RPM

func get_minimum_downshift_speed(gear: Gear) -> float:
	"""Get minimum speed before downshifting to prevent stalling"""
	var gear_ratio = GEAR_RATIOS[gear]
	var min_speed = (MIN_RPM_IDLE / MAX_RPM_REDLINE) * 120.0  # Rough conversion
	return max(min_speed, 5.0)

func calculate_lap_time(start_time: float) -> float:
	"""Calculate elapsed lap time"""
	return Time.get_ticks_msec() - start_time

func reset_lap_timer() -> void:
	"""Reset lap timing"""
	pass  # Implementation would integrate with GameManager's lap system

# ============================================================================
# DEBUGGING AND VISUALIZATION
# ============================================================================

func debug_print_state() -> void:
	print("\n=== VEHICLE STATE DEBUG ===")
	print("Speed: %.2f km/h" % current_speed)
	print("RPM: %.0f" % current_rpm)
	print("Gear: %s" % str(current_gear))
	print("Steering: %.2f rad (%.1f deg)" % [steering_angle, deg_to_rad(steering_angle)])
	print("Drift: %s (%.2f rad)" % [is_drifting, drift_angle])
	print("Traction Control: %s" % is_traction_control_active)
	print("Throttle: %.2f" % throttle_input)
	print("Brake: %.2f" % brake_input)
	print("Slip Ratios: [%s]" % ", ".join([str("%.3f" % r) for r in wheel_slip_ratios]))
	print("==========================\n")

func set_debug_mode(enabled: bool) -> void:
	debug_mode = enabled
	if enabled:
		$DebugDisplay.visible = true
	else:
		$DebugDisplay.visible = false

# ============================================================================
# INITIALIZATION AND CLEANUP
# ============================================================================

func _ready() -> void:
	_initialize_components()
	_connect_signals()
	_setup_physics()

func _initialize_components() -> void:
	# Find child nodes
	_rigid_body_2d = find_child("RigidBody2D") as RigidBody2D
	_collider_shape = find_child("CollisionShape2D") as CollisionShape2D
	
	# Find powertrain node
	_powertrain_node = find_child("Powertrain")
	
	# Find wheel nodes
	_wheel_nodes.clear()
	for child in get_children():
		if child.name.begins_with("Wheel"):
			_wheel_nodes.append(child)
	
	# Initialize arrays
	wheel_slip_ratios = [0.0, 0.0, 0.0, 0.0]
	wheel_torques = [0.0, 0.0, 0.0, 0.0]
	wheel_angular_velocities = [0.0, 0.0, 0.0, 0.0]

func _connect_signals() -> void:
	# Connect to GameManager for global coordination
	if is_instance_valid(GameManager):
		GameManager.game_state_changed.connect(_on_game_state_changed)

func _on_game_state_changed(new_state: GameManager.GameState) -> void:
	match new_state:
		GameManager.GameState.RACE_ACTIVE:
			# Resume physics processing
			_process_mode = ProcessModeEnum.ALWAYS
		GameManager.GameState.RACE_PAUSED:
			# Pause physics processing
			_process_mode = ProcessModeEnum.IDLE
		GameManager.GameState.MAIN_MENU:
			# Reset vehicle state for menu
			reset_vehicle_state()

func _setup_physics() -> void:
	# Configure rigid body properties
	if is_instance_valid(_rigid_body_2d):
		_rigid_body_2d.mass = vehicle_mass
		_rigid_body_2d.linear_damping = 0.1
		_rigid_body_2d.angular_damping = 0.5
		
		# Set center of mass
		_rigid_body_2d.center_of_mass = center_of_gravity

func _exit_tree() -> void:
	"""Cleanup when node is removed"""
	_disconnect_signals()

func _disconnect_signals() -> void:
	if is_instance_valid(GameManager):
		GameManager.game_state_changed.disconnect(_on_game_state_changed)

# ============================================================================
# EXPORTED FOR EDITOR/DEBUGGING
# ============================================================================

@export_group("Vehicle Configuration")
@export var display_speedometer: bool = true
@export var display_tachometer: bool = true
@export var show_debug_visualization: bool = false

@export_group("Performance Tuning")
@export var enable_suspension_simulation: bool = false
@export var enable_aerodynamics: bool = true
@export var enable_engine_braking: bool = true

</file_content>