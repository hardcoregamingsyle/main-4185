extends Node3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for the racing simulator
## Handles throttle, brake, steering inputs, wheel forces, gear shifting logic
## Integrates with PhysicsSettings for all physics constants
## Copyright 2026 Thalamus Racing Simulator Project

# ============================================================================
# SIGNALS
# ============================================================================
signal rpm_changed(new_rpm: float)
signal gear_changed(old_gear: int, new_gear: int)
signal speed_changed(speed_kmh: float)
signal traction_control_active(active: bool)
signal anti_lock_braking_active(active: bool)
signal handbrake_toggled(is_active: bool)

# ============================================================================
# CONSTANTS FROM PHYSICS SETTINGS
# ============================================================================
const MAX_THROTTLE_FORCE: float = 8000.0
const MAX_BRAKE_FORCE: float = 12000.0
const MAX_STEERING_ANGLE: float = 35.0 * DEG2RAD
const STEERING_SMOOTHING_FACTOR: float = 0.15
const GEAR_RATIOS: Array[float] = [3.8, 2.2, 1.5, 1.1, 0.9, 0.75]
const FINAL_DRIVE_RATIO: float = 3.5
const CLUTCH_DISPLACEMENT_THRESHOLD: float = 0.15
const CLUTCH_ENGAGEMENT_SPEED: float = 0.02
const MIN_IDLE_RPM: float = 800.0 / 60.0  # Convert to revs per second
const MAX_ENGINE_RPM: float = 8000.0 / 60.0
const SHIFT_UP_RPM: float = 7200.0 / 60.0
const SHIFT_DOWN_RPM: float = 2000.0 / 60.0
const WHEEL_RADIUS: float = 0.32
const SUSPENSION_TRAVEL: float = 0.15
const SPRING_DAMPING: float = 15000.0
const FRICTION_COEFFICIENT: float = 1.2

# ============================================================================
# PUBLIC PROPERTIES
# ============================================================================
@export var vehicle_mass: float = 1500.0
@export var center_of_mass_offset: Vector3 = Vector3(0.0, 0.5, 0.0)
@export var wheelbase: float = 2.5
@export var track_width: float = 1.6
@export var aerodynamic_drag_coefficient: float = 0.32
@export var frontal_area: float = 2.2

# ============================================================================
# STATE VARIABLES
# ============================================================================
var current_rpm: float = MIN_IDLE_RPM
var target_rpm: float = MIN_IDLE_RPM
var current_gear: int = 0  # 0 = neutral, -1 = reverse, 1-6 = forward gears
var next_gear: int = 0
var clutch_engaged: bool = true
var clutch_position: float = 1.0  # 1.0 = fully engaged, 0.0 = fully disengaged

# Input states
var throttle_input: float = 0.0
var brake_input: float = 0.0
var steering_input: float = 0.0
var handbrake_input: float = 0.0

# Current speed and forces
var speed_kmh: float = 0.0
var acceleration: float = 0.0
var longitudinal_force: float = 0.0
var lateral_force: float = 0.0
var total_force: float = 0.0

# Wheel-specific data
var wheel_speeds: Array[float] = []  # Angular velocity of each wheel in rad/s
var wheel_forces: Array[float] = []  # Force applied to each wheel
var suspension_compression: Array[float] = []  # Compression amount for each wheel

# Traction control state
var traction_control_enabled: bool = true
var traction_control_active_state: bool = false
var slip_ratio: float = 0.0

# Anti-lock braking state
var abs_enabled: bool = true
var abs_active_state: bool = false
var wheel_lock_threshold: float = 0.15

# Engine power state
var engine_power_output: float = 0.0
var torque_at_wheels: float = 0.0
var air_density: float = 1.225

# ============================================================================
# PRIVATE NODE REFERENCES
# ============================================================================
var _rigid_body: RigidBody3D = null
var _powertrain: Powertrain = null
var _input_manager: InputManager = null
var _audio_manager: AudioManager = null
var _physics_settings: PhysicsSettings = null

# ============================================================================
# LIFECYCLE METHODS
# ============================================================================
func _ready() -> void:
	_init_references()
	_initialize_wheels()
	_connect_signals_to_managers()
	_reset_vehicle_state()

func _exit_tree() -> void:
	if _rigid_body != null:
		_rigid_body.force_mode = RigidBody3D.FORCE_MODE_FORCE
		_rigid_body.apply_central_force(Vector3.ZERO)

func _process(delta: float) -> void:
	_update_input_states()
	_handle_gear_shifting(delta)
	_calculate_engine_output(delta)
	_apply_wheel_forces(delta)
	_update_vehicle_state(delta)

func _physics_process(delta: float) -> void:
	_update_suspension_physics(delta)
	_update_aerodynamics(delta)
	_update_traction_control(delta)
	_update_abs_system(delta)
	_sync_rpm_signal()

# ============================================================================
# INITIALIZATION
# ============================================================================
func _init_references() -> void:
	_rigid_body = get_parent()
	
	# Find child nodes
	_powertrain = find_child("Powertrain", false, false) as Powertrain
	if _powertrain == null:
		push_warning("VehicleController: No Powertrain child node found")
		_powertrain = Powertrain.new()
	
	# Get singleton references
	_input_manager = GameManager.get_node("/root/InputManager") if has_node("/root/InputManager") else null
	_audio_manager = GameManager.get_node("/root/AudioManager") if has_node("/root/AudioManager") else null
	_physics_settings = GameManager.get_node("/root/PhysicsSettings") if has_node("/root/PhysicsSettings") else null
	
	if _physics_settings == null:
		_physics_settings = load("res://scripts/core/PhysicsSettings.gd").new()

func _initialize_wheels() -> void:
	# Initialize wheel arrays (4 wheels: FL, FR, RL, RR)
	wheel_speeds.resize(4)
	wheel_forces.resize(4)
	suspension_compression.resize(4)
	
	for i in range(4):
		wheel_speeds[i] = 0.0
		wheel_forces[i] = 0.0
		suspension_compression[i] = 0.0

func _connect_signals_to_managers() -> void:
	if _input_manager != null:
		_input_manager.input_event.connect(_on_input_event)

# ============================================================================
# INPUT HANDLING
# ============================================================================
func _update_input_states() -> void:
	if _input_manager == null:
		return
	
	# Read normalized input values (-1.0 to 1.0)
	throttle_input = clamp(_input_manager.get_axis("throttle"), 0.0, 1.0)
	brake_input = clamp(_input_manager.get_axis("brake"), 0.0, 1.0)
	steering_input = clamp(_input_manager.get_axis("steering"), -1.0, 1.0)
	handbrake_input = clamp(_input_manager.get_axis("handbrake"), 0.0, 1.0)
	
	# Handle clutch input (if exists)
	var clutch_input = _input_manager.get_axis("clutch", 0.0)
	if abs(clutch_input) > CLUTCH_DISPLACEMENT_THRESHOLD:
		if clutch_input > 0:
			clutch_position = lerp(clutch_position, 0.0, CLUTCH_ENGAGEMENT_SPEED)
		else:
			clutch_position = lerp(clutch_position, 1.0, CLUTCH_ENGAGEMENT_SPEED)

func _on_input_event(event: Dictionary) -> void:
	if event.type == "gear_shift_up":
		request_gear_shift(true)
	elif event.type == "gear_shift_down":
		request_gear_shift(false)
	elif event.type == "toggle_traction_control":
		traction_control_enabled = !traction_control_enabled
	elif event.type == "toggle_abs":
		abs_enabled = !abs_enabled

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================
func request_gear_shift(shift_up: bool) -> void:
	if shift_up:
		if current_gear < 6:
			next_gear = current_gear + 1
		elif current_gear == 6 and throttle_input < 0.1:
			next_gear = 0  # Neutral
	else:
		if current_gear > 0:
			next_gear = current_gear - 1
		elif current_gear == 0:
			next_gear = -1  # Reverse

func _handle_gear_shifting(delta: float) -> void:
	# Smooth gear transition
	if next_gear != current_gress:
		if clutch_position < CLUTCH_DISPLACEMENT_THRESHOLD:
			# Clutch is disengaged, allow gear change
			old_gear = current_gear
			current_gear = next_gear
			clutch_position = 1.0
			
			gear_changed.emit(old_gear, current_gear)
			
			# Play shift sound
			if _audio_manager != null:
				_audio_manager.play_sound("gear_shift")
			
			# Calculate gear ratio for current gear
			_current_gear_ratio = _get_current_gear_ratio()

# ============================================================================
# ENGINE POWER CALCULATION
# ============================================================================
func _calculate_engine_output(delta: float) -> void:
	# Calculate target RPM based on gear and throttle
	var target_rpm = _calculate_target_rpm()
	target_rpm = clamp(target_rpm, MIN_IDLE_RPM, MAX_ENGINE_RPM)
	
	# Smooth RPM transition
	current_rpm = lerp(current_rpm, target_rpm, delta * 10.0)
	
	# Calculate engine torque based on RPM curve
	var engine_torque = _calculate_engine_torque(current_rpm)
	
	# Apply gear reduction to get wheel torque
	if clutch_position > 0.5 and current_gear != 0:
		var gear_reduction = _current_gear_ratio * FINAL_DRIVE_RATIO
		torque_at_wheels = engine_torque * gear_reduction * clutch_position
	else:
		torque_at_wheels = 0.0
	
	# Calculate power output (kW)
	engine_power_output = (torque_at_wheels * current_rpm * 2.0 * PI) / 1000.0
	
	# Distribute torque to wheels based on drivetrain type (RWD for now)
	_distribute_torque_to_wheels()

func _calculate_target_rpm() -> float:
	if current_gear == 0:
		# Neutral - idle or rev based on throttle
		if throttle_input > 0.1:
			return MIN_IDLE_RPM + (throttle_input * (MAX_ENGINE_RPM - MIN_IDLE_RPM) * 0.5)
		return MIN_IDLE_RPM
	
	if current_gear == -1:
		# Reverse
		if brake_input > 0.1:
			return MIN_IDLE_RPM
		return MIN_IDLE_RPM + (throttle_input * 1000.0)
	
	# Forward gears - calculate RPM based on vehicle speed
	var wheel_angular_velocity = _calculate_wheel_angular_velocity()
	var gear_ratio = _current_gear_ratio
	var final_drive = FINAL_DRIVE_RATIO
	
	var calculated_rpm = (wheel_angular_velocity * WHEEL_RADIUS) / (gauge_ratio * final_drive * WHEEL_RADIUS)
	calculated_rpm *= 60.0  # Convert to RPM
	
	if throttle_input > 0.3:
		calculated_rpm += (throttle_input * 500.0)  # Add throttle buffer
	
	return calculated_rpm

func _calculate_engine_torque(rpm: float) -> float:
	# Simplified torque curve - peak around mid-RPM
	var normalized_rpm = (rpm - MIN_IDLE_RPM) / (MAX_ENGINE_RPM - MIN_IDLE_RPM)
	
	# Torque curve shape (peak at ~4500 RPM)
	var torque_peak_rpm = 4500.0 / 60.0
	var torque_factor = exp(-pow((rpm - torque_peak_rpm) / 2000.0, 2))
	
	var max_torque = 500.0  # Nm
	var torque = max_torque * torque_factor
	
	# Reduce torque at extreme RPMs
	if rpm > torque_peak_rpm * 1.5:
		torque *= 0.5
	if rpm < MIN_IDLE_RPM * 1.2:
		torque *= 0.3
	
	return torque

func _distribute_torque_to_wheels() -> void:
	# Simple RWD distribution
	wheel_forces[2] = torque_at_wheels / 2.0  # Rear Left
	wheel_forces[3] = torque_at_wheels / 2.0  # Rear Right
	wheel_forces[0] = 0.0  # Front Left
	wheel_forces[1] = 0.0  # Front Right

# ============================================================================
# VEHICLE PHYSICS
# ============================================================================
func _apply_wheel_forces(delta: float) -> void:
	if _rigid_body == null:
		return
	
	# Clear previous forces
	_rigid_body.clear_forces()
	_rigid_body.clear_torques()
	
	# Calculate wheel positions relative to vehicle center
	var wheel_positions = _get_wheel_positions()
	
	# Apply driving forces
	for i in range(4):
		if i >= 2:  # Only rear wheels drive
			var force = wheel_forces[i] * clutch_position
			if force > 0:
				# Driving force - apply along vehicle forward direction
				var forward_dir = _rigid_body.transform.basis.z
				_rigid_body.apply_impulse(force * forward_dir * delta, wheel_positions[i])
		
		# Apply braking forces
		if brake_input > 0.0:
			var brake_force = MAX_BRAKE_FORCE * brake_input * 0.25
			if abs(brake_force) > abs(wheel_forces[i]):
				var brake_dir = -_rigid_body.transform.basis.z
				_rigid_body.apply_impulse(brake_force * brake_dir * delta, wheel_positions[i])

func _update_suspension_physics(delta: float) -> void:
	if _rigid_body == null:
		return
	
	# Simple spring-damper model for suspension
	var gravity_force = _physics_settings.gravity * vehicle_mass
	var suspension_stiffness = SPRING_DAMPING * vehicle_mass / 1000.0
	
	for i in range(4):
		var contact_point = _get_wheel_contact_point(i)
		var compression = max(0.0, SUSPENSION_TRAVEL - contact_point.y)
		suspension_compression[i] = compression
		
		# Spring force
		var spring_force = compression * suspension_stiffness
		
		# Damping force (simplified)
		var damping_force = 0.0  # Would need wheel velocity for accurate damping
		
		# Total suspension force
		var total_suspension_force = spring_force + damping_force

func _update_aerodynamics(delta: float) -> void:
	if _rigid_body == null:
		return
	
	# Calculate drag force: F = 0.5 * rho * v^2 * Cd * A
	var velocity = _rigid_body.linear_velocity
	var speed_mps = velocity.length()
	
	var drag_force = 0.5 * air_density * pow(speed_mps, 2) * aerodynamic_drag_coefficient * frontal_area
	
	# Apply drag opposite to velocity direction
	if speed_mps > 0.1:
		var drag_direction = -velocity.normalized()
		_rigid_body.apply_central_force(drag_direction * drag_force)

func _update_traction_control(delta: float) -> void:
	if not traction_control_enabled:
		traction_control_active_state = false
		return
	
	# Calculate slip ratio for driven wheels
	slip_ratio = _calculate_slip_ratio()
	
	# Activate TC if slip exceeds threshold
	if slip_ratio > 0.15:
		traction_control_active_state = true
		reduce_engine_power(slip_ratio)
	else:
		traction_control_active_state = false
	
	# Emit signal
	traction_control_active.emit(traction_control_active_state)

func _update_abs_system(delta: float) -> void:
	if not abs_enabled:
		abs_active_state = false
		return
	
	# Check for wheel lockup during braking
	for i in range(4):
		if wheel_speeds[i] < wheel_lock_threshold and brake_input > 0.3:
			abs_active_state = true
			modulate_brake_force(i)
			break
	
	abs_active_state = false  # Reset if no wheels locked
	anti_lock_braking_active.emit(abs_active_state)

func _calculate_slip_ratio() -> float:
	# Slip ratio = (wheel_speed - vehicle_speed) / vehicle_speed
	var wheel_avg_speed = 0.0
	for i in range(2, 4):  # Driven wheels only
		wheel_avg_speed += wheel_speeds[i]
	wheel_avg_speed /= 2.0
	
	var vehicle_speed = speed_kmh / 3.6  # Convert to m/s
	if vehicle_speed < 0.1:
		return 0.0
	
	return (wheel_avg_speed * WHEEL_RADIUS - vehicle_speed) / vehicle_speed

func reduce_engine_power(slip_ratio: float) -> void:
	# Reduce torque based on slip severity
	var reduction_factor = 1.0 - (slip_ratio * 0.5)
	torque_at_wheels *= max(0.5, reduction_factor)

func modulate_brake_force(wheel_index: int) -> void:
	# Pulsate brake force to prevent lockup
	pass  # Implementation would require direct wheel access

# ============================================================================
# STATE UPDATE
# ============================================================================
func _update_vehicle_state(delta: float) -> void:
	if _rigid_body == null:
		return
	
	# Get current speed from rigid body
	var velocity = _rigid_body.linear_velocity
	speed_kmh = velocity.length() * 3.6  # Convert to km/h
	
	# Calculate acceleration
	var old_acceleration = acceleration
	acceleration = (speed_kmh - (old_acceleration * delta * 3.6)) / delta
	
	# Calculate total forces
	longitudinal_force = engine_power_output / (speed_kmh / 3.6 + 0.1)
	lateral_force = _calculate_lateral_forces()
	total_force = sqrt(pow(longitudinal_force, 2) + pow(lateral_force, 2))
	
	# Update wheel speeds
	_update_wheel_speeds()

func _calculate_lateral_forces() -> float:
	# Simplified lateral force calculation based on steering and cornering
	var steering_angle = steering_input * MAX_STEERING_ANGLE
	var cornering_force = steering_angle * friction_coefficient * vehicle_mass * 0.5
	return cornering_force

func _update_wheel_speeds() -> void:
	# Update wheel angular velocities based on vehicle speed
	var vehicle_linear_speed = _rigid_body.linear_velocity.length()
	var wheel_angular_speed = vehicle_linear_speed / WHEEL_RADIUS
	
	for i in range(4):
		if i >= 2:  # Driven wheels
			wheel_speeds[i] = wheel_angular_speed + (engine_power_output * 0.01)
		else:
			wheel_speeds[i] = wheel_angular_speed

func _sync_rpm_signal() -> void:
	rpm_changed.emit(current_rpm)
	speed_changed.emit(speed_kmh)

# ============================================================================
# HELPER METHODS
# ============================================================================
func _get_wheel_positions() -> Array[Vector3]:
	var positions = []
	var half_track = track_width / 2.0
	var half_wheelbase = wheelbase / 2.0
	
	# Front Left
	positions.append(Vector3(-half_track, 0.0, half_wheelbase))
	# Front Right
	positions.append(Vector3(half_track, 0.0, half_wheelbase))
	# Rear Left
	positions.append(Vector3(-half_track, 0.0, -half_wheelbase))
	# Rear Right
	positions.append(Vector3(half_track, 0.0, -half_wheelbase))
	
	# Transform to world space
	for i in range(4):
		positions[i] = _rigid_body.global_transform * positions[i]
	
	return positions

func _get_wheel_contact_point(index: int) -> Vector3:
	return _get_wheel_positions()[index]

func _calculate_wheel_angular_velocity() -> float:
	if _rigid_body == null:
		return 0.0
	
	var linear_velocity = _rigid_body.linear_velocity
	return linear_velocity.length() / WHEEL_RADIUS

func _get_current_gear_ratio() -> float:
	if current_gear <= 0:
		return 0.0
	return GEAR_RATIOS[current_gear - 1]

func _reset_vehicle_state() -> void:
	current_rpm = MIN_IDLE_RPM
	current_gear = 0
	next_gear = 0
	clutch_engaged = true
	clutch_position = 1.0
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	handbrake_input = 0.0
	speed_kmh = 0.0
	acceleration = 0.0
	longitudinal_force = 0.0
	lateral_force = 0.0
	total_force = 0.0
	wheel_speeds.fill(0.0)
	wheel_forces.fill(0.0)
	suspension_compression.fill(0.0)
	traction_control_active_state = false
	abs_active_state = false
	engine_power_output = 0.0
	torque_at_wheels = 0.0

# ============================================================================
# DEBUG TOOLS
# ============================================================================
func debug_print_vehicle_state() -> void:
	print("=== Vehicle Controller Debug ===")
	print("RPM: %.2f" % (current_rpm * 60.0))
	print("Gear: %d" % current_gear)
	print("Speed: %.2f km/h" % speed_kmh)
	print("Throttle: %.2f" % throttle_input)
	print("Brake: %.2f" % brake_input)
	print("Steering: %.2f" % steering_input)
	print("Clutch: %.2f" % clutch_position)
	print("Traction Control: %s" % ("ACTIVE" if traction_control_active_state else "OFF"))
	print("ABS: %s" % ("ACTIVE" if abs_active_state else "OFF"))
	print("Engine Power: %.2f kW" % engine_power_output)
	print("Torque at Wheels: %.2f Nm" % torque_at_wheels)
	print("==============================")

</FILE>