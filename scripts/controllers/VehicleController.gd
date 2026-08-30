extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulator
## Implements throttle, brake, steering inputs, wheel forces, and gear shifting logic
## Uses PhysicsSettings constants for centralized tuning
## Copyright 2026 Thalamus Racing Simulator Project

# ============================================================================
# SIGNALS
# ============================================================================
signal speed_changed(new_speed: float)
signal rpm_changed(new_rpm: float)
signal gear_changed(old_gear: int, new_gear: int)
signal throttle_applied(amount: float)
signal brake_applied(amount: float)
signal steering_angle_changed(angle: float)
signal skidding(is_skidding: bool)
signal collision_detected(collision_info: Dictionary)
signal engine_stalled()
signal handbrake_toggled(is_active: bool)
signal traction_control_state_changed(active: bool)
signal anti_lock_braking_state_changed(active: bool)

# ============================================================================
# CONSTANTS & CONFIGURATION
# ============================================================================
const MAX_THROTTLE_FORCE: float = 8000.0
const MAX_BRAKE_FORCE: float = 15000.0
const MAX_STEERING_ANGLE: float = PI / 3  # 60 degrees
const STEERING_SPEED: float = 15.0
const MIN_GEAR_RATIO: float = 3.5
const MAX_GEAR_RATIO: float = 0.7
const GEAR_COUNT: int = 6
const CLUTCH_DISPLACEMENT: float = 0.05
const TRACTION_CONTROL_THRESHOLD: float = 0.15
const ABS_THRESHOLD: float = 0.20

# ============================================================================
# EXPORABLE SETTINGS
# ============================================================================
@export_group("Vehicle Configuration")
@export var vehicle_mass: float = 1500.0
@export var center_of_mass_offset: Vector3 = Vector3(0.0, 0.5, 0.0)
@export var wheel_base: float = 2.5
@export var track_width: float = 1.5
@export var drag_coefficient: float = 0.30
@export var frontal_area: float = 2.2
@export var rolling_resistance: float = 0.015

@export_group("Engine Characteristics")
@export var engine_max_rpm: float = 8000.0
@export var engine_idle_rpm: float = 800.0
@export var engine_peak_power_rpm: float = 6000.0
@export var engine_torque_curve: Array[float] = [0.4, 0.6, 0.8, 1.0, 0.95, 0.85, 0.70]
@export var powertrain: Powertrain = null

@export_group("Tire Properties")
@export var tire_friction: float = 1.2
@export var tire_side_friction: float = 0.8
@export var tire_stiffness: float = 25000.0
@export var tire_damping: float = 1500.0

@export_group("Transmission Settings")
@export var transmission_type: String = "manual"  # manual, automatic, semi_auto
@export var final_drive_ratio: float = 3.25
@export var gear_ratios: Array[float] = [3.50, 2.10, 1.45, 1.05, 0.80, 0.65]
@export var reverse_ratio: float = -3.80

@export_group("Suspension Settings")
@export var suspension_stiffness: float = 45000.0
@export var suspension_damping: float = 4500.0
@export var suspension_compression_limit: float = 0.25
@export var suspension_extension_limit: float = 0.35
@export var spring_rest_length: float = 0.28

@export_group("Control Systems")
@export var traction_control_enabled: bool = true
@export var abs_enabled: bool = true
@export var stability_control_enabled: bool = false
@export var differential_type: String = "limited_slip"
@export var diff_bias_front: float = 0.50

@export_group("Debug Visuals")
@export var debug_draw_wheels: bool = false
@export var debug_draw_forces: bool = false
@export var debug_spring_visualization: bool = false

# ============================================================================
# INTERNAL STATE
# ============================================================================
var _current_speed: float = 0.0
var _current_rpm: float = 0.0
var _current_gear: int = 0  # 0 = neutral, 1-6 = forward gears, -1 = reverse
var _throttle_input: float = 0.0
var _brake_input: float = 0.0
var _steering_input: float = 0.0
var _handbrake_active: bool = false
var _clutch_displacement: float = 0.0
var _engine_on: bool = true
var _is_skidding: bool = false
var _last_collision_time: float = 0.0

# Wheel states
var _wheel_states: Array[Dictionary] = []
const WHEEL_FRONT_LEFT: int = 0
const WHEEL_FRONT_RIGHT: int = 1
const WHEEL_REAR_LEFT: int = 2
const WHEEL_REAR_RIGHT: int = 3

# Physics simulation state
var _air_density: float = 1.225  # kg/m^3 at sea level
var _max_downforce: float = 0.0
var _lift_coefficient: float = 0.05
var _drag_force: float = 0.0
var _downforce_force: float = 0.0

# Gear shift state tracking
var _gear_shift_target: int = 0
var _gear_shift_progress: float = 0.0
var _shifting: bool = false
var _shift_timer: float = 0.0

# Suspension compression tracking
var _suspension_compression: Array[float] = [0.0, 0.0, 0.0, 0.0]

# Input smoothing
var _throttle_smoothed: float = 0.0
var _brake_smoothed: float = 0.0
var _steering_smoothed: float = 0.0

# ============================================================================
# INITIALIZATION
# ============================================================================
func _ready() -> void:
	_init_wheel_states()
	_reset_vehicle()
	_connect_signals_to_powertrain()

func _init_wheel_states() -> void:
	"""Initialize wheel state arrays for each wheel"""
	for i in range(4):
		_wheel_states.append({
			"position": Vector3.ZERO,
			"velocity": Vector3.ZERO,
			"angular_velocity": 0.0,
			"suspension_compression": 0.0,
			"slip_ratio": 0.0,
			"slip_angle": 0.0,
			"normal_force": 0.0,
			"driving_force": 0.0,
			"braking_force": 0.0,
			"steering_angle": 0.0,
			"is_in_contact": false
		})

func _reset_vehicle() -> void:
	"""Reset vehicle to initial state"""
	_current_speed = 0.0
	_current_rpm = 0.0
	_current_gear = 0
	_throttle_input = 0.0
	_brake_input = 0.0
	_steering_input = 0.0
	_handbrake_active = false
	_clutch_displacement = 0.0
	_engine_on = true
	_is_skidding = false
	_shifting = false
	
	for i in range(4):
		_suspension_compression[i] = 0.0
		_wheel_states[i]["suspension_compression"] = 0.0
		_wheel_states[i]["normal_force"] = vehicle_mass * PhysicsSettings.gravity / 4.0

func _connect_signals_to_powertrain() -> void:
	if powertrain != null:
		powertrain.rpm_changed.connect(_on_engine_rpm_changed)
		powertrain.engine_on_changed.connect(_on_engine_on_changed)

# ============================================================================
# MAIN PROCESS LOOP
# ============================================================================
func _physics_process(delta: float) -> void:
	if not is_inside_tree():
		return
	
	# Update input smoothing
	_update_input_smoothing(delta)
	
	# Read current inputs
	_read_inputs(delta)
	
	# Handle clutch engagement
	_handle_clutch(delta)
	
	# Calculate engine torque based on RPM and throttle
	var engine_torque: float = _calculate_engine_torque()
	
	# Apply engine torque through transmission
	var wheel_torque: float = _apply_transmission(engine_torque, delta)
	
	# Handle braking
	var braking_force: float = _handle_braking(delta)
	
	# Calculate aerodynamic forces
	_calculate_aerodynamics(delta)
	
	# Calculate suspension forces
	_calculate_suspension(delta)
	
	# Apply wheel forces to vehicle body
	_apply_wheel_forces(delta, wheel_torque, braking_force)
	
	# Handle traction control
	_handle_traction_control(delta)
	
	# Handle ABS
	_handle_abs(delta)
	
	# Check for collisions
	_check_collisions(delta)
	
	# Update gear shift state
	_update_gear_shift(delta)
	
	# Calculate current speed and RPM
	_calculate_vehicle_metrics(delta)
	
	# Emit signals
	_emit_signals()

# ============================================================================
# INPUT HANDLING
# ============================================================================
func _read_inputs(delta: float) -> void:
	"""Read and validate player inputs"""
	_throttle_input = clamp(InputManager.get_axis("throttle", "brake"), -1.0, 1.0)
	_brake_input = clamp(InputManager.get_axis("brake", "throttle"), -1.0, 1.0)
	_steering_input = clamp(InputManager.get_axis("steering_left", "steering_right"), -1.0, 1.0)
	
	# Handbrake toggle
	if InputManager.is_action_just_pressed("handbrake"):
		_handbrake_active = !_handbrake_active
		emit_signal("handbrake_toggled", _handbrake_active)
	
	# Clutch input
	_clutch_displacement = InputManager.get_axis("clutch_up", "clutch_down")
	_clutch_displacement = clamp(_clutch_displacement, 0.0, 1.0)
	
	# Gear shifting
	_handle_gear_shifting()

func _handle_gear_shifting() -> void:
	"""Handle manual gear shift inputs"""
	if transmission_type == "automatic":
		return
	
	# Upshift (Gear up)
	if InputManager.is_action_just_pressed("gear_up"):
		_request_gear_shift(_current_gear + 1)
	
	# Downshift (Gear down)
	elif InputManager.is_action_just_pressed("gear_down"):
		_request_gear_shift(_current_gear - 1)

func _request_gear_shift(target_gear: int) -> void:
	"""Request a gear shift to target gear"""
	if not _engine_on or not _shifting:
		if _current_gear != target_green:
			_gear_shift_target = target_gear
			_start_gear_shift()

func _start_gear_shift() -> void:
	"""Begin gear shift sequence"""
	if _current_gear == _gear_shift_target:
		return
	
	_shifting = true
	_shift_timer = 0.0
	_gear_shift_progress = 0.0
	
	# Disengage clutch during shift
	_clutch_displacement = 1.0

func _update_gear_shift(delta: float) -> void:
	"""Update gear shift progress"""
	if not _shifting:
		return
	
	_shift_timer += delta
	_gear_shift_progress += delta / 0.3  # 300ms shift time
	
	if _gear_shift_progress >= 1.0:
		_complete_gear_shift()

func _complete_gear_shift() -> void:
	"""Complete gear shift"""
	var old_gear: int = _current_gear
	_current_gear = _gear_shift_target
	_shifting = false
	_clutch_displacement = 0.0
	_gear_shift_progress = 0.0
	
	emit_signal("gear_changed", old_gear, _current_gear)

# ============================================================================
# PHYSICS CALCULATIONS
# ============================================================================
func _calculate_engine_torque() -> float:
	"""Calculate engine torque based on RPM curve and throttle"""
	if not _engine_on:
		return 0.0
	
	var rpm_normalized: float = (_current_rpm - engine_idle_rpm) / (engine_max_rpm - engine_idle_rpm)
	rpm_normalized = clamp(rpm_normalized, 0.0, 1.0)
	
	# Interpolate torque curve
	var torque_index: float = rpm_normalized * (engine_torque_curve.size() - 1)
	var low_index: int = floor(torque_index)
	var high_index: int = min(low_index + 1, engine_torque_curve.size() - 1)
	var lerp_factor: float = torque_index - low_index
	
	var base_torque: float = lerp(
		engine_torque_curve[low_index],
		engine_torque_curve[high_index],
		lerp_factor
	)
	
	# Apply throttle
	return base_torque * max(_throttle_smoothed, 0.0) * 400.0  # Peak ~400Nm

func _apply_transmission(engine_torque: float, delta: float) -> float:
	"""Apply engine torque through transmission to wheels"""
	if _current_gear == 0:  # Neutral
		return 0.0
	
	var gear_ratio: float = gear_ratios[_current_gear - 1] if _current_gear > 0 else reverse_ratio
	var total_ratio: float = gear_ratio * final_drive_ratio
	
	# Clutch slip effect
	var clutch_efficiency: float = 1.0 - _clutch_displacement * CLUTCH_DISPLACEMENT
	
	var transmitted_torque: float = engine_torque * total_ratio * clutch_efficiency
	
	# Differential split between wheels
	var front_rear_split: float = diff_bias_front
	var front_wheels: float = transmitted_torque * front_rear_split
	var rear_wheels: float = transmitted_torque * (1.0 - front_rear_split)
	
	# Distribute to individual wheels
	_wheel_states[WHEEL_FRONT_LEFT]["driving_force"] = front_wheels * 0.5
	_wheel_states[WHEEL_FRONT_RIGHT]["driving_force"] = front_wheels * 0.5
	_wheel_states[WHEEL_REAR_LEFT]["driving_force"] = rear_wheels * 0.5
	_wheel_states[WHEEL_REAR_RIGHT]["driving_force"] = rear_wheels * 0.5
	
	return transmitted_torque

func _handle_braking(delta: float) -> float:
	"""Handle brake system and calculate braking force"""
	var brake_force: float = 0.0
	
	# Combine foot brake and handbrake
	var combined_brake: float = _brake_smoothed
	if _handbrake_active:
		combined_brake = max(combined_brake, 0.5)
	
	if combined_brake > 0.0:
		# Brake force calculation
		brake_force = combined_brake * MAX_BRAKE_FORCE
		
		# Distribute braking force to wheels (60% front, 40% rear typical)
		var front_brake: float = brake_force * 0.6
		var rear_brake: float = brake_force * 0.4
		
		_wheel_states[WHEEL_FRONT_LEFT]["braking_force"] = front_brake * 0.5
		_wheel_states[WHEEL_FRONT_RIGHT]["braking_force"] = front_brake * 0.5
		_wheel_states[WHEEL_REAR_LEFT]["braking_force"] = rear_brake * 0.5
		_wheel_states[WHEEL_REAR_RIGHT]["braking_force"] = rear_brake * 0.5
	
	return brake_force

func _calculate_aerodynamics(delta: float) -> void:
	"""Calculate aerodynamic forces (drag, lift/downforce)"""
	var velocity_magnitude: float = linear_velocity.length()
	
	# Drag force: F_drag = 0.5 * rho * v^2 * Cd * A
	_drag_force = 0.5 * _air_density * pow(velocity_magnitude, 2) * drag_coefficient * frontal_area
	
	# Downforce (negative lift): F_downforce = 0.5 * rho * v^2 * Cl * A
	_downforce_force = 0.5 * _air_density * pow(velocity_magnitude, 2) * _lift_coefficient * frontal_area

func _calculate_suspension(delta: float) -> void:
	"""Calculate suspension compression and forces for each wheel"""
	var gravity_vector: Vector3 = Vector3.UP * PhysicsSettings.gravity
	var total_weight: float = vehicle_mass * PhysicsSettings.gravity
	
	# Estimate weight distribution (50/50 default)
	var weight_per_axle: float = total_weight * 0.5
	var weight_per_wheel: float = weight_per_axle * 0.5
	
	# Adjust for acceleration/deceleration load transfer
	var longitudinal_load_transfer: float = _linear_acceleration * vehicle_mass * center_of_mass_offset.y / wheel_base
	
	# Adjust for cornering lateral load transfer
	var lateral_load_transfer: float = _lateral_acceleration * vehicle_mass * center_of_mass_offset.y / track_width
	
	for i in range(4):
		var normal_force: float = weight_per_wheel
		var compression: float = 0.0
		
		# Load transfer calculations
		if i == WHEEL_FRONT_LEFT or i == WHEEL_FRONT_RIGHT:
			normal_force += longitudinal_load_transfer * 0.5
		else:
			normal_force -= longitudinal_load_transfer * 0.5
		
		# Add aerodynamic downforce
		normal_force += _downforce_force * 0.25
		
		# Clamp normal force
		normal_force = max(normal_force, 0.0)
		
		# Calculate compression based on normal force
		compression = normal_force / suspension_stiffness
		compression = clamp(compression, 0.0, suspension_compression_limit)
		
		_suspension_compression[i] = compression
		_wheel_states[i]["suspension_compression"] = compression
		_wheel_states[i]["normal_force"] = normal_force

func _apply_wheel_forces(delta: float, driving_torque: float, braking_force: float) -> void:
	"""Apply calculated wheel forces to vehicle body"""
	var wheel_positions: Array[Vector3] = _get_wheel_local_positions()
	
	for i in range(4):
		var wheel_position: Vector3 = wheel_positions[i]
		var wheel_state: Dictionary = _wheel_states[i]
		
		# Convert local position to world position
		var world_position: Vector3 = global_transform * wheel_position
		
		# Calculate wheel ground contact point
		var ground_position: Vector3 = world_position - Vector3.DOWN * (spring_rest_length - wheel_state["suspension_compression"])
		
		# Apply driving force
		if wheel_state["driving_force"] > 0.0:
			var drive_direction: Vector3 = global_transform.basis.z.normalized()
			apply_force(-drive_direction * wheel_state["driving_force"], wheel_position)
		
		# Apply braking force
		if wheel_state["braking_force"] > 0.0:
			var brake_direction: Vector3 = global_transform.basis.z.normalized()
			apply_force(-brake_direction * wheel_state["braking_force"], wheel_position)
		
		# Apply suspension force
		var suspension_force: float = -_suspension_compression[i] * suspension_stiffness
		var damping_force: float = -_suspension_velocity[i] * suspension_damping
		apply_force(Vector3.UP * (suspension_force + damping_force), wheel_position)

func _handle_traction_control(delta: float) -> void:
	"""Handle traction control system"""
	if not traction_control_enabled:
		return
	
	var worst_slip_ratio: float = 0.0
	for i in range(4):
		if _wheel_states[i]["is_in_contact"]:
			worst_slip_ratio = max(worst_slip_ratio, abs(_wheel_states[i]["slip_ratio"]))
	
	if worst_slip_ratio > TRACTION_CONTROL_THRESHOLD:
		# Reduce throttle when traction loss detected
		_throttle_smoothed = _throttle_smoothed * 0.8
		_is_skidding = true
		emit_signal("skidding", true)
	else:
		_is_skidding = false
		emit_signal("skidding", false)

func _handle_abs(delta: float) -> void:
	"""Handle Anti-lock Braking System"""
	if not abs_enabled or _brake_smoothed < 0.1:
		return
	
	for i in range(4):
		var wheel_state: Dictionary = _wheel_states[i]
		if wheel_state["is_in_contact"] and wheel_state["braking_force"] > 0.0:
			if abs(wheel_state["slip_ratio"]) > ABS_THRESHOLD:
				# Modulate brake pressure
				_brake_smoothed *= 0.7

func _check_collisions(delta: float) -> void:
	"""Check for collision events"""
	if colliding:
		var now: float = Time.get_ticks_msec() / 1000.0
		if now - _last_collision_time > 0.5:  # Debounce collisions
			_last_collision_time = now
			
			var collision_info: Dictionary = {
				"collided_with": collision_get_collider(),
				"collision_normal": collision_get_normal(),
				"impact_velocity": linear_velocity.dot(collision_get_normal()),
				"time": now
			}
			
			emit_signal("collision_detected", collision_info)

# ============================================================================
# VEHICLE METRICS
# ============================================================================
func _calculate_vehicle_metrics(delta: float) -> void:
	"""Calculate and update current vehicle metrics"""
	# Current speed (km/h)
	_current_speed = linear_velocity.length() * 3.6
	
	# Engine RPM based on gear ratio
	var wheel_angular_velocity: float = _current_speed * 1000.0 / (2.0 * PI * 0.3)  # Approximate wheel radius
	var gear_ratio: float = gear_ratios[_current_gear - 1] if _current_gear > 0 else reverse_ratio
	var total_ratio: float = gear_ratio * final_drive_ratio
	_current_rpm = wheel_angular_velocity * total_ratio
	
	# Ensure RPM stays within bounds
	_current_rpm = clamp(_current_rpm, engine_idle_rpm, engine_max_rpm)

func _emit_signals() -> void:
	"""Emit property change signals"""
	emit_signal("speed_changed", _current_speed)
	emit_signal("rpm_changed", _current_rpm)
	emit_signal("throttle_applied", _throttle_smoothed)
	emit_signal("brake_applied", _brake_smoothed)
	emit_signal("steering_angle_changed", _steering_smoothed * MAX_STEERING_ANGLE)

# ============================================================================
# HELPER METHODS
# ============================================================================
func _update_input_smoothing(delta: float) -> void:
	"""Smooth input values for more realistic response"""
	_throttle_smoothed = lerp(_throttle_smoothed, _throttle_input, delta * 5.0)
	_brake_smoothed = lerp(_brake_smoothed, _brake_input, delta * 5.0)
	_steering_smoothed = lerp(_steering_smoothed, _steering_input, delta * STEERING_SPEED)

func _handle_clutch(delta: float) -> void:
	"""Handle clutch engagement/disengagement"""
	if _shifting:
		# Auto-clutch during shifts
		_clutch_displacement = 1.0 - _gear_shift_progress
	else:
		# Manual clutch input
		_clutch_displacement = lerp(_clutch_displacement, InputManager.get_axis("clutch_up", "clutch_down"), delta * 10.0)

func _get_wheel_local_positions() -> Array[Vector3]:
	"""Get local positions of all four wheels"""
	var half_track: float = track_width * 0.5
	var half_wheelbase: float = wheel_base * 0.5
	
	return [
		Vector3(half_track, -center_of_mass_offset.y, -half_wheelbase),      # Front Left
		Vector3(-half_track, -center_of_mass_offset.y, -half_wheelbase),     # Front Right
		Vector3(half_track, -center_of_mass_offset.y, half_wheelbase),       # Rear Left
		Vector3(-half_track, -center_of_mass_offset.y, half_wheelbase)       # Rear Right
	]

func get_linear_acceleration() -> float:
	"""Get current longitudinal acceleration"""
	return linear_velocity.x / _get_physics_delta()

func get_lateral_acceleration() -> float:
	"""Get current lateral acceleration"""
	return linear_velocity.y / _get_physics_delta()

func _get_physics_delta() -> float:
	"""Get physics timestep"""
	return PhysicsSettings.physics_tick_rate / 60.0

# ============================================================================
# PUBLIC API
# ============================================================================
func start_engine() -> void:
	"""Start the engine"""
	_engine_on = true
	_current_rpm = engine_idle_rpm

func stop_engine() -> void:
	"""Stop the engine"""
	_engine_on = false
	_current_rpm = 0.0
	emit_signal("engine_stalled")

func set_gear(gear: int) -> void:
	"""Set gear directly (for AI or testing)"""
	if gear < -1 or gear > GEAR_COUNT:
		return
	
	_current_gear = gear
	_gear_shift_target = gear

func get_current_speed() -> float:
	"""Get current vehicle speed in km/h"""
	return _current_speed

func get_current_rpm() -> float:
	"""Get current engine RPM"""
	return _current_rpm

func get_current_gear() -> int:
	"""Get current gear"""
	return _current_gear

func is_shifting() -> bool:
	"""Check if currently shifting gears"""
	return _shifting

func is_skidding() -> bool:
	"""Check if vehicle is skidding"""
	return _is_skidding

func reset() -> void:
	"""Reset vehicle to initial state"""
	_reset_vehicle()

func _set_gravity(value: float) -> void:
	gravity = value
	update_physics()

func _set_physics_tick_rate(value: int) -> void:
	physics_tick_rate = value
	update_physics()

func _set_max_substeps(value: int) -> void:
	max_substeps = value
	update_physics()

func _set_time_scale(value: float) -> void:
	time_scale = value
	update_physics()

func _set_default_vehicle_mass(value: float) -> void:
	default_vehicle_mass = value
	if vehicle_mass == default_vehicle_mass:
		vehicle_mass = value

func _set_default_wheel_friction(value: float) -> void:
	default_wheel_friction = value
	tire_friction = value

func update_physics() -> void:
	"""Update physics settings"""
	pass

</file>