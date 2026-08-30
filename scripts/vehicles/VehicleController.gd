extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulation
## Handles throttle, brake, steering, wheel forces, gear shifting, and vehicle dynamics
## Uses PhysicsSettings singleton for all physics constants
## Copyright 2026 Thalamus Racing Simulator Project

signal speed_changed(new_speed: float)
signal rpm_changed(new_rpm: float)
signal gear_changed(old_gear: int, new_gear: int)
signal drift_started(drift_factor: float)
signal drift_ended()
signal collision_impact(impact_force: Vector3, impact_point: Vector3)
signal wheel_slip(wheel_index: int, slip_ratio: float)

# ============================================================================
# PHYSICS SETTINGS REFERENCE
# ============================================================================
var physics_settings: PhysicsSettings = null

# ============================================================================
# VEHICLE CONFIGURATION (Export Groups)
# ============================================================================
@export_group("Vehicle Configuration")
@export var vehicle_mass: float = 1500.0: set = _set_vehicle_mass
@export var center_of_mass_offset: Vector3 = Vector3(0.0, 0.5, 0.0)
@export var track_width: float = 1.6
@export var wheelbase: float = 2.8
@export var ride_height: float = 0.35
@export var drag_coeffent: float = 0.30
@export var frontal_area: float = 2.2
@export var downforce_coefficient: float = 0.5

@export_group("Suspension Geometry")
@export var suspension_stiffness: float = 35000.0
@export var suspension_damping: float = 3000.0
@export var suspension_compression_limit: float = 0.25
@export var suspension_rebound_limit: float = 0.35
@export var unsprung_mass_per_wheel: float = 45.0

@export_group("Aerodynamics")
@export var lift_coefficient: float = 0.05
@export var side_force_coefficient: float = 0.02
@export var aerodynamic_center_x: float = 0.75

@export_group("Tire Properties")
@export var tire_radius: float = 0.33
@export var tire_width: float = 0.26
@export var tire_friction_static: float = 1.2
@export var tire_friction_dynamic: float = 0.9
@export var tire_vertical_stiffness: float = 50000.0

# ============================================================================
# POWERTRAIN PARAMETERS (Export Groups)
# ============================================================================
@export_group("Engine Specifications")
@export var engine_max_torque: float = 500.0  # Nm
@export var engine_max_power: float = 250.0   # kW
@export var idle_rpm: float = 800.0
@export var redline_rpm: float = 7500.0
@export var torque_curve_points: Array[Vector2] = [
	Vector2(0, 0.0), Vector2(1500, 0.85), 
	Vector2(3000, 1.0), Vector2(4500, 0.95), 
	Vector2(6000, 0.80), Vector2(7500, 0.65)
]

@export_group("Transmission")
@export var transmission_type: String = "manual"
@export var final_drive_ratio: float = 3.5
@export var gear_ratios: Dictionary = {
	1: 3.8, 2: 2.2, 3: 1.5, 
	4: 1.1, 5: 0.9, 6: 0.75, R: -4.2
}
@export var shift_up_rpm_threshold: float = 6800.0
@export var shift_down_rpm_threshold: float = 2500.0

@export_group("Clutch Settings")
@export var clutch_engagement_rpm: float = 1000.0
@export var clutch_slip_factor: float = 0.15
@export var clutch_pressure_plate_force: float = 5000.0

# ============================================================================
# BRAKING SYSTEM
# ============================================================================
@export_group("Braking System")
@export var brake_force_per_piston: float = 8000.0
@export var brake_caliper_pistons: int = 4
@export var brake_disc_diameter: float = 0.32
@export var brake_pad_friction: float = 0.4
@export var front_brake_bias: float = 0.60
@export var rear_brake_bias: float = 0.40
@export var abs_enabled: bool = true
@export var brake_balance_adjustment: float = 0.0

@export_group("Parking Brake")
@export var parking_brake_force: float = 3000.0
@export var parking_brake_applied: bool = false

# ============================================================================
# DRIFT & HANDLING TUNING
# ============================================================================
@export_group("Drift Tuning")
@export var drift_friction_multiplier: float = 0.6
@export var drift_recovery_rate: float = 0.15
@export var oversteer_threshold: float = 0.75
@export var understeer_threshold: float = 0.70
@export var drift_angle_tolerance: float = 0.35

@export_group("Steering Tuning")
@export var max_steering_angle: float = 0.65  # radians (~37 degrees)
@export var steering_speed: float = 2.5       # radians per second
@export var steering_ratio: float = 15.0
@export var ackermann_geometry: bool = true

# ============================================================================
# INTERNAL STATE VARIABLES
# ============================================================================
var _current_speed: float = 0.0
var _current_rpm: float = idle_rpm
var _current_gear: int = 1
var _gear_in_neutral: bool = false
var _steering_angle: float = 0.0
var _target_steering_angle: float = 0.0
var _throttle_input: float = 0.0
var _brake_input: float = 0.0
var _clutch_input: float = 0.0
var _drift_state: bool = false
var _drift_factor: float = 0.0
var _wheel_slip_front_left: float = 0.0
var _wheel_slip_front_right: float = 0.0
var _wheel_slip_rear_left: float = 0.0
var _wheel_slip_rear_right: float = 0.0

# Wheel positions (relative to vehicle center)
var _wheel_positions: Dictionary = {}
var _wheel_velocity: Dictionary = {}
var _wheel_rotation: Dictionary = {}
var _suspension_compression: Dictionary = {}

# Aerodynamic forces
var _aero_force_drag: float = 0.0
var _aero_force_downforce: float = 0.0
var _aero_force_side: float = 0.0

# Collision history for impact tracking
var _collision_history: Array[Dictionary] = []
var _last_collision_time: float = 0.0

# ============================================================================
# INITIALIZATION
# ============================================================================
func _ready() -> void:
	physics_settings = PhysicsSettings
	
	# Initialize wheel positions based on geometry
	_initialize_wheel_positions()
	
	# Set initial gear
	_current_gear = 1
	_gear_in_neutral = false
	
	# Connect to GameManager signals if available
	if GameManager.has_signal("race_started"):
		GameManager.race_started.connect(_on_race_started)
	if GameManager.has_signal("vehicle_destroyed"):
		GameManager.vehicle_destroyed.connect(_on_vehicle_destroyed)
	
	# Apply default mass
	apply_mass(vehicle_mass)
	
	print("VehicleController initialized successfully")

func _initialize_wheel_positions() -> void:
	var half_track = track_width * 0.5
	var half_wb = wheelbase * 0.5
	
	# Front wheels
	_wheel_positions["front_left"] = Vector3(-half_track, -ride_height, -half_wb)
	_wheel_positions["front_right"] = Vector3(half_track, -ride_height, -half_wb)
	
	# Rear wheels
	_wheel_positions["rear_left"] = Vector3(-half_track, -ride_height, half_wb)
	_wheel_positions["rear_right"] = Vector3(half_track, -ride_height, half_wb)
	
	# Initialize wheel states
	for wheel_key in _wheel_positions.keys():
		_wheel_velocity[wheel_key] = Vector3.ZERO
		_wheel_rotation[wheel_key] = 0.0
		_suspension_compression[wheel_key] = 0.0

# ============================================================================
# MAIN PHYSICS LOOP
# ============================================================================
func _physics_process(delta: float) -> void:
	# Get player input
	_get_player_input()
	
	# Calculate RPM based on gear and speed
	_update_engine_rpm(delta)
	
	# Apply throttle and torque
	_apply_throttle_and_torque(delta)
	
	# Apply braking forces
	_apply_brakes(delta)
	
	# Handle steering
	_handle_steering(delta)
	
	# Calculate aerodynamic forces
	_calculate_aerodynamics(delta)
	
	# Calculate wheel forces and apply to velocity
	_apply_wheel_forces(delta)
	
	# Update vehicle movement
	_move_vehicle(delta)
	
	# Check for drift conditions
	_check_drift_conditions()
	
	# Track collisions
	_track_collisions()
	
	# Emit signals for changed values
	_emit_vehicle_signals()

# ============================================================================
# INPUT HANDLING
# ============================================================================
func _get_player_input() -> void:
	var input_manager = get_node_or_null("/root/InputManager")
	
	if input_manager == null:
		return
	
	# Get throttle input (0.0 to 1.0)
	var throttle = input_manager.get_axis("throttle", "brake")
	if throttle > 0.0:
		_throttle_input = clamp(throttle, 0.0, 1.0)
	else:
		_throttle_input = 0.0
	
	# Get brake input (0.0 to 1.0)
	var brake = input_manager.get_axis("brake_reverse", "brake")
	if brake > 0.0:
		_brake_input = clamp(brake, 0.0, 1.0)
	else:
		_brake_input = 0.0
	
	# Get steering input (-1.0 to 1.0)
	var steer = input_manager.get_axis("steer_left", "steer_right")
	_target_steering_angle = steer * max_steering_angle
	
	# Get clutch input (for manual transmission)
	if transmission_type == "manual":
		_clutch_input = input_manager.get_axis("clutch", "clutch_release")
	else:
		_clutch_input = 1.0  # Automatic always engaged

# ============================================================================
# ENGINE RPM CALCULATION
# ============================================================================
func _update_engine_rpm(delta: float) -> void:
	if _gear_in_neutral or _current_gear == 0:
		# Engine idles when in neutral or reverse not selected
		_current_rpm = lerp(_current_rpm, idle_rpm, delta * 3.0)
		return
	
	# Calculate wheel speed from vehicle forward speed
	var wheel_circumference = PI * tire_radius * 2.0
	var wheel_rps = abs(_current_speed) / wheel_circumference
	
	# Total gear ratio = gear ratio * final drive ratio
	var gear_ratio = gear_ratios[_current_gear]
	if gear_ratio == null:
		gear_ratio = gear_ratios[1]
	
	var total_ratio = gear_ratio * final_drive_ratio
	
	# Engine RPM = wheel RPM * total ratio * 60 (convert to RPM)
	var target_rpm = wheel_rps * total_ratio * 60.0
	
	# Apply throttle influence on RPM climb rate
	var rpm_change_rate = 2000.0 * _throttle_input + 500.0
	
	if target_rpm > _current_rpm:
		_current_rpm = min(target_rpm, _current_rpm + rpm_change_rate * delta)
	elif target_rpm < _current_rpm:
		_current_rpm = max(target_rpm, _current_rpm - rpm_change_rate * delta * 2.0)
	
	# Clamp to idle and redline
	_current_rpm = clamp(_current_rpm, idle_rpm, redline_rpm)

# ============================================================================
# THROTTLE AND TORQUE APPLICATION
# ============================================================================
func _apply_throttle_and_torque(delta: float) -> void:
	if _gear_in_neutral or _current_gear == 0:
		return
	
	# Calculate engine torque based on RPM using torque curve
	var engine_torque = _calculate_engine_torque()
	
	# Apply clutch slip factor (simulates partial engagement)
	var effective_torque = engine_torque * _clutch_input
	
	# Calculate wheel torque through transmission
	var gear_ratio = gear_ratios[_current_gear]
	if gear_ratio == null:
		gear_ratio = gear_ratios[1]
	
	var total_ratio = gear_ratio * final_drive_ratio
	var wheel_torque = effective_torque * total_ratio
	
	# Distribute torque to driven wheels
	var drive_wheels: Array[String] = []
	if transmission_type == "fwd":
		drive_wheels = ["front_left", "front_right"]
	elif transmission_type == "rwd":
		drive_wheels = ["rear_left", "rear_right"]
	else:  # AWD
		drive_wheels = ["front_left", "front_right", "rear_left", "rear_right"]
	
	# Apply torque to each drive wheel
	for wheel in drive_wheels:
		var torque_per_wheel = wheel_torque / drive_wheels.size()
		
		# Convert torque to force at wheel contact patch
		var wheel_force = torque_per_wheel / tire_radius
		
		# Apply friction limit
		var normal_force = calculate_normal_force(wheel)
		var max_traction = normal_force * tire_friction_dynamic
		
		wheel_force = min(wheel_force, max_traction)
		
		# Apply force in vehicle forward direction
		var forward_dir = transform.basis.z.normalized() * -1.0
		add_force(forward_dir * wheel_force)
		
		# Calculate slip ratio
		var wheel_linear_velocity = wheel_force / unsprung_mass_per_wheel
		var slip_ratio = (wheel_linear_velocity - _current_speed) / max(abs(_current_speed), 1.0)
		_wheel_slip_[wheel] = clamp(slip_ratio, -1.0, 1.0)
		
		# Emit wheel slip signal occasionally
		if abs(slip_ratio) > 0.3:
			emit_signal("wheel_slip", wheel.split("_")[0], slip_ratio)

# ============================================================================
# BRAKE APPLICATION
# ============================================================================
func _apply_brakes(delta: float) -> void:
	if _brake_input <= 0.0:
		return
	
	# Calculate total brake force
	var caliper_force = brake_force_per_piston * brake_caliper_pistons
	var pad_contact_area = PI * (brake_disc_diameter * 0.5) ** 2
	var brake_force = caliper_force * brake_pad_friction * _brake_input
	
	# Apply brake bias
	var front_brake_total = brake_force * front_brake_bias * (1.0 + brake_balance_adjustment)
	var rear_brake_total = brake_force * rear_brake_bias * (1.0 - brake_balance_adjustment)
	
	# Distribute to individual wheels
	var front_wheels = ["front_left", "front_right"]
	var rear_wheels = ["rear_left", "rear_right"]
	
	var front_force_each = front_brake_total / 2.0
	var rear_force_each = rear_brake_total / 2.0
	
	# Apply brake force opposite to motion
	var velocity_magnitude = velocity.length()
	if velocity_magnitude > 0.1:
		var brake_direction = -velocity.normalized()
		
		# Front brakes
		for wheel in front_wheels:
			var wheel_force = brake_force_each
			var normal = calculate_normal_force(wheel)
			
			# ABS prevents wheel lockup
			if abs_enabled:
				var wheel_slip = _wheel_slip_[wheel]
				if abs(wheel_slip) > 0.2:
					wheel_force *= 0.7  # Reduce brake force if slipping
			
			add_force(brake_direction * wheel_force)
		
		# Rear brakes
		for wheel in rear_wheels:
			var wheel_force = rear_force_each
			var normal = calculate_normal_force(wheel)
			
			if abs_enabled:
				var wheel_slip = _wheel_slip_[wheel]
				if abs(wheel_slip) > 0.25:
					wheel_force *= 0.7
			
			add_force(brake_direction * wheel_force)

# ============================================================================
# STEERING HANDLING
# ============================================================================
func _handle_steering(delta: float) -> void:
	# Smooth steering transition
	_steering_angle = lerp(_steering_angle, _target_steering_angle, delta * steering_speed)
	
	# Apply Ackermann geometry for realistic steering angles
	if ackermann_geometry and _steering_angle != 0.0:
		var inner_outer_ratio = (wheelbase + track_width) / (wheelbase - track_width)
		if _steering_angle > 0:
			# Turning right - outer wheel is left, inner wheel is right
			_wheel_positions["front_left"] = Vector3(_wheel_positions["front_left"].x, _wheel_positions["front_left"].y, _wheel_positions["front_left"].z)
			_wheel_positions["front_right"] = Vector3(_wheel_positions["front_right"].x, _wheel_positions["front_right"].y, _wheel_positions["front_right"].z)
		else:
			# Turning left
			pass
	
	# Apply lateral force for steering
	if abs(_current_speed) > 1.0:
		var steering_force = _steering_angle * _current_speed * 50.0
		var lateral_dir = transform.basis.x.normalized()
		add_force(lateral_dir * steering_force)

# ============================================================================
# AERODYNAMICS CALCULATION
# ============================================================================
func _calculate_aerodynamics(delta: float) -> void:
	# Calculate air density (standard sea level)
	var air_density: float = 1.225
	
	# Speed squared for aerodynamic calculations
	var speed_squared = _current_speed * _current_speed
	
	# Drag force: Fd = 0.5 * rho * Cd * A * v^2
	_aero_force_drag = 0.5 * air_density * drag_coeffent * frontal_area * speed_squared
	
	# Downforce: similar formula but positive (pushes car down)
	_aero_force_downforce = 0.5 * air_density * downforce_coefficient * frontal_area * speed_squared
	
	# Side force (crosswind effect)
	var lateral_speed = velocity.x
	_aero_force_side = 0.5 * air_density * side_force_coefficient * frontal_area * lateral_speed * abs(_current_speed)
	
	# Apply forces
	var drag_force = -transform.basis.z.normalized() * _aero_force_drag
	var downforce_force = Vector3.UP * _aero_force_downforce
	var side_force = transform.basis.x.normalized() * _aero_force_side
	
	add_force(drag_force)
	add_force(downforce_force)
	add_force(side_force)

# ============================================================================
# WHEEL FORCE CALCULATION
# ============================================================================
func _apply_wheel_forces(delta: float) -> void:
	# Calculate normal force on each wheel (simplified suspension model)
	var gravity_force = vehicle_mass * physics_settings.gravity
	
	# Weight distribution (simplified 40/60 front/rear)
	var weight_on_front = gravity_force * 0.4
	var weight_on_rear = gravity_force * 0.6
	
	# Add aerodynamic downforce contribution
	var aero_contribution_front = _aero_force_downforce * 0.55
	var aero_contribution_rear = _aero_force_downforce * 0.45
	
	var total_front_weight = weight_on_front + aero_contribution_front
	var total_rear_weight = weight_on_rear + aero_contribution_rear
	
	# Per wheel normal force
	var front_normal = total_front_weight * 0.5
	var rear_normal = total_rear_weight * 0.5
	
	# Suspension compression effect (load transfer during acceleration/braking)
	var load_transfer = (engine_torque * final_drive_ratio / tire_radius) / (wheelbase * gravity_force) * 0.3
	front_normal -= load_transfer
	rear_normal += load_transfer
	
	# Store normal forces for traction calculation
	_normal_force["front_left"] = front_normal
	_normal_force["front_right"] = front_normal
	_normal_force["rear_left"] = rear_normal
	_normal_force["rear_right"] = rear_normal

func calculate_normal_force(wheel_key: String) -> float:
	return _normal_force.get(wheel_key, 3675.0)  # Default ~1500kg/4

# ============================================================================
# VEHICLE MOVEMENT
# ============================================================================
func _move_vehicle(delta: float) -> void:
	# Apply calculated forces to body
	# Forces already added via add_force() calls above
	
	# Update velocity
	velocity += _forces_accumulated * delta / vehicle_mass
	
	# Friction/drag (simple linear damping)
	var friction_coeff = 0.02
	velocity *= (1.0 - friction_coeff)
	
	# Ground constraint (keep y position reasonable)
	if position.y < 0.2:
		position.y = 0.2
		velocity.y = 0.0
	
	# Move character body
	move_and_slide()
	
	# Update speed from velocity magnitude
	var ground_speed = Vector2(velocity.x, velocity.z).length()
	_current_speed = ground_speed

# ============================================================================
# DRIFT DETECTION
# ============================================================================
func _check_drift_conditions() -> void:
	if abs(_current_speed) < 5.0:
		_drift_state = false
		return
	
	# Calculate angle between velocity vector and forward direction
	var forward = transform.basis.z.normalized()
	var velocity_vector = Vector2(velocity.x, velocity.z).normalized()
	var angle_diff = Vector2(forward.x, forward.z).angle_to(velocity_vector)
	
	# Drift threshold based on speed and steering
	var drift_threshold = drift_angle_tolerance * (1.0 + _steering_angle * 2.0)
	
	if abs(angle_diff) > drift_threshold and abs(_steering_angle) > 0.2:
		_drift_state = true
		_drift_factor = min(abs(angle_diff) / PI, 1.0)
		
		if not is_connected("drift_started", _on_drift_started):
			connect("drift_started", _on_drift_started)
		emit_signal("drift_started", _drift_factor)
	else:
		if _drift_state:
			_drift_state = false
			emit_signal("drift_ended")

func _on_drift_started(drift_factor: float) -> void:
	print("Drift started! Factor: ", drift_factor)

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================
func shift_gear(new_gear: int, manual_shift: bool = false) -> bool:
	if _gear_in_neutral:
		return false
	
	# Validate gear range
	if new_gear < 0 or new_gear > 6:
		return false
	
	# Manual vs automatic shift rules
	if manual_shift:
		# Check clutch engagement for manual
		if _clutch_input < 0.3 and transmission_type == "manual":
			return false  # Cannot shift without sufficient clutch release
	
	# Prevent excessive rapid shifting
	if Time.get_ticks_msec() - _last_shift_time < 200:
		return false
	
	var old_gear = _current_gear
	_current_gear = new_gear
	
	emit_signal("gear_changed", old_gear, new_gear)
	_last_shift_time = Time.get_ticks_msec()
	
	return true

func auto_shift() -> void:
	if _current_gear == 0:
		return
	
	# Upshift logic
	if _current_rpm >= shift_up_rpm_threshold and _current_gear < 6:
		shift_gear(_current_gear + 1, false)
	
	# Downshift logic
	elif _current_rpm <= shift_down_rpm_threshold and _current_gear > 1:
		shift_gear(_current_gear - 1, false)

# ============================================================================
# COLLISION HANDLING
# ============================================================================
func _track_collisions() -> void:
	# Simple collision detection using move_and_slide results
	if is_colliding():
		var col_info = get_collision()
		var impact_velocity = velocity
		
		# Calculate impact force
		var impact_force = impact_velocity.length() * vehicle_mass
		
		# Record collision
		if Time.get_ticks_msec() - _last_collision_time > 100:
			_collision_history.append({
				"time": Time.get_ticks_msec(),
				"force": impact_force,
				"direction": impact_velocity.normalized(),
				"point": col_info.position
			})
			
			_last_collision_time = Time.get_ticks_msec()
			
			# Limit history size
			if _collision_history.size() > 10:
				_collision_history.pop_front()
			
			emit_signal("collision_impact", impact_force, col_info.position)

# ============================================================================
# SIGNAL EMITTING
# ============================================================================
func _emit_vehicle_signals() -> void:
	if speed_changed.is_connected(speed_changed):
		emit_signal("speed_changed", _current_speed)
	
	if rpm_changed.is_connected(rpm_changed):
		emit_signal("rpm_changed", _current_rpm)

# ============================================================================
# UTILITY METHODS
# ============================================================================
func _calculate_engine_torque() -> float:
	# Interpolate torque from torque curve based on current RPM
	if torque_curve_points.is_empty():
		return engine_max_torque
	
	var rpm_normalized = (_current_rpm - idle_rpm) / (redline_rpm - idle_rpm)
	rpm_normalized = clamp(rpm_normalized, 0.0, 1.0)
	
	# Linear interpolation through torque curve points
	var torque = 0.0
	for i in range(torque_curve_points.size() - 1):
		var p1 = torque_curve_points[i]
		var p2 = torque_curve_points[i + 1]
		
		var t = (rpm_normalized - p1.x) / (p2.x - p1.x)
		t = clamp(t, 0.0, 1.0)
		
		torque = p1.y + t * (p2.y - p1.y)
		if t <= 1.0:
			break
	
	return engine_max_torque * torque

func reset_vehicle() -> void:
	_current_speed = 0.0
	_current_rpm = idle_rpm
	_current_gear = 1
	_steering_angle = 0.0
	_throttle_input = 0.0
	_brake_input = 0.0
	_drift_state = false
	_drift_factor = 0.0
	
	velocity = Vector3.ZERO
	position = Vector3.ZERO

func set_vehicle_mass(new_mass: float) -> void:
	vehicle_mass = new_mass
	apply_mass(new_mass)

func _set_vehicle_mass(value: float) -> void:
	vehicle_mass = value
	apply_mass(value)

# ============================================================================
# RACE EVENT HANDLERS
# ============================================================================
func _on_race_started(race_data: Dictionary) -> void:
	# Reset vehicle for race start
	reset_vehicle()
	
	# Auto-engagement for automatic transmission
	if transmission_type == "automatic":
		_clutch_input = 1.0

func _on_vehicle_destroyed(vehicle: Node) -> void:
	if vehicle == self:
		print("Vehicle destroyed")
		# Cleanup handlers
		disconnect("drift_started", _on_drift_started)

# ============================================================================
# SAVE/LOAD SUPPORT
# ============================================================================
func save_vehicle_state() -> Dictionary:
	return {
		"position": position,
		"rotation": rotation,
		"velocity": velocity,
		"speed": _current_speed,
		"rpm": _current_rpm,
		"gear": _current_gear,
		"throttle": _throttle_input,
		"brake": _brake_input,
		"steering": _steering_angle
	}

func load_vehicle_state(state: Dictionary) -> void:
	position = state.get("position", position)
	rotation = state.get("rotation", rotation)
	velocity = state.get("velocity", Vector3.ZERO)
	_current_speed = state.get("speed", 0.0)
	_current_rpm = state.get("rpm", idle_rpm)
	_current_gear = state.get("gear", 1)
	_throttle_input = state.get("throttle", 0.0)
	_brake_input = state.get("brake", 0.0)
	_steering_angle = state.get("steering", 0.0)
