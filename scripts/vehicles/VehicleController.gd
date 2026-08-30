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
@export var vehicle_mass: float = 1500.0: set = _set_vehicle_mass
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
@export var launch_control_enabled: bool = false

@export_group("Tire & Suspension")
@export var tire_friction_coefficient: float = 1.2
@export var suspension_travel: float = 0.15
@export var spring_rate: float = 45000.0
@export var damping_compression: float = 15000.0
@export var damping_rebound: float = 8000.0
@export var anti_roll_bar_stiffness: float = 5000.0

@export_group("Drift & Handling")
@export var drift_threshold: float = 0.75
@export var drift_recovery_rate: float = 0.1
@export var grip_loss_factor: float = 0.3
@export var grip_recovery_rate: float = 0.2

# ============================================================================
# INTERNAL STATE
# ============================================================================

var current_speed: float = 0.0
var current_rpm: float = 0.0
var current_gear: int = 0
var target_gear: int = 0
var is_in_reverse: bool = false
var wheels_on_ground: Array[bool] = [true, true, true, true]
var wheel_slip_ratios: Array[float] = [0.0, 0.0, 0.0, 0.0]
var drift_angle: float = 0.0
var lateral_velocity: float = 0.0
var vertical_velocity: float = 0.0
var suspension_compression: Array[float] = [0.0, 0.0, 0.0, 0.0]
var wheel_torques: Array[float] = [0.0, 0.0, 0.0, 0.0]

# Input states (normalized -1 to 1)
var throttle_input: float = 0.0
var brake_input: float = 0.0
var steering_input: float = 0.0
var handbrake_input: float = 0.0

# Powertrain state
var engine_braking: bool = false
var clutch_disengaged: bool = false
var last_shift_time: float = 0.0
var shift_progress: float = 0.0
var is_shifting: bool = false
var power_delivery: float = 1.0

# Drift state
var drift_mode: bool = false
var drift_score: float = 0.0
var drift_multiplier: float = 1.0

# Cache values
var _wheel_radius: float = 0.33
var _tire_circumference: float = 2.0 * PI * _wheel_radius
var _gear_count: int = 7
var _torque_curve_cache: Dictionary = {}

# ============================================================================
# LIFECYCLE
# ============================================================================

func _ready() -> void:
	_init_torque_curve()
	_setup_physics_materials()
	_apply_initial_transform()

func _physics_process(delta: float) -> void:
	if delta < 0.0: return
	
	_update_input_states()
	_handle_gear_shifting(delta)
	_calculate_engine_rpm(delta)
	_apply_power_to_wheels(delta)
	_update_drift_state(delta)
	_solve_suspension_and_contact(delta)
	_apply_vehicle_forces(delta)
	_move_vehicle(delta)

func _input(event: InputEvent) -> void:
	if event is InputEventKey:
		if event.pressed:
			_process_keyboard_input(event)

# ============================================================================
# SETUP & INITIALIZATION
# ============================================================================

func _init_torque_curve() -> void:
	"""Initialize torque curve from curve points if not provided"""
	if torque_curve_points.is_empty():
		torque_curve_points = [
			Vector2(engine_min_rpm / engine_max_rpm, 0.3),
			Vector2(0.2, 0.5),
			Vector2(0.4, 0.8),
			Vector2(0.5, 1.0),
			Vector2(0.6, 0.95),
			Vector2(0.75, 0.9),
			Vector2(0.9, 0.85),
			Vector2(1.0, 0.7)
		]
	_generate_torque_cache()

func _generate_torque_cache() -> void:
	"""Generate lookup cache for torque curve"""
	_torque_curve_cache.clear()
	var step = 0.01
	for i in range(int(engine_max_rpm)):
		var rpm_normalized = float(i) / engine_max_rpm
		var torque_factor = _get_torque_from_curve(rpm_normalized)
		_torque_curve_cache[i] = torque_factor * max_torque

func _setup_physics_materials() -> void:
	"""Setup physics materials for vehicle surfaces"""
	var surface_material = PhysicsServer3D.material_create()
	surface_material.set_friction(tire_friction_coefficient)
	surface_material.set_bounce(0.0)
	surface_material.set_electricity_conductivity(0.0)
	surface_material.set_thermal_conductivity(0.0)
	
	# Apply to vehicle body
	if get_node_or_null("/root"):
		pass # Material setup would be done through Godot's material system

func _apply_initial_transform() -> void:
	"""Apply initial vehicle transform and position"""
	position.y = ground_clearance + _wheel_radius
	rotate_x(-PI / 2) # Orient for forward movement in Z

# ============================================================================
# INPUT HANDLING
# ============================================================================

func _update_input_states() -> void:
	"""Read and normalize input from InputManager"""
	throttle_input = InputManager.get_action_value("throttle")
	brake_input = InputManager.get_action_value("brake")
	steering_input = InputManager.get_action_value("steer")
	handbrake_input = InputManager.get_action_value("handbrake")
	
	# Clamp inputs to valid range
	throttle_input = clamp(throttle_input, 0.0, 1.0)
	brake_input = clamp(brake_input, 0.0, 1.0)
	steering_input = clamp(steering_input, -1.0, 1.0)
	handbrake_input = clamp(handbrake_input, 0.0, 1.0)
	
	# Auto-downshift when braking
	if brake_input > 0.5 and current_gear > 0:
		target_gear = _calculate_optimal_gear_for_speed(current_speed, true)

func _process_keyboard_input(event: InputEventKey) -> void:
	"""Process keyboard-specific actions"""
	if event.keycode == KEY_Z or event.keycode == KEY_COMMA:
		# Downshift
		request_gear_change(current_gear - 1)
	elif event.keycode == KEY_X or event.keycode == KEY_PERIOD:
		# Upshift
		request_gear_change(current_gear + 1)
	elif event.keycode == KEY_SPACE:
		# Emergency stop
		emergency_brake()

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================

func request_gear_change(new_gear: int) -> void:
	"""Request a gear change with validation"""
	new_gear = clamp(new_gear, 0, _gear_count - 1)
	
	if new_gear != current_gear and !is_shifting:
		target_gear = new_gear
		is_shifting = true
		shift_progress = 0.0
		last_shift_time = Time.get_ticks_msec() / 1000.0
		clutch_disengaged = true
		
		# Check for reverse
		if new_gear == 0:
			is_in_reverse = true
		else:
			is_in_reverse = false

func handle_manual_shift(direction: int) -> void:
	"""Manual up/down shift triggered by player"""
	var new_gear = current_gear + direction
	request_gear_change(new_gear)

func _handle_gear_shifting(delta: float) -> void:
	"""Handle automatic gear shifting and shift execution"""
	if is_shifting:
		shift_progress += delta * 3.0 # Shift completes in ~0.33s
		
		if shift_progress >= 1.0:
			_complete_gear_shift()
		return
	
	# Automatic upshift logic
	if throttle_input > 0.8 and current_gear < _gear_count - 1:
		if current_rpm >= engine_max_rpm * 0.95:
			request_gear_change(current_gear + 1)
	
	# Automatic downshift on deceleration
	elif throttle_input < 0.2 and current_gear > 0:
		if current_rpm <= engine_min_rpm * 1.2:
			request_gear_change(current_gear - 1)
		
		# Also downshift if about to stall
		elif current_rpm <= engine_min_rpm and current_speed > 5.0:
			request_gear_change(current_gear - 1)

func _complete_gear_shift() -> void:
	"""Complete the gear shift sequence"""
	current_gear = target_gear
	is_shifting = false
	clutch_disengaged = false
	shift_progress = 0.0
	power_delivery = 0.0
	
	# Rev match if enabled
	if rev_matching_enabled and current_gear != target_gear:
		var target_rpm = _calculate_target_rpm_for_gear(target_gear)
		_current_rpm = lerp(_current_rpm, target_rpm, 0.5)
	
	# Emit signal
	emit_signal("gear_changed", current_gear)

func _calculate_target_rpm_for_gear(gear: int) -> float:
	"""Calculate what RPM should be after gear change"""
	if gear == 0:
		return idle_rpm
	
	var wheel_speed = abs(current_speed / (_wheel_radius * gear_ratios[gear]))
	var engine_rpm = wheel_speed * gear_ratios[gear] * final_drive_ratio
	
	return clamp(engine_rpm, engine_min_rpm, engine_max_rpm)

func _calculate_optimal_gear_for_speed(speed: float, force_downshift: bool = false) -> int:
	"""Calculate optimal gear based on current speed"""
	if speed < 2.0:
		return 0
	
	for gear in range(_gear_count):
		var max_speed_for_gear = _get_max_speed_for_gear(gear)
		if speed <= max_speed_for_gear:
			if gear > current_gear and !force_downshift:
				return gear + 1
			return gear
	
	return _gear_count - 1

func _get_max_speed_for_gear(gear: int) -> float:
	"""Calculate maximum speed possible in given gear"""
	if gear == 0:
		return 10.0
	
	var max_wheel_rps = engine_max_rpm / (gear_ratios[gear] * final_drive_ratio)
	return max_wheel_rps * _wheel_radius

# ============================================================================
# ENGINE & POWERTRAIN
# ============================================================================

func _calculate_engine_rpm(delta: float) -> void:
	"""Calculate current engine RPM based on wheel speed and gear"""
	if current_gear == 0:
		current_rpm = idle_rpm
		return
	
	var wheel_speed = abs(current_speed)
	var wheel_rps = wheel_speed / _tire_circumference
	
	if is_in_reverse:
		wheel_rps *= -1
	
	# Engine RPM = wheel RPM * gear ratio * final drive
	current_rpm = abs(wheel_rps * gear_ratios[current_gear] * final_drive_ratio)
	
	# Add idle boost when clutch disengaged
	if clutch_disengaged and current_rpm < idle_rpm:
		current_rpm = lerp(current_rpm, idle_rpm, delta * 10.0)
	
	# Limit RPM
	current_rpm = clamp(current_rpm, engine_min_rpm, engine_max_rpm)
	
	emit_signal("rpm_changed", current_rpm)

func _get_engine_torque() -> float:
	"""Get current engine torque based on RPM"""
	var rpm_normalized = current_rpm / engine_max_rpm
	var torque_factor = _get_torque_from_curve(rpm_normalized)
	return torque_factor * max_torque

func _get_torque_from_curve(normalized_rpm: float) -> float:
	"""Look up torque factor from torque curve"""
	if torque_curve_points.is_empty():
		return 1.0
	
	# Linear interpolation through curve points
	for i in range(torque_curve_points.size() - 1):
		var point_a = torque_curve_points[i]
		var point_b = torque_curve_points[i + 1]
		
		if normalized_rpm >= point_a.x and normalized_rpm <= point_b.x:
			var t = (normalized_rpm - point_a.x) / (point_b.x - point_a.x)
			return point_a.y + t * (point_b.y - point_a.y)
	
	return torque_curve_points.back().y

func _apply_power_to_wheels(delta: float) -> void:
	"""Apply calculated power to driven wheels"""
	var engine_torque = _get_engine_torque()
	var total_gear_ratio = gear_ratios[current_gear] * final_drive_ratio
	var wheel_torque = engine_torque * total_gear_ratio * transmission_efficiency
	
	# Apply throttle
	var drive_torque = wheel_torque * throttle_input * power_delivery
	
	# Engine braking when not accelerating
	if throttle_input < 0.1 and current_speed > 5.0:
		engine_braking = true
		drive_torque *= -0.3 # Engine resistance
	
	# Distribute torque to wheels (RWD configuration)
	wheel_torques = [0.0, drive_torque, 0.0, drive_torque] # Rear wheels
	
	# Handbrake affects rear wheels only
	if handbrake_input > 0.1:
		wheel_torques[2] -= drive_torque * handbrake_input * 2.0
		wheel_torques[3] -= drive_torque * handbrake_input * 2.0
	
	# Apply torque to velocity
	if not clutch_disengaged:
		var acceleration = (drive_torque * 2.0) / (vehicle_mass * _wheel_radius)
		velocity.z += acceleration * delta
		
		# Air resistance
		var air_resistance = 0.5 * _physics.drag_coefficient * frontal_area * current_speed * current_speed / vehicle_mass
		velocity.z -= air_resistance * delta

# ============================================================================
# DRIFT MECHANICS
# ============================================================================

func _update_drift_state(delta: float) -> void:
	"""Update drift mechanics and calculate grip loss"""
	lateral_velocity = velocity.x
	
	# Calculate drift angle (difference between heading and velocity)
	var heading = global_transform.basis.z
	var velocity_direction = velocity.normalized()
	
	if velocity.length_squared() > 0.1:
		var dot_product = heading.dot(velocity_direction)
		var drift_calc = acos(clamp(dot_product, -1.0, 1.0))
		drift_angle = drift_calc * sign(lateral_velocity)
	
	# Detect drift entry
	var lateral_acceleration = abs(lateral_velocity) / (delta + 0.001)
	var throttle_oversteer = throttle_input * 0.5
	var turn_oversteer = abs(steering_input) * 0.5
	
	if (lateral_acceleration > drift_threshold or 
	    (throttle_oversteer > 0.7 and turn_oversteer > 0.5)) and !drift_mode:
		drift_mode = true
		emit_signal("drift_started")
	
	# Update drift score
	if drift_mode:
		var drift_intensity = min(abs(drift_angle) / 0.5, 1.0)
		var drift_duration = (Time.get_ticks_msec() / 1000.0) - last_shift_time
		
		if drift_intensity > 0.5:
			drift_score += drift_intensity * drift_intensity * 10.0 * delta
			drift_multiplier = 1.0 + drift_score * 0.1
			
			# Wheel slip increases during drift
			for i in range(wheel_slip_ratios.size()):
				wheel_slip_ratios[i] = lerp(wheel_slip_ratios[i], drift_intensity, delta * 2.0)
				emit_signal("wheel_slip", i, wheel_slip_ratios[i])
		else:
			drift_score = max(0.0, drift_score - 5.0 * delta)
			drift_multiplier = 1.0 + drift_score * 0.1
	
	# Drift recovery
	if not drift_mode or (not handbrake_input and throttle_input < 0.3):
		drift_angle = lerp(drift_angle, 0.0, drift_recovery_rate * delta)
		if abs(drift_angle) < 0.05:
			drift_mode = false
			emit_signal("drift_ended")

# ============================================================================
# SUSPENSION & CONTACT SOLVING
# ============================================================================

func _solve_suspension_and_contact(delta: float) -> void:
	"""Solve suspension compression and wheel contact"""
	var gravity_force = _physics.gravity * vehicle_mass
	
	# Simple spring-damper model for each wheel
	for i in range(4):
		var wheel_position = _get_wheel_local_position(i)
		var wheel_height = wheel_position.y
		
		# Calculate suspension compression
		var desired_height = ground_clearance + _wheel_radius
		var compression = (desired_height - wheel_height) / suspension_travel
		
		compression = clamp(compression, 0.0, 1.0)
		suspension_compression[i] = compression
		
		# Spring force
		var spring_force = spring_rate * compression * delta
		
		# Damping force (simplified)
		var damping_force = damping_compression * vertical_velocity * delta
		
		# Apply force upward
		var force_y = spring_force - damping_force
		velocity.y += force_y / vehicle_mass * delta

func _get_wheel_local_position(wheel_index: int) -> Vector3:
	"""Get local position of wheel"""
	var half_track = track_width / 2.0
	var half_wheelbase = wheel_base / 2.0
	
	match wheel_index:
		0: # Front Left
			return Vector3(-half_track, 0, half_wheelbase)
		1: # Front Right
			return Vector3(half_track, 0, half_wheelbase)
		2: # Rear Left
			return Vector3(-half_track, 0, -half_wheelbase)
		3: # Rear Right
			return Vector3(half_track, 0, -half_wheelbase)
	
	return Vector3.ZERO

# ============================================================================
# VEHICLE MOVEMENT
# ============================================================================

func _apply_vehicle_forces(delta: float) -> void:
	"""Apply aerodynamic and traction forces"""
	# Aerodynamic drag
	var air_density = 1.225
	var drag_force = 0.5 * air_density * drag_coefficient * frontal_area * current_speed * current_speed
	
	# Apply drag opposite to velocity
	if current_speed > 0:
		var drag_vector = velocity.normalized() * -drag_force
		velocity += drag_vector * delta / vehicle_mass
	
	# Steering effect
	if current_speed > 1.0:
		var steering_effect = steering_input * 5.0 * (current_speed / 10.0)
		velocity.x += steering_effect * delta
	
	# Friction
	var friction_force = -velocity * 0.1 * delta
	velocity += friction_force

func _move_vehicle(delta: float) -> void:
	"""Move vehicle using accumulated velocity"""
	move_and_slide()
	
	# Update speed magnitude
	current_speed = velocity.length()
	
	# Clamp horizontal movement
	velocity.y = 0.0
	
	emit_signal("speed_changed", current_speed)

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================

func emergency_brake() -> void:
	"""Apply maximum braking force"""
	var brake_force = 15.0 * vehicle_mass
	velocity -= velocity.normalized() * brake_force * 0.5
	
func reset_vehicle() -> void:
	"""Reset vehicle to starting state"""
	velocity = Vector3.ZERO
	current_speed = 0.0
	current_rpm = idle_rpm
	current_gear = 0
	target_gear = 0
	is_in_reverse = false
	drift_mode = false
	drift_score = 0.0
	wheel_slip_ratios.fill(0.0)

func get_wheel_contacts() -> Array[bool]:
	"""Return array indicating which wheels are touching ground"""
	return wheels_on_ground.duplicate()

func get_vehicle_speed_kmh() -> float:
	"""Convert speed to kilometers per hour"""
	return current_speed * 3.6

func get_vehicle_speed_mph() -> float:
	"""Convert speed to miles per hour"""
	return current_speed * 2.237

func _set_vehicle_mass(value: float) -> void:
	vehicle_mass = value
	# Update physics mass if we have a rigid body
	if has_node("../rigid_body"):
		var rb = get_node("../rigid_body")
		rb.mass = vehicle_mass

# ============================================================================
# RACE DATA COLLECTION
# ============================================================================

func get_race_statistics() -> Dictionary:
	"""Collect statistics for race replay/data logging"""
	return {
		"max_speed": current_speed,
		"avg_rpm": current_rpm,
		"current_gear": current_gear,
		"drift_duration": drift_score,
		"total_distance": global_position.length(),
		"wheels_on_ground": wheels_on_ground
	}

func record_lap_time(lap_number: int, time_ms: int) -> void:
	"""Record lap timing data"""
	pass # Would connect to GameManager for lap tracking

func get_telemetry_snapshot() -> Dictionary:
	"""Get current telemetry snapshot"""
	return {
		"timestamp": Time.get_unix_time_from_system(),
		"speed": current_speed,
		"rpm": current_rpm,
		"gear": current_gear,
		"throttle": throttle_input,
		"brake": brake_input,
		"steering": steering_input,
		"handbrake": handbrake_input,
		"lateral_velocity": lateral_velocity,
		"drift_angle": drift_angle,
		"suspension": suspension_compression,
		"wheel_slip": wheel_slip_ratios
	}