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
var yaw_angle: float = 0.0  # Current yaw rotation
var pitch_angle: float = 0.0  # Pitch rotation
var roll_angle: float = 0.0  # Roll rotation

var engine_rpm: float = 0.0
var engine_max_rpm: float = 7000.0
var engine_min_rpm: float = 800.0
var torque_output: float = 0.0
var power_output: float = 0.0

var current_gear: int = 0  # 0 = neutral, 1-6 = gears
var target_gear: int = 0
var clutch_engaged: bool = true

var drift_enabled: bool = false
var drift_coefficient: float = 0.85  # Lower = more slippery
var grip_coefficient: float = 1.0
var tire_temperature: Vector4 = Vector4(0.0, 0.0, 0.0, 0.0)  # FL, FR, RL, RR temperatures

# ============================================================================
# WHEELS CONFIGURATION
# ============================================================================

@export_group("Wheel Configuration")
@export var track_width: float = 1.5  # Distance between left/right wheels
@export var wheelbase: float = 2.5  # Distance between front/rear axles
@export var wheel_radius: float = 0.35  # Wheel radius in meters
@export var unsprung_mass: float = 45.0  # Mass per wheel assembly
@export var sprung_mass: float = 1500.0  # Body mass

@export_group("Suspension Settings")
@export var suspension_stiffness: float = 80000.0  # N/m
@export var suspension_damping: float = 12000.0  # Ns/m
@export var suspension_compression_limit: float = 0.2  # Max compression m
@export var suspension_extension_limit: float = 0.3  # Max extension m
@export var ride_height: float = 0.25  # Resting suspension height m

@export_group("Brake System")
@export var brake_force_per_wheel: float = 4500.0  # Brake force per caliper
@export var brake_pressure_distribution: Vector4 = Vector4(0.5, 0.5, 0.5, 0.5)  # FL, FR, RL, RR pressure
@export var anti_lock_braking_system: bool = true

@export_group("Tire Properties")
@export var tire_friction_static: float = 1.2
@export var tire_friction_dynamic: float = 0.9
@export var tire_camber: float = -2.5  # Degrees
@export var tire_toe: float = 0.1  # Degrees

# ============================================================================
# POWERTRAIN INTEGRATION
# ============================================================================

@onready var powertrain: Node = get_parent().get_node_or_null("Powertrain")
@onready var body_rigid_body: RigidBody3D = get_parent().get_node_or_null("RigidBody3D")

# Gear ratios and final drive ratio
var gear_ratios: Array[float] = [0.0, 3.8, 2.3, 1.55, 1.15, 0.95, 0.75]  # Neutral + 6 gears
var final_drive_ratio: float = 3.73
var differential_type: String = "limited_slip"
var differential_lock_ratio: float = 0.5

# ============================================================================
# DRIFT MECHANICS
# ============================================================================

var drift_state: String = "normal"  # normal, oversteer, understeer, drift
var drift_threshold: float = 15.0  # Slip angle threshold for drift
var drift_recovery_rate: float = 0.15  # How fast grip returns
var drift_momentum: float = 0.0  # Accumulated drift momentum

# ============================================================================
# COLLISION HANDLING
# ============================================================================

var collision_points: Array[Dictionary] = []
var collision_impulse: Vector3 = Vector3.ZERO
var collision_normal: Vector3 = Vector3.ZERO
var collision_damage: float = 0.0
var chassis_health: float = 100.0
var maximum_health: float = 100.0

# ============================================================================
# TRACK & LAP SYSTEM
# ============================================================================

var checkpoint_passed: int = 0
var lap_count: int = 0
var best_lap_time: float = 999999.0
var current_lap_start_time: float = 0.0
var current_lap_time: float = 0.0
var last_checkpoint_position: Vector3 = Vector3.ZERO
var track_length: float = 0.0

# ============================================================================
# AERO DYNAMICS
# ============================================================================

@export_group("Aerodynamics")
@export var drag_coefficient: float = 0.32
@export var frontal_area: float = 2.1  # m^2
@export var downforce_coefficient: float = 0.8
@export var lift_coefficient: float = -0.1
@export var center_of_pressure: Vector3 = Vector3(0.0, 0.5, 0.0)

var aerodynamic_drag: float = 0.0
var aerodynamic_downforce: float = 0.0
var aerodynamic_lift: float = 0.0

# ============================================================================
# CENTER OF GRAVITY
# ============================================================================

var center_of_gravity: Vector3 = Vector3(0.0, 0.4, 0.0)
var moment_of_inertia_x: float = 2500.0
var moment_of_inertia_y: float = 3500.0
var moment_of_inertia_z: float = 2800.0

# ============================================================================
# ENGINE CHARACTERISTICS
# ============================================================================

var torque_curve: PackedFloat64Array = []
var power_curve: PackedFloat64Array = []
var fuel_consumption_rate: float = 0.01  # Liters per second at full throttle
var fuel_tank_capacity: float = 60.0  # Liters
var current_fuel: float = 60.0

# ============================================================================
# TELEMETRY DATA
# ============================================================================

var telemetry_data: Dictionary = {}

func _ready() -> void:
	_init_torque_curve()
	_init_power_curve()
	current_lap_start_time = Time.get_ticks_msec() / 1000.0
	
	if powertrain != null:
		powertrain.power_output_changed.connect(_on_powertrain_power_output_changed)
		powertrain.engine_stalled.connect(_on_engine_stalled)
	
	body_rigid_body = get_parent().get_node_or_null("RigidBody3D")
	if body_rigid_body == null:
		body_rigid_body = $RigidBody3D
		if body_rigid_body == null:
			var rb_node = Node3D.new()
			rb_node.name = "RigidBody3D"
			get_parent().add_child(rb_node)
			body_rigid_body = RBPhysicsMaterial.new()
			body_rigid_body.physics_material_override = PhysicsMaterial.new()
			body_rigid_body.density = 1.0
	
	_process_mode = ProcessModeEnum.ALWAYS

func _init_torque_curve() -> void:
	"""Initialize torque curve based on engine characteristics"""
	torque_curve.resize(100)
	for i in range(100):
		var normalized_rpm = float(i) / 99.0
		var rpm = normalized_rpm * engine_max_rpm
		torque_curve[i] = _calculate_torque_at_rpm(rpm)

func _init_power_curve() -> void:
	"""Initialize power curve based on torque curve"""
	power_curve.resize(100)
	for i in range(100):
		var normalized_rpm = float(i) / 99.0
		var rpm = normalized_rpm * engine_max_rpm
		var torque = _calculate_torque_at_rpm(rpm)
		power_curve[i] = (torque * rpm * 2.0 * PI) / 60000.0  # Convert to kW

func _calculate_torque_at_rpm(rpm: float) -> float:
	"""Calculate torque output at given RPM using a bell curve"""
	var idle_rpm = 800.0
	var peak_torque_rpm = 4500.0
	var max_torque = 450.0  # Nm
	
	if rpm <= idle_rpm:
		return 100.0
	elif rpm <= peak_torque_rpm:
		# Rising portion of curve
		var progress = (rpm - idle_rpm) / (peak_torque_rpm - idle_rpm)
		return 100.0 + (max_torque - 100.0) * pow(progress, 0.8)
	else:
		# Falling portion of curve
		var progress = (rpm - peak_torque_rpm) / (engine_max_rpm - peak_torque_rpm)
		return max_torque * (1.0 - progress * 0.6)

func _physics_process(delta: float) -> void:
	_update_inputs(delta)
	_update_engine(delta)
	_update_gearbox(delta)
	_update_vehicle_dynamics(delta)
	_update_wheels(delta)
	_update_aerodynamics(delta)
	_update_drift(delta)
	_update_collision_handling(delta)
	_update_telemetry(delta)
	emit_signals()

func _update_inputs(delta: float) -> void:
	"""Smooth input transitions"""
	var throttle_smooth = lerp(_throttle_input, _last_throttle, delta * 5.0)
	var brake_smooth = lerp(_brake_input, _last_brake, delta * 5.0)
	var steering_smooth = lerp(_steering_input, _last_steering, delta * 10.0)
	
	_last_throttle = _throttle_input
	_last_brake = _brake_input
	_last_steering = _steering_input

func _update_engine(delta: float) -> void:
	"""Update engine state and RPM"""
	if clutch_engaged:
		var target_rpm = _calculate_target_rpm()
		var rpm_change = (target_rpm - engine_rpm) * delta * 20.0
		
		engine_rpm += rpm_change
		engine_rpm = clamp(engine_rpm, engine_min_rpm, engine_max_rpm)
		
		torque_output = _calculate_torque_output()
		power_output = (torque_output * engine_rpm * 2.0 * PI) / 60000.0
	else:
		# Engine revving without load
		if _throttle_input > 0.1:
			engine_rpm = min(engine_rpm + delta * 1500.0, engine_max_rpm)
		else:
			engine_rpm = lerp(engine_rpm, engine_min_rpm, delta * 3.0)
		
		torque_output = 0.0
		power_output = 0.0

func _calculate_target_rpm() -> float:
	"""Calculate target RPM based on gear and speed"""
	var wheel_speed = abs(current_speed) / wheel_radius
	var drivetrain_ratio = gear_ratios[current_gear] * final_drive_ratio if current_gear > 0 else 0.0
	
	if drivetrain_ratio > 0:
		var target_rpm = wheel_speed * drivetrain_ratio * 60.0 / (2.0 * PI)
		return target_rpm
	else:
		return engine_min_rpm

func _calculate_torque_output() -> float:
	"""Calculate torque output based on current RPM and throttle"""
	if engine_rpm < engine_min_rpm or engine_rpm > engine_max_rpm:
		return 0.0
	
	var normalized_rpm = (engine_rpm - engine_min_rpm) / (engine_max_rpm - engine_min_rpm)
	var torque_index = int(normalized_rpm * 99.0)
	torque_index = clamp(torque_index, 0, 99)
	
	var base_torque = torque_curve[torque_index]
	return base_torque * _throttle_input

func _update_gearbox(delta: float) -> void:
	"""Update gearbox logic and gear shifting"""
	if not clutch_engaged:
		return
	
	if current_gear == 0:
		# In neutral, stay in neutral until throttle applied
		return
	
	# Check for upshift
	if _throttle_input > 0.5 and current_gear < 6:
		var next_gear_rpm = _calculate_next_gear_rpm()
		if next_gear_rpm < engine_rpm * 0.9:
			_shift_gear(current_gear + 1)
	
	# Check for downshift
	if _throttle_input < 0.3 and current_gear > 1:
		var next_gear_rpm = _calculate_next_gear_rpm()
		if next_gear_rpm > engine_rpm * 1.1:
			_shift_gear(current_gear - 1)
	
	# Auto-downshift if RPM too low
	if engine_rpm < engine_min_rpm * 1.2 and current_gear > 1:
		_shift_gear(current_gear - 1)
	
	# Manual gear shift override
	if abs(_steering_input) > 0.9:  # Steering wheel buttons trigger shifts
		if _steering_input > 0 and current_gear < 6:
			_shift_gear(current_gear + 1)
		elif _steering_input < 0 and current_gear > 1:
			_shift_gear(current_gear - 1)

func _calculate_next_gear_rpm() -> float:
	"""Calculate what RPM would be in next gear"""
	if current_gear == 0 or current_gear >= 6:
		return engine_rpm
	
	var wheel_speed = abs(current_speed) / wheel_radius
	var current_ratio = gear_ratios[current_gear] * final_drive_ratio
	var next_ratio = gear_ratios[current_gear + 1] * final_drive_ratio
	
	if current_ratio > 0:
		var next_rpm = wheel_speed * next_ratio * 60.0 / (2.0 * PI)
		return next_rpm
	
	return engine_rpm

func _shift_gear(new_gear: int) -> void:
	"""Execute gear shift"""
	if new_gear == current_gear:
		return
	
	var old_gear = current_gear
	current_gear = new_gear
	target_gear = new_gear
	
	# Brief clutch disengage during shift
	clutch_engaged = false
	await get_tree().create_timer(0.15).timeout
	clutch_engaged = true
	
	gear_changed.emit(old_gear, new_gear)

func _update_vehicle_dynamics(delta: float) -> void:
	"""Update main vehicle movement and velocity"""
	# Calculate acceleration based on gear and throttle
	var drive_force = _calculate_drive_force()
	
	if current_speed >= 0:
		acceleration = drive_force / default_vehicle_mass
	else:
		deceleration = drive_force / default_vehicle_mass
	
	# Apply acceleration
	current_speed += acceleration * delta
	
	# Apply air resistance and rolling resistance
	var drag_force = aerodynamic_drag
	var rolling_resistance = 0.015 * default_vehicle_mass * 9.81
	var total_resistance = drag_force + rolling_resistance
	
	if current_speed > 0:
		current_speed -= (total_resistance / default_vehicle_mass) * delta
	elif current_speed < 0:
		current_speed += (total_resistance / default_vehicle_mass) * delta
	
	# Clamp speed
	current_speed = clamp(current_speed, -reverse_speed, max_speed)
	
	# Calculate longitudinal acceleration
	longitudinal_acceleration = acceleration - (total_resistance / default_vehicle_mass)
	
	# Update position based on velocity
	var velocity_vector = transform.basis.z * (-current_speed)
	position += velocity_vector * delta
	
	# Apply steering rotation
	if abs(current_speed) > 0.5:
		var steer_factor = _steering_input * 0.5
		var turn_rate = steer_factor * (abs(current_speed) / max_speed) * 2.5
		yaw_angle += turn_rate * delta
		rotation.y = yaw_angle

func _calculate_drive_force() -> float:
	"""Calculate drive force from engine"""
	if current_gear == 0:
		return 0.0
	
	var wheel_rpm = abs(current_speed) / wheel_radius * 60.0 / (2.0 * PI)
	var drivetrain_ratio = gear_ratios[current_gear] * final_drive_ratio
	var engine_rpm_calc = wheel_rpm * drivetrain_ratio
	
	if engine_rpm_calc < engine_min_rpm or engine_rpm_calc > engine_max_rpm:
		return 0.0
	
	var torque = _calculate_torque_output()
	var wheel_torque = torque * drivetrain_ratio * 0.98  # Drivetrain loss
	
	return wheel_torque / wheel_radius

func _update_wheels(delta: float) -> void:
	"""Update wheel states and forces"""
	var wheel_positions = _get_wheel_positions()
	var wheel_velocities = _calculate_wheel_velocities(wheel_positions)
	
	for i in range(4):
		var wheel_pos = wheel_positions[i]
		var wheel_vel = wheel_velocities[i]
		
		# Calculate slip
		var slip_ratio = _calculate_slip_ratio(wheel_vel, i)
		
		# Update tire temperature based on slip
		_tire_temperature[i] += abs(slip_ratio) * delta * 0.5
		_tire_temperature[i] = min(_tire_temperature[i], 120.0)
		
		# Brake application
		var brake_force = brake_force_per_wheel * _brake_input * brake_pressure_distribution[i]
		wheel_vel -= brake_force * delta / unsprung_mass

func _get_wheel_positions() -> Array[Vector3]:
	"""Get positions of all four wheels"""
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
	
	return positions.transform(transform)

func _calculate_wheel_velocities(wheel_positions: Array[Vector3]) -> Array[Vector3]:
	"""Calculate velocity at each wheel position"""
	var velocities = []
	
	for pos in wheel_positions:
		var local_pos = transform.xform_inv(pos)
		var angular_velocity = Vector3(0.0, rotation_velocity, 0.0)
		var v = velocity + angular_velocity.cross(local_pos)
		velocities.append(v)
	
	return velocities

func _calculate_slip_ratio(wheel_velocity: Vector3, wheel_index: int) -> float:
	"""Calculate slip ratio for a wheel"""
	var wheel_forward = transform.basis.z
	var wheel_speed = wheel_velocity.dot(wheel_forward)
	
	if abs(current_speed) < 0.1:
		return 0.0
	
	var slip = (wheel_speed - current_speed) / abs(current_speed)
	return clamp(slip, -1.0, 1.0)

func _update_aerodynamics(delta: float) -> void:
	"""Update aerodynamic forces"""
	var dynamic_pressure = 0.5 * 1.225 * pow(abs(current_speed), 2)
	
	aerodynamic_drag = 0.5 * 1.225 * pow(abs(current_speed), 2) * drag_coefficient * frontal_area
	aerodynamic_downforce = 0.5 * 1.225 * pow(abs(current_speed), 2) * downforce_coefficient * frontal_area
	aerodynamic_lift = 0.5 * 1.225 * pow(abs(current_speed), 2) * lift_coefficient * frontal_area

func _update_drift(delta: float) -> void:
	"""Update drift mechanics"""
	slip_angle = abs(rotation_velocity * wheelbase / (2.0 * abs(current_speed))) if abs(current_speed) > 0.5 else 0.0
	lateral_acceleration = rotation_velocity * current_speed
	
	var grip_level = grip_coefficient
	
	if _handbrake_input > 0.7 and abs(current_speed) > 20.0:
		grip_level *= drift_coefficient
		drift_enabled = true
	
	if slip_angle > drift_threshold and drift_enabled:
		if drift_state != "drift":
			drift_state = "drift"
			drift_started.emit()
		
		drift_momentum = min(drift_momentum + delta * 5.0, 1.0)
		grip_level *= (1.0 - drift_momentum * 0.3)
	else:
		drift_momentum = max(drift_momentum - delta * drift_recovery_rate, 0.0)
		
		if drift_momentum < 0.1 and drift_state == "drift":
			drift_state = "normal"
			drift_ended.emit()
		
		if slip_angle > drift_threshold * 1.5:
			drift_state = "understeer"
		elif slip_angle < -drift_threshold * 1.5:
			drift_state = "oversteer"
		else:
			drift_state = "normal"

func _update_collision_handling(delta: float) -> void:
	"""Handle collision detection and response"""
	if is_colliding():
		var collision = get_collision_contact_count()
		
		for i in range(collision):
			var col_info = get_collision_callback(i)
			
			collision_points.append({
				"position": col_info.position,
				"normal": col_info.normal,
				"timestamp": Time.get_ticks_msec()
			})
			
			collision_impulse = col_info.normal * col_info.get_normal().length()
			collision_normal = col_info.normal
			
			# Calculate damage based on impact force
			var impact_force = collision_impulse.length() * default_vehicle_mass
			collision_damage = impact_force * 0.001
			
			chassis_health = max(chassis_health - collision_damage, 0.0)
			
			collision_detected.emit({
				"position": col_info.position,
				"normal": col_info.normal,
				"damage": collision_damage
			})
	
	# Remove old collision points
	for i in range(collision_points.size() - 10):
		collision_points.remove_at(0)

func _update_telemetry(delta: float) -> void:
	"""Update telemetry data collection"""
	telemetry_data = {
		"speed": current_speed,
		"rpm": engine_rpm,
		"gear": current_gear,
		"throttle": _throttle_input,
		"brake": _brake_input,
		"steering": _steering_input,
		"lateral_accel": lateral_acceleration,
		"longitudinal_accel": longitudinal_acceleration,
		"slip_angle": slip_angle,
		"drift_state": drift_state,
		"fuel_level": current_fuel / fuel_tank_capacity * 100.0,
		"chassis_health": chassis_health,
		"tire_temperatures": tire_temperature
	}

func emit_signals() -> void:
	"""Emit state change signals"""
	if abs(speed_changed.signal_listed) > 0:
		speed_changed.emit(current_speed)
	
	if abs(rpm_changed.signal_listed) > 0:
		rpm_changed.emit(engine_rpm)
	
	if abs(gear_changed.signal_listed) > 0 and target_gear != current_gear:
		gear_changed.emit(current_gear, target_gear)

# ============================================================================
# PUBLIC API
# ============================================================================

func reset_vehicle() -> void:
	"""Reset vehicle to initial state"""
	current_speed = 0.0
	engine_rpm = engine_min_rpm
	current_gear = 0
	target_gear = 0
	chassis_health = maximum_health
	tire_temperature = Vector4(0.0, 0.0, 0.0, 0.0)
	drift_enabled = false
	drift_state = "normal"
	collision_points.clear()
	
	position = Vector3(0.0, 0.0, 0.0)
	rotation = Vector3(0.0, 0.0, 0.0)

func apply_damage(amount: float) -> void:
	"""Apply damage to vehicle"""
	chassis_health = max(chassis_health - amount, 0.0)
	
	if chassis_health <= 0:
		_on_vehicle_destroyed()

func check_fuel() -> void:
	"""Check and update fuel consumption"""
	if _throttle_input > 0.1:
		current_fuel -= fuel_consumption_rate * _throttle_input * 0.02
		if current_fuel <= 0:
			_on_out_of_fuel()

func get_wheel_force_vector(wheel_index: int) -> Vector3:
	"""Get force vector for a specific wheel"""
	var wheel_positions = _get_wheel_positions()
	var wheel_pos = wheel_positions[wheel_index]
	
	var force = Vector3.ZERO
	
	match wheel_index:
		0: # Front Left
			force = Vector3(0.0, 0.0, -1.0)
		1: # Front Right
			force = Vector3(0.0, 0.0, -1.0)
		2: # Rear Left
			force = Vector3(0.0, 0.0, 1.0)
		3: # Rear Right
			force = Vector3(0.0, 0.0, 1.0)
	
	return force.rotated(Vector3.UP, yaw_angle)

func set_track_length(length: float) -> void:
	"""Set track length for lap timing"""
	track_length = length
	checkpoint_passed = 0

func record_checkpoint() -> void:
	"""Record passing through a checkpoint"""
	checkpoint_passed += 1
	
	if checkpoint_passed >= 10:  # Assuming 10 checkpoints per lap
		lap_count += 1
		var lap_time = Time.get_ticks_msec() / 1000.0 - current_lap_start_time
		best_lap_time = min(best_lap_time, lap_time)
		current_lap_start_time = Time.get_ticks_msec() / 1000.0
		
		lap_completed.emit({
			"lap_number": lap_count,
			"time": lap_time,
			"best_lap": best_lap_time
		})

func get_current_lap_time() -> float:
	"""Get current lap elapsed time"""
	return Time.get_ticks_msec() / 1000.0 - current_lap_start_time

func get_telemetry_snapshot() -> Dictionary:
	"""Get current telemetry snapshot"""
	return telemetry_data.duplicate()

func set_gear_manually(gear: int) -> void:
	"""Manually set gear (for manual transmission mode)"""
	if gear >= 0 and gear <= 6:
		target_gear = gear
		if clutch_engaged:
			_shift_gear(gear)

func toggle_drift_mode(enabled: bool) -> void:
	"""Toggle drift mode"""
	drift_enabled = enabled

func get_vehicle_status() -> Dictionary:
	"""Get comprehensive vehicle status"""
	return {
		"speed": current_speed,
		"rpm": engine_rpm,
		"gear": current_gear,
		"throttle": _throttle_input,
		"brake": _brake_input,
		"steering": _steering_input,
		"health": chassis_health,
		"fuel": current_fuel,
		"drift_state": drift_state,
		"lap_count": lap_count,
		"best_lap": best_lap_time
	}

# ============================================================================
# SIGNAL HANDLERS
# ============================================================================

func _on_powertrain_power_output_changed(power: float) -> void:
	"""Handle powertrain power output changes"""
	power_output = power

func _on_engine_stalled() -> void:
	"""Handle engine stall event"""
	engine_stalled.emit()
	current_speed = 0.0
	engine_rpm = 0.0

func _on_vehicle_destroyed() -> void:
	"""Handle vehicle destruction"""
	queue_free()

func _on_out_of_fuel() -> void:
	"""Handle out of fuel event"""
	current_fuel = 0.0
	engine_rpm = engine_min_rpm
	current_speed = 0.0
	_on_engine_stalled()

# ============================================================================
# SETTERS
# ============================================================================

func _set_throttle_input(value: float) -> void:
	_throttle_input = clamp(value, 0.0, 1.0)

func _set_brake_input(value: float) -> void:
	_brake_input = clamp(value, 0.0, 1.0)

func _set_steering_input(value: float) -> void:
	_steering_input = clamp(value, -1.0, 1.0)

func _set_clutch_input(value: float) -> void:
	_clutch_input = clamp(value, 0.0, 1.0)
	clutch_engaged = value > 0.5

func _set_handbrake_input(value: float) -> void:
	_handbrake_input = clamp(value, 0.0, 1.0)

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================

func calculate_cornering_force(angle: float, grip: float) -> float:
	"""Calculate cornering force based on slip angle and grip"""
	var cornering_stiffness = 80000.0  # N/rad
	var slip_rad = deg_to_rad(angle)
	return cornering_stiffness * slip_rad * grip

func apply_suspension_force(wheel_pos: Vector3, ground_height: float) -> Vector3:
	"""Apply suspension force to wheel"""
	var wheel_offset = wheel_pos - position
	var suspension_distance = wheel_offset.y
	
	var compression = ride_height - suspension_distance
	compression = clamp(compression, -suspension_extension_limit, suspension_compression_limit)
	
	var spring_force = suspension_stiffness * compression
	var damping_force = suspension_damping * velocity.y
	
	return Vector3(0.0, spring_force + damping_force, 0.0)

func simulate_weather_effects(weather_condition: String, intensity: float) -> void:
	"""Simulate weather effects on vehicle dynamics"""
	match weather_condition:
		"rain":
			grip_coefficient = 1.0 - intensity * 0.4
			drift_coefficient = 0.85 - intensity * 0.3
		"snow":
			grip_coefficient = 1.0 - intensity * 0.7
			drift_coefficient = 0.7 - intensity * 0.3
		"ice":
			grip_coefficient = 1.0 - intensity * 0.9
			drift_coefficient = 0.5 - intensity * 0.3
		_:
			grip_coefficient = 1.0
			drift_coefficient = 0.85

func enable_anti_lock_braking(enable: bool) -> void:
	"""Enable/disable ABS"""
	anti_lock_braking_system = enable

func adjust_balance(front_bias: float, rear_bias: float) -> void:
	"""Adjust brake balance distribution"""
	if front_bias + rear_bias <= 1.0:
		brake_pressure_distribution.x = front_bias
		brake_pressure_distribution.y = front_bias
		brake_pressure_distribution.z = rear_bias
		brake_pressure_distribution.w = rear_bias

func get_performance_metrics() -> Dictionary:
	"""Get vehicle performance metrics"""
	return {
		"top_speed": max_speed,
		"acceleration_0_100": calculate_0_100_time(),
		"braking_distance": calculate_braking_distance(),
		"cornering_g": abs(lateral_acceleration / 9.81),
		"power_to_weight": power_output / default_vehicle_mass
	}

func calculate_0_100_time() -> float:
	"""Calculate theoretical 0-100 km/h time"""
	var target_speed = 100.0 / 3.6  # m/s
	var total_time = 0.0
	var current_v = 0.0
	
	while current_v < target_speed:
		var accel = _calculate_drive_force() / default_vehicle_mass
		total_time += 0.1 / accel if accel > 0 else 999.0
		current_v += accel * 0.1
	
	return total_time

func calculate_braking_distance() -> float:
	"""Calculate braking distance from max speed"""
	var decel = brake_force_per_wheel * 4.0 / default_vehicle_mass
	var braking_dist = pow(max_speed, 2) / (2.0 * decel)
	return braking_dist

func save_vehicle_state(file_path: String) -> void:
	"""Save vehicle state to file"""
	var state = get_vehicle_status()
	var json_str = JSON.stringify(state)
	
	var file = FileAccess.open(file_path, FileAccess.WRITE)
	if file:
		file.store_string(json_str)
		file.close()

func load_vehicle_state(file_path: String) -> void:
	"""Load vehicle state from file"""
	var file = FileAccess.open(file_path, FileAccess.READ)
	if file and file.file_exists(file_path):
		var json_str = file.get_as_text()
		file.close()
		
		var parser = JSON.new()
		var error = parser.parse(json_str)
		if error == OK:
			var data = parser.data
			current_speed = data.get("speed", current_speed)
			chassis_health = data.get("health", chassis_health)
			current_fuel = data.get("fuel", current_fuel)

func clone() -> VehicleController:
	"""Create a copy of this vehicle controller"""
	var clone = VehicleController.new()
	clone.current_speed = current_speed
	clone.max_speed = max_speed
	clone.engine_rpm = engine_rpm
	clone.current_gear = current_gear
	clone.chassis_health = chassis_health
	clone.drift_enabled = drift_enabled
	clone.track_length = track_length
	clone.lap_count = lap_count
	clone.best_lap_time = best_lap_time
	
	return clone
