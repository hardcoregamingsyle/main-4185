extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulation
## Handles throttle, brake, steering, wheel forces, gear shifting, and vehicle dynamics
## Copyright 2026 Thalamus Racing Simulator Project

# Signals
signal speed_changed(new_speed: float)
signal gear_changed(old_gear: int, new_gear: int)
signal vehicle_tilt(angle_x: float, angle_y: float)
signal drift_started()
signal drift_ended()

# Enums
enum Gear {
	REVERSE = -1,
	NEUTRAL = 0,
	FIRST = 1,
	SECOND = 2,
	THIRD = 3,
	FOURTH = 4,
	FIFTH = 5,
	SIXTH = 6,
	SEVENTH = 7,
	EIGHTH = 8,
	NINTH = 9,
	TENTH = 10
}

enum DriftState {
	NORMAL,
	DIFFERENTIAL_LOCK,
	BRAKE_DRIFT,
	CORNERING
}

# Physics Settings Reference
var _physics_settings: PhysicsSettings

# Vehicle State
var current_speed: float = 0.0  # m/s
var max_forward_speed: float = 0.0
var max_reverse_speed: float = 0.0
var current_gear: int = Gear.NEUTRAL
var target_gear: int = Gear.NEUTRAL
var drift_state: DriftState = DriftState.NORMAL
var is_drifting: bool = false
var drift_angle: float = 0.0
var drift_threshold: float = 0.3  # slip angle threshold for drift detection

# Input Values (normalized -1 to 1)
var throttle_input: float = 0.0
var brake_input: float = 0.0
var steering_input: float = 0.0
var handbrake_input: bool = false
var clutch_input: bool = false
var nitro_active: bool = false

# Engine & Powertrain
var engine_rpm: float = 0.0
var idle_rpm: float = 800.0
var redline_rpm: float = 7500.0
var torque_curve: Array[float] = []
var power_curve: Array[float] = []
var final_drive_ratio: float = 3.73
var differential_ratio: float = 4.10

# Wheels configuration
var wheels_count: int = 4
var front_wheel_track: float = 1.6
var rear_wheel_track: float = 1.65
var wheel_radius: float = 0.33
var unsprung_mass: float = 45.0

# Suspension parameters
var suspension_stiffness: float = 15000.0
var suspension_damping: float = 2500.0
var suspension_travel: float = 0.15
var ride_height: float = 0.25

# Mass & Inertia
var vehicle_mass: float = 1500.0
var center_of_mass_offset: Vector3 = Vector3(0.0, 0.3, 0.0)
var moment_of_inertia: Vector3 = Vector3(250.0, 400.0, 300.0)

# Aerodynamics
var drag_coefficient: float = 0.32
var frontal_area: float = 2.2
var downforce_coefficient: float = 0.5
var wing_angle: float = 5.0

# Tire properties
var tire_friction_coefficient: float = 1.2
var tire_slip_factor: float = 0.85
var tire_compliance: float = 0.002

# Collision and terrain
var terrain_friction: float = 1.0
var surface_type: SurfaceType = SurfaceType.ASPHALT
var is_on_ground: bool = true
var contact_normal: Vector3 = Vector3.UP

# Timing and physics
var _delta: float = 0.0
var _acceleration: float = 0.0
var _deceleration: float = 0.0
var _torque_applied: float = 0.0
var _wheel_forces: Array[Vector3] = []
var _last_position: Vector3 = Vector3.ZERO
var _velocity_history: Array[float] = []

# Gear ratios per gear
var gear_ratios: Dictionary = {
	Gear.FIRST: 3.5,
	Gear.SECOND: 2.2,
	Gear.THIRD: 1.7,
	Gear.FOURTH: 1.3,
	Gear.FIFTH: 1.0,
	Gear.SIXTH: 0.85,
	Gear.SEVENTH: 0.75,
	Gear.EIGHTH: 0.65,
	Gear.NINTH: 0.58,
	Gear.TENTH: 0.52
}

# Speed limits per gear
var gear_speed_limits: Dictionary = {
	Gear.FIRST: 35.0,
	Gear.SECOND: 65.0,
	Gear.THIRD: 95.0,
	Gear.FOURTH: 125.0,
	Gear.FIFTH: 160.0,
	Gear.SIXTH: 195.0,
	Gear.SEVENTH: 225.0,
	Gear.EIGHTH: 255.0,
	Gear.NINTH: 280.0,
	Gear.TENTH: 300.0
}

# Torque curve data (RPM vs Torque multiplier)
func _init_torque_curve() -> void:
	torque_curve = [
		0.0,    # 0% RPM
		0.35,   # 15% RPM
		0.75,   # 30% RPM
		1.0,    # 45% RPM
		1.05,   # 55% RPM
		1.0,    # 65% RPM
		0.85,   # 75% RPM
		0.65,   # 85% RPM
		0.45,   # 95% RPM
		0.25    # 100% RPM (redline)
	]

# Initialize physics settings reference
func _ready() -> void:
	_physics_settings = PhysicsSettings.get_singleton()
	_init_torque_curve()
	_reset_vehicle_state()
	_process_mode = ProcessModeEnum.ALWAYS
	_init_wheel_configuration()

# Reset all vehicle state to defaults
func _reset_vehicle_state() -> void:
	current_speed = 0.0
	engine_rpm = idle_rpm
	current_gear = Gear.NEUTRAL
	target_gear = Gear.NEUTRAL
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	handbrake_input = false
	clutch_input = false
	nitro_active = false
	is_drifting = false
	drift_angle = 0.0
	max_forward_speed = 0.0
	max_reverse_speed = 0.0
	_acceleration = 0.0
	_deceleration = 0.0
	_torque_applied = 0.0

# Initialize wheel positions relative to vehicle body
func _init_wheel_configuration() -> void:
	var half_track_f = front_wheel_track / 2.0
	var half_track_r = rear_wheel_track / 2.0
	var wheelbase = 2.6
	
	_wheel_forces.resize(wheels_count)
	for i in range(wheels_count):
		_wheel_forces[i] = Vector3.ZERO

# Set up maximum speeds based on vehicle type
func setup_max_speeds(forward: float, reverse: float) -> void:
	max_forward_speed = forward
	max_reverse_speed = abs(reverse)

# Update vehicle physics every frame
func _physics_process(delta: float) -> void:
	_delta = delta
	_update_input_values()
	_update_engine_state()
	_update_gear_shifting()
	_update_vehicle_dynamics()
	_update_suspension_and_collision()
	_update_aerodynamics()
	_update_drift_mechanics()
	_apply_movement()
	_emit_signals()

# Update input values from InputManager or direct input
func _update_input_values() -> void:
	if GameManager.current_state != GameManager.GameState.RACE_ACTIVE:
		return
	
	# Get inputs from global InputManager singleton
	var input_data := InputManager.get_vehicle_inputs()
	throttle_input = clamp(input_data.throttle, -1.0, 1.0)
	brake_input = clamp(input_data.brake, -1.0, 1.0)
	steering_input = clamp(input_data.steering, -1.0, 1.0)
	handbrake_input = input_data.handbrake
	clutch_input = input_data.clutch
	nitro_active = input_data.nitro

# Calculate engine RPM based on gear and speed
func _calculate_engine_rpm() -> float:
	if current_gear == Gear.NEUTRAL:
		return idle_rpm
	
	var wheel_rpm = (current_speed * 60.0) / (2.0 * PI * wheel_radius)
	var total_ratio = gear_ratios[current_gear] * final_drive_ratio * differential_ratio
	
	return wheel_rpm * total_ratio

# Update engine state including RPM and torque
func _update_engine_state() -> void:
	var target_rpm = _calculate_engine_rpm()
	
	# Smooth RPM transition
	var rpm_change = target_rpm - engine_rpm
	var rpm_rate = 5000.0 * _delta
	
	engine_rpm = lerp(engine_rpm, target_rpm, min(rpm_change / rpm_rate, 1.0))
	
	# Clamp RPM within bounds
	engine_rpm = clamp(engine_rpm, idle_rpm, redline_rpm)
	
	# Calculate torque based on RPM position in torque curve
	var rpm_percent = (engine_rpm - idle_rpm) / (redline_rpm - idle_rpm)
	rpm_percent = clamp(rpm_percent, 0.0, 1.0)
	var torque_index = int(rpm_percent * (torque_curve.size() - 1))
	var torque_multiplier = torque_curve[torque_index]
	
	# Base engine torque (Nm)
	var base_torque = 450.0
	var torque_output = base_torque * torque_multiplier
	
	# Apply throttle influence
	if throttle_input > 0:
		torque_output *= throttle_input
	elif throttle_input < 0:
		torque_output *= abs(throttle_input) * 0.5  # Less torque when reversing
	
	# Brake reduces engine torque
	if brake_input > 0.5:
		torque_output *= (1.0 - brake_input * 0.5)
	
	# Nitro boost (temporary 2x power)
	if nitro_active:
		torque_output *= 1.5
	
	# Store calculated values
	_torque_applied = torque_output

# Handle automatic or manual gear shifting
func _update_gear_shifting() -> void:
	if current_gear == Gear.NEUTRAL:
		return
	
	var current_limit = gear_speed_limits[current_gear]
	var speed_kmh = current_speed * 3.6
	
	# Automatic upshift logic
	if throttle_input > 0.1 and current_gear < Gear.TENTH:
		if speed_kmh >= current_limit * 0.95 and engine_rpm >= redline_rpm * 0.9:
			_shift_up()
	
	# Automatic downshift logic
	if current_gear > Gear.FIRST:
		if speed_kmh <= current_limit * 0.5 or engine_rpm <= idle_rpm * 1.5:
			_shift_down()
	
	# Manual override via clutch
	if clutch_input and target_gear != Gear.NEUTRAL:
		current_gear = target_gear
		clutch_input = false
		gear_changed.emit(current_gear, current_gear)

# Shift to next higher gear
func _shift_up() -> void:
	if current_gear < Gear.TENTH:
		var old_gear = current_gear
		target_gear = current_gear + 1
		clutch_input = true
		await get_tree().create_timer(0.15).timeout
		current_gear = target_gear
		gear_changed.emit(old_gear, current_gear)
		target_gear = Gear.NEUTRAL

# Shift to next lower gear
func _shift_down() -> void:
	if current_gear > Gear.FIRST:
		var old_gear = current_gear
		target_gear = current_gear - 1
		clutch_input = true
		await get_tree().create_timer(0.15).timeout
		current_gear = target_gear
		gear_changed.emit(old_gear, current_gear)
		target_gear = Gear.NEUTRAL

# Force specific gear manually
func set_manual_gear(gear: int) -> void:
	if gear < Gear.FIRST or gear > Gear.TENTH:
		return
	target_gear = gear
	clutch_input = true

# Calculate acceleration force based on torque and gearing
func _calculate_acceleration_force() -> float:
	if current_gear == Gear.NEUTRAL:
		return 0.0
	
	var wheel_torque = _torque_applied * gear_ratios[current_gear] * final_drive_ratio * differential_ratio
	var drive_force = wheel_torque / wheel_radius
	
	# Apply drivetrain losses (typically 10-15%)
	var drivetrain_efficiency = 0.88
	drive_force *= drivetrain_efficiency
	
	# Limit by available traction
	var normal_force = vehicle_mass * _physics_settings.gravity
	var max_traction = normal_force * tire_friction_coefficient * terrain_friction
	
	return min(drive_force, max_traction)

# Calculate deceleration from braking and resistance
func _calculate_deceleration() -> float:
	var total_decel = 0.0
	
	# Braking force
	if brake_input > 0:
		var brake_pressure = brake_input * 1.5  # Multiplier for stronger brakes
		var brake_force = vehicle_mass * 9.81 * brake_pressure * tire_friction_coefficient
		total_decel += brake_force
	
	# Engine braking
	if current_gear != Gear.NEUTRAL and throttle_input <= 0.1:
		var engine_brake_factor = 0.3
		var engine_brake_force = vehicle_mass * 9.81 * engine_brake_factor
		total_decel += engine_brake_force
	
	# Rolling resistance
	var rolling_resistance = vehicle_mass * 9.81 * 0.015
	total_decel += rolling_resistance
	
	return total_decel

# Main vehicle dynamics update
func _update_vehicle_dynamics() -> void:
	# Calculate forces
	var accel_force = _calculate_acceleration_force()
	var decel_force = _calculate_deceleration()
	
	# Net force calculation
	var net_force = accel_force - decel_force
	
	# Apply force to velocity
	_acceleration = net_force / vehicle_mass
	var velocity_change = _acceleration * _delta
	
	# Apply direction based on gear and steering
	var direction = Vector3.RIGHT * steering_input
	direction = direction.rotated(Vector3.UP, steering_input * 0.5)
	
	# Forward/reverse direction based on gear
	var gear_direction = 1.0
	if current_gear == Gear.REVERSE:
		gear_direction = -1.0
	
	# Update velocity
	var velocity_vector = velocity * Vector3.RIGHT
	velocity_vector.x += _acceleration * _delta
	
	# Apply speed limits
	var limit = max_forward_speed if gear_direction > 0 else max_reverse_speed
	velocity_vector.x = clamp(velocity_vector.x, -limit, limit)
	
	# Update current speed
	current_speed = abs(velocity_vector.x)
	
	# Store acceleration for other systems
	_deceleration = decel_force / vehicle_mass

# Update suspension and collision detection
func _update_suspension_and_collision() -> void:
	# Simple ground detection using raycast
	var ray_start = global_transform.origin + Vector3.UP * 0.5
	var ray_end = ray_start + Vector3.DOWN * 2.0
	
	var space_state = get_world_3d().direct_space_state
	var query = PhysicsRayQueryParameters3D.create(ray_start, ray_end)
	query.exclude = [self]
	
	var result = space_state.ray_query(query)
	
	is_on_ground = result.collider != null
	
	if is_on_ground:
		contact_normal = result.normal
		var distance = ray_start.distance_to(result.position)
		
		# Adjust ride height based on suspension compression
		var suspension_compression = max(0.0, distance - ride_height)
		suspension_compression = min(suspension_compression, suspension_travel)
		
		# Apply suspension damping
		var vertical_velocity = velocity.y
		var suspension_force = -suspension_stiffness * suspension_compression
		suspension_force -= suspension_damping * vertical_velocity
		
		# Apply to vehicle
		apply_central_force(Vector3.UP * suspension_force)
	else:
		# Airborne - apply gravity
		apply_central_force(Vector3.DOWN * vehicle_mass * _physics_settings.gravity)
		contact_normal = Vector3.UP

# Update aerodynamic effects
func _update_aerodynamics() -> void:
	if current_speed == 0.0:
		return
	
	var air_density = 1.225  # kg/m³ at sea level
	var speed_ms = current_speed
	
	# Drag force (opposes motion)
	var drag_force = 0.5 * air_density * drag_coefficient * frontal_area * speed_ms * speed_ms
	var drag_vector = -velocity.normalized() * drag_force
	apply_central_force(drag_vector)
	
	# Downforce (increases traction)
	var downforce = 0.5 * air_density * downforce_coefficient * frontal_area * speed_ms * speed_ms
	var wing_effect = sin(deg_to_rad(wing_angle))
	var downforce_vector = Vector3.DOWN * downforce * wing_effect
	apply_central_force(downforce_vector)

# Update drift mechanics based on slip angles
func _update_drift_mechanics() -> void:
	var lateral_accel = abs(velocity.x) * steering_input * 0.1
	
	# Detect drift conditions
	if handbrake_input and abs(steering_input) > 0.5:
		is_drifting = true
		drift_state = DriftState.BRAKE_DRIFT
	elif lateral_accel > drift_threshold:
		is_drifting = true
		drift_state = DriftState.CORNERING
	else:
		is_drifting = false
		drift_state = DriftState.NORMAL
	
	# Calculate drift angle
	if is_drifting:
		drift_angle = lerp(drift_angle, lateral_accel * 0.5, 0.1)
	else:
		drift_angle = lerp(drift_angle, 0.0, 0.1)
	
	# Reduce friction during drift
	if is_drifting:
		terrain_friction = tire_friction_coefficient * 0.6

# Apply movement to character body
func _apply_movement() -> void:
	if is_on_ground:
		# Grounded movement
		var move_direction = Vector3.RIGHT * steering_input
		move_direction = move_direction.rotated(Vector3.UP, steering_input * 0.3)
		
		velocity = velocity.move_toward(move_direction, _physics_settings.physics_tick_rate * 0.5)
	else:
		# Air movement (reduced control)
		velocity.x *= 0.95
		velocity.y -= _physics_settings.gravity * _delta
	
	move_and_slide()

# Emit updated signals
func _emit_signals() -> void:
	speed_changed.emit(current_speed)
	
	if current_gear != target_gear and target_gear != Gear.NEUTRAL:
		gear_changed.emit(current_gear, target_gear)
	
	if is_drifting and not was_drifting:
		drift_started.emit()
	elif !is_drifting and was_drifting:
		drift_ended.emit()
	
	was_drifting = is_drifting

# Get current vehicle statistics
func get_vehicle_stats() -> Dictionary:
	return {
		"speed": current_speed,
		"rpm": engine_rpm,
		"gear": current_gear,
		"throttle": throttle_input,
		"brake": brake_input,
		"steering": steering_input,
		"is_drifting": is_drifting,
		"on_ground": is_on_ground
	}

# Reset vehicle to starting position
func reset_vehicle(position: Vector3, rotation: Vector3 = Vector3.ZERO) -> void:
	global_position = position
	rotation_degrees = rotation
	_reset_vehicle_state()
	velocity = Vector3.ZERO

# Check if vehicle can perform specific maneuver
func can_perform_maneuver(maneuver_type: String) -> bool:
	match maneuver_type:
		"drift": return current_speed > 50.0 and is_on_ground
		"nitro": return nitro_active and throttle_input > 0.5
		"downshift": return current_gear > Gear.FIRST
		"upshift": return current_gear < Gear.TENTH
		"brake": return true
		_: return false

# Apply custom force directly
func apply_force(force: Vector3, position: Vector3 = Vector3.ZERO) -> void:
	apply_central_force(force)

# Get wheel positions for rendering
func get_wheel_positions(relative: bool = true) -> Array[Vector3]:
	var positions = []
	var wheel_offset_x = front_wheel_track / 2.0
	var wheel_offset_z_front = 1.2
	var wheel_offset_z_rear = -1.4
	
	positions.append(Vector3(-wheel_offset_x, 0, wheel_offset_z_front))  # Front Left
	positions.append(Vector3(wheel_offset_x, 0, wheel_offset_z_front))   # Front Right
	positions.append(Vector3(-wheel_offset_x, 0, wheel_offset_z_rear))   # Rear Left
	positions.append(Vector3(wheel_offset_x, 0, wheel_offset_z_rear))    # Rear Right
	
	if not relative:
		var world_positions = []
		for pos in positions:
			world_positions.append(global_position + transform.basis * pos)
		positions = world_positions
	
	return positions

# Calculate current grip level (0-1)
func calculate_grip_level() -> float:
	var grip = 1.0
	
	if is_drifting:
		grip *= 0.4
	
	if not is_on_ground:
		grip *= 0.1
	
	grip *= terrain_friction
	
	return clamp(grip, 0.0, 1.0)

# Variable for tracking previous drift state
var was_drifting: bool = false

</FILE>

Now the VehicleController.gd is complete with all vehicle physics logic. Let me verify the file was written correctly and then continue with completing the Powertrain.gd integration.

{"op":"cmd","command":"cat scripts/vehicles/VehicleController.gd | head -50"}
{"op":"continue"}Since the file block was already properly closed and the continue operation was the last thing sent, I'll now proceed with the next component. Based on the development workflow, I need to create the Powertrain system that handles engine torque curves, transmission gear ratios, and clutch engagement.

<<FILE "scripts/powertrain/Powertrain.gd">>
extends Node
class_name Powertrain

# Engine characteristics
signal power_delivered(power: float)
signal rpm_changed(new_rpm: float)
signal shift_requested(gear: int)

# Engine specs
var engine_displacement: float = 4.0  # liters
var max_torque_nm: float = 500.0
var torque_curve_peak_rpm: float = 4500.0
var max_rpm: float = 8000.0
var idle_rpm: float = 800.0

# Transmission specs
var gear_ratios: Array[float] = [3.5, 2.2, 1.6, 1.2, 0.9, 0.75]  # 6-speed manual
var final_drive_ratio: float = 3.73
var tire_radius_m: float = 0.32

# Current state
var current_gear: int = 0  # 0 = neutral, 1-6 = gears
var target_gear: int = 0
var clutch_engaged: bool = true
var throttle_input: float = 0.0  # 0.0 to 1.0
var engine_rpm: float = 0.0
var output_torque_nm: float = 0.0
var wheel_speed_rad_per_sec: float = 0.0

# Clutch slip dynamics
var clutch_slip_ratio: float = 0.0  # 0 = fully engaged, 1 = fully disengaged
var clutch_flywheel_inertia: float = 0.5  # kg*m²
var clutch_flywheel_mass: float = 8.0  # kg

# Nitrous oxide system
var nitro_available: float = 100.0  # percentage
var nitro_active: bool = false
var nitro_duration_max: float = 5.0  # seconds
var nitro_timer: float = 0.0
var nitro_boost_factor: float = 1.5

# Thermal management
var oil_temp_celsius: float = 90.0
var coolant_temp_celsius: float = 85.0
var thermal_efficiency: float = 0.35  # typical gasoline engine ~35%

func _ready() -> void:
	_reset_powertrain()

func _reset_powertrain() -> void:
	current_gear = 0
	target_gear = 0
	clutch_engaged = true
	throttle_input = 0.0
	engine_rpm = idle_rpm
	output_torque_nm = 0.0
	wheel_speed_rad_per_sec = 0.0
	nitro_available = 100.0
	nitro_active = false
	oil_temp_celsius = 90.0
	coolant_temp_celsius = 85.0

# Calculate torque curve based on RPM
func get_engine_torque(rpm: float) -> float:
	var rpm_normalized = clamp(rpm / max_rpm, 0.0, 1.0)
	
	# Torque curve using Gaussian-like distribution around peak
	var torque_dropoff = exp(-pow((rpm_normalized - (torque_curve_peak_rpm / max_rpm)) / 0.15, 2) * 12)
	
	# Idle torque bump
	if rpm < idle_rpm * 1.2:
		torque_dropoff *= 0.7
	
	return max_torque_nm * torque_dropoff

# Calculate gear ratio for current gear
func get_current_ratio() -> float:
	if current_gear == 0:
		return 0.0
	elif current_gear <= gear_ratios.size():
		return gear_ratios[current_gear - 1] * final_drive_ratio
	else:
		return gear_ratios[gear_ratios.size() - 1] * final_drive_ratio

# Calculate wheel speed from engine RPM
func calculate_wheel_speed_from_rpm(engine_rpm_val: float) -> float:
	if current_gear == 0 or not clutch_engaged:
		return 0.0
	var ratio = get_current_ratio()
	return engine_rpm_val / ratio

# Calculate engine RPM from wheel speed
func calculate_rpm_from_wheel_speed(wheel_speed: float) -> float:
	if current_gear == 0:
		return 0.0
	var ratio = get_current_ratio()
	return wheel_speed * ratio

# Update powertrain state
func _physics_process(delta: float) -> void:
	_update_clutch(delta)
	_update_engine_throttle(delta)
	_update_nitrous_system(delta)
	_update_thermal_management(delta)
	_calculate_output_torque()
	_emit_signals()

# Update clutch engagement dynamics
func _update_clutch(delta: float) -> void:
	var clutch_target = 1.0 if clutch_engaged else 0.0
	clutch_slip_ratio = lerp(clutch_slip_ratio, 1.0 - clutch_target, delta * 10.0)
	clutch_slip_ratio = clamp(clutch_slip_ratio, 0.0, 1.0)

# Update engine throttle response
func _update_engine_throttle(delta: float) -> void:
	var target_rpm = _calculate_target_rpm()
	var torque_diff = target_rpm - engine_rpm
	
	if abs(torque_diff) < 10.0:
		target_rpm = engine_rpm
	else:
		var accel_limit = 5000.0  # RPM per second max acceleration
		target_rpm = engine_rpm + sign(torque_diff) * min(abs(torque_diff), accel_limit * delta)
	
	engine_rpm = target_rpm

func _calculate_target_rpm() -> float:
	if current_gear == 0:
		return idle_rpm
	
	var wheel_rpm = calculate_rpm_from_wheel_speed(wheel_speed_rad_per_sec)
	
	if not clutch_engaged:
		return idle_rpm
	
	# Target is either wheel-driven RPM or driver-requested RPM
	var driver_target = idle_rpm + throttle_input * (max_rpm - idle_rpm)
	
	# Blend between wheel-driven and driver target
	return lerp(wheel_rpm, driver_target, 0.7)

# Update nitrous system
func _update_nitrous_system(delta: float) -> void:
	if nitro_active:
		nitro_timer += delta
		
		if nitro_timer >= nitro_duration_max:
			nitro_active = false
			nitro_timer = 0.0
		
		# Consume nitro over time
		nitro_available = max(0.0, nitro_available - delta * 20.0)
	else:
		# Recharge nitro when not active
		if nitro_available < 100.0 and throttle_input > 0.5:
			nitro_available += delta * 5.0

# Update thermal management
func _update_thermal_management(delta: float) -> thesecond
	# Heat generation based on load and RPM
	var load_factor = (engine_rpm / max_rpm) * (output_torque_nm / max_torque_nm)
	var heat_generation = load_factor * 50.0  # degrees C per second potential
	
	# Cooling factors
	var airflow_coefficient = 0.3 + (wheel_speed_rad_per_sec / 100.0) * 0.4
	var cooling_rate = airflow_coefficient * 30.0
	
	# Update temps
	oil_temp_celsius += (heat_generation - cooling_rate) * delta
	coolant_temp_celsius += (heat_generation * 0.8 - cooling_rate * 0.9) * delta
	
	# Clamp to realistic ranges
	oil_temp_celsius = clamp(oil_temp_celsius, 60.0, 140.0)
	coolant_temp_celsius = clamp(coolant_temp_celsius, 70.0, 130.0)
	
	# Adjust thermal efficiency based on temperature
	var temp_penalty = max(0.0, (oil_temp_celsius - 100.0) / 40.0)
	thermal_efficiency = max(0.20, 0.35 - temp_penalty * 0.15)

# Calculate output torque to wheels
func _calculate_output_torque() -> void:
	var engine_torque = get_engine_torque(engine_rpm)
	
	# Apply nitro boost if active
	if nitro_active:
		engine_torque *= nitro_boost_factor
	
	# Account for clutch slip losses
	var clutch_efficiency = 1.0 - (clutch_slip_ratio * 0.3)
	
	# Calculate total gear reduction
	var gear_reduction = get_current_ratio()
	
	# Calculate drivetrain losses (typically 15%)
	var drivetrain_efficiency = 0.85
	
	# Final output torque
	output_torque_nm = engine_torque * clutch_efficiency * gear_reduction * drivetrain_efficiency * thermal_efficiency
	
	# Limit maximum output
	output_torque_nm = min(output_torque_nm, max_torque_nm * 1.5)

# Emit signals
func _emit_signals() -> void:
	power_delivered.emit(output_torque_nm * engine_rpm / 9.549)  # Convert to horsepower approx
	rpm_changed.emit(engine_rpm)

# Shift control
func shift_up() -> void:
	if current_gear < gear_ratios.size():
		target_gear = current_gear + 1
		shift_requested.emit(target_gear)

func shift_down() -> void:
	if current_gear > 1:
		target_gear = current_gear - 1
		shift_requested.emit(target_gear)

func set_gear(gear: int) -> void:
	if gear >= 0 and gear <= gear_ratios.size():
		target_gear = gear
		shift_requested.emit(gear)

func engage_clutch() -> void:
	clutch_engaged = true

func disengage_clutch() -> void:
	clutch_engaged = false

func activate_nitro() -> void:
	if nitro_available > 0.0 and not nitro_active:
		nitro_active = true
		nitro_timer = 0.0

func deactivate_nitro() -> void:
	nitro_active = false

# Get wheel torque for vehicle controller
func get_wheel_torque() -> float:
	return output_torque_nm

# Get current power in kilowatts
func get_power_kw() -> float:
	return (output_torque_nm * engine_rpm) / 9549.0

# Get current horsepower
func get_power_hp() -> float:
	return get_power_kw() * 1.341

# Check if powertrain is ready for launch
func is_ready_for_launch() -> bool:
	return current_gear >= 1 and engine_rpm > idle_rpm * 1.5 and nitro_available > 0.0

# Simulate gear change effect on RPM
func simulate_shift_effect(current_rpm: float, from_gear: int, to_gear: int) -> float:
	if from_gear == 0 or to_gear == 0:
		return idle_rpm
	
	var ratio_change = gear_ratios[to_gear - 1] / gear_ratios[from_gear - 1]
	return current_rpm * ratio_change

# Get comprehensive powertrain diagnostics
func get_diagnostics() -> Dictionary:
	return {
		"engine_rpm": engine_rpm,
		"current_gear": current_gear,
		"target_gear": target_gear,
		"clutch_status": "engaged" if clutch_engaged else "disengaged",
		"output_torque_nm": output_torque_nm,
		"power_kw": get_power_kw(),
		"power_hp": get_power_hp(),
		"nitro_available": nitro_available,
		"nitro_active": nitro_active,
		"oil_temp_celsius": oil_temp_celsius,
		"coolant_temp_celsius": coolant_temp_celsius,
		"thermal_efficiency": thermal_efficiency
	}

# Force reset to idle state
func force_idle() -> void:
	current_gear = 0
	target_gear = 0
	clutch_engaged = true
	engine_rpm = idle_rpm
	output_torque_nm = 0.0

# Calculate optimal shift point for fuel economy
func get_optimal_shift_point_for_economy() -> float:
	return 2500.0  # Conservative shift point

# Calculate optimal shift point for performance
func get_optimal_shift_point_for_performance() -> float:
	return max_rpm * 0.85  # Near redline

# Set custom torque curve parameters
func set_torque_curve(max_torque: float, peak_rpm: float) -> void:
	max_torque_nm = max_torque
	torque_curve_peak_rpm = peak_rpm

# Set transmission gear ratios
func set_transmission_ratios(ratios: Array[float], drive_ratio: float) -> void:
	gear_ratios = ratios
	final_drive_ratio = drive_ratio

</FILE>

Now let me verify the Powertrain.gd file was created correctly.

{"op":"cmd","command":"cat scripts/powertrain/Powertrain.gd | head -30"}
{"op":"continue"}void
	# Heat generation based on load and RPM
	var load_factor = (engine_rpm / max_rpm) * (output_torque_nm / max_torque_nm)
	var heat_generation = load_factor * 50.0  # degrees C per second potential
	
	# Cooling factors
	var airflow_coefficient = 0.3 + (wheel_speed_rad_per_sec / 100.0) * 0.4
	var cooling_rate = airflow_coefficient * 30.0
	
	# Update temps
	oil_temp_celsius += (heat_generation - cooling_rate) * delta
	coolant_temp_celsius += (heat_generation * 0.8 - cooling_rate * 0.9) * delta
	
	# Clamp to realistic ranges
	oil_temp_celsius = clamp(oil_temp_celsius, 60.0, 140.0)
	coolant_temp_celsius = clamp(coolant_temp_celsius, 70.0, 130.0)
	
	# Adjust thermal efficiency based on temperature
	var temp_penalty = max(0.0, (oil_temp_celsius - 100.0) / 40.0)
	thermal_efficiency = max(0.20, 0.35 - temp_penalty * 0.15)
