extends Node
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
signal lap_complete(lap_time: float)

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
const WHEEL_RADIUS: float = 0.3                # meters
const GEAR_CHANGE_DELAY: float = 0.1           # seconds between gear changes

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
    Gear.REVERSE: 3.5
}

const FINAL_DRIVE_RATIO: float = 3.73          # Final drive differential ratio
const EFFICIENCY_FACTOR: float = 0.85          # Powertrain efficiency factor

# ============================================================================
# VEHICLE STATE VARIABLES
# ============================================================================

@export var vehicle_mass: float = 1500.0        # kg
@export var wheelbase: float = 2.5              # meters
@export var track_width: float = 1.5            # meters
@export var center_of_gravity_height: float = 0.5  # meters above ground
@export var drag_coefficient: float = 0.32     # Cd for aerodynamic drag
@export var frontal_area: float = 2.2          # m² for air resistance

var current_speed: float = 0.0                   # m/s (positive forward, negative reverse)
var current_rpm: float = MIN_RPM_IDLE
var current_gear: int = Gear.NEUTRAL
var target_gear: int = Gear.NEUTRAL
var steering_angle: float = 0.0                  # radians
var target_steering_angle: float = 0.0           # radians

var throttle_input: float = 0.0                  # 0.0 to 1.0
var brake_input: float = 0.0                     # 0.0 to 1.0
var steering_input: float = 0.0                  # -1.0 to 1.0

var drift_angle: float = 0.0                     # angle between velocity vector and heading
var drift_mode: bool = false
var traction_control_on: bool = true
var abs_active: bool = false

var last_position: Vector2 = Vector2.ZERO
var displacement_since_last_update: Vector2 = Vector2.ZERO
var total_distance: float = 0.0
var lap_start_time: float = 0.0
var lap_times: Array[float] = []

# Powertrain simulation
var engine_torque: float = 0.0                   # Nm at wheels
var clutch_engaged: bool = false
var gearbox_ratio: float = 1.0

# Wheel state for suspension simulation
var wheel_forces: Array[float] = [0.0, 0.0, 0.0, 0.0] # Front-L, Front-R, Rear-L, Rear-R
var suspension_compression: Array[float] = [0.0, 0.0, 0.0, 0.0]
var wheel_rotation_angles: Array[float] = [0.0, 0.0, 0.0, 0.0]

# Simulation state
var _is_paused: bool = false
var _gear_change_timer: float = 0.0
var _last_update_time: float = 0.0
var _physics_settings: PhysicsSettings = PhysicsSettings.new()
var _power_scale: float = 1.0

# ============================================================================
# INITIALIZATION AND SETUP
# ============================================================================

func _ready() -> void:
	_process_mode = ProcessModeEnum.ALWAYS
	_init_physics_settings()
	_setup_powertrain()
	reset_vehicle_state()

func _init_physics_settings() -> void:
	"""Load physics settings from centralized resource"""
	if Engine.has_singleton("PhysicsSettings"):
		var ps_node = Engine.get_singleton("PhysicsSettings")
		if ps_node != null:
			_physics_settings.gravity = ps_node.gravity
			_physics_settings.physics_tick_rate = ps_node.physics_tick_rate
			_physics_settings.max_substeps = ps_node.max_substeps

func _setup_powertrain() -> void:
	"""Initialize powertrain components"""
	gearbox_ratio = 1.0
	clutch_engaged = true
	wheel_forces.resize(4)
	suspension_compression.resize(4)
	wheel_rotation_angles.resize(4)

func reset_vehicle_state() -> void:
	"""Reset vehicle to initial state"""
	current_speed = 0.0
	current_rpm = MIN_RPM_IDLE
	current_gear = Gear.NEUTRAL
	target_gear = Gear.NEUTRAL
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	steering_angle = 0.0
	target_steering_angle = 0.0
	displacement_since_last_update = Vector2.ZERO
	total_distance = 0.0
	lap_start_time = 0.0
	lap_times.clear()

# ============================================================================
# INPUT HANDLING
# ============================================================================

func process_inputs(delta: float) -> void:
	"""Process player inputs and update vehicle controls"""
	if _is_paused:
		return
	
	# Get inputs from InputManager singleton
	var input_manager = GameManager.get_instance().get_node_or_null("/root/InputManager")
	if input_manager == null:
		input_manager = get_tree().root.get_node_or_null("InputManager")
	
	if input_manager != null:
		throttle_input = input_manager.get_axis("throttle", 0.0)
		brake_input = input_manager.get_axis("brake", 0.0)
		steering_input = input_manager.get_axis("steer", 0.0)
	else:
		# Fallback direct input check
		throttle_input = Input.get_action_strength("vehicle_throttle")
		brake_input = Input.get_action_strength("vehicle_brake")
		steering_input = Input.get_axis("vehicle_steer_left", "vehicle_steer_right")
	
	# Clamp inputs
	throttle_input = clamp(throttle_input, 0.0, 1.0)
	brake_input = clamp(brake_input, 0.0, 1.0)
	steering_input = clamp(steering_input, -1.0, 1.0)
	
	# Apply steering smoothing
	target_steering_angle = steering_input * MAX_STEERING_ANGLE
	steering_angle = lerp(steering_angle, target_steering_angle, delta * STEERING_SPEED)
	
	# Handle gear shifting
	_handle_gear_shifting()

func _handle_gear_shifting() -> void:
	"""Automatic or manual gear shifting logic"""
	if _is_paused or current_gear == Gear.NEUTRAL:
		return
	
	# Calculate target gear based on speed and throttle
	if current_speed > 0:
		target_gear = _calculate_optimal_gear(current_speed, throttle_input)
	else:
		target_gear = Gear.REVERSE if brake_input > 0 else Gear.FIRST
	
	# Check if gear change is needed
	if abs(target_gear - current_gear) > 0 and _gear_change_timer <= 0:
		_request_gear_change(target_gear)

func _calculate_optimal_gear(speed: float, throttle: float) -> int:
	"""Calculate optimal gear based on current speed and throttle demand"""
	var wheel_angular_velocity = abs(speed) / WHEEL_RADIUS
	var effective_rpm = _calculate_engine_rpm(wheel_angular_velocity, Gear.FIRST)
	
	# Determine target gear based on RPM and throttle
	if throttle < 0.2:
		# Light throttle - higher gears
		if speed < 5.0: return Gear.FIRST
		elif speed < 15.0: return Gear.SECOND
		elif speed < 25.0: return Gear.THIRD
		elif speed < 35.0: return Gear.FOURTH
		elif speed < 50.0: return Gear.FIFTH
		else: return Gear.SIXTH
	else:
		# Heavy throttle - lower gears for acceleration
		if speed < 8.0: return Gear.FIRST
		elif speed < 20.0: return Gear.SECOND
		elif speed < 35.0: return Gear.THIRD
		elif speed < 50.0: return Gear.FOURTH
		elif speed < 70.0: return Gear.FIFTH
		else: return Gear.SIXTH

func _request_gear_change(new_gear: int) -> void:
	"""Request a gear change with delay protection"""
	if new_gear == current_gear:
		return
	
	# Prevent rapid-fire gear changes
	if _gear_change_timer > 0:
		return
	
	# Prevent unsafe shifts (e.g., reverse while moving fast)
	if (current_gear in [Gear.FIRST, Gear.SECOND, Gear.THIRD, Gear.FOURTH, Gear.FIFTH, Gear.SIXTH]) and new_gear == Gear.REVERSE:
		if current_speed > 5.0:
			return
	if new_gear in [Gear.FIRST, Gear.SECOND, Gear.THIRD, Gear.FOURTH, Gear.FIFTH, Gear.SIXTH] and current_gear == Gear.REVERSE:
		if current_speed < -2.0:
			return
	
	# Execute gear change
	var old_gear = current_gear
	current_gear = new_gear
	target_gear = new_gear
	
	# Update gearbox ratio
	_update_gearbox_ratio()
	
	# Emit signal
	emit_signal("gear_changed", old_gear, current_gear)
	
	# Start gear change timer
	_gear_change_timer = GEAR_CHANGE_DELAY

func _update_gearbox_ratio() -> void:
	"""Update gearbox ratio based on current gear"""
	if current_gear in GEAR_RATIOS:
		gearbox_ratio = GEAR_RATIOS[current_gear] * FINAL_DRIVE_RATIO
	else:
		gearbox_ratio = 1.0

# ============================================================================
# ENGINE AND POWERTRAIN SIMULATION
# ============================================================================

func calculate_engine_output() -> void:
	"""Calculate engine torque output based on RPM and throttle"""
	if current_gear == Gear.NEUTRAL or not clutch_engaged:
		engine_torque = 0.0
		return
	
	# Simulate engine torque curve
	var torque_curve = _get_engine_torque_curve(current_rpm)
	
	# Apply throttle influence
	var applied_throttle = throttle_input * 1.0
	
	# Calculate actual torque
	engine_torque = torque_curve * applied_throttle * EFFICIENCY_FACTOR
	
	# Apply traction control if active
	if traction_control_on:
		engine_torque = _apply_traction_control(engine_torque)
	
	# Cap at redline
	if current_rpm >= MAX_RPM_REDLINE:
		engine_torque *= 0.5  # Power reduction near redline

func _get_engine_torque_curve(rpm: float) -> float:
	"""Get simulated engine torque at given RPM"""
	var normalized_rpm = (rpm - MIN_RPM_IDLE) / (MAX_RPM_REDLINE - MIN_RPM_IDLE)
	
	# Simple torque curve model (peak around 4000-5000 RPM)
	var peak_rpm = 4500.0
	var torque_peak = 400.0  # Peak torque in Nm
	
	if rpm <= MIN_RPM_IDLE:
		return 0.0
	elif rpm <= peak_rpm:
		# Rising torque curve
		var progress = (rpm - MIN_RPM_IDLE) / (peak_rpm - MIN_RPM_IDLE)
		return torque_peak * (0.5 + 0.5 * sin(progress * PI / 2))
	else:
		# Falling torque curve past peak
		var progress = (rpm - peak_rpm) / (MAX_RPM_REDLINE - peak_rpm)
		return torque_peak * (0.8 - 0.3 * progress)

func _apply_traction_control(torque: float) -> float:
	"""Apply traction control to prevent wheel spin"""
	if not traction_control_on:
		return torque
	
	# Detect slip based on sideslip angle
	var slip_ratio = abs(drift_angle)
	
	if slip_ratio > DRIFT_THRESHOLD:
		# Reduce torque when slipping too much
		var reduction = (slip_ratio - DRIFT_THRESHOLD) * 2.0
		reduction = min(reduction, 1.0)
		return torque * (1.0 - reduction * 0.5)
	
	return torque

func update_engine_rpm(delta: float) -> void:
	"""Update engine RPM based on vehicle dynamics"""
	if current_gear == Gear.NEUTRAL:
		# Engine idle or revving freely
		if throttle_input > 0.1:
			current_rpm = lerp(current_rpm, MIN_RPM_IDLE + throttle_input * (MAX_RPM_REDLINE - MIN_RPM_IDLE), delta * 10.0)
		else:
			current_rpm = lerp(current_rpm, MIN_RPM_IDLE, delta * 5.0)
	else:
		# Engine coupled to wheels
		var wheel_angular_velocity = abs(current_speed) / WHEEL_RADIUS
		current_rpm = _calculate_engine_rpm(wheel_angular_velocity, current_gear)
	
	# Smooth RPM transitions
	current_rpm = clamp(current_rpm, MIN_RPM_IDLE, MAX_RPM_REDLINE + 500.0)
	emit_signal("engine_rpm_changed", current_rpm)

func _calculate_engine_rpm(wheel_angular_velocity: float, gear: int) -> float:
	"""Calculate engine RPM from wheel angular velocity"""
	if gear == Gear.NEUTRAL:
		return MIN_RPM_IDLE
	
	var gear_ratio = GEAR_RATIOS.get(gear, 1.0)
	var final_ratio = gear_ratio * FINAL_DRIVE_RATIO
	
	# Engine RPM = wheel angular velocity * gear ratio * final drive
	var calculated_rpm = wheel_angular_velocity * final_ratio * 60.0 / (2.0 * PI)
	
	return calculated_rpm

# ============================================================================
# VEHICLE DYNAMICS AND MOVEMENT
# ============================================================================

func calculate_vehicle_dynamics(delta: float) -> void:
	"""Calculate vehicle dynamics including forces, acceleration, and velocity"""
	if _is_paused:
		return
	
	# Calculate net driving force
	var driving_force = _calculate_driving_force()
	
	# Calculate resistive forces
	var air_drag = _calculate_air_drag()
	var rolling_resistance = _calculate_rolling_resistance()
	var gravitational_force = _calculate_gravitational_force()
	
	# Net force
	var net_force = driving_force - air_drag - rolling_resistance - gravitational_force
	
	# Acceleration (F = ma)
	var acceleration = net_force / vehicle_mass
	
	# Update velocity
	current_speed += acceleration * delta
	
	# Apply friction damping for neutral/no input
	if current_gear == Gear.NEUTRAL and abs(current_speed) > 1.0:
		var friction_deceleration = 0.5
		if current_speed > 0:
			current_speed -= friction_deceleration * delta
		else:
			current_speed += friction_deceleration * delta
	
	# Clamp minimum speed
	current_speed = max(current_speed, -1.0)
	
	# Update position
	_update_position(delta)
	
	# Calculate drift
	_calculate_drift()
	
	# Update wheel states
	_update_wheels(delta)
	
	# Emit signals
	emit_signal("speed_changed", current_speed)
	emit_signal("vehicle_moved", displacement_since_last_update)

func _calculate_driving_force() -> float:
	"""Calculate total driving force from all wheels"""
	if current_gear == Gear.NEUTRAL or not clutch_engaged:
		return 0.0
	
	# Force = Torque / wheel_radius
	var base_force = engine_torque / WHEEL_RADIUS
	
	# Distribute force to rear wheels (RWD simulation)
	var rear_force = base_force * 0.5
	var front_force = 0.0
	
	# Apply brake force reduction
	var brake_factor = 1.0 - brake_input * 0.5
	rear_force *= brake_factor
	
	# Adjust for direction
	if current_speed < 0 and current_gear == Gear.REVERSE:
		return rear_force
	elif current_gear == Gear.REVERSE:
		return -rear_force
	else:
		return rear_force

func _calculate_air_drag() -> float:
	"""Calculate aerodynamic drag force"""
	var air_density = 1.225  # kg/m³ at sea level
	var velocity_squared = current_speed * current_speed
	
	var drag_force = 0.5 * air_density * drag_coefficient * frontal_area * velocity_squared
	
	# Drag opposes motion
	return drag_force if current_speed >= 0 else -drag_force

func _calculate_rolling_resistance() -> float:
	"""Calculate rolling resistance force"""
	var rolling_resistance_coefficient = 0.015  # Typical for performance tires
	var normal_force = vehicle_mass * _physics_settings.gravity
	
	return rolling_resistance_coefficient * normal_force if current_speed >= 0 else -rolling_resistance_coefficient * normal_force

func _calculate_gravitational_force() -> float:
	"""Calculate gravitational force component (simplified)"""
	# In flat terrain, this is zero
	# Could add hill climbing here with slope parameter
	return 0.0

func _update_position(delta: float) -> void:
	"""Update vehicle position based on current velocity"""
	var heading = global_rotation
	
	# Convert speed to velocity vector
	var vx = current_speed * cos(heading)
	var vy = current_speed * sin(heading)
	
	# Position change
	var delta_x = vx * delta
	var delta_y = vy * delta
	
	var new_position = global_position
	new_position.x += delta_x
	new_position.y += delta_y
	
	# Track displacement
	displacement_since_last_update = Vector2(delta_x, delta_y)
	last_position = global_position
	
	# Update total distance
	total_distance += abs(displacement_since_last_update.length())
	
	# Update global position
	global_position = new_position

func _calculate_drift() -> void:
	"""Calculate drift angle and detect drift state"""
	var heading = global_rotation
	
	# Velocity direction
	var velocity_dir = atan2(global_position.y - last_position.y, global_position.x - last_position.x)
	
	# Drift angle is difference between heading and velocity direction
	drift_angle = heading - velocity_dir
	
	# Normalize to -PI to PI
	while drift_angle > PI:
		drift_angle -= 2.0 * PI
	while drift_angle < -PI:
		drift_angle += 2.0 * PI
	
	# Detect drift mode
	var was_drift_mode = drift_mode
	drift_mode = abs(drift_angle) > DRIFT_THRESHOLD
	
	if drift_mode != was_drift_mode:
		emit_signal("drift_angle_changed", drift_angle)
		traction_control_on = !drift_mode or abs(drift_angle) < TRACTION_CONTROL_SENSITIVITY
		emit_signal("traction_control_active", traction_control_on)

func _update_wheels(delta: float) -> void:
	"""Update wheel rotation and suspension state"""
	var wheel_circumference = 2.0 * PI * WHEEL_RADIUS
	
	# Calculate wheel rotation based on speed
	var wheel_delta = abs(current_speed) * delta / WHEEL_RADIUS
	
	for i in range(4):
		wheel_rotation_angles[i] += wheel_delta * sign(current_speed) if i < 2 else wheel_delta * sign(current_speed)
		
		# Simplified suspension compression (based on vertical forces)
		suspension_compression[i] = lerp(suspension_compression[i], 0.1 * abs(current_speed) / 100.0, delta * 5.0)

# ============================================================================
# BRAKING AND ABS
# ============================================================================

func apply_braking(delta: float) -> void:
	"""Apply braking forces to wheels"""
	if brake_input <= 0.0:
		return
	
	var brake_pressure = brake_input * MAX_BRAKE_FORCE
	
	# Apply to all wheels
	for i in range(4):
		wheel_forces[i] = -brake_pressure
		
		# ABS modulation
		if abs(current_speed) < 2.0 and brake_input > 0.8:
			abs_active = true
			# Modulate brake pressure to prevent lockup
			wheel_forces[i] *= 0.7
		else:
			abs_active = false

# ============================================================================
# COLLISION HANDLING
# ============================================================================

func handle_collision(collider: Node, collision_info: Dictionary) -> void:
	"""Handle vehicle collision events"""
	var impact_speed = abs(current_speed)
	var impact_force = vehicle_mass * impact_speed
	
	# Categorize collision type
	var collision_type = "generic"
	
	if collider.has_method("get_collision_type"):
		collision_type = collider.get_collision_type()
	elif "type" in collision_info:
		collision_type = collision_info["type"]
	
	# Emit signal
	emit_signal("collision_detected", collision_type, impact_force)
	
	# Apply collision response
	_apply_collision_response(impact_force, collision_info)

func _apply_collision_response(force: float, collision_info: Dictionary) -> void:
	"""Apply physics response to collision"""
	var bounce_factor = 0.3  # Restitution coefficient
	var energy_loss = 1.0 - bounce_factor
	
	# Reduce speed based on impact
	current_speed *= energy_loss
	
	# Apply knockback in collision direction
	if "normal" in collision_info:
		var normal = collision_info["normal"]
		var knockback = normal * force * 0.001
		# Could apply velocity kick here

# ============================================================================
# LAP TIMING AND TRACK SYSTEM
# ============================================================================

func start_lap() -> void:
	"""Start lap timing"""
	lap_start_time = Time.get_ticks_msec() / 1000.0
	lap_times.clear()

func record_lap_time() -> void:
	"""Record completed lap time"""
	if lap_start_time > 0:
		var lap_time = Time.get_ticks_msec() / 1000.0 - lap_start_time
		lap_times.append(lap_time)
		emit_signal("lap_complete", lap_time)
		lap_start_time = 0.0

func get_best_lap_time() -> float:
	"""Get best recorded lap time"""
	if lap_times.is_empty():
		return 0.0
	
	var best_time = lap_times[0]
	for t in lap_times:
		if t < best_time:
			best_time = t
	return best_time

func get_average_lap_time() -> float:
	"""Get average lap time"""
	if lap_times.is_empty():
		return 0.0
	
	var total_time = 0.0
	for t in lap_times:
		total_time += t
	return total_time / lap_times.size()

# ============================================================================
# DEBUG AND UTILITIES
# ============================================================================

func get_vehicle_stats() -> Dictionary:
	"""Get comprehensive vehicle statistics"""
	return {
		"speed": current_speed,
		"rpm": current_rpm,
		"gear": current_gear,
		"steering_angle": steering_angle,
		"drift_angle": drift_angle,
		"drift_mode": drift_mode,
		"total_distance": total_distance,
		"best_lap": get_best_lap_time(),
		"average_lap": get_average_lap_time(),
		"laps_completed": lap_times.size(),
		"traction_control_active": traction_control_on,
		"abs_active": abs_active
	}

func set_power_scale(scale: float) -> void:
	"""Set overall power scale for testing or difficulty"""
	_power_scale = scale

func pause_simulation(pause: bool) -> void:
	"""Pause or resume vehicle physics simulation"""
	_is_paused = pause

func reset_to_checkpoint(checkpoint_data: Dictionary) -> void:
	"""Reset vehicle to checkpoint position"""
	if "position" in checkpoint_data:
		global_position = checkpoint_data["position"]
	if "rotation" in checkpoint_data:
		global_rotation = checkpoint_data["rotation"]
	if "speed" in checkpoint_data:
		current_speed = checkpoint_data["speed"]
	if "rpm" in checkpoint_data:
		current_rpm = checkpoint_data["rpm"]
	if "gear" in checkpoint_data:
		current_gear = checkpoint_data["gear"]
	
	displacement_since_last_update = Vector2.ZERO
	last_position = global_position

# ============================================================================
# MAIN PROCESS LOOP
# ============================================================================

func _process(delta: float) -> void:
	"""Main vehicle controller process loop"""
	if Engine.is_editor_hint():
		return
	
	# Skip if paused
	if _is_paused:
		return
	
	# Update time tracking
	if _last_update_time == 0:
		_last_update_time = Time.get_ticks_msec()
	
	# Process inputs
	process_inputs(delta)
	
	# Calculate engine output
	calculate_engine_output()
	
	# Update engine RPM
	update_engine_rpm(delta)
	
	# Calculate vehicle dynamics
	calculate_vehicle_dynamics(delta)
	
	# Apply braking
	apply_braking(delta)
	
	# Decrease gear change timer
	if _gear_change_timer > 0:
		_gear_change_timer -= delta

</file_content>