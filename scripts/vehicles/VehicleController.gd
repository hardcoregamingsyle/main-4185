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

# ============================================================================
# GEARBOX SYSTEM
# ============================================================================

var current_gear: int = 0  # 0 = Neutral, 1-6 = Forward gears, -1 = Reverse
var target_gear: int = 0
var gear_shift_progress: float = 0.0  # 0.0 to 1.0 during shift
var is_shifting: bool = false
var shift_duration: float = 0.2  # Seconds per gear change

var _rpm: float = 800.0  # Current engine RPM
var idle_rpm: float = 800.0
var redline_rpm: float = 7500.0
var rev_limiter_active: bool = false

# ============================================================================
# WHEEL CONFIGURATION
# ============================================================================

const NUM_WHEELS: int = 4
var wheel_positions: Array[Vector3] = []
var wheel_radii: Array[float] = [0.33, 0.33, 0.33, 0.33]
var wheel_forces: Array[float] = [0.0, 0.0, 0.0, 0.0]
var wheel_rotation_angles: Array[float] = [0.0, 0.0, 0.0, 0.0]

# Wheel indices: Front Left, Front Right, Rear Left, Rear Right
const WHEEL_FL: int = 0
const WHEEL_FR: int = 1
const WHEEL_RL: int = 2
const WHEEL_RR: int = 3

# ============================================================================
# POWERTRAIN REFERENCE
# ============================================================================

var powertrain: Node = null
var engine_torque: float = 0.0
var engine_power: float = 0.0
var torque_curve: Array[float] = []
var power_curve: Array[float] = []

# ============================================================================
# DRIFT AND SLIDING LOGIC
# ============================================================================

var is_drifting: bool = false
var drift_intensity: float = 0.0  # 0.0 to 1.0
var drift_threshold: float = 0.35  # Lateral acceleration threshold for drift
var grip_loss_factor: float = 1.0  # Multiplier for grip during drift

# ============================================================================
# COLLISION AND DAMAGE
# ============================================================================

var collisions: Array[Dictionary] = []
var damage_level: float = 0.0  # 0.0 to 1.0
var is_damaged: bool = false

# ============================================================================
# TRACK AND LAP DATA
# ============================================================================

var lap_count: int = 0
var current_lap_time: float = 0.0
var best_lap_time: float = 0.0
var track_checkpoint_positions: Array[Vector3] = []
var last_checkpoint_index: int = -1

# ============================================================================
# INTERNAL TIMERS AND HELPERS
# ============================================================================

var _shift_timer: float = 0.0
var _engine_cooldown: float = 0.0
var _drift_timer: float = 0.0
var _input_buffer: Dictionary = {}

func _ready() -> void:
	_process_mode = ProcessModeEnum.ALWAYS
	_init_wheel_positions()
	_load_powertrain()
	_connect_signals()
	_reset_vehicle()

func _init_wheel_positions() -> void:
	"""Initialize wheel positions relative to vehicle center"""
	var track_width: float = 1.5
	var front_offset: float = 1.2
	var rear_offset: float = 1.0
	
	wheel_positions.resize(NUM_WHEELS)
	
	# Front wheels
	wheel_positions[WHEEL_FL] = Vector3(-track_width / 2, -0.3, -front_offset)
	wheel_positions[WHEEL_FR] = Vector3(track_width / 2, -0.3, -front_offset)
	
	# Rear wheels
	wheel_positions[WHEEL_RL] = Vector3(-track_width / 2, -0.3, rear_offset)
	wheel_positions[WHEEL_RR] = Vector3(track_width / 2, -0.3, rear_offset)

func _load_powertrain() -> void:
	"""Load powertrain reference if exists"""
	if get_parent():
		var parent = get_parent()
		if parent.has_method("get_powertrain"):
			powertrain = parent.get_powertrain()
		elif parent is Node:
			for child in parent.get_children():
				if child.is_class("Powertrain"):
					powertrain = child
					break

func _connect_signals() -> void:
	"""Connect to relevant signals"""
	if powertrain:
		powertrain.rpm_changed.connect(_on_rpm_changed)
		powertrain.engine_stalled.connect(_on_engine_stalled)

func _reset_vehicle() -> void:
	"""Reset vehicle to initial state"""
	current_gear = 0
	target_gear = 0
	gear_shift_progress = 0.0
	is_shifting = false
	_rpm = idle_rpm
	is_drifting = false
	drift_intensity = 0.0
	damage_level = 0.0
	lap_count = 0
	current_lap_time = 0.0
	collisions.clear()

func _physics_process(delta: float) -> void:
	"""Main physics update loop"""
	if delta <= 0.0:
		return
	
	_update_input_buffer(delta)
	_calculate_physics(delta)
	_handle_gear_shifting(delta)
	_apply_forces(delta)
	_update_drift_state(delta)
	_handle_collisions(delta)

func _update_input_buffer(delta: float) -> void:
	"""Update and buffer input values"""
	_last_throttle = _throttle_input
	_last_brake = _brake_input
	_last_steering = _steering_input
	
	_input_buffer.throttle = _throttle_input
	_input_buffer.brake = _brake_input
	_input_buffer.steering = _steering_input
	_input_buffer.clutch = _clutch_input
	_input_buffer.handbrake = _handbrake_input

func _calculate_physics(delta: float) -> void:
	"""Calculate vehicle physics based on inputs and current state"""
	# Calculate acceleration/deceleration
	_calculate_longitudinal_acceleration(delta)
	
	# Calculate steering effect
	_calculate_steering(delta)
	
	# Update current speed
	_update_current_speed(delta)
	
	# Update RPM based on gear and speed
	_update_rpm(delta)

func _calculate_longitudinal_acceleration(delta: float) -> void:
	"""Calculate forward/backward acceleration"""
	if not powertrain:
		return
	
	var clutch_effective: float = 1.0 - _clutch_input
	var handbrake_effect: float = _handbrake_input * 0.8
	
	# Engine torque calculation
	engine_torque = powertrain.get_torque(_rpm, current_gear) * clutch_effective
	
	# Apply throttle
	var throttle_force: float = 0.0
	if current_gear > 0 or current_gear < 0:
		throttle_force = engine_torque * _throttle_input * clutch_effective
	
	# Calculate braking force
	var brake_force: float = 0.0
	if _brake_input > 0.0 or _handbrake_input > 0.0:
		brake_force = (_brake_input + handbrake_effect) * powertrain.max_braking_force
	
	# Calculate net acceleration
	var net_force: float = throttle_force - brake_force
	
	# Apply drag and rolling resistance
	var drag_coefficient: float = PhysicsSettings.drag_coefficient
	var rolling_resistance: float = PhysicsSettings.rolling_resistance
	var air_density: float = PhysicsSettings.air_density
	
	var drag_force: float = 0.5 * drag_coefficient * air_density * current_speed * current_speed * powertrain.frontal_area
	var total_resistance: float = drag_force + rolling_resistance * abs(current_speed)
	
	net_force -= total_resistance
	
	# Calculate acceleration (F = ma)
	var mass: float = powertrain.vehicle_mass
	acceleration = net_force / mass
	longitudinal_acceleration = acceleration / 9.81  # Convert to G-forces
	
	# Clamp acceleration
	acceleration = clamp(acceleration, -PhysicsSettings.max_deceleration, PhysicsSettings.max_acceleration)

func _calculate_steering(delta: float) -> void:
	"""Calculate steering effects on vehicle rotation"""
	if current_speed == 0.0:
		rotation_velocity = _steering_input * PhysicsSettings.max_steering_rate
		return
	
	var effective_steering: float = _steering_input * (1.0 - _handbrake_input * 0.5)
	
	# Steering effectiveness decreases at high speeds
	var speed_factor: float = 1.0 - min(abs(current_speed) / (max_speed * 1.5), 0.5)
	
	# Add some understeer/oversteer characteristics
	var understeer_factor: float = powertrain.understeer_coefficient
	var oversteer_factor: float = powertrain.oversteer_coefficient
	
	# Calculate turn radius
	var wheel_base: float = powertrain.wheelbase
	var steer_angle: float = effective_steering * PhysicsSettings.max_steering_angle * speed_factor
	
	# Yaw rate calculation
	var target_yaw_rate: float = (current_speed * tan(steer_angle)) / wheel_base
	
	# Apply inertia and damping
	rotation_velocity += (target_yaw_rate - rotation_velocity) * delta * 2.0
	
	# Limit rotation velocity
	rotation_velocity = clamp(rotation_velocity, -PhysicsSettings.max_rotation_velocity, PhysicsSettings.max_rotation_velocity)
	
	# Calculate slip angle (difference between heading and velocity direction)
	slip_angle = abs(rotation_velocity) * wheel_base / abs(current_speed) if current_speed != 0 else 0.0
	slip_angle = min(slip_angle, PI / 4)  # Cap at 45 degrees

func _update_current_speed(delta: float) -> void:
	"""Update current speed based on acceleration"""
	# Apply acceleration
	var new_speed: float = current_speed + acceleration * delta
	
	# Handle direction changes
	if abs(new_speed) < 0.5:
		new_speed = 0.0
	
	# Apply speed limits
	if new_speed > 0:
		new_speed = min(new_speed, max_speed)
	else:
		new_speed = max(new_speed, -reverse_speed)
	
	# Smooth speed transition
	current_speed = lerp(current_speed, new_speed, 0.1)
	
	# Emit signal if changed significantly
	if abs(current_speed - _last_speed) > 0.5:
		speed_changed.emit(current_speed)
		_last_speed = current_speed

var _last_speed: float = 0.0

func _update_rpm(delta: float) -> void:
	"""Update engine RPM based on gear and speed"""
	if not powertrain or current_gear == 0:
		_rpm = idle_rpm
		return
	
	var gear_ratio: float = powertrain.gear_ratios[current_gear - 1] if current_gear > 0 else 1.0
	var final_drive: float = powertrain.final_drive_ratio
	var wheel_radius: float = powertrain.wheel_radius
	
	# Calculate theoretical RPM from speed
	var wheel_speed: float = abs(current_speed) / wheel_radius
	var transmission_ratio: float = gear_ratio * final_drive
	var theoretical_rpm: float = wheel_speed * transmission_ratio * 60.0  # Convert to RPM
	
	# Idle RPM when coasting
	if _throttle_input == 0.0 and _brake_input == 0.0 and not is_shifting:
		theoretical_rpm = idle_rpm
	
	# Rev limiter
	if theoretical_rpm > redline_rpm:
		rev_limiter_active = true
		_rpm = lerp(_rpm, redline_rpm * 0.95, delta * 10.0)
	else:
		rev_limiter_active = false
		_rpm = lerp(_rpm, theoretical_rpm, delta * 5.0)
	
	rpm_changed.emit(_rpm)

func _handle_gear_shifting(delta: float) -> void:
	"""Handle automatic/manual gear shifting logic"""
	if not powertrain:
		return
	
	# Check if we're currently shifting
	if is_shifting:
		_shift_timer += delta
		gear_shift_progress = _shift_timer / shift_duration
		
		if gear_shift_progress >= 1.0:
			_complete_gear_shift()
		return
	
	# Automatic gear shifting logic
	target_gear = _calculate_target_gear()
	
	# Manual override check
	if _clutch_input > 0.5:
		# Manual gear selection
		if _input_buffer.manual_gear != current_gear:
			_request_gear_change(_input_buffer.manual_gear)
	else:
		# Auto shift request
		if target_gear != current_gear:
			_request_gear_change(target_gear)

func _calculate_target_gear() -> int:
	"""Calculate optimal gear based on speed and RPM"""
	if current_speed > 0:
		# Forward gears
		for i in range(len(powertrain.gear_ratios)):
			var gear = i + 1
			var gear_ratio: float = powertrain.gear_ratios[i]
			var final_drive: float = powertrain.final_drive_ratio
			var wheel_radius: float = powertrain.wheel_radius
			
			# Calculate RPM for this gear at current speed
			var wheel_speed: float = current_speed / wheel_radius
			var theoretical_rpm: float = wheel_speed * gear_ratio * final_drive * 60.0
			
			if theoretical_rpm < redline_rpm * 0.8:
				return gear
		
		return len(powertrain.gear_ratios)  # Highest gear
	else:
		return 0  # Neutral when stopped

func _request_gear_change(new_gear: int) -> void:
	"""Request a gear change"""
	if new_gear == current_gear:
		return
	
	if is_shifting:
		return
	
	var old_gear: int = current_gra
	current_gear = new_gear
	target_gear = new_gear
	is_shifting = true
	_shift_timer = 0.0
	gear_shift_progress = 0.0
	
	# Engage neutral briefly during shift
	if new_gear != 0:
		gear_changed.emit(old_gear, 0)
		await get_tree().create_timer(0.05).timeout
		gear_shift_progress = 1.0

func _complete_gear_shift() -> void:
	"""Complete the current gear shift"""
	is_shifting = false
	gear_shift_progress = 0.0
	_shift_timer = 0.0
	gear_changed.emit(target_gear, current_gear)

func _apply_forces(delta: float) -> void:
	"""Apply calculated forces to vehicle body"""
	if powertrain:
		# Calculate wheel forces
		_calculate_wheel_forces()
		
		# Apply drive forces to rear wheels
		if current_gear > 0 or current_gear < 0:
			var rear_force: float = wheel_forces[WHEEL_RL] * (1.0 - gear_shift_progress)
			_apply_wheel_force(WHEEL_RL, rear_force)
			_apply_wheel_force(WHEEL_RR, rear_force)
			
			# Differential effect (slight difference left/right)
			if abs(rotation_velocity) > 0.1:
				var diff_factor: float = sign(rotation_velocity) * 0.1
				_apply_wheel_force(WHEEL_RL, rear_force * (1.0 + diff_factor))
				_apply_wheel_force(WHEEL_RR, rear_force * (1.0 - diff_factor))
		
		# Apply steering to front wheels
		_apply_steering_to_wheels()
		
		# Move vehicle
		_move_vehicle(delta)

func _calculate_wheel_forces() -> void:
	"""Calculate individual wheel forces"""
	if not powertrain:
		return
	
	var drive_force: float = engine_torque * powertrain.differential_ratio * 0.5
	
	# Distribute force between rear wheels
	wheel_forces[WHEEL_RL] = drive_force * (1.0 + rotation_velocity * 0.1)
	wheel_forces[WHEEL_RR] = drive_force * (1.0 - rotation_velocity * 0.1)
	
	# Braking forces
	if _brake_input > 0.0 or _handbrake_input > 0.0:
		var brake_distribution: float = powertrain.brake_bias_front
		var total_brake_force: float = (_brake_input + _handbrake_input * 0.5) * powertrain.max_braking_force
		
		wheel_forces[WHEEL_FL] = -total_brake_force * brake_distribution * 0.5
		wheel_forces[WHEEL_FR] = -total_brake_force * brake_distribution * 0.5
		wheel_forces[WHEEL_RL] -= total_brake_force * (1.0 - brake_distribution) * 0.5
		wheel_forces[WHEEL_RR] -= total_brake_force * (1.0 - brake_distribution) * 0.5
	
	# Rolling resistance
	var rolling_resistance: float = powertrain.vehicle_mass * 9.81 * powertrain.rolling_resistance
	for i in range(NUM_WHEELS):
		wheel_forces[i] -= rolling_resistance * 0.25

func _apply_wheel_force(wheel_index: int, force: float) -> void:
	"""Apply force to specific wheel"""
	if wheel_index < 0 or wheel_index >= NUM_WHEELS:
		return
	
	wheel_forces[wheel_index] = force

func _apply_steering_to_wheels() -> void:
	"""Apply steering rotation to front wheels"""
	var steering_angle: float = _steering_input * powertrain.max_steering_angle
	
	wheel_rotation_angles[WHEEL_FL] = steering_angle
	wheel_rotation_angles[WHEEL_FR] = steering_angle

func _move_vehicle(delta: float) -> void:
	"""Move vehicle based on velocity and rotation"""
	# Get forward direction
	var forward: Vector3 = transform.basis.z
	var up: Vector3 = Vector3.UP
	
	# Calculate velocity vector
	var velocity_vector: Vector3 = forward * current_speed
	
	# Apply yaw rotation
	var yaw_delta: float = rotation_velocity * delta
	transform.origin.y += velocity_vector.y * delta
	
	# Rotate around Y axis
	var rotation_matrix: Basis = Basis()
	rotation_matrix = rotation_matrix.rotated(Vector3.UP, yaw_delta)
	
	# Combine movement
	var new_position: Vector3 = transform.origin + velocity_vector
	var new_basis: Basis = transform.basis.rotated(Vector3.UP, yaw_delta)
	
	# Apply position and rotation
	transform.origin = new_position
	transform.basis = new_basis

func _update_drift_state(delta: float) -> void:
	"""Update drift and sliding state"""
	# Calculate lateral acceleration
	lateral_acceleration = abs(rotation_velocity) * current_speed / 9.81
	
	# Check drift conditions
	var drift_condition: bool = (abs(_handbrake_input) > 0.5 or 
	                             abs(slip_angle) > drift_threshold or
	                             lateral_acceleration > drift_threshold)
	
	if drift_condition and not is_drifting:
		is_drifting = true
		drift_intensity = 0.0
		drift_started.emit()
	
	if not drift_condition and is_drusting:
		is_drifting = false
		drift_intensity = 0.0
		drift_ended.emit()
	
	if is_drifting:
		drift_intensity = lerp(drift_intensity, 1.0, delta * 2.0)
		_drift_timer += delta
		
		# Reduce grip during drift
		grip_loss_factor = 1.0 - drift_intensity * 0.6
	else:
		drift_intensity = lerp(drift_intensity, 0.0, delta * 3.0)
		grip_loss_factor = 1.0

func _handle_collisions(delta: float) -> void:
	"""Handle collision detection and response"""
	var collision_shape: Shape3D = $CollisionShape3D.shape
	if collision_shape:
		var shape_transform: Transform3D = $CollisionShape3D.global_transform
		var collider: Area3D = $Area3D
		
		if collider:
			var bodies: Array[Node3D] = collider.get_overlapping_bodies()
			
			for body in bodies:
				var collision_info: Dictionary = {
					"body": body,
					"timestamp": Time.get_ticks_msec(),
					"relative_velocity": body.linear_velocity - linear_velocity,
					"is_player": body.name.contains("Player")
				}
				
				collisions.append(collision_info)
				collision_detected.emit(collision_info)
				
				# Apply impact force
				var impact_force: float = collision_info["relative_velocity"].magnitude()
				if impact_force > 5.0:
					damage_level = min(damage_level + impact_force * 0.01, 1.0)
					is_damaged = true

func _set_throttle_input(value: float) -> void:
	"""Set throttle input with smoothing"""
	_throttle_input = clamp(value, 0.0, 1.0)
	_input_buffer.throttle = _throttle_input

func _set_brake_input(value: float) -> void:
	"""Set brake input with smoothing"""
	_brake_input = clamp(value, 0.0, 1.0)
	_input_buffer.brake = _brake_input

func _set_steering_input(value: float) -> void:
	"""Set steering input with smoothing"""
	_steering_input = clamp(value, -1.0, 1.0)
	_input_buffer.steering = _steering_input

func _set_clutch_input(value: float) -> void:
	"""Set clutch input with smoothing"""
	_clutch_input = clamp(value, 0.0, 1.0)
	_input_buffer.clutch = _clutch_input

func _set_handbrake_input(value: float) -> void:
	"""Set handbrake input with smoothing"""
	_handbrake_input = clamp(value, 0.0, 1.0)
	_input_buffer.handbrake = _handbrake_input

func _on_rpm_changed(rpm_value: float) -> void:
	"""Handle RPM changes from powertrain"""
	_rpm = rpm_value
	rpm_changed.emit(_rpm)

func _on_engine_stalled() -> void:
	"""Handle engine stall event"""
	_rpm = idle_rpm
	current_gear = 0
	engine_stalled.emit()

func get_speed_kmh() -> float:
	"""Get speed in km/h"""
	return current_speed * 3.6

func get_speed_mph() -> float:
	"""Get speed in mph"""
	return current_speed * 2.23694

func reset_lap() -> void:
	"""Reset lap timing data"""
	lap_count = 0
	current_lap_time = 0.0
	best_lap_time = 0.0
	last_checkpoint_index = -1
	track_checkpoint_positions.clear()

func record_checkpoint(index: int) -> void:
	"""Record passing through checkpoint"""
	if index > last_checkpoint_index:
		if index == len(track_checkpoint_positions) - 1 and last_checkpoint_index >= 0:
			# Completed lap
			record_lap_completion()
		last_checkpoint_index = index

func record_lap_completion() -> void:
	"""Record completed lap"""
	lap_count += 1
	if best_lap_time == 0.0 or current_lap_time < best_lap_time:
		best_lap_time = current_lap_time
	current_lap_time = 0.0
	lap_completed.emit({"lap": lap_count, "time": current_lap_time})

func set_track_checkpoints(checkpoints: Array[Vector3]) -> void:
	"""Set track checkpoint positions"""
	track_checkpoint_positions = checkpoints

func apply_damage(amount: float) -> void:
	"""Apply damage to vehicle"""
	damage_level = min(damage_level + amount, 1.0)
	is_damaged = true
	
	if damage_level >= 1.0:
		_engine_stall()

func _engine_stall() -> void:
	"""Stall the engine due to severe damage"""
	current_gear = 0
	_rpm = 0.0
	is_shifting = false
	engine_stalled.emit()

func repair_vehicle(amount: float) -> void:
	"""Repair vehicle damage"""
	damage_level = max(damage_level - amount, 0.0)
	is_damaged = damage_level > 0.0

func get_vehicle_status() -> Dictionary:
	"""Get comprehensive vehicle status"""
	return {
		"speed": current_speed,
		"speed_kmh": get_speed_kmh(),
		"rpm": _rpm,
		"gear": current_gear,
		"throttle": _throttle_input,
		"brake": _brake_input,
		"steering": _steering_input,
		"drifting": is_drifting,
		"damage": damage_level,
		"laps": lap_count,
		"best_lap": best_lap_time
	}

func _process(delta: float) -> void:
	"""Regular process updates (UI, non-physics)"""
	pass

func _input(event: InputEvent) -> void:
	"""Handle input events"""
	if event is InputEventKey:
		if event.pressed:
			match event.keycode:
				KEY_1, KEY_2, KEY_3, KEY_4, KEY_5, KEY_6:
					var gear_num: int = event.keycode - KEY_1 + 1
					_input_buffer.manual_gear = gear_num
				KEY_N:
					_input_buffer.manual_gear = 0

func _exit_tree() -> void:
	"""Cleanup on exit"""
	powertrain = null
</FILE>

Now let me verify the file was written correctly by checking its contents:

{"op":"cmd","command":"cat scripts/vehicles/VehicleController.gd | wc -l"}