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
signal drift_started(drift_angle: float)
signal drift_ended()
signal lap_completed(lap_time: float)
signal checkpoint_passed(checkpoint_id: int)
signal boost_used(duration: float)

# ============================================================================
# CONSTANTS & CONFIGURATION
# ============================================================================
const WHEEL_COUNT := 4
const MAX_STEERING_ANGLE := PI / 4  # 45 degrees max
const MIN_SPEED_TO_SHIFT := 5.0     # m/s minimum for safe shifting
const GEAR_RATIOS := [3.8, 2.5, 1.7, 1.3, 1.0, 0.8]  # forward gears
const REVERSE_GEAR := -1.0
const FINAL_DRIVE := 4.1
const ENGINE_IDLE_RPM := 800.0
const ENGINE_MAX_RPM := 7500.0
const CLUTCH_DISENGAGE_RPM := 200.0

# ============================================================================
# EXPORTED PROPERTIES (for inspector configuration)
# ============================================================================
@export_group("Vehicle Configuration")
@export var mass: float = 1500.0: set = _set_mass
@export var center_of_mass_offset: Vector3 = Vector3(0, -0.5, 0.2): set = _set_center_of_mass_offset
@export var wheel_base: float = 2.8  # distance between front and rear axles
@export var track_width: float = 1.6  # distance between left and right wheels
@export var tire_radius: float = 0.35

@export_group("Powertrain Settings")
@export var engine_power: float = 250000.0  # watts (approx 335 hp)
@export var engine_torque: float = 500.0    # Nm peak torque
@export var redline: float = 7000.0         # max RPM before limiter
@export var rev_limit_delay: float = 0.1    # seconds to stay at redline

@export_group("Drivetrain Settings")
@export var drivetrain_type: DrivetrainType = DrivetrainType.FWD
@export var differential_type: DifferentialType = DifferentialType.OPEN

@export_group("Suspension Settings")
@export var suspension_stiffness: float = 50000.0
@export var suspension_damping: float = 5000.0
@export var suspension_compression: float = 0.15
@export var suspension_rebound: float = 0.20
@export var suspension_rest_length: float = 0.30

@export_group("Brake Settings")
@export var brake_force: float = 8000.0     # force per brake caliper
@export var brake_pressure_max: float = 10.0 # bar maximum pressure
@export var abs_threshold: float = 0.15      # slip ratio threshold for ABS

@export_group("Traction Control Settings")
@export var tc_enabled: bool = true
@export var tc_slip_threshold: float = 0.20   # max wheel slip percentage
@export var tc_intervention_delay: float = 0.05

@export_group("Drift Settings")
@export var drift_enabled: bool = false
@export var drift_threshold: float = 0.30     # lateral acceleration threshold
@export var drift_recovery_rate: float = 0.95 # recovery factor when not drifting
@export var drift_bonus_multiplier: float = 1.5 # speed bonus multiplier during drift

@export_group("Boost Settings")
@export var boost_enabled: bool = false
@export var boost_capacity: float = 100.0     # percentage capacity
@export var boost_consumption_rate: float = 20.0 # percent per second
@export var boost_power_multiplier: float = 1.8 # power increase while boosting
@export var boost_cooldown_time: float = 3.0  # seconds between boosts

@export_group("AI Settings (if applicable)")
@export var is_ai_vehicle: bool = false
@export var ai_skill_level: int = 5          # 1-10 scale

enum DrivetrainType {
	FWD,
	RWD,
	AWD
}

enum DifferentialType {
	OPEN,
	LOCKING,
	LSD       # Limited Slip Differential
}

# ============================================================================
# INTERNAL STATE VARIABLES
# ============================================================================
# Physics state
var current_speed: float = 0.0              # forward speed in m/s
var current_rpm: float = ENGINE_IDLE_RPM    # engine RPM
var current_gear: int = 0                   # 0=neutral, 1-6=fwd, -1=reverse
var clutch_engaged: bool = true
var clutch_position: float = 1.0            # 0.0=fully disengaged, 1.0=fully engaged

# Input state
var throttle_input: float = 0.0             # 0.0 to 1.0
var brake_input: float = 0.0                # 0.0 to 1.0
var steering_input: float = 0.0             # -1.0 to 1.0
var handbrake_input: bool = false
var shift_up_request: bool = false
var shift_down_request: bool = false
var boost_requested: bool = false

# Wheel state (per wheel)
var wheel_speeds: Array[float] = [0.0, 0.0, 0.0, 0.0]  # angular velocity rad/s
var wheel_slip_ratios: Array[float] = [0.0, 0.0, 0.0, 0.0]
var wheel_contact_forces: Array[Vector3] = []
var wheel_suspension_compression: Array[float] = []

# Drift state
var drift_angle: float = 0.0                # yaw relative to velocity direction
var drift_score: float = 0.0                # accumulated drift points
var drift_duration: float = 0.0             # time in drift mode
var drift_traction_loss: float = 0.0        # temporary traction reduction

# Boost state
var boost_available: float = 100.0          # percentage
var boost_timer: float = 0.0
var boost_cooldown_timer: float = 0.0

# Suspension state
var suspension_heights: Array[float] = [0.0, 0.0, 0.0, 0.0]
var suspension_velocities: Array[float] = [0.0, 0.0, 0.0, 0.0]

# Collision detection
var collision_history: Array[Dictionary] = []
var collision_last_impact: float = 0.0

# Timing
var _physics_timestamp: float = 0.0
var _last_update_time: float = 0.0
var _lap_start_time: float = 0.0
var _current_lap_time: float = 0.0
var _checkpoint_times: Array[float] = []
var _lap_count: int = 0

# Helper arrays for wheel positions
var _wheel_positions: Array[Vector3] = []
var _wheel_contacts: Array[PhysicsRayResult3D] = []

# References
var _powertrain_node: Node = null
var _camera_node: Node3D = null
var _rigid_body: RigidBody3D = null

# ============================================================================
# ENUMS
# ============================================================================
enum CollisionType {
	VEHICLE,
	BARRIER,
	DEBRIS,
	ENVIRONMENT
}

# ============================================================================
# LIFECYCLE METHODS
# ============================================================================
func _ready() -> void:
	_init_wheel_positions()
	_init_suspension_state()
	_connect_signals_to_audio()
	if GameManager.current_state == GameManager.GameState.RACE_ACTIVE:
		_lap_start_time = Time.get_ticks_msec() as float / 1000.0

func _process(_delta: float) -> void:
	_handle_boost_logic(delta)
	_update_drift_state(delta)
	_check_lap_completion()

func _physics_process(delta: float) -> void:
	_physics_timestamp += delta
	_update_physics(delta)
	_apply_inputs(delta)
	_update_wheels(delta)
	_update_engine(delta)
	_update_suspension(delta)
	_sync_visuals(delta)

func _exit_tree() -> void:
	_cleanup_resources()

# ============================================================================
# INITIALIZATION
# ============================================================================
func _init_wheel_positions() -> void:
	var half_track := track_width * 0.5
	var half_wheelbase := wheel_base * 0.5
	
	# Front Left, Front Right, Rear Left, Rear Right
	_wheel_positions = [
		Vector3(-half_track, 0, half_wheelbase),
		Vector3(half_track, 0, half_wheelbase),
		Vector3(-half_track, 0, -half_wheelbase),
		Vector3(half_track, 0, -half_wheelbase)
	]

func _init_suspension_state() -> void:
	suspension_heights.resize(WHEEL_COUNT)
	suspension_velocities.resize(WHEEL_COUNT)
	wheel_contact_forces.resize(WHEEL_COUNT)
	wheel_speeds.resize(WHEEL_COUNT)
	wheel_slip_ratios.resize(WHEEL_COUNT)
	
	for i in range(WHEEL_COUNT):
		suspension_heights[i] = suspension_rest_length
		wheel_contact_forces[i] = Vector3.ZERO

func _connect_signals_to_audio() -> void:
	speed_changed.connect(AudioManager.play_sound.bind("speed_change"))
	rpm_changed.connect(AudioManager.play_sound.bind("engine_rpm_change"))
	gear_changed.connect(AudioManager.play_sound.bind("gear_shift"))
	collision_detected.connect(AudioManager.play_sound.bind("collision_impact"))
	
# ============================================================================
# PHYSICS UPDATE
# ============================================================================
func _update_physics(delta: float) -> void:
	# Update velocity based on input forces
	var gravity := PhysicsSettings.gravity
	velocity.y -= gravity * delta
	
	# Apply movement
	move_and_slide()
	
	# Clamp vertical velocity (grounded check)
	if is_on_floor():
		velocity.y = max(0, velocity.y)
		current_speed = velocity.xz.length()
	else:
		# In air - use forward component only
		var forward := transform.basis.z
		forward.y = 0
		forward = forward.normalized()
		current_speed = velocity.dot(forward)

# ============================================================================
# INPUT HANDLING
# ============================================================================
func _apply_inputs(delta: float) -> void:
	_get_inputs_from_manager()
	_handle_gearing(delta)
	_handle_steering(delta)
	_apply_drive_forces(delta)
	_apply_brake_forces(delta)
	_handle_handbrake(delta)
	_apply_aerodynamic_drag(delta)

func _get_inputs_from_manager() -> void:
	# Get inputs from InputManager singleton
	if InputManager.is_connected("inputs_updated"):
		InputManager.inputs_updated.connect(_on_inputs_updated)
		
	throttle_input = InputManager.get_axis("throttle", 0.0, 1.0)
	brake_input = InputManager.get_axis("brake", 0.0, 1.0)
	steering_input = InputManager.get_axis("steering_left", -1.0, 1.0)
	handbrake_input = InputManager.get_key_pressed("handbrake")
	shift_up_request = InputManager.get_action_pressed("shift_up")
	shift_down_request = InputManager.get_action_pressed("shift_down")
	boost_requested = InputManager.get_action_pressed("boost") if boost_enabled else false

func _handle_gearing(delta: float) -> void:
	# Automatic gear shifting logic
	if not clutch_engaged:
		return
		
	var target_gear := _calculate_target_gear()
	
	if shift_up_request and current_gear < len(GEAR_RATIOS):
		target_gear = min(current_gear + 1, len(GEAR_RATIOS))
	elif shift_down_request and current_gear > 0:
		target_gear = max(current_gear - 1, 0)
	
	if target_gear != current_gear and abs(current_speed - MIN_SPEED_TO_SHIFT) > 1.0:
		_shift_gear(target_gear, delta)

func _calculate_target_gear() -> int:
	if current_rpm >= redline:
		return min(current_gear + 1, len(GEAR_RATIOS))
	elif current_rpm < ENGINE_IDLE_RPM and current_gear > 0:
		return max(current_gear - 1, 0)
	elif current_gear == 0 and current_speed > MIN_SPEED_TO_SHIFT:
		return 1
	elif current_gear > 0 and current_speed < MIN_SPEED_TO_SHIFT:
		return 0
	return current_gear

func _shift_gear(new_gear: int, delta: float) -> void:
	if new_gear == current_gear:
		return
		
	var old_gear := current_gear
	current_gear = new_gear
	
	clutch_engaged = false
	await get_tree().create_timer(0.15).timeout
	clutch_engaged = true
	
	gear_changed.emit(old_gear, new_gear)
	AudioManager.play_sound("gear_shift", {"gear": new_gear})

# ============================================================================
# STEERING MECHANICS
# ============================================================================
func _handle_steering(delta: float) -> void:
	var steer_amount := steering_input * MAX_STEERING_ANGLE
	steering_angle_changed.emit(steer_amount)
	
	# Apply steering influence to front wheels (FWD/AWD) or all wheels (AWD)
	if drivetrain_type == DrivetrainType.FWD or drivetrain_type == DrivetrainType.AWD:
		# Front wheels turn
		var front_wheel_transform := $WheelFrontLeft.global_transform
		front_wheel_transform.basis = front_wheel_transform.basis.scaled(Vector3.ONE * cos(steer_amount))

# ============================================================================
# DRIVE FORCE APPLICATION
# ============================================================================
func _apply_drive_forces(delta: float) -> void:
	if not clutch_engaged:
		return
		
	var drive_torque := _calculate_engine_torque()
	var effective_gear_ratio := _get_current_gear_ratio()
	var wheel_torque := drive_torque * effective_gear_ratio * FINAL_DRIVE
	
	# Distribute torque based on drivetrain type
	match drivetrain_type:
		DrivetrainType.FWD:
			_apply_wheel_torque(0, wheel_torque * 0.5)
			_apply_wheel_torque(1, wheel_torque * 0.5)
		DrivetrainType.RWD:
			_apply_wheel_torque(2, wheel_torque * 0.5)
			_apply_wheel_torque(3, wheel_torque * 0.5)
		DrivetrainType.AWD:
			_apply_wheel_torque(0, wheel_torque * 0.4)
			_apply_wheel_torque(1, wheel_torque * 0.4)
			_apply_wheel_torque(2, wheel_torque * 0.3)
			_apply_wheel_torque(3, wheel_torque * 0.3)

func _calculate_engine_torque() -> float:
	var torque_curve := _get_torque_curve()
	var normalized_rpm := clamp((current_rpm - ENGINE_IDLE_RPM) / (redline - ENGINE_IDLE_RPM), 0.0, 1.0)
	var torque_factor := _lerp_torque_factor(normalized_rpm, torque_curve)
	
	return engine_torque * torque_factor

func _get_torque_curve() -> Array[float]:
	# Simplified torque curve approximation
	return [0.3, 0.6, 0.9, 1.0, 0.95, 0.90, 0.85]

func _lerp_torque_factor(rpm_normalized: float, curve: Array[float]) -> float:
	var index := int(rpm_normalized * (curve.size() - 1))
	var t := rpm_normalized * (curve.size() - 1) - index
	return curve[index] * (1.0 - t) + curve[min(index + 1, curve.size() - 1)] * t

func _get_current_gear_ratio() -> float:
	if current_gear == 0:
		return 0.0
	if current_gear < 0:
		return REVERSE_GEAR
	return GEAR_RATIOS[current_gear - 1]

func _apply_wheel_torque(wheel_index: int, torque: float) -> void:
	if wheel_index < 0 or wheel_index >= WHEEL_COUNT:
		return
	
	var traction_factor := _calculate_traction_factor(wheel_index)
	var effective_torque := torque * traction_factor
	
	# Apply torque to wheel angular velocity
	var wheel_radius := tire_radius
	var angular_acceleration := effective_torque / (mass * wheel_radius * wheel_radius)
	wheel_speeds[wheel_index] += angular_acceleration * 0.016 # approx frame time

# ============================================================================
# BRAKING SYSTEM
# ============================================================================
func _apply_brake_forces(delta: float) -> void:
	if brake_input <= 0.0:
		return
	
	var brake_pressure := brake_input * brake_pressure_max
	var total_brake_force := brake_force * brake_pressure
	
	# Apply brake force to all wheels (ABS will modulate individual wheels)
	for i in range(WHEEL_COUNT):
		var braking_torque := total_brake_force * tire_radius * 0.25 # even distribution
		wheel_speeds[i] -= braking_torque / (mass * tire_radius) * delta
		
		# ABS modulation
		if tc_enabled and wheel_slip_ratios[i] > abs(abs(wheel_slip_ratios[i]) - abs(brake_input)):
			wheel_speeds[i] *= 0.95  # reduce braking slightly

func _calculate_traction_factor(wheel_index: int) -> float:
	# Calculate available traction based on weight distribution and surface
	var grip := 1.0
	
	# Weight transfer calculation
	var longitudinal_weight_transfer := (center_of_mass_offset.z / wheel_base) * current_speed * current_speed * 0.001
	
	if wheel_index < 2:  # Front wheels
		grip *= 1.0 - longitudinal_weight_transfer * 0.5
	else:  # Rear wheels
		grip *= 1.0 + longitudinal_weight_transfer * 0.5
	
	# Surface friction modifier (would come from terrain data in full implementation)
	grip *= 0.9  # default asphalt
	
	return grip

# ============================================================================
# HANDBRAKE MECHANICS
# ============================================================================
func _handle_handbrake(delta: float) -> void:
	if not handbrake_input:
		return
	
	# Handbrake applies extra brake force to rear wheels
	var handbrake_force := brake_force * 2.0
	
	for i in range(2, 4):  # Rear wheels only
		wheel_speeds[i] -= handbrake_force / (mass * tire_radius) * delta
	
	handbrake_toggled.emit(true)
	AudioManager.play_sound("handbrake")

# ============================================================================
# AERODYNAMIC DRAG
# ============================================================================
func _apply_aerodynamic_drag(delta: float) -> void:
	# Simple quadratic drag model
	var drag_coefficient := 0.30  # typical sports car Cd
	var frontal_area := 2.2  # m^2
	var air_density := 1.225  # kg/m^3
	
	var drag_force := 0.5 * air_density * drag_coefficient * frontal_area * current_speed * current_speed
	
	# Apply drag opposite to velocity
	var drag_vector := velocity.normalized() * -drag_force
	velocity += drag_vector * delta / mass

# ============================================================================
# ENGINE MANAGEMENT
# ============================================================================
func _update_engine(delta: float) -> void:
	var target_rpm := _calculate_target_rpm()
	
	# Smooth RPM transition
	current_rpm = lerp(current_rpm, target_rpm, delta * 10.0)
	
	# Check for stall conditions
	if current_rpm < CLUTCH_DISENGAGE_RPM and current_gear != 0:
		current_rpm = ENGINE_IDLE_RPM
		engine_stalled.emit()
	
	# Redline limiter
	if current_rpm >= redline:
		current_rpm = lerp(current_rpm, redline - 200.0, delta * 5.0)
	
	rpm_changed.emit(current_rpm)

func _calculate_target_rpm() -> float:
	if current_gear == 0:
		return ENGINE_IDLE_RPM
	
	var wheel_speed := wheel_speeds[current_gear % 2] if drivetrain_type != DrivetrainType.RWD else wheel_speeds[3]
	var gear_ratio := _get_current_gear_ratio()
	
	var target_rpm := wheel_speed * gear_ratio * FINAL_DRIVE / (2.0 * PI * tire_radius)
	
	return clampf(target_rpm, ENGINE_IDLE_RPM, redline)

# ============================================================================
# WHEEL PHYSICS
# ============================================================================
func _update_wheels(delta: float) -> void:
	# Update wheel speeds based on vehicle velocity and rotation
	var forward_velocity := velocity.xz.length()
	
	for i in range(WHEEL_COUNT):
		# Calculate expected wheel speed based on vehicle motion
		var expected_wheel_speed := forward_velocity / tire_radius
		
		# Apply drive/brake forces
		var actual_speed := wheel_speeds[i]
		
		# Calculate slip ratio
		wheel_slip_ratios[i] = (actual_speed - expected_wheel_speed) / maxf(expected_wheel_speed, 0.1)
		
		# Limit wheel spin
		if abs(actual_speed) > forward_velocity / tire_radius * 1.5:
			actual_speed = lerp(actual_speed, forward_velocity / tire_radius, delta * 5.0)
		
		wheel_speeds[i] = actual_speed

# ============================================================================
# SUSPENSION PHYSICS
# ============================================================================
func _update_suspension(delta: float) -> void:
	# Raycast down from each wheel position
	for i in range(WHEEL_COUNT):
		var ray_origin := global_position + _wheel_positions[i]
		var ray_end := ray_origin + Vector3.DOWN * 2.0
		
		var space_state := get_world_3d().direct_space_state
		var query := PhysicsRayQuery3D.create(ray_origin, ray_end)
		var result := space_state.intersect_ray(query)
		
		if result:
			var compression := suspension_rest_length - result.position.distance_to(ray_origin)
			compression = clampf(compression, 0.0, suspension_compression * 2.0)
			
			suspension_heights[i] = lerp(suspension_heights[i], compression, delta * 10.0)
			
			# Apply suspension force upward
			var spring_force := (suspension_rest_length - suspension_heights[i]) * suspension_stiffness
			var damper_force := suspension_velocities[i] * suspension_damping
			
			var force := Vector3.UP * (spring_force - damper_force)
			apply_central_force(force * delta)

# ============================================================================
# DRIFT MECHANICS
# ============================================================================
func _update_drift_state(delta: float) -> void:
	if not drift_enabled:
		return
	
	# Calculate drift angle (difference between facing direction and velocity direction)
	var facing_direction := transform.basis.z
	var velocity_direction := velocity.normalized()
	
	if velocity_direction.length() < 0.1:
		return
	
	facing_direction.y = 0
	velocity_direction.y = 0
	
	facing_direction = facing_direction.normalized()
	velocity_direction = velocity_direction.normalized()
	
	var dot_product := facing_direction.dot(velocity_direction)
	var cross_product := facing_direction.cross(velocity_direction)
	
	drift_angle = atan2(cross_product.y, dot_product)
	drift_angle = abs(drift_angle)
	
	# Check if entering drift
	if drift_angle > drift_threshold and current_speed > 10.0:
		if not is_drifting():
			_drift_enter()
		else:
			_drift_continue(delta)
	else:
		if is_drifting():
			_drift_exit(delta)

func _drift_enter() -> void:
	is_drifting()
	drift_started.emit(drift_angle)
	AudioManager.play_sound("drift_start")

func _drift_continue(delta: float) -> void:
	drift_duration += delta
	drift_score += delta * 10.0
	
	# Apply drift traction loss
	var traction_loss := min(drift_duration * 0.05, 0.5)
	drift_traction_loss = traction_loss

func _drift_exit(delta: float) -> void:
	if is_drifting():
		drift_ended.emit()
		AudioManager.play_sound("drift_end")
	
	drift_duration = max(0.0, drift_duration - delta * drift_recovery_rate)
	drift_score = max(0.0, drift_score - delta * 20.0)
	drift_traction_loss = max(0.0, drift_traction_loss - delta * 0.1)

func is_drifting() -> bool:
	return drift_duration > 0.5

# ============================================================================
# BOOST SYSTEM
# ============================================================================
func _handle_boost_logic(delta: float) -> void:
	if not boost_enabled:
		return
	
	# Cooldown management
	if boost_cooldown_timer > 0:
		boost_cooldown_timer -= delta
		return
	
	# Charge boost when coasting
	if throttle_input == 0 and brake_input == 0 and current_speed > 20.0:
		if boost_available < 100.0:
			boost_available += delta * 5.0
	
	# Activate boost when requested
	if boost_requested and boost_available > 0:
		_activate_boost(delta)

func _activate_boost(delta: float) -> void:
	if boost_available <= 0:
		return
	
	boost_available -= delta * boost_consumption_rate
	boost_timer += delta
	
	# Apply boost power multiplier temporarily
	var boost_effective_power := engine_power * boost_power_multiplier
	
	# Visual/audio feedback
	AudioManager.play_sound("boost_activate")
	
	if boost_available <= 0:
		boost_cooldown_timer = boost_cooldown_time
		boost_used.emit(boost_timer)

# ============================================================================
# LAP TIMING & CHECKPOINTS
# ============================================================================
func _check_lap_completion() -> void:
	if GameManager.current_state != GameManager.GameState.RACE_ACTIVE:
		return
	
	var current_time := Time.get_ticks_msec() as float / 1000.0
	_current_lap_time = current_time - _lap_start_time
	
	# Check checkpoint system (placeholder - would connect to actual checkpoint node)
	# For now, just emit signal when crossing finish line area

func start_new_lap() -> void:
	_lap_start_time = Time.get_ticks_msec() as float / 1000.0
	_current_lap_time = 0.0
	_checkpoint_times.clear()
	_lap_count += 1

func reset_lap_data() -> void:
	_lap_start_time = 0.0
	_current_lap_time = 0.0
	_lap_count = 0
	_checkpoint_times.clear()

# ============================================================================
# VISUAL SYNC
# ============================================================================
func _sync_visuals(delta: float) -> void:
	# Sync wheel rotations
	for i in range(WHEEL_COUNT):
		var wheel_node := get_node_or_null("Wheel%s%d" % ["Front"/"Rear"[i//2], i%2])
		if wheel_node:
			var rotation := wheel_speeds[i] * delta
			wheel_node.rotate_x(rotation)

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================
func get_vehicle_speed() -> float:
	return current_speed

func get_vehicle_rpm() -> float:
	return current_rpm

func get_vehicle_gear() -> int:
	return current_gear

func is_vehicle_moving() -> bool:
	return current_speed > 0.5

func reset_vehicle() -> void:
	current_speed = 0.0
	current_rpm = ENGINE_IDLE_RPM
	current_gear = 0
	clutch_engaged = true
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	reset_lap_data()

func _set_mass(new_mass: float) -> void:
	mass = maxf(new_mass, 500.0)
	if _rigid_body:
		_rigid_body.mass = mass

func _set_center_of_mass_offset(offset: Vector3) -> void:
	center_of_mass_offset = offset
	if _rigid_body:
		_rigid_body.center_of_mass = center_of_mass_offset

func _cleanup_resources() -> void:
	# Cleanup any allocated resources
	pass

func _on_inputs_updated(inputs: Dictionary) -> void:
	# Handle external input updates
	if "throttle" in inputs:
		throttle_input = inputs["throttle"]
	if "brake" in inputs:
		brake_input = inputs["brake"]
	if "steering" in inputs:
		steering_input = inputs["steering"]

# ============================================================================
# DEBUG & TESTING
# ============================================================================
func debug_print_vehicle_state() -> void:
	print("=== VEHICLE STATE ===")
	print("Speed: %.2f m/s" % current_speed)
	print("RPM: %.0f" % current_rpm)
	print("Gear: %d" % current_gear)
	print("Throttle: %.2f" % throttle_input)
	print("Brake: %.2f" % brake_input)
	print("Steering: %.2f" % steering_input)
	print("Drift Angle: %.2f deg" % radians_to_degrees(drift_angle))
	print("=====================")

</file>