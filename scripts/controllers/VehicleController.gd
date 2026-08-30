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
@export var mass: float = 1500.0
@export var center_of_mass_offset: Vector3 = Vector3(0, -0.5, 0.2)
@export var wheel_base: float = 2.8  # distance between front and rear axles
@export var track_width: float = 1.6  # distance between left and right wheels
@export var tire_radius: float = 0.35

@export_group("Powertrain Settings")
@export var engine_power: float = 250000.0  # watts (approx 335 hp)
@export var engine_max_torque: float = 450.0  # Nm
@export var drivetrain_type: DrivetrainType = DrivetrainType.FWD  # FWD, RWD, AWD
@export var transmission_type: TransmissionType = TransmissionType.MANUAL

@export_group("Tire & Suspension")
@export var tire_friction_coefficient: float = 1.2
@export var suspension_stiffness: float = 35000.0
@export var suspension_damping: float = 3500.0
@export var suspension_travel: float = 0.15
@export var ride_height: float = 0.3

@export_group("Aerodynamics")
@export var drag_coefficient: float = 0.32
@export var frontal_area: float = 2.2  # m²
@export var downforce_coefficient: float = 0.5
@export var wing_angle: float = PI / 12  # ~15 degrees

@export_group("Brakes")
@export var brake_force_per_wheel: float = 4500.0
@export var parking_brake_force: float = 2000.0
@export var brake_bias_front: float = 0.6  # 60% front brake bias

@export_group("Drift & Traction Control")
@export var drift_threshold: float = 0.3  # lateral grip threshold
@export var drift_recovery_rate: float = 0.92
@export var traction_control_enabled: bool = true
@export var anti_lock_braking_enabled: bool = true
@export var torque_vectoring_enabled: bool = false

# ============================================================================
# ENUMERATIONS
# ============================================================================
enum DrivetrainType {
	FWD,
	RWD,
	AWD
}

enum TransmissionType {
	MANUAL,
	AUTOMATIC,
	SEMI_AUTOMATIC
}

enum WheelType {
	FRONT_LEFT,
	FRONT_RIGHT,
	REAR_LEFT,
	REAR_RIGHT
}

# ============================================================================
# INTERNAL STATE
# ============================================================================
var _current_speed: float = 0.0
var _current_rpm: float = ENGINE_IDLE_RPM
var _current_gear: int = 0  # 0 = neutral
var _target_gear: int = 0
var _throttle_input: float = 0.0  # 0.0 to 1.0
var _brake_input: float = 0.0     # 0.0 to 1.0
var _clutch_input: float = 1.0    # 0.0 = depressed, 1.0 = released
var _steering_input: float = 0.0  # -1.0 to 1.0
var _handbrake_active: bool = false
var _in_drift: bool = false
var _drift_angle: float = 0.0
var _wheel_slip_angles: Array[float] = []
var _wheel_contact_forces: Array[Vector3] = []
var _boost_available: float = 100.0
var _boost_cooldown: float = 0.0
var _lap_start_time: float = 0.0
var _last_checkpoint_id: int = -1
var _lap_times: Array[float] = []
var _total_distance: float = 0.0
var _is_engine_running: bool = true
var _is_in_collision: bool = false
var _collision_damage: float = 0.0

# Wheel positions (local to vehicle body)
var _wheel_positions: Array[Vector3] = []
var _wheel_rotations: Array[float] = []
var _suspension_compressions: Array[float] = []

# Engine characteristics curve data
var _engine_torque_curve: Dictionary = {}
var _engine_power_curve: Dictionary = {}

# ============================================================================
# PROPERTIES
# ============================================================================
func get_current_speed() -> float:
	return _current_speed

func get_current_rpm() -> float:
	return _current_rpm

func get_current_gear() -> int:
	return _current_gear

func get_throttle_input() -> float:
	return _throttle_input

func get_brake_input() -> float:
	return _brake_input

func get_steering_input() -> float:
	return _steering_input

func is_handbrake_active() -> bool:
	return _handbrake_active

func is_in_drift() -> bool:
	return _in_drift

func get_drift_angle() -> float:
	return _drift_angle

func get_boost_available() -> float:
	return _boost_available

func is_engine_running() -> bool:
	return _is_engine_running

func get_total_distance() -> float:
	return _total_distance

func get_lap_times() -> Array[float]:
	return _lap_times

func get_wheel_contact_points() -> Array[Vector3]:
	var contact_points: Array[Vector3] = []
	for i in range(WHEEL_COUNT):
		contact_points.append(global_position + _wheel_positions[i])
	return contact_points

# ============================================================================
# LIFECYCLE
# ============================================================================
func _ready() -> void:
	_init_wheel_configuration()
	_init_engine_curves()
	_connect_signals_to_manager()
	_update_suspension_positions()
	
	# Set physics material
	if collision_shape:
		collision_shape.set_deferred("material", create_physics_material())
		
	print("[VehicleController] Initialized successfully")

func _exit_tree() -> void:
	disconnect_signals()

func _process(delta: float) -> void:
	_update_inputs(delta)
	_handle_engine_logic(delta)
	_handle_transmission(delta)
	_update_drift(delta)
	_update_boost(delta)
	_handle_aerodynamics(delta)

func _physics_process(delta: float) -> void:
	_apply_physics(delta)
	_handle_collisions()
	_update_vehicle_state(delta)

# ============================================================================
# INITIALIZATION
# ============================================================================
func _init_wheel_configuration() -> void:
	_wheel_positions.resize(WHEEL_COUNT)
	_wheel_rotations.resize(WHEEL_COUNT)
	_suspension_compressions.resize(WHEEL_COUNT)
	_wheel_slip_angles.resize(WHEEL_COUNT)
	_wheel_contact_forces.resize(WHEEL_COUNT)
	
	# Configure wheel positions based on vehicle geometry
	# Front Left (-x, -z), Front Right (+x, -z), Rear Left (-x, +z), Rear Right (+x, +z)
	_wheel_positions[WheelType.FRONT_LEFT] = Vector3(-track_width / 2, 0, -wheel_base / 2)
	_wheel_positions[WheelType.FRONT_RIGHT] = Vector3(track_width / 2, 0, -wheel_base / 2)
	_wheel_positions[WheelType.REAR_LEFT] = Vector3(-track_width / 2, 0, wheel_base / 2)
	_wheel_positions[WheelType.REAR_RIGHT] = Vector3(track_width / 2, 0, wheel_base / 2)
	
	# Initialize wheel rotations to zero
	for i in range(WHEEL_COUNT):
		_wheel_rotations[i] = 0.0
		_suspension_compressions[i] = 0.0

func _init_engine_curves() -> void:
	# Generate engine torque/power curves based on RPM
	const RPM_SAMPLES := 100
	const min_rpm := ENGINE_IDLE_RPM
	const max_rpm := ENGINE_MAX_RPM
	const step := (max_rpm - min_rpm) / RPM_SAMPLES
	
	for i in range(RPM_SAMPLES):
		var rpm := min_rpm + i * step
		var normalized_rpm := (rpm - min_rpm) / (max_rpm - min_rpm)
		
		# Torque curve: peak around 40-60% of max RPM for typical engine
		var torque_peak_factor := sin(normalized_rpm * PI * 0.6)
		var torque := engine_max_torque * torque_peak_factor
		
		# Power = Torque * RPM / constant conversion
		var power := torque * rpm * 0.1047  # conversion factor
		
		_engine_torque_curve[rpm] = torque
		_engine_power_curve[rpm] = power

func _connect_signals_to_manager() -> void:
	GameManager.game_state_changed.connect(_on_game_state_changed)
	InputManager.input_updated.connect(_on_input_updated)

func disconnect_signals() -> void:
	if GameManager:
		GameManager.game_state_changed.disconnect(_on_game_state_changed)
	if InputManager:
		InputManager.input_updated.disconnect(_on_input_updated)

func create_physics_material() -> PhysicsMaterial3D:
	var material := PhysicsMaterial3D.new()
	material.static_friction = tire_friction_coefficient
	material.dynamic_friction = tire_friction_coefficient * 0.9
	material.bounce = 0.0
	return material

# ============================================================================
# INPUT HANDLING
# ============================================================================
func _update_inputs(delta: float) -> void:
	# Get input values from InputManager singleton
	if InputManager:
		_throttle_input = clamp(InputManager.get_axis("throttle"), 0.0, 1.0)
		_brake_input = clamp(InputManager.get_axis("brake"), 0.0, 1.0)
		_clutch_input = clamp(InputManager.get_axis("clutch"), 0.0, 1.0)
		_steering_input = clamp(InputManager.get_axis("steering"), -1.0, 1.0)
		_handbrake_active = InputManager.is_action_pressed("handbrake")
		
		# Handle shift inputs
		if InputManager.is_action_just_pressed("shift_up"):
			request_shift(1)
		elif InputManager.is_action_just_pressed("shift_down"):
			request_shift(-1)
		
		# Emit signal updates
		emit_signal("throttle_applied", _throttle_input)
		emit_signal("brake_applied", _brake_input)
		emit_signal("steering_angle_changed", _steering_input * MAX_STEERING_ANGLE)

func _on_input_updated(input_data: Dictionary) -> void:
	if input_data.has("throttle"):
		_throttle_input = clamp(input_data["throttle"], 0.0, 1.0)
	if input_data.has("brake"):
		_brake_input = clamp(input_data["brake"], 0.0, 1.0)
	if input_data.has("clutch"):
		_clutch_input = clamp(input_data["clutch"], 0.0, 1.0)
	if input_data.has("steering"):
		_steering_input = clamp(input_data["steering"], -1.0, 1.0)
	if input_data.has("handbrake"):
		_handbrake_active = input_data["handbrake"]
	if input_data.has("gear"):
		_target_gear = input_data["gear"]

func _on_game_state_changed(new_state: GameState) -> void:
	match new_state:
		GameState.RACE_ACTIVE:
			_is_engine_running = true
			_lap_start_time = Time.get_ticks_msec()
		GameState.RACE_PAUSED:
			pass
		GameState.RACE_FINISHED:
			_finalize_lap_results()

# ============================================================================
# ENGINE MANAGEMENT
# ============================================================================
func _handle_engine_logic(delta: float) -> void:
	if not _is_engine_running:
		_current_rpm = lerp(_current_rpm, 0.0, delta * 2.0)
		_current_speed = lerp(_current_speed, 0.0, delta * 2.0)
		return
	
	var engine_load: float = _calculate_engine_load()
	var target_rpm: float = _calculate_target_rpm(engine_load)
	
	# Smoothly interpolate RPM
	_current_rpm = lerp(_current_rpm, target_rpm, delta * 10.0)
	
	# Clamp RPM to valid range
	_current_rpm = clamp(_current_rpm, ENGINE_IDLE_RPM, ENGINE_MAX_RPM)
	
	# Check for stalling
	if _current_rpm < CLUTCH_DISENGAGE_RPM and _throttle_input < 0.1:
		if not _is_engine_stalled():
			_engine_stalled.emit()
			_is_engine_running = false
	
	# Check for redline over-rev
	if _current_rpm > ENGINE_MAX_RPM * 1.05:
		_current_rpm = ENGINE_MAX_RPM * 0.95
		# Add damage or warning here
	
	emit_signal("rpm_changed", _current_rpm)

func _calculate_engine_load() -> float:
	# Calculate engine load based on throttle, gear, and speed
	var gear_ratio: float = _get_current_gear_ratio()
	var speed_factor: float = _current_speed / (gear_ratio * 100.0)
	var load := _throttle_input * (1.0 - abs(speed_factor))
	return clamp(load, 0.0, 1.0)

func _calculate_target_rpm(engine_load: float) -> float:
	var base_rpm: float = ENGINE_IDLE_RPM
	var idle_variation: float = randf_range(-50.0, 50.0)
	
	if _current_gear == 0:
		# Neutral - just idle
		base_rpm += idle_variation
	else:
		# In gear - calculate expected RPM based on speed and gear ratio
		var gear_ratio: float = _get_current_gear_ratio()
		var wheel_rpm: float = _current_speed / (tire_radius * 2.0 * PI)
		var drive_wheel_rpm: float = wheel_rpm * gear_ratio * FINAL_DRIVE
		target_rpm = drive_wheel_rpm
		
		# Apply load factor for throttle response
		target_rpm = lerp(target_rpm, base_rpm, 1.0 - engine_load)
	
	return target_rpm + idle_variation

func _get_current_gear_ratio() -> float:
	if _current_gear <= 0:
		return 0.0
	return GEAR_RATIOS[_current_gear - 1] if _current_gear <= len(GEAR_RATIOS) else 0.0

func _is_engine_stalled() -> bool:
	return _current_rpm < CLUTCH_DISENGAGE_RPM and _clutch_input > 0.5

# ============================================================================
# TRANSMISSION & GEAR SHIFTING
# ============================================================================
func _handle_transmission(delta: float) -> void:
	match transmission_type:
		TransmissionType.MANUAL:
			_handle_manual_transmission(delta)
		TransmissionType.AUTOMATIC:
			_handle_automatic_transmission(delta)
		TransmissionType.SEMI_AUTOMATIC:
			_handle_semi_automatic_transmission(delta)

func _handle_manual_transmission(delta: float) -> void:
	# Manual: only shift when clutch is pressed
	if _clutch_input < 0.3:
		if _target_gear != _current_gear and _target_gear >= 0:
			_attempt_gear_shift(_target_gear, delta)
	else:
		# Clutch disengaged - allow immediate gear change
		_current_gear = _target_gear

func _handle_automatic_transmission(delta: float) -> void:
	# Automatic: compute optimal gear based on speed and throttle
	var optimal_gear: int = _compute_optimal_gear_auto()
	
	if optimal_gear != _current_gear:
		# Smooth automatic shift
		_current_gear = lerp_int(_current_gear, optimal_gear, delta * 5.0)
		_current_gear = round(_current_gear)

func _handle_semi_automatic_transmission(delta: float) -> void:
	# Semi-automatic: follow player input but prevent unsafe shifts
	if _clutch_input < 0.3:
		if _target_gear != _current_gear and _target_gear >= 0:
			if _is_safe_to_shift(_target_gear):
				_current_gear = _target_gear

func request_shift(direction: int) -> void:
	var new_gear: int = _current_gear + direction
	
	# Validate gear shift
	if new_gear < 0:
		new_gear = 0  # Neutral
	elif new_gear > len(GEAR_RATIOS) + 1:
		new_gear = len(GEAR_RATIOS)  # Max gear
	
	_target_gear = new_gear
	
	# Auto-depress clutch for manual
	if transmission_type == TransmissionType.MANUAL:
		_clutch_input = 0.0
		await get_tree().create_timer(0.1).timeout
		_clutch_input = 1.0

func _attempt_gear_shift(new_gear: int, delta: float) -> void:
	if new_gear == _current_gear:
		return
	
	var old_gear: int = _current_gear
	_current_gear = new_gear
	
	# Verify shift is safe
	if not _is_safe_to_shift(new_gear):
		_current_gear = old_gear
		return
	
	emit_signal("gear_changed", old_gear, new_gear)
	
	# Sound effect for gear change
	AudioManager.play_sound("gear_change_" + str(new_gear))

func _compute_optimal_gear_auto() -> int:
	var target_speed: float = _current_speed * 3.6  # Convert to km/h
	var throttle: float = _throttle_input
	
	# Simple automatic transmission logic
	if throttle < 0.2:
		# Cruising - upshift
		if _current_speed > 30.0:  # m/s
			return min(len(GEAR_RATIOS), max(1, _current_gear + 1))
		else:
			return max(1, _current_gear)
	elif throttle > 0.7:
		# Acceleration - downshift
		if _current_speed < 50.0:
			return max(1, _current_gear - 1)
		else:
			return _current_gear
	else:
		# Normal driving - maintain current gear
		return _current_gear

func _is_safe_to_shift(gear: int) -> bool:
	if gear == 0:
		return true  # Neutral always safe
	
	var gear_ratio: float = GEAR_RATIOS[gear - 1]
	var expected_rpm: float = _current_speed / (gear_ratio * tire_radius * 2.0 * PI) * FINAL_DRIVE
	
	# Prevent over-revving or lugging
	if expected_rpm > ENGINE_MAX_RPM * 1.2:
		return false
	if expected_rpm < ENGINE_IDLE_RPM * 0.5:
		return false
	
	return true

func _apply_gear_effect(delta: float) -> void:
	var gear_ratio: float = _get_current_gear_ratio()
	
	if _current_gear <= 0:
		# Neutral - no drive force
		return
	
	# Calculate wheel torque based on engine torque and gear ratio
	var engine_torque: float = _get_engine_torque_at_rpm()
	var total_gearing: float = gear_ratio * FINAL_DRIVE
	var wheel_torque: float = engine_torque * total_gearing * 0.95  # drivetrain loss
	
	# Distribute torque based on drivetrain type
	var torque_distribution: Dictionary = _get_torque_distribution(wheel_torque)
	
	# Apply forces to driven wheels
	for wheel_type in torque_distribution:
		var torque: float = torque_distribution[wheel_type]
		_apply_wheel_drive_force(wheel_type, torque, delta)

func _get_engine_torque_at_rpm() -> float:
	# Interpolate torque from curve based on current RPM
	var rpm_floor: int = floor(_current_rpm / 100.0) * 100
	var rpm_ceil: int = ceil(_current_rpm / 100.0) * 100
	
	var torque_floor: float = _engine_torque_curve.get(rpm_floor, engine_max_torque * 0.5)
	var torque_ceil: float = _engine_torque_curve.get(rpm_ceil, engine_max_torque * 0.5)
	
	var interpolation: float = (_current_rpm - rpm_floor) / (rpm_ceil - rpm_floor)
	return lerp(torque_floor, torque_ceil, interpolation)

func _get_torque_distribution(wheel_torque: float) -> Dictionary:
	var distribution: Dictionary = {}
	
	match drivetrain_type:
		DrivetrainType.FWD:
			distribution[WheelType.FRONT_LEFT] = wheel_torque * 0.5
			distribution[WheelType.FRONT_RIGHT] = wheel_torque * 0.5
		DrivetrainType.RWD:
			distribution[WheelType.REAR_LEFT] = wheel_torque * 0.5
			distribution[WheelType.REAR_RIGHT] = wheel_torque * 0.5
		DrivetrainType.AWD:
			distribution[WheelType.FRONT_LEFT] = wheel_torque * 0.4 * 0.5
			distribution[WheelType.FRONT_RIGHT] = wheel_torque * 0.4 * 0.5
			distribution[WheelType.REAR_LEFT] = wheel_torque * 0.6 * 0.5
			distribution[WheelType.REAR_RIGHT] = wheel_torque * 0.6 * 0.5
	
	return distribution

# ============================================================================
# PHYSICS APPLICATION
# ============================================================================
func _apply_physics(delta: float) -> void:
	# Clear velocity for fresh calculation
	velocity = Vector3.ZERO
	
	# Apply gravity
	var gravity_vector: Vector3 = -PhysicsSettings.gravity * Vector3.UP
	add_gravity(gravity_vector * delta)
	
	# Update suspension compression
	_update_suspension()
	
	# Calculate wheel velocities and apply forces
	_apply_wheel_forces(delta)
	
	# Apply aerodynamic forces
	_apply_aero_forces(delta)
	
	# Move character body
	move_and_slide()
	
	# Apply friction from ground contact
	_apply_ground_friction(delta)

func _update_suspension() -> void:
	# Raycast downwards from each wheel position to find ground height
	for i in range(WHEEL_COUNT):
		var start_pos: Vector3 = global_transform.origin + transform.basis * _wheel_positions[i]
		var end_pos: Vector3 = start_pos - Vector3.UP * (ride_height + suspension_travel + tire_radius)
		
		var space_state: PhysicsShapeQueryState3D = PhysicsShapeQueryParameters3D.new()
		space_state.collide_with_areas = true
		space_state.collide_with_bodies = true
		space_state.transform = Transform3D(Quaternion(), start_pos)
		space_state.radius = tire_radius
		space_state.margin = 0.01
		
		var result: Array = PhysicsServer3D.shape_get_result_list(space_state)
		
		if result.size() > 0:
			var hit_point: Vector3 = result[0].position
			var suspension_distance: float = start_pos.y - hit_point.y - tire_radius
			suspension_distance = clamp(suspension_distance, 0.0, suspension_travel)
			
			_suspension_compressions[i] = suspension_distance
			
			# Apply suspension force (spring-damper model)
			var spring_force: float = suspension_stiffness * suspension_distance
			var damper_velocity: float = -velocity.y
			var damper_force: float = suspension_damping * damper_velocity
			
			var total_force: float = spring_force + damper_force
			var wheel_force: Vector3 = Vector3.UP * total_force
			
			# Apply force at wheel position
			var wheel_global_pos: Vector3 = global_transform.origin + transform.basis * _wheel_positions[i]
			apply_impulse(wheel_force * delta, wheel_global_pos)

func _apply_wheel_forces(delta: float) -> void:
	# Calculate drive and brake forces for each wheel
	for i in range(WHEEL_COUNT):
		var wheel_type: WheelType = WheelType(i)
		var wheel_pos: Vector3 = _wheel_positions[i]
		
		# Calculate wheel linear velocity
		var angular_velocity: Vector3 = angular_velocity
		var wheel_vel: Vector3 = velocity + angular_velocity.cross(wheel_pos)
		
		# Drive force calculation
		var drive_force: float = 0.0
		if _current_gear > 0 and _throttle_input > 0:
			drive_force = _calculate_drive_force(wheel_type)
		
		# Brake force calculation
		var brake_force: float = 0.0
		if _brake_input > 0 or _handbrake_active:
			brake_force = _calculate_brake_force(wheel_type)
		
		# Steering angle application
		var steering_angle: float = 0.0
		if i < 2:  # Front wheels
			steering_angle = _steering_input * MAX_STEERING_ANGLE
			_rotate_wheel_visual(wheel_type, steering_angle)
		
		# Combine forces
		var total_force: float = drive_force - brake_force
		
		# Apply force in world direction
		var forward_dir: Vector3 = transform.basis.xform(Vector3.FORWARD)
		var force_vector: Vector3 = forward_dir * total_force
		
		# Apply impulse to vehicle body
		apply_impulse(force_vector * delta, global_transform.origin + transform.basis * wheel_pos)

func _calculate_drive_force(wheel_type: WheelType) -> float:
	var gear_ratio: float = _get_current_gear_ratio()
	var engine_torque: float = _get_engine_torque_at_rpm()
	var total_gearing: float = gear_ratio * FINAL_DRIVE
	
	var wheel_torque: float = engine_torque * total_gearing * 0.95
	
	# Torque vectoring for AWD
	if drivetrain_type == DrivetrainType.AWD and torque_vectoring_enabled:
		wheel_torque *= 1.0 + _steering_input * 0.1
	
	# Convert torque to force (torque = force * radius)
	var drive_force: float = wheel_torque / tire_radius
	
	# Limit by available grip
	var max_traction: float = mass * PhysicsSettings.gravity * tire_friction_coefficient / WHEEL_COUNT
	drive_force = min(drive_force, max_traction)
	
	return drive_force

func _calculate_brake_force(wheel_type: WheelType) -> float:
	var brake_pressure: float = _brake_input
	if _handbrake_active:
		brake_pressure = 1.0
	
	var base_brake_force: float = brake_force_per_wheel * brake_pressure
	
	# Brake bias for front/rear
	if wheel_type in [WheelType.FRONT_LEFT, WheelType.FRONT_RIGHT]:
		base_brake_force *= brake_bias_front
	else:
		base_brake_force *= (1.0 - brake_bias_front)
	
	# Anti-lock braking system logic
	if anti_lock_braking_enabled and _should_apply_abs(wheel_type):
		base_brake_force *= 0.7  # ABS reduces brake pressure
	
	return base_brake_force

func _should_apply_abs(wheel_type: WheelType) -> bool:
	# Simplified ABS: check if wheel is decelerating too fast
	var wheel_vel: float = abs(velocity.length())
	if wheel_vel < 1.0:
		return true
	return false

func _rotate_wheel_visual(wheel_type: WheelType, angle: float) -> void:
	# This would rotate actual mesh objects in a full implementation
	# For now, store the steering angle for potential visualization
	pass

# ============================================================================
# DRIFT MECHANICS
# ============================================================================
func _update_drift(delta: float) -> void:
	var lateral_velocity: float = velocity.x  # Assuming X is lateral direction
	
	# Calculate slip angle
	var heading: Vector3 = transform.basis.x
	var lateral_heading: Vector3 = Vector3(-heading.z, 0, heading.x)
	var lateral_velocity_component: float = velocity.dot(lateral_heading)
	var slip_angle: float = atan2(lateral_velocity_component, abs(velocity.length()))
	
	# Detect drift entry
	if not _in_drift and abs(slip_angle) > drift_threshold and _throttle_input > 0.3:
		_enter_drift(slip_angle)
	
	# Maintain drift
	if _in_drift:
		_drift_angle = lerp(_drift_angle, slip_angle, delta * 5.0)
		
		# Reduce grip during drift
		tire_friction_coefficient = max(0.1, tire_friction_coefficient - delta * 0.1)
		
		# Skid particles could be spawned here
		if randf() < 0.1:
			_spawn_skid_marks()
	
	# Exit drift
	if _in_drift and (abs(slip_angle) < drift_threshold * 0.5 or _throttle_input < 0.2):
		_exit_drift()
	
	# Recover from drift
	if _in_drift:
		tire_friction_coefficient = lerp(tire_friction_coefficient, 1.2, delta * drift_recovery_rate)
	
	emit_signal("skidding", _in_drift)

func _enter_drift(slip_angle: float) -> void:
	_in_drift = true
	_drift_angle = slip_angle
	emit_signal("drift_started", slip_angle)
	AudioManager.play_sound("drift_start")

func _exit_drift() -> void:
	_in_drift = false
	emit_signal("drift_ended")
	AudioManager.play_sound("drift_end")

func _spawn_skid_marks() -> void:
	# Spawn particle effects for skid marks
	# Implementation would add visual feedback
	pass

# ============================================================================
# AERODYNAMICS
# ============================================================================
func _handle_aerodynamics(delta: float) -> void:
	var aero_drag: float = _calculate_drag()
	var aero_downforce: float = _calculate_downforce()
	
	# Apply drag opposite to velocity
	var drag_force: Vector3 = -velocity.normalized() * aero_drag
	apply_impulse(drag_force * delta, global_transform.origin)
	
	# Apply downforce (negative Y)
	var downforce_vector: Vector3 = Vector3.DOWN * aero_downforce
	apply_impulse(downforce_vector * delta, global_transform.origin)

func _apply_aero_forces(delta: float) -> void:
	# Combined aerodynamic forces
	_handle_aerodynamics(delta)

func _calculate_drag() -> float:
	var air_density: float = 1.225  # kg/m³ at sea level
	var v_squared: float = velocity.length() ** 2
	
	var drag_force: float = 0.5 * air_density * v_squared * drag_coefficient * frontal_area
	return drag_force

func _calculate_downforce() -> float:
	var air_density: float = 1.225
	var v_squared: float = velocity.length() ** 2
	
	var downforce: float = 0.5 * air_density * v_squared * downforce_coefficient * frontal_area
	return downforce

# ============================================================================
# BOOST SYSTEM
# ============================================================================
func _update_boost(delta: float) -> void:
	if _boost_available < 100.0:
		# Recharge boost when not using it
		_boost_available = min(100.0, _boost_available + delta * 10.0)
	
	if _boost_cooldown > 0:
		_boost_cooldown -= delta

func use_boost(duration: float = 5.0) -> bool:
	if _boost_available >= 100.0 and _boost_cooldown <= 0:
		_boost_available = 0.0
		_boost_cooldown = duration
		emit_signal("boost_used", duration)
		
		# Apply temporary speed increase
		velocity *= 1.5
		AudioManager.play_sound("boost_activate")
		return true
	
	return false

# ============================================================================
# COLLISION HANDLING
# ============================================================================
func _handle_collisions() -> void:
	for i in range(get_slide_count()):
		var collision: KinematicCollision3D = get_slide_collision(i)
		
		_is_in_collision = true
		_collision_damage += collision.get_normal().length()
		
		# Screen shake effect
		_screen_shake(10.0)
		
		# Emit collision signal
		var collision_info: Dictionary = {
			"object": collision.get_collider(),
			"normal": collision.get_normal(),
			"position": collision.get_position(),
			"speed": collision.get_object_local_velocity().length()
		}
		collision_detected.emit(collision_info)
		
		# Apply bounce
		var bounce_force: Vector3 = collision.get_normal() * collision.get_speed() * 0.5
		velocity += bounce_force

func _screen_shake(intensity: float) -> void:
	# Simple screen shake implementation
	# Would typically affect camera, not vehicle directly
	pass

func reset_collision_damage() -> void:
	_collision_damage = 0.0
	_is_in_collision = false

# ============================================================================
# LAP & CHECKPOINT SYSTEM
# ============================================================================
func start_lap() -> void:
	_lap_start_time = Time.get_ticks_msec()
	_last_checkpoint_id = -1
	_lap_times.clear()

func record_checkpoint(checkpoint_id: int) -> void:
	if checkpoint_id > _last_checkpoint_id:
		_last_checkpoint_id = checkpoint_id
		checkpoint_passed.emit(checkpoint_id)
		
		# Record intermediate lap time
		var lap_time: float = (Time.get_ticks_msec() - _lap_start_time) / 1000.0
		_lap_times.append(lap_time)

func finish_lap() -> float:
	var lap_time: float = (Time.get_ticks_msec() - _lap_start_time) / 1000.0
	lap_completed.emit(lap_time)
	_lap_times.append(lap_time)
	return lap_time

func _finalize_lap_results() -> void:
	if _lap_times.size() > 0:
		var best_lap: float = _lap_times.min()
		var avg_lap: float = _lap_times.reduce(func(a, b): return a + b) / _lap_times.size()
		
		GameManager.set_best_lap(best_lap)
		GameManager.set_average_lap(avg_lap)

# ============================================================================
# VEHICLE STATE UPDATES
# ============================================================================
func _update_vehicle_state(delta: float) -> void:
	# Update speed from velocity
	_current_speed = velocity.length()
	
	# Clamp speed to reasonable limits
	_current_speed = clamp(_current_speed, 0.0, 200.0)  # 200 m/s max
	
	# Update total distance
	var delta_distance: float = _current_speed * delta
	_total_distance += delta_distance
	
	# Emit speed signal
	emit_signal("speed_changed", _current_speed)
	
	# Update wheel rotation angles
	_update_wheel_rotation_angles()

func _update_wheel_rotation_angles() -> void:
	for i in range(WHEEL_COUNT):
		var wheel_speed: float = _current_speed / tire_radius
		_wheel_rotations[i] += wheel_speed * 0.1  # Simplified rotation update

func _update_suspension_positions() -> void:
	# Called after vehicle moves to update suspension positions
	# Would involve raycasting in a full implementation
	pass

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================
func set_vehicle_health(health: float) -> void:
	# Health system for damage
	pass

func get_vehicle_health() -> float:
	return 100.0 - _collision_damage

func repair_vehicle() -> void:
	_collision_damage = 0.0

func reset_vehicle_position(position: Vector3, rotation: Quaternion) -> void:
	global_position = position
	global_rotation_quaternion = rotation
	velocity = Vector3.ZERO
	angular_velocity = Vector3.ZERO

func enable_all_systems(enabled: bool) -> void:
	# Toggle all vehicle systems
	pass

# ============================================================================
# DEBUG & VISUALIZATION
# ============================================================================
func _draw_debug_lines() -> void:
	# Debug drawing for development
	if not GameManager.debug_mode:
		return
	
	# Draw wheel positions
	for i in range(WHEEL_COUNT):
		draw_line(
			global_position + transform.basis * _wheel_positions[i],
			global_position + transform.basis * _wheel_positions[i] + Vector3.UP * 2.0,
			Color.GREEN,
			2.0
		)

func log_vehicle_state() -> void:
	print("[VehicleState]")
	print("\tSpeed: %.2f m/s (%.0f km/h)" % [_current_speed, _current_speed * 3.6])
	print("\tRPM: %.0f" % _current_rpm)
	print("\tGear: %d" % _current_gear)
	print("\tThrottle: %.2f" % _throttle_input)
	print("\tBrake: %.2f" % _brake_input)
	print("\tSteering: %.2f" % _steering_input)
	print("\tDistance: %.2f m" % _total_distance)
	print("\tLaps: %d" % _lap_times.size())
	print("\tIn Drift: %s" % _in_drift)