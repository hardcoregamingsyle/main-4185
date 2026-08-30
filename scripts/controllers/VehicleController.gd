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

# ============================================================================
# CONFIGURATION CONSTANTS
# ============================================================================
const MAX_SPEED_KMH: float = 350.0
const ACCELERATION_POWER: float = 20000.0
const BRAKING_FORCE: float = 40000.0
const STEERING_SPEED: float = 2.5
const MAX_STEERING_ANGLE: float = 35.0 * TAU / 180.0
const MIN_GEAR: int = -1  # Reverse
const MAX_GEAR: int = 6
const NEUTRAL_GEAR: int = 0
const IDLE_RPM: float = 800.0
const REDLINE_RPM: float = 8000.0
const CLUTCH_RELEASE_TIME: float = 0.3
const DIFFERENTIAL_LOCK_RATIO: float = 0.8
const DRIFT_FACTOR: float = 0.15
const TRACTION_CONTROL_THRESHOLD: float = 0.15
const ABS_THRESHOLD: float = 0.1
const SHIFT_POINT_RPM: float = 7000.0
const SHUTDOWN_RPM: float = 9000.0
const GEAR_SHIFT_DELAY: float = 0.1
const ENGINE_REVO_DROP_ON_DOWNSHIFT: float = 2000.0

# ============================================================================
# STATE VARIABLES
# ============================================================================
var _speed_kmh: float = 0.0
var _rpm: float = IDLE_RPM
var _current_gear: int = NEUTRAL_GEAR
var _target_gear: int = NEUTRAL_GEAR
var _throttle_input: float = 0.0
var _brake_input: float = 0.0
var _steering_input: float = 0.0
var _handbrake_input: float = 0.0

var _is_engine_running: bool = false
var _is_clutch_engaged: bool = true
var _is_handbrake_active: bool = false
var _is_skidding: bool = false
var _is_drifting: bool = false
var _is_in_shift: bool = false

var _gear_ratios: Array[float] = [0.0, 3.5, 2.5, 1.8, 1.4, 1.1, 0.9, 0.7]
var _final_drive_ratio: float = 3.5
var _wheel_base: float = 2.7
var _track_width: float = 1.6
var _wheel_radius: float = 0.33
var _vehicle_mass: float = 1500.0

var _last_position: Vector3 = Vector3.ZERO
var _velocity_direction: Vector3 = Vector3.ZERO
var _angular_velocity: float = 0.0
var _drift_angle: float = 0.0

var _traction_control_enabled: bool = true
var _abs_enabled: bool = true

# Suspension and tire data
var _front_left_wheel: Node3D
var _front_right_wheel: Node3D
var _rear_left_wheel: Node3D
var _rear_right_wheel: Node3D

var _wheel_suspension_strength: float = 40000.0
var _wheel_suspension_damping: float = 3000.0
var _tire_friction_coefficient: float = 1.2

var _inertia_tensor: Vector3 = Vector3(100.0, 100.0, 150.0)

# References
var _rigid_body: RigidBody3D = null
var _engine_node: Node = null
var _powertrain_node: Powertrain = null
var _collision_detection: CollisionShape3D = null

# Timing variables
var _shift_timer: float = 0.0
var _clutch_timer: float = 0.0
var _engine_warmup_time: float = 2.0
var _time_since_start: float = 0.0

# ============================================================================
# INITIALIZATION
# ============================================================================
func _ready() -> void:
	_init_rigid_body()
	_init_vehicle_components()
	_connect_signals_to_game_manager()
	_load_configuration()
	_set_initial_state()

func _physics_process(delta: float) -> void:
	if not _is_ready():
		return
	
	_time_since_start += delta
	
	# Update clutch state
	_update_clutch(delta)
	
	# Process input
	_process_inputs(delta)
	
	# Calculate RPM based on gear and speed
	_calculate_rpm()
	
	# Handle gear shifting
	_handle_gear_shifting(delta)
	
	# Apply engine power
	_apply_engine_power(delta)
	
	# Apply braking forces
	_apply_brakes(delta)
	
	# Apply steering
	_apply_steering(delta)
	
	# Handle drifting mechanics
	_handle_drift(delta)
	
	# Apply traction control if enabled
	_apply_traction_control(delta)
	
	# Apply ABS if enabled
	_apply_abs(delta)
	
	# Update vehicle position
	_update_position(delta)
	
	# Check for collisions
	_check_collisions(delta)
	
	# Emit signals
	_emit_signals()

func _input(event: InputEvent) -> void:
	if event.is_action_pressed("game_ui"):
		_toggle_engine()

func _unhandled_input(event: InputEvent) -> void:
	pass

# ============================================================================
# VEHICLE SETUP
# ============================================================================
func _init_rigid_body() -> void:
	var rigid_body = find_child("RigidBody3D", true, true) as RigidBody3D
	if rigid_body == null:
		rigid_body = RigidBody3D.new()
		rigid_body.name = "RigidBody3D"
		add_child(rigid_body)
	
	_rigid_body = rigid_body
	
	# Set mass from PhysicsSettings
	var settings: PhysicsSettings = get_physics_settings()
	_vehicle_mass = settings.default_vehicle_mass
	_rigid_body.mass = _vehicle_mass
	
	# Configure physics properties
	_rigid_body.collision_layer = 2  # Vehicle layer
	_rigid_body.collision_mask = 3   # Ground + other vehicles
	_rigid_body.linear_damp = 0.5
	_rigid_body.angular_damp = 0.5

func _init_vehicle_components() -> void:
	# Find wheel nodes
	_front_left_wheel = _find_wheel_node("FrontLeftWheel")
	_front_right_wheel = _find_wheel_node("FrontRightWheel")
	_rear_left_wheel = _find_wheel_node("RearLeftWheel")
	_rear_right_wheel = _find_wheel_node("RearRightWheel")
	
	# Find engine node
	var engine = find_child("Engine", true, true)
	if engine:
		_engine_node = engine
	
	# Find powertrain node
	_powertrain_node = find_child("Powertrain", true, true) as Powertrain
	if _powertrain_node == null:
		var powertrain_scene: PackedScene = load("res://scripts/vehicles/Powertrain.gd")
		_powertrain_node = powertrain_scene.instantiate()
		add_child(_powertrain_node)
	
	# Find collision detection
	_collision_detection = find_child("CollisionDetection", true, true) as CollisionShape3D
	if _collision_detection == null:
		_collision_detection = CollisionShape3D.new()
		_collision_detection.name = "CollisionDetection"
		add_child(_collision_detection)

func _find_wheel_node(name: String) -> Node3D:
	var wheel = find_child(name, true, true) as Node3D
	if wheel == null:
		wheel = _create_wheel_node(name)
	return wheel

func _create_wheel_node(name: String) -> Node3D:
	var wheel = Node3D.new()
	wheel.name = name
	wheel.position = _get_wheel_position(name)
	
	var mesh = MeshInstance3D.new()
	mesh.mesh = BoxMesh.new()
	mesh.mesh.size = Vector3(0.66, 0.66, 0.66)
	wheel.add_child(mesh)
	
	add_child(wheel)
	return wheel

func _get_wheel_position(name: String) -> Vector3:
	match name:
		"FrontLeftWheel": return Vector3(-0.8, 0.33, 0.8)
		"FrontRightWheel": return Vector3(-0.8, 0.33, -0.8)
		"RearLeftWheel": return Vector3(0.8, 0.33, 0.8)
		"RearRightWheel": return Vector3(0.8, 0.33, -0.8)
		_: return Vector3.ZERO

func _connect_signals_to_game_manager() -> void:
	if GameManager:
		GameManager.race_started.connect(_on_race_started)
		GameManager.game_state_changed.connect(_on_game_state_changed)

func _load_configuration() -> void:
	var settings: PhysicsSettings = get_physics_settings()
	_wheel_base = settings.vehicle_config.get("wheel_base", _wheel_base)
	_track_width = settings.vehicle_config.get("track_width", _track_width)
	_wheel_radius = settings.vehicle_config.get("wheel_radius", _wheel_radius)
	_vehicle_mass = settings.default_vehicle_mass

func _set_initial_state() -> void:
	_is_engine_running = false
	_current_gear = NEUTRAL_GEAR
	_target_gear = NEUTRAL_GEAR
	_rpm = IDLE_RPM
	_speed_kmh = 0.0
	_throttle_input = 0.0
	_brake_input = 0.0
	_steering_input = 0.0
	_handbrake_input = 0.0
	_last_position = global_position
	_velocity_direction = Vector3.FORWARD
	_is_skidding = false
	_is_drifting = false
	_is_clutch_engaged = true
	_is_handbrake_active = false

# ============================================================================
# INPUT PROCESSING
# ============================================================================
func _process_inputs(delta: float) -> void:
	# Read throttle input
	if Input.is_action_pressed("throttle"):
		_throttle_input = clampf(_throttle_input + delta * 2.0, 0.0, 1.0)
	elif Input.is_action_pressed("release_throttle"):
		_throttle_input = maxf(_throttle_input - delta * 3.0, 0.0)
	else:
		_throttle_input = lerp(_throttle_input, 0.0, delta * 5.0)
	
	# Read brake input
	if Input.is_action_pressed("brake"):
		_brake_input = clampf(_brake_input + delta * 2.0, 0.0, 1.0)
	elif Input.is_action_pressed("release_brake"):
		_brake_input = maxf(_brake_input - delta * 3.0, 0.0)
	else:
		_brake_input = lerp(_brake_input, 0.0, delta * 5.0)
	
	# Read steering input
	if Input.is_action_pressed("steer_left"):
		_steering_input = minf(_steering_input + delta * STEERING_SPEED, MAX_STEERING_ANGLE)
	elif Input.is_action_pressed("steer_right"):
		_steering_input = maxf(_steering_input - delta * STEERING_SPEED, -MAX_STEERING_ANGLE)
	else:
		_steering_input = lerp(_steering_input, 0.0, delta * 8.0)
	
	# Read handbrake input
	if Input.is_action_pressed("handbrake"):
		_handbrake_input = 1.0
	else:
		_handbrake_input = 0.0
	
	# Gear shifting via keyboard
	if Input.is_action_just_pressed("gear_up"):
		_request_gear_change(1)
	elif Input.is_action_just_pressed("gear_down"):
		_request_gear_change(-1)
	elif Input.is_action_just_pressed("gear_neutral"):
		_request_gear_change(NEUTRAL_GEAR - _current_gear)
	
	# Engine toggle
	if Input.is_action_just_pressed("toggle_engine"):
		_toggle_engine()

func _request_gear_change(change: int) -> void:
	if change < 0:  # Downshift
		var new_gear = _current_gear + change
		if new_gear >= MIN_GEAR and new_gear <= MAX_GEAR:
			_target_gear = new_gear
	elif change > 0:  # Upshift
		var new_gear = _current_gear + change
		if new_gear >= 0 and new_gear <= MAX_GEAR:
			_target_gear = new_gear
	elif change == NEUTRAL_GEAR - _current_gear:
		_target_gear = NEUTRAL_GEAR

# ============================================================================
# GEAR SHIFTER LOGIC
# ============================================================================
func _handle_gear_shifting(delta: float) -> void:
	if _is_in_shift:
		_shift_timer += delta
		if _shift_timer >= GEAR_SHIFT_DELAY:
			_complete_gear_shift()
		return
	
	if _target_gear != _current_gear:
		_start_gear_shift()

func _start_gear_shift() -> void:
	_is_in_shift = true
	_shift_timer = 0.0
	_is_clutch_engaged = false
	_clutch_timer = CLUTCH_RELEASE_TIME
	
	# Drop RPM during shift
	_rpm = max(IDLE_RPM, _rpm - 1000.0)

func _complete_gear_shift() -> void:
	var old_gear: int = _current_gear
	_current_gear = _target_gear
	
	if _current_gear != old_gear:
		gear_changed.emit(old_gear, _current_gear)
		
		# Adjust RPM based on gear ratio
		var current_ratio: float = _gear_ratios[abs(_current_gear)] if _current_gear >= 0 else 0.5
		var target_ratio: float = _gear_ratios[abs(old_gear)] if old_gear >= 0 else 0.5
		
		if _current_gear > old_gear:  # Upshift
			_rpm = lerp(_rpm, IDLE_RPM + (_rpm - IDLE_RPM) * 0.7, 0.5)
		elif _current_gear < old_gear:  # Downshift
			_rpm = lerp(_rpm, min(REDLINE_RPM, _rpm + ENGINE_REVO_DROP_ON_DOWNSHIFT), 0.5)
	
	_is_in_shift = false
	_target_gear = NEUTRAL_GEAR
	_clutch_timer = 0.0

func _update_clutch(delta: float) -> void:
	if _is_in_shift and _clutch_timer > 0:
		_clutch_timer -= delta
		if _clutch_timer <= 0:
			_is_clutch_engaged = true

# ============================================================================
# RPM AND SPEED CALCULATION
# ============================================================================
func _calculate_rpm() -> void:
	if not _is_engine_running:
		_rpm = lerp(_rpm, IDLE_RPM, 0.1)
		return
	
	var wheel_rotation_speed: float = _speed_kmh * 1000.0 / 60.0 / (_wheel_radius * 2.0 * PI)
	
	if _current_gear != NEUTRAL_GEAR:
		var drive_ratio: float = _gear_ratios[_current_gear] * _final_drive_ratio
		if _current_gear > 0:
			_rpm = wheel_rotation_speed * drive_ratio * 60.0
		else:  # Reverse
			_rpm = abs(wheel_rotation_speed) * drive_ratio * 60.0
	
	# Clamp RPM
	_rpm = clampf(_rpm, IDLE_RPM, SHUTDOWN_RPM)
	
	# Simulate engine revving when accelerating
	if _throttle_input > 0 and _is_clutch_engaged:
		var desired_rpm: float = IDLE_RPM + _throttle_input * (REDLINE_RPM - IDLE_RPM)
		_rpm = lerp(_rpm, desired_rpm, 0.1)
	
	# Auto upshift at redline
	if _rpm >= SHIFT_POINT_RPM and _current_gear < MAX_GEAR:
		_request_gear_change(1)
	
	# Auto downshift at low RPM
	elif _rpm <= 2000 and _current_gear > 0:
		_request_gear_change(-1)

func _get_speed_from_rpm() -> float:
	if _current_gear == NEUTRAL_GEAR:
		return 0.0
	
	var drive_ratio: float = _gear_ratios[_current_gear] * _final_drive_ratio
	var wheel_speed: float = _rpm / drive_ratio / 60.0
	return wheel_speed * _wheel_radius * 2.0 * PI * 3.6

# ============================================================================
# ENGINE POWER APPLICATION
# ============================================================================
func _apply_engine_power(delta: float) -> void:
	if not _is_engine_running or not _is_clutch_engaged:
		return
	
	if _current_gear == NEUTRAL_GEAR:
		return
	
	var forward_vector: Vector3 = _get_forward_vector()
	var gear_ratio: float = _gear_ratios[_current_gear]
	var effective_force: float = ACCELERATION_POWER * _throttle_input * gear_ratio
	
	# Apply force to rigid body
	_rigid_body.apply_central_impulse(forward_vector.normalized() * effective_force * delta)
	
	# Adjust speed based on applied force
	var acceleration: float = effective_force / _vehicle_mass * delta
	_speed_kmh = min(MAX_SPEED_KMH, _speed_kmh + acceleration * 3.6)
	
	# Drag coefficient
	var air_drag: float = 0.01 * (_speed_kmh * _speed_kmh)
	_speed_kmh = max(0.0, _speed_kmh - air_drag * delta)

# ============================================================================
# BRAKING SYSTEM
# ============================================================================
func _apply_brakes(delta: float) -> void:
	if _brake_input <= 0:
		return
	
	var brake_force: float = BRAKING_FORCE * _brake_input
	
	# Handbrake applies more force to rear wheels
	var total_brake_force: float = brake_force
	if _is_handbrake_active:
		total_brake_force *= 1.5
	
	# Apply braking force opposite to velocity
	var velocity: Vector3 = _rigid_body.linear_velocity
	if velocity.length() > 0.1:
		var brake_vector: Vector3 = -velocity.normalized() * total_brake_force * delta
		_rigid_body.apply_central_impulse(brake_vector)
	
	# Reduce speed
	_speed_kmh = max(0.0, _speed_kmh - (total_brake_force / _vehicle_mass * 3.6 * delta))
	
	# Stall engine if stopped with gear engaged
	if _speed_kmh < 1.0 and _current_gear != NEUTRAL_GEAR and _current_gear != MIN_GEAR:
		if _rpm < IDLE_RPM * 1.5:
			_is_engine_running = false
			engine_stalled.emit()

# ============================================================================
# STEERING SYSTEM
# ============================================================================
func _apply_steering(delta: float) -> void:
	if _speed_kmh < 5.0:  # No steering at very low speeds
		return
	
	var forward_vector: Vector3 = _get_forward_vector()
	var right_vector: Vector3 = Vector3.RIGHT.rotated(Vector3.UP, _steering_input)
	
	# Apply rotation torque
	var torque: float = right_vector.y * _steering_input * 5000.0
	_rigid_body.apply_torque(torque * Vector3.UP)
	
	# Update angular velocity
	_angular_velocity = lerp(_angular_velocity, _steering_input * 0.5, delta * 5.0)
	_rigid_body.angular_velocity.y = _angular_velocity

func _get_forward_vector() -> Vector3:
	return transform.basis.z.rotated(Vector3.UP, _steering_input * 0.5).inverse()

# ============================================================================
# DRIFT MECHANICS
# ============================================================================
func _handle_drift(delta: float) -> void:
	if _speed_kmh < 30.0:  # Minimum speed to drift
		if _is_drifting:
			_is_drifting = false
			drift_ended.emit()
		return
	
	if _is_handbrake_active and abs(_steering_input) > 0.3:
		_is_drifting = true
		_drift_angle = _steering_input * 0.5
		
		if not _is_skidding:
			skidding.emit(true)
			drift_started.emit(_drift_angle)
		
		# Add lateral velocity component for drift
		var drift_vector: Vector3 = Vector3.LEFT.rotated(Vector3.UP, _drift_angle)
		_rigid_body.velocity += drift_vector * 5.0 * delta
		
	elif _is_drifting:
		_is_drifting = false
		_drift_angle = 0.0
		drift_ended.emit()
		
		if _is_skidding:
			skidding.emit(false)

# ============================================================================
# TRACTION CONTROL & ABS
# ============================================================================
func _apply_traction_control(delta: float) -> void:
	if not _traction_control_enabled:
		return
	
	var wheel_slip: float = _calculate_wheel_slip()
	
	if wheel_slip > TRACTION_CONTROL_THRESHOLD:
		# Reduce throttle to prevent spin
		_throttle_input *= 0.7
		traction_control_state_changed.emit(true)
	else:
		traction_control_state_changed.emit(false)

func _apply_abs(delta: float) -> void:
	if not _abs_enabled:
		return
	
	var wheel_lock: bool = _check_wheel_lock()
	
	if wheel_lock and _brake_input > 0:
		# Pulse brakes to prevent lockup
		_brake_input = lerp(_brake_input, 0.0, delta * 10.0)
		abs_enabled.emit(true)
	else:
		abs_enabled.emit(false)

func _calculate_wheel_slip() -> float:
	var wheel_speed: float = _speed_kmh / 3.6 / _wheel_radius
	var engine_speed: float = _rpm / 60.0 * _gear_ratios[abs(_current_gear)] * _final_drive_ratio
	
	return abs(engine_speed - wheel_speed) / max(engine_speed, wheel_speed)

func _check_wheel_lock() -> bool:
	var wheel_velocity: Vector3 = _rigid_body.linear_velocity
	return abs(wheel_velocity.length()) < 0.5

# ============================================================================
# POSITION UPDATE
# ============================================================================
func _update_position(delta: float) -> void:
	_last_position = global_position
	
	# Move rigid body
	global_position = _rigid_body.global_position
	transform.basis = _rigid_body.basis

func _check_collisions(delta: float) -> void:
	var bodies = _get_overlapping_bodies()
	
	for body in bodies:
		if body is RigidBody3D:
			var relative_velocity: Vector3 = _rigid_body.linear_velocity - body.linear_velocity
			var impact: float = relative_velocity.length()
			
			if impact > 5.0:
				collision_detected.emit({
					"impact_velocity": impact,
					"other_body": body,
					"position": global_position
				})
				
				# Screen shake effect
				_apply_screen_shake(impact * 0.1)

func _get_overlapping_bodies() -> Array:
	var bodies: Array[Node3D] = []
	var area = find_child("Area3D", true, true)
	if area:
		bodies.append_array(area.get_overlapping_bodies())
	return bodies

func _apply_screen_shake(amount: float) -> void:
	# Simple screen shake by rotating camera parent
	var camera = get_viewport().get_camera_3d()
	if camera:
		camera.rotation.z += amount * 0.01

# ============================================================================
# SIGNAL EMITTERS
# ============================================================================
func _emit_signals() -> void:
	if abs(_speed_kmh - _get_previous_speed()) > 0.5:
		speed_changed.emit(_speed_kmh)
	
	if abs(_rpm - _get_previous_rpm()) > 100:
		rpm_changed.emit(_rpm)

func _get_previous_speed() -> float:
	return _speed_kmh

func _get_previous_rpm() -> float:
	return _rpm

# ============================================================================
# PUBLIC METHODS
# ============================================================================
func start_engine() -> void:
	_is_engine_running = true
	_rpm = IDLE_RPM
	_current_gear = NEUTRAL_GEAR
	_is_clutch_engaged = true

func stop_engine() -> void:
	_is_engine_running = false
	_rpm = IDLE_RPM
	_current_gear = NEUTRAL_GEAR

func set_gear(gear: int) -> void:
	if gear >= MIN_GEAR and gear <= MAX_GEAR:
		_target_gear = gear
		_request_gear_change(0)

func get_speed_kmh() -> float:
	return _speed_kmh

func get_rpm() -> float:
	return _rpm

func get_current_gear() -> int:
	return _current_gear

func get_throttle_input() -> float:
	return _throttle_input

func get_brake_input() -> float:
	return _brake_input

func get_steering_input() -> float:
	return _steering_input

func is_engine_running() -> bool:
	return _is_engine_running

func is_drifting() -> bool:
	return _is_drifting

func reset_vehicle() -> void:
	_speed_kmh = 0.0
	_rpm = IDLE_RPM
	_current_gear = NEUTRAL_GEAR
	_throttle_input = 0.0
	_brake_input = 0.0
	_steering_input = 0.0
	_handbrake_input = 0.0
	_is_engine_running = false
	_is_clutch_engaged = true
	_is_drifting = false
	_is_skidding = false
	_last_position = global_position

# ============================================================================
# GAME MANAGER CALLBACKS
# ============================================================================
func _on_race_started(race_data: Dictionary) -> void:
	_reset_for_race()

func _on_game_state_changed(new_state: GameState) -> void:
	match new_state:
		GameState.RACE_ACTIVE:
			_resume_vehicle()
		GameState.RACE_PAUSED:
			_pause_vehicle()
		GameState.RACE_FINISHED:
			_stop_vehicle()

func _reset_for_race() -> void:
	reset_vehicle()
	start_engine()

func _resume_vehicle() -> void:
	_is_engine_running = true
	_is_clutch_engaged = true

func _pause_vehicle() -> void:
	_is_clutch_engaged = false
	_throttle_input = 0.0
	_brake_input = 0.0

func _stop_vehicle() -> void:
	stop_engine()
	reset_vehicle()

# ============================================================================
# HELPER FUNCTIONS
# ============================================================================
func get_physics_settings() -> PhysicsSettings:
	if has_node("/root/PhysicsSettings"):
		return get_node("/root/PhysicsSettings")
	
	var settings = preload("res://scripts/core/PhysicsSettings.gd").new()
	return settings

func _is_ready() -> bool:
	return _rigid_body != null and _is_ready_for_simulation()

func _is_ready_for_simulation() -> bool:
	return is_instance_valid(_rigid_body) and not is_inside_tree() == false

func _toggle_engine() -> void:
	if _is_engine_running:
		stop_engine()
	else:
		start_engine()

func set_traction_control(enabled: bool) -> void:
	_traction_control_enabled = enabled

func set_abs(enabled: bool) -> void:
	_abs_enabled = enabled

func get_vehicle_mass() -> float:
	return _vehicle_mass

func get_wheel_positions() -> Dictionary:
	return {
		"front_left": _front_left_wheel.global_position,
		"front_right": _front_right_wheel.global_position,
		"rear_left": _rear_left_wheel.global_position,
		"rear_right": _rear_right_wheel.global_position
	}

func apply_tire_force(wheel_node: Node3D, force: Vector3) -> void:
	if wheel_node:
		wheel_node.add_force(force)

func get_drift_angle() -> float:
	return _drift_angle

func is_skidding() -> bool:
	return _is_skidding