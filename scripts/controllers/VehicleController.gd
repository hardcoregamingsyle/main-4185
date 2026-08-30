extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for the racing simulator
## Handles throttle, brake, steering inputs, wheel forces, gear shifting logic
## Copyright 2026 Thalamus Racing Simulator Project

# ============================================================================
# SIGNALS
# ============================================================================

signal speed_changed(new_speed: float)
signal rpm_changed(new_rpm: float)
signal gear_changed(old_gear: int, new_gear: int)
signal skidding(skid_intensity: float)
signal crash_impact(force_vector: Vector3)
signal lap_checkpoint_passed(lap_number: int)
signal vehicle_state_changed(state: GameState)

# ============================================================================
# GAME STATE ENUM
# ============================================================================

enum GameState {
	IDLE,
	DRIVING,
	BRAKING,
	SKIDDING,
	OVERSHOOT,
	CRASHED,
	RESETTING
}

# ============================================================================
# CONSTANTS (from PhysicsSettings)
# ============================================================================

const MIN_SPEED_THRESHOLD: float = 0.1
const MAX_SPEED_THRESHOLD: float = 350.0  # km/h
const STEERING_DEADZONE: float = 0.05
const BRAKE_THRESHOLD: float = 0.1
const ACCELERATION_THRESHOLD: float = 0.1
const SKID_THRESHOLD: float = 0.7
const RACE_START_DELAY: float = 3.0

# ============================================================================
# CLASS VARIABLES
# ============================================================================

@onready var _rigid_body: RigidBody3D = $VehicleRigidBody
@onready var _wheel_front_left: SpringArm3D = $Wheels/FrontLeft
@onready var _wheel_front_right: SpringArm3D = $Wheels/FrontRight
@onready var _wheel_rear_left: SpringArm3D = $Wheels/RearLeft
@onready var _wheel_rear_right: SpringArm3D = $Wheels/RearRight
@onready var _powertrain: Node = $Powertrain
@onready var _collision_shape: CollisionShape3D = $CollisionShape
@onready var _debug_visuals: Node3D = $DebugVisuals

var _current_speed: float = 0.0  # m/s
var _current_rpm: float = 0.0
var _current_gear: int = 0
var _max_gear: int = 6
var _neutral_gear: int = 0
var _reverse_gear: int = -1
var _state: GameState = GameState.IDLE
var _last_position: Vector3 = Vector3.ZERO
var _velocity_history: Array[Vector3] = []

# Input states (from InputManager)
var _throttle_input: float = 0.0
var _brake_input: float = 0.0
var _steering_input: float = 0.0
var _clutch_input: float = 0.0
var _handbrake_input: float = 0.0

# Gear ratios (default values, can be overridden)
var _gear_ratios: Dictionary = {
	-1: 3.5,   # Reverse
	1: 4.0,    # First
	2: 2.8,    # Second
	3: 2.0,    # Third
	4: 1.5,    # Fourth
	5: 1.2,    # Fifth
	6: 0.9     # Sixth
}

# Final drive ratio
var _final_drive_ratio: float = 3.73

# Differential type (0 = open, 1 = limited slip, 2 = locked)
var _differential_type: int = 1

# Tire grip coefficient (0.0-1.0)
var _tire_grip: float = 0.9

# Aerodynamic properties
var _drag_coefficient: float = 0.35
var _frontal_area: float = 2.2  # m²
var _air_density: float = 1.225  # kg/m³

# ============================================================================
# PUBLIC PROPERTIES
# ============================================================================

func get_current_speed() -> float:
	return _current_speed

func get_current_rpm() -> float:
	return _current_rpm

func get_current_gear() -> int:
	return _current_gear

func get_vehicle_state() -> GameState:
	return _state

func is_driving() -> bool:
	return _state == GameState.DRIVING

func is_skidding() -> bool:
	return _state == GameState.SKIDDING

func is_cruising() -> bool:
	return _state == GameState.DRIVING and abs(_current_speed) > 10.0

# ============================================================================
# INITIALIZATION
# ============================================================================

func _ready() -> void:
	_init_physics_components()
	_connect_signals()
	_calculate_aerodynamics()
	_setup_collision_detection()
	_load_default_settings()
	
	# Initialize velocity history for smooth rendering
	for i in range(10):
		_velocity_history.append(Vector3.ZERO)
	
	print("VehicleController initialized successfully")

func _init_physics_components() -> void:
	"""Initialize rigid body and wheel spring arms"""
	if _rigid_body:
		_rigid_body.mass = PhysicsSettings.default_vehicle_mass
		_rigid_body.gravity_scale = PhysicsSettings.gravity / 9.81
		_rigid_body.collision_layer = 1 << 2  # Vehicle layer
		_rigid_body.collision_mask = 1 << 0 | 1 << 1  # Ground + obstacles
	
	if _powertrain:
		_powertrain.set_physics_settings(PhysicsSettings)
		_powertrain.connect("rpm_updated", _on_powertrain_rpm_updated)
		_powertrain.connect("gear_shift_requested", _on_powertrain_gear_shift_requested)

func _connect_signals() -> void:
	"""Connect all internal signals"""
	speed_changed.connect(_on_speed_changed)
	rpm_changed.connect(_on_rpm_changed)
	gear_changed.connect(_on_gear_changed)
	skid_detected.connect(_on_skid_detected)
	crash_impact.connect(_on_crash_impact)
	
	# Connect to GameManager for state changes
	if GameManager:
		GameManager.game_state_changed.connect(_on_game_state_changed)

func _calculate_aerodynamics() -> void:
	"""Calculate aerodynamic drag based on vehicle dimensions"""
	_drag_force_constant = 0.5 * _drag_coefficient * _frontal_area * _air_density

func _setup_collision_detection() -> void:
	"""Setup collision detection for crashes and checkpoints"""
	if _collision_shape:
		_collision_shape.shape = BoxShape3D.new()
		_collision_shape.shape.size = Vector3(2.0, 1.0, 4.5)

func _load_default_settings() -> void:
	"""Load default vehicle settings from PhysicsSettings"""
	_tire_grip = clampf(PhysicsSettings.tire_friction_coefficient, 0.5, 1.0)
	_max_gear = clampi(PhysicsSettings.max_gears, 5, 8)
	_neutral_gear = 0
	_reverse_gear = -1

# ============================================================================
# INPUT HANDLING
# ============================================================================

func _process(delta: float) -> void:
	"""Main game loop - processes input and updates vehicle state"""
	_update_input_states(delta)
	_process_drivetrain(delta)
	_process_steering(delta)
	_update_physics(delta)
	_handle_state_transitions(delta)
	_update_debug_visuals(delta)

func _physics_process(delta: float) -> void:
	"""Physics update - runs at fixed timestep for consistent simulation"""
	if not Engine.is_editor_hint():
		_apply_forces_and_joints(delta)
		_check_wheel_suspension()
		_update_collision_detection()

func _update_input_states(delta: float) -> void:
	"""Read and validate input from InputManager"""
	_throttle_input = clampf(InputManager.get_axis("vehicle_throttle"), 0.0, 1.0)
	_brake_input = clampf(InputManager.get_axis("vehicle_brake"), 0.0, 1.0)
	_steering_input = clampf(InputManager.get_axis("vehicle_steering"), -1.0, 1.0)
	_clutch_input = clampf(InputManager.get_axis("vehicle_clutch"), 0.0, 1.0)
	_handbrake_input = clampf(InputManager.get_axis("vehicle_handbrake"), 0.0, 1.0)
	
	# Apply deadzone to steering
	if abs(_steering_input) < STEERING_DEADZONE:
		_steering_input = 0.0
	
	# Smooth input transitions
	_throttle_input = _lerp_inputs(_throttle_input, delta)
	_brake_input = _lerp_inputs(_brake_input, delta)
	_steering_input = _lerp_inputs(_steering_input, delta)

func _lerp_inputs(current: float, delta: float) -> float:
	"""Smoothly transition between input values"""
	var target = current
	var smoothing = 5.0  # Smoothing factor
	
	if abs(target - current) > 0.001:
		current = lerp(current, target, smoothing * delta)
	
	return clampf(current, 0.0, 1.0)

# ============================================================================
# DRIVETRAIN CONTROL
# ============================================================================

func _process_drivetrain(delta: float) -> void:
	"""Process throttle, braking, and gear shifting"""
	if _state == GameState.CRASHED or _state == GameState.RESETTING:
		return
	
	# Handle clutch engagement
	var clutch_engaged = _clutch_input < 0.2
	
	# Automatic gear shifting
	if not InputManager.get_action_pressed("vehicle_manual_shift"):
		_auto_shift_gears(delta, clutch_engaged)
	else:
		_handle_manual_shifting(clutch_engaged)
	
	# Calculate engine output
	if clutch_engaged and _current_gear != _neutral_gear:
		_apply_engine_force(delta)
		_update_engine_rpm(delta)
	else:
		_coast_to_idle(delta)
	
	# Handle braking
	if _brake_input > BRAKE_THRESHOLD:
		_apply_braking_force(delta)
		_set_state(GameState.BRAKING)
	else:
		_release_brakes()
	
	# Handbrake handling
	if _handbrake_input > 0.5:
		_apply_handbrake_force(delta)

func _auto_shift_gears(delta: float, clutch_engaged: bool) -> void:
	"""Automatic gear shifting based on RPM and speed"""
	var shift_up_threshold = 6500.0  # RPM
	var shift_down_threshold = 2500.0  # RPM
	var upshift_delay = 0.5  # Seconds
	
	if not clutch_engaged:
		return
	
	var should_upshift = false
	var should_downshift = false
	
	if _current_rpm > shift_up_threshold and _current_gear < _max_gear:
		should_upshift = true
	elif _current_rpm < shift_down_threshold and _current_gear > 1:
		should_downshift = true
	elif _current_speed < 10.0 and _current_gear > 1:
		should_downshift = true
	
	if should_upshift:
		_shift_up()
	elif should_downshift:
		_shift_down()

func _handle_manual_shifting(clutch_engaged: bool) -> void:
	"""Manual gear shifting with clutch"""
	if not clutch_engaged:
		return
	
	if InputManager.get_action_just_pressed("vehicle_shift_up"):
		_shift_up()
	elif InputManager.get_action_just_pressed("vehicle_shift_down"):
		_shift_down()

func _shift_up() -> void:
	"""Shift to next higher gear"""
	if _current_gear < _max_gear:
		var old_gear = _current_gear
		_current_gear += 1
		gear_changed.emit(old_gear, _current_gear)
		
		if _powertrain:
			_powertrain.shift_gear(_current_gear)

func _shift_down() -> void:
	"""Shift to next lower gear"""
	if _current_gear > 1:
		var old_gear = _current_gear
		_current_gear -= 1
		gear_changed.emit(old_gear, _current_gear)
		
		if _powertrain:
			_powertrain.shift_gear(_current_gear)

func _apply_engine_force(delta: float) -> void:
	"""Apply engine torque to wheels"""
	if _current_gear == _neutral_gear:
		return
	
	var gear_ratio = _gear_ratios[_current_gear]
	var total_ratio = gear_ratio * _final_drive_ratio
	var engine_torque = _calculate_engine_torque()
	var wheel_torque = engine_torque * total_ratio * 0.95  # 5% drivetrain loss
	
	var drive_wheels = [_wheel_rear_left, _wheel_rear_right] if _current_gear >= 1 else \
	                   [_wheel_front_left, _wheel_front_right]
	
	for wheel in drive_wheels:
		if wheel:
			wheel.apply_drive_torque(wheel_torque * _throttle_input)

func _calculate_engine_torque() -> float:
	"""Calculate current engine torque based on RPM"""
	var peak_torque_rpm = 4500.0
	var max_torque = 400.0  # Nm
	
	# Simple torque curve approximation
	var rpm_ratio = _current_rpm / peak_torque_rpm
	var torque_curve = exp(-pow(rpm_ratio - 1.0, 2) * 3.0) * max_torque
	
	return torque_curve * _throttle_input

func _update_engine_rpm(delta: float) -> void:
	"""Update engine RPM based on wheel speed and gear ratio"""
	if _current_gear == _neutral_gear:
		_current_rpm = lerp(_current_rpm, 800.0, 5.0 * delta)
		return
	
	var gear_ratio = _gear_ratios[_current_gear]
	var total_ratio = gear_ratio * _final_drive_ratio
	var wheel_radius = 0.33  # meters
	
	# Convert vehicle speed to wheel rotation
	var wheel_rotation_speed = _current_speed / wheel_radius
	_current_rpm = wheel_rotation_speed * total_ratio * 60.0 / (2.0 * PI)
	
	# Clamp RPM
	_current_rpm = clampf(_current_rpm, 800.0, 8500.0)
	rpm_changed.emit(_current_rpm)

func _coast_to_idle(delta: float) -> void:
	"""Engine idle when clutch disengaged or neutral"""
	var idle_rpm = 800.0
	var rev_drop_rate = 500.0  # RPM per second
	
	if _current_rpm > idle_rpm:
		_current_rpm = max(idle_rpm, _current_rpm - rev_drop_rate * delta)
	else:
		_current_rpm = idle_rpm
	
	rpm_changed.emit(_current_rpm)

func _apply_braking_force(delta: float) -> void:
	"""Apply braking force to all wheels"""
	var brake_pressure = _brake_input * 15000.0  # Brake pressure in Newtons
	
	# Apply to all four wheels
	var all_wheels = [_wheel_front_left, _wheel_front_right, 
	                  _wheel_rear_left, _wheel_rear_right]
	
	for wheel in all_wheels:
		if wheel:
			wheel.apply_brake_force(brake_pressure)

func _release_brakes() -> void:
	"""Release all brakes"""
	var all_wheels = [_wheel_front_left, _wheel_front_right, 
	                  _wheel_rear_left, _wheel_rear_right]
	
	for wheel in all_wheels:
		if wheel:
			wheel.release_brakes()

func _apply_handbrake_force(delta: float) -> void:
	"""Apply handbrake to rear wheels only"""
	var handbrake_force = _handbrake_input * 10000.0
	
	var rear_wheels = [_wheel_rear_left, _wheel_rear_right]
	
	for wheel in rear_wheels:
		if wheel:
			wheel.apply_brake_force(handbrake_force)

# ============================================================================
# STEERING CONTROL
# ============================================================================

func _process_steering(delta: float) -> void:
	"""Process steering input and apply to front wheels"""
	if _state == GameState.CRASHED or _state == GameState.RESETTING:
		return
	
	var steer_angle = _steering_input * 30.0 * deg_to_rad()  # Max 30 degrees
	
	# Front wheel steering
	if _wheel_front_left:
		_wheel_front_left.steer_angle = steer_angle
	if _wheel_front_right:
		_wheel_front_right.steer_angle = -steering_input * 30.0 * deg_to_rad()

func _check_wheel_suspension() -> void:
	"""Check and update wheel suspension compression"""
	var all_wheels = [_wheel_front_left, _wheel_front_right,
	                  _wheel_rear_left, _wheel_rear_right]
	
	for wheel in all_wheels:
		if wheel:
			wheel.check_suspension()

# ============================================================================
# PHYSICS UPDATE
# ============================================================================

func _apply_forces_and_joints(delta: float) -> void:
	"""Apply physical forces to vehicle body"""
	if _rigid_body:
		# Apply gravity
		_rigid_body.apply_central_impulse(Vector3.DOWN * PhysicsSettings.gravity * PhysicsSettings.default_vehicle_mass * delta)
		
		# Apply aerodynamic drag
		var velocity_magnitude = linear_velocity.length()
		if velocity_magnitude > 1.0:
			var drag_force = -linear_velocity.normalized() * _drag_force_constant * pow(velocity_magnitude, 2)
			_rigid_body.apply_central_force(drag_force)

func _update_collision_detection() -> void:
	"""Update collision detection for crashes and checkpoints"""
	var colliding = false
	
	for i in range(get_slide_collision_count()):
		var collision = get_slide_collision(i)
		var collider = collision.get_collider()
		
		if collider.has_method("is_obstacle") and collider.is_obstacle():
			colliding = true
			break
	
	if colliding and _state == GameState.DRIVING:
		_set_state(GameState.CRASHED)
		var impact_force = linear_velocity.length() * _rigid_body.mass
		crash_impact.emit(linear_velocity * impact_force)

# ============================================================================
# STATE MANAGEMENT
# ============================================================================

func _handle_state_transitions(delta: float) -> void:
	"""Handle transitions between vehicle states"""
	match _state:
		GameState.IDLE:
			_handle_idle_state(delta)
		GameState.DRIVING:
			_handle_driving_state(delta)
		GameState.BRAKING:
			_handle_braking_state(delta)
		GameState.SKIDDING:
			_handle_skidding_state(delta)
		GameState.CRASHED:
			_handle_crashed_state(delta)
		GameState.RESETTING:
			_handle_resetting_state(delta)

func _handle_idle_state(delta: float) -> void:
	"""Handle IDLE state"""
	if _throttle_input > ACCELERATION_THRESHOLD or _brake_input > BRAKE_THRESHOLD:
		_set_state(GameState.DRIVING)

func _handle_driving_state(delta: float) -> void:
	"""Handle DRIVING state"""
	var skid_intensity = _calculate_skid_intensity()
	
	if skid_intensity > SKID_THRESHOLD:
		_set_state(GameState.SKIDDING)
	elif _brake_input > BRAKE_THRESHOLD:
		_set_state(GameState.BRAKING)
	elif _throttle_input < ACCELERATION_THRESHOLD and _brake_input < BRAKE_THRESHOLD:
		_set_state(GameState.OVERSHOOT)

func _handle_braking_state(delta: float) -> void:
	"""Handle BRAKING state"""
	if _brake_input < BRAKE_THRESHOLD:
		_set_state(GameState.DRIVING)
	elif _current_speed < 1.0:
		_set_state(GameState.IDLE)

func _handle_skidding_state(delta: float) -> void:
	"""Handle SKIDDING state"""
	var skid_intensity = _calculate_skid_intensity()
	
	if skid_intensity < SKID_THRESHOLD:
		_set_state(GameState.DRIVING)

func _handle_crashed_state(delta: float) -> void:
	"""Handle CRASHED state"""
	pass  # Wait for reset signal

func _handle_resetting_state(delta: float) -> void:
	"""Handle RESETTING state"""
	pass  # Wait for completion

func _set_state(new_state: GameState) -> void:
	"""Change vehicle state with logging"""
	if _state != new_state:
		var old_state = _state
		_state = new_state
		vehicle_state_changed.emit(_state)
		print("Vehicle state changed: %s -> %s" % [GameState.keys()[old_state], GameState.keys()[new_state]])

# ============================================================================
# SKID DETECTION
# ============================================================================

func _calculate_skid_intensity() -> float:
	"""Calculate how much the vehicle is skidding"""
	var lateral_velocity = _get_lateral_velocity()
	var longitudinal_velocity = _current_speed
	
	if abs(longitudinal_velocity) < 1.0:
		return 0.0
	
	var skid_ratio = lateral_velocity.length() / abs(longitudinal_velocity)
	return clampf(skid_ratio, 0.0, 1.0)

func _get_lateral_velocity() -> Vector3:
	"""Get lateral velocity relative to vehicle heading"""
	var forward = global_transform.basis.z
	var velocity = linear_velocity
	var lateral = velocity - (velocity.dot(forward) * forward)
	return lateral

# ============================================================================
# DEBUG VISUALIZATION
# ============================================================================

func _update_debug_visuals(delta: float) -> void:
	"""Update debug visual elements"""
	if not DebugMode.enabled or not _debug_visuals:
		return
	
	# Update speed indicator
	if _debug_visuals.has_node("SpeedIndicator"):
		var indicator = _debug_visuals.get_node("SpeedIndicator")
		indicator.text = "%.1f km/h" % (_current_speed * 3.6)
	
	# Update RPM indicator
	if _debug_visuals.has_node("RPMIndicator"):
		var rpm_indicator = _debug_visuals.get_node("RPMIndicator")
		rpm_indicator.text = "%.0f RPM" % _current_rpm
	
	# Update gear indicator
	if _debug_visuals.has_node("GearIndicator"):
		var gear_indicator = _debug_visuals.get_node("GearIndicator")
		var gear_text = "N"
		match _current_gear:
			-1: gear_text = "R"
			0: gear_text = "N"
			_: gear_text = str(_current_gear)
		gear_indicator.text = gear_text

# ============================================================================
# SIGNAL HANDLERS
# ============================================================================

func _on_speed_changed(new_speed: float) -> void:
	"""Handle speed change events"""
	_current_speed = new_speed
	speed_changed.emit(new_speed)

func _on_rpm_changed(new_rpm: float) -> void:
	"""Handle RPM change events"""
	_current_rpm = new_rpm
	rpm_changed.emit(new_rpm)

func _on_gear_changed(old_gear: int, new_gear: int) -> void:
	"""Handle gear change events"""
	gear_changed.emit(old_gear, new_gear)

func _on_skid_detected(skid_intensity: float) -> void:
	"""Handle skid detection events"""
	if skid_intensity > SKID_THRESHOLD:
		skid_detected.emit(skid_intensity)

func _on_crash_impact(force_vector: Vector3) -> void:
	"""Handle crash impact events"""
	crash_impact.emit(force_vector)

func _on_game_state_changed(new_state: GameState) -> void:
	"""Handle game state changes from GameManager"""
	match new_state:
		GameManager.GameState.RACE_ACTIVE:
			_start_race()
		GameManager.GameState.RACE_PAUSED:
			_pause_vehicle()
		GameManager.GameState.RACE_FINISHED:
			_stop_vehicle()

func _start_race() -> void:
	"""Start race sequence"""
	_set_state(GameState.DRIVING)

func _pause_vehicle() -> void:
	"""Pause vehicle motion"""
	_set_state(GameState.IDLE)

func _stop_vehicle() -> void:
	"""Stop vehicle completely"""
	linear_velocity = Vector3.ZERO
	angular_velocity = Vector3.ZERO
	_set_state(GameState.IDLE)

# ============================================================================
# PUBLIC API
# ============================================================================

func set_gear_ratio(gear: int, ratio: float) -> void:
	"""Set custom gear ratio"""
	_gear_ratios[gear] = ratio

func set_final_drive_ratio(ratio: float) -> void:
	"""Set final drive ratio"""
	_final_drive_ratio = ratio

func set_tire_grip(grip: float) -> void:
	"""Set tire grip coefficient"""
	_tire_grip = clampf(grip, 0.3, 1.0)

func set_drag_coefficient(coefficient: float) -> void:
	"""Set aerodynamic drag coefficient"""
	_drag_coefficient = coefficient
	_calculate_aerodynamics()

func reset_vehicle() -> void:
	"""Reset vehicle to starting position"""
	_current_speed = 0.0
	_current_rpm = 800.0
	_current_gear = 1
	_state = GameState.IDLE
	
	if _rigid_body:
		_rigid_body.linear_velocity = Vector3.ZERO
		_rigid_body.angular_velocity = Vector3.ZERO

func get_vehicle_stats() -> Dictionary:
	"""Get comprehensive vehicle statistics"""
	return {
		"speed_kmh": _current_speed * 3.6,
		"rpm": _current_rpm,
		"gear": _current_gear,
		"state": GameState.keys()[_state],
		"throttle": _throttle_input,
		"brake": _brake_input,
		"steering": _steering_input,
		"skid_intensity": _calculate_skid_intensity(),
		"drift_mode": _handbrake_input > 0.5
	}

func is_valid_gear(gear: int) -> bool:
	"""Check if gear is valid"""
	return gear in _gear_ratios.keys()

func get_max_speed() -> float:
	"""Calculate theoretical maximum speed"""
	var max_rpm = 8500.0
	var gear_ratio = _gear_ratios[_max_gear]
	var total_ratio = gear_ratio * _final_drive_ratio
	var wheel_radius = 0.33
	
	var max_speed = (max_rpm * 2.0 * PI * wheel_radius) / (total_ratio * 60.0)
	return max_speed * 3.6  # Convert to km/h

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================

func _lerp(a: float, b: float, t: float) -> float:
	"""Linear interpolation helper"""
	return a + (b - a) * t

func _clamp(value: float, min_val: float, max_val: float) -> float:
	"""Clamp value to range"""
	return max(min_val, min(max_val, value))

func _abs(value: float) -> float:
	"""Absolute value helper"""
	return abs(value)

</file>