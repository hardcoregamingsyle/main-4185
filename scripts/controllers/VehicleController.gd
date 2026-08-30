extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulator
## Implements throttle, brake, steering inputs, wheel forces, and gear shifting logic
## Uses PhysicsSettings constants for all tuning values
## Copyright 2026 Thalamus Racing Simulator Project

# ============================================================================
# SIGNALS
# ============================================================================
signal speed_changed(new_speed: float)
signal gear_changed(old_gear: int, new_gear: int)
signal vehicle_state_changed(state: VehicleState)
signal wheel_slip_detected(wheel_index: int, slip_ratio: float)
signal collision_impact(force: Vector3, position: Vector3)

# ============================================================================
# ENUMS & CONSTANTS
# ============================================================================
enum VehicleState {
	DRIVING,
	BRAKING,
	ACCELERATING,
	COASTING,
	SKIDDED,
	JUMPED,
	CRASHED,
	REV_MATCHING
}

enum WheelType {
	FRONT_LEFT = 0,
	FRONT_RIGHT,
	REAR_LEFT,
	REAR_RIGHT
}

const MAX_SPEED_KMH: float = 350.0
const ACCELERATION_RATE: float = 15.0
const BRAKING_FORCE: float = 25.0
const STEERING_ANGLE_MAX: float = 45.0 * deg_to_rad(1)
const TURN_SPEED: float = 3.0
const GRIP_LEVEL: float = 1.5

# ============================================================================
# EXPORTED VARIABLES (Tunable in Inspector)
# ============================================================================
@export_group("Vehicle Configuration")
@export var vehicle_mass: float = 1500.0
@export var center_of_mass_offset: Vector3 = Vector3.ZERO
@export var wheel_base: float = 2.8
@export var track_width: float = 1.6
@export var suspension_travel: float = 0.15

@export_group("Powertrain Settings")
@export var max_rpm: float = 8000.0
@export var idle_rpm: float = 800.0
@export var redline_rpm: float = 7500.0
@export var transmission_type: TransmissionType = TransmissionType.MANUAL

@export_group("Wheel Settings")
@export var tire_friction: float = 1.2
@export var suspension_stiffness: float = 50000.0
@export var damping_rate: float = 5000.0

@export_group("Visual Debug")
@export var debug_draw: bool = false
@export var show_hitboxes: bool = false

# ============================================================================
# INTERNAL STATE
# ============================================================================
var _current_speed: float = 0.0
var _current_rpm: float = idle_rpm
var _target_gear: int = 0
var _current_gear: int = 0
var _throttle_input: float = 0.0
var _brake_input: float = 0.0
var _steering_input: float = 0.0
var _clutch_input: float = 1.0
var _vehicle_state: VehicleState = VehicleState.DRIVING
var _engine_running: bool = true
var _is_revving: bool = false
var _gear_shift_progress: float = 0.0
var _last_collision_time: float = 0.0

# Wheel state tracking
var _wheel_states: Array[Dictionary] = []
var _suspension_compression: Array[float] = [0.0, 0.0, 0.0, 0.0]

# Powertrain reference
var powertrain_node: Node = null

# ============================================================================
# TRANSMISSION TYPES
# ============================================================================
enum TransmissionType {
	MANUAL,
	AUTOMATIC,
	SEMI_AUTOMATIC
}

# Gear ratios for different transmission types
var _gear_ratios_manual: Array[float] = [
	3.5,  # 1st
	2.2,  # 2nd
	1.6,  # 3rd
	1.2,  # 4th
	0.95, # 5th
	0.75, # 6th
	0.6   # Reverse
]

var _gear_ratios_auto: Array[float] = [
	3.0,  # 1st
	2.0,  # 2nd
	1.5,  # 3rd
	1.1,  # 4th
	0.85, # 5th
	0.7   # Overdrive
]

func _ready() -> void:
	_init_wheel_states()
	_connect_signals()
	_setup_powertrain()
	
	# Apply physics settings from singleton
	if Engine.has_singleton("PhysicsSettings"):
		var settings: Resource = Engine.get_singleton("PhysicsSettings")
		_apply_settings(settings)

func _init_wheel_states() -> void:
	for i in range(4):
		_wheel_states.append({
			"position": Vector3.ZERO,
			"force": Vector3.ZERO,
			"slip_ratio": 0.0,
			"angular_velocity": 0.0,
			"is_braking": false,
			"is_locked": false
		})

func _setup_powertrain() -> void:
	# Try to find powertrain node in children
	var children = get_children()
	for child in children:
		if child is Node and child.name.contains("Powertrain"):
			powertrain_node = child
			break
	
	# If no powertrain found, create default behavior
	if powertrain_node == null:
		print("[VehicleController] No Powertrain node found, using basic physics simulation")

func _connect_signals() -> void:
	# Connect to GameManager if available
	if GameManager:
		GameManager.game_state_changed.connect(_on_game_state_changed)

func _on_game_state_changed(new_state: GameManager.GameState) -> void:
	match new_state:
		GameManager.GameState.RACE_ACTIVE:
			_engine_running = true
			_current_gear = 1
		GameManager.GameState.RACE_PAUSED:
			_is_revving = false
		GameManager.GameState.MAIN_MENU:
			_engine_running = false
			_current_rpm = idle_rpm

func _apply_settings(settings: Resource) -> void:
	if settings != null:
		vehicle_mass = settings.default_vehicle_mass
		max_rpm = settings.max_engine_rpm
		idle_rpm = settings.idle_engine_rpm
		redline_rpm = settings.redline_engine_rpm

# ============================================================================
# INPUT HANDLING
# ============================================================================
func _physics_process(delta: float) -> void:
	_handle_inputs(delta)
	_update_physics(delta)
	_update_visual_debug()
	
	set_position(get_position())
	set_velocity(get_velocity())

func _handle_inputs(delta: float) -> void:
	# Get input values from InputManager
	_throttle_input = InputManager.get_axis("throttle_forward", "throttle_backward")
	_brake_input = InputManager.get_axis("brake_forward", "brake_backward")
	_steering_input = InputManager.get_axis("steer_left", "steer_right")
	
	# Handle clutch for manual transmission
	if transmission_type == TransmissionType.MANUAL:
		_clutch_input = InputManager.get_axis("clutch", "")
	else:
		_clutch_input = 1.0
	
	# Handle gear shift inputs
	_handle_gear_shifts(delta)
	
	# Clamp inputs to valid ranges
	_throttle_input = clamp(_throttle_input, -1.0, 1.0)
	_brake_input = clamp(_brake_input, -1.0, 1.0)
	_steering_input = clamp(_steering_input, -1.0, 1.0)

func _handle_gear_shifts(delta: float) -> void:
	if not _engine_running:
		return
	
	# Manual transmission requires clutch
	if transmission_type == TransmissionType.MANUAL and _clutch_input < 0.9:
		return
	
	# Upshift detection
	if InputManager.is_action_just_pressed("gear_up"):
		_attempt_gear_shift(1)
	
	# Downshift detection
	if InputManager.is_action_just_pressed("gear_down"):
		_attempt_gear_shift(-1)

func _attempt_gear_shift(direction: int) -> void:
	var new_gear = _current_gear + direction
	
	# Validate gear number
	if direction > 0:
		new_gear = min(new_gear, len(_gear_ratios_manual) - 1)
	else:
		new_gear = max(new_gear, 0)
	
	if new_gear != _current_gear:
		_target_gear = new_gear
		_gear_shift_progress = 0.0
		emit_signal("gear_changed", _current_gear, new_gear)

func _update_gear_shifting(delta: float) -> void:
	if _target_gear != _current_gear and _clutch_input < 0.1:
		_gear_shift_progress += delta / 0.3  # 0.3 second shift time
		
		if _gear_shift_progress >= 1.0:
			_current_gear = _target_gear
			_gear_shift_progress = 0.0
			_is_revving = true
			
			# Rev matching
			if _target_gear < _current_gear:
				_target_rpm = _calculate_target_rpm_for_gear(_target_gear)
			
			emit_signal("gear_changed", _current_gear, _target_gear)

func _calculate_target_rpm_for_gear(gear: int) -> float:
	if gear == 0:
		return idle_rpm
	
	var current_speed_ms = _current_speed / 3.6
	var ratio = _gear_ratios_manual[gear]
	
	# Calculate engine RPM based on wheel speed and gear ratio
	var wheel_circumference = 2.0 * PI * 0.33  # ~0.66m radius tires
	var wheel_rps = current_speed_ms / wheel_circumference
	var target_rpm = wheel_rps * ratio * 60.0
	
	return clamp(target_rpm, idle_rpm, redline_rpm)

# ============================================================================
# PHYSICS SIMULATION
# ============================================================================
func _update_physics(delta: float) -> void:
	if not _engine_running:
		_coast_to_stop(delta)
		return
	
	_update_engine_rpm(delta)
	_calculate_forces(delta)
	_apply_forces(delta)
	_check_collisions()

func _update_engine_rpm(delta: float) -> void:
	var gear_ratio = _get_current_gear_ratio()
	var wheel_speed = _current_speed / (2.0 * PI * 0.33)  # Tire radius approx 0.33m
	var theoretical_rpm = wheel_speed * gear_ratio * 60.0
	
	# Engine acceleration based on throttle and gear
	var rpm_change = 0.0
	
	if _throttle_input > 0.0:
		rpm_change = _throttle_input * ACCELERATION_RATE * delta
	elif _brake_input > 0.0:
		rpm_change = -BRAKING_FORCE * delta
	else:
		rpm_change = -idle_rpm * 0.1 * delta
	
	_current_rpm += rpm_change
	_current_rpm = clamp(_current_rpm, idle_rpm, max_rpm)
	
	# Redline protection
	if _current_rpm >= redline_rpm:
		_current_rpm = redline_rpm
		_is_revving = true

func _get_current_gear_ratio() -> float:
	var ratios = _gear_ratios_manual if transmission_type == TransmissionType.MANUAL else _gear_ratios_auto
	return ratios[min(_current_gear, len(ratios) - 1)]

func _calculate_forces(delta: float) -> void:
	var forward_vector = transform.basis.z * -1.0
	var lateral_vector = transform.basis.x
	
	# Calculate driving force based on gear and throttle
	var gear_ratio = _get_current_gear_ratio()
	var driving_force = 0.0
	
	if _current_gear > 0:
		driving_force = _throttle_input * ACCELERATION_RATE * gear_ratio * vehicle_mass
	
	# Calculate braking force
	var braking_force = _brake_input * BRAKING_FORCE * vehicle_mass
	
	# Calculate steering effect
	var steering_effect = _steering_input * TURN_SPEED * sin(_current_speed / 50.0)
	
	# Update velocity components
	var velocity = get_velocity()
	
	# Forward/Backward movement
	velocity.z -= driving_force * delta
	velocity.z += braking_force * delta
	
	# Lateral movement (steering)
	velocity.x += steering_effect * _current_speed * delta
	
	# Apply gravity
	velocity.y -= PhysicsSettings.gravity * delta
	
	# Update current speed
	_current_speed = velocity.length() * 3.6  # Convert to km/h
	
	# Emit signal
	if abs(_current_speed - speed_changed.get_value()) > 1.0:
		emit_signal("speed_changed", _current_speed)

func _apply_forces(delta: float) -> void:
	var forces_applied = false
	
	# Apply wheel forces
	for wheel_idx in range(4):
		var wheel_force = _calculate_wheel_force(wheel_idx, delta)
		
		if wheel_force != Vector3.ZERO:
			_apply_wheel_force(wheel_idx, wheel_force, delta)
			forces_applied = true
	
	# Apply drag and air resistance
	_apply_air_resistance(delta)

func _calculate_wheel_force(wheel_idx: int, delta: float) -> Vector3:
	var wheel_pos = _get_wheel_position(wheel_idx)
	var forward_dir = transform.basis.z * -1.0
	
	var force = Vector3.ZERO
	
	# Drive wheels (rear wheels for RWD)
	if wheel_idx == WheelType.REAR_LEFT or wheel_idx == WheelType.REAR_RIGHT:
		force = forward_dir * _throttle_input * ACCELERATION_RATE
		
	# Steering wheels (front wheels)
	if wheel_idx == WheelType.FRONT_LEFT or wheel_idx == WheelType.FRONT_RIGHT:
		var steer_angle = _steering_input * STEERING_ANGLE_MAX
		var steer_direction = transform.basis.x.rotated(Vector3.UP, steer_angle)
		force = steer_direction * _throttle_input * 0.5
	
	# Braking applies to all wheels
	if _brake_input > 0:
		force = -forward_dir * _brake_input * BRAKING_FORCE * 0.5
	
	return force.normalized() * force.length()

func _apply_wheel_force(wheel_idx: int, force: Vector3, delta: float) -> void:
	# Apply force at wheel position
	var wheel_pos = _get_wheel_position(wheel_idx)
	add_force(force * vehicle_mass, wheel_pos)

func _get_wheel_position(wheel_idx: int) -> Vector3:
	var offset_x = track_width / 2.0
	var offset_y = suspension_travel / 2.0
	var offset_z = wheel_base / 2.0
	
	var positions = [
		Vector3(-offset_x, -offset_y, -offset_z),      # Front Left
		Vector3(offset_x, -offset_y, -offset_z),       # Front Right
		Vector3(-offset_x, -offset_y, offset_z),       # Rear Left
		Vector3(offset_x, -offset_y, offset_z)         # Rear Right
	]
	
	return global_transform * positions[wheel_idx]

func _apply_air_resistance(delta: float) -> void:
	var velocity = get_velocity()
	var speed = velocity.length()
	
	if speed > 0:
		var drag_coefficient = 0.3
		var air_density = 1.225
		var frontal_area = 2.0
		var drag_force = -velocity.normalized() * 0.5 * air_density * drag_coefficient * frontal_area * speed * speed
		
		add_force(drag_force * delta)

func _coast_to_stop(delta: float) -> void:
	var velocity = get_velocity()
	var speed = velocity.length()
	
	if speed > 0.1:
		var friction = 0.98
		velocity *= friction
		set_velocity(velocity)
	else:
		set_velocity(Vector3.ZERO)
		_current_speed = 0.0

func _check_collisions() -> void:
	for col in get_slide_collision_count():
		var collision = get_slide_collision(col)
		var body = collision.get_collider()
		
		if body:
			var impact_force = collision.get_normal() * collision.get_impact_point()
			
			_last_collision_time = Time.get_unix_time_from_system()
			emit_signal("collision_impact", impact_force, collision.get_position())
			
			# Apply crash effects
			if _current_speed > 20.0:
				_vehicle_state = VehicleState.CRASHED
				_audio_manager.play_sound("crash")

# ============================================================================
# VISUAL DEBUG & HITBOXES
# ============================================================================
func _update_visual_debug() -> void:
	if not debug_draw:
		return
	
	# Draw wheel positions
	for wheel_idx in range(4):
		var pos = _get_wheel_position(wheel_idx)
		Debug.draw_sphere(pos, 0.2, Color.GREEN)
	
	# Draw center of mass
	var com_pos = global_transform.origin + center_of_mass_offset
	Debug.draw_sphere(com_pos, 0.1, Color.YELLOW)
	
	# Draw velocity vector
	if _current_speed > 1.0:
		var vel_dir = get_velocity().normalized()
		Debug.draw_arrow(global_transform.origin, global_transform.origin + vel_dir * 2.0, Color.RED)

func _draw_hitboxes() -> void:
	if not show_hitboxes:
		return
	
	# Draw bounding box
	var size = Vector3(track_width, 1.0, wheel_base + 0.5)
	Debug.draw_box(global_transform.origin, size, Color.CYAN)

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================
func start_engine() -> void:
	_engine_running = true
	_current_rpm = idle_rpm
	_vehicle_state = VehicleState.DRIVING
	emit_signal("vehicle_state_changed", _vehicle_state)
	_audio_manager.play_sound("engine_start")

func stop_engine() -> void:
	_engine_running = false
	_current_rpm = idle_rpm
	_vehicle_state = VehicleState.COASTING
	emit_signal("vehicle_state_changed", _vehicle_state)
	_audio_manager.play_sound("engine_stop")

func reset_vehicle() -> void:
	_current_speed = 0.0
	_current_rpm = idle_rpm
	_current_gear = 1
	_throttle_input = 0.0
	_brake_input = 0.0
	_steering_input = 0.0
	_vehicle_state = VehicleState.DRIVING
	set_velocity(Vector3.ZERO)

func get_speed_kmh() -> float:
	return _current_speed

func get_rpm() -> float:
	return _current_rpm

func get_gear() -> int:
	return _current_gear

func get_vehicle_state() -> VehicleState:
	return _vehicle_state

func is_engine_running() -> bool:
	return _engine_running

func is_in_gear() -> bool:
	return _current_gear > 0

func can_shift_gear() -> bool:
	return _clutch_input < 0.1 and _engine_running

func apply_damage(damage_amount: float) -> void:
	# Reduce vehicle performance based on damage
	vehicle_mass = max(vehicle_mass * 0.9, 100.0)
	ACCELERATION_RATE *= 0.8
	BRAKING_FORCE *= 0.7
	
	# Visual feedback
	_audio_manager.play_sound("damage")

# ============================================================================
# AUTOPLAY SUPPORT FOR AUDIO
# ============================================================================
func resume_audio_context() -> void:
	if AudioManager and AudioManager.audio_context:
		AudioManager.audio_context.resume()

# ============================================================================
# EXPOSED PROPERTIES FOR INSPECTOR
# ============================================================================
func _get_property_list() -> Array[Dictionary]:
	return [
		{
			"name": "speed_kmh",
			"type": TYPE_FLOAT,
			"usage": PROPERTY_USAGE_NO_EDITOR | PROPERTY_USAGE_SCRIPT_VARIABLE
		},
		{
			"name": "rpm",
			"type": TYPE_FLOAT,
			"usage": PROPERTY_USAGE_NO_EDITOR | PROPERTY_USAGE_SCRIPT_VARIABLE
		},
		{
			"name": "current_gear",
			"type": TYPE_INT,
			"usage": PROPERTY_USAGE_NO_EDITOR | PROPERTY_USAGE_SCRIPT_VARIABLE
		},
		{
			"name": "vehicle_state",
			"type": TYPE_INT,
			"usage": PROPERTY_USAGE_NO_EDITOR | PROPERTY_USAGE_SCRIPT_VARIABLE
		}
	]