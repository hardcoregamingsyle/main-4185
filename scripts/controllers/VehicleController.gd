extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller using PhysicsSettings constants
## Handles throttle, brake, steering inputs, wheel forces, and gear shifting logic
## Copyright 2026 Thalamus Racing Simulator Project

# ============================================================================
# SIGNALS
# ============================================================================
signal engine_started
signal engine_stopped
signal gear_changed(old_gear: int, new_gear: int)
signal nitro_used(amount: float)
signal collision_detected(impact_force: float, collision_normal: Vector3)
signal wheel_slip(wheel_index: int, slip_ratio: float)
signal rpm_changed(current_rpm: float, max_rpm: float)
signal speed_changed(speed_kmh: float)

# ============================================================================
# EXPORTED PHYSICS SETTINGS (from PhysicsSettings resource)
# ============================================================================
@export_group("Vehicle Configuration")
@export var vehicle_mass: float = 1500.0
@export var center_of_mass_offset: Vector3 = Vector3.ZERO
@export var wheel_base: float = 2.8
@export var track_width: float = 1.6

@export_group("Engine & Powertrain")
@export var engine_max_rpm: float = 8000.0
@export var engine_idle_rpm: float = 800.0
@export var engine_peak_torque_rpm: float = 4500.0
@export var engine_peak_torque: float = 450.0  # Nm
@export var engine_redline: float = 7500.0
@export var nitro_capacity: float = 100.0
@export var nitro_usage_rate: float = 10.0  # per second when active

@export_group("Gear Ratios")
@export var final_drive_ratio: float = 3.73
@export var gear_ratios: Array[float] = [3.5, 2.1, 1.5, 1.1, 0.9, 0.7]
@export var reverse_ratio: float = 3.8

@export_group("Physics Constants from Resource")
@export var use_physics_settings: bool = true

@onready var _physics_settings: PhysicsSettings = $PhysicsSettings if has_node("$PhysicsSettings") else null

# ============================================================================
# INTERNAL STATE
# ============================================================================
var current_gear: int = 0  # 0 = neutral, 1-6 = forward gears, -1 = reverse
var target_gear: int = 0
var clutch_engaged: bool = true
var nitro_active: bool = false
var nitro_charge: float = 100.0
var engine_on: bool = false
var current_rpm: float = 0.0
var wheel_slip_ratios: Array[float] = []
var traction_control_active: bool = true

# Input values (-1.0 to 1.0)
var throttle_input: float = 0.0
var brake_input: float = 0.0
var steer_input: float = 0.0
var nitro_input: bool = false

# Vehicle state
var current_speed_kmh: float = 0.0
var acceleration_vector: Vector3 = Vector3.ZERO
var torque_vector: Vector3 = Vector3.ZERO

# Wheel references for force application
var front_left_wheel: Node3D = null
var front_right_wheel: Node3D = null
var rear_left_wheel: Node3D = null
var rear_right_wheel: Node3D = null

# ============================================================================
# INITIALIZATION
# ============================================================================
func _ready() -> void:
	_init_wheel_references()
	_reset_vehicle_state()
	_connect_signals_to_manager()
	
	if use_physics_settings and _physics_settings:
		_apply_physics_settings()

func _init_wheel_references() -> void:
	"""Find wheel nodes for force application"""
	front_left_wheel = get_node_or_null("WheelFrontLeft")
	front_right_wheel = get_node_or_null("WheelFrontRight")
	rear_left_wheel = get_node_or_null("WheelRearLeft")
	rear_right_wheel = get_node_or_null("WheelRearRight")

func _reset_vehicle_state() -> void:
	current_gear = 0
	target_gear = 0
	clutch_engaged = true
	nitro_active = false
	nitro_charge = nitro_capacity
	engine_on = false
	current_rpm = engine_idle_rpm
	wheel_slip_ratios = [0.0, 0.0, 0.0, 0.0]

# ============================================================================
# PHYSICS SETTINGS INTEGRATION
# ============================================================================
func _apply_physics_settings() -> void:
	if not _physics_settings:
		return
	
	vehicle_mass = _physics_settings.default_vehicle_mass
	engine_max_rpm = _physics_settings.engine_max_rpm
	engine_idle_rpm = _physics_settings.engine_idle_rpm
	engine_peak_torque = _physics_settings.engine_peak_torque
	engine_redline = _physics_settings.engine_redline * 0.95  # 95% of redline

# ============================================================================
# INPUT HANDLING
# ============================================================================
func _input(event: InputEvent) -> void:
	if event is InputEventKey:
		match event.keycode:
			KEY_SPACE:
				if event.pressed:
					_toggle_engine()
			KEY_N:
				if event.pressed:
					nitro_active = not nitro_active
			KEY_G:
				if event.pressed:
					_shift_gear_manual()

func _process(delta: float) -> void:
	_read_inputs()
	_update_engine_state(delta)
	_update_gear_system(delta)
	_update_nitro(delta)

func _physics_process(delta: float) -> void:
	apply_physics(delta)
	update_collision_detection()
	emit_vehicle_signals()

func _read_inputs() -> void:
	"""Read input from InputManager or direct input"""
	# Direct keyboard input fallback
	throttle_input = Input.get_action_strength("vehicle_throttle") - Input.get_action_strength("vehicle_brake")
	brake_input = Input.get_action_strength("vehicle_brake")
	steer_input = Input.get_axis("vehicle_steering_left", "vehicle_steering_right")
	nitro_input = Input.is_action_pressed("vehicle_nitro")
	
	# Clamp inputs
	throttle_input = clamp(throttle_input, -1.0, 1.0)
	brake_input = clamp(brake_input, 0.0, 1.0)
	steer_input = clamp(steer_input, -1.0, 1.0)

# ============================================================================
# ENGINE CONTROL
# ============================================================================
func toggle_engine(force_state: bool = false) -> void:
	if force_state:
		if force_state and not engine_on:
			_start_engine()
		elif not force_state and engine_on:
			_stop_engine()
	else:
		_toggle_engine()

func _toggle_engine() -> void:
	if engine_on:
		_stop_engine()
	else:
		_start_engine()

func _start_engine() -> void:
	engine_on = true
	current_rpm = engine_idle_rpm
	_set_gear(1)
	engine_started.emit()
	if AudioManager:
		AudioManager.play_sound("engine_start")

func _stop_engine() -> void:
	engine_on = false
	current_rpm = 0.0
	_set_gear(0)
	engine_stopped.emit()
	if AudioManager:
		AudioManager.play_sound("engine_stop")

func _update_engine_state(delta: float) -> void:
	if not engine_on:
		current_rpm = lerp(current_rpm, 0.0, delta * 10.0)
		return
	
	var target_rpm: float = _calculate_target_rpm()
	current_rpm = lerp(current_rpm, target_rpm, delta * 5.0)
	
	# Redline protection
	if current_rpm >= engine_redline:
		current_rpm = engine_redline
		_limit_power()
	
	rpm_changed.emit(current_rpm, engine_max_rpm)

func _calculate_target_rpm() -> float:
	if current_gear == 0:
		return engine_idle_rpm
	
	var speed_mps = current_speed_kmh / 3.6
	var gear_ratio = gear_ratios[current_gear - 1] if current_gear > 0 else reverse_ratio
	var wheel_radius: float = 0.3  # Approximate
	
	var drive_wheel_rpm = (speed_mps * 60.0) / (2.0 * PI * wheel_radius)
	var engine_rpm = drive_wheel_rpm * gear_ratio * final_drive_ratio
	
	# Calculate based on throttle
	var idle_rpm = engine_idle_rpm
	var max_rpm = engine_max_rpm
	var rpm_range = max_rpm - idle_rpm
	
	var throttle_factor = abs(throttle_input)
	var target = idle_rpm + (rpm_range * throttle_factor)
	
	# Blend between load-based and throttle-based RPM
	return lerp(engine_rpm, target, 0.5)

func _limit_power() -> void:
	"""Cut power when hitting redline"""
	if throttle_input > 0.1:
		throttle_input *= 0.95  # Reduce throttle slightly

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================
func set_gear(gear: int) -> void:
	_set_gear(gear)

func _set_gear(new_gear: int) -> void:
	if new_gear < -1 or new_gear > 6:
		return
	
	var old_gear = current_gear
	current_gear = new_gear
	target_gear = new_gear
	
	gear_changed.emit(old_gear, new_gear)
	if AudioManager:
		AudioManager.play_sound("gear_shift")

func shift_gear_up() -> void:
	if current_gear < 6:
		_set_gear(current_gear + 1)

func shift_gear_down() -> void:
	if current_gear > 0:
		_set_gear(current_gear - 1)
	elif current_gear == 0 and Engine.has_method("can_reverse"):
		_set_gear(-1)

func _shift_gear_manual() -> void:
	if Input.is_key_pressed(KEY_UP):
		shift_gear_up()
	elif Input.is_key_pressed(KEY_DOWN):
		shift_gear_down()

func _update_gear_system(delta: float) -> void:
	if not engine_on:
		return
	
	# Automatic upshift at redline
	if current_gear < 6 and current_rpm >= engine_redline * 0.95:
		_set_gear(current_gear + 1)
	
	# Automatic downshift at low RPM
	if current_gear > 1 and current_rpm < engine_idle_rpm * 1.2:
		_set_gear(current_gear - 1)
	
	# Downshift to neutral when stopping
	if current_speed_kmh < 2.0 and current_gear != 0:
		_set_gear(0)

func get_current_gear() -> int:
	return current_gear

func get_gear_ratio() -> float:
	if current_gear == 0:
		return 0.0
	elif current_gear > 0:
		return gear_ratios[current_gear - 1]
	else:  # Reverse
		return reverse_ratio

# ============================================================================
# WHEEL FORCE APPLICATION
# ============================================================================
func apply_physics(delta: float) -> void:
	if not engine_on:
		_apply_drag_and_friction(delta)
		return
	
	var gear_ratio = get_gear_ratio()
	if gear_ratio == 0.0:
		_apply_drag_and_friction(delta)
		return
	
	# Calculate engine torque
	var engine_torque = _calculate_engine_torque()
	var drive_torque = engine_torque * gear_ratio * final_drive_ratio
	
	# Apply torque to wheels (RWD configuration by default)
	_apply_wheel_forces(drive_torque, delta)
	
	# Apply braking if needed
	if brake_input > 0.0:
		_apply_braking(brake_input, delta)
	
	# Apply steering
	_apply_steering(steer_input, delta)
	
	# Update speed
	_update_speed(delta)

func _calculate_engine_torque() -> float:
	"""Calculate torque based on RPM curve"""
	if current_rpm <= engine_idle_rpm:
		return 0.0
	
	var rpm_normalized = (current_rpm - engine_idle_rpm) / (engine_peak_torque_rpm - engine_idle_rpm)
	
	# Torque curve (parabolic peak around peak torque RPM)
	var torque_curve = 1.0 - pow((rpm_normalized - 1.0), 2)
	torque_curve = max(0.0, torque_curve)
	
	# Apply throttle multiplier
	var throttle_multiplier = clamp(throttle_input, 0.0, 1.0)
	
	return engine_peak_torque * torque_curve * throttle_multiplier

func _apply_wheel_forces(torque: float, delta: float) -> void:
	"""Apply driving torque to rear wheels"""
	if torque <= 0.0:
		return
	
	var force_per_wheel = torque / (rear_left_wheel ? rear_left_wheel.position.y : 0.15)
	force_per_wheel *= delta
	
	# Apply force vector to rear wheels (simplified)
	var drive_direction = transform.basis.z * -1.0
	
	if rear_left_wheel:
		rear_left_wheel.apply_local_force(drive_direction * force_per_wheel * 0.5)
	if rear_right_wheel:
		rear_right_wheel.apply_local_force(drive_direction * force_per_wheel * 0.5)

func _apply_braking(brake_amount: float, delta: float) -> void:
	"""Apply braking force to all wheels"""
	var brake_force = 5000.0 * brake_amount * delta
	
	for wheel in [front_left_wheel, front_right_wheel, rear_left_wheel, rear_right_wheel]:
		if wheel:
			wheel.apply_local_force(transform.basis.z * brake_force)

func _apply_steering(steer_amount: float, delta: float) -> void:
	"""Apply steering rotation to front wheels"""
	var steer_angle = steer_amount * 30.0 * deg_to_rad()
	
	if front_left_wheel:
		front_left_wheel.rotate_y(steer_angle * delta)
	if front_right_wheel:
		front_right_wheel.rotate_y(steer_angle * delta)

func _apply_drag_and_friction(delta: float) -> void:
	"""Apply air resistance and rolling friction"""
	var drag_coefficient = 0.3
	var air_density = 1.225
	
	var velocity = global_velocity
	var speed_squared = velocity.length_squared()
	
	if speed_squared > 0.0:
		var drag_force = -velocity.normalized() * drag_coefficient * air_density * speed_squared * delta * 0.1
		velocity += drag_force
		
		var friction_force = -velocity.normalized() * 500.0 * delta
		velocity += friction_force
	
	global_velocity = velocity

func _update_speed(delta: float) -> void:
	"""Update vehicle speed from velocity"""
	current_speed_kmh = global_velocity.length() * 3.6
	speed_changed.emit(current_speed_kmh)

# ============================================================================
# NITRO SYSTEM
# ============================================================================
func _update_nitro(delta: float) -> void:
	if nitro_active and nitro_charge > 0.0:
		nitro_charge -= nitro_usage_rate * delta
		if nitro_charge <= 0.0:
			nitro_active = false
			nitro_charge = 0.0
	
	if nitro_active:
		_apply_nitro_boost(delta)

func _apply_nitro_boost(delta: float) -> void:
	"""Apply temporary power boost"""
	var boost_factor = 1.5
	var current_torque = _calculate_engine_torque()
	var boosted_torque = current_torque * boost_factor
	
	var gear_ratio = get_gear_ratio()
	var drive_torque = boosted_torque * gear_ratio * final_drive_ratio
	
	_apply_wheel_forces(drive_torque, delta)
	
	if nitro_charge <= 0.0:
		nitro_active = false
		nitro_used.emit(nitro_capacity)
		if AudioManager:
			AudioManager.play_sound("nitro_end")

func refill_nitro() -> void:
	nitro_charge = nitro_capacity

# ============================================================================
# COLLISION DETECTION
# ============================================================================
func update_collision_detection() -> void:
	"""Check for collisions and emit signals"""
	for collision in get_colliding_bodies():
		var contact_point = get_collision_contact_point()
		var impact_velocity = global_velocity.dot(contact_point.normal)
		
		if impact_velocity > 5.0:  # Significant impact
			collision_detected.emit(impact_velocity, contact_point.normal)
			if AudioManager:
				AudioManager.play_sound("collision_impact")

func get_collision_info() -> Dictionary:
	return {
		"colliding": get_colliding_bodies().size() > 0,
		"contact_normal": get_collision_normal(),
		"impact_velocity": global_velocity.length()
	}

# ============================================================================
# VEHICLE SIGNALS
# ============================================================================
func emit_vehicle_signals() -> void:
	"""Emit periodic state updates"""
	if frame_count % 30 == 0:  # Every ~0.5 seconds
		_check_wheel_slip()

func _check_wheel_slip() -> void:
	"""Monitor wheel slip ratios"""
	for i in range(4):
		var slip = _calculate_wheel_slip(i)
		wheel_slip_ratios[i] = slip
		
		if abs(slip) > 0.3:  # Significant slip detected
			wheel_slip.emit(i, slip)
			if traction_control_active and slip > 0.3:
				_apply_traction_control()

func _calculate_wheel_slip(wheel_index: int) -> float:
	"""Calculate slip ratio for a wheel"""
	var wheel_speed = 0.0  # Simplified calculation
	var vehicle_speed = current_speed_kmh / 3.6
	
	if vehicle_speed > 0.0:
		var slip = (wheel_speed - vehicle_speed) / vehicle_speed
		return clamp(slip, -1.0, 1.0)
	
	return 0.0

func _apply_traction_control() -> void:
	"""Reduce torque to prevent excessive slip"""
	if throttle_input > 0.0:
		throttle_input *= 0.8  # Reduce throttle by 20%

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================
func reset_vehicle() -> void:
	_reset_vehicle_state()
	position = Vector3.ZERO
	rotation = Vector3.ZERO
	global_velocity = Vector3.ZERO

func get_vehicle_state() -> Dictionary:
	return {
		"gear": current_gear,
		"rpm": current_rpm,
		"speed_kmh": current_speed_kmh,
		"throttle": throttle_input,
		"brake": brake_input,
		"steer": steer_input,
		"nitro_charge": nitro_charge,
		"engine_on": engine_on,
		"clutch_engaged": clutch_engaged
	}

func connect_signals_to_manager() -> void:
	"""Connect to GameManager and AudioManager"""
	if GameManager:
		GameManager.game_state_changed.connect(_on_game_state_changed)
	if AudioManager:
		pass  # Already connected via signals

func _on_game_state_changed(new_state: GameManager.GameState) -> void:
	match new_state:
		GameManager.GameState.RACE_ACTIVE:
			engine_on = true
		GameManager.GameState.LOADING:
			reset_vehicle()

# ============================================================================
# DESTRUCTOR
# ============================================================================
func _exit_tree() -> void:
	engine_stopped.emit()
	pass  # Cleanup resources