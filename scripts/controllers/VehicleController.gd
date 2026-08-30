extends Node2D
class_name VehicleController

## VehicleController - Core vehicle physics controller using PhysicsSettings constants
## Handles throttle, brake, steering inputs, wheel forces, and gear shifting logic
## Copyright 2026 Thalamus Racing Simulator Project

# Signals
signal engine_started
signal engine_stopped
signal gear_changed(old_gear: int, new_gear: int)
signal nitro_used(amount: float)
signal collision_detected(direction: Vector2)
signal speed_changed(current_speed: float, max_speed: float)
signal rpm_changed(current_rpm: float, max_rpm: float)
signal vehicle_moved(distance: float)

# References
@onready var powertrain: Powertrain = $Powertrain if get_node_or_null("Powertrain") else null
@onready var chassis: Chassis = $Chassis if get_node_or_null("Chassis") else null
var _physics_body: CharacterBody2D = null

# State
enum VehicleState { IDLE, RUNNING, REVVING, BRAKING, COLLIDED, DRIFTING }
var current_vehicle_state: VehicleState = VehicleState.IDLE
var last_collision_time: float = 0.0
var collision_damping: float = 0.3

# Speed and movement
var current_speed: float = 0.0
var target_speed: float = 0.0
var acceleration: float = 0.0
var braking_force: float = 0.0
var steering_angle: float = 0.0
var total_distance_traveled: float = 0.0

# Gear management
var current_gear: int = 0  # 0 = neutral, 1-6 = forward gears, -1 = reverse
var rpm: float = 0.0
var max_rpm: float = 8000.0
var idle_rpm: float = 800.0

# Nitrous system
var nitro_available: bool = true
var nitro_amount: float = 100.0
var nitro_cooldown: float = 0.0
const NITRO_BONUS: float = 1.5
const NITRO_CONSUMPTION_RATE: float = 50.0

# Input handling
var throttle_input: float = 0.0
var brake_input: float = 0.0
var steering_input: float = 0.0
var shift_up_request: bool = false
var shift_down_request: bool = false

# Physics settings reference
var _settings: PhysicsSettings = null

# Constants
const MIN_SPEED_FOR_GEAR_CHANGE: float = 5.0
const GEAR_RATIOS: Array[float] = [0.0, 3.5, 2.8, 2.1, 1.6, 1.2, 0.9, -3.8]
const MAX_STEERING_ANGLE: float = deg_to_rad(45.0)
const STEERING_SENSITIVITY: float = 0.6
const ACCELERATION_RATE: float = 1500.0
const BRAKE_ACCELERATION: float = 3000.0
const FRICTION_COEFFICIENT: float = 0.85
const AIR_RESISTANCE: float = 0.02
const TORQUE_MULTIPLIER: float = 0.9
const CLUTCH_ENGAGE_THRESHOLD: float = 0.3

func _init() -> void:
	"""Initialize physics controller with default values."""
	_settings = PhysicsSettings.new()

func _ready() -> void:
	"""Initialize vehicle controller when ready."""
	_connect_signals()
	_init_physics_body()
	_reset_vehicle()
	emit_signal("engine_started")

func _connect_signals() -> void:
	"""Connect all required signals."""
	if powertrain:
		powertrain.engine_started.connect(_on_engine_started)
		powertrain.engine_stopped.connect(_on_engine_stopped)
		powertrain.rpm_changed.connect(_update_rpm_display)

func _init_physics_body() -> void:
	"""Initialize reference to physics body."""
	_physics_body = get_parent() as CharacterBody2D
	if not _physics_body:
		push_warning("VehicleController: No CharacterBody2D parent found!")

func _reset_vehicle() -> void:
	"""Reset vehicle to initial state."""
	current_speed = 0.0
	target_speed = 0.0
	acceleration = 0.0
	braking_force = 0.0
	steering_angle = 0.0
	total_distance_traveled = 0.0
	current_gear = 0
	rpm = idle_rpm
	nitro_available = true
	nitro_amount = 100.0
	nitro_cooldown = 0.0
	current_vehicle_state = VehicleState.IDLE

# ============================================================================
# INPUT PROCESSING
# ============================================================================

func _process(delta: float) -> void:
	"""Process input and update vehicle state."""
	_handle_inputs(delta)
	_update_physics(delta)
	_check_gear_limits()
	_update_nitro(delta)
	_emit_signals()

func _handle_inputs(delta: float) -> void:
	"""Handle all input processing."""
	throttle_input = clamp(InputManager.get_axis("throttle", "brake"), 0.0, 1.0)
	brake_input = clamp(InputManager.get_axis("brake", "reverse"), 0.0, 1.0)
	steering_input = clamp(InputManager.get_axis("steering_left", "steering_right"), -1.0, 1.0)
	
	shift_up_request = InputManager.is_action_pressed("gear_up")
	shift_down_request = InputManager.is_action_pressed("gear_down")
	
	# Clamp steering to maximum angle
	steering_angle = steering_input * MAX_STEERING_ANGLE

# ============================================================================
# PHYSICS UPDATE
# ============================================================================

func _update_physics(delta: float) -> void:
	"""Update vehicle physics based on current state."""
	var torque_output: float = _calculate_torque_output()
	var effective_friction: float = _get_effective_friction()
	var air_drag: float = _calculate_air_resistance()
	
	# Calculate net force
	var net_force: float = torque_output - effective_friction - air_drag
	
	# Apply acceleration
	acceleration = net_force / _settings.default_vehicle_mass
	
	# Update speed with smoothing
	var old_speed: float = current_speed
	current_speed += acceleration * delta
	
	# Handle direction change
	if current_speed > 0 and old_speed < 0:
		current_speed = 0.0
	elif current_speed < 0 and old_speed > 0:
		current_speed = 0.0
	
	# Update distance traveled
	var delta_distance: float = abs(current_speed * delta)
	total_distance_traveled += delta_distance
	
	# Update vehicle state
	_update_vehicle_state()
	
	# Apply movement to physics body
	if _physics_body:
		_apply_movement_to_body()

func _calculate_torque_output() -> float:
	"""Calculate engine torque output based on gear and throttle."""
	var base_torque: float = powertrain.base_torque if powertrain else 500.0
	var gear_ratio: float = GEAR_RATIOS[current_gear] if current_gear >= 0 and current_gear <= 7 else 0.0
	
	# Apply throttle and torque multiplier
	var torque: float = base_torque * gear_ratio * throttle_input * TORQUE_MULTIPLIER
	
	# Apply nitro bonus if available
	if nitro_available and nitro_amount > 0:
		torque *= NITRO_BONUS
	
	return torque

func _get_effective_friction() -> float:
	"""Calculate friction force based on current conditions."""
	var base_friction: float = _settings.default_vehicle_mass * _settings.gravity * FRICTION_COEFFICIENT
	var braking_multiplier: float = 1.0 + (brake_input * 2.0)
	return base_friction * braking_multiplier

func _calculate_air_resistance() -> float:
	"""Calculate air resistance based on speed."""
	return 0.5 * AIR_RESISTANCE * pow(current_speed, 2)

func _apply_movement_to_body() -> void:
	"""Apply calculated movement to the physics body."""
	if not _physics_body:
		return
	
	var velocity_x: float = current_speed * cos(rotation)
	var velocity_y: float = current_speed * sin(rotation)
	
	_physics_body.velocity.x = velocity_x
	_physics_body.velocity.y = velocity_y

# ============================================================================
# VEHICLE STATE MANAGEMENT
# ============================================================================

func _update_vehicle_state() -> void:
	"""Update vehicle state based on current conditions."""
	if abs(rpm - idle_rpm) < 100 and abs(current_speed) < 1.0:
		current_vehicle_state = VehicleState.IDLE
	elif rpm > idle_rpm and current_speed > 0:
		if brake_input > 0.8:
			current_vehicle_state = VehicleState.BRAKING
		else:
			current_vehicle_state = VehicleState.RUNNING
	elif rpm > max_rpm * 0.9:
		current_vehicle_state = VehicleState.REVVING
	elif current_speed == 0.0 and current_gear != 0:
		current_vehicle_state = VehicleState.IDLE

func _check_gear_limits() -> void:
	"""Check and enforce gear limits."""
	var min_gear: int = -1 if current_speed < 0 else 0
	var max_gear: int = 6
	
	if current_gear < min_gear:
		current_gear = min_gear
	elif current_gear > max_gear:
		current_gear = max_gear

func _update_nitro(delta: float) -> void:
	"""Update nitrous system state."""
	if nitro_cooldown > 0:
		nitro_cooldown -= delta
		if nitro_cooldown < 0:
			nitro_cooldown = 0.0
	
	if nitro_amount < 100.0 and nitro_cooldown <= 0:
		nitro_amount += delta * 10.0  # Recharge rate
		if nitro_amount > 100.0:
			nitro_amount = 100.0

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================

func shift_gear(new_gear: int) -> bool:
	"""Attempt to shift to a specific gear."""
	if new_gear < -1 or new_gear > 6:
		return false
	
	if abs(current_speed) < MIN_SPEED_FOR_GEAR_CHANGE and new_gear != 0:
		return false
	
	var old_gear: int = current_gear
	current_gear = new_gear
	
	emit_signal("gear_changed", old_gear, new_gear)
	
	if powertrain:
		powertrain.shift_gear(new_gear)
	
	return true

func shift_up() -> bool:
	"""Request an upshift."""
	if current_gear < 6:
		return shift_gear(current_gear + 1)
	return false

func shift_down() -> bool:
	"""Request a downshift."""
	if current_gear > -1:
		return shift_gear(current_gear - 1)
	return false

func auto_shift() -> void:
	"""Automatically shift gears based on RPM."""
	if rpm > max_rpm * 0.9 and current_gear < 6:
		shift_up()
	elif rpm < idle_rpm and current_gear > 1:
		shift_down()

# ============================================================================
# NITROUS SYSTEM
# ============================================================================

func use_nitro() -> bool:
	"""Activate nitrous system."""
	if not nitro_available or nitro_amount <= 0:
		return false
	
	var actual_amount: float = min(nitro_amount, NITRO_CONSUMPTION_RATE)
	nitro_amount -= actual_amount
	nitro_cooldown = 5.0  # 5 second cooldown
	
	emit_signal("nitro_used", actual_amount)
	return true

func has_nitro_available() -> bool:
	"""Check if nitrous is available."""
	return nitro_available and nitro_amount > 0

# ============================================================================
# SIGNAL HANDLERS
# ============================================================================

func _on_engine_started() -> void:
	"""Handle engine start event."""
	rpm = idle_rpm
	current_vehicle_state = VehicleState.RUNNING

func _on_engine_stopped() -> void:
	"""Handle engine stop event."""
	rpm = 0.0
	current_vehicle_state = VehicleState.IDLE

func _update_rpm_display(new_rpm: float) -> void:
	"""Update RPM display signal."""
	rpm = new_rpm
	emit_signal("rpm_changed", rpm, max_rpm)

func _emit_signals() -> void:
	"""Emit relevant status signals."""
	if current_speed != 0:
		emit_signal("speed_changed", current_speed, max_rpm * GEAR_RATIOS[current_gear])
		
	if current_gear != 0:
		emit_signal("vehicle_moved", total_distance_traveled)

# ============================================================================
# COLLISION HANDLING
# ============================================================================

func handle_collision(collision_direction: Vector2) -> void:
	"""Handle vehicle collision."""
	last_collision_time = Time.get_ticks_msec() / 1000.0
	current_vehicle_state = VehicleState.COLLIDED
	
	# Apply collision damping
	collision_damping = 0.3
	
	# Reduce speed significantly
	current_speed *= 0.5
	
	# Emit collision signal
	emit_signal("collision_detected", collision_direction)
	
	# Cooldown for next collision check
	await get_tree().create_timer(0.5).timeout
	current_vehicle_state = VehicleState.RUNNING

func is_recently_collided() -> bool:
	"""Check if vehicle collided recently."""
	var time_since_collision: float = (Time.get_ticks_msec() / 1000.0) - last_collision_time
	return time_since_collision < 2.0

# ============================================================================
# UTILITY METHODS
# ============================================================================

func get_max_speed_for_gear() -> float:
	"""Get maximum speed possible in current gear."""
	var gear_ratio: float = GEAR_RATIOS[current_gear] if current_gear >= 0 and current_gear <= 7 else 0.0
	var max_engine_speed: float = max_rpm
	var base_max_speed: float = 200.0  # Base top speed in neutral
	
	return base_max_speed * gear_ratio * (max_engine_speed / 6000.0)

func get_current_power() -> float:
	"""Get current power output in kW."""
	var gear_ratio: float = GEAR_RATIOS[current_gear] if current_gear >= 0 and current_gear <= 7 else 0.0
	var engine_power: float = powertrain.max_power if powertrain else 150.0
	return engine_power * gear_ratio * (rpm / max_rpm)

func reset() -> void:
	"""Reset vehicle to initial state."""
	_reset_vehicle()
	if _physics_body:
		_physics_body.velocity = Vector2.ZERO
		_physics_body.linear_interpolate(Vector2.ZERO, 0.5)

func save_state() -> Dictionary:
	"""Save current vehicle state."""
	return {
		"current_speed": current_speed,
		"current_gear": current_gear,
		"rpm": rpm,
		"total_distance_traveled": total_distance_traveled,
		"nitro_amount": nitro_amount,
		"current_vehicle_state": current_vehicle_state
	}

func load_state(state_data: Dictionary) -> void:
	"""Load vehicle state from saved data."""
	current_speed = state_data.get("current_speed", 0.0)
	current_gear = state_data.get("current_gear", 0)
	rpm = state_data.get("rpm", idle_rpm)
	total_distance_traveled = state_data.get("total_distance_traveled", 0.0)
	nitro_amount = state_data.get("nitro_amount", 100.0)
	current_vehicle_state = VehicleState(state_data.get("current_vehicle_state", 0))

</file>