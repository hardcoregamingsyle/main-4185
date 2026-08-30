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

# Constants for physics calculations
const MIN_SPEED_FOR_GEAR_SHIFT: float = 50.0
const MAX_STEERING_ANGLE: float = PI / 4  # 45 degrees
const GEAR_RATIOS: Array[float] = [0.0, 3.8, 2.5, 1.7, 1.3, 1.0, 0.8]
const REVERSE_GEAR_RATIO: float = -4.0
const WHEELBASE: float = 2.5
const TRACK_WIDTH: float = 1.6
const DRAG_COEFFICIENT: float = 0.30
const AIR_DENSITY: float = 1.225
const FRONT_WEIGHT_DISTRIBUTION: float = 0.45
const REAR_WEIGHT_DISTRIBUTION: float = 0.55
const FRICTION_COEFFICIENT: float = 1.05
const MAX_ACCELERATION: float = 12.0
const MAX_DECELERATION: float = 15.0
const STEERING_SENSITIVITY: float = 0.8
const GEAR_SHIFT_TIME: float = 0.15
const RPM_REDLINE: float = 7500.0
const RPM_IDLE: float = 800.0
const RPM_MAX_TORQUE: float = 4500.0
const TORQUE_CURVE_PEAK: float = 350.0

# Movement tracking
var _last_position: Vector2 = Vector2.ZERO
var _movement_vector: Vector2 = Vector2.ZERO
var _speed_history: Array[float] = []
const SPEED_HISTORY_SIZE: int = 60

func _ready() -> void:
	_init_physics_reference()
	_connect_signals_to_powertrain()
	_reset_vehicle_state()
	_load_settings()

func _init_physics_reference() -> void:
	if Engine.has_singleton("PhysicsSettings"):
		_settings = Engine.get_singleton("PhysicsSettings")
	elif ResourceLoader.exists("res://scripts/core/PhysicsSettings.gd"):
		var settings_resource = load("res://scripts/core/PhysicsSettings.gd")
		if settings_resource:
			_settings = settings_resource.new()

func _connect_signals_to_powertrain() -> void:
	if powertrain:
		powertrain.signal_torque_applied.connect(_on_torque_applied)
		powertrain.signal_engine_warning.connect(_on_engine_warning)

func _reset_vehicle_state() -> void:
	current_speed = 0.0
	target_speed = 0.0
	acceleration = 0.0
	braking_force = 0.0
	steering_angle = 0.0
	current_gear = 0
	rpm = idle_rpm
	nitro_available = true
	nitro_amount = 100.0
	total_distance_traveled = 0.0
	_last_position = position
	_movement_vector = Vector2.ZERO
	_speed_history.clear()
	_fill_speed_history()

func _fill_speed_history() -> void:
	for i in range(SPEED_HISTORY_SIZE):
		_speed_history.append(0.0)

func _load_settings() -> void:
	if _settings:
		max_rpm = _settings.default_max_rpm if _settings.has_member("default_max_rpm") else 8000.0
		idle_rpm = _settings.default_idle_rpm if _settings.has_member("default_idle_rpm") else 800.0
	else:
		max_rpm = 8000.0
		idle_rpm = 800.0

func _physics_process(delta: float) -> void:
	_handle_inputs(delta)
	_update_physics(delta)
	_update_gearing(delta)
	_handle_nitrous(delta)
	_track_movement(delta)
	_check_vehicle_state()
	_emit_signals(delta)

func _handle_inputs(delta: float) -> void:
	throttle_input = InputManager.get_axis("throttle", "gas")
	brake_input = InputManager.get_axis("brake", "brake")
	steering_input = InputManager.get_axis("steer_left", "steer_right")
	
	shift_up_request = Input.is_action_just_pressed("gear_up")
	shift_down_request = Input.is_action_just_pressed("gear_down")
	
	# Clamp inputs to valid ranges
	throttle_input = clamp(throttle_input, -1.0, 1.0)
	brake_input = clamp(brake_input, -1.0, 1.0)
	steering_input = clamp(steering_input, -1.0, 1.0)
	
	# Apply steering sensitivity
	steering_input *= STEERING_SENSITIVITY

func _update_physics(delta: float) -> void:
	_calculate_acceleration(delta)
	_apply_drag_and_friction(delta)
	_update_velocity(delta)
	_update_steering(delta)
	_limit_by_redline(delta)

func _calculate_acceleration(delta: float) -> void:
	if not powertrain:
		return
	
	var torque_mult = _get_torque_multiplier()
	var effective_torque = torque_mult * TORQUE_CURVE_PEAK
	
	# Calculate force from torque
	var force = (effective_torque * GEAR_RATIOS[current_gear]) / WHEELBASE
	
	# Apply throttle
	if throttle_input > 0.0:
		force *= throttle_input
		acceleration = force / _settings.default_vehicle_mass if _settings else force / 1500.0
	else:
		acceleration = 0.0
	
	# Apply braking
	if brake_input > 0.0:
		braking_force = MAX_DECELERATION * brake_input
	else:
		braking_force = 0.0
	
	# Combine acceleration and braking
	acceleration -= braking_force

func _apply_drag_and_friction(delta: float) -> void:
	if current_speed <= 0.0:
		return
	
	var drag_force = 0.5 * AIR_DENSITY * DRAG_COEFFICIENT * pow(current_speed, 2)
	var friction_force = current_speed * FRICTION_COEFFICIENT * 0.1
	
	# Apply resistance based on gear
	var gear_resistance = 0.0
	if current_gear > 0:
		gear_resistance = friction_force * (current_gear * 0.1)
	
	acceleration -= (drag_force + friction_force + gear_resistance) / (_settings.default_vehicle_mass if _settings else 1500.0)

func _update_velocity(delta: float) -> void:
	var prev_speed = current_speed
	
	# Update speed
	current_speed += acceleration * delta
	
	# Clamp speed to reasonable limits
	var max_speed = _get_max_speed_for_gear()
	current_speed = min(max(abs(current_speed), max_speed), max_speed)
	
	# Preserve direction
	if prev_speed < 0 and current_speed > 0:
		current_speed = -abs(current_speed)
	elif prev_speed > 0 and current_speed < 0:
		current_speed = abs(current_speed)
	
	target_speed = current_speed
	
	# Store in history
	_speed_history.push_back(current_speed)
	if _speed_history.size() > SPEED_HISTORY_SIZE:
		_speed_history.pop_front()

func _update_steering(delta: float) -> void:
	if current_speed < MIN_SPEED_FOR_GEAR_SHIFT:
		steering_angle = 0.0
		return
	
	steering_angle = steering_input * MAX_STEERING_ANGLE
	
	# Smooth steering transition
	if chassis:
		chassis.steer(steering_angle)

func _limit_by_redline(delta: float) -> void:
	if rpm >= RPM_REDLINE and throttle_input > 0.0:
		acceleration *= 0.5
		current_vehicle_state = VehicleState.REVVING

func _update_gearing(delta: float) -> void:
	_auto_shift_gears(delta)
	_manual_shift_gears(delta)
	_update_rpm()

func _auto_shift_gears(delta: float) -> void:
	if throttle_input <= 0.0 or current_gear == 0:
		return
	
	var should_upshift = false
	var should_downshift = false
	
	# Upshift condition
	if rpm > RPM_REDLINE - 500 and current_gear < 6:
		should_upshift = true
	
	# Downshift condition
	if rpm < idle_rpm + 200 and current_gear > 1:
		should_downshift = true
	elif current_speed < MIN_SPEED_FOR_GEAR_SHIFT * current_gear:
		should_downshift = true
	
	if should_upshift:
		shift_gear(1)
	elif should_downshift:
		shift_gear(-1)

func _manual_shift_gears(delta: float) -> void:
	if shift_up_request and current_gear < 6:
		shift_gear(1)
		shift_up_request = false
	elif shift_down_request and current_gear > 0:
		shift_gear(-1)
		shift_down_request = false

func _update_rpm() -> void:
	if current_gear == 0:
		rpm = idle_rpm
		return
	
	var base_rpm = abs(current_speed) * GEAR_RATIOS[current_gear] * 10.0
	
	# Add throttle influence on RPM
	if throttle_input > 0.0:
		rpm = lerp(rpm, base_rpm + (throttle_input * 2000.0), 0.1)
	else:
		rpm = lerp(rpm, base_rpm, 0.05)
	
	# Ensure RPM stays in valid range
	rpm = clamp(rpm, idle_rpm, max_rpm)

func _handle_nitrous(delta: float) -> void:
	if nitro_cooldown > 0.0:
		nitro_cooldown -= delta
		return
	
	if Input.is_action_just_pressed("nitrous") and nitro_available and nitro_amount > 0.0:
		_use_nitrous()

func _use_nitrous() -> void:
	nitro_available = false
	nitro_cooldown = 10.0
	nitro_amount -= NITRO_CONSUMPTION_RATE
	
	# Apply nitrous bonus
	acceleration *= NITRO_BONUS
	
	nitro_used.emit(nitro_amount)
	
	if nitro_amount <= 0.0:
		nitro_available = false

func _track_movement(delta: float) -> void:
	_movement_vector = position - _last_position
	var distance_moved = _movement_vector.length()
	
	total_distance_traveled += distance_moved
	
	if distance_moved > 0.01:
		vehicle_moved.emit(distance_moved)
	
	_last_position = position

func _check_vehicle_state() -> void:
	if current_speed > 0.0 and throttle_input > 0.0:
		current_vehicle_state = VehicleState.RUNNING
	elif current_speed > 0.0 and throttle_input <= 0.0:
		current_vehicle_state = VehicleState.DRIFTING
	elif current_speed < 0.0:
		current_vehicle_state = VehicleState.BRAKING
	else:
		current_vehicle_state = VehicleState.IDLE
	
	# Check for collision state
	if Time.get_ticks_msec() - last_collision_time < 500:
		current_vehicle_state = VehicleState.COLLIDED

func _emit_signals(delta: float) -> void:
	speed_changed.emit(current_speed, _get_max_speed_for_gear())
	rpm_changed.emit(rpm, max_rpm)

func _get_max_speed_for_gear() -> float:
	var gear_ratio = GEAR_RATIOS[current_gear] if current_gear > 0 else 0.0
	var final_drive = 3.5
	var tire_radius = 0.3
	
	return (max_rpm / 60.0) * gear_ratio * final_drive * 2.0 * PI * tire_radius * 3.6

func _get_torque_multiplier() -> float:
	if rpm <= RPM_IDLE:
		return 0.3
	elif rpm <= RPM_MAX_TORQUE:
		return 1.0
	else:
		return max(0.5, 1.0 - (rpm - RPM_MAX_TORQUE) / (max_rpm - RPM_MAX_TORQUE))

func _on_torque_applied(torque: float) -> void:
	acceleration = torque / (_settings.default_vehicle_mass if _settings else 1500.0)

func _on_engine_warning(warning_type: String) -> void:
	match warning_type:
		"overheating":
			print("[Engine Warning] Temperature critical!")
			acceleration *= 0.7
		"low_oil_pressure":
			print("[Engine Warning] Oil pressure low!")
			rpm *= 0.8
		"redline":
			print("[Engine Warning] Redline exceeded!")
			_current_gear = max(0, current_gear - 1)

func shift_gear(direction: int) -> void:
	var old_gear = current_gear
	var new_gear = current_gear + direction
	
	# Validate gear shift
	if new_gear < 0 or new_gear > 6:
		return
	
	if direction > 0 and new_gear > 6:
		return
	if direction < 0 and new_gear < 0:
		return
	
	# Apply gear shift delay
	await get_tree().create_timer(GEAR_SHIFT_TIME).timeout
	
	current_gear = new_gear
	gear_changed.emit(old_gear, new_gear)
	
	# Adjust RPM after shift
	var rpm_drop = 1000.0 if direction > 0 else -500.0
	rpm = clamp(rpm + rpm_drop, idle_rpm, max_rpm)

func reset_vehicle() -> void:
	_reset_vehicle_state()
	current_vehicle_state = VehicleState.IDLE
	engine_stopped.emit()

func start_engine() -> void:
	rpm = idle_rpm
	current_vehicle_state = VehicleState.IDLE
	engine_started.emit()

func stop_engine() -> void:
	rpm = 0.0
	current_vehicle_state = VehicleState.IDLE
	engine_stopped.emit()

func apply_collision_impact(direction: Vector2, force: float) -> void:
	last_collision_time = Time.get_ticks_msec()
	current_vehicle_state = VehicleState.COLLIDED
	
	# Dampen speed
	current_speed *= collision_damping
	acceleration *= collision_damping
	
	collision_detected.emit(direction)

func get_vehicle_status() -> Dictionary:
	return {
		"speed": current_speed,
		"max_speed": _get_max_speed_for_gear(),
		"gear": current_gear,
		"rpm": rpm,
		"throttle": throttle_input,
		"brake": brake_input,
		"steering": steering_input,
		"nitro_amount": nitro_amount,
		"distance": total_distance_traveled,
		"state": current_vehicle_state
	}

func set_custom_gear_ratios(ratios: Array[float]) -> void:
	if ratios.size() >= 7:
		GEAR_RATIOS.splice(0, GEAR_RATIOS.size())
		GEAR_RATIOS.append_array(ratios)

func calculate_aerodynamic_downforce(speed: float) -> float:
	return 0.5 * AIR_DENSITY * DRAG_COEFFICIENT * pow(speed, 2) * WHEELBASE * TRACK_WIDTH

func simulate_suspension(compression: float) -> float:
	var spring_constant = 200000.0
	var damping_constant = 15000.0
	return -spring_constant * compression - damping_constant * compression

func get_weight_distribution() -> Dictionary:
	return {
		"front": FRONT_WEIGHT_DISTRIBUTION,
		"rear": REAR_WEIGHT_DISTRIBUTION,
		"total": 1.0
	}

func calculate_lateral_g(force_vector: Vector2) -> float:
	var lateral_component = force_vector.x
	var normal_force = _settings.default_vehicle_mass * 9.81 if _settings else 14715.0
	return lateral_component / normal_force

func reset_nitrous() -> void:
	nitro_amount = 100.0
	nitro_available = true
	nitro_cooldown = 0.0

func enable_debug_mode(enabled: bool) -> void:
	if GameManager.debug_mode != enabled:
		GameManager.debug_mode = enabled
		print("[VehicleController] Debug mode:", "enabled" if enabled else "disabled")

func save_vehicle_state() -> Dictionary:
	return {
		"position": position,
		"rotation": rotation,
		"speed": current_speed,
		"gear": current_gear,
		"rpm": rpm,
		"nitro_amount": nitro_amount,
		"distance": total_distance_traveled
	}

func restore_vehicle_state(state_data: Dictionary) -> void:
	position = state_data.get("position", position)
	rotation = state_data.get("rotation", rotation)
	current_speed = state_data.get("speed", 0.0)
	current_gear = state_data.get("gear", 0)
	rpm = state_data.get("rpm", idle_rpm)
	nitro_amount = state_data.get("nitro_amount", 100.0)
	total_distance_traveled = state_data.get("distance", 0.0)