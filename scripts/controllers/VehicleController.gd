extends CharacterBody2D
class_name VehicleController

## VehicleController - Core vehicle physics controller using PhysicsSettings constants
## Handles throttle, brake, steering inputs, wheel forces, gear shifting logic
## Copyright 2026 Thalamus Racing Simulator Project

# ============================================================================
# SIGNALS
# ============================================================================
signal engine_started
signal engine_stopped
signal gear_changed(old_gear: int, new_gear: int)
signal nitro_used(amount: float)
signal collision_detected(direction: Vector2)
signal speed_changed(current_speed: float, max_speed: float)
signal traction_loss(detected: bool)
signal rpm_changed(rpm: float)
signal vehicle_state_changed(state: VehicleState)

# ============================================================================
# ENUMERATIONS
# ============================================================================
enum VehicleState {
	IDLE,
	RUNNING,
	REVVING,
	BRAKING,
	COLLIDED,
	DRIFTING,
	SLIDING,
	JUMPING
}

enum GearType {
	NEUTRAL,
	FIRST,
	SECOND,
	THIRD,
	FOURTH,
	FIFTH,
	SIXTH,
	REVERSE
}

# ============================================================================
# REFERENCES
# ============================================================================
@onready var chassis: Node2D = $Chassis if $Chassis else null
@onready var powertrain: Powertrain = $Powertrain if $Powertrain else null
@onready var front_left_wheel: WheelCollider = $FrontLeftWheel if $FrontLeftWheel else null
@onready var front_right_wheel: WheelCollider = $FrontRightWheel if $FrontRightWheel else null
@onready var rear_left_wheel: WheelCollider = $RearLeftWheel if $RearLeftWheel else null
@onready var rear_right_wheel: WheelCollider = $RearRightWheel if $RearRightWheel else null

var _physics_body: CharacterBody2D = self

# ============================================================================
# STATE VARIABLES
# ============================================================================
var current_vehicle_state: VehicleState = VehicleState.IDLE
var last_collision_time: float = 0.0
var collision_damping: float = 0.3

# Input state (normalized -1.0 to 1.0)
var _throttle_input: float = 0.0
var _brake_input: float = 0.0
var _steering_input: float = 0.0
var _nitro_active: bool = false
var _reverse_active: bool = false

# Vehicle properties
var current_gear: int = GearType.NEUTRAL
var target_gear: int = GearType.NEUTRAL
var engine_rpm: float = 800.0
var max_rpm: float = 8000.0
var idle_rpm: float = 800.0
var clutch_engaged: bool = true
var transmission_type: String = "manual"

# Speed tracking
var current_speed: float = 0.0
var max_speed: float = 0.0
var acceleration: float = 0.0
var deceleration: float = 0.0
var forward_velocity: float = 0.0

# Physics references
var wheel_forces: Array[Vector2] = []
var steering_angle: float = 0.0
var wheel_rotation_angles: Dictionary = {}

# Drift mechanics
var drift_factor: float = 0.0
var drift_threshold: float = 0.7
var grip_level: float = 1.0
var lateral_slip: float = 0.0

# Nitro system
var nitro_amount: float = 100.0
var nitro_cooldown: float = 0.0
var nitro_multiplier: float = 1.5
var nitro_active_timer: float = 0.0

# Configuration from PhysicsSettings
var _settings: PhysicsSettings = null
var _time_scale: float = 1.0

# ============================================================================
# LATERAL PHYSICS CONSTANTS
# ============================================================================
var friction_coefficient: float = 0.95
var air_resistance: float = 0.001
var rolling_resistance: float = 0.002
var cornering_force: float = 1.5
var drift_recovery_rate: float = 0.02

# ============================================================================
# GEAR RATIOS AND MAX SPEEDS
# ============================================================================
var _gear_ratios: Dictionary = {
	GearType.FIRST: 3.5,
	GearType.SECOND: 2.2,
	GearType.THIRD: 1.6,
	GearType.FOURTH: 1.2,
	GearType.FIFTH: 0.95,
	GearType.SIXTH: 0.75,
	GearType.REVERSE: -3.0,
	GearType.NEUTRAL: 0.0
}

var _gear_max_speeds: Dictionary = {
	GearType.FIRST: 40.0,
	GearType.SECOND: 70.0,
	GearType.THIRD: 100.0,
	GearType.FOURTH: 130.0,
	GearType.FIFTH: 160.0,
	GearType.SIXTH: 200.0,
	GearType.REVERSE: 30.0,
	GearType.NEUTRAL: 0.0
}

# ============================================================================
# WHEEL DATA STRUCTURES
# ============================================================================
struct WheelData {
	var position: Vector2
	var force_applied: Vector2 = Vector2.ZERO
	var rotation_angle: float = 0.0
	var slip_ratio: float = 0.0
	var slip_angle: float = 0.0
	var normal_force: float = 0.0
	var is_braking: bool = false
	var is_driving: bool = false
}

var _wheel_data: Dictionary = {
	"front_left": WheelData(),
	"front_right": WheelData(),
	"rear_left": WheelData(),
	"rear_right": WheelData()
}

# ============================================================================
# INITIALIZATION
# ============================================================================
func _ready() -> void:
	_init_settings()
	_init_wheels()
	_connect_signals()
	_reset_vehicle_state()
	print("VehicleController initialized successfully")

func _init_settings() -> void:
	_settings = GameManager.get_node_or_null("/root/PhysicsSettings")
	if _settings == null:
		_settings = preload("res://scripts/core/PhysicsSettings.gd").new()
	
	max_rpm = _settings.max_engine_rpm if hasattr(_settings, "max_engine_rpm") else 8000.0
	idle_rpm = _settings.idle_engine_rpm if hasattr(_settings, "idle_engine_rpm") else 800.0
	max_speed = _settings.default_vehicle_top_speed if hasattr(_settings, "default_vehicle_top_speed") else 200.0

func _init_wheels() -> void:
	for wheel_key in _wheel_data.keys():
		_wheel_data[wheel_key].position = Vector2.ZERO
		_wheel_data[wheel_key].force_applied = Vector2.ZERO

func _connect_signals() -> void:
	if powertrain:
		powertrain.engine_started.connect(_on_powertrain_engine_started)
		powertrain.engine_stopped.connect(_on_powertrain_engine_stopped)
		powertrain.rpm_changed.connect(_on_powertrain_rpm_changed)
		powertrain.gear_changed.connect(_on_powertrain_gear_changed)

# ============================================================================
# INPUT HANDLING
# ============================================================================
func _input_event(event: InputEvent) -> void:
	if event.is_action_pressed("ui_accept"):
		_toggle_clutch()

func process_input(delta: float) -> void:
	_throttle_input = Input.get_axis("brake", "accelerate") * -1.0
	_brake_input = Input.get_axis("brake_up", "brake_down")
	_steering_input = Input.get_axis("turn_left", "turn_right")
	_reverse_active = Input.is_action_pressed("reverse")
	_nitro_active = Input.is_action_pressed("nitro")

func get_input_vector() -> Vector2:
	return Vector2(-_steering_input, _throttle_input)

# ============================================================================
# MAIN UPDATE LOOP
# ============================================================================
func _physics_process(delta: float) -> void:
	_time_scale = _settings.time_scale if _settings else 1.0
	
	process_input(delta)
	update_engine_state(delta)
	update_vehicle_state(delta)
	calculate_wheel_forces(delta)
	apply_physics(delta)
	handle_collisions(delta)
	update_drift_mechanics(delta)
	update_nitro_system(delta)
	update_visual_indicators()

func update_engine_state(delta: float) -> void:
	var throttle_factor = abs(_throttle_input)
	var brake_factor = _brake_input
	
	if clutch_engaged:
		if current_gear == GearType.NEUTRAL:
			engine_rpm = lerp(engine_rpm, idle_rpm, delta * 10.0)
		else:
			var gear_ratio = _gear_ratios[current_gear]
			var target_rpm = calculate_target_rpm(gear_ratio, throttle_factor)
			engine_rpm = lerp(engine_rpm, target_rpm, delta * 5.0)
			
			if engine_rpm >= max_rpm:
				engine_rpm = max_rpm
				current_vehicle_state = VehicleState.REVVING
			
			if engine_rpm < idle_rpm:
				engine_rpm = idle_rpm
	
	if brake_factor > 0.3 and not _reverse_active:
		engine_rpm = lerp(engine_rpm, idle_rpm, delta * 15.0)
	
	rpm_changed.emit(engine_rpm)

func calculate_target_rpm(gear_ratio: float, throttle_factor: float) -> float:
	var base_rpm = idle_rpm + (max_rpm - idle_rpm) * throttle_factor
	var speed_factor = current_speed / max_speed
	return base_rpm * (1.0 - speed_factor * 0.3)

func update_vehicle_state(delta: float) -> void:
	var is_moving = abs(forward_velocity) > 0.5
	var is_accelerating = _throttle_input > 0.3
	var is_braking = _brake_input > 0.3
	var is_revving = engine_rpm >= max_rpm * 0.95
	var is_drifting = drift_factor > drift_threshold
	
	if is_revving:
		current_vehicle_state = VehicleState.REVVING
	elif is_drifting:
		current_vehicle_state = VehicleState.DRIFTING
	elif is_braking:
		current_vehicle_state = VehicleState.BRAKING
	elif is_moving and is_accelerating:
		current_vehicle_state = VehicleState.RUNNING
	elif is_moving:
		current_vehicle_state = VehicleState.RUNNING
	else:
		current_vehicle_state = VehicleState.IDLE
	
	if current_vehicle_state != vehicle_state_changed:
		vehicle_state_changed.emit(current_vehicle_state)

func calculate_wheel_forces(delta: float) -> void:
	var total_force = Vector2.ZERO
	var driving_force = calculate_driving_force()
	var braking_force = calculate_braking_force()
	var steering_force = calculate_steering_force()
	
	_update_wheel_data(driving_force, braking_force, steering_force, delta)
	
	wheel_forces.clear()
	for wheel_key in _wheel_data.keys():
		total_force += _wheel_data[wheel_key].force_applied
		wheel_forces.append(_wheel_data[wheel_key].force_applied)

func calculate_driving_force() -> float:
	var gear_ratio = _gear_ratios[current_gear]
	var power_factor = (engine_rpm - idle_rpm) / (max_rpm - idle_rpm)
	
	if power_factor < 0:
		power_factor = 0.0
	
	var base_force = power_factor * _settings.default_vehicle_mass * 0.8
	var torque_multiplier = 1.0 + (nitro_amount / 100.0) * (nitro_multiplier - 1.0)
	
	if current_gear == GearType.REVERSE and _reverse_active:
		torque_multiplier *= -1.0
	
	return base_force * gear_ratio * torque_multiplier

func calculate_braking_force() -> float:
	var brake_strength = _brake_input * _settings.brake_force_multiplier
	return brake_strength * _settings.default_vehicle_mass * 0.5

func calculate_steering_force() -> float:
	var steering_effectiveness = grip_level * _settings.steering_sensitivity
	return _steering_input * steering_effectiveness

func _update_wheel_data(driving_force: float, braking_force: float, steering_force: float, delta: float) -> void:
	var speed_direction = sign(forward_velocity) if abs(forward_velocity) > 0.1 else 1
	var drive_wheels = ["rear_left", "rear_right"]
	var steer_wheels = ["front_left", "front_right"]
	
	for wheel_key in _wheel_data.keys():
		var wheel = _wheel_data[wheel_key]
		
		# Apply driving force to drive wheels
		if drive_wheels.has(wheel_key) and current_gear != GearType.NEUTRAL:
			wheel.force_applied.x = driving_force * 0.25
			wheel.is_driving = true
		else:
			wheel.force_applied.x = 0.0
			wheel.is_driving = false
		
		# Apply braking force to all wheels
		if braking_force > 0:
			wheel.force_applied.x -= braking_force * 0.25
			wheel.is_braking = true
		else:
			wheel.is_braking = false
		
		# Apply steering force to front wheels
		if steer_wheels.has(wheel_key):
			wheel.force_applied.y = steering_force * 0.5
			steering_angle = _steering_input * _settings.max_steering_angle
		else:
			wheel.force_applied.y = 0.0
			steering_angle = 0.0
		
		# Update wheel rotation
		wheel.rotation_angle += forward_velocity * delta * 0.5

func apply_physics(delta: float) -> void:
	var total_force = Vector2.ZERO
	
	for wheel_key in _wheel_data.keys():
		total_force += _wheel_data[wheel_key].force_applied
	
	# Apply forces to velocity
	var applied_acceleration = total_force.x / _settings.default_vehicle_mass
	forward_velocity += applied_acceleration * delta * 10.0
	
	# Apply resistance forces
	forward_velocity *= (1.0 - air_resistance - rolling_resistance)
	
	# Clamp speed
	var speed_limit = max_speed * (1.0 if forward_velocity >= 0 else -1.0)
	if abs(forward_velocity) > speed_limit:
		forward_velocity = lerp(forward_velocity, speed_limit, delta * 2.0)
	
	# Update velocity vector
	velocity = Vector2(forward_velocity, 0.0).rotated(rotation)
	
	# Move body
	move_and_slide()
	
	# Update speed tracking
	current_speed = abs(forward_velocity)
	speed_changed.emit(current_speed, max_speed)

func handle_collisions(delta: float) -> void:
	for col_idx in range(get_slide_collision_count()):
		var collision = get_slide_collision(col_idx)
		var collider = collision.get_collider()
		
		if collider.has_method("_on_vehicle_collision"):
			collider._on_vehicle_collision(self)
		
		var collision_normal = collision.get_normal()
		
		if engine_rpm < idle_rpm * 1.5 and abs(forward_velocity) > 5.0:
			last_collision_time = Time.get_unix_time_from_system()
			collision_detected.emit(collision_normal)
			current_vehicle_state = VehicleState.COLLIDED
			
			# Apply collision damping
			forward_velocity *= collision_damping
			
			# Screen shake effect (if camera exists)
			if get_parent() and get_parent().has_method("trigger_shake"):
				get_parent().trigger_shake(0.5)

func update_drift_mechanics(delta: float) -> void:
	var lateral_velocity = velocity.y
	
	if abs(forward_velocity) > 10.0:
		lateral_slip = lerp(lateral_slip, lateral_velocity * 0.1, delta * 5.0)
		
		if abs(lateral_slip) > drift_threshold:
			drift_factor = min(abs(lateral_slip) / drift_threshold, 1.0)
			grip_level = lerp(grip_level, 0.5, delta * grip_recovery_rate())
		else:
			drift_factor = 0.0
			grip_level = lerp(grip_level, 1.0, delta * grip_recovery_rate())
	else:
		drift_factor = 0.0
		lateral_slip = 0.0
	
	traction_loss.emit(abs(lateral_slip) > 0.5)

func grip_recovery_rate() -> float:
	return drift_recovery_rate * (1.0 - drift_factor * 0.5)

func update_nitro_system(delta: float) -> void:
	if nitro_amount <= 0:
		_nitro_active = false
	
	if _nitro_active and nitro_amount > 0 and nitro_cooldown <= 0:
		nitro_active_timer += delta
		nitro_amount -= delta * 10.0
		
		if nitro_active_timer >= 2.0:
			nitro_amount = max(nitro_amount, 0.0)
			_nitro_active = false
		
		nitro_used.emit(nitro_multiplier)
	else:
		nitro_cooldown = max(nitro_cooldown - delta, 0.0)
		nitro_active_timer = 0.0

func update_visual_indicators() -> void:
	if chassis:
		chassis.set_rotational_velocity(forward_velocity)
		chassis.set_engine_rpm(engine_rpm)

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================
func shift_gear(new_gear: int) -> bool:
	if new_gear == current_gear:
		return false
	
	var old_gear = current_gear
	
	# Prevent impossible shifts
	if not _is_valid_shift(old_gear, new_gear):
		return false
	
	# Engage clutch during shift
	clutch_engaged = false
	await wait_for_shift_completion()
	
	current_gear = new_gear
	target_gear = new_gear
	clutch_engaged = true
	
	gear_changed.emit(old_gear, new_gear)
	
	return true

func _is_valid_shift(from_gear: int, to_gear: int) -> bool:
	# Allow neutral to any gear
	if from_gear == GearType.NEUTRAL:
		return true
	
	# Allow reverse to neutral and vice versa
	if from_gear == GearType.REVERSE or to_gear == GearType.REVERSE:
		return to_gear == GearType.NEUTRAL or from_gear == GearType.NEUTRAL
	
	# Allow sequential shifts (+/- 1)
	var gear_diff = to_gear - from_gear
	return abs(gear_diff) <= 1

func auto_shift_gears() -> void:
	var speed_percent = current_speed / max_speed
	var rpm_percent = (engine_rpm - idle_rpm) / (max_rpm - idle_rpm)
	
	if rpm_percent > 0.9 and current_gear < GearType.SIXTH:
		shift_gear(current_gear + 1)
	elif rpm_percent < 0.2 and current_gear > GearType.FIRST:
		shift_gear(current_gear - 1)
	elif current_speed < 5.0 and current_gear != GearType.FIRST:
		shift_gear(GearType.FIRST)

func manual_shift_up() -> void:
	if current_gear < GearType.SIXTH:
		shift_gear(current_gear + 1)
	elif current_gear == GearType.SIXTH and engine_rpm > max_rpm * 0.8:
		shift_gear(GearType.NEUTRAL)

func manual_shift_down() -> void:
	if current_gear > GearType.FIRST:
		shift_gear(current_gear - 1)
	elif current_gear == GearType.FIRST and current_speed < 5.0:
		shift_gear(GearType.NEUTRAL)

func shift_to_neutral() -> void:
	shift_gear(GearType.NEUTRAL)

func shift_to_reverse() -> void:
	if current_gear != GearType.NEUTRAL:
		shift_gear(GearType.NEUTRAL)
		await wait_for_shift_completion()
		shift_gear(GearType.REVERSE)
	else:
		shift_gear(GearType.REVERSE)

func wait_for_shift_completion() -> void:
	await get_tree().create_timer(0.2).timeout

# ============================================================================
# CLUTCH AND ENGINE CONTROL
# ============================================================================
func toggle_clutch() -> void:
	clutch_engaged = not clutch_engaged
	if not clutch_engined:
		_on_clutch_disengaged()

func _toggle_clutch() -> void:
	clutch_engaged = not clutch_engaged
	if not clutch_engaged:
		_on_clutch_disengaged()

func _on_clutch_disengaged() -> void:
	if engine_rpm > idle_rpm * 1.5:
		engine_rpm = lerp(engine_rpm, idle_rpm, 0.1)

func engage_clutch() -> void:
	clutch_engaged = true

func disengage_clutch() -> void:
	clutch_engaged = false

func start_engine() -> void:
	if engine_rpm < idle_rpm:
		engine_rpm = lerp(engine_rpm, idle_rpm, 0.1)
		engine_started.emit()

func stop_engine() -> void:
	if engine_rpm > idle_rpm * 0.5:
		engine_rpm = lerp(engine_rpm, idle_rpm, 0.05)
		engine_stopped.emit()

func reset_engine_rpm() -> void:
	engine_rpm = idle_rpm

# ============================================================================
# VEHICLE RESET AND UTILITIES
# ============================================================================
func _reset_vehicle_state() -> void:
	current_vehicle_state = VehicleState.IDLE
	current_gear = GearType.NEUTRAL
	target_gear = GearType.NEUTRAL
	engine_rpm = idle_rpm
	current_speed = 0.0
	forward_velocity = 0.0
	velocity = Vector2.ZERO
	steering_angle = 0.0
	drift_factor = 0.0
	grip_level = 1.0
	nitro_amount = 100.0
	nitro_cooldown = 0.0
	nitro_active_timer = 0.0
	clutch_engaged = true
	_throttle_input = 0.0
	_brake_input = 0.0
	_steering_input = 0.0
	_reverse_active = false
	_nitro_active = false

func reset_position(new_position: Vector2) -> void:
	position = new_position
	_reset_vehicle_state()

func set_max_speed(speed: float) -> void:
	max_speed = speed

func set_grip_level(level: float) -> void:
	grip_level = clamp(level, 0.1, 1.0)

func get_gear_info() -> Dictionary:
	return {
		"current_gear": current_gear,
		"target_gear": target_gear,
		"rpm": engine_rpm,
		"speed": current_speed,
		"max_speed": max_speed,
		"clutch_engaged": clutch_engaged
	}

func get_wheel_info() -> Dictionary:
	var info: Dictionary = {}
	for key in _wheel_data.keys():
		info[key] = {
			"position": _wheel_data[key].position,
			"force": _wheel_data[key].force_applied,
			"rotation": _wheel_data[key].rotation_angle,
			"is_driving": _wheel_data[key].is_driving,
			"is_braking": _wheel_data[key].is_braking
		}
	return info

func get_debug_info() -> String:
	return "Speed: %.1f | RPM: %.0f | Gear: %d | State: %s | Drift: %.2f" % [
		current_speed,
		engine_rpm,
		current_gear,
		VehicleState.keys()[current_vehicle_state],
		drift_factor
	]

# ============================================================================
# POWERTRAIN SIGNAL HANDLERS
# ============================================================================
func _on_powertrain_engine_started() -> void:
	start_engine()

func _on_powertrain_engine_stopped() -> void:
	stop_engine()

func _on_powertrain_rpm_changed(rpm: float) -> void:
	engine_rpm = rpm

func _on_powertrain_gear_changed(gear: int) -> void:
	current_gear = gear

# ============================================================================
# DEBUG FUNCTIONS
# ============================================================================
func debug_print_vehicle_state() -> void:
	print(get_debug_info())

func debug_set_rpm(rpm: float) -> void:
	engine_rpm = rpm

func debug_set_speed(speed: float) -> void:
	forward_velocity = speed

func debug_set_gear(gear: int) -> void:
	current_gear = gear

func debug_trigger_collision() -> void:
	collision_detected.emit(Vector2.RIGHT)

func debug_inject_nitro(amount: float) -> void:
	nitro_amount = min(nitro_amount + amount, 100.0)

func debug_reset_all() -> void:
	_reset_vehicle_state()
	debug_set_rpm(idle_rpm)
	debug_set_speed(0.0)
	debug_set_gear(GearType.NEUTRAL)