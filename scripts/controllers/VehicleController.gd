extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulator
## Implements throttle, brake, steering inputs, wheel forces, and gear shifting logic
## Uses PhysicsSettings constants for centralized tuning
## Copyright 2026 Thalamus Racing Simulator Project

signal speed_changed(current_speed: float)
signal rpm_changed(rpm: float)
signal gear_changed(old_gear: int, new_gear: int)
signal skid_detected(skid_factor: float)
signal collision_impact(force: Vector3)

# ============================================================================
# CONSTANTS & CONFIGURATION (from PhysicsSettings)
# ============================================================================

const GEAR_RATIOS: Array[float] = [3.5, 2.2, 1.6, 1.2, 0.9, 0.7, 0.5]
const FINAL_DRIVE_RATIO: float = 3.73
const MAX_GEAR_COUNT: int = 6
const REVERSE_GEAR: int = -1

@export_group("Vehicle Parameters")
@export var vehicle_mass: float = 1500.0: set = _set_vehicle_mass
@export var center_of_mass_offset: Vector3 = Vector3(0, 0.3, 0): set = _set_center_of_mass_offset
@export var max_steering_angle_degrees: float = 30.0: set = _set_max_steering_angle_degrees
@export var steering_sensitivity: float = 1.0: set = _set_steering_sensitivity

@export_group("Powertrain Parameters")
@export var engine_max_rpm: float = 7000.0: set = _set_engine_max_rpm
@export var engine_min_rpm: float = 800.0: set = _set_engine_min_rpm
@export var idle_rpm: float = 800.0: set = _set_idle_rpm
@export var redline_rpm: float = 7200.0: set = _set_redline_rpm
@export var engine_torque_curve: Dictionary = {0: 0.0, 2000: 250.0, 4000: 350.0, 6000: 320.0, 7000: 280.0}

@export_group("Brake System")
@export var brake_force_per_wheel: float = 8000.0: set = _set_brake_force_per_wheel
@export var abs_enabled: bool = true
@export var brake_bias_front: float = 0.6: set = _set_brake_bias_front

@export_group("Aerodynamics")
@export var drag_coefficient: float = 0.32: set = _set_drag_coefficient
@export var frontal_area: float = 2.2: set = _set_frontal_area
@export var downforce_coefficient: float = 0.5: set = _set_downforce_coefficient

# ============================================================================
# PRIVATE STATE VARIABLES
# ============================================================================

var _current_rpm: float = 0.0
var _current_gear: int = 0
var _throttle_input: float = 0.0
var _brake_input: float = 0.0
var _steering_input: float = 0.0
var _clutch_pedal_pressed: bool = false
var _handbrake_active: bool = false
var _drift_mode: bool = false
var _wheel_slip_ratio: float = 0.0
var _tire_skid_factor: float = 0.0

# Wheel state tracking
var _wheel_states: Dictionary = {}
var _wheel_damages: Dictionary = {}

# Physics state
var _vehicle_velocity: Vector3 = Vector3.ZERO
var _angular_velocity: Vector3 = Vector3.ZERO
var _ground_contact_normal: Vector3 = Vector3.UP
var _surface_friction: float = 1.0
var _traction_loss_factor: float = 1.0

# Engine state
var _engine_load: float = 0.0
var _fuel_level: float = 100.0
var _engine_temperature: float = 90.0
var _oil_pressure: float = 45.0

# ============================================================================
# PHYSICS SETTINGS REFERENCES
# ============================================================================

var _physics_settings: PhysicsSettings

func _ready() -> void:
	_init_wheel_states()
	_connect_signals()
	_calculate_initial_values()
	
	if GameManager.has_singleton():
		_physics_settings = GameManager.get_singleton().get_tree().root.get_node_or_null("PhysicsSettings")
		if not _physics_settings:
			_physics_settings = preload("res://scripts/core/PhysicsSettings.gd").new()

func _init_wheel_states() -> void:
	# Initialize states for each wheel (front-left, front-right, rear-left, rear-right)
	var wheel_names = ["FL", "FR", "RL", "RR"]
	for name in wheel_names:
		_wheel_states[name] = {
			"rotation": 0.0,
			"slip_ratio": 0.0,
			"camber_angle": 0.0,
			"toe_angle": 0.0,
			"suspension_compression": 0.0,
			"force_output": Vector3.ZERO,
			"is_locked": false
		}
		_wheel_damages[name] = 0.0

func _connect_signals() -> void:
	if GameManager.has_singleton():
		GameManager.game_state_changed.connect(_on_game_state_changed)

func _calculate_initial_values() -> void:
	_current_rpm = idle_rpm
	_current_gear = 0
	_throttle_input = 0.0
	_brake_input = 0.0
	_steering_input = 0.0

# ============================================================================
# INPUT HANDLING
# ============================================================================

func _input(event: InputEvent) -> void:
	if event.is_action_pressed("vehicle_throttle_up"):
		_update_throttle_input(Input.get_axis("brake", "accelerator"))
	elif event.is_action_released("vehicle_throttle_up"):
		pass
	elif event.is_action_pressed("vehicle_steering_left"):
		_steering_input = clamp(_steering_input - 0.1, -1.0, 1.0)
	elif event.is_action_pressed("vehicle_steering_right"):
		_steering_input = clamp(_steering_input + 0.1, -1.0, 1.0)
	elif event.is_action_released("vehicle_steering"):
		_steering_input = 0.0
	elif event.is_action_pressed("vehicle_shift_up"):
		_shift_gear(true)
	elif event.is_action_pressed("vehicle_shift_down"):
		_shift_gear(false)
	elif event.is_action_pressed("vehicle_handbrake"):
		_handbrake_active = true
	elif event.is_action_released("vehicle_handbrake"):
		_handbrake_active = false

func _process(delta: float) -> void:
	_handle_continuous_inputs(delta)
	_update_physics(delta)
	_update_engine(delta)
	_update_wheels(delta)
	_check_collision_events()
	_apply_aerodynamics(delta)

func _handle_continuous_inputs(delta: float) -> void:
	# Smooth throttle input
	var target_throttle = Input.get_axis("brake", "accelerator")
	_throttle_input = lerp(_throttle_input, target_throttle, delta * 5.0)
	
	# Clamp throttle to valid range
	_throttle_input = clamp(_throttle_input, -1.0, 1.0)
	
	# Smooth steering input
	var target_steering = Input.get_axis("steer_left", "steer_right")
	_steering_input = lerp(_steering_input, target_steering, delta * 8.0)
	_steering_input = clamp(_steering_input, -1.0, 1.0)
	
	# Brake input (separate from reverse)
	var brake_target = Input.get_axis("brake", "")
	_brake_input = brake_target if brake_target > 0 else 0.0
	
	# Clutch handling
	_clutch_pedal_pressed = Input.is_action_pressed("vehicle_clutch")

# ============================================================================
# PHYSICS UPDATE
# ============================================================================

func _update_physics(delta: float) -> void:
	# Apply gravity from PhysicsSettings
	var gravity = _physics_settings.gravity if _physics_settings else 9.81
	var velocity_scale = _physics_settings.time_scale if _physics_settings else 1.0
	delta *= velocity_scale
	
	# Calculate current speed from body velocity
	_vehicle_velocity = linear_interpolate(_vehicle_velocity, velocity, delta * 10.0)
	var current_speed = _vehicle_velocity.length()
	
	# Update RPM based on gear ratio and speed
	_update_rpm_from_speed(current_speed)
	
	# Calculate traction forces
	var traction_force = _calculate_traction_forces()
	var braking_force = _calculate_braking_force()
	var aerodynamic_downforce = _calculate_downforce(current_speed)
	
	# Apply forces to vehicle body
	_apply_vehicle_forces(traction_force, braking_force, aerodynamic_downforce, gravity * vehicle_mass)
	
	# Handle wheel slip and skidding
	_update_wheel_slip()
	
	# Update ground contact
	_update_ground_contact()
	
	# Emit signals
	emit_signal("speed_changed", current_speed)
	emit_signal("rpm_changed", _current_rpm)

func _update_rpm_from_speed(speed: float) -> void:
	var gear_ratio = _get_effective_gear_ratio()
	var wheel_radius = 0.3 # Standard tire radius in meters
	var final_drive = FINAL_DRIVE_RATIO
	
	# Calculate expected RPM based on speed and gear
	var wheel_rotation_rate = speed / (wheel_radius * 2.0 * PI)
	var expected_rpm = wheel_rotation_rate * gear_ratio * final_drive * 60.0
	
	# Blend between calculated RPM and actual RPM for smooth transition
	_current_rpm = lerp(_current_rpm, expected_rpm, 0.1)
	
	# Ensure RPM stays within bounds
	_current_rpm = clamp(_current_rpm, engine_min_rpm, redline_rpm)

func _get_effective_gear_ratio() -> float:
	if _current_gear < 0:
		return GEAR_RATIOS[REVERSE_GEAR] * 1.2 # Reverse has slightly different ratio
	elif _current_gear >= GEAR_RATIOS.size():
		return GEAR_RATIOS[max(0, GEAR_RATIOS.size() - 1)]
	else:
		return GEAR_RATIOS[_current_gear]

func _calculate_traction_forces() -> float:
	var torque = _calculate_engine_torque()
	var gear_ratio = _get_effective_gear_ratio()
	var final_drive = FINAL_DRIVE_RATIO
	var wheel_radius = 0.3
	
	var total_ratio = gear_ratio * final_drive
	var drive_force = torque * total_ratio / wheel_radius
	
	# Apply clutch effect
	if _clutch_pedal_pressed:
		drive_force *= 0.1
	
	# Apply throttle modifier
	drive_force *= _throttle_input
	
	# Apply traction loss due to slip
	drive_force *= _traction_loss_factor
	
	return drive_force

func _calculate_engine_torque() -> float:
	var rpm = _current_rpm
	
	# Linear interpolation through torque curve
	var torque = 0.0
	var keys = engine_torque_curve.keys()
	for i in range(keys.size() - 1):
		if rpm >= keys[i] and rpm <= keys[i + 1]:
			var t = (rpm - keys[i]) / (keys[i + 1] - keys[i])
			torque = engine_torque_curve[keys[i]] + t * (engine_torque_curve[keys[i + 1]] - engine_torque_curve[keys[i]])
			break
		elif rpm >= keys.max():
			torque = engine_torque_curve[keys[keys.size() - 1]]
	
	return torque

func _calculate_braking_force() -> float:
	if _brake_input <= 0 and not _handbrake_active:
		return 0.0
	
	var base_force = brake_force_per_wheel * 4.0 # Four wheels
	
	if _handbrake_active:
		base_force *= 0.5 # Handbrake affects primarily rear wheels
	
	# Apply ABS logic
	if abs_enabled and _wheel_slip_ratio > 0.3:
		base_force *= 0.7
	
	return base_force * _brake_input

func _apply_vehicle_forces(traction: float, braking: float, downforce: float, weight: float) -> void:
	# Calculate net forward force
	var forward_force = traction - braking
	
	# Apply force along vehicle's forward direction
	var forward_dir = transform.basis.z
	forward_dir.y = 0
	forward_dir = forward_dir.normalized()
	
	# Apply acceleration
	var acceleration = forward_force / vehicle_mass
	velocity += acceleration * Vector3.FORWARD * get_physics_process_delta_time()
	
	# Apply downforce (affects normal force, thus friction)
	var vertical_force = weight - downforce
	var normal_force = max(vertical_force, 0)
	
	# Adjust surface friction based on downforce
	_surface_friction = 1.0 + (downforce / (weight * 0.5))

func _update_wheel_slip() -> void:
	var wheel_radius = 0.3
	var wheel_linear_speed = _vehicle_velocity.length()
	var wheel_rotational_speed = _wheel_states["FL"]["rotation"] / get_physics_process_delta_time()
	
	# Calculate slip ratio
	var slip_ratio = (wheel_linear_speed - wheel_rotational_speed * wheel_radius) / max(wheel_linear_speed, 0.1)
	_wheel_slip_ratio = clamp(slip_ratio, -0.5, 0.5)
	
	# Update skid factor for drift detection
	_tire_skid_factor = abs(_wheel_slip_ratio)
	
	if _tire_skid_factor > 0.25:
		emit_signal("skid_detected", _tire_skid_factor)

func _update_ground_contact() -> void:
	var ray_result = move_and_collide(Vector3.DOWN * 2.0, true)
	if ray_result:
		_ground_contact_normal = ray_result.normal
		_surface_friction = ray_result.collider.surface_get_param(0) if ray_result.collider.has_method("surface_get_param") else 1.0

# ============================================================================
# ENGINE MANAGEMENT
# ============================================================================

func _update_engine(delta: float) -> void:
	# Update engine load based on throttle and RPM
	_engine_load = _throttle_input * (_current_rpm / engine_max_rpm)
	_engine_load = clamp(_engine_load, 0.0, 1.0)
	
	# Simulate temperature changes
	var temp_change = _engine_load * 0.5 - (_current_rpm / redline_rpm) * 0.2
	_engine_temperature += temp_change * delta
	_engine_temperature = clamp(_engine_temperature, 60.0, 120.0)
	
	# Oil pressure decreases with lower RPM
	_oil_pressure = 45.0 + (_current_rpm / engine_max_rpm) * 15.0
	_oil_pressure = clamp(_oil_pressure, 30.0, 60.0)

func shift_gear(gear_number: int) -> void:
	if gear_number < -1 or gear_number >= GEAR_RATIOS.size():
		return
	
	var old_gear = _current_gear
	_current_gear = gear_number
	
	if old_gear != gear_number:
		emit_signal("gear_changed", old_gear, gear_number)

func _shift_gear(up: bool) -> void:
	var old_gear = _current_gear
	var new_gear = old_gear + (1 if up else -1)
	
	# Boundary checks
	if up and new_gear >= MAX_GEAR_COUNT:
		new_gear = MAX_GEAR_COUNT - 1
	elif down and new_gear < REVERSE_GEAR:
		new_gear = REVERSE_GEAR
	
	# Only shift if RPM allows
	var rpm_threshold = engine_min_rpm if not up else engine_max_rpm - 1000
	if up and _current_rpm < rpm_threshold:
		return
	
	if old_gear != new_gear:
		_current_gear = new_gear
		emit_signal("gear_changed", old_gear, new_gear)

func _calculate_downforce(speed: float) -> float:
	# Downforce scales with square of speed
	var dynamic_pressure = 0.5 * 1.225 * pow(speed, 2) # Air density ~1.225 kg/m³
	var downforce = dynamic_pressure * drag_coefficient * frontal_area * downforce_coefficient
	return downforce

func _apply_aerodynamics(delta: float) -> void:
	var air_density = 1.225 # kg/m³ at sea level
	var speed = _vehicle_velocity.length()
	
	# Drag force opposes motion
	var drag_force = 0.5 * air_density * pow(speed, 2) * drag_coefficient * frontal_area
	var drag_direction = -_vehicle_velocity.normalized() if speed > 0 else Vector3.ZERO
	
	# Apply drag deceleration
	var drag_acceleration = drag_force / vehicle_mass
	velocity -= drag_direction * drag_acceleration * get_physics_process_delta_time()

# ============================================================================
# WHEEL MANAGEMENT
# ============================================================================

func _update_wheels(delta: float) -> void:
	var wheel_radius = 0.3
	
	for name in _wheel_states:
		var state = _wheel_states[name]
		
		# Update wheel rotation based on vehicle speed
		var wheel_speed = _vehicle_velocity.length()
		var rotation_increment = wheel_speed / wheel_radius * delta
		
		state.rotation += rotation_increment
		state.slip_ratio = _wheel_slip_ratio
		
		# Apply camber and toe based on steering
		if name.begins_with("F"): # Front wheels steer
			state.camber_angle = -_steering_input * 2.0
			state.toe_angle = _steering_input * 0.5
		
		# Check for damage accumulation
		if _tire_skid_factor > 0.3:
			_wheel_damages[name] += _tire_skid_factor * delta * 10.0
			_wheel_damages[name] = min(_wheel_damages[name], 100.0)

func get_wheel_state(wheel_name: String) -> Dictionary:
	return _wheel_states[wheel_name] if _wheel_states.has(wheel_name) else {}

func get_wheel_damage(wheel_name: String) -> float:
	return _wheel_damages[wheel_name] if _wheel_damages.has(wheel_name) else 0.0

func reset_wheel_damage(wheel_name: String) -> void:
	if _wheel_damages.has(wheel_name):
		_wheel_damages[wheel_name] = 0.0

# ============================================================================
# COLLISION HANDLING
# ============================================================================

func _check_collision_events() -> void:
	if is_on_floor():
		var colliding_body = get_collision_mask()
		if colliding_body:
			# Get collision normal and force
			var collision_info = get_slide_collision(0)
			if collision_info:
				var impact_force = collision_info.get_normal()
				emit_signal("collision_impact", impact_force)

func _on_game_state_changed(new_state: GameState) -> void:
	if new_state == GameState.RACE_ACTIVE:
		_resume_engine()
	elif new_state == GameState.RACE_PAUSED:
		_pause_engine()

func _resume_engine() -> void:
	_current_rpm = idle_rpm
	_throttle_input = 0.0
	_brake_input = 0.0

func _pause_engine() -> void:
	_current_rpm = 0.0
	_throttle_input = 0.0
	_brake_input = 0.0

# ============================================================================
# PUBLIC API FOR EXTERNAL ACCESS
# ============================================================================

func get_current_speed() -> float:
	return _vehicle_velocity.length()

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

func get_engine_load() -> float:
	return _engine_load

func get_engine_temperature() -> float:
	return _engine_temperature

func get_fuel_level() -> float:
	return _fuel_level

func set_fuel_level(level: float) -> void:
	_fuel_level = clamp(level, 0.0, 100.0)

func refuel(amount: float) -> void:
	_fuel_level = min(_fuel_level + amount, 100.0)

func drain_fuel(amount: float) -> void:
	_fuel_level = max(_fuel_level - amount, 0.0)

func activate_drift_mode(active: bool) -> void:
	_drift_mode = active
	if active:
		_traction_loss_factor = 0.5
	else:
		_traction_loss_factor = 1.0

func is_drifting() -> bool:
	return _drift_mode and _tire_skid_factor > 0.3

func get_all_wheel_damages() -> Dictionary:
	return _wheel_damages.duplicate()

# ============================================================================
# PROPERTY SETTERS WITH VALIDATION
# ============================================================================

func _set_vehicle_mass(value: float) -> void:
	if value > 0:
		vehicle_mass = value

func _set_center_of_mass_offset(value: Vector3) -> void:
	center_of_mass_offset = value

func _set_max_steering_angle_degrees(value: float) -> void:
	if value > 0 and value <= 60:
		max_steering_angle_degrees = value

func _set_steering_sensitivity(value: float) -> void:
	if value > 0 and value <= 2:
		steering_sensitivity = value

func _set_engine_max_rpm(value: float) -> void:
	if value > 1000 and value < 15000:
		engine_max_rpm = value

func _set_engine_min_rpm(value: float) -> void:
	if value > 0 and value < engine_max_rpm:
		engine_min_rpm = value

func _set_idle_rpm(value: float) -> void:
	if value > 0 and value < engine_min_rpm:
		idle_rpm = value

func _set_redline_rpm(value: float) -> void:
	if value > engine_max_rpm:
		redline_rpm = value

func _set_brake_force_per_wheel(value: float) -> void:
	if value > 0 and value < 20000:
		brake_force_per_wheel = value

func _set_brake_bias_front(value: float) -> void:
	if value >= 0.3 and value <= 0.8:
		brake_bias_front = value

func _set_drag_coefficient(value: float) -> void:
	if value > 0 and value < 2:
		drag_coefficient = value

func _set_frontal_area(value: float) -> void:
	if value > 0 and value < 10:
		frontal_area = value

func _set_downforce_coefficient(value: float) -> void:
	if value >= 0 and value < 5:
		downforce_coefficient = value

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================

func get_total_weight() -> float:
	return vehicle_mass * _physics_settings.gravity if _physics_settings else vehicle_mass * 9.81

func calculate_power_to_weight_ratio() -> float:
	var power_hp = _calculate_engine_power()
	return power_hp / (vehicle_mass / 1000.0) # kW per ton

func _calculate_engine_power() -> float:
	var torque = _calculate_engine_torque()
	var power_kw = torque * _current_rpm / 9549.0 # Convert to kW
	return power_kw

func check_vehicle_health() -> Dictionary:
	return {
		"fuel_level": _fuel_level,
		"engine_temp": _engine_temperature,
		"oil_pressure": _oil_pressure,
		"wheels": _wheel_damages,
		"overall_health": _calculate_overall_health()
	}

func _calculate_overall_health() -> float:
	var health = 100.0
	
	# Deduct from fuel
	if _fuel_level < 10:
		health -= 10
	elif _fuel_level < 5:
		health -= 20
	
	# Deduct from engine temp
	if _engine_temperature > 110:
		health -= 15
	elif _engine_temperature > 100:
		health -= 5
	
	# Deduct from wheel damages
	var avg_wheel_damage = 0.0
	var damage_count = 0
	for damage in _wheel_damages.values():
		avg_wheel_damage += damage
		damage_count += 1
	
	if damage_count > 0:
		avg_wheel_damage /= damage_count
		health -= avg_wheel_damage * 0.5
	
	return max(health, 0.0)

func reset_vehicle_state() -> void:
	_current_rpm = idle_rpm
	_current_gear = 0
	_throttle_input = 0.0
	_brake_input = 0.0
	_steering_input = 0.0
	_fuel_level = 100.0
	_engine_temperature = 90.0
	_oil_pressure = 45.0
	
	for name in _wheel_damages:
		_wheel_damages[name] = 0.0

func _to_string() -> String:
	return "VehicleController[gear=%d,rpm=%.0f,speed=%.1f]" % [_current_gear, _current_rpm, _vehicle_velocity.length()]