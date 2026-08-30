extends Node2D
class_name VehicleController

## VehicleController - Core vehicle physics controller using PhysicsSettings constants
## Handles throttle, brake, steering inputs, wheel forces, and gear shifting logic
## Manages vehicle dynamics through centralized PhysicsSettings configuration
## Copyright 2026 Thalamus Racing Simulator Project

signal engine_rpm_changed(rpm: float)
signal vehicle_speed_changed(speed: float)
signal gear_changed(old_gear: int, new_gear: int)
signal vehicle_moved(distance: float)
signal vehicle_crashed(impact_force: float)

@export_group("Vehicle Configuration")
@export var max_engine_rpm: float = 8000.0
@export var idle_rpm: float = 900.0
@export var min_rpm: float = 600.0
@export var redline_shift_rpm: float = 7500.0

@export_group("Drivetrain Settings")
@export var drivetrain_type: DrivetrainType = DrivetrainType.FWD
enum DrivetrainType { FWD, RWD, AWD }

@export_group("Gear Ratios")
@export var gear_ratios: Array[float] = []
@export var final_drive_ratio: float = 3.5
@export var tire_radius: float = 0.32

# Runtime state variables
var _current_rpm: float = 0.0
var _current_gear: int = 0  # 0 = neutral, 1+ = forward gears, -1 = reverse
var _vehicle_speed: float = 0.0  # m/s
var _distance_traveled: float = 0.0
var _is_engine_on: bool = false
var _is_braking: bool = false
var _brake_pressure: float = 0.0

# Input state from InputManager
var _throttle_input: float = 0.0  # 0.0 to 1.0
var _brake_input: float = 0.0    # 0.0 to 1.0
var _steering_input: float = 0.0 # -1.0 to 1.0
var _clutch_input: float = 1.0   # 0.0 to 1.0 (not used yet, reserved)

# Physics references
var _powertrain: Powertrain = null
var _physics_settings: PhysicsSettings = null

func _ready() -> void:
	_init_physics_settings()
	_init_powertrain()
	_setup_gear_ratios()
	_connect_signals()
	
	_current_rpm = idle_rpm
	_is_engine_on = true
	emit_signal("engine_rpm_changed", _current_rpm)
	
	print("[VehicleController] Initialized successfully")

func _init_physics_settings() -> void:
	if Engine.has_singleton("PhysicsSettings"):
		_physics_settings = Engine.get_singleton("PhysicsSettings")
	else:
		_physics_settings = preload("res://scripts/core/PhysicsSettings.gd").new()
		print("[VehicleController] Warning: Using local PhysicsSettings instance")

func _init_powertrain() -> void:
	if has_node("../Powertrain"):
		_powertrain = get_node("../Powertrain") as Powertrain
	elif has_node("Powertrain"):
		_powertrain = get_node("Powertrain") as Powertrain
	else:
		_powertrain = Powertrain.new()
		add_child(_powertrain)
		_powertrain.name = "Powertrain"
		_powertrain.owner = self
	
	_powertrain.rpm_changed.connect(_on_powertrain_rpm_changed)

func _setup_gear_ratios() -> void:
	if gear_ratios.is_empty():
		_default_gear_ratios()

func _default_gear_ratios() -> void:
	gear_ratios = [
		3.5,   # 1st gear
		2.3,   # 2nd gear
		1.7,   # 3rd gear
		1.3,   # 4th gear
		1.0,   # 5th gear
		0.8,   # 6th gear
		0.65,  # Reverse
	]

func _connect_signals() -> void:
	GameManager.game_state_changed.connect(_on_game_state_changed)
	InputManager.input_updated.connect(_on_input_updated)

func _process(delta: float) -> void:
	if not _is_engine_on: return
	
	_update_engine_rpm(delta)
	_update_vehicle_dynamics(delta)
	_handle_auto_shifting(delta)

func _update_engine_rpm(delta: float) -> void:
	var target_rpm: float = _calculate_target_rpm()
	var rpm_change_rate: float = 1200.0 * delta  # RPM per second
	
	if target_rpm > _current_rpm:
		_current_rpm = min(target_rpm, max_engine_rpm)
	else:
		_current_rpm = max(target_rpm, idle_rpm if _is_engine_on else min_rpm)
	
	if abs(_current_rpm - target_rpm) < 10.0:
		_current_rpm = target_rpm
	
	emit_signal("engine_rpm_changed", _current_rpm)

func _calculate_target_rpm() -> float:
	var target_rpm: float = idle_rpm
	
	match _current_gear:
		0:  # Neutral
			target_rpm = idle_rpm
		-1:  # Reverse
			target_rpm = _calculate_rpm_for_reverse()
		_:  # Forward gears
			target_rpm = _calculate_rpm_for_forward()
	
	return target_rpm

func _calculate_rpm_for_forward() -> float:
	if _vehicle_speed <= 0:
		return idle_rpm + (_throttle_input * (redline_shift_rpm - idle_rpm))
	
	var gear_ratio: float = gear_ratios[_current_gear - 1]
	var wheel_rpm: float = (_vehicle_speed / (tire_radius * 2.0 * PI)) * 60.0
	var engine_rpm: float = wheel_rpm * gear_ratio * final_drive_ratio
	
	return engine_rpm

func _calculate_rpm_for_reverse() -> float:
	if _vehicle_speed >= 0:
		return idle_rpm
	
	var gear_ratio: float = gear_ratios[6]  # Reverse ratio
	var wheel_rpm: float = (abs(_vehicle_speed) / (tire_radius * 2.0 * PI)) * 60.0
	var engine_rpm: float = wheel_rpm * gear_ratio * final_drive_ratio
	
	return engine_rpm

func _update_vehicle_dynamics(delta: float) -> void:
	var acceleration: float = _calculate_acceleration(delta)
	
	if not _is_braking:
		_vehicle_speed += acceleration * delta
	
	if _vehicle_speed < 0:
		_vehicle_speed = 0.0
	
	if _vehicle_speed > 0:
		_distance_traveled += _vehicle_speed * delta
	
	emit_signal("vehicle_speed_changed", _vehicle_speed)
	emit_signal("vehicle_moved", _distance_traveled)

func _calculate_acceleration(delta: float) -> float:
	var mass: float = _physics_settings.default_vehicle_mass
	
	if _current_gear == 0:
		return 0.0
	
	var gear_ratio: float = gear_ratios[_current_gear - 1]
	var torque: float = _calculate_engine_torque()
	var wheel_torque: float = torque * gear_ratio * final_drive_ratio
	var wheel_force: float = wheel_torque / tire_radius
	
	var drag_coefficient: float = 0.35
	var air_density: float = 1.225
	var frontal_area: float = 2.2
	var air_resistance: float = 0.5 * air_density * frontal_area * drag_coefficient * _vehicle_speed * _vehicle_speed
	
	var rolling_resistance: float = mass * 9.81 * 0.015
	
	var net_force: float = wheel_force - air_resistance - rolling_resistance
	var acceleration: float = net_force / mass
	
	return acceleration

func _calculate_engine_torque() -> float:
	var rpm_normalized: float = _current_rpm / max_engine_rpm
	rpm_normalized = clamp(rpm_normalized, 0.0, 1.0)
	
	var peak_torque_rpm: float = 4000.0
	var torque_curve: Dictionary = {}
	torque_curve[0.0] = 0.3
	torque_curve[0.3] = 0.5
	torque_curve[0.5] = 1.0
	torque_curve[0.7] = 0.95
	torque_curve[1.0] = 0.7
	
	var max_torque: float = 350.0
	var torque: float = _interpolate_torque(rpm_normalized, torque_curve) * max_torque
	
	if _throttle_input < 1.0:
		torque *= _throttle_input
	
	return torque

func _interpolate_torque(normalized_rpm: float, curve: Dictionary) -> float:
	var keys: Array[float] = curve.keys()
	keys.sort()
	
	for i in range(keys.size() - 1):
		if normalized_rpm >= keys[i] and normalized_rpm <= keys[i + 1]:
			var t: float = (normalized_rpm - keys[i]) / (keys[i + 1] - keys[i])
			return curve[keys[i]] + t * (curve[keys[i + 1]] - curve[keys[i]])
	
	return curve[keys.back()]

func _handle_auto_shifting(delta: float) -> void:
	if _throttle_input < 0.01 and _current_gear > 0:
		_shift_down(delta)
		return
	
	if _current_rpm >= redline_shift_rpm and _current_gear < gear_ratios.size() - 1:
		_shift_up(delta)

func _shift_up(delta: float) -> void:
	var old_gear: int = _current_gear
	_current_gear += 1
	
	var clutch_drop: float = _clutch_input
	var rev_match: float = _calculate_rev_match()
	
	await _apply_clutch_transition(clutch_drop, rev_match)
	
	emit_signal("gear_changed", old_gear, _current_gear)
	print("[VehicleController] Shifted up: %d → %d" % [old_gear, _current_gear])

func _shift_down(delta: float) -> void:
	if _current_gear <= 0:
		return
	
	var old_gear: int = _current_gear
	_current_gear -= 1
	
	var clutch_drop: float = _clutch_input
	var rev_match: float = _calculate_rev_match()
	
	await _apply_clutch_transition(clutch_drop, rev_match)
	
	emit_signal("gear_changed", old_gear, _current_gear)
	print("[VehicleController] Shifted down: %d → %d" % [old_gear, _current_gear])

func _calculate_rev_match() -> float:
	var current_wheel_rpm: float = _get_wheel_rpm()
	var target_engine_rpm: float = _calculate_target_rpm()
	
	return target_engine_rpm - current_wheel_rpm

func _get_wheel_rpm() -> float:
	return (_vehicle_speed / (tire_radius * 2.0 * PI)) * 60.0

func _apply_clutch_transition(drop_amount: float, rev_match: float) -> void:
	pass

func _on_powertrain_rpm_changed(new_rpm: float) -> void:
	_current_rpm = new_rpm
	emit_signal("engine_rpm_changed", _current_rpm)

func _on_input_updated(input_data: Dictionary) -> void:
	_throttle_input = input_data.get("throttle", 0.0)
	_brake_input = input_data.get("brake", 0.0)
	_steering_input = input_data.get("steering", 0.0)
	_clutch_input = input_data.get("clutch", 1.0)
	
	_is_braking = _brake_input > 0.1

func _on_game_state_changed(new_state: GameState) -> void:
	match new_state:
		GameState.RACE_ACTIVE:
			_is_engine_on = true
			if _current_gear == 0:
				_current_gear = 1
		GameState.RACE_PAUSED:
			_is_engine_on = false
		GameState.MAIN_MENU:
			_reset_vehicle_state()

func _reset_vehicle_state() -> void:
	_current_rpm = idle_rpm
	_current_gear = 0
	_vehicle_speed = 0.0
	_distance_traveled = 0.0
	_is_engine_on = false
	_is_braking = false
	_throttle_input = 0.0
	_brake_input = 0.0
	_steering_input = 0.0

public func set_throttle(value: float) -> void:
	_throttle_input = clamp(value, 0.0, 1.0)

public func set_brake(value: float) -> void:
	_brake_input = clamp(value, 0.0, 1.0)
	_is_braking = _brake_input > 0.1

public func set_steer(value: float) -> void:
	_steering_input = clamp(value, -1.0, 1.0)

public func shift_gear(gear: int) -> void:
	if gear < -1 or gear > gear_ratios.size() - 1:
		push_warning("[VehicleController] Invalid gear requested: %d" % gear)
		return
	
	if gear == 0 and _current_gear != 0:
		var old_gear: int = _current_gear
		_current_gear = 0
		emit_signal("gear_changed", old_gear, _current_gear)
	elif gear != 0 and _current_gear == 0:
		_current_gear = gear
		emit_signal("gear_changed", 0, gear)
	elif gear != _current_gear:
		var old_gear: int = _current_gear
		_current_gear = gear
		emit_signal("gear_changed", old_gear, gear)

public func emergency_stop() -> void:
	_is_braking = true
	_brake_pressure = 1.0
	_throttle_input = 0.0
	_current_gear = 0

public func reset() -> void:
	_reset_vehicle_state()

public func get_rpm() -> float:
	return _current_rpm

public func get_speed() -> float:
	return _vehicle_speed

public func get_speed_kmh() -> float:
	return _vehicle_speed * 3.6

public func get_gear() -> int:
	return _current_gear

public func get_distance() -> float:
	return _distance_traveled

public func is_engine_on() -> bool:
	return _is_engine_on

public func is_braking() -> bool:
	return _is_braking

func _on_vehicle_crashed(impact: float) -> void:
	emit_signal("vehicle_crashed", impact)

</FILE>