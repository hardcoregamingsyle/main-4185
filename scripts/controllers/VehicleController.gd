extends CharacterBody2D
class_name VehicleController

## VehicleController - Core vehicle physics controller using PhysicsSettings constants
## Handles throttle, brake, steering inputs, wheel forces, and gear shifting logic
## Copyright 2026 Thalamus Racing Simulator Project

# Signals for external communication
signal acceleration_changed(throttle_amount: float)
signal braking_changed(brake_amount: float)
signal steering_changed(steering_angle: float)
signal gear_changed(old_gear: int, new_gear: int)
signal vehicle_speed_changed(speed: float)
signal vehicle_rotation_changed(rotation: float)
signal engine_rpm_changed(rpm: float)
signal skid_detected(skid_intensity: float)
signal crash_detected(force: Vector2)
signal lap_checkpoint_passed(checkpoint_id: int)

# Constants from PhysicsSettings (imported as autoload)
const PHYSICS_TICK_RATE := PhysicsSettings.physics_tick_rate
const MAX_SUBSTEPS := PhysicsSettings.max_substeps

@export_group("Vehicle Configuration")
@export var mass: float = 1500.0
@export var center_of_mass_offset: Vector2 = Vector2.ZERO
@export var suspension_stiffness: float = 50000.0
@export var suspension_damping: float = 5000.0
@export var max_wheel_travel: float = 0.15

@export_group("Wheel Configuration")
@export var front_wheel_width: float = 0.35
@export var rear_wheel_width: float = 0.38
@export var wheel_radius: float = 0.32
@export var wheel_separation: float = 1.45
@export var wheel_base: float = 2.5

@export_group("Performance Tuning")
@export var max_engine_power: float = 250000.0  # Watts
@export var max_torque: float = 500.0           # Nm
@export var brake_force: float = 15000.0        # Newtons per wheel
@export var traction_control_threshold: float = 0.3
@export var anti_roll_bar_stiffness: float = 15000.0

@export_group("Gear Ratios")
@export var final_drive_ratio: float = 3.73
@export var gear_ratios: Array[float] = [3.5, 2.1, 1.5, 1.1, 0.9, 0.7]
@export var reverse_ratio: float = 3.8

@export_group("Input Deadzones")
@export var steering_deadzone: float = 0.15
@export var throttle_deadzone: float = 0.1
@export var brake_deadzone: float = 0.1

var _current_gear: int = 0  # 0 = neutral, 1-6 = gears, -1 = reverse
var _engine_rpm: float = 0.0
var _max_rpm: float = 7500.0
var _min_rpm: float = 800.0
var _idle_rpm: float = 900.0

var _throttle_input: float = 0.0
var _brake_input: float = 0.0
var _steering_input: float = 0.0

var _wheel_forces: Dictionary = {
	"front_left": Vector2.ZERO,
	"front_right": Vector2.ZERO,
	"rear_left": Vector2.ZERO,
	"rear_right": Vector2.ZERO
}

var _wheel_rotation_angles: Dictionary = {
	"front_left": 0.0,
	"front_right": 0.0,
	"rear_left": 0.0,
	"rear_right": 0.0
}

var _skid_intensity: float = 0.0
var _drift_angle: float = 0.0

var _gear_shift_timer: float = 0.0
var _gear_shift_cooldown: float = 0.3

var _last_crash_time: float = 0.0
var _crash_force_threshold: float = 5000.0

func _ready() -> void:
	mass = PhysicsSettings.default_vehicle_mass if mass == 1500.0 else mass
	_process_mode = ProcessModeEnum.ALWAYS
	
	_init_wheels()
	_connect_signals_to_inputs()

func _init_wheels() -> void:
	_wheel_forces = {
		"front_left": Vector2.ZERO,
		"front_right": Vector2.ZERO,
		"rear_left": Vector2.ZERO,
		"rear_right": Vector2.ZERO
	}
	
	_wheel_rotation_angles = {
		"front_left": 0.0,
		"front_right": 0.0,
		"rear_left": 0.0,
		"rear_right": 0.0
	}

func _connect_signals_to_inputs() -> void:
	if InputManager.has_method("get_throttle"):
		InputManager.signal_throttle_changed.connect(_on_throttle_changed)
	if InputManager.has_method("get_brake"):
		InputManager.signal_brake_changed.connect(_on_brake_changed)
	if InputManager.has_method("get_steering"):
		InputManager.signal_steering_changed.connect(_on_steering_changed)

func _physics_process(delta: float) -> void:
	_handle_physics_step(delta)
	_update_wheels(delta)
	_apply_forces()
	_check_driving_conditions()

func _input_event(viewport: Viewport, event: InputEvent, shape_idx: int = -1) -> void:
	_handle_manual_gear_shift(event)

func _handle_physics_step(delta: float) -> void:
	_read_inputs()
	_calculate_transmission()
	apply_velocity(delta)

func _read_inputs() -> void:
	_throttle_input = clamp(InputManager.get_throttle(), -throttle_deadzone, 1.0)
	_brake_input = clamp(InputManager.get_brake(), -brake_deadzone, 1.0)
	_steering_input = clamp(InputManager.get_steering(), -1.0, 1.0)
	
	if abs(_steering_input) <= steering_deadzone:
		_steering_input = 0.0

func _calculate_transmission() -> void:
	var current_gear = _current_gear
	var target_gear = _determine_target_gear()
	
	# Apply gear shift cooldown
	if _gear_shift_timer > 0:
		_gear_shift_timer -= get_process_delta_time()
	
	if _gear_shift_timer <= 0 and current_gear != target_gear:
		_shift_gear(target_gear)
	
	# Calculate RPM based on current gear and speed
	_update_engine_rpm()

func _determine_target_gear() -> int:
	var speed = velocity.length()
	var throttle = _throttle_input
	var current_rpm = _engine_rpm
	
	if _current_gear == 0:  # Neutral
		if abs(_steering_input) < 0.1:
			return 1 if throttle > 0 else -1
		return 0
	
	if _current_gear < 0:  # Reverse
		if throttle > 0.2 and speed < 2.0:
			return 1
		return -1
	
	if throttle < 0.1 and current_rpm < _min_rpm + 500:
		if _current_gear > 1:
			return _current_gear - 1
		elif speed < 1.0:
			return 0
		return 1
	
	if throttle > 0.5 and current_rpm > _max_rpm - 1000:
		if _current_gear < gear_ratios.size():
			return _current_gear + 1
	
	if throttle > 0.8 and current_rpm > _max_rpm - 500:
		if _current_gear < gear_ratios.size():
			return _current_gear + 1
	
	if speed < 5.0 and _current_gear > 1 and throttle < 0.3:
		return 1
	
	return _current_gear

func _shift_gear(new_gear: int) -> void:
	if new_gear == _current_gear:
		return
	
	if _gear_shift_timer > 0:
		return
	
	var old_gear = _current_gear
	_current_gear = new_gear
	_gear_shift_timer = _gear_shift_cooldown
	
	emit_signal("gear_changed", old_gear, new_gear)
	
	if AudioManager:
		AudioManager.play_sound("gear_shift_" + str(new_gear))

func _update_engine_rpm() -> void:
	var speed = velocity.length()
	var effective_gear_ratio = _get_effective_gear_ratio()
	var wheel_angular_velocity = speed / wheel_radius if wheel_radius > 0 else 0.0
	var axle_angular_velocity = wheel_angular_velocity * effective_gear_ratio
	
	_engine_rpm = axle_angular_velocity * 60.0 / (2.0 * PI)
	
	if _current_gear == 0:
		_engine_rpm = lerp(_engine_rpm, _idle_rpm, 0.1)
	elif _current_gear < 0:
		_engine_rpm = max(_engine_rpm, _min_rpm)
	else:
		_engine_rpm = clamp(_engine_rpm, _min_rpm, _max_rpm)
	
	emit_signal("engine_rpm_changed", _engine_rpm)

func _get_effective_gear_ratio() -> float:
	var ratio: float
	
	if _current_gear == 0:
		ratio = 1.0
	elif _current_gear < 0:
		ratio = reverse_ratio
	else:
		ratio = gear_ratios[_current_gear - 1] if _current_gear <= gear_ratios.size() else gear_ratios.back()
	
	return ratio * final_drive_ratio

func _handle_manual_gear_shift(event: InputEvent) -> void:
	if event is InputEventKey and event.pressed:
		if event.keycode == KEY_UP or event.keycode == KEY_W:
			if _current_gear < gear_ratios.size():
				_shift_gear(_current_gear + 1)
		elif event.keycode == KEY_DOWN or event.keycode == KEY_S:
			if _current_gear > 1:
				_shift_gear(_current_gear - 1)
		elif event.keycode == KEY_Q:
			if _current_gear == 1:
				_shift_gear(-1)
			elif _current_gear == -1:
				_shift_gear(1)

func _update_wheels(delta: float) -> void:
	for wheel_name in _wheel_forces.keys():
		_update_single_wheel(wheel_name, delta)

func _update_single_wheel(wheel_name: String, delta: float) -> void:
	var force = _wheel_forces[wheel_name]
	var current_angle = _wheel_rotation_angles[wheel_name]
	
	# Update wheel rotation based on movement
	var distance_traveled = force.length() * delta
	var angle_change = distance_traveled / wheel_radius
	
	if wheel_name.begins_with("front"):
		current_angle += _steering_input * delta * 3.0
	
	_wheel_rotation_angles[wheel_name] = current_angle + angle_change

func _apply_forces() -> void:
	var forward_vector = Vector2.RIGHT.rotated(rotation)
	var lateral_vector = Vector2.UP.rotated(rotation)
	
	var total_drive_force = _calculate_drive_force(forward_vector)
	var total_brake_force = _calculate_brake_force(lateral_vector)
	var steering_effect = _calculate_steering_effect(lateral_vector)
	
	# Distribute forces to wheels
	_distribute_forces(total_drive_force, total_brake_force, steering_effect)
	
	# Apply wheel friction and drag
	_apply_wheel_friction()

func _calculate_drive_force(forward: Vector2) -> Vector2:
	var drive_force = Vector2.ZERO
	
	if _throttle_input > 0 and _current_gear >= 1:
		var rpm_factor = (_engine_rpm - _idle_rpm) / (_max_rpm - _idle_rpm)
		var torque_output = min(max_torque, max_torque * rpm_factor)
		
		var effective_ratio = _get_effective_gear_ratio()
		var wheel_torque = torque_output / effective_ratio
		
		var drive_per_wheel = wheel_torque / wheel_radius
		
		if _current_gear > 0:  # Forward gears
			drive_force = forward.normalized() * drive_per_wheel * 0.5
		elif _current_gear < 0:  # Reverse
			drive_force = -forward.normalized() * drive_per_wheel * 0.5
	
	return drive_force

func _calculate_brake_force(lateral: Vector2) -> Vector2:
	var brake_force = Vector2.ZERO
	
	if _brake_input > 0:
		var brake_per_wheel = brake_force * _brake_input
		brake_force = -velocity.normalized() * brake_per_wheel
	
	return brake_force

func _calculate_steering_effect(lateral: Vector2) -> Vector2:
	var steer_effect = Vector2.ZERO
	
	if abs(_steering_input) > 0.01:
		var steer_multiplier = _steering_input * 200.0
		steer_effect = lateral.normalized() * steer_multiplier
	
	return steer_effect

func _distribute_forces(drive: Vector2, brake: Vector2, steer: Vector2) -> void:
	var total_force = drive + brake + steer
	
	_front_wheel_force = total_force * 0.4
	_rear_wheel_force = total_force * 0.6
	
	_wheel_forces.front_left = _front_wheel_force * 0.5 + steer * 0.3
	_wheel_forces.front_right = _front_wheel_force * 0.5 - steer * 0.3
	_wheel_forces.rear_left = _rear_wheel_force * 0.5
	_wheel_forces.rear_right = _rear_wheel_force * 0.5

func _apply_wheel_friction() -> void:
	var friction_coefficient = 0.85
	var friction_force = -velocity.normalized() * friction_coefficient * mass * PhysicsSettings.gravity
	
	# Add aerodynamic drag
	var air_density = 1.225
	var drag_coefficient = 0.35
	var frontal_area = 2.0 * wheel_separation
	var air_resistance = -velocity.normalized() * 0.5 * air_density * drag_coefficient * frontal_area * velocity.length_squared()
	
	velocity += (friction_force + air_resistance) * get_process_delta_time() / mass

func _check_driving_conditions() -> void:
	_check_skidding()
	_check_crash_conditions()
	_check_lap_progress()

func _check_skidding() -> void:
	var speed = velocity.length()
	var velocity_direction = velocity.normalized()
	var facing_direction = Vector2.RIGHT.rotated(rotation).normalized()
	
	var slip_angle = facing_direction.angle() - velocity_direction.angle()
	slip_angle = wrapf(slip_angle, -PI, PI)
	
	_drift_angle = slip_angle
	
	if abs(slip_angle) > 0.4 and speed > 15.0:
		_skid_intensity = min(abs(slip_angle) / 1.0, 1.0)
		emit_signal("skid_detected", _skid_intensity)
	else:
		_skid_intensity = 0.0

func _check_crash_conditions() -> void:
	var current_time = Time.get_unix_time_from_system()
	
	if velocity.length() > 25.0 and current_time - _last_crash_time > 2.0:
		var impact_force = velocity.length() * mass
		if impact_force > _crash_force_threshold:
			_last_crash_time = current_time
			emit_signal("crash_detected", velocity * mass)
			
			if AudioManager:
				AudioManager.play_sound("crash_impact")

func _check_lap_progress() -> void:
	if GameManager.current_state == GameManager.GameState.RACE_ACTIVE:
		# Check collision with checkpoint nodes
		var checkpoints = get_tree().get_nodes_in_group("checkpoint")
		for checkpoint in checkpoints:
			if is_colliding_with(checkpoint):
				var checkpoint_id = checkpoint.get_node_or_null("CheckpointID")
				if checkpoint_id:
					emit_signal("lap_checkpoint_passed", checkpoint_id.checkpoint_value)

func apply_velocity(delta: float) -> void:
	var friction = PhysicsSettings.gravity * 0.02
	velocity *= (1.0 - friction * delta)
	add_to_velocity(velocity * 0.0)

func set_position_safe(pos: Vector2) -> void:
	position = pos

func set_rotation_safe(angle: float) -> void:
	rotation = angle

func get_speed_kmh() -> float:
	return velocity.length() * 3.6

func get_speed_mph() -> float:
	return velocity.length() * 2.237

func get_gear_label() -> String:
	match _current_gear:
		0: return "N"
		-1: return "R"
		_: return str(_current_gear)

func reset_vehicle() -> void:
	_current_gear = 0
	_engine_rpm = _idle_rpm
	_velocity = Vector2.ZERO
	_rotation = 0.0
	_wheel_forces = {
		"front_left": Vector2.ZERO,
		"front_right": Vector2.ZERO,
		"rear_left": Vector2.ZERO,
		"rear_right": Vector2.ZERO
	}

func _on_throttle_changed(value: float) -> void:
	emit_signal("acceleration_changed", value)

func _on_brake_changed(value: float) -> void:
	emit_signal("braking_changed", value)

func _on_steering_changed(value: float) -> void:
	emit_signal("steering_changed", value)

func get_wheel_positions() -> Dictionary:
	var half_track = wheel_separation / 2.0
	var half_base = wheel_base / 2.0
	
	return {
		"front_left": position + Vector2(half_base, half_track).rotated(rotation),
		"front_right": position + Vector2(half_base, -half_track).rotated(rotation),
		"rear_left": position + Vector2(-half_base, half_track).rotated(rotation),
		"rear_right": position + Vector2(-half_base, -half_track).rotated(rotation)
	}

func get_wheel_data() -> Dictionary:
	return {
		"front_left": {
			"force": _wheel_forces.front_left,
			"angle": _wheel_rotation_angles.front_left,
			"width": front_wheel_width,
			"radius": wheel_radius
		},
		"front_right": {
			"force": _wheel_forces.front_right,
			"angle": _wheel_rotation_angles.front_right,
			"width": front_wheel_width,
			"radius": wheel_radius
		},
		"rear_left": {
			"force": _wheel_forces.rear_left,
			"angle": _wheel_rotation_angles.rear_left,
			"width": rear_wheel_width,
			"radius": wheel_radius
		},
		"rear_right": {
			"force": _wheel_forces.rear_right,
			"angle": _wheel_rotation_angles.rear_right,
			"width": rear_wheel_width,
			"radius": wheel_radius
		}
	}

func set_physic_settings(settings: PhysicsSettings) -> void:
	mass = settings.default_vehicle_mass
	PhysicsSettings.gravity = settings.gravity
	PhysicsSettings.physics_tick_rate = settings.physics_tick_rate

func get_debug_info() -> Dictionary:
	return {
		"gear": _current_gear,
		"rpm": _engine_rpm,
		"speed_kmh": get_speed_kmh(),
		"speed_mph": get_speed_mph(),
		"throttle": _throttle_input,
		"brake": _brake_input,
		"steering": _steering_input,
		"skid_intensity": _skid_intensity,
		"drift_angle": _drift_angle,
		"wheel_forces": _wheel_forces.duplicate(true)
	}