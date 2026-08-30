extends Node
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulation
## Handles throttle, brake, steering inputs, wheel forces, and gear shifting logic
## Integrates with PhysicsSettings for configurable physics constants
## Designed as a base class for specific vehicle types (Cars, Motorcycles, etc.)
## Copyright 2026 Thalamus Racing Simulator Project

# Required references (set by parent vehicle script)
@onready var car_body: CharacterBody3D = $CarBody
@onready var suspension_system: Node = $SuspensionSystem
@onready var powertrain: Node = $Powertrain

# Signals for external communication
signal speed_changed(new_speed: float)
signal rpm_changed(new_rpm: float)
signal gear_changed(old_gear: int, new_gear: int)
signal throttle_input_changed(value: float)
signal brake_input_changed(value: float)
signal steering_input_changed(value: float)
signal vehicle_moved(position: Vector3, velocity: Vector3)
signal collision_detected(collision_info: Dictionary)

# Input state tracking
var _throttle_input: float = 0.0
var _brake_input: float = 0.0
var _steering_input: float = 0.0
var _reverse_input: bool = false

# Current vehicle state
var current_speed: float = 0.0  # m/s
var current_rpm: float = 0.0    # engine RPM
var current_gear: int = 1       # 1-7, 0 = neutral, -1 = reverse
var max_rpm: float = 8000.0     # Redline
var idle_rpm: float = 800.0     # Idle speed
var clutch_engaged: bool = true

# Wheel grip and traction control
var wheel_grip: float = 1.0     # 0.0-1.0, affected by surface and weather
var traction_control_active: bool = true

# Gear ratios (gear_ratio = final_drive * gearbox[gear])
var _gear_ratios: Array[float] = [4.5, 3.2, 2.1, 1.5, 1.1, 0.9, 0.7]
var _final_drive_ratio: float = 3.73

# Performance limits
var max_acceleration: float = 0.0
var max_deceleration: float = 0.0
var max_steering_angle: float = 0.5  # radians (~28 degrees)

# Physics constants from global settings
var physics_settings: PhysicsSettings = PhysicsSettings.new()
var vehicle_mass: float = 1500.0
var wheel_base: float = 2.7
var track_width: float = 1.8
var center_of_mass_offset: Vector3 = Vector3(0.0, 0.5, 0.0)

# Input manager reference (singleton)
var _input_manager: InputManager = null

# Internal state
var _is_active: bool = false
var _last_position: Vector3 = Vector3.ZERO
var _velocity: Vector3 = Vector3.ZERO
var _applied_forces: Dictionary = {}

func _ready() -> void:
	_init_references()
	_load_physics_settings()
	_connect_inputs()
	_setup_vehicle_defaults()
	_is_active = true

func _init_references() -> void:
	if Engine.has_singleton("InputManager"):
		_input_manager = Engine.get_singleton("InputManager")
	else:
		warning_text = "InputManager singleton not found!"

func _load_physics_settings() -> void:
	var settings = preload("res://scripts/core/PhysicsSettings.gd").new()
	settings.gravity = 9.81
	settings.physics_tick_rate = 120
	settings.max_substeps = 4
	settings.time_scale = 1.0
	settings.default_vehicle_mass = 1500.0
	vehicle_mass = settings.default_vehicle_mass
	
	# Load additional config if needed
	if settings.has_method("get_vehicle_config"):
		var config = settings.get_vehicle_config()
		if config.has_key("max_acceleration"):
			max_acceleration = config["max_acceleration"]
		if config.has_key("max_deceleration"):
			max_deceleration = config["max_deceleration"]
		if config.has_key("max_steering_angle"):
			max_steering_angle = config["max_steering_angle"]

func _setup_vehicle_defaults() -> void:
	# Set initial values
	current_speed = 0.0
	current_rpm = idle_rpm
	current_gear = 0  # Neutral start
	clutch_engaged = false
	
	# Clear applied forces
	_applied_forces.clear()

func _connect_inputs() -> void:
	if _input_manager != null:
		_input_manager.throttle_pressed.connect(_on_throttle_pressed)
		_input_manager.brake_pressed.connect(_on_brake_pressed)
		_input_manager.steer_left.connect(_on_steer_left)
		_input_manager.steer_right.connect(_on_steer_right)
		_input_manager.shift_up.connect(_on_shift_up)
		_input_manager.shift_down.connect(_on_shift_down)
		_input_manager.clutch_toggle.connect(_on_clutch_toggle)
		_input_manager.reverse_toggle.connect(_on_reverse_toggle)

func _process(delta: float) -> void:
	if not _is_active:
		return
	
	_update_state(delta)
	_process_input(delta)
	_apply_physics(delta)

func _physics_process(delta: float) -> void:
	if not _is_active or car_body == null:
		return
	
	_handle_collision_detection(delta)

func _update_state(delta: float) -> void:
	# Update velocity from car body
	if car_body != null:
		_velocity = car_body.linear_velocity
		
		# Calculate forward speed magnitude
		var forward_direction = get_transform().basis.z.normalized()
		current_speed = _velocity.dot(forward_direction)
		
		# Emit signals if changed significantly
		if abs(current_speed - _applied_forces.get("current_speed", 0.0)) > 0.5:
			speed_changed.emit(current_speed)
			_applied_forces["current_speed"] = current_speed

func _process_input(delta: float) -> void:
	# Get fresh input values from InputManager
	if _input_manager != null:
		_throttle_input = clamp(_input_manager.throttle_value, 0.0, 1.0)
		_brake_input = clamp(_input_manager.brake_value, 0.0, 1.0)
		_steering_input = clamp(_input_manager.steer_value, -1.0, 1.0)
		_reverse_input = _input_manager.is_reversing()
	
	# Update input signals
	throttle_input_changed.emit(_throttle_input)
	brake_input_changed.emit(_brake_input)
	steering_input_changed.emit(_steering_input)

func _apply_physics(delta: float) -> void:
	if car_body == null:
		return
	
	# Calculate torque based on gear and throttle
	var target_torque = _calculate_engine_torque()
	
	# Apply drivetrain forces through powertrain
	if powertrain != null and powertrain.has_method("apply_drive_force"):
		powertrain.apply_drive_force(target_torque, _throttle_input, current_gear)
	
	# Apply braking force
	_apply_braking(_brake_input)
	
	# Apply steering (handled by suspension/wheel controllers)
	_apply_steering(_steering_input)
	
	# Update applied forces cache
	_applied_forces["torque"] = target_torque
	_applied_forces["brake_force"] = _brake_input

func _calculate_engine_torque() -> float:
	# Simplified torque curve based on RPM
	if current_rpm <= idle_rpm:
		return 0.0
	
	var rpm_ratio = (current_rpm - idle_rpm) / (max_rpm - idle_rpm)
	
	# Peak torque at ~4000 RPM
	if rpm_ratio < 0.5:
		var peak_ratio = rpm_ratio / 0.5
		return peak_ratio * peak_ratio * 400.0  # Max 400Nm
	else:
		return 400.0 * (1.0 - (rpm_ratio - 0.5) * 0.5)  # Roll off after peak

func _apply_braking(brake_amount: float) -> void:
	if brake_amount <= 0.0:
		return
	
	if car_body == null:
		return
	
	# Apply counter-force to slow down
	var brake_force = brake_amount * max_deceleration * vehicle_mass
	var deceleration_vector = -_velocity.normalized() * brake_force
	
	car_body.apply_central_force(deceleration_vector)

func _apply_steering(steer_amount: float) -> void:
	# Steering is typically handled by wheel controllers
	# This method can be overridden for custom steering behavior
	pass

func _handle_collision_detection(delta: float) -> void:
	if not car_body:
		return
	
	var collisions = car_body.get_colliding_bodies()
	for collision in collisions:
		var collision_info = {
			"body": collision,
			"position": car_body.global_position,
			"velocity": _velocity,
			"time_stamp": Time.get_ticks_msec()
		}
		collision_detected.emit(collision_info)

func _on_throttle_pressed() -> void:
	_throttle_input += 0.1
	_throttle_input = clamp(_throttle_input, 0.0, 1.0)

func _on_brake_pressed() -> void:
	_brake_input += 0.1
	_brake_input = clamp(_brake_input, 0.0, 1.0)

func _on_steer_left() -> void:
	_steering_input -= 0.1
	_steering_input = clamp(_steering_input, -1.0, 1.0)

func _on_steer_right() -> void:
	_steering_input += 0.1
	_steering_input = clamp(_steering_input, -1.0, 1.0)

func _on_shift_up() -> void:
	if current_gear < 7:
		var old_gear = current_gear
		current_gear += 1
		gear_changed.emit(old_gear, current_gear)
		_on_gear_change()

func _on_shift_down() -> void:
	if current_gear > 0:
		var old_gear = current_gear
		current_gear -= 1
		gear_changed.emit(old_gear, current_gear)
		_on_gear_change()

func _on_gear_change() -> void:
	# Handle RPM changes during gear shift
	if current_gear > 0:
		var gear_ratio = _gear_ratios[current_gear - 1] * _final_drive_ratio
		current_rpm = current_speed * gear_ratio * 0.001 * max_rpm  # Simplified calculation
	else:
		current_rpm = idle_rpm
	
	rpm_changed.emit(current_rpm)

func _on_clutch_toggle() -> void:
	clutch_engaged = not clutch_engaged
	if not clutch_engaged:
		current_rpm = idle_rpm

func _on_reverse_toggle() -> void:
	_reverse_input = not _reverse_input

func set_gear(gear: int) -> void:
	current_gear = clamp(gear, -1, 7)
	gear_changed.emit(-1, current_gear)
	_on_gear_change()

func set_throttle(amount: float) -> void:
	_throttle_input = clamp(amount, 0.0, 1.0)

func set_brake(amount: float) -> void:
	_brake_input = clamp(amount, 0.0, 1.0)

func set_steering(amount: float) -> void:
	_steering_input = clamp(amount, -1.0, 1.0)

func reset_inputs() -> void:
	_throttle_input = 0.0
	_brake_input = 0.0
	_steering_input = 0.0

func get_current_gear() -> int:
	return current_gear

func get_current_rpm() -> float:
	return current_rpm

func get_current_speed() -> float:
	return current_speed

func get_wheels_data() -> Dictionary:
	if suspension_system == null:
		return {}
	
	return suspension_system.get_wheel_data()

func activate_traction_control(active: bool) -> void:
	traction_control_active = active

func set_wheel_grip(grip: float) -> void:
	wheel_grip = clamp(grip, 0.0, 1.0)

func get_vehicle_mass() -> float:
	return vehicle_mass

func get_max_rpm() -> float:
	return max_rpm

func set_max_rpm(rpm: float) -> void:
	max_rpm = max(rpm, idle_rpm)

func reset_vehicle_state() -> void:
	current_speed = 0.0
	current_rpm = idle_rpm
	current_gear = 0
	clutch_engaged = false
	reset_inputs()

func is_active() -> bool:
	return _is_active

func set_active(active: bool) -> void:
	_is_active = active
	if not active:
		reset_inputs()
		current_rpm = idle_rpm

func _exit_tree() -> void:
	_is_active = false

</FILE "scripts/controllers/VehicleController.gd">>