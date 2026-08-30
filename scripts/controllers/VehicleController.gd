extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulator
## Implements throttle, brake, steering inputs, wheel forces, and gear shifting logic
## Uses PhysicsSettings resource for centralized configuration
## Copyright 2026 Thalamus Racing Simulator Project

## Signals for vehicle state changes
signal speed_changed(current_speed: float, max_speed: float)
signal gear_changed(gear: int, rpm: float)
signal braking_changed(braking_force: float)
signal collision_detected(collision_data: Dictionary)

## Physics settings reference (loaded from autoload)
@onready var physics_settings = preload("res://scripts/core/PhysicsSettings.gd").new()
@onready var input_manager = $InputManager if get_node_or_null("/root/InputManager") else null
@onready var audio_manager = $AudioManager if get_node_or_null("/root/AudioManager") else null

# Vehicle properties
@export_group("Vehicle Configuration")
@export var vehicle_mass: float = 1500.0: set = _set_vehicle_mass
@export var center_of_mass_offset: Vector3 = Vector3(0, 0.5, 0): set = _set_center_of_mass_offset
@export var wheel_base: float = 2.8: set = _set_wheel_base
@export var track_width: float = 1.8: set = _set_track_width

@export_group("Performance Settings")
@export var engine_power: float = 300.0: set = _set_engine_power
@export var max_rpm: float = 7500.0: set = _set_max_rpm
@export var idle_rpm: float = 800.0: set = _set_idle_rpm
@export var clutch_disengage_rpm: float = 500.0: set = _set_clutch_disengage_rpm

@export_group("Drivetrain Settings")
@export var drivetrain_type: DrivetrainType = DrivetrainType.RWD
@export var final_drive_ratio: float = 3.5: set = _set_final_drive_ratio
@export var differential_type: DifferentialType = DifferentialType.OPEN

@export_group("Brake Settings")
@export var front_brake_bias: float = 0.6: range(0.4, 0.7): set = _set_front_brake_bias
@export var max_brake_pressure: float = 100.0: set = _set_max_brake_pressure
@export var abs_enabled: bool = true: set = _set_abs_enabled

@export_group("Aerodynamics")
@export var drag_coefficient: float = 0.32: set = _set_drag_coefficient
@export var frontal_area: float = 2.2: set = _set_frontal_area
@export var downforce_coefficient: float = 0.5: set = _set_downforce_coefficient

# Current state variables
var current_gear: int = 0  # 0 = neutral, negative = reverse
var current_rpm: float = 0.0
var current_speed: float = 0.0  # m/s
var target_gear: int = 0
var shift_progress: float = 0.0  # 0.0 to 1.0 during gear shifts
var is_shift_in_progress: bool = false
var shift_timer: float = 0.0

# Input states
var throttle_input: float = 0.0
var brake_input: float = 0.0
var steering_input: float = 0.0
var handbrake_input: float = 0.0

# Wheel data
var wheel_radii: Array[float] = []
var wheel_positions: Array[Vector3] = []
var wheel_forces: Array[float] = []

# Derived values
var aerodynamic_drag: float = 0.0
var aerodynamic_downforce: float = 0.0
var total_traction_force: float = 0.0
var total_brake_force: float = 0.0

# Drivetrain ratios per gear
var gear_ratios: Array[float] = [0.0, 3.8, 2.9, 2.2, 1.7, 1.3, 1.0]
var gear_reverses: Array[int] = [0, 0, 0, 0, 0, 0, 0]  # Reverse gear index

# Physical constants
const AIR_DENSITY: float = 1.225  # kg/m^3
const GRAVITY_FORCE: float = physics_settings.gravity * vehicle_mass
const MIN_GEAR_SHIFT_TIME: float = 0.15  # seconds
const MAX_GEAR_SHIFT_TIME: float = 0.35  # seconds

# Engine torque curve (simplified lookup)
var _torque_curve: Dictionary = {}

enum DrivetrainType {
	FWD,
	RWD,
	AWD
}

enum DifferentialType {
	OPEN,
	LOCKED,
	LSD
}

enum WheelPosition {
	FRONT_LEFT,
	FRONT_RIGHT,
	REAR_LEFT,
	REAR_RIGHT
}

func _init() -> void:
	_init_torque_curve()
	wheel_radii = [0.32, 0.32, 0.32, 0.32]
	wheel_positions = [
		Vector3(-track_width/2, 0, wheel_base/2),
		Vector3(track_width/2, 0, wheel_base/2),
		Vector3(-track_width/2, 0, -wheel_base/2),
		Vector3(track_width/2, 0, -wheel_base/2)
	]
	wheel_forces = [0.0, 0.0, 0.0, 0.0]

func _ready() -> void:
	set_physics_process(true)
	process_mode = ProcessModeEnum.ALWAYS
	
	# Initialize derived values
	vehicle_mass = _calculate_effective_mass()
	
	# Load gear ratios from project settings if available
	_load_gear_ratios()
	
	print("[VehicleController] Ready - ", get_path())

func _process(delta: float) -> void:
	_update_inputs(delta)
	_update_gear_system(delta)
	_update_aerodynamics(delta)

func _physics_process(delta: float) -> void:
	_apply_physics(delta)
	_apply_forces(delta)
	_update_velocity(delta)

func _update_inputs(delta: float) -> void:
	"""Read and process input values from InputManager"""
	if input_manager != null:
		throttle_input = clamp(input_manager.get_axis("throttle"), 0.0, 1.0)
		brake_input = clamp(input_manager.get_axis("brake"), 0.0, 1.0)
		steering_input = clamp(input_manager.get_axis("steering"), -1.0, 1.0)
		handbrake_input = clamp(input_manager.get_axis("handbrake"), 0.0, 1.0)
	else:
		# Fallback default inputs
		pass

func _update_gear_system(delta: float) -> void:
	"""Handle automatic or manual gear shifting logic"""
	
	if not is_shift_in_progress:
		_determine_target_gear()
		
		if current_gear != target_gear:
			_start_gear_shift()
	
	if is_shift_in_progress:
		_handle_gear_shift(delta)

func _determine_target_gear() -> void:
	"""Calculate desired gear based on RPM and throttle"""
	var target: int = current_gear
	
	if throttle_input > 0.1:
		# Accelerating - upshift when approaching redline
		if current_rpm > max_rpm * 0.95 and current_gear < gear_ratios.size() - 1:
			target = current_gear + 1
		elif current_rpm < idle_rpm * 1.5 and current_gear > 0:
			target = current_gear - 1
	else:
		# Decelerating - downshift
		if current_rpm < idle_rpm * 0.8 and current_gear > 1:
			target = max(1, current_gear - 1)
		elif current_gear == 1 and current_rpm < idle_rpm:
			target = 0  # Neutral when stopped
	
	target_gear = target

func _start_gear_shift() -> void:
	"""Initiate a gear shift sequence"""
	is_shift_in_progress = true
	shift_timer = 0.0
	shift_progress = 0.0
	
	# Trigger sound effect
	_play_sound("gear_shift")

func _handle_gear_shift(delta: float) -> void:
	"""Process ongoing gear shift"""
	shift_timer += delta
	
	# Progress through shift
	shift_progress = min(shift_timer / MIN_GEAR_SHIFT_TIME, 1.0)
	
	if shift_progress >= 1.0:
		current_gear = target_gear
		is_shift_in_progress = false
		shift_timer = 0.0
		shift_progress = 0.0
		
		emit_signal("gear_changed", current_gear, current_rpm)

func _apply_physics(delta: float) -> void:
	"""Apply fundamental vehicle physics calculations"""
	_update_engine_rpm(delta)
	_update_wheel_slip()
	_apply_gravity(delta)

func _update_engine_rpm(delta: float) -> void:
	"""Calculate engine RPM based on gear and vehicle speed"""
	if current_gear == 0:
		# Neutral - engine idles
		current_rpm = lerp(current_rpm, idle_rpm, delta * 5.0)
		return
	
	var gear_ratio: float = gear_ratios[current_gear]
	var wheel_radius: float = wheel_radii[WheelPosition.REAR_LEFT]
	
	# Convert vehicle speed to wheel angular velocity
	var wheel_angular_velocity: float = current_speed / wheel_radius
	
	# Calculate engine RPM from wheel speed
	var calculated_rpm: float = wheel_angular_velocity * gear_ratio * final_drive_ratio * (60.0 / (2.0 * PI))
	
	# Apply throttle influence
	var throttle_factor: float = throttle_input * 0.3
	
	if throttle_input > 0.0:
		current_rpm = lerp(current_rpm, max(calculated_rpm, idle_rpm), delta * 15.0 + throttle_factor * 10.0)
	else:
		current_rpm = lerp(current_rpm, calculated_rpm, delta * 8.0)
	
	current_rpm = clamp(current_rpm, idle_rpm, max_rpm)

func _update_wheel_slip() -> void:
	"""Calculate wheel slip ratio for traction control"""
	pass  # Placeholder for advanced traction control

func _apply_gravity(delta: float) -> void:
	"""Apply gravitational force to vehicle"""
	var gravity_force_vector: Vector3 = Vector3.UP * GRAVITY_FORCE
	add_force(gravity_force_vector)

func _apply_forces(delta: float) -> void:
	"""Apply drive, brake, and steering forces"""
	_calculate_traction_force()
	_calculate_brake_force()
	_apply_aerodynamic_forces()
	_apply_steering_effects()

func _calculate_traction_force() -> void:
	"""Calculate total traction force based on throttle and gear"""
	if current_gear == 0:
		total_traction_force = 0.0
		return
	
	var gear_ratio: float = gear_ratios[current_gear]
	var wheel_radius: float = wheel_radii[WheelPosition.REAR_LEFT]
	
	# Get torque from engine curve
	var torque: float = _get_engine_torque(current_rpm)
	
	# Calculate force at wheels
	var wheel_torque: float = torque * gear_ratio * final_drive_ratio
	var traction_force: float = wheel_torque / wheel_radius
	
	# Apply drivetrain distribution
	match drivetrain_type:
		DrivetrainType.FWD:
			total_traction_force = traction_force * 0.6
			wheel_forces[WheelPosition.FRONT_LEFT] = traction_force * 0.3
			wheel_forces[WheelPosition.FRONT_RIGHT] = traction_force * 0.3
		DrivetrainType.AWD:
			total_traction_force = traction_force
			wheel_forces[WheelPosition.FRONT_LEFT] = traction_force * 0.4
			wheel_forces[WheelPosition.FRONT_RIGHT] = traction_force * 0.4
			wheel_forces[WheelPosition.REAR_LEFT] = traction_force * 0.1
			wheel_forces[WheelPosition.REAR_RIGHT] = traction_force * 0.1
		DrivetrainType.RWD:
			_, fallthrough
	wheel_forces[WheelPosition.REAR_LEFT] = traction_force * 0.5
	wheel_forces[WheelPosition.REAR_RIGHT] = traction_force * 0.5

func _calculate_brake_force() -> void:
	"""Calculate brake force distribution"""
	if brake_input <= 0.0 and handbrake_input <= 0.0:
		total_brake_force = 0.0
		return
	
	var brake_pressure: float = brake_input * max_brake_pressure
	var handbrake_pressure: float = handbrake_input * max_brake_pressure * 0.8
	
	var total_pressure: float = brake_pressure + handbrake_pressure
	
	# Distribute brake force front/rear
	var front_force: float = total_pressure * front_brake_bias
	var rear_force: float = total_pressure * (1.0 - front_brake_bias)
	
	# Handbrake affects only rear wheels
	rear_force += handbrake_pressure * 0.5
	
	total_brake_force = front_force + rear_force
	
	# Apply to individual wheels
	var front_wheel_brake: float = front_force * 0.5
	var rear_wheel_brake: float = rear_force * 0.5
	
	wheel_forces[WheelPosition.FRONT_LEFT] -= front_wheel_brake
	wheel_forces[WheelPosition.FRONT_RIGHT] -= front_wheel_brake
	wheel_forces[WheelPosition.REAR_LEFT] -= rear_wheel_brake
	wheel_forces[WheelPosition.REAR_RIGHT] -= rear_wheel_brake

func _apply_aerodynamic_forces() -> void:
	"""Apply aerodynamic drag and downforce"""
	aerodynamic_drag = 0.5 * AIR_DENSITY * drag_coefficient * frontal_area * current_speed * current_speed
	aerodynamic_downforce = 0.5 * AIR_DENSITY * downforce_coefficient * frontal_area * current_speed * current_speed
	
	# Apply drag opposite to velocity direction
	var velocity_direction: Vector3 = global_position.direction_to(global_position + velocity)
	if velocity_direction != Vector3.ZERO:
		add_force(-velocity_direction.normalized() * aerodynamic_drag)
	
	# Apply downforce vertically downward
	add_force(Vector3.DOWN * aerodynamic_downforce)

func _apply_steering_effects() -> void:
	"""Apply steering to rotate vehicle"""
	var steer_angle: float = steering_input * 0.5  # Max 0.5 radians
	
	# Apply rotation to body
	rotate_y(-steer_angle)

func _update_velocity(delta: float) -> void:
	"""Update vehicle velocity from physics movement"""
	current_speed = velocity.length()
	
	if abs(current_speed - _previous_speed) > 0.5:
		emit_signal("speed_changed", current_speed, max_speed)
	
	_previous_speed = current_speed

func _get_engine_torque(rpm: float) -> float:
	"""Lookup torque value from engine curve"""
	if _torque_curve.is_empty():
		return 0.0
	
	var keys: Array[float] = _torque_curve.keys()
	var sorted_keys: Array[float] = keys.duplicate()
	sorted_keys.sort()
	
	# Find appropriate torque value
	for i in range(sorted_keys.size() - 1):
		if rpm >= sorted_keys[i] and rpm <= sorted_keys[i + 1]:
			var t1: float = _torque_curve[sorted_keys[i]]
			var t2: float = _torque_curve[sorted_keys[i + 1]]
			var ratio: float = (rpm - sorted_keys[i]) / (sorted_keys[i + 1] - sorted_keys[i])
			return t1 + (t2 - t1) * ratio
	
	return _torque_curve.values()[0]

func _init_torque_curve() -> void:
	"""Initialize simplified engine torque curve"""
	_torque_curve = {
		idle_rpm: 100.0,
		idle_rpm * 2.0: 280.0,
		max_rpm * 0.5: 350.0,
		max_rpm * 0.8: 380.0,
		max_rpm * 0.9: 360.0,
		max_rpm: 320.0
	}

func _load_gear_ratios() -> void:
	"""Load gear ratios from project configuration"""
	# Default 6-speed gearbox ratios
	gear_ratios = [0.0, 3.8, 2.9, 2.2, 1.7, 1.3, 1.0]
	gear_reverses = [0, 0, 0, 0, 0, 0, 0]

func _play_sound(sound_name: String) -> void:
	"""Play vehicle-related sound effects"""
	if audio_manager != null:
		audio_manager.play_sound(sound_name)

func _set_vehicle_mass(value: float) -> void:
	vehicle_mass = value
	GRAVITY_FORCE = physics_settings.gravity * vehicle_mass

func _set_center_of_mass_offset(value: Vector3) -> void:
	center_of_mass_offset = value

func _set_wheel_base(value: float) -> void:
	wheel_base = value

func _set_track_width(value: float) -> void:
	track_width = value

func _set_engine_power(value: float) -> void:
	engine_power = value

func _set_max_rpm(value: float) -> void:
	max_rpm = value

func _set_idle_rpm(value: float) -> void:
	idle_rpm = value

func _set_clutch_disengage_rpm(value: float) -> void:
	clutch_disengage_rpm = value

func _set_final_drive_ratio(value: float) -> void:
	final_drive_ratio = value

func _set_front_brake_bias(value: float) -> void:
	front_brake_bias = clamp(value, 0.4, 0.7)

func _set_max_brake_pressure(value: float) -> void:
	max_brake_pressure = value

func _set_abs_enabled(value: bool) -> void:
	abs_enabled = value

func _set_drag_coefficient(value: float) -> void:
	drag_coefficient = value

func _set_frontal_area(value: float) -> void:
	frontal_area = value

func _set_downforce_coefficient(value: float) -> void:
	downforce_coefficient = value

func _calculate_effective_mass() -> float:
	"""Calculate effective mass considering fuel load and driver"""
	var base_mass: float = physics_settings.default_vehicle_mass
	var fuel_mass: float = 60.0  # ~60L fuel tank
	var driver_mass: float = 80.0
	
	return base_mass + fuel_mass + driver_mass

func reset_vehicle() -> void:
	"""Reset vehicle to initial state"""
	current_gear = 0
	current_rpm = idle_rpm
	current_speed = 0.0
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	handbrake_input = 0.0
	velocity = Vector3.ZERO
	is_shift_in_progress = false
	shift_progress = 0.0

func force_gear(gear: int) -> void:
	"""Force change to specific gear"""
	if gear >= 0 and gear < gear_ratios.size():
		target_gear = gear
		if current_gear != gear:
			_start_gear_shift()

func emergency_stop() -> void:
	"""Initiate emergency braking"""
	brake_input = 1.0
	handbrake_input = 1.0

func _notification(what: int) -> void:
	match what:
		NOTIFICATION_PREDELETE:
			if physics_settings:
				physics_settings.free()

</File>