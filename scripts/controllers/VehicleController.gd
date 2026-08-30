extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulator
## Implements throttle, brake, steering inputs, wheel forces, and gear shifting logic
## Uses PhysicsSettings constants for centralized tuning
## Copyright 2026 Thalamus Racing Simulator Project

signal speed_changed(new_speed: float)
signal gear_changed(old_gear: int, new_gear: int)
signal vehicle_state_changed(state: VehicleState)
signal drift_started()
signal drift_ended()
signal crash_detected(impact_force: Vector3)

enum VehicleState {
	IDLE,
	ACCELERATING,
	BRAKING,
	DRIFTING,
	JUMPING,
	CRASHED
}

# ============================================================================
# PHYSICS SETTINGS REFERENCES (autoloaded singleton)
# ============================================================================
var _physics_settings: PhysicsSettings = null

# ============================================================================
# VEHICLE PHYSICS PROPERTIES
# ============================================================================
@export_group("Vehicle Mass & Weight")
@export var mass: float = 1500.0: set = _set_mass
@export var center_of_mass_offset: Vector3 = Vector3(0.0, 0.3, 0.0): set = _set_center_of_mass_offset
@export var weight_distribution_front: float = 0.45: set = _set_weight_distribution_front
@export var weight_distribution_rear: float = 0.55: set = _set_weight_distribution_rear

@export_group("Powertrain")
@export var engine_max_rpm: float = 8000.0: set = _set_engine_max_rpm
@export var engine_min_rpm: float = 800.0: set = _set_engine_min_rpm
@export var max_power: float = 300000.0: set = _set_max_power
@export var max_torque: float = 500.0: set = _set_max_torque
@export var torque_curve: Array[Vector2] = []
@export var power_curve: Array[Vector2] = []

@export_group("Gearbox")
@export var transmission_type: String = "manual"
@export var num_gears: int = 6: set = _set_num_gears
@export var final_drive_ratio: float = 3.73: set = _set_final_drive_ratio
@export var gear_ratios: Array[float] = [3.5, 2.0, 1.4, 1.0, 0.8, 0.6]
@export var clutch_engagement_threshold: float = 0.95

@export_group("Wheels & Suspension")
@export var wheel_radius: float = 0.33: set = _set_wheel_radius
@export var wheel_track_width: float = 1.6: set = _set_wheel_track_width
@export var suspension_travel: float = 0.15: set = _set_suspension_travel
@export var suspension_stiffness: float = 50000.0: set = _set_suspension_stiffness
@export var suspension_damping: float = 5000.0: set = _set_suspension_damping
@export var wheel_friction_coefficient: float = 1.2: set = _set_wheel_friction_coefficient

@export_group("Aerodynamics")
@export var drag_coefficient: float = 0.30: set = _set_drag_coefficient
@export var frontal_area: float = 2.2: set = _set_frontal_area
@export var downforce_coefficient: float = 0.5: set = _set_downforce_coefficient
@export var wing_angle: float = 5.0: set = _set_wing_angle

@export_group("Braking System")
@export var brake_pressure: float = 1.0: set = _set_brake_pressure
@export var brake_force_per_wheel: float = 4000.0: set = _set_brake_force_per_wheel
@export var abs_enabled: bool = true
@export var brake_bias_front: float = 0.6: set = _set_brake_bias_front

@export_group("Traction Control")
@export var traction_control_enabled: bool = true
@export var slip_threshold: float = 0.15: set = _set_slip_threshold
@export var slip_correction_factor: float = 0.5: set = _set_slip_correction_factor

# ============================================================================
# INPUT VARIABLES
# ============================================================================
var throttle_input: float = 0.0: set = _set_throttle_input
var brake_input: float = 0.0: set = _set_brake_input
var steering_input: float = 0.0: set = _set_steering_input
var clutch_input: float = 0.0
var handbrake_input: bool = false

# ============================================================================
# CURRENT STATE VARIABLES
# ============================================================================
var current_gear: int = 0
var target_gear: int = 0
var engine_rpm: float = 0.0
var wheel_speed: float = 0.0
var vehicle_speed: float = 0.0
var lateral_velocity: float = 0.0
var vertical_velocity: float = 0.0
var drift_angle: float = 0.0
var slip_angle: float = 0.0
var grip_level: float = 1.0

var _rigid_body_3d: RigidBody3D = null
var _vehicle_state: VehicleState = VehicleState.IDLE
var _is_moving: bool = false
var _is_jumping: bool = false
var _jump_impulse_applied: bool = false

# ============================================================================
# WHEEL DATA STRUCTURES
# ============================================================================
class WheelData:
	var index: int
	var position_local: Vector3
	var position_world: Vector3
	var normal: Vector3
	var force: float = 0.0
	var steering_angle: float = 0.0
	var drive_force: float = 0.0
	var brake_force: float = 0.0
	var angular_velocity: float = 0.0
	var slip_ratio: float = 0.0
	var normal_force: float = 0.0
	var friction_coefficient: float = 1.0

	var wheels: Array[WheelData] = []

# ============================================================================
# GAME SIGNALS CONNECTION
# ============================================================================
func _ready() -> void:
	_init_physics_settings()
	_connect_signals()
	_init_rigid_body()
	_init_wheels()
	_reset_vehicle_state()

func _init_physics_settings() -> void:
	if Engine.has_singleton("PhysicsSettings"):
		_physics_settings = Engine.get_singleton("PhysicsSettings")
	else:
		_physics_settings = preload("res://scripts/core/PhysicsSettings.gd").new()
	
	mass = _physics_settings.default_vehicle_mass
	engine_max_rpm = _physics_settings.engine_max_rpm
	max_power = _physics_settings.max_power
	max_torque = _physics_settings.max_torque
	wheel_radius = _physics_settings.wheel_radius
	suspension_travel = _physics_settings.suspension_travel
	drag_coefficient = _physics_settings.drag_coefficient
	frontal_area = _physics_settings.frontal_area

func _connect_signals() -> void:
	GameManager.race_started.connect(_on_race_started)
	GameManager.race_ended.connect(_on_race_ended)
	GameManager.vehicle_spawned.connect(_on_vehicle_spawned)
	InputManager.input_throttle.connect(_on_input_throttle)
	InputManager.input_brake.connect(_on_input_brake)
	InputManager.input_steering.connect(_on_input_steering)
	InputManager.input_clutch.connect(_on_input_clutch)
	InputManager.input_handbrake.connect(_on_input_handbrake)
	InputManager.input_shift_up.connect(_on_input_shift_up)
	InputManager.input_shift_down.connect(_on_input_shift_down)
	InputManager.input_jump.connect(_on_input_jump)
	InputManager.input_restart.connect(_on_input_restart)

func _init_rigid_body() -> void:
	_rigid_body_3d = get_parent()
	if _rigid_body_3d is RigidBody3D:
		_rigid_body_3d.mass = mass
		_rigid_body_3d.center_of_mass = center_of_mass_offset
		_rigid_body_3d.collision_layer = 1 << 2  # Vehicle layer
		_rigid_body_3d.collision_mask = (1 << 3) | (1 << 4)  # Track and obstacles

func _init_wheels() -> void:
	# Initialize wheel data structures
	for i in range(4):
		var wheel_data := WheelData.new()
		wheel_data.index = i
		
		match i:
			0: # Front Left
				wheel_data.position_local = Vector3(-wheel_track_width / 2, 0.0, wheel_track_width / 2)
			1: # Front Right
				wheel_data.position_local = Vector3(wheel_track_width / 2, 0.0, wheel_track_width / 2)
			2: # Rear Left
				wheel_data.position_local = Vector3(-wheel_track_width / 2, 0.0, -wheel_track_width / 2)
			3: # Rear Right
				wheel_data.position_local = Vector3(wheel_track_width / 2, 0.0, -wheel_track_width / 2)
		
		wheels.append(wheel_data)

func _reset_vehicle_state() -> void:
	current_gear = 0
	target_gear = 0
	engine_rpm = 0.0
	wheel_speed = 0.0
	vehicle_speed = 0.0
	lateral_velocity = 0.0
	vertical_velocity = 0.0
	drift_angle = 0.0
	slippage = 0.0
	grip_level = 1.0
	_is_moving = false
	_is_jumping = false
	_jump_impulse_applied = false
	set_vehicle_state(VehicleState.IDLE)

# ============================================================================
# MAIN PROCESS LOOP
# ============================================================================
func _physics_process(delta: float) -> void:
	_update_physics(delta)
	_apply_forces(delta)
	_update_visuals(delta)

func _update_physics(delta: float) -> void:
	_get_current_inputs()
	_calculate_engine_output()
	_update_gearbox()
	_update_wheels(delta)
	_apply_aerodynamics(delta)
	_check_drift()
	_update_vehicle_state()

func _get_current_inputs() -> void:
	throttle_input = InputManager.get_throttle()
	brake_input = InputManager.get_brake()
	steering_input = InputManager.get_steering()
	clutch_input = InputManager.get_clutch()
	handbrake_input = InputManager.get_handbrake()

func _calculate_engine_output() -> void:
	var torque_multiplier: float = 1.0
	
	if transmission_type == "manual":
		if abs(current_gear) <= num_gears:
			torque_multiplier = gear_ratios[current_gear - 1] if current_gear > 0 else 1.0
		else:
			torque_multiplier = 1.0
	
	var rpm_ratio: float = clamp((engine_rpm - engine_min_rpm) / (engine_max_rpm - engine_min_rpm), 0.0, 1.0)
	
	var engine_torque: float = max_torque * torque_multiplier * rpm_ratio
	var engine_power: float = engine_torque * engine_rpm / 9549.0
	
	_engine_output = {
		"torque": engine_torque,
		"power": min(engine_power, max_power),
		"rpm": engine_rpm,
		"throttle": throttle_input
	}

func _update_gearbox() -> void:
	var wheel_rpm: float = wheel_speed / (2.0 * PI * wheel_radius)
	var gear_rpm: float = wheel_rpm * final_drive_ratio * (gear_ratios[current_gear - 1] if current_gear > 0 else 1.0)
	
	if clutch_input >= clutch_engagement_threshold:
		engine_rpm = lerp(engine_rpm, gear_rpm, delta * 10.0)
	else:
		engine_rpm = lerp(engine_rpm, engine_rpm * 0.98, delta * 5.0)
	
	if _should_shift():
		perform_gear_shift(target_gear)

func _should_shift() -> void:
	if current_gear < num_gears and engine_rpm >= engine_max_rpm * 0.95:
		target_gear = current_gear + 1
		return true
	elif current_gear > 0 and engine_rpm <= engine_min_rpm * 1.1:
		target_gear = current_gear - 1
		return true
	return false

func perform_gear_shift(new_gear: int) -> void:
	if new_gear < 0 or new_gear > num_gears + 1:
		return
	
	var old_gear = current_gear
	current_gear = new_gear
	
	gear_changed.emit(old_gear, current_gear)
	
	if current_gear == 0:
		_release_clutch()
	else:
		engage_gear(new_gear)

func _release_clutch() -> void:
	pass

func _engage_gear(gear: int) -> void:
	pass

func _update_wheels(delta: float) -> void:
	var drive_force: float = _calculate_drive_force()
	var brake_force: float = _calculate_brake_force()
	
	for wheel in wheels:
		_update_wheel_forces(wheel, drive_force, brake_force, delta)
		_update_wheel_slip(wheel, delta)
		_apply_wheel_physics(wheel, delta)

func _calculate_drive_force() -> float:
	if current_gear == 0:
		return 0.0
	
	var gear_ratio: float = gear_ratios[current_gear - 1] if current_gear > 0 else 1.0
	var total_ratio: float = gear_ratio * final_drive_ratio
	
	var force: float = _engine_output.torque * total_ratio * 0.1
	force *= throttle_input
	
	if traction_control_enabled:
		force = _apply_traction_control(force)
	
	return force

func _apply_traction_control(base_force: float) -> float:
	var slip_total: float = 0.0
	for wheel in wheels:
		slip_total += abs(wheel.slip_ratio)
	
	var avg_slip: float = slip_total / wheels.size()
	
	if avg_slip > slip_threshold:
		var correction: float = 1.0 - ((avg_slip - slip_threshold) * slip_correction_factor)
		return base_force * clamp(correction, 0.0, 1.0)
	
	return base_force

func _calculate_brake_force() -> float:
	if brake_input <= 0.0:
		return 0.0
	
	var total_brake_force: float = brake_force_per_wheel * brake_input * brake_pressure
	var front_brake: float = total_brake_force * brake_bias_front
	var rear_brake: float = total_brake_force * (1.0 - brake_bias_front)
	
	if abs:
		var wheel_slips: float = 0.0
		for wheel in wheels:
			wheel_slips += abs(wheel.slip_ratio)
		
		if wheel_slips > slip_threshold:
			total_brake_force *= 0.5
	
	return total_brake_force

func _update_wheel_forces(wheel: WheelData, drive_force: float, brake_force: float, delta: float) -> void:
	if current_gear == 0:
		wheel.drive_force = 0.0
	else:
		var wheel_index: int = wheel.index
		if wheel_index < 2: # Front wheels don't drive
			wheel.drive_force = 0.0
		else: # Rear wheels drive
			wheel.drive_force = drive_force / 2.0
	
	wheel.brake_force = brake_force / 4.0

func _update_wheel_slip(wheel: WheelData, delta: float) -> void:
	var linear_speed: float = vehicle_speed
	var wheel_circumference: float = 2.0 * PI * wheel_radius
	var ideal_wheel_rpm: float = linear_speed / wheel_circumference
	var actual_wheel_rpm: float = wheel.angular_velocity
	
	wheel.slip_ratio = (actual_wheel_rpm - ideal_wheel_rpm) / max(actual_wheel_rpm, 0.001)

func _apply_wheel_physics(wheel: WheelData, delta: float) -> void:
	var traction: float = wheel.friction_coefficient * wheel.normal_force
	var applied_force: float = wheel.drive_force - wheel.brake_force
	
	applied_force = clamp(applied_force, -traction, traction)
	
	wheel.force = applied_force

func _apply_aerodynamics(delta: float) -> void:
	var air_density: float = 1.225
	var velocity_squared: float = vehicle_speed * vehicle_speed
	
	var drag_force: float = 0.5 * drag_coefficient * frontal_area * air_density * velocity_squared
	var downforce: float = 0.5 * downforce_coefficient * frontal_area * air_density * velocity_squared
	
	var aerodynamic_acceleration: Vector3 = -drag_force * global_velocity.normalized()
	_rigid_body_3d.apply_central_impulse(aerodynamic_acceleration * delta * mass)

func _check_drift() -> void:
	var lateral_speed: float = abs(lateral_velocity)
	var forward_speed: float = abs(vehicle_speed)
	
	if forward_speed > 5.0 and lateral_speed > 2.0:
		drift_angle = atan2(lateral_speed, forward_speed)
		
		if not _is_drifting and drift_angle > 0.1:
			_is_drifting = true
			emit_signal(drift_started)
	elif drift_angle > 0.1:
		drift_angle = lerp(drift_angle, 0.0, delta * 2.0)
	else:
		_is_drifting = false
		if _was_drifting:
			_was_drifting = false
			emit_signal(drift_ended)

func _update_vehicle_state() -> void:
	if _is_crashed:
		set_vehicle_state(VehicleState.CRASHED)
	elif _is_jumping:
		set_vehicle_state(VehicleState.JUMPING)
	elif handbrake_input or drift_angle > 0.3:
		set_vehicle_state(VehicleState.DRIFTING)
	elif brake_input > 0.1:
		set_vehicle_state(VehicleState.BRAKING)
	elif throttle_input > 0.1:
		set_vehicle_state(VehicleState.ACCELERATING)
	else:
		set_vehicle_state(VehicleState.IDLE)

func _apply_forces(delta: float) -> void:
	_apply_engine_force()
	_apply_brake_forces()
	_apply_steer_force()
	_apply_gravity_and_collision()

func _apply_engine_force() -> void:
	if current_gear == 0:
		return
	
	var drive_force: float = _calculate_drive_force()
	
	for i, wheel in enumerate(wheels):
		if i >= 2: # Only rear wheels drive
			var wheel_position: Vector3 = wheel.position_world
			var force_direction: Vector3 = global_transform.basis.z
			
			_rigid_body_3d.apply_impulse_at_location(
				force_direction * drive_force * delta,
				wheel_position
			)

func _apply_brake_forces() -> void:
	if brake_input <= 0.0:
		return
	
	var brake_force: float = _calculate_brake_force() / wheels.size()
	
	for wheel in wheels:
		var wheel_position: Vector3 = wheel.position_world
		var force_direction: Vector3 = -global_transform.basis.z
		
		_rigid_body_3d.apply_impulse_at_location(
			force_direction * brake_force * delta,
			wheel_position
		)

func _apply_steer_force() -> void:
	if abs(steering_input) < 0.01:
		return
	
	var steer_angle: float = steering_input * 30.0 * PI / 180.0
	
	if current_gear != 0:
		var wheel_position_fl: Vector3 = wheels[0].position_world
		var wheel_position_fr: Vector3 = wheels[1].position_world
		
		var steering_force: Vector3 = global_transform.basis.y.rotated(global_transform.basis.z, steer_angle)
		
		_rigid_body_3d.apply_central_impulse(steering_force * delta * 0.5)

func _apply_gravity_and_collision() -> void:
	var gravity_vector: Vector3 = -_physics_settings.gravity * Vector3.UP
	_rigid_body_3d.apply_central_force(gravity_vector * mass)

func _update_visuals(delta: float) -> void:
	_update_wheel_rotation()
	_update_camera_follow()
	_update_particle_effects()

func _update_wheel_rotation() -> void:
	for wheel in wheels:
		wheel.rotation.z -= wheel.angular_velocity * delta

func _update_camera_follow() -> void:
	pass

func _update_particle_effects() -> void:
	pass

# ============================================================================
# JUMP & DRIFT MECHANICS
# ============================================================================
func _on_input_jump() -> void:
	if _is_jumping or _is_crashed:
		return
	
	if vertical_velocity <= 0.1:
		_is_jumping = true
		_jump_impulse_applied = true
		
		var jump_impulse: Vector3 = Vector3.UP * 15.0
		_rigid_body_3d.apply_central_impulse(jump_impulse)

func _on_input_restart() -> void:
	_reset_vehicle_state()
	global_position = Vector3.ZERO
	global_rotation = Vector3.ZERO
	_rigid_body_3d.linear_velocity = Vector3.ZERO
	_rigid_body_3d.angular_velocity = Vector3.ZERO

# ============================================================================
# COLLISION & CRASH DETECTION
# ============================================================================
func _on_collision(body: Node, collision_shape: CollisionShape3D, local_pos: Vector3, normal: Vector3, contact_count: int) -> void:
	var impact_force: Vector3 = _rigid_body_3d.linear_velocity * mass
	
	if impact_force.length() > 50.0:
		_is_crashed = true
		crash_detected.emit(impact_force)
		set_vehicle_state(VehicleState.CRASHED)

func _on_collision_entered(collision: CollisionObject3D) -> void:
	pass

func _on_collision_exited(collision: CollisionObject3D) -> void:
	pass

# ============================================================================
# RACE EVENT HANDLERS
# ============================================================================
func _on_race_started(race_data: Dictionary) -> void:
	_reset_vehicle_state()
	current_gear = 1
	_is_crashed = false

func _on_race_ended(results: Dictionary) -> void:
	stop_race_timer()
	_reset_vehicle_state()

func _on_vehicle_spawned(vehicle: Node) -> void:
	if vehicle == self:
		_reset_vehicle_state()

func _on_input_throttle(value: float) -> void:
	throttle_input = value

func _on_input_brake(value: float) -> void:
	brake_input = value

func _on_input_steering(value: float) -> void:
	steering_input = value

func _on_input_clutch(value: float) -> void:
	clutch_input = value

func _on_input_handbrake(pressed: bool) -> void:
	handbrake_input = pressed

func _on_input_shift_up() -> void:
	if current_gear < num_gears:
		target_gear = current_gear + 1

func _on_input_shift_down() -> void:
	if current_gear > 1:
		target_gear = current_gear - 1

# ============================================================================
# GETTER & SETTER METHODS
# ============================================================================
func get_current_gear() -> int:
	return current_gear

func get_engine_rpm() -> float:
	return engine_rpm

func get_vehicle_speed() -> float:
	return vehicle_speed

func get_vehicle_state() -> VehicleState:
	return _vehicle_state

func get_wheel_data(index: int) -> WheelData:
	if index >= 0 and index < wheels.size():
		return wheels[index]
	return null

func get_all_wheel_data() -> Array[WheelData]:
	return wheels

func get_physics_settings() -> PhysicsSettings:
	return _physics_settings

func update_physics_settings(settings: PhysicsSettings) -> void:
	if settings:
		mass = settings.default_vehicle_mass
		max_power = settings.max_power
		max_torque = settings.max_torque
		engine_max_rpm = settings.engine_max_rpm
		wheel_radius = settings.wheel_radius
		suspension_travel = settings.suspension_travel
		drag_coefficient = settings.drag_coefficient
		frontal_area = settings.frontal_area

# ============================================================================
# PROPERTY SETTERS WITH VALIDATION
# ============================================================================
func _set_mass(value: float) -> void:
	mass = max(value, 500.0)
	if _rigid_body_3d:
		_rigid_body_3d.mass = mass

func _set_center_of_mass_offset(value: Vector3) -> void:
	center_of_mass_offset = value
	if _rigid_body_3d:
		_rigid_body_3d.center_of_mass = value

func _set_weight_distribution_front(value: float) -> void:
	weight_distribution_front = clamp(value, 0.0, 1.0)
	weight_distribution_rear = 1.0 - weight_distribution_front

func _set_engine_max_rpm(value: float) -> void:
	engine_max_rpm = max(value, 2000.0)

func _set_engine_min_rpm(value: float) -> void:
	engine_min_rpm = clamp(value, 500.0, engine_max_rpm * 0.8)

func _set_max_power(value: float) -> void:
	max_power = max(value, 10000.0)

func _set_max_torque(value: float) -> void:
	max_torque = max(value, 100.0)

func _set_num_gears(value: int) -> void:
	num_gears = clamp(value, 4, 10)

func _set_final_drive_ratio(value: float) -> void:
	final_drive_ratio = max(value, 2.0)

func _set_wheel_radius(value: float) -> void:
	wheel_radius = max(value, 0.2)

func _set_suspension_travel(value: float) -> void:
	suspension_travel = max(value, 0.05)

func _set_suspension_stiffness(value: float) -> void:
	suspension_stiffness = max(value, 1000.0)

func _set_suspension_damping(value: float) -> void:
	suspension_damping = max(value, 100.0)

func _set_wheel_friction_coefficient(value: float) -> void:
	wheel_friction_coefficient = clamp(value, 0.5, 2.0)

func _set_drag_coefficient(value: float) -> void:
	drag_coefficient = clamp(value, 0.1, 1.0)

func _set_frontal_area(value: float) -> void:
	frontal_area = max(value, 1.0)

func _set_downforce_coefficient(value: float) -> void:
	downforce_coefficient = max(value, 0.0)

func _set_wing_angle(value: float) -> void:
	wing_angle = clamp(value, -10.0, 20.0)

func _set_brake_pressure(value: float) -> void:
	brake_pressure = clamp(value, 0.0, 2.0)

func _set_brake_force_per_wheel(value: float) -> void:
	brake_force_per_wheel = max(value, 1000.0)

func _set_brake_bias_front(value: float) -> void:
	brake_bias_front = clamp(value, 0.3, 0.7)

func _set_slip_threshold(value: float) -> void:
	slip_threshold = clamp(value, 0.05, 0.3)

func _set_slip_correction_factor(value: float) -> void:
	slip_correction_factor = clamp(value, 0.1, 1.0)

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================
func reset_vehicle() -> void:
	_reset_vehicle_state()
	global_position = Vector3.ZERO
	global_rotation = Vector3.ZERO
	_rigid_body_3d.linear_velocity = Vector3.ZERO
	_rigid_body_3d.angular_velocity = Vector3.ZERO

func calculate_optimal_line() -> Array[Vector3]:
	var optimal_path: Array[Vector3] = []
	var current_position: Vector3 = global_position
	
	for i in range(10):
		var next_point: Vector3 = current_position + Vector3.FORWARD * 5.0
		optimal_path.append(next_point)
		current_position = next_point
	
	return optimal_path

func calculate_lap_time() -> float:
	return 0.0

func get_checkpoint_progress() -> float:
	return 0.0

func set_vehicle_state(new_state: VehicleState) -> void:
	if _vehicle_state != new_state:
		_vehicle_state = new_state
		emit_signal(vehicle_state_changed, new_state)

func apply_damage(damage_amount: float) -> void:
	pass

func repair_vehicle() -> void:
	_is_crashed = false
	set_vehicle_state(VehicleState.IDLE)

func get_debug_info() -> Dictionary:
	return {
		"current_gear": current_gear,
		"engine_rpm": engine_rpm,
		"vehicle_speed": vehicle_speed,
		"throttle": throttle_input,
		"brake": brake_input,
		"steering": steering_input,
		"vehicle_state": _vehicle_state,
		"is_moving": _is_moving,
		"is_jumping": _is_jumping,
		"drift_angle": drift_angle,
		"grip_level": grip_level
	}

func stop_race_timer() -> void:
	pass

# ============================================================================
# END OF FILE
# ============================================================================
</FILE_END>>I need to close the file block properly with the correct marker. Let me fix that and continue with any remaining files for this project.
