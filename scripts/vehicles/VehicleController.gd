extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulation
## Handles throttle, brake, steering, wheel forces, gear shifting, and vehicle dynamics
## Copyright 2026 Thalamus Racing Simulator Project

# ============================================
# SIGNALS
# ============================================
signal speed_changed(new_speed: float)
signal rpm_changed(new_rpm: float)
signal gear_changed(new_gear: int)
signal drift_started(drift_angle: float)
signal drift_ended()
signal collision_detected(impact_force: Vector3)
signal lap_completed(lap_data: Dictionary)
signal race_event(event_type: String, data: Dictionary)
signal engine_sound_changed(rpm_ratio: float)
signal suspension_compressed(suspension_distance: float)

# ============================================
# CONSTANTS
# ============================================
const MAX_REVERSE_SPEED := 15.0  # m/s (approx 54 km/h reverse)
const MAX_FORWARD_SPEED := 85.0  # m/s (approx 306 km/h forward)
const STEERING_SPEED := 2.5      # radians per second max steering rate
const BRAKE_FORCE_MULTIPLIER := 12.0
const DRIFT_TIRE_GRIP_REDUCTION := 0.45
const TRACTION_CONTROL_THRESHOLD := 0.75
const ABS_THRESHOLD := 0.85

# ============================================
# EXPORTED CONFIGURATION
# ============================================
@export_group("Vehicle Configuration")
@export var vehicle_mass: float = 1500.0: set = _set_vehicle_mass
@export var center_of_mass_y: float = 0.5
@export var aerodynamic_drag_coefficient: float = 0.32
@export var frontal_area: float = 2.2  # m²
@export var downforce_coefficient: float = 0.05

@export_group("Engine & Powertrain")
@export var engine_max_rpm: float = 8500.0
@export var engine_idle_rpm: float = 800.0
@export var torque_curve_factor: float = 1.0
@export var power_delivery_smoothness: float = 0.8

@export_group("Transmission")
@export var transmission_type: TransmissionType = TransmissionType.MANUAL
@export var final_drive_ratio: float = 3.5
@export var tire_radius: float = 0.33  # meters (~13 inches)

@export_group("Gear Ratios")
@export var gear_ratios: Array[float] = [3.8, 2.5, 1.8, 1.4, 1.1, 0.9]
@export var reverse_gear_ratio: float = 3.5
@export var shift_up_threshold: float = 0.95
@export var shift_down_threshold: float = 0.25

@export_group("Braking System")
@export var front_brake_bias: float = 0.6
@export var max_brake_pressure: float = 100.0
@export var brake_bleed_rate: float = 15.0

@export_group("Tire Grip")
@export var grip_coefficient: float = 1.1
@export var lateral_grip_reduction_speed: float = 0.001
@export var longitudinal_grip_reduction_slip: float = 0.02

@export_group("Suspension")
@export var suspension_stiffness: float = 25000.0
@export var suspension_damping: float = 3500.0
@export var max_suspension_travel: float = 0.15
@export var spring_preload: float = 5000.0

@export_group("Drift Settings")
@export var drift_entry_angle: float = 0.35
@export var drift_hold_angle: float = 0.55
@export var drift_exit_recovery: float = 0.15
@export var drift_tire_temperature_factor: float = 1.2

# ============================================
# ENUMERATIONS
# ============================================
enum TransmissionType {
	MANUAL,
	SEMI_AUTOMATIC,
	AUTOMATIC
}

enum BrakeMode {
	CUSTOM,
	EPSILON_BASED
}

# ============================================
# PUBLIC READ-ONLY PROPERTIES
# ============================================
var current_speed: float = 0.0  # m/s
var current_rpm: float = 0.0
var current_gear: int = 0  # 0 = neutral, 1-6 = gears
var target_gear: int = 0
var is_drifting: bool = false
var drift_angle: float = 0.0
var drift_intensity: float = 0.0
var traction_active: bool = false
var abs_active: bool = false
var acceleration_vector: Vector3 = Vector3.ZERO
var velocity_vector: Vector3 = Vector3.ZERO
var angular_velocity: Vector3 = Vector3.ZERO
var applied_brake_force: float = 0.0
var applied_throttle: float = 0.0
var steering_angle: float = 0.0

# ============================================
# PRIVATE STATE VARIABLES
# ============================================
var _throttle_input: float = 0.0
var _brake_input: float = 0.0
var _steering_input: float = 0.0
var _current_tire_grip: float = 1.0
var _lateral_slip: float = 0.0
var _longitudinal_slip: float = 0.0
var _drift_timer: float = 0.0
var _drift_target_angle: float = 0.0
var _last_rotation: Quaternion = Quaternion.IDENTITY
var _torque_output: float = 0.0
var _wheel_forces: Array[Vector3] = []
var _suspension_distances: Array[float] = []
var _engine_braking_force: float = 0.0
var _gear_change_cooldown: float = 0.0
var _is_in_transition: bool = false

# ============================================
# INITIALIZATION
# ============================================
func _ready() -> void:
	_init_wheel_configuration()
	_reset_physics_state()
	set_physics_material_override(PhysicsSettings.get_instance().get_road_friction())

func _init_wheel_configuration() -> void:
	_wheel_forces.resize(4)
	_suspension_distances.resize(4)
	for i in range(4):
		_wheel_forces[i] = Vector3.ZERO
		_suspension_distances[i] = 0.0

func _reset_physics_state() -> void:
	current_speed = 0.0
	current_rpm = engine_idle_rpm
	current_gear = 0
	target_gear = 0
	is_drifting = false
	drift_angle = 0.0
	drift_intensity = 0.0
	traction_active = false
	abs_active = false
	applied_brake_force = 0.0
	applied_throttle = 0.0
	steering_angle = 0.0
	_throttle_input = 0.0
	_brake_input = 0.0
	_steering_input = 0.0
	_current_tire_grip = grip_coefficient
	_lateral_slip = 0.0
	_longitudinal_slip = 0.0
	_drift_timer = 0.0
	_drift_target_angle = 0.0
	_last_rotation = transform.basis.get_rotation_quaternion()
	_torque_output = 0.0
	_engine_braking_force = 0.0
	_gear_change_cooldown = 0.0
	_is_in_transition = false

# ============================================
# INPUT HANDLING
# ============================================
func handle_input(delta: float) -> void:
	_read_input_inputs(delta)
	_process_transmission_logic(delta)
	_update_drift_mechanics(delta)

func _read_input_inputs(delta: float) -> void:
	if InputManager.is_action_just_pressed("throttle"):
		_throttle_input = 1.0
	elif InputManager.is_action_just_released("throttle"):
		_throttle_input = 0.0
	
	if InputManager.is_action_just_pressed("brake"):
		_brake_input = 1.0
	elif InputManager.is_action_just_released("brake"):
		_brake_input = 0.0
	
	if InputManager.is_action_just_pressed("steer_left"):
		_steering_input = -1.0
	elif InputManager.is_action_just_released("steer_left"):
		_steering_input = 0.0
	
	if InputManager.is_action_just_pressed("steer_right"):
		_steering_input = 1.0
	elif InputManager.is_action_just_released("steer_right"):
		_steering_input = 0.0
	
	# Smooth input interpolation
	_throttle_input = lerp(_throttle_input, Input.get_axis("reverse", "throttle"), delta * 10.0)
	_brake_input = lerp(_brake_input, Input.get_axis("brake", "accelerate"), delta * 10.0)
	_steering_input = lerp(_steering_input, Input.get_axis("turn_left", "turn_right"), delta * 10.0)

func _process_transmission_logic(delta: float) -> void:
	if _gear_change_cooldown > 0.0:
		_gear_change_cooldown -= delta
		return
	
	# Manual transmission logic
	if transmission_type == TransmissionType.MANUAL:
		_handle_manual_gear_shifts(delta)
	
	# Semi-automatic or automatic
	else:
		_handle_auto_gear_shifts(delta)
	
	# Update gear signals
	if current_gear != target_gear:
		current_gear = target_gear
		gear_changed.emit(current_gear)

func _handle_manual_gear_shifts(delta: float) -> void:
	if InputManager.is_action_just_pressed("shift_up"):
		if current_gear < gear_ratios.size():
			target_gear = min(current_gear + 1, gear_ratios.size())
			_trigger_gear_shift()
	
	if InputManager.is_action_just_pressed("shift_down"):
		if current_gear > 0:
			target_gear = max(current_gear - 1, 0)
			_trigger_gear_shift()

func _handle_auto_gear_shifts(delta: float) -> void:
	# Automatic upshift at high RPM
	if current_rpm >= engine_max_rpm * shift_up_threshold and current_gear < gear_ratios.size():
		target_gear = min(current_gear + 1, gear_ratios.size())
		_trigger_gear_shift()
	
	# Automatic downshift at low RPM
	elif current_rpm <= engine_idle_rpm * 1.5 and current_gear > 0:
		target_gear = max(current_gear - 1, 0)
		_trigger_gear_shift()

func _trigger_gear_shift() -> void:
	_gear_change_cooldown = 0.15
	_is_in_transition = true
	await get_tree().create_timer(0.1).timeout
	_is_in_transition = false

# ============================================
# DRIFT MECHANICS
# ============================================
func _update_drift_mechanics(delta: float) -> void:
	var speed_magnitude = velocity_vector.length()
	
	# Calculate desired drift angle based on inputs
	var target_drift_angle = _calculate_drift_target()
	
	# Check if drift conditions are met
	if _can_enter_drift():
		_drift_timer += delta
		is_drifting = true
		
		if _drift_timer > 0.3:  # Minimum time to maintain drift
			drift_angle = lerp(drift_angle, target_drift_angle, delta * 5.0)
			drift_intensity = clamp((_drift_timer - 0.3) / 2.0, 0.0, 1.0)
			
			if not drift_started.is_connected(_on_drift_started):
				drift_started.connect(_on_drift_started)
			drift_started.emit(drift_angle)
		
		# Reduce tire grip during drift
		_current_tire_grip = grip_coefficient * DRIFT_TIRE_GRIP_REDUCTION * drift_tire_temperature_factor
	else:
		# Exit drift
		if is_drifting:
			_drift_timer = 0.0
			drift_angle = lerp(drift_angle, 0.0, delta * drift_exit_recovery)
			
			if drift_angle < 0.05:
				is_dripping = false
				drift_intensity = 0.0
				if drift_ended.is_connected(_on_drift_ended):
					drift_ended.emit()
		
		# Gradually restore tire grip
		_current_tire_grip = lerp(_current_tire_grip, grip_coefficient, delta * 2.0)

func _calculate_drift_target() -> float:
	var combined_input = abs(_steering_input) + abs(_throttle_input)
	if combined_input > 0.5:
		return drift_hold_angle
	else:
		return drift_entry_angle * 0.5

func _can_enter_drift() -> bool:
	if current_gear == 0:
		return false
	
	var speed_check = current_speed > 10.0 and current_speed < 60.0
	var input_check = abs(_steering_input) > 0.3 and (_throttle_input > 0.5 or _brake_input > 0.3)
	var slip_check = abs(_lateral_slip) > 0.15
	
	return speed_check and input_check and slip_check

func _on_drift_started(angle: float) -> void:
	race_event.emit("drift_start", {"angle": angle, "speed": current_speed})

func _on_drift_ended() -> void:
	race_event.emit("drift_end", {"final_angle": drift_angle})

# ============================================
# PHYSICS CALCULATIONS
# ============================================
func _physics_process(delta: float) -> void:
	_apply_gravity(delta)
	_calculate_acceleration(delta)
	_apply_brakes(delta)
	_calculate_steering(delta)
	_apply_wheel_forces(delta)
	_update_suspension(delta)
	_calculate_traction_control(delta)
	_calculate_abs(delta)
	_apply_aerodynamics(delta)
	_finalize_movement(delta)
	_update_signals(delta)

func _apply_gravity(delta: float) -> void:
	var gravity_vector = Vector3.UP * -PhysicsSettings.gravity * vehicle_mass
	add_force(gravity_vector)

func _calculate_acceleration(delta: float) -> void:
	# Calculate engine torque based on RPM and gear
	var gear_ratio = _get_current_gear_ratio()
	var wheel_torque = _calculate_engine_torque() * gear_ratio * final_drive_ratio
	
	# Apply torque to wheels
	var drive_force = (wheel_torque / tire_radius) * power_delivery_smoothness
	
	# Direction based on gear
	if current_gear == 0:
		_torque_output = 0.0
	elif current_gear < 0:  # Reverse
		_torque_output = -drive_force * _throttle_input
	else:
		_torque_output = drive_force * _throttle_input

func _calculate_engine_torque() -> float:
	var rpm_ratio = (current_rpm - engine_idle_rpm) / (engine_max_rpm - engine_idle_rpm)
	rpm_ratio = clamp(rpm_ratio, 0.0, 1.0)
	
	# Simple torque curve approximation
	var torque = engine_max_rpm * torque_curve_factor * (rpm_ratio * (1.0 - rpm_ratio * 0.5))
	return torque

func _get_current_gear_ratio() -> float:
	if current_gear == 0:
		return 1.0
	elif current_gear < 0:
		return -reverse_gear_ratio
	else:
		return gear_ratios[current_gear - 1]

func _apply_brakes(delta: float) -> void:
	if _brake_input > 0.0:
		var brake_force = max_brake_pressure * _brake_input * BRAKE_FORCE_MULTIPLIER
		var front_force = brake_force * front_brake_bias
		var rear_force = brake_force * (1.0 - front_brake_bias)
		
		# Apply braking force to wheels
		_wheel_forces[0] = -Vector3.FORWARD * front_force  # Front left
		_wheel_forces[1] = -Vector3.FORWARD * front_force  # Front right
		_wheel_forces[2] = -Vector3.FORWARD * rear_force   # Rear left
		_wheel_forces[3] = -Vector3.FORWARD * rear_force   # Rear right
		
		applied_brake_force = brake_force
	else:
		_wheel_forces.fill(Vector3.ZERO)
		applied_brake_force = 0.0

func _calculate_steering(delta: float) -> void:
	var target_steering = _steering_input * PI / 4.0  # Max 45 degrees
	steering_angle = lerp(steering_angle, target_steering, delta * STEERING_SPEED)

func _apply_wheel_forces(delta: float) -> void:
	var total_force = Vector3.ZERO
	
	for i in range(4):
		total_force += _wheel_forces[i]
	
	# Apply combined force to vehicle body
	add_force(total_force)
	acceleration_vector = total_force / vehicle_mass

func _update_suspension(delta: float) -> void:
	# Simplified suspension calculation
	var suspension_effective_mass = vehicle_mass / 4.0
	
	for i in range(4):
		var spring_force = suspension_stiffness * _suspension_distances[i]
		var damping_force = suspension_damping * _suspension_distances[i]
		
		# Apply to individual wheel positions
		var wheel_position = _get_wheel_position(i)
		var force = Vector3.DOWN * (spring_force + damping_force)
		apply_central_impulse(force * delta)
		
		suspension_compressed.emit(_suspension_distances[i])

func _get_wheel_position(index: int) -> Vector3:
	var offsets = [
		Vector3(-0.7, 0.25, 0.5),  # Front left
		Vector3(0.7, 0.25, 0.5),   # Front right
		Vector3(-0.7, 0.25, -0.5), # Rear left
		Vector3(0.7, 0.25, -0.5)   # Rear right
	]
	return position + transform.basis * offsets[index]

func _calculate_traction_control(delta: float) -> void:
	# Monitor wheel slip and reduce power if slipping too much
	var total_longitudinal_slip = abs(_longitudinal_slip)
	
	if total_longitudinal_slip > TRACTION_CONTROL_THRESHOLD:
		traction_active = true
		_torque_output *= (1.0 - (total_longitudinal_slip - TRACTION_CONTROL_THRESHOLD) * 0.5)
	else:
		traction_active = false

func _calculate_abs(delta: float) -> void:
	# Anti-lock braking system
	if applied_brake_force > 0.0 and current_speed < 5.0:
		abs_active = true
		# Pulse brakes to prevent locking
		var pulse_duration = delta * 10.0
		applied_brake_force *= 0.7
	else:
		abs_active = false

func _apply_aerodynamics(delta: float) -> void:
	var air_density = 1.225  # kg/m³ at sea level
	var drag_force = 0.5 * air_density * aerodynamic_drag_coefficient * frontal_area * current_speed * current_speed
	
	var downforce = 0.5 * air_density * downforce_coefficient * frontal_area * current_speed * current_speed
	
	# Apply aerodynamic forces
	add_force(-velocity_vector.normalized() * drag_force)
	add_force(Vector3.DOWN * downforce)

func _finalize_movement(delta: float) -> void:
	move_and_slide()
	velocity_vector = linear_velocity

func _update_signals(delta: float) -> void:
	# Calculate speed from velocity
	current_speed = velocity_vector.length()
	
	# Calculate RPM based on speed and gear
	var gear_ratio = _get_current_gear_ratio()
	var expected_rpm = (current_speed * gear_ratio * final_drive_ratio) / (2.0 * PI * tire_radius) * 60.0
	current_rpm = clamp(expected_rpm, engine_idle_rpm, engine_max_rpm)
	
	# Emit signals
	speed_changed.emit(current_speed)
	rpm_changed.emit(current_rpm)
	engine_sound_changed.emit(current_rpm / engine_max_rpm)
	
	# Track slip values
	_lateral_slip = abs(angular_velocity.y) * 0.1
	_longitudinal_slip = abs(acceleration_vector.x) / 9.81

# ============================================
# HELPER METHODS
# ============================================
func reset_vehicle() -> void:
	_reset_physics_state()
	transform = Transform3D.IDENTITY
	linear_velocity = Vector3.ZERO
	angular_velocity = Vector3.ZERO

func apply_collision_force(force: Vector3) -> void:
	collision_detected.emit(force)
	# Apply impact reaction
	add_impulse(-force.normalized() * vehicle_mass * 0.5)

func get_vehicle_health() -> float:
	var health = 100.0
	if current_speed > MAX_FORWARD_SPEED:
		health -= (current_speed - MAX_FORWARD_SPEED) * 5.0
	if current_rpm > engine_max_rpm:
		health -= (current_rpm - engine_max_rpm) * 0.1
	return max(health, 0.0)

func set_custom_gear(gear: int) -> void:
	target_gear = clamp(gear, 0, gear_ratios.size())
	_trigger_gear_shift()

func _set_vehicle_mass(value: float) -> void:
	vehicle_mass = value
	notify_property_changed()

func _set_gravity(value: float) -> void:
	PhysicsSettings.gravity = value
	notify_property_changed()

func _set_physics_tick_rate(value: int) -> void:
	PhysicsSettings.physics_tick_rate = value
	notify_property_changed()

func _set_max_substeps(value: int) -> void:
	PhysicsSettings.max_substeps = value
	notify_property_changed()

func _set_time_scale(value: float) -> void:
	PhysicsSettings.time_scale = value
	notify_property_changed()

func _set_default_vehicle_mass(value: float) -> void:
	PhysicsSettings.default_vehicle_mass = value
	notify_property_changed()

func _set_default_whee(value: float) -> void:
	PhysicsSettings.default_wheel_base = value
	notify_property_changed()

func _set_physics_tick_rate(value: int) -> void:
	PhysicsSettings.physics_tick_rate = value
	notify_property_changed()

func _set_max_substeps(value: int) -> void:
	PhysicsSettings.max_substeps = value
	notify_property_changed()

func _set_time_scale(value: float) -> void:
	PhysicsSettings.time_scale = value
	notify_property_changed()

func _set_gravity(value: float) -> void:
	PhysicsSettings.gravity = value
	notify_property_changed()

func notify_property_changed() -> void:
	pass  # Placeholder for property change notification
</FILE>