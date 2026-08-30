extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulation
## Handles throttle, brake, steering, wheel forces, gear shifting, and vehicle dynamics
## Copyright 2026 Thalamus Racing Simulator Project

# ============================================================================
# SIGNALS
# ============================================================================
signal speed_changed(new_speed: float)
signal rpm_changed(rpm: int)
signal gear_changed(gear: int)
signal drift_started(drift_angle: float)
signal drift_ended()
signal collision_detected(impact_velocity: Vector3)
signal lap_completed(lap_data: Dictionary)
signal race_event(event_type: String)
signal engine_sound_changed(rpm_ratio: float)
signal suspension_compressed(suspension_amount: float)

# ============================================================================
# CONSTANTS & CONFIGURATION
# ============================================================================
const MAX_ENGINE_RPM: int = 8500
const IDLE_RPM: int = 800
const REDLINE_RPM: int = 7200
const RPM_PER_GEAR: Array[float] = [0.0, 1200.0, 2400.0, 3600.0, 4800.0, 6000.0, 7200.0, 8500.0]
const GEAR_RATIOS: Array[float] = [0.0, 3.8f, 2.4f, 1.7f, 1.3f, 1.1f, 0.9f, 0.75f]
const FINAL_DRIVE_RATIO: float = 3.73f
const MAX_REVERSE_SPEED: float = -15.0
const MAX_FORWARD_SPEED: float = 120.0
const DRIFT_THRESHOLD: float = 0.5
const GRIP_LEVEL_NORMAL: float = 0.95
const GRIP_LEVEL_DRIFT: float = 0.35
const STEERING_SPEED: float = 15.0
const BRAKING_FORCE: float = 2.5
const ACCELERATION_FORCE: float = 1.8
const TURNING_RADIUS: float = 8.0
const DRAG_COEFFICIENT: float = 0.32
const AIR_DENSITY: float = 1.225
const WHEEL_BASE: float = 2.5
const TRACK_WIDTH: float = 1.6
const GRAVITY_ACCEL: float = PhysicsSettings.gravity * 100

# ============================================================================
# EXPORTED PROPERTIES
# ============================================================================
@export_group("Vehicle Configuration")
@export var vehicle_mass: float = 1500.0
@export var drag_coefficient: float = 0.32
@export var frontal_area: float = 2.1
@export var center_of_mass_height: float = 0.5
@export var wheel_base: float = 2.5
@export var track_width: float = 1.6

@export_group("Engine Settings")
@export var max_engine_rpm: int = 8500
@export var idle_rpm: int = 800
@export var redline_rpm: int = 7200
@export var torque_curve: Array[float] = [0.3, 0.6, 0.8, 1.0, 0.95, 0.9, 0.85, 0.8, 0.75, 0.7]

@export_group("Gearbox Settings")
@export var num_gears: int = 6
@export var clutch_engaged: bool = true

@export_group("Drivetrain Settings")
@export var drivetrain_type: DrivetrainType = DrivetrainType.RWD
enum DrivetrainType { FWD, RWD, AWD }

@export_group("Tire Settings")
@export var tire_friction_normal: float = 1.2
@export var tire_friction_drift: float = 0.4
@export var grip_level: float = 0.95

@export_group("Suspension Settings")
@export var suspension_stiffness: float = 50000.0
@export var suspension_damping: float = 5000.0
@export var suspension_travel: float = 0.15
@export var suspension_rest_length: float = 0.3

# ============================================================================
# INTERNAL STATE
# ============================================================================
var current_speed: float = 0.0
var current_rpm: int = IDLE_RPM
var current_gear: int = 0
var target_gear: int = 0
var throttle_input: float = 0.0
var brake_input: float = 0.0
var steering_input: float = 0.0
var handbrake_active: bool = false

var wheel_torque: float = 0.0
var wheel_steering_angle: float = 0.0
var engine_braking: float = 0.0
var aerodynamic_drag: float = 0.0
var centrifugal_force: float = 0.0

var drift_angle: float = 0.0
var drift_active: bool = false
var drift_intensity: float = 0.0

var suspension_states: Array[Vector3] = []
var wheel_contacts: Array[PhysicsRayResult3D] = []

var _last_velocity: Vector3 = Vector3.ZERO
var _acceleration: Vector3 = Vector3.ZERO
var _angular_velocity: Vector3 = Vector3.ZERO
var _engine_force: float = 0.0
var _transmission_efficiency: float = 0.85
var _clutch_slip: float = 0.0

# ============================================================================
# INITIALIZATION
# ============================================================================
func _ready() -> void:
	_init_suspension_states()
	_connect_signals()
	_set_initial_state()

func _init_suspension_states() -> void:
	suspension_states.clear()
	for i in range(4):
		suspension_states.append(Vector3.ZERO)

func _connect_signals() -> void:
	if Engine.has_singleton("GameManager"):
		GameManager.race_started.connect(_on_race_started)
		GameManager.game_state_changed.connect(_on_game_state_changed)

func _set_initial_state() -> void:
	current_gear = 0
	target_gear = 0
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	current_rpm = IDLE_RPM
	wheel_torque = 0.0
	engine_braking = 0.0
	aerodynamic_drag = 0.0
	centrifugal_force = 0.0
	velocity = Vector3.ZERO
	angular_velocity = Vector3.ZERO
	set_process(true)

# ============================================================================
# MAIN GAME LOOP
# ============================================================================
func _physics_process(delta: float) -> void:
	if not is_node_ready():
		return
	
	var input_delta := delta
	if GameManager.current_state != GameManager.GameState.RACE_ACTIVE:
		input_delta = 0.0
	
	_update_inputs(input_delta)
	_update_engine(input_delta)
	_update_transmission(input_delta)
	_update_wheels(input_delta)
	_update_vehicle_dynamics(input_delta)
	_update_drifting(input_delta)
	_update_aerodynamics(input_delta)
	_update_suspension(input_delta)
	_apply_forces(input_delta)
	_emit_signals()

func _update_inputs(delta: float) -> void:
	# Read input from InputManager singleton
	if Engine.has_singleton("InputManager"):
		var input_data := InputManager.get_vehicle_inputs()
		
		# Clamp inputs to valid ranges
		throttle_input = clamp(input_data.throttle, -1.0, 1.0)
		brake_input = clamp(input_data.brake, 0.0, 1.0)
		steering_input = clamp(input_data.steer, -1.0, 1.0)
		handbrake_active = input_data.handbrake
		
		# Smooth input transitions
		throttle_input = lerp(throttle_input, throttle_input, 0.8)
		brake_input = lerp(brake_input, brake_input, 0.8)
		steering_input = lerp(steering_input, steering_input, 0.9)

func _update_engine(delta: float) -> void:
	var target_rpm := _calculate_target_rpm()
	
	if clutch_engaged:
		# Calculate engine force based on torque curve
		var rpm_ratio := (current_rpm - idle_rpm) / float(max_engine_rpm - idle_rpm)
		rpm_ratio = clamp(rpm_ratio, 0.0, 1.0)
		
		var torque_multiplier := _get_torque_at_rpm(current_rpm)
		var engine_torque := torque_multiplier * 350.0 # Peak torque Nm
		
		# Apply throttle
		if throttle_input > 0:
			_engine_force = engine_torque * throttle_input
		else:
			_engine_force *= 0.5 # Coasting resistance
		
		# Engine braking when not accelerating
		if throttle_input == 0 and current_speed > 0:
			engine_braking = 50.0 * abs(current_speed) / MAX_FORWARD_SPEED
		else:
			engine_braking = 0.0
	else:
		# Clutch disengaged - free spinning
		target_rpm = idle_rpm + (throttle_input * 2000)
		clutch_slip = min(clutch_slip + delta * 10.0, 1.0)

func _calculate_target_rpm() -> int:
	if current_gear <= 0:
		return idle_rpm
	
	var wheel_speed := abs(current_speed) / WHEEL_BASE
	var transmission_ratio := GEAR_RATIOS[current_gear] * FINAL_DRIVE_RATIO
	var target_wheel_rpm := wheel_speed * transmission_ratio * 60.0 / (2.0 * PI)
	
	return int(target_wheel_rpm)

func _get_torque_at_rpm(rpm: int) -> float:
	var normalized_rpm := (rpm - idle_rpm) / float(max_engine_rpm - idle_rpm)
	normalized_rpm = clamp(normalized_rpm, 0.0, 1.0)
	
	var index := int(normalized_rpm * float(torque_curve.size() - 1))
	index = clamp(index, 0, torque_curve.size() - 2)
	
	var t1 := torque_curve[index]
	var t2 := torque_curve[index + 1]
	var fraction := normalized_rpm * float(torque_curve.size() - 1) - index
	
	return t1 + (t2 - t1) * fraction

func _update_transmission(delta: float) -> void:
	if current_gear == target_gear:
		return
	
	# Gear shift timing
	var shift_time := 0.2
	var shift_progress := fmod(Time.get_ticks_msec() / 1000.0, shift_time)
	
	# Downshift logic
	if throttle_input < 0.1 and current_rpm > RPM_PER_GEAR[target_gear + 1]:
		target_gear = max(0, target_gear - 1)
		gear_shift_complete()
	
	# Upshift logic  
	if throttle_input > 0.8 and current_rpm >= redline_rpm:
		target_gear = min(num_gears, target_gear + 1)
		gear_shift_complete()
	
	# Auto downshift on deceleration
	if brake_input > 0.5 and current_speed < MAX_FORWARD_SPEED * 0.3:
		target_gear = max(0, target_gear - 1)
		gear_shift_complete()

func gear_shift_complete() -> void:
	if current_gear != target_gear:
		current_gear = target_gear
		gear_changed.emit(current_gear)
		
		# Rev match for downshifts
		if current_gear < target_gear:
			var target_down_rpm := _calculate_target_rpm()
			current_rpm = lerp(current_rpm, target_down_rpm, 0.5)
		elif current_gear > target_gear:
			# Rev up for upshifts
			current_rpm = lerp(current_rpm, idle_rpm, 0.3)

func _update_wheels(delta: float) -> void:
	# Calculate wheel torque distribution based on drivetrain
	var drive_wheel_torque := _engine_force * _transmission_efficiency
	
	match drivetrain_type:
		DrivetrainType.FWD:
			wheel_torque = drive_wheel_torque * 0.6 # Front wheels only
		DrivetrainType.RWD:
			wheel_torque = drive_wheel_torque * 0.6 # Rear wheels only
		DrivetrainType.AWD:
			wheel_torque = drive_wheel_torque * 0.7 # All wheels

	# Steering angle calculation
	var max_steering_angle := deg_to_rad(35.0)
	wheel_steering_angle = steering_input * max_steering_angle
	
	# Wheel rotation speed
	var wheel_circumference := 0.7 # Approximate 0.7m diameter
	var wheel_rotations_per_sec := abs(current_speed) / wheel_circumference
	var wheel_angular_speed := wheel_rotations_per_sec * 2.0 * PI
	
	# Apply wheel angular velocity to vehicle
	_angular_velocity.y = wheel_angular_speed * sign(current_speed)

func _update_vehicle_dynamics(delta: float) -> void:
	_last_velocity = velocity
	
	# Calculate acceleration
	var total_force := _engine_force - aerodynamic_drag - engine_braking
	var net_acceleration := total_force / vehicle_mass
	
	# Apply gravity component on slopes
	var slope_angle := atan2(global_transform.basis.z.y, global_transform.basis.z.x)
	net_acceleration -= GRAVITY_ACCEL * sin(slope_angle)
	
	# Update velocity
	var direction := global_transform.basis.z.normalized()
	var forward_velocity := direction * current_speed
	
	_current_velocity = forward_velocity + (_acceleration * delta)
	velocity = _current_velocity
	
	# Update speed
	current_speed = velocity.length()
	
	# Clamp speeds
	current_speed = clamp(current_speed, MAX_REVERSE_SPEED, MAX_FORWARD_SPEED)
	
	# Update angular velocity for turning
	if abs(steering_input) > 0.1 and abs(current_speed) > 1.0:
		var turn_radius := TURNING_RADIUS / abs(steering_input)
		var angular_turn := current_speed / turn_radius
		_angular_velocity.y = angular_turn * sign(steering_input)

func _update_drifting(delta: float) -> void:
	# Detect drift conditions
	var slip_angle := _calculate_slip_angle()
	var lateral_acceleration := _calculate_lateral_acceleration()
	
	# Check if drifting
	var drift_condition := (abs(lateral_acceleration) > 3.0 and 
						   abs(steering_input) > 0.3 and 
						   handbrake_active)
	
	if drift_condition and not drift_active:
		drift_active = true
		drift_intensity = 0.0
		drift_started.emit(slip_angle)
	
	if not drift_condition or current_speed < 10.0:
		if drift_active:
			drift_active = false
			drift_ended.emit()
			drift_intensity = 0.0
	
	# Update drift intensity
	if drift_active:
		drift_intensity = lerp(drift_intensity, 1.0, delta * 5.0)
		drift_angle = lerp(drift_angle, slip_angle * drift_intensity, delta * 10.0)
	else:
		drift_angle = lerp(drift_angle, 0.0, delta * 15.0)

func _calculate_slip_angle() -> float:
	var velocity_direction := velocity.normalized()
	var forward_direction := global_transform.basis.z
	return velocity_direction.angle_to(forward_direction)

func _calculate_lateral_acceleration() -> float:
	var forward := global_transform.basis.z
	var lateral := forward.cross(Vector3.UP).normalized()
	return velocity.dot(lateral)

func _update_aerodynamics(delta: float) -> void:
	# Aerodynamic drag formula: F = 0.5 * rho * v^2 * Cd * A
	var speed_squared := current_speed * current_speed
	aerodynamic_drag = 0.5 * AIR_DENSITY * speed_squared * drag_coefficient * frontal_area
	
	# Downforce generation (increases grip at high speeds)
	var downforce := 0.5 * AIR_DENSITY * speed_squared * drag_coefficient * frontal_area * 0.3
	vehicle_mass += downforce / GRAVITY_ACCEL

func _update_suspension(delta: float) -> void:
	for i in range(4):
		var target_position := suspension_rest_length
		var current_position := suspension_states[i].y
		
		# Simple spring-damper model
		var spring_force := (target_position - current_position) * suspension_stiffness
		var damping_force := -_linear_velocity.y * suspension_damping
		
		var total_force := spring_force + damping_force
		
		# Apply to suspension state
		var new_position := current_position + total_force * delta / vehicle_mass
		new_position = clamp(new_position, 0.0, suspension_travel)
		suspension_states[i].y = new_position

func _apply_forces(delta: float) -> void:
	# Apply engine force
	if clutch_engaged:
		var drive_direction := global_transform.basis.z
		force_applied_signal.emit(drive_direction * _engine_force)
	
	# Apply brake force
	if brake_input > 0:
		var brake_force := brake_input * BRAKING_FORCE * vehicle_mass
		force_applied_signal.emit(-global_transform.basis.z * brake_force)
	
	# Apply steering effect
	if abs(steering_input) > 0.1:
		var steer_force := steering_input * current_speed * 50.0
		force_applied_signal.emit(global_transform.basis.y * steer_force)

func _emit_signals() -> void:
	speed_changed.emit(current_speed)
	rpm_changed.emit(current_rpm)
	engine_sound_changed.emit(float(current_rpm) / float(max_engine_rpm))
	
	if abs(current_speed - _last_velocity.length()) > 1.0:
		collision_detected.emit(velocity - _last_velocity)

# ============================================================================
# HELPER FUNCTIONS
# ============================================================================
func set_gear(gear: int) -> void:
	current_gear = clamp(gear, 0, num_gears)
	gear_changed.emit(current_gear)

func get_current_gear() -> int:
	return current_gear

func get_current_rpm() -> int:
	return current_rpm

func get_speed_kmh() -> float:
	return current_speed * 3.6

func get_speed_mph() -> float:
	return current_speed * 2.237

func reset_vehicle() -> void:
	current_speed = 0.0
	current_rpm = IDLE_RPM
	current_gear = 0
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	velocity = Vector3.ZERO
	_angular_velocity = Vector3.ZERO
	drift_active = false
	drift_angle = 0.0

func _on_race_started(race_data: Dictionary) -> void:
	reset_vehicle()

func _on_game_state_changed(new_state: GameState) -> void:
	if new_state == GameManager.GameState.MAIN_MENU:
		reset_vehicle()

# ============================================================================
# DEBUG FUNCTIONS
# ============================================================================
func debug_print_vehicle_state() -> void:
	print("=== VEHICLE STATE ===")
	print("Speed: %.2f km/h (%.2f m/s)" % [get_speed_kmh(), current_speed])
	print("RPM: %d / %d" % [current_rpm, max_engine_rpm])
	print("Gear: %d / %d" % [current_gear, num_gears])
	print("Throttle: %.2f" % throttle_input)
	print("Brake: %.2f" % brake_input)
	print("Steering: %.2f" % steering_input)
	print("Drift Active: %s" % str(drift_active))
	print("Drift Angle: %.2f deg" % rad_to_deg(drift_angle))
	print("Engine Force: %.2f N" % _engine_force)
	print("Drag Force: %.2f N" % aerodynamic_drag)
	print("=====================")