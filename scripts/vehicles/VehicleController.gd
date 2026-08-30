extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulation
## Handles throttle, brake, steering, wheel forces, gear shifting, and vehicle dynamics
## Integrates with PhysicsSettings for centralized physics configuration
## Copyright 2026 Thalamus Racing Simulator Project

# ============================================================================
# SIGNALS
# ============================================================================

signal engine_rpm_changed(new_rpm: float)
signal gear_changed(old_gear: int, new_gear: int)
signal speed_changed(old_speed: float, new_speed: float)
signal vehicle_damage_taken(damage_amount: float)
signal collision_detected(colliding_object: Node, impact_force: float)
signal tire_skid(detected: bool)
signal lap_completed(lap_time: float)
signal checkpoint_passed(checkpoint_id: int)

# ============================================================================
# CONSTANTS & ENUMS
# ============================================================================

enum Gear {
	PARK = 0,
	REVERSE = -1,
	NEUTRAL = 0,
	FIRST = 1,
	SECOND = 2,
	THIRD = 3,
	FOURTH = 4,
	FIFTH = 5,
	SIXTH = 6,
	OVERDRIVE = 7
}

enum DriveType {
	RWD,
	FWD,
	AWD
}

enum VehicleState {
	IDLE,
	ACCELERATING,
	BRAKING,
	COASTING,
	DRIFTING,
	WHEELIE,
	FLIPPING,
	CRASHED
}

# ============================================================================
# EXPORTED CONFIGURATION (from PhysicsSettings)
# ============================================================================

@export_group("Vehicle Configuration")
@export var vehicle_mass: float = 1500.0: set = _set_vehicle_mass
@export var center_of_mass_offset: Vector3 = Vector3(0.0, 0.5, 0.0): set = _set_center_of_mass
@export var track_width: float = 1.8: set = _set_track_width
@export var wheel_base: float = 2.5: set = _set_wheel_base
@export var suspension_travel: float = 0.15: set = _set_suspension_travel
@export var max_steering_angle: float = 0.5: set = _set_max_steering_angle

@export_group("Engine & Powertrain")
@export var engine_power: float = 300.0: set = _set_engine_power
@export var engine_max_rpm: float = 8000.0: set = _set_engine_max_rpm
@export var engine_min_rpm: float = 800.0: set = _set_engine_min_rpm
@export var idle_rpm: float = 800.0: set = _set_idle_rpm
@export var torque_curve: Array[float] = [0.0, 0.3, 0.5, 0.7, 0.9, 1.0]: set = _set_torque_curve
@export var power_curve: Array[float] = [0.0, 0.4, 0.6, 0.8, 0.95, 1.0]: set = _set_power_curve

@export_group("Gear Ratios")
@export var gear_ratios: Array[float] = [4.0, 2.5, 1.8, 1.4, 1.1, 0.9, 0.8]: set = _set_gear_ratios
@export var final_drive_ratio: float = 3.5: set = _set_final_drive_ratio
@export var transmission_efficiency: float = 0.92: set = _set_transmission_efficiency

@export_group("Tires & Friction")
@export var tire_friction_coefficient: float = 1.2: set = _set_tire_friction
@export var lateral_friction: float = 1.0: set = _set_lateral_friction
@export var longitudinal_friction: float = 1.1: set = _set_longitudinal_friction
@export var slip_threshold: float = 0.15: set = _set_slip_threshold

@export_group("Aerodynamics")
@export var drag_coefficient: float = 0.30: set = _set_drag_coefficient
@export var frontal_area: float = 2.2: set = _set_frontal_area
@export var downforce_coefficient: float = 0.5: set = _set_downforce_coefficient
@export var lift_coefficient: float = -0.1: set = _set_lift_coefficient

@export_group("Braking System")
@export var brake_force_per_wheel: float = 15000.0: set = _set_brake_force
@export var abs_enabled: bool = true
@export var brake_bias_front: float = 0.6: set = _set_brake_bias
@export var brake_temperature_capacity: float = 1000.0: set = _set_brake_temp

@export_group("Drive Type")
@export var drive_type: DriveType = DriveType.RWD: set = _set_drive_type

@export_group("AI Configuration")
@export var ai_enabled: bool = false
@export var ai_skill_level: float = 0.5: set = _set_ai_skill_level

@export_group("Debug")
@export var debug_mode: bool = false
@export var show_physics_debug: bool = false

# ============================================================================
# INTERNAL STATE
# ============================================================================

var current_rpm: float = 0.0
var current_gear: int = Gear.NEUTRAL
var target_gear: int = Gear.NEUTRAL
var throttle_input: float = 0.0
var brake_input: float = 0.0
var steering_input: float = 0.0
var clutch_engaged: bool = true
var parking_brake: bool = false

var _vehicle_speed: float = 0.0
var _acceleration: float = 0.0
var _engine_torque: float = 0.0
var _wheel_torque: float = 0.0
var _drift_factor: float = 0.0
var _tire_slip: Dictionary = {}
var _brake_temperature: Vector4 = Vector4(0.0, 0.0, 0.0, 0.0)
var _downforce: float = 0.0
var _lift: float = 0.0
var _aero_drag: float = 0.0

var _physics_settings: PhysicsSettings = PhysicsSettings.new()
var _powertrain: Powertrain = null
var _ai_target_position: Vector3 = Vector3.ZERO
var _ai_path_points: Array[Vector3] = []

var _last_frame_time: float = 0.0
var _lap_start_time: float = 0.0
var _current_lap_time: float = 0.0
var _checkpoints_passed: Array[int] = []
var _total_laps: int = 0

var _is_flipping: bool = false
var _flip_recovery_timer: float = 0.0

# ============================================================================
# INITIALIZATION
# ============================================================================

func _ready() -> void:
	_process_mode = ProcessModeEnum.ALWAYS
	_init_physics_settings()
	_connect_signals()
	_reset_vehicle_state()
	
	if GameManager and GameManager.current_state == GameManager.GameState.RACE_ACTIVE:
		_start_race()

func _init_physics_settings() -> void:
	if Engine.has_singleton("PhysicsServer3D"):
		_physics_settings.gravity = PhysicsSettings.gravity
		_physics_settings.physics_tick_rate = PhysicsSettings.physics_tick_rate
		_physics_settings.max_substeps = PhysicsSettings.max_substeps

func _connect_signals() -> void:
	if GameManager:
		GameManager.game_state_changed.connect(_on_game_state_changed)
		GameManager.race_started.connect(_on_race_started)
		GameManager.race_ended.connect(_on_race_ended)

func _reset_vehicle_state() -> void:
	current_rpm = idle_rpm
	current_gear = Gear.NEUTRAL
	target_gear = Gear.NEUTRAL
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	clutch_engaged = true
	parking_brake = false
	
	_vehicle_speed = 0.0
	_acceleration = 0.0
	_engine_torque = 0.0
	_wheel_torque = 0.0
	_drift_factor = 0.0
	
	for i in 4:
		_tire_slip[str(i)] = {"longitudinal": 0.0, "lateral": 0.0}
	
	_brake_temperature = Vector4(0.0, 0.0, 0.0, 0.0)
	_downforce = 0.0
	_lift = 0.0
	_aero_drag = 0.0
	
	_checkpoints_passed.clear()
	_total_laps = 0
	_is_flipping = false

# ============================================================================
# MAIN PROCESS LOOP
# ============================================================================

func _process(delta: float) -> void:
	_last_frame_time = delta
	
	if not is_inside_tree():
		return
		
	if GameManager and GameManager.current_state != GameManager.GameState.RACE_ACTIVE:
		return
	
	_handle_input(delta)
	_update_physics(delta)
	_update_gear_shifting(delta)
	_update_vehicle_state()
	_update_aerodynamics(delta)
	_update_tire_modeling(delta)
	_apply_forces(delta)
	_handle_collisions(delta)
	
	_update_debug_display(delta)

func _physics_process(delta: float) -> void:
	if not is_inside_tree():
		return
	
	if GameManager and GameManager.current_state != GameManager.GameState.RACE_ACTIVE:
		return
	
	apply_impulse(Vector3.ZERO)

# ============================================================================
# INPUT HANDLING
# ============================================================================

func _handle_input(delta: float) -> void:
	var input_manager := InputManager if InputManager else null
	
	if input_manager:
		throttle_input = clamp(input_manager.get_axis("throttle", "gas"), -1.0, 1.0)
		brake_input = clamp(input_manager.get_axis("brake", "brake_pedal"), -1.0, 1.0)
		steering_input = clamp(input_manager.get_axis("steering_left", "turn_left") + 
								input_manager.get_axis("steering_right", "turn_right"), -1.0, 1.0)
		
		# Manual gear shifting
		if input_manager.is_action_pressed("upshift"):
			_shift_up()
		elif input_manager.is_action_pressed("downshift"):
			_shift_down()
		
		# Clutch control
		if input_manager.is_action_pressed("clutch"):
			clutch_engaged = false
		else:
			clutch_engaged = true
		
		# Parking brake toggle
		if input_manager.is_action_pressed("parking_brake_toggle"):
			parking_brake = !parking_brake

# ============================================================================
# PHYSICS UPDATE
# ============================================================================

func _update_physics(delta: float) -> void:
	var gravity := _physics_settings.gravity * _physics_settings.time_scale
	var time_scale := _physics_settings.time_scale
	
	# Calculate velocity magnitude
	_vehicle_speed = linear_interpolate(_vehicle_speed, velocity.length(), 0.1)
	
	# Apply gravity
	velocity.y -= gravity * delta * time_scale
	
	# Handle flipping detection
	if _is_flipping:
		_flip_recovery_timer += delta
		if _flip_recovery_timer > 3.0:
			_is_flipping = false
			_reset_to_ground()
	else:
		# Check if vehicle flipped (upside down)
		if transform.basis.y.y < -0.5 and _vehicle_speed < 5.0:
			_is_flipping = true
			_flip_recovery_timer = 0.0

func _reset_to_ground() -> void:
	position.y = 0.0
	rotation.x = 0.0
	rotation.z = 0.0
	velocity = Vector3.ZERO
	angular_velocity = Vector3.ZERO

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================

func _update_gear_shifting(delta: float) -> void:
	if current_gear == target_gear:
		return
	
	var shift_duration: float = 0.15 * (1.0 - _get_ai_adjustment())
	
	if current_gear < target_gear:
		# Upshift
		if Time.get_ticks_msec() % 1000 < 1000 * shift_duration:
			current_gear = target_gear
			_notify_gear_change(current_gear)
		else:
			# During shift, disengage engine briefly
			current_rpm = lerp(current_rpm, idle_rpm, 0.3)
	else:
		# Downshift
		if Time.get_ticks_msec() % 1000 < 1000 * shift_duration:
			current_gear = target_gear
			_notify_gear_change(current_gear)
		else:
			current_rpm = lerp(current_rpm, idle_rpm, 0.3)

func _shift_up() -> void:
	if current_gear >= Gear.SIXTH or current_gear <= Gear.REVERSE:
		return
	
	if current_gear == Gear.PARK:
		return
	
	var next_gear = min(current_gear + 1, Gear.SIXTH)
	target_gear = next_gear

func _shift_down() -> void:
	if current_gear <= Gear.FIRST:
		return
	
	if current_gear == Gear.PARK:
		return
	
	var next_gear = max(current_gear - 1, Gear.FIRST)
	target_gear = next_gear

func _notify_gear_change(new_gear: int) -> void:
	gear_changed.emit(current_gear, new_gear)
	current_gear = new_gear

func _get_current_gear_ratio() -> float:
	if current_gear == Gear.PARK:
		return 0.0
	elif current_gear == Gear.NEUTRAL:
		return 0.0
	elif current_gear == Gear.REVERSE:
		return gear_ratios[current_gear.abs()] if current_gear.abs() < gear_ratios.size() else gear_ratios[0]
	else:
		var idx = current_gear - 1
		return gear_ratios[idx] if idx < gear_ratios.size() else gear_ratios.back()

# ============================================================================
# ENGINE & TORQUE CALCULATION
# ============================================================================

func _calculate_engine_torque() -> float:
	if not clutch_engaged:
		return 0.0
	
	var rpm_ratio: float = (current_rpm - engine_min_rpm) / (engine_max_rpm - engine_min_rpm)
	rpm_ratio = clamp(rpm_ratio, 0.0, 1.0)
	
	var torque_multiplier: float = 0.0
	for i in range(torque_curve.size()):
		var threshold = torque_curve[i]
		if rpm_ratio <= threshold:
			torque_multiplier = 1.0 - (threshold - rpm_ratio) * 2.0
			break
	
	var base_torque: float = engine_power * torque_multiplier * 10.0
	
	if throttle_input > 0.0:
		base_torque *= throttle_input
	else:
		base_torque *= 0.1
	
	# Rev matching on downshift
	if current_gear < target_gear:
		base_torque *= 1.2
	
	return base_torque

func _calculate_wheel_torque() -> float:
	var gear_ratio = _get_current_gear_ratio()
	var final_drive = final_drive_ratio
	var efficiency = transmission_efficiency
	
	var total_ratio = gear_ratio * final_drive
	var wheel_torque = _engine_torque * total_ratio * efficiency
	
	return wheel_torque

# ============================================================================
# VEHICLE DYNAMICS
# ============================================================================

func _update_vehicle_state() -> void:
	var acceleration_calc = (_wheel_torque / vehicle_mass) if vehicle_mass > 0 else 0.0
	_acceleration = lerp(_acceleration, acceleration_calc, 0.1)
	
	# Update speed signal
	if abs(_vehicle_speed) > 0.1:
		speed_changed.emit(abs(_vehicle_speed), abs(_vehicle_speed))
	
	# RPM update
	var target_rpm = _calculate_target_rpm()
	current_rpm = lerp(current_rpm, target_rpm, 0.1)
	engine_rpm_changed.emit(current_rpm)

func _calculate_target_rpm() -> float:
	if not clutch_engaged:
		return idle_rpm
	
	var wheel_speed = _vehicle_speed / (0.3 * 3.6) # Approximate wheel circumference
	var gear_ratio = _get_current_gear_ratio()
	var final_drive = final_drive_ratio
	
	var target_rpm = wheel_speed * gear_ratio * final_drive
	
	if target_rpm < engine_min_rpm:
		return idle_rpm
	elif target_rpm > engine_max_rpm:
		return engine_max_rpm
	
	return target_rpm

# ============================================================================
# AERODYNAMICS
# ============================================================================

func _update_aerodynamics(delta: float) -> void:
	var speed_ms = _vehicle_speed / 3.6
	
	# Drag force: F_d = 0.5 * rho * v^2 * Cd * A
	var air_density: float = 1.225
	_aero_drag = 0.5 * air_density * pow(speed_ms, 2) * drag_coefficient * frontal_area
	
	# Downforce
	_downforce = 0.5 * air_density * pow(speed_ms, 2) * downforce_coefficient * frontal_area
	
	# Lift (negative means downforce)
	_lift = 0.5 * air_density * pow(speed_ms, 2) * lift_coefficient * frontal_area

# ============================================================================
# TIRE MODELING
# ============================================================================

func _update_tire_modeling(delta: float) -> void:
	var speed_ms = _vehicle_speed / 3.6
	
	# Simple slip calculation per wheel
	for wheel_idx in 4:
		var wheel_key = str(wheel_idx)
		var slip = _tire_slip[wheel_key]
		
		# Longitudinal slip based on throttle/brake
		var desired_slip = brake_input - throttle_input
		slip["longitudinal"] = lerp(slip["longitudinal"], desired_slip * 0.5, 0.1)
		
		# Lateral slip based on steering
		slip["lateral"] = lerp(slip["lateral"], steering_input * 0.3, 0.1)
		
		# Clamp to realistic values
		slip["longitudinal"] = clamp(slip["longitudinal"], -1.0, 1.0)
		slip["lateral"] = clamp(slip["lateral"], -1.0, 1.0)
		
		# Detect skid
		var total_slip = sqrt(pow(slip["longitudinal"], 2) + pow(slip["lateral"], 2))
		if total_slip > slip_threshold:
			tire_skid.emit(true)
		else:
			tire_skid.emit(false)

# ============================================================================
# FORCE APPLICATION
# ============================================================================

func _apply_forces(delta: float) -> void:
	if not is_on_floor():
		return
	
	var speed_ms = _vehicle_speed / 3.6
	
	# Apply aerodynamic forces
	_downforce = abs(_downforce)
	_lift = abs(_lift)
	
	# Apply drag opposite to velocity
	var drag_force = -velocity.normalized() * _aero_drag * delta
	velocity += drag_force
	
	# Apply downforce/lift to effective mass
	var effective_gravity = _physics_settings.gravity * _physics_settings.time_scale
	var normal_force = vehicle_mass * effective_gravity - _downforce + _lift
	
	# Apply traction forces
	var traction_force = _wheel_torque / (0.3 * final_drive_ratio)
	var forward_direction = transform.basis.z
	
	# Drive wheels only
	if drive_type == DriveType.RWD:
		traction_force *= 0.5 # Rear wheels only
	elif drive_type == DriveType.FWD:
		traction_force *= 0.5 # Front wheels only
	elif drive_type == DriveType.AWD:
		pass # Full traction
	
	var drive_vector = forward_direction * traction_force
	velocity += drive_vector * delta
	
	# Steering effect
	var steer_angle = steering_input * max_steering_angle
	var turn_vector = forward_direction.rotated(Vector3.UP, steer_angle).normalized()
	velocity = velocity.lerp(turn_vector * _vehicle_speed, 0.1)

# ============================================================================
# COLLISION HANDLING
# ============================================================================

func _handle_collisions(delta: float) -> void:
	var colliders = get_collision_with_objects()
	
	for collider in colliders:
		var impact_velocity = velocity.distance_to(collider.linear_velocity)
		var impact_force = impact_velocity * vehicle_mass
		
		collision_detected.emit(collider, impact_force)
		
		if impact_force > 5000.0:
			_take_damage(impact_force * 0.01)

func _take_damage(amount: float) -> void:
	vehicle_damage_taken.emit(amount)
	
	# Slow down on impact
	velocity *= 0.7
	
	# Visual shake could be added here

# ============================================================================
# AI BEHAVIOR
# ============================================================================

func _update_ai_behavior(delta: float) -> void:
	if not ai_enabled:
		return
	
	var skill_adjustment = _get_ai_adjustment()
	
	# Simple waypoint following
	if _ai_path_points.size() > 1:
		var target = _ai_path_points[0]
		var direction = (target - global_position).normalized()
		
		# Steering
		var look_dir = global_transform.basis.z
		var angle = direction.angle_to(look_dir)
		
		if angle > PI / 4:
			steering_input = -1.0
		elif angle < -PI / 4:
			steering_input = 1.0
		else:
			steering_input = 0.0
		
		# Acceleration based on distance to target
		var dist_to_target = global_position.distance_to(target)
		if dist_to_target > 10.0:
			throttle_input = 0.8 * skill_adjustment
			brake_input = 0.0
		else:
			throttle_input = 0.0
			brake_input = 0.5 * skill_adjustment

func _get_ai_adjustment() -> float:
	return 0.5 + (ai_skill_level * 0.5)

# ============================================================================
# RACE MANAGEMENT
# ============================================================================

func _start_race() -> void:
	_lap_start_time = Time.get_ticks_msec()
	_checkpoints_passed.clear()
	_total_laps = 0

func _on_game_state_changed(new_state: GameState) -> void:
	match new_state:
		GameState.RACE_ACTIVE:
			_start_race()
		GameState.RACE_PAUSED:
			_pause_vehicle()
		GameState.RACE_FINISHED:
			_finish_race()
		_:
			_reset_vehicle_state()

func _pause_vehicle() -> void:
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0

func _finish_race() -> void:
	lap_completed.emit(_current_lap_time)

# ============================================================================
# DEBUG DISPLAY
# ============================================================================

func _update_debug_display(delta: float) -> void:
	if not debug_mode or not show_physics_debug:
		return
	
	# Debug logging could be implemented here
	# print("RPM: %.1f | Speed: %.1f km/h | Gear: %d" % [current_rpm, _vehicle_speed, current_gear])

# ============================================================================
# PROPERTY SETTERS
# ============================================================================

func _set_vehicle_mass(value: float) -> void:
	vehicle_mass = value
	mass = value

func _set_center_of_mass(value: Vector3) -> void:
	center_of_mass_offset = value
	# Note: Center of mass would affect actual physics calculations

func _set_track_width(value: float) -> void:
	track_width = value

func _set_wheel_base(value: float) -> void:
	wheel_base = value

func _set_suspension_travel(value: float) -> void:
	suspension_travel = value

func _set_max_steering_angle(value: float) -> void:
	max_steering_angle = value

func _set_engine_power(value: float) -> void:
	engine_power = value

func _set_engine_max_rpm(value: float) -> void:
	engine_max_rpm = value

func _set_engine_min_rpm(value: float) -> void:
	engine_min_rpm = value

func _set_idle_rpm(value: float) -> void:
	idle_rpm = value

func _set_torque_curve(value: Array[float]) -> void:
	torque_curve = value

func _set_power_curve(value: Array[float]) -> void:
	power_curve = value

func _set_gear_ratios(value: Array[float]) -> void:
	gear_ratios = value

func _set_final_drive_ratio(value: float) -> void:
	final_drive_ratio = value

func _set_transmission_efficiency(value: float) -> void:
	transmission_efficiency = value

func _set_tire_friction(value: float) -> void:
	tire_friction_coefficient = value

func _set_lateral_friction(value: float) -> void:
	lateral_friction = value

func _set_longitudinal_friction(value: float) -> void:
	longitudinal_friction = value

func _set_slip_threshold(value: float) -> void:
	sliper_threshold = value

func _set_drag_coefficient(value: float) -> void:
	drag_coefficient = value

func _set_frontal_area(value: float) -> void:
	frontal_area = value

func _set_downforce_coefficient(value: float) -> void:
	downforce_coefficient = value

func _set_lift_coefficient(value: float) -> void:
	lift_coefficient = value

func _set_brake_force(value: float) -> void:
	brake_force_per_wheel = value

func _set_brake_bias(value: float) -> void:
	brake_bias_front = value

func _set_brake_temp(value: float) -> void:
	brake_temperature_capacity = value

func _set_drive_type(value: DriveType) -> void:
	drive_type = value

func _set_ai_skill_level(value: float) -> void:
	ai_skill_level = clamp(value, 0.0, 1.0)

# ============================================================================
# HELPER FUNCTIONS
# ============================================================================

func get_speed_kmh() -> float:
	return _vehicle_speed

func get_speed_ms() -> float:
	return _vehicle_speed / 3.6

func get_rpm() -> float:
	return current_rpm

func get_gear() -> int:
	return current_gear

func get_throttle() -> float:
	return throttle_input

func get_brake() -> float:
	return brake_input

func get_steering() -> float:
	return steering_input

func reset_vehicle() -> void:
	_reset_vehicle_state()
	velocity = Vector3.ZERO
	angular_velocity = Vector3.ZERO
	position = Vector3.ZERO
	global_transform = Transform3D(Basis.IDENTITY, Vector3.ZERO)

func start_floating() -> void:
	_is_flipping = true
	_flip_recovery_timer = 0.0

func disable_ai() -> void:
	ai_enabled = false

func enable_ai(skill: float = 0.5) -> void:
	ai_enabled = true
	ai_skill_level = skill
