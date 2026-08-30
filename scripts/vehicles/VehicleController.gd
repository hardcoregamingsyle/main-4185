extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulation
## Handles throttle, brake, steering, wheel forces, gear shifting, and vehicle dynamics
## Copyright 2026 Thalamus Racing Simulator Project

# ============================================================================
# SIGNALS - Game Event Notifications
# ============================================================================
signal speed_changed(current_speed: float)
signal rpm_changed(current_rpm: float)
signal gear_changed(new_gear: int)
signal drift_started(drift_intensity: float)
signal drift_ended()
signal collision_detected(collision_info: Dictionary)
signal lap_completed(lap_data: Dictionary)
signal race_event(event_type: String, data: Dictionary)
signal engine_sound_changed(rpm_ratio: float)
signal suspension_compressed(compression_amount: float)

# ============================================================================
# CONSTANTS - Physics Tuning Values
# ============================================================================
const MAX_SPEED_KMH: float = 320.0
const ACCELERATION_RATE: float = 12.0
const BRAKING_FORCE: float = 20.0
const TURN_SPEED: float = 4.5
const DRIFT_THRESHOLD: float = 0.7
const DRIFT_INTENSITY_MAX: float = 1.0
const MIN_GEAR: int = 0
const MAX_GEAR: int = 6
const IDLE_RPM: float = 800.0
const REDLINE_RPM: float = 7500.0
const SHIFT_RPM: float = 7000.0
const CLUTCH_RELEASE_TIME: float = 0.15
const TURBO_CHARGE_TIME: float = 2.5
const SUSPENSION_COMPRESSION_LIMIT: float = 0.3

# ============================================================================
# EXPORTED CONFIGURATION - Vehicle Setup (Exposed in Inspector)
# ============================================================================
@export_group("Vehicle Configuration")
@export var vehicle_mass: float = 1500.0
@export var center_of_gravity_height: float = 0.55
@export var track_width: float = 1.6
@export var wheelbase: float = 2.6
@export var drag_coefficient: float = 0.32
@export var frontal_area: float = 2.2

@export_group("Powertrain Settings")
@export var engine_max_power: float = 350.0  # kW
@export var engine_max_torque: float = 500.0  # Nm
@export var transmission_type: String = "manual"  # manual, automatic, dual_clutch
@export var final_drive_ratio: float = 3.5
@export var differential_type: String = "limited_slip"

@export_group("Wheel Configuration")
@export var front_wheel_radius: float = 0.32
@export var rear_wheel_radius: float = 0.32
@export var wheel_inertia: float = 1.5
@export var tire_friction_coefficient: float = 1.2

@export_group("Drift & Handling")
@export var grip_level: float = 0.95
@export var oversteer_bias: float = 0.1
@export var understeer_bias: float = 0.1
@export var anti_roll_bar_stiffness: float = 0.8
@export var steering_angle_max: float = 0.5  # radians (~28 degrees)

@export_group("Aerodynamics")
@export var downforce_coefficient: float = 0.15
@export var wing_angle: float = 0.15  # radians (~8.6 degrees)

@export_group("Suspension")
@export var suspension_stiffness: float = 50000.0
@export var suspension_damping: float = 5000.0
@export var suspension_travel: float = 0.15
@export var ride_height: float = 0.3

# ============================================================================
# PRIVATE VARIABLES - Internal State
# ============================================================================
var _current_speed: float = 0.0  # km/h
var _current_rpm: float = IDLE_RPM
var _current_gear: int = 1
var _target_gear: int = 1
var _is_clutch_engaged: bool = true
var _clutch_position: float = 1.0  # 0 = fully released, 1 = fully engaged
var _throttle_input: float = 0.0   # 0-1
var _brake_input: float = 0.0     # 0-1
var _steering_input: float = 0.0  # -1 to 1
var _handbrake_active: bool = false

var _engine_state: enum { OFF, IDLE, RUNNING } = EngineState.OFF
var _turbo_charging: bool = false
var _turbo_charge_timer: float = 0.0
var _turbo_boost: float = 0.0

var _drift_intensity: float = 0.0
var _is_drifting: bool = false
var _drift_timer: float = 0.0

var _suspension_positions: Array[float] = []  # [front_left, front_right, rear_left, rear_right]
var _wheel_rotations: Array[float] = []      # [front_left, front_right, rear_left, rear_right]
var _wheel_forces: Array[Vector3] = []       # Force vectors applied to wheels

var _air_density: float = 1.225  # kg/m^3 at sea level
var _wind_velocity: Vector3 = Vector3.ZERO
var _aerodynamic_downforce: float = 0.0

var _collision_cooldown: float = 0.0
var _last_collision_time: float = 0.0

var _gear_ratios: Array[float] = [3.8, 2.2, 1.5, 1.1, 0.9, 0.7]
var _reverse_ratio: float = -3.5
var _idle_ratio: float = 1.0

var _acceleration_history: Array[float] = []
var _max_acceleration_buffer_size: int = 10

var _ground_contact_force: float = 0.0
var _surface_friction: float = 1.2

var _vehicle_health: float = 1.0  # 0-1
var _damage_accumulator: float = 0.0

# ============================================================================
# PHYSICS SETTINGS REFERENCES
# ============================================================================
var _physics_settings: PhysicsSettings = PhysicsSettings.new()

# ============================================================================
# INITIALIZATION
# ============================================================================
func _ready() -> void:
	_process_mode = ProcessModeEnum.ALWAYS
	_init_suspension_system()
	_init_wheels()
	_connect_signals()
	_load_physics_settings()
	print("VehicleController initialized for ", get_name())

func _init_suspension_system() -> void:
	"""Initialize suspension positions for all four wheels"""
	for i in range(4):
		_suspension_positions.append(suspension_travel * 0.5)  # Start at midpoint
		_wheel_rotations.append(0.0)
		_wheel_forces.append(Vector3.ZERO)

func _init_wheels() -> void:
	"""Initialize wheel properties and positions"""
	pass  # Wheels will be positioned by parent scene

func _connect_signals() -> void:
	"""Connect internal signals to GameManager if available"""
	if GameManager:
		race_event.connect(_on_race_event)

func _load_physics_settings() -> void:
	"""Load physics settings from singleton or create defaults"""
	if is_singleton_available("PhysicsSettings"):
		_physics_settings = PhysicsSettings.global_get()
	else:
		print("Warning: PhysicsSettings singleton not found, using defaults")

func is_singleton_available(name: String) -> bool:
	"""Check if an autoload singleton exists"""
	return name in get_tree().root.get_children()

func global_get() -> Node:
	"""Get reference to global singleton"""
	var root = get_tree().root
	for child in root.get_children():
		if child.name == "GameManager":
			return child
	return null

# ============================================================================
# INPUT HANDLING - Throttle, Brake, Steering
# ============================================================================
func _physics_process(delta: float) -> void:
	"""Main physics update loop - runs every frame"""
	_update_input(delta)
	_update_engine_state(delta)
	_update_transmission(delta)
	_update_vehicle_dynamics(delta)
	_update_aerodynamics(delta)
	_update_suspension(delta)
	_update_drift(delta)
	_update_collisions(delta)
	_check_gear_shifts(delta)
	_apply_forces(delta)
	_update_signals()

func _update_input(delta: float) -> void:
	"""Process player input for throttle, brake, steering, handbrake"""
	_throttle_input = InputManager.get_axis("throttle", "brake")
	_brake_input = InputManager.get_axis("brake", "gas")
	_steering_input = InputManager.get_axis("steer_left", "steer_right")
	_handbrake_active = Input.is_action_pressed("handbrake")
	
	# Clamp inputs to valid ranges
	_throttle_input = clamp(_throttle_input, 0.0, 1.0)
	_brake_input = clamp(_brake_input, 0.0, 1.0)
	_steering_input = clamp(_steering_input, -1.0, 1.0)

func _update_engine_state(delta: float) -> void:
	"""Update engine state based on throttle and clutch"""
	match _engine_state:
		EngineState.OFF:
			if _throttle_input > 0.0:
				_start_engine()
		
		EngineState.IDLE:
			if _throttle_input > 0.01:
				_engine_state = EngineState.RUNNING
			elif _clutch_position < 1.0:
				_clutch_position = min(_clutch_position + delta / CLUTCH_RELEASE_TIME, 1.0)
			else:
				_stop_engine()
		
		EngineState.RUNNING:
			if _throttle_input <= 0.01 and _clutch_position >= 1.0:
				_engine_state = EngineState.IDLE

func _start_engine() -> void:
	"""Start the engine from OFF state"""
	_engine_state = EngineState.IDLE
	_current_rpm = IDLE_RPM
	_current_gear = 1
	_is_clutch_engaged = true

func _stop_engine() -> void:
	"""Stop the engine"""
	_engine_state = EngineState.OFF
	_current_rpm = IDLE_RPM
	_current_gear = MIN_GEAR

enum EngineState {
	OFF,
	IDLE,
	RUNNING
}

# ============================================================================
# TRANSMISSION - Gear Shifting Logic
# ============================================================================
func _update_transmission(delta: float) -> void:
	"""Calculate transmission output based on current gear and RPM"""
	var gear_ratio = _get_current_gear_ratio()
	var effective_ratio = gear_ratio * final_drive_ratio
	
	# Calculate wheel RPM based on vehicle speed
	var wheel_rpm = (_current_speed * 1000.0 / 60.0) / (2.0 * PI * front_wheel_radius)
	_current_rpm = wheel_rpm * effective_ratio
	
	# Handle clutch engagement
	if !_is_clutch_engaged:
		_current_rpm = lerp(_current_rpm, IDLE_RPM, delta * 5.0)
	
	# Turbo charge logic
	if _turbo_charging:
		_turbo_charge_timer += delta
		if _turbo_charge_timer >= TURBO_CHARGE_TIME:
			_turbo_charging = false
			_turbo_boost = 0.0
		else:
			_turbo_boost = 1.0 - (_turbo_charge_timer / TURBO_CHARGE_TIME)

func _get_current_gear_ratio() -> float:
	"""Get gear ratio for current gear"""
	if _current_gear == MIN_GEAR:
		return _idle_ratio
	elif _current_gear < 0:
		return _reverse_ratio
	elif _current_gear <= len(_gear_ratios):
		return _gear_ratios[_current_gear - 1]
	else:
		return _gear_ratios[-1]

func _check_gear_shifts(delta: float) -> void:
	"""Automatically shift gears based on RPM thresholds"""
	if transmission_type == "automatic" or transmission_type == "dual_clutch":
		_auto_shift_gears()
	elif transmission_type == "manual" and Input.is_action_just_pressed("up_shift"):
		_manual_shift_gear(1)
	elif transmission_type == "manual" and Input.is_action_just_pressed("down_shift"):
		_manual_shift_gear(-1)

func _auto_shift_gears() -> void:
	"""Automatic gear shifting based on RPM"""
	var target_gear = _current_gear
	
	if _current_rpm > SHIFT_RPM and _current_gear < MAX_GEAR:
		target_gear = _current_gear + 1
	elif _current_rpm < IDLE_RPM * 1.5 and _current_gear > 1:
		target_gear = _current_gear - 1
	
	if target_gear != _current_gear:
		_change_gear(target_gear)

func _manual_shift_gear(direction: int) -> void:
	"""Manual gear shift command"""
	var new_gear = _current_gear + direction
	
	if direction > 0 and _current_gear < MAX_GEAR:
		_change_gear(_current_gear + 1)
	elif direction < 0 and _current_gear > 1:
		_change_gear(_current_gear - 1)

func _change_gear(new_gear: int) -> void:
	"""Change to specified gear with clutch control"""
	if new_gear == _current_gear:
		return
	
	# Disengage clutch briefly during shift
	_is_clutch_engaged = false
	_clutch_position = 0.0
	
	await get_tree().create_timer(CLUTCH_RELEASE_TIME).timeout
	
	_current_gear = new_gear
	_is_clutch_engaged = true
	_clutch_position = 1.0
	
	gear_changed.emit(new_gear)

# ============================================================================
# VEHICLE DYNAMICS - Acceleration, Deceleration, Velocity
# ============================================================================
func _update_vehicle_dynamics(delta: float) -> void:
	"""Calculate vehicle acceleration and velocity changes"""
	if _engine_state == EngineState.OFF:
		_decelerate_with_drag(delta)
		return
	
	var acceleration = _calculate_acceleration(delta)
	var deceleration = _calculate_deceleration(delta)
	
	# Apply net acceleration
	var net_acceleration = acceleration - deceleration
	_current_speed += net_acceleration * delta * 3.6  # Convert m/s² to km/h
	
	# Cap speed to maximum
	_current_speed = clamp(_current_speed, -MAX_SPEED_KMH / 2.0, MAX_SPEED_KMH)
	
	# Store acceleration history for smooth movement
	_acceleration_history.append(net_acceleration)
	if _acceleration_history.size() > _max_acceleration_buffer_size:
		_acceleration_history.pop_front()

func _calculate_acceleration(delta: float) -> float:
	"""Calculate forward acceleration based on throttle and gear"""
	if _throttle_input <= 0.0 or _current_gear == MIN_GEAR:
		return 0.0
	
	var gear_ratio = abs(_get_current_gear_ratio())
	var torque_multiplier = _get_torque_curve_factor()
	var turbo_multiplier = 1.0 + (_turbo_boost * 0.3)
	var clutch_factor = _is_clutch_engaged ? 1.0 : 0.0
	
	var acceleration = (
		engine_max_torque * gear_ratio * final_drive_ratio *
		torque_multiplier * turbo_multiplier * clutch_factor *
		_throttle_input / vehicle_mass
	)
	
	return min(acceleration, ACCELERATION_RATE)

func _calculate_deceleration(delta: float) -> float:
	"""Calculate braking and rolling resistance deceleration"""
	var base_deceleration = 0.0
	
	# Braking force
	if _brake_input > 0.0:
		base_deceleration += BRAKING_FORCE * _brake_input
	
	# Rolling resistance
	base_deceleration += 0.5 * _current_speed * 0.01
	
	# Downhill gravity effect (simplified)
	var slope_factor = 0.0  # Would come from terrain height map
	base_deceleration += 9.81 * slope_factor * 0.01
	
	return base_deceleration

func _decelerate_with_drag(delta: float) -> void:
	"""Apply air resistance when engine is off"""
	var drag_force = _calculate_air_resistance()
	var deceleration = drag_force / vehicle_mass
	
	_current_speed -= deceleration * delta * 3.6
	_current_speed = max(_current_speed, 0.0)

func _get_torque_curve_factor() -> float:
	"""Get torque multiplier based on RPM curve"""
	var rpm_ratio = (_current_rpm - IDLE_RPM) / (REDLINE_RPM - IDLE_RPM)
	
	# Simulate torque curve (peak around 4000-5000 RPM)
	var peak_rpm_ratio = 0.55
	var torque_factor = 1.0 - pow((rpm_ratio - peak_rpm_ratio) / 0.3, 2)
	
	return max(torque_factor, 0.1)

# ============================================================================
# AERODYNAMICS - Drag, Downforce, Wind Effects
# ============================================================================
func _update_aerodynamics(delta: float) -> void:
	"""Update aerodynamic forces acting on vehicle"""
	_wind_velocity = _get_wind_vector()
	_air_density = _calculate_air_density()
	_aerodynamic_downforce = _calculate_downforce()

func _calculate_air_resistance() -> float:
	"""Calculate air drag force opposing motion"""
	var speed_ms = _current_speed / 3.6
	
	var air_resistance = 0.5 * _air_density * drag_coefficient * frontal_area * \
		pow(speed_ms, 2)
	
	# Adjust for wind
	if _wind_velocity.length() > 0:
		var wind_component = _wind_velocity.dot(get_velocity().normalized())
		air_resistance *= (1.0 + wind_component * 0.5)
	
	return air_resistance

func _calculate_downforce() -> float:
	"""Calculate aerodynamic downforce pushing vehicle into ground"""
	var speed_ms = _current_speed / 3.6
	
	var downforce = 0.5 * _air_density * downforce_coefficient * frontal_area * \
		pow(speed_ms, 2)
	
	# Add wing angle effect
	downforce *= (1.0 + sin(wing_angle) * 0.3)
	
	return downforce

func _get_wind_vector() -> Vector3:
	"""Get current wind velocity vector"""
	# In full implementation, this would come from weather system
	return Vector3(0.0, 0.0, 0.0)

func _calculate_air_density() -> float:
	"""Calculate air density based on altitude (simplified)"""
	# Simplified: always return standard sea level density
	return _air_density

# ============================================================================
# SUSPENSION SYSTEM - Wheel Forces, Compression, Damping
# ============================================================================
func _update_suspension(delta: float) -> void:
	"""Update suspension compression and damping"""
	var gravity_force = vehicle_mass * PhysicsSettings.gravity
	
	# Distribute weight across suspension points
	var total_spring_force = 0.0
	for i in range(4):
		var spring_compression = _suspension_positions[i]
		var spring_force = suspension_stiffness * spring_compression
		var damping_force = suspension_damping * _get_suspension_velocity(i)
		
		total_spring_force += spring_force
		_suspension_positions[i] = max(0.0, min(suspension_travel, 
			suspension_positions[i] + (spring_force - damping_force - gravity_force / 4.0) * delta))
	
	_ground_contact_force = total_spring_force
	
	# Emit suspension signal
	var avg_compression = sum(_suspension_positions) / 4.0
	if avg_compression > 0.1:
		suspension_compressed.emit(avg_compression)

func _get_suspension_velocity(index: int) -> float:
	"""Calculate velocity of specific suspension point"""
	# Simplified: would need actual position tracking in full implementation
	return 0.0

func _apply_forces(delta: float) -> void:
	"""Apply calculated forces to vehicle body and wheels"""
	# Apply drive force to wheels
	_apply_wheel_forces(delta)
	
	# Apply aerodynamic downforce
	var downforce_vector = Vector3(0.0, -_aerodynamic_downforce, 0.0)
	add_force(downforce_vector)
	
	# Apply drag force opposite to velocity
	var velocity = get_velocity()
	if velocity.length() > 0.1:
		var drag_vector = -velocity.normalized() * _calculate_air_resistance()
		add_force(drag_vector)

func _apply_wheel_forces(delta: float) -> void:
	"""Apply driving forces to individual wheels"""
	if _engine_state == EngineState.OFF:
		return
	
	var drive_force = _calculate_drive_force()
	
	# Distribute force to rear wheels (RWD configuration)
	_wheel_forces[2] = Vector3(drive_force, 0.0, 0.0)
	_wheel_forces[3] = Vector3(drive_force, 0.0, 0.0)
	
	# Apply steering to front wheels
	var steering_angle = _steering_input * steering_angle_max
	_wheel_forces[0] = Vector3(drive_force * cos(steering_angle), 0.0, drive_force * sin(steering_angle))
	_wheel_forces[1] = Vector3(drive_force * cos(steering_angle), 0.0, drive_force * sin(steering_angle))

func _calculate_drive_force() -> float:
	"""Calculate total drive force based on engine output"""
	if _current_gear == MIN_GEAR:
		return 0.0
	
	var gear_ratio = abs(_get_current_gear_ratio())
	var torque = _get_torque_curve_factor() * engine_max_torque
	var force = torque * gear_ratio * final_drive_ratio / front_wheel_radius
	
	return force * _throttle_input * (_is_clutch_engaged ? 1.0 : 0.0)

# ============================================================================
# DRIFT MECHANICS - Oversteer, Understeer, Grip Control
# ============================================================================
func _update_drift(delta: float) -> void:
	"""Update drift state and intensity"""
	var lateral_velocity = _get_lateral_velocity()
	var speed_ratio = _current_speed / MAX_SPEED_KMH
	
	# Detect drift conditions
	var drift_threshold = DRIFT_THRESHOLD * (1.0 - grip_level)
	var is_drifting = abs(_steering_input) > drift_threshold and \
		abs(lateral_velocity) > 2.0 and _throttle_input > 0.3
	
	if is_drifting and !_is_drifting:
		_is_drifting = true
		_drift_intensity = DRIFT_INTENSITY_MAX
		drift_started.emit(_drift_intensity)
	elif !is_drifting and _is_drifting:
		_is_drifting = false
		drift_ended.emit()
	
	if _is_drifting:
		_drift_timer += delta
		_drift_intensity = max(0.0, _drift_intensity - delta * 0.1)
		
		# Reduce grip during drift
		var current_grip = grip_level * (1.0 - _drift_intensity * 0.5)
		_apply_drift_handling(current_grip)

func _get_lateral_velocity() -> float:
	"""Calculate lateral (side-to-side) velocity component"""
	# Simplified: would use proper side vector in full implementation
	return 0.0

func _apply_drift_handling(grip: float) -> void:
	"""Apply drift-specific handling characteristics"""
	var velocity = get_velocity()
	var forward_vector = transform.basis.z
	var side_vector = transform.basis.x
	
	# Blend between grip and drift behavior
	var grip_factor = grip
	var drift_factor = 1.0 - grip
	
	var friction_force = -velocity * grip_factor * 0.8
	var drift_force = Vector3(0.0, 0.0, 0.0)
	
	if _is_drifting:
		# Allow sideways sliding
		drift_force = side_vector * velocity.dot(side_vector) * 0.5
		
		# Apply oversteer/understeer bias
		if _steering_input > 0:
			drift_force.y += oversteer_bias * _steering_input
		else:
			drift_force.y -= understeer_bias * _steering_input
	
	add_force(friction_force + drift_force)

# ============================================================================
# COLLISION DETECTION - Impacts, Damage, Cooldowns
# ============================================================================
func _update_collisions(delta: float) -> void:
	"""Handle collision events and damage accumulation"""
	_collision_cooldown -= delta
	if _collision_cooldown < 0.0:
		_collision_cooldown = 0.0
	
	# Check for collisions via collision shape
	# In full implementation, connect to collision signals
	pass

func _handle_collision(collision_info: Dictionary) -> void:
	"""Process collision impact and apply damage"""
	if _collision_cooldown > 0.0:
		return
	
	var impact_speed = collision_info.get("speed", 0.0)
	var impact_force = collision_info.get("force", 0.0)
	
	# Calculate damage based on impact severity
	var damage = min(impact_force / 5000.0, 0.5)
	_damage_accumulator += damage
	_vehicle_health = max(0.0, _vehicle_health - damage)
	
	collision_detected.emit(collision_info)
	
	# Trigger effects
	_trigger_collision_effects(impact_speed)
	
	# Set cooldown
	_collision_cooldown = 0.5
	_last_collision_time = Time.get_ticks_msec()

func _trigger_collision_effects(impact_speed: float) -> void:
	"""Trigger visual and audio effects for collision"""
	# Screen shake based on impact
	if GameManager:
		GameManager.trigger_screen_shake(min(impact_speed / 100.0, 2.0))
	
	# Audio feedback
	if AudioManager:
		AudioManager.play_sound("collision_impact")

# ============================================================================
# SIGNAL UPDATES - Broadcast State Changes
# ============================================================================
func _update_signals() -> void:
	"""Emit signals for external systems to react to state changes"""
	speed_changed.emit(_current_speed)
	rpm_changed.emit(_current_rpm)
	engine_sound_changed.emit(_current_rpm / REDLINE_RPM)

func _on_race_event(event_type: String, data: Dictionary) -> void:
	"""Handle race events from GameManager"""
	match event_type:
		"race_started":
			_reset_vehicle()
		"race_finished":
			_stop_engine()
		"lap_completed":
			lap_completed.emit(data)

func _reset_vehicle() -> void:
	"""Reset vehicle to starting state"""
	_current_speed = 0.0
	_current_rpm = IDLE_RPM
	_current_gear = 1
	_throttle_input = 0.0
	_brake_input = 0.0
	_steering_input = 0.0
	_vehicle_health = 1.0
	_damage_accumulator = 0.0

# ============================================================================
# UTILITY FUNCTIONS - Getters, Setters, Helpers
# ============================================================================
func get_current_speed() -> float:
	"""Get current vehicle speed in km/h"""
	return _current_speed

func get_current_rpm() -> float:
	"""Get current engine RPM"""
	return _current_rpm

func get_current_gear() -> int:
	"""Get current gear number"""
	return _current_gear

func get_throttle_input() -> float:
	"""Get current throttle input value"""
	return _throttle_input

func get_brake_input() -> float:
	"""Get current brake input value"""
	return _brake_input

func get_steering_input() -> float:
	"""Get current steering input value"""
	return _steering_input

func get_vehicle_health() -> float:
	"""Get current vehicle health (0-1)"""
	return _vehicle_health

func is_engine_running() -> bool:
	"""Check if engine is currently running"""
	return _engine_state == EngineState.RUNNING

func is_drifting() -> bool:
	"""Check if vehicle is currently drifting"""
	return _is_drifting

func set_gear(new_gear: int) -> void:
	"""Manually set gear (for AI or testing)"""
	_change_gear(new_gear)

func set_throttle(input_value: float) -> void:
	"""Set throttle input manually (for AI or testing)"""
	_throttle_input = clamp(input_value, 0.0, 1.0)

func set_brake(input_value: float) -> void:
	"""Set brake input manually (for AI or testing)"""
	_brake_input = clamp(input_value, 0.0, 1.0)

func reset_all() -> void:
	"""Reset all vehicle states to default"""
	_reset_vehicle()
	_is_drifting = false
	_drift_intensity = 0.0
	_turbo_charging = false
	_turbo_boost = 0.0
	_suspension_positions.fill(suspension_travel * 0.5)

# ============================================================================
# DEBUG FUNCTIONS - Development Tools
# ============================================================================
func _draw_debug_visuals() -> void:
	"""Draw debug visualization for development"""
	if not GameManager.debug_mode:
		return
	
	# Draw wheel positions
	for i in range(4):
		var color = Color.GREEN
		if i % 2 == 0:
			color = Color.BLUE  # Front wheels
		else:
			color = Color.RED   # Rear wheels
		
		# Would draw spheres at wheel positions in full implementation
		pass

func get_debug_dictionary() -> Dictionary:
	"""Return current state as dictionary for debugging"""
	return {
		"speed_kmh": _current_speed,
		"rpm": _current_rpm,
		"gear": _current_gear,
		"throttle": _throttle_input,
		"brake": _brake_input,
		"steering": _steering_input,
		"is_drifting": _is_drifting,
		"health": _vehicle_health,
		"engine_state": _engine_state,
		"suspension": _suspension_positions.duplicate(),
		"turbo_boost": _turbo_boost
	}