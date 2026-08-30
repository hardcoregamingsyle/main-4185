extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulation
## Handles throttle, brake, steering, gear shifting, wheel forces, and vehicle dynamics
## Integrates with PhysicsSettings singleton for all physics constants
## Copyright 2026 Thalamus Racing Simulator Project

# ============================================================================
# SIGNALS - Events emitted by this controller
# ============================================================================
signal speed_changed(new_speed: float)
signal rpm_changed(new_rpm: float)
signal gear_changed(old_gear: int, new_gear: int)
signal vehicle_moved(position: Vector3, velocity: Vector3)
signal collision_detected(collision_data: Dictionary)
signal engine_started()
signal engine_stopped()
signal handbrake_toggled(is_active: bool)
signal drift_started(angle: float)
signal drift_ended()

# ============================================================================
# CONSTANTS & CONFIGURATION - References to PhysicsSettings
# ============================================================================
const VEHICLE_BASE_MASS := 1500.0  # Base mass in kg

# ============================================================================
# PUBLIC PROPERTIES - Exposed for inspector and external access
# ============================================================================
@export_group("Vehicle Configuration")
@export var vehicle_mass: float = 1500.0: set = _set_vehicle_mass
@export var max_speed_kmh: float = 320.0: set = _set_max_speed_kmh
@export var acceleration_force: float = 8000.0: set = _set_acceleration_force
@export var braking_force: float = 15000.0: set = _set_braking_force
@export var steering_angle_max: float = 45.0: set = _set_steering_angle_max

@export_group("Drivetrain Settings")
@export var drivetrain_type: DrivetrainType = DrivetrainType.FWD
@export var final_drive_ratio: float = 3.5: set = _set_final_drive_ratio
@export var tire_radius: float = 0.33: set = _set_tire_radius
@export var torque_curve: Array[Vector2f] = [
	Vector2f(0.0, 0.0),   # RPM fraction -> Torque multiplier
	Vector2f(0.2, 0.75),  # Low RPM
	Vector2f(0.4, 0.95),  # Mid RPM
	Vector2f(0.6, 1.0),   # Peak torque
	Vector2f(0.8, 0.95),  # High RPM
	Vector2f(1.0, 0.85)   # Redline
]: set = _set_torque_curve

@export_group("Gear Ratios")
@export var gear_ratios: Array[float] = [-1.5, 3.5, 2.2, 1.6, 1.3, 1.0, 0.8]: set = _set_gear_ratios
@export var rev_limit_rpm: int = 7000: set = _set_rev_limit_rpm
@export var idle_rpm: int = 800: set = _set_idle_rpm

@export_group("Steering & Handling")
@export var steering_sensitivity: float = 1.0
@export var steering_damping: float = 0.95
@export var auto_centering: bool = true
@export var grip_loss_threshold: float = 0.75

# ============================================================================
# ENUMERATIONS & TYPES
# ============================================================================
enum DrivetrainType {
	FWD,      # Front-Wheel Drive
	RWD,      # Rear-Wheel Drive
	AWD       # All-Wheel Drive
}

enum GearState {
	NEUTRAL = 0,
	REVERSE = -1,
	DIRECT = 1,
	N = 1,    # Neutral (alias)
	D = 1,    # Drive (alias)
	R = -1    # Reverse (alias)
}

# ============================================================================
# PRIVATE VARIABLES - Internal state management
# ============================================================================
var _current_speed: float = 0.0           # Current speed in km/h
var _current_rpm: int = 0                 # Current engine RPM
var _current_gear: int = GearState.N      # Current gear selection
var _target_gear: int = GearState.N       # Target gear (for automatic shifting)
var _throttle_input: float = 0.0          # Throttle input (0.0 - 1.0)
var _brake_input: float = 0.0             # Brake input (0.0 - 1.0)
var _steering_input: float = 0.0          # Steering input (-1.0 to 1.0)
var _handbrake_active: bool = false       # Handbrake toggle state
var _clutch_pressed: bool = false         # Clutch pedal state
var _engine_running: bool = false         # Engine on/off state
var _is_drifting: bool = false            # Drift state flag
var _drift_angle: float = 0.0             # Current drift angle
var _last_position: Vector3               # Previous position for movement tracking
var _wheel_rotation_angles: Vector4 = Vector4.ZERO  # R, F_L, F_R, L_R wheel angles
var _powertrain_node: Node = null         # Reference to powertrain system
var _collision_box: CollisionShape3D = null # Vehicle collision shape
var _ground_contact_normal: Vector3 = Vector3.UP  # Ground normal vector
var _velocity_vector: Vector3 = Vector3.ZERO  # World velocity vector
var _acceleration_vector: Vector3 = Vector3.ZERO  # Acceleration vector

# ============================================================================
# PRIVATE VARIABLES - Performance & physics calculations
# ============================================================================
var _gear_shift_delay_timer: float = 0.0  # Cooldown between gear shifts
var _shift_timeout: float = 0.2           # Seconds before shift completes
var _vehicle_rotation: float = 0.0        # Y-axis rotation
var _current_steering_angle: float = 0.0  # Actual steering angle
var _max_steering_angle_rad: float = 0.785  # Max steering in radians (45°)
var _friction_coefficient: float = 1.2    # Tire friction coefficient
var _air_resistance: float = 0.05         # Aerodynamic drag factor
var _rolling_resistance: float = 0.01     # Rolling resistance factor
var _downforce_factor: float = 0.1        # Downforce at speed

# ============================================================================
# PUBLIC READ-ONLY PROPERTIES
# ============================================================================
func get_current_speed() -> float:
	return _current_speed

func get_current_rpm() -> int:
	return _current_rpm

func get_current_gear() -> int:
	return _current_gear

func get_throttle_input() -> float:
	return _throttle_input

func get_brake_input() -> float:
	return _brake_input

func get_steering_input() -> float:
	return _steering_input

func is_engine_running() -> bool:
	return _engine_running

func is_drifting() -> bool:
	return _is_drifting

func get_wheel_rotation_angles() -> Vector4:
	return _wheel_rotation_angles

# ============================================================================
# SETUP METHODS
# ============================================================================
func _ready() -> void:
	_last_position = global_position
	
	# Initialize collision box if not present
	if not _collision_box:
		_collision_box = CollisionShape3D.new()
		_collision_box.name = "CollisionBox"
		add_child(_collision_box)
	
	# Connect to GameManager signals for coordinated behavior
	GameManager.game_state_changed.connect(_on_game_state_changed)
	
	# Start engine automatically
	start_engine()
	
	# Set initial rotation
	vehicle_rotation = Vector3(0, 0, 0)

func _process(delta: float) -> void:
	_update_steering(delta)
	_handle_auto_centering(delta)
	_check_drift_conditions()
	_apply_downforce(delta)

func _physics_process(delta: float) -> void:
	# Update physics at fixed timestep
	_physics_step(delta)
	_update_vehicle_velocity()
	_sync_with_world()

# ============================================================================
# INPUT HANDLING - External control interface
# ============================================================================
func set_throttle(input_value: float) -> void:
	_throttle_input = clamp(input_value, 0.0, 1.0)
	if _throttle_input > 0.0 and not _engine_running:
		start_engine()

func set_brake(input_value: float) -> void:
	_brake_input = clamp(input_value, 0.0, 1.0)

func set_steering(input_value: float) -> void:
	_steering_input = clamp(input_value, -1.0, 1.0) * steering_sensitivity

func toggle_handbrake(is_active: bool) -> void:
	_handbrake_active = is_active
	handbrake_toggled.emit(is_active)

func set_clutch_pressed(is_pressed: bool) -> void:
	_clutch_pressed = is_pressed
	if not is_pressed and _current_gear == GearState.N:
		engage_gear(1)

func start_engine() -> void:
	if _engine_running:
		return
	_engine_running = true
	engine_started.emit()
	_current_rpm = idle_rpm

func stop_engine() -> void:
	if not _engine_running:
		return
	_engine_running = false
	_current_rpm = 0
	engine_stopped.emit()

func shift_up() -> void:
	if _current_gear < gear_ratios.size() - 1 and _current_gear > 0:
		var old_gear = _current_gear
		_current_gear += 1
		gear_changed.emit(old_gear, _current_gear)
		_reset_shift_cooldown()

func shift_down() -> void:
	if _current_gear > 1 and _current_gear <= gear_ratios.size():
		var old_gear = _current_gear
		_current_gear -= 1
		gear_changed.emit(old_gear, _current_gear)
		_reset_shift_cooldown()

func manual_shift(target_gear: int) -> void:
	if target_gear >= -1 and target_gear < gear_ratios.size():
		var old_gear = _current_gear
		_current_gear = target_gear
		gear_changed.emit(old_gear, _current_gear)
		_reset_shift_cooldown()

func engage_gear(gear: int) -> void:
	if gear >= -1 and gear < gear_ratios.size():
		_current_gear = gear
		gear_changed.emit(GearState.N, gear)
		_reset_shift_cooldown()

# ============================================================================
# PHYSICS UPDATE LOOP
# ============================================================================
func _physics_step(delta: float) -> void:
	if not _engine_running:
		_apply_friction_and_drag(delta)
		return
	
	# Calculate engine torque based on RPM and gear
	var torque_mult = _get_torque_multiplier(_current_rpm / rev_limit_rpm)
	var base_torque = 400.0  # Nm at peak
	var engine_torque = base_torque * torque_mult
	
	# Apply gear ratio and final drive
	var gear_ratio = gear_ratios[_current_gear] if _current_gear > 0 else 0.0
	var total_ratio = gear_ratio * final_drive_ratio
	
	# Calculate wheel force
	var wheel_torque = engine_torque * total_ratio
	var wheel_force = wheel_torque / tire_radius
	
	# Apply drivetrain distribution
	wheel_force = _apply_drivetrain_distribution(wheel_force)
	
	# Handle braking
	var brake_force = 0.0
	if _brake_input > 0.0 or _handbrake_active:
		var braking_effort = _brake_input * braking_force
		if _handbrake_active:
			braking_effort *= 1.5  # Handbrake bonus
		brake_force = -braking_effort
	
	# Combine forces
	var net_force = wheel_force + brake_force
	
	# Apply acceleration to velocity
	var acceleration = net_force / vehicle_mass
	_velocity_vector += acceleration * delta
	
	# Apply air resistance and rolling resistance
	_apply_aerodynamics(delta)
	
	# Update speed from velocity magnitude
	_update_speed_from_velocity()
	
	# Update RPM based on wheel speed and gear
	_update_engine_rpm()
	
	# Move vehicle body
	move_and_slide()

func _update_engine_rpm() -> void:
	if _current_gear == 0:
		# In neutral, RPM decays to idle
		_current_rpm = lerp(_current_rpm, idle_rpm, 0.1)
		return
	
	if _current_gear < 0:
		# Reverse gear
		var reverse_ratio = abs(gear_ratios[_current_gear])
		var wheel_speed = abs(_current_speed) / 3.6  # Convert km/h to m/s
		_current_rpm = int((wheel_speed / tire_radius) * reverse_ratio * final_drive_ratio * 60.0 / (2.0 * PI))
	else:
		# Forward gears
		var forward_ratio = gear_ratios[_current_gear]
		var wheel_speed = _current_speed / 3.6  # Convert km/h to m/s
		_current_rpm = int((wheel_speed / tire_radius) * forward_ratio * final_drive_ratio * 60.0 / (2.0 * PI))
	
	# Clamp to valid range
	_current_rpm = clamp(_current_rpm, idle_rpm, rev_limit_rpm)
	rpm_changed.emit(_current_rpm)

func _update_speed_from_velocity() -> void:
	_current_speed = _velocity_vector.length() * 3.6  # Convert m/s to km/h
	speed_changed.emit(_current_speed)

func _update_vehicle_velocity() -> void:
	velocity = _velocity_vector.normalized() * _current_speed / 3.6

func _sync_with_world() -> void:
	# Emit movement signal for tracking
	if _last_position != global_position:
		vehicle_moved.emit(global_position, _velocity_vector)
		_last_position = global_position

# ============================================================================
# STEERING SYSTEM
# ============================================================================
func _update_steering(delta: float) -> void:
	if _steering_input.abs() > 0.01:
		# Steer towards input
		var target_angle = _steering_input * _max_steering_angle_rad
		_current_steering_angle = lerp(_current_steering_angle, target_angle, delta * 10.0)
	else:
		# Return to center when no input
		_current_steering_angle = lerp(_current_steering_angle, 0.0, delta * 5.0)

func _handle_auto_centering(delta: float) -> void:
	if auto_centering and _steering_input.abs() < 0.01:
		_current_steering_angle = lerp(_current_steering_angle, 0.0, delta * 8.0)

# ============================================================================
# DRIFT MECHANICS
# ============================================================================
func _check_drift_conditions() -> void:
	if _current_speed < 10.0:
		_is_drifting = false
		return
	
	# Check for drift entry conditions
	var lateral_acceleration = _calculate_lateral_acceleration()
	var slip_angle = _calculate_slip_angle()
	
	if lateral_acceleration > grip_loss_threshold and _handbrake_active:
		if not _is_drifting:
			_is_drifting = true
			drift_started.emit(slip_angle)
	elif not _handbrake_active and slip_angle < grip_loss_threshold:
		if _is_drifting:
			_is_drifting = false
			drift_ended.emit()

func _calculate_lateral_acceleration() -> float:
	# Simplified lateral G-force calculation
	return abs(_velocity_vector.x) * 9.81 / 100.0

func _calculate_slip_angle() -> float:
	# Slip angle estimation based on steering and velocity
	return abs(_current_steering_angle) * (_current_speed / 100.0)

# ============================================================================
# AERODYNAMICS & FRICTION
# ============================================================================
func _apply_aerodynamics(delta: float) -> void:
	var air_drag = _velocity_vector.length_squared() * _air_resistance * delta
	_velocity_vector -= _velocity_vector.normalized() * air_drag

func _apply_friction_and_drag(delta: float) -> void:
	var friction_force = _velocity_vector.length() * _friction_coefficient * delta
	_velocity_vector -= _velocity_vector.normalized() * friction_force

func _apply_downforce(delta: float) -> void:
	var downforce = pow(_current_speed / 100.0, 2) * _downforce_factor * vehicle_mass
	# Apply as additional weight to ground reaction
	pass  # Implementation depends on suspension system

# ============================================================================
# POWERTRAIN INTEGRATION
# ============================================================================
func _apply_drivetrain_distribution(force: float) -> float:
	match drivetrain_type:
		DrivetrainType.FWD:
			return force * 0.6  # 60% front, 40% rear
		DrivetrainType.RWD:
			return force * 0.4  # 40% front, 60% rear
		DrivetrainType.AWD:
			return force  # Full power to all wheels
		_:
			return force

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================
func _get_torque_multiplier(rpm_fraction: float) -> float:
	# Linear interpolation through torque curve points
	for i in range(torque_curve.size() - 1):
		var p1 = torque_curve[i]
		var p2 = torque_curve[i + 1]
		if rpm_fraction >= p1.x and rpm_fraction <= p2.x:
			var t = (rpm_fraction - p1.x) / (p2.x - p1.x)
			return p1.y + (p2.y - p1.y) * t
	return torque_curve[-1].y

func _reset_shift_cooldown() -> void:
	_gear_shift_delay_timer = _shift_timeout

func _set_vehicle_mass(value: float) -> void:
	vehicle_mass = value
	# Adjust mass affects inertia calculations
	pass

func _set_max_speed_kmh(value: float) -> void:
	max_speed_kmh = value

func _set_final_drive_ratio(value: float) -> void:
	final_drive_ratio = value

func _set_tire_radius(value: float) -> void:
	tire_radius = value

func _set_torque_curve(value: Array[Vector2f]) -> void:
	torque_curve = value

func _set_gear_ratios(value: Array[float]) -> void:
	gear_ratios = value

func _set_rev_limit_rpm(value: int) -> void:
	rev_limit_rpm = value

func _set_idle_rpm(value: int) -> void:
	idle_rpm = value

# ============================================================================
# GAME STATE HANDLERS
# ============================================================================
func _on_game_state_changed(new_state: GameManager.GameState) -> void:
	match new_state:
		GameManager.GameState.MAIN_MENU:
			stop_engine()
			_current_gear = GearState.N
		GameManager.GameState.RACE_ACTIVE:
			start_engine()
		GameManager.GameState.RACE_PAUSED:
			# Pause physics updates handled elsewhere
			pass
		GameManager.GameState.RACE_FINISHED:
			stop_engine()
			_current_gear = GearState.N

# ============================================================================
# DEBUG & VISUALIZATION
# ============================================================================
func _draw_debug_info(debug_text: Label) -> void:
	if debug_text:
		debug_text.text = "Speed: %.1f km/h\nRPM: %d\nGear: %d\nThrottle: %.1f\nBrake: %.1f\nSteering: %.1f" % [
			_current_speed,
			_current_rpm,
			_current_gear,
			_throttle_input,
			_brake_input,
			_steering_input
		]

# ============================================================================
# SAVE/LOAD SUPPORT
# ============================================================================
func save_state() -> Dictionary:
	return {
		"speed": _current_speed,
		"rpm": _current_rpm,
		"gear": _current_gear,
		"position": global_position,
		"rotation": rotation,
		"throttle": _throttle_input,
		"brake": _brake_input,
		"steering": _steering_input
	}

func load_state(state_data: Dictionary) -> void:
	if state_data.has("speed"):
		_current_speed = state_data["speed"]
	if state_data.has("rpm"):
		_current_rpm = state_data["rpm"]
	if state_data.has("gear"):
		_current_gear = state_data["gear"]
	if state_data.has("position"):
		global_position = state_data["position"]
	if state_data.has("rotation"):
		rotation = state_data["rotation"]

# ============================================================================
# CLEANUP
# ============================================================================
func _exit_tree() -> void:
	stop_engine()
	_disconnect_signals()

func _disconnect_signals() -> void:
	GameManager.game_state_changed.disconnect(_on_game_state_changed)

# ============================================================================
# END OF FILE
# ============================================================================