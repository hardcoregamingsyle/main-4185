extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for the racing simulator
## Handles throttle, brake, steering inputs, wheel forces, and gear shifting logic
## Uses PhysicsSettings for all tunable constants
## Copyright 2026 Thalamus Racing Simulator Project

# ============================================================================
# SIGNALS
# ============================================================================

signal speed_changed(new_speed: float)
signal gear_changed(old_gear: int, new_gear: int)
signal skid_detected(skid_factor: float)
signal collision_impact(impact_force: Vector3)

# ============================================================================
# CONSTANTS & CONFIGURATION
# ============================================================================

const MAX_THROTTLE_FORCE := PhysicsSettings.default_throttle_force
const MAX_BRAKE_FORCE := PhysicsSettings.default_brake_force
const MAX_STEERING_ANGLE := PhysicsSettings.default_max_steering_angle
const STEERING_SMOOTHNESS := PhysicsSettings.default_steering_smoothness
const WHEEL_RADIUS := PhysicsSettings.default_wheel_radius
const GEAR_RATIOS := {
	1: 3.5,
	2: 2.2,
	3: 1.7,
	4: 1.4,
	5: 1.1,
	6: 0.9,
	R: 3.8,
	N: 0.0
}
const SHIFT_DELAY_TIME := 0.15

# ============================================================================
# PUBLIC PROPERTIES
# ============================================================================

## Current vehicle speed in m/s
var current_speed: float = 0.0 get: return _current_speed
## Current gear (N=Neutral, R=Reverse, 1-6 forward gears)
var current_gear: int = 0 get: return _current_gear
## Engine RPM (revolutions per minute)
var engine_rpm: float = 0.0 get: return _engine_rpm
## Total vehicle mass including cargo
var total_mass: float = PhysicsSettings.default_vehicle_mass get: return _total_mass set = _set_total_mass
## Acceleration coefficient (0-1, affects throttle response)
var acceleration_coefficient: float = 1.0 get: return _acceleration_coefficient set = _set_acceleration_coefficient
## Deceleration coefficient (0-1, affects braking efficiency)
var deceleration_coefficient: float = 1.0 get: return _deceleration_coefficient set = _set_deceleration_coefficient
## Steering sensitivity multiplier
var steering_multiplier: float = 1.0 get: return _steering_multiplier set = _set_steering_multiplier
## Drivetrain type: 0=FR, 1=RR, 2=FWD, 3=RWD, 4=AWD
var drivetrain_type: int = 3: set = _set_drivetrain_type
## Maximum achievable speed (km/h)
var max_speed_kmh: float = 320.0 get: return _max_speed_kmh
## Minimum achievable speed (m/s)
var min_speed_mps: float = 0.05 get: return _min_speed_mps

# ============================================================================
# PRIVATE STATE
# ============================================================================

var _current_speed: float = 0.0
var _engine_rpm: float = 0.0
var _target_gear: int = 0
var _gear_change_timer: float = 0.0
var _in_gear_shift: bool = false
var _throttle_input: float = 0.0
var _brake_input: float = 0.0
var _steering_input: float = 0.0
var _clutch_state: float = 1.0  # 1.0 = engaged, 0.0 = disengaged
var _total_mass: float = PhysicsSettings.default_vehicle_mass
var _acceleration_coefficient: float = 1.0
var _deceleration_coefficient: float = 1.0
var _steering_multiplier: float = 1.0
var _drivetrain_type: int = 3
var _max_speed_kmh: float = 320.0
var _min_speed_mps: float = 0.05

# Wheel forces array (4 wheels: FL, FR, RL, RR)
var _wheel_forces: Array[float] = [0.0, 0.0, 0.0, 0.0]
var _wheel_angles: Array[float] = [0.0, 0.0, 0.0, 0.0]
var _wheel_locks: Array[bool] = [false, false, false, false]

# Friction coefficients
var _road_friction: float = 0.85
var _gravel_friction: float = 0.55
var _ice_friction: float = 0.15
var _current_surface_friction: float = _road_friction

# Aerodynamic drag
var _drag_coefficient: float = 0.32
var _frontal_area: float = 2.2
var _air_density: float = 1.225

# ============================================================================
# LIFECYCLE
# ============================================================================

func _ready() -> void:
	_init_physics_properties()
	_connect_signals_to_manager()
	_update_max_speed()

func _physics_process(delta: float) -> void:
	_handle_inputs(delta)
	_update_gear_logic(delta)
	_apply_physics(delta)
	_update_aerodynamics(delta)
	_check_wheel_slip(delta)

# ============================================================================
# INPUT HANDLING
# ============================================================================

func _handle_inputs(delta: float) -> void:
	# Get normalized input values from InputManager
	_throttle_input = InputManager.get_axis(InputManager.THROTTLE)
	_brake_input = InputManager.get_axis(InputManager.BRAKE)
	_steering_input = InputManager.get_axis(InputManager.STEER) * steering_multiplier
	
	# Clamp inputs to valid range
	_throttle_input = clamp(_throttle_input, -1.0, 1.0)
	_brake_input = clamp(_brake_input, -1.0, 1.0)
	_steering_input = clamp(_steering_input, -1.0, 1.0)
	
	# Handle clutch (auto-clutch for manual transmission when shifting)
	if _in_gear_shift:
		_clutch_state = lerp(_clutch_state, 0.0, delta / SHIFT_DELAY_TIME)
	elif _throttle_input > 0.1 or _brake_input > 0.1:
		_clutch_state = lerp(_clutch_state, 1.0, delta / 0.1)
	
	# Calculate target gear based on RPM and input
	_target_gear = _calculate_target_gear()
	
	# Update visual/debug display
	Engine.debugger_send_notification("VehicleController", {
		"speed": current_speed,
		"rpm": engine_rpm,
		"gear": current_gear,
		"throttle": _throttle_input,
		"brake": _brake_input,
		"steering": _steering_input
	})

# ============================================================================
# GEAR LOGIC
# ============================================================================

func _update_gear_logic(delta: float) -> void:
	# Check if gear change is needed
	if _target_gear != _current_gear and not _in_gear_shift:
		_attempt_gear_shift(_target_gear)
	
	# Track gear shift timer
	if _in_gear_shift:
		_gear_change_timer += delta
		if _gear_change_timer >= SHIFT_DELAY_TIME:
			_complete_gear_shift()

func _calculate_target_gear() -> int:
	var target_gear: int = 1
	
	# Auto-gear logic based on RPM
	var rpm_threshold_high := PhysicsSettings.default_engine_rpm_redline * 0.95
	var rpm_threshold_low := PhysicsSettings.default_engine_rpm_idle * 1.2
	
	match _current_gear:
		0, 1, 2, 3, 4, 5, 6:
			# Upshift logic
			if _throttle_input < 0.1 and _engine_rpm < rpm_threshold_low:
				target_gear = _clamp_gear(_current_gear + 1)
		
		_:
			pass
	
	# Downshift logic
	if _throttle_input > 0.5 and _engine_rpm > rpm_threshold_high:
		target_gear = _clamp_gear(_current_gear - 1)
	
	# Reverse handling
	if _brake_input > 0.8 and _current_speed < 2.0:
		target_gear = 0  # Neutral first
		if _current_speed < -1.0:
			target_gear = 1  # Reverse
	
	return target_gear

func _clamp_gear(gear: int) -> int:
	if gear < 0:
		return 0  # Neutral
	elif gear > 6:
		return 6  # Max forward
	else:
		return gear

func _attempt_gear_shift(target_gear: int) -> void:
	if target_gear == _current_gear:
		return
	
	_in_gear_shift = true
	_gear_change_timer = 0.0
	
	# Signal the change
	emit_signal("gear_changed", _current_gear, target_gear)
	_current_gear = target_gear

func _complete_gear_shift() -> void:
	_in_gear_shift = false
	_gear_change_timer = 0.0
	_clutch_state = 1.0  # Re-engage clutch

# ============================================================================
# PHYSICS APPLICATION
# ============================================================================

func _apply_physics(delta: float) -> void:
	# Calculate gear ratio based on current gear
	var gear_ratio := GEAR_RATIOS[_current_gear]
	
	# Calculate drive force based on throttle and gear
	var drive_force: float = 0.0
	
	if _current_gear != 0:  # Not in neutral
		var torque: float = _calculate_engine_torque()
		drive_force = (torque * gear_ratio) / WHEEL_RADIUS
		
		# Apply drivetrain distribution
		_distribute_drive_force(drive_force)
	
	# Apply braking
	var brake_force: float = _calculate_brake_force()
	_apply_brake_force(brake_force)
	
	# Apply friction
	var friction_force: float = _calculate_friction_force()
	_apply_friction(friction_force)
	
	# Update velocity based on net forces
	_update_velocity(delta)
	
	# Move the body
	move_and_slide()

func _calculate_engine_torque() -> float:
	# Torque curve approximation based on RPM
	var normalized_rpm := _engine_rpm / PhysicsSettings.default_engine_rpm_redline
	var base_torque: float = PhysicsSettings.default_engine_max_torque
	
	# Simple torque curve (peak at ~60% redline)
	if normalized_rpm < 0.6:
		var torque_curve := 0.5 + 0.5 * sin(normalized_rpm * PI * 0.8)
		return base_torque * torque_curve
	else:
		var torque_curve := 1.0 - (normalized_rpm - 0.6) * 2.0
		return maxf(base_torque * torque_curve, 0.0)

func _distribute_drive_force(force: float) -> void:
	# Distribute force to wheels based on drivetrain type
	match _drivetrain_type:
		0:  # FR (Front Rear)
			_wheel_forces[0] = force * 0.4
			_wheel_forces[1] = force * 0.4
			_wheel_forces[2] = 0.0
			_wheel_forces[3] = force * 0.2
			
		1:  # RR (Rear Rear)
			_wheel_forces[0] = 0.0
			_wheel_forces[1] = 0.0
			_wheel_forces[2] = force * 0.5
			_wheel_forces[3] = force * 0.5
			
		2:  # FWD (Front Wheel Drive)
			_wheel_forces[0] = force * 0.6
			_wheel_forces[1] = force * 0.6
			_wheel_forces[2] = 0.0
			_wheel_forces[3] = 0.0
			
		3:  # RWD (Rear Wheel Drive)
			_wheel_forces[0] = 0.0
			_wheel_forces[1] = 0.0
			_wheel_forces[2] = force * 0.5
			_wheel_forces[3] = force * 0.5
			
		4:  # AWD (All Wheel Drive)
			_wheel_forces[0] = force * 0.35
			_wheel_forces[1] = force * 0.35
			_wheel_forces[2] = force * 0.15
			_wheel_forces[3] = force * 0.15
			
		_:  # Default to RWD
			_wheel_forces[0] = 0.0
			_wheel_forces[1] = 0.0
			_wheel_forces[2] = force * 0.5
			_wheel_forces[3] = force * 0.5

func _calculate_brake_force() -> float:
	var raw_brake: float = _brake_input * MAX_BRAKE_FORCE
	var effective_brake: float = raw_brake * _deceleration_coefficient
	
	# ABS simulation - prevent locking during hard braking
	if _brake_input > 0.8:
		for i in range(4):
			if _wheel_locks[i]:
				effective_brake *= 0.7  # Reduce brake force if wheel locked
	
	return effective_brake

func _apply_brake_force(force: float) -> void:
	# Apply brake force to all wheels
	for i in range(4):
		_wheel_forces[i] -= force * 0.25

func _calculate_friction_force() -> float:
	var normal_force: float = _total_mass * PhysicsSettings.gravity
	var friction: float = normal_force * _current_surface_friction
	
	# Speed-dependent friction reduction
	var speed_ratio: float = _current_speed / 30.0
	friction *= 1.0 - speed_ratio * 0.1
	
	return friction

func _apply_friction(force: float) -> void:
	# Apply friction opposite to movement direction
	if abs(_velocity.x) > 0.1 or abs(_velocity.z) > 0.1:
		var friction_dir: Vector3 = -_velocity.normalized()
		friction_dir.y = 0.0
		friction_dir = friction_dir.normalized()
		
		_velocity += friction_dir * force / _total_mass

func _update_velocity(delta: float) -> void:
	# Update speed magnitude
	var horizontal_velocity: Vector3 = Vector3(_velocity.x, 0.0, _velocity.z)
	_current_speed = horizontal_velocity.length()
	
	# Limit speed to maximum
	if _current_speed > _max_speed_kmh / 3.6:
		var limit_vector: Vector3 = horizontal_velocity.normalized() * (_max_speed_kmh / 3.6)
		_velocity.x = limit_vector.x
		_velocity.z = limit_vector.z
	
	# Clamp minimum speed
	if _current_speed < _min_speed_mps and _current_speed > 0:
		_current_speed = _min_speed_mps
	
	# Emit speed change signal
	if abs(_current_speed - speed_changed.emit(_current_speed)) > 0.1:
		speed_changed.emit(_current_speed)

# ============================================================================
# AERODYNAMICS
# ============================================================================

func _update_aerodynamics(delta: float) -> void:
	# Air resistance calculation: F = 0.5 * ρ * v² * Cd * A
	var velocity_squared: float = _current_speed * _current_speed
	var drag_force: float = 0.5 * _air_density * velocity_squared * _drag_coefficient * _frontal_area
	
	# Apply drag opposite to velocity
	if _current_speed > 0:
		var drag_dir: Vector3 = -_velocity.normalized()
		drag_dir.y = 0.0
		drag_dir = drag_dir.normalized()
		
		_velocity += drag_dir * drag_force / _total_mass * delta

# ============================================================================
# WHEEL SLIP DETECTION
# ============================================================================

func _check_wheel_slip(delta: float) -> void:
	# Detect wheel spin/sliding based on force vs traction
	var slip_threshold := PhysicsSettings.default_wheel_traction_coefficient
	
	for i in range(4):
		var wheel_force: float = abs(_wheel_forces[i])
		var normal_force: float = _total_mass * PhysicsSettings.gravity / 4.0
		var max_traction: float = normal_force * _current_surface_friction
		
		# Check if wheel is slipping
		if wheel_force > max_traction * slip_threshold:
			_wheel_locks[i] = true
			var slip_factor := (wheel_force - max_traction) / max_traction
			skid_detected.emit(slip_factor)
		else:
			_wheel_locks[i] = false

# ============================================================================
# UTILITY METHODS
# ============================================================================

func _init_physics_properties() -> void:
	"""Initialize vehicle physics properties from settings."""
	_total_mass = PhysicsSettings.default_vehicle_mass
	_drag_coefficient = PhysicsSettings.default_drag_coefficient
	_frontal_area = PhysicsSettings.default_frontal_area
	_road_friction = PhysicsSettings.default_road_friction
	_gravel_frraction = PhysicsSettings.default_gravel_friction
	_ice_friction = PhysicsSettings.default_ice_friction

func _connect_signals_to_manager() -> void:
	"""Connect to GameManager for state management."""
	GameManager.game_state_changed.connect(_on_game_state_changed)
	GameManager.race_started.connect(_on_race_started)
	GameManager.race_ended.connect(_on_race_ended)

func _on_game_state_changed(new_state: GameManager.GameState) -> void:
	match new_state:
		GameManager.GameState.RACE_ACTIVE:
			_reset_vehicle_state()
		GameManager.GameState.RACE_PAUSED:
			_pause_physics()
		_:
			pass

func _on_race_started(race_data: Dictionary) -> void:
	"""Handle race start - reset vehicle position and state."""
	_reset_vehicle_state()

func _on_race_ended(results: Dictionary) -> void:
	"""Handle race end - stop vehicle."""
	_velocity = Vector3.ZERO
	_current_speed = 0.0
	_current_gear = 0

func _reset_vehicle_state() -> void:
	"""Reset vehicle to initial state."""
	_velocity = Vector3.ZERO
	_current_speed = 0.0
	_engine_rpm = PhysicsSettings.default_engine_rpm_idle
	_current_gear = 0
	_target_gear = 0
	_in_gear_shift = false
	_gear_change_timer = 0.0
	_clutch_state = 1.0

func _pause_physics() -> void:
	"""Pause physics simulation."""
	_velocity = Vector3.ZERO
	_in_gear_shift = false

func _set_total_mass(value: float) -> void:
	_total_mass = maxf(value, PhysicsSettings.default_vehicle_mass * 0.5)
	_update_max_speed()

func _set_acceleration_coefficient(value: float) -> void:
	_acceleration_coefficient = clamp(value, 0.0, 2.0)

func _set_deceleration_coefficient(value: float) -> void:
	_deceleration_coefficient = clamp(value, 0.0, 2.0)

func _set_steering_multiplier(value: float) -> void:
	_steering_multiplier = clamp(value, 0.0, 3.0)

func _set_drivetrain_type(value: int) -> void:
	_drivetrain_type = clamp(value, 0, 4)

func _update_max_speed() -> void:
	"""Recalculate maximum speed based on gear ratios and engine characteristics."""
	var final_drive_ratio := PhysicsSettings.default_final_drive_ratio
	var top_gear_ratio := GEAR_RATIOS[6]
	
	var wheel_max_rpm := PhysicsSettings.default_engine_rpm_redline
	var wheel_max_speed := (wheel_max_rpm / 60.0) * (2.0 * PI * WHEEL_RADIUS)
	var gear_effective_ratio := top_gear_ratio * final_drive_ratio
	
	_max_speed_kmh = (wheel_max_speed / gear_effective_ratio) * 3.6

# ============================================================================
# DEBUG VISUALIZATION
# ============================================================================

func debug_visualize_wheels() -> void:
	"""Draw debug visualization of wheel forces and states."""
	if not GameManager.debug_mode:
		return
	
	for i in range(4):
		var color: Color = Color.WHITE
		if _wheel_locks[i]:
			color = Color.RED
		elif _wheel_forces[i] > 0:
			color = Color.GREEN
		elif _wheel_forces[i] < 0:
			color = Color.BLUE
		
		# Draw wheel force indicator (simple line)
		var force_vector := Vector3(0, 0, _wheel_forces[i] * 0.1)
		# Note: Actual visualization would use DebugDraw or similar

# ============================================================================
# EXPORTED API FOR EXTERNAL USE
# ============================================================================

func apply_manual_gear(gear: int) -> void:
	"""Manually set gear (for AI or external control)."""
	_target_gear = gear
	_attempt_gear_shift(gear)

func apply_throttle(amount: float) -> void:
	"""Apply throttle input directly (for AI or testing)."""
	_throttle_input = clamp(amount, -1.0, 1.0)

func apply_brake(amount: float) -> void:
	"""Apply brake input directly (for AI or testing)."""
	_brake_input = clamp(amount, -1.0, 1.0)

func apply_steering(amount: float) -> void:
	"""Apply steering input directly (for AI or testing)."""
	_steering_input = clamp(amount, -1.0, 1.0)

func set_surface_friction(surface_type: SurfaceType) -> void:
	"""Set surface friction based on terrain type."""
	match surface_type:
		SurfaceType.ROAD:
			_current_surface_friction = _road_friction
		SurfaceType.GRAVEL:
			_current_surface_friction = _gravel_friction
		SurfaceType.ICE:
			_current_surface_friction = _ice_friction
		SurfaceType.DIRT:
			_current_surface_friction = 0.65
		SurfaceType.WET_ROAD:
			_current_surface_friction = 0.60

enum SurfaceType {
	ROAD,
	GRAVEL,
	ICE,
	DIRT,
	WET_ROAD
}

</script>>