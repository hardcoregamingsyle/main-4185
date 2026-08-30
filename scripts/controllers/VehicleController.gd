extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulator
## Implements throttle, brake, steering inputs, wheel forces, and gear shifting logic
## This base class provides the foundation for all vehicle types in the game
## Copyright 2026 Thalamus Racing Simulator Project

# ============================================================================
# SIGNALS
# ============================================================================

signal throttle_applied(amount: float)
signal brake_applied(amount: float)
signal steering_input(angle: float)
signal gear_changed(old_gear: int, new_gear: int)
signal vehicle_spawned()
signal vehicle_destroyed()
signal lap_completed(lap_time: float)
signal checkpoint_reached(checkpoint_id: int)
signal damage_taken(damage_amount: float)
signal collision_detected(other_vehicle: Node)

# ============================================================================
# PHYSICS SETTINGS REFERENCE
# ============================================================================

@onready var physics_settings: PhysicsSettings = $"/root/PhysicsSettings"

# ============================================================================
# EXPORTED TUNABLE PROPERTIES
# ============================================================================

@export_group("Vehicle Configuration")
@export var vehicle_mass: float = 1500.0: set = _set_vehicle_mass
@export var center_of_mass_offset: Vector3 = Vector3(0.0, -0.3, 0.0)
@export var wheel_base: float = 2.8
@export var track_width: float = 1.6

@export_group("Throttle & Acceleration")
@export var max_throttle_force: float = 8000.0: set = _set_max_throttle_force
@export var min_throttle_force: float = 0.0
@export var throttle_curve: Curve = null
@export var acceleration_rate: float = 20.0

@export_group("Braking System")
@export var max_brake_force: float = 12000.0: set = _set_max_brake_force
@export var brake_pressure_ratio: float = 0.9
@export var brake_decay_rate: float = 15.0
@export var anti_lock_system: bool = true

@export_group("Steering Dynamics")
@export var max_steering_angle: float = PI / 3  # 60 degrees
@export var steering_sensitivity: float = 1.0
@export var steering_response_time: float = 0.15
@export var steering_curve: Curve = null
@export var lock_to_lock_turns: int = 2.5

@export_group("Gearbox Settings")
@export var gearbox_type: GearType = GearType.AUTO
@export var num_gears: int = 6
@export var gear_ratios: Array[float] = [3.5, 2.2, 1.6, 1.2, 0.9, 0.7]
@export var reverse_gear_ratio: float = -3.8
@export var final_drive_ratio: float = 3.2
@export var auto_shift_rpm_threshold: float = 6500.0
@export var manual_downshift_rpm_threshold: float = 2000.0

@export_group("Traction Control")
@export var traction_control_enabled: bool = true
@export var traction_limit: float = 0.85
@export var differential_type: DiffType = DiffType.LSD
@export var diff_lock_percentage: float = 0.3

@export_group("Aerodynamics")
@export var drag_coefficient: float = 0.32
@export var frontal_area: float = 2.4
@export var downforce_coefficient: float = 0.5
@export var wing_angle: float = 0.0

# ============================================================================
# ENUMERATIONS
# ============================================================================

enum GearType {
	AUTO,
	MANUAL,
	SEQUENTIAL
}

enum DiffType {
	OPEN,
	LSD,
	CLOSED
}

enum BrakingMode {
	STANDARD,
	PARKING,
	EMERGENCY
}

# ============================================================================
# PRIVATE STATE VARIABLES
# ============================================================================

var _current_gear: int = 0  # 0 = neutral, 1-6 = forward gears, -1 = reverse
var _target_gear: int = 0
var _engine_rpm: float = 0.0
var _idle_rpm: float = 800.0
var _redline_rpm: float = 7000.0
var _max_engine_rpm: float = 7500.0

var _throttle_input: float = 0.0  # 0.0 to 1.0
var _brake_input: float = 0.0     # 0.0 to 1.0
var _steering_input: float = 0.0  # -1.0 to 1.0

var _current_steering_angle: float = 0.0
var _target_steering_angle: float = 0.0

var _vehicle_speed: float = 0.0  # meters per second
var _forward_velocity: Vector3 = Vector3.ZERO
var _lateral_velocity: Vector3 = Vector3.ZERO

var _total_distance_traveled: float = 0.0
var _total_laps_completed: int = 0
var _current_lap_time: float = 0.0
var _lap_start_time: float = 0.0

var _is_spinning: bool = false
var _spin_recovery_timer: float = 0.0
var _damage_level: float = 0.0
var _health: float = 100.0

var _last_frame_time: float = 0.0

# ============================================================================
# INITIALIZATION
# ============================================================================

func _ready() -> void:
	_process_mode = ProcessModeEnum.PHYSICS
	
	# Apply center of mass
	rigid_body.mass = vehicle_mass
	center_of_mass = center_of_mass_offset
	
	# Initialize physics properties
	_reset_vehicle_state()
	
	# Connect to global signals
	GameManager.race_started.connect(_on_race_started)
	GameManager.game_state_changed.connect(_on_game_state_changed)
	InputManager.throttle_changed.connect(_on_throttle_changed)
	InputManager.brake_changed.connect(_on_brake_changed)
	InputManager.steering_changed.connect(_on_steering_changed)
	
	# Signal readiness
	vehicle_spawned.emit()

func _physics_process(delta: float) -> void:
	_last_frame_time = delta
	
	# Update vehicle dynamics
	_update_physics(delta)
	
	# Handle gear shifting
	_handle_gear_shifting(delta)
	
	# Apply engine forces
	_apply_engine_forces(delta)
	
	# Apply braking forces
	_apply_braking(delta)
	
	# Calculate steering response
	_calculate_steering_response(delta)
	
	# Update movement state
	_update_movement_state(delta)
	
	# Handle spin recovery
	_handle_spin_recovery(delta)
	
	# Check for collisions
	_check_collisions()
	
	# Emit update signals
	_emit_vehicle_signals(delta)

# ============================================================================
# CORE PHYSICS METHODS
# ============================================================================

func _update_physics(delta: float) -> void:
	"""Update all vehicle physics calculations"""
	
	# Get gravity from settings
	var gravity := physics_settings.gravity * physics_settings.time_scale
	
	# Apply gravity if not grounded
	if not is_on_floor():
		velocity.y -= gravity * delta * physics_settings.time_scale
	
	# Update speed from velocity
	_forward_velocity = velocity.transform(basis).x.normalized()
	_vehicle_speed = velocity.length()
	
	# Track distance traveled
	if _vehicle_speed > 0.1:
		_total_distance_traveled += _vehicle_speed * delta

func _apply_engine_forces(delta: float) -> void:
	"""Apply engine torque and thrust based on throttle input"""
	
	if _current_gear == 0:  # Neutral or parking
		return
	
	# Calculate gear ratio
	var gear_ratio: float
	if _current_gear < 0:
		gear_ratio = reverse_gear_ratio
	else:
		gear_ratio = gear_ratios[min(_current_gear - 1, num_gears - 1)]
	
	# Calculate total transmission ratio
	var total_ratio = gear_ratio * final_drive_ratio
	
	# Calculate engine power curve (simplified)
	var engine_power_factor := _calculate_engine_power_factor()
	
	# Calculate wheel torque
	var wheel_torque = engine_power_factor * max_throttle_force * total_ratio
	
	# Apply traction control
	if traction_control_enabled:
		wheel_torque *= _apply_traction_control(wheel_torque)
	
	# Distribute force to wheels
	var drive_wheels := _get_drive_wheel_positions()
	for wheel_pos in drive_wheels:
		var local_dir := transform.x.normalized()
		force_line = wheel_pos + Vector3.UP * 0.3
		add_force(linear_interpolate(force_line, local_dir * wheel_torque, 0.0))
	
	# Update RPM based on speed and gear
	_update_engine_rpm(delta)

func _apply_braking(delta: float) -> void:
	"""Apply braking forces based on brake input"""
	
	if _brake_input <= 0.0:
		_current_brake_pressure = 0.0
		return
	
	var brake_pressure: float
	
	match braking_mode:
		BrakingMode.STANDARD:
			brake_pressure = _brake_input * max_brake_force * brake_pressure_ratio
		BrakingMode.PARKING:
			brake_pressure = max_brake_force * 0.5
		BrakingMode.EMERGENCY:
			brake_pressure = max_brake_force
	
	# Anti-lock system prevents wheel lockup
	if anti_lock_system and _is_wheel_locking():
		brake_pressure *= 0.7
	
	# Decay brake pressure when releasing
	if _brake_input == 0.0 and _current_brake_pressure > 0.0:
		_current_brake_pressure = max(0.0, _current_brake_pressure - brake_decay_rate * delta)
		return
	
	_current_brake_pressure = lerp(_current_brake_pressure, brake_pressure, delta * 10.0)
	
	# Apply brake force to all wheels
	var wheel_positions := _get_all_wheel_positions()
	for wheel_pos in wheel_positions:
		var local_dir := transform.x.normalized() * -1.0
		add_force_at_position(local_dir * _current_brake_pressure * 0.25, wheel_pos)

func _calculate_steering_response(delta: float) -> void:
	"""Calculate smooth steering angle transition"""
	
	if _current_gear == 0:  # No steering in neutral
		_target_steering_angle = 0.0
		_current_steering_angle = lerp(_current_steering_angle, 0.0, delta * 10.0)
		return
	
	# Apply steering sensitivity
	var target_angle := _steering_input * max_steering_angle * steering_sensitivity
	
	# Apply steering curve if defined
	if steering_curve != null:
		target_angle = steering_curve.sample_baked(abs(target_angle)) * sign(target_angle)
	
	# Smooth steering transition
	_target_steering_angle = lerp(_target_steering_angle, target_angle, delta / steering_response_time)
	_current_steering_angle = lerp(_current_steering_angle, _target_steering_angle, delta * 5.0)
	
	# Clamp to maximum
	_current_steering_angle = clamp(_current_steering_angle, -max_steering_angle, max_steering_angle)
	
	# Apply steering rotation to front wheels
	_rotate_front_wheels(_current_steering_angle)

func _handle_gear_shifting(delta: float) -> void:
	"""Handle automatic or manual gear shifting logic"""
	
	match gearbox_type:
		GearType.AUTO:
			_auto_shift_gears(delta)
		GearType.MANUAL:
			_manual_shift_gears(delta)
		GearType.SEQUENTIAL:
			_sequential_shift_gears(delta)
	
	# Ensure gear stays within bounds
	_current_gear = clamp(_current_gear, -1, num_gears)
	_target_gear = clamp(_target_gear, -1, num_gears)
	
	# Sync current to target after shift delay
	if abs(_current_gear - _target_gear) <= 1:
		_current_gear = _target_gear

func _update_engine_rpm(delta: float) -> void:
	"""Update engine RPM based on vehicle speed and gear"""
	
	if _current_gear == 0:  # Neutral
		_engine_rpm = lerp(_engine_rpm, _idle_rpm, delta * 2.0)
		return
	
	# Calculate theoretical RPM based on wheel speed
	var wheel_radius: float = 0.3  # Assume standard tire radius
	var wheel_speed := _vehicle_speed / wheel_radius
	var gear_ratio: float
	if _current_gear < 0:
		gear_ratio = reverse_gear_ratio
	else:
		gear_ratio = gear_ratios[_current_gear - 1]
	
	var theoretical_rpm := wheel_speed * gear_ratio * final_drive_ratio * 60.0 / (2.0 * PI)
	
	# Engine inertia and torque delivery
	var rpm_target := theoretical_rpm
	if _throttle_input > 0.0:
		rpm_target = lerp(rpm_target, _redline_rpm, delta * acceleration_rate)
	else:
		rpm_target = lerp(rpm_target, _idle_rpm, delta * 5.0)
	
	_engine_rpm = clamp(rpm_target, _idle_rpm, _max_engine_rpm)

# ============================================================================
# INPUT HANDLERS
# ============================================================================

func _on_throttle_changed(amount: float) -> void:
	_throttle_input = clamp(amount, 0.0, 1.0)
	throttle_applied.emit(_throttle_input)

func _on_brake_changed(amount: float) -> void:
	_brake_input = clamp(amount, 0.0, 1.0)
	brake_applied.emit(_brake_input)

func _on_steering_changed(amount: float) -> void:
	_steering_input = clamp(amount, -1.0, 1.0)
	steering_input.emit(_steering_input)

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================

func _auto_shift_gears(delta: float) -> void:
	"""Automatic gear shifting based on RPM and speed"""
	
	if _current_gear == 0 and _throttle_input > 0.0:
		_target_gear = 1
		return
	
	if _engine_rpm >= auto_shift_rpm_threshold and _target_gear < num_gears:
		_target_gear += 1
	elif _engine_rpm < manual_downshift_rpm_threshold and _target_gear > 1:
		_target_gear -= 1
	elif _vehicle_speed < 5.0 and _target_gear > 1:
		_target_gear = 1

func _manual_shift_gears(delta: float) -> void:
	"""Manual gear shifting via player input"""
	pass  # Will be overridden by specific vehicle implementations

func _sequential_shift_gears(delta: float) -> void:
	"""Sequential gearbox with up/down shift commands"""
	pass  # Will be overridden by specific vehicle implementations

func shift_gear(gear_number: int) -> void:
	"""Manually shift to a specific gear"""
	var old_gear := _current_gear
	_target_gear = gear_number
	
	if gear_number != old_gear:
		gear_changed.emit(old_gear, gear_number)

func shift_up() -> void:
	"""Shift one gear up"""
	if _target_gear < num_gears:
		_target_gear += 1
		gear_changed.emit(_current_gear, _target_gear)

func shift_down() -> void:
	"""Shift one gear down"""
	if _target_gear > -1:
		_target_gear -= 1
		gear_changed.emit(_current_gear, _target_gear)

# ============================================================================
# TRACTION & DIFFERENTIAL CONTROL
# ============================================================================

func _apply_traction_control(torque: float) -> float:
	"""Apply traction control to limit wheel slip"""
	
	var wheel_slip := _calculate_wheel_slip()
	if wheel_slip > traction_limit:
		return (1.0 - (wheel_slip - traction_limit)) * 0.9
	return 1.0

func _calculate_wheel_slip() -> float:
	"""Calculate current wheel slip percentage"""
	var drive_force := _get_drive_force()
	var normal_force := _get_normal_force()
	
	if normal_force <= 0.0:
		return 0.0
	
	return drive_force / normal_force

func _get_diff_lock_effectiveness() -> float:
	"""Get differential lock effectiveness based on type"""
	
	match differential_type:
		DiffType.OPEN:
			return 0.0
		DiffType.LSD:
			return diff_lock_percentage
		DiffType.CLOSED:
			return 1.0
	
	return 0.0

# ============================================================================
# AERODYNAMICS & DOWNFORCE
# ============================================================================

func _calculate_aerodynamic_downforce() -> float:
	"""Calculate downforce from aerodynamics"""
	
	var air_density: float = 1.225  # kg/m³ at sea level
	var speed_squared := _vehicle_speed * _vehicle_speed
	
	var downforce = 0.5 * air_density * speed_squared * frontal_area * downforce_coefficient
	
	# Add wing contribution
	var wing_downforce = 0.5 * air_density * speed_squared * frontal_area * \
		downforce_coefficient * sin(wing_angle)
	
	return downforce + wing_downforce

func _calculate_drag_force() -> float:
	"""Calculate air resistance/drag"""
	
	var air_density: float = 1.225
	var speed_squared := _vehicle_speed * _vehicle_speed
	
	return 0.5 * air_density * speed_squared * frontal_area * drag_coefficient

# ============================================================================
# DAMAGE & COLLISION
# ============================================================================

func take_damage(damage_amount: float) -> void:
	"""Apply damage to vehicle"""
	_damage_level = min(1.0, _damage_level + damage_amount * 0.1)
	_health = max(0.0, _health - damage_amount)
	damage_taken.emit(damage_amount)
	
	if _health <= 0.0:
		vehicle_destroyed.emit()

func get_health_percentage() -> float:
	return _health

func get_damage_level() -> float:
	return _damage_level

func reset_damage() -> void:
	_damage_level = 0.0
	_health = 100.0

# ============================================================================
# LAP TIMING & TRACK DATA
# ============================================================================

func start_lap_timing() -> void:
	_lap_start_time = Time.get_unix_time_from_system()
	_current_lap_time = 0.0

func update_lap_time() -> void:
	if _lap_start_time > 0.0:
		_current_lap_time = Time.get_unix_time_from_system() - _lap_start_time

func complete_lap() -> void:
	_total_laps_completed += 1
	lap_completed.emit(_current_lap_time)
	start_lap_timing()

func add_checkpoint(checkpoint_id: int) -> void:
	checkpoint_reached.emit(checkpoint_id)

func get_current_lap_time() -> float:
	return _current_lap_time

func get_total_laps_completed() -> int:
	return _total_laps_completed

# ============================================================================
# VEHICLE STATE MANAGEMENT
# ============================================================================

func _reset_vehicle_state() -> void:
	"""Reset all vehicle state variables"""
	
	_current_gear = 0
	_target_gear = 0
	_engine_rpm = _idle_rpm
	_throttle_input = 0.0
	_brake_input = 0.0
	_steering_input = 0.0
	_current_steering_angle = 0.0
	_target_steering_angle = 0.0
	_vehicle_speed = 0.0
	_is_spinning = false
	_damage_level = 0.0
	_health = 100.0
	_total_distance_traveled = 0.0
	_total_laps_completed = 0.0
	_current_lap_time = 0.0
	_lap_start_time = 0.0
	_spin_recovery_timer = 0.0

func recover_from_spin() -> void:
	"""Initiate spin recovery procedure"""
	_is_spinning = false
	_spin_recovery_timer = 0.0
	_current_steering_angle = 0.0
	_steering_input = 0.0

# ============================================================================
# UTILITY METHODS
# ============================================================================

func get_vehicle_speed_kmh() -> float:
	return _vehicle_speed * 3.6

func get_vehicle_speed_mph() -> float:
	return _vehicle_speed * 2.237

func get_forward_velocity() -> Vector3:
	return _forward_velocity

func get_velocity_vector() -> Vector3:
	return velocity

func get_engine_rpm() -> float:
	return _engine_rpm

func get_current_gear() -> int:
	return _current_gear

func get_throttle_input() -> float:
	return _throttle_input

func get_brake_input() -> float:
	return _brake_input

func get_steering_angle() -> float:
	return _current_steering_angle

func is_in_reverse() -> bool:
	return _current_gear < 0

func is_on_track() -> bool:
	"""Override this method to check if vehicle is on valid track area"""
	return true

func get_total_distance() -> float:
	return _total_distance_traveled

# ============================================================================
# HELPER METHODS FOR WHEELS & FORCES
# ============================================================================

func _get_all_wheel_positions() -> Array[Vector3]:
	"""Get positions of all four wheels in world space"""
	var offset_z := track_width / 2.0
	var offset_y := 0.3  # Wheel height above ground
	var offset_x := wheel_base / 2.0
	
	return [
		transform * Vector3(-offset_x, offset_y, -offset_z),  # Front Left
		transform * Vector3(-offset_x, offset_y, offset_z),   # Front Right
		transform * Vector3(offset_x, offset_y, -offset_z),   # Rear Left
		transform * Vector3(offset_x, offset_y, offset_z)    # Rear Right
	]

func _get_drive_wheel_positions() -> Array[Vector3]:
	"""Get positions of driven wheels based on drivetrain type"""
	# Simplified: assume rear-wheel drive for now
	var offset_z := track_width / 2.0
	var offset_y := 0.3
	var offset_x := wheel_base / 2.0
	
	return [
		transform * Vector3(offset_x, offset_y, -offset_z),   # Rear Left
		transform * Vector3(offset_x, offset_y, offset_z)    # Rear Right
	]

func _rotate_front_wheels(angle: float) -> void:
	"""Rotate front wheels by specified angle (override for actual wheel meshes)"""
	pass

func _is_wheel_locking() -> bool:
	"""Check if any wheel is locking during braking"""
	return _vehicle_speed < 5.0 and _brake_input > 0.8

# ============================================================================
# GAME STATE INTEGRATION
# ============================================================================

func _on_race_started(race_data: Dictionary) -> void:
	"""Handle race start event"""
	_reset_vehicle_state()
	start_lap_timing()

func _on_game_state_changed(new_state: GameState) -> void:
	"""Handle game state changes"""
	match new_state:
		GameState.RACE_ACTIVE:
			_reset_vehicle_state()
			start_lap_timing()
		GameState.RACE_PAUSED:
			_process_mode = ProcessModeEnum.DISABLED
		GameState.RACE_FINISHED:
			_process_mode = ProcessModeEnum.ALWAYS

# ============================================================================
# EXTENSIBILITY HOOKS
# ============================================================================

func _calculate_engine_power_factor() -> float:
	"""Override for custom power curves"""
	# Default: linear interpolation between idle and redline
	var rpm_range := _max_engine_rpm - _idle_rpm
	var current_rpm_range := _engine_rpm - _idle_rpm
	return current_rpm_range / rpm_range

func _get_drive_force() -> float:
	"""Override for custom drive force calculation"""
	return _throttle_input * max_throttle_force

func _get_normal_force() -> float:
	"""Override for custom normal force calculation"""
	return vehicle_mass * physics_settings.gravity

# ============================================================================
# DESTRUCTOR
# ============================================================================

func _exit_tree() -> void:
	vehicle_destroyed.emit()

</script>