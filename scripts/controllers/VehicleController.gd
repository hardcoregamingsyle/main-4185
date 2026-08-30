extends Node
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulator
## Handles throttle, brake, steering, gear shifting, wheel forces, and collision detection
## Integrates with PhysicsSettings, InputManager, Powertrain, and AudioManager systems
## Copyright 2026 Thalamus Racing Simulator Project

signal speed_changed(new_speed: float)
signal gear_changed(old_gear: int, new_gear: int)
signal vehicle_damage_taken(damage_amount: float)
signal vehicle_destroyed(vehicle_node: Node)
signal lap_completed(lap_time: float)
signal checkpoint_reached(checkpoint_id: int)
signal race_position_updated(position: int)

# ============================================================================
# CONSTANTS & CONFIGURATION (from PhysicsSettings)
# ============================================================================

@onready var physics_settings = preload("res://scripts/core/PhysicsSettings.gd").new()

# Vehicle mass and inertia (from PhysicsSettings defaults)
var _vehicle_mass: float = physics_settings.default_vehicle_mass
var _inertia_tensor: Vector3 = Vector3(1.0, 1.0, 1.0)

# Wheel configuration
const NUM_WHEELS: int = 4
const WHEEL_TRACK_WIDTH: float = 1.5
const WHEEL_BASE_LENGTH: float = 2.8
const WHEEL_RADIUS: float = 0.35

# Max speeds per gear (m/s)
const MAX_SPEED_PER_GEAR: Array[float] = [
	15.0,   # Gear 1
	28.0,   # Gear 2
	45.0,   # Gear 3
	65.0,   # Gear 4
	90.0,   # Gear 5
	130.0   # Gear 6 / Overdrive
]

# Engine RPM limits
const MIN_RPM: float = 800.0
const IDLE_RPM: float = 1000.0
const MAX_RPM: float = 7500.0
const REDLINE_RPM: float = 8000.0
const OPTIMAL_RPM: float = 5500.0

# Torque curve (normalized 0.0-1.0 RPM ratio -> torque multiplier)
const TORQUE_CURVE: Array[float] = [
	0.45, 0.52, 0.60, 0.68, 0.75, 0.82, 0.88, 0.92, 0.95, 0.97,
	0.98, 0.97, 0.95, 0.92, 0.88, 0.82, 0.75, 0.68, 0.60, 0.52
]

# Braking force (N)
const MAX_BRAKE_FORCE: float = 25000.0
const ABS_THRESHOLD: float = 0.85  # Slip ratio threshold for ABS activation

# Steering parameters
const MAX_STEER_ANGLE: float = 35.0 * deg_to_rad
const STEER_SMOOTHING: float = 10.0
const STEER_RECOVERY_SPEED: float = 5.0

# Collision and damage
const COLLISION_DMG_PER_MPS: float = 150.0
const CRITICAL_DAMAGE_THRESHOLD: float = 5000.0
const TOTAL_HEALTH: float = 10000.0

# ============================================================================
# STATE VARIABLES
# ============================================================================

var current_speed: float = 0.0  # Speed in m/s
var current_rpm: float = IDLE_RPM
var current_gear: int = 0  # 0 = Neutral, 1-6 = Gears
var clutch_engaged: bool = false
var health: float = TOTAL_HEALTH

# Input states (updated from InputManager)
var throttle_input: float = 0.0  # 0.0 to 1.0
var brake_input: float = 0.0    # 0.0 to 1.0
var steering_input: float = 0.0 # -1.0 to 1.0
var shift_up_requested: bool = false
var shift_down_requested: bool = false
var handbrake_active: bool = false

# Internal simulation state
var _wheel_rpm: Array[float] = [0.0, 0.0, 0.0, 0.0]  # Front-L, Front-R, Rear-L, Rear-R
var _wheel_slip_ratio: Array[float] = [0.0, 0.0, 0.0, 0.0]
var _current_steer_angle: float = 0.0
var _target_steer_angle: float = 0.0
var _acceleration: float = 0.0
var _deceleration: float = 0.0
var _force_applied: float = 0.0
var _is_locked: bool = false
var _is_floating: bool = false
var _last_checkpoint: int = -1

# Track/race state
var _race_distance: float = 0.0
var _lap_time_accumulator: float = 0.0
var _start_time: float = 0.0
var _is_in_race: bool = false
var _position_tracker: Dictionary = {}

# Audio references
var _audio_manager: Node = null
var _powertrain_node: Node = null

# Debug
@export var debug_enabled: bool = false

# ============================================================================
# INITIALIZATION
# ============================================================================

func _ready() -> void:
	_init_dependencies()
	_setup_wheels()
	_reset_vehicle_state()
	
	if debug_enabled:
		print("[VehicleController] Initialized with mass:", _vehicle_mass)

func _init_dependencies() -> void:
	"""Initialize references to required systems."""
	if GameManager.has_singleton():
		_audio_manager = get_node_or_null("/root/AudioManager")
		_powertrain_node = get_node_or_null("../Powertrain") if has_parent() else null

func _setup_wheels() -> void:
	"""Set up wheel properties for each of the 4 wheels."""
	for i in range(NUM_WHEELS):
		_wheel_rpm[i] = 0.0
		_wheel_slip_ratio[i] = 0.0

func _reset_vehicle_state() -> void:
	"""Reset vehicle to initial state."""
	current_speed = 0.0
	current_rpm = IDLE_RPM
	current_gear = 0
	clutch_engaged = false
	health = TOTAL_HEALTH
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	shift_up_requested = false
	shift_down_requested = false
	handbrake_active = false
	_wheel_rpm.fill(0.0)
	_wheel_slip_ratio.fill(0.0)
	_current_steer_angle = 0.0
	_target_steer_angle = 0.0
	_acceleration = 0.0
	_deceleration = 0.0
	_force_applied = 0.0
	_is_locked = false
	_is_floating = false
	_last_checkpoint = -1
	_race_distance = 0.0
	_lap_time_accumulator = 0.0
	_start_time = 0.0
	_is_in_race = false

# ============================================================================
# MAIN GAME LOOP
# ============================================================================

func _physics_process(delta: float) -> void:
	"""Main physics update loop - runs at fixed timestep."""
	if delta <= 0.0:
		return
	
	_update_input_states()
	_apply_physics(delta)
	_update_simulation(delta)
	_check_collisions(delta)
	_handle_gameplay_events(delta)
	
	if debug_enabled:
		_log_debug_state(delta)

func _update_input_states() -> void:
	"""Read input states from InputManager singleton."""
	var input_manager = GameManager.get_singleton("InputManager")
	if input_manager == null:
		return
	
	# Read normalized input values (clamped to valid ranges)
	throttle_input = clamp(input_manager.get_axis("throttle"), 0.0, 1.0)
	brake_input = clamp(input_manager.get_axis("brake"), 0.0, 1.0)
	steering_input = clamp(input_manager.get_axis("steer_left_right"), -1.0, 1.0)
	
	# Check gear shift requests (debounced)
	if input_manager.is_action_just_pressed("gear_shift_up"):
		shift_up_requested = true
	if input_manager.is_action_just_pressed("gear_shift_down"):
		shift_down_requested = true
	
	# Check handbrake toggle
	handbrake_active = input_manager.is_action_pressed("handbrake")

# ============================================================================
# PHYSICS ENGINE
# ============================================================================

func _apply_physics(delta: float) -> void:
	"""Apply forces and calculate accelerations based on current state."""
	if _is_locked or _is_floating:
		return
	
	var engine_torque = _calculate_engine_torque()
	var drivetrain_efficiency = 0.85  # 15% loss in transmission
	
	# Calculate drive force based on gear and RPM
	var gear_ratio = _get_gear_ratio(current_gear)
	var final_drive_ratio = 3.73  # Final drive ratio
	var effective_ratio = gear_ratio * final_drive_ratio
	
	# Wheel angular velocity from vehicle linear velocity
	var wheel_circumference = 2.0 * PI * WHEEL_RADIUS
	var wheel_linear_speed = current_speed
	var theoretical_wheel_rpm = (wheel_linear_speed * 60.0) / (wheel_circumference)
	
	# Apply engine torque to driven wheels (RWD setup)
	var driven_wheels = [2, 3]  # Rear wheels (index 2, 3)
	for wheel_idx in driven_wheels:
		var torque_at_wheel = engine_torque * effective_ratio * drivetrain_efficiency
		var force = torque_at_wheel / WHEEL_RADIUS
		_force_applied += force
	
	# Calculate acceleration (F = ma)
	if _force_applied > 0:
		_acceleration = _force_applied / _vehicle_mass
	else:
		_acceleration = 0.0
	
	# Apply braking force
	var brake_force = brake_input * MAX_BRAKE_FORCE
	if handbrake_active:
		brake_force *= 1.5  # Handbrake bonus
	_brake_force_actual = min(brake_force, MAX_BRAKE_FORCE)
	
	# Air resistance and rolling resistance
	var air_resistance = 0.5 * 1.2 * pow(current_speed, 2) * 2.2 * 0.32  # Cd*A*rho*v^2
	var rolling_resistance = _vehicle_mass * 9.81 * 0.015  # Crr*m*g
	var total_drag = air_resistance + rolling_resistance
	
	# Net deceleration
	var total_decel_force = _brake_force_actual + total_drag
	_deceleration = total_decel_force / _vehicle_mass

func _calculate_engine_torque() -> float:
	"""Calculate torque output based on current RPM and throttle."""
	if current_gear == 0:  # Neutral
		return 0.0
	
	# Clamp RPM to valid range
	var clamped_rpm = clamp(current_rpm, MIN_RPM, MAX_RPM)
	
	# Get torque multiplier from curve (interpolate)
	var rpm_ratio = (clamped_rpm - MIN_RPM) / (MAX_RPM - MIN_RPM)
	var torque_curve_index = floor(rpm_ratio * (TORQUE_CURVE.size() - 1))
	torque_curve_index = clamp(torque_curve_index, 0, TORQUE_CURVE.size() - 2)
	var next_index = torque_curve_index + 1
	var t = (rpm_ratio * (TORQUE_CURVE.size() - 1)) - torque_curve_index
	var torque_multiplier = lerp(TORQUE_CURVE[int(torque_curve_index)], 
		TORQUE_CURVE[int(next_index)], t)
	
	# Base engine torque (Nm)
	var base_torque = 450.0  # High-performance engine
	
	# Apply throttle modifier
	var actual_torque = base_torque * torque_multiplier * throttle_input
	
	# Rev limiter effect (if over redline, reduce torque)
	if current_rpm > REDLINE_RPM:
		actual_torque *= 0.3  # Severely reduced torque when rev-limited
	
	return actual_torque

func _get_gear_ratio(gear: int) -> float:
	"""Get gear ratio for specified gear."""
	match gear:
		1: return 3.50
		2: return 2.10
		3: return 1.50
		4: return 1.10
		5: return 0.85
		6: return 0.65
		_: return 1.0  # Neutral or reverse
	return 1.0

func _update_simulation(delta: float) -> void:
	"""Update vehicle simulation state."""
	# Update speed with acceleration/deceleration
	var net_acceleration = _acceleration - _deceleration
	var new_speed = current_speed + net_acceleration * delta
	
	# Apply speed limits based on current gear
	var max_speed = MAX_SPEED_PER_GEAR[current_gear] if current_gear > 0 else 0.0
	
	# Gradual speed limiting (don't snap abruptly)
	if new_speed > max_speed:
		new_speed = max_speed * 0.99  # Slow down gradually
	elif new_speed < 0:
		new_speed = 0.0
	
	# Update speed
	current_speed = new_speed
	
	# Update RPM based on gear ratio and speed
	var wheel_circumference = 2.0 * PI * WHEEL_RADIUS
	var wheel_linear_rps = abs(current_speed) / wheel_circumference
	var gear_ratio = _get_gear_ratio(current_gear)
	var final_drive_ratio = 3.73
	var effective_ratio = gear_ratio * final_drive_ratio
	
	if clutch_engaged and current_gear > 0 and current_speed > 0:
		# Engine RPM follows wheel RPM through transmission
		var target_rpm = wheel_linear_rps * effective_ratio * 60.0  # Convert to RPM
		# Smooth RPM transition
		current_rpm = lerp(current_rpm, target_rpm, delta * 5.0)
	else:
		# Engine idles or coasts
		if throttle_input > 0:
			current_rpm = lerp(current_rpm, current_rpm + 100.0 * delta, delta * 10.0)
		else:
			current_rpm = lerp(current_rpm, IDLE_RPM, delta * 3.0)
	
	# Clamp RPM
	current_rpm = clamp(current_rpm, MIN_RPM, MAX_RPM)
	
	# Update wheel RPMs
	var wheel_rps = wheel_linear_rps
	for i in range(NUM_WHEELS):
		if i < 2:  # Front wheels (not driven)
			_wheel_rpm[i] = wheel_rps * 60.0
		else:  # Rear wheels (driven)
			_wheel_rpm[i] = wheel_rps * effective_ratio * 60.0
	
	# Calculate slip ratios
	for i in range(NUM_WHEELS):
		var wheel_linear_speed = _wheel_rpm[i] / 60.0 * wheel_circumference
		var slip = 0.0
		if wheel_linear_speed > 0:
			slip = (abs(wheel_linear_speed) - abs(current_speed)) / abs(current_speed)
		_wheel_slip_ratio[i] = slip
	
	# Update distance traveled
	_race_distance += current_speed * delta
	
	# Emit signals
	if abs(current_speed - _last_speed) > 0.1:
		speed_changed.emit(current_speed)
	
	_last_speed = current_speed

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================

func shift_gear(target_gear: int) -> bool:
	"""Request a gear shift. Returns true if successful."""
	if target_gear < 0 or target_gear > 6:
		return false
	
	if target_gear == current_gear:
		return false
	
	# Check if shift is possible (RPM must be in acceptable range)
	if not _can_shift(current_gear, target_gear):
		return false
	
	# Perform shift with clutch disengagement
	clutch_engaged = false
	
	# Delay clutch re-engagement for realism
	await get_tree().create_timer(0.1).timeout
	
	# Execute gear change
	var old_gear = current_gear
	current_gear = target_gear
	clutch_engaged = true
	
	gear_changed.emit(old_gear, current_gear)
	
	# Play shift sound
	_play_shift_sound()
	
	return true

func _can_shift(from_gear: int, to_gear: int) -> bool:
	"""Check if a gear shift is possible given current conditions."""
	# Can always shift into neutral
	if to_gear == 0:
		return true
	
	# Forward shifts require minimum RPM
	if to_gear > from_gear:
		return current_rpm >= IDLE_RPM
	
	# Downshifts require maximum RPM below limit
	if to_gear < from_gear:
		return current_rpm <= MAX_RPM * 0.9
	
	return false

func _handle_gear_shifting_inputs() -> void:
	"""Process automatic gear shifting based on inputs."""
	if shift_up_requested:
		if current_gear < 6:
			shift_gear(current_gear + 1)
		shift_up_requested = false
	
	if shift_down_requested:
		if current_gear > 1:
			shift_gear(current_gear - 1)
		shift_down_requested = false

# ============================================================================
# STEERING SYSTEM
# ============================================================================

func _update_steering(delta: float) -> void:
	"""Update steering angle with smoothing."""
	if _is_locked or _is_floating:
		return
	
	# Target angle from input
	_target_steer_angle = steering_input * MAX_STEER_ANGLE
	
	# Smoothly interpolate current angle toward target
	_current_steer_angle = lerp(_current_steer_angle, _target_steer_angle, delta * STEER_SMOOTHING)
	
	# Handle steering recovery when no input
	if abs(steering_input) < 0.05:
		_current_steer_angle = lerp(_current_steer_angle, 0.0, delta * STEER_RECOVERY_SPEED)

# ============================================================================
# COLLISION DETECTION
# ============================================================================

func _check_collisions(delta: float) -> void:
	"""Check for collisions and apply appropriate responses."""
	# This would integrate with Godot's collision system
	# For now, implement basic AABB-based collision checks
	
	var vehicle_bounds = _get_vehicle_bounds()
	
	# Check against track boundaries (simplified)
	var position = get_global_position()
	var x = position.x
	var z = position.z
	
	# Example: Track width check (expand this for real track data)
	if x < -15.0 or x > 15.0:  # Assuming 30m wide track
		_handle_boundary_collision()
	
	# Check against other vehicles (would use actual collision shape)
	# This is a simplified version - real implementation uses CollisionShape nodes
	pass

func _get_vehicle_bounds() -> Rect3:
	"""Get vehicle bounding box in world coordinates."""
	var pos = get_global_position()
	var half_width = WHEEL_TRACK_WIDTH / 2.0
	var half_length = WHEEL_BASE_LENGTH / 2.0
	var height = 1.2
	
	return Rect3(
		pos - Vector3(half_width, 0, half_length),
		Vector3(WHEEL_TRACK_WIDTH, height * 2, WHEEL_BASE_LENGTH)
	)

func _handle_boundary_collision() -> void:
	"""Handle collision with track boundaries."""
	# Reduce speed significantly on boundary hit
	var speed_reduction = 0.5
	current_speed *= speed_reduction
	
	# Bounce back slightly
	var direction = Vector3(-sign(get_global_position().x), 0, 0)
	add_impulse(direction * 5.0)
	
	# Damage calculation
	var damage = _calculate_collision_damage(current_speed)
	apply_damage(damage)
	
	# Visual feedback
	_trigger_screen_shake(0.2)
	_trigger_hit_flash()
	
	# Sound effect
	_play_collision_sound()

func _calculate_collision_damage(speed: float) -> float:
	"""Calculate damage based on impact speed."""
	var impact_damage = abs(speed) * COLLISION_DMG_PER_MPS
	return min(impact_damage, TOTAL_HEALTH * 0.5)  # Cap at 50% health

# ============================================================================
# DAMAGE SYSTEM
# ============================================================================

func apply_damage(amount: float) -> bool:
	"""Apply damage to vehicle. Returns true if vehicle is destroyed."""
	health -= amount
	
	if health <= 0:
		health = 0
		destroy_vehicle()
		return true
	
	if health <= CRITICAL_DAMAGE_THRESHOLD:
		# Critical damage warning
		_trigger_critical_warning()
	
	violence_damage_taken.emit(amount)
	return false

func destroy_vehicle() -> void:
	"""Mark vehicle as destroyed."""
	is_alive = false
	vehicle_destroyed.emit(self)
	
	# Stop simulation
	_is_locked = true
	_is_floating = true
	
	# Play crash sound
	_play_crash_sound()

func heal(amount: float) -> void:
	"""Repair vehicle by healing damaged health."""
	health = min(health + amount, TOTAL_HEALTH)

# ============================================================================
# RACE SYSTEM INTEGRATION
# ============================================================================

func start_race() -> void:
	"""Initialize vehicle for race participation."""
	_is_in_race = true
	_start_time = Time.get_ticks_msec() / 1000.0
	_lap_time_accumulator = 0.0
	_last_checkpoint = -1
	
	if debug_enabled:
		print("[VehicleController] Race started")

func end_race() -> void:
	"""Finalize race results."""
	_is_in_race = false
	var finish_time = Time.get_ticks_msec() / 1000.0 - _start_time
	
	race_finished.emit({
		"distance": _race_distance,
		"time": finish_time,
		"final_speed": current_speed,
		"peak_rpm": _peak_rpm_during_race
	})

func record_checkpoint(checkpoint_id: int) -> void:
	"""Record passing through a checkpoint."""
	if checkpoint_id > _last_checkpoint:
		_last_checkpoint = checkpoint_id
		checkpoint_reached.emit(checkpoint_id)
		
		# Increment lap count if completing full circuit
		if checkpoint_id == 0 and _last_checkpoint == 0:
			_complete_lap()

func _complete_lap() -> void:
	"""Handle lap completion."""
	var lap_time = Time.get_ticks_msec() / 1000.0 - _start_time
	lap_completed.emit(lap_time)
	_start_time = Time.get_ticks_msec() / 1000.0  # Reset for next lap

func update_position_tracking(other_vehicles: Array[Node]) -> void:
	"""Track position relative to other vehicles."""
	var my_distance = _race_distance
	var positions = []
	
	for vehicle in other_vehicles:
		var vc = vehicle.find_child("VehicleController", true, false)
		if vc and vc.has_method("get_race_distance"):
			positions.append({"node": vehicle, "distance": vc.get_race_distance()})
	
	# Sort by distance
	positions.sort_custom(func(a, b): return a["distance"] < b["distance"])
	
	# Find our position
	var my_pos = 1
	for p in positions:
		if p["node"] != self:
			my_pos += 1
		else:
			break
	
	position = my_pos
	race_position_updated.emit(my_pos)

func get_race_distance() -> float:
	"""Get total race distance traveled."""
	return _race_distance

# ============================================================================
# AUDIO INTEGRATION
# ============================================================================

func _play_shift_sound() -> void:
	"""Play gear shift sound effect."""
	if _audio_manager == null:
		return
	
	_audio_manager.play_sfx("gear_shift_" + str(current_gear))

func _play_collision_sound() -> void:
	"""Play collision impact sound."""
	if _audio_manager == null:
		return
	
	_audio_manager.play_sfx("collision_impact")

func _play_crash_sound() -> void:
	"""Play vehicle crash sound."""
	if _audio_manager == null:
		return
	
	_audio_manager.play_sfx("crash_explosion")

func _play_engine_idle() -> void:
	"""Play engine idle loop sound."""
	if _audio_manager == null:
		return
	
	_audio_manager.play_looping_sfx("engine_idle")

func _play_engine_rev() -> void:
	"""Play engine revving sound."""
	if _audio_manager == null:
		return
	
	_audio_manager.play_sfx("engine_rev")

# ============================================================================
# VISUAL EFFECTS (Game Feel)
# ============================================================================

func _trigger_screen_shake(duration: float, intensity: float = 1.0) -> void:
	"""Trigger screen shake effect."""
	# Would integrate with camera shake system
	# For now, mark that shake should occur
	_should_shake = true
	_shake_duration = duration
	_shake_intensity = intensity

func _trigger_hit_flash() -> void:
	"""Trigger flash effect on impact."""
	# Would trigger UI flash or screen overlay
	_should_flash = true
	_flash_duration = 0.15

func _trigger_critical_warning() -> void:
	"""Trigger critical damage warning."""
	# Would show warning indicator on HUD
	_should_warn = true

# ============================================================================
# POWERTRAIN INTEGRATION
# ============================================================================

func _update_powertrain() -> void:
	"""Update connected Powertrain node with current state."""
	if _powertrain_node == null:
		return
	
	# Send state updates to powertrain
	if _powertrain_node.has_method("_set_throttle"):
		_powertrain_node._set_throttle(throttle_input)
	if _powertrain_node.has_method("_set_brake"):
		_powertrain_node._set_brake(brake_input)
	if _powertrain_node.has_method("_set_gear"):
		_powertrain_node._set_gear(current_gear)
	if _powertrain_node.has_method("_set_rpm"):
		_powertrain_node._set_rpm(current_rpm)

# ============================================================================
# INPUT HANDLING (Direct Actions)
# ============================================================================

func set_throttle(value: float) -> void:
	"""Manually set throttle input (for AI or external control)."""
	throttle_input = clamp(value, 0.0, 1.0)

func set_brake(value: float) -> void:
	"""Manually set brake input (for AI or external control)."""
	brake_input = clamp(value, 0.0, 1.0)

func set_steering(value: float) -> void:
	"""Manually set steering input (for AI or external control)."""
	steering_input = clamp(value, -1.0, 1.0)

func set_gear_direct(gear: int) -> void:
	"""Set gear directly (bypasses shift validation)."""
	if gear >= 0 and gear <= 6:
		var old_gear = current_gear
		current_gear = gear
		gear_changed.emit(old_gear, current_gear)

func engage_clutch() -> void:
	"""Engage clutch."""
	clutch_engaged = true

func disengage_clutch() -> void:
	"""Disengage clutch."""
	clutch_engaged = false

# ============================================================================
# DEBUG & UTILITIES
# ============================================================================

func _log_debug_state(delta: float) -> void:
	"""Log current vehicle state for debugging."""
	print("[VC_DEBUG] Speed: ", str(round(current_speed * 3.6)) + " km/h | " +
		  "RPM: ", str(round(current_rpm)) + " | " +
		  "Gear: ", str(current_gear) + " | " +
		  "Throttle: ", str(round(throttle_input, 2)) + " | " +
		  "Steering: ", str(round(steering_input, 2)))

func get_vehicle_status() -> Dictionary:
	"""Get current vehicle status as dictionary."""
	return {
		"speed_kmh": round(current_speed * 3.6),
		"rpm": round(current_rpm),
		"gear": current_gear,
		"health_percent": round((health / TOTAL_HEALTH) * 100),
		"throttle": round(throttle_input, 2),
		"brake": round(brake_input, 2),
		"steering": round(steering_input, 2),
		"distance": round(_race_distance, 2),
		"is_alive": is_alive,
		"in_race": _is_in_race
	}

func reset_for_new_race() -> void:
	"""Reset vehicle for a new race session."""
	_reset_vehicle_state()
	health = TOTAL_HEALTH
	_race_distance = 0.0
	_lap_time_accumulator = 0.0

# ============================================================================
# SIGNALS
# ============================================================================

signal race_finished(results: Dictionary)
signal vehicle_death()
signal _should_shake(shake_data: Dictionary)
signal _should_flash(flash_data: Dictionary)
signal _should_warn(warn_data: Dictionary)

# ============================================================================
# EXPORTED PROPERTIES FOR EDITOR
# ============================================================================

@export_group("Vehicle Configuration")
@export var vehicle_name: String = "Race Car"
@export var team_color: Color = Color.WHITE
@export var car_number: int = 1

@export_group("Performance Tuning")
@export var engine_type: String = "V8"
@export var drivetrain_type: String = "RWD"
@export var tire_compound: String = "Soft"

# ============================================================================
# HELPER METHODS
# ============================================================================

func normalize_vector(vec: Vector3) -> Vector3:
	"""Normalize a 3D vector."""
	var length = vec.length()
	if length > 0:
		return vec.normalized()
	return Vector3.ZERO

func clamp_velocity_limit(limit: float) -> void:
	"""Clamp current velocity to specified limit."""
	if abs(current_speed) > limit:
		current_speed = sign(current_speed) * limit

func apply_force(force_vector: Vector3) -> void:
	"""Apply a force vector to the vehicle."""
	# In a real physics system, this would call rigidbody.apply_central_impulse
	pass

func apply_torque(torque_vector: Vector3) -> void:
	"""Apply a torque vector to the vehicle."""
	# In a real physics system, this would call rigidbody.apply_torque_impulse
	pass

func get_forward_direction() -> Vector3:
	"""Get the vehicle's forward direction."""
	return transform.basis.z

func get_right_direction() -> Vector3:
	"""Get the vehicle's right direction."""
	return transform.basis.x

func get_up_direction() -> Vector3:
	"""Get the vehicle's up direction."""
	return transform.basis.y

</FILE>