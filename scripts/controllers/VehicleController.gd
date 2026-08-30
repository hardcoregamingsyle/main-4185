extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulator
## Implements throttle, brake, steering inputs, wheel forces, and gear shifting logic
## Uses PhysicsSettings constants for centralized tuning
## Copyright 2026 Thalamus Racing Simulator Project

# ============================================================================
# SIGNALS
# ============================================================================
signal speed_changed(new_speed: float)
signal rpm_changed(new_rpm: float)
signal gear_changed(old_gear: int, new_gear: int)
signal throttle_applied(amount: float)
signal brake_applied(amount: float)
signal steering_angle_changed(angle: float)
signal skidding(is_skidding: bool)
signal collision_detected(collision_info: Dictionary)
signal engine_stalled()
signal handbrake_toggled(is_active: bool)
signal traction_control_state_changed(active: bool)
signal anti_lock_braking_state_changed(active: bool)
signal drift_started(drift_angle: float)
signal drift_ended()
signal vehicle_launched(launch_data: Dictionary)

# ============================================================================
# CONFIGURATION CONSTANTS
# ============================================================================
const MAX_SPEED_KMH: float = 350.0
const ACCELERATION_POWER: float = 20000.0
const BRAKING_FORCE: float = 40000.0
const STEERING_SPEED: float = 2.5
const MAX_STEERING_ANGLE: float = 35.0 * TAU / 180.0
const MIN_GEAR: int = -1  # Reverse
const MAX_GEAR: int = 6
const NEUTRAL_GEAR: int = 0
const IDLE_RPM: float = 800.0
const REDLINE_RPM: float = 8000.0
const CLUTCH_RELEASE_TIME: float = 0.3
const DIFFERENTIAL_LOCK_RATIO: float = 0.8
const DRIFT_FACTOR: float = 0.15
const TRACTION_CONTROL_WEIGHT: float = 0.7
const ABS_MODULATION_RATE: float = 10.0

# ============================================================================
# PRIVATE STATE VARIABLES
# ============================================================================
var _physics_settings: PhysicsSettings = null
var _vehicle_mesh: Node3D = null
var _powertrain: Powertrain = null
var _suspension_nodes: Array[Node3D] = []
var _wheel_colliders: Array[Area3D] = []

# Current state
var current_speed: float = 0.0
var current_rpm: float = IDLE_RPM
var current_gear: int = NEUTRAL_GEAR
var target_gear: int = NEUTRAL_GEAR
var clutch_engaged: bool = false
var clutch_progress: float = 0.0

# Input values (normalized -1 to 1)
var throttle_input: float = 0.0
var brake_input: float = 0.0
var steering_input: float = 0.0
var handbrake_input: bool = false

# Advanced systems
var traction_control_enabled: bool = true
var abs_enabled: bool = true
var active_drift_angle: float = 0.0
var is_skidding: bool = false
var is_on_ground: bool = true

# Suspension and tire state
var suspension_compression: Vector3 = Vector3.ZERO
var tire_slip_ratios: Vector3 = Vector3.ZERO
var lateral_g_force: float = 0.0
var longitudinal_g_force: float = 0.0

# Gear shift timing
var last_shift_time: float = 0.0
var shift_delay: float = 0.5
var upshift_rpm_threshold: float = REDLINE_RPM * 0.95
var downshift_rpm_threshold: float = IDLE_RPM * 1.3

# Drift mechanics
var drift_score: float = 0.0
var drift_multiplier: float = 1.0
var drift_target_angle: float = 0.0

# Performance metrics
var max_speed_achieved: float = 0.0
var acceleration_0_100: float = 0.0
var braking_distance: float = 0.0
var lap_times: Array[float] = []

# Collision and damage tracking
var total_collision_impact: float = 0.0
var vehicle_damage_level: float = 0.0
var repair_needed: bool = false

# Engine state
var engine_temperature: float = 90.0
var oil_pressure: float = 4.0
var fuel_consumption_rate: float = 0.0
var fuel_level: float = 100.0

# ============================================================================
# PUBLIC PROPERTIES
# ============================================================================
func get_current_speed() -> float:
	return current_speed

func get_current_rpm() -> float:
	return current_rpm

func get_current_gear() -> int:
	return current_gear

func get_steering_angle() -> float:
	return steering_input * MAX_STEERING_ANGLE

func is_engine_running() -> bool:
	return current_rpm > IDLE_RPM

func get_fuel_level() -> float:
	return fuel_level

func get_vehicle_health() -> float:
	return 1.0 - vehicle_damage_level

func get_performance_metrics() -> Dictionary:
	return {
		"max_speed": max_speed_achieved,
		"acceleration_0_100": acceleration_0_100,
		"braking_distance": braking_distance,
		"fuel_remaining": fuel_level,
		"damage_level": vehicle_damage_level
	}

# ============================================================================
# INITIALIZATION
# ============================================================================
func _ready() -> void:
	_physics_settings = PhysicsSettings.get_singleton() if has_node("/root/PhysicsSettings") else load("res://scripts/core/PhysicsSettings.gd").new()
	
	if not _physics_settings:
		print("[VehicleController] WARNING: PhysicsSettings singleton not found!")
		return
	
	_init_components()
	_connect_signals()
	_reset_vehicle_state()

func _init_components() -> void:
	_find_child_nodes()
	_setup_suspension()

func _find_child_nodes() -> void:
	_vehicle_mesh = find_child("VehicleMesh", true)
	_powertrain = find_child("Powertrain", true) as Powertrain
	
	if _powertrain:
		_powertrain.rpm_changed.connect(_on_powertrain_rpm_changed)

func _setup_suspension() -> void:
	_suspension_nodes.clear()
	_wheel_colliders.clear()
	
	for child in get_children():
		if child.has_method("_is_wheel"):
			var wheel = child as Node3D
			if wheel.get_parent() == self:
				_suspension_nodes.append(wheel)
				var area = wheel.find_child("WheelCollider", true) as Area3D
				if area:
					_wheel_colliders.append(area)

func _connect_signals() -> void:
	InputManager.input_updated.connect(_on_input_updated)
	GameManager.game_state_changed.connect(_on_game_state_changed)

func _reset_vehicle_state() -> void:
	current_speed = 0.0
	current_rpm = IDLE_RPM
	current_gear = NEUTRAL_GEAR
	target_gear = NEUTRAL_GEAR
	clutch_engaged = false
	clutch_progress = 0.0
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	handbrake_input = false
	traction_control_enabled = true
	abs_enabled = true
	active_drift_angle = 0.0
	is_skidding = false
	suspension_compression = Vector3.ZERO
	tire_slip_ratios = Vector3.ZERO
	lateral_g_force = 0.0
	longitudinal_g_force = 0.0
	last_shift_time = 0.0
	is_on_ground = true
	max_speed_achieved = 0.0
	engine_temperature = 90.0
	oil_pressure = 4.0
	fuel_level = 100.0
	total_collision_impact = 0.0
	vehicle_damage_level = 0.0
	repair_needed = false

# ============================================================================
# PHYSICS UPDATE LOOP
# ============================================================================
func _physics_process(delta: float) -> void:
	if not is_inside_tree():
		return
	
	if GameManager.current_state != GameManager.GameState.RACE_ACTIVE:
		return
	
	update_engine_state(delta)
	update_transmission(delta)
	process_inputs(delta)
	calculate_physics(delta)
	update_visuals(delta)

# ============================================================================
# ENGINE MANAGEMENT
# ============================================================================
func update_engine_state(delta: float) -> void:
	var engine_effort = calculate_engine_effort()
	var friction_loss = calculate_friction_loss()
	
	# RPM calculation based on gear and speed
	var gear_ratio = get_gear_ratio(current_gear)
	var wheel_radius = 0.3
	var wheel_angular_velocity = current_speed / wheel_radius
	var engine_target_rpm = wheel_angular_velocity * gear_ratio * 0.1
	
	# Smooth RPM transition
	var rpm_change = engine_target_rpm - current_rpm
	var rpm_damping = 1.0 + engine_effort * 0.5
	current_rpm += rpm_change * delta * rpm_damping
	
	# Clamp RPM
	current_rpm = clamp(current_rpm, IDLE_RPM, REDLINE_RPM)
	
	# Update engine temperature
	var temp_change = (engine_effort - 0.5) * delta * 2.0
	engine_temperature += temp_change
	engine_temperature = clamp(engine_temperature, 60.0, 120.0)
	
	# Oil pressure simulation
	oil_pressure = 4.0 + (current_rpm / REDLINE_RPM) * 2.0
	
	# Fuel consumption
	if current_rpm > IDLE_RPM:
		fuel_consumption_rate = 10.0 + engine_effort * 20.0
		fuel_level -= fuel_consumption_rate * delta / 3600.0
		fuel_level = max(fuel_level, 0.0)
		
		if fuel_level <= 0.0:
			_handle_out_of_fuel()
	
	# Emit signals
	rpm_changed.emit(current_rpm)
	speed_changed.emit(current_speed)

func calculate_engine_effort() -> float:
	if clutch_engaged:
		return throttle_input
	else:
		return 0.0

func calculate_friction_loss() -> float:
	var air_resistance = current_speed * current_speed * 0.0001
	var rolling_resistance = current_speed * 0.001
	return air_resistance + rolling_resistance

# ============================================================================
# TRANSMISSION & GEAR SHIFTING
# ============================================================================
func update_transmission(delta: float) -> void:
	# Handle clutch engagement
	if not clutch_engaged:
		clutch_progress += delta / CLUTCH_RELEASE_TIME
		if clutch_progress >= 1.0:
			clutch_engaged = true
			clutch_progress = 1.0
	else:
		clutch_progress = 1.0
	
	# Automatic gear shifting
	if _should_auto_shift():
		_auto_shift_gear()
	
	# Manual gear shifting
	if target_gear != current_gear and _can_shift_gear():
		_execute_gear_shift(target_gear)

func get_gear_ratio(gear: int) -> float:
	match gear:
		MIN_GEAR: return 3.5
		-1: return 3.5
		0: return 0.0
		1: return 3.5
		2: return 2.0
		3: return 1.5
		4: return 1.2
		5: return 1.0
		6: return 0.8
		_: return 0.8
	return 1.0

func _should_auto_shift() -> bool:
	if current_gear == NEUTRAL_GEAR:
		return false
	
	if current_rpm >= upshift_rpm_threshold and current_gear < MAX_GEAR:
		return true
	
	if current_rpm <= downshift_rpm_threshold and current_gear > MIN_GEAR:
		return true
	
	return false

func _auto_shift_gear() -> void:
	var next_gear = MIN_GEAR
	if current_rpm >= upshift_rpm_threshold and current_gear < MAX_GEAR:
		next_gear = current_gear + 1
	elif current_rpm <= downshift_rpm_threshold and current_gear > MIN_GEAR:
		next_gear = current_gear - 1
	
	if next_gear != current_gear:
		target_gear = next_gear

func _can_shift_gear() -> bool:
	var time_since_last_shift = Time.get_ticks_msec() / 1000.0 - last_shift_time
	return time_since_last_shift >= shift_delay and not (throttle_input > 0.9 and brake_input > 0.9)

func _execute_gear_shift(new_gear: int) -> void:
	var old_gear = current_gear
	current_gear = new_gear
	
	if old_gear != new_gear:
		gear_changed.emit(old_gear, new_gear)
		last_shift_time = Time.get_ticks_msec() / 1000.0
		
		# Temporary RPM drop on upshift
		if new_gear > old_gear:
			current_rpm *= 0.7
			current_rpm = max(current_rpm, IDLE_RPM)
		
		# Temporary RPM spike on downshift
		if new_gear < old_gear:
			current_rpm *= 1.3
			current_rpm = min(current_rpm, REDLINE_RPM)

# ============================================================================
# INPUT PROCESSING
# ============================================================================
func process_inputs(delta: float) -> void:
	# Steering input
	var target_steering = steering_input * MAX_STEERING_ANGLE
	var steering_smooth = lerp(lerp_angle(get_steering_angle(), target_steering, delta * STEERING_SPEED), 
		target_steering, delta * STEERING_SPEED)
	steering_angle_changed.emit(steering_smooth)
	
	# Throttle input
	if throttle_input != 0.0:
		throttle_applied.emit(throttle_input)
	
	# Brake input
	if brake_input != 0.0:
		brake_applied.emit(brake_input)
	
	# Handbrake handling
	handbrake_toggled.emit(handbrake_input)
	
	# Traction control toggle
	if Input.is_action_just_pressed("toggle_traction_control"):
		traction_control_enabled = !traction_control_enabled
		traction_control_state_changed.emit(traction_control_enabled)
	
	# ABS toggle
	if Input.is_action_just_pressed("toggle_abs"):
		abs_enabled = !abs_enabled
		anti_lock_braking_state_changed.emit(abs_enabled)

func _on_input_updated(input_data: Dictionary) -> void:
	throttle_input = input_data.get("throttle", 0.0)
	brake_input = input_data.get("brake", 0.0)
	steering_input = input_data.get("steering", 0.0)
	handbrake_input = input_data.get("handbrake", false)

# ============================================================================
# VEHICLE PHYSICS CALCULATION
# ============================================================================
func calculate_physics(delta: float) -> void:
	if not is_on_ground:
		_apply_air_resistance(delta)
		return
	
	# Calculate forward force
	var drive_force = _calculate_drive_force()
	var braking_force = _calculate_braking_force()
	var steering_force = _apply_steering()
	
	# Apply forces to velocity
	var forward_axis = transform.basis.z
	var right_axis = transform.basis.x
	
	# Longitudinal forces
	var net_longitudinal = drive_force - braking_force
	longitudinal_g_force = net_longitudinal / _get_vehicle_mass()
	current_speed += net_longitudinal * delta / _get_vehicle_mass()
	
	# Lateral forces (for drifting)
	var lateral_acceleration = steering_force * lateral_g_force
	current_velocity += lateral_acceleration * right_axis * delta
	
	# Friction and drag
	_apply_drag_and_friction(delta)
	
	# Update position
	move_and_slide()
	
	# Check ground contact
	is_on_ground = _check_ground_contact()
	
	# Update performance metrics
	_update_performance_metrics()

func _calculate_drive_force() -> float:
	if current_gear == NEUTRAL_GEAR or not clutch_engaged:
		return 0.0
	
	var gear_ratio = get_gear_ratio(current_gear)
	var engine_torque = calculate_engine_torque()
	var final_drive_ratio = 3.5
	var wheel_radius = 0.3
	
	var drive_force = engine_torque * gear_ratio * final_drive_ratio / wheel_radius
	
	# Apply traction control
	if traction_control_enabled:
		drive_force *= _apply_traction_control(drive_force)
	
	return drive_force

func calculate_engine_torque() -> float:
	var normalized_rpm = current_rpm / REDLINE_RPM
	# Simulate torque curve
	var base_torque = 400.0
	var torque_curve = base_torque * (1.0 - abs(normalized_rpm - 0.6) * 2.0)
	return torque_curve * throttle_input

func _calculate_braking_force() -> float:
	if brake_input <= 0.0:
		return 0.0
	
	var base_braking = BRAKING_FORCE * brake_input
	
	# ABS modulation
	if abs_enabled and is_skidding:
		base_braking *= 0.8
	
	# Handbrake adds extra braking
	if handbrake_input:
		base_braking *= 1.5
	
	return base_braking

func _apply_steering() -> float:
	var max_lateral_g = 1.2
	var actual_steering = get_steering_angle()
	lateral_g_force = actual_steering * max_lateral_g
	return lateral_g_force

func _apply_drag_and_friction(delta: float) -> void:
	var air_drag = current_speed * current_speed * 0.0002
	var rolling_resistance = current_speed * 0.005
	
	# Reduce speed by drag
	current_speed -= (air_drag + rolling_resistance) * delta
	
	# Prevent negative speed when reversing
	if current_speed < 0.0:
		current_speed = max(current_speed, -MAX_SPEED_KMH / 3.6)

func _apply_air_resistance(delta: float) -> void:
	var drag = current_speed * current_speed * 0.001
	current_speed -= drag * delta
	current_speed = max(current_speed, 0.0)

func _check_ground_contact() -> bool:
	if is_on_floor():
		return true
	return false

# ============================================================================
# PERFORMANCE METRICS
# ============================================================================
func _update_performance_metrics() -> void:
	var speed_kmh = current_speed * 3.6
	
	# Track max speed
	if speed_kmh > max_speed_achieved:
		max_speed_achieved = speed_kmh
	
	# Acceleration 0-100 km/h
	if speed_kmh >= 100.0 and acceleration_0_100 == 0.0:
		acceleration_0_100 = Time.get_unix_time_from_system()

func _handle_out_of_fuel() -> void:
	print("[VehicleController] Out of fuel! Engine will stall.")
	current_rpm = IDLE_RPM
	throttle_input = 0.0
	vehicle_destroyed.emit(self)

# ============================================================================
# VISUAL UPDATES
# ============================================================================
func update_visuals(delta: float) -> void:
	if _vehicle_mesh:
		_update_suspension_visuals(delta)
		_update_wheels_visuals(delta)

func _update_suspension_visuals(delta: float) -> void:
	var compression_factor = (current_speed * 0.01) + (longitudinal_g_force * 0.1)
	suspension_compression.y = clamp(compression_factor, 0.0, 0.5)

func _update_wheels_visuals(delta: float) -> void:
	for wheel in _suspension_nodes:
		var wheel_spin = current_speed * delta * 3.0
		wheel.rotation.y += wheel_spin

# ============================================================================
# ADVANCED SYSTEMS
# ============================================================================
func _apply_traction_control(drive_force: float) -> float:
	if not traction_control_enabled:
		return 1.0
	
	var slip_ratio = abs(tire_slip_ratios.x)
	if slip_ratio > 0.1:
		return 1.0 - (slip_ratio - 0.1) * 0.5
	return 1.0

func start_drift() -> void:
	if not is_on_ground:
		return
	
	if abs(steering_input) > 0.5 and abs(current_speed) > 50.0 / 3.6:
		active_drift_angle = PI / 4
		is_skidding = true
		skidding.emit(true)
		drift_started.emit(active_drift_angle)
		
		# Apply drift physics
		var drift_force = current_speed * DRIFT_FACTOR
		current_velocity += drift_force * transform.basis.x.normalized()

func end_drift() -> void:
	if is_skidding:
		is_skidding = false
		active_drift_angle = 0.0
		skidding.emit(false)
		drift_ended.emit()

func handle_collision(collision_data: Dictionary) -> void:
	var impact_force = collision_data.get("impact", 0.0)
	total_collision_impact += impact_force
	
	if impact_force > 5000.0:
		vehicle_damage_level += impact_force / 50000.0
		vehicle_damage_level = min(vehicle_damage_level, 1.0)
		
		if vehicle_damage_level >= 0.5:
			repair_needed = true
		
		collision_detected.emit({
			"impact": impact_force,
			"damage": vehicle_damage_level,
			"location": collision_data.get("location", Vector3.ZERO)
		})
		
		# Screen shake effect
		_trigger_screen_shake(impact_force / 1000.0)

func _trigger_screen_shake(intensity: float) -> void:
	# This would trigger visual effects
	pass

# ============================================================================
# GAME STATE HANDLERS
# ============================================================================
func _on_game_state_changed(new_state: GameState) -> void:
	match new_state:
		GameManager.GameState.MAIN_MENU:
			_reset_vehicle_state()
		GameManager.GameState.RACE_ACTIVE:
			_reset_vehicle_state()
			_start_race_sequence()
		GameManager.GameState.RACE_PAUSED:
			pass
		GameManager.GameState.RACE_FINISHED:
			_save_lap_data()

func _start_race_sequence() -> void:
	# Pre-race checks
	if fuel_level < 10.0:
		print("[VehicleController] Warning: Low fuel!")
	
	if vehicle_damage_level > 0.3:
		print("[VehicleController] Warning: Vehicle damaged!")
	
	# Ready vehicle
	current_rpm = IDLE_RPM
	current_gear = NEUTRAL_GEAR
	clutch_engaged = false

func _save_lap_data() -> void:
	var lap_time = Time.get_unix_time_from_system()
	lap_times.append(lap_time)
	print("[VehicleController] Lap data saved: %d laps" % lap_times.size())

# ============================================================================
# HELPER FUNCTIONS
# ============================================================================
func shift_up() -> void:
	if current_gear < MAX_GEAR:
		target_gear = current_gear + 1

func shift_down() -> void:
	if current_gear > MIN_GEAR:
		target_gear = current_gear - 1

func engage_clutch() -> void:
	clutch_engaged = false
	clutch_progress = 0.0

func release_clutch() -> void:
	engage_clutch()
	# Will be handled by update_transmission

func reset_vehicle() -> void:
	_reset_vehicle_state()
	position = Vector3.ZERO
	rotation = Vector3.ZERO
	velocity = Vector3.ZERO

func get_debug_info() -> Dictionary:
	return {
		"speed_kmh": current_speed * 3.6,
		"rpm": current_rpm,
		"gear": current_gear,
		"throttle": throttle_input,
		"brake": brake_input,
		"steering": steering_input,
		"handbrake": handbrake_input,
		"fuel": fuel_level,
		"temperature": engine_temperature,
		"damage": vehicle_damage_level,
		"skidding": is_skidding,
		"ground": is_on_ground
	}

# ============================================================================
# DEBUGGING & TESTING
# ============================================================================
func test_acceleration() -> void:
	throttle_input = 1.0
	await get_tree().create_timer(5.0).timeout
	throttle_input = 0.0
	print("[Test] Acceleration test complete. Max speed: %.2f km/h" % (max_speed_achieved * 3.6))

func test_braking() -> void:
	var test_speed = 100.0 / 3.6  # Convert to m/s
	velocity.x = test_speed
	brake_input = 1.0
	await get_tree().create_timer(3.0).timeout
	brake_input = 0.0
	print("[Test] Braking test complete. Final speed: %.2f m/s" % current_speed)

func test_steering() -> void:
	steering_input = 1.0
	await get_tree().create_timer(2.0).timeout
	steering_input = -1.0
	await get_tree().create_timer(2.0).timeout
	steering_input = 0.0
	print("[Test] Steering test complete")

# ============================================================================
# DESTRUCTOR
# ============================================================================
func _exit_tree() -> void:
	# Cleanup resources
	if _powertrain:
		_powertrain.disconnect("rpm_changed", Callable(self, "_on_powertrain_rpm_changed"))
	if _physics_settings:
		_physics_settings.free()

func _on_powertrain_rpm_changed(rpm: float) -> void:
	current_rpm = rpm

func _on_powertrain_gear_changed(gear: int) -> void:
	target_gear = gear

</FILE>