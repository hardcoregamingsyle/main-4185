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

# ============================================================================
# CONSTANTS & CONFIGURATION
# ============================================================================
const WHEEL_BASE: float = 2.5
const TRACK_WIDTH: float = 1.8
const MAX_STEERING_ANGLE: float = deg_to_rad(45.0)
const STEER_SPEED: float = 2.5
const BRAKE_FORCE: float = 15000.0
const PARKING_BRAKE_FORCE: float = 8000.0

# Gear ratios (final drive included)
const GEAR_RATIOS: Array[float] = [
	5.25,  # Reverse
	3.50,  # 1st
	2.75,  # 2nd
	2.00,  # 3rd
	1.50,  # 4th
	1.15,  # 5th
	0.90   # 6th
]
const FINAL_DRIVE: float = 3.73
const TRANSMISSION_GEAR_COUNT: int = 7  # R + 6 forward

# RPM limits
const IDLE_RPM: float = 800.0
const REDLINE_RPM: float = 7500.0
const SHIFT_POINT_RPM: float = 7200.0

# ============================================================================
# EXPORTED PROPERTIES
# ============================================================================
@export_group("Vehicle Configuration")
@export var max_power: float = 300.0: set = _set_max_power
@export var torque_curve: Curve: set = _set_torque_curve
@export var weight_distribution_front: float = 0.45: set = _set_weight_distribution_front
@export var aerodynamic_drag_coefficient: float = 0.32: set = _set_aerodynamic_drag_coefficient
@export var frontal_area: float = 2.2: set = _set_frontal_area

@export_group("Driving Aids")
@export var traction_control_enabled: bool = true: set = _set_traction_control_enabled
@export var anti_lock_braking_enabled: bool = true: set = _set_anti_lock_braking_enabled
@export var stability_control_enabled: bool = true: set = _set_stability_control_enabled
@export var manual_shift_mode: bool = false: set = _set_manual_shift_mode

@export_group("Drift Settings")
@export var drift_threshold: float = 0.3: set = _set_drift_threshold
@export var drift_recovery_rate: float = 0.92: set = _set_drift_recovery_rate
@export var drift_force_multiplier: float = 1.5: set = _set_drift_force_multiplier

# ============================================================================
# PRIVATE VARIABLES
# ============================================================================
var _physics_settings: PhysicsSettings = null
var _powertrain_node: Node = null
var _input_manager: InputManager = null
var _audio_manager: AudioManager = null
var _game_manager: GameManager = null

# Vehicle state
var current_speed: float = 0.0  # m/s
var current_rpm: float = IDLE_RPM
var current_gear: int = 0  # 0=neutral, 1-6=fwd, -1=reverse
var target_gear: int = 0
var is_in_neutral: bool = true
var is_engine_running: bool = true
var clutch_disengaged: bool = false

# Input states
var throttle_input: float = 0.0
var brake_input: float = 0.0
var steering_input: float = 0.0
var handbrake_input: bool = false
var upshift_input: bool = false
var downshift_input: bool = false
var neutral_input: bool = false

# Wheel forces and contact data
var front_left_wheel_force: float = 0.0
var front_right_wheel_force: float = 0.0
var rear_left_wheel_force: float = 0.0
var rear_right_wheel_force: float = 0.0

# Drift state
var drift_angle: float = 0.0
var is_drifting: bool = false
var lateral_velocity: float = 0.0

# Traction control state
var wheel_slip_ratio: float = 0.0
var slip_ratio_target: float = 0.15
var traction_control_active: bool = false

# Timing variables
var _last_update_time: float = 0.0
var _gear_change_timer: float = 0.0
var _gear_change_duration: float = 0.15  # seconds per gear shift
var _clutch_release_timer: float = 0.0
var _clutch_release_duration: float = 0.2

# Collision state
var last_collision_position: Vector3 = Vector3.ZERO
var last_collision_normal: Vector3 = Vector3.UP
var collision_impact_force: float = 0.0
var total_collisions: int = 0

# Engine state
var engine_temperature: float = 90.0  # Celsius
var oil_pressure: float = 45.0  # PSI
var coolant_temp: float = 90.0  # Celsius
var is_overheated: bool = false
var fuel_level: float = 100.0  # Percentage
var fuel_consumption_rate: float = 0.0

# ============================================================================
# INITIALIZATION
# ============================================================================
func _ready() -> void:
	_init_references()
	_connect_signals()
	_reset_vehicle_state()
	
	# Ensure we're using continuous physics for better collision detection
	collision_layer = 1 << 3  # Vehicle layer
	collision_mask = (1 << 0) | (1 << 1) | (1 << 2)  # Terrain, obstacles, other vehicles

func _init_references() -> void:
	_physics_settings = PhysicsSettings.new()
	_game_manager = GameManager.get_singleton() if GameManager.has_singleton() else null
	_audio_manager = AudioManager.get_singleton() if AudioManager.has_singleton() else null
	_input_manager = InputManager.get_singleton() if InputManager.has_singleton() else null
	
	# Find powertrain node
	_powertrain_node = get_node_or_null("../Powertrain") if has_node("../Powertrain") else null

func _connect_signals() -> void:
	if _game_manager != null:
		_game_manager.race_started.connect(_on_race_started)
		_game_manager.game_state_changed.connect(_on_game_state_changed)
	
	if _audio_manager != null:
		rpm_changed.connect(_audio_manager.play_rpm_sound.bind(current_rpm))
		speed_changed.connect(_audio_manager.play_speed_sound.bind(current_speed))
		
		skidding.connect(func(is_skid: bool):
			if is_skid:
				_audio_manager.play_sfx("skid_tire")
			else:
				_audio_manager.stop_sfx("skid_tire")
		)

func _reset_vehicle_state() -> void:
	current_speed = 0.0
	current_rpm = IDLE_RPM
	current_gear = 0
	target_gear = 0
	is_in_neutral = true
	is_engine_running = false
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	drift_angle = 0.0
	is_drifting = false
	lateral_velocity = 0.0
	engine_temperature = 90.0
	fuel_level = 100.0
	total_collisions = 0

# ============================================================================
# MAIN GAME LOOP
# ============================================================================
func _physics_process(delta: float) -> void:
	if not is_engine_running and current_speed <= 0.1:
		return
		
	_last_update_time = Time.get_ticks_msec() / 1000.0
	_apply_inputs(delta)
	_calculate_dynamics(delta)
	_handle_gearing(delta)
	_apply_wheel_forces(delta)
	_update_vehicle_state(delta)
	_check_drift_state(delta)
	_handle_collision_response(delta)
	_monitor_engine_health(delta)
	_update_fuel_system(delta)

# ============================================================================
# INPUT HANDLING
# ============================================================================
func _apply_inputs(delta: float) -> void:
	# Get input from InputManager if available
	if _input_manager != null:
		var input_data = _input_manager.get_vehicle_inputs(global_position)
		throttle_input = clamp(input_data.throttle, 0.0, 1.0)
		brake_input = clamp(input_data.brake, 0.0, 1.0)
		steering_input = clamp(input_data.steering, -1.0, 1.0)
		handbrake_input = input_data.handbrake
		upshift_input = input_data.upshift
		downshift_input = input_data.downshift
		neutral_input = input_data.neutral
	
	# Clamp values
	throttle_input = clamp(throttle_input, 0.0, 1.0)
	brake_input = clamp(brake_input, 0.0, 1.0)
	steering_input = clamp(steering_input, -1.0, 1.0)
	
	# Emit signals for input changes
	if abs(throttle_input - get_throttle_input()) > 0.01:
		throttle_applied.emit(throttle_input)
	if abs(brake_input - get_brake_input()) > 0.01:
		brake_applied.emit(brake_input)
	if abs(steering_input - get_steering_input()) > 0.01:
		steering_angle_changed.emit(steering_input * MAX_STEERING_ANGLE)

func _handle_gearing(delta: float) -> void:
	# Auto-shift logic
	if not manual_shift_mode and is_engine_running:
		_auto_shift_logic(delta)
	
	# Manual shift override
	if upshift_input or downshift_input:
		_manual_shift_attempt(delta)
	
	# Neutral gear handling
	if neutral_input:
		_set_gear(0)
	
	# Handle gear change in progress
	if _gear_change_timer > 0:
		_gear_change_timer -= delta
		if _gear_change_timer <= 0:
			_complete_gear_change()
	
	# Clutch release simulation
	if _clutch_release_timer > 0:
		_clutch_release_timer -= delta

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================
func _auto_shift_logic(delta: float) -> void:
	# Determine target gear based on RPM and speed
	var target_gear_calc = _calculate_optimal_gear()
	
	# Prevent rapid shifting back and forth
	if target_gear_calc != target_gear and abs(target_gear_calc - target_gear) <= 1:
		if current_rpm >= SHIFT_POINT_RPM and target_gear_calc > current_gear:
			_request_gear_change(target_gear_calc)
		elif current_rpm < IDLE_RPM * 1.5 and target_gear_calc < current_gear:
			_request_gear_change(target_gear_calc)

func _manual_shift_attempt(delta: float) -> void:
	if _gear_change_timer > 0:
		return  # Still in gear change
	
	if upshift_input and current_gear < TRANSMISSION_GEAR_COUNT - 1:
		_request_gear_change(current_gear + 1)
	elif downshift_input and current_gear > 0:
		_request_gear_change(current_gear - 1)

func _request_gear_change(new_gear: int) -> void:
	if new_gear == current_gear:
		return
	
	var old_gear = current_gear
	current_gear = new_gear
	target_gear = new_gear
	
	# Simulate clutch engagement
	clutch_disengaged = true
	_clutch_release_timer = _clutch_release_duration
	
	_gear_change_timer = _gear_change_duration
	
	gear_changed.emit(old_gear, new_gear)
	
	# Play shift sound
	if _audio_manager != null:
		_audio_manager.play_sfx("gear_shift")

func _complete_gear_change() -> void:
	clutch_disengaged = false
	target_gear = current_gear
	_sync_transmission_with_wheels()

func _sync_transmission_with_wheels() -> void:
	# Calculate transmission output speed from wheel speed
	var wheel_radius: float = 0.33  # meters (typical tire radius)
	var wheel_rotation_speed: float = current_speed / wheel_radius  # rad/s
	
	# Transmission output speed = wheel speed / final drive
	var transmission_output_speed: float = wheel_rotation_speed / FINAL_DRIVE
	
	# Input shaft speed = transmission output * gear ratio
	if current_gear != 0:
		var gear_ratio: float = GEAR_RATIOS[current_gear]
		current_rpm = transmission_output_speed * gear_ratio * 9.549  # Convert to RPM
	else:
		current_rpm = IDLE_RPM

func _calculate_optimal_gear() -> int:
	if current_speed <= 0.1:
		return 1
	
	# Work backwards from top gear
	for gear in range(TRANSMISSION_GEAR_COUNT - 1, 0, -1):
		var gear_ratio: float = GEAR_RATIOS[gear]
		var wheel_radius: float = 0.33
		var estimated_rpm: float = (current_speed / wheel_radius) / FINAL_DRIVE * gear_ratio * 9.549
		
		if estimated_rpm <= SHIFT_POINT_RPM and estimated_rpm >= IDLE_RPM * 1.2:
			return gear
	
	return 1  # Default to first gear

# ============================================================================
# PHYSICS CALCULATIONS
# ============================================================================
func _calculate_dynamics(delta: float) -> void:
	# Calculate wheel slip ratio
	var drive_wheel_speed: float = 0.0
	var ground_speed: float = velocity.length()
	
	if current_gear > 0:
		# Forward motion
		var wheel_radius: float = 0.33
		var engine_wheel_speed: float = (current_rpm / 9.549) / (GEAR_RATIOS[current_gear] * FINAL_DRIVE) * wheel_radius
		wheel_slip_ratio = (engine_wheel_speed - ground_speed) / max(abs(ground_speed), 0.1)
	elif current_gear < 0:
		# Reverse motion
		var wheel_radius: float = 0.33
		var engine_wheel_speed: float = (current_rpm / 9.549) / (GEAR_RATIOS[abs(current_gear)] * FINAL_DRIVE) * wheel_radius
		wheel_slip_ratio = (engine_wheel_speed + ground_speed) / max(abs(ground_speed), 0.1)
	else:
		wheel_slip_ratio = 0.0
	
	# Apply traction control if enabled
	if traction_control_enabled and wheel_slip_ratio > slip_ratio_target:
		traction_control_active = true
		throttle_input *= (slip_ratio_target / wheel_slip_ratio)
	else:
		traction_control_active = false

func _apply_wheel_forces(delta: float) -> void:
	# Calculate engine torque based on RPM
	var engine_torque: float = _get_engine_torque(current_rpm)
	
	# Apply drivetrain losses
	var drivetrain_efficiency: float = 0.85
	engine_torque *= drivetrain_efficiency
	
	# Calculate wheel force based on gear
	var wheel_force: float = 0.0
	if current_gear > 0:
		# Forward gear
		wheel_force = engine_torque * GEAR_RATIOS[current_gear] * FINAL_DRIVE / 0.33
	elif current_gear < 0:
		# Reverse gear
		wheel_force = -(engine_torque * GEAR_RATIOS[abs(current_gear)] * FINAL_DRIVE / 0.33)
	else:
		wheel_force = 0.0
	
	# Apply throttle multiplier
	wheel_force *= throttle_input
	
	# Apply brake force if braking
	if brake_input > 0.01 or handbrake_input:
		var brake_force: float = BRAKE_FORCE * brake_input
		if handbrake_input:
			brake_force *= 0.6  # Handbrake only affects rear wheels
		
		# Distribute brake force to wheels
		rear_left_wheel_force -= brake_force * 0.5
		rear_right_wheel_force -= brake_force * 0.5
		
		if not handbrake_input:
			front_left_wheel_force -= brake_force * 0.5
			front_right_wheel_force -= brake_force * 0.5
	
	# Apply steering to front wheels
	var steering_angle: float = steering_input * MAX_STEERING_ANGLE
	
	# Distribute drive force between front and rear wheels (RWD default)
	var drive_force_distribution: float = 0.0  # Rear wheel drive
	var drive_force: float = wheel_force * (1.0 - drive_force_distribution)
	
	# Apply to rear wheels
	rear_left_wheel_force += drive_force * 0.5
	rear_right_wheel_force += drive_force * 0.5
	
	# Apply drag and rolling resistance
	var air_density: float = 1.225  # kg/m³ at sea level
	var drag_force: float = 0.5 * air_density * aerodynamic_drag_coefficient * frontal_area * current_speed * current_speed
	var rolling_resistance: float = 0.015 * _get_vehicle_mass()
	
	# Apply opposite to direction of travel
	if current_speed > 0:
		var drag_and_resistance: float = drag_force + rolling_resistance
		rear_left_wheel_force -= drag_and_resistance * 0.5
		rear_right_wheel_force -= drag_and_resistance * 0.5
	
	# Clamp wheel forces to prevent unrealistic values
	rear_left_wheel_force = clamp(rear_left_wheel_force, -BRAKE_FORCE, BRAKE_FORCE * 3)
	rear_right_wheel_force = clamp(rear_right_wheel_force, -BRAKE_FORCE, BRAKE_FORCE * 3)
	front_left_wheel_force = clamp(front_left_wheel_force, -BRAKE_FORCE, BRAKE_FORCE)
	front_right_wheel_force = clamp(front_right_wheel_force, -BRAKE_FORCE, BRAKE_FORCE)

func _get_engine_torque(rpm: float) -> float:
	# Use torque curve if available, otherwise use simplified model
	if torque_curve != null:
		return torque_curve.sample_baked(rpm / REDLINE_RPM) * max_power * 0.5
	
	# Simplified torque curve model
	var normalized_rpm: float = clamp(rpm / REDLINE_RPM, 0.0, 1.0)
	
	# Peak torque around 4000-5000 RPM
	var peak_torque_pos: float = 0.6
	var torque_peak: float = 1.2
	
	var torque: float = torque_peak * exp(-pow((normalized_rpm - peak_torque_pos) / 0.3, 2))
	torque = max(torque, 0.2)  # Minimum idle torque
	
	return torque * max_power * 0.5

func _update_vehicle_state(delta: float) -> void:
	# Update speed from velocity magnitude
	var prev_speed: float = current_speed
	current_speed = velocity.length()
	
	if prev_speed != current_speed:
		speed_changed.emit(current_speed)
	
	# Update RPM based on current gear and speed
	if current_gear != 0:
		var wheel_radius: float = 0.33
		var wheel_rotation_speed: float = current_speed / wheel_radius
		var transmission_output_speed: float = wheel_rotation_speed / FINAL_DRIVE
		var gear_ratio: float = GEAR_RATIOS[current_gear]
		current_rpm = transmission_output_speed * gear_ratio * 9.549
	else:
		# Engine idles when in neutral
		if is_engine_running:
			current_rpm = lerp(current_rpm, IDLE_RPM, delta * 5.0)
	
	# Clamp RPM
	current_rpm = clamp(current_rpm, IDLE_RPM, REDLINE_RPM)
	
	if abs(current_rpm - IDLE_RPM) > 100:
		rpm_changed.emit(current_rpm)

# ============================================================================
# DRIFT MECHANICS
# ============================================================================
func _check_drift_state(delta: float) -> void:
	# Calculate lateral velocity relative to car's heading
	var forward_direction: Vector3 = global_transform.basis.z
	lateral_velocity = velocity.dot(global_transform.basis.x)
	
	# Calculate drift angle (difference between velocity direction and car heading)
	var velocity_direction: Vector3 = velocity.normalized() if velocity.length() > 0.1 else Vector3.ZERO
	velocity_direction.y = 0
	velocity_direction = velocity_direction.normalized()
	
	var angle_diff: float = velocity_direction.angle_to(forward_direction)
	drift_angle = abs(angle_diff)
	
	# Check if drifting
	var drift_threshold_force: float = 2000.0
	var lateral_force: float = abs(lateral_velocity) * _get_vehicle_mass() * 1.5
	
	if lateral_force > drift_threshold_force and abs(steering_input) > 0.3:
		if not is_drifting:
			is_drifting = true
			drift_started.emit(drift_angle)
			if _audio_manager != null:
				_audio_manager.play_sfx("drift_start")
	else:
		if is_drifting:
			is_drifting = false
			drift_ended.emit()
			if _audio_manager != null:
				_audio_manager.play_sfx("drift_end")

func _apply_drift_effects(delta: float) -> void:
	if not is_drifting:
		return
	
	# Apply reduced friction during drift
	var friction_reduction: float = 1.0 - ((drift_angle / PI) * 0.7)
	velocity *= drift_recovery_rate
	
	# Add drift audio
	if _audio_manager != null and randf() < 0.1:
		_audio_manager.play_sfx("tire_squeal")

# ============================================================================
# ENGINE HEALTH MONITORING
# ============================================================================
func _monitor_engine_health(delta: float) -> void:
	# Temperature increases with load and RPM
	var load_factor: float = throttle_input * (current_rpm / REDLINE_RPM)
	var temp_increase: float = load_factor * delta * 5.0
	engine_temperature += temp_increase
	
	# Cooling when below threshold
	if engine_temperature < 95.0:
		engine_temperature -= delta * 2.0
	
	# Check for overheating
	if engine_temperature > 110.0:
		is_overheated = true
		_on_engine_overheat()
	elif engine_temperature < 100.0:
		is_overheated = false
	
	# Oil pressure decreases with high RPM
	oil_pressure = 45.0 - (current_rpm / REDLINE_RPM) * 15.0
	oil_pressure = max(oil_pressure, 20.0)
	
	# Coolant temperature follows engine temperature
	coolant_temp = lerp(coolant_temp, engine_temperature, delta * 0.5)

func _on_engine_overheat() -> void:
	# Reduce power when overheated
	max_power *= 0.5
	
	# Warn player
	if _audio_manager != null:
		_audio_manager.play_sfx("warning_overheat")

# ============================================================================
# FUEL SYSTEM
# ============================================================================
func _update_fuel_system(delta: float) -> void:
	# Fuel consumption based on throttle and RPM
	if is_engine_running and throttle_input > 0.01:
		var consumption_rate: float = throttle_input * (current_rpm / REDLINE_RPM) * 0.05
		fuel_level -= consumption_rate * delta
		
		if fuel_level <= 0:
			fuel_level = 0.0
			is_engine_running = false
			
			if _audio_manager != null:
				_audio_manager.play_sfx("engine_out_of_fuel")

# ============================================================================
# COLLISION HANDLING
# ============================================================================
func _handle_collision_response(delta: float) -> void:
	if colliding():
		var collision: CollisionObject3D = get_collider()
		var collision_point: Vector3 = get_collision_point()
		var collision_normal: Vector3 = get_collision_normal()
		
		# Calculate impact force
		var impact_velocity: float = velocity.dot(collision_normal)
		collision_impact_force = abs(impact_velocity) * _get_vehicle_mass()
		
		# Apply bounce/restitution
		var restitution: float = 0.3
		var bounce_force: Vector3 = collision_normal * impact_velocity * restitution
		
		velocity += bounce_force
		
		# Log collision
		last_collision_position = collision_point
		last_collision_normal = collision_normal
		total_collisions += 1
		
		collision_detected.emit({
			"position": collision_point,
			"normal": collision_normal,
			"force": collision_impact_force,
			"collider": collision.name if collision else "unknown",
			"collision_count": total_collisions
		})
		
		# Screen shake effect (if camera exists)
		_trigger_screen_shake(min(collision_impact_force / 1000.0, 0.5))

func _trigger_screen_shake(magnitude: float) -> void:
	# Trigger screen shake via signal or direct camera manipulation
	if _game_manager != null and _game_manager.camera_3d != null:
		_game_manager.camera_3d.shake(magnitude)

# ============================================================================
# VEHICLE CONTROL METHODS
# ============================================================================
func start_engine() -> void:
	if is_engine_running:
		return
	
	is_engine_running = true
	current_rpm = IDLE_RPM
	
	if _audio_manager != null:
		_audio_manager.play_sfx("engine_start")

func stop_engine() -> void:
	is_engine_running = false
	current_rpm = IDLE_RPM
	current_gear = 0
	throttle_input = 0.0
	brake_input = 0.0
	
	if _audio_manager != null:
		_audio_manager.play_sfx("engine_stop")

func toggle_handbrake(active: bool) -> void:
	handbrake_input = active
	handbrake_toggled.emit(active)

func set_traction_control(active: bool) -> void:
	traction_control_enabled = active
	traction_control_state_changed.emit(active)

func set_anti_lock_braking(active: bool) -> void:
	anti_lock_braking_enabled = active
	anti_lock_braking_state_changed.emit(active)

func reset_vehicle() -> void:
	_reset_vehicle_state()
	velocity = Vector3.ZERO
	angular_velocity = Vector3.ZERO
	global_position = Vector3.ZERO
	rotation = Vector3.ZERO

# ============================================================================
# HELPER METHODS
# ============================================================================
func _get_vehicle_mass() -> float:
	return 1500.0  # Base mass, could be exported/loaded

func get_throttle_input() -> float:
	return throttle_input

func get_brake_input() -> float:
	return brake_input

func get_steering_input() -> float:
	return steering_input

func get_current_gear() -> int:
	return current_gear

func get_current_rpm() -> float:
	return current_rpm

func get_current_speed() -> float:
	return current_speed

func is_vehicle_driving() -> bool:
	return is_engine_running and abs(current_speed) > 0.5

func is_vehicle_moving_forward() -> bool:
	return current_speed > 0.5

func is_vehicle_moving_backward() -> bool:
	return current_speed < -0.5

# ============================================================================
# EVENT HANDLERS
# ============================================================================
func _on_race_started(race_data: Dictionary) -> void:
	# Reset vehicle for race start
	start_engine()
	_reset_vehicle_state()
	velocity = Vector3.ZERO

func _on_game_state_changed(new_state: GameState) -> void:
	match new_state:
		GameManager.GameState.RACE_ACTIVE:
			start_engine()
		GameManager.GameState.RACE_FINISHED:
			stop_engine()
		_:
			pass

# ============================================================================
# DEBUG / VISUALIZATION
# ============================================================================
func _draw_debug_lines() -> void:
	if not GameManager.debug_mode:
		return
	
	# Draw wheel forces as lines
	var wheel_positions: Array[Vector3] = [
		global_position + Vector3(-TRACK_WIDTH/2, 0.3, WHEEL_BASE/2),  # Front Left
		global_position + Vector3(TRACK_WIDTH/2, 0.3, WHEEL_BASE/2),   # Front Right
		global_position + Vector3(-TRACK_WIDTH/2, 0.3, -WHEEL_BASE/2), # Rear Left
		global_position + Vector3(TRACK_WIDTH/2, 0.3, -WHEEL_BASE/2)   # Rear Right
	]
	
	for i in range(wheel_positions.size()):
		var force_vector: Vector3 = Vector3.ZERO
		match i:
			0: force_vector = Vector3.RIGHT * front_left_wheel_force
			1: force_vector = Vector3.RIGHT * front_right_wheel_force
			2: force_vector = Vector3.RIGHT * rear_left_wheel_force
			3: force_vector = Vector3.RIGHT * rear_right_wheel_force
		
		draw_line(wheel_positions[i], wheel_positions[i] + force_vector * 0.001, Color.GREEN)

# ============================================================================
# PROPERTY SETTERS
# ============================================================================
func _set_max_power(value: float) -> void:
	max_power = value
	notify_property_list_changed()

func _set_torque_curve(value: Curve) -> void:
	torque_curve = value
	notify_property_list_changed()

func _set_weight_distribution_front(value: float) -> void:
	weight_distribution_front = clamp(value, 0.3, 0.7)
	notify_property_list_changed()

func _set_aerodynamic_drag_coefficient(value: float) -> void:
	aerodynamic_drag_coefficient = clamp(value, 0.2, 0.6)
	notify_property_list_changed()

func _set_frontal_area(value: float) -> void:
	frontal_area = clamp(value, 1.5, 3.5)
	notify_property_list_changed()

func _set_traction_control_enabled(value: bool) -> void:
	traction_control_enabled = value
	traction_control_state_changed.emit(value)

func _set_anti_lock_braking_enabled(value: bool) -> void:
	anti_lock_braking_enabled = value
	anti_lock_braking_state_changed.emit(value)

func _set_stability_control_enabled(value: bool) -> void:
	stability_control_enabled = value

func _set_manual_shift_mode(value: bool) -> void:
>manual_shift_mode = value

func _set_drift_threshold(value: float) -> void:
	drift_threshold = clamp(value, 0.1, 0.8)

func _set_drift_recovery_rate(value: float) -> void:
	drift_recovery_rate = clamp(value, 0.8, 1.0)

func _set_drift_force_multiplier(value: float) -> void:
	drift_force_multiplier = clamp(value, 1.0, 3.0)