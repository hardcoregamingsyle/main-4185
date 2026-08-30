extends CharacterBody2D
class_name VehicleController

## VehicleController - Core vehicle physics controller using PhysicsSettings constants
## Handles throttle, brake, steering inputs, wheel forces, gear shifting logic
## Copyright 2026 Thalamus Racing Simulator Project

# ============================================================================
# SIGNALS
# ============================================================================

signal rpm_changed(new_rpm: float)
signal gear_changed(old_gear: int, new_gear: int)
signal speed_changed(new_speed: float)
signal throttle_changed(new_throttle: float)
signal brake_changed(new_brake: float)
signal collision_detected(collision_shape: Shape2D, collision_point: Vector2)
signal skid_detected(wheel_index: int, slip_ratio: float)
signal engine_overheat(temperature: float)
signal transmission_error(error_type: String)

# ============================================================================
# CONSTANTS & CONFIGURATION
# ============================================================================

const MAX_SPEED_KMH: float = 320.0
const MIN_RPM: float = 800.0
const IDLE_RPM: float = 900.0
const REDLINE_RPM: float = 7500.0
const SHIFTER_DELAY_MS: float = 150.0

# Engine characteristics (tuneable via export)
@export var engine_displacement: float = 5.0  # liters
@export var max_torque_nm: float = 500.0
@export var torque_curve_peak_rpm: float = 4500.0
@export var cylinder_count: int = 8
@export var fuel_capacity_liters: float = 70.0

# Gear ratios (1st through 6th + reverse)
var _gear_ratios: Array[float] = [3.8, 2.2, 1.5, 1.1, 0.9, 0.7, -4.0]
var _final_drive_ratio: float = 3.5

# ============================================================================
# PHYSICS SETTINGS REFERENCE
# ============================================================================

var _physics_settings: PhysicsSettings = null

# ============================================================================
# VEHICLE STATE
# ============================================================================

var current_speed_kmh: float = 0.0
var current_rpm: float = IDLE_RPM
var current_gear: int = 0  # -1 reverse, 0 neutral, 1-6 forward
var clutch_engaged: bool = true

var throttle_input: float = 0.0
var brake_input: float = 0.0
var steering_input: float = 0.0

var _engine_temperature: float = 90.0
var _fuel_level: float = 100.0
var _traction_control_active: bool = true
var _abs_active: bool = false

# ============================================================================
# WHEEL PHYSICS
# ============================================================================

const NUM_WHEELS: int = 4
var _wheel_states: Array[Dictionary] = []

func _init_wheel_states() -> void:
	for i in range(NUM_WHEELS):
		_wheel_states.append({
			"index": i,
			"position": Vector2.ZERO,
			"rotation": 0.0,
			"angular_velocity": 0.0,
			"slip_ratio": 0.0,
			"brake_force": 0.0,
			"driven": false,
			"steerable": false,
			"contact_normal": Vector2.UP,
			"friction_coefficient": 1.0
		})

# ============================================================================
# INTERNAL VARIABLES
# ============================================================================

var _last_shift_time: float = 0.0
var _vehicle_mass: float = 1500.0
var _drag_coefficient: float = 0.30
var _frontal_area: float = 2.2
var _air_density: float = 1.225
var _inertia_factor: float = 0.5

var _velocity_direction: Vector2 = Vector2.RIGHT
var _acceleration: float = 0.0
var _braking_force: float = 0.0
var _current_torque: float = 0.0

# ============================================================================
# PROPERTIES
# ============================================================================

func get_current_speed_ms() -> float:
	return current_speed_kmh / 3.6

func get_current_rpm() -> float:
	return current_rpm

func get_current_gear() -> int:
	return current_gear

func get_engine_temperature() -> float:
	return _engine_temperature

func get_fuel_level() -> float:
	return _fuel_level

func get_is_clutch_engaged() -> bool:
	return clutch_engaged

func get_throttle_input() -> float:
	return throttle_input

func get_brake_input() -> float:
	return brake_input

func get_steering_input() -> float:
	return steering_input

# ============================================================================
# INITIALIZATION
# ============================================================================

func _ready() -> void:
	_connect_to_game_manager()
	_init_wheel_states()
	_load_physics_settings()
	_setup_vehicle_defaults()

func _process(delta: float) -> void:
	if not is_instance_valid(_physics_settings):
		return
	
	_update_physics(delta)
	_handle_inputs(delta)
	_check_conditions()

func _physics_process(delta: float) -> void:
	if not is_instance_valid(_physics_settings):
		return
	
	_apply_physics(delta)

# ============================================================================
# PHYSICS SETTINGS INTEGRATION
# ============================================================================

func _load_physics_settings() -> void:
	# Load PhysicsSettings resource if available
	var settings_path: String = "res://scripts/core/PhysicsSettings.gd"
	
	# Check if PhysicsSettings autoload exists
	if GameManager.has_singleton("PhysicsSettings"):
		_physics_settings = GameManager.get_singleton("PhysicsSettings") as PhysicsSettings
	else:
		# Fallback to defaults
		_physics_settings = preload(settings_path).new()
	
	# Apply physics settings to vehicle
	_vehicle_mass = _physics_settings.default_vehicle_mass
	
	# Update wheel count based on drivetrain type
	_update_drivetrain_configuration()

func _update_drivetrain_configuration() -> void:
	# Configure which wheels are driven and steerable
	match _physics_settings.drivetrain_type:
		"RWD":
			_wheel_states[0].driven = false  # Front left
			_wheel_states[1].driven = false  # Front right
			_wheel_states[2].driven = true   # Rear left
			_wheel_states[3].driven = true   # Rear right
			_wheel_states[0].steerable = true
			_wheel_states[1].steerable = true
		"FWD":
			_wheel_states[0].driven = true
			_wheel_states[1].driven = true
			_wheel_states[2].driven = false
			_wheel_states[3].driven = false
			_wheel_states[0].steerable = true
			_wheel_states[1].steerable = true
		"AWD":
			_wheel_states[0].driven = true
			_wheel_states[1].driven = true
			_wheel_states[2].driven = true
			_wheel_states[3].driven = true
			_wheel_states[0].steerable = true
			_wheel_states[1].steerable = true
		_:
			# Default RWD configuration
			_wheel_states[0].driven = false
			_wheel_states[1].driven = false
			_wheel_states[2].driven = true
			_wheel_states[3].driven = true
			_wheel_states[0].steerable = true
			_wheel_states[1].steerable = true

# ============================================================================
# INPUT HANDLING
# ============================================================================

func _connect_to_game_manager() -> void:
	# Connect to InputManager signals
	if GameManager.has_singleton("InputManager"):
		var input_manager: Node = GameManager.get_singleton("InputManager")
		input_manager.connect("input_updated", _on_input_updated)
	
	# Connect to AudioManager for sound feedback
	if GameManager.has_singleton("AudioManager"):
		var audio_manager: Node = GameManager.get_singleton("AudioManager")
		audio_manager.connect("sound_played", _on_sound_played)

func _on_input_updated(input_data: Dictionary) -> void:
	throttle_input = clamp(input_data.get("throttle", 0.0), 0.0, 1.0)
	brake_input = clamp(input_data.get("brake", 0.0), 0.0, 1.0)
	steering_input = clamp(input_data.get("steering", 0.0), -1.0, 1.0)
	
	emit_signal("throttle_changed", throttle_input)
	emit_signal("brake_changed", brake_input)

func _handle_inputs(delta: float) -> void:
	# Handle clutch control
	if Input.is_action_just_pressed("clutch_toggle"):
		clutch_engaged = not clutch_engaged
	
	# Handle gear shifting
	_handle_manual_shifting()
	
	# Apply traction control if active
	if _traction_control_active:
		_apply_traction_control()

func _handle_manual_shifting() -> void:
	if not clutch_engaged:
		return
	
	# Upshift
	if Input.is_action_just_pressed("shift_up"):
		if current_gear < 6:
			_shift_gear(current_gear + 1)
		elif current_gear == 6 and current_rpm >= REDLINE_RPM:
			transmission_error.emit("overrev_limit")
	
	# Downshift
	if Input.is_action_just_pressed("shift_down"):
		if current_gear > 1:
			_shift_gear(current_gear - 1)
		elif current_gear == 1 and current_rpm <= MIN_RPM:
			transmission_error.emit("stall_warning")
	
	# Neutral
	if Input.is_action_just_pressed("neutral"):
		if current_gear != 0:
			_shift_gear(0)

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================

func _shift_gear(new_gear: int) -> void:
	var old_gear: int = current_gear
	
	# Enforce shift delay
	var time_since_last_shift: float = Time.get_ticks_msec() - _last_shift_time
	if time_since_last_shift < SHIFTER_DELAY_MS:
		return
	
	# Prevent shifting from neutral into reverse directly
	if old_gear == 0 and new_gear < 0:
		transmission_error.emit("invalid_reverse_shift")
		return
	
	# Validate gear range
	if new_gear < -1 or new_gear > 6:
		transmission_error.emit("invalid_gear_request")
		return
	
	# Execute shift
	current_gear = new_gear
	_last_shift_time = Time.get_ticks_msec()
	
	# Calculate target RPM based on gear ratio
	_calculate_target_rpm()
	
	# Play shift sound
	if GameManager.has_singleton("AudioManager"):
		var audio_manager: Node = GameManager.get_singleton("AudioManager")
		audio_manager.play_sound("gear_shift")
	
	emit_signal("gear_changed", old_gear, new_gear)
	
	# Notify powertrain system
	_notify_powertrain_of_shift(old_gear, new_gear)

func _calculate_target_rpm() -> void:
	var wheel_rpm: float = (current_speed_kmh * 1000.0 / 60.0) / (_final_drive_ratio * _gear_ratios[current_gear])
	var target_rpm: float = wheel_rpm * 1000.0  # Convert to RPM
	
	# Clamp to valid RPM range
	target_rpm = clamp(target_rpm, MIN_RPM, REDLINE_RPM)
	
	# Smooth transition
	var rpm_difference: float = target_rpm - current_rpm
	_current_rpm_transition += rpm_difference * 0.1
	
	# Apply gradual change
	current_rpm += _current_rpm_transition
	_current_rpm_transition *= 0.9

func _notify_powertrain_of_shift(old_gear: int, new_gear: int) -> void:
	# Signal to Powertrain subsystem about gear change
	var powertrain_node: Node = find_child("Powertrain")
	if powertrain_node:
		powertrain_node.call_deferred("_on_gear_shifted", old_gear, new_gear)

# ============================================================================
# PHYSICS CALCULATIONS
# ============================================================================

func _update_physics(delta: float) -> void:
	# Update engine temperature
	_update_engine_temperature(delta)
	
	# Update fuel consumption
	_update_fuel_consumption(delta)
	
	# Calculate aerodynamic drag
	_calculate_drag_force()
	
	# Calculate rolling resistance
	_calculate_rolling_resistance()

func _apply_physics(delta: float) -> void:
	if current_gear == 0:
		# In neutral, apply minimal force
		_apply_coasting_physics(delta)
		return
	
	# Calculate engine torque based on RPM
	_current_torque = _calculate_engine_torque()
	
	# Apply drive force through wheels
	_apply_drive_force(delta)
	
	# Apply braking force
	_apply_braking_force(delta)
	
	# Update velocity based on net forces
	_update_velocity(delta)
	
	# Update position
	position += velocity * delta

func _calculate_engine_torque() -> float:
	# Torque curve function - peak torque at specified RPM
	var rpm_ratio: float = current_rpm / torque_curve_peak_rpm
	var torque_multiplier: float = exp(-pow(rpm_ratio - 1.0, 2) * 3.0)
	
	var engine_torque: float = max_torque_nm * torque_multiplier
	
	# Apply throttle influence
	engine_torque *= throttle_input
	
	# Reduce torque when clutch is disengaged
	if not clutch_engaged:
		engine_torque *= 0.1
	
	return engine_torque

func _apply_drive_force(delta: float) -> void:
	if current_gear == 0 or current_gear < 0 and current_speed_kmh > 0.1:
		return
	
	# Calculate wheel angular velocity based on speed
	var wheel_circumference: float = 2.0 * PI * 0.3  # 0.3m radius tires
	var wheel_rps: float = (current_speed_kmh / 3.6) / wheel_circumference
	var wheel_rpm: float = wheel_rps * 60.0
	
	# Calculate drive force
	var drive_force: float = (_current_torque / _final_drive_ratio) / 0.3  # Divide by wheel radius
	
	# Distribute force to driven wheels
	var num_driven_wheels: int = 0
	for wheel_state in _wheel_states:
		if wheel_state.driven:
			drive_force /= 2.0  # Split between two wheels
			wheel_state.angular_velocity = wheel_rpm
			
			# Add drive force to acceleration
			_acceleration += drive_force / _vehicle_mass
			num_driven_wheels += 1
	
	# Adjust for traction loss
	if num_driven_wheels > 0:
		# Apply traction coefficient
		_acceleration *= 0.95  # Slight loss factor

func _apply_braking_force(delta: float) -> void:
	if brake_input <= 0.0:
		return
	
	# Calculate maximum braking force per wheel
	var max_brake_force_per_wheel: float = 8000.0  # Newtons
	var total_brake_force: float = 0.0
	
	for wheel_state in _wheel_states:
		if brake_input > 0.0:
			var brake_pressure: float = brake_input * max_brake_force_per_wheel
			wheel_state.brake_force = brake_pressure
			total_brake_force += brake_pressure
	
	# Apply ABS if active
	if _abs_active:
		_total_brake_force = _apply_abs(total_brake_force, delta)
	else:
		_braking_force = total_brake_force / _vehicle_mass
	
	# Apply deceleration
	_acceleration -= _braking_force

func _apply_abs(brake_force: float, delta: float) -> float:
	# Simplified ABS calculation
	# Monitor wheel slip and modulate brake pressure
	var adjusted_brake: float = brake_force
	
	for wheel_state in _wheel_states:
		if wheel_state.slip_ratio > 0.15:  # Slip threshold
			adjusted_brake *= 0.8  # Reduce brake pressure on slipping wheel
	
	return adjusted_brake

func _update_velocity(delta: float) -> void:
	# Apply all accelerations
	velocity.x += _acceleration * delta
	
	# Apply drag
	velocity.x *= (1.0 - _drag_coefficient * delta)
	
	# Update speed
	current_speed_kmh = abs(velocity.x) * 3.6
	
	# Update direction
	if velocity.x != 0:
		_velocity_direction = velocity.normalized()
	
	# Clamp maximum speed
	if current_speed_kmh > MAX_SPEED_KMH:
		current_speed_kmh = MAX_SPEED_KMH
		velocity.x = (MAX_SPEED_KMH / 3.6) * sign(velocity.x)
	
	emit_signal("speed_changed", current_speed_kmh)

func _apply_coasting_physics(delta: float) -> void:
	# Apply rolling resistance during coasting
	var rolling_resistance: float = _vehicle_mass * 0.015  # Approximate coefficient
	
	# Apply drag
	var drag_force: float = 0.5 * _air_density * _drag_coefficient * _frontal_area * (velocity.x ** 2)
	
	# Combine resistive forces
	var total_resistance: float = rolling_resistance + drag_force
	
	# Apply deceleration
	velocity.x -= (total_resistance / _vehicle_mass) * delta

# ============================================================================
# AERODYNAMICS & RESISTANCE
# ============================================================================

func _calculate_drag_force() -> void:
	var air_density: float = _air_density
	var frontal_area: float = _frontal_area
	var drag_coefficient: float = _drag_coefficient
	
	var speed_ms: float = current_speed_kmh / 3.6
	var drag_force: float = 0.5 * air_density * frontal_area * drag_coefficient * (speed_ms ** 2)
	
	# Apply drag opposite to velocity direction
	if velocity.x != 0:
		var drag_acceleration: float = -drag_force / _vehicle_mass
		velocity.x += drag_acceleration * 0.01  # Damping factor

func _calculate_rolling_resistance() -> void:
	var rolling_resistance_coefficient: float = 0.015
	var normal_force: float = _vehicle_mass * _physics_settings.gravity
	
	var rolling_resistance: float = rolling_resistance_coefficient * normal_force
	
	# Apply opposite to velocity
	if velocity.x != 0:
		var resistance_acceleration: float = -rolling_resistance / _vehicle_mass
		velocity.x += resistance_acceleration * 0.01

# ============================================================================
# ENGINE MANAGEMENT
# ============================================================================

func _update_engine_temperature(delta: float) -> void:
	# Heat generation based on load and RPM
	var heat_generation: float = throttle_input * current_rpm * 0.001
	
	# Cooling based on airflow (speed)
	var cooling_rate: float = (current_speed_kmh / MAX_SPEED_KMH) * 0.5
	
	# Ambient temperature effect
	var ambient_temp: float = 25.0
	
	# Update temperature
	_engine_temperature += (heat_generation - cooling_rate) * delta
	
	# Clamp to realistic range
	_engine_temperature = clamp(_engine_temperature, 40.0, 120.0)
	
	# Emit overheating warning
	if _engine_temperature > 110.0:
		emit_signal("engine_overheat", _engine_temperature)

func _update_fuel_consumption(delta: float) -> void:
	# Fuel consumption increases with throttle and RPM
	var base_consumption: float = 0.0001  # liters per second idle
	var load_factor: float = throttle_input * (current_rpm / REDLINE_RPM)
	
	var consumption: float = base_consumption + (load_factor * 0.001)
	
	_fuel_level -= consumption * delta
	
	# Check for empty tank
	if _fuel_level <= 0.0:
		_fuel_level = 0.0
		transmission_error.emit("out_of_fuel")
		_disable_engine()

func _disable_engine() -> void:
	current_rpm = 0.0
	throttle_input = 0.0
	velocity.x = 0.0
	current_speed_kmh = 0.0
	current_gear = 0
	coast_to_stop()

# ============================================================================
# TRACTION CONTROL
# ============================================================================

func _apply_traction_control() -> void:
	for wheel_state in _wheel_states:
		if wheel_state.driven and wheel_state.slip_ratio > 0.2:
			# Reduce engine torque
			_current_torque *= 0.7
			
			# Apply brake to spinning wheel
			wheel_state.brake_force += 2000.0

# ============================================================================
# SKID DETECTION
# ============================================================================

func _check_conditions() -> void:
	# Check for skidding
	for wheel_state in _wheel_states:
		if abs(wheel_state.slip_ratio) > 0.25:
			emit_signal("skid_detected", wheel_state.index, wheel_state.slip_ratio)
		
		# Detect collision
		if _has_collision():
			var collision_info: Dictionary = _get_collision_info()
			emit_signal("collision_detected", collision_info.shape, collision_info.point)

func _has_collision() -> bool:
	for body in get_slide_collision_iterator():
		return true
	return false

func _get_collision_info() -> Dictionary:
	var collision: CollisionShape2D = get_slide_collision(0)
	if collision:
		return {
			"shape": collision.shape,
			"point": collision.get_collision_point(),
			"normal": collision.get_collision_normal()
		}
	return {}

# ============================================================================
# SOUND FEEDBACK
# ============================================================================

func _on_sound_played(sound_name: String) -> void:
	match sound_name:
		"gear_shift":
			# Gear shift sound already handled in _shift_gear
			pass
		"engine_start":
			current_rpm = IDLE_RPM
		"engine_stall":
			current_rpm = 0.0
		"skid":
			# Skid sound triggered when slip detected
			pass

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================

func reset_vehicle() -> void:
	current_speed_kmh = 0.0
	current_rpm = IDLE_RPM
	current_gear = 0
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	_engine_temperature = 90.0
	_fuel_level = 100.0
	velocity = Vector2.ZERO
	position = Vector2.ZERO

func set_gear(gear: int) -> void:
	current_gear = clamp(gear, -1, 6)
	_shift_gear(gear)

func set_clutch_engaged(is_engaged: bool) -> void:
	clutch_engaged = is_engaged

func enable_traction_control(enable: bool) -> void:
	_traction_control_active = enable

func enable_abs(enable: bool) -> void:
	_abs_active = enable

func refuel(amount: float) -> void:
	_fuel_level = min(_fuel_level + amount, fuel_capacity_liters)

func coast_to_stop() -> void:
	throttle_input = 0.0
	brake_input = 0.0
	current_gear = 0

func get_vehicle_status() -> Dictionary:
	return {
		"speed_kmh": current_speed_kmh,
		"rpm": current_rpm,
		"gear": current_gear,
		"throttle": throttle_input,
		"brake": brake_input,
		"steering": steering_input,
		"clutch_engaged": clutch_engaged,
		"engine_temperature": _engine_temperature,
		"fuel_level": _fuel_level,
		"traction_control": _traction_control_active,
		"abs_active": _abs_active,
		"velocity": velocity,
		"position": position
	}

func calculate_optimal_shift_point() -> int:
	# Returns recommended gear based on current speed and RPM
	var optimal_gear: int = 1
	
	for gear in range(1, 7):
		var wheel_rpm: float = (current_speed_kmh * 1000.0 / 60.0) / (_final_drive_ratio * _gear_ratios[gear])
		if wheel_rpm > IDLE_RPM and wheel_rpm < REDLINE_RPM:
			optimal_gear = gear
	
	return optimal_gear

func debug_print_status() -> void:
	print("[VehicleController]")
	print("Speed: %.2f km/h" % current_speed_kmh)
	print("RPM: %.0f" % current_rpm)
	print("Gear: %d" % current_gear)
	print("Throttle: %.2f" % throttle_input)
	print("Brake: %.2f" % brake_input)
	print("Steering: %.2f" % steering_input)
	print("Engine Temp: %.1f°C" % _engine_temperature)
	print("Fuel Level: %.1f%%" % _fuel_level)
	print("Velocity: %s" % str(velocity))
	print("---")

# ============================================================================
# DEBUG / TESTING
# ============================================================================

func _setup_vehicle_defaults() -> void:
	# Initialize default values
	current_speed_kmh = 0.0
	current_rpm = IDLE_RPM
	current_gear = 0
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	_engine_temperature = 90.0
	_fuel_level = 100.0
	vehicle_mass = _physics_settings.default_vehicle_mass

func _set_gravity(value: float) -> void:
	gravity = value
	_update_physics_settings()

func _set_physics_tick_rate(value: int) -> void:
	physics_tick_rate = value
	_update_physics_settings()

func _set_max_substeps(value: int) -> void:
	max_substeps = value
	_update_physics_settings()

func _set_time_scale(value: float) -> void:
	time_scale = value
	_update_physics_settings()

func _set_default_vehicle_mass(value: float) -> void:
	default_vehicle_mass = value
	vehicle_mass = value

func _update_physics_settings() -> void:
	# Apply updated physics settings to vehicle
	if _physics_settings:
		vehicle_mass = _physics_settings.default_vehicle_mass
		_update_drivetrain_configuration()

# ============================================================================
# DESTRUCTOR
# ============================================================================

func _exit_tree() -> void:
	# Cleanup any remaining resources
	_wheel_states.clear()
	_physics_settings = null

</file_content>