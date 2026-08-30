extends Node
class_name VehicleController

## VehicleController - Core vehicle physics controller using PhysicsSettings constants
## Handles throttle, brake, steering inputs, wheel forces, gear shifting logic
## Copyright 2026 Thalamus Racing Simulator Project

# ============================================================================
# SIGNALS
# ============================================================================
signal engine_rpm_changed(rpm: float)
signal speed_changed(speed: float)
signal gear_changed(gear: int)
signal traction_control_triggered(is_active: bool)
signal anti_lock_brake_triggered(is_active: bool)
signal drift_started(angle: float)
signal drift_ended()
signal wheel_slip_detected(wheel_index: int, slip_ratio: float)
signal collision_impact(velocity: Vector3, impact_point: Vector3)

# ============================================================================
# ENGINE CONSTANTS FROM PHYSICS SETTINGS
# ============================================================================
const GRAVITY_Y = -9.81
const MAX_SUBSTEPS = 4
const PHYSICS_TICK_RATE = 120

# ============================================================================
# VEHICLE STATE ENUMS
# ============================================================================
enum DrivingMode {
	NORMAL,
	SPORT,
	RACE,
	DRASTIC,
	DRIFT,
	CUSTOM
}

enum GearState {
	NEUTRAL,
	REVERSE,
	FIRST,
	SECOND,
	THIRD,
	FOURTH,
	FIFTH,
	SIXTH,
	SEVENTH,
	EIGHTH
}

enum TractionControlLevel {
	OFF,
	MINIMAL,
	STANDARD,
	AGGRESSIVE,
	MAXIMUM
}

enum BrakeSystemType {
	BASIC,
	ABS,
	COURSE_ABS,
	RACING_ABS
}

# ============================================================================
# EXPORTED CONFIGURATION
# ============================================================================
@export_group("Vehicle Configuration")
@export var vehicle_mass: float = 1500.0
@export var center_of_mass_offset: Vector3 = Vector3.ZERO
@export var aerodynamic_drag_coefficient: float = 0.30
@export var frontal_area: float = 2.2
@export var rolling_resistance_coefficient: float = 0.015

@export_group("Engine Settings")
@export var max_engine_rpm: float = 8000.0
@export var idle_rpm: float = 800.0
@export var torque_curve_max: float = 450.0
@export var power_curve_max: float = 300.0
@export var gear_ratios: Array[float] = [3.5, 2.2, 1.5, 1.1, 0.9, 0.75, 0.65, 0.55]
@export var final_drive_ratio: float = 3.73
@export var clutch_damping_factor: float = 0.1

@export_group("Transmission Settings")
@export var shift_up_threshold_rpm: float = 7000.0
@export var shift_down_threshold_rpm: float = 2500.0
@export var auto_shift_enabled: bool = true
@export var gear_delay_ms: int = 200

@export_group("Steering Settings")
@export var steering_angle_max: float = 30.0
@export var steering_speed: float = 45.0
@export var steering_centering_force: float = 0.5
@export var ackermann_geometry: bool = true

@export_group("Braking System")
@export var front_brake_bias: float = 0.55
@export var rear_brake_bias: float = 0.45
@export var max_brake_pressure: float = 150.0
@export var brake_modulation: float = 0.85
@export var brake_fade_temperature: float = 300.0
@export var brake_fade_rate: float = 0.001

@export_group("Suspension & Wheels")
@export var suspension_stiffness: float = 100000.0
@export var suspension_damping: float = 15000.0
@export var suspension_compression: float = 0.1
@export var suspension_rebound: float = 0.15
@export var wheel_radius: float = 0.32
@export var wheel_track_width: float = 1.6
@export var wheel_base_length: float = 2.5

@export_group("Traction Control")
@export var traction_control_level: TractionControlLevel = TractionControlLevel.STANDARD
@export var max_wheel_slip_ratio: float = 0.25
@export var slip_recovery_rate: float = 0.1

@export_group("Drift Mode")
@export var drift_tendency: float = 0.3
@export var drift_recovery_rate: float = 0.05
@export var drift_angle_threshold: float = 15.0

@export_group("AI Behavior")
@export var ai_racing_line: bool = false
@export var ai_aggression_level: float = 0.5
@export var ai_throttle_smoothness: float = 0.3
@export var ai_brake_smoothness: float = 0.3
@export var ai_steering_smoothness: float = 0.2

# ============================================================================
# INTERNAL STATE VARIABLES
# ============================================================================
var _current_gear: GearState = GearState.NEUTRAL
var _target_gear: GearState = GearState.NEUTRAL
var _engine_rpm: float = 800.0
var _vehicle_speed: float = 0.0
var _engine_torque: float = 0.0
var _wheel_torque: float = 0.0
var _throttle_input: float = 0.0
var _brake_input: float = 0.0
var _steering_input: float = 0.0
var _clutch_input: float = 1.0

var _driving_mode: DrivingMode = DrivingMode.NORMAL
var _traction_control_active: bool = false
var _abs_active: bool = false
var _in_drift: bool = false
var _drift_angle: float = 0.0

var _front_wheels: Array[Node] = []
var _rear_wheels: Array[Node] = []
var _suspension_nodes: Array[Node] = []

var _engine_temperature: float = 90.0
var _brake_temperatures: Dictionary = {}
var _oil_pressure: float = 4.5

var _last_collision_time: float = 0.0
var _collision_velocity: Vector3 = Vector3.ZERO
var _is_on_ground: bool = false

# Physics simulation variables
var _angular_velocity: Vector3 = Vector3.ZERO
var _linear_acceleration: Vector3 = Vector3.ZERO
var _slip_ratios: Array[float] = []
var _grip_levels: Array[float] = []

# Timing and smoothing
var _input_buffer: Dictionary = {}
var _gear_change_timer: float = 0.0
var _shift_animation_progress: float = 0.0
var _smoothed_throttle: float = 0.0
var _smoothed_brake: float = 0.0
var _smoothed_steering: float = 0.0

# Cache for performance
var _drag_coefficient: float = 0.0
var _air_density: float = 1.225
var _rolling_resistance: float = 0.0

# ============================================================================
# INITIALIZATION
# ============================================================================
func _ready() -> void:
	_init_wheels()
	_init_brake_temperatures()
	_reset_simulation_state()
	_connect_signals_to_vehicle()
	
	# Initialize input buffer
	_input_buffer = {
		"throttle": 0.0,
		"brake": 0.0,
		"steering": 0.0,
		"clutch": 1.0,
		"gear_up": false,
		"gear_down": false,
		"handbrake": false
	}

func _init_wheels() -> void:
	# Find wheel children in hierarchy
	for child in get_children():
		if child.has_method("_get_wheel_type"):
			var wheel_type = child._get_wheel_type()
			if wheel_type == "front":
				_front_wheels.append(child)
			elif wheel_type == "rear":
				_rear_wheels.append(child)
			elif wheel_type == "suspension":
				_suspension_nodes.append(child)
	
	# Initialize slip ratios and grip levels
	_slip_ratios.resize(max(_front_wheels.size(), _rear_wheels.size()))
	_grip_levels.resize(max(_front_wheels.size(), _rear_wheels.size()))

func _init_brake_temperatures() -> void:
	for i in range(4):
		_brake_temperatures[i] = 90.0  # Starting at optimal temperature

func _reset_simulation_state() -> void:
	_current_gear = GearState.NEUTRAL
	_target_gear = GearState.NEUTRAL
	_engine_rpm = idle_rpm
	_vehicle_speed = 0.0
	_engine_torque = 0.0
	_wheel_torque = 0.0
	_in_drift = false
	_traction_control_active = false
	_abs_active = false

func _connect_signals_to_vehicle() -> void:
	# Connect to parent vehicle node if it exists
	var parent = get_parent()
	if parent and parent.has_signal("on_wheel_rotation_changed"):
		parent.on_wheel_rotation_changed.connect(_on_wheel_rotation_changed)
	if parent and parent.has_signal("on_suspension_compressed"):
		parent.on_suspension_compressed.connect(_on_suspension_compressed)

# ============================================================================
# MAIN PROCESS LOOP
# ============================================================================
func _physics_process(delta: float) -> void:
	# Input processing
	_process_inputs()
	
	# Apply smoothing to inputs
	_smooth_inputs(delta)
	
	# Calculate engine output based on current state
	_calculate_engine_output()
	
	# Handle transmission/gear logic
	_update_transmission(delta)
	
	# Calculate wheel forces
	_calculate_wheel_forces(delta)
	
	# Update vehicle dynamics
	_update_vehicle_dynamics(delta)
	
	# Apply traction control
	_apply_traction_control(delta)
	
	# Apply ABS if needed
	_apply_abs(delta)
	
	# Handle drift mechanics
	_update_drift_logic(delta)
	
	# Monitor engine and brake temperatures
	_monitor_temperatures(delta)
	
	# Update signals
	_emit_state_signals()
	
	# Collision handling
	_handle_collisions(delta)

# ============================================================================
# INPUT PROCESSING
# ============================================================================
func _process_inputs() -> void:
	# Get input values from InputManager singleton
	var input_manager = GameManager.get_node_or_null("/root/InputManager")
	if not input_manager:
		return
	
	# Read throttle/accelerator input
	var throttle_axis = input_manager.get_axis("throttle", "brake")
	if throttle_axis > 0:
		_input_buffer.throttle = abs(throttle_axis)
	else:
		_input_buffer.brake = abs(throttle_axis)
	
	# Read steering input
	_input_buffer.steering = input_manager.get_axis("left", "right")
	
	# Read clutch input (for manual mode)
	_input_buffer.clutch = input_manager.get_axis("clutch_up", "clutch_down")
	
	# Check gear shift inputs
	_input_buffer.gear_up = input_manager.is_action_pressed("gear_up")
	_input_buffer.gear_down = input_manager.is_action_pressed("gear_down")
	
	# Check handbrake
	_input_buffer.handbrake = input_manager.is_action_pressed("handbrake")

# ============================================================================
# INPUT SMOOTHING
# ============================================================================
func _smooth_inputs(delta: float) -> void:
	# Smooth throttle with acceleration curve
	var throttle_target = _input_buffer.throttle * brake_modulation
	_smoothed_throttle = lerp(_smoothed_throttle, throttle_target, delta * 5.0)
	
	# Smooth brake
	var brake_target = _input_buffer.brake * brake_modulation
	_smoothed_brake = lerp(_smoothed_brake, brake_target, delta * 8.0)
	
	# Smooth steering with deadzone handling
	if abs(_input_buffer.steering) < 0.05:
		_smoothed_steering = lerp(_smoothed_steering, 0.0, delta * 10.0)
	else:
		_smoothed_steering = lerp(_smoothed_steering, _input_buffer.steering, delta * steering_speed / 100.0)

# ============================================================================
# ENGINE OUTPUT CALCULATION
# ============================================================================
func _calculate_engine_output() -> void:
	# Calculate torque based on RPM curve
	var rpm_normalized = (_engine_rpm - idle_rpm) / (max_engine_rpm - idle_rpm)
	rpm_normalized = clamp(rpm_normalized, 0.0, 1.0)
	
	# Torque curve approximation (peak around 60% of max RPM)
	var torque_peak_position = 0.6
	var torque_curve = 1.0 - pow((rpm_normalized - torque_peak_position) / (1.0 - torque_peak_position), 2)
	torque_curve = max(torque_curve, 0.2)  # Minimum torque at extreme RPM
	
	# Apply throttle input
	_engine_torque = _smoothed_throttle * torque_curve_max * torque_curve
	
	# Reduce torque at high engine temperatures
	if _engine_temperature > 100.0:
		var temp_penalty = (_engine_temperature - 100.0) / 50.0
		_engine_torque *= (1.0 - temp_penalty * 0.5)
	
	# Apply clutch damping if engaged
	if _input_buffer.clutch < 1.0:
		_engine_torque *= _input_buffer.clutch * (1.0 - clutch_damping_factor)
	
	# Calculate wheel torque based on gear ratio
	var gear_ratio = _get_current_gear_ratio()
	_wheel_torque = _engine_torque * gear_ratio * final_drive_ratio * 0.95  # 5% drivetrain loss

# ============================================================================
# GEAR RATIO RETRIEVAL
# ============================================================================
func _get_current_gear_ratio() -> float:
	match _current_gear:
		GearState.NEUTRAL:
			return 0.0
		GearState.REVERSE:
			return gear_ratios[0] * -1.0
		GearState.FIRST:
			return gear_ratios[0]
		GearState.SECOND:
			return gear_ratios[1]
		GearState.THIRD:
			return gear_ratios[2]
		GearState.FOURTH:
			return gear_ratios[3]
		GearState.FIFTH:
			return gear_ratios[4]
		GearState.SIXTH:
			return gear_ratios[5]
		GearState.SEVENTH:
			return gear_ratios[6]
		GearState.EIGHTH:
			return gear_ratios[7]
		_:
			return 0.0

# ============================================================================
# TRANSMISSION UPDATES
# ============================================================================
func _update_transmission(delta: float) -> void:
	# Manual gear shifting
	if not auto_shift_enabled:
		if _input_buffer.gear_up and _can_shift_gear_up():
			_shift_gear(1)
		elif _input_buffer.gear_down and _can_shift_gear_down():
			_shift_gear(-1)
	
	# Auto shifting logic
	if auto_shift_enabled:
		_auto_shift_logic(delta)
	
	# Handle gear change timer
	if _gear_change_timer > 0:
		_gear_change_timer -= delta
		_shift_animation_progress = 1.0 - (_gear_change_timer / float(gear_delay_ms / 1000.0))
		if _gear_change_timer <= 0:
			_current_gear = _target_gear
			_gear_change_timer = 0.0
			_shift_animation_progress = 0.0
			gear_changed.emit(_get_gear_number())

# ============================================================================
# AUTO SHIFT LOGIC
# ============================================================================
func _auto_shift_logic(delta: float) -> void:
	if _current_gear == GearState.NEUTRAL or _current_gear == GearState.REVERSE:
		return
	
	# Shift up threshold
	if _engine_rpm >= shift_up_threshold_rpm and _get_current_gear_number() < gear_ratios.size():
		_shift_gear(1)
	
	# Downshift when RPM too low
	elif _engine_rpm <= shift_down_threshold_rpm and _get_current_gear_number() > 1:
		_shift_gear(-1)
	
	# Emergency downshift on heavy braking
	elif _smoothed_brake > 0.8 and _engine_rpm < idle_rpm + 1000:
		var target_gear = _get_optimal_downshift_gear()
		if target_gear != _get_current_gear_number():
			_target_gear = _get_gear_from_number(target_gear)
			_gear_change_timer = gear_delay_ms / 1000.0

# ============================================================================
# GEAR SHIFT VALIDATION
# ============================================================================
func _can_shift_gear_up() -> bool:
	var current_number = _get_current_gear_number()
	return current_number < gear_ratios.size() and _engine_rpm > idle_rpm

func _can_shift_gear_down() -> bool:
	var current_number = _get_current_gear_number()
	return current_number > 1 and _engine_rpm > idle_rpm

func _get_optimal_downshift_gear() -> int:
	# Find gear where RPM would be near optimal range after downshift
	for i in range(_get_current_gear_number() - 1, 0, -1):
		var hypothetical_rpm = _engine_rpm * (gear_ratios[i] / gear_ratios[_get_current_gear_number() - 1])
		if hypothetical_rpm < max_engine_rpm and hypothetical_rpm > idle_rpm:
			return i
	return _get_current_gear_number()

# ============================================================================
# GEAR SHIFT EXECUTION
# ============================================================================
func _shift_gear(direction: int) -> void:
	var new_gear_number = _get_current_gear_number() + direction
	
	if new_gear_number < 1 or new_gear_number > gear_ratios.size():
		return
	
	_target_gear = _get_gear_from_number(new_gear_number)
	_gear_change_timer = gear_delay_ms / 1000.0
	
	# Emit signal for AI/spectator feedback
	engine_rpm_changed.emit(_engine_rpm)

func _get_gear_from_number(number: int) -> GearState:
	match number:
		0: return GearState.NEUTRAL
		-1: return GearState.REVERSE
		1: return GearState.FIRST
		2: return GearState.SECOND
		3: return GearState.THIRD
		4: return GearState.FOURTH
		5: return GearState.FIFTH
		6: return GearState.SIXTH
		7: return GearState.SEVENTH
		8: return GearState.EIGHTH
		_: return GearState.NEUTRAL

func _get_current_gear_number() -> int:
	match _current_gear:
		GearState.FIRST: return 1
		GearState.SECOND: return 2
		GearState.THIRD: return 3
		GearState.FOURTH: return 4
		GearState.FIFTH: return 5
		GearState.SIXTH: return 6
		GearState.SEVENTH: return 7
		GearState.EIGHTH: return 8
		GearState.NEUTRAL: return 0
		GearState.REVERSE: return -1
		_: return 0

# ============================================================================
# WHEEL FORCE CALCULATION
# ============================================================================
func _calculate_wheel_forces(delta: float) -> void:
	# Calculate drag coefficient
	_drag_coefficient = aerodynamic_drag_coefficient * frontal_area * 0.5 * _air_density
	
	# Calculate rolling resistance
	_rolling_resistance = vehicle_mass * GRAVITY_Y.abs() * rolling_resistance_coefficient
	
	# Distribute drive force between front and rear wheels
	var total_drive_force = _wheel_torque / wheel_radius
	var front_drive_force = total_drive_force * (1.0 - front_brake_bias)
	var rear_drive_force = total_drive_force * front_brake_bias
	
	# Apply wheel-specific forces
	for i in range(_front_wheels.size()):
		if i < _front_wheels.size():
			_apply_wheel_force(_front_wheels[i], front_drive_force / _front_wheels.size(), true)
	
	for i in range(_rear_wheels.size()):
		if i < _rear_wheels.size():
			_apply_wheel_force(_rear_wheels[i], rear_drive_force / _rear_wheels.size(), false)
	
	# Apply brake forces
	_apply_brake_forces(delta)

func _apply_wheel_force(wheel: Node, force: float, is_front: bool) -> void:
	if not wheel or not wheel.has_method("apply_drive_force"):
		return
	
	wheel.apply_drive_force(force, is_front)
	
	# Calculate slip ratio
	var wheel_speed = force / (wheel_mass * wheel_radius) if wheel.has_method("get_wheel_mass") else 0.0
	var slip_ratio = (wheel_speed - _vehicle_speed) / max(abs(_vehicle_speed), 1.0)
	slip_ratio = clamp(slip_ratio, -0.5, 0.5)
	
	# Update slip tracking
	if wheel.get_index() < _slip_ratios.size():
		_slip_ratios[wheel.get_index()] = slip_ratio

func _apply_brake_forces(delta: float) -> void:
	var total_brake_force = _smoothed_brake * max_brake_pressure / wheel_radius
	
	# Distribute brake force between axles
	var front_brake_force = total_brake_force * front_brake_bias
	var rear_brake_force = total_brake_force * rear_brake_bias
	
	# Apply to wheels
	for i in range(_front_wheels.size()):
		if i < _front_wheels.size():
			_apply_wheel_brake_force(_front_wheels[i], front_brake_force / _front_wheels.size())
	
	for i in range(_rear_wheels.size()):
		if i < _rear_wheels.size():
			_apply_wheel_brake_force(_rear_wheels[i], rear_brake_force / _rear_wheels.size())

func _apply_wheel_brake_force(wheel: Node, brake_force: float) -> void:
	if not wheel or not wheel.has_method("apply_brake_force"):
		return
	
	wheel.apply_brake_force(brake_force)
	
	# Update brake temperature
	var wheel_index = wheel.get_index()
	if wheel_index < 4:
		_brake_temperatures[wheel_index] += brake_force * 0.001 * delta
		_brake_temperatures[wheel_index] = min(_brake_temperatures[wheel_index], 600.0)

# ============================================================================
# VEHICLE DYNAMICS UPDATE
# ============================================================================
func _update_vehicle_dynamics(delta: float) -> void:
	# Calculate aerodynamic drag
	var drag_force = _drag_coefficient * _vehicle_speed * _vehicle_speed
	drag_force = min(drag_force, _engine_torque / wheel_radius * 2.0)  # Cap at available torque
	
	# Calculate net acceleration
	var net_force = _wheel_torque / wheel_radius - drag_force - _rolling_resistance
	_linear_acceleration = net_force / vehicle_mass
	
	# Update velocity
	_vehicle_speed += _linear_acceleration * delta
	_vehicle_speed = clamp(_vehicle_speed, 0.0, max_engine_rpm / (final_drive_ratio * gear_ratios.max() / wheel_radius) * 3.6)  # Convert to km/h
	
	# Update engine RPM based on gear ratio
	if _current_gear != GearState.NEUTRAL:
		var gear_ratio = _get_current_gear_ratio()
		var theoretical_rpm = (_vehicle_speed * 1000.0) / (wheel_radius * gear_ratio * final_drive_ratio)
		
		# Blend theoretical RPM with actual RPM
		if _input_buffer.throttle > 0.1:
			_engine_rpm = lerp(_engine_rpm, theoretical_rpm, delta * 10.0)
		else:
			_engine_rpm = lerp(_engine_rpm, theoretical_rpm, delta * 5.0)
	else:
		# Engine idling or revving
		if _input_buffer.throttle > 0.0:
			_engine_rpm = lerp(_engine_rpm, idle_rpm + _smoothed_throttle * (max_engine_rpm - idle_rpm), delta * 15.0)
		else:
			_engine_rpm = lerp(_engine_rpm, idle_rpm, delta * 10.0)
	
	# Clamp RPM
	_engine_rpm = clamp(_engine_rpm, idle_rpm, max_engine_rpm)
	
	# Apply steering effect to angular velocity
	var steering_effect = _smoothed_steering * steering_angle_max * 0.1
	_angular_velocity.z = lerp(_angular_velocity.z, steering_effect, delta * 2.0)

# ============================================================================
# TRACTION CONTROL SYSTEM
# ============================================================================
func _apply_traction_control(delta: float) -> void:
	var slip_threshold = max_wheel_slip_ratio * traction_control_level / 5.0
	var max_slip = 0.0
	
	# Check all wheel slip ratios
	for slip_ratio in _slip_ratios:
		max_slip = max(max_slip, abs(slip_ratio))
	
	_traction_control_active = max_slip > slip_threshold
	
	if _traction_control_active:
		var tc_reduction = (max_slip - slip_threshold) / max_wheel_slip_ratio
		tc_reduction = clamp(tc_reduction, 0.0, 1.0)
		
		# Reduce engine torque based on slip
		_engine_torque *= (1.0 - tc_reduction * 0.5)
		
		# Adjust brake bias for individual wheels if needed
		_adjust_brake_for_slip()
		
		traction_control_triggered.emit(true)
	else:
		traction_control_triggered.emit(false)

func _adjust_brake_for_slip() -> void:
	# Apply selective braking to wheels with excessive slip
	for i in range(_slip_ratios.size()):
		if abs(_slip_ratios[i]) > max_wheel_slip_ratio * 0.8:
			# Apply minor brake pressure to spinning wheel
			_apply_wheel_brake_force(get_child(i), max_brake_pressure * 0.2 * delta)

# ============================================================================
# ANTI-LOCK BRAKING SYSTEM
# ============================================================================
func _apply_abs(delta: float) -> void:
	if brake_system_type != BrakeSystemType.ABS and brake_system_type != BrakeSystemType.COURSE_ABS and brake_system_type != BrakeSystemType.RACING_ABS:
		return
	
	var abs_threshold = 0.15
	var max_slip_during_brake = 0.0
	
	# Check for wheel lockup during braking
	for i in range(_slip_ratios.size()):
		if _smoothed_brake > 0.3 and _slip_ratios[i] < -abs_threshold:
			max_slip_during_brake = max(max_slip_during_brake, abs(_slip_ratios[i]))
	
	_abs_active = max_slip_during_brake > abs_threshold
	
	if _abs_active:
		# Pulse brakes to prevent lockup
		var pulse_freq = 15.0  # Hz
		var pulse_phase = (Time.get_ticks_msec() / 1000.0 * pulse_freq) % 1.0
		
		if pulse_phase < 0.3:
			# Reduce brake pressure
			_smoothed_brake *= 0.7
		else:
			# Restore brake pressure
			_smoothed_brake = lerp(_smoothed_brake, _input_buffer.brake, delta * 20.0)
		
		anti_lock_brake_triggered.emit(true)
	else:
		anti_lock_brake_triggered.emit(false)

# ============================================================================
# DRIFT MECHANICS
# ============================================================================
func _update_drift_logic(delta: float) -> void:
	if _driving_mode != DrivingMode.DRIFT and not _input_buffer.handbrake:
		_in_drift = false
		return
	
	# Detect drift entry
	var lateral_acceleration = _angular_velocity.length() * _vehicle_speed
	var drift_entry_threshold = drift_angle_threshold * 0.5
	
	if lateral_acceleration > drift_entry_threshold and _smoothed_steering.abs() > 0.3:
		if not _in_drift:
			_in_drift = true
			drift_started.emit(drift_angle)
	
	if _in_drift:
		# Maintain drift angle
		_drift_angle = lerp(_drift_angle, _smoothed_steering * drift_tendency, delta * drift_recovery_rate)
		
		# Reduce grip during drift
		for i in range(_grip_levels.size()):
			_grip_levels[i] = lerp(_grip_levels[i], 0.6, delta * 0.1)
		
		# Apply drift recovery when releasing handbrake
		if not _input_buffer.handbrake and _smoothed_throttle < 0.5:
			_drift_angle = lerp(_drift_angle, 0.0, delta * drift_recovery_rate)
			if abs(_drift_angle) < 0.1:
				_in_drift = false
				drift_ended.emit()
	else:
		# Recover grip
		for i in range(_grip_levels.size()):
			_grip_levels[i] = lerp(_grip_levels[i], 1.0, delta * 0.2)

# ============================================================================
# TEMPERATURE MONITORING
# ============================================================================
func _monitor_temperatures(delta: float) -> void:
	# Engine temperature update
	var cooling_rate = 0.5
	var heating_rate = _engine_torque * 0.001
	
	if _engine_temperature < 90.0:
		_engine_temperature += heating_rate * delta
	elif _engine_temperature > 100.0:
		_engine_temperature -= cooling_rate * delta
	else:
		# Optimal range
		pass
	
	# Ambient cooling
	_engine_temperature -= cooling_rate * 0.5 * delta
	_engine_temperature = clamp(_engine_temperature, 70.0, 130.0)
	
	# Brake temperature decay
	for key in _brake_temperatures:
		_brake_temperatures[key] -= 0.3 * delta
		_brake_temperatures[key] = max(_brake_temperatures[key], 80.0)

# ============================================================================
# COLLISION HANDLING
# ============================================================================
func _handle_collisions(delta: float) -> void:
	# Check for collision events from vehicle body
	if has_signal("collision_impact"):
		collision_impact.emit(_collision_velocity, Vector3.ZERO)
	
	# Apply collision response
	if _collision_velocity.length() > 5.0:
		# Significant impact - apply damage and shake
		_engine_torque *= 0.5
		_linear_acceleration = _collision_velocity * 0.1
	
	# Reset collision state after delay
	if _collision_velocity.length() > 0 and Time.get_ticks_msec() - _last_collision_time > 500:
		_collision_velocity = Vector3.ZERO

func _on_collision(velocity: Vector3, point: Vector3) -> void:
	_last_collision_time = Time.get_ticks_msec()
	_collision_velocity = velocity
	collision_impact.emit(velocity, point)

# ============================================================================
# STATE SIGNALS
# ============================================================================
func _emit_state_signals() -> void:
	engine_rpm_changed.emit(_engine_rpm)
	speed_changed.emit(_vehicle_speed)

# ============================================================================
# WHEEL AND SUSPENSION EVENTS
# ============================================================================
func _on_wheel_rotation_changed(wheel_index: int, rotation: float) -> void:
	# Update wheel-specific data
	if wheel_index < _slip_ratios.size():
		_slip_ratios[wheel_index] = rotation

func _on_suspension_compressed(wheel_index: int, compression: float) -> void:
	# Update suspension state
	if wheel_index < _suspension_nodes.size():
		var suspension_node = _suspension_nodes[wheel_index]
		if suspension_node.has_method("set_compression"):
			suspension_node.set_compression(compression)

# ============================================================================
# PUBLIC API - EXTERNAL CONTROLS
# ============================================================================
func set_driving_mode(mode: DrivingMode) -> void:
	_driving_mode = mode
	match mode:
		DrivingMode.NORMAL:
			traction_control_level = TractionControlLevel.STANDARD
			drift_tendency = 0.1
		DrivingMode.SPORT:
			traction_control_level = TractionControlLevel.MINIMAL
			drift_tendency = 0.2
		DrivingMode.RACE:
			traction_control_level = TractionControlLevel.OFF
			drift_tendency = 0.3
		DrivingMode.DRASTIC:
			traction_control_level = TractionControlLevel.OFF
			drift_tendency = 0.5
		DrivingMode.DRIFT:
			traction_control_level = TractionControlLevel.OFF
			drift_tendency = 0.8
		DrivingMode.CUSTOM:
			pass  # Use custom settings

func get_vehicle_speed_kmh() -> float:
	return _vehicle_speed

func get_engine_rpm() -> float:
	return _engine_rpm

func get_current_gear() -> int:
	return _get_current_gear_number()

func get_current_gear_string() -> String:
	match _current_gear:
		GearState.NEUTRAL: return "N"
		GearState.REVERSE: return "R"
		GearState.FIRST: return "1"
		GearState.SECOND: return "2"
		GearState.THIRD: return "3"
		GearState.FOURTH: return "4"
		GearState.FIFTH: return "5"
		GearState.SIXTH: return "6"
		GearState.SEVENTH: return "7"
		GearState.EIGHTH: return "8"
		_: return "?"

func is_in_drift() -> bool:
	return _in_drift

func get_drift_angle() -> float:
	return _drift_angle

func reset_vehicle() -> void:
	_reset_simulation_state()
	_engine_temperature = 90.0
	for key in _brake_temperatures:
		_brake_temperatures[key] = 90.0
	_smoothed_throttle = 0.0
	_smoothed_brake = 0.0
	_smoothed_steering = 0.0

func force_gear(gear_num: int) -> void:
	if gear_num < 0 or gear_num > gear_ratios.size():
		return
	
	_target_gear = _get_gear_from_number(gear_num)
	_gear_change_timer = gear_delay_ms / 1000.0

# ============================================================================
# DEBUG / VISUALIZATION HELPERS
# ============================================================================
func debug_get_stats() -> Dictionary:
	return {
		"rpm": _engine_rpm,
		"speed": _vehicle_speed,
		"gear": _get_current_gear_number(),
		"throttle": _smoothed_throttle,
		"brake": _smoothed_brake,
		"steering": _smoothed_steering,
		"engine_temp": _engine_temperature,
		"in_drift": _in_drift,
		"tc_active": _traction_control_active,
		"abs_active": _abs_active
	}

func debug_set_debug_mode(enabled: bool) -> void:
	# Enable/disable debug visualizations
	pass

</File>