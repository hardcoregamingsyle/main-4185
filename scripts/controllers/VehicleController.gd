extends CharacterBody2D
class_name VehicleController

## VehicleController - Core vehicle physics controller using PhysicsSettings constants
## Handles throttle, brake, steering inputs, wheel forces, and gear shifting logic
## Copyright 2026 Thalamus Racing Simulator Project

# ============================================================================
# SIGNALS
# ============================================================================
signal speed_changed(new_speed: float)
signal gear_changed(old_gear: int, new_gear: int)
signal engine_rpm_changed(rpm: float)
signal vehicle_damage_taken(damage_amount: float)
signal tire_slip_changed(left_tire: float, right_tire: float)
signal collision_detected(impact_velocity: Vector2, impact_point: Vector2)

# ============================================================================
# CONSTANTS FROM PHYSICS SETTINGS
# ============================================================================
const GRAVITY := PhysicsSettings.gravity
const DEFAULT_VEHICLE_MASS := PhysicsSettings.default_vehicle_mass
const MAX_THROTTLE_FORCE := PhysicsSettings.max_throttle_force
const MAX_BRAKE_FORCE := PhysicsSettings.max_brake_force
const MAX_STEERING_ANGLE := PhysicsSettings.max_steering_angle_rad
const STEERING_SPEED := PhysicsSettings.steering_speed
const ACCELERATION_RATE := PhysicsSettings.acceleration_rate
const BRAKING_RATE := PhysicsSettings.braking_rate
const TRACTION_FACTOR := PhysicsSettings.traction_factor
const DRAG_COEFFICIENT := PhysicsSettings.drag_coefficient
const ROLLING_RESISTANCE := PhysicsSettings.rolling_resistance
const GEAR_RATIO_MIN := PhysicsSettings.gear_ratio_min
const GEAR_RATIO_MAX := PhysicsSettings.gear_ratio_max
const CLIP engages clutch at low RPM
const REDLINE_RPM := PhysicsSettings.redline_rpm
const IDLE_RPM := PhysicsSettings.idle_rpm
const SHIFT_POINT_LOW := PhysicsSettings.shift_point_low
const SHIFT_POINT_HIGH := PhysicsSettings.shift_point_high

# ============================================================================
# GEAR RATIOS (6-speed transmission)
# ============================================================================
var _gear_ratios: Array[float] = [
	3.85,  # 1st gear
	2.45,  # 2nd gear
	1.75,  # 3rd gear
	1.35,  # 4th gear
	1.05,  # 5th gear
	0.85   # 6th gear
]

var _reverse_ratio: float = 4.20
var _final_drive: float = 3.73

# ============================================================================
# STATE VARIABLES
# ============================================================================
var current_speed: float = 0.0  # m/s
var max_speed: float = 0.0      # m/s
var current_gear: int = 0       # 0=neutral, 1-6=fwd gears, -1=reverse
var target_gear: int = 0
var engine_rpm: float = IDLE_RPM
var target_rpm: float = IDLE_RPM
var throttle_input: float = 0.0 # -1.0 to 1.0
var brake_input: float = 0.0    # 0.0 to 1.0
var steering_input: float = 0.0 # -1.0 to 1.0
var clutch_engaged: bool = true
var clutch_slippage: float = 0.0
var traction_control_active: bool = false

# Wheel variables for force application
var front_wheel_angle: float = 0.0
var rear_wheel_force: float = 0.0
var left_tire_slip: float = 0.0
var right_tire_slip: float = 0.0

# Physics state
var acceleration: Vector2 = Vector2.ZERO
var velocity: Vector2 = Vector2.ZERO
var drag_force: Vector2 = Vector2.ZERO
var friction_force: Vector2 = Vector2.ZERO
var applied_force: Vector2 = Vector2.ZERO

# Driving mode
var driving_mode: String = "normal"  # normal, sport, race, drift, offroad

# ============================================================================
# INITIALIZATION
# ============================================================================
func _ready() -> void:
	_process_mode = ProcessModeEnum.ALWAYS
	
	# Initialize from physics settings
	max_speed = _calculate_max_speed()
	
	# Connect to global signals if needed
	GameManager.game_state_changed.connect(_on_game_state_changed)

# ============================================================================
# MAIN GAME LOOP
# ============================================================================
func _physics_process(delta: float) -> void:
	if not is_ready_to_simulate():
		return
	
	# Read input
	_read_inputs(delta)
	
	# Update physics
	_update_physics(delta)
	
	# Handle gear shifting
	_handle_gear_shifting(delta)
	
	# Apply forces
	_apply_forces(delta)
	
	# Move character body
	move_and_slide()
	
	# Update visual state
	_update_visual_state()

# ============================================================================
# INPUT HANDLING
# ============================================================================
func _read_inputs(delta: float) -> void:
	# Get normalized input values
	throttle_input = InputManager.get_axis("throttle")
	brake_input = InputManager.get_axis("brake")
	steering_input = InputManager.get_axis("steering")
	
	# Clamp inputs to valid range
	throttle_input = clampf(throttle_input, -1.0, 1.0)
	brake_input = clampf(brake_input, 0.0, 1.0)
	steering_input = clampf(steering_input, -1.0, 1.0)
	
	# Clutch control (shift key or button)
	if Input.is_action_just_pressed("clutch"):
		clutch_engaged = !clutch_engaged
	
	# Gear shift up/down
	if Input.is_action_just_pressed("gear_up"):
		_request_gear_shift(current_gear + 1)
	
	if Input.is_action_just_pressed("gear_down"):
		_request_gear_shift(current_gear - 1)
	
	# Emergency brake / handbrake
	var handbrake = Input.is_action_pressed("handbrake")
	if handbrake:
		brake_input = max(brake_input, 0.8)
	
	# Traction control toggle
	if Input.is_action_just_pressed("toggle_traction_control"):
		traction_control_active = !traction_control_active

# ============================================================================
# PHYSICS UPDATE
# ============================================================================
func _update_physics(delta: float) -> void:
	# Calculate engine torque based on RPM and throttle
	var engine_torque: float = _calculate_engine_torque(engine_rpm, throttle_input)
	
	# Calculate wheel torque considering gearing and clutch
	var wheel_torque: float = _calculate_wheel_torque(engine_torque)
	
	# Calculate drive force
	rear_wheel_force = _calculate_drive_force(wheel_torque)
	
	# Apply aerodynamic drag
	drag_force = _calculate_drag_force(current_speed)
	
	# Apply rolling resistance
	friction_force = _calculate_friction_force()
	
	# Calculate net acceleration
	acceleration = Vector2.ZERO
	if abs(rear_wheel_force) > 0:
		# Drive force in vehicle direction
		var forward_dir: Vector2 = Vector2.RIGHT.rotated(rotation)
		acceleration += (rear_wheel_force / DEFAULT_VEHICLE_MASS) * forward_dir
	
	# Apply drag (opposite to velocity)
	acceleration -= drag_force / DEFAULT_VEHICLE_MASS
	
	# Apply friction (opposite to motion)
	acceleration -= friction_force / DEFAULT_VEHICLE_MASS
	
	# Limit maximum acceleration
	acceleration = acceleration.limit_length(ACCELERATION_RATE * GRAVITY)
	
	# Update velocity
	velocity += acceleration * delta
	
	# Cap speed to max
	current_speed = velocity.length()
	if current_speed > max_speed:
		current_speed = max_speed
		velocity = velocity.normalized() * max_speed
	
	# Update RPM based on speed and gear
	_update_engine_rpm(delta)

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================
func _handle_gear_shifting(delta: float) -> void:
	# Automatic gear shifting
	if GameManager.debug_mode or Input.is_action_pressed("auto_shift"):
		_auto_shift()
	else:
		_manual_shift_check()
	
	# Smooth gear transition
	if current_gear != target_gear:
		_transition_gear(delta)

func _auto_shift() -> void:
	# Upshift logic
	if current_gear < 6 and engine_rpm > SHIFT_POINT_HIGH and throttle_input > 0.1:
		_request_gear_shift(current_gear + 1)
	
	# Downshift logic
	elif current_gear > 0 and engine_rpm < SHIFT_POINT_LOW and throttle_input < 0.5:
		_request_gear_shift(current_gear - 1)
	
	# Neutral at very low speeds
	elif current_speed < 1.0 and current_gear != 0:
		_request_gear_shift(0)

func _manual_shift_check() -> void:
	# Manual override for automatic downshifts under braking
	if brake_input > 0.3 and engine_rpm < SHIFT_POINT_LOW and current_gear > 1:
		_request_gear_shift(current_gear - 1)

func _request_gear_shift(new_gear: int) -> void:
	# Validate gear request
	if new_gear == current_gutter:
		return
	
	if new_gear < -1 or new_gear > 6:
		return
	
	# Prevent reverse while moving forward
	if new_gear == -1 and current_speed > 2.0:
		return
	
	target_gear = new_gear

func _transition_gear(delta: float) -> void:
	# Simulate gear shift time and clutch engagement
	var shift_time: float = 0.15  # seconds
	
	# Disengage clutch during shift
	if abs(current_gear - target_gear) > 0:
		clutch_engaged = false
		clutch_slippage = min(clutch_slippage + delta * 5.0, 1.0)
		
		# Re-engage after shift delay
		if clutch_slippage >= 1.0:
			current_gear = target_gear
			clutch_engaged = true
			clutch_slippage = 0.0
			
			# Emit signal
			gear_changed.emit(current_gear, target_gear)
			
			# Audio feedback
			if AudioManager:
				AudioManager.play_sound("gear_shift")
	else:
		clutch_engaged = true
		current_gear = target_gear

# ============================================================================
# FORCE CALCULATIONS
# ============================================================================
func _calculate_engine_torque(rpm: float, throttle: float) -> float:
	# Simplified torque curve model
	var base_torque: float = 400.0  # Nm
	var rpm_normalized: float = (rpm - IDLE_RPM) / (REDLINE_RPM - IDLE_RPM)
	rpm_normalized = clampf(rpm_normalized, 0.0, 1.0)
	
	# Torque curve (peak around 4000-5000 RPM)
	var torque_curve: float = 1.0 - pow(rpm_normalized - 0.5, 2.0) * 4.0
	torque_curve = clampf(torque_curve, 0.3, 1.0)
	
	return base_torque * throttle * torque_curve

func _calculate_wheel_torque(engine_torque: float) -> float:
	if not clutch_engaged:
		return 0.0
	
	# Calculate total gear ratio
	var gear_ratio: float = _get_current_gear_ratio()
	var total_ratio: float = gear_ratio * _final_drive
	
	# Apply torque multiplication
	var wheel_torque: float = engine_torque * total_ratio
	
	# Account for drivetrain efficiency losses
	wheel_torque *= 0.85  # 85% efficiency
	
	return wheel_torque

func _calculate_drive_force(wheel_torque: float) -> float:
	# Convert wheel torque to linear force
	# F = T / r where r is wheel radius
	var wheel_radius: float = 0.33  # meters (typical car tire)
	
	var drive_force: float = wheel_torque / wheel_radius
	
	# Apply traction limits
	drive_force = _apply_traction_limits(drive_force)
	
	return drive_force

func _apply_traction_limits(force: float) -> float:
	if not traction_control_active:
		return force
	
	# Calculate maximum traction force
	var max_traction: float = DEFAULT_VEHICLE_MASS * GRAVITY * TRACTION_FACTOR
	
	# Clip force to available traction
	if abs(force) > max_traction:
		force = sign(force) * max_traction
		
		# Report slip
		left_tire_slip = abs(force - _abs(force)) / max_traction
		right_tire_slip = left_tire_slip
		
		tire_slip_changed.emit(left_tire_slip, right_tire_slip)
	
	return force

func _calculate_drag_force(speed: float) -> Vector2:
	if speed <= 0:
		return Vector2.ZERO
	
	# Aerodynamic drag: F = 0.5 * rho * v^2 * Cd * A
	# Simplified: proportional to square of speed
	var air_density: float = 1.225  # kg/m^3
	var frontal_area: float = 2.2   # m^2 (typical sports car)
	
	var drag_magnitude: float = 0.5 * air_density * pow(speed, 2.0) * DRAG_COEFFICIENT * frontal_area
	
	# Opposite to velocity direction
	return -velocity.normalized() * drag_magnitude

func _calculate_friction_force() -> Vector2:
	# Rolling resistance: F = Crr * N where N = mg
	var normal_force: float = DEFAULT_VEHICLE_MASS * GRAVITY
	var friction_magnitude: float = ROLLING_RESISTANCE * normal_force
	
	# Opposite to velocity direction
	if current_speed > 0:
		return -velocity.normalized() * friction_magnitude
	
	return Vector2.ZERO

# ============================================================================
# GEAR RATIO HELPERS
# ============================================================================
func _get_current_gear_ratio() -> float:
	if current_gear == 0:
		return 0.0
	
	if current_gear == -1:
		return _reverse_ratio
	
	return _gear_ratios[current_gear - 1]

func _calculate_max_speed() -> float:
	# Calculate theoretical max speed in top gear
	# Speed = (RPM * TireCircumference) / (GearRatio * FinalDrive)
	var tire_circumference: float = PI * 0.33 * 2.0  # diameter * PI
	
	var top_gear_ratio: float = _gear_ratios[5]  # 6th gear
	var max_speed_calc: float = (REDLINE_RPM * tire_circumference) / (top_gear_ratio * _final_drive * 3.6)  # convert to km/h
	
	return max_speed_calc * 0.9  # 90% of theoretical due to drag

func _update_engine_rpm(delta: float) -> void:
	# Calculate target RPM based on speed and gear
	var wheel_rpm: float = _calculate_wheel_rpm()
	var gear_ratio: float = _get_current_gear_ratio()
	
	if gear_ratio > 0:
		target_rpm = wheel_rpm * gear_ratio * _final_drive
	else:
		target_rpm = IDLE_RPM if throttle_input <= 0 else min(target_rpm, REDLINE_RPM)
	
	# Smooth RPM transition
	engine_rpm = lerp(engine_rpm, target_rpm, delta * 10.0)
	
	# Clamp to safe range
	engine_rpm = clampf(engine_rpm, IDLE_RPM, REDLINE_RPM)
	
	# Over-rev protection
	if engine_rpm > REDLINE_RPM * 0.95:
		throttle_input = max(throttle_input * 0.5, 0.0)  # Cut throttle
	
	# Idle stabilization
	if current_gear == 0 and abs(throttle_input) < 0.1:
		engine_rpm = lerp(engine_rpm, IDLE_RPM, delta * 5.0)
	
	# Emit signal
	engine_rpm_changed.emit(engine_rpm)

func _calculate_wheel_rpm() -> float:
	# RPM = (Speed * 1000) / (TireCircumference * 60)
	# Speed in m/s, circumference in meters
	var tire_circumference: float = PI * 0.33 * 2.0
	var wheel_rpm: float = (current_speed * 60.0) / tire_circumference
	
	return wheel_rpm

# ============================================================================
# VISUAL UPDATES
# ============================================================================
func _update_visual_state() -> void:
	# Update front wheel angle (steering)
	front_wheel_angle = lerp(front_wheel_angle, 
		steering_input * MAX_STEERING_ANGLE, 
		STEERING_SPEED * get_delta())
	
	# Update speed display value
	speed_changed.emit(current_speed)

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================
func _abs(value: float) -> float:
	return value if value > 0 else -value

func is_ready_to_simulate() -> bool:
	return GameManager.current_state == GameManager.GameState.RACE_ACTIVE

func reset_vehicle() -> void:
	current_speed = 0.0
	max_speed = _calculate_max_speed()
	current_gear = 0
	target_gear = 0
	engine_rpm = IDLE_RPM
	target_rpm = IDLE_RPM
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	clutch_engaged = true
	clutch_slippage = 0.0
	velocity = Vector2.ZERO
	acceleration = Vector2.ZERO
	drag_force = Vector2.ZERO
	friction_force = Vector2.ZERO
	applied_force = Vector2.ZERO
	front_wheel_angle = 0.0
	rear_wheel_force = 0.0
	left_tire_slip = 0.0
	right_tire_slip = 0.0

func set_driving_mode(mode: String) -> void:
	if mode == "sport":
		TRACTION_FACTOR = 0.85
		BRAKING_RATE = 1.2
	elif mode == "race":
		TRACTION_FACTOR = 0.90
		BRAKING_RATE = 1.5
	elif mode == "drift":
		TRACTION_FACTOR = 0.65
		BRAKING_RATE = 1.0
	elif mode == "offroad":
		TRACTION_FACTOR = 0.70
		BRAKING_RATE = 0.8
	elif mode == "normal":
		TRACTION_FACTOR = PhysicsSettings.traction_factor
		BRAKING_RATE = PhysicsSettings.braking_rate
	
	driving_mode = mode

# ============================================================================
# COLLISION HANDLING
# ============================================================================
func _on_collision_entered(body: Node) -> void:
	var impact_velocity: Vector2 = velocity - (body as Node2D).velocity if body is Node2D else velocity
	var impact_point: Vector2 = global_position
	
	collision_detected.emit(impact_velocity, impact_point)
	
	# Calculate damage
	var impact_energy: float = 0.5 * DEFAULT_VEHICLE_MASS * pow(impact_velocity.length(), 2.0)
	var damage: float = impact_energy * 0.001
	
	if damage > 0:
		vehicle_damage_taken.emit(damage)
		if AudioManager:
			AudioManager.play_sound("collision")

func _on_collision_started(other: CollisionShape2D) -> void:
	# Continuous collision processing
	pass

# ============================================================================
# DEBUG & TESTING
# ============================================================================
func debug_get_stats() -> Dictionary:
	return {
		"speed_kmh": current_speed * 3.6,
		"speed_ms": current_speed,
		"gear": current_gear,
		"rpm": engine_rpm,
		"throttle": throttle_input,
		"brake": brake_input,
		"steering": steering_input,
		"clutch_engaged": clutch_engaged,
		"traction_active": traction_control_active,
		"mode": driving_mode
	}

func set_test_parameters(params: Dictionary) -> void:
	if "max_speed" in params:
		max_speed = params["max_speed"]
	if "engine_rpm" in params:
		engine_rpm = params["engine_rpm"]
	if "throttle" in params:
		throttle_input = params["throttle"]
	if "brake" in params:
		brake_input = params["brake"]
	if "steering" in params:
		steering_input = params["steering"]
	if "gear" in params:
		current_gear = params["gear"]

# ============================================================================
# SINGLETON CONNECTIONS
# ============================================================================
func _on_game_state_changed(new_state: GameState) -> void:
	match new_state:
		GameManager.GameState.RACE_ACTIVE:
			reset_vehicle()
		GameManager.GameState.MAIN_MENU:
			reset_vehicle()
		GameManager.GameState.RACE_PAUSED:
			pass  # Pause physics if needed
		_:
			pass

</FILE "scripts/controllers/VehicleController.gd">