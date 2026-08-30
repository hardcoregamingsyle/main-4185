extends Node2D
class_name VehicleController

## VehicleController - Core vehicle physics controller using PhysicsSettings constants
## Handles throttle, brake, steering inputs, wheel forces, and gear shifting logic
## Copyright 2026 Thalamus Racing Simulator Project

signal speed_changed(new_speed: float)
signal gear_changed(old_gear: int, new_gear: int)
signal vehicle_moved(displacement: Vector2)
signal drift_angle_changed(angle: float)
signal traction_control_active(active: bool)

# ============================================================================
# PHYSICS CONSTANTS - Derived from PhysicsSettings resource
# ============================================================================

const MAX_THROTTLE_FORCE: float = 15000.0      # Newtons - maximum acceleration force
const MAX_BRAKE_FORCE: float = 20000.0         # Newtons - maximum braking force
const MAX_STEERING_ANGLE: float = PI / 3       # 60 degrees max steering
const STEERING_SPEED: float = 4.0              # Radians per second steering rate
const DRIFT_THRESHOLD: float = 0.7             # Sideslip threshold for drift mode
const TRACTION_CONTROL_SENSITIVITY: float = 0.85 # TCS activation threshold

# ============================================================================
# GEAR RATIOS AND TRANSMISSION CONFIGURATION
# ============================================================================

enum Gear {
    NEUTRAL = 0,
    FIRST = 1,
    SECOND = 2,
    THIRD = 3,
    FOURTH = 4,
    FIFTH = 5,
    SIXTH = 6,
    REVERSE = -1
}

const GEAR_RATIOS: Dictionary = {
    Gear.FIRST: 3.5,
    Gear.SECOND: 2.2,
    Gear.THIRD: 1.6,
    Gear.FOURTH: 1.2,
    Gear.FIFTH: 0.9,
    Gear.SIXTH: 0.75,
    Gear.REVERSE: 3.0
}

const FINAL_DRIVE_RATIO: float = 3.73          # Final drive differential ratio
const REVERSE_GEAR_RATIO: float = 3.5          # Reverse gear ratio

# ============================================================================
# VEHICLE STATE VARIABLES
# ============================================================================

@export var vehicle_mass: float = 1500.0      # kg - overridden by PhysicsSettings default
@export var center_of_gravity: Vector2 = Vector2(0.0, 0.5)  # CG position relative to chassis
@export var wheelbase: float = 2.6           # Distance between front and rear axles (meters)
@export var track_width: float = 1.6         # Distance between left and right wheels (meters)
@export var wheel_radius: float = 0.3        # Wheel radius in meters
@export var tire_friction_coefficient: float = 1.15  # Dry asphalt coefficient
@export var aerodynamic_drag_coefficient: float = 0.32  # Cd value
@export var frontal_area: float = 2.2        # m² - vehicle frontal area

var current_gear: int = Gear.NEUTRAL
var engine_rpm: float = 0.0                  # Engine revolutions per minute
var target_rpm: float = 0.0
var clutch_engaged: bool = true
var handbrake_active: bool = false
var traction_control_enabled: bool = true
var anti_lock_braking_enabled: bool = true

# Velocity and motion tracking
var velocity: Vector2 = Vector2.ZERO
var angular_velocity: float = 0.0            # Yaw rotation rate
var acceleration: Vector2 = Vector2.ZERO
var friction_force: Vector2 = Vector2.ZERO
var drag_force: Vector2 = Vector2.ZERO
var lift_force: float = 0.0

# Drift and slip tracking
var sideslip_angle: float = 0.0
var drift_intensity: float = 0.0
var drift_timer: float = 0.0
var max_drift_time: float = 5.0              # Seconds before overheat penalty

# Wheel-specific data
var front_left_wheel: Dictionary = {}
var front_right_wheel: Dictionary = {}
var rear_left_wheel: Dictionary = {}
var rear_right_wheel: Dictionary = {}

# Powertrain integration
var _powertrain_node: Node = null

# ============================================================================
# ENGINE MANAGEMENT
# ============================================================================

func _init() -> void:
	_reset_vehicle_state()
	
# ============================================================================
# INITIALIZATION
# ============================================================================

func _ready() -> void:
	_connect_to_powertrain()
	_initialize_wheels()
	_set_default_physics_parameters()
	_connect_signals()
	
# ============================================================================
# POWERTRAIN INTEGRATION
# ============================================================================

func _connect_to_powertrain() -> void:
	var powertrain_path: String = "res://scripts/vehicles/Powertrain.gd"
	if get_tree().root.get_node_or_null("Powertrain"):
		_powertrain_node = get_tree().root.get_node("Powertrain")
	elif has_node("Powertrain"):
		_powertrain_node = get_node("Powertrain")
		
# ============================================================================
# WHEEL SYSTEM INITIALIZATION
# ============================================================================

func _initialize_wheels() -> void:
	# Initialize front wheels (steerable)
	front_left_wheel = {
		"position": Vector2(-track_width / 2.0, wheelbase / 2.0),
		"radius": wheel_radius,
		"friction": tire_friction_coefficient,
		"angle": 0.0,
		"locked": false,
		"slip_ratio": 0.0,
		"vertical_load": 0.0,
		"force_x": 0.0,
		"force_y": 0.0
	}
	
	front_right_wheel = {
		"position": Vector2(track_width / 2.0, wheelbase / 2.0),
		"radius": wheel_radius,
		"friction": tire_friction_coefficient,
		"angle": 0.0,
		"locked": false,
		"slip_ratio": 0.0,
		"vertical_load": 0.0,
		"force_x": 0.0,
		"force_y": 0.0
	}
	
	# Initialize rear wheels (non-steerable, driven)
	rear_left_wheel = {
		"position": Vector2(-track_width / 2.0, -wheelbase / 2.0),
		"radius": wheel_radius,
		"friction": tire_friction_coefficient,
		"angle": 0.0,
		"locked": false,
		"slip_ratio": 0.0,
		"vertical_load": 0.0,
		"force_x": 0.0,
		"force_y": 0.0
	}
	
	rear_right_wheel = {
		"position": Vector2(track_width / 2.0, -wheelbase / 2.0),
		"radius": wheel_radius,
		"friction": tire_friction_coefficient,
		"angle": 0.0,
		"locked": false,
		"slip_ratio": 0.0,
		"vertical_load": 0.0,
		"force_x": 0.0,
		"force_y": 0.0
	}
	
# ============================================================================
# PHYSICS PARAMETERS CONFIGURATION
# ============================================================================

func _set_default_physics_parameters() -> void:
	# Apply physics settings from global resource if available
	if PhysicsSettings:
		vehicle_mass = PhysicsSettings.default_vehicle_mass
		tire_friction_coefficient = PhysicsSettings.tire_friction_coefficient
		aerodynamic_drag_coefficient = PhysicsSettings.aerodynamic_drag_coefficient
		frontal_area = PhysicsSettings.frontal_area
		
# ============================================================================
# INPUT HANDLING
# ============================================================================

func _input(event: InputEvent) -> void:
	if event.is_action_pressed("game_throttle"):
		_throttle_input(event.pressed)
	elif event.is_action_pressed("game_brake"):
		_brake_input(event.pressed)
	elif event.is_action_pressed("game_steering_left"):
		_steering_input(-1.0)
	elif event.is_action_pressed("game_steering_right"):
		_steering_input(1.0)
	elif event.is_action_pressed("game_shift_up"):
		_shift_gear(Gear.FIRST + (current_gear - Gear.FIRST) % 6)
	elif event.is_action_pressed("game_shift_down"):
		_shift_gear((current_gear - Gear.FIRST - 1) % 6 + Gear.FIRST)
	elif event.is_action_pressed("game_handbrake"):
		handbrake_active = !handbrake_active
	
# ============================================================================
# THROTTLE CONTROL
# ============================================================================

func _throttle_input(pressed: bool) -> void:
	pass  # Handled in process_delta
	
# ============================================================================
# BRAKE CONTROL
# ============================================================================

func _brake_input(pressed: bool) -> void:
	pass  # Handled in process_delta
	
# ============================================================================
# STEERING CONTROL
# ============================================================================

func _steering_input(direction: float) -> void:
	pass  # Handled in process_delta
	
# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================

func shift_gear(target_gear: int) -> bool:
	# Validate gear change
	if not _is_valid_gear_change(current_gear, target_gear):
		return false
		
	# Check RPM limits for upshift/downshift
	var rpm_limit: float = _get_rpm_limit_for_gear(target_gear)
	
	if target_gear > current_gear and engine_rpm < 2000.0:
		# Too low RPM for downshift
		return false
		
	if target_gear < current_gear and engine_rpm > rpm_limit:
		# Too high RPM for upshift
		return false
		
	# Execute gear change
	var old_gear: int = current_gear
	current_gear = target_gear
	
	# Update transmission state
	_update_transmission_state()
	
	# Emit signal
	gear_changed.emit(old_gear, current_gear)
	
	return true
	
func _shift_gear(target_gear: int) -> void:
	if current_gear == Gear.NEUTRAL:
		# Neutral to first gear or reverse
		if target_gear == Gear.FIRST or target_gear == Gear.REVERSE:
			shift_gear(target_gear)
	elif current_gear >= Gear.FIRST:
		# Forward gears
		if target_gear == Gear.NEUTRAL:
			shift_gear(Gear.NEUTRAL)
		else:
			shift_gear(target_gear)
	elif current_gear == Gear.REVERSE:
		# From reverse
		if target_gear == Gear.NEUTRAL:
			shift_gear(Gear.NEUTRAL)
		else:
			shift_gear(target_gear)
			
# ============================================================================
# GEAR VALIDATION
# ============================================================================

func _is_valid_gear_change(from_gear: int, to_gear: int) -> bool:
	# Prevent invalid transitions
	if from_gear == to_gear:
		return false
		
	# Cannot shift directly from neutral to any gear except first/reverse
	if from_gear == Gear.NEUTRAL and to_gear not in [Gear.FIRST, Gear.REVERSE]:
		return false
		
	# Cannot shift directly to neutral unless from a driving gear
	if to_gear == Gear.NEUTRAL and from_gear not in [Gear.FIRST, Gear.SECOND, 
	                                                   Gear.THIRD, Gear.FOURTH, 
	                                                   Gear.FIFTH, Gear.SIXTH, 
	                                                   Gear.REVERSE]:
		return false
		
	return true
	
# ============================================================================
# RPM LIMIT CALCULATION
# ============================================================================

func _get_rpm_limit_for_gear(gear: int) -> float:
	# Maximum safe RPM varies by gear
	match gear:
		Gear.FIRST: return 6500.0
		Gear.SECOND: return 7000.0
		Gear.THIRD: return 7200.0
		Gear.FOURTH: return 7200.0
		Gear.FIFTH: return 7200.0
		Gear.SIXTH: return 7200.0
		Gear.REVERSE: return 6500.0
		_: return 7200.0
		
# ============================================================================
# TRANSMISSION STATE UPDATE
# ============================================================================

func _update_transmission_state() -> void:
	# Calculate transmission ratio
	var total_ratio: float = 0.0
	
	if current_gear != Gear.NEUTRAL:
		var gear_ratio: float = GEAR_RATIOS.get(current_gear, 1.0)
		total_ratio = gear_ratio * FINAL_DRIVE_RATIO
		if current_gear == Gear.REVERSE:
			total_ratio *= -1.0
			
	# Update clutch state
	if current_gear == Gear.NEUTRAL:
		clutch_engaged = false
	else:
		clutch_engaged = true
		
# ============================================================================
# MAIN PROCESS LOOP
# ============================================================================

func _process(delta: float) -> void:
	if delta <= 0.0:
		return
		
	_process_inputs(delta)
	_update_engine_rpm(delta)
	_calculate_forces(delta)
	_apply_physics(delta)
	_update_drift_state(delta)
	_update_wheel_data(delta)
	
# ============================================================================
# INPUT PROCESSING
# ============================================================================

func _process_inputs(delta: float) -> void:
	# Read throttle input
	var throttle_input: float = Input.get_axis("game_throttle_left", "game_throttle_right")
	if throttle_input == null:
		throttle_input = Input.get_axis("ui_up", "ui_down")
	
	# Normalize throttle (0.0 to 1.0)
	var normalized_throttle: float = clampf(throttle_input, -1.0, 1.0)
	if normalized_throttle > 0.0:
		normalized_throttle = clampf(normalized_throttle, 0.0, 1.0)
	else:
		normalized_throttle = 0.0
		
	# Read brake input
	var brake_input: float = Input.get_axis("game_brake_left", "game_brake_right")
	if brake_input == null:
		brake_input = Input.get_axis("ui_left", "ui_right")
		
	var normalized_brake: float = clampf(abs(brake_input), 0.0, 1.0)
	
	# Read steering input
	var steering_input: float = Input.get_axis("game_steering_left", "game_steering_right")
	if steering_input == null:
		steering_input = Input.get_axis("ui_left", "ui_right")
		
	var normalized_steering: float = clampf(steering_input, -1.0, 1.0)
	
	# Apply inputs to vehicle dynamics
	_apply_throttle_force(normalized_throttle, delta)
	_apply_brake_force(normalized_brake, delta)
	_apply_steering(normalized_steering, delta)
	
# ============================================================================
# THROTTLE FORCE APPLICATION
# ============================================================================

func _apply_throttle_force(amount: float, delta: float) -> void:
	if amount <= 0.0 or current_gear == Gear.NEUTRAL:
		return
		
	# Calculate drive force based on gear ratio
	var gear_ratio: float = GEAR_RATIOS.get(current_gear, 1.0)
	var drive_force: float = MAX_THROTTLE_FORCE * gear_ratio * amount
	
	# Apply to rear wheels (RWD configuration)
	if rear_left_wheel.size() > 0:
		rear_left_wheel["force_x"] += drive_force / 2.0
	if rear_right_wheel.size() > 0:
		rear_right_wheel["force_x"] += drive_force / 2.0
	
# ============================================================================
# BRAKE FORCE APPLICATION
# ============================================================================

func _apply_brake_force(amount: float, delta: float) -> void:
	if amount <= 0.0:
		return
		
	# Calculate brake force
	var brake_force: float = MAX_BRAKE_FORCE * amount
	
	# Distribute brake force across all wheels
	var wheel_count: int = 4
	var force_per_wheel: float = brake_force / wheel_count
	
	# Apply to all wheels
	for wheel_dict in [front_left_wheel, front_right_wheel, rear_left_wheel, rear_right_wheel]:
		wheel_dict["force_x"] -= force_per_wheel
		
	# Handbrake affects only rear wheels
	if handbrake_active:
		for rear_wheel in [rear_left_wheel, rear_right_wheel]:
			rear_wheel["force_x"] -= force_per_wheel * 0.5
			
# ============================================================================
# STEERING APPLICATION
# ============================================================================

func _apply_steering(amount: float, delta: float) -> void:
	if current_gear == Gear.NEUTRAL and abs(velocity.length()) < 1.0:
		return
		
	# Calculate target steering angle
	var target_angle: float = amount * MAX_STEERING_ANGLE
	
	# Smooth steering transition
	var front_wheel: Dictionary = front_left_wheel if amount < 0 else front_right_wheel
	var current_angle: float = front_wheel.get("angle", 0.0)
	
	# Steer towards target
	if abs(target_angle - current_angle) > 0.01:
		var steer_direction: float = sign(target_angle - current_angle)
		var steer_amount: float = min(STEERING_SPEED * delta, abs(target_angle - current_angle))
		
		current_angle += steer_direction * steer_amount
		front_wheel["angle"] = current_angle
		
# ============================================================================
# ENGINE RPM UPDATE
# ============================================================================

func _update_engine_rpm(delta: float) -> void:
	if current_gear == Gear.NEUTRAL:
		target_rpm = 800.0  # Idle RPM
		engine_rpm = lerp(engine_rpm, target_rpm, 5.0 * delta)
		return
		
	# Calculate theoretical RPM based on vehicle speed
	var wheel_angular_velocity: float = velocity.length() / wheel_radius
	var final_ratio: float = GEAR_RATIOS.get(current_gear, 1.0) * FINAL_DRIVE_RATIO
	
	if current_gear == Gear.REVERSE:
		final_ratio *= -1.0
		
	target_rpm = wheel_angular_velocity * final_ratio * 60.0 / (2.0 * PI)
	
	# Clamp RPM to realistic range
	target_rpm = clampf(target_rpm, 800.0, 8000.0)
	
	# Smooth RPM transition
	var rpm_change_rate: float = 2000.0 * delta
	engine_rpm = lerp(engine_rpm, target_rpm, min(1.0, rpm_change_rate / abs(target_rpm - engine_rpm + 0.01)))
	
# ============================================================================
# FORCE CALCULATION
# ============================================================================

func _calculate_forces(delta: float) -> void:
	# Calculate aerodynamic drag
	drag_force = _calculate_aerodynamic_drag()
	
	# Calculate friction forces
	friction_force = _calculate_friction_forces()
	
	# Calculate lift force (downforce at speed)
	lift_force = _calculate_aerodynamic_lift()
	
	# Accumulate wheel forces
	_accumulate_wheel_forces()
	
# ============================================================================
# AERODYNAMIC DRAG CALCULATION
# ============================================================================

func _calculate_aerodynamic_drag() -> Vector2:
	var air_density: float = 1.225  # kg/m³ at sea level
	var speed_squared: float = pow(velocity.length(), 2)
	
	var drag_magnitude: float = 0.5 * air_density * aerodynamic_drag_coefficient * frontal_area * speed_squared
	
	# Oppose velocity direction
	if velocity.length() > 0.0:
		var drag_direction: Vector2 = -velocity.normalized()
		return drag_direction * drag_magnitude
	
	return Vector2.ZERO
	
# ============================================================================
# FRICTION FORCE CALCULATION
# ============================================================================

func _calculate_friction_forces() -> Vector2:
	var normal_force: float = vehicle_mass * PhysicsSettings.gravity
	
	# Simple Coulomb friction model
	var friction_magnitude: float = normal_force * tire_friction_coefficient
	
	# Oppose velocity direction
	if velocity.length() > 0.0:
		var friction_direction: Vector2 = -velocity.normalized()
		return friction_direction * friction_magnitude
	
	return Vector2.ZERO
	
# ============================================================================
# AERODYNAMIC LIFT/DOWNFORCE CALCULATION
# ============================================================================

func _calculate_aerodynamic_lift() -> float:
	var air_density: float = 1.225
	var speed_squared: float = pow(velocity.length(), 2)
	
	# Negative coefficient for downforce
	var lift_coefficient: float = -0.15
	
	var lift_magnitude: float = 0.5 * air_density * lift_coefficient * frontal_area * speed_squared
	
	return lift_magnitude
	
# ============================================================================
# WHEEL FORCE ACCUMULATION
# ============================================================================

func _accumulate_wheel_forces() -> void:
	var total_force_x: float = 0.0
	var total_force_y: float = 0.0
	
	for wheel_dict in [front_left_wheel, front_right_wheel, rear_left_wheel, rear_right_wheel]:
		total_force_x += wheel_dict.get("force_x", 0.0)
		total_force_y += wheel_dict.get("force_y", 0.0)
		
	# Clear individual wheel forces after accumulation
	for wheel_dict in [front_left_wheel, front_right_wheel, rear_left_wheel, rear_right_wheel]:
		wheel_dict["force_x"] = 0.0
		wheel_dict["force_y"] = 0.0
		
# ============================================================================
# PHYSICS APPLICATION
# ============================================================================

func _apply_physics(delta: float) -> void:
	# Calculate net force
	var net_force_x: float = drag_force.x + friction_force.x
	var net_force_y: float = drag_force.y + friction_force.y
	
	# Apply Newton's second law: F = ma
	acceleration.x = net_force_x / vehicle_mass
	acceleration.y = net_force_y / vehicle_mass
	
	# Update velocity
	velocity.x += acceleration.x * delta
	velocity.y += acceleration.y * delta
	
	# Apply damping to prevent infinite sliding
	var damping: float = 0.99
	velocity.x *= damping
	.velocity.y *= damping
	
	# Cap maximum speed
	var max_speed: float = 120.0  # m/s ≈ 432 km/h
	if velocity.length() > max_speed:
		velocity = velocity.normalized() * max_speed
	
	# Update position
	position.x += velocity.x * delta
	position.y += velocity.y * delta
	
	# Emit signals
	speed_changed.emit(velocity.length())
	vehicle_moved.emit(velocity * delta)
	
# ============================================================================
# DRIFT STATE MANAGEMENT
# ============================================================================

func _update_drift_state(delta: float) -> void:
	# Calculate sideslip angle
	sideslip_angle = _calculate_sideslip_angle()
	
	# Detect drift condition
	var is_drifting: bool = abs(sideslip_angle) > DRIFT_THRESHOLD
	
	if is_drifting:
		_drift_timer += delta
		drift_intensity = min(drift_timer / max_drift_time, 1.0)
		
		# Trigger drift effects
		if drift_timer > 1.0 and int(drift_timer) % 5 == 0:
			_trigger_drift_particles()
	else:
		_drift_timer = max(0.0, _drift_timer - delta * 2.0)
		drift_intensity = 0.0
		
	# Track drift statistics
	if drift_intensity > 0.5:
		traction_control_active.emit(false)
		
# ============================================================================
# SIDESLIP ANGLE CALCULATION
# ============================================================================

func _calculate_sideslip_angle() -> float:
	if velocity.length() < 0.1:
		return 0.0
		
	# Calculate velocity direction vs heading direction
	var velocity_angle: float = velocity.angle()
	var heading_angle: float = rotation
	
	return fmod(heading_angle - velocity_angle + PI, 2.0 * PI) - PI
	
# ============================================================================
# DRIFT PARTICLES EFFECT
# ============================================================================

func _trigger_drift_particles() -> void:
	# Spawn particle effect at rear wheels
	# This would integrate with particle system
	pass
	
# ============================================================================
# WHEEL DATA UPDATE
# ============================================================================

func _update_wheel_data(delta: float) -> void:
	# Update vertical load distribution based on acceleration
	_update_weight_distribution()
	
	# Update wheel slip ratios
	_update_wheel_slip_ratios()
	
# ============================================================================
# WEIGHT DISTRIBUTION UPDATE
# ============================================================================

func _update_weight_distribution() -> void:
	var weight_transfer: float = (acceleration.x * center_of_gravity.y * vehicle_mass) / wheelbase
	
	# Front wheel load decreases during acceleration
	var front_load_change: float = -weight_transfer / 2.0
	var rear_load_change: float = weight_transfer / 2.0
	
	# Distribute to individual wheels
	front_left_wheel["vertical_load"] = (vehicle_mass * PhysicsSettings.gravity / 4.0) + front_load_change
	front_right_wheel["vertical_load"] = (vehicle_mass * PhysicsSettings.gravity / 4.0) + front_load_change
	rear_left_wheel["vertical_load"] = (vehicle_mass * PhysicsSettings.gravity / 4.0) + rear_load_change
	rear_right_wheel["vertical_load"] = (vehicle_mass * PhysicsSettings.gravity / 4.0) + rear_load_change
	
# ============================================================================
# WHEEL SLIP RATIO UPDATE
# ============================================================================

func _update_wheel_slip_ratios() -> void:
	var wheel_circumference: float = 2.0 * PI * wheel_radius
	
	for wheel_dict in [front_left_wheel, front_right_wheel, rear_left_wheel, rear_right_wheel]:
		var wheel_linear_velocity: float = velocity.length()
		var wheel_rotational_velocity: float = engine_rpm * 2.0 * PI / 60.0
		
		# Slip ratio = (wheel speed - vehicle speed) / vehicle speed
		if wheel_linear_velocity > 0.1:
			var slip_ratio: float = (wheel_rotational_velocity * wheel_radius - wheel_linear_velocity) / wheel_linear_velocity
			wheel_dict["slip_ratio"] = slip_ratio
		else:
			wheel_dict["slip_ratio"] = 0.0
			
# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================

func get_current_speed_kmh() -> float:
	return velocity.length() * 3.6  # Convert m/s to km/h
	
func get_current_speed_mph() -> float:
	return velocity.length() * 2.23694  # Convert m/s to mph
	
func get_engine_rpm_percentage() -> float:
	var max_rpm: float = 8000.0
	return clampf(engine_rpm / max_rpm, 0.0, 1.0)
	
func reset_vehicle_state() -> void:
	_reset_vehicle_state()
	
func _reset_vehicle_state() -> void:
	velocity = Vector2.ZERO
	angular_velocity = 0.0
	acceleration = Vector2.ZERO
	friction_force = Vector2.ZERO
	drag_force = Vector2.ZERO
	lift_force = 0.0
	sideslip_angle = 0.0
	_drift_timer = 0.0
	drift_intensity = 0.0
	engine_rpm = 800.0  # Idle
	current_gear = Gear.NEUTRAL
	clutch_engaged = false
	handbrake_active = false
	
	# Reset wheel states
	for wheel_dict in [front_left_wheel, front_right_wheel, rear_left_wheel, rear_right_wheel]:
		wheel_dict["force_x"] = 0.0
		wheel_dict["force_y"] = 0.0
		wheel_dict["slip_ratio"] = 0.0
		wheel_dict["vertical_load"] = 0.0
		
# ============================================================================
# SIGNAL CONNECTIONS
# ============================================================================

func _connect_signals() -> void:
	# Connect to GameManager if available
	if GameManager:
		GameManager.game_state_changed.connect(_on_game_state_changed)
		
# ============================================================================
# GAME STATE HANDLERS
# ============================================================================

func _on_game_state_changed(new_state: GameState) -> void:
	match new_state:
		GameManager.GameState.RACE_ACTIVE:
			# Enable full physics simulation
			traction_control_enabled = true
			anti_lock_braking_enabled = true
		GameManager.GameState.RACE_PAUSED:
			# Pause physics updates
			_passive_mode = true
		GameManager.GameState.RACE_FINISHED:
			# Stop vehicle
			velocity = Vector2.ZERO
			_passive_mode = false
			
# ============================================================================
# DEBUG VISUALIZATION
# ============================================================================

func _draw_debug_info() -> void:
	if not GameManager.debug_mode:
		return
		
	# Draw velocity vector
	var debug_color: Color = Color.GREEN
	draw_line(position, position + velocity * 0.5, debug_color, 2.0)
	
	# Draw wheel positions
	for wheel_name in ["front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel"]:
		var wheel_data: Dictionary = self[wheel_name]
		if wheel_data.size() > 0:
			var screen_pos: Vector2 = local_to_screen(wheel_data["position"])
			draw_circle(screen_pos, wheel_data["radius"] * 50, Color.RED)
			
# ============================================================================
# EXPORT FUNCTIONS FOR EXTERNAL ACCESS
# ============================================================================

func apply_manual_throttle(force_factor: float) -> void:
	_apply_throttle_force(force_factor, 0.016)
	
func apply_manual_brake(force_factor: float) -> void:
	_apply_brake_force(force_factor, 0.016)
	
func set_manual_steering(angle: float) -> void:
	var normalized_angle: float = clampf(angle, -1.0, 1.0)
	_apply_steering(normalized_angle, 0.016)
	
func emergency_stop() -> void:
	# Full brake application
	_apply_brake_force(1.0, 0.016)
	_apply_throttle_force(0.0, 0.016)
	
func recover_from_drift() -> void:
	# Counter-steer to regain traction
	if sideslip_angle > 0:
		_apply_steering(-0.5, 0.016)
	else:
		_apply_steering(0.5, 0.016)
		
</script>