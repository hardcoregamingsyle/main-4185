extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulation
## Handles throttle, brake, steering, wheel forces, gear shifting, and vehicle dynamics
## Implements realistic suspension, tire grip models, aerodynamic drag, and power delivery
## Copyright 2026 Thalamus Racing Simulator Project

# Signals for external systems to react to vehicle state changes
signal speed_changed(new_speed: float)
signal rpm_changed(new_rpm: float)
signal gear_changed(old_gear: int, new_gear: int)
signal drift_started(drift_angle: float)
signal drift_ended()
signal collision_impact(impact_force: Vector3, impact_point: Vector3)
signal wheel_slip(wheel_index: int, slip_ratio: float)

# Physics Settings singleton reference
var _physics: PhysicsSettings = null

# ============================================================================
# VEHICLE CONFIGURATION
# ============================================================================

@export_group("Vehicle Configuration")
@export var vehicle_mass: float = 1500.0: set = _set_vehicle_mass
@export var center_of_mass_offset: Vector3 = Vector3(0.0, 0.5, 0.0)
@export var track_width_front: float = 1.5
@export var track_width_rear: float = 1.5
@export var wheelbase: float = 2.6
@export var ground_clearance: float = 0.15
@export var suspension_travel: float = 0.12
@export var suspension_stiffness: float = 50000.0
@export var damping_rate: float = 4000.0

@export_group("Aerodynamics")
@export var drag_coefficient: float = 0.30
@export var lift_coefficient: float = 0.10
@export var frontal_area: float = 2.2
@export var downforce_at_100kph: float = 200.0

@export_group("Suspension Geometry")
@export var camber_angle_static: float = -1.5 * TAU / 360.0
@export var caster_angle: float = 8.0 * TAU / 360.0
@export var toe_angle_front: float = 0.1 * TAU / 360.0
@export var toe_angle_rear: float = 0.05 * TAU / 360.0

# ============================================================================
# POWERTRAIN PARAMETERS
# ============================================================================

@export_group("Engine Specifications")
@export var engine_max_rpm: float = 7500.0
@export var engine_idle_rpm: float = 800.0
@export var engine_peak_torque_rpm: float = 4500.0
@export var engine_peak_torque: float = 400.0  # Nm
@export var engine_max_power: float = 250.0     # kW
@export var rev_match_on_downshift: bool = true
@export var rev_limit_shifter: bool = true

@export_group("Transmission")
@export var transmission_type: String = "manual"  # "manual", "automatic", "sequential"
@export var gear_count: int = 6
@export var final_drive_ratio: float = 3.73
@export var gear_ratios: Array[float] = [3.8, 2.2, 1.5, 1.1, 0.9, 0.7]
@export var reverse_ratio: float = 3.5
@export var clutch_friction: float = 0.95
@export var clutch_engagement_speed: float = 1500.0

@export_group("Clutch & Throttle Response")
@export var clutch_spring_pressure: float = 5000.0
@export var throttle_response_curve: float = 0.85
@export var idle_air_control: float = 0.05

# ============================================================================
# BRAKING SYSTEM
# ============================================================================

@export_group("Braking System")
@export var brake_disc_diameter: float = 0.32
@export var brake_pad_material: String = "ceramic"
@export var max_brake_pressure: float = 100.0  # bar
@export var brake_bias_front: float = 0.60  # percentage of force on front wheels
@export var brake_force_per_bar: float = 3500.0
@export var abs_enabled: bool = true
@export var electronic_brake_distribution: bool = true
@export var brake_anti_squat: float = 0.15

@export_group("Tire & Grip")
@export var tire_width_front: float = 0.25
@export var tire_width_rear: float = 0.30
@export var tire_pressure_front: float = 2.2  # bar
@export var tire_pressure_rear: float = 2.3  # bar
@export var tire_grip_coefficient: float = 1.15
@export var tire_temperature_optimal: float = 80.0  # Celsius
@export var tire_degradation_rate: float = 0.001

# ============================================================================
# VEHICLE STATE VARIABLES
# ============================================================================

# Current dynamic state
var current_speed: float = 0.0  # m/s
var current_rpm: float = 0.0
var current_gear: int = 0  # 0 = neutral, -1 = reverse, 1-6 = forward gears
var target_gear: int = 0
var clutch_position: float = 1.0  # 1.0 = engaged, 0.0 = fully disengaged
var throttle_input: float = 0.0   # 0.0 - 1.0
var brake_input: float = 0.0      # 0.0 - 1.0
var steering_input: float = 0.0   # -1.0 - 1.0 (negative = left, positive = right)

# Wheel-specific states
var wheel_states: Array[Dictionary] = []
const WHEEL_FRL = 0
const WHEEL_FRR = 1
const WHEEL_FLR = 2
const WHEEL_FLR = 3

# Force application targets (calculated each frame)
var drive_force: float = 0.0
var brake_force: float = 0.0
var lateral_force: float = 0.0

# Drift and slip tracking
var drift_angle: float = 0.0
var is_drifting: bool = false
var wheel_slip_ratios: Array[float] = [0.0, 0.0, 0.0, 0.0]

# Aerodynamic forces
var aero_drag_force: float = 0.0
var aero_lift_force: float = 0.0
var aero_downforce: float = 0.0

# Suspension compression
var suspension_compression: Array[float] = [0.0, 0.0, 0.0, 0.0]

# ============================================================================
# INITIALIZATION
# ============================================================================

func _ready() -> void:
	# Initialize physics settings reference
	if has_node("/root/PhysicsSettings"):
		_physics = get_node("/root/PhysicsSettings")
	elif Engine.has_singleton("PhysicsSettings"):
		_physics = Engine.get_singleton("PhysicsSettings")
	else:
		_physics = preload("res://scripts/core/PhysicsSettings.gd").new()
	
	# Apply gravity scale
	gravity_scale = 1.0
	
	# Initialize wheel states
	_initialize_wheel_states()
	
	# Set initial position and rotation
	position.y = ground_clearance + (_physics.default_vehicle_mass / vehicle_mass) * 0.5
	
	# Connect to GameManager if available
	if GameManager:
		GameManager.game_state_changed.connect(_on_game_state_changed)

func _initialize_wheel_states() -> void:
	"""Initialize wheel state dictionaries for all four wheels"""
	wheel_states.clear()
	wheel_states.resize(4)
	
	for i in range(4):
		var wheel: Dictionary = {
			"id": i,
			"position_local": _get_wheel_position(i),
			"radius": 0.32,
			"width": tire_width_front if i < 2 else tire_width_rear,
			"compression": 0.0,
			"velocity": Vector3.ZERO,
			"slip_ratio": 0.0,
			"slip_angle": 0.0,
			"camber": camber_angle_static,
			"angle": 0.0,
			"force": Vector3.ZERO,
			"temperature": 20.0,
			"wear": 0.0
		}
		wheel_states[i] = wheel

func _get_wheel_position(index: int) -> Vector3:
	"""Get local position of wheel based on index"""
	match index:
		WHEEL_FRL: return Vector3(-wheelbase/2, -suspension_travel/2, -track_width_front/2)
		WHEEL_FRR: return Vector3(-wheelbase/2, -suspension_travel/2, track_width_front/2)
		WHEEL_FLR: return Vector3(wheelbase/2, -suspension_travel/2, -track_width_rear/2)
		WHEEL_FLL: return Vector3(wheelbase/2, -suspension_travel/2, track_width_rear/2)
	return Vector3.ZERO

# ============================================================================
# PHYSICS PROCESS
# ============================================================================

func _physics_process(delta: float) -> void:
	# Validate physics settings are loaded
	if not _physics:
		return
	
	# Step 1: Read input from InputManager
	_read_inputs()
	
	# Step 2: Update RPM based on gear ratio and speed
	_update_engine_rpm(delta)
	
	# Step 3: Calculate engine torque output
	var engine_torque: float = _calculate_engine_torque()
	
	# Step 4: Apply clutch and transmission effects
	engine_torque *= _apply_transmission_loss(engine_torque)
	
	# Step 5: Calculate drive force to wheels
	_calculate_drive_forces(engine_torque, delta)
	
	# Step 6: Calculate braking forces
	_calculate_brake_forces()
	
	# Step 7: Calculate aerodynamic forces
	_calculate_aerodynamics()
	
	# Step 8: Apply suspension and tire forces
	_apply_suspension_and_tires(delta)
	
	# Step 9: Apply all calculated forces to rigid body
	_apply_vehicle_forces(delta)
	
	# Step 10: Update velocity and detect drift
	_update_vehicle_velocity(delta)
	
	# Step 11: Emit signals for external systems
	_emit_signals()

func _read_inputs() -> void:
	"""Read input values from InputManager singleton"""
	if InputManager:
		throttle_input = clamp(InputManager.get_axis("throttle"), 0.0, 1.0)
		brake_input = clamp(InputManager.get_axis("brake"), 0.0, 1.0)
		steering_input = clamp(InputManager.get_axis("steering"), -1.0, 1.0)
	else:
		# Fallback to direct input if InputManager unavailable
		throttle_input = Input.get_axis("ui_left", "ui_right")
		brake_input = Input.get_axis("ui_up", "ui_down")
		steering_input = Input.get_axis("ui_left", "ui_right")
		
		# Correct axis mapping for steering
		if Input.is_action_pressed("ui_left"):
			steering_input = -1.0
		elif Input.is_action_pressed("ui_right"):
			steering_input = 1.0

func _update_engine_rpm(delta: float) -> void:
	"""Update engine RPM based on gear ratio and vehicle speed"""
	if current_gear == 0:  # Neutral
		current_rpm = lerp(current_rpm, engine_idle_rpm, delta * 10.0)
		return
	
	var gear_ratio: float = gear_ratios[current_gear - 1] if current_gear > 0 else reverse_ratio
	var wheel_circumference: float = PI * 0.64  # 2 * PI * radius
	var wheel_rps: float = current_speed / wheel_circumference
	var theoretical_rpm: float = wheel_rps * gear_ratio * final_drive_ratio * 60.0
	
	# Apply clutch engagement
	if clutch_position < 1.0:
		theoretical_rpm = lerp(theoretical_rpm, current_rpm, clutch_position)
	
	# Smooth RPM transitions
	current_rpm = lerp(current_rpm, theoretical_rpm, delta * 5.0)
	
	# Apply rev limiter
	if rev_limit_shifter and current_rpm > engine_max_rpm:
		current_rpm = engine_max_rpm

func _calculate_engine_torque() -> float:
	"""Calculate engine torque based on RPM using a torque curve"""
	# Normalize RPM to 0.0-1.0 range
	var normalized_rpm: float = (current_rpm - engine_idle_rpm) / (engine_max_rpm - engine_idle_rpm)
	normalized_rpm = clamp(normalized_rpm, 0.0, 1.0)
	
	# Create torque curve (peak at peak_torque_rpm)
	var torque_curve_factor: float = 1.0 - abs(normalized_rpm - (engine_peak_torque_rpm - engine_idle_rpm) / (engine_max_rpm - engine_idle_rpm)) * 2.0
	torque_curve_factor = max(0.0, torque_curve_factor)
	
	# Apply throttle input with response curve
	var throttle_effect: float = pow(throttle_input, throttle_response_curve)
	
	var torque: float = engine_peak_torque * torque_curve_factor * (0.5 + 0.5 * throttle_effect)
	
	# Reduce torque near idle
	if current_rpm < engine_idle_rpm * 1.2:
		torque *= 0.3
	
	return torque

func _apply_transmission_loss(torque: float) -> float:
	"""Apply transmission efficiency losses"""
	var loss_factor: float = 0.85  # 15% loss through transmission
	if transmission_type == "manual":
		loss_factor *= clutch_friction
	elif transmission_type == "automatic":
		loss_factor *= 0.92  # Torque converter loss
	return loss_factor

func _calculate_drive_forces(engine_torque: float, delta: float) -> void:
	"""Calculate drive forces applied to driven wheels"""
	if current_gear == 0 or current_gear < -1:
		drive_force = 0.0
		return
	
	var total_ratio: float = gear_ratios[current_gear - 1] * final_drive_ratio if current_gear > 0 else reverse_ratio * final_drive_ratio
	var wheel_radius: float = wheel_states[0].radius
	var differential_efficiency: float = 0.98
	
	var drive_torque_at_wheels: float = engine_torque * total_ratio * differential_efficiency
	
	# Distribute torque between driven wheels
	var torque_per_wheel: float = drive_torque_at_wheels / 2.0
	
	# Apply to rear wheels (RWD setup)
	if transmission_type != "FWD":
		wheel_states[WHEEL_FLR].force.x += torque_per_wheel / wheel_radius
		wheel_states[WHEEL_FLL].force.x += torque_per_wheel / wheel_radius
	
	# FWD override
	elif transmission_type == "FWD":
		wheel_states[WHEEL_FRL].force.x += torque_per_wheel / wheel_radius
		wheel_states[WHEEL_FRR].force.x += torque_per_wheel / wheel_radius

func _calculate_brake_forces() -> void:
	"""Calculate brake forces applied to all wheels"""
	if brake_input <= 0.0:
		brake_force = 0.0
		return
	
	var brake_pressure: float = brake_input * max_brake_pressure
	var total_brake_force: float = brake_pressure * brake_force_per_bar
	
	# Apply brake bias
	var front_brake_force: float = total_brake_force * brake_bias_front
	var rear_brake_force: float = total_brake_force * (1.0 - brake_bias_front)
	
	# Apply to individual wheels
	var force_per_front_wheel: float = front_brake_force / 2.0
	var force_per_rear_wheel: float = rear_brake_force / 2.0
	
	wheel_states[WHEEL_FRL].force.x -= force_per_front_wheel
	wheel_states[WHEEL_FRR].force.x -= force_per_front_wheel
	wheel_states[WHEEL_FLR].force.x -= force_per_rear_wheel
	wheel_states[WHEEL_FLL].force.x -= force_per_rear_wheel
	
	brake_force = total_brake_force

func _calculate_aerodynamics() -> void:
	"""Calculate aerodynamic drag and downforce forces"""
	var air_density: float = 1.225  # kg/m^3 at sea level
	var velocity_squared: float = current_speed * current_speed
	
	# Drag force: Fd = 0.5 * rho * v^2 * Cd * A
	aero_drag_force = 0.5 * air_density * velocity_squared * drag_coefficient * frontal_area
	
	# Lift/downforce: Fl = 0.5 * rho * v^2 * Cl * A
	aero_lift_force = 0.5 * air_density * velocity_squared * lift_coefficient * frontal_area
	aero_downforce = -aero_lift_force  # Negative lift = downforce
	
	# Scale downforce with speed squared (some cars have active aero)
	aero_downforce *= (current_speed / 27.78) ** 2  # Reference speed 100 kph = 27.78 m/s

func _apply_suspension_and_tires(delta: float) -> void:
	"""Apply suspension compression and tire grip forces"""
	for i in range(4):
		var wheel: Dictionary = wheel_states[i]
		
		# Calculate suspension compression based on height
		var target_height: float = ground_clearance - suspension_travel / 2.0
		var actual_height: float = position.y
		
		suspension_compression[i] = clamp(actual_height - target_height, 0.0, suspension_travel)
		
		# Spring force: F = k * x
		var spring_force: float = suspension_stiffness * suspension_compression[i]
		
		# Damper force: F = c * v
		var damper_velocity: float = linear_interpolate(0.0, 1.0, delta * 2.0)
		var damper_force: float = damping_rate * damper_velocity
		
		# Apply spring and damper forces upward
		var suspension_force: Vector3 = Vector3.UP * (spring_force + damper_force)
		
		# Tire grip calculation (simplified Pacejka model)
		var normal_force: float = (vehicle_mass * _physics.gravity / 4.0) + (aero_downforce / 4.0)
		var grip_coefficient: float = tire_grip_coefficient * (1.0 - tire_degradation_rate * wheel["wear"])
		
		# Lateral grip limit
		var max_lateral_force: float = normal_force * grip_coefficient
		var longitudinal_grip: float = normal_force * grip_coefficient * 0.8
		
		# Apply grip limits to wheel forces
		var wheel_total_force: float = wheel.force.length()
		if wheel_total_force > max_lateral_force:
			wheel.force = wheel.force.normalized() * max_lateral_force

func _apply_vehicle_forces(delta: float) -> void:
	"""Apply all calculated forces to the vehicle's rigid body"""
	# Apply gravity
	add_gravity(_physics.gravity)
	
	# Apply suspension forces to maintain height
	for i in range(4):
		var suspension_pos: Vector3 = _get_wheel_position(i).rotated(Vector3.DOWN, 0.0)
		var suspension_force: Vector3 = Vector3.UP * suspension_stiffness * suspension_compression[i]
		apply_central_force(suspension_force)
	
	# Apply aerodynamic forces
	var aero_force: Vector3 = Vector3.FORWARD * (-aero_drag_force) + Vector3.UP * aero_downforce
	apply_central_force(aero_force)
	
	# Apply drive and brake forces through wheels
	for i in range(4):
		var wheel_force: Vector3 = wheel_states[i].force
		if wheel_force.x != 0.0:
			var wheel_pos: Vector3 = global_transform * _get_wheel_position(i)
			apply_central_force(wheel_force)

func _update_vehicle_velocity(delta: float) -> void:
	"""Update vehicle velocity and calculate drift angle"""
	# Get horizontal velocity
	var horizontal_velocity: Vector3 = velocity.project(Vector3(1.0, 0.0, 0.0))
	current_speed = horizontal_velocity.length()
	
	# Calculate drift angle (difference between heading and velocity direction)
	var heading_direction: Vector3 = global_transform.basis.z.normalized()
	var velocity_direction: Vector3 = horizontal_velocity.normalized()
	
	if current_speed > 0.5:
		var angle_diff: float = heading_direction.angle_to(velocity_direction)
		var absolute_angle: float = abs(angle_diff)
		
		# Threshold for detecting drift
		if absolute_angle > 0.3 and current_speed > 5.0:
			is_drifting = true
		漂移_angle = angle_diff
			emit_signal(drift_started, drift_angle)
		else:
			is_drifting = false
			emit_signal(drift_ended())
		else:
			is_drifting = false

func _emit_signals() -> void:
	"""Emit signals for external systems to react to state changes"""
	if current_speed != 0.0:
		emit_signal(speed_changed, current_speed)
	
	if current_rpm != 0.0:
		emit_signal(rpm_changed, current_rpm)

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================

func shift_gear(new_gear: int) -> void:
	"""Initiate gear shift sequence"""
	if new_gear == current_gear:
		return
	
	var old_gear: int = current_gear
	target_gear = new_gear
	
	# Disengage clutch
	clutch_position = 0.0
	
	# Wait for clutch disengagement before changing gear
	await get_tree().create_timer(0.15).timeout
	
	# Perform gear change
	current_gear = new_gear
	target_gear = 0
	
	# Rev match on downshift if enabled
	if rev_match_on_downshift and new_gear < old_gear:
		var target_rpm: float = _calculate_target_rpm_for_gear(new_gear)
		current_rpm = lerp(current_rpm, target_rpm, 0.5)
	
	# Re-engage clutch smoothly
	await get_tree().create_timer(0.2).timeout
	clutch_position = 1.0
	
	# Emit gear change signal
	emit_signal(gear_changed, old_gear, new_gear)

func _calculate_target_rpm_for_gear(gear: int) -> float:
	"""Calculate target RPM for given gear at current speed"""
	if gear == 0:
		return engine_idle_rpm
	
	var gear_ratio: float = gear_ratios[gear - 1] if gear > 0 else reverse_ratio
	var wheel_circumference: float = PI * 0.64
	var wheel_rps: float = current_speed / wheel_circumference
	var target_rpm: float = wheel_rps * gear_ratio * final_drive_ratio * 60.0
	
	return target_rpm

func upshift() -> void:
	"""Shift to next higher gear"""
	if current_gear < gear_count and current_gear >= 0:
		shift_gear(current_gear + 1)

func downshift() -> void:
	"""Shift to next lower gear"""
	if current_gear > 0 and current_gear <= gear_count:
		shift_gear(current_gear - 1)
	elif current_gear == 0:
		shift_gear(-1)  # Reverse

func manual_shift(direction: int) -> void:
	"""Manual gear shift via input (+1 for up, -1 for down)"""
	if direction > 0:
		upshift()
	else:
		downshift()

# ============================================================================
# DRIFT MECHANICS
# ============================================================================

func initiate_drift() -> void:
	"""Force vehicle into drift state"""
	is_drifting = true
	drift_angle = 0.5
	emit_signal(drift_started, drift_angle)

func end_drift() -> void:
	"""End drift state"""
	is_drifting = false
	drift_angle = 0.0
	emit_signal(drift_ended())

func update_drift(delta: float) -> void:
	"""Update drift physics and decay"""
	if not is_drifting:
		return
	
	# Decay drift angle over time
	drift_angle = lerp(drift_angle, 0.0, delta * 2.0)
	
	if drift_angle < 0.1:
		end_drift()

# ============================================================================
# INPUT HANDLING METHODS
# ============================================================================

func handle_throttle_input(value: float) -> void:
	"""Process throttle input with smoothing"""
	throttle_input = lerp(throttle_input, value, 0.1)

func handle_brake_input(value: float) -> void:
	"""Process brake input with smoothing"""
	brake_input = lerp(brake_input, value, 0.1)

func handle_steering_input(value: float) -> void:
	"""Process steering input with smoothing"""
	steering_input = lerp(steering_input, value, 0.15)

func handle_clutch_input(value: float) -> void:
	"""Process clutch input"""
	clutch_position = lerp(clutch_position, value, 0.2)

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================

func get_current_mach_number() -> float:
	"""Get current Mach number (speed of sound approximation)"""
	var speed_of_sound: float = 343.0  # m/s at sea level
	return current_speed / speed_of_sound

func get_dynamic_weight_transfer() -> Dictionary:
	"""Calculate weight transfer during acceleration/braking"""
	var weight_transfer: Dictionary = {}
	
	# Longitudinal weight transfer
	var acceleration: float = (velocity.x - velocity.x_prev) / 0.016 if velocity.x != velocity.x_prev else 0.0
	var weight_transfer_amount: float = vehicle_mass * acceleration * center_of_mass_offset.y / wheelbase
	
	weight_transfer.front_axle = (vehicle_mass * _physics.gravity / 2.0) - weight_transfer_amount
	weight_transfer.rear_axle = (vehicle_mass * _physics.gravity / 2.0) + weight_transfer_amount
	
	return weight_transfer

func reset_vehicle() -> void:
	"""Reset vehicle to initial state"""
	velocity = Vector3.ZERO
	angular_velocity = Vector3.ZERO
	current_speed = 0.0
	current_rpm = engine_idle_rpm
	current_gear = 0
	target_gear = 0
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	clutch_position = 1.0
	is_drifting = false
	drift_angle = 0.0
	
	suspension_compression.fill(0.0)
	wheel_states.fill({})
	_initialize_wheel_states()

# ============================================================================
# EXPORT/IMPORT SETTINGS
# ============================================================================

func _set_vehicle_mass(value: float) -> void:
	vehicle_mass = value
	# Recalculate suspension forces based on new mass
	for i in range(4):
		suspension_compression[i] = 0.0

func _on_game_state_changed(new_state: GameState) -> void:
	"""Handle game state changes"""
	match new_state:
		GameState.RACE_ACTIVE:
			# Resume vehicle physics
			process_mode = ProcessModeEnum.ALWAYS
		GameState.RACE_PAUSED:
			# Pause physics updates
			process_mode = ProcessModeEnum.PAUSED
		GameState.MAIN_MENU:
			reset_vehicle()
			process_mode = ProcessModeEnum.DISABLED

</script>