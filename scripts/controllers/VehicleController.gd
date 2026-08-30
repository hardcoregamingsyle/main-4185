extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulator
## Handles throttle, brake, steering inputs, wheel forces, and gear shifting logic
## Integrates with PhysicsSettings, InputManager, and Powertrain systems
## Copyright 2026 Thalamus Racing Simulator Project

# ============================================================================
# SIGNALS
# ============================================================================

signal speed_changed(new_speed: float)
signal gear_changed(old_gear: int, new_gear: int)
signal traction_control_triggered(traction_loss_ratio: float)
signal skid_detected(skid_intensity: float)
signal engine_rpm_changed(rpm: float)
signal vehicle_damaged(damage_amount: float)

# ============================================================================
# CONSTANTS & CONFIGURATION
# ============================================================================

const MAX_SPEED_KMH: float = 320.0
const ACCELERATION_RATE: float = 15.0
const BRAKING_RATE: float = 25.0
const STEERING_SENSITIVITY: float = 45.0
const MIN_STEERING_SPEED: float = 2.0
const MAX_STEERING_ANGLE: float = PI / 3.0
const TRACTION_CONTROL_THRESHOLD: float = 0.75
const SKID_THRESHOLD: float = 0.85

# Gear ratios (final drive ratio included)
const GEAR_RATIOS: Dictionary = {
	"neutral": 0.0,
	"1st": 3.8,
	"2nd": 2.1,
	"3rd": 1.4,
	"4th": 1.0,
	"5th": 0.85,
	"6th": 0.7,
	"reverse": 3.5
}

const REV_LIMITER_RPM: float = 7500.0
const IDLE_RPM: float = 800.0
const CLUTCH_RELEASE_SPEED: float = 1.0

# Wheel configuration
const WHEEL_BASE: float = 2.5
const TRACK_WIDTH: float = 1.6
const SUSPENSION_TRAVEL: float = 0.3
const SPRING_STIFFNESS: float = 45000.0
const DAMPING_RATE: float = 3500.0

# ============================================================================
# PROPERTIES
# ============================================================================

@export_group("Vehicle Configuration")
@export var vehicle_mass: float = 1500.0: set = _set_vehicle_mass
@export var center_of_mass_offset: Vector3 = Vector3(0.0, -0.3, 0.0): set = _set_center_of_mass_offset
@export var aerodynamic_drag_coefficient: float = 0.32
@export var frontal_area: float = 2.1
@export var rolling_resistance_coefficient: float = 0.015

@export_group("Powertrain Settings")
@export var engine_max_torque: float = 450.0
@export var engine_max_power: float = 300.0
@export var transmission_type: String = "manual"
@export var differential_type: String = "open"

@export_group("Suspension & Wheels")
@export var suspension_stiffness: float = 45000.0: set = _set_suspension_stiffness
@export var suspension_damping: float = 3500.0: set = _set_suspension_damping
@export var tire_friction_coefficient: float = 1.2
@export var tire_camber_angle: float = -0.02

@export_group("Debug")
@export var debug_visualization: bool = false
@export_category("Advanced Tuning")
@export var traction_control_enabled: bool = true
@export var stability_control_enabled: bool = true
@export var anti_roll_bar_stiffness: float = 15000.0

# ============================================================================
# STATE VARIABLES
# ============================================================================

var current_speed: float = 0.0
var current_gear: int = 0
var target_gear: int = 0
var engine_rpm: float = IDLE_RPM
var clutch_engaged: bool = true
var brake_pressed: bool = false
var handbrake_active: bool = false
var traction_loss_ratio: float = 0.0
var skid_intensity: float = 0.0
var acceleration_vector: Vector3 = Vector3.ZERO
var steering_input: float = 0.0
var throttle_input: float = 0.0
var brake_input: float = 0.0

# Powertrain state
var torque_curve: Array[float] = []
var power_curve: Array[float] = []
var fuel_level: float = 100.0
var fuel_consumption_rate: float = 0.0

# Suspension state per wheel
var wheel_positions: Array[Vector3] = []
var wheel_velocities: Array[Vector3] = []
var suspension_compression: Array[float] = []
var wheel_contact_points: Array[Vector3] = []
var wheel_forces: Array[Vector3] = []

# Aerodynamics
var drag_force: Vector3 = Vector3.ZERO
var downforce_force: Vector3 = Vector3.ZERO
var lift_force: Vector3 = Vector3.ZERO

# Input mapping
enum SteeringMode {
	DIRECT,
	RACK_AND_PINION,
	SPEED_SENSITIVE
}
var steering_mode: SteeringMode = SteeringMode.SPEED_SENSITIVE

# Physics simulation variables
var velocity_magnitude: float = 0.0
var angular_velocity: Vector3 = Vector3.ZERO
var slip_ratio: float = 0.0
var slip_angle: float = 0.0

# Cache references
var powertrain_node: Node = null
var suspension_nodes: Array[Node] = []
var wheel_meshes: Array[MeshInstance3D] = []

# ============================================================================
# LIFECYCLE METHODS
# ============================================================================

func _ready() -> void:
	_init_properties()
	_connect_signals()
	_calibrate_curves()
	_setup_wheels()

func _process(delta: float) -> void:
	if not is_inside_tree():
		return
	
	_handle_inputs(delta)
	_update_aerodynamics(delta)
	_update_fuel_system(delta)
	_update_debug_visualization(delta)

func _physics_process(delta: float) -> void:
	if not is_inside_tree():
		return
	
	_apply_physics_simulation(delta)
	_update_wheel_states(delta)
	_check_traction_control()
	_update_engine_state(delta)

func _exit_tree() -> void:
	_disconnect_signals()

# ============================================================================
# INITIALIZATION
# ============================================================================

func _init_properties() -> void:
	# Set up default torque/power curves based on engine specs
	_calibrate_curves()
	
	# Initialize wheel arrays
	wheel_positions.resize(4)
	wheel_velocities.resize(4)
	suspension_compression.resize(4)
	wheel_contact_points.resize(4)
	wheel_forces.resize(4)
	
	for i in range(4):
		wheel_positions[i] = Vector3.ZERO
		wheel_velocities[i] = Vector3.ZERO
		suspension_compression[i] = 0.0
		wheel_contact_points[i] = Vector3.ZERO
		wheel_forces[i] = Vector3.ZERO
	
	current_gear = 0
	target_gear = 0
	engine_rpm = IDLE_RPM
	clutch_engaged = true
	fuel_level = 100.0

func _connect_signals() -> void:
	if GameManager and GameManager.has_signal("game_state_changed"):
		GameManager.game_state_changed.connect(_on_game_state_changed)
	
	if InputManager and InputManager.has_signal("input_updated"):
		InputManager.input_updated.connect(_on_input_updated)

func _disconnect_signals() -> void:
	if GameManager and GameManager.has_signal("game_state_changed"):
		GameManager.game_state_changed.disconnect(_on_game_state_changed)
	
	if InputManager and InputManager.has_signal("input_updated"):
		InputManager.input_updated.disconnect(_on_input_updated)

func _calibrate_curves() -> void:
	"""Generate torque and power curves based on engine characteristics"""
	torque_curve.clear()
	power_curve.clear()
	
	var max_rpm = REV_LIMITER_RPM * 1.2
	var samples = 100
	
	for i in range(samples + 1):
		var rpm = lerp(IDLE_RPM, max_rpm, float(i) / samples)
		var normalized_rpm = clamp((rpm - IDLE_RPM) / (max_rpm - IDLE_RPM), 0.0, 1.0)
		
		# Torque curve (bell-shaped, peak around 40-50% of max RPM)
		var torque_peak_ratio = 0.45
		var torque_value = engine_max_torque * (
			1.0 - pow(normalized_rpm - torque_peak_ratio, 2.0) / (torque_peak_ratio * (1.0 - torque_peak_ratio))
		)
		torque_value = max(torque_value, engine_max_torque * 0.6) # Minimum torque at idle
		
		# Power = Torque * RPM / constant
		var power_value = (torque_value * rpm) / 9549.0 # kW conversion
		
		torque_curve.append(torque_value)
		power_curve.append(power_value)

func _setup_wheels() -> void:
	"""Setup wheel nodes and mesh references"""
	# Find child nodes that match wheel naming convention
	for child in get_children():
		if child.name.contains("Wheel"):
			if child is MeshInstance3D:
				wheel_meshes.append(child)
			elif child is Node3D:
				suspension_nodes.append(child)

# ============================================================================
# INPUT HANDLING
# ============================================================================

func _handle_inputs(delta: float) -> void:
	"""Process input from InputManager and update vehicle controls"""
	if not InputManager:
		return
	
	throttle_input = InputManager.get_axis("accelerator", 0.0, 1.0)
	brake_input = InputManager.get_axis("brake", 0.0, 1.0)
	steering_input = InputManager.get_axis("steering_left_right", -1.0, 1.0)
	
	# Convert brake input to boolean for handbrake detection
	brake_pressed = brake_input > 0.0
	handbrake_active = InputManager.is_action_pressed("handbrake")
	
	# Gear shifting
	_handle_gear_shifting()
	
	# Apply inputs to vehicle
	_apply_throttle(throttle_input * delta)
	_apply_braking(brake_input * delta)
	_apply_steering(steering_input * delta)

func _handle_gear_shifting() -> void:
	"""Handle manual/automatic gear shifting logic"""
	if transmission_type == "automatic":
		_auto_shift_gears()
	else:
		_manual_shift_gears()

func _auto_shift_gears() -> void:
	"""Automatic gear shifting based on RPM and speed"""
	if current_speed < MIN_STEERING_SPEED and engine_rpm < IDLE_RPM * 1.5:
		target_gear = 0 # Neutral when stopped
	elif engine_rpm > REV_LIMITER_RPM * 0.9 and current_gear > 0:
		target_gear = current_gear - 1 # Upshift if over-revving
	elif engine_rpm < IDLE_RPM * 0.8 and current_gear < 6:
		target_gear = min(current_gear + 1, 6) # Downshift if under-loaded
	else:
		target_gear = current_gear
	
	# Smooth gear transition
	if target_gear != current_gear and abs(target_gear - current_gear) <= 1:
		_change_gear(target_gear)

func _manual_shift_gears() -> void:
	"""Manual gear shifting with up/down commands"""
	if InputManager.is_action_pressed("gear_up"):
		if current_gear < 6:
			target_gear = current_gear + 1
		elif current_gear == 6 and engine_rpm > REV_LIMITER_RPM * 0.85:
			target_gear = 6 # Stay in top gear but rev limiter will engage
	
	if InputManager.is_action_pressed("gear_down"):
		if current_gear > 0:
			target_gear = current_gear - 1
		elif current_gear == 0:
			target_gear = 1 # Downshift from neutral to first
	
	if target_gear != current_gear:
		_change_gear(target_gear)

func _change_gear(new_gear: int) -> void:
	"""Execute gear change with clutch engagement"""
	var old_gear = current_gear
	current_gear = new_gear
	
	# Engage clutch during shift
	clutch_engaged = false
	await get_tree().create_timer(0.15).timeout
	clutch_engaged = true
	
	gear_changed.emit(old_gear, current_gear)

func _apply_throttle(amount: float) -> void:
	"""Apply throttle input to engine"""
	if amount > 0.0:
		throttle_input = amount
		# Engine torque increases with throttle
		pass

func _apply_braking(amount: float) -> void:
	"""Apply braking force to wheels"""
	if amount > 0.0:
		brake_input = amount
		# Brake force applied to all wheels
		pass

func _apply_steering(amount: float) -> void:
	"""Apply steering input to front wheels"""
	if current_speed >= MIN_STEERING_SPEED:
		steering_input = amount
	else:
		# No steering at very low speeds for realistic feel
		steering_input = 0.0

func _on_input_updated(input_data: Dictionary) -> void:
	"""Handle external input updates"""
	if "throttle" in input_data:
		throttle_input = input_data.throttle
	if "brake" in input_data:
		brake_input = input_data.brake
	if "steering" in input_data:
		steering_input = input_data.steering
	if "gear" in input_data:
		target_gear = input_data.gear

func _on_game_state_changed(new_state: GameState) -> void:
	"""Handle game state changes"""
	match new_state:
		GameManager.GameState.RACE_ACTIVE:
			engine_rpm = IDLE_RPM
			current_gear = 0
			clutch_engaged = true
		GameManager.GameState.RACE_PAUSED:
			engine_rpm = IDLE_RPM
			current_gear = 0
		GameManager.GameState.MAIN_MENU:
			current_speed = 0.0
			engine_rpm = IDLE_RPM

# ============================================================================
# PHYSICS SIMULATION
# ============================================================================

func _apply_physics_simulation(delta: float) -> void:
	"""Main physics simulation step - applies forces and updates velocity"""
	# Calculate total forces acting on vehicle
	var total_force = Vector3.ZERO
	
	# Add engine/traction force
	var drive_force = _calculate_drive_force()
	total_force += drive_force
	
	# Add braking force
	var brake_force = _calculate_brake_force()
	total_force -= brake_force
	
	# Add aerodynamic forces
	total_force += drag_force
	total_force += downforce_force
	
	# Add gravity component (if on slope)
	var gravity_force = Vector3.DOWN * vehicle_mass * PhysicsSettings.gravity
	total_force += gravity_force
	
	# Apply Newton's second law: F = ma
	var acceleration = total_force / vehicle_mass
	
	# Update velocity
	velocity += acceleration * delta
	
	# Apply drag to velocity for realism
	var velocity_drag_factor = 1.0 - (aerodynamic_drag_coefficient * frontal_area * 0.001)
	velocity *= velocity_drag_factor
	
	# Clamp speed to maximum
	current_speed = velocity.length()
	if current_speed > MAX_SPEED_KMH / 3.6: # Convert km/h to m/s
		velocity = velocity.normalized() * MAX_SPEED_KMH / 3.6
	
	# Update body position
	move_and_slide()
	
	# Emit signals
	speed_changed.emit(current_speed)
	acceleration_vector = acceleration

func _calculate_drive_force() -> Vector3:
	"""Calculate drive force based on engine torque and gear ratio"""
	if current_gear <= 0 or not clutch_engaged:
		return Vector3.ZERO
	
	var gear_ratio = GEAR_RATIOS[str(current_gear)]
	var final_drive_ratio = 3.5 # Typical final drive ratio
	
	var total_ratio = gear_ratio * final_drive_ratio
	
	# Get torque at current RPM
	var torque_index = _get_torque_index_for_rpm(engine_rpm)
	var available_torque = torque_curve[torque_index]
	
	# Apply throttle factor
	var effective_torque = available_torque * throttle_input
	
	# Apply transmission losses (typical 15%)
	var drivetrain_efficiency = 0.85
	effective_torque *= drivetrain_efficiency
	
	# Calculate wheel force (torque / wheel radius)
	var wheel_radius = 0.33 # typical tire radius
	var wheel_torque = effective_torque * total_ratio
	var drive_force = wheel_torque / wheel_radius
	
	# Apply traction limit
	var max_traction_force = _calculate_max_traction_force()
	drive_force = min(drive_force, max_traction_force)
	
	return drive_force * Vector3.FORWARD

func _calculate_brake_force() -> Vector3:
	"""Calculate braking force based on brake input"""
	if not brake_pressed and not handbrake_active:
		return Vector3.ZERO
	
	var brake_pressure = brake_input
	if handbrake_active:
		brake_pressure *= 1.5 # Handbrake provides additional force
	
	# Maximum braking capability
	var max_brake_force = vehicle_mass * PhysicsSettings.gravity * 1.2 # 1.2g braking
	
	var actual_brake_force = max_brake_force * brake_pressure
	
	# Apply to opposite of movement direction
	if current_speed > 0.1:
		return actual_brake_force * -velocity.normalized()
	
	return Vector3.ZERO

func _calculate_max_traction_force() -> float:
	"""Calculate maximum available traction force"""
	var normal_force = vehicle_mass * PhysicsSettings.gravity * 0.5 # Assume even weight distribution
	var friction_limit = normal_force * tire_friction_coefficient
	
	return friction_limit

func _update_aerodynamics(delta: float) -> void:
	"""Calculate aerodynamic forces based on speed"""
	var speed_squared = current_speed * current_speed
	
	# Drag force: Fd = 0.5 * rho * Cd * A * v^2
	var air_density = 1.225 # kg/m^3 at sea level
	drag_force = -velocity.normalized() * 0.5 * air_density * aerodynamic_drag_coefficient * frontal_area * speed_squared
	
	# Downforce (simplified model)
	var downforce_coefficient = 0.5 # Simplified
	downforce_force = Vector3.DOWN * 0.5 * air_density * downforce_coefficient * frontal_area * speed_squared
	
	# Lift (minimal for racing cars)
	lift_force = Vector3.UP * 0.5 * air_density * 0.1 * frontal_area * speed_squared

func _update_wheel_states(delta: float) -> void:
	"""Update individual wheel states and contact physics"""
	# Update wheel positions relative to vehicle
	_update_wheel_positions()
	_update_wheel_contacts()
	_update_suspension_compression(delta)

func _update_wheel_positions() -> void:
	"""Calculate world positions of all four wheels"""
	var half_track = TRACK_WIDTH / 2.0
	var half_wheelbase = WHEEL_BASE / 2.0
	
	# Front left
	wheel_positions[0] = global_position + Vector3(-half_track, 0.0, half_wheelbase)
	# Front right
	wheel_positions[1] = global_position + Vector3(half_track, 0.0, half_wheelbase)
	# Rear left
	wheel_positions[2] = global_position + Vector3(-half_track, 0.0, -half_wheelbase)
	# Rear right
	wheel_positions[3] = global_position + Vector3(half_track, 0.0, -half_wheelbase)

func _update_wheel_contacts() -> void:
	"""Calculate wheel contact points with ground"""
	for i in range(4):
		var wheel_pos = wheel_positions[i]
		var ray_from = wheel_pos + Vector3.UP * SUSPENSION_TRAVEL
		var ray_to = wheel_pos + Vector3.DOWN * (SUSPENSION_TRAVEL + 0.5)
		
		var space_state = get_world_3d().direct_space_state
		var query = PhysicsRayQueryParameters3D.create(ray_from, ray_to)
		var result = space_state.intersect_ray(query)
		
		if result.has("collider"):
			wheel_contact_points[i] = result.position
		else:
			wheel_contact_points[i] = wheel_pos

func _update_suspension_compression(delta: float) -> void:
	"""Calculate suspension compression based on wheel travel"""
	for i in range(4):
		var rest_length = SUSPENSION_TRAVEL
		var compressed_length = (wheel_positions[i] - wheel_contact_points[i]).length()
		
		suspension_compression[i] = rest_length - compressed_length
		suspension_compression[i] = clamp(suspension_compression[i], -0.1, SUSPENSION_TRAVEL)

# ============================================================================
# ENGINE & TRANSMISSION
# ============================================================================

func _update_engine_state(delta: float) -> void:
	"""Update engine RPM based on vehicle dynamics"""
	if current_gear <= 0:
		# Engine idles when in neutral
		engine_rpm = lerp(engine_rpm, IDLE_RPM, delta * 5.0)
	else:
		# Engine RPM tied to wheel speed through gear ratio
		var gear_ratio = GEAR_RATIOS[str(current_gear)]
		var final_drive_ratio = 3.5
		var wheel_radius = 0.33
		var total_ratio = gear_ratio * final_drive_ratio
		
		var wheel_rps = current_speed / (2.0 * PI * wheel_radius)
		var engine_rpm_calc = wheel_rps * total_ratio * 60.0 # Convert to RPM
		
		# Apply clutch slip if disengaged
		if not clutch_engaged:
			engine_rpm = lerp(engine_rpm, IDLE_RPM, delta * 10.0)
		else:
			engine_rpm = lerp(engine_rpm, engine_rpm_calc, delta * 3.0)
		
		# Rev limiter
		if engine_rpm > REV_LIMITER_RPM:
			engine_rpm = REV_LIMITER_RPM
			# Cut fuel/ignition temporarily
			throttle_input *= 0.3
	
	engine_rpm_changed.emit(engine_rpm)

func _get_torque_index_for_rpm(rpm: float) -> int:
	"""Get torque curve index for given RPM"""
	var max_rpm = REV_LIMITER_RPM * 1.2
	var normalized_rpm = clamp((rpm - IDLE_RPM) / (max_rpm - IDLE_RPM), 0.0, 1.0)
	var index = int(normalized_rpm * (torque_curve.size() - 1))
	return max(0, min(index, torque_curve.size() - 1))

# ============================================================================
# TRACTION & STABILITY CONTROL
# ============================================================================

func _check_traction_control() -> void:
	"""Check and apply traction control if necessary"""
	if not traction_control_enabled:
		return
	
	traction_loss_ratio = _calculate_traction_loss()
	
	if traction_loss_ratio > TRACTION_CONTROL_THRESHOLD:
		_trigger_traction_control()
		traction_control_triggered.emit(traction_loss_ratio)
	
	if traction_loss_ratio > SKID_THRESHOLD:
		skid_intensity = traction_loss_ratio
		skid_detected.emit(skid_intensity)

func _calculate_traction_loss() -> float:
	"""Calculate ratio of wheel spin vs ideal traction"""
	var rear_wheel_indices = [2, 3] # Rear-wheel drive
	var max_slip = 0.0
	
	for idx in rear_wheel_indices:
		var wheel_velocity = wheel_velocities[idx].length()
		var vehicle_velocity = current_speed
		if vehicle_velocity > 0.1:
			var slip = abs(wheel_velocity - vehicle_velocity) / vehicle_velocity
			max_slip = max(max_slip, slip)
	
	return max_slip

func _trigger_traction_control() -> void:
	"""Apply traction control intervention"""
	# Reduce throttle proportionally to traction loss
	var reduction_factor = 1.0 - ((traction_loss_ratio - TRACTION_CONTROL_THRESHOLD) / 0.1)
	throttle_input = max(0.0, throttle_input * reduction_factor)

# ============================================================================
# FUEL SYSTEM
# ============================================================================

func _update_fuel_system(delta: float) -> void:
	"""Update fuel consumption based on engine load"""
	if engine_rpm > IDLE_RPM * 1.5 and throttle_input > 0.0:
		# Fuel consumption increases with RPM and throttle
		var consumption = fuel_consumption_rate * throttle_input * (engine_rpm / REV_LIMITER_RPM)
		fuel_level -= consumption * delta
		fuel_level = max(0.0, min(100.0, fuel_level))

func refuel(amount: float) -> void:
	"""Refuel the vehicle"""
	fuel_level = min(100.0, fuel_level + amount)

# ============================================================================
# DAMAGE SYSTEM
# ============================================================================

func take_damage(damage_amount: float) -> void:
	"""Apply damage to vehicle"""
	damage_amount = max(0.0, damage_amount)
	vehicle_damaged.emit(damage_amount)
	
	# Reduce performance based on damage
	engine_max_torque *= (1.0 - damage_amount * 0.01)
	engine_max_power *= (1.0 - damage_amount * 0.01)
	tire_friction_coefficient *= (1.0 - damage_amount * 0.005)

# ============================================================================
# DEBUG VISUALIZATION
# ============================================================================

func _update_debug_visualization(delta: float) -> void:
	"""Draw debug visualization if enabled"""
	if not debug_visualization:
		return
	
	_draw_wheel_markers()
	_draw_force_vectors()
	_draw_suspension_lines()

func _draw_wheel_markers() -> void:
	"""Draw markers at wheel contact points"""
	for i in range(4):
		if DebugRenderer:
			DebugRenderer.draw_sphere(wheel_contact_points[i], 0.1, Color.GREEN)

func _draw_force_vectors() -> void:
	"""Draw force vectors on vehicle"""
	if DebugRenderer:
		# Draw drag force
		DebugRenderer.draw_arrow(global_position, global_position + drag_force * 0.01, Color.RED)
		
		# Draw downforce
		DebugRenderer.draw_arrow(global_position, global_position + downforce_force * 0.01, Color.BLUE)

func _draw_suspension_lines() -> void:
	"""Draw suspension compression lines"""
	for i in range(4):
		if DebugRenderer and suspension_compression[i] > 0.01:
			DebugRenderer.draw_line(
				wheel_positions[i],
				wheel_positions[i] + Vector3.DOWN * suspension_compression[i],
				Color.YELLOW
			)

# ============================================================================
# GETTERS & SETTERS
# ============================================================================

func get_current_speed_kmh() -> float:
	"""Get current speed in km/h"""
	return current_speed * 3.6

func get_current_gear_name() -> String:
	"""Get current gear as string name"""
	if current_gear == 0:
		return "neutral"
	elif current_gear == -1:
		return "reverse"
	else:
		return str(current_gear) + "th"

func get_engine_load() -> float:
	"""Get current engine load percentage"""
	if engine_rpm < IDLE_RPM:
		return 0.0
	var torque_index = _get_torque_index_for_rpm(engine_rpm)
	var max_torque_at_rpm = torque_curve[torque_index]
	if max_torque_at_rpm == 0:
		return 0.0
	return (engine_max_torque * throttle_input) / max_torque_at_rpm * 100.0

func _set_vehicle_mass(value: float) -> void:
	vehicle_mass = value
	rebuild_collision_shapes()

func _set_center_of_mass_offset(value: Vector3) -> void:
	center_of_mass_offset = value
	update_center_of_mass()

func _set_suspension_stiffness(value: float) -> void:
	suspension_stiffness = value
	for node in suspension_nodes:
		if node has_method "set_stiffness":
			node.set_stiffness(value)

func _set_suspension_damping(value: float) -> void:
	suspension_damping = value
	for node in suspension_nodes:
		if node has_method "set_damping":
			node.set_damping(value)

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================

func reset_vehicle() -> void:
	"""Reset vehicle to initial state"""
	current_speed = 0.0
	velocity = Vector3.ZERO
	current_gear = 0
	target_gear = 0
	engine_rpm = IDLE_RPM
	clutch_engaged = true
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	fuel_level = 100.0
	traction_loss_ratio = 0.0
	skid_intensity = 0.0

func save_state() -> Dictionary:
	"""Save current vehicle state for saving/loading"""
	return {
		"speed": current_speed,
		"gear": current_gear,
		"rpm": engine_rpm,
		"fuel": fuel_level,
		"position": global_position,
		"rotation": global_rotation,
		"velocity": velocity
	}

func load_state(state_data: Dictionary) -> void:
	"""Load vehicle state from saved data"""
	if state_data.has("speed"):
		current_speed = state_data.speed
	if state_data.has("gear"):
		current_gear = state_data.gear
	if state_data.has("rpm"):
		engine_rpm = state_data.rpm
	if state_data.has("fuel"):
		fuel_level = state_data.fuel
	if state_data.has("position"):
		global_position = state_data.position
	if state_data.has("rotation"):
		global_rotation = state_data.rotation
	if state_data.has("velocity"):
		velocity = state_data.velocity

func get_vehicle_stats() -> Dictionary:
	"""Get comprehensive vehicle statistics"""
	return {
		"speed_kmh": get_current_speed_kmh(),
		"current_gear": current_gear,
		"gear_name": get_current_gear_name(),
		"engine_rpm": engine_rpm,
		"engine_load": get_engine_load(),
		"fuel_level": fuel_level,
		"throttle": throttle_input,
		"brake": brake_input,
		"steering": steering_input,
		"traction_loss": traction_loss_ratio,
		"skid_intensity": skid_intensity
	}

</FILE>