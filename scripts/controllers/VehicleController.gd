extends Node3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for the racing simulator
## Implements throttle, brake, steering inputs, wheel forces, and gear shifting logic
## Uses PhysicsSettings constants for centralized tuning
## Copyright 2026 Thalamus Racing Simulator Project

# Signals for external communication
signal speed_changed(current_speed: float)
signal rpm_changed(current_rpm: float)
signal gear_changed(new_gear: int)
signal skid_detected(skid_factor: float)
signal vehicle_collision(vehicle: Node3D)

# Constants for vehicle dynamics
const MAX_STEERING_ANGLE: float = PI / 3  # 60 degrees max steering
const STEERING_SPEED: float = 15.0  # radians per second
const ABS_THRESHOLD: float = 0.95  # wheel slip threshold for ABS
const TRACTION_CONTROL_THRESHOLD: float = 0.90  # wheel slip threshold for TCS

# State tracking
var current_speed: float = 0.0  # m/s
var current_rpm: float = 0.0  # revolutions per minute
var current_gear: int = 0  # -1 = reverse, 0 = neutral, 1+ = forward gears
var is_engine_running: bool = false
var is_braking: bool = false
var is_throttling: bool = false
var is_steering_left: bool = false
var is_steering_right: bool = false
var skid_factor: float = 0.0  # 0.0 = no skid, 1.0 = maximum skid

# Input values (normalized -1 to 1)
var throttle_input: float = 0.0
var brake_input: float = 0.0
var steering_input: float = 0.0

# Physics properties (configured via Powertrain or externally)
var vehicle_mass: float = 1500.0
var wheel_base: float = 2.5  # meters between front and rear axles
var track_width: float = 1.6  # meters between left and right wheels
var wheel_radius: float = 0.32  # meters
var tire_friction_coefficient: float = 1.2  # dry asphalt
var aerodynamic_drag_coefficient: float = 0.30  # Cd value
var frontal_area: float = 2.2  # square meters
var center_of_gravity_height: float = 0.5  # meters above ground

# Engine/Powertrain settings
var engine_max_rpm: float = 8000.0
var engine_min_rpm: float = 800.0  # idle RPM
var torque_curve: Array[float] = []  # indexed by normalized RPM
var power_curve: Array[float] = []  # indexed by normalized RPM
var gear_ratios: Array[float] = []
var final_drive_ratio: float = 3.73
var rev_limiters: Array[float] = []

# Suspension and wheel state
var suspension_compression: Vector4 = Vector4.ZERO  # FL, FR, RL, RR
var wheel_rotation_angles: Vector4 = Vector4.ZERO  # FL, FR, RL, RR
var wheel_slip_ratios: Vector4 = Vector4.ONE  # FL, FR, RL, RR (1.0 = no slip)
var wheel_contact_forces: Vector4 = Vector4.ZERO  # vertical force on each wheel

# Brake system
var brake_pressure: float = 0.0  # 0.0 to 1.0
var brake_force_distribution: Vector4 = Vector4(0.45, 0.45, 0.55, 0.55)  # bias toward rear
var abs_active: bool = false
var traction_control_active: bool = false

# Aerodynamics and downforce
var downforce_coefficient: float = 1.5  # lift coefficient (negative = downforce)
var wing_angle_degrees: float = 0.0
var active_aero_enabled: bool = false

# Gearbox state
var gearbox_type: String = "manual"  # "manual", "automatic", "sequential"
var upshift_threshold: float = 7500.0  # RPM for auto upshift
var downshift_threshold: float = 2000.0  # RPM for auto downshift
var clutch_engaged: bool = true

func _ready() -> void:
	_process_mode = ProcessModeEnum.ALWAYS
	_init_default_curves()
	_connect_signals_to_input_manager()

func _init_default_curves() -> void:
	"""Initialize default torque and power curves for a typical V8 engine"""
	# Torque curve (indexed by normalized RPM 0.0 to 1.0)
	torque_curve = [0.3, 0.5, 0.75, 0.95, 1.0, 0.95, 0.85, 0.70, 0.55, 0.40]
	
	# Power curve (derived from torque * RPM)
	power_curve = [0.0, 0.15, 0.45, 0.75, 1.0, 0.98, 0.90, 0.75, 0.55, 0.35]
	
	# Gear ratios (typical 6-speed manual transmission)
	gear_ratios = [3.8, 2.2, 1.5, 1.1, 0.9, 0.75]
	
	# Rev limiters per gear
	rev_limiters = [7200.0, 7500.0, 7700.0, 7800.0, 7900.0, 8000.0]

func _connect_signals_to_input_manager() -> void:
	"""Connect to InputManager for input handling"""
	if InputManager.has_method("register_vehicle_input"):
		InputManager.register_vehicle_input(self)

func _physics_process(delta: float) -> void:
	"""Main physics update loop - runs at fixed timestep"""
	_update_inputs()
	_update_engine_state(delta)
	_update_gearbox(delta)
	_update_suspension_and_wheels(delta)
	_apply_forces_to_vehicle(delta)
	_check_traction_and_skids(delta)
	_emit_signals()

func _update_inputs() -> void:
	"""Update input values from InputManager or direct key detection"""
	# Get input values (normalized -1 to 1)
	throttle_input = InputManager.get_axis("throttle")
	brake_input = InputManager.get_axis("brake")
	steering_input = InputManager.get_axis("steer_left") - InputManager.get_axis("steer_right")
	
	# Clamp inputs
	throttle_input = clamp(throttle_input, -1.0, 1.0)
	brake_input = clamp(brake_input, -1.0, 1.0)
	steering_input = clamp(steering_input, -1.0, 1.0)
	
	# Update boolean states
	is_throttling = throttle_input > 0.1
	is_braking = brake_input > 0.1
	is_steering_left = steering_input < -0.1
	is_steering_right = steering_input > 0.1
	
	# Handle gear shifting via input
	_handle_gear_shifting()

func _handle_gear_shifting() -> void:
	"""Handle manual gear shifting from input"""
	if InputManager.is_action_just_pressed("gear_up"):
		shift_gear(1)
	elif InputManager.is_action_just_pressed("gear_down"):
		shift_gear(-1)
	elif InputManager.is_action_just_pressed("neutral"):
		set_gear(0)
	elif InputManager.is_action_just_pressed("reverse"):
		if current_gear == 0:
			set_gear(-1)

func _update_engine_state(delta: float) -> void:
	"""Update engine RPM and torque based on current state"""
	if not is_engine_running:
		current_rpm = engine_min_rpm
		return
	
	# Calculate target RPM based on throttle and current gear
	var target_rpm: float = _calculate_target_rpm()
	
	# Apply engine inertia (RPM doesn't change instantaneously)
	var engine_inertia: float = 0.15  # seconds to reach target
	target_rpm = lerp(current_rpm, target_rpm, delta * engine_inertia)
	
	# Apply rev limiter
	if current_gear >= 0 and current_gear < rev_limiters.size():
		target_rpm = min(target_rpm, rev_limiters[current_gear])
	
	current_rpm = target_rpm

func _calculate_target_rpm() -> float:
	"""Calculate desired RPM based on throttle input and current conditions"""
	if clutch_engaged and current_gear != 0:
		# Engine connected to wheels - RPM tied to vehicle speed
		var wheel_speed_rps: float = current_speed / (2.0 * PI * wheel_radius)
		var total_reduction: float = gear_ratios[current_gear] * final_drive_ratio
		var engine_rpm_from_wheels: float = wheel_speed_rps * 60.0 * total_reduction
		
		# Add some slip based on throttle
		var throttle_multiplier: float = 1.0 + (throttle_input * 0.1)
		var target_rpm: float = engine_rpm_from_wheels * throttle_multiplier
		
		return max(engine_min_rpm, target_rpm)
	else:
		# Engine free-revving
		var idle_rpm: float = engine_min_rpm
		var max_rpm: float = engine_max_rpm
		var throttle_range: float = max_rpm - idle_rpm
		
		return idle_rpm + (throttle_range * throttle_input)

func _update_gearbox(delta: float) -> void:
	"""Update gearbox state and handle automatic shifting"""
	if gearbox_type == "automatic":
		_auto_shift_gears(delta)
	elif gearbox_type == "sequential":
		# Sequential boxes don't shift automatically, only on input
		pass

func _auto_shift_gears(delta: float) -> void:
	"""Automatic gear shifting logic"""
	if current_gear == -1:
		return  # Can't auto-shift in reverse
	
	# Upshift if RPM too high
	if current_rpm > upshift_threshold and current_gear < gear_ratios.size() - 1:
		shift_gear(1)
	
	# Downshift if RPM too low and moving
	if current_rpm < downshift_threshold and current_gear > 0 and current_speed > 5.0:
		shift_gear(-1)

func shift_gear(direction: int) -> void:
	"""Shift gear by direction (-1 = down, 1 = up)"""
	var new_gear: int = current_gear + direction
	
	# Validate gear range
	if gearbox_type == "manual":
		new_gear = clamp(new_gear, -1, gear_ratios.size())
	elif gearbox_type in ["automatic", "sequential"]:
		new_gear = clamp(new_gear, 0, gear_ratios.size())
	
	# Prevent shifting into reverse while moving forward
	if new_gear == -1 and current_speed > 2.0:
		new_gear = 0
	
	if new_gear != current_gear:
		current_gear = new_gear
		gear_changed.emit(current_gear)
		
		# Simulate clutch engagement delay for manual
		if gearbox_type == "manual":
			clutch_engaged = false
			await get_tree().create_timer(0.15).timeout
			clutch_engaged = true

func set_gear(gear: int) -> void:
	"""Set gear directly (for neutral or specific gear selection)"""
	if gear == -1 and current_speed > 2.0:
		return  # Safety check
	
	current_gear = gear
	gear_changed.emit(current_gear)

func _update_suspension_and_wheels(delta: float) -> void:
	"""Update suspension compression and wheel rotation angles"""
	# This would normally read from rigid body constraints
	# For now, simulate basic suspension behavior
	
	# Calculate wheel angular velocity based on vehicle speed and gear
	if clutch_engaged and current_gear != 0 and current_speed > 0.1:
		var wheel_speed_rps: float = current_speed / (2.0 * PI * wheel_radius)
		var total_reduction: float = gear_ratios[current_gear] * final_drive_ratio
		var wheel_angular_velocity: float = wheel_speed_rps * 60.0 * total_reduction
		
		# Update wheel rotation angles
		wheel_rotation_angles.x += wheel_angular_velocity * delta
		wheel_rotation_angles.y += wheel_angular_velocity * delta
		wheel_rotation_angles.z += wheel_angular_velocity * delta
		wheel_rotation_angles.w += wheel_angular_velocity * delta
	else:
		# Wheels spinning freely when engine disengaged
		var free_spin_rate: float = current_rpm * 60.0 / final_drive_ratio
		wheel_rotation_angles.x += free_spin_rate * delta * 0.5
		wheel_rotation_angles.y += free_spin_rate * delta * 0.5
		wheel_rotation_angles.z += free_spin_rate * delta * 0.5
		wheel_rotation_angles.w += free_spin_rate * delta * 0.5

func _apply_forces_to_vehicle(delta: float) -> void:
	"""Apply all forces to the vehicle (engine, brakes, drag, etc.)"""
	if not is_inside_tree():
		return
	
	# Get reference to the rigid body
	var vehicle_body: RigidBody3D = _get_vehicle_body()
	if vehicle_body == null:
		return
	
	# Calculate engine force
	var engine_force: float = _calculate_engine_force()
	
	# Apply braking force
	var braking_force: float = _calculate_braking_force()
	
	# Apply aerodynamic drag
	var drag_force: float = _calculate_aerodynamic_drag()
	
	# Net force along vehicle forward direction
	var net_force: float = engine_force - braking_force - drag_force
	
	# Apply acceleration to vehicle
	var acceleration: float = net_force / vehicle_mass
	
	# Update speed (simplified longitudinal dynamics)
	current_speed += acceleration * delta
	
	# Clamp speed to realistic limits
	current_speed = max(0.0, current_speed)
	
	# Apply forces to rigid body if available
	if vehicle_body.is_connected("body_entered", _on_body_entered):
		vehicle_body.apply_central_impulse(Vector3.FORWARD * net_force * delta)

func _calculate_engine_force() -> float:
	"""Calculate force produced by engine at current RPM and gear"""
	if current_gear == 0 or not clutch_engaged:
		return 0.0
	
	if current_gear == -1:
		# Reverse gear - use first gear ratio but negative
		var effective_ratio: float = gear_ratios[0] * final_drive_ratio
	else:
		var gear_index: int = current_gear - 1
		gear_index = clamp(gear_index, 0, gear_ratios.size() - 1)
		var effective_ratio: float = gear_ratios[gear_index] * final_drive_ratio
	
	# Get torque from curve based on normalized RPM
	var normalized_rpm: float = current_rpm / engine_max_rpm
	var torque_index: int = floor(normalized_rpm * (torque_curve.size() - 1))
	torque_index = clamp(torque_index, 0, torque_curve.size() - 1)
	var torque_mult: float = torque_curve[torque_index]
	
	var engine_torque: float = 450.0 * torque_mult  # Peak torque 450 Nm
	
	# Convert torque to wheel force
	var wheel_torque: float = engine_torque * effective_ratio * 0.85  # 85% drivetrain efficiency
	var wheel_force: float = wheel_torque / wheel_radius
	
	# Apply throttle multiplier
	wheel_force *= (0.5 + throttle_input * 0.5)  # 0.5 to 1.0 based on throttle
	
	return wheel_force

func _calculate_braking_force() -> float:
	"""Calculate braking force based on brake pressure and distribution"""
	if not is_braking and brake_pressure <= 0.0:
		return 0.0
	
	# Maximum braking force (based on tire friction)
	var max_brake_force: float = vehicle_mass * PhysicsSettings.gravity * tire_friction_coefficient * 1.2
	
	# Apply brake pressure
	var brake_force: float = max_brake_force * brake_pressure
	
	# Distribute force across wheels (already factored into brake_force_distribution)
	return brake_force

func _calculate_aerodynamic_drag() -> float:
	"""Calculate aerodynamic drag force"""
	var air_density: float = 1.225  # kg/m³ at sea level
	
	var drag_force: float = 0.5 * air_density * aerodynamic_drag_coefficient * frontal_area * current_speed * current_speed
	
	return drag_force

func _check_traction_and_skids(delta: float) -> void:
	"""Check for wheel slip and activate ABS/TCS if needed"""
	# Simplified traction check - in production, this would use actual wheel sensors
	
	var max_traction_force: float = vehicle_mass * PhysicsSettings.gravity * tire_friction_coefficient
	
	if is_throttling and current_gear >= 0:
		var engine_force: float = _calculate_engine_force()
		
		# Check if engine force exceeds traction limit
		if engine_force > max_traction_force:
			traction_control_active = true
			
			# Reduce engine output
			throttle_input *= 0.7
			
			skid_factor = min(1.0, (engine_force - max_traction_force) / max_traction_force)
			
			if skid_factor > 0.1:
				skid_detected.emit(skid_factor)
	
	if is_braking:
		# Check for wheel lockup (ABS)
		if current_speed > 1.0:
			abs_active = true
			
			# Modulate brake pressure to prevent lockup
			brake_pressure = lerp(brake_pressure, 0.8, delta * 10.0)

func _emit_signals() -> void:
	"""Emit signals for external systems"""
	speed_changed.emit(current_speed)
	rpm_changed.emit(current_rpm)

func _get_vehicle_body() -> RigidBody3D:
	"""Get the vehicle's rigid body component"""
	for child in get_children():
		if child is RigidBody3D:
			return child
	return null

func _on_body_entered(body: Node) -> void:
	"""Handle collision events"""
	vehicle_collision.emit(body)

# Public API methods for external control

func start_engine() -> void:
	"""Start the engine"""
	is_engine_running = true
	current_rpm = engine_min_rpm

func stop_engine() -> void:
	"""Stop the engine"""
	is_engine_running = false
	current_rpm = engine_min_rpm

func toggle_engine() -> void:
	"""Toggle engine state"""
	if is_engine_running:
		stop_engine()
	else:
		start_engine()

func set_throttle(value: float) -> void:
	"""Set throttle manually (for AI or remote control)"""
	throttle_input = clamp(value, -1.0, 1.0)
	is_throttling = throttle_input > 0.1

func set_brake(value: float) -> void:
	"""Set brake manually"""
	brake_input = clamp(value, -1.0, 1.0)
	is_braking = brake_input > 0.1
	brake_pressure = brake_input

func set_steering(value: float) -> void:
	"""Set steering angle manually"""
	steering_input = clamp(value, -1.0, 1.0)
	is_steering_left = steering_input < -0.1
	is_steering_right = steering_input > 0.1

func reset_all() -> void:
	"""Reset vehicle controller to initial state"""
	current_speed = 0.0
	current_rpm = engine_min_rpm
	current_gear = 0
	is_engine_running = false
	is_braking = false
	is_throttling = false
	is_steering_left = false
	is_steering_right = false
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	brake_pressure = 0.0
	abs_active = false
	traction_control_active = false
	skid_factor = 0.0

func get_vehicle_stats() -> Dictionary:
	"""Get current vehicle statistics as a dictionary"""
	return {
		"speed": current_speed,
		"speed_kmh": current_speed * 3.6,
		"rpm": current_rpm,
		"gear": current_gear,
		"throttle": throttle_input,
		"brake": brake_input,
		"steering": steering_input,
		"brake_pressure": brake_pressure,
		"is_engine_running": is_engine_running,
		"skid_factor": skid_factor,
		"abs_active": abs_active,
		"traction_control_active": traction_control_active
	}

func apply_custom_tuning(settings: Dictionary) -> void:
	"""Apply custom vehicle tuning settings"""
	if settings.has("mass"):
		vehicle_mass = settings["mass"]
	if settings.has("tire_friction"):
		tire_friction_coefficient = settings["tire_friction"]
	if settings.has("drag_coefficient"):
		aerodynamic_drag_coefficient = settings["drag_coefficient"]
	if settings.has("frontal_area"):
		frontal_area = settings["frontal_area"]
	if settings.has("max_rpm"):
		engine_max_rpm = settings["max_rpm"]
	if settings.has("torque_curve"):
		torque_curve = settings["torque_curve"]
	if settings.has("power_curve"):
		power_curve = settings["power_curve"]
	if settings.has("gear_ratios"):
		gear_ratios = settings["gear_ratios"]

func save_configuration(path: String) -> void:
	"""Save current configuration to file"""
	var config: Dictionary = get_vehicle_stats()
	config["tuning"] = {
		"vehicle_mass": vehicle_mass,
		"tire_friction_coefficient": tire_friction_coefficient,
		"aerodynamic_drag_coefficient": aerodynamic_drag_coefficient,
		"frontal_area": frontal_area,
		"engine_max_rpm": engine_max_rpm,
		"torque_curve": torque_curve,
		"gear_ratios": gear_ratios
	}
	
	var json_string: String = JSON.stringify(config, "\t")
	var file := FileAccess.open(path, FileAccess.WRITE)
	if file:
		file.store_string(json_string)
		file.close()

func load_configuration(path: String) -> bool:
	"""Load configuration from file"""
	if not FileAccess.file_exists(path):
		return false
	
	var file := FileAccess.open(path, FileAccess.READ)
	if not file:
		return false
	
	var json_string: String = file.get_as_text()
	file.close()
	
	var parser := JSON.new()
	var error := parser.parse(json_string)
	if error != OK:
		push_warning("Failed to parse vehicle config: %s" % parser.get_error_message())
		return false
	
	var config: Dictionary = parser.data
	
	if config.has("tuning"):
		apply_custom_tuning(config["tuning"])
	
	return true

</gd_script>