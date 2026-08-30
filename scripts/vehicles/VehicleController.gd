extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulation
## Handles throttle, brake, steering, wheel forces, gear shifting, and vehicle dynamics
## Integrates with PhysicsSettings, Powertrain, and game systems
## Copyright 2026 Thalamus Racing Simulator Project

# ============================================================================
# SIGNALS
# ============================================================================

signal speed_changed(new_speed: float)
signal rpm_changed(new_rpm: float)
signal gear_changed(old_gear: int, new_gear: int)
signal clutch_depressed(pressed: bool)
signal vehicle_collided(other: Node, impact_force: float)
signal vehicle_destroyed()

# ============================================================================
# EXPORTED PHYSICS SETTINGS
# ============================================================================

@export_group("Vehicle Physics Configuration")
@export var max_throttle_force: float = 40000.0
@export var max_brake_force: float = 60000.0
@export var max_steering_angle: float = 0.6 # radians (~34 degrees)
@export var steering_sensitivity: float = 0.5
@export var steer_recovery_speed: float = 3.0
@export var drift_factor: float = 0.92
@export var grip_coefficient: float = 0.95

@export_group("Wheel Configuration")
@export var track_width: float = 1.6
@export var wheelbase: float = 2.7
@export var wheel_radius: float = 0.32
@export var wheel_mass: float = 25.0
@export var suspension_max_travel: float = 0.15
@export var suspension_stiffness: float = 15000.0
@export var suspension_damping: float = 2000.0

@export_group("Gear Ratios & Transmission")
@export var gear_ratios: Array[float] = [3.5, 2.2, 1.5, 1.1, 0.85, 0.65]
@export var reverse_ratio: float = -3.8
@export var final_drive_ratio: float = 3.2
@export var idle_rpm: float = 800.0
@export var redline_rpm: float = 7500.0
@export var shift_up_threshold: float = 7000.0
@export var shift_down_threshold: float = 1800.0
@export var neutral_rpm: float = 1000.0

@export_group("Brake System")
@export var brake_pressure_distribution: Vector4 = Vector4(0.6, 0.4, 0.6, 0.4)
@export var abs_enabled: bool = true
@export var abs_threshold: float = 0.15
@export var brake_bias_front: float = 0.6

@export_group("Aerodynamics")
@export var drag_coefficient: float = 0.32
@export var frontal_area: float = 2.2
@export var downforce_coefficient: float = 0.8
@export var wing_angle: float = 0.15

# ============================================================================
# STATE VARIABLES
# ============================================================================

var current_rpm: float = idle_rpm
var current_gear: int = 0 # 0 = neutral, 1-6 = forward, -1 = reverse
var target_gear: int = 0
var clutch_engaged: bool = true
var clutch_position: float = 1.0 # 1.0 = fully engaged, 0.0 = fully depressed
var steering_angle: float = 0.0
var current_steering_input: float = 0.0

# Input values (normalized -1.0 to 1.0)
var throttle_input: float = 0.0
var brake_input: float = 0.0
var handbrake_input: bool = false
var shift_up_request: bool = false
var shift_down_request: bool = false

# Current vehicle state
var current_speed_kmh: float = 0.0
var acceleration: Vector3 = Vector3.ZERO
var angular_velocity_local: Vector3 = Vector3.ZERO
var slip_ratio: float = 0.0

# Wheel state
var front_left_wheel: RayCast3D = null
var front_right_wheel: RayCast3D = null
var rear_left_wheel: RayCast3D = null
var rear_right_wheel: RayCast3D = null

# Suspension state
var front_left_suspension: float = 0.0
var front_right_suspension: float = 0.0
var rear_left_suspension: float = 0.0
var rear_right_suspension: float = 0.0

# Powertrain reference
var powertrain: Powertrain = null
var chassis_mesh: Node3D = null
var collision_shape: CollisionShape3D = null

# Constants
var _wheel_positions: Dictionary = {}
var _last_turn_timestamp: float = 0.0
var _vehicle_mass: float = 1500.0
var _air_density: float = 1.225

# Audio references
var _audio_source: AudioStreamPlayer3D = null

# ============================================================================
# LIFECYCLE
# ============================================================================

func _ready() -> void:
	_init_vehicle()
	_connect_signals()
	_setup_wheels()
	_apply_initial_settings()

func _init_vehicle() -> void:
	"""Initialize vehicle components and setup"""
	# Get required nodes from scene
	chassis_mesh = $ChassisMesh if $ChassisMesh else null
	collision_shape = $CollisionShape3D if $CollisionShape3D else null
	
	# Setup wheel raycasts if they exist
	if has_node("Wheels/FrontLeft"):
		front_left_wheel = $Wheels/FrontLeft as RayCast3D
	if has_node("Wheels/FrontRight"):
		front_right_wheel = $Wheels/FrontRight as RayCast3D
	if has_node("Wheels/RearLeft"):
		rear_left_wheel = $Wheels/RearLeft as RayCast3D
	if has_node("Wheels/RearRight"):
		rear_right_wheel = $Wheels/RearRight as RayCast3D
	
	# Get audio source
	_audio_source = $AudioSource if $AudioSource else null
	
	# Initialize wheel positions relative to center
	var half_track = track_width * 0.5
	var half_wb = wheelbase * 0.5
	
	_wheel_positions = {
		"front_left": Vector3(-half_wb, -half_track, 0),
		"front_right": Vector3(-half_wb, half_track, 0),
		"rear_left": Vector3(half_wb, -half_track, 0),
		"rear_right": Vector3(half_wb, half_track, 0)
	}
	
	# Set initial mass from settings
	_vehicle_mass = PhysicsSettings.default_vehicle_mass

func _connect_signals() -> void:
	"""Connect to game manager and other system signals"""
	if GameManager:
		GameManager.race_started.connect(_on_race_started)
		GameManager.vehicle_destroyed.connect(_on_vehicle_destroyed)
	
	if AudioManager:
		AudioManager.sound_played.connect(_on_sound_played)

func _setup_wheels() -> void:
	"""Setup wheel raycast targets and physics"""
	for wheel_key in _wheel_positions:
		var position = _wheel_positions[wheel_key]
		var wheel_node = $"Wheels/{wheel_key.capitalize()}".get_node_or_null("RayCast3D")
		if wheel_node:
			wheel_node.target_position = Vector3(position.x, -wheel_radius + 0.1, position.z)
			wheel_node.collision_mask = 1 # Layer 1

func _apply_initial_settings() -> void:
	"""Apply initial configuration values"""
	current_gear = 0
	target_gear = 0
	clutch_engaged = true
	clutch_position = 1.0
	current_rpm = idle_rpm
	throttle_input = 0.0
	brake_input = 0.0
	steering_angle = 0.0

func _process(delta: float) -> void:
	"""Process non-physics updates (UI, inputs, etc.)"""
	_handle_input_processing(delta)
	_update_display_values()

func _physics_process(delta: float) -> void:
	"""Main physics update loop - runs at fixed timestep"""
	# Safety check for delta
	if delta > 0.1:
		return
	
	# Update physics state
	_update_physics(delta)
	_handle_gear_shifting(delta)
	_apply_forces_to_body()
	_move_and_slide()

# ============================================================================
# INPUT HANDLING
# ============================================================================

func _handle_input_processing(delta: float) -> void:
	"""Process player input and normalize values"""
	# Throttle input
	if Input.is_action_pressed("game_throttle"):
		throttle_input = clamp(throttle_input + delta * 2.0, 0.0, 1.0)
	elif Input.is_action_just_released("game_throttle"):
		throttle_input = 0.0
	
	# Brake input
	if Input.is_action_pressed("game_brake"):
		brake_input = clamp(brake_input + delta * 2.0, 0.0, 1.0)
	elif Input.is_action_just_released("game_brake"):
		brake_input = 0.0
	
	# Handbrake
	if Input.is_action_pressed("game_handbrake"):
		handbrake_input = true
	else:
		handbrake_input = false
	
	# Steering input
	if Input.is_action_pressed("game_steering_left"):
		current_steering_input = -1.0 * steering_sensitivity
	elif Input.is_action_pressed("game_steering_right"):
		current_steering_input = 1.0 * steering_sensitivity
	else:
		current_steering_input = 0.0
	
	# Gear shift up
	if Input.is_action_just_pressed("game_shift_up"):
		shift_up_request = true
	
	# Gear shift down
	if Input.is_action_just_pressed("game_shift_down"):
		shift_down_request = true
	
	# Clutch control (if enabled)
	if Input.is_action_pressed("game_clutch"):
		clutch_position = lerp(clutch_position, 0.0, delta * 5.0)
	else:
		clutch_position = lerp(clutch_position, 1.0, delta * 5.0)
	
	# Auto-shift support
	if not Input.is_action_pressed("game_clutch"):
		if throttle_input > 0.1 and current_gear < gear_ratios.size():
			if current_rpm >= shift_up_threshold:
				shift_up_request = true

func _update_display_values() -> void:
	"""Update visual display values and emit signals"""
	# Convert m/s to km/h
	current_speed_kmh = velocity.length() * 3.6
	
	emit_signal("speed_changed", current_speed_kmh)
	emit_signal("rpm_changed", current_rpm)

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================

func _handle_gear_shifting(delta: float) -> void:
	"""Handle automatic and manual gear shifting"""
	if shift_up_request and clutch_position < 0.3:
		_attempt_shift_up()
		shift_up_request = false
	
	if shift_down_request and clutch_position < 0.3:
		_attempt_shift_down()
		shift_down_request = false

func _attempt_shift_up() -> void:
	"""Attempt to shift to next gear"""
	if current_gear < gear_ratios.size():
		var old_gear = current_gear
		current_gear += 1
		target_gear = current_gear
		
		# Rev matching for smooth upshift
		var target_rpm = idle_rpm
		if current_gear == 0 or target_rpm < idle_rpm:
			target_rpm = idle_rpm
		
		# Short neutral period for mechanical realism
		if current_gear > 0:
			_set_neutral_temporarily()
		
		emit_signal("gear_changed", old_gear, current_gear)
		if AudioManager:
			AudioManager.play_sound("gear_up")

func _attempt_shift_down() -> void:
	"""Attempt to shift to lower gear"""
	if current_gear > 1:
		var old_gear = current_gear
		current_gear -= 1
		target_gear = current_gear
		
		# Rev match for downshift
		var target_rpm = idle_rpm
		if current_speed_kmh > 5:
			target_rpm = _calculate_target_rpm_for_speed(current_speed_kmh / 3.6, current_gear)
			target_rpm = clamp(target_rpm, idle_rpm, redline_rpm)
		
		# Short neutral period
		if current_gear > 0:
			_set_neutral_temporarily()
		
		emit_signal("gear_changed", old_gear, current_gear)
		if AudioManager:
			AudioManager.play_sound("gear_down")

func _attempt_shift_to_reverse() -> void:
	"""Shift into reverse gear (only when stopped)"""
	if current_speed_kmh < 2 and current_gear != -1:
		var old_gear = current_gear
		current_gear = -1
		target_gear = current_gear
		
		emit_signal("gear_changed", old_gear, current_gear)
		if AudioManager:
			AudioManager.play_sound("reverse")

func _set_neutral_temporarily() -> void:
	"""Set gear to neutral briefly during shift"""
	var old_gear = current_gear
	current_gear = 0
	target_gear = 0
	# Revert after short delay
	await get_tree().create_timer(0.1).timeout
	current_gear = target_gear
	target_gear = target_gear

func _calculate_target_rpm_for_speed(speed_ms: float, gear: int) -> float:
	"""Calculate target RPM based on current speed and gear"""
	if speed_ms <= 0:
		return idle_rpm
	
	var total_ratio = gear_ratios[gear - 1] * final_drive_ratio if gear > 0 else reverse_ratio * final_drive_ratio
	var wheel_rps = speed_ms / (2.0 * PI * wheel_radius)
	var engine_rps = wheel_rps * total_ratio
	return engine_rps * 60.0

func _calculate_rpm_from_speed(speed_ms: float, gear: int) -> float:
	"""Calculate engine RPM based on current speed"""
	if gear == 0 or speed_ms <= 0:
		return idle_rpm
	
	var ratio = gear_ratios[gear - 1] * final_drive_ratio if gear > 0 else reverse_ratio * final_drive_ratio
	var wheel_rps = speed_ms / (2.0 * PI * wheel_radius)
	var engine_rps = wheel_rps * ratio
	return engine_rps * 60.0

# ============================================================================
# PHYSICS CALCULATION
# ============================================================================

func _update_physics(delta: float) -> void:
	"""Update all physics calculations"""
	# Calculate traction forces
	_update_traction(delta)
	
	# Apply aerodynamic forces
	_apply_aerodynamics(delta)
	
	# Calculate braking forces
	_apply_braking(delta)
	
	# Handle clutch engagement effect
	_apply_clutch_effect()
	
	# Update RPM based on gear and speed
	_update_engine_rpm()
	
	# Calculate acceleration
	_calculate_acceleration(delta)
	
	# Apply lateral forces for steering
	_apply_steering(delta)
	
	# Handle drift mechanics
	_apply_drift_mechanics(delta)

func _update_traction(delta: float) -> void:
	"""Calculate and apply traction forces from wheels"""
	if current_gear == 0 or (not clutch_engaged and clutch_position < 0.5):
		return
	
	var drive_direction = Vector3.FORWARD
	if current_gear == -1:
		drive_direction = Vector3.BACKWARD
	
	# Calculate engine torque based on RPM and gear
	var engine_torque = _calculate_engine_torque()
	
	# Apply torque through transmission
	var total_ratio = gear_ratios[current_gear - 1] * final_drive_ratio if current_gear > 0 else reverse_ratio * final_drive_ratio
	var wheel_torque = engine_torque * total_ratio
	
	# Distribute torque between driven wheels (simplified RWD)
	var wheel_force = (wheel_torque * 0.5) / wheel_radius
	wheel_force *= throttle_input
	
	# Apply to chassis via body force
	var force_vector = drive_direction.normalized() * wheel_force
	apply_central_impulse(force_vector * delta)

func _calculate_engine_torque() -> float:
	"""Calculate engine torque based on RPM curve"""
	# Simplified torque curve approximation
	var rpm_normalized = (current_rpm - idle_rpm) / (redline_rpm - idle_rpm)
	rpm_normalized = clamp(rpm_normalized, 0.0, 1.0)
	
	# Peak torque around 4000 RPM
	var peak_rpm = (idle_rpm + redline_rpm) * 0.5
	var torque_curve = sin((rpm_normalized - 0.3) * PI) * 0.5 + 0.5
	
	var max_torque = 500.0 # Nm
	return torque_curve * max_torque

func _apply_aerodynamics(delta: float) -> void:
	"""Apply aerodynamic drag and downforce"""
	var speed = velocity.length()
	if speed <= 0.1:
		return
	
	# Drag force: Fd = 0.5 * rho * v^2 * Cd * A
	var drag_force = 0.5 * _air_density * speed * speed * drag_coefficient * frontal_area
	var drag_vector = -velocity.normalized() * drag_force
	
	# Downforce (increases tire grip at high speeds)
	var downforce = 0.5 * _air_density * speed * speed * downforce_coefficient * frontal_area
	
	# Apply forces to body
	apply_central_force(drag_vector * delta)
	
	# Apply downforce as increased normal force (handled by collision)
	# Modify gravity scale temporarily for downforce effect
	var effective_gravity = PhysicsSettings.gravity + (downforce / _vehicle_mass)
	# This would normally modify the rigid body's gravity scale

func _apply_braking(delta: float) -> void:
	"""Apply braking forces based on pedal input"""
	if brake_input <= 0.0:
		return
	
	# Total brake pressure
	var brake_pressure = brake_input * max_brake_force
	
	# ABS check (simplified)
	var wheel_slip = _calculate_wheel_slip()
	if abs(wheel_slip) > abs_threshold and abs_enabled:
		brake_pressure *= (1.0 - abs_threshold)
	
	# Distribute brake pressure front/rear
	var front_brake = brake_pressure * brake_pressure_distribution.x * brake_bias_front
	var rear_brake = brake_pressure * brake_pressure_distribution.y * (1.0 - brake_bias_front)
	
	# Apply braking impulse opposite to velocity
	if velocity.length() > 0.1:
		var brake_force = -(velocity.normalized() * brake_pressure * delta)
		apply_central_impulse(brake_force)

func _calculate_wheel_slip() -> float:
	"""Calculate average wheel slip ratio"""
	var total_slip = 0.0
	var wheel_count = 0
	
	for wheel_key in _wheel_positions:
		var wheel_node = $"Wheels/{wheel_key.capitalize()}".get_node_or_null("RayCast3D")
		if wheel_node and wheel_node.is_colliding():
			var contact_point = wheel_node.get_collision_point()
			var surface_normal = wheel_node.get_collision_normal()
			# Simplified slip calculation
			total_slip += 0.1
			wheel_count += 1
	
	if wheel_count > 0:
		return total_slip / wheel_count
	return 0.0

func _apply_clutch_effect() -> void:
	"""Apply clutch engagement/disengagement effects"""
	if clutch_position < 0.1:
		# Clutch disengaged - no power transfer
		current_rpm = lerp(current_rpm, idle_rpm + throttle_input * 2000.0, 0.1)
		clutch_engaged = false
	elif clutch_position > 0.9:
		# Clutch fully engaged
		clutch_engaged = true
		# Sync RPM with wheels
		var wheel_rpm = _calculate_rpm_from_speed(velocity.length(), current_gear)
		current_rpm = lerp(current_rpm, wheel_rpm, 0.2)

func _update_engine_rpm() -> void:
	"""Update engine RPM based on driving conditions"""
	if clutch_engaged and current_gear != 0:
		var wheel_speed = velocity.length()
		var target_rpm = _calculate_rpm_from_speed(wheel_speed, current_gear)
		
		# Smooth RPM transition
		if throttle_input > 0.1:
			current_rpm = lerp(current_rpm, min(target_rpm, redline_rpm), 0.1)
		else:
			current_rpm = lerp(current_rpm, idle_rpm, 0.05)
	else:
		# Free revving
		current_rpm = lerp(current_rpm, idle_rpm + throttle_input * 4000.0, 0.1)
	
	# Clamp RPM
	current_rpm = clamp(current_rpm, idle_rpm, redline_rpm)

func _calculate_acceleration(delta: float) -> void:
	"""Calculate vehicle acceleration"""
	var force_sum = Vector3.ZERO
	
	# Add thrust from engine
	if clutch_engaged and throttle_input > 0.01 and current_gear != 0:
		var engine_torque = _calculate_engine_torque()
		var total_ratio = gear_ratios[current_gear - 1] * final_drive_ratio if current_gear > 0 else reverse_ratio * final_drive_ratio
		var wheel_force = (engine_torque * total_ratio * throttle_input) / wheel_radius
		force_sum += Vector3.FORWARD * wheel_force
	
	acceleration = force_sum / _vehicle_mass

func _apply_steering(delta: float) -> void:
	"""Apply steering angle to vehicle rotation"""
	if current_gear == 0 and velocity.length() < 0.5:
		# Cannot steer when stopped (for realism)
		return
	
	# Smooth steering transition
	steering_angle = lerp(steering_angle, current_steering_input * max_steering_angle, delta * steer_recovery_speed)
	
	# Apply yaw rotation based on speed and steering
	var turn_rate = steering_angle * 2.0
	if velocity.length() > 0.5:
		turn_rate *= velocity.length() * 0.3
	
	# Rotate vehicle
	var rotation_delta = -turn_rate * delta
	rotate_y(rotation_delta)

func _apply_drift_mechanics(delta: float) -> void:
	"""Apply drift physics when handbrake is used"""
	if not handbrake_input:
		return
	
	var speed = velocity.length()
	if speed < 10:
		return
	
	# Reduce lateral grip during drift
	var lateral_damping = drift_factor
	
	# Modify velocity to allow sideways sliding
	var forward_vel = velocity.dot(transform.basis.z) * transform.basis.z
	var side_vel = velocity.dot(transform.basis.x) * transform.basis.x
	
	# Dampen forward velocity more than lateral during drift
	forward_vel *= 0.95
	side_vel *= lateral_damping
	
	# Reconstruct velocity vector
	velocity = forward_vel + side_vel

func _apply_forces_to_body() -> void:
	"""Apply calculated forces to the physical body"""
	# Gravity is handled by physics engine automatically
	# Custom forces are applied via apply_central_force/impulse in individual methods
	
	# Apply air resistance
	var air_resistance = -velocity * 0.01 * velocity.length()
	apply_central_force(air_resistance)

func _on_race_started(race_data: Dictionary) -> void:
	"""Handle race start event"""
	pass

func _on_vehicle_destroyed(vehicle: Node) -> void:
	"""Handle vehicle destruction event"""
	if vehicle == self:
		emit_signal("vehicle_destroyed")
		queue_free()

func _on_sound_played(sound_name: String) -> void:
	"""Handle sound playback events"""
	if sound_name == "engine_rev" and AudioManager:
		AudioManager.set_volume("engine", current_rpm / redline_rpm)

func _on_collision_entered(body: Node) -> void:
	"""Handle collision events"""
	if body.has_method("get_impact_force"):
		var impact = body.get_impact_force()
		emit_signal("vehicle_collided", body, impact)
		if AudioManager:
			AudioManager.play_sound("collision")

func _on_collision_exited(body: Node) -> void:
	"""Handle collision exit events"""
	pass

func _input(event: InputEvent) -> void:
	"""Global input handler for direct control"""
	if event is InputEventKey and event.pressed:
		match event.keycode:
			KEY_Z:
				shift_up_request = true
			KEY_X:
				shift_down_request = true
			KEY_C:
				Input.set_action_persistent("game_clutch", event.pressed)

func reset_vehicle() -> void:
	"""Reset vehicle to initial state"""
	current_gear = 0
	target_gear = 0
	current_rpm = idle_rpm
	throttle_input = 0.0
	brake_input = 0.0
	steering_angle = 0.0
	clutch_engaged = true
	clutch_position = 1.0
	
	velocity = Vector3.ZERO
	angular_velocity = Vector3.ZERO
	
	reset()

func _to_string() -> String:
	"""Debug string representation"""
	return "VehicleController[RPM:%.0f|Gear:%d|Speed:%.1fkm/h]" % [current_rpm, current_gear, current_speed_kmh]

</script>
<node name="VehicleController" type="CharacterBody3D">
<property name="position" type="Vector3" value="0,0,0"/>
<property name="rotation" type="Vector3" value="0,0,0"/>
</node>