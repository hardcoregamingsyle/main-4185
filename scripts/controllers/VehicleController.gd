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
signal wheel_slip_changed(front_slip: float, rear_slip: float)
signal engine_rpm_changed(rpm: float)
signal collision_detected(normal: Vector2, impact_force: float)

# ============================================================================
# PHYSICS CONSTANTS - Derived from PhysicsSettings resource
# ============================================================================

const MAX_THROTTLE_FORCE: float = 15000.0      # Newtons - maximum acceleration force
const MAX_BRAKE_FORCE: float = 20000.0         # Newtons - maximum braking force
const MAX_STEERING_ANGLE: float = PI / 3       # 60 degrees max steering
const STEERING_SPEED: float = 4.0              # Radians per second steering rate
const DRIFT_THRESHOLD: float = 0.7             # Sideslip threshold for drift mode
const TRACTION_CONTROL_SENSITIVITY: float = 0.85 # TCS activation threshold
const MIN_GEAR_RPM: float = 800.0              # Idle RPM
const MAX_GEAR_RPM: float = 8000.0             # Redline RPM
const SHIFT_POINT_NORMAL: float = 6500.0       # Normal shift point RPM
const SHIFT_POINT_AGGRESSIVE: float = 7500.0   # Aggressive shift point RPM
const CLUTCH_RELEASE_TIME: float = 0.2         # Seconds for clutch engagement
const BRAKE_BIAS_FRONT: float = 0.6            # 60% brake force to front wheels
const BRAKE_BIAS_REAR: float = 0.4             # 40% brake force to rear wheels

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
@export var wheelbase: float = 2.6           # Distance between front and rear axles
@export var track_width: float = 1.6         # Distance between left and right wheels
@export var wheel_radius: float = 0.32       # Wheel radius in meters
@export var wheel_track_offset: float = 0.8  # Half-track width offset

# Physical properties
@export var drag_coefficient: float = 0.32   # Aerodynamic drag coefficient
@export var frontal_area: float = 2.2        # Frontal area in m²
@export var air_density: float = 1.225       # Air density at sea level kg/m³
@export var rolling_resistance_coefficient: float = 0.015
@export var moment_of_inertia: float = 2500.0 # Rotational inertia kg·m²

# Wheel friction coefficients (dry asphalt, wet asphalt, gravel, etc.)
@export var tire_friction_dry: float = 1.2     # Dry asphalt grip coefficient
@export var tire_friction_wet: float = 0.8     # Wet asphalt grip coefficient
@export var tire_friction_gravel: float = 0.6  # Gravel surface grip coefficient
@export var tire_friction_snow: float = 0.3    # Snow surface grip coefficient

# ============================================================================
# CURRENT VEHICLE STATE
# ============================================================================

var velocity: Vector2 = Vector2.ZERO
var angular_velocity: float = 0.0               # Yaw rotation rate rad/s
var current_position: Vector2 = Vector2.ZERO
var current_rotation: float = 0.0               # In radians
var current_gear: Gear = Gear.NEUTRAL
var current_rpm: float = 0.0                    # Engine RPM
var target_rpm: float = 0.0
var clutch_engaged: bool = false
var clutch_progress: float = 0.0                # 0.0 to 1.0

# Input states
var throttle_input: float = 0.0                 # -1.0 to 1.0
var brake_input: float = 0.0                    # 0.0 to 1.0
var steering_input: float = 0.0                 # -1.0 to 1.0
var clutch_input: float = 0.0                   # 0.0 to 1.0

# Drift and traction control
var is_drifting: bool = false
var drift_angle: float = 0.0                    # Difference between heading and velocity direction
var traction_control_enabled: bool = true
var traction_control_active: bool = false
var slip_ratio_front: float = 0.0
var slip_ratio_rear: float = 0.0

# Wheel forces
var wheel_forces: Array[Vector2] = []
var wheel_torque: Vector2 = Vector2.ZERO
var lateral_force_front: Vector2 = Vector2.ZERO
var lateral_force_rear: Vector2 = Vector2.ZERO

# Performance metrics
var distance_traveled: float = 0.0
var lap_time: float = 0.0
var max_speed_recorded: float = 0.0
var average_speed: float = 0.0

# ============================================================================
# SURFACE AND ENVIRONMENT
# ============================================================================

var current_surface_type: String = "asphalt"
var surface_friction_modifier: float = 1.0
var road_bank_angle: float = 0.0                # Banking angle in radians
var wind_force: Vector2 = Vector2.ZERO
var environmental_resistance: float = 0.0

# ============================================================================
# PARTICLES AND VFX
# ============================================================================

var particle_emitters: Array[Node2D] = []
var particle_spawn_rate: float = 30.0           # Particles per second when drifting
var smoke_particles: Array[Dictionary] = []
var skid_marks: Array[Dictionary] = []
var screen_shake_intensity: float = 0.0
var hit_flash_timer: float = 0.0

# ============================================================================
# AUDIO SOURCES
# ============================================================================

var engine_sound_node: AudioStreamPlayer = null
var exhaust_sound_node: AudioStreamPlayer = null
var tire_sound_node: AudioStreamPlayer = null
var horn_sound_node: AudioStreamPlayer = null
var engine_idle_pitch: float = 1.0
var engine_revving_pitch: float = 2.0
var engine_max_pitch: float = 3.5

# ============================================================================
# INTERNAL TIMERS AND BUFFERS
# ============================================================================

var _update_timer: float = 0.0
var _physics_accumulator: float = 0.0
var _gear_shift_timer: float = 0.0
var _drift_timer: float = 0.0
var _input_buffer: Dictionary = {}
var _last_frame_velocity: Vector2 = Vector2.ZERO
var _collision_history: Array[Dictionary] = []

# ============================================================================
# READY - Initialize vehicle controller
# ============================================================================

func _ready() -> void:
	_process_mode = ProcessModeEnum.ALWAYS
	
	# Apply physics settings defaults if not set
	if vehicle_mass <= 0:
		vehicle_mass = PhysicsSettings.default_vehicle_mass
		
	_init_audio_sources()
	_reset_vehicle_state()
	_connect_signals_to_game_manager()
	
	print("VehicleController initialized for ", get_path())

# ============================================================================
# INPUT HANDLING
# ============================================================================

func _process(delta: float) -> void:
	_update_timer += delta
	
	# Update input buffer
	_update_input_buffer()
	
	# Handle gear shifting based on input
	_handle_gear_shifting()
	
	# Update clutch engagement
	_update_clutch(delta)
	
	# Update audio pitch based on RPM
	_update_audio_pitches()
	
	# Update VFX timers
	_update_vfx_timers(delta)

func _physics_process(delta: float) -> void:
	_physics_accumulator += delta
	
	# Fixed timestep physics updates
	while _physics_accumulator >= PhysicsSettings.physics_tick_rate:
		_physics_step(delta * PhysicsSettings.time_scale)
		_physics_accumulator -= 1.0 / PhysicsSettings.physics_tick_rate
	
	# Update particles and effects
	_update_particles(delta)
	_update_skid_marks(delta)

# ============================================================================
# INPUT BUFFER MANAGEMENT
# ============================================================================

func _update_input_buffer() -> void:
	# Capture latest input values
	_input_buffer.throttle = Input.get_axis("throttle_up", "throttle_down")
	_input_buffer.brake = Input.get_axis("brake_up", "brake_down")
	_input_buffer.steer_left = Input.get_axis("steer_left", "steer_right")
	_input_buffer.clutch = Input.get_axis("clutch_up", "clutch_down")
	_input_buffer.shift_up = Input.is_action_just_pressed("shift_up")
	_input_buffer.shift_down = Input.is_action_just_pressed("shift_down")
	_input_buffer.drift = Input.is_action_pressed("drift")
	_input_buffer.horn = Input.is_action_just_pressed("horn")
	
	# Smooth input interpolation
	throttle_input = lerp(throttle_input, clamp(_input_buffer.throttle, -1.0, 1.0), 0.1)
	brake_input = lerp(brake_input, clamp(_input_buffer.brake, 0.0, 1.0), 0.15)
	steering_input = lerp(steering_input, clamp(_input_buffer.steer_left, -1.0, 1.0), 0.2)
	clutch_input = lerp(clutch_input, clamp(_input_buffer.clutch, 0.0, 1.0), 0.1)

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================

func _handle_gear_shifting() -> void:
	# Manual gear shifting
	if _input_buffer.shift_up:
		_shift_gear(1)
	elif _input_buffer.shift_down:
		_shift_gear(-1)
	
	# Auto-shift logic (if enabled)
	if _should_auto_shift():
		var target_gear = _calculate_optimal_gear()
		if target_gear != current_gear and current_gear != Gear.NEUTRAL:
			_shift_gear(target_gear - current_gear)

func _shift_gear(direction: int) -> void:
	if direction == 0:
		return
	
	var old_gear = current_gear
	var new_gear_index = current_gear + direction
	
	# Validate gear range
	if new_gear_index < -1 or new_gear_index > 6:
		return
	
	# Map index to enum value
	var new_gear: Gear = Gear.FIRST
	match new_gear_index:
		-1: new_gear = Gear.REVERSE
		0: new_gear = Gear.FIRST
		1: new_gear = Gear.SECOND
		2: new_gear = Gear.THIRD
		3: new_gear = Gear.FOURTH
		4: new_gear = Gear.FIFTH
		5: new_gear = Gear.SIXTH
	
	# Prevent reverse while moving forward
	if new_gear == Gear.REVERSE and velocity.length() > 1.0:
		return
	
	if new_gear != old_gear:
		current_gear = new_gear
		gear_changed.emit(old_gear, new_gear)
		
		# Trigger clutch during shift
		clutch_engaged = false
		clutch_progress = 0.0
		_engine_cut_off()
		
		# Re-engage after short delay
		await get_tree().create_timer(CLUTCH_RELEASE_TIME).timeout
		if current_gear != Gear.NEUTRAL:
			clutch_engaged = true

func _should_auto_shift() -> bool:
	return false  # Settable via export for auto/manual transmission

func _calculate_optimal_gear() -> Gear:
	# Calculate optimal gear based on current RPM and speed
	var wheel_rpm = _calculate_wheel_rpm()
	var engine_rpm = wheel_rpm * _get_total_gear_ratio()
	
	if engine_rpm < MIN_GEAR_RPM:
		return Gear.FIRST
	elif engine_rpm > MAX_GEAR_RPM:
		return Gear.SIXTH
	else:
		# Find best gear for efficiency
		for gear in [Gear.SIXTH, Gear.FIFTH, Gear.FOURTH, Gear.THIRD, Gear.SECOND, Gear.FIRST]:
			var gear_rpm = wheel_rpm * _get_total_gear_ratio_for(gear)
			if gear_rpm >= MIN_GEAR_RPM and gear_rpm <= SHIFT_POINT_NORMAL:
				return gear
		return Gear.FIRST

func _get_total_gear_ratio_for(gear: Gear) -> float:
	var gear_ratio = GEAR_RATIOS.get(gear, 1.0)
	return gear_ratio * FINAL_DRIVE_RATIO

func _get_total_gear_ratio() -> float:
	return _get_total_gear_ratio_for(current_gear)

# ============================================================================
# CLUTCH CONTROL
# ============================================================================

func _update_clutch(delta: float) -> void:
	if clutch_engaged:
		# Engage clutch smoothly
		clutch_progress = min(clutch_progress + delta / CLUTCH_RELEASE_TIME, 1.0)
		if clutch_progress >= 1.0:
			_engine_cut_on()
	else:
		# Disengage clutch
		clutch_progress = max(clutch_progress - delta / CLUTCH_RELEASE_TIME, 0.0)
		if clutch_progress <= 0.0:
			_engine_cut_off()

func _engine_cut_on() -> void:
	pass  # Signal to powertrain to engage engine

func _engine_cut_off() -> void:
	pass  # Signal to powertrain to cut engine

# ============================================================================
# PHYSICS STEP - Main vehicle dynamics simulation
# ============================================================================

func _physics_step(delta: float) -> void:
	# Calculate forces acting on vehicle
	var driving_force: float = _calculate_driving_force()
	var braking_force: float = _calculate_braking_force()
	var aerodynamic_drag: float = _calculate_aerodynamic_drag()
	var rolling_resistance: float = _calculate_rolling_resistance()
	var gravitational_force: float = _calculate_gravitational_force()
	var lateral_forces: Vector2 = _calculate_lateral_forces(driving_force, braking_force)
	
	# Combine forces
	var total_longitudinal_force = driving_force - braking_force - aerodynamic_drag - rolling_resistance
	var total_force = Vector2(total_longitudinal_force * Vector2(cos(current_rotation), sin(current_rotation)) + lateral_forces)
	
	# Apply gravity component based on terrain slope
	total_force.y += gravitational_force * sin(road_bank_angle)
	
	# Calculate acceleration (F = ma)
	var acceleration = total_force / vehicle_mass
	
	# Update velocity
	velocity += acceleration * delta
	
	# Apply air resistance to velocity damping
	velocity *= (1.0 - air_density * frontal_area * drag_coefficient * delta * 0.01)
	
	# Clamp velocity to realistic limits
	var max_speed = _calculate_max_speed()
	if velocity.length() > max_speed:
		velocity = velocity.normalized() * max_speed
	
	# Update position
	var displacement = velocity * delta
	current_position += displacement
	position = current_position
	
	# Update rotation (steering affects yaw)
	var steering_effectiveness = _get_steering_effectiveness()
	var yaw_rate = steering_input * STEERING_SPEED * steering_effectiveness * (velocity.length() / 5.0)
	current_rotation += yaw_rate * delta
	rotation = current_rotation
	
	# Update angular velocity
	angular_velocity = yaw_rate
	
	# Update RPM
	_update_engine_rpm(delta)
	
	# Check for drift
	_check_drift_state()
	
	# Update performance metrics
	distance_traveled += velocity.length() * delta
	max_speed_recorded = max(max_speed_recorded, velocity.length())
	
	# Emit signals
	speed_changed.emit(velocity.length())
	wheel_slip_changed.emit(slip_ratio_front, slip_ratio_rear)
	engine_rpm_changed.emit(current_rpm)
	
	# Screen shake decay
	screen_shake_intensity = max(screen_shake_intensity - delta * 5.0, 0.0)

# ============================================================================
# FORCE CALCULATIONS
# ============================================================================

func _calculate_driving_force() -> float:
	if current_gear == Gear.NEUTRAL or current_gear == Gear.REVERSE:
		return 0.0
	
	if not clutch_engaged or clutch_progress < 0.1:
		return 0.0
	
	var gear_ratio = GEAR_RATIOS[current_gear]
	var total_ratio = gear_ratio * FINAL_DRIVE_RATIO
	
	# Torque curve approximation (peak torque at mid RPM)
	var torque_factor = _get_torque_curve_factor()
	var engine_torque = 300.0 * torque_factor  # Peak torque in Nm
	
	# Calculate wheel torque
	var wheel_torque_value = engine_torque * total_ratio * 0.85  # 85% drivetrain efficiency
	
	# Convert to force at contact patch
	var driving_force = abs(wheel_torque_value) / wheel_radius
	
	# Apply throttle input
	driving_force *= (throttle_input + 1.0) / 2.0
	
	# Adjust for reverse gear
	if current_gear == Gear.REVERSE:
		driving_force = -driving_force
	
	return driving_force

func _calculate_braking_force() -> float:
	if brake_input <= 0.0:
		return 0.0
	
	# Maximum braking force distributed front/rear
	var front_brake_force = MAX_BRAKE_FORCE * BRAKE_BIAS_FRONT * brake_input
	var rear_brake_force = MAX_BRAKE_FORCE * BRAKE_BIAS_REAR * brake_input
	
	return front_brake_force + rear_brake_force

func _calculate_aerodynamic_drag() -> float:
	var velocity_squared = velocity.length_squared()
	var drag_force = 0.5 * air_density * frontal_area * drag_coefficient * velocity_squared
	return drag_force

func _calculate_rolling_resistance() -> float:
	var normal_force = vehicle_mass * PhysicsSettings.gravity
	var rolling_resistance = normal_force * rolling_resistance_coefficient * surface_friction_modifier
	return rolling_resistance

func _calculate_gravitational_force() -> float:
	var normal_force = vehicle_mass * PhysicsSettings.gravity
	return normal_force * sin(road_bank_angle)

func _calculate_lateral_forces(driving_force: float, braking_force: float) -> Vector2:
	# Simplified lateral force calculation based on steering and speed
	var steer_angle = steering_input * MAX_STEERING_ANGLE
	var slip_angle = atan2(velocity.x * sin(current_rotation) - velocity.y * cos(current_rotation), 
	                       velocity.x * cos(current_rotation) + velocity.y * sin(current_rotation))
	
	var cornering_stiffness = 15000.0  # N/rad
	var lateral_force = -cornering_stiffness * slip_angle
	
	# Reduce lateral grip under high load (combined slip)
	var combined_load = (abs(driving_force) + abs(braking_force)) / MAX_BRAKE_FORCE
	var grip_reduction = 1.0 - combined_load * 0.3
	
	lateral_force *= grip_reduction
	
	# Apply tire friction modifier
	lateral_force *= surface_friction_modifier
	
	# Distribute forces front/rear
	var front_lateral = lateral_force * 0.55
	var rear_lateral = lateral_force * 0.45
	
	return Vector2(front_lateral, rear_lateral) * cos(current_rotation)

# ============================================================================
# ENGINE RPM UPDATE
# ============================================================================

func _update_engine_rpm(delta: float) -> void:
	if current_gear == Gear.NEUTRAL:
		target_rpm = MIN_GEAR_RPM
	else:
		var wheel_rpm = _calculate_wheel_rpm()
		var engine_rpm_from_wheel = wheel_rpm * _get_total_gear_ratio()
		target_rpm = engine_rpm_from_wheel
	
	# Smooth RPM transition (inertia effect)
	var rpm_acceleration_rate = 15000.0  # RPM per second
	var rpm_difference = target_rpm - current_rpm
	
	if abs(rpm_difference) > 500:
		current_rpm += sign(rpm_difference) * rpm_acceleration_rate * delta
	else:
		current_rpm = target_rpm
	
	# Clamp RPM
	current_rpm = clamp(current_rpm, 0.0, MAX_GEAR_RPM * 1.1)
	
	# Redline protection
	if current_rpm > MAX_GEAR_RPM:
		current_rpm = MAX_GEAR_RPM * 0.95
		_engine_cut_off()

func _calculate_wheel_rpm() -> float:
	if velocity.length() <= 0:
		return 0.0
	var wheel_linear_speed = velocity.length()
	var wheel_circumference = 2.0 * PI * wheel_radius
	var wheel_rps = wheel_linear_speed / wheel_circumference
	return wheel_rps * 60.0  # RPM

# ============================================================================
# DRIFT DETECTION
# ============================================================================

func _check_drift_state() -> void:
	# Calculate sideslip angle
	if velocity.length() < 0.5:
		is_drifting = false
		return
	
	var velocity_direction = velocity.angle()
	var heading_direction = current_rotation
	var sideslip = abs(heading_direction - velocity_direction)
	
	# Normalize angle difference
	sideslip = fmod(sideslip + PI, 2.0 * PI)
	if sideslip > PI:
		sideslip = 2.0 * PI - sideslip
	
	# Check if drifting based on sideslip threshold
	is_drifting = sideslip > DRIFT_THRESHOLD
	
	if is_drifting:
		_drift_timer += 1.0 / PhysicsSettings.physics_tick_rate
		if _drift_timer > 0.5:  # At least 0.5 seconds of drift
			spawn_smoke_particles()
			create_skid_mark()
			traction_control_active = true
			traction_control_active.emit(true)
	else:
		_drift_timer = 0.0
		traction_control_active = false
		traction_control_active.emit(false)
	
	# Update drift angle signal
	if abs(drift_angle - sideslip) > 0.01:
		drift_angle = sideslip
		drift_angle_changed.emit(sideslip)

# ============================================================================
# PARTICLE SYSTEM
# ============================================================================

func spawn_smoke_particles() -> void:
	if not is_drifting or not clutch_engaged:
		return
	
	# Spawn particles at rear wheels
	var rear_wheel_pos = _get_rear_wheel_position()
	
	for i in range(int(particle_spawn_rate / 30)):
		var particle = _create_particle(rear_wheel_pos)
		smoke_particles.append(particle)
		
		# Limit total particles
		if smoke_particles.size() > 100:
			smoke_particles.pop_front()

func _create_particle(position: Vector2) -> Dictionary:
	return {
		"position": position,
		"velocity": Vector2(randf_range(-2.0, 2.0), randf_range(-1.0, 0.0)),
		"life": 1.0,
		"max_life": randf_range(0.5, 1.5),
		"size": randf_range(0.1, 0.3),
		"color": Color(0.3, 0.3, 0.3, 1.0)
	}

func _update_particles(delta: float) -> void:
	for i in range(smoke_particles.size() - 1, -1, -1):
		var particle = smoke_particles[i]
		particle.life -= delta
		particle.position += particle.velocity * delta
		particle.size *= 1.0 + delta * 0.5  # Expand over time
		
		if particle.life <= 0:
			smoke_particles.remove_at(i)

func create_skid_mark() -> void:
	if not is_drifting or velocity.length() < 2.0:
		return
	
	var rear_wheel_pos = _get_rear_wheel_position()
	
	skid_marks.append({
		"position": rear_wheel_pos,
		"width": wheel_track_offset * 2,
		"alpha": 0.6,
		"fading": false
	})
	
	# Limit skid marks
	if skid_marks.size() > 200:
		skid_marks.pop_front()

func _update_skid_marks(delta: float) -> void:
	for mark in skid_marks:
		mark.alpha -= delta * 0.1
		if mark.alpha <= 0:
			mark.alpha = 0

func _get_rear_wheel_position() -> Vector2:
	var rear_x = -wheelbase / 2.0
	var rear_y = 0.0
	var rotated_x = rear_x * cos(current_rotation) - rear_y * sin(current_rotation)
	var rotated_y = rear_x * sin(current_rotation) + rear_y * cos(current_rotation)
	return current_position + Vector2(rotated_x, rotated_y)

# ============================================================================
# SCREEN SHAKE AND HIT FLASH
# ============================================================================

func apply_screen_shake(intensity: float) -> void:
	screen_shake_intensity = max(screen_shake_intensity, intensity)

func trigger_hit_flash() -> void:
	hit_flash_timer = 0.15

func _update_vfx_timers(delta: float) -> void:
	if hit_flash_timer > 0:
		hit_flash_timer -= delta

# ============================================================================
# AUDIO PITCH UPDATE
# ============================================================================

func _update_audio_pitches() -> void:
	# Map RPM to pitch
	var pitch_ratio = (current_rpm - MIN_GEAR_RPM) / (MAX_GEAR_RPM - MIN_GEAR_RPM)
	var target_pitch = lerp(engine_idle_pitch, engine_max_pitch, pitch_ratio)
	
	# Smooth pitch transition
	if engine_idle_pitch != target_pitch:
		engine_idle_pitch = lerp(engine_idle_pitch, target_pitch, 0.1)

# ============================================================================
# COLLISION DETECTION
# ============================================================================

func check_collision(other: Node2D) -> bool:
	var other_rect = Rect2(other.position - other.size / 2, other.size)
	var vehicle_rect = Rect2(position - Vector2(1.0, 0.5), Vector2(2.0, 1.0))
	
	return vehicle_rect.intersects(other_rect)

func handle_collision(normal: Vector2, impact_force: float) -> void:
	# Apply screen shake
	var shake_intensity = min(impact_force / 10000.0, 2.0)
	apply_screen_shake(shake_intensity)
	
	# Apply bounce
	var bounce_factor = 0.3
	velocity = velocity.bounce(normal) * bounce_factor
	
	# Emit collision signal
	collision_detected.emit(normal, impact_force)
	
	# Reduce speed
	velocity *= 0.7

# ============================================================================
# UTILITIES AND HELPERS
# ============================================================================

func _init_audio_sources() -> void:
	# Create audio stream players dynamically
	engine_sound_node = AudioStreamPlayer.new()
	add_child(engine_sound_node)
	
	exhaust_sound_node = AudioStreamPlayer.new()
	add_child(exhaust_sound_node)
	
	tire_sound_node = AudioStreamPlayer.new()
	add_child(tire_sound_node)
	
	horn_sound_node = AudioStreamPlayer.new()
	add_child(horn_sound_node)

func _connect_signals_to_game_manager() -> void:
	GameManager.game_state_changed.connect(_on_game_state_changed)

func _on_game_state_changed(new_state: GameManager.GameState) -> void:
	if new_state == GameManager.GameState.RACE_PAUSED:
		pause_simulation()
	elif new_state == GameManager.GameState.RACE_ACTIVE:
		resume_simulation()

func pause_simulation() -> void:
	_process_mode = ProcessModeEnum.DISABLED

func resume_simulation() -> void:
	_process_mode = ProcessModeEnum.ALWAYS

func _reset_vehicle_state() -> void:
	velocity = Vector2.ZERO
	angular_velocity = 0.0
	current_position = Vector2.ZERO
	current_rotation = 0.0
	current_gear = Gear.NEUTRAL
	current_rpm = MIN_GEAR_RPM
	target_rpm = MIN_GEAR_RPM
	clutch_engaged = false
	clutch_progress = 0.0
	throttle_input = 0.0
	brake_input = 0.0
	steering_input = 0.0
	clutch_input = 0.0
	is_drifting = false
	traction_control_enabled = true
	distance_traveled = 0.0
	max_speed_recorded = 0.0
	screen_shake_intensity = 0.0
	hit_flash_timer = 0.0
	smoke_particles.clear()
	skid_marks.clear()

func _get_torque_curve_factor() -> float:
	# Simple torque curve: peak at 4000 RPM, drops off at extremes
	var normalized_rpm = (current_rpm - MIN_GEAR_RPM) / (MAX_GEAR_RPM - MIN_GEAR_RPM)
	var torque_peak = 0.4
	var torque_factor = 1.0 - abs(normalized_rpm - torque_peak) * 1.5
	return clamp(torque_factor, 0.2, 1.0)

func _get_steering_effectiveness() -> float:
	# Steering is less effective at very low speeds
	var speed_factor = min(velocity.length() / 10.0, 1.0)
	return speed_factor * (1.0 - brake_input * 0.3)

func _calculate_max_speed() -> float:
	# Max speed depends on gear and engine characteristics
	var gear_ratio = GEAR_RATIOS[current_gear]
	var total_ratio = gear_ratio * FINAL_DRIVE_RATIO
	var max_engine_rps = MAX_GEAR_RPM / 60.0
	var max_wheel_rps = max_engine_rps / total_ratio
	var max_linear_speed = max_wheel_rps * 2.0 * PI * wheel_radius
	
	# Reduce speed in lower gears
	if current_gear == Gear.FIRST:
		max_linear_speed *= 0.4
	elif current_gear == Gear.SECOND:
		max_linear_speed *= 0.7
	
	return max_linear_speed

func get_current_speed() -> float:
	return velocity.length()

func get_current_rpm() -> float:
	return current_rpm

func get_current_gear() -> Gear:
	return current_gear

func get_position() -> Vector2:
	return current_position

func get_rotation() -> float:
	return current_rotation

func reset() -> void:
	_reset_vehicle_state()

func set_surface_type(surface: String) -> void:
	current_surface_type = surface
	match surface:
		"wet_asphalt":
			surface_friction_modifier = tire_friction_wet / tire_friction_dry
		"gravel":
			surface_friction_modifier = tire_friction_gravel / tire_friction_dry
		"snow":
			surface_friction_modifier = tire_friction_snow / tire_friction_dry
		_:
			surface_friction_modifier = 1.0

func activate_horn() -> void:
	if horn_sound_node:
		horn_sound_node.play()

func apply_impulse(force: Vector2, impulse_point: Vector2) -> void:
	# Apply impulse at specific point (e.g., collision)
	var torque_impulse = impulse_point.cross(force)
	angular_velocity += torque_impulse / moment_of_inertia
	velocity += force / vehicle_mass

func _exit_tree() -> void:
	# Cleanup
	for particle in smoke_particles:
		pass
	for mark in skid_marks:
		pass
	print("VehicleController destroyed for ", get_path())

</script>