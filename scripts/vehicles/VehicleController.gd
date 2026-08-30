extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulation
## Handles throttle, brake, steering, wheel forces, gear shifting, and vehicle dynamics
## Uses PhysicsSettings constants for consistent tuning across all vehicles
## Copyright 2026 Thalamus Racing Simulator Project

# --- SIGNALS ---
signal speed_changed(new_speed: float)
signal rpm_changed(new_rpm: int)
signal gear_changed(old_gear: int, new_gear: int)
signal drifting(is_drifting: bool)
signal collision_impact(impact_force: Vector3)
signal lap_completed(lap_time: float)
signal race_checkpoint(passed: bool, checkpoint_id: int)
signal engine_event(event_type: EngineEventType)

enum EngineEventType {
	STARTED,
	SHUTDOWN,
	REV_LIMITE,
	GEAR_UP,
	GEAR_DOWN,
}

# --- INPUT BINDINGS (from InputManager autoload) ---
var _throttle: float = 0.0
var _brake: float = 0.0
var _steering: float = 0.0
var _clutch: float = 0.0
var _handbrake: float = 0.0

# --- PHYSICS STATE ---
@export var max_speed: float = 200.0  # km/h
@export var acceleration_rate: float = 8.0  # m/s²
@export var braking_rate: float = 10.0  # m/s²
@export var steer_sensitivity: float = 1.5
@export var mass: float = 1500.0

var current_speed: float = 0.0  # m/s
var current_rpm: int = 0
var current_gear: int = 0
var target_gear: int = 0
var is_shifting: bool = false
var shift_progress: float = 0.0

# --- DRIFT MECHANICS ---
var slip_angle: float = 0.0
var lateral_g: float = 0.0
var is_drifting: bool = false
var drift_score: float = 0.0
var drift_multiplier: float = 1.0

# --- POWERTRAIN INTEGRATION ---
var powertrain: Node = null
var engine_torque: float = 0.0
var engine_power: float = 0.0
var transmission_ratio: float = 1.0

# --- WHEEL CONFIGURATION ---
const FRONT_WHEEL_OFFSET: float = 1.2
const REAR_WHEEL_OFFSET: float = 1.2
const WHEEL_BASE: float = 2.5
const TRACK_WIDTH: float = 1.5

# --- RACE DATA ---
var current_lap_time: float = 0.0
var total_race_time: float = 0.0
var lap_count: int = 0
var best_lap_time: float = 0.0
var checkpoints_passed: Array[int] = []

# --- REFERENCE TO GAME MANAGER ---
var game_manager: GameManager = GameManager

# --- READY ---
func _ready() -> void:
	process_mode = ProcessModeEnum.PROCESS_PHYSICS
	
	# Connect to global audio manager
	if AudioManager:
		AudioManager.sound_played.connect(_on_audio_played)
	
	# Initialize powertrain reference
	powertrain = get_node_or_null("../Powertrain")
	if powertrain == null:
		print("Warning: VehicleController could not find Powertrain node")
	
	# Set up initial state
	_init_vehicle_state()
	_setup_collision_detection()

# --- INITIALIZATION ---
func _init_vehicle_state() -> void:
	current_gear = 0  # Neutral
	target_gear = 0
	is_shifting = false
	shift_progress = 0.0
	current_speed = 0.0
	current_rpm = 0
	engine_torque = 0.0
	engine_power = 0.0
	transmission_ratio = 1.0
	
	slip_angle = 0.0
	lateral_g = 0.0
	is_drifting = false
	drift_score = 0.0
	drift_multiplier = 1.0
	
	current_lap_time = 0.0
	total_race_time = 0.0
	lap_count = 0
	best_lap_time = 999.0
	checkpoints_passed = []
	
	if powertrain:
		powertrain.engine_started.emit()
		engine_event.emit(EngineEventType.STARTED)

# --- COLLISION SETUP ---
func _setup_collision_detection() -> void:
	var collision_shape = CollisionShape3D.new()
	collision_shape.shape = BoxShape3D.new()
	collision_shape.shape.size = Vector3(2.0, 1.0, 4.5)
	add_child(collision_shape)
	
	var area_3d = Area3D.new()
	area_3d.collision_layer = 2  # Vehicle layer
	area_3d.collision_mask = 1    # Ground/Track layer
	area_3d.body_entered.connect(_on_body_entered)
	add_child(area_3d)

# --- PHYSICS UPDATE (FIXED TIME STEP) ---
func _physics_process(delta: float) -> void:
	if game_manager.current_state != GameManager.GameState.RACE_ACTIVE:
		return
	
	# Handle gear shifting
	_handle_shifting(delta)
	
	# Process inputs
	_process_inputs(delta)
	
	# Calculate RPM based on gear and speed
	_calculate_rpm()
	
	# Apply forces
	_apply_vehicle_forces(delta)
	
	# Update drift mechanics
	_update_drift_mechanics(delta)
	
	# Update lap timing
	_update_timing(delta)
	
	# Move vehicle
	move_and_slide()
	
	# Emit signals
	_emit_physics_signals()

# --- INPUT PROCESSING ---
func _process_inputs(delta: float) -> void:
	# Get input values from InputManager singleton
	_throttle = InputManager.get_axis("throttle", 0.0)
	_brake = InputManager.get_axis("brake", 0.0)
	_steering = InputManager.get_axis("steering", 0.0)
	_clutch = InputManager.get_axis("clutch", 0.0)
	_handbrake = InputManager.get_axis("handbrake", 0.0)
	
	# Clamp inputs
	_throttle = clamp(_throttle, 0.0, 1.0)
	_brake = clamp(_brake, 0.0, 1.0)
	_steering = clamp(_steering, -1.0, 1.0)
	_clutch = clamp(_clutch, 0.0, 1.0)
	_handbrake = clamp(_handbrake, 0.0, 1.0)
	
	# Auto-shift if no clutch pressed
	if _clutch < 0.1 and !is_shifting:
		_auto_shift()

# --- GEAR SHIFTING LOGIC ---
func _handle_shifting(delta: float) -> void:
	if is_shifting:
		shift_progress += delta * 2.0  # Shift takes ~0.5 seconds
		
		if shift_progress >= 1.0:
			_complete_shift()
	else:
		shift_progress = 0.0

func _auto_shift() -> void:
	var shift_up_threshold: int = 7500
	var shift_down_threshold: int = 2000
	
	if current_rpm > shift_up_threshold and current_gear < 6:
		_request_gear_change(current_gear + 1)
	elif current_rpm < shift_down_threshold and current_gear > 1:
		_request_gear_change(current_gear - 1)

func _request_gear_change(target: int) -> void:
	if target < 0 or target > 6:
		return
	
	if current_gear != target:
		target_gear = target
		is_shifting = true
		shift_progress = 0.0
		engine_event.emit(EngineEventType.GEAR_UP if target > current_gear else EngineEventType.GEAR_DOWN)

func _complete_shift() -> void:
	current_gear = target_gear
	is_shifting = false
	shift_progress = 0.0
	gear_changed.emit(current_gear - 1, target_gear)
	
	# Rev match based on speed and next gear
	_rev_match()

func _rev_match() -> void:
	if current_gear > 1 and current_speed > 0:
		var gear_ratio = _get_gear_ratio(current_gear)
		var final_drive = 3.5
		var tire_radius = 0.32
		
		var theoretical_rpm = (current_speed * 60.0 / (tire_radius * 2.0 * PI)) * gear_ratio * final_drive
		current_rpm = lerp(current_rpm, theoretical_rpm.floor(), 0.5 * _get_physics_settings().physics_tick_rate)

# --- RPM CALCULATION ---
func _calculate_rpm() -> void:
	if current_gear == 0:
		# Neutral - idle RPM
		current_rpm = lerp(current_rpm, 800, 0.1)
		return
	
	var gear_ratio = _get_gear_ratio(current_gear)
	var final_drive = 3.5
	var tire_radius = 0.32
	
	# Calculate theoretical RPM based on speed
	var theoretical_rpm = (current_speed * 60.0 / (tire_radius * 2.0 * PI)) * gear_ratio * final_drive
	
	# Apply throttle influence on RPM
	if _throttle > 0:
		var throttle_factor = 0.8 + (_throttle * 0.2)
		theoretical_rpm *= throttle_factor
	
	# Clamp RPM range
	current_rpm = clamp(theoretical_rpm.floor(), 800, 9000)
	
	# Check rev limiter
	if current_rpm >= 8800:
		engine_event.emit(EngineEventType.REV_LIMITER)

# --- GEAR RATIOS (typical 6-speed manual) ---
func _get_gear_ratio(gear: int) -> float:
	match gear:
		1: return 3.5
		2: return 2.2
		3: return 1.6
		4: return 1.2
		5: return 0.95
		6: return 0.75
		_: return 1.0

# --- VEHICLE FORCES APPLICATION ---
func _apply_vehicle_forces(delta: float) -> void:
	var forward_dir = transform.basis.z
	var right_dir = transform.basis.x
	
	# Calculate drive force based on gear and throttle
	var drive_force = _calculate_drive_force()
	
	# Apply braking
	var brake_force = _calculate_brake_force()
	
	# Apply steering
	_apply_steering(right_dir)
	
	# Apply forces to body
	var total_force = drive_force + brake_force
	
	# Add drag and rolling resistance
	var air_drag = _calculate_air_drag()
	var rolling_resistance = _calculate_rolling_resistance()
	
	total_force -= air_drag
	total_force -= rolling_resistance
	
	# Apply force to center of mass
	force_application(total_force, Vector3.ZERO)

# --- DRIVE FORCE CALCULATION ---
func _calculate_drive_force() -> float:
	if current_gear == 0:
		return Vector3.ZERO
	
	# Get torque from powertrain
	var torque = 0.0
	if powertrain:
		torque = powertrain.get_engine_torque()
	else:
		# Default torque curve
		torque = _default_torque_curve(current_rpm)
	
	# Apply throttle multiplier
	torque *= _throttle
	
	# Convert torque to wheel force
	var gear_ratio = _get_gear_ratio(current_gear)
	var final_drive = 3.5
	var tire_radius = 0.32
	
	var wheel_torque = torque * gear_ratio * final_drive
	var drive_force = wheel_torque / tire_radius
	
	# Limit maximum drive force
	drive_force = min(drive_force, max_speed * mass * 0.1)
	
	return drive_force * Vector3.FORWARD

# --- BRAKE FORCE CALCULATION ---
func _calculate_brake_force() -> float:
	var brake_force = _brake * mass * braking_rate
	return -brake_force * Vector3.FORWARD

# --- STEERING MECHANICS ---
func _apply_steering(right_dir: Vector3) -> void:
	var steer_angle = _steering * steer_sensitivity * deg_to_rad(30.0)
	
	# Apply rotation to vehicle
	var rotation_amount = steer_angle * _throttle  # No steering when stopped
	
	# Smooth steering transition
	var current_rotation = rotation.y
	var target_rotation = current_rotation + rotation_amount
	
	rotation.y = lerp(current_rotation, target_rotation, 0.1)
	
	# Slip angle calculation for drift mechanics
	var velocity_forward = velocity.normalized() if velocity.length() > 0.1 else Vector3.FORWARD
	var car_forward = -transform.basis.z
	
	slip_angle = velocity_forward.angle_to(car_forward)

# --- DRIFT MECHANICS ---
func _update_drift_mechanics(delta: float) -> void:
	# Detect drift conditions
	var is_oversteer = abs(slip_angle) > deg_to_rad(10.0)
	var handbrake_active = _handbrake > 0.5
	var cornering = abs(_steering) > 0.3
	var speed_above_threshold = current_speed > 15.0
	
	is_drifting = is_oversteer and (handbrake_active or cornering) and speed_above_threshold
	
	# Calculate lateral G-force
	lateral_g = sin(slip_angle) * current_speed * 0.1
	
	# Update drift score
	if is_drifting:
		drift_score += delta * 10.0
		drift_multiplier = 1.0 + min(drift_score * 0.1, 0.5)
		
		# Reduce grip during drift
		steer_sensitivity *= 0.7
	else:
		drift_score = max(0.0, drift_score - delta * 5.0)
		drift_multiplier = 1.0
		steer_sensitivity = clamp(steer_sensitivity + delta * 2.0, 0.5, 2.0)
	
	# Emit drift signal
	if drifting.is_connected(_on_drifting_changed):
		pass
	else:
		pass

func _on_drifting_changed(is_drifting_value: bool) -> void:
	drifting.emit(is_drifting_value)

# --- LAP TIMING ---
func _update_timing(delta: float) -> void:
	if game_manager.current_state == GameManager.GameState.RACE_ACTIVE:
		current_lap_time += delta
		total_race_time += delta
		
		# Check for lap completion
		if _check_lap_complete():
			_complete_lap()

func _check_lap_complete() -> bool:
	# Simple lap detection based on position or checkpoints
	# In production, use track markers
	var position = global_position
	var start_line_y = 100.0  # Adjust based on track layout
	
	if position.y < start_line_y and global_position.y >= start_line_y:
		return true
	
	return false

func _complete_lap() -> void:
	lap_count += 1
	var lap_time = current_lap_time
	
	if lap_time < best_lap_time:
		best_lap_time = lap_time
	
	lap_completed.emit(lap_time)
	current_lap_time = 0.0

# --- AIR DRAG ---
func _calculate_air_drag() -> float:
	var air_density = 1.225  # kg/m³ at sea level
	var drag_coefficient = 0.3  # Typical sports car
	var frontal_area = 2.2  # m²
	
	var speed_squared = current_speed * current_speed
	var drag_force = 0.5 * air_density * drag_coefficient * frontal_area * speed_squared
	
	return drag_force * Vector3.FORWARD

# --- ROLLING RESISTANCE ---
func _calculate_rolling_resistance() -> float:
	var rolling_resistance_coefficient = 0.015
	var normal_force = mass * 9.81
	
	var rolling_resistance = rolling_resistance_coefficient * normal_force
	
	return rolling_resistance * Vector3.FORWARD

# --- DEFAULT TORQUE CURVE ---
func _default_torque_curve(rpm: int) -> float:
	# Simplified torque curve (peak around 4500 RPM)
	if rpm < 2000:
		return 200.0 + (rpm - 2000) * 0.1
	elif rpm < 5000:
		return 300.0 - (rpm - 5000) * 0.05
	else:
		return 280.0 - (rpm - 5000) * 0.02

# --- COLLISION HANDLING ---
func _on_body_entered(body: Node) -> void:
	if body.has_method("take_damage"):
		body.take_damage(_get_collision_impact())
	
	var impact = _get_collision_impact()
	collision_impact.emit(impact)
	
	# Screen shake effect
	_screen_shake()

func _get_collision_impact() -> Vector3:
	return velocity * 0.5

func _screen_shake() -> void:
	if AudioManager:
		AudioManager.play_sound("screen_shake")

# --- AUDIO HANDLER ---
func _on_audio_played(sound_name: String) -> void:
	if sound_name == "engine_start":
		pass
	elif sound_name == "gear_shift":
		pass
	elif sound_name == "drift":
		pass

# --- SIGNAL EMITTERS ---
func _emit_physics_signals() -> void:
	speed_changed.emit(current_speed)
	rpm_changed.emit(current_rpm)

# --- PUBLIC API ---
func reset_vehicle() -> void:
	_init_vehicle_state()
	position = Vector3.ZERO
	velocity = Vector3.ZERO
	rotation = Vector3.ZERO

func set_track_position(position: Vector3) -> void:
	global_position = position

func get_current_speed_kmh() -> float:
	return current_speed * 3.6

func get_current_rpm() -> int:
	return current_rpm

func get_current_gear() -> int:
	return current_gear

func is_drifting_now() -> bool:
	return is_drifting

func get_drift_score() -> float:
	return drift_score

func get_best_lap_time() -> float:
	return best_lap_time

func get_total_race_time() -> float:
	return total_race_time

func get_slip_angle() -> float:
	return slip_angle

func get_lateral_g_force() -> float:
	return lateral_g

# --- HELPER METHODS ---
func _get_physics_settings() -> PhysicsSettings:
	if Engine.has_singleton("PhysicsSettings"):
		return Engine.get_singleton("PhysicsSettings")
	return ResourceLoader.load("res://scripts/core/PhysicsSettings.tres") as PhysicsSettings

func force_application(force: Vector3, point: Vector3) -> void:
	apply_central_impulse(force)

# --- DEBUG DISPLAY ---
func _draw_debug_info() -> void:
	if not game_manager.debug_mode:
		return
	
	var debug_text = [
		"Speed: %.1f km/h" % get_current_speed_kmh(),
		"RPM: %d" % current_rpm,
		"Gear: %d" % current_gear,
		"Drifting: %s" % str(is_drifting),
		"Slip Angle: %.1f°" % rad_to_deg(slip_angle),
		"Lap: %d/%.2fs" % [lap_count, current_lap_time],
	]
	
	# Display in viewport (simple text rendering)
	pass

# --- DESTRUCTOR ---
func _exit_tree() -> void:
	if powertrain:
		powertrain.engine_shutdown.emit()
		engine_event.emit(EngineEventType.SHUTDOWN)