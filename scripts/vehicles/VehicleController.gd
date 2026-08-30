extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulation
## Handles throttle, brake, steering, wheel forces, gear shifting, and vehicle dynamics
## Copyright 2026 Thalamus Racing Simulator Project

# ============================================================================
# SIGNALS - Game Event Notifications
# ============================================================================
signal speed_changed(current_speed: float)
signal rpm_changed(current_rpm: float)
signal gear_changed(new_gear: int)
signal drift_started(drift_intensity: float)
signal drift_ended()
signal collision_detected(collision_info: Dictionary)
signal lap_completed(lap_data: Dictionary)
signal race_event(event_type: String, data: Dictionary)
signal engine_sound_changed(rpm_ratio: float)
signal suspension_compressed(compression_amount: float)

# ============================================================================
# CONSTANTS - Physics Tuning Values
# ============================================================================
const MAX_SPEED_KMH: float = 320.0
const ACCELERATION_RATE: float = 12.0
const BRAKING_FORCE: float = 20.0
const TURN_SPEED: float = 4.5
const DRIFT_THRESHOLD: float = 0.7
const DRIFT_INTENSITY_MAX: float = 1.0
const MIN_GEAR: int = 0
const MAX_GEAR: int = 6
const IDLE_RPM: float = 800.0
const REDLINE_RPM: float = 7500.0
const SHIFT_RPM: float = 7000.0
const CLUTCH_RELEASE_TIME: float = 0.15
const TURBO_CHARGE_TIME: float = 2.5
const SUSPENSION_COMPRESSION_LIMIT: float = 0.3

# ============================================================================
# EXPORTED CONFIGURATION - Vehicle Setup (Exposed in Inspector)
# ============================================================================
@export_group("Vehicle Configuration")
@export var vehicle_mass: float = 1500.0
@export var engine_torque: float = 450.0
@export var transmission_ratio: float = 3.5
@export var differential_ratio: float = 3.8
@export var tire_friction: float = 0.9
@export var aerodynamic_drag: float = 0.35
@export var center_of_mass_offset: Vector3 = Vector3(0.0, 0.5, 0.0)

@export_group("Gear Ratios")
@export var gear_ratios: Array[float] = [3.8, 2.2, 1.6, 1.2, 0.9, 0.7, 0.6]
@export var reverse_ratio: float = 3.9
@export var final_drive_ratio: float = 3.5

@export_group("Brake System")
@export var brake_force_per_wheel: float = 4000.0
@export var abs_enabled: bool = true
@export var brake_bias_front: float = 0.55

@export_group("Suspension")
@export var suspension_stiffness: float = 100000.0
@export var suspension_damping: float = 8000.0
@export var suspension_travel: float = 0.15
@export var spring_rest_length: float = 0.3

# ============================================================================
# INTERNAL STATE
# ============================================================================
var _current_speed: float = 0.0
var _current_rpm: float = IDLE_RPM
var _current_gear: int = 1
var _is_reversing: bool = false
var _throttle_input: float = 0.0
var _brake_input: float = 0.0
var _steering_input: float = 0.0
var _drift_intensity: float = 0.0
var _turbo_active: bool = false
var _turbo_charge_level: float = 0.0
var _clutch_engaged: bool = true
var _engine_braking: bool = true
var _traction_control: bool = true

var _wheel_rpm: float = 0.0
var _suspension_states: Dictionary = {}
var _collision_history: Array[Dictionary] = []
var _last_collision_time: float = 0.0

# Powertrain reference
var _powertrain: Node = null

# ============================================================================
# INITIALIZATION
# ============================================================================
func _ready() -> void:
	_configure_suspension()
	_connect_signals()
	_init_powertrain()

func _configure_suspension() -> void:
	# Initialize suspension states for each wheel
	var wheel_names := ["front_left", "front_right", "rear_left", "rear_right"]
	for wheel in wheel_names:
		_suspension_states[wheel] = {
			"compression": 0.0,
			"velocity": 0.0,
			"force": 0.0
		}

func _init_powertrain() -> void:
	# Find powertrain node if it exists
	if has_node("../Powertrain"):
		_powertrain = get_node("../Powertrain")
	elif has_node("Powertrain"):
		_powertrain = get_node("Powertrain")
	else:
		print("[VehicleController] Warning: No Powertrain node found")

func _connect_signals() -> void:
	if GameManager:
		GameManager.game_state_changed.connect(_on_game_state_changed)

# ============================================================================
# INPUT HANDLING
# ============================================================================
func _input(event: InputEvent) -> void:
	if event is InputEventKey:
		match event.pressed:
			true:
				match event.keycode:
					KEY_W, KEY_UP:
						_throttle_input = min(_throttle_input + 0.1, 1.0)
					KEY_S, KEY_DOWN:
						_brake_input = min(_brake_input + 0.1, 1.0)
					KEY_A, KEY_LEFT:
						_steering_input = max(_steering_input - 0.1, -1.0)
					KEY_D, KEY_RIGHT:
						_steering_input = min(_steering_input + 0.1, 1.0)
					KEY_SPACE:
						_brake_input = 1.0
					KEY_SHIFT:
						_turbo_active = true
			false:
				match event.keycode:
					KEY_W, KEY_UP:
						_throttle_input = 0.0
					KEY_S, KEY_DOWN:
						_brake_input = 0.0
					KEY_A, KEY_LEFT:
						_steering_input = 0.0
					KEY_D, KEY_RIGHT:
						_steering_input = 0.0
					KEY_SPACE:
						_brake_input = 0.0
					KEY_SHIFT:
						_turbo_active = false

func _process_input_delta(delta: float) -> void:
	# Smooth input handling for better feel
	var target_throttle := InputManager.get_axis("throttle_up", "throttle_down")
	var target_brake := InputManager.get_axis("brake", "brake_reverse")
	var target_steering := InputManager.get_axis("steer_left", "steer_right")
	
	if target_throttle != null:
		_throttle_input = lerpf(_throttle_input, target_throttle * 1.0, delta * 5.0)
	
	if target_brake != null:
		_brake_input = lerpf(_brake_input, target_brake * 1.0, delta * 5.0)
	
	if target_steering != null:
		_steering_input = lerpf(_steering_input, target_steering * 1.0, delta * 8.0)

# ============================================================================
# PHYSICS UPDATE
# ============================================================================
func _physics_process(delta: float) -> void:
	_update_input(delta)
	_update_engine(delta)
	_update_transmission(delta)
	_update_drivetrain(delta)
	_update_wheels(delta)
	_update_suspension(delta)
	_update_aerodynamics(delta)
	_apply_forces(delta)
	_handle_drifting(delta)
	_check_gear_shifts(delta)
	_update_signals(delta)

func _update_input(delta: float) -> void:
	# Use InputManager for consistent input across platforms
	_process_input_delta(delta)
	
	# Reset inputs after processing
	if not Input.is_key_pressed(KEY_W) and not Input.is_key_pressed(KEY_UP):
		_throttle_input = lerp(_throttle_input, 0.0, delta * 10.0)
	if not Input.is_key_pressed(KEY_S) and not Input.is_key_pressed(KEY_DOWN):
		_brake_input = lerp(_brake_input, 0.0, delta * 10.0)
	if not Input.is_key_pressed(KEY_A) and not Input.is_key_pressed(KEY_LEFT) and \
	   not Input.is_key_pressed(KEY_D) and not Input.is_key_pressed(KEY_RIGHT):
		_steering_input = lerp(_steering_input, 0.0, delta * 10.0)

func _update_engine(delta: float) -> void:
	# Calculate current RPM based on gear and speed
	var gear_ratio := _get_current_gear_ratio()
	var wheel_radius := 0.3 # meters
	var effective_ratio := gear_ratio * final_drive_ratio
	
	if _current_speed > 0:
		_wheel_rpm = (_current_speed / 3.6) / (2.0 * PI * wheel_radius)
		_current_rpm = _wheel_rpm * effective_ratio
	else:
		# Engine idling when stationary
		if _throttle_input > 0.1:
			_current_rpm = lerp(_current_rpm, IDLE_RPM * 2.0, delta * 15.0)
		else:
			_current_rpm = lerp(_current_rpm, IDLE_RPM, delta * 5.0)
	
	# Apply engine braking effect
	if _engine_braking and _throttle_input == 0.0 and _current_gear > 0:
		var engine_resistance := 50.0 * delta
		_current_rpm = max(IDLE_RPM, _current_rpm - engine_resistance)
	
	# Clamp RPM within safe range
	_current_rpm = clamp(_current_rpm, IDLE_RPM, REDLINE_RPM * 1.1)

func _update_transmission(delta: float) -> void:
	# Manual gear shifting via keyboard
	var shift_request := 0
	if Input.is_action_just_pressed("gear_up"):
		shift_request = 1
	elif Input.is_action_just_pressed("gear_down"):
		shift_request = -1
	
	if shift_request != 0:
		var new_gear := _current_gear + shift_request
		new_gear = clamp(new_gear, MIN_GEAR, MAX_GEAR)
		
		if new_gear != _current_gear:
			_shift_gear(new_gear)

func _update_drivetrain(delta: float) -> void:
	# Calculate torque delivery to wheels
	var current_gear_ratio := _get_current_gear_ratio()
	var total_ratio := current_gear_ratio * final_drive_ratio
	
	var base_torque := engine_torque
	if _turbo_active:
		base_torque *= (1.0 + _turbo_charge_level * 0.5)
	
	var delivered_torque := base_torque * _throttle_input
	delivered_torque *= 0.5 # Distribute between front and rear
    
	# Apply torque to wheels based on drivetrain layout
	# For simplicity, assume RWD for this base implementation
	if _current_gear > 0:
		var wheel_torque := delivered_torque * transmission_ratio / total_ratio
		# Apply to rear wheels (simplified)
		# In full implementation, would apply to specific wheel nodes

func _update_wheels(delta: float) -> void:
	# Update wheel rotation based on vehicle movement
	var wheel_radius := 0.3
	var wheel_circumference := 2.0 * PI * wheel_radius
	
	if _current_speed > 0:
		var wheel_rotation_speed := (_current_speed / 3.6) / wheel_circumference
		# Rotate visual wheel meshes here
		pass
	else:
		# Wheels spinning while car is stationary (burnout)
		if _throttle_input > 0.8 and _brake_input < 0.1:
			_wheel_rpm = lerp(_wheel_rpm, _current_rpm * final_drive_ratio, delta * 10.0)

func _update_suspension(delta: float) -> void:
	# Simplified suspension model
	var wheel_positions := {
		"front_left": Vector3(-0.8, 0.0, 1.2),
		"front_right": Vector3(0.8, 0.0, 1.2),
		"rear_left": Vector3(-0.8, 0.0, -1.2),
		"rear_right": Vector3(0.8, 0.0, -1.2)
	}
	
	for wheel_name, position in wheel_positions:
		var state := _suspension_states[wheel_name]
		var height := 0.3 # Base height above ground
		var compression := 0.0
		
		# Raycast down to calculate compression
		var ray_end := global_position + Vector3(0, -height, 0)
		var space_state := get_world_3d().direct_space_state
		var query := PhysicsRayQueryParameters3D.create(
			global_position, 
			ray_end
		)
		var result := space_state.intersect_ray(query)
		
		if result and result.collider != null:
			var distance := global_position.distance_to(result.position)
			compression = max(0.0, (spring_rest_length - distance) / spring_rest_length)
			compression = min(compression, SUSPENSION_COMPRESSION_LIMIT)
			
			state["compression"] = lerp(state["compression"], compression, delta * 5.0)
			state["force"] = suspension_stiffness * compression
			
			# Emit suspension signal
			if compression > 0.1:
				suspension_compressed.emit(compression)

func _update_aerodynamics(delta: float) -> void:
	# Simplified drag calculation
	var speed_ms := _current_speed / 3.0 # Approximate conversion
	var drag_force := 0.5 * aerodynamic_drag * speed_ms * speed_ms
	
	# Apply drag as opposite force to velocity
	if _current_speed > 0:
		var drag_vector := velocity.normalized() * -drag_force
		velocity += drag_vector * delta / vehicle_mass

func _apply_forces(delta: float) -> void:
	# Calculate forward acceleration
	var gear_ratio := _get_current_gear_ratio()
	var drive_ratio := gear_ratio * final_drive_ratio
	
	var acceleration_factor := 1.0
	if _current_gear == 0:
		acceleration_factor = 0.0 # Neutral
	elif _is_reversing:
		acceleration_factor = -1.0
	
	var forward_acceleration := (engine_torque / vehicle_mass) * _throttle_input * acceleration_factor
	
	# Apply acceleration in direction of travel
	var move_direction := transform.basis.z * -1.0
	move_direction.y = 0.0
	move_direction = move_direction.normalized()
	
	velocity += move_direction * forward_acceleration * delta

func _handle_drifting(delta: float) -> void:
	# Detect drift conditions
	var speed_threshold := 40.0 # km/h minimum for drift
	var turn_threshold := 0.3
	
	if _current_speed > speed_threshold and abs(_steering_input) > turn_threshold:
		# Check lateral acceleration indicator (simplified)
		var lateral_acc := abs(velocity.x) / (_current_speed / 3.6 + 0.1)
		
		if lateral_acc > DRIFT_THRESHOLD:
			_drift_intensity = lerp(_drift_intensity, 1.0, delta * 3.0)
			if drift_intensity > 0.5 and drift_intensity < 1.0:
				drift_started.emit(_drift_intensity)
		else:
			_drift_intensity = lerp(_drift_intensity, 0.0, delta * 5.0)
			if _drift_intensity < 0.1:
				drift_ended.emit()
	else:
		_drift_intensity = lerp(_drift_intensity, 0.0, delta * 5.0)

func _check_gear_shifts(delta: float) -> void:
	# Automatic upshift at redline
	if _current_gear < MAX_GEAR and _current_rpm >= SHIFT_RPM:
		_shift_gear(_current_gear + 1)
	
	# Automatic downshift at low RPM
	elif _current_gear > 1 and _current_rpm <= IDLE_RPM * 1.5:
		_shift_gear(_current_gear - 1)

func _update_signals(delta: float) -> void:
	# Update and emit signals
	speed_changed.emit(_current_speed)
	rpm_changed.emit(_current_rpm)
	engine_sound_changed.emit((_current_rpm - IDLE_RPM) / (REDLINE_RPM - IDLE_RPM))

# ============================================================================
# GEAR SHIFTING
# ============================================================================
func _shift_gear(new_gear: int) -> void:
	if new_gear == _current_gear:
		return
	
	var old_gear := _current_gear
	_current_gear = new_gear
	
	# Handle neutral/reverse
	if new_gear == 0:
		_is_reversing = false
	elif new_gear < 0:
		_is_reversing = true
	else:
		_is_reversing = false
	
	# Emitter clutch engagement delay
	await get_tree().create_timer(CLUTCH_RELEASE_TIME).timeout
	_clutch_engaged = true
	
	gear_changed.emit(new_gear)
	race_event.emit("gear_shifted", {"old_gear": old_gear, "new_gear": new_gear})

func _get_current_gear_ratio() -> float:
	match _current_gear:
		0: return 0.0 # Neutral
		-1: return reverse_ratio # Reverse
		_:
			if _current_gear <= gear_ratios.size():
				return gear_ratios[_current_gear - 1]
			else:
				return gear_ratios.back()

# ============================================================================
# VEHICLE CONTROL METHODS
# ============================================================================
func set_throttle(amount: float) -> void:
	_throttle_input = clamp(amount, 0.0, 1.0)

func set_brake(amount: float) -> void:
	_brake_input = clamp(amount, 0.0, 1.0)

func set_steering(amount: float) -> void:
	_steering_input = clamp(amount, -1.0, 1.0)

func activate_turbo() -> void:
	_turbo_active = true
	_turbo_charge_level = 1.0

func deactivate_turbo() -> void:
	_turbo_active = false

func set_engine_braking(enabled: bool) -> void:
	_engine_braking = enabled

func set_traction_control(enabled: bool) -> void:
	_traction_control = enabled

func reset_vehicle() -> void:
	_current_speed = 0.0
	_current_rpm = IDLE_RPM
	_current_gear = 1
	_throttle_input = 0.0
	_brake_input = 0.0
	_steering_input = 0.0
	_drift_intensity = 0.0
	_velocity = Vector3.ZERO

# ============================================================================
# COLLISION HANDLING
# ============================================================================
func _on_collision_entered(body: Node) -> void:
	var collision_data := {
		"time": Time.get_ticks_msec(),
		"body": body.name if body else "unknown",
		"speed": _current_speed,
		"rpm": _current_rpm
	}
	_collision_history.append(collision_data)
	collision_detected.emit(collision_data)

func get_collision_count() -> int:
	return _collision_history.size()

func clear_collision_history() -> void:
	_collision_history.clear()

# ============================================================================
# GAME MANAGER EVENTS
# ============================================================================
func _on_game_state_changed(new_state: GameManager.GameState) -> void:
	match new_state:
		GameManager.GameState.RACE_ACTIVE:
			_reset_for_race()
		GameManager.GameState.RACE_PAUSED:
			_pause_physics()
		GameManager.GameState.RACE_FINISHED:
			_finish_race()

func _reset_for_race() -> void:
	reset_vehicle()
	_current_gear = 1
	_clutch_engaged = true

func _pause_physics() -> void:
	_process_mode = ProcessModeEnum.PROCESS_MODE_DISABLED

func _finish_race() -> void:
	set_process(false)
	set_physics_process(false)

# ============================================================================
# DEBUG AND UTILITIES
# ============================================================================
func get_speed_kmh() -> float:
	return _current_speed

func get_rpm() -> float:
	return _current_rpm

func get_current_gear() -> int:
	return _current_gear

func is_in_gear() -> bool:
	return _current_gear > 0

func is_reversing() -> bool:
	return _is_reversing

func is_drifting() -> bool:
	return _drift_intensity > 0.3

func get_drift_intensity() -> float:
	return _drift_intensity

func log_vehicle_status() -> Dictionary:
	return {
		"speed_kmh": _current_speed,
		"rpm": _current_rpm,
		"gear": _current_gear,
		"throttle": _throttle_input,
		"brake": _brake_input,
		"steering": _steering_input,
		"is_drifting": is_drifting(),
		"drift_intensity": _drift_intensity
	}