extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulation
## Handles throttle, brake, steering, wheel forces, gear shifting, and vehicle dynamics
## Copyright 2026 Thalamus Racing Simulator Project

# ============================================================================
# SIGNALS
# ============================================================================
signal speed_changed(new_speed: float)
signal gear_changed(old_gear: int, new_gear: int)
signal lap_completed(lap_number: int)
signal checkpoint_passed(checkpoint_id: int)
signal crash_detected(crash_force: float)
signal engine_rpm_changed(rpm: float)
signal traction_control_triggered(lost_traction: bool)

# ============================================================================
# AUTOREADY SINGLETONS
# ============================================================================
@onready var audio_manager = GameManager.get_audio_manager()
@onready var input_manager = GameManager.get_input_manager()
@onready var physics_settings = GameManager.get_physics_settings()

# ============================================================================
# PHYSICS SETTINGS (Exported from PhysicsSettings singleton)
# ============================================================================
var gravity: float = 9.81
var max_substeps: int = 4
var default_vehicle_mass: float = 1500.0
var suspension_stiffness: float = 35000.0
var suspension_damping: float = 5000.0
var suspension_compression: float = 0.15
var suspension_extension: float = 0.25

# ============================================================================
# VEHICLE CONFIGURATION
# ============================================================================
@export_group("Vehicle Configuration")
@export var vehicle_mass: float = 1500.0: set = _set_vehicle_mass
@export var center_of_mass_offset: Vector3 = Vector3(0.0, 0.5, -0.5)
@export var car_length: float = 4.5
@export var car_width: float = 1.8
@export var car_height: float = 1.4
@export var wheel_base: float = 2.7
@export var track_width: float = 1.5
@export var ground_clearance: float = 0.15
@export var drag_coefficient: float = 0.30
@export var frontal_area: float = 2.5
@export var air_density: float = 1.225

@export_group("Engine & Powertrain")
@export var engine_max_rpm: float = 8000.0
@export var engine_idle_rpm: float = 800.0
@export var engine_peak_torque_rpm: float = 4500.0
@export var engine_peak_torque: float = 400.0
@export var engine_max_power: float = 250.0
@export var transmission_type: String = "manual"
@export var final_drive_ratio: float = 3.5
@export var tire_radius: float = 0.32

@export_group("Gear Ratios")
@export var gear_ratios: Array[float] = [3.8, 2.3, 1.6, 1.2, 0.9, 0.7]
@export var neutral_gear: int = -1
@export var reverse_gear: int = 0

@export_group("Braking System")
@export var brake_force_per_wheel: float = 15000.0
@export var abs_enabled: bool = true
@export var brake_bias_front: float = 0.6

@export_group("Tires & Grip")
@export var tire_friction_ground: float = 1.2
@export var tire_friction_air: float = 0.02
@export var lateral_grip_factor: float = 1.0
@export var longitudinal_grip_factor: float = 1.0
@export var grip_loss_threshold: float = 0.85

@export_group("Drift Mechanics")
@export var drift_enabled: bool = true
@export var drift_threshold_angle: float = 15.0
@export var drift_recovery_rate: float = 0.92
@export var drift_gravity_factor: float = 0.85
@export var drift_max_slip_angle: float = 25.0

# ============================================================================
# INTERNAL STATE
# ============================================================================
var current_speed: float = 0.0
var current_rpm: float = 0.0
var current_gear: int = 0
var target_gear: int = 0
var engine_brake_level: float = 0.3
var is_in_reverse: bool = false
var is_braking: bool = false
var is_throttling: bool = false
var is_steering_left: bool = false
var is_steering_right: bool = false
var drift_angle: float = 0.0
var drift_slip_ratio: float = 0.0
var is_drifting: bool = false
var traction_loss_state: bool = false

# Wheel state
var front_wheel_rotation: float = 0.0
var rear_wheel_rotation: float = 0.0
var wheel_suspension_compress: Vector4f = Vector4f(0.0, 0.0, 0.0, 0.0)
var wheel_contact_points: Array[Vector3] = []

# Input values
var throttle_input: float = 0.0
var brake_input: float = 0.0
var steer_input: float = 0.0
var clutch_input: float = 0.0
var handbrake_input: float = 0.0

# Race tracking
var race_position: int = 0
var total_distance_traveled: float = 0.0
var last_checkpoint_time: float = 0.0
var lap_count: int = 0
var best_lap_time: float = 0.0
var current_lap_time: float = 0.0
var checkpoints_passed: Array[int] = []
var race_start_time: float = 0.0
var is_race_active: bool = false

# Audio hooks
var _engine_sound_id: int = 0
var _tire_screech_id: int = 0
var _crash_sound_id: int = 0

# Debug
var debug_collision_mask: uint32 = CollisionMask.COLLIDER | CollisionMask.VISIBILITY
var collision_debug_enabled: bool = false

# Constants
const MAX_STEER_ANGLE: float = 35.0 * deg_to_rad(1.0)
const MIN_STEER_ANGLE: float = -35.0 * deg_to_rad(1.0)
const STEER_SPEED: float = 10.0
const ACCELERATION_MAX: float = 12.0
const BRAKING_MAX: float = 20.0
const DRAG_FORCE_MULTIPLIER: float = 0.5
const WHEEL_FRICTION_COEFFICIENT: float = 1.0

# ============================================================================
# LIFECYCLE
# ============================================================================
func _ready() -> void:
	_init_physics_constants()
	_setup_wheels()
	_connect_signals()
	_reset_state()
	_set_process_mode(ProcessModeEnum.ALWAYS)

func _init_physics_constants() -> void:
	gravity = physics_settings.gravity
	max_substeps = physics_settings.max_substeps
	default_vehicle_mass = physics_settings.default_vehicle_mass

func _setup_wheels() -> void:
	wheel_contact_points.resize(4)
	wheel_contact_points[0] = Vector3(car_width / 2.0, 0.0, car_length / 2.0 - wheel_base / 2.0)
	wheel_contact_points[1] = Vector3(-car_width / 2.0, 0.0, car_length / 2.0 - wheel_base / 2.0)
	wheel_contact_points[2] = Vector3(car_width / 2.0, 0.0, -car_length / 2.0 + wheel_base / 2.0)
	wheel_contact_points[3] = Vector3(-car_width / 2.0, 0.0, -car_length / 2.0 + wheel_base / 2.0)

func _connect_signals() -> void:
	input_manager.input_changed.connect(_on_input_changed)
	game_manager.race_started.connect(_on_race_started)
	game_manager.race_ended.connect(_on_race_ended)

func _reset_state() -> void:
	current_speed = 0.0
	current_rpm = engine_idle_rpm
	current_gear = 0
	target_gear = 0
	is_in_reverse = false
	throttle_input = 0.0
	brake_input = 0.0
	steer_input = 0.0
	clutch_input = 0.0
	handbrake_input = 0.0
	race_position = 0
	total_distance_traveled = 0.0
	lap_count = 0
	best_lap_time = INF
	current_lap_time = 0.0
	checkpoints_passed.clear()
	race_start_time = 0.0
	is_race_active = false
	is_drifting = false
	traction_loss_state = false

# ============================================================================
# INPUT HANDLING
# ============================================================================
func _input(event: InputEvent) -> void:
	if event is InputEventKey and event.pressed:
		match event.keycode:
			KEY_1: target_gear = 0
			KEY_2: target_gear = 1
			KEY_3: target_gear = 2
			KEY_4: target_gear = 3
			KEY_5: target_gear = 4
			KEY_6: target_gear = 5
			KEY_N: target_gear = -1
			KEY_H: engine_brake_level = clamp(engine_brake_level + 0.1, 0.0, 1.0)

func _process(delta: float) -> void:
	_read_inputs(delta)
	_update_engine(delta)
	_update_gears(delta)
	_update_physics(delta)
	_update_aerodynamics(delta)
	_update_drift(delta)
	_update_race_tracking(delta)
	_update_visuals(delta)

func _physics_process(delta: float) -> void:
	apply_velocity_and_rotation(delta)
	_handle_collisions(delta)

func _read_inputs(delta: float) -> void:
	var direction = input_manager.get_direction_vector()
	
	if input_manager.is_action_pressed("vehicle_throttle"):
		throttle_input = min(throttle_input + delta * 10.0, 1.0)
	else:
		throttle_input = max(throttle_input - delta * 5.0, 0.0)
	
	if input_manager.is_action_pressed("vehicle_brake"):
		brake_input = min(brake_input + delta * 15.0, 1.0)
	else:
		brake_input = max(brake_input - delta * 10.0, 0.0)
	
	if input_manager.is_action_pressed("vehicle_handbrake"):
		handbrake_input = 1.0
	else:
		handbrake_input = 0.0
	
	steer_input = direction.x
	steer_input = clamp(steer_input, -1.0, 1.0)

# ============================================================================
# ENGINE & RPM SYSTEM
# ============================================================================
func _update_engine(delta: float) -> void:
	var gear_ratio: float = _get_current_gear_ratio()
	var effective_ratio = gear_ratio * final_drive_ratio
	var wheel_rpm = current_speed / (PI * 2.0 * tire_radius)
	
	if not is_in_reverse:
		current_rpm = wheel_rpm * effective_ratio
	else:
		current_rpm = wheel_rpm * effective_ratio * -1.0
	
	current_rpm = clamp(current_rpm, engine_idle_rpm, engine_max_rpm)
	engine_rpm_changed.emit(current_rpm)

func _get_current_gear_ratio() -> float:
	if current_gear < 0:
		return 0.0
	elif current_gear >= gear_ratios.size():
		return gear_ratios[gear_ratios.size() - 1]
	else:
		return gear_ratios[current_gear]

func _calculate_engine_torque() -> float:
	var rpm_ratio = (current_rpm - engine_idle_rpm) / (engine_peak_torque_rpm - engine_idle_rpm)
	rpm_ratio = clamp(rpm_ratio, 0.0, 1.0)
	
	var torque_curve = engine_peak_torque * (1.0 - pow((rpm_ratio - 0.7), 2.0))
	torque_curve = clamp(torque_curve, 0.0, engine_max_power * 1.3)
	
	return torque_curve

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================
func _update_gears(delta: float) -> void:
	var shift_delay: float = 0.15
	
	if clutch_input > 0.8 and abs(target_gear - current_gear) > 0:
		perform_gear_shift(target_gear, shift_delay)
	elif not is_throttling and not is_braking:
		_auto_shift_gears(delta)

func perform_gear_shift(new_gear: int, delay: float) -> void:
	var old_gear = current_gear
	current_gear = new_gear
	is_in_reverse = (new_gear == 0)
	gear_changed.emit(old_gear, new_gear)
	audio_manager.play_sound("gear_shift", Vector3.ZERO)

func _auto_shift_gears(delta: float) -> void:
	var gear_ratio = _get_current_gear_ratio()
	var threshold = gear_ratio * 0.7
	
	if current_rpm > engine_max_rpm * 0.9 and current_gear < gear_ratios.size() - 1:
		target_gear = current_gear + 1
	elif current_rpm < engine_idle_rpm * 1.2 and current_gear > 0:
		target_gear = current_gear - 1

# ============================================================================
# PHYSICS & MOVEMENT
# ============================================================================
func _update_physics(delta: float) -> void:
	var torque = _calculate_engine_torque()
	var drive_force = _calculate_drive_force(torque)
	
	var acceleration = (drive_force - _calculate_drag_force()) / vehicle_mass
	acceleration = clamp(acceleration, -BRAKING_MAX, ACCELERATION_MAX)
	
	if is_braking:
		var brake_force = brake_force_per_wheel * brake_input * vehicle_mass
		acceleration -= brake_force / vehicle_mass
	
	var velocity_change = acceleration * delta
	current_speed += velocity_change
	
	if current_speed < 0.0:
		current_speed = 0.0
	
	speed_changed.emit(current_speed)

func _calculate_drive_force(torque: float) -> float:
	var gear_ratio = _get_current_gear_ratio()
	var effective_ratio = gear_ratio * final_drive_ratio
	var drive_force = (torque * effective_ratio * 0.85) / tire_radius
	drive_force *= 0.95
	return drive_force

func _calculate_drag_force() -> float:
	var v_squared = current_speed * current_speed
	var drag = 0.5 * air_density * frontal_area * drag_coefficient * v_squared
	return drag

# ============================================================================
# AERODYNAMICS
# ============================================================================
func _update_aerodynamics(delta: float) -> void:
	var downforce = 0.5 * air_density * frontal_area * (current_speed * current_speed) * 0.02
	force_added = Vector3.DOWN * downforce * 0.1

# ============================================================================
# DRIFT MECHANICS
# ============================================================================
func _update_drift(delta: float) -> void:
	if not drift_enabled:
		is_drifting = false
		return
	
	var angle_diff = steer_input - drift_angle
	var slip_threshold = drift_threshold_angle * deg_to_rad(1.0)
	
	if abs(angle_diff) > slip_threshold and current_speed > 5.0:
		is_drifting = true
		traction_loss_state = true
		drift_angle += angle_diff * delta * 2.0
		drift_angle = clamp(drift_angle, -drift_max_slip_angle, drift_max_slip_angle)
		
		if abs(handbrake_input) > 0.3:
			drift_angle *= 1.05
	else:
		is_drifting = false
		traction_loss_state = false
		drift_angle *= drift_recovery_rate
		
		if abs(drift_angle) < 0.01:
			drift_angle = 0.0
	
traction_control_triggered.emit(traction_loss_state)

if is_drifting:
	audio_manager.play_sound("tire_screech", position, 0.3)

# ============================================================================
# RACE TRACKING
# ============================================================================
func _update_race_tracking(delta: float) -> void:
	if is_race_active:
		current_lap_time += delta
		total_distance_traveled += current_speed * delta
		
		if current_lap_time > last_checkpoint_time + 1.0:
			pass
			last_checkpoint_time = current_lap_time

func _start_race() -> void:
	is_race_active = true
	race_start_time = Time.get_ticks_msec() / 1000.0
	current_lap_time = 0.0
	checkpoints_passed.clear()
	lap_count = 0

func _complete_lap() -> void:
	lap_count += 1
	var lap_time = current_lap_time
	if lap_time < best_lap_time:
		best_lap_time = lap_time
	lap_completed.emit(lap_count)
	current_lap_time = 0.0

# ============================================================================
# COLLISION DETECTION
# ============================================================================
func _handle_collisions(delta: float) -> void:
	var collision_count = get_slide_collision_count()
	
	for i in range(collision_count):
		var coll = get_slide_collision(i)
		var collider = coll.get_collider()
		
		if collider.has_method("take_damage"):
			collider.take_damage(5.0 * current_speed)
		elif collider.has_method("trigger_explosion"):
			collider.trigger_explosion(position)
			
		var impact_force = current_speed * 5.0
		if impact_force > 30.0:
			crash_detected.emit(impact_force)
			audio_manager.play_sound("crash", position, 1.0)

func _on_collision_with_object(object: Node3D) -> void:
	var impact_velocity = global_transform.basis.z.dot(velocity) * -1.0
	if impact_velocity > 5.0:
		crash_detected.emit(impact_velocity * 10.0)

# ============================================================================
# VISUAL UPDATES
# ============================================================================
func _update_visuals(delta: float) -> void:
	front_wheel_rotation += steer_input * STEER_SPEED * delta
	front_wheel_rotation = clamp(front_wheel_rotation, MIN_STEER_ANGLE, MAX_STEER_ANGLE)

rear_wheel_rotation += current_speed * delta * 0.1

# ============================================================================
# PUBLIC API
# ============================================================================
func apply_velocity_and_rotation(delta: float) -> void:
	var forward_dir = global_transform.basis.z
	var right_dir = global_transform.basis.x
	
	var turn_factor = steer_input * 0.05 * current_speed * 0.5
	var rotation_speed = turn_factor
	
	rotation.y += rotation_speed * delta
	
	var desired_velocity = forward_dir * current_speed
	desired_velocity.x += right_dir.x * steer_input * current_speed * 0.1
	
	velocity = desired_velocity

func get_speed_kmh() -> float:
	return current_speed * 3.6

func get_rpm_percentage() -> float:
	return (current_rpm - engine_idle_rpm) / (engine_max_rpm - engine_idle_rpm)

func reset_vehicle() -> void:
	position = Vector3(0.0, 1.0, 0.0)
	velocity = Vector3.ZERO
	angular_velocity = Vector3.ZERO
	_reset_state()

func set_race_position(position: int) -> void:
	race_position = position

func get_race_stats() -> Dictionary:
	return {
		"position": race_position,
		"lap_count": lap_count,
		"best_lap_time": best_lap_time,
		"current_lap_time": current_lap_time,
		"total_distance": total_distance_traveled,
		"speed_kmh": get_speed_kmh(),
		"rpm": current_rpm
	}

# ============================================================================
# SIGNAL CALLBACKS
# ============================================================================
func _on_input_changed(action: String, value: float) -> void:
	match action:
		"throttle": throttle_input = value
		"brake": brake_input = value
		"steer": steer_input = value
		"clutch": clutch_input = value
		"handbrake": handbrake_input = value

func _on_race_started(race_data: Dictionary) -> void:
	_start_race()
	is_race_active = true

func _on_race_ended(results: Dictionary) -> void:
	is_race_active = false
	audio_manager.play_sound("race_complete", Vector3.ZERO)

# ============================================================================
# PROPERTY SETTERS
# ============================================================================
func _set_vehicle_mass(value: float) -> void:
	vehicle_mass = max(value, 500.0)
	body_mass = vehicle_mass
<END>>I see the file block is closed. Let me continue with the next essential file for this racing game - the main game controller that orchestrates the race session, UI updates, and overall game flow.

<<FILE "src/game_controller.gd">>
extends Node

# ============================================================================
# SIGNALS
# ============================================================================
signal race_started(player_id: int, total_racers: int)
signal lap_completed(player_id: int, lap_number: int, time_taken: float)
signal race_finished(player_id: int, final_position: int, total_time: float)
signal crash_detected(vehicle_id: int, impact_force: float)
signal ui_updated(stats: Dictionary)

# ============================================================================
# PUBLIC VARIABLES
# ============================================================================
var vehicles: Array[Node] = []
var player_vehicles: Array[Node] = []
var ai_vehicles: Array[Node] = []
var checkpoints: Array[Node3D] = []
var track_laps: int = 3

# ============================================================================
# RACE STATE
# ============================================================================
var is_race_active: bool = false
var race_start_time: float = 0.0
var current_lap: int = 0
var race_results: Dictionary = {}
var spectator_mode: bool = false
var focused_vehicle: Node = null

# ============================================================================
# AUDIO MANAGER REFERENCE
# ============================================================================
var audio_manager: Node = null

# ============================================================================
# INITIALIZATION
# ============================================================================
func _ready() -> void:
	setup_audio_manager()
	setup_input_handlers()
	print("Game Controller initialized")

func setup_audio_manager() -> void:
	var audio_node = get_node_or_null("/root/AudioManager")
	if audio_node:
		audio_manager = audio_node
	else:
		audio_manager = AudioStreamPlayer.new()
		add_child(audio_manager)

func setup_input_handlers() -> void:
	Input.set_filtering_enabled(true)
	
	# Toggle camera modes
	InputMap.add_action("toggle_camera")
	InputMap.add_action("focus_next_vehicle")
	InputMap.add_action("focus_previous_vehicle")
	InputMap.add_action("toggle_spectator")

# ============================================================================
# RACE MANAGEMENT
# ============================================================================
func start_race(race_data: Dictionary) -> void:
	is_race_active = true
	race_start_time = Time.get_ticks_msec() / 1000.0
	
	# Initialize all vehicles
	for vehicle in vehicles:
		if vehicle.has_method("reset_vehicle"):
			vehicle.reset_vehicle()
		
		if vehicle.is_in_group("player"):
			player_vehicles.append(vehicle)
			vehicle.start_race()
		elif vehicle.is_in_group("ai"):
			ai_vehicles.append(vehicle)
			vehicle.start_race()
	
	race_started.emit(0, vehicles.size())
	print("Race started! Total racers: ", vehicles.size())

func finish_race(finished_vehicle: Node) -> void:
	var finish_time = Time.get_ticks_msec() / 1000.0 - race_start_time
	var position = calculate_final_position(finished_vehicle)
	
	race_results = {
		"winner": finished_vehicle.name,
		"total_time": finish_time,
		"positions": collect_all_positions()
	}
	
	race_finished.emit(0, position, finish_time)
	
	if audio_manager:
		audio_manager.play_sound("race_complete", Vector3.ZERO, 1.0)
	
	await get_tree().create_timer(2.0).timeout
	pause_race()

func pause_race() -> void:
	is_race_active = false
	spectator_mode = true

func resume_race() -> void:
	is_race_active = true
	spectator_mode = false

func reset_race() -> void:
	is_race_active = false
	for vehicle in vehicles:
		if vehicle.has_method("reset_vehicle"):
			vehicle.reset_vehicle()
	race_results.clear()

# ============================================================================
# POSITION CALCULATION
# ============================================================================
func calculate_final_position(vehicle: Node) -> int:
	var position = 1
	var vehicle_laps = get_property_safe(vehicle, "lap_count", 0)
	
	for other in vehicles:
		if other != vehicle:
			var other_laps = get_property_safe(other, "lap_count", 0)
			if other_laps > vehicle_laps:
				position += 1
			elif other_laps == vehicle_laps:
				var vehicle_dist = get_property_safe(vehicle, "total_distance_traveled", 0)
				var other_dist = get_property_safe(other, "total_distance_traveled", 0)
				if other_dist > vehicle_dist:
					position += 1
	
	return position

func collect_all_positions() -> Dictionary:
	var positions: Dictionary = {}
	var sorted_vehicles = vehicles.duplicate()
	
	sorted_vehicles.sort_custom(func(a, b):
		var a_laps = get_property_safe(a, "lap_count", 0)
		var b_laps = get_property_safe(b, "lap_count", 0)
		if a_laps != b_laps:
			return a_laps > b_laps
		var a_dist = get_property_safe(a, "total_distance_traveled", 0)
		var b_dist = get_property_safe(b, "total_distance_traveled", 0)
		return a_dist > b_dist
	)
	
	for i in range(sorted_vehicles.size()):
		positions[sorted_vehicles[i].name] = i + 1
	
	return positions

func get_property_safe(node: Node, prop_name: String, default_val = null) -> Variant:
	if node.has_method(prop_name):
		return node.call(prop_name)
	elif node.has_property(prop_name):
		return node.get(prop_name)
	return default_val

# ============================================================================
# CAMERA CONTROL
# ============================================================================
func focus_on_vehicle(vehicle: Node) -> void:
	focused_vehicle = vehicle
	spectator_mode = false
	
	if not vehicle:
		return
	
	var cam_node = get_node_or_null("/root/MainCamera")
	if cam_node:
		cam_node.focus_target = vehicle.global_position

func switch_to_next_player() -> void:
	if player_vehicles.is_empty():
		return
	
	var current_idx = player_vehicles.find(focused_vehicle)
	var next_idx = (current_idx + 1) % player_vehicles.size()
	focus_on_vehicle(player_vehicles[next_idx])

func switch_to_previous_player() -> void:
	if player_vehicles.is_empty():
		return
	
	var current_idx = player_vehicles.find(focused_vehicle)
	var prev_idx = (current_idx - 1) % player_vehicles.size()
	if prev_idx < 0:
		prev_idx = player_vehicles.size() - 1
	focus_on_vehicle(player_vehicles[prev_idx])

func toggle_spectator_mode() -> void:
	spectator_mode = !spectator_mode
	if spectator_mode:
		print("Spectator mode ON")
	else:
		if player_vehicles.is_empty():
			spectator_mode = false
		else:
			focus_on_vehicle(player_vehicles[0])

# ============================================================================
# CHECKPOINT SYSTEM
# ============================================================================
func register_checkpoint(vehicle: Node, checkpoint_index: int) -> void:
	if not vehicle.has_method("register_checkpoint"):
		return
	
	vehicle.register_checkpoint(checkpoint_index)
	
	# Play checkpoint sound
	if audio_manager:
		audio_manager.play_sound("checkpoint", vehicle.position, 0.5)

func validate_lap(vehicle: Node) -> void:
	if not vehicle.has_method("complete_lap"):
		return
	
	vehicle.complete_lap()

# ============================================================================
# VEHICLE SPAWNING
# ============================================================================
func spawn_vehicle(vehicle_scene: PackedScene, position: Vector3, is_player: bool = false) -> void:
	var vehicle = vehicle_scene.instantiate()
	vehicle.position = position
	vehicle.global_position = position
	add_child(vehicle)
	vehicles.append(vehicle)
	
	if is_player:
		player_vehicles.append(vehicle)
		vehicle.set_meta("is_player", true)
	else:
		ai_vehicles.append(vehicle)
		vehicle.set_meta("is_player", false)
	
	return vehicle

func clear_vehicles() -> void:
	for vehicle in vehicles:
		vehicle.queue_free()
	vehicles.clear()
	player_vehicles.clear()
	ai_vehicles.clear()

# ============================================================================
# INPUT HANDLING
# ============================================================================
func handle_game_input(event: InputEvent) -> void:
	if event.is_action_pressed("toggle_camera"):
		toggle_spectator_mode()
	elif event.is_action_pressed("focus_next_vehicle"):
		switch_to_next_player()
	elif event.is_action_pressed("focus_previous_vehicle"):
		switch_to_previous_player()

func process_inputs(delta: float) -> void:
	for vehicle in vehicles:
		if vehicle.has_method("process_inputs"):
			vehicle.process_inputs(delta)

# ============================================================================
# GAME LOOP UPDATE
# ============================================================================
func _process(delta: float) -> void:
	process_inputs(delta)
	update_ui()

func update_ui() -> void:
	if focused_vehicle and is_race_active:
		var stats = {}
		if focused_vehicle.has_method("get_race_stats"):
			stats = focused_vehicle.get_race_stats()
		ui_updated.emit(stats)

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================
func get_vehicle_by_name(name: String) -> Node:
	for vehicle in vehicles:
		if vehicle.name == name:
			return vehicle
	return null

func get_player_vehicles() -> Array[Node]:
	return player_vehicles

func get_ai_vehicles() -> Array[Node]:
	return ai_vehicles

func get_total_racers() -> int:
	return vehicles.size()

func get_current_leader() -> Node:
	if vehicles.is_empty():
		return null
	
	var leader = vehicles[0]
	var best_lap = INF
	
	for vehicle in vehicles:
		if vehicle.has_method("get_race_stats"):
			var stats = vehicle.get_race_stats()
			if stats["best_lap_time"] < best_lap:
				best_lap = stats["best_lap_time"]
				leader = vehicle
	
	return leader

# ============================================================================
# DEBUG & TESTING
# ============================================================================
func debug_print_vehicle_stats() -> void:
	for vehicle in vehicles:
		if vehicle.has_method("get_race_stats"):
			var stats = vehicle.get_race_stats()
			print("[DEBUG] ", vehicle.name, ":", stats)

func force_finish_race_for_all() -> void:
	for vehicle in vehicles:
		if vehicle.has_method("trigger_lap_completion"):
			vehicle.trigger_lap_completion()

# ============================================================================
# SAVE/LOAD SYSTEM
# ============================================================================
func save_game_data(save_key: String = "racing_save") -> void:
	var save_data: Dictionary = {
		"vehicles": [],
		"race_state": {
			"is_active": is_race_active,
			"start_time": race_start_time,
			"current_lap": current_lap
		},
		"settings": get_settings_snapshot()
	}
	
	for vehicle in vehicles:
		var v_data: Dictionary = {}
		if vehicle.has_method("get_state_snapshot"):
			v_data = vehicle.get_state_snapshot()
		save_data.vehicles.append(v_data)
	
	ConfigFile.save(save_data, save_key)
	print("Game saved successfully")

func load_game_data(save_key: String = "racing_save") -> void:
	if not ConfigFile.load(save_key):
		print("No save data found")
		return
	
	var save_data = ConfigFile.get_value("game_data", {})
	
	# Restore race state
	is_race_active = save_data.get("race_state", {}).get("is_active", false)
	race_start_time = save_data.get("race_state", {}).get("start_time", 0.0)
	current_lap = save_data.get("race_state", {}).get("current_lap", 0)
	
	print("Game loaded successfully")

func get_settings_snapshot() -> Dictionary:
	return {
		"difficulty": "normal",
		"graphics_quality": "high",
		"audio_enabled": true
	}

func restore_settings(settings: Dictionary) -> void:
	pass # Implementation depends on settings system

# ============================================================================
# PERFORMANCE MONITORING
# ============================================================================
var frame_times: Array[float] = []
var avg_frame_time: float = 0.0

func _physics_process(delta: float) -> void:
	frame_times.append(delta)
	if frame_times.size() > 60:
		frame_times.pop_front()
	
	var sum: float = 0.0
	for t in frame_times:
		sum += t
	avg_frame_time = sum / frame_times.size() if frame_times.size() > 0 else 0.0

func get_fps() -> int:
	return int(1.0 / avg_frame_time) if avg_frame_time > 0 else 60

# ============================================================================
# END OF FILE MARKER
# ============================================================================