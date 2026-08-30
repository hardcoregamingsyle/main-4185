extends Node3D

signal race_started
signal race_ended(results: Dictionary)
signal checkpoint_passed(checkpoint_index: int)
signal lap_completed(lap_time: float)

@export_group("Race Settings")
@export var total_laps: int = 3
@export var track_length: float = 5000.0
@export var ai_count: int = 5
@export var difficulty_level: Difficulty = Difficulty.NORMAL

@export_group("Track Settings")
@export var checkpoints_path: PackedStringArray = []
@export var start_position: Vector3 = Vector3.ZERO
@export var finish_position: Vector3 = Vector3(0, 0, -track_length)

@export_group("Vehicle Settings")
@export var default_vehicle_type: String = "sports_car"
@export var vehicle_spawn_point: Node3D

var _race_active: bool = false
var _current_lap: int = 0
var _player_vehicle: VehicleController3D
var _ai_vehicles: Array[VehicleController3D] = []
var _checkpoint_nodes: Array[Node3D] = []
var _lap_timers: Dictionary = {}
var _race_results: Dictionary = {}
var _race_start_time: float = 0.0
var _finish_line_collision: Area3D

func _ready() -> void:
	_setup_track()
	_setup_collisions()
	_initialize_vehicles()
	
func _setup_track() -> void:
	for i in range(checkpoints_path.size()):
		var checkpoint: Node3D = get_node_or_null(checkpoints_path[i])
		if checkpoint:
			_checkpoint_nodes.append(checkpoint)
	
	_finish_line_collision = Area3D.new()
	_finish_line_collision.name = "FinishLineCollision"
	
	var box_shape: BoxShape3D = BoxShape3D.new()
	box_shape.size = Vector3(10, 10, 1)
	_finish_line_collision.add_shape(box_shape)
	_finish_line_collision.position = finish_position
	
	add_child(_finish_line_collision)
	_finish_line_collision.area_entered.connect(_on_checkin_arrived)

func _setup_collisions() -> void:
	var area = Area3D.new()
	area.name = "PlayerCheckInArea"
	
	var box_shape: BoxShape3D = BoxShape3D.new()
	box_shape.size = Vector3(5, 5, 5)
	area.add_shape(box_shape)
	area.position = start_position
	
	area.body_entered.connect(_on_player_arrived)
	add_child(area)

func _initialize_vehicles() -> void:
	_create_player_vehicle()
	
	for i in range(ai_count):
		var ai_vehicle: VehicleController3D = _create_ai_vehicle(i)
		_ai_vehicles.append(ai_vehicle)

func _create_player_vehicle() -> void:
	var vehicle_scene: PackedScene = preload("res://scenes/vehicle_controller.tscn")
	_player_vehicle = vehicle_scene.instantiate() as VehicleController3D
	_player_vehicle.position = start_position
	add_child(_player_vehicle)
	
	_player_vehicle.vehicle_state_changed.connect(_on_player_state_changed)
	_player_vehicle.race_data_ready.connect(_on_race_data_ready)

func _create_ai_vehicle(index: int) -> VehicleController3D:
	var vehicle_scene: PackedScene = preload("res://scenes/vehicle_controller.tscn")
	var ai_vehicle = vehicle_scene.instantiate() as VehicleController3D
	
	# Position AI vehicles slightly ahead
	var offset: Vector3 = Vector3(randf_range(-2, 2), 0, randf_range(-2, 2)) * 10
	ai_vehicle.position = start_position + offset
	
	# Set AI difficulty
	ai_vehicle.set_difficulty(difficulty_level)
	
	add_child(ai_vehicle)
	return ai_vehicle

func _input(event: InputEvent) -> void:
	if event.is_action_pressed("pause"):
		toggle_pause()

func _process(delta: float) -> void:
	if _race_active:
		_update_racing_logic(delta)
		
func _update_racing_logic(delta: float) -> void:
	for vehicle in _ai_vehicles:
		vehicle.update_ai_behavior(delta)
	
	_update_checkpoints()

func _update_checkpoints() -> void:
	var player_pos: Vector3 = _player_vehicle.global_position
	
	for i in range(_checkpoint_nodes.size()):
		var checkpoint: Node3D = _checkpoint_nodes[i]
		var distance: float = player_pos.distance_to(checkpoint.global_position)
		
		if distance < 5.0:
			emit_signal(checkpoint_passed, i)

func start_race() -> void:
	if _race_active:
		return
	
	_reset_race()
	_race_active = true
	_race_start_time = Time.get_unix_time_from_system()
	
	for vehicle in _ai_vehicles:
		vehicle.start_race()
	
	if _player_vehicle:
		_player_vehicle.start_race()
	
	emit_signal(race_started)

func stop_race() -> void:
	_race_active = false
	for vehicle in _ai_vehicles:
		vehicle.stop_race()
	
	if _player_vehicle:
		_player_vehicle.stop_race()
	
	_calculate_final_results()
	emit_signal(race_ended, _race_results)

func toggle_pause() -> void:
	get_tree().paused = !get_tree().paused

func _reset_race() -> void:
	_current_lap = 0
	_race_results.clear()
	_lap_timers.clear()
	
	for vehicle in _ai_vehicles:
		vehicle.reset_for_new_race()
	
	if _player_vehicle:
		_player_vehicle.reset_for_new_race()

func _calculate_final_results() -> void:
	for vehicle in _ai_vehicles:
		_race_results[vehicle.name] = {
			"position": _find_race_position(vehicle),
			"total_time": _calculate_total_time(vehicle),
			"laps_completed": _get_laps_completed(vehicle)
		}
	
	if _player_vehicle:
		_race_results["player"] = {
			"position": _find_race_position(_player_vehicle),
			"total_time": _calculate_total_time(_player_vehicle),
			"laps_completed": _get_laps_completed(_player_vehicle)
		}

func _find_race_position(vehicle: VehicleController3D) -> int:
	var sorted_vehicles: Array = _ai_vehicles.duplicate()
	sorted_vehicles.sort_custom(func(a, b): return a._progress > b._progress)
	
	for i in range(sorted_vehicles.size()):
		if sorted_vehicles[i] == vehicle:
			return i + 1
	
	return sorted_vehicles.size() + 1

func _calculate_total_time(vehicle: VehicleController3D) -> float:
	if vehicle._last_checkpoint_time == 0:
		return 0.0
	
	return Time.get_unix_time_from_system() - _race_start_time

func _get_laps_completed(vehicle: VehicleController3D) -> int:
	return vehicle.laps_completed

func _on_player_state_changed(new_state: VehicleState) -> void:
	match new_state:
		VehicleState.RACING:
			_current_lap += 1
			if _current_lap >= total_laps:
				stop_race()
		VehicleState.CRASHED:
			print("Player vehicle crashed!")
		VehicleState.IDLE:
			pass

func _on_race_data_ready(data: Dictionary) -> void:
	_update_leaderboard(data)

func _update_leaderboard(data: Dictionary) -> void:
	pass

func _on_checkin_arrived(body: Node3D) -> void:
	if body is VehicleController3D:
		body.on_finish_line()

func get_race_status() -> Dictionary:
	return {
		"active": _race_active,
		"current_lap": _current_lap,
		"total_laps": total_laps,
		"player_progress": _player_vehicle._progress if _player_vehicle else 0.0
	}

func reset_game() -> void:
	stop_race()
	_reset_race()
	_player_vehicle.reset_vehicle()
	for vehicle in _ai_vehicles:
		vehicle.reset_vehicle()

func load_track_config(config: Dictionary) -> void:
	total_laps = config.get("total_laps", 3)
	difficulty_level = config.get("difficulty", Difficulty.NORMAL)
	checkpoints_path = config.get("checkpoints", [])

func save_race_data(filename: String) -> void:
	var data: Dictionary = {
		"results": _race_results,
		"status": get_race_status(),
		"time": Time.get_datetime_string_from_system()
	}
	
	var json_str: String = JSON.stringify(data, "\t")
	var file := FileAccess.open(filename, FileAccess.WRITE)
	if file:
		file.store_string(json_str)
		file.close()

func load_race_data(filename: String) -> void:
	var file := FileAccess.open(filename, FileAccess.READ)
	if file:
		var json_str: String = file.get_as_text()
		file.close()
		
		var data: Dictionary = JSON.parse_string(json_str)
		_race_results = data.get("results", {})
	else:
		push_warning("Could not load race data from " + filename)

func _on_player_arrived(body: Node3D) -> void:
	if body is VehicleController3D:
		body.on_start_line()

# ============================================================================
# END OF FILE
# ============================================================================
</FILE_END>>
