extends Node
class_name GameManager

## GameManager - Central game state management, scene transitions, and high-level coordination

signal game_state_changed(new_state: GameState)
signal race_started(race_data: Dictionary)
signal race_ended(results: Dictionary)
signal vehicle_spawned(vehicle: Node)
signal vehicle_destroyed(vehicle: Node)

enum GameState {
	MAIN_MENU,
	LOADING,
	RACE_ACTIVE,
	RACE_PAUSED,
	RACE_FINISHED,
	SETTINGS,
	GARAGE,
	REPLAY
}

@export var current_state: GameState = GameState.MAIN_MENU
@export var debug_mode: bool = false

var _race_data: Dictionary = {}
var _active_vehicles: Array[Node] = []
var _checkpoint_system: Node = null
var _lap_timing: Dictionary = {}

func _ready() -> void:
	_process_mode = ProcessModeEnum.ALWAYS
	_init_singletons()
	_connect_signals()

func _init_singletons() -> void:
	# Ensure all core systems are initialized
	if AudioManager:
		AudioManager.initialize()
	if InputManager:
		InputManager.initialize()

func _connect_signals() -> void:
	# Connect to input manager for pause handling
	if InputManager:
		InputManager.pause_pressed.connect(_on_pause_pressed)

func _on_pause_pressed() -> void:
	if current_state == GameState.RACE_ACTIVE:
		set_game_state(GameState.RACE_PAUSED)
	elif current_state == GameState.RACE_PAUSED:
		set_game_state(GameState.RACE_ACTIVE)

func set_game_state(new_state: GameState) -> void:
	if current_state == new_state:
		return
	
	var old_state = current_state
	current_state = new_state
	
	match new_state:
		GameState.RACE_ACTIVE:
			Time.time_scale = 1.0
			InputManager?.set_input_mode(InputManager.InputMode.GAMEPLAY)
		GameState.RACE_PAUSED:
			Time.time_scale = 0.0
			InputManager?.set_input_mode(InputManager.InputMode.UI)
		GameState.MAIN_MENU:
			Time.time_scale = 1.0
			InputManager?.set_input_mode(InputManager.InputMode.UI)
		GameState.LOADING:
			Time.time_scale = 1.0
		GameState.SETTINGS:
			Time.time_scale = 0.0
			InputManager?.set_input_mode(InputManager.InputMode.UI)
		GameState.GARAGE:
			Time.time_scale = 1.0
			InputManager?.set_input_mode(InputManager.InputMode.UI)
		GameState.REPLAY:
			Time.time_scale = 1.0
			InputManager?.set_input_mode(InputManager.InputMode.GAMEPLAY)
	
	game_state_changed.emit(new_state)
	
	if debug_mode:
		print("[GameManager] State changed: ", old_state, " -> ", new_state)

func start_race(race_config: Dictionary) -> void:
	_race_data = race_config.duplicate(true)
	_lap_timing.clear()
	_active_vehicles.clear()
	set_game_state(GameState.LOADING)
	
	# Load race scene
	var scene_path = race_config.get("scene_path", "res://scenes/tracks/TestTrack.tscn")
	var scene = ResourceLoader.load_threaded_request(scene_path)
	
	# In a real implementation, you'd await the load and instance the scene
	# For now, emit signal that race is starting
	call_deferred("_deferred_race_start")

func _deferred_race_start() -> void:
	set_game_state(GameState.RACE_ACTIVE)
	race_started.emit(_race_data)

func end_race(results: Dictionary) -> void:
	set_game_state(GameState.RACE_FINISHED)
	race_ended.emit(results)

func register_vehicle(vehicle: Node) -> void:
	if vehicle and not _active_vehicles.has(vehicle):
		_active_vehicles.append(vehicle)
		vehicle.tree_exiting.connect(_on_vehicle_removed.bind(vehicle).once())
		vehicle_spawned.emit(vehicle)
		if debug_mode:
			print("[GameManager] Vehicle registered: ", vehicle.name)

func _on_vehicle_removed(vehicle: Node) -> void:
	if _active_vehicles.has(vehicle):
		_active_vehicles.erase(vehicle)
		vehicle_destroyed.emit(vehicle)

func get_active_vehicles() -> Array[Node]:
	return _active_vehicles.duplicate()

func get_vehicle_count() -> int:
	return _active_vehicles.size()

func record_lap_time(vehicle_id: int, lap: int, time: float) -> void:
	if not _lap_timing.has(vehicle_id):
		_lap_timing[vehicle_id] = []
	_lap_timing[vehicle_id].append({"lap": lap, "time": time, "timestamp": Time.get_ticks_msec()})

func get_lap_times(vehicle_id: int) -> Array[Dictionary]:
	return _lap_timing.get(vehicle_id, [])

func get_best_lap(vehicle_id: int) -> Dictionary:
	var laps = get_lap_times(vehicle_id)
	if laps.is_empty():
		return {}
	var best = laps[0]
	for lap in laps:
		if lap.time < best.time:
			best = lap
	return best

func pause_game() -> void:
	if current_state == GameState.RACE_ACTIVE:
		set_game_state(GameState.RACE_PAUSED)

func resume_game() -> void:
	if current_state == GameState.RACE_PAUSED:
		set_game_state(GameState.RACE_ACTIVE)

func return_to_menu() -> void:
	_active_vehicles.clear()
	_lap_timing.clear()
	_race_data.clear()
	set_game_state(GameState.MAIN_MENU)

func quit_game() -> void:
	get_tree().quit()

func _notification(what: int) -> void:
	if what == NotificationWMCloseRequest:
		quit_game()

## Debug utilities
func _on_debug_toggle() -> void:
	debug_mode = not debug_mode
	print("[GameManager] Debug mode: ", debug_mode)

func get_debug_info() -> Dictionary:
	return {
		"state": current_state,
		"vehicle_count": _active_vehicles.size(),
		"race_data": _race_data,
		"time_scale": Time.time_scale,
		"physics_tick_rate": PhysicsSettings.physics_tick_rate if PhysicsSettings else 120
	}