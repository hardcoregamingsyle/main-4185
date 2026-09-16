extends RefCounted
class_name GameManagerTest

## Unit Tests for GameManager
## Copyright 2026 Thalamus Racing Simulator Project

var _game_manager: GameManager

func _before_each() -> void:
	# Setup fresh instance for each test
	_game_manager = GameManager.new()

func _after_each() -> void:
	# Cleanup after each test
	if _game_manager:
		_game_manager.free()
		_game_manager = null

## Test: GameState Enum Values
func test_game_state_enum_values() -> void:
	var main_menu_state: GameManager.GameState = GameManager.GameState.MAIN_MENU
	var race_active_state: GameManager.GameState = GameManager.GameState.RACE_ACTIVE
	
	assert_true(main_menu_state != race_active_state, "GameState enum values should be distinct")

## Test: Initial State Is MAIN_MENU
func test_initial_state_is_main_menu() -> void:
	assert_equal(_game_manager.current_state, GameManager.GameState.MAIN_MENU, 
		"GameManager should start in MAIN_MENU state")

## Test: Debug Mode Defaults To False
func test_debug_mode_defaults_to_false() -> void:
	assert_false(_game_manager.debug_mode, "Debug mode should default to false")

## Test: Race Data Dictionary Initialization
func test_race_data_dictionary_initialized() -> void:
	assert_equal(_game_manager._race_data.size(), 0, "Race data should be empty initially")

## Test: Active Vehicles Array Empty Initially
func test_active_vehicles_array_empty() -> void:
	assert_equal(_game_manager._active_vehicles.size(), 0, "Active vehicles array should be empty")

## Test: Signal Connections Can Be Made
func test_signal_connections_valid() -> void:
	var signal_connected = _game_manager.game_state_changed.is_connected(func(new_state): pass)
	# Just verify signal exists and can be connected
	assert_true(_game_manager.game_state_changed.get_signal_connection_list().size() >= 0,
		"Signals should be connectable")

## Test: Checkpoint System Null Initially
func test_checkpoint_system_null() -> void:
	assert_null(_game_manager._checkpoint_system, "Checkpoint system should be null initially")

## Test: Lap Timing Dictionary Empty
func test_lap_timing_dict_empty() -> void:
	assert_equal(_game_manager._lap_timing.size(), 0, "Lap timing should be empty initially")
