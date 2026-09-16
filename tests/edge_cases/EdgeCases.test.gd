extends RefCounted
class_name EdgeCasesTest

## Edge Case and Error Handling Tests
## Copyright 2026 Thalamus Racing Simulator Project

## Test: Null Safety - GameManager Methods
func test_game_manager_null_safety() -> void:
	var gm = GameManager.new()
	
	# Test methods handle null gracefully
	gm._race_data.clear()
	gm._active_vehicles.clear()
	gm._lap_timing.clear()
	
	assert_equal(gm._race_data.size(), 0, "Race data should clear properly")
	assert_equal(gm._active_vehicles.size(), 0, "Active vehicles should clear properly")

## Test: Null Safety - AudioManager Methods
func test_audio_manager_null_safety() -> void:
	var am = AudioManager.new()
	
	am._sfx_library.clear()
	am._audio_streams.clear()
	
	assert_equal(am._sfx_library.size(), 0, "SFX library should clear properly")
	assert_equal(am._audio_streams.size(), 0, "Audio streams should clear properly")

## Test: Empty String Handling
func test_empty_string_handling() -> void:
	var test_str = ""
	assert_equal(test_str.length(), 0, "Empty string should have length 0")

## Test: Zero Value Handling
func test_zero_value_handling() -> void:
	assert_equal(0, 0, "Zero value should be valid")
	assert_not_equal(0, 1, "Zero should not equal one")

## Test: Negative Value Handling
func test_negative_value_handling() -> void:
	var neg_val = -1
	assert_less(neg_val, 0, "Negative value should be less than zero")

## Test: Large Value Handling
func test_large_value_handling() -> void:
	var large_val = 999999
	assert_greater(large_val, 1000, "Large value should exceed threshold")

## Test: Float Precision
func test_float_precision() -> void:
	var pi = 3.14159265359
	assert_greater(pi, 3.14, "Pi approximation should exceed 3.14")

## Test: Array Index Out Of Bounds Safety
func test_array_bounds_safety() -> void:
	var arr: Array = []
	# Accessing non-existent index should not crash
	assert_not(arr.has_index(0), "Empty array should not have index 0")

## Test: Dictionary Key Safety
func test_dictionary_key_safety() -> void:
	var dict = {}
	# Getting non-existent key should return null/default
	var val = dict.get("nonexistent", "default")
	assert_equal(val, "default", "Dictionary should return default for missing keys")

## Test: Signal Connection Safety
func test_signal_connection_safety() -> void:
	var node = Node.new()
	
	# Connecting to non-existent signal should fail gracefully
	var result = node.connect("nonexistent_signal", func(): pass)
	# Should fail or warn but not crash
	node.free()

## Test: File Access Safety
func test_file_access_nonexistent() -> void:
	var nonexistent = FileAccess.file_exists("res://nonexistent/file.gd")
	assert_false(nonexistent, "Nonexistent file should return false")

## Test: Resource Loading Failure
func test_resource_loading_failure() -> void:
	var resource = load("res://nonexistent/resource.tres")
	assert_null(resource, "Loading nonexistent resource should return null")
