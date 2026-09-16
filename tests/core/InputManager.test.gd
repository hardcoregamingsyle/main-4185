extends RefCounted
class_name InputManagerTest

## Unit Tests for InputManager
## Copyright 2026 Thalamus Racing Simulator Project

var _input_manager: InputManager

func _before_each() -> void:
	_input_manager = InputManager.new()

func _after_each() -> void:
	if _input_manager:
		_input_manager.free()
		_input_manager = null

## Test: Input Mapping Exists
func test_input_mapping_exists() -> void:
	assert_not_null(_input_manager._input_mappings, "Input mappings dictionary should exist")

## Test: Action Map Initialized
func test_action_map_initialized() -> void:
	assert_not_null(_input_manager._action_map, "Action map should be initialized")

## Test: Touch Controls Disabled By Default
func test_touch_controls_disabled_default() -> void:
	assert_false(_input_manager.touch_enabled, "Touch controls should be disabled by default")

## Test: Mouse Look Disabled By Default
func test_mouse_look_disabled_default() -> void:
	assert_false(_input_manager.mouse_look_enabled, "Mouse look should be disabled by default")

## Test: Keyboard Enabled By Default
func test_keyboard_enabled_default() -> void:
	assert_true(_input_manager.keyboard_enabled, "Keyboard controls should be enabled by default")

## Test: Deadzone Default Value
func test_deadzone_default() -> void:
	assert_equal(_input_manager.deadzone, 0.1, "Deadzone should default to 0.1")

## Test: Sensitivity Default Value
func test_sensitivity_default() -> void:
	assert_equal(_input_manager.sensitivity, 1.0, "Sensitivity should default to 1.0")

## Test: Axis Inversion Defaults
func test_axis_inversion_defaults() -> void:
	assert_false(_input_manager.invert_steering, "Steering inversion should default to false")
	assert_false(_input_manager.invert_camera, "Camera inversion should default to false")

## Test: Reset Actions Function Exists
func test_reset_actions_exists() -> void:
	assert_not_null(_input_manager.reset_actions, "reset_actions function should exist")

## Test: GetAxis Function Exists
func test_get_axis_exists() -> void:
	assert_not_null(_input_manager.get_axis, "get_axis function should exist")

## Test: GetButton Function Exists
func test_get_button_exists() -> void:
	assert_not_null(_input_manager.get_button, "get_button function should exist")

## Test: Process Function Exists
func test_process_function_exists() -> void:
	assert_not_null(_input_manager._process, "_process function should exist")
