extends RefCounted
class_name MainSceneTest

## Integration Tests for Main.tscn
## Verifies scene loads correctly without errors
## Copyright 2026 Thalamus Racing Simulator Project

## Test: Main Scene Loads Without Errors
func test_main_scene_loads() -> void:
	var main_scene = PackedScene.new()
	var result = main_scene.load("res://scenes/main/Main.tscn")
	assert_equal(result, OK, "Main scene should load successfully")

## Test: Main Scene Has Required Nodes
func test_main_scene_has_required_nodes() -> void:
	var main_scene = PackedScene.new()
	main_scene.load("res://scenes/main/Main.tscn")
	
	var scene_tree = main_scene.instantiate() as Node
	assert_not_null(scene_tree, "Main scene should instantiate successfully")
	
	# Check for required child nodes
	assert_true(scene_tree.has_node("World"), "Scene should have World node")
	assert_true(scene_tree.has_node("Terrain"), "Scene should have Terrain node under World")
	assert_true(scene_tree.has_node("Lighting"), "Scene should have Lighting node")
	assert_true(scene_tree.has_node("Environment"), "Scene should have Environment node")
	assert_true(scene_tree.has_node("PostProcessing"), "Scene should have PostProcessing node")

## Test: Main Scene Script Attached
func test_main_scene_script_attached() -> void:
	var main_scene = PackedScene.new()
	main_scene.load("res://scenes/main/Main.tscn")
	
	var scene_tree = main_scene.instantiate() as Node
	var main_node = scene_tree.get_node_or_null("Main")
	
	assert_not_null(main_node, "Main node should exist")
	assert_not_null(main_node.get_script(), "Main node should have script attached")

## Test: GameManager Autoload Exists
func test_game_manager_autoload() -> void:
	var gm = get_node("/root/GameManager")
	assert_not_null(gm, "GameManager autoload should exist")

## Test: AudioManager Autoload Exists
func test_audio_manager_autoload() -> void:
	var am = get_node("/root/AudioManager")
	assert_not_null(am, "AudioManager autoload should exist")

## Test: InputManager Autoload Exists
func test_input_manager_autoload() -> void:
	var im = get_node("/root/InputManager")
	assert_not_null(im, "InputManager autoload should exist")

## Test: PhysicsSettings Resource Loadable
func test_physics_settings_loadable() -> void:
	var ps = preload("res://scripts/core/PhysicsSettings.gd")
	assert_not_null(ps, "PhysicsSettings resource should be loadable")
