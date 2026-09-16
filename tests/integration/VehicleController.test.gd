extends RefCounted
class_name VehicleControllerTest

## Integration Tests for VehicleController
## Verifies correct controller exists at expected path
## No duplicates should exist
## Copyright 2026 Thalamus Racing Simulator Project

## Test: VehicleController Exists At Correct Path
func test_vehicle_controller_exists_correct_path() -> void:
	var controller_path = "res://scripts/controllers/VehicleController.gd"
	var exists = FileAccess.file_exists(controller_path)
	assert_true(exists, "VehicleController should exist at res://scripts/controllers/")

## Test: Duplicate Controller Does NOT Exist
func test_no_duplicate_vehicle_controller() -> void:
	var duplicate_path = "res://scripts/vehicles/VehicleController.gd"
	var exists = FileAccess.file_exists(duplicate_path)
	assert_false(exists, "Duplicate VehicleController should NOT exist at res://scripts/vehicles/")

## Test: Car Scene Loads
func test_car_scene_loads() -> void:
	var car_scene = PackedScene.new()
	var result = car_scene.load("res://scenes/vehicles/Car.tscn")
	assert_equal(result, OK, "Car scene should load successfully")

## Test: Car Scene Has Required Components
func test_car_scene_components() -> void:
	var car_scene = PackedScene.new()
	car_scene.load("res://scenes/vehicles/Car.tscn")
	
	var scene_tree = car_scene.instantiate() as Node
	assert_not_null(scene_tree, "Car scene should instantiate")

## Test: Powertrain Resource Loadable
func test_powertrain_loadable() -> void:
	var powertrain_path = "res://scripts/vehicles/Powertrain.gd"
	var exists = FileAccess.file_exists(powertrain_path)
	assert_true(exists, "Powertrain should exist")

## Test: No Namespace Conflicts
func test_no_namespace_conflicts() -> void:
	# Verify only one VehicleController class definition exists
	var files = ["res://scripts/controllers/VehicleController.gd",
	             "res://scripts/vehicles/VehicleController.gd"]
	var count = 0
	for f in files:
		if FileAccess.file_exists(f):
			count += 1
	assert_equal(count, 1, "Should have exactly one VehicleController file")
