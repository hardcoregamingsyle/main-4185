extends "res://tests/godot_test_runner.tscn"
class_name VehicleControllerTest

## VehicleController Integration Tests
## Tests for scripts/controllers/VehicleController.gd
## Copyright 2026 Thalamus Racing Simulator Project

var vehicle_controller_path: String = "res://scripts/controllers/VehicleController.gd"

func test_vehicle_controller_exists() -> void:
	"""Verify that VehicleController exists at the correct path"""
	assert_true(FileAccess.file_exists(vehicle_controller_path), 
		"VehicleController should exist at res://scripts/controllers/VehicleController.gd")

func test_no_duplicate_vehicle_controller() -> void:
	"""Verify that the duplicate VehicleController was removed from scripts/vehicles/"""
	var duplicate_path: String = "res://scripts/vehicles/VehicleController.gd"
	assert_false(FileAccess.file_exists(duplicate_path),
		"Duplicate VehicleController should NOT exist at res://scripts/vehicles/VehicleController.gd")

func test_vehicle_controller_inherits_from_node() -> void:
	"""Verify that VehicleController extends Node properly"""
	if FileAccess.file_exists(vehicle_controller_path):
		var file: FileAccess = FileAccess.open(vehicle_controller_path, FileAccess.READ)
		var content: String = file.get_as_text()
		file.close()
		
		assert_true(content.contains("extends Node"),
			"VehicleController should extend Node")
		assert_true(content.contains("class_name VehicleController"),
			"VehicleController should have class_name declaration")

func test_vehicle_controller_has_required_signals() -> void:
	"""Verify that VehicleController declares required signals"""
	if FileAccess.file_exists(vehicle_controller_path):
		var file: FileAccess = FileAccess.open(vehicle_controller_path, FileAccess.READ)
		var content: String = file.get_as_text()
		file.close()
		
		assert_true(content.contains("signal"),
			"VehicleController should declare signals")

func test_vehicle_controller_uses_physics_settings() -> void:
	"""Verify that VehicleController imports and uses PhysicsSettings"""
	if FileAccess.file_exists(vehicle_controller_path):
		var file: FileAccess = FileAccess.open(vehicle_controller_path, FileAccess.READ)
		var content: String = file.get_as_text()
		file.close()
		
		# Should reference GameManager for physics settings access
		assert_true(content.contains("GameManager") or content.contains("PhysicsSettings"),
			"VehicleController should use GameManager or PhysicsSettings")

func test_vehicle_controller_has_update_process() -> void:
	"""Verify that VehicleController has _process or _physics_process method"""
	if FileAccess.file_exists(vehicle_controller_path):
		var file: FileAccess = FileAccess.open(vehicle_controller_path, FileAccess.READ)
		var content: String = file.get_as_text()
		file.close()
		
		assert_true(content.contains("_process") or content.contains("_physics_process"),
			"VehicleController should have process loop")

func run_tests() -> void:
	"""Run all VehicleController tests"""
	test_vehicle_controller_exists()
	test_no_duplicate_vehicle_controller()
	test_vehicle_controller_inherits_from_node()
	test_vehicle_controller_has_required_signals()
	test_vehicle_controller_uses_physics_settings()
	test_vehicle_controller_has_update_process()
	
	print("VehicleController integration tests completed!")
</file>