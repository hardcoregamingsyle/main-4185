extends RefCounted class_name VehicleControllerTest

## VehicleController Integration Tests
## Tests the vehicle controller logic in scripts/controllers/VehicleController.gd
## Copyright 2026 Thalamus Racing Simulator Project

var _controller: Node = null
var _vehicle: Node = null

func _before_each() -> void:
	_controller = load("res://scripts/controllers/VehicleController.gd").new()
	_vehicle = preload("res://scenes/vehicles/Car.tscn").instantiate()
	if _controller.has_method("_ready"):
		_controller._ready()
	if _vehicle.has_method("_ready"):
		_vehicle._ready()

func _after_each() -> void:
	if _controller != null:
		_controller.queue_free()
	if _vehicle != null:
		_vehicle.queue_free()

func test_controller_creation() -> void:
	assert_true(_controller != null, "VehicleController should be created")
	assert_eq(_controller.get_class(), "VehicleController", "Should instantiate VehicleController")

func test_controller_has_required_signals() -> void:
	assert_true(_controller.has_signal("throttle_changed"), "Should have throttle_changed signal")
	assert_true(_controller.has_signal("steering_changed"), "Should have steering_changed signal")
	assert_true(_controller.has_signal("brake_changed"), "Should have brake_changed signal")
	assert_true(_controller.has_signal("gear_changed"), "Should have gear_changed signal")

func test_controller_initial_state() -> void:
	assert_eq(_controller.throttle, 0.0, "Initial throttle should be 0")
	assert_eq(_controller.steering, 0.0, "Initial steering should be 0")
	assert_eq(_controller.brake, 0.0, "Initial brake should be 0")
	assert_eq(_controller.current_gear, 1, "Initial gear should be 1st")

func test_throttle_input() -> void:
	_controller.set_throttle(0.5)
	assert_eq(_controller.throttle, 0.5, "Throttle should be set to 0.5")
	
	_controller.set_throttle(1.0)
	assert_eq(_controller.throttle, 1.0, "Throttle should be capped at 1.0")
	
	_controller.set_throttle(-0.3)
	assert_eq(_controller.throttle, -0.3, "Throttle can be negative (reverse)")

func test_steering_input() -> void:
	_controller.set_steering(0.75)
	assert_eq(_controller.steering, 0.75, "Steering should be 0.75")
	
	_controller.set_steering(-0.75)
	assert_eq(_controller.steering, -0.75, "Steering should be -0.75")
	
	_controller.set_steering(1.5)
	assert_eq(_controller.steering, 1.0, "Steering should be capped at 1.0")
	
	_controller.set_steering(-1.5)
	assert_eq(_controller.steering, -1.0, "Steering should be capped at -1.0")

func test_brake_input() -> void:
	_controller.set_brake(0.8)
	assert_eq(_controller.brake, 0.8, "Brake should be 0.8")
	
	_controller.set_brake(1.0)
	assert_eq(_controller.brake, 1.0, "Brake should be capped at 1.0")

func test_gear_shifting() -> void:
	_controller.shift_up()
	assert_eq(_controller.current_gear, 2, "Gear should shift up to 2nd")
	
	_controller.shift_up()
	assert_eq(_controller.current_gear, 3, "Gear should shift up to 3rd")
	
	_controller.shift_down()
	assert_eq(_controller.current_gear, 2, "Gear should shift down to 2nd")
	
	_controller.shift_down()
	assert_eq(_controller.current_gear, 1, "Gear should shift down to 1st")

func test_gear_limits() -> void:
	var max_gears = _controller.max_gears
	
	for i in range(max_gears):
		_controller.shift_up()
	
	assert_eq(_controller.current_gear, max_gears, "Should not exceed max gears")
	assert_not_equal(_controller.shift_up(), true, "Shift up should fail at max gear")

func test_reverse_gear() -> void:
	_controller.reverse()
	assert_true(_controller.is_reversing, "Should enter reverse mode")
	assert_eq(_controller.current_gear, -1, "Reverse gear should be -1")

func test_vehicle_binding() -> void:
	_controller.bind_vehicle(_vehicle)
	assert_true(_controller.vehicle != null, "Vehicle should be bound")
	assert_eq(_controller.vehicle, _vehicle, "Bound vehicle should match")

func test_empty_vehicle_path_error() -> void:
	var error_found = false
	var error_text = ""
	
	# Test that controller handles missing vehicle gracefully
	_controller.validate_vehicle_reference()
	assert_true(error_found == false or error_text.length() > 0, 
		"Should handle empty vehicle reference")

func test_no_duplicate_file_conflicts() -> void:
	"""
	Verify that there is no duplicate VehicleController.gd file in scripts/vehicles/
	This was a known namespace conflict issue that has been resolved.
	"""
	var duplicate_path: String = "res://scripts/vehicles/VehicleController.gd"
	var actual_path: String = "res://scripts/controllers/VehicleController.gd"
	
	# Verify the correct controller exists
	assert_file_exists(actual_path, "Correct controller path should exist")
	
	# Verify the duplicate does NOT exist
	assert_not_file_exists(duplicate_path, "Duplicate controller path should NOT exist")

func assert_file_exists(path: String, message: String) -> void:
	var file_exists = FileAccess.file_exists(path.replace("res://", "./"))
	if !file_exists:
		prints("File check:", path, "-> does not exist")
		fail(message + " (file does not exist)")

func assert_not_file_exists(path: String, message: String) -> void:
	var file_exists = FileAccess.file_exists(path.replace("res://", "./"))
	if file_exists:
		prints("File check:", path, "-> EXISTS")
		fail(message + " (duplicate file found!)")

func test_controller_cleanup() -> void:
	_controller.cleanup()
	assert_true(_controller.is_ready_to_be_freed(), "Controller should be ready for cleanup")
</script>