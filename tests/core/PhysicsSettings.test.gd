extends RefCounted
class_name PhysicsSettingsTest

## Unit Tests for PhysicsSettings
## Copyright 2026 Thalamus Racing Simulator Project

var _physics_settings: PhysicsSettings

func _before_each() -> void:
	_physics_settings = PhysicsSettings.new()

func _after_each() -> void:
	if _physics_settings:
		_physics_settings.free()
		_physics_settings = null

## Test: Default Gravity Value
func test_default_gravity_value() -> void:
	assert_equal(_physics_settings.gravity, 9.81, "Default gravity should be 9.81 m/s²")

## Test: Default Physics Tick Rate
func test_default_physics_tick_rate() -> void:
	assert_equal(_physics_settings.physics_tick_rate, 120, "Default tick rate should be 120Hz")

## Test: Default Max Substeps
func test_default_max_substeps() -> void:
	assert_equal(_physics_settings.max_substeps, 4, "Default max substeps should be 4")

## Test: Default Time Scale
func test_default_time_scale() -> void:
	assert_equal(_physics_settings.time_scale, 1.0, "Default time scale should be 1.0")

## Test: Default Vehicle Mass
func test_default_vehicle_mass() -> void:
	assert_equal(_physics_settings.default_vehicle_mass, 1500.0, "Default vehicle mass should be 1500kg")

## Test: Gravity Setter Function Exists
func test_gravity_setter_exists() -> void:
	assert_not_null(_physics_settings._set_gravity, "Gravity setter function should exist")

## Test: Tick Rate Setter Function Exists
func test_tick_rate_setter_exists() -> void:
	assert_not_null(_physics_settings._set_physics_tick_rate, "Tick rate setter should exist")

## Test: Boundary Value - Zero Gravity
func test_zero_gravity_accepted() -> void:
	_physics_settings.gravity = 0.0
	assert_equal(_physics_settings.gravity, 0.0, "Zero gravity should be accepted")

## Test: Boundary Value - Negative Gravity
func test_negative_gravity_accepted() -> void:
	_physics_settings.gravity = -9.81
	assert_equal(_physics_settings.gravity, -9.81, "Negative gravity should be accepted")

## Test: Boundary Value - Very High Tick Rate
func test_high_tick_rate_accepted() -> void:
	_physics_settings.physics_tick_rate = 500
	assert_equal(_physics_settings.physics_tick_rate, 500, "High tick rate should be accepted")

## Test: Boundary Value - Zero Substeps
func test_zero_substeps_accepted() -> void:
	_physics_settings.max_substeps = 0
	assert_equal(_physics_settings.max_substeps, 0, "Zero substeps should be accepted")

## Test: Time Scale Edge Cases
func test_time_scale_edge_cases() -> void:
	_physics_settings.time_scale = 0.0
	assert_equal(_physics_settings.time_scale, 0.0, "Time scale 0 should be accepted")
	
	_physics_settings.time_scale = 2.0
	assert_equal(_physics_settings.time_scale, 2.0, "Time scale 2.0 should be accepted")
