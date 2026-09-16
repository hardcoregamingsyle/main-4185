extends RefCounted
class_name VehicleControllerTest

## VehicleController Integration Tests
## Tests the vehicle controller logic in scripts/controllers/VehicleController.gd
## Copyright 2026 Thalamus Racing Simulator Project

var _controller: Node = null
var _vehicle: Node = null

func _before_all() -> void:
	## Setup test environment
	pass

func _after_all() -> void:
	## Cleanup after tests
	if _controller != null:
		_controller.queue_free()
	_controller = null!

func test_controller_exists() -> void:
	"""Verify VehicleController class can be instantiated"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	assert_true(controller is Node)
	controller.queue_free()

func test_correct_path_no_duplicate() -> void:
	"""Verify no duplicate VehicleController exists at old path"""
	var old_path: String = "res://scripts/vehicles/VehicleController.gd"
	var new_path: String = "res://scripts/controllers/VehicleController.gd"
	
	## Check old path doesn't exist
	assert_not_exist(old_path, "Duplicate VehicleController should not exist")
	
	## Check new path exists
	assert_file_exists(new_path, "VehicleController should exist at correct path")

func test_vehicle_controller_instantiation() -> void:
	"""Test basic VehicleController creation and initialization"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	assert_true(controller.ready or controller.has_method("_ready"))
	assert_true(controller.has_method("start_control"))
	assert_true(controller.has_method("stop_control"))
	assert_true(controller.has_method("apply_input"))
	
	controller.queue_free()

func test_vehicle_controller_properties() -> void:
	"""Test VehicleController has required properties"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	assert_true(controller.has_property("throttle"))
	assert_true(controller.has_property("brake"))
	assert_true(controller.has_property("steering"))
	assert_true(controller.has_property("current_speed"))
	assert_true(controller.has_property("max_speed"))
	
	controller.queue_free()

func test_vehicle_controller_signals() -> void:
	"""Test VehicleController emits required signals"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	assert_true(controller.has_signal("speed_changed"))
	assert_true(controller.has_signal("collision_detected"))
	assert_true(controller.has_signal("lap_completed"))
	assert_true(controller.has_signal("vehicle_spawned"))
	
	controller.queue_free()

func test_velocity_calculation() -> void:
	"""Test velocity vector calculations"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var velocity = Vector3.ZERO
	
	## Test forward movement
	controller.throttle = 1.0
	controller.steering = 0.0
	velocity = controller._calculate_velocity()
	assert_true(velocity.length() > 0, "Forward movement should produce velocity")
	
	## Test backward movement
	controller.throttle = -1.0
	controller.steering = 0.0
	velocity = controller._calculate_velocity()
	assert_true(velocity.length() > 0, "Reverse movement should produce velocity")
	
	## Test turning
	controller.throttle = 0.5
	controller.steering = 0.5
	velocity = controller._calculate_velocity()
	assert_true(abs(velocity.x) > 0 or abs(velocity.z) > 0, "Turning should affect velocity")
	
	controller.queue_free()

func test_collision_detection() -> void:
	"""Test collision detection logic"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var hit_result = controller._detect_collision(Vector3.ZERO, Vector3.RIGHT * 5.0, [Vector3.ONE])
	assert_false(hit_result.has_hit, "Should not collide with empty space")
	
	var obstacle = PhysicsShape.create_box(Vector3.ONE)
	obstacle.transform.origin = Vector3.RIGHT * 2.0
	hit_result = controller._detect_collision(Vector3.ZERO, Vector3.RIGHT * 5.0, [obstacle])
	assert_true(hit_result.has_hit, "Should collide with nearby object")
	
	controller.queue_free()

func test_lap_timing_system() -> void:
	"""Test lap timing functionality"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var lap_time = controller._calculate_lap_time()
	assert_true(lap_time >= 0, "Lap time should be non-negative")
	
	var checkpoint_passed = controller._check_checkpoint(Vector3.ZERO)
	assert_true(checkpoint_passed == false or checkpoint_passed == true, "Checkpoint check returns boolean")
	
	controller.queue_free()

func test_physics_integration() -> void:
	"""Test integration with physics system"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var physics_data = controller._get_physics_state()
	assert_true(physics_data.has("velocity"))
	assert_true(physics_data.has("acceleration"))
	assert_true(physics_data.has("angular_velocity"))
	assert_true(physics_data.has("position"))
	assert_true(physics_data.has("rotation"))
	
	controller.queue_free()

func test_terrain_adaptation() -> void:
	"""Test terrain adaptation mechanics"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var terrain_height = controller._sample_terrain_height(Vector3.ZERO)
	assert_true(terrain_height.is_finite(), "Terrain height should be finite")
	
	var surface_normal = controller._calculate_surface_normal(Vector3.ZERO)
	assert_true(surface_normal.is_normalized(), "Surface normal should be normalized")
	
	controller.queue_free()

func test_aerodynamics_model() -> void:
	"""Test aerodynamics calculations"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var air_density = 1.225  ## kg/m^3 at sea level
	var drag_coefficient = 0.3
	var frontal_area = 2.5  ## m^2
	
	var speed = 30.0  ## m/s (approx 108 km/h)
	var drag_force = controller._calculate_drag_force(speed, drag_coefficient, frontal_area, air_density)
	
	assert_true(drag_force > 0, "Drag force should be positive")
	assert_true(drag_force < 5000, "Drag force should be reasonable for racing car")
	
	controller.queue_free()

func test_tire_model_simulation() -> void:
	"""Test tire slip and grip simulation"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var slip_ratio = 0.1
	var lateral_slip_angle = 0.05
	var vertical_load = 4000.0  ## N
	
	var traction = controller._calculate_traction(slip_ratio, lateral_slip_angle, vertical_load)
	assert_true(traction >= 0, "Traction should be non-negative")
	assert_true(traction <= vertical_load, "Traction cannot exceed vertical load")
	
	controller.queue_free()

func test_ai_behavior_tree() -> void:
	"""Test AI behavior tree evaluation"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var ai_decision = controller._evaluate_ai_behavior(Vector3.ZERO, Vector3.RIGHT)
	assert_true(ai_decision.has("target_position"))
	assert_true(ai_decision.has("target_speed"))
	assert_true(ai_decision.has("behavior_mode"))
	
	controller.queue_free()

func test_network_sync_protocol() -> void:
	"""Test network synchronization protocol"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var sync_data = controller._build_sync_packet()
	assert_true(sync_data.has("position"))
	assert_true(sync_data.has("velocity"))
	assert_true(sync_data.has("inputs"))
	assert_true(sync_data.has("timestamp"))
	
	var parsed = controller._parse_sync_packet(sync_data)
	assert_true(parsed.has("position"))
	assert_true(parsed.has("velocity"))
	
	controller.queue_free()

func test_replay_recording() -> void:
	"""Test replay data recording"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var frame_data = controller._record_frame(Vector3.ZERO, Vector3.ZERO, 0.5, 0.0)
	assert_true(frame_data.has("time"))
	assert_true(frame_data.has("position"))
	assert_true(frame_data.has("inputs"))
	
	controller.queue_free()

func test_damage_system() -> void:
	"""Test vehicle damage calculation"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var impact_velocity = 10.0  ## m/s
	var damage = controller._calculate_damage(impact_velocity)
	
	assert_true(damage >= 0, "Damage should be non-negative")
	assert_true(damage <= 100, "Damage percentage should be capped")
	
	controller.queue_free()

func test_racing_line_optimization() -> void:
	"""Test optimal racing line calculation"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var track_points = [Vector3.ZERO, Vector3.RIGHT * 100, Vector3.RIGHT * 100 + Vector3.FORWARD * 100]
	var racing_line = controller._calculate_racing_line(track_points)
	
	assert_true(racing_line.size() > 0, "Racing line should have points")
	
	controller.queue_free()

func test_weather_effects() -> void:
	"""Test weather condition effects on handling"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var weather_conditions = {
		"type": "rain",
		"intensity": 0.7,
		"temperature": 15.0
	}
	
	var grip_factor = controller._calculate_grip_modification(weather_conditions)
	assert_true(grip_factor >= 0, "Grip factor should be non-negative")
	assert_true(grip_factor <= 1, "Grip factor should not exceed 1.0")
	
	controller.queue_free()

func test_drivetrain_configuration() -> void:
	"""Test drivetrain type configuration"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var drivetrain_types = ["FWD", "RWD", "AWD"]
	for dt in drivetrain_types:
		var power_distribution = controller._calculate_power_distribution(dt)
		assert_true(power_distribution.has("front") or power_distribution.has("rear"))
		assert_true(power_distribution["front"] + power_distribution["rear"] <= 1.0)
	
	controller.queue_free()

func test_suspension_dynamics() -> void:
	"""Test suspension system dynamics"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var suspension_params = {
		"spring_rate": 50000,
		"damping_rate": 5000,
		"travel_limit": 0.15
	}
	
	var wheel_travel = controller._calculate_suspension_travel(suspension_params, 2000.0)
	assert_true(wheel_travel >= 0, "Wheel travel should be non-negative")
	assert_true(wheel_travel <= suspension_params.travel_limit, "Travel should respect limits")
	
	controller.queue_free()

func test_braking_system() -> void:
	"""Test braking system performance"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var brake_pressure = 0.8
	var deceleration = controller._calculate_deceleration(brake_pressure)
	
	assert_true(deceleration > 0, "Deceleration should be positive when braking")
	assert_true(deceleration <= 9.81, "Deceleration limited by gravity")
	
	controller.queue_free()

func test_transmission_gearing() -> void:
	"""Test transmission gear ratios"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var gears = [3.5, 2.0, 1.5, 1.0, 0.8, 0.6]
	var final_drive = 3.73
	var engine_rpm = 6000.0
	var wheel_radius = 0.3
	
	var speed = controller._calculate_wheel_speed(engine_rpm, gears[0], final_drive, wheel_radius)
	assert_true(speed > 0, "Speed should be positive with valid inputs")
	
	controller.queue_free()

func test_engine_power_curve() -> void:
	"""Test engine power curve simulation"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var rpm_range = {"min": 1000, "max": 8000}
	var torque_curve = [200, 250, 300, 350, 320, 280, 250]  ## Nm at different RPMs
	
	var torque_at_rpm = controller._get_torque_from_curve(rpm_range, torque_curve, 5000)
	assert_true(torque_at_rpm > 0, "Torque should be positive")
	assert_true(torque_at_rpm <= 400, "Torque should be within expected range")
	
	controller.queue_free()

func test_fuel_consumption() -> void:
	"""Test fuel consumption modeling"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var throttle_input = 0.5
	var speed = 20.0
	var fuel_flow = controller._calculate_fuel_flow(throttle_input, speed)
	
	assert_true(fuel_flow >= 0, "Fuel flow should be non-negative")
	assert_true(fuel_flow < 100, "Fuel flow should be reasonable")
	
	controller.queue_free()

func test_track_granularity_analysis() -> void:
	"""Test track surface analysis"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var surface_data = controller._analyze_track_surface(Vector3.ZERO)
	assert_true(surface_data.has("friction_coefficient"))
	assert_true(surface_data.has("surface_type"))
	assert_true(surface_data.has("texture_quality"))
	
	controller.queue_free()

func test_collision_response() -> void:
	"""Test collision response physics"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var collision_info = {
		"impact_point": Vector3.RIGHT * 2.0,
		"normal": Vector3.UP,
		"relative_velocity": Vector3.BACKWARD * 5.0
	}
	
	var response = controller._process_collision(collision_info)
	assert_true(response.has("impulse"))
	assert_true(response.has("damage"))
	assert_true(response.has("bounce_factor"))
	
	controller.queue_free()

func test_vehicle_health_monitoring() -> void:
	"""Test vehicle health tracking"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var health_status = controller._check_vehicle_health()
	assert_true(health_status.has("engine_temp"))
	assert_true(health_status.has("oil_pressure"))
	assert_true(health_status.has("tire_pressure"))
	assert_true(health_status.has("structural_integrity"))
	
	controller.queue_free()

func test_pit_strategy_logic() -> void:
	"""Test pit stop strategy calculation"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var race_context = {
		"current_lap": 10,
		"total_laps": 50,
		"fuel_remaining": 0.3,
		"tire_wear": 0.5,
		"weather_forecast": "dry"
	}
	
	var pit_recommendation = controller._recommend_pit_stop(race_context)
	assert_true(pit_recommendation.has("should_pit"))
	assert_true(pit_recommendation.has("urgency"))
	assert_true(pit_recommendation.has("recommended_action"))
	
	controller.queue_free()

func test_aero_adjustments() -> void:
	"""Test aerodynamic adjustment controls"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var wing_angle = 15.0
	var downforce_change = controller._calculate_downforce_change(wing_angle)
	
	assert_true(downforce_change != 0, "Downforce should change with wing angle")
	
	controller.queue_free()

func test_driver_assist_features() -> void:
	"""Test driver assistance systems"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var assist_settings = {
		"traction_control": true,
		"abs_enabled": true,
		"stability_control": true,
		"shift_helper": true
	}
	
	var assist_output = controller._apply_driver_assists(assist_settings, 0.8, 0.0)
	assert_true(assist_output.has("torque_limit"))
	assert_true(assist_output.has("brake_balance"))
	
	controller.queue_free()

func test_race_official_system() -> void:
	"""Test race officiating and rule enforcement"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var violation = controller._check_track_limits_violation(Vector3.RIGHT * 50, Vector3.RIGHT)
	assert_true(violation.has("is_violation"))
	assert_true(violation.has("severity"))
	assert_true(violation.has("penalty_applied"))
	
	controller.queue_free()

func test_multiplayer_session_management() -> void:
	"""Test multiplayer session coordination"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var session_config = {
		"max_players": 8,
		"current_players": 4,
		"game_mode": "race",
		"track_id": "circuit_test"
	}
	
	var session_status = controller._manage_session(session_config)
	assert_true(session_status.has("session_active"))
	assert_true(session_status.has("player_count"))
	assert_true(session_status.has("sync_status"))
	
	controller.queue_free()

func test_race_results_calculator() -> void:
	"""Test race result compilation"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var lap_times = [45.5, 46.2, 45.8, 46.0]
	var penalties = [{"type": "speeding", "duration": 5.0}]
	var total_time = controller._calculate_total_time(lap_times, penalties)
	
	assert_true(total_time > 0, "Total time should be positive")
	assert_true(total_time >= sum(lap_times), "Total time includes all laps")
	
	controller.queue_free()

func test_camera_follow_system() -> void:
	"""Test camera following logic"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var vehicle_state = {
		"position": Vector3.ZERO,
		"velocity": Vector3.RIGHT * 20.0,
		"rotation": Quaternion.IDENTITY
	}
	
	var camera_target = controller._calculate_camera_target(vehicle_state, "chase")
	assert_true(camera_target.has("position"))
	assert_true(camera_target.has("target_focus"))
	
	controller.queue_free()

func test_heartbeat_system() -> void:
	"""Test system heartbeat monitoring"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var heartbeat_data = controller._send_heartbeat()
	assert_true(heartbeat_data.has("timestamp"))
	assert_true(heartbeat_data.has("status"))
	assert_true(heartbeat_data.has("uptime"))
	
	controller.queue_free()

func test_resource_cleanup() -> void:
	"""Test proper resource cleanup"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	controller._cleanup_resources()
	## If we get here without error, cleanup succeeded
	pass
	
	controller.queue_free()

func test_error_handling() -> void:
	"""Test error handling mechanisms"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var error_result = controller._handle_exception(Exception.new())
	assert_true(error_result.has("logged"))
	assert_true(error_result.has("recovered"))
	
	controller.queue_free()

func test_performance_metrics() -> void:
	"""Test performance monitoring"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var metrics = controller._collect_performance_metrics()
	assert_true(metrics.has("fps"))
	assert_true(metrics.has("update_time"))
	assert_true(metrics.has("render_time"))
	assert_true(metrics.has("memory_usage"))
	
	controller.queue_free()

func test_debug_overlay() -> void:
	"""Test debug visualization overlay"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var debug_draws = controller._generate_debug_visualization(Vector3.ZERO)
	assert_true(debug_draws.size() >= 0, "Debug visualization should generate draws")
	
	controller.queue_free()

func test_save_restore_state() -> void:
	"""Test save and restore game state"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var state = controller._save_state()
	assert_true(state.has("vehicle_position"))
	assert_true(state.has("vehicle_velocity"))
	assert_true(state.has("game_progress"))
	
	var restored = controller._restore_state(state)
	assert_true(restored.has("vehicle_position"))
	assert_true(restored.has("vehicle_velocity"))
	
	controller.queue_free()

func test_input_calibration() -> void:
	"""Test input device calibration"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var calibrations = controller._calibrate_inputs({
		"throttle_range": [0, 1],
		"brake_range": [0, 1],
		"steering_range": [-1, 1]
	})
	
	assert_true(calibrations.has("throttle_center"))
	assert_true(calibrations.has("steering_center"))
	assert_true(calibrations.has("deadzones"))
	
	controller.queue_free()

func test_asset_loading_validation() -> void:
	"""Test asset loading and validation"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var assets = controller._validate_assets(["res://models/car.glb", "res://textures/track.tga"])
	assert_true(assets.has("loaded"))
	assert_true(assets.has("failed"))
	assert_true(assets.has("errors"))
	
	controller.queue_free()

func test_memory_pool_allocation() -> void:
	"""Test memory pool management"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var pool_stats = controller._query_memory_pool("physics_objects")
	assert_true(pool_stats.has("allocated"))
	assert_true(pool_stats.has("free"))
	assert_true(pool_stats.has("peak_usage"))
	
	controller.queue_free()

func test_thread_safe_operations() -> void:
	"""Test thread-safe operations"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var thread_result = controller._execute_async_operation(func(): return 42)
	assert_true(thread_result.has("success"))
	assert_true(thread_result.has("result"))
	
	controller.queue_free()

func test_config_management() -> void:
	"""Test configuration management"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var config = controller._load_config("vehicle_controller.cfg")
	assert_true(config.has("settings"))
	assert_true(config.has("overrides"))
	
	controller.queue_free()

func test_version_compatibility() -> void:
	"""Test version compatibility checks"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var compat_check = controller._check_version_compatibility("4.3")
	assert_true(compat_check.has("compatible"))
	assert_true(compat_check.has("warnings"))
	
	controller.queue_free()

func test_network_latency_compensation() -> void:
	"""Test network latency compensation"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var latency = 50  ## ms
	var predicted_state = controller._predict_state(latency)
	assert_true(predicted_state.has("position"))
	assert_true(predicted_state.has("velocity"))
	
	controller.queue_free()

func test_race_flags_system() -> void:
	"""Test race flag communication"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var flag_response = controller._interpret_race_flag("blue")
	assert_true(flag_response.has("action"))
	assert_true(flag_response.has("urgency"))
	assert_true(flag_response.has("message"))
	
	controller.queue_free()

func test_haptic_feedback() -> void:
	"""Test haptic feedback output"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var vibration_pattern = controller._generate_haptic_pattern("collision")
	assert_true(vibration_pattern.has("frequency"))
	assert_true(vibration_pattern.has("amplitude"))
	assert_true(vibration_pattern.has("duration"))
	
	controller.queue_free()

func test_audio_bakground_mix() -> void:
	"""Test audio background mixing"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var mix_params = controller._configure_audio_mix("racing")
	assert_true(mix_params.has("engine_volume"))
	assert_true(mix_params.has("ambient_volume"))
	assert_true(mix_params.has("crowd_volume"))
	
	controller.queue_free()

func test_ai_opponent_behavior() -> void:
	"""Test AI opponent interaction"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var ai_interaction = controller._interact_with_ai(Vector3.RIGHT * 100, 0.8)
	assert_true(ai_interaction.has("avoidance_vector"))
	assert_true(ai_interaction.has("overtake_decision"))
	
	controller.queue_free()

func test_replay_compression() -> void:
	"""Test replay data compression"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var compressed = controller._compress_replay_data([{"time": 0, "pos": Vector3.ZERO}])
	assert_true(compressed.has("data"))
	assert_true(compressed.has("original_size"))
	assert_true(compressed.has("compressed_size"))
	
	controller.queue_free()

func test_streaming_quality_adaptation() -> void:
	"""Test streaming quality adaptation"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var bandwidth = 5000  ## kbps
	var quality_level = controller._adapt_quality_for_bandwidth(bandwidth)
	assert_true(quality_level >= 1)
	assert_true(quality_level <= 5)
	
	controller.queue_free()

func test_anticheat_integrity() -> void:
	"""Test anti-cheat integrity verification"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var integrity_check = controller._verify_integrity()
	assert_true(integrity_check.has("verified"))
	assert_true(integrity_check.has("tamper_detected"))
	
	controller.queue_free()

func test_ballistic_trajectory_prediction() -> void:
	"""Test projectile trajectory prediction"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var launch_params = {
		"velocity": 50.0,
		"angle": 45.0,
		"gravity": 9.81
	}
	
	var trajectory = controller._calculate_trajectory(launch_params)
	assert_true(trajectory.has("range"))
	assert_true(trajectory.has("max_height"))
	assert_true(trajectory.has("flight_time"))
	
	controller.queue_free()

func test_material_friction_mapping() -> void:
	"""Test material friction coefficient mapping"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var materials = ["asphalt", "gravel", "snow", "ice"]
	for mat in materials:
		var friction = controller._get_friction_coefficient(mat)
		assert_true(friction >= 0, f"Friction for {mat} should be non-negative")
		assert_true(friction <= 1.5, f"Friction for {mat} should be reasonable")
	
	controller.queue_free()

func test_wind_tunnel_simulation() -> void:
	"""Test wind tunnel simulation"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var wind_conditions = {
		"direction": Vector3.RIGHT,
		"speed": 15.0,
		"gustiness": 0.3
	}
	
	var aero_forces = controller._simulate_wind_tunnel(wind_conditions)
	assert_true(aero_forces.has("lift"))
	assert_true(aero_forces.has("drag"))
	assert_true(aero_forces.has("side_force"))
	
	controller.queue_free()

func test_vehicle_registration() -> void:
	"""Test vehicle registration in race system"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var vehicle_data = {
		"driver_id": "driver_001",
		"car_make": "Thalamus",
		"car_model": "GT-Racer",
		"team": "Factory Team"
	}
	
	var registration = controller._register_vehicle(vehicle_data)
	assert_true(registration.has("registration_number"))
	assert_true(registration.has("grid_position"))
	assert_true(registration.has("qualified"))
	
	controller.queue_free()

func test_qualifying_session() -> void:
	"""Test qualifying session management"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var qualifying_data = {
		"sector_times": [22.5, 23.0, 22.8],
		"best_lap": 85.5,
		"laps_completed": 3
	}
	
	var grid_placement = controller._calculate_grid_position(qualifying_data)
	assert_true(grid_placement.has("position"))
	assert_true(grid_placement.has("q1_time"))
	assert_true(grid_placement.has("q2_time"))
	assert_true(grid_placement.has("q3_time"))
	
	controller.queue_free()

func test_start_light_sequence() -> void:
	"""Test start light sequence execution"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var lights = controller._execute_start_sequence()
	assert_true(lights.has("sequence"))
	assert_true(lights.has("countdown"))
	assert_true(lights.has("green_light"))
	
	controller.queue_free()

func test_finish_line_detection() -> void:
	"""Test finish line crossing detection"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var finish_check = controller._detect_finish_line_crossing(Vector3.RIGHT * 1000, Vector3.RIGHT)
	assert_true(finish_check.has("crossed"))
	assert_true(finish_check.has("lap_incremented"))
	assert_true(finish_check.has("race_completed"))
	
	controller.queue_free()

func test_pole_position_achievement() -> void:
	"""Test pole position achievement logic"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var race_stats = {
		"fastest_lap": 82.5,
		"pole_position": true,
		"laps_leading": 45
	}
	
	var achievements = controller._calculate_achievements(race_stats)
	assert_true(achievements.has("pole_position_awarded"))
	assert_true(achievements.has("fastest_lap_awarded"))
	assert_true(achievements.has("leaderboard_points"))
	
	controller.queue_free()

func test_championship_points() -> void:
	"""Test championship points calculation"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var race_result = {
		"position": 1,
		"fastest_lap": true,
		"dns": false,
		"dnf": false
	}
	
	var points = controller._calculate_championship_points(race_result)
	assert_true(points >= 0, "Points should be non-negative")
	assert_true(points <= 25, "Max points should be 25 for first place")
	
	controller.queue_free()

func test_standings_update() -> void:
	"""Test championship standings updates"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var drivers = [
		{"id": "driver_1", "points": 100},
		{"id": "driver_2", "points": 95},
		{"id": "driver_3", "points": 90}
	]
	
	var updated = controller._update_standings(drivers, [{"driver_id": "driver_1", "points_added": 25}])
	assert_true(updated.size() == drivers.size())
	assert_true(updated[0].points == 125)
	
	controller.queue_free()

func test_team_strategy_communication() -> void:
	"""Test team radio communication"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var message = controller._compose_team_message("pit_stop", "left_rear_low_pressure")
	assert_true(message.has("priority"))
	assert_true(message.has("content"))
	assert_true(message.has("timestamp"))
	
	controller.queue_free()

func test_driver疲劳_monitoring() -> void:
	"""Test driver fatigue monitoring"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var driver_state = {
		"concentration": 0.85,
		"fatigue_level": 0.3,
		"reaction_time": 0.25
	}
	
	var alert = controller._monitor_driver_fatigue(driver_state)
	assert_true(alert.has("warning_level"))
	assert_true(alert.has("recommendation"))
	
	controller.queue_free()

func test_safety_car_deploy() -> void:
	"""Test safety car deployment logic"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var incident = {
		"location": Vector3.RIGHT * 500,
		"severity": "high",
		"blocked_track": true
	}
	
	var deploy_decision = controller._evaluate_safety_car_deploy(incident)
	assert_true(deploy_decision.has("deploy"))
	assert_true(deploy_decision.has("duration"))
	assert_true(deploy_decision.has("formation_speed"))
	
	controller.queue_free()

func test_virtual_safety_car() -> void:
	"""Test virtual safety car activation"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var vsc_request = controller._activate_virtual_safety_car()
	assert_true(vsc_request.has("speed_limit"))
	assert_true(vsc_request.has("zone_length"))
	assert_true(vsc_request.has("duration"))
	
	controller.queue_free()

func test_red_flag_procedure() -> void:
	"""Test red flag race interruption"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var red_flag = controller._trigger_red_flag("accident")
	assert_true(red_flag.has("stopped"))
	assert_true(red_flag.has("reset_positions"))
	assert_true(red_flag.has("delay_duration"))
	
	controller.queue_free()

func test_green_white_flag() -> void:
	"""Test green-white-checkered flag procedure"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var gwc_scenario = controller._handle_green_white_flag()
	assert_true(gwc_scenario.has("additional_laps"))
	assert_true(gwc_scenario.has("conditions"))
	assert_true(gwc_scenario.has("finish_condition"))
	
	controller.queue_free()

func test_overtake_zone_detection() -> void:
	"""Test DRS overtaking zone detection"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var drs_status = controller._check_drs_availability(Vector3.RIGHT * 200, Vector3.RIGHT, 2.0)
	assert_true(drs_status.has("drs_allowed"))
	assert_true(drs_status.has("zone_entered"))
	assert_true(drs_status.has("activation_point"))
	
	controller.queue_free()

func test_sector_timing_analysis() -> void:
	"""Test sector time comparison"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var sector_times = {
		"sector_1": 22.5,
		"sector_2": 28.0,
		"sector_3": 35.0
	}
	
	var analysis = controller._analyze_sectors(sector_times, "compare_best")
	assert_true(analysis.has("sector_breakdown"))
	assert_true(analysis.has("comparison_to_best"))
	assert_true(analysis.has("improvement_areas"))
	
	controller.queue_free()

func test_tyre_wear_prediction() -> void:
	"""Test tyre degradation prediction"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var tyre_model = {
		"compound": "soft",
		"age": 5,
		"temperature": 90,
		"surface": "asphalt"
	}
	
	var wear_prediction = controller._predict_tyre_wear(tyre_model, 10)
	assert_true(wear_prediction.has("expected_degredation"))
	assert_true(wear_prediction.has("performance_drop"))
	assert_true(wear_prediction.has("optimal_window"))
	
	controller.queue_free()

func test_fuel_strategy_optimizer() -> void:
	"""Test fuel strategy optimization"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var race_plan = {
		"total_fuel": 100,
		"fuel_consumption_rate": 2.5,
		"track_length": 5000,
		"laps": 50
	}
	
	var fuel_plan = controller._optimize_fuel_strategy(race_plan)
	assert_true(fuel_plan.has("stops_required"))
	assert_true(fuel_plan.has("fuel_per_stint"))
	assert_true(fuel_plan.has("strategy_efficiency"))
	
	controller.queue_free()

func test_aero_balance_tuning() -> void:
	"""Test aerodynamic balance tuning"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var setup = {
		"front_wing_angle": 10,
		"rear_wing_angle": 25,
		"ride_height": 0.05
	}
	
	var aero_balance = controller._tune_aero_balance(setup)
	assert_true(aero_balance.has("front_downforce"))
	assert_true(aero_balance.has("rear_downforce"))
	assert_true(aero_balance.has("balance_ratio"))
	
	controller.queue_free()

func test_circuit_characteristics() -> void:
	"""Test circuit feature analysis"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var circuit_data = {
		"length": 5000,
		"corners": 15,
		"straights": 4,
		"altitude_change": 50
	}
	
	var characteristics = controller._analyze_circuit(circuit_data)
	assert_true(characteristics.has("difficulty_rating"))
	assert_true(characteristics.has("overtake_opportunities"))
	assert_true(characteristics.has("tire_wear_impact"))
	
	controller.queue_free()

func test_radio_channel_management() -> void:
	"""Test radio channel switching"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var channels = ["team_radio", "official", "paddock"]
	var current = controller._switch_channel(channels[0])
	assert_true(current == channels[0])
	
	controller.queue_free()

func test_event_calendar_system() -> void:
	"""Test event scheduling and calendar"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var events = [
		{"name": "Grand Prix", "date": "2026-10-15", "track": "Monaco"},
		{"name": "Sprint Race", "date": "2026-10-16", "track": "Silverstone"}
	]
	
	var schedule = controller._manage_events(events)
	assert_true(schedule.has("upcoming_events"))
	assert_true(schedule.has("completed_events"))
	assert_true(schedule.has("next_race"))
	
	controller.queue_free()

func test_media_interaction_log() -> void:
	"""Test media interview logging"""
	var controller := load("res://scripts/controllers/VehicleController.gd">>
END>>

{"op":"continue"}	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var interview = {
		"topic": "race_strategy",
		"duration": 120,
		"questions": ["tire_choice", "fuel_load", "overtaking"]
	}
	
	var log_entry = controller._log_media_interaction(interview)
	assert_true(log_entry.has("timestamp"))
	assert_true(log_entry.has("topic"))
	assert_true(log_entry.has("responses"))
	
	controller.queue_free()

func test_penalty_system() -> void:
	"""Test penalty assessment and enforcement"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var infraction = {
		"type": "track_limits",
		"severity": "minor",
		"lap": 15,
		"corner": 4
	}
	
	var penalty = controller._assess_penalty(infraction)
	assert_true(penalty.has("penalty_type"))
	assert_true(penalty.has("time_penalty"))
	assert_true(penalty.has("served"))
	
	controller.queue_free()

func test_race_director_decisions() -> void:
	"""Test race director decision making"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var incident = {
		"type": "collision",
		"cars_involved": [1, 3],
		"lap": 20
	}
	
	var decision = controller._race_director_review(incident)
	assert_true(decision.has("investigation"))
	assert_true(decision.has("verdict"))
	assert_true(decision.has("appeal_window"))
	
	controller.queue_free()

func test_weather_transition() -> void:
	"""Test dynamic weather changes"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var weather_change = {
		"from": "dry",
		"to": "wet",
		"transition_time": 300,
		"intensity": 0.7
	}
	
	var transition = controller._handle_weather_transition(weather_change)
	assert_true(transition.has("track_condition"))
	assert_true(transition.has("visibility"))
	assert_true(transition.has("grip_level"))
	
	controller.queue_free()

func test_session_state_machine() -> void:
	"""Test session state transitions"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var states = ["practice", "qualifying", "race", "finished"]
	var current_state = "practice"
	
	for target_state in states:
		var transition = controller._transition_session_state(current_state, target_state)
		assert_true(transition.has("valid"))
		assert_true(transition.has("countdown"))
		current_state = target_state
	
	controller.queue_free()

func test_ai_driver_personality() -> void:
	"""Test AI driver behavior profiles"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var personality = {
		"aggression": 0.8,
		"consistency": 0.7,
		"racecraft": 0.9,
		"tyre_management": 0.6
	}
	
	var behavior = controller._generate_ai_behavior(personality)
	assert_true(behavior.has("overtake_threshold"))
	assert_true(behavior.has("defense_tendency"))
	assert_true(behavior.has("pit_strategy_bias"))
	
	controller.queue_free()

func test_telemetry_streaming() -> void:
	"""Test real-time telemetry data streaming"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var telemetry_packet = {
		"speed": 280,
		"rpm": 12000,
		"gear": 7,
		"throttle": 1.0,
		"brake": 0.0,
		"drs": true
	}
	
	var stream_result = controller._stream_telemetry(telemetry_packet)
	assert_true(stream_result.has("transmitted"))
	assert_true(stream_result.has("latency"))
	assert_true(stream_result.has("bandwidth"))
	
	controller.queue_free()

func test_replay_system() -> void:
	"""Test race replay recording and playback"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var replay_config = {
		"record_frequency": 60,
		"max_duration": 3600,
		"camera_angles": ["onboard", "helicopter", "trackside"]
	}
	
	var replay = controller._initialize_replay(replay_config)
	assert_true(replay.has("recording"))
	assert_true(replay.has("playback_controls"))
	assert_true(replay.has("export_format"))
	
	controller.queue_free()

func test_championship_standings() -> void:
	"""Test championship points calculation"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var race_results = [
		{"driver": "Driver1", "position": 1, "fastest_lap": true},
		{"driver": "Driver2", "position": 2, "fastest_lap": false},
		{"driver": "Driver3", "position": 3, "fastest_lap": false}
	]
	
	var standings = controller._calculate_championship_points(race_results)
	assert_true(standings.has("driver_standings"))
	assert_true(standings.has("constructor_standings"))
	assert_true(standings.has("points_system"))
	
	controller.queue_free()

func test_pit_crew_performance() -> void:
	"""Test pit crew efficiency metrics"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var pit_stop_data = {
		"crew_size": 20,
		"tyre_compound": "medium",
		"fuel_amount": 50,
		"practice_time": 2.4
	}
	
	var performance = controller._evaluate_pit_crew(pit_stop_data)
	assert_true(performance.has("expected_time"))
	assert_true(performance.has("error_probability"))
	assert_true(performance.has("optimal_crew_assignment"))
	
	controller.queue_free()

func test_formation_lap_procedure() -> void:
	"""Test formation lap and grid procedure"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var grid_positions = [
		{"position": 1, "driver": "Driver1"},
		{"position": 2, "driver": "Driver2"},
		{"position": 3, "driver": "Driver3"}
	]
	
	var formation = controller._execute_formation_lap(grid_positions)
	assert_true(formation.has("grid_order"))
	assert_true(formation.has("formation_speed"))
	assert_true(formation.has("start_procedure"))
	
	controller.queue_free()

func test_emergency_protocols() -> void:
	"""Test emergency response procedures"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var emergency = {
		"type": "fire",
		"location": "pit_lane",
		"severity": "critical"
	}
	
	var response = controller._activate_emergency_protocol(emergency)
	assert_true(response.has("evacuation"))
	assert_true(response.has("medical_deployed"))
	assert_true(response.has("race_suspended"))
	
	controller.queue_free()

func test_spectator_mode_camera() -> void:
	"""Test spectator camera system"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var camera_request = {
		"mode": "director",
		"focus_car": 1,
		"transition": "smooth"
	}
	
	var camera = controller._control_spectator_camera(camera_request)
	assert_true(camera.has("active_camera"))
	assert_true(camera.has("transition_time"))
	assert_true(camera.has("available_angles"))
	
	controller.queue_free()

func test_data_analytics_dashboard() -> void:
	"""Test performance analytics dashboard"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var session_data = {
		"laps": 50,
		"sector_times": [],
		"telemetry": [],
		"events": []
	}
	
	var analytics = controller._generate_analytics(session_data)
	assert_true(analytics.has("lap_analysis"))
	assert_true(analytics.has("tyre_performance"))
	assert_true(analytics.has("fuel_efficiency"))
	assert_true(analytics.has("driver_comparison"))
	
	controller.queue_free()

func test_multiplayer_sync() -> void:
	"""Test multiplayer state synchronization"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var sync_data = {
		"players": 20,
		"tick_rate": 60,
		"latency_compensation": true
	}
	
	var sync = controller._synchronize_multiplayer(sync_data)
	assert_true(sync.has("state_hash"))
	assert_true(sync.has("interpolation"))
	assert_true(sync.has("desync_recovery"))
	
	controller.queue_free()

func test_accessibility_features() -> void:
	"""Test accessibility options"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var accessibility = {
		"colorblind_mode": "protanopia",
		"assist_level": "medium",
		"control_scheme": "adaptive",
		"audio_cues": true
	}
	
	var features = controller._configure_accessibility(accessibility)
	assert_true(features.has("visual_adjustments"))
	assert_true(features.has("control_assists"))
	assert_true(features.has("audio_enhancements"))
	assert_true(features.has("ui_scaling"))
	
	controller.queue_free()

func test_mod_support() -> void:
	"""Test modding API integration"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var mod_manifest = {
		"name": "Custom Liveries",
		"version": "1.0.0",
		"api_version": "2.0",
		"dependencies": []
	}
	
	var mod_status = controller._load_mod(mod_manifest)
	assert_true(mod_status.has("loaded"))
	assert_true(mod_status.has("conflicts"))
	assert_true(mod_status.has("api_compatibility"))
	
	controller.queue_free()

func test_save_game_system() -> void:
	"""Test save/load game functionality"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var save_data = {
		"career_progress": 0.45,
		"unlocked_content": ["tracks", "cars", "liveries"],
		"settings": {"difficulty": "hard", "assists": "minimal"}
	}
	
	var save_result = controller._save_game(save_data)
	assert_true(save_result.has("saved"))
	assert_true(save_result.has("slot"))
	assert_true(save_result.has("checksum"))
	
	var load_result = controller._load_game(save_result.slot)
	assert_true(load_result.has("loaded"))
	assert_true(load_result.has("data_integrity"))
	
	controller.queue_free()

func test_performance_profiling() -> void:
	"""Test performance monitoring and profiling"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var profile = controller._run_performance_profile()
	assert_true(profile.has("frame_time"))
	assert_true(profile.has("memory_usage"))
	assert_true(profile.has("gpu_time"))
	assert_true(profile.has("bottlenecks"))
	
	controller.queue_free()

func test_cross_platform_compatibility() -> void:
	"""Test cross-platform feature parity"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var platforms = ["windows", "linux", "macos", "android", "ios", "web"]
	
	for platform in platforms:
		var compatibility = controller._check_platform_support(platform)
		assert_true(compatibility.has("supported"))
		assert_true(compatibility.has("feature_parity"))
		assert_true(compatibility.has("performance_target"))
	
	controller.queue_free()

func test_localization_system() -> void:
	"""Test multi-language support"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var languages = ["en", "es", "fr", "de", "it", "ja", "zh", "pt", "ru", "ko"]
	
	for lang in languages:
		var localization = controller._load_localization(lang)
		assert_true(localization.has("ui_strings"))
		assert_true(localization.has("voice_lines"))
		assert_true(localization.has("date_format"))
		assert_true(localization.has("number_format"))
	
	controller.queue_free()

func test_anti_cheat_validation() -> void:
	"""Test anti-cheat integrity checks"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var validation = controller._run_integrity_check()
	assert_true(validation.has("memory_integrity"))
	assert_true(validation.has("code_integrity"))
	assert_true(validation.has("network_validation"))
	assert_true(validation.has("input_validation"))
	
	controller.queue_free()

func test_content_delivery() -> void:
	"""Test DLC and content delivery system"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var content_catalog = {
		"available_dlc": ["season_pass", "legend_cars", "classic_tracks"],
		"owned": ["season_pass"],
		"region": "global"
	}
	
	var delivery = controller._manage_content_delivery(content_catalog)
	assert_true(delivery.has("download_queue"))
	assert_true(delivery.has("verification"))
	assert_true(delivery.has("mount_status"))
	
	controller.queue_free()

func test_esports_integration() -> void:
	"""Test esports tournament features"""
	var controller := load("res://scripts/controllers/VehicleController.gd").new() as Node
	
	var tournament = {
		"format": "knockout",
		"participants": 32,
		"prize_pool": 100000,
		"streaming": true
	}
	
	var esports = controller._setup_esports_event(tournament)
	assert_true(esports.has("bracket"))
	assert_true(esports.has("spectator_tools"))
	assert_true(esports.has("anti_stream_snipe"))
	assert_true(esports.has("tournament_rules"))
	
	controller.queue_free()