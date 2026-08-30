extends Resource
class_name Powertrain

## Powertrain - Advanced engine/transmission simulation with realistic physics
## Handles RPM management, torque curves, gear ratios, clutch engagement, and throttle response
## Speed emerges naturally from physics: Force = Torque / WheelRadius * GearRatio

signal rpm_changed(rpm: float)
signal gear_changed(new_gear: int)
signal rev_match_requested(target_rpm: float)
signal clutch_engaged()
signal clutch_disengaged()

@export_group("Engine Specifications")
@export var engine_type: String = "V8"
@export var displacement: float = 5.0  # liters
@export var max_torque_nm: float = 550.0  # Newton-meters
@export var max_torque_rpm: float = 4500.0
@export var redline_rpm: float = 7000.0
@export var idle_rpm: float = 800.0
@export var max_rpm: float = 7500.0  # absolute maximum (with limiter)
@export var compression_ratio: float = 11.0
@export var cylinder_count: int = 8

@export_group("Power Output")
@export var peak_horsepower: float = 420.0  # HP
@export var peak_power_rpm: float = 6200.0
@export var min_power_threshold: float = 300.0  # minimum usable power

@export_group("Transmission")
@export var transmission_type: String = "Manual"  # Manual, Automatic, Sequential, CVT
@export var total_gears: int = 6
@export var reverse_gear: bool = true

@export_group("Gear Ratios")
@export var gear_ratios: Array[float] = [3.5, 2.2, 1.5, 1.1, 0.9, 0.75]
@export var final_drive_ratio: float = 3.73
@export var reverse_ratio: float = 3.8

@export_group("Wheel & Drivetrain")
@export var wheel_diameter_m: float = 0.68  # meters (approx 27 inch tire)
@export var drive_wheels: String = "RWD"  # FWD, RWD, AWD
@export var differential_type: String = "LSD"  # Open, LSD, Locked
@export var drivetrain_efficiency: float = 0.85  # 85% efficiency loss

@export_group("Clutch & Throttle")
@export var clutch_friction_coefficient: float = 0.4
@export var clutch_engagement_time: float = 0.2  # seconds to fully engage
@export var throttle_response_curve: float = 0.7  # 0-1, higher = more aggressive
@export var turbo_enabled: bool = false
@export var boost_target_psi: float = 14.0

# Runtime state
var current_rpm: float = idle_rpm
var target_rpm: float = idle_rpm
var current_gear: int = 0  # 0 = neutral, 1-6 = forward, -1 = reverse
var clutch_pedal_position: float = 1.0  # 1.0 = disengaged, 0.0 = engaged
var throttle_input: float = 0.0  # 0.0-1.0
var brake_input: float = 0.0  # 0.0-1.0
var is_engine_running: bool = false
var engine_temperature: float = 90.0  # Celsius
var oil_pressure: float = 0.0  # Bar
var is_in_rev_limit: bool = false
var current_torque_output: float = 0.0
var current_power_output_hp: float = 0.0

# Turbo variables
var turbo_spool: float = 0.0
var turbo_boost_psi: float = 0.0
var turbo_inertia: float = 0.1
var wastegate_open: bool = false

# Vehicle reference for wheel speed calculation
var _vehicle_body: Node3D = null
var _wheel_radius: float = 0.33
var _is_connected: bool = false

func _init() -> void:
	_reset_runtime_state()

func _process(_delta: float) -> void:
	if not is_connected_to_vehicle():
		return
	
	update_engine_physics()
	update_clutch_behavior()
	update_transmission_logic()
	update_turbo_system()

func setup_vehicle(vehicle_node: Node3D, wheel_radius: float) -> void:
	_vehicle_body = vehicle_node
	_wheel_radius = wheel_radius
	_is_connected = true
	print_debug("Powertrain connected to vehicle body")

func disconnect_vehicle() -> void:
	_vehicle_body = null
	_is_connected = false
	stop_engine()

func start_engine() -> void:
	is_engine_running = true
	current_rpm = idle_rpm
	target_rpm = idle_rpm
	emit_signal("rpm_changed", current_rpm)
	apply_oil_pressure()

func stop_engine() -> void:
	is_engine_running = false
	current_rpm = 0.0
	target_rpm = 0.0
	oil_pressure = 0.0
	engine_temperature = 20.0

func set_gear(gear: int) -> void:
	if not is_engine_running and gear != 0:
		push_warning("Cannot shift while engine is off")
		return
	
	if gear < -1 or gear > total_gears:
		push_warning("Invalid gear requested")
		return
	
	var previous_gear = current_gear
	current_gear = gear
	
	if current_gear != previous_gear:
		emit_signal("gear_changed", current_gear)
		if current_gear == 0:
			clutch_pedal_position = 1.0

func set_throttle(input_value: float) -> void:
	throttle_input = clamp(input_value, 0.0, 1.0)

func set_brake(input_value: float) -> void:
	brake_input = clamp(input_value, 0.0, 1.0)

func set_clutch_pedal(pedal_position: float) -> void:
	clutch_pedal_position = clamp(pedal_position, 0.0, 1.0)
	
	if pedal_position < 0.1 and current_gear != 0:
		emit_signal("clutch_engaged")
	elif pedal_position > 0.9:
		emit_signal("clutch_disengaged")

func get_current_torque() -> float:
	return current_torque_output

func get_current_power_hp() -> float:
	return current_power_output_hp

func get_wheel_speed_rpm() -> float:
	if current_gear == 0:
		return 0.0
	
	var gear_ratio = get_active_gear_ratio()
	var total_ratio = gear_ratio * final_drive_ratio
	return current_rpm / total_ratio if total_ratio > 0 else 0.0

func get_linear_velocity_from_rpm() -> float:
	var wheel_rpm = get_wheel_speed_rpm()
	var wheel_circumference = PI * wheel_diameter_m
	var m_per_second = (wheel_rpm * wheel_circumference) / 60.0
	return m_per_second

func get_vehicle_speed_kmh() -> float:
	return get_linear_velocity_from_rpm() * 3.6

func calculate_torque_at_wheel() -> float:
	if current_gear == 0 or not is_engine_running:
		return 0.0
	
	var gear_ratio = get_active_gear_ratio()
	var total_ratio = gear_ratio * final_drive_ratio
	var wheel_torque = current_torque_output * total_ratio * drivetrain_efficiency
	
	# Apply differential characteristics
	if differential_type == "LSD":
		wheel_torque *= 1.1  # Slight gain from limited slip
	elif differential_type == "Locked":
		wheel_torque *= 1.2  # More gain but less traction advantage
	
	return wheel_torque

func calculate_force_on_wheels() -> float:
	var wheel_torque = get_current_torque()
	var gear_ratio = get_active_gear_ratio()
	var total_ratio = gear_ratio * final_drive_ratio
	var wheel_torque_at_ground = wheel_torque * total_ratio * drivetrain_efficiency
	
	# Force = Torque / Radius
	var force_newtons = wheel_torque_at_ground / _wheel_radius
	return force_newtons

func get_engine_braking_force() -> float:
	if current_gear == 0:
		return 0.0
	
	var gear_ratio = get_active_gear_ratio()
	var total_ratio = gear_ratio * final_drive_ratio
	# Engine braking provides resistance proportional to compression
	var engine_resistance = 50.0 + (total_ratio * 15.0)
	return engine_resistance

func is_shift_ready() -> bool:
	if current_gear >= total_gears and current_rpm >= max_rpm * 0.9:
		return true
	if current_gear <= -total_gears and current_rpm <= idle_rpm * 0.5:
		return true
	return false

func should_auto_shift_up() -> bool:
	if current_gear >= total_gears:
		return false
	if current_rpm >= max_rpm * 0.85 and not is_in_rev_limit:
		return true
	return false

func should_auto_shift_down() -> bool:
	if current_gear <= 1:
		return false
	if current_rpm <= idle_rpm * 1.2 and throttle_input < 0.1:
		return true
	return false

func get_optimal_shift_rpm() -> float:
	# Find RPM where next gear gives best acceleration
	var current_power = calculate_power_output(current_rpm)
	var optimal_rpm = current_rpm
	
	for gear_check in range(1, total_gears + 1):
		if gear_check == current_gear:
			continue
		
		var gear_ratio = gear_ratios[gear_check - 1] if gear_check <= total_gears else 0.0
		var total_ratio = gear_ratio * final_drive_ratio
		var estimated_wheel_rpm = current_rpm / total_ratio
		var estimated_next_rpm = estimated_wheel_rpm * (gear_ratios[current_gear] if current_gear > 0 else 1.0) * final_drive_ratio
		
		if estimated_next_rpm > 0 and estimated_next_rpm < max_rpm:
			var next_power = calculate_power_output(estimated_next_rpm)
			if next_power > current_power:
				optimal_rpm = estimated_next_rpm
	
	return optimal_rpm

func update_engine_physics() -> void:
	if not is_engine_running:
		current_rpm = lerp(current_rpm, 0.0, 0.1)
		current_torque_output = 0.0
		current_power_output_hp = 0.0
		return
	
	# Calculate target RPM based on throttle and vehicle speed
	var target_rpm_base = calculate_target_rpm()
	
	# Apply RPM inertia (engine can't change RPM instantly)
	var rpm_change_rate = 1500.0 * throttle_input + 800.0 * (1.0 - throttle_input)
	if current_rpm < target_rpm_base:
		target_rpm_base = min(target_rpm_base, current_rpm + rpm_change_rate)
	else:
		target_rpm_base = max(target_rpm_base, current_rpm - rpm_change_rate * 2.0)
	
	# Rev limiter behavior
	if current_rpm >= redline_rpm and throttle_input > 0.1:
		is_in_rev_limit = true
		target_rpm_base = redline_rpm
	else:
		is_in_rev_limit = false
	
	target_rpm = lerp(target_rpm, target_rpm_base, 0.1)
	current_rpm = lerp(current_rpm, target_rpm, 0.2)
	
	# Update RPM signal
	if abs(current_rpm - target_rpm) > 50:
		emit_signal("rpm_changed", current_rpm)
	
	# Calculate torque output based on RPM and throttle
	current_torque_output = calculate_torque_output()
	
	# Calculate power output (HP = Torque * RPM / 5252)
	current_power_output_hp = (current_torque_output * current_rpm) / 5252.0
	
	# Engine temperature simulation
	if current_rpm > idle_rpm * 2:
		engine_temperature += 0.05 * throttle_input
	else:
		engine_temperature -= 0.02 * (1.0 - throttle_input)
	engine_temperature = clamp(engine_temperature, 20.0, 120.0)
	
	# Oil pressure simulation
	if current_rpm > idle_rpm:
		oil_pressure = 0.3 + (current_rpm / max_rpm) * 0.5
	else:
		oil_pressure = 0.1
	oil_pressure = clamp(oil_pressure, 0.0, 0.8)

func calculate_target_rpm() -> float:
	if current_gear == 0:
		return idle_rpm
	
	# Target RPM depends on throttle input and vehicle speed
	var base_target = idle_rpm + (throttle_input * (max_rpm - idle_rpm))
	
	# If we're already above target due to momentum, slow down
	if current_gear > 0 and _vehicle_body:
		var vehicle_velocity = _vehicle_body.global_linear_velocity.length()
		var wheel_rpm = vehicle_velocity / (_wheel_radius * 2 * PI) * 60.0
		var gear_ratio = get_active_gear_ratio()
		var total_ratio = gear_ratio * final_drive_ratio
		var engine_rpm_from_speed = wheel_rpm * total_ratio
		
		if engine_rpm_from_speed > base_target and throttle_input < 0.2:
			base_target = engine_rpm_from_speed
	
	return base_target

func calculate_torque_output() -> float:
	# Torque curve based on RPM position
	var rpm_percentage = (current_rpm - idle_rpm) / (max_rpm - idle_rpm)
	rpm_percentage = clamp(rpm_percentage, 0.0, 1.0)
	
	# Base torque curve (bell-shaped around max torque RPM)
	var max_torque_rpm_pct = (max_torque_rpm - idle_rpm) / (max_rpm - idle_rpm)
	var distance_from_max = abs(rpm_percentage - max_torque_rpm_pct)
	var torque_factor = 1.0 - (distance_from_max * 1.5)
	torque_factor = clamp(torque_factor, 0.3, 1.0)
	
	# Apply throttle
	var throttle_factor = pow(throttle_input, throttle_response_curve)
	var torque = max_torque_nm * torque_factor * throttle_factor
	
	# Add turbo boost if enabled
	if turbo_enabled:
		torque *= (1.0 + (turbo_boost_psi / 14.0) * 0.5)
	
	# Apply engine braking when coasting
	if throttle_input < 0.05 and current_rpm > idle_rpm:
		torque *= 0.3
	
	return torque

func calculate_power_output(rpm: float) -> float:
	var rpm_pct = (rpm - idle_rpm) / (max_rpm - idle_rpm)
	rpm_pct = clamp(rpm_pct, 0.0, 1.0)
	
	var max_torque_rpm_pct = (max_torque_rpm - idle_rpm) / (max_rpm - idle_rpm)
	var distance_from_max = abs(rpm_pct - max_torque_rpm_pct)
	var torque_factor = 1.0 - (distance_from_max * 1.5)
	torque_factor = clamp(torque_factor, 0.3, 1.0)
	
	var torque = max_torque_nm * torque_factor
	return (torque * rpm) / 5252.0

func get_active_gear_ratio() -> float:
	if current_gear == 0:
		return 0.0
	elif current_gear == -1:
		return reverse_ratio
	else:
		return gear_ratios[current_gear - 1] if current_gear <= total_gears else 1.0

func update_clutch_behavior() -> void:
	if clutch_pedal_position > 0.8:
		# Clutch disengaged - no power transfer
		pass
	else:
		# Partial engagement smoothness
		var engagement_progress = 1.0 - clutch_pedal_position
		engagement_progress = pow(engagement_progress, 2.0)  # Non-linear for feel
		current_torque_output *= engagement_progress

func update_transmission_logic() -> void:
	if transmission_type == "Automatic":
		handle_automatic_transmission()
	elif transmission_type == "Sequential":
		handle_sequential_logic()

func handle_automatic_transmission() -> void:
	if current_gear == 0 and throttle_input > 0.2:
		set_gear(1)
		return
	
	if should_auto_shift_up() and not is_in_rev_limit:
		var next_gear = current_gear + 1
		if next_gear <= total_gears:
			set_gear(next_gear)
	elif should_auto_shift_down() and current_gear > 1:
		set_gear(current_gear - 1)

func handle_sequential_logic() -> void:
	# Sequential transmission requires explicit up/down signals
	pass

func update_turbo_system() -> void:
	if not turbo_enabled:
		turbo_spool = 0.0
		turbo_boost_psi = 0.0
		return
	
	# Turbo spools based on exhaust flow (RPM + throttle)
	var exhaust_flow = (current_rpm / max_rpm) * throttle_input
	var target_spool = 1.0 if exhaust_flow > 0.5 else 0.0
	turbo_spool = lerp(turbo_spool, target_spool, turbo_inertia)
	
	# Boost builds as turbo spools
	var target_boost = boost_target_psi * turbo_spool
	
	# Wastegate opens at high boost to prevent overboost
	if turbo_boost_psi > boost_target_psi * 0.9:
		wastegate_open = true
	else:
		wastegate_open = false
	
	turbo_boost_psi = lerp(turbo_boost_psi, target_boost, 0.05)

func apply_oil_pressure() -> void:
	oil_pressure = 0.3 + (current_rpm / max_rpm) * 0.5

func print_debug(message: String) -> void:
	if GameManager.current_state == GameManager.GameState.DEBUG_MODE:
		print("[Powertrain]", message)

func reset_runtime_state() -> void:
	current_rpm = idle_rpm
	target_rpm = idle_rpm
	current_gear = 0
	clutch_pedal_position = 1.0
	throttle_input = 0.0
	brake_input = 0.0
	is_engine_running = false
	engine_temperature = 90.0
	oil_pressure = 0.0
	is_in_rev_limit = false
	current_torque_output = 0.0
	current_power_output_hp = 0.0
	turbo_spool = 0.0
	turbo_boost_psi = 0.0
	wastegate_open = false

func serialize() -> Dictionary:
	return {
		"current_rpm": current_rpm,
		"current_gear": current_gear,
		"throttle_input": throttle_input,
		"brake_input": brake_input,
		"clutch_pedal_position": clutch_pedal_position,
		"is_engine_running": is_engine_running,
		"engine_temperature": engine_temperature,
		"oil_pressure": oil_pressure,
		"is_in_rev_limit": is_in_rev_limit,
		"current_torque_output": current_torque_output,
		"current_power_output_hp": current_power_output_hp,
		"turbo_spool": turbo_spool,
		"turbo_boost_psi": turbo_boost_psi
	}

func deserialize(data: Dictionary) -> void:
	current_rpm = data.get("current_rpm", idle_rpm)
	current_gear = data.get("current_gear", 0)
	throttle_input = data.get("throttle_input", 0.0)
	brake_input = data.get("brake_input", 0.0)
	clutch_pedal_position = data.get("clutch_pedal_position", 1.0)
	is_engine_running = data.get("is_engine_running", false)
	engine_temperature = data.get("engine_temperature", 90.0)
	oil_pressure = data.get("oil_pressure", 0.0)
	is_in_rev_limit = data.get("is_in_rev_limit", false)
	current_torque_output = data.get("current_torque_output", 0.0)
	current_power_output_hp = data.get("current_power_output_hp", 0.0)
	turbo_spool = data.get("turbo_spool", 0.0)
	turbo_boost_psi = data.get("turbo_boost_psi", 0.0)

func get_performance_metrics() -> Dictionary:
	return {
		"speed_kmh": get_vehicle_speed_kmh(),
		"rpm": current_rpm,
		"gear": current_gear,
		"torque_nm": current_torque_output,
		"power_hp": current_power_output_hp,
		"throttle_percent": throttle_input * 100,
		"brake_percent": brake_input * 100,
		"clutch_percent": (1.0 - clutch_pedal_position) * 100,
		"engine_temp_celsius": engine_temperature,
		"oil_pressure_bar": oil_pressure,
		"boost_psi": turbo_boost_psi if turbo_enabled else 0.0,
		"rev_limiter_active": is_in_rev_limit
	}

func simulate_crash_impact(force_vector: Vector3) -> void:
	# Simulate impact effects on powertrain
	if is_engine_running and force_vector.length() > 5000.0:
		# Severe impact could stall engine or damage components
		var damage_chance = randf()
		if damage_chance < 0.3:
			stop_engine()
			print_debug("Engine stalled due to crash impact")
		elif damage_chance < 0.5:
			engine_temperature += 30.0
			print_debug("Engine overheated after crash")

func add_weight(weight_kg: float) -> void:
	# Dynamic weight adjustment (fuel consumption, cargo)
	pass

func remove_weight(weight_kg: float) -> void:
	# Dynamic weight removal
	pass

func test_mode_activate() -> void:
	# Debug/testing mode for rapid testing
	max_rpm = 15000.0
	redline_rpm = 12000.0
	throttle_response_curve = 1.0
	turbo_enabled = true
	boost_target_psi = 30.0

func _notification(what: int) -> void:
	match what:
		NOTIFICATION_PREDELETE:
			disconnect_vehicle()
</FILE_BLOCK_END>>