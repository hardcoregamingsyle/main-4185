extends CharacterBody3D
class_name VehicleController

## VehicleController - Core vehicle physics controller for racing simulator
## Implements throttle, brake, steering inputs, wheel forces, and gear shifting logic
## Uses PhysicsSettings constants for centralized tuning
## Copyright 2026 Thalamus Racing Simulator Project

# ============================================================================
# SIGNALS
# ============================================================================
signal speed_changed(new_speed: float)
signal rpm_changed(new_rpm: float)
signal gear_changed(old_gear: int, new_gear: int)
signal throttle_applied(amount: float)
signal brake_applied(amount: float)
signal steering_angle_changed(angle: float)
signal skidding(is_skidding: bool)
signal collision_detected(collision_info: Dictionary)
signal engine_stalled()
signal handbrake_toggled(is_active: bool)
signal traction_control_state_changed(active: bool)
signal anti_lock_braking_state_changed(active: bool)
signal drift_started(drift_angle: float)
signal drift_ended()

# ============================================================================
# CONFIGURATION CONSTANTS
# ============================================================================
const MAX_SPEED_KMH: float = 350.0
const ACCELERATION_POWER: float = 20000.0
const BRAKING_FORCE: float = 40000.0
const STEERING_SPEED: float = 2.5
const MAX_STEERING_ANGLE: float = 35.0 * TAU / 180.0
const MIN_GEAR: int = -1  # Reverse
const MAX_GEAR: int = 6
const NEUTRAL_GEAR: int = 0
const IDLE_RPM: float = 800.0
const REDLINE_RPM: float = 8000.0
const CLUTCH_RELEASE_TIME: float = 0.3
const DIFFERENTIAL_LOCK_RATIO: float = 0.8
const DRIFT_FACTOR: float = 0.15
const TRACTION_CONTROL_THRESHOLD: float = 0.15
const ABS_THRESHOLD: float = 0.1
const SHIFT_POINT_RPM: float = 7000.0
const SHUTDOWN_RPM: float = 9000.0
const GEAR_SHIFT_DELAY: float = 0.1
const ENGINE_REVO_DROP_ON_DOWNSHIFT: float = 2000.0

# ============================================================================
# STATE VARIABLES
# ============================================================================
var _speed_kmh: float = 0.0
var _rpm: float = IDLE_RPM
var _current_gear: int = NEUTRAL_GEAR
var _target_gear: int = NEUTRAL_GEAR
var _throttle_input: float = 0.0
var _brake_input: float = 0.0
var _steering_input: float = 0.0
var _handbrake_input: float = 0.0

var _is_engine_running: bool = false
var _clutch_pedal_pressed: bool = false
var _clutch_release_timer: float = 0.0
var _engine_temperature: float = 90.0  # Celsius
var _engine_health: float = 100.0
var _fuel_level: float = 100.0
var _fuel_consumption_rate: float = 0.05  # liters per second at high RPM

# Gear-specific ratios (final drive included)
var _gear_ratios: Array[float] = [
    4.5,   # Reverse
    3.8,   # Neutral
    3.5,   # 1st
    2.5,   # 2nd
    1.8,   # 3rd
    1.4,   # 4th
    1.1,   # 5th
    0.9,   # 6th
]

# Tire grip values (0-1, affects acceleration and cornering)
var _tire_grip_front: float = 0.95
var _tire_grip_rear: float = 0.92

# Drift state tracking
var _drift_angle: float = 0.0
var _drift_velocity: Vector3 = Vector3.ZERO
var _is_drifting: bool = false
var _drift_threshold: float = 0.3

# Traction control and ABS state
var _traction_control_enabled: bool = true
var _abs_enabled: bool = true

# Skid detection
var _is_skidding: bool = false
var _wheel_slip_ratio: float = 0.0

# Collision history
var _last_collision_time: float = 0.0
var _collision_force: float = 0.0

# Time tracking
var _time_since_last_shift: float = 0.0
var _total_distance: float = 0.0
var _race_time: float = 0.0

# References to child nodes
var _powertrain_node: Node = null
var _visual_mesh: MeshInstance3D = null
var _wheel_colliders: Array[Node3D] = []

# ============================================================================
# PUBLIC PROPERTIES
# ============================================================================
func get_speed_kmh() -> float:
    return _speed_kmh

func get_rpm() -> float:
    return _rpm

func get_current_gear() -> int:
    return _current_gear

func get_throttle_input() -> float:
    return _throttle_input

func get_brake_input() -> float:
    return _brake_input

func get_steering_input() -> float:
    return _steering_input

func is_engine_running() -> bool:
    return _is_engine_running

func is_skidding() -> bool:
    return _is_skidding

func get_fuel_level() -> float:
    return _fuel_level

func get_engine_temperature() -> float:
    return _engine_temperature

func get_total_distance() -> float:
    return _total_distance

func get_race_time() -> float:
    return _race_time

# ============================================================================
# INITIALIZATION
# ============================================================================
func _ready() -> void:
    _init_references()
    _connect_signals_to_powertrain()
    _setup_wheel_colliders()
    _is_engine_running = true
    _rpm = IDLE_RPM

func _init_references() -> void:
    ## Find powertrain node if it exists as a child
    var powertrain_children = get_children()
    for child in powertrain_children:
        if child is Powertrain:
            _powertrain_node = child
            break
    
    ## Find visual mesh
    for child in get_children():
        if child is MeshInstance3D or child.name.begins_with("Visual"):
            _visual_mesh = child
            break
    
    ## Initialize wheel colliders array
    _wheel_colliders.resize(4)

func _connect_signals_to_powertrain() -> void:
    if _powertrain_node != null:
        # Connect any signals from powertrain if it has them
        pass

func _setup_wheel_colliders() -> void:
    """Initialize wheel collider references for proper force application"""
    # Front Left, Front Right, Rear Left, Rear Right
    _wheel_colliders[0] = get_node_or_null("Wheel_FL")
    _wheel_colliders[1] = get_node_or_null("Wheel_FR")
    _wheel_colliders[2] = get_node_or_null("Wheel_RL")
    _wheel_colliders[3] = get_node_or_null("Wheel_RR")

# ============================================================================
# INPUT HANDLING
# ============================================================================
func _physics_process(delta: float) -> void:
    _handle_physics_input(delta)
    _update_vehicle_physics(delta)
    _handle_gear_shifting(delta)
    _update_engine_simulation(delta)
    _update_drift_state(delta)
    _check_collision_events(delta)
    _apply_forces_to_body()

func _handle_physics_input(delta: float) -> void:
    """Process input from InputManager and update internal state"""
    
    # Get input values (normalized -1 to 1)
    _throttle_input = clamp(InputManager.get_axis("throttle", "gas"), 0.0, 1.0)
    _brake_input = clamp(InputManager.get_axis("brake", "brake"), 0.0, 1.0)
    _handbrake_input = clamp(InputManager.get_axis("handbrake", "handbrake"), 0.0, 1.0)
    _steering_input = clamp(InputManager.get_axis("left", "right"), -1.0, 1.0)
    
    # Clamp steering to maximum angle
    var target_steering_angle: float = _steering_input * MAX_STEERING_ANGLE
    _steering_input = lerp(_steering_input, target_steering_angle / MAX_STEERING_ANGLE, delta * STEERING_SPEED)
    
    # Update signals
    throttle_applied.emit(_throttle_input)
    brake_applied.emit(_brake_input)
    steering_angle_changed.emit(_steering_input * MAX_STEERING_ANGLE)
    
    # Handbrake signal
    if _handbrake_input > 0.5:
        handbrake_toggled.emit(true)
    else:
        handbrake_toggled.emit(false)

# ============================================================================
# VEHICLE PHYSICS UPDATE
# ============================================================================
func _update_vehicle_physics(delta: float) -> void:
    """Update vehicle velocity and position based on current forces"""
    
    # Calculate forward direction
    var forward_dir = global_transform.basis.z.rotated(Vector3.UP, PI).normalized()
    var right_dir = global_transform.basis.x.normalized()
    
    # Current speed magnitude
    var current_speed = linear_velocity.length()
    
    # Apply acceleration/braking based on gear and throttle/brake input
    var gear_effective_ratio: float = _get_gear_ratio(_current_gear)
    var effective_force: float = 0.0
    
    if _current_gear == NEUTRAL_GEAR:
        # Coasting - minimal resistance
        effective_force = -500.0  # Aerodynamic drag
    elif _current_gear > 0:
        # Forward gears
        if _throttle_input > 0:
            # Acceleration
            var max_speed_in_this_gear: float = _get_max_speed_for_gear(_current_gear)
            var speed_limit_factor = min(current_speed / max_speed_in_this_gear, 1.0)
            effective_force = ACCELERATION_POWER * _throttle_input * (1.0 - speed_limit_factor) * gear_effective_ratio
            
            # Engine braking when throttle released
            if _throttle_input < 0.1:
                effective_force -= ACCELERATION_POWER * 0.3 * gear_effective_ratio
        elif _brake_input > 0:
            # Braking
            effective_force = -BRAKING_FORCE * _brake_input
            if _abs_enabled:
                effective_force *= _calculate_abs_factor(current_speed)
    
    elif _current_gear == MIN_GEAR:
        # Reverse gear
        effective_force = -ACCELERATION_POWER * _throttle_input * 0.5 * gear_effective_ratio
    
    # Apply aerodynamic drag
    var air_density: float = 1.225  # kg/m³ at sea level
    var drag_coefficient: float = 0.30  # Typical sports car
    var frontal_area: float = 2.2  # m²
    var drag_force: float = 0.5 * air_density * drag_coefficient * frontal_area * current_speed * current_speed
    effective_force -= drag_force
    
    # Apply rolling resistance
    var rolling_resistance: float = 0.015 * PhysicsSettings.default_vehicle_mass * 9.81
    effective_force -= rolling_resistance
    
    # Apply handbrake effect (affects rear wheels primarily)
    if _handbrake_input > 0:
        effective_force -= BRAKING_FORCE * _handbrake_input * 0.5
    
    # Calculate acceleration
    var mass: float = PhysicsSettings.default_vehicle_mass
    var acceleration: float = effective_force / mass
    
    # Update velocity
    var new_acceleration = Vector3(0, 0, acceleration)
    linear_velocity += new_acceleration * delta
    
    # Clamp speed to maximum
    var current_speed_mag = linear_velocity.length()
    if current_speed_mag > MAX_SPEED_KMH * 1000.0 / 3600.0:
        linear_velocity = linear_velocity.normalized() * MAX_SPEED_KMH * 1000.0 / 3600.0
    
    # Update position
    position += linear_velocity * delta
    
    # Store total distance traveled
    _total_distance += current_speed_mag * delta
    
    # Update speed signal
    var speed_mps = current_speed_mag
    var speed_kmh = speed_mps * 3.6
    if abs(speed_kmh - _speed_kmh) > 0.1:
        _speed_kmh = speed_kmh
        speed_changed.emit(_speed_kmh)

# ============================================================================
# GEAR SHIFTING LOGIC
# ============================================================================
func _handle_gear_shifting(delta: float) -> void:
    """Handle automatic and manual gear shifting logic"""
    
    _time_since_last_shift += delta
    
    # Check for automatic upshifts
    if _current_gear >= MIN_GEAR and _current_gear < MAX_GEAR:
        if _rpm >= SHIFT_POINT_RPM and _throttle_input > 0.1:
            _request_gear_shift(_current_gear + 1)
    
    # Check for automatic downshifts
    elif _current_gear > NEUTRAL_GEAR:
        if _rpm <= IDLE_RPM + 500 and _throttle_input > 0.1:
            _request_gear_shift(_current_gear - 1)
    
    # Manual downshift on excessive rev drop
    elif _rpm < IDLE_RPM and _current_gear > NEUTRAL_GEAR:
        _request_gear_shift(NEUTRAL_GEAR)
    
    # Engine stall protection
    if _rpm < IDLE_RPM * 0.5 and _current_gear != NEUTRAL_GEAR:
        _rpm = IDLE_RPM
        _request_gear_shift(NEUTRAL_GEAR)
        engine_stalled.emit()

func _request_gear_shift(target_gear: int) -> void:
    """Request a gear shift (called by automatic logic or manual input)"""
    
    if _time_since_last_shift < GEAR_SHIFT_DELAY:
        return
    
    if target_gear < MIN_GEAR or target_gear > MAX_GEAR:
        return
    
    if target_gear == _current_gear:
        return
    
    # Trigger shift sequence
    _target_gear = target_gear
    _clutch_pedal_pressed = true
    _clutch_release_timer = CLUTCH_RELEASE_TIME
    
    # Emit signal after shift completes
    var old_gear = _current_gear
    await get_tree().create_timer(GEAR_SHIFT_DELAY).timeout
    _current_gear = target_gear
    gear_changed.emit(old_gear, target_gear)
    
    # Adjust RPM based on gear ratio change
    var old_ratio: float = _get_gear_ratio(old_gear)
    var new_ratio: float = _get_gear_ratio(target_gear)
    var ratio_change: float = old_ratio / new_ratio
    
    if target_gear < old_gear:
        # Downshift - engine speed increases
        _rpm = _rpm * ratio_change
    else:
        # Upshift - engine speed decreases
        _rpm = _rpm / ratio_change
        _rpm -= ENGINE_REVO_DROP_ON_DOWNSHIFT
    
    _clamp_rpm()
    _clutch_pedal_pressed = false

func _get_gear_ratio(gear: int) -> float:
    """Get the effective gear ratio for a given gear"""
    if gear < 0 or gear >= gear_ratios.size():
        return 1.0
    return _gear_ratios[gear]

# ============================================================================
# ENGINE SIMULATION
# ============================================================================
func _update_engine_simulation(delta: float) -> void:
    """Simulate engine behavior including RPM, temperature, and health"""
    
    if not _is_engine_running:
        _rpm = IDLE_RPM
        return
    
    # Calculate theoretical RPM based on vehicle speed and gear
    var gear_ratio: float = _get_gear_ratio(_current_gear)
    var wheel_radius: float = 0.33  # meters
    var final_drive: float = 3.5
    
    if _current_gear != NEUTRAL_GEAR and _current_gear != MIN_GEAR:
        var wheel_rpm = (_speed_kmh * 1000.0 / 3600.0) / (2.0 * PI * wheel_radius) * 60.0
        var theoretical_rpm = wheel_rpm * gear_ratio * final_drive
        
        # Blend theoretical RPM with current RPM based on throttle
        var target_rpm: float = 0.0
        if _throttle_input > 0:
            target_rpm = theoretical_rpm * 1.2  # Slight over-rev under acceleration
        else:
            target_rpm = theoretical_rpm * 0.8
        
        # Smooth RPM transition
        _rpm = lerp(_rpm, target_rpm, delta * 10.0)
    else:
        # Neutral or reverse - idle or simple calculation
        if _throttle_input > 0:
            _rpm = lerp(_rpm, IDLE_RPM + _throttle_input * (REDLINE_RPM - IDLE_RPM), delta * 5.0)
        else:
            _rpm = lerp(_rpm, IDLE_RPM, delta * 2.0)
    
    # Update fuel consumption
    if _is_engine_running:
        var fuel_usage: float = _fuel_consumption_rate * (_rpm / REDLINE_RPM) * (1.0 + _throttle_input)
        _fuel_level -= fuel_usage * delta
        _fuel_level = max(_fuel_level, 0.0)
    
    # Update engine temperature
    var temp_increase: float = _rpm * _throttle_input * 0.01
    var temp_decrease: float = 0.01 if _speed_kmh > 20 else 0.0
    _engine_temperature += (temp_increase - temp_decrease) * delta
    _engine_temperature = clamp(_engine_temperature, 80.0, 120.0)
    
    # Overheat penalty
    if _engine_temperature > 110.0:
        _engine_health -= 0.1 * delta
        _rpm *= 0.99
    
    # RPM limit protection
    _clamp_rpm()
    
    # Emit RPM signal
    rpm_changed.emit(_rpm)

func _clamp_rpm() -> void:
    """Clamp RPM within safe operating range"""
    if _rpm > SHUTDOWN_RPM:
        _rpm = SHUTDOWN_RPM
        # Redline protection
        _throttle_input = 0.0
    elif _rpm < IDLE_RPM * 0.3:
        _rpm = IDLE_RPM * 0.3

# ============================================================================
# DRIFT AND SKID DETECTION
# ============================================================================
func _update_drift_state(delta: float) -> void:
    """Calculate drift angle and detect skidding conditions"""
    
    # Calculate lateral velocity component
    var lateral_velocity = linear_velocity.dot(global_transform.basis.x)
    var longitudinal_velocity = linear_velocity.dot(global_transform.basis.z)
    
    # Calculate drift angle
    if longitudinal_velocity.abs() > 0.1:
        _drift_angle = atan2(lateral_velocity, longitudinal_velocity)
    else:
        _drift_angle = 0.0
    
    # Detect drifting condition
    var drift_intensity = abs(_drift_angle) * (1.0 + _handbrake_input)
    _is_drifting = drift_intensity > _drift_threshold
    
    # Detect skidding (wheel slip)
    var wheel_slip = _calculate_wheel_slip()
    _wheel_slip_ratio = wheel_slip
    _is_skidding = wheel_slip > TRACTION_CONTROL_THRESHOLD
    
    # Emit drift/skid signals
    if _is_skidding != _is_skidding:
        skidding.emit(_is_skidding)
    
    if _is_drifting and not _drift_angle > 0.1:
        drift_started.emit(_drift_angle)
    elif not _is_drifting and _drift_angle.abs() < 0.05:
        drift_ended.emit()

func _calculate_wheel_slip() -> float:
    """Calculate wheel slip ratio for traction control"""
    var wheel_speed: float = (_speed_kmh * 1000.0 / 3600.0) / (2.0 * PI * 0.33)
    var drive_wheel_speed: float = _rpm / 60.0 * _get_gear_ratio(_current_gear) * 3.5
    
    if wheel_speed > 0:
        return abs(drive_wheel_speed - wheel_speed) / wheel_speed
    return 0.0

# ============================================================================
# COLLISION HANDLING
# ============================================================================
func _check_collision_events(delta: float) -> void:
    """Monitor and respond to collision events"""
    
    if is_on_floor():
        return
    
    # Simple collision detection using velocity changes
    var previous_velocity = linear_velocity
    # Note: In production, use actual collision callbacks from body_entered signal
    
    # Check for significant impact
    var velocity_change = linear_velocity.length() - previous_velocity.length()
    if velocity_change < -5.0:
        _collision_force = abs(velocity_change)
        _last_collision_time = get_time()
        
        # Apply damage based on impact force
        _engine_health -= min(_collision_force / 100.0, 10.0)
        
        # Emit collision signal
        collision_detected.emit({
            "force": _collision_force,
            "timestamp": _last_collision_time,
            "vehicle_health": _engine_health
        })

func _on_collision_entered(body: Node) -> void:
    """Handle collision with other objects"""
    
    var impact_force: float = linear_velocity.length() * 100.0
    _collision_force = impact_force
    _last_collision_time = get_time()
    
    # Visual feedback
    if _visual_mesh != null:
        _flash_visual_impact()
    
    # Audio feedback
    AudioManager.play_sfx("collision_impact", {
        "volume": min(impact_force / 1000.0, 1.0),
        "pitch": 1.0 + randf_range(-0.2, 0.2)
    })
    
    # Update vehicle health
    _engine_health -= impact_force / 500.0
    
    # Emit signal
    collision_detected.emit({
        "body_id": body.get_instance_id(),
        "force": impact_force,
        "timestamp": _last_collision_time
    })

func _flash_visual_impact() -> void:
    """Flash visual mesh to indicate impact"""
    if _visual_mesh == null:
        return
    
    var original_material = _visual_mesh.material_override
    _visual_mesh.material_override = _visual_mesh.material_override.duplicate()
    _visual_mesh.material_override.set_shader_parameter("flash_intensity", 1.0)
    
    get_tree().create_timer(0.1).timeout.connect(func():
        _visual_mesh.material_override = original_material
    )

# ============================================================================
# FORCE APPLICATION TO BODY
# ============================================================================
func _apply_forces_to_body() -> void:
    """Apply calculated forces to the physics body"""
    
    # Apply torque for steering effect
    if _steering_input != 0:
        var turn_rate: float = _steering_input * 2.0
        angular_velocity.y = lerp(angular_velocity.y, turn_rate, 0.1)
    
    # Apply suspension forces (simplified)
    # In production, use RayCast3D for proper suspension simulation
    
    # Apply friction to ground contact
    if is_on_floor():
        var friction_coefficient: float = 0.8
        var gravity_vector = Vector3(0, -PhysicsSettings.gravity, 0)
        var normal_force = gravity_vector.length() * PhysicsSettings.default_vehicle_mass
        var friction_force = friction_coefficient * normal_force
        
        # Reduce velocity slightly for realistic friction
        linear_velocity.x *= 0.99
        linear_velocity.z *= 0.99

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================
func _get_max_speed_for_gear(gear: int) -> float:
    """Get maximum speed achievable in a specific gear"""
    var base_max: float = 40.0  # km/h per gear step
    return base_max * gear * 1.5

func reset_vehicle_state() -> void:
    """Reset vehicle to initial state"""
    _speed_kmh = 0.0
    _rpm = IDLE_RPM
    _current_gear = NEUTRAL_GEAR
    _throttle_input = 0.0
    _brake_input = 0.0
    _steering_input = 0.0
    _handbrake_input = 0.0
    _fuel_level = 100.0
    _engine_temperature = 90.0
    _engine_health = 100.0
    _total_distance = 0.0
    _race_time = 0.0
    _is_drifting = false
    _is_skidding = false
    _drift_angle = 0.0

func start_engine() -> void:
    """Start the engine"""
    _is_engine_running = true
    _rpm = IDLE_RPM

func stop_engine() -> void:
    """Stop the engine"""
    _is_engine_running = false
    _rpm = IDLE_RPM

func toggle_traction_control() -> void:
    """Toggle traction control system"""
    _traction_control_enabled = !_traction_control_enabled
    traction_control_state_changed.emit(_traction_control_enabled)

func toggle_abs() -> void:
    """Toggle anti-lock braking system"""
    _abs_enabled = !_abs_enabled
    anti_lock_braking_state_changed.emit(_abs_enabled)

func _calculate_abs_factor(speed: float) -> float:
    """Calculate ABS effectiveness factor based on speed"""
    if speed < 10.0:
        return 0.5
    elif speed < 50.0:
        return 0.75
    else:
        return 1.0

</file>
<</END>>