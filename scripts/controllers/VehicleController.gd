extends Node2D
class_name VehicleController

## VehicleController - Core vehicle physics controller using PhysicsSettings constants
## Handles throttle, brake, steering inputs, wheel forces, and gear shifting logic
## Copyright 2026 Thalamus Racing Simulator Project

signal speed_changed(new_speed: float)
signal gear_changed(old_gear: int, new_gear: int)
signal vehicle_moved(displacement: Vector2)
signal drift_angle_changed(angle: float)
signal traction_control_active(active: bool)
signal engine_rpm_changed(rpm: float)
signal collision_detected(collision_type: String, force: float)

# ============================================================================
# PHYSICS CONSTANTS - Derived from PhysicsSettings resource
# ============================================================================

const MAX_THROTTLE_FORCE: float = 15000.0      # Newtons - maximum acceleration force
const MAX_BRAKE_FORCE: float = 20000.0         # Newtons - maximum braking force
const MAX_STEERING_ANGLE: float = PI / 3       # 60 degrees max steering
const STEERING_SPEED: float = 4.0              # Radians per second steering rate
const DRIFT_THRESHOLD: float = 0.7             # Sideslip threshold for drift mode
const TRACTION_CONTROL_SENSITIVITY: float = 0.85 # TCS activation threshold
const MIN_RPM_IDLE: float = 800.0              # Idle RPM
const MAX_RPM_REDLINE: float = 7500.0          # Redline RPM
const OPTIMAL_POWER_RPM_START: float = 3500.0  # Start of power band
const OPTIMAL_POWER_RPM_END: float = 6500.0    # End of power band

# ============================================================================
# GEAR RATIOS AND TRANSMISSION CONFIGURATION
# ============================================================================

enum Gear {
    NEUTRAL = 0,
    FIRST = 1,
    SECOND = 2,
    THIRD = 3,
    FOURTH = 4,
    FIFTH = 5,
    SIXTH = 6,
    REVERSE = -1
}

const GEAR_RATIOS: Dictionary = {
    Gear.FIRST: 3.5,
    Gear.SECOND: 2.2,
    Gear.THIRD: 1.6,
    Gear.FOURTH: 1.2,
    Gear.FIFTH: 0.9,
    Gear.SIXTH: 0.75,
    Gear.REVERSE: 3.0
}

const FINAL_DRIVE_RATIO: float = 3.73          # Final drive differential ratio
const REVERSE_GEAR_RATIO: float = 3.5          # Reverse gear ratio

# ============================================================================
# VEHICLE STATE VARIABLES
# ============================================================================

@export var vehicle_mass: float = 1500.0      # kg - overridden by PhysicsSettings default
@export var center_of_gravity: Vector2 = Vector2(0.0, 0.5)  # CG position relative to chassis
@export var wheelbase: float = 2.6           # Distance between front and rear axles (meters)
@export var track_width: float = 1.6         # Distance between left and right wheels (meters)
@export var wheel_radius: float = 0.3        # Wheel radius in meters
@export var tire_friction_coefficient: float = 1.15  # Dry asphalt coefficient
@export var aerodynamic_drag_coefficient: float = 0.32  # Cd value
@export var frontal_area: float = 2.2        # m² - vehicle frontal area

var current_gear: int = Gear.NEUTRAL
var engine_rpm: float = 0.0                  # Engine revolutions per minute
var target_rpm: float = 0.0
var clutch_engaged: bool = true
var handbrake_active: bool = false
var traction_control_enabled: bool = true
var anti_lock_braking_enabled: bool = true
var drift_mode_active: bool = false

# Movement state
var velocity: Vector2 = Vector2.ZERO
var speed: float = 0.0                       # Current speed magnitude
var heading_angle: float = 0.0               # Vehicle facing direction
var angular_velocity: float = 0.0            # Rotational speed
var drift_angle: float = 0.0                 # Angle between heading and velocity vector

# Input state
var throttle_input: float = 0.0              # -1.0 to 1.0
var brake_input: float = 0.0                 # -1.0 to 1.0
var steering_input: float = 0.0              # -1.0 to 1.0 (left negative, right positive)

# Powertrain state
var engine_torque: float = 0.0               # Current torque output
var wheel_torque: float = 0.0                # Torque at wheels after gearing
var engine_braking: bool = true              # Whether engine provides braking effect

# Performance metrics
var distance_traveled: float = 0.0
var lap_time: float = 0.0
var best_lap_time: float = 0.0
var current_lap_start: float = 0.0

# Wheel states
var wheel_states: Array[Dictionary] = []
var wheel_slip_ratio: float = 0.0
var wheel_slide: bool = false

# Drift system
var drift_score: float = 0.0
var drift_combo: int = 0
var drift_duration: float = 0.0
var drift_multiplier: float = 1.0

# Reference to parent vehicle node
var _vehicle_node: Node2D = null
var _physics_settings: Resource = null

func _ready() -> void:
    _init_physics_settings()
    _init_wheel_states()
    _connect_signals_to_game_manager()
    
    # Get reference to parent vehicle node if available
    if get_parent():
        _vehicle_node = get_parent()
        
    # Initialize physics settings reference
    if GameManager and GameManager.is_connected("game_state_changed", _on_game_state_changed):
        pass
    
    print("[VehicleController] Initialized successfully")

func _init_physics_settings() -> void:
    """Initialize physics settings from global PhysicsSettings resource"""
    if GameManager:
        _physics_settings = PhysicsSettings
        
        # Override vehicle parameters with global defaults if not set
        if vehicle_mass == 1500.0:
            vehicle_mass = _physics_settings.default_vehicle_mass
            
        print(f"[VehicleController] Loaded physics settings - mass: {vehicle_mass}kg")

func _init_wheel_states() -> void:
    """Initialize wheel state tracking for each wheel"""
    wheel_states.clear()
    
    # Define wheel positions relative to vehicle center
    var wheel_positions = [
        {"name": "front_left", "offset": Vector2(-wheelbase * 0.5, -track_width * 0.5)},
        {"name": "front_right", "offset": Vector2(-wheelbase * 0.5, track_width * 0.5)},
        {"name": "rear_left", "offset": Vector2(wheelbase * 0.5, -track_width * 0.5)},
        {"name": "rear_right", "offset": Vector2(wheelbase * 0.5, track_width * 0.5)}
    ]
    
    for pos in wheel_positions:
        wheel_states.append({
            "name": pos["name"],
            "position": pos["offset"],
            "slip_ratio": 0.0,
            "force": Vector2.ZERO,
            "ground_contact": false,
            "driving_force": 0.0,
            "braking_force": 0.0,
            "steering_angle": 0.0
        })

func _input(event: InputEvent) -> void:
    """Handle direct input events"""
    if event is InputEventKey:
        _handle_key_input(event)
    elif event is InputEventMouseButton:
        _handle_mouse_input(event)

func _handle_key_input(event: InputEventKey) -> void:
    """Process keyboard input for vehicle control"""
    if event.pressed:
        if event.keycode == KEY_W or event.keycode == KEY_UP:
            throttle_input = 1.0
        elif event.keycode == KEY_S or event.keycode == KEY_DOWN:
            brake_input = 1.0
        elif event.keycode == KEY_A or event.keycode == KEY_LEFT:
            steering_input = -1.0
        elif event.keycode == KEY_D or event.keycode == KEY_RIGHT:
            steering_input = 1.0
        elif event.keycode == KEY_SPACE:
            handbrake_active = true
        elif event.keycode == KEY_SHIFT:
            shift_up()
        elif event.keycode == KEY_Z:
            shift_down()
            
    else:
        if event.keycode == KEY_W or event.keycode == KEY_UP:
            throttle_input = 0.0
        elif event.keycode == KEY_S or event.keycode == KEY_DOWN:
            brake_input = 0.0
        elif event.keycode == KEY_A or event.keycode == KEY_LEFT:
            steering_input = 0.0
        elif event.keycode == KEY_D or event.keycode == KEY_RIGHT:
            steering_input = 0.0
        elif event.keycode == KEY_SPACE:
            handbrake_active = false

func _handle_mouse_input(event: InputEventMouseButton) -> void:
    """Process mouse input for vehicle control (alternative controls)"""
    if event.button_index == MOUSE_BUTTON_LEFT and event.pressed:
        throttle_input = min(throttle_input + 0.1, 1.0)
    elif event.button_index == MOUSE_BUTTON_RIGHT and event.pressed:
        brake_input = min(brake_input + 0.1, 1.0)

func _process(delta: float) -> void:
    """Main game loop processing - called every frame"""
    _update_inputs(delta)
    _calculate_engine_state(delta)
    _update_gear_logic(delta)
    _apply_forces(delta)
    _update_drift_system(delta)
    _update_wheels(delta)
    _emit_signals()

func _update_inputs(delta: float) -> void:
    """Smoothly interpolate input values for better feel"""
    # Smooth throttle transition
    var target_throttle = clamp(throttle_input, 0.0, 1.0)
    throttle_input = lerp(throttle_input, target_throttle, delta * 10.0)
    
    # Smooth brake transition
    var target_brake = clamp(brake_input, 0.0, 1.0)
    brake_input = lerp(brake_input, target_brake, delta * 10.0)
    
    # Smooth steering transition
    var target_steering = clamp(steering_input, -1.0, 1.0)
    steering_input = lerp(steering_input, target_steering, delta * 15.0)

func _calculate_engine_state(delta: float) -> void:
    """Calculate current engine torque and RPM based on throttle and gear"""
    if not clutch_engaged:
        return
    
    # Calculate target RPM based on current speed and gear
    target_rpm = _calculate_target_rpm()
    
    # Smooth RPM transition
    engine_rpm = lerp(engine_rpm, target_rpm, delta * 10.0)
    
    # Calculate engine torque based on RPM curve
    engine_torque = _calculate_engine_torque()
    
    # Apply engine braking when throttle released
    if throttle_input < 0.1 and engine_rpm > MIN_RPM_IDLE and current_gear != Gear.NEUTRAL:
        engine_torque *= -0.3  # Mild engine braking effect
    
    # Clamp RPM to valid range
    engine_rpm = clamp(engine_rpm, MIN_RPM_IDLE, MAX_RPM_REDLINE)
    
    # Emit RPM change signal if significant change
    if abs(engine_rpm - target_rpm) > 50.0:
        engine_rpm_changed.emit(engine_rpm)

func _calculate_target_rpm() -> float:
    """Calculate target RPM based on vehicle speed and current gear"""
    if current_gear == Gear.NEUTRAL:
        return MIN_RPM_IDLE
    
    # Calculate wheel rotation speed from vehicle velocity
    var wheel_rotation_speed = 0.0
    if speed > 0.01:
        wheel_rotation_speed = speed / (wheel_radius * 2.0 * PI)  # rotations per second
        
    # Convert to RPM
    var wheel_rpm = wheel_rotation_speed * 60.0
    
    # Apply gear ratios
    var gear_ratio = GEAR_RATIOS.get(current_gear, 1.0)
    var total_ratio = gear_ratio * FINAL_DRIVE_RATIO
    
    var target = wheel_rpm * total_ratio
    
    # Add some offset for realistic feel
    return target + MIN_RPM_IDLE * 0.3

func _calculate_engine_torque() -> float:
    """Calculate engine torque output based on RPM curve"""
    # Simulate torque curve - peak around optimal power band
    var rpm_normalized = (engine_rpm - MIN_RPM_IDLE) / (MAX_RPM_REDLINE - MIN_RPM_IDLE)
    
    # Torque curve shape (bell curve centered on optimal power band)
    var torque_factor = 0.0
    if rpm_normalized >= 0.3 and rpm_normalized <= 0.75:
        # Peak torque zone
        torque_factor = 1.0
    else:
        # Drop-off at low and high RPM
        if rpm_normalized < 0.3:
            torque_factor = pow(rpm_normalized / 0.3, 2.0)
        else:
            torque_factor = pow((1.0 - rpm_normalized) / 0.25, 2.0)
    
    # Base torque multiplied by throttle input
    var base_torque = 400.0  # Nm - typical sports car peak torque
    var throttle_factor = throttle_input
    
    # Reduce torque at very high RPM (redline protection)
    if engine_rpm > MAX_RPM_REDLINE * 0.95:
        throttle_factor *= 0.5
    
    engine_torque = base_torque * torque_factor * throttle_factor
    
    return engine_torque

func _update_gear_logic(delta: float) -> void:
    """Update automatic gear shifting logic"""
    if current_gear == Gear.NEUTRAL:
        return
        
    # Automatic upshift logic
    if engine_rpm > MAX_RPM_REDLINE * 0.9 and current_gear < Gear.SIXTH:
        shift_up(false)  # Silent shift (not player-initiated)
        
    # Automatic downshift logic
    if engine_rpm < MIN_RPM_IDLE * 1.2 and current_gear > Gear.FIRST:
        shift_down(false)
        
    # Downshift on heavy braking
    if brake_input > 0.8 and engine_rpm < MIN_RPM_IDLE * 1.5:
        shift_down(false)

func shift_up(silent: bool = true) -> void:
    """Shift transmission up one gear"""
    if current_gengear == Gear.NEUTRAL:
        return
    if current_gear == Gear.SIXTH:
        return
        
    var old_gear = current_gear
    var new_gear = current_gear + 1
    
    if new_gear > Gear.SIXTH:
        new_gear = Gear.FIFTH
        
    current_gear = new_gear
    
    if not silent:
        gear_changed.emit(old_gear, new_gear)
        AudioManager.sfx_play("gear_shift")
    
    # RPM drop due to taller gear
    var rpm_drop = engine_rpm * 0.7
    engine_rpm = max(rpm_drop, MIN_RPM_IDLE)

func shift_down(silent: bool = true) -> void:
    """Shift transmission down one gear"""
    if current_gear == Gear.NEUTRAL:
        return
    if current_gear == Gear.FIRST:
        return
        
    var old_gear = current_gear
    var new_gear = current_gear - 1
    
    current_gear = new_gear
    
    if not silent:
        gear_changed.emit(old_gear, new_gear)
        AudioManager.sfx_play("gear_shift")
    
    # RPM rise due to shorter gear
    var rpm_rise = engine_rpm * 1.4
    engine_rpm = min(rpm_rise, MAX_RPM_REDLINE)

func apply_forces(delta: float) -> void:
    """Apply calculated forces to vehicle body"""
    if not _vehicle_node:
        return
        
    # Calculate total driving force from engine
    var gear_ratio = GEAR_RATIOS.get(current_gear, 1.0)
    var total_ratio = gear_ratio * FINAL_DRIVE_RATIO
    wheel_torque = engine_torque * total_ratio * 0.95  # 5% drivetrain loss
    
    # Convert torque to linear force at contact patch
    var driving_force = wheel_torque / wheel_radius
    
    # Apply aerodynamic drag
    var air_density = 1.225  # kg/m³ at sea level
    var drag_force = 0.5 * air_density * aerodynamic_drag_coefficient * frontal_area * (speed * speed)
    
    # Apply rolling resistance
    var rolling_resistance = 0.015 * vehicle_mass * 9.81
    
    # Apply friction forces to velocity
    var net_force = driving_force - drag_force - rolling_resistance
    
    # Acceleration = Force / Mass
    var acceleration = net_force / vehicle_mass
    
    # Update velocity
    velocity += Vector2.RIGHT * acceleration * delta * speed.max(0.01)
    
    # Clamp velocity to reasonable limits
    var max_speed = _calculate_max_speed()
    velocity.length = min(velocity.length, max_speed)
    
    # Calculate speed magnitude
    speed = velocity.length

func _apply_forces(delta: float) -> void:
    """Apply calculated forces to vehicle body"""
    if not _vehicle_node:
        return
        
    # Calculate total driving force from engine
    var gear_ratio = GEAR_RATIOS.get(current_gear, 1.0)
    var total_ratio = gear_ratio * FINAL_DRIVE_RATIO
    wheel_torque = engine_torque * total_ratio * 0.95  # 5% drivetrain loss
    
    # Convert torque to linear force at contact patch
    var driving_force = wheel_torque / wheel_radius
    
    # Apply aerodynamic drag
    var air_density = 1.225  # kg/m³ at sea level
    var drag_force = 0.5 * air_density * aerodynamic_drag_coefficient * frontal_area * (speed * speed)
    
    # Apply rolling resistance
    var rolling_resistance = 0.015 * vehicle_mass * 9.81
    
    # Apply friction forces to velocity
    var net_force = driving_force - drag_force - rolling_resistance
    
    # Acceleration = Force / Mass
    var acceleration = net_force / vehicle_mass
    
    # Update velocity
    velocity += Vector2.RIGHT * acceleration * delta * speed.max(0.01)
    
    # Clamp velocity to reasonable limits
    var max_speed = _calculate_max_speed()
    velocity.length = min(velocity.length, max_speed)
    
    # Calculate speed magnitude
    speed = velocity.length

func _calculate_max_speed() -> float:
    """Calculate theoretical maximum speed for current gear"""
    if current_gear == Gear.NEUTRAL:
        return 0.0
        
    var gear_ratio = GEAR_RATIOS.get(current_gear, 1.0)
    var total_ratio = gear_ratio * FINAL_DRIVE_RATIO
    
    # Max wheel RPM at redline
    var max_wheel_rpm = MAX_RPM_REDLINE / total_ratio
    
    # Convert to m/s
    var max_speed_ms = (max_wheel_rpm / 60.0) * 2.0 * PI * wheel_radius
    
    return max_speed_ms * 3.6  # Convert to km/h

func _update_drift_system(delta: float) -> void:
    """Update drift mechanics and scoring"""
    # Calculate drift angle (difference between heading and velocity direction)
    var velocity_angle = velocity.angle()
    drift_angle = abs(heading_angle - velocity_angle)
    
    # Normalize drift angle to 0-PI range
    while drift_angle > PI:
        drift_angle -= 2.0 * PI
    while drift_angle < 0:
        drift_angle += 2.0 * PI
        
    # Check drift conditions
    var is_drifting = speed > 10.0 and drift_angle > DRIFT_THRESHOLD
    
    if is_drifting:
        drift_mode_active = true
        drift_duration += delta
        drift_score += delta * speed * drift_multiplier
        drift_combo += 1
    else:
        drift_mode_active = false
        drift_duration = 0.0
        drift_combo = 0
        drift_multiplier = 1.0
    
    # Combo multiplier increases with sustained drift
    if drift_combo > 10:
        drift_multiplier = 1.0 + (drift_combo - 10) * 0.1
        drift_multiplier = min(drift_multiplier, 3.0)
        
    # Cap drift score
    drift_score = min(drift_score, 10000.0)
    
    # Emit drift angle change signal
    drift_angle_changed.emit(drift_angle)

func _update_wheels(delta: float) -> void:
    """Update individual wheel states"""
    for wheel in wheel_states:
        # Calculate wheel slip ratio
        var wheel_speed = velocity.length * (1.0 + wheel["name"].contains("rear")) * 0.5
        var desired_wheel_speed = engine_rpm * wheel_radius / 60.0 * 2.0 * PI
        
        if desired_wheel_speed > 0.01:
            wheel["slip_ratio"] = (desired_wheel_speed - wheel_speed) / max(desired_wheel_speed, 0.01)
        else:
            wheel["slip_ratio"] = 0.0
            
        # Apply traction control if enabled
        if traction_control_enabled and wheel["slip_ratio"] > TRACTION_CONTROL_SENSITIVITY:
            wheel["slip_ratio"] = lerp(wheel["slip_ratio"], TRACTION_CONTROL_SENSITIVITY, delta * 10.0)
            
        # Update wheel slide state
        wheel["slide"] = abs(wheel["slip_ratio"]) > 0.3

func _emit_signals() -> void:
    """Emit relevant signals for other systems"""
    if speed != _last_speed_value:
        speed_changed.emit(speed)
        _last_speed_value = speed
        
    if drift_mode_active:
        drift_score += 1.0 * delta

func _on_game_state_changed(new_state: GameState) -> void:
    """Handle game state changes that affect vehicle"""
    if new_state == GameState.RACE_ACTIVE:
        _reset_vehicle_state()
    elif new_state == GameState.RACE_PAUSED:
        _pause_vehicle_controls()
    elif new_state == GameState.RACE_FINISHED:
        _stop_vehicle()

func _reset_vehicle_state() -> void:
    """Reset vehicle to initial state"""
    velocity = Vector2.ZERO
    speed = 0.0
    engine_rpm = MIN_RPM_IDLE
    current_gear = Gear.FIRST
    clutch_engaged = true
    handbrake_active = false
    drift_mode_active = false
    drift_score = 0.0
    distance_traveled = 0.0

func _pause_vehicle_controls() -> void:
    """Pause vehicle input processing"""
    throttle_input = 0.0
    brake_input = 0.0
    steering_input = 0.0

func _stop_vehicle() -> void:
    """Bring vehicle to complete stop"""
    velocity = Vector2.ZERO
    speed = 0.0
    engine_rpm = MIN_RPM_IDLE
    current_gear = Gear.NEUTRAL

func _get_collision_info() -> Dictionary:
    """Get current collision information"""
    return {
        "in_collision": false,
        "collision_type": "",
        "collision_force": 0.0
    }

func _connect_signals_to_game_manager() -> void:
    """Connect to GameManager signals for lifecycle management"""
    if GameManager:
        GameManager.game_state_changed.connect(_on_game_state_changed)

func _set_gravity(value: float) -> void:
    """Setter for gravity property"""
    gravity = value
    if _physics_settings:
        _physics_settings.gravity = value

func _set_physics_tick_rate(value: int) -> void:
    """Setter for physics tick rate property"""
    physics_tick_rate = value

func _set_default_vehicle_mass(value: float) -> void:
    """Setter for default vehicle mass property"""
    default_vehicle_mass = value

# ============================================================================
# PUBLIC API METHODS
# ============================================================================

func reset() -> void:
    """Reset vehicle controller to initial state"""
    _reset_vehicle_state()
    throttle_input = 0.0
    brake_input = 0.0
    steering_input = 0.0
    drift_score = 0.0
    distance_traveled = 0.0

func enable_traction_control(enable: bool) -> void:
    """Enable or disable traction control"""
    traction_control_enabled = enable
    traction_control_active.emit(enable)

func toggle_handbrake() -> void:
    """Toggle handbrake state"""
    handbrake_active = not handbrake_active

func engage_clutch() -> void:
    """Engage clutch"""
    clutch_engaged = true

func disengage_clutch() -> void:
    """Disengage clutch"""
    clutch_engaged = false

func set_custom_gear(gear: int) -> void:
    """Manually set gear (for manual transmission mode)"""
    if gear in GEAR_RATIOS.keys():
        current_gear = gear
        gear_changed.emit(get_previous_gear(), gear)

func get_previous_gear() -> int:
    """Get previous gear before current shift"""
    if current_gear > Gear.FIRST and current_gear < Gear.SIXTH:
        return current_gear - 1
    elif current_gear == Gear.SIXTH:
        return Gear.FIFTH
    return Gear.FIRST

func get_current_speed_kmh() -> float:
    """Get current speed in km/h"""
    return speed * 3.6

func get_current_speed_mps() -> float:
    """Get current speed in meters per second"""
    return speed

func get_distance_traveled() -> float:
    """Get total distance traveled in meters"""
    return distance_traveled

func calculate_acceleration(force: float) -> float:
    """Calculate acceleration from force using F=ma"""
    return force / vehicle_mass

func calculate_top_speed() -> float:
    """Calculate theoretical top speed in km/h"""
    var max_gear = Gear.SIXTH
    var gear_ratio = GEAR_RATIOS[max_gear]
    var total_ratio = gear_ratio * FINAL_DRIVE_RATIO
    
    var max_wheel_rpm = MAX_RPM_REDLINE / total_ratio
    var max_speed_ms = (max_wheel_rpm / 60.0) * 2.0 * PI * wheel_radius
    
    return max_speed_ms * 3.6

func set_aerodynamics(cd: float, area: float) -> void:
    """Set aerodynamic properties"""
    aerodynamic_drag_coefficient = cd
    frontal_area = area

func update_physics_parameters(params: Dictionary) -> void:
    """Update physics parameters dynamically"""
    if params.has("mass"):
        vehicle_mass = params["mass"]
    if params.has("friction"):
        tire_friction_coefficient = params["friction"]
    if params.has("aerodynamic_cd"):
        aerodynamic_drag_coefficient = params["aerodynamic_cd"]
    if params.has("frontal_area"):
        frontal_area = params["frontal_area"]

</FILE>