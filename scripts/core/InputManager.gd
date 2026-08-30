extends Node
class_name InputManager

## InputManager - Centralized input handling for vehicle controls and game actions
## Maps input actions to physical controls with deadzone support

signal throttle_changed(value: float)
signal brake_changed(value: float)
signal steering_changed(value: float)
signal gear_changed(gear: int)
signal action_triggered(action: String)

enum Action {
	JUMP,
	RESTART,
	PAUSE,
	SKIP_LAP,
	TOGGLE_DEBUG,
	MUTE_AUDIO,
	SHOW_HUD
}

@export_group("Input Sensitivity")
@export var throttle_sensitivity: float = 1.0
@export var steering_sensitivity: float = 1.0
@export var brake_sensitivity: float = 1.0

@export_group("Deadzones")
@export var throttle_deadzone: float = 0.15
@export var steering_deadzone: float = 0.1
@export var brake_deadzone: float = 0.1

# Current input values (normalized -1 to 1)
var _throttle_input: float = 0.0
var _brake_input: float = 0.0
var _steering_input: float = 0.0
var _current_gear: int = 1

# Input configuration
var _input_config: Dictionary = {}

func _ready() -> void:
	_init_default_inputs()
	_update_input_map()

func _init_default_inputs() -> void:
	"""Initialize default input mappings"""
	_input_config = {
		"throttle_up": Key.KEY_W,
		"throttle_down": Key.KEY_S,
		"steering_left": Key.KEY_A,
		"steering_right": Key.KEY_D,
		"brake": Key.KEY_SPACE,
		"handbrake": Key.KEY_SHIFT,
		"reverse": Key.KEY_Q,
		"next_gear": Key.KEY_E,
		"pause": Key.KEY_ESCAPE,
		"restart": Key.KEY_R,
		"debug": Key.KEY_Tilde,
		"mute": Key.KEY_M,
		"hud": Key.KEY_H
	}

func _update_input_map() -> void:
	"""Update Godot input map from config"""
	InputMap.clear_actions()
	
	var mapping = [
		["throttle_up", ["throttle_up"]],
		["throttle_down", ["throttle_down"]],
		["steering_left", ["steering_left"]],
		["steering_right", ["steering_right"]],
		["brake", ["brake"]],
		["handbrake", ["handbrake"]],
		["reverse", ["reverse"]],
		["next_gear", ["next_gear"]],
		["pause", ["pause"]],
		["restart", ["restart"]],
		["debug", ["debug"]],
		["mute", ["mute"]],
		["hud", ["hud"]]
	]
	
	for action in mapping:
		if not InputMap.has_action(action[0]):
			InputMap.add_action(action[0])
			InputMap.action_add_event(action[0], InputEventKey.new())
		
		var key = _input_config.get(action[0], Key.KEY_NULL)
		if key != Key.KEY_NULL:
			var event = InputEventKey.new()
			event.pressed = true
			event.keycode = key
			InputMap.action_add_event(action[0], event)

func _process(_delta: float) -> void:
	"""Process input each frame"""
	_process_vehicle_controls()

func _process_vehicle_controls() -> void:
	"""Process vehicle control inputs"""
	_calculate_throttle()
	_calculate_brake()
	_calculate_steering()
	_check_gear_changes()
	_trigger_game_actions()

func _calculate_throttle() -> void:
	"""Calculate throttle input value"""
	var up = Input.is_action_pressed("throttle_up")
	var down = Input.is_action_pressed("throttle_down")
	
	var raw_value: float = 0.0
	
	if up:
		raw_value += 1.0
	if down:
		raw_value -= 1.0
	
	# Apply deadzone
	if abs(raw_value) < throttle_deadzone:
		raw_value = 0.0
	
	# Apply sensitivity
	raw_value *= throttle_sensitivity
	
	# Clamp to valid range
	_throttle_input = clamp(raw_value, -1.0, 1.0)
	
	if _is_changed(throttle_deadzone):
		throttle_changed.emit(_throttle_input)

func _calculate_brake() -> void:
	"""Calculate brake input value"""
	var brake_pressed = Input.is_action_pressed("brake")
	var handbrake_pressed = Input.is_action_pressed("handbrake")
	
	var raw_value: float = 0.0
	
	if brake_pressed:
		raw_value += 0.5
	if handbrake_pressed:
		raw_value += 0.5
	
	# Apply deadzone
	if raw_value > brake_deadzone:
		raw_value = 1.0
	else:
		raw_value = 0.0
	
	# Apply sensitivity
	raw_value *= brake_sensitivity
	
	_brake_input = raw_value
	
	if _is_changed(brake_deadzone):
		brake_changed.emit(_brake_input)

func _calculate_steering() -> void:
	"""Calculate steering input value"""
	var left = Input.is_action_pressed("steering_left")
	var right = Input.is_action_pressed("steering_right")
	
	var raw_value: float = 0.0
	
	if left:
		raw_value -= 1.0
	if right:
		raw_value += 1.0
	
	# Apply deadzone
	if abs(raw_value) < steering_deadzone:
		raw_value = 0.0
	
	# Apply sensitivity
	raw_value *= steering_sensitivity
	
	# Clamp to valid range
	_steering_input = clamp(raw_value, -1.0, 1.0)
	
	if _is_changed(steering_deadzone):
		steering_changed.emit(_steering_input)

func _check_gear_changes() -> void:
	"""Check for gear change inputs"""
	if Input.is_action_just_pressed("next_gear"):
		_current_gear = min(_current_gear + 1, 6)
		gear_changed.emit(_current_gear)
	
	if Input.is_action_just_pressed("reverse"):
		if _current_gear == 1:
			_current_gear = -1
		elif _current_gear < 0:
			_current_gear = 1
		gear_changed.emit(_current_gear)

func _trigger_game_actions() -> void:
	"""Check for game action triggers"""
	if Input.is_action_just_pressed("pause"):
		action_triggered.emit(Action.PAUSE)
	
	if Input.is_action_just_pressed("restart"):
		action_triggered.emit(Action.RESTART)
	
	if Input.is_action_just_pressed("debug"):
		action_triggered.emit(Action.TOGGLE_DEBUG)
	
	if Input.is_action_just_pressed("mute"):
		action_triggered.emit(Action.MUTE_AUDIO)
	
	if Input.is_action_just_pressed("hud"):
		action_triggered.emit(Action.SHOW_HUD)

func _is_changed(threshold: float) -> bool:
	"""Check if input changed significantly"""
	return abs(_throttle_input) > threshold or abs(_brake_input) > threshold or abs(_steering_input) > threshold

func get_throttle_input() -> float:
	return _throttle_input

func get_brake_input() -> float:
	return _brake_input

func get_steering_input() -> float:
	return _steering_input

func get_current_gear() -> int:
	return _current_gear

func set_input_config(config: Dictionary) -> void:
	"""Set custom input configuration"""
	_input_config.merge(config, true)
	_update_input_map()

func reset_to_defaults() -> void:
	"""Reset all inputs to default configuration"""
	_init_default_inputs()
	_update_input_map()

func is_debug_active() -> bool:
	return Input.is_key_pressed(KEY_MASK_DEBUG)

func toggle_debug_mode() -> void:
	pass  # Handled by GameManager

func get_all_input_values() -> Dictionary:
	"""Get all current input values as a dictionary"""
	return {
		"throttle": _throttle_input,
		"brake": _brake_input,
		"steering": _steering_input,
		"gear": _current_gear,
		"handbrake": Input.is_action_pressed("handbrake"),
		"pause": Input.is_action_just_pressed("pause"),
		"restart": Input.is_action_just_pressed("restart")
	}