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
		"gear_up": Key.KEY_E,
		"gear_down": Key.KEY_Q,
		"action_jump": Key.KEY_Z,
		"action_restart": Key.KEY_R,
		"action_pause": Key.KEY_ESCAPE,
		"action_debug": Key.KEY_F1,
		"action_hud": Key.KEY_TAB
	}

func _update_input_map() -> void:
	"""Update Godot input map with custom bindings"""
	InputMap.clear_actions()
	
	for action in _input_config.values():
		if not InputMap.has_action(str(action)):
			InputMap.add_action(str(action))
	
	InputMap.action_add_key("Throttle", _input_config["throttle_up"])
	InputMap.action_add_key("ThrottleReverse", _input_config["throttle_down"])
	InputMap.action_add_key("SteerLeft", _input_config["steering_left"])
	InputMap.action_add_key("SteerRight", _input_config["steering_right"])
	InputMap.action_add_key("Brake", _input_config["brake"])
	InputMap.action_add_key("GearUp", _input_config["gear_up"])
	InputMap.action_add_key("GearDown", _input_config["gear_down"])
	InputMap.action_add_key("ActionJump", _input_config["action_jump"])
	InputMap.action_add_key("ActionRestart", _input_config["action_restart"])
	InputMap.action_add_key("ActionPause", _input_config["action_pause"])
	InputMap.action_add_key("ActionDebug", _input_config["action_debug"])
	InputMap.action_add_key("ActionHud", _input_config["action_hud"])

func get_throttle() -> float:
	"""Get normalized throttle input with deadzone"""
	var raw_input: float = 0.0
	
	if Input.is_action_pressed("Throttle"):
		raw_input += 1.0
	if Input.is_action_pressed("ThrottleReverse"):
		raw_input -= 1.0
	
	# Apply deadzone
	if abs(raw_input) < throttle_deadzone:
		raw_input = 0.0
	
	# Apply sensitivity
	return clampf(raw_input * throttle_sensitivity, -1.0, 1.0)

func get_brake() -> float:
	"""Get normalized brake input with deadzone"""
	var raw_input: float = 0.0
	
	if Input.is_action_pressed("Brake"):
		raw_input = 1.0
	
	# Apply deadzone
	if abs(raw_input) < brake_deadzone:
		raw_input = 0.0
	
	# Apply sensitivity
	return clampf(raw_input * brake_sensitivity, 0.0, 1.0)

func get_steering() -> float:
	"""Get normalized steering input with deadzone"""
	var raw_input: float = 0.0
	
	if Input.is_action_pressed("SteerLeft"):
		raw_input -= 1.0
	if Input.is_action_pressed("SteerRight"):
		raw_input += 1.0
	
	# Apply deadzone
	if abs(raw_input) < steering_deadzone:
		raw_input = 0.0
	
	# Apply sensitivity
	return clampf(raw_input * steering_sensitivity, -1.0, 1.0)

func get_current_gear() -> int:
	return _current_gear

func shift_up() -> void:
	"""Shift transmission up one gear"""
	if _current_gear < 6:
		_current_gear += 1
		gear_changed.emit(_current_gear)

func shift_down() -> void:
	"""Shift transmission down one gear"""
	if _current_gear > -1:  # -1 = reverse
		_current_gear -= 1
		gear_changed.emit(_current_gear)

func trigger_action(action: Action) -> bool:
	"""Check if action was triggered this frame"""
	match action:
		Action.JUMP:
			return Input.is_action_just_pressed("ActionJump")
		Action.RESTART:
			return Input.is_action_just_pressed("ActionRestart")
		Action.PAUSE:
			return Input.is_action_just_pressed("ActionPause")
		Action.SKIP_LAP:
			return Input.is_action_just_pressed("SkipLap")
		Action.TOGGLE_DEBUG:
			return Input.is_action_just_pressed("ActionDebug")
		Action.MUTE_AUDIO:
			return Input.is_action_just_pressed("MuteAudio")
		Action.SHOW_HUD:
			return Input.is_action_just_pressed("ActionHud")
	return false

func set_input_binding(action: String, key_code: int) -> void:
	"""Set custom input binding"""
	if _input_config.has(action):
		_input_config[action] = key_code
		_update_input_map()

func get_input_binding(action: String) -> int:
	"""Get current input binding for an action"""
	if _input_config.has(action):
		return _input_config[action]
	return -1

func _process(_delta: float) -> void:
	"""Update input state each frame"""
	var old_throttle = _throttle_input
	var old_brake = _brake_input
	var old_steering = _steering_input
	
	_throttle_input = get_throttle()
	_brake_input = get_brake()
	_steering_input = get_steering()
	
	# Emit signals only when values change significantly
	if abs(_throttle_input - old_throttle) > 0.01:
		throttle_changed.emit(_throttle_input)
	if abs(_brake_input - old_brake) > 0.01:
		brake_changed.emit(_brake_input)
	if abs(_steering_input - old_steering) > 0.01:
		steering_changed.emit(_steering_input)