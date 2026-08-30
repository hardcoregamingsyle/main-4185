extends Node
class_name AudioManager

## AudioManager - Centralized audio management using Web Audio API and Godot AudioStreamPlayer
## Handles SFX synthesis, music playback, and spatial audio for 3D racing simulation

signal sound_played(sound_name: String)
signal music_changed(track_name: String)
signal volume_changed(volume_type: VolumeType, new_volume: float)

enum VolumeType {
	MUSIC,
	SFX,
	ENVIRONMENT,
	UI
}

@export_group("Audio Settings")
@export var master_volume: float = 0.8
@export var sfx_volume: float = 0.9
@export var music_volume: float = 0.6
@export var environment_volume: float = 0.5
@export var ui_volume: float = 1.0

@export_group("Sound Effects")
var _sfx_library: Dictionary = {}
var _audio_streams: Dictionary = {}
var _master_bus: int = 0

# Audio stream players
var _music_player: AudioStreamPlayer3D = null
var _sfx_players: Array[AudioStreamPlayer] = []

func _ready() -> void:
	_init_audio_system()
	_connect_signals()

func _init_audio_system() -> void:
	# Create master bus if not exists
	if not AudioServer.bus_exists("Master"):
		AudioServer.add_bus(0)
		_master_bus = AudioServer.bus_get_index("Master")
	
	# Set initial volumes
	_update_master_volume(master_volume)
	
	# Initialize sound effect library
	_build_sfx_library()

func _build_sfx_library() -> void:
	# Pre-load common racing game sounds
	_sfx_library["engine_idle"] = _synthesize_engine_sound("idle", 50.0)
	_sfx_library["engine_rev"] = _synthesize_engine_sound("rev", 120.0)
	_sfx_library["accelerate"] = _synthesize_thrust_sound(0.8)
	_sfx_library["brake"] = _synthesize_brake_sound()
	_sfx_library["drift"] = _synthesize_drift_sound()
	_sfx_library["collision_light"] = _synthesize_collision_sound(0.3)
	_sfx_library["collision_medium"] = _synthesize_collision_sound(0.7)
	_sfx_library["collision_heavy"] = _synthesize_collision_sound(1.0)
	_sfx_library["crash"] = _synthesize_crash_sound()
	_sfx_library["jump_land"] = _synthesize_land_sound()
	_sfx_library["boost"] = _synthesize_boost_sound()
	_sfx_library["turbo"] = _synthesize_turbo_sound()
	_sfx_library["ui_click"] = _synthesize_ui_sound("click")
	_sfx_library["ui_hover"] = _synthesize_ui_sound("hover")
	_sfx_library["win"] = _synthesize_win_sound()
	_sfx_library["lose"] = _synthesize_lose_sound()
	_sfx_library["lap_complete"] = _synthesize_lap_sound()
	_sfx_library["checkpoint"] = _synthesize_checkpoint_sound()

func _synthesize_engine_sound(type: String, duration: float) -> AudioStream:
	var stream = AudioStreamGenerator.new()
	stream.max_channels = 2
	
	var generator = AudioStreamPlayer.new()
	generator.stream = AudioStreamOggVorbis.new()
	generator.playback_finished.connect(func(): pass)
	add_child(generator)
	
	# For now, return a placeholder - actual synthesis would use AudioStreamPlaybackGenerate
	return null

func _synthesize_thrust_sound(intensity: float) -> AudioStream:
	return null

func _synthesize_brake_sound() -> AudioStream:
	return null

func _synthesize_drift_sound() -> AudioStream:
	return null

func _synthesize_collision_sound(severity: float) -> AudioStream:
	return null

func _synthesize_crash_sound() -> AudioStream:
	return null

func _synthesize_land_sound() -> AudioStream:
	return null

func _synthesize_boost_sound() -> AudioStream:
	return null

func _synthesize_turbo_sound() -> AudioStream:
	return null

func _synthesize_ui_sound(type: String) -> AudioStream:
	return null

func _synthesize_win_sound() -> AudioStream:
	return null

func _synthesize_lose_sound() -> AudioStream:
	return null

func _synthesize_lap_sound() -> AudioStream:
	return null

func _synthesize_checkpoint_sound() -> AudioStream:
	return null

func _connect_signals() -> void:
	pass

func play_sound(sound_name: String, position: Vector3 = Vector3.ZERO, one_shot: bool = true) -> AudioStreamPlayer:
	if not _sfx_library.has(sound_name):
		push_warning("AudioManager: Sound '%s' not found" % sound_name)
		return null
	
	var stream = _get_stream_for_sound(sound_name)
	if stream == null:
		return null
	
	var player = _create_sfx_player(position, stream, one_shot)
	player.play()
	
	sound_played.emit(sound_name)
	return player

func _create_sfx_player(position: Vector3, stream: AudioStream, one_shot: bool) -> AudioStreamPlayer:
	var player = AudioStreamPlayer.new()
	player.stream = stream
	player.volume_db = linear_to_db(sfx_volume * 2.0)
	player.bus = "SFX"
	
	if position != Vector3.ZERO:
		var spatial = SpatialAudioListener.new()
		spatial.position = position
		add_child(spatial)
		player.spatial_audio_enabled = true
		player.position_mode = AudioStreamPlayer.POSITION_MODE_3D
	
	if one_shot:
		player.one_shot = true
		player.finished.connect(_on_sfx_finished.bind(player))
	
	add_child(player)
	_sfx_players.append(player)
	
	return player

func _on_sfx_finished(player: AudioStreamPlayer) -> void:
	if player in _sfx_players:
		_sfx_players.erase(player)
		queue_free()

func get_stream_for_sound(sound_name: String) -> AudioStream:
	return _sfx_library.get(sound_name)

func play_music(track_name: String, fade_duration: float = 2.0) -> void:
	# Music implementation would load from files or generate procedurally
	music_changed.emit(track_name)

func stop_music(fade_duration: float = 2.0) -> void:
	if _music_player != null:
		_music_player.stop(fade_duration)

func set_volume(volume_type: VolumeType, new_volume: float) -> void:
	match volume_type:
		VolumeType.MUSIC:
			music_volume = clamp(new_volume, 0.0, 1.0)
		VolumeType.SFX:
			sfx_volume = clamp(new_volume, 0.0, 1.0)
		VolumeType.ENVIRONMENT:
			environment_volume = clamp(new_volume, 0.0, 1.0)
		VolumeType.UI:
			ui_volume = clamp(new_volume, 0.0, 1.0)
	
	_update_master_volume(master_volume)
	volume_changed.emit(volume_type, new_volume)

func _update_master_volume(volume: float) -> void:
	AudioServer.set_bus_volume_dsp_effect(_master_bus, linear_to_db(volume))

func pause_all() -> void:
	for player in _sfx_players:
		player.pause()

func resume_all() -> void:
	for player in _sfx_players:
		player.play()

func dispose() -> void:
	for player in _sfx_players.duplicate():
		player.queue_free()
	_sfx_players.clear()
</file>