extends RefCounted
class_name AudioManagerTest

## Unit Tests for AudioManager
## Copyright 2026 Thalamus Racing Simulator Project

var _audio_manager: AudioManager

func _before_each() -> void:
	_audio_manager = AudioManager.new()

func _after_each() -> void:
	if _audio_manager:
		_audio_manager.free()
		_audio_manager = null

## Test: Master Volume Default
func test_master_volume_default() -> void:
	assert_equal(_audio_manager.master_volume, 0.8, "Master volume should default to 0.8")

## Test: SFX Volume Default
func test_sfx_volume_default() -> void:
	assert_equal(_audio_manager.sfx_volume, 0.9, "SFX volume should default to 0.9")

## Test: Music Volume Default
func test_music_volume_default() -> void:
	assert_equal(_audio_manager.music_volume, 0.6, "Music volume should default to 0.6")

## Test: Environment Volume Default
func test_environment_volume_default() -> void:
	assert_equal(_audio_manager.environment_volume, 0.5, "Environment volume should default to 0.5")

## Test: UI Volume Default
func test_ui_volume_default() -> void:
	assert_equal(_audio_manager.ui_volume, 1.0, "UI volume should default to 1.0")

## Test: Volume Type Enum Exists
func test_volume_type_enum_exists() -> void:
	var music_type: AudioManager.VolumeType = AudioManager.VolumeType.MUSIC
	var sfx_type: AudioManager.VolumeType = AudioManager.VolumeType.SFX
	
	assert_true(music_type != sfx_type, "VolumeType enum values should be distinct")

## Test: SFX Library Dictionary Initialized
func test_sfx_library_initialized() -> void:
	assert_equal(_audio_manager._sfx_library.size(), 0, "SFX library should be empty initially")

## Test: Audio Streams Dictionary Initialized
func test_audio_streams_initialized() -> void:
	assert_equal(_audio_manager._audio_streams.size(), 0, "Audio streams should be empty initially")

## Test: Signals Exist and Are Connectable
func test_signals_exist() -> void:
	assert_not_null(_audio_manager.sound_played, "sound_played signal should exist")
	assert_not_null(_audio_manager.music_changed, "music_changed signal should exist")
	assert_not_null(_audio_manager.volume_changed, "volume_changed signal should exist")

## Test: Volume Bounds - Zero
func test_volume_zero_boundaries() -> void:
	_audio_manager.master_volume = 0.0
	assert_equal(_audio_manager.master_volume, 0.0, "Zero volume should be valid")

## Test: Volume Bounds - Maximum
func test_volume_maximum_boundary() -> void:
	_audio_manager.master_volume = 1.0
	assert_equal(_audio_manager.master_volume, 1.0, "Maximum volume 1.0 should be valid")

## Test: Volume Negative Rejection
func test_volume_negative_rejected() -> void:
	var original_volume = _audio_manager.master_volume
	_audio_manager.master_volume = -0.1
	# Should clamp to minimum
	assert_greater_equal(_audio_manager.master_volume, 0.0, "Volume should not go below 0")
