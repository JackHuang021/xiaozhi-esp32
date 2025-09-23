/**
 * @file audio_player.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-09-23
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include "audio_player.h"
#include "audio_codec.h"
#include "file_iterator.h"

class AudioPlayer {
public:
    static AudioPlayer& GetInstance() {
        static AudioPlayer instance;
        return instance;
    }

private:
    const char *music_dir_;
    audio_player_config_t player_config_;
    file_iterator_instance_t *file_iterator_ = nullptr;

private:
    static esp_err_t (mute_fn)(AUDIO_PLAYER_MUTE_SETTING setting);
    static esp_err_t (clk_set_fn)(uint32_t rate, uint32_t bits_cfg,
                      i2s_slot_mode_t ch);
    static esp_err_t (write_fn)(void *audio_buffer, size_t len,
                      size_t *bytes_written, uint32_t timeout_ms);
    static void (audio_player_cb)(audio_player_cb_ctx_t *ctx);

public:
    AudioPlayer();

    void Initialize(const char *dir);
    esp_err_t playMusic(const std::string& musicName);
};




