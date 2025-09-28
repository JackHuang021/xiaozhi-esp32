/**
 * @file audio_player.cc
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-09-23
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <cstring>
#include <sys/stat.h>
#include <vector>
#include "player.h"
#include "esp_log.h"
#include "esp_log_level.h"
#include "mcp_server.h"


static const char *TAG = "player";

esp_err_t(AudioPlayer::mute_fn)(AUDIO_PLAYER_MUTE_SETTING setting)
{
    esp_err_t ret = ESP_OK;
    ESP_LOGI(TAG, "mute fn callback %d", setting);
    return ret;
}

esp_err_t(AudioPlayer::clk_set_fn)(uint32_t rate, uint32_t bits_cfg, i2s_slot_mode_t ch)
{
    esp_err_t ret = ESP_OK;

    auto codec = Board::GetInstance().GetAudioCodec();

    // check & set sample rate
    if (rate != codec->output_sample_rate()) {
        codec->SetOutputSampleRate(rate);
    }

    return ret;
}

esp_err_t(AudioPlayer::write_fn)(void *audio_buffer, size_t len, size_t *bytes_written, uint32_t timeout_ms)
{
    esp_err_t ret = ESP_OK;

    int16_t* ptr = static_cast<int16_t*>(audio_buffer);
    std::vector<int16_t> data(ptr, ptr + len);
    auto *codec = Board::GetInstance().GetAudioCodec();
    codec->OutputData(data);
    *bytes_written = len;

    return ret;
}

void AudioPlayer::audio_player_cb(audio_player_cb_ctx_t *ctx)
{
    ESP_LOGI(TAG, "ctx->audio_event = %d", ctx->audio_event);

    switch (ctx->audio_event) {
    case AUDIO_PLAYER_CALLBACK_EVENT_IDLE:
        ESP_LOGI(TAG, "AUDIO_PLAYER_CALLBACK_EVENT_IDLE");
        break;
    case AUDIO_PLAYER_CALLBACK_EVENT_COMPLETED_PLAYING_NEXT:
        ESP_LOGI(TAG, "AUDIO_PLAYER_CALLBACK_EVENT_COMPLETED_PLAYING_NEXT");
        break;
    case AUDIO_PLAYER_CALLBACK_EVENT_PLAYING:
        ESP_LOGI(TAG, "AUDIO_PLAYER_CALLBACK_EVENT_PLAYING");
        break;
    case AUDIO_PLAYER_CALLBACK_EVENT_PAUSE:
        ESP_LOGI(TAG, "AUDIO_PLAYER_CALLBACK_EVENT_PAUSE");
        break;
    case AUDIO_PLAYER_CALLBACK_EVENT_SHUTDOWN:
        ESP_LOGI(TAG, "AUDIO_PLAYER_CALLBACK_EVENT_SHUTDOWN");
        break;
    case AUDIO_PLAYER_CALLBACK_EVENT_UNKNOWN_FILE_TYPE:
        ESP_LOGI(TAG, "AUDIO_PLAYER_CALLBACK_EVENT_UNKNOWN_FILE_TYPE");
        break;
    default:
        ESP_LOGI(TAG, "unknow player event");
        break;
    }
}

AudioPlayer::AudioPlayer()
{

}

void AudioPlayer::Initialize(const char *dir)
{
    music_dir_ = dir;
    /* create music directory if is not exist */
    mkdir(music_dir_, 0777);

    file_iterator_ = file_iterator_new(music_dir_);
    player_config_.mute_fn = AudioPlayer::mute_fn;
    player_config_.write_fn = AudioPlayer::write_fn;
    player_config_.clk_set_fn = AudioPlayer::clk_set_fn;
    player_config_.priority = 1;

    audio_player_new(player_config_);
    audio_player_callback_register(AudioPlayer::audio_player_cb, NULL);

    auto& mcp_server = McpServer::GetInstance();

    mcp_server.AddTool("self.music.play_song",
             "播放指定的歌曲。当用户要求播放音乐时使用此工具，会自动获取歌曲详情并开始流式播放。\n"
             "参数:\n"
             "  `song_name`: 要播放的歌曲名称（必需）。\n"
             "返回:\n"
             "  播放状态信息，不需确认，立刻播放歌曲。",
             PropertyList({
                 Property("song_name", kPropertyTypeString),//歌曲名称（必需）
             }),
             [this](const PropertyList& properties) -> ReturnValue {
                 auto song_name = properties["song_name"].value<std::string>();

                 if (!playMusic(song_name)) {
                     return "{\"success\": false, \"message\": \"获取音乐资源失败\"}";
                 }
                 return "{\"success\": true, \"message\": \"音乐开始播放\"}";
             });
}

esp_err_t AudioPlayer::playMusic(const std::string &musicName)
{
    esp_err_t ret = ESP_OK;
    int i = 0;
    int music_nums = file_iterator_get_count(file_iterator_);
    const char* music_name_cstr = musicName.c_str();
    const char *filename = nullptr;
    char fullpath[256];
    FILE *fp = nullptr;

    for (i = 0; i < music_nums; i++) {
        filename = file_iterator_get_name_from_index(file_iterator_, i);
        if (strstr(filename, music_name_cstr)) {
            ESP_LOGI(TAG, "found music %s", filename);
            break;
        }
    }

    if (i >= music_nums) {
        ESP_LOGI(TAG, "not found music %s", music_name_cstr);
        ret = ESP_FAIL;
        return ret;
    }

    // 切换为空闲状态，准备播放音乐
    file_iterator_get_full_path_from_index(file_iterator_, i, fullpath, 256);
    fp = fopen(fullpath, "rb");
    if (!fp) {
        ESP_LOGE(TAG, "failed to open music file %s", fullpath);
        ret = ESP_FAIL;
    } else {
        ESP_LOGI(TAG, "playing %s", filename);
        audio_player_play(fp);
    }

    return ret;
}
