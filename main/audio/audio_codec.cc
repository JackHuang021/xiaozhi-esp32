#include "audio_codec.h"
#include "board.h"
#include "settings.h"
#include "limits.h"

#include <esp_log.h>
#include <cstring>
#include <driver/i2s_common.h>
#include <math.h>

#define TAG "AudioCodec"

#define Q15_SHIFT 15
#define Q15_SCALE (1 << Q15_SHIFT)

typedef struct {
    int16_t b0, b1, b2;
    int32_t a1, a2;
    int32_t x1, x2; // 延迟的输入
    int32_t y1, y2; // 延迟的输出
} BiquadQ15;

// 初始化带通滤波器 (8kHz 采样率, 300~3000Hz)
void biquad_init(BiquadQ15 *f) {
    f->b0 = 5828;    // 0.17777623 * 32768
    f->b1 = 0;
    f->b2 = -5828;   // -0.17777623 * 32768
    f->a1 = -51248;  // -1.56101808 * 32768
    f->a2 = 21004;   // 0.64135154 * 32768

    f->x1 = f->x2 = 0;
    f->y1 = f->y2 = 0;
}

// 处理单个采样点 (int16 输入输出)
int16_t biquad_process(BiquadQ15 *f, int16_t x) {
    int32_t xn = x; // 转成 32 位防止溢出

    // Q15 定点运算
    int32_t yn = ( (int32_t)f->b0 * xn +
                   (int32_t)f->b1 * f->x1 +
                   (int32_t)f->b2 * f->x2 -
                   (int32_t)f->a1 * f->y1 -
                   (int32_t)f->a2 * f->y2 ) >> Q15_SHIFT;

    // 更新延迟
    f->x2 = f->x1;
    f->x1 = xn;
    f->y2 = f->y1;
    f->y1 = yn;

    // 限幅
    if (yn > 32767) yn = 32767;
    if (yn < -32768) yn = -32768;

    return (int16_t)yn;
}

// 处理一段音频数据 (samples 数组, len 样本数)
void process_audio_block(BiquadQ15 *f, int16_t *samples, int len) {
    for (int i = 0; i < len; i++) {
        samples[i] = biquad_process(f, samples[i]);
    }
}

static BiquadQ15 filter;

AudioCodec::AudioCodec() {
    biquad_init(&filter);
}

AudioCodec::~AudioCodec() {
}

uint32_t AudioCodec::calcu_volume_envelope(std::vector<int16_t> &data)
{
    uint32_t sum = 0;
    size_t i = 0;
    size_t cnt = 0;

    while (i < data.size()) {
        sum += abs(data[i]);
        i += 12;
        cnt ++;
    }

    return sum / cnt;  // RMS 结果
}

uint32_t AudioCodec::get_volume_envelope()
{
    return volume_envelope_;
}

void AudioCodec::OutputData(std::vector<int16_t>& data) {
    process_audio_block(&filter, data.data(), data.size());
    volume_envelope_ = calcu_volume_envelope(data);
    ESP_LOGI(TAG, "envelope: %lu", volume_envelope_);
    if (motion_callback_)
        motion_callback_(volume_envelope_);
    Write(data.data(), data.size());
}

bool AudioCodec::InputData(std::vector<int16_t>& data) {
    int samples = Read(data.data(), data.size());
    if (samples > 0) {
        return true;
    }
    return false;
}

void AudioCodec::Start() {
    Settings settings("audio", false);
    output_volume_ = settings.GetInt("output_volume", output_volume_);
    if (output_volume_ <= 0) {
        ESP_LOGW(TAG, "Output volume value (%d) is too small, setting to default (10)", output_volume_);
        output_volume_ = 10;
    }

    if (tx_handle_ != nullptr) {
        ESP_ERROR_CHECK(i2s_channel_enable(tx_handle_));
    }

    if (rx_handle_ != nullptr) {
        ESP_ERROR_CHECK(i2s_channel_enable(rx_handle_));
    }

    EnableInput(true);
    EnableOutput(true);
    ESP_LOGI(TAG, "Audio codec started");
}

void AudioCodec::SetOutputVolume(int volume) {
    output_volume_ = volume;
    ESP_LOGI(TAG, "Set output volume to %d", output_volume_);
    
    Settings settings("audio", true);
    settings.SetInt("output_volume", output_volume_);
}

void AudioCodec::EnableInput(bool enable) {
    if (enable == input_enabled_) {
        return;
    }
    input_enabled_ = enable;
    ESP_LOGI(TAG, "Set input enable to %s", enable ? "true" : "false");
}

void AudioCodec::EnableOutput(bool enable) {
    if (enable == output_enabled_) {
        return;
    }
    output_enabled_ = enable;
    ESP_LOGI(TAG, "Set output enable to %s", enable ? "true" : "false");
}
