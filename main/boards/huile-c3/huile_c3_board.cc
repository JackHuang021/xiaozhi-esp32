#include "wifi_board.h"
#include "audio_codecs/es8311_audio_codec.h"
#include "display/oled_display.h"
#include "application.h"
#include "button.h"
#include "led/single_led.h"
#include "iot/thing_manager.h"
#include "settings.h"
#include "config.h"
#include "power_save_timer.h"
#include "font_awesome_symbols.h"
#include "mcp_server.h"

#include <wifi_station.h>
#include <esp_log.h>
#include <esp_efuse_table.h>
#include <driver/i2c_master.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
#include <driver/ledc.h>
#include <esp_check.h>

#define TAG "huile_c3"

LV_FONT_DECLARE(font_puhui_14_1);
LV_FONT_DECLARE(font_awesome_14_1);

class PWM {
private:
    ledc_timer_t timer_num_;
    uint32_t freq_hz_;
    ledc_timer_bit_t duty_resolution_;
    ledc_mode_t speed_mode_;
    ledc_channel_t channel_;
    uint32_t duty_;
    uint8_t duty_to_percent_;
    gpio_num_t gpio_;

public:

    esp_err_t setupPWM(ledc_timer_t timer_num, uint32_t freq_hz,
        ledc_timer_bit_t duty_resolution, ledc_mode_t speed_mode,
        ledc_channel_t channel, uint32_t duty, gpio_num_t gpio) {

        esp_err_t ret = ESP_OK;

        this->timer_num_ = timer_num;
        this->freq_hz_ = freq_hz;
        this->duty_resolution_ = duty_resolution;
        this->speed_mode_ = speed_mode;
        this->channel_ = channel;
        this->duty_ = duty;
        this->gpio_ = gpio;

        ledc_timer_config_t timer_config = {
            .speed_mode = this->speed_mode_,
            .duty_resolution = this->duty_resolution_,
            .timer_num = this->timer_num_,
            .freq_hz = this->freq_hz_,
            .clk_cfg = LEDC_AUTO_CLK,
        };

        ledc_channel_config_t channel_config = {
            .gpio_num = this->gpio_,
            .speed_mode = this->speed_mode_,
            .channel = this->channel_,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = this->timer_num_,
            .duty = this->duty_,
        };

        ret = ledc_timer_config(&timer_config);
        ESP_RETURN_ON_ERROR(ret, TAG, "ledc timer config failed");
        ret = ledc_channel_config(&channel_config);
        ESP_RETURN_ON_ERROR(ret, TAG, "ledc channel config failed");

        return ret;
    }

    esp_err_t setDuty(uint8_t duty) {
        esp_err_t ret = ESP_OK;

        this->duty_to_percent_ = duty;
        this->duty_ = (1 << this->duty_resolution_) * duty / 100;
        ret = ledc_set_duty(this->speed_mode_, this->channel_, this->duty_);
        ESP_RETURN_ON_ERROR(ret, TAG, "set duty failed");
        ret = ledc_update_duty(this->speed_mode_, this->channel_);
        ESP_RETURN_ON_ERROR(ret, TAG, "update duty failed");

        return ret;
    };
};

class HuileC3Board : public WifiBoard {
private:
    i2c_master_bus_handle_t codec_i2c_bus_;
    esp_lcd_panel_io_handle_t panel_io_ = nullptr;
    esp_lcd_panel_handle_t panel_ = nullptr;
    Display* display_ = nullptr;
    Button touch_button_;
    Button switch_button_;
    PWM motor_pwm_;
    PWM mag_pwm_;
    bool lift_up_ = false;
    PowerSaveTimer* power_save_timer_;

    void InitializePowerSaveTimer() {
        power_save_timer_ = new PowerSaveTimer(160, 60);
        power_save_timer_->OnEnterSleepMode([this]() {
            ESP_LOGI(TAG, "Enabling sleep mode");
            auto display = GetDisplay();
            display->SetChatMessage("system", "");
            display->SetEmotion("sleepy");
            
            auto codec = GetAudioCodec();
            codec->EnableInput(false);
        });
        power_save_timer_->OnExitSleepMode([this]() {
            auto codec = GetAudioCodec();
            codec->EnableInput(true);
            
            auto display = GetDisplay();
            display->SetChatMessage("system", "");
            display->SetEmotion("neutral");
        });
        power_save_timer_->SetEnabled(true);
    }

    void InitializeCodecI2c() {
        // Initialize I2C peripheral
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &codec_i2c_bus_));
    }

    void InitializeButtons() {
        // 触摸按键按下后再弹起则触发录音
        touch_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting &&
                !WifiStation::GetInstance().IsConnected()) {
                ResetWifiConfiguration();
            }
                app.ToggleChatState();
        });

        // 触摸按键按下后唤醒设备
        touch_button_.OnPressDown([this]() {
            if (power_save_timer_)
                power_save_timer_->WakeUp();
        });

        // 提起检测的限位开关处理
        switch_button_.OnPressDown([this]() {
            lift_up_ = true;
        });

        switch_button_.OnPressUp([this]() {
            lift_up_ = false;
        });
    }

    void InitializePWM() {
        motor_pwm_.setupPWM(LEDC_TIMER_0, 4000, LEDC_TIMER_13_BIT,
                            LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0,
                            0, MOTOR_PWM_GPIO);
        mag_pwm_.setupPWM(LEDC_TIMER_1, 4000, LEDC_TIMER_13_BIT,
                            LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1,
                            0, MAG_PWM_GPIO);
    }

    esp_err_t InitializeOledDisplay() {
        esp_err_t ret = ESP_OK;
        esp_lcd_panel_io_i2c_config_t panel_io_config = {
            .dev_addr = 0x3C,
            .on_color_trans_done = nullptr,
            .user_ctx = nullptr,
            .control_phase_bytes = 1,
            .dc_bit_offset = 6,
            .lcd_cmd_bits = 8,
            .lcd_param_bits = 8,
            .flags = {
                .dc_low_on_data = 0,
                .disable_control_phase = 0,
            },
            .scl_speed_hz = 400 * 1000,
        };

        ret = esp_lcd_new_panel_io_i2c_v2(codec_i2c_bus_, &panel_io_config,
                                          &panel_io_);

        ESP_RETURN_ON_ERROR(ret, TAG, "failed to create lcd panel");

        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = GPIO_NUM_NC;
        panel_config.bits_per_pixel = 1;

        esp_lcd_panel_ssd1306_config_t ssd1306_config = {
            .height = DISPLAY_HEIGHT,
        };
        panel_config.vendor_config = &ssd1306_config;

        ret = esp_lcd_new_panel_ssd1306(panel_io_, &panel_config, &panel_);
        ESP_RETURN_ON_ERROR(ret, TAG, "failed to create ssd1306");

        ret = esp_lcd_panel_reset(panel_);
        ESP_RETURN_ON_ERROR(ret, TAG, "failed to reset lcd panel");
        ret = esp_lcd_panel_init(panel_);
        if (ESP_OK != ret) {
            ESP_LOGE(TAG, "failed to init ssd1306");
            display_ = new NoDisplay();
            return;
        }

        ESP_LOGI(TAG, "init ssd1306 done");
        ret = esp_lcd_panel_disp_on_off(panel_, true);

        display_ = new OledDisplay(panel_io_, panel_,
                                   DISPLAY_WIDTH, DISPLAY_HEIGHT,
                                   DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y,
                                   {&font_puhui_14_1, &font_awesome_14_1});
    }

    void InitMCPTools() {
        auto& mcp_server = McpServer::GetInstance();

        // 汇乐鹅跳舞
        mcp_server.AddTool("self.huile.dance", "汇乐鹅跳舞", PropertyList(),
                [this](const PropertyList& properties) -> ReturnValue {
            ESP_LOGI(TAG, "huile dance");
            return true;
        });

        // 张张嘴
        mcp_server.AddTool("self.huile.mouth_move", "汇乐鹅张张嘴", PropertyList(),
                [this](const PropertyList& properties) -> ReturnValue {
            ESP_LOGI(TAG, "huile mouse move");
            return true;
        });
    }

public:
    HuileC3Board() : touch_button_(TOUCH_BUTTON_GPIO),
                     switch_button_(SWITCH_GPIO, true) {
        // 把 ESP32C3 的 VDD SPI 引脚作为普通 GPIO 口使用
        esp_efuse_write_field_bit(ESP_EFUSE_VDD_SPI_AS_GPIO);

        display_ = new NoDisplay();

        InitializeCodecI2c();
        InitializeOledDisplay();
        InitializeButtons();
        InitializePowerSaveTimer();
        InitMCPTools();
    }

    virtual Led* GetLed() override {
        static SingleLed led(WS2812_LED_GPIO);
        return &led;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }

    virtual AudioCodec* GetAudioCodec() override {
        static Es8311AudioCodec audio_codec(codec_i2c_bus_, I2C_NUM_0, AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK, AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN,
            AUDIO_CODEC_PA_PIN, AUDIO_CODEC_ES8311_ADDR);
        return &audio_codec;
    }
};

DECLARE_BOARD(HuileC3Board);
