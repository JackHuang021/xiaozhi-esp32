#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_check.h"
#include "string.h"
#include "esp_err.h"
#include "config.h"
#include "driver/ledc.h"
#include "motion.h"

const static char *TAG = "huile_c3";

struct motion_args {
    int16_t speed;
    int16_t hold_time_ms;
};

struct motion_msg {
    motion_args args;
    motion_state state;
};

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

class Motion {
private:
    PWM motor_pwm_;
    PWM mag_pwm_;
    QueueHandle_t action_queue_;

public:
    Motion();
    esp_err_t motionInit();
};

Motion::Motion() {

}

esp_err_t Motion::motionInit() {
    esp_err_t ret = ESP_OK;
    action_queue_ = xQueueCreate(5, sizeof(struct motion_args));
    ESP_GOTO_ON_FALSE(action_queue_ != NULL, ESP_ERR_NO_MEM, err, TAG,
                      "failed to create motion queue");

    /* init pwm */
    ret = motor_pwm_.setupPWM(LEDC_TIMER_0, 4000, LEDC_TIMER_13_BIT,
                              LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0,
                              0, MOTOR_PWM_GPIO);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "failed to init motor pwm");
    ret = mag_pwm_.setupPWM(LEDC_TIMER_1, 4000, LEDC_TIMER_13_BIT,
                            LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1,
                            0, MAG_PWM_GPIO);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "failed to init mag pwm");

err:
    /* 销毁的操作全部放到析构函数中 */
    return ret;
}