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
    Motion *motion;
    int16_t speed;
    int16_t hold_time_ms;
};

struct motion_msg {
    motion_args args;
    motion_state state;
};

struct motion_entry {
    int id;
    const char *name;
    void (*funtion)(struct motion_args *args);
};

static void motion_dance(struct motion_args *args)
{
    Motion *motion = args->motion;

    motion->motor_pwm_.setDuty(100);
}

static void motion_lift(motion_args *args)
{

}

static void motion_stop(motion_args *args)
{
    Motion *motion = args->motion;

    motion->motor_pwm_.setDuty(0);
}

struct motion_entry motion_table[STATE_MAX] = {
    {STATE_DANCE, "dance", motion_dance},
    {STATE_LIFT, "lift", motion_lift},
    {STATE_IDLE, "idle", motion_stop},
};

esp_err_t PWM::setupPWM(ledc_timer_t timer_num, uint32_t freq_hz,
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

esp_err_t PWM::setDuty(uint8_t duty) {
    esp_err_t ret = ESP_OK;

    this->duty_to_percent_ = duty;
    this->duty_ = (1 << this->duty_resolution_) * duty / 100;
    ret = ledc_set_duty(this->speed_mode_, this->channel_, this->duty_);
    ESP_RETURN_ON_ERROR(ret, TAG, "set duty failed");
    ret = ledc_update_duty(this->speed_mode_, this->channel_);
    ESP_RETURN_ON_ERROR(ret, TAG, "update duty failed");

    return ret;
};


Motion::Motion() {

}

esp_err_t Motion::motionInit() {
    esp_err_t ret = ESP_OK;
    action_queue_ = xQueueCreate(5, sizeof(struct motion_msg));
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

    xTaskCreate(Motion::motion_task, "motion_task", 1024, this, 5, NULL);

err:
    /* 销毁的操作全部放到析构函数中 */
    return ret;
}


void Motion::motion_task(void *arg)
{
    struct motion_args args = {0};
    struct motion_msg msg = {0};
    Motion *self = static_cast<Motion *>(arg);
    args.motion = self;

    while (1) {
        if (xQueueReceive(self->action_queue_, &msg, portMAX_DELAY)) {
            self->state = msg.state;
            
            ESP_LOGI(TAG, "state: %d, motion: %s", self->state, motion_table[self->state].name);
            if (self->state < STATE_MAX) {
                motion_table[self->state].funtion(&args);
            }
        }
    }
    vTaskDelete(NULL);
    ESP_LOGI(TAG, "motion task deleted");
}

void Motion::motionSend(enum motion_state state)
{
    struct motion_msg msg = {0};

    msg.state = state;
    xQueueSend(action_queue_, &msg, portMAX_DELAY);
}
