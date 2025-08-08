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
#include "application.h"
#include "esp_timer.h"
#include "device_state_event.h"

const static char *TAG = "huile_c3";

struct motion_msg {
    motion_args args;
    motion_state state;
};

struct motion_entry {
    int id;
    const char *name;
    void (*funtion)(struct motion_args *args);
};

static void dance_timer_callback(void *args)
{
    Motion::GetInstance().setMotorPwm(0);
}

void Motion::motion_dance(struct motion_args *args)
{
    esp_timer_start_once(Motion::GetInstance().dance_timer_handle,
                         args->hold_time_ms * 1000);

    Motion::GetInstance().setMotorPwm(100);
}

void Motion::motion_lift(motion_args *args)
{
    Motion::GetInstance().setMotorPwm(100);
}

void Motion::motion_stop(motion_args *args)
{
    Motion::GetInstance().setMotorPwm(0);
}

void Motion::motion_on_state_change(DeviceState previous, DeviceState current)
{
    // if (current == kDeviceStateSpeaking)
    //     Motion::GetInstance().setMagPwm(20);
    // else
    //     Motion::GetInstance().setMagPwm(0);
}

struct motion_entry motion_table[STATE_MAX] = {
    {STATE_DANCE, "dance", Motion::motion_dance},
    {STATE_LIFT, "lift", Motion::motion_lift},
    {STATE_IDLE, "idle", Motion::motion_stop},
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
    ESP_LOGI(TAG, "set pwm duty %lu", this->duty_);
    ESP_RETURN_ON_ERROR(ret, TAG, "set duty failed");
    ret = ledc_update_duty(this->speed_mode_, this->channel_);
    ESP_RETURN_ON_ERROR(ret, TAG, "update duty failed");

    return ret;
};


Motion::Motion() {

}

Motion& Motion::GetInstance() {
    static Motion instance;
    return instance;
}

motion_state Motion::getMotionState()
{
    return state;
}

void Motion::motion_task(void *arg)
{
    struct motion_msg msg = {0};

    esp_timer_create_args_t dance_timer = {
        .callback = dance_timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "dance_timer"
    };
    esp_timer_create(&dance_timer, &Motion::GetInstance().dance_timer_handle);

    while (1) {
        if (xQueueReceive(Motion::GetInstance().action_queue_, &msg, portMAX_DELAY)) {
            Motion::GetInstance().state = msg.state;

            if (msg.state < STATE_MAX) {
                ESP_LOGI(TAG, "state: %d, motion: %s",
                     msg.state, motion_table[msg.state].name);
                motion_table[msg.state].funtion(&msg.args);
            }
        }
    }
    vTaskDelete(NULL);
    ESP_LOGI(TAG, "motion task deleted");
}

esp_err_t Motion::motionInit() {
    esp_err_t ret = ESP_OK;

    action_queue_ = xQueueCreate(5, sizeof(struct motion_msg));
    ESP_RETURN_ON_FALSE(action_queue_ != NULL, ESP_ERR_NO_MEM, TAG,
                        "failed to create motion queue");

    /* init pwm, motor pwm frequency: 4000 HZ */
    ret = motor_pwm_.setupPWM(LEDC_TIMER_0, 4000, LEDC_TIMER_13_BIT,
                              LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0,
                              0, MOTOR_PWM_GPIO);
    ESP_RETURN_ON_ERROR(ret, TAG, "failed to init motor pwm");
    ret = mag_pwm_.setupPWM(LEDC_TIMER_1, 10, LEDC_TIMER_13_BIT,
                            LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1,
                            0, MAG_PWM_GPIO);
    ESP_RETURN_ON_ERROR(ret, TAG, "failed to init mag pwm");

    xTaskCreate(Motion::motion_task, "motion_task", 1024, NULL, 5, NULL);

    auto & device_state_manager = DeviceStateEventManager::GetInstance();
    device_state_manager.RegisterStateChangeCallback(Motion::motion_on_state_change);

    return ret;
}

void Motion::motionSend(enum motion_state state, struct motion_args *args)
{
    struct motion_msg msg = {0};

    msg.state = state;
    if (args != NULL)
        memcpy(&msg.args, args, sizeof(struct motion_args));
    xQueueSend(action_queue_, &msg, portMAX_DELAY);
}

void Motion::setMotorPwm(uint8_t duty)
{
    motor_pwm_.setDuty(duty);
}

void Motion::setMagPwm(uint8_t duty)
{
    mag_pwm_.setDuty(duty);
}
