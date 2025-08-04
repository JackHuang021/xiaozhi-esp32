/**
 * @file motion.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-06-18
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include "esp_err.h"
#include "driver/ledc.h"

#ifdef __cplusplus
extern "C" {
#endif

enum motion_state {
    STATE_DANCE,
    STATE_LIFT,
    STATE_IDLE,
    STATE_MAX,
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
            ledc_channel_t channel, uint32_t duty, gpio_num_t gpio);

    esp_err_t setDuty(uint8_t duty);
};

class Motion {
private:
    QueueHandle_t action_queue_;
    enum motion_state state;

    static void motion_task(void *arg);
    static void mouth_action(void);

public:
    PWM motor_pwm_;
    PWM mag_pwm_;

    Motion();
    motion_state getMotionState();
    esp_err_t motionInit();
    void motionSend(enum motion_state state, struct motion_args *args);
};

struct motion_args {
    Motion *motion;
    int16_t speed;
    int16_t hold_time_ms;
};

#ifdef __cplusplus
}
#endif

