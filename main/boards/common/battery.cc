/**
 * @file battery.cc
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-09-25
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "battery.h"

static const char* TAG = "Battery";

Battery::Battery()
{
    // 电池状态轮询定时器
    const esp_timer_create_args_t timer_args = {
        .callback = [](void* arg) {
            auto self = static_cast<Battery*>(arg);
            self->state_ = self->getBatteryState();
        },
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "backlight_timer",
        .skip_unhandled_events = true,
    };
    esp_timer_create(&timer_args, &state_timer_);
}


