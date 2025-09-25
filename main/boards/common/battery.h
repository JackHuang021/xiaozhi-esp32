/**
 * @file battery.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-09-25
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <cstdint>
#include <esp_timer.h>

#pragma once

class Battery {
public:
    enum BatteryState {
        BATTERY_CHARGING,
        BATTERY_DISCHARGING,
        BATTERY_LOW,
        BATTERY_ERROR
    };
public:
    Battery();

protected:
    virtual uint8_t getBatteryLevel() = 0;
    virtual BatteryState getBatteryState() = 0;
    uint8_t battery_level_ = 0;
    BatteryState state_;
    esp_timer_handle_t state_timer_ = nullptr;
};
