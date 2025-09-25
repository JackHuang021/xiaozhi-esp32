/**
 * @file pmic_battery.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-09-25
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include "battery.h"
#include "pmic.h"

class PmicBattery : public Battery {
private:
    Pmic *pmic_;

public:
    PmicBattery(Pmic *pmic) : pmic_(pmic) {};
    uint8_t getBatteryLevel();
    BatteryState getBatteryState();
};
