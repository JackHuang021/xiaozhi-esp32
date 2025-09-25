#include "pmic_battery.h"

uint8_t PmicBattery::getBatteryLevel()
{
    return battery_level_;
}


Battery::BatteryState PmicBattery::getBatteryState()
{
    battery_level_ = pmic_->GetBatteryLevel();
    if (pmic_->IsCharging())
        state_ = Battery::BATTERY_CHARGING;
    else if (pmic_->IsDischarging())
        state_ = Battery::BATTERY_DISCHARGING;

    if (pmic_->IsCharging() && battery_level_ < 20)
        state_ = Battery::BATTERY_LOW;

    return state_;
}
