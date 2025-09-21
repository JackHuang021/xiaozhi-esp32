/**
 * @file pmic.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-09-21
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include <algorithm>
#include "axp2101.h"
#include "backlight.h"

class Pmic : public Axp2101 {
public:
    Pmic(i2c_master_bus_handle_t i2c_bus, uint8_t addr);

    void set_lcd_cs(bool state);
    void set_lcd_backlight(uint8_t brightness);
    void set_pa_enable(bool enable);
};

class PmicBacklight : public Backlight {

private:
    Pmic *pmic_;

public:
    PmicBacklight(Pmic *pmic) : pmic_(pmic){
    }

    void SetBrightnessImpl(uint8_t brightness) override {
        pmic_->set_lcd_backlight(brightness);
    }
};

