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
    Pmic(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : Axp2101(i2c_bus, addr) {
        WriteReg(0x22, 0b110); // PWRON > OFFLEVEL as POWEROFF Source enable
        WriteReg(0x27, 0x10);  // hold 4s to power off

        // Disable All DCs but DC1
        WriteReg(0x80, 0x01);
        // Disable All LDOs
        WriteReg(0x90, 0x00);
        WriteReg(0x91, 0x00);

        // Set DC1 to 3.3V
        WriteReg(0x82, (3300 - 1500) / 100);

        // Set ALDO1 to 3.3V AVDD
        WriteReg(0x92, (3300 - 500) / 100);

        // Set ALDO4 to 3.3V LCD_BL
        WriteReg(0x95, (2500 - 500) / 100);

        // Set BLDO2 to 3.3V PA_EN
        WriteReg(0x97, (3300 - 500) / 100);

        // Set DLDO1 to 3.3V LCD_CS
         WriteReg(0x99, (3300 - 500) / 100);

        // Enable ALDO1(MIC) & Set LCD_CS High & PA_EN
        WriteReg(0x90, 0xA9);

        WriteReg(0x64, 0x02); // CV charger voltage setting to 4.1V
        
        WriteReg(0x61, 0x02); // set Main battery precharge current to 50mA
        WriteReg(0x62, 0x08); // set Main battery charger current to 400mA ( 0x08-200mA, 0x09-300mA, 0x0A-400mA )
        WriteReg(0x63, 0x01); // set Main battery term charge current to 25mA
    }

    void set_lcd_cs(bool state) {
        uint8_t value = ReadReg(0x90);
        if (state)
            value |= BIT(7);
        else
            value &= ~BIT(7);
        WriteReg(0x90, value);
    }

    void set_lcd_backlight(uint8_t brightness) {
        uint8_t value = ReadReg(0x90);
        uint16_t voltage = 0;
        const uint16_t min_brightness_vol = 2800;
        const uint16_t max_brightness_vol = 3400;

        if (brightness == 0) {
            value &= ~BIT(3);
            WriteReg(0x90, value);
            return;
        }

        voltage = 2800 + (brightness / 10) * 100;
        voltage = std::clamp(voltage, min_brightness_vol, max_brightness_vol);
        WriteReg(0x95, (voltage - 500) / 100);

        value |= BIT(3);
        WriteReg(0x90, value);
    }
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

