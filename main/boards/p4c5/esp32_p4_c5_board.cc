#include "application.h"
#include "audio/codecs/es8389_audio_codec.h"
#include "button.h"
#include "config.h"
#include "display/display.h"
#include "p4c5_lcd_display.h"
#include "esp_video.h"
#include "lvgl_theme.h"
#include "wifi_board.h"

// Select the chip first so xpower.h typedefs XPowersPMU as XPowersAXP2101
#define XPOWERS_CHIP_AXP2101
#include "xpower.h"

#include <driver/i2c_master.h>
#include <driver/sdmmc_host.h>
#include <esp_idf_version.h>
#include <esp_lcd_mipi_dsi.h>
#include <esp_lcd_panel_ops.h>
#include <esp_ldo_regulator.h>
#include <esp_log.h>
#include <esp_vfs_fat.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sdmmc_cmd.h>

#include "esp_lcd_axs15260.h"
#include "esp_lcd_touch_axs15260.h"
#include "sd_pwr_ctrl_by_on_chip_ldo.h"

#define TAG "ESP32_P4C5"

class Pmic : public Xpowers {
public:
    Pmic(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : Xpowers(i2c_bus, addr) {

        ESP_LOGI(TAG, "axp2101 chip id is 0x%02x", pmu.getChipID());

        pmu.setVbusCurrentLimit(XPOWERS_AXP2101_VBUS_CUR_LIM_2000MA);
        pmu.setVbusVoltageLimit(XPOWERS_AXP2101_VBUS_VOL_LIM_4V36);
        pmu.setPrechargeCurr(XPOWERS_AXP2101_PRECHARGE_50MA);
        pmu.setChargerConstantCurr(XPOWERS_AXP2101_CHG_CUR_500MA);
        pmu.setChargerTerminationCurr(XPOWERS_AXP2101_CHG_ITERM_25MA);
        pmu.enableChargerTerminationLimit();
        pmu.setChargeTargetVoltage(XPOWERS_AXP2101_CHG_VOL_4V2);

        pmu.setChargingLedMode(XPOWERS_CHG_LED_CTRL_CHG);
        pmu.setLowBatWarnThreshold(10);
        pmu.setLowBatShutdownThreshold(5);
        pmu.setSysPowerDownVoltage(XPOWERS_AXP2101_VSYS_VOL_THRESHOLD_MIN);

        pmu.setDC1Voltage(XPOWERS_AXP2101_DCDC1_VOL_MAX);
        pmu.setALDO2Voltage(1800);
        pmu.setALDO3Voltage(1500);
        pmu.setALDO4Voltage(3300);

        pmu.enableDC1();
        pmu.disableDC3();
        pmu.disableALDO1();
        pmu.disableALDO2();
        pmu.disableALDO4();

        pmu.enableBattVoltageMeasure();
        pmu.enableVbusVoltageMeasure();
        pmu.enableSystemVoltageMeasure();
        pmu.enableTemperatureMeasure();
        pmu.enableTSPinMeasure();
        pmu.enableBattDetection();

        pmu.disableIRQ(XPOWERS_AXP2101_ALL_IRQ);
    }
};

class ESP32P4C5Board : public WifiBoard {
private:
    i2c_master_bus_handle_t codec_i2c_bus_ = nullptr;
    i2c_master_bus_handle_t touch_i2c_bus_ = nullptr;
    Button boot_button_;
    LcdDisplay* display_ = nullptr;
    esp_lcd_dsi_bus_handle_t dsi_bus_ = nullptr;
    esp_ldo_channel_handle_t dsi_phy_power_ = nullptr;
    lv_indev_t* touch_indev_ = nullptr;
    // AXS15260 built-in touch uses INT event mode: reading I2C while the
    // AXS15260 is idle lets its register pointer wander and return garbage
    // frames, so the LVGL indev only reads the cached state and the driver's own
    // task reads I2C only after a touch INT event, pushing the result via the
    // data callback.
    axs15260_touch_handle_t axs_touch_ = nullptr;
    int16_t last_touch_x_ = 0;
    int16_t last_touch_y_ = 0;
    lv_indev_state_t touch_state_ = LV_INDEV_STATE_RELEASED;
    // AXS15260 vendor wrapper panel (embeds the DPI panel). Kept after init but
    // never deleted: the real DPI panel is owned and freed by display_, so
    // deleting this one again would double-free.
    esp_lcd_panel_handle_t lcd_panel_ = nullptr;
    sdmmc_card_t* sd_card_ = nullptr;
    sd_pwr_ctrl_handle_t sd_power_ = nullptr;
    bool sd_card_mounted_ = false;
    Pmic* pmic;

    i2c_master_bus_handle_t CreateI2cBus(i2c_port_t port, gpio_num_t sda,
                                         gpio_num_t scl) {
        i2c_master_bus_config_t config = {
            .i2c_port = port,
            .sda_io_num = sda,
            .scl_io_num = scl,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = { .enable_internal_pullup = 1 },
        };
        i2c_master_bus_handle_t bus = nullptr;
        ESP_ERROR_CHECK(i2c_new_master_bus(&config, &bus));
        return bus;
    }

    void InitPmic() {
        pmic = new Pmic(touch_i2c_bus_, PMU_I2C_ADDR);
    }

    void InitializeI2cBus() {
        codec_i2c_bus_ = CreateI2cBus(AUDIO_CODEC_I2C_PORT, AUDIO_CODEC_I2C_SDA_PIN,
                                      AUDIO_CODEC_I2C_SCL_PIN);
        touch_i2c_bus_ = CreateI2cBus(TOUCH_I2C_PORT, TOUCH_I2C_SDA_PIN, TOUCH_I2C_SCL_PIN);
    }

    void InitializeLcd() {

        // 1. Enable DSI PHY power (internal LDO)
        esp_ldo_channel_config_t ldo_config = {
            .chan_id = MIPI_DSI_PHY_PWR_LDO_CHAN,
            .voltage_mv = MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV,
        };
        ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_config, &dsi_phy_power_));

        // 2. Reset the panel BEFORE creating the DSI bus, following the aibox
        //    reference: low 10ms -> high 170ms. GPIO26 is shared with the touch
        //    controller, so this single pulse also resets the touch before its
        //    own I2C init.
        gpio_config_t rst_conf = {
            .pin_bit_mask = 1ULL << DISPLAY_RESET_PIN,
            .mode = GPIO_MODE_OUTPUT,
        };
        ESP_ERROR_CHECK(gpio_config(&rst_conf));
        gpio_set_level(DISPLAY_RESET_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(DISPLAY_RESET_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(170));

        // 3. Create the MIPI DSI bus (2 lanes, 1000 Mbps)
        esp_lcd_dsi_bus_config_t bus_config = AXS15260_PANEL_BUS_DSI_2CH_CONFIG();
        ESP_ERROR_CHECK(esp_lcd_new_dsi_bus(&bus_config, &dsi_bus_));

        // 4. DBI command interface (used to send AXS15260 init commands)
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_dbi_io_config_t dbi_config = AXS15260_PANEL_IO_DBI_CONFIG();
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_dbi(dsi_bus_, &dbi_config, &panel_io));

        // 5. DPI video timing (452x1280 60Hz), in==out==RGB888. esp_lvgl_adapter
        //    drives this panel (see P4C5LcdDisplay); because the panel is RGB888,
        //    LVGL must be compiled with CONFIG_LV_COLOR_DEPTH_24=y (board config.json).
        //    The frame buffer count is dictated by the adapter's tear-avoidance and
        //    rotation configuration.
        esp_lcd_dpi_panel_config_t dpi_config =
            AXS15260_452_1280_PANEL_60HZ_CONFIG_CF(LCD_COLOR_FMT_RGB888);
        dpi_config.num_fbs =
            esp_lv_adapter_get_required_frame_buffer_count(P4C5LcdDisplay::kTearAvoidMode,
                                                           P4C5LcdDisplay::kRotation);

        // 6. Create the AXS15260 panel. Following the aibox driver, it sends the
        //    init command sequence via the DBI interface BEFORE creating the
        //    underlying DPI panel (the panel was reset at step 2, so it is ready
        //    to accept commands).
        axs15260_vendor_config_t vendor_config = {
            .mipi_config = {
                .dsi_bus = dsi_bus_,
                .dpi_config = &dpi_config,
                .lane_num = AXS15260_MIPI_LANES,
            },
            .flags = { .use_mipi_interface = 1 },
        };
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_RESET_PIN;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
        panel_config.bits_per_pixel = 24;  // AXS15260 panel is RGB888 (COLMOD 0x77)
        panel_config.vendor_config = &vendor_config;

        esp_lcd_panel_handle_t panel = nullptr;
        ESP_ERROR_CHECK(esp_lcd_new_panel_axs15260(panel_io, &panel_config, &panel));
        // 7. Initialize the DPI panel (this starts the DSI video mode). The init
        //    commands were already sent during panel creation, and the reset was
        //    done at step 2, so neither is repeated here.
        ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
        ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));
        lcd_panel_ = panel;

        esp_lcd_panel_handle_t dpi_panel = esp_lcd_axs15260_get_dpi_panel(panel);
        assert(dpi_panel != nullptr);
        display_ = new P4C5LcdDisplay(panel_io, dpi_panel);

        ESP_LOGI(TAG, "AXS15260 LCD init done: %dx%d portrait, active %dx%d landscape",
                 AXS15260_LCD_H_RES, AXS15260_LCD_V_RES, P4C5LcdDisplay::kActiveWidth,
                 P4C5LcdDisplay::kActiveHeight);
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
    }

    void InitializeTouch() {
        lv_display_t* lv_display = lv_display_get_default();
        if (lv_display == nullptr) {
            ESP_LOGE(TAG, "Cannot register AXS15260 touch without an LVGL display");
            return;
        }

        // The touch and the AXP2101 PMU share touch_i2c_bus_ (I2C1, 28/29). Only
        // the bus handle is passed to the driver, which attaches the touch device
        // to that bus instead of creating its own.
        // Touch reset shares GPIO26 with the LCD and is already done by
        // InitializeLcd, so NC is passed here to avoid a second reset pulse that
        // would blank the already-initialized LCD.
        axs15260_touch_config_t cfg = {
            .i2c_sda = TOUCH_I2C_SDA_PIN,
            .i2c_scl = TOUCH_I2C_SCL_PIN,
            .rst_gpio = GPIO_NUM_NC,
            .int_gpio = TOUCH_INT_PIN,
            .i2c_port = TOUCH_I2C_PORT,
            .i2c_freq_hz = AXS15260_TOUCH_I2C_FREQ_HZ,
            .i2c_bus = touch_i2c_bus_,
            .x_max = LCD_PHYSICAL_WIDTH,
            .y_max = LCD_PHYSICAL_HEIGHT,
            .flags = { .swap_xy = 0, .mirror_x = 0, .mirror_y = 0 },
        };
        if (axs15260_touch_new(&cfg, &axs_touch_) != ESP_OK) {
            ESP_LOGE(TAG, "AXS15260 touch init failed (addr 0x3B on I2C1)");
            return;
        }

        // The driver owns the event-driven touch task; LVGL indev uses EVENT mode
        // and only reads the cache, so its creation must happen inside the
        // esp_lvgl_adapter lock. `this` is stored as the indev driver data so the
        // read callback can recover it.
        if (esp_lv_adapter_lock(-1) == ESP_OK) {
            touch_indev_ = lv_indev_create();
            lv_indev_set_type(touch_indev_, LV_INDEV_TYPE_POINTER);
            lv_indev_set_read_cb(touch_indev_, AxsTouchReadCb);
            lv_indev_set_driver_data(touch_indev_, this);
            lv_indev_set_mode(touch_indev_, LV_INDEV_MODE_EVENT);
            lv_indev_set_display(touch_indev_, lv_display);
            esp_lv_adapter_unlock();
        }

        if (axs15260_touch_register_cb(axs_touch_, AxsTouchDataCb, this) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to register AXS15260 touch data callback");
            return;
        }
        ESP_LOGI(TAG, "AXS15260 touch attached to LVGL (INT event mode)");
    }

    // LVGL read callback: returns the cached state only, never touches I2C directly
    static void AxsTouchReadCb(lv_indev_t* indev, lv_indev_data_t* data) {
        auto* self = static_cast<ESP32P4C5Board*>(lv_indev_get_driver_data(indev));
        if (self == nullptr) {
            data->state = LV_INDEV_STATE_RELEASED;
            return;
        }
        data->point.x = self->last_touch_x_;
        data->point.y = self->last_touch_y_;
        data->state = self->touch_state_;
    }

    // Data callback (driver task context): maps physical portrait coordinates to
    // the 90-degree-rotated landscape active coordinates, updates the cache, and
    // asks LVGL to re-read the indev
    static void AxsTouchDataCb(axs15260_touch_handle_t handle, const axs15260_touch_data_t* data,
                               void* user_data) {
        (void)handle;
        auto* self = static_cast<ESP32P4C5Board*>(user_data);
        const axs15260_touch_point_t* point = &data->points[0];
        // Physical portrait coordinates (x in [0, 452), y in [0, 1280)) mapped to
        // LVGL active coordinates after 90-degree rotation (landscape). If the image
        // ends up flipped, mirror these two expressions instead.
        self->last_touch_x_ = LCD_PHYSICAL_HEIGHT - 1 - point->y;
        self->last_touch_y_ = point->x;
        self->touch_state_ = point->event == AXS15260_TOUCH_EVT_UP
                                 ? LV_INDEV_STATE_RELEASED
                                 : LV_INDEV_STATE_PRESSED;
        if (self->touch_indev_ && esp_lv_adapter_lock(-1) == ESP_OK) {
            lv_indev_read(self->touch_indev_);
            esp_lv_adapter_unlock();
        }
    }

    // void InitializeSdCard() {
    //     ESP_LOGI(TAG, "Initializing SD card");

    //     sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    //     host.slot = SDMMC_HOST_SLOT_0;
    //     host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;

    //     sdmmc_slot_config_t slot = {};
    //     slot.cd = SDMMC_SLOT_NO_CD;
    //     slot.wp = SDMMC_SLOT_NO_WP;
    //     slot.width = 4;

    //     const esp_vfs_fat_sdmmc_mount_config_t mount_config = {
    //         .format_if_mount_failed = false,
    //         .max_files = 5,
    //         .allocation_unit_size = 64 * 1024,
    //     };

    //     sd_pwr_ctrl_ldo_config_t power_config = {
    //         .ldo_chan_id = SD_CARD_PWR_LDO_CHAN,
    //     };
    //     esp_err_t ret = sd_pwr_ctrl_new_on_chip_ldo(&power_config, &sd_power_);
    //     if (ret != ESP_OK) {
    //         ESP_LOGE(TAG, "Failed to enable SD card power: %s", esp_err_to_name(ret));
    //         return;
    //     }
    //     host.pwr_ctrl_handle = sd_power_;

    //     ret = esp_vfs_fat_sdmmc_mount(SD_CARD_MOUNT_POINT, &host, &slot, &mount_config, &sd_card_);
    //     if (ret != ESP_OK) {
    //         ESP_LOGE(TAG, "Failed to mount SD card: %s", esp_err_to_name(ret));
    //         sd_pwr_ctrl_del_on_chip_ldo(sd_power_);
    //         sd_power_ = nullptr;
    //     } else {
    //         sd_card_mounted_ = true;
    //         ESP_LOGI(TAG, "SD card mounted successfully");
    //     }
    // }

    // void InitializeCamera() {
    //     ESP_LOGI(TAG, "Initializing camera");

    //     esp_video_init_csi_config_t csi_config = {
    //         .sccb_config =
    //             {
    //                 .init_sccb = false,
    //                 .i2c_handle = codec_i2c_bus_,
    //                 .freq = 400000,
    //             },
    //         .reset_pin = CAMERA_RESET_PIN,
    //         .pwdn_pin = CAMERA_PWDN_PIN,
    //     };
    //     esp_video_init_config_t video_config = {
    //         .csi = &csi_config,
    //     };

    //     camera_ = new EspVideo(video_config);
    // }

    // void InitializeFonts() {
    //     ESP_LOGI(TAG, "Initializing font support");
    //     auto& theme_manager = LvglThemeManager::GetInstance();
    //     auto current_theme = theme_manager.GetTheme("light");
    //     if (current_theme != nullptr) {
    //         auto text_font = current_theme->text_font();
    //         if (text_font != nullptr && text_font->font() != nullptr) {
    //             ESP_LOGI(TAG, "Custom font loaded successfully: line_height=%d",
    //                      text_font->font()->line_height);
    //         } else {
    //             ESP_LOGW(TAG, "Custom font not loaded, using built-in font");
    //         }
    //     }
    // }

public:
    ESP32P4C5Board() : boot_button_(BOOT_BUTTON_GPIO) {
        // LCD first: reset pin GPIO26 resets both the LCD and the touch, and only
        // then can the touch I2C be registered safely
        InitializeLcd();
        // Create two I2C buses: codec (I2C0, 7/8) + touch/PMIC (I2C1, 28/29)
        InitializeI2cBus();
        InitializeButtons();
        InitializeTouch();
        // InitializeSdCard();
        // InitializeCamera();
        // InitializeFonts();
        GetBacklight()->RestoreBrightness();
    }

    ~ESP32P4C5Board() {
        // delete camera_;
        // camera_ = nullptr;

        // if (sd_card_mounted_) {
        //     esp_err_t ret = esp_vfs_fat_sdcard_unmount(SD_CARD_MOUNT_POINT, sd_card_);
        //     if (ret != ESP_OK) {
        //         ESP_LOGE(TAG, "Failed to unmount SD card: %s", esp_err_to_name(ret));
        //     }
        //     sd_card_mounted_ = false;
        //     sd_card_ = nullptr;
        // }
        // if (sd_power_ != nullptr) {
        //     sd_pwr_ctrl_del_on_chip_ldo(sd_power_);
        //     sd_power_ = nullptr;
        // }

        // Stop the driver's touch task (touch_del also removes the ISR) before
        // deleting the indev, so no data callback can reference a freed indev
        if (axs_touch_ != nullptr) {
            axs15260_touch_del(axs_touch_);
            axs_touch_ = nullptr;
        }
        if (touch_indev_ != nullptr) {
            lv_indev_delete(touch_indev_);
            touch_indev_ = nullptr;
        }

        delete display_;
        display_ = nullptr;

        if (dsi_bus_ != nullptr) {
            esp_lcd_del_dsi_bus(dsi_bus_);
            dsi_bus_ = nullptr;
        }
        if (dsi_phy_power_ != nullptr) {
            esp_ldo_release_channel(dsi_phy_power_);
            dsi_phy_power_ = nullptr;
        }
    }

    virtual AudioCodec* GetAudioCodec() override {
        static Es8389AudioCodec audio_codec(
            codec_i2c_bus_, AUDIO_CODEC_I2C_PORT, AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK, AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT,
            AUDIO_I2S_GPIO_DIN, AUDIO_CODEC_PA_PIN, AUDIO_CODEC_ES8389_ADDR, false);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override { return display_; }

    virtual Backlight* GetBacklight() override {
        static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        return &backlight;
    }

    virtual Camera* GetCamera() override { return nullptr; }  // Camera not enabled yet
};

DECLARE_BOARD(ESP32P4C5Board);
