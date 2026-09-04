#ifndef P4C5_LCD_DISPLAY_H
#define P4C5_LCD_DISPLAY_H

#include "display/lcd_display.h"

#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>

#include "esp_lv_adapter.h"
#include "esp_lv_adapter_display.h"

// AXS15260 MIPI-DSI display for the ESP32-P4C5 board.
//
// Unlike the shared MipiLcdDisplay (which uses esp_lvgl_port), this board class
// registers the panel through esp_lvgl_adapter. The adapter owns the LVGL
// display object, the draw buffers and the LVGL worker task, and provides
// tearing avoidance plus the 90 degree rotation that turns the physical
// 452x1280 (portrait) panel into a 1280x452 (landscape) UI.
//
// Consequences the board must honour (see esp32_p4_c5_board.cc):
//   * The DPI panel must be created with the frame-buffer count returned by
//     esp_lv_adapter_get_required_frame_buffer_count(kTearAvoidMode, kRotation).
//   * LVGL must be built for RGB888 (CONFIG_LV_COLOR_DEPTH_24=y in the board
//     config.json) because the AXS15260 panel is a 24bpp MIPI-DSI display; the
//     adapter renders at the LVGL compile-time color depth.
class P4C5LcdDisplay : public LcdDisplay {
public:
    P4C5LcdDisplay(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel);

    // Panel native (physical) resolution.
    static constexpr int kNativeWidth = 452;
    static constexpr int kNativeHeight = 1280;
    // Active (logical) resolution once the adapter applies the rotation.
    static constexpr int kActiveWidth = kNativeHeight;   // 1280
    static constexpr int kActiveHeight = kNativeWidth;   // 452

    // Adapter configuration. These decide the number of DPI frame buffers the
    // panel has to provide, so the board uses them to size dpi_config.num_fbs.
    static constexpr esp_lv_adapter_rotation_t kRotation = ESP_LV_ADAPTER_ROTATE_90;
    static constexpr esp_lv_adapter_tear_avoid_mode_t kTearAvoidMode =
        ESP_LV_ADAPTER_TEAR_AVOID_MODE_DEFAULT_MIPI_DSI;  // = TRIPLE_PARTIAL

protected:
    // LcdDisplay routes every LVGL-touching UI call through these virtuals, so
    // pointing them at the adapter lock makes the whole base UI safe to use
    // with esp_lvgl_adapter without touching main/display.
    bool Lock(int timeout_ms = 0) override;
    void Unlock() override;

private:
    lv_display_t* lv_display_ = nullptr;
};

#endif  // P4C5_LCD_DISPLAY_H
