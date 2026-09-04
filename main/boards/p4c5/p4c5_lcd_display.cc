#include "p4c5_lcd_display.h"

#include <esp_log.h>

#define TAG "P4C5LcdDisplay"

P4C5LcdDisplay::P4C5LcdDisplay(esp_lcd_panel_io_handle_t panel_io,
                               esp_lcd_panel_handle_t panel)
    // LcdDisplay lays out its UI from width_/height_, so it must see the
    // *active* (rotated, landscape) resolution. The adapter profile below keeps
    // the native panel resolution and expresses the rotation itself.
    : LcdDisplay(panel_io, panel, kActiveWidth, kActiveHeight) {
    ESP_LOGI(TAG, "Init esp_lvgl_adapter: native %dx%d, active %dx%d, rotation=%d, tear=%d",
             kNativeWidth, kNativeHeight, kActiveWidth, kActiveHeight, (int)kRotation,
             (int)kTearAvoidMode);

    const esp_lv_adapter_config_t adapter_cfg = ESP_LV_ADAPTER_DEFAULT_CONFIG();
    ESP_ERROR_CHECK(esp_lv_adapter_init(&adapter_cfg));

    esp_lv_adapter_display_config_t disp_cfg =
        ESP_LV_ADAPTER_DISPLAY_MIPI_DEFAULT_CONFIG(panel, panel_io, kNativeWidth, kNativeHeight,
                                                   kRotation);

    lv_display_ = esp_lv_adapter_register_display(&disp_cfg);
    ESP_ERROR_CHECK(lv_display_ ? ESP_OK : ESP_FAIL);
    display_ = lv_display_;

    // Start the adapter's LVGL worker task. Everything created after this point
    // (e.g. the touch indev in the board) must take the adapter lock, which the
    // overridden Lock/Unlock below provide.
    ESP_ERROR_CHECK(esp_lv_adapter_start());

    ESP_LOGI(TAG, "esp_lvgl_adapter display registered (%dx%d)", kActiveWidth, kActiveHeight);
}

bool P4C5LcdDisplay::Lock(int timeout_ms) {
    // esp_lvgl_port treats timeout 0 as "wait forever"; mirror that on the
    // adapter, which uses -1 for an infinite wait.
    return esp_lv_adapter_lock(timeout_ms <= 0 ? -1 : timeout_ms) == ESP_OK;
}

void P4C5LcdDisplay::Unlock() {
    esp_lv_adapter_unlock();
}
