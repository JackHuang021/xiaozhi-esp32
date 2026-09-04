/**
 * @file esp_lcd_axs15260.h
 * @brief ESP LCD AXS15260 MIPI-DSI driver header
 *
 * @note Resolution: 452x1280, 2-lane MIPI DSI, 60Hz
 * @note Requires ESP-IDF v5.3 or later
 *
 * SPDX-FileCopyrightText: 2025
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "esp_idf_version.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_mipi_dsi.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// LCD resolution configuration
// ============================================================================
#define AXS15260_LCD_H_RES   452    // Horizontal resolution
#define AXS15260_LCD_V_RES   1280   // Vertical resolution

// ============================================================================
// Timing configuration (from the vendor init file)
// Reference: MIPI 2lane_452x1280_LV_15260D+CTC6.198_60Hz_TP=3ms_gamma2.4_20250313.txt
// ============================================================================
#define AXS15260_HBP         90     // Horizontal back porch
#define AXS15260_HFP         90     // Horizontal front porch
#define AXS15260_HSW         10     // Horizontal sync width
#define AXS15260_VBP         10     // Vertical back porch
#define AXS15260_VFP         250    // Vertical front porch
#define AXS15260_VSW         50     // Vertical sync width

// ============================================================================
// MIPI DSI configuration
// ============================================================================
#define AXS15260_MIPI_LANES  2      // MIPI data lane count
#define AXS15260_DCLK_MHZ    48     // Pixel clock (DPI clock) in MHz
#define AXS15260_HSCLK_MBPS  1000   // High-speed clock (HS clock) in Mbps

// ============================================================================
// Predefined configuration macros
// ============================================================================

/**
 * @brief AXS15260 MIPI DSI bus configuration (2 lanes)
 * @note phy_clk_src is not set explicitly (defaults to 0 = XTAL) to avoid an
 *       int-to-enum conversion error in C++
 */
#define AXS15260_PANEL_BUS_DSI_2CH_CONFIG()         \
    {                                               \
        .bus_id = 0,                                \
        .num_data_lanes = AXS15260_MIPI_LANES,      \
        .phy_clk_src = MIPI_DSI_PHY_CLK_SRC_DEFAULT,\
        .lane_bit_rate_mbps = AXS15260_HSCLK_MBPS,  \
    }

/**
 * @brief AXS15260 MIPI DBI IO configuration
 */
#define AXS15260_PANEL_IO_DBI_CONFIG()              \
    {                                               \
        .virtual_channel = 0,                       \
        .lcd_cmd_bits = 8,                          \
        .lcd_param_bits = 8,                        \
    }

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(6, 0, 0)
/**
 * @brief AXS15260 452x1280 60Hz DPI panel configuration (IDF < 6.0)
 * @param px_format Pixel format (e.g. LCD_COLOR_PIXEL_FORMAT_RGB888)
 * @note IDF 5.x uses the pixel_format and use_dma2d fields
 */
#define AXS15260_452_1280_PANEL_60HZ_CONFIG(px_format)          \
    {                                                           \
        .virtual_channel = 0,                                   \
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,            \
        .dpi_clock_freq_mhz = AXS15260_DCLK_MHZ,                \
        .pixel_format = px_format,                              \
        .num_fbs = 1,                                           \
        .video_timing = {                                       \
            .h_size = AXS15260_LCD_H_RES,                       \
            .v_size = AXS15260_LCD_V_RES,                       \
            .hsync_pulse_width = AXS15260_HSW,                  \
            .hsync_back_porch = AXS15260_HBP,                   \
            .hsync_front_porch = AXS15260_HFP,                  \
            .vsync_pulse_width = AXS15260_VSW,                  \
            .vsync_back_porch = AXS15260_VBP,                   \
            .vsync_front_porch = AXS15260_VFP,                  \
        },                                                      \
        .flags = { .use_dma2d = true, },                        \
    }
#endif

/**
 * @brief AXS15260 452x1280 60Hz DPI panel configuration (also IDF >= 6.0)
 * @param color_format Color format (e.g. LCD_COLOR_FMT_RGB888)
 * @note IDF 6.x removed pixel_format/use_dma2d and uses in/out_color_format
 */
#define AXS15260_452_1280_PANEL_60HZ_CONFIG_CF(color_format)    \
    {                                                           \
        .virtual_channel = 0,                                   \
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,            \
        .dpi_clock_freq_mhz = AXS15260_DCLK_MHZ,                \
        .in_color_format = color_format,                        \
        .out_color_format = color_format,                       \
        .num_fbs = 1,                                           \
        .video_timing = {                                       \
            .h_size = AXS15260_LCD_H_RES,                       \
            .v_size = AXS15260_LCD_V_RES,                       \
            .hsync_pulse_width = AXS15260_HSW,                  \
            .hsync_back_porch = AXS15260_HBP,                   \
            .hsync_front_porch = AXS15260_HFP,                  \
            .vsync_pulse_width = AXS15260_VSW,                  \
            .vsync_back_porch = AXS15260_VBP,                   \
            .vsync_front_porch = AXS15260_VFP,                  \
        },                                                      \
    }

// ============================================================================
// Init command structure
// ============================================================================

/**
 * @brief AXS15260 LCD init command structure
 */
typedef struct {
    uint8_t cmd;            // Command byte
    uint8_t data[64];       // Data byte array
    uint8_t data_bytes;     // Number of data bytes
    uint16_t delay_ms;      // Delay after the command, in milliseconds
} axs15260_lcd_init_cmd_t;

// ============================================================================
// Vendor configuration structures
// ============================================================================

/**
 * @brief AXS15260 MIPI configuration
 */
typedef struct {
    esp_lcd_dsi_bus_handle_t dsi_bus;                 // MIPI DSI bus handle
    const esp_lcd_dpi_panel_config_t *dpi_config;     // DPI panel configuration
    uint8_t lane_num;                                 // Data lane count (default 2)
} axs15260_mipi_config_t;

/**
 * @brief AXS15260 vendor configuration
 */
typedef struct {
    axs15260_mipi_config_t mipi_config;               // MIPI configuration
    const axs15260_lcd_init_cmd_t *init_cmds;         // Custom init commands (optional)
    uint16_t init_cmds_size;                          // Number of init commands
    struct {
        unsigned int use_mipi_interface: 1;           // Use the MIPI interface
        unsigned int mirror_by_cmd: 1;                // Mirror via command (not the LCD controller)
    } flags;
} axs15260_vendor_config_t;

// ============================================================================
// API functions
// ============================================================================

/**
 * @brief Create an AXS15260 LCD panel
 *
 * @param[in] io LCD panel IO handle
 * @param[in] panel_dev_config Panel device configuration
 * @param[out] ret_panel Returned panel handle
 * @return
 *      - ESP_OK: Success
 *      - ESP_ERR_INVALID_ARG: Invalid argument
 *      - ESP_ERR_NO_MEM: Out of memory
 *      - ESP_FAIL: Other error
 */
esp_err_t esp_lcd_new_panel_axs15260(const esp_lcd_panel_io_handle_t io,
                                     const esp_lcd_panel_dev_config_t *panel_dev_config,
                                     esp_lcd_panel_handle_t *ret_panel);

/**
 * @brief Get the DPI panel handle embedded in an AXS15260 panel
 * @note Used by LVGL and other callers that need direct DPI panel access
 *
 * @param[in] panel AXS15260 panel handle
 * @return DPI panel handle, or NULL if none exists
 */
esp_lcd_panel_handle_t esp_lcd_axs15260_get_dpi_panel(esp_lcd_panel_handle_t panel);

#ifdef __cplusplus
}
#endif
