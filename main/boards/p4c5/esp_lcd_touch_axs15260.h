/**
 * @file esp_lcd_touch_axs15260.h
 * @brief AXS15260 touch controller driver header
 *
 * @note I2C slave address: 0x3B
 * @note Supports up to 5 touch points
 * @note Resolution: 452x1280 (same as the LCD)
 *
 * SPDX-FileCopyrightText: 2025
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Configuration macros
// ============================================================================

// I2C configuration
#define AXS15260_TOUCH_I2C_ADDR    0x3B    // I2C slave address
#define AXS15260_TOUCH_I2C_FREQ_HZ 100000  // I2C clock frequency (100kHz)

// Touch configuration
#define AXS15260_TOUCH_MAX_POINTS  5       // Maximum number of touch points
#define AXS15260_TOUCH_POINT_SIZE  6       // Data length of a single touch point
#define AXS15260_TOUCH_BUF_SIZE    8       // Touch data buffer size

// Default resolution (same as the LCD)
#define AXS15260_TOUCH_H_RES       452     // Horizontal resolution
#define AXS15260_TOUCH_V_RES       1280    // Vertical resolution

// ============================================================================
// Touch event types
// ============================================================================

typedef enum {
    AXS15260_TOUCH_EVT_DOWN     = 0,    // Pressed
    AXS15260_TOUCH_EVT_UP       = 1,    // Released
    AXS15260_TOUCH_EVT_CONTACT  = 2,    // Sustained contact
} axs15260_touch_event_t;

// ============================================================================
// Data structures
// ============================================================================

/**
 * @brief Single touch point data
 */
typedef struct {
    uint16_t x;         // X coordinate
    uint16_t y;         // Y coordinate
    uint8_t id;         // Touch point ID (0-4)
    uint8_t event;      // Event type
    uint8_t weight;     // Pressure value
    uint8_t area;       // Touch area
} axs15260_touch_point_t;

/**
 * @brief Touch data
 */
typedef struct {
    uint8_t point_num;                                       // Number of touch points
    uint8_t gesture_id;                                      // Gesture ID
    axs15260_touch_point_t points[AXS15260_TOUCH_MAX_POINTS]; // Touch point array
} axs15260_touch_data_t;

/**
 * @brief Touch controller configuration
 */
typedef struct {
    gpio_num_t i2c_sda;                      // I2C SDA pin
    gpio_num_t i2c_scl;                      // I2C SCL pin
    gpio_num_t rst_gpio;                     // Reset pin (-1 to disable)
    gpio_num_t int_gpio;                     // Interrupt pin (-1 to disable)
    i2c_port_num_t i2c_port;                 // I2C port number (only used to create
                                             // a bus when i2c_bus is NULL)
    uint32_t i2c_freq_hz;                    // I2C clock frequency (0 for the default)
    i2c_master_bus_handle_t i2c_bus;         // Reused external I2C bus; no bus is
                                             // created when non-NULL
    uint16_t x_max;                          // Maximum X (0 for the default)
    uint16_t y_max;                          // Maximum Y (0 for the default)
    struct {
        uint8_t swap_xy: 1;                  // Swap the X/Y coordinates
        uint8_t mirror_x: 1;                 // Mirror the X coordinate
        uint8_t mirror_y: 1;                 // Mirror the Y coordinate
    } flags;
} axs15260_touch_config_t;

/**
 * @brief Touch controller handle (opaque pointer)
 */
typedef struct axs15260_touch_dev *axs15260_touch_handle_t;

/**
 * @brief Touch data callback type, invoked from the driver's touch task in task
 *        context after each successful data read
 */
typedef void (*axs15260_touch_data_cb_t)(axs15260_touch_handle_t handle,
                                         const axs15260_touch_data_t *data,
                                         void *user_data);

// ============================================================================
// API functions
// ============================================================================

/**
 * @brief Create an AXS15260 touch controller driver
 *
 * @param[in] config Touch controller configuration
 * @param[out] handle Returned touch controller handle
 * @return
 *      - ESP_OK: Success
 *      - ESP_ERR_INVALID_ARG: Invalid argument
 *      - ESP_ERR_NO_MEM: Out of memory
 */
esp_err_t axs15260_touch_new(const axs15260_touch_config_t *config,
                             axs15260_touch_handle_t *handle);

/**
 * @brief Delete an AXS15260 touch controller driver
 *
 * @param[in] handle Touch controller handle
 * @return
 *      - ESP_OK: Success
 *      - ESP_ERR_INVALID_ARG: Invalid argument
 */
esp_err_t axs15260_touch_del(axs15260_touch_handle_t handle);

/**
 * @brief Reset the touch controller
 *
 * @param[in] handle Touch controller handle
 * @return
 *      - ESP_OK: Success
 *      - ESP_ERR_INVALID_ARG: Invalid argument
 */
esp_err_t axs15260_touch_reset(axs15260_touch_handle_t handle);

/**
 * @brief Read touch data
 *
 * @param[in] handle Touch controller handle
 * @param[out] data Touch data
 * @return
 *      - ESP_OK: Success
 *      - ESP_ERR_NOT_FOUND: No new touch event for the current interrupt
 *      - ESP_ERR_INVALID_RESPONSE: Invalid touch frame received and discarded
 *      - ESP_ERR_INVALID_ARG: Invalid argument
 *      - ESP_ERR_TIMEOUT: Timeout
 */
esp_err_t axs15260_touch_read(axs15260_touch_handle_t handle,
                              axs15260_touch_data_t *data);

/**
 * @brief Read the firmware version
 *
 * @param[in] handle Touch controller handle
 * @param[out] version Firmware version
 * @return
 *      - ESP_OK: Success
 *      - ESP_ERR_INVALID_ARG: Invalid argument
 */
esp_err_t axs15260_touch_get_version(axs15260_touch_handle_t handle,
                                     uint16_t *version);

/**
 * @brief Register a data callback invoked by the driver's touch task after each
 *        successful read
 *
 * The callback runs in task context (not ISR context), so it may call APIs that
 * require a task (e.g. LVGL / lvgl_port locks). Pass NULL to unregister.
 * A callback can only be registered when an interrupt pin is configured.
 *
 * @param[in] handle Touch controller handle
 * @param[in] callback Callback function
 * @param[in] user_data User data
 * @return
 *      - ESP_OK: Success
 *      - ESP_ERR_NOT_SUPPORTED: No interrupt pin configured and callback is not NULL
 *      - ESP_ERR_INVALID_ARG: Invalid argument
 */
esp_err_t axs15260_touch_register_cb(axs15260_touch_handle_t handle,
                                     axs15260_touch_data_cb_t callback,
                                     void *user_data);

/**
 * @brief Set the coordinate transformation
 *
 * @param[in] handle Touch controller handle
 * @param[in] swap_xy Swap the X/Y coordinates
 * @param[in] mirror_x Mirror the X coordinate
 * @param[in] mirror_y Mirror the Y coordinate
 * @return
 *      - ESP_OK: Success
 *      - ESP_ERR_INVALID_ARG: Invalid argument
 */
esp_err_t axs15260_touch_set_swap_xy(axs15260_touch_handle_t handle,
                                     bool swap_xy, bool mirror_x, bool mirror_y);

/**
 * @brief Check whether a touch event is pending (via the interrupt pin)
 *
 * @param[in] handle Touch controller handle
 * @return true if touched, false otherwise
 */
bool axs15260_touch_is_pressed(axs15260_touch_handle_t handle);

#ifdef __cplusplus
}
#endif
