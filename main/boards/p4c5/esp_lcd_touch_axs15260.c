/**
 * @file esp_lcd_touch_axs15260.c
 * @brief AXS15260 touch controller driver implementation
 *
 * @note Ported from the AXS15260 Linux driver V2.2.4
 * @note I2C slave address: 0x3B
 * @note Supports up to 5 touch points
 *
 * SPDX-FileCopyrightText: 2025
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "esp_lcd_touch_axs15260.h"

static const char *TAG = "axs15260_touch";

// ============================================================================
// Register definitions
// ============================================================================
#define REG_VERSION             0x0C    // Firmware version register
#define AXS15260_I2C_TIMEOUT_MS 20
#define AXS15260_ESD_INVALID_LIMIT 50

// ============================================================================
// Internal data structures
// ============================================================================

struct axs15260_touch_dev {
    i2c_master_bus_handle_t i2c_bus;      // I2C bus handle
    i2c_master_dev_handle_t i2c_dev;      // I2C device handle
    gpio_num_t rst_gpio;                  // Reset pin
    gpio_num_t int_gpio;                  // Interrupt pin
    uint16_t x_max;                       // Maximum X
    uint16_t y_max;                       // Maximum Y
    uint8_t buf[AXS15260_TOUCH_BUF_SIZE]; // Data buffer
    axs15260_touch_data_cb_t callback;    // Data callback (task context)
    void *user_data;                      // User data
    TaskHandle_t task;                    // Internal event-driven touch task
    SemaphoreHandle_t lock;               // Mutex
    uint16_t invalid_count;               // Consecutive invalid frame counter
    uint16_t last_x;                      // Most recent valid X coordinate
    uint16_t last_y;                      // Most recent valid Y coordinate
    struct {
        uint8_t swap_xy: 1;
        uint8_t mirror_x: 1;
        uint8_t mirror_y: 1;
        uint8_t inited: 1;
        uint8_t isr_registered: 1;
        uint8_t last_pressed: 1;
        uint8_t owns_bus: 1;  // Whether the driver created the bus itself (only
                              // then is it destroyed in del)
    } flags;
};

// ============================================================================
// Internal functions
// ============================================================================

static void IRAM_ATTR touch_isr(void *arg)
{
    axs15260_touch_handle_t handle = (axs15260_touch_handle_t)arg;
    if (handle && handle->task) {
        BaseType_t woken = pdFALSE;
        vTaskNotifyGiveFromISR(handle->task, &woken);
        portYIELD_FROM_ISR(woken);
    }
}

// Event-driven touch task: a touch INT wakes it, it reads and parses the data,
// then invokes the registered callback in task context. It never polls.
static void axs15260_touch_task(void *arg)
{
    axs15260_touch_handle_t handle = (axs15260_touch_handle_t)arg;
    ESP_LOGI(TAG, "AXS15260 touch task started");
    while (true) {
        ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
        do {
            axs15260_touch_data_t data;
            if (axs15260_touch_read(handle, &data) == ESP_OK && handle->callback) {
                handle->callback(handle, &data, handle->user_data);
            }
        } while (ulTaskNotifyTake(pdFALSE, 0) > 0);
    }
}

static esp_err_t touch_i2c_read(axs15260_touch_handle_t handle, uint8_t *data, size_t len)
{
    return i2c_master_receive(handle->i2c_dev, data, len, AXS15260_I2C_TIMEOUT_MS);
}

static esp_err_t touch_i2c_write_read(axs15260_touch_handle_t handle,
                                      uint8_t *cmd, size_t cmd_len,
                                      uint8_t *data, size_t data_len)
{
    return i2c_master_transmit_receive(handle->i2c_dev, cmd, cmd_len, data, data_len,
                                       AXS15260_I2C_TIMEOUT_MS);
}

static bool touch_frame_is_all_ff(const uint8_t *buf)
{
    for (size_t i = 0; i < AXS15260_TOUCH_BUF_SIZE; i++) {
        if (buf[i] != 0xFF) {
            return false;
        }
    }
    return true;
}

static void touch_fill_release(axs15260_touch_handle_t handle, axs15260_touch_data_t *data)
{
    memset(data, 0, sizeof(*data));
    data->point_num = 1;
    data->points[0].x = handle->last_x;
    data->points[0].y = handle->last_y;
    data->points[0].event = AXS15260_TOUCH_EVT_UP;
}

static esp_err_t touch_recover_locked(axs15260_touch_handle_t handle)
{
    esp_err_t ret;

    if (handle->rst_gpio >= 0) {
        ESP_LOGW(TAG, "Invalid frames reached the threshold, performing a touch hardware reset");
        gpio_set_level(handle->rst_gpio, 1);
        vTaskDelay(pdMS_TO_TICKS(1));
        gpio_set_level(handle->rst_gpio, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(handle->rst_gpio, 1);
        vTaskDelay(pdMS_TO_TICKS(110));
        ret = ESP_OK;
    } else {
        ESP_LOGW(TAG, "Touch reset pin not connected, recovering via I2C bus reset");
        ret = i2c_master_bus_reset(handle->i2c_bus);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "I2C bus reset failed: %s", esp_err_to_name(ret));
            return ret;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    uint8_t cmd = REG_VERSION;
    uint8_t version_buf[2] = {0};
    ret = touch_i2c_write_read(handle, &cmd, 1, version_buf, sizeof(version_buf));
    if (ret == ESP_OK) {
        uint16_t version = ((uint16_t)version_buf[0] << 8) | version_buf[1];
        ESP_LOGI(TAG, "Touch recovery done, firmware version: 0x%04X", version);
    } else {
        ESP_LOGE(TAG, "Reading the version after touch recovery failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

static bool touch_record_invalid(axs15260_touch_handle_t handle, const char *reason,
                                 const uint8_t *buf)
{
    handle->invalid_count++;
    // Invalid frames are normal idle jitter; discard them without spamming the
    // log, keeping details at DEBUG level only. Real anomalies (accumulating to
    // the ESD threshold and triggering recovery) are still reported at WARN below.
    if (handle->invalid_count <= 3 || (handle->invalid_count % 10) == 0) {
        if (buf) {
            ESP_LOGD(TAG,
                     "Discarding invalid touch frame #%u, reason: %s, raw data: "
                     "%02X %02X %02X %02X %02X %02X %02X %02X",
                     handle->invalid_count, reason,
                     buf[0], buf[1], buf[2], buf[3],
                     buf[4], buf[5], buf[6], buf[7]);
        } else {
            ESP_LOGD(TAG, "Touch read anomaly #%u, reason: %s",
                     handle->invalid_count, reason);
        }
    }

    if (handle->invalid_count < AXS15260_ESD_INVALID_LIMIT) {
        return false;
    }

    touch_recover_locked(handle);
    handle->invalid_count = 0;
    return true;
}

// ============================================================================
// Public API implementation
// ============================================================================

esp_err_t axs15260_touch_new(const axs15260_touch_config_t *config,
                             axs15260_touch_handle_t *handle)
{
    ESP_RETURN_ON_FALSE(config && handle, ESP_ERR_INVALID_ARG, TAG, "Invalid argument");

    ESP_LOGI(TAG, "Creating the AXS15260 touch controller driver...");

    esp_err_t ret;

    // Allocate the device structure
    axs15260_touch_handle_t dev = calloc(1, sizeof(struct axs15260_touch_dev));
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_NO_MEM, TAG, "Memory allocation failed");

    // Save the configuration
    dev->rst_gpio = config->rst_gpio;
    dev->int_gpio = config->int_gpio;
    dev->x_max = config->x_max > 0 ? config->x_max : AXS15260_TOUCH_H_RES;
    dev->y_max = config->y_max > 0 ? config->y_max : AXS15260_TOUCH_V_RES;
    dev->flags.swap_xy = config->flags.swap_xy;
    dev->flags.mirror_x = config->flags.mirror_x;
    dev->flags.mirror_y = config->flags.mirror_y;

    // Create the mutex
    dev->lock = xSemaphoreCreateMutex();
    if (!dev->lock) {
        ESP_LOGE(TAG, "Failed to create the mutex");
        free(dev);
        return ESP_ERR_NO_MEM;
    }

    // Configure the reset pin
    if (config->rst_gpio >= 0) {
        ESP_LOGI(TAG, "Configuring the reset pin (GPIO %d)...", config->rst_gpio);
        gpio_config_t rst_conf = {
            .pin_bit_mask = (1ULL << config->rst_gpio),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&rst_conf);
        gpio_set_level(config->rst_gpio, 1);
    }

    // Configure the interrupt pin
    if (config->int_gpio >= 0) {
        ESP_LOGI(TAG, "Configuring the interrupt pin (GPIO %d)...", config->int_gpio);
        gpio_config_t int_conf = {
            .pin_bit_mask = (1ULL << config->int_gpio),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_NEGEDGE,
        };
        ret = gpio_config(&int_conf);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure the touch interrupt pin: %s", esp_err_to_name(ret));
            vSemaphoreDelete(dev->lock);
            free(dev);
            return ret;
        }
        ret = gpio_install_isr_service(0);
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "Failed to install the GPIO ISR service: %s", esp_err_to_name(ret));
            gpio_reset_pin(config->int_gpio);
            vSemaphoreDelete(dev->lock);
            free(dev);
            return ret;
        }
    }

    // Initialize I2C
    uint32_t freq = config->i2c_freq_hz > 0 ? config->i2c_freq_hz : AXS15260_TOUCH_I2C_FREQ_HZ;
    ESP_LOGI(TAG, "Initializing I2C (SDA=%d, SCL=%d, freq=%luHz)...",
             config->i2c_sda, config->i2c_scl, (unsigned long)freq);

    if (config->i2c_bus) {
        // Reuse an externally created I2C bus (shared with the codec/PMU); the
        // driver does not create one of its own.
        dev->i2c_bus = config->i2c_bus;
        ESP_LOGI(TAG, "Reusing the external I2C bus");
    } else {
        i2c_master_bus_config_t bus_cfg = {
            .i2c_port = config->i2c_port,
            .sda_io_num = config->i2c_sda,
            .scl_io_num = config->i2c_scl,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true,
        };

        ret = i2c_new_master_bus(&bus_cfg, &dev->i2c_bus);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create the I2C bus");
            vSemaphoreDelete(dev->lock);
            free(dev);
            return ret;
        }
        dev->flags.owns_bus = 1;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AXS15260_TOUCH_I2C_ADDR,
        .scl_speed_hz = freq,
    };

    ret = i2c_master_bus_add_device(dev->i2c_bus, &dev_cfg, &dev->i2c_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add the I2C device");
        if (dev->flags.owns_bus) {
            i2c_del_master_bus(dev->i2c_bus);
        }
        vSemaphoreDelete(dev->lock);
        free(dev);
        return ret;
    }

    ESP_LOGI(TAG, "I2C initialized (address: 0x%02X)", AXS15260_TOUCH_I2C_ADDR);

    // Perform a hardware reset
    ret = axs15260_touch_reset(dev);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Hardware reset failed, continuing initialization");
    }

    // Read the firmware version to verify communication
    uint16_t version = 0;
    ret = axs15260_touch_get_version(dev, &version);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read the touch firmware version, aborting init: %s", esp_err_to_name(ret));
        axs15260_touch_del(dev);
        return ret;
    }
    ESP_LOGI(TAG, "Firmware version: 0x%04X", version);

    if (dev->int_gpio >= 0) {
        // The ISR wakes the internal task, which reads the data and invokes the
        // registered callback in task context. Create the task first so no INT
        // notification can be lost before it exists.
        if (xTaskCreate(axs15260_touch_task, "axs_touch", 4096, dev, 5, &dev->task) != pdPASS) {
            ESP_LOGE(TAG, "Failed to create the AXS15260 touch task");
            axs15260_touch_del(dev);
            return ESP_ERR_NO_MEM;
        }

        ret = gpio_isr_handler_add(dev->int_gpio, touch_isr, dev);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to register the touch ISR: %s", esp_err_to_name(ret));
            axs15260_touch_del(dev);
            return ret;
        }
        dev->flags.isr_registered = 1;
    }

    dev->flags.inited = 1;
    *handle = dev;

    ESP_LOGI(TAG, "AXS15260 touch controller driver created (resolution: %dx%d)", dev->x_max, dev->y_max);
    return ESP_OK;
}

esp_err_t axs15260_touch_del(axs15260_touch_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    ESP_LOGI(TAG, "Deleting the AXS15260 touch controller driver...");

    // Remove the interrupt
    if (handle->int_gpio >= 0 && handle->flags.isr_registered) {
        gpio_intr_disable(handle->int_gpio);
        gpio_isr_handler_remove(handle->int_gpio);
        handle->flags.isr_registered = 0;
    }
    if (handle->int_gpio >= 0) {
        gpio_reset_pin(handle->int_gpio);
    }

    // Delete the internal touch task (now that the ISR is removed)
    if (handle->task) {
        vTaskDelete(handle->task);
        handle->task = NULL;
    }

    // Release the reset pin
    if (handle->rst_gpio >= 0) {
        gpio_reset_pin(handle->rst_gpio);
    }

    // Release I2C (an external bus only gets its device removed; a self-created
    // bus is destroyed)
    if (handle->i2c_dev) {
        i2c_master_bus_rm_device(handle->i2c_dev);
    }
    if (handle->i2c_bus && handle->flags.owns_bus) {
        i2c_del_master_bus(handle->i2c_bus);
    }

    // Delete the mutex
    if (handle->lock) {
        vSemaphoreDelete(handle->lock);
    }

    free(handle);
    ESP_LOGI(TAG, "Touch controller driver deleted");
    return ESP_OK;
}

esp_err_t axs15260_touch_reset(axs15260_touch_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    if (handle->rst_gpio < 0) {
        ESP_LOGD(TAG, "No reset pin configured, skipping reset");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Performing a hardware reset...");

    // Reset sequence: high -> low -> high
    gpio_set_level(handle->rst_gpio, 1);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(handle->rst_gpio, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(handle->rst_gpio, 1);
    vTaskDelay(pdMS_TO_TICKS(110));

    ESP_LOGI(TAG, "Hardware reset complete");
    return ESP_OK;
}

esp_err_t axs15260_touch_read(axs15260_touch_handle_t handle, axs15260_touch_data_t *data)
{
    ESP_RETURN_ON_FALSE(handle && data, ESP_ERR_INVALID_ARG, TAG, "Invalid argument");

    memset(data, 0, sizeof(*data));

    // Take the mutex
    if (xSemaphoreTake(handle->lock, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    // Read the touch data
    memset(handle->buf, 0xFF, AXS15260_TOUCH_BUF_SIZE);
    esp_err_t ret = touch_i2c_read(handle, handle->buf, AXS15260_TOUCH_BUF_SIZE);
    if (ret != ESP_OK) {
        bool was_pressed = handle->flags.last_pressed;
        bool recovered = touch_record_invalid(handle, esp_err_to_name(ret), NULL);
        if (recovered) {
            handle->flags.last_pressed = 0;
            if (was_pressed) {
                touch_fill_release(handle, data);
                ret = ESP_OK;
            }
        }
        xSemaphoreGive(handle->lock);
        return ret;
    }

    uint8_t *buf = handle->buf;

    // An all-0xFF frame is the normal idle frame when the AXS15260 has no new
    // data for the current interrupt.
    if (touch_frame_is_all_ff(buf)) {
        xSemaphoreGive(handle->lock);
        return ESP_ERR_NOT_FOUND;
    }

    // Data validity check. Normal frame: buf[0] = gesture ID (usually 0x00),
    // low nibble of buf[1] = touch point count (0-5).
    uint8_t gesture = buf[0];
    uint8_t point_byte = buf[1];
    uint8_t point_num = point_byte & 0x0F;

    uint8_t esd_flag = point_byte >> 4;
    if (gesture != 0 || esd_flag != 0 || point_num > AXS15260_TOUCH_MAX_POINTS) {
        bool was_pressed = handle->flags.last_pressed;
        bool recovered = touch_record_invalid(handle, "invalid status field", buf);
        if (recovered) {
            handle->flags.last_pressed = 0;
            if (was_pressed) {
                touch_fill_release(handle, data);
                xSemaphoreGive(handle->lock);
                return ESP_OK;
            }
        }
        xSemaphoreGive(handle->lock);
        return ESP_ERR_INVALID_RESPONSE;
    }

    handle->invalid_count = 0;
    data->gesture_id = gesture;

    if (point_num == 0) {
        if (handle->flags.last_pressed) {
            touch_fill_release(handle, data);
            handle->flags.last_pressed = 0;
            ESP_LOGD(TAG, "Touch released, source: empty point frame");
            xSemaphoreGive(handle->lock);
            return ESP_OK;
        }
        xSemaphoreGive(handle->lock);
        return ESP_ERR_NOT_FOUND;
    }

    // Parse the touch point (only the first one, as the buffer is 8 bytes)
    uint16_t x = ((buf[2] & 0x0F) << 8) | buf[3];
    uint16_t y = ((buf[4] & 0x0F) << 8) | buf[5];
    uint8_t event = buf[2] >> 6;

    if (x >= AXS15260_TOUCH_H_RES || y >= AXS15260_TOUCH_V_RES ||
        event > AXS15260_TOUCH_EVT_CONTACT) {
        bool was_pressed = handle->flags.last_pressed;
        bool recovered = touch_record_invalid(handle,
                                              event > AXS15260_TOUCH_EVT_CONTACT ?
                                              "invalid event field" : "physical coordinate out of range",
                                              buf);
        if (recovered) {
            handle->flags.last_pressed = 0;
            if (was_pressed) {
                touch_fill_release(handle, data);
                xSemaphoreGive(handle->lock);
                return ESP_OK;
            }
        }
        xSemaphoreGive(handle->lock);
        return ESP_ERR_INVALID_RESPONSE;
    }

    // Coordinate transformation
    if (handle->flags.swap_xy) {
        uint16_t tmp = x; x = y; y = tmp;
    }
    if (handle->flags.mirror_x) {
        x = handle->x_max - 1 - x;
    }
    if (handle->flags.mirror_y) {
        y = handle->y_max - 1 - y;
    }

    data->point_num = 1;
    data->points[0].x = x;
    data->points[0].y = y;
    data->points[0].event = event;
    data->points[0].id = buf[4] >> 4;
    data->points[0].weight = buf[6];
    data->points[0].area = buf[7] >> 4;

    bool pressed = event != AXS15260_TOUCH_EVT_UP;
    if (pressed != handle->flags.last_pressed) {
        ESP_LOGD(TAG,
                 "Touch %s, event=%u, coordinate=(%u,%u), raw data="
                 "%02X %02X %02X %02X %02X %02X %02X %02X",
                 pressed ? "pressed" : "released", event, x, y,
                 buf[0], buf[1], buf[2], buf[3],
                 buf[4], buf[5], buf[6], buf[7]);
    }
    handle->last_x = x;
    handle->last_y = y;
    handle->flags.last_pressed = pressed;

    xSemaphoreGive(handle->lock);
    return ESP_OK;
}

esp_err_t axs15260_touch_get_version(axs15260_touch_handle_t handle, uint16_t *version)
{
    ESP_RETURN_ON_FALSE(handle && version, ESP_ERR_INVALID_ARG, TAG, "Invalid argument");

    uint8_t cmd = REG_VERSION;
    uint8_t data[2] = {0};

    esp_err_t ret = touch_i2c_write_read(handle, &cmd, 1, data, 2);
    if (ret != ESP_OK) {
        return ret;
    }

    *version = (data[0] << 8) | data[1];
    return ESP_OK;
}

esp_err_t axs15260_touch_register_cb(axs15260_touch_handle_t handle,
                                     axs15260_touch_data_cb_t callback,
                                     void *user_data)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    if (handle->int_gpio < 0) {
        ESP_RETURN_ON_FALSE(callback == NULL, ESP_ERR_NOT_SUPPORTED, TAG,
                            "Touch interrupt pin not configured");
        return ESP_OK;
    }

    handle->callback = callback;
    handle->user_data = user_data;
    return ESP_OK;
}

esp_err_t axs15260_touch_set_swap_xy(axs15260_touch_handle_t handle,
                                     bool swap_xy, bool mirror_x, bool mirror_y)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    handle->flags.swap_xy = swap_xy;
    handle->flags.mirror_x = mirror_x;
    handle->flags.mirror_y = mirror_y;

    ESP_LOGI(TAG, "Coordinate transformation: swap_xy=%d, mirror_x=%d, mirror_y=%d",
             swap_xy, mirror_x, mirror_y);
    return ESP_OK;
}

bool axs15260_touch_is_pressed(axs15260_touch_handle_t handle)
{
    if (!handle || handle->int_gpio < 0) {
        return false;
    }
    // A low level on the interrupt pin means there is a touch
    return gpio_get_level(handle->int_gpio) == 0;
}
