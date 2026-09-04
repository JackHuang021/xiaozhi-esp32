#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

#include <driver/gpio.h>
#include <driver/i2c_types.h>

// ES8389
#define AUDIO_INPUT_SAMPLE_RATE 16000
#define AUDIO_OUTPUT_SAMPLE_RATE 16000

#define AUDIO_I2S_GPIO_MCLK GPIO_NUM_NC
#define AUDIO_I2S_GPIO_BCLK GPIO_NUM_12
#define AUDIO_I2S_GPIO_WS GPIO_NUM_10
#define AUDIO_I2S_GPIO_DOUT GPIO_NUM_9
#define AUDIO_I2S_GPIO_DIN GPIO_NUM_11
#define AUDIO_CODEC_PA_PIN GPIO_NUM_31

#define AUDIO_CODEC_I2C_PORT I2C_NUM_0
#define AUDIO_CODEC_I2C_SDA_PIN GPIO_NUM_7
#define AUDIO_CODEC_I2C_SCL_PIN GPIO_NUM_8

#define AUDIO_CODEC_ES8389_ADDR ES8389_CODEC_DEFAULT_ADDR

#define BOOT_BUTTON_GPIO GPIO_NUM_35

// AXS15260 6.2" MIPI-DSI display (physical 452x1280 portrait).
// DISPLAY_WIDTH/HEIGHT are the physical resolution; after LVGL 90 degree
// rotation the effective resolution is 1280x452 (landscape).
#define LCD_PHYSICAL_WIDTH  452
#define LCD_PHYSICAL_HEIGHT 1280
#define DISPLAY_WIDTH  LCD_PHYSICAL_WIDTH
#define DISPLAY_HEIGHT LCD_PHYSICAL_HEIGHT
#define DISPLAY_OFFSET_X 0
#define DISPLAY_OFFSET_Y 0
#define DISPLAY_MIRROR_X false
#define DISPLAY_MIRROR_Y false
#define DISPLAY_SWAP_XY false
#define DISPLAY_RESET_PIN GPIO_NUM_26
#define DISPLAY_BACKLIGHT_PIN GPIO_NUM_30
#define DISPLAY_BACKLIGHT_OUTPUT_INVERT false

// MIPI DSI lane count and bit rate are controlled by AXS15260_MIPI_LANES and
// AXS15260_HSCLK_MBPS in esp_lcd_axs15260.h (kept in sync with the macros).
#define MIPI_DSI_PHY_PWR_LDO_CHAN 3
#define MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV 2500

// AXS15260 built-in touch and AXP2101 PMU share one I2C bus (I2C1, SDA=28/SCL=29).
// The codec is on the other bus (I2C0, SDA=7/SCL=8). Touch INT pin is GPIO27 and
// the touch reset is shared with the LCD on GPIO26.
#define TOUCH_I2C_PORT    I2C_NUM_1
#define TOUCH_I2C_SDA_PIN GPIO_NUM_28
#define TOUCH_I2C_SCL_PIN GPIO_NUM_29
#define TOUCH_INT_PIN     GPIO_NUM_27

#define SD_CARD_MOUNT_POINT "/sdcard"
#define SD_CARD_PWR_LDO_CHAN 4

#define PMU_I2C_ADDR 0x34

#endif  // _BOARD_CONFIG_H_
