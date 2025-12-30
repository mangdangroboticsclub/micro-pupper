#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

#include <driver/gpio.h>

#define AUDIO_INPUT_SAMPLE_RATE  24000
#define AUDIO_OUTPUT_SAMPLE_RATE 24000

#define AUDIO_INPUT_REFERENCE    true

#define AUDIO_I2S_GPIO_MCLK GPIO_NUM_38
#define AUDIO_I2S_GPIO_WS GPIO_NUM_13
#define AUDIO_I2S_GPIO_BCLK GPIO_NUM_14
#define AUDIO_I2S_GPIO_DIN  GPIO_NUM_12
#define AUDIO_I2S_GPIO_DOUT GPIO_NUM_45

#define AUDIO_CODEC_USE_PCA9557
#define AUDIO_CODEC_I2C_SDA_PIN  GPIO_NUM_1
#define AUDIO_CODEC_I2C_SCL_PIN  GPIO_NUM_2
#define AUDIO_CODEC_ES8311_ADDR  ES8311_CODEC_DEFAULT_ADDR
#define AUDIO_CODEC_ES7210_ADDR  0x82

// #define BUILTIN_LED_GPIO        GPIO_NUM_48
#define BOOT_BUTTON_GPIO        GPIO_NUM_0
#define WAKE_BUTTON_GPIO        GPIO_NUM_21
#define VOLUME_UP_BUTTON_GPIO   GPIO_NUM_NC
#define VOLUME_DOWN_BUTTON_GPIO GPIO_NUM_NC

#define DISPLAY_WIDTH   160
#define DISPLAY_HEIGHT  80
#define DISPLAY_MIRROR_X false
#define DISPLAY_MIRROR_Y false
#define DISPLAY_SWAP_XY true

#define DISPLAY_OFFSET_X  1
#define DISPLAY_OFFSET_Y  26

#define DISPLAY_BACKLIGHT_PIN GPIO_NUM_42
#define DISPLAY_BACKLIGHT_OUTPUT_INVERT false

// Add these missing definitions:
#define DISPLAY_INVERT_COLOR    true
#define DISPLAY_RGB_ORDER       LCD_RGB_ELEMENT_ORDER_BGR
#define DISPLAY_SPI_MODE        0

// Motor control definitions
// #define MOTOR_SPEED_MAX             100

#define LEDC_TIMER                  LEDC_TIMER_2
#define LEDC_MODE                   LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES               LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_FREQUENCY              (4000) // Frequency in Hertz. Set frequency at 4 kHz

#define LEDC_CHANNEL_COUNT          (4)
#define LEDC_M1_CHANNEL_A           LEDC_CHANNEL_1
#define LEDC_M1_CHANNEL_B           LEDC_CHANNEL_2
#define LEDC_M2_CHANNEL_A           LEDC_CHANNEL_3
#define LEDC_M2_CHANNEL_B           LEDC_CHANNEL_4

#define LEDC_M1_CHANNEL_A_IO        (19)
#define LEDC_M1_CHANNEL_B_IO        (20)
#define LEDC_M2_CHANNEL_A_IO        (47)
#define LEDC_M2_CHANNEL_B_IO        (48)

/* Camera pins */
#define CAMERA_PIN_PWDN -1
#define CAMERA_PIN_RESET -1
#define CAMERA_PIN_XCLK 5
#define CAMERA_PIN_SIOD 1
#define CAMERA_PIN_SIOC 2

#define CAMERA_PIN_D7 9
#define CAMERA_PIN_D6 4
#define CAMERA_PIN_D5 6
#define CAMERA_PIN_D4 15
#define CAMERA_PIN_D3 17
#define CAMERA_PIN_D2 8
#define CAMERA_PIN_D1 18
#define CAMERA_PIN_D0 16
#define CAMERA_PIN_VSYNC 3
#define CAMERA_PIN_HREF 46
#define CAMERA_PIN_PCLK 7

#define XCLK_FREQ_HZ 24000000


#endif // _BOARD_CONFIG_H_