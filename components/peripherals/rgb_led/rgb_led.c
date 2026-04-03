/**
 * @file rgb_led.c
 * @brief Addressable RGB LED (WS2812/Neopixel) driver using the ESP32 RMT peripheral.
 * * This module manages a string of addressable LEDs using hardware acceleration (RMT).
 * It separates the memory update operations (fast) from the physical transmission (slow)
 * to prevent CPU blocking/overhang times.
 */

#include "rgb_led.h"
#include "colors.h"

// Standard Library
#include <string.h>
#include <stdbool.h>

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ESP-IDF Drivers
#include "driver/rmt_tx.h"
#include "led_strip_encoder.h"

// Project
#include "pinout.h"
#include "raven_log.h"
#include "raven_comm.h"

#define TAG "LED"

/* --- Macros & Configuration --- */
#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz (0.1us ticks) for WS2812 strict timing
#define RMT_LED_STRIP_GPIO_NUM      RGB_LED_PIN
#define NUMBER_OF_LEDS              20

/* --- Global Variables --- */
static bool initialized = false;

static rmt_channel_handle_t led_chan = NULL;
static rmt_encoder_handle_t led_encoder = NULL;
static rmt_transmit_config_t tx_config = {
    .loop_count = 0, // No looping, transmit once
};

// Internal GRB buffer (WS2812 standard requires Green, Red, Blue order)
static uint8_t led_pixels[NUMBER_OF_LEDS * 3];

/* ========================================================================== */
/* PUBLIC API IMPLEMENTATIONS                                                 */
/* ========================================================================== */

void rgb_led_init(void) {
    if (initialized) return;

    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = RMT_LED_STRIP_GPIO_NUM,
        .mem_block_symbols = 64,
        .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
        .trans_queue_depth = 4, 
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_chan));

    led_strip_encoder_config_t encoder_config = {
        .resolution = RMT_LED_STRIP_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&encoder_config, &led_encoder));
    ESP_ERROR_CHECK(rmt_enable(led_chan));

    rgb_led_clear();
    rgb_led_show();

    RAVEN_LOGI(TAG, "Initialized successfully.");
    initialized = true;
}

void rgb_led_set_color(uint8_t index, uint8_t red, uint8_t green, uint8_t blue) {
    if (index >= NUMBER_OF_LEDS) return;

    // Map to GRB format expected by WS2812
    led_pixels[index * 3 + 0] = green;
    led_pixels[index * 3 + 1] = red;
    led_pixels[index * 3 + 2] = blue;
}

void rgb_led_set_all_colors(uint8_t red, uint8_t green, uint8_t blue) {
    for (uint8_t i = 0; i < NUMBER_OF_LEDS; i++) {
        led_pixels[i * 3 + 0] = green;
        led_pixels[i * 3 + 1] = red;
        led_pixels[i * 3 + 2] = blue; 
    }
}

void rgb_led_clear(void) {
    if (!initialized) {
        raven_comm_send_message(TAG, "Not initialized!");
        return; 
    }   

    memset(led_pixels, 0, sizeof(led_pixels));
}

void rgb_led_show(void) {
    if (!initialized) {
        raven_comm_send_message(TAG, "Not initialized!");
        return;
    }

    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_pixels, sizeof(led_pixels), &tx_config));
}

void rgb_led_peripheral_validation(void) {
    if (!initialized) {
        raven_comm_send_message(TAG, "Not initialized!");
        return;
    }

    raven_comm_send_message(TAG, "Running LED color test sequence...");
    
    rgb_led_set_all_colors(COLOR_RED);
    rgb_led_show();
    vTaskDelay(pdMS_TO_TICKS(500));

    rgb_led_set_all_colors(COLOR_GREEN);
    rgb_led_show();
    vTaskDelay(pdMS_TO_TICKS(500));

    rgb_led_set_all_colors(COLOR_BLUE);
    rgb_led_show();
    vTaskDelay(pdMS_TO_TICKS(500));

    rgb_led_clear();
    rgb_led_show();
    raven_comm_send_message(TAG, "LED test complete.");
}