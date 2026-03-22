/**
 * @file rgb_led.c
 * @brief Addressable RGB LED (WS2812/Neopixel) driver using the ESP32 RMT peripheral.
 * * This module manages a string of addressable LEDs using hardware acceleration (RMT).
 * It separates the memory update operations (fast) from the physical transmission (slow)
 * to prevent CPU blocking/overhang times.
 */

#include "rgb_led.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "pinout.h"
#include "freertos/FreeRTOS.h"
#include "driver/rmt_tx.h"
#include "led_strip_encoder.h"
#include "raven_log.h"
#include "raven_comm.h"
#include "colors.h"

#define TAG "LED"
static bool initialized = false;

/* --- Macros & Configuration --- */
// 10MHz resolution means 1 tick = 0.1us. Essential for the strict WS2812 timing protocol.
#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 
#define RMT_LED_STRIP_GPIO_NUM      RGB_LED_PIN
#define NUMBER_OF_LEDS              20

/* --- Global Variables --- */
/**
 * @brief Hardware mapping correction array.
 * Maps logical LED indexes (0-19) to their physical positions on the hardware.
 * Useful for irregular LED layouts, folded strips, or custom PCB rings.
 */
static const uint8_t corrected_led_indexes[NUMBER_OF_LEDS] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 18, 17, 16, 15, 14, 13, 12, 11, 10, 19
};

// RAM buffer holding the current color state of all LEDs (Format: GRB)
static uint8_t led_pixels[NUMBER_OF_LEDS * 3];

// RMT peripheral handles
static rmt_channel_handle_t led_chan = NULL;
static rmt_encoder_handle_t led_encoder = NULL;
static rmt_transmit_config_t tx_config = {
    .loop_count = 0, // Ensure the transmission only happens once per call
};

/* --- Functions --- */

/**
 * @brief Initializes the RMT peripheral and configures the LED strip encoder.
 * Must be called once before any other LED functions.
 */
void rgb_led_init(void) {
    if (initialized) return;

    // 1. Create RMT TX Channel
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, 
        .gpio_num = RMT_LED_STRIP_GPIO_NUM,
        .mem_block_symbols = 64, 
        .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
        .trans_queue_depth = 4, // Allows up to 4 transmissions to queue up in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_chan));

    // 2. Install LED strip encoder (Translates raw bytes into WS2812 pulse widths)
    led_strip_encoder_config_t encoder_config = {
        .resolution = RMT_LED_STRIP_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&encoder_config, &led_encoder));

    // 3. Enable RMT TX channel
    ESP_ERROR_CHECK(rmt_enable(led_chan));

    // 4. Clear the RAM buffer and push to hardware to ensure LEDs are off on boot
    memset(led_pixels, 0, sizeof(led_pixels));
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_pixels, sizeof(led_pixels), &tx_config));

    raven_comm_send_message(TAG, "Initialized successfully.");
    initialized = true;
}

/**
 * @brief Sets the color of a specific LED in the RAM buffer.
 * * @note This function DOES NOT update the physical LED. You must call `rgb_led_show()` after.
 * * @param index Logical index of the LED (0 to NUMBER_OF_LEDS - 1).
 * @param red   Red intensity (0-255).
 * @param green Green intensity (0-255).
 * @param blue  Blue intensity (0-255).
 */
void rgb_led_set_color(uint8_t index, uint8_t red, uint8_t green, uint8_t blue) {
    // Translate logical index to physical index based on the hardware layout
    index = corrected_led_indexes[index];
    
    // Bounds check to prevent buffer overflow memory corruption
    if (index >= NUMBER_OF_LEDS) return;

    // WS2812 protocol requires colors in Green-Red-Blue (GRB) order
    led_pixels[index * 3 + 1] = red;
    led_pixels[index * 3 + 0] = green;
    led_pixels[index * 3 + 2] = blue; 
}

/**
 * @brief Sets all LEDs in the RAM buffer to the same color simultaneously.
 * * @note This function DOES NOT update the physical LEDs. You must call `rgb_led_show()` after.
 * * @param red   Red intensity (0-255).
 * @param green Green intensity (0-255).
 * @param blue  Blue intensity (0-255).
 */
void rgb_led_set_all_colors(uint8_t red, uint8_t green, uint8_t blue) {
    for (uint8_t i = 0; i < NUMBER_OF_LEDS; i++) {
        led_pixels[i * 3 + 1] = red;
        led_pixels[i * 3 + 0] = green;
        led_pixels[i * 3 + 2] = blue; 
    }
}

/**
 * @brief Pushes the current RAM buffer to the physical LED strip via the RMT peripheral.
 * * This is a non-blocking (fire-and-forget) function. The RMT hardware takes over
 * the transmission, allowing the FreeRTOS task to continue immediately.
 */
void rgb_led_show(void) {
    if (!initialized) {
        raven_comm_send_message(TAG, "Not initialized!");
        return;
    }

    // Transmit the buffer to the hardware queue
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_pixels, sizeof(led_pixels), &tx_config));
}

/**
 * @brief Clears the RAM buffer (sets all LEDs to black/off).
 * * @note This function DOES NOT update the physical LEDs. You must call `rgb_led_show()` after.
 */
void rgb_led_clear(void) {
    if (!initialized) {
        raven_comm_send_message(TAG, "Not initialized!");
        return; 
    }   

    // Fast memory operation to zero out the entire buffer
    memset(led_pixels, 0, sizeof(led_pixels)); 
}

/**
 * @brief Simple test routine to validate the LED peripheral hardware.
 * Sets all LEDs to purple and pushes the update immediately.
 * Requires `COLOR_PURPLE` macro to expand to three uint8_t values (R, G, B).
 */
void rgb_led_peripheral_validation(void) {
    rgb_led_set_all_colors(COLOR_PURPLE);
    rgb_led_show();
}