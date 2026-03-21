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

#define TAG "LEDS"
static bool initialized = false;

// MACROS
#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution, 1 tick = 0.1us
#define RMT_LED_STRIP_GPIO_NUM      RGB_LED_PIN
#define NUMBER_OF_LEDS              20

// GLOBALS
static const uint8_t corrected_led_indexes[NUMBER_OF_LEDS] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 18, 17, 16, 15, 14, 13, 12, 11, 10, 19
};
static uint8_t led_pixels[NUMBER_OF_LEDS * 3];
static rmt_channel_handle_t led_chan = NULL;
static rmt_encoder_handle_t led_encoder = NULL;
static rmt_transmit_config_t tx_config = {
    .loop_count = 0, // no transfer loop
};

// FUNCTIONS
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

    // 2. Install LED strip encoder
    led_strip_encoder_config_t encoder_config = {
        .resolution = RMT_LED_STRIP_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&encoder_config, &led_encoder));

    // 3. Enable RMT TX channel
    ESP_ERROR_CHECK(rmt_enable(led_chan));

    // Clear the buffer and ensure LEDs are off on boot
    memset(led_pixels, 0, sizeof(led_pixels));
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_pixels, sizeof(led_pixels), &tx_config));

    raven_comm_send_message(TAG, "Initialized successfully");
    initialized = true;
}

// you need to call rgb_led_show()
void rgb_led_set_color(uint8_t index, uint8_t red, uint8_t green, uint8_t blue) {
    index = corrected_led_indexes[index];
    if (index >= NUMBER_OF_LEDS) return;

    led_pixels[index * 3 + 1] = red;
    led_pixels[index * 3 + 0] = green;
    led_pixels[index * 3 + 2] = blue; 
}

// you need to call rgb_led_show()
void rgb_led_set_all_colors(uint8_t red, uint8_t green, uint8_t blue) {
    for (uint8_t i = 0; i < 3; i++) {
        for (uint8_t j = i; j < NUMBER_OF_LEDS; j += 3) {
            led_pixels[j * 3 + 1] = red;
            led_pixels[j * 3 + 0] = green;
            led_pixels[j * 3 + 2] = blue; 
        }
    }
}

void rgb_led_show(void) {
    if (!initialized) {
        RAVEN_LOGE(TAG, "Not initialized!");
        return;
    }

    // Pushes the buffer to the hardware queue and returns immediately.
    // The RMT peripheral handles the strict timing in the background.
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_pixels, sizeof(led_pixels), &tx_config));
}

void rgb_led_clear() {
    if (!initialized) {
        RAVEN_LOGE(TAG, "Not initialized!");
        return;
    }   

    memset(led_pixels, 0, sizeof(led_pixels)); 
}