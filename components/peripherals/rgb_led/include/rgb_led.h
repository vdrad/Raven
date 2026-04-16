/**
 * @file rgb_led.h
 * @brief Addressable RGB LED (WS2812/Neopixel) driver API.
 *
 * This header defines the interfaces for initializing, coloring, 
 * and updating the addressable LED strip using the ESP32 RMT peripheral.
 */

#pragma once

#include <stdint.h>
#include "colors.h"

/**
 * @brief Initializes the RMT peripheral and configures the LED strip.
 */
void rgb_led_init(void);

/**
 * @brief Sets a specific LED to a given RGB value in the RAM buffer.
 * @param index The physical index of the LED to change.
 * @param color The rgb_color_t struct containing the target color.
 */
void rgb_led_set_color(uint8_t index, rgb_color_t color);

/**
 * @brief Sets all LEDs to the same RGB value in the RAM buffer.
 * @param color The rgb_color_t struct containing the target color.
 */
void rgb_led_set_all_colors(rgb_color_t color);

/**
 * @brief Clears the RAM buffer (sets all LEDs to black/off).
 * @note This function DOES NOT update the physical LEDs. Call `rgb_led_show()` after.
 */
void rgb_led_clear(void);

/**
 * @brief Pushes the current RAM buffer to the physical LED strip via the RMT peripheral.
 */
void rgb_led_show(void);

/**
 * @brief Hardware validation sequence to verify LED functionality.
 */
void rgb_led_peripheral_validation(void);