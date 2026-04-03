/**
 * @file rgb_led.h
 * @brief Addressable RGB LED (WS2812/Neopixel) driver API.
 *
 * This header defines the interfaces for initializing, coloring, 
 * and updating the addressable LED strip using the ESP32 RMT peripheral.
 */

#pragma once

#include <stdint.h>

/**
 * @brief Initializes the RMT peripheral and configures the LED strip.
 */
void rgb_led_init(void);

/**
 * @brief Sets a specific LED to a given RGB value in the RAM buffer.
 * * @param index The physical index of the LED to change.
 * @param red   Red brightness (0-255).
 * @param green Green brightness (0-255).
 * @param blue  Blue brightness (0-255).
 */
void rgb_led_set_color(uint8_t index, uint8_t red, uint8_t green, uint8_t blue);

/**
 * @brief Sets all LEDs to the same RGB value in the RAM buffer.
 * * @param red   Red brightness (0-255).
 * @param green Green brightness (0-255).
 * @param blue  Blue brightness (0-255).
 */
void rgb_led_set_all_colors(uint8_t red, uint8_t green, uint8_t blue);

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