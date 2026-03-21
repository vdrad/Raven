#pragma once

// INCLUDES
#include <stdint.h>

// FUNCTIONS
void rgb_led_init(void);
void rgb_led_set_color(uint8_t index, uint8_t red, uint8_t green, uint8_t blue);
void rgb_led_set_all_colors(uint8_t red, uint8_t green, uint8_t blue);
void rgb_led_clear();
void rgb_led_show(void);