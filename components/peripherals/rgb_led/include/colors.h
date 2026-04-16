/**
 * @file colors.h
 * @brief Predefined RGB color macros for the LED strip.
 *
 * Provides standardized 8-bit RGB values for consistent lighting 
 * across the robot's UI and telemetry states.
 */

#pragma once

typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} rgb_color_t;

#define COLOR_RED             (rgb_color_t){255,    0,     0}
#define COLOR_DARK_ORANGE     (rgb_color_t){254,    23,    0}  
#define COLOR_LIGHT_ORANGE    (rgb_color_t){255,    48,    0}
#define COLOR_YELLOW          (rgb_color_t){255,    115,   0}
#define COLOR_LIME_GREEN      (rgb_color_t){163,    251,   0}
#define COLOR_LIGHT_GREEN     (rgb_color_t){0,      220,   20}
#define COLOR_GREEN           (rgb_color_t){0,      230,   0}
#define COLOR_EMERALD         (rgb_color_t){0,      250,   40}
#define COLOR_CYAN            (rgb_color_t){0,      255,   255}
#define COLOR_LIGHT_BLUE      (rgb_color_t){0,      90,    255}
#define COLOR_BLUE            (rgb_color_t){0,      0,     255}
#define COLOR_PURPLE          (rgb_color_t){252,    3,     232}
#define COLOR_PINK            (rgb_color_t){240,    0,     80}
#define COLOR_SCARLET         (rgb_color_t){255,    0,     6}
#define COLOR_WHITE           (rgb_color_t){200,    200,   200}
#define COLOR_BLANK           (rgb_color_t){0,      0,     0}