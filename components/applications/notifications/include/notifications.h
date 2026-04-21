/**
 * @file notifications.h
 * @brief Non-blocking LED and Buzzer notification module using preset profiles.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "rgb_led.h"
#include "buzzer.h"
#include "notes.h"

// --- Patterns ---
typedef enum {
    NOTIFY_PATTERN_OFF = 0,
    NOTIFY_PATTERN_HOLD,                   // Does nothing, preserves current LED state
    NOTIFY_PATTERN_BREATHER,               // Fades side bars in and out smoothly
    NOTIFY_PATTERN_EXHAUSTS_SPOOL,         // Flickers exhausts, turns solid, rising pitch
    NOTIFY_PATTERN_SWEEP_TO_CENTER,        // Outer wings collapse to center
    NOTIFY_PATTERN_SWEEP_TO_EDGES,         // Center bursts to outer wings
    NOTIFY_PATTERN_SWEEP_TO_EDGES_SINGLE,  // Center bursts to outer wings one LED at each side
    NOTIFY_PATTERN_BLINK_EDGES,            // Blinks outermost LEDs
    NOTIFY_PATTERN_BLINK_EXHAUSTS,         // Blinks exhaust LEDs
    NOTIFY_PATTERN_GAUGE,                  // Progress bar
    NOTIFY_PATTERN_SOLID_ALL               // All LEDs solid (utility)
} notification_pattern_t;

#define NOTIFY_INFINITE -1

// --- Unified Configuration Structure ---
typedef struct {
    notification_pattern_t pattern;
    rgb_color_t color;
    uint32_t base_tone_hz;
    uint16_t speed_ms;
    int32_t repetitions;
    bool freeze_at_end;                /**< If true, LEDs remain in their final state instead of turning off */
    uint8_t payload;                   /**< Generic parameter */
} notification_config_t;

extern const notification_config_t NOTIFY_PROFILE_OFF;

/**
 * @brief Initializes the background notification task.
 */
void notifications_init(void);

/**
 * @brief Safely requests a new LED/Buzzer pattern override using a configuration profile.
 * @param config Pointer to the notification_config_t structure to play.
 */
void notification_play(const notification_config_t *config);

/**
 * @brief Test sequence to validate the patterns.
 */
void notifications_validation(void);