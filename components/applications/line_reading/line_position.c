/**
 * @file line_position.c
 * @brief Line position calculation and tracking logic with EMA filtering.
 */

#include "line_position.h"

// Standard Libraries
#include <math.h>

// ESP-IDF
#include "esp_timer.h"

// Project Includes
#include "line_configs.h"
#include "line_calibration.h"

#define TAG "LIN"

/* ========================================================================== */
/* CONFIGURATION MACROS                                                       */
/* ========================================================================== */

#define LINE_POSITION_ON_LINE_THRESHOLD     ((uint16_t)round(0.30f  * LINE_CALIBRATION_MAX_VALUE))
#define LINE_POSITION_NOISE_THRESHOLD       ((uint16_t)round(0.065f * LINE_CALIBRATION_MAX_VALUE))

#define LINE_POSITION_CENTRAL_SENSOR_INDEX  5
#define LINE_POSITION_SENSOR_SPACING_MM     6.4f
#define LINE_POSITION_MAX_DISTANCE_MM       (LINE_POSITION_CENTRAL_SENSOR_INDEX * LINE_POSITION_SENSOR_SPACING_MM)

#define LINE_POSITION_LOST_THRESHOLD_MS     1500

/** * @brief EMA Filter Alpha for line position. 
 * High alpha (0.7) prioritized to minimize phase lag at 8m/s speeds 
 * where the robot travels >6mm per millisecond.
 */
#define LINE_POSITION_EMA_ALPHA             0.8f

/* ========================================================================== */
/* PRIVATE VARIABLES                                                          */
/* ========================================================================== */

static line_position_data_t current_line_position = {
    .position      = 0.0f,
    .robot_on_line = false,
    .robot_lost    = false
};

static int64_t last_time_on_line_us = 0;

/* ========================================================================== */
/* PRIVATE FUNCTIONS                                                          */
/* ========================================================================== */

static bool check_if_robot_is_lost(bool is_on_line) {
    int64_t current_time_us = esp_timer_get_time();
    bool is_lost = false;

    if (is_on_line) {
        last_time_on_line_us = current_time_us;
    } else {
        int64_t time_blind_us = current_time_us - last_time_on_line_us;
        if (time_blind_us > (LINE_POSITION_LOST_THRESHOLD_MS * 1000LL)) {
            is_lost = true;
        }
    }

    return is_lost;
}

/* ========================================================================== */
/* PUBLIC API                                                                 */
/* ========================================================================== */

void line_position_update(uint16_t normalized_readings[NUMBER_OF_FRONTAL_SENSORS]) {
    bool is_on_line = false;
    float weighted_sum = 0.0f;
    uint32_t sum_of_readings = 0;

    // 1. Process all sensor readings to find the center of mass
    for (uint8_t i = 0; i < NUMBER_OF_FRONTAL_SENSORS; i++) {
        if (normalized_readings[i] > LINE_POSITION_ON_LINE_THRESHOLD) {
            is_on_line = true;
        }
        
        if (normalized_readings[i] > LINE_POSITION_NOISE_THRESHOLD) {
            float distance_from_center = (i - LINE_POSITION_CENTRAL_SENSOR_INDEX) * LINE_POSITION_SENSOR_SPACING_MM;
            
            weighted_sum    += normalized_readings[i] * distance_from_center;
            sum_of_readings += normalized_readings[i];
        }
    }

    // 2. Calculate the final physical position with EMA filtering
    float calculated_position;

    if (is_on_line && sum_of_readings > 0) {
        float raw_position = weighted_sum / sum_of_readings;
        
        // SNAP LOGIC: If the robot just found the line after being off-line, 
        // instantly snap to the raw reading to prevent the EMA from dragging 
        // the saturated max/min edge values.
        if (!current_line_position.robot_on_line) {
            calculated_position = raw_position;
        } else {
            // Standard EMA Filter
            calculated_position = (LINE_POSITION_EMA_ALPHA * raw_position) + 
                                  ((1.0f - LINE_POSITION_EMA_ALPHA) * current_line_position.position);
        }
    } else {
        // Line lost memory: saturate output to the last known extreme side
        if (current_line_position.position < 0) {
            calculated_position = -LINE_POSITION_MAX_DISTANCE_MM;
        } else {
            calculated_position = LINE_POSITION_MAX_DISTANCE_MM;
        }
    }
    
    // 3. Atomically update the global encapsulated state
    current_line_position.robot_lost    = check_if_robot_is_lost(is_on_line);  
    current_line_position.robot_on_line = is_on_line;
    current_line_position.position      = calculated_position;
}

line_position_data_t line_position_get_data(void) {
    return current_line_position;
}