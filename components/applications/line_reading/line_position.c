/**
 * @file line_position.c
 * @brief Line position calculation and tracking logic.
 * * This module processes normalized line sensor readings to calculate
 * the robot's physical position relative to the line's center. It also
 * tracks the time elapsed since the line was last seen to determine if
 * the robot is lost.
 */

#include "line_position.h"

// Standard Libraries
#include <math.h>

// ESP-IDF
#include "esp_timer.h"

// Project Includes
#include "line_reading.h"
#include "line_calibration.h"

#define TAG "LIN"

/* ========================================================================== */
/* CONFIGURATION MACROS                                                       */
/* ========================================================================== */

/** * @brief Minimum normalized reading to consider the robot is over the line.
 * Tune: Choose a value safely between track noise and a weak line edge reading. 
 */
#define LINE_POSITION_ON_LINE_THRESHOLD     ((uint16_t)round(0.30f  * CALIBRATION_MAX_VALUE))

/** * @brief Minimum normalized reading to be considered valid signal.
 * Tune: Read normalized values when all sensors are on the track surface (no line). 
 * Choose a value slightly above the maximum observed noise.
 */
#define LINE_POSITION_NOISE_THRESHOLD       ((uint16_t)round(0.065f * CALIBRATION_MAX_VALUE))

#define LINE_POSITION_CENTRAL_SENSOR_INDEX  5
#define LINE_POSITION_SENSOR_SPACING_MM     6.4f
#define LINE_POSITION_MAX_DISTANCE_MM       (LINE_POSITION_CENTRAL_SENSOR_INDEX * LINE_POSITION_SENSOR_SPACING_MM)

/** @brief Time in milliseconds before the robot is declared 'lost' after losing the line. */
#define LINE_POSITION_LOST_THRESHOLD_MS     1500

/* ========================================================================== */
/* PRIVATE VARIABLES                                                          */
/* ========================================================================== */

/** @brief Encapsulated state containing the current position and tracking status. */
static line_position_data_t current_line_position = {
    .position      = 0,
    .robot_on_line = false,
    .robot_lost    = false
};

/** @brief Hardware timestamp (in microseconds) of the last time the line was detected. */
static int64_t last_time_on_line_us = 0;

/* ========================================================================== */
/* PRIVATE FUNCTIONS                                                          */
/* ========================================================================== */

/**
 * @brief Evaluates if the robot has been off the line longer than the defined threshold.
 * * @param is_on_line Current physical detection status of the line.
 * @return true if the robot has been blind longer than LINE_POSITION_LOST_THRESHOLD_MS.
 * @return false if the robot is currently on the line or recently lost it.
 */
static bool check_if_robot_is_lost(bool is_on_line) {
    int64_t current_time_us = esp_timer_get_time();
    bool is_lost = false;

    if (is_on_line) {
        // Robot sees the line, reset the tracking timer
        last_time_on_line_us = current_time_us;
    } else {
        // Robot does not see the line, evaluate how long it has been blind
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

/**
 * @brief Computes the center of mass of the line and updates the tracking state.
 * * @param normalized_readings Array containing the calibrated readings from the frontal sensor array.
 */
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

    // 2. Calculate the final physical position
    float calculated_position;

    if (is_on_line && sum_of_readings > 0) {
        // Standard center of mass calculation
        calculated_position = weighted_sum / sum_of_readings;
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

/**
 * @brief Retrieves the latest computed line position and tracking flags.
 * * @return line_position_data_t Struct containing position, on_line flag, and lost flag.
 */
line_position_data_t line_position_get_data(void) {
    return current_line_position;
}