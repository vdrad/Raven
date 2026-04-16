/**
 * @file line_position.h
 * @brief Public interface for the line position and tracking module.
 * * This header defines the data structures and public functions required to 
 * calculate the robot's center of mass relative to the line and check its 
 * tracking status.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

/* Include necessary headers for array size macros (e.g., NUMBER_OF_FRONTAL_SENSORS) */
#include "line_configs.h" 

/* ========================================================================== */
/* PUBLIC STRUCTURES                                                          */
/* ========================================================================== */

/**
 * @brief Represents the current tracking state and position of the robot.
 */
typedef struct {
    float position;      /**< Physical position from the center of the line in mm or weighted units. 
                                Negative = Left, Positive = Right, 0 = Centered. */
    bool robot_on_line;    /**< True if at least one sensor detects the line above the threshold. */
    bool robot_lost;       /**< True if the robot has been off the line for longer than the lost threshold. */
} line_position_data_t;

/* ========================================================================== */
/* PUBLIC API                                                                 */
/* ========================================================================== */

/**
 * @brief Computes the center of mass of the line and updates the tracking state.
 * * This function processes the array of calibrated sensor readings, applies noise 
 * filtering, calculates the weighted position, and manages the memory of the last 
 * known direction when the line is lost.
 * * @param normalized_readings Array containing the calibrated readings from the frontal sensor array (0 to LINE_CALIBRATION_MAX_VALUE).
 */
void line_position_update(uint16_t normalized_readings[NUMBER_OF_FRONTAL_SENSORS]);

/**
 * @brief Retrieves the latest computed line position and tracking flags.
 * * This function provides thread-safe access (via atomic struct return) to the 
 * internal tracking state.
 * * @return line_position_data_t Struct containing the current position, on_line flag, and lost flag.
 */
line_position_data_t line_position_get_data(void);