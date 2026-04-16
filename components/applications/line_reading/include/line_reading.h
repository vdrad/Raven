/**
 * @file line_reading.h
 * @brief Master module for line detection, abstracting SPI reads, calibration, and position.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "line_configs.h"
#include "line_position.h"
#include "line_markers.h"

/* ========================================================================== */
/* PUBLIC DEFINITIONS & STRUCTURES                                            */
/* ========================================================================== */

/**
 * @brief Master struct containing all processed line data.
 * This is the only data structure the main control loop needs to read.
 */
typedef struct {
    line_position_data_t position;    /**< Center of mass and line tracking state */
    line_markers_data_t markers;      /**< Detected intersections and markers  */
    bool is_valid;                    /**< True if the SPI read was successful */
} line_reading_data_t;

/* ========================================================================== */
/* PUBLIC API                                                                 */
/* ========================================================================== */

void line_reading_init(void);
void line_reading_enable_sensors(void);
void line_reading_disable_sensors(void);

/**
 * @brief Master update function. Fetches SPI data, normalizes it, and updates position/markers.
 * @note Should be called at a fixed high frequency (e.g., 1ms or 2ms) from a FreeRTOS task.
 */
void line_reading_update(void);

/**
 * @brief Returns the latest processed line data (atomic/thread-safe).
 */
line_reading_data_t line_reading_get_data(void);

/**
 * @brief Calibrates line sensors for optimal reading performance.
 */
void line_reading_calibrate(void);

// Debug
void line_reading_raw_validation(void);
void line_reading_normalized_validation(void);
void line_reading_position_validation(void);
void line_reading_markers_validation(void);