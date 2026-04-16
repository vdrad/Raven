/**
 * @file line_markers.h
 * @brief Public interface for track marker and crossing detection.
 * * This module processes lateral sensor readings to detect track markers 
 * (left/right) and intersections (both). It utilizes a peak-hold state 
 * machine to prevent false positives and contact bouncing.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "line_configs.h"

/* ========================================================================== */
/* PUBLIC ENUMS & STRUCTURES                                                  */
/* ========================================================================== */

/**
 * @brief Represents the logical state of the marker sensors.
 */
typedef enum {
    LINE_MARKER_NONE = 0,
    LINE_MARKER_LEFT,
    LINE_MARKER_RIGHT,
    LINE_MARKER_BOTH      /**< Represents a crossing or intersection */
} line_marker_status_t;

/**
 * @brief Encapsulated state containing marker telemetry and event counters.
 */
typedef struct {
    line_marker_status_t marker_status; /**< Instantaneous status of the markers */
    uint8_t left_markers_counter;       /**< Total left markers detected */
    uint8_t right_markers_counter;      /**< Total right markers detected */
    uint8_t crossings_counter;          /**< Total intersections detected */
} line_markers_data_t;

/* ========================================================================== */
/* PUBLIC API                                                                 */
/* ========================================================================== */

/**
 * @brief Processes the marker sensors and updates internal counters.
 * * @param normalized_readings Array of calibrated readings from the lateral sensors.
 */
void line_markers_update(uint16_t normalized_readings[NUMBER_OF_MARKER_SENSORS]);

/**
 * @brief Retrieves the latest marker telemetry and counters.
 * * @return line_markers_data_t Struct containing counters and instantaneous state.
 */
line_markers_data_t line_markers_get_data(void);

/**
 * @brief Resets all marker and crossing counters to zero.
 */
void line_markers_reset_counters(void);