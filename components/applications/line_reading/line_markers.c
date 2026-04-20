/**
 * @file line_markers.c
 * @brief Implementation of the marker detection logic using peak-hold.
 */

#include "line_markers.h"

// Standard Libraries
#include <stdbool.h>

// Project Includes
#include "line_calibration.h" 
#include "buzzer.h"

#define TAG "LIN"

/* ========================================================================== */
/* CONFIGURATION MACROS                                                       */
/* ========================================================================== */

/** @brief Threshold (0 to CALIBRATION_MAX_VALUE) to consider a lateral sensor active. */
#define LINE_MARKERS_DIGITAL_THRESHOLD ((uint16_t)(0.65f * LINE_CALIBRATION_MAX_VALUE))

/** * Define the logic gate used for lateral detection.
 * - REQUIRE_ANY (OR): Highly sensitive, triggers if at least one sensor sees the tape.
 * - REQUIRE_ALL (AND): Highly robust, triggers only if both side-by-side sensors see the tape.
 */
// #define LINE_MARKERS_REQUIRE_ALL
#define LINE_MARKERS_REQUIRE_ANY

#define LINE_MARKERS_DETECTION_NOTE_FREQUENCY     NOTE_B7
#define LINE_MARKERS_DETECTION_NOTE_DURATION_MS   120

#define LINE_MARKER_MINIMUM_LEFT_MARKERS_TO_COUNT 20

/* ========================================================================== */
/* PRIVATE VARIABLES                                                          */
/* ========================================================================== */

static line_markers_data_t current_data = {
    .marker_status         = LINE_MARKER_NONE,
    .left_markers_counter  = 0,
    .right_markers_counter = 0,
    .crossings_counter     = 0
};

/** @brief Tracks the highest priority state achieved during a single marker pass. */
static line_marker_status_t peak_active_marker = LINE_MARKER_NONE;

/* ========================================================================== */
/* PRIVATE FUNCTIONS                                                          */
/* ========================================================================== */

/**
 * @brief Converts analog normalized readings into a logical state.
 * * Assumes physical mapping: Left = [0, 1], Right = [2, 3].
 */
static line_marker_status_t instantaneous_read(uint16_t normalized_readings[NUMBER_OF_MARKER_SENSORS]) {
    bool digital_readings[NUMBER_OF_MARKER_SENSORS];
    bool is_left_detected = false;
    bool is_right_detected = false;

    // 1. Threshold all marker sensors
    for (uint8_t i = 0; i < NUMBER_OF_MARKER_SENSORS; i++) {
        digital_readings[i] = (normalized_readings[i] >= LINE_MARKERS_DIGITAL_THRESHOLD);
    }

    // 2. Apply chosen logic gate to group side-by-side sensors
#ifdef LINE_MARKERS_REQUIRE_ANY
    is_left_detected  = (digital_readings[0] || digital_readings[1]);
    is_right_detected = (digital_readings[2] || digital_readings[3]);
#else // LINE_MARKERS_REQUIRE_ALL
    is_left_detected  = (digital_readings[0] && digital_readings[1]);
    is_right_detected = (digital_readings[2] && digital_readings[3]);
#endif

    // 3. Resolve final state
    if (is_left_detected  && !is_right_detected) return LINE_MARKER_LEFT;
    if (!is_left_detected && is_right_detected)  return LINE_MARKER_RIGHT;
    if (is_left_detected  && is_right_detected)  return LINE_MARKER_BOTH;
    
    return LINE_MARKER_NONE;
}

/* ========================================================================== */
/* PUBLIC API                                                                 */
/* ========================================================================== */

void line_markers_update(uint16_t normalized_readings[NUMBER_OF_MARKER_SENSORS]) {
    line_marker_status_t instantaneous_status = instantaneous_read(normalized_readings);

    // 1. We are currently over a marker (Active Window)
    if (instantaneous_status != LINE_MARKER_NONE) {
        
        // Upgrade logic: BOTH (Crossing) is the highest priority state.
        if (instantaneous_status == LINE_MARKER_BOTH) {
            peak_active_marker = LINE_MARKER_BOTH;
        } 
        // If the window just opened, log the first thing we see.
        else if (peak_active_marker == LINE_MARKER_NONE) {
            peak_active_marker = instantaneous_status;
        }
    } 
    // 2. We just left the marker (Falling Edge Window)
    else if (peak_active_marker != LINE_MARKER_NONE) {
        
        // Tally the score based on the highest state achieved during the pass
        if (peak_active_marker == LINE_MARKER_LEFT) {
            current_data.left_markers_counter++;
            buzzer_play(
                LINE_MARKERS_DETECTION_NOTE_FREQUENCY, 
                LINE_MARKERS_DETECTION_NOTE_DURATION_MS
            );
        } 
        else if (peak_active_marker == LINE_MARKER_RIGHT) {
            // If it's the 1st right marker, count it unconditionally
            if (current_data.right_markers_counter == 0) {
                current_data.right_markers_counter++;
            } 
            // If we are waiting for the 2nd right marker (or beyond), enforce the left marker minimum
            else if (current_data.left_markers_counter >= LINE_MARKER_MINIMUM_LEFT_MARKERS_TO_COUNT) {
                current_data.right_markers_counter++;
            }
        } 
        else if (peak_active_marker == LINE_MARKER_BOTH) {
            current_data.crossings_counter++;
        }

        // Close the window and wait for the next physical marker on the track
        peak_active_marker = LINE_MARKER_NONE;
    }
    
    // 3. Update the global encapsulated state for telemetry
    current_data.marker_status = instantaneous_status;
}

line_markers_data_t line_markers_get_data(void) {
    return current_data;
}

void line_markers_reset_counters(void) {
    current_data.left_markers_counter = 0;
    current_data.right_markers_counter = 0;
    current_data.crossings_counter = 0;
}