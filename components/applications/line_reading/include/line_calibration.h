/**
 * @file line_calibration.h
 * @brief Implementation of the line calibration and normalization logic.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "line_reading.h"

/* ========================================================================== */
/* CONFIGURATIONS & MACROS                                                    */
/* ========================================================================== */

/** * @brief The maximum value after calibration. 
 * E.g., if set to 2000, outputs will range from 0 (background) to 2000 (line). 
 */
#define CALIBRATION_MAX_VALUE       2000

/** @brief Duration of the manual calibration process in milliseconds. */
#define CALIBRATION_DURATION_MS     5000

/**
 * @brief Color logic definitions.
 */
typedef enum {
    WHITE_LINE = 0,             /**< White line on black background (White is Min ADC) */
    BLACK_LINE                  /**< Black line on white background (Black is Max ADC) */
} line_color_t;

/** @brief Active line color configuration for the robot. */
#define LINE_COLOR                  WHITE_LINE

/**
 * @brief Defines how the calibration process should be handled.
 */
typedef enum {
    CALIB_MODE_MANUAL_NO_SAVE,      /**< Collect min/max manually, do not save to NVS. */
    CALIB_MODE_MANUAL_SAVE_NVS,     /**< Collect min/max manually and save to NVS. */
    CALIB_MODE_LOAD_FROM_NVS        /**< Bypass manual calibration and load previous data from NVS. */
} calibration_mode_t;

/* ========================================================================== */
/* PUBLIC API                                                                 */
/* ========================================================================== */

/**
 * @brief Initializes the calibration module.
 */
void line_calibration_init(void);

/**
 * @brief Executes the calibration process based on the selected mode.
 * If a manual mode is selected, this function blocks for CALIBRATION_DURATION_MS
 * while sampling the sensors.
 * @param mode The calibration operational mode.
 */
void line_calibration_run(calibration_mode_t mode);

/**
 * @brief Computes and returns the calibrated values for all sensors (line and markers) 
 * based on a raw reading array.
 * @param raw_readings The synchronized array of raw ADC readings.
 * @param calibrated_line Array of size 11 to be populated with calibrated values.
 * @param calibrated_markers Array of size 4 to be populated with calibrated values.
 */
void line_calibration_get_normalized(const uint16_t raw_readings[NUMBER_OF_ACTIVE_CHANNELS], 
                                     uint16_t calibrated_line[NUMBER_OF_LINE_SENSORS], 
                                     uint16_t calibrated_markers[NUMBER_OF_MARKER_SENSORS]);

/**
 * @brief Runs a validation loop printing the calibrated output for debugging.
 */
void line_calibration_validate_output(void);