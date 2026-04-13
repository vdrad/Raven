/**
 * @file line_sensor_commands.h
 * @brief Command parser and background task for line sensor configuration.
 */
#pragma once

#include <stdbool.h>
#include "line_calibration.h"

/**
 * @brief Background task that listens for incoming commands targeted at the line sensor.
 */
void line_commands_task(void *pvParameters);

/**
 * @brief Checks if a new calibration command has been received by the command task.
 * @param mode Pointer to store the requested calibration mode.
 * @param abort Pointer to a boolean that will be set to true if the user aborted.
 * @return true if a command was processed, false otherwise.
 */
bool line_commands_get_calibration_request(calibration_mode_t *mode, bool *abort);