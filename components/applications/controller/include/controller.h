/**
 * @file controller.h
 * @brief Master control loop and PID tuning utilities API.
 *
 * This module handles the execution of the PID controllers, parses incoming 
 * tuning commands, and provides a high-precision hardware timer-based tuning 
 * tool for system identification and telemetry data gathering.
 */

#pragma once

#include <stdint.h>
#include "pid.h"

/* =========================================================================
 * GLOBAL PID INSTANCES
 * ========================================================================= */

extern pid_context_t right_motor_pid;
extern pid_context_t left_motor_pid;

/**
 * @brief Initializes the controller module and spawns the command listener task.
 * * Must be called once during system boot.
 */
void controller_init(void);

/**
 * @brief Main execution function to update sensors and calculate motor responses.
 */
void controller_motors_run(void);

/* =========================================================================
 * TUNER API
 * ========================================================================= */

/**
 * @brief Triggers the automated tuning sequence for drive motors.
 */
void controller_tune_drive_motors(void);

/**
 * @brief Background task that listens for incoming PID tuning commands.
 */
void controller_commands_task(void *pvParameters);