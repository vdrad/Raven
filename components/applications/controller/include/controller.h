/**
 * @file controller.h
 * @brief Master control loop and PID tuning utilities API.
 */

#pragma once

#include <stdint.h>
#include "pid.h"

extern pid_context_t right_motor_pid;
extern pid_context_t left_motor_pid;

/* --- TUNER CONFIGURATIONS --- */
#define TUNER_DURATION_MS 500
#define LOOP_PERIOD_US    1000
#define TOTAL_SAMPLES     (TUNER_DURATION_MS / (LOOP_PERIOD_US / 1000))

/* =========================================================================
 * GLOBAL PID INSTANCES
 * ========================================================================= */

/**
 * @brief Initializes the controller module and spawns the command listener task.
 * Must be called once during system boot.
 */
void controller_init(void);

void controller_motors_run(void);

/* =========================================================================
 * TUNER API
 * ========================================================================= */

void controller_tune_drive_motors(void);

void controller_commands_task(void *pvParameters);