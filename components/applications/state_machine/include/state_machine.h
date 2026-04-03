/**
 * @file state_machine.h
 * @brief Public API for the robot's core State Machine.
 *
 * Exposes the necessary functions to initialize, step, and query 
 * the high-level operational state of the robot.
 */

#pragma once

#include <stdint.h>

/**
 * @brief Initializes the state machine tasks and internal structures.
 * * Must be called once during system boot before the scheduler starts 
 * or immediately after.
 */
void state_machine_init(void);

/**
 * @brief Executes one cycle of the state machine.
 * * Safely applies any pending state transitions requested by other tasks,
 * then executes the active state's callback function.
 */
void state_machine_step(void);

/**
 * @brief Retrieves the string name of the currently active state.
 * @return const uint8_t* Pointer to the state name string.
 */
const uint8_t *state_get_name(void);

/**
 * @brief Forces the state machine back to the initial waiting state.
 */
void state_machine_reset(void);