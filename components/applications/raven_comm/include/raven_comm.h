/**
 * @file raven_comm.h
 * @brief High-level communication definitions and API for the robot.
 * * This module defines the communication protocol structures, including command 
 * types and payload sizing. It exposes the public API for initializing the 
 * communication stack and transmitting telemetry data asynchronously, ensuring
 * the main deterministic control loops (e.g., PID controllers) are never blocked.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

/* ========================================================================== */
/* MACROS                                                                     */
/* ========================================================================== */

#define RAVEN_COMM_MAX_MESSAGE_LEN  150
#define RAVEN_COMM_MAX_PAYLOAD_LEN (RAVEN_COMM_MAX_MESSAGE_LEN - 3)

/* ========================================================================== */
/* ENUMERATIONS & STRUCTURES                                                  */
/* ========================================================================== */

/**
 * @brief Represents all available command categories the robot can understand.
 * * Each category corresponds to a specific single-character header received 
 * from the external controller (e.g., 'V' for CMD_VALIDATION).
 */
typedef enum {
    CMD_UNKNOWN = 0,                /**< Unrecognized command header. */
    CMD_VALIDATION,                 /**< Hardware or peripheral validation command (Header: 'V'). */
    CMD_STATE_MACHINE,              /**< State Machine command (Header: 'S'). */
    CMD_LINE_READING,               /**< Line Reading command (Header: 'L'). */
    CMD_CONTROLLER_TUNING,          /**< PID Controller command (Header: 'P'). */
    CMD_RACE_MANAGER,               /**< Race Manager command (Header: 'R'). */
    CMD_MOTOR_CHARACTERIZATION,     /**< Motor Characterization command (Header: 'M'). */

    CMD_MAX
} robot_cmd_type_t;

/**
 * @brief Data packet structure used to route commands internally.
 * * This structure holds the decoded command type and its cleaned payload string
 * (stripped of carriage returns and line feeds). It is passed by value into 
 * internal FreeRTOS queues.
 */
typedef struct {
    robot_cmd_type_t type;                              /**< The categorized command type. */
    char payload[RAVEN_COMM_MAX_PAYLOAD_LEN];           /**< The null-terminated payload string. */
} robot_command_t;

/* ========================================================================== */
/* PUBLIC API                                                                 */
/* ========================================================================== */

/**
 * @brief Initializes the communication module.
 * * Spawns the internal FreeRTOS task responsible for decoding incoming BLE messages.
 */
void raven_comm_init(void);

/**
 * @brief Formats and sends a message through the communication interface (BLE).
 * * Uses standard `printf` style formatting. Automatically appends the module tag 
 * and carriage returns.
 * * @param tag A 3-character string identifying the source module.
 * @param format Standard format string (e.g., "Value: %d").
 * @param ... Variable arguments matching the format string.
 */
void raven_comm_send_message(const char *tag, const char *format, ...);

/**
 * @brief Checks if a new message is available for a specific command category.
 * * Reads the internal mailbox associated with the provided command type.
 * If a new message exists, it copies the payload to the output buffer and
 * lowers the 'new message' flag. This read operation is protected by a spinlock.
 *
 * @param cmd_type    The target command category to check.
 * @param out_payload Buffer where the payload will be copied if a message exists.
 * @return true if a new message was retrieved, false otherwise.
 */
bool raven_comm_check_new_message(robot_cmd_type_t cmd_type, char *out_payload);