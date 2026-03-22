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

/* ========================================================================== */
/* MACROS                                                                     */
/* ========================================================================== */

#define RAVEN_COMM_MAX_MESSAGE_LEN  128
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
    CMD_UNKNOWN = 0,    /**< Unrecognized command header. */
    CMD_VALIDATION      /**< Hardware or peripheral validation command (Header: 'V'). */
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

void raven_comm_init(void);
void raven_comm_send_message(const char *tag, const char *format, ...);