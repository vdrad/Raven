/**
 * @file line_sensor_commands.c
 * @brief Implementation of the line sensor command parser.
 */
#include "line_commands.h"
#include "raven_comm.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "LIN"

/**
 * @brief Internal commands recognized by the Line Sensor command decoder.
 */
typedef enum {
    LIN_CMD_UNKNOWN = 0,
    LIN_CMD_CALIB_MANUAL_NO_SAVE,
    LIN_CMD_CALIB_MANUAL_SAVE_NVS,
    LIN_CMD_CALIB_LOAD_NVS,
    LIN_CMD_ABORT
} line_cmd_type_t;

/* ========================================================================== */
/* PRIVATE VARIABLES                                                          */
/* ========================================================================== */

static volatile bool has_new_cmd = false;
static volatile line_cmd_type_t pending_cmd = LIN_CMD_UNKNOWN;

/* ========================================================================== */
/* PRIVATE FUNCTIONS                                                          */
/* ========================================================================== */

/**
 * @brief Decodes the string payload for line sensor operations.
 * @param payload The raw string payload received from comms.
 */
static line_cmd_type_t command_decoder(const char *payload) {
    if (strcmp(payload, "C0") == 0)    return LIN_CMD_CALIB_MANUAL_NO_SAVE;
    if (strcmp(payload, "C1") == 0)    return LIN_CMD_CALIB_MANUAL_SAVE_NVS;
    if (strcmp(payload, "C2") == 0)    return LIN_CMD_CALIB_LOAD_NVS;
    if (strcmp(payload, "ABORT") == 0) return LIN_CMD_ABORT;
    
    return LIN_CMD_UNKNOWN;
}

/* ========================================================================== */
/* PUBLIC API                                                                 */
/* ========================================================================== */

void line_commands_task(void *pvParameters) {
    char received_cmd[RAVEN_COMM_MAX_PAYLOAD_LEN];

    for (;;) {
        if (raven_comm_check_new_message(CMD_LINE_READING, received_cmd)) {
            line_cmd_type_t cmd = command_decoder(received_cmd);

            if (cmd != LIN_CMD_UNKNOWN) {
                pending_cmd = cmd;
                has_new_cmd = true; // Signals the execution layer
            } else {
                raven_comm_send_message(TAG, "Invalid cmd. Use LC0, LC1, LC2, or ABORT.");
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(500)); 
    }
    vTaskDelete(NULL);
}

bool line_commands_get_calibration_request(calibration_mode_t *mode, bool *abort) {
    if (!has_new_cmd) return false;

    *abort = false;

    switch (pending_cmd) {
        case LIN_CMD_CALIB_MANUAL_NO_SAVE:
            *mode = CALIB_MODE_MANUAL_NO_SAVE;
            break;
        case LIN_CMD_CALIB_MANUAL_SAVE_NVS:
            *mode = CALIB_MODE_MANUAL_SAVE_NVS;
            break;
        case LIN_CMD_CALIB_LOAD_NVS:
            *mode = CALIB_MODE_LOAD_FROM_NVS;
            break;
        case LIN_CMD_ABORT:
            *abort = true;
            break;
        default:
            break;
    }
    
    has_new_cmd = false; // Consume the command
    return true;
}