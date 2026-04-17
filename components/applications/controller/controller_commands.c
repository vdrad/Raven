#include "controller.h"
#include "raven_comm.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
/* =========================================================================
 * COMMAND PARSER & TUNING TASK
 * ========================================================================= */

#define TAG "CTR"

/**
 * @brief Internal commands recognized by the Controller command decoder.
 */
typedef enum {
    CON_CMD_UNKNOWN = 0,
    CON_CMD_SET_KP,
    CON_CMD_SET_KI,
    CON_CMD_SET_KD,
    CON_CMD_SET_SETPOINT,
    CON_CMD_SET_TD,
    CON_CMD_GET_PARAMS
} controller_cmd_type_t;

/**
 * @brief Structure to hold the parsed command result.
 */
typedef struct {
    char target; // 'R' (Right), 'L' (Left), 'Y' (Yaw), 'N' (Line)
    controller_cmd_type_t type;
    float value;
} parsed_cmd_t;

/**
 * @brief Decodes the string payload (e.g., "P,1.5" or "S,1500").
 * @param payload The raw string payload received from comms.
 * @return A parsed_cmd_t structure containing the action and the float value.
 */
static parsed_cmd_t command_decoder(const char *payload) {
    parsed_cmd_t result = {'\0', CON_CMD_UNKNOWN, 0.0f};
    char target_char;
    char cmd_char;
    float val = 0.0f;

    // Agora o sscanf procura: Caractere do Alvo, Caractere do Comando, Float do Valor
    if (sscanf(payload, "%c,%c,%f", &target_char, &cmd_char, &val) >= 2) {
        result.target = target_char;
        result.value = val;
        
        switch (cmd_char) {
            case 'P': result.type = CON_CMD_SET_KP;       break;
            case 'I': result.type = CON_CMD_SET_KI;       break;
            case 'D': result.type = CON_CMD_SET_KD;       break;
            case 'S': result.type = CON_CMD_SET_SETPOINT; break;
            case 'T': result.type = CON_CMD_SET_TD;       break;
            case 'C': result.type = CON_CMD_GET_PARAMS;   break;
            default:  result.type = CON_CMD_UNKNOWN;      break;
        }
    }
    return result;
}

/**
 * @brief Maps a target character to the respective PID context pointer.
 */
static pid_context_t* get_target_pid(char target) {
    switch (target) {
        case 'R': return &right_motor_pid;
        case 'L': return &left_motor_pid;
        case 'N': return &line_position_pid;
        // case 'Y': return &yaw_pid;  // Para o futuro
        default:  return NULL;
    }
}

/**
 * @brief Clears residual PID values to prevent sudden jerks when tuning dynamically.
 */
static void reset_pid_residuals(pid_context_t *pid) {
    pid->integral_sum = 0.0f;
    pid->previous_error = 0.0f;
}

/**
 * @brief Updates Kp and Kd based on the desired response time Td
 * * @param target_pid Pointer to PID struct
 * @param td Desired response time in seconds
 */
static inline void controller_update_gains_from_td(pid_context_t *target_pid, float td) {
    if (td < 0.001f) {
        raven_comm_send_message(TAG, "ERROR: Td is too low!");
        return; 
    }

    float tm = target_pid->tm;
    float ff = target_pid->ff_coef;
    float td_squared = td * td;

    target_pid->kP = (ff * ((8.0f * tm) - td)) / td;
    target_pid->kI = (32.01f * tm * ff) / td_squared;
}

void controller_commands_task(void *pvParameters) {
    char received_cmd[RAVEN_COMM_MAX_PAYLOAD_LEN];
    char response[128]; 

    for (;;) {
        if (raven_comm_check_new_message(CMD_CONTROLLER_TUNING, received_cmd)) {
            parsed_cmd_t cmd = command_decoder(received_cmd);
            bool updated = false;

            // 1. Finds which PID will be altered
            pid_context_t *target_pid = get_target_pid(cmd.target);

            // If user sent a valid controller
            if (target_pid == NULL) {
                raven_comm_send_message("CTRL", "Invalid Target. Use R, L, Y, or N.");
                continue; // Continues for next iteration
            }

            // 2. Applies the given command
            switch (cmd.type) {
                case CON_CMD_SET_KP:
                    target_pid->kP = cmd.value;
                    updated = true;
                    break;
                case CON_CMD_SET_KI:
                    target_pid->kI = cmd.value;
                    updated = true;
                    break;
                case CON_CMD_SET_KD:
                    target_pid->kD = cmd.value;
                    updated = true;
                    break;
                case CON_CMD_SET_SETPOINT:
                    target_pid->setpoint = cmd.value;
                    updated = true;
                    break;
                case CON_CMD_SET_TD:
                    controller_update_gains_from_td(target_pid, cmd.value);
                    updated = true;
                    break;
                case CON_CMD_GET_PARAMS:
                    updated = true; 
                    break;

                default:
                    raven_comm_send_message(TAG, "Invalid CMD. Format: [Target],[CMD],[Val]. Ex: R,P,1.5");
                    break;
            }

            // 3. Confirma a alteração enviando o status de volta
            if (updated) {
                reset_pid_residuals(target_pid);
                
                snprintf(response, sizeof(response), 
                         "%c-PID | P:%.6f I:%.6f D:%.6f SP:%.2f", 
                         cmd.target, target_pid->kP, target_pid->kI, 
                         target_pid->kD, target_pid->setpoint);
                         
                raven_comm_send_message(TAG, response);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(500)); 
    }
    vTaskDelete(NULL);
}