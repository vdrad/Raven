#include "race_manager.h"
#include "raven_comm.h"
#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "RMG_CMD"

typedef enum {
    RMG_CMD_UNKNOWN = 0,
    RMG_CMD_SET_SPEED,
    RMG_CMD_SET_FAN,
    RMG_CMD_GET_PARAMS
} rmg_cmd_type_t;

typedef struct {
    rmg_cmd_type_t type;
    float value;
} parsed_rmg_cmd_t;

/**
 * @brief Decodes the string payload (e.g., "S,2.5" or "F,6.0").
 */
static parsed_rmg_cmd_t command_decoder(const char *payload) {
    parsed_rmg_cmd_t result = {RMG_CMD_UNKNOWN, 0.0f};
    char cmd_char;
    float val = 0.0f;

    // sscanf looks for: Command Character, Float Value
    if (sscanf(payload, "%c,%f", &cmd_char, &val) >= 1) {
        result.value = val;
        
        switch (cmd_char) {
            case 'S': result.type = RMG_CMD_SET_SPEED;  break;
            case 'F': result.type = RMG_CMD_SET_FAN;    break;
            case 'C': result.type = RMG_CMD_GET_PARAMS; break;
            default:  result.type = RMG_CMD_UNKNOWN;    break;
        }
    }
    return result;
}

void race_manager_commands_task(void *pvParameters) {
    char received_cmd[RAVEN_COMM_MAX_PAYLOAD_LEN];
    char response[128]; 

    for (;;) {
        // Assuming you define CMD_RACE_MANAGER in your raven_comm queues
        if (raven_comm_check_new_message(CMD_RACE_MANAGER, received_cmd)) {
            parsed_rmg_cmd_t cmd = command_decoder(received_cmd);
            bool updated = false;

            switch (cmd.type) {
                case RMG_CMD_SET_SPEED:
                    race_manager_set_configured_speed(cmd.value);
                    updated = true;
                    break;
                    
                case RMG_CMD_SET_FAN:
                    race_manager_set_configured_fan_voltage(cmd.value);
                    updated = true;
                    break;

                case RMG_CMD_GET_PARAMS:
                    updated = true; 
                    break;

                default:
                    raven_comm_send_message(TAG, "Invalid CMD. Format: [CMD],[Val]. Ex: S,2.5 or F,6.0");
                    break;
            }

            // Confirm alteration by echoing the current state
            if (updated) {
                snprintf(response, sizeof(response), 
                         "RMG_CFG | Speed: %.2f m/s | Fan: %.2f V", 
                         race_manager_get_configured_speed(), 
                         race_manager_get_configured_fan_voltage());
                         
                raven_comm_send_message(TAG, response);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(500)); 
    }
    vTaskDelete(NULL);
}