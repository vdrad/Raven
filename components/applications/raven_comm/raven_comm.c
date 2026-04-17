/**
 * @file raven_comm.c
 * @brief High-level communication manager for the robot.
 *
 * This module acts as the application-level wrapper for telemetry and data transmission.
 * It formats outgoing messages with a specific TAG, appends carriage return and line 
 * feed (\r\n) characters, and routes them to the underlying BLE manager.
 * It also manages an independent FreeRTOS task to decode incoming BLE commands 
 * without blocking the main robot control loop (PID/Sensors).
 */

#include "raven_comm.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdarg.h>
#include <string.h>

// FreeRTOS Includes
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

// Project Includes
#include "raven_log.h"
#include "ble_manager.h"
#include "peripheral_validation.h"

/* ========================================================================== */
/* MACROS & GLOBAL VARIABLES                                                  */
/* ========================================================================== */

#define TAG "CMM"
static bool initialized = false;

/** @brief Queue to safely pass data from the BLE ISR/Callback to the Decoder Task */
QueueHandle_t robot_command_queue = NULL;

/** @brief Handle for the decoder task */
static TaskHandle_t comm_decoder_task_handle = NULL;

/**
 * @brief Internal structure representing a single mailbox for a specific command type.
 */
typedef struct {
    char payload[RAVEN_COMM_MAX_PAYLOAD_LEN]; /**< Buffer storing the command payload */
    bool has_new;                             /**< Flag indicating an unread message */
} comm_mailbox_t;

/** @brief Array of mailboxes, one for each command type defined in robot_cmd_type_t */
static comm_mailbox_t mailboxes[CMD_MAX] = {0};

/** @brief Spinlock to prevent memory corruption if reading and writing happen simultaneously */
static portMUX_TYPE mailbox_spinlock = portMUX_INITIALIZER_UNLOCKED;

/* ========================================================================== */
/* PRIVATE FUNCTIONS                                                          */
/* ========================================================================== */

/**
 * @brief Callback triggered by the BLE manager when data is received from the app.
 *
 * Extracts the command header, strips carriage returns and line feeds from the 
 * payload, and safely dispatches the structured command to the FreeRTOS queue.
 *
 * @param data Pointer to the raw byte array received via BLE.
 * @param len  Length of the received data array.
 */
static void receive_message_cb(uint8_t *data, uint16_t len) {
    if (data == NULL || len == 0) return;

    robot_command_t new_command;
    memset(&new_command, 0, sizeof(robot_command_t));

    // 1. Extract the header to determine the command category
    char header = (char)data[0];
    switch (header) {
        case 'S': new_command.type = CMD_STATE_MACHINE;          break;
        case 'L': new_command.type = CMD_LINE_READING;           break;
        case 'V': new_command.type = CMD_VALIDATION;             break;
        case 'C': new_command.type = CMD_CONTROLLER_TUNING;      break;
        case 'M': new_command.type = CMD_MOTOR_CHARACTERIZATION; break;
        case 'R': new_command.type = CMD_RACE_MANAGER;           break;
        default:  new_command.type = CMD_UNKNOWN; break;
    }

    // 2. Extract payload, ignoring \r and \n characters
    uint16_t payload_idx = 0;
    for (uint16_t i = 1; i < len; i++) {
        char c = (char)data[i];
        if (c == '\r' || c == '\n') continue;
        
        if (payload_idx < (RAVEN_COMM_MAX_PAYLOAD_LEN - 1)) {
            new_command.payload[payload_idx] = c;
            payload_idx++;
        }
    }

    new_command.payload[payload_idx] = '\0';
    RAVEN_LOGI(TAG, "Header: '%c' -> Type: %d | Clean Payload: '%s'", header, new_command.type, new_command.payload);

    // 3. Send the structured command to the decoder task queue
    if (new_command.type != CMD_UNKNOWN) {
        if (robot_command_queue != NULL) {
            // Sends with 0 ticks to wait, ensuring the BLE callback is never blocked
            xQueueSend(robot_command_queue, &new_command, 0);
        } else {
            RAVEN_LOGE(TAG, "Queue robot_command_queue is not initialized!");
        }
    } else {
        RAVEN_LOGW(TAG, "Unknown command ignored.");
    }
}

/**
 * @brief FreeRTOS task that continuously processes incoming robot commands.
 *
 * Blocks indefinitely waiting for new messages on the `robot_command_queue`.
 * When a valid message is received, it safely stores the payload into the appropriate
 * mailbox using a critical section to prevent race conditions with reading modules.
 *
 * @param pvParameters Pointer to task parameters (unused).
 */
static void raven_comm_decoder_task(void *pvParameters) {
    robot_command_t received_cmd;

    for (;;) {
        if (xQueueReceive(robot_command_queue, &received_cmd, portMAX_DELAY) == pdTRUE) {
            // Protect against unknown/out-of-bounds commands
            if (received_cmd.type > CMD_UNKNOWN && received_cmd.type < CMD_MAX) {
                
                // ENTER CRITICAL SECTION: Quickly save the message to the correct mailbox
                taskENTER_CRITICAL(&mailbox_spinlock);
                strncpy(mailboxes[received_cmd.type].payload, received_cmd.payload, RAVEN_COMM_MAX_PAYLOAD_LEN);
                mailboxes[received_cmd.type].has_new = true;
                taskEXIT_CRITICAL(&mailbox_spinlock);
                
            }
        }
    }
}


/* ========================================================================== */
/* PUBLIC API IMPLEMENTATIONS                                                 */
/* ========================================================================== */

/**
 * @brief Initializes the high-level communication module.
 *
 * Sets up the underlying Bluetooth Low Energy (BLE) manager, creates the 
 * internal message queue, and spawns the dedicated decoder task.
 * Safely ignores repeated calls.
 */
void raven_comm_init(void) {
    if (initialized) return;

    // 1. Create the command queue BEFORE starting tasks or BLE
    robot_command_queue = xQueueCreate(10, sizeof(robot_command_t));
    if (robot_command_queue == NULL) {
        RAVEN_LOGE(TAG, "Failed to create robot_command_queue!");
        return;
    }

    // 2. Initialize BLE and register the callback
    ble_manager_init();
    ble_manager_receive_callback(receive_message_cb);

    // 3. Spawn the Decoder Task
    xTaskCreatePinnedToCore(
        raven_comm_decoder_task, 
        "comm_decoder", 
        4096, 
        NULL, 
        5, 
        &comm_decoder_task_handle, 
        1
    );

    initialized = true;
    raven_comm_send_message(TAG, "Initialized successfully.");
}

void raven_comm_send_message(const char *tag, const char *format, ...) {
    if (!initialized || tag == NULL || format == NULL) return;

    char buffer[RAVEN_COMM_MAX_MESSAGE_LEN];
    va_list args;
    
    // Format the prefix: "[TAG] "
    int prefix_len = snprintf(buffer, sizeof(buffer), "[%s] ", tag);
    if (prefix_len < 0 || prefix_len >= sizeof(buffer)) return;
    
    // Format the actual message payload
    va_start(args, format);
    int message_len = vsnprintf(buffer + prefix_len, sizeof(buffer) - prefix_len, format, args);
    va_end(args);
    
    if (message_len > 0) {
        int total_len = prefix_len + message_len;
        
        // Ensure there is room for the CRLF terminator
        if (total_len < sizeof(buffer) - 2) {
            buffer[total_len] = '\r';
            buffer[total_len + 1] = '\n';
            total_len += 2;
        } else {
            // Force line break if buffer is full
            buffer[sizeof(buffer) - 3] = '\r';
            buffer[sizeof(buffer) - 2] = '\n';
            total_len = sizeof(buffer) - 1;
        }
        
        ble_manager_send_message((uint8_t *)buffer, (uint16_t)total_len);
    }
}

bool raven_comm_check_new_message(robot_cmd_type_t cmd_type, char *out_payload) {
    if (cmd_type <= CMD_UNKNOWN || cmd_type >= CMD_MAX || out_payload == NULL) return false;
    
    bool is_new = false;

    // ENTER CRITICAL SECTION: Read and clear the mailbox safely
    taskENTER_CRITICAL(&mailbox_spinlock);
    if (mailboxes[cmd_type].has_new) {
        strncpy(out_payload, mailboxes[cmd_type].payload, RAVEN_COMM_MAX_PAYLOAD_LEN);
        mailboxes[cmd_type].has_new = false;
        is_new = true;
    }
    taskEXIT_CRITICAL(&mailbox_spinlock);

    return is_new;
}