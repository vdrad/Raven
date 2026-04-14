/**
 * @file line_reading.c
 * @brief Implementation of the raw line reading module via AD7490.
 */
#include "line_reading.h"

// Standard Libraries
#include <stdbool.h>

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Project Includes
#include "AD7490.h"
#include "raven_log.h"
#include "raven_comm.h"
#include "line_calibration.h"
#include "line_commands.h"

#define TAG "LIN"
static bool initialized = false;

void line_reading_init(void) {
    if (initialized) return;

    AD7490_init();
    line_calibration_init();
    xTaskCreate(line_commands_task, "line_commands_task", 4096, NULL, 5, NULL);

    initialized = true;
    RAVEN_LOGI(TAG, "Initialized successfully.");
}

void line_reading_get_raw(uint16_t array[NUMBER_OF_ACTIVE_CHANNELS]) {
    if (!initialized) return;
    AD7490_read_all_channels(array);
}

void line_reading_calibrate(void) {
    raven_comm_send_message(TAG, "=== LINE CALIBRATION MODE ===");
    raven_comm_send_message(TAG, "  LC0 -> Manual (No Save)");
    raven_comm_send_message(TAG, "  LC1 -> Manual (Save to NVS)");
    raven_comm_send_message(TAG, "  LC2 -> Load from NVS");
    raven_comm_send_message(TAG, "  ABORT -> Abort calibration");

    calibration_mode_t mode;
    bool abort_requested = false;

    // Waits for a valid input.
    while (!line_commands_get_calibration_request(&mode, &abort_requested)) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (abort_requested) {
        raven_comm_send_message(TAG, "Calibration aborted by user.");
    } else {
        // Sends feedback message based on chosen mode.
        if (mode == CALIB_MODE_MANUAL_NO_SAVE)        raven_comm_send_message(TAG, "Starting: Manual Calibration (No Save).");
        else if (mode == CALIB_MODE_MANUAL_SAVE_NVS)  raven_comm_send_message(TAG, "Starting: Manual Calibration (Saving to NVS).");
        else if (mode == CALIB_MODE_LOAD_FROM_NVS)    raven_comm_send_message(TAG, "Starting: Load Calibration from NVS.");
        
        line_calibration_run(mode);
    }
}

void line_reading_normalized_validation(void) {
    line_calibration_validate_output();
}

void line_reading_raw_validation(void) {
    uint16_t readings[NUMBER_OF_ACTIVE_CHANNELS] = {0};

    raven_comm_send_message(TAG, "=== STARTING RAW SENSOR VALIDATION ===");

    for (uint8_t i = 0; i < 50; i++) {
        line_reading_get_raw(readings);

        raven_comm_send_message(TAG, 
            "Sample [%2d] | L1-0: %4d %4d | S0-S10: %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d | R1-0: %4d %4d",
            i,
            readings[LM0], readings[LM1],
            readings[LS0], readings[LS1], readings[LS2], readings[LS3],
            readings[LS4], readings[LS5], readings[LS6], readings[LS7],
            readings[LS8], readings[LS9], readings[LS10],
            readings[RM1], readings[RM0]
        );

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}