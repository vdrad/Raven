/**
 * @file line_reading.c
 * @brief Implementation of the master line reading and processing facade.
 */
#include "line_reading.h"

// Standard Libraries
#include <stdbool.h>

// ESP-IDF Inlcudes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

// Project Includes
#include "pinout.h"
#include "AD7490.h"
#include "raven_log.h"
#include "raven_comm.h"
#include "line_commands.h"
#include "line_calibration.h"
#include "line_position.h"
#include "line_markers.h"

#define TAG "LIN"

/* ========================================================================== */
/* PRIVATE VARIABLES                                                          */
/* ========================================================================== */

static bool initialized = false;
static line_reading_data_t current_data = {0};

/* ========================================================================== */
/* HARDWARE CONTROL & INIT                                                    */
/* ========================================================================== */

void line_reading_enable_sensors(void) {
    gpio_set_level(LINE_SENSOR_IO_PIN, 1);
}

void line_reading_disable_sensors(void) {
    gpio_set_level(LINE_SENSOR_IO_PIN, 0);
}

void line_reading_init(void) {
    if (initialized) return;

    // Turns line sensors ON
    gpio_set_direction(LINE_SENSOR_IO_PIN, GPIO_MODE_OUTPUT);
    line_reading_enable_sensors();

    // Initializes submodules
    AD7490_init();
    line_calibration_init();

    // Spawns command listener task
    xTaskCreate(line_commands_task, "line_commands_task", 4096, NULL, 5, NULL);

    initialized = true;
    RAVEN_LOGI(TAG, "Initialized successfully.");
}

/* ========================================================================== */
/* CORE UPDATE LOOP (THE FACADE)                                              */
/* ========================================================================== */

void line_reading_update(void) {
    if (!initialized) return;

    uint16_t raw_readings[NUMBER_OF_LINE_SENSORS];
    uint16_t norm_frontal[NUMBER_OF_FRONTAL_SENSORS];
    uint16_t norm_markers[NUMBER_OF_MARKER_SENSORS];

    // 1. Fetch raw data from ADC
    AD7490_read_all_channels(raw_readings);

    // 2. Normalize data based on calibration
    line_calibration_get_normalized(raw_readings, norm_frontal, norm_markers);

    // 3. Process Position
    line_position_update(norm_frontal);
    
    // 4. Process Markers
    line_markers_update(norm_markers);

    // 5. Aggregate into global struct
    current_data.position = line_position_get_data();
    current_data.markers = line_markers_get_data();

    current_data.is_valid = true; 
}

line_reading_data_t line_reading_get_data(void) {
    return current_data;
}

/* ========================================================================== */
/* CALIBRATION WORKFLOW                                                       */
/* ========================================================================== */

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

/* ========================================================================== */
/* DEBUG & VALIDATION UTILITIES                                               */
/* ========================================================================== */

void line_reading_raw_validation(void) {
    uint16_t readings[NUMBER_OF_LINE_SENSORS] = {0};

    raven_comm_send_message(TAG, "=== STARTING RAW SENSOR VALIDATION ===");

    for (uint8_t i = 0; i < 50; i++) {
        AD7490_read_all_channels(readings);

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

void line_reading_normalized_validation(void) {
    line_calibration_validate_output();
}

void line_reading_position_validation(void) {
    line_reading_update();
    line_reading_data_t data = line_reading_get_data();

    raven_comm_send_message(TAG, "Line position: %.1f | Lost Flag: %d | On Line: %d", 
                            data.position.position, 
                            data.position.robot_lost,
                            data.position.robot_on_line);
                            
    vTaskDelay(pdMS_TO_TICKS(100));
}

void line_reading_markers_validation(void) {
    line_reading_update();
    line_reading_data_t data = line_reading_get_data();

    raven_comm_send_message(TAG, "Left Markers: %d | Right Markers: %d | Crossings: %d | Instantaneous STATUS: %d", 
                            data.markers.left_markers_counter, 
                            data.markers.right_markers_counter,
                            data.markers.crossings_counter,
                            data.markers.marker_status);
                            
    vTaskDelay(pdMS_TO_TICKS(100));
}