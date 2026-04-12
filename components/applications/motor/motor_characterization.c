/**
 * @file motor_characterization.c
 * @brief Open-loop motor characterization module for mapping voltage to velocity.
 */

#include "motor.h"
#include "odometry.h"
#include "raven_comm.h"
#include "raven_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TAG "MCH"

/* ========================================================================== */
/* TEST CONFIGURATIONS                                                        */
/* ========================================================================== */

// Duration of each test window in milliseconds
#define CHAR_DURATION_MS 500

// Sampling frequency (10000 us = 100 Hz)
#define CHAR_LOOP_PERIOD_US 10000
#define TOTAL_SAMPLES (uint32_t)(CHAR_DURATION_MS / (CHAR_LOOP_PERIOD_US / 1000.0f))

// Array of voltages to be tested sequentially
// static const float test_voltages[] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f, 10.0f, 11.0f, 12.0f};
static const float test_voltages[] = {0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f, 0.8f, 0.9f, 1.0f, 1.5f, 2.0f, 2.5f, 3.0f};
static const uint8_t num_test_voltages = sizeof(test_voltages) / sizeof(test_voltages[0]);

/* ========================================================================== */
/* DATA STRUCTURES & GLOBAL STATE                                             */
/* ========================================================================== */

/**
 * @brief Optimized data structure for the telemetry buffer.
 */
typedef struct {
    int64_t timestamp_us;
    float vel_left_m_s;
    float vel_right_m_s;
} char_sample_t;

static char_sample_t *char_log_buffer = NULL;
static int char_current_sample = 0;
static uint8_t current_voltage_index = 0;

static esp_timer_handle_t char_timer_handle = NULL;

// Concurrency control flags
static volatile bool is_sampling = false;
static volatile bool test_finished_flag = false; 

/* ========================================================================== */
/* HIGH-FREQUENCY TIMER CALLBACK (DATA COLLECTION)                            */
/* ========================================================================== */

/**
 * @brief Hardware timer callback that executes the high-frequency sampling loop.
 * @param arg Unused timer argument.
 */
static void char_timer_callback(void* arg) {
    if (char_current_sample < TOTAL_SAMPLES && char_log_buffer != NULL) {
        
        // 1. Update and fetch real odometry data
        odometry_update();
        odometry_data_t odom = odometry_get_data();

        // 2. Save the frame to the Heap
        char_log_buffer[char_current_sample].timestamp_us  = esp_timer_get_time();
        char_log_buffer[char_current_sample].vel_left_m_s  = odom.velocity_left_m_s;
        char_log_buffer[char_current_sample].vel_right_m_s = odom.velocity_right_m_s;
        
        char_current_sample++;
    }

    // 3. Stop condition reached
    if (char_current_sample >= TOTAL_SAMPLES) {
        esp_timer_stop(char_timer_handle);
        
        // Cut motors immediately
        motor_set_voltage(MOTOR_LEFT, 0.0f);
        motor_set_voltage(MOTOR_RIGHT, 0.0f);
        
        is_sampling = false;
        test_finished_flag = true; // Notifies the task to print the data
    }
}

/* ========================================================================== */
/* AUXILIARY ROUTINES                                                         */
/* ========================================================================== */

/**
 * @brief Safely prints the collected data and frees the allocated memory.
 */
static void char_print_and_cleanup(void) {
    float applied_voltage = test_voltages[current_voltage_index];

    RAVEN_LOGI(TAG, "--- CSV START ---");
    RAVEN_LOGI(TAG, "Metadata,Voltage:%.1fV,Duration:%dms", applied_voltage, CHAR_DURATION_MS);
    RAVEN_LOGI(TAG, "Point,DeltaTime_us,Vel_Left_m_s,Vel_Right_m_s");

    for (int i = 0; i < TOTAL_SAMPLES; i++) {
        int64_t delta_t = (i > 0) ? (char_log_buffer[i].timestamp_us - char_log_buffer[i-1].timestamp_us) : 0;
        
        char row_buf[128];
        snprintf(row_buf, sizeof(row_buf), "%d,%lld,%.3f,%.3f", 
                 i, 
                 delta_t, 
                 char_log_buffer[i].vel_left_m_s, 
                 char_log_buffer[i].vel_right_m_s);
                 
        RAVEN_LOGI(TAG, "%s", row_buf);
        
        // Feed the Watchdog and flush the UART buffer
        vTaskDelay(pdMS_TO_TICKS(20)); 
    }
    RAVEN_LOGI(TAG, "--- CSV END ---");

    // Cleanup
    free(char_log_buffer);
    char_log_buffer = NULL;

    // Prepare for the next voltage level
    current_voltage_index++;
    if (current_voltage_index >= num_test_voltages) {
        RAVEN_LOGI(TAG, "Characterization complete! All voltages tested.");
        current_voltage_index = 0; // Reset in case the user wants to run it again
    } else {
        RAVEN_LOGI(TAG, "Ready for next step: %.1fV. Send 'MPASS' to start.", test_voltages[current_voltage_index]);
    }
}

/**
 * @brief Starts the sampling process for the current voltage step.
 */
static void start_voltage_step(void) {
    if (is_sampling) return;

    // 1. Dynamic memory allocation
    char_log_buffer = (char_sample_t *)malloc(TOTAL_SAMPLES * sizeof(char_sample_t));
    if (char_log_buffer == NULL) {
        RAVEN_LOGE(TAG, "Failed to allocate memory for characterization buffer!");
        return;
    }

    // 2. State and hardware setup
    char_current_sample = 0;
    test_finished_flag = false;
    is_sampling = true;
    
    // Clear odometry history to avoid "dt" spikes or inherited velocity
    odometry_reset(); 

    // 3. Apply voltage and start the timer
    float target_voltage = test_voltages[current_voltage_index];
    RAVEN_LOGI(TAG, "Running step %d/%d: Applying %.1fV for %d ms...", 
               current_voltage_index + 1, num_test_voltages, target_voltage, CHAR_DURATION_MS);
               
    motor_set_voltage(MOTOR_LEFT, target_voltage);
    motor_set_voltage(MOTOR_RIGHT, target_voltage);

    esp_timer_start_periodic(char_timer_handle, CHAR_LOOP_PERIOD_US);
}

/* ========================================================================== */
/* PUBLIC API                                                                 */
/* ========================================================================== */

/**
 * @brief Runs the complete characterization sequence, blocking the State Machine.
 *
 * Waits for the 'PASS' command via mailbox to advance each voltage step.
 */
void motor_characterization_run(void) {
    char received_cmd[RAVEN_COMM_MAX_PAYLOAD_LEN];
    
    // Create the timer if it doesn't exist yet
    if (char_timer_handle == NULL) {
        const esp_timer_create_args_t timer_args = {
            .callback = &char_timer_callback,
            .name = "char_timer"
        };
        esp_timer_create(&timer_args, &char_timer_handle);
    }

    RAVEN_LOGI(TAG, "Entering Characterization Mode. Voltages to test: %d", num_test_voltages);

    // Loop through all configured voltages
    for (int i = 0; i < num_test_voltages; i++) {
        current_voltage_index = i;
        RAVEN_LOGI(TAG, "Ready for step %d/%d (%.1fV). Send 'PASS' to start or 'ABORT' to cancel.", 
                   i + 1, num_test_voltages, test_voltages[i]);

        // 1. Block execution waiting for user command for this step
        bool step_triggered = false;
        while (!step_triggered) {
            if (raven_comm_check_new_message(CMD_MOTOR_CHARACTERIZATION, received_cmd)) {
                if (strcmp(received_cmd, "PASS") == 0) {
                    step_triggered = true;
                } else if (strcmp(received_cmd, "ABORT") == 0) {
                    RAVEN_LOGW(TAG, "Characterization aborted by user.");
                    return; // Abort the entire function and return control to the State Machine
                }
            }
            vTaskDelay(pdMS_TO_TICKS(100)); // Feed the Watchdog while waiting for the user
        }

        // 2. User sent PASS. Start sampling
        start_voltage_step();

        // 3. Block execution waiting for the timer to collect all physical data
        while (is_sampling) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        // 4. Timer finished. Print CSV to serial
        char_print_and_cleanup();
    }

    RAVEN_LOGI(TAG, "Characterization sequence fully completed.");
}