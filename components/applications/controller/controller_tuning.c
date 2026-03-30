#include "controller.h"
#include "pid.h"
#include "raven_log.h"
#include "motor.h"
#include "odometry.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// TODO: adjust %4f to %6f in controller msg
// TODO: make tuning follow a trapezoid
// TODO: clean raven_send_comm() and put relevant parameters instead of initialized successfully.
// TODO: standardized code pattern, include, etc (ask for a fixed prompt)

/* --- TUNER CONFIGURATIONS --- */
#define TUNER_DURATION_MS 500
#define LOOP_PERIOD_US    1000
#define TOTAL_SAMPLES     (TUNER_DURATION_MS / (LOOP_PERIOD_US / 1000))

// Max number of PIDs you can tune at the same time (e.g., Left, Right, Yaw)
#define MAX_SIMULTANEOUS_PIDS 3 

/* --- 1. GLOBAL TUNER STATE --- */
/**
 * @brief Scalable telemetry structure using arrays for multiple controllers.
 */
typedef struct {
    int64_t timestamp_us;
    float reading[MAX_SIMULTANEOUS_PIDS];
    float setpoint[MAX_SIMULTANEOUS_PIDS];
    float output[MAX_SIMULTANEOUS_PIDS];
    float kp[MAX_SIMULTANEOUS_PIDS];
    float ki[MAX_SIMULTANEOUS_PIDS];
    float kd[MAX_SIMULTANEOUS_PIDS];
    float integral_sum[MAX_SIMULTANEOUS_PIDS];
} telemetry_sample_t;

static telemetry_sample_t *tuner_log_buffer = NULL;
static int tuner_current_sample = 0;
static esp_timer_handle_t tuner_timer_handle = NULL;
static bool tuner_is_running = false;

/* Generic state for the modular tuner */
static pid_context_t **tuner_active_pids = NULL;
static uint8_t tuner_active_pids_count = 0;
static void (*tuner_update_cb)(void) = NULL; // Notice: no arguments now
static void (*tuner_stop_cb)(void) = NULL;

/* --- 2. TIMER CALLBACK --- */
/**
 * @brief Hardware timer callback that executes the high-frequency tuning loop.
 */
static void pid_tuner_timer_callback(void* arg) {
    // Step 1: Execute the user-defined update function (reads sensors, computes PIDs, actuates)
    if (tuner_update_cb != NULL) {
        tuner_update_cb();
    }

    // Step 2: Save the snapshot of ALL tracked PIDs into the heap buffer
    if (tuner_current_sample < TOTAL_SAMPLES && tuner_log_buffer != NULL) {
        tuner_log_buffer[tuner_current_sample].timestamp_us = esp_timer_get_time();
        
        for (uint8_t i = 0; i < tuner_active_pids_count; i++) {
            tuner_log_buffer[tuner_current_sample].reading[i]      = tuner_active_pids[i]->current_reading;
            tuner_log_buffer[tuner_current_sample].setpoint[i]     = tuner_active_pids[i]->setpoint;
            tuner_log_buffer[tuner_current_sample].output[i]       = tuner_active_pids[i]->output;
            tuner_log_buffer[tuner_current_sample].kp[i]           = tuner_active_pids[i]->kP;
            tuner_log_buffer[tuner_current_sample].ki[i]           = tuner_active_pids[i]->kI;
            tuner_log_buffer[tuner_current_sample].kd[i]           = tuner_active_pids[i]->kD;
            tuner_log_buffer[tuner_current_sample].integral_sum[i] = tuner_active_pids[i]->integral_sum;
        }
        tuner_current_sample++;
    }

    // Step 3: Stop Condition
    if (tuner_current_sample >= TOTAL_SAMPLES) {
        esp_timer_stop(tuner_timer_handle);
        
        if (tuner_stop_cb != NULL) {
            tuner_stop_cb();
        }
        
        tuner_is_running = false;
    }
}

/* --- 3. GENERIC TUNER TRIGGER --- */
/**
 * @brief Runs a high-precision tuning sequence for multiple PID controllers.
 * @param target_pids Array of pointers to the PIDs to be logged.
 * @param num_pids Number of PIDs in the target_pids array.
 * @param update_func Pointer to the function that updates sensors and runs pid_compute for all targets.
 * @param stop_func Pointer to the function that shuts down the actuators.
 */
void controller_run_generic_tuner(pid_context_t **target_pids, uint8_t num_pids, 
                                  void (*update_func)(void), 
                                  void (*stop_func)(void)) {
                                      
    if (tuner_is_running) {
        RAVEN_LOGE("TUNER", "Tuner is already running!");
        return;
    }

    // Clamp the number of PIDs to the maximum allowed to prevent memory corruption
    tuner_active_pids_count = (num_pids > MAX_SIMULTANEOUS_PIDS) ? MAX_SIMULTANEOUS_PIDS : num_pids;

    // 1. Setup global state
    tuner_active_pids = target_pids;
    tuner_update_cb = update_func;
    tuner_stop_cb = stop_func;

    // 2. Allocate heap memory for telemetry
    tuner_log_buffer = (telemetry_sample_t *)malloc(TOTAL_SAMPLES * sizeof(telemetry_sample_t));
    if (tuner_log_buffer == NULL) {
        RAVEN_LOGE("TUNER", "Error: Failed to allocate memory for tuner buffer!");
        return;
    }

    // 3. Reset states for all tracked PIDs
    tuner_current_sample = 0;
    for (uint8_t i = 0; i < tuner_active_pids_count; i++) {
        tuner_active_pids[i]->integral_sum = 0.0f;
        tuner_active_pids[i]->previous_error = 0.0f;
        tuner_active_pids[i]->last_run_time_us = 0; 
    }

    // 4. Create and start hardware timer
    if (tuner_timer_handle == NULL) {
        const esp_timer_create_args_t timer_args = {
            .callback = &pid_tuner_timer_callback,
            .name = "tuner_timer"
        };
        esp_timer_create(&timer_args, &tuner_timer_handle);
    }

    RAVEN_LOGI("TUNER", "Starting %d ms high-precision tuning sequence for %d PIDs...", TUNER_DURATION_MS, tuner_active_pids_count);
    tuner_is_running = true;
    esp_timer_start_periodic(tuner_timer_handle, LOOP_PERIOD_US); 

    // 5. Wait for the timer task to finish gathering data
    while (tuner_is_running) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

// 6. Print the generated CSV dynamically based on the number of PIDs
    RAVEN_LOGI("TUNER", "--- CSV START ---");
    
    // Build and print CSV Header dynamically
    char header_buf[256];
    snprintf(header_buf, sizeof(header_buf), "Tick,DeltaTime_us");
    for (uint8_t i = 0; i < tuner_active_pids_count; i++) {
        char col_buf[128]; // Aumentado para 128 para satisfazer o compilador
        snprintf(col_buf, sizeof(col_buf), ",Reading_%u,Setpoint_%u,Output_%u,kp_%u,ki_%u,kd_%u,int_%u", i, i, i, i, i, i, i);
        strlcat(header_buf, col_buf, sizeof(header_buf)); 
    }
    RAVEN_LOGI("TUNER", "%s", header_buf);

    // Build and print CSV Rows
    for (int i = 0; i < TOTAL_SAMPLES; i++) {
        int64_t delta_t = (i > 0) ? (tuner_log_buffer[i].timestamp_us - tuner_log_buffer[i-1].timestamp_us) : 0;
        
        char row_buf[1024]; // Aumentado para 1024 para aguentar várias colunas de floats tranquilamente
        snprintf(row_buf, sizeof(row_buf), "%d,%lld", i, delta_t);
        
        for (uint8_t j = 0; j < tuner_active_pids_count; j++) {
            char val_buf[256]; // Aumentado para 256 para evitar truncamento com os floats
            snprintf(val_buf, sizeof(val_buf), ",%.2f,%.2f,%.2f,%.6f,%.6f,%.6f,%.2f",
                     tuner_log_buffer[i].reading[j],
                     tuner_log_buffer[i].setpoint[j],
                     tuner_log_buffer[i].output[j],
                     tuner_log_buffer[i].kp[j],
                     tuner_log_buffer[i].ki[j],
                     tuner_log_buffer[i].kd[j],
                     tuner_log_buffer[i].integral_sum[j]);
            strlcat(row_buf, val_buf, sizeof(row_buf));
        }
        RAVEN_LOGI("TUNER", "%s", row_buf);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    RAVEN_LOGI("TUNER", "--- CSV END ---");

    // 7. Cleanup
    free(tuner_log_buffer);
    tuner_log_buffer = NULL;
    tuner_active_pids = NULL;
}


/* =========================================================================
 * 4. SPECIFIC TUNER IMPLEMENTATIONS
 * ========================================================================= */

/**
 * @brief Update callback for driving both motors simultaneously.
 * No parameters needed; it knows which globals to interact with.
 */
static void dual_motor_update_cb(void) {
    controller_motors_run();
}

/**
 * @brief Shutdown callback for both motors.
 */
static void dual_motor_stop_cb(void) {
    motor_set_voltage(MOTOR_LEFT, 0.0f);
    motor_set_voltage(MOTOR_RIGHT, 0.0f);
}

/**
 * @brief Triggers the tuning sequence specifically for both drive motors.
 */
void controller_tune_drive_motors(void) {
    // We pass an array of the PID contexts we want the logger to track.
    // Index 0 = Left, Index 1 = Right.
    pid_context_t *pids_to_track[] = {&left_motor_pid, &right_motor_pid};
    
    controller_run_generic_tuner(pids_to_track, 2, dual_motor_update_cb, dual_motor_stop_cb);
}