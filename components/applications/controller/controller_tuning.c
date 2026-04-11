#include "controller.h"

// Standard Libraries
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// FreeRTOS
#include "freertos/FreeRTOS.h"

// Project Includes
#include "pid.h"
#include "raven_log.h"
#include "raven_comm.h"
#include "motor.h"
#include "odometry.h"
#include "esp_timer.h"

#define TAG "TUN"

/* --- TUNER CONFIGURATIONS --- */
#define ACCELERATION_RATE_M_S2  8.0f 
#define SETPOINT_SPEED_M_S      1.5f 

#define TUNER_DURATION_MS ((uint32_t)(4.0f * (SETPOINT_SPEED_M_S / ACCELERATION_RATE_M_S2) * 1000.0f)) 
#define LOOP_PERIOD_US    1000
#define TOTAL_SAMPLES     (uint32_t)(TUNER_DURATION_MS / (LOOP_PERIOD_US / 1000.0f))

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
    float integral_sum[MAX_SIMULTANEOUS_PIDS]; // Kept because it changes dynamically
} telemetry_sample_t;

static telemetry_sample_t *tuner_log_buffer = NULL;
static int tuner_current_sample = 0;
static esp_timer_handle_t tuner_timer_handle = NULL;
static bool tuner_is_running = false;

/* Generic state for the modular tuner */
static pid_context_t **tuner_active_pids = NULL;
static uint8_t tuner_active_pids_count = 0;
static void (*tuner_update_cb)(void) = NULL;
static void (*tuner_stop_cb)(void) = NULL;

/* --- 2. TIMER CALLBACK --- */
/**
 * @brief Hardware timer callback that executes the high-frequency tuning loop.
 */
static void pid_tuner_timer_callback(void* arg) {
    
    // --- TRAPEZOIDAL PROFILE GENERATOR ---
    float current_setpoint = 0.0f;
    int phase_1_end = TOTAL_SAMPLES / 4;          // 25% point
    int phase_2_end = (TOTAL_SAMPLES * 3) / 4;    // 75% point

    if (tuner_current_sample < phase_1_end) {
        // Phase 1: Acceleration (0 to Target)
        current_setpoint = SETPOINT_SPEED_M_S * ((float)tuner_current_sample / phase_1_end);
    } 
    else if (tuner_current_sample < phase_2_end) {
        // Phase 2: Steady State
        current_setpoint = SETPOINT_SPEED_M_S;
    } 
    else if (tuner_current_sample < TOTAL_SAMPLES) {
        // Phase 3: Deceleration (Target to 0)
        int decel_samples = TOTAL_SAMPLES - phase_2_end;
        int samples_into_decel = tuner_current_sample - phase_2_end;
        current_setpoint = SETPOINT_SPEED_M_S * (1.0f - ((float)samples_into_decel / decel_samples));
    }

    // Apply the dynamic setpoint to all tracked PIDs
    for (uint8_t i = 0; i < tuner_active_pids_count; i++) {
        tuner_active_pids[i]->setpoint = current_setpoint;
    }

    // Step 1: Execute the user-defined update function (reads sensors, computes PIDs, actuates)
    if (tuner_update_cb != NULL) {
        tuner_update_cb();
    }

    // Step 2: Save the snapshot of ALL tracked PIDs into the heap buffer
    if (tuner_current_sample < TOTAL_SAMPLES && tuner_log_buffer != NULL) {
        tuner_log_buffer[tuner_current_sample].timestamp_us = esp_timer_get_time();
        
        for (uint8_t i = 0; i < tuner_active_pids_count; i++) {
            tuner_log_buffer[tuner_current_sample].reading[i]      = tuner_active_pids[i]->current_reading;
            tuner_log_buffer[tuner_current_sample].setpoint[i]     = tuner_active_pids[i]->setpoint; // Will log the trapezoid!
            tuner_log_buffer[tuner_current_sample].output[i]       = tuner_active_pids[i]->output;
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
        tuner_active_pids[i]->current_reading = 0.0f;
        tuner_active_pids[i]->current_error = 0.0f;
        tuner_active_pids[i]->previous_error = 0.0f;
        tuner_active_pids[i]->delta_error = 0.0f;
        tuner_active_pids[i]->integral_sum = 0.0f;
        tuner_active_pids[i]->last_run_time_us = esp_timer_get_time(); 
        tuner_active_pids[i]->output = 0.0f;
    }
    odometry_reset();

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
    
    // --- 6a. Print Metadata Header (PID Constants) ---
    char meta_buf[256];
    snprintf(meta_buf, sizeof(meta_buf), "Metadata");
    for (uint8_t i = 0; i < tuner_active_pids_count; i++) {
        char pid_buf[128];
        snprintf(pid_buf, sizeof(pid_buf), ",PID_%u(P:%.5f I:%.5f D:%.5f)", 
                 i, tuner_active_pids[i]->kP, tuner_active_pids[i]->kI, tuner_active_pids[i]->kD);
        strlcat(meta_buf, pid_buf, sizeof(meta_buf));
    }
    RAVEN_LOGI("TUNER", "%s", meta_buf);

    // --- 6b. Build and print standard CSV Column Header ---
    char header_buf[256];
    snprintf(header_buf, sizeof(header_buf), "Tick,DeltaTime_us");
    for (uint8_t i = 0; i < tuner_active_pids_count; i++) {
        char col_buf[128];
        snprintf(col_buf, sizeof(col_buf), ",Reading_%u,Setpoint_%u,Output_%u,int_%u", i, i, i, i);
        strlcat(header_buf, col_buf, sizeof(header_buf)); 
    }
    RAVEN_LOGI("TUNER", "%s", header_buf);

    // --- 6c. Build and print CSV Rows ---
    for (int i = 0; i < TOTAL_SAMPLES; i++) {
        int64_t delta_t = (i > 0) ? (tuner_log_buffer[i].timestamp_us - tuner_log_buffer[i-1].timestamp_us) : 0;
        
        char row_buf[512]; // Reduced size since we removed 3 floats per PID!
        snprintf(row_buf, sizeof(row_buf), "%d,%lld", i, delta_t);
        
        for (uint8_t j = 0; j < tuner_active_pids_count; j++) {
            char val_buf[128];
            snprintf(val_buf, sizeof(val_buf), ",%.2f,%.2f,%.2f,%.2f",
                     1000.0f * tuner_log_buffer[i].reading[j],
                     1000.0f * tuner_log_buffer[i].setpoint[j],
                     tuner_log_buffer[i].output[j],
                     tuner_log_buffer[i].integral_sum[j]);
            strlcat(row_buf, val_buf, sizeof(row_buf));
        }
        RAVEN_LOGI("TUNER", "%s", row_buf);
        vTaskDelay(pdMS_TO_TICKS(2));
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