#include "controller.h"

// Standard Libraries
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h> // Added for sqrtf

// FreeRTOS
#include "freertos/FreeRTOS.h"

// Project Includes
#include "controller_pid.h"
#include "raven_log.h"
#include "raven_comm.h"
#include "motor.h"
#include "odometry.h"
#include "esp_timer.h"

#define TAG "TUN"

/* --- TUNER CONFIGURATIONS --- */
#define TARGET_DISTANCE_M       1.0f  // Fixed theoretical travel distance (spatial constraint)
#define ACCELERATION_RATE_M_S2  9.0f  // Profile acceleration and deceleration rate
#define SETPOINT_SPEED_M_S      1.0f  // Desired cruise speed target
#define LOOP_PERIOD_US          1000  // 1ms high-frequency control loop

// Max number of PIDs you can tune at the same time (e.g., Left, Right, Yaw)
#define MAX_SIMULTANEOUS_PIDS 2

/* --- 1. GLOBAL TUNER STATE --- */
typedef struct {
    int64_t timestamp_us;
    float reading[MAX_SIMULTANEOUS_PIDS];
    float setpoint[MAX_SIMULTANEOUS_PIDS];
    float output[MAX_SIMULTANEOUS_PIDS];
    float integral_sum[MAX_SIMULTANEOUS_PIDS]; 
} telemetry_sample_t;

static telemetry_sample_t *tuner_log_buffer = NULL;
static int tuner_current_sample = 0;
static esp_timer_handle_t tuner_timer_handle = NULL;
static bool tuner_is_running = false;

/* Profile Inflection Points (Calculated at runtime) */
static uint32_t tuner_total_samples = 0;
static uint32_t tuner_phase_1_end = 0;
static uint32_t tuner_phase_2_end = 0;
static float tuner_peak_speed = 0.0f;

/* Generic state for the modular tuner */
static pid_context_t **tuner_active_pids = NULL;
static uint8_t tuner_active_pids_count = 0;
static void (*tuner_update_cb)(void) = NULL;
static void (*tuner_stop_cb)(void) = NULL;

/* --- 2. TIMER CALLBACK --- */
/**
 * @brief Hardware timer callback executing the distance-constrained velocity profile loop.
 */
static void pid_tuner_timer_callback(void* arg) {
    // --- 0. CAPTURE TIMESTAMP IMMEDIATELY ---
    int64_t current_timestamp = esp_timer_get_time();
    
    // --- TRAPEZOIDAL/TRIANGULAR PROFILE GENERATOR ---
    float current_setpoint = 0.0f;

    if (tuner_current_sample < tuner_phase_1_end) {
        // Phase 1: Acceleration (0 to Peak Speed)
        if (tuner_phase_1_end > 0) {
            current_setpoint = tuner_peak_speed * ((float)tuner_current_sample / tuner_phase_1_end);
        }
    } 
    else if (tuner_current_sample < tuner_phase_2_end) {
        // Phase 2: Steady State (Constant Velocity)
        current_setpoint = tuner_peak_speed;
    } 
    else if (tuner_current_sample < tuner_total_samples) {
        // Phase 3: Deceleration (Peak Speed to 0)
        uint32_t decel_samples = tuner_total_samples - tuner_phase_2_end;
        uint32_t samples_into_decel = tuner_current_sample - tuner_phase_2_end;
        if (decel_samples > 0) {
            current_setpoint = tuner_peak_speed * (1.0f - ((float)samples_into_decel / decel_samples));
        }
    }

    // Apply the dynamic profile setpoint to all tracked PIDs
    for (uint8_t i = 0; i < tuner_active_pids_count; i++) {
        tuner_active_pids[i]->setpoint = current_setpoint;
    }

    // Step 1: Execute the user-defined update function (reads sensors, computes PIDs, actuates)
    if (tuner_update_cb != NULL) {
        tuner_update_cb();
    }

    // Step 2: Save the snapshot of ALL tracked PIDs into the heap buffer
    if (tuner_current_sample < tuner_total_samples && tuner_log_buffer != NULL) {
        tuner_log_buffer[tuner_current_sample].timestamp_us = current_timestamp;
        
        for (uint8_t i = 0; i < tuner_active_pids_count; i++) {
            tuner_log_buffer[tuner_current_sample].reading[i]      = tuner_active_pids[i]->current_reading;
            tuner_log_buffer[tuner_current_sample].setpoint[i]     = tuner_active_pids[i]->setpoint; 
            tuner_log_buffer[tuner_current_sample].output[i]       = tuner_active_pids[i]->output;
            tuner_log_buffer[tuner_current_sample].integral_sum[i] = tuner_active_pids[i]->integral_sum;
        }
        tuner_current_sample++;
    }

    // Step 3: Stop Condition
    if (tuner_current_sample >= tuner_total_samples) {
        esp_timer_stop(tuner_timer_handle);
        
        if (tuner_stop_cb != NULL) {
            tuner_stop_cb();
        }
        
        tuner_is_running = false;
    }
}

/* --- 3. GENERIC TUNER TRIGGER --- */
/**
 * @brief Runs a high-precision tuning sequence for multiple PID controllers bounded by distance.
 */
void controller_run_generic_tuner(pid_context_t **target_pids, uint8_t num_pids, 
                                  void (*update_func)(void), 
                                  void (*stop_func)(void)) {
                                      
    if (tuner_is_running) {
        RAVEN_LOGE("TUNER", "Tuner is already running!");
        return;
    }

    // --- Kinematic Calculations for Distance-Based Bounding ---
    float loop_period_s = LOOP_PERIOD_US / 1000000.0f;
    tuner_peak_speed = SETPOINT_SPEED_M_S;

    // Check if the requested profile must force a pure triangle due to distance limits
    float max_reachable_speed = sqrtf(ACCELERATION_RATE_M_S2 * TARGET_DISTANCE_M);
    if (tuner_peak_speed > max_reachable_speed) {
        tuner_peak_speed = max_reachable_speed;
        RAVEN_LOGW("TUNER", "Setpoint speed too high for target distance! Clamped to %.2f m/s (Triangular Profile)", tuner_peak_speed);
    }

    float t_ramp = tuner_peak_speed / ACCELERATION_RATE_M_S2;
    float t_const = (TARGET_DISTANCE_M / tuner_peak_speed) - (tuner_peak_speed / ACCELERATION_RATE_M_S2);
    if (t_const < 0.0f) t_const = 0.0f; // Safety sanity clamp

    // Convert time segments into discrete loop iteration ticks
    uint32_t samples_ramp = (uint32_t)(t_ramp / loop_period_s);
    uint32_t samples_const = (uint32_t)(t_const / loop_period_s);

    tuner_phase_1_end = samples_ramp;
    tuner_phase_2_end = samples_ramp + samples_const;
    tuner_total_samples = samples_ramp + samples_const + samples_ramp;

    uint32_t derived_duration_ms = (uint32_t)((t_ramp * 2.0f + t_const) * 1000.0f);

    // Clamp the number of PIDs to the maximum allowed to prevent memory corruption
    tuner_active_pids_count = (num_pids > MAX_SIMULTANEOUS_PIDS) ? MAX_SIMULTANEOUS_PIDS : num_pids;

    // 1. Setup global state
    tuner_active_pids = target_pids;
    tuner_update_cb = update_func;
    tuner_stop_cb = stop_func;

    // 2. Allocate heap memory dynamically based on calculated total sample steps
    tuner_log_buffer = (telemetry_sample_t *)malloc(tuner_total_samples * sizeof(telemetry_sample_t));
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

    RAVEN_LOGI("TUNER", "Starting %u ms sequence (Target: %.2f m) for %d PIDs...", derived_duration_ms, TARGET_DISTANCE_M, tuner_active_pids_count);
    tuner_is_running = true;
    esp_timer_start_periodic(tuner_timer_handle, LOOP_PERIOD_US); 

    // 5. Wait for the timer task to finish gathering data
    while (tuner_is_running) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // 6. Print the generated CSV dynamically based on the number of PIDs
    raven_comm_send_message("TUNER", "--- CSV START ---");
    
    // --- 6a. Print Metadata Header (PID Constants + Profile Configuration) ---
    char meta_buf[256];
    snprintf(meta_buf, sizeof(meta_buf), "Metadata,Total samples: %ld", tuner_total_samples);
    for (uint8_t i = 0; i < tuner_active_pids_count; i++) {
        char pid_buf[128];
        snprintf(pid_buf, sizeof(pid_buf), ",PID_%u(P:%.5f I:%.5f D:%.5f)", 
                 i, tuner_active_pids[i]->kP, tuner_active_pids[i]->kI, tuner_active_pids[i]->kD);
        strlcat(meta_buf, pid_buf, sizeof(meta_buf));
    }
    char profile_meta[64];
    snprintf(profile_meta, sizeof(profile_meta), ",Dist:%.2fm,V_peak:%.2fm/s", TARGET_DISTANCE_M, tuner_peak_speed);
    strlcat(meta_buf, profile_meta, sizeof(meta_buf));
    raven_comm_send_message("TUNER", "%s", meta_buf);

    // --- 6b. Build and print standard CSV Column Header ---
    char header_buf[256];
    snprintf(header_buf, sizeof(header_buf), "Tick,DeltaTime_us");
    for (uint8_t i = 0; i < tuner_active_pids_count; i++) {
        char col_buf[128];
        snprintf(col_buf, sizeof(col_buf), ",Reading_%u,Setpoint_%u,Output_%u,int_%u", i, i, i, i);
        strlcat(header_buf, col_buf, sizeof(header_buf)); 
    }
    raven_comm_send_message("TUNER", "%s", header_buf);

    // --- 6c. Build and print CSV Rows ---
    for (uint32_t i = 0; i < tuner_total_samples; i++) {
        int64_t delta_t = (i > 0) ? (tuner_log_buffer[i].timestamp_us - tuner_log_buffer[i-1].timestamp_us) : 0;
        
        char row_buf[512]; 
        snprintf(row_buf, sizeof(row_buf), "%lu,%lld", i, delta_t);
        
        for (uint8_t j = 0; j < tuner_active_pids_count; j++) {
            char val_buf[128];
            snprintf(val_buf, sizeof(val_buf), ",%.2f,%.2f,%.2f,%.2f",
                     1000.0f * tuner_log_buffer[i].reading[j],
                     1000.0f * tuner_log_buffer[i].setpoint[j],
                     tuner_log_buffer[i].output[j],
                     tuner_log_buffer[i].integral_sum[j]);
            strlcat(row_buf, val_buf, sizeof(row_buf));
        }
        raven_comm_send_message("TUNER", "%s", row_buf);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    raven_comm_send_message("TUNER", "--- CSV END ---");

    // 7. Cleanup
    free(tuner_log_buffer);
    tuner_log_buffer = NULL;
    tuner_active_pids = NULL;
}

/* =========================================================================
 * 4. SPECIFIC TUNER IMPLEMENTATIONS
 * ========================================================================= */

static void dual_motor_update_cb(void) {
    controller_motors_run_tuning();
}

static void dual_motor_stop_cb(void) {
    motor_set_voltage(MOTOR_LEFT, 0.0f);
    motor_set_voltage(MOTOR_RIGHT, 0.0f);
}

void controller_tune_drive_motors(void) {
    pid_context_t *pids_to_track[] = {&left_motor_pid, &right_motor_pid};
    controller_run_generic_tuner(pids_to_track, 2, dual_motor_update_cb, dual_motor_stop_cb);
}