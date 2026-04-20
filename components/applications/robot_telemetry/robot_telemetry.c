#include "robot_telemetry.h"
#include "raven_log.h"
#include "raven_comm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

// Project Modules
#include "line_reading.h"
#include "odometry.h"
#include "controller.h"
#include "battery_sensor.h"
#include "motor.h"

#define TAG "TLM"

static telemetry_frame_t *telemetry_buffer = NULL;
static uint32_t current_sample_index = 0;
static int64_t start_time_us = 0;

static TaskHandle_t download_task_handle = NULL;

/* ========================================================================== */
/* MEMORY MANAGEMENT                                                          */
/* ========================================================================== */

static bool allocate_buffer(void) {
    if (telemetry_buffer != NULL) return true; // Already allocated
    
    // Allocate 174 KB in the general heap (DIRAM)
    telemetry_buffer = (telemetry_frame_t*)malloc(sizeof(telemetry_frame_t) * TELEMETRY_MAX_SAMPLES);
    
    if (telemetry_buffer == NULL) {
        RAVEN_LOGE(TAG, "FAILED TO ALLOCATE TELEMETRY BUFFER! Insufficient RAM.");
        return false;
    }
    
    current_sample_index = 0;
    start_time_us = esp_timer_get_time();
    return true;
}

static void telemetry_free_buffer(void) {
    if (telemetry_buffer != NULL) {
        free(telemetry_buffer);
        telemetry_buffer = NULL;
    }
}

void robot_telemetry_init(void) {
    if (allocate_buffer()) raven_comm_send_message(TAG, "Initialized successfully");
}

/* ========================================================================== */
/* FAST RECORDING HOOK (Must be called inside race_manager_cb at 1kHz)        */
/* ========================================================================== */

void robot_telemetry_record_frame(void) {
    // 1. Safety Checks
    if (telemetry_buffer == NULL) return;
    if (current_sample_index >= TELEMETRY_MAX_SAMPLES) return; // Buffer full!

    // 2. Downsample from 1000 Hz to 100 Hz (Record every 10th tick)
    static uint8_t tick_counter = 0;
    tick_counter++;
    if (tick_counter < 10) return;
    tick_counter = 0;

    // 3. Gather Data
    line_reading_data_t line = line_reading_get_data();
    odometry_data_t odom = odometry_get_data();
    
    // Get time elapsed in ms
    uint16_t elapsed_ms = (uint16_t)((esp_timer_get_time() - start_time_us) / 1000);

    // Pack 16 bits: [Status:2][Left:7][Right:2][Cross:5]
    uint16_t packed_markers = ((line.markers.marker_status & 0x03) << 14)        | 
                              ((line.markers.left_markers_counter & 0x7F) << 7)  |
                              ((line.markers.right_markers_counter & 0x03) << 5) |
                              ((line.markers.crossings_counter & 0x1F));
    // 4. Quantize and Store (Fast math, zero formatting)
    telemetry_frame_t *frame = &telemetry_buffer[current_sample_index];
    
    frame->time_ms          = elapsed_ms;
    frame->distance_mm      = (uint16_t)(odom.distance_traveled_robot_m * 1000.0f);
    
    frame->pose_x_mm        = (int16_t)(odom.pose_x_m * 1000.0f);
    frame->pose_y_mm        = (int16_t)(odom.pose_y_m * 1000.0f);
    frame->yaw_mrad         = (int16_t)(odom.yaw_rad * 1000.0f);
    frame->accel_x_mg       = (int16_t)(odom.acceleration_x * 1000.0f);
    frame->accel_y_mg       = (int16_t)(odom.acceleration_y * 1000.0f);
    
    frame->vel_left_mmps    = (int16_t)(odom.velocity_left_m_s * 1000.0f);
    frame->vel_right_mmps   = (int16_t)(odom.velocity_right_m_s * 1000.0f);
    
    frame->line_position    = (int16_t)(line.position.position);
    frame->pid_line_out     = (int16_t)(line_position_pid.output * 1000.0f);
    
    frame->pid_left_set     = (int16_t)(left_motor_pid.setpoint * 1000.0f);
    frame->pid_right_set    = (int16_t)(right_motor_pid.setpoint * 1000.0f);
    frame->pid_left_out     = (int16_t)(left_motor_pid.output * 1000.0f);
    frame->pid_right_out    = (int16_t)(right_motor_pid.output * 1000.0f);
    
    frame->battery_dv       = (uint8_t)(battery_sensor_get_voltage() * 10.0f);
    frame->markers          = packed_markers;
    
    // --> NEW: Grab fan voltage (Adjust getter to match your codebase if needed)
    frame->fan_dv           = (uint8_t)(motor_get_voltage(MOTOR_FAN) * 10.0f); 

    // 5. Advance index
    current_sample_index++;
}

/* ========================================================================== */
/* SLOW DOWNLOAD TASK (Triggered via Bluetooth after the race)                */
/* ========================================================================== */

static void download_task(void *pvParameters) {
    if (telemetry_buffer == NULL || current_sample_index == 0) {
        raven_comm_send_message(TAG, "No telemetry data available.");
        vTaskDelete(NULL);
        return;
    }

    raven_comm_send_message(TAG, "--- TELEMETRY START (%lu samples) ---", current_sample_index);
    // Updated header string
    raven_comm_send_message(TAG, "Time_ms,Dist_mm,X_mm,Y_mm,Yaw_mrad,Accel_x_mg,Accel_y_mg,VelL_mmps,VelR_mmps,LinePos,PidLine,PidLSet,PidRSet,PidLOut,PidROut,Bat_dv,Markers,Fan_dv");
    raven_comm_send_message(TAG, "Metadata: Line kP: %.6f, Line kI: %.6f, Line kD: %.6f", line_position_pid.kP, line_position_pid.kI, line_position_pid.kD);

    char row_buf[256];
    for (uint32_t i = 0; i < current_sample_index; i++) {
        telemetry_frame_t *f = &telemetry_buffer[i];

        // Updated format string and parameter list
        snprintf(row_buf, sizeof(row_buf), 
            "%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u,%u",
            f->time_ms, f->distance_mm, f->pose_x_mm, f->pose_y_mm, f->yaw_mrad,
            f->accel_x_mg,f->accel_y_mg,f->vel_left_mmps, f->vel_right_mmps, 
            f->line_position, f->pid_line_out,f->pid_left_set, f->pid_right_set, 
            f->pid_left_out, f->pid_right_out,f->battery_dv, f->markers, f->fan_dv);

        raven_comm_send_message(TAG, "%s", row_buf);
        
        // Yield to allow the Bluetooth stack to clear its internal TX buffers
        vTaskDelay(pdMS_TO_TICKS(20)); 
    }

    raven_comm_send_message(TAG, "--- TELEMETRY END ---");
    
    // Free the RAM now that download is complete to prepare for the next run
    telemetry_free_buffer(); 
    vTaskDelete(NULL);
}

void robot_telemetry_trigger_download(void) {
    if (download_task_handle == NULL || eTaskGetState(download_task_handle) == eDeleted) {
        xTaskCreate(download_task, "dl_task", 4096, NULL, 3, &download_task_handle);
    } else {
        RAVEN_LOGW(TAG, "Download already in progress!");
    }
}