#pragma once

#include <stdint.h>
#include <stdbool.h>

// Project Modules (If you chose to pass structs directly)
#include "line_reading.h"
#include "odometry.h"

// 60 seconds at 100 Hz = 6000 samples. 
// At 29 bytes per sample, this requires 174,000 bytes (~169 KB) of heap RAM.
#define TELEMETRY_MAX_SAMPLES 6000 

// The highly compressed 29-byte data frame
typedef struct __attribute__((packed)) {
    uint16_t time_ms;           // Milliseconds since race start (Max 65.5s)
    uint16_t distance_mm;       // Total distance in mm (Max 65.5 meters)
    uint16_t markers;           // Packed bitfield: [Status:2][Left:2][Right:2][Cross:2]
    
    int16_t pose_x_mm;          // X position in mm (Max +/- 32.7 meters)
    int16_t pose_y_mm;          // Y position in mm (Max +/- 32.7 meters)
    int16_t yaw_mrad;           // Yaw in milliradians (1 rad = 1000 mrad)
    
    int16_t vel_left_mmps;      // Left wheel speed (mm/s)
    int16_t vel_right_mmps;     // Right wheel speed (mm/s)
    
    int16_t line_position;      // Raw or normalized line position
    int16_t pid_line_out;       // Line PID output * 1000
    
    int16_t pid_left_set;       // Left PID setpoint * 1000
    int16_t pid_right_set;      // Right PID setpoint * 1000
    int16_t pid_left_out;       // Left PID output * 1000
    int16_t pid_right_out;      // Right PID output * 1000
    
    uint8_t battery_dv;         // Battery in decivolts (e.g. 8.4V = 84)
    uint8_t fan_dv;             // Fan voltage in decivolts (e.g. 5.0V = 50) <-- NEW
} telemetry_frame_t;

void robot_telemetry_init(void);
void robot_telemetry_record_frame(void);
void robot_telemetry_trigger_download(void);