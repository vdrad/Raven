/**
 * @file odometry.c
 * @brief Procedural Odometry module implementation.
 */

#include "odometry.h"

// Standard Libraries
#include <stdbool.h>
#include <math.h>

// ESP-IDF Drivers
#include "esp_timer.h"

// Project Includes
#include "encoder.h"
#include "raven_log.h"
#include "raven_comm.h"

#define TAG "ODM"

/* ========================================================================== */
/* MACROS & CONSTANTS                                                         */
/* ========================================================================== */

/* --- ODOMETRY FILTER CONFIGURATION --- */
#define USE_EMA_FILTER 1          // Set to 1 to enable EMA filter, 0 to bypass it
#define ODOMETRY_EMA_ALPHA 0.2f   // Smoothing factor: 0.0 (ignore new) to 1.0 (no smoothing)

/* --- MECHANICAL CONSTANTS --- */
#define ENCODER_GEAR_TEETH  12.0f
#define WHEEL_GEAR_TEETH    60.0f
#define WHEEL_DIAMETER_MM   28.0f

#define PULSES_PER_ENCODER_REVOLUTION   2048.0f
#define GEAR_RATIO                      (WHEEL_GEAR_TEETH / ENCODER_GEAR_TEETH)
#define PULSES_PER_WHEEL_REVOLUTION     (PULSES_PER_ENCODER_REVOLUTION * GEAR_RATIO)

// Math constants pre-computed for maximum efficiency
#define WHEEL_CIRCUMFERENCE_M           ((WHEEL_DIAMETER_MM * M_PI) / 1000.0f)
#define METERS_PER_PULSE                (WHEEL_CIRCUMFERENCE_M / PULSES_PER_WHEEL_REVOLUTION)

/* ========================================================================== */
/* GLOBAL VARIABLES                                                           */
/* ========================================================================== */

static bool initialized = false;

static int last_left_count = 0;
static int last_right_count = 0;
static int64_t last_time_us = 0;

static odometry_data_t current_odom_data = {0};

/* ========================================================================== */
/* PUBLIC API IMPLEMENTATIONS                                                 */
/* ========================================================================== */

void odometry_init(void) {
    if (initialized) return;

    // Safely ensure hardware encoders are spinning before taking baseline
    encoder_init();

    // Initialize state variables to prevent huge dt spikes on the first update call
    encoder_get_count(ENCODER_LEFT, &last_left_count);
    encoder_get_count(ENCODER_RIGHT, &last_right_count);
    last_time_us = esp_timer_get_time();

    RAVEN_LOGI(TAG, "Initialized in procedural mode.");
    
    #if USE_EMA_FILTER
        raven_comm_send_message(TAG, "Using EMA Filter %.1f Alpha", ODOMETRY_EMA_ALPHA);
    #else
        raven_comm_send_message(TAG, "Not using EMA Filter");
    #endif
    
    initialized = true;
}

void odometry_update(void) {
    if (!initialized) return;

    int left_count = 0;
    int right_count = 0;
    encoder_get_count(ENCODER_LEFT, &left_count);
    encoder_get_count(ENCODER_RIGHT, &right_count);

    int64_t now_us = esp_timer_get_time();
    float dt_s = (float)(now_us - last_time_us) / 1000000.0f;

    // Prevent division by zero or extremely high frequencies
    if (dt_s <= 0.001f) return;

    // Calculate delta ticks
    int delta_left  = (int16_t)(left_count - last_left_count);
    int delta_right = (int16_t)(right_count - last_right_count);

    // Convert to meters
    float left_dist_m = delta_left * METERS_PER_PULSE;
    float right_dist_m = delta_right * METERS_PER_PULSE;

    // Calculate Instantaneous Speeds (m/s)
    float inst_vel_left = left_dist_m / dt_s;
    float inst_vel_right = right_dist_m / dt_s;

    // Apply EMA filter or raw passing
    #if USE_EMA_FILTER
        current_odom_data.velocity_left_m_s = (ODOMETRY_EMA_ALPHA * inst_vel_left) + 
                                              ((1.0f - ODOMETRY_EMA_ALPHA) * current_odom_data.velocity_left_m_s);
        current_odom_data.velocity_right_m_s = (ODOMETRY_EMA_ALPHA * inst_vel_right) + 
                                               ((1.0f - ODOMETRY_EMA_ALPHA) * current_odom_data.velocity_right_m_s);
    #else
        current_odom_data.velocity_left_m_s = inst_vel_left;
        current_odom_data.velocity_right_m_s = inst_vel_right;
    #endif

    // Center Robot Kinematics
    current_odom_data.velocity_robot_m_s = (current_odom_data.velocity_left_m_s + current_odom_data.velocity_right_m_s) / 2.0f;
    float step_dist_m = (left_dist_m + right_dist_m) / 2.0f;
    current_odom_data.distance_traveled_robot_m += step_dist_m;

    // Save state for the next cycle
    last_left_count = left_count;
    last_right_count = right_count;
    last_time_us = now_us;
}

odometry_data_t odometry_get_data(void) {
    return current_odom_data;
}

void odometry_reset(void) {
    if (!initialized) return;

    // Reset hardware tick baselines
    encoder_get_count(ENCODER_LEFT, &last_left_count);
    encoder_get_count(ENCODER_RIGHT, &last_right_count);
    
    // Clear data
    current_odom_data.distance_traveled_robot_m = 0.0f;
    current_odom_data.velocity_left_m_s  = 0.0f;
    current_odom_data.velocity_right_m_s = 0.0f;
    current_odom_data.velocity_robot_m_s = 0.0f;

    last_time_us = esp_timer_get_time();
}