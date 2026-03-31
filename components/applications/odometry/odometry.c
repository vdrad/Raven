/**
 * @file odometry.c
 * @brief Procedural Odometry module.
 */

#include "odometry.h"
#include <stdbool.h>
#include <math.h>

#include "raven_comm.h"
#include "encoder.h"
#include "esp_timer.h" // Kept ONLY for high-precision time reading (esp_timer_get_time)

#define TAG "ODM"
static bool initialized = false;

/* --- ODOMETRY FILTER CONFIGURATION --- */
#define USE_EMA_FILTER 1          // Set to 1 to enable EMA filter, 0 to bypass it
#define ODOMETRY_EMA_ALPHA 0.2f   // Smoothing factor: 0.0 (ignore new) to 1.0 (no smoothing)

/* Mechanical Constants */
#define ENCODER_GEAR_TEETH  12.0f
#define WHEEL_GEAR_TEETH    60.0f
#define WHEEL_DIAMETER_MM   28.0f

#define PULSES_PER_ENCODER_REVOLUTION   2048.0f
#define GEAR_RATIO                      (WHEEL_GEAR_TEETH / ENCODER_GEAR_TEETH)
#define PULSES_PER_WHEEL_REVOLUTION     (PULSES_PER_ENCODER_REVOLUTION * GEAR_RATIO)

/* Pre-computed constant: How many millimeters the wheel moves per 1 encoder tick */
#define MM_PER_TICK ((M_PI * WHEEL_DIAMETER_MM) / PULSES_PER_WHEEL_REVOLUTION)

/* Internal State Variables */
static odometry_data_t current_odom_data = {0};
static int last_left_count = 0;
static int last_right_count = 0;
static int64_t last_time_us = 0;

#if USE_EMA_FILTER
static float filtered_vel_left = 0.0f;
static float filtered_vel_right = 0.0f;
#endif

void odometry_update(void) {
    if (!initialized) return;

    int left_count, right_count;
    
    // Read hardware EXACTLY once per cycle
    encoder_get_count(ENCODER_LEFT, &left_count);
    encoder_get_count(ENCODER_RIGHT, &right_count);
    
    int64_t now_us = esp_timer_get_time();

    // Calculate real delta time in seconds
    float dt_s = (float)(now_us - last_time_us) / 1000000.0f;
    
    // Protection against division by zero or calling the function too fast (e.g., dt < 1ms)
    if (dt_s <= 0.001f) return; 

    // Calculate delta ticks since last update
    int delta_left = left_count - last_left_count;
    int delta_right = right_count - last_right_count;

    // Calculate displacement of each wheel in this timeframe (mm)
    float delta_dist_left_mm = (float)delta_left * MM_PER_TICK;
    float delta_dist_right_mm = (float)delta_right * MM_PER_TICK;

    // Calculate raw velocity in m/s
    float raw_vel_left = (delta_dist_left_mm / dt_s) / 1000.0f;
    float raw_vel_right = (delta_dist_right_mm / dt_s) / 1000.0f;

#if USE_EMA_FILTER
    // Apply Exponential Moving Average (EMA) filter
    filtered_vel_left = (ODOMETRY_EMA_ALPHA * raw_vel_left) + ((1.0f - ODOMETRY_EMA_ALPHA) * filtered_vel_left);
    filtered_vel_right = (ODOMETRY_EMA_ALPHA * raw_vel_right) + ((1.0f - ODOMETRY_EMA_ALPHA) * filtered_vel_right);
    
    current_odom_data.velocity_left_m_s = filtered_vel_left;
    current_odom_data.velocity_right_m_s = filtered_vel_right;
#else
    // Bypass filter, pass raw data directly to PID
    current_odom_data.velocity_left_m_s = raw_vel_left;
    current_odom_data.velocity_right_m_s = raw_vel_right;
#endif

    // Robot linear speed (m/s)
    current_odom_data.velocity_robot_m_s = (current_odom_data.velocity_left_m_s + current_odom_data.velocity_right_m_s) / 2.0f;

    // Calculate absolute distance travelled (m)
    float distance_travelled = (float)(left_count + right_count) / 2.0f;
    distance_travelled *= MM_PER_TICK;
    distance_travelled /= 1000.0f;
    current_odom_data.distance_traveled_robot_m = distance_travelled;

    // Save state for the next cycle
    last_left_count = left_count;
    last_right_count = right_count;
    last_time_us = now_us;
}

void odometry_init(void) {
    if (initialized) return;

    encoder_init();

    // Initialize state variables to prevent huge dt spikes on the first update call
    encoder_get_count(ENCODER_LEFT, &last_left_count);
    encoder_get_count(ENCODER_RIGHT, &last_right_count);
    last_time_us = esp_timer_get_time();

    initialized = true;
    raven_comm_send_message(TAG, "Initialized in procedural mode.");
}

odometry_data_t odometry_get_data(void) {
    return current_odom_data;
}

void odometry_reset(void) {
    if (!initialized) return;

    // 1. Reset hardware tick baselines
    encoder_get_count(ENCODER_LEFT, &last_left_count);
    encoder_get_count(ENCODER_RIGHT, &last_right_count);
    
    // 2. Reset time baseline to NOW
    last_time_us = esp_timer_get_time();

    // 3. Wipe the EMA filter memory
#if USE_EMA_FILTER
    filtered_vel_left = 0.0f;
    filtered_vel_right = 0.0f;
#endif

    // 4. Zero out the current struct
    current_odom_data.velocity_left_m_s = 0.0f;
    current_odom_data.velocity_right_m_s = 0.0f;
    current_odom_data.velocity_robot_m_s = 0.0f;
}