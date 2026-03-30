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

    // v = delta_s / dt
    current_odom_data.velocity_left_mm_s  = delta_dist_left_mm / dt_s;
    current_odom_data.velocity_right_mm_s = delta_dist_right_mm / dt_s;
    
    // Robot linear speed (mm/s)
    current_odom_data.velocity_robot_mm_s = (current_odom_data.velocity_left_mm_s + current_odom_data.velocity_right_mm_s) / 2.0f;

    // Calculate absolute distance travelled
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