/**
 * @file odometry.c
 * @brief Odometry module using high-resolution background timers.
 */

#include "odometry.h"
#include <stdbool.h>
#include <math.h>

#include "raven_comm.h"
#include "encoder.h"
#include "esp_timer.h" 

#define TAG "ODM"
static bool initialized = false;

/* Mechanical Constants */
#define ENCODER_GEAR_TEETH  13.0f
#define WHEEL_GEAR_TEETH    42.0f
#define WHEEL_DIAMETER_MM   25.0f

#define PULSES_PER_ENCODER_REVOLUTION   2048.0f
#define GEAR_RATIO                      (WHEEL_GEAR_TEETH / ENCODER_GEAR_TEETH)
#define PULSES_PER_WHEEL_REVOLUTION     (PULSES_PER_ENCODER_REVOLUTION * GEAR_RATIO)

/* Pre-computed constant: How many millimeters the wheel moves per 1 encoder tick */
#define MM_PER_TICK ((M_PI * WHEEL_DIAMETER_MM) / PULSES_PER_WHEEL_REVOLUTION)

/* Timer update interval in microseconds */
#define ODOMETRY_UPDATE_PERIOD_US 10000

/* Internal State Variables */
static odometry_data_t current_odom_data = {0};
static int last_left_count = 0;
static int last_right_count = 0;
static int64_t last_time_us = 0;

/**
 * @brief Background timer callback that computes speeds and distances at a fixed interval.
 */
static void odometry_timer_callback(void* arg) {
    int left_count, right_count;
    
    // Read hardware EXACTLY once per cycle
    encoder_get_count(ENCODER_LEFT, &left_count);
    encoder_get_count(ENCODER_RIGHT, &right_count);
    
    int64_t now_us = esp_timer_get_time();

    // Calculate real delta time in seconds
    float dt_s = (float)(now_us - last_time_us) / 1000000.0f;
    if (dt_s <= 0.0f) return; 

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

    // Calculate distance travelled
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

    // Initialize state variables before starting the timer
    encoder_get_count(ENCODER_LEFT, &last_left_count);
    encoder_get_count(ENCODER_RIGHT, &last_right_count);
    last_time_us = esp_timer_get_time();

    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &odometry_timer_callback,
        .name = "odometry_timer",
        .dispatch_method = ESP_TIMER_TASK
    };

    esp_timer_handle_t periodic_timer;
    esp_timer_create(&periodic_timer_args, &periodic_timer);
    
    esp_timer_start_periodic(periodic_timer, ODOMETRY_UPDATE_PERIOD_US);

    initialized = true;
    raven_comm_send_message(TAG, "Initialized successfully.");
}

odometry_data_t odometry_get_data(void) {
    return current_odom_data;
}