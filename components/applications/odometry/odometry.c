/**
 * @file odometry.c
 * @brief Procedural Odometry module implementation with Sensor Fusion architecture.
 *
 * Fuses encoder step-distance with IMU gyroscope data to project the robot's
 * movement accurately onto a 2D global coordinate plane.
 */

#include "odometry.h"

// Standard Libraries
#include <stdbool.h>
#include <math.h>

// ESP-IDF Drivers
#include "esp_timer.h"

// Project Includes
#include "encoder.h"
#include "ICM45686.h"
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
#define ENCODER_GEAR_TEETH  24.0f
#define WHEEL_GEAR_TEETH    60.0f
#define WHEEL_DIAMETER_MM   28.0f

#define PULSES_PER_ENCODER_REVOLUTION   2048.0f
#define GEAR_RATIO                      (WHEEL_GEAR_TEETH / ENCODER_GEAR_TEETH)
#define PULSES_PER_WHEEL_REVOLUTION     (PULSES_PER_ENCODER_REVOLUTION * GEAR_RATIO)

// Math constants pre-computed for maximum efficiency
#define WHEEL_CIRCUMFERENCE_M           ((WHEEL_DIAMETER_MM * M_PI) / 1000.0f)
#define METERS_PER_PULSE                (WHEEL_CIRCUMFERENCE_M / PULSES_PER_WHEEL_REVOLUTION)

/* ========================================================================== */
/* PRIVATE DATA STRUCTURES                                                    */
/* ========================================================================== */

/** @brief Intermediate data calculated from the wheel encoders */
typedef struct {
    float vel_left_m_s;
    float vel_right_m_s;
    float vel_center_m_s;
    float step_dist_m; // Distance traveled in the current dt
} encoder_odom_t;

/** @brief Intermediate data calculated from the IMU */
typedef struct {
    float yaw_rad;
    float accel_x;
    float accel_y;
} imu_odom_t;


/* ========================================================================== */
/* GLOBAL VARIABLES                                                           */
/* ========================================================================== */

static bool initialized = false;
static int64_t last_time_us = 0;

static int last_left_count = 0;
static int last_right_count = 0;

// Internal state structs
static encoder_odom_t raw_enc_data = {0};
static imu_odom_t raw_imu_data = {0};

// Final fused output
static odometry_data_t current_odom_data = {0};

/* ========================================================================== */
/* PRIVATE FUNCTION PROTOTYPES                                                */
/* ========================================================================== */

static void encoder_odometry_init(void);
static void imu_odometry_init(void);

static void encoder_odometry_update(float dt_s);
static void imu_odometry_update(float dt_s);
static void fuse_odometry(void);

static void encoder_odometry_reset(void);
static void imu_odometry_reset(void);

/* ========================================================================== */
/* PUBLIC API IMPLEMENTATIONS                                                 */
/* ========================================================================== */

void odometry_init(void) {
    if (initialized) return;

    encoder_odometry_init();
    imu_odometry_init();

    last_time_us = esp_timer_get_time();
    
    RAVEN_LOGI(TAG, "Initialized Successfully.");
    initialized = true;
}

void odometry_update(void) {
    if (!initialized) return;

    // 1. Calculate global unified delta time
    int64_t now_us = esp_timer_get_time();
    float dt_s = (float)(now_us - last_time_us) / 1000000.0f;

    // Prevent division by zero or extreme dt
    if (dt_s <= 0.001f) return;

    // 2. Fetch and process individual sensor data
    encoder_odometry_update(dt_s);
    imu_odometry_update(dt_s);

    // 3. Combine them into the final pose
    fuse_odometry();

    // 4. Save state for next cycle
    last_time_us = now_us;
}

odometry_data_t odometry_get_data(void) {
    return current_odom_data;
}

void odometry_reset(void) {
    if (!initialized) return;

    encoder_odometry_reset();
    imu_odometry_reset();

    current_odom_data = (odometry_data_t){0};
    last_time_us = esp_timer_get_time();
}

/* ========================================================================== */
/* PRIVATE FUNCTION IMPLEMENTATIONS                                           */
/* ========================================================================== */

/**
 * @brief Prepares the encoder hardware and initial states.
 */
static void encoder_odometry_init(void) {
    encoder_init();
    encoder_get_count(ENCODER_LEFT, &last_left_count);
    encoder_get_count(ENCODER_RIGHT, &last_right_count);

    #if USE_EMA_FILTER
        raven_comm_send_message(TAG, "Using EMA Filter %.1f Alpha", ODOMETRY_EMA_ALPHA);
    #else
        raven_comm_send_message(TAG, "Not using EMA Filter");
    #endif
}

/**
 * @brief Prepares the IMU odometry baseline.
 * @note Assumes the IMU hardware was already initialized by the main system.
 */
static void imu_odometry_init(void) {
    // Reset heading to exactly 0 on initialization
    raw_imu_data.yaw_rad = 0.0f; 
}

/**
 * @brief Reads wheel encoders and computes velocities and distance traveled.
 * @param dt_s Delta time in seconds since the last update.
 */
static void encoder_odometry_update(float dt_s) {
    int left_count = 0, right_count = 0;
    encoder_get_count(ENCODER_LEFT, &left_count);
    encoder_get_count(ENCODER_RIGHT, &right_count);

    int delta_left  = (int16_t)(left_count - last_left_count);
    int delta_right = (int16_t)(right_count - last_right_count);

    float left_dist_m = delta_left * METERS_PER_PULSE;
    float right_dist_m = delta_right * METERS_PER_PULSE;

    float inst_vel_left = left_dist_m / dt_s;
    float inst_vel_right = right_dist_m / dt_s;

    #if USE_EMA_FILTER
        raw_enc_data.vel_left_m_s = (ODOMETRY_EMA_ALPHA * inst_vel_left) + 
                                    ((1.0f - ODOMETRY_EMA_ALPHA) * raw_enc_data.vel_left_m_s);
        raw_enc_data.vel_right_m_s = (ODOMETRY_EMA_ALPHA * inst_vel_right) + 
                                     ((1.0f - ODOMETRY_EMA_ALPHA) * raw_enc_data.vel_right_m_s);
    #else
        raw_enc_data.vel_left_m_s = inst_vel_left;
        raw_enc_data.vel_right_m_s = inst_vel_right;
    #endif

    raw_enc_data.vel_center_m_s = (raw_enc_data.vel_left_m_s + raw_enc_data.vel_right_m_s) / 2.0f;
    raw_enc_data.step_dist_m = (left_dist_m + right_dist_m) / 2.0f;

    last_left_count = left_count;
    last_right_count = right_count;
}

/**
 * @brief Reads the IMU and performs Euler integration to update heading.
 * @param dt_s Delta time in seconds since the last update.
 */
static void imu_odometry_update(float dt_s) {
    icm45686_data_t imu_data;
    ICM45686_get_data(&imu_data);

    // Note: get_data() already converted gyro_z to Radians Per Second internally.
    // Integrate angular velocity to find total accumulated yaw angle.
    raw_imu_data.yaw_rad += (imu_data.gyro_z * dt_s);
    
    // Store accelerations if needed for advanced slip detection or filtering later
    raw_imu_data.accel_x = imu_data.accel_x;
    raw_imu_data.accel_y = imu_data.accel_y;
}

/**
 * @brief Fuses encoder and IMU data to update the global coordinate frame.
 */
static void fuse_odometry(void) {
    // 1. Pass through wheel speeds directly from encoder
    current_odom_data.velocity_left_m_s  = raw_enc_data.vel_left_m_s;
    current_odom_data.velocity_right_m_s = raw_enc_data.vel_right_m_s;
    current_odom_data.distance_traveled_robot_m += raw_enc_data.step_dist_m;

    // 2. Assign Yaw
    current_odom_data.yaw_rad = raw_imu_data.yaw_rad;

    // 3. Fuse X, Y Position using IMU Heading + Encoder Distance
    current_odom_data.pose_x_m += raw_enc_data.step_dist_m * cosf(current_odom_data.yaw_rad);
    current_odom_data.pose_y_m += raw_enc_data.step_dist_m * sinf(current_odom_data.yaw_rad);

    // 4. Update Center velocity
    current_odom_data.velocity_robot_m_s = raw_enc_data.vel_center_m_s; 
}

/**
 * @brief Resets the encoder internal variables.
 */
static void encoder_odometry_reset(void) {
    encoder_get_count(ENCODER_LEFT, &last_left_count);
    encoder_get_count(ENCODER_RIGHT, &last_right_count);
    raw_enc_data = (encoder_odom_t){0};
}

/**
 * @brief Resets the IMU internal yaw baseline.
 */
static void imu_odometry_reset(void) {
    raw_imu_data = (imu_odom_t){0};
}