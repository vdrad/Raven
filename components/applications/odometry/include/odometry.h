#pragma once

/**
 * @file odometry.h
 * @brief Procedural Odometry module for calculating distance and speed.
 */

#include <stdbool.h>

/**
 * @brief Structure containing the real-time kinematic data of the robot.
 */
typedef struct {
    float velocity_left_m_s;        /**< Current left wheel speed in mm/s */
    float velocity_right_m_s;       /**< Current right wheel speed in mm/s */
    float velocity_robot_m_s;       /**< Current linear speed of the robot center in mm/s */
    float distance_traveled_robot_m; /**< Absolute distance traveled by the robot in meters */
} odometry_data_t;

/**
 * @brief Initializes the Odometry hardware and state variables.
 */
void odometry_init(void);

/**
 * @brief Procedural update function. Must be called periodically by the master control loop.
 * It self-calculates the exact delta time since the last call for maximum precision.
 */
void odometry_update(void);

/**
 * @brief Retrieves the latest computed kinematic data of the robot.
 * @note To reset the distance traveled to zero (e.g., at the start line), 
 * you must call encoder_reset_count() from the encoder module.
 * @return odometry_data_t copy of the current speeds and distance.
 */
odometry_data_t odometry_get_data(void);

void odometry_reset(void);