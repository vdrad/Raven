/**
 * @file odometry.h
 * @brief Procedural Odometry module for calculating distance and speed.
 *
 * This module tracks the robot's kinematic state (speeds and distances) 
 * by fusing data from the left and right hardware encoders and calculating 
 * the true delta-time between reads.
 */

#pragma once

#include <stdbool.h>

/**
 * @brief Structure containing the real-time kinematic data of the robot.
 */
typedef struct {
    float velocity_left_m_s;         /**< Current left wheel speed in m/s */
    float velocity_right_m_s;        /**< Current right wheel speed in m/s */
    float velocity_robot_m_s;        /**< Current linear speed of the robot center in m/s */
    float distance_traveled_robot_m; /**< Absolute distance traveled by the robot in meters */
} odometry_data_t;

/**
 * @brief Initializes the Odometry hardware and state variables.
 */
void odometry_init(void);

/**
 * @brief Procedural update function. 
 * * Must be called periodically by the master control loop. It self-calculates 
 * the exact delta time since the last call for maximum precision.
 */
void odometry_update(void);

/**
 * @brief Retrieves the latest computed kinematic data of the robot.
 * * @return odometry_data_t copy of the current speeds and distance.
 */
odometry_data_t odometry_get_data(void);

/**
 * @brief Resets the internal odometry distance and encoder baselines to zero.
 */
void odometry_reset(void);