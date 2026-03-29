#pragma once

/**
 * @file odometry.h
 * @brief Odometry module for calculating distance and speed.
 */

#include <stdbool.h>

/**
 * @brief Structure containing the real-time kinematic data of the robot.
 */
typedef struct {
    float velocity_left_mm_s;        /**< Current left wheel speed in mm/s */
    float velocity_right_mm_s;       /**< Current right wheel speed in mm/s */
    float velocity_robot_mm_s;       /**< Current linear speed of the robot center in mm/s */
    float distance_traveled_robot_m; /**< Total distance traveled by the robot center in meters */
} odometry_data_t;

/**
 * @brief Initializes the PCNT hardware and the background odometry timer.
 */
void odometry_init(void);

/**
 * @brief Retrieves the latest computed kinematic data of the robot.
 * @return odometry_data_t copy of the current speeds and distances.
 */
odometry_data_t odometry_get_data(void);