/**
 * @file motor.h
 * @brief Application-level manager for the robot's locomotion and auxiliary motors.
 */
#pragma once

#include <stdint.h>

/**
 * @brief Identifiers for the robot's motor drivers.
 * Used as indices for the internal motor array.
 */
typedef enum {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT,
    MOTOR_MAX_COUNT /**< Always keep at the end to determine array size */
} motor_id_t;

void motor_init(void);

/**
 * @brief Sets the speed of a specific motor using percentages.
 * @param[in] id The target motor (MOTOR_LEFT, MOTOR_RIGHT).
 * @param[in] speed_percent Speed from -100.0 (reverse) to 100.0 (forward).
 */
void motor_set_speed_percent(motor_id_t id, float speed_percent);

void motor_brake(motor_id_t id);
void motor_coast(motor_id_t id);

/**
 * @brief Blocking diagnostic task to validate all configured motors.
 * Iterates through all motors applying basic movements.
 */
void motor_peripheral_validation(void);