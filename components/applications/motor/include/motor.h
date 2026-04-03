/**
 * @file motor.h
 * @brief Application-level manager for the robot's locomotion and auxiliary motors.
 *
 * This module abstracts the hardware layer and provides a physics-based API
 * (voltage control) to ensure consistent PID behavior regardless of battery drain.
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

/**
 * @brief Initializes all configured motors and their MCPWM hardware.
 */
void motor_init(void);

/**
 * @brief Sets the motor speed based on a target voltage to compensate for battery drop.
 * * @param[in] id The target motor (MOTOR_LEFT, MOTOR_RIGHT).
 * @param[in] voltage The desired voltage to apply to the motor. 
 * Positive values drive forward, negative values drive in reverse.
 * Safe operating limits are defined by BATTERY_MONITORING_HIGH_VOLTAGE.
 */
void motor_set_voltage(motor_id_t id, float voltage);

/**
 * @brief Forces a specific motor to stop immediately using active braking (Slow Decay).
 * * @param[in] id The target motor.
 */
void motor_brake(motor_id_t id);

/**
 * @brief Cuts power to a specific motor, allowing it to spin freely (Fast Decay).
 * * @param[in] id The target motor.
 */
void motor_coast(motor_id_t id);

/**
 * @brief Blocking diagnostic task to validate all configured motors.
 */
void motor_peripheral_validation(void);