#pragma once

/**
 * @file DRV8874.h
 * @brief Native Hardware Abstraction Layer for DRV8874 Motor Driver using ESP-IDF MCPWM.
 * * This module bypasses the bdc_motor library to gain explicit control over 
 * the IN1 and IN2 pins, enabling Slow Decay (active braking) behavior.
 */

#include <stdint.h>

/**
 * @brief Configuration structure for the DRV8874 hardware initialization.
 */
typedef struct {
    uint32_t pin_in1;       /**< GPIO for IN1 */
    uint32_t pin_in2;       /**< GPIO for IN2 */
    uint32_t pwm_freq_hz;   /**< PWM frequency (e.g., 50000 for 50kHz) */
    uint32_t resolution_hz; /**< Timer resolution (e.g., 40000000 for 40MHz) */
    int group_id;           /**< MCPWM group ID (usually 0) */
} drv8874_config_t;

/**
 * @brief Opaque handle to the internal motor context.
 */
typedef struct drv8874_context_t* drv8874_handle_t;

/**
 * @brief Initializes the DRV8874 driver with native MCPWM hardware.
 * * @param[in] config Pointer to the configuration struct.
 * @param[out] ret_motor Pointer to store the created motor handle.
 */
void DRV8874_init(const drv8874_config_t *config, drv8874_handle_t *ret_motor);

/**
 * @brief Sets the motor speed and direction using Slow Decay logic.
 * * @param[in] motor Handle of the motor to control.
 * @param[in] pwm_ticks Desired speed in raw MCPWM ticks (negative for reverse).
 */
void DRV8874_set_pwm_value(drv8874_handle_t motor, int32_t pwm_ticks);

/**
 * @brief Brakes the motor actively (Slow Decay: IN1=HIGH, IN2=HIGH).
 * * @param[in] motor Handle of the motor.
 */
void DRV8874_brake(drv8874_handle_t motor);

/**
 * @brief Coasts the motor (Fast Decay: IN1=LOW, IN2=LOW).
 * * @param[in] motor Handle of the motor.
 */
void DRV8874_coast(drv8874_handle_t motor);