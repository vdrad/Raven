/**
 * @file DRV8874.h
 * @brief Hardware Abstraction Layer for DRV8874 Motor Driver using MCPWM.
 */
#pragma once

#include "bdc_motor.h"
#include <stdint.h>

void DRV8874_init(const bdc_motor_config_t *motor_config, const bdc_motor_mcpwm_config_t *mcpwm_config, bdc_motor_handle_t *ret_motor);

/**
 * @brief Sets the motor speed and direction.
 * @param[in] motor Handle of the motor to control.
 * @param[in] pwm_ticks Desired speed in raw MCPWM ticks (can be negative for reverse).
 * @param[in] max_ticks The maximum allowed ticks (used for clamping).
 */
void DRV8874_set_pwm_value(bdc_motor_handle_t motor, int32_t pwm_ticks, uint32_t max_ticks);

void DRV8874_brake(bdc_motor_handle_t motor);
void DRV8874_coast(bdc_motor_handle_t motor);