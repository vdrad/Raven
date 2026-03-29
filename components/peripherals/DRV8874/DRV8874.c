/**
 * @file DRV8874.c
 * @brief Hardware Abstraction Layer for DRV8874 Motor Driver.
 * * This layer strictly handles the interaction with the ESP-IDF bdc_motor API.
 * It is unaware of application-level logic like robot geometry or pin assignments.
 */

#include "DRV8874.h"
#include "raven_comm.h"
#include <stdlib.h>

#define TAG "DRV"

void DRV8874_init(const bdc_motor_config_t *motor_config, const bdc_motor_mcpwm_config_t *mcpwm_config, bdc_motor_handle_t *ret_motor) {
    if (motor_config == NULL || mcpwm_config == NULL || ret_motor == NULL) {
        return;
    }

    if (bdc_motor_new_mcpwm_device(motor_config, mcpwm_config, ret_motor) != ESP_OK) {
        raven_comm_send_message(TAG, "ERROR: Failed to allocate MCPWM for motor.");
        return;
    }

    // The driver enables the hardware immediately after creation.
    bdc_motor_enable(*ret_motor);
}

void DRV8874_set_pwm_value(bdc_motor_handle_t motor, int32_t pwm_ticks, uint32_t max_ticks) {
    if (motor == NULL) return;

    // Clamp the PWM value to the hardware maximums provided by the application
    if (pwm_ticks > (int32_t)max_ticks) pwm_ticks = max_ticks;
    if (pwm_ticks < -(int32_t)max_ticks) pwm_ticks = -max_ticks;

    uint32_t speed = abs(pwm_ticks);

    if (pwm_ticks > 0) {
        bdc_motor_forward(motor);
        bdc_motor_set_speed(motor, speed);
    } 
    else if (pwm_ticks < 0) {
        bdc_motor_reverse(motor);
        bdc_motor_set_speed(motor, speed);
    } 
    else {
        // Active brake for PID stability when requested speed is 0
        bdc_motor_brake(motor); 
    }
}

void DRV8874_brake(bdc_motor_handle_t motor) {
    if (motor == NULL) return;
    bdc_motor_brake(motor);
}

void DRV8874_coast(bdc_motor_handle_t motor) {
    if (motor == NULL) return;
    bdc_motor_coast(motor);
}