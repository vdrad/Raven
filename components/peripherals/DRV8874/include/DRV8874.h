#pragma once

/**
 * @brief Identifiers for the robot's motor drivers.
 */
typedef enum {
    MOTOR_LEFT,  /**< Refers to the left motor driver */
    MOTOR_RIGHT, /**< Refers to the right motor driver */
    MOTOR_FAN    /**< Refers to the fan motor driver */
} motor_side_t;

// void DRV8874_init(const bdc_motor_config_t *motor_config, const bdc_motor_mcpwm_config_t *mcpwm_config, bdc_motor_handle_t *ret_motor);
void DRV8874_peripheral_validation(void);