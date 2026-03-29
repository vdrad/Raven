/**
 * @file DRV8874.c
 * @brief Hardware Abstraction Layer for DRV8874 Motor Driver using MCPWM.
 * * This module acts as the lowest level driver (Layer 1), interacting directly
 * with the ESP-IDF BDC Motor component. It does not contain application logic.
 */

#include "DRV8874.h"
#include <stdio.h>
#include <stdlib.h> // Required for abs()
#include <stdbool.h>
#include "bdc_motor.h"
#include "pinout.h"
#include "raven_comm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "DRV"

/* * Optimal constants for Coreless DC Motors.
 * Yields 800 ticks of resolution at 50 kHz.
 */
#define TIMER_RESOLUTION_HZ 40000000 // 40 MHz
#define MCPWM_FREQUENCY_HZ  50000    // 50 kHz
#define MCPWM_DUTY_TICK_MAX (TIMER_RESOLUTION_HZ / MCPWM_FREQUENCY_HZ)

/**
 * @brief Initializes the DRV8874 motor driver instance.
 * * Allocates the MCPWM hardware and enables the motor driver.
 * * @param[in] motor_config Pointer to the motor GPIO and frequency configuration.
 * @param[in] mcpwm_config Pointer to the MCPWM timer resolution configuration.
 * @param[out] ret_motor Pointer to a handle where the created motor instance will be saved.
 */
void DRV8874_init(const bdc_motor_config_t *motor_config, const bdc_motor_mcpwm_config_t *mcpwm_config, bdc_motor_handle_t *ret_motor) {
    if (motor_config == NULL || mcpwm_config == NULL || ret_motor == NULL) {
        raven_comm_send_message(TAG, "ERROR: Invalid pointers passed to init.");
        return;
    }

    // Attempt to create the BDC motor instance
    if (bdc_motor_new_mcpwm_device(motor_config, mcpwm_config, ret_motor) != ESP_OK) {
        raven_comm_send_message(TAG, "ERROR: Failed to allocate MCPWM for motor.");
        return;
    }

    // Enable the motor hardware
    if (bdc_motor_enable(*ret_motor) != ESP_OK) {
        raven_comm_send_message(TAG, "ERROR: Failed to enable motor.");
        return;
    }

    raven_comm_send_message(TAG, "Motor initialized successfully.");
}

/**
 * @brief Sets the motor speed and direction based on a signed PWM value.
 * * Positive values drive the motor forward, negative values drive it backward.
 * If the value is 0, the motor will actively brake (Slow Decay).
 * * @param[in] motor The handle of the motor to control.
 * @param[in] pwm_value The desired duty cycle in ticks (range: -MCPWM_DUTY_TICK_MAX to +MCPWM_DUTY_TICK_MAX).
 */
void DRV8874_set_pwm_value(bdc_motor_handle_t motor, int32_t pwm_value) {
    if (motor == NULL) {
        raven_comm_send_message(TAG, "ERROR: Motor handle is NULL.");
        return;
    }

    // Clamp the PWM value to the hardware maximums
    if (pwm_value > MCPWM_DUTY_TICK_MAX) pwm_value = MCPWM_DUTY_TICK_MAX;
    if (pwm_value < -MCPWM_DUTY_TICK_MAX) pwm_value = -MCPWM_DUTY_TICK_MAX;

    // Apply absolute speed and correct direction
    uint32_t speed = abs(pwm_value);

    if (pwm_value > 0) {
        bdc_motor_forward(motor);
        bdc_motor_set_speed(motor, speed);
    } 
    else if (pwm_value < 0) {
        bdc_motor_reverse(motor);
        bdc_motor_set_speed(motor, speed);
    } 
    else {
        // Active brake for PID stability when requested speed is 0
        bdc_motor_brake(motor); 
    }
}

/**
 * @brief Forces the motor to stop immediately using active braking.
 * * Shorts the motor terminals (Slow Decay), stopping the rotor quickly.
 * Ideal for locomotion wheels.
 * * @param[in] motor The handle of the motor to brake.
 */
void DRV8874_brake(bdc_motor_handle_t motor) {
    if (motor == NULL) return;
    bdc_motor_brake(motor);
}

/**
 * @brief Cuts power to the motor, allowing it to spin freely.
 * * Sets the driver to High-Z (Fast Decay). The motor will stop due to friction.
 * Ideal for high-speed components like the suction turbine.
 * * @param[in] motor The handle of the motor to coast.
 */
void DRV8874_coast(bdc_motor_handle_t motor) {
    if (motor == NULL) return;
    bdc_motor_coast(motor);
}

/**
 * @brief Blocking diagnostic task to validate the DRV8874 motor driver hardware.
 * * Takes no arguments. Internally configures the pins for the Left Motor, 
 * Right Motor, and Fan sequentially. Runs a 15% speed sweep (CW and CCW for 
 * wheels, CW only for the fan to avoid damage), and then cleans up.
 */
void DRV8874_peripheral_validation(void) {
    // Helper structure to iterate through the 3 motors cleanly
    typedef struct {
        const char *name;
        uint32_t pin_a;
        uint32_t pin_b;
        bool is_turbine; // Flag to apply safe testing rules for the fan
    } val_motor_t;

    val_motor_t motors_to_test[] = {
        {"Left Motor",  LEFT_MOTOR_DIR_PIN,  LEFT_MOTOR_VEL_PIN,  false},
        {"Right Motor", RIGHT_MOTOR_DIR_PIN, RIGHT_MOTOR_VEL_PIN, false},
        {"Fan Motor",   FAN_MOTOR_DIR_PIN,   FAN_MOTOR_VEL_PIN,   true}
    };

    bdc_motor_mcpwm_config_t mcpwm_config = {
        .group_id = 0,
        .resolution_hz = TIMER_RESOLUTION_HZ,
    };

    // Calculate 15% of the maximum duty cycle
    int32_t test_speed = (int32_t)(MCPWM_DUTY_TICK_MAX * 0.05);

    for (int i = 0; i < 3; i++) {
        raven_comm_send_message(TAG, "--- Testing %s ---", motors_to_test[i].name);

        // 1. Temporary configuration for the current motor
        bdc_motor_config_t motor_config = {
            .pwma_gpio_num = motors_to_test[i].pin_a,
            .pwmb_gpio_num = motors_to_test[i].pin_b,
            .pwm_freq_hz = MCPWM_FREQUENCY_HZ,
        };

        bdc_motor_handle_t test_motor = NULL;

        // 2. Initialize temporary hardware
        if (bdc_motor_new_mcpwm_device(&motor_config, &mcpwm_config, &test_motor) != ESP_OK) {
            raven_comm_send_message(TAG, "VAL ERROR: Failed to allocate MCPWM for %s.", motors_to_test[i].name);
            continue; // Skip to the next motor if initialization fails
        }
        bdc_motor_enable(test_motor);

        // 3. Test Sequence: Forward
        raven_comm_send_message(TAG, "[%s] Testing Forward at 5%%...", motors_to_test[i].name);
        DRV8874_set_pwm_value(test_motor, test_speed);
        vTaskDelay(pdMS_TO_TICKS(2000));

        // 4. Test Sequence: Stop
        raven_comm_send_message(TAG, "[%s] Stopping...", motors_to_test[i].name);
        if (motors_to_test[i].is_turbine) {
            DRV8874_coast(test_motor); // Safe stop for turbines
        } else {
            DRV8874_brake(test_motor); // Active brake for wheels
        }
        vTaskDelay(pdMS_TO_TICKS(1000));

        // 5. Test Sequence: Reverse (Only for wheels)
        if (!motors_to_test[i].is_turbine) {
            raven_comm_send_message(TAG, "[%s] Testing Reverse at 5%%...", motors_to_test[i].name);
            DRV8874_set_pwm_value(test_motor, -test_speed);
            vTaskDelay(pdMS_TO_TICKS(2000));

            raven_comm_send_message(TAG, "[%s] Final Brake.", motors_to_test[i].name);
            DRV8874_brake(test_motor);
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        // 6. Teardown temporary hardware to free the timer for the next motor
        bdc_motor_disable(test_motor);
        bdc_motor_del(test_motor);
        
        raven_comm_send_message(TAG, "--- %s Test Complete ---", motors_to_test[i].name);
        vTaskDelay(pdMS_TO_TICKS(500)); // Brief pause before starting the next motor
    }
}