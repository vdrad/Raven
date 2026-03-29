/**
 * @file motor.c
 * @brief Application-level Motor Manager using DRV8874 HAL.
 */

#include "motor.h"
#include "DRV8874.h"
#include "pinout.h"
#include "raven_comm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdbool.h>

#define TAG "MOT"
static bool initialized = false;

/* * Optimal constants for Coreless DC Motors.
 * Yields 800 ticks of resolution at 50 kHz.
 */
#define MOTOR_TIMER_RESOLUTION_HZ 40000000 // 40 MHz
#define MOTOR_FREQUENCY_HZ        50000    // 50 kHz
#define MOTOR_DUTY_TICK_MAX       (MOTOR_TIMER_RESOLUTION_HZ / MOTOR_FREQUENCY_HZ)

/**
 * @brief Internal structure mapping application context to hardware handles.
 */
typedef struct {
    const char *name;
    uint32_t pin_in1;
    uint32_t pin_in2;
    bdc_motor_handle_t handle;
} motor_instance_t;

/* * Array containing all robot motors. 
 * Indices are explicitly mapped to the motor_id_t enum.
 */
static motor_instance_t motors[MOTOR_MAX_COUNT] = {
    [MOTOR_LEFT]  = { .name = "LEFT",  .pin_in1 = LEFT_MOTOR_DIR_PIN,  .pin_in2 = LEFT_MOTOR_VEL_PIN,  .handle = NULL },
    [MOTOR_RIGHT] = { .name = "RIGHT", .pin_in1 = RIGHT_MOTOR_DIR_PIN, .pin_in2 = RIGHT_MOTOR_VEL_PIN, .handle = NULL }
};

void motor_init(void) {
    if (initialized) return;

    // Shared MCPWM timer configuration for all motors
    bdc_motor_mcpwm_config_t mcpwm_config = {
        .group_id = 0,
        .resolution_hz = MOTOR_TIMER_RESOLUTION_HZ,
    };

    // Iterate through the array to initialize each motor automatically
    for (int i = 0; i < MOTOR_MAX_COUNT; i++) {
        bdc_motor_config_t motor_config = {
            .pwma_gpio_num = motors[i].pin_in1,
            .pwmb_gpio_num = motors[i].pin_in2,
            .pwm_freq_hz = MOTOR_FREQUENCY_HZ,
        };

        DRV8874_init(&motor_config, &mcpwm_config, &motors[i].handle);
    }

    initialized = true;
    raven_comm_send_message(TAG, "Initialized successfully.");
}

void motor_set_speed_percent(motor_id_t id, float speed_percent) {
    if (!initialized || id >= MOTOR_MAX_COUNT) return;

    // Clamp percentage
    if (speed_percent > 100.0f) speed_percent = 100.0f;
    if (speed_percent < -100.0f) speed_percent = -100.0f;

    // Convert percentage to raw PWM ticks
    int32_t pwm_ticks = (int32_t)((speed_percent / 100.0f) * MOTOR_DUTY_TICK_MAX);

    DRV8874_set_pwm_value(motors[id].handle, pwm_ticks, MOTOR_DUTY_TICK_MAX);
}

void motor_brake(motor_id_t id) {
    if (!initialized || id >= MOTOR_MAX_COUNT) return;
    DRV8874_brake(motors[id].handle);
}

void motor_coast(motor_id_t id) {
    if (!initialized || id >= MOTOR_MAX_COUNT) return;
    DRV8874_coast(motors[id].handle);
}

void motor_peripheral_validation(void) {
    if (!initialized) {
        raven_comm_send_message(TAG, "Not initialized!");
        return;
    }

    // Sweep all motors without needing to temporarily map/unmap hardware
    for (int i = 0; i < MOTOR_MAX_COUNT; i++) {
        raven_comm_send_message(TAG, "--- Testing %s Motor ---", motors[i].name);

        raven_comm_send_message(TAG, "Forward 5%%...");
        motor_set_speed_percent((motor_id_t)i, 5.0f);
        vTaskDelay(pdMS_TO_TICKS(1500));

        raven_comm_send_message(TAG, "Braking...");
        motor_brake((motor_id_t)i);
        vTaskDelay(pdMS_TO_TICKS(1000));

        raven_comm_send_message(TAG, "Reverse 5%%...");
        motor_set_speed_percent((motor_id_t)i, -5.0f);
        vTaskDelay(pdMS_TO_TICKS(1500));

        raven_comm_send_message(TAG, "Final Brake.");
        motor_brake((motor_id_t)i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    raven_comm_send_message(TAG, "Validation Complete.");
}