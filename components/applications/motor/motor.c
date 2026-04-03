/**
 * @file motor.c
 * @brief Application-level Motor Manager using DRV8874 HAL.
 *
 * This module abstracts the hardware layer and provides a physics-based API
 * (voltage control) to ensure consistent PID behavior regardless of battery drain.
 */

#include "motor.h"

// Standard C Libraries
#include <stdbool.h>
#include <math.h> // Required for round()

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Project Includes (All correctly mapped as PRIV_REQUIRES)
#include "pinout.h"
#include "raven_log.h"
#include "raven_comm.h"
#include "DRV8874.h"
#include "battery_sensor.h"

#define TAG "MOT"

/* ========================================================================== */
/* MACROS & CONFIGURATIONS                                                    */
/* ========================================================================== */

static bool initialized = false;

/* * Optimal constants for Coreless DC Motors.
 * Yields 800 ticks of resolution at 50 kHz.
 */
#define MOTOR_TIMER_RESOLUTION_HZ 40000000 // 40 MHz
#define MOTOR_FREQUENCY_HZ        50000    // 50 kHz
#define MOTOR_DUTY_TICK_MAX       (MOTOR_TIMER_RESOLUTION_HZ / MOTOR_FREQUENCY_HZ)

#define MOTOR_MAX_VOLTAGE_ALLOWED BATTERY_MONITORING_HIGH_VOLTAGE

/**
 * @brief Internal structure mapping application context to hardware handles.
 */
typedef struct {
    const char *name;
    uint32_t pin_in1;
    uint32_t pin_in2;
    drv8874_handle_t handle; // Updated to use the native opaque pointer
} motor_instance_t;

/* * Array containing all robot motors. 
 * Indices are explicitly mapped to the motor_id_t enum.
 */
static motor_instance_t motors[MOTOR_MAX_COUNT] = {
    [MOTOR_LEFT]  = { .name = "LEFT",  .pin_in1 = LEFT_MOTOR_DIR_PIN,  .pin_in2 = LEFT_MOTOR_VEL_PIN,  .handle = NULL },
    [MOTOR_RIGHT] = { .name = "RIGHT", .pin_in1 = RIGHT_MOTOR_DIR_PIN, .pin_in2 = RIGHT_MOTOR_VEL_PIN, .handle = NULL }
};

/* ========================================================================== */
/* PUBLIC API IMPLEMENTATIONS                                                 */
/* ========================================================================== */

/**
 * @brief Initializes all configured motors and their native MCPWM hardware.
 */
void motor_init(void) {
    if (initialized) return;

    // Iterate through the array to initialize each motor automatically using the native HAL
    for (int i = 0; i < MOTOR_MAX_COUNT; i++) {
        drv8874_config_t config = {
            .pin_in1 = motors[i].pin_in1,
            .pin_in2 = motors[i].pin_in2,
            .pwm_freq_hz = MOTOR_FREQUENCY_HZ,
            .resolution_hz = MOTOR_TIMER_RESOLUTION_HZ,
            .group_id = 0
        };

        DRV8874_init(&config, &motors[i].handle);
    }

    initialized = true;
    RAVEN_LOGI(TAG, "Initialized successfully.");
    raven_comm_send_message(TAG, "PWM Frequency: %d kHz", MOTOR_FREQUENCY_HZ/1000);
}

/**
 * @brief Sets the motor speed based on a target voltage to compensate for battery drop.
 * * @param[in] id The target motor.
 * @param[in] voltage The desired voltage to apply to the motor.
 */
void motor_set_voltage(motor_id_t id, float voltage) {
    if (!initialized || id >= MOTOR_MAX_COUNT) return;

    // Clamp the requested voltage to the maximum allowed design limits
    if (voltage >  MOTOR_MAX_VOLTAGE_ALLOWED) voltage =  MOTOR_MAX_VOLTAGE_ALLOWED;
    if (voltage < -MOTOR_MAX_VOLTAGE_ALLOWED) voltage = -MOTOR_MAX_VOLTAGE_ALLOWED;

    float battery_voltage = battery_sensor_get_voltage();

    // Safety constraint: Prevent division by zero if battery reading fails or is critically low
    if (battery_voltage <= 0.1f) {
        raven_comm_send_message(TAG, "WARNING: Battery voltage critically low or zero. Halting motor.");
        DRV8874_brake(motors[id].handle);
        return;
    }

    // Calculate raw PWM ticks needed to achieve the target voltage
    int32_t pwm_ticks = (int32_t)round((voltage / battery_voltage) * (float)MOTOR_DUTY_TICK_MAX);

    // RAVEN_LOGI(TAG, "Requested: %.2fV | Battery: %.2fV | PWM: %ld", voltage, battery_voltage, pwm_ticks);

    // Hardware abstraction layer handles negative ticks automatically
    DRV8874_set_pwm_value(motors[id].handle, pwm_ticks); // Updated to remove max_ticks parameter
}

/**
 * @brief Forces a specific motor to stop immediately using active braking (Slow Decay).
 * * @param[in] id The target motor.
 */
void motor_brake(motor_id_t id) {
    if (!initialized || id >= MOTOR_MAX_COUNT) return;
    DRV8874_brake(motors[id].handle);
}

/**
 * @brief Cuts power to a specific motor, allowing it to spin freely (Fast Decay).
 * * @param[in] id The target motor.
 */
void motor_coast(motor_id_t id) {
    if (!initialized || id >= MOTOR_MAX_COUNT) return;
    DRV8874_coast(motors[id].handle);
}

/**
 * @brief Blocking diagnostic task to validate all configured motors.
 */
void motor_peripheral_validation(void) {
    if (!initialized) {
        raven_comm_send_message(TAG, "Not initialized!");
        return;
    }

    float test_voltage = 1.0f; // Use a safe target voltage for testing

    // Sweep all motors without needing to temporarily map/unmap hardware
    for (int i = 0; i < MOTOR_MAX_COUNT; i++) {
        raven_comm_send_message(TAG, "--- Testing %s Motor ---", motors[i].name);

        raven_comm_send_message(TAG, "Forward %.1f Volts...", test_voltage);
        motor_set_voltage((motor_id_t)i, test_voltage);
        vTaskDelay(pdMS_TO_TICKS(1500));

        raven_comm_send_message(TAG, "Braking...");
        motor_brake((motor_id_t)i);
        vTaskDelay(pdMS_TO_TICKS(1000));

        raven_comm_send_message(TAG, "Reverse %.1f Volts...", -test_voltage);
        motor_set_voltage((motor_id_t)i, -test_voltage);
        vTaskDelay(pdMS_TO_TICKS(1500));

        raven_comm_send_message(TAG, "Final Brake.");
        motor_brake((motor_id_t)i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    raven_comm_send_message(TAG, "Validation Complete.");
}