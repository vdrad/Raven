/**
 * @file DRV8874.c
 * @brief Native Hardware Abstraction Layer for DRV8874 Motor Driver.
 */

#include "DRV8874.h"

// Standard C Library
#include <stdlib.h>

// ESP-IDF Drivers
#include "driver/mcpwm_prelude.h"

// Project Includes
#include "raven_log.h"
#include "raven_comm.h"

#define TAG "DRV"

/**
 * @brief Internal context structure holding all native MCPWM handles.
 */
struct drv8874_context_t {
    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t  oper;
    mcpwm_cmpr_handle_t  cmpr_in1;
    mcpwm_cmpr_handle_t  cmpr_in2;
    mcpwm_gen_handle_t   gen_in1;
    mcpwm_gen_handle_t   gen_in2;
    uint32_t             max_ticks; /**< Cached period ticks for calculations */
};

/* ========================================================================== */
/* PUBLIC API IMPLEMENTATIONS                                                 */
/* ========================================================================== */

void DRV8874_init(const drv8874_config_t *config, drv8874_handle_t *ret_motor) {
    if (config == NULL || ret_motor == NULL) return;

    drv8874_handle_t motor = calloc(1, sizeof(struct drv8874_context_t));
    if (!motor) {
        raven_comm_send_message(TAG, "ERROR: Memory allocation failed.");
        return;
    }

    // Calculate maximum ticks based on resolution and frequency
    motor->max_ticks = config->resolution_hz / config->pwm_freq_hz;

    // 1. Create Timer
    mcpwm_timer_config_t timer_config = {
        .group_id = config->group_id,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = config->resolution_hz,
        .period_ticks = motor->max_ticks,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &motor->timer));

    // 2. Create Operator and connect to Timer
    mcpwm_operator_config_t oper_config = {
        .group_id = config->group_id,
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_config, &motor->oper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(motor->oper, motor->timer));

    // 3. Create Comparators for IN1 and IN2
    mcpwm_comparator_config_t cmpr_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(motor->oper, &cmpr_config, &motor->cmpr_in1));
    ESP_ERROR_CHECK(mcpwm_new_comparator(motor->oper, &cmpr_config, &motor->cmpr_in2));

    // 4. Create Generators for IN1 and IN2
    mcpwm_generator_config_t gen1_config = { .gen_gpio_num = config->pin_in1 };
    ESP_ERROR_CHECK(mcpwm_new_generator(motor->oper, &gen1_config, &motor->gen_in1));
    
    mcpwm_generator_config_t gen2_config = { .gen_gpio_num = config->pin_in2 };
    ESP_ERROR_CHECK(mcpwm_new_generator(motor->oper, &gen2_config, &motor->gen_in2));

    // 5. Configure Standard PWM Actions (High on Zero, Low on Compare)
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(motor->gen_in1, 
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(motor->gen_in1, 
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, motor->cmpr_in1, MCPWM_GEN_ACTION_LOW)));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(motor->gen_in2, 
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(motor->gen_in2, 
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, motor->cmpr_in2, MCPWM_GEN_ACTION_LOW)));

    // 6. Enable and Start the Timer
    ESP_ERROR_CHECK(mcpwm_timer_enable(motor->timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(motor->timer, MCPWM_TIMER_START_NO_STOP));

    // Ensure motor starts completely stopped (Brake)
    DRV8874_brake(motor);

    *ret_motor = motor;

    // Telemetry and Logging
    RAVEN_LOGI(TAG, "Motor initialized on pins %lu and %lu", config->pin_in1, config->pin_in2);
    raven_comm_send_message(TAG, "DRV8874 Setup -> Freq: %lu Hz | Max Ticks: %lu", config->pwm_freq_hz, motor->max_ticks);
}

void DRV8874_set_pwm_value(drv8874_handle_t motor, int32_t pwm_ticks) {
    if (motor == NULL) return;

    // Clamp limits safely
    if (pwm_ticks > (int32_t)motor->max_ticks) pwm_ticks = motor->max_ticks;
    if (pwm_ticks < -(int32_t)motor->max_ticks) pwm_ticks = -motor->max_ticks;

    uint32_t speed = abs(pwm_ticks);

    if (pwm_ticks > 0) {
        // Forward (Slow Decay): IN1 is solid HIGH, IN2 receives inverted PWM
        mcpwm_comparator_set_compare_value(motor->cmpr_in1, motor->max_ticks);
        mcpwm_comparator_set_compare_value(motor->cmpr_in2, motor->max_ticks - speed);
    } 
    else if (pwm_ticks < 0) {
        // Reverse (Slow Decay): IN1 receives inverted PWM, IN2 is solid HIGH
        mcpwm_comparator_set_compare_value(motor->cmpr_in1, motor->max_ticks - speed);
        mcpwm_comparator_set_compare_value(motor->cmpr_in2, motor->max_ticks);
    } 
    else {
        DRV8874_brake(motor); 
    }
}

void DRV8874_brake(drv8874_handle_t motor) {
    if (motor == NULL) return;
    
    // Slow Decay / Brake: Both inputs HIGH
    mcpwm_comparator_set_compare_value(motor->cmpr_in1, motor->max_ticks);
    mcpwm_comparator_set_compare_value(motor->cmpr_in2, motor->max_ticks);
}

void DRV8874_coast(drv8874_handle_t motor) {
    if (motor == NULL) return;
    
    // Fast Decay / Coast: Both inputs LOW
    mcpwm_comparator_set_compare_value(motor->cmpr_in1, 0);
    mcpwm_comparator_set_compare_value(motor->cmpr_in2, 0);
}