/**
 * @file controller_pid.c
 * @brief Proportional-Integral-Derivative (PID) controller implementation.
 */

#include "controller_pid.h"
#include <stdint.h>
#include <esp_timer.h>

/**
 * @brief Computes the new output of the PID controller.
 * * This function calculates the Proportional, Integral, and Derivative responses 
 * based on the exact time elapsed (dt) since the last computation. It features 
 * integral anti-windup protection and output clamping to prevent actuator saturation.
 * * @note A minimum delta time of 1ms (0.001s) is enforced to prevent division by zero 
 * and mitigate derivative noise. If polled faster than 1kHz, the function 
 * will safely bypass the calculation and accumulate time for the next cycle.
 * * @param[in,out] pid Pointer to the PID context structure. 
 * The user must ensure that setpoint and current_reading 
 * are updated before calling this function.
 */
void pid_compute(pid_context_t *pid) {
    int64_t now_us = esp_timer_get_time();

    // Calculate real delta time in seconds
    float dt_s = (float)(now_us - pid->last_run_time_us) / 1000000.0f;
    
    // Throttle execution to a maximum of 1kHz to ensure stability
    if (dt_s <= 0.001f) {
        // Initialize timer on the very first run
        if (pid->last_run_time_us == 0) pid->last_run_time_us = now_us; // deprecated by race_manager.c _start()?
        return; 
    }

    // --- Error Calculation ---
    pid->current_error = pid->setpoint - pid->current_reading;
    
    // Calculate derivative (rate of change of error)
    pid->delta_error = (pid->current_error - pid->previous_error) / dt_s;
    
    // Integrate error over time
    pid->integral_sum += (pid->current_error * dt_s);

    // --- Anti-Windup Protection ---
    if (pid->integral_sum > pid->max_integral_sum) pid->integral_sum = pid->max_integral_sum;
    else if (pid->integral_sum < -pid->max_integral_sum) pid->integral_sum = -pid->max_integral_sum;

    // --- Compute Output ---
    pid->output =  pid->bias +
                  (pid->kP * pid->current_error) +
                  (pid->kD * pid->delta_error)   +
                  (pid->kI * pid->integral_sum);

    // --- Compute FF Output -- 
    if (pid->setpoint >= 0) pid->output += pid->ff_coef * pid->setpoint + pid->ff_bias;
    else                    pid->output += pid->ff_coef * pid->setpoint - pid->ff_bias;

    // --- Output Saturation (Clamping) ---
    if (pid->output > pid->max_output) pid->output = pid->max_output;
    else if (pid->output < pid->min_output) pid->output = pid->min_output;

    // --- Update State for Next Cycle ---
    pid->previous_error = pid->current_error;
    pid->last_run_time_us = now_us;
}