/**
 * @file pid.h
 * @brief Header file for the Proportional-Integral-Derivative (PID) controller.
 */

#pragma once

#include <stdint.h>

/**
 * @brief Context structure for a PID controller instance.
 * * This structure holds all the tuning parameters, inputs, internal states, 
 * and limits required for computing the PID output. By instantiating this 
 * structure, multiple independent PID controllers can be run simultaneously.
 */
typedef struct {
    /* --- Tuning Parameters --- */
    float kP;               /**< Proportional gain */
    float kI;               /**< Integral gain */
    float kD;               /**< Derivative gain */
    float bias;             /**< Feedforward base value (added to the final output) */

    /* --- Feed Forward --- */
    float ff_coef;
    float ff_bias;

    /* --- System Data--- */
    float tm;

    /* --- Inputs --- */
    float setpoint;         /**< Target value the controller aims to reach */
    float current_reading;  /**< Current measured value from the system */

    /* --- Internal State (Memory) --- */
    float current_error;    /**< Current calculated error (setpoint - reading) */
    float previous_error;   /**< Error from the previous computation cycle */
    float delta_error;      /**< Rate of change of the error (derivative term) */
    
    float integral_sum;     /**< Accumulated error over time (integral term) */
    float max_integral_sum; /**< Maximum absolute value for the integral sum (Anti-windup) */
    
    int64_t last_run_time_us; /**< Timestamp of the last computation in microseconds */

    /* --- Limits and Output --- */
    float max_output;       /**< Maximum allowed positive output value */
    float min_output;       /**< Minimum allowed negative output value */
    float output;           /**< Final computed control variable (actuator command) */
} pid_context_t;

/**
 * @brief Computes the new output of the PID controller.
 * * @param[in,out] pid Pointer to the PID context structure. 
 */
void pid_compute(pid_context_t *pid);