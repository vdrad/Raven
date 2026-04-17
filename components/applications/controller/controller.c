/**
 * @file controller.c
 * @brief Master control loop and PID tuning utilities for the cascaded architecture.
 * * This module handles the execution of the PID controllers and provides a 
 * high-precision hardware timer-based tuning tool for system identification 
 * and telemetry data gathering.
 */
#include "controller.h"

// Standard Libraries
#include <stdio.h>
#include <stdlib.h>

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Project Includes
#include "controller_pid.h"
#include "odometry.h"
#include "motor.h"
#include "raven_log.h"
#include "raven_comm.h" 
#include "esp_timer.h"
#include "battery_sensor.h"

#define TAG "CTR"

/* --- GLOBAL PID INSTANCES --- */
pid_context_t line_position_pid = {
    .kP                 = 0.026f,      
    .kI                 = 0.0f,           
    .kD                 = 0.00026f,               
    .bias               = 0.0f,             

    .ff_coef            = 0.0,
    .ff_bias            = 0.0f,

    .tm                 = 0.0f,

    .setpoint           = 0.0f,        
    .current_reading    = 0.0f,  

    .integral_sum       = 0.0f,
    .max_integral_sum   = 0.0f,

    .max_output         = 0.0f,      
    .min_output         = 0.0f,           
};

pid_context_t right_motor_pid = {
    .kP                 = 12.6420f,      
    .kI                 = 452.0095f,           
    .kD                 = 0.0f,               
    .bias               = 0.0f,             

    .ff_coef            = 0.91418f,
    .ff_bias            = 0.11392f,

    .tm                 = 0.22243f,

    .setpoint           = 0.5f,        
    .current_reading    = 0.0f,  

    .integral_sum       = 0.0f,
    .max_integral_sum   = 0.08f,

    .max_output         =  BATTERY_MONITORING_HIGH_VOLTAGE,      
    .min_output         = -BATTERY_MONITORING_HIGH_VOLTAGE,           
};

pid_context_t left_motor_pid = {
    .kP                 = 12.8641f,               
    .kI                 = 459.2188f,               
    .kD                 = 0.0f,               
    .bias               = 0.0f,             

    .ff_coef            = 0.90832f,
    .ff_bias            = 0.13873f,

    .tm                 = 0.22744f,

    .setpoint           = 0.5f,        
    .current_reading    = 0.0f,  

    .integral_sum       = 0.0f,
    .max_integral_sum   = 0.08f,

    .max_output         =  BATTERY_MONITORING_HIGH_VOLTAGE,      
    .min_output         = -BATTERY_MONITORING_HIGH_VOLTAGE,           
};

/**
 * @brief Initializes the controller module and spawns the command listener task.
 */
void controller_init(void) {
    xTaskCreate(controller_commands_task, "ctrl_cmd_task", 4096, NULL, 5, NULL);
    
    RAVEN_LOGI(TAG, "Initialized successfully.");
    
    // Broadcast initial PID states to the UI/App
    raven_comm_send_message(TAG, "LINE PID: kP=%.2f kI=%.2f kD=%.2f", 
                            line_position_pid.kP, line_position_pid.kI, line_position_pid.kD);
    raven_comm_send_message(TAG, "RM PID: kP=%.2f kI=%.2f kD=%.2f", 
                            right_motor_pid.kP, right_motor_pid.kI, right_motor_pid.kD);
    raven_comm_send_message(TAG, "LM PID: kP=%.2f kI=%.2f kD=%.2f", 
                            left_motor_pid.kP, left_motor_pid.kI, left_motor_pid.kD);
}

void controller_motors_run_tuning(void) {
    // 1. Update sensors once
    odometry_update();
    odometry_data_t odom = odometry_get_data();
    
    // 2. Assign readings to the respective PIDs
    left_motor_pid.current_reading = odom.velocity_left_m_s;
    right_motor_pid.current_reading = odom.velocity_right_m_s;
    
    // 3. Compute both
    pid_compute(&left_motor_pid);
    pid_compute(&right_motor_pid);
    
    // 4. Actuate
    motor_set_voltage(MOTOR_LEFT, left_motor_pid.output);
    motor_set_voltage(MOTOR_RIGHT, right_motor_pid.output);
}