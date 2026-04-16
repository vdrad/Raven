/**
 * @file pinout.h
 * @brief Centralized GPIO pin mapping for the Raven PRL-1 robot.
 *
 * This file defines all hardware connections between the ESP32 microcontroller 
 * and external peripherals. Modifying these values directly impacts hardware routing.
 */

#pragma once

/**
 * @name User Interface & Debug
 * @{
 */
#define RGB_LED_PIN             5
#define BUZZER_PIN              42
/** @} */

/**
 * @name Analog & Power Sensors
 * @{
 */
#define BATTERY_METER_PIN       9
/** @} */

/**
 * @name SPI Line Sensor Array
 * @{
 */
#define LINE_SENSOR_IO_PIN      35
#define LINE_SENSOR_CS_PIN      10
#define SPI_SDIN_PIN            11
#define SPI_SCLK_PIN            12
#define SPI_SDOUT_PIN           13
/** @} */

/**
 * @name IMU (ICM-45686)
 * @{
 */
#define IMU_INT1_PIN            48 
#define IMU_INT2_PIN            47
/** @} */

/**
 * @name Quadrature Encoders (PCNT)
 * @{
 */
#define LEFT_ENCODER_A_PIN      40
#define LEFT_ENCODER_B_PIN      41
#define RIGHT_ENCODER_A_PIN     6
#define RIGHT_ENCODER_B_PIN     7
#define SCT_ENCODER_A_PIN       2
#define SCT_ENCODER_B_PIN       1
/** @} */

/**
 * @name I2C Bus
 * @{
 */
#define I2C_SDA_PIN             14
#define I2C_SCL_PIN             21
/** @} */

/**
 * @name Motor Drivers (DRV8874 / PWM)
 * @{
 */
// Left Motor
#define LEFT_MOTOR_IN1_PIN      37
#define LEFT_MOTOR_IN2_PIN      36

// Right Motor
#define RIGHT_MOTOR_IN1_PIN     16
#define RIGHT_MOTOR_IN2_PIN     15

// Fan Motor
#define FAN_MOTOR_IN1_PIN       18    
#define FAN_MOTOR_IN2_PIN       17   

// Shortcut Motor
#define SCT_MOTOR_IN1_PIN       39    
#define SCT_MOTOR_IN2_PIN       38     
/** @} */