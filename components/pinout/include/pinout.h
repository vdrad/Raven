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
#define INFRARED_PIN            39
#define RGB_LED_PIN             4
#define BUZZER_PIN              38
/** @} */

/**
 * @name Analog & Power Sensors
 * @{
 */
#define BATTERY_METER_PIN       10
/** @} */

/**
 * @name SPI Line Sensor Array
 * @{
 */
// #define LINE_SENSOR_IO_PIN      48  // 🚨 CONFLICT: Also assigned to IMU_INT1_PIN
#define LINE_SENSOR_CS_PIN      34
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
#define LEFT_ENCODER_A_PIN      41
#define LEFT_ENCODER_B_PIN      42
#define RIGHT_ENCODER_A_PIN     2 
#define RIGHT_ENCODER_B_PIN     5
/** @} */

/**
 * @name I2C Bus
 * @{
 */
#define I2C_SDA_PIN             21
#define I2C_SCL_PIN             26
/** @} */

/**
 * @name Motor Drivers (DRV8874 / PWM)
 * @{
 */
// Left Motor
#define LEFT_MOTOR_DIR_PIN      36
#define LEFT_MOTOR_VEL_PIN      35
#define LEFT_MOTOR_CUR_PIN      1

// Right Motor
#define RIGHT_MOTOR_DIR_PIN     16   
#define RIGHT_MOTOR_VEL_PIN     17   
#define RIGHT_MOTOR_CUR_PIN     18   

// Suction/Fan Motor
#define FAN_MOTOR_DIR_PIN       8    
#define FAN_MOTOR_VEL_PIN       9    
#define FAN_MOTOR_CUR_PIN       7   
/** @} */