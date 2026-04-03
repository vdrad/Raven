/**
 * @file peripheral_validation.h
 * @brief Hardware and peripheral validation suite API.
 *
 * Exposes the enumeration and function to trigger interactive
 * hardware validation tests for the robot's subsystems.
 */

#pragma once

/**
 * @brief Enumeration of available peripherals to validate.
 */
typedef enum {
    PERIPHERAL_ALL = 0,         /**< Executes the entire testing suite sequentially */
    PERIPHERAL_RGB_LED,         /**< Tests the WS2812 LED strip */
    PERIPHERAL_BUZZER,          /**< Tests the PWM audio buzzer */
    PERIPHERAL_BATTERY_SENSOR,  /**< Tests the ADC battery monitor */
    PERIPHERAL_AD7490,          /**< Tests the SPI line sensor ADC */
    PERIPHERAL_ICM45686,        /**< Tests the I2C IMU */
    PERIPHERAL_ENCODER,         /**< Tests the PCNT wheel encoders */
    PERIPHERAL_DRV8874,         /**< Tests the MCPWM motor drivers */
} peripheral_to_validate_t;

/**
 * @brief Executes the validation test for a specific peripheral.
 * * Initiates an interactive testing sequence over the communication interface,
 * prompting the user to manually verify physical outputs or sensor readings.
 * * @param peripheral The specific peripheral to test, or PERIPHERAL_ALL for a full system check.
 */
void peripheral_validation(peripheral_to_validate_t peripheral);