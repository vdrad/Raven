/**
 * @file ICM45686.h
 * @brief Driver for the ICM-45686 6-Axis IMU.
 *
 * This module provides the initialization, configuration, calibration, 
 * and data acquisition routines for the ICM-45686 sensor using the ESP32 I2C peripheral.
 * The external API is designed to return `void` as per project architecture,
 * emitting errors via the logging/communication system.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Accelerometer Full Scale Range (FSR) options.
 */
typedef enum {
    ICM45686_ACCEL_FSR_32G = 32,
    ICM45686_ACCEL_FSR_16G = 16,
    ICM45686_ACCEL_FSR_8G  = 8,
    ICM45686_ACCEL_FSR_4G  = 4,
    ICM45686_ACCEL_FSR_2G  = 2
} icm45686_accel_fsr_t;

/**
 * @brief Gyroscope Full Scale Range (FSR) options.
 */
typedef enum {
    ICM45686_GYRO_FSR_4000DPS = 4000,
    ICM45686_GYRO_FSR_2000DPS = 2000,
    ICM45686_GYRO_FSR_1000DPS = 1000,
    ICM45686_GYRO_FSR_500DPS  = 500,
    ICM45686_GYRO_FSR_250DPS  = 250,
    ICM45686_GYRO_FSR_125DPS  = 125,
    ICM45686_GYRO_FSR_62DPS   = 62,
    ICM45686_GYRO_FSR_31DPS   = 31,
    ICM45686_GYRO_FSR_15DPS   = 15
} icm45686_gyro_fsr_t;

/**
 * @brief Output Data Rate (ODR) options for Accel and Gyro.
 */
typedef enum {
    ICM45686_ODR_6400HZ = 6400,
    ICM45686_ODR_3200HZ = 3200,
    ICM45686_ODR_1600HZ = 1600,
    ICM45686_ODR_800HZ  = 800,
    ICM45686_ODR_400HZ  = 400,
    ICM45686_ODR_200HZ  = 200,
    ICM45686_ODR_100HZ  = 100,
    ICM45686_ODR_50HZ   = 50,
    ICM45686_ODR_25HZ   = 25,
    ICM45686_ODR_12HZ   = 12,
    ICM45686_ODR_6HZ    = 6,
    ICM45686_ODR_3HZ    = 3,
    ICM45686_ODR_1HZ    = 1
} icm45686_odr_t;

/**
 * @brief Raw sensor data structure containing 16-bit integer values.
 */
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temp;
} icm45686_raw_data_t;

/**
 * @brief Processed sensor data structure containing physical float values.
 */
typedef struct {
    float accel_x;  /**< Accelerometer X-axis in g */
    float accel_y;  /**< Accelerometer Y-axis in g */
    float accel_z;  /**< Accelerometer Z-axis in g */
    float gyro_x;   /**< Gyroscope X-axis in degrees per second (dps) */
    float gyro_y;   /**< Gyroscope Y-axis in degrees per second (dps) */
    float gyro_z;   /**< Gyroscope Z-axis in degrees per second (dps) */
    float temp;     /**< Temperature in degrees Celsius */
} icm45686_data_t;

/**
 * @brief Initializes the I2C bus, configures the ICM-45686, and runs calibration.
 * @note If initialization fails, an error is logged, but execution continues.
 * The user must decide whether to reboot the system or proceed without IMU.
 */
void ICM45686_init(void);

/**
 * @brief Fetches and processes the latest data from the IMU.
 * @param[out] out_data Pointer to the struct where processed data will be stored.
 */
void ICM45686_get_data(icm45686_data_t *out_data);

/**
 * @brief Reads data and prints it via the communication system for validation.
 */
void ICM45686_peripheral_validation(void);

/**
 * @brief Runs a 1000-sample burst read benchmark and prints the timing results.
 */
void ICM45686_benchmark_read(void);