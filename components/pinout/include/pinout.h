#pragma once

// DEBUG
#define INFRARED_PIN            39
#define RGB_LED_PIN             4
#define BUZZER_PIN              38

// SENSORS
#define BATTERY_METER_PIN       10

#define LINE_SENSOR_IO_PIN      48
#define LINE_SENSOR_CS_PIN      34
#define SPI_SDIN_PIN            11
#define SPI_SCLK_PIN            12
#define SPI_SDOUT_PIN           13

#define IMU_INT1_PIN            48
#define IMU_INT2_PIN            47

// I2C
#define I2C_SDA_PIN             21
#define I2C_SCL_PIN             26

// DRIVERS
#define LEFT_MOTOR_DIR_PIN      36
#define LEFT_MOTOR_VEL_PIN      35
#define LEFT_MOTOR_CUR_PIN      1

#define RIGHT_MOTOR_DIR_PIN     17  // Momentarily switch with FAN pins 
#define RIGHT_MOTOR_VEL_PIN     16  // Momentarily switch with FAN pins 
#define RIGHT_MOTOR_CUR_PIN     18  // Momentarily switch with FAN pins 

#define FAN_MOTOR_DIR_PIN       8   // Momentarily switch with FAN pins 
#define FAN_MOTOR_VEL_PIN       9   // Momentarily switch with FAN pins 
#define FAN_MOTOR_CUR_PIN       7   // Momentarily switch with FAN pins