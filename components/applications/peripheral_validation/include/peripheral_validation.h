#pragma once

// ENUMS
typedef enum {
    PERIPHERAL_ALL = 0,
    PERIPHERAL_RGB_LED,
    PERIPHERAL_BUZZER,
    PERIPHERAL_BATTERY_SENSOR,
    PERIPHERAL_AD7490,
    PERIPHERAL_ICM45686,
    PERIPHERAL_ENCODER,
    PERIPHERAL_DRV8874,
} peripheral_to_validate_t;

// FUNCTIONS
void peripheral_validation(peripheral_to_validate_t peripheral);