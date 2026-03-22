#pragma once

// ENUMS
typedef enum {
    PERIPHERAL_ALL = 0,
    PERIPHERAL_RGB_LED,
    PERIPHERAL_BUZZER,
    PERIPHERAL_BATTERY_SENSOR
} peripheral_to_validate_t;

// FUNCTIONS
void peripheral_validation(peripheral_to_validate_t peripheral);