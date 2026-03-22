#pragma once

// ENUMS
typedef enum {
    PERIPHERAL_RGB_LED,
    PERIPHERAL_BUZZER
} peripheral_to_validate_t;

// FUNCTIONS
void peripheral_validation(peripheral_to_validate_t peripheral);