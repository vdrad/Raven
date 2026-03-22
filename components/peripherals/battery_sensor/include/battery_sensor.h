#pragma once

/**
 * @brief Represents the current operational state of the battery.
 */
typedef enum {
    BATTERY_HIGH,
    BATTERY_MEDIUM,
    BATTERY_LOW,
    POWERED_BY_USB,
    BATTERY_STATUS_MAX
} battery_status_t;

void battery_sensor_init(void);
void battery_sensor_peripheral_validation(void);
float battery_sensor_get_voltage(void);
battery_status_t battery_sensor_get_status(void);