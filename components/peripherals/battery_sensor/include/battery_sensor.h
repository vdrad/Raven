#pragma once

#define BATTERY_3S 0
#define BATTERY_4S 1
#define CURRENT_BATTERY BATTERY_3S

// Define voltage thresholds based on the battery cell count
#if CURRENT_BATTERY == BATTERY_3S
    #define BATTERY_MONITORING_HIGH_VOLTAGE (4.0f * 3)
    #define BATTERY_MONITORING_LOW_VOLTAGE  (3.7f * 3)
#else
    #define BATTERY_MONITORING_HIGH_VOLTAGE (4.0f * 4)
    #define BATTERY_MONITORING_LOW_VOLTAGE  (3.7f * 4)
#endif

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