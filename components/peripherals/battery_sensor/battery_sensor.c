/**
 * @file battery_sensor.c
 * @brief Battery voltage monitoring and status evaluation.
 *
 * This module configures the ESP32 ADC in One-Shot mode to read the battery 
 * voltage. It runs a dedicated FreeRTOS task to periodically sample the ADC, 
 * apply moving average filtering, and evaluate the battery state of charge 
 * (High, Medium, Low, or USB powered).
 */

#include "battery_sensor.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ESP-IDF
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"

// Project
#include "pinout.h"
#include "raven_comm.h"

/* ========================================================================== */
/* MACROS & CONSTANTS                                                         */
/* ========================================================================== */

#define TAG "BAT"

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
    
#define BATTERY_MONITORING_USB_VOLTAGE  3.0f

#define BATTERY_SENSOR_SAMPLES_PER_READING 2
#define BATTERY_SENSOR_VOLTAGE_POINT       12.27f
#define BATTERY_SENSOR_ADC_POINT           1769.0f


/* ========================================================================== */
/* PRIVATE VARIABLES                                                          */
/* ========================================================================== */

static bool initialized = false;

/** @brief Printable string mapping for battery status. */
static const char *battery_status_text[] = {
    "HIGH", 
    "MEDIUM", 
    "LOW", 
    "USB"
};

// ADC Handlers
static adc_oneshot_unit_handle_t adc_handle;
static adc_channel_t adc_channel;

// Sensor State
static int raw_reading = 0;
static float voltage_reading = 0.0f;
static battery_status_t battery_status = POWERED_BY_USB;

/* ========================================================================== */
/* FREE RTOS TASKS                                                            */
/* ========================================================================== */

/**
 * @brief Task to continuously monitor battery voltage.
 * * Takes multiple ADC samples, calculates the average, converts it to a real 
 * voltage using the calibration points, and updates the global battery status.
 *
 * The voltage is calculated using a linear mapping:
 * $V_{actual} = \frac{ADC_{raw} \times V_{point}}{ADC_{point}}$
 * * @param pvParameters Standard FreeRTOS task parameters (unused).
 */
static void battery_sensor_task(void *pvParameters) {
    for (;;) {
        uint32_t sum_of_readings = 0;

        // 1. Take multiple samples for moving average
        for (uint8_t i = 0; i < BATTERY_SENSOR_SAMPLES_PER_READING; i++) {
            int current_reading;
            ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, adc_channel, &current_reading));
            sum_of_readings += current_reading;
        }

        // 2. Calculate average and actual voltage (Moved outside the loop)
        raw_reading = (int)round((float)sum_of_readings / BATTERY_SENSOR_SAMPLES_PER_READING);
        
        float raw_voltage = ((float)raw_reading * BATTERY_SENSOR_VOLTAGE_POINT) / BATTERY_SENSOR_ADC_POINT;
        voltage_reading = roundf(10.0f * raw_voltage) / 10.0f; // Round to 1 decimal place

        // 3. Evaluate battery state thresholds
        if (voltage_reading >= BATTERY_MONITORING_HIGH_VOLTAGE)     battery_status = BATTERY_HIGH;
        else if (voltage_reading >= BATTERY_MONITORING_LOW_VOLTAGE) battery_status = BATTERY_MEDIUM;
        else if (voltage_reading >= BATTERY_MONITORING_USB_VOLTAGE) battery_status = BATTERY_LOW;
        else                                                        battery_status = POWERED_BY_USB;

        // Wait before taking the next reading
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

/* ========================================================================== */
/* PUBLIC API                                                                 */
/* ========================================================================== */

/**
 * @brief Initializes the battery sensor ADC and spawns the monitoring task.
 */
void battery_sensor_init(void) {
    if (initialized) return;

    // Local configuration structs (frees up memory after init)
    adc_oneshot_unit_init_cfg_t adc_init_config = {
        .unit_id = ADC_UNIT_1
    };
    
    adc_oneshot_chan_cfg_t adc_config = {
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12
    };

    // Initialize ADC unit
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_init_config, &adc_handle));

    // Map the hardware pin to the ADC channel and configure it
    ESP_ERROR_CHECK(adc_oneshot_io_to_channel(BATTERY_METER_PIN, &adc_init_config.unit_id, &adc_channel));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, adc_channel, &adc_config));

    // Spawn the background reading task on Core 1
    xTaskCreatePinnedToCore(battery_sensor_task, "battery_sensor", 2048, NULL, 5, NULL, 1);

    raven_comm_send_message(TAG, "Initialized successfully.");
    initialized = true;
}

/**
 * @brief Gets the latest calculated battery voltage.
 * @return float Current battery voltage in volts (V).
 */
float battery_sensor_get_voltage(void) {
    return voltage_reading;
}

/**
 * @brief Gets the current evaluated operational status of the battery.
 * @return battery_status_t Current state enum (HIGH, MEDIUM, LOW, USB).
 */
battery_status_t battery_sensor_get_status(void) {
    return battery_status;
}

/**
 * @brief Prints current ADC and voltage readings to the communication interface.
 */
void battery_sensor_peripheral_validation(void) {
    if (!initialized) {
        raven_comm_send_message(TAG, "Not initialized!");
        return; 
    }  

    const char *status_str = "UNKNOWN";
    if (battery_status >= 0 && battery_status < BATTERY_STATUS_MAX) {
        status_str = battery_status_text[battery_status];
    }

    raven_comm_send_message(TAG, "ADC: %d | Voltage: %.2fV | Status: %s", 
                            raw_reading, voltage_reading, status_str);
                            
    // Delay to yield context if called inside a loop
    vTaskDelay(pdMS_TO_TICKS(2000));
}