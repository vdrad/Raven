/**
 * @file line_calibration.c
 * @brief Implementation of the line calibration and normalization logic.
 */
#include "line_calibration.h"

// Standard Includes
#include <string.h>
#include <stdbool.h>

// ESP-IDF & FreeRTOS
#include "esp_timer.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// NVS Includes
#include "nvs_flash.h"
#include "nvs.h"

// Project Includes
#include "raven_log.h"
#include "raven_comm.h"

#define TAG "CAL" 

/* ========================================================================== */
/* PRIVATE VARIABLES                                                          */
/* ========================================================================== */

static bool initialized = false;
static uint16_t min_raw_values[NUMBER_OF_ACTIVE_CHANNELS];
static uint16_t max_raw_values[NUMBER_OF_ACTIVE_CHANNELS];

/** @brief Mapping for internal iteration of line sensors. */
static const uint8_t LINE_INDEX_MAP[NUMBER_OF_LINE_SENSORS] = {
    LS0, LS1, LS2, LS3, LS4, LS5, LS6, LS7, LS8, LS9, LS10
};

/** @brief Mapping for internal iteration of marker sensors. */
static const uint8_t MARKER_INDEX_MAP[NUMBER_OF_MARKER_SENSORS] = {
    LM0, LM1, RM1, RM0
};

/* ========================================================================== */
/* PRIVATE FUNCTIONS                                                          */
/* ========================================================================== */

/**
 * @brief Normalizes raw value to [0, CALIBRATION_MAX_VALUE] based on LINE_COLOR.
 */
static inline uint16_t normalize_and_clamp(uint16_t raw, uint16_t min, uint16_t max) {
    uint16_t range = max - min;
    
    // Protection against division by zero (uncalibrated sensor)
    if (range == 0) return 0;

    // Linear mapping: min -> 0, max -> CALIBRATION_MAX_VALUE
    int32_t cal_value = ((int32_t)(raw - min) * CALIBRATION_MAX_VALUE) / range;

    // Clamping to [0, CALIBRATION_MAX_VALUE]
    if (cal_value < 0) cal_value = 0;
    if (cal_value > CALIBRATION_MAX_VALUE) cal_value = CALIBRATION_MAX_VALUE;

    /**
     * If WHITE_LINE: The whitest part (min ADC) must be the MAX_VALUE (100% Line).
     * If BLACK_LINE: The blackest part (max ADC) must be the MAX_VALUE.
     */
    if (LINE_COLOR == WHITE_LINE) return (uint16_t)(CALIBRATION_MAX_VALUE - cal_value);
    else return (uint16_t)cal_value;
}

static esp_err_t load_from_nvs(void) {
    nvs_handle_t my_handle;
    esp_err_t err;

    // Creates the 'line_calib' partition inside NVS.
    err = nvs_open("line_calib", NVS_READONLY, &my_handle);
    if (err != ESP_OK) {
        RAVEN_LOGW(TAG, "NVS open failed (Read): %s", esp_err_to_name(err));
        return err;
    }

    // Reads minimum-values array.
    size_t required_size = sizeof(min_raw_values);
    err = nvs_get_blob(my_handle, "min_vals", min_raw_values, &required_size);
    if (err != ESP_OK) {
        RAVEN_LOGW(TAG, "Failed to read min_vals from NVS: %s", esp_err_to_name(err));
        nvs_close(my_handle);
        return err;
    }

    // Reads maximum-values array.
    required_size = sizeof(max_raw_values);
    err = nvs_get_blob(my_handle, "max_vals", max_raw_values, &required_size);
    if (err != ESP_OK) {
        RAVEN_LOGW(TAG, "Failed to read max_vals from NVS: %s", esp_err_to_name(err));
        nvs_close(my_handle);
        return err;
    }

    nvs_close(my_handle);
    
    raven_comm_send_message(TAG, "Calibration data successfully LOADED from NVS.");
    return ESP_OK;
}

static esp_err_t save_to_nvs(void) {
    nvs_handle_t my_handle;
    esp_err_t err;

    // Opens 'line_calib' partition in write mode.
    err = nvs_open("line_calib", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        RAVEN_LOGE(TAG, "NVS open failed (Write): %s", esp_err_to_name(err));
        return err;
    }

    // Stores minimum-values array.
    err = nvs_set_blob(my_handle, "min_vals", min_raw_values, sizeof(min_raw_values));
    if (err != ESP_OK) {
        RAVEN_LOGE(TAG, "Failed to write min_vals: %s", esp_err_to_name(err));
        nvs_close(my_handle);
        return err;
    }

    // Stores maximum-values array.
    err = nvs_set_blob(my_handle, "max_vals", max_raw_values, sizeof(max_raw_values));
    if (err != ESP_OK) {
        RAVEN_LOGE(TAG, "Failed to write max_vals: %s", esp_err_to_name(err));
        nvs_close(my_handle);
        return err;
    }

    // Commits changes
    err = nvs_commit(my_handle);
    nvs_close(my_handle);

    if (err == ESP_OK)  raven_comm_send_message(TAG, "Calibration data successfully SAVED to NVS.");
    else RAVEN_LOGE(TAG, "Failed to commit NVS: %s", esp_err_to_name(err));
    
    return err;
}

static void send_calibration_report(void) {
    raven_comm_send_message(TAG, "Manual calibration complete. Captured extremes:");

    // Print MIN extremes
    raven_comm_send_message(TAG, 
        "MIN | L0-1: %4d %4d | S0-S10: %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d | R1-0: %4d %4d",
        min_raw_values[LM0], min_raw_values[LM1],
        min_raw_values[LS0], min_raw_values[LS1], min_raw_values[LS2], min_raw_values[LS3],
        min_raw_values[LS4], min_raw_values[LS5], min_raw_values[LS6], min_raw_values[LS7],
        min_raw_values[LS8], min_raw_values[LS9], min_raw_values[LS10],
        min_raw_values[RM1], min_raw_values[RM0]
    );

    // Print MAX extremes
    raven_comm_send_message(TAG, 
        "MAX | L0-1: %4d %4d | S0-S10: %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d | R1-0: %4d %4d",
        max_raw_values[LM0], max_raw_values[LM1],
        max_raw_values[LS0], max_raw_values[LS1], max_raw_values[LS2], max_raw_values[LS3],
        max_raw_values[LS4], max_raw_values[LS5], max_raw_values[LS6], max_raw_values[LS7],
        max_raw_values[LS8], max_raw_values[LS9], max_raw_values[LS10],
        max_raw_values[RM1], max_raw_values[RM0]
    );
}

/* ========================================================================== */
/* PUBLIC API                                                                 */
/* ========================================================================== */

void line_calibration_init(void) {
    if (initialized) return;
    
    for (uint8_t i = 0; i < NUMBER_OF_ACTIVE_CHANNELS; i++) {
        min_raw_values[i] = UINT16_MAX;
        max_raw_values[i] = 0;
    }

    initialized = true;
    RAVEN_LOGI(TAG, "Calibration module initialized.");
    raven_comm_send_message(TAG, "Normalized value range: 0-%d", CALIBRATION_MAX_VALUE);
}

void line_calibration_run(calibration_mode_t mode) {
    if (!initialized) {
        raven_comm_send_message(TAG, "Not initialized!");
        return;
    }

    if (mode == CALIB_MODE_LOAD_FROM_NVS) {
        if (load_from_nvs() == ESP_OK) {
            send_calibration_report();
            return;
        }
        RAVEN_LOGW(TAG, "Failed to load NVS. Falling back to manual calibration.");
        mode = CALIB_MODE_MANUAL_NO_SAVE;
    }

    raven_comm_send_message(TAG, "Starting manual calibration. Sweep the robot across the line for %d ms.", CALIBRATION_DURATION_MS);

    // Reset values for new calibration
    for (uint8_t i = 0; i < NUMBER_OF_ACTIVE_CHANNELS; i++) {
        min_raw_values[i] = UINT16_MAX;
        max_raw_values[i] = 0;
    }

    int64_t start_time = esp_timer_get_time();
    int64_t duration_us = (int64_t)CALIBRATION_DURATION_MS * 1000;
    uint16_t current_raw[NUMBER_OF_ACTIVE_CHANNELS];

    // Calibration Loop
    while ((esp_timer_get_time() - start_time) < duration_us) {
        line_reading_get_raw(current_raw);
        for (uint8_t i = 0; i < NUMBER_OF_ACTIVE_CHANNELS; i++) {
            if (current_raw[i] < min_raw_values[i]) min_raw_values[i] = current_raw[i];
            if (current_raw[i] > max_raw_values[i]) max_raw_values[i] = current_raw[i];
        }
        vTaskDelay(pdMS_TO_TICKS(20)); // Yield to FreeRTOS watchdog
    }

    send_calibration_report();

    if (mode == CALIB_MODE_MANUAL_SAVE_NVS) save_to_nvs();
}

void line_calibration_get_normalized(const uint16_t raw_readings[NUMBER_OF_ACTIVE_CHANNELS], 
                                     uint16_t calibrated_line[NUMBER_OF_LINE_SENSORS], 
                                     uint16_t calibrated_markers[NUMBER_OF_MARKER_SENSORS]) {
    if (!initialized) {
        raven_comm_send_message(TAG, "Not initialized!");
        return;
    }

    // Process Central Line Sensors
    for (uint8_t i = 0; i < NUMBER_OF_LINE_SENSORS; i++) {
        uint8_t idx = LINE_INDEX_MAP[i];
        calibrated_line[i] = normalize_and_clamp(raw_readings[idx], min_raw_values[idx], max_raw_values[idx]);
    }

    // Process Marker Sensors
    for (uint8_t i = 0; i < NUMBER_OF_MARKER_SENSORS; i++) {
        uint8_t idx = MARKER_INDEX_MAP[i];
        calibrated_markers[i] = normalize_and_clamp(raw_readings[idx], min_raw_values[idx], max_raw_values[idx]);
    }
}

void line_calibration_validate_output(void) {
    if (!initialized) {
        raven_comm_send_message(TAG, "Not initialized!");
        return;
    }

    uint16_t raw[NUMBER_OF_ACTIVE_CHANNELS];
    uint16_t line[NUMBER_OF_LINE_SENSORS];
    uint16_t markers[NUMBER_OF_MARKER_SENSORS];
    
    raven_comm_send_message(TAG, "=== STARTING CALIBRATED OUTPUT VALIDATION ===");

    for (int i = 0; i < 50; i++) {
        // Single synchronized snapshot of the sensors
        line_reading_get_raw(raw);
        
        // Pure math processing
        line_calibration_get_normalized(raw, line, markers);
        
        raven_comm_send_message(TAG, "LM: %4d %4d | LINE: %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d | RM: %4d %4d",
            markers[0], markers[1],
            line[0], line[1], line[2], line[3], line[4], line[5], line[6], line[7], line[8], line[9], line[10],
            markers[2], markers[3]);
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}