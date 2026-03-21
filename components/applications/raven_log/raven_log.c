/**
 * @file raven_log.c
 * @brief Custom logging initialization and configuration.
 * * This module serves as a centralized initialization point for the 
 * ESP-IDF logging system, allowing the project to easily control 
 * the log verbosity (e.g., Error, Warning, Info, Debug) across the application.
 */

#include "raven_log.h"
#include "esp_log.h"

/* --- Macros --- */
#define TAG "LOG"

/* --- Functions --- */

/**
 * @brief Initializes the custom logging system.
 * * Sets the logging level for the internal "LOG" tag based on the 
 * globally defined RAVEN_LOG_LEVEL macro. This ensures that only
 * messages of the specified severity (or higher) are printed to the console.
 */
void raven_log_init(void) {
    esp_log_level_set(TAG, RAVEN_LOG_LEVEL);
}