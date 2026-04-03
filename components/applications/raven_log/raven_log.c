/**
 * @file raven_log.c
 * @brief Custom logging initialization and configuration.
 * * This module serves as a centralized initialization point for the 
 * ESP-IDF logging system, allowing the project to easily control 
 * the log verbosity (e.g., Error, Warning, Info, Debug) across the application.
 */

#include "raven_log.h"
#include "esp_log.h"

#define TAG "LOG"

/* ========================================================================== */
/* PUBLIC API IMPLEMENTATIONS                                                 */
/* ========================================================================== */

void raven_log_init(void) {
    esp_log_level_set(TAG, RAVEN_LOG_LEVEL);
}