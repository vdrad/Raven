/**
 * @file raven_log.h
 * @brief Centralized logging macros and configuration for the Raven PRL-1 robot.
 *
 * Wraps the ESP-IDF logging system to provide a centralized toggle switch 
 * (RAVEN_LOG_ENABLED) and standardized formatting across all project modules.
 */

#pragma once

#include <stdbool.h>
#include "esp_log.h"

#define RAVEN_LOG_ENABLED 1 

#if RAVEN_LOG_ENABLED
    #define RAVEN_LOG_LEVEL ESP_LOG_INFO

    #define RAVEN_LOGI(tag, format, ...) \
        do { \
            ESP_LOGI(tag, format, ##__VA_ARGS__); \
        } while(0)
        
    #define RAVEN_LOGE(tag, format, ...) \
        do { \
            ESP_LOGE(tag, format, ##__VA_ARGS__); \
        } while(0)

    #define RAVEN_LOGD(tag, format, ...) \
        do { \
            ESP_LOGD(tag, format, ##__VA_ARGS__); \
        } while(0)

    #define RAVEN_LOGW(tag, format, ...) \
        do { \
            ESP_LOGW(tag, format, ##__VA_ARGS__); \
        } while(0)

#else
    #define RAVEN_LOG_LEVEL ESP_LOG_NONE

    #define RAVEN_LOGI(tag, format, ...) do {} while(0)
    #define RAVEN_LOGE(tag, format, ...) do {} while(0)
    #define RAVEN_LOGD(tag, format, ...) do {} while(0)
    #define RAVEN_LOGW(tag, format, ...) do {} while(0)
#endif

/**
 * @brief Initializes the custom logging system.
 * * Applies the global log level configuration to the core log tag.
 */
void raven_log_init(void);