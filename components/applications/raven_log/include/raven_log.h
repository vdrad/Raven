#pragma once

// INCLUDES
#include "esp_log.h"
#include <stdbool.h>

// MACROS
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

// FUNCTIONS
void raven_log_init(void);