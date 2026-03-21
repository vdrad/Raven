#include "raven_log.h"
#include "esp_log.h"

#define TAG "LOG"

void raven_log_init(void) {
    esp_log_level_set(TAG, RAVEN_LOG_LEVEL);
}
