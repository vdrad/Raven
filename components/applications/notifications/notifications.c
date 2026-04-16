/**
 * @file notifications.c
 * @brief Implementation of the non-blocking notifications task.
 */

#include "notifications.h"

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Project Includes
#include "raven_log.h"

#define TAG "NOT"

// --- Hardware Mapping ---
// 0: Right Exhaust, 1-9: Right Wing (out to in), 10-18: Left Wing (in to out), 19: Left Exhaust
static const uint8_t led_map[20] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 18, 17, 16, 15, 14, 13, 12, 11, 10, 19 };

// --- Active Request State ---
static notification_config_t active_request = {
    .pattern = NOTIFY_PATTERN_OFF,
    .color = {0,0,0},
    .base_tone_hz = 0,
    .speed_ms = 100,
    .repetitions = 0,
    .freeze_at_end = false
};

static portMUX_TYPE notify_spinlock = portMUX_INITIALIZER_UNLOCKED;

// --- UX Profile Definitions ---
// const notification_config_t NOTIFY_PROFILE_OFF = {
//     NOTIFY_PATTERN_OFF, 
//     {0,0,0}, 
//     0, 
//     100, 
//     NOTIFY_INFINITE, 
//     false
// };

// const notification_config_t NOTIFY_PROFILE_WAKEUP = {
//     NOTIFY_PATTERN_SWEEP_TO_EDGES, 
//     {200, 200, 200}, 
//     NOTE_C4, 
//     30, 
//     1, 
//     true // White, freezes on edges
// };

// const notification_config_t NOTIFY_PROFILE_WAIT_CONN = {
//     NOTIFY_PATTERN_BREATHER, 
//     {252, 3, 232}, 
//     0, 
//     60, 
//     NOTIFY_INFINITE, 
//     false // Purple
// };

// const notification_config_t NOTIFY_PROFILE_ARMED = {
//     NOTIFY_PATTERN_EXHAUSTS_SPOOL, 
//     {255, 0, 6}, 
//     NOTE_C4, 
//     40, 
//     1, 
//     true // Scarlet, freezes exhausts ON
// };

// const notification_config_t NOTIFY_PROFILE_CALIBRATING = {
//     NOTIFY_PATTERN_SWEEP_TO_CENTER, 
//     {0, 255, 255}, 
//     NOTE_E5, 
//     40, 
//     NOTIFY_INFINITE, 
//     false // Cyan sweep
// };

// const notification_config_t NOTIFY_PROFILE_FAILSAFE = {
//     NOTIFY_PATTERN_BLINK_EDGES, 
//     {255, 0, 0}, 
//     NOTE_A4, 
//     150, 
//     10, 
//     false // Red blink 10 times, then off
// };

// --- Helper Functions ---
static rgb_color_t color_scale(rgb_color_t base, float factor) {
    rgb_color_t scaled;
    scaled.r = (uint8_t)(base.r * factor);
    scaled.g = (uint8_t)(base.g * factor);
    scaled.b = (uint8_t)(base.b * factor);
    return scaled;
}

// --- Background Task ---
static void notifications_task(void *pvParameters) {
    notification_config_t req;

    for (;;) {
        // 1. Thread-safe snapshot of the current request
        taskENTER_CRITICAL(&notify_spinlock);
        req = active_request;
        taskEXIT_CRITICAL(&notify_spinlock);

        // 2. Repetition logic & Freezing
        if (req.repetitions == 0 && req.pattern != NOTIFY_PATTERN_OFF && req.pattern != NOTIFY_PATTERN_HOLD) {
            if (req.freeze_at_end) {
                // Change internal state to HOLD. We don't clear the LEDs.
                notification_play(&(notification_config_t){.pattern = NOTIFY_PATTERN_HOLD});
            } else {
                // Default fallback is turning off
                notification_play(&NOTIFY_PROFILE_OFF);
            }
            continue; 
        }

        // 3. Play one frame/loop of the pattern
        switch (req.pattern) {
            
            case NOTIFY_PATTERN_OFF:
                rgb_led_clear();
                rgb_led_show();
                vTaskDelay(pdMS_TO_TICKS(100)); 
                break;

            case NOTIFY_PATTERN_HOLD:
                // Do absolutely nothing, just sleep and preserve the LEDs
                vTaskDelay(pdMS_TO_TICKS(100));
                break;

            case NOTIFY_PATTERN_BREATHER: {
                for (int i = 1; i <= 10; i++) {
                    rgb_color_t current_color = color_scale(req.color, i / 10.0f);
                    for (int j = 1; j <= 18; j++) rgb_led_set_color(led_map[j], current_color);
                    rgb_led_show();
                    vTaskDelay(pdMS_TO_TICKS(req.speed_ms));
                }
                for (int i = 9; i >= 1; i--) {
                    rgb_color_t current_color = color_scale(req.color, i / 10.0f);
                    for (int j = 1; j <= 18; j++) rgb_led_set_color(led_map[j], current_color);
                    rgb_led_show();
                    vTaskDelay(pdMS_TO_TICKS(req.speed_ms));
                }
                break;
            }

            case NOTIFY_PATTERN_SWEEP_TO_EDGES: {
                rgb_led_clear();
                for (int step = 0; step < 9; step++) {
                    rgb_led_set_color(led_map[9 - step], req.color);
                    rgb_led_set_color(led_map[10 + step], req.color);
                    rgb_led_show();

                    if (req.base_tone_hz > 0) {
                        if (step == 0) buzzer_play(req.base_tone_hz, 40);             // Root
                        if (step == 3) buzzer_play((req.base_tone_hz * 5) / 4, 40);   // Major 3rd
                        if (step == 6) buzzer_play((req.base_tone_hz * 3) / 2, 40);   // Perfect 5th
                    }
                    vTaskDelay(pdMS_TO_TICKS(req.speed_ms));
                }
                vTaskDelay(pdMS_TO_TICKS(100)); 
                if(!req.freeze_at_end) { rgb_led_clear(); rgb_led_show(); }
                vTaskDelay(pdMS_TO_TICKS(100));
                break;
            }

            case NOTIFY_PATTERN_SWEEP_TO_CENTER: {
                rgb_led_clear();
                // Sweep from edges (1 and 18) to center (9 and 10)
                for (int step = 0; step < 9; step++) {
                    rgb_led_set_color(led_map[1 + step], req.color);
                    rgb_led_set_color(led_map[18 - step], req.color);
                    rgb_led_show();

                    // Descending arpeggio (sounding like "locking in")
                    if (req.base_tone_hz > 0) {
                        if (step == 0) buzzer_play((req.base_tone_hz * 3) / 2, 40); // Perfect 5th
                        if (step == 3) buzzer_play((req.base_tone_hz * 5) / 4, 40); // Major 3rd
                        if (step == 6) buzzer_play(req.base_tone_hz, 40);           // Root
                    }
                    vTaskDelay(pdMS_TO_TICKS(req.speed_ms));
                }
                vTaskDelay(pdMS_TO_TICKS(100));
                if(!req.freeze_at_end) { rgb_led_clear(); rgb_led_show(); }
                vTaskDelay(pdMS_TO_TICKS(100));
                break;
            }

            case NOTIFY_PATTERN_EXHAUSTS_SPOOL: {
                uint32_t current_freq = req.base_tone_hz;
                for (int i = 0; i < 5; i++) {
                    rgb_led_set_color(led_map[0], req.color);
                    rgb_led_set_color(led_map[19], req.color);
                    rgb_led_show();
                    if (current_freq > 0) buzzer_play(current_freq, 30);
                    
                    vTaskDelay(pdMS_TO_TICKS(req.speed_ms));
                    rgb_led_clear();
                    rgb_led_show();
                    vTaskDelay(pdMS_TO_TICKS(req.speed_ms));
                    
                    current_freq += 100;
                }
                // Solid blast phase
                rgb_led_set_color(led_map[0], req.color);
                rgb_led_set_color(led_map[19], req.color);
                rgb_led_show();
                if (req.base_tone_hz > 0) buzzer_play(current_freq * 2, 500);
                vTaskDelay(pdMS_TO_TICKS(500));
                
                // If not freezing, clear it. (If freezing, it stays on when pattern switches to HOLD).
                if(!req.freeze_at_end) { rgb_led_clear(); rgb_led_show(); }
                break;
            }

            case NOTIFY_PATTERN_BLINK_EDGES: {
                // Outermost LEDs only (Index 1 and 18)
                
                // State A: Both Outermost ON
                rgb_led_clear();
                rgb_led_set_color(led_map[1], req.color); 
                rgb_led_set_color(led_map[18], req.color); 
                rgb_led_show();
                if (req.base_tone_hz > 0) buzzer_play(req.base_tone_hz * 2, 100); 
                vTaskDelay(pdMS_TO_TICKS(req.speed_ms));

                // State B: Both Outermost OFF
                rgb_led_clear();
                rgb_led_show();
                if (req.base_tone_hz > 0) buzzer_play(req.base_tone_hz, 100); 
                vTaskDelay(pdMS_TO_TICKS(req.speed_ms));
                break;
            }

            default:
                vTaskDelay(pdMS_TO_TICKS(100));
                break;
        }

        // 4. Update repetitions (if not infinite)
        if (req.repetitions > 0) {
            taskENTER_CRITICAL(&notify_spinlock);
            if (active_request.pattern == req.pattern) { 
                active_request.repetitions--;
            }
            taskEXIT_CRITICAL(&notify_spinlock);
        }
    }
}

// --- Public API ---
void notifications_init(void) {
    rgb_led_init();
    buzzer_init();
    xTaskCreate(notifications_task, "notif_task", 2048, NULL, 2, NULL); 
}

void notification_play(const notification_config_t *config) {
    if (config == NULL) return;
    taskENTER_CRITICAL(&notify_spinlock);
    active_request = *config; // Copy the entire struct
    taskEXIT_CRITICAL(&notify_spinlock);
}

// void notifications_validation(void) {
//     RAVEN_LOGI(TAG, "Starting Notifications UX Test...");
//     vTaskDelay(pdMS_TO_TICKS(500)); 

//     RAVEN_LOGI(TAG, "1. Wake Up Profile (Center to Edges + Freeze)");
//     notification_play(&NOTIFY_PROFILE_WAKEUP);
//     vTaskDelay(pdMS_TO_TICKS(2000)); 

//     RAVEN_LOGI(TAG, "2. Armed Profile (Exhausts + Freeze)");
//     notification_play(&NOTIFY_PROFILE_ARMED);
//     vTaskDelay(pdMS_TO_TICKS(3000));

//     RAVEN_LOGI(TAG, "3. Calibrating Profile (Edges to Center)");
//     notification_play(&NOTIFY_PROFILE_CALIBRATING);
//     vTaskDelay(pdMS_TO_TICKS(4000)); 
    
//     RAVEN_LOGI(TAG, "4. Failsafe Profile (Blink outers)");
//     notification_play(&NOTIFY_PROFILE_FAILSAFE);
// }