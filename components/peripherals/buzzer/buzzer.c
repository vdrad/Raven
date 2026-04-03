/**
 * @file buzzer.c
 * @brief Implementation of the non-blocking PWM buzzer.
 */

#include "buzzer.h"
#include "notes.h"

// ESP-IDF Hardware Drivers
#include "driver/ledc.h"

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// Project Includes
#include "pinout.h"
#include "raven_log.h"
#include "raven_comm.h"

#define TAG "BZR"

static bool initialized = false;

/* --- LEDC Hardware Configuration --- */
#define BUZZER_LEDC_TIMER     LEDC_TIMER_0
#define BUZZER_LEDC_CHANNEL   LEDC_CHANNEL_0
#define BUZZER_LEDC_MODE      LEDC_LOW_SPEED_MODE
#define BUZZER_DUTY_RES       LEDC_TIMER_10_BIT
#define BUZZER_DUTY_VALUE     512  // 50% duty cycle for 10-bit resolution (1024 / 2)

/* --- Task & Queue Handles --- */
static QueueHandle_t buzzer_queue = NULL;
static TaskHandle_t buzzer_task_handle = NULL;

/* ========================================================================== */
/* PRIVATE FUNCTION IMPLEMENTATIONS                                           */
/* ========================================================================== */

static void buzzer_play_tone(uint32_t freq_hz) {
    if (freq_hz == 0) {
        buzzer_stop();
        return;
    }
    ESP_ERROR_CHECK(ledc_set_freq(BUZZER_LEDC_MODE, BUZZER_LEDC_TIMER, freq_hz));
    ESP_ERROR_CHECK(ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, BUZZER_DUTY_VALUE));
    ESP_ERROR_CHECK(ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL));
}

static void buzzer_task(void *pvParameters) {
    buzzer_note_t current_note;
    
    for (;;) {
        if (xQueueReceive(buzzer_queue, &current_note, portMAX_DELAY) == pdTRUE) {
            buzzer_play_tone(current_note.freq_hz);
            
            if (current_note.duration_ms > 0) {
                TickType_t delay = pdMS_TO_TICKS(current_note.duration_ms);
                vTaskDelay(delay > 0 ? delay : 1);
            }
            
            buzzer_stop();
            
            // Tiny pause between notes to ensure acoustic articulation
            TickType_t pause = pdMS_TO_TICKS(12);
            vTaskDelay(pause > 0 ? pause : 1); 
        }
    }
}

static bool buzzer_is_playing(void) {
    if (!initialized || buzzer_queue == NULL) {
        raven_comm_send_message(TAG, "Not initialized!");
        return false; 
    }  
    
    // Enforced single-line formatting
    if (uxQueueMessagesWaiting(buzzer_queue) > 0) return true;
    if (ledc_get_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL) > 0) return true;

    return false;
}

/* ========================================================================== */
/* PUBLIC API IMPLEMENTATIONS                                                 */
/* ========================================================================== */

void buzzer_init(void) {
    if (initialized) return;  

    ledc_timer_config_t timer_conf = {
        .speed_mode = BUZZER_LEDC_MODE, 
        .duty_resolution = BUZZER_DUTY_RES,
        .timer_num = BUZZER_LEDC_TIMER, 
        .freq_hz = 4000, 
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    ledc_channel_config_t ch_conf = {
        .gpio_num = BUZZER_PIN, 
        .speed_mode = BUZZER_LEDC_MODE,
        .channel = BUZZER_LEDC_CHANNEL, 
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = BUZZER_LEDC_TIMER, 
        .duty = 0, 
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_conf));

    buzzer_queue = xQueueCreate(100, sizeof(buzzer_note_t));
    xTaskCreatePinnedToCore(buzzer_task, "buzzer_task", 2048, NULL, 5, &buzzer_task_handle, 1);

    RAVEN_LOGI(TAG, "Initialized successfully.");
    
    // Transmit hardware config telemetry to the user
    raven_comm_send_message(TAG, "LEDC Resolution: 10-bit | Max Queue: 100 notes");
    
    initialized = true;
}

void buzzer_stop(void) {
    if (!initialized) {
        raven_comm_send_message(TAG, "Not initialized!");
        return; 
    }  

    ESP_ERROR_CHECK(ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, 0));
    ESP_ERROR_CHECK(ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL));
}

void buzzer_play(uint32_t freq_hz, uint32_t duration_ms) {
    if (!initialized || buzzer_queue == NULL) {
        raven_comm_send_message(TAG, "Not initialized!");
        return; 
    }  

    buzzer_note_t note = { .freq_hz = freq_hz, .duration_ms = duration_ms };
    
    // Enforced single-line formatting
    if (xQueueSend(buzzer_queue, &note, 0) != pdTRUE) raven_comm_send_message(TAG, "Queue full! Dropping note.");
}

void buzzer_peripheral_validation(void) {
    if (!initialized || buzzer_queue == NULL) {
        raven_comm_send_message(TAG, "Not initialized!");
        return; 
    }  

    if (!buzzer_is_playing()) {
        int melody[] = {
            NOTE_D5, NOTE_C5, NOTE_B4, NOTE_A4, NOTE_REST,
            NOTE_D5, NOTE_C5, NOTE_B4, NOTE_A4, NOTE_REST,

            // Pensaba que contigo iba a envejecer
            NOTE_A4, NOTE_A4, NOTE_A4, NOTE_D5, NOTE_C5, NOTE_B4, NOTE_A4, NOTE_D5, NOTE_C5, NOTE_B4, NOTE_A4, NOTE_C5, NOTE_REST,  

            // En otra vida, en otro mundo podrá ser
            NOTE_D5, NOTE_D5, NOTE_D5, NOTE_E5, NOTE_D5, NOTE_C5, NOTE_B4, NOTE_E5, NOTE_D5, NOTE_C5, NOTE_B4, NOTE_C5, NOTE_D5, NOTE_REST,

            // En esta solo queda irme un día
            NOTE_E5, NOTE_E5, NOTE_F5, NOTE_E5, NOTE_F5, NOTE_E5, NOTE_F5, NOTE_E5, NOTE_G5, NOTE_E5, NOTE_D5, NOTE_REST,

            // Y solamente verte en el atardecer
            NOTE_A4, NOTE_A4, NOTE_A4, NOTE_D5, NOTE_C5, NOTE_B4, NOTE_A4, NOTE_D5, NOTE_C5, NOTE_B4, NOTE_C5, NOTE_F4, NOTE_G4, NOTE_A4,
        };

        float note_durations[] = {
            0.25, 0.25, 0.25, 0.75, 0.5,
            0.25, 0.25, 0.25, 0.75, 0.5,

            0.25, 0.5,  0.5, 0.5, 0.5, 0.5,  0.5, 0.5, 0.5,  0.5,  0.5,  1.25, 0.50,
            0.25, 0.25, 0.5, 0.5, 0.5, 0.5,  0.5, 0.5, 0.5,  0.5,  0.5,  0.75, 1.25, 1.25,
            0.25, 0.6,  0.5, 0.5, 0.5, 0.5,  0.5, 0.5, 0.75, 0.25, 1.25, 1.0,
            0.25, 0.25, 0.5, 0.5, 0.5, 0.5,  0.5, 0.5, 0.5,  0.5,  0.5,  1.25, 1.00, 0.85,  
        };

        uint16_t num_notes = sizeof(melody) / sizeof(melody[0]);
        xQueueReset(buzzer_queue);
    
        for (int i = 0; i < num_notes; i++) {
            uint32_t duration = (uint32_t)(675.0f * note_durations[i]);
            buzzer_play(melody[i], duration);
        }
        
        raven_comm_send_message(TAG, "Playing validation melody...");
    } else {
        raven_comm_send_message(TAG, "Buzzer is already playing.");
    }
}