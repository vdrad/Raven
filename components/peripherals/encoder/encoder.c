/**
 * @file encoder.c
 * @brief PCNT-based Quadrature Encoder Driver for ESP32.
 * * This module configures and handles high-speed incremental encoders
 * utilizing the ESP32 Pulse Count (PCNT) peripheral with X4 decoding logic.
 */

#include "encoder.h"

// ESP-IDF Drivers
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Project
#include "pinout.h"
#include "raven_log.h"
#include "raven_comm.h"

#define TAG "ENC"

/**
 * @brief Structure representing a hardware encoder instance.
 */
typedef struct {
    int gpio_a;                     /**< GPIO pin number for channel A */
    int gpio_b;                     /**< GPIO pin number for channel B */
    pcnt_unit_handle_t unit_handle; /**< PCNT unit handle assigned to this encoder */
} encoder_t;

/* Static instances: Private to this file to ensure encapsulation */
static encoder_t left_encoder = { .gpio_a = LEFT_ENCODER_A_PIN, .gpio_b = LEFT_ENCODER_B_PIN };
static encoder_t right_encoder = { .gpio_a = RIGHT_ENCODER_B_PIN, .gpio_b = RIGHT_ENCODER_A_PIN };

/**< Flag to track the initialization state of the encoder module */
static bool initialized = false;

/* ========================================================================== */
/* PRIVATE FUNCTION IMPLEMENTATIONS                                           */
/* ========================================================================== */

/**
 * @brief Internal helper to initialize a specific PCNT unit instance.
 * * Configures the PCNT hardware for X4 quadrature decoding, applies a 
 * nanosecond glitch filter, and starts the counter in accumulation mode.
 * * @param[in,out] enc Pointer to the encoder instance structure to initialize.
 */
static void encoder_init_instance(encoder_t *enc) {
    pcnt_unit_config_t unit_config = {
        .high_limit = 32767,
        .low_limit = -32767,
        .flags.accum_count = 1, 
    };

    /* Glitch filter configuration for high-speed signals */
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 250,
    };

    /* Allocate the PCNT unit */
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &enc->unit_handle));

    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(enc->unit_handle, 32767));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(enc->unit_handle, -32767));

    /* Apply the noise filter */
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(enc->unit_handle, &filter_config));

    /* Channel A Setup: edges on A, level checked on B */
    pcnt_chan_config_t chan_a_config = { .edge_gpio_num = enc->gpio_a, .level_gpio_num = enc->gpio_b };
    pcnt_channel_handle_t chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(enc->unit_handle, &chan_a_config, &chan_a));

    /* Channel B Setup: edges on B, level checked on A */
    pcnt_chan_config_t chan_b_config = { .edge_gpio_num = enc->gpio_b, .level_gpio_num = enc->gpio_a };
    pcnt_channel_handle_t chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(enc->unit_handle, &chan_b_config, &chan_b));

    /* X4 Quadrature Logic Actions: quadruples resolution by evaluating all edges */
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    /* Enable, clear, and start the hardware counter */
    ESP_ERROR_CHECK(pcnt_unit_enable(enc->unit_handle));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(enc->unit_handle));
    ESP_ERROR_CHECK(pcnt_unit_start(enc->unit_handle));
}

/* ========================================================================== */
/* PUBLIC API IMPLEMENTATIONS                                                 */
/* ========================================================================== */

void encoder_init(void) {
    if (initialized) return;

    encoder_init_instance(&left_encoder);
    encoder_init_instance(&right_encoder);
    
    RAVEN_LOGI(TAG, "Initialized successfully.");
    raven_comm_send_message(TAG, "Glitch filter value: %dns", 250);
    
    initialized = true;
}

void encoder_get_count(encoder_side_t side, int *count) {
    if (!initialized) {
        raven_comm_send_message(TAG, "Not initialized!");
        return;
    }

    // Enforced single-line statement format
    if (count == NULL) return;

    pcnt_unit_handle_t target_handle = NULL;

    switch (side) {
        case ENCODER_LEFT:
            target_handle = left_encoder.unit_handle;
            break;
        case ENCODER_RIGHT:
            target_handle = right_encoder.unit_handle;
            break;
        default:
            return;
    }

    pcnt_unit_get_count(target_handle, count);
}

void encoder_reset_count(void) {
    if (!initialized) {
        raven_comm_send_message(TAG, "Not initialized!");
        return;
    }

    pcnt_unit_clear_count(left_encoder.unit_handle);
    pcnt_unit_clear_count(right_encoder.unit_handle);
}

void encoder_peripheral_validation(void) {
    if (!initialized) {
        raven_comm_send_message(TAG, "Not initialized!");
        return;
    }

    int left_count = 0;
    int right_count = 0;

    for (uint8_t i = 0; i < 20; i++) {
        encoder_get_count(ENCODER_LEFT, &left_count);
        encoder_get_count(ENCODER_RIGHT, &right_count);
        raven_comm_send_message(TAG, "LENC: %d | RENC: %d", left_count, right_count);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}