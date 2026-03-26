/**
 * @file encoder.c
 * @brief PCNT-based Quadrature Encoder Driver for ESP32.
 * * This module configures and handles high-speed incremental encoders
 * utilizing the ESP32 Pulse Count (PCNT) peripheral with X4 decoding logic.
 */

#include "encoder.h"
#include "driver/pulse_cnt.h"
#include "pinout.h"
#include "raven_log.h"
#include "raven_comm.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"

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
static encoder_t right_encoder = { .gpio_a = RIGHT_ENCODER_A_PIN, .gpio_b = RIGHT_ENCODER_B_PIN };

/**< Flag to track the initialization state of the encoder module */
static bool initialized = false;

/**
 * @brief Internal helper to initialize a specific PCNT unit instance.
 * * Configures the PCNT hardware for X4 quadrature decoding, applies a 
 * nanosecond glitch filter, and starts the counter in accumulation mode.
 * * @param[in,out] enc Pointer to the encoder instance structure to initialize.
 */
static void encoder_init_instance(encoder_t *enc) {
    /* Basic unit configuration: allow accumulation beyond limits */
    pcnt_unit_config_t unit_config = {
        .high_limit = 10000,
        .low_limit = -10000,
        .flags.accum_count = 1, 
    };

    /* Glitch filter configuration for high-speed signals */
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 250,
    };

    /* Allocate the PCNT unit */
    if (pcnt_new_unit(&unit_config, &enc->unit_handle) != ESP_OK) {
        raven_comm_send_message(TAG, "ERROR: Failed to create PCNT unit");
        return;
    }

    /* Apply the noise filter */
    pcnt_unit_set_glitch_filter(enc->unit_handle, &filter_config);

    /* Channel A Setup: edges on A, level checked on B */
    pcnt_chan_config_t chan_a_config = { .edge_gpio_num = enc->gpio_a, .level_gpio_num = enc->gpio_b };
    pcnt_channel_handle_t chan_a = NULL;
    pcnt_new_channel(enc->unit_handle, &chan_a_config, &chan_a);

    /* Channel B Setup: edges on B, level checked on A */
    pcnt_chan_config_t chan_b_config = { .edge_gpio_num = enc->gpio_b, .level_gpio_num = enc->gpio_a };
    pcnt_channel_handle_t chan_b = NULL;
    pcnt_new_channel(enc->unit_handle, &chan_b_config, &chan_b);

    /* X4 Quadrature Logic Actions: quadruples resolution by evaluating all edges */
    pcnt_channel_set_edge_action(chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    
    pcnt_channel_set_edge_action(chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    /* Enable, clear, and start the hardware counter */
    pcnt_unit_enable(enc->unit_handle);
    pcnt_unit_clear_count(enc->unit_handle);
    pcnt_unit_start(enc->unit_handle);
}

/**
 * @brief Initializes the hardware peripherals for all robot encoders.
 * * Should be called once during system boot before any other encoder functions.
 */
void encoder_init(void) {
    if (initialized) return;

    encoder_init_instance(&left_encoder);
    encoder_init_instance(&right_encoder);
    
    raven_comm_send_message(TAG, "Initialized successfully.");
    initialized = true;
}

/**
 * @brief Retrieves the current accumulated pulse count from the specified encoder.
 * * @param[in] side The encoder side to read (ENCODER_LEFT or ENCODER_RIGHT).
 * @param[out] count Pointer to an integer where the read count will be stored.
 */
void encoder_get_count(encoder_side_t side, int *count) {
    if (!initialized) {
        raven_comm_send_message(TAG, "Not initialized!");
        return;
    }

    if (count == NULL) {
        return;
    }

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

/**
 * @brief Resets the accumulated pulse counts for both encoders to zero.
 */
void encoder_reset_count(void) {
    if (!initialized) {
        raven_comm_send_message(TAG, "Not initialized!");
        return;
    }

    pcnt_unit_clear_count(left_encoder.unit_handle);
    pcnt_unit_clear_count(right_encoder.unit_handle);
}

/**
 * @brief Blocks the calling task to periodically print encoder counts for debugging.
 * * @note This is a blocking diagnostic function and should only be used during development.
 */
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