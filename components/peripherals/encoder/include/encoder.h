#pragma once

/**
 * @file encoder.h
 * @brief Public API for the PCNT-based Quadrature Encoder module.
 * * This header defines the interfaces for initializing, reading, and 
 * resetting the high-speed encoders attached to the robot's motors.
 */

/**
 * @brief Identifiers for the robot's hardware encoders.
 */
typedef enum {
    ENCODER_LEFT,  /**< Refers to the encoder on the left motor */
    ENCODER_RIGHT  /**< Refers to the encoder on the right motor */
} encoder_side_t;

/**
 * @brief Initializes the hardware peripherals for all robot encoders.
 * * Configures the PCNT units, channels, glitch filters, and X4 logic
 * for both the left and right encoders. Must be called before reading.
 */
void encoder_init(void);

/**
 * @brief Retrieves the current accumulated pulse count from a specific encoder.
 * * @param[in] side The target encoder to read (ENCODER_LEFT or ENCODER_RIGHT).
 * @param[out] count Pointer to an integer where the pulse count will be written.
 */
void encoder_get_count(encoder_side_t side, int *count);

/**
 * @brief Resets the accumulated pulse counts for both encoders to zero.
 */
void encoder_reset_count(void);

/**
 * @brief Blocking diagnostic task to validate encoder hardware.
 * * @note This function runs a fixed loop, printing the current counts of both 
 * encoders every 500ms for debugging purposes. Do not use in production control loops.
 */
void encoder_peripheral_validation(void);