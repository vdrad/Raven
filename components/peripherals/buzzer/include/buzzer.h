/**
 * @file buzzer.h
 * @brief FreeRTOS-driven PWM Buzzer module.
 *
 * Provides a non-blocking interface to play single tones or complex melodies 
 * on a passive buzzer using the ESP32 LEDC peripheral and a FreeRTOS queue.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Structure representing a single musical note or tone.
 */
typedef struct {
    uint32_t freq_hz;     /**< Frequency of the note in Hertz (0 for silence/rest) */
    uint32_t duration_ms; /**< Duration of the note in milliseconds */
} buzzer_note_t;

/**
 * @brief Initializes the LEDC peripheral, timer, and the background FreeRTOS task.
 * * Configures the hardware timer and spawns a dedicated task pinned to Core 1
 * to process incoming notes from the queue.
 */
void buzzer_init(void);

/**
 * @brief Immediately stops the buzzer and forces the PWM duty cycle to 0.
 */
void buzzer_stop(void);

/**
 * @brief Enqueues a tone to be played by the background task.
 * * @param freq_hz Frequency of the tone in Hertz (Use 0 for a rest).
 * @param duration_ms Duration to play the tone in milliseconds.
 */
void buzzer_play(uint32_t freq_hz, uint32_t duration_ms);

/**
 * @brief Executes a hardware validation routine.
 * * Plays a predefined melody to test the buzzer's acoustic output and 
 * non-blocking queue behavior.
 */
void buzzer_peripheral_validation(void);