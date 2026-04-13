/**
 * @file AD7490.h
 * @brief ESP-IDF SPI Driver for the AD7490 16-channel, 12-bit ADC.
 *
 * This driver manages both the SPI bus initialization and the specific 
 * device communication protocols for the AD7490. It is designed for a 
 * single-device SPI bus topology.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

/* ========================================================================== */
/* MACROS & CONFIGURATIONS                                                    */
/* ========================================================================== */

#define NUMBER_OF_ACTIVE_CHANNELS   15

/* ========================================================================== */
/* PUBLIC API                                                                 */
/* ========================================================================== */

/**
 * @brief Initializes the SPI bus and the AD7490 device.
 * * Configures the ESP32 SPI master and performs the AD7490 power-up 
 * and sequence configuration routines. Logs the initialization status
 * and sends the configured SPI frequency via the communication link.
 */
void AD7490_init(void);

/**
 * @brief Reads all configured channels sequentially from the AD7490.
 *
 * @param array Pointer to an array of uint16_t where the results will be stored.
 * The array must have at least NUMBER_OF_ACTIVE_CHANNELS elements.
 */
void AD7490_read_all_channels(uint16_t array[NUMBER_OF_ACTIVE_CHANNELS]);

/**
 * @brief Executes a hardware validation routine.
 * * Forces a sequencer reset, reads all channels, and sends the raw SPI 
 * output (embedded channel IDs) via the communication link to prove 
 * hardware integrity and wire connections.
 */
void AD7490_peripheral_validation(void);

/**
 * @brief Executes a performance benchmark on the AD7490 SPI read function.
 * * Calculates average, minimum, and maximum execution times across 1000 samples
 * for reading ALL active channels, using the internal CPU cycle counter. Results
 * are logged to the serial monitor.
 */
void AD7490_benchmark_read(void);