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

// ESP-IDF SPI Master
#include "driver/spi_master.h"
#include "driver/gpio.h"


/* ========================================================================== */
/* MACROS & CONFIGURATIONS                                                    */
/* ========================================================================== */

#define AD7490_SPI_FREQUENCY_HZ     (10 * 1000 * 1000) /**< 10 MHz SPI Clock */

// AD7490 CONTROL REGISTER (CR) BIT VALUES
#define AD7490_CR_WRITE_VALUE       1
#define AD7490_CR_SEQ_VALUE         1
#define AD7490_CR_PM_VALUE          3
#define AD7490_CR_SHADOW_VALUE      1
#define AD7490_CR_WEAK_VALUE        1
#define AD7490_CR_RANGE_VALUE       1
#define AD7490_CR_CODING_VALUE      1

// TODO: Change it for NUMBER_OF_LINE_SENSOR when this application is developed
#define NUMBER_OF_ACTIVE_CHANNELS   15

/* ========================================================================== */
/* PUBLIC API                                                                 */
/* ========================================================================== */

/**
 * @brief Initializes the SPI bus and the AD7490 device.
 *
 */
void AD7490_init();

/**
 * @brief Reads all configured channels sequentially from the AD7490.
 *
 * @param array Pointer to an array of uint16_t where the results will be stored.
 * The array must have at least NUMBER_OF_ACTIVE_CHANNELS elements.
 */
void AD7490_read_all_channels(uint16_t array[NUMBER_OF_ACTIVE_CHANNELS]);

/**
 * @brief Executes a hardware validation routine, reading all channels and 
 * sending the raw output via the communication link.
 */
void AD7490_peripheral_validation(bool print_values);

void AD7490_benchmark_read(void);