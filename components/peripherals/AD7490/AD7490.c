/**
 * @file AD7490.c
 * @brief ESP-IDF SPI Driver implementation for the AD7490 ADC.
 */

#include "AD7490.h"
#include <string.h>
#include <stdio.h>

// ESP-IDF SPI Master & GPIO (Moved from header to prevent dependency leakage)
#include "driver/spi_master.h"
#include "driver/gpio.h"

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// CPU includes for benchmark
#include "esp_cpu.h"
#include "esp_rom_sys.h"

// Project includes
#include "pinout.h"
#include "raven_log.h"
#include "raven_comm.h"

#define TAG "ADC"

/* ========================================================================== */
/* MACROS & CONFIGURATIONS                                                    */
/* ========================================================================== */

/** @brief SPI Clock frequency set to 10 MHz. */
#define AD7490_SPI_FREQUENCY_HZ     (10 * 1000 * 1000) 

// AD7490 CONTROL REGISTER (CR) BIT VALUES
#define AD7490_CR_WRITE_VALUE       1
#define AD7490_CR_SEQ_VALUE         1
#define AD7490_CR_PM_VALUE          3
#define AD7490_CR_SHADOW_VALUE      1
#define AD7490_CR_WEAK_VALUE        1
#define AD7490_CR_RANGE_VALUE       1
#define AD7490_CR_CODING_VALUE      1

/* ========================================================================== */
/* PRIVATE VARIABLES                                                          */
/* ========================================================================== */

static spi_device_handle_t spi_handle = NULL;
static bool initialized = false;

/* ========================================================================== */
/* PRIVATE FUNCTION DECLARATIONS                                              */
/* ========================================================================== */

static uint16_t generate_command(uint8_t write, uint8_t seq, uint8_t shadow, uint16_t addr);
static uint16_t write_to_register(uint16_t command);
static void powerup_routine(void);
static void read_from_sequence(uint8_t *channel, uint16_t *data);

/* ========================================================================== */
/* PRIVATE FUNCTION IMPLEMENTATIONS                                           */
/* ========================================================================== */

/**
 * @brief Assembles the 16-bit Control Register (CR) command for the AD7490.
 * * @param write  Write control bit.
 * @param seq    Sequencer control bit.
 * @param shadow Shadow register control bit.
 * @param addr   Channel address to configure.
 * @return uint16_t Fully assembled 16-bit command payload.
 */
static uint16_t generate_command(uint8_t write, uint8_t seq, uint8_t shadow, uint16_t addr) {
    return ((
        (write                  << 11)  |
        (seq                    << 10)  |
        ((uint16_t)addr         << 6)   |
        (AD7490_CR_PM_VALUE     << 4)   |
        (shadow                 << 3)   |
        (AD7490_CR_WEAK_VALUE   << 2)   |
        (AD7490_CR_RANGE_VALUE  << 1)   |
        (AD7490_CR_CODING_VALUE)
    ) << 4);    
}

/**
 * @brief Performs a full-duplex 16-bit SPI transaction with the AD7490.
 *
 * Hardware CS is managed automatically by the ESP-IDF SPI Master driver.
 * The ESP32 is Little-Endian, but SPI expects Big-Endian (MSB first).
 * * @param command 16-bit command to send.
 * @return uint16_t 16-bit response received from the ADC.
 */
static uint16_t write_to_register(uint16_t command) {
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    
    t.length = 16;                                          
    t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;  
    
    // Explicitly order bytes: MSB first, LSB second
    t.tx_data[0] = (command >> 8) & 0xFF; 
    t.tx_data[1] = command & 0xFF;        
    
    esp_err_t err = spi_device_polling_transmit(spi_handle, &t);
    if (err != ESP_OK) {
        RAVEN_LOGE(TAG, "SPI Transmit failed!");
        return 0;
    }
    
    // Reconstruct the 16-bit response from the received MSB and LSB
    return (t.rx_data[0] << 8) | t.rx_data[1];
}

/**
 * @brief Executes the power-up sequence required by the AD7490 datasheet.
 * * Performs two dummy writes of 0xFFFF with a 10ms delay, followed by 
 * the initial sequence configuration command.
 */
static void powerup_routine(void) {
    write_to_register(0xFFFF);
    vTaskDelay(pdMS_TO_TICKS(10));
    write_to_register(0xFFFF);
    vTaskDelay(pdMS_TO_TICKS(10));

    uint16_t command = generate_command(
        AD7490_CR_WRITE_VALUE, 
        AD7490_CR_SEQ_VALUE, 
        AD7490_CR_SHADOW_VALUE, 
        NUMBER_OF_ACTIVE_CHANNELS - 1
    );
    write_to_register(command);
}

/**
 * @brief Reads the next channel in the sequence from the AD7490.
 *
 * @param channel Pointer to store the extracted channel ID (top 4 bits).
 * @param data    Pointer to store the extracted 12-bit ADC reading.
 */
static void read_from_sequence(uint8_t *channel, uint16_t *data) {
    uint16_t payload = write_to_register(0x0000);
    *channel = payload >> 12;
    *data    = payload & 0x0FFF;
}

/* ========================================================================== */
/* PUBLIC API IMPLEMENTATIONS                                                 */
/* ========================================================================== */

void AD7490_init(void) {
    if (initialized) return;

    esp_err_t err;

    spi_bus_config_t buscfg = {
        .miso_io_num = SPI_SDOUT_PIN,
        .mosi_io_num = SPI_SDIN_PIN,
        .sclk_io_num = SPI_SCLK_PIN,
        .quadwp_io_num = -1, 
        .quadhd_io_num = -1, 
        .max_transfer_sz = 32
    };

    err = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        RAVEN_LOGE(TAG, "Failed to initialize SPI bus!");
        return;
    }

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = AD7490_SPI_FREQUENCY_HZ,
        .mode           = 0,                        
        .spics_io_num   = LINE_SENSOR_CS_PIN,       
        .queue_size     = 1,                        
        .flags          = 0                         
    };

    err = spi_bus_add_device(SPI2_HOST, &devcfg, &spi_handle);
    if (err != ESP_OK) {
        RAVEN_LOGE(TAG, "Failed to add AD7490 to SPI bus!");
        return;
    }

    powerup_routine();
    
    RAVEN_LOGI(TAG, "Initialized successfully.");
    raven_comm_send_message(TAG, "SPI Frequency: %d MHz", AD7490_SPI_FREQUENCY_HZ/1000000);
    initialized = true;
}

void AD7490_read_all_channels(uint16_t array[NUMBER_OF_ACTIVE_CHANNELS]) {
    if (!initialized) {
        raven_comm_send_message(TAG, "Not initialized!");
        return;
    }

    for (uint8_t i = 0; i < NUMBER_OF_ACTIVE_CHANNELS; i++) {
        uint8_t channel;
        uint16_t data;
        read_from_sequence(&channel, &data);
        
        // Single-line if statement enforced
        if (channel < NUMBER_OF_ACTIVE_CHANNELS) array[channel] = data;
    }
}

void AD7490_peripheral_validation(void) {
    if (!initialized) {
        raven_comm_send_message(TAG, "Not initialized!");
        return;
    }

    for (uint8_t repetitions = 0; repetitions < 10; repetitions++) {
        char buffer[RAVEN_COMM_MAX_MESSAGE_LEN];
        int offset = snprintf(buffer, sizeof(buffer), "Seq: ");
        
        for (uint8_t i = 0; i < NUMBER_OF_ACTIVE_CHANNELS; i++) {
            uint8_t channel;
            uint16_t data; 
            
            read_from_sequence(&channel, &data);
            
            // Single-line if statement enforced
            if (offset < sizeof(buffer)) offset += snprintf(buffer + offset, sizeof(buffer) - offset, "%d ", channel);
        }
    
        raven_comm_send_message(TAG, "%s", buffer);
    }
}

void AD7490_benchmark_read(void) {
    // Safety guard added
    if (!initialized) {
        raven_comm_send_message(TAG, "Not initialized!");
        return;
    }

    uint8_t channel;
    uint16_t data;

    uint64_t total_cycles = 0;
    uint32_t min_cycles = UINT32_MAX;
    uint32_t max_cycles = 0;

    uint32_t cycles_per_us = esp_rom_get_cpu_ticks_per_us();

    // Warm-up read to load function into cache
    read_from_sequence(&channel, &data);

    for (int i = 0; i < 1000; i++) {
        uint32_t start_cycles = esp_cpu_get_cycle_count();
        
        for (uint8_t ch = 0; ch < NUMBER_OF_ACTIVE_CHANNELS; ch++) {
            read_from_sequence(&channel, &data);
        }
        
        uint32_t end_cycles = esp_cpu_get_cycle_count();
        uint32_t cycles_taken = end_cycles - start_cycles;

        total_cycles += cycles_taken;
        
        // Single-line if statements enforced
        if (cycles_taken < min_cycles) min_cycles = cycles_taken;
        if (cycles_taken > max_cycles) max_cycles = cycles_taken;
    }

    uint32_t avg_cycles = (uint32_t)(total_cycles / 1000);

    float avg_us = (float)avg_cycles / cycles_per_us;
    float min_us = (float)min_cycles / cycles_per_us;
    float max_us = (float)max_cycles / cycles_per_us;

    // Logging left to RAVEN_LOGI as it's dense developer data, rather than real-time user UI data
    RAVEN_LOGI(TAG, "--- AD7490 Benchmark (%d array readings) ---", 1000);
    RAVEN_LOGI(TAG, "CPU Clock:    %lu MHz", cycles_per_us);
    RAVEN_LOGI(TAG, "Average Time: %.3f us (%lu cycles)", avg_us, avg_cycles);
    RAVEN_LOGI(TAG, "Min Time:     %.3f us (%lu cycles)", min_us, min_cycles);
    RAVEN_LOGI(TAG, "Max Time:     %.3f us (%lu cycles)", max_us, max_cycles);
}