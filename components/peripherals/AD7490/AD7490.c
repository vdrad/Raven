/**
 * @file AD7490.c
 * @brief ESP-IDF SPI Driver for the AD7490 16-channel, 12-bit ADC.
 */

#include "AD7490.h"
#include <string.h>
#include <stdio.h>

// FreeRTOS includes for vTaskDelay
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
 * The ESP32 is Little-Endian, but SPI typically expects Big-Endian (MSB first).
 * The tx_data array is explicitly populated MSB first to handle this correctly.
 *
 * @param command 16-bit command to send.
 * @return 16-bit response received from the ADC.
 */
static uint16_t write_to_register(uint16_t command) {
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    
    t.length = 16;                                          // Transaction length in bits
    t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;  // Use internal buffers (up to 32 bits)
    
    // Explicitly order bytes: MSB first, LSB second
    t.tx_data[0] = (command >> 8) & 0xFF; 
    t.tx_data[1] = command & 0xFF;        
    
    // Polling transmit blocks the task until the SPI transaction finishes
    esp_err_t err = spi_device_polling_transmit(spi_handle, &t);
    if (err != ESP_OK) {
        RAVEN_LOGE(TAG, "SPI Transmit failed!");
        return 0;
    }
    
    // Reconstruct the 16-bit response from the received MSB and LSB
    return (t.rx_data[0] << 8) | t.rx_data[1];
}

/**
 * @brief Executes the specific power-up sequence required by the AD7490.
 *
 * Requires two dummy writes of 0xFFFF with a 10ms delay between them.
 */
static void powerup_routine(void) {
    write_to_register(0xFFFF);
    vTaskDelay(pdMS_TO_TICKS(10));
    write_to_register(0xFFFF);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Configure the sequencer for the desired number of channels
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

void AD7490_init() {
    if (initialized) return;

    esp_err_t err;

    // 1. Configure and Initialize the SPI Bus
    spi_bus_config_t buscfg = {
        .miso_io_num = SPI_SDOUT_PIN,
        .mosi_io_num = SPI_SDIN_PIN,
        .sclk_io_num = SPI_SCLK_PIN,
        .quadwp_io_num = -1, // Not used
        .quadhd_io_num = -1, // Not used
        .max_transfer_sz = 32
    };

    err = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        RAVEN_LOGE(TAG, "Failed to initialize SPI bus!");
        return;
    }

    // 2. Configure and Attach the AD7490 Device to the Bus
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = AD7490_SPI_FREQUENCY_HZ,
        .mode           = 0,                        // SPI mode 0 (CPOL=0, CPHA=0)
        .spics_io_num   = LINE_SENSOR_CS_PIN,       // Hardware CS pin
        .queue_size     = 1,                        // We only use polling, queue of 1 is enough
        .flags          = 0                         // Default behavior
    };

    err = spi_bus_add_device(SPI2_HOST, &devcfg, &spi_handle);
    if (err != ESP_OK) {
        RAVEN_LOGE(TAG, "Failed to add AD7490 to SPI bus!");
        return;
    }

    // 3. Hardware specific setup
    powerup_routine();
    
    raven_comm_send_message(TAG, "Initialized successfully.");
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
        
        // Safety check to prevent array out-of-bounds corruption
        if (channel < NUMBER_OF_ACTIVE_CHANNELS) {
            array[channel] = data;
        }
    }
}

/**
 * @brief Executes a hardware validation routine by forcing a sequencer reset 
 * and reading all channels. Proves SPI integrity by printing the embedded 
 * channel ID from the ADC's raw frame.
 *
 * @param print_values If true, prints both the channel ID and its ADC reading (e.g., "C1:4095").
 * If false, prints only the channel sequence (e.g., "1 2 3 4").
 */
void AD7490_peripheral_validation(bool print_values) {
    if (!initialized) {
        raven_comm_send_message(TAG, "Not initialized!");
        return;
    }

    // 1. Prepare the string buffer for BLE transmission
    char buffer[RAVEN_COMM_MAX_MESSAGE_LEN];
    int offset = snprintf(buffer, sizeof(buffer), "Seq: ");
    
    // 2. Read directly from the sequence to capture the exact arrival order
    for (uint8_t i = 0; i < NUMBER_OF_ACTIVE_CHANNELS; i++) {
        uint8_t channel;
        uint16_t data; 
        
        // Fetch raw data straight from the SPI bus
        read_from_sequence(&channel, &data);
        
        // Append based on the user's choice
        if (offset < sizeof(buffer)) {
            if (print_values) offset += snprintf(buffer + offset, sizeof(buffer) - offset, "C%d:%d ", channel, data);
            else offset += snprintf(buffer + offset, sizeof(buffer) - offset, "%d ", channel);
        }
    }

    // 3. Send the proof of life to the app/terminal
    raven_comm_send_message(TAG, "%s", buffer);
}

/**
 * @brief Executes a performance benchmark on the AD7490 SPI read function.
 * Calculates average, minimum, and maximum execution times across 1000 samples
 * for reading ALL active channels, using the internal CPU cycle counter.
 *
 * Benchmark Results (ESP32-S3 Mini 1U) for a full array read:
 * - Average Time: 350.0 us
 * - Minimum Time: 349.0 us
 * - Maximum Time: 365.0 us
 * - Equivalent Array Sampling Freq: ~2.85 kHz 
 * * Note: Consumes roughly 35% of a 1ms tick, leaving plenty of headroom 
 * to run a 1 kHz PID loop synchronously.
 */
void AD7490_benchmark_read(void) {
    uint8_t channel;
    uint16_t data;

    uint64_t total_cycles = 0;
    uint32_t min_cycles = UINT32_MAX;
    uint32_t max_cycles = 0;

    // 1. Get the current CPU frequency dynamically (ticks per microsecond / MHz)
    uint32_t cycles_per_us = esp_rom_get_cpu_ticks_per_us();

    // 2. "Warm-up" read
    // Brings the function into cache memory (IRAM/Flash cache) to prevent 
    // the first read from being artificially slow due to a cache miss.
    read_from_sequence(&channel, &data);

    // 3. Main measurement loop
    for (int i = 0; i < 1000; i++) {
        // Lock the start cycle
        uint32_t start_cycles = esp_cpu_get_cycle_count();
        
        // Execute the target function (Reading all active channels)
        for (uint8_t ch = 0; ch < NUMBER_OF_ACTIVE_CHANNELS; ch++) {
            read_from_sequence(&channel, &data);
        }
        
        // Lock the end cycle and calculate the difference
        uint32_t end_cycles = esp_cpu_get_cycle_count();
        uint32_t cycles_taken = end_cycles - start_cycles;

        // Accumulate for the average and update the records (Min/Max)
        total_cycles += cycles_taken;
        if (cycles_taken < min_cycles) min_cycles = cycles_taken;
        if (cycles_taken > max_cycles) max_cycles = cycles_taken;
    }

    // 4. Final calculations
    uint32_t avg_cycles = (uint32_t)(total_cycles / 1000);

    float avg_us = (float)avg_cycles / cycles_per_us;
    float min_us = (float)min_cycles / cycles_per_us;
    float max_us = (float)max_cycles / cycles_per_us;

    // 5. Print the report through the robot's communication system
    raven_comm_send_message(TAG, "--- AD7490 Benchmark (%d array readings) ---", 1000);
    raven_comm_send_message(TAG, "CPU Clock:    %lu MHz", cycles_per_us);
    raven_comm_send_message(TAG, "Average Time: %.3f us (%lu cycles)", avg_us, avg_cycles);
    raven_comm_send_message(TAG, "Min Time:     %.3f us (%lu cycles)", min_us, min_cycles);
    raven_comm_send_message(TAG, "Max Time:     %.3f us (%lu cycles)", max_us, max_cycles);
}