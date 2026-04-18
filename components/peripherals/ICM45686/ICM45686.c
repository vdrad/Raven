/**
 * @file ICM45686.c
 * @brief Implementation of the ICM-45686 IMU driver.
 */

#include "ICM45686.h"

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Standard Math Library
#include <math.h>

// ESP-IDF Drivers
#include "driver/i2c_master.h"
#include "esp_cpu.h"
#include "esp_rom_sys.h"

// Project
#include "pinout.h"
#include "raven_log.h"
#include "raven_comm.h"

// ============================================================================
// Macros and Constants
// ============================================================================

#define TAG "ICM"

// Default Configuration Constants
#define ICM45686_CALIBRATION_SAMPLES    2000
#define DEFAULT_ACCEL_ODR               ICM45686_ODR_6400HZ
#define DEFAULT_ACCEL_FSR               ICM45686_ACCEL_FSR_4G
#define DEFAULT_GYRO_ODR                ICM45686_ODR_6400HZ
#define DEFAULT_GYRO_FSR                ICM45686_GYRO_FSR_4000DPS

// Register Map Addresess
#define ICM45686_I2C_ADDR               0x68
#define REG_WHO_AM_I_ADDRESS            0x72
#define REG_PWR_MGMT0                   0x10
#define REG_INT1_CONFIG0                0x16 
#define REG_INT1_CONFIG2                0x18 
#define REG_ACCEL_CONFIG0               0x1B
#define REG_GYRO_CONFIG0                0x1C
#define REG_ACCEL_DATA_X_LSB            0x00 

#define REG_WHO_AM_I_VALUE              0xE9

// ============================================================================
// Global Variables
// ============================================================================

static bool initialized = false;

static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t dev_handle;

// Variables to store the current Full Scale Ranges for mathematical conversion
static icm45686_accel_fsr_t current_accel_fsr;
static icm45686_gyro_fsr_t current_gyro_fsr;

// Variables to store the raw gyro offset for optimal subtraction
static float raw_gyro_offset_x = 0.0f;
static float raw_gyro_offset_y = 0.0f;
static float raw_gyro_offset_z = 0.0f;

// ============================================================================
// Internal Private Functions (Hardware Abstraction)
// ============================================================================

/**
 * @brief Writes a single byte to an I2C register.
 */
static esp_err_t icm_write_reg(uint8_t reg, uint8_t data) {
    uint8_t buffer[2] = {reg, data};
    return i2c_master_transmit(dev_handle, buffer, 2, 100);
}

/**
 * @brief Reads a sequence of bytes from an I2C register.
 */
static esp_err_t icm_read_regs(uint8_t reg, uint8_t *data, size_t len) {
    if (data == NULL) return ESP_ERR_INVALID_ARG;
    return i2c_master_transmit_receive(dev_handle, &reg, 1, data, len, 100);
}

/**
 * @brief Reads the WHO_AM_I register to verify device presence.
 */
static uint8_t get_who_am_i(void) {
    uint8_t who_am_i;
    esp_err_t err = icm_read_regs(REG_WHO_AM_I_ADDRESS, &who_am_i, 1);
    if (err != ESP_OK) return 0x00;
    return who_am_i;
}

/**
 * @brief Configures the accelerometer settings.
 */
static esp_err_t ICM45686_start_accel(icm45686_odr_t odr_hz, icm45686_accel_fsr_t fsr_g) {
    esp_err_t err;
    uint8_t config_reg = 0;
    uint8_t pwr_reg = 0;
    uint8_t check_reg = 0;

    err = icm_read_regs(REG_ACCEL_CONFIG0, &config_reg, 1);
    if (err != ESP_OK) return err;
    
    // Clear bits [6:4] (FSR) and [3:0] (ODR)
    config_reg &= ~(0x70 | 0x0F);

    if      (odr_hz >= ICM45686_ODR_6400HZ)   config_reg |= 0x03; 
    else if (odr_hz >= ICM45686_ODR_3200HZ)   config_reg |= 0x04; 
    else if (odr_hz >= ICM45686_ODR_1600HZ)   config_reg |= 0x05; 
    else if (odr_hz >= ICM45686_ODR_800HZ)    config_reg |= 0x06; 
    else if (odr_hz >= ICM45686_ODR_400HZ)    config_reg |= 0x07; 
    else if (odr_hz >= ICM45686_ODR_200HZ)    config_reg |= 0x08; 
    else if (odr_hz >= ICM45686_ODR_100HZ)    config_reg |= 0x09; 
    else if (odr_hz >= ICM45686_ODR_50HZ)     config_reg |= 0x0A; 
    else if (odr_hz >= ICM45686_ODR_25HZ)     config_reg |= 0x0B; 
    else if (odr_hz >= ICM45686_ODR_12HZ)     config_reg |= 0x0C; 
    else if (odr_hz >= ICM45686_ODR_6HZ)      config_reg |= 0x0D; 
    else if (odr_hz >= ICM45686_ODR_3HZ)      config_reg |= 0x0E; 
    else                                      config_reg |= 0x0F; 

    if      (fsr_g == ICM45686_ACCEL_FSR_32G) config_reg |= (0x00 << 4);
    else if (fsr_g == ICM45686_ACCEL_FSR_16G) config_reg |= (0x01 << 4);
    else if (fsr_g == ICM45686_ACCEL_FSR_8G)  config_reg |= (0x02 << 4);
    else if (fsr_g == ICM45686_ACCEL_FSR_4G)  config_reg |= (0x03 << 4);
    else                                      config_reg |= (0x04 << 4); 

    // Update the internal state for mathematical conversions later
    current_accel_fsr = fsr_g;

    err = icm_write_reg(REG_ACCEL_CONFIG0, config_reg);
    if (err != ESP_OK) return err;

    // Read-back verification
    err = icm_read_regs(REG_ACCEL_CONFIG0, &check_reg, 1);
    if (err != ESP_OK || check_reg != config_reg) {
        RAVEN_LOGE(TAG, "Accel config mismatch! Written: 0x%02X, Read: 0x%02X", config_reg, check_reg);
        return ESP_FAIL;
    }

    // Power Management Configuration
    err = icm_read_regs(REG_PWR_MGMT0, &pwr_reg, 1);
    if (err != ESP_OK) return err;

    pwr_reg &= ~0x03; 
    pwr_reg |= 0x03; // Turn on Accel in Low Noise mode

    err = icm_write_reg(REG_PWR_MGMT0, pwr_reg);
    if (err != ESP_OK) return err;

    err = icm_read_regs(REG_PWR_MGMT0, &check_reg, 1);
    if (err != ESP_OK || check_reg != pwr_reg) {
        RAVEN_LOGE(TAG, "PWR_MGMT0 config mismatch! Written: 0x%02X, Read: 0x%02X", pwr_reg, check_reg);
        return ESP_FAIL;
    }

    return ESP_OK;
}

/**
 * @brief Configures the gyroscope settings.
 */
static esp_err_t ICM45686_start_gyro(icm45686_odr_t odr_hz, icm45686_gyro_fsr_t fsr_dps) {
    esp_err_t err;
    uint8_t config_reg = 0;
    uint8_t pwr_reg = 0;
    uint8_t check_reg = 0;

    err = icm_read_regs(REG_GYRO_CONFIG0, &config_reg, 1);
    if (err != ESP_OK) return err;
    
    // Clear bits [7:4] (FSR) and [3:0] (ODR)
    config_reg &= (uint8_t)~(0xF0 | 0x0F);

    if      (odr_hz >= ICM45686_ODR_6400HZ) config_reg |= 0x03; 
    else if (odr_hz >= ICM45686_ODR_3200HZ) config_reg |= 0x04; 
    else if (odr_hz >= ICM45686_ODR_1600HZ) config_reg |= 0x05; 
    else if (odr_hz >= ICM45686_ODR_800HZ)  config_reg |= 0x06; 
    else if (odr_hz >= ICM45686_ODR_400HZ)  config_reg |= 0x07; 
    else if (odr_hz >= ICM45686_ODR_200HZ)  config_reg |= 0x08; 
    else if (odr_hz >= ICM45686_ODR_100HZ)  config_reg |= 0x09; 
    else if (odr_hz >= ICM45686_ODR_50HZ)   config_reg |= 0x0A; 
    else if (odr_hz >= ICM45686_ODR_25HZ)   config_reg |= 0x0B; 
    else if (odr_hz >= ICM45686_ODR_12HZ)   config_reg |= 0x0C; 
    else if (odr_hz >= ICM45686_ODR_6HZ)    config_reg |= 0x0D; 
    else if (odr_hz >= ICM45686_ODR_3HZ)    config_reg |= 0x0E; 
    else                                    config_reg |= 0x0F; 

    if      (fsr_dps >= ICM45686_GYRO_FSR_4000DPS) config_reg |= (0x00 << 4);
    else if (fsr_dps >= ICM45686_GYRO_FSR_2000DPS) config_reg |= (0x01 << 4);
    else if (fsr_dps >= ICM45686_GYRO_FSR_1000DPS) config_reg |= (0x02 << 4);
    else if (fsr_dps >= ICM45686_GYRO_FSR_500DPS)  config_reg |= (0x03 << 4);
    else if (fsr_dps >= ICM45686_GYRO_FSR_250DPS)  config_reg |= (0x04 << 4);
    else if (fsr_dps >= ICM45686_GYRO_FSR_125DPS)  config_reg |= (0x05 << 4);
    else if (fsr_dps >= ICM45686_GYRO_FSR_62DPS)   config_reg |= (0x06 << 4);
    else if (fsr_dps >= ICM45686_GYRO_FSR_31DPS)   config_reg |= (0x07 << 4);
    else                                           config_reg |= (0x08 << 4); 

    // Update the internal state for mathematical conversions later
    current_gyro_fsr = fsr_dps;

    err = icm_write_reg(REG_GYRO_CONFIG0, config_reg);
    if (err != ESP_OK) return err;

    // Read-back verification
    err = icm_read_regs(REG_GYRO_CONFIG0, &check_reg, 1);
    if (err != ESP_OK || check_reg != config_reg) {
        RAVEN_LOGE(TAG, "Gyro config mismatch! Written: 0x%02X, Read: 0x%02X", config_reg, check_reg);
        return ESP_FAIL;
    }

    err = icm_read_regs(REG_PWR_MGMT0, &pwr_reg, 1);
    if (err != ESP_OK) return err;

    // Mask 0x0C represents the Gyroscope power control bits
    pwr_reg &= ~0x0C; 
    pwr_reg |= 0x0C;  // Turn on Gyro in Low Noise mode

    err = icm_write_reg(REG_PWR_MGMT0, pwr_reg);
    if (err != ESP_OK) return err;

    err = icm_read_regs(REG_PWR_MGMT0, &check_reg, 1);
    if (err != ESP_OK || check_reg != pwr_reg) {
        RAVEN_LOGE(TAG, "PWR_MGMT0 (GYRO) config mismatch! Written: 0x%02X, Read: 0x%02X", pwr_reg, check_reg);
        return ESP_FAIL;
    }

    return ESP_OK;
}

/**
 * @brief Configures the Data Ready (DRDY) interrupt on the physical INT1 pin.
 */
static esp_err_t ICM45686_config_drdy_interrupt(void) {
    esp_err_t err;
    uint8_t int1_config0 = 0;
    uint8_t int1_config2 = 0;

    // STEP 1: Electrical Configuration (REG_INT1_CONFIG2 - 0x18)
    // Must be done BEFORE enabling the interrupt logic!
    err = icm_read_regs(REG_INT1_CONFIG2, &int1_config2, 1);
    if (err != ESP_OK) return err;

    // Clear bits 2, 1, and 0 (Forces Push-Pull, Pulse Mode, Active Low)
    int1_config2 &= ~0x07; 
    
    // Set bit 0 to 1 (Changes to Active High) -> Results in: Push-Pull, Pulse, Active High
    int1_config2 |= 0x01;  

    err = icm_write_reg(REG_INT1_CONFIG2, int1_config2);
    if (err != ESP_OK) {
        RAVEN_LOGE(TAG, "Failed to configure INT1_CONFIG2 (0x18) electrical settings.");
        return err;
    }

    // STEP 2: Logical Configuration (REG_INT1_CONFIG0 - 0x16)
    // Routes the "Data Ready" event to the pin
    err = icm_read_regs(REG_INT1_CONFIG0, &int1_config0, 1);
    if (err != ESP_OK) return err;

    // Enable Bit 2 (INT1_STATUS_EN_DRDY)
    int1_config0 |= (1 << 2); 

    err = icm_write_reg(REG_INT1_CONFIG0, int1_config0);
    if (err != ESP_OK) {
        RAVEN_LOGE(TAG, "Failed to route DRDY on INT1_CONFIG0 (0x16).");
        return err;
    }

    RAVEN_LOGI(TAG, "DRDY interrupt successfully configured on INT1 pin.");
    return ESP_OK;
}

/**
 * @brief Reads a 14-byte burst from the sensor and parses it into Little Endian 16-bit integers.
 */
static esp_err_t ICM45686_get_raw_data(icm45686_raw_data_t *raw_data) {
    if (raw_data == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t buffer[14]; 
    
    // Burst read starting from 0x00 (ACCEL_DATA_X_LSB)
    esp_err_t err = icm_read_regs(REG_ACCEL_DATA_X_LSB, buffer, 14);
    if (err != ESP_OK) return err;

    // Little Endian Formatting
    raw_data->accel_x = (int16_t)((uint16_t)buffer[1] << 8 | buffer[0]);
    raw_data->accel_y = (int16_t)((uint16_t)buffer[3] << 8 | buffer[2]);
    raw_data->accel_z = (int16_t)((uint16_t)buffer[5] << 8 | buffer[4]);
    
    raw_data->gyro_x = (int16_t)((uint16_t)buffer[7] << 8 | buffer[6]);
    raw_data->gyro_y = (int16_t)((uint16_t)buffer[9] << 8 | buffer[8]);
    raw_data->gyro_z = (int16_t)((uint16_t)buffer[11] << 8 | buffer[10]);
    
    raw_data->temp = (int16_t)((uint16_t)buffer[13] << 8 | buffer[12]);
    
    return ESP_OK;
}

/**
 * @brief Calculates and stores the static raw offsets of the gyroscope.
 */
static esp_err_t ICM45686_calibrate(uint16_t num_samples) {
    icm45686_raw_data_t raw;
    
    // Using int64_t to prevent overflow when summing thousands of 16-bit samples
    int64_t sum_gx = 0;
    int64_t sum_gy = 0;
    int64_t sum_gz = 0;

    raven_comm_send_message(TAG, "Starting Gyroscope calibration...");
    
    // Short delay to allow the user to release the robot
    vTaskDelay(pdMS_TO_TICKS(1000)); 

    for (uint16_t i = 0; i < num_samples; i++) {
        esp_err_t err = ICM45686_get_raw_data(&raw);
        if (err != ESP_OK) return err;

        sum_gx += raw.gyro_x;
        sum_gy += raw.gyro_y;
        sum_gz += raw.gyro_z;

        vTaskDelay(pdMS_TO_TICKS(1)); 
    }

    // Save the exact fractional average to prevent drift from integer truncation
    raw_gyro_offset_x = (float)sum_gx / (float)num_samples;
    raw_gyro_offset_y = (float)sum_gy / (float)num_samples;
    raw_gyro_offset_z = (float)sum_gz / (float)num_samples;

    raven_comm_send_message(TAG, "Calibration Completed!");
    RAVEN_LOGI(TAG, "Raw Offsets -> X: %+.3f | Y: %+.3f | Z: %+.3f", 
               raw_gyro_offset_x, raw_gyro_offset_y, raw_gyro_offset_z);

    return ESP_OK;
}

// ============================================================================
// Public API Functions
// ============================================================================

void ICM45686_init(void) {
    if (initialized) return;

    // 1. I2C Bus Initialization
    i2c_master_bus_config_t bus_config = {
        .i2c_port   = I2C_NUM_0,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ICM45686_I2C_ADDR,
        .scl_speed_hz = 1000000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle));
    
    vTaskDelay(pdMS_TO_TICKS(5)); // Wait for device to be fully supplied

    // 2. Hardware Checks and Configuration
    if (get_who_am_i() != REG_WHO_AM_I_VALUE) {
        raven_comm_send_message(TAG, "FATAL: IMU not found! Initialization aborted.");
        return; // Leave initialized = false. User must handle the state.
    }
    
    if (ICM45686_config_drdy_interrupt() != ESP_OK) {
        raven_comm_send_message(TAG, "FATAL: Failed to configure DRDY interrupt. Initialization aborted.");
        return;
    }

    if (ICM45686_start_accel(DEFAULT_ACCEL_ODR, DEFAULT_ACCEL_FSR) != ESP_OK) {
        raven_comm_send_message(TAG, "FATAL: Failed to start Accelerometer. Initialization aborted.");
        return;
    }

    if (ICM45686_start_gyro(DEFAULT_GYRO_ODR, DEFAULT_GYRO_FSR) != ESP_OK) {
        raven_comm_send_message(TAG, "FATAL: Failed to start Gyroscope. Initialization aborted.");
        return;
    }

    if (ICM45686_calibrate(ICM45686_CALIBRATION_SAMPLES) != ESP_OK) {
        raven_comm_send_message(TAG, "FATAL: Calibration failed. Initialization aborted.");
        return;
    }

    initialized = true;
    RAVEN_LOGI(TAG, "Initialized successfully.");
    raven_comm_send_message(TAG, "Accelerometer ODR: %d Hz | FSR: %d g", DEFAULT_ACCEL_ODR, DEFAULT_ACCEL_FSR);
    raven_comm_send_message(TAG, "Gyroscope ODR: %d Hz | FSR: %d dps",   DEFAULT_GYRO_ODR,  DEFAULT_GYRO_FSR);
}

void ICM45686_get_data(icm45686_data_t *out_data) {
    if (!initialized) {
        raven_comm_send_message(TAG, "Error: Cannot get data, IMU not initialized!");
        return;
    }
    if (out_data == NULL) {
        raven_comm_send_message(TAG, "Error: Null pointer provided for get_data!");
        return;
    }

    icm45686_raw_data_t raw;
    
    if (ICM45686_get_raw_data(&raw) != ESP_OK) {
        RAVEN_LOGE(TAG, "I2C read failure during get_data.");
        return;
    }

    // Accelerometer Conversion Math: (raw * FSR) / 32768.0f
    float accel_scale = (float)current_accel_fsr / 32768.0f;
    out_data->accel_x = (float)raw.accel_x * accel_scale;
    out_data->accel_y = (float)raw.accel_y * accel_scale;
    out_data->accel_z = (float)raw.accel_z * accel_scale;

    // Gyroscope Conversion Math with floating-point offset integration
    // Formula: (raw - raw_offset) * (FSR * PI) / (32768 * 180)
    float gyro_scale = ((float)current_gyro_fsr * (float)M_PI) / (32768.0f * 180.0f);
    
    out_data->gyro_x = ((float)raw.gyro_x - raw_gyro_offset_x) * gyro_scale;
    out_data->gyro_y = ((float)raw.gyro_y - raw_gyro_offset_y) * gyro_scale;
    out_data->gyro_z = ((float)raw.gyro_z - raw_gyro_offset_z) * gyro_scale;
    
    // Temperature Conversion (Datasheet formula)
    out_data->temp = ((float)raw.temp / 128.0f) + 25.0f;
}

void ICM45686_peripheral_validation(void) {
    if (!initialized) {
        raven_comm_send_message(TAG, "Error: Cannot run validation, IMU not initialized!");
        return;
    }

    icm45686_data_t sensor_data;
    
    // Fetch data (since the function returns void, we expect it to populate the struct safely)
    ICM45686_get_data(&sensor_data);
    
    raven_comm_send_message(TAG, "ACCEL [g]   | X: %+.3f | Y: %+.3f | Z: %+.3f", 
               sensor_data.accel_x, sensor_data.accel_y, sensor_data.accel_z);
               
    raven_comm_send_message(TAG, "GYRO  [rad] | X: %+.3f | Y: %+.3f | Z: %+.3f", 
               sensor_data.gyro_x, sensor_data.gyro_y, sensor_data.gyro_z);
               
    raven_comm_send_message(TAG, "TEMP  [C]   | %+.2f", sensor_data.temp);
}

/**
 * @brief Executes a performance benchmark on the ICM45686 I2C read function.
 * Calculates average, minimum, and maximum execution times across 1000 samples
 * for reading ALL active channels, using the internal CPU cycle counter.
 *
 * Benchmark Results (ESP32-S3 WROOM-1 N16R0) for a full array read:
 * - Average Time: 449.0 us
 * - Minimum Time: 448.0 us
 * - Maximum Time: 500.0 us
 * 
 */
void ICM45686_benchmark_read(void) {
    if (!initialized) {
        raven_comm_send_message(TAG, "Error: Cannot run benchmark, IMU not initialized!");
        return;
    }

    icm45686_data_t data;

    uint64_t total_cycles = 0;
    uint32_t min_cycles = UINT32_MAX;
    uint32_t max_cycles = 0;

    // Get the CPU frequency dynamically (ticks per microsecond / MHz)
    uint32_t cycles_per_us = esp_rom_get_cpu_ticks_per_us();

    // Warm-up read to bring the instruction stack into IRAM cache
    ICM45686_get_data(&data);

    // Main measurement loop
    for (int i = 0; i < 1000; i++) {
        uint32_t start_cycles = esp_cpu_get_cycle_count();
        
        ICM45686_get_data(&data);
        
        uint32_t end_cycles = esp_cpu_get_cycle_count();
        uint32_t cycles_taken = end_cycles - start_cycles;

        total_cycles += cycles_taken;
        if (cycles_taken < min_cycles) min_cycles = cycles_taken;
        if (cycles_taken > max_cycles) max_cycles = cycles_taken;
    }

    // Final mathematical calculations
    uint32_t avg_cycles = (uint32_t)(total_cycles / 1000);

    float avg_us = (float)avg_cycles / cycles_per_us;
    float min_us = (float)min_cycles / cycles_per_us;
    float max_us = (float)max_cycles / cycles_per_us;

    RAVEN_LOGI(TAG, "--- ICM45686 I2C Benchmark (1000 burst reads) ---");
    RAVEN_LOGI(TAG, "CPU Clock:    %lu MHz", cycles_per_us);
    RAVEN_LOGI(TAG, "Average Time: %.3f us (%lu cycles)", avg_us, avg_cycles);
    RAVEN_LOGI(TAG, "Min Time:     %.3f us (%lu cycles)", min_us, min_cycles);
    RAVEN_LOGI(TAG, "Max Time:     %.3f us (%lu cycles)", max_us, max_cycles);
}