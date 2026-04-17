/**
 * @file peripheral_validation.c
 * @brief Hardware and peripheral validation suite.
 *
 * This module provides an interactive test suite to validate the physical 
 * integrity and operation of the robot's peripherals (LEDs, buzzers, etc.). 
 * It interacts with the user via the communication link, asking for manual 
 * verification of hardware feedback.
 */

#include "peripheral_validation.h"
#include <stdbool.h>
#include <string.h>

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Project includes
#include "raven_comm.h"
#include "raven_log.h"
#include "rgb_led.h"
#include "buzzer.h"
#include "battery_sensor.h"
#include "AD7490.h"
#include "ICM45686.h"
#include "encoder.h"
#include "motor.h"

#define TAG "VLD"

/* ========================================================================== */
/* MACROS, TYPES & ENUMS                                                      */
/* ========================================================================== */

/**
 * @brief Structure representing a single peripheral test in the validation suite.
 */
typedef struct {
    const char *name;               /**< Printable name of the peripheral */
    void (*execute_test)(void);     /**< Function pointer to trigger the test */
    bool passed;                    /**< Stores the user's pass/fail verdict */
} peripheral_test_t;

/**
 * @brief Internal commands recognized by the Peripheral Validation command decoder.
 */
typedef enum {
    VLD_CMD_UNKNOWN = 0,    /**< Unrecognized command payload. */
    VLD_CMD_OK,             /**< Start the validation process (Payload: 'OK'). */
    VLD_CMD_PASS,           /**< Mark current test as passed (Payload: 'PASS'). */
    VLD_CMD_FAIL,           /**< Mark current test as failed (Payload: 'FAIL'). */
    VLD_CMD_ABORT,          /**< Cancel the validation process (Payload: 'ABORT'). */
} peripheral_validation_cmd_type_t;

/* ========================================================================== */
/* PRIVATE FUNCTION DECLARATIONS                                              */
/* ========================================================================== */

static void validate_all_peripherals(void);
static peripheral_validation_cmd_type_t command_decoder(char *payload);

/* ========================================================================== */
/* FUNCTION IMPLEMENTATIONS                                                   */
/* ========================================================================== */

/**
 * @brief Main routing function for peripheral validation.
 *
 * @param peripheral The specific peripheral or group of peripherals to validate.
 */
void peripheral_validation(peripheral_to_validate_t peripheral) {
    switch (peripheral) {
        case PERIPHERAL_ALL:
            validate_all_peripherals();
            break;
        case PERIPHERAL_RGB_LED:
            rgb_led_peripheral_validation();
            break;
        case PERIPHERAL_BUZZER:
            buzzer_peripheral_validation();
            break;
        case PERIPHERAL_BATTERY_SENSOR:
            battery_sensor_peripheral_validation();
            break;
        case PERIPHERAL_AD7490:
            AD7490_peripheral_validation();
            break;
        case PERIPHERAL_ICM45686:
            ICM45686_peripheral_validation();
            break;
        case PERIPHERAL_ENCODER:
            encoder_peripheral_validation();
            break;

        default:
            RAVEN_LOGW(TAG, "Unknown peripheral for validation.");
            break;
    }
}

/**
 * @brief Decodes the string payload into a specific validation command enum.
 * * @param payload The raw string payload received from the communication link.
 * @return The corresponding peripheral_validation_cmd_type_t enum value.
 */
static peripheral_validation_cmd_type_t command_decoder(char *payload) {
    if (strcmp(payload, "OK") == 0)    return VLD_CMD_OK;
    if (strcmp(payload, "PASS") == 0)  return VLD_CMD_PASS;
    if (strcmp(payload, "FAIL") == 0)  return VLD_CMD_FAIL;
    if (strcmp(payload, "ABORT") == 0) return VLD_CMD_ABORT;

    return VLD_CMD_UNKNOWN;
}

/**
 * @brief Validates all peripherals sequentially, waiting for user confirmation.
 *
 * This function blocks execution until a 'VOK' command is received, then 
 * iterates through a suite of tests, waiting for 'VPASS', 'VFAIL', or 'VABORT' 
 * for each. Finally, it generates a comprehensive validation report.
 */
static void validate_all_peripherals(void) {
    // 1. Define the Test Suite using an array of structs
    peripheral_test_t test_suite[] = {
        {"RGB LED",   rgb_led_peripheral_validation,         false},
        {"FAN",       fan_peripheral_validation,             false},
        {"MOTOR",     motor_peripheral_validation,           false},
        {"BUZZER",    buzzer_peripheral_validation,          false},
        {"BATTERY",   battery_sensor_peripheral_validation,  false},
        {"ADC",       AD7490_peripheral_validation,          false},
        {"IMU",       ICM45686_peripheral_validation,        false},
        {"ENCODER",   encoder_peripheral_validation,         false}
        // To add a new device, just add one line here! e.g., {"Infrared", ir_validate, false}
    };
    
    uint8_t num_tests = sizeof(test_suite) / sizeof(test_suite[0]);
    uint8_t passed_count = 0;
    uint8_t failed_count = 0;
    char received_cmd[RAVEN_COMM_MAX_PAYLOAD_LEN];
    bool start_validation = false;

    raven_comm_send_message(TAG, "========= COMPLETE PERIPHERAL VALIDATION =========");
    raven_comm_send_message(TAG, "Send 'VOK' to start the process.");

    // 2. Wait for the initial "OK" or "ABORT"
    while (!start_validation) {
        if (raven_comm_check_new_message(CMD_VALIDATION, received_cmd)) {
            peripheral_validation_cmd_type_t cmd = command_decoder(received_cmd);
            
            switch (cmd) {
                case VLD_CMD_OK:
                    raven_comm_send_message(TAG, "'VOK' command accepted! Starting test suite...");
                    start_validation = true;
                    break;
                    
                case VLD_CMD_ABORT:
                    raven_comm_send_message(TAG, "Validation aborted by user before starting.");
                    return; // Exits the function completely
                    
                default:
                    raven_comm_send_message(TAG, "Expected 'VOK', but received 'V%s'.", received_cmd);
                    break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Yield to the OS
    }

    // 3. Sequential Validation Loop
    for (uint8_t i = 0; i < num_tests; i++) {
        raven_comm_send_message(TAG, "=====================================");
        raven_comm_send_message(TAG, "Testing [%s].", test_suite[i].name);
        raven_comm_send_message(TAG, "Send 'VPASS', 'VFAIL', or 'VABORT'.");
        
        // Trigger the specific hardware function
        test_suite[i].execute_test();
        bool waiting_for_verdict = true;

        // Wait for the human to judge the test
        while (waiting_for_verdict) {
            if (raven_comm_check_new_message(CMD_VALIDATION, received_cmd)) {
                peripheral_validation_cmd_type_t cmd = command_decoder(received_cmd);
                
                switch (cmd) {
                    case VLD_CMD_PASS:
                        test_suite[i].passed = true;
                        passed_count++;
                        waiting_for_verdict = false;
                        raven_comm_send_message(TAG, "[%s] marked as PASSED.", test_suite[i].name);
                        break;

                    case VLD_CMD_FAIL:
                        test_suite[i].passed = false;
                        failed_count++;
                        waiting_for_verdict = false;
                        raven_comm_send_message(TAG, "[%s] marked as FAILED.", test_suite[i].name);
                        break;

                    case VLD_CMD_ABORT:
                        raven_comm_send_message(TAG, "VALIDATION ABORTED! Canceling remaining tests...");
                        return; // Exits the entire validation process immediately

                    default:
                        raven_comm_send_message(TAG, "Invalid input '%s'. Expecting 'VPASS', 'VFAIL', or 'VABORT'.", received_cmd);
                        break;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(100)); // Yield to the OS
        }
        
        // Small breather before jumping to the next hardware test
        vTaskDelay(pdMS_TO_TICKS(500)); 
    }

    // 4. Generate the Final Report
    raven_comm_send_message(TAG, "==================================================");
    raven_comm_send_message(TAG, "                VALIDATION REPORT                 ");
    raven_comm_send_message(TAG, "==================================================");
    raven_comm_send_message(TAG, "Total Devices Tested : %d", num_tests);
    raven_comm_send_message(TAG, "Successfully Passed  : %d", passed_count);
    raven_comm_send_message(TAG, "Failed Devices       : %d", failed_count);
    raven_comm_send_message(TAG, "--------------------------------------------------");
    
    // Print Passed List
    if (passed_count > 0) {
        raven_comm_send_message(TAG, "PASSED DEVICES:");
        for (uint8_t i = 0; i < num_tests; i++) {
            if (test_suite[i].passed) raven_comm_send_message(TAG, " - %s", test_suite[i].name);
        }
    }

    // Print Failed List
    if (failed_count > 0) {
        raven_comm_send_message(TAG, "FAILED DEVICES:");
        for (uint8_t i = 0; i < num_tests; i++) {
            if (!test_suite[i].passed) raven_comm_send_message(TAG, " - %s", test_suite[i].name);
        }
    }
    raven_comm_send_message(TAG, "==================================================");
}