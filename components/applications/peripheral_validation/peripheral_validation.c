#include "peripheral_validation.h"
#include <stdbool.h>
#include <string.h>

// FreeRTOS includes for vTaskDelay
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Project includes
#include "raven_comm.h" // Required for RAVEN_COMM_MAX_PAYLOAD_LEN
#include "raven_log.h"
#include "rgb_led.h"
#include "buzzer.h"

#define TAG "VALIDATION"

/* ========================================================================== */
/* PRIVATE MODULE VARIABLES                                                   */
/* ========================================================================== */

// Static buffer to store the last command received from the communication task
static char current_payload[RAVEN_COMM_MAX_PAYLOAD_LEN] = {0};

// Flag indicating a new command is available
static volatile bool has_new_command = false;

/* ========================================================================== */
/* INTERNAL STRUCTURES                                                        */
/* ========================================================================== */

/**
 * @brief Structure representing a single peripheral test in the validation suite.
 */
typedef struct {
    const char *name;               /**< Printable name of the peripheral */
    void (*execute_test)(void);     /**< Function pointer to trigger the test */
    bool passed;                    /**< Stores the user's pass/fail verdict */
} peripheral_test_t;

/* ========================================================================== */
/* PRIVATE FUNCTION DECLARATIONS                                              */
/* ========================================================================== */

void validate_all_peripherals(void);
bool peripheral_validation_get_command(char *out_buffer);

/* ========================================================================== */
/* FUNCTION IMPLEMENTATIONS                                                   */
/* ========================================================================== */

/**
 * @brief Receives the payload from the communication module and stores it locally.
 * * This function is called by the decoder task in raven_comm.
 * * @param payload The null-terminated string received via BLE.
 */
void peripheral_validation_handle_command(const char *payload) {
    if (payload == NULL) return;

    // Copies the payload safely and raises the new command flag
    strncpy(current_payload, payload, RAVEN_COMM_MAX_PAYLOAD_LEN - 1);
    current_payload[RAVEN_COMM_MAX_PAYLOAD_LEN - 1] = '\0';
    has_new_command = true; 
}

/**
 * @brief Reads the stored command and clears the new command flag.
 * * @param out_buffer Pointer to a character array where the payload will be copied.
 * @return true if a new command was read, false otherwise.
 */
bool peripheral_validation_get_command(char *out_buffer) {
    if (!has_new_command) {
        return false;
    }

    // Copies the saved command and resets the local buffer
    strncpy(out_buffer, current_payload, RAVEN_COMM_MAX_PAYLOAD_LEN);
    memset(current_payload, 0, RAVEN_COMM_MAX_PAYLOAD_LEN);
    has_new_command = false;

    return true;
}

/**
 * @brief Main routing function for peripheral validation.
 * * @param peripheral The specific peripheral or group to validate.
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

        default:
            RAVEN_LOGW(TAG, "Unknown peripheral for validation.");
            break;
    }
}
/**
 * @brief Validates all peripherals sequentially, waiting for app confirmation.
 * * This function blocks execution until a 'VOK' command is received, then 
 * iterates through a suite of tests, waiting for 'VPASS' or 'VFAIL' for each.
 * Finally, it generates a comprehensive validation report via the log system.
 */
void validate_all_peripherals(void) {
    raven_comm_send_message(TAG, "=== COMPLETE PERIPHERAL VALIDATION ===");
    raven_comm_send_message(TAG, "Send 'VOK' to start the process.");
    
    char received_cmd[RAVEN_COMM_MAX_PAYLOAD_LEN];
    bool start_validation = false;

    // 1. Wait for the initial "OK"
    while (!start_validation) {
        if (peripheral_validation_get_command(received_cmd)) {
            if (strcmp(received_cmd, "OK") == 0) {
                raven_comm_send_message(TAG, "'VOK' command accepted! Starting test suite...");
                start_validation = true; 
            } else {
                raven_comm_send_message(TAG, "Expected 'VOK', but received 'V%s'.", received_cmd);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Yield to the OS
    }

    // 2. Define the Test Suite using an array of structs
    peripheral_test_t test_suite[] = {
        {"RGB LED", rgb_led_peripheral_validation, false},
        {"Buzzer",  buzzer_peripheral_validation,  false}
        // To add a new device, just add one line here! e.g., {"Infrared", ir_validate, false}
    };
    
    uint8_t num_tests = sizeof(test_suite) / sizeof(test_suite[0]);
    uint8_t passed_count = 0;
    uint8_t failed_count = 0;

    // 3. Sequential Validation Loop
    for (uint8_t i = 0; i < num_tests; i++) {
        raven_comm_send_message(TAG, "=====================================");
        raven_comm_send_message(TAG, "Testing [%s]. Observe the hardware.", test_suite[i].name);
        raven_comm_send_message(TAG, "Send 'VPASS' if working, or 'VFAIL' if it failed.");
        
        // Trigger the specific hardware function
        test_suite[i].execute_test();
        bool waiting_for_verdict = true;

        // Wait for the human to judge the test
        while (waiting_for_verdict) {
            if (peripheral_validation_get_command(received_cmd)) {
                if (strcmp(received_cmd, "PASS") == 0) {
                    test_suite[i].passed = true;
                    passed_count++;
                    waiting_for_verdict = false;
                    raven_comm_send_message(TAG, "[%s] marked as PASSED.", test_suite[i].name);
                } else if (strcmp(received_cmd, "FAIL") == 0) {
                    test_suite[i].passed = false;
                    failed_count++;
                    waiting_for_verdict = false;
                    raven_comm_send_message(TAG, "[%s] marked as FAILED.", test_suite[i].name);
                } else {
                    raven_comm_send_message(TAG, "Invalid input '%s'. Expecting 'VPASS' or 'VFAIL'.", received_cmd);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(100)); // Yield to the OS
        }
        
        // Small breather before jumping to the next hardware test
        vTaskDelay(pdMS_TO_TICKS(500)); 
    }

    // 4. Generate the Final Report (Zero Heap Fragmentation!)
    raven_comm_send_message(TAG, "==================================================");
    raven_comm_send_message(TAG, "             VALIDATION REPORT                    ");
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