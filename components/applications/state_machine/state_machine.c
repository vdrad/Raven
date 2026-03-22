#include <stddef.h>
#include <stdint.h>

// Standard ESP-IDF FreeRTOS includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Project-specific includes
#include "state_machine.h"
#include "raven_log.h"
#include "raven_comm.h"
#include "ble_manager.h"
#include "peripheral_validation.h"
#include "rgb_led.h"
#include "buzzer.h"

/* ========================================================================== */
/* MACROS & TYPES                                                             */
/* ========================================================================== */

// Macro to forward-declare a state function and its string name for debugging
#define ADD_STATE(state)                    \
    static void *state_##state(void *args); \
    static const uint8_t _state_##state##_name[] = {#state}

// Macro to change the current state by updating the callback and name pointers
#define CHANGE_STATE(state_func)   \
    state_machine.cb = state_func; \
    state_machine.name = _##state_func##_name

// Typedef for the state function signature
typedef void *(*state_callback)(void *);


/* ========================================================================== */
/* STATE DECLARATIONS                                                         */
/* ========================================================================== */

// Forward declare all available states here using the macro
ADD_STATE(wait_user_connection);
ADD_STATE(initialization);
ADD_STATE(configuration);
ADD_STATE(test);
ADD_STATE(validation);


/* ========================================================================== */
/* PRIVATE VARIABLES                                                          */
/* ========================================================================== */

// The core state machine object tracking the current state
static struct {
    const uint8_t *name;  // Stores the string name of the state (useful for logs)
    state_callback cb;    // Function pointer to the current state execution
} state_machine = {
    .name = _state_wait_user_connection_name,
    .cb = state_wait_user_connection
};

// Handle to manage the FreeRTOS task running the state machine
static TaskHandle_t state_machine_task_handle = NULL;


/* ========================================================================== */
/* STATE FUNCTIONS                                                            */
/* ========================================================================== */

/**
 * @brief Wait for user connection state: Waits for a client device to be connected.
 * Transitions to: initialization state.
 */
static void *state_wait_user_connection(void *args) {
    raven_comm_init();

    RAVEN_LOGI("WAIT CONN", "Waiting for user connection.");
    if (ble_manager_get_connection_status()) {
        CHANGE_STATE(state_initialization);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));

    return NULL;
}

/**
 * @brief Initialization state: Sets up peripherals and communications.
 * Transitions to: Test state.
 */
static void *state_initialization(void *args) {
    rgb_led_init();
    buzzer_init();

    CHANGE_STATE(state_configuration);
    return NULL;
}

/**
 * @brief Configuration state: Configurates robot.
 * Transitions to: Test state or validation state.
 */
static void *state_configuration(void *args) {
    // todo: Add command read, and based on that, switch states.
    CHANGE_STATE(state_test);
    return NULL;
}

/**
 * @brief Test state: Acts as a transitional or self-test phase.
 * Transitions to: Validation state.
 */
static void *state_test(void *args) {
    // Add any self-test logic or checks here if needed in the future

    CHANGE_STATE(state_validation);
    return NULL;
}

/**
 * @brief Validation state: Continuously runs peripheral validations.
 * Transitions to: Itself (looping state).
 */
static void *state_validation(void *args) {
    peripheral_validation(PERIPHERAL_ALL);
    
    // Delay to yield to other tasks and prevent hardware watchdog triggers
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Loop back to the same state
    CHANGE_STATE(state_validation);
    return NULL;
}


/* ========================================================================== */
/* FREERTOS TASK                                                              */
/* ========================================================================== */

/**
 * @brief FreeRTOS task that continuously drives the state machine.
 * @param pvParameters Standard FreeRTOS task parameter (unused here).
 */
static void state_machine_task(void *pvParameters) {
    // Setup: Ensure we start from a clean, known state (Initialization)
    state_machine_reset();

    // Infinite loop processing the current state callback
    for (;;) {
        state_machine_step();
    }

    // Failsafe: if we ever break out of the loop, delete the task properly
    vTaskDelete(NULL);
}


/* ========================================================================== */
/* PUBLIC API                                                                 */
/* ========================================================================== */

/**
 * @brief Forces the state machine back to the initialization state.
 */
void state_machine_reset(void) { 
    CHANGE_STATE(state_wait_user_connection); 
}

/**
 * @brief Executes the current state's callback function once.
 */
void state_machine_step(void) { 
    // Null-check to prevent HardFaults just in case the pointer gets corrupted
    if (state_machine.cb != NULL) {
        state_machine.cb(NULL); 
    }
}

/**
 * @brief Retrieves the string name of the current state.
 * @return const uint8_t* Pointer to the state name string.
 */
const uint8_t *state_get_name() { 
    return state_machine.name; 
}

/**
 * @brief Initializes and spawns the FreeRTOS task for the state machine.
 * Pinned to Core 1 (APP_CPU) to keep it isolated from Wi-Fi/Radio tasks on Core 0.
 */
void state_machine_init(void) {
    xTaskCreatePinnedToCore(
        state_machine_task,          // Task function
        "state_machine",             // Task name (for debugging in RTOS monitors)
        4096,                        // Stack size in words
        NULL,                        // Task parameters
        5,                           // Priority (higher number = higher priority)
        &state_machine_task_handle,  // Task handle
        1                            // Pinned to Core 1
    );  
}