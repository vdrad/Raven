/**
 * @file state_machine.c
 * @brief Core State Machine for the robot.
 *
 * Manages the high-level behavior of the robot, including initialization, 
 * testing, validation, and operational modes. It uses a thread-safe pending 
 * transition architecture to prevent race conditions when external FreeRTOS 
 * tasks request state changes.
 */

#include <stddef.h>
#include <stdint.h>
#include <string.h>

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
#include "battery_sensor.h"

#define TAG "SMA"

/* ========================================================================== */
/* MACROS, TYPES & ENUMS                                                      */
/* ========================================================================== */

/**
 * @brief Macro to forward-declare a state function and its string name for debugging.
 */
#define ADD_STATE(state)                    \
    static void *state_##state(void *args); \
    static const uint8_t _state_##state##_name[] = {#state}

/**
 * @brief Macro to safely request a state transition.
 * * Instead of changing the pointer immediately, this registers a pending request
 * protected by a spinlock. The main state machine loop will apply this change
 * before the next execution cycle, preventing race conditions.
 */
#define REQUEST_STATE(state_func) do {                                 \
    taskENTER_CRITICAL(&state_spinlock);                               \
    state_machine.pending_cb = state_func;                             \
    state_machine.pending_name = _##state_func##_name;                 \
    state_machine.has_pending_request = true;                          \
    taskEXIT_CRITICAL(&state_spinlock);                                \
} while(0)

/** @brief Typedef for the standard state function signature. */
typedef void *(*state_callback)(void *);

/**
 * @brief Internal commands recognized by the State Machine command decoder.
 */
typedef enum {
    SMA_CMD_UNKNOWN = 0,                    /**< Unrecognized command header. */
    SMA_CMD_ENTER_TESTING_STATE,            /**< State Machine command (Header: 'S', Payload: 'TST'). */
    SMA_CMD_ENTER_VALIDATION_STATE,         /**< Hardware validation command (Header: 'S', Payload: 'VLD'). */
} state_machine_cmd_type_t;

/* ========================================================================== */
/* FORWARD DECLARATIONS                                                       */
/* ========================================================================== */

// --- States ---
ADD_STATE(wait_user_connection);
ADD_STATE(initialization);
ADD_STATE(configuration);
ADD_STATE(test);
ADD_STATE(validation);

// --- Private Helpers ---
static state_machine_cmd_type_t command_decoder(char *payload);
static void apply_pending_state_transition(void);

// --- FreeRTOS Tasks ---
static void state_machine_task(void *pvParameters);
static void state_machine_commands_task(void *pvParameters);
static void state_machine_failsafe_task(void *pvParameters);

/* ========================================================================== */
/* PRIVATE VARIABLES                                                          */
/* ========================================================================== */

/** * @brief The core state machine object tracking current and pending states. 
 */
static struct {
    const uint8_t *name;               /**< String name of the current state */
    state_callback cb;                 /**< Function pointer to the current state execution */
    
    const uint8_t *pending_name;       /**< String name of the requested next state */
    state_callback pending_cb;         /**< Function pointer to the requested next state */
    bool has_pending_request;          /**< Flag indicating a transition was requested */
} state_machine = {
    .name = _state_wait_user_connection_name,
    .cb = state_wait_user_connection,
    .pending_name = NULL,
    .pending_cb = NULL,
    .has_pending_request = false
};

/** @brief Spinlock to protect the pending state variables during concurrent access. */
static portMUX_TYPE state_spinlock = portMUX_INITIALIZER_UNLOCKED;

// --- Task Handles ---
static TaskHandle_t state_machine_task_handle = NULL;
static TaskHandle_t state_machine_commands_task_handle = NULL;
static TaskHandle_t state_machine_failsafe_task_handle = NULL;

/* ========================================================================== */
/* PUBLIC API                                                                 */
/* ========================================================================== */

/**
 * @brief Forces the state machine back to the initial waiting state.
 */
void state_machine_reset(void) { 
    REQUEST_STATE(state_wait_user_connection); 
}

/**
 * @brief Executes one cycle of the state machine.
 * * Safely applies any pending state transitions requested by other tasks, 
 * then executes the active state's callback function.
 */
void state_machine_step(void) { 
    // 1. Check and apply any requested state changes safely
    apply_pending_state_transition();

    // 2. Execute the current state
    state_machine.cb(NULL); 
}

/**
 * @brief Retrieves the string name of the currently active state.
 * @return const uint8_t* Pointer to the state name string.
 */
const uint8_t *state_get_name(void) { 
    return state_machine.name; 
}

/**
 * @brief Initializes and spawns all FreeRTOS tasks related to the state machine.
 * * Tasks are pinned to Core 1 (APP_CPU) to keep them isolated from 
 * Wi-Fi/Radio/BLE tasks running on Core 0.
 */
void state_machine_init(void) {
    // 1. Core State Machine Task
    xTaskCreatePinnedToCore(
        state_machine_task, "state_machine", 4096, NULL, 5, 
        &state_machine_task_handle, 1
    );  

    // 2. Command Listener Task
    xTaskCreatePinnedToCore(
        state_machine_commands_task, "sma_commands", 4096, NULL, 5, 
        &state_machine_commands_task_handle, 1
    );  

    // 3. Failsafe & Emergency Stop Task
    xTaskCreatePinnedToCore(
        state_machine_failsafe_task, "sma_failsafe", 4096, NULL, 5, 
        &state_machine_failsafe_task_handle, 1
    );
}

/* ========================================================================== */
/* STATE FUNCTIONS                                                            */
/* ========================================================================== */

/**
 * @brief Waits for a client device to connect via BLE.
 * @return NULL
 */
static void *state_wait_user_connection(void *args) {
    raven_comm_init();

    RAVEN_LOGI("SWC", "Waiting for user connection.");
    if (ble_manager_get_connection_status()) {
        RAVEN_LOGI("SWC", "User connected successfully.");
        REQUEST_STATE(state_initialization);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));

    return NULL;
}

/**
 * @brief Sets up hardware peripherals and communications.
 * @return NULL
 */
static void *state_initialization(void *args) {
    rgb_led_init();
    buzzer_init();
    battery_sensor_init();

    raven_comm_send_message(TAG, "All devices initialized.");
    REQUEST_STATE(state_configuration);
    return NULL;
}

/**
 * @brief Handles robot configuration routines.
 * @return NULL
 */
static void *state_configuration(void *args) {
    // Add configuration logic here
    vTaskDelay(pdMS_TO_TICKS(500));
    return NULL;
}

/**
 * @brief Acts as a transitional or self-test phase for the hardware.
 * @return NULL
 */
static void *state_test(void *args) {
    // Add self-test logic here
    battery_sensor_peripheral_validation();
    vTaskDelay(pdMS_TO_TICKS(500));
    return NULL;
}

/**
 * @brief Continuously runs peripheral validations triggered by the user.
 * @return NULL
 */
static void *state_validation(void *args) {
    peripheral_validation(PERIPHERAL_ALL);
    
    // Delay to yield to other tasks and prevent hardware watchdog triggers
    vTaskDelay(pdMS_TO_TICKS(1000));
    return NULL;
}

/* ========================================================================== */
/* PRIVATE HELPER FUNCTIONS                                                   */
/* ========================================================================== */

/**
 * @brief Safely applies a requested state transition using a critical section.
 */
static void apply_pending_state_transition(void) {
    // ENTER CRITICAL SECTION: Quickly check and clear the request flag
    if (state_machine.has_pending_request) {
        taskENTER_CRITICAL(&state_spinlock);
        state_machine.cb = state_machine.pending_cb;
        state_machine.name = state_machine.pending_name;
        state_machine.has_pending_request = false;
        taskEXIT_CRITICAL(&state_spinlock);

        raven_comm_send_message(TAG, "State transitioned to: %s", state_machine.name);
    }
}

/**
 * @brief Decodes the string payload into a specific state machine command enum.
 * @param payload The raw string payload received from BLE.
 * @return The corresponding state_machine_cmd_type_t enum value.
 */
static state_machine_cmd_type_t command_decoder(char *payload) {
    if (strcmp(payload, "TST") == 0) return SMA_CMD_ENTER_TESTING_STATE;
    if (strcmp(payload, "VLD") == 0) return SMA_CMD_ENTER_VALIDATION_STATE;

    raven_comm_send_message(TAG, "Invalid input '%s'. Expecting 'STST' or 'SVLD'.", payload);
    return SMA_CMD_UNKNOWN;
}

/* ========================================================================== */
/* FREERTOS TASKS                                                             */
/* ========================================================================== */

/**
 * @brief Main task that continuously drives the state machine execution.
 */
static void state_machine_task(void *pvParameters) {
    // Setup: Ensure we start from a clean, known state
    state_machine_reset();

    // Infinite loop processing the current state callback
    for (;;) {
        state_machine_step();
    }
    
    // Failsafe: if we ever break out of the loop, delete the task properly
    vTaskDelete(NULL);
}

/**
 * @brief Background task that listens for incoming commands targeted at the state machine.
 */
static void state_machine_commands_task(void *pvParameters) {
    char received_cmd[RAVEN_COMM_MAX_PAYLOAD_LEN];

    for (;;) {
        // Elegantly checks the central mailbox for new messages
        if (raven_comm_check_new_message(CMD_STATE_MACHINE, received_cmd)) {
            state_machine_cmd_type_t cmd = command_decoder(received_cmd);

            switch (cmd) {
                case SMA_CMD_ENTER_TESTING_STATE:
                    REQUEST_STATE(state_test);
                    break;

                case SMA_CMD_ENTER_VALIDATION_STATE:
                    REQUEST_STATE(state_validation);
                    break;

                default:
                    break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    vTaskDelete(NULL);
}

/**
 * @brief High-priority task monitoring critical systems to trigger emergency stops.
 */
static void state_machine_failsafe_task(void *pvParameters) {
    for (;;) {
        // TODO: Read battery level
        // TODO: Read line sensor (cliff detection)
        // TODO: Monitor emergency stop button/command
        
        // Example logic:
        // if (battery_is_critical() || cliff_detected()) {
        //     motor_stop_all();
        //     REQUEST_STATE(state_wait_user_connection); // Safe fallback state
        // }

        // Runs frequently to ensure extremely fast reaction times
        vTaskDelay(pdMS_TO_TICKS(10000)); 
    }
    vTaskDelete(NULL);
}