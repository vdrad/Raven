/**
 * @file state_machine.c
 * @brief Core State Machine for the robot.
 *
 * Manages the high-level behavior of the robot, including initialization, 
 * testing, validation, and operational modes. It uses a thread-safe pending 
 * transition architecture to prevent race conditions when external FreeRTOS 
 * tasks request state changes.
 */

#include "state_machine.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

// Standard ESP-IDF and FreeRTOS includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

// Project-specific includes
#include "raven_log.h"
#include "raven_comm.h"
#include "ble_manager.h"
#include "peripheral_validation.h"
#include "battery_sensor.h"
#include "ICM45686.h"
#include "motor.h"
#include "odometry.h"
#include "controller.h"
#include "line_reading.h"
#include "notifications.h"

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
    SMA_CMD_UNKNOWN = 0,                        /**< Unrecognized command header. */
    SMA_CMD_ENTER_LINE_CALIBRATION_STATE,       /**< Line Calibration command (Header: 'S', Payload: 'CAL'). */
    SMA_CMD_ENTER_RACING_STATE,                 /**< Racing command (Header: 'S', Payload: 'RCN'). */
    SMA_CMD_ENTER_EMERGENCY_STOP_STATE,         /**< Emergency Stop command (Header: 'S', Payload: 'STP'). */
    SMA_CMD_ENTER_TESTING_STATE,                /**< State Machine command (Header: 'S', Payload: 'TST'). */
    SMA_CMD_ENTER_VALIDATION_STATE,             /**< Hardware validation command (Header: 'S', Payload: 'VLD'). */
    SMA_CMD_ENTER_PID_TUNING_STATE,             /**< PID Tuning command (Header: 'S', Payload: 'PID'). */
    SMA_CMD_ENTER_MOTOR_CHARACTERIZATION_STATE  /**< Motor Characterization command (Header: 'S', Payload: 'MCR'). */
} state_machine_cmd_type_t;

/* ========================================================================== */
/* FORWARD DECLARATIONS                                                       */
/* ========================================================================== */

// --- States ---

// Initialization
ADD_STATE(wait_user_connection);
ADD_STATE(initialization);
ADD_STATE(configuration);
ADD_STATE(line_calibration);

// Core
ADD_STATE(armed);
ADD_STATE(countdown);
ADD_STATE(racing);
ADD_STATE(cooldown);
ADD_STATE(emergency_stop);

// Auxiliary
ADD_STATE(test);
ADD_STATE(validation);
ADD_STATE(pid_tuning);
ADD_STATE(motor_characterization);

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
/* PUBLIC API IMPLEMENTATIONS                                                 */
/* ========================================================================== */

void state_machine_reset(void) {
    REQUEST_STATE(state_wait_user_connection);
}

void state_machine_step(void) {
    // 1. Check and apply any requested state changes safely
    apply_pending_state_transition();
    
    // 2. Execute the current state
    if (state_machine.cb != NULL) state_machine.cb(NULL); // Single-line if enforced
}

const uint8_t *state_get_name(void) {
    return state_machine.name;
}

void state_machine_init(void) {
    // TODO: move this to robot_manager
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize tasks for failsafe and background command listening
    xTaskCreate(state_machine_task, "sma_task", 4096, NULL, 5, &state_machine_task_handle);
    xTaskCreate(state_machine_commands_task, "sma_cmd_task", 2048, NULL, 4, &state_machine_commands_task_handle);
    xTaskCreatePinnedToCore(state_machine_failsafe_task, "sma_failsafe", 2048, NULL, 6, &state_machine_failsafe_task_handle, 1);

    RAVEN_LOGI(TAG, "Initialized successfully.");
    raven_comm_send_message(TAG, "State Machine Booted. Active State: %s", state_machine.name);
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
    notifications_init();
    battery_sensor_init();
    motor_init();
    odometry_init();
    controller_init();
    line_reading_init();
    ICM45686_init();

    raven_comm_send_message(TAG, "All devices initialized.\n");
    REQUEST_STATE(state_configuration);
    return NULL;
}

/**
 * @brief Handles robot configuration routines.
 * @return NULL
 */
static void *state_configuration(void *args) {
    REQUEST_STATE(state_line_calibration);
    return NULL;
}

/**
 * @brief Handles robot line calibration.
 * @return NULL
 */
static void *state_line_calibration(void *args) {
    line_reading_calibrate();
    REQUEST_STATE(state_armed);
    return NULL;
}

/**
 * @brief System is configured, calibrated, and waiting for the start signal.
 * @return NULL
 */
static void *state_armed(void *args) {
    vTaskDelay(pdMS_TO_TICKS(500));
    return NULL;
}

/**
 * @brief Warmup suction fan and start accelerating.
 * @return NULL
 */
static void *state_countdown(void *args) {
    vTaskDelay(pdMS_TO_TICKS(500));
    REQUEST_STATE(state_racing);
    return NULL;
}

/**
 * @brief The main PID control loop is driving the robot.
 * @return NULL
 */
static void *state_racing(void *args) {
    vTaskDelay(pdMS_TO_TICKS(500));
    REQUEST_STATE(state_cooldown);
    return NULL;
}

/**
 * @brief Race is finished, the robot is progressively decelerating untill full stop.
 * @return NULL
 */
static void *state_cooldown(void *args) {
    vTaskDelay(pdMS_TO_TICKS(500));
    REQUEST_STATE(state_emergency_stop);
    return NULL;
}

/**
 * @brief The user requested immediate stop or a failsafe condition was detected.
 * @return NULL
 */
static void *state_emergency_stop(void *args) {
    vTaskDelay(pdMS_TO_TICKS(500));
    return NULL;
}

/**
 * @brief Acts as a transitional or self-test phase for the hardware.
 * @return NULL
 */
static void *state_test(void *args) {
    // Add self-test logic here
    // AD7490_peripheral_validation();
    // AD7490_benchmark_read();
    // ICM45686_peripheral_validation();
    // ICM45686_benchmark_read();
    // ICM45686_i2c_scan();
    // encoder_peripheral_validation();
    // motor_set_voltage(MOTOR_RIGHT, 1.0f);
    // odometry_data_t odometry_data = odometry_get_data();
    // RAVEN_LOGI("TST", "Right Vel: %.1f | Distance: %.2fm", odometry_data.velocity_right_mm_s/1000.0f, odometry_data.distance_traveled_robot_m);
    // vTaskDelay(pdMS_TO_TICKS(10));
    
    // controller_run();
    // controller_pid_tuner();
    // odometry_update();
    // odometry_data_t data = odometry_get_data();
    // RAVEN_LOGI("TST", "%.2fm", data.distance_traveled_robot_m);
    // vTaskDelay(pdMS_TO_TICKS(100));

    // line_reading_raw_validation();
    // line_reading_normalized_validation();
    // line_reading_position_validation();
    // line_reading_markers_validation();

    notifications_validation();

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

/**
 * @brief Perform PID Tuning.
 * @return NULL
 */
static void *state_pid_tuning(void *args) {
    controller_tune_drive_motors();
    REQUEST_STATE(state_configuration);

    return NULL;
}

/**
 * @brief Perform Motor Characterization.
 * @return NULL
 */
static void *state_motor_characterization(void *args) {
    motor_characterization_run();
    REQUEST_STATE(state_configuration);

    return NULL;
}

/* ========================================================================== */
/* PRIVATE FUNCTION IMPLEMENTATIONS                                           */
/* ========================================================================== */

static void apply_pending_state_transition(void) {
    if (!state_machine.has_pending_request) return;

    const uint8_t *old_state = state_machine.name;
    const uint8_t *new_state = state_machine.pending_name;

    taskENTER_CRITICAL(&state_spinlock);
    state_machine.name = state_machine.pending_name;
    state_machine.cb = state_machine.pending_cb;
    
    state_machine.pending_name = NULL;
    state_machine.pending_cb = NULL;
    state_machine.has_pending_request = false;
    taskEXIT_CRITICAL(&state_spinlock);

    raven_comm_send_message(TAG, "Transition: [%s] -> [%s]", old_state, new_state);
}

/**
 * @brief Decodes the string payload into a specific state machine command enum.
 * @param payload The raw string payload received from BLE.
 * @return The corresponding state_machine_cmd_type_t enum value.
 */
static state_machine_cmd_type_t command_decoder(char *payload) {
    if (strcmp(payload, "CAL") == 0) return SMA_CMD_ENTER_LINE_CALIBRATION_STATE;
    if (strcmp(payload, "RCN") == 0) return SMA_CMD_ENTER_RACING_STATE;
    if (strcmp(payload, "STP") == 0) return SMA_CMD_ENTER_EMERGENCY_STOP_STATE;
    if (strcmp(payload, "TST") == 0) return SMA_CMD_ENTER_TESTING_STATE;
    if (strcmp(payload, "VLD") == 0) return SMA_CMD_ENTER_VALIDATION_STATE;
    if (strcmp(payload, "PID") == 0) return SMA_CMD_ENTER_PID_TUNING_STATE;
    if (strcmp(payload, "MCR") == 0) return SMA_CMD_ENTER_MOTOR_CHARACTERIZATION_STATE;

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
                case SMA_CMD_ENTER_PID_TUNING_STATE:
                    REQUEST_STATE(state_pid_tuning);
                    break;
                case SMA_CMD_ENTER_MOTOR_CHARACTERIZATION_STATE:
                    REQUEST_STATE(state_motor_characterization);
                    break;
                case SMA_CMD_ENTER_LINE_CALIBRATION_STATE:
                    REQUEST_STATE(state_line_calibration);
                    break;
                case SMA_CMD_ENTER_RACING_STATE:
                    if (state_machine.cb == state_armed) REQUEST_STATE(state_countdown);
                    else raven_comm_send_message(TAG, "Command Rejected: Robot is not ARMED.");
                    break;
                case SMA_CMD_ENTER_EMERGENCY_STOP_STATE:
                    REQUEST_STATE(state_emergency_stop);
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