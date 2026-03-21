#include "state_machine.h"
#include "freeRTOS/freeRTOS.h"
#include <stddef.h>
#include "raven_log.h"
#include "raven_comm.h"

// MACROS
#define ADD_STATE(state)                    \
    static void *state_##state(void *args); \
    static const uint8_t _state_##state##_name[] = {#state}

#define CHANGE_STATE(state_func)   \
    state_machine.cb = state_func; \
    state_machine.name = _##state_func##_name

typedef void *(*state_callback)(void *);

// AVAILABLE STATES
ADD_STATE(initialization);
ADD_STATE(test);

// STATE CREATION
static struct {
    const uint8_t *name;  // Debug reasons
    state_callback cb;
} state_machine = {.name = _state_initialization_name,
                   .cb = state_initialization};

void state_machine_reset(void) { CHANGE_STATE(state_initialization); }

// PUBLIC API
void state_machine_step(void) { state_machine.cb(NULL); }
const uint8_t *state_get_name() { return state_machine.name; }

// STATE FUNCTIONS
// Initialization
static void *state_initialization(void *args) {
    raven_comm_init();

    CHANGE_STATE(state_test);
    return NULL;
}

// Test
static void *state_test(void *args) {
    raven_comm_send_message("TEST", "Hello World!");
    vTaskDelay(pdMS_TO_TICKS(1000));

    return NULL;
}