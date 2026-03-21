#include "state_machine.h"

void app_main(void) {
    // SETUP
    state_machine_reset();

    // LOOP
    for (;;) state_machine_step();
}