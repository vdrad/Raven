#pragma once

// INCLUDES
#include <stdint.h>

// FUNCTIONS
void state_machine_init(void);
void state_machine_step(void);
const uint8_t *state_get_name(void);
void state_machine_reset(void);
void state_machine_set_command(const char *payload);