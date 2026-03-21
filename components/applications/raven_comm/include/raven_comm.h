#pragma once

// MACROS
#define RAVEN_COMM_MAX_MESSAGE_LEN  128

// FUNCTIONS
void raven_comm_init(void);
void raven_comm_send_message(const char *tag, const char *format, ...);