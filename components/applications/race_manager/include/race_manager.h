#pragma once

typedef enum {
    RACE_STATUS_PRE_START_ZONE, // Inside the start-finish region
    RACE_STATUS_RACING,         // Running on the track
    RACE_STATUS_COMPLETED,      // Run is over
    RACE_STATUS_STOPPED,        // Braking finished, velocity is 0
    RACE_STATUS_OFF_TRACK       // Robot left the line
} race_manager_status_t;

race_manager_status_t race_manager_get_status(void);
void race_manager_init(void);
void race_manager_stop(void);
void race_manager_start(void);
void race_manager_benchmark_cb(void);