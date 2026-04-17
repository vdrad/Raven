#pragma once

/* ========================================================================== */
/* MACROS & CONFIGURATIONS                                                    */
/* ========================================================================== */

#define RACE_MANAGER_DEFAULT_STRAIGHTLINE_SPEED_MPS  2.6f
#define RACE_MANAGER_MAX_ROBOT_SPEED_MPS             4.0f

#define RACE_MANAGER_DEFAULT_FAN_VOLTAGE             9.0f
#define RACE_MANAGER_DEFAULT_FAN_ACCELERATION_VPS    1.0f

// Define your different acceleration profiles
#define RACE_MANAGER_DEFAULT_ROBOT_ACCELERATION_MPS2 8.0f
#define RACE_MANAGER_SLOW_ACCELERATION_MPS2          4.0f

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

// Config API
void race_manager_set_configured_speed(float speed);
float race_manager_get_configured_speed(void);

void race_manager_set_configured_fan_voltage(float voltage);
float race_manager_get_configured_fan_voltage(void);