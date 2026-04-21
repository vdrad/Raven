#include "race_manager.h"

#include <math.h>

// ESP-IDF Includes
#include "esp_timer.h"
#include "esp_cpu.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Project Includes
#include "raven_log.h"
#include "raven_comm.h"
#include "line_reading.h"
#include "odometry.h"
#include "controller.h"
#include "motor.h"
#include "robot_telemetry.h"

#define TAG "RMG"

#define RACE_MANAGER_REFRESH_RATE_MS 1

// Macro function to compute the increment per tick based on the given acceleration
#define GET_ACCEL_PER_TICK(accel_mps2) \
    ((accel_mps2) * ((float)RACE_MANAGER_REFRESH_RATE_MS / 1000.0f))

/* ========================================================================== */
/* INTERNAL STATE VARIABLES                                                   */
/* ========================================================================== */

static race_manager_status_t race_status = RACE_STATUS_PRE_START_ZONE;

static esp_timer_handle_t race_timer_handle = NULL;
static bool is_timer_running = false;

static float current_straightline_speed_mps = 0.0f;
static float target_straightline_speed_mps  = RACE_MANAGER_DEFAULT_STRAIGHTLINE_SPEED_MPS;

static float configured_straightline_speed_mps = RACE_MANAGER_DEFAULT_STRAIGHTLINE_SPEED_MPS;
static float configured_fan_voltage = RACE_MANAGER_DEFAULT_FAN_VOLTAGE;

// Tracks the current acceleration rate requested by the state logic
static float current_acceleration_mps2 = RACE_MANAGER_DEFAULT_ROBOT_ACCELERATION_MPS2;

// Tracks the start time of the race in microseconds
static int64_t race_start_time_us = 0;

/* ========================================================================== */
/* PRIVATE FUNCTION IMPLEMENTATIONS                                           */
/* ========================================================================== */

static void straightline_speed_manager(void) {
    float error = target_straightline_speed_mps - current_straightline_speed_mps;
    float current_step = GET_ACCEL_PER_TICK(current_acceleration_mps2);

    if (fabsf(error) <= current_step) {
        current_straightline_speed_mps = target_straightline_speed_mps;
    } 
    else if (error > 0.0f) {
        current_straightline_speed_mps += current_step;
    } 
    else if (error < 0.0f) {
        current_straightline_speed_mps -= current_step;
    }
}

static void race_manager_cb(void *arg) {
    straightline_speed_manager();

    line_reading_update();
    odometry_update();

    line_reading_data_t line_data = line_reading_get_data();
    odometry_data_t     odom_data = odometry_get_data();

    line_position_pid.current_reading = line_data.position.position / 1000.0f;
    pid_compute(&line_position_pid);
    float position_feedback = line_position_pid.output;

    left_motor_pid.current_reading  = odom_data.velocity_left_m_s;
    right_motor_pid.current_reading = odom_data.velocity_right_m_s;

    left_motor_pid.setpoint  = current_straightline_speed_mps - position_feedback;
    right_motor_pid.setpoint = current_straightline_speed_mps + position_feedback;

    if (left_motor_pid.setpoint  > RACE_MANAGER_MAX_ROBOT_SPEED_MPS)  left_motor_pid.setpoint  =  RACE_MANAGER_MAX_ROBOT_SPEED_MPS;
    if (left_motor_pid.setpoint  < -RACE_MANAGER_MAX_ROBOT_SPEED_MPS) left_motor_pid.setpoint  = -RACE_MANAGER_MAX_ROBOT_SPEED_MPS;
    if (right_motor_pid.setpoint > RACE_MANAGER_MAX_ROBOT_SPEED_MPS)  right_motor_pid.setpoint =  RACE_MANAGER_MAX_ROBOT_SPEED_MPS;
    if (right_motor_pid.setpoint < -RACE_MANAGER_MAX_ROBOT_SPEED_MPS) right_motor_pid.setpoint = -RACE_MANAGER_MAX_ROBOT_SPEED_MPS;

    pid_compute(&left_motor_pid);
    pid_compute(&right_motor_pid);

    motor_set_voltage(MOTOR_LEFT, left_motor_pid.output);
    motor_set_voltage(MOTOR_RIGHT, right_motor_pid.output);
    
    // A. Check for Disaster (Global Override)
    if (line_data.position.robot_lost == true && 
        race_status != RACE_STATUS_OFF_TRACK && 
        race_status != RACE_STATUS_STOPPED) {

        race_status = RACE_STATUS_OFF_TRACK;
        motor_set_voltage(MOTOR_LEFT, 0.0f);
        motor_set_voltage(MOTOR_RIGHT, 0.0f);
        
        target_straightline_speed_mps = 0.0f;
        current_straightline_speed_mps = 0.0f; 
    
        robot_telemetry_record_frame();

        raven_comm_send_message(TAG, "Line lost. Dist: %.2fm | Vel: %.2fm/s", 
                                odom_data.distance_traveled_robot_m, 
                                odom_data.velocity_robot_m_s);
        return;
    }

    // B. Normal Track Progression
    switch (race_status) {
        case RACE_STATUS_PRE_START_ZONE:
            if (line_data.markers.right_markers_counter == 1) {
                odometry_reset(); 
                race_start_time_us = esp_timer_get_time();
                
                race_status = RACE_STATUS_RACING;
                current_acceleration_mps2 = RACE_MANAGER_DEFAULT_ROBOT_ACCELERATION_MPS2;
                target_straightline_speed_mps  = configured_straightline_speed_mps; 

                raven_comm_send_message(TAG, "Start line crossed. Accelerating to %.1f m/s.", 
                                        target_straightline_speed_mps);
            }
            break;

        case RACE_STATUS_RACING:
            if (line_data.markers.right_markers_counter == 2) {
                race_status = RACE_STATUS_COMPLETED;
                float lap_time_s = (float)(esp_timer_get_time() - race_start_time_us) / 1000000.0f;
                
                target_straightline_speed_mps = 0.0f;
                current_acceleration_mps2 = RACE_MANAGER_SLOWDOWN_ACCELERATION_MPS2;

                raven_comm_send_message(TAG, "Finish line crossed!\nTrack length: %.2fm\nLap Time: %.3fs.\nAverage Velocity: %.1f m/s", 
                                        odom_data.distance_traveled_robot_m, lap_time_s, odom_data.distance_traveled_robot_m/lap_time_s);
            }
            break;

        case RACE_STATUS_COMPLETED:
            if (current_straightline_speed_mps <= 0.0f) {
                race_status = RACE_STATUS_STOPPED;
                raven_comm_send_message(TAG, "Full stop achieved. Total distance incl. braking: %.2fm", 
                                        odom_data.distance_traveled_robot_m);
            }
            break;

        case RACE_STATUS_STOPPED:
        case RACE_STATUS_OFF_TRACK:
            break;
            
        default:
            break;
    }

    if (race_status == RACE_STATUS_RACING) robot_telemetry_record_frame();
}

/* ========================================================================== */
/* PUBLIC API IMPLEMENTATIONS                                                 */
/* ========================================================================== */

void race_manager_init(void) {
    xTaskCreate(race_manager_commands_task, "race_manager_commands_task", 4096, NULL, 5, NULL);

    if (race_timer_handle == NULL) {
        const esp_timer_create_args_t timer_args = {
            .callback = &race_manager_cb,
            .name = "race_timer"
        };
        esp_timer_create(&timer_args, &race_timer_handle);
    }

    line_position_pid.max_output =  2.0f * RACE_MANAGER_MAX_ROBOT_SPEED_MPS;
    line_position_pid.min_output = -2.0f * RACE_MANAGER_MAX_ROBOT_SPEED_MPS;

    RAVEN_LOGI(TAG, "Initialized successfully.");
}

void set_target_straightline_speed(float given_speed_mps) {
    target_straightline_speed_mps = given_speed_mps;
}

void race_manager_start(void) {
    if (is_timer_running) return;

    current_straightline_speed_mps = 0.0f;
    target_straightline_speed_mps = 0.8;
    current_acceleration_mps2      = RACE_MANAGER_DEFAULT_ROBOT_ACCELERATION_MPS2;
    race_start_time_us             = 0;
    race_status                    = RACE_STATUS_PRE_START_ZONE;

    line_position_pid.last_run_time_us = esp_timer_get_time();

    esp_timer_start_periodic(race_timer_handle, RACE_MANAGER_REFRESH_RATE_MS * 1000);
    is_timer_running = true;
}

void race_manager_stop(void) {
    if (race_timer_handle != NULL && is_timer_running) {
        esp_timer_stop(race_timer_handle);
        is_timer_running = false;
    }
    
    motor_set_voltage(MOTOR_LEFT,  0.0f);
    motor_set_voltage(MOTOR_RIGHT, 0.0f);
    
    current_straightline_speed_mps = 0.0f;
    target_straightline_speed_mps  = 0.0f;
    race_start_time_us             = 0;
    race_status                    = RACE_STATUS_PRE_START_ZONE;
}

race_manager_status_t race_manager_get_status(void) {
    return race_status;
}

void race_manager_benchmark_cb(void) {
    if (race_timer_handle == NULL) {
        raven_comm_send_message(TAG, "Error: Cannot run benchmark, Race Manager not initialized!");
        return;
    }

    uint64_t total_cycles = 0;
    uint32_t min_cycles = UINT32_MAX;
    uint32_t max_cycles = 0;
    uint32_t cycles_per_us = esp_rom_get_cpu_ticks_per_us();

    // --- SAFETY PRESERVATION ---
    race_manager_status_t original_status = race_status;
    float original_target = target_straightline_speed_mps;
    float original_current = current_straightline_speed_mps;

    race_manager_cb(NULL);

    for (int i = 0; i < 1000; i++) {
        uint32_t start_cycles = esp_cpu_get_cycle_count();
        race_manager_cb(NULL);
        uint32_t end_cycles = esp_cpu_get_cycle_count();
        uint32_t cycles_taken = end_cycles - start_cycles;

        total_cycles += cycles_taken;
        if (cycles_taken < min_cycles) min_cycles = cycles_taken;
        if (cycles_taken > max_cycles) max_cycles = cycles_taken;
    }

    // --- SAFETY RESTORE ---
    race_status = original_status;
    target_straightline_speed_mps = original_target;
    current_straightline_speed_mps = original_current;
    motor_set_voltage(MOTOR_LEFT, 0.0f);
    motor_set_voltage(MOTOR_RIGHT, 0.0f);

    uint32_t avg_cycles = (uint32_t)(total_cycles / 1000);
    float avg_us = (float)avg_cycles / cycles_per_us;
    float min_us = (float)min_cycles / cycles_per_us;
    float max_us = (float)max_cycles / cycles_per_us;

    RAVEN_LOGI(TAG, "--- Race Manager Callback Benchmark (1000 runs) ---");
    RAVEN_LOGI(TAG, "Average Time: %.3f us (%lu cycles)", avg_us, avg_cycles);
    raven_comm_send_message(TAG, "CB Benchmark - Avg: %.1fus | Max: %.1fus", avg_us, max_us);
}

void race_manager_set_configured_speed(float speed) {
    configured_straightline_speed_mps = speed;
}

float race_manager_get_configured_speed(void) {
    return configured_straightline_speed_mps;
}

void race_manager_set_configured_fan_voltage(float voltage) {
    configured_fan_voltage = voltage;
}

float race_manager_get_configured_fan_voltage(void) {
    return configured_fan_voltage;
}