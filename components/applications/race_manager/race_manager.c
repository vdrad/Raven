#include "race_manager.h"

#include <math.h>

// ESP-IDF Includes
#include "esp_timer.h"
#include "esp_cpu.h"
#include "esp_rom_sys.h"

// Project Includes
#include "raven_log.h"
#include "raven_comm.h"
#include "line_reading.h"
#include "odometry.h"
#include "controller.h"
#include "motor.h"

#define TAG "RMG"

/* ========================================================================== */
/* MACROS & CONFIGURATIONS                                                    */
/* ========================================================================== */

#define RACE_MANAGER_REFRESH_RATE_MS                 1
#define RACE_MANAGER_DEFAULT_STRAIGHTLINE_SPEED_MPS  2.2f
#define RACE_MANAGER_MAX_ROBOT_SPEED_MPS             4.0f

// Define your different acceleration profiles
#define RACE_MANAGER_DEFAULT_ROBOT_ACCELERATION_MPS2 8.0f
#define RACE_MANAGER_SLOW_ACCELERATION_MPS2          3.0f

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

// Tracks the current acceleration rate requested by the state logic
static float current_acceleration_mps2 = RACE_MANAGER_DEFAULT_ROBOT_ACCELERATION_MPS2;

// Tracks the start time of the race in microseconds
static int64_t race_start_time_us = 0;

/* ========================================================================== */
/* PRIVATE FUNCTION IMPLEMENTATIONS                                           */
/* ========================================================================== */

/**
 * @brief Smoothly ramps the current speed towards the target speed using dynamic acceleration.
 */
static void straightline_speed_manager(void) {
    float error = target_straightline_speed_mps - current_straightline_speed_mps;
    
    // Compute the step size dynamically based on the active acceleration profile
    float current_step = GET_ACCEL_PER_TICK(current_acceleration_mps2);

    // Snap to target if within one tick step, otherwise ramp
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

/**
 * @brief The 1 kHz Hardware Timer Callback
 */
static void race_manager_cb(void *arg) {
    // 1. Process speed profiling
    straightline_speed_manager();

    // 2. Update sensors
    line_reading_update();
    odometry_update();

    line_reading_data_t line_data = line_reading_get_data();
    odometry_data_t     odom_data = odometry_get_data();

    // 3. Calculate position output (Outer Loop)
    line_position_pid.current_reading = line_data.position.position;
    pid_compute(&line_position_pid);
    float position_feedback = line_position_pid.output;

    // 4. Calculate motor inputs (Inner Loop)
    left_motor_pid.current_reading  = odom_data.velocity_left_m_s;
    right_motor_pid.current_reading = odom_data.velocity_right_m_s;

    left_motor_pid.setpoint  = current_straightline_speed_mps - position_feedback;
    right_motor_pid.setpoint = current_straightline_speed_mps + position_feedback;

    // 5. Feed motor controllers
    pid_compute(&left_motor_pid);
    pid_compute(&right_motor_pid);

    // 6. Update motors
    motor_set_voltage(MOTOR_LEFT, left_motor_pid.output);
    motor_set_voltage(MOTOR_RIGHT, right_motor_pid.output);
    
    // 7. Update status & Handle Emergencies
    
    // A. Check for Disaster (Global Override)
    if (line_data.position.robot_lost == true && 
        race_status != RACE_STATUS_OFF_TRACK && 
        race_status != RACE_STATUS_STOPPED) {

        race_status = RACE_STATUS_OFF_TRACK;
        motor_set_voltage(MOTOR_LEFT, 0.0f);
        motor_set_voltage(MOTOR_RIGHT, 0.0f);
        
        target_straightline_speed_mps = 0.0f;
        current_straightline_speed_mps = 0.0f; 
    
        raven_comm_send_message(TAG, "Line lost. Dist: %.2fm | Vel: %.2fm/s", 
                                odom_data.distance_traveled_robot_m, 
                                odom_data.velocity_robot_m_s);
        return;
    }

    // B. Normal Track Progression (State Machine)
    switch (race_status) {
        
        case RACE_STATUS_PRE_START_ZONE:
            if (line_data.markers.right_markers_counter == 1) {
                // Edge trigger: sync distances and record start time!
                odometry_reset(); 
                race_start_time_us = esp_timer_get_time();
                
                race_status = RACE_STATUS_RACING;
                current_acceleration_mps2 = RACE_MANAGER_DEFAULT_ROBOT_ACCELERATION_MPS2;

                raven_comm_send_message(TAG, "Start line crossed. Accelerating to %.1f m/s.", 
                                        target_straightline_speed_mps);
            }
            break;

        case RACE_STATUS_RACING:
            if (line_data.markers.right_markers_counter == 2) {
                race_status = RACE_STATUS_COMPLETED;
                
                // Calculate lap time in seconds
                float lap_time_s = (float)(esp_timer_get_time() - race_start_time_us) / 1000000.0f;
                
                // Switch to slow braking
                target_straightline_speed_mps = 0.0f;
                current_acceleration_mps2 = RACE_MANAGER_SLOW_ACCELERATION_MPS2;

                raven_comm_send_message(TAG, "Finish line crossed!\nTrack length: %.2fm\nLap Time: %.3fs.\nAverage Velocity: %.1f m/s", 
                                        odom_data.distance_traveled_robot_m, lap_time_s, odom_data.distance_traveled_robot_m/lap_time_s);
            }
            break;

        case RACE_STATUS_COMPLETED:
            if (current_straightline_speed_mps <= 0.0f) {
                race_status = RACE_STATUS_STOPPED;

                // Fixed the indentation here
                raven_comm_send_message(TAG, "Full stop achieved. Total distance incl. braking: %.2fm", 
                                        odom_data.distance_traveled_robot_m);
            }
            break;

        case RACE_STATUS_STOPPED:
        case RACE_STATUS_OFF_TRACK:
            // Terminal states. We just sit here and wait for the global 
            // FreeRTOS state_machine task to call race_manager_stop().
            break;
            
        default:
            break;
    }
}

/* ========================================================================== */
/* PUBLIC API IMPLEMENTATIONS                                                 */
/* ========================================================================== */

void race_manager_init(void) {
    if (race_timer_handle == NULL) {
        const esp_timer_create_args_t timer_args = {
            .callback = &race_manager_cb,
            .name = "race_timer"
        };
        esp_timer_create(&timer_args, &race_timer_handle);
    }

    line_position_pid.max_output =  RACE_MANAGER_MAX_ROBOT_SPEED_MPS;
    line_position_pid.min_output = -RACE_MANAGER_MAX_ROBOT_SPEED_MPS;

    RAVEN_LOGI(TAG, "Initialized successfully.");
}

void set_target_straightline_speed(float given_speed_mps) {
    target_straightline_speed_mps = given_speed_mps;
}

void race_manager_start(void) {
    if (is_timer_running) return;

    // Reset speeds, acceleration, timers, and status for a safe, fresh start
    current_straightline_speed_mps = 0.0f;
    target_straightline_speed_mps  = RACE_MANAGER_DEFAULT_STRAIGHTLINE_SPEED_MPS;
    current_acceleration_mps2      = RACE_MANAGER_DEFAULT_ROBOT_ACCELERATION_MPS2;
    race_start_time_us             = 0;
    race_status                    = RACE_STATUS_PRE_START_ZONE;

    esp_timer_start_periodic(race_timer_handle, RACE_MANAGER_REFRESH_RATE_MS * 1000);
    is_timer_running = true;
}

void race_manager_stop(void) {
    if (race_timer_handle != NULL && is_timer_running) {
        esp_timer_stop(race_timer_handle);
        is_timer_running = false;
    }
    
    // If the global state machine triggers an emergency stop and calls this function,
    // the timer physically dies and 0V is immediately sent to the hardware.
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

/**
 * @brief Executes a performance benchmark on the 1 kHz race_manager callback.
 * Calculates average, minimum, and maximum execution times across 1000 samples
 * using the internal CPU cycle counter. 
 * * Target: Must be strictly < 1000.0 us to fit within the 1ms refresh rate!
 */
void race_manager_benchmark_cb(void) {
    if (race_timer_handle == NULL) {
        raven_comm_send_message(TAG, "Error: Cannot run benchmark, Race Manager not initialized!");
        return;
    }

    uint64_t total_cycles = 0;
    uint32_t min_cycles = UINT32_MAX;
    uint32_t max_cycles = 0;

    // Get the CPU frequency dynamically (ticks per microsecond / MHz)
    uint32_t cycles_per_us = esp_rom_get_cpu_ticks_per_us();

    // --- SAFETY PRESERVATION ---
    // Save current states so the 1000 loops don't permanently mess up the robot
    race_manager_status_t original_status = race_status;
    float original_target = target_straightline_speed_mps;
    float original_current = current_straightline_speed_mps;

    // Warm-up run to bring the instruction stack into IRAM cache
    race_manager_cb(NULL);

    // Main measurement loop
    for (int i = 0; i < 1000; i++) {
        uint32_t start_cycles = esp_cpu_get_cycle_count();
        
        // Execute the full sensor, PID, and logic stack
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

    // Final mathematical calculations
    uint32_t avg_cycles = (uint32_t)(total_cycles / 1000);

    float avg_us = (float)avg_cycles / cycles_per_us;
    float min_us = (float)min_cycles / cycles_per_us;
    float max_us = (float)max_cycles / cycles_per_us;

    RAVEN_LOGI(TAG, "--- Race Manager Callback Benchmark (1000 runs) ---");
    RAVEN_LOGI(TAG, "CPU Clock:    %lu MHz", cycles_per_us);
    RAVEN_LOGI(TAG, "Average Time: %.3f us (%lu cycles)", avg_us, avg_cycles);
    RAVEN_LOGI(TAG, "Min Time:     %.3f us (%lu cycles)", min_us, min_cycles);
    RAVEN_LOGI(TAG, "Max Time:     %.3f us (%lu cycles)", max_us, max_cycles);
    
    // Also send a summary back via Bluetooth for easy viewing
    raven_comm_send_message(TAG, "CB Benchmark - Avg: %.1fus | Max: %.1fus", avg_us, max_us);
}