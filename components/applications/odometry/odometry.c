#include "odometry.h"

#include <stdbool.h>
#include <math.h>

#include "raven_comm.h"
#include "encoder.h"

#define TAG "ODM"
static bool initialized = false;

#define ENCODER_GEAR_TEETH  13.0f
#define WHEEL_GEAR_TEETH    42.0f
#define WHEEL_DIAMETER_MM   25.0f

#define PULSES_PER_ENCODER_REVOLUTION   2048
#define GEAR_RATIO                      (WHEEL_GEAR_TEETH / ENCODER_GEAR_TEETH)
#define PULSES_PER_WHEEL_REVOLUTION     (PULSES_PER_ENCODER_REVOLUTION * GEAR_RATIO)

void odometry_init(void) {
    if (initialized) return;

    encoder_init();

    initialized = true;
    raven_comm_send_message(TAG, "Initialized successfully.");
}

float odometry_get_travelled_distance(bool milimeters) {
    if (!initialized) {
        raven_comm_send_message(TAG, "Not initialized!");
        return 0.0f;
    }

    int left_count, right_count;
    encoder_get_count(ENCODER_LEFT,  &left_count);
    encoder_get_count(ENCODER_RIGHT, &right_count);

    float distance_travelled;
    distance_travelled = (float)(left_count + right_count) / 2.0f;
    distance_travelled /= PULSES_PER_WHEEL_REVOLUTION;
    distance_travelled *= (M_PI * WHEEL_DIAMETER_MM);

    if (milimeters) return distance_travelled;
    return distance_travelled / 1000.0f;
}

