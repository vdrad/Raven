/**
 * @file line_config.h
 * @brief Shared configuration macros for the line sensing modules.
 */

#pragma once

/* ========================================================================== */
/* MACROS & ENUMS                                                             */
/* ========================================================================== */

#define NUMBER_OF_LINE_SENSORS      15
#define NUMBER_OF_FRONTAL_SENSORS   11
#define NUMBER_OF_MARKER_SENSORS    (NUMBER_OF_LINE_SENSORS - NUMBER_OF_FRONTAL_SENSORS)

/**
 * @brief Mapping of the AD7490 channels to physical robot sensors.
 */
typedef enum {
    LM1     = 0,                    /**< Left Marker 1 (Inner) */
    LM0     = 1,                    /**< Left Marker 0 (Outer) */
    LS0     = 2,                    /**< Central Line Sensor 0 (Leftmost) */
    LS1     = 3,                    /**< Central Line Sensor 1 */
    LS2     = 4,                    /**< Central Line Sensor 2 */
    LS3     = 5,                    /**< Central Line Sensor 3 */
    LS4     = 6,                    /**< Central Line Sensor 4 */
    LS5     = 7,                    /**< Central Line Sensor 5 (Center) */
    LS6     = 8,                    /**< Central Line Sensor 6 */
    LS7     = 9,                    /**< Central Line Sensor 7 */
    LS8     = 10,                   /**< Central Line Sensor 8 */
    LS9     = 11,                   /**< Central Line Sensor 9 */
    LS10    = 12,                  /**< Central Line Sensor 10 (Rightmost) */
    RM1     = 13,                   /**< Right Marker 1 (Inner) */
    RM0     = 14                    /**< Right Marker 0 (Outer) */
} line_sensor_index_t;