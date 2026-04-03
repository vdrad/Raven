idf_component_register(
    SRCS "motor.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES pinout raven_log raven_comm DRV8874 battery_sensor
)