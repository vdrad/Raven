idf_component_register(
    SRCS "motor_characterization.c" "motor.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_timer pinout raven_log raven_comm DRV8874 battery_sensor odometry
)