idf_component_register(
    SRCS "controller.c" "controller_tuning.c" "controller_commands.c" "controller_pid.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_timer raven_log raven_comm battery_sensor odometry motor
)