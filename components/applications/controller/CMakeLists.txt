idf_component_register(
    SRCS "controller.c" "controller_tuning.c" "controller_commands.c" "controller_characterization.c"
    INCLUDE_DIRS "include"
    REQUIRES pid
    PRIV_REQUIRES esp_timer raven_log raven_comm battery_sensor odometry motor
)