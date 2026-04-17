idf_component_register(
    SRCS "race_manager.c" "race_manager_commands.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_timer esp_hw_support raven_log raven_comm line_reading odometry controller motor robot_telemetry
)