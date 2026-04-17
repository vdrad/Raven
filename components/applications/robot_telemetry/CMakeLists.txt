idf_component_register(
    SRCS "robot_telemetry.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_timer raven_log raven_comm battery_sensor line_reading odometry controller race_manager
)