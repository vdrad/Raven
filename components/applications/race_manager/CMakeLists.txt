idf_component_register(
    SRCS "race_manager.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_timer raven_log raven_comm line_reading odometry controller motor
)