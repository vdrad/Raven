idf_component_register(
    SRCS "odometry.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_timer raven_log raven_comm encoder ICM45686
)