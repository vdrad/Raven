idf_component_register(
    SRCS "ICM45686.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES driver esp_hw_support pinout raven_log raven_comm 
)