idf_component_register(
    SRCS "ICM45686.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES driver esp_hw_support raven_configs raven_log raven_comm 
)