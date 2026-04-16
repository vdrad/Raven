idf_component_register(
    SRCS "encoder.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_driver_pcnt esp_driver_gpio raven_configs raven_log raven_comm
)