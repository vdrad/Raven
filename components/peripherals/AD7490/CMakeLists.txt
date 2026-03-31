idf_component_register(
    SRCS "AD7490.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_driver_spi esp_driver_gpio esp_hw_support pinout raven_log raven_comm 
)