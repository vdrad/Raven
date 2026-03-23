idf_component_register(
    SRCS "AD7490.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_spi raven_log raven_comm pinout
    PRIV_REQUIRES esp_driver_gpio esp_hw_support
)