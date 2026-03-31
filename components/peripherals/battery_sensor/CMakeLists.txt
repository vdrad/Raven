idf_component_register(
    SRCS "battery_sensor.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_adc raven_log raven_comm pinout
)