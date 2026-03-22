idf_component_register(
    SRCS "battery_sensor.c"
    INCLUDE_DIRS "include"
    REQUIRES raven_log raven_comm pinout
    PRIV_REQUIRES esp_adc
)