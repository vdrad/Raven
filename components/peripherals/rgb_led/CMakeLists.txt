idf_component_register(
    SRCS "rgb_led.c" "led_strip_encoder.c"
    INCLUDE_DIRS "include"
    REQUIRES pinout raven_log raven_comm
    PRIV_REQUIRES esp_driver_rmt
)