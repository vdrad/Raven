idf_component_register(
    SRCS "line_markers.c" "line_position.c" "line_commands.c" "line_calibration.c" "line_reading.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_timer nvs_flash esp_driver_gpio pinout raven_log raven_comm AD7490
)