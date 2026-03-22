idf_component_register(
    SRCS "peripheral_validation.c"
    INCLUDE_DIRS "include"
    REQUIRES raven_log raven_comm rgb_led buzzer
)