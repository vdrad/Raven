idf_component_register(
    SRCS "notifications.c"
    INCLUDE_DIRS "include"
    REQUIRES raven_log rgb_led buzzer
)