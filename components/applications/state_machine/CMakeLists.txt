idf_component_register(
    SRCS "state_machine.c"
    INCLUDE_DIRS "include"
    REQUIRES raven_log raven_comm rgb_led
)