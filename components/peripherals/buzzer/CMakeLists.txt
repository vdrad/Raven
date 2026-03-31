idf_component_register(
    SRCS "buzzer.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES driver pinout raven_log raven_comm
)