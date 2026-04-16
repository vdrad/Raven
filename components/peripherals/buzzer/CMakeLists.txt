idf_component_register(
    SRCS "buzzer.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES driver raven_configs raven_log raven_comm
)