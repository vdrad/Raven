idf_component_register(SRCS "buzzer.c"
                       INCLUDE_DIRS "include"
                       REQUIRES driver pinout raven_log raven_comm
)