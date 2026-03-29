idf_component_register(SRCS "DRV8874.c"
                       INCLUDE_DIRS "include"
                       REQUIRES driver pinout raven_log raven_comm bdc_motor
)