idf_component_register(SRCS "DRV8874.c"
                       INCLUDE_DIRS "include"
                       REQUIRES driver raven_log raven_comm bdc_motor
)