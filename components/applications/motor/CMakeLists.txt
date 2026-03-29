idf_component_register(SRCS "motor.c"
                       INCLUDE_DIRS "include"
                       REQUIRES pinout raven_log raven_comm DRV8874 bdc_motor
)