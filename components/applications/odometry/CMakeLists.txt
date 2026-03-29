idf_component_register(SRCS "odometry.c"
                       INCLUDE_DIRS "include"
                       REQUIRES raven_comm encoder
)