idf_component_register(
    SRCS "robot_manager.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES raven_log raven_comm
)