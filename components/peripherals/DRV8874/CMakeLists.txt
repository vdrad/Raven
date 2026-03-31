idf_component_register(
    SRCS "DRV8874.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES driver raven_log raven_comm
)