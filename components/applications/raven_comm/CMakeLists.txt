idf_component_register(
    SRCS "raven_comm.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES raven_log ble_manager peripheral_validation
)