idf_component_register(
    SRCS "ble_manager.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES nvs_flash bt raven_log
)