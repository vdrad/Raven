idf_component_register(
    SRCS "state_machine.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES nvs_flash raven_log raven_comm ble_manager peripheral_validation battery_sensor ICM45686 motor odometry controller line_reading notifications race_manager
)