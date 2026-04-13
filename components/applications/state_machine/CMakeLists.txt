idf_component_register(
    SRCS "state_machine.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES raven_log raven_comm ble_manager peripheral_validation rgb_led buzzer battery_sensor ICM45686 motor odometry controller line_reading
)