idf_component_register(
    SRCS "state_machine.c"
    INCLUDE_DIRS "include"
    REQUIRES raven_log raven_comm ble_manager peripheral_validation rgb_led buzzer
)