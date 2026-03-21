#pragma once

#include <stdint.h>
#include <stdbool.h>

#define BLE_DEVICE_NAME "Raven PRL-1"

typedef void (*ble_receive_message_cb_t)(uint8_t *data, uint16_t size);

void ble_manager_init(void);
void ble_manager_send_message(uint8_t *data, uint16_t size);
void ble_manager_receive_callback(ble_receive_message_cb_t callback);