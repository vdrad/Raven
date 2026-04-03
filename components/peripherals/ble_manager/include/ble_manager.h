/**
 * @file ble_manager.h
 * @brief Bluetooth Low Energy (BLE) Manager using the NimBLE stack.
 *
 * This module sets up a custom BLE server acting like a Serial Port Profile (SPP).
 * It handles advertising, connections, disconnections, and provides an interface 
 * for sending and receiving raw byte arrays to/from connected clients.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#define BLE_DEVICE_NAME "Raven PRL-1"

/**
 * @brief Callback signature for handling incoming BLE payloads.
 * @param data Pointer to the received payload buffer.
 * @param size Number of bytes received.
 */
typedef void (*ble_receive_message_cb_t)(uint8_t *data, uint16_t size);

/**
 * @brief Initializes the BLE peripheral, sets up GATT/GAP, and starts the NimBLE task.
 */
void ble_manager_init(void);

/**
 * @brief Sends a message to all connected and subscribed BLE clients.
 * * @param data Pointer to the payload buffer.
 * @param size Number of bytes to send.
 */
void ble_manager_send_message(uint8_t *data, uint16_t size);

/**
 * @brief Registers a callback function to handle incoming BLE messages.
 * * @param callback Function pointer to the user's data processing function.
 */
void ble_manager_receive_callback(ble_receive_message_cb_t callback);

/**
 * @brief Retrieves the current BLE connection status.
 * * @return true if at least one client is connected, false otherwise.
 */
bool ble_manager_get_connection_status(void);