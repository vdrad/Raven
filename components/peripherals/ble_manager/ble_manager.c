/**
 * @file ble_manager.c
 * @brief Bluetooth Low Energy (BLE) Manager using the NimBLE stack.
 * * This module sets up a custom BLE server acting like a Serial Port Profile (SPP).
 * It handles advertising, connections, disconnections, and provides an interface 
 * for sending and receiving raw byte arrays to/from connected clients.
 */

#include "ble_manager.h"
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "raven_log.h"

#define TAG "BLE_MANAGER"
static bool initialized = false;

/* --- BLE Custom UUIDs --- */
// Primary Service UUID
#define BLE_SVC_SPP_UUID16  0xABF0
// Characteristic UUID (Read, Write, Notify)
#define BLE_CHR_SPP_UUID16  0xABF1

/* --- BLE Configuration & State Variables --- */
static uint8_t own_addr_type;                            // Stores the device's own BLE address type
static uint16_t ble_spp_svc_gatt_read_val_handle;        // Handle for the SPP characteristic
static bool conn_handle_subs[CONFIG_BT_NIMBLE_MAX_CONNECTIONS + 1]; // Tracks which connections are subscribed to notifications

// Pointer to the user-defined callback function for incoming data
static ble_receive_message_cb_t g_reception_callback = NULL;

/* --- Function Prototypes --- */
static void ble_spp_server_advertise(void);
static int ble_spp_server_gap_event(struct ble_gap_event *event, void *arg);


/**
 * @brief Registers a callback function to handle incoming BLE messages.
 * * @param callback Function pointer to the user's data processing function.
 */
void ble_manager_receive_callback(ble_receive_message_cb_t callback) {
    g_reception_callback = callback;
}


/**
 * @brief Sends a message to all connected and subscribed BLE clients.
 * * @param data Pointer to the payload buffer.
 * @param size Number of bytes to send.
 */
void ble_manager_send_message(uint8_t *data, uint16_t size) {
    if (size == 0 || data == NULL) return;

    // Iterate through all possible connection handles
    for (int i = 0; i <= CONFIG_BT_NIMBLE_MAX_CONNECTIONS; i++) {
        // Only send if this specific connection has subscribed to notifications
        if (conn_handle_subs[i]) {
            // Allocate a NimBLE memory buffer (mbuf) and copy the data into it
            struct os_mbuf *txom = ble_hs_mbuf_from_flat(data, size);
            
            // Send the notification to the client
            int rc = ble_gatts_notify_custom(i, ble_spp_svc_gatt_read_val_handle, txom);
            if (rc != 0) {
                ESP_LOGE(TAG, "Error sending notification: %d", rc);
            }
        }
    }
}


/**
 * @brief GATT access callback. Handles read/write requests from the BLE client.
 * * @param conn_handle Connection handle of the client making the request.
 * @param attr_handle Attribute handle being accessed.
 * @param ctxt        Context containing the operation type and data buffer.
 * @param arg         Optional argument passed during characteristic registration.
 * @return 0 on success, or a BLE_ATT_ERR code on failure.
 */
static int ble_svc_gatt_handler(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_WRITE_CHR: {
            // Retrieve the length of the incoming packet
            uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
            
            if (len > 0) {
                // Allocate a temporary buffer to hold the incoming data
                uint8_t *data = (uint8_t *)malloc(len);
                if (data != NULL) {
                    // Extract data from the mbuf into our linear buffer
                    os_mbuf_copydata(ctxt->om, 0, len, data);
                    
                    // Trigger the upper-layer callback if one is registered
                    if (g_reception_callback != NULL) {
                        g_reception_callback(data, len);
                    }
                    
                    // Free the temporary buffer to prevent memory leaks
                    free(data);
                }
            }
            break;
        }
        default:
            // This handler currently ignores READ operations
            break;
    }
    return 0;
}


/**
 * @brief Definition of the Custom GATT Service and its characteristics.
 */
static const struct ble_gatt_svc_def new_ble_svc_gatt_defs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(BLE_SVC_SPP_UUID16),
        .characteristics = (struct ble_gatt_chr_def[]) { 
            {
                .uuid = BLE_UUID16_DECLARE(BLE_CHR_SPP_UUID16),
                .access_cb = ble_svc_gatt_handler,
                .val_handle = &ble_spp_svc_gatt_read_val_handle,
                // Allow the client to Read, Write, and Subscribe to Notifications
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
            }, {
                0, /* No more characteristics */
            }
        },
    },
    { 0, /* No more services */ },
};


/**
 * @brief Handles GAP (Generic Access Profile) events like connect/disconnect.
 * * @param event Pointer to the GAP event structure.
 * @param arg   Optional user argument.
 * @return 0 on success.
 */
static int ble_spp_server_gap_event(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            // If connection failed, resume advertising
            if (event->connect.status != 0) {
                ble_spp_server_advertise();
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            // Clear the subscription status for the disconnected client
            conn_handle_subs[event->disconnect.conn.conn_handle] = false;
            // Resume advertising so other devices can connect
            ble_spp_server_advertise();
            break;

        case BLE_GAP_EVENT_SUBSCRIBE:
            // Update subscription array when a client enables/disables notifications
            conn_handle_subs[event->subscribe.conn_handle] = event->subscribe.cur_notify;
            break;
    }
    return 0;
}


/**
 * @brief Configures advertising data and starts broadcasting.
 */
static void ble_spp_server_advertise(void) {
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    int rc;

    // 1. Configure Advertisement Fields (Payload)
    memset(&fields, 0, sizeof fields);
    
    // Set flags to General Discoverable and indicate BR/EDR (Classic BT) is not supported
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    
    // Include the transmission power level in the advertisement
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    // Attach the device name to the advertisement payload
    const char *name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    // Apply the configured fields to the stack
    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) return;

    // 2. Configure Advertisement Parameters (How it broadcasts)
    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND; // Undirected connectable mode
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN; // General discoverable mode
    
    // Start advertising indefinitely (BLE_HS_FOREVER)
    ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_spp_server_gap_event, NULL);
}


/**
 * @brief Callback triggered when the NimBLE host and controller are synchronized.
 */
static void ble_spp_server_on_sync(void) {
    // Determine the best address type automatically
    ble_hs_id_infer_auto(0, &own_addr_type);
    
    // Start broadcasting once the stack is fully synced
    ble_spp_server_advertise();
}


/**
 * @brief FreeRTOS task that continuously runs the NimBLE host stack.
 * * @param param Task parameter (unused).
 */
static void ble_spp_server_host_task(void *param) {
    ESP_LOGI(TAG, "BLE Host Task Started");
    
    // This function blocks and runs the BLE event loop
    nimble_port_run();
    
    // If the loop exits, clean up the FreeRTOS resources
    nimble_port_freertos_deinit();
}


/**
 * @brief Initializes the BLE peripheral, sets up GATT/GAP, and starts the NimBLE task.
 */
void ble_manager_init(void) {
    if (initialized) return;

    // 1. Initialize NVS (Non-Volatile Storage) - Required by NimBLE to store bonding/pairing data
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Initialize the NimBLE port
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        RAVEN_LOGE(TAG, "Failed to initialize NimBLE: %d", ret);
        return;
    }

    // Initialize the subscription tracking array
    for (int i = 0; i <= CONFIG_BT_NIMBLE_MAX_CONNECTIONS; i++) {
        conn_handle_subs[i] = false;
    }

    // 3. Configure NimBLE Callbacks and Services
    ble_hs_cfg.sync_cb = ble_spp_server_on_sync;
    
    ble_svc_gap_init();
    ble_svc_gatt_init();
    
    // Register custom GATT services
    ble_gatts_count_cfg(new_ble_svc_gatt_defs);
    ble_gatts_add_svcs(new_ble_svc_gatt_defs);
    
    // Set the device name that will appear in scans
    ble_svc_gap_device_name_set(BLE_DEVICE_NAME);

    // 4. Start the NimBLE host task pinned to FreeRTOS
    nimble_port_freertos_init(ble_spp_server_host_task);

    RAVEN_LOGI(TAG, "Initialized successfully.");
    initialized = true;
}