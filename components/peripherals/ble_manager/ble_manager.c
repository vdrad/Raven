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

// Custom UUIDs
#define BLE_SVC_SPP_UUID16  0xABF0
#define BLE_CHR_SPP_UUID16  0xABF1

// BLE CONFIG
static uint8_t own_addr_type;
static uint16_t ble_spp_svc_gatt_read_val_handle;
static bool conn_handle_subs[CONFIG_BT_NIMBLE_MAX_CONNECTIONS + 1];

static ble_receive_message_cb_t g_reception_callback = NULL;

// FUNCTION PROTOTYPES
static void ble_spp_server_advertise(void);
static int ble_spp_server_gap_event(struct ble_gap_event *event, void *arg);

void ble_manager_receive_callback(ble_receive_message_cb_t callback) {
    g_reception_callback = callback;
}

void ble_manager_send_message(uint8_t *data, uint16_t size) {
    if (size == 0 || data == NULL) return;

    for (int i = 0; i <= CONFIG_BT_NIMBLE_MAX_CONNECTIONS; i++) {
        if (conn_handle_subs[i]) {
            struct os_mbuf *txom = ble_hs_mbuf_from_flat(data, size);
            int rc = ble_gatts_notify_custom(i, ble_spp_svc_gatt_read_val_handle, txom);
            if (rc != 0) ESP_LOGE(TAG, "Error sending notification: %d", rc);
        }
    }
}

static int ble_svc_gatt_handler(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_WRITE_CHR: {
            uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
            if (len > 0) {
                uint8_t *data = (uint8_t *)malloc(len);
                if (data != NULL) {
                    os_mbuf_copydata(ctxt->om, 0, len, data);
                    
                    if (g_reception_callback != NULL) g_reception_callback(data, len);
                    free(data);
                }
            }
            break;
        }
        default:
            break;
    }
    return 0;
}

static const struct ble_gatt_svc_def new_ble_svc_gatt_defs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(BLE_SVC_SPP_UUID16),
        .characteristics = (struct ble_gatt_chr_def[]) { 
            {
                .uuid = BLE_UUID16_DECLARE(BLE_CHR_SPP_UUID16),
                .access_cb = ble_svc_gatt_handler,
                .val_handle = &ble_spp_svc_gatt_read_val_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
            }, {
                0, /* No more characteristics */
            }
        },
    },
    { 0, /* No more services */ },
};

static int ble_spp_server_gap_event(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status != 0) {
                ble_spp_server_advertise();
            }
            break;
        case BLE_GAP_EVENT_DISCONNECT:
            conn_handle_subs[event->disconnect.conn.conn_handle] = false;
            ble_spp_server_advertise();
            break;
        case BLE_GAP_EVENT_SUBSCRIBE:
            conn_handle_subs[event->subscribe.conn_handle] = event->subscribe.cur_notify;
            break;
    }
    return 0;
}

static void ble_spp_server_advertise(void) {
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    int rc;

    memset(&fields, 0, sizeof fields);
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    const char *name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) return;

    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_spp_server_gap_event, NULL);
}

static void ble_spp_server_on_sync(void) {
    ble_hs_id_infer_auto(0, &own_addr_type);
    ble_spp_server_advertise();
}

static void ble_spp_server_host_task(void *param) {
    ESP_LOGI(TAG, "BLE Host Task Started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void ble_manager_init(void) {
    if (initialized) return;

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ret = nimble_port_init();
    if (ret != ESP_OK) {
        RAVEN_LOGE(TAG, "Failed to initialize NimBLE: %d", ret);
        return;
    }

    for (int i = 0; i <= CONFIG_BT_NIMBLE_MAX_CONNECTIONS; i++) {
        conn_handle_subs[i] = false;
    }

    ble_hs_cfg.sync_cb = ble_spp_server_on_sync;
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatts_count_cfg(new_ble_svc_gatt_defs);
    ble_gatts_add_svcs(new_ble_svc_gatt_defs);
    ble_svc_gap_device_name_set(BLE_DEVICE_NAME);

    nimble_port_freertos_init(ble_spp_server_host_task);

    RAVEN_LOGI(TAG, "Initialized successfully.");
    initialized = true;
}