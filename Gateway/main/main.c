/* main.c - Application main entry point */

/*
 * Copyright (c) 2018 Espressif Systems (Shanghai) PTE LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdlib.h> 
#include <string.h>
#include <stdint.h>

#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#include "esp_intr_alloc.h"
#include "esp32/rom/uart.h"

#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_config_model_api.h"

#include "ble_mesh_example_init.h"
#include "ble_mesh_example_nvs.h"
// #include "board.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

#define LED_GPIO GPIO_NUM_2

#define EX_UART_NUM UART_NUM_0

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)

static bool led_state = false;
static intr_handle_t handle_console;
static volatile uint8_t rxFlag = 0;

#define TAG "EXAMPLE"
#define MASTER_TAG "MASTER"

#define CID_ESP             0x02E5

#define PROV_OWN_ADDR       0x0001

#define MSG_SEND_TTL        3
#define MSG_SEND_REL        false
#define MSG_TIMEOUT         0
#define MSG_ROLE            ROLE_PROVISIONER

#define COMP_DATA_PAGE_0    0x00

#define APP_KEY_IDX         0x0000
#define APP_KEY_OCTET       0x12

#define COMP_DATA_1_OCTET(msg, offset)      (msg[offset])
#define COMP_DATA_2_OCTET(msg, offset)      (msg[offset + 1] << 8 | msg[offset])

#define ESP_BLE_MESH_VND_MODEL_ID_CLIENT    0x0000
#define ESP_BLE_MESH_VND_MODEL_ID_SERVER    0x0001

#define ESP_BLE_MESH_VND_MODEL_OP_SEND      ESP_BLE_MESH_MODEL_OP_3(0x00, CID_ESP)
#define ESP_BLE_MESH_VND_MODEL_OP_STATUS    ESP_BLE_MESH_MODEL_OP_3(0x01, CID_ESP)

static uint8_t dev_uuid[ESP_BLE_MESH_OCTET16_LEN];

static void IRAM_ATTR uart_intr_handle(void *arg);
static void delay_seconds(uint32_t seconds);

typedef enum {
    REQUEST_FRAME           = 1u,
    ACTIVATE_ALARM_FRAME    = 2u,
    DEACTIVATE_ALARM_FRAME  = 3u
}Gateway_SendType;

static struct example_info_store {
    uint16_t server_addr;   /* Vendor server unicast address */
    uint16_t vnd_tid;       /* TID contained in the vendor message */
    Gateway_SendType frameType;      /* Frame type to be sent */
    uint16_t data;          
    uint8_t *dataArr;       /* Data received */
    uint8_t node_num; 
    uint16_t first_addr;
} store = {
    .server_addr = ESP_BLE_MESH_ADDR_UNASSIGNED,
    .vnd_tid = 0,
    .frameType = REQUEST_FRAME,
    .data = 0,
    .node_num = 0,
    .dataArr = NULL,
};

/**
 * Pointer to data array received. The order of data:
 * Data_arr[0]: room number
 * Data_arr[1]: battery percentage
 * Data_arr[2]: Temperature value
 * Data_arr[3]: Smoke value
 * Data_arr[4]: Temperature alarm flag. Value is 1 if temperature is too high => fire
 * Data_arr[5]: Flame sensor alarm flag. Value is 1 if detect flame => fire
 * Data_arr[6]: Smoke alarm flag. Value is 1 if detect smoke is too high => fire
 * Data_arr[7]: General fire alarm flag. Value is 1 if detect any sign of fire from sensors
 */

static nvs_handle_t NVS_HANDLE;
static const char * NVS_KEY = "vendor_client";

static struct esp_ble_mesh_key {
    uint16_t net_idx;
    uint16_t app_idx;
    uint8_t  app_key[ESP_BLE_MESH_OCTET16_LEN];
} prov_key;

static esp_ble_mesh_cfg_srv_t config_server = {
    .beacon = ESP_BLE_MESH_BEACON_DISABLED,
#if defined(CONFIG_BLE_MESH_FRIEND)
    .friend_state = ESP_BLE_MESH_FRIEND_ENABLED,
#else
    .friend_state = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
#endif
    .default_ttl = 7,
    /* 3 transmissions with 20ms interval */
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
};

static esp_ble_mesh_client_t config_client;

static const esp_ble_mesh_client_op_pair_t vnd_op_pair[] = {
    { ESP_BLE_MESH_VND_MODEL_OP_SEND, ESP_BLE_MESH_VND_MODEL_OP_STATUS },
};

static esp_ble_mesh_client_t vendor_client = {
    .op_pair_size = ARRAY_SIZE(vnd_op_pair),
    .op_pair = vnd_op_pair,
};

static esp_ble_mesh_model_op_t vnd_op[] = {
    ESP_BLE_MESH_MODEL_OP(ESP_BLE_MESH_VND_MODEL_OP_STATUS, 2),
    ESP_BLE_MESH_MODEL_OP_END,
};

static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
    ESP_BLE_MESH_MODEL_CFG_CLI(&config_client),
};

static esp_ble_mesh_model_t vnd_models[] = {
    ESP_BLE_MESH_VENDOR_MODEL(CID_ESP, ESP_BLE_MESH_VND_MODEL_ID_CLIENT,
    vnd_op, NULL, &vendor_client),
};

static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, vnd_models),
};

static esp_ble_mesh_comp_t composition = {
    .cid = CID_ESP,
    .elements = elements,
    .element_count = ARRAY_SIZE(elements),
};

static esp_ble_mesh_prov_t provision = {
    .prov_uuid          = dev_uuid,
    .prov_unicast_addr  = PROV_OWN_ADDR,
    .prov_start_address = 0x0005,
};

static void mesh_example_info_store(void)
{
    ble_mesh_nvs_store(NVS_HANDLE, NVS_KEY, &store, sizeof(store));
}

static void mesh_example_info_restore(void)
{
    esp_err_t err = ESP_OK;
    bool exist = false;

    err = ble_mesh_nvs_restore(NVS_HANDLE, NVS_KEY, &store, sizeof(store), &exist);
    if (err != ESP_OK) {
        return;
    }

    if (exist) {
        // ESP_LOGI(TAG, "Restore, server_addr 0x%04x", store.server_addr);
    }
}

static void example_ble_mesh_set_msg_common(esp_ble_mesh_client_common_param_t *common,
                                            esp_ble_mesh_node_t *node,
                                            esp_ble_mesh_model_t *model, uint32_t opcode)
{
    common->opcode = opcode;
    common->model = model;
    common->ctx.net_idx = prov_key.net_idx;
    common->ctx.app_idx = prov_key.app_idx;
    common->ctx.addr = node->unicast_addr;
    common->ctx.send_ttl = MSG_SEND_TTL;
    common->ctx.send_rel = MSG_SEND_REL;
    common->msg_timeout = MSG_TIMEOUT;
    common->msg_role = MSG_ROLE;
}

static esp_err_t prov_complete(uint16_t node_index, const esp_ble_mesh_octet16_t uuid,
                               uint16_t primary_addr, uint8_t element_num, uint16_t net_idx)
{
    esp_ble_mesh_client_common_param_t common = {0};
    esp_ble_mesh_cfg_client_get_state_t get = {0};
    esp_ble_mesh_node_t *node = NULL;
    char name[10] = {'\0'};
    esp_err_t err;

    // ESP_LOGI(TAG, "node_index %u, primary_addr 0x%04x, element_num %u, net_idx 0x%03x",
    //     node_index, primary_addr, element_num, net_idx);
    // ESP_LOG_BUFFER_HEX("uuid", uuid, ESP_BLE_MESH_OCTET16_LEN);

    store.server_addr = primary_addr;

    store.node_num++;

    store.first_addr = primary_addr;
    // Change the server unicast address for other nodes to join
    // store.number_of_nodes++;
    
    mesh_example_info_store(); /* Store proper mesh example info */

    sprintf(name, "%s%02x", "NODE-", node_index);
    err = esp_ble_mesh_provisioner_set_node_name(node_index, name);
    if (err != ESP_OK) {
        // ESP_LOGE(TAG, "Failed to set node name");
        return ESP_FAIL;
    }

    node = esp_ble_mesh_provisioner_get_node_with_addr(primary_addr);
    if (node == NULL) {
        // ESP_LOGE(TAG, "Failed to get node 0x%04x info", primary_addr);
        return ESP_FAIL;
    }

    example_ble_mesh_set_msg_common(&common, node, config_client.model, ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_GET);
    get.comp_data_get.page = COMP_DATA_PAGE_0;
    err = esp_ble_mesh_config_client_get_state(&common, &get);
    if (err != ESP_OK) {
        // ESP_LOGE(TAG, "Failed to send Config Composition Data Get");
        return ESP_FAIL;
    }

    return ESP_OK;
}

static void recv_unprov_adv_pkt(uint8_t dev_uuid[ESP_BLE_MESH_OCTET16_LEN], uint8_t addr[BD_ADDR_LEN],
                                esp_ble_mesh_addr_type_t addr_type, uint16_t oob_info,
                                uint8_t adv_type, esp_ble_mesh_prov_bearer_t bearer)
{
    esp_ble_mesh_unprov_dev_add_t add_dev = {0};
    esp_err_t err;

    /* Due to the API esp_ble_mesh_provisioner_set_dev_uuid_match, Provisioner will only
     * use this callback to report the devices, whose device UUID starts with 0xdd & 0xdd,
     * to the application layer.
     */

    // ESP_LOG_BUFFER_HEX("Device address", addr, BD_ADDR_LEN);
    // ESP_LOGI(TAG, "Address type 0x%02x, adv type 0x%02x", addr_type, adv_type);
    // ESP_LOG_BUFFER_HEX("Device UUID", dev_uuid, ESP_BLE_MESH_OCTET16_LEN);
    // ESP_LOGI(TAG, "oob info 0x%04x, bearer %s", oob_info, (bearer & ESP_BLE_MESH_PROV_ADV) ? "PB-ADV" : "PB-GATT");

    memcpy(add_dev.addr, addr, BD_ADDR_LEN);
    add_dev.addr_type = (uint8_t)addr_type;
    memcpy(add_dev.uuid, dev_uuid, ESP_BLE_MESH_OCTET16_LEN);
    add_dev.oob_info = oob_info;
    add_dev.bearer = (uint8_t)bearer;
    /* Note: If unprovisioned device adv packets have not been received, we should not add
             device with ADD_DEV_START_PROV_NOW_FLAG set. */
    err = esp_ble_mesh_provisioner_add_unprov_dev(&add_dev,
            ADD_DEV_RM_AFTER_PROV_FLAG | ADD_DEV_START_PROV_NOW_FLAG | ADD_DEV_FLUSHABLE_DEV_FLAG);
    if (err != ESP_OK) {
        // ESP_LOGE(TAG, "Failed to start provisioning device");
    }
}

static void example_ble_mesh_provisioning_cb(esp_ble_mesh_prov_cb_event_t event,
                                             esp_ble_mesh_prov_cb_param_t *param)
{
    switch (event) {
    case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
        // ESP_LOGI(TAG, "ESP_BLE_MESH_PROV_REGISTER_COMP_EVT, err_code %d", param->prov_register_comp.err_code);
        mesh_example_info_restore(); /* Restore proper mesh example info */
        break;
    case ESP_BLE_MESH_PROVISIONER_PROV_ENABLE_COMP_EVT:
        // ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_PROV_ENABLE_COMP_EVT, err_code %d", param->provisioner_prov_enable_comp.err_code);
        break;
    case ESP_BLE_MESH_PROVISIONER_PROV_DISABLE_COMP_EVT:
        // ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_PROV_DISABLE_COMP_EVT, err_code %d", param->provisioner_prov_disable_comp.err_code);
        break;
    case ESP_BLE_MESH_PROVISIONER_RECV_UNPROV_ADV_PKT_EVT:
        // ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_RECV_UNPROV_ADV_PKT_EVT");
        recv_unprov_adv_pkt(param->provisioner_recv_unprov_adv_pkt.dev_uuid, param->provisioner_recv_unprov_adv_pkt.addr,
                            param->provisioner_recv_unprov_adv_pkt.addr_type, param->provisioner_recv_unprov_adv_pkt.oob_info,
                            param->provisioner_recv_unprov_adv_pkt.adv_type, param->provisioner_recv_unprov_adv_pkt.bearer);
        break;
    case ESP_BLE_MESH_PROVISIONER_PROV_LINK_OPEN_EVT:
        // ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_PROV_LINK_OPEN_EVT, bearer %s",
        //     param->provisioner_prov_link_open.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_PROVISIONER_PROV_LINK_CLOSE_EVT:
        // ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_PROV_LINK_CLOSE_EVT, bearer %s, reason 0x%02x",
        //     param->provisioner_prov_link_close.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT", param->provisioner_prov_link_close.reason);
        break;
    case ESP_BLE_MESH_PROVISIONER_PROV_COMPLETE_EVT:
        prov_complete(param->provisioner_prov_complete.node_idx, param->provisioner_prov_complete.device_uuid,
                      param->provisioner_prov_complete.unicast_addr, param->provisioner_prov_complete.element_num,
                      param->provisioner_prov_complete.netkey_idx);
        break;
    case ESP_BLE_MESH_PROVISIONER_ADD_UNPROV_DEV_COMP_EVT:
        // ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_ADD_UNPROV_DEV_COMP_EVT, err_code %d", param->provisioner_add_unprov_dev_comp.err_code);
        break;
    case ESP_BLE_MESH_PROVISIONER_SET_DEV_UUID_MATCH_COMP_EVT:
        // ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_SET_DEV_UUID_MATCH_COMP_EVT, err_code %d", param->provisioner_set_dev_uuid_match_comp.err_code);
        break;
    case ESP_BLE_MESH_PROVISIONER_SET_NODE_NAME_COMP_EVT:
        // ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_SET_NODE_NAME_COMP_EVT, err_code %d", param->provisioner_set_node_name_comp.err_code);
        if (param->provisioner_set_node_name_comp.err_code == 0) {
            const char *name = esp_ble_mesh_provisioner_get_node_name(param->provisioner_set_node_name_comp.node_index);
            if (name) {
                // ESP_LOGI(TAG, "Node %d name %s", param->provisioner_set_node_name_comp.node_index, name);
            }
        }
        break;
    case ESP_BLE_MESH_PROVISIONER_ADD_LOCAL_APP_KEY_COMP_EVT:
        // ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_ADD_LOCAL_APP_KEY_COMP_EVT, err_code %d", param->provisioner_add_app_key_comp.err_code);
        if (param->provisioner_add_app_key_comp.err_code == 0) {
            prov_key.app_idx = param->provisioner_add_app_key_comp.app_idx;
            esp_err_t err = esp_ble_mesh_provisioner_bind_app_key_to_local_model(PROV_OWN_ADDR, prov_key.app_idx,
                    ESP_BLE_MESH_VND_MODEL_ID_CLIENT, CID_ESP);
            if (err != ESP_OK) {
                // ESP_LOGE(TAG, "Failed to bind AppKey to vendor client");
            }
        }
        break;
    case ESP_BLE_MESH_PROVISIONER_BIND_APP_KEY_TO_MODEL_COMP_EVT:
        // ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_BIND_APP_KEY_TO_MODEL_COMP_EVT, err_code %d", param->provisioner_bind_app_key_to_model_comp.err_code);
        break;
    case ESP_BLE_MESH_PROVISIONER_STORE_NODE_COMP_DATA_COMP_EVT:
        // ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_STORE_NODE_COMP_DATA_COMP_EVT, err_code %d", param->provisioner_store_node_comp_data_comp.err_code);
        break;
    default:
        break;
    }
}

static void example_ble_mesh_parse_node_comp_data(const uint8_t *data, uint16_t length)
{
    uint16_t cid, pid, vid, crpl, feat;
    uint16_t loc, model_id, company_id;
    uint8_t nums, numv;
    uint16_t offset;
    int i;

    cid = COMP_DATA_2_OCTET(data, 0);
    pid = COMP_DATA_2_OCTET(data, 2);
    vid = COMP_DATA_2_OCTET(data, 4);
    crpl = COMP_DATA_2_OCTET(data, 6);
    feat = COMP_DATA_2_OCTET(data, 8);
    offset = 10;

    // ESP_LOGI(TAG, "********************** Composition Data Start **********************");
    // ESP_LOGI(TAG, "* CID 0x%04x, PID 0x%04x, VID 0x%04x, CRPL 0x%04x, Features 0x%04x *", cid, pid, vid, crpl, feat);
    for (; offset < length; ) {
        loc = COMP_DATA_2_OCTET(data, offset);
        nums = COMP_DATA_1_OCTET(data, offset + 2);
        numv = COMP_DATA_1_OCTET(data, offset + 3);
        offset += 4;
        // ESP_LOGI(TAG, "* Loc 0x%04x, NumS 0x%02x, NumV 0x%02x *", loc, nums, numv);
        for (i = 0; i < nums; i++) {
            model_id = COMP_DATA_2_OCTET(data, offset);
            // ESP_LOGI(TAG, "* SIG Model ID 0x%04x *", model_id);
            offset += 2;
        }
        for (i = 0; i < numv; i++) {
            company_id = COMP_DATA_2_OCTET(data, offset);
            model_id = COMP_DATA_2_OCTET(data, offset + 2);
            // ESP_LOGI(TAG, "* Vendor Model ID 0x%04x, Company ID 0x%04x *", model_id, company_id);
            offset += 4;
        }
    }
    // ESP_LOGI(TAG, "*********************** Composition Data End ***********************");
}

static void example_ble_mesh_config_client_cb(esp_ble_mesh_cfg_client_cb_event_t event,
                                              esp_ble_mesh_cfg_client_cb_param_t *param)
{
    esp_ble_mesh_client_common_param_t common = {0};
    esp_ble_mesh_cfg_client_set_state_t set = {0};
    esp_ble_mesh_node_t *node = NULL;
    esp_err_t err;

    // ESP_LOGI(TAG, "Config client, err_code %d, event %u, addr 0x%04x, opcode 0x%04x",
    //     param->error_code, event, param->params->ctx.addr, param->params->opcode);

    if (param->error_code) {
        // ESP_LOGE(TAG, "Send config client message failed, opcode 0x%04x", param->params->opcode);
        return;
    }

    node = esp_ble_mesh_provisioner_get_node_with_addr(param->params->ctx.addr);
    if (!node) {
        // ESP_LOGE(TAG, "Failed to get node 0x%04x info", param->params->ctx.addr);
        return;
    }

    switch (event) {
    case ESP_BLE_MESH_CFG_CLIENT_GET_STATE_EVT:
        if (param->params->opcode == ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_GET) {
            // ESP_LOG_BUFFER_HEX("Composition data", param->status_cb.comp_data_status.composition_data->data,
            //     param->status_cb.comp_data_status.composition_data->len);
            example_ble_mesh_parse_node_comp_data(param->status_cb.comp_data_status.composition_data->data,
                param->status_cb.comp_data_status.composition_data->len);
            err = esp_ble_mesh_provisioner_store_node_comp_data(param->params->ctx.addr,
                param->status_cb.comp_data_status.composition_data->data,
                param->status_cb.comp_data_status.composition_data->len);
            if (err != ESP_OK) {
                // ESP_LOGE(TAG, "Failed to store node composition data");
                break;
            }

            example_ble_mesh_set_msg_common(&common, node, config_client.model, ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD);
            set.app_key_add.net_idx = prov_key.net_idx;
            set.app_key_add.app_idx = prov_key.app_idx;
            memcpy(set.app_key_add.app_key, prov_key.app_key, ESP_BLE_MESH_OCTET16_LEN);
            err = esp_ble_mesh_config_client_set_state(&common, &set);
            if (err != ESP_OK) {
                // ESP_LOGE(TAG, "Failed to send Config AppKey Add");
            }
        }
        break;
    case ESP_BLE_MESH_CFG_CLIENT_SET_STATE_EVT:
        if (param->params->opcode == ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD) {
            example_ble_mesh_set_msg_common(&common, node, config_client.model, ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND);
            set.model_app_bind.element_addr = node->unicast_addr;
            set.model_app_bind.model_app_idx = prov_key.app_idx;
            set.model_app_bind.model_id = ESP_BLE_MESH_VND_MODEL_ID_SERVER;
            set.model_app_bind.company_id = CID_ESP;
            err = esp_ble_mesh_config_client_set_state(&common, &set);
            if (err != ESP_OK) {
                // ESP_LOGE(TAG, "Failed to send Config Model App Bind");
            }
        } else if (param->params->opcode == ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND) {
            // ESP_LOGW(TAG, "%s, Provision and config successfully", __func__);
        }
        break;
    case ESP_BLE_MESH_CFG_CLIENT_PUBLISH_EVT:
        if (param->params->opcode == ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_STATUS) {
            // ESP_LOG_BUFFER_HEX("Composition data", param->status_cb.comp_data_status.composition_data->data,
            //     param->status_cb.comp_data_status.composition_data->len);
        }
        break;
    case ESP_BLE_MESH_CFG_CLIENT_TIMEOUT_EVT:
        switch (param->params->opcode) {
        case ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_GET: {
            esp_ble_mesh_cfg_client_get_state_t get = {0};
            example_ble_mesh_set_msg_common(&common, node, config_client.model, ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_GET);
            get.comp_data_get.page = COMP_DATA_PAGE_0;
            err = esp_ble_mesh_config_client_get_state(&common, &get);
            if (err != ESP_OK) {
                // ESP_LOGE(TAG, "Failed to send Config Composition Data Get");
            }
            break;
        }
        case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
            example_ble_mesh_set_msg_common(&common, node, config_client.model, ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD);
            set.app_key_add.net_idx = prov_key.net_idx;
            set.app_key_add.app_idx = prov_key.app_idx;
            memcpy(set.app_key_add.app_key, prov_key.app_key, ESP_BLE_MESH_OCTET16_LEN);
            err = esp_ble_mesh_config_client_set_state(&common, &set);
            if (err != ESP_OK) {
                // ESP_LOGE(TAG, "Failed to send Config AppKey Add");
            }
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND:
            example_ble_mesh_set_msg_common(&common, node, config_client.model, ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND);
            set.model_app_bind.element_addr = node->unicast_addr;
            set.model_app_bind.model_app_idx = prov_key.app_idx;
            set.model_app_bind.model_id = ESP_BLE_MESH_VND_MODEL_ID_SERVER;
            set.model_app_bind.company_id = CID_ESP;
            err = esp_ble_mesh_config_client_set_state(&common, &set);
            if (err != ESP_OK) {
                // ESP_LOGE(TAG, "Failed to send Config Model App Bind");
            }
            break;
        default:
            break;
        }
        break;
    default:
        // ESP_LOGE(TAG, "Invalid config client event %u", event);
        break;
    }
}

void example_ble_mesh_send_vendor_message(Gateway_SendType frameType)
{
    esp_ble_mesh_msg_ctx_t ctx = {0};
    uint32_t opcode;
    esp_err_t err;
    
    ctx.net_idx = prov_key.net_idx;
    ctx.app_idx = prov_key.app_idx;
    // ctx.addr = store.server_addr;
    ctx.addr = 0xFFFF;
    // ctx.addr = store.first_addr;
    ctx.send_ttl = MSG_SEND_TTL;
    ctx.send_rel = MSG_SEND_REL;
    opcode = ESP_BLE_MESH_VND_MODEL_OP_SEND;

    if (frameType == REQUEST_FRAME) {
        store.frameType = REQUEST_FRAME;
    }
    else if (frameType == ACTIVATE_ALARM_FRAME) {
        store.frameType = ACTIVATE_ALARM_FRAME;
    }
    else if (frameType == DEACTIVATE_ALARM_FRAME) {
        store.frameType = DEACTIVATE_ALARM_FRAME;
    }

    // for(int i=0; i<store.node_num; i++) {
        err = esp_ble_mesh_client_model_send_msg(vendor_client.model, &ctx, opcode,
            sizeof(store.frameType), (uint8_t *)&store.frameType, MSG_TIMEOUT, true, MSG_ROLE);
        if (err != ESP_OK) {
            // ESP_LOGE(TAG, "Failed to send vendor message 0x%06x", opcode);
            return;
        }
    //     ctx.addr++;
    // }
    
    mesh_example_info_store(); /* Store proper mesh example info */
}

static void example_ble_mesh_custom_model_cb(esp_ble_mesh_model_cb_event_t event,
                                             esp_ble_mesh_model_cb_param_t *param)
{
    static int64_t start_time;

    switch (event) {
    case ESP_BLE_MESH_MODEL_OPERATION_EVT:
        if (param->model_operation.opcode == ESP_BLE_MESH_VND_MODEL_OP_STATUS) {
            int64_t end_time = esp_timer_get_time();
            // store.data = *(uint16_t *)param->model_operation.msg;
            store.dataArr = (uint8_t *)param->model_operation.msg;
            // ESP_LOGI(TAG, "Recv 0x%06x, tid 0x%04x, time %lldus",
            //     param->model_operation.opcode, store.vnd_tid, end_time - start_time);
            // ESP_LOGW(MASTER_TAG, "Number room: %d", store.dataArr[0]);
            // ESP_LOGW(MASTER_TAG, "Battery: %d", store.dataArr[1]);
            // ESP_LOGW(MASTER_TAG, "Temperature: %d", store.dataArr[2]);
            // ESP_LOGW(MASTER_TAG, "Smoke: %d", store.dataArr[3]);
            // ESP_LOGW(MASTER_TAG, "Temperature Alarm: %d", store.dataArr[4]);
            // ESP_LOGW(MASTER_TAG, "Flame Alarm: %d", store.dataArr[5]);
            // ESP_LOGW(MASTER_TAG, "Smoke Alarm: %d", store.dataArr[6]);
            // ESP_LOGW(MASTER_TAG, "FIRE STATUS: %d", store.dataArr[7]);
        }
        break;
    case ESP_BLE_MESH_MODEL_SEND_COMP_EVT:
        if (param->model_send_comp.err_code) {
            // ESP_LOGE(TAG, "Failed to send message 0x%06x", param->model_send_comp.opcode);
            break;
        }
        start_time = esp_timer_get_time();
        // ESP_LOGI(TAG, "Send 0x%06x", param->model_send_comp.opcode);
        break;
    case ESP_BLE_MESH_CLIENT_MODEL_RECV_PUBLISH_MSG_EVT:
        // ESP_LOGI(TAG, "Receive publish message 0x%06x", param->client_recv_publish_msg.opcode);
        store.dataArr = (uint8_t *)param->model_operation.msg;
        // ESP_LOGW(MASTER_TAG, "Received public message... ");
        // ESP_LOGW(MASTER_TAG, "Number room: %d", store.dataArr[0]);
        // ESP_LOGW(MASTER_TAG, "Battery: %d", store.dataArr[1]);
        // ESP_LOGW(MASTER_TAG, "Temperature: %d", store.dataArr[2]);
        // ESP_LOGW(MASTER_TAG, "Smoke: %d", store.dataArr[3]);
        // ESP_LOGW(MASTER_TAG, "Temperature Alarm: %d", store.dataArr[4]);
        // ESP_LOGW(MASTER_TAG, "Flame Alarm: %d", store.dataArr[5]);
        // ESP_LOGW(MASTER_TAG, "Smoke Alarm: %d", store.dataArr[6]);
        // ESP_LOGW(MASTER_TAG, "FIRE STATUS: %d", store.dataArr[7]);
        // int sof = 1111;
        // int ETX = 9999;
        // printf("%d ", sof);

        for(int i=0; i<8; i++) {
            printf("%d ", store.dataArr[i]);
        }
        // printf("%d", ETX);
        printf("\n");

        if(store.dataArr[7] == 1) {
            example_ble_mesh_send_vendor_message(ACTIVATE_ALARM_FRAME);
        }
        break;
    case ESP_BLE_MESH_CLIENT_MODEL_SEND_TIMEOUT_EVT:
        // ESP_LOGW(TAG, "Client message 0x%06x timeout", param->client_send_timeout.opcode);
        example_ble_mesh_send_vendor_message(REQUEST_FRAME);
        break;
    default:
        break;
    }
}

static esp_err_t ble_mesh_init(void)
{
    uint8_t match[2] = { 0x32, 0x10 };
    esp_err_t err;

    prov_key.net_idx = ESP_BLE_MESH_KEY_PRIMARY;
    prov_key.app_idx = APP_KEY_IDX;
    memset(prov_key.app_key, APP_KEY_OCTET, sizeof(prov_key.app_key));

    esp_ble_mesh_register_prov_callback(example_ble_mesh_provisioning_cb);
    esp_ble_mesh_register_config_client_callback(example_ble_mesh_config_client_cb);
    esp_ble_mesh_register_custom_model_callback(example_ble_mesh_custom_model_cb);

    err = esp_ble_mesh_init(&provision, &composition);
    if (err != ESP_OK) {
        // ESP_LOGE(TAG, "Failed to initialize mesh stack");
        return err;
    }

    err = esp_ble_mesh_client_model_init(&vnd_models[0]);
    if (err) {
        // ESP_LOGE(TAG, "Failed to initialize vendor client");
        return err;
    }

    err = esp_ble_mesh_provisioner_set_dev_uuid_match(match, sizeof(match), 0x0, false);
    if (err != ESP_OK) {
        // ESP_LOGE(TAG, "Failed to set matching device uuid");
        return err;
    }

    err = esp_ble_mesh_provisioner_prov_enable(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT);
    if (err != ESP_OK) {
        // ESP_LOGE(TAG, "Failed to enable mesh provisioner");
        return err;
    }

    err = esp_ble_mesh_provisioner_add_local_app_key(prov_key.app_key, prov_key.net_idx, prov_key.app_idx);
    if (err != ESP_OK) {
        // ESP_LOGE(TAG, "Failed to add local AppKey");
        return err;
    }

    // ESP_LOGI(TAG, "ESP BLE Mesh Provisioner initialized");

    return ESP_OK;
}

/************************************** USER FUNCTION *********************************************/
/************************************** USER FUNCTION *********************************************/
/************************************** USER FUNCTION *********************************************/

static void init_led_gpio()
{
    gpio_pad_select_gpio(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 0); // Initially turn off LED
}

static void configure_uart()
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};

    ESP_ERROR_CHECK(uart_param_config(EX_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(EX_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
}

static void configure_uart_interrupt()
{
    ESP_ERROR_CHECK(uart_isr_free(EX_UART_NUM));
    ESP_ERROR_CHECK(uart_isr_register(EX_UART_NUM, uart_intr_handle, NULL, ESP_INTR_FLAG_IRAM, &handle_console));
    ESP_ERROR_CHECK(uart_enable_rx_intr(EX_UART_NUM));
}

static void IRAM_ATTR uart_intr_handle(void *arg)
{
    uint16_t rx_fifo_len;
    rx_fifo_len = UART0.status.rxfifo_cnt; // read number of bytes in UART buffer

    if (rx_fifo_len > 0)
    {
        uint8_t data = UART0.fifo.rw_byte;

        if (data == '1')
        {
            rxFlag = 1;
            gpio_set_level(LED_GPIO, 1); // Turn on LED
            led_state = true;
        }
        // else if (data == '0')
        // {
        //     gpio_set_level(LED_GPIO, 0); // Turn off LED
        //     led_state = false;
        // }
    }

    uart_clear_intr_status(EX_UART_NUM, UART_RXFIFO_FULL_INT_CLR | UART_RXFIFO_TOUT_INT_CLR);
    uart_write_bytes(EX_UART_NUM, (const char *)"RX Done", 7);
}

// Delay for a number of seconds
static void delay_seconds(uint32_t seconds) {
    vTaskDelay(seconds * portTICK_PERIOD_MS * 10);
}

void timer_callback(void *param) {
    example_ble_mesh_send_vendor_message(REQUEST_FRAME);
}

void app_main(void)
{
    esp_err_t err;

    // ESP_LOGI(TAG, "Initializing...");

    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // board_init();

    err = bluetooth_init();
    if (err != ESP_OK) {
        // ESP_LOGE(TAG, "esp32_bluetooth_init failed (err %d)", err);
        return;
    }

    /* Open nvs namespace for storing/restoring mesh example info */
    err = ble_mesh_nvs_open(&NVS_HANDLE);
    if (err) {
        return;
    }

    ble_mesh_get_dev_uuid(dev_uuid);

    /* Initialize the Bluetooth Mesh Subsystem */
    err = ble_mesh_init();
    if (err != ESP_OK) {
        // ESP_LOGE(TAG, "Bluetooth mesh init failed (err %d)", err);
    }

    init_led_gpio();
    configure_uart();
    configure_uart_interrupt();

    // Configure Timer
    const esp_timer_create_args_t my_timer_args = {
        .callback = &timer_callback,
        .name = "My Timer"};
    esp_timer_handle_t timer_handler;
    ESP_ERROR_CHECK(esp_timer_create(&my_timer_args, &timer_handler));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer_handler, 8000000));

    while(1) {
        // ESP_LOGW(MASTER_TAG, "Comp size: %d", sizeof(composition) );
        // example_ble_mesh_send_vendor_message(REQUEST_FRAME);
        // delay_seconds(6);
        if(rxFlag == 1) {
            example_ble_mesh_send_vendor_message(DEACTIVATE_ALARM_FRAME);
            delay_seconds(1);
            gpio_set_level(LED_GPIO, 0); // Turn off LED
            rxFlag = 0;
        }
        else {
            delay_seconds(1);
        }
    }
}
