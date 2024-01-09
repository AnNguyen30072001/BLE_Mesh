/* main.c - Application main entry point */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_local_data_operation_api.h"

#include "board.h"
#include "ble_mesh_example_init.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/timer.h"
#include "esp_adc_cal.h"

#define TAG "EXAMPLE"
#define NODE_TAG "NODE"

#define CID_ESP     0x02E5

#define ESP_BLE_MESH_VND_MODEL_ID_CLIENT    0x0000
#define ESP_BLE_MESH_VND_MODEL_ID_SERVER    0x0001

#define ESP_BLE_MESH_VND_MODEL_OP_SEND      ESP_BLE_MESH_MODEL_OP_3(0x00, CID_ESP)
#define ESP_BLE_MESH_VND_MODEL_OP_STATUS    ESP_BLE_MESH_MODEL_OP_3(0x01, CID_ESP)

// Configure for sensor reading
#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

#define MP2_DIGITAL_PIN     GPIO_NUM_26
#define KY026_DIGITAL_PIN   GPIO_NUM_25
#define BUTTON_DIGITAL_PIN  GPIO_NUM_4
#define BUZZER_PIN          GPIO_NUM_5
#define LED_DIGITAL_PIN     GPIO_NUM_2
#define BLE_LED_PIN         GPIO_NUM_18

#define NODE_ID             6

typedef enum {
    FLAME_OK            = 0u,
    FLAME_ALARM         = 1u,
} Status_Flame_t;

typedef enum {
    TEMP_OK             = 0u,
    TEMP_ALARM          = 1u,
} Status_Temp_t;

typedef enum {
    SMOKE_OK            = 0u,
    SMOKE_ALARM         = 1u,
} Status_Smoke_t;

typedef enum {
    NO_FIRE             = 0u,
    FIRE                = 1u,
} Fire_Sts_t;

typedef enum {
    NORMAL_MEASURE_MODE = 0u,
    FALSE_ALARM_MEASURE_MODE = 1u
} Measure_Mode_t;

static esp_adc_cal_characteristics_t *adc_chars;

static const adc_channel_t ky026_channel = ADC_CHANNEL_5;       // GPIO33 if ADC1, GPIO12 if ADC2 
static const adc_channel_t lm35_channel = ADC_CHANNEL_6;        // GPIO34 if ADC1, GPIO14 if ADC2
static const adc_channel_t mp2_channel = ADC_CHANNEL_7;       // GPIO35 if ADC1, GPIO27 if ADC2  
static const adc_channel_t battery_channel = ADC_CHANNEL_3;       // GPIO39 if ADC1, GPIO15 if ADC2 

static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;


// Const value for Euler number
const float euler_val = 2.718281828459045;

// Global variables that store sensor value and battery capacity
float temperature_val = 0;
int smoke_ppm_val = 0;
uint8_t ppm_digit[4] = {0};
uint8_t bat_capacity = 0;
uint8_t prev_bat_capacity = 0;

// Update flag
volatile uint8_t update_flag = 0;
volatile uint8_t ctx_flag = 0;
volatile uint8_t alarm_flag = 0;
volatile uint8_t BLE_flag = 0;

// False alarm flag
static volatile uint8_t false_alarm_flag = 0;
static Measure_Mode_t measure_mode = NORMAL_MEASURE_MODE;

// Variables that store voltage value
uint32_t lm35_voltage = 0;
uint32_t ky026_voltage = 0;
uint32_t mp2_voltage = 0;
uint32_t bat_voltage = 0;

/**
 * Data array to send to gateway. The order of data:
 * Data_arr[0]: room number
 * Data_arr[1]: battery percentage
 * Data_arr[2]: Temperature value
 * Data_arr[3]: Smoke value
 * Data_arr[4]: Temperature alarm flag. Value is 1 if temperature is too high => fire
 * Data_arr[5]: Flame sensor alarm flag. Value is 1 if detect flame => fire
 * Data_arr[6]: Smoke alarm flag. Value is 1 if detect smoke is too high => fire
 * Data_arr[7]: General fire alarm flag. Value is 1 if detect any sign of fire from sensors
 * Data_arr[8] - [11]: ppm value in digits
 */
// Sensor data to be sent to gateway
// Room number is 101
uint8_t Data_arr[12] = {NODE_ID, 0, 0, 0, TEMP_OK, FLAME_OK, SMOKE_OK, NO_FIRE, 0, 0, 0, 0};

static esp_ble_mesh_msg_ctx_t ctx_user = {
    .net_idx = 0,
    .app_idx = 0,
    .addr = 0,
    .send_rel = true,
    .send_ttl = ESP_BLE_MESH_TTL_DEFAULT,
};

static uint8_t dev_uuid[ESP_BLE_MESH_OCTET16_LEN] = { 0x32, 0x10 };

static esp_ble_mesh_cfg_srv_t config_server = {
    .relay = ESP_BLE_MESH_RELAY_ENABLED,
    .beacon = ESP_BLE_MESH_BEACON_ENABLED,
#if defined(CONFIG_BLE_MESH_FRIEND)
    .friend_state = ESP_BLE_MESH_FRIEND_ENABLED,
#else
    .friend_state = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
#endif
#if defined(CONFIG_BLE_MESH_GATT_PROXY_SERVER)
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_ENABLED,
#else
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
#endif
    .default_ttl = 7,
    /* 3 transmissions with 20ms interval */
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
};

static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
};

static esp_ble_mesh_model_op_t vnd_op[] = {
    ESP_BLE_MESH_MODEL_OP(ESP_BLE_MESH_VND_MODEL_OP_SEND, 2),
    ESP_BLE_MESH_MODEL_OP_END,
};

static esp_ble_mesh_model_t vnd_models[] = {
    ESP_BLE_MESH_VENDOR_MODEL(CID_ESP, ESP_BLE_MESH_VND_MODEL_ID_SERVER,
    vnd_op, NULL, NULL),
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
    .uuid = dev_uuid,
};

static void prov_complete(uint16_t net_idx, uint16_t addr, uint8_t flags, uint32_t iv_index)
{
    ESP_LOGI(TAG, "net_idx 0x%03x, addr 0x%04x", net_idx, addr);
    ESP_LOGI(TAG, "flags 0x%02x, iv_index 0x%08x", flags, iv_index);
    board_led_operation(LED_G, LED_OFF);
}

static void example_ble_mesh_provisioning_cb(esp_ble_mesh_prov_cb_event_t event,
                                             esp_ble_mesh_prov_cb_param_t *param)
{
    switch (event) {
    case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROV_REGISTER_COMP_EVT, err_code %d", param->prov_register_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT, err_code %d", param->node_prov_enable_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT, bearer %s",
            param->node_prov_link_open.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT, bearer %s",
            param->node_prov_link_close.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT");
        prov_complete(param->node_prov_complete.net_idx, param->node_prov_complete.addr,
            param->node_prov_complete.flags, param->node_prov_complete.iv_index);
        break;
    case ESP_BLE_MESH_NODE_PROV_RESET_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_RESET_EVT");
        break;
    case ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT, err_code %d", param->node_set_unprov_dev_name_comp.err_code);
        break;
    default:
        break;
    }
}

static void example_ble_mesh_config_server_cb(esp_ble_mesh_cfg_server_cb_event_t event,
                                              esp_ble_mesh_cfg_server_cb_param_t *param)
{
    if (event == ESP_BLE_MESH_CFG_SERVER_STATE_CHANGE_EVT) {
        switch (param->ctx.recv_op) {
        case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD");
            ESP_LOGI(TAG, "net_idx 0x%04x, app_idx 0x%04x",
                param->value.state_change.appkey_add.net_idx,
                param->value.state_change.appkey_add.app_idx);
            ESP_LOG_BUFFER_HEX("AppKey", param->value.state_change.appkey_add.app_key, 16);
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND");
            ESP_LOGI(TAG, "elem_addr 0x%04x, app_idx 0x%04x, cid 0x%04x, mod_id 0x%04x",
                param->value.state_change.mod_app_bind.element_addr,
                param->value.state_change.mod_app_bind.app_idx,
                param->value.state_change.mod_app_bind.company_id,
                param->value.state_change.mod_app_bind.model_id);
            break;
        default:
            break;
        }
    }
}

static void example_ble_mesh_custom_model_cb(esp_ble_mesh_model_cb_event_t event,
                                             esp_ble_mesh_model_cb_param_t *param)
{
    switch (event) {
    case ESP_BLE_MESH_MODEL_OPERATION_EVT:
        if (param->model_operation.opcode == ESP_BLE_MESH_VND_MODEL_OP_SEND) {
            uint16_t received_frame_type = *(uint16_t *)param->model_operation.msg;
            
            ESP_LOGI(TAG, "Recv 0x%06x, frame type 0x%04x", param->model_operation.opcode, received_frame_type);

            if(received_frame_type == 1) {
                esp_err_t err = esp_ble_mesh_server_model_send_msg(&vnd_models[0],
                    param->model_operation.ctx, ESP_BLE_MESH_VND_MODEL_OP_STATUS,
                    sizeof(Data_arr), (uint8_t *)Data_arr);
                if (err) {
                    ESP_LOGE(TAG, "Failed to send message 0x%06x", ESP_BLE_MESH_VND_MODEL_OP_STATUS);
                    gpio_set_level(BLE_LED_PIN, 0);
                }
                else {
                    gpio_set_level(BLE_LED_PIN, 1);
                    BLE_flag = 1;
                }
            }
            else if(received_frame_type == 2) {
                gpio_set_level(BUZZER_PIN, 1);
                gpio_set_level(LED_DIGITAL_PIN, 1);
                ESP_LOGW(NODE_TAG, "Received Activate Alarm frame");
            }
            else if(received_frame_type == 3) {
                gpio_set_level(BUZZER_PIN, 0);
                gpio_set_level(LED_DIGITAL_PIN, 0);
                ESP_LOGW(NODE_TAG, "Received Deactivate Alarm frame");
            }

            if(ctx_flag == 0) {
                ctx_user.net_idx = param->model_operation.ctx->net_idx;
                ctx_user.app_idx = param->model_operation.ctx->app_idx;
                ctx_user.addr = param->model_operation.ctx->addr;
                ctx_flag = 1;
            }
        }
        break;
    case ESP_BLE_MESH_MODEL_SEND_COMP_EVT:
        if (param->model_send_comp.err_code) {
            ESP_LOGE(TAG, "Failed to send message 0x%06x", param->model_send_comp.opcode);
            break;
        }
        ESP_LOGI(TAG, "Send 0x%06x", param->model_send_comp.opcode);
        break;
    default:
        break;
    }
}

static esp_err_t ble_mesh_init(void)
{
    esp_err_t err;

    esp_ble_mesh_register_prov_callback(example_ble_mesh_provisioning_cb);
    esp_ble_mesh_register_config_server_callback(example_ble_mesh_config_server_cb);
    esp_ble_mesh_register_custom_model_callback(example_ble_mesh_custom_model_cb);

    err = esp_ble_mesh_init(&provision, &composition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize mesh stack");
        return err;
    }

    err = esp_ble_mesh_node_prov_enable(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable mesh node");
        return err;
    }

    board_led_operation(LED_G, LED_ON);

    ESP_LOGI(TAG, "BLE Mesh Node initialized");

    return ESP_OK;
}


// Delay for a number of seconds
void delay_seconds(uint32_t seconds) {
    vTaskDelay(seconds * portTICK_PERIOD_MS * 10);
}

void send_fire_alarm_immediately()
{
    esp_ble_mesh_msg_ctx_t *ctx_user_ptr = &ctx_user;

    esp_err_t err = esp_ble_mesh_server_model_send_msg(&vnd_models[0], ctx_user_ptr,
    ESP_BLE_MESH_VND_MODEL_OP_STATUS, sizeof(Data_arr), (uint8_t *)Data_arr);
    if (err) {
        ESP_LOGW(NODE_TAG, "Send message to Gateway failed");
        return;
    }
    else {
        ESP_LOGW(NODE_TAG, "Message sent to Gateway OK");
    }
}

// Warning Button interrupt handler
static void IRAM_ATTR gpio_interrupt_handler(void *args)
{
    gpio_set_level(BUZZER_PIN, 1);
    gpio_set_level(LED_DIGITAL_PIN, 1);
    Data_arr[7] = FIRE;
    alarm_flag = 1;
}

// Timer callback for BLE timeout
void timer_callback_BLE(void *param)
{
    gpio_set_level(BLE_LED_PIN, 0);
}

// Timer callback for reading sensor
void timer_callback(void *param)
{
    // char* extreme_status = "Not Reached";
    uint32_t lm35_adc_reading = 0;
    uint32_t ky026_adc_reading = 0;
    uint32_t mp2_adc_reading = 0;
    uint32_t bat_adc_reading = 0;
    //Multisampling
    for (int i = 0; i < NO_OF_SAMPLES; i++) {
        if (unit == ADC_UNIT_1) {
            lm35_adc_reading += adc1_get_raw((adc1_channel_t)lm35_channel);
        } else {
            // int raw;
            // adc2_get_raw((adc2_channel_t)lm35_channel, width, &raw);
            // lm35_adc_reading += raw;
        }
    }

    for (int j = 0; j < NO_OF_SAMPLES; j++) {
        if (unit == ADC_UNIT_1) {
            ky026_adc_reading += adc1_get_raw((adc1_channel_t)ky026_channel);
        }
    }

    for (int k = 0; k < NO_OF_SAMPLES; k++) {
        if (unit == ADC_UNIT_1) {
            mp2_adc_reading += adc1_get_raw((adc1_channel_t)mp2_channel);
        }
    }

    for (int m = 0; m < NO_OF_SAMPLES; m++) {
        if (unit == ADC_UNIT_1) {
            bat_adc_reading += adc1_get_raw((adc1_channel_t)battery_channel);
        }
    }
    

    lm35_adc_reading /= NO_OF_SAMPLES;
    ky026_adc_reading /= NO_OF_SAMPLES;
    mp2_adc_reading /= NO_OF_SAMPLES;
    bat_adc_reading /= NO_OF_SAMPLES;

    // Convert lm35_adc_reading to voltage in mV
    lm35_voltage = esp_adc_cal_raw_to_voltage(lm35_adc_reading, adc_chars);
    ky026_voltage = esp_adc_cal_raw_to_voltage(ky026_adc_reading, adc_chars);
    mp2_voltage = esp_adc_cal_raw_to_voltage(mp2_adc_reading, adc_chars);
    bat_voltage = esp_adc_cal_raw_to_voltage(bat_adc_reading, adc_chars);

    // Convert to temperature value
    temperature_val = lm35_voltage / 10;
    // Convert to Smoke ppm value
    float temp = 2.3812 * (float)((float)mp2_voltage / 1000);
    smoke_ppm_val = (int)(5.9627 * pow(euler_val, temp));
    int tmp1 = smoke_ppm_val;
    // Convert to battery capacity
    float bat_voltage_V = (float)((float)bat_voltage / 1000);
    printf("bat vol: %f\n", bat_voltage_V);
    bat_voltage_V = bat_voltage_V * 10;
    bat_voltage_V += 2;
    if(bat_voltage_V > 11.4) {
        bat_capacity = prev_bat_capacity;
    }
    if(bat_voltage_V < 5.55) {
        bat_capacity = 0;
    }
    else if((bat_voltage_V > 9.63) && (bat_voltage_V < 11.4)) {
        bat_capacity = 100;
    }
    else {
        bat_capacity = (uint8_t)(0.0969*pow(bat_voltage_V, 6) - 7.5439*pow(bat_voltage_V, 5) + 200.95*pow(bat_voltage_V, 4) \
            - 2581.2*pow(bat_voltage_V, 3) + 17483*pow(bat_voltage_V, 2) - 60279*bat_voltage_V + 83631);
        
        bat_capacity -= 5;
        bat_capacity = 100 - bat_capacity;
        prev_bat_capacity = bat_capacity;
    }

    printf(" Raw: %d\t Mp2 Voltage: %dmV\t Mp2 PPM: %dppm\n", mp2_adc_reading, mp2_voltage, smoke_ppm_val);

    // Update digits of smoke value to update later
    if(smoke_ppm_val > 9999) {
        for(uint8_t j=0; j<4; j++) {
            ppm_digit[3-j] = 9;
        }
    }
    else {
        for(uint8_t i=0; i<4; i++) {
            tmp1 = smoke_ppm_val % 10;
            ppm_digit[3-i] = tmp1;
            smoke_ppm_val /= 10;
        }
    }
      
    // Set flag to update into buffer
    update_flag = 1;

    printf(" \nBat capacity: %d\n", bat_capacity);
    printf(" Raw: %d\t Lm35 Voltage: %dmV\n", lm35_adc_reading, lm35_voltage);
    printf(" Lm35 Temp: %f oC\n", temperature_val);
    printf(" Raw: %d\t Ky026 voltage: %dmV\n", ky026_adc_reading, ky026_voltage);
    printf("Temp = %f\n", temp);
    printf("\n");
    
}

// Check efuse ADC
static void check_efuse(void)
{
#if CONFIG_IDF_TARGET_ESP32
    //Check if TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }
    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
#elif CONFIG_IDF_TARGET_ESP32S2
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("Cannot retrieve eFuse Two Point calibration values. Default calibration values will be used.\n");
    }
#else
#error "This example is configured for ESP32/ESP32S2."
#endif
}

// Print ADC config
static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}

void dataUpdate(void) {
    Data_arr[1] = bat_capacity;
    Data_arr[2] = temperature_val;
    Data_arr[3] = bat_capacity;
    Data_arr[7] = NO_FIRE;
    Data_arr[8] = ppm_digit[0];
    Data_arr[9] = ppm_digit[1];
    Data_arr[10] = ppm_digit[2];
    Data_arr[11] = ppm_digit[3];
    if( (temperature_val < 80) && (ky026_voltage > 600) && (mp2_voltage < 2900) ) {
        false_alarm_flag = 0;
    }
    if(temperature_val >= 80 && temperature_val <= 100) {
        false_alarm_flag = 1;
    }
    else if(ky026_voltage <= 600 && ky026_voltage >= 400) {
        false_alarm_flag = 1;
    }
    else if(mp2_voltage >= 2900 && mp2_voltage <= 3100) {
        false_alarm_flag = 1;
    }
    if(temperature_val > 80) {
        Data_arr[4] = TEMP_ALARM;
        Data_arr[7] = FIRE;
    }
    else {
        Data_arr[4] = TEMP_OK;
    }
    if(ky026_voltage < 400) {
        Data_arr[5] = FLAME_ALARM;
        Data_arr[7] = FIRE;
    }
    else {
        Data_arr[5] = FLAME_OK;
    }
    if(mp2_voltage > 3100) {
        Data_arr[6] = SMOKE_ALARM;
        Data_arr[7] = FIRE;
    }
    else {
        Data_arr[6] = SMOKE_OK;
    }
    
    if(Data_arr[7] == FIRE) {
        false_alarm_flag = 0;
        ESP_LOGW(NODE_TAG, "FIRE DETECTED. SENDING TO GATEWAY NOW...");
        send_fire_alarm_immediately();
        gpio_set_level(BUZZER_PIN, 1);
        gpio_set_level(LED_DIGITAL_PIN, 1);
    }
    
    update_flag = 0;
}


void app_main(void)
{
    // Configure GPIOs
    gpio_pad_select_gpio(KY026_DIGITAL_PIN);
    gpio_set_direction(KY026_DIGITAL_PIN, GPIO_MODE_INPUT);

    gpio_pad_select_gpio(BUZZER_PIN);
    gpio_set_direction(BUZZER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(BUZZER_PIN, 0);

    gpio_pad_select_gpio(BUTTON_DIGITAL_PIN);
    gpio_set_direction(BUTTON_DIGITAL_PIN, GPIO_MODE_INPUT);
    // gpio_pulldown_dis(BUTTON_DIGITAL_PIN);
    // gpio_pullup_en(BUTTON_DIGITAL_PIN);
    gpio_set_pull_mode(BUTTON_DIGITAL_PIN, GPIO_PULLUP_ONLY);
    gpio_set_intr_type(BUTTON_DIGITAL_PIN, GPIO_INTR_ANYEDGE);

    gpio_pad_select_gpio(LED_DIGITAL_PIN);
    gpio_set_direction(LED_DIGITAL_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_DIGITAL_PIN, 0);

    gpio_pad_select_gpio(BLE_LED_PIN);
    gpio_set_direction(BLE_LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(BLE_LED_PIN, 0);

    // Configure gpio interrupt
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_DIGITAL_PIN, gpio_interrupt_handler, (void *)BUTTON_DIGITAL_PIN);

    //Check if Two Point or Vref are burned into eFuse
    check_efuse();

    //Configure ADC
    if (unit == ADC_UNIT_1) {
        adc1_config_width(width);
        adc1_config_channel_atten(lm35_channel, atten);
        adc1_config_channel_atten(ky026_channel, atten);
        adc1_config_channel_atten(mp2_channel, atten);
        adc1_config_channel_atten(battery_channel, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)lm35_channel, atten);
        adc2_config_channel_atten((adc2_channel_t)ky026_channel, atten);
        adc2_config_channel_atten((adc2_channel_t)mp2_channel, atten);
        adc2_config_channel_atten((adc2_channel_t)battery_channel, atten);
    }

    // Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

    // Configure Timer
    const esp_timer_create_args_t my_timer_args = {
        .callback = &timer_callback,
        .name = "My Timer"};
    esp_timer_handle_t timer_handler;
    ESP_ERROR_CHECK(esp_timer_create(&my_timer_args, &timer_handler));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer_handler, 5000000));

    // BLE configure
    esp_err_t err;

    ESP_LOGI(TAG, "Initializing...");

    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    board_init();

    err = bluetooth_init();
    if (err) {
        ESP_LOGE(TAG, "esp32_bluetooth_init failed (err %d)", err);
        return;
    }

    ble_mesh_get_dev_uuid(dev_uuid);

    /* Initialize the Bluetooth Mesh Subsystem */
    err = ble_mesh_init();
    if (err) {
        ESP_LOGE(TAG, "Bluetooth mesh init failed (err %d)", err);
    }

    // Configure Timer for BLE status
    const esp_timer_create_args_t my_timer_args_BLE = {
        .callback = &timer_callback_BLE,
        .name = "My BLE timer"};
    esp_timer_handle_t timer_handler_BLE;
    ESP_ERROR_CHECK(esp_timer_create(&my_timer_args_BLE, &timer_handler_BLE));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer_handler_BLE, 7000000));

    while(1) {
        if(alarm_flag == 1) {
            send_fire_alarm_immediately();
            alarm_flag = 0;
        }
        if(BLE_flag == 1){
            ESP_ERROR_CHECK(esp_timer_stop(timer_handler_BLE));
            ESP_ERROR_CHECK(esp_timer_start_periodic(timer_handler_BLE, 11000000));
            BLE_flag = 0;
        }
        if(update_flag == 1) {
            dataUpdate();
        }
        if(false_alarm_flag == 1 || Data_arr[7] == FIRE) {
            if(measure_mode == NORMAL_MEASURE_MODE) {
                ESP_ERROR_CHECK(esp_timer_stop(timer_handler));
                ESP_ERROR_CHECK(esp_timer_start_periodic(timer_handler, 2000000));
                measure_mode = FALSE_ALARM_MEASURE_MODE;
            }
        }
        else if( (Data_arr[7] == NO_FIRE) || (false_alarm_flag == 0) ) {
            if(measure_mode == FALSE_ALARM_MEASURE_MODE) {
                ESP_ERROR_CHECK(esp_timer_stop(timer_handler));
                ESP_ERROR_CHECK(esp_timer_start_periodic(timer_handler, 5000000));
            }
            measure_mode = NORMAL_MEASURE_MODE;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

