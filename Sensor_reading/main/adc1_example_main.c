/* ADC1 Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

#define MP2_DIGITAL_PIN     GPIO_NUM_26
#define KY026_DIGITAL_PIN   GPIO_NUM_25
#define BUTTON_DIGITAL_PIN  GPIO_NUM_4
#define BUZZER_PIN          GPIO_NUM_5
#define LED_DIGITAL_PIN     GPIO_NUM_2

typedef enum {
    NORMAL_MEASURE_MODE = 0u,
    FALSE_ALARM_MEASURE_MODE = 1u
} Measure_Mode_t;

// Global variable that store temperature value
float temperature_val = 0;
// Global flags
static volatile uint8_t false_alarm_flag = 0;
static volatile uint8_t fire_flag = 0;
static Measure_Mode_t measure_mode = NORMAL_MEASURE_MODE;

// Queue Handler
// xQueueHandle interputQueue;

static esp_adc_cal_characteristics_t *adc_chars;

static const adc_channel_t ky026_channel = ADC_CHANNEL_5;       // GPIO33 if ADC1, GPIO12 if ADC2 
static const adc_channel_t lm35_channel = ADC_CHANNEL_6;        // GPIO34 if ADC1, GPIO14 if ADC2
static const adc_channel_t mp2_channel = ADC_CHANNEL_7;       // GPIO35 if ADC1, GPIO27 if ADC2  

static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

static void IRAM_ATTR gpio_interrupt_handler(void *args)
{
    // int pinNumber = (int)args;
    gpio_set_level(BUZZER_PIN, 1);
    gpio_set_level(LED_DIGITAL_PIN, 1);
    // xQueueSendFromISR(interputQueue, &pinNumber, NULL);
}

// void LED_Buzzer_Control_Task(void *params)
// {
//     int pinNumber;
//     while (true)
//     {
//         if (xQueueReceive(interputQueue, &pinNumber, portMAX_DELAY))
//         {
//             gpio_set_level(BUZZER_PIN, 1);
//             gpio_set_level(LED_DIGITAL_PIN, 1);
//         }
//     }
// }

void timer_callback(void *param)
{
    char* extreme_status = "Not Reached";
    uint32_t lm35_adc_reading = 0;
    uint32_t ky026_adc_reading = 0;
    uint32_t mp2_adc_reading = 0;
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

    lm35_adc_reading /= NO_OF_SAMPLES;
    ky026_adc_reading /= NO_OF_SAMPLES;
    mp2_adc_reading /= NO_OF_SAMPLES;

    // Maximum output voltage of lm35 is 1.5V -> max lm35_adc_reading value is 1675 (?)
    // Convert lm35_adc_reading to voltage in mV
    uint32_t lm35_voltage = esp_adc_cal_raw_to_voltage(lm35_adc_reading, adc_chars);
    uint32_t ky026_voltage = esp_adc_cal_raw_to_voltage(ky026_adc_reading, adc_chars);
    uint32_t mp2_voltage = esp_adc_cal_raw_to_voltage(mp2_adc_reading, adc_chars);

    // Convert to temperature
    temperature_val = lm35_voltage / 10;
    // Get extreme status of KY026
    if(temperature_val >= 40 && temperature_val <= 50) {
        false_alarm_flag = 1;
    }
    else if(ky026_voltage <= 500 && ky026_voltage >= 350) {
        false_alarm_flag = 1;
    }
    else if(mp2_voltage >= 1000 && mp2_voltage <= 1500) {
        false_alarm_flag = 1;
    }
    if(temperature_val > 50) {
        fire_flag = 1;
    }
    else if(ky026_voltage < 350) {
        fire_flag = 1;
    }
    else if(mp2_voltage > 1500) {
        fire_flag = 1;
    }
    if( (temperature_val < 40) && (ky026_voltage > 500) && (mp2_voltage < 1000) ) {
        false_alarm_flag = 0;
    }
    if(fire_flag == 1) {
        false_alarm_flag = 0;
        gpio_set_level(BUZZER_PIN, 1);
        gpio_set_level(LED_DIGITAL_PIN, 1);
    }
    
    // if(ky026_voltage <= 380) {
    //     false_alarm_flag = 0;
    //     extreme_status = "Reached!";
    //     gpio_set_level(BUZZER_PIN, 1);
    //     gpio_set_level(LED_DIGITAL_PIN, 1);
    // }
    printf(" Raw: %d\t Lm35 Voltage: %dmV\n", lm35_adc_reading, lm35_voltage);
    printf(" Lm35 Temp: %f oC\n", temperature_val);
    printf(" Raw: %d\t Ky026 voltage: %dmV\t Ky026 extreme value: %s\n", ky026_adc_reading, ky026_voltage, extreme_status);
    printf(" Raw: %d\t Mp2 Voltage: %dmV\n", mp2_adc_reading, mp2_voltage);
    printf("False alarm flag: %d\n", false_alarm_flag);
    printf("Fire flag: %d\n", fire_flag);
    printf("\n");

}

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


void app_main(void)
{
    gpio_pad_select_gpio(KY026_DIGITAL_PIN);
    gpio_set_direction(KY026_DIGITAL_PIN, GPIO_MODE_OUTPUT);

    gpio_pad_select_gpio(BUZZER_PIN);
    gpio_set_direction(BUZZER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(BUZZER_PIN, 0);

    gpio_pad_select_gpio(BUTTON_DIGITAL_PIN);
    gpio_set_direction(BUTTON_DIGITAL_PIN, GPIO_MODE_INPUT);
    gpio_pulldown_dis(BUTTON_DIGITAL_PIN);
    gpio_pullup_en(BUTTON_DIGITAL_PIN);
    gpio_set_intr_type(BUTTON_DIGITAL_PIN, GPIO_INTR_NEGEDGE);

    gpio_pad_select_gpio(LED_DIGITAL_PIN);
    gpio_set_direction(LED_DIGITAL_PIN, GPIO_MODE_INPUT);
    gpio_set_level(LED_DIGITAL_PIN, 0);

    // interputQueue = xQueueCreate(10, sizeof(int));
    // xTaskCreate(LED_Buzzer_Control_Task, "LED_Buzzer_Control_Task", 2048, NULL, 1, NULL);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_DIGITAL_PIN, gpio_interrupt_handler, (void *)BUTTON_DIGITAL_PIN);

    //Check if Two Point or Vref are burned into eFuse
    check_efuse();

    //Configure ADC
    if (unit == ADC_UNIT_1) {
        adc1_config_width(width);
        adc1_config_channel_atten(lm35_channel, atten);
        adc1_config_channel_atten(ky026_channel, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)lm35_channel, atten);
        adc2_config_channel_atten((adc2_channel_t)ky026_channel, atten);
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
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer_handler, 15000000));

    while (1) {
        // esp_timer_dump(stdout);
        // vTaskDelay(pdMS_TO_TICKS(1000));
        if(false_alarm_flag == 1 || fire_flag == 1) {
            if(measure_mode == NORMAL_MEASURE_MODE) {
                ESP_ERROR_CHECK(esp_timer_stop(timer_handler));
                ESP_ERROR_CHECK(esp_timer_start_periodic(timer_handler, 5000000));
                measure_mode = FALSE_ALARM_MEASURE_MODE;
            }
        }
        else if( (fire_flag == 0) || (false_alarm_flag == 0) ) {
            if(measure_mode == FALSE_ALARM_MEASURE_MODE) {
                ESP_ERROR_CHECK(esp_timer_stop(timer_handler));
                ESP_ERROR_CHECK(esp_timer_start_periodic(timer_handler, 15000000));
            }
            measure_mode = NORMAL_MEASURE_MODE;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
