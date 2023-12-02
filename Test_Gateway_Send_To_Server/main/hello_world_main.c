#include <stdio.h>
#include <stdlib.h> 
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_intr_alloc.h"
#include "esp32/rom/uart.h"

#define LED_GPIO GPIO_NUM_2

static const char *TAG = "uart_events";

#define EX_UART_NUM UART_NUM_0

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)

static bool led_state = false;
static intr_handle_t handle_console;

// Sample array of 8 numbers
static uint8_t sample_array[8] = {101, 2, 3, 4, 5, 6, 7, 8};

static void IRAM_ATTR uart_intr_handle(void *arg);

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
            gpio_set_level(LED_GPIO, 1); // Turn on LED
            led_state = true;
        }
        else if (data == '0')
        {
            gpio_set_level(LED_GPIO, 0); // Turn off LED
            led_state = false;
        }
    }

    uart_clear_intr_status(EX_UART_NUM, UART_RXFIFO_FULL_INT_CLR | UART_RXFIFO_TOUT_INT_CLR);
    uart_write_bytes(EX_UART_NUM, (const char *)"RX Done", 7);
}

void app_main()
{
    esp_log_level_set(TAG, ESP_LOG_INFO);

    init_led_gpio();
    configure_uart();
    configure_uart_interrupt();

    while (1)
    {
        sample_array[7] = rand() % 2;
        sample_array[0] = 101 + rand() % 4;
        // Print the sample array every 5 seconds
        for (int i = 0; i < 8; i++)
        {
            printf("%d ", sample_array[i]);
        }
        printf("\n");

        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}
