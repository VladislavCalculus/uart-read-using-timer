#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/uart.h"
#include <esp_err.h>
#include "driver/gpio.h"
#include "esp_timer.h"
#include "soc/gpio_sig_map.h"


#define BOOTRATE 155000
#define DATA_LENGTH 10

//uart and pins for this project
const uart_port_t UART_NUM = UART_NUM_2;
const gpio_num_t GPIO_NUM = GPIO_NUM_4;
const uint32_t TX_NUM = 16;
const uint32_t RX_NUM = 17;

//calculating byte lenght in time
const int BYTE_LENGTH = 1000000 / BOOTRATE * 10;

void uart_init();
void main_task(void *pvParameters);
void blink_LED_task(void *pvParameters);

void app_main(void)
{   
    uart_init();
    xTaskCreate(main_task, "main task", 1024, NULL, 1, NULL);
}

void uart_init() {
    const int uart_buffer_size = (1024 * 2);
    QueueHandle_t uart_queue;
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0));
    //TX RX RTS CTS
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, TX_NUM, RX_NUM, 18, 19));

    gpio_set_direction(GPIO_NUM, GPIO_MODE_OUTPUT);
}

//0 - being written in, 1 - waiting
int pin_state(int data[]) {
    for(int i = 0; i < DATA_LENGTH-1; i++) {
        if(data[i] != data[i+1]) {
            return 0;
        }
    }
    return 1;
}

bool flag = 0;
//callback func for timer
//it tracks when TX of the uart finished writing
void timer_callback(void *arg) {
    // TaskHandle_t LED_task_handle;
    // xTaskCreate(blink_LED_task, "blink LED task", 1024, NULL, 1, &LED_task_handle);
    // vTaskDelete(LED_task_handle);
    // esp_rom_gpio_connect_in_signal(RX_NUM, U2RXD_IN_IDX, 0);

    int data[DATA_LENGTH];
    int data_idx = 0;
    while(1) {
        data[data_idx++] = gpio_get_level(TX_NUM);
        if(data_idx == DATA_LENGTH) {
            data_idx = 0;
            if(pin_state(data)) {
                flag = 1;
                break;
            }
        }
        vTaskDelay(BYTE_LENGTH*10);
    }
}

void main_task(void *pvParameters) {
    esp_timer_handle_t timer_handle;
    esp_timer_create_args_t timer_config = {
        .callback = &timer_callback,
        .arg = NULL,
        .name = "read TX",
        .dispatch_method = ESP_TIMER_ISR
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_config, &timer_handle));
    char package[64];
    while(1) {
        //tick timer before writing the package
        //than write the package
        //timer will tick when astimated package sending time ends
        esp_timer_start_once(timer_handle, BYTE_LENGTH);
        uart_write_bytes(UART_NUM, package, sizeof(package));
        if(flag) {
            ESP_LOGI("tag", "WORKS!");
        }
        vTaskDelay(100);
    }
}

void blink_LED_task(void *pvParameters) {
    while(1) {
        gpio_set_level(GPIO_NUM, 1);
        vTaskDelay(BYTE_LENGTH);
        gpio_set_level(GPIO_NUM, 0);
        vTaskDelay(BYTE_LENGTH);
    }
}