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
#include "rom/ets_sys.h"

#define BOOTRATE 155000
#define DATA_LENGTH 10

//uart and pins for this project
const uart_port_t UART_NUM = UART_NUM_2;
const gpio_num_t GPIO_NUM = GPIO_NUM_4;
const uint32_t TX_NUM = 16;
const uint32_t RX_NUM = 17;

//uart_num_2 queue
QueueHandle_t uart_queue;


//calculating byte lenght in time
const int BIT_LENGTH = 1000000 / BOOTRATE;
const int BYTE_LENGTH = BIT_LENGTH * 10;

void uart_init();
void main_task(void *pvParameters);
void blink_LED_task(void *pvParameters);

void app_main(void)
{   
    uart_init();
    xTaskCreate(main_task, "main task", 4096, NULL, 1, NULL);
}

void uart_init() {
    const int uart_buffer_size = (1024 * 2);
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

int timer_print_end = 0;
//callback func for timer
//it tracks when TX of the uart finished writing
void timer_callback(void *arg) {
    int last_pin = NULL;
    int curr_pin = NULL;
    gpio_set_level(GPIO_NUM, 1);

    //this loop is monitoring the TX pin state
    while(1) {
        curr_pin = gpio_get_level(TX_NUM);

        ets_delay_us(BIT_LENGTH);
    }

    gpio_set_level(GPIO_NUM, 0);
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
    char package[32];
    while(1) {
        //tick timer before writing the package
        //than write the package
        //timer will tick when astimated package sending time ends
        esp_timer_start_once(timer_handle, BYTE_LENGTH);
        uart_write_bytes(UART_NUM, package, sizeof(package));
        ESP_LOGI("package", "send");
        if(timer_print_end) {
            timer_print_end = 0;
            ESP_LOGI("package", "ended");
        }
        vTaskDelay(100);
        // gpio_set_level(GPIO_NUM, 0);
    }
}

void blink_LED_task(void *pvParameters) {
    while(1) {
        gpio_set_level(GPIO_NUM, 1);
        vTaskDelay(BYTE_LENGTH);
        vTaskDelete(NULL);
    }
}