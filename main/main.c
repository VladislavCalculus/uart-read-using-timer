#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/uart.h"
#include <esp_err.h>
#include "driver/gpio.h"
#include "esp_timer.h"

uart_port_t UART_NUM = UART_NUM_2;

void uart_init();
void main_task(void *pvParameters);

void app_main(void)
{   
    uart_init();
    xTaskCreate(main_task, "task", 1024, NULL, 1, NULL);
}

void uart_init() {
    const int uart_buffer_size = (1024 * 2);
    QueueHandle_t uart_queue;
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0));
}

void timer_callback(void *arg) {
    
}

void main_task(void *pvParameters) {
    esp_timer_handle_t timer_handle;
    esp_timer_create_args_t timer_config = {
        .callback = NULL,
        .arg = NULL,
        .name = "read TX",
        .dispatch_method = ESP_TIMER_ISR
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_config, &timer_handle));

    while(1) {
        vTaskDelay(10);
    }
}