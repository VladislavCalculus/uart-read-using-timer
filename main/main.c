#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/uart.h"
#include <esp_err.h>
#include "driver/gpio.h"

uart_port_t UART_NUM = UART_NUM_2;

void uart_init();
void task(void *pvParameters);

void app_main(void)
{   
    uart_init();
    // xTaskCreate(task, "task", 1024, NULL, 1, NULL);
}

void uart_init() {
    const int uart_buffer_size = (1024 * 2);
    QueueHandle_t uart_queue;
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0));
}

void task(void *pvParameters) {
    
}