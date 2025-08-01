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

#define BOOTRATE 115200
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
const int BYTE_LENGTH = 10000000 / BOOTRATE;

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
    uart_config_t uart_config = {
        .baud_rate = BOOTRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    //TX RX RTS CTS
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, TX_NUM, RX_NUM, 18, 19));

    gpio_set_direction(GPIO_NUM, GPIO_MODE_OUTPUT);
}

int timer_print_end = 0; //for visualizing data
int64_t timer_finish_read; //visual
//callback func for timer
//it tracks when TX of the uart finished writing
void timer_callback(void *arg) {
    int last_pin;
    int curr_pin;

    gpio_set_level(GPIO_NUM, 1); //this can be deleted, only for visual

    //after this time, we need to exit the loop
    int64_t time_till_finish = esp_timer_get_time() + BYTE_LENGTH + BIT_LENGTH * 4; //this can be deleted, only for visual

    //this loop is monitoring the TX pin state
    curr_pin = gpio_get_level(TX_NUM);
    ets_delay_us(BIT_LENGTH);
    while(esp_timer_get_time() < time_till_finish) {
        last_pin = curr_pin;
        curr_pin = gpio_get_level(TX_NUM);

        if(last_pin != curr_pin) {
            time_till_finish = esp_timer_get_time() + BYTE_LENGTH + BIT_LENGTH * 4;
        }

        ets_delay_us(BIT_LENGTH);
    }

    //--------
    //here should be code that will be executed as reading end
    //code below can be deleted, it is for visual representation
    timer_finish_read = esp_timer_get_time();
    timer_print_end = 1; 
    gpio_set_level(GPIO_NUM, 0);
    //---------
}

void main_task(void *pvParameters) {

    //----------
    //Init timer
    esp_timer_handle_t timer_handle;
    esp_timer_create_args_t timer_config = {
        .callback = &timer_callback,
        .arg = NULL,
        .name = "read TX",
        .dispatch_method = ESP_TIMER_ISR //FOR THIS MENUCONFIG NEED TO BE ADJUSTED
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_config, &timer_handle));
    //----------

    char package[32];
    while(1) {
        //tick timer before writing the package
        //than write the package
        //timer will tick when astimated package sending time ends
        gpio_set_level(GPIO_NUM, 1);
        
        //----------
        //starting the timer
        //requires eso_timer_handle_t and time in ms 
        esp_timer_start_once(timer_handle, BYTE_LENGTH*sizeof(package));
        //----------
        
        int64_t timer_start_time = esp_timer_get_time();
        uart_write_bytes(UART_NUM, package, sizeof(package));
        gpio_set_level(GPIO_NUM, 0);
        int64_t write_bytes_finish_time = esp_timer_get_time();
        // ESP_LOGI("package", "send");
        if(timer_print_end) {
            timer_print_end = 0;
            ESP_LOGI("package", "\nTimer start: %llu\nWrite bytes finish: %llu\nTrasmitting finish: %llu\n", timer_start_time, write_bytes_finish_time, timer_finish_read);
            // ESP_LOGI("package", "ended");
        }
        vTaskDelay(100);
        // gpio_set_level(GPIO_NUM, 0);
    }
}
