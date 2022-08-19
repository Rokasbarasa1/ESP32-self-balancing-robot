#include <stdio.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "../lib/esp_01/esp_01.h"
#include "../lib/adxl345/adxl345.h"

// ESP32 DevkitC v4 // ESP-WROOM-32D
// 160 Mhz

char *wifi_name = "Stofa70521";
char *wifi_password = "bis56lage63";
char *server_ip = "192.168.87.178";
char *server_port = "4000";

void init_lights(){
        gpio_set_direction(GPIO_NUM_32, GPIO_MODE_OUTPUT);
        gpio_set_direction(GPIO_NUM_33, GPIO_MODE_OUTPUT);
        gpio_set_direction(GPIO_NUM_25, GPIO_MODE_OUTPUT);
        gpio_set_direction(GPIO_NUM_26, GPIO_MODE_OUTPUT);
}

void app_main() {
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_2, 1);
    init_lights();
    // init_esp_01(UART_NUM_2, GPIO_NUM_19);
    // esp_01_connect_wifi(UART_NUM_2, wifi_name, wifi_password);
    // esp_01_send_http(
    //     UART_NUM_2, 
    //     server_ip, 
    //     server_port, 
    //     "GET /users HTTP/1.1\r\nHost: 192.168.87.178\r\n"
    // );

    // esp_01_send_http(
    //     UART_NUM_2, 
    //     server_ip, 
    //     server_port, 
    //     "GET / HTTP/1.1\r\nHost: 192.168.87.178\r\n"
    // );

    init_esp_01_server(UART_NUM_2, GPIO_NUM_19, wifi_name, wifi_password);

    init_adxl345(22,21);
    int16_t data[] = {0,0,0};

    while (true){
        uint result = esp_01_IPD(UART_NUM_2, "HTTP/1.1", 2000);
        if(result > 0){
            printf("The result is : %d\n", result);
            // send response
            esp_01_server_OK(UART_NUM_2, 0);
        }
        adxl345_get_axis_readings(data);

        printf("X= %d", data[0]);
        printf(" Y= %d", data[1]);
        printf(" Z= %d\n", data[2]);
        

        gpio_set_level(GPIO_NUM_2, 1);
        gpio_set_level(GPIO_NUM_32, 1);
        gpio_set_level(GPIO_NUM_33, 1);
        gpio_set_level(GPIO_NUM_25, 1);
        gpio_set_level(GPIO_NUM_26, 1);
        
        vTaskDelay(500 / portTICK_RATE_MS);

        gpio_set_level(GPIO_NUM_2, 0);
        gpio_set_level(GPIO_NUM_32, 0);
        gpio_set_level(GPIO_NUM_33, 0);
        gpio_set_level(GPIO_NUM_25, 0);
        gpio_set_level(GPIO_NUM_26, 0);
        vTaskDelay(500 / portTICK_RATE_MS);
    }
}