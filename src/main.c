#include <stdio.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "../lib/esp_01/esp_01.h"
#include "../lib/adxl345/adxl345.h"

// ESP32 DevkitC v4 // ESP-WROOM-32D
// 160 Mhz

// char *wifi_name = "Stofa70521";
// char *wifi_password = "bis56lage63";
// char *server_ip = "192.168.87.178";
// char *server_port = "3500";

char *wifi_name = "ESP32_wifi";
char *wifi_password = "1234567890";
char *server_ip = "192.168.8.1";
char *server_port = "3500";

void init_lights(){
        gpio_set_direction(GPIO_NUM_32, GPIO_MODE_OUTPUT);
        gpio_set_direction(GPIO_NUM_33, GPIO_MODE_OUTPUT);
        gpio_set_direction(GPIO_NUM_25, GPIO_MODE_OUTPUT);
        gpio_set_direction(GPIO_NUM_26, GPIO_MODE_OUTPUT);
}

void extract_request_values(char *request, uint request_size, uint *x, uint *y){
    uint x_index = 1;
    uint x_end_index = 0;

    for(uint i = 1; i < request_size; i++){
        if(request[i] == '/'){
            x_end_index = i;
            break;
        }
    }

    uint y_index = x_end_index+1;
    uint y_end_index = request_size-1;

    uint x_length = x_end_index - x_index;
    char x_substring[x_length+1];
    strncpy(x_substring, &request[x_index], x_length);
    x_substring[x_length] = '\0';
    // printf("X IS : %s\n", x_substring);
    *x = atoi(x_substring);

    uint y_length = y_end_index - y_index+1;
    char y_substring[y_length+1];
    strncpy(y_substring, &request[y_index], y_length);
    y_substring[y_length] = '\0';
    // printf("Y IS : %s\n", y_substring);
    *y = atoi(y_substring);

}

void manipulate_leds(uint x, uint y){

    // blue - 32 low y
    // red - 33 high x
    // yellow - 25 high y
    // green - 26 low x


    // 32 up 25 down
    if(x < 40){
        gpio_set_level(GPIO_NUM_26, 1);
        gpio_set_level(GPIO_NUM_33, 0);
    }else if(x > 60){
        gpio_set_level(GPIO_NUM_33, 1);
        gpio_set_level(GPIO_NUM_26, 0);
    }else{
        gpio_set_level(GPIO_NUM_26, 0);
        gpio_set_level(GPIO_NUM_33, 0);
    }

    // 33 left 26 right
    if(y < 40){
        gpio_set_level(GPIO_NUM_32, 1);
        gpio_set_level(GPIO_NUM_25, 0);
    }else if(y > 60){
        gpio_set_level(GPIO_NUM_25, 1);
        gpio_set_level(GPIO_NUM_32, 0);
    }else{
        gpio_set_level(GPIO_NUM_32, 0);
        gpio_set_level(GPIO_NUM_25, 0);
    }
}

void app_main() {
    printf("STARTING PROGRAM\n");
    // status led
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_2, 1);

    init_lights();

    init_esp_01_server(UART_NUM_2, GPIO_NUM_19, wifi_name, wifi_password, server_port, server_ip, false);

    init_adxl345(22,21);
    int16_t data[] = {0,0,0};
    uint x = 50;
    uint y = 50;

    printf("\n\n====START OF LOOP====\n\n");
    while (true){
        char buffer[1024];

        uint result = esp_01_server_IPD(UART_NUM_2, "HTTP/1.1", 2000, buffer, false);

        if(result > 0){
            uint connection_id = 999;
            uint request_size = 999;
            char *request = esp_01_trim_response(buffer, 1024, &connection_id, &request_size);
            esp_01_server_OK(UART_NUM_2, connection_id);
            extract_request_values(request, request_size, &x, &y);
            free(request);
            printf("Transmission x:%d y:%d\n", x, y);
        }
        // use the information to set the leds 
        manipulate_leds(x, y);

        // accelerometer
        adxl345_get_axis_readings(data);
        printf("X= %d", data[0]);
        printf(" Y= %d", data[1]);
        printf(" Z= %d\n", data[2]);
        
        // status led
        gpio_set_level(GPIO_NUM_2, 1);
        vTaskDelay(30 / portTICK_RATE_MS);
        gpio_set_level(GPIO_NUM_2, 0);
        vTaskDelay(30 / portTICK_RATE_MS);
    }
}