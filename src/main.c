#include <stdio.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "../lib/esp_01/esp_01.h"
#include "../lib/adxl345/adxl345.h"
#include "../lib/TB6612/TB6612.h"

// ESP32 DevkitC v4 // ESP-WROOM-32D
// 160 Mhz

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
    *x = atoi(x_substring);

    uint y_length = y_end_index - y_index+1;
    char y_substring[y_length+1];
    strncpy(y_substring, &request[y_index], y_length);
    y_substring[y_length] = '\0';
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

void manipulate_motors(uint x, uint y){
    
    if(y < 40){
        change_speed_motor_B(((int)y-50)*2, 30);
    }else if(y > 60){
        change_speed_motor_B(((int)y-50)*2, 30);
    }else{
        change_speed_motor_B(0, 30);
    }

    if(x < 40){
        change_speed_motor_A(((int)x-50)*2, 30);
    }else if(x > 60){
        change_speed_motor_A(((int)x-50)*2, 30);
    }else{
        change_speed_motor_A(0, 30);
    }
}

void app_main() {
    printf("STARTING PROGRAM\n");
    // status led
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_2, 1);

    // init_lights();

    // init_esp_01_server(UART_NUM_2, GPIO_NUM_19, wifi_name, wifi_password, server_port, server_ip, false, true);
    init_adxl345(22,21);
    init_TB6612(GPIO_NUM_33,GPIO_NUM_32, GPIO_NUM_25, GPIO_NUM_27, GPIO_NUM_26, GPIO_NUM_14);

    float acceleration_data[] = {0,0,0};

    float roll = 0;
    float pitch = 0;
    
    uint x = 50;
    uint y = 50;
    
    //The goal of the PID is to reach 
    // x = 0 (dont care about this one)
    // y = 0 will let decide direction in which to spin wheels
    // z = 1 how fast the wheels have to spin

    float desired_accel_x = 0.0, desired_accel_y = 0.0, desired_accel_z = 1.0;
    uint error_history_size = 20;
    uint error_history_index = 0;
    // proportional gain
    float gain_p = 1;

    // integration gain
    float gain_i = 4;
    float error_history[error_history_size];

    // derivative gain
    float gain_d = 1;

    // int speed = 0;
    // int speed_limit = 100;
    
    printf("\n\n====START OF LOOP====\n\n");
    while (true){
        // char buffer[1024];
        // uint result = esp_01_server_IPD(UART_NUM_2, "HTTP/1.1", 20, buffer, false);
 
        // if(result > 0 && result < 1025){
        //     printf("result %d\n", result);
        //     uint connection_id = 999;
        //     uint request_size = 999;
        //     char *request = esp_01_trim_response(buffer, 1024, &connection_id, &request_size);
        //     esp_01_server_OK(UART_NUM_2, connection_id);
        //     extract_request_values(request, request_size, &x, &y);
        //     free(request);
        //     printf("Transmission x:%d y:%d\n", x, y);
        // }else if(result == 9999){
        //     // this means the esp01 fucked up something. DO RESET
        //     init_esp_01_server(UART_NUM_2, GPIO_NUM_19, wifi_name, wifi_password, server_port, server_ip, false, false);
        // }

        // manipulate_motors(x, y);
        
        // adxl345_get_axis_readings_float(acceleration_data);
        // printf("Data X= %10.4f Y= %10.4f Z= %10.4f", acceleration_data[0], acceleration_data[1], acceleration_data[2]);

        // float error_x = 0, error_y = 0, error_z = 0, total_error = 0;
        // float error_z_p = 0, error_z_i = 0, error_z_d = 0;

        // float motor_data = 0;
        // error_x = desired_accel_x - acceleration_data[0];
        // error_y = desired_accel_y - acceleration_data[1];
        // // Use value of y error to set this negative or positive
        // error_z = desired_accel_z - acceleration_data[2];

        // if(error_y > 0){
        //     error_z *= 1;
        // }else if(error_y < 0){
        //     error_z *= -1;
        // }else{
        //     error_z = 0;
        // }
        
        // // proportional
        // {
        //     error_z_p = error_z;
        // }

        // // integral
        // {
        //     // The order doesn't matter 
        //     // overwriting it works good
        //     error_history[error_history_index] = error_z;
        //     error_history_index++;
        //     error_history_index = error_history_index % error_history_size;
        //     // i guess divide the sum of history by the amount of history
        //     error_z_i = 0;

        //     for(uint i = 0; i < error_history_size; i++){
        //         error_z_i += error_history[error_history_index];
        //     }

        //     error_z_i /= error_history_size;
        // }
        


        // // derivative
        // {
        //     error_z_d = 0;
        // }

        // // end result
        // total_error = (gain_p * error_z_p) + (gain_i * error_z_i) + (gain_d * error_z_d);

        // motor_data = total_error * 100.0;
        // change_speed_motor_B(motor_data, 27);
        // change_speed_motor_A(motor_data, 27);
        // printf("          Error %10.4f  Motor value: %12.4f\n", total_error,motor_data);




        // printf("Eik tu nahui  ");

        printf("Speed\n");
        change_speed_motor_B(100, 27);
        change_speed_motor_A(-100, 27);
        // speed++;
        // speed = speed % speed_limit;


        // calculate_pitch_and_roll(acceleration_data, &roll, &pitch);
        // printf("Roll: %.2f   Pitch %.2f\n", roll, pitch);
        // printf("\n");
        // printf("\n");

        // status led
        gpio_set_level(GPIO_NUM_2, 1);
        vTaskDelay(1000 / portTICK_RATE_MS);
        gpio_set_level(GPIO_NUM_2, 0);
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}
