#include <stdio.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "../lib/esp_01/esp_01.h"
// #include "../lib/adxl345/adxl345.h"
#include "../lib/mpu6050/mpu6050.h"
#include "../lib/TB6612/TB6612.h"

// ESP32 DevkitC v4 // ESP-WROOM-32D
// 160 Mhz

char *wifi_name = "ESP32_wifi";
char *wifi_password = "1234567890";
char *server_ip = "192.168.8.1";
char *server_port = "3500";

const float refresh_rate_hz = 100;

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

void app_main() {
    printf("STARTING PROGRAM\n");
    // status led
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_2, 1);

    init_mpu6050(22,21);
    init_TB6612(GPIO_NUM_33,GPIO_NUM_32, GPIO_NUM_25, GPIO_NUM_27, GPIO_NUM_26, GPIO_NUM_14);

    float acceleration_data[] = {0,0,0};
    float gyro_angular[] = {0,0,0};
    float gyro_degrees[] = {0,0,0};

    //The goal of the PID is to reach 
    // x = 0 (dont care about this one)
    // y = 0 will let decide direction in which to spin wheels
    // z = 1 how fast the wheels have to spin

    float desired_accel_x = 0.0, desired_accel_y = 0.0, desired_accel_z = 1.0;
    float desired_gyro_x = 0.0, desired_gyro_y = 0.0, desired_gyro_z = 1.0;

    // proportional gain
    float gain_p = 0.7;

    // integration gain
    float gain_i = 0.1;
    float integral_sum = 0.0;

    // derivative gain
    float gain_d = 0;

    // Find calibration values for gyro and accel
    // vTaskDelay(2000 / portTICK_RATE_MS);
    // find_accelerometer_error(1000);
    // find_gyro_error(1000);



    printf("\n\n====START OF LOOP====\n\n");
    
    while (true){
        mpu6050_accelerometer_readings_float(acceleration_data);
        printf("ACCEL, %6.2f, %6.2f, %6.2f, ", acceleration_data[0], acceleration_data[1], acceleration_data[2]);

        mpu6050_gyro_readings_float(gyro_angular);

        // Convert angular velocity to actual degrees
        gyro_degrees[0] += (gyro_angular[0] * (1.0/refresh_rate_hz));
        gyro_degrees[1] += (gyro_angular[1] * (1.0/refresh_rate_hz));
        gyro_degrees[2] += (gyro_angular[2] * (1.0/refresh_rate_hz));
        
        printf("GYRO, %6.2f, %6.2f, %6.2f, \n", gyro_degrees[0], gyro_degrees[1], gyro_degrees[2]);

        float error_x = 0, error_y = 0, error_z = 0, total_error = 0;
        float error_z_p = 0, error_z_i = 0, error_z_d = 0;

        float motor_data = 0;
        // error_x = desired_accel_x - acceleration_data[0]; // for mpu6050 use x axis
        // error_y = desired_accel_y - acceleration_data[1]; // for adxl345 use the y axis of accelerometer
        // error_z = desired_accel_z - acceleration_data[2];

        error_x = (desired_gyro_x - gyro_degrees[0])/ -90.0; 
        error_y = (desired_gyro_y - gyro_degrees[1])/ -90.0; 
        error_z = (desired_gyro_z - gyro_degrees[2])/ -90.0;

        // if(error_x > 0){
        //     error_z *= 1;
        // }else if(error_x < 0){
        //     error_z *= -1;
        // }else{
        //     error_z = 0;
        // }

        // if(error_y > 0){
        //     error_y *= 1;
        // }else if(error_y < 0){
        //     error_y *= -1;
        // }else{
        //     error_y = 0;
        // }
        
        // proportional
        {
            // error_z_p = error_z;
            error_z_p = error_y;
        }

        // integral
        {
            // integral_sum += error_z;
            integral_sum += error_y;

            // clamp the integral if it is getting out of bounds
            if(integral_sum > 1.0){
                integral_sum = 1.0;
            }else if(integral_sum < -1.0){
                integral_sum = -1.0;
            }
            error_z_i = integral_sum;
        }

        // derivative
        {
            error_z_d = 0;
        }

        // end result
        total_error = (gain_p * error_z_p) + (gain_i * error_z_i) + (gain_d * error_z_d);
        // printf("    %10.2f = %10.2f + %10.2f + %10.2f    ",total_error, (gain_p * error_z_p), (gain_i * error_z_i), (gain_d * error_z_d));


        motor_data = total_error * 100.0;
        change_speed_motor_B(motor_data, 27);
        change_speed_motor_A(motor_data, 27);
        // printf("          Error %10.2f  Motor value: %12.2f\n", total_error,motor_data);

        // Refresh rate must not be more than 1 KHz 
        // That is max for MPU6050
        vTaskDelay((1000/ refresh_rate_hz) / portTICK_RATE_MS);
    }
}
