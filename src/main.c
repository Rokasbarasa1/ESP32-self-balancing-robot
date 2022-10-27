#include <stdio.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/spi_master.h"

// Drivers
#include "../lib/esp_01/esp_01.h"
#include "../lib/mpu6050/mpu6050.h"
#include "../lib/TB6612/TB6612.h"
#include "../lib/gy271_qmc5883l/gy271.h"
#include "../lib/nrf24l01/nrf24l01.h"
#include "../lib/esp_01/esp_01.h" // I wrote variable extraction function earlier, im using that from it

// Other imports
#include "../lib/util/ned_coordinates/ned_coordinates.h"
#include "../lib/util/spi/spi.h"

// ESP32 DevkitC v4 - ESP-WROOM-32D - 160 Mhz

// For calibrating the magnetometer I
// used a method found in MicWro Engr Video:
// https://www.youtube.com/watch?v=MinV5V1ioWg
// Mag field norm at my location is 50.503
// use software called Magneto 1.2

float hard_iron_correction[3] = {
    185.447609, 360.541288, 491.294615
};
float soft_iron_correction[3][3] = {
    {1.001470, 0.025460, -0.035586},
    {0.025460, 0.405497, -0.054355},
    {-0.035586, -0.054355, 1.219251}
};

float accelerometer_correction[3] = {
    0.068885,0.054472,0.952431
};
float gyro_correction[3] = {
    0.208137,-4.056841,0.413817
};

float accelerometer_correction_no_cable[3] = {
    -0.020629, -0.023295, 1.005922
};
float gyro_correction_no_cable[3] = {
    0.105298, -0.114373, 0.077229
};

bool using_cable = true;

const float desired_accel_x = 0.0, desired_accel_y = 0.0, desired_accel_z = 1.0;
const float desired_gyro_x = 0.0, desired_gyro_y = 0.0, desired_gyro_z = 1.0;
const float refresh_rate_hz = 100;
const float balance_margin = 1.0;

const float complementary_ratio = 0.02;

// PID gains
const float gain_p = 3; 
const float gain_i = 0.5;
const float gain_d = 15;

uint8_t tx_address[5] = {0xEE, 0xDD, 0xCC, 0xBB, 0xAA};
uint8_t rx_data[32];

void influence_motors_with_PID(
    float gain_p,
    float gain_i,
    float gain_d,
    float desired_gyro_x,
    float desired_gyro_y,
    float desired_gyro_z,
    float desired_accel_x,
    float desired_accel_y,
    float desired_accel_z,
    float gyro_degrees_x,
    float gyro_degrees_y,
    float gyro_degrees_z,
    float *integral_sum, // Pass reference to it
    float *last_error,
    float elapsed_time,
    float balance_margin
);
// Switches the magnetometer axis to be like accelerometer and gyro
void fix_mag_axis(float* magnetometer_data);

void app_main() {
    printf("STARTING PROGRAM\n");
    init_spi3();

    if(using_cable){
        init_mpu6050(22,21, true, true, accelerometer_correction, gyro_correction);
    }else{
        init_mpu6050(22,21, true, true, accelerometer_correction_no_cable, gyro_correction_no_cable);
    }

    init_gy271(22,21, false, true, hard_iron_correction, soft_iron_correction);
    init_TB6612(GPIO_NUM_33,GPIO_NUM_32, GPIO_NUM_25, GPIO_NUM_27, GPIO_NUM_26, GPIO_NUM_14);

    bool nrf24_setup = nrf24_init(SPI3_HOST, 17, 16);
    nrf24_rx_mode(tx_address, 10);


    float acceleration_data[] = {0,0,0};
    float gyro_angular[] = {0,0,0};
    float gyro_degrees[] = {0,0,0};
    float magnetometer_data[] = {0,0,0};

    // integration
    float integral_sum = 0.0;
    // derivative
    float last_error = 0;
    bool first_load = true;

    float pitch = 0;
    float roll = 0;
    float yaw = 0;

    // Joystick values, 50 is basically zero position
    uint x = 50;
    uint y = 50;

    // Find calibration values for gyro and accel
    // vTaskDelay(2000 / portTICK_RATE_MS);
    // find_accelerometer_error(1000);
    // find_gyro_error(1000);


    printf("\n\n====START OF LOOP====\n\n");
    while (true){

         if(nrf24_data_available(1)){
            nrf24_receive(rx_data);
            for(uint8_t i = 0; i < strlen((char*) rx_data); i++ ){
                printf("%c", ((char*) rx_data)[i]);
            }
            printf("\n");

            extract_request_values((char*) rx_data, strlen((char*) rx_data), &x, &y);
            printf("Dat %d %d\n", x, y);
        }


        // read sensor values
        mpu6050_accelerometer_readings_float(acceleration_data);
        mpu6050_gyro_readings_float(gyro_angular);
        gy271_magnetometer_readings_micro_teslas(magnetometer_data);
        fix_mag_axis(magnetometer_data);
        
        // NED coordinates probably not needed
        float north_direction[] = {0,0,0};  // x
        float east_direction[] = {0,0,0};   // y
        float down_direction[] = {0,0,0};   // z

        get_ned_coordinates(acceleration_data, magnetometer_data, north_direction, east_direction, down_direction);
        calculate_pitch_and_roll(acceleration_data, &roll, &pitch);
        calculate_yaw(magnetometer_data, &yaw);

        // Find the initial position in degrees and apply it to the gyro measurement integral
        // This will tell the robot which way to go to get the actual upward
        if(first_load){
            gyro_degrees[0] = -roll;
            gyro_degrees[1] = -pitch;
            gyro_degrees[2] = -yaw;
            printf("Initial locaton x: %.2f y: %.2f, z: %.2f\n", gyro_degrees[0], gyro_degrees[1], gyro_degrees[2]);
            first_load = false;
        }
        
        // Convert angular velocity to actual degrees that it moved and add it to the integral (dead reckoning not PID)
        gyro_degrees[0] += (gyro_angular[0] * (1.0/refresh_rate_hz));
        gyro_degrees[1] += (gyro_angular[1] * (1.0/refresh_rate_hz));
        gyro_degrees[2] += (gyro_angular[2] * (1.0/refresh_rate_hz));

        // Apply complimentary filter
        gyro_degrees[0] = (gyro_degrees[0] * (1-complementary_ratio)) + (complementary_ratio * (-roll));
        gyro_degrees[1] = (gyro_degrees[1] * (1-complementary_ratio)) + (complementary_ratio * (-pitch));
        gyro_degrees[2] = (gyro_degrees[2] * (1-complementary_ratio)) + (complementary_ratio * (-yaw));

        // Used for telemetry monitor that I made
        // printf("ACCEL, %6.2f, %6.2f, %6.2f, ", acceleration_data[0], acceleration_data[1], acceleration_data[2]);
        // printf("GYRO, %6.2f, %6.2f, %6.2f, ", gyro_degrees[0], gyro_degrees[1], gyro_degrees[2]);
        // printf("MAG, %6.2f, %6.2f, %6.2f, ", magnetometer_data[0], magnetometer_data[1], magnetometer_data[2]);
        // printf("NORTH, %6.2f, %6.2f, %6.2f, ", north_direction[0], north_direction[1], north_direction[2]);
        // Use complimentary filter or kalman filter to combine all the measurements

        influence_motors_with_PID(
            gain_p,
            gain_i,
            gain_d,
            desired_gyro_x,
            desired_gyro_y,
            desired_gyro_z,
            desired_accel_x,
            desired_accel_y,
            desired_accel_z,
            gyro_degrees[0],
            gyro_degrees[1],
            gyro_degrees[2],
            &integral_sum, // Pass reference to it
            &last_error,
            1000.0/refresh_rate_hz,
            balance_margin
        );

        // printf(
        //     "%f,%f,%f", 
        //     magnetometer_data[0], 
        //     magnetometer_data[1], 
        //     magnetometer_data[2]
        // );
        // Refresh rate must not be more than 1 KHz 
        // That is max for MPU6050
        // printf("\n"); 
        vTaskDelay((1000/ refresh_rate_hz) / portTICK_RATE_MS);
    }
}


void influence_motors_with_PID(
    float gain_p,
    float gain_i,
    float gain_d,
    float desired_gyro_x,
    float desired_gyro_y,
    float desired_gyro_z,
    float desired_accel_x,
    float desired_accel_y,
    float desired_accel_z,
    float gyro_degrees_x,
    float gyro_degrees_y,
    float gyro_degrees_z,
    float *integral_sum, // Pass reference to it
    float *last_error,
    float elapsed_time,
    float balance_margin
){
    
    //The goal of the PID is to reach 
    // x = 0 (dont care about this one)
    // y = 0 will let decide direction in which to spin wheels
    // z = 1 how fast the wheels have to spin
    float error_x = 0, error_y = 0, error_z = 0, total_error = 0;
    float error_z_p = 0, error_z_i = 0, error_z_d = 0;

    float motor_data = 0;

    error_x = (desired_gyro_x - gyro_degrees_x)/ -90.0;
    error_y = (desired_gyro_y - gyro_degrees_y)/ -90.0;
    error_z = (desired_gyro_z - gyro_degrees_z)/ -90.0;
    

    // proportional
    {
        error_z_p = error_y;
    }

    // integral
    {
        *integral_sum += error_y;

        // clamp the integral if it is getting out of bounds
        if(*integral_sum > 1.0){
            *integral_sum = 1.0;
        }else if(*integral_sum < -1.0){
            *integral_sum = -1.0;
        }
        error_z_i = *integral_sum;
    }

    // derivative
    {
        error_z_d = (error_z_p - *last_error);///elapsed_time;
        // printf("Deriv %f", error_z_d);
    }

    // end result
    total_error = (gain_p * error_z_p) + (gain_i * error_z_i) + (gain_d * error_z_d);
    *last_error = error_y;

    // printf("  ERRORS  %10.2f = %10.2f + %10.2f + %10.2f    ",total_error, (gain_p * error_z_p), (gain_i * error_z_i), (gain_d * error_z_d));


    motor_data = total_error * 100.0;
    // The a motor is now the b motor RENAME THESE FUNCTION
    if(motor_data > balance_margin || motor_data < -balance_margin){
        change_speed_motor_A(motor_data, 27-balance_margin*1.5); // this wheel responds faster
        change_speed_motor_B(motor_data, 27-balance_margin*1.5);
        // printf("          Error %10.2f  Motor value: %12.2f    ", total_error,motor_data);
    }else{
        change_speed_motor_A(0, 27);
        change_speed_motor_B(0, 27);
    }
    
}

void fix_mag_axis(float* magnetometer_data){
    float temp = 0.0;
    temp = magnetometer_data[0];
    // y is x
    magnetometer_data[0] = magnetometer_data[1];
    // x is -y
    magnetometer_data[1] = -temp;
}