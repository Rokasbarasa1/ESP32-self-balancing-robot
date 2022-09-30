#include <stdio.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "../lib/esp_01/esp_01.h"
// #include "../lib/adxl345/adxl345.h"
#include "../lib/mpu6050/mpu6050.h"
#include "../lib/TB6612/TB6612.h"
#include "../lib/gy271_qmc5883l/gy271.h"

// const float mag_field_norm = 50.503

// ESP32 DevkitC v4 // ESP-WROOM-32D
// 160 Mhz

char *wifi_name = "ESP32_wifi";
char *wifi_password = "1234567890";
char *server_ip = "192.168.8.1";
char *server_port = "3500";

float refresh_rate_hz = 100;

// Used method found in MicWro Engr Video:
// https://www.youtube.com/watch?v=MinV5V1ioWg
// Mag field norm at my location is 50.503
// use software called Magneto 1.2

float hard_iron[3] = {
    224.858620,363.804991,489.334513
};

float soft_iron[3][3] = {
    {1.066046,0.019169,-0.033308},
    {0.019169,0.400514,-0.073532},
    {-0.033308,-0.073532,1.223357}
};

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
    float *integral_sum // Pass reference to it
){
    float error_x = 0, error_y = 0, error_z = 0, total_error = 0;
    float error_z_p = 0, error_z_i = 0, error_z_d = 0;

    float motor_data = 0;
    // error_x = desired_accel_x - acceleration_data[0]; // for mpu6050 use x axis
    // error_y = desired_accel_y - acceleration_data[1]; // for adxl345 use the y axis of accelerometer
    // error_z = desired_accel_z - acceleration_data[2];

    error_x = (desired_gyro_x - gyro_degrees_x)/ -90.0; 
    error_y = (desired_gyro_y - gyro_degrees_y)/ -90.0; 
    error_z = (desired_gyro_z - gyro_degrees_z)/ -90.0;

    // if(error_x > 0){
    //     error_z *= 1;
    // }else if(error_x < 0){
    //     error_z *= -1;
    // }else{
    //     error_z = 0;
    // }

    // proportional
    {
        // error_z_p = error_z;
        error_z_p = error_y;
    }

    // integral
    {
        // integral_sum += error_z;
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
        error_z_d = 0;
    }

    // end result
    total_error = (gain_p * error_z_p) + (gain_i * error_z_i) + (gain_d * error_z_d);
    // printf("    %10.2f = %10.2f + %10.2f + %10.2f    ",total_error, (gain_p * error_z_p), (gain_i * error_z_i), (gain_d * error_z_d));


    motor_data = total_error * 100.0;
    change_speed_motor_B(motor_data, 27);
    change_speed_motor_A(motor_data, 27);
    // printf("          Error %10.2f  Motor value: %12.2f\n", total_error,motor_data);
}

void get_ned_coordinates(float* acceleration_data, float* magnetometer_data, float* north_vector, float* east_vector, float* down_vector){
    // get 2d compass heading

    // get unit vector of acceleration
    // get length of vector

    // acceleration_data[0] = -0.02;
    // acceleration_data[1] = 0.00;
    // acceleration_data[2] = 1.01;

    // magnetometer_data[0] = 6.29;
    // magnetometer_data[1] = -14.48;
    // magnetometer_data[2] = -45.75;

    // printf("\n\n");
    // printf("A, %6.2f, %6.2f, %6.2f, \n", acceleration_data[0], acceleration_data[1], acceleration_data[2]);
    // printf("M, %6.2f, %6.2f, %6.2f, \n", magnetometer_data[0], magnetometer_data[1], magnetometer_data[2]);

    const float acc_vector_length = sqrt(pow(acceleration_data[0], 2) + pow(acceleration_data[1], 2) + pow(acceleration_data[2], 2));
    float acc_unit_vector[] = {
        acceleration_data[0] / acc_vector_length,
        acceleration_data[1] / acc_vector_length,
        acceleration_data[2] / acc_vector_length
    };
    // printf("A_unit, %6.2f, %6.2f, %6.2f, \n", acc_unit_vector[0], acc_unit_vector[1], acc_unit_vector[2]);


    // get unit vector of magnetometer 
    // get length of vector
    const float mag_vector_length = sqrt(pow(magnetometer_data[0], 2) + pow(magnetometer_data[1], 2) + pow(magnetometer_data[2], 2));
    float mag_unit_vector[] = {
          magnetometer_data[0] / mag_vector_length,
          magnetometer_data[1] / mag_vector_length,
          magnetometer_data[2] / mag_vector_length
    };
    // printf("M_unit, %6.2f, %6.2f, %6.2f, \n", mag_unit_vector[0], mag_unit_vector[1], mag_unit_vector[2]);

    // inverse of accel unit
    float down_direction[] = {
        -acc_unit_vector[0],
        -acc_unit_vector[1],
        -acc_unit_vector[2]
    };

    // printf("D, %6.2f, %6.2f, %6.2f, \n", down_direction[0], down_direction[1], down_direction[2]);

    // cross product of direction of gravity and mag direction unit vector
    float east_direction[] ={
        down_direction[1] * mag_unit_vector[2] - down_direction[2] * mag_unit_vector[1],
        down_direction[2] * mag_unit_vector[0] - down_direction[0] * mag_unit_vector[2],
        down_direction[0] * mag_unit_vector[1] - down_direction[1] * mag_unit_vector[0],
    };
    // printf("E, %6.2f, %6.2f, %6.2f, \n", east_direction[0], east_direction[1], east_direction[2]);

    // east normalization and turning into unit vector
    const float east_vector_length = sqrt(pow(east_direction[0], 2) + pow(east_direction[1], 2) + pow(east_direction[2], 2));
    float east_unit_direction[] = {
        east_direction[0] / east_vector_length,
        east_direction[1] / east_vector_length,
        east_direction[2] / east_vector_length
    };
    // printf("E_unit, %6.2f, %6.2f, %6.2f, \n", east_unit_direction[0], east_unit_direction[1], east_unit_direction[2]);

    // cross east and direction of gravity
    float north_direction[] ={
        east_unit_direction[1] * down_direction[2] - east_unit_direction[2] * down_direction[1],
        east_unit_direction[2] * down_direction[0] - east_unit_direction[0] * down_direction[2],
        east_unit_direction[0] * down_direction[1] - east_unit_direction[1] * down_direction[0],
    };
    // printf("N, %6.2f, %6.2f, %6.2f, \n", north_direction[0], north_direction[1], north_direction[2]);

    const float north_vector_length = sqrt(pow(north_direction[0], 2) + pow(north_direction[1], 2) + pow(north_direction[2], 2));
    
    north_vector[0] = north_direction[0] / north_vector_length;
    north_vector[1] = north_direction[1] / north_vector_length;
    north_vector[2] = north_direction[2] / north_vector_length;
    // printf("N_unit, %6.2f, %6.2f, %6.2f, \n", north_unit_vector[0], north_unit_vector[1], north_unit_vector[2]);
    // printf("\n");

    east_vector[0] = east_unit_direction[0];
    east_vector[1] = east_unit_direction[1];
    east_vector[2] = east_unit_direction[2];

    down_vector[0] = down_direction[0];
    down_vector[1] = down_direction[1];
    down_vector[2] = down_direction[2];
}

void fix_mag_axis(float* magnetometer_data){
    float temp = 0.0;
    temp = magnetometer_data[0];
    // y is x
    magnetometer_data[0] = magnetometer_data[1];
    // x is -y
    magnetometer_data[1] = -temp;

}

float angle_between_2d_vectors(float ax, float ay, float bx, float by ){
    float a_length = sqrt(pow(ax, 2) + pow(ay, 2));
    float b_length = sqrt(pow(bx, 2) + pow(by, 2));

    ax = ax /a_length;
    ax = ax /a_length;

    bx = bx /b_length;
    bx = bx /b_length;

    return atan2(
        (ax * by - ay * bx), 
        (ax * bx + ay * by)
    )*180/M_PI;
}

void app_main() {
    printf("STARTING PROGRAM\n");

    init_mpu6050(22,21, true);
    init_gy271(22,21, false, hard_iron, soft_iron);
    init_TB6612(GPIO_NUM_33,GPIO_NUM_32, GPIO_NUM_25, GPIO_NUM_27, GPIO_NUM_26, GPIO_NUM_14);

    float acceleration_data[] = {0,0,0};
    float gyro_angular[] = {0,0,0};
    float gyro_degrees[] = {0,0,0};
    float magnetometer_data[] = {0,0,0};

    //The goal of the PID is to reach 
    // x = 0 (dont care about this one)
    // y = 0 will let decide direction in which to spin wheels
    // z = 1 how fast the wheels have to spin

    float desired_accel_x = 0.0, desired_accel_y = 0.0, desired_accel_z = 1.0;
    float desired_gyro_x = 0.0, desired_gyro_y = 0.0, desired_gyro_z = 1.0;

    // proportional gain
    float gain_p = 0.8;

    // integration gain
    float gain_i = 0.1;
    float integral_sum = 0.0;

    // derivative gain
    float gain_d = 0;
    bool first_load = true;

    // Find calibration values for gyro and accel
    // vTaskDelay(2000 / portTICK_RATE_MS);
    find_accelerometer_error(1000);
    find_gyro_error(1000);
    // find_mag_error(1000);
    float complementary_ratio = 0.0001;
    printf("\n\n====START OF LOOP====\n\n");
    while (true){
        // read sensor values
        mpu6050_accelerometer_readings_float(acceleration_data);
        mpu6050_gyro_readings_float(gyro_angular);
        gy271_magnetometer_readings_micro_teslas(magnetometer_data);
        fix_mag_axis(magnetometer_data);
        
        float pitch = 0;
        float roll = 0;
        float yaw = 0;
        calculate_pitch_and_roll(acceleration_data, &roll, &pitch);
        if(first_load){
            gyro_degrees[0] = -roll;
            gyro_degrees[1] = -pitch;
            first_load = false;
        }


        // Convert angular velocity to actual degrees that it moved and add it to the integral (dead reckoning not PID)
        gyro_degrees[0] += (gyro_angular[0] * (1.0/refresh_rate_hz)) * (1-complementary_ratio) + (complementary_ratio * roll);
        gyro_degrees[1] += (gyro_angular[1] * (1.0/refresh_rate_hz)) * (1-complementary_ratio) + (complementary_ratio * pitch);
        gyro_degrees[2] += (gyro_angular[2] * (1.0/refresh_rate_hz));


        // float north_direction[] = {0,0,0}; // x
        // float east_direction[] = {0,0,0}; // y
        // float down_direction[] = {0,0,0}; // z

        
        

        // get_ned_coordinates(acceleration_data, magnetometer_data, north_direction, east_direction, down_direction);

        // pitch = angle_between_2d_vectors(north_direction[1], north_direction[2], 1, 0); // around x // use y and z // 
        // roll = angle_between_2d_vectors(north_direction[0], north_direction[2], 0, 1); // around y // use x and z
        // yaw = angle_between_2d_vectors(north_direction[0], north_direction[1], 1, 0); // around z // use x and y
        printf("PITCH, %6.2f, ROLL %6.2f,  ", pitch, roll);

        printf("ACCEL, %6.2f, %6.2f, %6.2f, ", acceleration_data[0], acceleration_data[1], acceleration_data[2]);
        printf("GYRO, %6.2f, %6.2f, %6.2f, \n", gyro_degrees[0], gyro_degrees[1], gyro_degrees[2]);
        // printf("MAG, %6.2f, %6.2f, %6.2f, ", magnetometer_data[0], magnetometer_data[1], magnetometer_data[2]);
        // printf("NORTH, %6.2f, %6.2f, %6.2f, ", north_direction[0], north_direction[1], north_direction[2]);
        // printf("GYRO, %6.2f, %6.2f, %6.2f, \n", pitch, roll, yaw);

        // Use complimentary filter of kalman filter to combine all the measurements










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
            &integral_sum // Pass reference to it
        );

        // printf(
        //     "%f,%f,%f\n", 
        //     magnetometer_data[0], 
        //     magnetometer_data[1], 
        //     magnetometer_data[2]
        // );
        // Refresh rate must not be more than 1 KHz 
        // That is max for MPU6050
        vTaskDelay((1000/ refresh_rate_hz) / portTICK_RATE_MS);
    }
}


