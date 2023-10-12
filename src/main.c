#include <stdio.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/spi_master.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_task_wdt.h"

// Drivers
#include "../lib/esp_01/esp_01.h"
#include "../lib/mpu6050/mpu6050.h"
#include "../lib/TB6612/TB6612.h"
#include "../lib/gy271_qmc5883l/gy271.h"
#include "../lib/nrf24l01/nrf24l01.h"
#include "../lib/esp_01/esp_01.h" // I wrote variable extraction function earlier, im using that from it
#include "../lib/optical_encoder/optical_encoder.h"

// Other imports
#include "../lib/util/ned_coordinates/ned_coordinates.h"
#include "../lib/util/spi/spi.h"
#include "../lib/pid/pid.h"
#include "../lib/pid2/PID_V1.h"

#define Gravity 9.81

// Switches the magnetometer axis to be like accelerometer and gyro
void fix_mag_axis(float* magnetometer_data);
void check_calibrations();
void init_sensors();
void init_esp32_peripherals();
void init_loop_timer();
void track_time();
void get_initial_position();
void extract_joystick_request_values(char *request, uint8_t request_size, uint8_t *throttle, uint8_t *yaw, uint8_t *pitch, uint8_t *roll);
void extract_request_type(char *request, uint8_t request_size, char *type_output);
void extract_pid_request_values(char *request, uint8_t request_size, double *added_proportional, double *added_integral, double *added_derivative, double *added_master);
void send_pid_base_info_to_remote();
void send_pid_added_info_to_remote();
char* generate_message_pid_values_nrf24(double base_proportional, double base_integral, double base_derivative, double base_master);
void handle_loop_timing();
double map_value(double value, double input_min, double input_max, double output_min, double output_max);
double apply_dead_zone(double value, double max_value, double min_value, double dead_zone);
void calculate_speeds_no_yaw(float* acceleration, float* gyro_degrees, float* speed, float refresh_rate);
void calculate_speeds(float* acceleration, float* gyro_degrees, float* speed, float refresh_rate);

// ESP32 DevkitC v4 - ESP-WROOM-32D - 160 Mhz

// For calibrating the magnetometer I
// used a method found in MicWro Engr Video:
// https://www.youtube.com/watch?v=MinV5V1ioWg
// Mag field norm at my location is 50.503
// use software called Magneto 1.2

/**
 * SPI3 RADIO nRF24L01+
 * 
 * GPIO23 TX
 * GPIO18 SCK
 * GPIO19 RX
 * 
 * GPIO17 CSN
 * GPIO16 CE
 */


/**
 * TB6612
 * 
 * GPIO33 AI1
 * GPIO32 AI2
 * GPIO25 PWMA
 * GPIO27 BI1
 * GPIO26 BI2
 * GPIO14 PWMB
 * 
 */

/**
 * GY271
 * 
 * GPIO22 SCL
 * GPIO21 SDA
 * 
 */

/**
 * MPU6050
 * 
 * GPIO22 SCL
 * GPIO21 SDA
 * 
 */

/**
 * Encoder 1
 * 
 * GPIO4
 * 
 */

/**
 * Encoder 1
 * 
 * GPIO15
 * 
 */

// Calibrations
float hard_iron_correction[3] = {
    185.447609, 360.541288, 491.294615
};
float soft_iron_correction[3][3] = {
    {1.001470, 0.025460, -0.035586},
    {0.025460, 0.405497, -0.054355},
    {-0.035586, -0.054355, 1.219251}
};

// float accelerometer_correction[3] = {
//     0.030813, 0.021909, 0.952762
// };
float accelerometer_correction[3] = {
    -0.043031, 0.005, 1.011565
};
float gyro_correction[3] = {
    -6.119031, 2.084936, -0.766718
};

// handling loop timing
uint32_t loop_start_time = 0;
uint32_t loop_end_time = 0;
int16_t delta_loop_time = 0;

// Accelerometer values to degrees conversion
float accelerometer_x_rotation = 0;
float accelerometer_y_rotation = 0;
float accelerometer_z_rotation = 0;

// For radio
uint8_t tx_address[5] = {0xEE, 0xDD, 0xCC, 0xBB, 0xAA};
char rx_data[32];
char rx_type[32];

uint8_t throttle = 0;
uint8_t yaw = 50;
uint8_t pitch = 50;
uint8_t roll = 50;
bool data_received = false;

// Radio to pid target
double pitch_target = 0;
double yaw_target = 0;
double target_dead_zone_percent = 0.0;

// PID errors
double error_pitch = 0;
double error_yaw = 0;
double error_speed_x = 0;


// Actual PID adjustment for pitch
const double base_pitch_master_gain = 1.0; // 1 - 100%
const double base_pitch_gain_p = 0;
const double base_pitch_gain_i = 0;
const double base_pitch_gain_d = 0;

// Used for smooth changes to PID while using remote control
double pitch_master_gain = base_pitch_master_gain;
double pitch_gain_p = base_pitch_gain_p;
double pitch_gain_i = base_pitch_gain_i;
double pitch_gain_d = base_pitch_gain_d;

double added_pitch_master_gain = 0;
double added_pitch_gain_p = 0;
double added_pitch_gain_i = 0;
double added_pitch_gain_d = 0;

// PID for speed
const float speed_gain_p = 0.0; 
const float speed_gain_i = 0.0;
const float speed_gain_d = 0.0;

// Sensor stuff
const float complementary_ratio = 0.02;
float acceleration_data[] = {0,0,0};
float gyro_angular[] = {0,0,0};
float gyro_degrees[] = {0,0,0};
float magnetometer_data[] = {0,0,0};
float acceleration_speed[] = {0,0,0};
int8_t optical_encoder_1_result = -1;
int8_t optical_encoder_2_result = -1;
double wheels_speed[] = {0,0};

// Refresh rate
const float refresh_rate_hz = 250;

void app_main() {

    printf("STARTING PROGRAM\n"); 

    init_esp32_peripherals();
    init_sensors();
    init_loop_timer();
    // check_calibrations();
    get_initial_position();

    printf("\n\n====START OF LOOP====\n\n");
    
    struct pid pitch_pid = pid_init(pitch_master_gain * pitch_gain_p, pitch_master_gain * pitch_gain_i, pitch_master_gain * pitch_gain_d, 0.0, esp_timer_get_time(), 1000.0, -1000.0, 1);
    struct pid yaw_pid = pid_init(pitch_master_gain * pitch_gain_p, pitch_master_gain * pitch_gain_i, pitch_master_gain * pitch_gain_d, 0.0, esp_timer_get_time(), 80.0, -80.0, 1);
    struct pid wheel_speed_x_pid = pid_init(speed_gain_p, speed_gain_i, speed_gain_d, 0.0, esp_timer_get_time(), 100.0, -100, 1);


    // truct PidType pitch_pid2;
    // ID_init(&pitch_pid2, pitch_master_gain * pitch_gain_p, pitch_master_gain * pitch_gain_i, pitch_master_gain * pitch_gain_d, PID_Direction_Direct);
    // ID_SetMode(&pitch_pid2, PID_Mode_Automatic);
    // ID_SetSampleTime(&pitch_pid2, (int)(1000/refresh_rate_hz));
    // ID_SetOutputLimits(&pitch_pid2, -255.0, 255.0);

    while (true){
        // Read sensor data ######################################################################################################################


        mpu6050_get_accelerometer_readings_gravity(acceleration_data);
        mpu6050_get_gyro_readings_dps(gyro_angular);
        gy271_magnetometer_readings_micro_teslas(magnetometer_data);
        wheels_speed[0] = optical_encoder_get_hertz(optical_encoder_1_result, 2.55, -2.55);
        wheels_speed[1] = optical_encoder_get_hertz(optical_encoder_2_result, 2.55, -2.55);

        // printf("(1) ");
        // track_time();

        // Convert the sensor data to data that is useful
        fix_mag_axis(magnetometer_data); // Switches around the x and the y to match mpu6050 outputs
        calculate_degrees_x_y(acceleration_data, &accelerometer_x_rotation, &accelerometer_y_rotation);
        calculate_yaw(magnetometer_data, &accelerometer_z_rotation);
        convert_angular_rotation_to_degrees(gyro_angular, gyro_degrees, accelerometer_x_rotation, accelerometer_y_rotation, accelerometer_z_rotation, esp_timer_get_time());

        // printf("(2) ");
        // track_time();

        // Receive remote control data ###########################################################################################################

        // Initialize values to be default again
        throttle = 0;
        yaw = gyro_degrees[2];// Use current yaw as default 
        pitch = 50;
        roll = 50;
        data_received = false;

        
        if(nrf24_data_available(1)){
            nrf24_receive(rx_data);
            
            // Get the type of request
            extract_request_type(rx_data, strlen(rx_data), rx_type);
            
            printf("'%s'\n", rx_type);
            for(uint i = 0; i < strlen(rx_data); i++){
                printf("%c", rx_data[i]);
            }
            printf("\n");
            

            if(strcmp(rx_type, "joystick") == 0){
                printf("Got joystick\n");
                extract_joystick_request_values(rx_data, strlen(rx_data), &throttle, &yaw, &pitch, &roll);
            }else if(strcmp(rx_type, "pid") == 0){
                printf("Got pid\n");

                double added_proportional = 0;
                double added_integral = 0;
                double added_derivative = 0;
                double added_master_gain = 0;

                extract_pid_request_values(rx_data, strlen(rx_data), &added_proportional, &added_integral, &added_derivative, &added_master_gain);
                printf("Values %.2f\n", added_proportional);

                pitch_gain_p = base_pitch_gain_p + added_proportional;
                pitch_gain_i = base_pitch_gain_i + added_integral;
                pitch_gain_d = base_pitch_gain_d + added_derivative;
                pitch_master_gain = base_pitch_master_gain + added_master_gain;

                added_pitch_gain_p = added_proportional;
                added_pitch_gain_i = added_integral;
                added_pitch_gain_d = added_derivative;
                added_pitch_master_gain = added_master_gain;

                pid_set_proportional_gain(&pitch_pid, pitch_gain_p * pitch_master_gain);
                pid_set_integral_gain(&pitch_pid, pitch_gain_i * pitch_master_gain);
                pid_set_derivative_gain(&pitch_pid, pitch_gain_d * pitch_master_gain);
                pid_reset_integral_sum(&pitch_pid);

                // PID_SetTunings(&pitch_pid2, pitch_gain_p * pitch_master_gain, pitch_gain_i * pitch_master_gain, pitch_gain_d * pitch_master_gain);

            }else if(strcmp(rx_type, "remoteSyncBase") == 0){
                printf("Got sync base\n");
                send_pid_base_info_to_remote();
                printf("Done\n");
            }else if(strcmp(rx_type, "remoteSyncAdded") == 0){
                printf("Got sync added\n");
                send_pid_added_info_to_remote();
                printf("Done\n");
            }

            rx_type[0] = '\0'; // Clear out the string
            data_received = true;
        }

        // printf("(3) ");
        // track_time();
        // React to the data received from sensors and remote control ###########################################################################################################

        if(gyro_degrees[0] < 45 && gyro_degrees[0] > -45){

            // Update pid targets by transforming remote input
            pitch_target = map_value(pitch, 0.0, 100.0, -20.0, 20.0);
            yaw_target = map_value(yaw, -20.0, 20.0, 0.0, 100.0);
            pid_set_desired_value(&pitch_pid, pitch_target);
            pid_set_desired_value(&yaw_pid, yaw_target);

            // printf("(4) ");
            // track_time();
            // CHeck that acceleration and gravity dont boost each other. 
            // If that is the case then it is wrong. Should not add speed in those cases
            // Also add a limit that the resulting speed cannot be more than 9.8 m/s after dividing that by hertz

            calculate_speeds_no_yaw(acceleration_data, gyro_degrees, acceleration_speed, esp_timer_get_time());

            float gyro_degrees_dead_zone_adjusted[3] = {0.0,0.0,0.0};
            gyro_degrees_dead_zone_adjusted[0] = apply_dead_zone(gyro_degrees[0], 180.0, -180.0, target_dead_zone_percent);
            gyro_degrees_dead_zone_adjusted[1] = apply_dead_zone(gyro_degrees[1], 180.0, -180.0, target_dead_zone_percent);
            gyro_degrees_dead_zone_adjusted[2] = apply_dead_zone(gyro_degrees[2], 180.0, -180.0, target_dead_zone_percent);


            // pitch_pid2.myInput = gyro_degrees_dead_zone_adjusted[0];
            // PID_Compute(&pitch_pid2);
            // printf("  %6.4f == %6.4f ",error_pitch, pitch_pid2.myOutput);
            // double error_pitch2 = map_value(pitch_pid2.myOutput , -45.0, 45.0, -100.0, 100.0);

            error_pitch = pid_get_error(&pitch_pid, gyro_degrees_dead_zone_adjusted[0], esp_timer_get_time());
            error_pitch = map_value(error_pitch, -45.0, 45.0, -100.0, 100.0);

            // error_yaw = map_value(pid_get_error(&yaw_pid, gyro_degrees[2], esp_timer_get_time()), -180.0, 180.0, -100.0, 100.0);
            // error_pitch = apply_dead_zone(error_pitch, 100.0, -100.0, target_dead_zone_percent);
            // error_speed_x = pid_get_error(&wheel_speed_x_pid, wheels_speed[1], esp_timer_get_time());

            // Try adding error of acceleration here
            float motor_a_control = (-error_pitch) + error_yaw - error_speed_x; // Decrease speed at the cost of pitch 
            float motor_b_control = (-error_pitch) - error_yaw - error_speed_x;

            //printf("(5) ");
            //track_time();

            change_speed_motor_A(motor_a_control, 31.5 - 2); // 31.5 - 2
            change_speed_motor_B(motor_b_control, 26.9 - 2); // 26.9 - 2

            optical_encoder_set_clockwise(optical_encoder_1_result , motor_a_control > 0);
            optical_encoder_set_clockwise(optical_encoder_2_result , motor_b_control > 0);

            // printf("(6) ");
            // track_time();

        }else{
            change_speed_motor_A(0, 0);
            change_speed_motor_B(0, 0);
        }

        // printf("(7) ");
        // track_time();

        // Monitoring ###########################################################################################################################################################

        // printf("SPEED, %9.5f, %9.5f, %9.5f, ", speed[0], error_speed_x, 100.0);
        // printf(" Loop Scanner 1 - %d   %9.5f  ", 
        //     optical_encoder_get_count(optical_encoder_2_result),
        //     optical_encoder_get_hertz(optical_encoder_2_result, 2.55, -2.55)
        // );
        // printf("ACCEL, %6.2f, %6.2f, %6.2f, ", acceleration_data[0], acceleration_data[1], acceleration_data[2]);
        // printf("GYRO, %6.2f, %6.2f, %6.2f, ", gyro_degrees[0], gyro_degrees[1], gyro_degrees[2]);
        // printf("ERROR, %6.2f, %6.2f, ", error_pitch, error_speed_x);

        // printf("TARGET, %6.2f, ", pitch_target);
        // printf("MAG, %6.2f, %6.2f, %6.2f, ", magnetometer_data[0], magnetometer_data[1], magnetometer_data[2]);
        // printf("NORTH, %6.2f, %6.2f, %6.2f, ", north_direction[0], north_direction[1], north_direction[2]);
        // printf("SPEED 1, %6.2f, %6.2f, %6.2f, ", wheels_speed[0], wheels_speed[1], 0.0);
        

        handle_loop_timing();
        printf("\n"); 
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

void check_calibrations(){
    // Checking errors of mpu6050
    find_accelerometer_error(1000);
    find_gyro_error(300);

    // Check the fuckedupedness of motors and at what percentage they start to spin.
    float speed = 25;
    // while (speed != 100)
    // {
    //     printf("Speed %9.5f\n", speed);
    //     change_speed_motor_A(speed, 0); // 34.9 the one with the broken pegs 31.5
    //     speed += 0.1;
    //     vTaskDelay(250 / portTICK_RATE_MS);
    //     change_speed_motor_A(0, 0); // sometimes the motor doesn't have enough leverage between steps of voltage
    //     vTaskDelay(250 / portTICK_RATE_MS);
    // }
    change_speed_motor_A(0, 0);
    speed = 25;
    while (speed != 100)
    {
        printf("Speed %9.5f\n", speed);
        change_speed_motor_B(speed, 0); // 26.9
        speed += 0.1;
        vTaskDelay(250 / portTICK_RATE_MS);
        change_speed_motor_B(0, 0); // sometimes the motor doesn't have enough leverage between steps of voltage
        vTaskDelay(250 / portTICK_RATE_MS);
    }
    change_speed_motor_B(0, 0);
}

void init_sensors(){
    // Init sensors
    printf("-----------------------------INITIALIZING MODULES...\n");

    bool mpu6050 = init_mpu6050(GPIO_NUM_22, GPIO_NUM_21, true, true, accelerometer_correction, gyro_correction, complementary_ratio);
    bool gy271 = init_gy271(GPIO_NUM_22, GPIO_NUM_21, false, true, hard_iron_correction, soft_iron_correction);
    init_TB6612(GPIO_NUM_33, GPIO_NUM_32, GPIO_NUM_25, GPIO_NUM_27, GPIO_NUM_26, GPIO_NUM_14);
    bool nrf24 = nrf24_init(SPI3_HOST, GPIO_NUM_17, GPIO_NUM_16);
    optical_encoder_1_result = init_optical_encoder(GPIO_NUM_4, true, 0.2356, 20, 0.2);
    optical_encoder_2_result = init_optical_encoder(GPIO_NUM_15, false, 0.2356, 20, 0.2);

    printf("-----------------------------INITIALIZING MODULES DONE... ");
    if (mpu6050 && gy271 && nrf24 && optical_encoder_1_result >= 0 && optical_encoder_2_result >= 0){
        printf("OK\n");
    }else{
        printf("NOT OK %d %d %d %d %d\n", mpu6050, gy271, nrf24, optical_encoder_1_result, optical_encoder_2_result);
    }

    nrf24_rx_mode(tx_address, 10);
}

void init_esp32_peripherals(){
    init_spi3();
}

void init_loop_timer(){
    // Init loop timer
    loop_start_time = esp_timer_get_time()/1000;
    vTaskDelay(1000 / portTICK_RATE_MS);
}

void get_initial_position(){
    // Find the initial position in degrees and apply it to the gyro measurement integral
    // This will tell the robot which way to go to get the actual upward
    mpu6050_get_accelerometer_readings_gravity(acceleration_data);
    mpu6050_get_gyro_readings_dps(gyro_angular);
    gy271_magnetometer_readings_micro_teslas(magnetometer_data);
    fix_mag_axis(magnetometer_data); // Switches around the x and the y to match mpu6050

    calculate_degrees_x_y(acceleration_data, &accelerometer_x_rotation, &accelerometer_y_rotation);
    calculate_yaw(magnetometer_data, &accelerometer_z_rotation);

    gyro_degrees[0] = accelerometer_x_rotation;
    gyro_degrees[1] = accelerometer_y_rotation;
    gyro_degrees[2] = accelerometer_z_rotation;

    printf("Initial location x: %.2f y: %.2f, z: %.2f\n", gyro_degrees[0], gyro_degrees[1], gyro_degrees[2]);
}


void extract_joystick_request_values(char *request, uint8_t request_size, uint8_t *throttle, uint8_t *yaw, uint8_t *pitch, uint8_t *roll)
{
    // Skip the request type
    char* start = strchr(request, '/') + 1;
    char* end = strchr(start, '/');

    start = strchr(end, '/') + 1;
    end = strchr(start, '/');
    if(start == NULL || end == NULL ) return;
    int length = end - start;
    char throttle_string[length + 1];
    strncpy(throttle_string, start, length);
    throttle_string[length] = '\0';
    *throttle = atoi(throttle_string);
    //printf("'%s'\n", throttle_string);

    start = strchr(end, '/') + 1;
    end = strchr(start, '/');
    if(start == NULL || end == NULL ) return;
    length = end - start;
    char yaw_string[length + 1];
    strncpy(yaw_string, start, length);
    yaw_string[length] = '\0';
    *yaw = atoi(yaw_string);
    //printf("'%s'\n", yaw_string);

    start = strchr(end, '/') + 1;
    end = strchr(start, '/');
    if(start == NULL || end == NULL ) return;
    length = end - start;
    char pitch_string[length + 1];
    strncpy(pitch_string, start, length);
    pitch_string[length] = '\0';
    *pitch = atoi(pitch_string);
    //printf("'%s'\n", pitch_string);

    start = strchr(end, '/') + 1;
    end = strchr(start, '/');
    if(start == NULL || end == NULL ) return;
    length = end - start;
    char roll_string[length + 1];
    strncpy(roll_string, start, length);
    roll_string[length] = '\0';
    *roll = atoi(roll_string);
    //printf("'%s'\n", roll_string);
}

void extract_request_type(char *request, uint8_t request_size, char *type_output){
    char* start = strchr(request, '/') + 1;
    char* end = strchr(start, '/');
    if(start == NULL || end == NULL ) return;
    int length = end - start;
    //char type_string[length + 1];
    // You better be sure the length of the output string is big enough
    strncpy(type_output, start, length);
    type_output[length] = '\0';
    //printf("'%s'\n", type_output);

}

void extract_pid_request_values(char *request, uint8_t request_size, double *added_proportional, double *added_integral, double *added_derivative, double *added_master){
    // Skip the request type
    char* start = strchr(request, '/') + 1;
    char* end = strchr(start, '/');

    start = strchr(end, '/') + 1;
    end = strchr(start, '/');
    if(start == NULL || end == NULL ) return;
    int length = end - start;
    char added_proportional_string[length + 1];
    strncpy(added_proportional_string, start, length);
    added_proportional_string[length] = '\0';
    *added_proportional = strtod(added_proportional_string, NULL);
    //printf("'%s'\n", added_proportional);

    start = strchr(end, '/') + 1;
    end = strchr(start, '/');
    if(start == NULL || end == NULL ) return;
    length = end - start;
    char added_integral_string[length + 1];
    strncpy(added_integral_string, start, length);
    added_integral_string[length] = '\0';
    *added_integral = strtod(added_integral_string, NULL);
    //printf("'%s'\n", added_integral);

    start = strchr(end, '/') + 1;
    end = strchr(start, '/');
    if(start == NULL || end == NULL ) return;
    length = end - start;
    char added_derivative_string[length + 1];
    strncpy(added_derivative_string, start, length);
    added_derivative_string[length] = '\0';
    *added_derivative = strtod(added_derivative_string, NULL);
    //printf("'%s'\n", added_derivative);

    start = strchr(end, '/') + 1;
    end = strchr(start, '/');
    if(start == NULL || end == NULL ) return;
    length = end - start;
    char added_master_string[length + 1];
    strncpy(added_master_string, start, length);
    added_master_string[length] = '\0';
    *added_master = strtod(added_master_string, NULL);
    //printf("'%s'\n", added_master);
}

void handle_loop_timing(){
    loop_end_time = esp_timer_get_time()/1000;
    delta_loop_time = loop_end_time - loop_start_time;
    printf("Tim: %d ms", delta_loop_time);
    if (delta_loop_time < (1000 / refresh_rate_hz))
    {
        vTaskDelay(((1000 / refresh_rate_hz) - delta_loop_time) / portTICK_RATE_MS);
    }
    loop_start_time = esp_timer_get_time()/1000;
}

void track_time(){
    uint32_t loop_end_time_temp = esp_timer_get_time()/1000;
    uint32_t delta_loop_time_temp = loop_end_time - loop_start_time;

    printf("%5d ms ", delta_loop_time_temp);
}

double map_value(double value, double input_min, double input_max, double output_min, double output_max) {
    // Calculate the input and output ranges' lengths
    double input_range = input_max - input_min;
    double output_range = output_max - output_min;

    // Normalize the input value relative to the input range
    double normalized_value = (value - input_min) / input_range;

    // Scale the normalized value according to the output range
    double scaled_value = normalized_value * output_range;

    // Add the output range's minimum value to the scaled value
    double output_value = output_min + scaled_value;

    return output_value;
}

double apply_dead_zone(double value, double max_value, double min_value, double dead_zone){
    double mid_point = (max_value + min_value) / 2;
    double half_range = (max_value - min_value) / 2;
    double normalized_value = (value - mid_point) / half_range; // this will be -1 at min_value, +1 at max_value

    double dead_zone_normalized = dead_zone / half_range;

    double return_value;

    // remove the deadzone
    if (normalized_value > dead_zone_normalized) {
        return_value = (normalized_value - dead_zone_normalized) / (1.0 - dead_zone_normalized);
    } else if (normalized_value < -dead_zone_normalized) {
        return_value = (normalized_value + dead_zone_normalized) / (1.0 - dead_zone_normalized);
    } else {
        return_value = 0.0;
    }

    return return_value * half_range + mid_point; // scale back to original range
}

void calculate_speeds_no_yaw(float* acceleration, float* gyro_degrees, float* speed, float refresh_rate){
    // Converting to radians
    float pitch_radian = gyro_degrees[1] * M_PI / 180.0;
    float roll_radian = gyro_degrees[0] * M_PI / 180.0;

    // Convert acceleration to m/s^2
    double acceleration_meters[] = {acceleration[0] * 9.81, acceleration[1] * 9.81, acceleration[2] * 9.81};

    // Calculate the projected acceleration along each axis
    double speed_x_big = acceleration_meters[0] + (9.81 * sin(pitch_radian));
    double speed_y_big = acceleration_meters[1] + (9.81 * sin(roll_radian));
    double speed_z_big = acceleration_meters[2] + (9.81 * cos(pitch_radian) * cos(roll_radian));

    // Calculating speed by integrating actual acceleration over time
    speed[0] += speed_x_big / refresh_rate;
    speed[1] += speed_y_big / refresh_rate;
    speed[2] += speed_z_big / refresh_rate;
}

void calculate_speeds(float* acceleration, float* gyro_degrees, float* speed, float refresh_rate){
    // Convert to radians
    float roll_radian = gyro_degrees[0] * M_PI / 180.0;
    float pitch_radian = gyro_degrees[1] * M_PI / 180.0;
    float yaw_radian = gyro_degrees[2] * M_PI / 180.0;

    // Convert acceleration to m/s^2 and adjust for gravity
    double acceleration_meters[] = {acceleration[0] * 9.81, acceleration[1] * 9.81, acceleration[2] * 9.81};

    double rotation_matrix[3][3];

    rotation_matrix[0][0] = cos(yaw_radian)*cos(pitch_radian); 
    rotation_matrix[0][1] = cos(yaw_radian)*sin(pitch_radian)*sin(roll_radian) - sin(yaw_radian)*cos(roll_radian);
    rotation_matrix[0][2] = cos(yaw_radian)*sin(pitch_radian)*cos(roll_radian) + sin(yaw_radian)*sin(roll_radian);

    rotation_matrix[1][0] = sin(yaw_radian)*cos(pitch_radian); 
    rotation_matrix[1][1] = sin(yaw_radian)*sin(pitch_radian)*sin(roll_radian) + cos(yaw_radian)*cos(roll_radian);
    rotation_matrix[1][2] = sin(yaw_radian)*sin(pitch_radian)*cos(roll_radian) - cos(yaw_radian)*sin(roll_radian);

    rotation_matrix[2][0] = -sin(pitch_radian); 
    rotation_matrix[2][1] = cos(pitch_radian)*sin(roll_radian); 
    rotation_matrix[2][2] = cos(pitch_radian)*cos(roll_radian); 

    double speed_x_big = rotation_matrix[0][0] * acceleration_meters[0] + rotation_matrix[0][1] * acceleration_meters[1] + rotation_matrix[0][2] * acceleration_meters[2]; 
    double speed_y_big = rotation_matrix[1][3] * acceleration_meters[0] + rotation_matrix[1][4] * acceleration_meters[1] + rotation_matrix[1][5] * acceleration_meters[2]; 
    double speed_z_big = rotation_matrix[2][6] * acceleration_meters[0] + rotation_matrix[2][7] * acceleration_meters[1] + rotation_matrix[2][8] * acceleration_meters[2]; 

    speed[0] += speed_x_big / refresh_rate;
    speed[1] += speed_y_big / refresh_rate;
    speed[2] += speed_z_big / refresh_rate;
}


void send_pid_base_info_to_remote(){
    nrf24_tx_mode(tx_address, 10);
    vTaskDelay(250 / portTICK_RATE_MS);

    char *string = generate_message_pid_values_nrf24(
        base_pitch_gain_p, 
        base_pitch_gain_i, 
        base_pitch_gain_d,
        base_pitch_master_gain
    );
    
    for (size_t i = 0; i < 100; i++)
    {
        if(!nrf24_transmit(string)){
            printf("FAILED\n");
        }
    }
    free(string);
    
    nrf24_rx_mode(tx_address, 10);
}

void send_pid_added_info_to_remote(){
    nrf24_tx_mode(tx_address, 10);
    vTaskDelay(250 / portTICK_RATE_MS);

    char *string = generate_message_pid_values_nrf24(
        added_pitch_gain_p, 
        added_pitch_gain_i, 
        added_pitch_gain_d,
        added_pitch_master_gain
    );
    
    for (size_t i = 0; i < 200; i++)
    {
        if(!nrf24_transmit(string)){
            printf("FAILED\n");
        }
    }
    free(string);
    
    nrf24_rx_mode(tx_address, 10);
}

char* generate_message_pid_values_nrf24(double base_proportional, double base_integral, double base_derivative, double base_master){
    // calculate the length of the resulting string
    int length = snprintf(
        NULL, 
        0, 
        "/s/%.2f/%.2f/%.2f/%.2f/  ", 
        base_proportional, 
        base_integral, 
        base_derivative, 
        base_master
    );
    
    // allocate memory for the string
    char *string = malloc(length + 1); // +1 for the null terminator

    // format the string
    snprintf(
        (char*)string, 
        length + 1, 
        "/s/%.2f/%.2f/%.2f/%.2f/  ", 
        base_proportional, 
        base_integral, 
        base_derivative, 
        base_master
    );

    return string;
}