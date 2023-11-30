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
#include "../lib/mpu6050/mpu6050.h"
#include "../lib/TB6612/TB6612.h"
#include "../lib/gy271_qmc5883l/gy271.h"
#include "../lib/nrf24l01/nrf24l01.h"
#include "../lib/optical_encoder/optical_encoder.h"

// Other imports
#include "../lib/util/ned_coordinates/ned_coordinates.h"
#include "../lib/util/spi/spi.h"
#include "../lib/pid/pid.h"

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

// ESP32 DevkitC v4 - ESP-WROOM-32D - 160 Mhz

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
 * Encoder 1 Wheel B
 * 
 * GPIO4
 * 
 */

/**
 * Encoder 2 Wheel A
 * 
 * GPIO15
 * 
 */


// Sensor corrections #################################################################################

// For calibrating the magnetometer I
// used a method found in MicWro Engr Video:
// https://www.youtube.com/watch?v=MinV5V1ioWg
// Mag field norm at my location is 50.503
// use software called Magneto 1.2


// https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm
// Convert readings from this site to same units nanoTeslas to microteslas

// When calculating this remember that these values are already
// correcting and that the axis of the magnetometer are switched.
// float hard_iron_correction[3] = {
//     0, 0, 0
// };

// float soft_iron_correction[3][3] = {
//     {1,0,0},
//     {0,1,0},
//     {0,0,1}
// };

float hard_iron_correction[3] = {
    416.657934, 146.816267, -177.441049
};

float soft_iron_correction[3][3] = {
    {0.177997,-0.000304,-0.003343},
    {-0.000304,0.176037,-0.005926},
    {-0.003343,-0.005926,0.216928}
};

float accelerometer_correction[3] = {
    -0.031383, -0.007, 1.034993
};
float gyro_correction[3] = {
    -6.133409, 1.828601, -0.318321
};

// handling loop timing ###################################################################################
uint32_t loop_start_time = 0;
uint32_t loop_end_time = 0;
int16_t delta_loop_time = 0;

// Accelerometer values to degrees conversion #############################################################
float accelerometer_x_rotation = 0;
float accelerometer_y_rotation = 0;
float magnetometer_z_rotation = 0;

// Radio config ########################################################################################### 
uint8_t tx_address[5] = {0xEE, 0xDD, 0xCC, 0xBB, 0xAA};
char rx_data[32];
char rx_type[32];

// PID errors ##############################################################################################
double error_pitch = 0;
double error_yaw = 0;
double error_speed_A = 0;
double error_speed_B = 0;
double error_position_A = 0;
double error_position_B = 0;

// Actual PID adjustment for pitch
const double base_pitch_master_gain = 1.0;
const double base_pitch_gain_p = 15.0;
const double base_pitch_gain_i = 350.0;
const double base_pitch_gain_d = 0.31;

// PID for speed
const float speed_gain_p = 0.0; 
const float speed_gain_i = 0.0;
const float speed_gain_d = 0.0;

// PID for postion of robot
const float position_gain_p = 0.0; 
const float position_gain_i = 0.0;
const float position_gain_d = 0.0;

// PID for yaw
const float yaw_gain_p = 0.6; 
const float yaw_gain_i = 0.0;
const float yaw_gain_d = 0.0;

// PID for RPM
const float rpm_gain_p = 0.0000; 
const float rpm_gain_i = 0.015;
const float rpm_gain_d = 0.0000;

// Used for smooth changes to PID while using remote control. Do not touch this
double pitch_master_gain = base_pitch_master_gain;
double pitch_gain_p = base_pitch_gain_p;
double pitch_gain_i = base_pitch_gain_i;
double pitch_gain_d = base_pitch_gain_d;
double added_pitch_master_gain = 0;
double added_pitch_gain_p = 0;
double added_pitch_gain_i = 0;
double added_pitch_gain_d = 0;

// Refresh rate ##############################################################################################
const float refresh_rate_hz = 400;

// Sensor stuff ##############################################################################################
const float complementary_ratio = 1.0 - 1.0/(1.0+(1.0/refresh_rate_hz)); // Depends on how often the loop runs. 1 second / (1 second + one loop time)
float acceleration_data[] = {0,0,0};
float gyro_angular[] = {0,0,0};
float gyro_degrees[] = {0,0,0};
float magnetometer_data[] = {0,0,0};
float acceleration_speed[] = {0,0,0};
int8_t optical_encoder_1_result = -1;
int8_t optical_encoder_2_result = -1;
double wheel_speed[] = {0,0};
double wheel_rpm[] = {0,0};
double wheel_position[] = {0,0};

double target_pitch = 0.0;
double target_yaw = 0.0;

// Remote control settings ############################################################################################
double max_yaw_attack = 40.0;
double max_pitch_attack = 1.5;
double pitch_attack_step = 0.1;
double max_rpm_on_pitch = 35;

int8_t last_robot_direction = 0; // -1 negative degrees, 1 positive degrees, 0 nothing
uint8_t throttle = 0;
uint8_t yaw = 50;
uint8_t last_yaw = 50;
uint8_t pitch = 50;
uint8_t roll = 50;
bool shit_encoder_mode = true;
bool slowing_lock = false;
double last_raw_yaw = 0;
double delta_yaw = 0;

void app_main() {
    printf("STARTING PROGRAM\n"); 
    init_esp32_peripherals();
    init_sensors();
    init_loop_timer();
    // check_calibrations();
    get_initial_position();

    struct pid pitch_pid = pid_init(pitch_master_gain * pitch_gain_p, pitch_master_gain * pitch_gain_i, pitch_master_gain * pitch_gain_d, 0.0, esp_timer_get_time(), 100.0, -100.0, 1);
    struct pid yaw_pid = pid_init(yaw_gain_p, yaw_gain_i, yaw_gain_d, 0.0, esp_timer_get_time(), 0, 0, 0);
    // struct pid wheel_speed_A_pid = pid_init(speed_gain_p, speed_gain_i, speed_gain_d, 0.0, esp_timer_get_time(), 0, 0, 0);
    // struct pid wheel_speed_B_pid = pid_init(speed_gain_p, speed_gain_i, speed_gain_d, 0.0, esp_timer_get_time(), 0, 0, 0);
    // struct pid wheel_position_A_pid = pid_init(position_gain_p, position_gain_i, position_gain_d, 0.0, esp_timer_get_time(), 0, 0, 0);
    // struct pid wheel_position_B_pid = pid_init(position_gain_p, position_gain_i, position_gain_d, 0.0, esp_timer_get_time(), 0, 0, 0);
    struct pid wheel_rpm_pid = pid_init(rpm_gain_p, rpm_gain_i, rpm_gain_d, max_rpm_on_pitch, esp_timer_get_time(), 0, 0, 0);

    printf("\n\n====START OF LOOP====\n\n");
    while (true){
        // Read sensor data ######################################################################################################################
        mpu6050_get_accelerometer_readings_gravity(acceleration_data);
        mpu6050_get_gyro_readings_dps(gyro_angular);
        gy271_magnetometer_readings_micro_teslas(magnetometer_data);
        // wheel_speed[0] = optical_encoder_get_hertz(optical_encoder_2_result, 2.55, -2.55);
        // wheel_speed[1] = optical_encoder_get_hertz(optical_encoder_1_result, 2.55, -2.55);
        // wheel_position[0] = optical_encoder_get_count(optical_encoder_2_result);
        // wheel_position[1] = optical_encoder_get_count(optical_encoder_1_result);
        wheel_rpm[0] = optical_encoder_get_rpm(optical_encoder_2_result, 200, -200);
        wheel_rpm[1] = optical_encoder_get_rpm(optical_encoder_1_result, 200, -200);

        // Convert the sensor data to data that is useful
        fix_mag_axis(magnetometer_data); // Switches around the x and the y of the magnetometer to match mpu6050 outputs
        calculate_degrees_x_y(acceleration_data, &accelerometer_x_rotation, &accelerometer_y_rotation); // Get roll and pitch from the data. I call it x and y. Ranges -90 to 90. 

        // My mpu6050 has a drift problem when rotating around z axis. I have only seen Joop Broking having this problem.
        // Raw yaw - yaw without tilt adjustment
        // Yaw - yaw with tilt adjustment
        // Get raw yaw. The results of this adjustment have to modify the values of gyro degrees before complementary filter. So only the non tilt adjusted yaw is available
        calculate_yaw(magnetometer_data, &magnetometer_z_rotation);

        if(yaw != 50){ // This is not an issue when not rotating.
            // Joop Brokings method did not work for me and it didn't make sense subtracting scaled total yaw. I subtract delta yaw instead
            delta_yaw = angle_difference(magnetometer_z_rotation, last_raw_yaw); // Helper function to handle wrapping

            // gyro_angular[0] -= delta_yaw * -16.5; 
            gyro_angular[0] -= delta_yaw * -5.0; 

            // This robot doesn't use the y axis so i ignore it 
        }

        // Save the raw yaw for next loop. No mater if yaw changes or not. Need to know the latest one
        last_raw_yaw = magnetometer_z_rotation;

        // Use complementary filter to correct the gyro drift. 
        convert_angular_rotation_to_degrees_x_y(gyro_angular, gyro_degrees, accelerometer_x_rotation, accelerometer_y_rotation, esp_timer_get_time(), true);

        // Get yaw that is adjusted by x and y degrees
        calculate_yaw_tilt_compensated(magnetometer_data, &magnetometer_z_rotation, gyro_degrees[0], gyro_degrees[1]);

        // Complementary filter did not work good for magnetometer+gyro. 
        // Gyro just too slow and inaccurate for this.
        // I trust the magnetometer more than the gyro in this case.
        gyro_degrees[2] = magnetometer_z_rotation;

        // Receive remote control data ###########################################################################################################
        if(nrf24_data_available(1)){
            
            nrf24_receive(rx_data);
            
            // Get the type of request
            extract_request_type(rx_data, strlen(rx_data), rx_type);

            if(strcmp(rx_type, "joystick") == 0){


                extract_joystick_request_values(rx_data, strlen(rx_data), &throttle, &yaw, &roll, &pitch);

                // Check if pitch is neutral
                if(pitch == 50){

                    // Had some issues with floats not being zero.
                    // If value is between pitch attack and negative pitch attack then just set it to zero
                    if(target_pitch < pitch_attack_step/2 && target_pitch > -pitch_attack_step/2){
                        target_pitch = 0.0;
                    }

                    // If value is above pitch attack values 
                    // Start slowing going back to zero
                    if(target_pitch > 0 && target_pitch != 0){
                        target_pitch = target_pitch - pitch_attack_step;
                    }else if(target_pitch < 0  && target_pitch != 0){
                        target_pitch = target_pitch + pitch_attack_step;
                    }
                }else if (pitch != 50){
                    // If pitch is not neutral start increasing into some direction.

                    if(shit_encoder_mode){
                        // Because my encoder is shit it prefers to have the extremes of pitch as 
                        // that doesn't overwhelm the pid of it.
                        if(pitch > 60){
                            target_pitch = target_pitch + pitch_attack_step;
                        }else if(pitch < 40){
                            target_pitch = target_pitch - pitch_attack_step;
                        }
                    }else{
                        target_pitch = target_pitch + map_value(pitch, 0.0, 100.0, -pitch_attack_step, pitch_attack_step);
                    }
                }

                // Make sure that the value is in the boundaries
                if(target_pitch > max_pitch_attack){
                    target_pitch = max_pitch_attack;
                }else if(target_pitch < -max_pitch_attack){
                    target_pitch = -max_pitch_attack;
                }

                // Handle the yaw value
                if(yaw != 50){
                    // There is an issue with the remote sometimes sending a 0 yaw

                    target_yaw = gyro_degrees[2] + map_value(yaw, 0.0, 100.0, -max_yaw_attack, max_yaw_attack);
                    
                    // handle the switch from -180 to 180 degrees
                    if(target_yaw > 180.0){
                        target_yaw = target_yaw - 360.0;
                    }else if(target_yaw < -180.0){
                        target_yaw = target_yaw + 360.0;
                    }
                }

                // Reset the yaw to the current degrees
                if(last_yaw != 50 && yaw == 50){
                    target_yaw = gyro_degrees[2];
                }

                last_yaw = yaw;
            }else if(strcmp(rx_type, "pid") == 0){
                double added_proportional = 0;
                double added_integral = 0;
                double added_derivative = 0;
                double added_master_gain = 0;

                extract_pid_request_values(rx_data, strlen(rx_data), &added_proportional, &added_integral, &added_derivative, &added_master_gain);

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
            }else if(strcmp(rx_type, "remoteSyncBase") == 0){
                send_pid_base_info_to_remote();
            }else if(strcmp(rx_type, "remoteSyncAdded") == 0){
                send_pid_added_info_to_remote();
            }

            rx_type[0] = '\0'; // Clear out the string by setting its first char to string terminator
        }

        // React to the data received from sensors and remote control ###########################################################################################################
        if(gyro_degrees[0] < 20 && gyro_degrees[0] > -20){

            // Adjust the pitch if robot going too fast. Use the target pitch to know robots direction
            double new_target_pitch = target_pitch;

            // So the problem i have with this is that my sensor cannot tell the direction of the motion
            // because it is not a quadrature sensor. I do not have the precision to attach a second sensor
            // in such a way that it is slightly lagging behind every activation of the first sensor. The only
            // way i can achieve this is to upgrade to a magnetic encoder. I may do that later in the project

            // What i can do now though is use my diy optical encoder and infer the direction of the robot in
            // some other way. If my robot is told to move forwards i can assume that the direction of the rpm
            // is forwards also. If no move input is made i cannot assume anything.

            // What i do is if the speed goes above what i like i start slowing it down with only pid with integral.
            // I use integral because my encoder disk with holes is also small resolution, the integral can accumulate 
            // even if the value doesn't change. This happens when the robot is told to move forward by remote.

            // When remote stops saying move forward i use the previous information that it moved forward at some speed
            // to make it slow down back to zero speed. This is what limits the pid the most. If i put too much gains on pid
            // it will overshoot the 0 speed point start gaining speed again. At that point the slowing functionality only 
            // amplifies the speed. So the integral is a cautious 0.015 and everything else is zero. 

            // I also adjust the desired pitch and not add this pid to the error that is applied to the motors. This is because
            // the pitch change can impact things in a more significant and predictable way. I dont need to have a new set of pid 
            // settings for the pitch pid

            if(max_rpm_on_pitch < wheel_rpm[0] && !slowing_lock){
                // Check which direction to adjust to
                if( new_target_pitch < 0){
                    pid_set_desired_value(&wheel_rpm_pid, max_rpm_on_pitch);
                    // more than zero so adjust back
                    new_target_pitch = target_pitch - pid_get_error(&wheel_rpm_pid, wheel_rpm[0], esp_timer_get_time());

                    // remember the heading
                    last_robot_direction = 1;
                }else if(new_target_pitch > 0){
                    pid_set_desired_value(&wheel_rpm_pid, max_rpm_on_pitch);
                    // less than zero so adjust forward
                    new_target_pitch = target_pitch + pid_get_error(&wheel_rpm_pid, wheel_rpm[0], esp_timer_get_time());
                    last_robot_direction = -1;
                }
            }else if(new_target_pitch != 0){
                // When it is not going over the speed limit reset the integral so it doesn't stay wound up
                pid_reset_integral_sum(&wheel_rpm_pid);
            }

            // Slow down the robot when the desired pitch goes to 0 and there was adjusting action before.
            if(target_pitch == 0.0 && last_robot_direction != 0 && wheel_rpm[0] != 0){
                slowing_lock = true;
                pid_set_desired_value(&wheel_rpm_pid, 0);
                if(last_robot_direction == 1){
                     new_target_pitch = target_pitch + pid_get_error(&wheel_rpm_pid, wheel_rpm[0], esp_timer_get_time());
                }else if(last_robot_direction == -1){
                    new_target_pitch = target_pitch -pid_get_error(&wheel_rpm_pid, wheel_rpm[0], esp_timer_get_time());
                }
            }else if(target_pitch == 0.0 && wheel_rpm[0] == 0){
                last_robot_direction = 0;
                slowing_lock = false;
                pid_reset_integral_sum(&wheel_rpm_pid);
            }

            // Update pid targets by transforming remote input
            pid_set_desired_value(&pitch_pid, new_target_pitch);
            pid_set_desired_value(&yaw_pid, target_yaw);

            error_pitch = pid_get_error(&pitch_pid, gyro_degrees[0], esp_timer_get_time());
            // error_position_A = pid_get_error(&wheel_position_A_pid, wheel_position[0], esp_timer_get_time());
            // error_position_B = pid_get_error(&wheel_position_B_pid, wheel_position[1], esp_timer_get_time());
            // Special case, dont want the pid to go crazy when robot goes from 179 to -179. Calculate own error
            error_yaw = pid_get_error_own_error(&yaw_pid, -angle_difference(target_yaw, gyro_degrees[2]), esp_timer_get_time()); 

            float motor_A_control = (-error_pitch) - error_yaw + error_speed_A + error_position_A; 
            float motor_B_control = (-error_pitch) + error_yaw + error_speed_B + error_position_B;

            change_speed_motor_A(motor_A_control, 0);
            change_speed_motor_B(motor_B_control, 0);
        }else{
            change_speed_motor_A(0, 0);
            change_speed_motor_B(0, 0);
        }

        // Monitoring ###########################################################################################################################################################

        // printf("ACCEL, %6.2f, %6.2f, %6.2f, ", acceleration_data[0], acceleration_data[1], acceleration_data[2]);
        // printf("GYRO, %f, %f, %f, ", gyro_degrees[0], gyro_degrees[1], gyro_degrees[2]);
        // printf("MAG, %f, %f, %f, ", magnetometer_data[0], magnetometer_data[1], magnetometer_data[2]);
        // printf("RPM, %6.2f, %6.2f, ", wheel_rpm[0], wheel_rpm[1]);
        // printf("TARGET, %6.2f, %6.2f, ", target_pitch, target_yaw);
        // printf("\n"); // End the line after all the data has been printed. So that all data is on one line

        handle_loop_timing(); // Make sure the loop has exact timing as specified in refresh rate
    }
}

// Switch magnetometer axis. x-> y and y->x
void fix_mag_axis(float* magnetometer_data){
    float temp = 0.0;
    temp = magnetometer_data[0];
    // y is x
    magnetometer_data[0] = magnetometer_data[1];
    // x is -y
    magnetometer_data[1] = -temp;
}

// Find the errors for the accelerometer and gyro
void check_calibrations(){
    // Checking errors of mpu6050
    find_accelerometer_error(1000);
    find_gyro_error(300);

    
    // I dont use the code bellow anymore but maybe you will

    // Testing with different motor drivers
    // TB6612 and power bank - A 31.5  B 26.9
    // TB6612 and 3 x 18650 cells - A 15.5  B 12.5  
    // L298 and 2 x 18650 cells - A 69.8   B 70  // Worked very good with above settings though, very smooth
    // L298 and 3 x 18650 cells - A 52.5   B 52.5  

    uint8_t movements_in_a_row = 5;
    uint8_t move_counter = 0;

    // Check the fuckedupedness of motors and at what percentage they start to spin.
    float speed = 0;
    while (speed != 100)
    {
        printf("Speed %9.5f\n", speed);
        change_speed_motor_A(speed, 0); // 34.9 the one with the broken pegs 31.5
        speed += 0.5;
        vTaskDelay(250 / portTICK_RATE_MS);
        change_speed_motor_A(0, 0); // sometimes the motor doesn't have enough leverage between steps of voltage
        vTaskDelay(250 / portTICK_RATE_MS);

        // Use optical encoders to determine if it moved
        if(optical_encoder_get_count(optical_encoder_2_result) != 0 ){
            printf("Spin \n");
            optical_encoder_set_count(optical_encoder_2_result, 0);
            move_counter++;

            if(move_counter == movements_in_a_row){
                printf("Motor A start speed - %f \n", speed - 0.5);
                break;
            }
        }else{
            move_counter = 0;
        }
    }
    change_speed_motor_A(0, 0);
    speed = 0;
    while (speed != 100)
    {
        printf("Speed %9.5f\n", speed);
        change_speed_motor_B(speed, 0); // 26.9
        speed += 0.5;
        vTaskDelay(250 / portTICK_RATE_MS);
        change_speed_motor_B(0, 0); // sometimes the motor doesn't have enough leverage between steps of voltage
        vTaskDelay(250 / portTICK_RATE_MS);

        // Use optical encoders to determine if it moved
        if(optical_encoder_get_count(optical_encoder_1_result) != 0 ){
            printf("Spin \n");
            optical_encoder_set_count(optical_encoder_1_result, 0);
            move_counter++;

            if(move_counter == movements_in_a_row){
                printf("Motor B start speed - %f \n", speed - 0.5);
                break;
            }
        }else{
            move_counter = 0;
        }
    }
    change_speed_motor_B(0, 0);
}

// Abstract away initialization of sensor drivers
void init_sensors(){
    // Init sensors
    printf("-----------------------------INITIALIZING MODULES...\n");

    bool mpu6050 = init_mpu6050(GPIO_NUM_22, GPIO_NUM_21, true, true, accelerometer_correction, gyro_correction, complementary_ratio);
    bool gy271 = init_gy271(GPIO_NUM_22, GPIO_NUM_21, false, true, hard_iron_correction, soft_iron_correction);
    init_TB6612(GPIO_NUM_33, GPIO_NUM_32, GPIO_NUM_25, GPIO_NUM_27, GPIO_NUM_26, GPIO_NUM_14);
    bool nrf24 = nrf24_init(SPI3_HOST, GPIO_NUM_17, GPIO_NUM_16);
    optical_encoder_1_result = init_optical_encoder(GPIO_NUM_4, true, 0.2356, 30, 0.2);
    optical_encoder_2_result = init_optical_encoder(GPIO_NUM_15, false, 0.2356, 30, 0.2);

    printf("-----------------------------INITIALIZING MODULES DONE... ");

    if (mpu6050 && gy271 && nrf24 && optical_encoder_1_result >= 0 && optical_encoder_2_result >= 0){
        printf("OK\n");
    }else{
        printf("NOT OK %d %d %d %d %d\n", mpu6050, gy271, nrf24, optical_encoder_1_result, optical_encoder_2_result);
        return;
    }

    nrf24_rx_mode(tx_address, 10);
}

// Abstract away peripheral setup
void init_esp32_peripherals(){
    init_spi3();
}

// initialize the initial time of the 
void init_loop_timer(){
    // Init loop timer
    loop_start_time = esp_timer_get_time()/1000;

    // There were some errors if i dont add a delay here. 
    vTaskDelay(1000 / portTICK_RATE_MS);
}

// Get the initial position of the robot so it is not confused when it gets into the loop
void get_initial_position(){
    // Find the initial position in degrees and apply it to the gyro measurement integral
    // This will tell the robot which way to go to get the actual upward
    mpu6050_get_accelerometer_readings_gravity(acceleration_data);
    mpu6050_get_gyro_readings_dps(gyro_angular);
    gy271_magnetometer_readings_micro_teslas(magnetometer_data);
    fix_mag_axis(magnetometer_data); // Switches around the x and the y to match mpu6050

    calculate_degrees_x_y(acceleration_data, &accelerometer_x_rotation, &accelerometer_y_rotation);

    // Get a good raw yaw for calculations later
    calculate_yaw(magnetometer_data, &magnetometer_z_rotation);
    last_raw_yaw = magnetometer_z_rotation;

    calculate_yaw_tilt_compensated(magnetometer_data, &magnetometer_z_rotation, accelerometer_x_rotation, accelerometer_y_rotation);

    // Use accelerometer as the degrees are instantaneous. Gyro would need multiple loops.
    gyro_degrees[0] = accelerometer_x_rotation;
    gyro_degrees[1] = accelerometer_y_rotation;
    gyro_degrees[2] = magnetometer_z_rotation; 

    printf("Initial location x: %.2f y: %.2f, z: %.2f\n", gyro_degrees[0], gyro_degrees[1], gyro_degrees[2]);

    // Set the desired yaw as the initial one
    target_yaw = gyro_degrees[2];
}


// Extract the values form a slash separated stirng into specific variables for motion control parameters 
void extract_joystick_request_values(char *request, uint8_t request_size, uint8_t *throttle, uint8_t *yaw, uint8_t *roll, uint8_t *pitch)
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
    char roll_string[length + 1];
    strncpy(roll_string, start, length);
    roll_string[length] = '\0';
    *roll = atoi(roll_string);
    //printf("'%s'\n", roll_string);

    start = strchr(end, '/') + 1;
    end = strchr(start, '/');
    if(start == NULL || end == NULL ) return;
    length = end - start;
    char pitch_string[length + 1];
    strncpy(pitch_string, start, length);
    pitch_string[length] = '\0';
    *pitch = atoi(pitch_string);
    //printf("'%s'\n", pitch_string);
}

// Extract specifically the request type from a slash separated string
void extract_request_type(char *request, uint8_t request_size, char *type_output){
    char* start = strchr(request, '/') + 1;
    char* end = strchr(start, '/');
    if(start == NULL || end == NULL ) return;
    int length = end - start;

    // You better be sure the length of the output string is big enough
    strncpy(type_output, start, length);
    type_output[length] = '\0';
    //printf("'%s'\n", type_output);
}

// Extract the values form a slash separated stirng into specific variables for pid control parameters 
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

// Delay the loop enough to match the refresh rate specified
void handle_loop_timing(){
    loop_end_time = esp_timer_get_time()/1000;
    delta_loop_time = loop_end_time - loop_start_time;
    // printf("Tim: %d ms", delta_loop_time);
    if (delta_loop_time < (1000 / refresh_rate_hz))
    {
        vTaskDelay(((1000 / refresh_rate_hz) - delta_loop_time) / portTICK_RATE_MS);
    }
    loop_start_time = esp_timer_get_time()/1000;
}

// Print out how much time has passed since the start of the loop. To debug issues with performance
void track_time(){
    uint32_t delta_loop_time_temp = loop_end_time - loop_start_time;

    printf("%5d ms ", delta_loop_time_temp);
}

// Map value from a specified range to a new range
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

// Apply a dead zone to a value 
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

// Switch to transmit mode and send out a the base pid values to the remote
void send_pid_base_info_to_remote(){
    nrf24_tx_mode(tx_address, 10);
    // Delay a bit to avoid problem
    vTaskDelay(250 / portTICK_RATE_MS);

    // Make a slash separated value
    char *string = generate_message_pid_values_nrf24(
        base_pitch_gain_p, 
        base_pitch_gain_i, 
        base_pitch_gain_d,
        base_pitch_master_gain
    );
    

    // The remote always receives data as a gibberish with corrupted characters. Sending many of them will mean the remote can reconstruct the message
    // And no crc check did not work, i tried.
    for (size_t i = 0; i < 100; i++)
    {
        if(!nrf24_transmit(string)){
            printf("FAILED\n"); // Very djank
        }
    }
    free(string);
    
    // Switch back to receiver mode
    nrf24_rx_mode(tx_address, 10);
}

// Switch to transmit mode and send out a the added  pid values to the remote
void send_pid_added_info_to_remote(){
    nrf24_tx_mode(tx_address, 10);
    // Delay a bit to avoid problem
    vTaskDelay(250 / portTICK_RATE_MS);

    // Make a slash separated value
    char *string = generate_message_pid_values_nrf24(
        added_pitch_gain_p, 
        added_pitch_gain_i, 
        added_pitch_gain_d,
        added_pitch_master_gain
    );
    
    // The remote always receives data as a gibberish with corrupted characters. Sending many of them will mean the remote can reconstruct the message
    // And no crc check did not work, i tried.
    for (size_t i = 0; i < 200; i++)
    {
        if(!nrf24_transmit(string)){
            printf("FAILED\n"); // Very djank
        }
    }
    free(string);
    
    // Switch back to receiver mode
    nrf24_rx_mode(tx_address, 10);
}

char* generate_message_pid_values_nrf24(double base_proportional, double base_integral, double base_derivative, double base_master){
    // calculate the length of the resulting string

    // the s is there for reasons... I just ran out of space on the 32 byte buffer for sending. It was originally supposed to be a full name
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