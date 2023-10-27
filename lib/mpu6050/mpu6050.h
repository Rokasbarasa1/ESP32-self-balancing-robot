#pragma once

#include <stdio.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/uart.h>
#include <string.h>
#include "driver/i2c.h"
#include <math.h>

enum t_power_management {
    PWR_RESET    = 0b10000000,
    PWR_SLEEP    = 0b01000000,
    PWR_CYCLE    = 0b00100000,
    PWR_TEMP_DIS = 0b00001000,
    PWR_CLOCK_INTERNAL_8MHZ = 0b00000000,
    PWR_CLOCK_X_GYRO = 0b00000001,
    PWR_CLOCK_INTERNAL_STOP = 0b00000111,
};

bool init_mpu6050(uint scl_pin, uint sda_pin, bool initialize_i2c, bool apply_calibration, float accelerometer_correction[3], float gyro_correction[3], float complementary_ratio);
void mpu6050_get_accelerometer_readings_gravity(float *data);
void mpu6050_get_gyro_readings_dps(float *data);
void calculate_pitch_and_roll(float *data, float *roll, float *pitch);
void calculate_degrees_x_y(float *data, float *rotation_around_x, float *rotation_around_y);
void find_accelerometer_error(uint64_t sample_size);
void find_gyro_error(uint64_t sample_size);
void convert_angular_rotation_to_degrees(float* gyro_angular, float* gyro_degrees, float rotation_around_x, float rotation_around_y, float rotation_around_z, int64_t time);
void convert_angular_rotation_to_degrees_x_y(float* gyro_angular, float* gyro_degrees, float rotation_around_x, float rotation_around_y, int64_t time, bool set_timestamp);
void convert_angular_rotation_to_degrees_z(float* gyro_angular, float* gyro_degrees, float rotation_around_z, int64_t time);