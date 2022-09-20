#pragma once

#include <stdio.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/uart.h>
#include <string.h>
#include "driver/i2c.h"
#include <math.h>
// Needs 3.6V
// Needs to be connected to i2c 

void init_mpu6050(uint scl_pin, uint sda_pin);
void mpu6050_accelerometer_readings_int(int16_t* data);
void mpu6050_accelerometer_readings_float(float* data);
void mpu6050_gyro_readings_float(float* data);
void calculate_pitch_and_roll(float* data, float *roll, float *pitch);