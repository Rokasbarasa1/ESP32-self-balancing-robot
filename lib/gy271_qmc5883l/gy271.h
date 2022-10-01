#pragma once

#include <stdio.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/uart.h>
#include <string.h>
#include "driver/i2c.h"
#include <math.h>


bool init_gy271(uint scl_pin, uint sda_pin, bool initialize_i2c, bool apply_calibration, float hard_iron[3],  float soft_iron[3][3]);
void gy271_magnetometer_readings_micro_teslas(float* data);
void find_mag_error(uint sample_size);
void calculate_yaw(float* magnetometer_data, float *yaw);

