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

bool init_adxl345(uint scl_pin, uint sda_pin);
void adxl345_get_axis_readings_int(int16_t* data);
void adxl345_get_axis_readings_float(double* data);
void calculate_pitch_and_roll(double* data, double *roll, double *pitch);