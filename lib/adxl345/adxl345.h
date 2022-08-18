#pragma once

#include <stdio.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/uart.h>
#include <string.h>
#include "driver/i2c.h"

// Needs 3.6V
// Needs to be connected to i2c 

bool init_adxl345(uint scl_pin, uint sda_pin);
void adxl345_get_axis_readings(int16_t* data);
