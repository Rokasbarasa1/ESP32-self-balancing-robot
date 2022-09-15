#pragma once
#include <stdio.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/ledc.h"
#include <math.h>

void init_TB6612(uint motor_a_opt1, uint motor_a_opt2, uint motor_a_pwm, uint motor_b_opt1, uint motor_b_opt2, uint motor_b_pwm);

// -100 to 100
void change_speed_motor_A(float speed, float trim_speed);
void change_speed_motor_B(float speed, float trim_speed);