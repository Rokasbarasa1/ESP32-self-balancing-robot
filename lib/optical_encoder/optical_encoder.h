#pragma once 
#include <stdio.h>
#include "driver/gpio.h"
#include "esp_timer.h"

struct optical_encoder
{
    bool m_clockwise;
    int32_t m_counter;
    int64_t m_last_time_microseconds;
    int64_t m_latest_time_microseconds;
    int64_t m_delta_time_microseconds;
    uint8_t m_interrupt_pin;
    gpio_config_t m_handle;
    double m_circle_perimeter_meters;
    uint16_t m_encoder_resolution;
    double m_step_size_meters;
    double m_value_drop_threshold_seconds;
};

int8_t init_optical_encoder(uint8_t interrupt_pin, bool init_interrupts, double circle_perimeter_meters, uint16_t encoder_resolution, double value_drop_threshold_seconds);
int32_t optical_encoder_get_count(int8_t encoder_id);
void optical_encoder_set_count(int8_t encoder_id, int32_t count);
void optical_encoder_set_clockwise(int8_t encoder_id, bool clockwise);
double optical_encoder_get_speed_meters_per_second(int8_t encoder_id, double max, double min);
double optical_encoder_get_hertz(int8_t encoder_id, double max, double min);
double optical_encoder_get_rpm(int8_t encoder_id, double max, double min);
bool optical_encoder_check_if_value_bad(int8_t encoder_id);
bool optical_encoder_get_clockwise(int8_t encoder_id);