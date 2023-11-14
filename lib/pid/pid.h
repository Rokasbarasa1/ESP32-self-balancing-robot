#pragma once
#include "stdint.h"

struct pid
{
    double m_gain_proportional;
    double m_gain_integral;
    double m_gain_derivative;
    double m_integral_sum;
    double m_last_error;
    double m_desired_value;
    int64_t m_previous_time;
    double m_max_value;
    double m_min_value;
    uint8_t m_stop_windup;
};

struct pid pid_init(
    double gain_proportional, 
    double gain_integral, 
    double gain_derivative, 
    double desired_value,
    int64_t time,
    double max_value,
    double min_value,
    uint8_t stop_windup);
double pid_get_error(struct pid* pid_instance, double value, int64_t time);
double pid_get_error_own_error(struct pid* pid_instance, double error, int64_t time);
void pid_set_desired_value(struct pid* pid_instance, double value);
void pid_set_proportional_gain(struct pid* pid_instance, double proportional_gain);
void pid_set_integral_gain(struct pid* pid_instance, double integral_gain);
void pid_set_derivative_gain(struct pid* pid_instance, double derivative_gain);
void pid_reset_integral_sum(struct pid* pid_instance);
void pid_set_previous_time(struct pid* pid_instance, int64_t time);