#include "./pid.h"

#include <string.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
/**
 * @brief Initialize pid configuration and store it in struct
 * 
 * @param gain_proportional 
 * @param gain_integral 
 * @param gain_derivative 
 * @param desired_value value that you want to achieve
 * @param time the current time in ticks. Stm32 tick
 * @return struct pid 
 */
struct pid pid_init(
    double gain_proportional, 
    double gain_integral, 
    double gain_derivative, 
    double desired_value,
    int64_t time,
    double max_value,
    double min_value,
    uint8_t stop_windup
){
    struct pid new_pid;

    new_pid.m_gain_proportional = gain_proportional;
    new_pid.m_gain_integral = gain_integral;
    new_pid.m_gain_derivative = gain_derivative;
    new_pid.m_integral_sum = 0;
    new_pid.m_last_error = 0;
    new_pid.m_desired_value = desired_value;
    new_pid.m_previous_time = time;
    new_pid.m_max_value = max_value;
    new_pid.m_min_value = min_value;
    new_pid.m_stop_windup = stop_windup;

    return new_pid;
}


/**
 * @brief Calculate the error based on the configuration of the pid and the new value
 * 
 * @param pid_instance pid config
 * @param value current value that the error will be calculated for 
 * @param time current time in ticks. Stm32 tick 
 * @return double error result
 */
double pid_get_error(struct pid* pid_instance, double value, int64_t time){

    double error_p = 0, error_i = 0, error_d = 0;

    double error = (pid_instance->m_desired_value - value);

    double time_passed_proportion = (((double)time/1000.0)-((double)pid_instance->m_previous_time/1000.0)) / 1000.0;

    // printf("%6.10f = ((%lld/1000.0)-(%lld/1000.0)) / 1000.0;", time_passed_proportion, time, pid_instance->m_previous_time );
    
    // proportional
    {
        error_p = error;
    }

    // integral
    {
        pid_instance->m_integral_sum += (error * time_passed_proportion);
        // printf("%6.9f", (error * time_passed_proportion));

        if(pid_instance->m_stop_windup){
            // clamp the integral if it is getting out of bounds
            if(pid_instance->m_integral_sum > pid_instance->m_max_value){
                pid_instance->m_integral_sum = pid_instance->m_max_value;
            }else if(pid_instance->m_integral_sum < pid_instance->m_min_value){
                pid_instance->m_integral_sum = pid_instance->m_min_value;
            }
        }
        error_i = pid_instance->m_integral_sum;
    }

    // derivative
    {
        // divide by the time passed
        error_d = (error_p - pid_instance->m_last_error) * time_passed_proportion;

        // Dont let it get out of bounds 
        if(error_d > pid_instance->m_max_value){
            error_d = pid_instance->m_max_value;
        }else if(error_d < pid_instance->m_min_value){
            error_d = pid_instance->m_min_value;
        }
        
        // set the previous error for the next iteration
        pid_instance->m_last_error = error;
    }

    // printf("P| %6.3f %6.3f | ", (error_p * pid_instance->m_gain_proportional), error_p);
    // printf("I| %6.3f %6.3f | ", (error_i * pid_instance->m_gain_integral), error_i);
    // printf("D| %6.3f %6.3f | ", (error_d * pid_instance->m_gain_derivative), error_d);


    // printf("p: %8.4f, ", pid_instance->m_gain_proportional * error_p);
    // printf("i: %8.4f, ", pid_instance->m_gain_integral * error_i);
    // printf("d: %8.4f, ", pid_instance->m_gain_derivative * error_d);

    // printf("ERRORS, %8.4f, %8.4f, %8.4f, ", pid_instance->m_gain_proportional * error_p, pid_instance->m_gain_integral * error_i, pid_instance->m_gain_derivative * error_d);


    if(pid_instance->m_gain_integral * error_i > pid_instance->m_max_value){
        // printf("P:%8.4f,I:%8.4f,D:%8.4f\n", pid_instance->m_gain_proportional * error_p, pid_instance->m_max_value, pid_instance->m_gain_derivative * error_d);
        printf("%8.4f,%8.4f,%8.4f,%8.4f\n", value, pid_instance->m_gain_proportional * error_p, pid_instance->m_max_value, pid_instance->m_gain_derivative * error_d);
    }else if(pid_instance->m_gain_integral * error_i < pid_instance->m_min_value){
        // printf("P:%8.4f,I:%8.4f,D:%8.4f\n", pid_instance->m_gain_proportional * error_p, pid_instance->m_min_value, pid_instance->m_gain_derivative * error_d);
        printf("%8.4f,%8.4f,%8.4f,%8.4f\n", value, pid_instance->m_gain_proportional * error_p, pid_instance->m_min_value, pid_instance->m_gain_derivative * error_d);
    }else{
        // printf("P:%8.4f,I:%8.4f,D:%8.4f\n", pid_instance->m_gain_proportional * error_p, pid_instance->m_gain_integral * error_i, pid_instance->m_gain_derivative * error_d);
        printf("%8.4f,%8.4f,%8.4f,%8.4f\n", value, pid_instance->m_gain_proportional * error_p, pid_instance->m_gain_integral * error_i, pid_instance->m_gain_derivative * error_d);
    }

    // printf("ERRORS, 25.0, 50.0, 75.0, ");

    // end result
    double total_error = (pid_instance->m_gain_proportional * error_p) + 
                (pid_instance->m_gain_integral * error_i) + 
                (pid_instance->m_gain_derivative * error_d) ;

    // save the time for next calculation
    pid_instance->m_previous_time = time;
    return total_error;
}

/**
 * @brief Change the desired value. Mainly for user inputs for pid
 * 
 * @param pid_instance pid config
 * @param value new desired value
 */
void pid_set_desired_value(struct pid* pid_instance, double value){
    pid_instance->m_desired_value = value;
}