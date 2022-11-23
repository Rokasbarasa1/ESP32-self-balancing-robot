#include "./ned_coordinates.h"
#include <stdio.h>
#include <math.h>


void get_ned_coordinates(float* acceleration_data, float* magnetometer_data, float* north_vector, float* east_vector, float* down_vector){

    // A unit
    const float acc_vector_length = sqrt(pow(acceleration_data[0], 2) + pow(acceleration_data[1], 2) + pow(acceleration_data[2], 2));
    float acc_unit_vector[] = {
        acceleration_data[0] / acc_vector_length,
        acceleration_data[1] / acc_vector_length,
        acceleration_data[2] / acc_vector_length
    };


    // M unit
    const float mag_vector_length = sqrt(pow(magnetometer_data[0], 2) + pow(magnetometer_data[1], 2) + pow(magnetometer_data[2], 2));
    float mag_unit_vector[] = {
          magnetometer_data[0] / mag_vector_length,
          magnetometer_data[1] / mag_vector_length,
          magnetometer_data[2] / mag_vector_length
    };

    // D unit
    float down_direction[] = {
        -acc_unit_vector[0],
        -acc_unit_vector[1],
        -acc_unit_vector[2]
    };


    // E 
    // cross product of direction of gravity and mag direction unit vector
    float east_direction[] ={
        down_direction[1] * mag_unit_vector[2] - down_direction[2] * mag_unit_vector[1],
        down_direction[2] * mag_unit_vector[0] - down_direction[0] * mag_unit_vector[2],
        down_direction[0] * mag_unit_vector[1] - down_direction[1] * mag_unit_vector[0],
    };

    // E unit
    // east normalization and turning into unit vector
    const float east_vector_length = sqrt(pow(east_direction[0], 2) + pow(east_direction[1], 2) + pow(east_direction[2], 2));
    float east_unit_direction[] = {
        east_direction[0] / east_vector_length,
        east_direction[1] / east_vector_length,
        east_direction[2] / east_vector_length
    };

    // N 
    // cross east and direction of gravity
    float north_direction[] ={
        east_unit_direction[1] * down_direction[2] - east_unit_direction[2] * down_direction[1],
        east_unit_direction[2] * down_direction[0] - east_unit_direction[0] * down_direction[2],
        east_unit_direction[0] * down_direction[1] - east_unit_direction[1] * down_direction[0],
    };

    // N unit
    const float north_vector_length = sqrt(pow(north_direction[0], 2) + pow(north_direction[1], 2) + pow(north_direction[2], 2));
    
    // Return NED coordinates
    north_vector[0] = north_direction[0] / north_vector_length;
    north_vector[1] = north_direction[1] / north_vector_length;
    north_vector[2] = north_direction[2] / north_vector_length;

    east_vector[0] = east_unit_direction[0];
    east_vector[1] = east_unit_direction[1];
    east_vector[2] = east_unit_direction[2];

    down_vector[0] = down_direction[0];
    down_vector[1] = down_direction[1];
    down_vector[2] = down_direction[2];
}


float angle_between_2d_vectors(float ax, float ay, float bx, float by ){
    float a_length = sqrt(pow(ax, 2) + pow(ay, 2));
    float b_length = sqrt(pow(bx, 2) + pow(by, 2));

    ax = ax /a_length;
    ax = ax /a_length;

    bx = bx /b_length;
    bx = bx /b_length;

    return atan2(
        (ax * by - ay * bx), 
        (ax * bx + ay * by)
    )*180/M_PI;
}