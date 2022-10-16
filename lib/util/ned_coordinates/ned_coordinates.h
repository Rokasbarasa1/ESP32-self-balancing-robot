#pragma once

void get_ned_coordinates(float* acceleration_data, float* magnetometer_data, float* north_vector, float* east_vector, float* down_vector);
float angle_between_2d_vectors(float ax, float ay, float bx, float by );