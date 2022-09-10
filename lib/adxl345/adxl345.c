#include "./adxl345.h"

#define ADXL345 0x53


#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

bool init_adxl345(uint scl_pin, uint sda_pin){

    uint8_t dataInit[] = {0x2D, 8};
    // the second value has to be 2's complement
    uint8_t x_calibration[] = {0x1E, 0};
    uint8_t y_calibration[] = {0x1F, 0};
    // uint8_t z_calibration[] = {0x20, 0b11111101};
    uint8_t z_calibration[] = {0x20, 0b11110110};



    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };


    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);

    // calibration
    i2c_master_write_to_device(I2C_MASTER_NUM, ADXL345, dataInit, sizeof(dataInit), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_master_write_to_device(I2C_MASTER_NUM, ADXL345, x_calibration, sizeof(dataInit), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_master_write_to_device(I2C_MASTER_NUM, ADXL345, y_calibration, sizeof(dataInit), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_master_write_to_device(I2C_MASTER_NUM, ADXL345, z_calibration, sizeof(dataInit), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    return true;
}

void adxl345_get_axis_readings_int(int16_t* data){
    uint8_t data_register[] = {0x32};
    uint8_t retrieved_data[] = {0,0,0,0,0,0};

    i2c_master_write_read_device(
        I2C_MASTER_NUM, 
        ADXL345, 
        data_register, 
        sizeof(data_register), 
        retrieved_data, 
        sizeof(retrieved_data), 
        I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS
    );
    
    int16_t X_out, Y_out, Z_out;

    X_out = (int16_t)retrieved_data[0] | ((int16_t)retrieved_data[1] << 8);
    // X_out = X_out/256;
    Y_out = (int16_t)retrieved_data[2] | ((int16_t)retrieved_data[3] << 8);
    // Y_out = Y_out/256;
    Z_out = (int16_t)retrieved_data[4] | ((int16_t)retrieved_data[5] << 8);
    // Z_out = Z_out/256;

    data[0] = X_out;
    data[1] = Y_out;
    data[2] = Z_out;
}

void adxl345_get_axis_readings_float(float* data){
    uint8_t data_register[] = {0x32};
    uint8_t retrieved_data[] = {0,0,0,0,0,0};

    i2c_master_write_read_device(
        I2C_MASTER_NUM, 
        ADXL345, 
        data_register, 
        sizeof(data_register), 
        retrieved_data, 
        sizeof(retrieved_data), 
        I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS
    );

    int16_t X, Y, Z;
    float X_out, Y_out, Z_out;

    X = (int16_t)retrieved_data[0] | ((int16_t)retrieved_data[1] << 8);
    Y = (int16_t)retrieved_data[2] | ((int16_t)retrieved_data[3] << 8);
    Z = (int16_t)retrieved_data[4] | ((int16_t)retrieved_data[5] << 8);

    // printf("X= %d Y= %d Z= %d\n", 
    //     X, 
    //     Y, 
    //     Z
    // );
    
    X_out = ((float) X) / 256.0;
    Y_out = ((float) Y) / 256.0;
    Z_out = ((float) Z) / 256.0;

    data[0] = X_out;
    data[1] = Y_out;
    data[2] = Z_out;
}


void calculate_pitch_and_roll(float* data, float *roll, float *pitch){
    float x = data[0] * 9.81;
    float y = data[1];
    float z = data[2];

    // rotation around the x axis
    *roll = atan2f(y, z) * 180 / M_PI;

    // rotation around the y axis
    *pitch = asinf(x/9.81) * 180 / M_PI;
}