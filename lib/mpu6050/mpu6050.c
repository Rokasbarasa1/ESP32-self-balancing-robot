#include "./mpu6050.h"

#define MPU6050 0x68

#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000


void init_mpu6050(uint scl_pin, uint sda_pin, bool initialize_i2c){

    if(initialize_i2c){
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
    }

    // reset it 
    uint8_t reset_device1[] = {0x6B, 0b10001000};
    i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050, reset_device1, sizeof(reset_device1), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    vTaskDelay(100 / portTICK_RATE_MS);

    uint8_t reset_device2[] = {0x6B, 0b00000111};
    i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050, reset_device2, sizeof(reset_device2), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    vTaskDelay(100 / portTICK_RATE_MS);

    uint8_t reset_device3[] = {0x6B, 0b00001000};
    i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050, reset_device3, sizeof(reset_device3), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    vTaskDelay(100 / portTICK_RATE_MS);
}

void mpu6050_accelerometer_readings_int(int16_t* data){
    uint8_t data_register[] = {0x3B};
    uint8_t retrieved_data[] = {0,0,0,0,0,0};

    i2c_master_write_read_device(
        I2C_MASTER_NUM, 
        MPU6050, 
        data_register, 
        sizeof(data_register), 
        retrieved_data, 
        sizeof(retrieved_data), 
        I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS
    );
    
    int16_t X_out, Y_out, Z_out;


    X_out = (int16_t)retrieved_data[0] | ((int16_t)retrieved_data[1] << 8);
    Y_out = (int16_t)retrieved_data[2] | ((int16_t)retrieved_data[3] << 8);
    Z_out = (int16_t)retrieved_data[4] | ((int16_t)retrieved_data[5] << 8);
    data[0] = X_out;
    data[1] = Y_out;
    data[2] = Z_out;
}

void mpu6050_accelerometer_readings_float(float* data){
    uint8_t data_register[] = {0x3B};
    uint8_t retrieved_data[] = {0,0,0,0,0,0};

    i2c_master_write_read_device(
        I2C_MASTER_NUM, 
        MPU6050, 
        data_register, 
        sizeof(data_register), 
        retrieved_data, 
        sizeof(retrieved_data), 
        I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS
    );

    int16_t X, Y, Z;
    float X_out, Y_out, Z_out;

    X = ((int16_t)retrieved_data[0] << 8) | (int16_t)retrieved_data[1];
    Y = ((int16_t)retrieved_data[2] << 8) | (int16_t)retrieved_data[3];
    Z = ((int16_t)retrieved_data[4] << 8) | (int16_t)retrieved_data[5];
    
    X_out = ((float) X) / 16384.0;
    Y_out = ((float) Y) / 16384.0;
    Z_out = ((float) Z) / 16384.0;

    data[0] = X_out - 0.076622;
    data[1] = Y_out - 0.009298;
    data[2] = Z_out + 0.036924;
}

void mpu6050_gyro_readings_float(float* data){
    uint8_t data_register[] = {0x43};
    uint8_t retrieved_data[] = {0,0,0,0,0,0};

    i2c_master_write_read_device(
        I2C_MASTER_NUM, 
        MPU6050, 
        data_register, 
        sizeof(data_register), 
        retrieved_data, 
        sizeof(retrieved_data), 
        I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS
    );

    int16_t X, Y, Z;
    float X_out, Y_out, Z_out;

    X = ((int16_t)retrieved_data[0] << 8) | (int16_t)retrieved_data[1];
    Y = ((int16_t)retrieved_data[2] << 8) | (int16_t)retrieved_data[3];
    Z = ((int16_t)retrieved_data[4] << 8) | (int16_t)retrieved_data[5];
    
    X_out = ((float) X) / 131.0;
    Y_out = ((float) Y) / 131.0;
    Z_out = ((float) Z) / 131.0;

    data[0] = X_out + 0.096947;
    data[1] = Y_out + 4.177492;
    data[2] = Z_out - 0.440870;
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

void find_accelerometer_error(uint sample_size){

    float x_sum = 0, y_sum = 0, z_sum = 0;
    float data[] = {0,0,0};
    
    for(uint i = 0; i < sample_size ; i++){
        mpu6050_accelerometer_readings_float(data);
        x_sum += data[0];
        y_sum += data[1];
        z_sum += data[2];
        // It can sample stuff at 1KHz
        // but 0.5Khz is just to be safe
        vTaskDelay(2 / portTICK_RATE_MS);
    }

    printf(
        "ACCELEROMETER errors: X: %f   Y: %f   Z: %f\n", 
        x_sum/sample_size, 
        y_sum/sample_size, 
        z_sum/sample_size
    );
}

void find_gyro_error(uint sample_size){

    float x_sum = 0, y_sum = 0, z_sum = 0;
    float data[] = {0,0,0};
    
    for(uint i = 0; i < sample_size ; i++){
        mpu6050_gyro_readings_float(data);

        x_sum += data[0];
        y_sum += data[1];
        z_sum += data[2];
        // It can sample stuff at 1KHz
        // but 0.5Khz is just to be safe
        vTaskDelay(2 / portTICK_RATE_MS);
    }

    printf(
        "GYRO errors: X: %f   Y: %f   Z: %f\n", 
        x_sum/sample_size, 
        y_sum/sample_size, 
        z_sum/sample_size
    );
}