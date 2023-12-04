#include "./qmc5883l.h"

#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000


#define qmc5883l_I2C_ID 0x0D
#define ID_REG 0x0D
#define ID_VALUE 0b11111111
#define CONTROL1_REG 0x09
#define CONTROL2_REG 0x0A

#define OUTPUT_DATA1_REG 0x00

// Storage of hard iron correction, values should be replaced by what is passed
volatile float m_hard_iron[3] = {
    0, 0, 0};

// Storage of soft iron correction, values should be replaced by what is passed
volatile float m_soft_iron[3][3] = {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}};

// max value output is at 200 Hz
bool init_qmc5883l(uint scl_pin, uint sda_pin, bool initialize_i2c, bool apply_calibration, float hard_iron[3],  float soft_iron[3][3]){

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

    // assign the correction for irons
    if(apply_calibration){
        for (uint i = 0; i < 3; i++){
            m_hard_iron[i] = hard_iron[i];
        }

        for (uint i = 0; i < 3; i++){
            for (uint k = 0; k < 3; k++){
                m_soft_iron[i][k] = soft_iron[i][k];
            }
        }
    }
    

    // Test the sensor by reading it's address register
    uint8_t test_register[] = {ID_REG};
    uint8_t test_data[] = {0};

    i2c_master_write_read_device(
        I2C_MASTER_NUM, 
        qmc5883l_I2C_ID, 
        test_register, 
        1, 
        test_data, 
        1, 
        I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS
    );

    // Check if the id value is as it should be
    if (test_data[0] != ID_VALUE)
    {
        printf("qmc5883l initialization failed: %d != %d\n", test_data[0], ID_VALUE);
        return false;
    }

    // reset it 
    uint8_t settings2[] = {CONTROL2_REG, INTERRUPT_PIN_DISABLED};
    i2c_master_write_to_device(
        I2C_MASTER_NUM, 
        qmc5883l_I2C_ID, 
        settings2, 
        2, 
        I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    // Set some essential settings that control the data being outputted
    uint8_t settings1[] = {CONTROL1_REG, OS_RATIO_512|MEASURE_SCALE_2G|ODR_50HZ|MODE_CONTINUOUS};
    i2c_master_write_to_device(
        I2C_MASTER_NUM, 
        qmc5883l_I2C_ID, 
        settings1, 
        2, 
        I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    
    printf("qmc5883l initialized\n");
    return true;
}

void qmc5883l_magnetometer_readings_micro_teslas(float *data){
    uint8_t data_register[] = {OUTPUT_DATA1_REG};
    uint8_t retrieved_data[] = {0, 0, 0, 0, 0, 0};

    i2c_master_write_read_device(
        I2C_MASTER_NUM, 
        qmc5883l_I2C_ID, 
        data_register, 
        1, 
        retrieved_data, 
        6, 
        I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS
    );

    // First is least significant and second is most significant
    int16_t X = ((int16_t)retrieved_data[1] << 8) | (int16_t)retrieved_data[0];
    int16_t Y = ((int16_t)retrieved_data[3] << 8) | (int16_t)retrieved_data[2];
    int16_t Z = ((int16_t)retrieved_data[5] << 8) | (int16_t)retrieved_data[4];
    
    // Convert the mag's adc value to gauss
    // ADC accuracy is 2 MilliGauss per 1 step
    // Divide by 20 to do micro teslas
    data[0] = (float)X / 20;
    data[1] = (float)Y / 20;
    data[2] = (float)Z / 20;

    // Use the soft and hard iron calibrations
    for (uint8_t i = 0; i < 3; i++)
    {
        data[i] = data[i] - m_hard_iron[i];
    }

    for (uint8_t i = 0; i < 3; i++)
    {
        data[i] = (m_soft_iron[i][0] * data[0]) +
                  (m_soft_iron[i][1] * data[1]) +
                  (m_soft_iron[i][2] * data[2]);
    }
}

void calculate_yaw(float *magnetometer_data, float *yaw)
{
    float x = magnetometer_data[0];
    float y = magnetometer_data[1];
    float z = magnetometer_data[2];

    // rotation around the z axis
    *yaw = atan2f(y, x) * (180 / M_PI);

    // Convert yaw to [0, 360] range
    if (*yaw > 180) {
        *yaw -= 360;
    }
}

void calculate_yaw_tilt_compensated(float *magnetometer_data, float *yaw, float gyro_x_axis_rotation_degrees, float gyro_y_axis_rotation_degrees){
    float roll = gyro_x_axis_rotation_degrees * (M_PI / 180);  //  Convert roll from degrees to radians
    float pitch = gyro_y_axis_rotation_degrees * (M_PI / 180);  // Convert pitch from degrees to radians

    float mx = magnetometer_data[0];
    float my = magnetometer_data[1];
    float mz = magnetometer_data[2];

    float Xc = mx * cos(pitch) + mz * sin(pitch);
    float Yc = mx * sin(roll) * sin(pitch) + my * cos(roll) - mz * sin(roll) * cos(pitch);

    *yaw = atan2(Yc, Xc) * (180 / M_PI);
    if (*yaw > 180) {
        *yaw -= 360;
    }
}