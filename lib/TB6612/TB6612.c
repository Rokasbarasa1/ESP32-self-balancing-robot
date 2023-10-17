#include "./TB6612.h"

volatile uint a1 = 0;
volatile uint a2 = 0;
volatile uint b1 = 0;
volatile uint b2 = 0;


// #define LEDC_TIMER              LEDC_TIMER_0
// #define LEDC_MODE               LEDC_LOW_SPEED_MODE
// #define LEDC_OUTPUT_IO          (5) // Define the output GPIO
// #define LEDC_CHANNEL            LEDC_CHANNEL_0
// #define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
// #define LEDC_DUTY               (4095) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
// #define LEDC_FREQUENCY          (5000) // Frequency in Hertz. Set frequency at 5 kHz


void init_TB6612(uint motor_a_opt1, uint motor_a_opt2, uint motor_a_pwm, uint motor_b_opt1, uint motor_b_opt2, uint motor_b_pwm){
    
    a1 = motor_a_opt1;
    a2 = motor_a_opt2;
    b1 = motor_b_opt1;
    b2 = motor_b_opt2;

    // a
    gpio_set_direction(motor_a_opt1, GPIO_MODE_OUTPUT);
    gpio_set_direction(motor_a_opt2, GPIO_MODE_OUTPUT);

    // a pwm
    ledc_timer_config_t ledc_timer1 = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_13_BIT,
        .freq_hz          = 5000,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer1);
    ledc_channel_config_t ledc_channel1 = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_LOW_SPEED_MODE,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = motor_a_pwm,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel1);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, 0, 0);

    // b
    gpio_set_direction(motor_b_opt1, GPIO_MODE_OUTPUT);
    gpio_set_direction(motor_b_opt2, GPIO_MODE_OUTPUT);

    // b pwm
    ledc_timer_config_t ledc_timer2 = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_13_BIT,
        .freq_hz          = 5000,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer2);

    ledc_channel_config_t ledc_channel2 = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_1,
        .timer_sel      = LEDC_LOW_SPEED_MODE,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = motor_b_pwm,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel2);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, 1, 0);
}

// -100 to 100

// but they start spinning at 30 speed so ...
void change_speed_motor_B(float speed, float trim_speed){
    if( speed < -80){
        speed = -80;
    }else if( speed > 80){
        speed = 80;
    }

    if(speed > 0){
        speed = (trim_speed) + ((100 - (trim_speed)) / (100 - 0)) * (speed - 0);

        // forward
        gpio_set_level(a1, 1);
        gpio_set_level(a2, 0);
        
        // set pwm
        ledc_set_duty(LEDC_LOW_SPEED_MODE, 0, (int)((pow(2.0, 13.0) - 1.0) * (speed / 100.0)));
    }else if(speed < 0){
        speed = ((trim_speed) + ((100 - (trim_speed)) / (100 - 0)) * ((speed * -1) - 0)) * -1;

        // reverse
        gpio_set_level(a1, 0);
        gpio_set_level(a2, 1);

        // set pwm
        ledc_set_duty(LEDC_LOW_SPEED_MODE, 0, (int)((pow(2.0, 13.0) - 1.0) * ((speed * -1) / 100.0)));
    }else{
        gpio_set_level(a1, 0);
        gpio_set_level(a2, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, 0, 0);
    }

    ledc_update_duty(LEDC_LOW_SPEED_MODE, 0);
}

void change_speed_motor_A(float speed, float trim_speed){
    // Clamping extreme values
    if( speed < -70){
        speed = -70;
    }else if( speed > 70){
        speed = 70;
    }

    if(speed > 0){
        speed = (trim_speed) + ((100 - (trim_speed)) / (100 - 0)) * (speed - 0);

        // forward
        gpio_set_level(b1, 1);
        gpio_set_level(b2, 0);

        // set pwm
        ledc_set_duty(LEDC_LOW_SPEED_MODE, 1, (int)((pow(2.0, 13.0) - 1.0) * (speed / 100.0)));

    }else if(speed < 0){
        speed = ((trim_speed) + ((100 - (trim_speed)) / (100 - 0)) * ((speed * -1) - 0)) * -1;

        // reverse
        gpio_set_level(b1, 0);
        gpio_set_level(b2, 1);

        // set pwm
        ledc_set_duty(LEDC_LOW_SPEED_MODE, 1, (int)((pow(2.0, 13.0) - 1.0) * ((speed * -1) / 100.0)));
    }else{
        gpio_set_level(b1, 0);
        gpio_set_level(b2, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, 1, 0);
    }

    ledc_update_duty(LEDC_LOW_SPEED_MODE, 1);
}