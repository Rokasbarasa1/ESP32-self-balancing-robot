#include "./optical_encoder.h"

struct optical_encoder optical_encoders[5];
uint8_t amount_of_encoders_in_array = 0;

static void IRAM_ATTR gpio_interrupt_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;

    uint8_t index_of_encoder = 200;
    for(uint8_t i = 0; i < amount_of_encoders_in_array; i++){
        if(gpio_num == optical_encoders[i].m_interrupt_pin){
            index_of_encoder = i;
            break;
        }
    }

    if(index_of_encoder == 200){
        return;
    }

    optical_encoders[index_of_encoder].m_delta_time_microseconds = esp_timer_get_time() - optical_encoders[index_of_encoder].m_last_time_microseconds;
    optical_encoders[index_of_encoder].m_last_time_microseconds = esp_timer_get_time();

    if(optical_encoders[index_of_encoder].m_clockwise){
        optical_encoders[index_of_encoder].m_counter++;
    }else{
        optical_encoders[index_of_encoder].m_counter--;
    }
}


int8_t init_optical_encoder(uint8_t interrupt_pin, bool init_interrupts, double circle_perimeter_meters, uint16_t encoder_resolution, double value_drop_threshold_seconds ){
    struct optical_encoder new_optical_encoder;

    // MEASURE METERS PER SECOND ##############################

    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;  // Interrupt on rising edge (GPIO_PIN_INTR_NEGEDGE for falling edge)
    io_conf.pin_bit_mask = (1ULL << interrupt_pin);  // Bit mask of the pin
    io_conf.mode = GPIO_MODE_INPUT;  // Set as Input
    io_conf.pull_down_en = 0;  // Disable pull-down mode
    io_conf.pull_up_en = 1;  // Enable pull-up mode
    gpio_config(&io_conf);  // Configure the GPIO with the settings

    if(init_interrupts){
        gpio_install_isr_service(0);  // Install gpio isr service
    }
    gpio_isr_handler_add(interrupt_pin, gpio_interrupt_handler, (void*) interrupt_pin);  // Add ISR handler
    
    // Configure the struct
    new_optical_encoder.m_clockwise = true;
    new_optical_encoder.m_counter = 0;
    new_optical_encoder.m_interrupt_pin = interrupt_pin;
    new_optical_encoder.m_handle = io_conf;
    new_optical_encoder.m_last_time_microseconds = esp_timer_get_time();
    new_optical_encoder.m_delta_time_microseconds = 1000;
    new_optical_encoder.m_circle_perimeter_meters = circle_perimeter_meters;
    new_optical_encoder.m_encoder_resolution = encoder_resolution;
    new_optical_encoder.m_value_drop_threshold_seconds = value_drop_threshold_seconds;
    new_optical_encoder.m_step_size_meters = new_optical_encoder.m_circle_perimeter_meters / (double) new_optical_encoder.m_encoder_resolution;

    // Store the struct in the array
    optical_encoders[amount_of_encoders_in_array] = new_optical_encoder;
    amount_of_encoders_in_array++;

    return amount_of_encoders_in_array - 1;
}


int32_t optical_encoder_get_count(int8_t encoder_id){
    return optical_encoders[encoder_id].m_counter;
}

void optical_encoder_set_count(int8_t encoder_id, int32_t count){
    optical_encoders[encoder_id].m_counter = count;
}

void optical_encoder_set_clockwise(int8_t encoder_id, bool clockwise){
    optical_encoders[encoder_id].m_clockwise = clockwise;
}

bool optical_encoder_get_clockwise(int8_t encoder_id){
    return optical_encoders[encoder_id].m_clockwise;
}

double optical_encoder_get_speed_meters_per_second(int8_t encoder_id, double max, double min){

    // Check if value stale
    if(optical_encoder_check_if_value_bad(encoder_id)){
        return 0.0;
    }

    // Get meters per second
    double value = optical_encoders[encoder_id].m_circle_perimeter_meters / (optical_encoders[encoder_id].m_delta_time_microseconds/1000000.0);

    if(!optical_encoders[encoder_id].m_clockwise){
        value *= -1;
    }

    if(value > max){
        return max;
    }else if (value < min){
        return min;
    }else{
        return value;
    }
}

double optical_encoder_get_hertz(int8_t encoder_id, double max, double min){

    // Check if value stale
    if(optical_encoder_check_if_value_bad(encoder_id)){
        return 0.0;
    }

    double value =  (1.0/(optical_encoders[encoder_id].m_delta_time_microseconds/1000000.0))/(double)optical_encoders[encoder_id].m_encoder_resolution;

    if(!optical_encoders[encoder_id].m_clockwise){
        value *= -1;
    }

    // printf("%9.5f ", value);

    if(value > max){
        return max;
    }else if (value < min){
        return min;
    }else{
        return value;
    }
}

double optical_encoder_get_rpm(int8_t encoder_id, double max, double min){
    return optical_encoder_get_hertz(encoder_id, max/60.0, min/60.0) * 60;
}

// Check if value stale
bool optical_encoder_check_if_value_bad(int8_t encoder_id){

    double time_delta_seconds = (((double)esp_timer_get_time()/1000.0) - (double) optical_encoders[encoder_id].m_last_time_microseconds/1000.0)/1000.0;
    //printf(" ((%lld/1000.0) - %lld/1000.0)/1000.0 = %9.5f ", esp_timer_get_time(), optical_encoders[encoder_id].m_last_time_microseconds, time_delta_seconds);
    return optical_encoders[encoder_id].m_value_drop_threshold_seconds < time_delta_seconds;
}