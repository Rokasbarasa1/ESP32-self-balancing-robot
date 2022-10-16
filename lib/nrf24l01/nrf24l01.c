#include "./nrf24l01.h"



#define PIN_NUM_MISO 1
#define PIN_NUM_MOSI 1
#define PIN_NUM_CLK 1

volatile spi_host_device_t chosen_spi_port = SPI1_HOST;
volatile spi_device_handle_t device_handle = NULL;
volatile uint8_t cs_pin = 0;
volatile uint8_t csn_pin = 0;

void set_cs_pin_high(){
    gpio_set_direction(csn_pin, 1);
}

void set_cs_pin_low(){
    gpio_set_direction(csn_pin, 0);
}

void init_nrf24l01(spi_host_device_t spi_port, uint pin_csn_temp, uint8_t pin_cs_temp){
    // TRUST THAT THE SPI IS INITIALIZED
    cs_pin = pin_cs_temp;
    csn_pin = pin_csn_temp;
    chosen_spi_port = spi_port;
    // Make profile for the slave

    // Deselect the device

    // spi_device_interface_config_t device_config = {
    //     .clock_speed_hz = 4000000,  // 125Khz
    //     .mode = 3,                  //CPOL and CPHA high
    //     .spics_io_num = -1,         // CS Pin
    //     .queue_size = 1,
    //     .command_bits= 0,
    //     .address_bits = 0,
    //     .flags = SPI_DEVICE_HALFDUPLEX,
    //     .pre_cb = NULL,
    //     .post_cb = NULL,
    // };

    set_cs_pin_high();

    // spi_device_interface_config_t device_config = {
    //     .command_bits = 2,
    //     .address_bits = 8,
    //     .clock_speed_hz = 2000000,  // 125Khz
    //     // .input_delay_ns = 10,
    //     // .duty_cycle_pos = 128,
    //     .mode = 0,                  //CPOL and CPHA high
    //     .spics_io_num = csn_pin,         // CS Pin
    //     .queue_size = 1,
    //     .flags = SPI_DEVICE_HALFDUPLEX,
    //     .pre_cb = NULL,
    //     .post_cb = NULL,
    // };

    // handle for the slave that will be used in this driver
    printf("Setup slave\n");
    
    // init_spi_device(spi_port, &device_config, &device_handle);
    // ESP_ERROR_CHECK(spi_bus_add_device(spi_port, &device_config, &device_handle));

    // spi_write(device_handle, 0x2D, 8);
    // set_cs_pin_low();
    // spi_write(device_handle, 0x1E, 0b00000000);
    // set_cs_pin_high();

    // uint8_t data1[1] = {0};
    // set_cs_pin_low();
    // spi_read(device_handle, 0x1E, 1, data1);
    // set_cs_pin_high();
    // printf("%d\n", data1[0]);

    uint8_t data[1] = {0};
    // set_cs_pin_low();
    readRegister(0x00, 1, data);
    // spi_read(device_handle, 0x00, 1, data);
    // set_cs_pin_high();
    printf("%d\n", data[0]);

    // if(data1[0] == data[0]){
    //     printf("They are equal");
    // }
}

void nrf24_mode_transmitter(){
    gpio_set_direction(cs_pin, GPIO_MODE_OUTPUT);
}

void nrf24_mode_receiver(){
    gpio_set_direction(cs_pin, GPIO_MODE_INPUT);
}

void nrf24_transmit(){
}

void nrf24_read(){
}

void readRegister(char registerAddress, int numBytes, unsigned char * values){
    //Since we're performing a read operation, the most significant bit of the register address should be set.
    char address = 0x80 | registerAddress;
    //If we're doing a multi-byte read, bit 6 needs to be set as well.
    if(numBytes > 1)address = address | 0x40;
    
    //Set the Chip select pin low to start an SPI packet.
    set_cs_pin_low();
    //Transfer the starting register address that needs to be read.
    SPI.transfer(address);
    //Continue to read registers until we've read the number specified, storing the results to the input buffer.
    for(int i=0; i<numBytes; i++){
        values[i] = SPI.transfer(0x00);
    }
    //Set the Chips Select pin high to end the SPI packet.
    set_cs_pin_high();
}