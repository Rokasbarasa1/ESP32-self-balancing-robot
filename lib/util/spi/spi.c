#include "./spi.h"


void init_spi3(){
    // MOSI 23
    // MISO 19
    // CLK 18
    
    SPI.begin(18, 19, 23);
    SPI.setDataMode(SPI_MODE3);

    // spi_bus_config_t buscfg = {
    //     .miso_io_num = 19,
    //     .mosi_io_num = 23,
    //     .sclk_io_num = 18,
    //     .quadwp_io_num = -1,
    //     .quadhd_io_num = -1,
    //     .max_transfer_sz = 32,
    // };
    // printf("Setup spi bus 3\n");
    // ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, 0));
    vTaskDelay(100 / portTICK_RATE_MS);
}

void init_spi2(){
    // MOSI 13
    // MISO 12
    // CLK 14
    SPI.begin(14, 12, 13);
    SPI.setDataMode(SPI_MODE3);
    // spi_bus_config_t buscfg = {
    //     .miso_io_num = 12,
    //     .mosi_io_num = 13,
    //     .sclk_io_num = 14,
    //     .quadwp_io_num = -1,
    //     .quadhd_io_num = -1,
    //     .max_transfer_sz = 32,
    // };

    // printf("Setup spi bus 2\n");
    // ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, 0));
    vTaskDelay(100 / portTICK_RATE_MS);
}

void init_spi_device(spi_host_device_t host_id, const spi_device_interface_config_t *dev_config, spi_device_handle_t *handle){
    ESP_ERROR_CHECK(spi_bus_add_device(host_id, dev_config, handle));
}

void spi_read(spi_device_handle_t device_handle, uint8_t address, uint8_t read_length, uint8_t* buffer){

    
    // spi_transaction_t t;
	// memset(&t, 0, sizeof(t));
    // t.cmd       = address;
    // t.length    = 8 * 1; // write length
    // t.rxlength  = 8 * read_length; // read length
    // t.rx_buffer = buffer;

    // spi_transaction_t t = {
    //     .cmd        = 0b11,
    //     .addr       = address,
    //     .length     = 0, // write length
    //     .tx_buffer  = NULL,
    //     .flags      = SPI_TRANS_USE_RXDATA,
    //     .rxlength   = 8 * read_length, // read length
    // };

    // uint8_t read_address = 0b11000000 | address;
    // spi_transaction_t t = {
    //     .length     = 8 * 1, // write length
    //     .tx_buffer  = &read_address,
    //     .flags      = SPI_TRANS_USE_RXDATA,
    //     .rxlength   = 8 * read_length, // read length
    //     // .rx_buffer  = buffer,
    // };

    // printf("Read slave\n");
    // ESP_ERROR_CHECK(spi_device_polling_transmit(device_handle, &t));
    
    // // printf("DAta: %d\n", t.rx_data[0]);
    // // *buffer = t.rx_data[0];
    // for(uint8_t i = 0; i < read_length; i++){
    //     buffer[i] = t.rx_data[i];
    // }
}

void spi_write(spi_device_handle_t device_handle, uint8_t address, uint8_t value){
    // uint8_t data_tx[1] = { value };
    // spi_transaction_t t;
	// memset(&t, 0, sizeof(t));
    // t.cmd        = address;
    // t.length     = 8 * 2;
    // t.flags      = SPI_TRANS_USE_TXDATA;
    // t.tx_data    = data_tx;

    // spi_transaction_t t = {
    //     .cmd        = 0b01,
    //     .addr       = address,
    //     .length     = 8 * 1,
    //     .tx_buffer  = &value,
    // };

    
    uint8_t write_address = 0b01000000 | address;

    uint8_t buffer[2] = {write_address, value};
    spi_transaction_t t = {
        .length     = 8 * 2, // write length
        .tx_buffer  = buffer,
    };

    printf("Write slave\n");
    ESP_ERROR_CHECK(spi_device_polling_transmit(device_handle, &t));
}
