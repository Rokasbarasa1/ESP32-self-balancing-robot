#include "./spi.h"

#define SPIBUS_READ     (0x80)  /*!< addr | SPIBUS_READ  */
#define SPIBUS_WRITE    (0x7F)  /*!< addr & SPIBUS_WRITE */

// I used the "esp32-SPIBus" library form natanaeljr as an example to implement spi.

void init_spi3(){
    // MOSI 23
    // MISO 19
    // CLK 18
    spi_bus_config_t buscfg = {
        .miso_io_num = 19,
        .mosi_io_num = 23,
        .sclk_io_num = 18,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };
    // printf("Setup spi bus 3\n");
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, 0));
    vTaskDelay(100 / portTICK_RATE_MS);
}

void init_spi2(){
    // MOSI 13
    // MISO 12
    // CLK 14

    spi_bus_config_t buscfg = {
        .miso_io_num = 12,
        .mosi_io_num = 13,
        .sclk_io_num = 14,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };

    // printf("Setup spi bus 2\n");
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, 0));
    vTaskDelay(100 / portTICK_RATE_MS);
}

void init_spi_device(spi_host_device_t host_id, const spi_device_interface_config_t *dev_config, spi_device_handle_t *handle){
    ESP_ERROR_CHECK(spi_bus_add_device(host_id, dev_config, handle));
}


void spi_write(spi_device_handle_t device_handle, uint8_t address, size_t length, const uint8_t *data){
    spi_transaction_t t = {
        .flags      = 0,
        .cmd        = 0,
        .addr       = address,
        .length     = 8 * length, // write length
        .rxlength   = 0,
        .user       = NULL,
        .tx_buffer  = data,
        .rx_buffer  = NULL,
    };
    // printf("Write slave\n");
    ESP_ERROR_CHECK(spi_device_polling_transmit(device_handle, &t));
}

void spi_read(spi_device_handle_t device_handle, uint8_t address, size_t length, uint8_t *data){
   spi_transaction_t t = {
        .flags      = 0,
        .cmd        = 0,
        .addr       = address,
        .length     = 8 * length, // write length
        .rxlength   = 8 * length,
        .user       = NULL,
        .tx_buffer  = NULL,
        .rx_buffer  = data,
    };
    // printf("Read slave\n");
    ESP_ERROR_CHECK(spi_device_polling_transmit(device_handle, &t));
}