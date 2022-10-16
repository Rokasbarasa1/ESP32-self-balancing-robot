#pragma once
#include <stdio.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/spi_master.h"
#include "../util/spi/spi.h"

// I have tried to setup spi with esp idf for a week straight
// 30 hours of re-flashing and getting no results with a billion completely  
// different configurations for it. IM DONE
#include <Arduino.h>
#include <SPI.h>

void init_nrf24l01(spi_host_device_t spi_port, uint pin_csn, uint8_t pin_cs);
void nrf24_mode_transmitter();
void nrf24_mode_receiver();
void nrf24_transmit();
void nrf24_read();