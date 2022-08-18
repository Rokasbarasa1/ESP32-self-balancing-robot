#pragma once
#include <stdio.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/uart.h>
#include <string.h>

// Instructions:
// * Connect rx of pico to tx of the esp 01
// * Connect tx of pico to rx of the esp 01
// * Connect the enable pin that you select to the enable pin of the pico

bool sendAT(uint uart, char *command , char *ack, uint timeout_ms);
bool init_esp_01_client(uint uart, uint enable_pin);
bool init_esp_01_server(uint uart, uint enable_pin, char *wifi_name, char *wifi_password);
bool esp_01_connect_wifi(uint uart, char *wifi_name, char *wifi_password);
bool esp_01_send_http(uint uart, char *ADDRESS, char *PORT, char *command);
bool esp_01_server_get_connections(uint uart);
bool esp_01_server_get_connection_data(uint uart);
bool esp_01_IPD(uint uart, char *ack, uint timeout_ms);
