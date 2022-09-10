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

/**
 * Initialize the esp01 as a client 
 * 
 * uart - which uart you are using 0,1,2...
 * enable_pin - which gpio pin should be used as enable for the esp01
 * 
 */
bool init_esp_01_client(uint uart, uint enable_pin);
/**
 * Initialize the esp01 as a network and a server with a specific port 
 * 
 * uart - which uart you are using 0,1,2...
 * enable_pin - which gpio pin should be used as enable for the esp01
 * 
 */
bool init_esp_01_server(uint uart, uint enable_pin, char *wifi_name, char *wifi_password, char *server_port, char* server_ip, bool logging, bool install_driver);
bool esp_01_client_connect_wifi(uint uart, char *wifi_name, char *wifi_password);
bool esp_01_client_send_http(uint uart, char *ADDRESS, char *PORT, char *command);
uint esp_01_server_IPD(uint uart, char *ack, uint timeout_ms, char* buffer, bool logging);
bool esp_01_server_OK(uint uart, uint connection_id);
char* esp_01_trim_response(char* buffer, uint buffer_size, uint *connection_id, uint *request_size);