#include "./esp_01.h"

bool first_transmission = true;
static const int RX_BUF_SIZE = 1024;

bool sendAT(uint uart, char *command , char *ack, uint timeout_ms){
    // put the closing tags to the command... pain in the ass
    char command_result[strlen(command) + 2];
    memset(command_result, 0, (strlen(command) + 2)*sizeof(char));
    strcat(command_result, command);
    strcat(command_result, "\r\n");

    uart_write_bytes(uart, command_result, strlen(command_result));

    char *error = "ERROR";
    uint e = 0;
    uint error_length = strlen(error);

    uint ack_length = strlen(ack);
    uint u = 0;

    uart_wait_tx_done(uart, timeout_ms / portTICK_RATE_MS);

    // Flawed method
    // char* data = (char*) malloc(RX_BUF_SIZE);
    // uint result = uart_read_bytes(uart, data, RX_BUF_SIZE, timeout_ms / portTICK_RATE_MS);

    // if(result){
    //     for(uint i = 0; i < RX_BUF_SIZE+1; i++){
    //         printf("%c", data[i]);

    //         // look for ack
    //         if(data[i] == ack[u]){
    //             u++;
    //             if(u == ack_length){
    //                 printf("\n");
    //                 uart_flush(uart);
    //                 free(data);
    //                 return true;
    //             }
    //         }else{
    //             u = 0;
    //         }

    //         // look for error
    //         if(data[i] == error[e]){
    //             e++;
    //             if(e == error_length){
    //                 printf("\n");
    //                 uart_flush(uart);
    //                 free(data);
    //                 return false;
    //             }
    //         }else{
    //             e = 0;
    //         }
    //     }
    // }
    
    for(uint i = 0; i < 1024; i++){
            char* data = (char*) malloc(1+1);
            uint result = uart_read_bytes(uart, data, 1, timeout_ms / portTICK_RATE_MS);
            if(result == 0){
                break;
            }
            printf("%c", data[0]);

            // Check if ack reached
            if(data[0] == ack[u]){
                u++;
                if(u == ack_length){
                    printf("\n");
                    uart_flush(uart);
                    free(data);
                    return true;
                }
            }else{
                u = 0;
            }

            if(data[0] == error[e]){
                e++;
                if(e == error_length){
                    printf("\n");
                    uart_flush(uart);
                    free(data);
                    return false;
                }
            }else{
                e = 0;
            }
            free(data);
    }
    return false;
}

bool sendAT_no_read(uint uart, char *command){
    // put the closing tags to the command... pain in the ass
    char command_result[strlen(command) + 2];
    memset(command_result, 0, (strlen(command) + 2)*sizeof(char));
    strcat(command_result, command);
    strcat(command_result, "\r\n");

    uart_write_bytes(uart, command_result, strlen(command_result));
    return true;
}

bool init_esp_01_client(uint uart, uint enable_pin){

    // enable the device 
    gpio_set_direction(enable_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(enable_pin, 1);
    
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    // We won't use a buffer for sending data.
    uart_driver_install(uart, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(uart, &uart_config);

    if(uart == 0){
        uart_set_pin(uart, GPIO_NUM_17, GPIO_NUM_16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }else if(uart == 1){
        uart_set_pin(uart, GPIO_NUM_17, GPIO_NUM_16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }else if(uart == 2){
        uart_set_pin(uart, GPIO_NUM_17, GPIO_NUM_16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }

    bool result1 = sendAT(uart, "AT+RST", "ready", 500);
    bool result2 = sendAT(uart, "AT+CWMODE=1", "OK", 500);
    bool result3 = sendAT(uart, "AT+CWLAP", "OK", 5000);

    return result1 && result2 && result3;
}

bool init_esp_01_server(uint uart, uint enable_pin, char *wifi_name, char *wifi_password){

    // enable the device 
    gpio_set_direction(enable_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(enable_pin, 1);
    
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    // We won't use a buffer for sending data.
    uart_driver_install(uart, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(uart, &uart_config);

    if(uart == 0){
        uart_set_pin(uart, GPIO_NUM_17, GPIO_NUM_16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }else if(uart == 1){
        uart_set_pin(uart, GPIO_NUM_17, GPIO_NUM_16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }else if(uart == 2){
        uart_set_pin(uart, GPIO_NUM_17, GPIO_NUM_16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }

    bool result1 = sendAT(uart, "AT+RST", "ready", 500);
    bool result2 = sendAT(uart, "AT+CWMODE=2", "OK", 500);
    bool result3 = sendAT(uart, "AT+CWSAP=\"ESP32_wifi\",\"1234567890\",5,3", "OK", 500);
    bool result4 = sendAT(uart, "AT+CWDHCP=0,1", "OK", 500); // enable dhcp
    bool result5 = sendAT(uart, "AT+CIPMUX=1", "OK", 500); // set multiple connections mode for server
    bool result6 = sendAT(uart, "AT+CIPSERVER=1,3500", "OK", 500); // init server with ip 3500
    bool result7 = sendAT(uart, "AT+CIFSR", "OK", 500); // check ip
    bool result8 = sendAT(uart, "AT+CIPAP=\"192.168.8.1\"", "OK", 500); // set the ip 
    bool result9 = sendAT(uart, "AT+CIFSR", "OK", 500);// check ip
    bool result10 = sendAT(uart, "AT+CWLIF", "OK", 500);// check ip

    return result1 && result2 && result3 && result4 && result5 && result6 && result7 && result8 && result9 && result10;
}

bool esp_01_connect_wifi(uint uart, char *wifi_name, char *wifi_password){
    uint connect_command_length = strlen(wifi_name) + strlen(wifi_password) + 14;
    char connect_command[connect_command_length];
    memset(connect_command, 0, connect_command_length * sizeof(char));

    strcat(connect_command, "AT+CWJAP=\"");
    strcat(connect_command, wifi_name);
    strcat(connect_command, "\",\"");
    strcat(connect_command, wifi_password);
    strcat(connect_command, "\"");

    // sendAT(uart1, "AT+CWJAP=\"Stofa70521\",\"bis56lage63\"\r\n", "OK");

    return  sendAT(uart, connect_command, "OK", 20000);
}

int numPlaces (int n) {
    if (n < 0) return 0;
    if (n < 10) return 1;
    if (n < 100) return 2;
    if (n < 1000) return 3;
    if (n < 10000) return 4;
    if (n < 100000) return 5;
    if (n < 1000000) return 6;
    if (n < 10000000) return 7;
    if (n < 100000000) return 8;
    if (n < 1000000000) return 9;
    /*      2147483647 is 2^31-1 - add more ifs as needed
       and adjust this final return as well. */
    return 10;
}

bool esp_01_send_http(uint uart, char *ADDRESS, char *PORT, char *command){

    // connect to the server
    // sendAT(uart1, "AT+CIPSTART=\"TCP\",\"192.168.87.178\",4000", "OK");

    uint connect_server_length = strlen(ADDRESS) + strlen(PORT) + strlen("AT+CIPSTART=\"TCP\",\""); // 21
    char connect_server[connect_server_length];
    memset(connect_server, 0, connect_server_length * sizeof(char));

    strcat(connect_server, "AT+CIPSTART=\"TCP\",\"");
    strcat(connect_server, ADDRESS);
    strcat(connect_server, "\",");
    strcat(connect_server, PORT);
 
    sendAT(uart, connect_server, "OK", 2000);

    // send how many bytes the message will be
    // sendAT(uart1, "AT+CIPSEND=45", "OK");
    // the plus 2 is for the /r/n that gets added to every message that is very important
    uint command_length = strlen(command) + 2;                      // length is: 10 + 2
    char number[numPlaces(command_length)];                         // the lenghth of 12 is 2
    memset(number, 0, numPlaces(command_length) * sizeof(char));    // allocate the memory for it
    sprintf(number, "%d", command_length);                          // then convert the number into char

    // get characters for the whole command
    uint bytes_length = strlen("AT+CIPSEND=") + numPlaces(command_length);
    char bytes_command[bytes_length];
    memset(bytes_command, 0, bytes_length * sizeof(char));

    strcat(bytes_command, "AT+CIPSEND=");
    strcat(bytes_command, number);
    sendAT(uart, bytes_command, "OK", 2000);

    // send the actual command
    // sendAT(uart, "GET /users HTTP/1.1\r\nHost: 192.168.87.178\r\n", "CLOSED");

    sendAT(uart, command, "CLOSED", 10000);

    return true;
}

bool esp_01_server_get_connections(uint uart){
    sendAT(uart, "AT+CWLIF", "OK", 500);// check ip
    return true;
}

bool esp_01_server_get_connection_data(uint uart){
    sendAT(uart, "+IPD", "OK", 10000);// check ip
    // sendAT(uart, "+IPD,1,10:", "OK", 500);// check ip
    return true;
}

uint esp_01_IPD(uint uart, char *ack, uint timeout_ms){

    char *error = "ERROR";
    uint e = 0;
    uint error_length = strlen(error);

    uint u = 0;
    uint ack_length = strlen(ack);
    uint iterations = 0;
    for(uint i = 0; i < 1024; i++){
        char* data = (char*) malloc(1+1);
        uint result = uart_read_bytes(uart, data, 1, timeout_ms / portTICK_RATE_MS);
        if(result == 0){
            break;
        }
        printf("%c", data[0]);
        iterations = iterations + 1;
        // Check if ack reached
        if(data[0] == ack[u]){
            u++;
            if(u == ack_length){
                printf("\n");
                uart_flush(uart);
                free(data);
                break;
            }
        }else{
            u = 0;
        }

        if(data[0] == error[e]){
            e++;
            if(e == error_length){
                printf("\n");
                uart_flush(uart);
                free(data);
                break;
            }
        }else{
            e = 0;
        }
        free(data);
    }
    return iterations;
}

bool esp_01_server_OK(uint uart, uint connection_id){
    
    // HTTP/1.1 200 OK\r\n

    // Content-Type: text/html\r\n

    // Connection: close\r\n

    // \r\n
    
    // Get the lenght of the command as a number and turn that number into string.
    char *command = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n";

    uint command_length = strlen(command); // length is: 10 + 2
    char number[numPlaces(command_length)];                         // the lenghth of 12 is 2
    memset(number, 0, numPlaces(command_length) * sizeof(char));    // allocate the memory for it
    sprintf(number, "%d", command_length);                          // then convert the number into char

    // get characters for the whole command
    uint bytes_length = strlen("AT+CIPSEND=0,") + numPlaces(command_length);
    char bytes_command[bytes_length];
    memset(bytes_command, 0, bytes_length * sizeof(char));

    strcat(bytes_command, "AT+CIPSEND=0,");
    strcat(bytes_command, number);
    sendAT(uart, bytes_command, "OK", 2000);

    // send the actual command

    sendAT(uart, command, "OK", 20000);

    // sendAT_no_read(uart, command);

    sendAT(uart, "AT+CIPCLOSE=0", "OK", 2000);

    return true;
}