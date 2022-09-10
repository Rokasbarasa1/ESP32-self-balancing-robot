#include "./esp_01.h"

bool first_transmission = true;
static const int RX_BUF_SIZE = 1024;

bool sendAT(uint uart, char *command , char *ack, uint timeout_ms, bool logging){
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
    
    for(uint i = 0; i < 1024; i++){
            char* data = (char*) malloc(1+1);
            uint result = uart_read_bytes(uart, data, 1, timeout_ms / portTICK_RATE_MS);
            if(result == 0){
                break;
            }
            if(logging){
                printf("%c", data[0]);
            }

            // Check if ack reached
            if(data[0] == ack[u]){
                u++;
                if(u == ack_length){
                    if(logging){
                        printf("\n");
                    }
                    
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
                    if(logging){
                        printf("\n");
                    }
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

    bool result1 = sendAT(uart, "AT+RST", "ready", 500, true);
    bool result2 = sendAT(uart, "AT+CWMODE=1", "OK", 500, true);
    bool result3 = sendAT(uart, "AT+CWLAP", "OK", 5000, true);

    return result1 && result2 && result3;
}

bool init_esp_01_server(uint uart, uint enable_pin, char *wifi_name, char *wifi_password, char *server_port, char* server_ip, bool logging, bool install_driver){

    // enable the device 
    if(install_driver){
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
    }
    
    bool result11 = sendAT(uart, "AT+RELOAD", "ready", 20000, logging);

    bool result1 = sendAT(uart, "AT+RST", "ready", 500, logging);
    bool result2 = sendAT(uart, "AT+CWMODE=2", "OK", 500, logging);

    uint string1_length = strlen("AT+CWSAP=\"\",\"\",5,3") + strlen(wifi_name) + strlen(wifi_password);
    char string1[string1_length];
    memset(string1, 0, string1_length * sizeof(char));
    strcat(string1, "AT+CWSAP=\"");
    strcat(string1, wifi_name);
    strcat(string1, "\",\"");
    strcat(string1, wifi_password);
    strcat(string1, "\",5,3");
    bool result3 = sendAT(uart, string1, "OK", 500, logging);
    // bool result3 = sendAT(uart, "AT+CWSAP=\"ESP32_wifi\",\"1234567890\",5,3", "OK", 500, true);
    
    bool result4 = sendAT(uart, "AT+CWDHCP=0,1", "OK", 500, logging); // enable dhcp
    bool result5 = sendAT(uart, "AT+CIPMUX=1", "OK", 500, logging); // set multiple connections mode for server
        
    uint string2_length = strlen("AT+CIPSERVER=1,") + strlen(server_port);
    char string2[string2_length];
    memset(string2, 0, string2_length * sizeof(char));
    strcat(string2, "AT+CIPSERVER=1,");
    strcat(string2, server_port);
    bool result6 = sendAT(uart, string2, "OK", 500, logging); // init server with port 3500
    // bool result6 = sendAT(uart, "AT+CIPSERVER=1,3500", "OK", 500, true); // init server with ip 3500

    bool result7 = sendAT(uart, "AT+CIFSR", "OK", 500, logging); // check ip

    uint string3_length = strlen("AT+CIPAP=\"\"") + strlen(server_ip);
    char string3[string3_length];
    memset(string3, 0, string3_length * sizeof(char));
    strcat(string3, "AT+CIPAP=\"");
    strcat(string3, server_ip);
    strcat(string3, "\"");
    bool result8 = sendAT(uart, string3, "OK", 500, logging); // set the ip 
    // bool result8 = sendAT(uart, "AT+CIPAP=\"192.168.8.1\"", "OK", 500, true); // set the ip 

    bool result9 = sendAT(uart, "AT+CIFSR", "OK", 500, logging);// check ip
    bool result10 = sendAT(uart, "AT+CWLIF", "OK", 500, logging);// check ip

    return result1 && result2 && result3 && result4 && result5 && result6 && result7 && result8 && result9 && result10 && result11;
}

bool esp_01_client_connect_wifi(uint uart, char *wifi_name, char *wifi_password){
    uint connect_command_length = strlen(wifi_name) + strlen(wifi_password) + 14;
    char connect_command[connect_command_length];
    memset(connect_command, 0, connect_command_length * sizeof(char));

    strcat(connect_command, "AT+CWJAP=\"");
    strcat(connect_command, wifi_name);
    strcat(connect_command, "\",\"");
    strcat(connect_command, wifi_password);
    strcat(connect_command, "\"");

    // sendAT(uart1, "AT+CWJAP=\"Stofa70521\",\"bis56lage63\"\r\n", "OK");

    return  sendAT(uart, connect_command, "OK", 20000, true);
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

bool esp_01_client_send_http(uint uart, char *ADDRESS, char *PORT, char *command){

    // connect to the server
    // sendAT(uart1, "AT+CIPSTART=\"TCP\",\"192.168.87.178\",4000", "OK");

    uint connect_server_length = strlen(ADDRESS) + strlen(PORT) + strlen("AT+CIPSTART=\"TCP\",\""); // 21
    char connect_server[connect_server_length];
    memset(connect_server, 0, connect_server_length * sizeof(char));

    strcat(connect_server, "AT+CIPSTART=\"TCP\",\"");
    strcat(connect_server, ADDRESS);
    strcat(connect_server, "\",");
    strcat(connect_server, PORT);
 
    sendAT(uart, connect_server, "OK", 2000, true);

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
    sendAT(uart, bytes_command, "OK", 2000, true);

    // send the actual command
    // sendAT(uart, "GET /users HTTP/1.1\r\nHost: 192.168.87.178\r\n", "CLOSED");

    sendAT(uart, command, "CLOSED", 10000, true);

    return true;
}

uint esp_01_server_IPD(uint uart, char *ack, uint timeout_ms, char* buffer, bool logging){

    char *error = "ERROR";
    uint e = 0;
    uint error_length = strlen(error);

    uint u = 0;
    uint ack_length = strlen(ack);
    uint iterations = 0;

    char *reset = "wdt reset";
    uint r = 0;
    uint reset_length = strlen(reset);

    for(uint i = 0; i < 1023; i++){
        char* data = (char*) malloc(1+1);
        uint result = uart_read_bytes(uart, data, 1, timeout_ms / portTICK_RATE_MS);
        if(result == 0){
            break;
        }
        if(logging){
            printf("%c", data[0]);
        }
        buffer[i] = data[0];
        iterations = iterations + 1;

        // Check if ack reached
        if(data[0] == ack[u]){
            u++;
            if(u == ack_length){
                if(logging){
                    printf("\n");
                }
                buffer[iterations] = '\0';
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
                if(logging){
                    printf("\n");
                }

                iterations = 0;
                uart_flush(uart);
                free(data);
                break;
            }
        }else{
            e = 0;
        }

        if(data[0] == reset[r]){
            r++;
            if(r == reset_length){
                printf("\nESP01 crashed: wdt reset\n");
                
                iterations = 9999;
                uart_flush(uart);
                free(data);
                break;
            }
        }else{
            r = 0;
        }
        free(data);
    }
    // buffer[iterations] = '\0';
    
    return iterations;
}

bool esp_01_server_OK(uint uart, uint connection_id){
    
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
    sendAT(uart, bytes_command, "OK", 2000, false);

    // send the actual command

    sendAT(uart, command, "OK", 20000, false);

    sendAT(uart, "AT+CIPCLOSE=0", "OK", 2000, false);

    return true;
}

char* esp_01_trim_response(char* buffer, uint buffer_size, uint *connection_id, uint *request_size){

    // Strategy find pieces of text that i know are there and then trim everything based on that

    // find connection id
    char *text1 = "+IPD,";
    uint e1 = 0;
    uint text_length1 = strlen(text1);
    uint conneciton_id_index = 0;
    uint conneciton_id_end_index = 0;

    for(uint i = 0; i < buffer_size; i++){

        if(buffer[i] == text1[e1]){
            e1++;
            if(e1 == text_length1){
                conneciton_id_index = i+1;
                // found the character string
                break;
            }
        }else{
            e1 = 0;
        }
    }

    // find end of connection id
    for(uint i = conneciton_id_index; i < buffer_size; i++){
        if(buffer[i] == ','){
            conneciton_id_end_index = i-1;
        }
    }

    // find the request URL
    char *text2 = "GET ";
    uint e2 = 0;
    uint text_length2 = strlen(text2);
    uint request_index = 0;
    uint request_end_index = 0;

    for(uint i = conneciton_id_end_index + 2; i < buffer_size; i++){

        if(buffer[i] == text2[e2]){
            e2++;
            if(e2 == text_length2){
                request_index = i+1;
                break;
            }
        }else{
            e2 = 0;
        }
    }

    // find the end of the request url
    char *text3 = " HTTP/1.1";
    uint e3 = 0;
    uint text_length3 = strlen(text3);

    for(uint i = conneciton_id_end_index + 2; i < buffer_size; i++){

        if(buffer[i] == text3[e3]){
            e3++;
            if(e3 == text_length3){
                request_end_index = i-9;
                break;
            }
        }else{
            e3 = 0;
        }
    }

    // cut out the substring of the id
    uint id_length = conneciton_id_end_index - conneciton_id_index+1;
    char id_substring[id_length+1];
    strncpy(id_substring, &buffer[conneciton_id_index], id_length);
    id_substring[id_length] = '\0';

    // Cut out the substring of the request
    uint request_length = request_end_index - request_index+1;
    char request_substring[request_length+1];
    strncpy(request_substring, &buffer[request_index], request_length);
    request_substring[request_length] = '\0';

    // convert the id substring to int and set the reference to that value
    *connection_id = atoi(id_substring);

    *request_size = request_length;
    // convert the substring of the request to allocated memory and then return it.
    char *return_string = malloc(request_length+1);

    for(uint8_t i = 0; i < request_length; i++){
        return_string[i] = request_substring[i];
    }
    return_string[request_length] = '\0';
    return return_string;
}