#include "Logging.h"
#include <stdlib.h>
#include <string.h>

static UART_HandleTypeDef *huart_log;

/* Transmit buffers*/
static char format_buffer[400] = {0}; //the size of the buffer determines how many characters can be queued
static int argument_buffer[30] = {0}; //the size of the buffer determines how many arguments can be queued
static char uart_tx_buffer[2][500] = {{0}}; //the size of the buffer determines how many characters can be sent at once

static uint16_t format_write_index = 0;
static uint16_t format_read_index = 0;
static uint16_t arg_write_index = 0;
static uint16_t arg_read_index = 0;

static uint8_t buffer_selector = 0;
static uint16_t tx_packet_length = 0;
static volatile uint8_t uart_tx_free_flag = 1; // Set to 1 when the UART is ready for new transmission

/* Receive buffer */
static char uart_rx_buffer[100] = {0};
static volatile uint16_t rx_packet_length = 0;
static volatile uint8_t uart_rx_flag = 0; //set to 1 when a new packet is received


/*Functions*/
static void Log_Transmit();
static void Log_Receive();

/**
 * @brief Initializes the logging system with the specified UART handle.
 * @param huart Pointer to the UART handle used for logging.
 * @note This function sets up the UART for DMA reception and initializes internal buffers.
 */
void Log_Setup(UART_HandleTypeDef *huart){
    huart_log = huart;
    format_write_index = 0;
    format_read_index = 0;
    arg_write_index = 0;
    arg_read_index = 0;
    tx_packet_length = 0;
    buffer_selector = 0;
    uart_tx_free_flag = 1; // Set the flag to indicate UART is ready for transmission

    HAL_UARTEx_ReceiveToIdle_DMA(huart_log, (uint8_t*)uart_rx_buffer, sizeof(uart_rx_buffer));
}

/**
 * @brief Queues a message to be printed to the UART.
 * @param format The format string for the log message. It uses the same format as printf.
 * @param ... The integer arguments to be included in the log message.
 * @note Only %d, and %% is supported. If the mesasge does not fit in the buffer, it will be skipped.
 */
void Log_printf(const char* format, ...){
    
    uint32_t format_size = strlen(format);
    uint32_t format_buffer_size = sizeof(format_buffer) / sizeof(format_buffer[0]);
    uint32_t format_buffer_space_available = (format_read_index - format_write_index + format_buffer_size- 1) % format_buffer_size;
    if (format_size > format_buffer_space_available){
        return;
    } 

    uint32_t arg_count = 0;
    const char *p = format;
    while (*p) {
        if (*p == '%' && *(p + 1) == 'd') {
            arg_count++;
            p++; // Skip 'd'
        }
        p++;
    }

    uint32_t arg_buffer_size = sizeof(argument_buffer) / sizeof(argument_buffer[0]);
    uint32_t arg_buffer_space_available = (arg_read_index - arg_write_index + arg_buffer_size-1) % arg_buffer_size;
    if (arg_count > arg_buffer_space_available){
        return;
    } 

    if(format_write_index + format_size > format_buffer_size-1) {
        uint32_t first_part_size = format_buffer_size - format_write_index;
        uint32_t second_part_size = format_size - first_part_size;

        memcpy(&format_buffer[format_write_index], &format[0], first_part_size);
        memcpy(&format_buffer[0], &format[first_part_size], second_part_size); // Wrap around if needed

    } else{
        memcpy(&format_buffer[format_write_index], format, format_size);
    }
    format_write_index = (format_write_index + format_size) % format_buffer_size;
    format_buffer[format_write_index] = '\0'; // Null-terminate the buffer


    va_list args;
    va_start(args, format);
    for (uint32_t i = 0; i < arg_count; i++) {
        uint32_t index = (arg_write_index + i) % arg_buffer_size;
        argument_buffer[index] = va_arg(args, int);
    }
    va_end(args);
    
    arg_write_index = (arg_write_index + arg_count) % arg_buffer_size;
    __NOP();
}

/**
 * @brief Main loop for the logging system. It handles both transmission and reception of log messages.
 * @note This function should be called periodically to ensure that log messages are processed. 
 *       One call only processes a part of the queue, so it should be called multiple times per message in order to sent the whole queue.
 */
void Log_Loop(){
    Log_Transmit();
    Log_Receive();
}

/* The Tx part of the loop*/
static void Log_Transmit(){
    if(uart_tx_free_flag && tx_packet_length > 0){
        uart_tx_free_flag = 0;
        HAL_UART_Transmit_DMA(huart_log, (uint8_t*)uart_tx_buffer[buffer_selector], tx_packet_length);
        tx_packet_length = 0; // Reset for next transmission
        buffer_selector ^= 1; // Switch buffer
    }


    uint32_t format_buffer_size = sizeof(format_buffer) / sizeof(format_buffer[0]);
    uint32_t arg_buffer_size = sizeof(argument_buffer) / sizeof(argument_buffer[0]);

    if (format_write_index == format_read_index) return; // No new data to process

    for(uint32_t loop_counter = 0;format_read_index != format_write_index;format_read_index = (format_read_index + 1) % format_buffer_size, loop_counter++){
    
        if(format_buffer[format_read_index] == '%' && format_buffer[(format_read_index + 1) % format_buffer_size] == 'd'){
            uint32_t uart_tx_buffer_space_available = sizeof(uart_tx_buffer[0]) - tx_packet_length; //this includes the null terminator
            uint32_t uart_tx_buffer_characters_needed = snprintf(uart_tx_buffer[buffer_selector] + tx_packet_length, uart_tx_buffer_space_available, "%d", argument_buffer[arg_read_index]); //this excludes the null terminator

            if(uart_tx_buffer_characters_needed >= uart_tx_buffer_space_available){
                tx_packet_length += uart_tx_buffer_space_available; //the number is truncated to fit the buffer
            }else{
                tx_packet_length += uart_tx_buffer_characters_needed;
            }

            arg_read_index = (arg_read_index + 1) % arg_buffer_size;

            format_read_index = (format_read_index + 2) % format_buffer_size; // Skip the '%d'
            break; //end loop after processing one argument
        } else if(format_buffer[format_read_index] == '%' && format_buffer[(format_read_index + 1) % format_buffer_size] == '%'){
            uint32_t uart_tx_buffer_space_available = sizeof(uart_tx_buffer[0]) - tx_packet_length; //this includes the null terminator
            if(uart_tx_buffer_space_available > 0){
                uart_tx_buffer[buffer_selector][tx_packet_length] = '%'; // Add the '%' character
                tx_packet_length++;
            } else{
                uart_tx_buffer[buffer_selector][tx_packet_length-1] = '!'; // Indicate buffer overflow
            }
            format_read_index = (format_read_index + 2) % format_buffer_size; // Skip the '%%'
            break; //end loop after processing one argument
        }else{
            if(loop_counter > 20){
                break; //end loop if we have processed too many characters without finding a '%d'
            }

            if(tx_packet_length < sizeof(uart_tx_buffer[0])){
                uart_tx_buffer[buffer_selector][tx_packet_length] = format_buffer[format_read_index];
                tx_packet_length++;
            } else{
                if(tx_packet_length > 300){
                    __NOP(); //error
                }
                uart_tx_buffer[buffer_selector][tx_packet_length-1] = '!'; // Indicate buffer overflow
                break;
            }

        }
    }
}

/* The Rx part of the loop*/
static void Log_Receive(){
    if(uart_rx_flag){
        Log_ProcessRxPacket(uart_rx_buffer, rx_packet_length);
        HAL_UARTEx_ReceiveToIdle_DMA(huart_log, (uint8_t*)uart_rx_buffer, sizeof(uart_rx_buffer));
        uart_rx_flag = 0;
    }
}

/**
 * @brief Processes a received packet.
 * @param packet The received packet.
 * @param Length The length of the received packet.
 * @note This function can be overwritten
 */
__weak void Log_ProcessRxPacket(const char* packet, uint16_t Length){
    //null terminate
    if(Length < sizeof(uart_rx_buffer)){
        uart_rx_buffer[Length] = '\0'; // Ensure null termination
    } else {
        uart_rx_buffer[sizeof(uart_rx_buffer) - 1] = '\0'; // Truncate if too long
    }
    Log_printf("Received packet: ");
    Log_printf(packet);
    Log_printf(" Length:%d\n", Length);
}







void Log_UART_TxCpltCallback(UART_HandleTypeDef *huart){
    if(huart->Instance == huart_log->Instance){
        uart_tx_free_flag = 1;
    }
}

void Log_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
    if(huart->Instance == huart_log->Instance){
        rx_packet_length = Size;
        uart_rx_flag = 1;
    }  
}
