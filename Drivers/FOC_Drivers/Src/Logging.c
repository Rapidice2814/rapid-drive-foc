#include "Logging.h"
#include <stdlib.h>
#include <string.h>

static UART_HandleTypeDef *huart_log;

static uint8_t buffer_selector = 0;
static char format_buffer[300] = {0};
static int argument_buffer[30] = {0};

static uint32_t format_write_index = 0;
static uint32_t format_read_index = 0;
static uint32_t arg_write_index = 0;
static uint32_t arg_read_index = 0;

static char tx_buffer[2][300] = {{0}};
static uint32_t tx_packet_length = 0;

static volatile uint8_t uart_tx_free_flag = 1;
int cnt = 0;

void Log_Setup(UART_HandleTypeDef *huart){
    huart_log = huart;
    format_write_index = 0;
    format_read_index = 0;
    arg_write_index = 0;
    arg_read_index = 0;
    memset(format_buffer, 0, sizeof(format_buffer));
    memset(argument_buffer, 0, sizeof(argument_buffer));
    memset(tx_buffer, 0, sizeof(tx_buffer));
    tx_packet_length = 0;
    buffer_selector = 0;
    uart_tx_free_flag = 1; // Set the flag to indicate UART is ready for transmission
}

void Log_Queue(const char* format, ...){
    
    uint32_t format_size = strlen(format);
    uint32_t format_buffer_size = sizeof(format_buffer) / sizeof(format_buffer[0]);
    uint32_t format_buffer_space_available = (format_read_index - format_write_index + format_buffer_size- 1) % format_buffer_size;
    if (format_size > format_buffer_space_available) return; // Does not fit in the format buffer

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
    if (arg_count > arg_buffer_space_available) return; // Does not fit in the argument buffer


    if(format_write_index + format_size >= format_buffer_size-1) {
        uint32_t first_part_size = format_buffer_size - format_write_index;
        uint32_t second_part_size = format_size - first_part_size;

        memcpy(&format_buffer[format_write_index], &format[0], first_part_size);
        memcpy(&format_buffer[0], &format[first_part_size], second_part_size); // Wrap around if needed
        format_buffer[second_part_size] = '\0'; // Null-terminate the buffer

    } else{
        memcpy(&format_buffer[format_write_index], format, format_size);
    }
    format_write_index = (format_write_index + format_size) % format_buffer_size;


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

void Log_Loop(){

    if(uart_tx_free_flag && tx_packet_length > 0){
        cnt++;
        uart_tx_free_flag = 0;
        HAL_UART_Transmit_DMA(huart_log, (uint8_t*)tx_buffer[buffer_selector], tx_packet_length);
        tx_packet_length = 0; // Reset for next transmission
        buffer_selector ^= 1; // Switch buffer
    }

    uint32_t format_buffer_size = sizeof(format_buffer) / sizeof(format_buffer[0]);

    if (format_write_index == format_read_index) return; // No new data to process

    for(;format_read_index != format_write_index;format_read_index = (format_read_index + 1) % format_buffer_size){
        // if(format_buffer[format_read_index] == '\0') break;
    
        if(format_buffer[format_read_index] == '%' && format_buffer[(format_read_index + 1) % format_buffer_size] == 'd'){
                tx_packet_length += snprintf(tx_buffer[buffer_selector] + tx_packet_length, sizeof(tx_buffer[0]) - tx_packet_length - 1, "%d", argument_buffer[arg_read_index]);
                arg_read_index++;

                format_read_index = (format_read_index + 2) % format_buffer_size; // Skip the '%d'
                break;

        } else{

            if(tx_packet_length < sizeof(tx_buffer[0])){
                tx_buffer[buffer_selector][tx_packet_length] = format_buffer[format_read_index];
                tx_packet_length++;
            } else{
                tx_buffer[buffer_selector][tx_packet_length-2] = '!'; // Indicate buffer overflow
                tx_buffer[buffer_selector][tx_packet_length-1] = '\0';
                break;
            }

        }
    }

    __NOP();
}



void Log_TxCompleteCallback(UART_HandleTypeDef *huart){
    if(huart->Instance == huart_log->Instance){
        uart_tx_free_flag = 1;
    }
}
