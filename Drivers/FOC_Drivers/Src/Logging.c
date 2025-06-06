#include "Logging.h"
#include "main.h"
#include <stdlib.h>
#include <string.h>

void testqueue(const char* format, ...);
void testloop();

extern UART_HandleTypeDef huart3;

void test(){
    int arr[10] = {543, -1531, 2456, 376, -4678, 56879, -6345, 7234, -837, 9653};
    // testqueue("Hello: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n", arr[0], arr[1], arr[2], arr[3], arr[4], arr[5], arr[6], arr[7], arr[8], arr[9]);
    testqueue("Hello: ");
    testqueue("%d, ", arr[0]);
    testqueue("%d, ", arr[1]);
    testqueue("%d, ", arr[2]);
    testqueue("%d, ", arr[3]);
    testqueue("%d, ", arr[4]);
    testqueue("%d, ", arr[5]);
    testqueue("%d, ", arr[6]);
    testqueue("%d, ", arr[7]);
    testqueue("%d, ", arr[8]);
    testqueue("%d\n", arr[9]);
    __NOP();
}


static uint8_t buffer_selector = 0;
static char format_buffer[200] = {0};
static int argument_buffer[20] = {0};

static uint32_t format_max_index = 0;
static uint32_t format_max_arg = 0;

void testqueue(const char* format, ...){
    
    uint32_t format_size = strlen(format);
    uint32_t format_buffer_size = sizeof(format_buffer) / sizeof(format_buffer[0]);
    if (format_max_index + format_size >= format_buffer_size) return; // Does not fit in the format buffer

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
    if (format_max_arg + arg_count > arg_buffer_size) return; // Does not fit in the argument buffer


    strcpy(&format_buffer[format_max_index], format);
    format_max_index += strlen(format);

    va_list args;
    va_start(args, format);
    for (uint32_t i = format_max_arg; i < (format_max_arg + arg_count); i++) {
        argument_buffer[i] = va_arg(args, int);
    }
    va_end(args);
    
    format_max_arg += arg_count;
    __NOP();

}


static char tx_buffer[2][300] = {{0}};
static uint32_t tx_packet_length = 0;
static uint32_t format_processed_index = 0;
static uint32_t arg_processed_index = 0;

static volatile uint8_t uart_tx_free_flag = 1;
int cnt = 0;

void testloop(){

    if(uart_tx_free_flag && tx_packet_length > 0){
        cnt++;
        uart_tx_free_flag = 0;
        HAL_UART_Transmit_DMA(&huart3, (uint8_t*)tx_buffer[buffer_selector], tx_packet_length);
        tx_packet_length = 0; // Reset for next transmission
        buffer_selector ^= 1; // Switch buffer
    }

    if (format_max_index == format_processed_index) return; // No new data to process

    for (; format_processed_index < format_max_index; format_processed_index++) {
        if(format_buffer[format_processed_index] == '\0') break;
    
        if(format_buffer[format_processed_index] == '%' && format_buffer[format_processed_index + 1] == 'd'){
                tx_packet_length += snprintf(tx_buffer[buffer_selector] + tx_packet_length, sizeof(tx_buffer[0]) - tx_packet_length - 1, "%d", argument_buffer[arg_processed_index]);
                arg_processed_index++;
                format_processed_index++; // Skip 'd'

                format_processed_index++; break;
        } else{

            if(tx_packet_length < sizeof(tx_buffer[0])){
                tx_buffer[buffer_selector][tx_packet_length] = format_buffer[format_processed_index];
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



void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
    if(huart->Instance == huart3.Instance){
        uart_tx_free_flag = 1;
    }
}
