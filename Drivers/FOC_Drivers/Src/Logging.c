#include "Logging.h"
#include "main.h"
#include <stdlib.h>
#include <string.h>

int mini_snprintf(char* buffer, size_t size, const char* format, ...);
void int_to_str(int value, char* buffer);
uint8_t int_to_str2(int value, char* buffer);

void testqueue(const char* format, ...);
void testqueue2(const char* format, ...);

extern UART_HandleTypeDef huart3;
char buffer[200];

void test(){
    int arr[10] = {543, -1531, 2456, 376, -4678, 56879, -6345, 7234, -837, 9653};
    // mini_snprintf(buffer, sizeof(buffer), "Hello: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n", arr[0], arr[1], arr[2], arr[3], arr[4], arr[5], arr[6], arr[7], arr[8], arr[9]);
    // snprintf(buffer, sizeof(buffer), "Hello: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n", arr[0], arr[1], arr[2], arr[3], arr[4], arr[5], arr[6], arr[7], arr[8], arr[9]);
    // snprintf(buffer, sizeof(buffer), "Hello: ");
    // snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), "%d, ", arr[0]);
    // snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), "%d, ", arr[1]);
    // snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), "%d, ", arr[2]);
    // snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), "%d, ", arr[3]);
    // snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), "%d, ", arr[4]);
    // snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), "%d, ", arr[5]);
    // snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), "%d, ", arr[6]);
    // snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), "%d, ", arr[7]);
    // snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), "%d, ", arr[8]);
    // snprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), "%d\n", arr[9]);
    testqueue("Hello: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n", arr[0], arr[1], arr[2], arr[3], arr[4], arr[5], arr[6], arr[7], arr[8], arr[9]);
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

typedef struct {
    char *format;     // pointer to the format string
    int *args;        // pointer to an array of arguments
    int arg_count;
} LogEntryTypeDef;


void testqueue2(const char* format, ...){

    // Step 1: Allocate entry
    LogEntryTypeDef *entry = malloc(sizeof(LogEntryTypeDef));
    if (!entry) return; // Allocation failed

    // Step 2: Copy format string
    entry->format = strdup(format);
    if (!entry->format) { // Allocation failed
        free(entry);
        return;
    }

    // Step 3: Parse number of %d's in the format
    entry->arg_count = 0;
    const char *p = format;
    while (*p) {
        if (*p == '%' && *(p + 1) == 'd') {
            entry->arg_count++;
            p++; // Skip 'd'
        }
        p++;
    }


    // Step 4: Allocate argument buffer
    entry->args = malloc(sizeof(int) * entry->arg_count);
    if (!entry->args) { // Allocation failed
        free(entry->format);
        free(entry);
        return;
    }


    // Step 5: Extract args using va_list
    va_list args;
    va_start(args, format);
    for (int i = 0; i < entry->arg_count; i++) {
        entry->args[i] = va_arg(args, int);
    }
    va_end(args);
    __NOP();
    
    free(entry->format);
    free(entry->args);
    free(entry);

}

static uint8_t buffer_selector = 0;
static char unformatted_buffer[2][200] = {{0}};
static int32_t argument_buffer[2][20] = {{0}};

static uint32_t buffer_max_index = 0;
static uint32_t buffer_max_arg = 0;

void testqueue(const char* format, ...){
    
    uint32_t format_size = strlen(format);
    uint32_t format_buffer_size = sizeof(unformatted_buffer[0]) / sizeof(unformatted_buffer[0][0]);
    if (buffer_max_index + format_size >= format_buffer_size) return; // Does not fit in the format buffer

    uint32_t arg_count = 0;
    const char *p = format;
    while (*p) {
        if (*p == '%' && *(p + 1) == 'd') {
            arg_count++;
            p++; // Skip 'd'
        }
        p++;
    }

    uint32_t arg_buffer_size = sizeof(argument_buffer[0]) / sizeof(argument_buffer[0][0]);
    if (buffer_max_arg + arg_count > arg_buffer_size) return; // Does not fit in the argument buffer


    strcpy(&unformatted_buffer[buffer_selector][buffer_max_index], format);
    buffer_max_index += strlen(format);

    va_list args;
    va_start(args, format);
    for (uint32_t i = buffer_max_arg; i < (buffer_max_arg + arg_count); i++) {
        argument_buffer[buffer_selector][i] = va_arg(args, int);
    }
    va_end(args);
    
    buffer_max_arg += arg_count;
    __NOP();

}







int mini_snprintf(char* buffer, size_t size, const char* format, ...) {
    va_list args;
    va_start(args, format);
    
    size_t pos = 0;
    
    for (const char* p = format; *p != '\0'; ++p) {
        if (*p == '%' && *(p + 1) == 'd') {
            p++; // Skip 'd'
            int val = va_arg(args, int);
            char numbuf[32];
            int_to_str(val, numbuf);

            for (int i = 0; numbuf[i] != '\0' && pos < size - 1; ++i) {
                buffer[pos++] = numbuf[i];
            }
        } else {
            if (pos < size - 1) {
                buffer[pos++] = *p;
            }
        }
    }

    buffer[pos] = '\0';
    va_end(args);
    return (int)pos;
}


void int_to_str(int value, char* buffer) {
    int i = 0;
    int is_negative = 0;

    if (value == 0) {
        buffer[i++] = '0';
        buffer[i] = '\0';
        return;
    }

    if (value < 0) {
        is_negative = 1;
        value = -value;
    }

    // Convert digits in reverse order
    while (value != 0) {
        buffer[i++] = (value % 10) + '0';
        value /= 10;
    }

    if (is_negative) {
        buffer[i++] = '-';
    }

    buffer[i] = '\0';

    // Reverse the string
    for (int j = 0, k = i - 1; j < k; j++, k--) {
        char temp = buffer[j];
        buffer[j] = buffer[k];
        buffer[k] = temp;
    }
}





int powers_of_ten[10] = {1, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000};

//returns the number of digits in a number(in base 10), input needs to be be positive
uint8_t calculate_digits_count(int32_t num) {
    uint8_t length = 1; //if less than 10, it has 1 digit
    while(num >= powers_of_ten[length]) {
        length++;
    }
    return length;
}

uint8_t int_to_str2(int value, char* buffer){
    uint8_t i = 0;

    if (value == 0) {
        buffer[i++] = '0';
        buffer[i] = '\0';
        return 1; 
    }

    if (value < 0) {
        value = -value;
        buffer[i++] = '-';
    }

    uint8_t length = calculate_digits_count(value);

    while (length > 0) {
        int digit = value / powers_of_ten[length - 1];
        buffer[i++] = digit + '0';
        value -= digit * powers_of_ten[length - 1];
        length--;
    }
    buffer[i] = '\0';

    return length;
}