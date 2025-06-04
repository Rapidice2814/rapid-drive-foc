#include "Logging.h"
#include "main.h"
#include <string.h>

int mini_snprintf(char* buffer, size_t size, const char* format, ...);
void int_to_str(int value, char* buffer);
uint8_t int_to_str2(int value, char* buffer);

extern UART_HandleTypeDef huart3;
char buffer[200];

void test(){
    int arr[10] = {0543, -1531, 2456, 376, -4678, 56879, -6345, 7234, -837, 9653};
    mini_snprintf(buffer, sizeof(buffer), "Hello: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n", arr[0], arr[1], arr[2], arr[3], arr[4], arr[5], arr[6], arr[7], arr[8], arr[9]);
    // snprintf(buffer, sizeof(buffer), "Hello: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n", arr[0], arr[1], arr[2], arr[3], arr[4], arr[5], arr[6], arr[7], arr[8], arr[9]);

    buffer;
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