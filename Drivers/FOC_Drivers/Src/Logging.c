#include "Logging.h"
#include <string.h>



void my_printf(const char *format, ...){
    int len = strlen(format);
    va_list args;
    va_start(args, format);

    for(int i = 0; i < len; i++){
        if(format[i] == '%'){
            if(format[i + 1] == 'd'){
                int value = va_arg(args, int);
                printf("%d", value);
                i++;
            }
        }
    }
    char buffer[100];
    
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    buffer;
}