#ifndef FOC_USB_DEBUG_H
#define FOC_USB_DEBUG_H

#include "stdint.h"

typedef enum{
	DEBUG_OK,
	DEBUG_STOPPED,
	DEBUG_BUSY,
	DEBUG_ERROR
}Debug_StatusTypeDef;

Debug_StatusTypeDef FOC_USB_Setup();
Debug_StatusTypeDef FOC_USB_Debug_CaptureSamples();
uint8_t Debug_SendTextResponse(const char* format, ...) __attribute__((format(printf, 1, 2)));


#endif // FOC_USB_DEBUG_H