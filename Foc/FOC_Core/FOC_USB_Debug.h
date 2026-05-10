#ifndef FOC_USB_DEBUG_H
#define FOC_USB_DEBUG_H

#include "FOC_Utils.h"
#include "FOC_USB.h"

typedef enum{
	DEBUG_OK,
	DEBUG_STOPPED,
	DEBUG_BUSY,
	DEBUG_ERROR
}Debug_StatusTypeDef;

Debug_StatusTypeDef FOC_USB_Setup();
Debug_StatusTypeDef FOC_USB_Debug_CaptureSamples();


#endif // FOC_USB_DEBUG_H