#ifndef FOC_USB_DEBUG_H
#define FOC_USB_DEBUG_H

#include "main.h"
#include "FOC_Utils.h"

typedef enum{
	DEBUG_OK,
	DEBUG_ERROR
}Debug_StatusTypeDef;

Debug_StatusTypeDef FOC_USB_Debug_CaptureSamples(FOC_HandleTypeDef *hfoc);
void FOC_USB_Debug_TransmitCpltCallback();


#endif // FOC_USB_DEBUG_H