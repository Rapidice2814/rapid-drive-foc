#ifndef FOC_USB_DEBUG_H
#define FOC_USB_DEBUG_H

#include "main.h"
#include "FOC_Utils.h"

typedef enum{
	DEBUG_OK,
	DEBUG_STOPPED,
	DEBUG_ERROR
}Debug_StatusTypeDef;

Debug_StatusTypeDef FOC_USB_Setup();
Debug_StatusTypeDef FOC_USB_Debug_CaptureSamples(FOC_HandleTypeDef *hfoc);
void FOC_USB_Debug_TransmitCpltCallback();
void Debug_ReceivePacket(uint8_t* buf, uint32_t* len);


#endif // FOC_USB_DEBUG_H