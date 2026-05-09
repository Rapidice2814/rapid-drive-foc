#ifndef FOC_USB_H
#define FOC_USB_H

#include "main.h"


void USB_ReceivePacket();
void USB_TransmitPacket();

void USB_TransmitCpltCallback();
void USB_ReceiveCallback(uint8_t* buf, uint32_t* len);

uint8_t USB_SendResponse(uint8_t* buf, uint16_t len);
uint8_t USB_printf(const char* format, ...) __attribute__((format(printf, 1, 2)));


void USB_ProcessReceivedPacket(uint8_t* buf, uint16_t len);



#endif // FOC_USB_H