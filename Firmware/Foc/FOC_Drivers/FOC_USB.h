#ifndef FOC_USB_H
#define FOC_USB_H

#include "main.h"

typedef struct{
    uint16_t length;
    uint8_t payload[5 + 32];
} RxMsg_t;

typedef struct{
    uint16_t length;
    uint8_t payload[1024];
} TxMsg_t;


void USB_ReceivePacket();
void USB_TransmitPacket();

void USB_TransmitCpltCallback();
void USB_ReceiveCallback(uint8_t* buf, uint32_t* len);

TxMsg_t* USB_GetTxBuffer();
uint8_t USB_CommitTxBuffer();

uint8_t USB_SendResponse(uint8_t* buf, uint16_t len);
uint8_t USB_printf(const char* format, ...) __attribute__((format(printf, 1, 2)));


void USB_ProcessReceivedPacket(uint8_t* buf, uint16_t len);




#endif // FOC_USB_H