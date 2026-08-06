#ifndef FOC_USB_H
#define FOC_USB_H

#include <stdint.h>

#define RX_USB_BUF_SIZE 128
#define TX_USB_BUF_SIZE 1024

typedef struct{
    uint16_t length;
    uint8_t payload[RX_USB_BUF_SIZE];
} RxUsbBuf_t;

typedef struct{
    uint16_t length;
    uint8_t payload[TX_USB_BUF_SIZE];
} TxUsbBuf_t;


void USB_ReceivePacket();
void USB_TransmitPacket();

void USB_TransmitCpltCallback();
void USB_ReceiveCallback(uint8_t* buf, uint32_t* len);

TxUsbBuf_t* USB_AllocTxBuffer();
uint8_t USB_PushTxBuffer(TxUsbBuf_t* buf);
uint8_t USB_FreeTxBuffer(TxUsbBuf_t* buf);
RxUsbBuf_t* USB_AllocRxBuffer();
uint8_t USB_PushRxBuffer(RxUsbBuf_t* buf);


uint8_t USB_printf(const char* format, ...) __attribute__((format(printf, 1, 2)));


void USB_ProcessReceivedPacket(uint8_t* buf, uint16_t len);




#endif // FOC_USB_H