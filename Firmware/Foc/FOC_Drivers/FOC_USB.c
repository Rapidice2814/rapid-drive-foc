#include "FOC_USB.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include "Utils.h"
#include "usbd_cdc_if.h"

#define RX_QUEUE_SIZE 2
#define TX_QUEUE_SIZE 4

typedef struct{
    uint16_t length;
    uint8_t payload[5 + 32];
} RxMsg_t;

typedef struct{
    uint16_t length;
    uint8_t payload[1024];
} TxMsg_t;

DECLARE_QUEUE(RxMsg_t, RxQueue, RX_QUEUE_SIZE)
DECLARE_QUEUE(TxMsg_t, TxQueue, TX_QUEUE_SIZE)

typedef struct{
    volatile uint8_t tx_busy_flag;
    uint32_t tx_missed_packets;
    TxQueue_t tx_queue;    
    RxQueue_t rx_queue;
    TxMsg_t tx_usb_buffer;
} DebugUSBHandleTypeDef;

static DebugUSBHandleTypeDef hdebugusb = {0};


/**
 * @brief Transmits a USB packet if there is one ready in the buffer and the USB interface is not busy. 
 *        and the USB interface is connected.
 * @return None.
 */
void USB_TransmitPacket(){
    if(!CDC_IsConnected()){
        hdebugusb.tx_busy_flag = 0;
        return;
    }

    if(hdebugusb.tx_busy_flag) return;

    if(!TxQueue_pop(&hdebugusb.tx_queue, &hdebugusb.tx_usb_buffer)) return;

    hdebugusb.tx_busy_flag = 1;
    if(CDC_Transmit_FS(hdebugusb.tx_usb_buffer.payload, hdebugusb.tx_usb_buffer.length) == USBD_OK){
        HAL_GPIO_WritePin(DEBUG_LED2_GPIO_Port, DEBUG_LED2_Pin, GPIO_PIN_SET);
        return;
    }else{
        return;
    }  
}

/**
 * @brief This function needs to be called ragularly to check for received USB packets. If a packet is received, it processes the packet and executes the corresponding command.
 * @param buf The buffer containing the received data.
 * @param len The length of the received data.
 * @return None.
 */
void USB_ReceivePacket(){
    RxMsg_t rxmsg;
    if(!RxQueue_pop(&hdebugusb.rx_queue, &rxmsg)){
        return;
    }

    USB_ProcessReceivedPacket(rxmsg.payload, rxmsg.length);
}

/**
 * @brief A weak implementation of the packet processing function.
 * @param buf The buffer containing the received data.
 * @param len The length of the received data.
 * @return None.
 */
__weak void USB_ProcessReceivedPacket(uint8_t* buf, uint16_t len){
    UNUSED(buf);
    UNUSED(len);
}

/**
 * @brief Callback function that is called when a USB packet transmission is completed.
 * @return None.
 */
void USB_TransmitCpltCallback(){
    hdebugusb.tx_busy_flag = 0;
    HAL_GPIO_WritePin(DEBUG_LED2_GPIO_Port, DEBUG_LED2_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Callback function that is called when a USB packet is received.
 * @param buf The buffer containing the received data.
 * @param len The length of the received data.
 * @return None.
 */
void USB_ReceiveCallback(uint8_t* buf, uint32_t* len){
    if(*len < 1){
        return;
    }

    RxMsg_t rxmsg;
    memcpy(rxmsg.payload, buf, *len);
    rxmsg.length = (uint16_t)*len;

    if(!RxQueue_push(&hdebugusb.rx_queue, &rxmsg)){
        return;
    }
}

/**
 * @brief Sends a response packet over USB.
 * @param buf The buffer containing the data to send.
 * @param len The length of the data to send.
 * @return 1 if the message was successfully queued for transmission, 0 otherwise.
 */
uint8_t USB_SendResponse(uint8_t* buf, uint16_t len){

    if (len > sizeof(TxMsg_t) - 1) return 0;

    TxMsg_t msg;
    memcpy(msg.payload, buf, len);
    msg.length = len;

    uint8_t ret = TxQueue_push(&hdebugusb.tx_queue, &msg);
    if(!ret){
        hdebugusb.tx_missed_packets++;
    }
    return ret;
}

/**
 * @brief A printf-like function that formats a string and sends it as a text response over USB. 
 *        The formatted string is limited to the size of the TxMsg_t payload.
 * @param format The format string (like in printf).
 * @param ... The variables to format according to the format string.
 * @return 1 if the message was successfully queued for transmission, 0 otherwise.
 */
uint8_t USB_printf(const char* format, ...){
    return 0;
    TxMsg_t msg;
    va_list args;
    int n;

    va_start(args, format);
    n = vsnprintf((char*)msg.payload, sizeof(msg.payload), format, args);
    va_end(args);

    if (n < 0) {
        return 0;
    }

    if ((size_t)n >= sizeof(msg.payload)) {
        n = sizeof(msg.payload) - 1;
    }

    msg.length = (uint16_t)n;
    uint8_t ret = TxQueue_push(&hdebugusb.tx_queue, &msg);
    if(!ret){
        hdebugusb.tx_missed_packets++;
    }
    return ret;
}
