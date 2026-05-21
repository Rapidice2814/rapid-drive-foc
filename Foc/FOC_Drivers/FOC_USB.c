#include "FOC_USB.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include "Utils.h"
#include "usbd_cdc_if.h"


#define RX_QUEUE_SIZE 4
#define TX_QUEUE_SIZE 4

DECLARE_POOL(RxUsbBuf_t, RxQueue, RX_QUEUE_SIZE)
DECLARE_POOL(TxUsbBuf_t, TxQueue, TX_QUEUE_SIZE)

typedef struct{
    volatile uint8_t tx_busy_flag;
    uint32_t tx_missed_packets;
    uint32_t rx_missed_packets;
    TxUsbBuf_t *tx_buffer;
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

    hdebugusb.tx_buffer = TxQueue_pop();
    if(!hdebugusb.tx_buffer) return;

    hdebugusb.tx_busy_flag = 1;
    if(CDC_Transmit_FS(hdebugusb.tx_buffer->payload, hdebugusb.tx_buffer->length) == USBD_OK){
        HAL_GPIO_WritePin(DEBUG_LED2_GPIO_Port, DEBUG_LED2_Pin, GPIO_PIN_SET);
        return;
    }else{
        TxQueue_push(hdebugusb.tx_buffer);
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
    RxUsbBuf_t* rxmsg = RxQueue_pop();
    if(!rxmsg) return;

    USB_ProcessReceivedPacket(rxmsg->payload, rxmsg->length);
    RxQueue_free(rxmsg);
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
    TxQueue_free(hdebugusb.tx_buffer);
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

    RxUsbBuf_t* rxmsg = RxQueue_alloc();
    if(!rxmsg) return;
    memcpy(rxmsg->payload, buf, *len);
    rxmsg->length = (uint16_t)*len;

    uint8_t ret = RxQueue_push(rxmsg);
    if(!ret){
        hdebugusb.rx_missed_packets++;
    }
}


/**
 * @brief Gets a buffer from the USB transmit queue.
 * @return A pointer to the reserved buffer, or 0 if no buffer is available.
 */
TxUsbBuf_t* USB_AllocTxBuffer(){
    return TxQueue_alloc();
}

/**
 * @brief Pushes the previously reserved buffer to the USB transmit queue, making it available for transmission.
 * @param buf Pointer to the buffer that was reserved and filled with data to be transmitted.
 * @return 1 if the buffer was successfully pushed, 0 otherwise.
 */
uint8_t USB_PushTxBuffer(TxUsbBuf_t* buf){
    return TxQueue_push(buf);
}

/**
 * @brief Gets a buffer from the USB receive queue.
 * @return A pointer to the allocated buffer, or 0 if no buffer is available.
 */
RxUsbBuf_t* USB_AllocRxBuffer(){
    return RxQueue_alloc();
}

/**
 * @brief Pushes a received buffer to the USB receive queue for processing.
 * @param buf Pointer to the buffer that was reserved and filled with data to be transmitted.
 * @return 1 if the buffer was successfully pushed, 0 otherwise.
 */
uint8_t USB_PushRxBuffer(RxUsbBuf_t* buf){
    return RxQueue_push(buf);
}


/**
 * @brief A printf-like function that formats a string and sends it as a text response over USB. 
 *        The formatted string is limited to the size of the TxUsbBuf_t payload.
 * @param format The format string (like in printf).
 * @param ... The variables to format according to the format string.
 * @return 1 if the message was successfully queued for transmission, 0 otherwise.
 */
uint8_t USB_printf(const char* format, ...){
    TxUsbBuf_t* msg = TxQueue_alloc();
    va_list args;
    int n;

    va_start(args, format);
    n = vsnprintf((char*)msg->payload, sizeof(msg->payload), format, args);
    va_end(args);

    if (n < 0) {
        return 0;
    }

    if ((size_t)n >= sizeof(msg->payload)) {
        n = sizeof(msg->payload) - 1;
    }

    msg->length = (uint16_t)n;
    uint8_t ret = TxQueue_push(msg);
    if(!ret){
        hdebugusb.tx_missed_packets++;
    }
    return ret;
}
