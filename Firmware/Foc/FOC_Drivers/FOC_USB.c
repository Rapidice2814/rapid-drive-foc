#include "FOC_USB.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include "Utils.h"
#include "usbd_cdc_if.h"


#define RX_QUEUE_SIZE 2
#define TX_QUEUE_SIZE 4

DECLARE_QUEUE_RESERVE(RxMsg_t, RxQueue, RX_QUEUE_SIZE)
DECLARE_QUEUE_RESERVE(TxMsg_t, TxQueue, TX_QUEUE_SIZE)

typedef struct{
    volatile uint8_t tx_busy_flag;
    uint32_t tx_missed_packets;
    uint32_t rx_missed_packets;
    TxQueue_t tx_queue;    
    RxQueue_t rx_queue;
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

    TxMsg_t* txmsg = TxQueue_reserve_pop(&hdebugusb.tx_queue);

    if(!txmsg) return;

    hdebugusb.tx_busy_flag = 1;
    if(CDC_Transmit_FS(txmsg->payload, txmsg->length) == USBD_OK){
        HAL_GPIO_WritePin(DEBUG_LED2_GPIO_Port, DEBUG_LED2_Pin, GPIO_PIN_SET);
        return;
    }else{
        TxQueue_cancel_pop(&hdebugusb.tx_queue);
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
    RxMsg_t* rxmsg = RxQueue_reserve_pop(&hdebugusb.rx_queue);
    if(!rxmsg) return;

    USB_ProcessReceivedPacket(rxmsg->payload, rxmsg->length);
    RxQueue_commit_pop(&hdebugusb.rx_queue);
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
    TxQueue_commit_pop(&hdebugusb.tx_queue);
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

    RxMsg_t* rxmsg = RxQueue_reserve_push(&hdebugusb.rx_queue);
    if(!rxmsg) return;
    memcpy(rxmsg->payload, buf, *len);
    rxmsg->length = (uint16_t)*len;

    uint8_t ret = RxQueue_commit_push(&hdebugusb.rx_queue);
    if(!ret){
        hdebugusb.rx_missed_packets++;
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

    TxMsg_t* txmsg = TxQueue_reserve_push(&hdebugusb.tx_queue);
    if (!txmsg) return 0;

    memcpy(txmsg->payload, buf, len);
    txmsg->length = len;

    uint8_t ret = TxQueue_commit_push(&hdebugusb.tx_queue);
    if(!ret){
        hdebugusb.tx_missed_packets++;
    }
    return ret;
}

/**
 * @brief Gets a buffer from the USB transmit queue.
 * @return A pointer to the reserved buffer, or 0 if no buffer is available.
 */
TxMsg_t* USB_GetTxBuffer(){
    return TxQueue_reserve_push(&hdebugusb.tx_queue);
}

/**
 * @brief Commits the previously reserved buffer to the USB transmit queue, making it available for transmission.
 * @return 1 if the buffer was successfully committed, 0 otherwise.
 */
uint8_t USB_CommitTxBuffer(){
    return TxQueue_commit_push(&hdebugusb.tx_queue);
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
