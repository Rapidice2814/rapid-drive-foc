#include <string.h>
#include <stdarg.h>
#include <stdio.h>

#include "main.h"
#include "FOC_USB.h"
#include "Utils.h"
#include "usbd_cdc_if.h"


#define RX_QUEUE_SIZE 8
#define TX_QUEUE_SIZE 8

DECLARE_POOL(RxUsbBuf_t, RxQueue, RX_QUEUE_SIZE)
DECLARE_POOL(TxUsbBuf_t, TxQueue, TX_QUEUE_SIZE)

typedef struct{
    volatile uint8_t tx_busy_flag;
    uint32_t tx_no_buffers; // Number of times a transmit buffer could not be allocated due to the pool being full
    uint32_t rx_no_buffers; // Number of times a receive buffer could not be allocated due to the pool being full
    TxUsbBuf_t *tx_buffer;
} DebugUSBHandleTypeDef;

static DebugUSBHandleTypeDef hdebugusb = {0};


/**
 * @brief Starts the next queued transfer when the CDC connection is available.
 * @note Sends at most one buffer per call. A failed CDC submission clears the busy
 *       flag and requeues the buffer so a later call can retry.
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
        hdebugusb.tx_busy_flag = 0;
        TxQueue_push(hdebugusb.tx_buffer);
        return;
    }  
}

/**
 * @brief Dispatches one queued receive item and returns its buffer to the pool.
 * @note Call regularly from the FOC loop; parsing runs outside the receive callback.
 */
void USB_ReceivePacket(){
    RxUsbBuf_t* rxmsg = RxQueue_pop();
    if(!rxmsg) return;

    USB_ProcessReceivedPacket(rxmsg->payload, rxmsg->length);
    RxQueue_free(rxmsg);
}

/**
 * @brief Provides the default weak handler for queued receive bytes.
 * @param buf Received bytes; unused by this default implementation.
 * @param len Received byte count; unused by this default implementation.
 * @note FOC_USB_Debug.c overrides this handler with the protocol parser.
 */
__weak void USB_ProcessReceivedPacket(uint8_t* buf, uint16_t len){
    UNUSED(buf);
    UNUSED(len);
}

/**
 * @brief Releases the completed transmit buffer and clears the transfer-busy flag.
 * @note Called by the CDC transmit-complete callback.
 */
void USB_TransmitCpltCallback(){
    hdebugusb.tx_busy_flag = 0;
    TxQueue_free(hdebugusb.tx_buffer);
    HAL_GPIO_WritePin(DEBUG_LED2_GPIO_Port, DEBUG_LED2_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Copies one bounded CDC receive chunk into the receive pool.
 * @param buf Incoming bytes, which are copied before returning.
 * @param len Pointer to the received byte count supplied by the USB stack.
 * @note Empty, oversized, and unbufferable chunks are dropped.
 */
void USB_ReceiveCallback(uint8_t* buf, uint32_t* len){
    if(*len < 1 || *len > RX_USB_BUF_SIZE){
        return;
    }

    RxUsbBuf_t* rxmsg = RxQueue_alloc();
    if(!rxmsg) return;
    memcpy(rxmsg->payload, buf, *len);
    rxmsg->length = (uint16_t)*len;

    RxQueue_push(rxmsg);
}


/**
 * @brief Reserves a transmit-pool buffer for the caller to fill.
 * @return Reserved buffer, or NULL when the pool is exhausted.
 * @note Submit with USB_PushTxBuffer or release with USB_FreeTxBuffer.
 */
TxUsbBuf_t* USB_AllocTxBuffer(){
    TxUsbBuf_t* buf = TxQueue_alloc();
    if(!buf) hdebugusb.tx_no_buffers++;
    return buf;
}

/**
 * @brief Queues an allocated transmit buffer for submission to CDC.
 * @param buf Allocated buffer containing payload and length.
 * @return 1 if queued, otherwise 0.
 * @note Queue ownership transfers only on success; the caller owns a rejected buffer.
 */
uint8_t USB_PushTxBuffer(TxUsbBuf_t* buf){
    return TxQueue_push(buf);
}

/**
 * @brief Returns a reserved or completed transmit buffer to its pool.
 * @param buf Buffer allocated from the transmit pool.
 * @return 1 if released, or 0 if the pointer is invalid.
 */
uint8_t USB_FreeTxBuffer(TxUsbBuf_t* buf){
    return TxQueue_free(buf);
}

/**
 * @brief Reserves a receive-pool buffer for a producer to fill.
 * @return Reserved buffer, or NULL when the pool is exhausted.
 */
RxUsbBuf_t* USB_AllocRxBuffer(){
    RxUsbBuf_t* buf = RxQueue_alloc();
    if(!buf) hdebugusb.rx_no_buffers++;
    return buf;
}

/**
 * @brief Queues a filled receive-pool buffer for foreground processing.
 * @param buf Allocated receive buffer containing payload and length.
 * @return 1 if queued, otherwise 0.
 * @note The foreground loop releases successfully queued receive buffers.
 */
uint8_t USB_PushRxBuffer(RxUsbBuf_t* buf){
    return RxQueue_push(buf);
}


/**
 * @brief Formats and queues unframed USB text.
 * @param format printf-style format string.
 * @param ... Arguments consumed by the format string.
 * @return 1 if queued, or 0 if allocation, formatting, or enqueueing fails.
 * @note Text is truncated to the buffer capacity. Use Debug_SendTextResponse for
 *       framed text that the binary plotter can decode.
 */
uint8_t USB_printf(const char* format, ...){
    TxUsbBuf_t* msg = TxQueue_alloc();
    if(!msg) return 0;

    va_list args;
    int n;

    va_start(args, format);
    n = vsnprintf((char*)msg->payload, sizeof(msg->payload), format, args);
    va_end(args);

    if(n < 0) {
        TxQueue_free(msg);
        return 0;
    }

    if((size_t)n >= sizeof(msg->payload)) {
        n = sizeof(msg->payload) - 1;
    }

    msg->length = (uint16_t)n;
    uint8_t ret = TxQueue_push(msg);
    return ret;
}
