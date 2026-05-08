#include "FOC_USB_Debug.h"
#include <string.h>
#include "Utils.h"
#include "usbd_cdc_if.h"

/******Packet Structure******/
/*
All values are represented either as a 4-byte float or a 4-byte integer, signed or unsigned depending on the signal type. 
The PC should interpret the values based on the signal definitions in FOC_USB_DEBUG_SIGNAL_LIST and the corresponding types (f for float, i32 for signed int, u32 for unsigned int).
All the USB packets follow the same structure: 
    SOF (2 bytes) | Msg Type (1 byte) | Payload Length (2 bytes) | Payload (N bytes)
Depending on the Msg Type, the payload can have different formats:
MSG_LOG_DATA: FOC -> PC
    Payload: Timestamp (4 bytes) | Sample Count (2 bytes) | Signal Count (2 bytes) | Data Buffer (Sample Count * Signal Count * 4 bytes)
    The data buffer contains the captured signal values in the order defined by the signal mask. Each value is a 4-byte float or integer depending on the signal type.
MSG_SET_MASK: PC -> FOC
    Payload: Signal Mask (4 bytes)
    The signal mask is a 32-bit value where each bit corresponds to a specific signal. 
    FOC_USB_DEBUG_SIGNAL_LIST defines the mapping (and type) of bits to signals in the FOC_HandleTypeDef structure.
    At most MAX_LOGDATA_SIGNAL_COUNT bits can be set in the mask, which determines how many signals will be captured and included in the log data packets.
    Mask can only be updated when logging is stopped.
MSG_START_LOG: PC -> FOC
    Payload: None
    Enables the logging of data based on the current signal mask. The FOC_USB_Debug_CaptureSamples function will start capturing samples and filling the log data payload.
MSG_STOP_LOG: PC -> FOC
    Payload: None
    Disables the logging of data. The FOC_USB_Debug_CaptureSamples function will stop capturing samples and filling the log data payload.
MSG_SET_PID: PC -> FOC
    Payload: Controller ID (1 byte) | PID Gains (12 bytes: 4 bytes for Kp, 4 bytes for Ki, 4 bytes for Kd)
    Sets the PID gains for a specific controller (e.g., position, speed, current).
    FOC_PID_CONTROLLERS_LIST defines the mapping of controller IDs to the actual PID controllers in the FOC_HandleTypeDef structure.
MSG_GET_PID: PC -> FOC
    Payload: Controller ID (1 byte)
    Requests the current PID gains for a specific controller.
MSG_PID_REPLY: FOC -> PC
    Payload: Controller ID (1 byte) | PID Gains (12 bytes: 4 bytes for Kp, 4 bytes for Ki, 4 bytes for Kd)
    Reply to a MSG_GET_PID request, containing the current PID gains for the requested controller.
MSG_SET_VAR: PC -> FOC
    Payload: Variable ID (1 byte) | Variable Value (4 bytes)
    VAR_ID_LIST defines the mapping of variable IDs to specific variables in the FOC_HandleTypeDef structure that can be set or read.
MSG_GET_VAR: PC -> FOC
    Payload: Variable ID (1 byte)
    Requests the current value of a specific variable defined in VAR_ID_LIST.
MSG_VAR_REPLY: FOC -> PC
    Payload: Variable ID (1 byte) | Variable Value (4 bytes)
    Reply to a MSG_GET_VAR request, containing the current value of the requested variable.
MSG_FLASH_SAVE: PC -> FOC
    Payload: None
    Instructs the FOC firmware to save the current configuration (e.g., PID gains, settings) to flash memory.
    Can only be executed, when FOC is in IDLE mode.
MSG_FLASH_LOAD: PC -> FOC
    Payload: None
    Instructs the FOC firmware to load the configuration from flash memory.
    Can only be executed, when FOC is in IDLE mode.
MSG_SET_STATE: PC -> FOC
    Payload: Desired State (4 byte)
    Sets the desired state of the FOC driver (e.g., RUN, OPENLOOP, FLASH_SAVE).
MSG_GET_STATE: PC -> FOC
    Payload: None
    Requests the current state of the FOC driver.
MSG_STATE_REPLY: FOC -> PC
    Payload: Current State (4 byte)
    Reply to a MSG_GET_STATE request, containing the current state of the FOC driver.
MSG_UNKNOWN_TYPE: FOC -> PC
    Payload: None
    Sent by the FOC firmware when it receives a message with an unrecognized Msg Type. Can be used for debugging and error handling on the PC side.
MSG_INVALID_PAYLOAD: FOC -> PC
    Payload: None
    Sent by the FOC firmware when it receives a message with a recognized Msg Type but the payload is invalid (e.g., wrong length, invalid values). Can be used for debugging and error handling on the PC side.
MSG_ACK: FOC -> PC
    Payload: None
    Sent by the FOC firmware to acknowledge the successful receipt and processing of a command from the PC (e.g., MSG_SET_MASK, MSG_START_LOG).
MSG_ERROR: FOC -> PC
    Payload: None
    Sent by the FOC firmware to indicate an error in processing a command from the PC (e.g., command not allowed in current state).

*/

#define SOF1 0xAA
#define SOF2 0x55

typedef enum {
    MSG_LOG_DATA = 0x01, //FOC -> PC
    MSG_SET_MASK = 0x02, //PC -> FOC
    MSG_START_LOG = 0x03, //PC -> FOC
    MSG_STOP_LOG = 0x04, //PC -> FOC
    MSG_SET_PID = 0x05, //PC -> FOC
    MSG_GET_PID = 0x06, //PC -> FOC
    MSG_PID_REPLY = 0x07, //FOC -> PC
    MSG_SET_VAR = 0x08, //PC -> FOC
    MSG_GET_VAR = 0x09, //PC -> FOC
    MSG_VAR_REPLY = 0x0A, //FOC -> PC
    MSG_FLASH_SAVE = 0x0B, //PC -> FOC
    MSG_FLASH_LOAD = 0x0C, //PC -> FOC
    MSG_SET_STATE = 0x0D, //PC -> FOC
    MSG_GET_STATE = 0x0E, //PC -> FOC
    MSG_STATE_REPLY = 0x0F, //FOC -> PC

    MSG_UNKNOWN_TYPE = 0xFA, //FOC -> PC
    MSG_INVALID_PAYLOAD = 0xFB, //FOC -> PC
    MSG_UNKNOWN_ID = 0xFC, //FOC -> PC
    MSG_BUFFER_OVERFLOW = 0xFD, //FOC -> PC
    MSG_ACK = 0xFE, //FOC -> PC
    MSG_ERROR = 0xFF //FOC -> PC
} MsgTypeTypeDef;

#define FOC_USB_DEBUG_SIGNAL_LIST(X)            \
    X(0, u32,   timestamp)                      \
    X(1, f,     adc_values.motor_temp)          \
    X(2, f,     adc_values.mosfet_temp)         \
    X(3, f,     adc_values.vbus)                \
    X(4, f,     ibus)                           \
    X(5, f,     adc_values.phase_current.a)     \
    X(6, f,     adc_values.phase_current.b)     \
    X(7, f,     adc_values.phase_current.c)     \
    X(8, f,     ab_current.alpha)               \
    X(9, f,     ab_current.beta)                \
    X(10, f,    dq_current.d)                   \
    X(11, f,    dq_current.q)                   \
    X(12, f,    phase_voltage.a)                \
    X(13, f,    phase_voltage.b)                \
    X(14, f,    phase_voltage.c)                \
    X(15, f,    ab_voltage.alpha)               \
    X(16, f,    ab_voltage.beta)                \
    X(17, f,    dq_voltage.d)                   \
    X(18, f,    dq_voltage.q)                   \
    X(19, f,    encoder_angle_mechanical)       \
    X(20, f,    encoder_speed_mechanical)       \
    X(21, f,    encoder_angle_electrical)       \
    X(22, f,    encoder_speed_electrical)       \
    X(23, f,    dq_current_setpoint.d)          \
    X(24, f,    dq_current_setpoint.q)          \
    X(25, f,    angle_setpoint)                 \
    X(26, f,    speed_setpoint)                 \
    X(27, u32,  execution_time.loop_max)        \

#define FOC_PID_CONTROLLERS_LIST(X)             \
    X(0, pid_current_d)                         \
    X(1, pid_current_q)                         \
    X(2, pid_speed)                             \
    X(3, pid_position)                          \

#define VAR_ID_LIST(X)                          \
    X(0, f, dq_current_setpoint.d)             \
    X(1, f, dq_current_setpoint.q)             \
    X(2, f, angle_setpoint)                    \
    X(3, f, speed_setpoint)                    \




#define MAX_LOGDATA_SAMPLE_COUNT 30
#define MAX_LOGDATA_SIGNAL_COUNT 8

#define RX_QUEUE_SIZE 2
#define TX_QUEUE_SIZE 2


typedef union{
    float    f;
    int32_t  i32;
    uint32_t u32;
} TypeConverterUnion;

typedef struct{
    uint32_t timestamp;
    uint16_t sample_count;
    uint16_t signal_count;
    uint32_t buffer[MAX_LOGDATA_SAMPLE_COUNT * MAX_LOGDATA_SIGNAL_COUNT];
} LogDataPayloadTypeDef;

typedef struct{
    uint8_t is_running;
    uint32_t signal_mask;
    LogDataPayloadTypeDef payload;
} LogDataHandleTypeDef;

typedef struct {
    uint8_t msg_type_uint;
    uint16_t payload_length;
} MsgHdr_t;

typedef struct{
    MsgHdr_t hdr;
    uint8_t payload[32];
} RxMsg_t;

typedef struct{
    MsgHdr_t hdr;
    uint8_t payload[(MAX_LOGDATA_SAMPLE_COUNT * MAX_LOGDATA_SIGNAL_COUNT) * 4 + 2];
} TxMsg_t;

typedef struct {
    volatile uint8_t head;
    volatile uint8_t tail;
    RxMsg_t items[RX_QUEUE_SIZE];
} RxQueue_t;

typedef struct {
    volatile uint8_t head;
    volatile uint8_t tail;
    TxMsg_t items[TX_QUEUE_SIZE];
} TxQueue_t;

typedef struct{
    volatile uint8_t tx_busy_flag;
    uint32_t tx_missed_packets;
    TxQueue_t tx_queue;
    uint8_t tx_buffer[5 + (MAX_LOGDATA_SAMPLE_COUNT * MAX_LOGDATA_SIGNAL_COUNT) * 4 + 2];
    
    RxQueue_t rx_queue;
} DebugUSBHandleTypeDef;



uint16_t Debug_ConstructPacket(uint8_t* usb_buffer, MsgTypeTypeDef msg_type, uint8_t* payload_buffer, uint16_t payload_length);


uint8_t Debug_AddPacketToTxBuffer(TxQueue_t* queue, uint8_t* payload, uint16_t len, MsgTypeTypeDef msg_type);
uint8_t Debug_AddPacketToRxBuffer(RxQueue_t* queue, uint8_t* payload, uint16_t len, MsgTypeTypeDef msg_type);

uint16_t Debug_PopPacketFromTxBuffer(TxQueue_t* queue, uint8_t *usb_buffer);
uint8_t Debug_PopPacketFromRxBuffer(RxQueue_t *queue, RxMsg_t *msg);

Debug_StatusTypeDef Debug_UpdateMask(uint32_t new_mask);
void Debug_StartLogging();
void Debug_StopLogging();

static DebugUSBHandleTypeDef hdebugusb = {0};
static LogDataHandleTypeDef hlogdata = {0};


Debug_StatusTypeDef FOC_USB_Setup(){
    Debug_StopLogging();
    Debug_UpdateMask(0);
    return DEBUG_OK;
}

/**
 * @brief Captures samples for the selected signals and stores them in the debug handle buffer. 
 *        Once enough samples are collected, it packages them into a USB packet and adds it to the transmission buffer.
 *        If the buffer is full, the captured samples will be discarded until there is space in the buffer.
 * @param hfoc Pointer to the FOC handle containing the current state and signal values.
 * @return Debug_StatusTypeDef indicating the status of the capture operation.
 */
Debug_StatusTypeDef FOC_USB_Debug_CaptureSamples(FOC_HandleTypeDef *hfoc){

    if(!hlogdata.is_running) return DEBUG_STOPPED;

    uint8_t signal_index = 0;
    uint16_t write_index;

    TypeConverterUnion conv;

    write_index = (uint16_t)hlogdata.payload.sample_count * hlogdata.payload.signal_count;

    #define CAPTURE_SIGNAL(bit, member, field)                                      \
        do {                                                                        \
            if ((hlogdata.signal_mask & (1u << (bit))) != 0u)                       \
            {                                                                       \
                conv.member = hfoc->field;                                          \
                hlogdata.payload.buffer[write_index + signal_index] = conv.u32;     \
                signal_index++;                                                     \
            }                                                                       \
        } while (0);

    FOC_USB_DEBUG_SIGNAL_LIST(CAPTURE_SIGNAL);
    #undef CAPTURE_SIGNAL

    if(++hlogdata.payload.sample_count >= MAX_LOGDATA_SAMPLE_COUNT){
        uint16_t payload_bytes = 8 + hlogdata.payload.sample_count * hlogdata.payload.signal_count * 4;
        if(!Debug_AddPacketToTxBuffer(&hdebugusb.tx_queue, (uint8_t*)&hlogdata.payload, payload_bytes, MSG_LOG_DATA)){
            hdebugusb.tx_missed_packets++;
        }
        hlogdata.payload.sample_count = 0;
    }

    hlogdata.payload.timestamp++;
    return DEBUG_OK;
}

/**
 * @brief Transmits a USB packet if there is one ready in the buffer and the USB interface is not busy. 
 *        and the USB interface is connected.
 * @return Debug_StatusTypeDef indicating the status of the transmission.
 */
Debug_StatusTypeDef FOC_USB_Debug_TransmitPacket(){
    if(!CDC_IsConnected()){
        hdebugusb.tx_busy_flag = 0;
        return DEBUG_ERROR;
    }

    if(hdebugusb.tx_busy_flag) return DEBUG_BUSY;

    uint16_t len = Debug_PopPacketFromTxBuffer(&hdebugusb.tx_queue, hdebugusb.tx_buffer);
    if(len == 0) return DEBUG_OK;
    
    hdebugusb.tx_busy_flag = 1;
    if(CDC_Transmit_FS(hdebugusb.tx_buffer, len) == USBD_OK){
        HAL_GPIO_WritePin(DEBUG_LED2_GPIO_Port, DEBUG_LED2_Pin, GPIO_PIN_SET);
        return DEBUG_OK;
    }else{
        return DEBUG_ERROR;
    }
    
}

void FOC_USB_Debug_ExecuteReceivedCommand(FOC_HandleTypeDef *hfoc){
    RxMsg_t rxmsg;
    if(!Debug_PopPacketFromRxBuffer(&hdebugusb.rx_queue, &rxmsg)){
        return;
    }
    uint8_t controller_id;

    switch ((MsgTypeTypeDef)(rxmsg.hdr.msg_type_uint)){
    case MSG_SET_MASK:
        if(rxmsg.hdr.payload_length != 4){
            Debug_AddPacketToTxBuffer(&hdebugusb.tx_queue, NULL, 0, MSG_INVALID_PAYLOAD);
            break;
        }
        uint32_t new_mask;
        memcpy(&new_mask, &rxmsg.payload[0], 4);
        if(Debug_UpdateMask(new_mask) != DEBUG_OK){
            Debug_AddPacketToTxBuffer(&hdebugusb.tx_queue, NULL, 0, MSG_ERROR);
            break;
        }
        Debug_AddPacketToTxBuffer(&hdebugusb.tx_queue, NULL, 0, MSG_ACK);
        break;
    
    case MSG_START_LOG:
        Debug_StartLogging();
        Debug_AddPacketToTxBuffer(&hdebugusb.tx_queue, NULL, 0, MSG_ACK);
        break;

    case MSG_STOP_LOG:
        Debug_StopLogging();
        Debug_AddPacketToTxBuffer(&hdebugusb.tx_queue, NULL, 0, MSG_ACK);
        break;

    case MSG_SET_PID:
        if(rxmsg.hdr.payload_length != 13){
            Debug_AddPacketToTxBuffer(&hdebugusb.tx_queue, NULL, 0, MSG_INVALID_PAYLOAD);
            break;
        }
        controller_id = rxmsg.payload[0];
        PIDValuesTypeDef new_gains;
        memcpy(&new_gains, &rxmsg.payload[1], sizeof(new_gains));
        switch (controller_id) {
        #define X(id, name)                                                         \
            case id:                                                                \
                PID_SetGains(&hfoc->name, new_gains);                               \
                break;
            FOC_PID_CONTROLLERS_LIST(X)
        #undef X
            default:
                Debug_AddPacketToTxBuffer(&hdebugusb.tx_queue, NULL, 0, MSG_UNKNOWN_ID);
                break;
        }
        Debug_AddPacketToTxBuffer(&hdebugusb.tx_queue, NULL, 0, MSG_ACK);
        break;
    case MSG_GET_PID:
        if(rxmsg.hdr.payload_length != 1){
            Debug_AddPacketToTxBuffer(&hdebugusb.tx_queue, NULL, 0, MSG_INVALID_PAYLOAD);
            break;
        }
        controller_id = rxmsg.payload[0];
        PIDValuesTypeDef current_gains;
        switch (controller_id) {
        #define X(id, name)                                                         \
            case id:                                                                \
                current_gains = PID_GetGains(&hfoc->name);                          \
                break;
            FOC_PID_CONTROLLERS_LIST(X)
        #undef X
            default:
                Debug_AddPacketToTxBuffer(&hdebugusb.tx_queue, NULL, 0, MSG_UNKNOWN_ID);
                break;
        }
        uint8_t response_payload[13];
        response_payload[0] = controller_id;
        memcpy(&response_payload[1], &current_gains, sizeof(current_gains));
        Debug_AddPacketToTxBuffer(&hdebugusb.tx_queue, response_payload, sizeof(response_payload), MSG_PID_REPLY);
        break;
    case MSG_FLASH_SAVE:
        //not implemented yet
        break;
    case MSG_FLASH_LOAD:
        //not implemented yet
        break;
    case MSG_SET_STATE:
        //not implemented yet
        break;
    case MSG_GET_STATE:
        //not implemented yet
        break;
    default:
        Debug_AddPacketToTxBuffer(&hdebugusb.tx_queue, NULL, 0, MSG_UNKNOWN_TYPE);
        break;
    }
}

/**
 * @brief Adds a packet to the USB transmission buffer.
 * @param buf Pointer to the buffer containing the packet data.
 * @param len Length of the packet data.
 * @param msg_type Type of the message.
 * @return Debug_StatusTypeDef indicating the status of the operation.
 */
uint8_t Debug_AddPacketToTxBuffer(TxQueue_t* queue, uint8_t* payload, uint16_t len, MsgTypeTypeDef msg_type){
    if(queue == NULL) return 0;

    uint8_t next = (uint8_t)((queue->head + 1U) % TX_QUEUE_SIZE);

    if (next == queue->tail) return 0;

    if (len > sizeof(queue->items[0].payload)) return 0;
    queue->items[queue->head].hdr.msg_type_uint = (uint8_t)msg_type;
    queue->items[queue->head].hdr.payload_length = len;

    if ((payload != NULL) && (len > 0U)){
        memcpy(queue->items[queue->head].payload, payload, len);
    }

    queue->head = next;
    return 1;
}

/**
 * @brief Adds a packet to the USB reception buffer.
 * @param queue Pointer to the reception queue.
 * @param payload Pointer to the buffer containing the packet data.
 * @param len Length of the packet data.
 * @param msg_type Type of the message.
 * @return Status of the operation. Returns 1 if successful, 0 otherwise.
 */
uint8_t Debug_AddPacketToRxBuffer(RxQueue_t* queue, uint8_t* payload, uint16_t len, MsgTypeTypeDef msg_type){
    if(queue == NULL || payload == NULL) return 0;

    uint8_t next = (uint8_t)((queue->head + 1U) % RX_QUEUE_SIZE);

    if (next == queue->tail) return 0;

    if (len > sizeof(queue->items[0].payload)) return 0;

    queue->items[queue->head].hdr.msg_type_uint = (uint8_t)msg_type;
    queue->items[queue->head].hdr.payload_length = len;

    if ((payload != NULL) && (len > 0U)){
        memcpy(queue->items[queue->head].payload, payload, len);
    }

    queue->head = next;
    return 1;
}

/**
 * @brief Pops a packet from the USB transmission buffer.
 * @param queue Pointer to the transmission queue.
 * @param usb_buffer Pointer to the buffer where the packet data will be copied.
 * @return Length of the popped packet. Returns 0 if the buffer is empty or if there was an error.
 */
uint16_t Debug_PopPacketFromTxBuffer(TxQueue_t* queue, uint8_t *usb_buffer){
    if ((usb_buffer == NULL) || (queue == NULL)) return 0;

    if (queue->head == queue->tail) return 0;

    TxMsg_t *msg;

    msg = &queue->items[queue->tail];

    usb_buffer[0] = SOF1;
    usb_buffer[1] = SOF2;
    usb_buffer[2] = (uint8_t)msg->hdr.msg_type_uint;
    usb_buffer[3] = (uint8_t)(msg->hdr.payload_length & 0xFFU);
    usb_buffer[4] = (uint8_t)((msg->hdr.payload_length >> 8) & 0xFFU);

    if (msg->hdr.payload_length > 0U){
        memcpy(&usb_buffer[5], msg->payload, msg->hdr.payload_length);
    }

    queue->tail = (uint8_t)((queue->tail + 1U) % TX_QUEUE_SIZE);

    return (uint16_t)(5U + msg->hdr.payload_length);
}



/**
 * @brief Pops a packet from the USB reception buffer.
 * @param queue Pointer to the reception queue.
 * @param msg Pointer to the message structure where the packet data will be copied.
 * @return Status of the operation. Returns 1 if successful, 0 otherwise.
 */
uint8_t Debug_PopPacketFromRxBuffer(RxQueue_t *queue, RxMsg_t *msg){
    if ((queue == NULL) || (msg == NULL)) return 0;

    if (queue->head == queue->tail) return 0;

    *msg = queue->items[queue->tail];
    queue->tail = (uint8_t)((queue->tail + 1U) % RX_QUEUE_SIZE);

    return 1;
}







void FOC_USB_Debug_TransmitCpltCallback(){
    hdebugusb.tx_busy_flag = 0;
    HAL_GPIO_WritePin(DEBUG_LED2_GPIO_Port, DEBUG_LED2_Pin, GPIO_PIN_RESET);
}

void Debug_ReceivePacket(uint8_t* buf, uint32_t* len){
    if(*len < 1){
        Debug_AddPacketToTxBuffer(&hdebugusb.tx_queue, NULL, 0, MSG_ERROR);
        return;
    }
    if(buf[0] != SOF1 || buf[1] != SOF2){
        Debug_AddPacketToTxBuffer(&hdebugusb.tx_queue, NULL, 0, MSG_ERROR);
        return;
    }
    
    MsgTypeTypeDef msg_type = (MsgTypeTypeDef)buf[2];
    uint16_t payload_length = (uint16_t)buf[3] | ((uint16_t)buf[4] << 8);
    if(!Debug_AddPacketToRxBuffer(&hdebugusb.rx_queue, &buf[5], payload_length, msg_type)){
        Debug_AddPacketToTxBuffer(&hdebugusb.tx_queue, NULL, 0, MSG_BUFFER_OVERFLOW);
    }
}


void Debug_StartLogging(){
    hlogdata.payload.sample_count = 0;
    hlogdata.is_running = 1;
}

void Debug_StopLogging(){
    hlogdata.is_running = 0;
}

Debug_StatusTypeDef Debug_UpdateMask(uint32_t new_mask){
    if(hlogdata.is_running) return DEBUG_ERROR;

    uint8_t set_bits = countbits(new_mask);
    if(set_bits > MAX_LOGDATA_SIGNAL_COUNT) return DEBUG_ERROR;
    
    hlogdata.signal_mask = new_mask;
    hlogdata.payload.signal_count = set_bits;

    return DEBUG_OK;
}
