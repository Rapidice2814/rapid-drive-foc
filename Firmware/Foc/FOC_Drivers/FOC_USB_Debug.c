#include "FOC_USB_Debug.h"
#include <string.h>
#include "Utils.h"
#include "usbd_cdc_if.h"

typedef enum {
    MSG_LOG_DATA = 0x01, //FOC -> PC
    MSG_SET_MASK = 0x02, //PC -> FOC
    MSG_START_LOG = 0x03, //PC -> FOC
    MSG_STOP_LOG = 0x04, //PC -> FOC
    MSG_SET_PID = 0x05, //PC -> FOC
    MSG_GET_PID = 0x06, //PC -> FOC
    MSG_PID_REPLY = 0x07, //FOC -> PC
    MSG_FLASH_SAVE = 0x08, //PC -> FOC
    MSG_FLASH_LOAD = 0x09, //PC -> FOC
    MSG_SET_STATE = 0x0A, //PC -> FOC
    MSG_GET_STATE = 0x0B, //PC -> FOC
    MSG_STATE_REPLY = 0x0C, //FOC -> PC
    MSG_ACK = 0x0D, //FOC -> PC
    MSG_ERROR = 0x0E //FOC -> PC
} MsgTypeTypeDef;


#define MAX_SAMPLE_COUNT 30
#define MAX_SIGNAL_COUNT 8

#define USB_TX_BUFFERS 2

typedef union
{
    float    f;
    int32_t  i32;
    uint32_t u32;
} TypeConverterUnion;

typedef struct
{
    uint32_t timestamp;
    uint16_t sample_count;
    uint16_t signal_count;
    uint32_t buffer[MAX_SAMPLE_COUNT * MAX_SIGNAL_COUNT];
} DebugPayloadTypeDef;

typedef struct
{
    uint32_t signal_mask;
    uint8_t is_running;
    uint32_t missed_usb_packets;
    DebugPayloadTypeDef payload;
    volatile uint8_t usb_busy_flag;
    volatile uint16_t usb_buffer_length[USB_TX_BUFFERS];
    uint8_t usb_buffer[USB_TX_BUFFERS][5 + (MAX_SAMPLE_COUNT * MAX_SIGNAL_COUNT) * 4 + 2]; //header + payload + CRC
    uint8_t buffer_read_index;
    uint8_t buffer_write_index;
    uint8_t buffer_interrupt_index;
} DebugHandleTypeDef;

uint16_t Debug_ConstructPacket(uint8_t* usb_buffer, MsgTypeTypeDef msg_type, uint8_t* payload_buffer, uint16_t payload_length);
Debug_StatusTypeDef Debug_AddPacketToBuffer(uint8_t* buf, uint16_t len, MsgTypeTypeDef msg_type);
Debug_StatusTypeDef Debug_UpdateMask(uint32_t new_mask);
void Debug_StartLogging();
void Debug_StopLogging();



static DebugHandleTypeDef debug_handle = {0};



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

Debug_StatusTypeDef FOC_USB_Setup(){
    Debug_StopLogging();
    Debug_UpdateMask(0);
    return DEBUG_OK;
}

Debug_StatusTypeDef FOC_USB_Debug_CaptureSamples(FOC_HandleTypeDef *hfoc){

    if(!debug_handle.is_running) return DEBUG_STOPPED;
    if(debug_handle.payload.sample_count >= MAX_SAMPLE_COUNT) return DEBUG_ERROR;

    uint8_t signal_index = 0;
    uint16_t write_index;

    TypeConverterUnion conv;

    write_index = (uint16_t)debug_handle.payload.sample_count * debug_handle.payload.signal_count;

    #define CAPTURE_SIGNAL(bit, member, field)                                    \
        do {                                                                      \
            if ((debug_handle.signal_mask & (1u << (bit))) != 0u)                 \
            {                                                                     \
                conv.member = hfoc->field;                                        \
                debug_handle.payload.buffer[write_index + signal_index] = conv.u32;\
                signal_index++;                                                   \
            }                                                                     \
        } while (0);

    FOC_USB_DEBUG_SIGNAL_LIST(CAPTURE_SIGNAL);
    #undef CAPTURE_SIGNAL

    if(++debug_handle.payload.sample_count >= MAX_SAMPLE_COUNT){
        uint16_t payload_bytes = 8 + debug_handle.payload.sample_count * debug_handle.payload.signal_count * 4;
        Debug_AddPacketToBuffer((uint8_t*)&debug_handle.payload, payload_bytes, MSG_LOG_DATA);
        debug_handle.payload.sample_count = 0;
    }

    debug_handle.payload.timestamp++;
    return DEBUG_OK;
}

Debug_StatusTypeDef FOC_USB_Debug_TransmitPacket(){
    if(debug_handle.usb_buffer_length[debug_handle.buffer_read_index] == 0){
        return DEBUG_OK;
    }

    if(!CDC_IsConnected()){
        debug_handle.usb_busy_flag = 0;
        return DEBUG_ERROR;
    }

    if(!debug_handle.usb_busy_flag){
        debug_handle.usb_busy_flag = 1;

        if(CDC_Transmit_FS(debug_handle.usb_buffer[debug_handle.buffer_read_index], debug_handle.usb_buffer_length[debug_handle.buffer_read_index]) == USBD_OK){
            debug_handle.buffer_interrupt_index = debug_handle.buffer_read_index;
            debug_handle.buffer_read_index = (debug_handle.buffer_read_index + 1) % USB_TX_BUFFERS;
            HAL_GPIO_WritePin(DEBUG_LED2_GPIO_Port, DEBUG_LED2_Pin, GPIO_PIN_SET);
            return DEBUG_OK;
        }else{
            return DEBUG_ERROR;
        }
    }
    return DEBUG_BUSY;
}

Debug_StatusTypeDef Debug_AddPacketToBuffer(uint8_t* buf, uint16_t len, MsgTypeTypeDef msg_type){
    if(debug_handle.usb_buffer_length[debug_handle.buffer_write_index] != 0){
        debug_handle.missed_usb_packets++;
        return DEBUG_BUSY;
    }
    debug_handle.usb_buffer_length[debug_handle.buffer_write_index] = Debug_ConstructPacket(debug_handle.usb_buffer[debug_handle.buffer_write_index], msg_type, buf, len);
    debug_handle.buffer_write_index = (debug_handle.buffer_write_index + 1) % USB_TX_BUFFERS;

    uint8_t temp_counter = 0;
    for(uint8_t i = 0; i < USB_TX_BUFFERS; i++){
        if(debug_handle.usb_buffer_length[i] != 0){
            temp_counter++;
        }
    }
    if(temp_counter >= 2){
        __NOP();
    }

    return DEBUG_OK;
}

void FOC_USB_Debug_TransmitCpltCallback(){
    debug_handle.usb_busy_flag = 0;
    debug_handle.usb_buffer_length[debug_handle.buffer_interrupt_index] = 0;
    HAL_GPIO_WritePin(DEBUG_LED2_GPIO_Port, DEBUG_LED2_Pin, GPIO_PIN_RESET);
}

void Debug_ReceivePacket(uint8_t* buf, uint32_t* len){
    if(*len < 1) return;
    if(buf[0] != 0xAA || buf[1] != 0x55) return; //check SOF
    MsgTypeTypeDef msg_type = (MsgTypeTypeDef)buf[2];
    uint16_t payload_length = (uint16_t)buf[3] | ((uint16_t)buf[4] << 8);
    UNUSED(payload_length);
    
    switch (msg_type)
    {
    case MSG_SET_MASK:
        if(*len >= 5){
            uint32_t new_mask = (uint32_t)buf[5] | ((uint32_t)buf[6] << 8) | ((uint32_t)buf[7] << 16) | ((uint32_t)buf[8] << 24);
            Debug_UpdateMask(new_mask);
        }
        Debug_AddPacketToBuffer(NULL, 0, MSG_ACK);
        break;
    
    case MSG_START_LOG:
        Debug_StartLogging();
        Debug_AddPacketToBuffer(NULL, 0, MSG_ACK);
        break;

    case MSG_STOP_LOG:
        Debug_StopLogging();
        Debug_AddPacketToBuffer(NULL, 0, MSG_ACK);
        break;

    case MSG_SET_PID:
        //not implemented yet
        break;
    case MSG_GET_PID:
        //not implemented yet
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
        break;
    }
}


void Debug_StartLogging() {
    debug_handle.payload.sample_count = 0;
    debug_handle.is_running = 1;
}

void Debug_StopLogging() {
    debug_handle.is_running = 0;
}

Debug_StatusTypeDef Debug_UpdateMask(uint32_t new_mask) {
    if(debug_handle.is_running) return DEBUG_ERROR;

    uint8_t set_bits = countbits(new_mask);
    if(set_bits > MAX_SIGNAL_COUNT) return DEBUG_ERROR;
    
    debug_handle.signal_mask = new_mask;
    debug_handle.payload.signal_count = set_bits;

    return DEBUG_OK;
}

uint16_t Debug_ConstructPacket(uint8_t* usb_buffer, MsgTypeTypeDef msg_type, uint8_t* payload, uint16_t payload_length) {
    usb_buffer[0] = 0xAA; //SOF1
    usb_buffer[1] = 0x55; //SOF2
    usb_buffer[2] = (uint8_t)msg_type;
    usb_buffer[3] = (uint8_t)(payload_length & 0xFF); //Payload length LSB
    usb_buffer[4] = (uint8_t)((payload_length >> 8) & 0xFF); //Payload length MSB
    if (payload != NULL && payload_length > 0) {
        memcpy(&usb_buffer[5], payload, payload_length);
    }

    return 5 + payload_length;
}
