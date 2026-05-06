#include "FOC_USB_Debug.h"
#include <string.h>
#include "Utils.h"
#include "usbd_cdc_if.h"

typedef enum {
    MSG_LOG_DATA = 0x01,
    MSG_SET_MASK = 0x02,
    MSG_START_LOG = 0x03,
    MSG_STOP_LOG = 0x04
} MsgTypeTypeDef;


#define MAX_SAMPLE_COUNT 30
#define MAX_SIGNAL_COUNT 8

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
    DebugPayloadTypeDef payload;
    uint8_t usb_busy_flag;
    uint8_t usb_buffer[5 + (MAX_SAMPLE_COUNT * MAX_SIGNAL_COUNT) * 4 + 2]; //header + payload + CRC
} DebugHandleTypeDef;

uint16_t Debug_ConstructPacket(uint8_t* usb_buffer, MsgTypeTypeDef msg_type, uint8_t* payload_buffer, uint16_t payload_length);
void Debug_TransmitPacket();
Debug_StatusTypeDef Debug_UpdateMask(uint32_t new_mask);



static DebugHandleTypeDef debug_handle = {0};



#define FOC_USB_DEBUG_SIGNAL_LIST(X)                             \
    X(0, u32,   timestamp)                                         \
    X(1, f,   ab_current.alpha)                                  \
    X(2, f,   ab_current.beta)                                   \
    X(3, f,   dq_current.d)                                      \
    X(4, f,   dq_current.q)                                      \
    X(5, f,   ab_voltage.alpha)                                  \
    X(6, f,   ab_voltage.beta)                                   \
    X(7, f,   dq_voltage.d)                                      \
    X(8, f,   dq_voltage.q)                                      \
    X(9, u32, execution_time.loop_max)

Debug_StatusTypeDef FOC_USB_Debug_CaptureSamples(FOC_HandleTypeDef *hfoc){

    Debug_UpdateMask(3);
    debug_handle.is_running = 1;

    if(!debug_handle.is_running) return DEBUG_ERROR;
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
        Debug_TransmitPacket();
        debug_handle.payload.sample_count = 0;
    }

    debug_handle.payload.timestamp++;
    return DEBUG_OK;
}

static uint32_t missed_usb_packets = 0;

void Debug_TransmitPacket() {
    if(!CDC_IsConnected()){
        debug_handle.usb_busy_flag = 0;
        return;
    }
    
    if(!debug_handle.usb_busy_flag){
        uint16_t payload_bytes = 8 + debug_handle.payload.sample_count * debug_handle.payload.signal_count * 4;
        uint16_t usb_buffer_len;
        usb_buffer_len = Debug_ConstructPacket(debug_handle.usb_buffer, MSG_LOG_DATA, (uint8_t*)&debug_handle.payload, payload_bytes);
        debug_handle.usb_busy_flag = 1;

        if(CDC_Transmit_FS(debug_handle.usb_buffer, usb_buffer_len) == USBD_OK){
            HAL_GPIO_TogglePin(DEBUG_LED2_GPIO_Port, DEBUG_LED2_Pin);
        }else{
            __NOP(); //ERROR
        }
    } else{
        missed_usb_packets++;
        __NOP(); //USB busy, skip this packet
    }
}

void FOC_USB_Debug_TransmitCpltCallback(){
    debug_handle.usb_busy_flag = 0;
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
    memcpy(&usb_buffer[5], payload, payload_length);

    return 5 + payload_length;
}
