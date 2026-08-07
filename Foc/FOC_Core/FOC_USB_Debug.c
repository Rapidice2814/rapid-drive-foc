#include <string.h>
#include <stdarg.h>
#include <stdio.h>

#include "FOC_USB_Debug.h"
#include "FOC_Handle.h"
#include "FOC_States.h"
#include "FOC_USB.h"
#include "Timing.h"
#include "FOC_Config.h"

uint32_t usb_debug_times[5] = {0};

/******Packet Structure******/
/*
Binary Packets:
Binary packets are detected based on the SOF bytes at the beginning of the payload. In case the SOF bytes are not found, the packet is decoded as a text packet, based on ASCII values.
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
    Payload: Desired State (1 byte)
    Sets the desired state of the FOC driver (e.g., RUN, OPENLOOP, FLASH_SAVE).
MSG_GET_STATE: PC -> FOC
    Payload: None
    Requests the current state of the FOC driver.
MSG_STATE_REPLY: FOC -> PC
    Payload: Current State (1 byte)
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

typedef enum {
    MSG_GET_VERSION = 0x00, //PC -> FOC
    MSG_VERSION_REPLY = 0x01, //FOC -> PC
    MSG_LOG_DATA = 0x02, //FOC -> PC
    MSG_SET_MASK = 0x03, //PC -> FOC
    MSG_START_LOG = 0x04, //PC -> FOC
    MSG_STOP_LOG = 0x05, //PC -> FOC
    MSG_SET_PID = 0x06, //PC -> FOC
    MSG_GET_PID = 0x07, //PC -> FOC
    MSG_PID_REPLY = 0x08, //FOC -> PC
    MSG_SET_VAR = 0x09, //PC -> FOC
    MSG_GET_VAR = 0x0A, //PC -> FOC
    MSG_VAR_REPLY = 0x0B, //FOC -> PC
    MSG_FLASH_SAVE = 0x0C, //PC -> FOC
    MSG_FLASH_LOAD = 0x0D, //PC -> FOC
    MSG_SET_STATE = 0x0E, //PC -> FOC
    MSG_GET_STATE = 0x0F, //PC -> FOC
    MSG_STATE_REPLY = 0x10, //FOC -> PC
    MSG_TEXT_COMMAND = 0x11, // PC -> FOC
    MSG_TEXT_REPLY = 0x12, // FOC -> PC

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
    X(19, f,    encoder_angle_mechanical_wrapped) \
    X(20, f,    encoder_angle_mechanical_unwrapped) \
    X(21, f,    encoder_speed_mechanical)       \
    X(22, f,    encoder_angle_electrical)       \
    X(23, f,    encoder_speed_electrical)       \
    X(24, f,    dq_current_setpoint.d)          \
    X(25, f,    dq_current_setpoint.q)          \
    X(26, f,    angle_setpoint)                 \
    X(27, f,    speed_setpoint)                 \
    X(28, u32,  execution_time.loop_max)        \

#define FOC_PID_CONTROLLERS_LIST(X)             \
    X(0, pid_current_d)                         \
    X(1, pid_current_q)                         \
    X(2, pid_speed)                             \
    X(3, pid_position)                          \

#define VAR_ID_LIST(X)                         \
    X(0, f, dq_current_setpoint.d)             \
    X(1, f, dq_current_setpoint.q)             \
    X(2, f, angle_setpoint)                    \
    X(3, f, speed_setpoint)                    \
    X(4, f, flash_data.limits.max_dq_current)  \
    X(5, f, flash_data.limits.max_dq_voltage)  \
    X(6, f, flash_data.limits.vbus_overvoltage_trip_level) \
    X(7, f, flash_data.limits.vbus_undervoltage_trip_level) \
    X(8, f, flash_data.limits.ibus_overcurrent_trip_level) \
    X(9, f, flash_data.limits.motor_temp_trip_level) \
    X(10, f, flash_data.limits.mosfet_temp_trip_level) \
    X(11, u32, flash_data.motor.pole_pairs) \
    X(12, f, flash_data.motor.phase_resistance) \
    X(13, f, flash_data.motor.phase_inductance) \
    X(14, f, flash_data.motor.torque_constant) \



typedef union{
    float    f;
    int32_t  i32;
    uint32_t u32;
} TypeConvU_t;

typedef struct{
    uint8_t is_running;
    uint32_t signal_mask;
    uint32_t timestamp;
    uint16_t sample_count;
    uint16_t signal_count;
    TxUsbBuf_t* txbuf;
} LogDataHandleTypeDef;

extern FOC_HandleTypeDef hfoc;

static uint8_t Debug_SendBinaryResponse(MsgTypeTypeDef msg_type, uint8_t* payload, uint16_t len);
static void Debug_ExecuteTextCommand(const char *packet, uint16_t length);

static Debug_StatusTypeDef Debug_UpdateMask(uint32_t new_mask);
static void Debug_StartLogging();
static void Debug_StopLogging();

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
Debug_StatusTypeDef FOC_USB_Debug_CaptureSamples(void){
    uint32_t start_time = get_current_time();
    if(!hlogdata.is_running) return DEBUG_STOPPED;

    if(hlogdata.txbuf == NULL){
        hlogdata.txbuf = USB_AllocTxBuffer();
        if(hlogdata.txbuf == NULL) return DEBUG_ERROR;

        hlogdata.sample_count = 0;

        hlogdata.txbuf->payload[0] = DEBUG_SOF1_BIN;
        hlogdata.txbuf->payload[1] = DEBUG_SOF2_BIN;
        hlogdata.txbuf->payload[2] = (uint8_t)MSG_LOG_DATA;

        write_u32_le(&hlogdata.txbuf->payload[5], hlogdata.timestamp);
        write_u16_le(&hlogdata.txbuf->payload[9], 0);
        write_u16_le(&hlogdata.txbuf->payload[11], hlogdata.signal_count);
    }

    uint8_t signal_index = 0;
    uint16_t write_index = (uint16_t)(13u + hlogdata.sample_count * hlogdata.signal_count * 4u);
    TypeConvU_t conv;

    #define CAPTURE_SIGNAL(bit, member, field)                                  \
        do{                                                                      \
            if((hlogdata.signal_mask & (1u << (bit))) != 0u){                   \
                conv.member = hfoc.field;                                        \
                write_u32_le(&hlogdata.txbuf->payload[write_index + signal_index * 4u], conv.u32); \
                signal_index++;                                                  \
            }                                                                    \
        }while(0);

    FOC_USB_DEBUG_SIGNAL_LIST(CAPTURE_SIGNAL);
    #undef CAPTURE_SIGNAL

    hlogdata.sample_count++;
    write_u16_le(&hlogdata.txbuf->payload[9], hlogdata.sample_count);
    
    hlogdata.timestamp++;
    
    if(hlogdata.sample_count >= MAX_LOGDATA_SAMPLE_COUNT){
        uint16_t payload_bytes = (uint16_t)(8u + hlogdata.sample_count * hlogdata.signal_count * 4u);

        hlogdata.txbuf->payload[3] = (uint8_t)(payload_bytes & 0xFFu);
        hlogdata.txbuf->payload[4] = (uint8_t)((payload_bytes >> 8) & 0xFFu);
        hlogdata.txbuf->length = (uint16_t)(payload_bytes + 5u);

        USB_PushTxBuffer(hlogdata.txbuf);
        hlogdata.txbuf = NULL;
        hlogdata.sample_count = 0;
    }

    calculate_execution_time(&usb_debug_times[0], start_time);
    return DEBUG_OK;
}

static void Debug_StartLogging(){
    hlogdata.sample_count = 0;
    hlogdata.is_running = 1;
}

static void Debug_StopLogging(){
    hlogdata.is_running = 0;
}

static Debug_StatusTypeDef Debug_UpdateMask(uint32_t new_mask){
    if(hlogdata.is_running) return DEBUG_ERROR;

    uint8_t set_bits = countbits(new_mask);
    if(set_bits > MAX_LOGDATA_SIGNAL_COUNT) return DEBUG_ERROR;
    
    hlogdata.signal_mask = new_mask;
    hlogdata.signal_count = set_bits;

    return DEBUG_OK;
}

static void Debug_ExecuteBinaryCommand(MsgTypeTypeDef msg_type, uint8_t* payload, uint16_t payload_length){
    uint8_t controller_id;
    uint8_t var_id;
    uint8_t response_payload[13];

    switch ((MsgTypeTypeDef)(msg_type)){
    case MSG_SET_MASK:
        if(payload_length != 4){
            Debug_SendBinaryResponse(MSG_INVALID_PAYLOAD, NULL, 0);
            break;
        }
        uint32_t new_mask;
        memcpy(&new_mask, payload, 4);
        if(Debug_UpdateMask(new_mask) != DEBUG_OK){
            Debug_SendBinaryResponse(MSG_ERROR, NULL, 0);
            break;
        }
        Debug_SendBinaryResponse(MSG_ACK, NULL, 0);
        break;
    
    case MSG_START_LOG:
        Debug_StartLogging();
        Debug_SendBinaryResponse(MSG_ACK, NULL, 0);
        break;

    case MSG_STOP_LOG:
        Debug_StopLogging();
        Debug_SendBinaryResponse(MSG_ACK, NULL, 0);
        break;

    case MSG_SET_PID:
        if(payload_length != 13){
            Debug_SendBinaryResponse(MSG_INVALID_PAYLOAD, NULL, 0);
            break;
        }
        controller_id = payload[0];
        PIDValuesTypeDef new_gains;
        memcpy(&new_gains, &payload[1], sizeof(new_gains));
        switch (controller_id) {
        #define X(id, name)                                                         \
            case id:                                                                \
                PID_SetGains(&hfoc.name, new_gains);                               \
                break;
            FOC_PID_CONTROLLERS_LIST(X)
        #undef X
            default:
                Debug_SendBinaryResponse(MSG_UNKNOWN_ID, NULL, 0);
                break;
        }
        Debug_SendBinaryResponse(MSG_ACK, NULL, 0);
        break;
    case MSG_GET_PID:
        if(payload_length != 1){
            Debug_SendBinaryResponse(MSG_INVALID_PAYLOAD, NULL, 0);
            break;
        }
        controller_id = payload[0];
        PIDValuesTypeDef current_gains;
        switch (controller_id) {
        #define X(id, name)                                                         \
            case id:                                                                \
                current_gains = PID_GetGains(&hfoc.name);                          \
                break;
            FOC_PID_CONTROLLERS_LIST(X)
        #undef X
            default:
                Debug_SendBinaryResponse(MSG_UNKNOWN_ID, NULL, 0);
                break;
        }
        response_payload[0] = controller_id;
        memcpy(&response_payload[1], &current_gains, sizeof(current_gains));
        Debug_SendBinaryResponse(MSG_PID_REPLY, response_payload, 13);
        break;
    case MSG_SET_VAR:
        if(payload_length != 5){
            Debug_SendBinaryResponse(MSG_INVALID_PAYLOAD, NULL, 0);
            break;
        }

        var_id = payload[0];
        TypeConvU_t conv;
        conv.u32 = read_u32_le(&payload[1]);

        switch (var_id) {
        #define X(id, typ, field)                                     \
            case id:                                                  \
                hfoc.field = conv.typ;                                \
                break;
            VAR_ID_LIST(X)
        #undef X
            default:
                Debug_SendBinaryResponse(MSG_UNKNOWN_ID, NULL, 0);
                break;
        }

        Debug_SendBinaryResponse(MSG_ACK, NULL, 0);
        break;
    case MSG_GET_VAR:
        if(payload_length != 1){
            Debug_SendBinaryResponse(MSG_INVALID_PAYLOAD, NULL, 0);
            break;
        }

        var_id = payload[0];
        response_payload[0] = var_id;

        switch (var_id) {
        #define X(id, typ, field)                                     \
            case id: {                                                \
                TypeConvU_t conv;                                     \
                conv.typ = hfoc.field;                                \
                write_u32_le(&response_payload[1], conv.u32);         \
                break;                                                \
            }
            VAR_ID_LIST(X)
        #undef X
            default:
                Debug_SendBinaryResponse(MSG_UNKNOWN_ID, NULL, 0);
                break;
        }

        Debug_SendBinaryResponse(MSG_VAR_REPLY, response_payload, 5);
        break;
    case MSG_FLASH_SAVE:
        //not implemented yet
        break;
    case MSG_FLASH_LOAD:
        //not implemented yet
        break;
    case MSG_SET_STATE:
        if(payload_length != 1){
            Debug_SendBinaryResponse(MSG_INVALID_PAYLOAD, NULL, 0);
            break;
        }

        FOC_SetState(&hfoc, (FOC_StateTypeDef)payload[0], FOC_STATE_NONE);
        Debug_SendBinaryResponse(MSG_ACK, NULL, 0);
        break;
    case MSG_GET_STATE:
        if(payload_length != 0){
            Debug_SendBinaryResponse(MSG_INVALID_PAYLOAD, NULL, 0);
            break;
        }

        // response_payload[0] = (uint8_t)FOC_GetState(&hfoc);
        Debug_SendBinaryResponse(MSG_STATE_REPLY, response_payload, 1);
        break;
    case MSG_TEXT_COMMAND:
        Debug_ExecuteTextCommand((const char*)payload, payload_length);
        break;
    default:
        Debug_SendBinaryResponse(MSG_UNKNOWN_TYPE, NULL, 0);
        break;
    }
}

static void Debug_ExecuteTextCommand(const char *packet, uint16_t length){
    for(int i = 0; i < 1; i++){
        if(packet[i] == 'D'){
            if(packet[i+1] == 'a'){
                hfoc.flash_data.controller.anticogging_FF_enabled = !hfoc.flash_data.controller.anticogging_FF_enabled;
            } else if(packet[i+1] == 'f'){
                hfoc.flash_data.controller.current_PID_FF_enabled = !hfoc.flash_data.controller.current_PID_FF_enabled;
            } 
        }
        if(packet[i] == 'A'){
            FOC_SetState(&hfoc, FOC_STATE_ANTICOGGING, FOC_STATE_NONE);
        }
        if(packet[i] == 'R'){
            FOC_SetState(&hfoc, FOC_STATE_RUN, FOC_STATE_NONE);
        }
        if(packet[i] == 'E'){
            FOC_SetState(&hfoc, FOC_STATE_ERROR, FOC_STATE_NONE);
        }
        if(packet[i] == 'O'){
            FOC_SetState(&hfoc, FOC_STATE_STOP, FOC_STATE_NONE);
        }
        if(packet[i] == 'F'){
            FOC_SetState(&hfoc, FOC_STATE_FLASH_SAVE, FOC_STATE_RUN);
        }
        if(packet[i] == 'B'){
            FOC_SetState(&hfoc, FOC_STATE_BOOTLOADER, FOC_STATE_NONE);
        }
        if(packet[i] == 'M'){
            if(packet[i+1] == 's'){
                hfoc.flash_data.controller.speed_PID_enabled = 1;
                hfoc.flash_data.controller.position_PID_enabled = 0;
                Debug_SendTextResponse("Enabled speed PID, disabled position PID\n");
            } else if(packet[i+1] == 'p'){
                hfoc.flash_data.controller.position_PID_enabled = 1;
                hfoc.flash_data.controller.speed_PID_enabled = 0;
                Debug_SendTextResponse("Enabled position PID, disabled speed PID\n");
            } else if(packet[i+1] == 'o'){
                hfoc.flash_data.controller.speed_PID_enabled = 0;
                hfoc.flash_data.controller.position_PID_enabled = 0;
                Debug_SendTextResponse("Disabled both speed and position PID\n");
            }
        }
        if(packet[i] == 'K'){
            hfoc.motor_disable_flag = 1;
        }
        if(packet[i] == 'T'){

        }
        if(packet[i] == 'C'){
            hfoc.flash_data.encoder.offset_valid = 0;
            hfoc.flash_data.motor.phase_inductance = 0;
            hfoc.flash_data.motor.phase_resistance = 0;
            FOC_SetState(&hfoc, FOC_STATE_CHECKLIST, FOC_STATE_NONE);
        }
    }

    if(packet[0] == 'P' && packet[1] == 'd'){
        int Pd = 0;
        sscanf(packet, "Pd%d", &Pd);
        hfoc.flash_data.controller.PID_gains_d.Kp = (float)Pd / 1000.0f;
        Debug_SendTextResponse("Set Pd to %dm\n", (int)(hfoc.flash_data.controller.PID_gains_d.Kp * 1000.0f));
    }
    if(packet[0] == 'P' && packet[1] == 'q'){
        int Pq = 0;
        sscanf(packet, "Pq%d", &Pq);
        hfoc.flash_data.controller.PID_gains_q.Kp = (float)Pq / 1000.0f;
        Debug_SendTextResponse("Set Pq to %dm\n", (int)(hfoc.flash_data.controller.PID_gains_q.Kp * 1000.0f));
    }
    if(packet[0] == 'P' && packet[1] == 's'){
        int Ps = 0;
        sscanf(packet, "Ps%d", &Ps);
        hfoc.flash_data.controller.PID_gains_speed.Kp = (float)Ps / 1000.0f;
        Debug_SendTextResponse("Set Ps to %dm\n", (int)(hfoc.flash_data.controller.PID_gains_speed.Kp * 1000.0f));
    }
    if(packet[0] == 'P' && packet[1] == 'p'){
        int Pp = 0;
        sscanf(packet, "Pp%d", &Pp);
        hfoc.flash_data.controller.PID_gains_position.Kp = (float)Pp / 1000.0f;
        Debug_SendTextResponse("Set Pp to %dm\n", (int)(hfoc.flash_data.controller.PID_gains_position.Kp * 1000.0f));
    }


    
    if(packet[0] == 'I' && packet[1] == 'd'){
        int Id = 0;
        sscanf(packet, "Id%d", &Id);
        hfoc.flash_data.controller.PID_gains_d.Ki = (float)Id / 1000.0f;
        Debug_SendTextResponse("Set Id to %dm\n", (int)(hfoc.flash_data.controller.PID_gains_d.Ki * 1000.0f));
    }
    if(packet[0] == 'I' && packet[1] == 'q'){
        int Iq = 0;
        sscanf(packet, "Iq%d", &Iq);
        hfoc.flash_data.controller.PID_gains_q.Ki = (float)Iq / 1000.0f;
        Debug_SendTextResponse("Set Iq to %dm\n", (int)(hfoc.flash_data.controller.PID_gains_q.Ki * 1000.0f));
    }
    if(packet[0] == 'I' && packet[1] == 's'){
        int Is = 0;
        sscanf(packet, "Is%d", &Is);
        hfoc.flash_data.controller.PID_gains_speed.Ki = (float)Is / 1000.0f;
        Debug_SendTextResponse("Set Is to %dm\n", (int)(hfoc.flash_data.controller.PID_gains_speed.Ki * 1000.0f));
    }
    if(packet[0] == 'I' && packet[1] == 'p'){
        int Ip = 0;
        sscanf(packet, "Ip%d", &Ip);
        hfoc.flash_data.controller.PID_gains_position.Ki = (float)Ip / 1000.0f;
        Debug_SendTextResponse("Set Ip to %dm\n", (int)(hfoc.flash_data.controller.PID_gains_position.Ki * 1000.0f));
    }

    if(packet[0] == 'S' && packet[1] == 'q'){
        int Sq = 0;
        sscanf(packet, "Sq%d", &Sq);
        hfoc.dq_current_setpoint.q = (float)Sq / 1000.0f;
        Debug_SendTextResponse("Set Sq to %dmA\n", (int)(hfoc.dq_current_setpoint.q * 1000.0f));
    }
    if(packet[0] == 'S' && packet[1] == 'd'){
        int Sd = 0;
        sscanf(packet, "Sd%d", &Sd);
        hfoc.dq_current_setpoint.d = (float)Sd / 1000.0f;
        Debug_SendTextResponse("Set Sd to %dmA\n", (int)(hfoc.dq_current_setpoint.d * 1000.0f));
    }
    if(packet[0] == 'S' && packet[1] == 's'){
        int Ss = 0;
        sscanf(packet, "Ss%d", &Ss);
        hfoc.speed_setpoint = (float)Ss;
        Debug_SendTextResponse("Set Ss to %dRad/s\n", (int)hfoc.speed_setpoint);
    }
    if(packet[0] == 'S' && packet[1] == 'p'){
        int Sp = 0;
        sscanf(packet, "Sp%d", &Sp);
        hfoc.angle_setpoint = (float)Sp / 1000.0f;
        Debug_SendTextResponse("Set Sp to %dRad\n", (int)(hfoc.angle_setpoint * 1000.0f));
    }
}



static uint8_t Debug_SendBinaryResponse(MsgTypeTypeDef msg_type, uint8_t* payload, uint16_t len){
    if(payload == NULL && len > 0) return 0;

    TxUsbBuf_t* txbuf = USB_AllocTxBuffer();
    if(!txbuf) return 0;

    txbuf->payload[0] = DEBUG_SOF1_BIN;
    txbuf->payload[1] = DEBUG_SOF2_BIN;
    txbuf->payload[2] = (uint8_t)msg_type;
    txbuf->payload[3] = (uint8_t)(len & 0xFF);
    txbuf->payload[4] = (uint8_t)((len >> 8) & 0xFF);
    if(len > sizeof(txbuf->payload) - 5) {
        USB_FreeTxBuffer(txbuf);
        return 0;
    }

    if((len > 0U)){
        memcpy(&txbuf->payload[5], payload, len);
    }
    txbuf->length = len + 5;
    USB_PushTxBuffer(txbuf);
    return 1;
}

uint8_t Debug_SendTextResponse(const char* format, ...){
    va_list args;

    TxUsbBuf_t* txbuf = USB_AllocTxBuffer();
    if(!txbuf) return 0;

    va_start(args, format);
    int len = vsnprintf((char*)&txbuf->payload[5], sizeof(txbuf->payload) - 5, format, args);
    va_end(args);

    if (len < 0) {
        USB_FreeTxBuffer(txbuf);
        return 0;
    }

    if (len >= (int)(sizeof(txbuf->payload) - 5)) {
        USB_FreeTxBuffer(txbuf);
        return 0;
    }

    txbuf->payload[0] = DEBUG_SOF1_BIN;
    txbuf->payload[1] = DEBUG_SOF2_BIN;
    txbuf->payload[2] = (uint8_t)MSG_TEXT_REPLY;
    txbuf->payload[3] = (uint8_t)(len & 0xFF);
    txbuf->payload[4] = (uint8_t)((len >> 8) & 0xFF);

    if((size_t)len > sizeof(txbuf->payload) - 5) {
        USB_FreeTxBuffer(txbuf);
        return 0;
    }

    txbuf->length = len + 5;
    USB_PushTxBuffer(txbuf);
    return 1;
}

void USB_ProcessReceivedPacket(uint8_t* buf, uint16_t len){
    UNUSED(len);
    if (buf[0] == DEBUG_SOF1_BIN || buf[1] == DEBUG_SOF2_BIN){
        MsgTypeTypeDef msg_type = (MsgTypeTypeDef)buf[2];
        uint16_t payload_length = (uint16_t)buf[3] | ((uint16_t)buf[4] << 8);
        uint8_t* payload = &buf[5];
        Debug_ExecuteBinaryCommand(msg_type, payload, payload_length);
    } else {
        Debug_ExecuteTextCommand((const char*)buf, len);
    }
}