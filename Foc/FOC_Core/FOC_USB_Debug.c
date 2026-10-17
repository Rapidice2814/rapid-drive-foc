#include <string.h>
#include <stdarg.h>
#include <stdio.h>

#include "FOC_USB_Debug.h"
#include "FOC_Handle.h"
#include "FOC_States.h"
#include "FOC_USB.h"
#include "Timing.h"
#include "FOC_Config.h"
#include "FOC_CAN.h"
#include "FOC_Diagnostics.h"

uint32_t usb_debug_times[5] = {0};

typedef union{
    float    f;
    int32_t  i32;
    uint32_t u32;
} TypeConvU_t;

typedef struct{
    uint8_t is_running;
    uint8_t signal_mask[SIGNAL_MASK_BYTES];
    uint32_t timestamp;
    uint16_t sample_count;
    uint16_t signal_count;
    TxUsbBuf_t* txbuf;
} LogDataHandleTypeDef;

extern FOC_HandleTypeDef hfoc;

static uint8_t Debug_SendBinaryResponse(MsgTypeTypeDef msg_type, uint8_t* payload, uint16_t len);
static void Debug_ExecuteTextCommand(const char *packet, uint16_t length);

static Debug_StatusTypeDef Debug_UpdateMask(const uint8_t *new_mask);
static uint8_t* Debug_GetMask();
static Debug_StatusTypeDef Debug_ClearMask();
static void Debug_StartLogging();
static void Debug_StopLogging();

static LogDataHandleTypeDef hlogdata = {0};


Debug_StatusTypeDef FOC_USB_Setup(){
    Debug_StopLogging();
    Debug_ClearMask();
    return DEBUG_OK;
}

/**
 * @brief Captures samples for the selected signals and stores them in the debug handle buffer. 
 *        Once enough samples are collected, it packages them into a USB packet and adds it to the transmission buffer.
 *        Ifthe buffer is full, the captured samples will be discarded until there is space in the buffer.
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

    #define SIGNAL_MASK_BIT_IS_SET(bit) \
        (((bit) < (SIGNAL_MASK_BYTES * 8u)) && \
        ((hlogdata.signal_mask[(bit) / 8u] & (1u << ((bit) & 7u))) != 0u))

    #define CAPTURE_SIGNAL(bit, member, field)                                  \
        do {                                                                    \
            if(SIGNAL_MASK_BIT_IS_SET(bit)) {                                  \
                conv.member = hfoc.field;                                       \
                write_u32_le(&hlogdata.txbuf->payload[write_index + signal_index * 4u], conv.u32); \
                signal_index++;                                                 \
            }                                                                   \
        } while (0);

    FOC_USB_DEBUG_SIGNAL_LIST(CAPTURE_SIGNAL);
    #undef CAPTURE_SIGNAL

    #undef SIGNAL_MASK_BIT_IS_SET

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

static Debug_StatusTypeDef Debug_UpdateMask(const uint8_t *new_mask){
    if(hlogdata.is_running){
        return DEBUG_ERROR;
    }

    uint8_t set_bits = countbits_array(new_mask, SIGNAL_MASK_BYTES);
    if(set_bits > MAX_LOGDATA_SIGNAL_COUNT){
        return DEBUG_ERROR;
    }

    for (uint8_t i = 0; i < SIGNAL_MASK_BYTES; i++){
        hlogdata.signal_mask[i] = new_mask[i];
    }

    hlogdata.signal_count = set_bits;

    return DEBUG_OK;
}

static uint8_t* Debug_GetMask(){
    return hlogdata.signal_mask;
}

static Debug_StatusTypeDef Debug_ClearMask(){
    const uint8_t zero_mask[SIGNAL_MASK_BYTES] = {0};
    return Debug_UpdateMask(zero_mask);
}

static void Debug_ExecuteBinaryCommand(MsgTypeTypeDef msg_type, uint8_t* payload, uint16_t payload_length){
    uint8_t controller_id;
    uint8_t var_id;
    uint8_t response_payload[13];

    switch ((MsgTypeTypeDef)(msg_type)){
        
    case MSG_GET_VERSION:
        if(payload_length != 0){
            Debug_SendBinaryResponse(MSG_INVALID_PAYLOAD, NULL, 0);
            break;
        }
        response_payload[0] = FOC_VERSION_MAJOR;
        response_payload[1] = FOC_VERSION_MINOR;
        response_payload[2] = FOC_VERSION_PATCH;
        Debug_SendBinaryResponse(MSG_VERSION_REPLY, response_payload, 3);
        break;

    case MSG_SET_MASK:
        if(payload_length != SIGNAL_MASK_BYTES){
            Debug_SendBinaryResponse(MSG_INVALID_PAYLOAD, NULL, 0);
            break;
        }
        uint8_t new_mask[SIGNAL_MASK_BYTES];
        memcpy(new_mask, payload, SIGNAL_MASK_BYTES);
        if(Debug_UpdateMask(new_mask) != DEBUG_OK){
            Debug_SendBinaryResponse(MSG_ERROR, NULL, 0);
            break;
        }
        Debug_SendBinaryResponse(MSG_ACK, NULL, 0);
        break;

    case MSG_GET_MASK:
        if(payload_length != 0){
            Debug_SendBinaryResponse(MSG_INVALID_PAYLOAD, NULL, 0);
            break;
        }
        uint8_t* current_mask = Debug_GetMask();
        Debug_SendBinaryResponse(MSG_MASK_REPLY, current_mask, SIGNAL_MASK_BYTES);
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
        if(payload_length != 0){
            Debug_SendBinaryResponse(MSG_INVALID_PAYLOAD, NULL, 0);
            break;
        }
        if(FOC_SetState(&hfoc, FOC_STATE_FLASH_SAVE, FOC_STATE_RUN) != FOC_STATETRANSITION_OK){
            Debug_SendBinaryResponse(MSG_ERROR, NULL, 0);
            break;
        }
        Debug_SendBinaryResponse(MSG_ACK, NULL, 0);
        break;

    case MSG_FLASH_LOAD:
        if(payload_length != 0){
            Debug_SendBinaryResponse(MSG_INVALID_PAYLOAD, NULL, 0);
            break;
        }
        if(FOC_SetState(&hfoc, FOC_STATE_FLASH_LOAD, FOC_STATE_RUN) != FOC_STATETRANSITION_OK){
            Debug_SendBinaryResponse(MSG_ERROR, NULL, 0);
            break;
        }
        Debug_SendBinaryResponse(MSG_ACK, NULL, 0);
        break;

    case MSG_FLASH_CLEAR:
        if(payload_length != 0){
            Debug_SendBinaryResponse(MSG_INVALID_PAYLOAD, NULL, 0);
            break;
        }
        if(FOC_SetState(&hfoc, FOC_STATE_FLASH_CLEAR, FOC_STATE_RUN) != FOC_STATETRANSITION_OK){
            Debug_SendBinaryResponse(MSG_ERROR, NULL, 0);
            break;
        }
        Debug_SendBinaryResponse(MSG_ACK, NULL, 0);
        break;

    case MSG_SET_STATE:
        if(payload_length != 1){
            Debug_SendBinaryResponse(MSG_INVALID_PAYLOAD, NULL, 0);
            break;
        }
        if(FOC_SetState(&hfoc, (FOC_StateTypeDef)payload[0], FOC_STATE_NONE) != FOC_STATETRANSITION_OK){
            Debug_SendBinaryResponse(MSG_ERROR, NULL, 0);
            break;
        }
        Debug_SendBinaryResponse(MSG_ACK, NULL, 0);
        break;

    case MSG_GET_STATE:
        if(payload_length != 0){
            Debug_SendBinaryResponse(MSG_INVALID_PAYLOAD, NULL, 0);
            break;
        }
        response_payload[0] = (uint8_t)FOC_GetState(&hfoc);
        Debug_SendBinaryResponse(MSG_STATE_REPLY, response_payload, 1);
        break;
    case MSG_TEXT_COMMAND:
        Debug_ExecuteTextCommand((const char*)payload, payload_length);
        break;
    case MSG_ENTER_BOOTLOADER:
        if(payload_length != 0){
            Debug_SendBinaryResponse(MSG_INVALID_PAYLOAD, NULL, 0);
            break;
        }
        if(FOC_SetState(&hfoc, FOC_STATE_BOOTLOADER, FOC_STATE_NONE) != FOC_STATETRANSITION_OK){
            Debug_SendBinaryResponse(MSG_ERROR, NULL, 0);
            break;
        }
        Debug_SendBinaryResponse(MSG_ACK, NULL, 0);
        break;
    
    case MSG_SET_NODE_ID:
        if(payload_length != 1){
            Debug_SendBinaryResponse(MSG_INVALID_PAYLOAD, NULL, 0);
            break;
        }
        FOC_SetNodeId(&hfoc, payload[0]);
        Debug_SendBinaryResponse(MSG_ACK, NULL, 0);
        break;

    case MSG_GET_NODE_ID:
        if(payload_length != 0){
            Debug_SendBinaryResponse(MSG_INVALID_PAYLOAD, NULL, 0);
            break;
        }
        response_payload[0] = FOC_GetNodeId(&hfoc);
        Debug_SendBinaryResponse(MSG_NODE_ID_REPLY, response_payload, 1);
        break;

    case MSG_GET_ACTIVE_ERRORS:
        if(payload_length != 0){
            Debug_SendBinaryResponse(MSG_INVALID_PAYLOAD, NULL, 0);
            break;
        }
        write_u32_le(&response_payload[0], FOC_GetActiveErrors(&hfoc));
        Debug_SendBinaryResponse(MSG_ACTIVE_ERRORS_REPLY, response_payload, 4);
        break;
    
    case MSG_GET_LATCHED_ERRORS:
        if(payload_length != 0){
            Debug_SendBinaryResponse(MSG_INVALID_PAYLOAD, NULL, 0);
            break;
        }
        write_u32_le(&response_payload[0], FOC_GetLatchedErrors(&hfoc));
        Debug_SendBinaryResponse(MSG_LATCHED_ERRORS_REPLY, response_payload, 4);
        break;

    case MSG_CLEAR_LATCHED_ERRORS:
        if(payload_length != 0){
            Debug_SendBinaryResponse(MSG_INVALID_PAYLOAD, NULL, 0);
            break;
        }
        FOC_ClearLatchedErrors(&hfoc);
        Debug_SendBinaryResponse(MSG_ACK, NULL, 0);
        break;

    case MSG_SET_CAN_CYCLIC_RATE:
        if(payload_length != 5){
            Debug_SendBinaryResponse(MSG_INVALID_PAYLOAD, NULL, 0);
            break;
        }
        uint32_t new_rate = read_u32_le(&payload[1]);
        FOC_SetCyclicRate(&hfoc, (CAN_CyclicIndexTypeDef)payload[0], new_rate);
        Debug_SendBinaryResponse(MSG_ACK, NULL, 0);
        break;

    case MSG_GET_CAN_CYCLIC_RATE:
        if(payload_length != 1){
            Debug_SendBinaryResponse(MSG_INVALID_PAYLOAD, NULL, 0);
            break;
        }
        response_payload[0] = payload[0];
        uint32_t current_rate = FOC_GetCyclicRate(&hfoc, (CAN_CyclicIndexTypeDef)payload[0]);
        write_u32_le(&response_payload[1], current_rate);
        Debug_SendBinaryResponse(MSG_CAN_CYCLIC_REPLY, response_payload, 5);
        break;
    
    case MSG_SET_CONTROL_MODE:
        if(payload_length != 1){
            Debug_SendBinaryResponse(MSG_INVALID_PAYLOAD, NULL, 0);
            break;
        }
        ControlModeTypeDef new_mode = (ControlModeTypeDef)payload[0];
        if(FOC_SetControlMode(&hfoc, new_mode) != 0){
            Debug_SendBinaryResponse(MSG_INVALID_PAYLOAD, NULL, 0);
            break;
        }
        Debug_SendBinaryResponse(MSG_ACK, NULL, 0);
        break;
    
    case MSG_GET_CONTROL_MODE:
        if(payload_length != 0){
            Debug_SendBinaryResponse(MSG_INVALID_PAYLOAD, NULL, 0);
            break;
        }
        response_payload[0] = (uint8_t)FOC_GetControlMode(&hfoc);
        Debug_SendBinaryResponse(MSG_CONTROL_MODE_REPLY, response_payload, 1);
        break;
    

    default:
        Debug_SendBinaryResponse(MSG_UNKNOWN_TYPE, NULL, 0);
        break;
    }
}

static void Debug_ExecuteTextCommand(const char *packet, uint16_t length){
    UNUSED(length);
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
        if(packet[i] == 'L'){
            FOC_SetState(&hfoc, FOC_STATE_FLASH_CLEAR, FOC_STATE_RUN);
        }
        if(packet[i] == 'B'){
            FOC_SetState(&hfoc, FOC_STATE_BOOTLOADER, FOC_STATE_NONE);
        }
        if(packet[i] == 'M'){
            if(packet[i+1] == 's'){
                FOC_SetControlMode(&hfoc, CONTROL_MODE_SPEED);
            } else if(packet[i+1] == 'p'){
                FOC_SetControlMode(&hfoc, CONTROL_MODE_POSITION);
            } else if(packet[i+1] == 'o'){
                FOC_SetControlMode(&hfoc, CONTROL_MODE_OPENLOOP);
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

    if(len < 0) {
        USB_FreeTxBuffer(txbuf);
        return 0;
    }

    if(len >= (int)(sizeof(txbuf->payload) - 5)) {
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
    if(buf[0] == DEBUG_SOF1_BIN || buf[1] == DEBUG_SOF2_BIN){
        MsgTypeTypeDef msg_type = (MsgTypeTypeDef)buf[2];
        uint16_t payload_length = (uint16_t)buf[3] | ((uint16_t)buf[4] << 8);
        uint8_t* payload = &buf[5];
        Debug_ExecuteBinaryCommand(msg_type, payload, payload_length);
    } else {
        Debug_ExecuteTextCommand((const char*)buf, len);
    }
}