#ifndef FOC_USB_DEBUG_H
#define FOC_USB_DEBUG_H

#include "stdint.h"

typedef enum{
	DEBUG_OK,
	DEBUG_STOPPED,
	DEBUG_BUSY,
	DEBUG_ERROR
}Debug_StatusTypeDef;

/**
 * @file
 * @brief USB debug protocol declarations.
 * @see docs/usb-protocol.md for identifiers, payloads, and command behavior.
 */

typedef enum {
    MSG_ENTER_BOOTLOADER = 0x00,
    MSG_GET_DEVICE_INFO = 0x01,
    MSG_DEVICE_INFO_REPLY = 0x02,
    MSG_GET_DEFINITIONS = 0x03,
    MSG_DEFINITIONS_REPLY = 0x04,
    MSG_DEFINITIONS_ERROR = 0x05,
    MSG_LOG_DATA = 0x06,
    MSG_SET_MASK = 0x07,
    MSG_GET_MASK = 0x08,
    MSG_MASK_REPLY = 0x09,
    MSG_START_LOG = 0x0A,
    MSG_STOP_LOG = 0x0B,
    MSG_SET_PID = 0x0C,
    MSG_GET_PID = 0x0D,
    MSG_PID_REPLY = 0x0E,
    MSG_SET_VAR = 0x0F,
    MSG_GET_VAR = 0x10,
    MSG_VAR_REPLY = 0x11,
    MSG_FLASH_SAVE = 0x12,
    MSG_FLASH_LOAD = 0x13,
    MSG_FLASH_CLEAR = 0x14,
    MSG_SET_STATE = 0x15,
    MSG_GET_STATE = 0x16,
    MSG_STATE_REPLY = 0x17,
    MSG_TEXT_COMMAND = 0x18,
    MSG_TEXT_REPLY = 0x19,
    MSG_SET_NODE_ID = 0x1A,
    MSG_GET_NODE_ID = 0x1B,
    MSG_NODE_ID_REPLY = 0x1C,
    MSG_GET_ACTIVE_ERRORS = 0x1D,
    MSG_ACTIVE_ERRORS_REPLY = 0x1E,
    MSG_GET_LATCHED_ERRORS = 0x1F,
    MSG_LATCHED_ERRORS_REPLY = 0x20,
    MSG_CLEAR_LATCHED_ERRORS = 0x21,
    MSG_SET_CAN_CYCLIC_RATE = 0x22,
    MSG_GET_CAN_CYCLIC_RATE = 0x23,
    MSG_CAN_CYCLIC_REPLY = 0x24,
    MSG_SET_CONTROL_MODE = 0x25,
    MSG_GET_CONTROL_MODE = 0x26,
    MSG_CONTROL_MODE_REPLY = 0x27,

    MSG_UNKNOWN_TYPE = 0xFA,
    MSG_INVALID_PAYLOAD = 0xFB,
    MSG_UNKNOWN_ID = 0xFC,
    MSG_BUFFER_OVERFLOW = 0xFD,
    MSG_ACK = 0xFE,
    MSG_ERROR = 0xFF
} MsgTypeTypeDef;


#define DEBUG_PROTOCOL_VERSION 3

Debug_StatusTypeDef FOC_USB_Setup();
Debug_StatusTypeDef FOC_USB_Debug_CaptureSamples();
uint8_t Debug_SendTextResponse(const char* format, ...) __attribute__((format(printf, 1, 2)));


#endif // FOC_USB_DEBUG_H
