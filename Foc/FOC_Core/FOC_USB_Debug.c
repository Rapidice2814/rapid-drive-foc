#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <stddef.h>

#include "FOC_USB_Debug.h"
#include "FOC_Handle.h"
#include "FOC_States.h"
#include "FOC_USB.h"
#include "Timing.h"
#include "FOC_Config.h"
#include "FOC_CAN.h"
#include "FOC_Diagnostics.h"


/* Single source of truth. Array order supplies all wire IDs and mask bits. */
#define FOC_USB_DEBUG_SIGNAL_LIST(X)            \
    X(u32,   tick)                      		\
    X(f,     adc_values.motor_temp)          \
    X(f,     adc_values.mosfet_temp)         \
    X(f,     adc_values.vbus)                \
    X(f,     ibus)                           \
    X(f,     adc_values.phase_current.a)     \
    X(f,     adc_values.phase_current.b)     \
    X(f,     adc_values.phase_current.c)     \
    X(f,     ab_current.alpha)               \
    X(f,     ab_current.beta)                \
    X(f,    dq_current.d)                   \
    X(f,    dq_current.q)                   \
    X(f,    dq_current_filtered.d)          \
    X(f,    dq_current_filtered.q)          \
    X(f,    phase_voltage.a)                \
    X(f,    phase_voltage.b)                \
    X(f,    phase_voltage.c)                \
    X(f,    ab_voltage.alpha)               \
    X(f,    ab_voltage.beta)                \
    X(f,    dq_voltage.d)                   \
    X(f,    dq_voltage.q)                   \
    X(f,    encoder_angle_mechanical_wrapped) \
    X(f,    encoder_angle_mechanical_unwrapped) \
    X(f,    encoder_speed_mechanical)       \
    X(f,    encoder_angle_electrical)       \
    X(f,    encoder_speed_electrical)       \
    X(f,    dq_current_setpoint.d)          \
    X(f,    dq_current_setpoint.q)          \
    X(f,    angle_setpoint)                 \
    X(f,    speed_setpoint)                 \
    X(u32,  execution_time.loop_max)        \
    X(f,  hfi.injection_phase)              \
    X(f,  hfi.i_alpha_l_raw)                \
    X(f,  hfi.i_beta_l_raw)                 \
    X(f,  hfi.i_alpha_l_filtered)           \
    X(f,  hfi.i_beta_l_filtered)            \

#define FOC_PID_CONTROLLERS_LIST(X)             \
    X(pid_current_d)                         \
    X(pid_current_q)                         \
    X(pid_speed)                             \
    X(pid_position)                          \

#define VAR_ID_LIST(X)                         \
    X(f, dq_current_setpoint.d)             \
    X(f, dq_current_setpoint.q)             \
    X(f, angle_setpoint)                    \
    X(f, speed_setpoint)                    \
    X(f, flash_data.limits.max_dq_current)  \
    X(f, flash_data.limits.max_dq_voltage)  \
    X(f, flash_data.limits.vbus_overvoltage_trip_level) \
    X(f, flash_data.limits.vbus_undervoltage_trip_level) \
    X(f, flash_data.limits.ibus_overcurrent_trip_level) \
    X(f, flash_data.limits.motor_temp_trip_level) \
    X(f, flash_data.limits.mosfet_temp_trip_level) \
    X(u32, flash_data.motor.pole_pairs) \
    X(f, flash_data.motor.phase_resistance) \
    X(f, flash_data.motor.phase_inductance) \
    X(f, flash_data.motor.torque_constant) \
    X(u32, flash_data.hfi.hfi_enabled) \
    X(f, flash_data.hfi.injection_amplitude) \
    X(f, flash_data.hfi.injection_omega) \
    X(f, flash_data.controller.current_control_bandwidth) \


#define COUNT_VALUE(type, field) + 1
#define COUNT_PID(field) + 1
enum {
    SIGNAL_COUNT = 0 FOC_USB_DEBUG_SIGNAL_LIST(COUNT_VALUE),
    VAR_COUNT = 0 VAR_ID_LIST(COUNT_VALUE),
    PID_COUNT = 0 FOC_PID_CONTROLLERS_LIST(COUNT_PID)
};
#undef COUNT_VALUE
#undef COUNT_PID
#define MAX_SUPPORTED_SIGNALS 256u
#define SIGNAL_MASK_BYTES ((SIGNAL_COUNT + 7u) / 8u)
#define DEFINITION_PAGE_BYTES 256u

typedef struct {
    size_t offset;
    const char *type;
    const char *name;
    size_t size;
    uint8_t is_signed;
} DebugDescriptor;

#define VALUE_DESCRIPTOR(type, field) {offsetof(FOC_HandleTypeDef, field), #type, #field, sizeof(((FOC_HandleTypeDef*)0)->field), _Generic(((FOC_HandleTypeDef*)0)->field, int8_t: 1, int16_t: 1, int32_t: 1, default: 0)},
#define PID_DESCRIPTOR(field) {offsetof(FOC_HandleTypeDef, field), "", #field, sizeof(PIDControllerTypeDef), 0},
static const DebugDescriptor signal_definitions[] = {FOC_USB_DEBUG_SIGNAL_LIST(VALUE_DESCRIPTOR) {0, NULL, NULL, 0, 0}};
static const DebugDescriptor variable_definitions[] = {VAR_ID_LIST(VALUE_DESCRIPTOR) {0, NULL, NULL, 0, 0}};
static const DebugDescriptor pid_definitions[] = {FOC_PID_CONTROLLERS_LIST(PID_DESCRIPTOR) {0, NULL, NULL, 0, 0}};
#undef VALUE_DESCRIPTOR
#undef PID_DESCRIPTOR

#define VALID_TYPE_f(field) _Generic((field), float: 1, default: 0)
#define VALID_TYPE_u32(field) _Generic((field), uint8_t: 1, uint16_t: 1, uint32_t: 1, default: 0)
#define VALID_TYPE_i32(field) _Generic((field), int8_t: 1, int16_t: 1, int32_t: 1, default: 0)
#define ASSERT_VALUE(type, field) \
    _Static_assert(VALID_TYPE_##type(((FOC_HandleTypeDef*)0)->field), "Unsupported debug storage/wire type"); \
    _Static_assert(sizeof(#field) <= 128, "Debug name too long");
FOC_USB_DEBUG_SIGNAL_LIST(ASSERT_VALUE)
VAR_ID_LIST(ASSERT_VALUE)
#undef ASSERT_VALUE
#define ASSERT_PID(field) \
    _Static_assert(_Generic(&((FOC_HandleTypeDef*)0)->field, PIDControllerTypeDef*: 1, default: 0), "PID type mismatch"); \
    _Static_assert(sizeof(#field) <= 128, "PID name too long");
FOC_PID_CONTROLLERS_LIST(ASSERT_PID)
#undef ASSERT_PID
_Static_assert(SIGNAL_COUNT > 0 && SIGNAL_COUNT <= MAX_SUPPORTED_SIGNALS, "Signal count exceeds mask capacity");
_Static_assert(VAR_COUNT <= 256 && PID_COUNT <= 256, "Definition IDs exceed one-byte wire format");
_Static_assert(MAX_LOGDATA_SIGNAL_COUNT > 0 && MAX_LOGDATA_SIGNAL_COUNT <= SIGNAL_COUNT && MAX_LOGDATA_SIGNAL_COUNT <= 255, "Invalid selected-signal limit");
_Static_assert(SIGNAL_MASK_BYTES <= RX_USB_BUF_SIZE - 5, "Mask exceeds command buffer");

static uint64_t definitions_fingerprint;
static uint8_t definitions_valid;

/**
 * @brief Reads a scalar field as its four-byte wire representation.
 * @param entry Descriptor containing the field offset, native width, and signedness.
 * @param handle Base address of the FOC handle containing the field.
 * @return Raw value bits, with smaller signed integers sign-extended.
 * @note The STM32 target stores native scalar bytes in little-endian order.
 */
static uint32_t Debug_ReadRaw(const DebugDescriptor *entry, const void *handle){
    uint32_t raw = 0;
    memcpy(&raw, (const uint8_t*)handle + entry->offset, entry->size);
    if(entry->is_signed && entry->size < 4 && (raw & (1u << (entry->size * 8u - 1u)))){
        raw |= UINT32_MAX << (entry->size * 8u);
    }
    return raw;
}

/**
 * @brief Writes a wire value to its native field after checking integer range.
 * @param entry Descriptor of the destination field.
 * @param handle Base address of the FOC handle to update.
 * @param raw Four-byte wire representation of the requested value.
 * @return 1 if written, or 0 if the value cannot fit a smaller integer field.
 */
static uint8_t Debug_WriteRaw(const DebugDescriptor *entry, void *handle, uint32_t raw){
    if(entry->size < 4){
        uint32_t bits = entry->size * 8u;
        if(entry->is_signed){
            int32_t value;
            memcpy(&value, &raw, sizeof(value));
            int32_t limit = (int32_t)(1u << (bits - 1u));
            if(value < -limit || value >= limit) return 0;
        }else if(raw >= (1u << bits)) return 0;
    }
    memcpy((uint8_t*)handle + entry->offset, &raw, entry->size);
    return 1;
}

/**
 * @brief Selects the descriptor table and count for a discovery list.
 * @param kind List selector: 0 signals, 1 variables, or 2 PID controllers.
 * @param count Receives the entry count, or zero for an unknown kind.
 * @return Descriptor table, or NULL for an unknown kind.
 */
static const DebugDescriptor *Debug_Definitions(uint8_t kind, uint16_t *count){
    switch(kind){
    case 0: *count = SIGNAL_COUNT; return signal_definitions;
    case 1: *count = VAR_COUNT; return variable_definitions;
    case 2: *count = PID_COUNT; return pid_definitions;
    default: *count = 0; return NULL;
    }
}

/**
 * @brief Formats one canonical, newline-terminated discovery row.
 * @param out Destination text buffer.
 * @param capacity Destination capacity, including the terminating NUL.
 * @param kind List selector; PID rows omit the primitive type.
 * @param index Automatically assigned list index.
 * @param entry Descriptor whose name and type are serialized.
 * @return snprintf result: required characters excluding NUL, or a negative error.
 * @note Discovery replies and the fingerprint use this same format.
 */
static int Debug_FormatDefinition(char *out, size_t capacity, uint8_t kind, uint16_t index,
                                  const DebugDescriptor *entry){
    return kind == 2 ? snprintf(out, capacity, "%u\t%s\n", (unsigned)index, entry->name)
                     : snprintf(out, capacity, "%u\t%s\t%s\n", (unsigned)index, entry->type, entry->name);
}

/**
 * @brief Adds text bytes to the running FNV-1a definitions fingerprint.
 * @param text NUL-terminated text; the terminating NUL is not hashed.
 */
static void Debug_HashText(const char *text){
    while(*text){
        definitions_fingerprint ^= (uint8_t)*text++;
        definitions_fingerprint *= UINT64_C(1099511628211);
    }
}

/**
 * @brief Builds the fingerprint and detects duplicate names within each list.
 * @note Called during USB setup; invalid definitions prevent configuration and telemetry.
 */
static void Debug_InitDefinitions(void){
    static const char *const headings[] = {"signals\n", "variables\n", "pids\n"};
    definitions_fingerprint = UINT64_C(14695981039346656037);
    definitions_valid = 1;
    char line[160];
    for(uint8_t kind = 0; kind < 3; ++kind){
        uint16_t count;
        const DebugDescriptor *entries = Debug_Definitions(kind, &count);
        Debug_HashText(headings[kind]);
        for(uint16_t i = 0; i < count; ++i){
            for(uint16_t j = 0; j < i; ++j){
                if(strcmp(entries[i].name, entries[j].name) == 0) definitions_valid = 0;
            }
            Debug_FormatDefinition(line, sizeof(line), kind, i, &entries[i]);
            Debug_HashText(line);
        }
    }
}

_Static_assert(13u + MAX_LOGDATA_SAMPLE_COUNT * MAX_LOGDATA_SIGNAL_COUNT * 4u <= TX_USB_BUF_SIZE,
               "Telemetry packet exceeds the USB transmit buffer");

uint32_t usb_debug_times[5] = {0};

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


/**
 * @brief Initializes discovery metadata and resets USB telemetry capture.
 * @return DEBUG_OK when names are unique, otherwise DEBUG_ERROR.
 * @note Stops capture, releases a partial sample buffer, and clears the signal mask.
 */
Debug_StatusTypeDef FOC_USB_Setup(){
    Debug_InitDefinitions();
    Debug_StopLogging();
    Debug_ClearMask();
    return definitions_valid ? DEBUG_OK : DEBUG_ERROR;
}

/**
 * @brief Captures one selected-signal sample and queues a full telemetry packet.
 * @return DEBUG_STOPPED if disabled, DEBUG_ERROR if definitions are invalid or no
 *         transmit buffer is available, otherwise DEBUG_OK.
 * @note Called once per completed current-loop sample. Samples are skipped without
 *       a free buffer; a completed packet is discarded if enqueueing fails.
 */
Debug_StatusTypeDef FOC_USB_Debug_CaptureSamples(void){
    uint32_t start_time = get_current_time();
    if(!definitions_valid) return DEBUG_ERROR;
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
    for(uint16_t bit = 0; bit < SIGNAL_COUNT; ++bit){
        if(hlogdata.signal_mask[bit / 8u] & (1u << (bit % 8u))){
            uint32_t raw = Debug_ReadRaw(&signal_definitions[bit], &hfoc);
            write_u32_le(&hlogdata.txbuf->payload[write_index + signal_index * 4u], raw);
            ++signal_index;
        }
    }

    hlogdata.sample_count++;
    write_u16_le(&hlogdata.txbuf->payload[9], hlogdata.sample_count);
    
    hlogdata.timestamp++;
    
    if(hlogdata.sample_count >= MAX_LOGDATA_SAMPLE_COUNT){
        uint16_t payload_bytes = (uint16_t)(8u + hlogdata.sample_count * hlogdata.signal_count * 4u);

        hlogdata.txbuf->payload[3] = (uint8_t)(payload_bytes & 0xFFu);
        hlogdata.txbuf->payload[4] = (uint8_t)((payload_bytes >> 8) & 0xFFu);
        hlogdata.txbuf->length = (uint16_t)(payload_bytes + 5u);

        if(!USB_PushTxBuffer(hlogdata.txbuf)) USB_FreeTxBuffer(hlogdata.txbuf);
        hlogdata.txbuf = NULL;
        hlogdata.sample_count = 0;
    }

    calculate_execution_time(&usb_debug_times[0], start_time);
    return DEBUG_OK;
}

/**
 * @brief Enables telemetry capture with a fresh sample count.
 * @note The caller must stop logging first to discard any partial packet.
 */
static void Debug_StartLogging(){
    hlogdata.sample_count = 0;
    hlogdata.is_running = 1;
}

/**
 * @brief Disables telemetry capture and releases any partially filled transmit buffer.

 */
static void Debug_StopLogging(){
    hlogdata.is_running = 0;
    if(hlogdata.txbuf != NULL){
        USB_FreeTxBuffer(hlogdata.txbuf);
        hlogdata.txbuf = NULL;
    }
    hlogdata.sample_count = 0;
}

/**
 * @brief Validates and applies a new signal mask while logging is stopped.
 * @param new_mask SIGNAL_MASK_BYTES bytes with unused high bits cleared.
 * @return DEBUG_OK on success, or DEBUG_ERROR if logging is active, unsupported
 *         bits are selected, or the simultaneous-signal limit is exceeded.
 */
static Debug_StatusTypeDef Debug_UpdateMask(const uint8_t *new_mask){
    if(hlogdata.is_running){
        return DEBUG_ERROR;
    }

    for(uint16_t bit = SIGNAL_COUNT; bit < SIGNAL_MASK_BYTES * 8u; ++bit){
        if(new_mask[bit / 8u] & (1u << (bit % 8u))) return DEBUG_ERROR;
    }
    uint16_t set_bits = 0;
    for(uint16_t bit = 0; bit < SIGNAL_COUNT; ++bit){
        if(new_mask[bit / 8u] & (1u << (bit % 8u))) ++set_bits;
    }
    if(set_bits > MAX_LOGDATA_SIGNAL_COUNT){
        return DEBUG_ERROR;
    }

    for (uint8_t i = 0; i < SIGNAL_MASK_BYTES; i++){
        hlogdata.signal_mask[i] = new_mask[i];
    }

    hlogdata.signal_count = set_bits;

    return DEBUG_OK;
}

/**
 * @brief Returns the current internal signal-mask buffer.
 * @return Pointer to SIGNAL_MASK_BYTES bytes owned by the logging state.
 */
static uint8_t* Debug_GetMask(){
    return hlogdata.signal_mask;
}

/**
 * @brief Disables every signal by applying an all-zero mask.
 * @return Result of Debug_UpdateMask; logging must already be stopped.
 */
static Debug_StatusTypeDef Debug_ClearMask(){
    const uint8_t zero_mask[SIGNAL_MASK_BYTES] = {0};
    return Debug_UpdateMask(zero_mask);
}

/**
 * @brief Validates and executes one complete binary USB command.
 * @param msg_type Command identifier decoded from the frame header.
 * @param payload Complete command payload, which may be empty.
 * @param payload_length Number of available payload bytes.
 * @note Queues a reply or error. A state-changing ACK means the request was accepted,
 *       not that a subsequent flash or bootloader action has completed.
 * @see docs/usb-protocol.md
 */
static void Debug_ExecuteBinaryCommand(MsgTypeTypeDef msg_type, uint8_t* payload, uint16_t payload_length){
    uint8_t controller_id;
    uint8_t var_id;
    uint8_t response_payload[29];

    if(!definitions_valid && msg_type != MSG_GET_DEVICE_INFO &&
       msg_type != MSG_GET_DEFINITIONS && msg_type != MSG_STOP_LOG){
        Debug_SendBinaryResponse(MSG_ERROR, NULL, 0);
        return;
    }
    switch ((MsgTypeTypeDef)(msg_type)){
    case MSG_GET_DEVICE_INFO:
        if(payload_length != 0){
            Debug_SendBinaryResponse(MSG_INVALID_PAYLOAD, NULL, 0);
            return;
        }
        response_payload[0] = DEBUG_PROTOCOL_VERSION;
        response_payload[1] = FOC_VERSION_MAJOR;
        response_payload[2] = FOC_VERSION_MINOR;
        response_payload[3] = FOC_VERSION_PATCH;
        write_u16_le(&response_payload[4], SIGNAL_COUNT);
        write_u16_le(&response_payload[6], VAR_COUNT);
        write_u16_le(&response_payload[8], PID_COUNT);
        response_payload[10] = SIGNAL_MASK_BYTES;
        response_payload[11] = MAX_LOGDATA_SIGNAL_COUNT;
        write_u32_le(&response_payload[12], (uint32_t)CURRENT_LOOP_FREQUENCY);
        write_u16_le(&response_payload[16], RX_USB_BUF_SIZE - 5);
        write_u16_le(&response_payload[18], TX_USB_BUF_SIZE - 5);
        write_u32_le(&response_payload[20], (uint32_t)definitions_fingerprint);
        write_u32_le(&response_payload[24], (uint32_t)(definitions_fingerprint >> 32));
        response_payload[28] = definitions_valid ? 0 : 1;
        Debug_SendBinaryResponse(MSG_DEVICE_INFO_REPLY, response_payload, 29);
        return;

    case MSG_GET_DEFINITIONS: {
        if(payload_length != 3){
            Debug_SendBinaryResponse(MSG_INVALID_PAYLOAD, NULL, 0);
            return;
        }
        uint8_t kind = payload[0];
        uint16_t index = read_u16_le(&payload[1]), count;
        const DebugDescriptor *entries = Debug_Definitions(kind, &count);
        uint8_t error = !entries ? 1 : !definitions_valid ? 3 :
                        (index >= count && !(count == 0 && index == 0)) ? 2 : 0;
        if(error){
            uint8_t reply[] = {kind, payload[1], payload[2], error};
            Debug_SendBinaryResponse(MSG_DEFINITIONS_ERROR, reply, sizeof(reply));
            return;
        }
        uint8_t page[DEFINITION_PAGE_BYTES];
        uint16_t used = 7, returned = 0;
        char line[160];
        for(uint16_t i = index; i < count; ++i){
            int n = Debug_FormatDefinition(line, sizeof(line), kind, i, &entries[i]);
            if(n < 0 || (size_t)used + (size_t)n > sizeof(page)) break;
            memcpy(&page[used], line, (size_t)n);
            used += (uint16_t)n;
            ++returned;
        }
        page[0] = kind;
        write_u16_le(&page[1], index);
        write_u16_le(&page[3], count);
        write_u16_le(&page[5], returned);
        Debug_SendBinaryResponse(MSG_DEFINITIONS_REPLY, page, used);
        return;
    }

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
        if(payload_length != 0){
            Debug_SendBinaryResponse(MSG_INVALID_PAYLOAD, NULL, 0);
            break;
        }
        Debug_StopLogging();
        Debug_StartLogging();
        Debug_SendBinaryResponse(MSG_ACK, NULL, 0);
        break;

    case MSG_STOP_LOG:
        if(payload_length != 0){
            Debug_SendBinaryResponse(MSG_INVALID_PAYLOAD, NULL, 0);
            break;
        }
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
        if(controller_id >= PID_COUNT){
            Debug_SendBinaryResponse(MSG_UNKNOWN_ID, NULL, 0);
            return;
        }
        PID_SetGains((PIDControllerTypeDef*)((uint8_t*)&hfoc + pid_definitions[controller_id].offset), new_gains);
        Debug_SendBinaryResponse(MSG_ACK, NULL, 0);
        break;

    case MSG_GET_PID:
        if(payload_length != 1){
            Debug_SendBinaryResponse(MSG_INVALID_PAYLOAD, NULL, 0);
            break;
        }
        controller_id = payload[0];
        PIDValuesTypeDef current_gains;
        if(controller_id >= PID_COUNT){
            Debug_SendBinaryResponse(MSG_UNKNOWN_ID, NULL, 0);
            return;
        }
        current_gains = PID_GetGains((PIDControllerTypeDef*)((uint8_t*)&hfoc + pid_definitions[controller_id].offset));
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
        if(var_id >= VAR_COUNT){
            Debug_SendBinaryResponse(MSG_UNKNOWN_ID, NULL, 0);
            return;
        }
        uint32_t raw = read_u32_le(&payload[1]);
        if(!Debug_WriteRaw(&variable_definitions[var_id], &hfoc, raw)){
            Debug_SendBinaryResponse(MSG_INVALID_PAYLOAD, NULL, 0);
            return;
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

        if(var_id >= VAR_COUNT){
            Debug_SendBinaryResponse(MSG_UNKNOWN_ID, NULL, 0);
            return;
        }
        uint32_t current = Debug_ReadRaw(&variable_definitions[var_id], &hfoc);
        write_u32_le(&response_payload[1], current);
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
        Debug_SendBinaryResponse(MSG_ACK, NULL, 0);
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

/**
 * @brief Executes a bounded legacy text command against the global FOC handle.
 * @param packet Command bytes, which need not be NUL-terminated.
 * @param length Byte count; commands larger than RX_USB_BUF_SIZE are ignored.
 * @note Copies input to a terminated local buffer before sscanf or look-ahead access.
 *       Unknown text is ignored; the caller acknowledges framed text commands.
 */
static void Debug_ExecuteTextCommand(const char *packet, uint16_t length){
    /* Legacy commands use sscanf and look ahead by one byte. */
    char terminated[RX_USB_BUF_SIZE + 2] = {0};
    if(length > RX_USB_BUF_SIZE) return;
    memcpy(terminated, packet, length);
    packet = terminated;
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



/**
 * @brief Builds and submits one framed binary USB response.
 * @param msg_type Response identifier.
 * @param payload Payload bytes, or NULL when len is zero.
 * @param len Number of payload bytes.
 * @return 0 for invalid input, oversized payload, or pool exhaustion; 1 after
 *         submitting the allocated buffer to the transmit queue.
 */
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

/**
 * @brief Formats text and submits it as a framed MSG_TEXT_REPLY response.
 * @param format printf-style format string.
 * @param ... Arguments consumed by the format string.
 * @return 0 if allocation or formatting fails or text is too large; 1 after
 *         submitting the frame to the transmit queue.
 */
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

/**
 * @brief Accumulates receive bytes and dispatches complete binary or legacy text commands.
 * @param buf Incoming bytes from one receive-queue item.
 * @param len Number of incoming bytes.
 * @note Handles split and concatenated frames. Legacy text requires CR or LF;
 *       incomplete input is discarded after a receive gap longer than 250 ms.
 */
void USB_ProcessReceivedPacket(uint8_t* buf, uint16_t len){
    static uint8_t pending[RX_USB_BUF_SIZE + 1];
    static uint16_t used = 0;
    static uint32_t last_received = 0;
    uint32_t now = HAL_GetTick();
    if(used && (uint32_t)(now - last_received) > 250u) used = 0;
    last_received = now;
    for(uint16_t i = 0; i < len; ++i){
        pending[used++] = buf[i];
        if(pending[0] != DEBUG_SOF1_BIN){
            if(buf[i] == DEBUG_SOF1_BIN){
                pending[0] = DEBUG_SOF1_BIN;
                used = 1;
                continue;
            }
            /* Unframed legacy commands are newline terminated. */
            if(buf[i] == '\n' || buf[i] == '\r'){
                Debug_ExecuteTextCommand((const char*)pending, used);
                used = 0;
            }else if(used == RX_USB_BUF_SIZE){
                used = 0;
                Debug_SendBinaryResponse(MSG_BUFFER_OVERFLOW, NULL, 0);
            }
            continue;
        }
        if(used == 2 && pending[1] != DEBUG_SOF2_BIN){
            used = pending[1] == DEBUG_SOF1_BIN ? 1 : 0;
            continue;
        }
        if(used < 5) continue;
        uint16_t payload_length = read_u16_le(&pending[3]);
        if(payload_length > RX_USB_BUF_SIZE - 5){
            used = 0;
            Debug_SendBinaryResponse(MSG_INVALID_PAYLOAD, NULL, 0);
            continue;
        }
        if(used == payload_length + 5){
            Debug_ExecuteBinaryCommand((MsgTypeTypeDef)pending[2], &pending[5], payload_length);
            used = 0;
        }
    }
}
