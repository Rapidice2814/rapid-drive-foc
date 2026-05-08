import struct
from enum import IntEnum
import queue
from unittest import case

import config
from logger_hdf5 import get_log_filename, init_hdf5_file, append_decoded_batch_to_hdf5

FOC_USB_DEBUG_SIGNAL_LIST = [
    {"bit": 0,  "type": "u32",   "name": "timestamp"},
    {"bit": 1,  "type": "f",     "name": "adc_values.motor_temp"},
    {"bit": 2,  "type": "f",     "name": "adc_values.mosfet_temp"},
    {"bit": 3,  "type": "f",     "name": "adc_values.vbus"},
    {"bit": 4,  "type": "f",     "name": "ibus"},
    {"bit": 5,  "type": "f",     "name": "adc_values.phase_current.a"},
    {"bit": 6,  "type": "f",     "name": "adc_values.phase_current.b"},
    {"bit": 7,  "type": "f",     "name": "adc_values.phase_current.c"},

    {"bit": 8,  "type": "f",     "name": "ab_current.alpha"},
    {"bit": 9,  "type": "f",     "name": "ab_current.beta"},
    {"bit": 10, "type": "f",     "name": "dq_current.d"},
    {"bit": 11, "type": "f",     "name": "dq_current.q"},
    {"bit": 12, "type": "f",     "name": "phase_voltage.a"},
    {"bit": 13, "type": "f",     "name": "phase_voltage.b"},
    {"bit": 14, "type": "f",     "name": "phase_voltage.c"},
    {"bit": 15, "type": "f",     "name": "ab_voltage.alpha"},

    {"bit": 16, "type": "f",     "name": "ab_voltage.beta"},
    {"bit": 17, "type": "f",     "name": "dq_voltage.d"},
    {"bit": 18, "type": "f",     "name": "dq_voltage.q"},
    {"bit": 19, "type": "f",     "name": "encoder_angle_mechanical"},
    {"bit": 20, "type": "f",     "name": "encoder_speed_mechanical"},
    {"bit": 21, "type": "f",     "name": "encoder_angle_electrical"},
    {"bit": 22, "type": "f",     "name": "encoder_speed_electrical"},
    {"bit": 23, "type": "f",     "name": "dq_current_setpoint.d"},
    
    {"bit": 24, "type": "f",     "name": "dq_current_setpoint.q"},
    {"bit": 25, "type": "f",     "name": "angle_setpoint"},
    {"bit": 26, "type": "f",     "name": "speed_setpoint"},
    {"bit": 27, "type": "u32",   "name": "execution_time.loop_max"},
]

FOC_PID_CONTROLLERS_LIST = [
    {"id": 0, "name": "pid_current_d"},
    {"id": 1, "name": "pid_current_q"},
    {"id": 2, "name": "pid_speed"},
    {"id": 3, "name": "pid_position"},
]

VAR_ID_LIST = [
    {"id": 0, "type": "f", "name": "dq_current_setpoint.d"},
    {"id": 1, "type": "f", "name": "dq_current_setpoint.q"},
    {"id": 2, "type": "f", "name": "angle_setpoint"},
    {"id": 3, "type": "f", "name": "speed_setpoint"},
]


class MsgType(IntEnum):
    MSG_LOG_DATA        = 0x01  # FOC -> PC
    MSG_SET_MASK        = 0x02  # PC -> FOC
    MSG_START_LOG       = 0x03  # PC -> FOC
    MSG_STOP_LOG        = 0x04  # PC -> FOC
    MSG_SET_PID         = 0x05  # PC -> FOC
    MSG_GET_PID         = 0x06  # PC -> FOC
    MSG_PID_REPLY       = 0x07  # FOC -> PC
    MSG_SET_VAR         = 0x08  # PC -> FOC
    MSG_GET_VAR         = 0x09  # PC -> FOC
    MSG_VAR_REPLY       = 0x0A  # FOC -> PC
    MSG_FLASH_SAVE      = 0x0B  # PC -> FOC
    MSG_FLASH_LOAD      = 0x0C  # PC -> FOC
    MSG_SET_STATE       = 0x0D  # PC -> FOC
    MSG_GET_STATE       = 0x0E  # PC -> FOC
    MSG_STATE_REPLY     = 0x0F  # FOC -> PC

    MSG_UNKNOWN_TYPE    = 0xFA  # FOC -> PC
    MSG_INVALID_PAYLOAD = 0xFB  # FOC -> PC
    MSG_UNKNOWN_ID      = 0xFC  # FOC -> PC
    MSG_BUFFER_OVERFLOW = 0xFD  # FOC -> PC
    MSG_ACK             = 0xFE  # FOC -> PC
    MSG_ERROR           = 0xFF  # FOC -> PC




def u32_to_f32(v):
    return struct.unpack('<f', struct.pack('<I', v))[0]


def u32_to_i32(v):
    return struct.unpack('<i', struct.pack('<I', v))[0]


def cast_u32_value(v, typ):
    if typ == 'u32':
        return v
    elif typ == 'i32':
        return u32_to_i32(v)
    elif typ == 'f':
        return u32_to_f32(v)
    else:
        raise ValueError(f"Unsupported type: {typ}")
    
def get_enabled_signals(log_mask):
    enabled = []
    for sig in FOC_USB_DEBUG_SIGNAL_LIST:
        if log_mask & (1 << sig["bit"]):
            enabled.append(sig)
    return enabled

def extract_packets(buffer):
    packets = []
    HEADER_LEN = 5   # 2 SOF + 1 msg_type + 2 payload_length

    while True:
        if len(buffer) < HEADER_LEN:
            break

        sof_index = buffer.find(bytes([config.SOF1, config.SOF2]))
        if sof_index == -1:
            buffer.clear()
            break

        if sof_index > 0:
            del buffer[:sof_index]

        msg_type = buffer[2]
        payload_length = buffer[3] | (buffer[4] << 8)
        packet_length = HEADER_LEN + payload_length

        if len(buffer) < packet_length:
            break

        payload = bytes(buffer[5:5 + payload_length])

        packets.append({
            "msg_type": msg_type,
            "payload_length": payload_length,
            "payload": payload
        })

        del buffer[:packet_length]

    return packets



def extract_log_payload(payload, log_mask):

    payload_header_format = '<IHH'
    payload_header_size = struct.calcsize(payload_header_format)

    if len(payload) < payload_header_size:
        return None, 0.0

    timestamp, sample_count, signal_count = struct.unpack_from(payload_header_format, payload, 0)

    enabled_signals = get_enabled_signals(log_mask)

    if len(enabled_signals) != signal_count:
        raise ValueError(
            f"Mask enables {len(enabled_signals)} signals, but payload says signal_count={signal_count}"
        )

    data_count = sample_count * signal_count
    data_format = f'<{data_count}I'
    data_size = struct.calcsize(data_format)
    expected_size = payload_header_size + data_size

    if len(payload) != expected_size:
        return None, 0.0

    raw_data = struct.unpack_from(data_format, payload, payload_header_size)

    signal_buffers = {}
    for sig in enabled_signals:
        signal_buffers[sig["name"]] = []

    for sample_idx in range(sample_count):
        base_idx = sample_idx * signal_count
        for sig_idx, sig in enumerate(enabled_signals):
            raw_u32 = raw_data[base_idx + sig_idx]
            value = cast_u32_value(raw_u32, sig["type"])
            signal_buffers[sig["name"]].append(value)

    return {
        "timestamp": timestamp,
        "sample_count": sample_count,
        "signal_count": signal_count,
        "enabled_signals": enabled_signals,
        "signals": signal_buffers,
    }

previous_timestamp = 0
decoded_batch = []

def decode_packet(pkt, plot_queue):
    global previous_timestamp, decoded_batch

    try:
        pkt_type = MsgType(pkt["msg_type"])
    except ValueError:
        print(f"Unknown packet type: {pkt['msg_type']}")
        return None
    
    match pkt_type:
        case MsgType.MSG_LOG_DATA:
            decoded = extract_log_payload(pkt["payload"], config.log_mask)
            if not decoded:
                print("payload decode error (skipped)")
                print(f"payload hex = {pkt['payload'].hex(' ')}")
                return None

            current_timestamp = decoded["timestamp"]
            if previous_timestamp != 0:
                if (current_timestamp - previous_timestamp) != decoded["sample_count"]:
                    print(f"Warning: timestamp jump detected! Jump={current_timestamp - previous_timestamp}, expected={decoded['sample_count']}")
            previous_timestamp = current_timestamp

            if not config.hdf5_initialized:
                config.log_filename = get_log_filename()
                init_hdf5_file(config.log_filename, decoded, config.log_mask)
                config.hdf5_initialized = True
                decoded_batch.clear()

            decoded_batch.append(decoded)
            if len(decoded_batch) >= config.LOG_BATCH_PACKETS:
                append_decoded_batch_to_hdf5(config.log_filename, decoded_batch)
                decoded_batch.clear()

            try:
                plot_queue.put_nowait(decoded)
            except queue.Full:
                try:
                    plot_queue.get_nowait()
                except queue.Empty:
                    pass
                try:
                    plot_queue.put_nowait(decoded)
                except queue.Full:
                    pass
        case MsgType.MSG_PID_REPLY:
            if pkt["payload_length"] != 13:
                print(f"Invalid PID_REPLY payload length: {pkt['payload_length']}")
                return None
            controller_id = pkt["payload"][0]
            gains_data = pkt["payload"][1:13]
            kp, ki, kd = struct.unpack('<fff', gains_data)
            print(f"PID Reply - Controller ID: {controller_id}, Kp: {kp}, Ki: {ki}, Kd: {kd}")
        case _:
            print(f"Received packet: {pkt_type.name}")

def build_packet(msg_type, payload: bytes) -> bytes:
    msg_type = int(msg_type)

    if not (0 <= msg_type <= 0xFF):
        raise ValueError("msg_type must fit in one byte")

    payload_length = len(payload)
    if payload_length > 0xFFFF:
        raise ValueError("payload too large for 16-bit length")

    return struct.pack('<BBBH', config.SOF1, config.SOF2, msg_type, payload_length) + payload
