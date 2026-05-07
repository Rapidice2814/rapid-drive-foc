from enum import IntEnum
import msvcrt
import threading
import threading

import serial
import struct
import time
import h5py
import numpy as np
from datetime import datetime
from pathlib import Path

import queue
from collections import deque
import matplotlib.pyplot as plt
from matplotlib.widgets import Button


def get_log_filename():
    timestamp = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
    return Path(f"debug_log_{timestamp}.h5")

TIMESTAMP_HZ = 8000.0
LOG_PLOT_DECIMATION = 100
LOG_PLOT_MAX_POINTS = 800
LOG_PLOT_UPDATE_PERIOD_S = 0.1

LOG_BATCH_PACKETS = 20
LOG_FILE = get_log_filename()


PORT = 'COM3'
BAUDRATE = 115200
TIMEOUT = 0.1


SOF1 = 0xAA
SOF2 = 0x55
HEADER_LEN = 5   # 2 SOF + 1 msg_type + 2 payload_length

log_mask = 0b00000011

SIGNAL_TABLE = [
    {"bit": 0, "type": "u32", "name": "timestamp"},
    {"bit": 1, "type": "f",   "name": "ab_current.alpha"},
    {"bit": 2, "type": "f",   "name": "ab_current.beta"},
    {"bit": 3, "type": "f",   "name": "dq_current.d"},
    {"bit": 4, "type": "f",   "name": "dq_current.q"},
    {"bit": 5, "type": "f",   "name": "ab_voltage.alpha"},
    {"bit": 6, "type": "f",   "name": "ab_voltage.beta"},
    {"bit": 7, "type": "f",   "name": "dq_voltage.d"},
    {"bit": 8, "type": "f",   "name": "dq_voltage.q"},
    {"bit": 9, "type": "u32", "name": "execution_time.loop_max"},
]

class MsgType(IntEnum):
    MSG_LOG_DATA  = 0x01
    MSG_SET_MASK  = 0x02
    MSG_START_LOG = 0x03
    MSG_STOP_LOG  = 0x04

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
    for sig in SIGNAL_TABLE:
        if log_mask & (1 << sig["bit"]):
            enabled.append(sig)
    return enabled

def _dtype_from_values(values):
    first = values[0]
    if isinstance(first, float):
        return np.float32
    return np.uint32

def init_hdf5_file(filename, decoded, log_mask):
    with h5py.File(filename, "a") as f:
        if "time" not in f:
            f.create_dataset(
                "time",
                shape=(0,),
                maxshape=(None,),
                dtype=np.uint32,
                chunks=True
            )

        for name, values in decoded["signals"].items():
            if name not in f:
                f.create_dataset(
                    name,
                    shape=(0,),
                    maxshape=(None,),
                    dtype=_dtype_from_values(values),
                    chunks=True
                )

        f.attrs["log_mask"] = int(log_mask)
        f.attrs["signal_count"] = int(decoded["signal_count"])
        f.attrs["sample_count"] = int(decoded["sample_count"])
        print(f"Initialized HDF5 file with log_mask={log_mask:08b} and signals {[sig['name'] for sig in decoded['enabled_signals']]}")


def append_decoded_batch_to_hdf5(filename, decoded_list):
    if not decoded_list:
        return

    total_samples = sum(d["sample_count"] for d in decoded_list)

    time_parts = []
    signal_parts = {}

    for decoded in decoded_list:
        sample_count = decoded["sample_count"]
        base_timestamp = decoded["timestamp"]

        time_parts.append(
            np.arange(base_timestamp, base_timestamp + sample_count, dtype=np.uint32)
        )

        for name, values in decoded["signals"].items():
            if name not in signal_parts:
                signal_parts[name] = []
            signal_parts[name].append(np.asarray(values))

    time_array = np.concatenate(time_parts)

    with h5py.File(filename, "a") as f:
        time_ds = f["time"]
        old_len = time_ds.shape[0]
        new_len = old_len + total_samples

        time_ds.resize((new_len,))
        time_ds[old_len:new_len] = time_array

        for name, parts in signal_parts.items():
            ds = f[name]
            values_np = np.concatenate(parts).astype(ds.dtype, copy=False)
            ds.resize((new_len,))
            ds[old_len:new_len] = values_np



def extract_packets(buffer):
    packets = []

    while True:
        if len(buffer) < HEADER_LEN:
            break

        sof_index = buffer.find(bytes([SOF1, SOF2]))
        if sof_index == -1:
            buffer.clear()
            break

        if sof_index > 0:
            del buffer[:sof_index]

        if len(buffer) < HEADER_LEN:
            break

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
        "raw_data": raw_data,
    }

def build_packet(msg_type, payload: bytes) -> bytes:
    msg_type = int(msg_type)

    if not (0 <= msg_type <= 0xFF):
        raise ValueError("msg_type must fit in one byte")

    payload_length = len(payload)
    if payload_length > 0xFFFF:
        raise ValueError("payload too large for 16-bit length")

    return struct.pack('<BBBH', SOF1, SOF2, msg_type, payload_length) + payload

def send_packet(ser, msg_type, payload: bytes):
    packet = build_packet(msg_type, payload)
    ser.write(packet)
    ser.flush()

def execute_terminal_command(ser, command_str):
    global log_mask

    command_str = command_str.strip()
    if not command_str:
        send_packet(ser, MsgType.MSG_START_LOG, b'')
        print("Sent: MSG_START_LOG")
        return True

    parts = command_str.split()
    cmd = parts[0].lower()

    if cmd == "start":
        send_packet(ser, MsgType.MSG_START_LOG, b'')
        print("Sent: MSG_START_LOG")
        return True

    elif cmd == "stop":
        send_packet(ser, MsgType.MSG_STOP_LOG, b'')
        print("Sent: MSG_STOP_LOG")
        return True

    elif cmd == "setmask":
        if len(parts) != 2:
            print("Usage: setmask <value>")
            return False

        try:
            new_mask = int(parts[1], 0)
        except ValueError:
            print("Invalid mask value. Examples: setmask 3, setmask 0x03")
            return False

        payload = struct.pack('<I', new_mask)
        send_packet(ser, MsgType.MSG_SET_MASK, payload)
        log_mask = new_mask
        print(f"Sent: MSG_SET_MASK = 0x{new_mask:08X}")
        return True

    else:
        print(f"Unknown command: {command_str}")
        print("Commands: start, stop, setmask <value>")
        return False
    
def handle_terminal_input(ser, line):
    if line is None:
        return
    execute_terminal_command(ser, line)

def poll_terminal_line():
    if not hasattr(poll_terminal_line, "buffer"):
        poll_terminal_line.buffer = ""

    while msvcrt.kbhit():
        ch = msvcrt.getwch()

        if ch == '\003':
            raise KeyboardInterrupt

        if ch in ('\r', '\n'):
            line = poll_terminal_line.buffer
            poll_terminal_line.buffer = ""
            return line

        if ch == '\b':
            poll_terminal_line.buffer = poll_terminal_line.buffer[:-1]
        else:
            poll_terminal_line.buffer += ch

    return None

def init_live_plot(decoded,
                   decimation=LOG_PLOT_DECIMATION,
                   max_points=LOG_PLOT_MAX_POINTS,
                   update_period_s=LOG_PLOT_UPDATE_PERIOD_S,
                   queue_maxsize=200,
                   timestamp_hz=TIMESTAMP_HZ):
    signal_names = list(decoded["signals"].keys())
    n_signals = len(signal_names)

    fig, axes = plt.subplots(
        n_signals, 1,
        sharex=True,
        figsize=(12, max(2.5 * n_signals, 4)),
        squeeze=False
    )
    axes = axes[:, 0]

    lines = {}
    x_buffer = deque(maxlen=max_points)
    y_buffers = {}

    for ax, name in zip(axes, signal_names):
        line, = ax.plot([], [], lw=1.0, label=name)
        ax.set_ylabel(name)
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right")

        lines[name] = line
        y_buffers[name] = deque(maxlen=max_points)

    axes[-1].set_xlabel(f"Time [s] (decimated x{decimation})")

    ax_pause = plt.axes([0.8, 0.025, 0.1, 0.075])  # [left, bottom, width, height]
    pause_button = Button(ax_pause, 'Pause')

    plt.ion()
    plt.show(block=False)

    plot_state = {
        "fig": fig,
        "axes": axes,
        "lines": lines,
        "signal_names": signal_names,
        "x_buffer": x_buffer,
        "y_buffers": y_buffers,
        "decimation": decimation,
        "max_points": max_points,
        "update_period_s": update_period_s,
        "last_plot_time": 0.0,
        "queue": queue.Queue(maxsize=queue_maxsize),
        "timestamp_hz": timestamp_hz,
        "is_paused": False,
    }

    def toggle_pause(event):
        plot_state["is_paused"] = not plot_state["is_paused"]
        print(f"Paused: {plot_state['is_paused']}")
        pause_button.label.set_text('Resume' if plot_state["is_paused"] else 'Pause')
        fig.canvas.draw_idle()

    pause_button.on_clicked(toggle_pause)

    return plot_state

def queue_live_plot_data(plot_state, decoded):
    
    decimation = plot_state["decimation"]
    timestamp_hz = plot_state["timestamp_hz"]

    sample_count = decoded["sample_count"]
    base_timestamp = decoded["timestamp"]

    idx = np.arange(0, sample_count, decimation)
    if idx.size == 0:
        return

    time_array = (
        np.arange(base_timestamp, base_timestamp + sample_count, dtype=np.float64)
        / timestamp_hz
    )[idx]

    payload = {
        "x": time_array,
        "signals": {
            name: np.asarray(decoded["signals"][name])[idx]
            for name in plot_state["signal_names"]
        }
    }

    try:
        plot_state["queue"].put_nowait(payload)
    except queue.Full:
        try:
            plot_state["queue"].get_nowait()
        except queue.Empty:
            pass
        try:
            plot_state["queue"].put_nowait(payload)
        except queue.Full:
            pass

def update_live_plot_if_due(plot_state, force=False):
    is_paused = plot_state["is_paused"]
    if is_paused:
        return

    now = time.perf_counter()

    if not force and (now - plot_state["last_plot_time"] < plot_state["update_period_s"]):
        return

    drained = []

    while True:
        try:
            drained.append(plot_state["queue"].get_nowait())
        except queue.Empty:
            break

    if not drained:
        if force:
            plot_state["fig"].canvas.draw_idle()
            plot_state["fig"].canvas.flush_events()
        plot_state["last_plot_time"] = now
        return

    for item in drained:
        plot_state["x_buffer"].extend(item["x"].tolist())

        for name in plot_state["signal_names"]:
            plot_state["y_buffers"][name].extend(np.asarray(item["signals"][name]).tolist())

    x = np.asarray(plot_state["x_buffer"])
    if x.size == 0:
        plot_state["last_plot_time"] = now
        return

    for ax, name in zip(plot_state["axes"], plot_state["signal_names"]):
        y = np.asarray(plot_state["y_buffers"][name])
        plot_state["lines"][name].set_data(x, y)

        if y.size > 0:
            ymin = np.min(y)
            ymax = np.max(y)
            if ymin == ymax:
                pad = 1.0 if ymin == 0 else abs(ymin) * 0.05
            else:
                pad = (ymax - ymin) * 0.1
            ax.set_ylim(ymin - pad, ymax + pad)

    if x.size > 1 and x[0] != x[-1]:
        plot_state["axes"][-1].set_xlim(x[0], x[-1])

    plot_state["fig"].canvas.draw_idle()
    plot_state["fig"].canvas.flush_events()
    plot_state["last_plot_time"] = now

hdf5_initialized = False
plot_state = None
previous_timestamp = 0
decoded_batch = []

def usb_serial_worker(ser, plot_queue, command_queue, stop_event):
    global hdf5_initialized, previous_timestamp, decoded_batch, log_mask

    rx_buffer = bytearray()

    while not stop_event.is_set():
        try:
            try:
                cmd = command_queue.get_nowait()
                execute_terminal_command(ser, cmd)
            except queue.Empty:
                pass

            data = ser.read(256)
            if data:
                rx_buffer.extend(data)

                packets = extract_packets(rx_buffer)
                for pkt in packets:
                    try:
                        pkt_type = MsgType(pkt["msg_type"])
                    except ValueError:
                        print(f"Unknown packet type: {pkt['msg_type']}")
                        continue

                    if pkt_type == MsgType.MSG_LOG_DATA:
                        decoded = extract_log_payload(pkt["payload"], log_mask)
                        if not decoded:
                            print("payload decode error (skipped)")
                            print(f"payload hex = {pkt['payload'].hex(' ')}")
                            continue

                        current_timestamp = decoded["timestamp"]
                        if previous_timestamp != 0:
                            if (current_timestamp - previous_timestamp) != decoded["sample_count"]:
                                print(f"Warning: timestamp jump detected! Jump={current_timestamp - previous_timestamp}, expected={decoded['sample_count']}")
                        previous_timestamp = current_timestamp

                        if not hdf5_initialized:
                            init_hdf5_file(LOG_FILE, decoded, log_mask)
                            hdf5_initialized = True

                        decoded_batch.append(decoded)
                        if len(decoded_batch) >= LOG_BATCH_PACKETS:
                            append_decoded_batch_to_hdf5(LOG_FILE, decoded_batch)
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
                    else:
                        print(f"Received packet: {pkt_type.name}")

            else:
                time.sleep(0.001)

        except Exception as e:
            print(f"Serial worker error: {e}")
            time.sleep(0.01)

    if decoded_batch:
        append_decoded_batch_to_hdf5(LOG_FILE, decoded_batch)

def terminal_worker(command_queue, stop_event):
    while not stop_event.is_set():
        try:
            line = input("cmd> ")
            command_queue.put(line)
        except EOFError:
            break
        except Exception as e:
            print(f"Terminal worker error: {e}")
            break

def main_plot_loop(plot_queue, stop_event):
    plot_state = None

    while not stop_event.is_set():
        drained = []

        while True:
            try:
                drained.append(plot_queue.get_nowait())
            except queue.Empty:
                break

        for decoded in drained:
            if plot_state is None:
                plot_state = init_live_plot(
                    decoded,
                    decimation=LOG_PLOT_DECIMATION,
                    update_period_s=LOG_PLOT_UPDATE_PERIOD_S,
                    timestamp_hz=TIMESTAMP_HZ
                )
                print(f"Initialized live plot with signals: {plot_state['signal_names']}")

            queue_live_plot_data(plot_state, decoded)

        if plot_state is not None:
            update_live_plot_if_due(plot_state)
            plt.pause(0.001)
        else:
            plt.pause(0.05)

plot_queue = queue.Queue(maxsize=200)
command_queue = queue.Queue()
stop_event = threading.Event()
ser = serial.Serial(PORT, BAUDRATE, timeout=TIMEOUT)

usb_serial_thread = threading.Thread(
    target=usb_serial_worker,
    args=(ser, plot_queue, command_queue, stop_event),
    daemon=True
)

terminal_thread = threading.Thread(
    target=terminal_worker,
    args=(command_queue, stop_event),
    daemon=True
)

try:
    print(f"Listening on {PORT} at {BAUDRATE} baud...")
    print("Commands: start, stop, setmask <value>")

    usb_serial_thread.start()
    terminal_thread.start()

    main_plot_loop(plot_queue, stop_event)

except KeyboardInterrupt:
    print("Stopping...")

finally:
    stop_event.set()
    time.sleep(0.1)

    if ser.is_open:
        ser.close()