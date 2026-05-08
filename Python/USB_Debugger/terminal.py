import msvcrt
import struct

from protocol import MsgType, build_packet, FOC_USB_DEBUG_SIGNAL_LIST, FOC_PID_CONTROLLERS_LIST, VAR_ID_LIST
import config

PID_LOOKUP = {item["id"]: item for item in FOC_PID_CONTROLLERS_LIST}


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

def execute_terminal_command(ser, command_str):

    command_str = command_str.strip()
    if not command_str:
        send_packet(ser, MsgType.MSG_START_LOG, b'')
        print("Sent: MSG_START_LOG")
        return True

    parts = command_str.split()
    cmd = parts[0].lower()

    if cmd == "start":
        send_packet(ser, MsgType.MSG_START_LOG, b'')
        config.log_isrunning = True
        print("Sent: MSG_START_LOG")
        return True

    elif cmd == "stop":
        send_packet(ser, MsgType.MSG_STOP_LOG, b'')
        config.log_isrunning = False
        print("Sent: MSG_STOP_LOG")
        return True

    elif cmd == "setmask":
        if (config.log_isrunning):
            print("Cannot change log mask while logging is running. Please stop logging first.")
            return False
        
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
        config.log_mask = new_mask
        config.hdf5_initialized = False
        print(f"Sent: MSG_SET_MASK = 0x{new_mask:08X}")
        return True
    elif cmd == "getpid":
        if len(parts) != 2:
            print("Usage: getpid <controller_id>")
            return False

        try:
            controller_id = int(parts[1], 0)
            controller = PID_LOOKUP[controller_id]
        except ValueError:
            print("Invalid controller ID. Examples: getpid 1, getpid 0x01")
            return False

        payload = struct.pack('<B', controller_id)
        send_packet(ser, MsgType.MSG_GET_PID, payload)
        print(f"Sent: MSG_GET_PID for Controller ID {controller_id}, Name: {controller['name']}")
        return True
    elif cmd == "setpid":
        if len(parts) != 5:
            print("Usage: setpid <controller_id> <kp> <ki> <kd>")
            return False

        try:
            controller_id = int(parts[1], 0)
            kp = float(parts[2])
            ki = float(parts[3])
            kd = float(parts[4])
            controller = PID_LOOKUP[controller_id]
        except ValueError:
            print("Invalid arguments. Examples: setpid 1 0.1 0.01 0.001")
            return False

        payload = struct.pack('<Bfff', controller_id, kp, ki, kd)
        send_packet(ser, MsgType.MSG_SET_PID, payload)
        print(f"Sent: MSG_SET_PID for Controller ID {controller_id}, Name: {controller['name']} with Kp={kp}, Ki={ki}, Kd={kd}")
        return True
    else:
        print(f"Unknown command: {command_str}")
        print("Commands: start, stop, setmask <value>, getpid <controller_id>, setpid <controller_id> <kp> <ki> <kd>")
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

def send_packet(ser, msg_type, payload: bytes):
    packet = build_packet(msg_type, payload)
    ser.write(packet)
    ser.flush()