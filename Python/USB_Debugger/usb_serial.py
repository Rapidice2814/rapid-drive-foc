import time
import queue

from serial import SerialException


from protocol import decode_packet, extract_packets
from terminal import execute_terminal_command



def usb_serial_worker(ser, plot_queue, command_queue, stop_event):

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
                    decode_packet(pkt, plot_queue)

            else:
                time.sleep(0.001)
        except (SerialException, PermissionError) as e:
            print(f"Serial port disconnected: {e}")
            break
        except Exception as e:
            print(f"Serial worker error: {e}")
            time.sleep(0.01)

    # if decoded_batch:
    #     append_decoded_batch_to_hdf5(LOG_FILE, decoded_batch)

