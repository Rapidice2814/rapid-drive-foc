import threading
import serial
import time
import queue

import config
from usb_serial import usb_serial_worker
from terminal import terminal_worker
from matlib_plotter import main_plot_loop


plot_queue = queue.Queue(maxsize=200)
command_queue = queue.Queue()
stop_event = threading.Event()
ser = serial.Serial(config.PORT, config.BAUDRATE, timeout=config.TIMEOUT)

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
    print(f"Listening on {config.PORT} at {config.BAUDRATE} baud...")
    print("Commands: start, stop, setmask <value>")

    usb_serial_thread.start()
    terminal_thread.start()

    command_queue.put("stop")
    command_queue.put("setmask 0x080000EF")
    command_queue.put("start")

    main_plot_loop(plot_queue, stop_event)

    while True:
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopping...")

finally:
    stop_event.set()
    time.sleep(0.1)

    if ser.is_open:
        ser.close()