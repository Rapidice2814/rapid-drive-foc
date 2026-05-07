PORT = 'COM3'
BAUDRATE = 115200
TIMEOUT = 0.1

TIMESTAMP_HZ = 8000.0
LOG_PLOT_DECIMATION = 100
LOG_PLOT_MAX_POINTS = 800
LOG_PLOT_UPDATE_PERIOD_S = 0.1

LOG_BATCH_PACKETS = 20

SOF1 = 0xAA
SOF2 = 0x55

log_isrunning = False
log_mask = 0
hdf5_initialized = False
plot_state = None
log_filename = None