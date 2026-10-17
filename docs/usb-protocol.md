# USB protocol reference (v3)

Frames use `AA 55 | message:u8 | payload_length:u16le | payload`.
All multi-byte integers and four-byte numeric values are little endian.
Maximum command payload is currently 123 bytes; replies allow 1019 bytes.
Device info reports the signal-mask width and telemetry limits.

The transport is USB CDC (a serial/COM port on the host). A host read or device
receive callback is not a packet boundary: binary frames may be split across
chunks or several frames may arrive together. Maximum command frame size is
128 bytes including the header. A receive gap longer than 250 ms discards a
partial command. Unframed legacy text requires CR or LF; binary text replies
are always carried in MSG_TEXT_REPLY frames. There is no application checksum
or transaction ID. The plotter therefore keeps one command outstanding at a time.

## Command IDs

| ID | Command |
| --- | --- |
| 0x00 | `MSG_ENTER_BOOTLOADER` |
| 0x01 | `MSG_GET_DEVICE_INFO` |
| 0x02 | `MSG_DEVICE_INFO_REPLY` |
| 0x03 | `MSG_GET_DEFINITIONS` |
| 0x04 | `MSG_DEFINITIONS_REPLY` |
| 0x05 | `MSG_DEFINITIONS_ERROR` |
| 0x06 | `MSG_LOG_DATA` |
| 0x07 | `MSG_SET_MASK` |
| 0x08 | `MSG_GET_MASK` |
| 0x09 | `MSG_MASK_REPLY` |
| 0x0A | `MSG_START_LOG` |
| 0x0B | `MSG_STOP_LOG` |
| 0x0C | `MSG_SET_PID` |
| 0x0D | `MSG_GET_PID` |
| 0x0E | `MSG_PID_REPLY` |
| 0x0F | `MSG_SET_VAR` |
| 0x10 | `MSG_GET_VAR` |
| 0x11 | `MSG_VAR_REPLY` |
| 0x12 | `MSG_FLASH_SAVE` |
| 0x13 | `MSG_FLASH_LOAD` |
| 0x14 | `MSG_FLASH_CLEAR` |
| 0x15 | `MSG_SET_STATE` |
| 0x16 | `MSG_GET_STATE` |
| 0x17 | `MSG_STATE_REPLY` |
| 0x18 | `MSG_TEXT_COMMAND` |
| 0x19 | `MSG_TEXT_REPLY` |
| 0x1A | `MSG_SET_NODE_ID` |
| 0x1B | `MSG_GET_NODE_ID` |
| 0x1C | `MSG_NODE_ID_REPLY` |
| 0x1D | `MSG_GET_ACTIVE_ERRORS` |
| 0x1E | `MSG_ACTIVE_ERRORS_REPLY` |
| 0x1F | `MSG_GET_LATCHED_ERRORS` |
| 0x20 | `MSG_LATCHED_ERRORS_REPLY` |
| 0x21 | `MSG_CLEAR_LATCHED_ERRORS` |
| 0x22 | `MSG_SET_CAN_CYCLIC_RATE` |
| 0x23 | `MSG_GET_CAN_CYCLIC_RATE` |
| 0x24 | `MSG_CAN_CYCLIC_REPLY` |
| 0x25 | `MSG_SET_CONTROL_MODE` |
| 0x26 | `MSG_GET_CONTROL_MODE` |
| 0x27 | `MSG_CONTROL_MODE_REPLY` |

Response/error IDs occupy 0xFA through 0xFF.

## Payloads and command behavior

H→D means host to driver; D→H means driver to host. Fields are listed in wire
order. `f32` means IEEE-754 32-bit float, and `u8`, `u16`, and `u32` are unsigned
integers of the stated widths. A variable's four-byte value is interpreted
using its discovered type (`f`, `u32`, or `i32`). Empty means zero payload bytes.

| Command | Direction | Payload | Response or behavior |
| --- | --- | --- | --- |
| MSG_ENTER_BOOTLOADER | H→D | Empty | Requests BOOTLOADER from IDLE; ACK if accepted, ERROR if denied |
| MSG_GET_DEVICE_INFO | H→D | Empty | DEVICE_INFO_REPLY; see detailed layout below |
| MSG_DEVICE_INFO_REPLY | D→H | 29 bytes | Protocol/firmware versions, counts, limits, rate, fingerprint, status |
| MSG_GET_DEFINITIONS | H→D | kind:u8, start:u16 (3 bytes) | DEFINITIONS_REPLY or DEFINITIONS_ERROR |
| MSG_DEFINITIONS_REPLY | D→H | 7-byte page header and UTF-8 rows | See discovery rules below |
| MSG_DEFINITIONS_ERROR | D→H | kind:u8, start:u16, error:u8 (4 bytes) | Explicit discovery failure |
| MSG_LOG_DATA | D→H | timestamp:u32, samples:u16, signals:u16, sample words | Sample-major telemetry; see below |
| MSG_SET_MASK | H→D | mask:ceil(signal_count / 8) bytes | ACK if valid and logging is stopped; otherwise ERROR |
| MSG_GET_MASK | H→D | Empty | MASK_REPLY |
| MSG_MASK_REPLY | D→H | Current mask bytes | Same width as reported by device info |
| MSG_START_LOG | H→D | Empty | Discards any partial packet, enables capture, ACK |
| MSG_STOP_LOG | H→D | Empty | Disables capture, discards any partial packet, ACK |
| MSG_SET_PID | H→D | id:u8, Kp:f32, Ki:f32, Kd:f32 (13 bytes) | Applies gains to discovered controller ID, ACK |
| MSG_GET_PID | H→D | id:u8 (1 byte) | PID_REPLY |
| MSG_PID_REPLY | D→H | id:u8, Kp:f32, Ki:f32, Kd:f32 (13 bytes) | Current gains |
| MSG_SET_VAR | H→D | id:u8, value:4 bytes (5 bytes) | Applies typed value, ACK; a value too wide for native integer storage is INVALID_PAYLOAD |
| MSG_GET_VAR | H→D | id:u8 (1 byte) | VAR_REPLY |
| MSG_VAR_REPLY | D→H | id:u8, value:4 bytes (5 bytes) | Current typed value |
| MSG_FLASH_SAVE | H→D | Empty | Requests FLASH_SAVE from IDLE, then RUN; ACK or ERROR |
| MSG_FLASH_LOAD | H→D | Empty | Requests FLASH_LOAD from IDLE, then RUN; ACK or ERROR |
| MSG_FLASH_CLEAR | H→D | Empty | Requests FLASH_CLEAR from IDLE, then RUN; ACK or ERROR |
| MSG_SET_STATE | H→D | state:u8 (1 byte) | Requests a state transition; ACK or ERROR |
| MSG_GET_STATE | H→D | Empty | STATE_REPLY |
| MSG_STATE_REPLY | D→H | state:u8 (1 byte) | Current state enum |
| MSG_TEXT_COMMAND | H→D | Bounded command text, up to 123 bytes | Executes legacy text parsing and sends ACK; may also emit TEXT_REPLY |
| MSG_TEXT_REPLY | D→H | UTF-8/ASCII text bytes | Diagnostic text; no terminating NUL on the wire |
| MSG_SET_NODE_ID | H→D | node:u8 (1 byte) | Reconfigures CAN address/filters and ACKs; valid node range is 0–15 |
| MSG_GET_NODE_ID | H→D | Empty | NODE_ID_REPLY |
| MSG_NODE_ID_REPLY | D→H | node:u8 (1 byte) | Stored CAN node-ID byte |
| MSG_GET_ACTIVE_ERRORS | H→D | Empty | ACTIVE_ERRORS_REPLY |
| MSG_ACTIVE_ERRORS_REPLY | D→H | errors:u32 (4 bytes) | Active error bitmap |
| MSG_GET_LATCHED_ERRORS | H→D | Empty | LATCHED_ERRORS_REPLY |
| MSG_LATCHED_ERRORS_REPLY | D→H | errors:u32 (4 bytes) | Latched error bitmap |
| MSG_CLEAR_LATCHED_ERRORS | H→D | Empty | Clears latched errors and ACKs; active faults can latch again |
| MSG_SET_CAN_CYCLIC_RATE | H→D | index:u8, period_ticks:u32 (5 bytes) | Stores the cyclic period and ACKs; zero disables the entry |
| MSG_GET_CAN_CYCLIC_RATE | H→D | index:u8 (1 byte) | CAN_CYCLIC_REPLY |
| MSG_CAN_CYCLIC_REPLY | D→H | index:u8, period_ticks:u32 (5 bytes) | Current period, in FOC ticks rather than milliseconds |
| MSG_SET_CONTROL_MODE | H→D | mode:u8 (1 byte) | Selects mode and ACKs, or INVALID_PAYLOAD for an unsupported mode |
| MSG_GET_CONTROL_MODE | H→D | Empty | CONTROL_MODE_REPLY |
| MSG_CONTROL_MODE_REPLY | D→H | mode:u8 (1 byte) | Current control mode |
| MSG_UNKNOWN_TYPE (0xFA) | D→H | Empty | Command ID is not supported |
| MSG_INVALID_PAYLOAD (0xFB) | D→H | Empty | Wrong payload length, oversized command, invalid scalar range, or unsupported control mode |
| MSG_UNKNOWN_ID (0xFC) | D→H | Empty | Variable/PID ID is outside its discovered list |
| MSG_BUFFER_OVERFLOW (0xFD) | D→H | Empty | Unframed input exceeded the receive-buffer capacity |
| MSG_ACK (0xFE) | D→H | Empty | Command accepted; no request ID is included |
| MSG_ERROR (0xFF) | D→H | Empty | Command rejected by state/mask rules, or invalid driver definitions prevent configuration |

Payload lengths are checked for the fixed-layout commands above. Node IDs and
CAN cyclic indexes are exceptions to semantic range validation: the current
setters/getters do not check them. Valid node IDs are 0–15 and cyclic indexes
are 0–8, as listed in the [CAN reference](can-protocol.md#cyclic-replies-and-timing).

The plotter restricts PID and setpoint editors to RUN; the low-level USB
SET_PID/SET_VAR handlers do not independently enforce that UI restriction.
Control-mode selection resets the target associated with the newly selected
mode when the mode changes; it does not enter RUN. Flash actions and bootloader
entry are state-machine requests: an ACK confirms acceptance, not completion.
Flash actions are accepted only from IDLE and set RUN as the next state.
After a successful flash operation, a denied transition to RUN falls back to
STOP. A failed flash operation enters ERROR without attempting RUN.
Resulting values and states are available through their corresponding GET commands.

Diagnostic TEXT_REPLY frames and LOG_DATA can arrive between command replies.
Neither message acknowledges a pending binary request. If definitions are
invalid, device info, discovery errors, and STOP_LOG remain available; other
commands are rejected. Replies may be lost when no transmit buffer is available.

### Telemetry layout

LOG_DATA begins with an eight-byte header. Timestamp is the capture-sample
counter at the start of this packet, not milliseconds or the MCU wall clock.
For each sample, the payload contains one four-byte word for each enabled
signal, in ascending signal-ID/mask-bit order:

```text
timestamp:u32 | sample_count:u16 | signal_count:u16
sample 0: signal a | signal b | ...
sample 1: signal a | signal b | ...
...
```

Payload size is `8 + sample_count * signal_count * 4` bytes. With the current
30-sample packets and eight-signal limit, the complete USB frame is 973 bytes.
The mask is not repeated in each packet. A layout change follows the sequence
STOP_LOG → SET_MASK → GET_MASK → START_LOG before telemetry uses the new layout.
Sample values use their discovered types. Buffer shortages skip capture; the
capture counter does not advance while allocation fails and is not a continuous
real-time clock. The separately selectable `tick` signal is the FOC tick.

### Legacy text commands

Legacy parsing is case-sensitive. Raw text requires CR or LF; text inside MSG_TEXT_COMMAND uses
the frame length and need not have a newline. Unknown commands are ignored,
and a framed text ACK alone does not prove that the text was recognized.

| Text | Effect |
| --- | --- |
| Da / Df | Toggle anticogging feed-forward / current-PID feed-forward |
| A / R / E / O | Request ANTICOGGING / RUN / ERROR / STOP |
| F / L / B | Request flash save / flash clear / system bootloader; state rules still apply |
| Mo / Ms / Mp | Select open-loop current / speed / position control |
| K | Set motor_disable_flag |
| C | Invalidate encoder offset and motor resistance/inductance, then request CHECKLIST |
| PdN / PqN / PsN / PpN | Set the corresponding current-d/current-q/speed/position Kp to integer N / 1000 |
| IdN / IqN / IsN / IpN | Set the corresponding Ki to integer N / 1000 |
| SdN / SqN | Set d/q current target to integer N / 1000 A |
| SsN | Set speed target to integer N rad/s |
| SpN | Set angle target to integer N / 1000 rad |
| T | Recognized placeholder with no action |

Numeric arguments are integers, not floating-point text. These legacy commands
can alter state/configuration directly and do not share all plotter UI
restrictions. There is no separate firmware help/version text API.

## Initialization

Connection initialization follows this sequence:

1. Device-info request and protocol/status validation.
2. STOP_LOG.
3. Loading of validated cached definitions for the reported protocol/fingerprint,
   or discovery of all three lists.
4. Definition and fingerprint validation, with saved names resolved to current IDs.
5. SET_MASK and GET_MASK.
6. START_LOG.

IDs are scoped to the reported definitions fingerprint. An invalid cache falls
back to discovery. Invalid device definitions abort initialization and are not
cached.

## Device info

`MSG_GET_DEVICE_INFO = 0x01` has no payload.
`MSG_DEVICE_INFO_REPLY = 0x02` has 29 bytes, with Python layout
`<BBBBHHHBBIHHQB`:

| Offset | Type | Meaning |
| --- | --- | --- |
| 0 | u8 | Protocol version (3) |
| 1–3 | 3 × u8 | Firmware major, minor, patch |
| 4 | u16 | Signal count |
| 6 | u16 | Variable count |
| 8 | u16 | PID controller count |
| 10 | u8 | Mask width: ceil(signal count / 8) |
| 11 | u8 | Maximum simultaneously selected signals |
| 12 | u32 | Telemetry samples per second |
| 16 | u16 | Maximum command payload bytes |
| 18 | u16 | Maximum reply payload bytes |
| 20 | u64 | Definitions fingerprint |
| 28 | u8 | Status: 0 valid, 1 duplicate definition names |

Device-info polling refreshes firmware version and connection health without
restarting discovery or logging. Changes to capabilities or the definitions
fingerprint require a new connection initialization.

Protocol 3 has no separate firmware-version commands. Its command IDs are not
compatible with earlier versions: 0x00 is ENTER_BOOTLOADER, not a version request.

## Definition pages

`MSG_GET_DEFINITIONS = 0x03` has three bytes: `kind:u8 | start:u16`.
Kinds are 0 signals, 1 variables, and 2 PID controllers. Start is the zero-based
entry index. IDs are assigned by entry order, starting at zero independently
for each list.

`MSG_DEFINITIONS_REPLY = 0x04` starts with the seven-byte header
`kind:u8 | start:u16 | total:u16 | returned:u16`, followed by UTF-8 text.
The current firmware bounds each reply payload to 256 bytes, including header.
Only complete rows are returned. The next page starts at `start + returned`;
pagination ends when that index equals total. Each page of a nonempty list
contains at least one row. Empty variable and PID lists return total=returned=0
for start=0. At least one signal is required.

Signal and variable rows are `id<TAB>type<TAB>name<LF>`.
PID rows are `id<TAB>name<LF>`. Types are `f`, `u32`, and `i32`; each is four bytes
on the wire. Names are stringified C member paths, up to 127 printable ASCII
bytes. Tabs, newlines, spaces, and control characters cannot occur in a name.
Repeated names within a list, repeated/out-of-order IDs, unknown types,
wrong counts, partial rows, and unexpected kind/start values are invalid.
A name may occur once in each of multiple lists.

`MSG_DEFINITIONS_ERROR = 0x05` has four bytes:
`kind:u8 | requested_start:u16 | error:u8`.
Error 1 means unknown list, 2 means index out of range, and 3 means invalid
driver definitions. Invalid payload lengths use `MSG_INVALID_PAYLOAD`.
An error response is distinct from a valid empty-list response.

## Fingerprint and caching

The fingerprint is 64-bit FNV-1a, initialized to 14695981039346656037.
Each UTF-8 byte updates the hash as
`hash = ((hash XOR byte) * 1099511628211) mod 2^64`.
The input stream has no NUL terminators and consists of:

1. `signals<LF>` followed by all canonical signal rows.
2. `variables<LF>` followed by all canonical variable rows.
3. `pids<LF>` followed by all canonical PID rows.

IDs use ordinary decimal notation without leading zeros; line endings are LF.
The stream includes kind, order, IDs, wire types, and names. It excludes memory
offsets, current values, simulator defaults, and firmware version. This is a
change detector, not an authenticity/security mechanism.

Cached definitions are reusable only for the same protocol version and
fingerprint, with counts, IDs, unique names, types, and the recomputed fingerprint
matching device info. A cache miss or invalid cache triggers discovery.
Saved selections contain names, which resolve to the current IDs during
initialization. Cache storage locations are documented in the plotter README.

## Mask and storage limits

The current implementation supports up to 256 signals and 256 variables/PIDs
per list. Variables and PID IDs are u8; discovery indexes/counts are u16 so a
full 256-entry list can be represented. Mask bytes are little endian, and all
unused high bits must be zero. Selected-signal limits are checked independently
of mask width.

All numeric values occupy four bytes on the wire, including values backed by
smaller integer fields. Writes outside the underlying field's range are
rejected. Memory offsets and native field sizes are not transmitted.

## Shared state, control-mode, and error values

These values are used by both USB and CAN. They are separate from the discovered
signal/variable/PID IDs and are defined in FOC_States.h, FOC_Loops.h, and
FOC_Diagnostics.h.

| State | ID | State | ID |
| --- | --- | --- | --- |
| NONE (not a transition target) | 0 | ALIGNMENT | 10 |
| INIT | 1 | ALIGNMENT_TEST | 11 |
| RESET | 2 | RUN | 12 |
| BOOTUP_SOUND | 3 | STOP | 13 |
| CURRENT_SENSOR_CALIBRATION | 4 | IDLE | 14 |
| IDENTIFY | 5 | FLASH_SAVE | 15 |
| ANTICOGGING | 6 | FLASH_LOAD | 16 |
| CHECKLIST | 7 | FLASH_CLEAR | 17 |
| PID_AUTOTUNE | 8 | BOOTLOADER | 18 |
| ERROR | 9 | OPENLOOP | 19 |

FOC_STATE_COUNT (20) is a sentinel, not a runtime state. Actual state transitions
are guarded by FOC_SetState and the destination transition function. In
particular, IDLE requires STOP and absolute mechanical speed at most 0.1 rad/s.

| Control mode | ID |
| --- | --- |
| CONTROL_MODE_OPENLOOP (direct d/q current targets) | 0 |
| CONTROL_MODE_POSITION | 1 |
| CONTROL_MODE_SPEED | 2 |

The OPENLOOP control-mode label is distinct from FOC_STATE_OPENLOOP, which runs
the separate open-loop voltage routine.

| Error bit | Name |
| --- | --- |
| 0 | FOC_ERROR_ESTOP |
| 1 | FOC_ERROR_SYSTEM_FAULT |
| 2 | FOC_ERROR_TIMING_VIOLATION |
| 3 | FOC_ERROR_INVALID_CONFIGURATION |
| 4 | FOC_ERROR_DRIVER_FAULT |
| 5 | FOC_ERROR_MOTOR_OT |
| 6 | FOC_ERROR_MOTOR_UT |
| 7 | FOC_ERROR_MOTOR_DISCONNECTED |
| 8 | FOC_ERROR_MOTOR_STALL |
| 9 | FOC_ERROR_MOSFET_OT |
| 10 | FOC_ERROR_MOSFET_UT |
| 11 | FOC_ERROR_VBUS_OV |
| 12 | FOC_ERROR_VBUS_UV |
| 13 | FOC_ERROR_IBUS_OC |
| 14 | FOC_ERROR_ENCODER_FAULT |

Not all defined error bits are set by the current diagnostic paths. Latched
faults can reappear after clearing if the underlying fault remains present.
