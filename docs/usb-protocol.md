# USB protocol reference (v11)

Calibration control, progress and editable-table payloads are specified in
[Calibration table transfers](calibration-tables.md).

Frames use `AA 55 | message:u8 | payload_length:u16le | payload`.
All multi-byte integers and four-byte numeric values are little endian.
Maximum command payload is currently 123 bytes; replies allow 1019 bytes.
Device info reports the signal-mask width and telemetry limits.

The transport is USB CDC (a serial/COM port on the host). A host read or device
receive callback is not a packet boundary: binary frames may be split across
chunks or several frames may arrive together. Maximum command frame size is
128 bytes including the header. A receive gap longer than 250 ms discards a
partial command. Unframed text requires CR or LF; binary text replies
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
| 0x28 | `MSG_GET_ENCODER_INFO` |
| 0x29 | `MSG_ENCODER_INFO_REPLY` |
| 0x2A | `MSG_SET_ENCODER_SOURCE` |
| 0x2B | `MSG_SET_ENCODER_CORRECTION` |
| 0x2C | `MSG_GET_CALIBRATION_INFO` |
| 0x2D | `MSG_CALIBRATION_INFO_REPLY` |
| 0x2E | `MSG_START_CALIBRATION` |
| 0x2F | `MSG_ENABLE_CALIBRATION` |
| 0x30 | `MSG_GET_CALIBRATION_TABLE` |
| 0x31 | `MSG_CALIBRATION_TABLE_REPLY` |
| 0x32 | `MSG_BEGIN_CALIBRATION_UPLOAD` |
| 0x33 | `MSG_WRITE_CALIBRATION_TABLE` |
| 0x34 | `MSG_COMMIT_CALIBRATION_UPLOAD` |
| 0x35 | `MSG_ABORT_CALIBRATION_UPLOAD` |
| 0x36 | `MSG_GET_CONFIGURATION` |
| 0x37 | `MSG_CONFIGURATION_REPLY` |
| 0x38 | `MSG_SET_CONFIGURATION` |
| 0x39 | `MSG_SET_TARGET` |
| 0x3A | `MSG_GET_TARGETS` |
| 0x3B | `MSG_TARGETS_REPLY` |

Response/error IDs occupy 0xFA through 0xFF.

## Payloads and command behavior

H→D means host to driver; D→H means driver to host. Fields are listed in wire
order. `f32` means IEEE-754 32-bit float, and `u8`, `u16`, and `u32` are unsigned
integers of the stated widths. A variable's four-byte value is interpreted
using its discovered type (`f`, `u32`, or `i32`). Empty means zero payload bytes.

| Command | Direction | Payload | Response or behavior |
| --- | --- | --- | --- |
| MSG_ENTER_BOOTLOADER | H→D | Empty | Requests BOOTLOADER from IDLE or ERROR; ACK if accepted, ERROR if denied |
| MSG_GET_DEVICE_INFO | H→D | Empty | DEVICE_INFO_REPLY; see detailed layout below |
| MSG_DEVICE_INFO_REPLY | D→H | 34 bytes | Versions, five list counts, limits, rate, fingerprint, status and debug-write capability |
| MSG_GET_DEFINITIONS | H→D | kind:u8, start:u16 (3 bytes) | DEFINITIONS_REPLY or DEFINITIONS_ERROR |
| MSG_DEFINITIONS_REPLY | D→H | 7-byte page header and UTF-8 rows | See discovery rules below |
| MSG_DEFINITIONS_ERROR | D→H | Command-error payload | Explicit discovery failure; includes kind/start in text |
| MSG_LOG_DATA | D→H | timestamp:u32, samples:u16, signals:u16, sample words | Sample-major telemetry; see below |
| MSG_SET_MASK | H→D | mask:ceil(signal_count / 8) bytes | ACK if valid and logging is stopped; otherwise ERROR |
| MSG_GET_MASK | H→D | Empty | MASK_REPLY |
| MSG_MASK_REPLY | D→H | Current mask bytes | Same width as reported by device info |
| MSG_START_LOG | H→D | Empty | Discards any partial packet, enables capture, ACK |
| MSG_STOP_LOG | H→D | Empty | Disables capture, discards any partial packet, ACK |
| MSG_SET_PID | H→D | id:u8, Kp:f32, Ki:f32, Kd:f32 (13 bytes) | Applies gains to discovered controller ID, ACK |
| MSG_GET_PID | H→D | id:u8 (1 byte) | PID_REPLY |
| MSG_PID_REPLY | D→H | id:u8, Kp:f32, Ki:f32, Kd:f32 (13 bytes) | Current gains |
| MSG_SET_VAR | H→D | id:u8, value:4 bytes (5 bytes) | Development-only raw write; disabled in production; native integer width checked |
| MSG_GET_VAR | H→D | id:u8 (1 byte) | VAR_REPLY |
| MSG_VAR_REPLY | D→H | id:u8, value:4 bytes (5 bytes) | Current typed value |
| MSG_FLASH_SAVE | H→D | Empty | Requests FLASH_SAVE from IDLE or ERROR; ACK or ERROR |
| MSG_FLASH_LOAD | H→D | Empty | Requests FLASH_LOAD from IDLE or ERROR; ACK or ERROR |
| MSG_FLASH_CLEAR | H→D | Empty | Requests FLASH_CLEAR from IDLE or ERROR; ACK or ERROR |
| MSG_SET_STATE | H→D | state:u8 (1 byte) | Requests a state transition; ACK or ERROR |
| MSG_GET_STATE | H→D | Empty | STATE_REPLY |
| MSG_STATE_REPLY | D→H | state:u8 (1 byte) | Current state enum |
| MSG_TEXT_COMMAND | H→D | Bounded command text, up to 123 bytes | Development-only text parsing; production returns ERROR |
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
| MSG_SET_CONTROL_MODE | H→D | mode:u8 (1 byte) | Validates mode, IDLE/RUN, faults and upload state; ACK or structured ERROR |
| MSG_GET_CONTROL_MODE | H→D | Empty | CONTROL_MODE_REPLY |
| MSG_CONTROL_MODE_REPLY | D→H | mode:u8 (1 byte) | Current control mode |
| MSG_UNKNOWN_TYPE (0xFA) | D→H | Command-error payload | Command ID is not supported |
| MSG_INVALID_PAYLOAD (0xFB) | D→H | Command-error payload | Wrong payload length, oversized command, invalid scalar range, or unsupported control mode |
| MSG_UNKNOWN_ID (0xFC) | D→H | Command-error payload | Variable/PID ID is outside its discovered list |
| MSG_BUFFER_OVERFLOW (0xFD) | D→H | Command-error payload | Unframed input exceeded the receive-buffer capacity |
| MSG_ACK (0xFE) | D→H | Empty | Command accepted; no request ID is included |
| MSG_ERROR (0xFF) | D→H | Command-error payload | Command rejected by state/mask rules, or invalid driver definitions prevent configuration |

Payload lengths are checked for the fixed-layout commands above. USB node-ID
commands validate the range 0–15; USB CAN cyclic-rate commands validate index
0–8, as listed in the [CAN reference](can-protocol.md#cyclic-replies-and-timing).

Production configuration/target layouts and validation are specified in
[Validated configuration](configuration.md). SET_PID validates complete gain sets
in firmware. SET_VAR remains unsafe development access and is disabled, along
in non-debug builds by default.
Control-mode selection resets the target associated with the newly selected
mode when the mode changes; it does not enter RUN. Flash actions and bootloader
entry are state-machine requests: an ACK confirms acceptance, not completion.
Flash actions and bootloader entry are accepted from IDLE or ERROR and enforce
high impedance with zero phase voltage. Fault polling continues to latch faults
without preempting these maintenance states. Flash actions from IDLE request
RUN after success; a denied transition falls back to STOP. Flash actions from
ERROR preserve faults and return to ERROR while faults remain, or STOP if they
have been cleared; they never automatically resume RUN. A failed flash operation
enters ERROR without attempting RUN. Bootloader entry remains available with
active or latched faults.
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

### Text commands

Text is available only when FOC_ENABLE_DEBUG_WRITES is enabled.
Text parsing is case-sensitive. Raw text requires CR or LF; text inside MSG_TEXT_COMMAND uses
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

Numeric arguments are integers, not floating-point text. These commands
can alter state/configuration directly and do not share all plotter UI
restrictions. There is no separate firmware help/version text API.

## Initialization

Connection initialization follows this sequence:

1. Device-info request and protocol/status validation.
2. STOP_LOG.
3. Loading of validated cached definitions for the reported protocol/fingerprint,
   or discovery of all five lists.
4. Definition and fingerprint validation, with saved names resolved to current IDs.
5. SET_MASK and GET_MASK.
6. START_LOG.

IDs are scoped to the reported definitions fingerprint. An invalid cache falls
back to discovery. Invalid device definitions abort initialization and are not
cached.

## Device info

`MSG_GET_DEVICE_INFO = 0x01` has no payload.
`MSG_DEVICE_INFO_REPLY = 0x02` has 34 bytes, with Python layout
`<BBBBHHHBBIHHQBHHB`:

| Offset | Type | Meaning |
| --- | --- | --- |
| 0 | u8 | Protocol version (14) |
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
| 28 | u8 | Status: 0 valid, 1 invalid/duplicate definition names |
| 29 | u16 | CAN cyclic-message count |
| 31 | u16 | Configuration-field count |
| 33 | u8 | Raw debug writes enabled: 0 no, 1 yes |

Device-info polling refreshes firmware version and connection health without
restarting discovery or logging. Changes to capabilities or the definitions
fingerprint require a new connection initialization.

Only USB protocol 14 is supported. Firmware and plotter must be updated together.
Unknown or older versions are rejected before discovery. There are no alternate
device-info layouts. The text command interface remains available independently
of binary discovery in development builds. Adding entries within the current
list schema changes its fingerprint without requiring a protocol bump.

### Encoder source and calibration control

GET_ENCODER_INFO has an empty payload. ENCODER_INFO_REPLY contains:

`selected:u8 | flags:u8 | calibration_stage:u8 | error:u8 | progress:u16le | count:u8`

followed by `count` entries: `source_id:u8 | capabilities:u8 | name_length:u8 | ASCII name`.
Names are printable ASCII, 1-31 bytes; IDs/names are unique and count is 1-8.
Capability bit 0 indicates calibration support. Flags bits 0-3 indicate valid
correction, enabled correction, valid alignment, and source health. The selected
ID can be unavailable in a failed configuration. Progress is 0-1000. Stages are
0 idle, 1 lock, 2 apply, 3 sample, 4 validate, 5 save, 6 done, 7 failed, 8 cancelled.
Routine error codes are 0 none, 1 not started, 2 invalid stage, 3 invalid input,
4 timeout, 5 encoder, 6 current sense, 7 invalid result.

SET_ENCODER_SOURCE accepts one source-ID byte. SET_ENCODER_CORRECTION accepts
one byte (0/1). Both require stationary IDLE/ERROR and return ACK/structured error;
GET_ENCODER_INFO reads confirmed settings. Normal flash save persists selection.
A source initialization failure can leave the chosen source selected but unhealthy.

SET_STATE value 20 requests encoder calibration from fault-free stationary IDLE
only. ACK confirms acceptance, not completion. Poll encoder info/state for results.
SET_STATE STOP cancels measurement. Success saves the table and matching zero,
invalidates anti-cogging and enters STOP. See [Encoder calibration](encoder-calibration.md)
for motion requirements, source contracts, validation and flash compatibility.

## Definition pages

`MSG_GET_DEFINITIONS = 0x03` has three bytes: `kind:u8 | start:u16`.
Kinds are 0 signals, 1 variables, 2 PID controllers, 3 CAN cyclic messages, and
4 configuration fields ([metadata layout](configuration.md#catalog-and-absolute-bounds)). Start is the zero-based
entry index. IDs are assigned by entry order, starting at zero independently
for each list.

`MSG_DEFINITIONS_REPLY = 0x04` starts with the seven-byte header
`kind:u8 | start:u16 | total:u16 | returned:u16`, followed by UTF-8 text.
The current firmware bounds each reply payload to 256 bytes, including header.
Only complete rows are returned. The next page starts at `start + returned`;
pagination ends when that index equals total. Each page of a nonempty list
contains at least one row. Empty variable, PID and CAN lists return total=returned=0
for start=0. At least one signal is required.

Signal and variable rows are `id<TAB>type<TAB>name<LF>`.
PID rows are `id<TAB>name<TAB>minimum<TAB>maximum<TAB>writable_states<LF>`.
The same gain bounds apply to Kp, Ki and Kd; writable states are a u32 bitmask.
PID metadata participates in the canonical fingerprint.
CAN cyclic rows are `id<TAB>name<LF>`. Types are `f`, `u32`, and `i32`; each is four bytes
on the wire. Names are stringified C member paths, up to 127 printable ASCII
bytes. Tabs, newlines, spaces, and control characters cannot occur in a member-path
name. CAN names are display labels from `FOC_CAN_CYCLIC_LIST`: internal ASCII
spaces are permitted, but leading/trailing spaces, control characters and
non-ASCII characters are rejected. Names must be unique within their list.
Repeated names within a list, repeated/out-of-order IDs, unknown types,
wrong counts, partial rows, and unexpected kind/start values are invalid.
A name may occur once in each of multiple lists.

`MSG_DEFINITIONS_ERROR = 0x05` uses the common command-error payload below.
Its original command is GET_DEFINITIONS. Codes 18, 19, and 16 indicate an unknown
list, an out-of-range index, and invalid definitions respectively. The text
includes the requested kind/index. Invalid payload lengths use
`MSG_INVALID_PAYLOAD`. An error response is distinct from a valid empty list.

## Command errors

All error response types (0x05 and 0xFA–0xFD/0xFF) use one self-contained payload:

```text
original_command:u8 | error_code:u16le | message:UTF-8
```

The message is 1–160 bytes with no terminating NUL. Total payload length is
4–163 bytes; frame length determines message length. Driver-generated messages
are bounded ASCII text and may be truncated to 160 bytes. Codes are independent
of the active/latched device-fault bitmaps and discovered field IDs.

| Code | Name | Meaning |
| --- | --- | --- |
| 1 | UNKNOWN_COMMAND | Unsupported request identifier |
| 2 | INVALID_LENGTH | Wrong or oversized command payload |
| 3 | UNKNOWN_ID | Unknown variable, PID, or cyclic index |
| 4 | OUT_OF_RANGE | Value cannot fit storage, or invalid node/control-mode value |
| 5 | INVALID_STATE | Invalid or denied target state |
| 6 | ADC_NOT_CALIBRATED | ADC calibration prevents RUN |
| 7 | ENCODER_NOT_ALIGNED | Encoder alignment prevents RUN |
| 8 | ACTIVE_FAULTS | Active faults prevent RUN |
| 9 | LATCHED_FAULTS | Latched faults prevent RUN |
| 10 | MUST_BE_IDLE | Flash/bootloader operation requires IDLE or ERROR |
| 11 | MUST_BE_STOPPED | IDLE requires the STOP state |
| 12 | MOTOR_MOVING | Speed prevents entry into IDLE |
| 13 | LOGGING_ACTIVE | Signal mask cannot change while logging |
| 14 | INVALID_MASK | Unsupported signal bits |
| 15 | TOO_MANY_SIGNALS | Selection exceeds the device limit |
| 16 | INVALID_DEFINITIONS | Duplicate definition names |
| 17 | RECEIVE_OVERFLOW | Unframed input exceeded receive capacity |
| 18 | UNKNOWN_DEFINITION_LIST | Unsupported discovery kind |
| 19 | DEFINITION_INDEX | Discovery start index is out of range |
| 20 | INTERNAL | Internal command/state-transition failure |

For example, an encoder-alignment rejection of SET_STATE has command 0x15,
code 7, and text describing the denial. State-transition errors also include
target/current states and fault bitmaps within the text limit. Explanations are
carried with the error rather than inferred from preceding TEXT_REPLY messages.

The plotter keeps one command outstanding and checks that the error's command
matches that request. Malformed or mismatched errors stop the session. Ordinary
runtime rejections are displayed without disconnecting; initialization errors
and receive-buffer overflow remain fatal. Unknown numeric codes remain
displayable using their numeric value and driver text.

Unframed overflow has original_command=0xFF because no command can be identified.
Complete command frames are processed in receive order, including concatenated
frames; each error is built from that frame's command context. There are no
request sequence IDs, automatic retries, or new asynchronous completion replies.
An ACK remains empty and confirms command acceptance. Flash completion/failure
and later state changes remain asynchronous diagnostics.

## Fingerprint and caching

The fingerprint is 64-bit FNV-1a, initialized to 14695981039346656037.
Each UTF-8 byte updates the hash as
`hash = ((hash XOR byte) * 1099511628211) mod 2^64`.
The input stream has no NUL terminators and consists of:

1. `signals<LF>` followed by all canonical signal rows.
2. `variables<LF>` followed by all canonical variable rows.
3. `pids<LF>` followed by all canonical PID rows.
4. `can_cyclic<LF>` followed by all canonical CAN cyclic rows.

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

The current implementation supports up to 256 entries in each list, including
CAN cyclic messages. Variables, PID and CAN cyclic IDs are u8; discovery indexes/counts are u16 so a
full 256-entry list can be represented. Mask bytes are little endian, and all
unused high bits must be zero. Selected-signal limits are checked independently
of mask width.

All numeric values occupy four bytes on the wire, including values backed by
smaller integer fields. Writes outside the underlying field's range are
rejected. Memory offsets and native field sizes are not transmitted.

## Shared state, control-mode, and error values

### Retained test/calibration result

`MSG_GET_ROUTINE_RESULT` (0x3C, empty payload) returns
`MSG_ROUTINE_RESULT_REPLY` (0x3D, UTF-8 text, up to 767 bytes). This read-only
additive command is available in production builds; firmware may reject
it as unknown. The last terminal routine result is retained in RAM until another
routine finishes or the device reboots. It includes routine name, outcome,
failure reason, failed stage, elapsed time and fault masks. Fault-driven cleanup
explicitly says it was interrupted by a fault; user STOP says cancelled.

Covered operations: current-sensor calibration, encoder alignment, encoder
direction test, encoder correction calibration, R/L identification, current PI
tuning and anti-cogging. Reports are also sent once through MSG_TEXT_REPLY.
Encoder failures distinguish missing sweep coverage, excessive tracking error,
forward/reverse disagreement, invalid correction mapping, invalid measurements,
current limit, configuration changes and flash save failure.

In stopped/IDLE/ERROR states the query additionally attempts a validated AS5047P
DIAAGC read. MAGL (bit 11) means weak magnetic field, MAGH (bit 10) strong field,
COF (bit 9) unreliable angle and LF (bit 8) completed offset compensation.
These are snapshots at readback time, not recorded proof of the magnetic field
at the instant of failure. No SPI diagnostics are inserted into the running
sample path. Busy SPI, bad parity, sensor error flags or unavailable sources are
reported as unknown/unavailable, never as a healthy magnet. The plotter displays
the report in Calibration > Last test / calibration result and reads it on
entry to IDLE/ERROR; the read button supports manual retry and reconnects.

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
| ENCODER_CALIBRATION | 20 | | |

FOC_STATE_COUNT (21) is a sentinel, not a runtime state. Actual state transitions
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
| 15 | FOC_ERROR_OPERATION_FAILED |
| 16 | FOC_ERROR_PHASE_OC |

ERROR requires at least one latched fault. Active faults are reevaluated from
the hardware/measurements and latched automatically. State-changing requests
cannot bypass active or latched faults. Entering ERROR cancels any pending
next state and immediately disables motor output.

CLEAR_LATCHED_ERRORS acknowledges stored faults and immediately reevaluates the
active conditions. Present faults relatch and retain ERROR. When both masks are
clear, ERROR transitions to STOP, never directly to RUN. The normal STOP logic
then enters IDLE once stationary.

OPERATION_FAILED (bit 15, 0x00008000) is the common latched event for failures in
current-sensor calibration, identification, PID autotune, alignment/alignment
test, anticogging, and flash save/load/clear. Diagnostic text identifies the
particular operation; clearing the event does not retry that operation.
Other defined error bits may not be set by the current diagnostic paths.

PHASE_OC (bit 16, 0x00010000, "Phase over-current" in the plotter) detects
either polarity of measured phase current exceeding `max_dq_current`, once ADC
calibration is complete. It is checked each measurement cycle, including during
current-controlled encoder calibration. Encoder/anti-cogging current-limit failures
also report this specific latched fault rather than only OPERATION_FAILED.
It remains latched after output is disabled and current falls; acknowledgement
while the condition is present immediately relatches it. IBUS_OC remains the
separate bus-current fault. All faults reported with FOC_SetError set the same
bits in both active and latched masks, including one-shot software faults.

## Protocol 12 motor Setup additions

See [Motor Setup](motor-setup.md) for commissioning prerequisites, fail-stop
sequencing and combined direction/pole detection. State 21 is MOTOR_PROBE;
COUNT becomes 22. Commands 0x3E/0x3F get/return a correlated 20-byte setup status;
0x40 sets runtime detection/alignment voltage. Existing IDs are unchanged.
