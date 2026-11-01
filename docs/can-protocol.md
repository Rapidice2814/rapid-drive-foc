# CAN protocol reference

CAN provides motor control, configuration, device addressing, and status
messages. Its command IDs are independent of USB. Firmware version is reported
by CAN_GET_VERSION / CAN_VERSION_REPLY.

## Bus and identifier layout

The bus uses CAN FD with bit-rate switching (BRS) and standard 11-bit
identifiers. Replies are FD/BRS data frames, including replies with eight bytes
or fewer.

| Parameter | Value |
| --- | --- |
| Nominal bit rate | 1 Mbit/s |
| Data bit rate | 1 Mbit/s |

The identifier stores the command in the high seven bits and the node in the
low four bits:

```text
bits 10..4: command (0..127)
bits  3..0: node ID (0..15)
identifier = ((command & 0x7F) << 4) | (node_id & 0x0F)
```

For example, GET_STATE (0x09) addressed to node 3 has identifier `0x093`;
STATE_REPLY (0x0A) from node 3 has identifier `0x0A3`. There is no USB-style
start marker or length prefix inside the CAN payload; DLC carries its length.
Multi-byte integers and IEEE-754 32-bit floats are little endian.

Node 0 is broadcast/unassigned. Individual devices use unique node IDs 1–15.
Receive filters accept node 0 and the configured node. The dispatcher accepts
only ESTOP, GET_ADDRESS, and SET_ADDRESS at node 0; other broadcast commands are
ignored. Cyclic transmission is disabled while the local node ID is zero.

## Command and payload reference

H→D means host to driver; D→H means driver to host. `f32` is a four-byte float;
`u8`, `u16`, and `u32` are unsigned integers of the stated widths. All payloads
below are at most eight bytes. Empty means DLC 0. The documented payload length
is part of each command's format, including commands without length validation.

| ID | Command | Direction | Payload | Current behavior |
| --- | --- | --- | --- | --- |
| 0x00 | CAN_ESTOP | H→D | Empty | Sets/latches FOC_ERROR_ESTOP and enters ERROR; no reply |
| 0x01 | CAN_GET_VERSION | H→D | Empty | Requests CAN_VERSION_REPLY |
| 0x02 | CAN_VERSION_REPLY | D→H | major:u8, minor:u8, patch:u8 (3 bytes) | Firmware version |
| 0x03 | CAN_REBOOT | H→D | Empty | Declared but unimplemented; no action or reply |
| 0x04 | CAN_ENTER_BOOTLOADER | H→D | Empty | Declared but unimplemented; no action or reply |
| 0x05 | CAN_SET_ADDRESS | H→D | node:u8, UID:6 bytes (7 bytes) | Changes address according to the UID rules below; no ACK |
| 0x06 | CAN_GET_ADDRESS | H→D | Empty | Schedules CAN_ADDRESS_REPLY after 1–800 FOC ticks |
| 0x07 | CAN_ADDRESS_REPLY | D→H | node:u8, UID:6 bytes (7 bytes) | Configured address and device UID bytes |
| 0x08 | CAN_SET_STATE | H→D | state:u8 (1 byte) | Requests a state transition; ACK if accepted, ERROR if denied |
| 0x09 | CAN_GET_STATE | H→D | Empty | Requests CAN_STATE_REPLY |
| 0x0A | CAN_STATE_REPLY | D→H | state:u8 (1 byte) | Current FOC state |
| 0x0B | CAN_SET_CONTROL_MODE | H→D | mode:u8 (1 byte) | Calls FOC_SetControlMode; return status is ignored and no reply is sent |
| 0x0C | CAN_GET_CONTROL_MODE | H→D | Empty | Requests CAN_CONTROL_MODE_REPLY |
| 0x0D | CAN_CONTROL_MODE_REPLY | D→H | mode:u8 (1 byte) | Current control mode |
| 0x0E | CAN_GET_HEARTBEAT | H→D | Empty | Requests CAN_HEARTBEAT_REPLY |
| 0x0F | CAN_HEARTBEAT_REPLY | D→H | state:u8, motor_temp:u8, tick:u16 (4 bytes) | Temperature is clamped to 0–255 °C and truncated; tick is the low 16 bits |
| 0x10 | CAN_GET_ENCODER_ESTIMATES | H→D | Empty | Requests encoder estimates |
| 0x11 | CAN_ENCODER_ESTIMATES_REPLY | D→H | angle:f32, speed:f32 (8 bytes) | Unwrapped mechanical angle (rad), mechanical speed (rad/s) |
| 0x12 | CAN_GET_BUS_VOLTGE_CURRENT | H→D | Empty | Requests bus measurements; identifier spelling is retained from the source |
| 0x13 | CAN_BUS_VOLTAGE_CURRENT_REPLY | D→H | voltage:f32, current:f32 (8 bytes) | Bus voltage (V), measured bus current (A) |
| 0x14 | CAN_GET_TEMPERATURES | H→D | Empty | Requests temperatures |
| 0x15 | CAN_TEMPERATURES_REPLY | D→H | mosfet_temp:f32, motor_temp:f32 (8 bytes) | Temperatures in °C, in this order |
| 0x16 | CAN_SET_TORQUE | H→D | torque:f32 (4 bytes) | Validated target in Nm; Iq = torque / torque_constant (Nm/A) |
| 0x17 | CAN_GET_TORQUE | H→D | Empty | Requests CAN_TORQUE_REPLY |
| 0x18 | CAN_TORQUE_REPLY | D→H | target:f32, measured:f32 (8 bytes) | Target/measured q-current × torque_constant, in Nm |
| 0x19 | CAN_SET_CURRENT | H→D | q_current:f32 (4 bytes) | Sets q-axis current target (A) |
| 0x1A | CAN_GET_CURRENT | H→D | Empty | Requests CAN_CURRENT_REPLY |
| 0x1B | CAN_CURRENT_REPLY | D→H | target:f32, measured:f32 (8 bytes) | q-axis target and measured current (A) |
| 0x1C | CAN_SET_SPEED | H→D | speed:f32 (4 bytes) | Sets mechanical speed target (rad/s) |
| 0x1D | CAN_GET_SPEED | H→D | Empty | Requests CAN_SPEED_REPLY |
| 0x1E | CAN_SPEED_REPLY | D→H | target:f32, measured:f32 (8 bytes) | Mechanical target and measured speed (rad/s) |
| 0x1F | CAN_SET_POSITION | H→D | angle:f32 (4 bytes) | Sets mechanical angle target (rad) |
| 0x20 | CAN_GET_POSITION | H→D | Empty | Requests CAN_POSITION_REPLY |
| 0x21 | CAN_POSITION_REPLY | D→H | target:f32, measured:f32 (8 bytes) | Target and unwrapped mechanical angle (rad) |
| 0x22 | CAN_GET_ERRORS | H→D | Empty | Requests CAN_ERRORS_REPLY |
| 0x23 | CAN_ERRORS_REPLY | D→H | active:u32, latched:u32 (8 bytes) | Error bitmaps |
| 0x24 | CAN_CLEAR_LATCHED_ERRORS | H→D | Empty | Clears latched errors; no reply |
| 0x3E | CAN_ACK | D→H | Empty | State request accepted, not necessarily completed |
| 0x3F | CAN_ERROR | D→H | Empty | State/control-mode/target request rejected |

State, control-mode, and error-bit values are shared with USB and listed in the
[shared value reference](usb-protocol.md#shared-state-control-mode-and-error-values).
Setpoint writes update targets but do not select a control mode or enter RUN.
Targets and control modes share the USB firmware validation functions; target
writes require fault-free RUN and matching mode, finite values and configured/
absolute bounds. Rejected targets leave the previous value unchanged. Successful
target writes retain the no-ACK behavior to avoid adding traffic to control
streams; rejected ones emit CAN_ERROR. Detailed reason codes/text are available
on USB, not in the empty CAN_ERROR payload. Absolute bounds are in FOC_Config.h.
The torque conversion now consistently uses Nm/A; hosts relying on the former
inverted conversion must update their units. Configuration catalog discovery and
atomic grouped writes currently use USB; there is no CAN configuration command.

## Address assignment

GET_ADDRESS can be broadcast. Its reply uses the current node ID and is delayed
by `random_number % 800 + 1` FOC ticks, so several drivers need not reply at
exactly the same instant. The delay is not milliseconds and does not guarantee
collision-free discovery. Only one delayed-message slot exists per handle;
another queued request replaces its contents.

SET_ADDRESS contains the desired node ID and the six UID bytes reported in
ADDRESS_REPLY:

- A matching UID or six zero UID bytes applies the supplied node ID.
- A nonmatching UID with a requested node equal to the current node sets that
  device to node 0.
- Other nonmatching requests leave the address unchanged.

A device UID targets an individual device; the zero-UID wildcard applies to
every receiving device. Valid node IDs are 0–15; receive length and native setter
range checks reject invalid addresses. Address changes take effect immediately. Persistence uses USB
FLASH_SAVE.

## Cyclic replies and timing

Cyclic periods are configured through USB SET_CAN_CYCLIC_RATE or the stored
node settings. There is no separate CAN command to set these periods.

`FOC_CAN_CYCLIC_LIST` in `Foc/FOC_Drivers/FOC_CAN.h` defines the cyclic symbols,
display names and reply commands once. Array order generates the cyclic IDs and
the scheduler mapping. USB discovery exposes the names/IDs through definition
kind 3; the CAN tab is built from that catalog at startup or from its validated
fingerprint cache. Adding a cyclic entry requires no Python catalog update.
Periods use the sample-clock rate reported by device info, not a host constant.

The existing entries below retain their IDs. New entries should be appended:
cyclic indexes also address persisted period arrays. Reordering/removing entries
requires an explicit settings migration or flash-format change. Count changes
alter the stored record size; old-size records are rejected rather than read
using a different layout. The current registry refactor preserves flash layout.

| Cyclic index | Selector | Reply |
| --- | --- | --- |
| 0 | CAN_CYCLIC_HEARTBEAT | CAN_HEARTBEAT_REPLY |
| 1 | CAN_CYCLIC_ENCODER_ESTIMATES | CAN_ENCODER_ESTIMATES_REPLY |
| 2 | CAN_CYCLIC_BUS_VOLTAGE_CURRENT | CAN_BUS_VOLTAGE_CURRENT_REPLY |
| 3 | CAN_CYCLIC_TEMPERATURES | CAN_TEMPERATURES_REPLY |
| 4 | CAN_CYCLIC_TORQUE | CAN_TORQUE_REPLY |
| 5 | CAN_CYCLIC_CURRENT | CAN_CURRENT_REPLY |
| 6 | CAN_CYCLIC_SPEED | CAN_SPEED_REPLY |
| 7 | CAN_CYCLIC_POSITION | CAN_POSITION_REPLY |
| 8 | CAN_CYCLIC_ERRORS | CAN_ERRORS_REPLY |

A zero period disables the entry. Nonzero periods are measured in FOC ticks.
The tick rate is reported by USB device info as the telemetry sample rate.
The loop sends a due entry once, advances its last-attempt tick, and does not
backfill missed periods.

## Current implementation limits

- State changes emit ACK or ERROR; rejected target/control-mode writes emit ERROR.
  Other successful writes generally have no acknowledgement. These empty replies
  contain no originating command ID or detailed error reason.
- REBOOT and ENTER_BOOTLOADER are unimplemented; neither command performs an
  action or produces a reply.
- Payload-length checks remain command-specific. Malformed frames do not produce a consistent protocol
  error response.
- Torque commands require a valid positive Nm/A torque constant; telemetry reports
  current multiplied by that constant. Debug writes can still corrupt configuration
  in development builds and are not a supported production interface.
- FIFO-full replies are dropped and counted in missed_can_messages. There is
  no application-level retry, sequence number, or request correlation.
- Receive processing polls at most one frame per service call; the FIFO callback
  currently does no processing. The protocol is not a guaranteed-latency stop
  mechanism.
