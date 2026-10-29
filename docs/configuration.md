# Validated configuration and operational commands

USB protocol 8 separates production configuration/targets from debug memory
access. `FOC_Configuration.c` owns validation and application. USB and CAN target
and control-mode commands call the same functions from the serialized FOC loop.
No control ISR observes a partially committed configuration group.

## Catalog and absolute bounds

`FOC_CONFIGURATION_GROUPS` and `FOC_CONFIGURATION_FIELDS` in
`Foc/FOC_Core/FOC_Configuration.h` define the catalog. Array order supplies IDs;
member keys such as `limits.max_dq_current` are the persistent identity. IDs are
valid only for the discovered fingerprint. Groups currently cover Limits,
Motor, Controller, and Encoder calibration voltage. Communication,
encoder source/compensation, PID gains, calibration and storage retain their
dedicated commands.

Absolute min/max bounds live in `Foc/FOC_Core/FOC_Config.h` as `CONFIG_ABS_*`.
Electrical/temperature ceilings reference the board constants in the same file.
Numerical R/L, torque-constant, PID, speed and position bounds describe supported
software ranges, not proof of a motor's rating or closed-loop stability.
Changing a bound requires a firmware rebuild. Configuration bounds are included
in discovered metadata and its fingerprint, so hosts refresh cached ranges.
Booleans remain strictly 0/1 and native storage width cannot be expanded by
widening a configured bound.

Definition kind 4 uses these ten tab-separated columns, followed by LF:

```text
id  group_id  group_name  type  minimum  maximum  unit  writable_states  key  label
```

Numeric/boolean types are `f`, `u32`, `i32`, `bool`; all values occupy four wire
bytes. Bounds are finite decimal/scientific text (nine significant digits of
the native float). Labels/group names/units may contain spaces but no tabs or
control characters. `writable_states` is a u32 decimal bitmask with bit `state`
indicating permission; all current fields are writable. Limits permit IDLE and
ERROR; other groups require IDLE. Reads do not require IDLE. Full metadata rows,
including bounds and state permissions, participate in the canonical FNV-1a hash
under the `configuration\n` heading. Changes to current values do not.

## Group reads and atomic writes

All integers and f32 values are little endian.

| USB ID | Operation | Payload |
| --- | --- | --- |
| 0x36 | GET_CONFIGURATION | group:u8 |
| 0x37 | CONFIGURATION_REPLY | group:u8, count:u8, repeated(field:u8, value:4 bytes) |
| 0x38 | SET_CONFIGURATION | group:u8, count:u8, repeated(field:u8, value:4 bytes) |

GET returns every field in the group, ordered by catalog ID. SET may supply a
nonempty subset, without duplicate IDs or fields belonging to another group.
The complete candidate group is validated before any field is committed. Unknown
IDs, malformed lengths, non-finite/out-of-range values, invalid native integer
values, disallowed states, motor movement, unfinished table uploads and failed
cross-field checks reject the whole transaction. Successful SET returns ACK;
rejection returns the existing structured command error with code and text.
The firmware bounds transactions to the catalog (maximum 48 fields); the actual
USB payload must also fit the maximum reported in device info. Large atomic
updates cannot be split into independent writes without losing atomicity.

Cross-field checks require undervoltage < overvoltage and the voltage limit <=
undervoltage / sqrt(3), preserving the modulation envelope throughout the permitted
bus-voltage range. Encoder calibration voltage
not exceeding the configured d/q voltage limit, and valid positive R/L before
enabling current feed-forward. At first boot, unchanged zero/unmeasured R/L may
remain while other motor settings are configured; explicit R/L writes must be
positive. Current-control bandwidth does not automatically recalculate PI gains;
the current-PI tuning action performs that calculation.

Changed configuration invalidates/disables anti-cogging and resets PID memory.
Changing pole pairs or direction additionally invalidates encoder alignment and
correction and resets encoder history. Rejected writes have no such side effects.
Limits changed in ERROR do not clear faults or resume motor operation.
Configuration writes affect RAM only; flash save is a separate operation.

## Functional targets and gains

`MSG_SET_TARGET` (0x39) carries `operation:u8, value:f32`. Operations are stable:
0 d-current (A), 1 q-current (A), 2 mechanical speed (rad/s), 3 unwrapped mechanical
position (rad), 4 torque (Nm). These are operation codes, not discovered field IDs.
The command requires fault-free RUN, no table upload, a finite in-range value,
and a matching control mode. Current/torque targets require open-loop current
mode; speed and position require their corresponding modes. The d/q vector
magnitude cannot exceed the configured current limit. Torque converts using
`Iq = torque / torque_constant` with the constant in Nm/A. Rejection preserves
the previous target; there is no silent clamping.

`MSG_GET_TARGETS` (0x3A) is empty. `MSG_TARGETS_REPLY` (0x3B) contains four f32
values in order: d-current, q-current, speed, position. It requires no debug
variable definitions. Existing SET_PID validates all three gains together,
rejects NaN/infinity/negative/out-of-bound gains, checks fault/state/upload
conditions, resets controller memory, and invalidates anti-cogging. PID updates
require IDLE/RUN. Control-mode writes share validation on USB and CAN.

## Development versus production

`FOC_ENABLE_DEBUG_WRITES` defaults to 1 with `DEBUG` defined and 0 otherwise;
it can be explicitly overridden at compile time. Non-debug builds reject
MSG_SET_VAR and legacy text commands in firmware, independent of UI restrictions.
Debug reads and telemetry remain available. Development raw writes intentionally
bypass physical validation and must not be used as a production control API.
The current debug-variable catalog is empty: configuration fields and live
targets are not duplicated there. VAR_ID_LIST remains available for future
debug-only data. Target telemetry remains in the signal catalog.
Previously saved debug settings are not made trustworthy merely by their CRC;
production commissioning should start from known, reviewed settings.

Device info byte 33 reports whether debug writes are enabled. The plotter labels
Variables as unsafe debug access and disables writes when this capability is
false. Its Configuration panel is generated from discovery, retains rejected
local edits, shows command busy state, and reads back a successful update.

## Extending the catalog

Add storage if necessary, a group entry for a new group, and a scalar descriptor.
Bounds belong in FOC_Config.h. Add any required group validator/application side
effects to FOC_ConfigurationSet, and regression tests. The descriptor automatically
participates in discovery, fingerprinting and the generated plotter form. Schema
changes/new primitive kinds require a protocol extension; adding fields/groups
within this schema does not. Specialized operations/visualizers remain dedicated
commands/UI, not raw configuration offsets.
