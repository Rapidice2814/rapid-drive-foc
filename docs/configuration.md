# Validated configuration and operational commands

The USB protocol separates production configuration/targets from debug memory
access. `FOC_Configuration.c` owns validation and application. USB and CAN target
and control-mode commands call the same functions from the serialized FOC loop.
No control ISR observes a partially committed configuration group.

## Catalog and absolute bounds

`FOC_CONFIGURATION_GROUPS` and `FOC_CONFIGURATION_FIELDS` in
`Foc/FOC_Core/FOC_Configuration.h` define the catalog. Array order supplies IDs;
member keys such as `limits.max_dq_current` are the persistent identity. IDs are
valid only for the discovered fingerprint. Groups currently cover Limits,
Motor, Controller, and Encoder calibration D current. Communication,
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
bus-voltage range. Encoder calibration D current
not exceeding the configured d/q current limit, and valid positive R/L before
enabling current feed-forward. Zero is the unset sentinel for numeric motor/configuration values. Partial
groups can be configured incrementally, and explicit zero writes clear a setting.
Nonzero values must satisfy the advertised bounds. Cross-field checks involving
an unset limit are deferred until both related limits are set; output remains
blocked until the operating requirements are satisfied. Current-control bandwidth does not automatically recalculate PI gains;
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
MSG_SET_VAR and text commands in firmware, independent of UI restrictions.
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

## Generic driver commissioning and motor presets

New/cleared flash contains zero motor parameters, limits, bandwidth, calibration
current and PID gains. Board electrical capabilities, current-sense conversion,
NTC data and algorithm timing constants remain hardware/firmware constants.
Saved records are accepted only when their magic, format version, size and CRC
match the current format. Records from other formats are rejected, without
per-field migration; apply a preset and recalibrate after a format change.

An incomplete driver boots through STOP to IDLE with the inverter in high
impedance. USB/CAN configuration and storage continue to work. Motor-driving
state requests are rejected before entry side effects and identify the missing
setting in terminal output. Every operation requires all seven nonzero operating
limits, within absolute bounds and consistent with each other. Direction zero
is a valid direction, and disabled booleans/zero PID terms remain valid.

Identification/probing can measure unknown R/L/pole pairs. Other motion needs
pole pairs; RUN requires positive R/L and configured d/q PI gains. PI tuning
needs positive R/L and bandwidth. Encoder calibration also needs its calibration
current; anti-cogging needs its position gains. Torque constant is only required
for torque targets. Speed/position modes require the corresponding gains.
A preset itself may omit any field; required values must eventually come from
manual entry, another preset or measurement before the operation can start.

Voltage-driven commissioning is also constrained by the configured voltage limit.
R/L identification seeds its model from the measured current step instead of a
hard-coded motor model. Board-level voltage, temperature and current monitoring
remains available during commissioning when application limits are unset.

The plotter Configuration controls (also embedded in Motor Setup) offer a model
preset dropdown, Add preset, Save preset from driver, Load into editor, and Apply
to driver. Files have separate `parameters` and `pids` sections. Scalar configuration fields
(including limits) use configuration transactions. PID controllers use the
existing PID registry and GET_PID/SET_PID commands; gains are not duplicated in
the scalar configuration catalog. Every field has an
optional inclusion checkbox. A driver snapshot reads fresh configuration groups and PID controllers, not local
edits. A loaded preset changes only included fields and skips unsupported keys
with a visible message. Writes run sequentially with readback per group; failure
stops the sequence. Earlier successful groups remain in RAM. Save to driver flash
is explicit. Per-device encoder alignment/correction/anti-cogging tables, encoder
hardware selection, CAN node IDs/rates and live motion targets are not model
parameters and retain their dedicated controls.

Presets are versioned JSON files under `%LOCALAPPDATA%/RapidDrive/motor-presets`
on Windows (XDG data home, or `~/.local/share`, on other systems). Override with
`RAPID_DRIVE_PRESET_DIR`. Parameter keys, never discovered numeric IDs, identify
settings. For example, a partial preset is:

```json
{"version": 2, "name": "Example model", "parameters": {"motor.pole_pairs": 7}, "pids": {"pid_speed": {"kp": 0.1}}}
```

Each preset PID controller and each of its `kp`, `ki`, `kd` values is optional.
Before applying a PID override, the plotter reads that controller's current
three gains, merges the supplied subset, and sends one complete SET_PID. It
reads all three back and compares their f32 values. Omitted gains retain the
freshly read values; explicit zero sets a gain to zero. Rejection, mismatch or
disconnect stops subsequent writes. Whole presets are not atomic: earlier
successful configuration groups/controllers remain applied in RAM.

PID discovery rows contain `id`, `name`, `minimum`, `maximum`, and writable-state
mask. Shared bounds and states come from the same constants used by validated
PID writes and participate in the definitions fingerprint. The plotter PID tab
uses these bounds and permissions instead of its own tuning limits. Preset
schema 2 is the only supported file layout. The saved flash layout is unchanged.
