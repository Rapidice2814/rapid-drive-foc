# Calibration control and table transfers

USB protocol 6 adds calibration progress, multi-revolution control and editable
table transfers. Flash format remains 2. All integers/floats use little-endian
encoding; floats are IEEE-754 binary32. Existing protocol-5 commands retain IDs.
`kind` is 0 for encoder linearization and 1 for anti-cogging.

## Control and status

| Command | Payload | Reply |
| --- | --- | --- |
| GET_CALIBRATION_INFO (0x2C) | kind:u8 | CALIBRATION_INFO_REPLY (0x2D) |
| START_CALIBRATION (0x2E) | kind:u8, revolutions:u8 | ACK/error |
| ENABLE_CALIBRATION (0x2F) | kind:u8, enabled:u8 | ACK/error |

Status is `kind:u8, flags:u8, stage:u8, error:u8, progress:u16,
revolutions:u8, bins:u16, channels:u8` (10 bytes). Flags: bit 0 valid,
bit 1 enabled, bit 2 measuring, bit 3 any table upload active. Progress is 0-1000
(0-100%). Error 0 means no error; codes 1-7 match encoder routine errors.
Stages 0-8 are idle, starting/locking, positioning/applying, measuring,
reporting/validating, finishing/saving, complete, failed, cancelled.

Revolutions are 1-8 **per direction**, default three. Settings are runtime-only.
START requires stationary, healthy, aligned, fault-free IDLE and calibrated ADCs.
Encoder calibration additionally requires a calibratable source and known motor
direction/pole count. STOP cancels; success stops rather than starting RUN.
Encoder calibration saves all settings to flash; anti-cogging results remain in
RAM until FLASH_SAVE. Enable/disable requires stationary IDLE/ERROR; enabling
also requires valid data. Changing encoder coordinates invalidates anti-cogging.

Anti-cogging settles at 1000 positions per mechanical revolution in each direction
and averages repeated settled-current measurements into separate forward/reverse
tables. A point watchdog is 10 s; the overall watchdog is 600 s multiplied by the
configured revolution count. Cancellation/failure leaves its in-place table
invalid. PID gains are restored and outputs disabled on termination. Calibration
must run unloaded; this estimates compensation current, not a direct torque or
independently measured position-error trace.

Terminal failures send a detailed `MSG_TEXT_REPLY` to the plotter terminal after
motor excitation is removed. Timeout code 4 distinguishes point settling timeout
from overall calibration timeout in the text. Diagnostics include failed stage,
direction, revolution, table bin, progress, elapsed/limit times, target and
wrapped/unwrapped positions, wrapped settling error, unwrapped PID error, speed,
filtered q-axis current, pre-cleanup current command, hold count and fault masks.
Positions are in milliradians, errors in microradians, speed in microradians/s,
and currents in milliamps; nonfinite/out-of-range measurements display `invalid`.
Settling still requires absolute wrapped error below 0.005 rad and absolute speed
below 0.001 rad/s for 11 consecutive checks. Calibration status packets retain
the existing error codes and layout; detailed text does not require a protocol change.

## Reading and editing tables

Transfers require stationary IDLE/ERROR, with no calibration routine running.
Values are flattened: encoder has 128 correction radians indexed by raw absolute
sensor angle; anti-cogging has 1000 mechanical bins with interleaved forward and
reverse compensation amperes (`fwd0, rev0, fwd1, rev1, ...`). Bin angle is
`index * 360 / bins` degrees; anti-cogging is indexed in aligned/corrected
mechanical coordinates. Sizes/channels come from status, not host assumptions.

| Command | Payload | Reply |
| --- | --- | --- |
| GET_CALIBRATION_TABLE (0x30) | kind:u8, offset:u16, count:u8 | CALIBRATION_TABLE_REPLY (0x31) |
| BEGIN_CALIBRATION_UPLOAD (0x32) | kind:u8, total_values:u16 | ACK/error |
| WRITE_CALIBRATION_TABLE (0x33) | kind:u8, offset:u16, count:u8, values:f32[count] | ACK/error |
| COMMIT_CALIBRATION_UPLOAD (0x34) | kind:u8, crc32:u32 | ACK/error |
| ABORT_CALIBRATION_UPLOAD (0x35) | empty | ACK/error |

Read reply: `kind:u8, offset:u16, total_values:u16, count:u8, values:f32[count]`.
Counts are 1-12 floats, offsets count floats rather than bins or bytes. A chunk
never exceeds the advertised command payload limit. A read is rejected while an
upload is active. Reading invalid tables is allowed for inspection; status flags
determine whether values are usable. Reads return accepted tables, not live
measurement buffers or per-revolution residuals.

Uploads require BEGIN, sequential acknowledged WRITE chunks starting at offset 0,
and COMMIT. CRC-32/ISO-HDLC covers all little-endian float bytes in transfer order
(Python `zlib.crc32`). Missing, overlapping, reordered, nonfinite or out-of-range
values are rejected. The source must remain unchanged. Encoder magnitudes must
not exceed 0.1 rad and the periodic corrected coordinate must remain monotonic,
including wraparound. Anti-cogging values must not exceed `max_dq_current`.

Encoder uploads stage in the idle calibration workspace; unsuccessful uploads do
not replace the accepted encoder table. Anti-cogging uploads invalidate/disable
the table at BEGIN and write in place because RAM cannot hold a second table.
Failed or aborted anti-cogging uploads therefore require a complete re-upload or
flash load to restore a valid table. A failed COMMIT leaves the upload open until
ABORT. An abandoned upload blocks motion and flash operations; ABORT is available
after reconnect. STOP and ERROR remain permitted during an upload.

Successful commits mark data valid but leave compensation **disabled**. Applying
an encoder table preserves raw zero, rebases the disabled coordinate, resets speed
history and invalidates anti-cogging. ENABLE explicitly activates the new table;
FLASH_SAVE separately persists all current settings/tables. The transfer does
not automatically erase or program flash. Flash writes can interrupt telemetry.

## Plotter interface

The right-side Calibration tab contains encoder source controls and separate
Encoder/Anti-cogging pages. Each page shows percentage/stage, validity, enable
setting, selectable revolution count, plots and numeric tables. Read table loads
a complete snapshot. Numeric value cells are editable. Selecting a plotted point
or table cell selects a value; dragging the yellow horizontal line edits it.
Plots show correction/compensation versus angle, with RMS and peak magnitudes;
they do not verify achieved physical accuracy after compensation.

Edits stay local until Apply edited table to RAM is confirmed. Uploads are
acknowledged one chunk at a time and stop at the first rejection. Reconnect clears
local snapshots so another device cannot receive stale edits inadvertently.
Save all settings and tables to flash is a separate explicit operation.
