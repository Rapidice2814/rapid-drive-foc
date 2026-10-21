# Encoder sources and linearization

## Compatibility and storage

USB protocol 6 and flash format 2 are required. Older settings records are not
migrated: header/version/length/CRC validation rejects them and startup uses
defaults. Reading does not erase old records; flash save or successful encoder
calibration replaces them. Existing settings must be backed up before upgrading.

The record stores source selection, raw/corrected zero, 128 correction floats,
validity/enabled flags, and calibration voltage. CRC-32/ISO-HDLC covers all bytes
before the CRC field; magic, format version, record length, and terminator are
also checked. Invalid source bindings/tables are rejected on load. Storage is
single-bank: interrupted programming can destroy the previous flash record.
There is no atomic dual-record power-loss recovery. A reported calibration-save
failure restores the previous table and zero in RAM.

## Source interface

`FOC_ENCODER_SOURCE_LIST` in `Foc/FOC_Drivers/FOC_Encoder.h` defines stable source
IDs, printable ASCII names (1-31 bytes), initialization callbacks, read callbacks,
and calibration capability. IDs/names must be unique; at most eight sources are
supported. IDs are persisted and must not change meaning across releases.

Production source 0 is `AS5047P timer`. Its backend initializes the sensor and
seeds the quadrature counter from the absolute SPI angle. Normal sampling retains
the 4096-count timer path; no per-sample SPI transactions are introduced. Only the
selected source is initialized. Failed sensor initialization leaves USB available
in ERROR; applying that source again retries setup. Flash load initializes the
stored source too.

Readers return a healthy absolute mechanical angle in radians, independent of
stored zero/correction. Initialization/read callbacks must not enable motor
outputs. Read callbacks must fit the sample-loop budget. Timer readings alone
cannot detect every physical sensor failure.

A backend needs initialization/read functions, a registry row, and its source
file added to CMake, for example:

```c
X(7, "Other absolute encoder", OtherEncoder_Init, OtherEncoder_Read, 1)
```

The plotter discovers names/IDs/capabilities from the driver. No Python source
list needs changing. Tests build a second mock backend through this same registry.
The physical encoder backend is separate from the experimental [HFI feedback](hfi.md)
selector. HFI seeds electrical polarity and mechanical origin from this aligned
encoder, which remains sampled independently as a safety reference.

Selection requires stationary IDLE/ERROR (absolute speed <= 0.1 rad/s). Changing
sources invalidates alignment, correction, and anti-cogging, resets estimator
history, and initializes the backend. A failed initialization leaves the chosen
source selected but unavailable rather than silently falling back. Selection
does not clear faults and is persisted by normal flash save.

## Measurement

Calibration requires fault-free stationary IDLE, calibrated current sensing,
valid alignment, a healthy calibratable source, and correct motor direction and
pole count. The shaft must be unloaded and free to rotate. The alignment test
can check direction first. This is motor-referenced compensation, not precision
calibration against an independent angular reference; load, cogging and friction
can contaminate measurements even with bidirectional averaging.

The Start/Update/Finish routine commands a stator field independently of the
encoder position PID. It locks for 300 ms, then steps through 1-8 mechanical
revolutions in each direction (default three), using 1024 positions per revolution. Each position
settles for 10 ms before eight ADC-synchronous samples are accumulated. Signed
unwrapping tracks complete turns. HFI and current/anti-cogging feedforward are
temporarily disabled; the old correction is bypassed, not overwritten.

Default excitation is 0.5 V (`flash_data.encoder.calibration_voltage`). Voltage
must be positive and within configured voltage limits. Invalid current limits,
phase current above `max_dq_current`, unhealthy/nonfinite readings, source/motor
parameter changes, loss of tracking, and a watchdog of 120 s per configured
revolution per direction cause failure. Tracking
error must stay below min(0.15 rad, pi/(2*pole_pairs)).

Errors are binned by raw absolute angle into 128 entries. Every bin must have
forward and reverse coverage. Directional disagreement is limited to 0.03 rad.
The combined table is mean-centered; entries must be finite and within 0.1 rad.
Corrected adjacent-angle increments must lie between 0.1 and 2 times the bin
interval, including the wrap boundary. Runtime lookup uses periodic linear
interpolation before zero subtraction, mechanical unwrapping, and speed/electrical
angle calculation. Zero alignment uses the same corrected coordinate.

## Completion, cancellation and controls

After both sweeps, outputs enter high impedance before validation/persistence.
Only a validated candidate replaces the table and matching zero. Success saves
all current settings, enables correction, invalidates anti-cogging, and enters
STOP, never RUN. Anti-cogging must subsequently be remeasured. Cancellation or
measurement failure preserves the previous table/zero; failure latches
OPERATION_FAILED. Cleanup restores temporary flags and removes excitation.

Flash programming still blocks interrupts and can pause telemetry during the
final save. No flash writes occur during either measurement sweep. Actual motor
behavior, calibration limits and 8 kHz timing require target validation.

The Calibration tab in the plotter exposes discovered sources, correction enable,
progress/status, editable plots/tables and revolution counts. Calibration requires an
unloaded-shaft confirmation, including through the generic state selector. STOP
cancels measurement. Correction toggles require stationary IDLE/ERROR, rebase zero
from its raw coordinate, invalidate anti-cogging, and reset position/speed history.
Calibration is never automatically started at boot.

Table download, validated RAM upload, anti-cogging controls and separate flash
persistence are described in [Calibration table transfers](calibration-tables.md).

The simulator exercises permissions, progress, cancellation and simulated saves,
not encoder physics. Executable C tests use the real calibration/provider code
with injected periodic angle error, including interpolation/wraparound, source
initialization/recovery, cancellation, CRC validation, and RAM rollback.
