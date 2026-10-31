# Motor Setup workflow

USB protocol 12 adds a plotter Setup tab, a combined encoder-direction/pole-pair
detection routine, and correlated routine-result polling. Existing state and
command IDs are preserved; the new state is appended.

## Operator workflow

Open the Setup tab beside Calibration. The settings at the beginning duplicate the normal
configuration pages: physical encoder selection, motor/driver current and voltage
limits, bus/temperature trips, current-control bandwidth, encoder calibration D
current and optional anti-cogging PID gains. Read/apply edits before starting.
Detection/alignment excitation voltage is a separate runtime-only setting
(default 0.5 V); it must be positive and within the configured voltage limit.
The motor must be unloaded. ADC current-sensor calibration must have completed.

The selected steps run in this order:

1. Detect encoder direction and pole pairs (one combined test).
2. Align encoder.
3. Measure L/R.
4. Tune d/q current PI controllers.
5. Calibrate encoder.
6. Calibrate anti-cogging.

All boxes default checked except anti-cogging. Each row also has a standalone
Run step button. Connection changes reset the defaults and stop the workflow;
no run resumes automatically. Existing pages remain available. The top Run
selected steps button confirms motor motion once for the chosen sequence.
Stop cancels immediately. Clear latched faults never resumes a sequence.

Only a fresh driver terminal result for the expected step permits progression.
An ACK is acceptance, not completion. A matching successful result must also
report fault-free IDLE before the next request. Failed/cancelled results, nonzero
routine error codes, unexpected operation/sequence IDs, rejection, disconnect
or a status-read failure stop the sequence. Skipped prerequisites are not silently
run; the driver may reject a later step, ending the sequence with its reason.

Encoder and anti-cogging tables remain disabled until explicitly enabled. If
anti-cogging is selected directly after encoder calibration, it measures in the
currently active encoder coordinates. Enabling encoder correction afterward
invalidates anti-cogging; enable correction explicitly and rerun anti-cogging if
a corrected-coordinate table is desired. Encoder calibration saves current
settings to flash. Anti-cogging still needs explicit Save settings afterward.
Calibration progress and the existing plotter-only ETA are duplicated in Setup.

## Combined direction and pole detection

FOC_STATE_MOTOR_PROBE (21) does not require a prior pole count, encoder offset,
R/L estimate or tuned current PI gains. It uses a low-voltage stator field:
200 ms excitation ramp, 500 ms initial lock, eight electrical turns forward at
one electrical turn/s, 500 ms hold, eight turns backward and a final 500 ms hold.
Raw physical encoder increments are unwrapped independently of correction/zero.
Internal simulator sources cannot measure their own direction/pole count.

For each sweep, pole pairs = (8 * 2*pi) / abs(mechanical travel). The sign of
forward travel gives encoder direction. Forward and reverse must have opposite
signs and at least 0.1 rad travel each. Both estimates must be within 0.2 of the
same integer from 1 through 255, and agree within 0.25 pole pairs. These are
validation heuristics; poor field following or encoder nonlinearity may prevent
reliable detection. Uncertain measurements fail rather than silently guessing.

Only success commits direction/pole pairs. It invalidates encoder alignment,
encoder correction and anti-cogging, and resets encoder history. Failure and
cancellation preserve prior parameters. Outputs are removed on every terminal
path; ordinary fault checks, including phase overcurrent, remain active. The
routine watchdog is 25 s. Actual motor behaviour requires target validation.

## Wire additions

- GET_SETUP_STATUS (0x3E): empty payload.
- SETUP_STATUS_REPLY (0x3F): 20 bytes, little-endian:
  sequence:u32, routine_state:u8, status:u8, routine_error:u8, driver_state:u8,
  active_faults:u32, latched_faults:u32, setup_voltage:f32.
- SET_SETUP_VOLTAGE (0x40): voltage:f32; requires fault-free IDLE and voltage
  within the configured limit; ACK/error response. The value is not persisted.

Sequence increments on each accepted routine entry, including initialization
failures. Status values match FOC_LoopStatusTypeDef: 0 running, 1 completed,
2 error, 3 cancelled. Text failure details remain available through the retained
routine-result command. Detection/commissioning simulation checks workflow and
readback only; it does not validate physical motor behaviour.
