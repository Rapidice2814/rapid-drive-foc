# Experimental high-frequency injection (HFI)

HFI estimates the rotor's saliency axis from a rotating alpha/beta voltage
carrier and the measured phase currents. It operates in RUN at low speed.
Encoder feedback remains the default; enabling injection alone does not select
HFI for control. Encoder readings and HFI estimates are separate telemetry.

This implementation requires an aligned, healthy encoder. Saliency alone has
a pi-electrical polarity ambiguity and cannot identify an absolute mechanical
sector. The initial estimate chooses the nearest saliency branch using the
encoder and seeds the mechanical origin once. Subsequent angles/speed come from
the HFI observer, not continuous encoder correction. The encoder remains an
independent validity check. This is not encoder-free startup or a high-speed
sensorless drive.

## Estimator and control

Each current-loop sample demodulates the negative-sequence current at the
previous held carrier's midpoint phase. Four 40 Hz low-pass filters extract
negative/positive sequence components. The negative sequence gives the double
rotor angle; a pi-periodic PLL tracks electrical angle and speed. Incremental
electrical motion divided by pole count produces unwrapped mechanical position.
Motor direction is applied consistently with the encoder feedback convention.

Carrier voltage is added after inverse Park conversion. The base voltage vector
is reduced to reserve carrier headroom within both the configured voltage limit
and bus/sqrt(3). While injection is active, the current regulator uses 200 Hz
filtered d/q currents rather than reacting directly to the HF carrier. This
changes current-loop dynamics; the gains and injection settings require motor-
specific review. Normal encoder-only operation retains unfiltered PID feedback.

`FOC_Config.h` contains demodulation/PLL constants, signal thresholds, saliency
polarity, measured ADC/PWM phase correction, and guards. Defaults assume Ld<Lq.
`HFI_SALIENCY_POLARITY` can be -1 for the opposite saliency. The default injection
is 0.2 V at 1000 Hz; the validated frequency range is 500–2000 Hz at an 8 kHz
sample rate. Frequency settings use rad/s, not Hz. No settings guarantee a usable
signal on a motor without measurable saliency. Resistance, saturation, inverter
dead time, sample delay, load and current regulation can bias the estimate.

Validity requires settling (0.2 s), sufficient saliency amplitude/ratio,
electrical speed <=50 rad/s, bounded PLL/reference errors and 0.1 s consecutive
good samples. The default reference guard is 0.35 electrical rad, including
unwrapped mechanical cycle-slip detection. Estimates outside this guard remain
visible for comparison but cannot be selected. Angles before SEEDED are not
measurements; SEEDED without VALID is not trustworthy control feedback.

## Comparison and selection

1. Start with a safe unloaded motor, reviewed current/voltage limits, correct
   motor direction/pole count and a healthy aligned encoder. Keep encoder feedback.
2. In stationary IDLE, set a small reviewed amplitude/frequency and enable HFI
   in Configuration → HFI. Enter zero-current open-loop RUN. RUN energizes the
   motor; the shaft must be clear. Injection is inactive outside RUN.
3. Plot `encoder_angle_electrical`, `hfi.angle_electrical`,
   `hfi.error_electrical`, `hfi.signal_amplitude`, `hfi.saliency_ratio` and
   `hfi.status`. For mechanical comparison use the two unwrapped mechanical
   angles and `hfi.error_mechanical`. Errors are radians; electrical error wraps
   to ±pi. Up to eight signals can be captured together. Slow manual movement
   can expose angular bias, but injection is still energized.
4. Only after comparison, the plotter's HFI tab can request HFI feedback with a
   confirmation. Switching either way requires fault-free stationary open-loop
   RUN (encoder speed <=0.1 rad/s, |d/q targets|<=0.001 A), no upload, and for HFI
   an active valid lock. Switching resets PIDs, disables anti-cogging feedforward,
   and initializes the position target to the selected position. Speed/position
   control modes then use selected feedback, as does electrical commutation.
   Entering position mode with HFI selected holds the current estimated position
   until another position target is sent, rather than commanding mechanical zero.

STOP/ERROR restores encoder feedback and clears HFI state. Selection is runtime
only, never persisted. STOP does not disable the saved injection-enabled setting:
another RUN can start comparison again. While HFI is selected, loss of validity
latches `FOC_ERROR_HFI_FAULT` (bit 16) and disables motor output. Excess phase-
current magnitude (>1.1 times the configured d/q limit) also faults during
comparison. There is no automatic live fallback or BEMF handoff.

## USB protocol 9

Existing device-info layout and fingerprint discovery are unchanged. New signal
rows change the fingerprint and automatically increase mask width as necessary.

| Command | Payload | Reply |
| --- | --- | --- |
| 0x3C MSG_SET_FEEDBACK_SOURCE | source:u8 (0 encoder, 1 HFI) | ACK or structured ERROR |
| 0x3D MSG_GET_HFI_INFO | empty | MSG_HFI_INFO_REPLY |
| 0x3E MSG_HFI_INFO_REPLY | source:u8, flags:u8, electrical_angle:f32, saliency_current:f32, electrical_error:f32, mechanical_speed:f32 | 18 bytes |

Floats are little-endian, angles/errors radians, saliency current amperes,
mechanical speed rad/s. Flags: enabled=1, active=2, seeded=4, locked=8, valid=16,
HFI selected=32. The same bits appear in the u32 `hfi.status` signal.

## Verification limits

Native compiled-C tests cover analytical carrier-current responses, both motor
directions, wraparound, tracking, no/wrong saliency, independent handles,
selection guards, overcurrent and STOP/fault cleanup. The plotter simulator's
HFI values are synthetic UI fixtures, not an estimator or accuracy benchmark.
Actual motor accuracy, ADC/PWM phase, stability and the 125 us sample-loop budget
require hardware measurement; software tests do not establish these properties.

Saliency/polarity background: [TI startup and initial angle detection](https://software-dl.ti.com/mcu-sdk/esd/AM13E2X-SDK/26_01_00_03/exports/docs/docs/am13e230x/user_guide/chapters/solutions/am13e230x/motor_control/control_start_sequence.html).
