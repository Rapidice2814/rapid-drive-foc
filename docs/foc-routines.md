# FOC calibration routines

Encoder alignment, current-sensor calibration, motor identification, PID tuning,
alignment testing, encoder linearization, and anti-cogging measurement use explicit per-handle
workspaces and a Start/Update/Finish lifecycle. Open-loop phase, the RUN speed-loop
divider, and checklist progress also belong to the FOC handle.

Encoder linearization and flash/source requirements are described in
[Encoder calibration](encoder-calibration.md).

## Ownership and lifecycle

`FOC_HandleTypeDef.alignment`, `current_calibration`, `identification`,
`pid_autotune`, `alignment_test`, and `anticogging` contain each routine's
named stage, timers, counters, candidate results, and retained status. These
workspaces are RAM-only and are not part of the flash format or USB definitions.
Persistent function-local variables and dynamic allocation are not used by the
routines.

- `Start` initializes the entire workspace. Starting an active routine first
  cancels it. A current-sense cleanup failure prevents that restart.
- `Update` performs one bounded stage. After completion, error, or cancellation,
  it returns the retained status without repeating hardware operations. An
  update before the first start returns ERROR / NOT_STARTED.
- `Finish` releases owned hardware state once and retains a terminal status.
  It accepts COMPLETED, ERROR, or CANCELLED. Cancellation does not commit
  candidate measurements and does not resume motor movement.

The state controller invokes Start on accepted entry and Finish on accepted
exit, including STOP and ERROR. Rejected transitions neither cancel the active
routine nor replace its continuation. Repeating the current state does not
restart its routine. Leaving and re-entering explicitly starts it again.
Successful calibration follows its requested continuation, or enters STOP when
none exists. A denied continuation also falls back to STOP rather than repeating
the completed routine.

Alignment removes its phase voltage on every finish path. Current calibration
leaves CSA calibration mode on completion, cancellation, timeout, invalid data,
or driver failure. A failed CSA cleanup forces high impedance and ERROR with
`FOC_ERROR_OPERATION_FAILED`, overriding an otherwise accepted destination.
Cleanup never enables the gate driver.

Anti-cogging saves position PID gains before applying its temporary tuning and
restores them on success, cancellation, or failure. Cleanup resets position/speed
PID memory, selects open-loop control, and clears motor targets and phase voltage;
it does not restore previous motion. The measurement table is updated in place
to avoid allocating another 8000-byte LUT. Its validity flag is cleared on Start
and remains clear after an interrupted or failed run. Only successful completion
marks the full two-direction table valid.

Identification and alignment testing remove excitation voltage on every finish
path. Identification retains accepted motor R/L on failure or cancellation;
alignment testing retains accepted motor direction. PID tuning leaves accepted
gains unchanged until all input/result checks pass.

Open-loop control retains its sample-update interface and has an explicit Reset
hook. Its phase resets on state entry and exit. RUN's divider resets on entry;
neither is shared across handles. Checklist progress survives only the requested
return from its calibration children; unrelated state changes cancel it. A
rejected transition does not advance the checklist.

The motor-setup state-entry validators are side-effect-free. Other state
transition callbacks still combine validation and entry actions; their behavior
has not been globally refactored. Fault entry disables output before cleanup.

## Timing and results

Updates remain in the ADC-synchronous FOC execution path. Current calibration
collects at most one sample per update, preserving the original 100-sample
collection rate after CSA settling. Alignment preserves the ten alternating
phase steps, zero-phase hold, encoder measurement, and voltage-release delays.

Elapsed milliseconds use unsigned subtraction, so settling and watchdog timers
work across HAL tick wraparound. Alignment has a 5000 ms watchdog; current
calibration has a 1000 ms watchdog. PID tuning has a 1000 ms watchdog, alignment
testing 30000 ms, and identification 60000 ms with at most 4096 fitting iterations
per phase per attempt. Alignment testing and identification allow the original
initial attempt plus three retries. Anti-cogging has a 600000 ms watchdog multiplied
by the configured revolution count and a 10000 ms settling limit per point.
Terminal failures report the timeout kind and a pre-cleanup control snapshot in
the plotter terminal. These constants are defined in the respective
routine source files; they are operation limits, not blocking waits or changes
to the sample-loop frequency.

Identification preserves the phase excitation sequence, 25 ADC-synchronous
samples, incremental model evaluation, gradient perturbations/scales, and spread
threshold. Invalid/nonpositive fitting results fail explicitly instead of
continuing with invalid model parameters. Anti-cogging preserves its current
filter, speed-loop divider, target progression, and table ordering. Alignment
test retries reset sweep counters, reference angles, and accumulated errors.

Current offsets are committed only after all samples, finite-result checks,
CSA cleanup, and final settling succeed. The ADC-calibrated flag is cleared on
start and set only on success. Previously installed offsets remain in place
after an unsuccessful attempt, but are not marked calibrated.

When a requested operation is rejected specifically because the current sensor
is uncalibrated, the state controller can run current calibration first. This
automatic prerequisite is accepted only from stationary, fault-free IDLE with
a finite positive bus voltage and valid configuration for the requested operation.
Outputs remain in high impedance during calibration. On success the controller
returns through STOP/IDLE, rechecks the original operation's entry guards, and
tries it once, preserving its requested continuation. Calibration failure, a
rejected retry, or movement before retry stops the operation and latches a fault.
An accepted state change away from current calibration, including an explicit
STOP or ERROR, discards the pending request. There is no automatic resume after
fault acknowledgement. The checklist retains its existing explicit calibration
step.

The automatic prerequisite leaves the setup-result sequence unchanged; the
requested routine increments it when it actually starts. This lets the plotter
wait for the original step without mistaking internal current calibration for
an unrelated setup operation. Explicit current-calibration requests still
produce their own setup sequence.

Alignment stages the encoder-zero helper's result, restoring the
accepted offset and validity immediately after the helper returns. Its candidate
is committed only on successful completion. Updates and state transitions share
the existing single FOC execution context; these APIs are not interrupt-safe or
concurrently reentrant for the same handle.

The retained context exposes `runtime.status`, `runtime.error`, and
`failed_stage`. The state wrappers retain the shared operation-failed latch.
These diagnostic fields are not automatically exposed in the plotter.

## Adding a routine

1. Define a named stage enum and context in a hardware-independent header.
2. Add workspace storage owned by the FOC handle, not flash configuration.
3. Implement Start, bounded Update, and idempotent Finish. Use explicit stage
   assignments; counters represent repeated measurements, not hidden stages.
4. Validate inputs, bound waits/iterations, and reject unknown stages.
5. Stage results until validation and cleanup succeed. Define cancellation
   behavior for every temporary hardware or controller change.
6. Register start/finish hooks in the state controller and handle the returned
   loop status in the state wrapper.
7. Test success, interruption, restart, hardware failure, invalid stages, and
   clock wraparound. Measure worst-case execution time on the target.

The contexts are separate structures rather than a union. Additional routines
must account for the MCU's RAM budget before adding large measurement arrays.

## Executable regression tests

`tests/routines/run_tests.py` compiles the actual routine sources, state
controller, calibration/storage/startup/RUN/STOP state handlers, and fault polling for
Cortex-M3 software floating point, then executes them under
Unicorn. Peripheral calls are mocked. This tests C state-machine behavior, not
motor dynamics, peripheral timing, CORDIC accuracy, or physical safety.
Maintenance tests cover entry from IDLE/ERROR, rejection from RUN, fault polling
between acceptance and execution, preservation of fault masks, and flash failures.
Bootloader tests check acceptance and fault polling, not the ROM jump itself.
Routine tests cover all finite-routine cancellation stages, retained results,
partial-work restart, anti-cogging PID restoration and LUT validity, an analytic
RL response with the original fitter's final gradient update, candidate direction
commit, wraparound timing, independent handles, and checklist continuation.
Current/speed control and peripheral calls are mocked; the real RUN divider and
open-loop phase-update code are executed.

```powershell
python -m venv build/.venv-routine-tests
./build/.venv-routine-tests/Scripts/python.exe -m pip install -r tests/routines/requirements.txt
./build/.venv-routine-tests/Scripts/python.exe tests/routines/run_tests.py --compiler C:/ST/STM32CubeCLT_1.21.0/GNU-tools-for-STM32/bin/arm-none-eabi-gcc.exe
```

`--compiler` may be omitted when `arm-none-eabi-gcc` is on PATH. Test stubs are
isolated under `tests/routines/stubs` and are not part of firmware builds.
