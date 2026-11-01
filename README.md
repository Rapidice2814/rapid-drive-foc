# Rapid Drive FOC

Rapid Drive FOC is C firmware for a three-phase motor controller based on the
STM32G431CBUx. It implements field-oriented current control, speed and position
loops, encoder feedback, calibration and motor identification, fault handling,
and configuration storage in internal flash. USB provides configuration and
high-rate telemetry; CAN provides control and cyclic status messages.

The separate [Rapid Drive Plotter](https://github.com/Rapidice2814/rapid-drive-plotter)
runs on the computer and operates with programmed controllers independently
of the firmware source tree and STM32 build tools.

## Documentation

- [USB protocol and command reference](docs/usb-protocol.md): framing, discovery,
  telemetry, configuration commands, and errors.
- [CAN protocol and command reference](docs/can-protocol.md): identifier layout,
  bus settings, payloads, cyclic replies, addressing, and implementation limits.
- [Calibration controls and editable tables](docs/calibration-tables.md):
  multi-revolution measurements, progress, compensation and USB table transfers.
- [Validated configuration](docs/configuration.md): discovered settings, absolute
  bounds, atomic group updates, functional targets and production/debug separation.
- [Plotter installation and use](https://github.com/Rapidice2814/rapid-drive-plotter#readme).

Current firmware version: **0.3.0**. USB protocol version: **14**. Firmware and
plotter must use the same current protocol. Signals, variables, PID controllers,
CAN cyclic entries and configuration metadata are discovered and cached by
fingerprint. Catalog additions do not require a plotter update. Absolute bounds
are editable as `CONFIG_ABS_*` in `Foc/FOC_Core/FOC_Config.h`.

[Encoder sources and calibration](docs/encoder-calibration.md) documents source
selection, linearization, backend integration and storage. Flash format 3 uses
CRC validation and accepts only the current layout. Other saved formats are
rejected and the driver starts unconfigured. Reapply a JSON motor preset and
recalibrate after upgrading from an older format. Successful encoder calibration
saves current settings and invalidates anti-cogging.

## Development tools

The examples use Windows PowerShell. The command-line build does not require
STM32CubeIDE or an editor extension.

| Tool | Purpose |
| --- | --- |
| Git | Repository cloning and revision management |
| [STM32CubeCLT](https://www.st.com/en/development-tools/stm32cubeclt.html) | Arm GNU compiler, CMake, Ninja, STM32CubeProgrammer, and ST-LINK tools |
| ST-LINK driver (Windows, supplied with the ST tools) | USB access to the programming/debug probe |
| [Python](https://www.python.org/downloads/) 3.10 or newer | Plotter and regression tests; not required for firmware builds |

The tested GCC toolchain consists of STM32CubeCLT 1.21.0, GNU Tools for STM32
GCC 14.3.1, CMake 3.28.1, and Ninja 1.11.1. The project requires CMake 3.22 or
newer. The IOC records CubeMX 6.17.0 and STM32CubeG4 firmware package V1.6.2.

Example PowerShell environment for STM32CubeCLT installed at
`C:\ST\STM32CubeCLT_1.21.0`:

```powershell
$cubeClt = 'C:\ST\STM32CubeCLT_1.21.0'
$env:PATH = "$cubeClt\GNU-tools-for-STM32\bin;$cubeClt\CMake\bin;$cubeClt\Ninja\bin;$cubeClt\STM32CubeProgrammer\bin;$env:PATH"

arm-none-eabi-gcc --version
cmake --version
ninja --version
Get-Command STM32_Programmer_CLI
```

The PATH assignment applies to the current terminal session. Builds on
Linux/macOS use the corresponding installed tool directories and shell syntax.

## Source checkout

```powershell
git clone https://github.com/Rapidice2814/OHboteam_FOC.git rapid-drive-foc
cd rapid-drive-foc
```

The Git remote currently uses the historical name OHboteam_FOC; the local folder
name above keeps it consistent with the plotter's optional sibling-source
checks. Cross-repository contract tests use a sibling plotter checkout.

## Firmware build

Debug build from the firmware repository root, with the tools on PATH:

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

Optimized GCC build:

```powershell
cmake --preset Release
cmake --build --preset Release
```

The build generates these files in build/Debug or build/Release:

| File | Use |
| --- | --- |
| rapid_drive_foc.elf | Flashing with embedded addresses, debugging, and symbols |
| rapid_drive_foc.bin | Raw application image; flash at 0x08000000 |
| rapid_drive_foc.map | Linker layout and memory usage |
| compile_commands.json | Compiler commands for tools such as clangd |

Debug and Release use [cmake/gcc-arm-none-eabi.cmake](cmake/gcc-arm-none-eabi.cmake).
Debug_Clang and Release_Clang presets also exist for ST Arm Clang, but are an
alternative setup. The commands above use GCC. Each toolchain uses a separate
build directory.

All C sources, including application code, STM32 HAL and USB middleware, use
`-Oz` and link-time optimization (LTO). This policy applies to both Debug and
Release. `-Oz` prioritizes flash size while retaining most `-O2` optimizations;
it is not a guarantee of the fastest execution for every routine. Function/data
sections and linker garbage collection remove unused code. Precompiled runtime
libraries retain their vendor build settings.

Debug retains `-g3` symbols, but optimized code may inline functions and remove
local variables, limiting source-level stepping. Release omits debug information.
Fast-math remains disabled because fault validation depends on NaN/Inf checks.
The GCC toolchain keeps configuration flags separate from the project-wide
optimization policy so CMake's default Release `-O3` cannot override `-Oz`.

The compiled-C regression suite supports the same optimization policy:

```powershell
python tests/routines/run_tests.py --optimized --compiler arm-none-eabi-gcc
python tests/routines/check_build_optimization.py build/Release
```

These checks require the routine-test Python dependencies (`pyelftools` and
`unicorn`). The build check verifies every C compile command, the reset vector,
application bounds and the retained NOLOAD configuration reservation. Execution
time and the 8 kHz sample deadline still require hardware measurement after a
compiler/optimization change.

Both linker scripts reserve 112 KiB for application flash and 16 KiB at
0x0801C000 for persistent configuration, with 32 KiB RAM. The .permanent
section is NOLOAD. A full chip erase removes saved configuration.

## Programming and startup

ST-LINK programming uses SWDIO, SWCLK, GND, and the target voltage-reference
pins. NRST supports hardware reset and connect-under-reset. Target power is
provided according to the board's power design.

From the repository root, with STM32_Programmer_CLI on PATH:

```powershell
STM32_Programmer_CLI -c port=SWD -w .\build\Debug\rapid_drive_foc.elf -v -rst
```

The ELF supplies its addresses; -v verifies the write and -rst resets the
controller. BIN programming includes the application start address:

```powershell
STM32_Programmer_CLI -c port=SWD -w .\build\Debug\rapid_drive_foc.bin 0x08000000 -v -rst
```

The [STM32CubeProgrammer GUI](https://www.st.com/en/development-tools/stm32cubeprog.html)
also supports ELF programming over ST-LINK/SWD. Additional interfaces and
connection options are documented in ST's
[programmer CLI reference](https://dev.st.com/stm32cube-docs/prog/2.23.0/en/docs/markup/CubeProg_Command_Lines.html).

The USB bootloader command is accepted from IDLE or ERROR. It jumps to the MCU
system bootloader and is separate from normal USB CDC telemetry. SWD is the
initial programming path described above; entering the system bootloader does
not itself upload a new firmware image.

## Plotter connection

Plotter installation and launch from the parent directory:

```powershell
git clone https://github.com/Rapidice2814/rapid-drive-plotter.git rapid-drive-plotter
cd rapid-drive-plotter
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install .
.\.venv\Scripts\rapid-drive-plotter.exe
```

The plotter connects to the controller's USB serial/COM port. Device info and
discovery populate the signal, variable, and PID controls automatically. The
current configuration supports up to eight selected signals at once. Firmware
reports the sample rate. The simulator operates without hardware.

Motor setpoint and PID editing controls require RUN in the plotter. Flash
save/load/clear and system-bootloader entry require IDLE or ERROR; the state machine
checks transitions, and entering IDLE requires STOP with the motor stationary.
Flash actions requested from IDLE can return to RUN after success. A denied
transition to RUN falls back to STOP; a failed flash operation enters ERROR.
Actions started from ERROR keep motor outputs disabled, preserve fault masks,
and return to ERROR while faults remain. If faults have been cleared, they
return to STOP rather than RUN. Fault polling does not preempt flash or
bootloader maintenance states.

ERROR is tied to latched fault bits. Failed calibration, identification,
tuning, alignment, anticogging, and storage actions latch OPERATION_FAILED
(bit 15, 0x00008000). Clearing latched faults immediately reevaluates active
conditions: present faults relatch, while fault-free recovery enters STOP
instead of RUN. STOP then enters IDLE once stationary.

HDF5 recordings and cached definitions are stored in user directories, not in
the firmware repository. Their locations and environment-variable overrides
are documented in the plotter README.

## Project layout and configuration

| Path | Purpose |
| --- | --- |
| Foc/FOC_Core | Main handle, configuration, diagnostics, and USB discovery/telemetry |
| Foc/FOC_Drivers | ADC, CAN, flash, USB transport, and supporting driver logic |
| Foc/FOC_Loops | Current/speed/position control, identification, calibration and operational loops |
| Foc/FOC_States | Startup, operation, stop/idle/fault, calibration, and storage state handling |
| Foc/Hardware_Drivers | DRV8323, AS5047P, and WS2812B interfaces |
| Foc/Utils and Foc/Sequencers | Numeric helpers, PID/filter code, timing, sounds, and lights |
| Core, USB_Device | CubeMX-generated application/peripheral setup and USB CDC glue |
| Drivers, Middlewares | Checked-in ST HAL/CMSIS and USB device stack |
| cmake, CMakePresets.json | Compiler configuration and build presets |
| rapid_drive_foc.ioc | CubeMX board/pin/clock configuration |

Board and motor configuration is defined in FOC_Config.h, the defaults in
FOC_Flash.c, and the peripheral/pin setup in Core. A valid flash configuration
overrides defaults at startup. Changing a C default alone does not replace
previously saved settings.

The number-free signal, variable, and PID X-macro lists are defined in
Foc/FOC_Core/FOC_USB_Debug.c. List order assigns IDs and signal bits. Native
storage access, discovery text, and the fingerprint all derive from those
lists. Mask width is (SIGNAL_COUNT + 7) / 8; compile-time checks enforce field
types and capacities, and duplicate names are rejected within each list.

Changing names, types, or list order changes the fingerprint automatically.
Changing the wire format or primitive encoding requires a USB protocol-version
bump in both repositories. The simulator uses a separate text fixture; the
contract test checks it against the firmware's exposed lists.

## Optional editor and CubeMX workflow

VS Code can use CMake Tools, clangd or the C/C++ extension, and Cortex-Debug.
The workspace includes Debug configure/build presets and ST-LINK launch/attach
configurations.

The checked-in .vscode files contain local assumptions: cube-cmake as the
CMake executable, extension-specific CubeCLT paths, a Debug_Clang build task,
and a probe serial number in the launch configuration. These settings are
installation-specific; the serial number selects an individual ST-LINK probe.

rapid_drive_foc.ioc contains the CubeMX peripheral configuration. Generated
sources use USER CODE sections for application integration; the main application
logic resides in Foc.

## Tests and troubleshooting

The calibration/identification routine lifecycle and executable C tests
are documented in [FOC routines](docs/foc-routines.md).

The plotter repository contains protocol, cache, simulator, and dynamic-UI
tests:

```powershell
cd ..\rapid-drive-plotter
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
.\.venv\Scripts\python.exe tests/smoke_app.py
```

The contract test checks the C lists and command IDs using a sibling
rapid-drive-foc checkout. RAPID_DRIVE_FIRMWARE selects an alternative firmware
checkout. The GUI smoke test runs the simulator offscreen and writes temporary
HDF5 output; it does not exercise real USB/CAN hardware or flash the controller.

| Symptom | Check |
| --- | --- |
| Compiler, Ninja, or programmer not found | Tool installation directory and PATH in the terminal used by CMake/editor |
| CMake refers to another machine's tools | Cached tool paths in the build directory; a fresh configuration uses the local toolchain |
| Linker reports region overflow | Memory report/map and the application/storage split |
| ST-LINK cannot connect | Probe driver/firmware, target power, SWD/NRST wiring, and other tools holding the probe |
| No plotter serial port | USB data cable, device enumeration, board initialization, and whether the MCU is in its system bootloader |
| Protocol mismatch or timeout | Matched firmware/plotter protocol versions, correct port, and a single application owning it |
| Changed defaults are not taking effect | Existing flash configuration may still be loaded |
| No cyclic CAN replies | Assigned nonzero node ID, enabled tick periods, CAN FD/BRS bus settings, transceiver, and termination |
