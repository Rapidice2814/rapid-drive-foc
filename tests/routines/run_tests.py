"""Compile real C routines for ARM and execute them without target hardware."""
import argparse
from pathlib import Path
import shutil
import subprocess
import tempfile

from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_HOOK_CODE
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_R0, UC_ARM_REG_PC


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler", default=shutil.which("arm-none-eabi-gcc"))
    args = parser.parse_args()
    if args.compiler is None:
        parser.error("Supply --compiler with the path to arm-none-eabi-gcc")
    here = Path(__file__).resolve().parent
    root = here.parents[1]
    with tempfile.TemporaryDirectory(prefix="foc-routines-") as directory:
        elf_path = Path(directory) / "routines.elf"
        subprocess.run([
            args.compiler, "-mcpu=cortex-m3", "-mthumb", "-mfloat-abi=soft",
            "-std=gnu11", "-O1", "-Wall", "-Wextra", "-Werror", "-fno-builtin",
            "-nostdlib", "-Wl,-Ttext=0x10000,-Tdata=0x40000,-e,test_alignment_success",
            "-I" + str(here / "stubs"), "-I" + str(root / "Foc/FOC_Loops"),
            "-I" + str(root / "Foc/FOC_States"), "-I" + str(root / "Foc/Utils"),
            "-I" + str(root / "Foc/FOC_Core"),
            "-I" + str(root / "Foc/FOC_Drivers"),
            "-include", str(here / "stubs/encoder_sources.h"),
            "-include", str(here / "stubs/FOC_Handle.h"),
            str(here / "test_routines.c"),
            str(root / "Foc/FOC_Loops/FOC_Alignment.c"),
            str(root / "Foc/FOC_Loops/FOC_CurrentSensorCalibration.c"),
            str(root / "Foc/FOC_Loops/FOC_EncoderCalibration.c"),
            str(root / "Foc/FOC_Drivers/FOC_Encoder.c"),
            str(root / "Foc/FOC_Drivers/FOC_Calibration.c"),
            str(root / "Foc/FOC_Loops/FOC_MotorIdentification.c"),
            str(root / "Foc/FOC_Loops/FOC_PIDAutotune.c"),
            str(root / "Foc/FOC_Loops/FOC_AlignmentTest.c"),
            str(root / "Foc/FOC_Loops/FOC_AntiCogging.c"),
            str(root / "Foc/FOC_Loops/FOC_OpenLoop.c"),
            str(root / "Foc/FOC_States/FOC_StateController.c"),
            str(root / "Foc/FOC_States/FOC_States_Calibration.c"),
            str(root / "Foc/FOC_States/FOC_States_Storage.c"),
            str(root / "Foc/FOC_States/FOC_States_Startup.c"),
            str(root / "Foc/FOC_States/FOC_States_Operation.c"),
            str(root / "Foc/FOC_Core/FOC_Diagnostics.c"),
            str(root / "Foc/FOC_Core/FOC_Configuration.c"),
            "-Wl,--start-group", "-lm", "-lc", "-lgcc", "-lnosys", "-Wl,--end-group", "-o", str(elf_path),
        ], check=True)
        with elf_path.open("rb") as stream:
            elf = ELFFile(stream)
            segments = [(s["p_vaddr"], s.data()) for s in elf.iter_segments() if s["p_type"] == "PT_LOAD"]
            tests = {s.name: s["st_value"] for s in elf.get_section_by_name(".symtab").iter_symbols()
                     if s.name.startswith("test_") and s["st_info"]["type"] == "STT_FUNC"}
        for name, entry in sorted(tests.items()):
            cpu = Uc(UC_ARCH_ARM, UC_MODE_THUMB)
            cpu.mem_map(0x10000, 0x100000)
            cpu.mem_map(0x20000000, 0x10000)
            cpu.mem_map(0x30000000, 0x1000)
            for address, data in segments:
                cpu.mem_write(address, data)
            cpu.reg_write(UC_ARM_REG_SP, 0x2000FFF0)
            cpu.reg_write(UC_ARM_REG_LR, 0x30000001)
            def stop_on_return(uc, address, size, data):
                if address == 0x30000000:
                    uc.emu_stop()
            cpu.hook_add(UC_HOOK_CODE, stop_on_return, begin=0x30000000, end=0x30000000)
            cpu.emu_start(entry | 1, 0x30000002, timeout=60_000_000, count=200_000_000)
            if cpu.reg_read(UC_ARM_REG_PC) != 0x30000000:
                raise AssertionError(f"{name}: did not return within the execution budget (PC={cpu.reg_read(UC_ARM_REG_PC):#x})")
            line = cpu.reg_read(UC_ARM_REG_R0)
            if line:
                raise AssertionError(f"{name}: CHECK failed at test_routines.c:{line}")
            print(f"PASS: {name}")
        print(f"{len(tests)} compiled-C test groups passed (mock peripherals, not hardware).")


if __name__ == "__main__":
    main()
