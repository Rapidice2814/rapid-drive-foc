"""Validate whole-project compiler policy and the persistent ARM image layout."""
import argparse
import json
from pathlib import Path
import re
import struct

from elftools.elf.elffile import ELFFile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("build", type=Path)
    args = parser.parse_args()
    commands = json.loads((args.build / "compile_commands.json").read_text())
    c_commands = [item for item in commands if item["file"].endswith(".c")]
    if not c_commands:
        raise AssertionError("No C compile commands found")
    for item in c_commands:
        command = item["command"]
        levels = re.findall(r"(?<!\S)-O(?:[0-3sgz]|fast)(?=\s|$)", command)
        if not levels or levels[-1] != "-Oz" or "-flto" not in command or "-fno-fast-math" not in command:
            raise AssertionError(f"Incorrect optimization policy: {item['file']}")
    with (args.build / "rapid_drive_foc.elf").open("rb") as stream:
        elf = ELFFile(stream)
        permanent = elf.get_section_by_name(".permanent")
        if (permanent is None or permanent["sh_addr"] != 0x0801C000 or
                permanent["sh_type"] != "SHT_NOBITS" or not 0 < permanent["sh_size"] <= 0x4000):
            raise AssertionError("Persistent configuration reservation missing or moved")
        vector = elf.get_section_by_name(".isr_vector")
        if vector is None or vector["sh_addr"] != 0x08000000:
            raise AssertionError("Interrupt vectors missing or moved")
        stack, reset = struct.unpack_from("<II", vector.data())
        if stack != 0x20008000 or not reset & 1 or not 0x08000000 <= (reset & ~1) < 0x0801C000:
            raise AssertionError("Invalid initial stack/reset vector")
        for segment in elf.iter_segments():
            if segment["p_type"] != "PT_LOAD" or not segment["p_filesz"]:
                continue
            address, size = segment["p_paddr"], segment["p_filesz"]
            if not 0x08000000 <= address < address + size <= 0x0801C000:
                raise AssertionError("Loadable image overlaps saved configuration or lies outside application flash")
    binary = args.build / "rapid_drive_foc.bin"
    if not 8 <= binary.stat().st_size <= 112 * 1024:
        raise AssertionError("Invalid application binary size")
    print(f"PASS: {len(c_commands)} C commands use -Oz/LTO; reset vector, binary and NOLOAD storage bounds preserved")


if __name__ == "__main__":
    main()
