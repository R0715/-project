import struct
import sys
from pathlib import Path

FLASH_START = 0x08000000
FLASH_END = FLASH_START + 64 * 1024
SRAM_START = 0x20000000
SRAM_END = SRAM_START + 20 * 1024
MACH_HEADER = struct.Struct("<7I")
SEGMENT = struct.Struct("<2I16s8I")
SECTION = struct.Struct("<16s16s9I")


def flash_sections(image):
    magic, cpu, subtype, filetype, command_count, _, _ = MACH_HEADER.unpack_from(image)
    if (magic, cpu, subtype, filetype) != (0xFEEDFACE, 12, 15, 2):
        raise ValueError("Expected a linked 32-bit ARMv7-M Mach-O image")

    sections = []
    offset = MACH_HEADER.size
    for _ in range(command_count):
        command, size = struct.unpack_from("<II", image, offset)
        if command == 1:  # LC_SEGMENT
            section_count = SEGMENT.unpack_from(image, offset)[-2]
            for i in range(section_count):
                position = offset + SEGMENT.size + i * SECTION.size
                name, _, address, length, file_offset, _, _, _, flags, _, _ = (
                    SECTION.unpack_from(image, position)
                )
                if length == 0:
                    continue
                if FLASH_START <= address < address + length <= FLASH_END:
                    data = image[file_offset:file_offset + length]
                    if file_offset == 0 or len(data) != length:
                        raise ValueError(f"Flash section is missing data: {name!r}")
                    sections.append((address, data))
                elif not (
                    SRAM_START <= address < address + length < SRAM_END
                    and name.rstrip(b"\0") == b"__bss"
                    and (flags & 0xFF) == 1  # S_ZEROFILL. Reset_Handler clears __bss.
                ):
                    raise ValueError(f"Unexpected section at {address:#010x}: {name!r}")
        offset += size

    sections.sort()
    if not sections or sections[0][0] != FLASH_START or len(sections[0][1]) < 57 * 4:
        raise ValueError("Missing Flash vector table")
    vectors = sections[0][1]
    if struct.unpack_from("<I", vectors)[0] != SRAM_END:
        raise ValueError("Incorrect initial stack pointer")
    for index in (1, 15, 23, 56):  # Reset, SysTick, EXTI1, EXTI15_10.
        entry = struct.unpack_from("<I", vectors, index * 4)[0]
        if not (entry & 1 and any(a <= entry - 1 < a + len(data) for a, data in sections[1:])):
            raise ValueError(f"Invalid Thumb handler in vector {index}")
    return sections


def record(address, kind, data):
    payload = bytes([len(data)]) + address.to_bytes(2, "big") + bytes([kind]) + data
    return ":" + (payload + bytes([-sum(payload) & 0xFF])).hex().upper()


def encode_hex(sections):
    # The STM32F103C8's 64 KiB of Flash uses one extended linear address.
    yield record(0, 4, (FLASH_START >> 16).to_bytes(2, "big"))
    for address, data in sections:
        for offset in range(0, len(data), 16):
            yield record(address + offset - FLASH_START, 0, data[offset:offset + 16])
    yield record(0, 1, b"")


if __name__ == "__main__":
    source, target = map(Path, sys.argv[1:])
    content = "\n".join(encode_hex(flash_sections(source.read_bytes()))) + "\n"
    target.write_text(content, encoding="ascii")
