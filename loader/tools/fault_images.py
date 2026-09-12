"""Deterministic synthetic APP images with controlled defects for Loader fault tests.

Each image is a minimal but layout-correct Cortex-M firmware for ``APP_BASE``:
a 256-byte vector table, a Thumb code stub at offset 0x100, and 0xFF padding.
Exactly one safety-relevant field is defective per image so the *Loader's own*
checks become the observable:

- ``hang_no_comms``      : fully valid; reset handler spins forever (never talks)
- ``return_from_reset``  : fully valid; reset handler returns -> Loader resets
- ``bad_sp_out_of_sram`` : initial SP outside the accepted SRAM window
- ``bad_sp_unaligned``   : initial SP not 8-byte aligned
- ``bad_reset_thumb_bit``: reset vector bit0 missing (not a Thumb address)
- ``bad_reset_outside``  : reset vector beyond the recorded image size

The loader-side decision matrix lives in ``loader_jump.c:loader_image_valid``;
the HIL expectations live in ``loader/tools/loader_fault_test.py``.

Run ``python fault_images.py --outdir <dir>`` to materialize all images plus a
``manifest.json`` with size/CRC/boot-validity for manual inspection.
"""

import argparse
from dataclasses import dataclass
import json
from pathlib import Path
import struct

from loader_proto import APP_BASE_ADDRESS, crc32

APP_BASE = APP_BASE_ADDRESS
IMAGE_SIZE = 0x200  # every variant is one 512 B image, 8-byte aligned
TABLE_SIZE = 0x100  # 64 vectors
CODE_OFFSET = 0x100  # Thumb stub right behind the table
VALID_SP = 0x20008000  # stack top accepted by loader_image_valid

STUB_LOOP = bytes.fromhex("fee7")  # b .    : runs but never communicates
STUB_RETURN = bytes.fromhex("7047")  # bx lr : returns into the Loader

VECTOR_NAMES = ("NMI", "HardFault", "MemManage", "BusFault", "UsageFault")


@dataclass(frozen=True)
class FaultImage:
    """One synthetic image plus the boot-validation verdict it must provoke."""

    name: str
    data: bytes
    boot_valid: bool
    note: str

    @property
    def size(self) -> int:
        return len(self.data)

    @property
    def crc32(self) -> int:
        return crc32(self.data)


def _build(*, sp: int, reset: int, size: int = IMAGE_SIZE, stub: bytes = STUB_LOOP) -> bytes:
    """Lay out vectors + stub; every handler points at the stub inside the image."""
    if size % 8 or size < CODE_OFFSET + len(stub):
        raise ValueError(f"image size {size:#x} cannot hold the stub")
    buffer = bytearray(b"\xff" * size)
    vector = [0] * (TABLE_SIZE // 4)
    vector[0] = sp
    vector[1] = reset
    handler = (APP_BASE + CODE_OFFSET) | 1
    for index in range(2, len(vector)):
        vector[index] = handler
    buffer[0:TABLE_SIZE] = struct.pack(f"<{len(vector)}I", *vector)
    buffer[CODE_OFFSET:CODE_OFFSET + len(stub)] = stub
    return bytes(buffer)


def build_all() -> list[FaultImage]:
    """Return every fault variant in a fixed order (safe images first)."""
    reset_ok = (APP_BASE + CODE_OFFSET) | 1
    return [
        FaultImage(
            "hang_no_comms",
            _build(sp=VALID_SP, reset=reset_ok),
            True,
            "vector table valid, CRC valid; reset handler spins forever, CAN never answers",
        ),
        FaultImage(
            "return_from_reset",
            _build(sp=VALID_SP, reset=reset_ok, stub=STUB_RETURN),
            True,
            "reset handler returns immediately -> Loader NVIC_SystemReset() loop",
        ),
        FaultImage(
            "bad_sp_out_of_sram",
            _build(sp=0x10000000, reset=reset_ok),
            False,
            "initial SP outside 0x20000020..0x20008000 -> boot validation must refuse",
        ),
        FaultImage(
            "bad_sp_unaligned",
            _build(sp=0x200063E5, reset=reset_ok),
            False,
            "initial SP not 8-byte aligned -> boot validation must refuse",
        ),
        FaultImage(
            "bad_reset_thumb_bit",
            _build(sp=VALID_SP, reset=APP_BASE + CODE_OFFSET),
            False,
            "reset vector bit0 = 0 -> boot validation must refuse",
        ),
        FaultImage(
            "bad_reset_outside",
            _build(sp=VALID_SP, reset=(APP_BASE + 0x400) | 1),
            False,
            "reset vector beyond the recorded size -> boot validation must refuse",
        ),
    ]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--outdir", type=Path, required=True)
    args = parser.parse_args()
    args.outdir.mkdir(parents=True, exist_ok=True)

    manifest = []
    for image in build_all():
        path = args.outdir / f"{image.name}.bin"
        path.write_bytes(image.data)
        manifest.append({
            "name": image.name,
            "file": path.name,
            "size": image.size,
            "crc32": f"0x{image.crc32:08X}",
            "boot_valid": image.boot_valid,
            "note": image.note,
        })
    (args.outdir / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(f"{len(manifest)} fault images -> {args.outdir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
