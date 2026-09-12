"""Post-download check: prove the resident Loader is intact after flashing an APP.

Reads the loader partition (0x08000000..) over SWD and byte-compares it with a
reference image. Use after any Keil/J-Link download of the APP.

Reference resolution order:
    1. --expected <loader.bin>
    2. loader/mdk/Vector_Mini_ST_Loader/Vector_Mini_ST_Loader.axf -> fromelf
    3. outputs/loader_migration_20260912/loader_raw.bin (historical build)

Usage:
    python host_app/tools/verify_loader_intact.py [--expected loader.bin] [--jlink PATH]
"""

from __future__ import annotations

import argparse
from pathlib import Path
import subprocess
import sys
import tempfile

REPO = Path(__file__).resolve().parents[2]
DEFAULT_JLINK = Path(r"C:\Program Files\SEGGER\JLink_V964\JLink.exe")
DEFAULT_FROMELF = Path(r"C:\Keil_v5\ARM\ARMCC\bin\fromelf.exe")
LOADER_BASE = 0x08000000
FALLBACK_BIN = REPO / "outputs" / "loader_migration_20260912" / "loader_raw.bin"
LOADER_AXF = REPO / "loader" / "mdk" / "Vector_Mini_ST_Loader" / "Vector_Mini_ST_Loader.axf"


def resolve_expected(path: Path | None) -> Path:
    if path is not None:
        if not path.exists():
            raise SystemExit(f"error: expected image not found: {path}")
        return path
    if LOADER_AXF.exists() and DEFAULT_FROMELF.exists():
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "loader.bin"
            result = subprocess.run(
                [str(DEFAULT_FROMELF), "--bin", f"--output={out}", str(LOADER_AXF)],
                capture_output=True,
                text=True,
                timeout=120,
            )
            if result.returncode == 0 and out.exists():
                cached = Path(tempfile.gettempdir()) / "loader_reference.bin"
                cached.write_bytes(out.read_bytes())
                return cached
    if FALLBACK_BIN.exists():
        return FALLBACK_BIN
    raise SystemExit("error: no loader reference image available")


def read_loader(jlink: Path, size: int) -> bytes:
    with tempfile.TemporaryDirectory() as tmp:
        dump = Path(tmp) / "loader_dump.bin"
        script = Path(tmp) / "read.jlink"
        script.write_text(
            "si SWD\nspeed 4000\nconnect\n"
            f'savebin "{dump.as_posix()}", 0x{LOADER_BASE:08X}, 0x{size:X}\n'
            "q\n",
            encoding="ascii",
        )
        result = subprocess.run(
            [str(jlink), "-device", "STM32G431CB", "-if", "SWD", "-speed", "4000",
             "-autoconnect", "1", "-CommanderScript", str(script)],
            capture_output=True,
            text=True,
            timeout=120,
        )
        if not dump.exists():
            raise SystemExit(f"error: JLink read failed:\n{result.stdout}\n{result.stderr}")
        return dump.read_bytes()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--expected", type=Path, default=None)
    parser.add_argument("--jlink", type=Path, default=DEFAULT_JLINK)
    args = parser.parse_args()

    if not args.jlink.exists():
        print(f"error: JLink not found: {args.jlink}")
        return 2
    expected = resolve_expected(args.expected).read_bytes()
    actual = read_loader(args.jlink, len(expected))

    sp = int.from_bytes(actual[0:4], "little")
    reset = int.from_bytes(actual[4:8], "little")
    print(f"reference: {len(expected):,} B  |  flash read: {len(actual):,} B")
    print(f"loader vector: SP=0x{sp:08X} reset=0x{reset:08X}")
    print(f"  expected[0:16] {expected[:16].hex(' ')}")
    print(f"  actual  [0:16] {actual[:16].hex(' ')}")
    if actual == expected:
        print(f"PASS: Loader intact ({len(expected):,} B @ 0x08000000), APP download did not touch it")
        return 0
    if len(actual) != len(expected):
        print(f"FAIL: length mismatch ({len(actual)} vs {len(expected)})")
    limit = min(len(expected), len(actual))
    for index in range(limit):
        if expected[index] != actual[index]:
            print(f"FAIL: first difference at 0x08000000+0x{index:X}: expected 0x{expected[index]:02X}, read 0x{actual[index]:02X}")
            break
    else:
        if len(actual) != len(expected):
            print("FAIL: common prefix identical but lengths differ")
    print("FAIL: Loader does not match the reference image!")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
