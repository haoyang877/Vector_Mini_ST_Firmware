"""验证共享 CRC32、字节序、请求条件；不访问硬件。"""

from __future__ import annotations

import argparse
import random
import subprocess
import sys
import zlib
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
from project_paths import NATIVE_INCLUDE_FLAGS, ROOT


def main() -> None:
    """以独立标准库生成 CRC 向量，再执行生产 C 实现。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", required=True)
    parser.add_argument("--out", type=Path, default=ROOT / "outputs/shared_contracts")
    args = parser.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    payload = random.Random(20260929).randbytes(2048)
    rows = []
    for n in (0, 1, 2, 7, 28, 31, 64, 96, 255, 1024, 2048):
        for seed in (0, 1, 0x12345678, 0xFFFFFFFF):
            rows.append(f"{{{n}U,{seed}U,{zlib.crc32(payload[:n], seed)}U}}")
    header = "typedef struct {uint32_t bytes, seed, expected;} CrcVector;\n"
    header += "static const uint8_t payload[]={" + ",".join(map(str, payload)) + "};\n"
    header += "static const CrcVector vectors[]={" + ",".join(rows) + "};\n"
    (out / "crc_vectors.h").write_text(header, encoding="utf-8")
    cc = [args.cc] + (["cc"] if Path(args.cc).stem == "zig" else [])
    exe = out / "contracts.exe"
    command = (
        cc
        + NATIVE_INCLUDE_FLAGS
        + [
            "-I",
            str(out),
            "-std=c99",
            "-O2",
            "-UNDEBUG",
            "-Wall",
            "-Wextra",
            "-Werror",
            "tests/unit/shared_contracts_test.c",
            "shared/checksum/crc32.c",
            "-o",
            str(exe),
        ]
    )
    logs = []
    for invocation in (command, [str(exe)]):
        result = subprocess.run(invocation, cwd=ROOT, capture_output=True, text=True)
        logs.append(result.stdout + result.stderr)
        (out / "test.log").write_text("\n".join(logs), encoding="utf-8")
        print(logs[-1], end="")
        result.check_returncode()


if __name__ == "__main__":
    main()
