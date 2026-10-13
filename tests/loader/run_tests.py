"""离线编译真实Loader C；假Flash不接触探针、电机或CAN。"""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys


def main():
    """生成独立黄金输入，编译真实核心并运行故障场景。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", required=True)
    parser.add_argument("--out", type=Path, default=Path("outputs/loader_native"))
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    vectors = root / "docs/plans/2026-10-13-can-loader-minimal-v1-vectors.json"
    data = json.loads(vectors.read_text(encoding="utf-8"))
    rows = []
    for case in data["frames"]:
        raw = bytes.fromhex(case["frame_hex"])
        values = ",".join(str(x) for x in raw)
        rows.append(
            "{{"
            + values
            + "},"
            + str(len(raw))
            + ","
            + str(len(bytes.fromhex(case["payload_hex"])))
            + "}"
        )
    (out / "golden_vectors.h").write_text(
        "static const struct { unsigned char bytes[64]; unsigned char length, payload_length; } golden[] = {\n"
        + ",\n".join(rows)
        + "\n};\nstatic const unsigned golden_count = sizeof(golden)/sizeof(golden[0]);\n",
        encoding="ascii",
    )
    sources = [
        "tests/loader/loader_core_test.c",
        "shared/boot_contract.c",
        "loader/firmware/loader_core.c",
        "loader/firmware/loader_proto.c",
        "firmware/services/boot_handoff/boot_handoff.c",
    ]
    compiler = [args.cc] + (["cc"] if Path(args.cc).stem == "zig" else [])
    exe = out / "loader_test.exe"
    command = compiler + [
        "-std=c99",
        "-O1",
        "-Wall",
        "-Wextra",
        "-Werror",
        "-UNDEBUG",
        "-I",
        "shared",
        "-I",
        "loader/firmware",
        "-I",
        str(out),
        "-I",
        "firmware/services/boot_handoff",
        *sources,
        "-o",
        str(exe),
    ]
    logs = []
    for cmd in [
        command,
        [str(exe)],
        [str(exe), "stop-failure"],
        [str(exe), "tx-failure"],
        [str(exe), "success"],
        [sys.executable, str(root / "tests/loader/test_host_bridge.py"), str(exe)],
        [sys.executable, str(root / "tests/loader/test_cycle_guard.py")],
        [
            sys.executable,
            str(root / "tests/loader/test_startup_parameter_read.py"),
            args.cc,
            str(out),
        ],
    ]:
        result = subprocess.run(cmd, cwd=root, capture_output=True, text=True)
        logs.append(result.stdout + result.stderr)
        (out / "test.log").write_text("\n".join(logs), encoding="utf-8")
        print(result.stdout + result.stderr, end="")
        result.check_returncode()
    summary = {
        "status": "PASS",
        "hardware_accessed": False,
        "source_sha256": {
            p: hashlib.sha256((root / p).read_bytes()).hexdigest() for p in sources
        },
        "startup_read_sha256": hashlib.sha256(
            (root / "firmware/platform/stm32g4/bsp/flash.c").read_bytes()
        ).hexdigest(),
        "vectors_sha256": hashlib.sha256(vectors.read_bytes()).hexdigest(),
    }
    (out / "summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )


if __name__ == "__main__":
    main()
