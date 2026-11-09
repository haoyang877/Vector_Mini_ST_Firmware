"""验证核心参数、三类标定通信入口及退役请求；不访问硬件。"""

import argparse
import json
import subprocess
from pathlib import Path

from run_position_servo_tests import NATIVE_INCLUDE_FLAGS, ROOT


def main() -> None:
    """编译真实参数所有者和协议适配器，检查标定映射及黄金向量。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", required=True)
    parser.add_argument("--out", type=Path, default=ROOT / "outputs/core_parameter_tests")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    compiler = [args.cc] + (["cc"] if Path(args.cc).stem.lower() == "zig" else [])
    for name, implementation in (("core_protocol", "firmware/app/app_protocol.c"),):
        executable = args.out / f"{name}.exe"
        command = (
            compiler
            + NATIVE_INCLUDE_FLAGS
            + [
                "-std=c99",
                "-O1",
                "-UNDEBUG",
                "-Wall",
                "-Wextra",
                "-Werror",
                "-DBSP_BOARD=1",
                str(ROOT / f"tests/unit/native/{name}_test.c"),
                str(ROOT / implementation),
                "-o",
                str(executable),
            ]
        )
        if name == "core_protocol":
            command += [
                str(ROOT / "shared/protocol/yg_protocol_crc.c"),
                str(ROOT / "shared/protocol/yg_protocol_frame_codec.c"),
                str(ROOT / "shared/protocol/yg_protocol_can_id.c"),
                str(ROOT / "tests/unit/yg_protocol_bsp_stubs.c"),
                str(ROOT / "tests/unit/bsp_preemption_stubs.c"),
            ]
        logs = []
        for invocation in (command, [str(executable)]):
            result = subprocess.run(invocation, capture_output=True, text=True, check=False)
            logs.append(result.stdout + result.stderr)
            (args.out / f"{name}.log").write_text("\n".join(logs), encoding="utf-8")
            print(logs[-1], end="")
            result.check_returncode()
        if name == "core_protocol":
            vectors = json.loads(
                (ROOT / "tests/fixtures/protocol/yg_protocol_core_v1_8_vectors.json").read_text(
                    encoding="utf-8"
                )
            )
            for vector in vectors["vectors"]:
                expected = f"VECTOR {vector['name']} {vector['payload_hex']}"
                if expected not in logs[-1].splitlines():
                    raise RuntimeError(f"协议向量不匹配: {expected}")


if __name__ == "__main__":
    main()
