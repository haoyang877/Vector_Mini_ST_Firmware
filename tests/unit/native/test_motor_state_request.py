"""验证核心参数、三类标定通信入口及退役请求；不访问硬件。"""

import argparse
import json
import re
import subprocess
from pathlib import Path

from run_position_servo_tests import NATIVE_INCLUDE_FLAGS, ROOT


def check_parameter_catalog() -> None:
    """核对共享目录与生产字段类型、倍率及权限，防止客户端另建漂移数值表。"""
    catalog = json.loads(
        (ROOT / "shared/protocol/parameter_catalog_v1.json").read_text(encoding="utf-8")
    )
    rows = {entry["id"]: entry for entry in catalog["parameters"]}
    source = (ROOT / "firmware/parameters/parameter_values.c").read_text(encoding="utf-8")
    fields = source.split("static const ScalarField s_fields[] = {", 1)[1].split("};", 1)[0]
    field_ids = set()
    pattern = r"(READ|WRITE|CAL)_FIELD\((0x[0-9A-Fa-f]+|[0-9]+)U,\s*[^,]+,\s*([0-9]+)U\)"
    for kind, number, scale in re.findall(pattern, fields):
        parameter_id = int(number, 0)
        field_ids.add(parameter_id)
        expected = (1, int(scale), {"READ": 1, "WRITE": 3, "CAL": 5}[kind])
        row = rows[parameter_id]
        actual = (row["value_type"], row["scale"], row["access"])
        if actual != expected:
            raise RuntimeError(f"共享参数目录与生产映射不一致：{parameter_id}")
    read_cases = source.split("ParameterStatus ParameterValue_Read", 1)[1].split(
        "static float speed_ceiling", 1
    )[0]
    special_ids = {
        int(value, 0) for value in re.findall(r"case (0x[0-9A-Fa-f]+|[0-9]+)U:", read_cases)
    }
    if set(rows) != field_ids | special_ids | {0x70}:
        raise RuntimeError("共享参数目录遗漏或重复登记了生产ID")
    header = (ROOT / "firmware/parameters/parameter_values.h").read_text(encoding="utf-8")
    for macro, ids in (
        ("PARAMETER_PI_KP_MAX", (11, 13, 16)),
        ("PARAMETER_CURRENT_KI_MAX", (12, 14)),
        ("PARAMETER_SPEED_KI_MAX", (17,)),
    ):
        limit = float(re.search(rf"#define {macro} ([0-9.]+)f", header)[1])
        if any(rows[parameter_id]["maximum"] != limit for parameter_id in ids):
            raise RuntimeError(f"共享目录范围与生产上限不一致：{macro}")


def main() -> None:
    """编译真实参数所有者和协议适配器，检查标定映射及黄金向量。"""
    check_parameter_catalog()
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
                (ROOT / "tests/fixtures/protocol/yg_protocol_core_v1_9_vectors.json").read_text(
                    encoding="utf-8"
                )
            )
            for vector in vectors["vectors"]:
                expected = f"VECTOR {vector['name']} {vector['payload_hex']}"
                if expected not in logs[-1].splitlines():
                    raise RuntimeError(f"协议向量不匹配: {expected}")


if __name__ == "__main__":
    main()
