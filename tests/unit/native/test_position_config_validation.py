"""对真实位置配置判定执行特殊浮点、阈值邻域及多字段随机对照；不接触硬件。"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
from project_paths import NATIVE_INCLUDE_FLAGS, ROOT
from run_position_servo_tests import function_source


def main() -> None:
    """仅在输出目录生成生产函数切片；参考判定独立冻结在测试夹具中。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", required=True)
    parser.add_argument(
        "--out", type=Path, default=ROOT / "outputs/tests/position_config_validation"
    )
    args = parser.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    source = (ROOT / "firmware/motor/position/position_cascade.c").read_text(encoding="utf-8")
    header = (ROOT / "firmware/motor/position/position_cascade.h").read_text(encoding="utf-8")
    config_body = header[: header.index("} PositionCascadeConfig;")]
    fields = re.findall(r"\bfloat\s+(\w+)\s*;", config_body)
    # 从公开结构独立枚举字段，生产有限值表漏项时测试仍覆盖该项。
    offsets = (
        "static const size_t test_offsets[] = {\n"
        + ",\n".join(f"offsetof(PositionCascadeConfig, {field})" for field in fields)
        + "};\n"
    )
    scenario = (ROOT / "tests/unit/position_servo_test.c").read_text(encoding="utf-8")
    defaults = function_source(scenario, "config").replace("config(void)", "valid_config(void)")
    actual = function_source(source, "position_cascade_config_is_valid")
    (out / "validation_actual.inc").write_text(offsets + defaults + actual, encoding="utf-8")
    compiler = Path(args.cc)
    command = [str(compiler)] + (["cc"] if compiler.stem.lower() == "zig" else [])
    command += NATIVE_INCLUDE_FLAGS + [
        "-I",
        str(out),
        "-std=c99",
        "-O2",
        "-Wall",
        "-Wextra",
        "-Werror",
    ]
    executable = out / ("config_validation.exe" if sys.platform == "win32" else "config_validation")
    command += [
        str(ROOT / "tests/unit/native/position_config_validation_test.c"),
        "-o",
        str(executable),
    ]
    logs = []
    for invocation in (command, [str(executable)]):
        result = subprocess.run(invocation, cwd=ROOT, capture_output=True, text=True)
        logs.append(result.stdout + result.stderr)
        (out / "test.log").write_text("\n".join(logs), encoding="utf-8")
        print(logs[-1], end="")
        result.check_returncode()


if __name__ == "__main__":
    main()
