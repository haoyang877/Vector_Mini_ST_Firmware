"""从版本化检测契约生成固件默认值与合法范围，不访问设备。"""

import json
import subprocess
import sys
from pathlib import Path


def main() -> None:
    """生成固定尺寸的阈值目录，字段顺序同时用于线路编码。"""
    root = Path(__file__).resolve().parents[2]
    source = root / "shared/protocol/hardware_check_limits_v1.json"
    fields = json.loads(source.read_text(encoding="utf-8"))["fields"]
    lines = [
        "/* 由 generate_hardware_limits.py 生成，修改版本化JSON契约。 */",
        "#ifndef HARDWARE_CHECK_LIMITS_H",
        "#define HARDWARE_CHECK_LIMITS_H",
        "#include <stdint.h>",
        "enum",
        "{",
        *[f"    CHECK_{name.upper()}," for name, *_ in fields],
        "    CHECK_LIMIT_COUNT",
        "};",
    ]
    for label, index in (("DEFAULTS", 1), ("MINIMUMS", 2), ("MAXIMUMS", 3)):
        values = ", ".join(str(field[index]) + "U" for field in fields)
        lines.append(f"#define CHECK_LIMIT_{label} {{{values}}}")
    lines += ["#endif", ""]
    header = source.parent / "hardware_check_limits.h"
    header.write_text("\n".join(lines), encoding="utf-8")
    subprocess.run([sys.executable, str(root / "tools/run.py"), "format", str(header)], check=True)


if __name__ == "__main__":
    main()
