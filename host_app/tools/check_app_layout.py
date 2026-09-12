"""Post-build layout gate: fail the Keil build if the image could overlap the Loader.

Wired into the Keil projects' "After Build" step. It parses the linker map and
verifies the single load region matches the registered partition:

    app    : base 0x08004000, max 0x18000 (hard fail if size > 0x17000)
    loader : base 0x08000000, max 0x3000

Exit codes: 0 = OK, 1 = layout violation, 2 = cannot read/parse the map.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import re
import sys

LAYOUTS = {
    "app": {"base": 0x08004000, "max": 0x18000, "hard_limit": 0x17000, "soft_limit": 0x16800},
    "loader": {"base": 0x08000000, "max": 0x3000, "hard_limit": 0x3000, "soft_limit": 0x3800},
}

REGION_RE = re.compile(
    r"Load Region LR_IROM1 \(Base: 0x([0-9A-Fa-f]+), Size: 0x([0-9A-Fa-f]+), Max: 0x([0-9A-Fa-f]+)"
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", type=Path, required=True)
    parser.add_argument("--target", choices=sorted(LAYOUTS), required=True)
    args = parser.parse_args()

    try:
        text = args.map.read_text(encoding="utf-8", errors="replace")
    except OSError as exc:
        print(f"[layout-gate] FAIL: cannot read map {args.map}: {exc}")
        return 2

    matches = REGION_RE.findall(text)
    if len(matches) != 1:
        print(f"[layout-gate] FAIL: expected exactly one LR_IROM1 region, found {len(matches)}")
        return 2

    base, size, max_size = (int(value, 16) for value in matches[0])
    spec = LAYOUTS[args.target]
    errors = []
    if base != spec["base"]:
        errors.append(f"base 0x{base:08X} != 0x{spec['base']:08X}")
    if max_size != spec["max"]:
        errors.append(f"max 0x{max_size:08X} != 0x{spec['max']:08X}")
    if size > spec["hard_limit"]:
        errors.append(f"size 0x{size:04X} exceeds hard limit 0x{spec['hard_limit']:04X}")
    if errors:
        print("[layout-gate] FAIL " + "; ".join(errors))
        print("[layout-gate] 该镜像可能覆盖 Loader/参数区，禁止下载！检查 scatter 与 IROM 设置。")
        return 1

    warn = f" (注意: 已超软限 0x{spec['soft_limit']:04X})" if size > spec["soft_limit"] else ""
    print(
        f"[layout-gate] OK {args.target}: base 0x{base:08X}, size 0x{size:04X}, "
        f"max 0x{max_size:08X}{warn}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
