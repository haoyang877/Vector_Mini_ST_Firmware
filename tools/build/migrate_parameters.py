"""离线迁移单份参数到schema14，保留标定、不连接设备、不授予配置确认。"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from prepare_calibration_reset import prepare


def main() -> None:
    """要求完整快照、已核对哈希与新文件路径；旧无CRC布局需要显式选择。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--source-sha256", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--node", type=int)
    parser.add_argument("--legacy-layout", choices=("vector-mini-schema10-delivery-v1",))
    args = parser.parse_args()
    if args.output.exists() or args.output.with_suffix(".json").exists():
        parser.error("输出必须是新文件，禁止覆盖原始快照")
    try:
        output, report = prepare(
            args.input.read_bytes(),
            args.source_sha256,
            args.node,
            preserve_calibration=True,
            legacy_layout=args.legacy_layout,
        )
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_bytes(output)
    args.output.with_suffix(".json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, ensure_ascii=True))


if __name__ == "__main__":
    main()
