"""仓库命令模板：导入时无副作用，默认不接触硬件。"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class ToolRequest:
    """命令层拥有且已经完成类型转换的输入。"""

    input_path: Path
    output_path: Path


def execute(request: ToolRequest) -> None:
    """执行确定性的文件操作；失败时抛出包含修复方法的异常。"""
    if not request.input_path.is_file():
        raise FileNotFoundError(
            f"Input does not exist: {request.input_path}. Provide an existing file with --input."
        )
    request.output_path.parent.mkdir(parents=True, exist_ok=True)
    request.output_path.write_bytes(request.input_path.read_bytes())


def parse_args() -> ToolRequest:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    return ToolRequest(input_path=args.input.resolve(), output_path=args.output.resolve())


def main() -> int:
    try:
        execute(parse_args())
    except (OSError, ValueError) as error:
        print(f"ERROR: {error}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
