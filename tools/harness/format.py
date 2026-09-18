"""格式化仓库自有源码，或以只读方式验证格式一致性。"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _style import (
    apply_format,
    collect_format_issues,
    debt,
    debt_allows,
    managed_files,
    module_names,
    partition_debt,
    relative,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="report differences without editing")
    parser.add_argument("--all", action="store_true", help="also migrate unchanged legacy debt")
    parser.add_argument("--module", choices=module_names())
    parser.add_argument("paths", nargs="*", type=Path)
    args = parser.parse_args()

    if args.paths:
        paths = [path.resolve() for path in args.paths]
    else:
        paths = managed_files(args.module)

    if args.check:
        new, allowed = partition_debt(collect_format_issues(paths))
        for issue in new:
            print(f"{relative(issue['path'])}:{issue['line']}: {issue['message']}")
            print(f"  Fix: uv run python tools/run.py format {relative(issue['path'])}")
        print(
            f"format: {'PASS' if not new else 'FAIL'} "
            f"({len(new)} new problems, {len(allowed)} unchanged legacy files/checks allowed)"
        )
        return int(bool(new))

    baseline = debt()
    if not args.paths and not args.all:
        format_checks = ("python-format", "c-format")
        paths = [
            path
            for path in paths
            if not any(debt_allows(path, check, baseline) for check in format_checks)
        ]
    apply_format(paths)
    new, _ = partition_debt(collect_format_issues(paths))
    print(f"format: {'PASS' if not new else 'FAIL'} ({len(paths)} files processed)")
    return int(bool(new))


if __name__ == "__main__":
    raise SystemExit(main())
