"""执行可读性检查，并用内容哈希确保历史债务只能减少。"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _style import (
    collect_lint_issues,
    collect_style_config_issues,
    managed_files,
    module_names,
    partition_debt,
    relative,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--module", choices=module_names())
    args = parser.parse_args()
    issues = collect_lint_issues(managed_files(args.module))
    issues.extend(collect_style_config_issues(args.module))
    new, allowed = partition_debt(issues)
    for issue in new:
        print(f"{relative(issue['path'])}:{issue['line']}: {issue['message']}")
        print("  Fix: follow STYLE.md, then rerun format and lint.")
    print(
        f"lint: {'PASS' if not new else 'FAIL'} "
        f"({len(new)} new problems, {len(allowed)} unchanged legacy findings allowed)"
    )
    return int(bool(new))


if __name__ == "__main__":
    raise SystemExit(main())
