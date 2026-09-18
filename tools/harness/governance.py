"""执行只读的架构、文档和仓库卫生治理检查。"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import ROOT, git_info, write_json


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=ROOT / "outputs/governance/latest.json")
    args = parser.parse_args()
    checks = []
    for name in ("check_architecture", "check_interfaces", "check_docs", "check_hygiene"):
        result = subprocess.run(
            [sys.executable, str(ROOT / "tools/harness" / f"{name}.py"), "--json"],
            cwd=ROOT,
            capture_output=True,
            text=True,
            check=False,
        )
        try:
            details = json.loads(result.stdout)
        except json.JSONDecodeError:
            details = {"status": "FAIL", "raw_output": result.stdout + result.stderr}
        checks.append(
            {
                "name": name.removeprefix("check_"),
                "status": details.get("status", "FAIL"),
                "details": details,
            }
        )
    report = {
        "schema_version": 1,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "hardware_contacted": False,
        "git": git_info(),
        "checks": checks,
        "status": "PASS" if all(check["status"] == "PASS" for check in checks) else "FAIL",
    }
    out = args.out if args.out.is_absolute() else ROOT / args.out
    write_json(out, report)
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
