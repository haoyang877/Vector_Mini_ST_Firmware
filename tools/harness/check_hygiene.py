"""检查仓库可移植性以及硬件工具不可绕过的安全不变量。"""

from __future__ import annotations

import argparse
import ast
import json
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import ROOT

ABSOLUTE = re.compile(r"(?<![A-Za-z0-9_])[A-Za-z]:[\\/]")
SERIAL_OPEN = re.compile(r"\.open\(\s*\d{6,}\s*\)")


def problems() -> list[dict]:
    issues = []
    for root_name in ("tools", "tests"):
        for path in (ROOT / root_name).rglob("*"):
            if path.suffix.lower() not in (".py", ".toml", ".yaml", ".yml", ".md"):
                continue
            text = path.read_text(encoding="utf-8", errors="replace")
            for number, line in enumerate(text.splitlines(), 1):
                if path.name != "check_docs.py" and ABSOLUTE.search(line):
                    issues.append(
                        {
                            "file": path.relative_to(ROOT).as_posix(),
                            "line": number,
                            "message": "tracked absolute workstation path",
                        }
                    )
                if path.suffix == ".py" and SERIAL_OPEN.search(line):
                    issues.append(
                        {
                            "file": path.relative_to(ROOT).as_posix(),
                            "line": number,
                            "message": "hard-coded hardware serial",
                        }
                    )
                if (
                    path.suffix == ".py"
                    and path.name != "check_hygiene.py"
                    and "sys.path.insert" in line
                    and "outputs" in line
                ):
                    issues.append(
                        {
                            "file": path.relative_to(ROOT).as_posix(),
                            "line": number,
                            "message": "runtime dependency loaded from historical outputs",
                        }
                    )
    safety_files = [
        ROOT / "tools/flash/servo_hil_flash.py",
        ROOT / "tools/bench/servo_hil_run.py",
        ROOT / "tools/bench/servo_hil_emergency.py",
    ]
    for path in safety_files:
        text = path.read_text(encoding="utf-8", errors="replace")
        for number, line in enumerate(text.splitlines(), 1):
            if re.match(r"^\s*assert\b", line):
                issues.append(
                    {
                        "file": path.relative_to(ROOT).as_posix(),
                        "line": number,
                        "message": "hardware safety check uses optimizable assert",
                    }
                )
    commands = {}
    for directory in (
        "tools/harness",
        "tools/build",
        "tools/flash",
        "tools/bench",
        "tools/analysis",
        "tests/unit/native",
    ):
        for path in (ROOT / directory).glob("*.py"):
            if path.name.startswith("_") or path.stem == "common":
                continue
            commands.setdefault(path.stem, []).append(path.relative_to(ROOT).as_posix())
    for name, paths in commands.items():
        if len(paths) > 1:
            issues.append(
                {
                    "file": ", ".join(paths),
                    "line": 1,
                    "message": f"duplicate tool command name: {name}",
                }
            )
    run_source = (ROOT / "tests/run.py").read_text(encoding="utf-8")
    tree = ast.parse(run_source)
    native = set()
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
            isinstance(target, ast.Name) and target.id == "NATIVE" for target in node.targets
        ):
            native = set(ast.literal_eval(node.value))
    scripts = {
        path.stem
        for path in (ROOT / "tests/unit/native").glob("*.py")
        if not path.name.startswith("_")
    }
    optional = {"test_position_core_equivalence"}
    if scripts != native | optional:
        issues.append(
            {
                "file": "tests/run.py",
                "line": 1,
                "message": f"native test inventory drift: scripts={sorted(scripts)}, configured={sorted(native | optional)}",
            }
        )
    return issues


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    issues = problems()
    result = {"schema_version": 1, "status": "PASS" if not issues else "FAIL", "problems": issues}
    if args.json:
        print(json.dumps(result, indent=2))
    else:
        for issue in issues:
            print(f"{issue['file']}:{issue['line']}: {issue['message']}")
            print("  Fix: use environment/CLI configuration and explicit RuntimeError checks.")
        print(f"hygiene: {result['status']} ({len(issues)} problems)")
    return 0 if not issues else 1


if __name__ == "__main__":
    raise SystemExit(main())
