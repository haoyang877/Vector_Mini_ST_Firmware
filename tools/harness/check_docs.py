"""检查文档入口、索引覆盖范围和本地 Markdown 链接。"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path
from urllib.parse import unquote

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import ROOT

LINK = re.compile(r"!?\[[^\]]*\]\((?:<([^>]+)>|([^\s)]+))(?:\s+['\"][^'\"]*['\"])?\)")
REQUIRED_INDEX_LINKS = (
    "architecture/",
    "protocols/",
    "guides/",
    "hardware/",
    "analysis/",
    "reports/",
    "releases/",
    "plans/",
)
STALE_CURRENT_PATTERNS = {
    "tools/servo_hil_run.py": "use the repository path tools/bench/servo_hil_run.py",
    "J-Link序号602722271": "document probe selection through JLINK_PROBE_SERIAL instead of a fixed serial",
    "C:/path/to/zig.exe": "use the locked uv/Zig environment",
}


def problems() -> list[dict]:
    issues = []
    agents = ROOT / "AGENTS.md"
    if not agents.is_file():
        issues.append({"file": "AGENTS.md", "message": "missing repository agent map"})
    elif len(agents.read_text(encoding="utf-8").splitlines()) > 100:
        issues.append(
            {
                "file": "AGENTS.md",
                "message": "must stay at or below 100 lines; move detail into docs",
            }
        )
    index = ROOT / "docs/README.md"
    index_text = index.read_text(encoding="utf-8") if index.is_file() else ""
    for value in REQUIRED_INDEX_LINKS:
        if f"]({value})" not in index_text:
            issues.append(
                {"file": "docs/README.md", "message": f"missing documentation area link: {value}"}
            )
    markdown = [
        ROOT / "README.md",
        ROOT / "ARCHITECTURE.md",
        ROOT / "AGENTS.md",
        *sorted((ROOT / "docs").rglob("*.md")),
        *sorted((ROOT / "tests").glob("*.md")),
        *sorted((ROOT / "loader").glob("*.md")),
        *sorted((ROOT / "host_app").glob("*.md")),
        *sorted((ROOT / "shared").glob("*.md")),
    ]
    for path in markdown:
        if not path.is_file():
            continue
        text = path.read_text(encoding="utf-8", errors="replace")
        for match in LINK.finditer(text):
            value = unquote(match.group(1) or match.group(2) or "")
            if not value or value.startswith(("http://", "https://", "mailto:", "#")):
                continue
            target_text = value.split("#", 1)[0]
            if not target_text:
                continue
            target = (path.parent / target_text).resolve()
            if not target.exists():
                issues.append(
                    {
                        "file": path.relative_to(ROOT).as_posix(),
                        "message": f"broken local link: {value}",
                    }
                )
        relative = path.relative_to(ROOT).as_posix()
        is_current = relative in (
            "README.md",
            "ARCHITECTURE.md",
            "AGENTS.md",
            "tests/README.md",
        ) or relative.startswith(("docs/guides/", "docs/architecture/", "docs/plans/"))
        if is_current:
            for stale, remediation in STALE_CURRENT_PATTERNS.items():
                if stale in text:
                    issues.append(
                        {
                            "file": relative,
                            "message": f"stale current guidance: {stale}; {remediation}",
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
        print(json.dumps(result, indent=2, ensure_ascii=False))
    else:
        for issue in issues:
            print(f"{issue['file']}: {issue['message']}")
            print("  Fix: repair the link or add the durable document to the nearest index.")
        print(f"docs: {result['status']} ({len(issues)} problems)")
    return 0 if not issues else 1


if __name__ == "__main__":
    raise SystemExit(main())
