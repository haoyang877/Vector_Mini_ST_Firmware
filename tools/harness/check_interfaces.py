"""检查公共 C/C++ 接口是否具备完整且可维护的中文契约。"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _style import managed_files, relative
from common import ROOT

DEBT = ROOT / "tools/harness/interface_debt.json"
CHINESE = re.compile(r"[\u3400-\u4dbf\u4e00-\u9fff]")
DECLARATION = re.compile(r"^(?P<prefix>.+?)\b(?P<name>[A-Za-z_]\w*)\s*\((?P<params>.*)\)\s*;$")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _split_parameters(text: str) -> list[str]:
    values = []
    start = 0
    depth = 0
    for index, character in enumerate(text):
        if character in "([":
            depth += 1
        elif character in ")]":
            depth = max(0, depth - 1)
        elif character == "," and depth == 0:
            values.append(text[start:index])
            start = index + 1
    values.append(text[start:])
    return [value.strip() for value in values if value.strip() and value.strip() != "void"]


def _parameter_names(text: str) -> tuple[list[str], bool]:
    names = []
    unnamed = False
    for parameter in _split_parameters(text):
        clean = re.sub(r"\[[^]]*\]", "", parameter)
        identifiers = re.findall(r"[A-Za-z_]\w*", clean)
        if not identifiers:
            unnamed = True
            continue
        candidate = identifiers[-1]
        if candidate in {"const", "volatile", "struct", "enum", "union", "unsigned", "signed"}:
            unnamed = True
        else:
            names.append(candidate)
    return names, unnamed


def _preceding_contract(source: str, start: int) -> str | None:
    prefix = source[:start].rstrip()
    if not prefix.endswith("*/"):
        return None
    begin = prefix.rfind("/**")
    if begin < 0:
        return None
    contract = prefix[begin:]
    return contract if "*/" in contract else None


def _without_comments(source: str) -> str:
    """移除注释内容但保留换行，使声明行号仍能映射到原文件。"""
    pattern = re.compile(r"//[^\n]*|/\*.*?\*/", flags=re.DOTALL)

    def replace(match: re.Match) -> str:
        return "\n" * match.group(0).count("\n")

    return pattern.sub(replace, source)


def _prototypes(source: str) -> list[dict]:
    """按分号结束的顶层声明扫描，避免在大型头文件上发生正则回溯。"""
    clean_lines = _without_comments(source).splitlines(keepends=True)
    original_lines = source.splitlines(keepends=True)
    declarations = []
    current = []
    start_line = 0
    start_offset = 0
    offset = 0
    brace_depth = 0
    for number, line in enumerate(clean_lines, 1):
        original_line = original_lines[number - 1]
        stripped = line.strip()
        depth_before = brace_depth
        brace_depth += line.count("{") - line.count("}")
        if not current:
            if (
                depth_before != 0
                or "(" not in stripped
                or stripped.startswith(("#", "typedef", "static inline"))
                or "{" in stripped
                or "(*" in stripped
            ):
                offset += len(original_line)
                continue
            current = [stripped]
            start_line = number
            start_offset = offset + len(original_line) - len(original_line.lstrip())
        else:
            current.append(stripped)
        offset += len(original_line)
        declaration = " ".join(current)
        if ";" not in declaration:
            continue
        current = []
        if declaration.count("(") != declaration.count(")") or "{" in declaration:
            continue
        match = DECLARATION.match(declaration)
        if match:
            declarations.append(
                {
                    "prefix": match.group("prefix"),
                    "name": match.group("name"),
                    "params": match.group("params"),
                    "line": start_line,
                    "offset": start_offset,
                }
            )
    return declarations


def scan(paths: list[Path] | None = None) -> list[dict]:
    headers = paths or [
        path for path in managed_files() if path.suffix.lower() in {".h", ".hh", ".hpp"}
    ]
    violations = []
    for path in headers:
        source = path.read_text(encoding="utf-8-sig", errors="replace")
        for declaration in _prototypes(source):
            name = declaration["name"]
            line = declaration["line"]
            contract = _preceding_contract(source, declaration["offset"])
            parameters, unnamed = _parameter_names(declaration["params"])
            return_type = " ".join(declaration["prefix"].split())
            missing = []
            if contract is None:
                missing.append("紧邻声明的 /** ... */ 契约")
                contract = ""
            if "@brief" not in contract or not CHINESE.search(contract):
                missing.append("中文 @brief")
            if unnamed:
                missing.append("有意义的参数名")
            for parameter in parameters:
                if not re.search(rf"@param(?:\[[^]]+\])?\s+{re.escape(parameter)}\b", contract):
                    missing.append(f"@param {parameter}")
            if return_type != "void" and "@return" not in contract:
                missing.append("@return")
            if missing:
                violations.append(
                    {
                        "id": f"{relative(path)}::{name}",
                        "file": relative(path),
                        "line": line,
                        "interface": name,
                        "missing": missing,
                        "sha256": sha256(path),
                    }
                )
    return violations


def _baseline() -> dict:
    if not DEBT.is_file():
        return {"schema_version": 1, "files": {}}
    return json.loads(DEBT.read_text(encoding="utf-8"))


def evaluate(paths: list[Path] | None = None) -> tuple[list[dict], list[dict], list[str]]:
    actual = scan(paths)
    baseline = _baseline()["files"]
    new = []
    current_ids = {entry["id"] for entry in actual}
    for entry in actual:
        debt = baseline.get(entry["file"])
        if (
            not debt
            or debt["sha256"] != entry["sha256"]
            or entry["interface"] not in debt["interfaces"]
        ):
            new.append(entry)
    resolved = [
        f"{file_name}::{interface}"
        for file_name, debt in baseline.items()
        for interface in debt["interfaces"]
        if f"{file_name}::{interface}" not in current_ids
    ]
    return actual, new, resolved


def tracked_baseline() -> dict:
    tracked = {
        line.strip().replace("\\", "/")
        for line in subprocess.run(
            ["git", "ls-files"], cwd=ROOT, capture_output=True, text=True, check=True
        ).stdout.splitlines()
    }
    grouped: dict[str, dict] = {}
    for entry in scan():
        if entry["file"] not in tracked:
            continue
        value = grouped.setdefault(entry["file"], {"sha256": entry["sha256"], "interfaces": []})
        value["interfaces"].append(entry["interface"])
    for value in grouped.values():
        value["interfaces"].sort()
    return {"schema_version": 1, "files": dict(sorted(grouped.items()))}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--print-baseline", action="store_true")
    args = parser.parse_args()
    if args.print_baseline:
        print(json.dumps(tracked_baseline(), indent=2, ensure_ascii=False))
        return 0
    actual, new, resolved = evaluate()
    result = {
        "schema_version": 1,
        "status": "PASS" if not new else "FAIL",
        "current_debt": len(actual),
        "new_violations": new,
        "resolved_debt": resolved,
    }
    if args.json:
        print(json.dumps(result, indent=2, ensure_ascii=False))
    else:
        for entry in new:
            print(f"{entry['file']}:{entry['line']}: {entry['interface']} 接口契约不完整")
            print(f"  缺少: {', '.join(entry['missing'])}")
            print("  修复: 按 STYLE.md 补充中文 Doxygen 契约，不要只复述函数名。")
        for entry in resolved:
            print(f"RESOLVED: {entry}（可从 interface_debt.json 删除）")
        print(
            f"interfaces: {result['status']} "
            f"({len(actual)} known, {len(new)} new, {len(resolved)} resolved)"
        )
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
