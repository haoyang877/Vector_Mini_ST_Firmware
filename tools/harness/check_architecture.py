"""拒绝超出分层依赖图的新固件 include，历史债务只允许减少。"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import ROOT, config

DEBT = ROOT / "tools/harness/architecture_debt.json"
INCLUDE = re.compile(r'^\s*#\s*include\s+"([^"]+)"', re.MULTILINE)


def layer_config():
    layers = config()["architecture"]["layers"]
    roots = []
    for name, definition in layers.items():
        for value in definition["paths"]:
            roots.append((ROOT / value, name))
    roots.sort(key=lambda pair: len(pair[0].parts), reverse=True)
    return layers, roots


def owner(path: Path, roots) -> str | None:
    path = path.resolve()
    for root, name in roots:
        try:
            path.relative_to(root.resolve())
            return name
        except ValueError:
            pass
    return None


def scan() -> list[dict]:
    layers, roots = layer_config()
    headers: dict[str, list[Path]] = {}
    sources = []
    for root, _ in roots:
        if not root.exists():
            continue
        for path in root.rglob("*"):
            if path.suffix.lower() in (".c", ".h"):
                sources.append(path)
                if path.suffix.lower() == ".h":
                    headers.setdefault(path.name, []).append(path)
    violations = []
    for source in sorted(set(sources)):
        source_layer = owner(source, roots)
        if source_layer is None:
            continue
        text = source.read_text(encoding="utf-8-sig", errors="replace")
        for include in INCLUDE.findall(text):
            candidates = []
            local = (source.parent / include).resolve()
            if local.is_file():
                candidates = [local]
            elif Path(include).name in headers:
                candidates = headers[Path(include).name]
            targets = {(candidate, owner(candidate, roots)) for candidate in candidates}
            targets = {(candidate, layer) for candidate, layer in targets if layer}
            if len({layer for _, layer in targets}) != 1:
                continue
            target, target_layer = sorted(targets, key=lambda pair: str(pair[0]))[0]
            if target_layer in layers[source_layer]["allows"]:
                continue
            source_rel = source.relative_to(ROOT).as_posix()
            target_rel = target.relative_to(ROOT).as_posix()
            violation_id = f"{source_rel} -> {target_rel}"
            violations.append(
                {
                    "id": violation_id,
                    "from_layer": source_layer,
                    "to_layer": target_layer,
                    "include": include,
                }
            )
    return sorted(
        {value["id"]: value for value in violations}.values(), key=lambda value: value["id"]
    )


def evaluate() -> tuple[list[dict], list[dict], list[str]]:
    actual = scan()
    baseline = json.loads(DEBT.read_text(encoding="utf-8")) if DEBT.exists() else {"violations": []}
    known = {
        entry if isinstance(entry, str) else entry["id"]: entry for entry in baseline["violations"]
    }
    current = {entry["id"]: entry for entry in actual}
    new = [entry for key, entry in current.items() if key not in known]
    resolved = [key for key in known if key not in current]
    return actual, new, resolved


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", action="store_true")
    parser.add_argument(
        "--print-baseline",
        action="store_true",
        help="Print current violations as baseline JSON; never writes files",
    )
    args = parser.parse_args()
    actual, new, resolved = evaluate()
    if args.print_baseline:
        print(
            json.dumps(
                {
                    "schema_version": 1,
                    "reason": "Existing reverse dependencies; remove entries as boundaries are repaired.",
                    "violations": [entry["id"] for entry in actual],
                },
                indent=2,
            )
        )
        return 0
    result = {
        "schema_version": 1,
        "status": "PASS" if not new else "FAIL",
        "current_debt": len(actual),
        "new_violations": new,
        "resolved_debt": resolved,
    }
    if args.json:
        print(json.dumps(result, indent=2))
    else:
        for entry in new:
            print(f"NEW: {entry['id']} ({entry['from_layer']} -> {entry['to_layer']})")
            print(
                "  Fix: depend through the documented forward layer or update code to remove the reverse include."
            )
        for entry in resolved:
            print(f"RESOLVED: {entry} (remove it from architecture_debt.json)")
        print(
            f"architecture: {result['status']} ({len(actual)} known, {len(new)} new, {len(resolved)} resolved)"
        )
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
