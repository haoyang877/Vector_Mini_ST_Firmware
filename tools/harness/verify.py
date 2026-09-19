"""运行指定验证级别并保存可追踪的机器可读证据。"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import (
    ROOT,
    command_output,
    config,
    configured_path,
    file_version,
    git_info,
    make_run_id,
    sha256,
    write_json,
    zig_path,
)
from doctor import collect as doctor_collect


def run_check(name: str, command: list[str], run_dir: Path) -> dict:
    started = time.monotonic()
    result = subprocess.run(command, cwd=ROOT, capture_output=True, text=True, check=False)
    output = (result.stdout or "") + (result.stderr or "")
    log = run_dir / f"{name}.log"
    log.write_text(output, encoding="utf-8", errors="replace")
    entry = {
        "name": name,
        "status": "PASS" if result.returncode == 0 else "FAIL",
        "exit_code": result.returncode,
        "duration_seconds": round(time.monotonic() - started, 3),
        "log": log.relative_to(ROOT).as_posix(),
    }
    if result.returncode:
        entry["remediation"] = (
            f"Inspect {entry['log']} and rerun this check after correcting the first error."
        )
    return entry


def record_check(
    run_dir: Path,
    name: str,
    status: str,
    message: str,
    remediation: str | None = None,
    details: dict | None = None,
) -> dict:
    log = run_dir / f"{name}.log"
    log.write_text(message.rstrip() + "\n", encoding="utf-8")
    result = {
        "name": name,
        "status": status,
        "exit_code": 0 if status == "PASS" else 1,
        "duration_seconds": 0.0,
        "log": log.relative_to(ROOT).as_posix(),
    }
    if remediation:
        result["remediation"] = remediation
    if details is not None:
        result["details"] = details
    return result


def firmware_artifacts() -> list[dict]:
    artifacts = []
    build_root = ROOT / "outputs/build/keil"
    for target in ("Vector_Mini_ST",):
        directory = build_root / target
        target_files = []
        for suffix in (".axf", ".hex", ".map"):
            candidates = sorted(directory.glob(f"*{suffix}")) if directory.exists() else []
            if candidates:
                path = candidates[0]
                target_files.append(
                    {
                        "path": path.relative_to(ROOT).as_posix(),
                        "bytes": path.stat().st_size,
                        "sha256": sha256(path),
                    }
                )
        log = ROOT / "outputs/build/logs" / f"{target}.log"
        errors = warnings = None
        if log.is_file():
            text = log.read_text(encoding="utf-8", errors="replace")
            match = re.search(r"(\d+) Error\(s\), (\d+) Warning\(s\)", text)
            if match:
                errors, warnings = map(int, match.groups())
        artifacts.append(
            {"target": target, "errors": errors, "warnings": warnings, "files": target_files}
        )
    return artifacts


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", choices=("quick", "pr", "release"), default="quick")
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--run-id")
    parser.add_argument("--reference-source", type=Path)
    args = parser.parse_args()

    run_id = args.run_id or make_run_id()
    run_dir = ROOT / "outputs/runs" / run_id
    run_dir.mkdir(parents=True, exist_ok=False)
    started_at = datetime.now(timezone.utc).isoformat()
    cfg = config()
    doctor = doctor_collect(args.profile)
    checks = [
        record_check(
            run_dir,
            "doctor",
            doctor["status"],
            json.dumps(doctor, indent=2),
            "Run doctor for this profile and follow each reported fix."
            if doctor["status"] != "PASS"
            else None,
            doctor,
        )
    ]
    initial_git = git_info()
    if args.profile == "release" and initial_git["dirty"]:
        checks.append(
            record_check(
                run_dir,
                "clean-release-source",
                "FAIL",
                "Release source has tracked or untracked changes:\n"
                + "\n".join(initial_git["status"]),
                "Commit or remove source changes before producing release evidence.",
            )
        )

    if doctor["status"] == "PASS":
        checks.append(
            run_check(
                "format", [sys.executable, str(ROOT / "tools/run.py"), "format", "--check"], run_dir
            )
        )
        checks.append(
            run_check("lint", [sys.executable, str(ROOT / "tools/run.py"), "lint"], run_dir)
        )
        checks.append(
            run_check(
                "project-layout",
                [sys.executable, str(ROOT / "tools/build/check_project_layout.py")],
                run_dir,
            )
        )
        test_command = [sys.executable, str(ROOT / "tests/run.py"), "--out", str(run_dir / "tests")]
        if args.profile in ("pr", "release"):
            test_command += ["--cc", str(zig_path())]
        if args.reference_source:
            test_command += ["--reference-source", str(args.reference_source.resolve())]
        checks.append(run_check("tests", test_command, run_dir))
        if args.profile in ("pr", "release"):
            for name in (
                "check_architecture",
                "check_interfaces",
                "check_docs",
                "check_hygiene",
            ):
                checks.append(
                    run_check(
                        name.replace("check_", ""),
                        [sys.executable, str(ROOT / "tools/harness" / f"{name}.py")],
                        run_dir,
                    )
                )
        if args.profile == "release" and all(entry["status"] == "PASS" for entry in checks):
            keil = configured_path("keil")
            checks.append(
                run_check(
                    "keil-build",
                    [
                        sys.executable,
                        str(ROOT / "tools/build/build_firmware.py"),
                        "--uv4",
                        str(keil),
                        "--target",
                        "all",
                    ],
                    run_dir,
                )
            )

    artifacts = firmware_artifacts() if args.profile == "release" else []
    fresh_build = any(
        entry["name"] == "keil-build" and entry["status"] == "PASS" for entry in checks
    )
    release_ok = (
        (
            fresh_build
            and all(
                target["errors"] == 0 and target["warnings"] == 0 and len(target["files"]) == 3
                for target in artifacts
            )
        )
        if args.profile == "release" and artifacts
        else args.profile != "release"
    )
    if args.profile == "release":
        checks.append(
            record_check(
                run_dir,
                "release-manifest",
                "PASS" if release_ok else "FAIL",
                json.dumps(artifacts, indent=2),
                "Rebuild the Keil target in this run; require AXF/HEX/MAP and 0 errors/0 warnings."
                if not release_ok
                else None,
            )
        )

    status = "PASS" if all(entry["status"] == "PASS" for entry in checks) else "FAIL"
    compiler = zig_path()
    keil = configured_path("keil") if args.profile == "release" else None
    summary = {
        "schema_version": 1,
        "run_id": run_id,
        "profile": args.profile,
        "status": status,
        "started_at": started_at,
        "finished_at": datetime.now(timezone.utc).isoformat(),
        "hardware_contacted": False,
        "git": git_info(),
        "tools": {
            "python": sys.version.split()[0],
            "zig": {
                "required": cfg["zig_version"],
                "path": str(compiler) if compiler else None,
                "version": command_output([str(compiler), "version"]) if compiler else None,
            },
            "keil": {"path": str(keil) if keil else None, "version": file_version(keil)},
        },
        "checks": checks,
        "firmware_artifacts": artifacts,
        "reference_source_supplied": bool(args.reference_source),
    }
    write_json(run_dir / "summary.json", summary)
    if args.profile == "release":
        write_json(
            run_dir / "release-manifest.json",
            {
                "schema_version": 1,
                "run_id": run_id,
                "git": summary["git"],
                "status": status,
                "firmware_artifacts": artifacts,
            },
        )
    if args.json:
        print(json.dumps(summary, indent=2, ensure_ascii=False))
    else:
        for entry in checks:
            print(f"{entry['name']}: {entry['status']}")
            if "remediation" in entry:
                print(f"  Fix: {entry['remediation']}")
        print(f"verify {args.profile}: {status}")
        print(f"Evidence: {(run_dir / 'summary.json').relative_to(ROOT)}")
    return 0 if status == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
