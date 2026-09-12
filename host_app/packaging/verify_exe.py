"""Acceptance test for the packaged executables (GUI + CLI).

Runs each exe's self-test / smoke commands and reports PASS/FAIL:
  GUI:  --selftest (offline, screenshots)  and  --selftest-live (hardware)
  CLI:  --help, scan (hardware), read 0x67 (hardware)

Usage:  python packaging/verify_exe.py [--skip-live]
Exit code 0 only when every executed check passes.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]  # host_app/..  -> see below
ROOT = Path(__file__).resolve().parents[1]  # host_app/
DIST = ROOT / "dist"
GUI_EXE = DIST / "mdrive-host.exe"
CLI_EXE = DIST / "mdrive-cli.exe"


def run(label: str, args: list[str], expect_stdout: str | None = None) -> bool:
    try:
        result = subprocess.run(args, capture_output=True, text=True, timeout=180)
    except subprocess.TimeoutExpired:
        print(f"FAIL {label}: timeout")
        return False
    combined = (result.stdout or "") + (result.stderr or "")
    if result.returncode != 0:
        print(f"FAIL {label}: exit {result.returncode}\n  {combined.strip()[:400]}")
        return False
    if expect_stdout and expect_stdout not in combined:
        print(f"FAIL {label}: missing marker {expect_stdout!r}\n  {combined.strip()[:400]}")
        return False
    print(f"PASS {label}")
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--skip-live", action="store_true", help="skip checks that need hardware")
    args = parser.parse_args()

    if not GUI_EXE.exists() or not CLI_EXE.exists():
        print(f"missing exe: build first (pyinstaller). Looked for {GUI_EXE} / {CLI_EXE}")
        return 2

    checks: list[tuple[str, list[str], str | None]] = []
    with tempfile.TemporaryDirectory() as tmp:
        shot = Path(tmp) / "selftest.png"
        checks.append(("GUI --selftest", [str(GUI_EXE), "--selftest", "--screenshot", str(shot)], "SELFTEST OK"))
        checks.append(("CLI --help", [str(CLI_EXE), "--help"], "flashinfo"))
        if not args.skip_live:
            checks.append(("GUI --selftest-live", [str(GUI_EXE), "--selftest-live"], "SELFTEST-LIVE OK"))
            checks.append(("CLI scan", [str(CLI_EXE), "scan"], "0"))
            checks.append(("CLI read 0x67", [str(CLI_EXE), "read", "0x67"], "= 2"))

        passed = 0
        for label, command, marker in checks:
            if run(label, command, marker):
                passed += 1
        total = len(checks)
        print(f"\n{passed}/{total} exe checks passed")
        return 0 if passed == total else 1


if __name__ == "__main__":
    raise SystemExit(main())
