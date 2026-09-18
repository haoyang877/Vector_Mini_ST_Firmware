"""检查 harness 环境依赖，全程不得连接任何硬件。"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import (
    command_output,
    config,
    configured_path,
    env_name,
    file_version,
    module_available,
    zig_path,
)


def item(name: str, ok: bool, found=None, required=None, fix=None) -> dict:
    result = {"name": name, "status": "PASS" if ok else "FAIL"}
    if found is not None:
        result["found"] = str(found)
    if required is not None:
        result["required"] = str(required)
    if not ok and fix:
        result["remediation"] = fix
    return result


def collect(profile: str) -> dict:
    cfg = config()
    checks = []
    required_python = cfg["python_version"]
    actual_python = f"{sys.version_info.major}.{sys.version_info.minor}"
    checks.append(
        item(
            "python",
            actual_python == required_python,
            actual_python,
            required_python,
            "Install the pinned Python version, then run uv sync --locked --extra dev.",
        )
    )

    if profile in ("quick", "pr", "release"):
        checks.append(
            item(
                "numpy",
                module_available("numpy"),
                "installed" if module_available("numpy") else "missing",
                "installed",
                "Run uv sync --locked --extra dev.",
            )
        )
        ruff_version = command_output([sys.executable, "-m", "ruff", "--version"])
        required_ruff = f"ruff {cfg['ruff_version']}"
        checks.append(
            item(
                "ruff",
                ruff_version == required_ruff,
                ruff_version or "missing",
                required_ruff,
                "Run uv sync --locked --extra dev.",
            )
        )
        adjacent_clang = Path(sys.executable).resolve().parent / (
            "clang-format.exe" if os.name == "nt" else "clang-format"
        )
        clang_format = shutil.which("clang-format")
        if clang_format is None and adjacent_clang.is_file():
            clang_format = str(adjacent_clang)
        clang_version = command_output([clang_format, "--version"]) if clang_format else None
        required_clang = cfg["clang_format_version"]
        checks.append(
            item(
                "clang-format",
                bool(clang_version and required_clang in clang_version),
                clang_version or "missing",
                required_clang,
                "Run uv sync --locked --extra dev.",
            )
        )

    if profile in ("pr", "release"):
        zig = zig_path()
        version = None
        if zig and zig.is_file():
            version = command_output([str(zig), "version"])
        checks.append(
            item(
                "zig",
                version == cfg["zig_version"],
                f"{version or 'missing'} ({zig or 'not found'})",
                cfg["zig_version"],
                "Run uv sync --locked --extra dev or set HARNESS_CC to the pinned Zig executable.",
            )
        )

    if profile == "release":
        keil = configured_path("keil")
        version = file_version(keil)
        checks.append(
            item(
                "keil",
                bool(keil and keil.is_file() and version == cfg["keil_version"]),
                f"{version or 'missing'} ({keil or 'not found'})",
                cfg["keil_version"],
                f"Set {env_name('keil')} to the supported licensed UV4.exe ({cfg['keil_version']}).",
            )
        )

    if profile == "hil":
        for module, distribution in (("elftools", "pyelftools"), ("pylink", "pylink-square")):
            checks.append(
                item(
                    distribution,
                    module_available(module),
                    "installed" if module_available(module) else "missing",
                    "installed",
                    "Run uv sync --locked --extra bench.",
                )
            )
        dll_value = os.environ.get(env_name("jlink_dll"))
        dll = Path(dll_value).expanduser().resolve() if dll_value else None
        version = file_version(dll)
        checks.append(
            item(
                "jlink-dll",
                bool(dll and dll.is_file() and version == cfg["jlink_version"]),
                f"{version or 'missing'} ({dll or 'not configured'})",
                cfg["jlink_version"],
                f"Set {env_name('jlink_dll')} to the supported JLink_x64.dll ({cfg['jlink_version']}).",
            )
        )
        for key, label in (("jlink_probe_serial", "probe-serial"), ("bench_id", "bench-id")):
            value = os.environ.get(env_name(key))
            checks.append(
                item(
                    label,
                    bool(value),
                    "configured" if value else "missing",
                    "configured",
                    f"Set {env_name(key)} in the local bench environment.",
                )
            )

    return {
        "schema_version": 1,
        "profile": profile,
        "hardware_contacted": False,
        "status": "PASS" if all(c["status"] == "PASS" for c in checks) else "FAIL",
        "checks": checks,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", choices=("quick", "pr", "release", "hil"), default="quick")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    result = collect(args.profile)
    if args.json:
        print(json.dumps(result, indent=2, ensure_ascii=False))
    else:
        for check in result["checks"]:
            suffix = f": {check.get('found', '')}"
            print(f"{check['name']}: {check['status']}{suffix}")
            if "remediation" in check:
                print(f"  Fix: {check['remediation']}")
        print(f"doctor {args.profile}: {result['status']} (no hardware contacted)")
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
