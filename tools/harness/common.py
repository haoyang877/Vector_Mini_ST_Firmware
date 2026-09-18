"""提供 project harness 共用且不会访问硬件的基础能力。"""

from __future__ import annotations

import hashlib
import importlib.util
import json
import os
import shutil
import subprocess
import sys
import tomllib
from datetime import datetime, timezone
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from project_paths import ROOT

CONFIG_PATH = ROOT / "harness.toml"


def config() -> dict:
    with CONFIG_PATH.open("rb") as stream:
        return tomllib.load(stream)


def env_name(key: str) -> str:
    return config()["environment"][key]


def configured_path(key: str) -> Path | None:
    value = os.environ.get(env_name(key))
    if value:
        return Path(value).expanduser().resolve()
    executable = shutil.which({"keil": "UV4.exe"}.get(key, key))
    return Path(executable).resolve() if executable else None


def zig_path() -> Path | None:
    value = os.environ.get(env_name("compiler"))
    if value:
        return Path(value).expanduser().resolve()
    executable = shutil.which("zig")
    if executable:
        return Path(executable).resolve()
    try:
        import ziglang
    except ImportError:
        return None
    candidate = Path(ziglang.__file__).resolve().parent / ("zig.exe" if os.name == "nt" else "zig")
    return candidate if candidate.is_file() else None


def module_available(name: str) -> bool:
    return importlib.util.find_spec(name) is not None


def command_output(command: list[str]) -> str | None:
    try:
        result = subprocess.run(
            command, cwd=ROOT, capture_output=True, text=True, timeout=10, check=False
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    text = (result.stdout or result.stderr).strip()
    return text.splitlines()[0] if text else None


def file_version(path: Path | None) -> str | None:
    if path is None or not path.is_file():
        return None
    if os.name != "nt":
        return None
    script = f"(Get-Item -LiteralPath '{str(path).replace("'", "''")}').VersionInfo.FileVersion"
    return command_output(["powershell", "-NoProfile", "-Command", script])


def git_info() -> dict:
    def git(*args: str) -> str:
        result = subprocess.run(
            ["git", *args], cwd=ROOT, capture_output=True, text=True, check=False
        )
        return result.stdout.strip() if result.returncode == 0 else "unknown"

    status_result = subprocess.run(
        ["git", "status", "--porcelain", "--untracked-files=normal"],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    status = status_result.stdout.rstrip("\r\n") if status_result.returncode == 0 else "unknown"
    return {
        "sha": git("rev-parse", "HEAD"),
        "branch": git("branch", "--show-current"),
        "dirty": bool(status),
        "status": status.splitlines(),
    }


def make_run_id() -> str:
    now = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    return f"{now}-{git_info()['sha'][:8]}"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def write_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
