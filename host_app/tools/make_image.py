"""One-click application image builder following docs/firmware/FIRMWARE_NAMING.md.

Steps: Keil build (UV4 CLI) -> fromelf --bin -> trim/pad (mdrive_core.image_prep)
-> canonical file name + sidecar manifest (CRC/version/git) ready for the GUI
upgrade page or `mdrive upgrade`.

Usage:
    python host_app/tools/make_image.py --variant product
    python host_app/tools/make_image.py --variant factory --version 1.2.3
    python host_app/tools/make_image.py --no-build
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

HOST_APP = Path(__file__).resolve().parents[1]
REPO = HOST_APP.parent
sys.path.insert(0, str(HOST_APP / "src"))

from mdrive_core.services.image_prep import ImagePrepError, prepare_image  # noqa: E402

DEFAULT_UV4 = Path(r"C:\Keil_v5\UV4\UV4.exe")
DEFAULT_FROMELF = Path(r"C:\Keil_v5\ARM\ARMCC\bin\fromelf.exe")
VARIANTS = ("product", "factory", "debug")
IMAGE_TYPES = ("app", "loader")


def slug(text: str) -> str:
    return re.sub(r"[^a-z0-9]+", "_", text.lower()).strip("_")


def git_info() -> dict:
    """Short hash, dirty flag, and monotonic commit count (empty-safe)."""

    def run(args: list[str]) -> str:
        try:
            result = subprocess.run(["git", *args], capture_output=True, text=True, cwd=REPO, timeout=30)
        except OSError:
            return ""
        return result.stdout.strip() if result.returncode == 0 else ""

    count_text = run(["rev-list", "--count", "HEAD"])
    return {
        "commit": run(["rev-parse", "--short", "HEAD"]) or "nogit",
        "dirty": bool(run(["status", "--porcelain"])),
        "count": int(count_text) if count_text.isdigit() else 0,
    }


def parse_version(text: str | None, commit_count: int) -> tuple[str, int]:
    """Return (semver string without leading v, version_u32) per the naming spec."""
    stamp = time.strftime("%Y%m%d.%H%M%S")
    if text:
        value = text[1:] if text.startswith("v") else text
        match = re.match(r"^(\d+)\.(\d+)\.(\d+)", value)
        if not match:
            raise ValueError(f"version must be SemVer (e.g. 1.2.3): {text}")
        major, minor, patch = (int(part) for part in match.groups())
        build = commit_count & 0xFF
        return value, (major << 24) | (minor << 16) | (patch << 8) | build
    build = commit_count & 0xFF
    return f"0.0.{commit_count}+{stamp}", (0 << 24) | (0 << 16) | (0 << 8) | build


def build(uv4: Path, project: Path, target: str) -> tuple[bool, str]:
    """Run a Keil batch build; returns (ok, log tail)."""
    log = Path(tempfile.gettempdir()) / f"make_image_build_{int(time.time())}.log"
    result = subprocess.run(
        [str(uv4), "-b", str(project), "-t", target, "-o", str(log)],
        capture_output=True,
        text=True,
        timeout=900,
    )
    tail = ""
    if log.exists():
        lines = log.read_text(encoding="utf-8", errors="replace").splitlines()
        tail = "\n".join(lines[-8:])
    # UV4: 0 = clean, 1 = warnings, >=2 = errors/fatal
    return result.returncode < 2, tail


def axf_path(project: Path) -> Path:
    """Resolve the target .axf from the project file's output settings."""
    tree = ET.parse(project)
    common = tree.getroot().find("Targets/Target/TargetOption/TargetCommonOption")
    out_dir = common.findtext("OutputDirectory", default="") if common is not None else ""
    out_name = common.findtext("OutputName", default="") if common is not None else ""
    if not out_name:
        out_name = project.stem
    return (project.parent / out_dir.strip("\\/") / f"{out_name}.axf").resolve()


def convert(fromelf: Path, axf: Path):
    """axf -> raw bin -> trimmed/aligned PreparedImage."""
    with tempfile.TemporaryDirectory() as tmp:
        raw = Path(tmp) / "raw.bin"
        result = subprocess.run(
            [str(fromelf), "--bin", f"--output={raw}", str(axf)],
            capture_output=True,
            text=True,
            timeout=300,
        )
        if result.returncode != 0 or not raw.exists():
            raise RuntimeError(f"fromelf failed ({result.returncode}): {result.stdout}{result.stderr}")
        data = raw.read_bytes()
    return prepare_image(data)


def canonical_name(image_type: str, variant: str, board: str, mcu: str, version: str, commit: str, crc32: int) -> str:
    if image_type == "loader":
        return f"loader_{board}_{mcu}_v{version}_{commit}_{crc32:08X}.bin"
    return f"app_{variant}_{board}_{mcu}_v{version}_{commit}_{crc32:08X}.bin"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project", type=Path, default=REPO / "MDK-ARM" / "Vector_Mini_ST.uvprojx")
    parser.add_argument("--target", default="Vector_Mini_ST")
    parser.add_argument("--image-type", choices=IMAGE_TYPES, default="app")
    parser.add_argument("--variant", choices=VARIANTS, default="product")
    parser.add_argument("--board", default=None, help="board slug (default: derived from --target)")
    parser.add_argument("--mcu", default="stm32g4")
    parser.add_argument("--version", default=None, help="release SemVer (default: dev 0.0.<count>+<stamp>)")
    parser.add_argument("--uv4", type=Path, default=DEFAULT_UV4)
    parser.add_argument("--fromelf", type=Path, default=DEFAULT_FROMELF)
    parser.add_argument("--out", type=Path, default=None, help="exact output path (default: canonical name)")
    parser.add_argument("--no-build", action="store_true", help="skip the Keil build")
    args = parser.parse_args()

    if not args.project.exists():
        print(f"error: project not found: {args.project}")
        return 2
    if not args.fromelf.exists():
        print(f"error: fromelf not found: {args.fromelf}")
        return 2

    if not args.no_build:
        if not args.uv4.exists():
            print(f"error: UV4 not found: {args.uv4}")
            return 2
        ok, tail = build(args.uv4, args.project, args.target)
        if not ok:
            print("error: Keil build failed\n" + tail)
            return 1
        print(f"build OK ({args.target})")
        if tail:
            print("\n".join("  " + line for line in tail.splitlines()[-3:]))

    axf = axf_path(args.project)
    if not axf.exists():
        print(f"error: axf not found: {axf}")
        return 1

    git = git_info()
    try:
        version, version_u32 = parse_version(args.version, git["count"])
    except ValueError as exc:
        print(f"error: {exc}")
        return 2

    try:
        prepared = convert(args.fromelf, axf)
    except (RuntimeError, ImagePrepError) as exc:
        print(f"error: {exc}")
        return 3

    board = args.board or slug(args.target)
    name = canonical_name(args.image_type, args.variant, board, args.mcu, version, git["commit"], prepared.crc32)
    out = args.out or (REPO / "outputs" / "firmware_images" / name)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_bytes(prepared.data)

    manifest = {
        "schema": 1,
        "image_type": args.image_type,
        "variant": args.variant if args.image_type == "app" else None,
        "board": board,
        "mcu": args.mcu,
        "version": version,
        "version_u32": f"0x{version_u32:08X}",
        "git_commit": git["commit"],
        "git_dirty": git["dirty"],
        "build_time_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "image_size": prepared.size,
        "crc32": f"0x{prepared.crc32:08X}",
        "toolchain": "ARMCC 5.06u7",
        "source": {
            "project": str(args.project.relative_to(REPO)) if args.project.is_relative_to(REPO) else str(args.project),
            "target": args.target,
        },
        "notes": "dirty worktree - internal debug only" if git["dirty"] else "",
    }
    manifest_path = out.with_suffix(".json")
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    summary = {
        "image": str(out),
        "manifest": str(manifest_path),
        "version": version,
        "version_u32": f"0x{version_u32:08X}",
        "image_bytes": prepared.size,
        "crc32": f"0x{prepared.crc32:08X}",
        "git_commit": git["commit"],
        "git_dirty": git["dirty"],
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    print(f'升级: GUI 升级页选择该 bin，或  mdrive upgrade --image "{out}" --version 0x{version_u32:08X}')
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
