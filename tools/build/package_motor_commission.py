"""生成包含 Windows 工装、AT32 FULL 和六轴参数生成器的独立版本包。"""

import argparse
import hashlib
import json
import shutil
import subprocess
import sys
from pathlib import Path
from tempfile import TemporaryDirectory
from zipfile import ZIP_DEFLATED, ZipFile

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / path) for path in ("tools/bench", "tools/build", "tools")]
from commission_motor import (  # noqa: E402
    GEARED,
    GENERATOR_NAME,
    PARAMETER_VERSION,
    TOOL_NAME,
    VERSION,
    default_speed,
    new_record,
    parameter_filename,
)
from harness.common import zig_path  # noqa: E402
from install_axis_board import AXES, BENCH_THERMAL, load_factory  # noqa: E402
from make_axis_record import build_generator  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", type=Path, required=True)
    parser.add_argument("--can-dll", type=Path, required=True)
    parser.add_argument("--out", type=Path, default=ROOT / "outputs/deliveries")
    args = parser.parse_args()
    source = args.bundle.resolve()
    load_factory(source)
    contract = ROOT / "shared/targets/yg_esc_v1_at32f456.json"
    if json.loads((source / "target.json").read_text()) != json.loads(contract.read_text()):
        raise ValueError("包内目标契约与工装版本不一致")
    compiler = zig_path()
    if not compiler:
        raise RuntimeError("打包机需要仓库配套 Zig")
    args.out.mkdir(parents=True, exist_ok=True)
    name = f"YG-Motor-Commission-v{VERSION}-win64"
    archive = args.out / (name + ".zip")
    if archive.exists():
        raise FileExistsError(archive)
    with TemporaryDirectory(dir=args.out) as tmp:
        work = Path(tmp).resolve()
        kit = work / name
        kit.mkdir()
        (kit / "bin").mkdir()
        build_generator(kit / "bin" / GENERATOR_NAME, compiler)
        (kit / "parameters").mkdir()
        for axis in AXES:
            thermal = BENCH_THERMAL if axis.node in GEARED else ()
            raw = new_record(kit, axis, default_speed(axis), thermal)
            (kit / "parameters" / parameter_filename(axis)).write_bytes(raw)
        version_tuple = (*map(int, VERSION.split(".")), 0)
        version_file = work / "windows-version.txt"
        version_file.write_text(
            f"VSVersionInfo(ffi=FixedFileInfo(filevers={version_tuple}, prodvers={version_tuple}, "
            "mask=0x3f, flags=0, OS=0x40004, fileType=1, subtype=0, date=(0,0)), "
            "kids=[StringFileInfo([StringTable('040904B0', ["
            f"StringStruct('FileVersion','{VERSION}'), StringStruct('ProductVersion','{VERSION}'), "
            f"StringStruct('OriginalFilename','{TOOL_NAME}.exe'), "
            "StringStruct('ProductName','YG Motor Commission')])]), "
            "VarFileInfo([VarStruct('Translation',[1033,1200])])])",
            encoding="utf-8",
        )
        shutil.copy2(args.can_dll, kit / "bin/ControlCANFD.dll")
        summary = json.loads((source / "summary.json").read_text(encoding="utf-8"))
        for entry in summary["artifacts"]:
            path = Path(entry["path"])
            dest = kit / "firmware" / path
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source / path, dest)
        # 仅随交付传送构建身份，工作站的本地改动列表与工装无关。
        summary["source"].pop("status", None)
        (kit / "firmware/summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
        shutil.copy2(ROOT / "docs/guides/motor_commission.md", kit / "使用说明.md")
        subprocess.run(
            [
                sys.executable,
                "-m",
                "PyInstaller",
                "--onefile",
                "--console",
                "--noupx",
                "--distpath",
                str(kit),
                "--workpath",
                str(work / "build"),
                "--specpath",
                str(work),
                "--name",
                TOOL_NAME,
                "--version-file",
                str(version_file),
                "--copy-metadata",
                "pylink-square",
                "--add-data",
                f"{contract}:.",
                "--add-data",
                f"{contract}:contracts",
                *[
                    arg
                    for path in (
                        "tools",
                        "tools/bench",
                        "tools/build",
                        "loader/tools",
                        "host_app",
                        "shared/targets",
                    )
                    for arg in ("--paths", str(ROOT / path))
                ],
                str(ROOT / "tools/bench/commission_motor.py"),
            ],
            check=True,
            cwd=work,
        )
        manifest = {
            "version": VERSION,
            "tool_executable": f"{TOOL_NAME}.exe",
            "parameter_version": PARAMETER_VERSION,
            "parameter_schema": 14,
            "parameter_presets": {
                f"0x{axis.node:02X}": "parameters/" + parameter_filename(axis) for axis in AXES
            },
            "full_hex": "firmware/"
            + next(
                item["path"]
                for item in summary["artifacts"]
                if item["path"].startswith("YG2026-ESC-FULL-")
            ),
            "platform": "Windows x64",
            "firmware_source": summary["source"],
            "firmware_release_approved": summary["release_approved"],
            "full_hex_sha256": next(
                item["sha256"]
                for item in summary["artifacts"]
                if item["path"].startswith("YG2026-ESC-FULL-")
            ),
            "firmware": summary["combined"],
            "files": {
                p.relative_to(kit).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
                for p in sorted(kit.rglob("*"))
                if p.is_file()
            },
        }
        (kit / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
        subprocess.run([str(kit / f"{TOOL_NAME}.exe"), "--check"], cwd=work, check=True)
        staged = work / archive.name
        with ZipFile(staged, "x", compression=ZIP_DEFLATED) as output:
            for path in sorted(kit.rglob("*")):
                if path.is_file():
                    output.write(path, path.relative_to(work))
        staged.rename(archive.resolve())
    print(archive.resolve(), hashlib.sha256(archive.read_bytes()).hexdigest())


if __name__ == "__main__":
    main()
