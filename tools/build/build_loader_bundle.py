"""独立构建并冻结Loader/配套APP/schema12部署包；只构建，不下载或接触CAN/J-Link。"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import shutil
import subprocess
import sys
import time
from datetime import datetime, timedelta, timezone
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from harness.common import git_info
from project_paths import ROOT


def main() -> int:
    """两工程都零错误/警告且产物新鲜才冻结；任何失败不输出成功summary。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--uv4", type=Path, required=True)
    parser.add_argument("--fromelf", type=Path, required=True)
    parser.add_argument("--token", type=lambda value: int(value, 0), required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not 1 <= args.token <= 0xFFFFFFFF:
        parser.error("token必须是非零u32")
    output = args.output.resolve()
    if not output.is_relative_to((ROOT / "outputs").resolve()):
        parser.error("构建与冻结证据必须放当前工作树outputs下")
    if not args.uv4.is_file() or not args.fromelf.is_file():
        parser.error("编译器入口或fromelf不存在")
    output.mkdir(parents=True, exist_ok=False)
    source = git_info()
    target_file = ROOT / "shared/targets/vector_mini_st_g431.json"
    constants = json.loads(target_file.read_text(encoding="utf-8"))["constants"]
    date = datetime.now(timezone(timedelta(hours=8))).strftime("%Y%m%d")
    report = {
        "success": False,
        "release_approved": False,
        "hardware_validation": "not_run",
        "source": source,
        "target_sha256": hashlib.sha256(target_file.read_bytes()).hexdigest(),
        "artifacts": [],
        "resources": {},
        "parameter_schema": 12,
        "parameter_bytes": 4304,
        "profile": "loader-image-v3-crc32",
    }
    try:
        for product, name, project, directory, version_key in (
            (
                "LOADER",
                "YG2026_ESC_LOADER",
                "loader/targets/vector_mini_st_g431/YG2026_ESC_LOADER.uvprojx",
                "outputs/build/loader/vector_mini_st_g431",
                "TARGET_LOADER_VERSION",
            ),
            (
                "APP",
                "Vector_Mini_ST_LoaderApp",
                "firmware/platform/stm32g4/cubemx/MDK-ARM/Vector_Mini_ST_LoaderApp.uvprojx",
                "outputs/build/keil/Vector_Mini_ST_LoaderApp",
                "TARGET_APP_VERSION",
            ),
        ):
            log = output / f"{name}.log"
            started = time.time()
            result = subprocess.run(
                [str(args.uv4), "-r", str(ROOT / project), "-j0", "-o", str(log)],
                timeout=180,
                check=False,
            )
            text = log.read_text(encoding="utf-8", errors="replace") if log.is_file() else ""
            if result.returncode or re.search(r"\b0 Error\(s\), 0 Warning\(s\)", text) is None:
                raise RuntimeError(f"{name}构建失败或存在警告；检查当前构建日志")
            version = int(constants[version_key].removesuffix("U"), 0)
            tag = f"v{version >> 16}.{(version >> 8) & 255}.{version & 255}"
            prefix = f"YG2026-ESC-{product}-STM32G431CB-debug-{tag}-{date}"
            for suffix in ("axf", "hex", "map"):
                artifact = ROOT / directory / f"{name}.{suffix}"
                if not artifact.is_file() or artifact.stat().st_mtime < started - 2:
                    raise RuntimeError(f"{artifact.name}缺失或不是本次构建产物")
                artifact_suffix = (
                    "unpackaged.hex" if product == "APP" and suffix == "hex" else suffix
                )
                destination = output / f"{prefix}.{artifact_suffix}"
                shutil.copyfile(artifact, destination)
                report["artifacts"].append(
                    {
                        "path": destination.name,
                        "bytes": destination.stat().st_size,
                        "sha256": hashlib.sha256(destination.read_bytes()).hexdigest(),
                    }
                )
            binary = output / f"{prefix}.bin"
            subprocess.run(
                [
                    str(args.fromelf),
                    "--bin",
                    "--output",
                    str(binary),
                    str(output / f"{prefix}.axf"),
                ],
                check=True,
                timeout=30,
            )
            if product == "APP":
                subprocess.run(
                    [
                        sys.executable,
                        str(ROOT / "loader/tools/package_image.py"),
                        "--app-bin",
                        str(binary),
                        "--token",
                        str(args.token),
                        "--output",
                        str(output / "app-package"),
                    ],
                    check=True,
                    timeout=30,
                )
                # 标准APP文件名必须对应可下载包；原始占位BIN只留作打包溯源。
                binary.rename(output / f"{prefix}.unpackaged.bin")
                shutil.copyfile(output / "app-package/image.bin", binary)
            report["artifacts"].append(
                {
                    "path": binary.name,
                    "bytes": binary.stat().st_size,
                    "sha256": hashlib.sha256(binary.read_bytes()).hexdigest(),
                }
            )
            map_text = (output / f"{prefix}.map").read_text(encoding="utf-8", errors="replace")
            flash_region = re.search(r"Load Region LR_\w+ \(.*?Size: (0x[0-9a-fA-F]+)", map_text)
            ram_region = re.search(r"Execution Region RW_\w+ \(.*?Size: (0x[0-9a-fA-F]+)", map_text)
            if flash_region is None or ram_region is None:
                raise RuntimeError("MAP缺少固定Flash/RAM区域，无法冻结资源证据")
            report["resources"][product] = {
                "flash_region_bytes": int(flash_region[1], 16),
                "download_bytes": binary.stat().st_size,
                "ram_region_bytes": int(ram_region[1], 16),
            }
        for artifact in sorted((output / "app-package").iterdir()):
            if artifact.is_file():
                report["artifacts"].append(
                    {
                        "path": artifact.relative_to(output).as_posix(),
                        "bytes": artifact.stat().st_size,
                        "sha256": hashlib.sha256(artifact.read_bytes()).hexdigest(),
                    }
                )
        shutil.copyfile(target_file, output / "target.json")
        for artifact in [output / "target.json", *output.glob("*.unpackaged.bin")]:
            report["artifacts"].append(
                {
                    "path": artifact.name,
                    "bytes": artifact.stat().st_size,
                    "sha256": hashlib.sha256(artifact.read_bytes()).hexdigest(),
                }
            )
        report["success"] = True
    except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
        report["failure"] = str(exc)
    finally:
        (output / "summary.json").write_text(
            json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
        )
    return 0 if report["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
