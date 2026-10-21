"""组合已构建EXE、初装包及CAN包；只写发布目录，不访问硬件。"""

import argparse
from datetime import datetime, timedelta, timezone
import hashlib
import json
from pathlib import Path
import shutil
import zipfile

from loader_factory import load_package
from maintenance_backend import version_text
from maintenance_version import VERSION_TEXT


def main():
    """先校验输入和源码快照，再生成下载文件、逐文件哈希及ZIP。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--flasher-build", type=Path, required=True)
    parser.add_argument("--factory-package", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument(
        "--loader-version", type=lambda text: int(text, 0), required=True
    )
    args = parser.parse_args()
    if not 0 <= args.loader_version <= 0xFFFFFFFF:
        raise ValueError("Loader版本编码超出uint32范围")
    _, firmware = load_package(args.factory_package)
    build = json.loads((args.flasher_build / "build_summary.json").read_text())
    exe = args.flasher_build / "download/VectorMiniFlasher.exe"
    if (
        build["software_version"] != VERSION_TEXT
        or hashlib.sha256(exe.read_bytes()).hexdigest() != build["exe_sha256"]
    ):
        raise ValueError("EXE版本或哈希与构建记录不匹配")
    tools = Path(__file__).resolve().parent
    for name, digest in build["source_sha256"].items():
        if hashlib.sha256((tools / name).read_bytes()).hexdigest() != digest:
            raise ValueError("冻结EXE与当前源码不一致：" + name)
    date = datetime.now(timezone(timedelta(hours=8))).strftime("%Y%m%d")
    loader_version = version_text(args.loader_version)
    app_version = version_text(firmware["app"]["app_version"])
    label = f"Loader_v{loader_version}_APP_v{app_version}_{date}"
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=False)
    download = out / "download"
    download.mkdir()
    shutil.copy2(exe, download / exe.name)
    shutil.copytree(args.factory_package, download / "factory")
    shutil.copy2(args.factory_package / "combined.hex", download / (label + ".hex"))
    can = download / "can_app"
    can.mkdir()
    shutil.copy2(args.factory_package / "app.bin", can / "app.bin")
    (can / "manifest.json").write_text(
        json.dumps(firmware["app"], indent=2) + "\n", encoding="utf-8"
    )
    scripts = download / "scripts"
    scripts.mkdir()
    for name in (
        "flash_factory.ps1",
        "loader_factory.py",
        "loader_updater.py",
        "loader_proto.py",
        "usbcan_adapter.py",
    ):
        shutil.copy2(tools / name, scripts / name)
    for name in ("THIRD_PARTY_NOTICES.txt", "SEGGER_License.txt"):
        source = args.flasher_build / "drivers" / name
        if source.is_file():
            shutil.copy2(source, download / name)
    shutil.copy2(
        tools.parents[1] / "docs/guides/loader_flasher_exe_v1.md",
        download / "维护工具说明.md",
    )
    (download / "使用说明.txt").write_text(
        f"维护软件 v{VERSION_TEXT} / Loader v{loader_version} / APP v{app_version}\n\n"
        "解压后双击 VectorMiniFlasher.exe，两个DLL已内置，无需安装Python。\n"
        "仅有旧APP的板：先在初装烧录页选择根目录合并HEX，经J-Link安装Loader+APP。\n"
        "已有Loader的板：CAN升级页选择can_app/manifest.json，使用USB-CAN升级APP。\n"
        "保留原板参数，不执行全片擦除；不导入其他电机的标定。\n"
        "版本页读取运行固件；标定页读取保存的标志、零位和LUT。\n"
        "备选脚本：安装pylink-square==2.0.2，使用scripts/flash_factory.ps1，\n"
        "Package指定factory目录，并显式指定探针序列号、MCU UID及记录目录。\n"
        "新电脑仍需对应的Windows USB设备驱动。操作记录位于LOCALAPPDATA/VectorMiniFlasher/outputs。\n",
        encoding="utf-8-sig",
    )
    manifest = {
        "format": "vector-mini-maintenance-package-v1",
        "status": "PACKAGED",
        "software_version": VERSION_TEXT,
        "loader_version": loader_version,
        "app_version": app_version,
        "date": date,
        "files": {
            path.relative_to(download).as_posix(): {
                "bytes": path.stat().st_size,
                "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            }
            for path in sorted(download.rglob("*"))
            if path.is_file()
        },
    }
    (download / "manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    archive = out / f"VectorMini_v{VERSION_TEXT}_{label}.zip"
    with zipfile.ZipFile(archive, "w", zipfile.ZIP_DEFLATED) as bundle:
        for path in sorted(download.rglob("*")):
            if path.is_file():
                bundle.write(path, path.relative_to(download))
    print(archive)


if __name__ == "__main__":
    main()
