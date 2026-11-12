"""构建可独立解压的 Windows x64 Rust 上位机及 SWD 工具包。"""

import argparse
import hashlib
import importlib.metadata
import json
import shutil
import subprocess
import sys
import tomllib
import xml.etree.ElementTree as ET
from pathlib import Path
from tempfile import TemporaryDirectory
from zipfile import ZIP_DEFLATED, ZipFile

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "loader/tools"))
from device_maintenance import load_bundle  # noqa: E402


def copy_python(dest: Path) -> None:
    base = Path(sys.base_prefix)
    dest.mkdir(parents=True)
    for name in (
        "python.exe",
        "python3.dll",
        "python313.dll",
        "vcruntime140.dll",
        "vcruntime140_1.dll",
        "LICENSE.txt",
    ):
        shutil.copy2(base / name, dest)
    shutil.copytree(
        base / "DLLs",
        dest / "DLLs",
        ignore=shutil.ignore_patterns("*test*", "*tkinter*", "tcl*.dll", "tk*.dll"),
    )
    lib = base / "Lib"
    with ZipFile(dest / "python313.zip", "x", ZIP_DEFLATED) as stdlib:
        for path in sorted(lib.rglob("*")):
            relative = path.relative_to(lib)
            if path.is_file() and not {
                "__pycache__",
                "site-packages",
                "test",
                "tests",
                "idlelib",
                "ensurepip",
                "tkinter",
                "turtledemo",
            }.intersection(relative.parts):
                stdlib.write(path, relative)
    (dest / "python313._pth").write_text("python313.zip\nDLLs\npackages\n.\n", encoding="ascii")
    for name in ("pylink-square", "six", "psutil"):
        distribution = importlib.metadata.distribution(name)
        for path in distribution.files or []:
            if ".." in path.parts or "__pycache__" in path.parts or path.is_absolute():
                continue
            output = dest / "packages" / path
            output.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(distribution.locate_file(path), output)


def copy_jlink(source: Path, dest: Path, device: str) -> None:
    dest.mkdir(parents=True)
    shutil.copy2(source / "JLink_x64.dll", dest)
    for pattern in ("msvcp140*.dll", "vcruntime140*.dll", "concrt140.dll"):
        for lib in source.glob(pattern):
            shutil.copy2(lib, dest)
    # AT32 的算法和连接脚本由板厂器件描述引用。仅随包携带支持板所需的文件。
    entries = ET.parse(source / "JLinkDevices.xml").getroot()
    entry = next(e for e in entries.findall("Device") if e.find("ChipInfo").get("Name") == device)
    root = ET.Element("DataBase")
    root.append(entry)
    for reference in (
        entry.find("ChipInfo").attrib["JLinkScriptFile"],
        entry.find("FlashBankInfo").attrib["Loader"],
    ):
        origin = (source / reference).resolve()
        relative = origin.relative_to(source.resolve())
        (dest / relative).parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(origin, dest / relative)
    # SDK 9.64 的器件解析器要求根标记和 Device 分行，使用厂商的原始文本格式。
    ET.indent(root)
    xml = ET.tostring(root, encoding="unicode").replace(" />", "/>")
    (dest / "JLinkDevices.xml").write_text(xml + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    for option in ("exe", "can-dll", "jlink-dir", "firmware"):
        parser.add_argument("--" + option, type=Path, required=True)
    parser.add_argument("--out", type=Path, default=ROOT / "outputs/deliveries")
    args = parser.parse_args()
    if sys.platform != "win32" or sys.maxsize <= 2**32 or sys.version_info[:2] != (3, 13):
        parser.error("打包需使用项目的 Windows x64 Python 3.13 环境")
    with (ROOT / "host_app/rust/Cargo.toml").open("rb") as config:
        version = tomllib.load(config)["package"]["version"]
    exe = args.exe.resolve()
    if subprocess.check_output([exe, "--version"], text=True).strip() != f"YG_MC_HOST v{version}":
        raise ValueError("EXE 版本与发布版本不符")
    summary = json.loads((args.firmware / "summary.json").read_bytes())
    load_bundle(args.firmware, summary["target"])
    at32 = json.loads((ROOT / "shared/targets/yg_esc_v1_at32f456.json").read_bytes())
    name = f"YG_MC_HOST_v{version}"
    args.out.mkdir(parents=True, exist_ok=True)
    archive = args.out.resolve() / f"{name}-win64.zip"
    with TemporaryDirectory(dir=args.out) as tmp:
        stage = Path(tmp).resolve()
        kit = stage / name
        kit.mkdir()
        shutil.copy2(exe, kit / f"{name}.exe")
        shutil.copy2(args.can_dll, kit / "ControlCANFD.dll")
        copy_python(kit / "runtime/python")
        copy_jlink(args.jlink_dir, kit / "runtime/jlink", at32["build"]["jlink_device"])
        for path in (
            "loader/tools/device_maintenance.py",
            "host_app/upgrade_client.py",
            "shared/targets/target_contract.py",
            "shared/targets/vector_mini_st_g431.json",
            "shared/targets/yg_esc_v1_at32f456.json",
        ):
            (kit / path).parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(ROOT / path, kit / path)
        for path in ["summary.json", *(item["path"] for item in summary["artifacts"])]:
            dest = kit / "firmware" / path
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(args.firmware / path, dest)
        (kit / "打开离线样机.cmd").write_bytes(
            f'@start "" "%~dp0{name}.exe" --studio-preview\r\n'.encode("ascii")
        )
        note = (
            f"YG_MC_HOST v{version} · Windows x64\n"
            f"整机包目标 {summary['target']}，Loader {summary['combined']['loader_version']}，APP {summary['combined']['app_version']}。\n"
            "解压整个文件夹，双击 EXE。中控 → 烧录整机固件。自动识别 USB J-Link，多支时选择序列号。"
            "选择板型，在设备停机且独占 SWD 时开始备份或安装。上位机自动读取、校验并显示 MCU UID。换板后请重新扫描。Windows 需能够识别 USB 探针。\n"
            "上位机使用随包 Python 和 SWD 脚本，备份与烧录记录保存在 %LOCALAPPDATA%/YG/YG_MC_HOST/swd。\n"
            "中控 → 添加设备会自动扫描 CAN 适配器与节点，选择节点后点击连接。\n"
        )
        (kit / "版本说明.txt").write_text(note, encoding="utf-8-sig")
        # 直接由发布解释器验证双板器件索引。DLL 加载成功不足以证明 AT32 FLM 已被识别。
        subprocess.run(
            [
                kit / "runtime/python/python.exe",
                "-X",
                "utf8",
                "-B",
                "-c",
                "import sys;from pathlib import Path;"
                "kit=Path(sys.argv[1]);sys.path.insert(0,str(kit/'loader/tools'));"
                "import device_maintenance as m;probe=m.create_probe(None);"
                "[probe.get_device_index(m.load_target(t)['build']['jlink_device']) "
                "for t in ('yg_esc_v1_at32f456','vector_mini_st_g431')];"
                "m.load_bundle(kit/'firmware',sys.argv[2])",
                kit,
                summary["target"],
            ],
            cwd=stage,
            check=True,
        )
        files = {
            p.relative_to(kit).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(kit.rglob("*"))
            if p.is_file()
        }
        (kit / "manifest.json").write_text(
            json.dumps({"version": version, "files": files}, indent=2), encoding="utf-8"
        )
        with ZipFile(stage / archive.name, "x", ZIP_DEFLATED) as package:
            for path in sorted(kit.rglob("*")):
                if path.is_file():
                    package.write(path, path.relative_to(stage))
        (stage / archive.name).replace(archive)
    print(archive)


if __name__ == "__main__":
    main()
