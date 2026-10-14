"""将GUI打包为Windows x64单文件；构建只写outputs，不访问探针或CAN。"""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import struct


def main():
    """使用当前环境的已安装PyInstaller，记录版本、源文件及EXE哈希。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--jlink-dll", type=Path, required=True)
    parser.add_argument("--can-dll", type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=False)
    drivers = out / "drivers"
    drivers.mkdir()
    driver_manifest = {
        "format": "vector-mini-bundled-drivers-v1",
        "architecture": "Windows x64",
        "files": {},
    }
    for path, name in (
        (args.jlink_dll, "JLink_x64.dll"),
        (args.can_dll, "ControlCANFD.dll"),
    ):
        data = path.read_bytes()
        if len(data) < 64 or data[:2] != b"MZ":
            raise ValueError("驱动不是Windows PE文件：" + name)
        pe = struct.unpack_from("<I", data, 60)[0]
        if (
            pe + 6 > len(data)
            or data[pe : pe + 4] != b"PE\0\0"
            or struct.unpack_from("<H", data, pe + 4)[0] != 0x8664
        ):
            raise ValueError("驱动不是64位DLL：" + name)
        (drivers / name).write_bytes(data)
        driver_manifest["files"][name] = {
            "bytes": len(data),
            "sha256": hashlib.sha256(data).hexdigest(),
        }
    (drivers / "manifest.json").write_text(
        json.dumps(driver_manifest, indent=2) + "\n", encoding="utf-8"
    )
    license_path = args.jlink_dll.parent / "Doc/LicenseIncGUI.txt"
    if license_path.is_file():
        (drivers / "SEGGER_License.txt").write_bytes(license_path.read_bytes())
    (drivers / "THIRD_PARTY_NOTICES.txt").write_text(
        "J-Link DLL: Copyright SEGGER Microcontroller GmbH.\n"
        "Use with original SEGGER / authorized OEM probes. Redistribution information:\n"
        "https://www.segger.com/products/debug-probes/j-link/tools/j-link-sdk/\n"
        "ControlCANFD.dll: vendor binary supplied by the local USB-CAN SDK; not modified.\n"
        "Bundled DLLs do not install Windows USB kernel drivers.\n",
        encoding="utf-8",
    )
    command = [
        sys.executable,
        "-m",
        "PyInstaller",
        "--onefile",
        "--windowed",
        "--clean",
        "--noconfirm",
        "--name",
        "VectorMiniFlasher",
        "--hidden-import",
        "pylink",
        "--add-binary",
        str(drivers / "JLink_x64.dll") + ";drivers",
        "--add-binary",
        str(drivers / "ControlCANFD.dll") + ";drivers",
        "--add-data",
        str(drivers / "manifest.json") + ";drivers",
        "--add-data",
        str(drivers / "THIRD_PARTY_NOTICES.txt") + ";drivers",
        "--distpath",
        str(out / "download"),
        "--workpath",
        str(out / "build"),
        "--specpath",
        str(out / "build"),
        str(root / "loader/tools/loader_flasher_gui.py"),
    ]
    if (drivers / "SEGGER_License.txt").is_file():
        command += ["--add-data", str(drivers / "SEGGER_License.txt") + ";drivers"]
    with (out / "build.log").open("w", encoding="utf-8") as log:
        result = subprocess.run(command, cwd=root, stdout=log, stderr=subprocess.STDOUT)
    result.check_returncode()
    exe = out / "download/VectorMiniFlasher.exe"
    paths = [
        "loader_flasher_gui.py",
        "loader_factory.py",
        "loader_updater.py",
        "loader_proto.py",
        "usbcan_adapter.py",
        "maintenance_backend.py",
    ]
    snapshot = out / "source_snapshot"
    snapshot.mkdir()
    hashes = {}
    for name in paths:
        data = (root / "loader/tools" / name).read_bytes()
        (snapshot / name).write_bytes(data)
        hashes[name] = hashlib.sha256(data).hexdigest()
    versions = subprocess.check_output(
        [sys.executable, "-m", "pip", "freeze"], text=True
    )
    (out / "requirements_snapshot.txt").write_text(versions, encoding="utf-8")
    summary = {
        "status": "BUILT_NOT_HARDWARE_TESTED",
        "hardware_accessed": False,
        "exe": exe.name,
        "exe_bytes": exe.stat().st_size,
        "exe_sha256": hashlib.sha256(exe.read_bytes()).hexdigest(),
        "source_sha256": hashes,
        "python_version": sys.version,
        "jlink_dll_included": True,
        "can_dll_included": True,
        "bundled_drivers": driver_manifest,
        "requires": "Windows USB device drivers; no manual DLL selection or Python installation",
    }
    (out / "build_summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
