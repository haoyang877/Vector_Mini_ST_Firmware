"""合成初装包并显式通过J-Link烧录；保留页及16 KiB参数区不在写入范围。"""

import argparse
import hashlib
import json
from pathlib import Path
import struct
import subprocess
import sys
import time

from loader_proto import crc32
from loader_updater import (
    APP_BASE,
    CAPACITY,
    HARDWARE_ID,
    LAYOUT_ID,
    PARAM_ABI,
    validate_image,
)

FLASH_BASE = 0x08000000
LOADER_BYTES = 0x3000
RECORD_BASE = FLASH_BASE + LOADER_BYTES
PAGE_BYTES = 0x800
PARAM_BASE = APP_BASE + CAPACITY
FLASH_BYTES = 0x20000


def sha256(data):
    """返回文件内容的SHA256。"""
    return hashlib.sha256(data).hexdigest()


def boot_record(app, version):
    """编码与Boot_RecordEncode相同的48 B小端有效记录。"""
    prefix = struct.pack(
        "<IHH7I",
        0x3252444C,
        1,
        48,
        LAYOUT_ID,
        len(app),
        crc32(app),
        version,
        HARDWARE_ID,
        PARAM_ABI,
        0,
    )
    return prefix + struct.pack("<I", crc32(prefix)) + b"BCOMMIT1"


def validate_components(loader, app, manifest):
    """在打开硬件前核对两个向量、容量及APP清单；拒绝旧基址APP。"""
    validate_image(app)
    if not 8 <= len(loader) <= LOADER_BYTES:
        raise ValueError("Loader exceeds 12 KiB or has no vectors")
    sp, reset = struct.unpack_from("<II", loader)
    if (
        sp % 8
        or not 0x20000028 <= sp <= 0x20008000
        or not reset & 1
        or not FLASH_BASE <= reset & ~1 < FLASH_BASE + len(loader)
    ):
        raise ValueError("Invalid Loader stack/reset vector")
    expected = {
        "app_base": APP_BASE,
        "layout_id": LAYOUT_ID,
        "hardware_id": HARDWARE_ID,
        "param_abi": PARAM_ABI,
        "protocol_version": 2,
        "image_bytes": len(app),
        "image_crc32": crc32(app),
        "image_sha256": sha256(app),
    }
    if any(manifest.get(k) != v for k, v in expected.items()):
        raise ValueError("APP manifest/layout mismatch")
    version = manifest.get("app_version")
    if type(version) is not int or not 0 <= version <= 0xFFFFFFFF:
        raise ValueError("Invalid APP version")


def segments(loader, app, record):
    """写入单元覆盖各自整页；不覆盖页7或参数页56～63。"""
    return (
        (FLASH_BASE, loader.ljust(LOADER_BYTES, b"\xff")),
        (RECORD_BASE, record.ljust(PAGE_BYTES, b"\xff")),
        (APP_BASE, app.ljust(CAPACITY, b"\xff")),
    )


def combined_hex(parts):
    """生成带地址的稀疏Intel HEX，仅包含三个允许写入的分区。"""
    lines = []

    def emit(kind, address, payload):
        raw = struct.pack(">BHB", len(payload), address, kind) + payload
        lines.append(":" + (raw + bytes([-sum(raw) & 255])).hex().upper())

    upper = None
    for base, data in parts:
        for offset in range(0, len(data), 16):
            address = base + offset
            if address >> 16 != upper:
                upper = address >> 16
                emit(4, 0, struct.pack(">H", upper))
            emit(0, address & 0xFFFF, data[offset : offset + 16])
    emit(1, 0, b"")
    return "\n".join(lines) + "\n"


def build_package(loader_path, app_package, out, app_bin=None, app_version=None):
    """从独立构建的Loader BIN和CAN APP包生成可追溯初装包，不访问硬件。"""
    loader = loader_path.read_bytes()
    if app_package:
        app = (app_package / "app.bin").read_bytes()
        app_manifest = json.loads(
            (app_package / "manifest.json").read_text(encoding="utf-8")
        )
    else:
        app = app_bin.read_bytes()
        app_manifest = {
            "app_base": APP_BASE,
            "layout_id": LAYOUT_ID,
            "hardware_id": HARDWARE_ID,
            "param_abi": PARAM_ABI,
            "protocol_version": 2,
            "app_version": app_version,
            "image_bytes": len(app),
            "image_crc32": crc32(app),
            "image_sha256": sha256(app),
            "note": "Raw BIN input; supplied version must match the independently built APP.",
        }
    validate_components(loader, app, app_manifest)
    record = boot_record(app, app_manifest["app_version"])
    files = {
        "loader.bin": loader,
        "app.bin": app,
        "boot_record.bin": record,
        "combined.hex": combined_hex(segments(loader, app, record)).encode("ascii"),
    }
    root = Path(__file__).resolve().parents[2]
    head = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=root, text=True
    ).strip()
    dirty = bool(subprocess.check_output(["git", "status", "--porcelain"], cwd=root))
    manifest = {
        "format": "vector-mini-factory-v1",
        "device": "STM32G431CB",
        "status": "OFFLINE_PACKAGED_NOT_FLASHED",
        "app": app_manifest,
        "packaging_source_head": head,
        "packaging_source_dirty": dirty,
        "files": {
            name: {"bytes": len(data), "sha256": sha256(data)}
            for name, data in files.items()
        },
        "write_ranges": [
            [base, base + len(data)] for base, data in segments(loader, app, record)
        ],
        "preserved_ranges": [
            [RECORD_BASE + PAGE_BYTES, APP_BASE],
            [PARAM_BASE, FLASH_BASE + FLASH_BYTES],
        ],
        "note": "Initial SWD deployment; no parameters included. Not a CAN update package.",
    }
    out.mkdir(parents=True, exist_ok=False)
    for name, data in files.items():
        (out / name).write_bytes(data)
    (out / "manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )
    return manifest


def load_package(package):
    """严格预检清单、文件哈希、记录和HEX；失败不得打开J-Link。"""
    manifest = json.loads((package / "manifest.json").read_text(encoding="utf-8"))
    if (
        manifest.get("format") != "vector-mini-factory-v1"
        or manifest.get("device") != "STM32G431CB"
    ):
        raise ValueError("Unknown factory package format/device")
    names = ("loader.bin", "app.bin", "boot_record.bin", "combined.hex")
    if set(manifest.get("files", {})) != set(names):
        raise ValueError("Factory files mismatch")
    files = {name: (package / name).read_bytes() for name in names}
    for name, data in files.items():
        if manifest["files"][name] != {"bytes": len(data), "sha256": sha256(data)}:
            raise ValueError("Factory hash/length mismatch: " + name)
    loader, app, record = (files[name] for name in names[:3])
    validate_components(loader, app, manifest["app"])
    parts = segments(loader, app, record)
    if record != boot_record(app, manifest["app"]["app_version"]):
        raise ValueError("Boot record does not match APP")
    if files["combined.hex"] != combined_hex(parts).encode("ascii"):
        raise ValueError("Combined HEX does not match components")
    if manifest.get("write_ranges") != [
        [base, base + len(data)] for base, data in parts
    ]:
        raise ValueError("Write ranges mismatch")
    if manifest.get("preserved_ranges") != [
        [RECORD_BASE + PAGE_BYTES, APP_BASE],
        [PARAM_BASE, FLASH_BASE + FLASH_BYTES],
    ]:
        raise ValueError("Preserved ranges mismatch")
    return parts, manifest


def load_hex(path: Path) -> tuple:
    """读取当前初装布局的完整HEX；拒绝保护区、重叠、坏记录及不匹配APP。"""
    if path.stat().st_size > 2 * 1024 * 1024:
        raise ValueError("HEX文件超过2 MiB")
    image = bytearray(b"\xff" * FLASH_BYTES)
    covered = bytearray(FLASH_BYTES)
    upper, ended = 0, False
    allowed = (
        (FLASH_BASE, RECORD_BASE + PAGE_BYTES),
        (APP_BASE, PARAM_BASE),
    )
    source = path.read_bytes()
    for number, line in enumerate(source.decode("ascii").splitlines(), 1):
        line = line.strip()
        if not line:
            continue
        if ended or not line.startswith(":"):
            raise ValueError(f"HEX第{number}行格式或结束标记错误")
        raw = bytes.fromhex(line[1:])
        if len(raw) < 5 or len(raw) != raw[0] + 5 or sum(raw) & 255:
            raise ValueError(f"HEX第{number}行长度或校验和错误")
        count, address, kind = raw[0], int.from_bytes(raw[1:3], "big"), raw[3]
        payload = raw[4:-1]
        if kind == 0:
            base = upper + address
            if (
                count == 0
                or address + count > 0x10000
                or not any(
                    start <= base and base + count <= end for start, end in allowed
                )
            ):
                raise ValueError(f"HEX第{number}行包含保护区或错误地址")
            offset = base - FLASH_BASE
            if any(covered[offset : offset + count]):
                raise ValueError(f"HEX第{number}行地址重复")
            image[offset : offset + count] = payload
            covered[offset : offset + count] = b"\x01" * count
        elif kind == 4 and count == 2 and address == 0:
            upper = int.from_bytes(payload, "big") << 16
        elif kind == 1 and count == 0 and address == 0:
            ended = True
        else:
            raise ValueError(f"HEX第{number}行记录类型不支持")
    if not ended or any(
        0 in covered[start - FLASH_BASE : end - FLASH_BASE] for start, end in allowed
    ):
        raise ValueError("HEX缺少结束记录或完整Loader/记录/APP分区")
    loader = bytes(image[:LOADER_BYTES])
    record_page = bytes(image[LOADER_BYTES : LOADER_BYTES + PAGE_BYTES])
    record = record_page[:48]
    size, _, version = struct.unpack_from("<3I", record, 12)
    if not 8 <= size <= CAPACITY or size % 8:
        raise ValueError("APP记录大小/对齐错误")
    app_page = bytes(image[APP_BASE - FLASH_BASE : PARAM_BASE - FLASH_BASE])
    app = app_page[:size]
    if (
        record != boot_record(app, version)
        or record_page[48:] != b"\xff" * (PAGE_BYTES - 48)
        or app_page[size:] != b"\xff" * (CAPACITY - size)
    ):
        raise ValueError("APP记录、CRC、布局或填充区错误")
    app_manifest = {
        "app_base": APP_BASE,
        "layout_id": LAYOUT_ID,
        "hardware_id": HARDWARE_ID,
        "param_abi": PARAM_ABI,
        "protocol_version": 2,
        "app_version": version,
        "image_bytes": size,
        "image_crc32": crc32(app),
        "image_sha256": sha256(app),
    }
    validate_components(loader, app, app_manifest)
    return segments(loader, app, record), {
        "app": app_manifest,
        "hex_sha256": sha256(source),
    }


def check_power_off(probe):
    """允许空片输入、STOP模拟态或Loader输出低；PWM关断且两次相关位一致。"""
    rows = []
    for _ in range(2):
        values = [
            probe.memory_read32(a, 1)[0]
            for a in (
                0x40012C20,
                0x48000000,
                0x48000400,
                0x48000014,
                0x48000414,
                0x48000010,
                0x48000410,
            )
        ]
        ccer, mode_a, mode_b, odr_a, odr_b, idr_a, idr_b = values
        modes = (mode_a & 0x3FC000, mode_b & 15)
        if ccer & 0x555 or modes not in ((0, 0), (0x3FC000, 15), (0x154000, 5)):
            raise RuntimeError("STOP/power disable not verified")
        if modes == (0x154000, 5) and ((odr_a | idr_a) & 0x780 or (odr_b | idr_b) & 3):
            raise RuntimeError("Loader phase outputs are not low")
        rows.append((ccer & 0x555, *modes))
    if rows[0] != rows[1]:
        raise RuntimeError("Power state changed during preflight")
    return rows


def program_probe(probe, parts, uid, out, run=False):
    """只操作已匹配UID且停机的板；失败不复位，备份及后置回读均保存。"""
    actual_uid = struct.pack("<3I", *probe.memory_read32(0x1FFF7590, 3))
    if actual_uid != uid:
        raise RuntimeError("Selected MCU UID mismatch; no writes performed")
    if (
        probe.memory_read16(0x1FFF75E0, 1)[0] != 128
        or probe.memory_read32(0xE0042000, 1)[0] & 0xFFF != 0x468
    ):
        raise RuntimeError("Target is not the expected 128 KiB STM32G431")
    power = check_power_off(probe)
    probe.halt()
    before = bytes(probe.memory_read8(FLASH_BASE, FLASH_BYTES))
    (out / "flash_before.bin").write_bytes(before)
    if len(before) != FLASH_BYTES or before != bytes(
        probe.memory_read8(FLASH_BASE, FLASH_BYTES)
    ):
        raise RuntimeError("Full Flash backup is inconsistent")
    (out / "parameters_before.bin").write_bytes(before[PARAM_BASE - FLASH_BASE :])

    def write_verified(base, data):
        probe.flash_write8(base, list(data))
        if bytes(probe.memory_read8(base, len(data))) != data:
            raise RuntimeError(f"Program readback mismatch at {base:#x}")

    # 先撤销旧有效记录；Loader和APP回读正确后，最后写有效记录。
    write_verified(RECORD_BASE, b"\xff" * PAGE_BYTES)
    for index in (0, 2):
        write_verified(*parts[index])
    pending = bytearray(before)
    pending[LOADER_BYTES : LOADER_BYTES + PAGE_BYTES] = b"\xff" * PAGE_BYTES
    for index in (0, 2):
        base, data = parts[index]
        pending[base - FLASH_BASE : base - FLASH_BASE + len(data)] = data
    intermediate = bytes(probe.memory_read8(FLASH_BASE, FLASH_BYTES))
    (out / "flash_before_record.bin").write_bytes(intermediate)
    if intermediate != bytes(pending) or intermediate != bytes(
        probe.memory_read8(FLASH_BASE, FLASH_BYTES)
    ):
        raise RuntimeError("Pre-commit Flash mismatch; boot record remains invalid")
    write_verified(*parts[1])
    after = bytes(probe.memory_read8(FLASH_BASE, FLASH_BYTES))
    (out / "flash_after.bin").write_bytes(after)
    expected = bytearray(before)
    for base, data in parts:
        expected[base - FLASH_BASE : base - FLASH_BASE + len(data)] = data
    if after != bytes(expected) or after != bytes(
        probe.memory_read8(FLASH_BASE, FLASH_BYTES)
    ):
        raise RuntimeError(
            "Full Flash differs from expected; protected region/readback failure"
        )
    report = {
        "status": "PASS",
        "uid": uid.hex(),
        "power_preflight": power,
        "written_bytes": sum(len(data) for _, data in parts) + PAGE_BYTES,
        "parameters_unchanged": True,
        "spare_page_unchanged": True,
        "parameter_sha256": sha256(after[PARAM_BASE - FLASH_BASE :]),
        "flash_sha256": sha256(after),
        "cpu_running": False,
    }
    if run:
        try:
            probe.reset(halt=False)
            time.sleep(1)
            if probe.halted():
                raise RuntimeError("CPU did not start after verified programming")
            check_power_off(probe)
        except BaseException as exc:
            # 仅在已匹配目标且本脚本启动后执行紧急关相；禁止失败后自动恢复。
            try:
                probe.halt()
                for address, mask, value in (
                    (0x40012C20, 0x555, 0),
                    (0x40012C44, 0x8000, 0),
                    (0x48000000, 0x3FC000, 0x3FC000),
                    (0x48000400, 15, 15),
                ):
                    current = probe.memory_read32(address, 1)[0]
                    probe.memory_write32(address, [(current & ~mask) | value])
                check_power_off(probe)
            except Exception as cleanup_error:
                raise RuntimeError(
                    f"Startup failed: {exc}; disable failed: {cleanup_error}; physical intervention required"
                ) from exc
            raise
        report["cpu_running"] = True
    return report


def flash_package(args):
    """显式烧录入口；预检失败不加载DLL，异常关闭探针并保留原错误。"""
    hex_path = getattr(args, "hex", None)
    parts, manifest = load_hex(hex_path) if hex_path else load_package(args.package)
    expected_hash = getattr(args, "expected_hex_sha256", None)
    if expected_hash and manifest.get("hex_sha256") != expected_hash:
        raise ValueError("所选固件已变化，请重新选择固件")
    uid = bytes.fromhex(args.uid)
    if len(uid) != 12 or args.jlink_serial <= 0 or not 1 <= args.speed_khz <= 4000:
        raise ValueError("Invalid UID/probe serial/SWD speed")
    args.out.mkdir(parents=True, exist_ok=False)
    if args.pylink_dir:
        sys.path.insert(0, str(args.pylink_dir))
    import pylink  # 默认合包及所有离线测试均不访问硬件。

    library = (
        pylink.library.Library(dllpath=str(args.jlink_dll)) if args.jlink_dll else None
    )
    probe = pylink.JLink(lib=library)
    opened = False
    report = {
        "status": "FAIL",
        "uid": args.uid,
        "jlink_serial": args.jlink_serial,
        "firmware_source_sha256": sha256(
            (hex_path or args.package / "manifest.json").read_bytes()
        ),
        "app_sha256": manifest["app"]["image_sha256"],
        "speed_khz": args.speed_khz,
    }
    if hex_path:
        report["firmware_source_sha256"] = manifest["hex_sha256"]
    else:
        report["package_manifest_sha256"] = report["firmware_source_sha256"]
    failure = None
    started = time.monotonic()
    try:
        probe.open(serial_no=args.jlink_serial)
        opened = True
        probe.exec_command("SetRestartOnClose = 0")
        probe.set_tif(pylink.enums.JLinkInterfaces.SWD)
        probe.connect("STM32G431CB", speed=args.speed_khz)
        report.update(program_probe(probe, parts, uid, args.out, args.run))
    except BaseException as exc:
        failure = exc
        report["error"] = str(exc)
    finally:
        if opened:
            if failure is None and report.get("cpu_running"):
                try:
                    # J-Link关闭可重新暂停目标；仅已通过启动检查的--run保留运行态。
                    probe.exec_command("SetRestartOnClose = 1")
                except Exception as exc:
                    report["status"] = "FAIL"
                    report["restart_policy_error"] = str(exc)
                    failure = exc
            try:
                probe.close()  # 默认/失败保持暂停；成功的--run保持已验证运行态。
            except Exception as exc:
                report["status"] = "FAIL"
                report["close_error"] = str(exc)
                if failure is None:
                    failure = exc
        report["elapsed_seconds"] = round(time.monotonic() - started, 3)
        (args.out / "summary.json").write_text(
            json.dumps(report, indent=2) + "\n", encoding="utf-8"
        )
    if failure:
        raise failure
    return report


def main():
    """合包与烧录分成显式子命令，默认不接触任何设备。"""
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    pack = commands.add_parser("pack", help="生成初装HEX及清单，离线执行")
    pack.add_argument("--loader-bin", type=Path, required=True)
    app_input = pack.add_mutually_exclusive_group(required=True)
    app_input.add_argument("--app-package", type=Path)
    app_input.add_argument("--app-bin", type=Path)
    pack.add_argument(
        "--app-version",
        type=lambda value: int(value, 0),
        help="原始APP BIN必填，编码0xMMmmppbb，与已构建APP一致",
    )
    pack.add_argument("--out", type=Path, required=True)
    flash = commands.add_parser(
        "flash", help="通过显式选定的J-Link烧录，备份并验证参数保持"
    )
    firmware_input = flash.add_mutually_exclusive_group(required=True)
    firmware_input.add_argument("--package", type=Path)
    firmware_input.add_argument("--hex", type=Path, help="当前布局的完整APP+Loader HEX")
    flash.add_argument("--jlink-serial", type=int, required=True)
    flash.add_argument("--uid", required=True)
    flash.add_argument("--jlink-dll", type=Path)
    flash.add_argument("--pylink-dir", type=Path)
    flash.add_argument("--speed-khz", type=int, default=100)
    flash.add_argument("--out", type=Path, required=True)
    flash.add_argument(
        "--run", action="store_true", help="全区验证成功后复位运行；默认保持暂停"
    )
    args = parser.parse_args()
    if args.command == "pack" and (
        (args.app_bin is not None) != (args.app_version is not None)
    ):
        parser.error(
            "--app-bin requires --app-version; --app-package already contains the version"
        )
    result = (
        build_package(
            args.loader_bin, args.app_package, args.out, args.app_bin, args.app_version
        )
        if args.command == "pack"
        else flash_package(args)
    )
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
