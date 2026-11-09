"""双板SWD备份、保护区比较及受控初装；默认离线，不增加CAN命令。"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import struct
import sys
import time
import zlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "shared/targets"))
from target_contract import load_target, target_value  # noqa: E402

sys.path.insert(0, str(ROOT / "host_app"))
from upgrade_client import UpgradePackage  # noqa: E402


def digest(data: bytes) -> dict:
    """计算本机证据摘要，不将CRC当作认证。"""
    return {
        "bytes": len(data),
        "crc32": f"{zlib.crc32(data):08x}",
        "sha256": hashlib.sha256(data).hexdigest(),
    }


def read_twice(probe, address: int, size: int) -> bytes:
    """以不超过4KiB的SWD事务读取两份完整固定区；短读或不同即失败，不重试。"""
    if size <= 0 or address < 0 or address > 0x100000000 - size:
        raise ValueError("SWD读取范围无效")

    def read_pass() -> bytes:
        chunks = []
        for offset in range(0, size, 4096):
            count = min(4096, size - offset)
            try:
                block = bytes(probe.memory_read8(address + offset, count))
            except Exception as error:
                raise RuntimeError(f"SWD读取失败：0x{address + offset:08x}，{count}B") from error
            if len(block) != count:
                raise RuntimeError(f"SWD短读：0x{address + offset:08x}")
            chunks.append(block)
        return b"".join(chunks)

    first = read_pass()
    second = read_pass()
    if first != second:
        raise RuntimeError(f"SWD双次回读不一致：0x{address:08x}")
    return first


def check_identity(probe, target: dict, uid: bytes) -> None:
    """写操作前核对真实UID及物理Flash容量，不按枚举顺序换板。"""
    if len(uid) != 12:
        raise ValueError("UID必须为12字节")
    at32 = target["device"].startswith("AT32F456")
    address = 0x1FFFF7E8 if at32 else 0x1FFF7590
    capacity = 0x1FFFF7E0 if at32 else 0x1FFF75E0
    if read_twice(probe, address, 12) != uid:
        raise RuntimeError("实际UID与绑定设备不符")
    if probe.memory_read16(capacity, 1)[0] * 1024 != target_value(target, "TARGET_FLASH_BYTES"):
        raise RuntimeError("实际Flash容量与目标不符")
    if not at32 and probe.memory_read32(0xE0042000, 1)[0] & 0xFFF != 0x468:
        raise RuntimeError("实际芯片不是G431")


def check_outputs_off(probe, target: dict) -> None:
    """两次核对对应板卡的相通道、主使能及六路引脚低电平。"""
    at32 = target["device"].startswith("AT32F456")
    timer, gpio, channel_mask, a_mask, b_mask = (
        (0x40010000, 0x40020000, 0x5550, 0xE00, 0x13)
        if at32
        else (0x40012C00, 0x48000000, 0x555, 0x780, 3)
    )
    previous = None
    for _ in range(2):
        state = (
            probe.memory_read32(timer + 0x20, 1)[0] & channel_mask,
            probe.memory_read32(timer + 0x44, 1)[0] & 0x8000,
            probe.memory_read32(gpio + 0x10, 1)[0] & a_mask,
            probe.memory_read32(gpio + 0x400 + 0x10, 1)[0] & b_mask,
        )
        if any(state) or (previous is not None and previous != state):
            raise RuntimeError("未确认六路输出关闭，拒绝操作")
        previous = state


def capture(probe, target: dict, uid: bytes, directory: Path) -> dict:
    """保持CPU原运行状态，备份完整物理Flash和完整保护区。"""
    directory.mkdir(parents=True, exist_ok=False)
    check_identity(probe, target, uid)
    check_outputs_off(probe, target)
    raw = read_twice(
        probe, target_value(target, "TARGET_FLASH_BASE"), target_value(target, "TARGET_FLASH_BYTES")
    )
    check_outputs_off(probe, target)
    (directory / "flash.bin").write_bytes(raw)
    base = target_value(target, "TARGET_FLASH_BASE")
    parameters = target_value(target, "TARGET_PARAM_BASE") - base
    (directory / "parameters.bin").write_bytes(raw[parameters : parameters + 16384])
    report = {
        "target": target["target"],
        "uid": uid.hex(),
        "flash": digest(raw),
        "parameters": digest(raw[parameters : parameters + 16384]),
        "cpu_state_changed": False,
    }
    (directory / "snapshot.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


def verify_protected(before: bytes, after: bytes, target: dict) -> dict:
    """APP更新只允许APP槽变化；比较Loader、参数及其余物理Flash。"""
    size = target_value(target, "TARGET_FLASH_BYTES")
    if len(before) != size or len(after) != size:
        raise ValueError("必须比较完整物理Flash快照")
    base = target_value(target, "TARGET_FLASH_BASE")
    start = target_value(target, "TARGET_APP_BASE") - base
    end = start + target_value(target, "TARGET_APP_BYTES")
    if before[:start] != after[:start] or before[end:] != after[end:]:
        raise RuntimeError("APP槽以外发生变化：Loader/参数/身份/保留区验证失败")
    return {
        "protected_regions_verified": True,
        "loader": digest(after[:start]),
        "parameters": digest(after[0x1C000:0x20000]),
    }


def validate_parameters(raw: bytes) -> dict:
    """只读解释schema14，不重建默认值或更改任何保存数据。"""
    if len(raw) != 16384:
        raise ValueError("参数必须包含完整16KiB")
    schema, magic = struct.unpack_from("<II", raw, 2160)
    if (schema, magic) != (14, 0x454E4332):
        raise ValueError("仅接受已校验的schema14；未知/空白格式只备份，不迁移")
    crc = zlib.crc32(raw[2144:2288], zlib.crc32(raw[:2140]))
    if crc != struct.unpack_from("<I", raw, 2140)[0]:
        raise ValueError("参数记录CRC损坏")
    node = struct.unpack_from("<f", raw)[0]
    if not math.isfinite(node) or node != int(node) or not 0 <= node < 255:
        raise ValueError("保存的节点无效")
    return {"schema": schema, "node": int(node), "record_crc32": f"{crc:08x}", **digest(raw)}


def decode_parameters(raw: bytes) -> dict:
    """按当前持久化ABI显示保存值和1024点LUT，不宣称当次标定有效。"""
    result = validate_parameters(raw)
    names = (
        "node",
        "current_offset_a",
        "current_offset_b",
        "current_offset_c",
        "pole_pairs",
        "phase_resistance_ohm",
        "ld_h",
        "lq_h",
        "flux_wb",
    )
    result["motor"] = dict(zip(names, struct.unpack_from("<9f", raw), strict=True))
    result["electrical_zero_q15"], result["mechanical_zero_q15"] = struct.unpack_from(
        "<HH", raw, 36
    )
    result["calibration_flag"], result["encoder_reverse"] = struct.unpack_from("<BB", raw, 40)
    result["lut_q15"] = list(struct.unpack_from("<1024h", raw, 44))
    result["configuration_flags"] = struct.unpack_from("<I", raw, 2136)[0]
    return result


def prepare_migration(snapshot: Path, output: Path, source_sha256: str) -> dict:
    """仅生成AT32旧址到新址的原样迁移包；不调用schema转换器。"""
    metadata = json.loads((snapshot / "snapshot.json").read_text(encoding="utf-8"))
    raw = (snapshot / "flash.bin").read_bytes()
    if (
        metadata["target"] != "yg_esc_v1_at32f456"
        or len(raw) != 262144
        or digest(raw)["sha256"] != source_sha256.lower()
    ):
        raise ValueError("迁移必须绑定AT32的完整快照及准确SHA256")
    if metadata["flash"] != digest(raw):
        raise ValueError("快照元数据不匹配")
    # 新描述符表明该板已迁移，不能再覆盖后来保存的新参数。
    if struct.unpack_from(">4I", raw, 0x4200) == (0x59494431, 3, 0x53593549, 7):
        raise ValueError("该板已具备新布局，拒绝重复迁移")
    parameters = raw[0x3C000:0x40000]
    info = validate_parameters(parameters)
    output.mkdir(parents=True, exist_ok=False)
    (output / "parameters.bin").write_bytes(parameters)
    report = {
        "uid": metadata["uid"],
        "target": metadata["target"],
        "source_flash_sha256": source_sha256.lower(),
        "source_address": 0x0803C000,
        "destination_address": 0x0801C000,
        "parameters": info,
    }
    (output / "migration.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


def load_bundle(directory: Path, selection: str) -> tuple[dict, bytes, bytes]:
    """拒绝混配或改动的构建包；返回目标、Loader及未激活APP。"""
    target = load_target(selection)
    report = json.loads((directory / "summary.json").read_text(encoding="utf-8"))
    if not report.get("success") or report.get("target") != target["target"]:
        raise ValueError("构建包未通过或目标不符")
    loader = None
    for item in report["artifacts"]:
        path = (directory / item["path"]).resolve()
        if (
            not path.is_relative_to(directory.resolve())
            or digest(path.read_bytes())["sha256"] != item["sha256"]
        ):
            raise ValueError("构建包文件或哈希无效")
        if path.name.startswith("YG2026-ESC-LOADER-") and path.suffix == ".bin":
            if loader is not None:
                raise ValueError("包中有多个Loader")
            loader = path.read_bytes()
    package = UpgradePackage.load(directory / "app-package", selection)
    if loader is None or not 8 <= len(loader) <= 16384:
        raise ValueError("缺少合法16KiB Loader")
    sp, reset = struct.unpack_from("<II", loader)
    if (
        not target_value(target, "TARGET_RAM_BASE") < sp <= target_value(target, "TARGET_RAM_END")
        or sp % 8
        or not reset & 1
        or not 0x08000000 <= reset & ~1 < 0x08000000 + len(loader)
    ):
        raise ValueError("Loader向量不合法")
    return target, loader, package.image


def install(
    probe,
    target: dict,
    uid: bytes,
    loader: bytes,
    image: bytes,
    output: Path,
    migration: Path | None = None,
) -> dict:
    """受控初装；首次参数迁移显式传包，普通安装不写保护区。"""
    if not 8 <= len(loader) <= 16384 or not 0 < len(image) <= 98304:
        raise ValueError("镜像超过16/96KiB预算")
    offset = target_value(target, "TARGET_DESCRIPTOR_OFFSET")
    if image[offset + 64 : offset + 88] != b"\xff" * 24:
        raise ValueError("初装只接受未激活APP")
    capture(probe, target, uid, output / "before")
    before = (output / "before/flash.bin").read_bytes()
    parameters = None
    at32 = target["target"] == "yg_esc_v1_at32f456"
    new_layout = at32 and struct.unpack_from(">4I", before, 0x4200) == (
        0x59494431,
        3,
        0x53593549,
        7,
    )
    if at32 and not new_layout and migration is None and before[0x3C000:] != b"\xff" * 16384:
        raise ValueError("旧AT32参数区非空：先生成并指定原样迁移包；未知或损坏格式须单独处理")
    if migration is not None:
        plan = json.loads((migration / "migration.json").read_text(encoding="utf-8"))
        parameters = (migration / "parameters.bin").read_bytes()
        if (
            target["target"] != "yg_esc_v1_at32f456"
            or plan["uid"] != uid.hex()
            or plan["source_flash_sha256"] != digest(before)["sha256"]
            or validate_parameters(parameters) != plan["parameters"]
            or parameters != before[0x3C000:0x40000]
        ):
            raise ValueError("迁移包不属于当前设备/当前完整快照")
        if new_layout:
            raise ValueError("已是新布局，拒绝重复迁移")
    check_identity(probe, target, uid)
    check_outputs_off(probe, target)
    # 暂停期间禁止继承WDT复位打断SWD事务；只改调试寄存器，不改选项字节。
    # 保持暂停退出，由Loader下一次正常启动撤销调试冻结。
    frozen = probe.memory_read32(0xE0042008, 1)[0]
    probe.memory_write32(0xE0042008, [frozen | (1 << 12)])
    if not probe.halt():
        raise RuntimeError("探针未确认CPU暂停，拒绝写入Flash")
    check_outputs_off(probe, target)
    # 两款MCU的WDT寄存器偏移相同。Flash算法运行时CPU会暂时离开暂停态，
    # 调试冻结不足以保护原APP的短看门狗；先延长已有WDT，不启动未运行的WDT。
    probe.memory_write32(0x40003000, [0x5555])
    probe.memory_write32(0x40003004, [6])
    probe.memory_write32(0x40003008, [4095])
    probe.memory_write32(0x40003010, [4095])
    deadline = time.monotonic() + 0.1
    while probe.memory_read32(0x4000300C, 1)[0]:
        if time.monotonic() >= deadline:
            raise RuntimeError("WDT配置更新超时，拒绝写入Flash")
        time.sleep(0.001)
    if (
        probe.memory_read32(0x40003004, 1)[0] != 6
        or probe.memory_read32(0x40003008, 1)[0] != 4095
        or probe.memory_read32(0x40003010, 1)[0] != 4095
    ):
        raise RuntimeError("WDT配置回读不符，拒绝写入Flash")
    probe.memory_write32(0x40003000, [0xAAAA])
    expected = bytearray(before)
    padded = image + b"\xff" * (98304 - len(image))
    # 先使APP描述符失效，再安装Loader。全部写入由精确长度限制，不调用整片擦除。
    probe.flash_write8(0x08004000, list(padded))
    expected[0x4000:0x1C000] = padded
    if parameters is not None:
        probe.flash_write8(0x0801C000, list(parameters))
        expected[0x1C000:0x20000] = parameters
    padded_loader = loader + b"\xff" * (16384 - len(loader))
    probe.flash_write8(0x08000000, list(padded_loader))
    expected[:0x4000] = padded_loader
    if read_twice(probe, 0x08000000, len(before)) != bytes(expected):
        raise RuntimeError("初装提交前全片不符，保持暂停，禁止激活")
    # SWD工具可能按页擦写：最后提交完整首APP页，避免单写8B破坏同页向量。
    active = bytearray(padded[:2048])
    struct.pack_into(">II", active, offset + 64, 0x41435433, ~0x41435433 & 0xFFFFFFFF)
    probe.flash_write8(0x08004000, list(active))
    expected[0x4000:0x4800] = active
    after = read_twice(probe, 0x08000000, len(before))
    if after != bytes(expected):
        raise RuntimeError("初装提交后全片不符，保持暂停")
    (output / "flash_after.bin").write_bytes(after)
    return {
        "install_verified": True,
        "uid": uid.hex(),
        "target": target["target"],
        "parameters_migrated": parameters is not None,
        "parameters": digest(after[0x1C000:0x20000]),
        "flash": digest(after),
        "app_confirmed": False,
        "cpu_halted": True,
    }


def main() -> None:
    """默认离线命令；SWD动作必须显式给出目标、探针、UID和输出目录。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "action", choices=("prepare-migration", "parameters", "verify", "backup", "install")
    )
    parser.add_argument("--target", default="vector_mini_st_g431")
    parser.add_argument("--input", type=Path)
    parser.add_argument("--after", type=Path)
    parser.add_argument("--source-sha256")
    parser.add_argument("--out", type=Path)
    parser.add_argument("--bundle", type=Path)
    parser.add_argument("--migration", type=Path)
    parser.add_argument("--probe", type=int)
    parser.add_argument("--uid")
    parser.add_argument("--jlink-dll", type=Path)
    parser.add_argument("--exclusive-bench", "--exclusive-no-power", action="store_true")
    args = parser.parse_args()
    target = load_target(args.target)
    if args.action == "prepare-migration":
        if not args.input or not args.out or not args.source_sha256:
            parser.error("迁移需要快照目录、SHA256和新输出目录")
        report = prepare_migration(args.input, args.out, args.source_sha256)
    elif args.action == "parameters":
        if not args.input:
            parser.error("请选择完整16KiB参数快照")
        report = decode_parameters(args.input.read_bytes())
    elif args.action == "verify":
        if not args.input or not args.after:
            parser.error("请选择下载前后两份完整Flash快照")
        report = verify_protected(args.input.read_bytes(), args.after.read_bytes(), target)
    else:
        if (
            not args.exclusive_bench
            or not args.probe
            or args.probe <= 0
            or not args.uid
            or not args.out
        ):
            parser.error("SWD操作需要明确探针、UID、停机且独占台架和输出目录")
        if args.action == "install" and not args.bundle:
            parser.error("初装需要已通过校验的配套构建包")
        uid = bytes.fromhex(args.uid)
        bundle = load_bundle(args.bundle, args.target) if args.action == "install" else None
        import pylink  # 离线命令不加载DLL，更不打开设备。

        library = pylink.library.Library(dllpath=str(args.jlink_dll)) if args.jlink_dll else None
        probe = pylink.JLink(lib=library)
        report = {"success": False}
        args.out.mkdir(parents=True, exist_ok=False)
        try:
            probe.open(serial_no=args.probe)
            # 回读必须来自目标Flash，不能把DLL缓存命中当作第二次物理校验。
            # 退出不改CPU运行/暂停状态，也不停止APP拥有的DWT时基。
            for command in (
                "SetAllowFlashCache = 0",
                "SetEnableMemCache = 0",
                "SetSkipDebugDeInit = 1",
                "SetDbgPowerDownOnClose = 0",
                "SetRestartOnClose = 0",
            ):
                probe.exec_command(command)
            probe.set_tif(pylink.enums.JLinkInterfaces.SWD)
            probe.connect(target["build"]["jlink_device"], speed=1000)
            report = (
                capture(probe, target, uid, args.out / "snapshot")
                if bundle is None
                else install(probe, target, uid, bundle[1], bundle[2], args.out, args.migration)
            )
            report["success"] = True
        except Exception as error:
            report["error"] = str(error)
            raise
        finally:
            try:
                probe.close()
            except Exception as error:
                report.update(success=False, close_error=str(error))
                raise
            finally:
                (args.out / "summary.json").write_text(
                    json.dumps(report, indent=2) + "\n", encoding="utf-8"
                )
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
