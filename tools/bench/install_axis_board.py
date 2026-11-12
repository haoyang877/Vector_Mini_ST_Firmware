"""通过 J-Link 将同一 AT32 工厂大包和六轴预设装入升级测试空板。"""

import argparse
import json
import os
import struct
import subprocess
import sys
import time
from contextlib import contextmanager
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from tempfile import TemporaryDirectory

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "loader/tools"))
sys.path.insert(0, str(ROOT / "tools/build"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import device_maintenance as maintenance  # noqa: E402
from _axis_boot import boot_layout, verify_boot  # noqa: E402
from keil_loader_app import MARKERS, image_segment, intel_hex  # noqa: E402


@dataclass(frozen=True)
class Axis:
    name: str
    node: int
    joint: int
    model: str
    speed: float


AXES = (
    Axis("左轮", 3, 5, "HT8115-4", 20),
    Axis("右轮", 4, 4, "HT8115-4", 20),
    Axis("ROLL", 5, 1, "HT8115-4", 3.14159265),
    Axis("PITCH", 6, 2, "YQ-5740", 30.0),
    Axis("YAW", 7, 3, "YQ-5708", 2.0943951),
    Axis("Lift", 8, 6, "YQ-5740", 30.0),
)
# 仅为空板生成未确认型号记录。沿用 PITCH 空板台架的热配置占位值。
BENCH_THERMAL = (3, 3, 3600, 0.8, 0.7, 1, 0.5)
BASE, APP, PARAM = 0x08000000, 0x08004000, 0x0801C000


def record_path(path):
    """Windows 记录树可超过 MAX_PATH；保留原目录和安装摘要以支持中断续写。"""
    if os.name != "nt":
        return path
    absolute = os.path.abspath(path)
    if absolute.startswith(("\\\\?\\", "\\\\.\\")):
        return Path(absolute)
    if absolute.startswith("\\\\"):
        return Path("\\\\?\\UNC\\" + absolute[2:])
    return Path("\\\\?\\" + absolute)


def parse_axis(value):
    for axis, english in zip(AXES, ("left", "right", "roll", "pitch", "yaw", "lift"), strict=True):
        if value.strip().lower() in {
            axis.name.lower(),
            english,
            str(axis.node),
            f"0x{axis.node:02x}",
            f"0x{axis.node:x}",
        }:
            return axis
    raise ValueError("可输入 左轮 / 右轮 / ROLL / PITCH / YAW / Lift 或 0x03 至 0x08")


@dataclass
class Bundle:
    target: dict
    loader: bytes
    image: bytes
    debug: bytes
    fields: dict
    sha256: str


def load_factory(path):
    """核对所选 FULL 文件及实际写入的两个段，启动回读只采用配套 AXF。"""
    path = path.expanduser().resolve()
    folder = path if path.is_dir() else path.parent
    target, loader, image = maintenance.load_bundle(folder, "yg_esc_v1_at32f456")
    summary = json.loads((folder / "summary.json").read_text(encoding="utf-8"))
    files = [folder / item["path"] for item in summary["artifacts"]]
    (full,) = (file for file in files if file.match("YG2026-ESC-FULL-*.hex"))
    (axf,) = (file for file in files if file.match("YG2026-ESC-APP-*.axf"))
    if path not in (folder, full.resolve()):
        raise ValueError("请选择工厂包目录或该目录清单内的 FULL HEX")
    _, debug = image_segment(axf.read_bytes(), APP, PARAM - APP)
    expected = bytearray(image)
    offset = maintenance.target_value(target, "TARGET_DESCRIPTOR_OFFSET")
    for delta, magic in MARKERS:
        struct.pack_into(">II", expected, offset + delta, magic, (~magic) & 0xFFFFFFFF)
    combined = intel_hex(BASE, loader).removesuffix(b":00000001FF\n") + intel_hex(APP, debug)
    if debug != expected or full.read_bytes() != combined:
        raise ValueError("FULL HEX、调试 AXF 与安装包不一致")
    return Bundle(
        target, loader, image, debug, boot_layout(axf), maintenance.digest(combined)["sha256"]
    )


def make_record(axis, folder, speed, thermal):
    output = folder / "parameters.bin"
    args = [
        sys.executable,
        str(ROOT / "tools/build/make_axis_record.py"),
        "--node",
        str(axis.node),
        "--model",
        axis.model,
        "--output-shaft",
        "--speed",
        str(axis.speed if speed is None else speed),
        "--output",
        str(output),
    ]
    if thermal is not None or axis.node in (6, 7, 8):
        values = thermal if thermal is not None else BENCH_THERMAL
        print(f"热参数 {values}" + ("（升级测试空板占位）" if thermal is None else ""))
        args += ["--thermal", *map(str, values)]
    subprocess.run(args, check=True)
    raw = output.read_bytes()
    info = maintenance.validate_parameters(raw.ljust(16384, b"\xff"))
    if (
        len(raw) != 2288
        or info["node"] != axis.node
        or raw[40]
        or struct.unpack_from("<I", raw, 2216)[0] != axis.joint
    ):
        raise ValueError("生成的节点、角色或标定状态不符")
    return raw


def blank_outputs(probe, target):
    """按 AT32 LoaderBoard_Safe 的电平设置空片的六个桥臂引脚。"""

    def update(addr, mask, value):
        current = probe.memory_read32(addr, 1)[0]
        probe.memory_write32(addr, [(current & ~mask) | value])

    if (
        probe.memory_read32(0x40010020, 1)[0] & 0x5550
        or probe.memory_read32(0x40010044, 1)[0] & 0x8000
    ):
        raise RuntimeError("桥臂定时器未关闭")
    if not probe.halt():
        raise RuntimeError("空板未暂停")
    update(0x40023830, 3, 3)
    for base, pins, pairs, out in (
        (0x40020000, 0xE00, 0xFC0000, 0x540000),
        (0x40020400, 0x13, 0x30F, 0x105),
    ):
        probe.memory_write32(base + 0x28, [pins])
        update(base + 4, pins, 0)
        update(base + 8, pairs, out)
        update(base + 12, pairs, 0)
        update(base, pairs, out)
        if probe.memory_read32(base, 1)[0] & pairs != out:
            raise RuntimeError("空板 GPIO 输出模式回读异常")
    maintenance.check_outputs_off(probe, target)


def process_board(probe, bundle, axis, record, root, verify_only, *, record_name="parameters.bin"):
    uid = maintenance.read_twice(probe, 0x1FFFF7E8, 12)
    if uid in (bytes(12), b"\xff" * 12):
        raise ValueError("芯片 UID 无效")
    maintenance.check_identity(probe, bundle.target, uid)
    blank = b"\xff" * maintenance.physical_flash_bytes(bundle.target)
    before = maintenance.read_twice(probe, BASE, len(blank))
    expected = bytearray(blank)
    expected[: len(bundle.loader)] = bundle.loader
    expected[APP - BASE : APP - BASE + len(bundle.debug)] = bundle.debug
    expected[PARAM - BASE : PARAM - BASE + len(record)] = record
    digest = maintenance.digest(expected)["sha256"]
    plan = record_path(root) / uid.hex() / digest
    plan.mkdir(parents=True, exist_ok=True)
    backup = plan / "original.bin"
    report = {
        "uid": uid.hex(),
        "node": axis.node,
        "joint": axis.joint,
        "model": axis.model,
        "bundle_sha256": bundle.sha256,
        "expected_flash_sha256": digest,
        "operation": "verify" if verify_only else "install",
        "success": False,
    }
    run = plan / datetime.now().strftime("%Y%m%d-%H%M%S-%f")
    run.mkdir()
    (run / record_name).write_bytes(record)
    print(f"物理板 {uid.hex()}  {axis.name}  0x{axis.node:02X}", flush=True)
    try:
        if before != expected:
            if verify_only:
                raise ValueError("Flash 与所选大包及轴参数不符")
            if before == blank:
                if not backup.exists():
                    staged = run / "blank-flash.tmp"
                    staged.write_bytes(before)
                    staged.replace(backup)
            elif not backup.is_file() or any(
                a & b != b for a, b in zip(before, expected, strict=True)
            ):
                raise ValueError("此板已有不同内容。请核对板号或使用维护工具备份和迁移")
            if backup.read_bytes() != blank:
                raise ValueError("空板备份不完整")
            maintenance.check_identity(probe, bundle.target, uid)
            if before == blank:
                blank_outputs(probe, bundle.target)
            else:
                try:
                    maintenance.check_outputs_off(probe, bundle.target)
                except RuntimeError:
                    # 写入中断后空片可能再次复位，引脚回到浮空态。
                    blank_outputs(probe, bundle.target)
            maintenance.install(probe, bundle.target, uid, bundle.loader, bundle.image, run)
            maintenance.check_identity(probe, bundle.target, uid)
            maintenance.check_outputs_off(probe, bundle.target)
            if not probe.halted():
                raise RuntimeError("写入参数前 CPU 未暂停")
            probe.flash_write8(PARAM, list(record))
            page = maintenance.target_value(bundle.target, "TARGET_ERASE_BYTES")
            probe.flash_write8(APP, list(bundle.debug[:page]))
            if maintenance.read_twice(probe, BASE, len(blank)) != expected:
                raise RuntimeError("写后全片回读不同，保持暂停")
            report["flash_written"] = True
        else:
            report["flash_written"] = False
        if not verify_only and (
            report["flash_written"]
            or probe.halted()
            or probe.memory_read32(0xE000ED08, 1)[0] != APP
        ):
            maintenance.check_outputs_off(probe, bundle.target)
            maintenance.release_reset_catch(probe)
            if probe.halted():
                probe.reset(halt=True)
                maintenance.release_reset_catch(probe)
                probe.restart()
                time.sleep(0.3)
            probe.memory_write32(0xE000ED0C, [0x05FA0004])
            time.sleep(0.2)
        speed = struct.unpack_from("<f", record, 2152)[0]
        verify_boot(
            probe, bundle.fields, APP, len(bundle.debug), axis.node, axis.joint, axis.model, speed
        )
        maintenance.check_outputs_off(probe, bundle.target)
        maintenance.check_identity(probe, bundle.target, uid)
        report.update(
            success=True,
            app_running=True,
            active_speed_rad_s=speed,
            calibration_flag=0,
            six_outputs_off=True,
        )
    except BaseException as error:
        report["error"] = str(error)
        raise
    finally:
        (run / "summary.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(f"记录 {run / 'summary.json'}", flush=True)
    print(f"通过  {uid.hex()}  {axis.name}  0x{axis.node:02X}  全片一致，APP 配置生效")


@contextmanager
def connected_probe(target, serial, dll):
    import pylink

    library = pylink.library.Library(dllpath=str(dll)) if dll else None
    probe = pylink.JLink(lib=library)
    serials = [int(item.SerialNumber) for item in probe.connected_emulators()]
    if serial is None and len(serials) != 1:
        raise ValueError(f"检测到探针 {serials}，请连接一只或通过 --probe 指定")
    serial = serial if serial is not None else serials[0]
    if serial not in serials:
        raise ValueError(f"未连接指定探针 {serial}")
    probe.open(serial_no=serial)
    try:
        for command in (
            "SetAllowFlashCache = 0",
            "SetEnableMemCache = 0",
            "SetSkipDebugDeInit = 1",
            "SetDbgPowerDownOnClose = 0",
            "SetRestartOnClose = 0",
        ):
            probe.exec_command(command)
        probe.set_tif(pylink.enums.JLinkInterfaces.SWD)
        probe.connect(target["build"]["jlink_device"], speed=4000)
        print(f"SWD {probe.speed} kHz", flush=True)
        yield probe
    finally:
        probe.close()


def run_axis(args, bundle, axis):
    with TemporaryDirectory() as temp:
        record = make_record(axis, Path(temp), args.speed, args.thermal)
    with connected_probe(bundle.target, args.probe, args.jlink_dll) as probe:
        process_board(probe, bundle, axis, record, args.out, args.verify_only)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", type=Path, help="构建包目录或包内 FULL HEX，省略则提示输入")
    parser.add_argument("--axis", help="单板执行；省略则逐板输入轴名或节点")
    parser.add_argument("--verify-only", action="store_true", help="只读验证已装板")
    parser.add_argument("--speed", type=float, help="输出轴速度上限 rad/s，单板可选")
    parser.add_argument(
        "--thermal", type=float, nargs=7, help="单板热参数，顺序同 make_axis_record"
    )
    parser.add_argument("--probe", type=int)
    parser.add_argument("--jlink-dll", type=Path)
    parser.add_argument("--out", type=Path, default=ROOT / "outputs/axis-board-install")
    args = parser.parse_args()
    if not args.axis and (args.speed is not None or args.thermal is not None):
        parser.error("定制参数请用 --axis 指定所属板")
    path = args.bundle or Path(input("输入工厂包目录或 FULL HEX 路径：").strip().strip('"'))
    bundle = load_factory(path)
    print(f"AT32F456 完整包 SHA-256 {bundle.sha256}", flush=True)
    if args.axis:
        run_axis(args, bundle, parse_axis(args.axis))
        return
    while True:
        value = input(
            "换好板后输入 左轮 / 右轮 / ROLL / PITCH / YAW / Lift 或节点，回车退出："
        ).strip()
        if not value:
            return
        try:
            run_axis(args, bundle, parse_axis(value))
        except Exception as error:
            print(f"本板失败：{error}", file=sys.stderr)


if __name__ == "__main__":
    main()
