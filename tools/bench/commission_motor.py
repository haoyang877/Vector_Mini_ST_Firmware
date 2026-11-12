"""AT32F456 电机工装：按轴烧录工厂包、生成本机参数并执行角度标定。"""

import argparse
import ctypes
import hashlib
import json
import math
import os
import struct
import subprocess
import sys
import time
import zlib
from datetime import datetime
from pathlib import Path
from tempfile import TemporaryDirectory

sys.path.insert(0, str(Path(__file__).resolve().parent))
import install_axis_board as board
from _calibration_curves import export_curves
from motor_calibration_can import JOB as JOB
from motor_calibration_can import STATUS as STATUS
from motor_calibration_can import idle, run_job
from ports.controlcanfd import ControlCanFd
from upgrade_client import UpgradeClient

VERSION = "1.4.0"
PARAMETER_VERSION = "1.2.0"
TOOL_NAME = f"YG-Motor-Commission-v{VERSION}"
GENERATOR_NAME = f"axis-record-v{PARAMETER_VERSION}.exe"
GEARED = (6, 7, 8)


def verify_kit(kit):
    manifest = json.loads((kit / "manifest.json").read_text(encoding="utf-8"))
    if manifest["version"] != VERSION or manifest["parameter_version"] != PARAMETER_VERSION:
        raise ValueError("交付清单与工装版本不符")
    for name, digest in manifest["files"].items():
        path = (kit / name).resolve()
        if not path.is_relative_to(kit) or hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            raise ValueError(f"交付文件校验失败 {name}")


def parameter_filename(axis, stage="preset", version=PARAMETER_VERSION):
    return f"YG-Params-{axis.name}-{axis.model}-0x{axis.node:02X}-v{version}-{stage}.bin"


def default_speed(axis):
    return 0.1 if axis.name == "YAW" else axis.speed


def new_record(kit, axis, speed, thermal):
    if not all(math.isfinite(v) for v in [speed, *thermal]) or speed <= 0:
        raise ValueError("限速须为正值，热参数须为有限值")
    with TemporaryDirectory() as temp:
        path = Path(temp) / "axis.bin"
        command = [
            str(kit / "bin" / GENERATOR_NAME),
            str(axis.node),
            "default",
            "default",
            str(speed),
            "1",
            path.name,
            "1",
            axis.model,
            *map(str, thermal),
        ]
        subprocess.run(command, cwd=temp, check=True)
        raw = path.read_bytes()
    info = board.maintenance.decode_parameters(raw.ljust(16384, b"\xff"))
    if (
        len(raw) != 2288
        or info["node"] != axis.node
        or info["configuration_flags"] != 1
        or info["encoder_reverse"] != 0
        or info["calibration_flag"]
    ):
        raise ValueError("新配参记录回读不符")
    return raw


def saved_parameters(data, axis):
    info = board.maintenance.decode_parameters(data)
    if info["node"] != axis.node or struct.unpack_from("<I", data, 2216)[0] != axis.joint:
        raise ValueError("板上节点或轴角色与所选电机不符")
    if info["encoder_reverse"] != 0:
        raise ValueError("工装只接受同向编码器，板上 encoder_reverse 必须为 0")
    if info["configuration_flags"] != 1:
        raise ValueError("板上型号尚未确认，请先完成实物配参")
    return info


def check_calibration(data, axis):
    info = saved_parameters(data, axis)
    if info["calibration_flag"] & 3 != 3:
        raise ValueError("Flash 缺少编码器 LUT 或电气零位")
    if axis.node in GEARED:
        magic, motor_crc, ratio, _, _, direction, status, lut = struct.unpack_from(
            "<IIfffiII", data, 2288
        )
        if (
            magic != 0x334E454F
            or motor_crc != struct.unpack_from("<I", data, 2140)[0]
            or zlib.crc32(data[2288:4368]) != struct.unpack_from("<I", data, 4368)[0]
            or ratio != 40
            or direction not in (-1, 1)
            or status & 0xFF != 128
            or lut != 128
            or struct.unpack_from("<I", data, 2204)[0] != 0x48410180
        ):
            raise ValueError("输出轴 Hall 标定记录或 CRC 不符")
        info["output_direction"] = direction
    return {key: val for key, val in info.items() if key not in ("lut_q15", "motor")}


def calibrate(bus, probe, bundle, axis, uid, run):
    client = UpgradeClient(bus, axis.node, uid)
    client.sequence = int(time.monotonic() * 1000) & 0xFFFF
    _, identity, _ = client.identify()
    descriptor = board.maintenance.target_value(bundle.target, "TARGET_DESCRIPTOR_OFFSET")
    expected_crc = struct.unpack_from(">I", bundle.image, descriptor + 48)[0]
    if struct.unpack_from("<I", identity, 24)[0] != expected_crc:
        raise ValueError("CAN 在线镜像与所选工厂包 CRC 不同")
    if identity[22:24] != bytes((1, 1)):
        raise ValueError("CAN 目标未运行已确认 APP")
    data = board.maintenance.read_twice(probe, board.PARAM, 16384)
    saved_parameters(data, axis)
    (run / parameter_filename(axis, "before")).write_bytes(data)
    board.verify_boot(
        probe,
        bundle.fields,
        board.APP,
        len(bundle.debug),
        axis.node,
        axis.joint,
        axis.model,
        struct.unpack_from("<f", data, 2152)[0],
    )
    board.maintenance.check_outputs_off(probe, bundle.target)
    tickets = []
    for kind in (1,) if axis.node in GEARED else (1, 2):
        tickets.append(run_job(client, kind))
        idle(client)
    board.maintenance.check_outputs_off(probe, bundle.target)
    board.maintenance.check_identity(probe, bundle.target, uid)
    data = board.maintenance.read_twice(probe, board.PARAM, 16384)
    result = check_calibration(data, axis)
    (run / parameter_filename(axis, "calibrated")).write_bytes(data)
    return dict(
        tickets=tickets,
        idle=True,
        outputs_off=True,
        before_parameters=parameter_filename(axis, "before"),
        saved_parameters=parameter_filename(axis, "calibrated"),
        **result,
    )


def operate(args, kit, bundle, axis, operation, record):
    with board.connected_probe(bundle.target, args.probe, args.jlink_dll) as probe:
        uid = board.maintenance.read_twice(probe, 0x1FFFF7E8, 12)
        if uid in (bytes(12), b"\xff" * 12):
            raise ValueError("无效芯片 UID")
        board.maintenance.check_identity(probe, bundle.target, uid)
        run = board.record_path(args.out) / uid.hex() / datetime.now().strftime("%Y%m%d-%H%M%S-%f")
        run.mkdir(parents=True)
        report = dict(
            version=VERSION,
            parameter_version=PARAMETER_VERSION,
            uid=uid.hex(),
            model=axis.model,
            node=axis.node,
            operation=operation,
            bundle_sha256=bundle.sha256,
            success=False,
        )
        print(f"{axis.name} · {axis.model} · UID {uid.hex()}\n记录目录 {run}", flush=True)
        try:
            if operation != "calibrate":
                board.process_board(
                    probe,
                    bundle,
                    axis,
                    record,
                    args.out / "install",
                    False,
                    record_name=parameter_filename(axis, "install"),
                )
            if operation != "install":
                if (
                    board.maintenance.read_twice(probe, board.APP, len(bundle.debug))
                    != bundle.debug
                ):
                    raise ValueError("板上 APP 与本交付包不匹配，请选择对应版本的工装")
                dll = (args.dll or kit / "bin/ControlCANFD.dll").resolve()
                with (
                    os.add_dll_directory(str(dll.parent)),
                    ControlCanFd(dll, channel_index=args.channel - 1) as bus,
                ):
                    report["calibration"] = calibrate(bus, probe, bundle, axis, uid, run)
            if operation != "install":
                cal = report["calibration"]
                print(f"标定成功 · {axis.name} · {axis.model} · UID {uid.hex()}", flush=True)
                print(f"Flash 已保存并回读，CRC32 {cal['record_crc32']}，空闲、零故障、六路关断。")
                print(f"电气零位 {cal['electrical_zero_q15']} Q15，电机编码器同向。")
                if "output_direction" in cal:
                    print(f"输出轴 Hall 方向 {cal['output_direction']:+d}")
                print(f"标定参数 {run / cal['saved_parameters']}", flush=True)
                record_path = run / cal["saved_parameters"]
                try:
                    curves = export_curves(
                        record_path.read_bytes(), axis, axis.node in GEARED, uid, run
                    )
                except Exception as exc:
                    raise RuntimeError(f"标定参数已保存，曲线导出失败：{exc}") from exc
                cal["curves"] = curves
                spans = curves["peak_to_peak_deg"]
                print(f"电机轴补偿峰峰值 {spans['motor']:.4f}°")
                if "hall" in spans:
                    print(f"输出轴 Hall 补偿峰峰值 {spans['hall']:.4f}°")
                print(f"角度曲线 {run / curves['png']}\n曲线数据 {run / curves['csv']}", flush=True)
            report["success"] = True
        except BaseException as exc:
            report["error"] = str(exc) or type(exc).__name__
            raise
        finally:
            (run / "result.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(f"执行结果 {run / 'result.json'}", flush=True)


def configure(args, kit, axis, interactive):
    preset = (kit / "parameters" / parameter_filename(axis)).read_bytes()
    if (
        len(preset) != 2288
        or saved_parameters(preset.ljust(16384, b"\xff"), axis)["calibration_flag"]
    ):
        raise ValueError("初装预设无效")
    # schema14 的七个连续 float 与记录生成器的物理单位和顺序相同。
    thermal = struct.unpack_from("<7f", preset, 2240)
    speed = args.speed
    if interactive and speed is None:
        while True:
            value = input(
                f"输出轴限速 rad/s（最高约 {axis.speed:.6g}，回车 {default_speed(axis):g}）："
            ).strip()
            try:
                speed = float(value) if value else default_speed(axis)
                if 0 < speed <= axis.speed:
                    break
            except ValueError:
                pass
            print(f"{axis.name} 的输出轴限速须大于 0 且不超过 {axis.speed:g} rad/s。")
    if speed is None:
        speed = default_speed(axis)
    if not 0 < speed <= axis.speed:
        raise ValueError(f"{axis.name} 的输出轴限速须大于 0 且不超过 {axis.speed:g} rad/s")
    if speed == default_speed(axis) and not args.thermal:
        return preset
    return new_record(kit, axis, speed, args.thermal or thermal)


def export_saved_curves(result_path):
    """按已完成标定的设备和摘要重建曲线，保留原始运行报告。"""
    result_path = board.record_path(result_path)
    report = json.loads(result_path.read_text(encoding="utf-8"))
    cal = report.get("calibration") or {}
    if not cal.get("tickets") or not cal.get("idle") or not cal.get("outputs_off"):
        raise ValueError("记录中缺少已完成的标定及停机回读")
    axis = board.parse_axis(str(report["node"]))
    uid = bytes.fromhex(report["uid"])
    if (
        len(uid) != 12
        or report["model"] != axis.model
        or report["parameter_version"] not in ("1.0.0", "1.1.0", PARAMETER_VERSION)
    ):
        raise ValueError("标定记录的设备或参数版本不匹配")
    run = result_path.resolve().parent
    record = (
        run / parameter_filename(axis, "calibrated", report["parameter_version"])
    ).read_bytes()
    if hashlib.sha256(record).hexdigest() != cal["sha256"]:
        raise ValueError("参数文件与本次标定回读摘要不符")
    check_calibration(record, axis)
    curves = export_curves(record, axis, axis.node in GEARED, uid, run)
    print(f"已导出角度曲线 {run / curves['png']}\n曲线数据 {run / curves['csv']}", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--version", action="version", version=VERSION)
    parser.add_argument(
        "--kit", type=Path, default=Path(sys.executable).parent, help="交付目录，默认 EXE 所在目录"
    )
    parser.add_argument(
        "--export-curves",
        type=Path,
        metavar="RESULT.JSON",
        help="从已保存的标定结果离线导出角度曲线",
    )
    parser.add_argument("--check", action="store_true", help="离线核验大包和六轴参数生成")
    parser.add_argument("--operation", choices=("full", "install", "calibrate"))
    parser.add_argument("--axis", help="left/right/roll/pitch/yaw 或节点地址")
    parser.add_argument("--direction", choices=("same",), help=argparse.SUPPRESS)
    parser.add_argument("--speed", type=float, help="输出轴速度上限 rad/s")
    heat = parser.add_mutually_exclusive_group()
    heat.add_argument("--thermal", type=float, nargs=7, help=argparse.SUPPRESS)
    heat.add_argument("--bench-thermal", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--channel", type=int, choices=(1, 2), default=1)
    parser.add_argument("--dll", type=Path, help="覆盖 USB-CAN FD SDK")
    parser.add_argument("--probe", type=int)
    parser.add_argument("--jlink-dll", type=Path)
    parser.add_argument("--out", type=Path, help="运行记录目录，默认交付包旁的 records")
    args = parser.parse_args()
    kit = args.kit.resolve()
    args.out = args.out or kit / "records"
    verify_kit(kit)
    if args.export_curves:
        if args.operation or args.axis or args.check:
            parser.error("--export-curves 单独使用")
        export_saved_curves(args.export_curves)
        return
    bundle = board.load_factory(kit / "firmware")
    print(f"YG 电机工装 v{VERSION}  |  AT32F456  |  FULL {bundle.sha256[:12]}", flush=True)
    if args.check:
        import pylink

        if not callable(pylink.JLink):
            raise RuntimeError("J-Link 接口库加载失败")
        with os.add_dll_directory(str(kit / "bin")):
            sdk = ctypes.WinDLL(str(kit / "bin/ControlCANFD.dll"))
            for symbol in ("ZCAN_OpenDevice", "ZCAN_TransmitFD", "ZCAN_ReceiveFD"):
                getattr(sdk, symbol)
        for axis in board.AXES:
            expected = new_record(
                kit, axis, default_speed(axis), board.BENCH_THERMAL if axis.node in GEARED else ()
            )
            if (kit / "parameters" / parameter_filename(axis)).read_bytes() != expected:
                raise ValueError(f"{axis.name} 随包参数与生成结果不符")
        print(f"离线自检通过：FULL、配套 AXF 和六轴同向参数 v{PARAMETER_VERSION}。")
        return
    interactive = not (args.operation or args.axis)
    if not interactive and not (args.operation and args.axis):
        parser.error("单板命令须同时指定 --operation 和 --axis")
    while True:
        operation = args.operation
        if interactive:
            selection = input(
                "\n1 烧录 + 配参 + 标定    2 仅烧录配参    3 已配板重新标定    回车退出\n请选择："
            ).strip()
            if not selection:
                return
            operation = {"1": "full", "2": "install", "3": "calibrate"}.get(selection)
            if not operation:
                continue
        try:
            axis = board.parse_axis(
                args.axis or input("实物轴名（左轮 / 右轮 / ROLL / PITCH / YAW / Lift）：")
            )
            print(f"{axis.name}  节点 0x{axis.node:02X}  型号 {axis.model}", flush=True)
            record = None if operation == "calibrate" else configure(args, kit, axis, interactive)
            if operation == "calibrate" and (
                args.speed is not None or args.thermal or args.bench_thermal
            ):
                raise ValueError("重新标定沿用本机参数，请移除初装参数选项")
            if operation != "install" and not (args.dll or kit / "bin/ControlCANFD.dll").is_file():
                raise FileNotFoundError("缺少 ControlCANFD.dll")
            if interactive and operation != "install":
                if input("标定将旋转电机。输入 Y 开始：").strip().lower() != "y":
                    continue
            operate(args, kit, bundle, axis, operation, record)
        except Exception as error:
            if not interactive:
                raise
            print(f"本板失败：{error}", file=sys.stderr)
        if not interactive:
            return


if __name__ == "__main__":
    main()
