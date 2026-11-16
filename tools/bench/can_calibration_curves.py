"""标定保存后核对CAN快照并只读回读Flash，复用工装补偿曲线导出。"""

import hashlib
import json
import math
import os
import re
import struct
import zlib
from contextlib import contextmanager
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import five_axis_jlink as board
from _calibration_curves import export_curves
from hardware_check import Link

ARCHIVE_TYPES = {3: "左轮电机", 4: "右轮电机", 5: "R轴电机", 6: "P轴电机", 7: "Y轴电机"}


def calibration_archive(motor_type: str, specimen_serial: str, test_serial: str) -> dict[str, str]:
    """区分样机与测试编号；保留前导零，不写MCU生产身份。"""
    if motor_type not in ARCHIVE_TYPES.values():
        raise ValueError("请选择五种归档电机类型之一")
    specimen, test = specimen_serial.strip(), test_serial.strip()
    for label, serial in (("样机序列号", specimen), ("测试序列号", test)):
        if not re.fullmatch(r"[A-Za-z0-9_-]{1,32}", serial):
            raise ValueError(f"{label}须为1～32位数字、字母、短横线或下划线")
    return dict(
        type=motor_type, specimen_serial=specimen, test_serial=test, name=f"{motor_type}-{specimen}"
    )


def model_directory_name(label: str) -> str:
    """用户型号仅作为归档名称，不解释为路径，也不写入板端型号。"""
    name = re.sub(r'[<>:"/\\|?*\x00-\x1f]', "_", label.strip()).strip(". ")[:64]
    if not name:
        raise ValueError("请填写归档电机型号")
    if name.split(".", 1)[0].upper() in {
        "CON",
        "PRN",
        "AUX",
        "NUL",
        *(f"COM{i}" for i in range(1, 10)),
        *(f"LPT{i}" for i in range(1, 10)),
    }:
        name = "_" + name
    return name


def _configuration(record: bytes, info: dict) -> dict:
    """按已验证schema14解码持久化配置；完整BIN保留原始浮点及扩展字节。"""
    names = (
        "id_kp",
        "id_ki",
        "iq_kp",
        "iq_ki",
        "speed_acceleration_rad_s2",
        "speed_deceleration_rad_s2",
        "speed_kp",
        "speed_ki",
        "position_acceleration_rad_s2",
        "position_deceleration_rad_s2",
        "position_maximum_speed_rad_s",
    )
    config = {key: value for key, value in info.items() if key != "lut_q15"}
    config["control"] = dict(zip(names, struct.unpack_from("<11f", record, 2092), strict=True))
    config["limits"] = dict(
        zip(
            ("calibration_current_a", "current_limit_a", "speed_limit_rad_s"),
            struct.unpack_from("<3f", record, 2144),
            strict=True,
        )
    )
    config["current_sense_shunt_milliohm"] = struct.unpack_from("<I", record, 2172)[0]
    config["can_bitrate_record"] = struct.unpack_from("<I", record, 2168)[0]
    config["position_gains"] = dict(
        zip(("kp", "kd"), struct.unpack_from("<2f", record, 2180), strict=True)
    )
    config["hall_coefficients"] = dict(
        zip(
            (
                "sin_offset_counts",
                "cos_offset_counts",
                "sin_amplitude_counts",
                "cos_amplitude_counts",
            ),
            struct.unpack_from("<4f", record, 2188),
            strict=True,
        )
    )
    axis = struct.unpack_from("<4I3fI", record, 2208)
    config["axis_profile"] = dict(
        zip(
            (
                "magic",
                "version",
                "joint_type",
                "config_revision",
                "minimum_position_rad",
                "maximum_position_rad",
                "maximum_speed_rad_s",
                "crc32",
            ),
            axis,
            strict=True,
        )
    )
    config["thermal"] = dict(
        zip(
            (
                "continuous_vector_a",
                "peak_a",
                "heating_tau_s",
                "derating_enter_heat",
                "derating_exit_heat",
                "trip_heat",
                "clear_heat",
            ),
            struct.unpack_from("<7f", record, 2240),
            strict=True,
        )
    )
    config["speed_feedforward"] = dict(
        zip(
            (
                "enabled",
                "gain_a_per_rad_s2",
                "acceleration_limit_rad_s2",
                "jerk_limit_rad_s3",
                "current_limit_a",
            ),
            struct.unpack_from("<I4f", record, 2268),
            strict=True,
        )
    )
    if record[2288:2292] == b"OEN3":
        config["output_encoder"] = dict(
            zip(
                (
                    "gear_ratio",
                    "phase_offset_rad",
                    "mechanical_zero_rad",
                    "direction",
                    "status",
                    "lut_status",
                ),
                struct.unpack_from("<3fiII", record, 2296),
                strict=True,
            )
        )
    return config


def _saved_snapshot(link: Link, uid: str, revision: int) -> tuple[int, int]:
    identity = link.request(1, page=0)
    status = link.status()
    saved = link.request(123, bytes(4))
    config = link.request(190, bytes(4))
    if len(identity) != 32 or identity[:12].hex() != uid:
        raise ValueError("曲线回读时芯片UID改变")
    if status[4:8] != bytes(4) or status[43:] != bytes((5, 0)):
        raise ValueError("曲线回读需要关相空闲及零故障")
    if (
        len(saved) != 20
        or saved[:6] != b"\0\0\0\0\0\x02"
        or struct.unpack_from("<II", saved, 12) != (revision, revision)
    ):
        raise ValueError("曲线回读时标定保存修订改变")
    if (
        len(config) != 44
        or config[:5] != b"\0\0\0\0\x01"
        or config[6] != link.node
        or config[5] not in (1, 2, 3)
    ):
        raise ValueError("曲线回读的活动型号配置无效")
    return struct.unpack_from("<I", config, 8)[0], config[5]


@contextmanager
def calibration_probe(uid: str):
    """串行占用已核对身份、处于关相状态的探针；退出不复位目标。"""
    import pylink

    target = board.maintenance.load_target("yg_esc_v1_at32f456")
    library = pylink.library.Library(dllpath=os.environ.get("JLINK_DLL") or None)
    probe = pylink.JLink(lib=library)
    attached_running = False
    try:
        probes = probe.connected_emulators()
        if len(probes) != 1:
            raise ValueError("补偿表回读需要连接本次电机的一只J-Link")
        probe.open(serial_no=probes[0].SerialNumber)
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
        board.maintenance.check_identity(probe, target, bytes.fromhex(uid))
        board.maintenance.check_outputs_off(probe, target)
        if probe.halted():
            raise ValueError("目标CPU已暂停，保留当前状态，不读取本次运行曲线")
        attached_running = True
        yield probe, target
    finally:
        try:
            # 读取连接会启用复位捕获；不能让下一次正常CAN升级停在复位入口。
            if attached_running and not probe.halted():
                board.maintenance.release_reset_catch(probe)
        finally:
            probe.close()


def _read_record(uid: str) -> bytes:
    with calibration_probe(uid) as (probe, target):
        record = board.maintenance.read_twice(
            probe, board.maintenance.target_value(target, "TARGET_PARAM_BASE"), 16384
        )
        board.maintenance.check_outputs_off(probe, target)
        return record


def _validate_record(record: bytes, node: int, crc: int, has_hall: bool) -> dict:
    info = board.maintenance.decode_parameters(record)
    if info["node"] != node or int(info["record_crc32"], 16) != crc:
        raise ValueError("Flash参数与本次CAN节点或保存记录CRC不一致")
    if not info["calibration_flag"] & 1:
        raise ValueError("Flash没有有效编码器LUT，不能绘制补偿曲线")
    if has_hall:
        magic, binding, ratio, _, _, direction, flags, lut_status = struct.unpack_from(
            "<IIfffiII", record, 2288
        )
        lut = struct.unpack_from("<1024h", record, 2320)
        if (
            magic != 0x334E454F
            or binding != crc
            or zlib.crc32(record[2288:4368]) != struct.unpack_from("<I", record, 4368)[0]
            or not math.isfinite(ratio)
            or ratio < 1
            or direction not in (-1, 1)
            or flags & 255 != 128
            or lut_status != 128
            or struct.unpack_from("<I", record, 2204)[0] != 0x48410180
            or any(64 + lut[(i + 1) % 1024] - lut[i] <= 0 for i in range(1024))
        ):
            raise ValueError("Flash Hall标定表、绑定或CRC无效")
    return info


def export_calibration_curves(bus: Any, report: dict, out: Path, model_label: str = "") -> dict:
    """导出本票据保存的LUT；无设备写入，回读前后核对UID、修订和记录CRC。"""
    if not report.get("passed") or not report.get("stop_confirmed"):
        raise ValueError("只有本次标定成功并确认关相后才能绘制")
    link = Link(bus, report["node"], timeout_s=0.5)
    revision = report["saved_revision"]["saved_revision"]
    try:
        crc, model = _saved_snapshot(link, report["uid"], revision)
        record = _read_record(report["uid"])
        info = _validate_record(record, report["node"], crc, model in (2, 3))
        if _saved_snapshot(link, report["uid"], revision) != (crc, model):
            raise ValueError("补偿表回读期间活动配置已改变")
        axis = SimpleNamespace(
            model=model_label or {1: "HT8115-4", 2: "YQ-5708", 3: "YQ-5740"}.get(model, "Motor"),
            node=report["node"],
        )
        if report.get("archive", {}).get("test_serial"):
            axis.model += " · 测试" + report["archive"]["test_serial"]
        curves = export_curves(
            record,
            axis,
            model in (2, 3),
            bytes.fromhex(report["uid"]),
            out,
            raw_capture=report.get("raw_capture"),
        )
        (out / "calibration-parameters.bin").write_bytes(record)
        configuration = dict(
            archive_name=model_label or axis.model,
            archive=report.get("archive"),
            firmware_model={1: "HT8115-4", 2: "YQ-5708", 3: "YQ-5740"}.get(model),
            uid=report["uid"],
            node=report["node"],
            ticket=report.get("ticket"),
            saved_revision=revision,
            app_version=report.get("app_version"),
            parameters=_configuration(record, info),
            calibration_values=curves["calibration_values"],
            raw_capture=report.get("raw_capture"),
        )
        (out / "motor-configuration.json").write_text(
            json.dumps(configuration, ensure_ascii=False, indent=2, allow_nan=False),
            encoding="utf-8",
        )
        curves.update(
            source="本次Flash保存表（J-Link只读回读）",
            calibration_flags=info["calibration_flag"],
            saved_revision=revision,
            record_crc32=info["record_crc32"],
            parameter_sha256=hashlib.sha256(record).hexdigest(),
            electrical_zero_q15=info["electrical_zero_q15"],
            table_updated=report["kind"] == 1,
            configuration_json="motor-configuration.json",
            parameters_bin="calibration-parameters.bin",
        )
        manifest = {
            "model": configuration["archive_name"],
            "archive": report.get("archive"),
            "uid": report["uid"],
            "ticket": report.get("ticket"),
            "files": {},
        }
        names = [
            curves["png"],
            curves["csv"],
            curves["values_csv"],
            curves["configuration_json"],
            curves["parameters_bin"],
        ]
        for name in ("hall-raw.bin", "hall-raw.csv", "hall-raw-receive.csv"):
            if (out / name).exists():
                names.append(name)
        for name in names:
            data = (out / name).read_bytes()
            manifest["files"][name] = {
                "bytes": len(data),
                "sha256": hashlib.sha256(data).hexdigest(),
            }
        (out / "calibration-manifest.json").write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        curves["manifest"] = "calibration-manifest.json"
        return curves
    finally:
        report["curve_read_timeouts"] = link.timeouts
