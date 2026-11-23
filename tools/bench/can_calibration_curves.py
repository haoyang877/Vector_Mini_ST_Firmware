"""通过CAN或已选J-Link回读保存记录，核对身份与一致性并导出补偿曲线。"""

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
def calibration_probe(uid: str, probe_serial: int | None = None):
    """串行占用已核对身份、处于关相状态的探针；退出不复位目标。"""
    import pylink

    target = board.maintenance.load_target("yg_esc_v1_at32f456")
    library = pylink.library.Library(dllpath=os.environ.get("JLINK_DLL") or None)
    probe = pylink.JLink(lib=library)
    attached_running = False
    try:
        probes = probe.connected_emulators()
        serials = [item.SerialNumber for item in probes]
        if probe_serial is None and len(serials) == 1:
            probe_serial = serials[0]
        if probe_serial is None:
            raise ValueError("请在整包烧录页选择本次电机的 J-Link，再读取标定曲线")
        if probe_serial not in serials:
            raise ValueError("所选 J-Link 已断开，请重新扫描并选择")
        probe.open(serial_no=probe_serial)
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


def _read_record(uid: str, probe_serial: int | None = None) -> bytes:
    with calibration_probe(uid, probe_serial) as (probe, target):
        record = board.maintenance.read_twice(
            probe, board.maintenance.target_value(target, "TARGET_PARAM_BASE"), 16384
        )
        board.maintenance.check_outputs_off(probe, target)
        return record


def _read_record_can(link: Link, revision: int) -> bytes:
    """按v1分块重组真实保存记录；修订、CRC或元数据变化立即拒绝。"""

    def request(offset: int = 0, count: int = 0, crc: int = 0) -> tuple[tuple, bytes]:
        raw = link.request(
            194, struct.pack("<IIHBB", revision if count else 0, crc, offset, count, 1)
        )
        if len(raw) < 18:
            raise ValueError(f"CAN 标定记录回读被拒绝：{raw.hex()}")
        header = struct.unpack_from("<HHIIHHBB", raw)
        if header[:2] != (0, 0) or header[5:] != (offset, count, 1) or len(raw) != 18 + count:
            raise ValueError("CAN 标定记录回读格式错误")
        return header, raw[18:]

    metadata, _ = request()
    if metadata[2] != revision or metadata[4] != 4372:
        raise ValueError("CAN 标定记录修订或布局不匹配")
    record = bytearray()
    for offset in range(0, metadata[4], 28):
        count = min(28, metadata[4] - offset)
        header, data = request(offset, count, metadata[3])
        if header[2:5] != metadata[2:5]:
            raise ValueError("CAN 标定记录读取期间已改变")
        record.extend(data)
    if request()[0] != metadata or zlib.crc32(record) != metadata[3]:
        raise ValueError("CAN 标定记录完整性校验失败")
    return bytes(record)


def read_committed_calibration_fault(bus: Any, report: dict, out: Path) -> dict:
    """只读回本轮已提交的联合标定失败记录，核对身份、关相和保存修订。"""
    link = Link(bus, report["node"], timeout_s=0.5)

    def saved_revision() -> int:
        identity = link.request(1, page=0)
        status = link.status()
        saved = link.request(123, bytes(4))
        if len(identity) != 32 or identity[:12].hex() != report["uid"]:
            raise ValueError("标定诊断回读时设备身份改变")
        if status[43:] not in (bytes((5, 0)), bytes((8, 0))):
            raise ValueError("标定诊断回读需要板端已关相")
        if len(saved) != 20 or saved[:6] != b"\0\0\0\0\0\x02":
            raise ValueError("标定诊断无法读取保存修订")
        active, revision = struct.unpack_from("<II", saved, 12)
        if not revision or active != revision:
            raise ValueError("标定诊断的活动修订与保存修订不一致")
        return revision

    revision = saved_revision()
    before = report["before_revision"]["saved_revision"]
    if revision <= before:
        raise ValueError("Flash 未出现本次标定的新保存修订")
    record = _read_record_can(link, revision)
    if saved_revision() != revision:
        raise ValueError("标定诊断回读期间保存修订改变")
    info = board.maintenance.decode_parameters(record)
    if info["node"] != report["node"]:
        raise ValueError("标定诊断保存记录的节点不匹配")
    diagnostic = (
        _saved_output_fault(record)
        if report.get("stop_faults") == 0x20
        else _saved_hall_fault(record)
    )
    path = out / "failed-calibration-parameters.bin"
    path.write_bytes(record)
    result = dict(
        diagnostic,
        calibration_flags=info["calibration_flag"],
        saved_revision=revision,
        parameters_bin=path.name,
    )
    if info["calibration_flag"] & 1:
        axis = SimpleNamespace(
            model=report.get("archive", {}).get("name") or "Motor",
            node=report["node"],
        )
        diagnostic_hall = False
        if report.get("stop_faults") == 0x4000:
            try:
                diagnostic_hall = _has_hall_diagnostic_lut(record, result["hall_fault_mask"])
            except (ValueError, struct.error) as error:
                result["hall_diagnostic_error"] = str(error)
        try:
            result["encoder_curves"] = export_curves(
                record,
                axis,
                False,
                bytes.fromhex(report["uid"]),
                out,
                diagnostic_hall=diagnostic_hall,
            )
            if diagnostic_hall:
                result["hall_diagnostic_curve"] = True
        except Exception as error:
            # 绘图失败不影响已核验的板端故障和保存标志。
            result["encoder_curve_error"] = str(error)
    return result


def _has_hall_diagnostic_lut(record: bytes, faults: int) -> bool:
    magic, binding, _, _, _, _, flags, lut_status = struct.unpack_from("<IIfffiII", record, 2288)
    if lut_status != 3:
        return False
    lut = struct.unpack_from("<1024h", record, 2320)
    if (
        magic != 0x334E454F
        or binding != struct.unpack_from("<I", record, 2140)[0]
        or zlib.crc32(record[2288:4368]) != struct.unpack_from("<I", record, 4368)[0]
        or not faults
        or faults & ~0x1E
        or flags & 0xFF != 2
        or any(64 + lut[(i + 1) % 1024] - lut[i] <= 0 for i in range(1024))
    ):
        raise ValueError("Hall 诊断曲线记录的状态、绑定或 CRC 无效")
    return True


def _saved_hall_fault(record: bytes) -> dict:
    tag = struct.unpack_from("<I", record, 2204)[0]
    flags = tag & 0x7F
    if tag & 0xFFFFFF80 != 0x48410100 or not flags:
        raise ValueError("本轮保存记录没有有效的 Hall 失败标志")
    descriptions = (
        "Hall 扫圈无有效样本",
        "Hall sin 最小值过低",
        "Hall sin 最大值过高",
        "Hall cos 最小值过低",
        "Hall cos 最大值过高",
        "Hall sin 幅值不足",
        "Hall cos 幅值不足",
    )
    return dict(
        hall_fault_mask=flags,
        details=[text for bit, text in enumerate(descriptions) if flags & (1 << bit)],
    )


def _saved_output_fault(record: bytes) -> dict:
    magic, binding, _, _, _, _, flags, lut = struct.unpack_from("<IIfffiII", record, 2288)
    if (
        magic != 0x334E454F
        or binding != struct.unpack_from("<I", record, 2140)[0]
        or zlib.crc32(record[2288:4368]) != struct.unpack_from("<I", record, 4368)[0]
    ):
        raise ValueError("输出轴标定诊断的记录绑定或 CRC 无效")
    status = flags & 0xFF
    reason = {
        1: "输出轴与电机轴配对采样不足",
        2: "输出轴 Hall 基础标定未通过",
        3: {
            0: "输出轴减速比校验失败",
            1: "输出轴 Hall 补偿表相位覆盖不足",
            2: "输出轴 Hall 与电机轴角映射或相位误差超限",
        }.get(lut, "输出轴 Hall 与电机轴配对校验失败"),
    }.get(status)
    if not reason:
        raise ValueError("本轮保存记录没有有效的输出轴配对失败标志")
    return dict(output_status=status, hall_lut_status=lut, details=[reason])


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


def export_calibration_curves(
    bus: Any, report: dict, out: Path, model_label: str = "", *, probe_serial: int | None = None
) -> dict:
    """导出本票据保存的LUT；无设备写入，回读前后核对UID、修订和记录CRC。"""
    if not report.get("passed") or not report.get("stop_confirmed"):
        raise ValueError("只有本次标定成功并确认关相后才能绘制")
    link = Link(bus, report["node"], timeout_s=0.5)
    revision = report["saved_revision"]["saved_revision"]
    try:
        crc, model = _saved_snapshot(link, report["uid"], revision)
        caps = link.request(2, page=0)
        if len(caps) != 16:
            raise ValueError("标定回读能力应答无效")
        can_readback = bool(int.from_bytes(caps[:4], "little") & (1 << 24))
        record = (
            _read_record_can(link, revision)
            if can_readback
            else _read_record(report["uid"], probe_serial)
        )
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
            source="本次Flash保存表（CAN只读回读）"
            if can_readback
            else "本次Flash保存表（J-Link只读回读）",
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


def read_saved_calibration(
    bus: Any, node: int, uid: str, out: Path, probe_serial: int | None = None
) -> dict:
    """读取已有保存记录，不启动标定，不把回读结果当作新的标定票据。"""
    link = Link(bus, node, timeout_s=0.5)
    info = link.request(1, page=0)
    saved = link.request(123, bytes(4))
    if len(info) != 32 or info[:12].hex() != uid or len(saved) != 20:
        raise ValueError("保存记录的设备身份或参数回复无效")
    revision = struct.unpack_from("<I", saved, 16)[0]
    if not revision:
        raise ValueError("设备没有可验证的已保存记录")
    report = dict(
        profile="calibration_readback",
        node=node,
        uid=uid,
        passed=True,
        stop_confirmed=True,
        kind=0,
        app_version=struct.unpack_from("<I", info, 20)[0],
        saved_revision={"saved_revision": revision},
    )
    report["curves"] = export_calibration_curves(bus, report, out, probe_serial=probe_serial)
    return report
