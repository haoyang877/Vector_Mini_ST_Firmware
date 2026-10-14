"""维护页后端：现有CAN升级和只读Flash快照；导入不打开设备、不发运动命令。"""

import hashlib
import json
import math
from pathlib import Path
import secrets
import struct
import time
import zlib

from loader_factory import FLASH_BASE, FLASH_BYTES, PARAM_BASE, boot_record, load_hex
from loader_updater import (
    APP_BASE,
    CAPACITY,
    HARDWARE_ID,
    LAYOUT_ID,
    PARAM_ABI,
    Client,
    LoaderBus,
    app_parameter_crc,
    app_query,
    app_ready,
    validate_image,
)


def version_text(code: int) -> str:
    """显示0xMMmmppbb版本编码。"""
    text = ".".join(str((code >> shift) & 255) for shift in (24, 16, 8))
    return text + (f" (build {code & 255})" if code & 255 else "")


def load_upgrade(path: Path) -> tuple:
    """预检完整HEX或APP清单与BIN，返回镜像、清单及绑定全部输入的哈希。"""
    if path.suffix.lower() == ".hex":
        parts, manifest = load_hex(path)
        app = manifest["app"]
        image = next(data for base, data in parts if base == APP_BASE)
        return image[: app["image_bytes"]], app, manifest["hex_sha256"]
    if path.suffix.lower() != ".json" or path.stat().st_size > 65536:
        raise ValueError("请选择完整合并HEX或APP包的manifest.json")
    source = path.read_bytes()
    manifest = json.loads(source)
    app = manifest.get("app", manifest)
    bin_path = path.parent / "app.bin"
    if bin_path.stat().st_size > CAPACITY:
        raise ValueError("APP超过升级分区容量")
    image = bin_path.read_bytes()
    validate_image(image)
    expected = {
        "app_base": APP_BASE,
        "layout_id": LAYOUT_ID,
        "hardware_id": HARDWARE_ID,
        "param_abi": PARAM_ABI,
        "protocol_version": 2,
        "image_bytes": len(image),
        "image_crc32": zlib.crc32(image),
        "image_sha256": hashlib.sha256(image).hexdigest(),
    }
    if any(app.get(key) != value for key, value in expected.items()):
        raise ValueError("APP清单、哈希或布局不匹配")
    if (
        type(app.get("app_version")) is not int
        or not 0 <= app["app_version"] <= 0xFFFFFFFF
    ):
        raise ValueError("APP版本无效")
    return image, app, hashlib.sha256(source + image).hexdigest()


def app_value(bus, channel: int, node: int, op: int, fmt: str = ">f"):
    """串行读取当前协议参数；无序号的旧查询不作原子实时快照保证。"""
    size = struct.calcsize(fmt)
    reply = app_query(bus, channel, node, op, bytes(4), lambda data: len(data) == size)
    value = struct.unpack(fmt, reply)[0]
    if not math.isfinite(value):
        raise ValueError(f"参数{op:#x}为非有限值")
    return value


def app_uid(bus, channel: int, node: int) -> bytes:
    """用随机nonce关联当前节点UID，查询不改变模式。"""
    nonce = secrets.token_bytes(4)
    reply = app_query(
        bus,
        channel,
        node,
        0x63,
        nonce,
        lambda data: len(data) == 16 and data[:4] == nonce,
    )
    return reply[4:]


def query_device(bus, channel: int, node: int) -> dict:
    """识别APP或Loader；查询失败不发送BOOT、不复位、不停止电机。"""
    if not 0 <= node <= 7 or channel not in (0, 1):
        raise ValueError("CAN节点或通道超出范围")
    try:
        uid = app_uid(bus, channel, node)
    except TimeoutError:
        client = Client(bus, channel, node)
        uid = client.query(3, 16)
        version = client.check_target(uid)
        state = client.query(2, 24)
        stage, error = state[:2]
        offset, size, image_crc, session = struct.unpack(">4I", state[4:])
        return {
            "runtime": "Loader",
            "uid": uid.hex(),
            "node": node,
            "loader_version": version,
            "parameter_crc": client.parameter_crc,
            "loader_state": stage,
            "loader_error": error,
            "session": session,
            "rows": [
                ("设备", "UID", uid.hex(), ""),
                ("设备", "CAN节点", node, ""),
                ("固件", "当前运行", "Loader", ""),
                ("固件", "Loader版本", version_text(version), ""),
                ("升级", "状态 / 错误", f"{stage} / {error}", ""),
                ("升级", "已接收 / 镜像大小", f"{offset} / {size}", "B"),
                ("升级", "会话", f"0x{session:08X}", ""),
                ("升级", "镜像CRC", f"0x{image_crc:08X}", ""),
                ("参数", "全参数CRC", f"0x{client.parameter_crc:08X}", ""),
            ],
        }
    revision = app_value(bus, channel, node, 0x67)
    if revision != 2:
        raise ValueError("APP参数协议版本不是2，不按当前速度/位置编码解读")
    version = app_value(bus, channel, node, 0x6F, ">I")
    mode = app_value(bus, channel, node, 0x01)
    crc = app_parameter_crc(bus, channel, node)
    rows = [
        ("设备", "UID", uid.hex(), ""),
        ("设备", "CAN节点", node, ""),
        ("固件", "当前运行", "APP", ""),
        ("固件", "APP版本", version_text(version), ""),
        ("固件", "Loader版本", "未读取（当前运行APP）", ""),
        ("运行", "模式", mode, ""),
        ("参数", "全参数CRC", f"0x{crc:08X}", ""),
    ]
    warnings = []
    for label, op, fmt, scale, unit in (
        ("错误码", 0x4D, ">f", 1, ""),
        ("母线电压", 0x2D, ">f", 1, "V"),
        ("温度", 0x43, ">f", 1, "℃"),
        ("实际位置", 0x41, ">i", 180 / math.pi / 1000, "°"),
        ("实际速度", 0x3F, ">i", 180 / math.pi / 100, "°/s"),
        ("Iq", 0x39, ">h", 0.001, "A"),
        ("编码器在线", 0x0D, ">f", 1, ""),
    ):
        try:
            value = app_value(bus, channel, node, op, fmt)
            if fmt in (">i", ">h") and value == -(1 << (8 * struct.calcsize(fmt) - 1)):
                raise ValueError("固件返回无效哨兵")
            rows.append(("运行", label, value * scale, unit))
        except (TimeoutError, ValueError) as exc:
            rows.append(("运行", label, "未读取", unit))
            warnings.append(str(exc))
    if app_uid(bus, channel, node) != uid:
        raise RuntimeError("查询期间节点UID改变或重复节点冲突")
    return {
        "runtime": "APP",
        "uid": uid.hex(),
        "node": node,
        "app_version": version,
        "mode": mode,
        "parameter_crc": crc,
        "rows": rows,
        "warnings": warnings,
    }


def can_snapshot(settings: dict) -> dict:
    """打开用户选定的单个适配器和通道，读取后释放，不扫描其他节点。"""
    bus = LoaderBus(
        Path(settings["dll"]),
        (settings["channel"],),
        settings["index"],
        settings.get("serial"),
    )
    try:
        report = query_device(bus, settings["channel"], settings["node"])
        report["adapter"] = bus.info
        return report
    finally:
        bus.close()


def run_can_upgrade(
    settings: dict,
    binding: dict,
    path: Path,
    expected_hash: str,
    out: Path,
    progress=None,
) -> dict:
    """升级同一已绑定UID；停机、镜像、参数CRC任一核对失败即中止并留证。"""
    image, manifest, digest = load_upgrade(path)
    if digest != expected_hash:
        raise ValueError("选择后的升级文件已改变，请重新选择")
    out.mkdir(parents=True, exist_ok=False)
    report = {
        "status": "FAIL",
        "uid": binding["uid"],
        "node": settings["node"],
        "channel": settings["channel"],
        "image_sha256": manifest["image_sha256"],
    }
    started = time.monotonic()
    bus, client = None, None
    failure = None
    try:
        bus = LoaderBus(
            Path(settings["dll"]),
            (settings["channel"],),
            settings["index"],
            binding["adapter"]["serial"],
        )
        current = query_device(bus, settings["channel"], settings["node"])
        if current["uid"] != binding["uid"]:
            raise RuntimeError("节点UID已改变，未开始升级")
        baseline = current["parameter_crc"]
        uid = bytes.fromhex(binding["uid"])
        if current["runtime"] == "APP":
            if current["mode"] != 0:
                raise RuntimeError("电机尚未停机，请停机后重新读取设备")
            app_query(
                bus,
                settings["channel"],
                settings["node"],
                0x66,
                b"BOOT",
                lambda data: data == bytes(4),
            )
            time.sleep(0.1)
        elif current["session"] or current["loader_state"] != 0:
            raise RuntimeError("Loader存在升级会话，请等待会话结束后重新读取")
        client = Client(bus, settings["channel"], settings["node"])
        client.check_target(uid)
        if client.parameter_crc != baseline:
            raise RuntimeError("进入Loader后参数CRC改变，未擦写APP")
        if progress:
            progress(0, "已核对目标，正在升级APP")
            original = client.request

            def request(op, payload=b"", **kwargs):
                reply = original(op, payload, **kwargs)
                if op == 0x22:
                    completed = struct.unpack(">I", reply)[0]
                    progress(
                        completed / len(image) * 90,
                        f"已传输 {completed}/{len(image)} B",
                    )
                return reply

            client.request = request
        report.update(client.update(image, manifest["app_version"], uid))
        app_ready(
            bus, settings["channel"], settings["node"], manifest["app_version"], uid
        )
        after = app_parameter_crc(bus, settings["channel"], settings["node"])
        report.update(parameter_crc_before=baseline, parameter_crc_after=after)
        if after != baseline:
            raise RuntimeError("升级后参数CRC改变")
        if app_value(bus, settings["channel"], settings["node"], 1) != 0:
            raise RuntimeError("升级后STOP未确认，停止后续操作并现场检查")
        report.update(status="PASS", app_version=manifest["app_version"])
        if progress:
            progress(100, "版本、UID、STOP和全参数CRC核对通过")
    except BaseException as exc:
        failure = exc
        report["error"] = str(exc)
    finally:
        if bus is not None:
            try:
                bus.close()
            except Exception as exc:
                report.update(status="FAIL", close_error=str(exc))
                failure = failure or exc
        report["elapsed_seconds"] = round(time.monotonic() - started, 3)
        report["requests"] = client.trace if client else []
        (out / "summary.json").write_text(
            json.dumps(report, indent=2) + "\n", encoding="utf-8"
        )
    if failure:
        raise failure
    return report


def decode_calibration(data: bytes) -> dict:
    """按当前真实schema10固定ABI解析保存数据；未知格式不猜偏移、不生成默认标定。"""
    if len(data) != 0x4000:
        raise ValueError("需要完整16 KiB参数区")
    result = {
        "rows": [],
        "lut_deg": [],
        "parameter_sha256": hashlib.sha256(data).hexdigest(),
    }
    rows = result["rows"]
    if data == bytes(len(data)) or data == b"\xff" * len(data):
        result["state"] = "未配置"
        rows.append(("存储", "参数区", "空白，尚无保存的标定参数", ""))
        return result
    schema, magic = struct.unpack_from("<II", data, 2160)
    if magic != 0x454E4332 or schema != 10:
        raise ValueError(
            f"参数格式不支持（schema={schema}, magic={magic:#x}），保留原始备份"
        )
    result.update(state="已读取保存值", schema=schema)
    rows.extend(
        [
            ("存储", "参数schema", schema, ""),
            ("存储", "校验范围", "基础区无整体CRC；独立尾记录逐项校验", ""),
        ]
    )

    def floating(offset):
        value = struct.unpack_from("<f", data, offset)[0]
        if not math.isfinite(value):
            raise ValueError(f"参数偏移{offset}为非有限值")
        return value

    for label, offset, unit in (
        ("CAN节点", 0, ""),
        ("极对数", 16, ""),
        ("相电阻", 20, "Ω"),
        ("Ld", 24, "H"),
        ("Lq", 28, "H"),
        ("磁链", 32, "Wb"),
    ):
        rows.append(("电机", label, floating(offset), unit))
    for label, offset in (("A相零偏", 4), ("B相零偏", 8), ("C相零偏", 12)):
        rows.append(("电流采样", label, floating(offset), "ADC count"))
    flags, reverse = data[40:42]
    if reverse > 1 or flags & ~7:
        raise ValueError("编码器方向或标定标志损坏")
    result["flags"] = flags
    rows.append(("编码器", "保存的标定标志", f"0x{flags:02X}", ""))
    for label, bit in (("线性化", 1), ("电零位", 2), ("机械零位", 4)):
        rows.append(
            ("编码器", label + "保存标志", "已置位" if flags & bit else "未置位", "")
        )
    rows.append(("编码器", "方向反转", "是" if reverse else "否", ""))
    for label, offset in (("电零位", 36), ("机械零位", 38)):
        value = struct.unpack_from("<H", data, offset)[0]
        rows.extend(
            [
                ("编码器", label, value * 360 / 65536, "°"),
                ("编码器", label + "原始值", value, "Q15 count"),
            ]
        )
    lut = list(struct.unpack_from("<1024h", data, 44))
    result["lut_deg"] = [value * 360 / 65536 for value in lut]
    rows.extend(
        [
            ("编码器", "线性化LUT点数", 1024, ""),
            (
                "编码器",
                "LUT最小 / 最大",
                f"{min(result['lut_deg']):.5f} / {max(result['lut_deg']):.5f}",
                "°",
            ),
        ]
    )
    for label, offset in (("标定电流", 2144), ("运行电流限制", 2148)):
        rows.append(("限制", label, floating(offset), "A"))
    rows.append(("电流采样", "采样电阻", struct.unpack_from("<I", data, 2172)[0], "mΩ"))
    for label, offset, unit in (
        ("正向库仑摩擦", 2188, "A"),
        ("反向库仑摩擦", 2192, "A"),
        ("正向粘性摩擦", 2196, "A/(°/s)"),
        ("反向粘性摩擦", 2200, "A/(°/s)"),
    ):
        value = floating(offset)
        rows.append(
            (
                "摩擦模型",
                label,
                value * math.pi / 180 if offset >= 2196 else value,
                unit,
            )
        )
    valid = struct.unpack_from("<I", data, 2204)[0]
    rows.append(
        (
            "摩擦模型",
            "模型保存标志",
            "已置位" if valid == 1 else "未置位" if valid == 0 else "非法",
            "",
        )
    )

    def tail(offset, length, tag, version):
        raw = data[offset : offset + length]
        if raw in (bytes(length), b"\xff" * length):
            return None, "未保存"
        if (
            raw[:4] != tag
            or struct.unpack_from("<I", raw, 4)[0] != version
            or zlib.crc32(raw[:-4]) != struct.unpack_from("<I", raw, length - 4)[0]
        ):
            return None, "格式 / CRC无效"
        return raw, "CRC有效"

    axis_tag = data[2208:2212]
    axis, state = tail(2208, 32, axis_tag, 2 if axis_tag == b"AXS2" else 1)
    if axis_tag not in (b"AXS1", b"AXS2") and axis is not None:
        axis, state = None, "未知轴格式"
    rows.append(("身份 / 范围", "轴记录", state, ""))
    if axis:
        bounds = struct.unpack_from("<3f", axis, 16)
        if not all(math.isfinite(value) for value in bounds):
            rows.append(("身份 / 范围", "轴记录数值", "非有限值，拒绝解释", ""))
            axis = None
    if axis:
        names = {
            0: "unknown",
            1: "roll",
            2: "pitch",
            3: "yaw",
            4: "wheel_right",
            5: "wheel_left",
        }
        name = (
            axis[8:16].rstrip(b"\0").decode("ascii", "replace")
            if axis_tag == b"AXS1"
            else names.get(struct.unpack_from("<I", axis, 8)[0], "未知")
        )
        rows.append(("身份 / 范围", "电机类型", name, ""))
        for label, value in zip(
            ("最小位置", "最大位置", "巡航速度限制"),
            struct.unpack_from("<3f", axis, 16),
        ):
            rows.append(
                (
                    "身份 / 范围",
                    label,
                    math.degrees(value),
                    "°/s" if "速度" in label else "°",
                )
            )
    load, state = tail(2240, 16, b"DMP1", 1)
    rows.append(("摩擦模型", "阻尼 / 前馈记录", state, ""))
    if load:
        value = struct.unpack_from("<I", load, 8)[0]
        if value not in (0, 1, 3):
            rows.append(("摩擦模型", "装配配置", "非法", ""))
        else:
            rows.extend(
                [
                    ("摩擦模型", "阻尼环", "有" if value & 1 else "无", ""),
                    ("摩擦模型", "摩擦前馈", "开启" if value & 2 else "关闭", ""),
                ]
            )
    filt, state = tail(2256, 16, b"PVF1", 1)
    rows.append(("控制参数", "速度滤波记录", state, ""))
    if filt:
        cutoff = struct.unpack_from("<I", filt, 8)[0]
        rows.append(
            (
                "控制参数",
                "位置速度滤波",
                5 if cutoff == 1 else 10 if cutoff == 0 else "非法",
                "Hz",
            )
        )
    for label, offset, unit in (
        ("Id Kp", 2092, ""),
        ("Id Ki", 2096, ""),
        ("Iq Kp", 2100, ""),
        ("Iq Ki", 2104, ""),
        ("转速 Kp", 2116, ""),
        ("转速 Ki", 2120, ""),
        ("位置 Kp", 2136, ""),
        ("位置 Kd", 2140, ""),
        ("位置 Ki", 2168, ""),
    ):
        rows.append(("控制参数", label, floating(offset), unit))
    pi, state = tail(2272, 20, b"PSP1", 1)
    rows.append(("控制参数", "位置内环PI记录", state, ""))
    if pi:
        for label, value in zip(
            ("位置内环速度 Kp", "位置内环速度 Ki"), struct.unpack_from("<2f", pi, 8)
        ):
            rows.append(("控制参数", label, value, ""))
    return result


def flash_snapshot(dll: Path, serial: int, speed_khz: int, out: Path) -> dict:
    """指定探针只读完整Flash两次，保留CPU状态；保存原始数据后解析，失败不写MCU。"""
    import pylink

    out.mkdir(parents=True, exist_ok=False)
    probe = pylink.JLink(lib=pylink.library.Library(dllpath=str(dll)))
    opened, running = False, False
    report = {"status": "FAIL", "read_only": True, "jlink_serial": serial}
    try:
        probe.open(serial_no=serial)
        opened = True
        probe.exec_command("SetRestartOnClose = 0")
        probe.set_tif(pylink.enums.JLinkInterfaces.SWD)
        probe.connect("STM32G431CB", speed=speed_khz)
        running = not probe.halted()
        if (
            probe.memory_read16(0x1FFF75E0, 1)[0] != 128
            or probe.memory_read32(0xE0042000, 1)[0] & 0xFFF != 0x468
        ):
            raise ValueError("目标不是128 KiB STM32G431")
        report.update(
            uid=struct.pack("<3I", *probe.memory_read32(0x1FFF7590, 3)).hex(),
            cpu_running=running,
        )
        data = bytes(probe.memory_read8(FLASH_BASE, FLASH_BYTES))
        (out / "flash_read.bin").write_bytes(data)
        parameters = data[PARAM_BASE - FLASH_BASE :]
        (out / "parameters_read.bin").write_bytes(parameters)
        if bytes(probe.memory_read8(FLASH_BASE, FLASH_BYTES)) != data:
            raise RuntimeError("连续两次Flash读取不一致，请暂停外部参数保存后再读")
        rows = [
            ("设备", "UID", report["uid"], ""),
            ("运行", "CPU", "运行" if running else "暂停", ""),
        ]
        vtor = probe.memory_read32(0xE000ED08, 1)[0]
        cfsr, hfsr = probe.memory_read32(0xE000ED28, 2)
        rows.extend(
            [
                ("运行", "向量表", f"0x{vtor:08X}", ""),
                ("运行", "CFSR / HFSR", f"0x{cfsr:08X} / 0x{hfsr:08X}", ""),
            ]
        )
        record = data[0x3000:0x3030]
        size, _, version = struct.unpack_from("<3I", record, 12)
        valid = False
        if 8 <= size <= CAPACITY and size % 8 == 0:
            image = data[0x4000 : 0x4000 + size]
            try:
                validate_image(image)
                valid = record == boot_record(image, version)
            except ValueError:
                pass
        rows.append(
            (
                "固件",
                "Flash APP版本",
                version_text(version) if valid else "无有效APP提交记录",
                "",
            )
        )
        rows.append(("参数", "全参数CRC", f"0x{zlib.crc32(parameters):08X}", ""))
        try:
            calibration = decode_calibration(parameters)
        except ValueError as exc:
            calibration = {
                "state": "格式不支持 / 损坏",
                "rows": [("存储", "解析结果", str(exc), "")],
                "lut_deg": [],
            }
        for group, label, value, unit in calibration["rows"]:
            if label in ("CAN节点", "电机类型", "保存的标定标志"):
                rows.append((group, label + "（Flash）", value, unit))
        report.update(
            status="PASS",
            rows=rows,
            calibration=calibration,
            flash_sha256=hashlib.sha256(data).hexdigest(),
        )
        return report
    except BaseException as exc:
        report.update(status="FAIL", error=str(exc))
        raise
    finally:
        try:
            if opened:
                try:
                    probe.exec_command(
                        "SetRestartOnClose = " + ("1" if running else "0")
                    )
                finally:
                    probe.close()
        except BaseException as exc:
            report.update(status="FAIL", cleanup_error=str(exc))
            raise
        finally:
            (out / "summary.json").write_text(
                json.dumps(report, indent=2) + "\n", encoding="utf-8"
            )
