"""五轴 AT32F456 的 SWD 识别、备份、合并包写入与回读。"""

import hashlib
import io
import json
import os
import struct
import sys
import time
import zlib
from pathlib import Path

from five_axis_can_stability import batch_versions, controlcan_dll_path, query_restarted_motor

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "loader/tools"))
import device_maintenance as maintenance  # noqa: E402

TARGET = "yg_esc_v1_at32f456"
FIRST_FIT_AXES = {
    3: ("左轮", "HT8115-4", 5),
    4: ("右轮", "HT8115-4", 4),
    5: ("ROLL", "HT8115-4", 1),
    6: ("PITCH", "YQ-5740", 2),
    7: ("YAW", "YQ-5708", 3),
}


def check_axis_record(raw, node):
    """校验 schema14 中的 AXS2 角色和独立 CRC。"""
    if (
        node not in FIRST_FIT_AXES
        or struct.unpack_from("<4I", raw, 2208) != (0x32535841, 2, FIRST_FIT_AXES[node][2], 2)
        or zlib.crc32(raw[2208:2236]) != struct.unpack_from("<I", raw, 2236)[0]
    ):
        raise ValueError("轴角色、修订或 CRC 与所选节点不符")


def initial_record(resource, node):
    path = resource / "first-fit" / f"{node}.bin"
    record = path.read_bytes()
    if (
        len(record) != 2288
        or record[40]
        or any(record[44:2092])
        or struct.unpack_from("<I", record, 2136)[0] != 1
    ):
        raise ValueError("初装需要未标定的 schema14 基础参数")
    if maintenance.validate_parameters(record.ljust(16384, b"\xff"))["node"] != node:
        raise ValueError("初装参数的节点与所选型号不符")
    check_axis_record(record, node)
    return record


def firmware_segments(raw: bytes, expected_hash: str, target: dict) -> list[tuple[int, bytes]]:
    """只接受本批 HEX，返回两个分区的加载字节，并检查实际擦除页边界。"""
    from intelhex import IntelHex

    if hashlib.sha256(raw).hexdigest() != expected_hash:
        raise ValueError("HEX 的 SHA-256 与本批五轴固件不符")
    image = IntelHex(io.StringIO(raw.decode("ascii")))
    base, app, param, page = (
        maintenance.target_value(target, key)
        for key in (
            "TARGET_FLASH_BASE",
            "TARGET_APP_BASE",
            "TARGET_PARAM_BASE",
            "TARGET_ERASE_BYTES",
        )
    )
    ranges = image.segments()
    if len(ranges) != 2 or image.start_addr is not None:
        raise ValueError("需要两个独立的 Loader 和 APP 加载段")
    for (start, end), origin, limit in zip(ranges, (base, app), (app, param), strict=True):
        if start != origin or start % page or (end + page - 1) // page * page > limit:
            raise ValueError("HEX 的地址或擦除页超出 Loader/APP 分区")
    return [(start, image.tobinstr(start=start, end=end - 1)) for start, end in ranges]


def identify_motor(probe, target, motors, serial, log_root, hex_hash):
    """用硅片 UID 识别板卡，再区分无参数、初装中断和已配置三种情况。"""
    uid = maintenance.read_twice(probe, 0x1FFFF7E8, 12)
    if uid in (bytes(12), b"\xff" * 12):
        raise ValueError("芯片 UID 无效")
    maintenance.check_identity(probe, target, uid)
    param = maintenance.target_value(target, "TARGET_PARAM_BASE")
    size = maintenance.target_value(target, "TARGET_PARAM_BYTES")
    raw = maintenance.read_twice(probe, param, size)
    identity = {"serial": serial, "uid": uid.hex(), "node": None}
    identity_offset = maintenance.target_value(target, "TARGET_IDENTITY_BASE") - param
    blank = raw[:identity_offset] == b"\xff" * identity_offset
    if blank or uid.hex() not in motors:
        # AT32F45x RM Rev 2.00, DEBUG_IDCODE (PID), table on p. 744.
        if probe.memory_read32(0xE0042000, 1)[0] != 0x70053253:
            raise ValueError("芯片型号与 AT32F456CCU7 不符")
    pending = log_root / f"jlink-{uid.hex()}-{hex_hash}.pending"
    prior = json.loads(pending.read_text(encoding="utf-8")) if pending.exists() else {}
    if prior.get("first_fit_node") is not None:
        return {**identity, "resume_node": prior["first_fit_node"]}
    if blank:
        if prior:
            raise ValueError("已有板升级中断且原参数丢失，请从原始备份恢复")
        return identity
    node = maintenance.validate_parameters(raw)["node"]
    if node not in FIRST_FIT_AXES:
        raise ValueError(f"Flash 中的节点 {node} 不属于本批五种电机")
    receipt = log_root / f"first-fit-{uid.hex()}.json"
    registered = (
        json.loads(receipt.read_text(encoding="utf-8"))["node"]
        if receipt.exists()
        else motors.get(uid.hex())
    )
    if registered is not None and registered != node:
        raise ValueError(f"UID 对应节点 {registered}，Flash 参数内的节点为 {node}")
    if motors.get(uid.hex()) != node:
        check_axis_record(raw, node)
        descriptor = maintenance.target_value(target, "TARGET_APP_BASE") + maintenance.target_value(
            target, "TARGET_DESCRIPTOR_OFFSET"
        )
        if not prior and struct.unpack(">4I", maintenance.read_twice(probe, descriptor, 16)) != (
            0x59494431,
            3,
            0x53593549,
            7,
        ):
            raise ValueError("已配置板的 APP 布局与本批不符")
    return {**identity, "node": node}


def select_flash_speed(probe, target, stage, blank=False):
    """与低速参考比对。空板没有 APP 代码，可用出厂 UID 试读。"""
    address = 0x1FFFF7E8 if blank else maintenance.target_value(target, "TARGET_APP_BASE")
    reference = maintenance.read_twice(probe, address, 12 if blank else 4096)
    for speed in (8000, 4000):
        try:
            probe.set_speed(speed)
            if maintenance.read_twice(probe, address, len(reference)) == reference:
                return probe.speed
        except (RuntimeError, OSError) as exc:
            stage(f"SWD {speed} kHz 试读失败，降低时钟：{exc}")
            continue
        stage(f"SWD {speed} kHz 试读数据不符，降低时钟")
    probe.set_speed(1000)
    if maintenance.read_twice(probe, address, len(reference)) != reference:
        raise RuntimeError("SWD 回退至 1000 kHz 后与原始试读数据不符")
    return probe.speed


def verify_untouched_pages(current, before, segments, base, page):
    """按擦除页边界校验两个加载段以外的全部物理 Flash。"""
    end_previous = 0
    for start, data in segments:
        begin = start - base
        if current[end_previous:begin] != before[end_previous:begin]:
            raise RuntimeError("加载段以外的 Flash 页与备份不一致")
        end_previous = (begin + len(data) + page - 1) // page * page
    if current[end_previous:] != before[end_previous:]:
        raise RuntimeError("镜像后的 Flash（含参数、身份区）与备份不一致")


def save_initial_binding(out, identity):
    """回读通过后保存芯片和节点的配对，供后续 SWD 和 CAN 压测使用。"""
    temp = out / "binding.tmp"
    temp.write_text(
        json.dumps({"node": identity["node"], "uid": identity["uid"]}), encoding="utf-8"
    )
    temp.replace(out.parent / f"first-fit-{identity['uid']}.json")


def set_blank_board_outputs_low(probe):
    """空板缺少 LoaderBoard_Safe 初始化。暂停后只配置六个栅极引脚，再检查实际电平。"""
    if not probe.halt():
        raise RuntimeError("空板 CPU 未暂停，无法初始化六路输出")
    gpio_clock, timer_clock = 0x40023830, 0x40023844
    probe.memory_write32(gpio_clock, [probe.memory_read32(gpio_clock, 1)[0] | 3])
    if probe.memory_read32(gpio_clock, 1)[0] & 3 != 3:
        raise RuntimeError("GPIOA/B 时钟未开启，无法读取六路电平")
    for gpio, pins, mode_mask, output_mode in (
        (0x40020000, 0xE00, 0xFC0000, 0x540000),  # PA9、PA10、PA11
        (0x40020400, 0x013, 0x00030F, 0x000105),  # PB0、PB1、PB4
    ):
        probe.memory_write32(gpio + 0x28, [pins])  # 先清输出锁存，再接通推挽输出
        if probe.memory_read32(gpio + 0x14, 1)[0] & pins:
            raise RuntimeError("空板 GPIO 输出锁存未清零")
        for address, mask, bits in (
            (gpio + 0x04, pins, 0),
            (gpio + 0x0C, mode_mask, 0),
            (gpio, mode_mask, output_mode),
        ):
            probe.memory_write32(address, [(probe.memory_read32(address, 1)[0] & ~mask) | bits])
            if probe.memory_read32(address, 1)[0] & mask != bits:
                raise RuntimeError(f"空板 GPIO 配置回读不符，地址 0x{address:08X}")
    probe.memory_write32(timer_clock, [probe.memory_read32(timer_clock, 1)[0] | 1])
    if not probe.memory_read32(timer_clock, 1)[0] & 1:
        raise RuntimeError("TMR1 时钟未开启，无法读取 PWM 使能状态")


def program_motor(probe, target, identity, raw, segments, out, report, stage, record=b""):
    """备份、按扇区写入、物理回读；遇到失败保留备份和已到达的阶段。"""
    uid = bytes.fromhex(identity["uid"])
    base, app, param, page = (
        maintenance.target_value(target, key)
        for key in (
            "TARGET_FLASH_BASE",
            "TARGET_APP_BASE",
            "TARGET_PARAM_BASE",
            "TARGET_ERASE_BYTES",
        )
    )
    blank_program = bool(record) and maintenance.read_twice(probe, base, param - base) == (
        b"\xff" * (param - base)
    )
    if blank_program:
        stage("已确认空白程序区，初始化并回读六路关相电平")
        set_blank_board_outputs_low(probe)
        maintenance.check_outputs_off(probe, target)
    stage("双读备份整片 Flash")
    backup = maintenance.capture(probe, target, uid, out / "before")
    before = (out / "before/flash.bin").read_bytes()
    if maintenance.digest(before) != backup["flash"]:
        raise OSError("磁盘备份校验失败")
    offset = app - base + maintenance.target_value(target, "TARGET_DESCRIPTOR_OFFSET")
    # 中途断开后始终与第一次写入前的备份核对，避免把损坏的保护区当成新基线。
    pending = out.parent / f"jlink-{identity['uid']}-{report['hex_sha256']}.pending"
    reference = before
    if pending.exists():
        prior = json.loads(pending.read_text(encoding="utf-8"))
        name = prior["directory"]
        if not name or Path(name).name != name or name in (".", ".."):
            raise ValueError("原始备份目录无效")
        reference = (out.parent / name / "before/flash.bin").read_bytes()
        if (
            len(reference) != len(before)
            or hashlib.sha256(reference).hexdigest() != prior["sha256"]
        ):
            raise ValueError("上次中断烧录的原始备份校验失败")
        if prior.get("first_fit_node") != (identity["node"] if record else None):
            raise ValueError("当前选择与上次中断写入的角色不同")
        if record and prior.get("record_sha256") != report["parameter_record_sha256"]:
            raise ValueError("待重试的初装参数与原任务不一致")
        if record and any(
            (actual & wanted) != wanted
            for actual, wanted in zip(
                before[param - base : param - base + len(record)], record, strict=True
            )
        ):
            raise RuntimeError("芯片上的参数不属于本次中断的初装记录")
        if before[param - base + len(record) :] != reference[param - base + len(record) :]:
            raise RuntimeError(f"保护区与上次烧录前的备份不一致，请检查记录 {name}")
        report["recovery_backup"] = name
    current_layout = struct.unpack_from(">4I", reference, offset) == (0x59494431, 3, 0x53593549, 7)
    param_offset = param - base
    size = maintenance.target_value(target, "TARGET_PARAM_BYTES")
    if record:
        from intelhex import IntelHex

        id_offset = maintenance.target_value(target, "TARGET_IDENTITY_BASE") - base
        if reference[param_offset:id_offset] != b"\xff" * (id_offset - param_offset):
            raise ValueError("参数区已有内容，拒绝初装")
        if not current_layout and (
            reference[:param_offset] != b"\xff" * param_offset
            or reference[param_offset + size :] != b"\xff" * (len(reference) - param_offset - size)
        ):
            raise ValueError("现有 Flash 并非空白板或本批布局，已保存备份")
        expected_params = record + reference[param_offset + len(record) : param_offset + size]
        (out / "initial-parameters.bin").write_bytes(expected_params)
        image = IntelHex(io.StringIO(raw.decode("ascii")))
        image.frombytes(record, offset=param)
        stream = io.StringIO()
        image.write_hex_file(stream)
        raw = stream.getvalue().encode("ascii")
        segments = [*segments, (param, record)]
    elif not current_layout:
        raise ValueError("电机原镜像不是本批 layout 7，已保存备份")
    elif (
        maintenance.validate_parameters(before[param_offset : param_offset + size])["node"]
        != identity["node"]
    ):
        raise ValueError("备份中的节点与当前电机不一致")
    report["backup_sha256"] = backup["flash"]["sha256"]
    if "recovery_backup" in report:
        verify_untouched_pages(before, reference, segments, base, page)
    if all(before[start - base : start - base + len(data)] == data for start, data in segments):
        if record:
            save_initial_binding(out, identity)
        pending.unlink(missing_ok=True)
        report.update(already_installed=True, protected_regions_verified=True)
        return
    hex_path = out / "firmware.hex"
    hex_path.write_bytes(raw)
    if hex_path.read_bytes() != raw:
        raise OSError("待烧录 HEX 写盘校验失败")
    maintenance.check_identity(probe, target, uid)
    maintenance.check_outputs_off(probe, target)
    stage("暂停电机控制器")
    if record:
        # 初装时板上可能没有执行过时钟初始化。WDT 配置同步需要 LICK。
        ctrlsts = 0x40023874
        if not probe.memory_read32(ctrlsts, 1)[0] & 2:
            probe.memory_write32(ctrlsts, [1])
            deadline = time.monotonic() + 0.1
            while not probe.memory_read32(ctrlsts, 1)[0] & 2:
                if time.monotonic() >= deadline:
                    raise RuntimeError("初装时 LICK 时钟未就绪")
                time.sleep(0.001)
    maintenance.halt_for_flash(probe, target)
    if maintenance.read_twice(probe, base, len(before)) != before:
        raise RuntimeError("暂停后 Flash 与备份不一致，已停止")
    maintenance.check_identity(probe, target, uid)
    if not pending.exists():
        journal = out / "pending.tmp"
        journal.write_text(
            json.dumps(
                {
                    "directory": out.name,
                    "sha256": report["backup_sha256"],
                    "first_fit_node": identity["node"] if record else None,
                    "record_sha256": report.get("parameter_record_sha256"),
                }
            ),
            encoding="utf-8",
        )
        journal.replace(pending)
    report["write_started"] = True
    stage("按扇区写入 Loader、APP 和所选轴的初始参数" if record else "按扇区写入 Loader 和 APP")
    probe.flash_file(str(hex_path), 0)
    if not probe.halt():
        raise RuntimeError("烧录后无法确认 CPU 已暂停")
    if blank_program and (
        probe.memory_read32(0x40023830, 1)[0] & 3 != 3
        or not probe.memory_read32(0x40023844, 1)[0] & 1
    ):
        # Flash 算法可能复位外设；此时 CPU 已暂停，新固件尚未启动。
        set_blank_board_outputs_low(probe)
    maintenance.check_outputs_off(probe, target)
    stage("双读校验镜像、参数和身份区")
    after = maintenance.read_twice(probe, base, len(before))
    (out / "flash_after.bin").write_bytes(after)
    for start, data in segments:
        begin = start - base
        if after[begin : begin + len(data)] != data:
            raise RuntimeError(f"0x{start:08X} 镜像回读不符")
    verify_untouched_pages(after, reference, segments, base, page)
    if record and after[param_offset : param_offset + size] != expected_params:
        raise RuntimeError("初装参数区或工厂身份页回读不符")
    if record:
        save_initial_binding(out, identity)
    pending.unlink()
    report.update(
        protected_regions_verified=True,
        cpu_halted=True,
        flash_sha256=maintenance.digest(after)["sha256"],
    )


def restart_verified_app(probe, target, parameter_file):
    """通过 SWD 复位并运行，确认 Loader 已将向量表切到本批 APP。"""
    maintenance.check_outputs_off(probe, target)
    # VC_CORERESET 会在复位入口截停 CPU，保留 DEMCR 中的其他调试位。
    demcr = 0xE000EDFC
    probe.memory_write32(demcr, [probe.memory_read32(demcr, 1)[0] & ~1])
    # 烧录算法可留下 C_MASKINTS，清除前必须暂停，否则 APP 的 CAN 中断不会执行。
    dhcsr = 0xE000EDF0
    if probe.memory_read32(dhcsr, 1)[0] & 8:
        if not probe.halt():
            raise RuntimeError("无法暂停 CPU 以恢复中断")
        probe.memory_write32(dhcsr, [0xA05F0003])  # DBGKEY | C_HALT | C_DEBUGEN
        if probe.memory_read32(dhcsr, 1)[0] & 8:
            raise RuntimeError("调试中断屏蔽未清除")
    # 使用 Cortex-M 系统复位，经过 Loader 的正常启动检查。
    aircr = 0xE000ED0C
    priority_group = probe.memory_read32(aircr, 1)[0] & 0x700
    probe.memory_write32(aircr, [0x05FA0004 | priority_group])
    time.sleep(0.1)
    if probe.halted():
        probe.memory_write32(demcr, [probe.memory_read32(demcr, 1)[0] & ~1])
        probe.restart()
    app_base = maintenance.target_value(target, "TARGET_APP_BASE")
    deadline = time.monotonic() + 3.0
    while probe.halted() or probe.memory_read32(0xE000ED08, 1)[0] != app_base:
        if time.monotonic() >= deadline:
            raise RuntimeError("重启后未检测到 APP 运行")
        time.sleep(0.05)
    maintenance.check_outputs_off(probe, target)
    before = parameter_file.read_bytes()
    after = maintenance.read_twice(
        probe, maintenance.target_value(target, "TARGET_PARAM_BASE"), len(before)
    )
    if after != before:
        raise RuntimeError("重启后参数区与烧录前备份不一致")


def run_jlink(settings: dict, out: Path, events, stop) -> None:
    """界面工作进程入口；识别和烧录用独立连接，烧录前重新绑定探针和芯片。"""
    mode = settings["profile"]
    report = {"profile": mode, "success": False, "write_started": False}

    report["steps"] = []
    started = time.monotonic()

    def save():
        temporary = out / "summary.tmp"
        temporary.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        temporary.replace(out / "summary.json")

    def stage(label):
        report["stage"] = label
        line = f"{time.monotonic() - started:6.2f} s  {label}"
        report["steps"].append(line)
        save()
        events.put(("stage", line))

    try:
        import pylink

        resource = Path(getattr(sys, "_MEIPASS", Path(__file__).resolve().parent))
        target = maintenance.load_target(
            TARGET, directory=resource if getattr(sys, "frozen", False) else None
        )
        batch = json.loads((resource / "five_axis_can_stability.json").read_text(encoding="utf-8"))
        motors = {item["uid"]: item["node"] for item in batch["motors"]}
        if mode == "jlink_flash":
            raw = Path(settings["hex"]).read_bytes()
            report["hex_sha256"] = batch["hex_sha256"]
            segments = firmware_segments(raw, report["hex_sha256"], target)
        elif mode != "jlink_identify":
            raise ValueError("未知 J-Link 操作")
        stage("连接 J-Link / SWD")
        library = pylink.library.Library(dllpath=os.environ.get("JLINK_DLL") or None)
        if library.dll() is None:
            raise OSError("找不到 SEGGER J-Link 64 位驱动库，请安装 J-Link 或设置 JLINK_DLL")
        probe = pylink.JLink(lib=library)
        try:
            probes = probe.connected_emulators()
            if len(probes) != 1:
                raise RuntimeError(f"检测到 {len(probes)} 只 J-Link，请连接本次使用的一只探针")
            serial = probes[0].SerialNumber
            if mode == "jlink_flash" and settings["identity"]["serial"] != serial:
                raise RuntimeError("J-Link 已更换，请重新识别")
            probe.open(serial_no=serial)
            for command in (
                "SetAllowFlashCache = 0",
                "SetEnableMemCache = 0",
                "SetSkipDebugDeInit = 1",
                "SetDbgPowerDownOnClose = 0",
                "SetRestartOnClose = 0",
            ):
                # 这些设置命令可以返回 0 或 1；驱动错误由 pylink 通过异常报告。
                probe.exec_command(command)
            probe.set_tif(pylink.enums.JLinkInterfaces.SWD)
            stage("通过 SWD 连接 AT32F456")
            probe.connect(target["build"]["jlink_device"], speed=1000)
            identity = identify_motor(
                probe, target, motors, serial, out.parent, batch["hex_sha256"]
            )
            report.update(identity=identity)
            if mode == "jlink_flash":
                if identity != settings["identity"]:
                    raise RuntimeError("当前电机与界面已识别的电机不同，请重新识别")
                node = settings.get("first_fit_node")
                record = b""
                if identity["node"] is None:
                    if node not in FIRST_FIT_AXES or identity.get("resume_node", node) != node:
                        raise ValueError("请选择待初装的电机类型；中断恢复须沿用原选择")
                    directory = (
                        resource
                        if getattr(sys, "frozen", False)
                        else ROOT / "outputs/canfd-desktop/resources"
                    )
                    record = initial_record(directory, node)
                    identity = {**identity, "node": node}
                    report.update(
                        identity=identity,
                        first_fit_node=node,
                        parameter_record_sha256=hashlib.sha256(record).hexdigest(),
                    )
                    stage(
                        f"初装 {FIRST_FIT_AXES[node][0]} · {FIRST_FIT_AXES[node][1]} · 节点 {node}"
                    )
                elif node is not None:
                    raise ValueError("该板已有电机参数，拒绝覆盖为初装配置")
                report["swd_khz"] = select_flash_speed(probe, target, stage, blank=bool(record))
                stage(f"SWD 试读通过，当前时钟 {report['swd_khz']} kHz")
                program_motor(probe, target, identity, raw, segments, out, report, stage, record)
                if settings.get("restart_after_flash", False):
                    stage("Loader、APP 和保护区已核验，正在通过 J-Link 复位芯片")
                    report.pop("cpu_halted", None)
                    restart_verified_app(
                        probe,
                        target,
                        out / ("initial-parameters.bin" if record else "before/parameters.bin"),
                    )
                    report.update(
                        app_started=True, cpu_halted=False, parameters_verified_after_boot=True
                    )
        finally:
            if probe.opened():
                probe.close()
        if report.get("app_started"):
            bus = settings["bus"]
            stage(f"APP 已启动，参数区一致。打开 USB-CAN 通道 {bus['channel_index'] + 1}，1M/5M")
            result = {
                "planned": 11,
                "passed": 0,
                "timeout_ms": 50,
                "application_retries": 0,
                **bus,
                "queries": [],
            }
            report["can_test"] = result
            query_restarted_motor(
                controlcan_dll_path(resource),
                bus,
                identity["node"],
                bytes.fromhex(identity["uid"]),
                result,
                stage,
                versions=batch_versions(batch),
            )
        report["success"] = True
    except Exception as exc:
        report["fatal"] = str(exc) or type(exc).__name__
    finally:
        report["elapsed_s"] = round(time.monotonic() - started, 3)
        save()
        events.put(("done", report))
