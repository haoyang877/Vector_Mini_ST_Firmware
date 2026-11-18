"""一轮生产检测：先预检，再标定、回读、监控及关相。"""

import json
import re
from collections.abc import Callable
from pathlib import Path

from _calibration_curves import export_raw_hall_waveform
from can_calibration_capture import HallRawCapture
from can_calibration_curves import export_calibration_curves, read_committed_calibration_fault
from can_motor_operations import STAGES, describe_faults, run_angle_calibration
from hall_waveform_check import ADC_MAX_COUNTS, hall_signal_limits, inspect_hall_waveform
from hardware_check import run_check
from inspection_waveforms import export_waveform_overview
from production_power_check import run_power_check
from production_report import export_report, inspection_overview

INSPECTION_STAGES = {
    "identity": "确认编号",
    "preflight": "运动前预检",
    "calibration": "标定与回读",
    "power": "高速角度、转速及功率检测",
    "archive": "关相与归档",
}
PRODUCTION_VERSION = "2.3.16"


def inspection_numbers(motor_serial: str, test_requirement: str) -> dict[str, str]:
    """检查归档目录段并保留操作员输入的实际编号。"""
    numbers = dict(motor_serial=motor_serial.strip(), test_requirement=test_requirement.strip())
    for value, label in zip(numbers.values(), ("电机序列号", "测试需求号"), strict=True):
        if not re.fullmatch(r"[\w-]{1,40}", value) or re.fullmatch(
            r"CON|PRN|AUX|NUL|COM[1-9¹²³]|LPT[1-9¹²³]", value, re.IGNORECASE
        ):
            raise ValueError(
                f"{label}须为1～40位字母、数字、下划线或短横线，且不能使用系统保留名称"
            )
    return numbers


def run_motor_calibration(
    bus,
    settings: dict,
    out: Path,
    *,
    stopped: Callable[[], bool],
    progress: Callable[[str], None],
) -> dict:
    """执行既有标定服务并导出本轮保存表；复用原码、关相和回读校验。"""
    kind = settings["calibration_kind"]
    capture = HallRawCapture(out, probe_serial=settings.get("probe_serial"))
    try:
        report = run_angle_calibration(
            bus,
            settings["node"],
            kind=kind,
            stopped=stopped,
            progress=progress,
            on_identified=capture.start,
            on_sample=capture.sample,
            expected_uid=settings.get("expected_uid"),
        )
    finally:
        raw_capture = capture.finish()
    report.update(raw_capture=raw_capture, archive=settings.get("archive", {}))
    saved = report.get("passed") and report.get("stop_confirmed")
    if saved:
        progress("标定已保存，正在只读回读补偿表并绘图")
        try:
            if capture.thread is not None and capture.thread.is_alive():
                raise OSError(raw_capture["error"])
            report["curves"] = export_calibration_curves(
                bus,
                report,
                out,
                report["archive"].get("name", ""),
                probe_serial=settings.get("probe_serial"),
            )
        except Exception as error:
            report["curve_error"] = str(error)
    if kind == 1:
        report["hall_waveform"] = inspect_hall_waveform(out, raw_capture)
        if raw_capture.get("samples"):
            try:
                report["hall_raw_png"] = export_raw_hall_waveform(out, raw_capture)
            except (ImportError, OSError, RuntimeError, ValueError, KeyError) as error:
                report["hall_plot_error"] = str(error)
    if not saved and not report.get("cancelled"):
        _annotate_calibration_failure(bus, report, out)
    return report


def _annotate_calibration_failure(bus, report: dict, out: Path) -> None:
    stage = report.get("last_calibration_stage")
    parts = [f"失败阶段 {STAGES[stage]}"] if stage in STAGES else []
    if (
        stage == 0x1004
        and report.get("phase") == 5
        and report.get("effects") == 2
        and report.get("stop_confirmed")
        and report.get("stop_faults") in (0x20, 0x4000)
        and report.get("capabilities", 0) & (1 << 24)
    ):
        try:
            report["saved_calibration_fault"] = read_committed_calibration_fault(bus, report, out)
        except (OSError, ValueError, RuntimeError) as error:
            report["diagnostic_readback_error"] = str(error)
    if report.get("saved_calibration_fault"):
        parts.append("板端回读 " + "、".join(report["saved_calibration_fault"]["details"]))
    extrema = report.get("hall_waveform", {}).get("extrema_counts", {})
    for channel, (low, high) in extrema.items():
        if low == 0 or high == ADC_MAX_COUNTS:
            parts.append(f"Hall {channel} 原码 {low}～{high} cnt，触及 ADC 端点")
    failure = report.get("calibration_error")
    if failure and failure != report.get("fatal"):
        parts.append(failure)
    observed = report.get("observed_stop_faults", 0)
    if observed & ~report.get("stop_faults", 0):
        parts.append(f"关相核对期间读到 {describe_faults(observed)}（0x{observed:08X}）")
    if report.get("stop_faults") in (0x20, 0x4000) and not report.get("saved_calibration_fault"):
        parts.append("固件共用故障位，板端细项未确认")
    parts.append(report.get("fatal") or "标定未完成")
    report["fatal"] = " · ".join(parts)


def _verified_hall_only_fault(calibration: dict) -> int:
    """只接受本票据提交且已回读编码器、电零位与 Hall 类细项的故障。"""
    if not (
        calibration.get("phase") == 5
        and calibration.get("effects") == 2
        and calibration.get("stop_confirmed")
        and calibration.get("communication_reliability_passed")
    ):
        return 0
    diagnostic = calibration.get("saved_calibration_fault", {})
    if diagnostic.get("calibration_flags", 0) & 3 != 3 or not diagnostic.get("saved_revision"):
        return 0
    faults = calibration.get("stop_faults")
    if faults == 0x4000 and diagnostic.get("hall_fault_mask"):
        return faults
    if faults == 0x20 and diagnostic.get("output_status"):
        return faults
    return 0


def initial_inspection(settings: dict) -> dict:
    """构建可归档的初始状态；打开设备失败也保留本轮编号。"""
    return dict(
        profile="production_inspection",
        node=settings["node"],
        expected_uid=settings["expected_uid"],
        **settings["numbers"],
        phase_states=dict.fromkeys(INSPECTION_STAGES, "未测"),
        passed=False,
        cancelled=False,
        verdict="未通过 / 未完成",
        stop_confirmed=None,
    )


def archive_inspection(report: dict, out: Path) -> None:
    """归档正常、中止和启动失败报告；磁盘错误反馈在任务结果中。"""
    overview = out / "waveform-overview.png"
    try:
        if export_waveform_overview(report, out, overview):
            report["waveform_overview"] = overview.name
    except (ImportError, OSError, RuntimeError, ValueError, KeyError) as error:
        report["waveform_overview_error"] = f"波形总览生成失败：{error}"
    try:
        report["phase_states"]["archive"] = "已归档"
        report["production_report"] = export_report(
            report, out, tool_version=PRODUCTION_VERSION
        ).name
        (out / "summary.json").write_text(
            json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
        )
    except (OSError, ValueError, KeyError) as error:
        report.update(report_error=str(error), passed=False, verdict="归档失败")
        report["phase_states"]["archive"] = "归档失败"


def run_inspection(
    bus,
    settings: dict,
    out: Path,
    report: dict,
    *,
    limits: dict,
    stopped: Callable[[], bool],
    emit: Callable[[str, object], None],
) -> None:
    """在一个独占 CAN 任务内串行工作；任一阶段异常即返回现有证据。"""
    node, uid = settings["node"], settings["expected_uid"]
    phases = report["phase_states"]
    active = "identity"
    static_completed = None

    def stage(key: str, status: str = "进行中") -> None:
        nonlocal active
        active = key
        phases[key] = status
        emit("inspection_phases", phases.copy())
        emit("inspection_results", inspection_overview(report))

    def proceed() -> None:
        if stopped():
            raise InterruptedError("本轮生产检测已停止")

    def save_child(key: str, result: dict) -> None:
        report[key] = result
        report["stop_confirmed"] = result.get("stop_confirmed")
        (out / (key + ".json")).write_text(
            json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
        )

    def progress(text: str) -> None:
        emit("stage", text)

    def publish_static(snapshot: dict) -> None:
        nonlocal static_completed
        completed = snapshot["static_completed"]
        if completed != static_completed:
            static_completed = completed
            emit("inspection_results", inspection_overview(report, static_snapshot=snapshot))

    def verify_calibration(result: dict) -> None:
        if not result.get("passed") or not result.get("stop_confirmed") or not result.get("curves"):
            raise ValueError(
                result.get("fatal") or result.get("curve_error") or "标定或保存回读未通过"
            )
        if not result.get("communication_reliability_passed"):
            raise ValueError("标定通信未通过")

    try:
        report.update(inspection_numbers(report["motor_serial"], report["test_requirement"]))
        stage("identity", "已确认")
        check = dict(
            ambient=settings["ambient"],
            expected_uid=uid,
            limits=limits,
            stopped=stopped,
            progress=progress,
        )
        proceed()
        stage("preflight")
        preflight = run_check(
            bus, node, resistance_only=True, static_progress=publish_static, **check
        )
        save_child("preflight", preflight)
        if preflight.get("uid") == uid:
            report.update({key: preflight[key] for key in ("uid", "app_version", "loader_version")})
        proceed()
        if not all(
            preflight.get(key)
            for key in ("subset_passed", "stop_confirmed", "communication_reliability_passed")
        ):
            raise ValueError(preflight.get("note") or "静态和相电阻预检未通过")
        stage("preflight", "通过")
        hall_installed = preflight["board_items"][7] != 3
        stage("calibration")
        calibration_settings = dict(
            settings,
            archive=dict(
                name=report["motor_serial"],
                test_serial=report["test_requirement"],
            ),
        )
        calibration_out = out / "calibration"
        calibration_out.mkdir()
        calibration = run_motor_calibration(
            bus,
            dict(calibration_settings, calibration_kind=1),
            calibration_out,
            stopped=stopped,
            progress=progress,
        )
        save_child("calibration", calibration)
        shape = calibration.get("hall_waveform", {})
        signal = hall_signal_limits(shape, limits["limits"]) if hall_installed else {}
        report["hall_signal"] = signal
        proceed()
        hall_fault = _verified_hall_only_fault(calibration) if hall_installed else 0
        if hall_fault:
            encoder_record = calibration["saved_calibration_fault"]
            report["fatal"] = calibration.get("fatal") or "板端 Hall 标定未通过"
        else:
            verify_calibration(calibration)
            encoder_record = calibration["curves"]
        flags = encoder_record["calibration_flags"]
        saved_revision = encoder_record["saved_revision"]
        if not flags & 1:
            raise ValueError("电机端编码器补偿表保存未确认")
        if not flags & 2:
            if hall_installed:
                raise ValueError("带 Hall 轴的联合电零位未通过保存回读")
            zero_out = out / "electrical-zero"
            zero_out.mkdir()
            zero = run_motor_calibration(
                bus,
                dict(calibration_settings, calibration_kind=2),
                zero_out,
                stopped=stopped,
                progress=progress,
            )
            save_child("electrical_zero", zero)
            proceed()
            verify_calibration(zero)
            if not zero["curves"]["calibration_flags"] & 2:
                raise ValueError("电零位保存标志未确认")
            saved_revision = zero["curves"]["saved_revision"]
        hall_signal_failed = any(
            signal.get(key) is False for key in ("range_passed", "amplitude_passed")
        )
        hall_failed = bool(hall_fault) or hall_signal_failed
        hall_review = hall_installed and shape.get("status") != "clear"
        if hall_signal_failed and not hall_fault:
            report["fatal"] = "Hall 原码极值或峰峰值未通过"
        stage("calibration", "未通过" if hall_failed else "待复核" if hall_review else "通过")
        proceed()
        if not settings.get("power_check"):
            report["power_check"] = dict(
                status="not_measured",
                reason="本轮缺少高速试验配置",
            )
        else:
            stage("power")
            high_speed = run_power_check(
                bus,
                settings,
                out,
                saved_revision=saved_revision,
                hall_fault=hall_fault,
                stopped=stopped,
                progress=progress,
                angle_sample=lambda sample: emit("inspection_angle", sample),
            )
            report["power_check"] = high_speed
            if high_speed["run"]["enable_attempted"]:
                report["stop_confirmed"] = high_speed["stop_confirmed"]
            proceed()
        result = report["power_check"]
        outcomes = (result["status"], result.get("speed_stability", {}).get("status"))
        high_speed_passed = (
            outcomes == ("passed", "passed")
            and result.get("rotor_continuity", {}).get("status") == "clear"
        )
        power_result = (
            "通过" if high_speed_passed else "未通过" if "failed" in outcomes else "待复核"
        )
        report["passed"] = high_speed_passed and not hall_failed and not hall_review
        report["verdict"] = (
            "本轮通过"
            if report["passed"]
            else "未通过"
            if hall_failed or power_result == "未通过"
            else "待复核"
        )
        stage("power", power_result)
        if not report.get("stop_confirmed"):
            raise ValueError("未取得最终关相确认")
        stage("archive", "关相已确认")
    except InterruptedError as error:
        report.update(cancelled=True, verdict="已停止", note=str(error))
        if phases[active] == "进行中":
            stage(active, "已停止")
    except (OSError, ValueError, RuntimeError) as error:
        report.update(fatal=str(error))
        stage(active, "未通过")
