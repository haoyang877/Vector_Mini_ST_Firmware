"""每轮硬件检测的人类可读报告；沿用原始判据，不拼接不同轮次判定合格。"""

import csv
import html
import json
from datetime import datetime
from pathlib import Path

from hall_waveform_check import hall_waveform_criterion
from hardware_check import ITEMS, result_rows
from production_power_check import (
    DEFAULT_POWER_MAX_W,
    DEFAULT_POWER_MIN_W,
    RECORD_S,
    SPEED_ERROR_FRACTION,
)

STATIC_CHECK_ITEMS = ITEMS[:8]
ENCODER_CALIBRATION_ITEM = "编码器标定"
HALL_CALIBRATION_ITEM = "Hall 标定"
ROTOR_CONTINUITY_ITEM = "电机端角度连续性"
SPEED_ITEM = "输出轴转速稳定性"
POWER_ITEM = "高速输入功率稳定性"
STOP_ITEM = "关相确认"
HIGH_SPEED_RESULTS = {
    "passed": "通过",
    "failed": "未通过",
    "incomplete": "待复核",
    "not_measured": "未测",
    "cancelled": "已停止",
}

SCOPE_NAMES = {
    "static": "关相静态",
    "static_resistance": "静态＋相电阻",
    "full": "完整检测",
    "production_inspection": "整轮生产检测",
}
SUGGESTIONS = {
    "Ia": "检查A相采样、零偏和接线。",
    "Ib": "检查B相采样、零偏和接线。",
    "Ic": "检查C相采样、零偏和接线。",
    "母线电流": "检查母线电流采样及零偏，确认测试前已停机。",
    "母线电压": "核对电源输出、线缆压降和电压采样。",
    "电机 NTC": "核对NTC接线、阻值和传感器类型；无效值不能当作真实温度。",
    "电机角静态": "检查编码器连接、磁钢安装及静态转子稳定性。",
    "霍尔静态": "检查Hall供电、接线及模拟采样。",
    "相电阻": "按具体故障检查相线、绕组和接头；结合ABC/AB/BC/CA实测值定位。",
    "电机角动态": "检查编码器方向、相序和机械运动条件。",
    "霍尔动态": "检查Hall接线、安装方向和有效行程。",
}


def report_rows(report: dict) -> list[dict]:
    """保留原始结果与数值，并为异常/未完成项提供排查方向。"""
    if report.get("profile") == "production_inspection":
        return inspection_rows(report)
    rows = []
    for name, measured, limit, state in result_rows(report):
        if name == "相电阻" and report.get("scope") == "static":
            measured, state = "本轮未执行", "不在本轮范围"
        excluded = state in ("不在本轮范围", "未装")
        failure = state not in ("正常", "未测", "不在本轮范围", "未装")
        rows.append(
            dict(
                item=name,
                measured=measured,
                limit=limit,
                result=state,
                attention=not excluded and state != "正常",
                outcome="excluded"
                if excluded
                else "failed"
                if failure
                else "passed"
                if state == "正常"
                else "pending",
                suggestion=SUGGESTIONS.get(name, "核对原始记录。")
                if failure
                else "本轮未取得完整结果，请检查中止原因。"
                if state == "未测"
                else "",
            )
        )
    return rows


def _encoder_calibration_result(report: dict, no_hall: bool) -> tuple[str, str]:
    result = report.get("calibration", {})
    phase = report.get("phase_states", {}).get("calibration", "未测")
    saved = result.get("curves") or result.get("saved_calibration_fault", {})
    zero = report.get("electrical_zero", {})
    zero_saved = zero.get("curves", {}) if zero else saved
    lut_done = bool(saved.get("calibration_flags", 0) & 1)
    if lut_done and zero_saved.get("calibration_flags", 0) & 2:
        return f"角度补偿与电角零位已回读 · 修订 {zero_saved['saved_revision']}", "已保存"
    if phase == "已停止":
        return ("角度补偿已回读 · 电角零位停止" if lut_done else "标定已停止"), phase
    if phase == "未通过" and zero:
        reason = zero.get("fatal") or zero.get("curve_error") or "未取得保存回读"
        return f"电角零位 · {reason}", phase
    stage = result.get("last_calibration_stage")
    if phase == "未通过" and (no_hall or stage in (0x1001, 0x1002)):
        reason = result.get("fatal") or result.get("curve_error") or report.get("fatal")
        return reason or "编码器标定未完成", phase
    if lut_done or stage == 0x1004:
        return "角度补偿已完成 · 电角零位保存未确认", "待复核"
    if phase == "未通过":
        return "编码器结果未确认 · 见本轮故障详情", "待复核"
    return "—", phase


def _hall_calibration_result(report: dict, no_hall: bool) -> tuple[str, str]:
    if no_hall:
        return "该轴未装线性 Hall", "未装"
    result = report.get("calibration", {})
    phase = report.get("phase_states", {}).get("calibration", "未测")
    diagnostic = result.get("saved_calibration_fault", {})
    if details := diagnostic.get("details"):
        faults = result.get("stop_faults", 0)
        code = f"（0x{faults:08X}）" if faults else ""
        curve = " · 已生成 Hall 诊断曲线" if diagnostic.get("hall_diagnostic_curve") else ""
        return "板端回读 · " + "、".join(details) + code + curve, "未通过"
    if curves := result.get("curves"):
        return f"Hall 补偿表已回读 · 修订 {curves['saved_revision']}", "已保存"
    signal = report.get("hall_signal", {})
    if signal.get("range_passed") is False:
        low, high = signal["min_counts"], signal["max_counts"]
        limits = [
            f"{channel} {start}～{end} cnt"
            for channel, (start, end) in signal["extrema_counts"].items()
            if start < low or end > high
        ]
        return f"原码实测越界 · {'、'.join(limits)}", "未通过"
    if phase == "未通过":
        if result.get("last_calibration_stage") == 0x1004:
            return "Hall 标定结果未确认 · 见本轮故障详情", "待复核"
        return "未取得 Hall 标定结果", "未测"
    return "—", phase


def inspection_rows(report: dict) -> list[dict]:
    """投影同轮预检、标定、角度采样和收尾证据。"""
    rows = []
    phases = report.get("phase_states", {})

    def append(item, measured, limit, result, **layout):
        outcome = {
            "通过": "passed",
            "已保存": "passed",
            "已记录": "passed",
            "未通过": "failed",
            "未装": "excluded",
            "待复核": "review",
        }.get(result, "pending")
        rows.append(
            dict(
                item=item,
                measured=measured,
                limit=limit,
                result=result,
                outcome=outcome,
                attention=outcome not in ("passed", "excluded"),
                suggestion="",
                **layout,
            )
        )

    preflight = report.get("preflight")
    if preflight:
        rows.extend(row for row in report_rows(preflight) if row["result"] != "不在本轮范围")
    calibration = report.get("calibration", {})
    no_hall = bool(preflight and preflight.get("board_items", [0] * 8)[7] == 3)
    encoder_measured, encoder_result = _encoder_calibration_result(report, no_hall)
    if error := calibration.get("saved_calibration_fault", {}).get("encoder_curve_error"):
        encoder_measured += f" · 曲线导出失败：{error}"
    append(
        ENCODER_CALIBRATION_ITEM,
        encoder_measured,
        "角度补偿及电角零位已保存并回读",
        encoder_result,
        evidence="encoder",
    )
    hall_measured, hall_result = _hall_calibration_result(report, no_hall)
    append(
        HALL_CALIBRATION_ITEM,
        hall_measured,
        "Hall 校验及输出轴配对通过并回读",
        hall_result,
    )
    waveform = calibration.get("hall_waveform", {})
    signal = report.get("hall_signal", {})
    waiting = "未装" if no_hall else "待复核" if calibration.get("passed") else "未测"
    raw_capture = calibration.get("raw_capture", {})
    sweep_verified = waveform.get("status") in ("clear", "detected") or (
        calibration.get("passed")
        and not raw_capture.get("error")
        and raw_capture.get("complete") is not False
    )
    extrema = signal.get("extrema_counts", {})
    ranges = " · ".join(f"{key} {low}–{high}" for key, (low, high) in extrema.items())
    append(
        "Hall 最大值 / 最小值",
        ranges + " cnt" if ranges else waveform.get("reason", "—"),
        f"两路均在 [{signal['min_counts']}, {signal['max_counts']}] cnt"
        if signal
        else "两路均在配置区间内",
        ("通过" if sweep_verified else "待复核")
        if extrema and not no_hall and signal["range_passed"]
        else "未通过"
        if extrema and not no_hall
        else waiting,
        subitem=True,
    )
    amplitude = signal.get("peak_to_peak_counts", {})
    append(
        "Hall 峰峰值",
        " · ".join(f"{key} {value} cnt" for key, value in amplitude.items())
        or waveform.get("reason", "—"),
        "两路均 > ADC 全量程的一半（≥2048 cnt）",
        ("通过" if signal["amplitude_passed"] else "未通过" if sweep_verified else "待复核")
        if amplitude and not no_hall
        else waiting,
        subitem=True,
    )
    sectors = []
    for sector in waveform.get("sectors", []):
        start = sector["start_deg"]
        end = start + sector["width_deg"]
        depth = f"，深度 {sector['max_chord_depth']:.4f}" if "max_chord_depth" in sector else ""
        sectors.append(
            f"{start}～{end if end <= 360 else end % 360}°（{sector['repeat_count']}次{depth}）"
        )
    append(
        "Hall 波形凹坑",
        "重复凹陷 · " + "、".join(sectors)
        if sectors
        else waveform.get("reason", "未检出" if waveform.get("status") == "clear" else "—"),
        hall_waveform_criterion(waveform),
        "通过" if not no_hall and waveform.get("status") == "clear" else waiting,
        subitem=True,
        evidence="hall",
    )
    power = report.get("power_check", {})
    rotor = power.get("rotor_continuity", report.get("rotor_continuity", {}))
    rotor_status = rotor.get("status")
    speed_state = phases.get("power", "未测")
    motion_state = speed_state if speed_state in ("进行中", "已停止") else "未测"
    angle_state = phases.get("angle_monitor", motion_state)
    append(
        ROTOR_CONTINUITY_ITEM,
        f"{rotor['pairs']} 对 · 跳变 {rotor['jumps']} · 缺样 {rotor['gaps']}"
        if "pairs" in rotor
        else rotor.get("reason")
        or (report.get("fatal") if angle_state == "未通过" else None)
        or power.get("reason")
        or "—",
        "稳速段相邻角步进 · 试验判线",
        "通过" if rotor_status == "clear" else "待复核" if rotor_status else angle_state,
        evidence="angle",
    )
    speed = power.get("speed_stability", {})
    tolerance = SPEED_ERROR_FRACTION * 100
    window_s = power.get("window_s", RECORD_S)
    speed_value = speed.get("reason") or power.get("reason") or "—"
    if "min_rpm" in speed:
        speed_value = f"{speed['min_rpm']:.2f}–{speed['max_rpm']:.2f} rpm · 最大偏差 {speed['max_error_pct']:.3f}%"
        if speed.get("reason"):
            speed_value += " · " + speed["reason"]
    if window := power.get("steady_window_s"):
        speed_value += f" · 判定窗口 {window[0]:.2f}～{window[1]:.2f} s"
    append(
        SPEED_ITEM,
        speed_value,
        f"{window_s:g} 秒内各样本在 {power['output_rpm']:g} rpm ±{tolerance:g}%"
        if "output_rpm" in power
        else f"{window_s:g} 秒内各样本在目标 ±{tolerance:g}%",
        HIGH_SPEED_RESULTS.get(speed.get("status"), motion_state),
        evidence="speed",
    )
    power_value = power.get("reason", "—")
    if "peak_to_peak_w" in power:
        power_value = (
            f"最小 {power['min_w']:.2f} / 最大 {power['max_w']:.2f} W · "
            f"均值 {power['mean_w']:.2f} W · 峰峰值 {power['peak_to_peak_w']:.2f} W"
        )
        if power.get("reason"):
            power_value += " · " + power["reason"]
    power_limit = (
        f"母线输入功率峰峰值 ≤{power['ripple_limit_w']:g} W"
        if "ripple_limit_w" in power
        else f"各样本在 [{power.get('power_min_w', DEFAULT_POWER_MIN_W):g}, "
        f"{power.get('power_max_w', DEFAULT_POWER_MAX_W):g}] W"
    )
    append(
        POWER_ITEM,
        power_value,
        power_limit,
        HIGH_SPEED_RESULTS.get(power.get("status"), speed_state),
        evidence="power",
    )
    append(
        STOP_ITEM,
        "输出已关闭" if report.get("stop_confirmed") else "—",
        "板端关相确认",
        "通过"
        if report.get("stop_confirmed")
        else "未通过"
        if report.get("stop_confirmed") is False
        else "未测",
    )
    return rows


def inspection_overview(report: dict, *, static_snapshot: dict | None = None) -> list[dict]:
    """将同轮证据投影到操作台，静态子项使用本轮采样和实际判据。"""
    phases = report.get("phase_states", {})
    preflight = report.get("preflight", {})
    evidence = {row["item"]: row for row in inspection_rows(report)}

    def row(item: str, measured: str, limit: str, result: str) -> dict:
        outcome = {
            "通过": "passed",
            "未通过": "failed",
            "待复核": "review",
        }.get(result, "pending")
        return dict(item=item, measured=measured, limit=limit, result=result, outcome=outcome)

    in_preflight = phases.get("preflight") == "进行中"
    waiting = "进行中" if in_preflight else "未测"
    communication = row("CAN 通信", "—", "设备应答、UID 与通信可靠性", waiting)
    if preflight:
        communicating = (
            preflight.get("communication") == "正常"
            and preflight.get("communication_reliability_passed") is True
            and preflight.get("uid") == report.get("expected_uid")
        )
        communication = row(
            "CAN 通信",
            f"节点 {preflight['node']} · UID 已核对"
            if communicating
            else preflight.get("note") or "预检通信或身份核验异常",
            communication["limit"],
            "通过" if communicating else "未通过",
        )
    static_source = (
        {test["item"]: test for test in report_rows(static_snapshot)}
        if static_snapshot is not None
        else evidence
    )
    static = []
    for index, name in enumerate(STATIC_CHECK_ITEMS):
        test = static_source.get(name, row(name, "—", "—", "未测"))
        if static_snapshot is not None:
            completed = static_snapshot["static_completed"]
            if index >= completed and test["outcome"] != "excluded":
                test = row(name, "—", test["limit"], "进行中" if index == completed else "未测")
        static.append(dict(test, subitem=True))
    applicable = [test for test in static if test["outcome"] != "excluded"]
    failed = [test for test in applicable if test["outcome"] == "failed"]
    passed = sum(test["outcome"] == "passed" for test in applicable)
    static_result = row(
        "静态检测",
        "、".join(test["item"] for test in failed) + "异常"
        if failed
        else f"{passed}/{len(applicable)} 项通过"
        if preflight or static_snapshot is not None
        else "—",
        "电流、电压、温度、编码器与 Hall",
        "未通过" if failed else "通过" if applicable and passed == len(applicable) else waiting,
    )
    static_result["group"] = True
    resistance = evidence.get("相电阻") or row("相电阻", "—", "绕组相电阻与一致性", waiting)
    if resistance["outcome"] == "failed":
        resistance = dict(
            resistance,
            measured=f"{resistance['result']} · {resistance['measured']}",
            result="未通过",
        )
    calibration_rows = [
        test
        for test in evidence.values()
        if test["item"] in (ENCODER_CALIBRATION_ITEM, HALL_CALIBRATION_ITEM, ROTOR_CONTINUITY_ITEM)
        or test.get("subitem")
    ]
    stop = evidence[STOP_ITEM]
    if any(
        phases.get(key) == "进行中"
        for key in ("preflight", "calibration", "angle_monitor", "power")
    ):
        stop = row(stop["item"], "—", stop["limit"], "未测")
    return [
        communication,
        static_result,
        *static,
        resistance,
        *calibration_rows,
        evidence[SPEED_ITEM],
        evidence[POWER_ITEM],
        stop,
    ]


def failure_summary(report: dict) -> str:
    """区分设备异常、缺失结果和执行错误，列出本轮数值与排查建议。"""
    if report.get("profile") == "production_inspection":
        return "\n".join(
            filter(
                None,
                (
                    report.get("verdict", "未通过 / 未完成"),
                    report.get("fatal"),
                    report.get("note"),
                    report.get("report_error"),
                    report.get("calibration", {}).get("diagnostic_readback_error"),
                    report.get("calibration", {}).get("hall_plot_error"),
                    report.get("power_check", {}).get("curve_error"),
                    "关相未确认，请核对设备状态" if report.get("stop_confirmed") is False else "",
                ),
            )
        )
    rows = report_rows(report)
    failed = [row for row in rows if row["attention"] and row["result"] != "未测"]
    missing = [row["item"] for row in rows if row["result"] == "未测"]
    lines = [
        report[key]
        for key in ("fatal", "note", "measurement_note", "resistance_note", "report_error")
        if report.get(key)
    ]
    if report.get("communication") != "正常":
        lines.insert(0, "通信未通过，不能判定本轮合格。")
    if report.get("scope") != "static" and not report.get("stop_confirmed"):
        lines.insert(0, "未取得关相确认；请先确认设备已停止。")
    if failed:
        lines.append(f"异常 {len(failed)} 项：")
        lines.extend(
            f"{row['item']} · {row['result']}；实测 {row['measured']}；要求 {row['limit']}；{row['suggestion']}"
            for row in failed
        )
    if missing:
        lines.append("未测 " + str(len(missing)) + " 项：" + "、".join(missing))
    return "\n".join(lines) or "本轮未发现异常，最终结论以范围判定为准。"


def build_report(report: dict, *, tool_version: str, created_at: str) -> dict:
    """仅本轮完整且无异常的已请求范围可通过；结果来自设备，不推断未测项。"""
    inspection = report.get("profile") == "production_inspection"
    scope = (
        "production_inspection"
        if inspection
        else report.get("scope", "full" if report.get("drive") else "static")
    )
    rows = report_rows(report)
    verdict = (
        report.get("verdict", "未通过 / 未完成")
        if inspection
        else ("本范围通过" if hardware_scope_passed(report, scope, rows) else "未通过 / 未完成")
    )

    def version(key):
        value = report.get(key)
        return "未取得" if value is None else f"v{value >> 16}.{(value >> 8) & 255}.{value & 255}"

    data = dict(
        schema_version=1,
        tool_version=tool_version,
        created_at=created_at,
        node=report.get("node"),
        uid=report.get("uid", "未取得"),
        axis=report.get("axis", "未取得"),
        model=report.get("model", "未取得"),
        model_source="操作开始时的轴型号选择（非独立硬件鉴定）",
        app_version=version("app_version"),
        loader_version=version("loader_version"),
        run_id=report.get("run_id"),
        threshold_revision=report.get("threshold_config", {}).get("revision", "固件内置"),
        threshold_sha256=report.get("threshold_config", {}).get("sha256", "—"),
        scope=scope,
        scope_name=SCOPE_NAMES.get(scope, scope),
        verdict=verdict,
        failure_summary=failure_summary(report),
        notes=[
            report[key]
            for key in ("fatal", "note", "measurement_note", "scope_note")
            if report.get(key)
        ],
        rows=rows,
        raw=report,
    )
    if inspection:
        data.update(
            motor_serial=report.get("motor_serial", "未取得"),
            test_requirement=report.get("test_requirement", "未取得"),
        )
    return data


def hardware_scope_passed(report: dict, scope: str, rows: list[dict]) -> bool:
    """复核既有单项报告的采集范围及关相证据。"""
    considered = [
        row
        for row in rows
        if row["result"] not in ("不在本轮范围", "未装")
        and not (scope == "static" and row["item"] == "相电阻")
    ]
    passed = bool(considered) and all(row["result"] == "正常" for row in considered)
    passed = passed and report.get("communication") == "正常" and not report.get("note")
    passed = passed and not report.get("measurement_note") and not report.get("fatal")
    required_items = range(10) if scope == "full" else range(8)
    measurements = report.get("measurements", {})
    passed = passed and all(
        report.get("board_items", [0] * 10)[item] == 3
        or (
            measurements.get(str(item), {}).get("samples", 0) > 0
            and any(
                value is not None for value in measurements.get(str(item), {}).get("values", [])
            )
        )
        for item in required_items
    )
    if scope == "static_resistance":
        passed = passed and bool(report.get("subset_passed"))
    elif scope == "full":
        passed = passed and report.get("stage") == 4 and bool(report.get("stop_confirmed"))
    else:
        passed = passed and report.get("stage") == 4

    return passed


def export_report(report: dict, out: Path, *, tool_version: str) -> Path:
    """在本轮目录导出三种格式；失败由调用者报告，绝不改变测试结果。"""
    data = build_report(
        report,
        tool_version=tool_version,
        created_at=datetime.now().astimezone().isoformat(timespec="seconds"),
    )
    out.mkdir(parents=True, exist_ok=True)
    (out / "production-report.json").write_text(
        json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    with (out / "production-report.csv").open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.writer(stream)
        inspection = data["scope"] == "production_inspection"
        headings = ("电机序列号", "测试需求号") if inspection else ()
        identifiers = (data["motor_serial"], data["test_requirement"]) if inspection else ()
        writer.writerow(
            (*headings, "UID", "节点", "范围", "项目", "实测值", "判据", "结果", "排查建议")
        )
        for row in data["rows"]:
            writer.writerow(
                (
                    *identifiers,
                    data["uid"],
                    data["node"],
                    data["scope_name"],
                    row["item"],
                    row["measured"],
                    row["limit"],
                    row["result"],
                    row["suggestion"],
                )
            )

    def escape(value: object) -> str:
        return html.escape(str(value))

    fields = {
        "created_at": "生成时间",
        "tool_version": "上位机版本",
        "uid": "设备UID",
        "node": "节点",
        "axis": "轴类型",
        "model": "选择的型号",
        "app_version": "APP版本",
        "loader_version": "Loader版本",
        "run_id": "检测轮次",
        "threshold_revision": "阈值配置版本",
        "threshold_sha256": "阈值文件SHA-256",
        "scope_name": "测试范围",
    }
    links = figures = ""
    if inspection:
        fields = {"motor_serial": "电机序列号", "test_requirement": "测试需求号", **fields}
        curves = report.get("power_check", {}).get("curves", {})
        overview = report.get("waveform_overview")
        images = [
            (curves[key], title)
            for key, title in (("speed", "输出轴转速曲线"), ("power", "母线输入功率曲线"))
            if key in curves and (out / curves[key]).is_file()
        ]
        if overview and (out / overview).is_file():
            images = [(overview, "生产检测波形总览")]
        figures = "".join(
            f'<figure style="break-inside:avoid;margin:24px 0"><figcaption>{title}</figcaption>'
            f'<a href="{escape(filename)}"><img src="{escape(filename)}" alt="{title}" '
            'style="max-width:100%;height:auto"></a></figure>'
            for filename, title in images
        )
        links = " ".join(
            f'<a href="{filename}">{title}</a>'
            for filename, title in (
                ("angle-trace.csv", "连续角度CSV"),
                ("high-speed/speed.csv", "高速轴角、转速与母线遥测CSV"),
                ("high-speed/output-speed.png", "输出轴转速曲线"),
                ("high-speed/input-power.png", "母线输入功率曲线"),
                ("calibration/angle-calibration.png", "编码器补偿曲线"),
                ("calibration/angle-calibration.csv", "角度补偿表CSV"),
                ("calibration/hall-raw.png", "Hall 原码波形与 XY 轨迹"),
                ("calibration/hall-raw.csv", "Hall原码CSV"),
                ("calibration.json", "标定票据与诊断"),
                ("preflight.json", "预检原始报告"),
                ("recheck.json", "复检原始报告"),
            )
            if (out / filename).is_file()
        )
    metadata = "".join(
        f"<dt>{escape(label)}</dt><dd>{escape(data[key])}</dd>" for key, label in fields.items()
    )
    rows = "".join(
        f"<tr class='{row['outcome']}'>"
        + "".join(
            f"<td>{escape(row[key])}</td>"
            for key in ("item", "measured", "limit", "result", "suggestion")
        )
        + "</tr>"
        for row in data["rows"]
    )
    document = (
        "<!doctype html><html lang='zh-CN'><meta charset='utf-8'>"
        "<title>电机生产测试报告</title><style>body{font:16px sans-serif;margin:32px}"
        "table{border-collapse:collapse;width:100%}td,th{border:1px solid #bbb;padding:8px}"
        ".failed{color:#b03b42;background:#fff0e8}.passed{color:#18765b}"
        ".pending,.excluded{color:#64748b}.review{color:#875313;background:#fff3dc}dt{font-weight:bold}dd{margin-bottom:8px}"
        "@media print{body{margin:8px}}</style><h1>电机生产测试报告</h1>"
        f"<h2>{escape(data['scope_name'])}：{escape(data['verdict'])}</h2>"
        "<p>仅代表本次所选范围，不代替整机出厂验收。型号来自操作选择。</p>"
        f"<dl>{metadata}</dl><p>{links}</p><p>{escape('；'.join(data['notes']))}</p>"
        f"<h3>本轮摘要</h3><pre style='white-space:pre-wrap'>{escape(data['failure_summary'])}</pre>"
        "<table><tr><th>项目</th><th>实测值</th><th>判定范围</th><th>结果</th><th>排查建议</th>"
        f"</tr>{rows}</table>{figures}</html>"
    )
    path = out / "production-report.html"
    path.write_text(document, encoding="utf-8")
    return path
