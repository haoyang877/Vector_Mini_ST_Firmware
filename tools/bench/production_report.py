"""每轮硬件检测的人类可读报告；沿用原始判据，不拼接不同轮次判定合格。"""

import csv
import html
import json
from datetime import datetime
from pathlib import Path

from hardware_check import result_rows

SCOPE_NAMES = {"static": "关相静态", "static_resistance": "静态＋相电阻", "full": "完整检测"}
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


def failure_summary(report: dict) -> str:
    """区分设备异常、缺失结果和执行错误，列出本轮数值与排查建议。"""
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
    scope = report.get("scope", "full" if report.get("drive") else "static")
    rows = report_rows(report)
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

    def version(key):
        value = report.get(key)
        return "未取得" if value is None else f"v{value >> 16}.{(value >> 8) & 255}.{value & 255}"

    return dict(
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
        verdict="本范围通过" if passed else "未通过 / 未完成",
        failure_summary=failure_summary(report),
        notes=[
            report[key]
            for key in ("fatal", "note", "measurement_note", "scope_note")
            if report.get(key)
        ],
        rows=rows,
        raw=report,
    )


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
        writer.writerow(("UID", "节点", "范围", "项目", "实测值", "判据", "结果", "排查建议"))
        for row in data["rows"]:
            writer.writerow(
                (
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
        ".pending,.excluded{color:#64748b}dt{font-weight:bold}dd{margin-bottom:8px}"
        "@media print{body{margin:8px}}</style><h1>电机生产测试报告</h1>"
        f"<h2>{escape(data['scope_name'])}：{escape(data['verdict'])}</h2>"
        "<p>仅代表本次所选范围，不代替整机出厂验收。型号来自操作选择。</p>"
        f"<dl>{metadata}</dl><p>{escape('；'.join(data['notes']))}</p>"
        f"<h3>本轮摘要</h3><pre style='white-space:pre-wrap'>{escape(data['failure_summary'])}</pre>"
        "<table><tr><th>项目</th><th>实测值</th><th>判定范围</th><th>结果</th><th>排查建议</th>"
        f"</tr>{rows}</table></html>"
    )
    path = out / "production-report.html"
    path.write_text(document, encoding="utf-8")
    return path
