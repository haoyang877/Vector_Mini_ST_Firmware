"""本次生产会话按UID记录流程证据，不推断设备历史生产状态。"""

from production_report import build_report


class ProductionFlow:
    """由GUI串行维护；重新烧录或标定会使依赖的后续结果失效。"""

    def __init__(self) -> None:
        self.devices: dict[str, dict] = {}

    def begin(self, uid: str | None, profile: str, *, kind: int = 0, full: bool = False) -> None:
        """动作发起时使对应旧结果失效，异常退出也不会保留旧的完成状态。"""
        if not uid or profile not in (
            "jlink_flash",
            "jlink_erase",
            "motor_calibration",
            "hardware_check",
        ):
            return
        device = self.devices.setdefault(uid, dict(uid=uid, steps={}, records=[]))
        steps = device["steps"]
        if profile in ("jlink_flash", "jlink_erase"):
            steps.clear()
        elif profile == "motor_calibration":
            steps.pop("recheck", None)
            if kind == 1:
                steps.pop("angle", None)
                steps.pop("zero", None)
            elif kind == 2:
                steps.pop("zero", None)
        elif profile == "hardware_check" and full:
            key = "recheck" if steps.get("angle") and steps.get("zero") else "hardware"
            steps.pop(key, None)

    def record(self, report: dict, directory: str) -> None:
        """只接收有设备UID的实际操作报告；保存证据目录而非复制原始数据。"""
        profile = report.get("profile")
        uid = report.get("uid") or report.get("identity", {}).get("uid")
        if not uid:
            return
        if profile == "parameters" and (report.get("applied") or report.get("save_sent")):
            steps = self.devices.get(uid, {}).get("steps", {})
            for key in ("hardware", "recheck"):
                steps.pop(key, None)
            return
        if profile not in ("jlink_flash", "hardware_check", "motor_calibration"):
            return
        if report.get("needs_initial_setup"):
            return
        device = self.devices.setdefault(uid, dict(uid=uid, steps={}, records=[]))
        steps = device["steps"]
        if profile == "jlink_flash":
            steps.clear()
            steps["flash"] = bool(
                report.get("success")
                and report.get("app_started")
                and report.get("can_test", {}).get("version_verified")
            )
        elif profile == "motor_calibration":
            steps.pop("recheck", None)
            if report.get("kind") == 1:
                steps.pop("zero", None)
            key = (
                "angle" if report.get("kind") == 1 else "zero" if report.get("kind") == 2 else None
            )
            if key is None:
                return
            steps[key] = bool(report.get("passed") and report.get("stop_confirmed"))
            if (
                key == "angle"
                and steps[key]
                and report.get("curves", {}).get("calibration_flags", 0) & 2
            ):
                steps["zero"] = True
        else:
            if report.get("scope") != "full":
                return
            key = "recheck" if steps.get("angle") and steps.get("zero") else "hardware"
            steps[key] = (
                build_report(report, tool_version="", created_at="")["verdict"] == "本范围通过"
            )
        device["records"].append(dict(profile=profile, directory=directory, steps=steps.copy()))

    def summary(self, uid: str | None) -> str:
        """只汇总当前UID的本次证据；完整顺序要求不代替整机出厂验收。"""
        steps = self.devices.get(uid, {}).get("steps", {})
        labels = (
            ("flash", "烧录"),
            ("hardware", "完整检测"),
            ("angle", "角度标定"),
            ("zero", "电零位"),
            ("recheck", "完整复检"),
        )
        parts = [
            f"{name}：{'完成' if steps[key] else '未通过'}" if key in steps else f"{name}：未完成"
            for key, name in labels
        ]
        complete = all(steps.get(key) for key, _ in labels)
        return "  →  ".join(parts) + (
            "\n本台流程完成（仅本次记录）" if complete else "\n本台流程未完成"
        )
