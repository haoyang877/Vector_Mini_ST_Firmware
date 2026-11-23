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
            "production_inspection",
        ):
            return
        device = self.devices.setdefault(uid, dict(uid=uid, steps={}, records=[]))
        steps = device["steps"]
        if profile in ("jlink_flash", "jlink_erase"):
            steps.clear()
            device.pop("inspection", None)
        elif profile == "production_inspection":
            for key in ("hardware", "angle", "zero", "recheck"):
                steps.pop(key, None)
            device["inspection"] = "进行中"
        elif profile == "motor_calibration":
            device.pop("inspection", None)
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
        if not uid and profile == "production_inspection" and not report.get("passed"):
            # 尚未取得板端UID的失败或排队取消，也须结束已绑定设备的本轮状态。
            uid = report.get("expected_uid")
        if not uid:
            return
        if profile == "parameters" and (report.get("applied") or report.get("save_sent")):
            device = self.devices.get(uid, {})
            device.pop("inspection", None)
            steps = device.get("steps", {})
            for key in ("hardware", "recheck"):
                steps.pop(key, None)
            return
        if profile not in (
            "jlink_flash",
            "hardware_check",
            "motor_calibration",
            "production_inspection",
        ):
            return
        if report.get("needs_initial_setup"):
            return
        device = self.devices.setdefault(uid, dict(uid=uid, steps={}, records=[]))
        steps = device["steps"]
        if profile == "production_inspection":
            device["inspection"] = report.get("verdict", "未通过 / 未完成")
        elif profile == "jlink_flash":
            steps.clear()
            device.pop("inspection", None)
            steps["flash"] = bool(
                report.get("success")
                and report.get("app_started")
                and report.get("can_test", {}).get("version_verified")
            )
            if (
                not steps["flash"]
                and report.get("success")
                and report.get("app_started")
                and report.get("protected_regions_verified")
                and report.get("swd_versions")
                and "can_test" not in report
            ):
                # SWD 已通过但未执行 CAN 验收；既不是失败，也不能计为完整通过。
                steps["flash"] = None
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

    def summary(self, uid: str | None, *, integrated: bool = False) -> str:
        """只汇总当前UID的本次证据；完整顺序要求不代替整机出厂验收。"""
        device = self.devices.get(uid, {})
        steps = device.get("steps", {})
        if integrated or "inspection" in device:
            flash = (
                "待 CAN 验证"
                if "flash" in steps and steps["flash"] is None
                else ("完成" if steps.get("flash") else "未确认")
            )
            return f"烧录：{flash}    整轮生产检测：{device.get('inspection', '未开始')}"
        labels = (
            ("flash", "烧录"),
            ("hardware", "完整检测"),
            ("angle", "角度标定"),
            ("zero", "电零位"),
            ("recheck", "完整复检"),
        )
        parts = [
            f"{name}：待 CAN 验证"
            if key == "flash" and key in steps and steps[key] is None
            else f"{name}：{'完成' if steps[key] else '未通过'}"
            if key in steps
            else f"{name}：未完成"
            for key, name in labels
        ]
        complete = all(steps.get(key) for key, _ in labels)
        return "  →  ".join(parts) + (
            "\n本台流程完成（仅本次记录）" if complete else "\n本台流程未完成"
        )
