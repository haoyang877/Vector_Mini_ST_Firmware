"""约束状态所有权和功率许可，防止旧模式体系重新进入生产调用链。"""

import re
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from project_paths import ROOT

sys.path.insert(0, str(ROOT / "tests/unit/native"))
from run_position_servo_tests import function_source


def production_text(path: Path) -> str:
    """去除说明文字，只检查 C 生产声明和调用。"""
    return re.sub(r"/\*.*?\*/|//[^\n]*", "", path.read_text(encoding="utf-8"), flags=re.S)


class MotorStateMachineBoundaryTests(unittest.TestCase):
    def test_watch_objects_are_not_cross_module_data_interfaces(self) -> None:
        """Watch符号保留；生产模块不得自行extern或直接使用其它所有者的运行对象。"""
        owners = {
            "firmware/app/motor_axis.c": (
                "s_motor_configuration",
                "s_motor_state",
                "s_foc_state",
                "s_rotor_encoder",
                "s_speed_pi",
                "s_flux_observer",
                "s_sensorless_startup",
                "s_current_offset_calibration",
            ),
        }
        directories = [
            ROOT / "firmware" / name
            for name in (
                "app",
                "motor",
                "parameters",
                "services",
                "communication",
                "platform/bsp/include",
                "platform/bsp/mcu/stm32g4",
                "platform/bsp/mcu/at32f456",
            )
        ] + [ROOT / "shared"]
        for directory in directories:
            for path in directory.rglob("*.[ch]"):
                relative = path.relative_to(ROOT).as_posix()
                source = production_text(path)
                for owner, names in owners.items():
                    if relative != owner:
                        for name in names:
                            self.assertIsNone(re.search(rf"\b{name}\b", source), (relative, name))
                if relative != "firmware/app/motor_lifecycle.c":
                    self.assertIsNone(
                        re.search(r"\bextern\b[^;{}]*\bs_context\b", source), relative
                    )

    def test_runtime_does_not_implement_control_or_thermal_algorithms(self) -> None:
        """runtime调用motor能力，不能再拥有外环、当前模式或热保护计算的实现。"""
        implementations = (
            r"\b(?:void|bool|float|MotorTaskResult)\s+"
            r"(?:MotorOuterLoop_\w+|MotorThermalControl_\w+|MotorCurrentOffsets_\w+|"
            r"Foc_ConfigureControllers|Foc_UpdateCurrentLimits|"
            r"Task_(?:Current|Vq)_Mode|MotorControl_Step|MotorExecution_ControlStep)"
            r"\s*\([^;{}]*\)\s*\{"
        )
        for path in (ROOT / "firmware/app").glob("*.[ch]"):
            self.assertIsNone(
                re.search(implementations, production_text(path)), path.relative_to(ROOT).as_posix()
            )

    def test_parameter_transactions_have_explicit_callers(self) -> None:
        """事务统一使用启动绑定的对象；适配器不取得可写配置或控制器地址。"""
        permitted = {
            "firmware/app/motor_axis.c": {"Init", "AdvanceRevision"},
            "firmware/app/app_protocol.c": {"Read", "Write"},
            "firmware/app/app.c": {"NodeId"},
        }
        for directory in ("app", "motor", "parameters", "services", "communication"):
            for path in (ROOT / "firmware" / directory).rglob("*.c"):
                relative = path.relative_to(ROOT).as_posix()
                if relative == "firmware/app/motor_parameter_transaction.c":
                    continue
                calls = set(
                    re.findall(r"\bMotorParameterTransaction_(\w+)\s*\(", production_text(path))
                )
                self.assertLessEqual(calls, permitted.get(relative, set()), relative)

    def test_unused_maintenance_query_is_retired(self) -> None:
        """维护类型使用现有查询，不保留只有测试调用的重复接口。"""
        for directory in ("app", "motor", "parameters", "services"):
            for path in (ROOT / "firmware" / directory).rglob("*.[ch]"):
                self.assertNotIn("MotorLifecycle_IsMaintenanceActive", production_text(path))

    def test_state_module_has_no_goto(self) -> None:
        """状态机使用结构化分支，禁止跳转逻辑重新进入生产模块。"""
        for path in (ROOT / "firmware/app").glob("*.[ch]"):
            self.assertIsNone(re.search(r"\bgoto\b", production_text(path)), path.as_posix())

    def test_state_core_does_not_own_request_history_or_background_jobs(self) -> None:
        """状态核心只计算迁移，功率动作和保存协议留在各自实现。"""
        module = ROOT / "firmware/app"
        core = function_source(
            production_text(module / "motor_state_machine.c"), "MotorStateMachine_NextState"
        ).split("{", 1)[1]
        for token in (
            "last_maintenance_request",
            "latest_start",
            "active_start",
            "stop_sequence",
            "save_finish",
            "FocBackgroundClaim",
            "MotorMaintenance_Cancel",
            "MotorParameterTransaction_",
            "MotorLifecycle_SubmitControl",
        ):
            self.assertNotIn(token, core)
        runtime = production_text(ROOT / "firmware/app/motor_lifecycle.c")
        for token in (
            "MotorMaintenance",
            "FocBackground",
            "last_maintenance",
            "latest_start",
            "active_start",
            "save_finish",
            "ReadControlProgress",
            "RequestIdentified",
        ):
            self.assertNotIn(token, runtime)
        for path in module.glob("*.c"):
            if path.name != "motor_lifecycle.c":
                self.assertIsNone(re.search(r"\bs_context\b", production_text(path)), path.name)

    def test_keil_projects_include_single_runtime_owner(self) -> None:
        """每个状态机工程只编译一份生命周期实现，不再编译退役转发层。"""
        for path in (ROOT / "projects").rglob("*.uvprojx"):
            text = path.read_text(encoding="utf-8-sig")
            if "<FileName>motor_lifecycle.c</FileName>" in text:
                for name in (
                    "motor_state_machine.c",
                    "motor_lifecycle.c",
                    "motor_lifecycle_exchange.c",
                ):
                    self.assertEqual(text.count(f"<FileName>{name}</FileName>"), 1, path.name)
                self.assertNotRegex(
                    text, r"<FileName>motor_lifecycle_(?:requests|maintenance|safety)\.c"
                )
                self.assertNotIn("<FileName>motor_command.c</FileName>", text)

    def test_no_legacy_state_contract(self) -> None:
        retired = (
            r"\b(?:ModeNow|ModeLast|ModeNow_TypeDef|MotorWorkOutcome_TypeDef|next_mode|power_off)\b"
        )
        for directory in ("app", "motor", "parameters", "services"):
            for path in (ROOT / "firmware" / directory).rglob("*"):
                if path.suffix in (".c", ".h"):
                    self.assertIsNone(re.search(retired, production_text(path)), path.as_posix())
        for name in ("foc_run", "foc_run_state", "foc_mode_dispatch"):
            for suffix in (".c", ".h"):
                self.assertFalse((ROOT / "firmware/app" / (name + suffix)).exists())
        self.assertFalse((ROOT / "firmware/motor/motor_work.h").exists())

    def test_normal_enable_has_one_owner(self) -> None:
        callers = set()
        for directory in ("app", "motor", "parameters", "services", "communication"):
            for path in (ROOT / "firmware" / directory).rglob("*.c"):
                if re.search(r"\bpower->enable\s*\(", production_text(path)):
                    callers.add(path.relative_to(ROOT).as_posix())
        self.assertEqual(callers, {"firmware/app/motor_lifecycle.c"})

    def test_five_state_core_has_no_old_framework(self) -> None:
        """仅保留五态纯迁移；自检占位，初始化直接空闲。"""
        module = ROOT / "firmware/app"
        self.assertEqual(
            {p.name for p in module.glob("*.c")},
            {
                "app.c",
                "app_loader.c",
                "app_protocol.c",
                "motor_lifecycle.c",
                "motor_lifecycle_exchange.c",
                "motor_state_machine.c",
                "motor_axis.c",
                "motor_calibration.c",
            },
        )
        for path in (ROOT / "firmware/app").rglob("*.[ch]"):
            source = production_text(path)
            self.assertNotRegex(
                source, r"MOTOR_STATE_(?:INIT|STARTING|STOPPING|MAINTENANCE|FATAL)\b"
            )
            self.assertNotIn("MotorStateRules_", source)
        core = function_source(
            production_text(module / "motor_state_machine.c"), "MotorStateMachine_NextState"
        ).split("{", 1)[1]
        self.assertNotRegex(core, r"Bsp|MotorMaintenance|MotorLifecycle|g_motor_context")
        owner = production_text(ROOT / "firmware/app/motor_lifecycle.c")
        writes = re.compile(r"\blife->state\s*=(?!=)")
        self.assertEqual(len(writes.findall(owner)), 2)
        self.assertIn(
            "life->state = MOTOR_STATE_IDLE;", function_source(owner, "MotorLifecycle_Init")
        )
        self.assertIn(
            "life->state = MotorStateMachine_NextState", function_source(owner, "apply_state_event")
        )
        exchange = production_text(module / "motor_lifecycle_exchange.c")
        self.assertNotRegex(exchange, writes)
        self.assertNotRegex(
            exchange, r"\bphase_enable_epoch\s*=(?!=)|\bpower->(?:enable|disable)\("
        )
        self.assertNotRegex(owner, r"\b(?:run|execute|start)_self_test\s*\(")
        self.assertNotIn("MotorLifecycle_ReadReadiness", owner)


if __name__ == "__main__":
    unittest.main()
