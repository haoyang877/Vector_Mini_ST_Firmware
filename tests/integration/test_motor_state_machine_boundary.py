"""约束状态所有权和功率许可，防止旧模式体系重新进入生产调用链。"""

import re
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from project_paths import ROOT


def production_text(path: Path) -> str:
    """去除说明文字，只检查 C 生产声明和调用。"""
    return re.sub(r"/\*.*?\*/|//[^\n]*", "", path.read_text(encoding="utf-8"), flags=re.S)


class MotorStateMachineBoundaryTests(unittest.TestCase):
    def test_watch_objects_are_not_cross_module_data_interfaces(self) -> None:
        """Watch符号保留；生产模块不得自行extern或直接使用其它所有者的运行对象。"""
        owners = {
            "firmware/app/runtime/motor_axis.c": (
                "s_motor_configuration",
                "s_motor_state",
                "s_foc_state",
                "s_rotor_encoder",
                "s_speed_pi",
                "s_flux_observer",
                "s_sensorless_startup",
                "s_current_offset_calibration",
            ),
            "firmware/app/runtime/motor_parameter_transaction.c": (
                "s_config_revision",
                "s_revision_work_epoch",
                "s_last_read",
                "s_last_write",
                "s_save_job",
            ),
        }
        directories = [
            ROOT / "firmware" / name
            for name in (
                "app",
                "motor",
                "services",
                "communication",
                "platform/api",
                "platform/stm32g4/bsp",
                "platform/stm32g4/ports",
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
                if relative != "firmware/app/motor_state_machine/motor_state_machine.c":
                    self.assertIsNone(
                        re.search(r"\bextern\b[^;{}]*\bs_context\b", source), relative
                    )

    def test_runtime_does_not_implement_control_or_thermal_algorithms(self) -> None:
        """runtime调用motor能力，不能再拥有外环、当前模式或热保护计算的实现。"""
        implementations = (
            r"\b(?:void|bool|float|MotorTaskResult)\s+"
            r"(?:MotorOuterLoop_\w+|MotorThermalControl_\w+|MotorCurrentOffsets_\w+|"
            r"FOC_ConfigureControllers|FOC_UpdateCurrentControllerLimits|"
            r"Task_(?:Current|Vq)_Mode|MotorExecution_ControlStep)"
            r"\s*\([^;{}]*\)\s*\{"
        )
        for path in (ROOT / "firmware/app/runtime").glob("*.[ch]"):
            self.assertIsNone(
                re.search(implementations, production_text(path)), path.relative_to(ROOT).as_posix()
            )

    def test_parameter_mutations_enter_through_axis_owner(self) -> None:
        """协议/业务适配不得绕过Axis，直接调用接受可写配置的内部参数事务。"""
        permitted = {
            "firmware/app/runtime/motor_axis.c",
            "firmware/app/runtime/motor_parameter_transaction.c",
        }
        for directory in ("app", "motor", "services", "communication"):
            for path in (ROOT / "firmware" / directory).rglob("*.c"):
                relative = path.relative_to(ROOT).as_posix()
                if relative not in permitted:
                    self.assertIsNone(
                        re.search(r"\bMotorParameterTransaction_\w+\s*\(", production_text(path)),
                        relative,
                    )

    def test_unused_maintenance_query_is_retired(self) -> None:
        """维护类型使用现有查询，不保留只有测试调用的重复接口。"""
        for directory in ("app", "motor", "services"):
            for path in (ROOT / "firmware" / directory).rglob("*.[ch]"):
                self.assertNotIn("MotorStateMachine_IsMaintenanceActive", production_text(path))

    def test_state_module_has_no_goto(self) -> None:
        """状态机使用结构化分支，禁止跳转逻辑重新进入生产模块。"""
        for path in (ROOT / "firmware/app/motor_state_machine").glob("*.[ch]"):
            self.assertIsNone(re.search(r"\bgoto\b", production_text(path)), path.as_posix())

    def test_no_legacy_state_contract(self) -> None:
        retired = (
            r"\b(?:ModeNow|ModeLast|ModeNow_TypeDef|MotorWorkOutcome_TypeDef|next_mode|power_off)\b"
        )
        for directory in ("app", "motor", "services"):
            for path in (ROOT / "firmware" / directory).rglob("*"):
                if path.suffix in (".c", ".h"):
                    self.assertIsNone(re.search(retired, production_text(path)), path.as_posix())
        for name in ("foc_run", "foc_run_state", "foc_mode_dispatch"):
            for suffix in (".c", ".h"):
                self.assertFalse((ROOT / "firmware/app" / (name + suffix)).exists())
        self.assertFalse((ROOT / "firmware/motor/motor_work.h").exists())

    def test_normal_enable_has_one_owner(self) -> None:
        callers = set()
        for directory in ("app", "motor", "services", "communication"):
            for path in (ROOT / "firmware" / directory).rglob("*.c"):
                if re.search(r"\bpower_stage_hw_start\s*\(", production_text(path)):
                    callers.add(path.relative_to(ROOT).as_posix())
        self.assertEqual(callers, {"firmware/app/motor_state_machine/motor_state_machine.c"})

    def test_private_transition_rules_have_one_production_owner(self) -> None:
        module = ROOT / "firmware/app/motor_state_machine"
        self.assertFalse((ROOT / "firmware/services/lifecycle/app_lifecycle.c").exists())
        self.assertFalse((ROOT / "firmware/services/lifecycle/app_lifecycle.h").exists())
        owned_files = [
            path
            for directory in ("app", "motor", "services", "communication")
            for path in (ROOT / "firmware" / directory).rglob("*")
        ]
        for path in owned_files:
            if path.suffix not in (".c", ".h"):
                continue
            if path.parent != module:
                self.assertNotIn('"motor_state_internal.h"', production_text(path), str(path))
        callers = {
            path.relative_to(ROOT).as_posix()
            for path in owned_files
            if path.suffix == ".c"
            and path.name != "motor_state_rules.c"
            and re.search(r"\bMotorStateRules_Step\s*\(", production_text(path))
        }
        self.assertEqual(callers, {"firmware/app/motor_state_machine/motor_state_machine.c"})


if __name__ == "__main__":
    unittest.main()
