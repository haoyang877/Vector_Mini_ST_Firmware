# 位置伺服边界检查：保持配置与算法层独立，验证当前前馈回退入口。
import sys as _sys
from pathlib import Path as _Path

_sys.path.insert(0, str(_Path(__file__).resolve().parents[2] / "tools"))
import re
import unittest

from project_paths import ROOT

_sys.path.insert(0, str(ROOT / "tests/unit/native"))
from run_position_servo_tests import function_source

SERVO_HEADER = ROOT / "firmware/motor/position/position_cascade.h"
SERVO_SOURCE = ROOT / "firmware/motor/position/position_cascade.c"
FOC_ADAPTER = ROOT / "firmware/app/runtime/motor_execution.c"


class PositionServoBoundaryTests(unittest.TestCase):
    def test_servo_core_has_no_vendor_or_board_dependency(self) -> None:
        includes = re.findall(
            r'^#include\s+([<"][^>"]+[>"])',
            SERVO_SOURCE.read_text(encoding="utf-8"),
            flags=re.MULTILINE,
        )
        self.assertEqual(
            includes,
            [
                '"fast_loop_profile.h"',
                '"position_cascade.h"',
                "<math.h>",
                "<stdint.h>",
                "<stddef.h>",
                "<string.h>",
                '"foc_pid.h"',
                '"position_cascade_config.h"',
                '"position_smooth_trajectory.h"',
            ],
        )

    def test_public_api_has_no_firmware_or_mcu_types(self) -> None:
        public_api = SERVO_HEADER.read_text(encoding="utf-8")
        for forbidden in (
            "MotorControlState",
            "FocState",
            "EncoderState",
            "TIM1",
            "ADC1",
            "HAL_",
            "LL_",
            "stm32",
        ):
            self.assertNotIn(forbidden, public_api)

    def test_adapter_selects_identified_model_and_safe_fallback(self) -> None:
        adapter = FOC_ADAPTER.read_text(encoding="utf-8")
        public_api = SERVO_HEADER.read_text(encoding="utf-8")
        self.assertIn("motor_state->device.friction_model_valid", adapter)
        self.assertIn("motor_state->device.friction_coulomb_pos_a", adapter)
        self.assertIn("POSITION_SERVO_FALLBACK_FRICTION_POSITIVE_A", adapter)
        self.assertIn("POSITION_SERVO_ACCEL_FF_GAIN_A_PER_RAD_S2", adapter)
        self.assertIn("friction_fast_release_slew_rate", public_api)
        self.assertIn("POSITION_SERVO_FRICTION_FAST_RELEASE_SLEW_A_PER_S", adapter)

    def test_servo_exports_coherent_diagnostics(self) -> None:
        public_api = SERVO_HEADER.read_text(encoding="utf-8")
        source = SERVO_SOURCE.read_text(encoding="utf-8")
        self.assertIn("PositionCascadeTelemetry", public_api)
        self.assertIn("PositionCascade_GetTelemetry", public_api)
        self.assertIn("trajectory_speed_reference", public_api)
        self.assertIn("current_saturated", public_api)
        self.assertIn("friction_landing_active", public_api)
        self.assertIn("settle_recovery_active", public_api)
        self.assertIn("hold_candidate_active", public_api)
        self.assertIn("input->feedforward_candidate_a > current_limit_a", source)

    def test_landing_and_settle_recovery_use_measured_state(self) -> None:
        source = SERVO_SOURCE.read_text(encoding="utf-8")
        self.assertIn("target_error = input->target_position_rad - measured_position", source)
        self.assertIn("speed_toward_target * speed_toward_target", source)
        self.assertIn("state->recovery_active = true", source)
        self.assertRegex(
            source,
            r"desired_direction\s*=\s*state->direction\s*!=\s*0\s*\?\s*"
            r"state->direction\s*:\s*\(target_error\s*>\s*0\.0f\s*\?\s*1\s*:\s*-1\)",
        )
        self.assertRegex(source, r"inside_braking_envelope\s*=\s*remaining_distance\s*<=")
        self.assertRegex(
            source,
            r"state->landing_active = true;\s*"
            r"state->recovery_active = false;",
        )
        self.assertIn("opposes_speed_correction", source)
        self.assertIn("state.hold_counter > 0U", source)
        self.assertIn("hold_exit_position", source)
        self.assertIn("position_cascade_integral_transport_request", source)
        self.assertIn("state.target_transition_active", source)

    def test_private_algorithms_use_explicit_state_and_narrow_inputs(self) -> None:
        source = SERVO_SOURCE.read_text(encoding="utf-8")
        names = sorted(set(re.findall(r"\b(position_cascade_(?:core|friction)_\w+)\s*\(", source)))
        for name in names:
            block = function_source(source, name)
            self.assertNotIn("s_state", block)
            self.assertNotIn("PositionCascadeConfig", block)
            self.assertNotIn("POSITION_SERVO_PHASE_", block)
        friction = function_source(source, "position_cascade_friction_update")
        self.assertNotIn("PiController", friction)
        self.assertNotIn("speed_controller", friction)
        self.assertNotIn("position_cascade_core_", friction)
        coordinator = function_source(source, "position_cascade_run_control")
        self.assertNotIn("s_state.core.speed_controller", coordinator)
        self.assertNotRegex(coordinator, r"s_state\.friction\.\w+\s*=")


if __name__ == "__main__":
    unittest.main()
