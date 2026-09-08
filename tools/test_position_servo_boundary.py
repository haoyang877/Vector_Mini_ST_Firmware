import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SERVO_HEADER = ROOT / "Foc" / "position_cascade.h"
SERVO_SOURCE = ROOT / "Foc" / "position_cascade.c"
FOC_ADAPTER = ROOT / "Foc" / "foc_run.c"


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
            "MotorControl_TypeDef",
            "FOC_TypeDef",
            "Encoder_TypeDef",
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
        self.assertIn("MotorControl->friction_model_valid", adapter)
        self.assertIn("MotorControl->friction_coulomb_pos_a", adapter)
        self.assertIn("POSITION_IMPEDANCE_FRICTION_POSITIVE_A", adapter)
        self.assertIn("POSITION_SERVO_ACCEL_FF_GAIN_A_PER_RAD_S2", adapter)
        self.assertIn("friction_fast_release_slew_rate", public_api)
        self.assertIn(
            "POSITION_SERVO_FRICTION_FAST_RELEASE_SLEW_A_PER_S", adapter
        )

    def test_servo_exports_coherent_diagnostics(self) -> None:
        public_api = SERVO_HEADER.read_text(encoding="utf-8")
        source = SERVO_SOURCE.read_text(encoding="utf-8")
        self.assertIn("PositionCascadeTelemetry_TypeDef", public_api)
        self.assertIn("PositionCascade_GetTelemetry", public_api)
        self.assertIn("trajectory_speed_reference", public_api)
        self.assertIn("current_saturated", public_api)
        self.assertIn("friction_landing_active", public_api)
        self.assertIn("settle_recovery_active", public_api)
        self.assertIn("hold_candidate_active", public_api)
        self.assertIn("feedforward_candidate > config->current_limit", source)

    def test_landing_and_settle_recovery_use_measured_state(self) -> None:
        source = SERVO_SOURCE.read_text(encoding="utf-8")
        self.assertIn(
            "target_error = config->target_position - measured_position", source
        )
        self.assertIn("speed_toward_target * speed_toward_target", source)
        self.assertIn("state.settle_recovery_active = true", source)
        self.assertIn("Latch direction so crossing the target starts landing", source)
        self.assertIn(
            "inside_braking_envelope = remaining_distance <=", source
        )
        self.assertRegex(
            source,
            r"(?s)controller_requests_braking\s*=.*?inside_braking_envelope;",
        )
        self.assertRegex(
            source,
            r"state\.friction_landing_active = true;\s*"
            r"state\.settle_recovery_active = false;",
        )
        self.assertIn("opposes_speed_correction", source)
        self.assertIn("state.hold_counter > 0U", source)
        self.assertIn("hold_exit_position", source)
        self.assertIn("PositionCascade_UpdateIntegralTransport", source)
        self.assertIn("state.target_transition_active", source)


if __name__ == "__main__":
    unittest.main()
