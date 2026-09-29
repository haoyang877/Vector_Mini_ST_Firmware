"""验证冻结源码改名投影的边界，防止把成员访问、比较符号和字符串混为一谈。"""

import importlib.util
import unittest
from pathlib import Path

_SPEC = importlib.util.spec_from_file_location(
    "_naming_reference", Path(__file__).parent / "native/_naming_reference.py"
)
_MODULE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MODULE)
project_reference = _MODULE.project_reference


class NamingReferenceTests(unittest.TestCase):
    """只验证历史测试词汇变换，不替代生产行为或 ABI 检查。"""

    def test_private_ownership_and_pointer_comparison(self):
        source = "MotorControl_TypeDef *motor; FOC > 0; FOC.Iq; object->FOC;"
        self.assertEqual(
            project_reference(source, "firmware/app/motor_state.c"),
            "MotorControlState *motor; s_foc_state > 0; s_foc_state.iq_a; object->foc;",
        )

    def test_literals_numbers_and_layout_are_preserved(self):
        source = 'FOC_TypeDef FOC; const char *label = "FOC Iq"; float x = -0.5f;'
        self.assertEqual(
            project_reference(source),
            'FocState foc; const char *label = "FOC Iq"; float x = -0.5f;',
        )

    def test_filename_is_not_a_member_prefix(self):
        source = '#include "motor_state.c"\nmotor_state.control_mode = 3;'
        self.assertEqual(
            project_reference(source),
            '#include "motor_axis.c"\nmotor_state.control_mode = 3;',
        )

    def test_unrelated_member_is_not_private_state(self):
        source = "state.phase = packet.state; return state > 0;"
        self.assertEqual(
            project_reference(source, "firmware/motor/position/position_cascade.c"),
            "s_state.phase = packet.state; return s_state > 0;",
        )


if __name__ == "__main__":
    unittest.main()
