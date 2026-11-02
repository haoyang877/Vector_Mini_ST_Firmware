"""冻结修改前 FOC/PI，比较实际入口状态与PWM顺序；只用主机输出桩。"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
from _motor_config_projection import project_reference_motor_config
from project_paths import NATIVE_INCLUDE_FLAGS, ROOT


def main() -> None:
    """参考只生成在outputs，比较运算结果与PWM调用。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", required=True)
    parser.add_argument("--out", type=Path, default=ROOT / "outputs/foc_equivalence")
    parser.add_argument("--reference-ref", default="05d98016")
    args = parser.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)

    def frozen(path: str) -> str:
        """读取固定提交，禁止将当前实现作为自身参考。"""
        return subprocess.check_output(
            ["git", "show", args.reference_ref + ":" + path], cwd=ROOT
        ).decode("utf-8")

    foc = project_reference_motor_config(frozen("firmware/motor/foc/foc_algorithm.c"))
    foc = foc.replace('"control_config.h"', '"application_config.h"')
    # 只投影退役的PWM头文件与输出符号，不改变冻结参考的运算和控制分支。
    foc = foc.replace('#include "motor_hw.h"', '#include "bsp_pwm.h"').replace(
        "motor_hw_pwm_set_duty", "BspPwm_SetDuty"
    )
    # 退役的诊断钩子从冻结参考投影中一并移除，不改算术与控制分支。
    foc = foc.replace('#include "fast_loop_profile.h"', "")
    foc = re.sub(r"(?m)^\s*FAST_PROFILE_(?:BEGIN|END)\([^\n]*\);\n", "", foc)
    # 唯一有意改变的控制行为：输出角度由1Ts提前改为1.5Ts；其余冻结参考不变。
    advance = "phase + phase_vel * Current_Ts"
    if foc.count(advance) != 2:
        raise ValueError("冻结参考的两个输出角度预测位置发生变化")
    foc = foc.replace(advance, "phase + phase_vel * (1.5f * Current_Ts)")
    pi = frozen("firmware/motor/foc/foc_pid.c")
    foc = foc.replace('"foc_algorithm.h"', '"foc.h"').replace('"foc_pid.h"', '"pi_controller.h"')
    pi = pi.replace('"foc_pid.h"', '"pi_controller.h"')
    # 仅投影符号名称，参考算法的运算与调用次序保持不变。
    symbol_map = {
        "FOC_MaximumVoltage": "Foc_MaxVoltage",
        "FOC_ConfigureControllers": "Foc_ConfigureControllers",
        "FOC_UpdateCurrentControllerLimits": "Foc_UpdateCurrentLimits",
        "FOC_Step": "Foc_Step",
        "FOC_CurrentWithReference": "Foc_RunCurrentRef",
        "FOC_CurrentController_Reset": "Foc_ResetCurrent",
        "FOC_Current": "Foc_RunCurrent",
        "FOC_Vq_Mode": "Foc_RunVq",
        "SVM_ZeroInjection": "Svm_ZeroInjection",
        "SVM_SectorJudge": "Svm_SectorJudge",
        "Inverse_Clarke_Transform": "Clarke_Inverse",
        "Inverse_Park_Transform": "Park_Inverse",
    }
    for old_name, new_name in symbol_map.items():
        foc = re.sub(r"\b" + old_name + r"\b", new_name, foc)
        pi = re.sub(r"\b" + old_name + r"\b", new_name, pi)
    names = set(re.findall(r"^(?:float|void|bool)\s+(\w+)\s*\(", foc + "\n" + pi, re.M))
    # 仅投影函数名，常量、算术次序、控制分支及结构保持冻结内容。
    defines = "\n".join(
        "#define " + n + " " + n + "_Reference" for n in sorted(names | {"BspPwm_SetDuty"})
    )
    # 冻结拆分前的状态布局，参考代码仍用自己的字段；当前实现单独使用测量对象。
    legacy_header = (ROOT / "tests/fixtures/foc_before_measurement.h").read_text(encoding="utf-8")
    legacy_type = re.search(r"typedef struct\s*\{.*?\} FocState;", legacy_header, re.S)
    if legacy_type is None:
        raise ValueError("冻结 FocState 缺失")
    (out / "reference_type.h").write_text(
        legacy_type.group(0).replace("FocState;", "FocReference;"), encoding="utf-8"
    )
    (out / "reference_foc.h").write_text(legacy_header, encoding="utf-8")
    foc = foc.replace('"foc.h"', '"reference_foc.h"')
    (out / "reference_foc.c").write_text(
        defines + "\n#define Current_Ts FOC_TS_S\n#define FocState FocReference\n" + foc,
        encoding="utf-8",
    )
    (out / "reference_pi.c").write_text(defines + "\n" + pi, encoding="utf-8")
    cc = [args.cc] + (["cc"] if Path(args.cc).stem == "zig" else [])
    logs = []
    exe = out / "foc_equivalence.exe"
    command = (
        cc
        + NATIVE_INCLUDE_FLAGS
        + ["-DBSP_BOARD=1", "-DFOC_ACTIVE_MOTOR_PROFILE=1"]
        + ["-I", str(out)]
        + [
            "-std=c99",
            "-O2",
            "-UNDEBUG",
            "-Wall",
            "-Wextra",
            "-Werror",
            "tests/unit/foc_equivalence_test.c",
            "firmware/motor/control/foc.c",
            "firmware/motor/control/foc_output.c",
            "firmware/motor/control/pi_controller.c",
            "firmware/motor/math/control_math.c",
            "firmware/motor/protection/motor_thermal.c",
            "firmware/motor/protection/motor_thermal_config.c",
            "firmware/motor/protection/motor_thermal_control.c",
            str(out / "reference_foc.c"),
            str(out / "reference_pi.c"),
            "-o",
            str(exe),
        ]
    )
    for invocation in (command, [str(exe)]):
        result = subprocess.run(invocation, cwd=ROOT, capture_output=True, text=True)
        logs.append(result.stdout + result.stderr)
        (out / "test.log").write_text("\n".join(logs), encoding="utf-8")
        print(logs[-1], end="")
        result.check_returncode()


if __name__ == "__main__":
    main()
