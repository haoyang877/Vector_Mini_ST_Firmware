"""冻结修改前 FOC/PI，比较实际入口状态、PWM 与诊断钩子顺序；只用主机输出桩。"""

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
    """参考只生成在 outputs，开启/关闭计时宏各比较一次。"""
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
    # 只投影退役的PWM头文件与输出符号，不改变冻结参考的运算和控制分支。
    foc = foc.replace('#include "motor_hw.h"', '#include "bsp_pwm.h"').replace(
        "motor_hw_pwm_set_duty", "BspPwm_SetDuty"
    )
    pi = frozen("firmware/motor/foc/foc_pid.c")
    names = set(re.findall(r"^(?:float|void|bool)\s+(\w+)\s*\(", foc + "\n" + pi, re.M))
    # 仅投影函数名，常量、算术次序、控制分支及结构保持冻结内容。
    defines = "\n".join(
        "#define " + n + " " + n + "_Reference"
        for n in sorted(names | {"BspPwm_SetDuty", "FastLoopProfile_Begin", "FastLoopProfile_End"})
    )
    (out / "reference_foc.c").write_text(defines + "\n" + foc, encoding="utf-8")
    (out / "reference_pi.c").write_text(defines + "\n" + pi, encoding="utf-8")
    cc = [args.cc] + (["cc"] if Path(args.cc).stem == "zig" else [])
    logs = []
    for profile in (0, 1):
        exe = out / f"foc_profile_{profile}.exe"
        command = (
            cc
            + NATIVE_INCLUDE_FLAGS
            + [
                "-std=c99",
                "-O2",
                "-UNDEBUG",
                "-Wall",
                "-Wextra",
                "-Werror",
                f"-DBSP_ENABLED_FAST_LOOP_PROFILE={profile}",
                "tests/unit/foc_equivalence_test.c",
                "firmware/motor/foc/foc_algorithm.c",
                "firmware/motor/foc/foc_pid.c",
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
