"""以真实SDK声明编译MT6835端口，覆盖帧、异常收尾与既有角度层。"""

import sys as _sys
from pathlib import Path as _Path

_sys.path.insert(0, str(_Path(__file__).resolve().parents[3] / "tools"))
import argparse
import subprocess
from pathlib import Path

from project_paths import NATIVE_INCLUDE_FLAGS, ROOT
from run_position_servo_tests import function_source


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    native = ROOT / "tests/unit/native"
    angle_source = (ROOT / "firmware/motor/feedback/angle_feedback.c").read_text(encoding="utf-8")
    functions = (
        "encoder_mark_read_status",
        "Encoder_BeginSample",
        "Encoder_BeginReadback",
        "encoder_map_sensor_status",
        "encoder_read_frame",
        "encoder_apply_direction_q15",
        "encoder_update_shadow",
    )
    (args.out / "r40_angle_contract.inc").write_text(
        '#include "angle_feedback.h"\n'
        + "\n".join(function_source(angle_source, name) for name in functions),
        encoding="utf-8",
    )
    command = [args.cc] + (["cc"] if Path(args.cc).stem == "zig" else [])
    command += ["-std=c11", "-O2", "-UNDEBUG", "-Wall", "-Wextra", "-Werror"]
    for directory in (
        native / "r40_encoder_fake",
        ROOT / "firmware/third_party/at32f45x/drivers/inc",
        ROOT / "firmware/platform/at32f456/bsp",
        ROOT / "firmware/platform/api",
    ):
        command += ["-I", str(directory)]
    command += ["-I", str(args.out.resolve())] + NATIVE_INCLUDE_FLAGS
    executable = args.out.resolve() / "r40_encoder.exe"
    subprocess.run(
        command + [str(native / "r40_encoder_test.c"), "-o", str(executable)], check=True
    )
    for scenario in range(3):
        subprocess.run([str(executable), str(scenario)], check=True)
    app_executable = args.out.resolve() / "r40_encoder_app.exe"
    subprocess.run(
        command
        + ["-DR40_APP_BINDINGS=1", str(native / "r40_encoder_test.c"), "-o", str(app_executable)],
        check=True,
    )
    for scenario in range(3):
        subprocess.run([str(app_executable), str(scenario)], check=True)
    fast_executable = args.out.resolve() / "r40_encoder_fast_spi.exe"
    subprocess.run(
        command
        + [
            "-DR40_APP_BINDINGS=1",
            "-DR40_MT6835_FAST_SPI=1",
            str(native / "r40_encoder_test.c"),
            "-o",
            str(fast_executable),
        ],
        check=True,
    )
    for scenario in range(3):
        subprocess.run([str(fast_executable), str(scenario)], check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
