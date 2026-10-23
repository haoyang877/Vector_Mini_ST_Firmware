"""验证AT32参数存储及温度端口；仅硬件地址访问由离线模型代替。"""

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
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    native = ROOT / "tests/unit/native"
    platform = ROOT / "firmware/platform/at32f456"
    source = (platform / "ports/param_store_at32f456.c").read_text(encoding="utf-8")
    for primitive in ("flash_memory", "flash_store_word"):
        source = source.replace(function_source(source, primitive), "")
    (out / "r40_storage_production.inc").write_text(source, encoding="utf-8")
    source = (ROOT / "firmware/motor/foc/foc_sensing.c").read_text(encoding="utf-8")
    (out / "r40_temperature_production.inc").write_text(
        function_source(source, "Temperature_Update"), encoding="utf-8"
    )
    command = [args.cc] + (["cc"] if Path(args.cc).stem == "zig" else [])
    command += ["-std=c11", "-O2", "-UNDEBUG", "-Wall", "-Wextra", "-Werror"]
    for include in (
        native / "r40_storage_fake",
        ROOT / "firmware/third_party/at32f45x/drivers/inc",
        platform / "bsp",
        out,
    ):
        command += ["-I", str(include)]
    command += NATIVE_INCLUDE_FLAGS
    executable = out / "storage.exe"
    subprocess.run(
        command + [str(native / "r40_storage_test.c"), "-o", str(executable)], check=True
    )
    subprocess.run([str(executable)], check=True)
    calibrated = [
        "-DR40_TEMPERATURE_SENSING_ENABLED=1",
        "-DR40_TEMPERATURE_CALIBRATED=1",
        "-DR40_INTERNAL_REFERENCE_MV=1200.0f",
        "-DR40_TEMPERATURE_CAL_MV=1280.0f",
        "-DR40_TEMPERATURE_SLOPE_MV_PER_C=4.27f",
    ]
    variants = [
        ("disabled", ["-DR40_TEMPERATURE_SENSING_ENABLED=0"], 0),
        ("uncalibrated", ["-DR40_TEMPERATURE_SENSING_ENABLED=1"], 0),
        ("calibrated", calibrated, 1),
        ("invalid_calibration", calibrated[:-1] + ["-DR40_TEMPERATURE_SLOPE_MV_PER_C=0.0f"], 0),
    ]
    for name, flags, expected in variants:
        executable = out / f"temperature_{name}.exe"
        subprocess.run(
            command
            + flags
            + [
                f"-DEXPECT_CALIBRATED={expected}",
                str(native / "r40_temperature_test.c"),
                "-lm",
                "-o",
                str(executable),
            ],
            check=True,
        )
        subprocess.run([str(executable)], check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
