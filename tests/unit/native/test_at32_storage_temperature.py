"""验证AT32参数存储及温度端口；仅硬件地址访问由离线模型代替。"""

import sys as _sys
from pathlib import Path as _Path

_sys.path.insert(0, str(_Path(__file__).resolve().parents[3] / "tools"))
import argparse
import subprocess
from pathlib import Path

from _adc_fixture import adc_source
from _bsp_test_support import run_c
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
    platform = ROOT / "firmware/platform/bsp/mcu/at32f456"
    source = (platform / "flash.c").read_text(encoding="utf-8")
    for primitive in ("flash_memory", "flash_store_word"):
        source = source.replace(function_source(source, primitive), "")
    (out / "at32f456_storage_production.inc").write_text(source, encoding="utf-8")
    command = [args.cc] + (["cc"] if Path(args.cc).stem == "zig" else [])
    command += ["-DBSP_BOARD=2", "-std=c11", "-O2", "-UNDEBUG", "-Wall", "-Wextra", "-Werror"]
    for include in (
        native / "at32f456_storage_fake",
        ROOT / "firmware/platform/sdk/at32f45x/drivers/inc",
        platform,
        platform,
        out,
    ):
        command += ["-I", str(include)]
    command += NATIVE_INCLUDE_FLAGS
    executable = out / "storage.exe"
    subprocess.run(
        command + [str(native / "at32f456_storage_test.c"), "-o", str(executable)], check=True
    )
    subprocess.run([str(executable)], check=True)
    run_c(
        args,
        "motor_temperature",
        adc_source(2)
        + r"""
int main(void) {
    adc2.odt=372;
    assert(BspAdc_ReadRawChannel(BSP_ADC_MOTOR_TEMPERATURE)==372);
    assert(fabsf(BspAdc_ReadChannel(BSP_ADC_MOTOR_TEMPERATURE)-25.0f)<0.1f);
    assert(isnan(BspAdc_ReadRawChannel(BSP_ADC_MCU_TEMPERATURE)));
    assert(isnan(BspAdc_ReadRawChannel(BSP_ADC_ANALOG_SUPPLY_VOLTAGE)));
    puts("PASS motor NTC, internal ADC channels disabled");
    return 0;
}
""",
        2,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
