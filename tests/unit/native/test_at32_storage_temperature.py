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
    (out / "r40_storage_production.inc").write_text(source, encoding="utf-8")
    command = [args.cc] + (["cc"] if Path(args.cc).stem == "zig" else [])
    command += ["-DBSP_BOARD=2", "-std=c11", "-O2", "-UNDEBUG", "-Wall", "-Wextra", "-Werror"]
    for include in (
        native / "r40_storage_fake",
        ROOT / "firmware/platform/sdk/at32f45x/drivers/inc",
        platform,
        platform,
        out,
    ):
        command += ["-I", str(include)]
    command += NATIVE_INCLUDE_FLAGS
    executable = out / "storage.exe"
    subprocess.run(
        command + [str(native / "r40_storage_test.c"), "-o", str(executable)], check=True
    )
    subprocess.run([str(executable)], check=True)
    # 未提供逐板校准时仅暴露原码，不把典型值伪装成温度或供电电压。
    run_c(
        args,
        "temperature_uncalibrated",
        adc_source(2)
        + r"""
int main(void) {
    s_temperature_dma[0]=1000; s_temperature_dma[1]=1500;
    s_temperature_dma[2]=1200; s_temperature_dma[3]=1600;
    dma_internal.dtcnt=2;
    assert(BspAdc_ReadRawChannel(BSP_ADC_MCU_TEMPERATURE)==1000);
    assert(BspAdc_ReadRawChannel(BSP_ADC_ANALOG_SUPPLY_VOLTAGE)==1500);
    dma_internal.dtcnt=4;
    assert(BspAdc_ReadRawChannel(BSP_ADC_MCU_TEMPERATURE)==1200);
    assert(BspAdc_ReadRawChannel(BSP_ADC_ANALOG_SUPPLY_VOLTAGE)==1600);
    assert(isnan(BspAdc_ReadChannel(BSP_ADC_MCU_TEMPERATURE)));
    assert(isnan(BspAdc_ReadChannel(BSP_ADC_ANALOG_SUPPLY_VOLTAGE)));
    puts("PASS AT32 uncalibrated temperature: completed DMA half and unavailable physical values");
    return 0;
}
""",
        2,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
