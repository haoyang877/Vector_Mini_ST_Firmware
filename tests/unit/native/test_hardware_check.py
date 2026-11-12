"""在原生平台编译 AT32 检测、独立霍尔角算法并注入 ADC/DMA 和运动事件。"""

from _bsp_test_support import ROOT, arguments, run_c


def main():
    args = arguments(__doc__)
    run_c(
        args,
        "hardware_check",
        (ROOT / "tests/unit/native/hardware_check_test.c").read_text(encoding="utf-8"),
        2,
        [
            "-DFIRMWARE_TARGET_AT32_OFF",
            "-I",
            ROOT / "projects/at32f456/application",
            ROOT / "projects/at32f456/application/hardware_check.c",
            ROOT / "firmware/motor/feedback/linear_hall.c",
            ROOT / "firmware/motor/math/control_math.c",
        ],
    )


if __name__ == "__main__":
    main()
