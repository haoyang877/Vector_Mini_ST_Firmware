"""用RL回路验证真实相电阻算法，无设备访问。"""

from _bsp_test_support import ROOT, arguments, run_c


def main():
    """检查双档双向测量、相别诊断和即时中止。"""
    args = arguments(__doc__)
    run_c(
        args,
        "phase_resistance",
        (ROOT / "tests/unit/native/phase_resistance_test.c").read_text(encoding="utf-8"),
        2,
        [ROOT / "firmware/motor/calibration/phase_resistance.c"],
    )


if __name__ == "__main__":
    main()
