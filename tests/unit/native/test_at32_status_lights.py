"""用实际前台刷新函数和 GPIO 输出函数验证 AT32 状态灯。"""

from _bsp_test_support import ROOT, arguments, production, resources, run_c
from run_position_servo_tests import function_source


def main() -> None:
    args = arguments(__doc__)
    main_c = (ROOT / "projects/at32f456/application/main.c").read_text(encoding="utf-8")
    fixture = (ROOT / "tests/unit/native/at32_status_lights_test.c").read_text(encoding="utf-8")
    functions = production(2, "system", "BspSystem_WriteStatusLights", "init_status_lights")
    functions += "\n" + function_source(main_c, "update_status_lights")
    source = fixture.replace("/* BOARD_RESOURCES */", resources(2)).replace(
        "/* FUNCTIONS */", functions
    )
    run_c(args, "at32_status_lights", source, 2, ["-DFIRMWARE_TARGET_AT32_OFF"])


if __name__ == "__main__":
    main()
