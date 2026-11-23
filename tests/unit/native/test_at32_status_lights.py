"""用实际监督拍灯语模块和 GPIO 输出函数验证 AT32 状态灯。"""

from _bsp_test_support import ROOT, arguments, production, resources, run_c


def main() -> None:
    args = arguments(__doc__)
    fixture = (ROOT / "tests/unit/native/at32_status_lights_test.c").read_text(encoding="utf-8")
    functions = production(
        2, "status_light_gpio", "BspSystem_WriteStatusLights", "McuStatusLights_Init"
    )
    source = fixture.replace("/* BOARD_RESOURCES */", resources(2)).replace(
        "/* FUNCTIONS */", functions
    )
    run_c(
        args,
        "at32_status_lights",
        source,
        2,
        [
            "-DFIRMWARE_TARGET_AT32_OFF",
            "-I",
            ROOT / "projects/at32f456/application",
            ROOT / "projects/at32f456/application/status_lights.c",
        ],
    )


if __name__ == "__main__":
    main()
