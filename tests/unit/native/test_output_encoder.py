"""验证输出轴绝对定位、连续位置与 OEN1 存储扩展，不连接硬件。"""

import json

from _bsp_test_support import arguments, run_c
from run_position_servo_tests import ROOT


def main() -> None:
    """在两种板型配置下执行核心算法与真实参数编解码。"""
    args = arguments(__doc__)
    source = (ROOT / "tests/unit/native/output_encoder_test.c").read_text(encoding="utf-8")
    vector = json.loads(
        (ROOT / "tests/fixtures/protocol/output_encoder_v1.json").read_text(encoding="utf-8")
    )
    expected = ",".join(str(v) for v in bytes.fromhex(vector["record_hex"]))
    source = source.replace(
        "    direct_drive();",
        "    static const unsigned char expected[]={" + expected + "};\n"
        "    MotorAxisRecord record={0}; record.motor.record_crc32=0x12345678;\n"
        "    OutputEncoder output={.params={8,.25f,1.5f,-1,0x180}};\n"
        "    Param_StoreOutputEncoder(&record,&output);\n"
        "    assert(!memcmp(&record.output,expected,sizeof expected));\n"
        "    direct_drive();",
    )
    extra = [
        ROOT / name
        for name in (
            "firmware/motor/feedback/output_encoder.c",
            "firmware/motor/feedback/linear_hall.c",
            "firmware/motor/math/control_math.c",
            "firmware/parameters/parameter_record.c",
            "firmware/parameters/parameter_values.c",
            "firmware/parameters/motor_axis_profile.c",
            "firmware/motor/protection/motor_thermal.c",
            "firmware/motor/protection/motor_thermal_config.c",
            "firmware/motor/control/speed_feedforward.c",
            "shared/checksum/crc32.c",
        )
    ]
    for board in (1, 2):
        run_c(args, f"output_encoder_{board}", source, board, extra)


if __name__ == "__main__":
    main()
