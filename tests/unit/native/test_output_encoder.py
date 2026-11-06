"""验证输出轴定位、Hall LUT 与 OEN3/旧 OEN1/OEN2 存储，不连接硬件。"""

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
        "    memcpy(&record.output,expected,sizeof expected);\n"
        "    OutputEncoder legacy; OutputEncoder_Init(&legacy,8);\n"
        "    LinearHall hall; LinearHall_Init(&hall);\n"
        "    assert(Param_RestoreOutputEncoder(&record,&legacy,&hall));\n"
        "    assert(!memcmp(&legacy.params,&output.params,sizeof output.params));\n"
        "    assert(hall.lut_status==0 && hall.lut_q15[0]==0);\n"
        "    direct_drive();",
    )
    vector_v2 = json.loads(
        (ROOT / "tests/fixtures/protocol/output_encoder_v2.json").read_text(encoding="utf-8")
    )
    expected_v2 = ",".join(str(v) for v in bytes.fromhex(vector_v2["record_hex"]))
    source = source.replace(
        "    direct_drive();",
        "    static const unsigned char expected_v2[]={" + expected_v2 + "};\n"
        "    memcpy(&record.output,expected_v2,sizeof expected_v2);\n"
        "    hall.calibration_success=true;\n"
        "    assert(Param_RestoreOutputEncoder(&record,&legacy,&hall));\n"
        "    assert(hall.lut_q15[0]==32 && hall.lut_q15[1016]==-16);\n"
        "    assert(hall.lut_q15[1]==28 && hall.lut_q15[1023]==26);\n"
        "    record.output.hall_lut_q15[1]=-479;\n"
        "    uint32_t narrow_crc=Crc32_Compute(&record.output,288,0);\n"
        "    memcpy((uint8_t *)&record.output+288,&narrow_crc,4);\n"
        "    assert(!Param_RestoreOutputEncoder(&record,&legacy,&hall));\n"
        "    record.output.hall_lut_q15[1]=-472;\n"
        "    narrow_crc=Crc32_Compute(&record.output,288,0);\n"
        "    memcpy((uint8_t *)&record.output+288,&narrow_crc,4);\n"
        "    assert(Param_RestoreOutputEncoder(&record,&legacy,&hall));\n"
        "    assert(LinearHall_LutIsMonotonic(hall.lut_q15));\n"
        "    direct_drive();",
    )
    vector_v3 = json.loads(
        (ROOT / "tests/fixtures/protocol/output_encoder_v3.json").read_text(encoding="utf-8")
    )
    expected_v3 = ",".join(str(v) for v in bytes.fromhex(vector_v3["record_hex"]))
    source = source.replace(
        "    direct_drive();",
        "    static const unsigned char expected_v3[]={" + expected_v3 + "};\n"
        "    memset(hall.lut_q15,0,sizeof hall.lut_q15);\n"
        "    hall.calibration_success=true; hall.lut_status=LINEAR_HALL_LUT_OK;\n"
        "    hall.lut_q15[0]=32; hall.lut_q15[1023]=-16;\n"
        "    Param_StoreOutputEncoder(&record,&output,&hall);\n"
        "    assert(sizeof expected_v3==sizeof record.output);\n"
        "    assert(!memcmp(&record.output,expected_v3,sizeof expected_v3));\n"
        "    assert(Param_RestoreOutputEncoder(&record,&legacy,&hall));\n"
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
